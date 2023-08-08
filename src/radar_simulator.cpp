#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <rmagine/simulation/OnDnSimulatorEmbree.hpp>
#include <rmagine/simulation/OnDnSimulatorOptix.hpp>
#include <rmagine/util/prints.h>
#include <rmagine/util/StopWatch.hpp>
#include <random>

#include <tf2_ros/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <radarays_ros/RadarModelConfig.h>

#include <sensor_msgs/PointCloud.h>

#include <radarays_ros/radar_types.h>
#include <radarays_ros/radar_algorithms.h>
#include <radarays_ros/radar_math.h>


#include <radarays_ros/RadarParams.h>

#include <actionlib/server/simple_action_server.h>
#include <radarays_ros/GenRadarImageAction.h>


#include <radarays_ros/GetRadarParams.h>
#include <radarays_ros/image_algorithms.h>


#include <opencv2/highgui.hpp>

#include <omp.h>

#if defined WITH_CUDA
#include <opencv2/core/cuda.hpp>
#endif // defined WITH_CUDA


using namespace radarays_ros;

namespace rm = rmagine;

struct Signal
{
    double time;
    double strength;
};

// parameters
static RadarModel default_params_model()
{
    RadarModel model;
    model.beam_width = 8.0 * M_PI / 180.0;
    model.n_samples = 200;
    model.n_reflections = 2;
    return model;
}

static RadarParams default_params()
{
    RadarParams ret;
    ret.model = default_params_model();
    return ret;
}


std::string map_frame = "map";
std::string sensor_frame = "navtech";


cv::Mat polar_image;

// bool image_resize_required = false;
// size_t n_cells_new;

std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;
ros::Publisher pub_pcl;

rm::EmbreeMapPtr map;
// thread -> sim
std::unordered_map<unsigned int, rm::OnDnSimulatorEmbreePtr> sims;
rm::SphericalModel radar_model;



// STATE
rm::Transform Tsm_last = rm::Transform::Identity();
bool has_last;
ros::Time Tsm_stamp_last;
ros::Time stamp_last;

RadarParams params = default_params();
radarays_ros::RadarModelConfig cfg;

std::shared_ptr<ros::NodeHandle> nh_p;

// materials
int material_id_air = 0;
std::vector<int> object_materials;


std::vector<DirectedWave> waves_start;
bool resample = true;

// TODO: precompute this:
cv::Mat perlin_noise_buffer;

#if defined WITH_CUDA
cv::cuda::GpuMat perlin_noise_buffer_cuda;
#endif // defined WITH_CUDA



// intersection attributes
using ResT = rm::Bundle<
    rm::Hits<rm::RAM>,
    rm::Ranges<rm::RAM>,
    rm::Normals<rm::RAM>,
    rm::ObjectIds<rm::RAM> // connection to material
>;



template<typename T>
T loadFromRPC(XmlRpc::XmlRpcValue);

template<>
RadarMaterial loadFromRPC<RadarMaterial>(XmlRpc::XmlRpcValue material_xml)
{
    RadarMaterial ret;
    if(material_xml.hasMember("velocity"))
    {
        ret.velocity = (double)material_xml["velocity"];
    } else {
        ret.velocity = 0.0;
    }

    if(material_xml.hasMember("ambient"))
    {
        ret.ambient = (double)material_xml["ambient"];
    } else {
        ret.ambient = 0.0;
    }

    if(material_xml.hasMember("diffuse"))
    {
        ret.diffuse = (double)material_xml["diffuse"];
    } else {
        ret.diffuse = 0.0;
    }

    if(material_xml.hasMember("specular"))
    {
        ret.specular = (double)material_xml["specular"];
    } else {
        ret.specular = 0.0;
    }

    return ret;
}

RadarMaterials loadRadarMaterialsFromParameterServer(
    std::shared_ptr<ros::NodeHandle> nh)
{
    RadarMaterials ret;
    
    XmlRpc::XmlRpcValue materials_xml;
    nh->getParam("materials", materials_xml);

    if(!materials_xml.valid())
    {
        std::cout << "Loaded XmlRpcValue is invalid" << std::endl;
    }

    if(materials_xml.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        size_t n_materials = materials_xml.size();

        for(size_t i=0; i<n_materials; i++)
        {
            auto material_xml = materials_xml[i];
            auto material = loadFromRPC<RadarMaterial>(material_xml);
            // std::cout << i << ": " << material << std::endl;
            ret.data.push_back(material);
        }
    }

    return ret;
}

void loadParameters()
{
    // material properties
    params.materials = loadRadarMaterialsFromParameterServer(nh_p);
    nh_p->getParam("object_materials", object_materials);
    nh_p->getParam("material_id_air", material_id_air);
}

/**
 * @brief Update the simulation parameters
 * Right now, many parameters are for testing and are removed in the future
 * 
 * @param config 
 * @param level 
 */
void modelCB(
    radarays_ros::RadarModelConfig &config,
    uint32_t level) 
{   
    ROS_INFO("Changing Model");
    
    // z_offset
    // auto T = rm::Transform::Identity();
    // T.t.z = config.z_offset;
    // sim->setTsb(T);

    if(   config.beam_sample_dist != cfg.beam_sample_dist 
        || abs(config.beam_width - cfg.beam_width ) > 0.001
        || config.n_samples != cfg.n_samples
        || abs(config.beam_sample_dist_normal_p_in_cone - cfg.beam_sample_dist_normal_p_in_cone) > 0.001
    )
    {
        resample = true;
    }

    // update radar model
    radar_model.range.min = config.range_min;
    radar_model.range.max = config.range_max;

    // update params model
    params.model.beam_width = config.beam_width * M_PI / 180.0;
    params.model.n_samples = config.n_samples;
    params.model.n_reflections = config.n_reflections;

    cfg = config;
}

std::optional<rm::Transform> getTsm()
{
    rm::Transform Tsm;
    // Tsm: T[v[0.863185,-1.164,1.49406], E[0.0149786, 0.00858233, 3.04591]]

    // Tsm.R = rm::EulerAngles{0.0149786, 0.00858233, 3.04591};
    // Tsm.t = rm::Vector3{0.863185,-1.164,1.49406};

    try {
        geometry_msgs::TransformStamped Tsm_ros = tf_buffer->lookupTransform(
            map_frame,
            sensor_frame,
            ros::Time(0)
        );

        Tsm.t.x = Tsm_ros.transform.translation.x;
        Tsm.t.y = Tsm_ros.transform.translation.y;
        Tsm.t.z = Tsm_ros.transform.translation.z;
        Tsm.R.x = Tsm_ros.transform.rotation.x;
        Tsm.R.y = Tsm_ros.transform.rotation.y;
        Tsm.R.z = Tsm_ros.transform.rotation.z;
        Tsm.R.w = Tsm_ros.transform.rotation.w;

    } catch(tf2::TransformException ex) {
        ROS_WARN_STREAM("TF-Error: " << ex.what());
        return {};
    }

    return Tsm;
}

// updating global vars:
// - Tsm_last = Tsm;
// - Tsm_stamp_last = Tsm_stamp;
// - stamp_last = ros::Time::now();
// - has_last = true;
bool updateTsm()
{
    std::optional<rm::Transform> Tsm_opt;
    ros::Time Tsm_stamp;

    try {
        rm::Transform Tsm;
        geometry_msgs::TransformStamped Tsm_ros = tf_buffer->lookupTransform(
            map_frame,
            sensor_frame,
            ros::Time(0)
        );

        Tsm.t.x = Tsm_ros.transform.translation.x;
        Tsm.t.y = Tsm_ros.transform.translation.y;
        Tsm.t.z = Tsm_ros.transform.translation.z;
        Tsm.R.x = Tsm_ros.transform.rotation.x;
        Tsm.R.y = Tsm_ros.transform.rotation.y;
        Tsm.R.z = Tsm_ros.transform.rotation.z;
        Tsm.R.w = Tsm_ros.transform.rotation.w;

        Tsm_opt = Tsm;
        Tsm_stamp = Tsm_ros.header.stamp;
    } catch(tf2::TransformException ex) {
        ROS_WARN_STREAM("TF-Error: " << ex.what());
    }
    
    if(!Tsm_opt && !has_last)
    {
        // cannot simulate from old because nothing exists yet
        std::cout << "No current, no old transform available. Skipping..." << std::endl;
        return false;
    }

    rm::Transform Tsm;
    if(Tsm_opt)
    {
        Tsm = *Tsm_opt;
    } else {
        Tsm = Tsm_last;
        // extrapolate time
        Tsm_stamp = Tsm_stamp_last + (ros::Time::now() - stamp_last);
    }

    {
        // updating global stuff
        Tsm_last = Tsm;
        Tsm_stamp_last = Tsm_stamp;
        stamp_last = ros::Time::now();
        has_last = true;
    }

    return true;
}


void simulateImage2()
{
    float wave_energy_threshold = 0.001;

    if(polar_image.rows != cfg.n_cells)
    {
        // polar_image = cv::Mat_<unsigned char>(n_cells, radar_model.theta.size);
        std::cout << "Resize canvas" << std::endl;

        polar_image.resize(cfg.n_cells);
        perlin_noise_buffer.resize(cfg.n_cells);

        #if defined WITH_CUDA
        perlin_noise_buffer_cuda = cv::cuda::GpuMat(perlin_noise_buffer.size(), perlin_noise_buffer.type());
        #endif // defined WITH_CUDA
        
        std::cout << "Resizing canvas - done." << std::endl;
    }

    // std::cout << "Fill canvas: " << polar_image.cols << "x" << polar_image.rows << std::endl;
    polar_image.setTo(cv::Scalar(0));

    std::vector<float> denoising_weights;
    int denoising_mode = 0;

    if(cfg.signal_denoising > 0)
    {
        // std::cout << "Signal Denoising: ";
        if(cfg.signal_denoising == 1)
        {
            // std::cout << "Triangular";
            denoising_mode = cfg.signal_denoising_triangular_mode * cfg.signal_denoising_triangular_width;
            denoising_weights = make_denoiser_triangular(
                cfg.signal_denoising_triangular_width,
                denoising_mode
            );
            
        } else if(cfg.signal_denoising == 2) {
            // std::cout << "Gaussian";
            denoising_mode = cfg.signal_denoising_gaussian_mode * cfg.signal_denoising_gaussian_width;
            denoising_weights = make_denoiser_gaussian(
                cfg.signal_denoising_gaussian_width,
                denoising_mode
            );

        } else if(cfg.signal_denoising == 3) {
            // std::cout << "Maxwell Boltzmann";
            denoising_mode = cfg.signal_denoising_mb_mode * cfg.signal_denoising_mb_width;
            denoising_weights = make_denoiser_maxwell_boltzmann(
                cfg.signal_denoising_mb_width,
                denoising_mode
            );
        }
        // std::cout << std::endl;

        // scale so that mode has weight 1
        // if(false)
        if(denoising_weights.size() > 0)
        {
            double denoising_mode_val = denoising_weights[denoising_mode];

            for(size_t i=0; i<denoising_weights.size(); i++)
            {
                denoising_weights[i] /= denoising_mode_val;
            }
        }
        
    }

    // std::cout << "Denoising weights (mode:" << denoising_mode << "): ";
    // for(size_t i=0; i<denoising_weights.size(); i++)
    // {
    //     std::cout << denoising_weights[i] << ", ";
    // }
    // std::cout << std::endl;

    // gen ambient noise
    bool static_tf = true;

    // rm::StopWatch sw;
    // double el;

    DirectedWave wave;
    wave.energy       =  1.0;
    wave.polarization =  0.5;
    wave.frequency    = 76.5; // GHz
    wave.velocity     =  0.3; // m / ns - speed in air
    wave.material_id  =  0;   // air
    wave.time         =  0.0; // ns
    wave.ray.orig = {0.0, 0.0, 0.0};
    wave.ray.dir = {1.0, 0.0, 0.0};

    // el = sw();

    int n_cells = polar_image.rows;
    int n_angles = polar_image.cols;

    // std::random_device                      rand_dev;
    // std::mt19937                            gen(rand_dev());
    // std::uniform_real_distribution<float>   dist_uni(0.0, 1.0);


    // without motion: update Tsm only once
    if(!cfg.include_motion)
    {
        if(!updateTsm())
        {
            std::cout << "Couldn't get Transform between sensor and map. Skipping..." << std::endl;
            return;
        }
    }

    if(resample)
    {
        waves_start = sample_cone_local(
            wave,
            params.model.beam_width,
            params.model.n_samples,
            cfg.beam_sample_dist,
            cfg.beam_sample_dist_normal_p_in_cone);
        resample = false;
    }

    rm::StopWatch sw_radar_sim;
    sw_radar_sim();

    bool enable_omp = false;
    
    // std::cout << "Threads: " << OMP_NUM_THREADS << std::endl;


    #pragma omp parallel for if(!cfg.include_motion)
    for(size_t angle_id = 0; angle_id < n_angles; angle_id++)
    {
        // #if defined MEASURE_TIME_INNER
        // rm::StopWatchHR sw_inner;
        // sw_inner();
        // #endif // MEASURE_TIME_INNER

        int tid = 0;
        if(!cfg.include_motion)
        {
            // threaded. get sim that belong to current thread
            tid = omp_get_thread_num();
        }

        auto sims_it = sims.find(tid);
        if(sims.find(tid) == sims.end())
        {
            sims[tid] = std::make_shared<rm::OnDnSimulatorEmbree>(map);
            sims_it = sims.find(tid);
            auto Tsb = rm::Transform::Identity();
            sims_it->second->setTsb(Tsb);

            #pragma omp critical
            std::cout << "Created new simulator for thread " << tid << std::endl; 
        }
        auto sim = sims_it->second;

        if(!sim)
        {
            std::cout << "ERROR!! Sim shared ptr empty" << std::endl;
        }

        
        std::vector<DirectedWave> waves = waves_start;

        rm::OnDnModel model = make_model(waves);
        sim->setModel(model);

        // with motion: update at each angle
        if(cfg.include_motion)
        {
            if(!updateTsm())
            {
                continue;
            }
        }
        
        // make Tam ? angle to map Tam = Tsm * Tas
        // Tas is the transformation of angle to sensor
        
        rm::Transform Tas;
        Tas.R = rm::EulerAngles{0.0, 0.0, radar_model.getTheta(angle_id)};
        Tas.t = radar_model.getOrigin(0, angle_id);

        rm::Transform Tsm = Tsm_last;
        rm::Transform Tam = Tsm * Tas;
        
        rm::Memory<rm::Transform> Tams(1);
        Tams[0] = Tam;

        // double el1 = sw();
        // std::cout << "el1: " << el1 << std::endl;
        // std::cout << "Create Signal" << std::endl;
        
        // double el1 = sw_inner();

        std::vector<Signal> signals;
        ///////
        /// 1. Signal generation
        for(size_t pass_id = 0; pass_id < params.model.n_reflections; pass_id++)
        {
            ResT results;
            results.hits.resize(model.size());
            results.ranges.resize(model.size());
            results.normals.resize(model.size());
            results.object_ids.resize(model.size());
            sim->simulate(Tams, results);
            
            // reflect / refract / absorb / return
            std::vector<DirectedWave> waves_new;

            // Move rays
            // #pragma omp parallel for
            for(size_t i=0; i < waves.size(); i++)
            {
                // get
                DirectedWave wave = waves[i];
                const float wave_range = results.ranges[i];
                const rmagine::Vector surface_normal = results.normals[i].normalize();
                const unsigned int obj_id = results.object_ids[i];


                if(obj_id > 10000)
                {
                    continue;
                }
                
                // do
                const DirectedWave incidence = wave.move(wave_range);

                // inititalize
                DirectedWave reflection = incidence;
                DirectedWave refraction = incidence;

                // if wave was in air, switch to new material
                // else if wave was in material, switch to air (is this right ?)
                if(incidence.material_id == material_id_air)
                {
                    refraction.material_id = object_materials[obj_id];
                } else {
                    refraction.material_id = material_id_air;
                }

                float v_refraction = 1.0;

                if(incidence.material_id != refraction.material_id)
                {
                    v_refraction = params.materials.data[refraction.material_id].velocity;
                } else {
                    v_refraction = incidence.velocity;
                }

                // Build surface patch
                auto res = fresnel(surface_normal, incidence, v_refraction);

                reflection.ray.dir = res.first.ray.dir;
                reflection.energy = res.first.energy;

                if(reflection.energy > wave_energy_threshold)
                {
                    waves_new.push_back(reflection);
                    
                    // split total reflection energy into 
                    // - scene reflection energy
                    // - path return energy
                    // - air return energy
                    //
                    // path return energy = air return energy 
                    //      for pass 0
                    // path return energy = air return energy = scene reflection energy
                    //      for pass 0 and perpendicular hit
                    
                    if(reflection.material_id == material_id_air)
                    {
                        // 1. signal travelling back along the pass

                        auto material = params.materials.data[refraction.material_id];

                        double incidence_angle = get_incidence_angle(surface_normal, incidence);
                        // 1. signal traveling over path
                        double return_energy_path = back_reflection_shader(
                            incidence_angle,
                            reflection.energy,
                            material.ambient, // ambient
                            material.diffuse, // diffuse
                            material.specular // specular
                        );

                        // signal traveling the air path == traversal path
                        if(pass_id == 0 || cfg.record_multi_reflection)
                        {   
                            float time_back = incidence.time * 2.0;
                            signals.push_back({time_back, return_energy_path});
                        }

                        if(pass_id > 0 && cfg.record_multi_path)
                        {
                            // 2. signal traveling the air path
                            Signal signal_air;
                            // signal_path.time = incidence.time * 2.0;
                            // signal_path.strength = return_energy_path;
                            
                            rm::Vector dir_sensor_to_hit = reflection.ray.orig;
                            const double distance_between_sensor_and_hit = dir_sensor_to_hit.l2norm();
                            dir_sensor_to_hit.normalizeInplace();
                            // time_to_sensor in [ns] assuming transmission through space
                            const double time_to_sensor = distance_between_sensor_and_hit / reflection.velocity;


                            double sensor_view_scalar = wave.ray.dir.dot(dir_sensor_to_hit);

                            double angle_between_reflection_and_sensor_dir 
                                = angle_between(-reflection.ray.dir, dir_sensor_to_hit);

                            if(cfg.record_multi_path 
                                && sensor_view_scalar > cfg.multipath_threshold)
                            {
                                double return_energy_air = back_reflection_shader(
                                    angle_between_reflection_and_sensor_dir,
                                    reflection.energy,
                                    material.ambient, // ambient
                                    material.diffuse, // diffuse
                                    material.specular // specular
                                );

                                signal_air.time = incidence.time + time_to_sensor;
                                signal_air.strength = return_energy_air;
                                
                                signals.push_back(signal_air);
                            }
                        }
                    }
                }

                refraction.ray.dir = res.second.ray.dir;
                refraction.energy = res.second.energy;

                if(refraction.energy > wave_energy_threshold)
                {
                    waves_new.push_back(refraction);
                }
            }

            // skip certain distance for safety
            float skip_dist = 0.001;
            for(size_t i=0; i<waves_new.size(); i++)
            {
                waves_new[i].moveInplace(skip_dist);
            }

            waves = waves_new;

            // update sensor model
            if(pass_id < params.model.n_reflections - 1)
            {
                model = make_model(waves);
                sim->setModel(model);
            } else {
                // last run. dont need to update
            }
            
            // std::cout << "Angle " << angle_id << " - pass " << pass_id << " - done." << std::endl;
        }


        // double el2 = sw_inner();

        //////////////////
        /// 2. Signals -> Canvas
        /// 2.1. Signal Noise -> Slice
        /// 2.2. Ambient noise -> Slice
        /// 2.3. Slice -> Canvas
        cv::Mat_<float> slice(polar_image.rows, 1, 0.0);

        float max_val = 0.0;
        // Draw signals to slice
        for(size_t i=0; i<signals.size(); i++)
        {
            auto signal = signals[i];
            // wave speed in air (light speed) * t / 2
            float half_time = signal.time / 2.0;
            float signal_dist = 0.3 * half_time;

            int cell = static_cast<int>(signal_dist / cfg.resolution);
            if(cell < slice.rows)
            {
                float signal_old = slice.at<float>(cell, 0);

                if(cfg.signal_denoising > 0)
                {
                    // signal denoising
                    for(int vid = 0; vid < denoising_weights.size(); vid++)
                    {
                        int glob_id = vid + cell - denoising_mode;
                        if(glob_id > 0 && glob_id < slice.rows)
                        {
                            slice.at<float>(glob_id, 0) += signal.strength * denoising_weights[vid];

                            if(slice.at<float>(glob_id, 0) > max_val)
                            {
                                max_val = slice.at<float>(glob_id, 0);
                            }
                        }
                    }
                } else {
                    // no signal denoising

                    // slice.at<float>(cell, 0) += 1.0;
                    // slice.at<float>(cell, 0) += signal.strength;
                    slice.at<float>(cell, 0) = std::max(
                        slice.at<float>(cell, 0),
                        (float)signal.strength
                    );

                    if(slice.at<float>(cell, 0) > max_val)
                    {
                        max_val = slice.at<float>(cell, 0);
                    }
                }
            }
        }

        // normalize
        slice *= cfg.energy_max;

        // double el3 = sw_inner();

        int col = (cfg.scroll_image + angle_id) % polar_image.cols;

        if(cfg.ambient_noise)
        {
            std::random_device                      rand_dev;
            std::mt19937                            gen(rand_dev());
            std::uniform_real_distribution<float>   dist_uni(0.0, 1.0);

            
            // apply noise
            // low freq perlin
            double scale = 0.05;
            // high freq perlin
            double scale2 = 0.2;

            double random_begin = dist_uni(gen) * 1000.0;
            
            for(size_t i=0; i<slice.rows; i++)
            {
                float signal = slice.at<float>(i) * cfg.energy_max;

                double p;

                if(cfg.ambient_noise == 1) // UNIFORM
                {
                    p = dist_uni(gen);
                } else if(cfg.ambient_noise == 2) // PERLIN
                {
                    double p_perlin1 = perlin_noise(
                        random_begin + static_cast<double>(i) * scale, 
                        static_cast<double>(col) * scale);
                    
                    double p_perlin2 = perlin_noise(
                        random_begin + static_cast<double>(i) * scale2, 
                        static_cast<double>(col) * scale2);

                    p = 0.9 * p_perlin1 + 0.1 * p_perlin2;
                }
                
                // p = p * 
                // p = (p + 1.0) / 2.0; // [0.0-1.0]

                // verwurschteltn
                float signal_min = 0;
                float signal_max = max_val;
                float signal_amp = signal_max - signal_min;

                float signal_ = 1.0 - ((signal - signal_min) / signal_amp);

                float noise_at_0 = signal_amp * cfg.ambient_noise_at_signal_0;
                float noise_at_1 = signal_amp * cfg.ambient_noise_at_signal_1;

                float signal__ = std::pow(signal_, 4.0);

                float noise_amp = (signal__ * noise_at_0 + (1.0 - signal__) * noise_at_1);

                // noise_amp * p * signal_max;
                
                float noise_energy_max = signal_max * cfg.ambient_noise_energy_max;
                float noise_energy_min = signal_max * cfg.ambient_noise_energy_min;
                float energy_loss = cfg.ambient_noise_energy_loss;

                float y_noise = noise_amp * p;

                float x = (static_cast<float>(i) + 0.5) * cfg.resolution;

                y_noise = y_noise + (noise_energy_max - noise_energy_min) * exp(-energy_loss * x) + noise_energy_min;
                y_noise = abs(y_noise);

                slice.at<float>(i) = signal + y_noise;
            }
        }
        
        // double el4 = sw_inner();

        float max_signal = 120.0;
        slice *= max_signal / max_val;

        slice.convertTo(polar_image.col(col), CV_8UC1);

        // double el5 = sw_inner();

        // double tot = el1 + el2 + el3 + el4 + el5;

        // std::cout << std::fixed << std::setprecision(8) 
        //     << el1/tot << ", " << el2/tot << ", " << el3/tot << ", " << el4/tot << ", " << el5/tot << std::endl;

        // #pragma omp critical
        // {
        //     std::cout << "Runtime TID " << tid << ": " << el1 + el2 + el3 + el4 + el5 << "s" << std::endl; 
        //     std::cout << "- prepare: " << el1 * 1000.0 << "ms " << std::endl;
        //     std::cout << "- signals: " << el2 * 1000.0 << "ms" << std::endl;
        //     std::cout << "- noise (system): " << el3 * 1000.0 << "ms" << std::endl;
        //     std::cout << "- noise (ambient): " << el4 * 1000.0 << "ms" << std::endl;
        //     std::cout << "- postprocess: " << el5 * 1000.0 << "ms" << std::endl;
        // }
        

        if(cfg.include_motion)
        {
            ros::spinOnce();
        }
    }

    double el_radar_sim = sw_radar_sim();

    // std::cout << "SIM in " << el_radar_sim << "s" << std::endl;
    std::cout << std::fixed << std::setprecision(8) << el_radar_sim << std::endl;
}

std::shared_ptr<actionlib::SimpleActionServer<radarays_ros::GenRadarImageAction> > as_;
radarays_ros::GenRadarImageFeedback feedback_;
radarays_ros::GenRadarImageResult result_;

void executeCB(const radarays_ros::GenRadarImageGoalConstPtr &goal)
{
    std::cout << "CALL ACTION" << std::endl;

    params = goal->params;
    
    simulateImage2();

    std::cout << polar_image.size() << std::endl;

    result_.polar_image = 
        *cv_bridge::CvImage(
            std_msgs::Header(), 
            "mono8",
            polar_image).toImageMsg();

    result_.polar_image.header.stamp = ros::Time::now();
    as_->setSucceeded(result_);
}

bool getRadarParamsCB(radarays_ros::GetRadarParams::Request  &req,
         radarays_ros::GetRadarParams::Response &res)
{
//   res.sum = req.a + req.b;
//   ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
//   ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    res.params = params;
    return true;
}

int main_action_server(int argc, char** argv)
{
    std::cout << "STARTING RADAR SIMULATOR - ACTION SERVER" << std::endl;
    ros::init(argc, argv, "radar_simulator");

    ros::NodeHandle nh;
    nh_p = std::make_shared<ros::NodeHandle>("~");

    std::string map_file;
    nh_p->getParam("map_file", map_file);
    nh_p->getParam("map_frame", map_frame);
    nh_p->getParam("sensor_frame", sensor_frame);

    loadParameters();

    std::cout << "SPEED IN AIR: " << params.materials.data[material_id_air].velocity << std::endl;

    map = rm::import_embree_map(map_file);

    // offset of sensor center to frame
    auto Tsb = rm::Transform::Identity();
    Tsb.t.z = 0.0;

    // sim->setTsb(Tsb);

    // n_cells = 3424;
    // n_cells = 1712;
    radar_model.theta.inc = -(2 * M_PI) / 400;
    radar_model.theta.min = 0.0;
    radar_model.theta.size = 400;
    radar_model.phi.inc = 1.0;
    radar_model.phi.min = 0.0;
    radar_model.phi.size = 1;

    // setting up dynamic reconfigure
    dynamic_reconfigure::Server<RadarModelConfig> server;
    dynamic_reconfigure::Server<RadarModelConfig>::CallbackType f;
    f = boost::bind(&modelCB, _1, _2);
    server.setCallback(f);

    // setting up tf
    // tf_buffer = std::make_shared<tf2_ros::Buffer>();
    // tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    polar_image = cv::Mat_<unsigned char>(0, radar_model.theta.size);

    // Start service server
    ros::ServiceServer service = nh_p->advertiseService("get_radar_params", getRadarParamsCB);
    
    // Start action server
    std::string action_name = "gen_radar_image";
    as_ = std::make_shared<actionlib::SimpleActionServer<radarays_ros::GenRadarImageAction> >(
        *nh_p, action_name, executeCB, false
    );

    as_->start();

    ros::Rate r(100);
    ros::Time tp = ros::Time::now();

    while(ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        auto tc = ros::Time::now();
        if(tc < tp)
        {
            // jump back in time
            std::cout << "Jump back in time detected" << std::endl;
            ros::Duration(2.0).sleep();
            as_->shutdown();
            as_ = std::make_shared<actionlib::SimpleActionServer<radarays_ros::GenRadarImageAction> >(
                *nh_p, action_name, executeCB, false
            );
            as_->start();
            ros::Duration(2.0).sleep();
        }

        tp = tc;
    }
    

    return 0;
}


int main_publisher(int argc, char** argv)
{
    ros::init(argc, argv, "radar_simulator");

    ros::NodeHandle nh;
    nh_p = std::make_shared<ros::NodeHandle>("~");

    std::string map_file;
    nh_p->getParam("map_file", map_file);
    nh_p->getParam("map_frame", map_frame);
    nh_p->getParam("sensor_frame", sensor_frame);

    loadParameters();

    // // object to material
    // nh_p.getParam("object_materials", object_materials);

    std::cout << "SPEED IN AIR: " << params.materials.data[material_id_air].velocity << std::endl;

    map = rm::import_embree_map(map_file);

    // offset of sensor center to frame
    // auto Tsb = rm::Transform::Identity();
    // Tsb.t.z = 0.0;

    // sim->setTsb(Tsb);

    // n_cells = 3424;
    // n_cells = 1712;
    radar_model.theta.inc = -(2 * M_PI) / 400;
    radar_model.theta.min = 0.0;
    radar_model.theta.size = 400;
    radar_model.phi.inc = 1.0;
    radar_model.phi.min = 0.0;
    radar_model.phi.size = 1;

    // setting up dynamic reconfigure
    dynamic_reconfigure::Server<RadarModelConfig> server;
    dynamic_reconfigure::Server<RadarModelConfig>::CallbackType f;
    f = boost::bind(&modelCB, _1, _2);
    server.setCallback(f);

    // setting up tf
    tf_buffer = std::make_shared<tf2_ros::Buffer>();
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // image transport
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("radar/image", 1);

    // pcl
    pub_pcl = nh_p->advertise<sensor_msgs::PointCloud>("radar/pcl_real", 1);


    // TODO: 
    // - set RADAR model (radar_model)
    // - set image size
    // - load and set map
    polar_image = cv::Mat_<unsigned char>(0, radar_model.theta.size);


    // polar_image = cv::Mat_<unsigned char>(n_cells, radar_model.theta.size);
    // perlin_noise_buffer = cv::Mat_<float>(n_cells, radar_model.theta.size);

    #if defined WITH_CUDA
    perlin_noise_buffer_cuda = cv::cuda::GpuMat(perlin_noise_buffer.size(), perlin_noise_buffer.type());
    #endif // defined WITH_CUDA

    ros::Rate r(100);

    while(nh.ok())
    {
        loadParameters();
        // ROS_INFO("Simulate!");
        simulateImage2();

        sensor_msgs::ImagePtr msg = 
            cv_bridge::CvImage(
                std_msgs::Header(), 
                "mono8",
                polar_image).toImageMsg();
        
        msg->header.stamp = Tsm_stamp_last;
        pub.publish(msg);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}

int main(int argc, char** argv)
{
    // return main_action_server(argc, argv);
    return main_publisher(argc, argv);
}
