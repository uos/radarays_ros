#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <rmagine/simulation/OnDnSimulatorEmbree.hpp>
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

#if defined WITH_CUDA
#include <opencv2/core/cuda.hpp>
#endif // defined WITH_CUDA


using namespace radarays_ros;

namespace rm = rmagine;

std::string map_frame = "map";
std::string sensor_frame = "navtech";


cv::Mat polar_image;

// bool image_resize_required = false;
// size_t n_cells_new;

std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;
ros::Publisher pub_pcl;

rm::EmbreeMapPtr map;
rm::OnDnSimulatorEmbreePtr sim;
rm::SphericalModel radar_model;

// intersection attributes
using ResT = rm::Bundle<
    rm::Hits<rm::RAM>,
    rm::Ranges<rm::RAM>,
    rm::Normals<rm::RAM>,
    rm::ObjectIds<rm::RAM> // connection to material
>;

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

rm::Transform Tsm_last = rm::Transform::Identity();
bool has_last;
ros::Time Tsm_stamp_last;
ros::Time stamp_last;

RadarParams params = default_params();

// float beam_width = 8.0 * M_PI / 180.0; // width of the beam in radian
float resolution = 0.1; // resolution of a range measurement
size_t n_cells = 300; // number of range measurements per beam
// size_t n_samples = 100; // number samples per beam
// size_t n_reflections = 2; // maximum number of reflections per beam

float energy_max = 1.0;
float energy_min = 0.0;

float multipath_threshold = 0.5;

int scroll_image;

// Sample distribution of uniform
// 0: uniform
// 1: wrong uniform. limitted normal?
// 2: correct normal?
// 3: wrong normal?
int beam_sample_dist = 0;

// never set this to 1.0
float beam_sample_dist_normal_p_in_cone = 0.95;

// Denoising of Beam sampling
// 0: None
// 1: Triangular
// 2: Gaussian
// 3: Maxwell Boltzmann
int signal_denoising = 0;

int signal_denoising_triangular_width;
int signal_denoising_triangular_mode;

int signal_denoising_gaussian_width;
int signal_denoising_gaussian_mode;

int signal_denoising_mb_width;
int signal_denoising_mb_mode;


// Ambient Noise
// bool ambient_noise = false;
int ambient_noise = 0;

float ambient_noise_at_signal_0 = 0.3;
float ambient_noise_at_signal_1 = 0.03;
float ambient_noise_energy_max = 0.5;
float ambient_noise_energy_min = 0.1;
float ambient_noise_energy_loss = 0.05;

float ambient_noise_uniform_max = 0.8;


// Particle Noise
bool particle_noise = false;
float particle_noise_exp_mu = 1.0;

bool record_multi_reflection = true;
bool record_multi_path = true;

bool include_motion = true;

std::shared_ptr<ros::NodeHandle> nh_p;

// materials
int material_id_air = 0;
std::vector<int> object_materials;


cv::Mat perlin_noise_buffer;

#if defined WITH_CUDA
cv::cuda::GpuMat perlin_noise_buffer_cuda;
#endif // defined WITH_CUDA

struct Signal
{
    double time;
    double strength;
};

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
    auto T = rm::Transform::Identity();
    T.t.z = config.z_offset;
    sim->setTsb(T);
    
    radar_model.range.min = config.range_min;
    radar_model.range.max = config.range_max;
    
    params.model.beam_width = config.beam_width * M_PI / 180.0;

    resolution = config.resolution;

    n_cells_new = config.n_cells;
    // if(n_cells != config.n_cells)
    // {

    //     std::cout << "BLAAAAA" << std::endl;
    //     // Image update needed
    //     image_resize_required = true;
    //     n_cells = config.n_cells;
    // } else {
    //     image_resize_required = false;
    // }

    params.model.n_samples = config.n_samples;
    
    if(beam_sample_dist != config.beam_sample_dist)
    {
        std::cout << "!!!!! changing beam sample distribution to " << config.beam_sample_dist << std::endl;
    }
    beam_sample_dist = config.beam_sample_dist;
    beam_sample_dist_normal_p_in_cone = config.beam_sample_dist_normal_p_in_cone;

    signal_denoising = config.signal_denoising;

    signal_denoising_triangular_width = config.signal_denoising_triangular_width;
    signal_denoising_triangular_mode = signal_denoising_triangular_width * config.signal_denoising_triangular_mode;
    
    signal_denoising_gaussian_width = config.signal_denoising_gaussian_width;
    signal_denoising_gaussian_mode = signal_denoising_gaussian_width * config.signal_denoising_gaussian_mode;

    signal_denoising_mb_width = config.signal_denoising_mb_width;
    signal_denoising_mb_mode = signal_denoising_mb_width * config.signal_denoising_mb_mode;



    params.model.n_reflections = config.n_reflections;

    energy_min = config.energy_min;
    energy_max = config.energy_max;

    ambient_noise = config.ambient_noise;
    ambient_noise_at_signal_0 = config.ambient_noise_at_signal_0;
    ambient_noise_at_signal_1 = config.ambient_noise_at_signal_1;
    ambient_noise_uniform_max = config.ambient_noise_uniform_max;
    ambient_noise_energy_max = config.ambient_noise_energy_max;
    ambient_noise_energy_min = config.ambient_noise_energy_min;
    ambient_noise_energy_loss = config.ambient_noise_energy_loss;

    particle_noise = config.particle_noise;
    particle_noise_exp_mu = config.particle_noise_exp_mu;

    scroll_image = config.scroll_image;

    multipath_threshold = config.multipath_threshold;
    record_multi_reflection = config.record_multi_reflection;
    record_multi_path = config.record_multi_path;

    include_motion = config.include_motion;
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

    std::cout << "Reset Buffers" << std::endl;

    if(image_resize_required)
    {
        // polar_image = cv::Mat_<unsigned char>(n_cells, radar_model.theta.size);
        std::cout << "Resize Canvas" << std::endl;

        polar_image.resize(n_cells);
        perlin_noise_buffer.resize(n_cells);

        #if defined WITH_CUDA
        perlin_noise_buffer_cuda = cv::cuda::GpuMat(perlin_noise_buffer.size(), perlin_noise_buffer.type());
        #endif // defined WITH_CUDA
        
        std::cout << "Resizing Canvas - Done." << std::endl;
        image_resize_required = false;
    }

    polar_image.setTo(cv::Scalar(0));

    // gen ambient noise


    bool static_tf = true;

    rm::StopWatch sw;
    double el;

    DirectedWave wave;
    wave.energy       =  1.0;
    wave.polarization =  0.5;
    wave.frequency    = 76.5; // GHz
    wave.velocity     =  0.3; // m / ns - speed in air
    wave.material_id  =  0;   // air
    wave.time         =  0.0; // ns

    el = sw();

    int n_angles = polar_image.cols;

    float noise_energy_max = 80.0;
    float noise_energy_min = 5.0;
    float energy_loss = 0.05;

    std::random_device                      rand_dev;
    std::mt19937                            gen(rand_dev());
    std::uniform_real_distribution<float>   dist_uni(0.0, 1.0);


    // without motion: update Tsm only once
    if(!include_motion)
    {
        if(!updateTsm())
        {
            return;
        }
    }

    rm::StopWatch sw_radar_sim;
    sw_radar_sim();
    for(size_t angle_id = 0; angle_id < n_angles; angle_id++)
    {   
        std::vector<Signal> signals;
        wave.ray.orig     = radar_model.getOrigin(0, angle_id);
        wave.ray.dir      = radar_model.getDirection(0, angle_id);
        
        // std::cout << "- Angle: " << angle_id << std::endl;
        std::vector<DirectedWave> waves = sample_cone(
            wave, 
            params.model.beam_width,
            params.model.n_samples, 
            beam_sample_dist, 
            beam_sample_dist_normal_p_in_cone);

        // with motion: update at each angle
        if(include_motion)
        {
            if(!updateTsm())
            {
                continue;
            }
        }

        rm::Transform Tsm = Tsm_last;
        
        rm::Memory<rm::Transform> Tbms(1);
        Tbms[0] = Tsm;

        for(size_t pass_id = 0; pass_id < params.model.n_reflections; pass_id++)
        {
            // std::cout << "Pass " << pass_id << std::endl; 
            rm::OnDnModel model = make_model(waves);

            // raycast
            sim->setModel(model);
            
            // results in sensor frame!
            ResT results;
            results.hits.resize(model.size());
            results.ranges.resize(model.size());
            results.normals.resize(model.size());
            results.object_ids.resize(model.size());
            
            sim->simulate(Tbms, results);
            
            // Move rays
            for(size_t i=0; i < waves.size(); i++)
            {
                const float wave_range = results.ranges[i];
                
                DirectedWave wave = waves[i];
                wave.moveInplace(wave_range);
                waves[i] = wave;
            }

            // reflect / refract / absorb / return
            std::vector<DirectedWave> waves_new;

            for(size_t i=0; i < waves.size(); i++)
            {
                // read ray data
                const DirectedWave incidence = waves[i];
                const rmagine::Vector surface_normal = results.normals[i].normalize();
                const unsigned int obj_id = results.object_ids[i];

                if(obj_id > 10000)
                {
                    continue;
                } 

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
                        if(pass_id == 0 || record_multi_reflection)
                        {   
                            float time_back = incidence.time * 2.0;
                            signals.push_back({time_back, return_energy_path});
                        }

                        if(pass_id > 0 && record_multi_path)
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

                            if(record_multi_path 
                                && sensor_view_scalar > multipath_threshold)
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

            // update sensor model
            waves = waves_new;
        }

        // std::cout << "Angle: " << angle_id << " " << sw.toc() << ", Signals: " << signals.size() << std::endl;
    
        // sort signals

        // cv::Mat 
        cv::Mat_<float> slice(n_cells, 1, 0.0);

        std::vector<unsigned int> signal_counts(n_cells, 0);

        std::vector<float> verschmierer;
        unsigned int verschmierer_mode;


        if(signal_denoising > 0)
        {
            // std::cout << "Signal Denoising: ";
            if(signal_denoising == 1)
            {
                // std::cout << "Triangular";
                verschmierer = make_denoiser_triangular(
                    signal_denoising_triangular_width,
                    signal_denoising_triangular_mode
                );
                verschmierer_mode = signal_denoising_triangular_mode;
            } else if(signal_denoising == 2) {
                // std::cout << "Gaussian";

                verschmierer = make_denoiser_gaussian(
                    signal_denoising_gaussian_width,
                    signal_denoising_gaussian_mode
                );
                verschmierer_mode = signal_denoising_gaussian_mode;

            } else if(signal_denoising == 3) {
                // std::cout << "Maxwell Boltzmann";
                verschmierer = make_denoiser_maxwell_boltzmann(
                    signal_denoising_mb_width,
                    signal_denoising_mb_mode
                );

                verschmierer_mode = signal_denoising_mb_mode;
            }
            // std::cout << std::endl;
        }

        size_t n_verschmierer = verschmierer.size();


        float max_val = 0.0;
        // Signals to Image
        for(size_t i=0; i<signals.size(); i++)
        {
            auto signal = signals[i];
            // wave speed in air (light speed) * t / 2
            float half_time = signal.time / 2.0;

            float signal_dist = 0.3 * half_time;

            // time_travelled * 
            // float signal_dist = 0.3 * signal.time / 2.0;

            int cell = static_cast<int>(signal_dist / resolution);
            if(cell < n_cells)
            {
                float signal_old = slice.at<float>(cell, 0);
                unsigned signal_count_old = signal_counts[cell];

                // float signal_count_old_f = static_cast<float>(signal_count_old);

                // float signal_new = (signal_old * signal_count_old_f / ( signal_count_old_f + 1.0) ) 
                //     + (signal.strength * 1.0 / (signal_count_old_f + 1.0) );

                // signal_counts[cell]++;

                // signal_count_old_f * signal_old + (1.0 - signal_count_old_f) * signal;

                // slice.at<float>(cell, 0) += signal.strength;

                // float weights[] = {0.1, 0.2, 0.5, 0.2, 0.1}
                // float weights[] = {}

                // slice.at<float>(cell, 0) += signal.strength * pow(
                //     signal_dist, 1.0);

                if(signal_denoising > 0)
                {
                    // triangular signal denoising
                    for(size_t vid = 0; vid < n_verschmierer; vid++)
                    {
                        int glob_id = static_cast<int>(vid) + static_cast<int>(cell) - static_cast<int>(verschmierer_mode);
                        if(glob_id > 0 && glob_id < n_cells)
                        {
                            slice.at<float>(glob_id, 0) += signal.strength * verschmierer[vid];
                            slice.at<float>(glob_id, 0) = std::max(
                                    slice.at<float>(glob_id, 0),
                                    (float)signal.strength * verschmierer[vid]
                                );

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
        slice *= energy_max;

        // // cv::GaussianBlur(slice, slice, cv::Size(1, 9), 0);
    
        // std::cout << "max_val: " << max_val << std::endl;
        int col = (scroll_image + angle_id) % polar_image.cols;

        if(ambient_noise)
        {
            // apply noise
            // low freq perlin
            double scale = 0.05;
            // high freq perlin
            double scale2 = 0.2;

            double random_begin = dist_uni(gen) * 1000.0;
            
            for(size_t i=0; i<slice.rows; i++)
            {
                float signal = slice.at<float>(i) * energy_max;

                double p;

                if(ambient_noise == 1) // UNIFORM
                {
                    p = dist_uni(gen);
                } else if(ambient_noise == 2) // PERLIN
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

                float noise_at_0 = signal_amp * ambient_noise_at_signal_0;
                float noise_at_1 = signal_amp * ambient_noise_at_signal_1;

                float signal__ = std::pow(signal_, 4.0);

                float noise_amp = (signal__ * noise_at_0 + (1.0 - signal__) * noise_at_1);

                // noise_amp * p * signal_max;
                
                float noise_energy_max = signal_max * ambient_noise_energy_max;
                float noise_energy_min = signal_max * ambient_noise_energy_min;
                float energy_loss = ambient_noise_energy_loss;

                float y_noise = noise_amp * p;

                float x = (static_cast<float>(i) + 0.5) * resolution;

                y_noise = y_noise + (noise_energy_max - noise_energy_min) * exp(-energy_loss * x) + noise_energy_min;
                y_noise = abs(y_noise);

                slice.at<float>(i) = signal + y_noise;
            }
        }
        
        float max_signal = 120.0;
        slice *= max_signal / max_val;

        slice.convertTo(polar_image.col(col), CV_8UC1);

        if(include_motion)
        {
            ros::spinOnce();
        }
    }

    double el_radar_sim = sw_radar_sim();

    std::cout << "SIM in " << el_radar_sim << "s" << std::endl;
}


// void simulateImage()
// {
//     float wave_energy_threshold = 0.001;

//     std::cout << "Reset Buffers" << std::endl;

//     if(image_resize_required)
//     {
//         // polar_image = cv::Mat_<unsigned char>(n_cells, radar_model.theta.size);
//         std::cout << "Resize Canvas" << std::endl;
//         polar_image.resize(n_cells);
//         std::cout << "Resizing Canvas - Done." << std::endl;
//         image_resize_required = false;
//     }

//     polar_image.setTo(cv::Scalar(0));

//     bool static_tf = true;

//     rm::StopWatch sw;
//     double el;

//     DirectedWave wave;
//     wave.energy       =  1.0;
//     wave.polarization =  0.5;
//     wave.frequency    = 76.5; // GHz
//     wave.velocity     =  0.3; // m / ns - speed in air
//     wave.material_id  =  0;   // air
//     wave.time         =  0.0; // ns

//     el = sw();

//     int n_angles = polar_image.cols;

//     // cv::Mat_<float> perlin_noise_image(cv::Size(n_angles, n_cells));
//     // fill_perlin_noise(perlin_noise_image, 0.1);

//     float noise_energy_max = 80.0;
//     float noise_energy_min = 5.0;
//     float energy_loss = 0.05;


//     // without motion: update Tsm only once
//     if(!include_motion)
//     {
//         if(!updateTsm())
//         {
//             return;
//         }
//     }

//     rm::StopWatch sw_radar_sim;
//     sw_radar_sim();
//     for(size_t angle_id=0; angle_id < n_angles; angle_id++)
//     {   
//         std::vector<Signal> signals;
//         wave.ray.orig     = radar_model.getOrigin(0, angle_id);
//         wave.ray.dir      = radar_model.getDirection(0, angle_id);
        
//         // std::cout << "- Angle: " << angle_id << std::endl;
//         std::vector<DirectedWave> waves = sample_cone(
//             wave, 
//             params.model.beam_width,
//             params.model.n_samples, 
//             beam_sample_dist, 
//             beam_sample_dist_normal_p_in_cone);

//         // std::cout << "-- getTsm" << std::endl;
//         // auto Tsm_opt = getTsm();

//         // Tsm: T[v[0.863185,-1.164,1.49406], E[0.0149786, 0.00858233, 3.04591]]

//         // Tsm.R = rm::EulerAngles{0.0149786, 0.00858233, 3.04591};
//         // Tsm.t = rm::Vector3{0.863185,-1.164,1.49406};

//         // with motion: update at each angle
//         if(include_motion)
//         {
//             if(!updateTsm())
//             {
//                 continue;
//             }
//         }

//         rm::Transform Tsm = Tsm_last;
        
//         rm::Memory<rm::Transform> Tbms(1);
//         Tbms[0] = Tsm;


//         for(size_t pass_id=0; pass_id < params.model.n_reflections; pass_id++)
//         {
//             // std::cout << "Pass " << pass_id << std::endl; 
//             rm::OnDnModel model = make_model(waves);

//             // raycast
//             sim->setModel(model);
            
//             // results in sensor frame!
//             ResT results;
//             results.hits.resize(model.size());
//             results.ranges.resize(model.size());
//             results.normals.resize(model.size());
//             results.object_ids.resize(model.size());
//             // std::cout << "SIM" << std::endl;
//             sim->simulate(Tbms, results);
//             // std::cout << "done" << std::endl;

//             // Move rays
//             for(size_t i=0; i < waves.size(); i++)
//             {
//                 const float wave_range = results.ranges[i];
                
//                 DirectedWave wave = waves[i];
//                 wave.moveInplace(wave_range);
//                 waves[i] = wave;
//             }

//             // reflect / refract / absorb / return
//             std::vector<DirectedWave> waves_new;

//             for(size_t i=0; i < waves.size(); i++)
//             {
//                 // read ray data
//                 const DirectedWave incidence = waves[i];
//                 const rmagine::Vector surface_normal = results.normals[i].normalize();
//                 const unsigned int obj_id = results.object_ids[i];

//                 if(obj_id > 10000)
//                 {
//                     continue;
//                 } 

//                 // inititalize
//                 DirectedWave reflection = incidence;
//                 DirectedWave refraction = incidence;

//                 // if wave was in air, switch to new material
//                 // else if wave was in material, switch to air (is this right ?)
//                 if(incidence.material_id == material_id_air)
//                 {
//                     refraction.material_id = object_materials[obj_id];
//                 } else {
//                     refraction.material_id = material_id_air;
//                 }

//                 float v_refraction = 1.0;

//                 if(incidence.material_id != refraction.material_id)
//                 {
//                     v_refraction = params.materials.data[refraction.material_id].velocity;
//                 } else {
//                     v_refraction = incidence.velocity;
//                 }

//                 // Build surface patch
//                 auto res = fresnel(surface_normal, incidence, v_refraction);

//                 reflection.ray.dir = res.first.ray.dir;
//                 reflection.energy = res.first.energy;

//                 if(reflection.energy > wave_energy_threshold)
//                 {
//                     waves_new.push_back(reflection);
                    
//                     // split total reflection energy into 
//                     // - scene reflection energy
//                     // - path return energy
//                     // - air return energy
//                     //
//                     // path return energy = air return energy 
//                     //      for pass 0
//                     // path return energy = air return energy = scene reflection energy
//                     //      for pass 0 and perpendicular hit
                    
//                     if(reflection.material_id == material_id_air)
//                     {
//                         // 1. signal travelling back along the pass

//                         auto material = params.materials.data[refraction.material_id];

//                         double incidence_angle = get_incidence_angle(surface_normal, incidence);
//                         // 1. signal traveling over path
//                         double return_energy_path = back_reflection_shader(
//                             incidence_angle,
//                             reflection.energy,
//                             material.ambient, // ambient
//                             material.diffuse, // diffuse
//                             material.specular // specular
//                         );

//                         // signal traveling the air path == traversal path
//                         if(pass_id == 0 || record_multi_reflection)
//                         {   
//                             float time_back = incidence.time * 2.0;
//                             signals.push_back({time_back, return_energy_path});
//                         }

//                         if(pass_id > 0 && record_multi_path)
//                         {
//                             // 2. signal traveling the air path
//                             Signal signal_air;
//                             // signal_path.time = incidence.time * 2.0;
//                             // signal_path.strength = return_energy_path;
                            
//                             rm::Vector dir_sensor_to_hit = reflection.ray.orig;
//                             const double distance_between_sensor_and_hit = dir_sensor_to_hit.l2norm();
//                             dir_sensor_to_hit.normalizeInplace();
//                             // time_to_sensor in [ns] assuming transmission through space
//                             const double time_to_sensor = distance_between_sensor_and_hit / reflection.velocity;


//                             double sensor_view_scalar = wave.ray.dir.dot(dir_sensor_to_hit);

//                             double angle_between_reflection_and_sensor_dir 
//                                 = angle_between(-reflection.ray.dir, dir_sensor_to_hit);

//                             if(record_multi_path 
//                                 && sensor_view_scalar > multipath_threshold)
//                             {
//                                 double return_energy_air = back_reflection_shader(
//                                     angle_between_reflection_and_sensor_dir,
//                                     reflection.energy,
//                                     material.ambient, // ambient
//                                     material.diffuse, // diffuse
//                                     material.specular // specular
//                                 );

//                                 signal_air.time = incidence.time + time_to_sensor;
//                                 signal_air.strength = return_energy_air;
                                
//                                 signals.push_back(signal_air);
//                             }
//                         }
//                     }
//                 }

//                 refraction.ray.dir = res.second.ray.dir;
//                 refraction.energy = res.second.energy;

//                 if(refraction.energy > wave_energy_threshold)
//                 {
//                     waves_new.push_back(refraction);
//                 }
//             }

//             // skip certain distance for safety
//             float skip_dist = 0.001;
//             for(size_t i=0; i<waves_new.size(); i++)
//             {
//                 waves_new[i].moveInplace(skip_dist);
//             }

//             // update sensor model
//             waves = waves_new;
//         }

//         // std::cout << "Angle: " << angle_id << " " << sw.toc() << ", Signals: " << signals.size() << std::endl;
    
//         // sort signals

//         // cv::Mat 
//         cv::Mat_<float> slice(n_cells, 1, 0.0);

//         std::vector<unsigned int> signal_counts(n_cells, 0);

//         std::vector<float> verschmierer;
//         unsigned int verschmierer_mode;


//         if(signal_denoising > 0)
//         {
//             // std::cout << "Signal Denoising: ";
//             if(signal_denoising == 1)
//             {
//                 // std::cout << "Triangular";
//                 verschmierer = make_denoiser_triangular(
//                     signal_denoising_triangular_width,
//                     signal_denoising_triangular_mode
//                 );
//                 verschmierer_mode = signal_denoising_triangular_mode;
//             } else if(signal_denoising == 2) {
//                 // std::cout << "Gaussian";

//                 verschmierer = make_denoiser_gaussian(
//                     signal_denoising_gaussian_width,
//                     signal_denoising_gaussian_mode
//                 );
//                 verschmierer_mode = signal_denoising_gaussian_mode;

//             } else if(signal_denoising == 3) {
//                 // std::cout << "Maxwell Boltzmann";
//                 verschmierer = make_denoiser_maxwell_boltzmann(
//                     signal_denoising_mb_width,
//                     signal_denoising_mb_mode
//                 );

//                 verschmierer_mode = signal_denoising_mb_mode;
//             }
//             // std::cout << std::endl;
//         }

//         size_t n_verschmierer = verschmierer.size();


//         float max_val = 0.0;
//         // Signals to Image
//         for(size_t i=0; i<signals.size(); i++)
//         {
//             auto signal = signals[i];
//             // wave speed in air (light speed) * t / 2
//             float half_time = signal.time / 2.0;

//             float signal_dist = 0.3 * half_time;

//             // time_travelled * 
//             // float signal_dist = 0.3 * signal.time / 2.0;

//             int cell = static_cast<int>(signal_dist / resolution);
//             if(cell < n_cells)
//             {
//                 float signal_old = slice.at<float>(cell, 0);
//                 unsigned signal_count_old = signal_counts[cell];

//                 // float signal_count_old_f = static_cast<float>(signal_count_old);

//                 // float signal_new = (signal_old * signal_count_old_f / ( signal_count_old_f + 1.0) ) 
//                 //     + (signal.strength * 1.0 / (signal_count_old_f + 1.0) );

//                 // signal_counts[cell]++;

//                 // signal_count_old_f * signal_old + (1.0 - signal_count_old_f) * signal;

//                 // slice.at<float>(cell, 0) += signal.strength;

//                 // float weights[] = {0.1, 0.2, 0.5, 0.2, 0.1}
//                 // float weights[] = {}

//                 // slice.at<float>(cell, 0) += signal.strength * pow(
//                 //     signal_dist, 1.0);

//                 if(signal_denoising > 0)
//                 {
//                     // triangular signal denoising
//                     for(size_t vid = 0; vid < n_verschmierer; vid++)
//                     {
//                         int glob_id = static_cast<int>(vid) + static_cast<int>(cell) - static_cast<int>(verschmierer_mode);
//                         if(glob_id > 0 && glob_id < n_cells)
//                         {
//                             slice.at<float>(glob_id, 0) += signal.strength * verschmierer[vid];
//                             slice.at<float>(glob_id, 0) = std::max(
//                                     slice.at<float>(glob_id, 0),
//                                     (float)signal.strength * verschmierer[vid]
//                                 );

//                             if(slice.at<float>(glob_id, 0) > max_val)
//                             {
//                                 max_val = slice.at<float>(glob_id, 0);
//                             }
//                         }
//                     }
//                 } else {
//                     // no signal denoising

//                     // slice.at<float>(cell, 0) += 1.0;
//                     // slice.at<float>(cell, 0) += signal.strength;
//                     slice.at<float>(cell, 0) = std::max(
//                         slice.at<float>(cell, 0),
//                         (float)signal.strength
//                     );

//                     if(slice.at<float>(cell, 0) > max_val)
//                     {
//                         max_val = slice.at<float>(cell, 0);
//                     }
//                 }
//             }
//         }

//         // normalize
//         slice *= energy_max;

//         // // cv::GaussianBlur(slice, slice, cv::Size(1, 9), 0);
    
//         // std::cout << "max_val: " << max_val << std::endl;
//         int col = (scroll_image + angle_id) % polar_image.cols;

//         std::random_device                      rand_dev;
//         std::mt19937                            gen(rand_dev());
//         std::uniform_real_distribution<float>   dist_uni(0.0, 1.0);    

//         if(ambient_noise)
//         {
//             // apply noise
//             // low freq perlin
//             double scale = 0.05;
//             // high freq perlin
//             double scale2 = 0.2;

//             double random_begin = dist_uni(gen) * 1000.0;
            
//             for(size_t i=0; i<slice.rows; i++)
//             {
//                 float signal = slice.at<float>(i) * energy_max;

//                 double p;

//                 if(ambient_noise == 1) // UNIFORM
//                 {
//                     p = dist_uni(gen);
//                 } else if(ambient_noise == 2) // PERLIN
//                 {
//                     double p_perlin1 = perlin_noise(
//                         random_begin + static_cast<double>(i) * scale, 
//                         random_begin + static_cast<double>(col) * scale);
                    
//                     double p_perlin2 = perlin_noise(
//                         random_begin + static_cast<double>(i) * scale2, 
//                         random_begin + static_cast<double>(col) * scale2);

//                     p = 0.9 * p_perlin1 + 0.1 * p_perlin2;
//                 }
                

//                 // p = p * 
//                 // p = (p + 1.0) / 2.0; // [0.0-1.0]

//                 // verwurschteltn
//                 float signal_min = 0;
//                 float signal_max = max_val;
//                 float signal_amp = signal_max - signal_min;

//                 float signal_ = 1.0 - ((signal - signal_min) / signal_amp);

//                 float noise_at_0 = signal_amp * ambient_noise_at_signal_0;
//                 float noise_at_1 = signal_amp * ambient_noise_at_signal_1;

//                 float signal__ = std::pow(signal_, 4.0);

//                 float noise_amp = (signal__ * noise_at_0 + (1.0 - signal__) * noise_at_1);

//                 // noise_amp * p * signal_max;
                
//                 float noise_energy_max = signal_max * ambient_noise_energy_max;
//                 float noise_energy_min = signal_max * ambient_noise_energy_min;
//                 float energy_loss = ambient_noise_energy_loss;

//                 float y_noise = noise_amp * p;

//                 float x = (static_cast<float>(i) + 0.5) * resolution;

//                 y_noise = y_noise + (noise_energy_max - noise_energy_min) * exp(-energy_loss * x) + noise_energy_min;
//                 y_noise = abs(y_noise);

//                 slice.at<float>(i) = signal + y_noise;
//             }
//         }
        
//         float max_signal = 120.0;

//         slice *= max_signal / max_val;

//         slice.convertTo(polar_image.col(col), CV_8UC1);

//         if(include_motion)
//         {
//             ros::spinOnce();
//         }
//     }

//     double el_radar_sim = sw_radar_sim();

//     std::cout << "SIM in " << el_radar_sim << "s" << std::endl;
// }

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
    sim = std::make_shared<rm::OnDnSimulatorEmbree>(
        map);

    // offset of sensor center to frame
    auto Tsb = rm::Transform::Identity();
    Tsb.t.z = 0.0;

    sim->setTsb(Tsb);

    n_cells = 3424;
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

    polar_image = cv::Mat_<unsigned char>(n_cells, radar_model.theta.size);

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
    sim = std::make_shared<rm::OnDnSimulatorEmbree>(
        map);

    // offset of sensor center to frame
    auto Tsb = rm::Transform::Identity();
    Tsb.t.z = 0.0;

    sim->setTsb(Tsb);

    n_cells = 3424;
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
    polar_image = cv::Mat_<unsigned char>(n_cells, radar_model.theta.size);
    perlin_noise_buffer = cv::Mat_<float>(n_cells, radar_model.theta.size);

    #if defined WITH_CUDA
    perlin_noise_buffer_cuda = cv::cuda::GpuMat(perlin_noise_buffer.size(), perlin_noise_buffer.type());
    #endif // defined WITH_CUDA

    ros::Rate r(100);

    while(nh.ok())
    {
        loadParameters();
        ROS_INFO("Simulate!");
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
