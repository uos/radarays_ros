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


using namespace radarays_ros;

namespace rm = rmagine;

std::string map_frame = "map";
std::string sensor_frame = "navtech";


cv::Mat polar_image;

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

RadarParams params = default_params();

// float beam_width = 8.0 * M_PI / 180.0; // width of the beam in radian
float resolution = 0.1; // resolution of a range measurement
size_t n_cells = 300; // number of range measurements per beam
// size_t n_samples = 100; // number samples per beam
// size_t n_reflections = 2; // maximum number of reflections per beam

float energy_max = 1.0;
float energy_min = 0.0;

float multipath_threshold = 0.5;
// unsigned record_reflection = 3;

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
float ambient_noise_uniform_max = 0.8;

// Particle Noise
bool particle_noise = false;
float particle_noise_exp_mu = 1.0;

bool record_multi_reflection = true;
bool record_multi_path = true;

std::shared_ptr<ros::NodeHandle> nh_p;

// materials
int material_id_air = 0;
std::vector<int> object_materials;

// std::vector<double> velocities;
// std::vector<double> diffuse;
// std::vector<double> specular;
// std::vector<double> ambient;


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

    bool image_resize_required = false;

    // z_offset
    auto T = rm::Transform::Identity();
    T.t.z = config.z_offset;
    sim->setTsb(T);
    
    radar_model.range.min = config.range_min;
    radar_model.range.max = config.range_max;
    
    params.model.beam_width = config.beam_width * M_PI / 180.0;

    resolution = config.resolution;

    if(n_cells != config.n_cells)
    {
        // Image update needed
        image_resize_required = true;
        n_cells = config.n_cells;
    }

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
    ambient_noise_uniform_max = config.ambient_noise_uniform_max;

    particle_noise = config.particle_noise;
    particle_noise_exp_mu = config.particle_noise_exp_mu;



    scroll_image = config.scroll_image;


    multipath_threshold = config.multipath_threshold;
    record_multi_reflection = config.record_multi_reflection;
    record_multi_path = config.record_multi_path;
}

void updateImageAmbientNoise()
{
    std::random_device                      rand_dev;
    std::mt19937                            gen(rand_dev());
    std::uniform_int_distribution<uint8_t>  dist_uni(static_cast<int>(energy_min * 255.0), static_cast<int>(ambient_noise_uniform_max * 255.0));
    
    for(size_t i=0; i<polar_image.rows; i++)
    {
        for(size_t j=0; j<polar_image.cols; j++)
        {
            polar_image.at<uint8_t>(i,j) = std::max(dist_uni(gen), polar_image.at<uint8_t>(i,j));
        }
    }
}

void updateImageParticleNoise()
{
    std::random_device                      rand_dev;
    std::mt19937                            gen(rand_dev());
    std::normal_distribution<float>         dist_norm(0.0, particle_noise_exp_mu);
    
    size_t Ndraws = 100;

    for(size_t col=0; col<polar_image.cols; col++)
    {
        for(size_t j=0; j<Ndraws; j++)
        {
            int rand_num = static_cast<int>(fabs(dist_norm(gen)));
            if(rand_num < polar_image.rows)
            {
                // std::cout << energy_max << " -> " << rand_num << ", " << col << std::endl;
                polar_image.at<uint8_t>(rand_num, col) = static_cast<int>(energy_max * 255.0);
            }
        }
    }
}

void updateImageGaussian()
{
    cv::GaussianBlur(polar_image, polar_image, cv::Size(5, 5), 0);
}

void updateImageBeam()
{
    sensor_msgs::PointCloud pcl;
    pcl.header.frame_id = sensor_frame;
    pcl.header.stamp = ros::Time::now();
    // TODO:
    // - get Tsb from tf
    // - 
    rm::Transform Tsm;

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
        return;
    }

    std::cout << "Tsm: " << Tsm << std::endl;

    std::cout << "Create RADAR Model" << std::endl;

    rm::OnDnModel ondn_model;
    ondn_model.dirs.resize(radar_model.size() * params.model.n_samples);
    ondn_model.origs.resize(radar_model.size() * params.model.n_samples);
    ondn_model.width = radar_model.getWidth() * params.model.n_samples;
    ondn_model.height = radar_model.getHeight();
    ondn_model.range = radar_model.range;

    std::random_device                      rand_dev;
    std::mt19937                            gen(rand_dev());
    std::uniform_real_distribution<float>   dist_uni(0.0, 1.0);
    std::normal_distribution<float>         dist_normal(0.0, 1.0);



    float percentil = beam_sample_dist_normal_p_in_cone 
        + (1.0 - beam_sample_dist_normal_p_in_cone) / 2.0;
    float z = quantile(percentil);
    float beam_radius = params.model.beam_width / 2.0;
    float stretch = beam_radius / z;
    
    for(size_t vid=0; vid < radar_model.getHeight(); vid++)
    {
        for(size_t hid=0; hid < radar_model.getWidth(); hid++)
        {
            auto dir = radar_model.getDirection(vid, hid);
            auto orig = radar_model.getOrigin(vid, hid);
            
            size_t thid = hid * params.model.n_samples;

            size_t bufid0 = ondn_model.getBufferId(vid, thid);
            ondn_model.dirs[bufid0]  = dir;
            ondn_model.origs[bufid0] = orig;

            for(size_t i=1; i<params.model.n_samples; i++)
            {
                size_t bufid = ondn_model.getBufferId(vid, thid + i);

                float random_radius, random_angle;

                if(beam_sample_dist == 0)
                {
                    random_radius = sqrt(dist_uni(gen)) * beam_radius;
                    random_angle = dist_uni(gen) * 2.0f * M_PI - M_PI;
                } else if(beam_sample_dist == 1) {
                    random_radius = dist_uni(gen) * beam_radius;
                    random_angle = dist_uni(gen) * 2.0f * M_PI - M_PI;
                } else if(beam_sample_dist == 2) {
                    random_radius = sqrt(dist_normal(gen)) * stretch;
                    random_angle = dist_uni(gen) * 2.0f * M_PI - M_PI;
                } else if(beam_sample_dist == 3) {
                    random_radius = dist_normal(gen) * stretch;
                    random_angle = dist_uni(gen) * 2.0f * M_PI - M_PI;
                }

                float alpha = random_radius * cos(random_angle);
                float beta = random_radius * sin(random_angle);

                rm::EulerAngles e = {0.f, alpha, beta};
                ondn_model.dirs[bufid] = e * dir;
                ondn_model.origs[bufid] = orig;
            }
        }
    }

    std::cout << "RADAR Model created." << std::endl;
    sim->setModel(ondn_model);

    ResT results;
    results.hits.resize(ondn_model.size());
    results.ranges.resize(ondn_model.size());
    results.normals.resize(ondn_model.size());
    results.object_ids.resize(ondn_model.size());

    

    std::cout << "Simulate " << ondn_model.getHeight() << "x" << ondn_model.getWidth() << std::endl;
    rm::Memory<rm::Transform> Tbms(1);
    Tbms[0] = Tsm;
    sim->simulate(Tbms, results);

    rm::Memory<float> current_ranges(ondn_model.size());
    rm::Memory<float> current_energies(ondn_model.size());
    for(size_t i=0; i<ondn_model.size(); i++)
    {
        current_ranges[i] = 0.0;
        current_energies[i] = 1.0;
    }
    
    // create (two?) reflection NxNmodel
    std::vector<size_t> hit_history;
    
    for(size_t i=0; i<params.model.n_reflections; i++)
    {
        size_t Nhits = 0;
        for(size_t vid = 0; vid < ondn_model.getHeight(); vid++)
        {
            for(size_t hid = 0; hid < ondn_model.getWidth(); hid++)
            {
                size_t buf_id = ondn_model.getBufferId(vid, hid);

                if(results.hits[buf_id])
                {
                    const size_t shid = hid / params.model.n_samples;
                    float skip_dist = 0.01;
                    const float range = results.ranges[buf_id];
                    const rm::Vector p = ondn_model.getOrigin(vid, hid) + ondn_model.getDirection(vid, hid) * range;

                    // computing reflection direction
                    const rm::Vector dir = ondn_model.getDirection(vid, hid);
                    const rm::Vector normal = results.normals[buf_id].normalize();
                    rm::Vector dir_ref = dir + normal * (2.0 * dir.dot(-normal));

                    ondn_model.dirs[buf_id] = dir_ref;
                    ondn_model.origs[buf_id] = p + dir_ref * skip_dist;

                    const rm::Vector dir_orig = radar_model.getDirection(vid, shid);
                    
                    // some measures
                    float ray_travel_range = current_ranges[buf_id] + results.ranges[buf_id] + skip_dist;
                    float hit_to_origin_range = p.l2norm();

                    // same dir: 1, 90deg difference: 0
                    float hit_to_origin_angle_similarity = abs((-dir_ref).dot(dir_orig));

                    // 1 if orthogonal to surface, 0 if parallel to surface
                    float hit_scalar = dir.dot(-normal);
                    hit_scalar = hit_scalar * hit_scalar;

                    // std::cout << ray_travel_range << ", " << hit_to_origin_angle_similarity << std::endl;

                    // float hit_to_origin_angle = atan2( (-dir_ref).cross(dir_orig).l2norm(), (-dir_ref).dot(dir_orig) );
                    // float hit_to
                    // float total_ray_range = current_ranges[buf_id] + results.ranges[buf_id];
                    // float range_to_origin = p.l2norm();
                    // float scalar_to_origin = m_sensor_model.getDirection(vid, hid) * normal;
                    // float scalar_to_ray

                    int cell = static_cast<int>(ray_travel_range / resolution);

                    float signal_strength = std::min(1.0f, 
                        hit_to_origin_angle_similarity / ray_travel_range);

                    float signal_energy = hit_scalar * current_energies[buf_id];

                    // at intersection signal_energy is split up into
                    // - absoption
                    // - reflection/transmission to world
                    // - reflection to receiver

                    if(cell < n_cells)
                    {
                        int signal_strength_int = static_cast<int>(255.0 * signal_energy);

                        // polar_image.at<unsigned char>(cell, shid) = signal_strength_int;
                        polar_image.at<unsigned char>(cell, shid) = static_cast<int>(energy_max * 255.0);
                    }

                    current_ranges[buf_id] = ray_travel_range;
                    current_energies[buf_id] = signal_energy;
                    Nhits++;
                } else {
                    // TODO
                    ondn_model.dirs[buf_id] = {0.0, 0.0, 0.0};
                }
            }
        }

        hit_history.push_back(Nhits);

        // std::cout << "Nhits: " << Nhits << std::endl;
        if(Nhits == 0)
        {
            break;
        }

        sim->setModel(ondn_model);
        sim->simulate(Tbms, results);
    }

    std::cout << "Hit History:" << std::endl;
    for(auto hit : hit_history)
    {
        std::cout << hit << " ";
    } 
    std::cout << std::endl;

    if(particle_noise)
    {
        std::cout << "-- updateImageParticleNoise" << std::endl;
        updateImageParticleNoise();
    }

    if(signal_denoising == 1)
    {
        std::cout << "-- Gaussian Smooth" << std::endl;
        updateImageGaussian();
    }
}

/**
 * @brief 
 * 
 * @param signal 
 * @param perlin_noise 
 * @param max_signal_value 
 */
void apply_perlin_noise(
    cv::Mat_<float>& signal, 
    const cv::Mat_<float>& perlin_noise,
    float max_signal_value)
{
    float signal_min = 0.0;
    float signal_max = max_signal_value;
    float signal_amp = signal_max - signal_min;

    cv::Mat_<float> signal_ = 1.0 - ((signal - signal_min) / signal_amp);

    float noise_at_0 = 1.0;
    float noise_at_1 = 0.0;
    
    cv::Mat_<float> signal__;
    cv::pow(signal_, 4.0, signal__);


    // signal_
    const cv::Mat_<float> noise_amps = (signal__ * noise_at_0 + (1.0 - signal__) * noise_at_1);
    
    // std::cout << "noise amps" << std::endl;
    // for(size_t i=0; i<noise_amps.rows; i++)
    // {
    //     std::cout << signal.at<float>(i) << "|" << noise_amps.at<float>(i) << ", ";
    // }

    // std::cout << std::endl;

    cv::Mat_<float> perlin_noise_amped = perlin_noise.mul(noise_amps * signal_max);


    // energy loss
    for(size_t i=0; i<perlin_noise_amped.rows; i++)
    {
        
    }

    signal += perlin_noise_amped;
}

void updateImageBeamNew()
{
    


    float wave_energy_threshold = 0.001;

    std::cout << "Reset Buffers" << std::endl;
    polar_image.setTo(cv::Scalar(0));

    bool static_tf = true;

    

    rm::Transform Tsm;

    // Tsm: T[v[0.863185,-1.164,1.49406], E[0.0149786, 0.00858233, 3.04591]]

    Tsm.R = rm::EulerAngles{0.0149786, 0.00858233, 3.04591};
    Tsm.t = rm::Vector3{0.863185,-1.164,1.49406};

    // try {
    //     geometry_msgs::TransformStamped Tsm_ros = tf_buffer->lookupTransform(
    //         map_frame,
    //         sensor_frame,
    //         ros::Time(0)
    //     );

    //     Tsm.t.x = Tsm_ros.transform.translation.x;
    //     Tsm.t.y = Tsm_ros.transform.translation.y;
    //     Tsm.t.z = Tsm_ros.transform.translation.z;
    //     Tsm.R.x = Tsm_ros.transform.rotation.x;
    //     Tsm.R.y = Tsm_ros.transform.rotation.y;
    //     Tsm.R.z = Tsm_ros.transform.rotation.z;
    //     Tsm.R.w = Tsm_ros.transform.rotation.w;

    // } catch(tf2::TransformException ex) {
    //     ROS_WARN_STREAM("TF-Error: " << ex.what());
    //     return;
    // }

    rm::Memory<rm::Transform> Tbms(1);
    Tbms[0] = Tsm;

    // std::cout << "Tsm: " << Tsm << std::endl;

    // rm::StopWatch sw;

    // sw();
    // init waves

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

    cv::Mat_<float> perlin_noise_image(cv::Size(n_angles, n_cells));
    fill_perlin_noise(perlin_noise_image, 0.1);

    float noise_energy_max = 80.0;
    float noise_energy_min = 5.0;
    float energy_loss = 0.05;


    // #pragma omp parallel for
    for(size_t angle_id=0; angle_id < n_angles; angle_id++)
    {   
        std::vector<Signal> signals;
        // std::cout << "Init angle " << angle_id << std::endl;
        wave.ray.orig     = radar_model.getOrigin(0, angle_id);
        wave.ray.dir      = radar_model.getDirection(0, angle_id);
        
        std::vector<DirectedWave> waves = sample_cone(
            wave, 
            params.model.beam_width,
            params.model.n_samples, 
            beam_sample_dist, 
            beam_sample_dist_normal_p_in_cone);

        for(size_t pass_id=0; ;pass_id++)
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

            if(pass_id >= params.model.n_reflections)
            {
                break;
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

                                // share energy

                                // if(pass_id == 1)
                                // {
                                    // std::cout << "Signal pass " << pass_id << std::endl;
                                    // std::cout << "- Air: " << signal_air.strength << ", ang: " << angle_between_reflection_and_sensor_dir << std::endl; 
                                    // std::cout << "- Path: " << signal_path.strength << ", ang: " << incidence_angle << std::endl;
                                    // if(signal_air.strength > signal_path.strength)
                                    // {
                                    //     std::cout << "Air signal > Path signal" << std::endl;
                                    // }
                                // }
                                
                                signals.push_back(signal_air);
                            }

                            // signals.push_back(signal_path);
                        }


                    }
                }

                refraction.ray.dir = res.second.ray.dir;
                refraction.energy = res.second.energy;

                if(refraction.energy > wave_energy_threshold)
                {
                    waves_new.push_back(refraction);

                    if(refraction.material_id == material_id_air)
                    {
                        rm::Vector dir_sensor = refraction.ray.orig;

                        // multipath only when direction to sensor 
                        // is similar to beam direction (minus)


                        // distance_to_sensor in [m]
                        const double distance_to_sensor = dir_sensor.l2norm();
                        dir_sensor.normalizeInplace();


                        // refraction.velocity in [nm / s]
                        const double time_to_sensor = distance_to_sensor / refraction.velocity;
                        double angle_diff = get_incidence_angle(refraction.ray.dir, dir_sensor);

                        auto material = params.materials.data[incidence.material_id];

                        double energy_back = back_reflection_shader(
                            angle_diff,
                            refraction.energy,
                            material.ambient,
                            material.diffuse,
                            material.specular
                        );

                        // signal to receiver through air
                        Signal sig;
                        sig.time = time_to_sensor + refraction.time;
                        sig.strength = energy_back;
                        // signals.push_back(sig);
                    }
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

        // {
        //     std::sort(signals.begin(), signals.end(), 
        //         [](const Signal & a, const Signal & b)
        //     { 
        //         return a.time > b.time; 
        //     });

        //     std::vector<Signal> signals_new;

        //     if(signals.size() > 0)
        //     {
        //         signals_new.push_back(signals[0]);

        //         // insert signal between
        //         for(size_t i=1; i<signals.size(); i++)
        //         {
        //             Signal int_sig;
        //             int_sig.time = (signals[i].time + signals[i-1].time) / 2.0;
        //             int_sig.time = (signals[i].strength + signals[i-1].strength) / 2.0;
        //             signals_new.push_back(int_sig);
        //             signals_new.push_back(signals[i]);
        //         }
        //     }

        //     signals = signals_new;
        // }
        

        // cv::Mat 
        cv::Mat_<float> slice(n_cells, 1, 0.0);
        // cv::Mat_<unsigned int> signal_counts(n_cells, 1, 0 );

        std::vector<unsigned int> signal_counts(n_cells, 0);
        // cv::Mat_<unsigned int> signal_counts(cv::Size(1, n_cells), 0);

        
        // float verschmierer[] = {1.0, 2.0, 5.0, 10.0, 9.0, 7.5, 6.0, 4.0, 2.0, 1.0};

        // float verschmierer[] = {1.0, 3.0, 10.0, 7.0, 4.0, 2.0, 1.0};

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


                // slice.at<float>(cell, 0) = 0.9 * current_val + 0.1 * signal.strength;
                
                // slice.at<float>(cell, 0)
                
                // slice.at<float>(cell, 0) += pow(signal.strength, 1.0);
                // slice.at<float>(cell, 0) += signal.strength;

                // slice.at<float>(cell, 0) = signal_new;

                // slice.at<float>(cell, 0) = std::max(
                //     slice.at<float>(cell, 0),
                //     powf(signal.strength, 1.0/4.0)
                // );

                // slice.at<float>(cell, 0) = std::max(
                //     signal_old,
                //     (float)signal.strength
                // );

                // slice.at<float>(cell, 0) = 1.0;

                // slice.at<float>(cell, 0) = 1.0;
                // if(slice.at<float>(cell,0) > max_val)
                // {
                //     max_val = slice.at<float>(cell,0);
                // }
            }
        }

        // // cv::GaussianBlur(slice, slice, cv::Size(1, 9), 0);
    
        // std::cout << "max_val: " << max_val << std::endl;
        int col = (scroll_image + angle_id) % polar_image.cols;

        std::random_device                      rand_dev;
        std::mt19937                            gen(rand_dev());
        std::uniform_real_distribution<float>   dist_uni(0.0, 1.0);    

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
                float signal = slice.at<float>(i);

                double p;

                if(ambient_noise == 1)
                {
                    p = dist_uni(gen);
                }

                if(ambient_noise == 2)
                {
                    double p_perlin1 = perlin_noise(
                        random_begin + static_cast<double>(i) * scale, 
                        random_begin + static_cast<double>(col) * scale);
                    
                    double p_perlin2 = perlin_noise(
                        random_begin + static_cast<double>(i) * scale2, 
                        random_begin + static_cast<double>(col) * scale2);

                    p = 0.9 * p_perlin1 + 0.1 * p_perlin2;
                }
                

                // p = p * 
                // p = (p + 1.0) / 2.0; // [0.0-1.0]

                // verwurschteltn
                float signal_min = 0;
                float signal_max = max_val;
                float signal_amp = signal_max - signal_min;


                float signal_ = 1.0 - ((signal - signal_min) / signal_amp);

                float noise_at_0 = signal_amp * 0.3;
                float noise_at_1 = signal_amp * 0.3;

                float signal__ = std::pow(signal_, 4.0);

                float noise_amp = (signal__ * noise_at_0 + (1.0 - signal__) * noise_at_1);

                // noise_amp * p * signal_max;
                
                float noise_energy_max = signal_max * 0.5;
                float noise_energy_min = signal_max * 0.1;
                float energy_loss = 0.05;

                float y_noise = noise_amp * p;

                float x = (static_cast<float>(i) + 0.5) * resolution;

                y_noise = y_noise + (noise_energy_max - noise_energy_min) / exp(energy_loss * x) + noise_energy_min;
                y_noise = abs(y_noise);

                slice.at<float>(i) = signal + y_noise;

            }
        }
        
        float max_signal = 120.0;
        slice *= max_signal / (max_val);

        slice.convertTo(polar_image.col(col), CV_8UC1);
    }

    // cv::Mat perlin_noise_layer(cv::Size(n_angles, n_cells), CV_8UC1);

    

    
    // sw();
    
    

    // el = sw();
    // std::cout << "Perlin noise in " << el << "s" << std::endl;

    // cv::imshow("bla", perlin_noise_layer);
    // cv::waitKey(0);
}

void updateImage()
{
    std::cout << "Reset Buffers" << std::endl;
    polar_image.setTo(cv::Scalar(0));

    // beam model
    updateImageBeam();

    // ambient noise model
    if(ambient_noise)
    {
        std::cout << "-- updateImageAmbientNoise" << std::endl;
        updateImageAmbientNoise();
    }
}

std::shared_ptr<actionlib::SimpleActionServer<radarays_ros::GenRadarImageAction> > as_;
radarays_ros::GenRadarImageFeedback feedback_;
radarays_ros::GenRadarImageResult result_;

void executeCB(const radarays_ros::GenRadarImageGoalConstPtr &goal)
{
    std::cout << "CALL ACTION" << std::endl;

    params = goal->params;
    // auto params = goal->params;

    updateImageBeamNew();
    // updateImage();

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

    ros::Rate r(10);
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
    // tf_buffer = std::make_shared<tf2_ros::Buffer>();
    // tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

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

    ros::Rate r(5);

    while(nh.ok())
    {
        loadParameters();
        ROS_INFO("Simulate!");
        updateImageBeamNew();
        // updateImage();

        sensor_msgs::ImagePtr msg = 
            cv_bridge::CvImage(
                std_msgs::Header(), 
                "mono8",
                polar_image).toImageMsg();
        
        msg->header.stamp = ros::Time::now();
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
