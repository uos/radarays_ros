#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <rmagine/simulation/OnDnSimulatorEmbree.hpp>
#include <rmagine/util/prints.h>
#include <rmagine/util/StopWatch.hpp>

#include <tf2_ros/transform_listener.h>

#include <iostream>

#include <dynamic_reconfigure/server.h>
#include <radarays_ros/RayReflectionConfig.h>

#include <random>

#include <radarays_ros/radar_math.h>
#include <radarays_ros/radar_types.h>
#include <radarays_ros/radar_algorithms.h>

using namespace radarays_ros;

namespace rm = rmagine;

// intersection attributes
using ResT = rm::Bundle<
    rm::Hits<rm::RAM>,
    rm::Ranges<rm::RAM>,
    rm::Normals<rm::RAM>,
    rm::ObjectIds<rm::RAM> // connection to material
>;

std::string map_frame = "map";
std::string sensor_frame = "navtech";

std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;

rm::OnDnSimulatorEmbreePtr sim;
rm::EmbreeMapPtr map;

std::shared_ptr<ros::NodeHandle> nh_p;


ros::Publisher pub_marker;

// params
size_t n_reflections = 0; // maximum number of reflections per beam
float ray_yaw = 0.0;
bool spinning = false;
bool transparency_by_energy = false;
float wave_start_energy = 1.0;
float wave_end_energy = 0.0001;



bool    cone_sampling = true;
int     cone_n = 1;
float   cone_width = 0.01;
float   cone_sample_dist_normal_p_in_cone = 0.95;
int     cone_sample_dist = 0;


bool    shoot_all_directions = false;
float   yaw_increment = 0.01;


std::vector<int> object_materials;
std::vector<double> radiowave_velocity;
int air_mat_id = 0;





// // https://gpg.geosci.xyz/content/GPR/table_velocity.html
// // unit: m/ns
// static double radiowave_velocity[] = {
//     0.3,    // 0: air
//     0.033,  // 1: water
//     0.01,   // 2: water sea
//     0.16,   // 3: ice
//     0.15,   // 4: dry sand
//     0.06,   // 5: saturated sand
//     0.12,   // 6: limestone
//     0.09,   // 7: shales
//     0.07,   // 8: silts
//     0.06,   // 9: clays
//     0.13,   // 10: granite
//     0.13,   // 11: anhydrites,
//     0.0     // 12: zero
// };

// static unsigned int air_mat_id = 0;


void cfgCB(
    radarays_ros::RayReflectionConfig &config,
    uint32_t level) 
{
    ray_yaw = config.ray_yaw * M_PI / 180.0;
    n_reflections = config.n_reflections;
    spinning = config.spinning;
    
    transparency_by_energy = config.energy;

    cone_sampling = config.cone_sampling;
    cone_width = config.cone_width * M_PI / 180.0;
    cone_n = config.cone_samples;
    cone_sample_dist = config.cone_sample_dist;
    cone_sample_dist_normal_p_in_cone = config.cone_sample_dist_normal_p_in_cone;
    

    shoot_all_directions = config.shoot_all_directions;
    yaw_increment = config.yaw_increment * M_PI / 180.0;
}

struct MyColor {
    float r;
    float g;
    float b;
    float a;

    std_msgs::ColorRGBA to_ros() const
    {
        std_msgs::ColorRGBA ret;
        ret.r = r;
        ret.g = g;
        ret.b = b;
        ret.a = a;
        return ret;
    }
};

void add_line(
    visualization_msgs::Marker& marker, 
    const rm::Vector& p1, 
    const rm::Vector& p2,
    MyColor color)
{
    marker.colors.push_back(color.to_ros());
    marker.colors.push_back(color.to_ros());

    geometry_msgs::Point p_start, p_end;
    p_start.x   = p1.x;
    p_start.y   = p1.y;
    p_start.z   = p1.z;
    p_end.x     = p2.x;
    p_end.y     = p2.y;
    p_end.z     = p2.z;

    marker.points.push_back(p_start);
    marker.points.push_back(p_end);
}

void loadMaterials()
{
    nh_p->getParam("object_materials", object_materials);

    // material properties
    nh_p->getParam("velocities", radiowave_velocity);
    // nh_p->getParam("ambient", ambient);
    // nh_p->getParam("diffuse", diffuse);
    // nh_p->getParam("specular", specular);

    nh_p->getParam("material_id_air", air_mat_id);
}

void shootRay()
{
    // std::cout << "shootRay" << std::endl;
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

    

    std::vector<DirectedWave> waves;
    
    if(shoot_all_directions)
    {
        float yaw_inc = 0.01;
        float yaw_cur = -M_PI;

        while(yaw_cur < M_PI)
        {
            DirectedWave wave;
            wave.ray.orig = rm::Vector::Zeros();
            wave.ray.dir = {cos(yaw_cur), sin(yaw_cur), 0.0};
            wave.energy = wave_start_energy;
            wave.polarization = 0.5; // both, s and p polarizations
            wave.frequency = 76.5; // GHz. Navtech 76-77 GHz
            wave.velocity = 0.3; // m / ns
            wave.material_id = air_mat_id; // spawn wave in air

            waves.push_back(wave);

            yaw_cur += yaw_inc;
        }
    } else {
        // define first wave
        DirectedWave wave_start;
        wave_start.ray.orig = rm::Vector::Zeros();
        wave_start.ray.dir = {cos(ray_yaw), sin(ray_yaw), 0.0};
        wave_start.energy = wave_start_energy;
        wave_start.polarization = 0.5; // both, s and p polarizations
        wave_start.frequency = 76.5; // GHz. Navtech 76-77 GHz
        wave_start.velocity = 0.3; // m / ns
        wave_start.material_id = air_mat_id; // spawn wave in air

        if(cone_sampling)
        {
            waves = sample_cone(
                wave_start, 
                cone_width, 
                cone_n,
                cone_sample_dist,
                cone_sample_dist_normal_p_in_cone);
        } else {
            waves.push_back(wave_start);
        }
    }
    
    rm::Memory<rm::Transform> Tbms(1);
    Tbms[0] = Tsm;

    visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = sensor_frame;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.pose.orientation.w = 1.0;
    marker.id = 0;
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;

    size_t n_passes = n_reflections + 1;

    rm::StopWatch sw;

    sw();
    for(size_t i=0; ;i++)
    {
        // std::cout << "Pass: " << i+1 << std::endl;
        // std::cout << "- Compute Intersections" << std::endl;
        // 1. Compute next intersections
        rm::OnDnModel model = make_model(waves);

        // raycast
        sim->setModel(model);

        ResT results;
        results.hits.resize(model.size());
        results.ranges.resize(model.size());
        results.normals.resize(model.size());
        results.object_ids.resize(model.size());
        sim->simulate(Tbms, results);

        // Move rays
        // draw intermediate
        for(size_t ray_id=0; ray_id < results.ranges.size(); ray_id++)
        {
            // read ray data
            const DirectedWave wave = waves[ray_id];
            const float wave_range = results.ranges[ray_id];

            // intersection and new origin of reflection and refraction rays
            const rm::Vector intersection_point = wave.ray.orig + wave.ray.dir * wave_range;

            MyColor color = {1.0, 0.0, 0.0, 1.0};

            if(transparency_by_energy)
            {
                color.a = (float)wave.energy;
            }

            if(wave.material_id != air_mat_id)
            {
                color.r = 0.0;
                color.g = 1.0;
                color.b = 0.0;
            }

            add_line(marker, wave.ray.orig, intersection_point, color);

            waves[ray_id] = wave.move(wave_range);

            if(ray_id == 0 && i == 0)
            {
                std::cout << "First Hit: " << wave_range << "m, object: " << results.object_ids[ray_id] << std::endl;
            }
        }

        if(i >= n_reflections)
        {
            break;
        }

        // Reflactions and Refractions: generate new set of waves
        // change this to somehow push_back

        std::vector<DirectedWave> waves_new = fresnel(
            waves, 
            results.normals,
            results.object_ids, 
            object_materials.data(),
            radiowave_velocity.data(),
            air_mat_id,
            wave_end_energy);

        // skip certain distance for safety
        float skip_dist = 0.001;
        for(size_t j=0; j<waves_new.size(); j++)
        {
            waves_new[j].moveInplace(skip_dist);
        }

        // update sensor model
        waves = waves_new;
    }

    double el = sw();
    std::cout << "shootRay: " << el << "s" << std::endl;

    pub_marker.publish(marker);

    if(spinning)
    {
        ray_yaw += 0.01;

        if(ray_yaw > M_PI)
        {
            ray_yaw = -M_PI;
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ray_reflection_test");

    ros::NodeHandle nh;
    nh_p = std::make_shared<ros::NodeHandle>("~");

    std::string map_file = "/home/amock/blender_projects/oru/oru3.dae";

    nh_p->getParam("map_file", map_file);
    nh_p->getParam("map_frame", map_frame);

    loadMaterials();


    map = rm::import_embree_map(map_file);
    sim = std::make_shared<rm::OnDnSimulatorEmbree>(
        map);

    auto Tsb = rm::Transform::Identity();
    sim->setTsb(Tsb);

    // setting up dynamic reconfigure
    dynamic_reconfigure::Server<radarays_ros::RayReflectionConfig> server;
    dynamic_reconfigure::Server<radarays_ros::RayReflectionConfig>::CallbackType f;
    f = boost::bind(&cfgCB, _1, _2);
    server.setCallback(f);

    // setting up tf
    tf_buffer = std::make_shared<tf2_ros::Buffer>();
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // traversal markers
    pub_marker = nh_p->advertise<visualization_msgs::Marker>("traversal", 1);

    ROS_INFO("Ray reflection test started.");

    ros::Rate r(100);
    while(ros::ok())
    {
        loadMaterials();
        shootRay();
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}