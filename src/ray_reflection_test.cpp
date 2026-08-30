#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <rmagine/simulation/OnDnSimulatorEmbree.hpp>
#include <rmagine/util/prints.h>
#include <rmagine/util/StopWatch.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <iostream>

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

rclcpp::Node::SharedPtr g_node;

std::string map_frame = "map";
std::string sensor_frame = "navtech";

std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;

rm::OnDnSimulatorEmbreePtr sim;
rm::EmbreeMapPtr map;

rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker;

// params (the dynamic_reconfigure/RayReflection.cfg equivalents)
size_t n_reflections = 0; // maximum number of reflections per beam
float ray_yaw = 0.0;
bool spinning = false;
bool transparency_by_energy = false;
float wave_start_energy = 1.0;
float wave_end_energy = 0.0001;

bool    cone_sampling = false;
int     cone_n = 10;
float   cone_width = 1.8 * M_PI / 180.0;
float   cone_sample_dist_normal_p_in_cone = 0.95;
int     cone_sample_dist = 0;

bool    shoot_all_directions = false;
float   yaw_increment = 0.1 * M_PI / 180.0;

std::vector<int> object_materials;
std::vector<double> radiowave_velocity;
int air_mat_id = 0;

rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle;

rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for(const auto &param : parameters)
    {
        const auto &name = param.get_name();

        if(name == "ray_yaw")
        {
            ray_yaw = param.as_double() * M_PI / 180.0;
        }
        else if(name == "n_reflections")
        {
            n_reflections = static_cast<size_t>(param.as_int());
        }
        else if(name == "spinning")
        {
            spinning = param.as_bool();
        }
        else if(name == "energy")
        {
            transparency_by_energy = param.as_bool();
        }
        else if(name == "cone_sampling")
        {
            cone_sampling = param.as_bool();
        }
        else if(name == "cone_width")
        {
            cone_width = param.as_double() * M_PI / 180.0;
        }
        else if(name == "cone_samples")
        {
            cone_n = static_cast<int>(param.as_int());
        }
        else if(name == "cone_sample_dist")
        {
            cone_sample_dist = static_cast<int>(param.as_int());
        }
        else if(name == "cone_sample_dist_normal_p_in_cone")
        {
            cone_sample_dist_normal_p_in_cone = param.as_double();
        }
        else if(name == "shoot_all_directions")
        {
            shoot_all_directions = param.as_bool();
        }
        else if(name == "yaw_increment")
        {
            yaw_increment = param.as_double() * M_PI / 180.0;
        }
    }

    return result;
}

void declareReconfigurableParams(rclcpp::Node::SharedPtr node)
{
    auto declare_ranged_double = [node](const std::string &name, double value, double min, double max)
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = min;
        range.to_value = max;
        descriptor.floating_point_range.push_back(range);
        node->declare_parameter(name, value, descriptor);
    };
    auto declare_ranged_int = [node](const std::string &name, int value, int min, int max)
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        rcl_interfaces::msg::IntegerRange range;
        range.from_value = min;
        range.to_value = max;
        descriptor.integer_range.push_back(range);
        node->declare_parameter(name, value, descriptor);
    };

    declare_ranged_double("ray_yaw", 0.0, -180.0, 180.0);
    declare_ranged_int("n_reflections", 0, 0, 20);
    node->declare_parameter("spinning", false);
    node->declare_parameter("energy", false);

    node->declare_parameter("cone_sampling", false);
    declare_ranged_double("cone_width", 1.8, 0.0, 30.0);
    declare_ranged_int("cone_samples", 10, 0, 1000);
    declare_ranged_int("cone_sample_dist", 0, 0, 3);
    declare_ranged_double("cone_sample_dist_normal_p_in_cone", 0.95, 0.0, 0.999);

    node->declare_parameter("shoot_all_directions", false);
    declare_ranged_double("yaw_increment", 0.1, 0.0, 10.0);

    param_callback_handle = node->add_on_set_parameters_callback(onSetParameters);

    // Same fix as Radar::declareReconfigurableParams(): a --params-file/-p
    // override is resolved at declare_parameter() time but not reflected in
    // these globals until onSetParameters() runs, so replay the resolved
    // values through it once, right after declaring everything.
    onSetParameters(node->get_parameters({
        "ray_yaw", "n_reflections", "spinning", "energy",
        "cone_sampling", "cone_width", "cone_samples", "cone_sample_dist",
        "cone_sample_dist_normal_p_in_cone",
        "shoot_all_directions", "yaw_increment"
    }));
}

struct MyColor {
    float r;
    float g;
    float b;
    float a;

    std_msgs::msg::ColorRGBA to_ros() const
    {
        std_msgs::msg::ColorRGBA ret;
        ret.r = r;
        ret.g = g;
        ret.b = b;
        ret.a = a;
        return ret;
    }
};

void add_line(
    visualization_msgs::msg::Marker& marker,
    const rm::Vector& p1,
    const rm::Vector& p2,
    MyColor color)
{
    marker.colors.push_back(color.to_ros());
    marker.colors.push_back(color.to_ros());

    geometry_msgs::msg::Point p_start, p_end;
    p_start.x   = p1.x;
    p_start.y   = p1.y;
    p_start.z   = p1.z;
    p_end.x     = p2.x;
    p_end.y     = p2.y;
    p_end.z     = p2.z;

    marker.points.push_back(p_start);
    marker.points.push_back(p_end);
}

void loadMaterials(rclcpp::Node::SharedPtr node)
{
    auto object_materials_i64 = node->get_parameter("object_materials").as_integer_array();
    object_materials.assign(object_materials_i64.begin(), object_materials_i64.end());

    radiowave_velocity = node->get_parameter("velocities").as_double_array();

    air_mat_id = static_cast<int>(node->get_parameter("material_id_air").as_int());
}

void shootRay()
{
    rm::Transform Tsm;

    try {
        geometry_msgs::msg::TransformStamped Tsm_ros = tf_buffer->lookupTransform(
            map_frame,
            sensor_frame,
            rclcpp::Time(0)
        );

        Tsm.t.x = Tsm_ros.transform.translation.x;
        Tsm.t.y = Tsm_ros.transform.translation.y;
        Tsm.t.z = Tsm_ros.transform.translation.z;
        Tsm.R.x = Tsm_ros.transform.rotation.x;
        Tsm.R.y = Tsm_ros.transform.rotation.y;
        Tsm.R.z = Tsm_ros.transform.rotation.z;
        Tsm.R.w = Tsm_ros.transform.rotation.w;

    } catch(const tf2::TransformException &ex) {
        RCLCPP_WARN_STREAM(g_node->get_logger(), "TF-Error: " << ex.what());
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

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = g_node->get_clock()->now();
    marker.header.frame_id = sensor_frame;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.pose.orientation.w = 1.0;
    marker.id = 0;
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;

    rm::StopWatch sw;

    sw();
    for(size_t i=0; ;i++)
    {
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

        // Move rays, draw intermediate
        for(size_t ray_id=0; ray_id < results.ranges.size(); ray_id++)
        {
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

        // Reflections and Refractions: generate new set of waves
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

    pub_marker->publish(marker);

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
    rclcpp::init(argc, argv);
    g_node = std::make_shared<rclcpp::Node>("ray_reflection_test");

    std::string map_file = "/home/amock/blender_projects/oru/oru3.dae";
    g_node->declare_parameter("map_file", map_file);
    g_node->declare_parameter("map_frame", map_frame);
    map_file = g_node->get_parameter("map_file").as_string();
    map_frame = g_node->get_parameter("map_frame").as_string();

    g_node->declare_parameter("object_materials", std::vector<int64_t>{});
    g_node->declare_parameter("velocities", std::vector<double>{});
    g_node->declare_parameter("material_id_air", 0);

    declareReconfigurableParams(g_node);
    loadMaterials(g_node);

    map = rm::import_embree_map(map_file);
    sim = std::make_shared<rm::OnDnSimulatorEmbree>(map);

    auto Tsb = rm::Transform::Identity();
    sim->setTsb(Tsb);

    // setting up tf
    tf_buffer = std::make_shared<tf2_ros::Buffer>(g_node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, g_node);

    // traversal markers
    pub_marker = g_node->create_publisher<visualization_msgs::msg::Marker>("~/traversal", 1);

    RCLCPP_INFO(g_node->get_logger(), "Ray reflection test started.");

    rclcpp::Rate r(100);
    while(rclcpp::ok())
    {
        loadMaterials(g_node);
        shootRay();
        r.sleep();
        rclcpp::spin_some(g_node);
    }

    rclcpp::shutdown();
    return 0;
}
