#include "radarays_ros/Radar.hpp"

#include <radarays_ros/ros_helper.h>
#include <rmagine/util/StopWatch.hpp>
#include <rmagine/math/types.h>

namespace radarays_ros
{

Radar::Radar(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<tf2_ros::TransformListener> tf_listener,
    std::string map_frame,
    std::string sensor_frame)
:m_node(node)
,m_tf_buffer(tf_buffer)
,m_tf_listener(tf_listener)
,m_map_frame(map_frame)
,m_sensor_frame(sensor_frame)
{
    m_params = default_params();
    m_material_id_air = 0;
    m_wave_energy_threshold = 0.001;
    m_resample = true;

    m_radar_model.theta.inc = -(2 * M_PI) / 400;
    m_radar_model.theta.min = 0.0;
    m_radar_model.theta.size = 400;
    m_radar_model.phi.inc = 1.0;
    m_radar_model.phi.min = 0.0;
    m_radar_model.phi.size = 1;

    m_polar_image = cv::Mat_<unsigned char>(0, m_radar_model.theta.size);

    declareReconfigurableParams();
    loadParams();
}

std::optional<rm::Transform> Radar::getTsm()
{
    return getTsm(rclcpp::Time(0, 0, m_node->get_clock()->get_clock_type()));
}

std::optional<rm::Transform> Radar::getTsm(rclcpp::Time stamp)
{
    rm::Transform Tsm;

    try {
        geometry_msgs::msg::TransformStamped Tsm_ros = m_tf_buffer->lookupTransform(
            m_map_frame,
            m_sensor_frame,
            stamp
        );

        Tsm.t.x = Tsm_ros.transform.translation.x;
        Tsm.t.y = Tsm_ros.transform.translation.y;
        Tsm.t.z = Tsm_ros.transform.translation.z;
        Tsm.R.x = Tsm_ros.transform.rotation.x;
        Tsm.R.y = Tsm_ros.transform.rotation.y;
        Tsm.R.z = Tsm_ros.transform.rotation.z;
        Tsm.R.w = Tsm_ros.transform.rotation.w;

    } catch(const tf2::TransformException &ex) {
        RCLCPP_WARN_STREAM(m_node->get_logger(), "TF-Error: " << ex.what());
        return {};
    }

    return Tsm;
}

// updating global vars:
// - Tsm_last = Tsm;
// - Tsm_stamp_last = Tsm_stamp;
// - stamp_last = now();
// - has_last = true;
bool Radar::updateTsm()
{
    return updateTsm(rclcpp::Time(0, 0, m_node->get_clock()->get_clock_type()));
}

bool Radar::updateTsm(rclcpp::Time stamp)
{
    std::optional<rm::Transform> Tsm_opt;
    rclcpp::Time Tsm_stamp;

    try {
        rm::Transform Tsm;
        geometry_msgs::msg::TransformStamped Tsm_ros = m_tf_buffer->lookupTransform(
            m_map_frame,
            m_sensor_frame,
            stamp);

        Tsm.t.x = Tsm_ros.transform.translation.x;
        Tsm.t.y = Tsm_ros.transform.translation.y;
        Tsm.t.z = Tsm_ros.transform.translation.z;
        Tsm.R.x = Tsm_ros.transform.rotation.x;
        Tsm.R.y = Tsm_ros.transform.rotation.y;
        Tsm.R.z = Tsm_ros.transform.rotation.z;
        Tsm.R.w = Tsm_ros.transform.rotation.w;

        Tsm_opt = Tsm;
        Tsm_stamp = Tsm_ros.header.stamp;
    } catch(const tf2::TransformException &ex) {
        RCLCPP_WARN_STREAM(m_node->get_logger(), "TF-Error: " << ex.what());
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
        Tsm_stamp = Tsm_stamp_last + (m_node->get_clock()->now() - stamp_last);
    }

    {
        // updating global stuff
        Tsm_last = Tsm;
        Tsm_stamp_last = Tsm_stamp;
        stamp_last = m_node->get_clock()->now();
        has_last = true;
    }

    return true;
}

void Radar::declareReconfigurableParams()
{
    auto declare_ranged_double = [this](const std::string &name, double value, double min, double max)
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        rcl_interfaces::msg::FloatingPointRange range;
        range.from_value = min;
        range.to_value = max;
        descriptor.floating_point_range.push_back(range);
        m_node->declare_parameter(name, value, descriptor);
    };
    auto declare_ranged_int = [this](const std::string &name, int value, int min, int max)
    {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        rcl_interfaces::msg::IntegerRange range;
        range.from_value = min;
        range.to_value = max;
        descriptor.integer_range.push_back(range);
        m_node->declare_parameter(name, value, descriptor);
    };

    declare_ranged_double("z_offset", m_cfg_z_offset, -2.0, 2.0);
    declare_ranged_double("range_min", m_cfg_range_min, 0.0, 10.0);
    declare_ranged_double("range_max", m_cfg_range_max, 0.0, 1000.0);
    declare_ranged_double("beam_width", m_cfg_beam_width_deg, 0.0, 90.0);
    declare_ranged_double("resolution", m_cfg_resolution, 0.0, 3.0);
    declare_ranged_int("n_cells", m_cfg_n_cells, 1, 10000);

    declare_ranged_int("n_samples", m_cfg_n_samples, 1, 10000);
    declare_ranged_int("beam_sample_dist", m_cfg_beam_sample_dist, 0, 3);
    declare_ranged_double("beam_sample_dist_normal_p_in_cone", m_cfg_beam_sample_dist_normal_p_in_cone, 0.0, 0.999);

    declare_ranged_int("n_reflections", m_cfg_n_reflections, 0, 20);

    declare_ranged_double("energy_min", m_cfg_energy_min, 0.0, 1.0);
    declare_ranged_double("energy_max", m_cfg_energy_max, 0.0, 1.0);
    declare_ranged_double("signal_max", m_cfg_signal_max, 0.0, 255.0);

    declare_ranged_int("signal_denoising", m_cfg_signal_denoising, 0, 3);
    declare_ranged_int("signal_denoising_triangular_width", m_cfg_signal_denoising_triangular_width, 1, 200);
    declare_ranged_double("signal_denoising_triangular_mode", m_cfg_signal_denoising_triangular_mode, 0.0, 1.0);
    declare_ranged_int("signal_denoising_gaussian_width", m_cfg_signal_denoising_gaussian_width, 1, 200);
    declare_ranged_double("signal_denoising_gaussian_mode", m_cfg_signal_denoising_gaussian_mode, 0.0, 1.0);
    declare_ranged_int("signal_denoising_mb_width", m_cfg_signal_denoising_mb_width, 1, 200);
    declare_ranged_double("signal_denoising_mb_mode", m_cfg_signal_denoising_mb_mode, 0.0, 1.0);

    declare_ranged_int("ambient_noise", m_cfg_ambient_noise, 0, 2);
    declare_ranged_double("ambient_noise_at_signal_0", m_cfg_ambient_noise_at_signal_0, 0.0, 1.0);
    declare_ranged_double("ambient_noise_at_signal_1", m_cfg_ambient_noise_at_signal_1, 0.0, 1.0);
    declare_ranged_double("ambient_noise_energy_max", m_cfg_ambient_noise_energy_max, 0.0, 1.0);
    declare_ranged_double("ambient_noise_energy_min", m_cfg_ambient_noise_energy_min, 0.0, 1.0);
    declare_ranged_double("ambient_noise_energy_loss", m_cfg_ambient_noise_energy_loss, 0.0, 1.0);
    declare_ranged_double("ambient_noise_uniform_max", m_cfg_ambient_noise_uniform_max, 0.0, 1.0);
    declare_ranged_double("ambient_noise_perlin_scale_low", m_cfg_ambient_noise_perlin_scale_low, 0.0, 1.0);
    declare_ranged_double("ambient_noise_perlin_scale_high", m_cfg_ambient_noise_perlin_scale_high, 0.0, 1.0);
    declare_ranged_double("ambient_noise_perlin_p_low", m_cfg_ambient_noise_perlin_p_low, 0.0, 1.0);

    declare_ranged_int("scroll_image", m_cfg_scroll_image, 0, 400);
    declare_ranged_double("multipath_threshold", m_cfg_multipath_threshold, 0.0, 1.0);
    m_node->declare_parameter("record_multi_reflection", m_cfg_record_multi_reflection);
    m_node->declare_parameter("record_multi_path", m_cfg_record_multi_path);
    m_node->declare_parameter("include_motion", m_cfg_include_motion);

    // materials (not reconfigurable, see loadParams()/ros_helper.h -- ROS 2
    // params have no native array-of-object type, so these are read from a
    // YAML file instead of the parameter server)
    m_node->declare_parameter("materials_file", std::string(""));
    m_node->declare_parameter("material_id_air", static_cast<int64_t>(m_material_id_air));
    m_node->declare_parameter("object_materials", std::vector<int64_t>{});

    m_param_callback_handle = m_node->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters)
        {
            return onSetParameters(parameters);
        });

    // declare_parameter(name, default) resolves any override from a
    // --params-file/-p at declaration time, but that resolved value isn't
    // reflected in the m_cfg_* members yet -- only onSetParameters() (fired
    // on a later explicit `ros2 param set`) updates them. Without this, an
    // initial override (e.g. n_cells in a params file) is silently ignored
    // and the hardcoded C++ default is used instead. Sync once, right after
    // declaring everything, by replaying the resolved values through the
    // same apply logic used for live updates.
    onSetParameters(m_node->get_parameters({
        "z_offset", "range_min", "range_max", "beam_width", "resolution", "n_cells",
        "n_samples", "beam_sample_dist", "beam_sample_dist_normal_p_in_cone",
        "n_reflections", "energy_min", "energy_max", "signal_max",
        "signal_denoising", "signal_denoising_triangular_width", "signal_denoising_triangular_mode",
        "signal_denoising_gaussian_width", "signal_denoising_gaussian_mode",
        "signal_denoising_mb_width", "signal_denoising_mb_mode",
        "ambient_noise", "ambient_noise_at_signal_0", "ambient_noise_at_signal_1",
        "ambient_noise_energy_max", "ambient_noise_energy_min", "ambient_noise_energy_loss",
        "ambient_noise_uniform_max", "ambient_noise_perlin_scale_low", "ambient_noise_perlin_scale_high",
        "ambient_noise_perlin_p_low", "scroll_image", "multipath_threshold",
        "record_multi_reflection", "record_multi_path", "include_motion"
    }));
}

rcl_interfaces::msg::SetParametersResult Radar::onSetParameters(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // Validate first, without mutating anything: a batch is applied
    // atomically, so one invalid value must not leave other, valid ones
    // from the same batch half-applied.
    for(const auto &param : parameters)
    {
        const auto &name = param.get_name();
        if(name == "n_cells" && param.as_int() < 1)
        {
            result.successful = false;
            result.reason = "n_cells must be >= 1";
        }
        else if(name == "n_samples" && param.as_int() < 1)
        {
            result.successful = false;
            result.reason = "n_samples must be >= 1";
        }
    }
    if(!result.successful)
    {
        return result;
    }

    bool resample = false;
    for(const auto &param : parameters)
    {
        const auto &name = param.get_name();
        if(name == "z_offset") { m_cfg_z_offset = param.as_double(); }
        else if(name == "range_min") { m_cfg_range_min = param.as_double(); }
        else if(name == "range_max") { m_cfg_range_max = param.as_double(); }
        else if(name == "beam_width")
        {
            if(std::abs(param.as_double() - m_cfg_beam_width_deg) > 0.001) { resample = true; }
            m_cfg_beam_width_deg = param.as_double();
        }
        else if(name == "resolution") { m_cfg_resolution = param.as_double(); }
        else if(name == "n_cells") { m_cfg_n_cells = static_cast<int>(param.as_int()); }
        else if(name == "n_samples")
        {
            if(param.as_int() != m_cfg_n_samples) { resample = true; }
            m_cfg_n_samples = static_cast<int>(param.as_int());
        }
        else if(name == "beam_sample_dist")
        {
            if(param.as_int() != m_cfg_beam_sample_dist) { resample = true; }
            m_cfg_beam_sample_dist = static_cast<int>(param.as_int());
        }
        else if(name == "beam_sample_dist_normal_p_in_cone")
        {
            if(std::abs(param.as_double() - m_cfg_beam_sample_dist_normal_p_in_cone) > 0.001) { resample = true; }
            m_cfg_beam_sample_dist_normal_p_in_cone = param.as_double();
        }
        else if(name == "n_reflections") { m_cfg_n_reflections = static_cast<int>(param.as_int()); }
        else if(name == "energy_min") { m_cfg_energy_min = param.as_double(); }
        else if(name == "energy_max") { m_cfg_energy_max = param.as_double(); }
        else if(name == "signal_max") { m_cfg_signal_max = param.as_double(); }
        else if(name == "signal_denoising") { m_cfg_signal_denoising = static_cast<int>(param.as_int()); }
        else if(name == "signal_denoising_triangular_width") { m_cfg_signal_denoising_triangular_width = static_cast<int>(param.as_int()); }
        else if(name == "signal_denoising_triangular_mode") { m_cfg_signal_denoising_triangular_mode = param.as_double(); }
        else if(name == "signal_denoising_gaussian_width") { m_cfg_signal_denoising_gaussian_width = static_cast<int>(param.as_int()); }
        else if(name == "signal_denoising_gaussian_mode") { m_cfg_signal_denoising_gaussian_mode = param.as_double(); }
        else if(name == "signal_denoising_mb_width") { m_cfg_signal_denoising_mb_width = static_cast<int>(param.as_int()); }
        else if(name == "signal_denoising_mb_mode") { m_cfg_signal_denoising_mb_mode = param.as_double(); }
        else if(name == "ambient_noise") { m_cfg_ambient_noise = static_cast<int>(param.as_int()); }
        else if(name == "ambient_noise_at_signal_0") { m_cfg_ambient_noise_at_signal_0 = param.as_double(); }
        else if(name == "ambient_noise_at_signal_1") { m_cfg_ambient_noise_at_signal_1 = param.as_double(); }
        else if(name == "ambient_noise_energy_max") { m_cfg_ambient_noise_energy_max = param.as_double(); }
        else if(name == "ambient_noise_energy_min") { m_cfg_ambient_noise_energy_min = param.as_double(); }
        else if(name == "ambient_noise_energy_loss") { m_cfg_ambient_noise_energy_loss = param.as_double(); }
        else if(name == "ambient_noise_uniform_max") { m_cfg_ambient_noise_uniform_max = param.as_double(); }
        else if(name == "ambient_noise_perlin_scale_low") { m_cfg_ambient_noise_perlin_scale_low = param.as_double(); }
        else if(name == "ambient_noise_perlin_scale_high") { m_cfg_ambient_noise_perlin_scale_high = param.as_double(); }
        else if(name == "ambient_noise_perlin_p_low") { m_cfg_ambient_noise_perlin_p_low = param.as_double(); }
        else if(name == "scroll_image") { m_cfg_scroll_image = static_cast<int>(param.as_int()); }
        else if(name == "multipath_threshold") { m_cfg_multipath_threshold = param.as_double(); }
        else if(name == "record_multi_reflection") { m_cfg_record_multi_reflection = param.as_bool(); }
        else if(name == "record_multi_path") { m_cfg_record_multi_path = param.as_bool(); }
        else if(name == "include_motion") { m_cfg_include_motion = param.as_bool(); }
    }

    if(resample)
    {
        m_resample = true;
    }

    // update radar model
    m_radar_model.range.min = m_cfg_range_min;
    m_radar_model.range.max = m_cfg_range_max;

    // update params model
    m_params.model.beam_width = m_cfg_beam_width_deg * M_PI / 180.0;
    m_params.model.n_samples = m_cfg_n_samples;
    m_params.model.n_reflections = m_cfg_n_reflections;

    return result;
}

void Radar::loadParams()
{
    // material properties
    const std::string materials_file = m_node->get_parameter("materials_file").as_string();
    m_params.materials = loadRadarMaterialsFromFile(materials_file);
    m_object_materials = m_node->get_parameter("object_materials").as_integer_array();
    m_material_id_air = static_cast<int>(m_node->get_parameter("material_id_air").as_int());
}


} // namespace radarays_ros
