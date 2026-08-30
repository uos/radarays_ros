#ifndef RADARAYS_RADAR_HPP
#define RADARAYS_RADAR_HPP

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <optional>
#include <unordered_map>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <opencv2/core.hpp>
#include <radarays_ros/msg/radar_params.hpp>

#include <radarays_ros/radar_types.h>

#include <rmagine/types/sensor_models.h>

#include <sensor_msgs/msg/image.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>


namespace rm = rmagine;

namespace radarays_ros
{

/**
 * Abstract class for different radar implementations
*/
class Radar {
public:
    Radar(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        std::shared_ptr<tf2_ros::TransformListener> tf_listener,
        std::string map_frame,
        std::string sensor_frame);

    void loadParams();

    std::optional<rm::Transform> getTsm();
    std::optional<rm::Transform> getTsm(rclcpp::Time stamp);

    bool updateTsm();
    bool updateTsm(rclcpp::Time stamp);

    inline msg::RadarParams getParams()
    {
        return m_params;
    }

    inline void setParams(msg::RadarParams params)
    {
        m_params = params;
    }

    /**
     * Implement this function
    */
    virtual sensor_msgs::msg::Image::SharedPtr simulate(rclcpp::Time stamp) = 0;

protected:

    // Live tuning of the radar-physics parameters, the ROS 2 equivalent of
    // Classic's dynamic_reconfigure/rqt_reconfigure (RadarModelConfig, see
    // cfg/RadarModel.cfg). Mirrors the pattern already proven in
    // radarays_gazebo_plugins's radarays_embree_sensor_system.
    void declareReconfigurableParams();
    rcl_interfaces::msg::SetParametersResult onSetParameters(
        const std::vector<rclcpp::Parameter> &parameters);

    // ROS NODE
    rclcpp::Node::SharedPtr m_node;

    // TF
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_param_callback_handle;

    rm::Transform Tsm_last = rm::Transform::Identity();
    bool has_last = false;
    rclcpp::Time Tsm_stamp_last;
    rclcpp::Time stamp_last;

    // MAP
    std::string m_map_frame;
    std::string m_sensor_frame;

    // Params
    msg::RadarParams m_params;
    rm::SphericalModel m_radar_model;

    // -- dynamic reconfigure equivalent: RadarModel.cfg fields --
    double m_cfg_z_offset = 0.0;
    double m_cfg_range_min = 0.0;
    double m_cfg_range_max = 600.0;
    double m_cfg_beam_width_deg = 8.0;
    double m_cfg_resolution = 0.0438;
    int m_cfg_n_cells = 3424;

    int m_cfg_n_samples = 10;
    int m_cfg_beam_sample_dist = 2;
    double m_cfg_beam_sample_dist_normal_p_in_cone = 0.8;

    int m_cfg_n_reflections = 4;

    double m_cfg_energy_min = 0.0;
    double m_cfg_energy_max = 0.5;
    double m_cfg_signal_max = 120.0;

    // 0 = none, 1 = triangular, 2 = gaussian, 3 = maxwell-boltzmann
    int m_cfg_signal_denoising = 1;
    int m_cfg_signal_denoising_triangular_width = 50;
    double m_cfg_signal_denoising_triangular_mode = 0.35;
    int m_cfg_signal_denoising_gaussian_width = 50;
    double m_cfg_signal_denoising_gaussian_mode = 0.5;
    int m_cfg_signal_denoising_mb_width = 50;
    double m_cfg_signal_denoising_mb_mode = 0.4;

    // 0 = none, 1 = uniform, 2 = perlin
    int m_cfg_ambient_noise = 2;
    double m_cfg_ambient_noise_at_signal_0 = 0.3;
    double m_cfg_ambient_noise_at_signal_1 = 0.03;
    double m_cfg_ambient_noise_energy_max = 0.5;
    double m_cfg_ambient_noise_energy_min = 0.1;
    double m_cfg_ambient_noise_energy_loss = 0.05;
    double m_cfg_ambient_noise_uniform_max = 0.15;
    double m_cfg_ambient_noise_perlin_scale_low = 0.05;
    double m_cfg_ambient_noise_perlin_scale_high = 0.2;
    double m_cfg_ambient_noise_perlin_p_low = 0.9;

    int m_cfg_scroll_image = 0;
    double m_cfg_multipath_threshold = 0.5;
    bool m_cfg_record_multi_reflection = true;
    bool m_cfg_record_multi_path = false;
    bool m_cfg_include_motion = true;

    // materials
    int m_material_id_air = 0;
    std::vector<int64_t> m_object_materials;

    float m_wave_energy_threshold = 0.001;
    std::vector<DirectedWave> m_waves_start;
    bool m_resample = true;
    float m_max_signal = 120.0;

    // PUT TO IMPL?
    cv::Mat m_polar_image;

};

using RadarPtr = std::shared_ptr<Radar>;

} // namespace radarays_ros


#endif // RADARAYS_RADAR_HPP
