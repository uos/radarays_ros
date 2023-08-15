#ifndef RADARAYS_RADAR_HPP
#define RADARAYS_RADAR_HPP

#include <ros/ros.h>

#include <memory>
#include <optional>
#include <unordered_map>

#include <tf2_ros/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <radarays_ros/RadarModelConfig.h>
#include <radarays_ros/RadarParams.h>
#include <radarays_ros/radar_types.h>

#include <radarays_ros/RadarMaterials.h>

#include <rmagine/types/sensor_models.h>

#include <sensor_msgs/Image.h>

#include <dynamic_reconfigure/server.h>


namespace rm = rmagine;

namespace radarays_ros
{

/**
 * Abstract class for different radar implementations
*/
class Radar {
public:
    Radar(
        std::shared_ptr<ros::NodeHandle> nh_p,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        std::shared_ptr<tf2_ros::TransformListener> tf_listener,
        std::string map_frame, 
        std::string sensor_frame);
    
    void loadParams();

    std::optional<rm::Transform> getTsm();
    std::optional<rm::Transform> getTsm(ros::Time stamp);

    bool updateTsm();
    bool updateTsm(ros::Time stamp);

    inline RadarParams getParams()
    {
        return m_params;
    }

    inline void setParams(RadarParams params)
    {
        m_params = params;
    }

    /**
     * Implement this function
    */
    virtual sensor_msgs::ImagePtr simulate(ros::Time stamp) = 0;

protected:

    void updateDynCfg(RadarModelConfig &config, uint32_t level);

    // ROS NODE
    std::shared_ptr<ros::NodeHandle> m_nh_p;

    // TF
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

    // DynRec
    dynamic_reconfigure::Server<RadarModelConfig> m_dyn_rec_server;


    rm::Transform Tsm_last = rm::Transform::Identity();
    bool has_last;
    ros::Time Tsm_stamp_last;
    ros::Time stamp_last;

    // MAP
    std::string m_map_frame;
    std::string m_sensor_frame;

    // Params
    RadarParams m_params;
    rm::SphericalModel m_radar_model;
    radarays_ros::RadarModelConfig m_cfg;

    // materials
    int m_material_id_air = 0;
    std::vector<int> m_object_materials;

    float m_wave_energy_threshold = 0.001;
    std::vector<DirectedWave> m_waves_start;
    bool m_resample = true;
    float m_max_signal = 120.0;

    // PUT TO IMPL?
    cv::Mat m_polar_image;

};

using RadarPtr = std::shared_ptr<Radar>;

} // namespace radarays


#endif // RADARAYS_RADAR_HPP