#include "radarays_ros/Radar.hpp"

#include <radarays_ros/ros_helper.h>
#include <rmagine/util/StopWatch.hpp>
#include <rmagine/math/types.h>

namespace radarays_ros
{

Radar::Radar(
    std::shared_ptr<ros::NodeHandle> nh_p,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<tf2_ros::TransformListener> tf_listener,
    std::string map_frame,
    std::string sensor_frame)
:m_nh_p(nh_p)
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

    loadParams();

    dynamic_reconfigure::Server<RadarModelConfig>::CallbackType f;
    f = boost::bind(&Radar::updateDynCfg, this, _1, _2);
    m_dyn_rec_server.setCallback(f);
}

std::optional<rm::Transform> Radar::getTsm()
{
    return getTsm(ros::Time(0));
}

std::optional<rm::Transform> Radar::getTsm(ros::Time stamp)
{
    rm::Transform Tsm;

    try {
        geometry_msgs::TransformStamped Tsm_ros = m_tf_buffer->lookupTransform(
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
bool Radar::updateTsm()
{
    std::optional<rm::Transform> Tsm_opt;
    ros::Time Tsm_stamp;

    try {
        rm::Transform Tsm;
        geometry_msgs::TransformStamped Tsm_ros = m_tf_buffer->lookupTransform(
            m_map_frame,
            m_sensor_frame,
            ros::Time(0));

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

bool Radar::updateTsm(ros::Time stamp)
{
    std::optional<rm::Transform> Tsm_opt;
    ros::Time Tsm_stamp;

    try {
        rm::Transform Tsm;
        geometry_msgs::TransformStamped Tsm_ros = m_tf_buffer->lookupTransform(
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

void Radar::updateDynCfg(
    RadarModelConfig &config,
    uint32_t level)
{
    ROS_INFO("Changing Model");
    
    // z_offset
    // auto T = rm::Transform::Identity();
    // T.t.z = config.z_offset;
    // sim->setTsb(T);

    if(   config.beam_sample_dist != m_cfg.beam_sample_dist 
        || abs(config.beam_width - m_cfg.beam_width ) > 0.001
        || config.n_samples != m_cfg.n_samples
        || abs(config.beam_sample_dist_normal_p_in_cone - m_cfg.beam_sample_dist_normal_p_in_cone) > 0.001
    )
    {
        m_resample = true;
    }

    // update radar model
    m_radar_model.range.min = config.range_min;
    m_radar_model.range.max = config.range_max;

    // update params model
    m_params.model.beam_width = config.beam_width * M_PI / 180.0;
    m_params.model.n_samples = config.n_samples;
    m_params.model.n_reflections = config.n_reflections;

    m_cfg = config;
}

void Radar::loadParams()
{
    // material properties
    m_params.materials = loadRadarMaterialsFromParameterServer(m_nh_p);
    m_nh_p->getParam("object_materials", m_object_materials);
    m_nh_p->getParam("material_id_air", m_material_id_air);
}


} // namespace radarays