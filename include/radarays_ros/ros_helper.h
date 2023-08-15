#ifndef RADARAYS_ROS_ROS_HELPER_H
#define RADARAYS_ROS_ROS_HELPER_H

#include <radarays_ros/RadarParams.h>
#include <ros/ros.h>

template<typename T>
T loadFromRPC(XmlRpc::XmlRpcValue);

template<>
radarays_ros::RadarMaterial loadFromRPC<radarays_ros::RadarMaterial>(
    XmlRpc::XmlRpcValue material_xml);

radarays_ros::RadarMaterials loadRadarMaterialsFromParameterServer(
    std::shared_ptr<ros::NodeHandle> nh);    

namespace radarays_ros
{

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

} // namespace radarays_ros




#endif // RADARAYS_ROS_ROS_HELPER_H