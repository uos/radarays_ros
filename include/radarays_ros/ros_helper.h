#ifndef RADARAYS_ROS_ROS_HELPER_H
#define RADARAYS_ROS_ROS_HELPER_H

#include <radarays_ros/msg/radar_params.hpp>
#include <cmath>
#include <string>

// ROS 2 parameters have no native array-of-object type (ROS 1's
// dynamic-typed XmlRpcValue nested arrays-of-dicts don't have an
// equivalent), so the materials list -- an array of {velocity, ambient,
// diffuse, specular} -- is read from a plain YAML file instead of the
// parameter server. material_id_air and object_materials are flat
// arrays/scalars and stay real ROS 2 parameters (see Radar::loadParams()).
radarays_ros::msg::RadarMaterials loadRadarMaterialsFromFile(
    const std::string &materials_file);

namespace radarays_ros
{

// parameters
static msg::RadarModel default_params_model()
{
    msg::RadarModel model;
    model.beam_width = 8.0 * M_PI / 180.0;
    model.n_samples = 200;
    model.n_reflections = 2;
    return model;
}

static msg::RadarParams default_params()
{
    msg::RadarParams ret;
    ret.model = default_params_model();
    return ret;
}

} // namespace radarays_ros




#endif // RADARAYS_ROS_ROS_HELPER_H
