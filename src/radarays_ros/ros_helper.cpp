#include <radarays_ros/ros_helper.h>

#include <yaml-cpp/yaml.h>
#include <iostream>

namespace
{

radarays_ros::msg::RadarMaterial materialFromYaml(const YAML::Node &node)
{
    radarays_ros::msg::RadarMaterial ret;
    ret.velocity = node["velocity"] ? node["velocity"].as<float>() : 0.0f;
    ret.ambient  = node["ambient"]  ? node["ambient"].as<float>()  : 0.0f;
    ret.diffuse  = node["diffuse"]  ? node["diffuse"].as<float>()  : 0.0f;
    ret.specular = node["specular"] ? node["specular"].as<float>() : 0.0f;
    return ret;
}

} // namespace

radarays_ros::msg::RadarMaterials loadRadarMaterialsFromFile(
    const std::string &materials_file)
{
    radarays_ros::msg::RadarMaterials ret;

    if(materials_file.empty())
    {
        return ret;
    }

    YAML::Node root;
    try
    {
        root = YAML::LoadFile(materials_file);
    }
    catch(const std::exception &e)
    {
        std::cerr << "[radarays_ros] Failed to load materials_file '" << materials_file
                  << "': " << e.what() << std::endl;
        return ret;
    }

    YAML::Node materials_node = root["materials"];
    if(!materials_node || !materials_node.IsSequence())
    {
        std::cerr << "[radarays_ros] materials_file '" << materials_file
                  << "' has no 'materials' sequence." << std::endl;
        return ret;
    }

    for(const auto &material_node : materials_node)
    {
        ret.data.push_back(materialFromYaml(material_node));
    }

    return ret;
}
