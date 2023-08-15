#include <radarays_ros/ros_helper.h>

template<>
radarays_ros::RadarMaterial loadFromRPC<radarays_ros::RadarMaterial>(
    XmlRpc::XmlRpcValue material_xml)
{
    radarays_ros::RadarMaterial ret;
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


radarays_ros::RadarMaterials loadRadarMaterialsFromParameterServer(
    std::shared_ptr<ros::NodeHandle> nh)
{
    radarays_ros::RadarMaterials ret;
    
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
            auto material = loadFromRPC<radarays_ros::RadarMaterial>(material_xml);
            ret.data.push_back(material);
        }
    }

    return ret;
}