// ROS 2 port of the ROS 1 mesh_publisher, deliberately NOT a 1:1 message
// port. The original published mesh_msgs/MeshGeometryStamped, meant to be
// viewed via RViz1's rviz_map_plugin/mesh_tools -- neither mesh_msgs nor
// that plugin has a ROS 2 release (see MIGRATION.md, "mesh_publisher.cpp").
// This publishes the same mesh data as a visualization_msgs/MarkerArray
// (TRIANGLE_LIST per mesh) instead -- RViz2 renders that natively, no
// extra plugin or external dependency needed, using the same pattern
// ray_reflection_test.cpp already uses for its own Marker publishing.
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/header.hpp>

#include <rmagine/map/EmbreeMap.hpp>

using namespace rmagine;
namespace rm = rmagine;

namespace
{

rm::Transform pre_transform = rm::Transform::Identity();

void appendMeshTriangles(
    visualization_msgs::msg::Marker& marker,
    EmbreeMeshPtr mesh,
    const rm::Matrix4x4& T)
{
    auto vertices = mesh->verticesTransformed();
    auto faces = mesh->faces();

    for(size_t i = 0; i < faces.size(); i++)
    {
        const auto& face = faces[i];
        for(unsigned int v : {face.v0, face.v1, face.v2})
        {
            auto vt = T * vertices[v];
            geometry_msgs::msg::Point p;
            p.x = vt.x;
            p.y = vt.y;
            p.z = vt.z;
            marker.points.push_back(p);
        }
    }
}

// Walks the scene graph the same way the ROS 1 original did (nested
// EmbreeInstances compose their transform into their children), appending
// one Marker per leaf mesh to `markers`.
void collectMarkers(
    EmbreeScenePtr scene,
    const rm::Matrix4x4& T,
    const std_msgs::msg::Header& header,
    visualization_msgs::msg::MarkerArray& markers)
{
    const rm::Matrix4x4 pre_transform_matrix = rm::compose(pre_transform, rm::Vector3{1.0, 1.0, 1.0});

    for(auto elem : scene->geometries())
    {
        EmbreeInstancePtr inst = std::dynamic_pointer_cast<EmbreeInstance>(elem.second);
        if(inst)
        {
            collectMarkers(inst->scene(), T * pre_transform_matrix * inst->matrix(), header, markers);
            continue;
        }

        EmbreeMeshPtr mesh = std::dynamic_pointer_cast<EmbreeMesh>(elem.second);
        if(!mesh)
        {
            continue;
        }

        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "mesh";
        marker.id = static_cast<int>(elem.first);
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.r = 0.6;
        marker.color.g = 0.6;
        marker.color.b = 0.65;
        marker.color.a = 0.9;

        appendMeshTriangles(marker, mesh, T * pre_transform_matrix);
        markers.markers.push_back(marker);
    }
}

} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mesh_publisher");

    node->declare_parameter("map_file", std::string(""));
    node->declare_parameter("map_frame", std::string("map"));
    node->declare_parameter("publish_rate", 0.1);
    node->declare_parameter("pre_transform", std::vector<double>{});

    const std::string map_file = node->get_parameter("map_file").as_string();
    const std::string map_frame = node->get_parameter("map_frame").as_string();
    const double publish_rate = node->get_parameter("publish_rate").as_double();

    if(map_file.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "map_file parameter is required.");
        rclcpp::shutdown();
        return 1;
    }

    const auto transform_params = node->get_parameter("pre_transform").as_double_array();
    if(transform_params.size() == 6)
    {
        pre_transform.t = rm::Vector{
            (float)transform_params[0], (float)transform_params[1], (float)transform_params[2]};
        pre_transform.R = rm::EulerAngles{
            (float)transform_params[3], (float)transform_params[4], (float)transform_params[5]};
    } else if(transform_params.size() == 7) {
        pre_transform.t = rm::Vector{
            (float)transform_params[0], (float)transform_params[1], (float)transform_params[2]};
        pre_transform.R = rm::Quaternion{
            (float)transform_params[3], (float)transform_params[4],
            (float)transform_params[5], (float)transform_params[6]};
    }

    auto map = rm::import_embree_map(map_file);

    auto pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("mesh_markers", 1);

    RCLCPP_INFO(node->get_logger(), "Publishing '%s' as a MarkerArray on 'mesh_markers' at %.3f Hz.",
        map_file.c_str(), publish_rate);

    rclcpp::Rate r(publish_rate);
    while(rclcpp::ok())
    {
        std_msgs::msg::Header header;
        header.stamp = node->get_clock()->now();
        header.frame_id = map_frame;

        visualization_msgs::msg::MarkerArray markers;
        collectMarkers(map->scene, rm::Matrix4x4::Identity(), header, markers);
        pub->publish(markers);

        r.sleep();
        rclcpp::spin_some(node);
    }

    rclcpp::shutdown();
    return 0;
}
