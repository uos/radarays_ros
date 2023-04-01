#include <ros/ros.h>

#include <mesh_msgs/MeshGeometry.h>
#include <mesh_msgs/MeshGeometryStamped.h>
#include <mesh_msgs/TriangleIndices.h>

#include <rmagine/map/EmbreeMap.hpp>

using namespace rmagine;

namespace rm = rmagine;

rm::Transform pre_transform;

mesh_msgs::MeshGeometry embreeToRos(EmbreeMeshPtr mesh, Matrix4x4 T)
{
    mesh_msgs::MeshGeometry mesh_ros;

    // Vertices
    auto vertices = mesh->verticesTransformed();
    for(int i=0; i<vertices.size(); i++)
    {
        geometry_msgs::Point vertex_ros;
        // mesh->ver
        auto vt = T * vertices[i];
        vertex_ros.x = vt.x;
        vertex_ros.y = vt.y;
        vertex_ros.z = vt.z;
        mesh_ros.vertices.push_back(vertex_ros);
    }

    // Faces
    auto faces = mesh->faces();
    for(int i=0; i<faces.size(); i++)
    {
        mesh_msgs::TriangleIndices face_ros;
        face_ros.vertex_indices[0] = faces[i].v0;
        face_ros.vertex_indices[1] = faces[i].v1;
        face_ros.vertex_indices[2] = faces[i].v2;
        mesh_ros.faces.push_back(face_ros);
    }

    return mesh_ros;
}

std::unordered_map<unsigned int, mesh_msgs::MeshGeometry> embreeToRos(
    EmbreeScenePtr scene, 
    Matrix4x4 T = Matrix4x4::Identity())
{
    std::unordered_map<unsigned int, mesh_msgs::MeshGeometry> ret;

    for(auto elem : scene->geometries())
    {
        size_t geom_id = elem.first;
        EmbreeInstancePtr inst = std::dynamic_pointer_cast<EmbreeInstance>(elem.second);
        if(inst)
        {
            rm::Matrix4x4 M = rm::compose(pre_transform, rm::Vector3{1.0, 1.0, 1.0});
            Matrix4x4 T_ = T * M * inst->matrix();
            std::cout << "instance: " << inst->name << std::endl;

            auto ret_ = embreeToRos(inst->scene(), T_);
            ret.insert(ret_.begin(), ret_.end());
        } else {
            EmbreeMeshPtr mesh = std::dynamic_pointer_cast<EmbreeMesh>(elem.second);
            
            if(mesh)
            {
                unsigned int mesh_id = mesh->id(scene);
                std::cout << "mesh " << mesh_id << ": " << mesh->name << std::endl;
                // leaf
                // rm::Matrix4x4 pre_transform_matrix = (rm::Matrix4x4)pre_transform;
                rm::Matrix4x4 M = rm::compose(pre_transform, rm::Vector3{1.0, 1.0, 1.0});
                ret[mesh_id] = embreeToRos(mesh, T * M);
            }
        }
    }

    return ret;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mesh_publisher");
    ros::NodeHandle n;
    ros::NodeHandle nh_p("~");

    std::string map_frame;
    std::string meshfile;

    double publish_freq;
    nh_p.param<std::string>("file", meshfile, "/home/amock/ros_workspaces/amcl_flex/avz_floor.ply");
    nh_p.param<std::string>("frame", map_frame, "map");
    nh_p.param<double>("publish_freq", publish_freq, 0.1);

    pre_transform = rm::Transform::Identity();
    std::vector<double> transform_params;
    if(nh_p.getParam("pre_transform", transform_params))
    {
        if(transform_params.size() == 6)
        {
            pre_transform.t = rm::Vector{
                    (float)transform_params[0], 
                    (float)transform_params[1], 
                    (float)transform_params[2]};
            pre_transform.R = rm::EulerAngles{
                    (float)transform_params[3],
                    (float)transform_params[4],
                    (float)transform_params[5]};
        } else if(transform_params.size() == 7) {
            pre_transform.t = rm::Vector{
                    (float)transform_params[0],
                    (float)transform_params[1],
                    (float)transform_params[2]};
            pre_transform.R = rm::Quaternion{
                    (float)transform_params[3],
                    (float)transform_params[4],
                    (float)transform_params[5],
                    (float)transform_params[6]
            };
        }
    }


    auto map = import_embree_map(meshfile);

    auto meshes_ros = embreeToRos(map->scene);

    std::unordered_map<unsigned int, mesh_msgs::MeshGeometryStamped> meshes_stamped;

    for(auto elem : meshes_ros)
    {
        std::stringstream ss;
        ss << "mesh/" << elem.first;
        mesh_msgs::MeshGeometryStamped mesh_stamped;
        mesh_stamped.mesh_geometry = elem.second;
        mesh_stamped.header.frame_id = map_frame;
        mesh_stamped.uuid = ss.str();
        meshes_stamped[elem.first] = mesh_stamped;
    }

    std::unordered_map<unsigned int, std::shared_ptr<ros::Publisher> > mesh_pubs;

    for(auto elem : meshes_stamped)
    {
        mesh_pubs[elem.first] = std::make_shared<ros::Publisher>(
            nh_p.advertise<mesh_msgs::MeshGeometryStamped>(elem.second.uuid, 10)
        );
        std::cout << elem.first << " - " << elem.second.mesh_geometry.vertices.size() << "v, " 
                    << elem.second.mesh_geometry.faces.size() << "f" << std::endl; 
    }

    ros::Rate r(publish_freq);

    std::cout << "Publishing Meshes (" << meshes_stamped.size() << ") at " << publish_freq << "hz ..." << std::endl;
    while(ros::ok())
    {
        for(size_t i=0; i<meshes_stamped.size(); i++)
        {
            auto mesh = meshes_stamped[i];
            auto pub = mesh_pubs[i];
            mesh.header.stamp = ros::Time::now();
            pub->publish(mesh);
        }
        
        r.sleep();
        ros::spinOnce();
    }

    return 0;
}
