#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <rmagine/simulation/OnDnSimulatorEmbree.hpp>
#include <rmagine/simulation/OnDnSimulatorOptix.hpp>
#include <rmagine/util/prints.h>
#include <rmagine/util/StopWatch.hpp>
#include <random>

#include <tf2_ros/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <radarays_ros/RadarModelConfig.h>

#include <sensor_msgs/PointCloud.h>

#include <radarays_ros/radar_types.h>

#include <radarays_ros/radar_math.h>
#include <radarays_ros/image_algorithms.h>
#include <radarays_ros/radar_algorithms.h>
#include <radarays_ros/image_algorithms.cuh>
#include <radarays_ros/radar_algorithms.cuh>


#include <radarays_ros/RadarParams.h>

#include <actionlib/server/simple_action_server.h>
#include <radarays_ros/GenRadarImageAction.h>


#include <radarays_ros/GetRadarParams.h>



#include <opencv2/highgui.hpp>

#include <omp.h>

#include <radarays_ros/ros_helper.h>

#include <radarays_ros/RadarCPU.hpp>
#include <radarays_ros/RadarGPU.hpp>



using namespace radarays_ros;

namespace rm = rmagine;


std::string map_frame = "map";
std::string sensor_frame = "navtech";

std::shared_ptr<ros::NodeHandle> nh_p;
std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;
ros::Publisher pub_pcl;
image_transport::Publisher pub_polar;

RadarPtr radarays_sim;





bool getRadarParamsCB(radarays_ros::GetRadarParams::Request  &req,
         radarays_ros::GetRadarParams::Response &res)
{
    if(radarays_sim)
    {
        res.params = radarays_sim->getParams();
        return true;
    } else {
        return false;
    }
    return true;
}

// std::shared_ptr<actionlib::SimpleActionServer<radarays_ros::GenRadarImageAction> > as_;
// radarays_ros::GenRadarImageFeedback feedback_;
// radarays_ros::GenRadarImageResult result_;

// void executeCB(const radarays_ros::GenRadarImageGoalConstPtr &goal)
// {
//     std::cout << "CALL ACTION" << std::endl;

//     params = goal->params;
    
//     simulateImageGPU();

//     std::cout << polar_image.size() << std::endl;

//     result_.polar_image = 
//         *cv_bridge::CvImage(
//             std_msgs::Header(), 
//             "mono8",
//             polar_image).toImageMsg();

//     result_.polar_image.header.stamp = ros::Time::now();
//     as_->setSucceeded(result_);
// }

// int main_action_server(int argc, char** argv)
// {
//     std::cout << "STARTING RADAR SIMULATOR - ACTION SERVER" << std::endl;
//     ros::init(argc, argv, "radar_simulator");

//     ros::NodeHandle nh;
//     nh_p = std::make_shared<ros::NodeHandle>("~");

//     std::string map_file;
//     nh_p->getParam("map_file", map_file);
//     nh_p->getParam("map_frame", map_frame);
//     nh_p->getParam("sensor_frame", sensor_frame);

//     loadParameters();

//     std::cout << "SPEED IN AIR: " << params.materials.data[material_id_air].velocity << std::endl;

//     map = rm::import_embree_map(map_file);

//     // offset of sensor center to frame
//     auto Tsb = rm::Transform::Identity();
//     Tsb.t.z = 0.0;

//     // sim->setTsb(Tsb);

//     // n_cells = 3424;
//     // n_cells = 1712;
//     radar_model.theta.inc = -(2 * M_PI) / 400;
//     radar_model.theta.min = 0.0;
//     radar_model.theta.size = 400;
//     radar_model.phi.inc = 1.0;
//     radar_model.phi.min = 0.0;
//     radar_model.phi.size = 1;

//     // setting up dynamic reconfigure
//     dynamic_reconfigure::Server<RadarModelConfig> server;
//     dynamic_reconfigure::Server<RadarModelConfig>::CallbackType f;
//     f = boost::bind(&modelCB, _1, _2);
//     server.setCallback(f);

//     // setting up tf
//     // tf_buffer = std::make_shared<tf2_ros::Buffer>();
//     // tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

//     polar_image = cv::Mat_<unsigned char>(0, radar_model.theta.size);

//     // Start service server
//     ros::ServiceServer service = nh_p->advertiseService("get_radar_params", getRadarParamsCB);
    
//     // Start action server
//     std::string action_name = "gen_radar_image";
//     as_ = std::make_shared<actionlib::SimpleActionServer<radarays_ros::GenRadarImageAction> >(
//         *nh_p, action_name, executeCB, false
//     );

//     as_->start();

//     ros::Rate r(100);
//     ros::Time tp = ros::Time::now();

//     while(ros::ok())
//     {
//         r.sleep();
//         ros::spinOnce();
//         auto tc = ros::Time::now();
//         if(tc < tp)
//         {
//             // jump back in time
//             std::cout << "Jump back in time detected" << std::endl;
//             ros::Duration(2.0).sleep();
//             as_->shutdown();
//             as_ = std::make_shared<actionlib::SimpleActionServer<radarays_ros::GenRadarImageAction> >(
//                 *nh_p, action_name, executeCB, false
//             );
//             as_->start();
//             ros::Duration(2.0).sleep();
//         }
//         tp = tc;
//     }
//     return 0;
// }

void sync_cb(const sensor_msgs::Image::ConstPtr& sync_msg)
{
    radarays_sim->loadParams();
    sensor_msgs::ImagePtr msg = radarays_sim->simulate(sync_msg->header.stamp);

    if(!msg)
    {
        ROS_INFO_STREAM("SYNC: " << sync_msg->header.stamp << " -- " << " None");
        return;
    }
    
    ROS_INFO_STREAM("SYNC: " << sync_msg->header.stamp << " -- " << msg->header.stamp << ", SYN ERR: " << (sync_msg->header.stamp - msg->header.stamp).toSec() * 1000.0 << "ms" );
    pub_polar.publish(msg);
}

int main_publisher(int argc, char** argv)
{
    ros::init(argc, argv, "radar_simulator");

    ros::NodeHandle nh;
    nh_p = std::make_shared<ros::NodeHandle>("~");


    // setting up tf
    tf_buffer = std::make_shared<tf2_ros::Buffer>();
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);


    
    std::string map_file;
    nh_p->getParam("map_file", map_file);
    nh_p->getParam("map_frame", map_frame);
    nh_p->getParam("sensor_frame", sensor_frame);

    bool use_gpu = false;
    
    if(!use_gpu)
    {
        // CPU
        rm::EmbreeMapPtr map_cpu = rm::import_embree_map(map_file);

        std::cout << "RadarCPU" << std::endl;
        radarays_sim = std::make_shared<RadarCPU>(
            nh_p,
            tf_buffer,
            tf_listener,
            map_frame,
            sensor_frame,
            map_cpu
        );
    } else { 
        // GPU
        rm::OptixMapPtr map_gpu = rm::import_optix_map(map_file);

        std::cout << "RadarGPU" << std::endl;
        radarays_sim = std::make_shared<RadarGPU>(
            nh_p,
            tf_buffer,
            tf_listener,
            map_frame,
            sensor_frame,
            map_gpu
        );
    }

    // image transport
    image_transport::ImageTransport it(nh);
    pub_polar = it.advertise("radar/image", 1);

    // pcl
    pub_pcl = nh_p->advertise<sensor_msgs::PointCloud>("radar/pcl_real", 1);



    std::string sync_topic = "";

    if(nh_p->getParam("sync_topic", sync_topic))
    {
        std::cout << "SYNC SIMULATIONS WITH TOPIC " << sync_topic << std::endl;
        ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>(sync_topic, 1, sync_cb);
        ros::spin();

    } else {
        // unsynced
        ros::Rate r(100);
        while(nh.ok())
        {
            radarays_sim->loadParams();
            sensor_msgs::ImagePtr msg = radarays_sim->simulate(ros::Time(0));

            if(msg)
            {
                pub_polar.publish(msg);
            } else {
                std::cout << "MESSAGE EMPTY" << std::endl;
            }

            ros::spinOnce();
            r.sleep();
        }
    }
    


    
    return 0;
}



int main(int argc, char** argv)
{
    // return main_action_server(argc, argv);
    return main_publisher(argc, argv);
}
