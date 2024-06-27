#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>







#include <rmagine/util/prints.h>
#include <rmagine/util/StopWatch.hpp>
#include <random>

#include <tf2_ros/transform_listener.h>

#include <dynamic_reconfigure/server.h>
#include <radarays_ros/RadarModelConfig.h>

#include <sensor_msgs/PointCloud.h>

#include <radarays_ros/radar_types.h>
#include <radarays_ros/radar_math.h>
#include <radarays_ros/RadarParams.h>
#include <actionlib/server/simple_action_server.h>
#include <radarays_ros/GenRadarImageAction.h>

#include <radarays_ros/GetRadarParams.h>

#include <opencv2/highgui.hpp>

#include <omp.h>

#include <radarays_ros/ros_helper.h>


#include <radarays_ros/image_algorithms.h>
#include <radarays_ros/radar_algorithms.h>

#if defined RADARAYS_WITH_CPU
#include <rmagine/simulation/OnDnSimulatorEmbree.hpp>
#include <radarays_ros/RadarCPU.hpp>
#endif // defined RADARAYS_WITH_CPU

#if defined RADARAYS_WITH_GPU
#include <rmagine/simulation/OnDnSimulatorOptix.hpp>
#include <radarays_ros/image_algorithms.cuh>
#include <radarays_ros/radar_algorithms.cuh>
#include <radarays_ros/RadarGPU.hpp>
#endif // defined RADARAYS_WITH_GPU


using namespace radarays_ros;

namespace rm = rmagine;


std::string map_frame = "map";
std::string sensor_frame = "navtech";

std::shared_ptr<ros::NodeHandle> nh;
std::shared_ptr<ros::NodeHandle> nh_p;

std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;
ros::Publisher pub_pcl;
image_transport::Publisher pub_polar;
std::shared_ptr<image_transport::ImageTransport> it;


// Specific for action server
std::shared_ptr<actionlib::SimpleActionServer<GenRadarImageAction> > as_;
GenRadarImageFeedback feedback_;
GenRadarImageResult result_;



RadarPtr radarays_sim;




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

void init_sim()
{   
    // setting up tf
    tf_buffer = std::make_shared<tf2_ros::Buffer>();
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    std::string map_file;
    nh_p->getParam("map_file", map_file);
    nh_p->getParam("map_frame", map_frame);
    nh_p->getParam("sensor_frame", sensor_frame);

    // desired computing unit
    bool use_gpu = false;
    nh_p->param<bool>("gpu", use_gpu, false);

    // available computing unit
    bool gpu_available = false;
    bool cpu_available = false;
    #if defined RADARAYS_WITH_GPU
        gpu_available = true;
    #endif // defined RADARAYS_WITH_GPU

    #if defined RADARAYS_WITH_CPU
        cpu_available = true;
    #endif // defined RADARAYS_WITH_CPU


    if(use_gpu && !gpu_available)
    {
        std::cout << "Desired computing unit 'GPU' is not available on your system." << std::endl;
        throw std::runtime_error("Desired computing unit 'GPU' is not available on your system.");
    }

    if(!use_gpu && !cpu_available)
    {
        std::cout << "Desired computing unit 'CPU' is not available on your system." << std::endl;
        throw std::runtime_error("Desired computing unit 'CPU' is not available on your system.");
    }
    
    if(!use_gpu)
    {
        // CPU
        #if defined RADARAYS_WITH_CPU
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
        #endif // defined RADARAYS_WITH_CPU
    } else {
        // GPU
        #if defined RADARAYS_WITH_GPU
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
        #endif // defined RADARAYS_WITH_GPU
    }

    radarays_sim->loadParams();
}

void run_radar_publisher()
{
    std::cout << "STARTING RADAR SIMULATOR" << std::endl;

    // image transport
    it = std::make_shared<image_transport::ImageTransport>(*nh);
    pub_polar = it->advertise("radar/image", 1);

    // pcl
    pub_pcl = nh_p->advertise<sensor_msgs::PointCloud>("radar/pcl_real", 1);
    

    std::string sync_topic = "";

    if(nh_p->getParam("sync_topic", sync_topic))
    {
        std::cout << "SYNC SIMULATIONS WITH TOPIC " << sync_topic << std::endl;
        ros::Subscriber sub = nh->subscribe<sensor_msgs::Image>(sync_topic, 1, sync_cb);
        ros::spin();
    } else {
        // unsynced
        ros::Rate r(100);
        while(nh->ok())
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
}


void generate_radar_image_cb(const GenRadarImageGoalConstPtr &goal)
{
    std::cout << "Client requested for radar image. Producing result..." << std::endl;
    auto params = goal->params;
    radarays_sim->setParams(goal->params);
    sensor_msgs::ImagePtr msg = radarays_sim->simulate(ros::Time(0));
    result_.polar_image = *msg;
    std::cout << "finished." << std::endl;
    as_->setSucceeded(result_);
}


bool get_radar_params_cb(radarays_ros::GetRadarParams::Request &req,
         radarays_ros::GetRadarParams::Response &res)
{
    if(radarays_sim)
    {
        std::cout << "Return simulation parameters!" << std::endl;
        res.params = radarays_sim->getParams();
        return true;
    } else {
        return false;
    }
    return true;
}


void run_radar_server()
{
    std::cout << "STARTING RADAR SIMULATION SERVER" << std::endl;

    // Start service server
    // can be used to receive current radar parameters
    // - in this case it is mainly used to receive the current parameters to make small changes
    ros::ServiceServer service = nh_p->advertiseService("get_radar_params", get_radar_params_cb);

    // Start action server
    std::string action_name = "gen_radar_image";
    as_ = std::make_shared<actionlib::SimpleActionServer<GenRadarImageAction> >(
        *nh_p, action_name, generate_radar_image_cb, false
    );

    as_->start();

    ros::Rate r(100);
    ros::Time tp = ros::Time::now();

    while(ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        auto tc = ros::Time::now();
        if(tc < tp)
        {
            // jump back in time
            std::cout << "Jump back in time detected" << std::endl;
            ros::Duration(2.0).sleep();
            as_->shutdown();
            as_ = std::make_shared<actionlib::SimpleActionServer<GenRadarImageAction> >(
                *nh_p, action_name, generate_radar_image_cb, false
            );
            as_->start();
            ros::Duration(2.0).sleep();
        }

        tp = tc;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "radar_simulator");
    
    nh = std::make_shared<ros::NodeHandle>();
    nh_p = std::make_shared<ros::NodeHandle>("~");
    init_sim();

    std::string node_mode;
    nh_p->param<std::string>("node_mode", node_mode, "publisher");

    if(node_mode == "publisher")
    {
        run_radar_publisher();
    } else if(node_mode == "server") {
        run_radar_server();
    } else {
        std::stringstream ss;
        ss << "node_mode '" << node_mode << "' not known!"; 
        throw std::runtime_error(ss.str());
    }
    
    return 0;
}
