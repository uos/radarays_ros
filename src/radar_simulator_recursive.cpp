#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>




#include <std_msgs/Float32MultiArray.h>


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

#include <radarays_ros/RadarCPURec.hpp>


using namespace radarays_ros;

namespace rm = rmagine;


std::string map_frame = "map";
std::string sensor_frame = "navtech";

std::shared_ptr<ros::NodeHandle> nh_p;
std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;
ros::Publisher pub_pcl;
ros::Publisher pub_signals;
image_transport::Publisher pub_polar;
image_transport::Publisher pub_polar_gt;

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

std::vector<float> image_column_to_plot(const cv::Mat& mat, int col)
{
    std::vector<float> ret(mat.rows, 0.0);

    auto column_uint = mat.col(col);

    cv::Mat column_float;
    column_uint.convertTo(column_float, CV_32FC1);

    for(int i=0; i<column_float.rows; i++)
    {
        ret[i] = column_float.at<float>(i);
    }

    return ret;
}

std_msgs::Float32MultiArray image_column_to_plot(const sensor_msgs::Image::ConstPtr& msg, int viz_column)
{
    std_msgs::Float32MultiArray plot;
    
    cv::Mat image;
    try
    {
        image = cv_bridge::toCvShare(msg)->image;
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR(msg->encoding.c_str());
        return plot;
    }

    plot.data = image_column_to_plot(image, viz_column);
    return plot;
}

void sync_cb(const sensor_msgs::Image::ConstPtr& sync_msg)
{
    radarays_sim->loadParams();
    sensor_msgs::ImagePtr msg = radarays_sim->simulate(sync_msg->header.stamp);
    if(!msg)
    {
        ROS_INFO_STREAM("SYNC: " << sync_msg->header.stamp << " -- " << " None");
        return;
    }

    int viz_column = 300;
    if(viz_column >= 0)
    {
        const std_msgs::Float32MultiArray plot = image_column_to_plot(sync_msg, viz_column);
        pub_signals.publish(plot);
    }
    
    ROS_INFO_STREAM("SYNC: " << sync_msg->header.stamp << " -- " << msg->header.stamp << ", SYN ERR: " << (sync_msg->header.stamp - msg->header.stamp).toSec() * 1000.0 << "ms" );
    pub_polar.publish(msg);
    pub_polar_gt.publish(sync_msg);
}

int main_publisher(int argc, char** argv)
{
    ros::init(argc, argv, "radar_simulator");

    std::cout << "STARTING RECURSIVE RADAR SIMULATOR" << std::endl;

    ros::NodeHandle nh;
    nh_p = std::make_shared<ros::NodeHandle>("~");


    std::cout << "CREATING TF BUFFER WITH CACHE TIME: " << tf2::BufferCore::DEFAULT_CACHE_TIME << std::endl;

    // setting up tf
    tf_buffer = std::make_shared<tf2_ros::Buffer>();
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    
    std::string map_file;
    nh_p->getParam("map_file", map_file);
    nh_p->getParam("map_frame", map_frame);
    nh_p->getParam("sensor_frame", sensor_frame);

     
    rm::EmbreeMapPtr map_cpu = rm::import_embree_map(map_file);

    std::cout << "RadarCPURec" << std::endl;
    radarays_sim = std::make_shared<RadarCPURec>(
        nh_p,
        tf_buffer,
        tf_listener,
        map_frame,
        sensor_frame,
        map_cpu
    );


    // image transport
    image_transport::ImageTransport it(nh);
    pub_polar = it.advertise("radar/image", 1);
    pub_polar_gt = it.advertise("radar/image_gt", 1);

    // pcl
    pub_pcl = nh_p->advertise<sensor_msgs::PointCloud>("radar/pcl_real", 1);
    pub_signals = nh_p->advertise<std_msgs::Float32MultiArray>("data_real", 1);

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

void how_to_estimate_refractive_index()
{
    // realtime rendering table 9.2 titanium
    float n1 = 0.542;
    float l1 = 700.0; // nm red (approx)
    float n2 = 0.497;
    float l2 = 550.0; // nm green (approx)
    float n3 = 0.449;
    float l3 = 470.0; // nm blue (approx)

    // 73.0 in ghz
    // 0.00410675 m wavelength
    // 4.10675 mm
    float l4 = 4.106745e+6; // um

    // float l4 = 73.0 / 
    std::cout << l4 << std::endl;

    std::cout << "Estimated n for a 73 Ghz wave: " << cauchy_estimate_n(n1, l1, n2, l2, n3, l3, l4) << std::endl;
}

int main(int argc, char** argv)
{
    // return main_action_server(argc, argv);
    return main_publisher(argc, argv);
}
