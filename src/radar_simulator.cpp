#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>

#include <rmagine/util/prints.h>
#include <rmagine/util/StopWatch.hpp>
#include <random>
#include <optional>
#include <chrono>
#include <thread>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <radarays_ros/radar_types.h>
#include <radarays_ros/radar_math.h>
#include <radarays_ros/msg/radar_params.hpp>
#include <radarays_ros/srv/get_radar_params.hpp>
#include <radarays_ros/action/gen_radar_image.hpp>

#include <radarays_ros/ros_helper.h>

#include <radarays_ros/image_algorithms.h>
#include <radarays_ros/radar_algorithms.h>

#include <rmagine/simulation/OnDnSimulatorEmbree.hpp>
#include <radarays_ros/RadarCPU.hpp>


using namespace radarays_ros;

namespace rm = rmagine;

// serve_action mode: GetRadarParams service + GenRadarImage action server,
// request-driven instead of continuous-publish. Kept mutually exclusive with
// the free-running/sync_topic loop below -- both would call
// radarays_sim->simulate() from different threads on the same Radar object
// otherwise (Radar isn't built to be thread-safe against itself). Mirrors
// Classic's main_publisher vs main_action_server being alternate main()
// entry points, never run together.
//
// The accepted-goal callback only *queues* the goal -- it must not call
// simulate() directly. RadarCPU::simulate() itself calls
// rclcpp::spin_some(m_node) when include_motion is set (the default), and
// that call would be reentrant if simulate() ran from inside a callback
// already dispatched by an active spin/spin_some on the same node (rclcpp
// throws "Node has already been added to an executor"). Draining the queue
// happens in main()'s own loop, between spin_some() calls, same as the
// existing free-running publish loop already does safely.
class GenRadarImageServer
{
public:
    using GenRadarImage = radarays_ros::action::GenRadarImage;
    using GoalHandleGenRadarImage = rclcpp_action::ServerGoalHandle<GenRadarImage>;

    GenRadarImageServer(rclcpp::Node::SharedPtr node, RadarPtr radar)
    : node_(node)
    , radar_(radar)
    {
        action_server_ = rclcpp_action::create_server<GenRadarImage>(
            node_,
            "gen_radar_image",
            [](const rclcpp_action::GoalUUID&, std::shared_ptr<const GenRadarImage::Goal>)
            {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [](const std::shared_ptr<GoalHandleGenRadarImage>)
            {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<GoalHandleGenRadarImage> goal_handle)
            {
                pending_goal_ = goal_handle;
            });

        service_server_ = node_->create_service<radarays_ros::srv::GetRadarParams>(
            "get_radar_params",
            [this](
                const std::shared_ptr<radarays_ros::srv::GetRadarParams::Request>,
                std::shared_ptr<radarays_ros::srv::GetRadarParams::Response> response)
            {
                response->params = radar_->getParams();
            });
    }

    // Called from main()'s loop, never from within an active spin_some().
    void processPendingGoal()
    {
        if(!pending_goal_)
        {
            return;
        }
        std::shared_ptr<GoalHandleGenRadarImage> goal_handle = pending_goal_;
        pending_goal_.reset();

        const auto goal = goal_handle->get_goal();
        radar_->setParams(goal->params);
        sensor_msgs::msg::Image::SharedPtr msg = radar_->simulate(node_->get_clock()->now());

        auto result = std::make_shared<GenRadarImage::Result>();
        if(msg)
        {
            result->polar_image = *msg;
            goal_handle->succeed(result);
        } else {
            goal_handle->abort(result);
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    RadarPtr radar_;
    rclcpp_action::Server<GenRadarImage>::SharedPtr action_server_;
    rclcpp::Service<radarays_ros::srv::GetRadarParams>::SharedPtr service_server_;
    std::shared_ptr<GoalHandleGenRadarImage> pending_goal_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::cout << "STARTING RADAR SIMULATOR" << std::endl;

    auto node = std::make_shared<rclcpp::Node>("radar_simulator");

    // setting up tf
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer, node);

    node->declare_parameter("map_file", std::string(""));
    node->declare_parameter("map_frame", std::string("map"));
    node->declare_parameter("sensor_frame", std::string("navtech"));
    node->declare_parameter("sync_topic", std::string(""));
    node->declare_parameter("publish_rate", 100.0);
    node->declare_parameter("serve_action", false);
    // Classic's main() read a `gpu` param and dispatched to RadarCPU or
    // RadarGPU from this same executable; the ROS 2 port split GPU out
    // into a separate `radar_simulator_gpu` executable instead (see
    // MIGRATION.md), so this node has no way to actually honor `gpu:=true`
    // -- declared here only so an old launch file/params file setting it
    // fails loudly instead of silently running on the CPU anyway.
    node->declare_parameter("gpu", false);

    const std::string map_file = node->get_parameter("map_file").as_string();
    const std::string map_frame = node->get_parameter("map_frame").as_string();
    const std::string sensor_frame = node->get_parameter("sensor_frame").as_string();

    if(map_file.empty())
    {
        RCLCPP_ERROR(node->get_logger(), "map_file parameter is required.");
        return 1;
    }

    if(node->get_parameter("gpu").as_bool())
    {
        RCLCPP_ERROR(node->get_logger(),
            "gpu:=true was passed to radar_simulator, but this executable is "
            "CPU/Embree-only -- GPU/OptiX is a separate executable in this "
            "ROS 2 port. Run radar_simulator_gpu instead (see MIGRATION.md, "
            "\"Known Gaps vs the ROS 1 (noetic) Branch\").");
        return 1;
    }

    // CPU / Embree only for now -- GPU/OptiX parity is deferred workspace-wide.
    rm::EmbreeMapPtr map_cpu = rm::import_embree_map(map_file);

    std::cout << "RadarCPU" << std::endl;
    RadarPtr radarays_sim = std::make_shared<RadarCPU>(
        node,
        tf_buffer,
        tf_listener,
        map_frame,
        sensor_frame,
        map_cpu
    );

    if(node->get_parameter("serve_action").as_bool())
    {
        std::cout << "SERVE MODE: gen_radar_image action + get_radar_params service" << std::endl;
        GenRadarImageServer server(node, radarays_sim);
        rclcpp::Rate r(100.0);
        while(rclcpp::ok())
        {
            radarays_sim->loadParams();
            rclcpp::spin_some(node);
            server.processPendingGoal();
            r.sleep();
        }
        rclcpp::shutdown();
        return 0;
    }

    // image transport
    image_transport::ImageTransport it(node);
    image_transport::Publisher pub_polar = it.advertise("radar/image", 1);

    const std::string sync_topic = node->get_parameter("sync_topic").as_string();

    if(!sync_topic.empty())
    {
        std::cout << "SYNC SIMULATIONS WITH TOPIC " << sync_topic << std::endl;

        // The subscription callback only queues the latest sync stamp --
        // it must not call simulate() directly. RadarCPU::simulate()
        // itself calls rclcpp::spin_some(m_node) when include_motion is
        // set (the default), which is reentrant (and throws "Node has
        // already been added to an executor") if simulate() runs from
        // inside a callback already dispatched by an active
        // spin()/spin_some() on the same node -- exactly what a blocking
        // rclcpp::spin(node) here would do. Same fix as the serve_action
        // path above: drain the queued stamp from a manual polling loop,
        // between spin_some() calls, never from within one.
        std::optional<rclcpp::Time> pending_stamp;
        auto sub = node->create_subscription<sensor_msgs::msg::Image>(
            sync_topic, 1,
            [&](const sensor_msgs::msg::Image::ConstSharedPtr &sync_msg)
            {
                pending_stamp = rclcpp::Time(sync_msg->header.stamp);
            });

        while(rclcpp::ok())
        {
            rclcpp::spin_some(node);

            if(pending_stamp)
            {
                const rclcpp::Time stamp = *pending_stamp;
                pending_stamp.reset();

                radarays_sim->loadParams();
                sensor_msgs::msg::Image::SharedPtr msg = radarays_sim->simulate(stamp);

                if(!msg)
                {
                    RCLCPP_INFO_STREAM(node->get_logger(), "SYNC: " << stamp.nanoseconds() << " -- None");
                } else {
                    pub_polar.publish(*msg);
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    } else {
        // unsynced
        const double publish_rate = node->get_parameter("publish_rate").as_double();
        rclcpp::Rate r(publish_rate);
        while(rclcpp::ok())
        {
            radarays_sim->loadParams();
            sensor_msgs::msg::Image::SharedPtr msg = radarays_sim->simulate(rclcpp::Time(0, 0, node->get_clock()->get_clock_type()));

            if(msg)
            {
                pub_polar.publish(*msg);
            } else {
                std::cout << "MESSAGE EMPTY" << std::endl;
            }

            rclcpp::spin_some(node);
            r.sleep();
        }
    }

    rclcpp::shutdown();
    return 0;
}
