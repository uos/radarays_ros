#ifndef RADARAYS_RADAR_CPU_HPP
#define RADARAYS_RADAR_CPU_HPP

#include "Radar.hpp"

#include <rmagine/simulation/OnDnSimulatorEmbree.hpp>
#include <rmagine/map/EmbreeMap.hpp>
#include <unordered_map>


namespace rm = rmagine;

namespace radarays_ros
{

class RadarCPU : public Radar
{
public:
    using Base = Radar;

    RadarCPU(
        rclcpp::Node::SharedPtr node,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        std::shared_ptr<tf2_ros::TransformListener> tf_listener,
        std::string map_frame,
        std::string sensor_frame,
        rm::EmbreeMapPtr map
    );

    virtual sensor_msgs::msg::Image::SharedPtr simulate(rclcpp::Time stamp);

protected:
    std::unordered_map<unsigned int, rm::OnDnSimulatorEmbreePtr> m_sims;
    rm::EmbreeMapPtr m_map;
};

using RadarCPUPtr = std::shared_ptr<RadarCPU>;

} // namespace radarays_ros

#endif // RADARAYS_RADAR_CPU_HPP
