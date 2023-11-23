#ifndef RADARAYS_RADAR_GPU_HPP
#define RADARAYS_RADAR_GPU_HPP

#include "Radar.hpp"

#include <rmagine/simulation/OnDnSimulatorOptix.hpp>
#include <rmagine/map/OptixMap.hpp>
#include <unordered_map>

namespace rm = rmagine;

namespace radarays_ros
{

class RadarGPU : public Radar 
{
public:
    using Base = Radar;

    RadarGPU(
        std::shared_ptr<ros::NodeHandle> nh_p,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        std::shared_ptr<tf2_ros::TransformListener> tf_listener,
        std::string map_frame,
        std::string sensor_frame,
        rm::OptixMapPtr map
    );

    virtual sensor_msgs::ImagePtr simulate(
        ros::Time stamp);

protected:
    std::unordered_map<unsigned int, rm::OnDnSimulatorOptixPtr> m_sims;
    rm::OptixMapPtr m_map;
};

using RadarGPUPtr = std::shared_ptr<RadarGPU>;

} // namespace radarays

#endif // RADARAYS_RADAR_CPU_HPP