#ifndef RADARAYS_RADARCPUREC_HPP
#define RADARAYS_RADARCPUREC_HPP

#include "Radar.hpp"

// #include <rmagine/simulation/OnDnSimulatorEmbree.hpp>
#include <rmagine/map/EmbreeMap.hpp>
#include <unordered_map>
#include "radar_types.h"
#include <vector>
#include <rmagine/math/types.h>

#include <radarays_ros/wave_shaders.h>


namespace radarays_ros
{


using WaveGenFunc = std::function<std::vector<DirectedWave>()>;

/**
 * Simulating radar measurements using a recursive strategy of approximating the rendering equation
*/
class RadarCPURec : public Radar 
{
public:
    using Base = Radar;

    RadarCPURec(
        std::shared_ptr<ros::NodeHandle> nh_p,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        std::shared_ptr<tf2_ros::TransformListener> tf_listener,
        std::string map_frame,
        std::string sensor_frame,
        rm::EmbreeMapPtr map
    );

    // function that is generating 
    void setWaveGenFunc(WaveGenFunc wave_gen_func);

    std::optional<Intersection> moveUntilIntersection(DirectedWave& wave) const;

    virtual sensor_msgs::ImagePtr simulate(ros::Time stamp);

protected:

    rm::EmbreeMapPtr m_map;
    std::vector<Material> m_materials;

    WaveGenFunc m_wave_generator;
};

} // namespace radarays_ros


#endif // RADARAYS_RADARCPU2_HPP