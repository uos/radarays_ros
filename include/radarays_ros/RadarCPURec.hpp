#ifndef RADARAYS_RADARCPUREC_HPP
#define RADARAYS_RADARCPUREC_HPP

#include "Radar.hpp"

// #include <rmagine/simulation/OnDnSimulatorEmbree.hpp>
#include <rmagine/map/EmbreeMap.hpp>
#include <unordered_map>
#include "radar_types.h"
#include <vector>
#include <rmagine/math/types.h>

#include <radarays_ros/definitions.h>
#include <radarays_ros/wave_shaders.h>


namespace radarays_ros
{


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
    void setSampleFunc(WaveGenFunc wave_gen_func);

    bool isFreeInBetween(const rm::Vector& p1, const rm::Vector& p2, float t1_offset = 0.001) const;

    std::optional<Intersection> intersect(DirectedWave& wave) const;

    // void sample(rm::Vector direction, const Sender& sender, std::vector<float>& range_returns, int tree_depth = 3) const;

    float renderSingleWave(const DirectedWave& wave, const Sender& sender, std::vector<float>& range_returns, int tree_depth = 3) const;
    
    virtual sensor_msgs::ImagePtr simulate(ros::Time stamp);




protected:

    rm::EmbreeMapPtr m_map;
    std::vector<Material> m_materials;

    WaveGenFunc m_wave_generator; // used for all azimuth angles
};

} // namespace radarays_ros


#endif // RADARAYS_RADARCPU2_HPP