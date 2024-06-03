#ifndef RADARAYS_RADARCPUFLEX_HPP
#define RADARAYS_RADARCPUFLEX_HPP

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
 * - Flexibly using different BRDF functions
 * - Not optimized for performance
 * 
 * Flexibly changing the BRDFs comes with a slight performance drop. Hence, we recommend using this
 * class only to test new BRDF functions.
 * For faster implementations use RadarCPU or RadarGPU instead.
 * 
*/
class RadarCPUFlex : public Radar 
{
public:
    using Base = Radar;

    RadarCPUFlex(
        std::shared_ptr<ros::NodeHandle> nh_p,
        std::shared_ptr<tf2_ros::Buffer> tf_buffer,
        std::shared_ptr<tf2_ros::TransformListener> tf_listener,
        std::string map_frame,
        std::string sensor_frame,
        rm::EmbreeMapPtr map
    );

    // function that is generating 
    void setSampleFunc(InitSamplingFunc init_sampling_func);

    bool isFreeInBetween(const rm::Vector& p1, const rm::Vector& p2, float t1_offset = 0.001) const;

    std::optional<Intersection> intersect(DirectedWave& wave) const;

    void renderSingleWave(
        const DirectedWave& wave, 
        const Sender& sender, 
        std::vector<float>& range_returns, 
        std::vector<int>& range_counts,
        int tree_depth = 3) const;

    void renderWave(
        const Receiver& receiver,
        const Sender& sender,
        const DirectedWave& wave_init,
        std::vector<float>& range_returns, 
        std::vector<int>& range_counts) const;

    std::vector<float> energy_to_decibel(float sent_energy, const std::vector<float>& range_returns, const std::vector<int>& range_counts) const;
    // std::vector<float> blur(const std::vector<float>& energy) const;
    
    virtual sensor_msgs::ImagePtr simulate(ros::Time stamp);
protected:

    rm::EmbreeMapPtr m_map;
    std::vector<Material> m_materials;

    InitSamplingFunc m_init_sampling_func; // used for all azimuth angles

    // ros::Publisher m_data_pub;
};

} // namespace radarays_ros


#endif // RADARAYS_RADARCPUFLEX_HPP