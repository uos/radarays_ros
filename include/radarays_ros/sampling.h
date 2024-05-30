#ifndef RADARAYS_ROS_SAMPLING_H
#define RADARAYS_ROS_SAMPLING_H

#include <rmagine/math/types.h>
#include <radarays_ros/radar_types.h>
#include <random>
#include <cassert>

namespace rm = rmagine;

namespace radarays_ros
{

// TODO: put some sample functions here

// rm::Vector sample_reflection_perfect_mirror(
//     const DirectedWave& incidence, // incident wave
//     const rm::Vector3& normal, // surface normal
//     const Material* material // surface material
//     )
// {
//     rm::Vector ret{1.0, 0.0, 0.0};
//     return ret;
// }


// hemisphere is pointing in positive x-direction
// see: https://www.mathematik.uni-marburg.de/~thormae/lectures/graphics1/code/ImportanceSampling/index.html
// "Uniform sampling of a hemisphere"
inline rm::Vector sample_hemisphere_uniform()
{
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 gen(rd()); // seed the generator
    std::uniform_real_distribution<float> distr(0.0, 1.0); // define the range
    const float u = distr(gen);
    const float v = distr(gen);
    // map two random values to polar angles of hemisphere

    // PHI: vertical, y-rot, pitch, polar angle, height
    // range [-PI/2, PI/2)
    float phi = 2.0 * M_PI * u;
    float theta = acos(1.0 - v);

    // the resulting vector has never x value < 0
    const rm::Vector sample_optical{
        cosf(phi) * sin(theta), 
        sinf(phi) * sinf(theta), 
        cosf(theta)};

    const rm::Vector sample{
        sample_optical.z,
        -sample_optical.x,
        -sample_optical.y
    };

    // TODO: figure out what the limits are
    assert(sample.x >= 0.0);
    return sample;
}

// inline rm::Vector sample_hemisphere_cosine()
// {
//     std::random_device rd; // obtain a random number from hardware
//     std::mt19937 gen(rd()); // seed the generator
//     std::uniform_int_distribution<float> distr(0.0, 1.0); // define the range
//     const float u = distr(gen);
//     const float v = distr(gen);
//     // map two random values to polar angles of hemisphere

//     // PHI: vertical, y-rot, pitch, polar angle, height
//     // range [-PI/2, PI/2)
//     float phi = 2.0 * M_PI * u;
//     float theta = acos(1.0 - v);

//     // the resulting vector has never x value < 0
//     const rm::Vector sample{cosf(phi) * cosf(theta), cosf(phi) * sinf(theta), sinf(phi)};

//     assert(sample.x >= 0.0);
//     return sample;
// }


} // radarays_ros

#endif // RADARAYS_ROS_SAMPLING_H