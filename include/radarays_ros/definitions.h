#ifndef RADARAYS_DEFINITIONS_H
#define RADARAYS_DEFINITIONS_H

// but all definitions and macros here
#include <memory>
#include <functional>

#include <rmagine/math/types.h>

namespace rm = rmagine;

namespace radarays_ros
{

struct Ray;
struct Signal;
struct DirectedWave;
struct DirectedWaveAttributes;
struct AmbientNoiseParams;
struct Material;
struct Intersection;

struct Sender;
struct Receiver;


using BRDFFunc = std::function<float( // returns reflectance value
            const DirectedWave&, // incident wave
            const rm::Vector3&, // surface normal
            const Material*, // surface material
            const rm::Vector3& // out_direction
            )>;

/**
 * How the receiver samples the scene with rays initially
*/
using InitSamplingFunc = std::function<std::vector<DirectedWave>()>;

using SamplingFunc = std::function<rm::Vector(
    const DirectedWave&, // incident wave
    const rm::Vector3&, // surface normal
    const Material*, // surface material
    const rm::Vector3& // out_direction
)>;

using ReceiverFunc = std::function<float(
            const rm::Vector& // incoming 
            )>;

using MaterialPtr = std::shared_ptr<Material>;

} // namespace radarays_ros

#endif // RADARAYS_CONSTANTS_H