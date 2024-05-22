#ifndef RADARAYS_DEFINITIONS_H
#define RADARAYS_DEFINITIONS_H

// but all definitions and macros here
#include <memory>



namespace radarays_ros
{

struct DirectedWave;
struct Intersection;
struct Material;

using MaterialPtr = std::shared_ptr<Material>;

} // namespace radarays_ros

#endif // RADARAYS_CONSTANTS_H