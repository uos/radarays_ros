#include "radarays_ros/radar_math.h"
#include "radarays_ros/radar_types.h"

namespace radarays_ros
{

// transform ray
Ray operator*(const rm::Transform& T, const Ray& ray)
{
    Ray ret;
    ret.orig = T * ray.orig;
    ret.dir = T.R * ray.dir;
    return ret;
}

DirectedWave operator*(const rm::Transform& T, const DirectedWave& wave)
{
    DirectedWave ret = wave;
    ret.ray = T * wave.ray;
    return ret;
}

std::vector<DirectedWave> operator*(const rm::Transform& T, const std::vector<DirectedWave>& waves)
{
    std::vector<DirectedWave> ret = waves;

    for(DirectedWave& wave : ret)
    {
        wave.ray = T * wave.ray;
    }

    return ret;
}

} // namespace radarays_ros