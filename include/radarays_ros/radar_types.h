#ifndef RADARAYS_ROS_RADAR_TYPES_H
#define RADARAYS_ROS_RADAR_TYPES_H

#include <rmagine/math/types.h>

namespace radarays_ros
{

struct Ray
{
    rmagine::Vector orig;
    rmagine::Vector dir;

    static Ray Zeros()
    {
        Ray ret;
        ret.orig = rmagine::Vector::Zeros();
        ret.dir = rmagine::Vector::Zeros();
        return ret;
    }
};

struct Signal
{
    double time;
    double strength;
};

struct DirectedWave
{
    // origin and direction
    Ray ray;
    // !!Not!! energy of wave ( E = h * v / l ). 
    // INSTEAD: user defined energy. 
    double energy;
    // ratio of s-polarized and p-polarized
    // 1.0: only s-polarized waves
    // 0.0: only p-polarized waves
    // 0.5: unpolarized
    double polarization;
    // velocity (normally given in [m/ns])
    double velocity;
    // frequency of wave (GHz)
    double frequency;
    
    // time of travelling [ns?]
    double time;

    // current medium (is this redundant with velocity?)
    unsigned int material_id;
    
    void setLength(double l)
    {
        velocity = l * frequency;
    }

    // l = v / f
    double length() const
    {
        return velocity / frequency;
    }

    static DirectedWave Zeros()
    {
        DirectedWave ret;
        ret.ray = Ray::Zeros();
        ret.energy = 0.0;
        ret.polarization = 0.5;
        ret.frequency = 0.0;
        ret.velocity = 0.0;
        return ret;
    }

    DirectedWave& moveInplace(double distance)
    {
        ray.orig = ray.orig + ray.dir * distance;
        time += distance / velocity;
        return *this;
    }

    DirectedWave move(double distance) const
    {
        DirectedWave wave = *this;
        wave.moveInplace(distance);
        return wave;
    }
};

} // namespace radarays_ros

#endif // RADARAYS_ROS_RADAR_TYPES_H