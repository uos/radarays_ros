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

struct DirectedWaveAttributes
{
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


    static DirectedWaveAttributes Zeros()
    {
        DirectedWaveAttributes ret;
        ret.energy = 0.0;
        ret.polarization = 0.5;
        ret.frequency = 0.0;
        ret.velocity = 0.0;
        return ret;
    }

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

struct AmbientNoiseParams
{
    float noise_at_signal_0 = 0.1;
    float noise_at_signal_1 = 0.03;
    float noise_energy_min = 0.05;
    float noise_energy_max = 0.08;
    float noise_energy_loss = 0.05;
    float resolution = 0.0595238;
};

} // namespace radarays_ros

#endif // RADARAYS_ROS_RADAR_TYPES_H