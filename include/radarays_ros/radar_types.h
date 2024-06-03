#ifndef RADARAYS_ROS_RADAR_TYPES_H
#define RADARAYS_ROS_RADAR_TYPES_H

#include <functional>
#include <vector>
#include <rmagine/math/types.h>
#include "radar_math.h"
#include "definitions.h"


namespace rm = rmagine; // TODO: remvove this from headers

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
    // ratio of s-polarized and p-polarized (currently mostly unused)
    // 1.0: only s-polarized waves
    // 0.0: only p-polarized waves
    // 0.5: unpolarized
    double polarization;

    // frequency of wave (GHz) (unchanged during propagation)
    double frequency;
    
    // time of travelling [ns]
    double time;

    // material
    const Material* material;


    // obsolete fields: TODO remove

    // velocity (normally given in [m/ns]) (changed if transmitted to another medium)
    double velocity;
    // current medium (is this redundant with velocity?)
    unsigned int material_id;


    double getVelocity() const;
    double getRefractiveIndex() const;

    // l = v / f
    double getWaveLength() const;

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

    DirectedWave& moveInplace(double distance);

    DirectedWave move(double distance) const;

    

    inline double getDistanceAir() const
    {
        return time * M_C_IN_M_PER_NANOSECOND;
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



// template for a BRDF function
struct Material
{
    std::string name;
    
    // refractive index of material
    // can be computed by n = c/v
    // set to a really high value to
    double n;

    // value between 0-1. how much of the refracted energy is transmitted through the material (per meter!)
    // absorption = 1 - transmittance
    // extreme example:
    // - black color is not reflecting light nor does it transmit the light through the material as glass
    // rather than having a fixed parameter have absorption per distance instead
    // like this you can calculate the actual absorpt part after the next intersection test
    // 1. continue shooting
    // 2. compute the the absoption via absorption = 1-(1-absorption_per_m)^(meter)
    // see if the energy falls below a value. If yes, finish; if no, continue
    // special case: if absorption is equal to 1, it is clear that everything is absorpt. We can stop without doing the first intersection test
    double transmittance;


    // variable number of parameters used in brdf func
    std::vector<float> brdf_params;
    // user specific brdf function
    BRDFFunc brdf_func;

    float brdf(const DirectedWave& incidence, const rm::Vector& normal, const rm::Vector3& out_direction) const;
};



struct Intersection
{
    // surface normal
    rm::Vector normal;

    const Material* material;

    float brdf(const DirectedWave& incidence, 
        const rm::Vector3& out_direction) const;

    // get "mode" wave of reflection and transmission(refraction) 
    // according to snell's law and fresnel's equation
    std::pair<DirectedWave, DirectedWave> fresnel(const DirectedWave& wave_in) const;
};


// Sender in map coordinates
struct Receiver
{
    // in map coordinates
    rm::Transform Tsm;

    // wave gen in sensor coordinates
    InitSamplingFunc sample_func = []() -> std::vector<DirectedWave>{
        DirectedWave wave;
        wave.energy       =  1.0; //
        wave.polarization =  0.5;
        wave.frequency    = 76.5; // GHz
        wave.time         =  0.0; // in ns
        wave.ray.orig = {0.0, 0.0, 0.0};
        wave.ray.dir = {1.0, 0.0, 0.0};
        return {wave};
    };

    std::vector<DirectedWave> genSamples() const
    {
        return sample_func();
    }

    // generates rays ready to shoot in map coordinates
    std::vector<DirectedWave> genSamplesMap() const
    {
        return ~Tsm * genSamples();
    }
};

/**
 * From the receiver we raytrace to finde traces to the sender
*/
struct Sender
{
    // sensor -> map transform / pose in map coordinates
    rm::Transform Tsm;
    
    // energy_density function
    // wave: in sensor coordinates
    ReceiverFunc energy_density_func = [](const rm::Vector dir) -> float {
        
        // const rm::Vector front = {1.0, 0.0, 0.0};
        // return front.dot(dir);
        return 1.0;
    };

    float getEnergyDensity(const rm::Vector& dir) const {
        // transform wave to sensor coordinates and compute fraction of return energy
        return energy_density_func(~Tsm.R * dir);
    }
};


} // namespace radarays_ros

#endif // RADARAYS_ROS_RADAR_TYPES_H