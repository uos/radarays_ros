#ifndef RADARAYS_ROS_RADAR_ALGORITHMS_H
#define RADARAYS_ROS_RADAR_ALGORITHMS_H

#include <vector>
#include <random>
#include "radar_types.h"
#include "radar_math.h"

#include <rmagine/types/sensor_models.h>
#include <rmagine/types/Memory.hpp>

namespace rm = rmagine;

namespace radarays_ros
{

inline double angle_between(
    const rmagine::Vector a,
    const rmagine::Vector b
)
{
    return acos(a.dot(b));
}

inline double get_incidence_angle(
    const rmagine::Vector surface_normal,
    const rmagine::Vector incidence_dir
)
{
    return acos((-incidence_dir).dot(surface_normal));
}

inline double get_incidence_angle(
    const rmagine::Vector surface_normal,
    const DirectedWave incidence)
{
    return acos((-incidence.ray.dir).dot(surface_normal));
}

/**
 * @brief Function to compute reflection and refraction including fresnel energy split
 * 
 * The returning reflection and refraction waves contain the directions for a perfect flat surface.
 * So in reality they would represent the peak of the reflection distribution. 
 * The energy then gives the integral of the reflection/refraction distribution
 * 
 * For more realistic reflections/refractions the results of the functions should be postprocess
 * to model the distributions properly.
 * 
 * @param surface_normal 
 * @param incidence 
 * @param v2 
 * @return std::pair<DirectedWave, DirectedWave> ReflectionWave and RefractionWave
 */
inline std::pair<DirectedWave, DirectedWave> fresnel(
    rmagine::Vector surface_normal,
    DirectedWave incidence,
    double v2)
{
    const double v1 = incidence.velocity;

    const double n1 = v2;
    const double n2 = v1;

    DirectedWave reflection = incidence;
    DirectedWave refraction = incidence;

    // is this correct?
    double incidence_angle = acos((-incidence.ray.dir).dot(surface_normal));

    // reflection
    reflection.ray.orig = incidence.ray.orig;
    reflection.ray.dir = incidence.ray.dir + surface_normal * 2.0 * (-surface_normal).dot(incidence.ray.dir);

    // refraction
    refraction.ray.orig = incidence.ray.orig;
    refraction.ray.dir = rmagine::Vector::Zeros();
    refraction.velocity = v2;

    if(n1 > 0.0)
    {
        double n21 = n2 / n1;
        double angle_limit = 100.0;

        if(abs(n21) <= 1.0)
        {
            angle_limit = asin(n21);
        }

        if(incidence_angle <= angle_limit)
        {
            if(surface_normal.dot(incidence.ray.dir) > 0.0)
            {
                surface_normal = -surface_normal;
            }
            if(n2 > 0.0)
            {
                double n12 = n1 / n2;
                double c = cos(incidence_angle);
                refraction.ray.dir = incidence.ray.dir * n12 + surface_normal * (n12 * c - sqrt(1 - n12*n12 * ( 1 - c*c ) ) );
            }
        }
    }
    
    // energy
    double refraction_angle = acos((refraction.ray.dir).dot(-surface_normal));

    double rs = 0.0;
    double rp = 0.0;
    double eps = 0.0001;
    
    if(incidence_angle + refraction_angle < eps)
    {
        rs = (n1 - n2) / (n1 + n2);
        rp = rs;
    } else if(incidence_angle + refraction_angle > M_PI - eps) {
        rs = 1.0;
        rp = 1.0;
    } else {
        rs = - sin(incidence_angle - refraction_angle) / sin(incidence_angle + refraction_angle);
        rp = tan(incidence_angle - refraction_angle) / tan(incidence_angle + refraction_angle);
    }

    double sp_ratio = 0.5;

    double Rs = rs * rs;
    double Rp = rp * rp;
    
    double Reff = incidence.polarization * Rs + (1.0 - incidence.polarization) * Rp;
    
    double Ts   = 1.0 - Rs;
    double Tp   = 1.0 - Rp;
    double Teff = 1.0 - Reff;

    reflection.energy = Reff * incidence.energy;
    refraction.energy = Teff * incidence.energy;

    return {reflection, refraction};
}

inline float maxwell_boltzmann_a_from_mode(float mode)
{
    return mode / M_SQRT2;
}

inline float maxwell_boltzmann_pdf(
    float mode,
    float x)
{
    float a = maxwell_boltzmann_a_from_mode(mode);
    // M_SQRT2
    // np.sqrt(2/np.pi) * x**2.0 * np.exp(-x**2.0 / (2*a**2)) / (a**3)
    const float xx = x*x;
    const float aa = a*a;
    const float aaa = a*a*a;
    return sqrt(2.0 / M_PI) * xx * exp(- xx / (2 * aa)) / aaa;
}

/**
 * @brief computes the total energy of back reflection
 * 
 * @param incidence_angle radian
 * @param energy 
 * @param diffuse 
 * @param specular 
 * @return float 
 */
inline float back_reflection_shader(
    float incidence_angle, 
    float energy,
    float diffuse,
    float specular_fac, 
    float specular_exp)
{
    // Diffuse: Lambertian model - NdotL
    // NdotL = v*w. for our case v*w = ||v|| * ||w|| * cos(incidence_angle) = cos(incidence_angle)
    // incidence_angle -> 0 - pi/2
    // I_diffuse 1 -> 0
    float IdotR = cos(incidence_angle);
    float I_diffuse = 1.0;
    float I_specular = pow(IdotR, specular_exp);

    // polynom
    float I_total = diffuse * I_diffuse + specular_fac * I_specular;

    return I_total * energy;
}

// inline float viewpoint_shader(

// )


/**
 * @brief Compute the fresnel reflections for a batch of incidences
 * 
 * @param incidences 
 * @param surface_normals 
 * @param object_ids 
 * @param materials 
 * @param material_velocities 
 * @return std::vector<DirectedWave> 
 */
std::vector<DirectedWave> fresnel(
    const std::vector<DirectedWave>&        incidences,
    const rmagine::MemView<rmagine::Vector> surface_normals,
    const rmagine::MemView<unsigned int>    object_ids, 
    const unsigned int*                     materials,
    const double*                           material_velocities,
    unsigned int                            material_id_air,
    float                                   wave_energy_threshold);

std::vector<DirectedWave> fresnel(
    const std::vector<DirectedWave>&        incidences,
    const rmagine::MemView<rmagine::Vector> surface_normals,
    const rmagine::MemView<unsigned int>    object_ids, 
    const int*                              materials,
    const double*                           material_velocities,
    int                                     material_id_air,
    float                                   wave_energy_threshold);

rmagine::OnDnModel make_model(
    const std::vector<DirectedWave>& waves);

rmagine::OnDnModel make_model(
    const std::vector<std::vector<DirectedWave> >& waves);

std::vector<DirectedWave> sample_cone_local(
    const DirectedWave& wave_ex, 
    float width,
    int n_samples,
    int sample_dist,
    float p_in_cone);

rmagine::Transform ray_to_transform(
    rmagine::Vector orig, 
    rmagine::Vector dir);

rmagine::Quaternion polar_to_quat(
    float phi, float theta);

std::vector<DirectedWave> sample_cone(
    const DirectedWave& wave_mean, 
    float width,
    int n_samples,
    int sample_dist,
    float p_in_cone);

rm::Memory<rm::Vector> sample_cone(
    const rm::Vector& ray_dir_mean, 
    float width,
    int n_samples,
    int sample_dist,
    float p_in_cone);

// std::vector<std::vector<DirectedWave> > sample_cones(
//     const std::vector<DirectedWave>& wave_means, 
//     float width,
//     int n_samples,
//     int sample_dist,
//     float p_in_cone)
// {
//     std::vector<std::vector<DirectedWave> > ret;
//     return ret;
// }

inline void normalize_inplace(std::vector<float>& data)
{
    // sum
    float data_sum = 0.0;
    for(size_t i=0; i<data.size(); i++)
    {
        data_sum += data[i];
    }

    // normalize        
    for(size_t i=0; i<data.size(); i++)
    {
        data[i] /= data_sum;
    }
}

inline std::vector<float> make_denoiser_triangular(
    int width,
    int mode)
{
    std::vector<float> verschmierer(width);

    float verschmierer_min = 0.0;
    float verschmierer_max = 1.0;

    for(size_t i=0; i<width; i++)
    {
        float p;
        if(i <= mode)
        {
            p = static_cast<float>(i) / static_cast<float>(mode);
        } else {
            p = 1.0 - ((static_cast<float>(i) - static_cast<float>(mode) ) 
                / (static_cast<float>(width) - static_cast<float>(mode) ));
        }
        
        verschmierer[i] = p * verschmierer_max + (1.0 - p) * verschmierer_min;
    }

    normalize_inplace(verschmierer);
    return verschmierer;
}

inline std::vector<float> make_denoiser_gaussian(
    int width,
    int mode)
{
    std::vector<float> verschmierer(width);

    float verschmierer_min = 0.0;
    float verschmierer_max = 1.0;

    for(size_t i=0; i < width; i++)
    {
        float p;
        if(i <= mode)
        {
            p = static_cast<float>(i) / static_cast<float>(mode);
        } else {
            p = 1.0 - ((static_cast<float>(i) - static_cast<float>(mode) ) 
                / (static_cast<float>(width) - static_cast<float>(mode) ));
        }
        
        verschmierer[i] = p * verschmierer_max + (1.0 - p) * verschmierer_min;
    }

    normalize_inplace(verschmierer);
    return verschmierer;
}

inline std::vector<float> make_denoiser_maxwell_boltzmann(
    int width,
    int mode)
{
    std::vector<float> verschmierer(width);

    for(size_t i=0; i < width; i++)
    {
        // float X = static_cast<float>(mode + i) - static_cast<float>(width) / 2.0;
        verschmierer[i] = maxwell_boltzmann_pdf((float)mode, (float)i);
    }
    
    normalize_inplace(verschmierer);
    return verschmierer;
}



} // namespace radarays_ros

#endif // RADARAYS_ROS_RADAR_ALGORITHMS_H