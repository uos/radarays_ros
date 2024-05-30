#ifndef RADARAYS_ROS_RADAR_ALGORITHMS_H
#define RADARAYS_ROS_RADAR_ALGORITHMS_H

#include <vector>
#include <random>
#include "radar_types.h"
#include "radar_math.h"

#include <rmagine/types/sensor_models.h>
#include <rmagine/types/Memory.hpp>

#include <math.h>

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
inline float lambertian_phong_simple(
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

/**
 * implementation explained in RadaRays paper
 * 
*/
inline float lambertian_phong(
    float incidence_angle,
    float energy,
    float A,
    float B,
    float C)
{
    const float S = 1.0 - A - B;
    const float Iret = A + B * cos(incidence_angle) + S * pow(cos(incidence_angle), C);
    return Iret * energy;
}

/**
 * A, B and C semantics depend on the used shader. Read the respective code docs
 * 
 * back reflection shading is a special case where the light source is at the same point as the sensor
*/
inline float back_reflection_shader(
    float incidence_angle, 
    float energy,
    float A,
    float B, 
    float C)
{
    return lambertian_phong_simple(incidence_angle, energy, A, B, C);
}

// //-------------------------------------------------------------------------//
// // Cook-Torrance and related microfacet BRDFs
// float Dfunc( float roughness, float n_dot_h )
// {
// 	// The original paper of Cook and Torrance says:
// 	// float D = (1/(m*m * pow( cos(alpha), 4.0))) * exp (-pow(tan(alpha)/m, 2.0));
// 	// with alpha = the angle between H and N

// 	// The book Real-Time Rendering 4 (eq 9.35) says:
// 	float D =
// 		std::max(0.f, n_dot_h) // This is == Xi+(n,m) in the book
// 		/ (M_PI * roughness*roughness * pow(n_dot_h , 4.0))
// 		* exp(
// 		      (n_dot_h*n_dot_h - 1)
// 		      /
// 		      (roughness*roughness * n_dot_h*n_dot_h)
// 		      )
// 		;
// 	// The book says dot(n,m) but that is the same as dot(n,h) since
// 	// only micronormals m that are exactly = h contribute.
// 	// The term in the exponent is in fact equivalent to the tan term in
// 	// cookpaper.pdf, since tan^2(theta) == ((1-theta^2)/theta^2)
// 	// See http://www.codinglabs.net/article_physically_based_rendering_cook_torrance.aspx
// 	return D;
// }

// inline float Dfunc_GGX( float roughness, float n_dot_h )
// {
// 	// This is the GGX distribution function (eq 9.41) from Real-Time Rendering 4
// 	float D =
// 		(std::max(0.f, n_dot_h) * roughness*roughness)
// 		/
// 		( M_PI * pow((1 + n_dot_h*n_dot_h * (roughness*roughness-1)), 2) );
// 	return D;
// }

// inline float Gfunc( float n_dot_h, float o_dot_h,
//              float n_dot_o, float n_dot_i )
// {
// 	float G1 = 2 * (n_dot_h * n_dot_o) / (o_dot_h);
// 	float G2 = 2 * (n_dot_h * n_dot_i) / (o_dot_h);
// 	float G = std::min( G1, G2 );
// 	G = std::min( 1.f, G );
// 	return G;
// }

// inline double clamp(double d, double min, double max) 
// {
//   const double t = d < min ? min : d;
//   return t > max ? max : t;
// }

// inline float clamp(float d, float min, float max) {
//   const float t = d < min ? min : d;
//   return t > max ? max : t;
// }

// // Using eq (9.34) from RTR4 adapted to radar waves instead of colors
// inline float cook_torrance_brdf(
//     const float int_energy,
//     const rm::Vector in_direction,
//     const rm::Vector out_direction, 
//     const rm::Vector normal,
//     const float A, // diffuse in [0, 1]
//     const float B // roughness in [0, 1]
//     )
// {
//     const rm::Vector in_dir_inv = -in_direction;

// 	const float k_L = A;
// 	const float k_g = 1 - k_L;
//     const float roughness = B; 

//     rm::Vector h = (in_dir_inv + out_direction).normalize(); // half-vector

// 	float n_dot_h = std::max(0.f, normal.dot(h));
// 	float n_dot_o = std::max(0.f, normal.dot(out_direction));
// 	float n_dot_i = std::max(0.f, normal.dot(in_dir_inv));
// 	float o_dot_h = std::max(0.f, out_direction.dot(h) );

//     // fresnel is already hidden in the energy term 
// 	float D = Dfunc( roughness, n_dot_h ); // Use `Dfunc` or `Dfunc_GGX`
// 	float G = Gfunc( n_dot_h, o_dot_h, n_dot_o, n_dot_i );
//     // float F = ...

//     // why the times 1?
// 	float I = k_L + k_g * 1 * ( (G*D) / ( 4.0 * n_dot_i * n_dot_o ) );

// 	float Iref = clamp(I, 0.f, 1.f);
// 	return Iref * int_energy;
// }

// /**
//  * returns a density value of energy function with integral -> int_energy
// */
// inline float reflection_shader(
//     float int_energy,
//     const rm::Vector incidence_dir,
//     const rm::Vector reflection_dir,
//     const rm::Vector normal,
//     float A,
//     float B,
//     float C)
// {
//     return cook_torrance_brdf(int_energy, incidence_dir, reflection_dir, normal, A, B);
// }


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


std::vector<rm::Vector3> fibonacci_sphere(
    const size_t& n_samples);


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

    // sum is 1
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