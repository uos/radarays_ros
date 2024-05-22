#ifndef RADARAYS_WAVE_SHADERS_H
#define RADARAYS_WAVE_SHADERS_H


#include "definitions.h"

#include "radar_types.h"
#include "radar_math.h"
#include <rmagine/math/types.h>
#include <vector>


#include <algorithm>




using std::max;
using std::min;


namespace rm = rmagine;


namespace radarays_ros
{

struct SurfacePatch;
struct Material;


// template for a BRDF function
using BRDF = std::function<float( // returns reflectance value
            const DirectedWave&, // incident wave
            const rm::Vector3&, // surface normal
            const Material*, // surface material
            const rm::Vector3& // out_direction
            )>;

struct Material
{
    // refractive index of material
    // can be computed by n = c/v
    // set to a really high value to
    float n;

    // value between 0-1. how much of the transmitted energy is absorped by the material
    // extreme example:
    // - black color is not reflecting light nor does it transmit the light through the material as glass
    float absorption;
    // TODO: Check the following idea
    // rather than having a fixed parameter have absorption per distance instead
    // like this you can calculate the actual absorpt part after the next intersection test
    // 1. continue shooting
    // 2. compute the the absoption via absorption = 1-(1-absorption_per_m)^(meter)
    // see if the energy falls below a value. If yes, finish; if no, continue
    // special case: if absorption is equal to 1, it is clear that everything is absorpt. We can stop without doing the first intersection test

    // variable number of parameters used in brdf func
    std::vector<float> brdf_params;
    // user specific brdf function
    BRDF brdf_func;

    float brdf(const DirectedWave& incidence, const rm::Vector& normal, const rm::Vector3& out_direction) const
    {
        return brdf_func(incidence, normal, this, out_direction);
    }
};

// use this for convencience
// 
struct Intersection
{
    // surface normal
    rm::Vector normal;

    const Material* material;

    float brdf(const DirectedWave& incidence, 
        const rm::Vector3& out_direction) const
    {
        return material->brdf(incidence, normal, out_direction);
    }
};

// // legacy code - deprectated
// struct SurfacePatch
// {
//     // surface normal
//     rm::Vector normal;

//     // index of refraction for material of positive normal direction
//     float n1;
//     // index of refraction for material of negative normal direction
//     float n2;

//     std::vector<float> reflection_parameters;
//     // reflection parameters dependend on used type of BRDF
//     // Blinn-Phong
//     // - 0: k_L = diffuse amount [0,1]
//     // - 1: k_g = glossy amount [0,1]
//     // - 2: s = specular amount (exponent) [0,inf]
//     // Cook-Torrance
//     // - 0: k_L = diffuse amount in [0,1]
//     // - 1: roughness in [0,1]
// };

float compute_fzero(const DirectedWave& incidence, const Material& mat)
{
    return compute_fzero(incidence.indexOfRefraction(), mat.n);
}

float fresnel_reflection_coefficient(float h_dot_in, float F0)
{
    return F0 + (1 - F0) * pow(1 - h_dot_in, 5.0);
}

float fresnel_reflection_coefficient(
    rm::Vector half, // half vector
    rm::Vector3 in_direction, // in direction (inversed?)
    float F0)
{   
    return fresnel_reflection_coefficient(half.dot(-in_direction), F0);
}

// float fresnel_reflection_coefficient(
//     rm::Vector3 in_direction, 
//     Intersection intersection, 
//     rm::Vector out_direction)
// {
//     rm::Vector3 half = (-in_direction + out_direction).normalize(); // half-vector
//     const float F0 = compute_fzero(intersection);
//     return fresnel_reflection_coefficient(half, in_direction, F0);
// }

//-------------------------------------------------------------------------//
// Cook-Torrance and related microfacet BRDFs
float Dfunc( float roughness, float n_dot_h )
{
	// The original paper of Cook and Torrance says:
	// float D = (1/(m*m * pow( cos(alpha), 4.0))) * exp (-pow(tan(alpha)/m, 2.0));
	// with alpha = the angle between H and N

	// The book Real-Time Rendering 4 (eq 9.35) says:
	float D =
		max(0.f, n_dot_h) // This is == Xi+(n,m) in the book
		/ (M_PI * roughness*roughness * pow(n_dot_h , 4.0))
		* exp(
		      (n_dot_h*n_dot_h - 1)
		      /
		      (roughness*roughness * n_dot_h*n_dot_h)
		      )
		;
	// The book says dot(n,m) but that is the same as dot(n,h) since
	// only micronormals m that are exactly = h contribute.
	// The term in the exponent is in fact equivalent to the tan term in
	// cookpaper.pdf, since tan^2(theta) == ((1-theta^2)/theta^2)
	// See http://www.codinglabs.net/article_physically_based_rendering_cook_torrance.aspx
	return D;
}


float Dfunc_GGX( float roughness, float n_dot_h )
{
	// This is the GGX distribution function (eq 9.41) from Real-Time Rendering 4
	float D =
		(max(0.f, n_dot_h) * roughness*roughness)
		/
		( M_PI * pow((1.f + n_dot_h*n_dot_h * (roughness*roughness-1.f)), 2.f) );
	return D;
}

float Gfunc( float n_dot_h, float o_dot_h,
             float n_dot_o, float n_dot_i )
{
	float G1 = 2 * (n_dot_h * n_dot_o) / (o_dot_h);
	float G2 = 2 * (n_dot_h * n_dot_i) / (o_dot_h);
	float G = min( G1, G2 );
	G = min(1.f, G);
	return G;
}


//-------------------------------------------------------------------------//
// Lambertian BRDF
float lambertian_brdf(
    const DirectedWave& incidence, 
    const rm::Vector& normal,
    const Material* material,
    const rm::Vector3& out_direction)
{
    return incidence.energy / M_PI; // why devided by pi?
}

//-------------------------------------------------------------------------//
// Cook torrance BRDF
float cook_torrance_brdf(
    const DirectedWave& incidence, 
    const rm::Vector& normal,
    const Material* material,
    const rm::Vector3& out_direction)
{
    float k_L = material->brdf_params[0]; // diffuse amount
    float k_g = 1.0 - k_L; // specular amount
    float roughness = material->brdf_params[1]; // roughness
    // TODO: check how to determine k_s
    // float k_s = surface.reflection_parameters[2] // non-metallic-ness - TODO: is this right?
    // or from actual surface
    float k_s = compute_fzero(incidence.indexOfRefraction(), material->n);

    const rm::Vector half = (-incidence.ray.dir + out_direction).normalize();

    const float n_dot_h = max(0.f, normal.dot(half));
	const float n_dot_o = max(0.f, normal.dot(out_direction));
	const float n_dot_i = max(0.f, normal.dot(-incidence.ray.dir));
	const float o_dot_h = max(0.f, out_direction.dot(half));

    float F = fresnel_reflection_coefficient(half, incidence.ray.dir, k_s);
    float D = Dfunc( roughness, n_dot_h );
    float G = Gfunc( n_dot_h, o_dot_h, n_dot_o, n_dot_i );

    float r_s = (F*G*D) / ( 4*n_dot_i*n_dot_o );

    float result =
		k_L * incidence.energy / M_PI +
		k_g * incidence.energy * r_s;

    return result;
}

//-------------------------------------------------------------------------//
// Blinn-Phong BRDF
float blinn_phong_brdf(
    const DirectedWave& incidence, 
    const rm::Vector& normal,
    const Material* material,
    const rm::Vector3& out_direction)
{
    float k_L = material->brdf_params[0]; // diffuse amount
	float k_g = material->brdf_params[1]; // glossy amount
	float s   = material->brdf_params[2]; // specular exponent
	rm::Vector3 w_h = (-incidence.ray.dir + out_direction).normalize();
    float k_s = ((8.0 + s) / 8.0) * pow( max(0.f, normal.dot(w_h)), s); // specular amount

    float result;

    // { // use only the reflection part of the energy?
    //     float reflected_energy_part = fresnel_reflection_coefficient(incidence.ray.dir, surface, out_direction);
    //     float reflected_energy = incidence.energy * reflected_energy_part;
    //     result = k_L * reflected_energy + (k_g * k_s) * reflected_energy;
    // }
    
    { // or use the whole "input energy"?
        result = k_L * incidence.energy + (k_g * k_s) * incidence.energy;
    }

    return result / M_PI;
}

} // namespace radarays_ros


#endif // RADARAYS_WAVE_SHADERS_H