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

float compute_fzero(const Material* m1, const Material* m2)
{
    return compute_fzero(m1->n, m2->n);
}

float compute_fzero(const DirectedWave& incidence, const Material* mat)
{
    return compute_fzero(incidence.material, mat);
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

///
// RadaRays BRDF
// similar to phong
float radarays_brdf(
    const DirectedWave& incidence, 
    const rm::Vector& normal,
    const Material* material,
    const rm::Vector3& out_direction)
{
    const float A = material->brdf_params[0];
    const float B = material->brdf_params[1];
    const float C = material->brdf_params[2];

    const float i_dot_n = incidence.ray.dir.dot(-normal);
    const float i_dot_o = incidence.ray.dir.dot(-out_direction);
    const float o_dot_n = out_direction.dot(normal);

    const float dist_from_perfect_reflection = fabs(i_dot_n - o_dot_n);

    const float S = 1.0 - A - B;
    const float Iret = A + B * cos(dist_from_perfect_reflection) + S * pow(cos(dist_from_perfect_reflection), C);
    return Iret; // why devided by pi?
}

//-------------------------------------------------------------------------//
// Lambertian BRDF
float lambertian_brdf(
    const DirectedWave& incidence, 
    const rm::Vector& normal,
    const Material* material,
    const rm::Vector3& out_direction)
{
    float k_L = material->brdf_params[0];
    return k_L / M_PI; // why devided by pi?
}

//-------------------------------------------------------------------------//
// Cook torrance BRDF
// http://www.codinglabs.net/article_physically_based_rendering_cook_torrance.aspx
float cook_torrance_brdf(
    const DirectedWave& incidence,
    const rm::Vector& normal,
    const Material* material,
    const rm::Vector3& out_direction)
{
    // // black plastic dice
    // roughness = 0.1;
    // k_L = 0.7;
	// k_g = 0.3;
	// rho = 0.02;
	// k_s = 1.0; compute_fzero(incidence.material, material)

    float k_L = material->brdf_params[0]; // diffuse amount (k_d * c)
    float k_s = 1 - k_L; // specular amount
    float roughness = material->brdf_params[1]; // roughness

    // float r_d = 1.0;

    // TODO: check how to determine k_s
    // float k_s = surface.reflection_parameters[2] // non-metallic-ness - TODO: is this right?
    // or from actual surface
    float F0 = compute_fzero(incidence.material, material);

    const rm::Vector half = (-incidence.ray.dir + out_direction).normalize();

    const float n_dot_h = max(0.f, normal.dot(half));
	const float n_dot_o = max(0.f, normal.dot(out_direction));
	const float n_dot_i = max(0.f, normal.dot(-incidence.ray.dir));
	const float o_dot_h = max(0.f, out_direction.dot(half));

    const float F = fresnel_reflection_coefficient(half, incidence.ray.dir, F0);
    const float D = Dfunc( roughness, n_dot_h );
    const float G = Gfunc( n_dot_h, o_dot_h, n_dot_o, n_dot_i);
    // or f_cook_toorance
    const float r_s = (F*G*D) / (4*n_dot_i*n_dot_o);

    const float result = k_L / M_PI + k_s * r_s;

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
    const float k_L = material->brdf_params[0]; // diffuse amount
	const float k_g = material->brdf_params[1]; // glossy amount
	const float s   = material->brdf_params[2]; // specular exponent (shininess)
	rm::Vector3 w_h = (-incidence.ray.dir + out_direction).normalize();
    const float n_dot_h = max(0.f, normal.dot(w_h));
    
    const float kEnergyConservation = ( 8.0 + s ) / ( 8.0 ); 
    const float k_s = kEnergyConservation * pow(n_dot_h, s); // specular amount
    float result = k_L + k_g * k_s;
    return result / M_PI;
}

} // namespace radarays_ros


#endif // RADARAYS_WAVE_SHADERS_H