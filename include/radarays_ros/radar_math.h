#ifndef RADARAYS_ROS_RADAR_MATH_H
#define RADARAYS_ROS_RADAR_MATH_H

#include <math.h>
#include "definitions.h"

namespace radarays_ros
{

// lightspeed in vacuum
static constexpr double M_C_IN_M_PER_SECOND = 299792458.0;
static constexpr double M_C_IN_M_PER_NANOSECOND = 0.299792458;

// shortcut for standard units
static constexpr double M_C = M_C_IN_M_PER_SECOND;


// TODO: move to rmagine?
inline float erfinvf(float a)
{
    float p, r, t;
    t = fmaf(a, 0.0f - a, 1.0f);
    t = logf(t);
    
    if(fabsf(t) > 6.125f) { // maximum ulp error = 2.35793
        p =             3.03697567e-10f; //  0x1.4deb44p-32 
        p = fmaf(p, t,  2.93243101e-8f); //  0x1.f7c9aep-26 
        p = fmaf(p, t,  1.22150334e-6f); //  0x1.47e512p-20 
        p = fmaf(p, t,  2.84108955e-5f); //  0x1.dca7dep-16 
        p = fmaf(p, t,  3.93552968e-4f); //  0x1.9cab92p-12 
        p = fmaf(p, t,  3.02698812e-3f); //  0x1.8cc0dep-9 
        p = fmaf(p, t,  4.83185798e-3f); //  0x1.3ca920p-8 
        p = fmaf(p, t, -2.64646143e-1f); // -0x1.0eff66p-2 
        p = fmaf(p, t,  8.40016484e-1f); //  0x1.ae16a4p-1 
    } else { // maximum ulp error = 2.35002
        p =              5.43877832e-9f; //  0x1.75c000p-28 
        p = fmaf(p, t,  1.43285448e-7f); //  0x1.33b402p-23 
        p = fmaf(p, t,  1.22774793e-6f); //  0x1.499232p-20 
        p = fmaf(p, t,  1.12963626e-7f); //  0x1.e52cd2p-24 
        p = fmaf(p, t, -5.61530760e-5f); // -0x1.d70bd0p-15 
        p = fmaf(p, t, -1.47697632e-4f); // -0x1.35be90p-13 
        p = fmaf(p, t,  2.31468678e-3f); //  0x1.2f6400p-9 
        p = fmaf(p, t,  1.15392581e-2f); //  0x1.7a1e50p-7 
        p = fmaf(p, t, -2.32015476e-1f); // -0x1.db2aeep-3 
        p = fmaf(p, t,  8.86226892e-1f); //  0x1.c5bf88p-1 
    }

    r = a * p;
    return r;
}

inline float quantile(float p)
{
    return M_SQRT2 * erfinvf(2 * p - 1.0);
}

/**
 * Compute F0 by two refractive indices
*/
inline float compute_fzero(float n1, float n2)
{
    const float inner = (n1 - n2) / (n1 + n2);
    return inner * inner;
}

/**
 * Compute F0 by two wave velocities
 * 
 * Note: v1/v2 = n2/n1
*/
inline float compute_fzero_vel(float v1, float v2)
{
    return compute_fzero(v2, v1);
}


// Cauchy functions according to https://en.wikipedia.org/wiki/Cauchy%27s_equation

// is cauchy only for light?
inline float cauchy_n(float wl, float A, float B, float C)
{
    const float tmp = wl * wl;
    return A + B / tmp + C / (tmp * tmp);
}

inline float cauchy_n(float wl, float A, float B)
{
    return A + B / wl;
}

// TODO: we could 

inline void cauchy_estimate_params(
    float n1, float l1, // input sample 1 
    float n2, float l2, // input sample 2
    float& A, float& B)
{
    // n = A + B / l^2
    // we need two "samples" (n1,l1) and (n2,l2) to estimate A and B

    const float z1 = 1.0 / (l1 * l1);
    const float z2 = 1.0 / (l2 * l2);

    const float div = (z1 - z2);

    A = (-z2 * n1 + z1 * n2) / div;
    B = (n1 - n2) / div;
}

inline void cauchy_estimate_params(
    float n1, float l1,
    float n2, float l2,
    float n3, float l3,
    float& A, float& B, float& C)
{
    // Same as two sample version but with better approximation:
    // n = A + B / l^2 + C / l^4
    const float z1 = 1.0 / (l1 * l1);
    const float z2 = 1.0 / (l2 * l2);
    const float z3 = 1.0 / (l3 * l3);

    const float A11 =   (z2 * z3) / ((z1 - z2) * (z1 - z3));
    const float A12 = - (z1 * z3) / ((z1 - z2) * (z2 - z3));
    const float A13 =   (z1 * z2) / ((z1 - z3) * (z2 - z3));

    const float A21 = - (z2 + z3) / ((z1 - z2) * (z1 - z3));
    const float A22 =   (z1 + z3) / ((z1 - z2) * (z2 - z3));
    const float A23 =   (z1 + z2) / ((z1 - z3) * (z3 - z2));

    const float A31 =   (1.0)     / ((z1 - z2) * (z1 - z3));
    const float A32 = - (1.0)     / ((z1 - z2) * (z2 - z3));
    const float A33 =   (1.0)     / ((z1 - z3) * (z2 - z3));

    A = A11 * n1 + A12 * n2 + A13 * n3;
    B = A21 * n1 + A22 * n2 + A23 * n3;
    C = A31 * n1 + A32 * n2 + A33 * n3;
}

/**
 * If we know samples for waves of certain wave length (l) with their respective refractive index (n)
 * we can estimate the refractive index for other wavelengths using cauchy's equations
*/
inline float cauchy_estimate_n(
    float n1, float l1,
    float n2, float l2,
    float l3) // wavelength for which the refractive index should be estimated
{
    float A, B;
    cauchy_estimate_params(n1, l1, n2, l2, A, B);
    return cauchy_n(l3, A, B);
}

inline float cauchy_estimate_n(
    float n1, float l1,
    float n2, float l2,
    float n3, float l3,
    float l4)
{
    float A, B, C;
    cauchy_estimate_params(n1, l1, n2, l2, n3, l3, A, B, C);
    return cauchy_n(l4, A, B, C);
}


// transformations
Ray operator*(const rm::Transform& T, const Ray& ray);

DirectedWave operator*(const rm::Transform& T, const DirectedWave& wave);

std::vector<DirectedWave> operator*(const rm::Transform& T, const std::vector<DirectedWave>& wave);


// if both in and out dir are on the same side of the plane
inline bool are_same_medium(
    const rm::Vector& in_dir, 
    const rm::Vector& normal, 
    const rm::Vector& out_dir)
{
    return in_dir.dot(normal) * out_dir.dot(normal) >= 0.0;
}


} // namespace radarays_ros

#endif // RADARAYS_ROS_RADAR_MATH_H