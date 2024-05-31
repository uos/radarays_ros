#include "radarays_ros/radar_types.h"
#include <cassert>
#include <algorithm>

namespace radarays_ros
{

double DirectedWave::getVelocity() const
{
    return M_C_IN_M_PER_NANOSECOND / material->n;
}

double DirectedWave::getRefractiveIndex() const 
{
    return material->n;
}

DirectedWave& DirectedWave::moveInplace(double distance)
{
    ray.orig = ray.orig + ray.dir * distance;
    time += distance / getVelocity();
    const double energy_loss = pow(material->transmittance, distance);
    energy *= energy_loss;
    return *this;
}

DirectedWave DirectedWave::move(double distance) const
{
    DirectedWave wave = *this;
    wave.moveInplace(distance);
    return wave;
}

float Material::brdf(const DirectedWave& incidence, const rm::Vector& normal, const rm::Vector3& out_direction) const
{
    return brdf_func(incidence, normal, this, out_direction);
}

float Intersection::brdf(const DirectedWave& incidence, 
    const rm::Vector3& out_direction) const
{
    return material->brdf(incidence, normal, out_direction);
}

std::pair<DirectedWave, DirectedWave> Intersection::fresnel(const DirectedWave& incidence) const
{
    const double n1 = incidence.getRefractiveIndex();
    const double n2 = material->n;

    // std::cout << "Changing from material n1=" << n1 << " to n2=" << n2 << std::endl; 

    assert(n1 > 0.0);
    assert(n2 > 0.0);
    const double n12 = n1 / n2;
    // air -> water: n1 / n2 = 1 / 1.3 = 0.7

    // init as copy
    DirectedWave reflection = incidence;
    DirectedWave transmission = incidence;
    transmission.material = material;
    

    rm::Vector normal_oriented = normal;
    
    // reflection
    reflection.ray.orig = incidence.ray.orig;
    reflection.ray.dir = incidence.ray.dir + normal_oriented * 2.0 * (-normal_oriented).dot(incidence.ray.dir);

    // is this correct?
    double i_dot_n = (-incidence.ray.dir).dot(normal);


    assert(i_dot_n >= 0.0);
    // if(i_dot_n > 1.0)
    // {
    //     std::cout << "WARNING: i_dot_n > 1: " << i_dot_n << std::endl;
    //     std::cout << "- i: " << incidence.ray.dir.x << ", " << incidence.ray.dir.y << ", " << incidence.ray.dir.z << std::endl;
    //     std::cout << "- n: " << normal.x << ", " << normal.y << ", " << normal.z << std::endl;
    //     i_dot_n = 1.0;
    // }
    
    i_dot_n = std::clamp(i_dot_n, 0.0, 1.0);
    // assert(i_dot_n <= 1.00001);
    // std::cout << "i_dot_n: " << i_dot_n << std::endl;


    const double incidence_angle = acos(i_dot_n);
    // std::cout << "Incidence angle: " << incidence_angle << std::endl;

    // // refraction
    transmission.ray.orig = incidence.ray.orig;
    transmission.ray.dir = rmagine::Vector::Zeros();
    

    double angle_limit = 100.0;
    if(n1 > n2)
    {
        // in this case n12 is in [0,1]
        double angle_limit = asin(1.0 / n12);
    }
    assert(angle_limit == angle_limit);
    
    if(incidence_angle <= angle_limit)
    {
        transmission.ray.dir = incidence.ray.dir * n12 + normal_oriented * (n12 * i_dot_n - sqrt(1 - n12*n12 * ( 1 - i_dot_n*i_dot_n ) ) );
        const double t_dot_n = transmission.ray.dir.dot(-normal_oriented);
        
        // must point in different direction
        assert(t_dot_n >= 0.0);
        double refraction_angle = 0.0;
        if(t_dot_n < 1.0)
        {
            refraction_angle = acos(t_dot_n);
        }

        // compute energy parts
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
            rs = -sin(incidence_angle - refraction_angle) / sin(incidence_angle + refraction_angle);
            rp = tan(incidence_angle - refraction_angle) / tan(incidence_angle + refraction_angle);
        }

        double sp_ratio = 0.5;

        double Rs = rs * rs;
        double Rp = rp * rp;

        double Reff = incidence.polarization * Rs + (1.0 - incidence.polarization) * Rp;
        
        double Ts   = 1.0 - Rs;
        double Tp   = 1.0 - Rp;
        double Teff = 1.0 - Reff;

        reflection.energy = Reff;
        transmission.energy = Teff;
    } else {
        // total reflection
        reflection.energy = 1.0;
        transmission.energy = 0.0;
    }

    return {reflection, transmission};
}

} // namespace radarays_ros


