#include "radarays_ros/radar_types.h"


namespace radarays_ros
{

double DirectedWave::getVelocity() const
{
    return M_C_IN_M_PER_NANOSECOND / material->n;
}

DirectedWave& DirectedWave::moveInplace(double distance)
{
    ray.orig = ray.orig + ray.dir * distance;
    time += distance / getVelocity();
    energy *= (1.0 - pow(material->transmittance, distance));
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

} // namespace radarays_ros


