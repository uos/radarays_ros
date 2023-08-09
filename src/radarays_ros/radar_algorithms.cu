#include "radarays_ros/radar_algorithms.cuh"
#include <iostream>

namespace radarays_ros
{


/**
 * @brief computes the total energy of back reflection
 * 
 * @param incidence_angle radian
 * @param energy 
 * @param diffuse 
 * @param specular 
 * @return float 
 */

__device__ __forceinline__
float back_reflection_shader(
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

__device__ __forceinline__
double get_incidence_angle(
    const rm::Vector& surface_normal,
    const DirectedWave& incidence)
{
    return acos((-incidence.ray.dir).dot(surface_normal));
}

__device__ __forceinline__
double get_incidence_angle(
    const rm::Vector& surface_normal,
    const rm::Vector& incidence_dir)
{
    return acos((-incidence_dir).dot(surface_normal));
}


__global__ 
void propagate_waves_kernel(
    const RadarMaterial* materials,
    const int* object_materials,
    int material_id_air,

    const DirectedWave* waves,
    unsigned int n_waves,
    const uint8_t* res_hits,
    const float* res_ranges, 
    const rm::Vector3* res_normals, 
    const unsigned int* res_object_ids,
    
    Signal* signals,
    DirectedWave* waves_new,
    uint8_t* waves_new_mask)
{
    const float wave_energy_threshold = 0.001;
    const float skip_dist = 0.001;

    unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;
    unsigned int refractionid = tid * 2 + 0;
    unsigned int reflectionid = tid * 2 + 1;

    const uint8_t res_hit = res_hits[tid];
    const float res_range = res_ranges[tid];
    rm::Vector3 surface_normal = res_normals[tid];
    const unsigned int obj_id = res_object_ids[tid];
    

    if(obj_id > 10000)
    {
        // finish and mark
        waves_new_mask[refractionid] = 0;
        waves_new_mask[reflectionid] = 0;
        return;
    }

    if(res_hit == 0)
    {
        // finish and mark
        waves_new_mask[refractionid] = 0;
        waves_new_mask[reflectionid] = 0;
        return;
    }

    // 1. move wave to incident
    DirectedWave incidence = waves[tid];
    {
        incidence.ray.orig = incidence.ray.orig + incidence.ray.dir * res_range;
        incidence.time += res_range / incidence.velocity;
    }

    // 2. split to reflection and refraction
    DirectedWave reflection = incidence;
    DirectedWave refraction = incidence;

    // if wave was in air, switch to new material
    // else if wave was in material, switch to air (is this right ?)
    if(incidence.material_id == material_id_air)
    {
        refraction.material_id = object_materials[obj_id];
    } else {
        refraction.material_id = material_id_air;
    }

    float v_refraction = 1.0;

    if(incidence.material_id != refraction.material_id)
    {
        v_refraction = materials[refraction.material_id].velocity;
    } else {
        v_refraction = incidence.velocity;
    }


    // 3. fresnel
    {
        const double v1 = incidence.velocity;
        const double v2 = v_refraction;

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
    }

    // 4. returning signals
    if(reflection.energy > wave_energy_threshold)
    {
        // there is some energy reflected, so let it return
        if(reflection.material_id == material_id_air)
        {
            // 1. signal travelling back along the pass
            auto material = materials[refraction.material_id];
            double incidence_angle = get_incidence_angle(
                surface_normal, incidence);
             // 1. signal traveling over path
            double return_energy_path = back_reflection_shader(
                incidence_angle,
                reflection.energy,
                material.ambient, // ambient
                material.diffuse, // diffuse
                material.specular // specular
            );

            float time_back = incidence.time * 2.0;

            Signal sig;
            sig.time = incidence.time * 2.0;
            sig.strength = return_energy_path;
            signals[tid] = sig;
        }


        { // move with skip_dist and add to new waves
            reflection.ray.orig = reflection.ray.orig + reflection.ray.dir * skip_dist;
            reflection.time += skip_dist / reflection.velocity;
        }

        waves_new[reflectionid] = reflection;
        waves_new_mask[reflectionid] = 1;
    } else {
        waves_new_mask[reflectionid] = 0;
    }

    if(refraction.energy > wave_energy_threshold)
    {
        { // move with skip_dist and add to new waves
            refraction.ray.orig = refraction.ray.orig + refraction.ray.dir * skip_dist;
            refraction.time += skip_dist / refraction.velocity;
        }

        waves_new[refractionid] = refraction;
        waves_new_mask[refractionid] = 1;
    } else {
        waves_new_mask[refractionid] = 0;
    }


}

void propagate_waves(
    const rm::MemView<RadarMaterial, rm::VRAM_CUDA>& materials,
    const rm::MemView<int, rm::VRAM_CUDA>& object_materials,
    int material_id_air,

    const rm::MemView<DirectedWave, rm::VRAM_CUDA>& waves,
    const rm::MemView<uint8_t, rm::VRAM_CUDA>& hits,
    const rm::MemView<float, rm::VRAM_CUDA>& ranges,
    const rm::MemView<rm::Vector, rm::VRAM_CUDA>& normals,
    const rm::MemView<unsigned int, rm::VRAM_CUDA>& object_ids,
    
    rm::MemView<Signal, rm::VRAM_CUDA>& signals,
    rm::MemView<DirectedWave, rm::VRAM_CUDA>& waves_new,
    rm::MemView<uint8_t, rm::VRAM_CUDA>& waves_new_mask)
{
    propagate_waves_kernel<<<waves.size(),1>>>(
        materials.raw(),
        object_materials.raw(),
        material_id_air,

        waves.raw(), 
        waves.size(), 
        hits.raw(),
        ranges.raw(),
        normals.raw(),
        object_ids.raw(),
        
        signals.raw(),
        waves_new.raw(),
        waves_new_mask.raw()
    );
}


__global__ 
void move_waves_kernel(
    rm::Vector* origs,
    rm::Vector* dirs,
    DirectedWaveAttributes* attr,
    unsigned int n_waves,
    const float* ranges,
    const uint8_t* mask)
{
    unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if(tid < n_waves && mask[tid])
    {
        origs[tid] = origs[tid] + dirs[tid] * ranges[tid];
        attr[tid].time += ranges[tid] / attr[tid].velocity;
    }
}

void move_waves(
    rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& wave_origs,
    rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& wave_dirs,
    rm::MemoryView<DirectedWaveAttributes, rm::VRAM_CUDA>& wave_attributes,
    const rm::MemoryView<float, rm::VRAM_CUDA>& distances,
    const rm::MemoryView<uint8_t, rm::VRAM_CUDA>& mask)
{
    constexpr unsigned int blockSize = 64;
    const unsigned int gridSize = (wave_origs.size() + blockSize - 1) / blockSize;

    move_waves_kernel<<<gridSize, blockSize>>>(
        wave_origs.raw(),
        wave_dirs.raw(),
        wave_attributes.raw(),
        wave_origs.size(),
        distances.raw(),
        mask.raw()
    );
}


__global__ 
void signal_shader_kernel(
    const RadarMaterial* materials,
    const int* object_materials,
    int material_id_air,

    const rm::Vector* dirs,
    const DirectedWaveAttributes* attr,
    unsigned int n_waves,

    const uint8_t* hits,
    const rm::Vector* surface_normals,
    const unsigned int* object_ids,

    Signal* signals
    )
{
    unsigned int tid = blockIdx.x * blockDim.x + threadIdx.x;
    if(tid < n_waves && hits[tid])
    {
        const rm::Vector incidence_dir = dirs[tid];
        const DirectedWaveAttributes incidence_attr = attr[tid];
        rm::Vector surface_normal = surface_normals[tid];
        const unsigned int obj_id = object_ids[tid];

        // 2. split to reflection and refraction
        DirectedWaveAttributes reflection_attr = incidence_attr;
        DirectedWaveAttributes refraction_attr = incidence_attr;
        

        // if wave was in air, switch to new material
        // else if wave was in material, switch to air (is this right ?)
        if(incidence_attr.material_id == material_id_air)
        {
            refraction_attr.material_id = object_materials[obj_id];
        } else {
            refraction_attr.material_id = material_id_air;
        }

        float v_refraction = 1.0;

        if(incidence_attr.material_id != refraction_attr.material_id)
        {
            v_refraction = materials[refraction_attr.material_id].velocity;
        } else {
            v_refraction = incidence_attr.velocity;
        }

        // 3. fresnel
        {
            const double v1 = incidence_attr.velocity;
            const double v2 = v_refraction;

            const double n1 = v2;
            const double n2 = v1;

            double incidence_angle = acos((-incidence_dir).dot(surface_normal));
            
            // refraction
            rm::Vector refraction_dir = rmagine::Vector::Zeros();
            refraction_attr.velocity = v2;

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
                    if(surface_normal.dot(incidence_dir) > 0.0)
                    {
                        surface_normal = -surface_normal;
                    }
                    if(n2 > 0.0)
                    {
                        double n12 = n1 / n2;
                        double c = cos(incidence_angle);
                        refraction_dir = incidence_dir * n12 
                                        + surface_normal * (n12 * c - sqrt(1 - n12*n12 * ( 1 - c*c ) ) );
                    }
                }
            }
            
            // // energy
            double refraction_angle = acos((refraction_dir).dot(-surface_normal));

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

            double Rs = rs * rs;
            double Rp = rp * rp;
            
            double Reff = incidence_attr.polarization * Rs 
                + (1.0 - incidence_attr.polarization) * Rp;

            reflection_attr.energy = Reff * incidence_attr.energy;
        }

        // there is some energy reflected, so let it return
        {
            // 1. signal travelling back along the pass
            auto material = materials[refraction_attr.material_id];
            double incidence_angle = get_incidence_angle(
                surface_normal, incidence_dir);
             // 1. signal traveling over path
            double return_energy_path = back_reflection_shader(
                incidence_angle,
                reflection_attr.energy,
                material.ambient, // ambient
                material.diffuse, // diffuse
                material.specular // specular
            );

            float time_back = incidence_attr.time * 2.0;

            Signal sig;
            sig.time = incidence_attr.time * 2.0;
            sig.strength = return_energy_path;
            signals[tid] = sig;
        }
    

    }
}


void signal_shader(
    const rm::MemView<RadarMaterial, rm::VRAM_CUDA>& materials,
    const rm::MemView<int, rm::VRAM_CUDA>& object_materials,
    int material_id_air,

    const rm::MemView<rm::Vector, rm::VRAM_CUDA>& dirs,
    const rm::MemView<DirectedWaveAttributes, rm::VRAM_CUDA>& attr,
    const rm::MemView<uint8_t, rm::VRAM_CUDA>& hits,
    const rm::MemView<rm::Vector, rm::VRAM_CUDA>& surface_normals,
    const rm::MemView<unsigned int, rm::VRAM_CUDA>& object_ids,

    rm::MemView<Signal, rm::VRAM_CUDA>& signals)
{
    constexpr unsigned int blockSize = 64;
    const unsigned int gridSize = (dirs.size() + blockSize - 1) / blockSize;

    signal_shader_kernel<<<gridSize, blockSize>>>(
        materials.raw(),
        object_materials.raw(),
        material_id_air,

        dirs.raw(),
        attr.raw(),
        dirs.size(),
        
        hits.raw(),
        surface_normals.raw(),
        object_ids.raw(),

        signals.raw()
    );
}


} // namespace radarays_ros

