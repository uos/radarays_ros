#include "radarays_ros/radar_algorithms.h"


namespace radarays_ros
{


std::vector<DirectedWave> fresnel(
    const std::vector<DirectedWave>&        incidences,
    const rmagine::MemView<rmagine::Vector> surface_normals,
    const rmagine::MemView<unsigned int>    object_ids, 
    const int*                              materials,
    const double*                           material_velocities,
    int                                     material_id_air,
    float                                   wave_energy_threshold)
{
    std::vector<DirectedWave> waves_new;

    for(size_t i=0; i < incidences.size(); i++)
    {
        // read ray data
        const DirectedWave incidence = incidences[i];
        const rmagine::Vector surface_normal = surface_normals[i].normalize();
        const unsigned int obj_id = object_ids[i];

        // reflection refractions
        // copy everything to a new wave called incidence

        if(obj_id > 10000)
        {
            continue;
        } 

        // inititalize
        DirectedWave reflection = incidence;
        DirectedWave refraction = incidence;

        // if wave was in air, switch to new material
        // else if wave was in material, switch to air (is this right ?)
        if(incidence.material_id == material_id_air)
        {
            refraction.material_id = materials[obj_id];
        } else {
            refraction.material_id = material_id_air;
        }

        float v_refraction = 1.0;

        if(incidence.material_id != refraction.material_id)
        {
            v_refraction = material_velocities[refraction.material_id];
        } else {
            v_refraction = incidence.velocity;
        }
        
        // Build surface patch
        auto res = fresnel(surface_normal, incidence, v_refraction);

        reflection.ray.dir = res.first.ray.dir;
        reflection.energy = res.first.energy;

        if(reflection.energy > wave_energy_threshold)
        {
            waves_new.push_back(reflection);
        }

        refraction.ray.dir = res.second.ray.dir;
        refraction.energy = res.second.energy;

        if(refraction.energy > wave_energy_threshold)
        {
            waves_new.push_back(refraction);
        }
    }

    return waves_new;
}

std::vector<DirectedWave> fresnel(
    const std::vector<DirectedWave>&        incidences,
    const rmagine::MemView<rmagine::Vector> surface_normals,
    const rmagine::MemView<unsigned int>    object_ids, 
    const unsigned int*                     materials,
    const double*                           material_velocities,
    unsigned int                            material_id_air,
    float                                   wave_energy_threshold)
{
    std::vector<DirectedWave> waves_new;

    for(size_t i=0; i < incidences.size(); i++)
    {
        // read ray data
        const DirectedWave incidence = incidences[i];
        const rmagine::Vector surface_normal = surface_normals[i].normalize();
        const unsigned int obj_id = object_ids[i];

        // reflection refractions
        // copy everything to a new wave called incidence

        if(obj_id > 10000)
        {
            continue;
        } 

        // inititalize
        DirectedWave reflection = incidence;
        DirectedWave refraction = incidence;

        // if wave was in air, switch to new material
        // else if wave was in material, switch to air (is this right ?)
        if(incidence.material_id == material_id_air)
        {
            refraction.material_id = materials[obj_id];
        } else {
            refraction.material_id = material_id_air;
        }

        float v_refraction = 1.0;

        if(incidence.material_id != refraction.material_id)
        {
            v_refraction = material_velocities[refraction.material_id];
        } else {
            v_refraction = incidence.velocity;
        }
        
        // Build surface patch
        auto res = fresnel(surface_normal, incidence, v_refraction);

        reflection.ray.dir = res.first.ray.dir;
        reflection.energy = res.first.energy;

        if(reflection.energy > wave_energy_threshold)
        {
            waves_new.push_back(reflection);
        }

        refraction.ray.dir = res.second.ray.dir;
        refraction.energy = res.second.energy;

        if(refraction.energy > wave_energy_threshold)
        {
            waves_new.push_back(refraction);
        }
    }

    return waves_new;
}

rmagine::OnDnModel make_model(
    const std::vector<DirectedWave>& waves)
{
    rmagine::OnDnModel model;

    model.width = waves.size();
    model.height = 1;
    model.range.min = 0.0;
    model.range.max = 1000.0;
    model.dirs.resize(model.width);
    model.origs.resize(model.width);
    
    for(size_t i=0; i<waves.size(); i++)
    {
        model.origs[i] = waves[i].ray.orig;
        model.dirs[i] = waves[i].ray.dir;
    }

    return model;
}

rmagine::OnDnModel make_model(
    const std::vector<std::vector<DirectedWave> >& waves)
{
    rmagine::OnDnModel model;

    if(waves.size() > 0 && waves[0].size() > 0)
    {
        model.width = waves.size();
        model.height = waves[0].size();
        model.range.min = 0.0;
        model.range.max = 1000.0;
        model.origs.resize(model.width * model.height);
        model.dirs.resize(model.width * model.height);
        
        for(size_t vid = 0; vid < model.getHeight(); vid++)
        {
            for(size_t hid = 0; hid < model.getWidth(); hid++)
            {
                size_t buf_id = model.getBufferId(vid, hid);
                auto wave = waves[hid][vid];
                model.origs[buf_id] = wave.ray.orig;
                model.dirs[buf_id] = wave.ray.dir;
            }
        }
    }
    
    return model;
}

// rm::Memory<rm::Transform> local_transforms(
//     const std::vector<DirectedWave>& waves)
// {

// }


// rmagine::Transform angle_to_transform(
//     rmagine::Vector
// )

rmagine::Transform ray_to_transform(
    rmagine::Vector orig, 
    rmagine::Vector dir)
{
    rmagine::Transform T;
    T.t = orig;
    auto dirn = dir.normalize();

    rmagine::Vector up{0.0, 0.0, 1.0};

    rmagine::Vector xaxis = up.cross(dirn).normalize();
    rmagine::Vector yaxis = dirn.cross(xaxis).normalize();

    rmagine::Matrix3x3 R;
    R(0, 0) = xaxis.x;
    R(0, 1) = yaxis.x;
    R(0, 2) = dirn.x;

    R(1, 0) = xaxis.y;
    R(1, 1) = yaxis.y;
    R(1, 2) = dirn.y;

    R(2, 0) = xaxis.z;
    R(2, 1) = yaxis.z;
    R(2, 2) = dirn.z;

    T.R = R;

    return T;
}

rmagine::Quaternion polar_to_quat(float phi, float theta)
{
    rmagine::Quaternion q = rmagine::EulerAngles{0.0, phi, theta};
    return q;
}

std::vector<DirectedWave> sample_cone_local(
    const DirectedWave& wave_ex, 
    float width,
    int n_samples,
    int sample_dist,
    float p_in_cone)
{
    std::vector<DirectedWave> waves;
    // n_samples--;

    std::random_device                      rand_dev;
    std::mt19937                            gen(rand_dev());
    std::uniform_real_distribution<float>   dist_uni(0.0, 1.0);
    std::normal_distribution<float>         dist_normal(0.0, 1.0);

    float z = M_SQRT2 * erfinvf(p_in_cone);

    float radius = width / 2.0;

    for(int i=0; i<n_samples; i++)
    {
        float random_angle = dist_uni(gen) * 2.0f * M_PI - M_PI;

        float random_radius;
        if(sample_dist == 0) {
            random_radius = dist_uni(gen) * radius;
        } else if(sample_dist == 1) {
            random_radius = sqrt(dist_uni(gen)) * radius;
        } else if(sample_dist == 2) {
            random_radius = (dist_normal(gen) / z) * radius;
        } else if(sample_dist == 3) {
            random_radius = sqrt(abs(dist_normal(gen)) / z) * radius;
        }

        float alpha = random_radius * cos(random_angle);
        float beta = random_radius * sin(random_angle);

        rmagine::EulerAngles e = {0.f, alpha, beta};

        DirectedWave wave = wave_ex;
        wave.ray.orig = {0.0, 0.0, 0.0};
        wave.ray.dir = e * rmagine::Vector{1.0, 0.0, 0.0};
        waves.push_back(wave);
    }

    return waves;
}

rm::Memory<rm::Vector> sample_cone(
    const rm::Vector& ray_dir_mean, 
    float width,
    int n_samples,
    int sample_dist,
    float p_in_cone)
{
    rm::Memory<rm::Vector> ray_dirs(n_samples);

    std::random_device                      rand_dev;
    std::mt19937                            gen(rand_dev());
    std::uniform_real_distribution<float>   dist_uni(0.0, 1.0);
    std::normal_distribution<float>         dist_normal(0.0, 1.0);

    float z = M_SQRT2 * erfinvf(p_in_cone);

    float radius = width / 2.0;

    for(size_t i=0; i<n_samples; i++)
    {
        float random_angle = dist_uni(gen) * 2.0f * M_PI - M_PI;

        float random_radius;
        if(sample_dist == 0) {
            random_radius = dist_uni(gen) * radius;
        } else if(sample_dist == 1) {
            random_radius = sqrt(dist_uni(gen)) * radius;
        } else if(sample_dist == 2) {
            random_radius = (dist_normal(gen) / z) * radius;
        } else if(sample_dist == 3) {
            random_radius = sqrt(abs(dist_normal(gen)) / z) * radius;
        }

        float alpha = random_radius * cos(random_angle);
        float beta = random_radius * sin(random_angle);

        rmagine::EulerAngles e = {0.f, alpha, beta};
        ray_dirs[i] = e * ray_dir_mean;
    }

    return ray_dirs;
}

std::vector<DirectedWave> sample_cone(
    const DirectedWave& wave_mean, 
    float width,
    int n_samples,
    int sample_dist,
    float p_in_cone)
{
    std::vector<DirectedWave> waves;
    waves.push_back(wave_mean);
    n_samples--;

    std::random_device                      rand_dev;
    std::mt19937                            gen(rand_dev());
    std::uniform_real_distribution<float>   dist_uni(0.0, 1.0);
    std::normal_distribution<float>         dist_normal(0.0, 1.0);

    float z = M_SQRT2 * erfinvf(p_in_cone);

    float radius = width / 2.0;

    for(; n_samples>0; n_samples--)
    {
        float random_angle = dist_uni(gen) * 2.0f * M_PI - M_PI;

        float random_radius;
        if(sample_dist == 0) {
            random_radius = dist_uni(gen) * radius;
        } else if(sample_dist == 1) {
            random_radius = sqrt(dist_uni(gen)) * radius;
        } else if(sample_dist == 2) {
            random_radius = (dist_normal(gen) / z) * radius;
        } else if(sample_dist == 3) {
            random_radius = sqrt(abs(dist_normal(gen)) / z) * radius;
        }

        float alpha = random_radius * cos(random_angle);
        float beta = random_radius * sin(random_angle);

        rmagine::EulerAngles e = {0.f, alpha, beta};

        DirectedWave wave = wave_mean;
        wave.ray.dir = e * wave_mean.ray.dir;
        waves.push_back(wave);
    }

    return waves;
}

} // namespace radarays_ros