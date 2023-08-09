#ifndef RADARAYS_ROS_RADAR_ALGORITHMS_CUH
#define RADARAYS_ROS_RADAR_ALGORITHMS_CUH

#include <radarays_ros/radar_types.h>
#include <rmagine/types/MemoryCuda.hpp>
#include <rmagine/types/Bundle.hpp>
#include <rmagine/simulation/SimulationResults.hpp>
#include <radarays_ros/RadarParams.h>

namespace rm = rmagine;

namespace radarays_ros
{

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
    rm::MemView<uint8_t, rm::VRAM_CUDA>& waves_new_mask
);


void move_waves(
    rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& wave_origs,
    rm::MemoryView<rm::Vector, rm::VRAM_CUDA>& wave_dirs,
    rm::MemoryView<DirectedWaveAttributes, rm::VRAM_CUDA>& wave_attributes,
    const rm::MemoryView<float, rm::VRAM_CUDA>& distances,
    const rm::MemoryView<uint8_t, rm::VRAM_CUDA>& mask);

void signal_shader(
    const rm::MemView<RadarMaterial, rm::VRAM_CUDA>& materials,
    const rm::MemView<int, rm::VRAM_CUDA>& object_materials,
    int material_id_air,

    const rm::MemView<rm::Vector, rm::VRAM_CUDA>& dirs,
    const rm::MemView<DirectedWaveAttributes, rm::VRAM_CUDA>& attr,
    const rm::MemView<uint8_t, rm::VRAM_CUDA>& hits,
    const rm::MemView<rm::Vector, rm::VRAM_CUDA>& surface_normals,
    const rm::MemView<unsigned int, rm::VRAM_CUDA>& object_ids,

    rm::MemView<Signal, rm::VRAM_CUDA>& signals
);


void fresnel_split(
    const rm::MemView<RadarMaterial, rm::VRAM_CUDA>& materials,
    const rm::MemView<int, rm::VRAM_CUDA>& object_materials,
    int material_id_air,
    // INCIDENCE
    const rm::MemView<rm::Vector, rm::VRAM_CUDA>& incidence_origs,
    const rm::MemView<rm::Vector, rm::VRAM_CUDA>& incidence_dirs,
    const rm::MemView<DirectedWaveAttributes, rm::VRAM_CUDA>& incidence_attrs,
    const rm::MemView<uint8_t, rm::VRAM_CUDA>& hits,
    const rm::MemView<rm::Vector, rm::VRAM_CUDA>& surface_normals,
    const rm::MemView<unsigned int, rm::VRAM_CUDA>& object_ids,
    // SPLIT
    rm::MemView<rm::Vector, rm::VRAM_CUDA>& reflection_origs,
    rm::MemView<rm::Vector, rm::VRAM_CUDA>& reflection_dirs,
    rm::MemView<DirectedWaveAttributes, rm::VRAM_CUDA>& reflection_attrs,
    rm::MemView<rm::Vector, rm::VRAM_CUDA>& refraction_origs,
    rm::MemView<rm::Vector, rm::VRAM_CUDA>& refraction_dirs,
    rm::MemView<DirectedWaveAttributes, rm::VRAM_CUDA>& refraction_attrs);

void draw_signals(
    rm::MemView<float, rm::VRAM_CUDA>& img,
    rm::MemView<float, rm::VRAM_CUDA>& max_vals,
    rm::MemView<unsigned int, rm::VRAM_CUDA>& signal_counts,
    unsigned int n_angles,
    unsigned int n_cells,
    const rm::MemView<Signal, rm::VRAM_CUDA>& signals,
    const rm::MemView<uint8_t, rm::VRAM_CUDA>& mask,
    unsigned int n_samples,
    const unsigned int denoising_type,
    const rm::MemView<float, rm::VRAM_CUDA> denoising_weights,
    int denoising_mode,
    float resolution);




} // namespace radarays_ros

#endif // RADARAYS_ROS_RADAR_ALGORITHMS_CUH