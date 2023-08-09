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


} // namespace radarays_ros

#endif // RADARAYS_ROS_RADAR_ALGORITHMS_CUH