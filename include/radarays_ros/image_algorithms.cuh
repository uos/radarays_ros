#ifndef RADARAYS_ROS_IMAGE_ALGORITHMS_CUH
#define RADARAYS_ROS_IMAGE_ALGORITHMS_CUH

#include <opencv2/core/cuda.hpp>
// #include <opencv2/cudev/ptr2d/gpumat.hpp>


#include <math.h>
#include <iostream>

#include <rmagine/types/MemoryCuda.hpp>

#include <radarays_ros/radar_types.h>

namespace rm = rmagine;

namespace radarays_ros
{

void fill_perlin_noise(
    cv::cuda::GpuMat& img,
    const double& scale
);


void fill_perlin_noise_hilo(
    rm::MemView<float, rm::VRAM_CUDA>& img,
    const rm::MemView<float, rm::VRAM_CUDA>& max_vals,
    unsigned int width, unsigned int height,
    double off_x, double off_y,
    double scale_low, double scale_high,
    double p_low,
    AmbientNoiseParams params = {}
);

void fill_perlin_noise_hilo(
    rm::MemView<float, rm::UNIFIED_CUDA>& img,
    const rm::MemView<float, rm::UNIFIED_CUDA>& max_vals,
    unsigned int width, unsigned int height,
    double off_x, double off_y,
    double scale_low, double scale_high,
    double p_low,
    AmbientNoiseParams params = {}
);

// one offset per column (per scan angle)
void fill_perlin_noise_hilo(
    rm::MemView<float, rm::UNIFIED_CUDA>& img,
    const rm::MemView<float, rm::UNIFIED_CUDA>& max_vals,
    unsigned int width, unsigned int height,
    const rm::MemView<float, rm::UNIFIED_CUDA>& off_x, 
    const rm::MemView<float, rm::UNIFIED_CUDA>& off_y,
    double scale_low, double scale_high,
    double p_low,
    AmbientNoiseParams params = {}
);

} // namespace radarays_ros


#endif // RADARAYS_ROS_IMAGE_ALGORITHMS_CUH