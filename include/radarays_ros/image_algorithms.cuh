#ifndef RADARAYS_ROS_IMAGE_ALGORITHMS_CUH
#define RADARAYS_ROS_IMAGE_ALGORITHMS_CUH

#include <opencv2/core/cuda.hpp>
// #include <opencv2/cudev/ptr2d/gpumat.hpp>


#include <math.h>
#include <iostream>


namespace radarays_ros
{

void fill_perlin_noise(
    cv::cuda::GpuMat& img,
    const double& scale
);



} // namespace radarays_ros


#endif // RADARAYS_ROS_IMAGE_ALGORITHMS_CUH