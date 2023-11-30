#include "radarays_ros/image_algorithms.cuh"



namespace radarays_ros 
{

__device__
static constexpr int PERLIN_PERMUTATIONS[512] = {
    151, 160, 137, 91, 90, 15, 131, 13, 201,
    95, 96, 53, 194, 233, 7, 225, 140, 36, 103, 30, 69, 142, 8, 99, 37,
    240, 21, 10, 23, 190, 6, 148, 247, 120, 234, 75, 0, 26, 197, 62,
    94, 252, 219, 203, 117, 35, 11, 32, 57, 177, 33, 88, 237, 149, 56,
    87, 174, 20, 125, 136, 171, 168, 68, 175, 74, 165, 71, 134, 139,
    48, 27, 166, 77, 146, 158, 231, 83, 111, 229, 122, 60, 211, 133,
    230, 220, 105, 92, 41, 55, 46, 245, 40, 244, 102, 143, 54, 65, 25,
    63, 161, 1, 216, 80, 73, 209, 76, 132, 187, 208, 89, 18, 169, 200,
    196, 135, 130, 116, 188, 159, 86, 164, 100, 109, 198, 173, 186, 3,
    64, 52, 217, 226, 250, 124, 123, 5, 202, 38, 147, 118, 126, 255,
    82, 85, 212, 207, 206, 59, 227, 47, 16, 58, 17, 182, 189, 28, 42,
    223, 183, 170, 213, 119, 248, 152, 2, 44, 154, 163, 70, 221, 153,
    101, 155, 167, 43, 172, 9, 129, 22, 39, 253, 19, 98, 108, 110, 79,
    113, 224, 232, 178, 185, 112, 104, 218, 246, 97, 228, 251, 34, 242,
    193, 238, 210, 144, 12, 191, 179, 162, 241, 81, 51, 145, 235, 249,
    14, 239, 107, 49, 192, 214, 31, 181, 199, 106, 157, 184, 84, 204,
    176, 115, 121, 50, 45, 127, 4, 150, 254, 138, 236, 205, 93, 222,
    114, 67, 29, 24, 72, 243, 141, 128, 195, 78, 66, 215, 61, 156, 180,
    151, 160, 137, 91, 90, 15, 131, 13, 201,
    95, 96, 53, 194, 233, 7, 225, 140, 36, 103, 30, 69, 142, 8, 99, 37,
    240, 21, 10, 23, 190, 6, 148, 247, 120, 234, 75, 0, 26, 197, 62,
    94, 252, 219, 203, 117, 35, 11, 32, 57, 177, 33, 88, 237, 149, 56,
    87, 174, 20, 125, 136, 171, 168, 68, 175, 74, 165, 71, 134, 139,
    48, 27, 166, 77, 146, 158, 231, 83, 111, 229, 122, 60, 211, 133,
    230, 220, 105, 92, 41, 55, 46, 245, 40, 244, 102, 143, 54, 65, 25,
    63, 161, 1, 216, 80, 73, 209, 76, 132, 187, 208, 89, 18, 169, 200,
    196, 135, 130, 116, 188, 159, 86, 164, 100, 109, 198, 173, 186, 3,
    64, 52, 217, 226, 250, 124, 123, 5, 202, 38, 147, 118, 126, 255,
    82, 85, 212, 207, 206, 59, 227, 47, 16, 58, 17, 182, 189, 28, 42,
    223, 183, 170, 213, 119, 248, 152, 2, 44, 154, 163, 70, 221, 153,
    101, 155, 167, 43, 172, 9, 129, 22, 39, 253, 19, 98, 108, 110, 79,
    113, 224, 232, 178, 185, 112, 104, 218, 246, 97, 228, 251, 34, 242,
    193, 238, 210, 144, 12, 191, 179, 162, 241, 81, 51, 145, 235, 249,
    14, 239, 107, 49, 192, 214, 31, 181, 199, 106, 157, 184, 84, 204,
    176, 115, 121, 50, 45, 127, 4, 150, 254, 138, 236, 205, 93, 222,
    114, 67, 29, 24, 72, 243, 141, 128, 195, 78, 66, 215, 61, 156, 180};

__device__
inline double perlin_fade(double t) {
    return t * t * t * (t * (t * 6 - 15) + 10);
}

__device__
inline double perlin_lerp(double t, double a, double b) {
    return a + t * (b - a);
}

__device__
inline double perlin_grad(int hash, double x, double y, double z) {
    int h = hash & 15;
    double u = h < 8 ? x : y;
    double v = h < 4 ? y : h == 12 || h == 14 ? x : z;
    return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
}

// Compute Perlin noise at coordinates x, y
__device__
double perlin_noise(
    double src_x,
    double src_y,
    double src_z = 0.0) 
{
    int X = (int)floor(src_x) & 255;
    int Y = (int)floor(src_y) & 255;
    int Z = (int)floor(src_z) & 255;

    double x = src_x - floor(src_x);
    double y = src_y - floor(src_y);
    double z = src_z - floor(src_z);

    double u = perlin_fade(x);
    double v = perlin_fade(y);
    double w = perlin_fade(z);

    int A = PERLIN_PERMUTATIONS[X] + Y;
    int AA = PERLIN_PERMUTATIONS[A] + Z;
    int AB = PERLIN_PERMUTATIONS[A + 1] + Z;
    int B = PERLIN_PERMUTATIONS[X + 1] + Y;
    int BA = PERLIN_PERMUTATIONS[B] + Z;
    int BB = PERLIN_PERMUTATIONS[B + 1] + Z;

    double r = perlin_lerp(
        w,
        perlin_lerp(v,
            perlin_lerp(u, perlin_grad(PERLIN_PERMUTATIONS[AA], x, y, z), perlin_grad(PERLIN_PERMUTATIONS[BA], x - 1, y, z)),
            perlin_lerp(u, perlin_grad(PERLIN_PERMUTATIONS[AB], x, y - 1, z),
            perlin_grad(PERLIN_PERMUTATIONS[BB], x - 1, y - 1, z))),
        perlin_lerp(v,
            perlin_lerp(u, perlin_grad(PERLIN_PERMUTATIONS[AA + 1], x, y, z - 1),
            perlin_grad(PERLIN_PERMUTATIONS[BA + 1], x - 1, y, z - 1)),
            perlin_lerp(u, perlin_grad(PERLIN_PERMUTATIONS[AB + 1], x, y - 1, z - 1),
            perlin_grad(PERLIN_PERMUTATIONS[BB + 1], x - 1, y - 1, z - 1))));

    return r;
}

/**
 * Overlayed high and low perlin noise
*/
__device__
double perlin_noise_hilo(
    double off_x, double off_y,
    double x, double y,
    double scale_low, double scale_high,
    double p_low)
{
    // low freq perlin
    // const double scale_low = 0.05;
    // high freq perlin
    // const double scale_high = 0.2;

    double p_perlin_low = perlin_noise(
        off_x + x * scale_low, 
        off_y + y * scale_low);
    
    double p_perlin_high = perlin_noise(
        off_x + x * scale_high, 
        off_y + y * scale_high);

    return p_low * p_perlin_low + (1.0 - p_low) * p_perlin_high;
}

__global__ 
void fill_perlin_noise_kernel(
    cv::cuda::PtrStepSzf img,
    double scale)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x <= img.cols - 1 && y <= img.rows - 1 && y >= 0 && x >= 0)
    {
        float p = perlin_noise(x  * scale, y * scale); // [-1.0,1.0]
        p = (p + 1.0) / 2.0; // [0.0-1.0]
        img(y, x) = p;
    }
}

void fill_perlin_noise(
    cv::cuda::GpuMat& img,
    const double& scale)
{
    dim3 cthreads(16, 16);
    dim3 cblocks(
        static_cast<int>(std::ceil(img.size().width /
            static_cast<double>(cthreads.x))),
        static_cast<int>(std::ceil(img.size().height / 
            static_cast<double>(cthreads.y))));

    fill_perlin_noise_kernel<<<cblocks, cthreads>>>(img, scale);
}

__global__ 
void fill_perlin_noise_hilo_kernel(
    cv::cuda::PtrStepSzf img,
    double off_x, double off_y,
    double scale_low, double scale_high,
    double p_low)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x <= img.cols - 1 && y <= img.rows - 1 && y >= 0 && x >= 0)
    {
        float p = perlin_noise_hilo(
            off_x, off_y,
            x, y,
            scale_low, scale_high,
            p_low); // [-1.0,1.0]

        img(y, x) = p;
    }
}

void fill_perlin_noise_hilo(
    cv::cuda::GpuMat& img,
    double off_x, double off_y,
    double scale_low, double scale_high,
    double p_low)
{
    dim3 cthreads(16, 16);
    dim3 cblocks(
        static_cast<int>(std::ceil(img.size().width /
            static_cast<double>(cthreads.x))),
        static_cast<int>(std::ceil(img.size().height / 
            static_cast<double>(cthreads.y))));

    fill_perlin_noise_hilo_kernel<<<cblocks, cthreads>>>(
        img, off_x, off_y, scale_low, scale_high, p_low);
}

__global__ 
void fill_perlin_noise_hilo_kernel(
    float* img,
    const float* max_vals,
    unsigned int width, unsigned int height,
    double off_x, double off_y,
    double scale_low, double scale_high,
    double p_low,
    AmbientNoiseParams params)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x < width && y < height && y >= 0 && x >= 0)
    {
        float max_val = max_vals[x];
        float signal = img[x * height + y];
        float p = perlin_noise_hilo(
            off_x, off_y,
            x, y,
            scale_low, scale_high,
            p_low); // [-1.0,1.0]

        // verwurschteltn
        float signal_min = 0;
        float signal_max = max_val;
        float signal_amp = signal_max - signal_min;

        float signal_ = 1.0 - ((signal - signal_min) / signal_amp);

        float noise_at_0 = signal_amp * params.noise_at_signal_0;
        float noise_at_1 = signal_amp * params.noise_at_signal_1;

        float signal__ = std::pow(signal_, 4.0);

        float noise_amp = (signal__ * noise_at_0 + (1.0 - signal__) * noise_at_1);

        // noise_amp * p * signal_max;
        
        float noise_energy_max = signal_max * params.noise_energy_max;
        float noise_energy_min = signal_max * params.noise_energy_min;
        float energy_loss = params.noise_energy_loss;

        float y_noise = noise_amp * p;

        float range = (static_cast<float>(y) + 0.5) * params.resolution;

        y_noise = y_noise + (noise_energy_max - noise_energy_min) * exp(-energy_loss * range) + noise_energy_min;
        y_noise = abs(y_noise);

        img[x * height + y] = signal + y_noise;
    }
}


__global__ 
void fill_perlin_noise_hilo_kernel(
    float* img,
    const float* max_vals,
    unsigned int width, unsigned int height,
    const float* off_x, const float* off_y,
    double scale_low, double scale_high,
    double p_low,
    AmbientNoiseParams params)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x < width && y < height && y >= 0 && x >= 0)
    {
        float max_val = max_vals[x];
        float signal = img[x * height + y];
        float p = perlin_noise_hilo(
            off_x[x], off_y[x],
            x, y,
            scale_low, scale_high,
            p_low); // [-1.0,1.0]

        // verwurschteltn
        float signal_min = 0;
        float signal_max = max_val;
        float signal_amp = signal_max - signal_min;

        float signal_ = 1.0 - ((signal - signal_min) / signal_amp);

        float noise_at_0 = signal_amp * params.noise_at_signal_0;
        float noise_at_1 = signal_amp * params.noise_at_signal_1;

        float signal__ = std::pow(signal_, 4.0);

        float noise_amp = (signal__ * noise_at_0 + (1.0 - signal__) * noise_at_1);

        // noise_amp * p * signal_max;
        
        float noise_energy_max = signal_max * params.noise_energy_max;
        float noise_energy_min = signal_max * params.noise_energy_min;
        float energy_loss = params.noise_energy_loss;

        float y_noise = noise_amp * p;

        float range = (static_cast<float>(y) + 0.5) * params.resolution;

        y_noise = y_noise + (noise_energy_max - noise_energy_min) * exp(-energy_loss * range) + noise_energy_min;
        y_noise = abs(y_noise);

        img[x * height + y] = signal + y_noise;
    }
}

void fill_perlin_noise_hilo(
    rm::MemView<float, rm::VRAM_CUDA>& img,
    const rm::MemView<float, rm::VRAM_CUDA>& max_vals,
    unsigned int width, unsigned int height,
    double off_x, double off_y,
    double scale_low, double scale_high,
    double p_low,
    AmbientNoiseParams params)
{
    dim3 cthreads(16, 16);
    dim3 cblocks(
        static_cast<int>(std::ceil(width /
            static_cast<double>(cthreads.x))),
        static_cast<int>(std::ceil(height / 
            static_cast<double>(cthreads.y))));

    fill_perlin_noise_hilo_kernel<<<cblocks, cthreads>>>(
        img.raw(), max_vals.raw(), width, height, 
        off_x, off_y, scale_low, scale_high, p_low, params);
    
}

void fill_perlin_noise_hilo(
    rm::MemView<float, rm::UNIFIED_CUDA>& img,
    const rm::MemView<float, rm::UNIFIED_CUDA>& max_vals,
    unsigned int width, unsigned int height,
    double off_x, double off_y,
    double scale_low, double scale_high,
    double p_low,
    AmbientNoiseParams params)
{
    dim3 cthreads(16, 16);
    dim3 cblocks(
        static_cast<int>(std::ceil(width /
            static_cast<double>(cthreads.x))),
        static_cast<int>(std::ceil(height / 
            static_cast<double>(cthreads.y))));

    fill_perlin_noise_hilo_kernel<<<cblocks, cthreads>>>(
        img.raw(), max_vals.raw(), width, height, 
        off_x, off_y, scale_low, scale_high, p_low, params);
    
}


void fill_perlin_noise_hilo(
    rm::MemView<float, rm::UNIFIED_CUDA>& img,
    const rm::MemView<float, rm::UNIFIED_CUDA>& max_vals,
    unsigned int width, unsigned int height,
    const rm::MemView<float, rm::UNIFIED_CUDA>& off_x, 
    const rm::MemView<float, rm::UNIFIED_CUDA>& off_y,
    double scale_low, double scale_high,
    double p_low, 
    AmbientNoiseParams params)
{
    dim3 cthreads(16, 16);
    dim3 cblocks(
        static_cast<int>(std::ceil(width /
            static_cast<double>(cthreads.x))),
        static_cast<int>(std::ceil(height / 
            static_cast<double>(cthreads.y))));

    fill_perlin_noise_hilo_kernel<<<cblocks, cthreads>>>(
        img.raw(), max_vals.raw(), width, height, 
        off_x.raw(), off_y.raw(), scale_low, scale_high, p_low, params);
}


} // namespace radarays_ros