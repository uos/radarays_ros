#include "radarays_ros/RadarGPU.hpp"

#include <cv_bridge/cv_bridge.h>
#include <omp.h>

#include <radarays_ros/radar_algorithms.h>
#include <radarays_ros/image_algorithms.cuh>
#include <radarays_ros/radar_algorithms.cuh>
#include <random>

#include <rmagine/util/StopWatch.hpp>


namespace rm = rmagine;

namespace radarays_ros
{

RadarGPU::RadarGPU(
    std::shared_ptr<ros::NodeHandle> nh_p,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<tf2_ros::TransformListener> tf_listener,
    std::string map_frame,
    std::string sensor_frame,
    rm::OptixMapPtr map)
:Base(nh_p, tf_buffer, tf_listener, map_frame, sensor_frame)
,m_map(map)
{

}

sensor_msgs::ImagePtr RadarGPU::simulate(
    ros::Time stamp)
{
    sensor_msgs::ImagePtr msg;
    
    // 
    if(m_polar_image.rows != m_cfg.n_cells)
    {
        std::cout << "Resize canvas" << std::endl;
        m_polar_image.resize(m_cfg.n_cells);
        std::cout << "Resizing canvas - done." << std::endl;
    }


    std::vector<float> denoising_weights;
    int denoising_mode = 0;

    if(m_cfg.signal_denoising > 0)
    {
        // std::cout << "Signal Denoising: ";
        if(m_cfg.signal_denoising == 1)
        {
            // std::cout << "Triangular";
            denoising_mode = m_cfg.signal_denoising_triangular_mode * m_cfg.signal_denoising_triangular_width;
            denoising_weights = make_denoiser_triangular(
                m_cfg.signal_denoising_triangular_width,
                denoising_mode
            );
            
        } else if(m_cfg.signal_denoising == 2) {
            // std::cout << "Gaussian";
            denoising_mode = m_cfg.signal_denoising_gaussian_mode * m_cfg.signal_denoising_gaussian_width;
            denoising_weights = make_denoiser_gaussian(
                m_cfg.signal_denoising_gaussian_width,
                denoising_mode
            );

        } else if(m_cfg.signal_denoising == 3) {
            // std::cout << "Maxwell Boltzmann";
            denoising_mode = m_cfg.signal_denoising_mb_mode * m_cfg.signal_denoising_mb_width;
            denoising_weights = make_denoiser_maxwell_boltzmann(
                m_cfg.signal_denoising_mb_width,
                denoising_mode
            );
        }
        // std::cout << std::endl;

        // scale so that mode has weight 1
        // if(false)
        if(denoising_weights.size() > 0)
        {
            double denoising_mode_val = denoising_weights[denoising_mode];

            for(size_t i=0; i<denoising_weights.size(); i++)
            {
                denoising_weights[i] /= denoising_mode_val;
            }
        }
    }

    DirectedWaveAttributes wave_att_ex;
    wave_att_ex.energy       =  1.0;
    wave_att_ex.polarization =  0.5;
    wave_att_ex.frequency    = 76.5; // GHz
    wave_att_ex.velocity     =  0.3; // m / ns - speed in air
    wave_att_ex.material_id  =  0;   // air
    wave_att_ex.time         =  0.0; // ns

    int n_cells = m_polar_image.rows;
    int n_angles = m_polar_image.cols;

    // without motion: update Tsm only once
    if(!updateTsm())
    {
        std::cout << "Couldn't get Transform between sensor and map. Skipping..." << std::endl;
        return msg;
    }

    rm::Memory<int, rm::RAM_CUDA> object_materials2(m_object_materials.size());
    for(size_t i=0; i<m_object_materials.size(); i++)
    {
        object_materials2[i] = m_object_materials[i];
    }
    rm::Memory<int, rm::VRAM_CUDA> object_materials_gpu = object_materials2;

    rm::Memory<RadarMaterial, rm::RAM_CUDA> materials2(m_params.materials.data.size());
    for(size_t i=0; i<m_params.materials.data.size(); i++)
    {
        materials2[i] = m_params.materials.data[i];
    }
    rm::Memory<RadarMaterial, rm::VRAM_CUDA> materials_gpu = materials2;

    
    rm::Memory<float, rm::VRAM_CUDA> denoising_weights_gpu;
    if(denoising_weights.size())
    {
        rm::Memory<float, rm::RAM_CUDA> denoising_weights2(denoising_weights.size());
        for(size_t i=0; i<denoising_weights.size(); i++)
        {
            denoising_weights2[i] = denoising_weights[i];
        }
        denoising_weights_gpu = denoising_weights2;
    }
    
    // prepare radar model

    size_t n_rays = n_angles * m_cfg.n_samples;
    // std::cout << "Waves: " << n_rays << std::endl;
    
    
    rm::OnDnModel waves;
    waves.range = m_radar_model.range;
    waves.dirs.resize(n_rays);
    waves.origs.resize(n_rays);
    waves.width = n_angles;
    waves.height = m_params.model.n_samples;
    rm::Memory<DirectedWaveAttributes> wave_attributes(n_rays);


    // auto samples = sample_cone

    rm::Vector front = {1.0, 0.0, 0.0};
    rm::Memory<rm::Vector> ray_dirs_local = sample_cone(
        front,
        m_params.model.beam_width,
        m_params.model.n_samples,
        m_cfg.beam_sample_dist,
        m_cfg.beam_sample_dist_normal_p_in_cone
    );

    // std::cout << "Filling Model" << std::endl;

    for(size_t angle_id=0; angle_id<n_angles; angle_id++)
    {
        auto orig = m_radar_model.getOrigin(0, angle_id);
        auto dir = m_radar_model.getDirection(0, angle_id);

        rm::Transform Tas;
        Tas.R = rm::EulerAngles{0.0, 0.0, m_radar_model.getTheta(angle_id)};
        Tas.t = m_radar_model.getOrigin(0, angle_id);

        for(size_t sample_id = 0; sample_id < m_params.model.n_samples; sample_id++)
        {
            size_t buf_id = waves.getBufferId(sample_id, angle_id);
            waves.dirs[buf_id] = Tas.R * ray_dirs_local[sample_id];
            waves.origs[buf_id] = {0.0, 0.0, 0.0};
            wave_attributes[buf_id] = wave_att_ex;
        }

    }

    // std::cout << "Done filling model" << std::endl;

    // going to GPU

    // 1. preallocate everything necessary

    using ResT = rm::Bundle<
        rm::Hits<rm::VRAM_CUDA>,
        rm::Ranges<rm::VRAM_CUDA>,
        rm::Normals<rm::VRAM_CUDA>,
        rm::ObjectIds<rm::VRAM_CUDA>
        >;

    // pass 1
    rm::OnDnModel_<rm::VRAM_CUDA> waves_gpu1;
    {
        waves_gpu1.width = waves.width;
        waves_gpu1.height = waves.height;
        waves_gpu1.range = waves.range;
        waves_gpu1.origs = waves.origs;
        waves_gpu1.dirs = waves.dirs;
    }

    rm::Memory<DirectedWaveAttributes, rm::VRAM_CUDA> wave_attributes_gpu1
        = wave_attributes;
    
    ResT results1;
    rm::resize_memory_bundle<rm::VRAM_CUDA>(
        results1, waves_gpu1.width, waves_gpu1.height, 1);
    rm::Memory<Signal, rm::VRAM_CUDA> signals1(waves_gpu1.size());
    rm::Memory<uint8_t, rm::VRAM_CUDA> signal_mask1(waves_gpu1.size());
    

    // pass 2
    rm::OnDnModel_<rm::VRAM_CUDA> waves_gpu2;
    {
        waves_gpu2.width = waves_gpu1.width;
        waves_gpu2.height = waves_gpu1.height * 2;
        waves_gpu2.range = waves_gpu1.range;
        waves_gpu2.origs.resize(waves_gpu1.origs.size() * 2);
        waves_gpu2.dirs.resize(waves_gpu1.dirs.size() * 2);
    }

    rm::Memory<DirectedWaveAttributes, rm::VRAM_CUDA> wave_attributes_gpu2(wave_attributes_gpu1.size() * 2);

    ResT results2;
    rm::resize_memory_bundle<rm::VRAM_CUDA>(
        results2, waves_gpu2.width, waves_gpu2.height, 1);
    rm::Memory<Signal, rm::VRAM_CUDA> signals2(waves_gpu2.size());
    rm::Memory<uint8_t, rm::VRAM_CUDA> signal_mask2(waves_gpu2.size());

    // pass 3
    rm::OnDnModel_<rm::VRAM_CUDA> waves_gpu3;
    {
        waves_gpu3.width = waves_gpu2.width;
        waves_gpu3.height = waves_gpu2.height * 2;
        waves_gpu3.range = waves_gpu2.range;
        waves_gpu3.origs.resize(waves_gpu2.origs.size() * 2);
        waves_gpu3.dirs.resize(waves_gpu2.dirs.size() * 2);
    }

    rm::Memory<DirectedWaveAttributes, rm::VRAM_CUDA> wave_attributes_gpu3(wave_attributes_gpu2.size() * 2);

    ResT results3;
    rm::resize_memory_bundle<rm::VRAM_CUDA>(
        results3, waves_gpu3.width, waves_gpu3.height, 1);
    rm::Memory<Signal, rm::VRAM_CUDA> signals3(waves_gpu3.size());
    rm::Memory<uint8_t, rm::VRAM_CUDA> signal_mask3(waves_gpu3.size());
    
    auto sim = std::make_shared<rm::OnDnSimulatorOptix>(m_map);
    sim->setTsb(rm::Transform::Identity());
    sim->preBuildProgram<ResT>();

    rm::Transform Tsm = Tsm_last;
    rm::Memory<rm::Transform> Tsms(1);
    Tsms[0] = Tsm;

    sim->setModel(waves_gpu1);

    rm::StopWatch sw;
    double el_tot = 0.0;
    double el1;
    double el2;
    double el3;

    // 1. Generate Signals
    sw();
    {
        
        // sw();
        sim->simulate(Tsms, results1);
        // cudaDeviceSynchronize();
        // el = sw(); el_tot += el;
        // std::cout << "- ray cast: " << el*1000.0 << "ms" << std::endl;

        // sw();
        move_waves(
            waves_gpu1.origs,
            waves_gpu1.dirs,
            wave_attributes_gpu1,
            results1.ranges, 
            results1.hits);
        // cudaDeviceSynchronize();
        // el = sw(); el_tot += el;

        // std::cout << "- move: " << el*1000.0 << "ms" << std::endl;

        // sw();
        signal_shader(
            materials_gpu,
            object_materials_gpu,
            m_material_id_air,

            waves_gpu1.dirs,
            wave_attributes_gpu1,
            results1.hits,
            results1.normals,
            results1.object_ids,
            
            signals1
        );
        // cudaDeviceSynchronize();
        // el = sw(); el_tot += el;
        // std::cout << "- signal shader: " << el*1000.0 << "ms" << std::endl;
        
        // sw();
        {
            auto lo = waves_gpu2.origs(0, waves_gpu1.origs.size());
            auto ld = waves_gpu2.dirs(0, waves_gpu1.dirs.size());
            auto la = wave_attributes_gpu2(0, wave_attributes_gpu1.size());
            
            auto ro = waves_gpu2.origs(waves_gpu1.origs.size(), waves_gpu1.origs.size() * 2);
            auto rd = waves_gpu2.dirs(waves_gpu1.dirs.size(), waves_gpu1.dirs.size() * 2);
            auto ra = wave_attributes_gpu2(wave_attributes_gpu1.size(), wave_attributes_gpu1.size() * 2);
            
            // FRESNEL SPLIT
            fresnel_split(
                materials_gpu,
                object_materials_gpu,
                m_material_id_air,
                // INCIDENCE
                waves_gpu1.origs,
                waves_gpu1.dirs,
                wave_attributes_gpu1,
                results1.hits,
                results1.normals,
                results1.object_ids,
                // SPLIT
                lo, ld, la,
                ro, rd, ra
            );
            // cudaDeviceSynchronize();
        }
        // el = sw(); el_tot += el;
        // std::cout << "- fresnel split: " << el*1000.0 << "ms" << std::endl;
        
        // std::cout << "- total: " << el_tot << std::endl;
        // std::cout << "Pass 2 - Propagating " << waves_gpu2.size() << " waves:" << std::endl;

        // sw();
        sim->setModel(waves_gpu2);
        // cudaDeviceSynchronize();
        // el = sw(); el_tot += el;
        // std::cout << "- update model: " << el*1000.0 << "ms" << std::endl;
        
        // sw();
        sim->simulate(Tsms, results2);
        // cudaDeviceSynchronize();
        // el = sw(); el_tot += el;

        // std::cout << "- ray cast: " << el*1000.0 << "ms" << std::endl;

        // sw();
        move_waves(
            waves_gpu2.origs,
            waves_gpu2.dirs,
            wave_attributes_gpu2,
            results2.ranges, 
            results2.hits);
        // cudaDeviceSynchronize();
        // el = sw(); el_tot += el;

        // std::cout << "- move: " << el*1000.0 << "ms" << std::endl;
        

        // sw();
        signal_shader(
            materials_gpu,
            object_materials_gpu,
            m_material_id_air,

            waves_gpu2.dirs,
            wave_attributes_gpu2,
            results2.hits,
            results2.normals,
            results2.object_ids,
            
            signals2
        );
        // cudaDeviceSynchronize();
        // el = sw(); el_tot += el;
        // std::cout << "- signal shader: " << el*1000.0 << "ms" << std::endl;

        // sw();
        {
            auto lo = waves_gpu3.origs(0, waves_gpu2.origs.size());
            auto ld = waves_gpu3.dirs(0, waves_gpu2.dirs.size());
            auto la = wave_attributes_gpu3(0, wave_attributes_gpu2.size());
            
            auto ro = waves_gpu3.origs(waves_gpu2.origs.size(), waves_gpu2.origs.size() * 2);
            auto rd = waves_gpu3.dirs(waves_gpu2.dirs.size(), waves_gpu2.dirs.size() * 2);
            auto ra = wave_attributes_gpu3(wave_attributes_gpu2.size(), wave_attributes_gpu2.size() * 2);
            
            // FRESNEL SPLIT
            fresnel_split(
                materials_gpu,
                object_materials_gpu,
                m_material_id_air,
                // INCIDENCE
                waves_gpu2.origs,
                waves_gpu2.dirs,
                wave_attributes_gpu2,
                results2.hits,
                results2.normals,
                results2.object_ids,
                // SPLIT
                lo, ld, la,
                ro, rd, ra
            );
            // cudaDeviceSynchronize();
        }
        
        // el = sw(); el_tot += el;
        // std::cout << "- fresnel split: " << el*1000.0 << "ms" << std::endl;

        // std::cout << "- total: " << el_tot*1000.0 << "ms" << std::endl;

        // std::cout << "Pass 3 - Propagating " << waves_gpu3.size() << " waves:" << std::endl;
        

        // sw();
        sim->setModel(waves_gpu3);
        // cudaDeviceSynchronize();
        // el = sw(); el_tot += el;
        // std::cout << "- update model: " << el*1000.0 << "ms" << std::endl;

        // sw();
        sim->simulate(Tsms, results3);
        // cudaDeviceSynchronize();
        // el = sw(); el_tot += el;
        // std::cout << "- ray cast: " << el*1000.0 << "ms" << std::endl;

        // sw();
        move_waves(
            waves_gpu3.origs,
            waves_gpu3.dirs,
            wave_attributes_gpu3,
            results3.ranges, 
            results3.hits);
        // cudaDeviceSynchronize();
        // el = sw(); el_tot += el;

        // std::cout << "- move: " << el*1000.0 << "ms" << std::endl;

        // sw();
        signal_shader(
            materials_gpu,
            object_materials_gpu,
            m_material_id_air,

            waves_gpu3.dirs,
            wave_attributes_gpu3,
            results3.hits,
            results3.normals,
            results3.object_ids,
            
            signals3
        );
        cudaDeviceSynchronize();

        

        
    }
    el1 = sw(); el_tot += el1;
    
    // std::cout << "RUNTIME" << std::endl;
    // std::cout << "- Signal Gen: " << el << "s" << std::endl;

    bool use_unified_memory = true;

    if(!use_unified_memory)
    {
        rm::Memory<float, rm::VRAM_CUDA> img(n_cells * n_angles);
        rm::Memory<float, rm::VRAM_CUDA> max_vals(n_angles);
        rm::Memory<unsigned int, rm::VRAM_CUDA> signal_counts(n_angles);

        cudaMemset(img.raw(), 0, n_cells * n_angles * sizeof(float) );
        cudaMemset(max_vals.raw(), 0, n_angles * sizeof(float) );
        cudaMemset(signal_counts.raw(), 0, n_angles * sizeof(unsigned int) );

        cudaDeviceSynchronize();

        // 2. noise(signal+system) signals
        sw();
        {
            // draw signals
            draw_signals(img, max_vals, signal_counts,
                n_angles, n_cells,
                signals1, results1.hits,
                m_cfg.n_samples * 1, 
                m_cfg.signal_denoising,
                denoising_weights_gpu,
                denoising_mode,
                m_cfg.resolution
            );

            draw_signals(img, max_vals, signal_counts,
                n_angles, n_cells,
                signals2, results2.hits,
                m_cfg.n_samples * 2, 
                m_cfg.signal_denoising,
                denoising_weights_gpu,
                denoising_mode,
                m_cfg.resolution
            );

            draw_signals(img, max_vals, signal_counts,
                n_angles, n_cells,
                signals3, results3.hits,
                m_cfg.n_samples * 4, 
                m_cfg.signal_denoising,
                denoising_weights_gpu,
                denoising_mode,
                m_cfg.resolution
            );
            cudaDeviceSynchronize();
        }
        el2 = sw(); el_tot += el2;
        // std::cout << "- Noise signal + system: " << el2 << "s" << std::endl;


        // 3. ambient noise
        sw();
        if(m_cfg.ambient_noise)
        {
            std::random_device                      rand_dev;
            std::mt19937                            gen(rand_dev());
            std::uniform_real_distribution<float>   dist_uni(0.0, 1.0);

            
            // apply noise
            // low freq perlin
            double scale_lo = 0.05;
            // high freq perlin
            double scale_hi = 0.2;
            double p_low = 0.9;

            double random_begin = dist_uni(gen) * 1000.0;

            fill_perlin_noise_hilo(
                img, max_vals,
                n_angles, n_cells,
                random_begin, random_begin,
                scale_lo, scale_hi,
                p_low
            );
            cudaDeviceSynchronize();
        }
        el3 = sw(); el_tot += el3;
        // std::cout << "- Noise ambient: " << el << "s" << std::endl;
        // std::cout << "- Total: " << el_tot << "s" << std::endl;
        
        rm::Mem<float> max_vals_cpu = max_vals;
        rm::Mem<float> img_cpu = img;
        float max_signal = 120.0;

        cv::Mat_<float> polar_img_f(n_cells, n_angles);

        for(size_t x=0; x<n_angles; x++)
        {
            float max_val = max_vals_cpu[x];
            for(size_t y=0; y<n_cells; y++)
            {
                polar_img_f.at<float>(y, x) = img_cpu[x * n_cells + y] * max_signal / max_val;
            }
        }
        polar_img_f.convertTo(m_polar_image, CV_8UC1);
    } else {
        // std::cout << "UNIFIED TEST" << std::endl;
        // USE UNIFIED MEMORY
        rm::Memory<float, rm::UNIFIED_CUDA> img(n_cells * n_angles);
        rm::Memory<float, rm::UNIFIED_CUDA> max_vals(n_angles);
        rm::Memory<unsigned int, rm::UNIFIED_CUDA> signal_counts(n_angles);

        cudaMemset(img.raw(), 0, n_cells * n_angles * sizeof(float) );
        cudaMemset(max_vals.raw(), 0, n_angles * sizeof(float) );
        cudaMemset(signal_counts.raw(), 0, n_angles * sizeof(unsigned int) );

        rm::Memory<Signal> signals_cpu1 = signals1;
        rm::Memory<uint8_t> hits_cpu1 = results1.hits;

        rm::Memory<Signal> signals_cpu2 = signals2;
        rm::Memory<uint8_t> hits_cpu2 = results2.hits;

        rm::Memory<Signal> signals_cpu3 = signals3;
        rm::Memory<uint8_t> hits_cpu3 = results3.hits;

        // std::cout << "bla: " << signals_cpu1.size() << std::endl;

        cudaDeviceSynchronize();
        // std::cout << "- buffer created" << std::endl;

        // 2. noise(signal+system) signals
        sw();
        {
            size_t n_samples = m_cfg.n_samples;
            
            #pragma omp parallel for
            for(size_t angle_id = 0; angle_id < n_angles; angle_id++)
            {
                // std::cout << "- angle " << angle_id << std::endl; 
                unsigned int img_offset = angle_id * n_cells;

                float max_val = 0.0;
                unsigned int signal_count = 0;    

                // Draw signals to slice
                // draw signals 1
                for(size_t sample_id=0; sample_id < n_samples; sample_id++)
                {
                    // std::cout << "angle, sample: " << angle_id << "/" << n_angles <<  ", " << sample_id << "/" << n_samples << std::endl;
                    const unsigned int signal_id = sample_id * n_angles + angle_id;

                    // std::cout << "signal: " << signal_id << "/" << signals_cpu1.size() << "-" << results1.hits.size() << std::endl;
                    
                    
                    if(hits_cpu1[signal_id])
                    {
                        // std::cout << "Fetch signal" << std::endl;
                        auto signal = signals_cpu1[signal_id];
                        // std::cout << "- done" << std::endl;
                        // wave speed in air (light speed) * t / 2
                        float half_time = signal.time / 2.0;
                        float signal_dist = 0.3 * half_time;

                        int cell = static_cast<int>(signal_dist / m_cfg.resolution);
                        if(cell < n_cells)
                        {
                            // std::cout << "Cell hit: " << cell << std::endl;
                            // float signal_old = slice.at<float>(cell, 0);

                            if(m_cfg.signal_denoising > 0)
                            {
                                // signal denoising
                                for(int vid = 0; vid < denoising_weights.size(); vid++)
                                {
                                    int glob_id = vid + cell - denoising_mode;
                                    if(glob_id > 0 && glob_id < n_cells)
                                    {
                                        // TODO: check this
                                        const float old_val = img[img_offset + glob_id];
                                        const float new_val = old_val + signal.strength * denoising_weights[vid];
                                        img[img_offset + glob_id] = new_val;

                                        if(new_val > max_val)
                                        {
                                            max_val = new_val;
                                        }
                                    }
                                }
                            } else {
                                // read 
                                // TODO: check this
                                const float old_val = img[img_offset + cell];
                                const float new_val = std::max(old_val, (float)signal.strength);
                                img[img_offset + cell] = new_val;

                                if(new_val > max_val)
                                {
                                    max_val = new_val;
                                }
                            }

                            signal_count++;
                        }
                    }
                }


                // Draw signals to slice
                // draw signals 1
                for(size_t sample_id=0; sample_id < n_samples * 2; sample_id++)
                {
                    // std::cout << "angle, sample: " << angle_id << "/" << n_angles <<  ", " << sample_id << "/" << n_samples << std::endl;
                    const unsigned int signal_id = sample_id * n_angles + angle_id;

                    // std::cout << "signal: " << signal_id << "/" << signals_cpu1.size() << "-" << results1.hits.size() << std::endl;
                    
                    
                    if(hits_cpu2[signal_id])
                    {
                        // std::cout << "Fetch signal" << std::endl;
                        auto signal = signals_cpu2[signal_id];
                        // std::cout << "- done" << std::endl;
                        // wave speed in air (light speed) * t / 2
                        float half_time = signal.time / 2.0;
                        float signal_dist = 0.3 * half_time;

                        int cell = static_cast<int>(signal_dist / m_cfg.resolution);
                        if(cell < n_cells)
                        {
                            // std::cout << "Cell hit: " << cell << std::endl;
                            // float signal_old = slice.at<float>(cell, 0);

                            if(m_cfg.signal_denoising > 0)
                            {
                                // signal denoising
                                for(int vid = 0; vid < denoising_weights.size(); vid++)
                                {
                                    int glob_id = vid + cell - denoising_mode;
                                    if(glob_id > 0 && glob_id < n_cells)
                                    {
                                        // TODO: check this
                                        const float old_val = img[img_offset + glob_id];
                                        const float new_val = old_val + signal.strength * denoising_weights[vid];
                                        img[img_offset + glob_id] = new_val;

                                        if(new_val > max_val)
                                        {
                                            max_val = new_val;
                                        }
                                    }
                                }
                            } else {
                                // read 
                                // TODO: check this
                                const float old_val = img[img_offset + cell];
                                const float new_val = std::max(old_val, (float)signal.strength);
                                img[img_offset + cell] = new_val;

                                if(new_val > max_val)
                                {
                                    max_val = new_val;
                                }
                            }

                            signal_count++;
                        }
                    }
                }


                // Draw signals to slice
                // draw signals 1
                for(size_t sample_id=0; sample_id < n_samples * 4; sample_id++)
                {
                    // std::cout << "angle, sample: " << angle_id << "/" << n_angles <<  ", " << sample_id << "/" << n_samples << std::endl;
                    const unsigned int signal_id = sample_id * n_angles + angle_id;

                    // std::cout << "signal: " << signal_id << "/" << signals_cpu1.size() << "-" << results1.hits.size() << std::endl;
                    
                    
                    if(hits_cpu3[signal_id])
                    {
                        // std::cout << "Fetch signal" << std::endl;
                        auto signal = signals_cpu3[signal_id];
                        // std::cout << "- done" << std::endl;
                        // wave speed in air (light speed) * t / 2
                        float half_time = signal.time / 2.0;
                        float signal_dist = 0.3 * half_time;

                        int cell = static_cast<int>(signal_dist / m_cfg.resolution);
                        if(cell < n_cells)
                        {
                            // std::cout << "Cell hit: " << cell << std::endl;
                            // float signal_old = slice.at<float>(cell, 0);

                            if(m_cfg.signal_denoising > 0)
                            {
                                // signal denoising
                                for(int vid = 0; vid < denoising_weights.size(); vid++)
                                {
                                    int glob_id = vid + cell - denoising_mode;
                                    if(glob_id > 0 && glob_id < n_cells)
                                    {
                                        // TODO: check this
                                        const float old_val = img[img_offset + glob_id];
                                        const float new_val = old_val + signal.strength * denoising_weights[vid];
                                        img[img_offset + glob_id] = new_val;

                                        if(new_val > max_val)
                                        {
                                            max_val = new_val;
                                        }
                                    }
                                }
                            } else {
                                // read 
                                // TODO: check this
                                const float old_val = img[img_offset + cell];
                                const float new_val = std::max(old_val, (float)signal.strength);
                                img[img_offset + cell] = new_val;

                                if(new_val > max_val)
                                {
                                    max_val = new_val;
                                }
                            }

                            signal_count++;
                        }
                    }
                }

                max_vals[angle_id] = max_val;
                signal_counts[angle_id] = signal_count;
            }
        
            // cudaDeviceSynchronize();

        }
        el2 = sw(); el_tot += el2;
        // std::cout << "- Noise signal + system: " << el << "s" << std::endl;

        // 3. ambient noise
        sw();
        if(m_cfg.ambient_noise)
        {
            std::random_device                      rand_dev;
            std::mt19937                            gen(rand_dev());
            std::uniform_real_distribution<float>   dist_uni(0.0, 1.0);

            
            // apply noise
            // low freq perlin
            double scale_lo = 0.05;
            // high freq perlin
            double scale_hi = 0.2;
            double p_low = 0.9;

            double random_begin = dist_uni(gen) * 1000.0;

            fill_perlin_noise_hilo(
                img, max_vals,
                n_angles, n_cells,
                random_begin, random_begin,
                scale_lo, scale_hi,
                p_low
            );
            cudaDeviceSynchronize();
        }
        el3 = sw(); el_tot += el3;
        // std::cout << "- Noise ambient: " << el3 << "s" << std::endl;
        // std::cout << "- Total: " << el_tot << "s" << std::endl;

        float max_signal = 120.0;
        cv::Mat_<float> polar_img_f(n_cells, n_angles);

        for(size_t x=0; x<n_angles; x++)
        {
            float max_val = max_vals[x];
            for(size_t y=0; y<n_cells; y++)
            {
                polar_img_f.at<float>(y, x) = img[x * n_cells + y] * max_signal / max_val;
            }
        }
        polar_img_f.convertTo(m_polar_image, CV_8UC1);

    }
    
    std::cout << std::fixed << std::setprecision(8) << el1/el_tot << ", " << el2/el_tot << ", " << el3/el_tot << ", " << el_tot << std::endl;



    msg = cv_bridge::CvImage(
                std_msgs::Header(), 
                "mono8",
                m_polar_image).toImageMsg();

    msg->header.stamp = stamp;
    msg->header.frame_id = m_sensor_frame;
    
    return msg;
}


} // namespace radarays