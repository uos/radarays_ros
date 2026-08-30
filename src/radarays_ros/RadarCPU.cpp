#include "radarays_ros/RadarCPU.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <omp.h>

#include <radarays_ros/radar_algorithms.h>
#include <radarays_ros/image_algorithms.h>
#include <random>

#include <rmagine/util/StopWatch.hpp>

namespace rm = rmagine;

namespace radarays_ros
{

RadarCPU::RadarCPU(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<tf2_ros::TransformListener> tf_listener,
    std::string map_frame,
    std::string sensor_frame,
    rm::EmbreeMapPtr map)
:Base(node, tf_buffer, tf_listener, map_frame, sensor_frame)
,m_map(map)
{

}

sensor_msgs::msg::Image::SharedPtr RadarCPU::simulate(
    rclcpp::Time stamp)
{
    sensor_msgs::msg::Image::SharedPtr msg;

    //
    if(m_polar_image.rows != m_cfg_n_cells)
    {
        std::cout << "[RadarCPU] Resize canvas to " << m_cfg_n_cells << std::endl;

        m_polar_image.resize(m_cfg_n_cells);

        std::cout << "[RadarCPU] Resizing canvas - done." << std::endl;
    }

    m_polar_image.setTo(cv::Scalar(0));

    std::vector<float> denoising_weights;
    int denoising_mode = 0;

    if(m_cfg_signal_denoising > 0)
    {
        if(m_cfg_signal_denoising == 1)
        {
            denoising_mode = m_cfg_signal_denoising_triangular_mode * m_cfg_signal_denoising_triangular_width;
            denoising_weights = make_denoiser_triangular(
                m_cfg_signal_denoising_triangular_width,
                denoising_mode
            );

        } else if(m_cfg_signal_denoising == 2) {
            denoising_mode = m_cfg_signal_denoising_gaussian_mode * m_cfg_signal_denoising_gaussian_width;
            denoising_weights = make_denoiser_gaussian(
                m_cfg_signal_denoising_gaussian_width,
                denoising_mode
            );

        } else if(m_cfg_signal_denoising == 3) {
            denoising_mode = m_cfg_signal_denoising_mb_mode * m_cfg_signal_denoising_mb_width;
            denoising_weights = make_denoiser_maxwell_boltzmann(
                m_cfg_signal_denoising_mb_width,
                denoising_mode
            );
        }

        // scale so that mode has weight 1
        if(denoising_weights.size() > 0)
        {
            double denoising_mode_val = denoising_weights[denoising_mode];

            for(size_t i=0; i<denoising_weights.size(); i++)
            {
                denoising_weights[i] /= denoising_mode_val;
            }
        }

    }

    DirectedWave wave;
    wave.energy       =  1.0;
    wave.polarization =  0.5;
    wave.frequency    = 76.5; // GHz
    wave.velocity     =  0.3; // m / ns - speed in air
    wave.material_id  =  0;   // air
    wave.time         =  0.0; // ns
    wave.ray.orig = {0.0, 0.0, 0.0};
    wave.ray.dir = {1.0, 0.0, 0.0};

    int n_cells = m_polar_image.rows;
    int n_angles = m_polar_image.cols;

    // without motion: update Tsm only once
    if(!m_cfg_include_motion)
    {
        if(!updateTsm())
        {
            std::cout << "Couldn't get Transform between sensor and map. Skipping..." << std::endl;
            return msg;
        }
    }

    if(m_resample)
    {
        m_waves_start = sample_cone_local(
            wave,
            m_params.model.beam_width,
            m_params.model.n_samples,
            m_cfg_beam_sample_dist,
            m_cfg_beam_sample_dist_normal_p_in_cone);
        m_resample = false;
    }

    rm::StopWatch sw_radar_sim;
    sw_radar_sim();

    #pragma omp parallel for if(!m_cfg_include_motion)
    for(size_t angle_id = 0; angle_id < n_angles; angle_id++)
    {

        int tid = 0;
        if(!m_cfg_include_motion)
        {
            // threaded. get sim that belong to current thread
            tid = omp_get_thread_num();
        }

        // m_sims is a plain std::unordered_map shared across all OMP
        // threads, lazily populated on first use per tid. find()/operator[]
        // are not safe to call concurrently (one thread's insert can
        // rehash while another thread's find() is mid-lookup) -- the old
        // code only wrapped the log line in `#pragma omp critical`, leaving
        // the actual map read/insert unsynchronized. Under a real
        // sync_topic message stream this raced and segfaulted after a few
        // messages (reproduced with `ros2 run radar_simulator` +
        // `ros2 bag play` directly, independent of any launch wrapper).
        // Whole find-or-create block now runs under one critical section.
        std::shared_ptr<rm::OnDnSimulatorEmbree> sim;
        #pragma omp critical (radarcpu_sims_map)
        {
            auto sims_it = m_sims.find(tid);
            if(sims_it == m_sims.end())
            {
                m_sims[tid] = std::make_shared<rm::OnDnSimulatorEmbree>(m_map);
                sims_it = m_sims.find(tid);
                auto Tsb = rm::Transform::Identity();
                sims_it->second->setTsb(Tsb);

                std::cout << "Created new simulator for thread " << tid << std::endl;
            }
            sim = sims_it->second;
        }

        if(!sim)
        {
            std::cout << "ERROR!! Sim shared ptr empty" << std::endl;
        }

        std::vector<DirectedWave> waves = m_waves_start;

        rm::OnDnModel model = make_model(waves);
        sim->setModel(model);

        // with motion: update at each angle
        if(m_cfg_include_motion)
        {
            if(!updateTsm())
            {
                continue;
            }
        }

        // make Tam ? angle to map Tam = Tsm * Tas
        // Tas is the transformation of angle to sensor

        rm::Transform Tas;
        Tas.R = rm::EulerAngles{0.0, 0.0, m_radar_model.getTheta(angle_id)};
        Tas.t = m_radar_model.getOrigin(0, angle_id);

        rm::Transform Tsm = Tsm_last;
        rm::Transform Tam = Tsm * Tas;

        rm::Memory<rm::Transform> Tams(1);
        Tams[0] = Tam;

        std::vector<Signal> signals;
        ///////
        /// 1. Signal generation
        for(size_t pass_id = 0; pass_id < m_params.model.n_reflections; pass_id++)
        {
            using ResT = rm::Bundle<
                rm::Hits<rm::RAM>,
                rm::Ranges<rm::RAM>,
                rm::Normals<rm::RAM>,
                rm::ObjectIds<rm::RAM> // connection to material
            >;

            ResT results;

            results.hits.resize(model.size());
            results.ranges.resize(model.size());
            results.normals.resize(model.size());
            results.object_ids.resize(model.size());

            sim->simulate(Tams, results);

            // reflect / refract / absorb / return
            std::vector<DirectedWave> waves_new;

            // Move rays
            for(size_t i=0; i < waves.size(); i++)
            {
                // get
                DirectedWave wave = waves[i];
                const float wave_range = results.ranges[i];
                const rmagine::Vector surface_normal = results.normals[i].normalize();
                const unsigned int obj_id = results.object_ids[i];


                if(obj_id > 10000)
                {
                    continue;
                }

                // do
                const DirectedWave incidence = wave.move(wave_range);

                // inititalize
                DirectedWave reflection = incidence;
                DirectedWave refraction = incidence;

                // if wave was in air, switch to new material
                // else if wave was in material, switch to air (is this right ?)
                if(incidence.material_id == m_material_id_air)
                {
                    refraction.material_id = (obj_id < m_object_materials.size()) ? m_object_materials[obj_id] : 0;
                } else {
                    refraction.material_id = m_material_id_air;
                }

                float v_refraction = 1.0;

                if(incidence.material_id != refraction.material_id)
                {
                    v_refraction = m_params.materials.data[refraction.material_id].velocity;
                } else {
                    v_refraction = incidence.velocity;
                }

                // Build surface patch
                auto res = fresnel(surface_normal, incidence, v_refraction);

                reflection.ray.dir = res.first.ray.dir;
                reflection.energy = res.first.energy;

                if(reflection.energy > m_wave_energy_threshold)
                {
                    waves_new.push_back(reflection);

                    // split total reflection energy into
                    // - scene reflection energy
                    // - path return energy
                    // - air return energy
                    //
                    // path return energy = air return energy
                    //      for pass 0
                    // path return energy = air return energy = scene reflection energy
                    //      for pass 0 and perpendicular hit

                    if(reflection.material_id == m_material_id_air)
                    {
                        // 1. signal travelling back along the path

                        auto material = m_params.materials.data[refraction.material_id];

                        double incidence_angle = get_incidence_angle(surface_normal, incidence);
                        // 1. signal traveling over path
                        double return_energy_path = back_reflection_shader(
                            incidence_angle,
                            reflection.energy,
                            material.ambient, // ambient
                            material.diffuse, // diffuse
                            material.specular // specular
                        );

                        // signal traveling the air path == traversal path
                        if(pass_id == 0 || m_cfg_record_multi_reflection)
                        {
                            float time_back = incidence.time * 2.0;
                            signals.push_back({time_back, return_energy_path});
                        }

                        if(pass_id > 0 && m_cfg_record_multi_path)
                        {
                            // 2. signal traveling the air path
                            Signal signal_air;

                            rm::Vector dir_sensor_to_hit = reflection.ray.orig;
                            const double distance_between_sensor_and_hit = dir_sensor_to_hit.l2norm();
                            dir_sensor_to_hit.normalizeInplace();
                            // time_to_sensor in [ns] assuming transmission through space
                            const double time_to_sensor = distance_between_sensor_and_hit / reflection.velocity;


                            double sensor_view_scalar = wave.ray.dir.dot(dir_sensor_to_hit);

                            double angle_between_reflection_and_sensor_dir
                                = angle_between(-reflection.ray.dir, dir_sensor_to_hit);

                            if(m_cfg_record_multi_path
                                && sensor_view_scalar > m_cfg_multipath_threshold)
                            {
                                double return_energy_air = back_reflection_shader(
                                    angle_between_reflection_and_sensor_dir,
                                    reflection.energy,
                                    material.ambient, // ambient
                                    material.diffuse, // diffuse
                                    material.specular // specular
                                );

                                signal_air.time = incidence.time + time_to_sensor;
                                signal_air.strength = return_energy_air;

                                signals.push_back(signal_air);
                            }
                        }
                    }
                }

                refraction.ray.dir = res.second.ray.dir;
                refraction.energy = res.second.energy;

                if(refraction.energy > m_wave_energy_threshold)
                {
                    waves_new.push_back(refraction);
                }
            }

            // skip certain distance for safety
            float skip_dist = 0.001;
            for(size_t i=0; i<waves_new.size(); i++)
            {
                waves_new[i].moveInplace(skip_dist);
            }

            waves = waves_new;

            // update sensor model
            if(pass_id < m_params.model.n_reflections - 1)
            {
                model = make_model(waves);
                sim->setModel(model);
            } else {
                // last run. dont need to update
            }
        }

        //////////////////
        /// 2. Signals -> Canvas
        /// 2.1. Signal Noise -> Slice
        /// 2.2. Ambient noise -> Slice
        /// 2.3. Slice -> Canvas
        cv::Mat_<float> slice(m_polar_image.rows, 1, 0.0);

        float max_val = 0.0;
        // Draw signals to slice
        for(size_t i=0; i<signals.size(); i++)
        {
            auto signal = signals[i];
            // wave speed in air (light speed) * t / 2
            float half_time = signal.time / 2.0;
            float signal_dist = 0.3 * half_time;

            int cell = static_cast<int>(signal_dist / m_cfg_resolution);
            if(cell < slice.rows)
            {
                if(m_cfg_signal_denoising > 0)
                {
                    // signal denoising
                    for(int vid = 0; vid < denoising_weights.size(); vid++)
                    {
                        int glob_id = vid + cell - denoising_mode;
                        if(glob_id > 0 && glob_id < slice.rows)
                        {
                            slice.at<float>(glob_id, 0) += signal.strength * denoising_weights[vid];

                            if(slice.at<float>(glob_id, 0) > max_val)
                            {
                                max_val = slice.at<float>(glob_id, 0);
                            }
                        }
                    }
                } else {
                    // no signal denoising
                    slice.at<float>(cell, 0) = std::max(
                        slice.at<float>(cell, 0),
                        (float)signal.strength
                    );

                    if(slice.at<float>(cell, 0) > max_val)
                    {
                        max_val = slice.at<float>(cell, 0);
                    }
                }
            }
        }

        // normalize
        slice *= m_cfg_energy_max;

        int col = (m_cfg_scroll_image + angle_id) % m_polar_image.cols;

        if(m_cfg_ambient_noise)
        {
            std::random_device                      rand_dev;
            std::mt19937                            gen(rand_dev());
            std::uniform_real_distribution<float>   dist_uni(0.0, 1.0);


            // apply noise
            // low freq perlin
            double scale = m_cfg_ambient_noise_perlin_scale_low;
            // high freq perlin
            double scale2 = m_cfg_ambient_noise_perlin_scale_high;

            double random_begin = dist_uni(gen) * 1000.0;

            for(size_t i=0; i<slice.rows; i++)
            {
                float signal = slice.at<float>(i);

                double p = 0.0;

                if(m_cfg_ambient_noise == 1) // UNIFORM
                {
                    p = dist_uni(gen);
                } else if(m_cfg_ambient_noise == 2) // PERLIN
                {
                    double p_perlin1 = perlin_noise(
                        random_begin + static_cast<double>(i) * scale,
                        static_cast<double>(col) * scale);

                    double p_perlin2 = perlin_noise(
                        random_begin + static_cast<double>(i) * scale2,
                        static_cast<double>(col) * scale2);

                    p = m_cfg_ambient_noise_perlin_p_low * p_perlin1 + (1.0 - m_cfg_ambient_noise_perlin_p_low) * p_perlin2;
                }

                // verwurschteltn
                float signal_min = 0;
                float signal_max = max_val;
                float signal_amp = signal_max - signal_min;

                float signal_ = 1.0 - ((signal - signal_min) / signal_amp);

                float noise_at_0 = signal_amp * m_cfg_ambient_noise_at_signal_0;
                float noise_at_1 = signal_amp * m_cfg_ambient_noise_at_signal_1;

                float signal__ = std::pow(signal_, 4.0);

                float noise_amp = (signal__ * noise_at_0 + (1.0 - signal__) * noise_at_1);

                float noise_energy_max = signal_max * m_cfg_ambient_noise_energy_max;
                float noise_energy_min = signal_max * m_cfg_ambient_noise_energy_min;
                float energy_loss = m_cfg_ambient_noise_energy_loss;

                float y_noise = noise_amp * p;

                float x = (static_cast<float>(i) + 0.5) * m_cfg_resolution;

                y_noise = y_noise + (noise_energy_max - noise_energy_min) * exp(-energy_loss * x) + noise_energy_min;
                y_noise = abs(y_noise);

                slice.at<float>(i) = signal + y_noise;
            }
        }

        slice *= m_cfg_signal_max / max_val;

        slice.convertTo(m_polar_image.col(col), CV_8UC1);

        if(m_cfg_include_motion)
        {
            rclcpp::spin_some(m_node);
        }
    }

    double el_radar_sim = sw_radar_sim();
    std::cout << std::fixed << std::setprecision(8) << el_radar_sim << std::endl;

    msg = cv_bridge::CvImage(
                std_msgs::msg::Header(),
                "mono8",
                m_polar_image).toImageMsg();

    msg->header.stamp = stamp;
    msg->header.frame_id = m_sensor_frame;

    return msg;
}


} // namespace radarays_ros
