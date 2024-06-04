

#include "radarays_ros/RadarCPUFlex.hpp"

#include <cv_bridge/cv_bridge.h>
#include <omp.h>

#include <radarays_ros/radar_algorithms.h>
#include <radarays_ros/image_algorithms.h>
#include <radarays_ros/sampling.h>

#include <random>

#include <rmagine/util/StopWatch.hpp>
#include <rmagine/util/prints.h>

#include <std_msgs/Float32MultiArray.h>

#include <omp.h>



namespace rm = rmagine;




namespace radarays_ros
{

/**
 * Example of how to generate a BRDF function from string (factory)
*/
BRDFFunc make_brdf(std::string type)
{
    if(type == "radarays")
    {
        return radarays_brdf;
    } else if(type == "lambertian") {
        return lambertian_brdf;
    } else if(type == "blinn_phong") {
        return blinn_phong_brdf;
    } else if(type == "cook_torrance") {
        return cook_torrance_brdf;
    } else {
        std::stringstream ss;
        ss << "BRDF type '" << type << "' unknown!";
        throw std::runtime_error(ss.str());
    }
}

RadarCPUFlex::RadarCPUFlex(
    std::shared_ptr<ros::NodeHandle> nh_p,
    std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<tf2_ros::TransformListener> tf_listener,
    std::string map_frame,
    std::string sensor_frame,
    rm::EmbreeMapPtr map)
:Base(nh_p, tf_buffer, tf_listener, map_frame, sensor_frame)
,m_map(map)
{
    // load all materials

    // lets load some dummy materials.
    // later they are defined somewhere in the parameters / model

    // material 0: air
    Material material_air;
    material_air.name = "AIR";
    material_air.brdf_func = lambertian_brdf;
    material_air.brdf_params.resize(1); // lambertian doesnt need parameters to be set
    material_air.brdf_params[1] = M_PI;
    material_air.n = 1.0;
    material_air.transmittance = 1.0;
    m_materials.push_back(material_air);


    // what is this material?
    Material material1;
    material1.name = "radarays";
    material1.brdf_func = make_brdf("radarays");
    material1.brdf_params.resize(1); // lambertian_brdf needs 1 parameter
    material1.brdf_params[0] = 0.6;  // A: diffuse factor
    material1.brdf_params[1] = 0.3;  // B: glossy factor
    material1.brdf_params[2] = 53.0; // C: specular exponent, specular factor = 1 - diffuse - glossy
    material1.n = 1000000.0;
    material1.transmittance = 0.0;
    // m_materials.push_back(material1);

    // what is this material?
    Material material2;
    material2.name = "blinn_phong";
    material2.brdf_func = make_brdf("blinn_phong");
    material2.brdf_params.resize(3); // blinn phong needs 3 parameters
    material2.brdf_params[0] = 0.5; // diffuse amount
    material2.brdf_params[1] = 0.2; // glossy amount
    material2.brdf_params[2] = 1.0; // specular amount
    material2.n = 1000.0;
    material2.transmittance = 0.1;
    // m_materials.push_back(material2);

    Material material3;
    material3.name = "cook_torrance";
    material3.brdf_func = make_brdf("cook_torrance");
    material3.brdf_params.resize(4); // cook_torrance_brdf needs 2 parameters
    material3.brdf_params[0] = 0.2; // diffuse amount
    material3.brdf_params[1] = 0.5; // roughness
    material3.n = 100.0;
    material3.transmittance = 0.1;
    m_materials.push_back(material3);


    // Material material3;
    // material3.name = "cook_torrance";
    // material3.brdf_func = make_brdf("cook_torrance");
    // material3.brdf_params.resize(4); // cook_torrance_brdf needs 2 parameters
    // material3.brdf_params[0] = 0.02; // diffuse amount
    // material3.brdf_params[1] = 0.8; // roughness
    // material3.n = 100.0;
    // material3.transmittance = 0.1;
    // m_materials.push_back(material3);

    // m_data_pub = m_nh_p->advertise<std_msgs::Float32MultiArray>("data", 10);
}

void RadarCPUFlex::setSampleFunc(InitSamplingFunc init_sampling_func)
{
    m_init_sampling_func = init_sampling_func;
}

bool RadarCPUFlex::isFreeInBetween(
    const rm::Vector& p1,
    const rm::Vector& p2,
    float t1_offset) const
{

    rm::Vector dir = p2 - p1;
    float max_distance = dir.l2norm();
    dir.normalizeInplace();

    struct RTCRayHit rayhit;
    rayhit.ray.org_x = p1.x;
    rayhit.ray.org_y = p1.y;
    rayhit.ray.org_z = p1.z;
    rayhit.ray.dir_x = dir.x;
    rayhit.ray.dir_y = dir.y;
    rayhit.ray.dir_z = dir.z;
    rayhit.ray.tnear = t1_offset;
    rayhit.ray.tfar = max_distance;
    rayhit.ray.mask = -1;
    rayhit.ray.flags = 0;
    rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
    rtcIntersect1(m_map->scene->handle(), &rayhit);

    return rayhit.hit.geomID == RTC_INVALID_GEOMETRY_ID; // miss -> empty
}

std::optional<Intersection> RadarCPUFlex::intersect(DirectedWave& wave) const
{
    struct RTCRayHit rayhit;
    rayhit.ray.org_x = wave.ray.orig.x;
    rayhit.ray.org_y = wave.ray.orig.y;
    rayhit.ray.org_z = wave.ray.orig.z;
    rayhit.ray.dir_x = wave.ray.dir.x;
    rayhit.ray.dir_y = wave.ray.dir.y;
    rayhit.ray.dir_z = wave.ray.dir.z;
    rayhit.ray.tnear = 0.001; // try to prevent hits if origin is located directly on a face
    rayhit.ray.tfar = std::numeric_limits<float>::infinity();
    rayhit.ray.mask = -1;
    rayhit.ray.flags = 0;
    rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
    // std::cout << "rtcIntersect1" << std::endl;
    rtcIntersect1(m_map->scene->handle(), &rayhit);
    // std::cout << "done." << std::endl;
    

    if(rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID)
    {
        // hit!
        // std::cout << "Move wave" << std::endl;
        wave = wave.move(rayhit.ray.tfar);
        Intersection intersection;

        // compute intersection elements

        // 1. fetch material
        {
            // std::cout << "1. Fetch material" << std::endl;
            unsigned int obj_id;
            if(rayhit.hit.instID[0] != RTC_INVALID_GEOMETRY_ID)
            {
                obj_id = rayhit.hit.instID[0];
            } else {
                obj_id = rayhit.hit.geomID;
            }

            unsigned int mat_id = m_object_materials[obj_id];

            // copy material to result
            intersection.material = &m_materials[mat_id];
        }
        
        // 2. fetch surface normal
        {
            // std::cout << "2. Fetch surface normal" << std::endl;
            intersection.normal = rm::Vector{
                rayhit.hit.Ng_x,
                rayhit.hit.Ng_y,
                rayhit.hit.Ng_z
            };
            // orient normal
            if(intersection.normal.dot(-wave.ray.dir) < 0.0)
            {
                intersection.normal = -intersection.normal;
            }

            intersection.normal.normalizeInplace();
        }

        // return intersection
        return intersection;
    } else {
        // miss
        return std::nullopt;
    }
}


// after intersection:
// split N number of intersection rays into a fraction that goes into the material 
// and a fraction that is transmitted into the material
// do it  
// part of rays that are reflected: E_R / (E_R + E_T)
// part of rays that are transmitted: (E_T * (1 - absorption) ) / (E_R * E_T)
// -> in this case the absorption is doing a clever thing:
//    imagine you have light and black material:
//       - set refractive index equal to air -> nothing is reflected, everything goes into the material
//       - set absorption to 1.0 -> everything of the energy is absorpt (transformed into heat)
std::string operator*(std::string str, int times)
{
    std::stringstream ss;
    for(int i=0; i<times; i++)
    {
        ss << str;
    }
    return ss.str();
}

// from receiver incoming signals along certain direction
// the returning wave is filled with
void RadarCPUFlex::renderSingleWave(
    const DirectedWave& wave,
    const Sender& sender,
    std::vector<float>& range_returns,
    std::vector<int>& range_counts,
    int tree_depth) const
{
    // std::cout << std::string("-") * tree_depth << "<" << std::endl;
    // propagate wave

    // todo: parameter for this
    if(wave.energy < 0.0001)
    {
        // std::cout << "Wave has no energy. Stopping" << std::endl;
        return;
    }

    // TODO: implement
    // 1. find intersection with scene
    // 2. send ray from intersection to sender to see how much is going to sender
    // 3. if tree_depth > 0: scatter for reflection and transmission
    DirectedWave incidence = wave;
    const auto hit = intersect(incidence);

    if(incidence.energy < 0.00001)
    {
        // std::cout << "After transmission, wave lost to much energy to be measured anymore: " << wave.energy << " -> " << incidence.energy << ". Stopping recursion." << std::endl;
        return;
    }

    if(hit)
    {
        const Intersection intersection = *hit;
        DirectedWave reflection_fresnel, transmission_fresnel;
        std::tie(reflection_fresnel, transmission_fresnel) = intersection.fresnel(incidence);

        // convervation of energy
        assert(abs(1.0 - reflection_fresnel.energy - transmission_fresnel.energy) < 0.00001);

        // we dont allow emitting materials -> Le 0
        // thus Lo = Le + Lr -> Lo = Lr
        // compute the total reflectance
        
        // collect signal strenghts at certain distance
        float Lo = 0.0;
        size_t n_samples = 0;

        // if trace is not blocked by any object compute energy add signal to range_returns
        if(isFreeInBetween(incidence.ray.orig, sender.Tsm.t))
        { 
            // std::cout << "Free between hit and sender!" << std::endl;
            // intersection is visible by the sender
            rm::Vector dir_to_sender = (sender.Tsm.t - incidence.ray.orig);
            const float distance_to_sender = dir_to_sender.l2norm();
            dir_to_sender.normalizeInplace();
            
            if(are_same_medium(-incidence.ray.dir, intersection.normal, dir_to_sender))
            {   
                // send wave to sender
                DirectedWave wave_to_sender = reflection_fresnel;
                wave_to_sender.ray.dir = dir_to_sender;

                float fr = intersection.brdf(incidence, dir_to_sender);
                assert(fr == fr);
                assert(fr >= 0.0);
                
                wave_to_sender.energy *= fr; // energy gets smaller at reflection
                wave_to_sender.moveInplace(distance_to_sender);

                // 
                float sent_energy_part = sender.getEnergyDensity(-wave_to_sender.ray.dir);
                
                // distance travelled (assuming the wave was all the time in air)
                const float distance_travelled = wave_to_sender.getDistanceAir();
                const float expected_object_distance = distance_travelled / 2.0;
                const float energy_part = sent_energy_part * wave_to_sender.energy;

                int distance_bucket = expected_object_distance / m_cfg.resolution;
                if(distance_bucket < range_returns.size())
                {
                    range_returns[distance_bucket] += energy_part;
                    range_counts[distance_bucket] += 1;
                }

            } else {
                // what if the sender is in the transmitted material?
                // std::cout << "Incident wave and sender are in diffeent media" << std::endl;
                DirectedWave wave_to_sender = transmission_fresnel;
                wave_to_sender.ray.dir = dir_to_sender;

                 // TODO: how to scatter at transmission?
                float fr = 1.0 / M_PI;
                wave_to_sender.energy *= fr; // energy gets smaller at reflection
                wave_to_sender.moveInplace(distance_to_sender);

                // 
                float sent_energy_part = sender.getEnergyDensity(-wave_to_sender.ray.dir);

                // distance travelled (assuming the wave was all the time in air)
                const float distance_travelled = wave_to_sender.getDistanceAir();
                const float expected_object_distance = distance_travelled / 2.0;
                const float energy_part = sent_energy_part * wave_to_sender.energy;

                int distance_bucket = expected_object_distance / m_cfg.resolution;
                if(distance_bucket < range_returns.size())
                {
                    range_returns[distance_bucket] += energy_part;
                    range_counts[distance_bucket] += 1;
                }
            }
        } else {
            // occluded: can this be part valuable?
            // std::cout << "No free way between hit and sender :(" << std::endl;
        }


        float current_distance = incidence.getDistanceAir();

        int current_cell = static_cast<int>(current_distance / m_cfg.resolution);

        // compute the rest of the energy
        if(tree_depth > 0)
        {
            // get the two distribution modes for reflection and transmission including change of
            // - directions
            // - material -> speed
            // - 
            // (limits this approach to unimodal reflection distributions)

            // collect L1 - Ln from next rays

            // TODO: compute these numbers based on refractive indices
            size_t n_rays = 100;

            const float total_reflection_energy = reflection_fresnel.energy;            
            const float total_transmission_energy = transmission_fresnel.energy 
                * transmission_fresnel.material->transmittance;

            const float P_reflect = total_reflection_energy / incidence.energy;
            const float P_transmit = total_transmission_energy / incidence.energy;
            const float P_absorp = 1.0 - P_reflect - P_transmit;

            // monte carlo split
            std::random_device rd; // obtain a random number from hardware
            std::mt19937 gen(rd()); // seed the generator
            std::uniform_real_distribution<float> dist_uni(0.0, 1.0); // define the range
            std::normal_distribution<float> dist_normal(0.0, 1.0);

            // Monte Carlo Integration:
            // 
            // F(X) ~ 1/N * Sum(f(Xi)/p(Xi))
            // with p(Xi) is the PDF of the sampling function and f(Xi) is the reflectance
            // if the sampling function is in an interval [a,b] with integral  P(Xi) = 1 
            // then p(Xi) = 1/(b-a)
            // F(X) ~ (b-a)/N * Sum(f(Xi))
            // or in 2D:
            // F(x,y) = (x1-x0)*(y1-y0)/N * Sum(f(Xi))
            // for a hemisphere the area is
            // 
            // draw random sample according to distribution that is similar 
            // to the brdf. -> see PBR book (monte carlo integration 751-754)
            // F(x,y) = (x1-x0)*(y1-y0)/N * Sum(f(Xi))
            // (x1-x0) and (y1-y0) are the intervals over which the integral is computed
            // in 2D case it an area over which is integrated
            // for a unitsphere the area is 4pi. So for a hemisphere it is 2pi
            // but in theory we can consider the area as constant we apply as postprocessing step

            const float P = dist_uni(gen);
            if(P < P_reflect)
            {
                float nwi = intersection.normal.dot(-incidence.ray.dir);
                DirectedWave Xi = reflection_fresnel;

                // Current impl:
                // - reflect like a perfect mirror, but considering right amount of energy that is reflected
                // Optimization: 
                // - reflect more diffuse

                // // 90 degree opening angle is hit 50% of the time. +-45 degree around reflection dir
                // float z = 0.67; // erfinf(p=0.5) * sqrt(2)
                // float radius = 45.0 * M_PI / 180.0;

                // const float random_angle = dist_uni(gen) * 2.0f * M_PI - M_PI;
                // const float random_radius = sqrt(abs(dist_normal(gen)) / z);
                // float alpha = random_radius * cos(random_angle);
                // float beta = random_radius * sin(random_angle);

                // rm::EulerAngles e = {0.f, alpha, beta};
                // Xi.ray.dir = e * Xi.ray.dir;

                // const float dist_to_surface = Xi.ray.dir.dot(intersection.normal);
                // if(dist_to_surface < 0.0)
                // {
                //     // wrong side: just flip. no better idea
                //     Xi.ray.dir = Xi.ray.dir - intersection.normal * dist_to_surface * 2.0; // should be normed after this
                // }

                const float fr = intersection.brdf(incidence, Xi.ray.dir);
                assert(fr == fr); // nan not allowed
                Xi.energy *= fr * M_PI * nwi;

                renderSingleWave(Xi, sender, range_returns, range_counts, tree_depth - 1);

            } else if(P < P_reflect + P_transmit) {
                // transmit
                DirectedWave Xi = transmission_fresnel;
                renderSingleWave(Xi, sender, range_returns, range_counts, tree_depth - 1);
            } else {
                // absorption
            }
        }
    } else {
        // miss: do nothing?
    }
}

void RadarCPUFlex::renderWave(
    const Receiver& receiver,
    const Sender& sender,
    const DirectedWave& wave_init,
    std::vector<float>& range_returns, 
    std::vector<int>& range_counts) const
{
    // const std::vector<DirectedWave> waves = sample_cone_local(
    //         wave_init,
    //         m_params.model.beam_width,
    //         m_params.model.n_samples,
    //         m_cfg.beam_sample_dist,
    //         m_cfg.beam_sample_dist_normal_p_in_cone);

    const std::vector<DirectedWave> waves = receiver.genSamples();

    for(size_t i=0; i<waves.size(); i++)
    {
        auto wave = receiver.Tsm * waves[i];
        // energy is splitted equally over all rays
        // the total energy per volume is still higher in the center
        // -> sampler
        wave.energy /= static_cast<float>(waves.size());
        renderSingleWave(wave, sender, range_returns, range_counts, m_params.model.n_reflections);
    }
}

sensor_msgs::ImagePtr RadarCPUFlex::simulate(
    ros::Time stamp)
{
    rm::StopWatch sw;
    sw();

    sensor_msgs::ImagePtr msg;

    // 
    if(m_polar_image.rows != m_cfg.n_cells)
    {
        std::cout << "[RadarCPUFlex] Resize canvas to " << m_cfg.n_cells << std::endl;
        m_polar_image.resize(m_cfg.n_cells);
        std::cout << "[RadarCPUFlex] Resizing canvas - done." << std::endl;
    }

    // copy the parameters once so that they cannot be changed during one simulation
    int n_cells = m_polar_image.rows;
    int n_angles = m_polar_image.cols;

    int viz_column = 300;
    std_msgs::Float32MultiArray viz_data;

    // wave template
    DirectedWave wave_init;
    wave_init.energy       =  1.0; //
    wave_init.polarization =  0.5;
    wave_init.frequency    = 76.5; // GHz
    wave_init.time         =  0.0; // in ns
    wave_init.ray.orig = {0.0, 0.0, 0.0};
    wave_init.ray.dir = {1.0, 0.0, 0.0};
    wave_init.material = &m_materials[0]; // air


    int processors = omp_get_num_procs();
    std::cout << "OpenMP" << std::endl;
    std::cout << "- PROCS: " << omp_get_num_procs() << std::endl;

    omp_set_num_threads(processors);

    updateTsm();

    const double desired_rate = 4.0; // 4 hz
    const double desired_duration = 1.0 / desired_rate; // in seconds
    const double disered_duration_per_angle = desired_duration / static_cast<float>(n_angles);

    // Fill these vectors with transformations per angle to 
    // get correct motion distortion
    std::vector<rm::Transform> Tsm_at_angle(n_angles, Tsm_last);
    std::vector<bool>          Tsm_available(n_angles, true);
    
    // #pragma omp parallel for
    #pragma omp parallel for default(shared)
    for(size_t angle_id = 0; angle_id < n_angles; angle_id++)
    {
        double past_time = desired_duration - disered_duration_per_angle * static_cast<float>(angle_id+1);
        ros::Time angle_stamp = stamp - ros::Duration(past_time);

        if(!Tsm_available[angle_id])
        {
            std::cout << "WARNING! No transform available" << std::endl;
        }

        const rm::Transform Tsm = Tsm_at_angle[angle_id];
        // std::cout << angle_id+1 << "/" << n_angles <<  std::endl;
        // a buffer to store the returned energy per distance (cell)
        std::vector<float> range_returns(n_cells, 0.0);
        // for monte carlo integration
        std::vector<int> range_counts(n_cells, 0);

        rm::Transform Tas = rm::Transform::Identity();
        Tas.t = m_radar_model.getOrigin(0, angle_id);
        const double theta = m_radar_model.getTheta(angle_id);
        Tas.R = rm::EulerAngles{0.0, 0.0, m_radar_model.getTheta(angle_id)};

        Sender sender;
        sender.Tsm = Tsm * Tas;
        sender.energy_density_func = [](const rm::Vector dir) -> float {
            const rm::Vector front = {1.0, 0.0, 0.0};
            float scalar = front.dot(dir);
            return std::clamp(scalar, 0.f, 1.f);
        };

        Receiver receiver;
        receiver.Tsm = sender.Tsm; // place receive at same point as sender
        receiver.sample_func = [this, wave_init](){
            return sample_cone_local(
                wave_init,
                m_params.model.beam_width,
                m_params.model.n_samples,
                m_cfg.beam_sample_dist,
                m_cfg.beam_sample_dist_normal_p_in_cone);
        };
        
        renderWave(receiver, sender, wave_init, range_returns, range_counts);

        const double sent_log = 10.0 * log(wave_init.energy);
        const double pixel_val_at_decibel_zero = m_cfg.signal_max;
        
        // the more waves we have the more we trust our system
        // 1 wave -> 100 process noise added per pixel
        // 100 waves -> 1 process noise added per pixel
        float process_noise = 1000.0 / static_cast<float>(
            m_params.model.n_samples * m_cfg.signal_denoising_triangular_width);
        range_returns = blur_kalman(range_returns, range_counts, process_noise);
        // range_returns = blur_blur(range_returns, range_counts, 5);
        

        std::vector<float> decibels = energy_to_decibel(wave_init.energy, range_returns, range_counts);

        int col = (m_cfg.scroll_image + angle_id) % m_polar_image.cols;
        
        cv::Mat slice(decibels.size(), 1, CV_32FC1, decibels.data());


        if(m_cfg.ambient_noise)
        {
            std::random_device                      rand_dev;
            std::mt19937                            gen(rand_dev());
            std::uniform_real_distribution<float>   dist_uni(0.0, 1.0);

            
            // apply noise
            // low freq perlin
            double scale = 0.05;
            // high freq perlin
            double scale2 = 0.2;

            double random_begin = dist_uni(gen) * 1000.0;
            
            for(size_t i=0; i<slice.rows; i++)
            {
                float signal = slice.at<float>(i);

                double p;

                if(m_cfg.ambient_noise == 1) // UNIFORM
                {
                    p = dist_uni(gen);
                } else if(m_cfg.ambient_noise == 2) // PERLIN
                {
                    double p_perlin1 = perlin_noise(
                        random_begin + static_cast<double>(i) * scale, 
                        static_cast<double>(col) * scale);
                    
                    double p_perlin2 = perlin_noise(
                        random_begin + static_cast<double>(i) * scale2, 
                        static_cast<double>(col) * scale2);

                    p = 0.9 * p_perlin1 + 0.1 * p_perlin2;
                }
                
                // p = p * 
                // p = (p + 1.0) / 2.0; // [0.0-1.0]

                // verwurschteltn
                float signal_min = 0;
                float signal_max = m_cfg.signal_max;
                float signal_amp = signal_max - signal_min;

                float signal_ = 1.0 - ((signal - signal_min) / signal_amp);

                float noise_at_0 = signal_amp * m_cfg.ambient_noise_at_signal_0;
                float noise_at_1 = signal_amp * m_cfg.ambient_noise_at_signal_1;

                float signal__ = std::pow(signal_, 4.0);

                float noise_amp = (signal__ * noise_at_0 + (1.0 - signal__) * noise_at_1);

                // noise_amp * p * signal_max;
                
                float noise_energy_max = signal_max * m_cfg.ambient_noise_energy_max;
                float noise_energy_min = signal_max * m_cfg.ambient_noise_energy_min;
                float energy_loss = m_cfg.ambient_noise_energy_loss;

                float y_noise = noise_amp * p;

                float x = (static_cast<float>(i) + 0.5) * m_cfg.resolution;

                y_noise = y_noise + (noise_energy_max - noise_energy_min) * exp(-energy_loss * x) + noise_energy_min;
                y_noise = abs(y_noise);

                slice.at<float>(i) = signal + y_noise;
            }
        }


        slice.convertTo(m_polar_image.col(col), CV_8UC1);
    }

    // m_data_pub.publish(viz_data);

    // prepare output
    msg = cv_bridge::CvImage(
                std_msgs::Header(), 
                "mono8",
                m_polar_image).toImageMsg();

    msg->header.stamp = stamp;
    msg->header.frame_id = m_sensor_frame;
    
    double el = sw();
    std::cout << "[RadarCPUFlex] Image simulation speed: " << el << " s" << std::endl;
    return msg;
}

std::vector<float> RadarCPUFlex::energy_to_decibel(
    float sent_energy,
    const std::vector<float>& range_returns,
    const std::vector<int>& range_counts) const
{
    std::vector<float> decibels(range_returns.size(), 0.0);

    const double sent_log = 10.0 * log(sent_energy);
    const double pixel_val_at_decibel_zero = m_cfg.signal_max;

    for(size_t i=0; i<range_returns.size(); i++)
    {
        // std::cout << range_returns[i]  << "(" << range_counts[i] << ") " ;
        const double ret_energy = range_returns[i]; // / static_cast<float>(range_counts[i]);
        if(ret_energy > 0.0001)
        {
            const double ret_log = 10.0 * log(ret_energy);
            double decibel = sent_log - ret_log;
            decibels[i] = pixel_val_at_decibel_zero - decibel;
        }
    }

    return decibels;
}



// TODO: Add something like this for motion distortion:
// override this if desired

// rm::Transform Tsm_angle_last = Tsm_last;
// size_t angle_id_last = 0;
// rm::Transform Tsm_new = *getTsm(stamp);

// // Tsms:
// // - Tsm_last: at beginning of the message
// // - Tsm_new: at end of the message
// // - Tsm_angle_last: last valid angle Tsm

// for(size_t angle_id = 0; angle_id < n_angles; angle_id++)
// {
//     double past_time = desired_duration - disered_duration_per_angle * static_cast<float>(angle_id + 1);
//     ros::Time angle_stamp = stamp - ros::Duration(past_time);

//     try {
//         geometry_msgs::TransformStamped Tsm_ros;
//         Tsm_ros = m_tf_buffer->lookupTransform(
//             m_map_frame,
//             m_sensor_frame,
//             angle_stamp
//         );
        
//         rm::Transform Tsm; 
//         Tsm.t.x = Tsm_ros.transform.translation.x;
//         Tsm.t.y = Tsm_ros.transform.translation.y;
//         Tsm.t.z = Tsm_ros.transform.translation.z;
//         Tsm.R.x = Tsm_ros.transform.rotation.x;
//         Tsm.R.y = Tsm_ros.transform.rotation.y;
//         Tsm.R.z = Tsm_ros.transform.rotation.z;
//         Tsm.R.w = Tsm_ros.transform.rotation.w;
//         Tsm.stamp = (Tsm_ros.header.stamp - angle_stamp).toSec(); // result stamp - requested stamp

//         Tsm_at_angle[angle_id] = Tsm;
//         Tsm_available[angle_id] = true;
//         Tsm_angle_last = Tsm;
//         angle_id_last = angle_id;
//     } catch(const tf2::TransformException& ex) {
//         // TODO: what to do here?
//         // throw std::runtime_error("DONT KNOW WHAT TO DO");
//         // std::cout << "Couldnt find transform!" << std::endl;
//         // try to interpolate
        
//         size_t left_angles = n_angles - (angle_id_last + 1);
//         rm::Transform Tsm;
//         if(left_angles > 0)
//         {
            
//             size_t angle_loc = angle_id - angle_id_last;
//             float t = static_cast<float>(angle_loc) / static_cast<float>(left_angles);
//             rm::Transform T_new_angle = ~Tsm_angle_last * Tsm_new;

//             rm::Transform Tsm = Tsm_angle_last * T_new_angle.pow(t);

//             std::cout << "Try to interpolate" << std::endl;
//             std::cout << "between " << Tsm_angle_last << " and " << Tsm_new << std::endl;
//             std::cout << "- delta: " << T_new_angle << std::endl;
//             std::cout << "- t: " << t << std::endl;
//             std::cout << "- delta^t: " << T_new_angle.pow(t) << std::endl;
//             std::cout << "- result: " << Tsm << std::endl;
            

//         } else {
//             Tsm = Tsm_new;
//         }

//         Tsm_at_angle[angle_id] = Tsm;
//         Tsm_available[angle_id] = true;
//     }
// }


} // namespace radarays_ros