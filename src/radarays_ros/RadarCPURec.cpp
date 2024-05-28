

#include "radarays_ros/RadarCPURec.hpp"

#include <cv_bridge/cv_bridge.h>
#include <omp.h>

#include <radarays_ros/radar_algorithms.h>
#include <radarays_ros/image_algorithms.h>


#include <random>

#include <rmagine/util/StopWatch.hpp>
#include <rmagine/util/prints.h>



namespace rm = rmagine;

namespace radarays_ros
{

RadarCPURec::RadarCPURec(
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
    material_air.brdf_func = lambertian_brdf;
    material_air.brdf_params.resize(1); // lambertian doesnt need parameters to be set
    material_air.brdf_params[1] = M_PI;
    material_air.n = 1.0;
    material_air.transmittance = 1.0;
    m_materials.push_back(material_air);

    // what is this material?
    Material material1;
    material1.brdf_func = lambertian_brdf;
    material1.brdf_params.resize(1); // lambertian_brdf needs 1 parameter
    material1.brdf_params[0] = 0.5; // diffuse amount
    material1.n = 1000.0;
    material1.transmittance = 0.1;
    m_materials.push_back(material1);


    // what is this material?
    Material material2;
    material2.brdf_func = blinn_phong_brdf;
    material2.brdf_params.resize(3); // blinn phong needs 3 parameters
    material2.brdf_params[0] = 0.2; // diffuse amount
    material2.brdf_params[1] = 0.5; // glossy amount
    material2.brdf_params[2] = 0.3; // specular amount
    material2.n = 100.0;
    material2.transmittance = 0.0;
    m_materials.push_back(material2);

    // TODO: what is this material
    Material material3;
    material3.brdf_func = cook_torrance_brdf;
    material3.brdf_params.resize(4); // cook_torrance_brdf needs 2 parameters
    material3.brdf_params[0] = 0.2; // diffuse amount
    material3.brdf_params[1] = 0.5; // roughness
    material3.n = 100.0;
    material3.transmittance = 0.5;
    m_materials.push_back(material3);

    // Water?
    Material material4;
    material4.brdf_func = cook_torrance_brdf;
    material4.brdf_params.resize(2); // cook_torrance_brdf needs 2 parameters
    material4.brdf_params[0] = 0.2; // diffuse amount
    material4.brdf_params[1] = 0.0; // roughness
    material4.n = 1.333;
    material4.transmittance = 1.0;
    m_materials.push_back(material4);

}

void RadarCPURec::setSampleFunc(WaveGenFunc wave_gen_func)
{
    m_wave_generator = wave_gen_func;
}

bool RadarCPURec::isFreeInBetween(
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

std::optional<Intersection> RadarCPURec::intersect(DirectedWave& wave) const
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
float RadarCPURec::renderSingleWave(
    const DirectedWave& wave,
    const Sender& sender,
    std::vector<float>& range_returns,
    int tree_depth) const
{
    std::cout << std::string("-") * tree_depth << "<" << std::endl;
    // propagate wave

    // TODO: implement
    // 1. find intersection with scene
    // 2. send ray from intersection to sender
    // 3. if tree_depth > 0: scatter for reflection and transmission
    DirectedWave incidence = wave;
    const auto hit = intersect(incidence);

    if(hit)
    {   
        const Intersection intersection = *hit;

        DirectedWave reflection_fresnel, transmission_fresnel;
        std::tie(reflection_fresnel, transmission_fresnel) = intersection.fresnel(incidence);

        

        std::cout << "Fresnel Energy Split:" << std::endl;
        std::cout << "- Reflection: " << reflection_fresnel.energy << std::endl;
        std::cout << "- Transmission: " << transmission_fresnel.energy << std::endl;
        std::cout << "-> Energy conservation: " << reflection_fresnel.energy + transmission_fresnel.energy << std::endl;
        std::cout << "--- Transmission (including absorption): " << transmission_fresnel.energy 
                  << " * " << transmission_fresnel.material->transmittance 
                  << " = " << transmission_fresnel.energy * transmission_fresnel.material->transmittance << std::endl;

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
            float distance_to_sender = dir_to_sender.l2norm();
            dir_to_sender.normalizeInplace();
            float Li = sender.getEnergyDensity(-dir_to_sender); // incoming radiance part

            std::cout << "Li: " << Li << std::endl;
            if(Li < 0.0)
            {
                std::cout << "Li: " << Li << std::endl;
            }
            assert(Li >= 0.0);

            if(are_same_medium(-incidence.ray.dir, intersection.normal, dir_to_sender))
            {   
                // std::cout << "Same medium!" << std::endl;
                float nwi = intersection.normal.dot(-incidence.ray.dir);
                assert(nwi >= 0.0);
                float fr = intersection.brdf(incidence, dir_to_sender);
                assert(fr == fr);
                assert(fr >= 0.0);
                float Lr0 = fr * Li * nwi; // reflection part that comes directly from the sender

                n_samples++;
                Lo += Lr0;
            } else {
                // what if the sender is in the transmitted material?
                std::cout << "Incident wave and sender are in diffeent media" << std::endl;
                
            }
        } else {
            // occluded: can this be part valuable?
            std::cout << "No free way between hit and sender :(" << std::endl;
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
            size_t n_reflections = 1;
            size_t n_transmissions = 1;

            float total_reflection_energy = reflection_fresnel.energy;
            
            float total_transmission_energy = transmission_fresnel.energy 
                * transmission_fresnel.material->transmittance;


           

            for(size_t i=0; i<n_reflections; i++)
            {
                DirectedWave reflection_sample = reflection_fresnel;

                // recursion
                // compute incoming radiance
                const float Li = renderSingleWave(reflection_sample, sender, range_returns, tree_depth - 1);
                float fr = intersection.brdf(incidence, reflection_sample.ray.dir);
                assert(fr == fr);
                float nwi = intersection.normal.dot(-incidence.ray.dir);
                // compute reflected radiance
                float Lri = fr * Li * nwi;
                
                // add reflected radiance to total reflected power
                Lo += Lri;
                n_samples++; // for later normalization? 
            }

            for(size_t i=0; i<n_transmissions; i++)
            {
                // TODO
                DirectedWave transmission_sample = transmission_fresnel;
                const float Li = renderSingleWave(transmission_sample, sender, range_returns, tree_depth - 1);

                Lo += Li;
                n_samples++;
            }
        }

        // std::cout << "Collected " << n_samples << " valid samples for computing the integral" << std::endl;
        if(n_samples > 0)
        {
            Lo = Lo / static_cast<float>(n_samples);
            float distance = incidence.getDistanceAir();
            int distance_bucket = distance / m_cfg.resolution;
            if(distance_bucket < range_returns.size())
            {
                range_returns[distance_bucket] += Lo;
            }
        }

        return Lo;
    } else {
        // miss: do nothing?
    }

    return 0.0;
}


sensor_msgs::ImagePtr RadarCPURec::simulate(
    ros::Time stamp)
{
    rm::StopWatch sw;
    sw();

    sensor_msgs::ImagePtr msg;

    // 
    if(m_polar_image.rows != m_cfg.n_cells)
    {
        std::cout << "[RadarCPURec] Resize canvas to " << m_cfg.n_cells << std::endl;
        m_polar_image.resize(m_cfg.n_cells);
        std::cout << "[RadarCPURec] Resizing canvas - done." << std::endl;
    }

    updateTsm();

    // a buffer to store the returned energy per distance (cell)
    std::vector<float> range_returns(m_cfg.n_cells, 0.0);

    

    Sender sender;
    sender.Tsm = Tsm_last; // TODO: put actual sensor pose here

    Receiver receiver;
    receiver.Tsm = sender.Tsm; // place receive at same point as sender

    // rm::Transform bla = receiver.Tsm;
    std::cout << "Radar at: " << receiver.Tsm.t.x << ", " << receiver.Tsm.t.y << ", " << receiver.Tsm.t.z << std::endl;


    DirectedWave wave;
    wave.energy       =  1.0; //
    wave.polarization =  0.5;
    wave.frequency    = 76.5; // GHz
    wave.time         =  0.0; // in ns
    wave.ray.orig = {0.0, 0.0, 0.0};
    wave.ray.dir = {0.0, 1.0, 0.0};
    wave.material = &m_materials[0]; // air
    wave = receiver.Tsm * wave;

    float Li = renderSingleWave(wave, sender, range_returns, 10);
    std::cout << "Returned energy: " << Li << std::endl;


    // for(auto wave : receiver.genSamplesMap())
    // {
    //     renderSingleWave(wave, sender, range_returns, 3);
    // }

    // DirectedWave wave;
    // wave.energy       =  1.0;
    // wave.polarization =  0.5;
    // wave.frequency    = 76.5; // GHz
    // // wave.velocity     =  0.3; // m / ns - speed in air
    // // wave.material_id  =  0;   // 0: air
    // wave.material = &m_materials[0];
    // wave.time         =  0.0; // in ns
    // wave.ray.orig = {0.0, 0.0, 0.0};
    // wave.ray.dir = {1.0, 0.0, 0.0};
    // std::cout << "Wave speed: " << wave.getVelocity() << " m/ns" << std::endl;

    int n_cells = m_polar_image.rows;
    int n_angles = m_polar_image.cols;

    
    // prepare output
    msg = cv_bridge::CvImage(
                std_msgs::Header(), 
                "mono8",
                m_polar_image).toImageMsg();

    msg->header.stamp = stamp;
    msg->header.frame_id = m_sensor_frame;
    
    double el = sw();
    std::cout << "Image simulation speed: " << el << " s" << std::endl;
    return msg;
}


} // namespace radarays