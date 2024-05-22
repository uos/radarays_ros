#include "radarays_ros/RadarCPURec.hpp"

#include <cv_bridge/cv_bridge.h>
#include <omp.h>

#include <radarays_ros/radar_algorithms.h>
#include <radarays_ros/image_algorithms.h>


#include <random>

#include <rmagine/util/StopWatch.hpp>




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
    material_air.brdf_params.resize(0); // lambertian doesnt need parameters to be set
    material_air.n = 1.0;
    m_materials.push_back(material_air);


    // what is this material?
    Material material1;
    material1.brdf_func = blinn_phong_brdf;
    material1.brdf_params.resize(3); // blinn phong needs 3 parameters
    material1.brdf_params[0] = 0.2; // diffuse amount
    material1.brdf_params[1] = 0.5; // glossy amount
    material1.brdf_params[2] = 0.3; // specular amount
    material1.n = 100.0;
    m_materials.push_back(material1);

    // TODO: what is this material
    Material material2;
    material2.brdf_func = cook_torrance_brdf;
    material2.brdf_params.resize(2); // cook_torrance_brdf needs 2 parameters
    material2.brdf_params[0] = 0.2; // diffuse amount
    material2.brdf_params[1] = 0.5; // roughness
    material2.n = 100.0;
    m_materials.push_back(material1);
}

void RadarCPURec::setWaveGenFunc(WaveGenFunc wave_gen_func)
{
    m_wave_generator = wave_gen_func;
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
    rayhit.ray.tnear = 0;
    rayhit.ray.tfar = std::numeric_limits<float>::infinity();
    rayhit.ray.mask = -1;
    rayhit.ray.flags = 0;
    rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
    rtcIntersect1(m_map->scene->handle(), &rayhit);

    if(rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID)
    {
        // hit!
        wave = wave.move(rayhit.ray.tfar);
        Intersection intersection;

        // compute intersection elements

        // 1. fetch material
        {
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
            intersection.normal = rm::Vector{
                rayhit.hit.Ng_x,
                rayhit.hit.Ng_y,
                rayhit.hit.Ng_z
            };
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

    // a buffer to store the returned energy per distance (cell)
    std::vector<float> returned_energy(m_cfg.n_cells, 0.0);

    DirectedWave wave;
    wave.energy       =  1.0;
    wave.polarization =  0.5;
    wave.frequency    = 76.5; // GHz
    wave.velocity     =  0.3; // m / ns - speed in air
    wave.material_id  =  0;   // 0: air
    wave.time         =  0.0; // in ns
    wave.ray.orig = {0.0, 0.0, 0.0};
    wave.ray.dir = {1.0, 0.0, 0.0};

    int n_cells = m_polar_image.rows;
    int n_angles = m_polar_image.cols;

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