# RadaRays - ROS package

Rotating FMCW radar simulation based on ray-tracing. This package contains all the tools that has been used to simulate radar data and compare it to real sensor data.

## Requirements

- ROS noetic
- Rmagine for ray tracing on CPU or NVIDIA GPUs (RTX or non-RTX): https://github.com/uos/rmagine
- (optional) mesh_tools fork for visualizations: [https://github.com/aock/mesh_tools](https://github.com/aock/mesh_tools)

Mainly tested on:
- Ubuntu 20.04
- AMD Ryzen 7 3800X
- NVIDIA GeForce RTX 2070 SUPER

And others: see paper (preprint is coming soon).

## MulRan

Demonstration of simulating rotating FMCW radar data in large-scale triangle meshes from MulRan datasets. We reconstructed a mesh of the sequences DCC, and KAIST, and added a localization frame to each bag file using MICP-L as described here: [https://github.com/aock/micp_experiments/tree/main/micp_mulran](https://github.com/aock/micp_experiments/tree/main/micp_mulran). The final bag files are piped to RadaRays. The simulation given the localization is started by


```console
roslaunch radarays_ros mulran_sim.launch gui:=false
```

The simulated data is published on topic `/radar/image`. To enable the visualization you will need [https://github.com/aock/mesh_tools](https://github.com/aock/mesh_tools). Then start the same launch file:

```console
roslaunch radarays_ros mulran_sim.launch gui:=true
```

The results should look as follows:

![BRDF](dat/kaist02_radarays_papercolor.png)

Left is the real radar polar image from the MulRan datasets. Right is the RadaRays simulated polar image.

Videos: https://youtube.com/playlist?list=PL9wBuzh6ev06rcl8ksSnxRtv-7jAkR_wx

## BRDF

Easily customize the models reflection parameters using the following interactive tool:

```console
rosrun radaray_ros radarays_snell_fresnel_brdf.py
```

![BRDF](dat/radarays_snell_fresnel_brdf.png)

## Citation

Please reference the following papers when using RadaRays in your scientific work.

```bib
@article{mock2025radarays,
  author={Mock, Alexander and Magnusson, Martin and Hertzberg, Joachim},
  journal={IEEE Robotics and Automation Letters}, 
  title={RadaRays: Real-Time Simulation of Rotating FMCW Radar for Mobile Robotics via Hardware-Accelerated Ray Tracing}, 
  year={2025},
  volume={10},
  number={3},
  pages={2470-2477},
  doi={10.1109/LRA.2025.3531689}
}
```

The paper is available on [IEEE Xplore](https://ieeexplore.ieee.org/abstract/document/10845807) and as preprint on [arXiv](https://arxiv.org/abs/2310.03505).

## Radarays Plugins

- Gazebo: [`https://github.com/uos/radarays_gazebo_plugins`](https://github.com/uos/radarays_gazebo_plugins)

## Roadmap

- [ ] Put every non-ROS component to C++ only repository ([`https://github.com/uos/radarays`](https://github.com/uos/radarays)) for better reusage for non-ROS projects.


## Development

There are currently the following active development tracks on individual branches:

1. [dev/flex](https://github.com/uos/radarays_ros/tree/dev/flex)
    - flexible reflection models for rapid prototyping (CPU-only)
    - Video for Cook-Torrance reflection model: https://www.youtube.com/watch?v=PI5j87NQzmk
2. [dev/opti](https://github.com/uos/radarays_ros/tree/dev/opti)
    - automatic material property optimization using the standard RadaRays simulation model

Individual components of these development branches are transferred to the main branch after extensive testing.