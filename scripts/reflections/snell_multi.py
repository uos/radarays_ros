#!/usr/bin/env python
import numpy as np
from mpl_toolkits import mplot3d
from matplotlib import pyplot as plt
from matplotlib.widgets import Slider, Button


def snell_return_dir(surface_normal, ray_dir):
    return -ray_dir

def snell_reflect_dir(surface_normal, ray_dir):
    return ray_dir + 2.0 * (-surface_normal).dot(ray_dir) * surface_normal

def snell_refract_dir(surface_normal, ray_dir, n1, n2):
    tmp1 = (-ray_dir).dot(surface_normal)
    angle1 = np.arccos(tmp1)
    r = n1 / n2
    c = np.cos(angle1)
    refract_dir = r * ray_dir + (r * c - np.sqrt(1 - r**2 * ( 1 - c**2 ) ) ) * surface_normal
    return refract_dir


def snell_dirs(surface_normal, ray_dir, n1, n2):
    return snell_reflect_dir(surface_normal, ray_dir), snell_refract_dir(surface_normal, ray_dir, n1, n2)

# mesh conisisting of normals and points

mesh = [
    ([0.0, 0.0], [0.0,  1.0]),
    ([0.0,-0.2], [0.0, -1.0])
]

def intersect_plane(plane, ray): 
    # assuming vectors are all normalized
    denom = plane[1].dot(ray[1])
    if denom > 1e-6:
        p0l0 = plane[0] - ray[0]
        t = p0l0.dot(plane[1]) / denom; 
        return t
    return 100000000.0

def next_intersection(mesh, ray):
    intersection = None
    nearest_dist = 10000.0
    
    for plane in mesh:
        t = intersect_plane(ray, plane)
        
        if t < nearest_dist:
            nearest_dist = t
            intersection = ray[0] + t * ray[1]

    return intersection


"""
    Return more rays
"""
def propagate_rays(mesh, rays):
    next_rays = []

    for ray in rays:
        pass
        

if __name__ == '__main__':
    print('Snell')

    angle_ray_deg = 45.0
    angle_ray = angle_ray_deg * np.pi / 180.0
    angle_normal = 0.0 * np.pi / 180.0

    ray_dir = np.array([
        np.sin(angle_ray),
        -np.cos(angle_ray)])
    surface_normal = np.array([np.sin(angle_normal), np.cos(angle_normal)])

    n1 = 1.3
    n2 = 1.0

    return_dir = snell_return_dir(surface_normal, ray_dir)
    reflect_dir = snell_reflect_dir(surface_normal, ray_dir)
    refract_dir = snell_refract_dir(surface_normal, ray_dir, n1, n2)

    fig, ax = plt.subplots()

    # ax.plot([-1, 1], [1, 1])
    ax.fill_between([-10, 10], [-0.2, -0.2], color='k', alpha=0.1)

    line_inc, = ax.plot([-ray_dir[0], 0], [-ray_dir[1], 0], label='Incident')
    line_refl, = ax.plot([0, reflect_dir[0]], [0, reflect_dir[1]], label='Reflection')
    line_refr, = ax.plot([0, refract_dir[0]], [0, refract_dir[1]], label='Refraction')

    fig.subplots_adjust(bottom=0.25)

    # slider visables
    ax_inc_angle = fig.add_axes([0.2, 0.05, 0.65, 0.03])

    ax.set_xlim([-1.05, 1.05])
    ax.set_ylim([-1.05, 1.05])
    ax.legend()

    inc_angle_slider = Slider(
        ax=ax_inc_angle,
        label='Incident Angle',
        valmin=0.0,
        valmax=90.0,
        valinit=angle_ray_deg,
    )

    def update(val):
        # print("Change")
        # line.set_ydata()

        # Update Incident Ray
        angle_ray = val * np.pi / 180.0
        ray_dir = np.array([np.sin(angle_ray), -np.cos(angle_ray)])
        ray_pos = np.array([0.0, 0.0])
        
        line_inc.set_xdata([-ray_dir[0], 0])
        line_inc.set_ydata([-ray_dir[1], 0])

        # Update Reflecion Ray
        reflect_dir = snell_reflect_dir(surface_normal, ray_dir)
        line_refl.set_xdata([0, reflect_dir[0]])
        line_refl.set_ydata([0, reflect_dir[1]])

        # Update Refraction Ray
        refract_dir = snell_refract_dir(surface_normal, ray_dir, n1, n2)
        line_refr.set_xdata([0, refract_dir[0]])
        line_refr.set_ydata([0, refract_dir[1]])
        fig.canvas.draw_idle()
    
    inc_angle_slider.on_changed(update)

    # plt.plot([-ray_dir[0], 0], [-ray_dir[1], 0], label='Incident')
    # plt.plot([0, reflect_dir[0]], [0, reflect_dir[1]], label='Reflection')
    # plt.plot([0, refract_dir[0]], [0, refract_dir[1]], label='Refraction')

    # plt.legend()

    plt.show()
