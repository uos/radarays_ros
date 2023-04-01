#!/usr/bin/env python
import numpy as np
from mpl_toolkits import mplot3d
from matplotlib import pyplot as plt
from matplotlib.widgets import Slider, Button

# n = c / v

def incident_angle(surface_normal, ray_dir):
    return np.arccos((-ray_dir).dot(surface_normal))

def transmission_angle(surface_normal, refract_dir):
    return np.arccos((refract_dir).dot(-surface_normal))

def angle_limit(n1, n2):
    return np.arcsin(n2 / n1)


# def snell_return_dir(surface_normal, ray_dir):
#     return -ray_dir

"""
Returns the direction and the reflectance in percent
"""
def fresnel_reflect_dir(surface_normal, ray_dir, n1, n2):
    reflect_dir = np.zeros_like(ray_dir)
    reflectance = 0.0

    reflect_dir = ray_dir + 2.0 * (-surface_normal).dot(ray_dir) * surface_normal

    return reflect_dir, reflectance

"""
Returns the direction and the transmittance in percent
"""
def fresnel_refract_dir(surface_normal, ray_dir, n1, n2):
    refract_dir = np.zeros_like(ray_dir)
    transmittance = 0.0

    # check input
    if n1 > 0.0:
        n21 = n2 / n1
        if np.abs(n21) <= 1.0:
            angle_limit = np.arcsin(n21)
        else:
            # there is no angle_limit
            angle_limit = 100.0

        angle_inc = incident_angle(surface_normal, ray_dir)

        if angle_inc <= angle_limit:
            if n21 > 0.0:
                n12 = 1.0 / n21
                c = np.cos(angle_inc)
                refract_dir = n12 * ray_dir + (n12 * c - np.sqrt(1 - n12*n12 * ( 1 - c*c ) ) ) * surface_normal

    return refract_dir, transmittance
    

if __name__ == '__main__':
    print('Fresnel')

    angle_ray_deg = 45.0
    angle_ray = angle_ray_deg * np.pi / 180.0
    angle_normal = 0.0 * np.pi / 180.0

    ray_dir = np.array([
        np.sin(angle_ray),
        -np.cos(angle_ray)])
    surface_normal = np.array([np.sin(angle_normal), np.cos(angle_normal)])

    n1 = 1.0
    n2 = 1.3

    # return_dir = fresnel_return_dir(surface_normal, ray_dir)
    reflect_dir,_ = fresnel_reflect_dir(surface_normal, ray_dir, n1, n2)
    refract_dir,_ = fresnel_refract_dir(surface_normal, ray_dir, n1, n2)

    fig, ax = plt.subplots()

    # ax.plot([-1, 1], [1, 1])
    ax.fill_between([-10, 10], [10, 10], color='k', alpha=0.1)

    line_inc, = ax.plot([-ray_dir[0], 0], [-ray_dir[1], 0], label='Incident')
    line_refl, = ax.plot([0, reflect_dir[0]], [0, reflect_dir[1]], label='Reflection')
    line_refr, = ax.plot([0, refract_dir[0]], [0, refract_dir[1]], label='Refraction')

    fig.subplots_adjust(bottom=0.25)

    # slider visables
    ax_inc_angle = fig.add_axes([0.2, 0.13, 0.65, 0.03])
    ax_n1 = fig.add_axes([0.2, 0.08, 0.65, 0.03])
    ax_n2 = fig.add_axes([0.2, 0.03, 0.65, 0.03])

    ax.set_xlim([-1.05, 1.05])
    ax.set_ylim([-1.05, 1.05])
    ax.legend()

    def render():
        ray_dir = np.array([
            np.sin(angle_ray),
            -np.cos(angle_ray)])
        line_inc.set_xdata([-ray_dir[0], 0])
        line_inc.set_ydata([-ray_dir[1], 0])

        # Update Reflecion Ray
        reflect_dir,_ = fresnel_reflect_dir(surface_normal, ray_dir, n1, n2)
        line_refl.set_xdata([0, reflect_dir[0]])
        line_refl.set_ydata([0, reflect_dir[1]])

        # Update Refraction Ray
        refract_dir,_ = fresnel_refract_dir(surface_normal, ray_dir, n1, n2)
        line_refr.set_xdata([0, refract_dir[0]])
        line_refr.set_ydata([0, refract_dir[1]])

        i_angle = incident_angle(surface_normal, ray_dir)
        t_angle = transmission_angle(surface_normal, refract_dir)
        
        print(ray_dir)
        print(refract_dir)

        print("Incidence angle (deg):",  i_angle * 180.0 / np.pi)
        print("Refraction angle (deg):", t_angle * 180.0 / np.pi)

        rs = 0.0
        rp = 0.0

        eps = 0.00001

        if i_angle + t_angle < eps:
            rs = (n1 - n2) / (n1 + n2)
            rp = rs
        elif i_angle + t_angle > np.pi - eps:
            rs = 1.0
            rp = 1.0
        else:
            rs = -np.sin(i_angle - t_angle) / np.sin(i_angle + t_angle)
            rp = np.tan(i_angle - t_angle) / np.tan(i_angle + t_angle)


        Rs = rs*rs
        Rp = rp*rp
        Reff = 0.5 * (Rs + Rp)

        # # ts
        Ts = 1.0 - Rs
        Tp = 1.0 - Rp
        Teff = 1.0 - Reff

        # or
        # ts
        # ts = (2.0 * n1 * np.cos(i_angle)) / (n1 * np.cos(i_angle) + n2 * np.cos(t_angle))
        # tp = (2.0 * n1 * np.cos(i_angle)) / (n2 * np.cos(i_angle) + n1 * np.cos(t_angle))
        # Tfactor = (n2 * np.cos(t_angle)) / (n1 * np.cos(i_angle))

        # Ts = Tfactor * ts * ts
        # Tp = Tfactor * tp * tp
        # Teff = 0.5 * (Ts + Tp)

        print("Reflectance:",     Rs, Rp, Reff)
        print("Transmittance: ", Ts, Tp, Teff)
        print("1.0 =", Reff + Teff)

        line_refl.set_alpha(Reff)
        line_refr.set_alpha(Teff)

        fig.canvas.draw_idle()


    inc_angle_slider = Slider(
        ax=ax_inc_angle,
        label='Incident Angle',
        valmin=0.0,
        valmax=90.0,
        valinit=angle_ray_deg)

    n1_slider = Slider(
        ax=ax_n1,
        label='N1: Upper medium (or v2)',
        valmin=0.0,
        valmax=5.0,
        valinit=1.0)

    n2_slider = Slider(
        ax=ax_n2,
        label='N2: Lower medium (or v1)',
        valmin=0.0,
        valmax=5.0,
        valinit=1.3)

    def update_inc_angle(val):
        global angle_ray
        angle_ray = val * np.pi / 180.0
        render()

    inc_angle_slider.on_changed(update_inc_angle)
    
    def update_n1(val):
        global n1
        n1 = val
        render()

    n1_slider.on_changed(update_n1)

    def update_n2(val):
        global n2
        n2 = val
        render()

    n2_slider.on_changed(update_n2)

    plt.show()
