#!/usr/bin/env python

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.widgets import Slider, Button

cmap = plt.get_cmap("tab10")

def energy_return_function(A, B, C, w):
    res = A
    cos_w = np.cos(w)
    cos_w[cos_w < 0.0] = 0.0 # here?
    res += B * cos_w
    # cos_w[cos_w < 0.0] = 0.0 # or here?
    S = (1.0 - A - B)
    res += S * cos_w**C

    res[res < 0.0] = 0.0

    return res

def energy_reflect_function(A, B, C, w_in, w_ref):
    w_diff = np.abs(w_ref - w_in)
    return energy_return_function(A, B, C, w_diff)

def incident_angle(ray_dir):
    surface_normal = np.array([0.0, 1.0])
    return np.arccos((-ray_dir).dot(surface_normal))

def transmission_angle(refract_dir):
    surface_normal = np.array([0.0, 1.0])
    return np.arccos((refract_dir).dot(-surface_normal))

def angle_limit(n1, n2):
    return np.arcsin(n2 / n1)

def snell_refract_dir(ray_dir, n1, n2):
    surface_normal = np.array([0.0, 1.0])
    refract_dir = np.zeros_like(ray_dir)

    # check input
    if n1 > 0.0:
        n21 = n2 / n1
        if np.abs(n21) <= 1.0:
            angle_limit = np.arcsin(n21)
        else:
            # there is no angle_limit
            angle_limit = 100.0

        angle_inc = incident_angle(ray_dir)

        if angle_inc <= angle_limit:
            if n21 > 0.0:
                n12 = 1.0 / n21
                c = np.cos(angle_inc)
                refract_dir = n12 * ray_dir + (n12 * c - np.sqrt(1 - n12*n12 * ( 1 - c*c ) ) ) * surface_normal

    return refract_dir



# Parameters adjustable at run time

incidence_angle = 55.0 * np.pi / 180.0 # 45 degrees
v1 = 3.33564
v2 = 0.40
A = 0.4
B = 0.2
C = 25.0
#################

reflection_angles = np.linspace(-np.pi / 2.0, np.pi / 2.0, 1000)
reflection_dirs = np.array([-np.sin(reflection_angles), np.cos(reflection_angles)])
refraction_dirs = np.copy(reflection_dirs)
refraction_dirs[1] *= -1.0 




fig, ax = plt.subplots()
ax.set_aspect("equal")
fig.set_size_inches(10, 12)

# init PLOT
ax.fill_between([-10, 10], [-10, -10], color='k', alpha=0.1)
line_n, = ax.plot([0.0, 0.0], [1.0, 0.0], label="Surface normal", color=cmap(3))

# draw energy return/reflect plot
line_refl_energy, = ax.plot(reflection_dirs[0], reflection_dirs[1], label="$E_R$", color=cmap(1))
line_refl_dir, = ax.plot([0.0, 0.0], [0.0, 0.0], label="Reflection", color=cmap(1))

line_refr_energy, = ax.plot(refraction_dirs[0], refraction_dirs[1], label="$E_T$", color=cmap(2))
line_refr_dir, = ax.plot([0.0, 0.0], [0.0, 0.0], label="Refraction", color=cmap(2))

# draw incidence ray
line_inc, = ax.plot([0.0, 0.0], [0.0, 0.0], label="Incidence", color=cmap(0))

ax.set(xlim=(-1.2, 1.2), ylim=(-0.8, 1.2))
fig.subplots_adjust(left=0.1, bottom=0.25, right=0.99, top=0.99)


# Make a horizontal slider to control the frequency.
axincidence = fig.add_axes([0.25, 0.19, 0.65, 0.02])
incidence_slider = Slider(
    ax=axincidence,
    label='incident angle [°]',
    valmin=0.0,
    valmax=90.0,
    valinit=incidence_angle * 180.0 / np.pi,
)

axv2 = fig.add_axes([0.25, 0.16, 0.65, 0.02])
v2_slider = Slider(
    ax=axv2,
    label='wave speed in material [m/ns]',
    valmin=0.0,
    valmax=3.33564,
    valinit=v2,
)

axA = fig.add_axes([0.25, 0.13, 0.65, 0.02])
A_slider = Slider(
    ax=axA,
    label='A',
    valmin=0.0,
    valmax=1.0,
    valinit=A,
)

axB = fig.add_axes([0.25, 0.1, 0.65, 0.02])
B_slider = Slider(
    ax=axB,
    label='B',
    valmin=0.0,
    valmax=1.0,
    valinit=B,
)

axC = fig.add_axes([0.25, 0.07, 0.65, 0.02])
C_slider = Slider(
    ax=axC,
    label='C',
    valmin=0.0,
    valmax=200.0,
    valinit=C,
)

# The function to be called anytime a slider's value changes
def render():
    global incidence_angle, v1, v2, A, B, C, reflection_angles, reflection_dirs, refraction_dirs
    incidence_angle = incidence_slider.val * np.pi / 180.0

    A = A_slider.val
    B = B_slider.val
    C = C_slider.val
    v2 = v2_slider.val

    n1 = v2
    n2 = v1

    reflection_energies = energy_reflect_function(A, B, C, incidence_angle, -reflection_angles)
    reflection_directed_energies = np.multiply(reflection_dirs, reflection_energies)

    incidence_dir = np.array([np.sin(incidence_angle),-np.cos(incidence_angle)])
    reflection_dir = np.array([incidence_dir[0], -incidence_dir[1]])
    
    refraction_dir = snell_refract_dir(incidence_dir, n1, n2)
    refraction_angle = transmission_angle(refraction_dir)

    refraction_energies = energy_reflect_function(A, B, C, -refraction_angle, reflection_angles)
    refraction_directed_energies = np.multiply(refraction_dirs, refraction_energies)

    print("Incidence angle (deg):",  incidence_angle * 180.0 / np.pi)
    print("Refraction angle (deg):", refraction_angle * 180.0 / np.pi)


    rs = 0.0
    rp = 0.0

    eps = 0.00001


    if incidence_angle + refraction_angle < eps:
        rs = (n1 - n2) / (n1 + n2)
        rp = rs
    elif incidence_angle + refraction_angle > np.pi - eps:
        rs = 1.0
        rp = 1.0
    else:
        rs = -np.sin(incidence_angle - refraction_angle) / np.sin(incidence_angle + refraction_angle)
        rp = np.tan(incidence_angle - refraction_angle) / np.tan(incidence_angle + refraction_angle)


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
    # print("1.0 =", Reff + Teff)

    reflection_directed_energies *= Reff
    refraction_directed_energies *= Teff

    # DRAW

    line_refl_energy.set_xdata(reflection_directed_energies[0])
    line_refl_energy.set_ydata(reflection_directed_energies[1])

    line_refr_energy.set_xdata(refraction_directed_energies[0])
    line_refr_energy.set_ydata(refraction_directed_energies[1])

    line_inc.set_xdata([-incidence_dir[0], 0.0])
    line_inc.set_ydata([-incidence_dir[1], 0.0])

    line_refl_dir.set_xdata([0.0, reflection_dir[0]])
    line_refl_dir.set_ydata([0.0, reflection_dir[1]])

    line_refr_dir.set_xdata([0.0, refraction_dir[0]])
    line_refr_dir.set_ydata([0.0, refraction_dir[1]])

    fig.canvas.draw_idle()


def update(val):
    render()

incidence_slider.on_changed(update)
# v1_slider.on_changed(update)
v2_slider.on_changed(update)
A_slider.on_changed(update)
B_slider.on_changed(update)
C_slider.on_changed(update)

# Create a `matplotlib.widgets.Button` to reset the sliders to initial values.
resetax = fig.add_axes([0.8, 0.025, 0.1, 0.03])
button = Button(resetax, 'Reset', hovercolor='0.975')

def reset(event):
    incidence_slider.reset()
    # v1_slider.reset()
    v2_slider.reset()
    A_slider.reset()
    B_slider.reset()
    C_slider.reset()
button.on_clicked(reset)


render()
fig.legend()
plt.show()
