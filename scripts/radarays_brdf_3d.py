import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# draw a vector
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
from matplotlib.widgets import Slider, Button

import math
import matplotlib

# tested with:
# Ubuntu20
# - python 3.8.10
# - matplotlib 3.1.2
# - numpy 1.24.3
# Ubuntu22
# - python 3.10.12
# - matplotlib 3.5.1
# - numpy 1.26.4

print("Libraries:")
print("- matplotlib: ", matplotlib.__version__)
print("- numpy", np.__version__)


class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)

    def do_3d_projection(self, renderer=None):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, self.axes.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        return np.min(zs)


def incident_angle(ray_dir, surface_normal):
    return np.arccos((-ray_dir).dot(surface_normal))

def transmission_angle(refract_dir, surface_normal):
    return np.arccos((refract_dir).dot(-surface_normal))

def angle_limit(n1, n2):
    return np.arcsin(n2 / n1)

def snell_reflect_dir(in_dir, surface_normal):
    return 2.0 * surface_normal.dot(-in_dir) * surface_normal + in_dir

def snell_refract_dir(in_dir, surface_normal, n1, n2):
    refract_dir = np.zeros_like(in_dir)

    # check input
    if n1 > 0.0:
        n21 = n2 / n1
        if np.abs(n21) <= 1.0:
            angle_limit = np.arcsin(n21)
        else:
            # there is no angle_limit
            angle_limit = 100.0

        angle_inc = incident_angle(in_dir, surface_normal)

        if angle_inc <= angle_limit:
            if n21 > 0.0:
                n12 = 1.0 / n21
                c = np.cos(angle_inc)
                refract_dir = n12 * in_dir + (n12 * c - np.sqrt(1 - n12*n12 * ( 1 - c*c ) ) ) * surface_normal

    return refract_dir

def compute_fzero(n1, n2):
    return ((n1 - n2) / (n1 + n2))**2

# for cook_torrance use h_dot_in instread: 
# expanation: https://graphicscompendium.com/raytracing/11-fresnel-beer
def fresnel_reflection_coefficient(n_dot_in, F0):
    return F0 + (1 - F0) * (1 - n_dot_in) ** 5.0

def Dfunc(roughness, n_dot_h):
    # The original paper of Cook and Torrance says:
	# float D = (1/(m*m * pow( cos(alpha), 4.0))) * exp (-pow(tan(alpha)/m, 2.0));
	# with alpha = the angle between H and N

	# The book Real-Time Rendering 4 (eq 9.35) says:
	D = max(0, n_dot_h) / (roughness**2 * n_dot_h**4.0) * math.exp((n_dot_h**2 - 1) / (roughness**2 * n_dot_h**2))
    # The book says dot(n,m) but that is the same as dot(n,h) since
	# only micronormals m that are exactly = h contribute.
	# The term in the exponent is in fact equivalent to the tan term in
	# cookpaper.pdf, since tan^2(theta) == ((1-theta^2)/theta^2)
	# See http://www.codinglabs.net/article_physically_based_rendering_cook_torrance.aspx
	return D

def Dfunc_GGX(roughness, n_dot_h):
	# This is the GGX distribution function (eq 9.41) from Real-Time Rendering 4
	D = (max(0, n_dot_h) * roughness**2) / ((1 + n_dot_h**2 * (roughness**2 - 1)))**2
	return D

def Gfunc(n_dot_h, o_dot_h, n_dot_o, n_dot_i):
	G1 = 2 * (n_dot_h * n_dot_o) / (o_dot_h)
	G2 = 2 * (n_dot_h * n_dot_i) / (o_dot_h)
	G = min( G1, G2 )
	G = min(1, G)
	return G

def brdf_cook_torrance(in_dir, out_dir, normal, n1, n2, reflection_params):
    
    
    
    # color?
    incidence_energy = 1.0
    
    k_L = reflection_params[0]
    k_g = 1.0 - k_L
    roughness = reflection_params[1]

    k_s = compute_fzero(n1, n2)
    half = (-in_dir + out_dir)
    half /= np.linalg.norm(half)

    n_dot_h = max(0.001, normal.dot(half))
    n_dot_o = max(0.001, normal.dot(out_dir))
    n_dot_i = max(0.001, normal.dot(-in_dir))
    o_dot_h = max(0.001, out_dir.dot(half))

    F = fresnel_reflection_coefficient(n_dot_i, k_s)
    D = Dfunc( roughness, n_dot_h ) # 1/pi
    G = Gfunc( n_dot_h, o_dot_h, n_dot_o, n_dot_i)

    r_s = (F*G*D) / (4.0*n_dot_i*n_dot_o)
    result = (k_L + k_g * r_s) / np.pi

    result = np.clip(result, 0.0, 1.0)
    # Where to clamp?
    # how to clamp with incidence_energy higher than 1?

    return result * incidence_energy

def brdf_cook_torrance_snell(in_dir, out_dir, normal, n1, n2, reflection_params):
    incidence_energy = 1.0
    
    k_L = reflection_params[0]
    k_g = 1.0 - k_L
    roughness = reflection_params[1]


    reflection_dir = snell_reflect_dir(in_dir, normal)
    refraction_dir = snell_refract_dir(in_dir, normal, n1, n2)

    F0 = compute_fzero(n1, n2)
    k_s = F0

    n_dot_i = max(0.0001, normal.dot(-in_dir))
    F = fresnel_reflection_coefficient(n_dot_i, k_s)
    
    n_dot_h = 0.0
    n_dot_o = 0.0
    o_dot_h = 0.0

    # is in_dir on same side as out_dir?
    if ((-in_dir).dot(normal) * out_dir.dot(normal)) > 0.0:
        # print("Reflection!")
        out_mode = reflection_dir

        # reflection
        n_dot_h = max(0.0001, np.sqrt(out_dir.dot(out_mode) + 1) / np.sqrt(2))
        n_dot_o = max(0.0001, normal.dot(out_dir))

        # o_dot_h reformulation:
        # o_dot_h = out_dir.dot(half)
        # half depends only of out_dir and in_dir
        # with reflection_dir and surface normal the in_dir can be recovered
        # so with surface_normal and reflection_dir it should be possible to compute o_dot_h
        in_dir_tmp = -snell_reflect_dir(-out_mode, normal)
        half_tmp = (-in_dir_tmp + out_dir)
        half_tmp /= np.linalg.norm(half_tmp)

        o_dot_h = max(0.0001, out_dir.dot(half_tmp))
    else:
        # print("Refraction!")
        # refraction
        out_mode = refraction_dir

        # reflection
        n_dot_h = max(0.0001, np.sqrt(out_dir.dot(out_mode) + 1) / np.sqrt(2))
        n_dot_o = max(0.0001, normal.dot(out_dir))
        # n_dot_i = max(0.0001, normal.dot(out_mode)) # the same as normal.dot(reflection_dir)

        in_dir_tmp = -snell_reflect_dir(-out_mode, normal)
        half_tmp = (-in_dir_tmp + out_dir)
        half_tmp /= np.linalg.norm(half_tmp)

        o_dot_h = max(0.0001, out_dir.dot(half_tmp))


    D = Dfunc( roughness, n_dot_h ) # 1/pi
    G = Gfunc( n_dot_h, o_dot_h, n_dot_o, n_dot_i)

    # print("D:", D)
    # print("G:", G)

    r_s = (F*G*D) / (4.0*n_dot_i*n_dot_o)

    # print("r_s:", r_s)
    result = k_L + k_g * r_s

    return result * incidence_energy / np.pi

def brdf_blinn_phong(in_dir, out_dir, normal, n1, n2, reflection_params):
    incidence_energy = 1.0

    half = (-in_dir + out_dir)
    half /= np.linalg.norm(half)

    n_dot_h = normal.dot(half)

    k_L = reflection_params[0]
    k_g = reflection_params[1]
    s = reflection_params[2]

    k_s = ((8.0 + s) / 8.0) * max(0.0, n_dot_h)**s

    result = k_L + (k_g * k_s)

    return result * incidence_energy / np.pi


def brdf_blinn_phong_snell(in_dir, out_dir, normal, n1, n2, reflection_params):
    incidence_energy = 1.0

    reflection_dir = snell_reflect_dir(in_dir, normal)
    refraction_dir = snell_refract_dir(in_dir, normal, n1, n2)

    F0 = compute_fzero(n1, n2)
    
    # we are interested in the macrosurface fresnel:
    # explanation: https://graphicscompendium.com/raytracing/11-fresnel-beer
    n_dot_in = normal.dot(-in_dir)
    Reff = fresnel_reflection_coefficient(n_dot_in, F0)
    Teff = 1.0 - Reff

    k_L = reflection_params[0]
    k_g = reflection_params[1]
    s = reflection_params[2]

    # after a bit of reading the brdf_blinn_phong and brdf_cook_torrance functions:
    # the half vector is mostly used to determine the difference to the normal 
    # this should be similar to have the difference between the ray and the perfect reflection (according to snells law)
    # if we have the perfect reflection the half vector is equal to the normal
    # so I assume:
    # acos(half.dot(normal)) == acos(out_dir.dot(snell_reflection)) / 2.0
    # <=>
    # half.dot(normal) == cos(acos(out_dir.dot(snell_reflection)) / 2.0)
    # <=>
    # half.dot(normal) == sqrt(out_dir.dot(snell_reflection) + 1) / sqrt(2)
    # 
    # analogously it gets possible to compute the n_dot_h for refraction 
    # using "snell_refraction" vector instead of the "snell_reflection" vector

    result = 0.0

    # is in_dir on same side as out_dir?
    if ((-in_dir).dot(normal) * out_dir.dot(normal)) > 0.0:
        # reflection
        n_dot_h = np.sqrt(out_dir.dot(reflection_dir) + 1) / np.sqrt(2)
        k_s = ((8.0 + s) / 8.0) * max(0.0, n_dot_h)**s

        result = k_L + (k_g * k_s)
        result *= Reff
        # print("Reflection:", Reff)
    else:
        # refraction
        n_dot_h = np.sqrt(out_dir.dot(refraction_dir) + 1) / np.sqrt(2)
        k_s = ((8.0 + s) / 8.0) * max(0.0, n_dot_h)**s

        result = k_L + (k_g * k_s)
        result *= Teff
        # print("Refraction:", Teff)

    return result * incidence_energy / np.pi


def energy_return_function(A, B, S, w):
    res = A
    cos_w = np.cos(w)
    if cos_w < 0.0:
        cos_w = 0.0
        
    res += B * cos_w
    
    C = (1.0 - A - B)
    res += C * cos_w**S

    if res < 0.0:
        res = 0.0
        
    return res

def brdf_radarays(in_dir, out_dir, normal, n1, n2, reflection_params):

    reflection_dir = snell_reflect_dir(in_dir, normal)
    incidence_angle = incident_angle(in_dir, normal)
    refraction_dir = snell_refract_dir(in_dir, normal, n1, n2)
    refraction_angle = transmission_angle(refraction_dir, normal)

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

    A = reflection_params[0]
    B = reflection_params[1]
    S = reflection_params[2]
    

    # is in_dir on same side as out_dir?
    if ((-in_dir).dot(normal) * out_dir.dot(normal)) > 0.0:
        # reflection
        angle_to_mode = np.arccos(reflection_dir.dot(out_dir))
        return Reff * energy_return_function(A, B, S, angle_to_mode)
    else:
        # refraction
        angle_to_mode = np.arccos(refraction_dir.dot(out_dir))
        return Teff * energy_return_function(A, B, S, angle_to_mode)


# MAIN



# change this:
brdf_func = brdf_radarays
# brdf_func = brdf_cook_torrance
# brdf_func = brdf_cook_torrance_snell
# brdf_func = brdf_blinn_phong
# brdf_func = brdf_blinn_phong_snell

# the rest can be adjusted with sliders


n_air = 1.0
n_water = 1.333
n_stone = 100.0
n1 = n_air
n2 = n_stone

# shader dependent reflection parameters
# sliders:
# A -> reflection_params[0]
# B -> reflection_params[1]
# C -> reflection_params[2]
reflection_params = [0.2, 0.3, 100.0]


incidence_angle_phi = (45.0 * np.pi / 180.0) - np.pi
incidence_angle_theta = (0.0 * np.pi / 180.0) - np.pi


surface_normal = np.array([0.0, 0.0, 1.0])



# Create a grid of theta and phi values
theta = np.linspace(0, 2 * np.pi, 50)
phi = np.linspace(0, np.pi, 50)
theta, phi = np.meshgrid(theta, phi)


# Convert spherical coordinates to Cartesian coordinates
rX = np.sin(phi) * np.cos(theta)
rY = np.sin(phi) * np.sin(theta)
rZ = np.cos(phi)
energy_density = np.ones_like(rX)





sxx, syy = np.meshgrid([-1, 1], [-1, 1])
sz = sxx * 0.0

# Create a 3D wireframe plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
fig.set_size_inches(10, 12)

# init plot
# ax.fill_between([-10, 10], [-10, -10], color='k', alpha=0.1)

# plot the plane
ax.plot_surface(sxx, syy, sz, alpha=0.5)

# surf: art3d.Poly3DCollection
surf = ax.plot_surface(rX, rY, rZ, cmap='viridis', alpha=0.6)

# draw point of incidence
ax.scatter([0], [0], [0], color="g", s=100)

# initialize arrows. the actual data is filled in the render function
incidence_arrow = Arrow3D([0, 1], [0, 1], [0, 1], mutation_scale=20,
            lw=1, arrowstyle="-|>", color="k")
ax.add_artist(incidence_arrow)

reflection_arrow = Arrow3D([0, 1], [0, 1], [0, 1], mutation_scale=20,
            lw=1, arrowstyle="-|>", color="k")
ax.add_artist(reflection_arrow)

refraction_arrow = Arrow3D([0, 1], [0, 1], [0, 1], mutation_scale=20,
            lw=1, arrowstyle="-|>", color="k")
ax.add_artist(refraction_arrow)


# Set plot limits
ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

fig.subplots_adjust(left=0.1, bottom=0.25, right=0.99, top=0.99)



# Make a horizontal slider to control the frequency.
axincidencephi = fig.add_axes([0.25, 0.19, 0.65, 0.02])
incidence_phi_slider = Slider(
    ax=axincidencephi,
    label='incident angle phi [°]',
    valmin=0.0,
    valmax=90.0,
    valinit=(incidence_angle_phi + np.pi) * 180.0 / np.pi,
)

axincidencetheta = fig.add_axes([0.25, 0.16, 0.65, 0.02])
incidence_theta_slider = Slider(
    ax=axincidencetheta,
    label='incident angle theta [°]',
    valmin=0.0,
    valmax=90.0,
    valinit= (incidence_angle_theta + np.pi) * 180.0 / np.pi,
)

axn1 = fig.add_axes([0.25, 0.13, 0.65, 0.02])
n1_slider = Slider(
    ax=axn1,
    label='n1',
    valmin=0.0,
    valmax=50.0,
    valinit=n1,
)

axn2 = fig.add_axes([0.25, 0.10, 0.65, 0.02])
n2_slider = Slider(
    ax=axn2,
    label='n2',
    valmin=0.0,
    valmax=50.0,
    valinit=n2,
)

axA = fig.add_axes([0.25, 0.07, 0.65, 0.02])
A_slider = Slider(
    ax=axA,
    label='A',
    valmin=0.0,
    valmax=1.0,
    valinit=reflection_params[0],
)

axB = fig.add_axes([0.25, 0.04, 0.65, 0.02])
B_slider = Slider(
    ax=axB,
    label='B',
    valmin=0.0,
    valmax=1.0,
    valinit=reflection_params[1],
)

axS = fig.add_axes([0.25, 0.01, 0.65, 0.02])
S_slider = Slider(
    ax=axS,
    label='S',
    valmin=0.0,
    valmax=1000.0,
    valinit=reflection_params[2],
)


def render():
    global incidence_angle_phi, incidence_angle_phi
    global incidence_phi_slider, incidence_arrow, fig, ax
    global energy_density, rX, rY, rZ
    global brdf_func
    global n1, n2, surface_normal, reflection_params
    global surf

    incidence_angle_phi = (incidence_phi_slider.val * np.pi / 180.0) - np.pi
    incidence_angle_theta = (incidence_theta_slider.val * np.pi / 180.0) - np.pi
    n1 = n1_slider.val
    n2 = n2_slider.val

    reflection_params[0] = A_slider.val
    reflection_params[1] = B_slider.val
    reflection_params[2] = S_slider.val

    dir_in = np.array([
        np.sin(incidence_angle_phi) * np.cos(incidence_angle_theta), 
        np.sin(incidence_angle_phi) * np.sin(incidence_angle_theta), 
        np.cos(incidence_angle_phi)])
    dir_in /= np.linalg.norm(dir_in)

    dir_reflection = snell_reflect_dir(dir_in, surface_normal)
    dir_refraction = snell_refract_dir(dir_in, surface_normal, n1, n2)

    incidence_arrow._verts3d = [[-dir_in[0], 0], [-dir_in[1], 0], [-dir_in[2], 0]]
    reflection_arrow._verts3d = [[0, dir_reflection[0]], [0, dir_reflection[1]], [0, dir_reflection[2]]]
    refraction_arrow._verts3d = [[0, dir_refraction[0]], [0, dir_refraction[1]], [0, dir_refraction[2]]]

    for index in np.ndindex(energy_density.shape):
        dir_out = np.array([rX[index], rY[index], rZ[index]])
        dir_out /= np.linalg.norm(dir_out) # normalize just to be sure
        energy_density[index] = brdf_func(dir_in, dir_out, surface_normal, n1, n2, reflection_params)


    new_x = np.multiply(rX, energy_density)
    new_y = np.multiply(rY, energy_density)
    new_z = np.multiply(rZ, energy_density)

    surf.remove()
    surf = ax.plot_surface(new_x, new_y, new_z, cmap='viridis', alpha=0.6, facecolors=plt.cm.viridis(energy_density / energy_density.max()))
    
    fig.canvas.draw_idle()

render()

def update(val):
    render()

incidence_phi_slider.on_changed(update)
incidence_theta_slider.on_changed(update)
n1_slider.on_changed(update)
n2_slider.on_changed(update)
A_slider.on_changed(update)
B_slider.on_changed(update)
S_slider.on_changed(update)

# Show the plot
fig.legend()
plt.show()
