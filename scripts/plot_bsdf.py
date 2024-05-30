# Import libraries
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt



def sample_hemisphere_uniform(UV):
    phi = 2.0 * np.pi * UV[0]
    theta = np.arccos(np.ones_like(phi) - UV[1])
    return np.array([phi, theta])

def sample_hemisphere_cos(UV):
    phi = 2.0 * np.pi * UV[0]
    theta = np.arccos(np.sqrt(UV[1]))
    return np.array([phi, theta])

def sample_hemisphere_phong(UV, n=40):
    phi = 2.0 * np.pi * UV[0]
    theta = np.arccos((1.0 - UV[1])**(1.0/(n + 1)))
    return np.array([phi, theta])

def sample_hemisphere_ggx(UV, r=0.5):
    r = 0.5 
    a = r*r
    phi = 2.0 * np.pi * UV[0] 
    theta = np.arccos(np.sqrt( (1 - UV[1]) / ( 1 + (a*a - 1) * UV[1] ) ))
    return np.array([phi, theta])


def polar_to_cartesian(polar):
    phi = polar[0]
    theta = polar[1]
    range = polar[2]

    ox = np.multiply(np.cos(phi), np.sin(theta))
    oy = np.multiply(np.sin(phi), np.sin(theta))
    oz = np.cos(theta)

    x = oz
    y = -ox
    z = -oy

    cartesian = np.array([x,y,z])
    cartesian = np.multiply(cartesian, range)
    return cartesian


n_samples = 1000
UV = np.random.sample(size=(2, n_samples))
polar_angles = sample_hemisphere_ggx(UV)
polar = np.array([polar_angles[0], polar_angles[1], np.ones_like(polar_angles[0])])
cartesian = polar_to_cartesian(polar)






# Creating figure
fig = plt.figure(figsize = (10, 7))
ax = plt.axes(projection ="3d")

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')


# Creating plot
ax.scatter3D(cartesian[0], cartesian[1], cartesian[2], color = "green")
plt.title("simple 3D scatter plot")

# show plot
plt.show()

