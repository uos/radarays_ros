import numpy as np
from matplotlib import pyplot as plt
from scipy.special import erfinv

if __name__ == '__main__':
    print("Driving")


    # parameters
    width = 10.0
    p_in_cone = 0.9
    n_samples = 1000

    radius = width / 2.0

    circle_ranges = np.ones(100) * radius
    circle_phis = np.linspace(-np.pi, np.pi, 100)

    circle_X = np.multiply(circle_ranges, np.cos(circle_phis))
    circle_Y = np.multiply(circle_ranges, np.sin(circle_phis))

    
    samples = np.random.uniform(0.0, 1.0, size=n_samples)
    r_approx = samples * radius
    r = np.sqrt(samples) * radius
    phi = np.random.uniform(-np.pi, np.pi, size=n_samples)


    fig, axes = plt.subplots(2,2)
    
    fig.subplots_adjust(left=0.125, bottom=0.2, right=0.95, top=0.9, wspace=0.4, hspace=0.5)

    xlim = (-radius - radius/3.0, radius + radius/3.0)
    ylim = xlim

    axes[0,0].set_xlim(xlim)
    axes[0,0].set_ylim(ylim)
    axes[0,0].set_xlabel("$\\varphi_{az}$ [°]")
    axes[0,0].set_ylabel("$\\theta_{inc}$ [°]")

    axes[0,1].set_xlim(xlim)
    axes[0,1].set_ylim(ylim)
    axes[0,1].set_xlabel("$\\varphi_{az}$ [°]")
    axes[0,1].set_ylabel("$\\theta_{inc}$ [°]")

    axes[1,0].set_xlim(xlim)
    axes[1,0].set_ylim(ylim)
    axes[1,0].set_xlabel("$\\varphi_{az}$ [°]")
    axes[1,0].set_ylabel("$\\theta_{inc}$ [°]")

    axes[1,1].set_xlim(xlim)
    axes[1,1].set_ylim(ylim)
    axes[1,1].set_xlabel("$\\varphi_{az}$ [°]")
    axes[1,1].set_ylabel("$\\theta_{inc}$ [°]")
    

    X_uni_approx = np.multiply(r_approx, np.cos(phi))
    Y_uni_approx = np.multiply(r_approx, np.sin(phi))
    X_uni = np.multiply(r, np.cos(phi))
    Y_uni = np.multiply(r, np.sin(phi))


    axes[0,0].set_title("$D_1$")
    axes[0,0].scatter(X_uni_approx, Y_uni_approx, s=1)
    axes[0,0].plot(circle_X, circle_Y, c='r')


    axes[0,1].set_title("$D_2$")
    axes[0,1].scatter(X_uni, Y_uni, s=1)
    axes[0,1].plot(circle_X, circle_Y, c='r')


    samples = np.random.normal(0.0, 1.0, size = n_samples)
    phi     = np.random.uniform(-np.pi, np.pi, size = n_samples)



    z = np.sqrt(2) * erfinv(p_in_cone)
    r1      = (samples / z ) * radius

    # check how many samples are in circle
    n_inside = np.count_nonzero(np.abs(r1) < radius)
    print("N1: n_inside / n_samples: %d / %d = %f" % (n_inside, n_samples, n_inside / n_samples))

    X_N1 = np.multiply(r1, np.cos(phi))
    Y_N1 = np.multiply(r1, np.sin(phi))

    axes[1,0].set_title("$D_3$")
    axes[1,0].scatter(X_N1, Y_N1, s=1)
    axes[1,0].plot(circle_X, circle_Y, c='r')

    r2 = np.sqrt(np.abs(samples) / z) * radius

    n_inside = np.count_nonzero(np.abs(r2) < radius)
    print("N2: n_inside / n_samples: %d / %d = %f" % (n_inside, n_samples, n_inside / n_samples))

    X_N2 = np.multiply(r2, np.cos(phi))
    Y_N2 = np.multiply(r2, np.sin(phi))

    axes[1,1].set_title("$D_4$")
    axes[1,1].scatter(X_N2, Y_N2, s=1)
    axes[1,1].plot(circle_X, circle_Y, c='r')


    plt.show()
