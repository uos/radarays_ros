import numpy as np
from matplotlib import pyplot as plt

from scipy.special import erf

def maxwell_boltzmann_pdf(x, a):
    return np.sqrt(2/np.pi) * x**2.0 * np.exp(-x**2.0 / (2*a**2)) / (a**3)

def maxwell_boltzmann_a_from_mode(mode):
    return mode / np.sqrt(2)

def maxwell_boltzmann_a_from_mean(mean):
    return mean / (2.0 * np.sqrt(2.0 / np.pi))



if __name__ == '__main__':
    print("Maxwell Boltzmann")

    x = np.linspace(0.0, 100.0, 1000)



    maxwell_boltzmann_a_from_mode(1)

    # y1 = maxwell_boltzmann_pdf(x, a=maxwell_boltzmann_a_from_mode(2))
    # y2 = maxwell_boltzmann_pdf(x, a=maxwell_boltzmann_a_from_mode(5))
    y = maxwell_boltzmann_pdf(x, a=maxwell_boltzmann_a_from_mode(10)) ** (1/10.0)


    

    a = 5
    mean = 2 * a * np.sqrt(2 / np.pi)
    mode = np.sqrt(2) * a
    print("a %f, mean %f, mode %f" % (a, mean, mode))

    # plt.plot(x, y1, c='blue', label='mode = 2')
    # plt.plot(x, y2, c='red', label='mode = 5')
    plt.plot(x, y)
    plt.show()