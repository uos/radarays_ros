import numpy as np
from matplotlib import pyplot as plt



def noise_amplitude(signal):
    signal_min = np.min(signal)
    signal_max = np.max(signal)
    signal_amp = signal_max - signal_min

    signal_ = 1.0 - ((signal - signal_min) / signal_amp)

    # signal between 0 and 1
    noise_at_0 = signal_amp * 0.01
    noise_at_1 = signal_amp * 0.4

    signal__ = signal_ ** 4.0

    noise_amp = ((1.0 - signal__) * noise_at_0 + signal__ * noise_at_1)
    
    return noise_amp


N = 1000

x = np.linspace(0, 132, N)

fig, axs = plt.subplots(2, 2)


# Raytracing
y_signal_0 = -((x-1.0)*5.0)**2.0 + 10.0
y_signal_0[y_signal_0 < 0.0] = 0.0

y_signal_1 = -((x-11.0)*5.0)**2 + 85.0
y_signal_1[y_signal_1 < 0.0] = 0.0

y_signal_2 = -((x-36.0)*1.5)**2 + 82.0
y_signal_2[y_signal_2 < 0.0] = 0.0

y_signal_3 = -((x-60.0)*1.7)**2 + 60.0
y_signal_3[y_signal_3 < 0.0] = 0.0

y_signal = y_signal_0 + y_signal_1 + y_signal_2 + y_signal_3

axs[0, 0].plot(x, y_signal)
axs[0, 0].set_title('Signal produced by raytracing')




# y_real = y_real

# compute signal dependend noise
noise_type = 1

# general noise settings

# noise amplitude dependent of signal
# - 0: for high signals
# - 1: 
# noise_amp = [(0.0, 40.0), (140.0, 0.01)]
# 0: uniform noise
# 1: perlin noise
perlin_factor = 100



A = noise_amplitude(y_signal)

y_noise = np.zeros(N)
if noise_type == 0:
    y_noise = np.random.random_sample(N) - 0.5
    y_noise = np.multiply(y_noise, A)
    axs[0,1].set_title("Uniform Noise")
elif noise_type == 1:
    import perlin
    p = perlin.Perlin(6789)

    for i in range(N):
        y_noise[i] = p.one(i * perlin_factor) / 50.0

    y_noise = np.multiply(y_noise, A)
    axs[0,1].set_title("Perlin Noise")


axs[0,1].plot(x, y_noise)


# k_energy = 0.05
noise_energy_max = 80.0
noise_energy_min = 5.0
energy_loss = 0.05

y_noise = y_noise + (noise_energy_max - noise_energy_min) / np.e**(energy_loss * x) + noise_energy_min

axs[1,0].set_title("Noise energy loss")
axs[1,0].plot(x, y_noise)


# y = y_noise
# Combine signal with noise
# y = np.max(np.vstack((y_signal, y_noise)), axis=0)
y = y_signal + y_noise

axs[1,1].set_title("Noisy signal")
axs[1,1].plot(x, y)


plt.show()



