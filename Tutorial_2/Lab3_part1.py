from scipy.io import wavfile
import matplotlib.pyplot as plt
import numpy as np
import scipy.signal as sps

samplerate, data = wavfile.read('human_voice.wav')

print(samplerate)

length = data.shape[0] / samplerate

time = np.linspace(0., length, data.shape[0])
plt.plot(time, data, label="original sample rate")
plt.legend()
plt.xlabel("Time [s]")
plt.ylabel("Amplitude")
plt.show()

new_rate = 8000

number_of_samples = round(len(data) * float(new_rate) / samplerate)
new_data = sps.resample(data, number_of_samples)


time = np.linspace(0., length, new_data.shape[0])
plt.plot(time, new_data, label="new sample rate")
plt.legend()
plt.xlabel("Time [s]")
plt.ylabel("Amplitude")
plt.show()