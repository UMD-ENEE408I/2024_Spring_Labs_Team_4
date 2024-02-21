import numpy as np
from scipy.signal import butter,filtfilt
from scipy.io import wavfile
from scipy.fft import fft, fftfreq
import matplotlib.pyplot as plt

rate, wavFile = wavfile.read("Cafe_with_noise.wav", mmap=False)
print(wavFile)

# Used some code from here to develop the filter:
# https://medium.com/analytics-vidhya/how-to-filter-noise-with-a-low-pass-filter-python-885223e5e9b7
T = 1/rate      # Sample Period
fs = rate       # sample rate, Hz
cutoff = 700    # desired cutoff frequency of the filter, Hz
nyq = 0.5 * fs  # Nyquist Frequency
order = 4       # high order because the high freq is so strong

def butter_lowpass_filter(data, cutoff, fs, order):
    normal_cutoff = cutoff / nyq
    # Get the filter coefficients 
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y

times = np.arange(len(wavFile))/rate

# Filter the data, and plot both the original and filtered signals.
y = butter_lowpass_filter(wavFile, cutoff, fs, order)
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 3), sharex=True, sharey=True)
ax1.plot(times,wavFile)
ax1.set_title("Original Signal")
ax1.margins(0, .1)
ax1.grid(alpha=.5, ls='--')
ax2.plot(times,y)
ax2.set_title("Low-Pass Filtered")
ax2.grid(alpha=.5, ls='--')
plt.tight_layout()
plt.show()

# plot fft
N = len(times)
yf = fft(y)
xf = fftfreq(N, T)[:N//2]
plt.plot(xf, 2.0/N * np.abs(yf[0:N//2]))
plt.show()

wavfile.write("Cafe.wav",rate,y.astype(np.int16))