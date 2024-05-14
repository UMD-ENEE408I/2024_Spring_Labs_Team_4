from scipy.io.wavfile import read
import matplotlib.pyplot as plt
from scipy import signal
import numpy as np

x = [4,4,4,4,6,8,10,8,6,4,4,4,4,4,4,4,4,4,4,4,4,4,4]
y = [4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,4,6,8,10,8,6,4,4]
correlation = signal.correlate(y-np.mean(y), x*5 - np.mean(x*5), mode="full")
lags = signal.correlation_lags(len(x), len(y), mode="full")
lag = lags[np.argmax(abs(correlation))]
print(lag)


# read audio samples
input_data = read("mic1.wav")
input2 = read("mic2.wav")
audio = input_data[1]
audio2 = input2[1]
plt.subplot(2,1,1)
plt.plot(audio)
plt.subplot(2,1,2)
plt.plot(audio2)
# label the axes
plt.ylabel("Amplitude")
plt.xlabel("Time")
# set the title  
plt.title("Sample Wav")
# display the plot
plt.show()