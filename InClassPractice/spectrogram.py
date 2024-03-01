import scipy.signal
import scipy.io
import matplotlib.pyplot as plt


rate, data = scipy.io.wavfile.read('test.wav')

f,t,fig = scipy.signal.spectrogram(data)
plt.pcolormesh(t, f, fig, shading='gouraud')
plt.show()