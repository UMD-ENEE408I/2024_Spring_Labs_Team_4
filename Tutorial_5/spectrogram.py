import scipy.io.wavfile as wavfile
import matplotlib.pyplot as plt
'''
rate, data = wavfile.read('test.wav')

f,t,fig = scipy.signal.spectrogram(data)
plt.pcolormesh(t, f, fig, shading='gouraud')
plt.show()
'''

Fs, audio = wavfile.read('test.wav')
audio = audio[:,0]

power, freq, time, imageAx = plt.specgram(audio, Fs=Fs)
plt.show()

