import scipy
import matplotlib.pyplot as plt

wavFile = scipy.io.wavfile.read("human_voice.wav", mmap=False)
print(wavFile)
plt.plot(wavFile[1])

wavFile_D = wavFile[1]
plt.plot(wavFile_D[0:len(wavFile_D):6])
print(len(wavFile_D[0:len(wavFile_D):6]))
print(len(wavFile[1]))
plt.show()

scipy.io.wavfile.write("human_voice_downsample.wav",8000,wavFile_D[0:len(wavFile_D):6])