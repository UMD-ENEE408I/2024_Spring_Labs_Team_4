import pyaudio
import wave
import sys


#following https://people.csail.mit.edu/hubert/pyaudio/
#record a sound

CHUNK = 1024
TIME = 3

with wave.open('test.wav','wb') as wf:
    p = pyaudio.PyAudio()
    wf.setnchannels(2)
    wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
    wf.setframerate(44100)

    stream = p.open(format = pyaudio.paInt16, channels = 2, rate = 44100,input = True)

    print('recording....')

    for _ in range(0, 44100 // CHUNK*TIME):
        wf.writeframes(stream.read(CHUNK))
    print("Done")

    stream.close()
    p.terminate()

