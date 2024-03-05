import pyaudio
import wave
import sys

audio = pyaudio.PyAudio()

info = audio.get_host_api_info_by_index(0)
numdevices = info.get('deviceCount')
num = 0
for i in range(0, numdevices):
    if (audio.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
        print("Input Device id", i, "-", audio.get_device_info_by_host_api_device_index(0, i).get('name'))
    if (audio.get_device_info_by_host_api_device_index(0, i).get('name') == 'default'):
        num = i

#following https://people.csail.mit.edu/hubert/pyaudio/
#record a sound

CHUNK = 1024
TIME = 3

with wave.open('test.wav','wb') as wf:
    p = pyaudio.PyAudio()
    wf.setnchannels(2)
    wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
    wf.setframerate(44100)

    stream = p.open(format = pyaudio.paInt16, channels = 2, rate = 44100,input = True, input_device_index = num)

    print('recording....')

    for _ in range(0, 44100 // CHUNK*TIME):
        wf.writeframes(stream.read(CHUNK))
    print("Done")

    stream.close()
    p.terminate()

