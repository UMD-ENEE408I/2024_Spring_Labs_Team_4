import pyaudio
import wave
import sys
import scipy
from scipy import signal
import scipy.io
import scipy.io.wavfile
import scipy.signal as signal
from scipy.signal import correlation_lags
import matplotlib.pyplot as plt
import numpy as np

#following https://people.csail.mit.edu/hubert/pyaudio/ for basic audio recording


#for testing on my laptop, these are the indicies of the microphones on here, this should be changed for final implementation on nvidia
num1 = 24
num2 = 25
CHUNK = 1024
TIME = 1 #second

def get_recording_on_both():
    p = pyaudio.PyAudio()

    info = p.get_host_api_info_by_index(0)
    numdevices = info.get('deviceCount')
    num = 0
    for i in range(0, numdevices):
        if (p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
            print("Input Device id", i, "-", p.get_device_info_by_host_api_device_index(0, i).get('name'), " Channels: ", p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels'), " dict: ", p.get_device_info_by_host_api_device_index(0,i).get("defaultSampleRate"))

        name = p.get_device_info_by_host_api_device_index(0, i).get('name')
        if (name == 'default'):
            num = i
            print(num)

    with wave.open('mic1.wav','wb') as mic1, wave.open('mic2.wav','wb') as mic2:
        
        
        mic1.setnchannels(1) #was 2
        mic1.setsampwidth(p.get_sample_size(pyaudio.paInt16))
        mic1.setframerate(44100) #44100

        
        mic2.setnchannels(1) #was 2
        mic2.setsampwidth(p.get_sample_size(pyaudio.paInt16))
        mic2.setframerate(44100)


        #switched channels to 1
        #44100 origionally
        stream1 = p.open(format = pyaudio.paInt16, channels = 1, rate = 48000,input = True,input_device_index = num1)
        stream2 = p.open(format = pyaudio.paInt16, channels = 1, rate = 48000,input = True,input_device_index = num2)


        print('recording....')

        for _ in range(0, 44100 // CHUNK*TIME):
            mic1.writeframes(stream1.read(CHUNK,exception_on_overflow = False))
            mic2.writeframes(stream2.read(CHUNK, exception_on_overflow = False))
        print("Done")

        stream1.close()
        stream2.close()
        
    #end the audio
    p.terminate()


#this function will return either a 0 or 1.
#a zero indecates that mic1 got the sound first
#a 1 indecates that mic2 got the sound first
def compareAudioFiles(filepath1,filepath2):

    mic1File = scipy.io.wavfile.read(filepath1, mmap=False)
    mic2File = scipy.io.wavfile.read(filepath2, mmap=False)

    # need to change the type because np array causes issues in np.correlate calculation
    file1 = mic1File[1].tolist()
    file2 = mic2File[1].tolist()

    #print(file1)
    #print(file2)

    #take the absolute values to remove any negative numbers, they seem to mess with the correlation function
    sig1 = file1-np.mean(file1)
    sig2 = file2-np.mean(file2)
    #run the correlation
    R_xy_default = signal.correlate(sig1, sig2 , mode="full")
    n = len(sig1)
    #R_xy_default = signal.correlate(sig2, sig1, mode='same') / np.sqrt(signal.correlate(sig1, sig1, mode='same')[int(n/2)] * signal.correlate(sig2, sig2, mode='same')[int(n/2)])

    lags =scipy.signal.correlation_lags(len(sig1), len(sig2), mode="full")
    #gets the lag
    lag = lags[np.argmax(R_xy_default)]
    #lag = lag + 44100
    
    delay = lag / mic1File[0]

    print("Inbuild np.correlate function:")
    print("For ", lag, "index delay, the time delay is ", delay)

    #determine which way the sound is, if positive then it is on the left
    #if negative then it is on the right
    #if close to zero it is stright ahead
    #plt.subplot(3,1,1)
    #plt.plot(sig1)
    #plt.subplot(3,1,2)
    #plt.plot(sig2)
    #plt.subplot(3,1,3)
    #plt.plot(R_xy_default)
    #plt.show()

    THRESHOLD = 250

    if lag + THRESHOLD < 0:
        #very negative
        print("The sound is to the LEFT")

    elif lag - THRESHOLD > 0:
        #very positive
        print("The sound is to the RIGHT")
    else:
        #close to zero
        print("The sound is either stright ahead or behind!")
    




#first record
get_recording_on_both()
#then print the time delay
compareAudioFiles('mic1.wav','mic2.wav')







