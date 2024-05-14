import wave
import sys
import scipy.io.wavfile
import pyaudio
import numpy as np
import time

#This is a one microphone strategy using magnitude of the sound to determine which direction to travel for the sound source
#this is nessassary since cross correlation wont work well given the conditions we are working in the lab, sound seems to refelct off the walls causing
#cross correlation to not work consistently

CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1 #for usb mic
RATE = 44100
RECORD_SECONDS = 2


def record_file(name):
    with wave.open(name, 'wb') as wf:
        p = pyaudio.PyAudio()
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)

        stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True)

        print('Recording...')
        for _ in range(0, RATE // CHUNK * RECORD_SECONDS):
            wf.writeframes(stream.read(CHUNK))
        print('Done')

        stream.close()
        p.terminate()


#deteriming the RMS of the sounds
def compare_sounds(l_file,r_file):
    #this is the RMS function we created for Lab3
    def RMS(data):
        squares = []
        for i in range(len(data)):
            val = data[i]
            #print(val)
            squares.append(float(val)*float(val))
        #take the mean
        m = np.mean(squares)
        return np.sqrt(m)
    
    #open up the two files
    left_sound = scipy.io.wavfile.read(l_file, mmap=False)
    right_sound = scipy.io.wavfile.read(r_file, mmap=False)

    #print(left_sound)
    #print(left_sound[1].tolist())

    left_RMS = RMS(left_sound[1].tolist())
    right_RMS = RMS(right_sound[1].tolist())

    print("LEFT: ", left_RMS)
    print("RIGHT:", right_RMS)
    return "L" if left_RMS > right_RMS else "R"



#record_file("left.wav")
#time.sleep(5)
#record_file("right.wav")
#print(compare_sounds("left.wav","right.wav"))