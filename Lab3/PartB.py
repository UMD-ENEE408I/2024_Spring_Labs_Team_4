import scipy
import matplotlib.pyplot as plt
import numpy as np

M1 = scipy.io.wavfile.read("M1.wav", mmap=False)
M2 = scipy.io.wavfile.read("M2.wav", mmap=False)
M3 = scipy.io.wavfile.read("M3.wav", mmap = False)



#caluclate the RMS values (This is root mean squared)
#THis can be accomplished with numpy operations for root, mean, and squares
#There seems to be an issue with values overflowing the size of the variables being use, so Ill make a function which makes a new array to take care of this
#converts to float so larger numbers are possible
def RMS(data):
    squares = []
    for i in range(len(data)):
        val = data[i]
        squares.append(float(val)*float(val))
    #take the mean
    m = np.mean(squares)
    return np.sqrt(m)

#use my new funtion to generate the RMS values
rms1 = RMS(M1[1])
rms2 = RMS(M2[1])
rms3 = RMS(M3[1])

#print the RMS values
print('rms1: ', rms1)
print('rms2: ', rms2)
print('rms3: ', rms3)






# R_xy[m] = infSUM_wrt_n( x[n] * y[n-m] )
# M1 --> y[n]
# M2 --> x[n]
R_xy = [0] * M2[1]
num = 0
for i in range(0,len(M1[1])): # time shift (m)
    for j in range(0,len(M2[1])): # infinite sum (n)
        R_xy[i] += M2[1][j]*M1[1][j-i]
    
    #less prints
    if num % 100 == 0:
        print("Iter: ",num)
    num += 1

maxI = np.argmax(R_xy)

# seconds = sample * (seconds / sample) = (samples) / (sample rate)
delay = maxI / M1[0]

print("Custom function:")
print("For ", maxI, "index delay, the time delay is ", delay)

# need to change the type because np array causes issues in np.correlate calculation
M1_ = M1[1].tolist()
M2_ = M2[1].tolist()

# compute using the default functions
R_xy_default = np.correlate(M2_,M1_, mode='full')
maxI = np.argmax(R_xy_default) - len(M1[1]) # because 24000 is the middle of cross-correlation
delay = maxI / M1[0]

#plt.plot(R_xy_default[10000:40000])
#plt.plot(M1_)
#plt.plot(M2_)
#plt.show()

print("Inbuild np.correlate function:")
print("For ", maxI, "index delay, the time delay is ", delay)