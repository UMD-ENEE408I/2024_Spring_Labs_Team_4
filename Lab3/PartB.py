import scipy
import matplotlib.pyplot as plt
import numpy as np

M1 = scipy.io.wavfile.read("M1.wav", mmap=False)
M2 = scipy.io.wavfile.read("M2.wav", mmap=False)

# R_xy[m] = infSUM_wrt_n( x[n] * y[n-m] )
# M1 --> y[n]
# M2 --> x[n]
R_xy = [0] * M2[1]
num = 0
for i in range(0,len(M1[1])): # time shift (m)
    for j in range(0,len(M2[1])): # infinite sum (n)
        R_xy[i] += M2[1][j]*M1[1][j-i]
    print(num)
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