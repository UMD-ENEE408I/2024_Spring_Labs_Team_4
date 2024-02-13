import math
import scipy
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fsolve,minimize
from scipy.fft import fft,fftfreq

#SCIPY
print("SCIPY Question 1")
#solves the system of equations 3x+y = 9 and x+2y = 8
def eq(x):
    return [3*x[0] + x[1] - 9,x[0] + 2*x[1] - 8]
print(fsolve(eq,[0,0]))


print("SCIPY Question 2")
#fins the minimum of y = x^2 + 2x
def eq2(x):
    return [(x[0]**2) + (2*x[0])]
print(minimize(eq2,[-1,-1])
      )


N = 500
T = 1/1000
#This performs a fourier transormation to plot the frequency response of 
#f(x) = sin(100*pi*x) + 1/2 sin(160*pi*x)
#follows this : https://docs.scipy.org/doc/scipy/tutorial/fft.html
x = np.linspace(0,N*T,N)
y = np.sin(x*100*math.pi) + 0.5*np.sin(160*math.pi*x)
yf = fft(y)
xf = fftfreq(N, T)[:N//2]
plt.plot(xf, 2.0/N * np.abs(yf[0:N//2]))
plt.grid()
plt.show()
