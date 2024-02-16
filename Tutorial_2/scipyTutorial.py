import numpy as np
from scipy import linalg
import scipy.optimize as opt
from scipy.fftpack import fft
import matplotlib.pyplot as plt

a = np.array([[3, 1], [1, 2]])
b = np.array([9, 8])

y = linalg.solve(a,b)

print(y)

def f(x):
    return x**2 + 2*x

result = opt.minimize(f, x0=0)

print(result.fun)
print(result.x)

def f(x):
    return np.sin(100*np.pi*x) +0.5*np.sin(160*np.pi*x)

x = np.arange(0,5,0.001)

ffty = fft(f(x))

plt.plot(x,ffty)
plt.show()