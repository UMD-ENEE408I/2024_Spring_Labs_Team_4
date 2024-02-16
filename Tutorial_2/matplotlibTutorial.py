import numpy as np
import matplotlib.pyplot as plt


x = np.arange(0, 2*np.pi, 0.1)
y = np.sin(x)

plt.plot(x,y)

plt.xlabel("X Values")
plt.ylabel("Y Values")

plt.show()

fig = plt.figure()
x3d = np.arange(0, 2*np.pi, 0.1)
y3d = np.arange(0, 2*np.pi, 0.1)
X, Y = np.meshgrid(x3d, y3d)

z3d = np.sin(np.sqrt(np.square(X) + np.square(Y)))

fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
ax.plot_surface(X, Y, z3d, vmin=z3d.min() * 2)

ax.set(xticklabels=[],
       yticklabels=[],
       zticklabels=[])

plt.show()
