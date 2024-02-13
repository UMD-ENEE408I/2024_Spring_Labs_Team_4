import numpy as np
import matplotlib.pyplot as plt
import math

#MATPLOTLIB Questions 1,2
#Plots a sine wave over one period
print("MATPLOTLIB QUESTIONS")
x = np.linspace(0,2*math.pi)
y = np.sin(x)

plt.plot(x,y)
#adds the axis labels to the graph
plt.xlabel("x Axis")
plt.ylabel('y axis')
plt.show()

#MATPLOT LIB Q 3
#creates a 3d surface plot of z = sine(sqrt(x^2 + y^2))
x = np.linspace(0,2*math.pi)
y = np.linspace(0,2*math.pi)
x,y = np.meshgrid(x,y)
z = np.sin(np.sqrt(x**2 + y**2))
fig,ax = plt.subplots(subplot_kw={"projection": "3d"})

ax.plot_surface(x,y,z)

plt.show()

