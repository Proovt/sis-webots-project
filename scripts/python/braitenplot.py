import numpy as np
import matplotlib.pyplot as plt

def brait(x):
    # Convert input to numpy array if it's not already
    x = np.array(x)
    # For values below 900, use 800 instead (ignore far sensors)
    adjusted_x = np.where(x < 900, 800, x)
    # Calculate nonlinear influence
    y = ((adjusted_x - 800) / (960 - 800)) ** 4
    return y

x = np.linspace(0, 1024, 10000)
y = brait(x)

plt.scatter(x, y, s=1)  # s=1 for smaller dots
plt.xlabel("Sensor reading")
plt.ylabel("Influence")
plt.title("Braitenberg Influence Function")
plt.show()