

import numpy as np
import matplotlib.pyplot as plt



# Make graph
fig, ax = plt.subplots()


# Define span of domain
z = np.linspace(0, 100)


# Define funciton parameters
Amplitude = 2
freq = 200
phase = 30

# Define the funciton to be plotted
func = Amplitude * np.cos(2 * np.pi * freq * z + phase)

# plot function
ax.plot()



