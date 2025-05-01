import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Simulate a real-time data stream (for demonstration purposes)
def data_stream():
    """Simulate streaming data by generating random numbers."""
    while True:
        yield np.random.random()

# Set up the figure and axis
fig, ax = plt.subplots()
x_data, y_data = [], []  # Store data for plotting
line, = ax.plot([], [], 'r-', label='Real-time data')

# Initialize the plot
def init():
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 1)
    ax.set_xlabel('Time')
    ax.set_ylabel('Value')
    ax.legend(loc='upper right')
    return line,

# Update function for the animation
def update(frame):
    x_data.append(frame)
    y_data.append(next(data_gen))  # Get the next value from the data stream

    # Limit the x-axis to the last 10 data points
    if len(x_data) > 10:
        x_data.pop(0)
        y_data.pop(0)

    line.set_data(x_data, y_data)  # Update the data
    return line,

# Create a generator for the data stream
data_gen = data_stream()

# Create the animation
ani = FuncAnimation(fig, update, frames=np.arange(0, 100), init_func=init, blit=True, interval=100)

plt.show()
