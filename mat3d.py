import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import serial
import time

# Initialize serial connection
ser = serial.Serial('COM3', 115200)  # Adjust COM port and baudrate as needed

# Define a cube
cube_vertices = np.array([
    [-1, -1, -1],
    [1, -1, -1],
    [1, 1, -1],
    [-1, 1, -1],
    [-1, -1, 1],
    [1, -1, 1],
    [1, 1, 1],
    [-1, 1, 1]
])

# Define edges of the cube
cube_edges = [
    (0, 1), (1, 2), (2, 3), (3, 0),
    (4, 5), (5, 6), (6, 7), (7, 4),
    (0, 4), (1, 5), (2, 6), (3, 7)
]

# Initialize the figure and axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Set aspect ratio to be equal

ax.set_box_aspect([1, 1, 1])

time.sleep(10)
# Function to update the plot in real-time
def update_plot():
    global cube_vertices
    # Read data from serial port
    line = ser.readline().strip()
    if len(line.split(b' ')) < 9 :
        return
    rotation_values = [float(x) for x in line.split(b' ')]  # Assuming data is comma-separated
    
    if len(rotation_values) < 9:
        return
    # Convert rotation values to rotation matrix
    rotation_matrix = np.reshape(rotation_values, (3, 3))
    
    # Apply transformation
    cube_vertices_new = np.dot(cube_vertices, rotation_matrix)
    
    # Clear previous plot
    ax.clear()
    
    # Plot cube with transformed vertices
    for edge in cube_edges:
        ax.plot3D(*zip(*[cube_vertices_new[vertex] for vertex in edge]), color='b')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Real-time 3D Matrix Rotation')

# Enter interactive mode
plt.ion()

# Continuous update loop
while True:
    update_plot()
    plt.draw()
    plt.pause(0.01)  # Adjust the pause time as needed

# Disable interactive mode
plt.ioff()
plt.show()
