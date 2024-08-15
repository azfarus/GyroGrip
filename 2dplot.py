import serial
import matplotlib.pyplot as plt
import time

# Open serial port
ser = serial.Serial('COM5', 115200)  # Change baudrate to match your device

# Initialize empty lists to store data
x_data = []
y_data = []

# Create a figure and axis for plotting
fig, ax = plt.subplots()
point, = ax.plot([], [], 'bo')  # Plot as blue circles

# Set axis labels and title
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_title('XY Plot')

ax.set_xlim(-1, 1)
ax.set_ylim(-1, 1)

# Function to update plot data
def update_plot(new_x, new_y):
    x_data.append(new_x)
    y_data.append(new_y)
    point.set_xdata(x_data)
    point.set_ydata(y_data)
    plt.draw()
    plt.pause(0.01)

# Main loop to continuously read data from serial port and plot

time.sleep(10)  
try:
    while True:
        # Read data from serial port
        data = ser.readline().strip()
        if data:
            try:
                x, z, y = map(float, data.split())  # Assuming data is space-separated floats
                update_plot(x, y)
            except ValueError:
                print("Invalid data received:", data)
except KeyboardInterrupt:
    print("Plotting stopped by user")

# Close serial port
ser.close()
