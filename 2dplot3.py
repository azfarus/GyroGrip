# Import required libraries
import serial
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication
from collections import deque

# Open serial port
ser = serial.Serial('COM6', 115200, timeout=100.0)  # Change baudrate to match your device

# Create a PyQtGraph application
app = QApplication([])

# Create a plot window
win = pg.plot()
win.setWindowTitle('XY Plot')
win.setLabel('left', 'Y')
win.setLabel('bottom', 'X')
win.setXRange(-1, 1)
win.setYRange(-1, 1)
curve = win.plot([], [], pen='b')

# Initialize empty lists to store data
x_data = deque([], maxlen=10)
y_data = deque([], maxlen=10)
bool_data = deque([], maxlen=10)

# Function to update plot data
# Function to update plot data
def update_plot(new_x, new_y, bool_values):
    x_data.append(new_x)
    y_data.append(new_y)
    bool_data.append(bool_values)
    
    # Clear previous plot
    curve.clear()
    
    # Plot the x and y data
    curve.setData(x_data, y_data)
    
    # Plot boolean values
    for i, (x, y, bool_val) in enumerate(zip(x_data, y_data, bool_data)):
        for j, val in enumerate(bool_val):
            if val:  # True
                symbol = 'o'  # Circle symbol
                color = (255, 0, 0)  # Red color
            else:  # False
                symbol = 's'  # Square symbol
                color = (0, 0, 255)  # Blue color
            scatter = pg.ScatterPlotItem()
            scatter.addPoints([{'pos': (x, y), 'symbol': symbol, 'size': 10, 'pen': pg.mkPen(None), 'brush': pg.mkBrush(color)}])
            win.addItem(scatter)


# Main loop to continuously read data from serial port and plot
def update():
    global ser
    if ser.in_waiting > 0:
        data = ser.readline().decode().strip()
        if data:
            try:
                values = data.split()
                x, y = map(float, values[:2])  # Assuming first two values are floats
                bool_values = list(map(bool, map(int, values[2:])))  # Convert remaining values to boolean
                update_plot(x, y, bool_values)
            except ValueError:
                print("Invalid data received:", data)

# Start the application event loop
if __name__ == '__main__':
    update_timer = pg.QtCore.QTimer()
    update_timer.timeout.connect(update)
    update_timer.start(0)
    QApplication.instance().exec_()

# Close serial port
ser.close()
