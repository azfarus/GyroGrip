import serial
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication

# Open serial port
ser = serial.Serial('COM6', 115200 , timeout=100.0)  # Change baudrate to match your device

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
x_data = []
y_data = []

# Function to update plot data
def update_plot(new_x, new_y):
    x_data.append(new_x)
    y_data.append(new_y)
    curve.setData(x_data, y_data)

# Main loop to continuously read data from serial port and plot
def update():
    global ser
    if ser.in_waiting > 0:
        data = ser.readline().decode().strip()
        if data:
            try:
                x, y= map(float, data.split())  # Assuming data is space-separated floats
                update_plot(x, y)
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
