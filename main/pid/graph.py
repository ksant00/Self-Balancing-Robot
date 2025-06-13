import numpy as numpy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import sys
import time
import serial

# function to read data from serial
def data_gen(ser, data_sync, start_time, time_steps, temp_values):
    while True:
        if ser.in_waiting > 0:
            # reads line of serial and decodes utf-8 encodeing and strips unwanted characters
            string = ser.readline().decode('utf-8').strip()
        
            # stores string as float value
            temp = float(string)

            with data_sync:
                # adds new theta values to list and updates time
                temp_values.append(temp)
                time_steps.append(time.time() - start_time)

                if len(time_steps) > 50:
                    # if length of list is greater than 50 then remove first from list
                    time_steps.pop(0)
                    temp_values.pop(0)

# function to update plot
def run_plot(frame, data_sync, line1, time_steps, temp_values, ax):
    with data_sync:
        # sets line values for plot
        line1.set_xdata(time_steps)
        line1.set_ydata(temp_values)

        # adjust time axis
        if len(time_steps) > 1:
            ax.set_xlim(time_steps[0], time_steps[-1])

    return line1


def main() -> None:
    #opens serial port
    ser = serial.Serial(
    port = '/dev/tty.usbmodem101',
    baudrate = 9600,
    timeout = 1
)

    # sets time of code startup as base time
    start_time = time.time()
    data_sync = threading.Lock()

    # list for values
    time_steps = []
    temp_values = []

    # settings for plot
    fig = plt.figure()
    ax = fig.subplots()
    ax.set_title("Temperature vs Time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Temperature (°C)")
    line1, = ax.plot([], [], label = "Temperature", color = "r")
    ax.legend()
    ax.set_ylim(-90, 90)

    # sets serial reading as separate thread
    serial_thread = threading.Thread(
        target = data_gen, 
        args = (ser, data_sync, start_time, time_steps, temp_values), 
        daemon = True
    )
    serial_thread.start()

    # start plot animation
    ani = animation.FuncAnimation(
        fig, 
        run_plot, 
        fargs = (data_sync, line1, time_steps, temp_values, ax), 
        interval = 50, 
        blit = False, 
        repeat = False,
        cache_frame_data = False
    )

    plt.show()

if __name__ == "__main__":
    sys.exit(main())