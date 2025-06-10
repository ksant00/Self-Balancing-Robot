import numpy as numpy
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import sys
import time
import serial

# function to read data from serial
def data_gen(ser, data_sync, start_time, time_steps, theta_s_values, theta_l_values, theta_s_values_a, theta_l_values_a, theta_s_values_g, theta_l_values_g):
    while True:
        if ser.in_waiting > 0:
            # reads line of serial and decodes utf-8 encodeing and strips unwanted characters
            string = ser.readline().decode('utf-8').strip()
        
            # splits the values into two seperate lists and stores them as float values
            theta_s_a, theta_l_a, theta_s_g, theta_l_g, theta_s, theta_l = map(float, string.split(","))

            with data_sync:
                # adds new theta values to list and updates time
                theta_s_values.append(-theta_s)
                theta_l_values.append(theta_l)
                theta_s_values_a.append(-theta_s_a)
                theta_l_values_a.append(theta_l_a)
                theta_s_values_g.append(-theta_s_g)
                theta_l_values_g.append(theta_l_g)
                time_steps.append(time.time() - start_time)

                if len(time_steps) > 50:
                    # if length of list is greater than 50 then remove first from list
                    time_steps.pop(0)
                    theta_s_values.pop(0)
                    theta_l_values.pop(0)
                    theta_s_values_a.pop(0)
                    theta_l_values_a.pop(0)
                    theta_s_values_g.pop(0)
                    theta_l_values_g.pop(0)

# function to update plot
def run_plot(frame, data_sync, line1, line2, line3, line4, line5, line6, time_steps, theta_s_values, theta_l_values, theta_s_values_a, theta_l_values_a, theta_s_values_g, theta_l_values_g, ax):
    with data_sync:
        # sets line values for plot
        line1.set_xdata(time_steps)
        line1.set_ydata(theta_s_values)

        line2.set_xdata(time_steps)
        line2.set_ydata(theta_l_values)

        line3.set_xdata(time_steps)
        line3.set_ydata(theta_s_values_g)

        line4.set_xdata(time_steps)
        line4.set_ydata(theta_l_values_g)

        line5.set_xdata(time_steps)
        line5.set_ydata(theta_s_values_a)

        line6.set_xdata(time_steps)
        line6.set_ydata(theta_l_values_a)

        # adjust time axis
        if len(time_steps) > 1:
            ax.set_xlim(time_steps[0], time_steps[-1])

    return line1, line2


def main() -> None:
    #opens serial port
    ser = serial.Serial(
    port = '/dev/tty.usbmodem1401',
    baudrate = 9600,
    timeout = 1
    )

    # sets time of code startup as base time
    start_time = time.time()
    data_sync = threading.Lock()

    # list for values
    time_steps = []
    theta_s_values = []
    theta_l_values = []
    theta_s_values_a = []
    theta_l_values_a = []
    theta_s_values_g = []
    theta_l_values_g = []

    # settings for plot
    fig = plt.figure()
    ax = fig.subplots()
    ax.set_title("Theta Values")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Values (°)")
    line1, = ax.plot([], [], label = "theta_s", color = "r")
    line2, = ax.plot([], [], label = "theta_l", color = "b")
    line3, = ax.plot([], [], label = "theta_s_g", color = "g")
    line4, = ax.plot([], [], label = "theta_l_g", color = "y")
    line5, = ax.plot([], [], label = "theta_s_a", color = "c")
    line6, = ax.plot([], [], label = "theta_l_a", color = "m")
    ax.legend()
    ax.set_ylim(-90, 90)

    # sets serial reading as separate thread
    serial_thread = threading.Thread(
        target = data_gen, 
        args = (ser, data_sync, start_time, time_steps, theta_s_values, theta_l_values, theta_s_values_a, theta_l_values_a, theta_s_values_g, theta_l_values_g), 
        daemon = True
    )
    serial_thread.start()

    # start plot animation
    ani = animation.FuncAnimation(
        fig, 
        run_plot, 
        fargs = (data_sync, line1, line2, line3, line4,line5, line6, time_steps, theta_s_values, theta_l_values, theta_s_values_a, theta_l_values_a, theta_s_values_g, theta_l_values_g, ax), 
        interval = 75, 
        blit = False, 
        repeat = False,
        cache_frame_data = False
    )

    plt.show()

if __name__ == "__main__":
    sys.exit(main())
