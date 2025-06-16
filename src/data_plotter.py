#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from collections import deque

# Parameters
BUFFER_SIZE = 200  # Number of data points to keep in the plot

# Data buffer
glove_data = deque(maxlen=BUFFER_SIZE)

# Callback for /glove_data topic
def glove_callback(msg):
    glove_data.append(msg.data)

# Setup
def main():
    rospy.init_node('glove_plot_node', anonymous=True)
    rospy.Subscriber("/glove_data", Float32, glove_callback)

    # Set up the plot
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots()
    line, = ax.plot([], [])
    ax.set_ylim(-100, 100)  # Adjust based on your sensor range
    ax.set_title("Real-time Glove Sensor Data")
    ax.set_xlabel("Sample")
    ax.set_ylabel("Value")

    while not rospy.is_shutdown():
        line.set_ydata(glove_data)
        line.set_xdata(range(len(glove_data)))

        ax.relim()
        ax.autoscale_view()

        plt.pause(0.01)  # Refresh plot
        plt.draw()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
