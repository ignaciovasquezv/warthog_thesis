#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import math

# Variables para almacenar los datos
timestamps = []
angular_velocities = {'x': [], 'y': [], 'z': []}
linear_accelerations = {'x': [], 'y': [], 'z': []}
magnitude_velocities = []
magnitude_accelerations = []

# Función de callback para procesar los datos del tópico
def imu_callback(data):
    current_time = time.time()
    timestamps.append(current_time)
    
    for axis, value in zip(['x', 'y', 'z'], [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]):
        angular_velocities[axis].append(value)
    
    for axis, value in zip(['x', 'y', 'z'], [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]):
        linear_accelerations[axis].append(value)
    
    magnitude_vel = math.sqrt(data.angular_velocity.x ** 2 + data.angular_velocity.y ** 2 + data.angular_velocity.z ** 2)
    magnitude_velocities.append(magnitude_vel)

    magnitude_ac = math.sqrt(data.linear_acceleration.x ** 2 + data.linear_acceleration.y ** 2 + data.linear_acceleration.z ** 2)
    magnitude_accelerations.append(magnitude_ac)

def update_plot(frame):
    plt.clf()

    plt.subplot(2, 2, 1)
    for axis in ['x', 'y', 'z']:
        plt.plot(timestamps, angular_velocities[axis], label=f'Angular Velocity ({axis})')
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.legend()

    plt.subplot(2, 2, 2)
    for axis in ['x', 'y', 'z']:
        plt.plot(timestamps, linear_accelerations[axis], label=f'Linear Acceleration ({axis})')
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.legend()

    plt.subplot(2, 2, 3)
    plt.plot(timestamps, magnitude_velocities, label='Magnitude of Angular Velocity')
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.legend()

    plt.subplot(2, 2, 4)
    plt.plot(timestamps, magnitude_accelerations, label='Magnitude of Linear Acceleration')
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.legend()

    plt.tight_layout()

def imu_plot():
    rospy.init_node('imu_plot_node', anonymous=True)
    rospy.Subscriber('/imu/data', Imu, imu_callback)

    ani = FuncAnimation(plt.gcf(), update_plot, interval=1000) 
    plt.show()

if __name__ == '__main__':
    imu_plot()