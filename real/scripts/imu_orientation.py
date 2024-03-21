#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import math
import tf

timestamps = []
orientations = {'roll': [], 'pitch': [], 'yaw': []}

def imu_callback(data):
    current_time = time.time()
    timestamps.append(current_time / 60.0)
    
    quaternion = (
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w
    )
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll_deg = math.degrees(euler[0])
    pitch_deg = math.degrees(euler[1])
    yaw_deg = math.degrees(euler[2])

    orientations['roll'].append(roll_deg)
    orientations['pitch'].append(pitch_deg)
    orientations['yaw'].append(yaw_deg)

def update_plot(frame):
    plt.clf()
    plt.plot(timestamps, orientations['roll'], label='Roll')
    plt.plot(timestamps, orientations['pitch'], label='Pitch')
    plt.plot(timestamps, orientations['yaw'], label='Yaw')
    plt.xlabel('Time (min)')
    plt.ylabel('Angle (deg)')
    plt.title('IMU Orientations')
    plt.legend()

def plot_orientations():
    rospy.init_node('orientation_plot_node', anonymous=True)
    rospy.Subscriber('/um7/imu/data', Imu, imu_callback)

    ani = FuncAnimation(plt.gcf(), update_plot, interval=1000) 
    plt.show()

if __name__ == '__main__':
    plot_orientations()
