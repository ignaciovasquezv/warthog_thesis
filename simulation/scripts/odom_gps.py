#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Quaternion, Twist
import tf.transformations as tf
import time
import subprocess

class OdomPublisher:
    def __init__(self):
        rospy.init_node('odom_publisher', anonymous=True)
        self.initialized = False
        self.initial_lat = 0.0
        self.initial_lon = 0.0
        self.initial_alt = 0.0
        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_time = time.time()
        self.current_yaw = 0.0
        self.prev_yaw = 0.0  # Inicializando prev_yaw

        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        rospy.Subscriber("/navsat/fix", NavSatFix, self.navsat_callback)
        rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        rospy.spin()

        comando = f"rosnode kill /ekf_localization"

        #subprocess.run(comando, shell=True)

    def navsat_callback(self, data):
        if not self.initialized:
            self.initial_lat = data.latitude
            self.initial_lon = data.longitude
            self.initial_alt = data.altitude
            self.initialized = True
            self.prev_x = 0.0
            self.prev_y = 0.0
            self.prev_time = time.time()
            return

        x = (data.longitude - self.initial_lon) * 111319.0
        y = (data.latitude - self.initial_lat) * 111319.0
        z = data.altitude - self.initial_alt

        current_time = time.time()
        dt = current_time - self.prev_time

        linear_speed_x = (x - self.prev_x) / dt
        linear_speed_y = (y - self.prev_y) / dt

        self.prev_x = x
        self.prev_y = y
        self.prev_time = current_time

        self.publish_odom(x, y, z, linear_speed_x, linear_speed_y)

    def imu_callback(self, data):
        orientation_quaternion = data.orientation
        roll, pitch, yaw = tf.euler_from_quaternion([orientation_quaternion.x,
                                                     orientation_quaternion.y,
                                                     orientation_quaternion.z,
                                                     orientation_quaternion.w])
        self.current_yaw = yaw

    def publish_odom(self, x, y, z, linear_speed_x, linear_speed_y):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = z

        odom_quat = tf.quaternion_from_euler(0, 0, self.current_yaw)
        odom.pose.pose.orientation = Quaternion(*odom_quat)

        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = linear_speed_x
        odom.twist.twist.linear.y = linear_speed_y

        current_time = time.time()
        dt = current_time - self.prev_time
        if dt != 0:
            angular_speed = (self.current_yaw - self.prev_yaw) / dt
            odom.twist.twist.angular.z = angular_speed
            self.prev_yaw = self.current_yaw
            self.prev_time = current_time

        self.odom_pub.publish(odom)

if __name__ == '__main__':
    try:
        OdomPublisher()
    except rospy.ROSInterruptException:
        pass
