#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu, PointCloud2
import csv

# Función de callback para datos de odometria
def odometry_callback(data):
    global odometry_writer
    # Extraer datos relevantes del mensaje de odometria
    position_x = data.pose.pose.position.x
    position_y = data.pose.pose.position.y

    orientation_z = data.pose.pose.orientation.z
    orientation_w = data.pose.pose.orientation.w

    vel_linear_x = data.twist.twist.linear.x
    vel_angular_z = data.twist.twist.angular.z

    # Escribir los datos en el archivo CSV
    odometry_writer.writerow([rospy.Time.now(), position_x, position_y, 
                            orientation_z, orientation_w, 
                            vel_linear_x, vel_angular_z])

# Función de callback para datos GPS
def gps_callback(data):
    global gps_writer
    # Extraer datos relevantes del mensaje NavSatFix
    latitude = data.latitude
    longitude = data.longitude
    altitude = data.altitude

    # Escribir los datos en el archivo CSV
    gps_writer.writerow([rospy.Time.now(), latitude, longitude, altitude])  

def imu_callback(data):
    global imu_writer
    orientation_x = data.orientation.x
    orientation_y = data.orientation.y
    orientation_z = data.orientation.z
    orientation_w = data.orientation.w

    vel_angular_x = data.angular_velocity.x
    vel_angular_y = data.angular_velocity.y
    vel_angular_z = data.angular_velocity.z

    linear_acceleration_x = data.linear_acceleration.x
    linear_acceleration_y = data.linear_acceleration.y
    linear_acceleration_z = data.linear_acceleration.z

    # Escribir los datos en el archivo CSV
    imu_writer.writerow([rospy.Time.now(), orientation_x, orientation_y, orientation_z, orientation_w,
                        vel_angular_x, vel_angular_y, vel_angular_z,
                        linear_acceleration_x, linear_acceleration_y, linear_acceleration_z])  

if __name__ == '__main__':
    rospy.init_node('data_logger_node', anonymous=True)

    # Crear archivos CSV para datos de odometría y GPS
    odometry_file = open('/home/nacho/catkin_ws/src/package_nacho/data/liceo/odometry_data.csv', 'w')
    gps_file = open('/home/nacho/catkin_ws/src/package_nacho/data/liceo/gps_data.csv', 'w')
    imu_file = open('/home/nacho/catkin_ws/src/package_nacho/data/liceo/imu_data.csv', 'w')

    odometry_writer = csv.writer(odometry_file)
    gps_writer = csv.writer(gps_file)
    imu_writer = csv.writer(imu_file)

    # Encabezados 
    odometry_writer.writerow(['Timestamp', 'Position_X', 'Position_Y', 
                            'Orientation_Z', 'Orientation_W', 
                            'Vel_lin_X', 'Vel_ang_Z'])  

    gps_writer.writerow(['Timestamp', 'Latitude', 'Longitude', 'Altitude'])

    imu_writer.writerow(['Timestamp', 'Orientation_X', 'Orientation_Y', 'Orientation_Z', 'Orientation_W',
                        'Vel_ang_X', 'Vel_ang_Y', 'Vel_ang_Z',
                        'Acc_linear_X', 'Acc_linear_Y', 'Acc_linear_Z'])

    # Subscripcion a topicos
    rospy.Subscriber('/odometry/filtered', Odometry, odometry_callback)
    rospy.Subscriber('/piksi/navsatfix_best_fix', NavSatFix, gps_callback)
    rospy.Subscriber('/um7/imu/data', Imu ,imu_callback)

    rospy.spin()

    odometry_file.close()
    gps_file.close()
