#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import csv

# Función de callback para datos de odometria
def odometry_callback(data):
    global odometry_writer
    # Extraer datos relevantes del mensaje de odometria
    position_x = data.pose.pose.position.x
    position_y = data.pose.pose.position.y

    # Escribir los datos en el archivo CSV
    odometry_writer.writerow([rospy.Time.now(), position_x, position_y])  # Add more data fields as needed

# Función de callback para datos GPS
def gps_callback(data):
    global gps_writer
    # Extraer datos relevantes del mensaje NavSatFix
    latitude = data.latitude
    longitude = data.longitude

    # Escribir los datos en el archivo CSV
    gps_writer.writerow([rospy.Time.now(), latitude, longitude])  # Add more data fields as needed

if __name__ == '__main__':
    rospy.init_node('data_logger_node', anonymous=True)

    # Crear archivos CSV para datos de odometría y GPS
    odometry_file = open('/home/nacho/warthog_ws/src/package_nacho/data/odometry_data.csv', 'w')
    gps_file = open('/home/nacho/warthog_ws/src/package_nacho/data/gps_data.csv', 'w')

    odometry_writer = csv.writer(odometry_file)
    gps_writer = csv.writer(gps_file)

    # Encabezados 
    odometry_writer.writerow(['Timestamp', 'Position_X', 'Position_Y'])  
    gps_writer.writerow(['Timestamp', 'Latitude', 'Longitude'])  

    # Subscripcion a topicos
    rospy.Subscriber('/odometry/filtered', Odometry, odometry_callback)
    rospy.Subscriber('/navsat/fix', NavSatFix, gps_callback)

    rospy.spin()

    odometry_file.close()
    gps_file.close()
