#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs.point_cloud2 as pc2
import numpy as np

def pointcloud_callback(point_cloud):
    # Convertir la PointCloud2 a un array numpy
    cloud_points = np.array(list(pc2.read_points(point_cloud, field_names=("x", "y", "z"), skip_nans=True)))

    # Ejemplo: Calcular un LaserScan simple con ángulo mínimo, máximo y rangos
    ranges = np.linalg.norm(cloud_points[:, :2], axis=1)
    angle_min = -np.pi / 1  # Ángulo mínimo en radianes
    angle_max = np.pi / 1   # Ángulo máximo en radianes

    # Crear el mensaje LaserScan
    laser_scan = LaserScan()
    laser_scan.header = point_cloud.header
    laser_scan.angle_min = angle_min
    laser_scan.angle_max = angle_max
    laser_scan.angle_increment = (angle_max - angle_min) / len(ranges)
    laser_scan.time_increment = 0.0  # Incremento de tiempo entre escaneos
    laser_scan.scan_time = 0.1       # Tiempo que toma un escaneo
    laser_scan.range_min = 0.0       # Rango mínimo
    laser_scan.range_max = 10.0      # Rango máximo
    laser_scan.ranges = ranges.tolist()

    # Publicar LaserScan
    global laser_scan_pub
    laser_scan_pub.publish(laser_scan)

def main():
    global laser_scan_pub
    rospy.init_node('pointcloud_to_laserscan')
    rospy.Subscriber('/points', PointCloud2, pointcloud_callback)
    laser_scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    main()
