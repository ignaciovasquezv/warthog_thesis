#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import numpy as np
import math

def filter_and_publish_points(input_topic, output_topic):
    rospy.init_node('pointcloud_segmenter', anonymous=True)
    rospy.Subscriber(input_topic, PointCloud2, filter_points_and_publish, callback_args=output_topic)
    rospy.spin()

def filter_points_and_publish(data, output_topic):
    # Obtener los ángulos mínimos y máximos deseados en radianes (-XX a XX grados)
    min_angle = math.radians(-150)
    max_angle = math.radians(150)
    
    # Convertir los datos de PointCloud2 a un array numpy
    gen = point_cloud2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
    points = np.array(list(gen))

    # Obtener ángulos a partir de las coordenadas X e Y
    angles = np.arctan2(points[:, 1], points[:, 0])

    # Filtrar puntos dentro del rango de ángulos deseado
    filtered_points = points[(angles >= min_angle) & (angles <= max_angle)]

    # Crear un nuevo mensaje PointCloud2 con los puntos filtrados
    header = data.header
    filtered_cloud = point_cloud2.create_cloud_xyz32(header, filtered_points)

    # Publicar el nuevo mensaje en el tópico de salida
    pub = rospy.Publisher(output_topic, PointCloud2, queue_size=10)
    pub.publish(filtered_cloud)
    #rospy.loginfo(f"Nube de puntos segmentada publicada en {output_topic}")

if __name__ == '__main__':
    input_topic = '/velodyne_points'  # Tópico de entrada
    output_topic = '/velodyne_points_segmented'  # Nuevo tópico para la nube de puntos segmentada
    filter_and_publish_points(input_topic, output_topic)

