#!/usr/bin/env python3

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf.transformations as tf
import numpy as np

last_valid_translation = None

def odometry_callback(msg):
    global last_valid_translation

    translation = msg.pose.pose.position
    rotation = msg.pose.pose.orientation

    if last_valid_translation is None:
        last_valid_translation = translation
    else:
        # Calcular las diferencias en las posiciones en los ejes x e y
        x_diff = abs(translation.x - last_valid_translation.x)
        y_diff = abs(translation.y - last_valid_translation.y)

        # Establecer los límites de cambio en x e y (por ejemplo, 1 metro)
        x_limit = 10.0
        y_limit = 10.0

        # Verificar y mantener la última posición válida si se supera el límite en x o y
        if x_diff > x_limit:
            translation.x = last_valid_translation.x

        if y_diff > y_limit:
            translation.y = last_valid_translation.y

    # Convertir el cuaternión de la orientación actual a una lista de elementos
    rotation_list = [rotation.x, rotation.y, rotation.z, rotation.w]

    # Aplicar la rotación fija de 90 grados sobre el eje Z (en radianes)
    rotation_fixed = tf.quaternion_from_euler(0, 0, np.pi / 2.0) #24/11
    #rotation_fixed = tf.quaternion_from_euler(0, 0, 0) #13/09

    # Multiplicar los cuaterniones para obtener la rotación resultante
    rotation_result = tf.quaternion_multiply(rotation_list, rotation_fixed)

    # Obtener la matriz de rotación a partir del cuaternión resultante
    rot_matrix = tf.quaternion_matrix(rotation_result)

    # Definir un vector en la dirección de la orientación de longitud -0.425 metros
    #additional_translation = [0.0, -0.425, 0.0, 1.0] #LiDAR
    #additional_translation = [0.0, -0.237, 0.0, 1.0] #ZED 13/09
    additional_translation = [0.1965, -0.83511, 0.0, 1.0] #ZED 24/11

    # Multiplicar el vector por la matriz de rotación para obtener la traslación en el marco de la odometría
    additional_translation_odom = np.dot(rot_matrix, additional_translation)[:3]

    # Restar la traslación adicional a la coordenada correspondiente
    translation.x -= additional_translation_odom[0]
    translation.y -= additional_translation_odom[1]
    translation.z -= additional_translation_odom[2]

    static_transformStamped.transform.translation.x = translation.x
    static_transformStamped.transform.translation.y = translation.y
    static_transformStamped.transform.translation.z = translation.z
    static_transformStamped.transform.rotation = Quaternion(*rotation_result)

    # Actualizar la última posición válida
    last_valid_translation = translation

if __name__ == '__main__':
    rospy.init_node('dynamic_tf2_broadcaster')
    
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = TransformStamped()

    static_transformStamped.header.frame_id = "odom"
    #static_transformStamped.child_frame_id = "velodyne"
    static_transformStamped.child_frame_id = "zed_link"

    rospy.Subscriber("/odometry/filtered", Odometry, odometry_callback)

    rate = rospy.Rate(100.0)  # Frecuencia de publicación de la transformación estática

    while not rospy.is_shutdown():
        static_transformStamped.header.stamp = rospy.Time.now()
        broadcaster.sendTransform(static_transformStamped)
        rate.sleep()

    rospy.Subscriber("/odometry/filtered", Odometry, odometry_callback)

    rate = rospy.Rate(100.0)  # Frecuencia de publicación de la transformación estática

    while not rospy.is_shutdown():
        static_transformStamped.header.stamp = rospy.Time.now()
        broadcaster.sendTransform(static_transformStamped)
        rate.sleep()

