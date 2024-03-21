#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

def imu_data_callback(imu_msg):
    # Limitar las aceleraciones angulares a 0.02 (mínimo)
    if imu_msg.angular_velocity.x < 0.1 and imu_msg.angular_velocity.x > -0.1:
        imu_msg.angular_velocity.x = 0.0

    if imu_msg.angular_velocity.y < 0.1 and imu_msg.angular_velocity.y > -0.1:
        imu_msg.angular_velocity.y = 0.0

    if imu_msg.angular_velocity.z < 0.1 and imu_msg.angular_velocity.z > -0.1:
        imu_msg.angular_velocity.z = 0.0

    # Publicar los datos procesados en el nuevo tópico /imu/data_proccess
    pub.publish(imu_msg)

if __name__ == '__main__':
    rospy.init_node('imu_data_processor')

    # Suscribirse al tópico /imu/data
    rospy.Subscriber('/imu/data', Imu, imu_data_callback)

    # Publicar en el nuevo tópico /imu/data_proccess
    pub = rospy.Publisher('/imu/data_proccess', Imu, queue_size=10)

    rate = rospy.Rate(10)  # Frecuencia de publicación (10 Hz)

    try:
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass