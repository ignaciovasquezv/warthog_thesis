#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

def imu_data_callback(imu_msg):
    # Limitar las aceleraciones angulares a 0.02 (mínimo)
    if imu_msg.angular_velocity.x < 0.02 and imu_msg.angular_velocity.x > -0.02:
        imu_msg.angular_velocity.x = 0.0

    if imu_msg.angular_velocity.y < 0.02 and imu_msg.angular_velocity.y > -0.02:
        imu_msg.angular_velocity.y = 0.0

    if imu_msg.angular_velocity.z < 0.02 and imu_msg.angular_velocity.z > -0.02:
        imu_msg.angular_velocity.z = 0.0

    
    if imu_msg.linear_acceleration.x < 0.02 and imu_msg.linear_acceleration.x > -0.02:
        imu_msg.linear_acceleration.x = 0.0

    if imu_msg.linear_acceleration.y < 0.02 and imu_msg.linear_acceleration.y > -0.02:
        imu_msg.linear_acceleration.y = 0.0

    if imu_msg.linear_acceleration.z < 0.02 and imu_msg.linear_acceleration.z > -0.02:
        imu_msg.linear_acceleration.z = 0.0

    # Publicar los datos procesados en el nuevo tópico
    pub.publish(imu_msg)

if __name__ == '__main__':
    rospy.init_node('imu_data_processor')

    # Suscribirse al tópico /imu/data
    rospy.Subscriber('/um7/imu/data', Imu, imu_data_callback)

    # Publicar en el nuevo tópico /imu/data_proccess
    pub = rospy.Publisher('/imu/data_process', Imu, queue_size=10)

    rate = rospy.Rate(10) 

    try:
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass