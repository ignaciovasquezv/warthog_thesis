#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

last_image = None

# Función de devolución de llamada para procesar la imagen comprimida recibida del tópico
def image_callback(msg):
    global last_image
    try:
        # Convierte el mensaje de imagen comprimida a una imagen OpenCV
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        # Almacena la última imagen recibida
        last_image = cv_image
    except Exception as e:
        print(e)

if __name__ == "__main__":
    # Inicializa el nodo ROS
    rospy.init_node("image_viewer", anonymous=True)

    # Inicializa el puente CvBridge
    bridge = CvBridge()

    # Suscribe la función de devolución de llamada al tópico de la cámara comprimida (seleccionar topico)
    #rospy.Subscriber("/camera/image_color/compressed", CompressedImage, image_callback)
    rospy.Subscriber("/zed2_node/rgb/image_rect_color/compressed", CompressedImage, image_callback)

    # Crea la ventana de visualización
    cv2.namedWindow("Camera Compressed", cv2.WINDOW_NORMAL)

    # Espera hasta que se cierre la ventana de visualización
    while not rospy.is_shutdown():
        if last_image is not None:
            # Muestra la última imagen comprimida recibida en la ventana
            cv2.imshow("Camera Compressed", last_image)
        
        # Espera un breve período para que se muestre la imagen y procese eventos de ventana
        cv2.waitKey(1)

    cv2.destroyAllWindows()