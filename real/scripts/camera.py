#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

last_image = None

# Función de devolución de llamada para procesar la imagen recibida del tópico
def image_callback(msg):
    global last_image
    try:
        # Convierte el mensaje de imagen a una imagen OpenCV
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Almacena la última imagen recibida
        last_image = cv_image

    except Exception as e:
        print(e)

if __name__ == "__main__":
    # Inicializa el nodo ROS
    rospy.init_node("image_viewer", anonymous=True)

    # Inicializa el puente CvBridge
    bridge = CvBridge()

    # Suscribe la función de devolución de llamada al tópico de la cámara (seleccionar topico)
    rospy.Subscriber("/zed2_node/depth/depth_registered", Image, image_callback)
    #rospy.Subscriber("/camera/image_color", Image, image_callback)

    # Crea la ventana de visualización
    cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)

    # Espera hasta que se cierre la ventana de visualización
    while not rospy.is_shutdown():
        if last_image is not None:
            # Muestra la última imagen recibida en la ventana
            cv2.imshow("Camera", last_image)
        
        # Espera un breve período para que se muestre la imagen y procese eventos de ventana
        cv2.waitKey(100)

    cv2.destroyAllWindows()