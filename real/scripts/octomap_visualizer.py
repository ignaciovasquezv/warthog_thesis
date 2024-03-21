#!/usr/bin/env python3

import subprocess

nombre_archivo = input("Ingresa el nombre del archivo (con extensión): ")

ruta = "/home/nacho/catkin_ws/src/package_nacho/maps/"

comando = f"rosrun octomap_server octomap_server_node \"{ruta}{nombre_archivo}\""

# Ejecutar el comando en la consola
subprocess.run(comando, shell=True)