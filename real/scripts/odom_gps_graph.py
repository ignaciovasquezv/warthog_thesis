#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Lee los datos desde los archivos CSV (Ingresar ruta acorde)
gps_data = pd.read_csv('/home/nacho/catkin_ws/src/package_nacho/data/gps_data_2411_2.csv').iloc[0:]
odometry_data = pd.read_csv('/home/nacho/catkin_ws/src/package_nacho/data/odometry_data_2411_2.csv').iloc[0:]

escala_conversion_lat = 111319  
escala_conversion_lon = ((40075*np.cos(np.radians(gps_data['Latitude'])))/360)*1000

# Restar el primer valor de odometría para iniciar en 0,0
odometry_data['Position_X'] -= odometry_data['Position_X'].iloc[0]
odometry_data['Position_Y'] -= odometry_data['Position_Y'].iloc[0]

# Restar el primer valor de GPS para iniciar en 0,0
gps_data['Latitude'] -= gps_data['Latitude'].iloc[0]
gps_data['Longitude'] -= gps_data['Longitude'].iloc[0]

# Rotar los datos de la odometría en XX grados
theta = np.radians(-90)
rotated_odometry_x = odometry_data['Position_X'] * np.cos(theta) - odometry_data['Position_Y'] * np.sin(theta)
rotated_odometry_y = odometry_data['Position_X'] * np.sin(theta) + odometry_data['Position_Y'] * np.cos(theta)

# Reflejar los datos de GPS en el eje X
gps_data['Longitude'] = -gps_data['Longitude']

# Obtener la posición inicial de la odometría
initial_odometry_x = odometry_data['Position_X'].iloc[0]
initial_odometry_y = odometry_data['Position_Y'].iloc[0]

# Calcular las diferencias entre la posición inicial de GPS y las demás posiciones de GPS
diff_lat = gps_data['Latitude'] - gps_data['Latitude'].iloc[0]
diff_lon = gps_data['Longitude'] - gps_data['Longitude'].iloc[0]

# Convertir las diferencias de grados a metros usando la escala de conversión
relative_gps_lat = diff_lat * escala_conversion_lat + initial_odometry_x
relative_gps_lon = diff_lon * escala_conversion_lon + initial_odometry_y

# Graficar trayectorias de odometría y GPS
plt.figure()

# Trayectoria de Odometría
plt.scatter(rotated_odometry_x, rotated_odometry_y, label='Odometría (GPS)', color='blue', alpha=0.8)

# Trayectoria de GPS transformada a coordenadas relativas con la escala ajustada
plt.scatter(relative_gps_lat, relative_gps_lon, label='GPS (Relativo)', color='red', alpha=0.1)

title = str('"Fundo la Torre 24-11-2023"')

plt.title('Trayectorias: Odometría vs GPS (Relativo) \n' + title, fontsize=26)
plt.xlabel('Coordenada X / Latitud', fontsize=26)
plt.ylabel('Coordenada Y / Longitud', fontsize=26)
plt.legend(fontsize=26)
plt.grid(True)
plt.show()
