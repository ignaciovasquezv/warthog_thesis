# Implementación y análisis de herramienta de mapeo de entornos frutícolas mediante sensor LiDAR y cámara estéreo montados en plataforma robótica móvil Warthog
Este repositorio alberga una copia de respaldo del trabajo de título elaborado para la obtención del grado de Ingeniero Civil Eléctrico de la Universidad de O'Higgins. Incluye los códigos desarrollados e implementados para este propósito, así como archivos .launch y detalladas instrucciones sobre su utilización. Además, se encuentran disponibles archivos en formato PDF que contienen el trabajo de título, la presentación y manuales importantes.

### Instalación
Para utilizar este repositorio, es necesario clonarlo dentro de su directorio de trabajo correspondiente. Para ello, ejecute el siguiente comando en la terminal (reemplace 'ros_workspace' con el nombre de su espacio de trabajo):

```
cd ~/ros_workspace/src
git clone https://github.com/ignaciovasquezv/warthog_thesis.git

cd ~/ros_workspace/
catkin_make
source devel/setup.bash
```


## Vehículo terrestre no tripulado Warthog

[Warthog UGV Tutorials](https://www.clearpathrobotics.com/assets/guides/kinetic/warthog/index.html)

## Calibración y configuración del equipo
Para lograr una correcta configuración y calibración del equipo en cuestión, se deben seguir una serie de pasos y consideraciones clave. En este contexto, se centrarán en tres aspectos fundamentales: la configuración de la odometría, la configuración manual del URDF (Unified Robot Description Format) y la calibración de la cámara-LiDAR. Estos procesos son esenciales para garantizar el funcionamiento preciso y confiable de un robot en un entorno robótico y son cruciales para tareas de navegación autónoma, percepción del entorno y toma de decisiones. A continuación, se profundizará en cada uno de estos aspectos para comprender cómo llevar a cabo estas configuraciones de manera efectiva.

### Odometría

[robot_localization](https://docs.ros.org/en/melodic/api/robot_localization/html/index.html)

### Configuración "manual"

[tf](http://wiki.ros.org/tf)

### Calibración de cámara

[camera_calibration](http://wiki.ros.org/camera_calibration)

### Calibración de cámara y sensor LiDAR

[extender_lidar_camera_calib](https://github.com/AFEICHINA/extended_lidar_camera_calib)


## Base de datos y uso de esta


| Encabezado 1 | Encabezado 2 | Encabezado 3 |
| ------------ | ------------ | ------------ |
| Celda 1,1    | Celda 1,2    | Celda 1,3    |
| Celda 2,1    | Celda 2,2    | Celda 2,3    |
| Celda 3,1    | Celda 3,2    | Celda 3,3    |

```
rosparam set use_sim_time true
```

## Algoritmo de mapeo

### OctoMap
que es, instalacon, uso

### Visualización de mapas



