# Implementación y análisis de herramienta de mapeo de entornos frutícolas mediante sensor LiDAR y cámara estéreo montados en plataforma robótica móvil Warthog
Este repositorio alberga una copia de respaldo del trabajo de título elaborado para la obtención del grado de Ingeniero Civil Eléctrico de la Universidad de O'Higgins. Incluye los códigos desarrollados e implementados para este propósito, así como archivos .launch y detalladas instrucciones sobre su utilización. Además, se encuentran disponibles archivos en formato PDF que contienen el trabajo de título, la presentación y manuales importantes.

### Instalación
Para utilizar este repositorio, es necesario clonarlo dentro de su directorio de trabajo correspondiente. Para ello, ejecute el siguiente comando en la terminal (reemplace 'ros_workspace' con el nombre de su espacio de trabajo):

```bash
cd ~/ros_workspace/src
git clone https://github.com/ignaciovasquezv/warthog_thesis.git

cd ~/ros_workspace/
catkin_make
source devel/setup.bash
```


## Vehículo terrestre no tripulado Warthog
La plataforma robótica Warthog es una opción excepcional para una variedad de proyectos debido a su versatilidad y capacidad para operar en entornos desafiantes. Diseñada para enfrentar terrenos difíciles, desde suelos blandos y vegetación densa hasta lodo espeso y pendientes pronunciadas, Warthog es un vehículo terrestre no tripulado todo terreno con capacidad incluso para desplazarse brevemente en el agua. Su naturaleza de movimiento, junto con su tracción en las cuatro ruedas y habilidad anfibia, le otorgan una versatilidad sobresaliente en una amplia gama de situaciones. 

Además, su flexibilidad de personalización, con placas de montaje de carga útil y puertos de comunicación y alimentación accesibles, facilita la integración de sensores y equipos adicionales según las necesidades específicas de cada proyecto. Como primer paso, es importante tener en cuenta la ["Guía de Inicio Rápido del Robot Personalizado"](manuals/UDOH03_robotsmith_memo.pdf) disponible en el repositorio.

Clearpath Robotics ofrece tutoriales de sus robots en sus páginas web, donde además de la configuración y operación de la plataforma robótica, proporcionan un tutorial sobre cómo trabajar con el robot de manera simulada en Gazebo. Para acceder a este tutorial, diríjase al siguiente enlace: [Warthog UGV Tutorials](https://www.clearpathrobotics.com/assets/guides/kinetic/warthog/index.html).

<p align="center">
  <img src="images/warthog.png" width="432"/>
</p>


## Base de datos
La base de datos generada durante el desarrollo del trabajo de titulación se construyó a partir de las salidas a terreno con el robot, donde se recopiló información sobre todos los temas publicados por ROS, es decir, información de todos los sensores. Es importante destacar que estos archivos .bag están disponibles en una unidad física (HDD) en el Laboratorio de Robótica y Sistemas Inteligentes (RISLAB) de la Universidad de O'Higgins. La tabla a continuación muestra una descripción de la base de datos.

<div align="center">

| Lugar | Fecha | Descripción | Tamaño |
| --- | --- | --- | --- |
| Celda 1      | Celda 2      | Celda 3      | hola |
| Celda 4      | Celda 5      | Celda 6      | hola |

</div>

### Uso de la base de datos
Para utilizar la base de datos, es necesario ejecutar una serie de comandos en la consola.

1. Ejecutar 'roscore'
  
2. Especificar a ROS la utilización del tiempo de los datos.
   ```
   rosparam set use_sim_time true
   ```
3. Reproducir los datos desde el archivo .bag. Es importante destacar que se debe especificar qué tópico se desea reproducir. En este caso, se reproducen los datos del sensor LiDAR, la cámara estéreo ZED2, el sensor GPS, ... .
   ```
   rosbag play ...
   ```

## Calibración y configuración del equipo
Para lograr una correcta configuración y calibración del equipo en cuestión, se deben seguir una serie de pasos y consideraciones clave. En este contexto, se centrarán en tres aspectos fundamentales: la configuración de la odometría, la configuración manual del URDF (Unified Robot Description Format) y la calibración de la cámara-LiDAR. Estos procesos son esenciales para garantizar el funcionamiento preciso y confiable de un robot en un entorno robótico y son cruciales para tareas de navegación autónoma, percepción del entorno y toma de decisiones. A continuación, se profundizará en cada uno de estos aspectos para comprender cómo llevar a cabo estas configuraciones de manera efectiva.

### Odometría
La odometría es una técnica esencial en robótica que estima la posición y desplazamiento del robot mediante la recopilación y procesamiento de datos de sensores como encoders y sensores inerciales. Para configurar esta odometría se utiliza el paquete robot_localization de ROS, el cual ofrece algoritmos de fusión sensorial para mejorar la precisión de las estimaciones de posición. Esta configuración se realiza a través de un archivo .YAML (comúnmente llamado localization.yaml), donde se definen los parámetros a tener en cuenta para estimar esta odometría. Para obtener más información sobre la configuración de la odometría, puedes acceder al siguiente enlace: [robot_localization wiki](https://docs.ros.org/en/melodic/api/robot_localization/html/index.html).

```yaml
ekf_localization:
  imu0: um7/imu/data
```

Además, se ha implementado la odometría del robot utilizando datos del GPS con un algoritmo de Python que transforma las coordenadas geográficas (latitud y longitud) a metros. Las ecuaciones 1 y 2 muestran las escalas de conversión utilizadas. 

1. $$1° latitud = 111321 [m]$$
2. $$1° longitud = {40075000 * cos(latitud[°]) \over 360 [m]}$$

Es importante destacar que este algoritmo está diseñado exclusivamente para utilizar la base de datos (no se debe reproducir el topico de odometría obtenido en terreno) y toma como entrada las coordenadas provenientes del tópico 'topic_ros'. Para utilizarlo, ejecute el siguiente comando en la ventana de comandos:

```bash
roslaunch warthog_tesis ?.launch
```

### Configuración "manual"

[tf](http://wiki.ros.org/tf)

### Calibración de cámara

[camera_calibration](http://wiki.ros.org/camera_calibration)

### Calibración de cámara y sensor LiDAR

[extender_lidar_camera_calib](https://github.com/AFEICHINA/extended_lidar_camera_calib)


## Algoritmo de mapeo

### OctoMap
que es, instalacon, uso

### Visualización de mapas



