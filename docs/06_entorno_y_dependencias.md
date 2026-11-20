# Entorno y dependencias

## Regla de autosuficiencia

Todo código, configuración, modelo, prueba y documentación propios se guarda
en este repositorio. Las herramientas del sistema no se copian al repositorio:
se declaran aquí para poder reproducir el entorno.

## Entorno detectado

| Componente | Versión o estado |
| --- | --- |
| Python | 3.10.12 |
| ROS 2 | Humble, instalado en /opt/ros/humble |
| Gazebo | Ignition Gazebo Fortress 6.18.0 |
| Integración ROS/Gazebo | ros_gz_sim y ros_gz_bridge disponibles |
| Control | gz_ros2_control, controller_manager y joint trajectory controller disponibles |
| Descripción | xacro y robot_state_publisher disponibles |

ROS 2 no está cargado automáticamente en todas las terminales. El iniciador
integrado `scripts/start_base.sh` debe cargar
`/opt/ros/humble/setup.bash` y el espacio de trabajo `ros2_ws/install/setup.bash`
de manera explícita, y fallar con un mensaje claro si alguno no existe.

## Computadora base y arranque integrado

La sesión cooperativa se inicia en la computadora base mediante un único
comando, ejecutado desde la raíz del proyecto:

~~~bash
bash scripts/start_base.sh
~~~

La base requiere Python 3, PyQt5, VTK, numpy, pycollada, `websockets`, OpenSSL,
ROS 2 Humble, Gazebo, el workspace ya compilado y una sesión gráfica
disponible. El iniciador verifica estos requisitos, detecta la IP LAN y
arranca:

1. Gazebo y los controladores del Kinova JACO2 J2N6S300.
2. El puente cartesiano/IK.
3. El gateway WebSocket local y el servidor HTTPS/WSS en la LAN.
4. Gazebo sin GUI y una consola nativa Qt/VTK fullscreen de solo observación.

La URL LAN mostrada por el iniciador, por ejemplo
`https://192.168.1.10:8443`, es la dirección para los clientes de la misma red.
Cada equipo debe confiar en la CA pública indicada al arrancar; no requiere
ROS, Gazebo ni acceso a tópicos. La página no solicita credenciales y debe
utilizarse únicamente en una LAN confiable: cualquier equipo remoto conectado
puede elegir un modo disponible, incluido administrador.

Para ejecución automatizada o en una base sin pantalla se dispone del modo
headless opcional:

~~~bash
bash scripts/start_base.sh --headless
~~~

Ese modo mantiene Gazebo, el puente, el gateway y el servidor HTTPS sin abrir la
ventana gráfica de Gazebo ni la consola nativa. La URL LAN sigue disponible
para los clientes.

## Gateway y web H3

El gateway requiere `websockets`; la consola requiere PyQt5, VTK, numpy y
pycollada. Se necesita la misma instalación de Python que utiliza ROS 2 Humble.
En Ubuntu 22.04 pueden instalarse PyQt5, VTK y numpy con los paquetes del
sistema `python3-pyqt5`, `python3-vtk9` y `python3-numpy`, y después:

~~~bash
python3 -m pip install --user -r requirements.txt
~~~

El visor usa PyQt5 bajo sus condiciones de licencia GPL/comercial, VTK y
pycollada; revisar compatibilidad de licencias antes de distribuir el proyecto.
La interfaz en `web/` no requiere Node ni compilación. El servidor integrado
usa HTTPS/WSS; el modo de desarrollo sin autenticación queda restringido a
loopback y no admite conexión ROS.

## Puente cartesiano Jaco2

El ensayo monomáquina previo a LAN añade dos procesos ROS 2: el publicador
opcional del gateway (`--ros-twist-topic`) y `scripts/jaco2_cartesian_ik_bridge.py`.
Este último requiere únicamente paquetes incluidos en Humble: `rclpy`,
`geometry_msgs`, `sensor_msgs`, `std_msgs`, `trajectory_msgs` y
`controller_manager_msgs`. Ambos deben
ejecutarse después de cargar `/opt/ros/humble/setup.bash`; el script también
necesita `PYTHONPATH="src:${PYTHONPATH:-}"` para reutilizar la cinemática propia
sin ocultar los paquetes Python de ROS.

El puente usa los seis joints canónicos `j2n6s300_joint_1` a
`j2n6s300_joint_6`, FK/Jacobiano propios y trayectorias breves hacia
`arm_controller`. No añade dependencias ni autoridad ROS al navegador y no usa
el CORE de tesis como componente operativo.

## Política del modelo robótico

El gemelo integrado es Kinova JACO2 J2N6S300. Se derivó de la descripción
publicada por Kinova Robotics, revisión
`924781d3dfe241b2b94b3c72a804b80d3658cf02` de `kinova-ros`, bajo BSD-3-Clause.
La licencia, los archivos derivados y las modificaciones ROS 2 se registran en
`ros2_ws/src/course_teleop_description/third_party/kinova_j2n6s300/`.
No se copiaron ni modificaron archivos de IMPLEMENTACION_ROS2_CORE.
