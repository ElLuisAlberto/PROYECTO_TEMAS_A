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

ROS 2 no está cargado automáticamente en todas las terminales. Los futuros
scripts de Gazebo deberán cargar /opt/ros/humble/setup.bash de manera explícita
y fallar con un mensaje claro si no existe.

## Política del modelo robótico

No se encontró un modelo Jaco2/Kinova instalado como dependencia del sistema.
Por tanto:

1. El arnés cartesiano determinista de H2b no se denomina gemelo Jaco2.
2. H2c ya incorporó un sustituto propio de seis GDL para validar ROS/Gazebo.
3. La descripción Jaco2 fiel se incorporó bajo BSD-3-Clause en
   `ros2_ws/src/jaco2_description`; no habilita movimiento físico.
4. Su fuente inmediata, aviso, versión observada y modificaciones están en
   `ros2_ws/src/jaco2_description/UPSTREAM.md` y `KINOVA_LICENSE`.
5. No se copia código operativo, interfaces, simulación, hardware ni
   dependencias del CORE de tesis.
