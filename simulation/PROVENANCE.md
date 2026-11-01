# Procedencia de los modelos

## Sustituto articulado H2c

- Nombre: course_six_dof_arm
- Autoría: modelo original creado para este proyecto.
- Geometría: primitivas cilíndricas; no contiene mallas externas.
- Propósito: validar ROS 2, Gazebo, ros2_control y seis articulaciones.
- Limitación: no reproduce geometría, masas, inercias ni límites del Jaco2.

La integración sigue las interfaces públicas documentadas por:

- https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html
- https://github.com/ros-controls/gz_ros2_control/tree/humble
- https://gazebosim.org/docs/fortress/ros2_interop/

## Modelo Jaco2 incorporado para H2d

La descripción BSD-3-Clause J2N6S300 está en
`ros2_ws/src/jaco2_description`. Su aviso de licencia, atribución, subconjunto
de archivos, revisión de la fuente inmediata y transformaciones locales están
registrados en `KINOVA_LICENSE` y `UPSTREAM.md` dentro de ese paquete.

La incorporación no habilita aún el JACO2 fiel en Gazebo: falta su paquete de
integración Fortress con `gz_ros2_control`, y el adaptador cartesiano/IK
propio. No se incorporaron código operativo, interfaces, hardware, percepción
ni dependencias del CORE de tesis.
