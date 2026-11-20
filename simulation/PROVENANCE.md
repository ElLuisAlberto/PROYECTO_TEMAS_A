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

## Gemelo Jaco2 integrado

- Modelo: Kinova JACO2 `J2N6S300` (6 GDL, muñeca no esférica, mano de tres
  dedos).
- Fuente: `Kinovarobotics/kinova-ros`, rama `noetic-devel`, revisión
  `924781d3dfe241b2b94b3c72a804b80d3658cf02`.
- Licencia: BSD-3-Clause, copyright Kinova Robotics inc. (2017).
- Material y detalle de cambios: 
  `ros2_ws/src/course_teleop_description/third_party/kinova_j2n6s300/IMPORT.md`.

Las mallas visuales, límites e inercias proceden de esa descripción. El wrapper
ROS 2, el control y las colisiones primitivas de tiempo real pertenecen a este
proyecto. No se copiaron archivos desde IMPLEMENTACION_ROS2_CORE.
