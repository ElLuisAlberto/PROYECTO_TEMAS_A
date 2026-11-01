# Simulación

## Estado actual: H2c completado; H2d en integración

El proyecto dispone de un arnés cartesiano determinista. Este arnés valida la
frontera entre el árbitro y un simulador, pero no representa todavía la
geometría ni la dinámica del Jaco2.

El arnés:

- integra velocidades lineales en el marco world;
- integra velocidades angulares mediante cuaterniones;
- mantiene una región de trabajo explícita;
- rechaza pasos temporales excesivos y valores no finitos;
- conserva la pose anterior si un comando es inválido;
- fuerza FAULT y revoca leases si el simulador deja de estar disponible.

Ejecutar la demostración:

~~~bash
python3 scripts/run_deterministic_demo.py
~~~

## Modelos disponibles y próximos pasos

- H2c: sustituto articulado propio de seis GDL en Gazebo Fortress, validado
  para reloj, controladores, movimiento articular y retorno.
- H2d: la descripción fiel JACO2 J2N6S300 ya está en
  `ros2_ws/src/jaco2_description`, con licencia y procedencia registradas.
  Falta crear su integración propia con `gz_ros2_control` y el adaptador
  cartesiano/IK; todavía no se declara simulación operativa del JACO2 fiel.

Los mundos, URDF/Xacro, controladores, archivos de lanzamiento y licencias de
ambos modelos permanecerán dentro de simulation o ros2_ws en este repositorio.

## H2c — Sustituto articulado en Gazebo

Compilar y validar la descripción:

~~~bash
bash scripts/build_and_check_h2c.sh
~~~

Iniciar Gazebo headless:

~~~bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch course_teleop_simulation course_arm_sim.launch.py
~~~

En otra terminal, validar reloj, controladores, movimiento y retorno:

~~~bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
python3 scripts/validate_gazebo_runtime.py
~~~
