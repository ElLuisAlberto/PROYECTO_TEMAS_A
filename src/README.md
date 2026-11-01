# Implementación actual

H0 está cerrado. La primera entrega de código, H1, ya contiene un núcleo puro
de dominio con reservas, expiración, validación de secuencias y composición
limitada del vector cartesiano.

~~~text
teleop_cooperativo/
  domain.py            tipos y validación de las seis componentes
  lease_manager.py     claims, leases, input, vencimientos y parada global
  command_composer.py  fusión y normalización de velocidades
  arbiter.py           autoridad única y estados de la sala
  simulation_port.py   frontera abstracta con cualquier simulador
  tool_simulator.py    arnés cartesiano determinista
  simulation_runner.py ejecución segura y transición a FAULT
tests/                 pruebas unitarias sin red, ROS ni Gazebo
~~~

Ejecutar las pruebas desde este directorio:

~~~bash
python3 -m unittest discover -s src/tests -v
~~~

El sustituto articulado propio H2c ya existe en `ros2_ws/`. La descripción
fiel JACO2 licenciada también fue incorporada, pero el siguiente hito es crear
el adaptador cartesiano/IK y la integración ROS/Gazebo propios.

IMPLEMENTACION_ROS2_CORE no se modifica ni se usa como dependencia. La única
excepción de procedencia es el paquete BSD-3-Clause `jaco2_description`, cuyo
aviso y registro de extracción están conservados en su propio directorio.
