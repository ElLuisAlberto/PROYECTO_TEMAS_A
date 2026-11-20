# Futura implementación

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
  gateway.py           frontera WebSocket, sesiones y estado público de sala
  local_simulation_runtime.py ciclo único árbitro → arnés determinista
  event_log.py         evidencia JSONL de telemetría con reloj de la madre
tests/                 pruebas unitarias sin red, ROS ni Gazebo
~~~

Ejecutar las pruebas desde este directorio:

~~~bash
python3 -m unittest discover -s src/tests -v
~~~

H3 añade un gateway WebSocket, una interfaz estática en `../web` y, de forma
opcional, el arnés cartesiano local con telemetría y log JSONL. El gateway
requiere la dependencia opcional declarada en `../requirements.txt`; el núcleo
anterior sigue sin dependencias de red. Ejecutar H3 se documenta en el README
principal y en `../docs/07_gateway_y_web_h3.md`.

El siguiente avance de integración será un adaptador cartesiano/IK entre el
comando único del árbitro y el sustituto de Gazebo. El Jaco2 fiel se añadirá
después con licencia y procedencia documentadas.

No se copiará ni modificará código de IMPLEMENTACION_ROS2_CORE. Ese proyecto
solo se consulta como referencia de conceptos, pruebas y organización.
