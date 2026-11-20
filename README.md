# Fase 1 — Teleoperación cartesiana cooperativa simulada

## Propósito

Este directorio contiene el planeamiento, la futura implementación, los
experimentos y la evidencia del proyecto de curso. La fase 1 estudia cómo
varios operadores web controlan cooperativamente la pose cartesiana de un
Jaco2 simulado cuando la red tiene latencia, jitter, pérdida o desconexiones.

Cada operador puede reservar hasta tres de las seis componentes:

~~~
X, Y, Z, roll, pitch, yaw
~~~

La laptop madre fusiona las intenciones y es la única autoridad que envía
comandos al simulador.

## Regla de aislamiento

IMPLEMENTACION_ROS2_CORE es solo referencia técnica. Este proyecto no añadirá,
editará, moverá ni eliminará archivos dentro de esa carpeta, ni dependerá de
modificarla para funcionar. La implementación será autosuficiente dentro de
este directorio.

## Alcance de la fase 1

- Simulación del manipulador y teleoperación cooperativa por red local.
- Página web para operadores, anfitrión y observadores.
- Asignación exclusiva de componentes, con máximo de tres por sesión.
- Fusión de velocidades, límites, parada global y liberación por pérdida de
  conectividad.
- Tarea de posicionamiento cooperativo y, después, pick-and-place con agarre
  virtual.
- Métricas reproducibles de red, coordinación, control y éxito de tarea.

## Fuera de alcance

- Movimiento del Jaco2 físico, salida USB o habilitación de hardware.
- Gripper físico, URDF físico o fabricación mecánica.
- Acceso de navegadores a ROS 2, controladores o articulaciones.
- Control por joints o poses absolutas de varios usuarios.

## Estructura

~~~
docs/          contratos, arquitectura, hoja de ruta y pruebas
experiments/   perfiles de red y hojas de ejecución
evidence/      capturas, videos, logs, figuras y resultados
ros2_ws/       descripción y simulación ROS 2/Gazebo propias
scripts/       construcción, validación y demostraciones reproducibles
simulation/    estado, procedencia y guía del gemelo digital
src/           futura implementación independiente del curso
web/           cliente estático para operadores web
~~~

## Documentos

1. [Alcance y requisitos](docs/01_alcance_y_requisitos.md)
2. [Arquitectura y protocolo](docs/02_arquitectura_y_protocolo.md)
3. [Hoja de ruta modular](docs/03_hoja_de_ruta.md)
4. [Pruebas y métricas](docs/04_pruebas_y_metricas.md)
5. [Bitácora de decisiones](docs/05_bitacora_de_decisiones.md)
6. [Entorno y dependencias](docs/06_entorno_y_dependencias.md)
7. [Gateway y web H3](docs/07_gateway_y_web_h3.md)
8. [Estación base y conexión LAN segura](docs/08_estacion_base_y_lan_segura.md)

Ningún hito se considera terminado sin su prueba, registro de evidencia y
actualización de la bitácora.

## Estado actual

- H1: dominio, leases y composición terminados.
- H2a: árbitro local terminado.
- H2b: arnés cartesiano determinista terminado.
- H2c: sustituto articulado de seis GDL validado en Gazebo Fortress.
- H2d: gemelo visual/cinemático Kinova JACO2 J2N6S300 integrado en Gazebo Sim,
  con control cartesiano, controladores activos y procedencia BSD-3-Clause.
- H3: gateway, interfaz, arnés cartesiano local, telemetría y registro JSONL
  implementados y probados de forma automatizada. Falta el ensayo LAN con dos
  navegadores y su evidencia.
- Puente cartesiano/IK para el sustituto de seis GDL validado en una máquina:
  WebSocket → ROS → IK → Gazebo, parada segura y pérdida del puente. La hoja
  del ensayo con evidencia persistente está en
  `experiments/run_sheets/h3_h2c_single_host_006.md`.
- La telemetría real del Jaco2 en Gazebo (pose por FK, articulaciones, twist,
  fuente y antigüedad) se publica en tiempo real hacia la consola nativa de la
  base y los clientes web.
- La consola nativa Qt/VTK es de solo observación, ocupa la pantalla y
  reconstruye el brazo a partir de la telemetría. La conexión LAN usa TLS;
  la página remota permite elegir un modo sin contraseña. La validación en
  dos equipos físicos sigue pendiente.

## Arranque integrado de la computadora base

La computadora base es la única que ejecuta ROS 2, Gazebo, el puente IK, el
gateway WebSocket y el servidor web. Para iniciar la sesión cooperativa con el
gemelo Jaco2 J2N6S300, desde este directorio se usa un único comando:

~~~bash
bash scripts/start_base.sh
~~~

El iniciador carga ROS 2 e inicia Gazebo sin interfaz, los controladores, el
puente IK, el gateway, el servidor HTTPS y la consola nativa a pantalla
completa. La consola reconstruye la postura del brazo con las articulaciones
observadas; no es video de Gazebo ni muestra toda la escena. La velocidad
mostrada corresponde al comando, no a una medición de velocidad.

La vista utiliza los materiales de las mallas DAE, incluye anillos y dedos en
una postura fija, y muestra una mesa que corresponde al mundo físico Jaco2 de
Gazebo (`jaco2_lab.sdf`). La cámara puede restablecerse desde el visor. El
laboratorio del sustituto H2c conserva su mundo anterior.

La consola recibe el estado público del gateway y no reclama componentes ni
accede directamente a ROS. Al inicio también se detecta e imprime la IP de la
red local. Por ejemplo, si la base indica `192.168.1.10`, los operadores y
observadores de la misma LAN deben abrir en su navegador:

~~~text
https://192.168.1.10:8443
~~~

Los clientes LAN usan exclusivamente el navegador: no necesitan ROS, Gazebo ni
el puente IK. Primero deben confiar en la CA pública de laboratorio cuya ruta
imprime el iniciador. Al abrir la página eligen el modo operador, observador o
administrador y pueden ver si está disponible. Para elegir un modo, pulse su
tarjeta una vez y confírmela con un segundo clic o con «Confirmar modo». El
operador selecciona hasta tres ejes libres antes de ingresar a sus controles;
el servidor puede rechazar la reserva si otro operador ocupó un eje antes.

La página no solicita claves ni autentica la identidad de los usuarios: cualquier
equipo remoto que pueda acceder desde la LAN puede ocupar un modo disponible,
incluido administrador. Use exclusivamente una red de laboratorio confiable y
no exponga el servicio a Internet. HTTPS/WSS cifra el tráfico, pero por sí solo
no identifica ni autoriza a los participantes. La consola madre solo observa;
STOP y rearme corresponden al administrador remoto y no son un paro físico.

El brazo visualizado es el Kinova JACO2 **J2N6S300**: seis articulaciones,
muñeca no esférica y mano de tres dedos. Sus mallas, límites e inercias derivan
de la descripción publicada por Kinova; se registra revisión y licencia en
`ros2_ws/src/course_teleop_description/third_party/kinova_j2n6s300/IMPORT.md`.
Para mantener el gemelo interactivo en tiempo real, Gazebo usa envolventes de
colisión primitivas; la geometría visual sigue siendo la malla oficial.

Para ejecutar la simulación sin interfaz gráfica, por ejemplo en validaciones
automáticas o en una máquina sin pantalla, usar el modo headless opcional del
iniciador:

~~~bash
bash scripts/start_base.sh --headless
~~~

En ese modo no se abre la ventana de Gazebo ni la consola nativa; los clientes
LAN pueden seguir entrando mediante la URL que el iniciador imprime.

## Ejecución local del arnés determinista

Para una prueba de desarrollo aislada, instalar las dependencias indicadas en
`requirements.txt` y cargar el entorno de ROS cuando corresponda:

~~~bash
python3 -m pip install --user -r requirements.txt
~~~

En una terminal, iniciar la autoridad WebSocket:

~~~bash
PYTHONPATH=src python3 -m teleop_cooperativo.gateway --host 127.0.0.1 --local-simulator --insecure-development
~~~

El modo de desarrollo es solo local, sin autenticación y sin conexión ROS;
no debe usarse para la sesión LAN. Usa el arnés determinista, que no representa
el Jaco2 en Gazebo. Para la sesión segura use el arranque integrado.

Para conservar un log JSONL no sobrescribible de una corrida local, añadir:

~~~bash
--event-log evidence/logs/h3-local-001.jsonl --run-id h3-local-001
~~~

al comando del gateway. La telemetría de pose y twist que muestra la web
corresponde al arnés cartesiano determinista, no al modelo Gazebo ni a un
Jaco2 físico.
