# Simulación

## Estado actual: Jaco2 J2N6S300 + puente cartesiano y estación base integrada

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

## Modelos

- H2c se conserva como sustituto de infraestructura y prueba.
- H2d está completado con Kinova JACO2 J2N6S300, seis GDL y mano de tres dedos.
  La descripción importada, fuente y licencia BSD-3-Clause están registradas
  en `ros2_ws/src/course_teleop_description/third_party/kinova_j2n6s300/`.
- El adaptador cartesiano diferencial del Jaco2:
  recibe exclusivamente el `Twist` validado por la laptop madre, calcula un
  Jacobiano geométrico e IK amortiguada y publica trayectorias articulares de
  horizonte corto. Su validación runtime persistente está en
  `scripts/validate_jaco2_cartesian_bridge_runtime.py` verifica movimiento y
  retorno usando sólo su entrada cartesiana pública.

Los mundos, URDF/Xacro, controladores, archivos de lanzamiento y licencias de
ambos modelos permanecerán dentro de simulation o ros2_ws en este repositorio.

## Gemelo Jaco2 en Gazebo

Gazebo muestra el Kinova JACO2 J2N6S300 oficial en lo visual y cinemático. Las
mallas DAE de alta densidad se usan sólo en la visualización; las colisiones se
representan con cilindros y esferas ajustados a las envolventes inerciales para
que la simulación avance en tiempo real.

El launch Jaco2 usa `worlds/jaco2_lab.sdf`: una mesa con tablero y patas de
colisión, cuya superficie está a `world z=0`, bajo la base fija del brazo. El
visor nativo reconstruye la misma mesa y muestra materiales de los DAE; los
dedos son visuales y se muestran fijos. El mundo `teleoperation_lab.sdf` se
conserva para el sustituto H2c. Una prueba aislada de movimiento y comparación
de eslabones con TF está en `experiments/run_sheets/jaco2_lab_visual_001.md`.

### Arranque integrado de la computadora base

Para una sesión cooperativa en LAN, la computadora base concentra todos los
procesos de simulación y control. Desde la raíz del proyecto, iniciar con:

~~~bash
bash scripts/start_base.sh
~~~

El comando inicia Gazebo sin ventana, controladores, puente cartesiano/IK,
gateway y servidor HTTPS/WSS. En la computadora base abre una consola nativa
fullscreen de solo observación. Su visor reconstruye la postura del brazo a
partir de articulaciones observadas; no muestra un video de la escena Gazebo.

El iniciador detecta y muestra la URL LAN, por ejemplo
`https://192.168.1.10:8443`. Cada equipo remoto necesita confiar en la CA
pública indicada al arrancar y elegir un modo disponible, sin clave. Como los
participantes no se autentican, use solo una LAN confiable. Los equipos remotos
no ejecutan ROS 2, Gazebo ni el puente IK, y no publican en tópicos ROS.

El modo sin interfaz gráfica es opcional y está destinado a validación,
automatización o equipos sin sesión gráfica:

~~~bash
bash scripts/start_base.sh --headless
~~~

Con `--headless` no se abre la consola nativa. El gateway y el
servidor web continúan disponibles para los clientes LAN mediante la URL que
se imprime al inicio.

### Inicio y validación manuales

Compilar y validar la descripción:

~~~bash
bash scripts/build_and_check_h2c.sh
~~~

Para iniciar Gazebo manualmente en modo headless:

~~~bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch course_teleop_simulation jaco2_sim.launch.py headless:=true
~~~

En otra terminal, validar reloj, controladores, movimiento y retorno:

~~~bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
PYTHONPATH="src:${PYTHONPATH:-}" python3 scripts/validate_jaco2_cartesian_bridge_runtime.py
~~~

## Ensayo cartesiano manual de una sola máquina (depuración)

Para depurar únicamente Gazebo y el puente, con el gateway detenido, use dos
terminales desde la raíz de `FASE_1_TELEOPERACION_COOPERATIVA`:

~~~bash
# 1. Gazebo y controladores
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch course_teleop_simulation jaco2_sim.launch.py

# 2. Puente IK del Jaco2
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
PYTHONPATH="src:${PYTHONPATH:-}" python3 scripts/jaco2_cartesian_ik_bridge.py

~~~

Para una sesión LAN, cierre estos procesos de depuración e inicie por separado
`bash scripts/start_base.sh`; el iniciador crea sus propios Gazebo y puente.

El puente publica `true` en `/teleop/bridge_ready` al recibir estado fresco de
las seis articulaciones. El gateway espera ese estado hasta 30 segundos al
arrancar y, si deja de recibirlo, pasa a `FAULT`, revoca las leases y publica
cero. La pose por FK, las seis posiciones articulares y su antigüedad se
difunden a la consola nativa y a la web. Con esta ruta no debe iniciarse
`--local-simulator`, porque ese arnés determinista es una simulación distinta
del robot de Gazebo.

~~~bash
source /opt/ros/humble/setup.bash
ros2 topic echo --once /teleop/bridge_ready std_msgs/msg/Bool
~~~

Con Gazebo y el puente activos, y el gateway detenido, ejecutar la comprobación
automatizada de movimiento y parada. Esta prueba publica directamente en ROS:
no debe competir con el gateway como segunda fuente de comandos.

~~~bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ROS_LOG_DIR=/tmp/teleop_ros_logs PYTHONPATH="src:${PYTHONPATH:-}" \
  python3 scripts/validate_jaco2_cartesian_bridge_runtime.py
~~~
