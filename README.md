# JACO2 · Teleoperación cooperativa

Dos operadores pueden mover un mismo brazo **Kinova JACO2 (J2N6S300)** desde sus navegadores. Se reparten los seis ejes cartesianos y la pinza, y eligen la velocidad de cada componente. La **interfaz madre** conecta el brazo por USB, reconstruye su postura en 3D a partir de las articulaciones medidas y muestra la misma telemetría que recibe la web. También se puede practicar con un brazo simulado.

## Arranque en dos pasos

### 1. Iniciar la interfaz madre

En la computadora conectada al Kinova, abre una terminal en la carpeta que contiene este README y ejecuta lo siguiente. Si es la primera instalación, consulta la preparación al final de la guía.

```bash
bash scripts/start_base.sh --direct-usb --allow-local-web
```

El comando abre la consola y sirve la web por HTTPS. **La salida física comienza desarmada.** Los botones **Armar salida** y **Desarmar salida** están en la consola madre. Para practicar sin conectar el brazo, sustituye `--direct-usb` por `--direct-mock`.

### 2. Abrir la web

Copia la URL que aparece junto a **Interfaz para las otras computadoras** en la terminal de la madre. Pégala en el navegador de cualquier equipo de la misma red local. En Linux también puedes abrirla desde una segunda terminal con este comando.

```bash
read -r -p 'Pega la URL HTTPS de la estación: ' url && xdg-open "$url"
```

La primera vez, instala como autoridad de confianza el certificado público `ca.crt` cuya ruta indica el lanzador. Mantén abierta la terminal del primer paso durante la sesión. Para terminar, pulsa **Cerrar estación de forma segura** en la interfaz madre: el sistema solicita detener la salida y cerrar el SDK USB antes de apagar la web y los procesos ROS 2 de esta estación. La pestaña del navegador mostrará que se desconectó y puede cerrarse. Si no se confirma la parada o el cierre USB, la estación permanece abierta y muestra el error.

## Pinza simulada de tres dedos

En el modo operador, **PINZA · SIMULADA** se reserva como un componente. Sus botones − y + abren y cierran los tres dedos virtuales mientras se mantienen pulsados. La posición se identifica como simulada en la web y en la vista 3D de la madre; el brazo conserva su telemetría física cuando está conectado.

La pinza simulada puede usarse con la salida física desarmada. Su simulación se ejecuta en el gateway y no envía órdenes de dedos al bridge USB. Sus límites y contactos son virtuales: no miden ni mueven los dedos físicos y no representan fuerzas reales. STOP, la pausa de recuperación y la pérdida de la reserva detienen el movimiento simulado.

## Tarea cooperativa con muñecos virtuales

La vista de la madre muestra cinco muñecos virtuales de unos 10 cm y 100 g: Luna, Cubo, Sol, Orejas y Casco. Cada uno tiene un destino del mismo color; los destinos están a distintas alturas. Los operadores pueden repartirse los ejes y reservar **PINZA · SIMULADA** para cerrar o abrir los dedos virtuales. La puntuación aparece en la madre y en la web cuando el muñeco permanece estable en su destino. En la madre puedes reiniciar un muñeco seleccionado o todos con **Reiniciar objetos virtuales**; la web también permite reiniciar cada muñeco.

El juego usa PyBullet (`python3 -m pip install --user -r requirements-game.txt`). Los muñecos tienen gravedad, masa y contactos con la mesa, pedestales y puntas virtuales. El agarre es asistido: al cerrar cerca de un muñeco el juego lo une temporalmente a la pinza y al abrirlo lo libera. La física y las colisiones son aproximadas; no representan contacto real ni frenan al JACO físico. Mantén libre su espacio real y supervisa sus movimientos.

## Recuperación por articulaciones

Con el brazo conectado, telemetría en vivo y la salida armada, pulsa **Activar recuperación** en la madre. Las reservas web se conservan mientras sus operadores sigan conectados y los navegadores permanecen en estado pausado. Abre **Controles de joints** y mantén pulsado **−** o **+** para mover una sola articulación; al soltar, perder el foco o cerrar la ventana se solicita velocidad cero. La velocidad se limita a 5°/s y las articulaciones con topes tienen un margen de protección.

El STOP de administrador, la pérdida de comunicación o de telemetría interrumpen la recuperación y desarman la salida. Al pulsar **Desactivar recuperación**, la madre detiene el movimiento y vuelve al control web; los operadores conectados recuperan sus controles y deben pulsar de nuevo para mover. La recuperación por articulaciones no evita colisiones con el entorno: supervisa el movimiento y usa el STOP cuando corresponda.

## Qué puede hacer cada usuario

| Interfaz | Controles |
| --- | --- |
| **Madre** · escritorio | Visualiza el brazo en 3D, supervisa la telemetría y las reservas, y arma o desarma la salida física. |
| **Operador** · web | Reserva de 1 a 3 componentes disponibles entre X, Y, Z, roll, pitch, yaw y PINZA. Ajusta la velocidad de cada eje y mantiene pulsado **+** o **−** para moverlo. |
| **Observador** · web | Consulta la posición, orientación, articulaciones y reservas de los operadores en tiempo real. |
| **Administrador** · web | Usa **STOP** para detener el movimiento y liberar las reservas. Puede rearmar la sala. Accede con la credencial privada de sesión cuya ruta imprime el lanzador. |

<details>
<summary><strong>Preparar la estación por primera vez</strong></summary>

La madre requiere Linux x86_64 con sesión gráfica, ROS 2 Humble en `/opt/ros/humble`, `colcon`, CMake, un compilador C++17 y OpenSSL. Instala PyQt5, VTK, NumPy, pycollada y `websockets>=12,<16` para el Python 3 de la estación. Los clientes web solo necesitan un navegador y acceso HTTPS a la madre por el puerto `8443`.

El bridge enlaza el **SDK USB de Kinova** tanto en modo físico como en simulación. Ajusta `KINOVA_ROOT` al directorio externo del driver que contiene `include/kinova/` y `lib/x86_64-linux-gnu/`. Desde la raíz del repositorio, compila los dos paquetes que carga el lanzador.

```bash
export KINOVA_ROOT="/ruta/al/kinova_driver"
source /opt/ros/humble/setup.bash
(cd ros2_ws && colcon build --packages-select course_teleop_description)
(cd src/kinova_direct_bridge && colcon --log-base .work/log build \
  --base-paths . --build-base .work/build --install-base .work/install \
  --packages-select kinova_direct_bridge)
```

Conserva `KINOVA_ROOT` en el entorno desde el que arrancas la madre para que el bridge encuentre las bibliotecas del SDK.

</details>

## Dónde está el código

| Componente | Punto de entrada |
| --- | --- |
| Arranque de la estación | [scripts/start_base.sh](scripts/start_base.sh) |
| Interfaz madre y vista 3D | [Consola](src/teleop_cooperativo/desktop_console.py) · [Vista Qt/VTK](src/teleop_cooperativo/desktop_qt.py) |
| Interfaz web | [Página](web/index.html) · [Controles](web/app.js) · [Estilos](web/styles.css) |
| Comunicación con el brazo | [Gateway de teleoperación](src/teleop_cooperativo/gateway.py) · [Bridge USB para ROS 2](src/kinova_direct_bridge/) |

El proyecto se distribuye bajo la [licencia Apache 2.0](LICENSE). El componente USB de Kinova tiene su [propia licencia](licenses/kinova-usb-LICENSE).
