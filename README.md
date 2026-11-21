# JACO2 · Teleoperación cooperativa

Dos operadores pueden mover un mismo brazo **Kinova JACO2 (J2N6S300)** desde sus navegadores. Se reparten los seis ejes cartesianos y eligen la velocidad de cada uno. La **interfaz madre** conecta el brazo por USB, reconstruye su postura en 3D a partir de las articulaciones medidas y muestra la misma telemetría que recibe la web. También se puede practicar con un brazo simulado.

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

La primera vez, instala como autoridad de confianza el certificado público `ca.crt` cuya ruta indica el lanzador. Mantén abierta la terminal del primer paso durante la sesión. Pulsa **Ctrl+C** allí para cerrar la estación y sus servicios.

## Qué puede hacer cada usuario

| Interfaz | Controles |
| --- | --- |
| **Madre** · escritorio | Visualiza el brazo en 3D, supervisa la telemetría y las reservas, y arma o desarma la salida física. |
| **Operador** · web | Reserva de 1 a 3 ejes disponibles entre X, Y, Z, roll, pitch y yaw. Ajusta la velocidad de cada eje y mantiene pulsado **+** o **−** para moverlo. |
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
