# Arquitectura y protocolo

## Arquitectura independiente

La implementación futura vive únicamente en este proyecto. El CORE de tesis
puede orientar decisiones de diseño, pero no se modifica ni se usa como
componente operativo obligatorio.

~~~
Operador A ─┐
Operador B ─┼─ WebSocket/LAN ──► gateway de la laptop madre
Operador C ─┘                         ├─ servidor web y sala
                                              ├─ sesiones y leases
Consola nativa local ◄─ WebSocket ──────├─ compositor y telemetría
  (solo observación)                         ├─ registro de eventos
                                              └─ puente ROS 2
                                                        │
                                                        ▼
                                              Gazebo + Jaco2 simulado
~~~

## Módulos

| Módulo | Responsabilidad | No debe hacer |
| --- | --- | --- |
| domain | Tipos, validación y constantes puras | Conocer sockets, ROS o simulador |
| lease_manager | Reclamos, máximo de tres, expiración y liberación | Calcular cinemática |
| command_composer | Fusionar seis valores y limitar normas | Decidir propiedad de ejes |
| gateway | WebSocket, sala y estado enviado a la web | Mandar órdenes directas al simulador |
| ros_bridge | Publicar el único comando del curso hacia el simulador | Aceptar clientes web directamente |
| telemetry | Persistir eventos y métricas por corrida | Participar en decisiones de control |
| desktop_console | Mostrar estado y telemetría en la base | Reclamar ejes o acceder directamente a ROS |

Los primeros tres módulos son Python puro y se prueban sin ROS, Gazebo ni
navegador. La consola nativa también separa su reductor de estado de Qt/VTK,
por lo que sus mensajes se validan sin requerir una pantalla.

## Convención cartesiana

| Slot web | Variable interna | Significado |
| --- | --- | --- |
| X | vx | Velocidad lineal en X de world |
| Y | vy | Velocidad lineal en Y de world |
| Z | vz | Velocidad lineal en Z de world |
| roll | wx | Velocidad angular alrededor de X de world |
| pitch | wy | Velocidad angular alrededor de Y de world |
| yaw | wz | Velocidad angular alrededor de Z de world |

El navegador transmite valores normalizados entre -1 y 1. La laptop madre los
convierte a velocidades limitadas. Después de fusionar normaliza por separado
las ternas lineal y angular para garantizar los límites globales.

## Estados

Estado global de la sala:

~~~
NOT_READY → READY → ACTIVE → FAULT
~~~

- NOT_READY: simulador o puente no disponibles; no se admiten reclamos.
- READY: la sala puede aceptar operadores y reclamos.
- ACTIVE: existe al menos una reserva válida.
- FAULT: salida cero, reservas revocadas y bloqueo hasta rearme del host.

Una componente sigue este ciclo:

~~~
DISPONIBLE → ASIGNADA → ACTIVA → INPUT_VENCIDO → LIBERADA → DISPONIBLE
~~~

## Leases y validación

Cada componente conserva:

~~~
axis, session_id, lease_id, generation,
last_input_monotonic, last_heartbeat_monotonic, value
~~~

La generación evita que un paquete demorado de un dueño anterior modifique una
componente reasignada. El servidor usa su propio reloj monotónico para validar
expiraciones; la hora del navegador se conserva solo como métrica.

Un input se acepta solo si la sesión está activa, su secuencia es estrictamente
creciente, lease y generación corresponden al dueño actual y cada valor es
finito y pertenece a [-1, 1].

## Mensajes WebSocket iniciales

| Dirección | Tipo | Finalidad |
| --- | --- | --- |
| Cliente → madre | hello | Crear una sesión de sala |
| Cliente → madre | claim | Solicitar una lista atómica de ejes libres |
| Cliente → madre | input | Enviar snapshot de valores y secuencia |
| Cliente → madre | heartbeat | Mantener viva la sesión sin movimiento |
| Cliente → madre | release | Soltar una o varias componentes |
| Madre → cliente | welcome, room_state | Estado de sesión y propietarios |
| Madre → cliente | claim_granted, release_confirmed | Resultado de una reserva o liberación propia |
| Madre → cliente | input_accepted, heartbeat_ack, error | Confirmaciones y rechazos controlados |

El contrato original de H3 está en `docs/07_gateway_y_web_h3.md`. La versión
integrada añade selección de rol, STOP/rearme administrativo remoto y
telemetría observacional ROS/Gazebo mediante el puente cartesiano/IK. El
procedimiento actualizado está en `docs/08_estacion_base_y_lan_segura.md`.
El acceso LAN sin clave no autentica participantes; requiere una red confiable.

## Desconexión y parada

~~~
input vencido      → valor de esa componente = 0
heartbeat vencido  → valor = 0 y lease liberada
socket cerrado     → valor = 0 y lease liberada inmediatamente
release explícito  → valor = 0 antes de liberar
STOP GLOBAL        → seis valores = 0 y todas las leases revocadas
~~~

La reconexión nunca restaura ejes automáticamente: el operador debe reclamarlos
de nuevo. Esto hace medible la pérdida de señal y evita autoridad residual.
