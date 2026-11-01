# Hoja de ruta modular

Cada jornada puede ocupar más o menos de un día según el avance real. No se
inicia un hito posterior hasta que el anterior tenga evidencia verificable.

## H0 — Contrato congelado

Producto: documentos de alcance, arquitectura, protocolo y pruebas.

Salida: el equipo puede explicar qué ocurre cuando un operador reclama,
controla, se desconecta o vuelve.

## H1 — Núcleo de dominio sin red ni ROS

Producto: módulo puro de reservas y composición de comandos.

Trabajo: modelar sesiones, leases, secuencias, expiración, límites y parada
global con un reloj inyectable para pruebas.

Salida: pruebas unitarias cubren exclusividad, máximo de tres, claims atómicos,
paquetes viejos, desconexión y normalización vectorial.

## H2 — Árbitro local y preparación de simulación

Producto: árbitro local que entrega un único comando cartesiano. La simulación
propia se añade en el siguiente subhito.

Salida H2a: el árbitro mantiene los estados de sala, fusiona entradas válidas,
emite cero seguro ante expiración y revoca autoridad ante un fallo.

Salida H2b: el arnés cartesiano determinista integra un único comando, se
detiene al vencer el input y revoca autoridad ante fallos. No se presenta como
gemelo Jaco2.

Salida H2c: un sustituto articulado propio de seis GDL arranca en Gazebo,
publica su estado y acepta comandos articulares pequeños.

Estado: completado. El modelo valida seis articulaciones, controladores activos,
reloj simulado, movimiento pequeño y retorno a la pose inicial.

Salida H2d: el sustituto se reemplaza por un Jaco2 fiel obtenido de una fuente
permitida, con licencia y procedencia registradas.

## H3 — Gateway y web mínima

Producto futuro: servicio WebSocket y página con sala, seis controles,
reclamar/liberar y estado visible.

Salida: dos navegadores en LAN reclaman ejes sin conflictos ni autoridad
duplicada.

## H4 — Cooperación estable

Producto futuro: demo de dos usuarios; A controla X/Y/Z y B controla
roll/pitch/yaw.

Salida: ambos operadores alcanzan una pose objetivo simulada de forma
repetible.

## H5 — Pérdida y reasignación

Producto futuro: manejo completo de input vencido, heartbeat vencido, cierre
de navegador, reconexión y parada global.

Salida: las métricas pérdida a cero y pérdida a disponible cumplen los
timeouts configurados en todas las repeticiones.

## H6 — Escena y tarea cooperativa

Producto futuro: escena con marcador y, después, cubo con agarre virtual.

Salida: intentos clasificados como éxito, caída, STOP, timeout o aborto.

## H7 — Red no ideal y campaña experimental

Producto: perfiles de red, corridas repetibles y resultados.

Salida: tablas y gráficas muestran desempeño, degradación y límites de la
arquitectura.

## H8 — Entrega académica de fase 1

Producto: demo ensayada, video de respaldo, presentación, resultados y
limitaciones.

Salida: exposición con protocolos, código propio, métricas, análisis y ruta
explícita hacia una fase 2.

## Fase 2, fuera de este alcance

Diseño URDF, gripper simulado avanzado, fabricación y ensayo de gripper físico,
y validación física separada. Ninguna tarea de fase 2 se adelanta mientras
H0–H8 no estén cerrados.
