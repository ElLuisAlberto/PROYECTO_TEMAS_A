# Pruebas y métricas

## Niveles de prueba

| Nivel | Qué valida | Evidencia mínima |
| --- | --- | --- |
| Unitario | Dominio, leases, secuencias, límites y expiración | Resultado automatizado |
| WebSocket | Sesiones, JSON inválido, cierre y concurrencia | Log de servidor y cliente de prueba |
| ROS/simulación | Comando único, frame, límites y estado del simulador | Log, gráfica o rosbag equivalente |
| Sistema | Operadores humanos, tarea y red no ideal | Hoja de ensayo, CSV y video/captura |

## Matriz de pruebas obligatorias

| ID | Escenario | Resultado esperado |
| --- | --- | --- |
| T01 | Una sesión reclama cuatro ejes | Rechazo atómico; estado sin cambios |
| T02 | Dos sesiones reclaman el mismo eje | Una sola recibe la reserva |
| T03 | Claim de tres ejes con uno ocupado | Ninguno se concede |
| T04 | Secuencia repetida, atrasada o lease antigua | Input descartado sin renovar frescura |
| T05 | NaN, infinito, JSON truncado o valor fuera de rango | Rechazo controlado; no cambia la salida |
| T06 | Dos usuarios controlan ternas distintas | Vector fusionado, limitado y trazable |
| T07 | Usuario conectado sin enviar input | Su componente pasa a cero en timeout de input |
| T08 | Pérdida de heartbeat o cierre abrupto | Cero y liberación de sus componentes |
| T09 | Reconexión después de caída | Sin ejes restaurados automáticamente |
| T10 | STOP GLOBAL durante movimiento | Seis componentes en cero y sala bloqueada |
| T11 | Falla del simulador o puente | Transición a FAULT, sin autoridad residual |
| T12 | Dos operadores alcanzan un objetivo | Éxito o fallo registrado objetivamente |

## Perfiles de red

Los perfiles se aplican entre navegadores y laptop madre, no dentro de la
simulación. La herramienta concreta se seleccionará en H7; todo perfil debe
quedar registrado en la hoja de ensayo.

| ID | Perfil | Parámetros iniciales |
| --- | --- | --- |
| N0 | Base | LAN normal, sin degradación añadida |
| N1 | Moderado | 50 ms de retardo, ±20 ms de jitter |
| N2 | Degradado | 150 ms, ±50 ms, 1–2% de pérdida |
| N3 | Adverso | 300 ms, ±100 ms, 3–5% de pérdida |
| N4 | Corte | Pérdida total o cierre del socket |

## Métricas

### Red y coordinación

- RTT y jitter por cliente.
- Frecuencia de inputs aceptados, rechazados y vencidos.
- Tiempo de solicitud a concesión.
- Tiempo desde pérdida hasta valor cero y hasta componente disponible.
- Tiempo de reasignación y conflictos resueltos.

### Control y seguridad

- Normas lineal/angular publicadas y eventos de saturación.
- Edad del input usado en cada tick.
- Frecuencia de publicación del coordinador.
- Rechazos del simulador o de seguridad.
- Movimiento residual posterior a desconexión.

### Tarea

- Éxito, fallo, abandono, caída, STOP o timeout.
- Duración total y error final de posición/orientación.
- Reasignaciones durante tarea y tasa de éxito por perfil de red.

## Criterios cuantitativos iniciales

- En 100 carreras por un eje: cero casos con dos propietarios.
- En 100 desconexiones: cada componente llega a cero antes de input_timeout más
  un tick, y se libera antes de lease_timeout más un tick.
- En pruebas aleatorias: ninguna salida supera los límites configurados.
- En N0: al menos 90% de éxito después de práctica controlada.
- En N1–N3: se permite degradación, pero nunca ejecución de input vencido ni
  pérdida de trazabilidad.

## Reproducibilidad

Cada corrida registra run_id, fecha, participantes, escenario, perfil de red,
parámetros, hash del código del proyecto, resultado y rutas a logs, videos o
figuras. La laptop madre define el timestamp canónico.
