# Ensayo monomáquina persistente — H3 a sustituto Gazebo H2c

## Identificación

- run_id: H3-H2C-SINGLE-006
- Fecha: 2026-11-01
- Escenario: dos clientes WebSocket locales, gateway, ROS 2, IK diferencial y Gazebo Fortress
- Perfil de red: local; no sustituye el ensayo LAN con dispositivos separados
- Modelo: sustituto propio H2c de seis GDL, no un Jaco2 fiel

## Configuración y seguridad

- Gateway: único árbitro; publica `/teleop/cartesian_twist`.
- Puente: 20 Hz, `input_timeout_sec=0.15`; exige joints válidos y los
  controladores `joint_state_broadcaster` y `arm_controller` activos.
- Telemetría en JSONL: pose por FK de joints observados, etiquetada
  `ros_gazebo_h2c_fk`; no equivale aún a telemetría mostrada en la web.
- Prueba de pérdida: se detuvo el puente para comprobar `FAULT` y revocación.

## Resultado

| Caso | Resultado |
| --- | --- |
| Dos clientes reclaman ternas disjuntas | PASS |
| Claim conflictivo sobre X | PASS, `claim_denied` |
| Input X aceptado y movimiento ROS/Gazebo | PASS |
| Vencimiento de input a comando cero | PASS; transición registrada |
| Cierre del cliente A | PASS; libera solo sus ejes |
| Liberación final del cliente B | PASS; sala `READY` |
| Pérdida deliberada del puente | PASS; sala `FAULT` y claim `room_not_operational` |
| Cierre de procesos | PASS; gateway sin `ERROR` |

## Evidencia persistente

- [JSONL del gateway](../../evidence/logs/h3-h2c-single-006-gateway.jsonl)
- [Resumen completo](../../evidence/logs/h3-h2c-single-006-full-summary.json): 402 registros, 10 de protocolo y 392 de telemetría; máximo lineal 0.05 m/s.
- [Auditoría de dos clientes](../../evidence/logs/h3-h2c-single-006-clients.json): PASS, sin IDs de sesión ni lease sin redactar.
- [Prueba de pérdida del puente](../../evidence/logs/h3-h2c-single-006-bridge-loss.json)
- Salidas de Gazebo, puente y gateway: `evidence/logs/h3-h2c-single-006-*.log`.

## Decisión

El ensayo monomáquina de la cadena de control queda respaldado. Continúa
pendiente el ensayo H3 en LAN con dos navegadores/dispositivos físicos, así
como la telemetría Gazebo visible en la interfaz web.
