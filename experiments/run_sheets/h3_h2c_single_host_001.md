# Ensayo monomáquina — H3 a sustituto Gazebo H2c

Este ensayo verifica la cadena local completa previa a la prueba LAN. El brazo
es el sustituto propio de seis GDL; no constituye evidencia sobre un Jaco2
fiel ni sustituye el ensayo H3 con dos dispositivos.

## Identificación

- run_id: H3-H2C-SINGLE-001
- Fecha: 2026-11-01
- Escenario: gateway WebSocket, puente IK diferencial y Gazebo en una máquina
- Perfil de red: local (sin LAN ni degradación)
- Versión: árbol de trabajo sin metadatos Git disponibles

## Configuración

- Simulador: Gazebo Fortress, sustituto articulado H2c
- Controladores requeridos: `joint_state_broadcaster` y `arm_controller`, ambos `active`
- Entrada: `/teleop/cartesian_twist`; salida: `/arm_controller/joint_trajectory`
- Frecuencia del puente: 20 Hz; timeout de input: 0.15 s
- Seguridad: el gateway espera `/teleop/bridge_ready`; una pérdida del
  heartbeat lleva la sala a `FAULT` y revoca las leases.

## Resultado observado

| Prueba | Resultado |
| --- | --- |
| Construcción, URDF y configuración H2c | PASS |
| Reloj, controladores, trayectoria y retorno a cero | PASS |
| Puente IK con `bridge_ready=true` | PASS |
| Twist lineal X = 0.02 | Movimiento medido: `joint_2=+0.0118096`, `joint_3=+0.0008520`, `joint_5=-0.0125384` |
| Parada tras Twist cero | Residual máximo `2.36e-13` rad, PASS |
| WebSocket → ROS → IK → Gazebo | 20 inputs aceptados; posición final adicional: `joint_2=0.0320733`, `joint_3=0.00230206`, `joint_5=-0.0340391` |
| Cierre deliberado del puente | Gateway en `FAULT`; nuevo claim rechazado con `room_not_operational`, PASS |

## Evidencia y limitaciones

- Los procesos de Gazebo, puente y gateway se cerraron correctamente al final.
- Las salidas de terminal de esta ejecución no se conservaron como archivos
  independientes. Una repetición para entrega debe guardar stdout/JSONL y una
  captura o video bajo `evidence/` antes de cerrar formalmente el hito.
- Pendiente para H3: ensayo LAN con dos navegadores/dispositivos y sus métricas.
