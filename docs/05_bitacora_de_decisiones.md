# Bitácora de decisiones

Esta bitácora evita que decisiones importantes queden solo en conversaciones.
Se añade una entrada por cambio de alcance, arquitectura, parámetros o resultado
experimental relevante.

| Fecha | ID | Decisión | Motivo | Impacto | Evidencia / responsable |
| --- | --- | --- | --- | --- | --- |
| 2026-10-31 | D-001 | Fase 1 exclusivamente simulada | El gripper físico y la validación física están fuera del alcance actual | Reduce riesgo y permite validar coordinación/red primero | Planeamiento inicial |
| 2026-10-31 | D-002 | Asignación por seis componentes cartesianas, no por joints | El proyecto se centra en la pose del efector final | Define UI, protocolo y métricas | Planeamiento inicial |
| 2026-10-31 | D-003 | Máximo tres componentes por operador | Requisito indicado para el curso | Impone límite de leases | Planeamiento inicial |
| 2026-10-31 | D-004 | CORE de tesis en modo solo referencia | Se preserva por completo el trabajo de tesis | El proyecto de curso será independiente | Solicitud del propietario |
| 2026-10-31 | D-005 | H1 inicia con un núcleo Python puro | Permite validar exclusividad y vencimientos antes de añadir red, ROS o simulación | Las reglas críticas se prueban sin dependencias externas | 10 pruebas unitarias aprobadas |
| 2026-10-31 | D-006 | H2a implementa un árbitro local sin ROS | Se separa la autoridad de sala de la futura integración con el gemelo digital | Mantiene control verificable y evita depender del CORE | 15 pruebas unitarias aprobadas |
| 2026-10-31 | D-007 | H2b usa primero un arnés determinista y no un falso Jaco2 | No existe un modelo Kinova instalado y no se copiará el CORE | Permite validar tiempo, pose, workspace y fallos antes de Gazebo | 25 pruebas unitarias aprobadas |
| 2026-10-31 | D-008 | H2c usa un sustituto propio de seis GDL en Gazebo | Se necesita validar infraestructura sin atribuir fidelidad al Jaco2 | ROS 2, Gazebo, controladores, movimiento y retorno quedan probados | Validación estática y runtime aprobada |
| 2026-10-31 | D-009 | H3 separa el gateway WebSocket del núcleo y de la interfaz estática | Las reglas de autoridad deben seguir siendo comprobables sin navegador, ROS ni Gazebo | Un único árbitro en la laptop madre atiende una sala; `websockets` es dependencia opcional y la web nunca recibe acceso ROS | 30 pruebas automatizadas y prueba local de transporte WebSocket aprobadas; falta ensayo LAN humano |
| 2026-10-31 | D-010 | Antes de LAN, H3 integra el gateway con el arnés cartesiano determinista | Permite validar de extremo a extremo autoridad, vencimientos y observabilidad sin atribuir fidelidad al Jaco2 | Un ciclo único avanza árbitro y arnés; pose/twist se observan en web y pueden registrarse en JSONL bajo solicitud explícita | 38 pruebas automatizadas y prueba WebSocket local con desplazamiento X aprobadas |

| 2026-11-01 | D-011 | El sustituto H2c usa IK diferencial amortiguada como puente cartesiano previo a LAN | La teleoperación continua requiere un único `Twist` hacia el controlador de posición existente, sin exponer ROS al navegador | Jacobiano y límites se prueban sin ROS; el gateway solo arranca con heartbeat fresco del puente y entra en `FAULT` si se pierde | 46 pruebas unitarias y H3-H2C-SINGLE-001 aprobados; falta conservar evidencia cruda para la entrega |
| 2026-11-01 | D-012 | La disponibilidad del puente exige estado articular válido y ambos controladores ROS activos | Un `joint_state` fresco no garantiza que el controlador acepte trayectorias | El gateway no se arma sin heartbeat del puente y falla seguro ante su pérdida; entradas no finitas desarman el puente | H3-H2C-SINGLE-006: cadena WebSocket→ROS→IK→Gazebo y pérdida del puente aprobadas |
| 2026-11-01 | D-013 | La evidencia H3 en ruta ROS registra protocolo y telemetría FK observada de Gazebo | La telemetría del arnés local no representa el estado del sustituto y los ensayos necesitan trazabilidad persistente | `--event-log` funciona con ROS; se registran outcomes sin tokens, comando, joints, pose FK y edad de muestra; el analizador produce resúmenes JSON | H3-H2C-SINGLE-006: 402 records y pérdida de puente documentada |

## Plantilla de nueva entrada

~~~
Fecha:
ID:
Decisión:
Alternativas consideradas:
Motivo:
Impacto en alcance, código, pruebas o evidencia:
Evidencia / responsable:
~~~
