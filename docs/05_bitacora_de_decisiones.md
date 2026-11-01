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
| 2026-10-31 | D-009 | Repo autónoma con descripción JACO2 BSD-3-Clause curada | Centralizar solo material reutilizable con procedencia clara, sin depender del CORE | Se incorporan Xacro y mallas J2N6S300; el bridge, IK, WebSocket y telemetría permanecen propios | `jaco2_description/UPSTREAM.md`, `KINOVA_LICENSE`, `THIRD_PARTY_NOTICES.md` |

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
