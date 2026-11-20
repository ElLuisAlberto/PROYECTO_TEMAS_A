# Ensayo H3 — gateway y web en LAN base

Esta hoja prepara la evidencia de aceptación de H3. No constituye un resultado
hasta completar todos sus campos y adjuntar las rutas de evidencia.

## Identificación

- run_id: H3-N0-001
- Fecha y hora:
- Responsables:
- Versión o hash del código del proyecto:
- URL del gateway y puerto:
- Escenario: H3, dos operadores y seis componentes disjuntas
- Perfil de red: N0 (LAN sin degradación añadida)

## Configuración

- Comando usado para iniciar gateway:
- Frecuencia de tick/publicación:
- Timeout de input y lease:
- Navegador/dispositivo de operador A:
- Navegador/dispositivo de operador B:
- Reparto inicial: A = X/Y/Z; B = ROLL/PITCH/YAW
- Estado inicial del árbitro/simulador:

## Pasos y observaciones

| Paso | Resultado esperado | Resultado observado | Evidencia |
| --- | --- | --- | --- |
| A y B completan `hello` | Dos sesiones distintas y estado consistente |  |  |
| A reclama X/Y/Z | Claim completo y visible en ambos clientes |  |  |
| B reclama ROLL/PITCH/YAW | Claim completo y visible en ambos clientes |  |  |
| B reclama X | Denegación, sin cambio de propietarios |  |  |
| A mueve X y deja de enviar input | Telemetría cambia y X vuelve a cero en timeout |  |  |
| A cierra el socket | X/Y/Z liberados; B conserva sus ejes |  |  |
| B libera sus ejes | Sala pasa a READY; seis componentes en cero |  |  |

## Métricas y resultado

- Duración total:
- RTT y jitter resumidos:
- Tiempo de input vencido a cero:
- Tiempo de cierre a liberación:
- Claims aceptados/denegados:
- JSON inválidos, secuencias viejas o tokens inválidos probados:
- Clasificación: éxito / fallo / STOP / timeout / aborto
- Rutas a logs del gateway y clientes:
- Rutas a capturas, video, datos y figuras:

## Observaciones y decisión

- Desviaciones respecto a [contrato H3](../../docs/07_gateway_y_web_h3.md):
- ¿Se repite el ensayo?:
- Cambio propuesto para la bitácora:
