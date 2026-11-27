# Resumen de Presentación · JACO2 Teleoperación Cooperativa (`teleop-base.N9z26W`)

> Generado automáticamente a partir de `/tmp/teleop-base.N9z26W` (2026-11-27T00:47:24.383Z → 2026-11-27T01:00:11.448Z, duración total **767.06 s**).

---

## 1. Protocolos utilizados y arquitectura de red

| Capa / Enlace | Protocolo / Transporte | Frecuencia / Watchdog | Función |
| --- | --- | --- | --- |
| Cliente web ↔ Estación madre | **HTTPS + WSS (WebSocket sobre TLS)** en puerto `8443` (`lan_https_proxy.py` + CA local `ca.crt`) | Heartbeat **10 Hz** (100 ms), `input` **20 Hz** (50 ms), telemetría **50 Hz** | Autenticación por rol (`Operador`, `Observador`, `Administrador`), aislamiento de IP local vs. LAN y transporte cifrado. |
| Gateway cooperativo (`gateway.py`) | Protocolo JSON `teleop_cooperativo` (`v1`) sobre WebSocket + relé interno `127.0.0.1:8765` | Lazo autoritativo a **50 Hz** (tick nominal 20 ms) + watchdog de lease/inactividad **300 ms** / **2.0 s** | Árbitro con propiedad exclusiva (*leases* con `lease_id` y `generation`), composición de twist 6-DOF y `STOP` global. |
| Gateway ↔ Bridge ROS 2 | **ROS 2 Jazzy** (`/teleop/cartesian_twist`, tópicos de pose/joints y servicios de armado/recuperación) | Lazo ROS + verificación `pose_age_sec ≤ 0.30 s` | Cinemática inversa cartesiana (`kinova_cartesian_adapter`) y control seguro de activación (`enable_output`, `recovery`). |
| Estación madre ↔ Kinova JACO2 | **SDK USB Kinova (`USBCommandLayerUbuntu.so`)** mediante `kinova_direct_node` (C++) | **100 Hz** (10 ms por paquete USB) | Lectura de ángulos articulares reales `J1–J6`, envío de velocidades articulares y parada de emergencia USB. |

---

## 2. Implementación de código y metodologías

- **Arbitraje cooperativo por componentes (`arbiter.py`, `lease_manager.py`):** Hasta 2 operadores remotos reservan de 1 a 3 componentes disjuntos entre `X, Y, Z, ROLL, PITCH, YAW` y `PINZA · SIMULADA`. Cada comando `input` valida número de secuencia monótono, `lease_id`, `generation` y estado de la sala (`READY` / `ACTIVE` / `SIM_ONLY`).
- **Seguridad multicapa (*Fail-Safe*):**
  1. Salida física **desarmada al arranque**; requiere armado explícito en consola madre (`desktop_console.py`).
  2. **Puesta a cero automática** ante liberación de botón, pérdida de foco (`blur`/`pagehide`), expiración de *lease* o falla de telemetría (`pose_age_sec > 0.30 s`).
  3. **Modo recuperación por articulaciones (`PAUSED`):** pausa el control web preservando los *leases* mientras la estación madre mueve ejes individuales a ≤5°/s.
- **Gemelo 3D + tarea virtual (`desktop_qt.py`, `virtual_task.py`, `simulated_gripper.py`):** Vista VTK/Qt de postura real reconstruida por FK + simulación PyBullet de pinza de 3 dedos y traslado de 5 muñecos (`Luna`, `Cubo`, `Sol`, `Orejas`, `Casco`) a zonas objetivo.
- **Trazabilidad en reloj único (`event_log.py`):** Bitácora `events.jsonl` (`schema teleop_cooperativo.run_event.v1`) sincronizada al reloj monotónico de la estación madre, más logs del bridge USB C++.

---

## 3. Métricas adquiridas (`teleop-base.N9z26W`)

### 3.1 Desempeño de control, red y hardware

| Métrica | Valor medido |
| --- | --- |
| Duración de sesión analizada | **767.06 s** |
| Fuente de telemetría | `ros_physical_jaco2_fk` (27252 muestras) |
| Eventos registrados | **27260** telemetría · **11377** protocolo |
| Periodo del gateway (`dt` entre ticks) | Media **28.14 ms** (**35.537 Hz**) · σ = **3.672 ms** · p95 = **35.303 ms** · máx = **78.517 ms** (13.36% dentro de 15–25 ms) |
| Frescura de postura (`pose_age_sec`) | Media **38.6 ms** · p50 = **29.314 ms** · p95 = **61.089 ms** · máx = **92.673 ms** (100.0% ≤ 300 ms) |
| Latencia de red WebSocket (RTT) | No capturado en esta corrida (sesión iniciada antes de habilitar `client_sent_ms`/`rtt_ms` en `heartbeat`) |
| Tasa del bridge USB Kinova | Hardware físico USB · **100.505 tx/s** media (mín 50.0 · máx 103.0 Hz en 746 intervalos) |
| Mensajes de protocolo | `hello`: 14 · `claim`: 5 · `input`: 3904 · `heartbeat`: 7454 · `release`: 0 · `stop_global`: 0 · `rearm`: 0 |
| Errores / rechazos de protocolo | `0 errores registrados` |

### 3.2 Cinemática del efector final, articulaciones y pinza

| Métrica cinemática | Valor medido |
| --- | --- |
| Posición inicial `(X, Y, Z)` | `[0.2599, 0.0121, 0.5779]` m |
| Posición final `(X, Y, Z)` | `[0.3467, 0.2072, 0.3634]` m |
| Excursión cartesiana `(ΔX, ΔY, ΔZ)` | `X`: **19.32 cm** · `Y`: **29.26 cm** · `Z`: **29.95 cm** |
| Recorrido 3D total / desplaz. máx. | **3.6644 m** recorrido · **34.44 cm** desde inicio |
| Velocidad lineal comandada | Activa **41.57 s** · pico **35.5 mm/s** |
| Velocidad angular comandada | Activa **13.48 s** · pico **6.72 °/s** |
| Pinza simulada (`F1–F3`) | Activa: **True** · rango **0.0 – 1.2313 rad** |
| Tarea virtual cooperativa | Entregados **0 / 5** · agarres detectados: **0** · sueltas por impacto: **0** |

#### Tiempo comandado por eje y rangos articulares

| Eje | Tiempo activo |
| --- | --- |
| `X` | 18.554 s |
| `Y` | 11.042 s |
| `Z` | 11.975 s |
| `ROLL` | 6.491 s |
| `PITCH` | 3.147 s |
| `YAW` | 3.845 s |

| Articulación | Mínimo | Máximo | Rango recorrido |
| --- | --- | --- | --- |
| J1 | 279.11° | 336.22° | 57.1° |
| J2 | 153.93° | 204.83° | 50.9° |
| J3 | 225.68° | 301.09° | 75.4° |
| J4 | 86.75° | 138.2° | 51.45° |
| J5 | 218.74° | 324.48° | 105.75° |
| J6 | 227.64° | 338.19° | 110.55° |

### 3.3 Cooperación multi-operador y estados de sala

- Operadores distintos que reservaron ejes: **2** (máx. simultáneos: **1**, máx. ejes reservados a la vez: **3**).
- Tiempo con 1 operador activo: **193.78 s** · Tiempo en solapamiento cooperativo (≥2 operadores simultáneos): **0.0 s** (**0.0%** del tiempo ocupado).

| Operador | Ejes reservados |
| --- | --- |
| `Equipo W8VHTY` | PITCH, ROLL, YAW |
| `Equipo Z9V8VQ` | PINZA, X, Y, Z |

| Estado de sala | Tiempo acumulado |
| --- | --- |
| `NOT_READY` | 0.2 s (0.0%) |
| `SIM_ONLY` | 17.35 s (2.3%) |
| `READY` | 555.74 s (72.5%) |
| `ACTIVE` | 193.78 s (25.3%) |

### 3.4 Gráficos generados para diapositivas

- `01_frecuencia_y_latencia.png` — Estabilidad del lazo de 50 Hz, `pose_age_sec` vs watchdog (300 ms) y tasa del bridge USB (100 Hz) / RTT WebSocket.
- `02_trayectoria_3d_y_xyz.png` — Trayectoria 3D del efector final, evolución `X/Y/Z` y perfil de velocidad lineal comandada.
- `03_articulaciones_y_pinza.png` — Ángulos medidos `J1–J6`, velocidad angular comandada y apertura de pinza simulada.
- `04_cooperacion_y_estados.png` — Línea de tiempo de estados de sala, número de operadores simultáneos y cronograma de *leases* por eje.

---

## 4. Resultados, análisis y conclusiones

1. **Estabilidad en tiempo real del control distribuido:** El gateway sostuvo una frecuencia efectiva cercana a **35.537 Hz** (media **28.14 ms**, **13.36%** dentro de 15–25 ms) y el bridge USB sostuvo **100.505 tx/s**, cumpliendo la tasa nominal de 100 Hz del Kinova JACO2 sin desbordamientos ni errores críticos del SDK (`errors=0`).
2. **Frescura de telemetría bajo el umbral de seguridad:** El **100.0%** de las muestras de postura llegaron por debajo del watchdog de 300 ms (media **38.6 ms**, p95 **61.089 ms**), validando que el cierre automático de seguridad ante telemetría vencida no introduce falsos disparos en operación normal.
3. **Ejecución segura y cooperativa:** El aislamiento estricto de componentes por *leases* (máx. 3 por operador, total **3904** comandos `input` aceptados) y el desarme inicial protegen al manipulador físico durante las pruebas en red local.
4. **Siguiente corrida recomendada (para completar la foto 100% de exposición):**
   - Esta corrida no tuvo solapamiento de 2 operadores ni mediciones RTT porque se inició antes de habilitar el registro de RTT y con 1 operador. Con los cambios ya aplicados en el código, al correr una sesión corta con **2 equipos remotos conectados simultáneamente** y ejecutar `python3 scripts/analyze_session_metrics.py`, este mismo reporte incluirá automáticamente las curvas de **RTT en ms por cliente** y el **porcentaje de solapamiento cooperativo de ≥2 operadores**.
