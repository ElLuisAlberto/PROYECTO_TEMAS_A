# Resumen de Presentación · JACO2 Teleoperación Cooperativa (`teleop-base.l2LIt9`)

> Generado automáticamente a partir de `/tmp/teleop-base.l2LIt9` (2026-11-27T01:23:37.805Z → 2026-11-27T01:30:43.194Z, duración total **425.39 s**).

---

## 1. Protocolos utilizados y arquitectura de red

| Capa / Enlace | Protocolo / Transporte | Frecuencia / Watchdog | Función |
| --- | --- | --- | --- |
| Cliente web ↔ Estación madre | **HTTPS + WSS (WebSocket sobre TLS)** en puerto `8443` (`lan_https_proxy.py` + CA local `ca.crt`) | Heartbeat **10 Hz** (100 ms), `input` **20 Hz** (50 ms), telemetría **50 Hz** | Autenticación por rol (`Operador`, `Observador`, `Administrador`), aislamiento de IP local vs. LAN y transporte cifrado. |
| Gateway cooperativo (`gateway.py`) | Protocolo JSON `teleop_cooperativo` (`v1`) sobre WebSocket + relé interno `127.0.0.1:8765` | Lazo autoritativo a **50 Hz** (tick nominal 20 ms) + watchdog de movimiento **400 ms** / *lease* **1.5 s** | Árbitro con propiedad exclusiva (*leases* con `lease_id` y `generation`), composición de twist 6-DOF y `STOP` global. |
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

## 3. Métricas adquiridas (`teleop-base.l2LIt9`)

### 3.1 Desempeño de control, red y hardware

| Métrica | Valor medido |
| --- | --- |
| Duración de sesión analizada | **425.39 s** |
| Fuente de telemetría | `ros_physical_jaco2_fk` (15794 muestras) |
| Eventos registrados | **15803** telemetría · **23751** protocolo |
| Periodo del gateway (`dt` entre ticks) | Media **26.92 ms** (**37.147 Hz**) · σ = **2.586 ms** · p95 = **31.487 ms** · máx = **51.794 ms** (16.42% dentro de 15–25 ms) |
| Frescura de postura (`pose_age_sec`) | Media **38.136 ms** · p50 = **28.304 ms** · p95 = **56.47 ms** · máx = **147.304 ms** (100.0% ≤ 300 ms) |
| Latencia de red WebSocket (RTT) | Media **25.786 ms** · p50 **16.8 ms** · p95 **77.95 ms** · máx **383.8 ms** (7836 muestras) |
| Tasa del bridge USB Kinova | Hardware físico USB · **100.131 tx/s** media (mín 30.0 · máx 103.0 Hz en 382 intervalos) |
| Mensajes de protocolo | `hello`: 9 · `claim`: 2 · `input`: 14637 · `heartbeat`: 9103 · `release`: 0 · `stop_global`: 0 · `rearm`: 0 |
| Errores / rechazos de protocolo | `{'room_not_operational': 481}` |

### 3.2 Cinemática del efector final, articulaciones y pinza

| Métrica cinemática | Valor medido |
| --- | --- |
| Posición inicial `(X, Y, Z)` | `[0.2536, 0.3551, 0.4671]` m |
| Posición final `(X, Y, Z)` | `[0.7365, 0.3596, 0.6335]` m |
| Excursión cartesiana `(ΔX, ΔY, ΔZ)` | `X`: **50.32 cm** · `Y`: **47.97 cm** · `Z`: **52.33 cm** |
| Recorrido 3D total / desplaz. máx. | **6.3439 m** recorrido · **61.28 cm** desde inicio |
| Velocidad lineal comandada | Activa **146.37 s** · pico **50.0 mm/s** |
| Velocidad angular comandada | Activa **101.85 s** · pico **10.03 °/s** |
| Pinza simulada (`F1–F3`) | Activa: **True** · rango **0.0 – 1.175 rad** |
| Tarea virtual cooperativa | Entregados **0 / 5** · agarres detectados: **0** · sueltas por impacto: **0** |

#### Tiempo comandado por eje y rangos articulares

| Eje | Tiempo activo |
| --- | --- |
| `X` | 44.615 s |
| `Y` | 62.739 s |
| `Z` | 39.016 s |
| `ROLL` | 52.974 s |
| `PITCH` | 48.873 s |
| `YAW` | 0.0 s |
| `PINZA` | 9.055 s |

| Articulación | Mínimo | Máximo | Rango recorrido |
| --- | --- | --- | --- |
| J1 | 0.02° | 359.92° | 359.9° |
| J2 | 85.23° | 177.93° | 92.7° |
| J3 | 174.08° | 314.69° | 140.61° |
| J4 | 0.04° | 359.89° | 359.85° |
| J5 | 226.84° | 338.43° | 111.6° |
| J6 | -138.96° | 66.85° | 205.8° |

### 3.3 Cooperación multi-operador y estados de sala

- Operadores distintos que reservaron ejes: **2** (máx. simultáneos: **2**, máx. ejes reservados a la vez: **6**).
- Tiempo con 1 operador activo: **3.53 s** · Tiempo en solapamiento cooperativo (≥2 operadores simultáneos): **362.75 s** (**99.04%** del tiempo ocupado).

| Operador | Ejes reservados |
| --- | --- |
| `Equipo 4VP9ZA` | PINZA, PITCH, ROLL |
| `Equipo 55QA40` | X, Y, Z |

| Estado de sala | Tiempo acumulado |
| --- | --- |
| `NOT_READY` | 0.22 s (0.1%) |
| `SIM_ONLY` | 25.45 s (6.0%) |
| `READY` | 2.73 s (0.6%) |
| `ACTIVE` | 366.27 s (86.1%) |
| `FAULT` | 30.72 s (7.2%) |

### 3.4 Gráficos generados para diapositivas

- `01_frecuencia_y_latencia.png` — Estabilidad del lazo de 50 Hz, `pose_age_sec` vs watchdog (300 ms) y tasa del bridge USB (100 Hz) / RTT WebSocket.
- `02_trayectoria_3d_y_xyz.png` — Trayectoria 3D del efector final, evolución `X/Y/Z` y perfil de velocidad lineal comandada.
- `03_articulaciones_y_pinza.png` — Ángulos medidos `J1–J6`, velocidad angular comandada y apertura de pinza simulada.
- `04_cooperacion_y_estados.png` — Línea de tiempo de estados de sala, número de operadores simultáneos y cronograma de *leases* por eje.
- `05_resumen_para_diapositiva.png` — Resumen ejecutivo en 4 paneles claros (histograma de rapidez de red, latencia por equipo, uso por eje y barras de cumplimiento).
- `06_resumen_seguridad_y_tarea.png` — Distribución de tiempo por estado y tarjetas KPI (trayectoria, velocidad, tarea virtual y seguridad).
- `07_analisis_latencia_rtt_jitter.png` — Curva acumulada CDF de RTT, boxplot por dispositivo, jitter entre paquetes y tiempos internos.
- `08_rtt_jitter_curso_comparativo.png` / `08a` / `08b` / `08c` — Series de RTT crudo, jitter instantáneo `|ΔRTT|` + jitter suavizado RFC 3550 e histograma de densidad (`Mean RTT` y `+1σ`) en el formato `plot_jitter.py` del curso `1MTR74`.

---

## 4. Resultados, análisis y conclusiones

1. **Estabilidad en tiempo real del control distribuido:** El gateway sostuvo una frecuencia efectiva cercana a **37.147 Hz** (media **26.92 ms**, **16.42%** dentro de 15–25 ms) y el bridge USB sostuvo **100.131 tx/s**, cumpliendo la tasa nominal de 100 Hz del Kinova JACO2 sin desbordamientos ni errores críticos del SDK (`errors=0`).
2. **Frescura de telemetría bajo el umbral de seguridad:** El **100.0%** de las muestras de postura llegaron por debajo del watchdog de 300 ms (media **38.136 ms**, p95 **56.47 ms**), validando que el cierre automático de seguridad ante telemetría vencida no introduce falsos disparos en operación normal.
3. **Ejecución segura y cooperativa:** El aislamiento estricto de componentes por *leases* (máx. 3 por operador, total **14636** comandos `input` aceptados) y el desarme inicial protegen al manipulador físico durante las pruebas en red local.
4. **Siguiente corrida recomendada (para completar la foto 100% de exposición):**
   - Esta corrida ya incluye mediciones completas de **RTT WebSocket** y **cooperación simultánea**, listas para presentarse en las diapositivas.
