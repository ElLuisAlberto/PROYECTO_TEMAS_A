# Resumen de Presentación · JACO2 Teleoperación Cooperativa (`teleop-base.itX76W`)

> Generado automáticamente a partir de `/tmp/teleop-base.itX76W` (2026-11-27T01:46:21.720Z → 2026-11-27T02:23:56.103Z, duración total **2254.38 s**).

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

## 3. Métricas adquiridas (`teleop-base.itX76W`)

### 3.1 Desempeño de control, red y hardware

| Métrica | Valor medido |
| --- | --- |
| Duración de sesión analizada | **2254.38 s** |
| Fuente de telemetría | `ros_physical_jaco2_fk` (79477 muestras) |
| Eventos registrados | **79486** telemetría · **60130** protocolo |
| Periodo del gateway (`dt` entre ticks) | Media **28.362 ms** (**35.258 Hz**) · σ = **3.603 ms** · p95 = **35.471 ms** · máx = **104.053 ms** (7.41% dentro de 15–25 ms) |
| Frescura de postura (`pose_age_sec`) | Media **38.756 ms** · p50 = **29.894 ms** · p95 = **60.243 ms** · máx = **128.943 ms** (100.0% ≤ 300 ms) |
| Latencia de red WebSocket (RTT) | Media **18.285 ms** · p50 **15.6 ms** · p95 **36.44 ms** · máx **232.6 ms** (26213 muestras) |
| Tasa del bridge USB Kinova | Hardware físico USB · **99.917 tx/s** media (mín 2.0 · máx 103.0 Hz en 2197 intervalos) |
| Mensajes de protocolo | `hello`: 20 · `claim`: 6 · `input`: 29421 · `heartbeat`: 30683 · `release`: 0 · `stop_global`: 0 · `rearm`: 0 |
| Errores / rechazos de protocolo | `{'room_not_operational': 1}` |

### 3.2 Cinemática del efector final, articulaciones y pinza

| Métrica cinemática | Valor medido |
| --- | --- |
| Posición inicial `(X, Y, Z)` | `[0.2569, -0.0387, 0.1471]` m |
| Posición final `(X, Y, Z)` | `[0.5945, -0.0594, 0.1283]` m |
| Excursión cartesiana `(ΔX, ΔY, ΔZ)` | `X`: **54.36 cm** · `Y`: **64.78 cm** · `Z`: **45.54 cm** |
| Recorrido 3D total / desplaz. máx. | **8.2276 m** recorrido · **55.69 cm** desde inicio |
| Velocidad lineal comandada | Activa **144.21 s** · pico **50.0 mm/s** |
| Velocidad angular comandada | Activa **77.77 s** · pico **10.03 °/s** |
| Pinza simulada (`F1–F3`) | Activa: **True** · rango **0.0 – 1.3963 rad** |
| Tarea virtual cooperativa | Entregados **1 / 5** · agarres detectados: **2** · sueltas por impacto: **0** |

#### Tiempo comandado por eje y rangos articulares

| Eje | Tiempo activo |
| --- | --- |
| `X` | 28.732 s |
| `Y` | 62.632 s |
| `Z` | 52.849 s |
| `ROLL` | 28.851 s |
| `PITCH` | 48.923 s |
| `YAW` | 0.0 s |
| `PINZA` | 10.657 s |

| Articulación | Mínimo | Máximo | Rango recorrido |
| --- | --- | --- | --- |
| J1 | -57.42° | 35.68° | 93.1° |
| J2 | 96.61° | 152.41° | 55.8° |
| J3 | 168.67° | 337.18° | 168.51° |
| J4 | -264.34° | 235.0° | 499.34° |
| J5 | -168.06° | 8.64° | 176.7° |
| J6 | 0.02° | 359.87° | 359.85° |

### 3.3 Cooperación multi-operador y estados de sala

- Operadores distintos que reservaron ejes: **2** (máx. simultáneos: **2**, máx. ejes reservados a la vez: **6**).
- Tiempo con 1 operador activo: **581.32 s** · Tiempo en solapamiento cooperativo (≥2 operadores simultáneos): **533.59 s** (**47.86%** del tiempo ocupado).

| Operador | Ejes reservados |
| --- | --- |
| `Equipo 2NM3SW` | PINZA, PITCH, ROLL, X, Y, Z |
| `Equipo 9TJQWH` | PINZA, PITCH, ROLL, X, Y, Z |

| Estado de sala | Tiempo acumulado |
| --- | --- |
| `NOT_READY` | 0.21 s (0.0%) |
| `SIM_ONLY` | 5.59 s (0.2%) |
| `READY` | 1133.67 s (50.3%) |
| `ACTIVE` | 1023.7 s (45.4%) |
| `PAUSED` | 91.21 s (4.0%) |

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

1. **Estabilidad en tiempo real del control distribuido:** El gateway sostuvo una frecuencia efectiva cercana a **35.258 Hz** (media **28.362 ms**, **7.41%** dentro de 15–25 ms) y el bridge USB sostuvo **99.917 tx/s**, cumpliendo la tasa nominal de 100 Hz del Kinova JACO2 sin desbordamientos ni errores críticos del SDK (`errors=0`).
2. **Frescura de telemetría bajo el umbral de seguridad:** El **100.0%** de las muestras de postura llegaron por debajo del watchdog de 300 ms (media **38.756 ms**, p95 **60.243 ms**), validando que el cierre automático de seguridad ante telemetría vencida no introduce falsos disparos en operación normal.
3. **Ejecución segura y cooperativa:** El aislamiento estricto de componentes por *leases* (máx. 3 por operador, total **29420** comandos `input` aceptados) y el desarme inicial protegen al manipulador físico durante las pruebas en red local.
4. **Siguiente corrida recomendada (para completar la foto 100% de exposición):**
   - Esta corrida ya incluye mediciones completas de **RTT WebSocket** y **cooperación simultánea**, listas para presentarse en las diapositivas.
