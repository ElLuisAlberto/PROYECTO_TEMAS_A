# Alcance y requisitos de la fase 1

## Objetivo verificable

Demostrar que dos o más operadores controlan cooperativamente las seis
componentes de velocidad cartesiana de un Jaco2 simulado. Cada computadora
reserva hasta tres componentes y, si pierde conectividad, el sistema pone sus
componentes en cero y las libera para otro participante.

> ¿La asignación exclusiva de componentes cartesianas y la liberación por
> pérdida de conectividad permiten una teleoperación cooperativa trazable bajo
> condiciones de red no ideales?

## Modelo de control

La interfaz muestra X, Y, Z, roll, pitch y yaw. La implementación los
representa como velocidades:

~~~
u = [vx, vy, vz, wx, wy, wz]
~~~

vx, vy y vz son velocidades lineales. wx, wy y wz son velocidades angulares.
Aunque la interfaz use roll, pitch y yaw, no se combinan ángulos Euler
absolutos. Todas las componentes se expresan respecto del marco fijo world.

## Requisitos funcionales

| ID | Requisito |
| --- | --- |
| RF-01 | El anfitrión crea una sala y los operadores se conectan desde un navegador. |
| RF-02 | Un operador puede solicitar una, dos o tres componentes libres. |
| RF-03 | La solicitud de varios componentes es atómica: se concede todo o nada. |
| RF-04 | Una componente solo puede tener un propietario activo. |
| RF-05 | La laptop madre fusiona entradas válidas en un único vector cartesiano. |
| RF-06 | Soltar un control pone esa componente en cero sin perder necesariamente la reserva. |
| RF-07 | Desconexión, cierre de navegador o vencimiento de heartbeat libera los ejes del usuario. |
| RF-08 | Una parada global pone las seis componentes en cero y revoca las reservas. |
| RF-09 | El sistema muestra propietarios, conexión y estado de cada componente. |
| RF-10 | Cada corrida genera registros suficientes para reconstruir eventos y resultados. |

## Requisitos no funcionales

| ID | Requisito |
| --- | --- |
| RNF-01 | La laptop madre es la única autoridad de control del simulador. |
| RNF-02 | Los navegadores envían intenciones normalizadas, nunca joints, poses o trayectorias. |
| RNF-03 | Entradas inválidas, repetidas, vencidas o fuera de rango no cambian el comando activo. |
| RNF-04 | La salida lineal y angular respeta límites globales tras fusionar usuarios. |
| RNF-05 | La fase 1 se ejecuta exclusivamente en simulación. |
| RNF-06 | La implementación y los datos permanecen aquí, sin modificar el CORE de tesis. |

## Parámetros iniciales para calibrar

| Parámetro | Valor inicial |
| --- | ---: |
| Publicación del coordinador | 20 Hz |
| Input web | 20–30 Hz |
| Máximo de ejes por sesión | 3 |
| Timeout de input | 150 ms |
| Timeout de lease/heartbeat | 250 ms |
| Velocidad lineal resultante | 0.05 m/s |
| Velocidad angular resultante | 0.175 rad/s (10°/s) |

Estos son valores iniciales: se revisarán después de medir la red y la
respuesta de la simulación.

## Definición de éxito

- Dos operadores controlan componentes distintas de forma simultánea.
- No existe más de un propietario por componente ni más de tres por operador.
- Una pérdida de conexión produce cero y liberación en el tiempo configurado.
- Se completa una tarea de posicionamiento cooperativo repetible en LAN normal.
- Se ejecutan y documentan ensayos con degradación de red.
- Cada resultado tiene logs, parámetros, evidencia visual y conclusiones.
