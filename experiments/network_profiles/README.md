# Perfiles de red

Los perfiles se aplicarán entre cada navegador y la laptop madre. Deben
registrarse en la hoja de ensayo y no se modifican durante una misma serie de
corridas.

| Perfil | Retardo | Jitter | Pérdida | Propósito |
| --- | ---: | ---: | ---: | --- |
| N0 | natural de LAN | — | 0% | Línea base |
| N1 | 50 ms | ±20 ms | 0% | Red remota aceptable |
| N2 | 150 ms | ±50 ms | 1–2% | Degradación moderada |
| N3 | 300 ms | ±100 ms | 3–5% | Condición adversa |
| N4 | corte total | — | 100% | Pérdida de usuario |

La herramienta de inyección se elegirá al iniciar H7. Antes de aplicarla se
registrará el comando o la configuración exacta usada.
