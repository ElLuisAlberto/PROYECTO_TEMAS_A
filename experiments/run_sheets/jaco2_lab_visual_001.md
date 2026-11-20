# Jaco2 — validación aislada del laboratorio y visor 001

- Fecha: 2 de noviembre de 2026.
- Modelo: J2N6S300 simulado; mundo `jaco2_lab.sdf` con mesa física.
- Aislamiento: `ROS_DOMAIN_ID=73`, `ROS_LOCALHOST_ONLY=1` y
  `IGN_PARTITION=teleop_jaco2_lab_validation`; la sesión existente del usuario
  continuó en funcionamiento. No se conectó hardware físico.
- El validador del puente publicó directamente en ROS **solo en la instancia
  aislada**, sin gateway que compitiese por el tópico de comandos.
- Movimiento cartesiano corto y retorno: PASS; desplazamiento máximo medido
  de una articulación 0,013014 rad, error máximo al retorno 0,0000053 rad.
- Visor frente a TF: 427 muestras sincronizadas de base y seis eslabones,
  durante el ensayo; diferencia máxima de traslación 2,52 × 10⁻¹² m y de
  rotación 3,66 × 10⁻⁸ rad, dentro de los límites del script. Esta comparación
  comprueba transformaciones, no apariencia física ni todas las trayectorias.
- Herramientas: `scripts/validate_jaco2_cartesian_bridge_runtime.py` y
  `scripts/validate_jaco2_view_tf.py`; este último solo observa ROS/TF.
- Comprobación adicional de extremo a extremo, en una instancia aislada
  `ROS_DOMAIN_ID=74` y puertos temporales: navegador automatizado local →
  HTTPS/WSS → gateway → IK → Gazebo → telemetría web. Tras confirmar el modo,
  reservar X y mantener pulsado el botón durante 0,6 s, la posición mostrada
  cambió de 0,596 a 0,612 m. El navegador automatizado omitió la validación
  del certificado para esta prueba aislada; no demuestra que un navegador
  remoto confíe en la CA. El código temporal del ensayo está en
  `/tmp/teleop-jaco2-e2e-check.py` y no forma parte de la instalación.
- Modelo y mesa: geometría del visor contrastada automáticamente con
  `jaco2_lab.sdf`; materiales de las mallas DAE contrastados con los archivos
  importados. Los dedos se representan fijos, sin telemetría propia.
- Alcance pendiente: ensayo LAN con dos computadoras físicas, validación de más
  trayectorias y medición de rendimiento integral de simulación e interfaces.

El registro normal de ROS de esta ejecución se creó bajo
`~/.ros/log/2026-11-02-09-11-43-052467-miNave-66995`; puede ser temporal según
la política de limpieza de ROS. Las métricas anteriores son el resumen de las
salidas observadas durante la prueba, no un registro crudo persistido de TF.
