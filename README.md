# Fase 1 — Teleoperación cartesiana cooperativa simulada

## Propósito

Este repositorio contiene el planeamiento, la implementación, los experimentos
y la evidencia del proyecto de curso. La fase 1 estudia cómo
varios operadores web controlan cooperativamente la pose cartesiana de un
Jaco2 simulado cuando la red tiene latencia, jitter, pérdida o desconexiones.

Cada operador puede reservar hasta tres de las seis componentes:

~~~
X, Y, Z, roll, pitch, yaw
~~~

La laptop madre fusiona las intenciones y es la única autoridad que envía
comandos al simulador.

## Regla de aislamiento

IMPLEMENTACION_ROS2_CORE no se modifica ni es una dependencia de ejecución.
Esta repo es autosuficiente. El único material incorporado desde un snapshot
auditado es el modelo Kinova JACO2 BSD-3-Clause en `jaco2_description`, con
licencia y procedencia conservadas; el resto del código se desarrolla aquí.

## Alcance de la fase 1

- Simulación del manipulador y teleoperación cooperativa por red local.
- Página web para operadores, anfitrión y observadores.
- Asignación exclusiva de componentes, con máximo de tres por sesión.
- Fusión de velocidades, límites, parada global y liberación por pérdida de
  conectividad.
- Tarea de posicionamiento cooperativo y, después, pick-and-place con agarre
  virtual.
- Métricas reproducibles de red, coordinación, control y éxito de tarea.

## Fuera de alcance

- Movimiento del Jaco2 físico, salida USB o habilitación de hardware.
- Gripper físico, URDF físico o fabricación mecánica.
- Acceso de navegadores a ROS 2, controladores o articulaciones.
- Control por joints o poses absolutas de varios usuarios.

## Estructura

~~~
docs/          contratos, arquitectura, hoja de ruta y pruebas
experiments/   perfiles de red y hojas de ejecución
evidence/      capturas, videos, logs, figuras y resultados
ros2_ws/       descripción y simulación ROS 2/Gazebo propias
scripts/       construcción, validación y demostraciones reproducibles
simulation/    estado, procedencia y guía del gemelo digital
src/           núcleo independiente de coordinación y pruebas
~~~

## Documentos

1. [Alcance y requisitos](docs/01_alcance_y_requisitos.md)
2. [Arquitectura y protocolo](docs/02_arquitectura_y_protocolo.md)
3. [Hoja de ruta modular](docs/03_hoja_de_ruta.md)
4. [Pruebas y métricas](docs/04_pruebas_y_metricas.md)
5. [Bitácora de decisiones](docs/05_bitacora_de_decisiones.md)
6. [Entorno y dependencias](docs/06_entorno_y_dependencias.md)

Ningún hito se considera terminado sin su prueba, registro de evidencia y
actualización de la bitácora.

## Estado actual

- H1: dominio, leases y composición terminados.
- H2a: árbitro local terminado.
- H2b: arnés cartesiano determinista terminado.
- H2c: sustituto articulado de seis GDL validado en Gazebo Fortress.
- H2d: descripción fiel JACO2 J2N6S300 BSD-3-Clause incorporada; falta su
  integración `gz_ros2_control` y el adaptador cartesiano/IK propio.
