# Estación base y acceso LAN

La estación solo ejecuta la simulación Jaco2. La ventana nativa es de
observación; los movimientos se envían desde otras computadoras por la web.
El visor reconstruye el brazo a partir de sus articulaciones y no es un video
de la escena completa de Gazebo. La velocidad mostrada es la comandada.
La mesa coincide con el mundo Jaco2 de Gazebo; los dedos se muestran en una
postura fija porque no se recibe su estado articulado. El botón «Restablecer
vista» devuelve la cámara a su posición inicial.

## Navegar por la escena de la madre

- Mantén pulsado el **botón izquierdo** y arrastra sobre el brazo o la mesa
  para girar la cámara alrededor del punto de la superficie donde empezaste.
  Si empiezas sobre el fondo, se usa el centro actual de la vista. El giro es
  intencionalmente lento y no se combina con el control Trackball por defecto
  de VTK.
- Mantén pulsado el **botón central** del ratón y arrastra para desplazar la
  vista sin girarla.
- Como alternativa para girar, mantén pulsada **Mayús (Shift)** y arrastra con
  el botón central.
- Gira la **rueda** para acercar o alejar hacia la posición del cursor. El zoom
  tiene límites para evitar perder la escena.
- Pulsa **Restablecer vista** para recuperar el encuadre inicial.

Estas acciones solo mueven la cámara de la representación; no envían órdenes
al brazo ni modifican el simulador.

## Preparación de la madre

Instalar y compilar ROS 2 Humble, Gazebo Fortress y el workspace como se indica
en [dependencias](06_entorno_y_dependencias.md). En Ubuntu 22.04, el Python del
sistema necesita los paquetes `python3-pyqt5`, `python3-vtk9`, `python3-numpy`
y `python3-pip`. Después, desde la raíz del proyecto:

```bash
python3 -m pip install --user -r requirements.txt
bash scripts/start_base.sh
```

El iniciador abre la consola nativa fullscreen y muestra la URL HTTPS y la ruta
del certificado público de la CA. F11 o Esc alternan el modo fullscreen; cerrar la
consola finaliza la sesión integrada. Ctrl+C en la terminal detiene los procesos
iniciados por el script. Para ejecutar sin abrir la consola, añadir `--headless`.

### Prueba web desde la misma madre

Para probar la web y controlar el brazo simulado desde esta misma computadora,
detenga primero cualquier ejecución anterior e inicie:

```bash
bash scripts/start_base.sh --allow-local-web
```

Abra en el navegador la misma dirección HTTPS con IP LAN que imprime el
iniciador, por ejemplo `https://192.168.1.10:8443`. La CA pública indicada al
iniciar debe ser confiable también en el navegador de la madre. El modo de
prueba admite los mismos roles y reservas que un cliente remoto; la ventana
nativa permanece de solo observación. Para volver al comportamiento normal,
detenga el sistema y ejecute el script sin `--allow-local-web`.

## Conectar equipos remotos

1. Conectar las computadoras a una LAN de laboratorio confiable. No publicar
   el servicio en Internet.
2. Distribuir únicamente el archivo público `ca.crt` indicado al inicio e
   instalarlo como autoridad confiable en el sistema o navegador de cada
   equipo. El procedimiento depende del sistema y del navegador. Verificar
   que la identidad de la CA recibida sea la esperada por un canal confiable.
   No distribuir nunca `ca.key` ni `server.key`.
3. En cada computadora remota abrir la URL exacta impresa. La página muestra
   los modos operador, observador y administrador, junto con su disponibilidad.
   Ya no es necesario escribir sala, alias ni clave. El certificado corresponde
   a la IP detectada; si esta cambia, usar la nueva URL indicada por un nuevo
   arranque y confiar en la misma CA si continúa siendo válida.
4. Elegir un modo con un clic para ver su disponibilidad y confirmarlo con un
   segundo clic en la misma tarjeta o con «Confirmar modo». El operador selecciona entre uno y tres ejes
   libres y confirma su ingreso; el servidor concede la reserva solo si todos
   siguen disponibles. Si otro operador acaba de ocupar un eje, la reserva se
   rechaza y se debe elegir nuevamente. La vista de control muestra los ejes
   concedidos y sus botones momentáneos. Observador muestra el estado sin mover.
5. STOP y rearme están disponibles en el modo administrador remoto;
   STOP revoca reservas y rearme exige puente y telemetría disponibles. Es una
   parada lógica de la simulación, no un paro de emergencia físico.

El gateway sigue usando identificadores internos para mantener la exclusividad
de los ejes y liberar reservas al desconectarse; no se presentan como un inicio
de sesión ni requieren datos de acceso. Los registros y la configuración interna
quedan en el directorio temporal privado indicado al terminar la ejecución.

## Alcance y límites

HTTPS/WSS verifica la identidad del servidor y cifra el transporte. El acceso
web sin clave **no autentica al participante**: cualquier equipo remoto capaz
de acceder a esta red puede elegir un modo disponible, incluido administrador,
ocupar ejes, mover el brazo simulado o usar STOP y rearme. Use una LAN de
laboratorio confiable y no exponga el servicio a Internet. El gateway comprueba
el rol elegido, el origen, las reservas y los límites de comandos, pero esto no
demuestra la identidad del usuario.
Las IP locales de la madre están bloqueadas para acceso web en el modo normal;
`--allow-local-web` elimina esa restricción durante la prueba.
La IP no es una identidad criptográfica de dispositivo. La madre y sus procesos
locales pertenecen al límite de confianza: esto no protege contra malware o un
proceso local malicioso. ROS está configurado para comunicarse localmente, lo
que no protege frente a procesos locales no confiables.

Los ensayos con computadoras físicas en LAN y una validación más amplia de
movimientos y poses deben completarse antes de declarar cerrado H3. Una prueba
aislada ya comparó los eslabones del visor con TF durante un movimiento corto;
sus resultados están en experiments/run_sheets/jaco2_lab_visual_001.md. Las pruebas
automatizadas no sustituyen ese ensayo. Ante un fallo se muestra FAULT o datos
no disponibles; no se deben interpretar datos vencidos como actuales.
