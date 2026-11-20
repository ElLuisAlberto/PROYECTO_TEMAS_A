# H3 — Gateway WebSocket y web mínima

> Nota de actualización: esta página conserva el contrato original de H3 y
> sus pruebas. El arranque integrado actual usa HTTPS/WSS y selección de rol
> sin clave para los equipos remotos; la LAN debe ser confiable. Ofrece también
> administración remota de STOP/rearme. Para el procedimiento
> vigente consulte [estación base y LAN segura](08_estacion_base_y_lan_segura.md).
> La interfaz actual establece la conexión automáticamente y no muestra el
> intercambio interno `hello` al usuario.

## Propósito y límite

H3 expone el árbitro local a operadores web en una red LAN. El gateway es la
única parte que conoce WebSocket; los navegadores nunca reciben acceso a ROS 2,
Gazebo, joints, poses ni controladores. Con `--local-simulator`, la laptop
madre conecta ese comando al arnés cartesiano determinista. Con la ruta Jaco2,
el adaptador ROS publica telemetría observacional de pose por FK, joints y
twist. En ambos casos las interfaces solo observan esos datos; no constituyen
una ruta de control hacia ROS.

Una instancia del gateway representa una sala. El identificador de sala que
recibe el navegador es informativo en H3: no crea árbitros ni rutas de control
adicionales. La laptop madre conserva un único `CooperativeArbiter` y es la
autoridad de su reloj monotónico.

## Transporte y formato

- El cliente abre un WebSocket con la URL publicada por la laptop madre.
- Cada frame de aplicación es un objeto JSON UTF-8; no se aceptan arrays,
  escalares ni JSON malformado.
- Todo mensaje debe contener `type`, una cadena no vacía.
- Un error de formato o de autorización genera `error`; no cambia leases ni
  comando activo. El servidor puede cerrar conexiones que persistan en enviar
  frames inválidos.
- Los ejes se expresan como `X`, `Y`, `Z`, `ROLL`, `PITCH` y `YAW`. Los
  valores se normalizan en el intervalo cerrado `[-1, 1]`.

Los campos adicionales se ignoran solo cuando sean puramente informativos. No
deben alterar la autoridad ni la semántica de un mensaje.

## Mensajes de cliente

### `hello`

Debe ser el primer mensaje útil de cada conexión.

```json
{
  "type": "hello",
  "operator_name": "operador-a",
  "room_id": "laboratorio-1"
}
```

`operator_name` es una etiqueta visible, no una credencial. El servidor genera
un `session_id` opaco y lo asocia al socket; el cliente no puede elegirlo ni
usar el de otro socket.

### `claim`

```json
{"type": "claim", "axes": ["X", "Y", "Z"]}
```

La solicitud es atómica y solo es válida después de `hello`. Puede contener de
uno a tres ejes únicos. El límite total de tres ejes por sesión sigue vigente;
por ello una solicitud que exceda ese límite o incluya un eje ocupado no cambia
ninguna reserva.

### `input`

```json
{
  "type": "input",
  "sequence": 7,
  "values": {"X": 1.0, "Y": -0.25},
  "leases": {
    "X": {"lease_id": "uuid", "generation": 1},
    "Y": {"lease_id": "uuid", "generation": 1}
  }
}
```

`sequence` es un entero estrictamente creciente por sesión. `values` no puede
estar vacío y cada eje debe pertenecer a la sesión. `leases` contiene el token
que se recibió en `claim_granted`; el gateway lo traduce al formato interno del
dominio. El tiempo del navegador no se usa para frescura. La aceptación de un
`input` renueva el heartbeat de los ejes de esa sesión.

### `heartbeat`

```json
{"type": "heartbeat"}
```

Mantiene vivas las reservas existentes, sin producir movimiento. La web debe
enviarlo a una frecuencia que deje margen suficiente antes del timeout de
lease. No renueva el timeout de input, por lo que el comando se vuelve cero si
se deja de enviar `input`.

### `release`

```json
{"type": "release", "axes": ["Y"]}
```

Libera los ejes propios indicados después de poner su valor en cero. Si se
omite `axes`, se liberan todos los ejes de la sesión. Liberar un eje que no
pertenece a la sesión no concede ni revoca autoridad de terceros.

## Mensajes del servidor

Todos incluyen `type`. Las instantáneas de sala usan la siguiente forma
canónica:

```json
{
  "type": "room_state",
  "state": "ACTIVE",
  "axes": {
    "X": {"assigned": true, "owner": "operador-a", "generation": 1},
    "Y": {"assigned": false, "owner": null, "generation": null},
    "Z": {"assigned": false, "owner": null, "generation": null},
    "ROLL": {"assigned": false, "owner": null, "generation": null},
    "PITCH": {"assigned": false, "owner": null, "generation": null},
    "YAW": {"assigned": false, "owner": null, "generation": null}
  }
}
```

Los identificadores de lease nunca aparecen en `room_state`; se entregan solo
al propietario dentro de `claim_granted`. El nombre del propietario es
informativo y no autoriza mensajes.

| Tipo | Destinatario | Campos mínimos | Significado |
| --- | --- | --- | --- |
| `welcome` | socket nuevo | `session_id`, `state` | Confirma la sesión creada. |
| `room_state` | todos | `state`, `axes` | Instantánea luego de un cambio de ownership, expiración, conexión o desconexión. |
| `claim_granted` | solicitante | `leases` | Confirma la lista completa de leases, con `axis`, `lease_id` y `generation`. |
| `release_confirmed` | solicitante | `axes` | Confirma que los ejes propios se pusieron en cero y liberaron. |
| `telemetry` | todos | `state`, `twist`, `telemetry_available` y, si está disponible, `pose` | Observación del arnés local o de ROS/Gazebo; puede incluir `source`, `joint_positions` y `pose_age_sec`. |
| `error` | emisor | `code`, `detail` | Frame inválido, secuencia vieja, token inválido o estado no operativo. |

Los valores permitidos de `state` son `NOT_READY`, `READY`, `ACTIVE` y
`FAULT`. Los motivos y códigos se registran además en el log de la madre.

## Ciclo y cierre seguro

El gateway ejecuta un ciclo periódico no menor que 20 Hz. Con el arnés local,
un único ciclo avanza el árbitro y el simulador: no hay una segunda expiración
ni un segundo comando. Si el input vence, publica el comando ya puesto a cero;
si el heartbeat vence, actualiza `room_state` con el eje disponible. Cuando un
socket cierra, el gateway invoca inmediatamente `release` para la sesión
asociada, publica el nuevo estado y no reutiliza la sesión. Un STOP global o
un fallo del host deja la sala en `FAULT`, con seis valores cero y sin leases.

## Criterios de aceptación de H3

1. Dos clientes LAN pueden conectarse, reclamar ternas disjuntas y ver el
   mismo `room_state`.
2. Un claim que excede tres ejes, tiene duplicados o contiene un eje ocupado se
   rechaza sin asignación parcial.
3. Un `input` viejo, inválido o con token de otra generación no modifica el
   comando ni refresca la frescura.
4. Un cierre abrupto libera únicamente los ejes de ese socket y notifica al
   otro cliente.
5. La web permite realizar estos flujos sin enviar joints, poses o mensajes
   ROS al navegador.
6. El ensayo H3 queda registrado con logs de gateway y clientes; la hoja base
   está en `experiments/run_sheets/h3_gateway_lan_base.md`.

## Prueba manual reproducible

1. Iniciar el gateway siguiendo el README principal y anotar el comando, URL y
   parámetros en la hoja de ensayo.
2. Abrir la página en dos navegadores o perfiles separados y completar
   `hello` con nombres distintos.
3. A reclama `X/Y/Z` y B reclama `ROLL/PITCH/YAW`; comprobar en ambos clientes
   propietarios y tokens.
4. Iniciar con `--local-simulator`, enviar un control momentáneo desde A y
   comprobar la pose/twist mostrados en la web y el retorno a cero al vencer el
   input, manteniendo heartbeat.
5. Cerrar abruptamente A, medir la transición a disponible y comprobar que las
   reservas de B no cambian.
6. Adjuntar logs y capturas a la hoja de ensayo antes de declarar H3 cerrado.
