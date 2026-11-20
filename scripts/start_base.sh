#!/usr/bin/env bash
# Start the base station for the cooperative teleoperation demo.
# The simulated robot and ROS 2 stay on this computer; LAN clients use HTTPS.

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd -P)"
readonly ROS_SETUP="/opt/ros/humble/setup.bash"
readonly WORKSPACE_SETUP="$PROJECT_ROOT/ros2_ws/install/setup.bash"
readonly HTTPS_PORT=8443
readonly WEBSOCKET_PORT=8765
readonly STARTUP_TIMEOUT_SEC=60
readonly ROBOT_MODEL="jaco2"
readonly JACO2_LAUNCH_FILE="$PROJECT_ROOT/ros2_ws/src/course_teleop_simulation/launch/jaco2_sim.launch.py"
readonly JACO2_BRIDGE_FILE="$PROJECT_ROOT/scripts/jaco2_cartesian_ik_bridge.py"

declare -a CHILD_NAMES=()
declare -a CHILD_PIDS=()
LOG_DIR=""
CERT_DIR=""
CLEANUP_STARTED=0
HEADLESS=0
ALLOW_LOCAL_WEB=0

info() {
    printf '[base] %s\n' "$*"
}

error() {
    printf '[base] ERROR: %s\n' "$*" >&2
}

die() {
    error "$*"
    exit 1
}

usage() {
    cat <<'EOF'
Uso: bash scripts/start_base.sh [--headless] [--allow-local-web]

Inicia Gazebo Jaco2 sin GUI, puente IK, gateway, HTTPS/WSS y consola nativa.

Opciones:
  --headless  Ejecuta sin consola nativa; conserva los servicios LAN seguros.
  --allow-local-web  Permite usar la web desde la madre para probar la simulación.
  --help      Muestra esta ayuda.
EOF
}

parse_arguments() {
    while (($#)); do
        case "$1" in
            --headless)
                HEADLESS=1
                ;;
            --allow-local-web)
                ALLOW_LOCAL_WEB=1
                ;;
            --help)
                usage
                exit 0
                ;;
            *)
                error "Opción no reconocida: $1"
                usage >&2
                exit 2
                ;;
        esac
        shift
    done
}

require_command() {
    local command_name="$1"
    command -v "$command_name" >/dev/null 2>&1 || die "No se encontró el comando requerido: $command_name"
}

port_is_listening() {
    local port="$1"
    ss -H -ltn "sport = :$port" | grep -q .
}

require_free_port() {
    local port="$1"
    if port_is_listening "$port"; then
        die "El puerto $port ya está en uso. No se modificó ni detuvo ningún proceso existente."
    fi
}

is_child_running() {
    local pid="$1"
    kill -0 "$pid" 2>/dev/null
}

stop_process_group() {
    local pid="$1"
    # Each child is started with setsid, so this reaches any child it spawned too.
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
}

force_stop_process_group() {
    local pid="$1"
    kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
}

cleanup() {
    local exit_status=$?
    local index pid deadline local_any_running

    if (( CLEANUP_STARTED )); then
        return
    fi
    CLEANUP_STARTED=1
    trap - EXIT INT TERM

    if ((${#CHILD_PIDS[@]})); then
        info "Cerrando los procesos iniciados por esta estación base..."
        for ((index=${#CHILD_PIDS[@]} - 1; index >= 0; index--)); do
            pid="${CHILD_PIDS[index]}"
            if is_child_running "$pid"; then
                stop_process_group "$pid"
            fi
        done

        deadline=$((SECONDS + 10))
        while ((SECONDS < deadline)); do
            local_any_running=0
            for pid in "${CHILD_PIDS[@]}"; do
                if is_child_running "$pid"; then
                    local_any_running=1
                    break
                fi
            done
            (( local_any_running )) || break
            sleep 0.2
        done

        for pid in "${CHILD_PIDS[@]}"; do
            if is_child_running "$pid"; then
                force_stop_process_group "$pid"
            fi
        done
        for pid in "${CHILD_PIDS[@]}"; do
            wait "$pid" 2>/dev/null || true
        done
    fi

    if [[ -n "$LOG_DIR" ]]; then
        info "Registros de esta ejecución: $LOG_DIR"
    fi
    return "$exit_status"
}

on_interrupt() {
    info "Interrupción recibida."
    exit 130
}

start_process() {
    local name="$1"
    shift
    local logfile="$LOG_DIR/$name.log"
    local pid

    # A new session makes shutdown deterministic without ever signalling a
    # process that was already running before this script was invoked.
    setsid "$@" >"$logfile" 2>&1 &
    pid="$!"
    CHILD_NAMES+=("$name")
    CHILD_PIDS+=("$pid")
    info "Iniciado $name (PID $pid). Registro: $logfile"
}

child_pid_for() {
    local requested_name="$1"
    local index
    for index in "${!CHILD_NAMES[@]}"; do
        if [[ "${CHILD_NAMES[index]}" == "$requested_name" ]]; then
            printf '%s\n' "${CHILD_PIDS[index]}"
            return 0
        fi
    done
    return 1
}

report_child_failure() {
    local name="$1"
    local pid logfile
    pid="$(child_pid_for "$name")"
    logfile="$LOG_DIR/$name.log"
    error "$name terminó antes de completar el inicio (PID $pid)."
    if [[ -s "$logfile" ]]; then
        error "Últimas líneas de $logfile:"
        tail -n 20 "$logfile" >&2 || true
    fi
    exit 1
}

wait_for_port() {
    local port="$1"
    local process_name="$2"
    local deadline=$((SECONDS + STARTUP_TIMEOUT_SEC))
    local pid
    pid="$(child_pid_for "$process_name")"

    while ((SECONDS < deadline)); do
        if port_is_listening "$port"; then
            return 0
        fi
        is_child_running "$pid" || report_child_failure "$process_name"
        sleep 0.25
    done
    die "Tiempo de espera agotado esperando que $process_name atienda en el puerto $port. Revise $LOG_DIR/$process_name.log"
}

wait_for_clock_topic() {
    local process_name="$1"
    local deadline=$((SECONDS + STARTUP_TIMEOUT_SEC))
    local pid
    pid="$(child_pid_for "$process_name")"

    while ((SECONDS < deadline)); do
        # An existing ROS CLI daemon may have been started with different DDS
        # isolation. Query this launch environment directly and bound discovery.
        if timeout 5s ros2 topic list --no-daemon --spin-time 0.5 2>/dev/null | grep -Fxq '/clock'; then
            return 0
        fi
        is_child_running "$pid" || report_child_failure "$process_name"
        sleep 0.5
    done
    die "Tiempo de espera agotado esperando /clock de Gazebo. Revise $LOG_DIR/$process_name.log"
}

default_route_lan_ipv4() {
    local route interface
    route="$(ip -4 route get 1.1.1.1 2>/dev/null || true)"
    LAN_IPV4="$(awk '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }' <<<"$route")"
    if [[ -n "$LAN_IPV4" ]]; then
        return 0
    fi

    route="$(ip -4 route show default 2>/dev/null || true)"
    LAN_IPV4="$(awk '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }' <<<"$route")"
    if [[ -n "$LAN_IPV4" ]]; then
        return 0
    fi

    interface="$(awk '{ for (i = 1; i <= NF; i++) if ($i == "dev") { print $(i + 1); exit } }' <<<"$route")"
    [[ -n "$interface" ]] || die "No se pudo determinar la interfaz de la ruta IPv4 por defecto."
    LAN_IPV4="$(ip -o -4 addr show dev "$interface" scope global | awk '{ split($4, address, "/"); print address[1]; exit }')"
    [[ -n "$LAN_IPV4" ]] || die "No se pudo determinar una IPv4 LAN en la interfaz $interface."
}

main() {
    local lan_url credentials_file
    local base_ip
    local -a local_ip_args=()
    local -a local_web_args=()
    if (( ALLOW_LOCAL_WEB )); then
        local_web_args+=(--allow-local-web)
    fi

    [[ -f "$ROS_SETUP" ]] || die "No existe ROS 2 Humble en $ROS_SETUP"
    [[ -f "$WORKSPACE_SETUP" ]] || die "No existe el workspace instalado en $WORKSPACE_SETUP. Compile ros2_ws antes de iniciar la estación base."
    [[ -f "$JACO2_LAUNCH_FILE" ]] || die "No existe el launch de simulación Jaco2: $JACO2_LAUNCH_FILE"
    [[ -f "$JACO2_BRIDGE_FILE" ]] || die "No existe el puente cartesiano Jaco2: $JACO2_BRIDGE_FILE"

    # ROS 2 Humble setup scripts read optional variables that may be unset.
    # Keep strict mode for this launcher, but temporarily allow that upstream
    # convention while loading their environment.
    set +u
    # shellcheck disable=SC1091
    source "$ROS_SETUP"
    # shellcheck disable=SC1091
    source "$WORKSPACE_SETUP"
    set -u
    export PYTHONPATH="$PROJECT_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
    export ROS_LOCALHOST_ONLY=1
    umask 077

    if (( ! HEADLESS )); then
        [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]] || die "La consola necesita una sesión gráfica local. Use --headless si no hay pantalla."
    fi
    require_command ros2
    require_command ign
    require_command ss
    require_command ip
    require_command setsid
    require_command timeout
    require_command python3
    require_command openssl

    if (( ! HEADLESS )); then
        if ! python3 -c 'import PyQt5, vtk, collada' >/dev/null 2>&1; then
            die "La consola requiere PyQt5, VTK y pycollada para Python 3. Consulte requirements.txt."
        fi
    fi

    if ! python3 -c '
import websockets
major = int(websockets.__version__.split(".", 1)[0])
raise SystemExit(0 if 12 <= major < 16 else 1)
'; then
        die "Se requiere websockets>=12,<16 para el gateway."
    fi

    require_free_port "$HTTPS_PORT"
    require_free_port "$WEBSOCKET_PORT"
    default_route_lan_ipv4
    while IFS= read -r base_ip; do
        [[ -n "$base_ip" ]] && local_ip_args+=(--base-local-ip "$base_ip")
    done < <(ip -o -4 addr show | awk '{split($4, address, "/"); print address[1]}')
    ((${#local_ip_args[@]})) || die "No se pudieron detectar las direcciones locales de la estación base."

    LOG_DIR="$(mktemp -d "${TMPDIR:-/tmp}/teleop-base.XXXXXX")"
    CERT_DIR="${XDG_DATA_HOME:-$HOME/.local/share}/teleop-jaco2"
    mkdir -p -m 700 "$CERT_DIR"
    credentials_file="$LOG_DIR/credentials.json"
    python3 "$SCRIPT_DIR/prepare_lan_security.py" --ip "$LAN_IPV4" --run-dir "$LOG_DIR" --cert-dir "$CERT_DIR" || die "No se pudo preparar la CA, el certificado o las credenciales. No se sobrescribieron configuraciones previas."
    info "Raíz del proyecto: $PROJECT_ROOT"
    if (( HEADLESS )); then
        info "Modo headless: no se abrirá la consola local."
    else
        info "Sesión gráfica local detectada."
    fi
    info "IPv4 LAN detectada por la ruta por defecto: $LAN_IPV4"

    start_process jaco2_gazebo \
        ros2 launch course_teleop_simulation jaco2_sim.launch.py "headless:=true"
    wait_for_clock_topic jaco2_gazebo

    start_process jaco2_cartesian_ik_bridge python3 "$JACO2_BRIDGE_FILE"
    start_process gateway \
        python3 -m teleop_cooperativo.gateway \
        --host 127.0.0.1 \
        --port "$WEBSOCKET_PORT" \
        --credentials-file "$credentials_file" \
        --anonymous-lan \
        "${local_web_args[@]}" \
        --allowed-origin "https://$LAN_IPV4:$HTTPS_PORT" \
        --trusted-proxy 127.0.0.1 \
        "${local_ip_args[@]}" \
        --event-log "$LOG_DIR/events.jsonl" \
        --run-id "$(basename -- "$LOG_DIR")" \
        --robot-model "$ROBOT_MODEL" \
        --ros-twist-topic /teleop/cartesian_twist \
        --ros-ready-timeout-sec "$STARTUP_TIMEOUT_SEC"
    start_process https_proxy \
        python3 "$SCRIPT_DIR/lan_https_proxy.py" --host "$LAN_IPV4" --port "$HTTPS_PORT" \
        --backend-port "$WEBSOCKET_PORT" --directory "$PROJECT_ROOT/web" \
        --cert "$CERT_DIR/server-$LAN_IPV4/server.crt" --key "$CERT_DIR/server-$LAN_IPV4/server.key"

    wait_for_port "$WEBSOCKET_PORT" gateway
    wait_for_port "$HTTPS_PORT" https_proxy

    if (( ! HEADLESS )); then
        start_process desktop_console \
            env -u GTK_PATH -u GTK_EXE_PREFIX -u GIO_MODULE_DIR \
            -u GTK_IM_MODULE_FILE -u GDK_PIXBUF_MODULE_FILE -u GDK_PIXBUF_MODULEDIR \
            -u GSETTINGS_SCHEMA_DIR -u LOCPATH \
            python3 -m teleop_cooperativo.desktop_console \
            --server-url "ws://127.0.0.1:$WEBSOCKET_PORT/ws" \
            --credentials-file "$credentials_file" \
            --room-id default
    fi

    lan_url="https://$LAN_IPV4:$HTTPS_PORT"
    info "Interfaz para las otras computadoras: $lan_url"
    if (( ALLOW_LOCAL_WEB )); then
        info "Prueba local habilitada: esta computadora también puede abrir $lan_url y operar la simulación."
    fi
    info "Instale la CA pública $CERT_DIR/ca/ca.crt como confiable en cada equipo remoto."
    info "Acceso web sin claves: use solo una LAN de confianza; cualquier equipo remoto puede elegir un modo disponible."
    info "No comparta la clave privada de la CA ni el archivo de credenciales internas."
    if (( HEADLESS )); then
        info "Modo headless listo: use el enlace LAN desde un navegador remoto."
    else
        info "Gazebo funciona sin GUI; consola nativa iniciada en la computadora base."
        info "Interfaz web de operadores y observadores: $lan_url"
    fi

    info "Estación base lista. Pulse Ctrl+C para cerrarla ordenadamente."
    while true; do
        local index name pid
        for index in "${!CHILD_NAMES[@]}"; do
            name="${CHILD_NAMES[index]}"
            pid="${CHILD_PIDS[index]}"
            is_child_running "$pid" || report_child_failure "$name"
        done
        sleep 1
    done
}

trap cleanup EXIT
trap on_interrupt INT TERM
parse_arguments "$@"
main
