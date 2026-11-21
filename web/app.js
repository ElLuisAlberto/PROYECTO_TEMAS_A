/* A staged, single-screen remote client for the simulated Jaco2 gateway. */
(() => {
  "use strict";
  const AXES = ["X", "Y", "Z", "ROLL", "PITCH", "YAW"];
  const el = (id) => document.getElementById(id);
  const ui = Object.fromEntries([
    "connection", "endpoint", "system-log", "axis-summary", "axis-ring-fill", "reserved-count", "rail-note", "workspace-intro", "home", "modes", "selection", "dashboard", "mode-message", "mode-description", "confirm-mode",
    "axis-options", "count", "enter", "back", "selection-message",
    "dashboard-kicker", "dashboard-title", "room-state", "operator-panel",
    "admin-panel", "controls", "telemetry-status", "telemetry", "telemetry-note",
    "owners", "dashboard-message", "stop", "rearm", "telemetry-advanced", "admin-credential", "admin-token",
  ].map((id) => [id, el(id)]));
  const identity = Math.random().toString(36).slice(2, 8).toUpperCase();
  const state = {
    socket: null, connected: false, role: null, screen: "modes", session: null,
    axes: {}, modes: {}, room: null, revision: -1, selected: new Set(),
    leases: new Map(), values: new Map(), speeds: new Map(), grantRevision: -1,
    sequence: 0, heartbeat: null, input: null, retry: null,
    pendingClaim: false, telemetry: null, selectedRole: null,
  };
  const ROLE_NAMES = { operator: "Operador", observer: "Observador", admin: "Administrador" };
  let physical = false, readOnly = false;
  let accessToken = null;
  el("robot-mode").textContent = physical ? "J2N6S300 · BRAZO FÍSICO" : "J2N6S300 · CONECTANDO";

  function show(screen) {
    state.screen = screen;
    ui["workspace-intro"].hidden = screen !== "modes";
    for (const name of ["modes", "selection", "dashboard"]) ui[name].hidden = name !== screen;
    ui.home.hidden = screen === "modes";
  }
  function log(text) {
    const time = new Date().toLocaleTimeString("es", {hour12:false});
    ui["system-log"].textContent = `${time} · ${text}`;
  }
  function renderRail() {
    const known = Boolean(state.connected && state.session && AXES.every((axis) => state.axes[axis] && typeof state.axes[axis].assigned === "boolean"));
    const reserved = known ? AXES.filter((axis) => state.axes[axis].assigned).length : 0;
    ui["reserved-count"].textContent = known ? String(reserved) : "—";
    ui["axis-ring-fill"].setAttribute("stroke-dasharray", `${reserved / AXES.length * 100} 100`);
    ui["axis-summary"].setAttribute("aria-valuetext", known ? `${reserved} ejes reservados de 6` : "Esperando datos");
    if (known) ui["axis-summary"].setAttribute("aria-valuenow", String(reserved));
    else ui["axis-summary"].removeAttribute("aria-valuenow");
    const subject = physical ? "Brazo físico" : "Simulación";
    const roomLabels = {READY:`${subject} listo para operar.`, ACTIVE:"Control cooperativo en curso.", FAULT:`${subject} · movimiento bloqueado.`, NOT_READY:`${subject} todavía no está listo.`};
    ui["rail-note"].textContent = !known ? `Esperando estado: ${subject}.` : roomLabels[state.room] || "Estado recibido de la computadora madre.";
  }
  function status(text, kind = "") {
    ui.connection.textContent = text;
    ui.connection.className = "badge" + (kind ? " " + kind : "");
    log(`${text} · ${location.host}`);
    renderRail();
  }
  function send(message) {
    if (!state.connected || !state.socket || state.socket.readyState !== WebSocket.OPEN) return false;
    state.socket.send(JSON.stringify(message));
    return true;
  }
  function stopTimers() {
    clearInterval(state.heartbeat); clearInterval(state.input);
    state.heartbeat = state.input = null;
  }
  function sendInputs() {
    if (state.role !== "operator" || !state.session || !state.leases.size) return;
    const values = {}, leases = {};
    for (const [axis, lease] of state.leases) {
      values[axis] = state.values.get(axis) || 0;
      leases[axis] = {lease_id: lease.lease_id, generation: lease.generation};
    }
    send({type: "input", sequence: ++state.sequence, values, leases});
  }
  function zeroAll() {
    for (const axis of state.leases.keys()) state.values.set(axis, 0);
    sendInputs();
  }
  function startTimers() {
    stopTimers();
    state.heartbeat = setInterval(() => send({type:"heartbeat"}), 100);
    state.input = setInterval(sendInputs, 50);
  }
  function closeCurrent() {
    clearTimeout(state.retry); state.retry = null;
    zeroAll();
    if (state.role === "operator" && state.leases.size) {
      send({type:"release", axes:[...state.leases.keys()]});
    }
    const old = state.socket;
    state.socket = null;
    stopTimers();
    state.connected = false; state.session = null; state.leases.clear();
    state.values.clear(); state.sequence = 0; state.pendingClaim = false;
    if (old && old.readyState < WebSocket.CLOSING) old.close();
  }
  function connect(role, claimAfterWelcome = false) {
    closeCurrent();
    state.role = role;
    state.revision = -1;
    state.pendingClaim = claimAfterWelcome;
    status("Conectando…");
    if (location.protocol !== "https:") {
      status("Se requiere HTTPS", "bad");
      ui["mode-message"].textContent = "Abre la dirección HTTPS que muestra la computadora madre.";
      return;
    }
    let socket;
    try { socket = new WebSocket(`wss://${location.host}/ws`); }
    catch { status("Sin conexión", "bad"); scheduleRetry(); return; }
    state.socket = socket;
    socket.addEventListener("open", () => {
      if (state.socket !== socket) return;
      state.connected = true;
      socket.send(JSON.stringify({type:"hello", room_id:"default",
        operator_name:`Equipo ${identity}`, role, ...(accessToken ? {credential:accessToken} : {})}));
      accessToken = null;
    });
    socket.addEventListener("message", (event) => {
      if (state.socket !== socket) return;
      let message;
      try { message = JSON.parse(event.data); } catch { return; }
      if (message && typeof message === "object") onMessage(message);
    });
    socket.addEventListener("error", () => {
      if (state.socket === socket) status("Sin conexión", "bad");
    });
    socket.addEventListener("close", () => {
      if (state.socket !== socket) return;
      stopTimers(); state.socket = null; state.connected = false; state.session = null;
      state.leases.clear(); state.values.clear(); state.pendingClaim = false;
      state.axes = {}; state.modes = {}; state.room = null; state.telemetry = null;
      state.selectedRole = null;
      status("Desconectado", "bad");
      show("modes"); renderModes(); renderTelemetry();
      ui["mode-message"].textContent = "Conexión perdida. Reconectando a la computadora madre…";
      scheduleRetry();
    });
  }
  function scheduleRetry() {
    log("Reconectando a la computadora madre…");
    clearTimeout(state.retry);
    state.retry = setTimeout(() => connect("observer"), 1500);
  }
  function goHome(message = "") {
    state.selected.clear();
    state.selectedRole = null;
    show("modes");
    ui["mode-message"].textContent = message || "Selecciona un modo para continuar.";
    connect("observer");
    renderModes();
  }
  function renderModes() {
    for (const role of ["operator", "observer", "admin"]) {
      const button = document.querySelector(`[data-role="${role}"]`);
      const label = el(`available-${role}`);
      const known = state.session && typeof state.modes[role] === "boolean";
      const selected = state.selectedRole === role;
      button.disabled = !known;
      button.classList.toggle("selected", selected);
      button.setAttribute("aria-pressed", String(selected));
      label.textContent = !known ? "Consultando…" : state.modes[role] ? "● Disponible" : "● Ocupado";
      label.classList.toggle("unavailable", Boolean(known && !state.modes[role]));
      button.setAttribute("aria-label", `${ROLE_NAMES[role]}: ${label.textContent}${selected ? ", seleccionado" : ""}`);
    }
    const role = state.selectedRole;
    ui["admin-credential"].hidden = role !== "admin";
    const known = Boolean(role && state.session && typeof state.modes[role] === "boolean");
    ui["confirm-mode"].disabled = !known || !state.connected || !state.modes[role]
      || (role === "admin" && ui["admin-token"].value.trim().length < 32);
    ui["mode-description"].textContent = !role ? "Elige una tarjeta para empezar."
      : !known ? `${ROLE_NAMES[role]} · Consultando disponibilidad…`
      : state.modes[role] ? `${ROLE_NAMES[role]} disponible · Vuelve a pulsar la tarjeta o confirma.`
      : `${ROLE_NAMES[role]} no disponible · Selecciona otro modo.`;
  }
  function renderOptions(focusAxis = null) {
    ui["axis-options"].replaceChildren();
    for (const axis of AXES) {
      const item = state.axes[axis];
      const taken = !item || item.assigned === true;
      if (taken) state.selected.delete(axis);
      const button = document.createElement("button");
      button.dataset.axis = axis;
      button.className = "axis-option" + (state.selected.has(axis) ? " selected" : "");
      button.disabled = taken || !state.session || state.room === "FAULT" || state.room === "NOT_READY";
      button.setAttribute("aria-pressed", String(state.selected.has(axis)));
      const name = document.createElement("span"); name.textContent = axis;
      const note = document.createElement("small");
      note.textContent = !item ? "Sin datos" : taken ? `Ocupado · ${item.owner || "otro equipo"}` : state.selected.has(axis) ? "Seleccionado" : "Disponible";
      button.append(name, note);
      button.addEventListener("click", () => {
        if (state.selected.has(axis)) state.selected.delete(axis);
        else if (state.selected.size < 3) state.selected.add(axis);
        else ui["selection-message"].textContent = "Puedes seleccionar hasta tres componentes.";
        renderOptions(axis);
      });
      ui["axis-options"].append(button);
      if (axis === focusAxis && !button.disabled) button.focus();
    }
    ui.count.textContent = `${state.selected.size} / 3 seleccionados`;
    ui.enter.disabled = !state.selected.size || !state.session || state.room === "FAULT" || state.room === "NOT_READY";
  }
  function renderOwners() {
    ui.owners.replaceChildren();
    for (const axis of AXES) {
      const item = state.axes[axis];
      const row = document.createElement("div");
      const ownership = !item ? "unknown" : state.leases.has(axis) ? "mine" : item.assigned ? "taken" : "free";
      row.className = `owner is-${ownership}`;
      const name = document.createElement("strong"); name.textContent = axis;
      const owner = document.createElement("span");
      owner.textContent = !item ? "Sin datos" : state.leases.has(axis) ? "Tú" : item.assigned ? `Ocupado · ${item.owner || "otro equipo"}` : "Disponible";
      row.append(name, owner); ui.owners.append(row);
    }
  }
  function formatVector(value, size) {
    return Array.isArray(value) && value.length === size && value.every(Number.isFinite)
      ? value.map((n) => n.toFixed(3)).join("  ") : "—";
  }
  function renderTelemetry() {
    ui.telemetry.replaceChildren();
    const t = state.telemetry;
    const age = t && Number.isFinite(t.pose_age_sec) && t.pose_age_sec >= 0
      ? t.pose_age_sec + (performance.now() - t.receivedAt) / 1000 : Infinity;
    const fresh = state.connected && t && t.telemetry_available !== false && age <= 1;
    ui["telemetry-status"].textContent = fresh ? "En vivo" : "No disponible";
    ui["telemetry-status"].className = "badge " + (fresh ? "good" : "bad");
    ui["telemetry-note"].textContent = fresh
      ? `${physical ? "Jaco2 físico · USB · movimiento no validado · armado no disponible" : "Jaco2 simulado"} · muestra hace ${age.toFixed(2)} s`
      : "Sin muestra reciente; no se muestran datos vencidos.";
    const groups = [
      ["Posición", "m", t?.pose?.position],
      ["Velocidad lineal comandada", "m/s", t?.twist?.linear],
      ["Velocidad angular comandada", "rad/s", t?.twist?.angular],
    ];
    for (const [label, unit, values] of groups) {
      const group = document.createElement("section"); group.className = "telemetry-group";
      const heading = document.createElement("h4"); heading.className = "telemetry-label";
      heading.textContent = `${label} (${unit})`;
      const vector = document.createElement("dl"); vector.className = "telemetry-vector";
      const valid = fresh && Array.isArray(values) && values.length === 3 && values.every(Number.isFinite);
      for (let i = 0; i < 3; i++) {
        const metric = document.createElement("div"); metric.className = "telemetry-metric";
        const axis = document.createElement("dt"); axis.className = "metric-axis"; axis.textContent = AXES[i];
        const value = document.createElement("dd"); value.className = "metric-value";
        value.textContent = valid ? values[i].toFixed(3) : "—";
        metric.append(axis, value); vector.append(metric);
      }
      group.append(heading, vector); ui.telemetry.append(group);
    }
    ui["telemetry-advanced"].replaceChildren();
    for (const [label, value] of [
      ["Cuaternión (x, y, z, w)", fresh ? formatVector(t.pose?.quaternion_xyzw, 4) : "—"],
      ["Articulaciones J1–J6 (rad)", fresh ? formatVector(t.joint_positions, 6) : "—"],
    ]) {
      const dt = document.createElement("dt"), dd = document.createElement("dd");
      dt.textContent = label; dd.textContent = value; ui["telemetry-advanced"].append(dt, dd);
    }
  }

  function renderRoom(message) {
    if (Number.isInteger(message.revision) && message.revision < state.revision) return;
    const oldOptions = JSON.stringify([state.axes, state.room]);
    if (Number.isInteger(message.revision)) state.revision = message.revision;
    const previousRoom = state.room;
    state.room = message.state || state.room;
    if (previousRoom !== state.room) log(`${physical ? "Estado del brazo físico" : "Estado de simulación"}: ${state.room}`);
    if (message.modes && typeof message.modes === "object") state.modes = message.modes;
    if (message.axes && typeof message.axes === "object") state.axes = message.axes;
    if (state.leases.size && state.revision > state.grantRevision) {
      let changed = false;
      for (const [axis, lease] of state.leases) {
        const item = state.axes[axis];
        if (!item?.assigned || item.generation !== lease.generation) {
          state.values.set(axis, 0); state.leases.delete(axis); changed = true;
        }
      }
      if (changed) {
        renderControls();
        ui["dashboard-message"].textContent = "Una reserva terminó. Vuelve a elegir componentes si deseas continuar.";
      }
    }
    ui["room-state"].textContent = state.room || "—";
    ui["room-state"].className = "badge " + (state.room === "FAULT" ? "bad" : "good");
    ui.rearm.disabled = state.room !== "FAULT";
    renderModes(); renderOwners(); renderRail();
    if (state.screen === "selection" && oldOptions !== JSON.stringify([state.axes, state.room])) {
      const focusedAxis = document.activeElement?.dataset?.axis || null;
      renderOptions(focusedAxis);
    }
  }
  function makeMoveButton(axis, direction) {
    const button = document.createElement("button");
    button.className = "move"; button.textContent = direction < 0 ? "−" : "+";
    button.setAttribute("aria-label", `Mover ${axis} ${direction < 0 ? "negativo" : "positivo"}`);
    const start = (event) => {
      if (!state.leases.has(axis) || !state.connected) return;
      if (event.type === "keydown" && event.repeat) return;
      event.preventDefault(); state.values.set(axis, direction * (state.speeds.get(axis) ?? 0.1)); sendInputs();
      if (event.type === "pointerdown") {
        try { button.setPointerCapture(event.pointerId); } catch { /* already released */ }
      }
    };
    const end = (event) => {
      if (event && event.type === "keyup" && ![" ", "Enter"].includes(event.key)) return;
      if (Math.sign(state.values.get(axis) || 0) === direction) { state.values.set(axis, 0); sendInputs(); }
    };
    button.addEventListener("pointerdown", start);
    for (const event of ["pointerup", "pointercancel", "lostpointercapture", "blur"]) button.addEventListener(event, end);
    button.addEventListener("keydown", (event) => { if ([" ", "Enter"].includes(event.key)) start(event); });
    button.addEventListener("keyup", end);
    return button;
  }
  function renderControls() {
    ui.controls.replaceChildren();
    for (const axis of AXES) {
      if (!state.leases.has(axis)) continue;
      const row = document.createElement("div"); row.className = "control-row";
      const title = document.createElement("div"); title.className = "control-name";
      const name = document.createElement("strong"); name.textContent = axis;
      const kind = document.createElement("small"); kind.textContent = AXES.indexOf(axis) < 3 ? "Traslación" : "Rotación";
      title.append(name, kind);
      const speed = document.createElement("label"); speed.className = "control-speed";
      const description = document.createElement("span");
      const slider = document.createElement("input");
      slider.type = "range"; slider.min = "1"; slider.max = "100"; slider.step = "1";
      slider.value = String(Math.round((state.speeds.get(axis) ?? 0.1) * 100));
      slider.setAttribute("aria-label", `Velocidad de ${axis}`);
      const updateDescription = () => {
        const percent = Number(slider.value);
        const linear = AXES.indexOf(axis) < 3;
        description.textContent = `Velocidad: ${percent}% · ${(percent * (linear ? 0.5 : 0.175 * 180 / Math.PI / 100)).toFixed(linear ? 1 : 2)} ${linear ? "mm/s" : "°/s"} máximo`;
      };
      updateDescription();
      slider.addEventListener("input", () => {
        const value = Number(slider.value) / 100;
        state.speeds.set(axis, value);
        const current = state.values.get(axis) || 0;
        if (current !== 0) { state.values.set(axis, Math.sign(current) * value); sendInputs(); }
        updateDescription();
      });
      speed.append(description, slider);
      row.append(title, makeMoveButton(axis, -1), makeMoveButton(axis, 1), speed);
      ui.controls.append(row);
    }
  }
  function enterDashboard(role) {
    ui.dashboard.dataset.role = role;
    ui["dashboard-kicker"].textContent = `${ROLE_NAMES[role].toUpperCase()} · ${physical ? "BRAZO FÍSICO" : "SIMULACIÓN"}`;
    ui["dashboard-title"].textContent = readOnly ? "Estado del brazo" : role === "operator" ? "Tus controles" : role === "admin" ? "Administración" : "Estado del brazo";
    ui["operator-panel"].hidden = readOnly || role !== "operator";
    ui["admin-panel"].hidden = readOnly || role !== "admin";
    ui["dashboard-message"].textContent = readOnly ? "Sesión física de solo lectura." : "";
    log(`${ROLE_NAMES[role]} conectado · ${physical ? "Brazo físico" : "Simulación"} ${state.room || "—"}`);
    show("dashboard"); renderOwners(); renderTelemetry();
  }
  function onMessage(message) {
    switch (message.type) {
      case "welcome":
        if (Boolean(message.physical_session) !== physical || Boolean(message.read_only) !== readOnly) {
          closeCurrent(); status("Modo incorrecto", "bad");
          ui["mode-message"].textContent = "El modo de esta página no coincide con el servidor. Usa la dirección indicada por la estación base.";
          break;
        }
        el("robot-mode").textContent = `J2N6S300 · ${physical ? "BRAZO FÍSICO" : "SIMULACIÓN"}`;
        el("admin-title").textContent = physical ? "Seguridad del brazo físico" : "Seguridad de la simulación";
        state.session = message.session_id;
        state.room = message.state;
        status("Conectado", "good"); startTimers();
        if (state.role === "observer" && state.screen === "modes" &&
            /^(Esperando|Conexión perdida)/.test(ui["mode-message"].textContent)) {
          ui["mode-message"].textContent = "";
        }
        if (state.pendingClaim) {
          state.pendingClaim = false;
          if (!send({type:"claim", axes:[...state.selected]})) goHome("No se pudo solicitar la reserva.");
        } else if (state.role === "admin" || (readOnly && state.role === "operator")) enterDashboard(state.role);
        break;
      case "room_state": renderRoom(message); break;
      case "telemetry":
        state.telemetry = {...message, receivedAt:performance.now()};
        if (message.state) { state.room = message.state; ui["room-state"].textContent = message.state; renderRail(); }
        if (state.screen === "dashboard") renderTelemetry();
        break;
      case "claim_granted":
        // A queued room_state can precede this grant even if it arrives after
        // the direct response. Only newer states may revoke the new leases.
        state.grantRevision = Number.isInteger(message.revision) ? message.revision : state.revision;
        for (const lease of message.leases || []) {
          if (AXES.includes(lease.axis) && typeof lease.lease_id === "string" && Number.isInteger(lease.generation)) {
            state.leases.set(lease.axis, lease); state.values.set(lease.axis, 0);
          }
        }
        if (!state.leases.size) { goHome("No se recibió una reserva válida."); break; }
        renderControls(); enterDashboard("operator"); sendInputs(); break;
      case "stop_confirmed": log("STOP confirmado · Reservas revocadas"); ui["dashboard-message"].textContent = "STOP confirmado: reservas revocadas."; break;
      case "rearm_confirmed": log("Rearme confirmado"); ui["dashboard-message"].textContent = "Rearme confirmado."; break;
      case "error":
        if (message.code === "authentication_failed" && state.role === "observer") {
          closeCurrent(); state.axes = {}; state.modes = {}; state.telemetry = null;
          show("modes"); renderModes(); renderTelemetry();
          status("Acceso no disponible", "bad");
          ui["mode-message"].textContent = "Abre esta página desde otra computadora de la misma red; la madre utiliza su consola nativa.";
        } else if (message.code === "mode_unavailable" || message.code === "authentication_failed") {
          goHome(message.code === "mode_unavailable" ? "Ese modo acaba de ocuparse. Elige otro." : "No se pudo acceder. Verifica que uses una computadora remota en la misma red.");
        } else if (state.screen === "selection") {
          ui["selection-message"].textContent = message.code === "claim_denied" ? "Uno de los ejes acaba de ocuparse. Actualiza tu selección." : (message.detail || "No fue posible ingresar.");
          if (state.role === "operator" && !state.leases.size) {
            state.role = "operator"; renderOptions();
          }
        } else {
          ui["dashboard-message"].textContent = message.detail || "La operación no pudo completarse.";
        }
        break;
    }
  }
  function confirmMode() {
    const role = state.selectedRole;
    if (!role || !state.connected || !state.session || !state.modes[role]) {
      ui["mode-message"].textContent = "Este modo no está disponible. Selecciona otro.";
      renderModes(); return;
    }
    if (role === "operator") {
      if (readOnly) { connect("operator"); return; }
      if (state.room === "FAULT" || state.room === "NOT_READY") { ui["mode-message"].textContent = "El brazo aún no está listo para operar."; return; }
      state.selected.clear(); ui["selection-message"].textContent = ""; show("selection"); renderOptions();
    } else if (role === "observer") enterDashboard("observer");
    else {
      accessToken = ui["admin-token"].value.trim();
      if (accessToken.length < 32) {
        ui["mode-message"].textContent = "Introduce la credencial privada de administrador.";
        renderModes(); return;
      }
      ui["admin-token"].value = "";
      connect("admin");
    }
  }
  document.querySelectorAll(".mode-card").forEach((button) => button.addEventListener("click", () => {
    const role = button.dataset.role;
    if (state.selectedRole === role && state.modes[role]) { confirmMode(); return; }
    state.selectedRole = role;
    ui["mode-message"].textContent = "";
    renderModes();
  }));
  ui["confirm-mode"].addEventListener("click", confirmMode);
  ui["admin-token"].addEventListener("input", renderModes);
  ui.back.addEventListener("click", () => { state.selected.clear(); show("modes"); renderModes(); });
  ui.enter.addEventListener("click", () => {
    if (state.selected.size < 1 || state.selected.size > 3) return;
    ui.enter.disabled = true;
    ui["selection-message"].textContent = "Comprobando disponibilidad y reservando…";
    if (state.role === "operator" && state.session && state.connected) {
      send({type:"claim", axes:[...state.selected]});
    } else {
      connect("operator", true);
    }
  });
  ui.home.addEventListener("click", () => goHome());
  ui.stop.addEventListener("click", () => { if (confirm(`¿Detener ${physical ? "el brazo físico" : "la simulación"} y liberar todos los ejes?`)) send({type:"stop_global"}); });
  ui.rearm.addEventListener("click", () => { if (confirm(`¿Rearmar ${physical ? "la sesión del brazo físico" : "la simulación"} si está lista?`)) send({type:"rearm"}); });
  window.addEventListener("blur", zeroAll);
  window.addEventListener("pagehide", () => { zeroAll(); if (state.socket) state.socket.close(); });
  document.addEventListener("visibilitychange", () => { if (document.hidden) zeroAll(); });
  setInterval(() => { if (state.screen === "dashboard") renderTelemetry(); }, 250);
  ui.endpoint.textContent = location.host;
  async function openStation() {
    show("modes"); renderModes(); renderTelemetry();
    try {
      const response = await fetch("/session-mode", {cache:"no-store"});
      if (!response.ok) throw new Error();
      const {mode} = await response.json();
      if (!["read-only", "physical", "simulation"].includes(mode)) throw new Error();
      readOnly = mode === "read-only";
      physical = mode !== "simulation";
    } catch {
      status("Estación no disponible", "bad");
      ui["mode-message"].textContent = "No se pudo consultar la sesión de la computadora madre.";
      setTimeout(openStation, 1500);
      return;
    }
    if (readOnly) {
      for (const card of document.querySelectorAll(".mode-card")) {
        card.querySelector(".access-label").textContent = "SOLO LECTURA";
        card.querySelector(".module-description").textContent = "Consulta la postura del brazo físico.";
      }
    } else if (physical) {
      document.querySelector('[data-role="admin"] .module-description').textContent = "Supervisa, detiene y rearma la sesión del brazo físico.";
    }
    connect("observer");
  }
  openStation();
})();
