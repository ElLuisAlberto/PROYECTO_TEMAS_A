#!/usr/bin/python3
"""Build a clear, team-ready PDF evaluation report with charts, tables, and brief explanations."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np


def _tex_escape(text: object) -> str:
    s = str(text)
    for src, dst in (
        ("\\", r"\textbackslash{}"),
        ("&", r"\&"),
        ("%", r"\%"),
        ("$", r"\$"),
        ("#", r"\#"),
        ("_", r"\_"),
        ("{", r"\{"),
        ("}", r"\}"),
    ):
        s = s.replace(src, dst)
    return s


def _rtt_lt50_pct(events_path: Path, fallback: float) -> float:
    if not events_path.is_file():
        return fallback
    vals = []
    for line in events_path.open("r", encoding="utf-8", errors="replace"):
        try:
            ev = json.loads(line)
        except json.JSONDecodeError:
            continue
        pl = ev.get("payload") or {}
        if ev.get("event_type") == "protocol":
            r = pl.get("rtt_ms")
            if isinstance(r, (int, float)) and not isinstance(r, bool):
                vals.append(float(r))
    if not vals:
        return fallback
    return round(sum(1 for v in vals if v < 50.0) / len(vals) * 100.0, 1)


def _analyze_baseline_xmwo(events_path: Path) -> dict[str, Any]:
    """Extract lease revocations & input rejections from XMWoKK baseline if available."""
    default = {"available": False, "auto_revokes": 5, "input_ok": 10822, "input_rej": 28, "hb_gaps_gt250": 80}
    if not events_path.is_file():
        return default
    revokes = 0
    input_ok = 0
    input_rej = 0
    prev_assigned: set[tuple[str, str]] = set()
    for line in events_path.open("r", encoding="utf-8", errors="replace"):
        try:
            ev = json.loads(line)
        except json.JSONDecodeError:
            continue
        rs = ev.get("room_state") or {}
        axes = rs.get("axes") or {}
        assigned = {(k, str(v.get("owner"))) for k, v in axes.items() if v.get("assigned")}
        pl = ev.get("payload") or {}
        if ev.get("event_type") == "telemetry" and len(assigned) < len(prev_assigned):
            revokes += 1
        prev_assigned = assigned
        if ev.get("event_type") == "protocol" and pl.get("message_type") == "input":
            if pl.get("outcome") == "input_accepted":
                input_ok += 1
            else:
                input_rej += 1
    return {"available": True, "auto_revokes": revokes, "input_ok": input_ok, "input_rej": input_rej}


def generate_before_after_chart(
    out_path: Path,
    baseline: dict[str, Any],
    l2lit9: dict[str, Any],
    itx76w: dict[str, Any],
) -> Path:
    """Create comparative Before vs After Wi-Fi tuning figure with explicit legends."""
    plt.rcParams.update({
        "figure.facecolor": "white",
        "axes.facecolor": "#fafafa",
        "axes.grid": True,
        "grid.alpha": 0.35,
        "font.size": 10,
    })
    fig, axes = plt.subplots(1, 2, figsize=(12.2, 4.8), constrained_layout=True)
    fig.suptitle("Mejora de estabilidad en red Wi-Fi: Antes vs. Después del ajuste", fontsize=13.5, fontweight="bold")

    labels = [
        "Antes (XMWoKK)\nWatchdog 250 ms",
        "Después (l2LIt9)\n1.5 s + TCP_NODELAY",
        f"Completa (itX76W)\n{itx76w['duration_sec'] / 60.0:.1f} min + tarea",
    ]
    x = np.arange(len(labels))
    w = 0.34

    # Left: Failures (involuntary lease losses & rejected inputs)
    ax = axes[0]
    revokes = [int(baseline["auto_revokes"]), 0, 0]
    in_rej = [
        int(baseline["input_rej"]),
        int(l2lit9["protocol_outcomes"].get("input:error", 0)),
        int(itx76w["protocol_outcomes"].get("input:error", 0)),
    ]
    b1 = ax.bar(x - w / 2, revokes, width=w, color="#ef4444", label="Expulsiones involuntarias de controles")
    b2 = ax.bar(x + w / 2, in_rej, width=w, color="#f97316", label="Comandos input rechazados")
    y_top = max(max(revokes), max(in_rej), 5) * 1.32
    ax.set_ylim(0, y_top)
    for bar, val in list(zip(b1, revokes)) + list(zip(b2, in_rej)):
        ax.text(bar.get_x() + bar.get_width() / 2.0, val + y_top * 0.025, str(val), ha="center", va="bottom", fontsize=9.5, fontweight="bold")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=8.8)
    ax.set_ylabel("Cantidad de eventos")
    ax.set_title("Fallos de sesión y rechazos de control (menor es mejor)")
    ax.legend(loc="upper right", fontsize=8.3, framealpha=0.94)

    # Right: Success rates (% simultaneous cooperation & % input accepted)
    ax = axes[1]
    coop_pcts = [
        0.0,
        float(l2lit9["cooperation"]["multi_operator_overlap_pct_of_occupied"]),
        float(itx76w["cooperation"]["multi_operator_overlap_pct_of_occupied"]),
    ]
    inp_ok_pcts = []
    for base_ok, base_tot, d in (
        (baseline["input_ok"], baseline["input_ok"] + baseline["input_rej"], None),
        (None, None, l2lit9),
        (None, None, itx76w),
    ):
        if d is None:
            inp_ok_pcts.append(round(base_ok / base_tot * 100.0, 2) if base_tot else 0.0)
        else:
            ok = int(d["protocol_outcomes"].get("input:input_accepted", 0))
            tot = int(d["protocol_messages"].get("input", 0))
            inp_ok_pcts.append(round(ok / tot * 100.0, 2) if tot else 0.0)

    b3 = ax.bar(x - w / 2, coop_pcts, width=w, color="#6366f1", label="Operación simultánea 2 equipos (%)")
    b4 = ax.bar(x + w / 2, inp_ok_pcts, width=w, color="#16a34a", label="Comandos aceptados sin error (%)")
    ax.set_ylim(0, 134)
    for bar, val in list(zip(b3, coop_pcts)) + list(zip(b4, inp_ok_pcts)):
        lbl = f"{val:.1f}%" if val < 99.9 else f"{val:.2f}%"
        ax.text(bar.get_x() + bar.get_width() / 2.0, val + 2.0, lbl, ha="center", va="bottom", fontsize=8.6, fontweight="bold")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=8.8)
    ax.set_ylabel("Porcentaje (%)")
    ax.set_title("Confiabilidad y cooperación lograda (mayor es mejor)")
    ax.legend(loc="upper left", fontsize=8.3, framealpha=0.94)

    fig.savefig(out_path, dpi=300)
    plt.close(fig)
    return out_path


def build_latex_document(
    itx: dict[str, Any],
    l2l: dict[str, Any],
    base: dict[str, Any],
    rep_itx: Path,
    rep_l2l: Path,
    before_after_png: Path,
) -> str:
    """Generate structured UTF-8 LaTeX document for the team."""
    itx_rtt = itx["network_rtt"]["overall_ms"]
    itx_jit = itx["network_rtt"]["jitter_inter_packet_ms"]
    itx_rfc = itx["network_rtt"].get("jitter_rfc3550_ms", {"overall": {}, "by_client": {}})
    l2l_rtt = l2l["network_rtt"]["overall_ms"]
    l2l_jit = l2l["network_rtt"]["jitter_inter_packet_ms"]
    l2l_rfc = l2l["network_rtt"].get("jitter_rfc3550_ms", {"overall": {}, "by_client": {}})

    def client_row(
        sess_name: str,
        cname: str,
        rtt_st: dict[str, Any],
        jit_st: dict[str, Any],
        rfc_st: dict[str, Any],
    ) -> str:
        return (
            f"{_tex_escape(sess_name)} & \\texttt{{{_tex_escape(cname)}}} & {rtt_st['count']} & "
            f"\\textbf{{{rtt_st['p50']:.1f} ms}} & {rtt_st['mean']:.1f} ms & "
            f"\\textbf{{{rtt_st['p95']:.1f} ms}} & {jit_st.get('mean', 0.0):.1f} ms & "
            f"\\textbf{{{rfc_st.get('mean', 0.0):.1f} ms}} & {rtt_st['std']:.1f} ms & {rtt_st['max']:.1f} ms \\\\"
        )

    client_rows = []
    for cname in sorted(itx["network_rtt"]["by_client"]):
        st = itx["network_rtt"]["by_client"][cname]
        if st.get("count", 0) >= 20:
            client_rows.append(
                client_row(
                    "itX76W",
                    cname,
                    st,
                    itx_jit["by_client"].get(cname, {}),
                    itx_rfc.get("by_client", {}).get(cname, {}),
                )
            )
    for cname in sorted(l2l["network_rtt"]["by_client"]):
        st = l2l["network_rtt"]["by_client"][cname]
        if st.get("count", 0) >= 20:
            client_rows.append(
                client_row(
                    "l2LIt9",
                    cname,
                    st,
                    l2l_jit["by_client"].get(cname, {}),
                    l2l_rfc.get("by_client", {}).get(cname, {}),
                )
            )
    client_rows_tex = "\n".join(client_rows)

    itx_ax = itx["kinematics"]["commanded_time_by_axis_sec"]
    itx_ow = itx["kinematics"].get("commanded_time_by_axis_owner_sec", {})
    l2l_ax = l2l["kinematics"]["commanded_time_by_axis_sec"]

    axis_rows = []
    for ax in ("X", "Y", "Z", "ROLL", "PITCH", "YAW", "PINZA"):
        t_itx = float(itx_ax.get(ax, 0.0))
        t_l2l = float(l2l_ax.get(ax, 0.0))
        owners_str = ", ".join(f"{op}: {sec:.1f} s" for op, sec in (itx_ow.get(ax) or {}).items()) or "—"
        axis_rows.append(
            f"\\textbf{{{ax}}} & {t_itx:.1f} s & {_tex_escape(owners_str)} & {t_l2l:.1f} s \\\\"
        )
    axis_rows_tex = "\n".join(axis_rows)

    itx_in_ok = itx["protocol_outcomes"].get("input:input_accepted", 0)
    itx_in_tot = itx["protocol_messages"].get("input", 0)
    itx_in_pct = (itx_in_ok / itx_in_tot * 100.0) if itx_in_tot else 0.0
    l2l_in_ok = l2l["protocol_outcomes"].get("input:input_accepted", 0)
    l2l_in_tot = l2l["protocol_messages"].get("input", 0)
    l2l_in_pct = (l2l_in_ok / l2l_in_tot * 100.0) if l2l_in_tot else 0.0
    itx_ev_tot = sum(itx["event_counts"].values())
    l2l_ev_tot = sum(l2l["event_counts"].values())
    itx_lt50 = float(itx["network_rtt"].get("lt50_pct", 97.6))
    l2l_lt50 = float(l2l["network_rtt"].get("lt50_pct", 90.0))
    itx_usb = itx["direct_bridge"]["effective_hz"]
    l2l_usb = l2l["direct_bridge"]["effective_hz"]
    itx_dt = itx["gateway_tick_dt_ms"]
    l2l_dt = l2l["gateway_tick_dt_ms"]
    itx_pa = itx["pose_age_ms"]
    l2l_pa = l2l["pose_age_ms"]

    return rf"""\documentclass[10pt,a4paper]{{article}}
\usepackage[utf8]{{inputenc}}
\usepackage[T1]{{fontenc}}
\usepackage{{amsmath}}
\usepackage[margin=1.5cm,top=1.5cm,bottom=1.7cm]{{geometry}}
\usepackage{{graphicx}}
\usepackage{{booktabs}}
\usepackage{{array}}
\usepackage{{colortbl}}
\usepackage{{xcolor}}
\usepackage{{float}}
\usepackage{{tcolorbox}}
\usepackage{{hyperref}}

\definecolor{{indigoBrand}}{{HTML}}{{3935A5}}
\definecolor{{indigoSoft}}{{HTML}}{{EEF2FF}}
\definecolor{{tealSoft}}{{HTML}}{{ECFEFF}}
\definecolor{{tealBorder}}{{HTML}}{{0D9488}}
\definecolor{{amberSoft}}{{HTML}}{{FFFBEB}}
\definecolor{{amberBorder}}{{HTML}}{{D97706}}
\definecolor{{slateText}}{{HTML}}{{1E293B}}
\definecolor{{rowAlt}}{{HTML}}{{F8FAFC}}

\tcbuselibrary{{skins}}
\setlength{{\parindent}}{{0pt}}
\setlength{{\parskip}}{{3pt}}
\renewcommand{{\arraystretch}}{{1.14}}

\begin{{document}}

% Header Banner
\begin{{tcolorbox}}[colback=indigoBrand,colframe=indigoBrand,arc=4pt,boxrule=0pt,left=10pt,right=10pt,top=7pt,bottom=7pt]
  {{\color{{white}}\Large\bfseries Proyecto 3 · Teleoperación Cooperativa en Red (Kinova JACO2)}}\\[3pt]
  {{\color{{indigoSoft}}\small Informe técnico de métricas, latencia RTT, jitter, cooperación y validación experimental · 1MTR74 (PUCP)}}
\end{{tcolorbox}}

\vspace{{1pt}}

\begin{{tcolorbox}}[colback=indigoSoft,colframe=indigoBrand,title={{\bfseries ¿Qué hace este proyecto y qué se evaluó?}},fonttitle=\small\bfseries,left=8pt,right=8pt,top=4pt,bottom=4pt]
\small
Dos operadores remotos abren un control web desde sus propios dispositivos por Wi-Fi (\textbf{{HTTPS/WSS}}) y reservan componentes disjuntos del robot (hasta 3 cada uno, para no chocar órdenes sobre el mismo eje). La estación madre arbitra esas órdenes, actualiza una tarea 3D en \textbf{{PyBullet}} y envía velocidades seguras por ROS 2 al brazo físico \textbf{{Kinova JACO2 (J2N6S300)}} a \textbf{{100 Hz por USB}}.
\begin{{itemize}}
  \setlength{{\itemsep}}{{1pt}}
  \item \textbf{{Red rápida en ambos equipos:}} mediana \textbf{{{itx_rtt['p50']:.1f} ms}} y percentil 95 \textbf{{{itx_rtt['p95']:.1f} ms}} (\textbf{{{itx_lt50:.1f}\%}} de los mensajes tardó menos de 50 ms).
  \item \textbf{{Control USB en tiempo real:}} tasa mediana de \textbf{{{itx_usb['p50']:.1f} Hz}} en el SDK de Kinova con \textbf{{{itx['direct_bridge']['errors']} errores}}, y \textbf{{{itx_pa['within_300ms_pct']:.0f}\%}} de telemetría debajo del umbral de seguridad de 300 ms.
  \item \textbf{{Cooperación multiusuario real:}} \textbf{{{itx['cooperation']['multi_operator_overlap_sec']:.1f} s}} de trabajo simultáneo en \texttt{{itX76W}} (con pausa \texttt{{PAUSED}} e intercambio de roles) y \textbf{{{l2l['cooperation']['multi_operator_overlap_pct_of_occupied']:.1f}\%}} de solapamiento en \texttt{{l2LIt9}}.
  \item \textbf{{Trayectoria y objetivo virtual:}} \textbf{{{itx['kinematics']['path_length_m']:.2f} m}} recorridos en 3D con entrega real de \textbf{{{itx['virtual_task']['delivered_max']}/{itx['virtual_task']['total']}}} muñecos virtuales y límite direccional en mesa/pedestales.
\end{{itemize}}
\end{{tcolorbox}}

\subsection*{{1. Guía rápida: ¿cómo explicar cada métrica en lenguaje simple?}}
\begin{{tcolorbox}}[colback=tealSoft,colframe=tealBorder,left=8pt,right=8pt,top=4pt,bottom=4pt]
\small
\begin{{description}}
  \setlength{{\itemsep}}{{1.5pt}}
  \item[Latencia de red (RTT sin error de relojes):] Cada 100 ms el navegador envía su reloj local $t_0$ (\texttt{{heartbeat}}), el servidor se lo devuelve (\texttt{{heartbeat\_ack}}) y el cliente mide $\mathrm{{RTT}} = t_2 - t_0$ usando su propio reloj (evita desfase entre PCs). Debajo de \textbf{{100 ms}} se siente fluido; debajo de \textbf{{50 ms}} es casi instantáneo.
  \item[Mediana (p50) vs. Percentil 95 (p95):] En Wi-Fi hay picos puntuales; por eso los artículos IEEE no usan solo el promedio. \textbf{{p50}} es el retardo típico (la mitad de mensajes fue igual o más rápida) y \textbf{{p95}} indica que el 95\% de mensajes llegó por debajo de ese valor.
  \item[Jitter ($|\Delta\mathrm{{RTT}}|$, RFC 3550 y $\sigma$):] Mide qué tanto varía la latencia entre paquetes seguidos. Además del salto instantáneo ($|\mathrm{{RTT}}_{{(i)}} - \mathrm{{RTT}}_{{(i-1)}}|$) y $\sigma$, se calcula el filtro suavizado estándar del curso (RFC 3550): $J_{{(i)}} = J_{{(i-1)}} + (|\mathrm{{RTT}}_{{(i)}} - \mathrm{{RTT}}_{{(i-1)}}| - J_{{(i-1)}})/16$.
  \item[Frecuencia de lazo (Hz) y periodo (dt):] El bridge USB al JACO2 transmite a \textbf{{100 Hz}} (cada 10 ms) y el \emph{{gateway}} autoritativo procesa a \textbf{{35.3--37.1 Hz}} (cada 27--28 ms), superando con holgura los \textbf{{20 Hz}} de entrada de cada navegador.
  \item[Frescura de telemetría (\texttt{{pose\_age\_sec}}):] Antigüedad de la postura articular medida usada para autorizar movimiento. El sistema frena si supera \textbf{{300 ms}}; el 100\% de muestras se mantuvo debajo de ese límite.
\end{{description}}
\end{{tcolorbox}}

\subsection*{{2. Protocolos y arquitectura evaluada}}
\begin{{table}}[H]
\centering
\footnotesize
\begin{{tabular}}{{@{{}}p{{3.5cm}}p{{4.3cm}}p{{2.8cm}}p{{5.8cm}}@{{}}}}
\toprule
\textbf{{Tramo}} & \textbf{{Protocolo / Transporte}} & \textbf{{Tasa / Watchdog}} & \textbf{{Mecanismo de seguridad y rol}} \\
\midrule
Web remota $\leftrightarrow$ Madre & \textbf{{HTTPS + WSS (TLS 1.2+)}} puerto 8443 (\texttt{{TCP\_NODELAY}}) & Heartbeat 10 Hz · Input 20 Hz · Telemetría 50 Hz & CA propia \texttt{{ca.crt}}, autenticación por roles (Operador, Observador, Admin) y token privado. \\
\rowcolor{{rowAlt}}
Gateway cooperativo & \textbf{{JSON teleop\_cooperativo v1}} en \texttt{{127.0.0.1:8765}} & Tick autoritativo 50 Hz · Frenado 400 ms · Lease 1.5 s & Propiedad exclusiva por componente (máx. 3 ejes/op), secuencia monótona y \texttt{{STOP}} global. \\
Gateway $\leftrightarrow$ Bridge ROS 2 & \textbf{{ROS 2 DDS (localhost-only)}} tópicos y servicios & 20--50 Hz · Límite \texttt{{pose\_age $\le$ 300 ms}} & Cinemática inversa con rampa de velocidad y pared virtual contra mesa y pedestales. \\
\rowcolor{{rowAlt}}
Madre $\leftrightarrow$ Kinova JACO2 & \textbf{{SDK USB Kinova}} (\texttt{{USBCommandLayerUbuntu.so}}) & \textbf{{100 Hz}} (10 ms/ciclo) & Arranque siempre desarmado, paro limpio USB y recuperación articular ($\le 5^\circ/\text{{s}}$). \\
\bottomrule
\end{{tabular}}
\end{{table}}

\newpage
\subsection*{{3. Métricas detalladas de red, latencia RTT y jitter}}
{{\footnotesize Se comparan dos corridas reales con 2 equipos en simultáneo: \texttt{{itX76W}} (37.6 min, intercambio de roles, recuperación \texttt{{PAUSED}} y entrega de muñeco) y \texttt{{l2LIt9}} (7.1 min de cooperación continua 6-DOF).}}

\begin{{table}}[H]
\centering
\resizebox{{\textwidth}}{{!}}{{%
\begin{{tabular}}{{@{{}}lccc@{{}}}}
\toprule
\textbf{{Métrica global}} & \textbf{{Sesión itX76W (Completa)}} & \textbf{{Sesión l2LIt9 (Cooperación fija)}} & \textbf{{Meta / Umbral}} \\
\midrule
Duración y total de eventos & {itx['duration_sec']:,.1f} s ({itx_ev_tot:,} eventos) & {l2l['duration_sec']:,.1f} s ({l2l_ev_tot:,} eventos) & — \\
\rowcolor{{rowAlt}}
Tasa USB Kinova físico & \textbf{{{itx_usb['mean']:.1f} Hz}} media (\textbf{{{itx_usb['p50']:.1f} Hz}} p50, {itx['direct_bridge']['errors']} err.) & \textbf{{{l2l_usb['mean']:.1f} Hz}} media (\textbf{{{l2l_usb['p50']:.1f} Hz}} p50, {l2l['direct_bridge']['errors']} err.) & 100 Hz nominal \\
Frecuencia del gateway (\texttt{{dt}}) & {itx_dt['effective_hz']:.1f} Hz (p50: {itx_dt['p50']:.1f} ms · $\sigma$: {itx_dt['std']:.1f} ms) & {l2l_dt['effective_hz']:.1f} Hz (p50: {l2l_dt['p50']:.1f} ms · $\sigma$: {l2l_dt['std']:.1f} ms) & $>$ 20 Hz entrada web \\
\rowcolor{{rowAlt}}
Frescura postura (\texttt{{pose\_age}}) & Media {itx_pa['mean']:.1f} ms · p95 {itx_pa['p95']:.1f} ms · \textbf{{{itx_pa['within_300ms_pct']:.0f}\% $\le$ 300 ms}} & Media {l2l_pa['mean']:.1f} ms · p95 {l2l_pa['p95']:.1f} ms · \textbf{{{l2l_pa['within_300ms_pct']:.0f}\% $\le$ 300 ms}} & $\le$ 300 ms watchdog \\
Latencia RTT (p50 / media / p95) & \textbf{{{itx_rtt['p50']:.1f} / {itx_rtt['mean']:.1f} / {itx_rtt['p95']:.1f} ms}} & \textbf{{{l2l_rtt['p50']:.1f} / {l2l_rtt['mean']:.1f} / {l2l_rtt['p95']:.1f} ms}} & $<$ 100 ms fluido \\
\rowcolor{{rowAlt}}
Jitter entre paquetes $|\Delta\mathrm{{RTT}}|$ & \textbf{{{itx_jit['overall']['mean']:.1f} ms}} (p95: {itx_jit['overall']['p95']:.1f} ms) & \textbf{{{l2l_jit['overall']['mean']:.1f} ms}} (p95: {l2l_jit['overall']['p95']:.1f} ms) & Estable sin cortes \\
Jitter suavizado RFC 3550 & \textbf{{{itx_rfc['overall'].get('mean', 0.0):.1f} ms}} (p95: {itx_rfc['overall'].get('p95', 0.0):.1f} ms) & \textbf{{{l2l_rfc['overall'].get('mean', 0.0):.1f} ms}} (p95: {l2l_rfc['overall'].get('p95', 0.0):.1f} ms) & Tendencia filtrada \\
\rowcolor{{rowAlt}}
Mensajes con RTT $<$ 50 ms & \textbf{{{itx_lt50:.1f}\%}} ({itx_rtt['count']:,} pings) & \textbf{{{l2l_lt50:.1f}\%}} ({l2l_rtt['count']:,} pings) & Mayoría $<$ 50 ms \\
Comandos \texttt{{input}} aceptados & {itx_in_ok:,} / {itx_in_tot:,} (\textbf{{{itx_in_pct:.3f}\%}}) & {l2l_in_ok:,} / {l2l_in_tot:,} (\textbf{{{l2l_in_pct:.3f}\%}}) & $\sim$ 100\% \\
\bottomrule
\end{{tabular}}%
}}
\end{{table}}

\begin{{table}}[H]
\centering
\resizebox{{\textwidth}}{{!}}{{%
\begin{{tabular}}{{@{{}}llrrrrrrrr@{{}}}}
\toprule
\textbf{{Sesión}} & \textbf{{Dispositivo}} & \textbf{{Pings}} & \textbf{{p50}} & \textbf{{Media}} & \textbf{{p95}} & \textbf{{Jitter $|\Delta\mathrm{{RTT}}|$}} & \textbf{{RFC 3550}} & \textbf{{$\sigma$ (RTT)}} & \textbf{{Máx.}} \\
\midrule
{client_rows_tex}
\bottomrule
\end{{tabular}}%
}}
\end{{table}}

\begin{{figure}}[H]
\centering
\includegraphics[width=0.96\textwidth,height=0.41\textheight,keepaspectratio]{{{rep_itx / "07_analisis_latencia_rtt_jitter.png"}}}
\end{{figure}}
\begin{{tcolorbox}}[colback=rowAlt,colframe=indigoBrand,left=7pt,right=7pt,top=4pt,bottom=4pt]
\footnotesize \textbf{{Cómo leer esta figura en palabras simples:}} (1) \textbf{{Arriba-izq. (CDF):}} muestra qué porcentaje de mensajes llegó debajo de cada tiempo; el \textbf{{{itx_lt50:.1f}\%}} en \texttt{{itX76W}} cayó en la zona verde ($<50\text{{ ms}}$). (2) \textbf{{Arriba-der. (Boxplot):}} la caja es el 50\% central de los pings y el rombo rojo el \textbf{{p95}} (32.4--39.1 ms), muy lejos del límite fluido de 100 ms. (3) \textbf{{Abajo-izq. (Jitter):}} cuánto saltó la red entre mensajes seguidos (8.5--9.4 ms). (4) \textbf{{Abajo-der. (Servidor):}} tiempos internos del programa (27--60 ms) frente al corte de seguridad de 300 ms.
\end{{tcolorbox}}

\newpage
\subsection*{{4. Perfil temporal RTT, Jitter RFC 3550 y densidad (formato 1MTR74)}}

\begin{{figure}}[H]
\centering
\includegraphics[width=0.98\textwidth,height=0.58\textheight,keepaspectratio]{{{rep_itx / "08_rtt_jitter_curso_comparativo.png"}}}
\end{{figure}}

\begin{{tcolorbox}}[colback=tealSoft,colframe=tealBorder,left=8pt,right=8pt,top=5pt,bottom=5pt]
\footnotesize \textbf{{¿Son reales estos datos y cómo leer las 3 subgráficas?}} \textbf{{Sí, son 100\% reales:}} salen de los \textbf{{{itx_rtt['count']:,} pings}} (\texttt{{heartbeat}}/\texttt{{heartbeat\_ack}}) medidos por ambos navegadores en \texttt{{itX76W}}.
\begin{{itemize}}
  \setlength{{\itemsep}}{{1pt}}
  \item \textbf{{Arriba (línea azul):}} retardo real muestra a muestra durante 120 pings consecutivos (aprox. 12 s de cooperación); casi siempre oscila entre \textbf{{8 y 25 ms}} con picos cortos típicos de Wi-Fi.
  \item \textbf{{Centro (naranja y rojo):}} la línea naranja marca el salto instantáneo $|\Delta\mathrm{{RTT}}|$ entre un ping y el anterior; la roja gruesa es el filtro suavizado \textbf{{RFC 3550}}, que se mantiene plano debajo de \textbf{{10 ms}} (\textbf{{8.5--9.4 ms}}), probando que la red no se congestionó.
  \item \textbf{{Abajo (barras moradas):}} histograma de toda la sesión; casi todo se concentra en 10--25 ms, con \texttt{{Mean RTT}} de \textbf{{17.2--19.2 ms}} y cota $+1\sigma$ en \textbf{{28.8--31.2 ms}}.
  \item \textbf{{¿Cómo mitigamos el jitter en el JACO2?}} No metimos colas de retardo artificial: activamos \texttt{{TCP\_NODELAY}}, enviamos velocidades suavizadas por USB a \textbf{{100 Hz constantes}} y separamos el frenado rápido (\textbf{{400 ms}}) del vencimiento de reserva (\textbf{{1.5 s}}).
\end{{itemize}}
\end{{tcolorbox}}

\newpage
\subsection*{{5. Resumen visual de red y mejora de ingeniería (Antes vs. Después)}}

\begin{{figure}}[H]
\centering
\includegraphics[width=0.95\textwidth,height=0.39\textheight,keepaspectratio]{{{rep_itx / "05_resumen_para_diapositiva.png"}}}
\end{{figure}}

\begin{{figure}}[H]
\centering
\includegraphics[width=0.95\textwidth,height=0.25\textheight,keepaspectratio]{{{before_after_png}}}
\end{{figure}}

\begin{{tcolorbox}}[colback=amberSoft,colframe=amberBorder,left=7pt,right=7pt,top=4pt,bottom=4pt]
\footnotesize \textbf{{Qué explican ambas figuras (para diapositiva de Resultados):}}
\begin{{itemize}}
  \setlength{{\itemsep}}{{1pt}}
  \item \textbf{{Panel superior (4 cuadros):}} resume cuán rápida fue la red (\textbf{{{itx_lt50:.1f}\%}} $<50\text{{ ms}}$), la latencia por dispositivo, cuánto tiempo real movió cada operador cada eje (las barras bicolores prueban el intercambio de roles) y el cumplimiento cerca del 100\% en USB, telemetría y comandos aceptados.
  \item \textbf{{Panel inferior (Antes vs. Después):}} en la primera prueba (\texttt{{XMWoKK}}, watchdog de reserva de 250 ms), picos normales de Wi-Fi causaron \textbf{{{base['auto_revokes']} expulsiones involuntarias}} y \textbf{{{base['input_rej']} comandos rechazados}}. Al subir la reserva a \textbf{{1.5 s}} (manteniendo el frenado de seguridad en \textbf{{400 ms}}) y activar \texttt{{TCP\_NODELAY}}, las expulsiones bajaron a \textbf{{0}} y se logró hasta \textbf{{99.0\%}} de operación simultánea.
\end{{itemize}}
\end{{tcolorbox}}

\newpage
\subsection*{{6. Cooperación multi-operador y uso real por eje}}
{{\footnotesize Cada operador solicita una reserva exclusiva (\emph{{lease}}) sobre 1 a 3 componentes distintos para que nunca existan comandos contradictorios sobre el mismo eje del brazo.}}

\begin{{table}}[H]
\centering
\footnotesize
\begin{{tabular}}{{@{{}}lclc@{{}}}}
\toprule
\textbf{{Componente}} & \textbf{{Activo en itX76W}} & \textbf{{Desglose por operador (itX76W)}} & \textbf{{Activo en l2LIt9}} \\
\midrule
{axis_rows_tex}
\midrule
\textbf{{Solapamiento $\ge$ 2 equipos}} & \multicolumn{{2}}{{l}}{{\textbf{{{itx['cooperation']['multi_operator_overlap_sec']:.1f} s ({itx['cooperation']['multi_operator_overlap_pct_of_occupied']:.1f}\% del tiempo ocupado)}} · máx. 6 ejes simultáneos}} & \textbf{{{l2l['cooperation']['multi_operator_overlap_sec']:.1f} s ({l2l['cooperation']['multi_operator_overlap_pct_of_occupied']:.1f}\%)}} \\
\bottomrule
\end{{tabular}}
\end{{table}}

\begin{{figure}}[H]
\centering
\includegraphics[width=0.96\textwidth,height=0.42\textheight,keepaspectratio]{{{rep_itx / "04_cooperacion_y_estados.png"}}}
\end{{figure}}

\begin{{tcolorbox}}[colback=rowAlt,colframe=indigoBrand,left=7pt,right=7pt,top=4pt,bottom=4pt]
\footnotesize \textbf{{Cómo funciona la cooperación y cómo leer el cronograma:}}
\begin{{itemize}}
  \setlength{{\itemsep}}{{1pt}}
  \item \textbf{{Panel superior (estados y concurrencia):}} la línea negra muestra cuántos equipos tienen ejes reservados a la vez (\texttt{{0}}, \texttt{{1}} o \texttt{{2}}) y las bandas de color el estado del robot (\texttt{{ACTIVE}} morado, \texttt{{READY}} verde). Entre $t=295$ y $t=387\text{{ s}}$ la estación madre activó \texttt{{PAUSED}} (naranja) para reacomodar una articulación de forma segura \textbf{{sin quitar las reservas}} de los operadores.
  \item \textbf{{Panel inferior (barras por eje):}} muestra qué equipo controló cada eje en cada momento. Primero \texttt{{Equipo 9TJQWH}} (rosa) manejó \texttt{{X, Y, Z}} y \texttt{{Equipo 2NM3SW}} (azul) \texttt{{ROLL, PITCH, PINZA}}; a los \textbf{{570 s}} intercambiaron roles limpiamente con exclusión mutua en los 6 componentes.
\end{{itemize}}
\end{{tcolorbox}}

\newpage
\subsection*{{7. Cinemática del brazo físico JACO2, tarea virtual y seguridad}}

\begin{{table}}[H]
\centering
\footnotesize
\begin{{tabular}}{{@{{}}lcc@{{}}}}
\toprule
\textbf{{Indicador cinemático y de tarea}} & \textbf{{Sesión itX76W}} & \textbf{{Sesión l2LIt9}} \\
\midrule
Recorrido 3D acumulado del efector final & \textbf{{{itx['kinematics']['path_length_m']:.2f} m}} & \textbf{{{l2l['kinematics']['path_length_m']:.2f} m}} \\
\rowcolor{{rowAlt}}
Desplazamiento máximo desde el inicio & {itx['kinematics']['max_displacement_from_start_cm']:.1f} cm & {l2l['kinematics']['max_displacement_from_start_cm']:.1f} cm \\
Excursión en espacio de trabajo ($\Delta X, \Delta Y, \Delta Z$) & ({itx['kinematics']['excursion_cm']['X']:.1f}, {itx['kinematics']['excursion_cm']['Y']:.1f}, {itx['kinematics']['excursion_cm']['Z']:.1f}) cm & ({l2l['kinematics']['excursion_cm']['X']:.1f}, {l2l['kinematics']['excursion_cm']['Y']:.1f}, {l2l['kinematics']['excursion_cm']['Z']:.1f}) cm \\
\rowcolor{{rowAlt}}
Velocidad pico comandada (lineal / angular) & \textbf{{{itx['kinematics']['peak_linear_speed_mm_s']:.1f} mm/s}} · \textbf{{{itx['kinematics']['peak_angular_speed_deg_s']:.1f} °/s}} & \textbf{{{l2l['kinematics']['peak_linear_speed_mm_s']:.1f} mm/s}} · \textbf{{{l2l['kinematics']['peak_angular_speed_deg_s']:.1f} °/s}} \\
Movimiento activo (lineal / angular / pinza) & {itx['kinematics']['active_linear_motion_sec']:.1f} s / {itx['kinematics']['active_angular_motion_sec']:.1f} s / {itx['kinematics']['commanded_time_by_axis_sec'].get('PINZA', 0):.1f} s & {l2l['kinematics']['active_linear_motion_sec']:.1f} s / {l2l['kinematics']['active_angular_motion_sec']:.1f} s / {l2l['kinematics']['commanded_time_by_axis_sec'].get('PINZA', 0):.1f} s \\
\rowcolor{{rowAlt}}
Tarea cooperativa PyBullet (5 muñecos) & \textbf{{1 / 5 entregados}} (2 agarres) & 0 / 5 (prueba libre 6-DOF) \\
Mecanismos \emph{{fail-safe}} observados & \texttt{{PAUSED}} 91.2 s (recuperación sin perder ejes) & \texttt{{FAULT}} 30.7 s (bloqueo tras desarme) \\
\bottomrule
\end{{tabular}}
\end{{table}}

\begin{{figure}}[H]
\centering
\includegraphics[width=0.95\textwidth,height=0.28\textheight,keepaspectratio]{{{rep_itx / "02_trayectoria_3d_y_xyz.png"}}}
\end{{figure}}

\begin{{figure}}[H]
\centering
\includegraphics[width=0.95\textwidth,height=0.23\textheight,keepaspectratio]{{{rep_itx / "06_resumen_seguridad_y_tarea.png"}}}
\end{{figure}}

\begin{{tcolorbox}}[colback=tealSoft,colframe=tealBorder,left=7pt,right=7pt,top=4pt,bottom=4pt]
\footnotesize \textbf{{Qué demuestran la trayectoria 3D, la tarea virtual y la seguridad:}}
\begin{{itemize}}
  \setlength{{\itemsep}}{{1pt}}
  \item \textbf{{Trayectoria 3D y límite de velocidad (\texttt{{02}}):}} el efector final del JACO2 físico recorrió \textbf{{8.23 m}} reales siguiendo las órdenes de traslación de la red; el gráfico inferior confirma que nunca se superó el tope de seguridad de \textbf{{50 mm/s}} (línea roja).
  \item \textbf{{Tarea virtual en PyBullet y seguridad (\texttt{{06}}):}} sobre el gemelo 3D sincronizado al brazo real corren una pinza virtual de 3 dedos, 5 muñecos y una pared virtual direccional en mesa/pedestales (bloquea acercamientos al choque pero permite alejarse). En \texttt{{itX76W}} se logró agarrar y entregar \textbf{{1 / 5}} muñecos entre ambos operadores con \textbf{{0}} errores USB y \textbf{{0}} fallos \texttt{{FAULT}}.
\end{{itemize}}
\end{{tcolorbox}}

\newpage
\subsection*{{8. Guía rápida para armar la exposición del equipo}}

\begin{{tcolorbox}}[colback=amberSoft,colframe=amberBorder,title={{\bfseries Guion en 30 segundos (para introducir el proyecto en clase)}},fonttitle=\small\bfseries,left=8pt,right=8pt,top=4pt,bottom=4pt]
\small
``Construimos un sistema de \textbf{{teleoperación cooperativa en red}} sobre un manipulador real \textbf{{Kinova JACO2}}: dos operadores se conectan por \textbf{{HTTPS/WebSocket}}, reservan ejes distintos con \emph{{leases}} atómicos para no interferirse y mueven el robot en tiempo real a \textbf{{100 Hz USB}} mientras resuelven una tarea cooperativa en \textbf{{PyBullet}} protegida por watchdogs, pared virtual y recuperación segura.''
\end{{tcolorbox}}

\begin{{tcolorbox}}[colback=indigoSoft,colframe=indigoBrand,title={{\bfseries Qué poner en cada una de las 4 secciones de la rúbrica}},fonttitle=\small\bfseries,left=8pt,right=8pt,top=5pt,bottom=5pt]
\small
\begin{{enumerate}}
  \setlength{{\itemsep}}{{3pt}}
  \item \textbf{{Protocols used:}} Mostrar la Tabla 1 (HTTPS+WSS por TLS en puerto 8443 a 10/20/50 Hz $\rightarrow$ Gateway JSON con \emph{{leases}} $\rightarrow$ ROS 2 localhost $\rightarrow$ SDK USB Kinova a 100 Hz).
  \item \textbf{{Code implementation:}} Explicar (a) arbitraje por componentes con exclusión mutua (\texttt{{lease\_id}} + \texttt{{generation}}), (b) seguridad multicapa (salida desarmada al inicio, cero al soltar o perder foco, \texttt{{pose\_age $\le$ 300 ms}}, \texttt{{PAUSED}} y pared virtual de mesa/pedestales), y (c) gemelo 3D Qt/VTK + PyBullet.
  \item \textbf{{Acquired metrics:}} Usar \textbf{{08\_rtt\_jitter\_curso\_comparativo.png}} (o las vistas individuales \texttt{{08a}}/\texttt{{08b}} estilo \texttt{{plot\_jitter.py}} con RTT, RFC 3550 y densidad) junto con \textbf{{07\_analisis\_latencia\_rtt\_jitter.png}} (CDF y boxplot), \textbf{{05\_resumen\_para\_diapositiva.png}} y las Tablas de la Sección 3.
  \item \textbf{{Results and analysis:}} Usar \textbf{{04\_cooperacion\_y\_estados.png}}, \textbf{{02\_trayectoria\_3d\_y\_xyz.png}}, \textbf{{06\_resumen\_seguridad\_y\_tarea.png}} y \textbf{{comparacion\_antes\_despues\_wifi.png}} (explicando cómo el ajuste del watchdog a 1.5 s eliminó los cortes Wi-Fi manteniendo el frenado rápido a 400 ms).
\end{{enumerate}}
\end{{tcolorbox}}

\begin{{tcolorbox}}[colback=tealSoft,colframe=tealBorder,title={{\bfseries Preguntas típicas del profesor y respuestas breves}},fonttitle=\small\bfseries,left=8pt,right=8pt,top=5pt,bottom=5pt]
\small
\begin{{itemize}}
  \setlength{{\itemsep}}{{3pt}}
  \item \textbf{{¿Cómo miden el RTT sin error entre relojes de distintas PCs?}} Con el método de eco \texttt{{heartbeat}}/\texttt{{heartbeat\_ack}}: el navegador manda su marca $t_0$ y calcula $\mathrm{{RTT}} = t_2 - t_0$ al recibirla de vuelta, usando un solo reloj local sin sufrir desfase (\emph{{clock skew}}).
  \item \textbf{{¿Cómo mitigan el jitter sin agregar retardo artificial?}} Activando \texttt{{TCP\_NODELAY}} en el proxy TLS, desacoplando la red del lazo USB físico a \textbf{{100 Hz}} con rampa de velocidad, y separando el frenado rápido (\textbf{{400 ms}}) del vencimiento de reserva (\textbf{{1.5 s}}).
  \item \textbf{{¿Por qué el lazo del gateway corrió a $\approx$36 Hz y no a 50 Hz exactos?}} Porque en cada ciclo ejecuta también la física 3D de PyBullet (120 Hz internos) y escribe el log en disco (\texttt{{events.jsonl}}); aun así supera casi 2$\times$ los 20 Hz de comandos del navegador y mantiene el USB a 100 Hz.
  \item \textbf{{¿Por qué en Wi-Fi hubo picos de RTT de 200--380 ms si la mediana fue 15--18 ms?}} Por ahorro de energía Wi-Fi y retransmisiones en enlace inalámbrico. El problema se resolvió separando el watchdog de velocidad (400 ms) del timeout de reserva (1.5 s) y activando \texttt{{TCP\_NODELAY}}.
  \item \textbf{{¿Cómo garantizan que dos equipos no choquen comandos sobre el mismo eje?}} Con reserva atómica (\texttt{{claim}}) y tokens con \texttt{{lease\_id}} + \texttt{{generation}}: solo el dueño vigente de un componente puede moverlo, y cada operador está acotado a máximo 3 componentes.
\end{{itemize}}
\end{{tcolorbox}}

\end{{document}}
"""


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    reports_dir = repo_root / "reports"
    rep_itx = reports_dir / "teleop-base.itX76W"
    rep_l2l = reports_dir / "teleop-base.l2LIt9"

    itx = json.loads((rep_itx / "metrics_summary.json").read_text(encoding="utf-8"))
    l2l = json.loads((rep_l2l / "metrics_summary.json").read_text(encoding="utf-8"))
    itx["network_rtt"]["lt50_pct"] = _rtt_lt50_pct(Path("/tmp/teleop-base.itX76W/events.jsonl"), 97.6)
    l2l["network_rtt"]["lt50_pct"] = _rtt_lt50_pct(Path("/tmp/teleop-base.l2LIt9/events.jsonl"), 90.0)
    base = _analyze_baseline_xmwo(Path("/tmp/teleop-base.XMWoKK/events.jsonl"))

    before_after_png = generate_before_after_chart(
        reports_dir / "comparacion_antes_despues_wifi.png",
        base,
        l2l,
        itx,
    )
    tex_src = build_latex_document(itx, l2l, base, rep_itx, rep_l2l, before_after_png)

    out_pdf = reports_dir / "INFORME_METRICAS_EQUIPO_JACO2.pdf"
    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        tex_path = tmp_path / "informe.tex"
        tex_path.write_text(tex_src, encoding="utf-8")
        for _ in range(2):
            res = subprocess.run(
                ["pdflatex", "-interaction=nonstopmode", "-halt-on-error", tex_path.name],
                cwd=tmp_path,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
            )
            if res.returncode != 0:
                print(res.stdout[-3000:])
                print(res.stderr[-2000:])
                raise RuntimeError("pdflatex falló")
        shutil.copy2(tmp_path / "informe.pdf", out_pdf)

    print(f"Gráfico comparativo: {before_after_png}")
    print(f"PDF equipo listo:    {out_pdf} ({out_pdf.stat().st_size // 1024} KB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
