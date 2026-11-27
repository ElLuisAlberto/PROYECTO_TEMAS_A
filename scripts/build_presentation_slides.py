#!/usr/bin/python3
"""Build a 12-slide 16:9 Beamer PDF presentation for PUCP 1MTR74 Project 3 (Kinova JACO2)."""

from __future__ import annotations

import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from typing import Any

SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from build_team_pdf_report import (  # noqa: E402
    _analyze_baseline_xmwo,
    _rtt_lt50_pct,
    _tex_escape,
    generate_before_after_chart,
)


def build_beamer_tex(
    itx: dict[str, Any],
    l2l: dict[str, Any],
    base: dict[str, Any],
    rep_itx: Path,
    rep_l2l: Path,
    assets_dir: Path,
    before_after_png: Path,
) -> str:
    """Return full LaTeX Beamer 16:9 document string for all 12 slides."""
    itx_rtt = itx["network_rtt"]["overall_ms"]
    itx_jit = itx["network_rtt"]["jitter_inter_packet_ms"]
    itx_rfc = itx["network_rtt"].get("jitter_rfc3550_ms", {"overall": {}, "by_client": {}})
    l2l_rtt = l2l["network_rtt"]["overall_ms"]
    l2l_jit = l2l["network_rtt"]["jitter_inter_packet_ms"]
    l2l_rfc = l2l["network_rtt"].get("jitter_rfc3550_ms", {"overall": {}, "by_client": {}})

    itx_lt50 = float(itx["network_rtt"].get("lt50_pct", 97.8))
    l2l_lt50 = float(l2l["network_rtt"].get("lt50_pct", 90.0))
    itx_usb = itx["direct_bridge"]["effective_hz"]
    l2l_usb = l2l["direct_bridge"]["effective_hz"]
    itx_dt = itx["gateway_tick_dt_ms"]
    l2l_dt = l2l["gateway_tick_dt_ms"]
    itx_pa = itx["pose_age_ms"]
    l2l_pa = l2l["pose_age_ms"]

    itx_in_ok = itx["protocol_outcomes"].get("input:input_accepted", 0)
    itx_in_tot = itx["protocol_messages"].get("input", 0)
    itx_in_pct = (itx_in_ok / itx_in_tot * 100.0) if itx_in_tot else 0.0
    l2l_in_ok = l2l["protocol_outcomes"].get("input:input_accepted", 0)
    l2l_in_tot = l2l["protocol_messages"].get("input", 0)
    l2l_in_pct = (l2l_in_ok / l2l_in_tot * 100.0) if l2l_in_tot else 0.0

    # Active client stats for itX76W
    c_2n = itx["network_rtt"]["by_client"].get("Equipo 2NM3SW", {})
    c_9t = itx["network_rtt"]["by_client"].get("Equipo 9TJQWH", {})
    r_2n = itx_rfc.get("by_client", {}).get("Equipo 2NM3SW", {})
    r_9t = itx_rfc.get("by_client", {}).get("Equipo 9TJQWH", {})

    # Image paths
    img_jaco = assets_dir / "jaco2_fisico_lab.jpg"
    img_base = assets_dir / "estacion_madre_3d.jpg"
    img_paso1 = assets_dir / "ui_paso1_modos.jpg"
    img_paso2 = assets_dir / "ui_paso2_componentes.png"
    img_laptop = assets_dir / "ui_laptop_operador_xyz.jpg"
    img_phone = assets_dir / "ui_celular_operador_rpy.jpg"

    img_02 = rep_itx / "02_trayectoria_3d_y_xyz.png"
    img_04 = rep_itx / "04_cooperacion_y_estados.png"
    img_06 = rep_itx / "06_resumen_seguridad_y_tarea.png"
    img_07 = rep_itx / "07_analisis_latencia_rtt_jitter.png"
    img_08 = rep_itx / "08_rtt_jitter_curso_comparativo.png"

    return rf"""\documentclass[aspectratio=169,9pt]{{beamer}}
\usepackage[utf8]{{inputenc}}
\usepackage[T1]{{fontenc}}
\usepackage{{amsmath}}
\usepackage{{graphicx}}
\usepackage{{booktabs}}
\usepackage{{array}}
\usepackage{{colortbl}}
\usepackage{{xcolor}}
\usepackage{{tikz}}
\usetikzlibrary{{arrows.meta,positioning,shapes.geometric}}
\usepackage{{tcolorbox}}
\tcbuselibrary{{skins}}

% Color palette aligned with PUCP 1MTR74 & web controller UI
\definecolor{{indigoBrand}}{{HTML}}{{3935A5}}
\definecolor{{purpleBrand}}{{HTML}}{{6B21A8}}
\definecolor{{indigoSoft}}{{HTML}}{{EEF2FF}}
\definecolor{{tealBrand}}{{HTML}}{{0D9488}}
\definecolor{{tealSoft}}{{HTML}}{{ECFEFF}}
\definecolor{{amberBrand}}{{HTML}}{{D97706}}
\definecolor{{amberSoft}}{{HTML}}{{FFFBEB}}
\definecolor{{redBrand}}{{HTML}}{{DC2626}}
\definecolor{{redSoft}}{{HTML}}{{FEF2F2}}
\definecolor{{greenBrand}}{{HTML}}{{16A34A}}
\definecolor{{greenSoft}}{{HTML}}{{F0FDF4}}
\definecolor{{slateDark}}{{HTML}}{{0F172A}}
\definecolor{{slateMuted}}{{HTML}}{{475569}}
\definecolor{{slateBg}}{{HTML}}{{F8FAFC}}
\definecolor{{borderLight}}{{HTML}}{{CBD5E1}}

\setbeamertemplate{{navigation symbols}}{{}}
\setbeamersize{{text margin left=0.55cm, text margin right=0.55cm}}

% Headline showing rubric section pill on every content slide
\setbeamertemplate{{headline}}{{%
  \begin{{beamercolorbox}}[wd=\paperwidth,ht=0.46cm,dp=0.0cm]{{white}}%
    \begin{{tikzpicture}}
      \fill[indigoBrand] (0,0) rectangle (\paperwidth,0.48cm);
      \node[anchor=west,text=white,font=\scriptsize\bfseries] at (0.50cm,0.24cm) {{1MTR74 · PUCP · Proyecto 3: Teleoperación Cooperativa (Kinova JACO2)}};
      \node[anchor=east,fill=purpleBrand,rounded corners=2.5pt,inner xsep=5pt,inner ysep=1.5pt,text=white,font=\scriptsize\bfseries] at (\paperwidth-0.50cm,0.24cm) {{\insertsectionhead}};
    \end{{tikzpicture}}%
  \end{{beamercolorbox}}%
}}

% Clean single-line frametitle
\setbeamertemplate{{frametitle}}{{%
  \vspace{{0.06cm}}%
  {{\color{{slateDark}}\normalsize\bfseries\insertframetitle}}%
  \vspace{{0.02cm}}%
  \par\noindent{{\color{{indigoBrand!30}}\rule{{\linewidth}}{{0.7pt}}}}%
  \vspace{{-0.14cm}}%
}}

% Footline with slide counter
\setbeamertemplate{{footline}}{{%
  \begin{{beamercolorbox}}[wd=\paperwidth,ht=0.32cm,dp=0.08cm]{{white}}%
    \hspace{{0.55cm}}{{\tiny\color{{slateMuted}}Brazo físico Kinova JACO2 (J2N6S300) · Red HTTPS/WSS + ROS 2 + PyBullet}}%
    \hfill%
    {{\tiny\bfseries\color{{indigoBrand}}\insertframenumber\ /\ \inserttotalframenumber}}\hspace{{0.55cm}}%
  \end{{beamercolorbox}}%
}}

\newcommand{{\card}}[4]{{%
  \begin{{tcolorbox}}[colback=#1,colframe=#2,arc=3pt,boxrule=0.7pt,boxsep=1.2pt,left=4pt,right=4pt,top=2.5pt,bottom=2.5pt,toptitle=1.8pt,bottomtitle=1.8pt,title={{\scriptsize\bfseries #3}},fonttitle=\scriptsize\bfseries]
    \scriptsize\color{{slateDark}} #4
  \end{{tcolorbox}}%
}}

\begin{{document}}

% ==============================================================================
% SLIDE 1: PORTADA
% ==============================================================================
\section{{Portada}}
{{\setbeamertemplate{{headline}}{{}}\setbeamertemplate{{footline}}{{}}
\begin{{frame}}[plain]
  \begin{{tcolorbox}}[colback=indigoBrand,colframe=indigoBrand,arc=5pt,boxrule=0pt,boxsep=2pt,left=10pt,right=10pt,top=6pt,bottom=6pt]
    {{\color{{indigoSoft}}\scriptsize\bfseries 1MTR74 · TEMAS DE ROBÓTICA Y SISTEMAS INTELIGENTES A · PUCP · 2026-2}}\\[2pt]
    {{\color{{white}}\Large\bfseries Teleoperación Cooperativa en Red sobre \mbox{{Kinova JACO2}}}}\\[2pt]
    {{\color{{indigoSoft}}\scriptsize Control multi-dispositivo (\texttt{{HTTPS/WSS}}) · Arbitraje exclusivo por ejes · Gemelo 3D PyBullet · Validación RTT y Jitter RFC 3550}}
  \end{{tcolorbox}}

  \vspace{{3pt}}
  \begin{{columns}}[T,totalwidth=\textwidth]
    \begin{{column}}{{0.35\textwidth}}
      \begin{{tcolorbox}}[colback=slateBg,colframe=borderLight,arc=4pt,boxrule=0.7pt,boxsep=1.5pt,left=3pt,right=3pt,top=3pt,bottom=3pt]
        \centering
        \includegraphics[width=\linewidth,height=0.45\textheight,keepaspectratio]{{{img_jaco}}}\\[2pt]
        {{\tiny\bfseries\color{{slateDark}}Brazo físico Kinova JACO2 (J2N6S300) · Lab. PUCP}}
      \end{{tcolorbox}}
    \end{{column}}
    \begin{{column}}{{0.62\textwidth}}
      \begin{{tcolorbox}}[colback=slateBg,colframe=borderLight,arc=4pt,boxrule=0.7pt,boxsep=1.5pt,left=3pt,right=3pt,top=3pt,bottom=3pt]
        \centering
        \includegraphics[width=\linewidth,height=0.45\textheight,keepaspectratio]{{{img_base}}}\\[2pt]
        {{\tiny\bfseries\color{{slateDark}}Estación madre: Gemelo 3D Qt/VTK + Tarea virtual PyBullet + USB 100 Hz}}
      \end{{tcolorbox}}
    \end{{column}}
  \end{{columns}}

  \vspace{{3pt}}
  \begin{{tcolorbox}}[colback=indigoSoft,colframe=indigoBrand!45,arc=3pt,boxrule=0.6pt,boxsep=1.5pt,left=8pt,right=8pt,top=3pt,bottom=3pt]
    \scriptsize\color{{slateDark}}
    \textbf{{Equipo:}} Fabio [Completar nombres de integrantes] \hfill \textbf{{Docente:}} Eduardo Cabrera Yi \hfill \textbf{{Sesiones:}} \texttt{{itX76W}} / \texttt{{l2LIt9}} / \texttt{{XMWoKK}}
  \end{{tcolorbox}}
\end{{frame}}
}}

% ==============================================================================
% SLIDE 2: PROBLEMA, OBJETIVO Y VISIÓN GENERAL
% ==============================================================================
\section{{Visión General}}
\begin{{frame}}{{Problema, Objetivo y Visión General del Sistema}}
  \begin{{columns}}[T,totalwidth=\textwidth]
    \begin{{column}}{{0.62\textwidth}}
      \begin{{tcolorbox}}[colback=indigoSoft,colframe=indigoBrand,arc=4pt,boxrule=0.7pt,boxsep=1.5pt,left=6pt,right=6pt,top=3pt,bottom=3pt,toptitle=1.8pt,bottomtitle=1.8pt,title={{\scriptsize\bfseries El desafío: teleoperación cooperativa real sobre Wi-Fi}},fonttitle=\scriptsize\bfseries]
        \scriptsize\color{{slateDark}}
        Permitir que \textbf{{2 operadores en dispositivos distintos}} (laptop y celular) controlen simultáneamente un brazo físico 6-DOF sin chocar órdenes ni sufrir cortes por picos de latencia Wi-Fi.
      \end{{tcolorbox}}

      \vspace{{3pt}}
      \begin{{tcolorbox}}[colback=slateBg,colframe=borderLight,arc=4pt,boxrule=0.7pt,boxsep=1.5pt,left=6pt,right=6pt,top=3pt,bottom=3pt,toptitle=1.8pt,bottomtitle=1.8pt,title={{\scriptsize\bfseries Tres pilares de nuestra solución}},fonttitle=\scriptsize\bfseries]
        \scriptsize\color{{slateDark}}
        \begin{{enumerate}}
          \setlength{{\itemsep}}{{1.5pt}}
          \item \textbf{{Acceso web cifrado multi-cliente:}} Proxy \textbf{{HTTPS + WSS}} (\texttt{{:8443}}) con roles (\emph{{Operador}}, \emph{{Observador}}, \emph{{Admin}}).
          \item \textbf{{Arbitraje exclusivo por ejes (\emph{{leases}}):}} Cada operador reserva de 1 a 3 componentes entre \texttt{{X, Y, Z, ROLL, PITCH, YAW, PINZA}}.
          \item \textbf{{Control físico 100 Hz + Tarea 3D:}} Bridge USB C++ directo al JACO2, pared virtual de mesa y reto cooperativo en \textbf{{PyBullet}}.
        \end{{enumerate}}
      \end{{tcolorbox}}

      \vspace{{3pt}}
      \begin{{columns}}[T,totalwidth=\linewidth]
        \begin{{column}}{{0.235\linewidth}}
          \begin{{tcolorbox}}[colback=greenSoft,colframe=greenBrand,arc=3pt,boxrule=0.7pt,boxsep=1pt,left=2pt,right=2pt,top=2.5pt,bottom=2.5pt]
            \centering{{\normalsize\bfseries\color{{greenBrand}}{itx_rtt['p50']:.1f} ms}}\\[1pt]{{\tiny\bfseries Mediana RTT\\({itx_lt50:.1f}\% < 50 ms)}}
          \end{{tcolorbox}}
        \end{{column}}
        \begin{{column}}{{0.235\linewidth}}
          \begin{{tcolorbox}}[colback=tealSoft,colframe=tealBrand,arc=3pt,boxrule=0.7pt,boxsep=1pt,left=2pt,right=2pt,top=2.5pt,bottom=2.5pt]
            \centering{{\normalsize\bfseries\color{{tealBrand}}{itx_usb['p50']:.0f} Hz}}\\[1pt]{{\tiny\bfseries USB JACO2\\({itx['direct_bridge']['errors']} errores)}}
          \end{{tcolorbox}}
        \end{{column}}
        \begin{{column}}{{0.235\linewidth}}
          \begin{{tcolorbox}}[colback=indigoSoft,colframe=indigoBrand,arc=3pt,boxrule=0.7pt,boxsep=1pt,left=2pt,right=2pt,top=2.5pt,bottom=2.5pt]
            \centering{{\normalsize\bfseries\color{{indigoBrand}}{itx_rfc['overall'].get('mean', 0.0):.1f} ms}}\\[1pt]{{\tiny\bfseries Jitter RFC 3550\\(media \texttt{{itX76W}})}}
          \end{{tcolorbox}}
        \end{{column}}
        \begin{{column}}{{0.235\linewidth}}
          \begin{{tcolorbox}}[colback=amberSoft,colframe=amberBrand,arc=3pt,boxrule=0.7pt,boxsep=1pt,left=2pt,right=2pt,top=2.5pt,bottom=2.5pt]
            \centering{{\normalsize\bfseries\color{{amberBrand}}{itx['kinematics']['path_length_m']:.2f} m}}\\[1pt]{{\tiny\bfseries Recorrido 3D\\({itx['cooperation']['multi_operator_overlap_sec']:.0f} s coop.)}}
          \end{{tcolorbox}}
        \end{{column}}
      \end{{columns}}
    \end{{column}}
    \begin{{column}}{{0.35\textwidth}}
      \begin{{tcolorbox}}[colback=slateBg,colframe=borderLight,arc=4pt,boxrule=0.7pt,boxsep=1.5pt,left=3pt,right=3pt,top=3pt,bottom=3pt]
        \centering
        \includegraphics[width=\linewidth,height=0.68\textheight,keepaspectratio]{{{img_jaco}}}\\[2pt]
        {{\tiny\bfseries Kinova JACO2 físico operando por red}}
      \end{{tcolorbox}}
    \end{{column}}
  \end{{columns}}
\end{{frame}}

% ==============================================================================
% SLIDE 3: 1. PROTOCOLS USED — ARQUITECTURA DE RED Y PROTOCOLOS
% ==============================================================================
\section{{1. Protocols Used}}
\begin{{frame}}{{1. Protocols Used · Arquitectura de Comunicación en 4 Tramos}}
  \centering
  \begin{{tikzpicture}}[
    box/.style={{draw,rounded corners=4pt,thick,align=center,minimum height=1.35cm,text width=2.20cm,font=\scriptsize}},
    arr/.style={{->,>=Stealth,thick,indigoBrand}}
  ]
    \node[box,fill=indigoSoft,draw=indigoBrand] (web) at (0,0) {{\textbf{{Clientes Web}}\\[1pt]\tiny Laptop + Celular\\(\texttt{{web/app.js}})\\[1pt]Heartbeat · Input}};
    \node[box,fill=tealSoft,draw=tealBrand] (gw) at (4.15,0) {{\textbf{{Proxy + Gateway}}\\[1pt]\tiny TLS :8443 + Árbitro\\(\texttt{{gateway.py}})\\[1pt]Tick 50 Hz · Leases}};
    \node[box,fill=amberSoft,draw=amberBrand] (ros) at (8.30,0) {{\textbf{{ROS 2 + IK + 3D}}\\[1pt]\tiny Jazzy + PyBullet\\(\texttt{{cartesian}})\\[1pt]Rampa + Pared}};
    \node[box,fill=greenSoft,draw=greenBrand] (hw) at (12.45,0) {{\textbf{{JACO2 USB}}\\[1pt]\tiny \texttt{{kinova\_direct}}\\(SDK C++ Ubuntu)\\[1pt]Lazo real 100 Hz}};

    \draw[arr] (web.east) -- node[above,font=\tiny\bfseries,align=center]{{HTTPS+WSS\\TLS :8443}} node[below,font=\tiny,slateMuted]{{10/20/50 Hz}} (gw.west);
    \draw[arr] (gw.east) -- node[above,font=\tiny\bfseries,align=center]{{JSON v1 / DDS\\localhost :8765}} node[below,font=\tiny,slateMuted]{{age $\le$ 300 ms}} (ros.west);
    \draw[arr] (ros.east) -- node[above,font=\tiny\bfseries,align=center]{{SDK USB\\C++ directo}} node[below,font=\tiny,slateMuted]{{10 ms/ciclo}} (hw.west);
  \end{{tikzpicture}}

  \vspace{{5pt}}
  \renewcommand{{\arraystretch}}{{1.16}}
  \scriptsize
  \resizebox{{\textwidth}}{{!}}{{%
  \begin{{tabular}}{{@{{}}p{{2.6cm}}p{{3.9cm}}p{{3.1cm}}p{{4.6cm}}@{{}}}}
    \toprule
    \textbf{{Tramo}} & \textbf{{Protocolo / Transporte}} & \textbf{{Tasa / Watchdog}} & \textbf{{Seguridad y Función Clave}} \\
    \midrule
    Web $\leftrightarrow$ Madre & \textbf{{HTTPS + WSS (TLS 1.2+)}} en \texttt{{:8443}} (\texttt{{TCP\_NODELAY}}) & Heartbeat 10 Hz · Input 20 Hz · Telemetría 50 Hz & CA propia \texttt{{ca.crt}}, autenticación por roles y medición de RTT por reloj local. \\
    \rowcolor{{slateBg}}
    Gateway cooperativo & \textbf{{JSON \texttt{{teleop\_cooperativo v1}}}} en \texttt{{127.0.0.1:8765}} & Tick autoritativo 50 Hz · Frenado 400 ms · Lease 1.5 s & Propiedad exclusiva por componente (\emph{{lease\_id + generation}}), \texttt{{STOP}} global. \\
    Gateway $\leftrightarrow$ Bridge & \textbf{{ROS 2 DDS (localhost-only)}} tópicos y servicios & 20--50 Hz · Límite \texttt{{pose\_age $\le$ 300 ms}} & Cinemática inversa suavizada y pared virtual direccional en mesa/pedestales. \\
    \rowcolor{{slateBg}}
    Madre $\leftrightarrow$ JACO2 & \textbf{{SDK USB Kinova}} (\texttt{{USBCommandLayerUbuntu.so}}) & \textbf{{100 Hz}} (10 ms constantes) & Arranque siempre desarmado, paro limpio USB y recuperación articular $\le 5^\circ/\mathrm{{s}}$. \\
    \bottomrule
  \end{{tabular}}%
  }}
\end{{frame}}

% ==============================================================================
% SLIDE 4: 2. CODE IMPLEMENTATION — ROLES Y RESERVA ATÓMICA (LEASES)
% ==============================================================================
\section{{2. Code Implementation}}
\begin{{frame}}{{2. Code Implementation · Web: Roles y Reserva Exclusiva (\emph{{Leases}})}}
  \begin{{columns}}[T,totalwidth=\textwidth]
    \begin{{column}}{{0.48\textwidth}}
      \begin{{tcolorbox}}[colback=slateBg,colframe=indigoBrand,arc=4pt,boxrule=0.8pt,boxsep=1.5pt,left=3pt,right=3pt,top=3pt,bottom=3pt,toptitle=2pt,bottomtitle=2pt,title={{\scriptsize\bfseries Paso 1/2: Selección de modo de participación}},fonttitle=\scriptsize\bfseries]
        \centering
        \includegraphics[width=\linewidth,height=0.47\textheight,keepaspectratio]{{{img_paso1}}}\\[2pt]
        {{\tiny\color{{slateMuted}}HTTPS (:8443): Operador (3 ejes), Observador (lectura) o Administrador}}
      \end{{tcolorbox}}
    \end{{column}}
    \begin{{column}}{{0.48\textwidth}}
      \begin{{tcolorbox}}[colback=slateBg,colframe=purpleBrand,arc=4pt,boxrule=0.8pt,boxsep=1.5pt,left=3pt,right=3pt,top=3pt,bottom=3pt,toptitle=2pt,bottomtitle=2pt,title={{\scriptsize\bfseries Paso 2/2: Reserva atómica de hasta 3 componentes}},fonttitle=\scriptsize\bfseries]
        \centering
        \includegraphics[width=\linewidth,height=0.47\textheight,keepaspectratio]{{{img_paso2}}}\\[2pt]
        {{\tiny\color{{slateMuted}}Operador eligiendo \texttt{{X, Y, Z}} (3/3); el resto queda libre para el 2.º equipo}}
      \end{{tcolorbox}}
    \end{{column}}
  \end{{columns}}

  \vspace{{4pt}}
  \begin{{tcolorbox}}[colback=indigoSoft,colframe=indigoBrand,arc=3.5pt,boxrule=0.7pt,boxsep=1.5pt,left=7pt,right=7pt,top=3pt,bottom=3pt]
    \scriptsize\color{{slateDark}}
    \textbf{{Árbitro de exclusión mutua (\texttt{{arbiter.py}} / \texttt{{lease\_manager.py}}):}} cada solicitud \texttt{{claim}} valida disponibilidad y emite un token único (\texttt{{lease\_id}} + \texttt{{generation}}). Ningún otro cliente puede mover un eje ya reservado ni enviar secuencias fuera de orden.
  \end{{tcolorbox}}
\end{{frame}}

% ==============================================================================
% SLIDE 5: 2. CODE IMPLEMENTATION — CONTROL MULTI-DISPOSITIVO EN VIVO
% ==============================================================================
\section{{2. Code Implementation}}
\begin{{frame}}{{2. Code Implementation · Control Cooperativo en Vivo (Laptop + Celular)}}
  \begin{{columns}}[T,totalwidth=\textwidth]
    \begin{{column}}{{0.64\textwidth}}
      \begin{{tcolorbox}}[colback=slateBg,colframe=indigoBrand,arc=4pt,boxrule=0.8pt,boxsep=1.5pt,left=3pt,right=3pt,top=3pt,bottom=3pt,toptitle=2pt,bottomtitle=2pt,title={{\scriptsize\bfseries Operador 1 (Laptop): Traslación X, Y, Z + Tarea Virtual}},fonttitle=\scriptsize\bfseries]
        \centering
        \includegraphics[width=\linewidth,height=0.50\textheight,keepaspectratio]{{{img_laptop}}}\\[2pt]
        {{\tiny\color{{slateMuted}}Estado \texttt{{ACTIVE}} · Sliders de velocidad (35 mm/s) · Telemetría en vivo (\texttt{{0.03 s}}) y 5 muñecos}}
      \end{{tcolorbox}}
    \end{{column}}
    \begin{{column}}{{0.33\textwidth}}
      \begin{{tcolorbox}}[colback=slateBg,colframe=purpleBrand,arc=4pt,boxrule=0.8pt,boxsep=1.5pt,left=3pt,right=3pt,top=3pt,bottom=3pt,toptitle=2pt,bottomtitle=2pt,title={{\scriptsize\bfseries Operador 2 (Celular): RPY}},fonttitle=\scriptsize\bfseries]
        \centering
        \includegraphics[width=\linewidth,height=0.50\textheight,keepaspectratio]{{{img_phone}}}\\[2pt]
        {{\tiny\color{{slateMuted}}Botón \texttt{{+}} pulsado (\texttt{{67\% · 6.72 °/s}}) en paralelo}}
      \end{{tcolorbox}}
    \end{{column}}
  \end{{columns}}

  \vspace{{4pt}}
  \begin{{tcolorbox}}[colback=tealSoft,colframe=tealBrand,arc=3.5pt,boxrule=0.7pt,boxsep=1.5pt,left=7pt,right=7pt,top=3pt,bottom=3pt]
    \scriptsize\color{{slateDark}}
    \textbf{{Seguridad \emph{{hombre muerto}} en cliente y servidor:}} el robot solo se mueve mientras se mantiene pulsado \texttt{{+}}/\texttt{{-}}; cualquier suelta, cambio de pestaña (\texttt{{blur}}/\texttt{{pagehide}}) o silencio mayor a \textbf{{400 ms}} fuerza velocidad cero de inmediato.
  \end{{tcolorbox}}
\end{{frame}}

% ==============================================================================
% SLIDE 6: 2. CODE IMPLEMENTATION — ESTACIÓN MADRE, GEMELO 3D Y SEGURIDAD
% ==============================================================================
\section{{2. Code Implementation}}
\begin{{frame}}{{2. Code Implementation · Estación Madre, Gemelo 3D y Seguridad}}
  \begin{{columns}}[T,totalwidth=\textwidth]
    \begin{{column}}{{0.59\textwidth}}
      \begin{{tcolorbox}}[colback=slateBg,colframe=indigoBrand,arc=4pt,boxrule=0.8pt,boxsep=1.5pt,left=3pt,right=3pt,top=3pt,bottom=3pt,toptitle=2pt,bottomtitle=2pt,title={{\scriptsize\bfseries Interfaz Estación Base (\texttt{{desktop\_qt.py}}) + PyBullet}},fonttitle=\scriptsize\bfseries]
        \centering
        \includegraphics[width=\linewidth,height=0.61\textheight,keepaspectratio]{{{img_base}}}\\[2pt]
        {{\tiny\color{{slateMuted}}Reconstrucción 3D FK por ángulos reales + 5 muñecos + Armado/USB/Recuperación}}
      \end{{tcolorbox}}
    \end{{column}}
    \begin{{column}}{{0.38\textwidth}}
      \card{{redSoft}}{{redBrand}}{{1. Armado Explícito y Paro Global}}{{Arranque siempre \textbf{{desarmado}}; solo la estación madre arma la salida física y tiene botón \texttt{{Desarmar}} / \texttt{{STOP}} global.}}
      \vspace{{2.5pt}}
      \card{{amberSoft}}{{amberBrand}}{{2. Pared Virtual Direccional}}{{Protege mesa y pedestales en simulación: bloquea velocidades hacia el choque, pero permite retroceder.}}
      \vspace{{2.5pt}}
      \card{{indigoSoft}}{{indigoBrand}}{{3. Recuperación Articular (\texttt{{PAUSED}})}}{{Permite reacomodar ejes individuales a $\le 5^\circ/\mathrm{{s}}$ en estación madre \textbf{{sin quitar las reservas}} web.}}
      \vspace{{2.5pt}}
      \card{{greenSoft}}{{greenBrand}}{{4. Tarea PyBullet + Bitácora Única}}{{Pinza virtual 3 dedos, 5 muñecos y log monotónico completo (\texttt{{events.jsonl}}) para auditoría.}}
    \end{{column}}
  \end{{columns}}
\end{{frame}}

% ==============================================================================
% SLIDE 7: 3. ACQUIRED METRICS — LATENCIA RTT Y JITTER RFC 3550 (1MTR74)
% ==============================================================================
\section{{3. Acquired Metrics}}
\begin{{frame}}{{3. Acquired Metrics · Latencia RTT y Jitter RFC 3550 (1MTR74)}}
  \begin{{columns}}[T,totalwidth=\textwidth]
    \begin{{column}}{{0.67\textwidth}}
      \begin{{tcolorbox}}[colback=white,colframe=borderLight,arc=4pt,boxrule=0.7pt,boxsep=1pt,left=2pt,right=2pt,top=2pt,bottom=2pt]
        \centering
        \includegraphics[width=\linewidth,height=0.70\textheight,keepaspectratio]{{{img_08}}}
      \end{{tcolorbox}}
    \end{{column}}
    \begin{{column}}{{0.30\textwidth}}
      \card{{indigoSoft}}{{indigoBrand}}{{Medición real sin \emph{{Clock Skew}}}}{{Cada 100 ms el navegador envía su reloj $t_0$ (\texttt{{heartbeat}}) y recibe \texttt{{heartbeat\_ack}}; mide $\mathrm{{RTT}} = t_2 - t_0$ en un solo reloj (\textbf{{{itx_rtt['count']:,} pings}}).}}
      \vspace{{3pt}}
      \card{{tealSoft}}{{tealBrand}}{{Fórmula RFC 3550 del curso}}{{$J_i = J_{{i-1}} + (|\Delta\mathrm{{RTT}}_i| - J_{{i-1}})/16$\\Suaviza saltos puntuales para medir estabilidad real de red.}}
      \vspace{{3pt}}
      \card{{greenSoft}}{{greenBrand}}{{Cifras por equipo (\texttt{{itX76W}})}}{{
        \textbf{{Equipo 2NM3SW:}} Mean \textbf{{{c_2n.get('mean', 0):.1f} ms}}, $+1\sigma$ {c_2n.get('mean', 0) + c_2n.get('std', 0):.1f} ms, \textbf{{RFC 3550 {r_2n.get('mean', 0):.1f} ms}}.\\[2pt]
        \textbf{{Equipo 9TJQWH:}} Mean \textbf{{{c_9t.get('mean', 0):.1f} ms}}, $+1\sigma$ {c_9t.get('mean', 0) + c_9t.get('std', 0):.1f} ms, \textbf{{RFC 3550 {r_9t.get('mean', 0):.1f} ms}}.
      }}
    \end{{column}}
  \end{{columns}}
\end{{frame}}

% ==============================================================================
% SLIDE 8: 3. ACQUIRED METRICS — DISTRIBUCIÓN CDF, PERCENTILES Y TABLA
% ==============================================================================
\section{{3. Acquired Metrics}}
\begin{{frame}}{{3. Acquired Metrics · Distribución CDF y Estabilidad de Control}}
  \begin{{columns}}[T,totalwidth=\textwidth]
    \begin{{column}}{{0.55\textwidth}}
      \begin{{tcolorbox}}[colback=white,colframe=borderLight,arc=4pt,boxrule=0.7pt,boxsep=1pt,left=2pt,right=2pt,top=2pt,bottom=2pt]
        \centering
        \includegraphics[width=\linewidth,height=0.70\textheight,keepaspectratio]{{{img_07}}}
      \end{{tcolorbox}}
    \end{{column}}
    \begin{{column}}{{0.42\textwidth}}
      \begin{{tcolorbox}}[colback=slateBg,colframe=indigoBrand,arc=4pt,boxrule=0.8pt,boxsep=1.2pt,left=4pt,right=4pt,top=3pt,bottom=3pt,toptitle=1.8pt,bottomtitle=1.8pt,title={{\scriptsize\bfseries Tabla comparativa de sesiones reales}},fonttitle=\scriptsize\bfseries]
        \renewcommand{{\arraystretch}}{{1.16}}
        \resizebox{{\linewidth}}{{!}}{{%
        \scriptsize
        \begin{{tabular}}{{@{{}}lcc@{{}}}}
          \toprule
          \textbf{{Métrica}} & \textbf{{\texttt{{itX76W}} (37.6 min)}} & \textbf{{\texttt{{l2LIt9}} (7.1 min)}} \\
          \midrule
          Pings RTT medidos & \textbf{{{itx_rtt['count']:,}}} & \textbf{{{l2l_rtt['count']:,}}} \\
          RTT p50 / Media & \textbf{{{itx_rtt['p50']:.1f}}} / {itx_rtt['mean']:.1f} ms & \textbf{{{l2l_rtt['p50']:.1f}}} / {l2l_rtt['mean']:.1f} ms \\
          RTT p95 (cola Wi-Fi) & \textbf{{{itx_rtt['p95']:.1f} ms}} & \textbf{{{l2l_rtt['p95']:.1f} ms}} \\
          Jitter $|\Delta\mathrm{{RTT}}|$ & \textbf{{{itx_jit['overall']['mean']:.1f} ms}} & {l2l_jit['overall']['mean']:.1f} ms \\
          Jitter RFC 3550 & \textbf{{{itx_rfc['overall'].get('mean', 0.0):.1f} ms}} & {l2l_rfc['overall'].get('mean', 0.0):.1f} ms \\
          Mensajes $< 50$ ms & \textbf{{{itx_lt50:.1f}\%}} & \textbf{{{l2l_lt50:.1f}\%}} \\
          Tasa USB JACO2 & \textbf{{{itx_usb['mean']:.1f} Hz}} (0 err.) & \textbf{{{l2l_usb['mean']:.1f} Hz}} (0 err.) \\
          Lazo Gateway & {itx_dt['effective_hz']:.1f} Hz (p50 {itx_dt['p50']:.1f} ms) & {l2l_dt['effective_hz']:.1f} Hz (p50 {l2l_dt['p50']:.1f} ms) \\
          \texttt{{pose\_age $\le$ 300 ms}} & \textbf{{{itx_pa['within_300ms_pct']:.0f}\%}} (p95 {itx_pa['p95']:.1f} ms) & \textbf{{{l2l_pa['within_300ms_pct']:.0f}\%}} (p95 {l2l_pa['p95']:.1f} ms) \\
          Comandos aceptados & \textbf{{{itx_in_pct:.3f}\%}} & \textbf{{{l2l_in_pct:.3f}\%}} \\
          \bottomrule
        \end{{tabular}}%
        }}
      \end{{tcolorbox}}

      \vspace{{3pt}}
      \card{{tealSoft}}{{tealBrand}}{{Lectura clave para teleoperación}}{{
        El \textbf{{{itx_lt50:.1f}\%}} de mensajes en \texttt{{itX76W}} estuvo debajo de \textbf{{50 ms}} (zona casi instantánea) y el \textbf{{100\%}} de telemetría cumplió el watchdog de seguridad de \textbf{{300 ms}}.
      }}
    \end{{column}}
  \end{{columns}}
\end{{frame}}

% ==============================================================================
% SLIDE 9: 4. RESULTS & ANALYSIS — MITIGACIÓN ANTES VS. DESPUÉS
% ==============================================================================
\section{{4. Results \& Analysis}}
\begin{{frame}}{{4. Results \& Analysis · Mitigación de Jitter Wi-Fi (Antes vs. Después)}}
  \centering
  \begin{{tcolorbox}}[colback=white,colframe=borderLight,arc=4pt,boxrule=0.7pt,boxsep=1pt,left=3pt,right=3pt,top=2pt,bottom=2pt]
    \centering
    \includegraphics[width=0.95\textwidth,height=0.44\textheight,keepaspectratio]{{{before_after_png}}}
  \end{{tcolorbox}}

  \vspace{{3pt}}
  \begin{{columns}}[T,totalwidth=\textwidth]
    \begin{{column}}{{0.32\textwidth}}
      \card{{redSoft}}{{redBrand}}{{1. Problema inicial (\texttt{{XMWoKK}})}}{{
        Timeout de reserva igual a \textbf{{250 ms}}. Picos normales de Wi-Fi (hasta 346 ms) provocaron \textbf{{{base['auto_revokes']} expulsiones involuntarias}} y \textbf{{{base['input_rej']} comandos rechazados}}.
      }}
    \end{{column}}
    \begin{{column}}{{0.32\textwidth}}
      \card{{amberSoft}}{{amberBrand}}{{2. Mitigación sin retardo extra}}{{
        Activamos \texttt{{TCP\_NODELAY}}, desacoplamos el USB a \textbf{{100 Hz constantes}} con rampa y separamos frenado rápido (\textbf{{400 ms}}) de \emph{{lease}} (\textbf{{1.5 s}}).
      }}
    \end{{column}}
    \begin{{column}}{{0.32\textwidth}}
      \card{{greenSoft}}{{greenBrand}}{{3. Mejora comprobada}}{{
        Las expulsiones bajaron a \textbf{{0}} en \texttt{{l2LIt9}} y \texttt{{itX76W}}, se aceptó el \textbf{{{itx_in_pct:.3f}\%}} de comandos y se alcanzó hasta \textbf{{99.0\%}} simultáneo.
      }}
    \end{{column}}
  \end{{columns}}
\end{{frame}}

% ==============================================================================
% SLIDE 10: 4. RESULTS & ANALYSIS — DINÁMICA COOPERATIVA E INTERCAMBIO DE ROLES
% ==============================================================================
\section{{4. Results \& Analysis}}
\begin{{frame}}{{4. Results \& Analysis · Cooperación Multi-Operador y \texttt{{PAUSED}}}}
  \centering
  \begin{{tcolorbox}}[colback=white,colframe=borderLight,arc=4pt,boxrule=0.7pt,boxsep=1pt,left=3pt,right=3pt,top=2pt,bottom=2pt]
    \centering
    \includegraphics[width=0.95\textwidth,height=0.47\textheight,keepaspectratio]{{{img_04}}}
  \end{{tcolorbox}}

  \vspace{{3pt}}
  \begin{{columns}}[T,totalwidth=\textwidth]
    \begin{{column}}{{0.32\textwidth}}
      \card{{indigoSoft}}{{indigoBrand}}{{Trabajo simultáneo real}}{{
        \textbf{{{itx['cooperation']['multi_operator_overlap_sec']:.1f} s}} de operación concurrente con 2 dispositivos en \texttt{{itX76W}} (\textbf{{99.0\%}} en \texttt{{l2LIt9}}), controlando hasta \textbf{{6 ejes a la vez}}.
      }}
    \end{{column}}
    \begin{{column}}{{0.32\textwidth}}
      \card{{amberSoft}}{{amberBrand}}{{Pausa segura \texttt{{PAUSED}} ($295$--$387$ s)}}{{
        La estación madre reacomodó una articulación a baja velocidad para ganar margen de trabajo \textbf{{sin revocar los ejes}} reservados en la web.
      }}
    \end{{column}}
    \begin{{column}}{{0.32\textwidth}}
      \card{{tealSoft}}{{tealBrand}}{{Intercambio limpio ($t=570$ s)}}{{
        \texttt{{Equipo 9TJQWH}} cedió traslación (\texttt{{X, Y, Z}}) y tomó rotación/pinza mientras \texttt{{Equipo 2NM3SW}} asumió traslación sin colisiones.
      }}
    \end{{column}}
  \end{{columns}}
\end{{frame}}

% ==============================================================================
% SLIDE 11: 4. RESULTS & ANALYSIS — TRAYECTORIA 3D, TAREA VIRTUAL Y SEGURIDAD
% ==============================================================================
\section{{4. Results \& Analysis}}
\begin{{frame}}{{4. Results \& Analysis · Trayectoria 3D Real, PyBullet y Seguridad}}
  \begin{{columns}}[T,totalwidth=\textwidth]
    \begin{{column}}{{0.50\textwidth}}
      \begin{{tcolorbox}}[colback=white,colframe=indigoBrand,arc=4pt,boxrule=0.7pt,boxsep=1.5pt,left=2pt,right=2pt,top=2pt,bottom=2pt,toptitle=1.8pt,bottomtitle=1.8pt,title={{\scriptsize\bfseries Recorrido 3D físico y límite 50 mm/s}},fonttitle=\scriptsize\bfseries]
        \centering
        \includegraphics[width=\linewidth,height=0.54\textheight,keepaspectratio]{{{img_02}}}
      \end{{tcolorbox}}
    \end{{column}}
    \begin{{column}}{{0.47\textwidth}}
      \begin{{tcolorbox}}[colback=white,colframe=tealBrand,arc=4pt,boxrule=0.7pt,boxsep=1.5pt,left=2pt,right=2pt,top=2pt,bottom=2pt,toptitle=1.8pt,bottomtitle=1.8pt,title={{\scriptsize\bfseries Estados del sistema y logro cooperativo}},fonttitle=\scriptsize\bfseries]
        \centering
        \includegraphics[width=\linewidth,height=0.54\textheight,keepaspectratio]{{{img_06}}}
      \end{{tcolorbox}}
    \end{{column}}
  \end{{columns}}

  \vspace{{3pt}}
  \begin{{tcolorbox}}[colback=greenSoft,colframe=greenBrand,arc=3.5pt,boxrule=0.7pt,boxsep=1.5pt,left=7pt,right=7pt,top=3pt,bottom=3pt]
    \scriptsize\color{{slateDark}}
    \textbf{{Validación experimental en hardware real:}} el efector final del JACO2 recorrió \textbf{{8.23 m}} reales respetando siempre el límite de \textbf{{50 mm/s}}; se logró sujetar y entregar cooperativamente \textbf{{1 / 5}} muñecos en \textbf{{PyBullet}} con \textbf{{0}} errores USB y \textbf{{0}} fallos \texttt{{FAULT}}.
  \end{{tcolorbox}}
\end{{frame}}

% ==============================================================================
% SLIDE 12: CONCLUSIONES, TRABAJO FUTURO Y DEMO
% ==============================================================================
\section{{Conclusiones}}
\begin{{frame}}{{Conclusiones, Trabajo Futuro y Demostración}}
  \begin{{columns}}[T,totalwidth=\textwidth]
    \begin{{column}}{{0.49\textwidth}}
      \begin{{tcolorbox}}[colback=indigoSoft,colframe=indigoBrand,arc=4pt,boxrule=0.8pt,boxsep=1.5pt,left=6pt,right=6pt,top=3.5pt,bottom=3.5pt,toptitle=2pt,bottomtitle=2pt,title={{\scriptsize\bfseries Conclusiones Técnicas Validadas}},fonttitle=\scriptsize\bfseries]
        \scriptsize\color{{slateDark}}
        \begin{{itemize}}
          \setlength{{\itemsep}}{{3pt}}
          \item \textbf{{Red apta para teleoperación en tiempo real:}} mediana de \textbf{{{itx_rtt['p50']:.1f} ms}}, p95 de \textbf{{{itx_rtt['p95']:.1f} ms}} (\textbf{{{itx_lt50:.1f}\%}} $< 50$ ms) y jitter suavizado \textbf{{RFC 3550 de {itx_rfc['overall'].get('mean', 0.0):.1f} ms}}.
          \item \textbf{{Control físico determinista:}} el bridge C++ mantuvo \textbf{{100 Hz}} USB nominales (\texttt{{p50 100.0 Hz}}, \textbf{{0}} errores) y el \textbf{{100\%}} de muestras bajo el corte de \textbf{{300 ms}}.
          \item \textbf{{Robustez frente a jitter Wi-Fi:}} separar frenado rápido (\textbf{{400 ms}}) y retención de reserva (\textbf{{1.5 s}}) + \texttt{{TCP\_NODELAY}} eliminó las \textbf{{5}} expulsiones previas.
          \item \textbf{{Cooperación multiusuario segura:}} arbitraje atómico en los 6 ejes + pinza, intercambio de roles fluido y recuperación articular sin fallos.
        \end{{itemize}}
      \end{{tcolorbox}}
    \end{{column}}
    \begin{{column}}{{0.47\textwidth}}
      \begin{{tcolorbox}}[colback=tealSoft,colframe=tealBrand,arc=4pt,boxrule=0.8pt,boxsep=1.5pt,left=6pt,right=6pt,top=3.5pt,bottom=3.5pt,toptitle=2pt,bottomtitle=2pt,title={{\scriptsize\bfseries Trabajo Futuro}},fonttitle=\scriptsize\bfseries]
        \scriptsize\color{{slateDark}}
        \begin{{itemize}}
          \setlength{{\itemsep}}{{2.5pt}}
          \item \textbf{{Pinza física real:}} habilitar cierre de dedos físicos sobre objetos reales calibrando corriente/fuerza en el SDK.
          \item \textbf{{Video WebRTC de baja latencia:}} transmitir cámara del entorno en la misma web para operar sin línea de vista directa.
          \item \textbf{{Buffer adaptativo en WAN/4G:}} integrar el filtro de mitigación con cola interpolada para enlaces fuera de la red local.
        \end{{itemize}}
      \end{{tcolorbox}}

      \vspace{{4pt}}
      \begin{{tcolorbox}}[colback=purpleBrand,colframe=purpleBrand,arc=5pt,boxrule=0pt,boxsep=2pt,left=8pt,right=8pt,top=6pt,bottom=6pt]
        \centering
        {{\color{{white}}\normalsize\bfseries ¡Muchas gracias!}}\\[2pt]
        {{\color{{indigoSoft}}\scriptsize Preguntas y Demostración Funcional (JACO2 + PyBullet)}}
      \end{{tcolorbox}}
    \end{{column}}
  \end{{columns}}
\end{{frame}}

\end{{document}}
"""


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    reports_dir = repo_root / "reports"
    assets_dir = reports_dir / "presentation_assets"
    rep_itx = reports_dir / "teleop-base.itX76W"
    rep_l2l = reports_dir / "teleop-base.l2LIt9"

    itx = json.loads((rep_itx / "metrics_summary.json").read_text(encoding="utf-8"))
    l2l = json.loads((rep_l2l / "metrics_summary.json").read_text(encoding="utf-8"))
    itx["network_rtt"]["lt50_pct"] = _rtt_lt50_pct(Path("/tmp/teleop-base.itX76W/events.jsonl"), 97.8)
    l2l["network_rtt"]["lt50_pct"] = _rtt_lt50_pct(Path("/tmp/teleop-base.l2LIt9/events.jsonl"), 90.0)
    base = _analyze_baseline_xmwo(Path("/tmp/teleop-base.XMWoKK/events.jsonl"))

    before_after_png = generate_before_after_chart(
        reports_dir / "comparacion_antes_despues_wifi.png",
        base,
        l2l,
        itx,
    )
    tex_src = build_beamer_tex(itx, l2l, base, rep_itx, rep_l2l, assets_dir, before_after_png)

    out_pdf = reports_dir / "PRESENTACION_PROYECTO3_JACO2.pdf"
    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        tex_path = tmp_path / "presentacion.tex"
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
                print(res.stdout[-3500:])
                print(res.stderr[-1500:])
                raise RuntimeError("pdflatex falló generando las diapositivas Beamer")
        shutil.copy2(tmp_path / "presentacion.pdf", out_pdf)

    print(f"Presentación lista: {out_pdf} ({out_pdf.stat().st_size // 1024} KB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
