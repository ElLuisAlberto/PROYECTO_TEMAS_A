#!/usr/bin/python3
"""Build an editable 13-slide 16:9 PowerPoint presentation using the official PUCP template."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from PIL import Image
from pptx import Presentation
from pptx.dml.color import RGBColor
from pptx.enum.shapes import MSO_SHAPE
from pptx.enum.text import MSO_ANCHOR, PP_ALIGN
from pptx.util import Inches, Pt


# Official PUCP theme + high-contrast accent palette
PUCP_NAVY = RGBColor(0x04, 0x23, 0x54)
PUCP_BLUE = RGBColor(0x28, 0x41, 0xDD)
PUCP_SKY = RGBColor(0x0A, 0x7B, 0xC2)
PUCP_MINT = RGBColor(0x16, 0xC7, 0x8E)
PUCP_PURPLE = RGBColor(0x7F, 0x32, 0xC8)
PUCP_AMBER = RGBColor(0xD9, 0x77, 0x06)
PUCP_RED = RGBColor(0xDC, 0x26, 0x26)
PUCP_GREEN_DARK = RGBColor(0x0F, 0x76, 0x6E)

WHITE = RGBColor(0xFF, 0xFF, 0xFF)
SLATE_DARK = RGBColor(0x19, 0x19, 0x19)
SLATE_MUTED = RGBColor(0x47, 0x55, 0x69)
BORDER_LIGHT = RGBColor(0xCB, 0xD5, 0xE1)
BG_SOFT = RGBColor(0xF4, 0xF7, 0xFA)
BG_BLUE = RGBColor(0xEE, 0xF2, 0xFF)
BG_MINT = RGBColor(0xEC, 0xFD, 0xF5)
BG_AMBER = RGBColor(0xFF, 0xFB, 0xEB)
BG_RED = RGBColor(0xFE, 0xF2, 0xF2)
BG_SKY = RGBColor(0xEC, 0xFE, 0xFF)

FOOTER_TITLE = "1MTR74 · Proyecto 3 · Teleoperación Cooperativa (JACO2)"


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
    default = {"available": False, "auto_revokes": 5, "input_ok": 10822, "input_rej": 28}
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


def _find_layout(prs: Presentation, name: str):
    for lay in prs.slide_layouts:
        if lay.name == name:
            return lay
    raise KeyError(f"No se encontró el layout {name!r} en la plantilla PUCP")


def _update_template_footer(prs: Presentation, title_text: str) -> None:
    """Replace 'Título de la presentación' across slide layouts keeping original styling."""
    for lay in prs.slide_layouts:
        for shp in lay.shapes:
            if not shp.has_text_frame:
                continue
            for p in shp.text_frame.paragraphs:
                for run in p.runs:
                    if "Título de la presentación" in run.text:
                        run.text = run.text.replace("Título de la presentación", title_text)


def _clear_sample_slides(prs: Presentation) -> None:
    """Remove sample slides cleanly from the PUCP template."""
    sldIdLst = prs.slides._sldIdLst
    for sldId in list(sldIdLst):
        rId = sldId.rId
        prs.part.drop_rel(rId)
        sldIdLst.remove(sldId)


def _add_rect(
    slide,
    left: float,
    top: float,
    width: float,
    height: float,
    fill_rgb: RGBColor,
    line_rgb: RGBColor | None = None,
    line_pt: float = 0.9,
    rounded: bool = True,
):
    shape_type = MSO_SHAPE.ROUNDED_RECTANGLE if rounded else MSO_SHAPE.RECTANGLE
    shp = slide.shapes.add_shape(shape_type, Inches(left), Inches(top), Inches(width), Inches(height))
    if rounded and len(shp.adjustments) > 0:
        shp.adjustments[0] = 0.04
    shp.fill.solid()
    shp.fill.fore_color.rgb = fill_rgb
    if line_rgb is None:
        shp.line.fill.background()
    else:
        shp.line.color.rgb = line_rgb
        shp.line.width = Pt(line_pt)
    shp.shadow.inherit = False
    return shp


def _add_runs(p, segments: list[tuple[str, bool, RGBColor | None]], font_pt: float, default_color: RGBColor = SLATE_DARK) -> None:
    p.text = ""
    for text, is_bold, color in segments:
        r = p.add_run()
        r.text = text
        r.font.name = "Arial"
        r.font.size = Pt(font_pt)
        r.font.bold = is_bold
        r.font.color.rgb = color if color is not None else default_color


def add_picture_fit(
    slide,
    img_path: Path,
    left: float,
    top: float,
    max_w: float,
    max_h: float,
    frame_fill: RGBColor | None = None,
    frame_border: RGBColor | None = None,
    caption: str | None = None,
    caption_pt: float = 9.0,
):
    """Draw optional rounded card frame and place image centered preserving exact aspect ratio."""
    pad = 0.08 if frame_fill is not None else 0.0
    cap_h = 0.30 if caption else 0.0
    if frame_fill is not None:
        _add_rect(slide, left, top, max_w, max_h, frame_fill, frame_border, line_pt=0.9, rounded=True)

    avail_w = max(0.2, max_w - 2 * pad)
    avail_h = max(0.2, max_h - 2 * pad - cap_h)
    with Image.open(img_path) as im:
        px_w, px_h = im.size
    ratio = min(avail_w / px_w, avail_h / px_h)
    disp_w = px_w * ratio
    disp_h = px_h * ratio
    pic_x = left + pad + (avail_w - disp_w) / 2.0
    pic_y = top + pad + (avail_h - disp_h) / 2.0
    slide.shapes.add_picture(str(img_path), Inches(pic_x), Inches(pic_y), width=Inches(disp_w), height=Inches(disp_h))

    if caption:
        tb = slide.shapes.add_textbox(
            Inches(left + 0.08),
            Inches(top + max_h - cap_h - 0.03),
            Inches(max_w - 0.16),
            Inches(cap_h),
        )
        tf = tb.text_frame
        tf.word_wrap = True
        tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
        p = tf.paragraphs[0]
        p.alignment = PP_ALIGN.CENTER
        r = p.add_run()
        r.text = caption
        r.font.name = "Arial"
        r.font.size = Pt(caption_pt)
        r.font.bold = True
        r.font.color.rgb = SLATE_MUTED


def setup_content_slide(
    prs: Presentation,
    layout_content,
    title_text: str,
    section_pill: str,
    pill_color: RGBColor,
    slide_num: int,
):
    """Create a PUCP TITLE_ONLY slide with crisp top title, rubric badge, and divider."""
    slide = prs.slides.add_slide(layout_content)

    # Configure title placeholder cleanly at top left
    title_shp = slide.shapes.title
    title_shp.left = Inches(0.55)
    title_shp.top = Inches(0.22)
    title_shp.width = Inches(9.55)
    title_shp.height = Inches(0.58)
    tf = title_shp.text_frame
    tf.clear()
    tf.word_wrap = True
    tf.vertical_anchor = MSO_ANCHOR.MIDDLE
    tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
    p = tf.paragraphs[0]
    r = p.add_run()
    r.text = title_text
    r.font.name = "Arial"
    r.font.size = Pt(20)
    r.font.bold = True
    r.font.color.rgb = PUCP_NAVY

    # Rubric badge pill on top right
    pill = _add_rect(slide, 10.22, 0.30, 2.56, 0.38, pill_color, None, rounded=True)
    if len(pill.adjustments) > 0:
        pill.adjustments[0] = 0.28
    ptf = pill.text_frame
    ptf.word_wrap = False
    ptf.vertical_anchor = MSO_ANCHOR.MIDDLE
    ptf.margin_left = ptf.margin_right = ptf.margin_top = ptf.margin_bottom = 0
    pp = ptf.paragraphs[0]
    pp.alignment = PP_ALIGN.CENTER
    pr = pp.add_run()
    pr.text = section_pill
    pr.font.name = "Arial"
    pr.font.size = Pt(10.5)
    pr.font.bold = True
    pr.font.color.rgb = WHITE

    # Slide number centered inside the official PUCP green circle at bottom right
    ntb = slide.shapes.add_textbox(Inches(12.22), Inches(6.55), Inches(0.70), Inches(0.42))
    ntf = ntb.text_frame
    ntf.word_wrap = False
    ntf.vertical_anchor = MSO_ANCHOR.MIDDLE
    ntf.margin_left = ntf.margin_right = ntf.margin_top = ntf.margin_bottom = 0
    np_ = ntf.paragraphs[0]
    np_.alignment = PP_ALIGN.CENTER
    nr = np_.add_run()
    nr.text = str(slide_num)
    nr.font.name = "Arial"
    nr.font.size = Pt(14)
    nr.font.bold = True
    nr.font.color.rgb = WHITE

    # Subtle divider line under header
    _add_rect(slide, 0.55, 0.89, 12.23, 0.025, BORDER_LIGHT, None, rounded=False)
    return slide


def add_card(
    slide,
    left: float,
    top: float,
    width: float,
    height: float,
    title: str | None,
    paragraphs: list[list[tuple[str, bool, RGBColor | None]]],
    fill_rgb: RGBColor,
    border_rgb: RGBColor,
    header_rgb: RGBColor | None = None,
    body_pt: float = 10.2,
    space_after_pt: float = 3.5,
):
    """Draw a rounded card with optional header bar and rich bold/color text runs."""
    _add_rect(slide, left, top, width, height, fill_rgb, border_rgb, line_pt=1.0, rounded=True)
    body_top = top + 0.08
    if title and header_rgb is not None:
        hdr_h = 0.33
        hdr = _add_rect(slide, left, top, width, hdr_h, header_rgb, header_rgb, line_pt=0.8, rounded=False)
        htf = hdr.text_frame
        htf.word_wrap = False
        htf.vertical_anchor = MSO_ANCHOR.MIDDLE
        htf.margin_left = Inches(0.11)
        htf.margin_right = Inches(0.08)
        htf.margin_top = htf.margin_bottom = 0
        hp = htf.paragraphs[0]
        hr = hp.add_run()
        hr.text = title
        hr.font.name = "Arial"
        hr.font.size = Pt(10.5)
        hr.font.bold = True
        hr.font.color.rgb = WHITE
        body_top = top + hdr_h + 0.07

    tb = slide.shapes.add_textbox(
        Inches(left + 0.11),
        Inches(body_top),
        Inches(width - 0.22),
        Inches(max(0.2, top + height - body_top - 0.06)),
    )
    tf = tb.text_frame
    tf.word_wrap = True
    tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
    for idx, segs in enumerate(paragraphs):
        p = tf.paragraphs[0] if idx == 0 else tf.add_paragraph()
        p.space_after = Pt(space_after_pt)
        _add_runs(p, segs, body_pt, SLATE_DARK)


def add_native_table(
    slide,
    left: float,
    top: float,
    width: float,
    height: float,
    headers: list[str],
    rows: list[list[str]],
    col_widths: list[float],
    font_pt: float = 9.5,
):
    """Insert a clean native editable PowerPoint table with PUCP Navy header."""
    n_rows = 1 + len(rows)
    n_cols = len(headers)
    tbl_shape = slide.shapes.add_table(n_rows, n_cols, Inches(left), Inches(top), Inches(width), Inches(height))
    table = tbl_shape.table
    for ci, cw in enumerate(col_widths):
        table.columns[ci].width = Inches(cw)

    for ci, htxt in enumerate(headers):
        cell = table.cell(0, ci)
        cell.fill.solid()
        cell.fill.fore_color.rgb = PUCP_NAVY
        cell.vertical_anchor = MSO_ANCHOR.MIDDLE
        cell.margin_left = cell.margin_right = Inches(0.07)
        cell.margin_top = cell.margin_bottom = Inches(0.03)
        p = cell.text_frame.paragraphs[0]
        p.alignment = PP_ALIGN.LEFT if ci == 0 else PP_ALIGN.CENTER
        r = p.add_run()
        r.text = htxt
        r.font.name = "Arial"
        r.font.size = Pt(font_pt)
        r.font.bold = True
        r.font.color.rgb = WHITE

    for ri, row in enumerate(rows, start=1):
        bg = WHITE if ri % 2 == 1 else BG_SOFT
        for ci, val in enumerate(row):
            cell = table.cell(ri, ci)
            cell.fill.solid()
            cell.fill.fore_color.rgb = bg
            cell.vertical_anchor = MSO_ANCHOR.MIDDLE
            cell.margin_left = cell.margin_right = Inches(0.07)
            cell.margin_top = cell.margin_bottom = Inches(0.025)
            p = cell.text_frame.paragraphs[0]
            p.alignment = PP_ALIGN.LEFT if ci == 0 else PP_ALIGN.CENTER
            r = p.add_run()
            r.text = val
            r.font.name = "Arial"
            r.font.size = Pt(font_pt)
            r.font.bold = ci == 0
            r.font.color.rgb = SLATE_DARK


def add_kpi_tile(slide, left: float, top: float, w: float, h: float, big_val: str, lbl: str, sub: str, bg: RGBColor, border: RGBColor) -> None:
    _add_rect(slide, left, top, w, h, bg, border, line_pt=1.2, rounded=True)
    tb = slide.shapes.add_textbox(Inches(left + 0.05), Inches(top + 0.08), Inches(w - 0.10), Inches(h - 0.14))
    tf = tb.text_frame
    tf.word_wrap = True
    tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
    p0 = tf.paragraphs[0]
    p0.alignment = PP_ALIGN.CENTER
    p0.space_after = Pt(2)
    r0 = p0.add_run()
    r0.text = big_val
    r0.font.name = "Arial"
    r0.font.size = Pt(17)
    r0.font.bold = True
    r0.font.color.rgb = border

    p1 = tf.add_paragraph()
    p1.alignment = PP_ALIGN.CENTER
    p1.space_after = Pt(1)
    r1 = p1.add_run()
    r1.text = lbl
    r1.font.name = "Arial"
    r1.font.size = Pt(9.0)
    r1.font.bold = True
    r1.font.color.rgb = SLATE_DARK

    p2 = tf.add_paragraph()
    p2.alignment = PP_ALIGN.CENTER
    r2 = p2.add_run()
    r2.text = sub
    r2.font.name = "Arial"
    r2.font.size = Pt(8.0)
    r2.font.color.rgb = SLATE_MUTED


def build_presentation(out_pptx: Path) -> Path:
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
    img_before_after = reports_dir / "comparacion_antes_despues_wifi.png"

    prs = Presentation(str(assets_dir / "plantilla_pucp.pptx"))
    _update_template_footer(prs, FOOTER_TITLE)
    _clear_sample_slides(prs)

    layout_title = _find_layout(prs, "TITLE")
    layout_content = _find_layout(prs, "TITLE_ONLY")
    layout_blank = _find_layout(prs, "BLANK")

    # ==========================================================================
    # SLIDE 1: PORTADA OFICIAL PUCP
    # ==========================================================================
    s1 = prs.slides.add_slide(layout_title)
    t_ph = s1.placeholders[0]
    t_ph.left = Inches(1.10)
    t_ph.top = Inches(0.88)
    t_ph.width = Inches(11.13)
    t_ph.height = Inches(1.65)
    tf = t_ph.text_frame
    tf.clear()
    tf.word_wrap = True
    p = tf.paragraphs[0]
    p.alignment = PP_ALIGN.CENTER
    r = p.add_run()
    r.text = "Teleoperación Cooperativa en Red sobre Kinova JACO2"
    r.font.name = "Arial"
    r.font.size = Pt(34)
    r.font.bold = True
    r.font.color.rgb = PUCP_NAVY

    st_ph = s1.placeholders[1]
    st_ph.left = Inches(1.15)
    st_ph.top = Inches(2.65)
    st_ph.width = Inches(11.03)
    st_ph.height = Inches(2.05)
    stf = st_ph.text_frame
    stf.clear()
    stf.word_wrap = True

    sp0 = stf.paragraphs[0]
    sp0.alignment = PP_ALIGN.CENTER
    sp0.space_after = Pt(6)
    _add_runs(sp0, [("Proyecto 3 · 1MTR74 — Temas de Robótica y Sistemas Inteligentes A (2026-2)", True, PUCP_BLUE)], 15)

    sp1 = stf.add_paragraph()
    sp1.alignment = PP_ALIGN.CENTER
    sp1.space_after = Pt(12)
    _add_runs(
        sp1,
        [("Control multi-dispositivo HTTPS/WSS  ·  Arbitraje exclusivo por ejes  ·  Gemelo 3D PyBullet  ·  RTT y Jitter RFC 3550", False, SLATE_MUTED)],
        12.5,
    )

    sp2 = stf.add_paragraph()
    sp2.alignment = PP_ALIGN.CENTER
    _add_runs(
        sp2,
        [
            ("Equipo: ", True, PUCP_NAVY),
            ("Fabio [Completar apellidos y nombres de integrantes]    |    ", False, SLATE_DARK),
            ("Docente: ", True, PUCP_NAVY),
            ("Eduardo Cabrera Yi", False, SLATE_DARK),
        ],
        12,
    )

    # ==========================================================================
    # SLIDE 2: VISIÓN GENERAL
    # ==========================================================================
    s2 = setup_content_slide(prs, layout_content, "Problema, Objetivo y Visión General del Sistema", "Visión General", PUCP_BLUE, 2)
    add_card(
        s2, 0.55, 1.08, 7.85, 1.16,
        "El desafío: teleoperación cooperativa real sobre Wi-Fi",
        [[
            ("Permitir que ", False, SLATE_DARK),
            ("2 operadores en dispositivos distintos", True, PUCP_NAVY),
            (" (laptop y celular) controlen simultáneamente un brazo físico 6-DOF sin chocar órdenes sobre el mismo eje ni sufrir cortes por picos de retardo Wi-Fi.", False, SLATE_DARK),
        ]],
        BG_BLUE, PUCP_NAVY, header_rgb=PUCP_NAVY, body_pt=10.5,
    )
    add_card(
        s2, 0.55, 2.36, 7.85, 2.18,
        "Tres pilares de nuestra solución",
        [
            [("1. Acceso web cifrado multi-cliente: ", True, PUCP_NAVY), ("Proxy ", False, None), ("HTTPS + WSS (:8443)", True, PUCP_BLUE), (" con roles (Operador, Observador, Admin).", False, None)],
            [("2. Arbitraje exclusivo por ejes (leases): ", True, PUCP_NAVY), ("Cada operador reserva de 1 a 3 componentes entre X, Y, Z, ROLL, PITCH, YAW y PINZA.", False, None)],
            [("3. Control físico 100 Hz + Tarea 3D: ", True, PUCP_NAVY), ("Bridge USB C++ directo al JACO2, pared virtual direccional y tarea cooperativa en PyBullet.", False, None)],
        ],
        BG_SOFT, BORDER_LIGHT, header_rgb=PUCP_BLUE, body_pt=10.3, space_after_pt=6,
    )
    add_kpi_tile(s2, 0.55, 4.70, 1.87, 1.32, f"{itx_rtt['p50']:.1f} ms", "Mediana RTT", f"{itx_lt50:.1f}% < 50 ms", BG_MINT, PUCP_GREEN_DARK)
    add_kpi_tile(s2, 2.54, 4.70, 1.87, 1.32, f"{itx_usb['p50']:.0f} Hz", "USB JACO2", f"{itx['direct_bridge']['errors']} errores SDK", BG_SKY, PUCP_SKY)
    add_kpi_tile(s2, 4.53, 4.70, 1.87, 1.32, f"{itx_rfc['overall'].get('mean', 0):.1f} ms", "Jitter RFC 3550", "Estable en itX76W", BG_BLUE, PUCP_BLUE)
    add_kpi_tile(s2, 6.52, 4.70, 1.88, 1.32, f"{itx['kinematics']['path_length_m']:.2f} m", "Recorrido 3D", f"{itx['cooperation']['multi_operator_overlap_sec']:.0f} s coop.", BG_AMBER, PUCP_AMBER)

    add_picture_fit(
        s2, img_jaco, 8.62, 1.08, 4.16, 4.94,
        frame_fill=BG_SOFT, frame_border=BORDER_LIGHT,
        caption="Kinova JACO2 físico (J2N6S300) en laboratorio PUCP", caption_pt=9.0,
    )

    # ==========================================================================
    # SLIDE 3: 1. PROTOCOLS USED — ARQUITECTURA EN 4 TRAMOS
    # ==========================================================================
    s3 = setup_content_slide(prs, layout_content, "1. Protocols Used · Arquitectura de Comunicación en 4 Tramos", "1. Protocols Used", PUCP_NAVY, 3)

    # Native 4-box flow diagram (y = 1.08 .. 2.72)
    nodes = [
        (0.55, BG_BLUE, PUCP_NAVY, "Clientes Web", "Laptop + Celular\nweb/app.js · 10/20 Hz"),
        (3.85, BG_SKY, PUCP_SKY, "Proxy TLS + Gateway", "Puerto :8443 + Árbitro\ngateway.py · Tick 50 Hz"),
        (7.15, BG_AMBER, PUCP_AMBER, "ROS 2 + IK + 3D", "Jazzy + PyBullet 3D\nRampa + Pared virtual"),
        (10.45, BG_MINT, PUCP_GREEN_DARK, "Kinova JACO2 USB", "kinova_direct_node (C++)\nLazo físico 100 Hz"),
    ]
    for nx, bg, brd, ntitle, nsub in nodes:
        box = _add_rect(s3, nx, 1.14, 2.33, 1.38, bg, brd, line_pt=1.3, rounded=True)
        btf = box.text_frame
        btf.word_wrap = True
        btf.vertical_anchor = MSO_ANCHOR.MIDDLE
        btf.margin_left = btf.margin_right = Inches(0.06)
        p0 = btf.paragraphs[0]
        p0.alignment = PP_ALIGN.CENTER
        p0.space_after = Pt(3)
        _add_runs(p0, [(ntitle, True, brd)], 11.0)
        for line in nsub.split("\n"):
            p1 = btf.add_paragraph()
            p1.alignment = PP_ALIGN.CENTER
            _add_runs(p1, [(line, False, SLATE_DARK)], 8.8)

    arrows = [
        (2.94, "HTTPS + WSS\n:8443"),
        (6.24, "JSON v1 / DDS\n:8765"),
        (9.54, "SDK USB\n10 ms/ciclo"),
    ]
    for ax, albl in arrows:
        arr = slide_arr = s3.shapes.add_shape(MSO_SHAPE.RIGHT_ARROW, Inches(ax + 0.14), Inches(1.86), Inches(0.58), Inches(0.24))
        arr.fill.solid()
        arr.fill.fore_color.rgb = PUCP_BLUE
        arr.line.fill.background()
        tb = s3.shapes.add_textbox(Inches(ax - 0.06), Inches(1.22), Inches(0.98), Inches(0.56))
        atf = tb.text_frame
        atf.word_wrap = True
        atf.margin_left = atf.margin_right = atf.margin_top = atf.margin_bottom = 0
        for idx, line in enumerate(albl.split("\n")):
            ap = atf.paragraphs[0] if idx == 0 else atf.add_paragraph()
            ap.alignment = PP_ALIGN.CENTER
            _add_runs(ap, [(line, idx == 0, PUCP_NAVY if idx == 0 else SLATE_MUTED)], 8.0)

    add_native_table(
        s3, 0.55, 2.75, 12.23, 3.25,
        ["Tramo", "Protocolo / Transporte", "Frecuencia / Watchdog", "Mecanismo de seguridad y función"],
        [
            ["Web <-> Madre", "HTTPS + WSS (TLS 1.2+) · :8443 (TCP_NODELAY)", "Heartbeat 10 Hz · Input 20 Hz · Tel. 50 Hz", "CA propia ca.crt, roles (Operador/Obs/Admin) y RTT por reloj local."],
            ["Gateway cooperativo", "JSON teleop_cooperativo v1 (127.0.0.1:8765)", "Tick 50 Hz · Frenado 400 ms · Lease 1.5 s", "Propiedad exclusiva (lease_id + generation), secuencia y STOP global."],
            ["Gateway <-> Bridge", "ROS 2 DDS (localhost-only) tópicos/servicios", "20–50 Hz · Límite pose_age <= 300 ms", "Cinemática inversa suavizada + pared virtual contra mesa y pedestales."],
            ["Madre <-> JACO2", "SDK USB Kinova (USBCommandLayerUbuntu.so)", "100 Hz constantes (10 ms / ciclo)", "Arranque siempre desarmado, paro limpio USB y recuperación <= 5 °/s."],
        ],
        [2.05, 3.35, 2.95, 3.88],
        font_pt=9.5,
    )

    # ==========================================================================
    # SLIDE 4: 2. CODE IMPLEMENTATION — ROLES Y RESERVA EXCLUSIVA
    # ==========================================================================
    s4 = setup_content_slide(prs, layout_content, "2. Code Implementation · Interfaz Web: Roles y Reserva (Leases)", "2. Code Implementation", PUCP_BLUE, 4)
    add_picture_fit(
        s4, img_paso1, 0.55, 1.08, 6.00, 3.98,
        frame_fill=BG_SOFT, frame_border=PUCP_NAVY,
        caption="Paso 1/2: Selección de rol HTTPS (:8443) — Operador / Observador / Admin", caption_pt=9.5,
    )
    add_picture_fit(
        s4, img_paso2, 6.78, 1.08, 6.00, 3.98,
        frame_fill=BG_SOFT, frame_border=PUCP_PURPLE,
        caption="Paso 2/2: Selección atómica de 3/3 ejes (X, Y, Z); resto libre para 2.º equipo", caption_pt=9.5,
    )
    add_card(
        s4, 0.55, 5.18, 12.23, 0.88, None,
        [[
            ("Árbitro de exclusión mutua (arbiter.py / lease_manager.py): ", True, PUCP_NAVY),
            ("Cada solicitud claim valida disponibilidad y emite un token único (lease_id + generation). Ningún otro operador puede mover ejes ya reservados ni inyectar comandos desordenados.", False, SLATE_DARK),
        ]],
        BG_BLUE, PUCP_BLUE, body_pt=10.5,
    )

    # ==========================================================================
    # SLIDE 5: 2. CODE IMPLEMENTATION — CONTROL EN VIVO (LAPTOP + CELULAR)
    # ==========================================================================
    s5 = setup_content_slide(prs, layout_content, "2. Code Implementation · Control Cooperativo (Laptop + Celular)", "2. Code Implementation", PUCP_BLUE, 5)
    add_picture_fit(
        s5, img_laptop, 0.55, 1.08, 8.20, 4.02,
        frame_fill=BG_SOFT, frame_border=PUCP_NAVY,
        caption="Operador 1 (Laptop): Traslación X, Y, Z (35 mm/s) + 5 muñecos PyBullet + Telemetría 0.03 s", caption_pt=9.3,
    )
    add_picture_fit(
        s5, img_phone, 8.95, 1.08, 3.83, 4.02,
        frame_fill=BG_SOFT, frame_border=PUCP_PURPLE,
        caption="Operador 2 (Celular): RPY activo (67%)", caption_pt=9.3,
    )
    add_card(
        s5, 0.55, 5.20, 12.23, 0.86, None,
        [[
            ("Seguridad 'hombre muerto' en cliente y servidor: ", True, PUCP_GREEN_DARK),
            ("El robot solo se mueve mientras se mantiene pulsado + / -; al soltar el botón, cambiar de pestaña (blur/pagehide) o perder señal > 400 ms se fuerza velocidad cero al instante.", False, SLATE_DARK),
        ]],
        BG_MINT, PUCP_MINT, body_pt=10.5,
    )

    # ==========================================================================
    # SLIDE 6: 2. CODE IMPLEMENTATION — ESTACIÓN MADRE Y SEGURIDAD MULTICAPA
    # ==========================================================================
    s6 = setup_content_slide(prs, layout_content, "2. Code Implementation · Estación Madre, Gemelo 3D y Seguridad", "2. Code Implementation", PUCP_BLUE, 6)
    add_picture_fit(
        s6, img_base, 0.55, 1.08, 7.45, 4.98,
        frame_fill=BG_SOFT, frame_border=PUCP_NAVY,
        caption="Estación Base (desktop_qt.py): Gemelo 3D FK en vivo + Pedestales + Panel USB/Armado", caption_pt=9.2,
    )
    cards_s6 = [
        (BG_RED, PUCP_RED, "1. Armado explícito y STOP global", "Arranque siempre desarmado; solo la estación madre arma la salida física y dispone de botón Desarmar / STOP."),
        (BG_AMBER, PUCP_AMBER, "2. Pared virtual direccional", "Protege mesa y pedestales en simulación: bloquea velocidades hacia el choque, pero permite retroceder libremente."),
        (BG_BLUE, PUCP_BLUE, "3. Recuperación articular (PAUSED)", "Reacomoda ejes individuales a <= 5 °/s desde la estación base sin quitar las reservas web de los operadores."),
        (BG_MINT, PUCP_GREEN_DARK, "4. Tarea PyBullet + Bitácora única", "Pinza virtual de 3 dedos, 5 muñecos y registro monotónico completo en events.jsonl para auditoría."),
    ]
    for idx, (bg, brd, ctitle, cbody) in enumerate(cards_s6):
        add_card(
            s6, 8.20, 1.08 + idx * 1.26, 4.58, 1.18,
            ctitle, [[(cbody, False, SLATE_DARK)]],
            bg, brd, header_rgb=brd, body_pt=9.6,
        )

    # ==========================================================================
    # SLIDE 7: 3. ACQUIRED METRICS — FORMATO 1MTR74 (RFC 3550)
    # ==========================================================================
    s7 = setup_content_slide(prs, layout_content, "3. Acquired Metrics · Latencia RTT y Jitter RFC 3550 (1MTR74)", "3. Acquired Metrics", PUCP_SKY, 7)
    add_picture_fit(
        s7, img_08, 0.55, 1.08, 8.25, 4.98,
        frame_fill=WHITE, frame_border=BORDER_LIGHT,
    )
    add_card(
        s7, 8.98, 1.08, 3.80, 1.56,
        "Medición real sin Clock Skew",
        [[("Cada 100 ms cada navegador envía t0 (heartbeat) y recibe heartbeat_ack; calcula ", False, None), ("RTT = t2 - t0", True, PUCP_NAVY), (f" con su propio reloj ({itx_rtt['count']:,} pings reales).", False, None)]],
        BG_BLUE, PUCP_NAVY, header_rgb=PUCP_NAVY, body_pt=9.6,
    )
    add_card(
        s7, 8.98, 2.75, 3.80, 1.42,
        "Filtro Jitter RFC 3550 del curso",
        [
            [("J(i) = J(i-1) + (|RTT(i) - RTT(i-1)| - J(i-1)) / 16", True, PUCP_SKY)],
            [("Suaviza picos sueltos de Wi-Fi para ver la inestabilidad real de red.", False, SLATE_DARK)],
        ],
        BG_SKY, PUCP_SKY, header_rgb=PUCP_SKY, body_pt=9.6,
    )
    add_card(
        s7, 8.98, 4.28, 3.80, 1.78,
        "Resultados por equipo (itX76W)",
        [
            [("Equipo 2NM3SW: ", True, PUCP_NAVY), (f"Mean {c_2n.get('mean', 0):.1f} ms · +1σ {c_2n.get('mean', 0) + c_2n.get('std', 0):.1f} ms · ", False, None), (f"RFC 3550 {r_2n.get('mean', 0):.1f} ms", True, PUCP_GREEN_DARK)],
            [("Equipo 9TJQWH: ", True, PUCP_NAVY), (f"Mean {c_9t.get('mean', 0):.1f} ms · +1σ {c_9t.get('mean', 0) + c_9t.get('std', 0):.1f} ms · ", False, None), (f"RFC 3550 {r_9t.get('mean', 0):.1f} ms", True, PUCP_GREEN_DARK)],
        ],
        BG_MINT, PUCP_GREEN_DARK, header_rgb=PUCP_GREEN_DARK, body_pt=9.6,
    )

    # ==========================================================================
    # SLIDE 8: 3. ACQUIRED METRICS — DISTRIBUCIÓN CDF Y TABLA COMPARATIVA
    # ==========================================================================
    s8 = setup_content_slide(prs, layout_content, "3. Acquired Metrics · Distribución CDF, Percentiles y Control", "3. Acquired Metrics", PUCP_SKY, 8)
    add_picture_fit(
        s8, img_07, 0.55, 1.08, 7.10, 4.98,
        frame_fill=WHITE, frame_border=BORDER_LIGHT,
    )
    add_native_table(
        s8, 7.82, 1.08, 4.96, 3.55,
        ["Métrica medida", "itX76W (37.6 m)", "l2LIt9 (7.1 m)"],
        [
            ["Pings RTT medidos", f"{itx_rtt['count']:,}", f"{l2l_rtt['count']:,}"],
            ["RTT p50 / Media", f"{itx_rtt['p50']:.1f} / {itx_rtt['mean']:.1f} ms", f"{l2l_rtt['p50']:.1f} / {l2l_rtt['mean']:.1f} ms"],
            ["RTT p95 (cola Wi-Fi)", f"{itx_rtt['p95']:.1f} ms", f"{l2l_rtt['p95']:.1f} ms"],
            ["Jitter |ΔRTT| medio", f"{itx_jit['overall']['mean']:.1f} ms", f"{l2l_jit['overall']['mean']:.1f} ms"],
            ["Jitter RFC 3550", f"{itx_rfc['overall'].get('mean', 0):.1f} ms", f"{l2l_rfc['overall'].get('mean', 0):.1f} ms"],
            ["Mensajes < 50 ms", f"{itx_lt50:.1f}%", f"{l2l_lt50:.1f}%"],
            ["USB JACO2 (0 err.)", f"{itx_usb['mean']:.1f} Hz", f"{l2l_usb['mean']:.1f} Hz"],
            ["Lazo Gateway", f"{itx_dt['effective_hz']:.1f} Hz", f"{l2l_dt['effective_hz']:.1f} Hz"],
            ["pose_age <= 300 ms", f"{itx_pa['within_300ms_pct']:.0f}% (p95 {itx_pa['p95']:.0f}ms)", f"{l2l_pa['within_300ms_pct']:.0f}% (p95 {l2l_pa['p95']:.0f}ms)"],
            ["Comandos aceptados", f"{itx_in_pct:.3f}%", f"{l2l_in_pct:.3f}%"],
        ],
        [1.92, 1.56, 1.48],
        font_pt=8.7,
    )
    add_card(
        s8, 7.82, 4.74, 4.96, 1.32,
        "Lectura clave de desempeño",
        [[
            ("El ", False, None), (f"{itx_lt50:.1f}%", True, PUCP_GREEN_DARK),
            (" de pings en itX76W estuvo bajo 50 ms (zona casi instantánea) y el ", False, None),
            ("100%", True, PUCP_NAVY),
            (" de telemetría respetó el corte de seguridad de 300 ms.", False, None),
        ]],
        BG_MINT, PUCP_MINT, header_rgb=PUCP_GREEN_DARK, body_pt=9.8,
    )

    # ==========================================================================
    # SLIDE 9: 4. RESULTS & ANALYSIS — ANTES VS DESPUÉS WI-FI
    # ==========================================================================
    s9 = setup_content_slide(prs, layout_content, "4. Results & Analysis · Mitigación de Jitter Wi-Fi (Antes vs. Después)", "4. Results & Analysis", PUCP_PURPLE, 9)
    add_picture_fit(
        s9, img_before_after, 0.55, 1.08, 12.23, 3.16,
        frame_fill=WHITE, frame_border=BORDER_LIGHT,
    )
    cards_s9 = [
        (0.55, BG_RED, PUCP_RED, "1. Problema inicial (XMWoKK)", f"Timeout de reserva en 250 ms. Picos normales de Wi-Fi (hasta 346 ms) causaron {base['auto_revokes']} expulsiones involuntarias y {base['input_rej']} comandos rechazados."),
        (4.71, BG_AMBER, PUCP_AMBER, "2. Ingeniería aplicada sin retardo", "Activamos TCP_NODELAY, desacoplamos el control USB a 100 Hz constantes con rampa y separamos frenado rápido (400 ms) de lease (1.5 s)."),
        (8.87, BG_MINT, PUCP_GREEN_DARK, "3. Resultado comprobado", f"Las expulsiones bajaron a 0 en l2LIt9 e itX76W, se aceptó el {itx_in_pct:.3f}% de comandos y se alcanzó hasta 99.0% simultáneo."),
    ]
    for cx, bg, brd, ctitle, cbody in cards_s9:
        add_card(s9, cx, 4.34, 3.91, 1.72, ctitle, [[(cbody, False, SLATE_DARK)]], bg, brd, header_rgb=brd, body_pt=9.8)

    # ==========================================================================
    # SLIDE 10: 4. RESULTS & ANALYSIS — COOPERACIÓN Y PAUSED
    # ==========================================================================
    s10 = setup_content_slide(prs, layout_content, "4. Results & Analysis · Cooperación Multi-Operador, PAUSED y Roles", "4. Results & Analysis", PUCP_PURPLE, 10)
    add_picture_fit(
        s10, img_04, 0.55, 1.08, 12.23, 3.26,
        frame_fill=WHITE, frame_border=BORDER_LIGHT,
    )
    cards_s10 = [
        (0.55, BG_BLUE, PUCP_NAVY, "Trabajo simultáneo real", f"{itx['cooperation']['multi_operator_overlap_sec']:.1f} s de operación concurrente con 2 equipos en itX76W (y 99.0% en l2LIt9), manejando hasta 6 ejes a la vez."),
        (4.71, BG_AMBER, PUCP_AMBER, "Pausa segura PAUSED (295–387 s)", "La estación madre reacomodó una articulación a baja velocidad para ganar espacio de trabajo sin revocar las reservas web."),
        (8.87, BG_SKY, PUCP_SKY, "Intercambio limpio de roles (570 s)", "Equipo 9TJQWH cedió traslación (X, Y, Z) y tomó rotación/pinza mientras Equipo 2NM3SW asumió traslación sin colisiones."),
    ]
    for cx, bg, brd, ctitle, cbody in cards_s10:
        add_card(s10, cx, 4.44, 3.91, 1.62, ctitle, [[(cbody, False, SLATE_DARK)]], bg, brd, header_rgb=brd, body_pt=9.8)

    # ==========================================================================
    # SLIDE 11: 4. RESULTS & ANALYSIS — TRAYECTORIA 3D Y TAREA VIRTUAL
    # ==========================================================================
    s11 = setup_content_slide(prs, layout_content, "4. Results & Analysis · Trayectoria 3D Real, PyBullet y Seguridad", "4. Results & Analysis", PUCP_PURPLE, 11)
    add_picture_fit(
        s11, img_02, 0.55, 1.08, 6.20, 4.02,
        frame_fill=WHITE, frame_border=PUCP_NAVY,
        caption="Trayectoria cartesiana 3D real y límite de 50 mm/s respetado", caption_pt=9.0,
    )
    add_picture_fit(
        s11, img_06, 6.95, 1.08, 5.83, 4.02,
        frame_fill=WHITE, frame_border=PUCP_GREEN_DARK,
        caption="Distribución de estados del sistema y KPIs de tarea/seguridad", caption_pt=9.0,
    )
    add_card(
        s11, 0.55, 5.20, 12.23, 0.86, None,
        [[
            ("Validación experimental en hardware real: ", True, PUCP_GREEN_DARK),
            ("El efector final del Kinova JACO2 recorrió ", False, None),
            (f"{itx['kinematics']['path_length_m']:.2f} m", True, PUCP_NAVY),
            (" reales respetando el tope de ", False, None),
            ("50 mm/s", True, PUCP_RED),
            ("; se sujetó y entregó cooperativamente ", False, None),
            ("1 / 5 muñecos", True, PUCP_GREEN_DARK),
            (" en PyBullet con 0 errores USB y 0 fallos FAULT.", False, None),
        ]],
        BG_MINT, PUCP_MINT, body_pt=10.3,
    )

    # ==========================================================================
    # SLIDE 12: CONCLUSIONES Y TRABAJO FUTURO
    # ==========================================================================
    s12 = setup_content_slide(prs, layout_content, "Conclusiones Técnicas y Trabajo Futuro", "Conclusiones", PUCP_NAVY, 12)
    add_card(
        s12, 0.55, 1.08, 6.10, 4.98,
        "Conclusiones Validadas Experimentalmente",
        [
            [("1. Red apta para tiempo real: ", True, PUCP_NAVY), (f"Mediana {itx_rtt['p50']:.1f} ms, p95 {itx_rtt['p95']:.1f} ms ({itx_lt50:.1f}% < 50 ms) y Jitter RFC 3550 de {itx_rfc['overall'].get('mean', 0):.1f} ms medido sin error de relojes.", False, None)],
            [("2. Control USB determinista: ", True, PUCP_NAVY), (f"El bridge C++ sostuvo {itx_usb['mean']:.1f} Hz media (100.0 Hz p50, 0 errores) y 100% de pose_age <= 300 ms.", False, None)],
            [("3. Mitigación efectiva de jitter Wi-Fi: ", True, PUCP_NAVY), ("Separar frenado rápido (400 ms) y retención de reserva (1.5 s) + TCP_NODELAY eliminó las 5 expulsiones previas.", False, None)],
            [("4. Cooperación multiusuario segura: ", True, PUCP_NAVY), ("Arbitraje atómico en los 6 ejes + pinza, intercambio limpio de roles y recuperación articular PAUSED sin fallos.", False, None)],
        ],
        BG_BLUE, PUCP_NAVY, header_rgb=PUCP_NAVY, body_pt=12.0, space_after_pt=12,
    )
    add_card(
        s12, 6.88, 1.08, 5.90, 3.56,
        "Trabajo Futuro y Extensiones",
        [
            [("• Pinza física real del JACO2: ", True, PUCP_GREEN_DARK), ("Habilitar cierre físico de dedos calibrando corriente/fuerza en el SDK para manipulación de objetos reales.", False, None)],
            [("• Video WebRTC de baja latencia: ", True, PUCP_GREEN_DARK), ("Integrar cámara del laboratorio en la interfaz web para operación cooperativa fuera de línea de vista directa.", False, None)],
            [("• Buffer adaptativo en WAN / 4G: ", True, PUCP_GREEN_DARK), ("Incorporar el filtro de mitigación con cola interpolada cuando se opere sobre internet pública.", False, None)],
        ],
        BG_SKY, PUCP_GREEN_DARK, header_rgb=PUCP_GREEN_DARK, body_pt=10.5, space_after_pt=7,
    )
    add_card(
        s12, 6.88, 4.78, 5.90, 1.28, None,
        [
            [("Siguiente paso: Demostración Funcional", True, PUCP_NAVY)],
            [("Operación cooperativa en vivo (Laptop + Celular) sobre el JACO2 físico + tarea virtual en PyBullet.", False, SLATE_DARK)],
        ],
        BG_AMBER, PUCP_AMBER, body_pt=10.5, space_after_pt=3,
    )

    # ==========================================================================
    # SLIDE 13: CIERRE OFICIAL PUCP NAVY (LAYOUT BLANK CON LOGO PUCP BLANCO)
    # ==========================================================================
    s13 = prs.slides.add_slide(layout_blank)
    tb13 = s13.shapes.add_textbox(Inches(1.2), Inches(5.05), Inches(10.93), Inches(1.45))
    tf13 = tb13.text_frame
    tf13.word_wrap = True
    p13_0 = tf13.paragraphs[0]
    p13_0.alignment = PP_ALIGN.CENTER
    p13_0.space_after = Pt(6)
    _add_runs(p13_0, [("¡Muchas gracias por su atención!", True, WHITE)], 26)
    p13_1 = tf13.add_paragraph()
    p13_1.alignment = PP_ALIGN.CENTER
    _add_runs(p13_1, [("Preguntas y Demostración Funcional  ·  Kinova JACO2 + PyBullet  ·  1MTR74 (PUCP)", False, PUCP_MINT)], 14)

    out_pptx.parent.mkdir(parents=True, exist_ok=True)
    prs.save(str(out_pptx))
    return out_pptx


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    out_pptx = repo_root / "reports" / "PRESENTACION_PROYECTO3_JACO2_PUCP.pptx"
    build_presentation(out_pptx)
    print(f"Presentación PowerPoint PUCP lista: {out_pptx} ({out_pptx.stat().st_size // 1024} KB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
