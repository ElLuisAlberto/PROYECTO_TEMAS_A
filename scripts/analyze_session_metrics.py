#!/usr/bin/python3
"""Analyze a teleop_cooperativo session directory for presentation metrics and plots."""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
import sys
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np


AXES_ORDER = ("X", "Y", "Z", "ROLL", "PITCH", "YAW", "PINZA")
ROOM_STATES_ORDER = ("NOT_READY", "SIM_ONLY", "READY", "ACTIVE", "PAUSED", "FAULT")
ROOM_STATE_COLORS = {
    "NOT_READY": "#94a3b8",
    "SIM_ONLY": "#38bdf8",
    "READY": "#22c55e",
    "ACTIVE": "#6366f1",
    "PAUSED": "#f59e0b",
    "FAULT": "#ef4444",
}
BRIDGE_LINE_RE = re.compile(
    r"\[kinova_direct_node[^\]]*\]\s+\[(INFO|WARN|ERROR)\]\s+\[([0-9]+\.[0-9]+)\]\s+\[[^\]]+\]:\s+(.*)"
)
BRIDGE_TX_RE = re.compile(r"Joint velocity transmissions in last interval:\s+(\d+)")
BRIDGE_START_RE = re.compile(r"Direct bridge started:\s+mock=(true|false)")


@dataclass
class Span:
    start: float
    end: float
    label: str


def find_latest_run_dir(tmp_root: Path = Path("/tmp")) -> Path:
    """Return the most recently modified /tmp/teleop-base.* folder with a non-empty log."""
    candidates: list[tuple[float, Path]] = []
    for path in tmp_root.glob("teleop-base.*"):
        events = path / "events.jsonl"
        if path.is_dir() and events.is_file() and events.stat().st_size > 0:
            candidates.append((events.stat().st_mtime, path))
    if not candidates:
        raise FileNotFoundError("No se encontraron carpetas /tmp/teleop-base.* con events.jsonl.")
    candidates.sort(key=lambda item: item[0], reverse=True)
    return candidates[0][1]


def summarize_array(values: list[float] | np.ndarray) -> dict[str, Any]:
    """Return standard descriptive statistics for a numeric sequence."""
    arr = np.asarray(values, dtype=float)
    arr = arr[np.isfinite(arr)]
    if arr.size == 0:
        return {"count": 0}
    return {
        "count": int(arr.size),
        "mean": round(float(np.mean(arr)), 3),
        "std": round(float(np.std(arr)), 3),
        "min": round(float(np.min(arr)), 3),
        "p50": round(float(np.percentile(arr, 50)), 3),
        "p95": round(float(np.percentile(arr, 95)), 3),
        "p99": round(float(np.percentile(arr, 99)), 3),
        "max": round(float(np.max(arr)), 3),
    }


def compute_rfc3550_jitter(values: list[float] | np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Compute instantaneous inter-packet jitter |ΔRTT| and RFC 3550 smoothed jitter."""
    arr = np.asarray(values, dtype=float)
    if arr.size == 0:
        return np.zeros(0, dtype=float), np.zeros(0, dtype=float)
    inst = np.zeros(arr.size, dtype=float)
    rfc = np.zeros(arr.size, dtype=float)
    j = 0.0
    for i in range(1, arr.size):
        d = abs(float(arr[i] - arr[i - 1]))
        j = j + (d - j) / 16.0
        inst[i] = d
        rfc[i] = j
    return inst, rfc


def parse_direct_bridge_log(log_path: Path) -> dict[str, Any]:
    """Parse kinova_direct_bridge output log for USB 100 Hz telemetry and events."""
    if not log_path.is_file():
        return {"available": False}

    mode_mock: bool | None = None
    tx_times: list[float] = []
    tx_counts: list[int] = []
    tx_hz: list[float] = []
    warn_count = 0
    error_count = 0
    sdk_cmd_accepted = 0
    recovery_events = 0
    prev_tx_time: float | None = None

    with log_path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            if mode_mock is None:
                m_start = BRIDGE_START_RE.search(line)
                if m_start:
                    mode_mock = m_start.group(1) == "true"
            m = BRIDGE_LINE_RE.search(line)
            if not m:
                continue
            level, ts_str, msg = m.group(1), m.group(2), m.group(3)
            ts = float(ts_str)
            if level == "WARN":
                warn_count += 1
            elif level == "ERROR":
                error_count += 1
            m_tx = BRIDGE_TX_RE.search(msg)
            if m_tx:
                count = int(m_tx.group(1))
                tx_times.append(ts)
                tx_counts.append(count)
                if prev_tx_time is not None and ts > prev_tx_time:
                    tx_hz.append(count / (ts - prev_tx_time))
                prev_tx_time = ts
            elif "SDK accepted joint velocity command" in msg:
                sdk_cmd_accepted += 1
            elif "recovery" in msg.lower():
                recovery_events += 1

    if not tx_times and mode_mock is None and warn_count == 0 and error_count == 0:
        return {"available": False}

    t0 = tx_times[0] if tx_times else 0.0
    rel_tx_times = [round(t - t0, 3) for t in tx_times]
    return {
        "available": True,
        "mock": mode_mock,
        "intervals_count": len(tx_counts),
        "transmissions_per_interval": summarize_array([float(x) for x in tx_counts]),
        "effective_hz": summarize_array(tx_hz),
        "sdk_accepted_joint_cmd_logs": sdk_cmd_accepted,
        "recovery_logs": recovery_events,
        "warnings": warn_count,
        "errors": error_count,
        "series": {
            "t_rel_sec": rel_tx_times,
            "tx_counts": tx_counts,
            "tx_hz": [round(x, 2) for x in tx_hz],
        },
    }


def load_and_analyze(run_dir: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    """Read events.jsonl + direct_bridge.log and compute summary + plot series."""
    events_path = run_dir / "events.jsonl"
    if not events_path.is_file():
        raise FileNotFoundError(f"No se encontró {events_path}")

    run_id = run_dir.name
    first_utc: str | None = None
    last_utc: str | None = None
    t0_mono: float | None = None
    last_mono: float | None = None
    malformed_lines = 0

    event_type_counts: Counter[str] = Counter()
    proto_msg_counts: Counter[str] = Counter()
    proto_outcome_counts: Counter[str] = Counter()
    proto_error_counts: Counter[str] = Counter()
    sources: Counter[str] = Counter()

    # Telemetry arrays
    tel_t: list[float] = []
    tel_mono: list[float] = []
    tel_state: list[str] = []
    tel_pose_age_t: list[float] = []
    tel_pose_age_ms: list[float] = []
    tel_pos_t: list[float] = []
    tel_pos: list[list[float]] = []
    tel_twist_lin: list[list[float]] = []
    tel_twist_ang: list[list[float]] = []
    tel_joints_t: list[float] = []
    tel_joints: list[list[float]] = []
    tel_fingers: list[list[float]] = []
    tel_finger_t: list[float] = []
    tel_axis_owners: list[dict[str, str | None]] = []
    tel_finger_owners: list[str | None] = []

    # Cooperation & state tracking across all events
    state_spans: list[Span] = []
    axis_spans: dict[str, list[Span]] = {axis: [] for axis in AXES_ORDER}
    open_axis_span: dict[str, tuple[float, str] | None] = {axis: None for axis in AXES_ORDER}
    open_state_span: tuple[float, str] | None = None
    state_transitions: Counter[str] = Counter()

    operator_axes_seen: dict[str, set[str]] = defaultdict(set)
    n_ops_t: list[float] = []
    n_ops_val: list[int] = []

    # RTT tracking
    rtt_t: list[float] = []
    rtt_ms: list[float] = []
    rtt_by_operator: dict[str, list[float]] = defaultdict(list)
    rtt_t_by_operator: dict[str, list[float]] = defaultdict(list)

    # Virtual task tracking
    vtask_max_delivered = 0
    vtask_total = 0
    vtask_grabs = 0
    vtask_impact_releases = 0
    virtual_contacts_detected = 0
    virtual_wall_blocks_detected = 0
    prev_held: int | None = None
    prev_release_reason: str | None = None
    prev_vcol = False
    prev_vwall = False
    vtask_seen = False

    # Cooperation timing accumulators
    prev_ev_mono: float | None = None
    prev_room_state: str | None = None
    prev_op_count: int = 0
    prev_reserved_count: int = 0
    state_dwell_sec: Counter[str] = Counter()
    op_count_dwell_sec: Counter[int] = Counter()
    max_simultaneous_ops = 0
    max_reserved_axes = 0

    with events_path.open("r", encoding="utf-8", errors="replace") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line:
                continue
            try:
                ev = json.loads(line)
            except json.JSONDecodeError:
                malformed_lines += 1
                continue

            if "run_id" in ev and ev["run_id"]:
                run_id = str(ev["run_id"])
            ts_utc = ev.get("timestamp_utc")
            if isinstance(ts_utc, str):
                if first_utc is None:
                    first_utc = ts_utc
                last_utc = ts_utc

            mono = ev.get("monotonic_sec")
            if not isinstance(mono, (int, float)) or not math.isfinite(mono):
                continue
            mono = float(mono)
            if t0_mono is None:
                t0_mono = mono
            rel_t = mono - t0_mono
            last_mono = mono

            ev_type = str(ev.get("event_type", "unknown"))
            event_type_counts[ev_type] += 1
            payload = ev.get("payload") or {}
            room_state = ev.get("room_state") or {}
            r_state = str(room_state.get("state") or "UNKNOWN")
            axes_map = room_state.get("axes") or {}

            # Update dwell time using previous interval
            if prev_ev_mono is not None and mono >= prev_ev_mono:
                ev_dt = mono - prev_ev_mono
                if prev_room_state:
                    state_dwell_sec[prev_room_state] += ev_dt
                op_count_dwell_sec[prev_op_count] += ev_dt

            # Track active owners and axes
            active_owners: set[str] = set()
            reserved_now = 0
            for axis in AXES_ORDER:
                info = axes_map.get(axis) or {}
                assigned = bool(info.get("assigned"))
                owner = info.get("owner")
                owner_str = str(owner) if assigned and owner else None
                if owner_str:
                    active_owners.add(owner_str)
                    operator_axes_seen[owner_str].add(axis)
                    reserved_now += 1
                cur = open_axis_span[axis]
                if owner_str != (cur[1] if cur else None):
                    if cur is not None:
                        axis_spans[axis].append(Span(cur[0], rel_t, cur[1]))
                    open_axis_span[axis] = (rel_t, owner_str) if owner_str else None

            op_count_now = len(active_owners)
            max_simultaneous_ops = max(max_simultaneous_ops, op_count_now)
            max_reserved_axes = max(max_reserved_axes, reserved_now)
            if not n_ops_val or op_count_now != n_ops_val[-1]:
                n_ops_t.append(rel_t)
                n_ops_val.append(op_count_now)

            if open_state_span is None:
                open_state_span = (rel_t, r_state)
            elif open_state_span[1] != r_state:
                state_spans.append(Span(open_state_span[0], rel_t, open_state_span[1]))
                state_transitions[f"{open_state_span[1]}->{r_state}"] += 1
                open_state_span = (rel_t, r_state)

            prev_ev_mono = mono
            prev_room_state = r_state
            prev_op_count = op_count_now
            prev_reserved_count = reserved_now

            if ev_type == "protocol":
                m_type = str(payload.get("message_type") or "unknown")
                outcome = str(payload.get("outcome") or "none")
                proto_msg_counts[m_type] += 1
                proto_outcome_counts[f"{m_type}:{outcome}"] += 1
                if outcome == "error":
                    proto_error_counts[str(payload.get("error_code") or "unknown")] += 1
                raw_rtt = payload.get("rtt_ms")
                if isinstance(raw_rtt, (int, float)) and not isinstance(raw_rtt, bool) and math.isfinite(raw_rtt):
                    rtt_t.append(rel_t)
                    rtt_ms.append(float(raw_rtt))
                    op_key = str(payload.get("operator_name") or payload.get("role") or "web")
                    rtt_by_operator[op_key].append(float(raw_rtt))
                    rtt_t_by_operator[op_key].append(rel_t)

            elif ev_type == "telemetry":
                src = payload.get("source")
                if isinstance(src, str):
                    sources[src] += 1
                tel_t.append(rel_t)
                tel_mono.append(mono)
                tel_state.append(r_state)
                tel_axis_owners.append({
                    ax: (str((axes_map.get(ax) or {}).get("owner")) if (axes_map.get(ax) or {}).get("assigned") and (axes_map.get(ax) or {}).get("owner") else None)
                    for ax in AXES_ORDER
                })

                pose_age = payload.get("pose_age_sec")
                if isinstance(pose_age, (int, float)) and math.isfinite(pose_age):
                    tel_pose_age_t.append(rel_t)
                    tel_pose_age_ms.append(float(pose_age) * 1000.0)

                pose = (payload.get("pose") or {}).get("position")
                if isinstance(pose, list) and len(pose) == 3:
                    tel_pos_t.append(rel_t)
                    tel_pos.append([float(pose[0]), float(pose[1]), float(pose[2])])

                twist = payload.get("twist") or {}
                lin = twist.get("linear")
                ang = twist.get("angular")
                tel_twist_lin.append([float(x) for x in lin] if isinstance(lin, list) and len(lin) == 3 else [0.0, 0.0, 0.0])
                tel_twist_ang.append([float(x) for x in ang] if isinstance(ang, list) and len(ang) == 3 else [0.0, 0.0, 0.0])

                joints = payload.get("joint_positions")
                if isinstance(joints, list) and len(joints) == 6:
                    tel_joints_t.append(rel_t)
                    tel_joints.append([float(x) for x in joints])

                fingers = payload.get("finger_positions")
                if isinstance(fingers, list) and len(fingers) == 3:
                    tel_finger_t.append(rel_t)
                    tel_fingers.append([float(x) for x in fingers])
                    p_info = axes_map.get("PINZA") or {}
                    tel_finger_owners.append(str(p_info.get("owner")) if p_info.get("assigned") and p_info.get("owner") else None)

                vtask = payload.get("virtual_task")
                if isinstance(vtask, dict):
                    vtask_seen = True
                    total = vtask.get("total")
                    delivered = vtask.get("delivered")
                    if isinstance(total, int):
                        vtask_total = max(vtask_total, total)
                    if isinstance(delivered, int):
                        vtask_max_delivered = max(vtask_max_delivered, delivered)
                    held = vtask.get("held")
                    if isinstance(held, int) and held != prev_held:
                        vtask_grabs += 1
                    prev_held = held if isinstance(held, int) else None
                    rel_reason = vtask.get("release_reason")
                    if rel_reason == "impacto" and prev_release_reason != "impacto":
                        vtask_impact_releases += 1
                    prev_release_reason = rel_reason if isinstance(rel_reason, str) else None
                    vcol = bool(vtask.get("virtual_collision"))
                    if vcol and not prev_vcol:
                        virtual_contacts_detected += 1
                    prev_vcol = vcol
                    vwall = bool(vtask.get("blocked_by"))
                    if vwall and not prev_vwall:
                        virtual_wall_blocks_detected += 1
                    prev_vwall = vwall

    if t0_mono is None or last_mono is None:
        raise RuntimeError(f"No hay eventos válidos con monotonic_sec en {events_path}")

    duration_sec = max(0.0, last_mono - t0_mono)
    if open_state_span is not None:
        state_spans.append(Span(open_state_span[0], duration_sec, open_state_span[1]))
    for axis in AXES_ORDER:
        cur = open_axis_span[axis]
        if cur is not None:
            axis_spans[axis].append(Span(cur[0], duration_sec, cur[1]))
    n_ops_t.append(duration_sec)
    n_ops_val.append(prev_op_count)

    # Gateway tick dt stats
    tel_mono_arr = np.asarray(tel_mono, dtype=float)
    dt_ms_arr = np.diff(tel_mono_arr) * 1000.0 if tel_mono_arr.size >= 2 else np.asarray([], dtype=float)
    dt_stats = summarize_array(dt_ms_arr)
    if dt_stats["count"] > 0:
        dt_stats["effective_hz"] = round(float(1000.0 / dt_stats["mean"]), 3) if dt_stats["mean"] > 0 else 0.0
        dt_stats["within_15_25_ms_pct"] = round(float(np.mean((dt_ms_arr >= 15.0) & (dt_ms_arr <= 25.0)) * 100.0), 2)

    pose_age_arr = np.asarray(tel_pose_age_ms, dtype=float)
    pose_age_stats = summarize_array(pose_age_arr)
    if pose_age_stats["count"] > 0:
        pose_age_stats["within_300ms_pct"] = round(float(np.mean(pose_age_arr <= 300.0) * 100.0), 2)

    # Kinematics & commanded twist
    pos_arr = np.asarray(tel_pos, dtype=float) if tel_pos else np.zeros((0, 3), dtype=float)
    lin_arr = np.asarray(tel_twist_lin, dtype=float) if tel_twist_lin else np.zeros((0, 3), dtype=float)
    ang_arr = np.asarray(tel_twist_ang, dtype=float) if tel_twist_ang else np.zeros((0, 3), dtype=float)
    lin_mag_mms = np.linalg.norm(lin_arr, axis=1) * 1000.0 if lin_arr.size else np.zeros(0)
    ang_mag_degs = np.rad2deg(np.linalg.norm(ang_arr, axis=1)) if ang_arr.size else np.zeros(0)

    sample_dt = np.diff(tel_mono_arr, append=tel_mono_arr[-1]) if tel_mono_arr.size else np.zeros(0)
    if sample_dt.size >= 2:
        sample_dt[-1] = float(np.median(sample_dt[:-1]))

    active_lin_sec = float(np.sum(sample_dt[lin_mag_mms > 1e-3])) if lin_mag_mms.size == sample_dt.size else 0.0
    active_ang_sec = float(np.sum(sample_dt[ang_mag_degs > 1e-2])) if ang_mag_degs.size == sample_dt.size else 0.0
    axis_cmd_sec: dict[str, float] = {}
    axis_cmd_by_owner: dict[str, dict[str, float]] = {ax: defaultdict(float) for ax in AXES_ORDER}
    if lin_arr.size and lin_arr.shape[0] == sample_dt.size:
        for idx, ax in enumerate(("X", "Y", "Z")):
            mask = np.abs(lin_arr[:, idx]) > 1e-6
            axis_cmd_sec[ax] = round(float(np.sum(sample_dt[mask])), 3)
            for k in np.flatnonzero(mask):
                owner = tel_axis_owners[k].get(ax) or "Sin dueño"
                axis_cmd_by_owner[ax][owner] += float(sample_dt[k])
        for idx, ax in enumerate(("ROLL", "PITCH", "YAW")):
            mask = np.abs(ang_arr[:, idx]) > 1e-6
            axis_cmd_sec[ax] = round(float(np.sum(sample_dt[mask])), 3)
            for k in np.flatnonzero(mask):
                owner = tel_axis_owners[k].get(ax) or "Sin dueño"
                axis_cmd_by_owner[ax][owner] += float(sample_dt[k])
    if len(tel_fingers) >= 2 and len(tel_finger_t) == len(tel_fingers):
        f_arr = np.asarray(tel_fingers, dtype=float)
        f_dt = np.diff(np.asarray(tel_finger_t, dtype=float))
        moving_pinza = np.abs(np.diff(f_arr[:, 0])) > 1e-4
        axis_cmd_sec["PINZA"] = round(float(np.sum(f_dt[moving_pinza])), 3)
        for k in np.flatnonzero(moving_pinza):
            owner = tel_finger_owners[k + 1] or tel_finger_owners[k] or "Sin dueño"
            axis_cmd_by_owner["PINZA"][owner] += float(f_dt[k])
    else:
        axis_cmd_sec["PINZA"] = 0.0
    axis_cmd_by_owner_rounded = {
        ax: {op: round(sec, 3) for op, sec in sorted(owners.items()) if sec > 0.0}
        for ax, owners in axis_cmd_by_owner.items()
    }

    kinematics: dict[str, Any] = {"samples": int(pos_arr.shape[0])}
    if pos_arr.shape[0] >= 2:
        step_dists = np.linalg.norm(np.diff(pos_arr, axis=0), axis=1)
        path_length_m = float(np.sum(step_dists))
        disp_from_start_cm = np.linalg.norm(pos_arr - pos_arr[0], axis=1) * 100.0
        mins = np.min(pos_arr, axis=0)
        maxs = np.max(pos_arr, axis=0)
        kinematics.update({
            "start_xyz_m": [round(float(x), 4) for x in pos_arr[0]],
            "end_xyz_m": [round(float(x), 4) for x in pos_arr[-1]],
            "excursion_cm": {
                "X": round(float((maxs[0] - mins[0]) * 100.0), 2),
                "Y": round(float((maxs[1] - mins[1]) * 100.0), 2),
                "Z": round(float((maxs[2] - mins[2]) * 100.0), 2),
            },
            "path_length_m": round(path_length_m, 4),
            "max_displacement_from_start_cm": round(float(np.max(disp_from_start_cm)), 2),
            "active_linear_motion_sec": round(active_lin_sec, 2),
            "active_angular_motion_sec": round(active_ang_sec, 2),
            "peak_linear_speed_mm_s": round(float(np.max(lin_mag_mms)), 2) if lin_mag_mms.size else 0.0,
            "peak_angular_speed_deg_s": round(float(np.max(ang_mag_degs)), 2) if ang_mag_degs.size else 0.0,
            "commanded_time_by_axis_sec": axis_cmd_sec,
            "commanded_time_by_axis_owner_sec": axis_cmd_by_owner_rounded,
        })

    joints_arr_deg = np.rad2deg(np.asarray(tel_joints, dtype=float)) if tel_joints else np.zeros((0, 6))
    joints_summary: dict[str, dict[str, float]] = {}
    if joints_arr_deg.shape[0] > 0:
        for i in range(6):
            col = joints_arr_deg[:, i]
            joints_summary[f"J{i + 1}"] = {
                "min_deg": round(float(np.min(col)), 2),
                "max_deg": round(float(np.max(col)), 2),
                "range_deg": round(float(np.max(col) - np.min(col)), 2),
            }

    fingers_arr = np.asarray(tel_fingers, dtype=float) if tel_fingers else np.zeros((0, 3))
    gripper_summary: dict[str, Any] = {"samples": int(fingers_arr.shape[0])}
    if fingers_arr.shape[0] > 0:
        f_min = float(np.min(fingers_arr))
        f_max = float(np.max(fingers_arr))
        gripper_summary.update({
            "active": bool((f_max - f_min) > 1e-4 or f_max > 1e-4),
            "min_rad": round(f_min, 4),
            "max_rad": round(f_max, 4),
            "range_rad": round(f_max - f_min, 4),
        })

    # Cooperation & dwell breakdown
    with_1_op = round(float(op_count_dwell_sec.get(1, 0.0)), 2)
    with_2plus_op = round(float(sum(v for k, v in op_count_dwell_sec.items() if k >= 2)), 2)
    any_op_sec = with_1_op + with_2plus_op
    coop_overlap_pct = round((with_2plus_op / any_op_sec) * 100.0, 2) if any_op_sec > 0 else 0.0

    bridge_info = parse_direct_bridge_log(run_dir / "direct_bridge.log")

    rtt_info: dict[str, Any] = {"available": len(rtt_ms) > 0}
    if rtt_ms:
        rtt_info["overall_ms"] = summarize_array(rtt_ms)
        rtt_info["by_client"] = {
            op: summarize_array(vals) for op, vals in sorted(rtt_by_operator.items())
        }
        all_deltas: list[float] = []
        all_rfc: list[float] = []
        jitter_by_client: dict[str, dict[str, Any]] = {}
        rfc_by_client: dict[str, dict[str, Any]] = {}
        for op, vals in sorted(rtt_by_operator.items()):
            if len(vals) >= 2:
                inst_j, rfc_j = compute_rfc3550_jitter(vals)
                jitter_by_client[op] = summarize_array(inst_j[1:])
                rfc_st = summarize_array(rfc_j[1:])
                rfc_st["final"] = round(float(rfc_j[-1]), 3)
                rfc_by_client[op] = rfc_st
                all_deltas.extend(inst_j[1:].tolist())
                all_rfc.extend(rfc_j[1:].tolist())
            else:
                jitter_by_client[op] = {"count": 0}
                rfc_by_client[op] = {"count": 0}
        rtt_info["jitter_inter_packet_ms"] = {
            "overall": summarize_array(all_deltas),
            "by_client": jitter_by_client,
        }
        rtt_info["jitter_rfc3550_ms"] = {
            "overall": summarize_array(all_rfc),
            "by_client": rfc_by_client,
        }

    summary: dict[str, Any] = {
        "run_id": run_id,
        "run_dir": str(run_dir),
        "start_utc": first_utc,
        "end_utc": last_utc,
        "duration_sec": round(duration_sec, 2),
        "malformed_tail_lines_skipped": malformed_lines,
        "telemetry_sources": dict(sources),
        "event_counts": dict(event_type_counts),
        "protocol_messages": dict(proto_msg_counts),
        "protocol_outcomes": dict(proto_outcome_counts),
        "protocol_errors": dict(proto_error_counts),
        "gateway_tick_dt_ms": dt_stats,
        "pose_age_ms": pose_age_stats,
        "network_rtt": rtt_info,
        "direct_bridge": {k: v for k, v in bridge_info.items() if k != "series"},
        "room_state_dwell_sec": {
            st: round(float(state_dwell_sec.get(st, 0.0)), 2)
            for st in ROOM_STATES_ORDER
            if state_dwell_sec.get(st, 0.0) > 0
        },
        "state_transitions": dict(state_transitions),
        "cooperation": {
            "distinct_operators": len(operator_axes_seen),
            "operators_axes": {op: sorted(axes) for op, axes in sorted(operator_axes_seen.items())},
            "max_simultaneous_operators": max_simultaneous_ops,
            "max_simultaneous_reserved_axes": max_reserved_axes,
            "single_operator_sec": with_1_op,
            "multi_operator_overlap_sec": with_2plus_op,
            "multi_operator_overlap_pct_of_occupied": coop_overlap_pct,
        },
        "kinematics": kinematics,
        "joints_deg": joints_summary,
        "simulated_gripper": gripper_summary,
        "virtual_task": {
            "available": vtask_seen,
            "delivered_max": vtask_max_delivered,
            "total": vtask_total,
            "grabs_detected": vtask_grabs,
            "impact_releases": vtask_impact_releases,
            "virtual_contacts_detected": virtual_contacts_detected,
            "virtual_wall_blocks_detected": virtual_wall_blocks_detected,
        },
    }

    series = {
        "duration_sec": duration_sec,
        "tel_t": np.asarray(tel_t, dtype=float),
        "dt_t": np.asarray(tel_t[1:], dtype=float) if len(tel_t) >= 2 else np.zeros(0),
        "dt_ms": dt_ms_arr,
        "pose_age_t": np.asarray(tel_pose_age_t, dtype=float),
        "pose_age_ms": pose_age_arr,
        "pos_t": np.asarray(tel_pos_t, dtype=float),
        "pos": pos_arr,
        "lin_mag_mms": lin_mag_mms,
        "ang_mag_degs": ang_mag_degs,
        "joints_t": np.asarray(tel_joints_t, dtype=float),
        "joints_deg": joints_arr_deg,
        "finger_t": np.asarray(tel_finger_t, dtype=float),
        "fingers": fingers_arr,
        "rtt_t": np.asarray(rtt_t, dtype=float),
        "rtt_ms": np.asarray(rtt_ms, dtype=float),
        "rtt_by_client": {op: np.asarray(vals, dtype=float) for op, vals in sorted(rtt_by_operator.items())},
        "rtt_t_by_client": {op: np.asarray(ts, dtype=float) for op, ts in sorted(rtt_t_by_operator.items())},
        "bridge_series": bridge_info.get("series", {}),
        "state_spans": state_spans,
        "axis_spans": axis_spans,
        "n_ops_t": np.asarray(n_ops_t, dtype=float),
        "n_ops_val": np.asarray(n_ops_val, dtype=int),
        "operators": sorted(operator_axes_seen.keys()),
    }
    return summary, series


def _decimate(x: np.ndarray, y: np.ndarray, max_points: int = 8000) -> tuple[np.ndarray, np.ndarray]:
    if x.size <= max_points:
        return x, y
    step = int(math.ceil(x.size / max_points))
    return x[::step], y[::step]


def generate_plots(summary: dict[str, Any], series: dict[str, Any], out_dir: Path) -> list[Path]:
    """Render 4 publication/slide-ready PNG figures into out_dir."""
    plt.rcParams.update({
        "figure.facecolor": "white",
        "axes.facecolor": "#fafafa",
        "axes.grid": True,
        "grid.alpha": 0.35,
        "font.size": 10,
        "axes.titlesize": 11,
        "axes.labelsize": 10,
    })
    out_paths: list[Path] = []
    run_id = summary["run_id"]
    dur = series["duration_sec"]

    # 1. Control loop frequency & latency / freshness / RTT or bridge
    fig, axes = plt.subplots(3, 1, figsize=(11, 9.2), constrained_layout=True)
    fig.suptitle(f"Desempeño temporal y latencia · {run_id}", fontsize=13, fontweight="bold")

    ax = axes[0]
    if series["dt_ms"].size > 0:
        tx, ty = _decimate(series["dt_t"], series["dt_ms"])
        ax.plot(tx, ty, color="#4f46e5", linewidth=0.9, label="Periodo entre muestras (dt)")
        ax.axhline(20.0, color="#16a34a", linestyle="--", linewidth=1.2, label="Objetivo 20 ms (50 Hz)")
        st = summary["gateway_tick_dt_ms"]
        ax.set_title(
            f"Lazo del gateway: media {st['mean']} ms ({st.get('effective_hz', 0)} Hz) · "
            f"σ={st['std']} ms · p95={st['p95']} ms · máx={st['max']} ms"
        )
        y_hi = min(max(float(np.percentile(series["dt_ms"], 99.5)) * 1.4, 30.0), float(np.max(series["dt_ms"])) * 1.15)
        ax.set_ylim(0, max(y_hi, 30.0))
        ax.legend(loc="upper right", fontsize=9)
    else:
        ax.text(0.5, 0.5, "Sin muestras de dt del gateway", ha="center", va="center", transform=ax.transAxes)
    ax.set_ylabel("dt (ms)")
    ax.set_xlim(0, max(dur, 1.0))

    ax = axes[1]
    if series["pose_age_ms"].size == series["pose_age_t"].size and series["pose_age_ms"].size > 0:
        tx, ty = _decimate(series["pose_age_t"], series["pose_age_ms"])
        ax.plot(tx, ty, color="#0284c7", linewidth=0.9, label="pose_age_sec")
        ax.axhline(300.0, color="#dc2626", linestyle="--", linewidth=1.2, label="Límite watchdog (300 ms)")
        st = summary["pose_age_ms"]
        ax.set_title(
            f"Frescura de telemetría (edad de postura): media {st['mean']} ms · "
            f"p95={st['p95']} ms · máx={st['max']} ms · {st.get('within_300ms_pct', 0)}% ≤ 300 ms"
        )
        ax.legend(loc="upper right", fontsize=9)
    else:
        ax.text(0.5, 0.5, "Sin pose_age_sec registrado en esta sesión", ha="center", va="center", transform=ax.transAxes)
    ax.set_ylabel("Edad postura (ms)")
    ax.set_xlim(0, max(dur, 1.0))

    ax = axes[2]
    if series["rtt_ms"].size > 0:
        ax.plot(series["rtt_t"], series["rtt_ms"], color="#7c3aed", linewidth=0.9, marker=".", markersize=2.5, label="RTT WebSocket cliente")
        st = summary["network_rtt"]["overall_ms"]
        ax.set_title(
            f"Latencia de red WebSocket (RTT): n={st['count']} · media {st['mean']} ms · "
            f"p50={st['p50']} ms · p95={st['p95']} ms · máx={st['max']} ms"
        )
        ax.set_ylabel("RTT (ms)")
        ax.set_xlabel("Tiempo desde inicio (s)")
        ax.set_xlim(0, max(dur, 1.0))
        ax.legend(loc="upper right", fontsize=9)
    elif series["bridge_series"].get("tx_counts"):
        bs = series["bridge_series"]
        bt = np.asarray(bs["t_rel_sec"], dtype=float)
        bc = np.asarray(bs["tx_counts"], dtype=float)
        ax.plot(bt, bc, color="#0d9488", linewidth=1.2, marker="o", markersize=2.5, label="Transmisiones SDK por intervalo (~1 s)")
        ax.axhline(100.0, color="#16a34a", linestyle="--", linewidth=1.1, label="Tasa nominal Kinova 100 Hz")
        b_st = summary["direct_bridge"]["transmissions_per_interval"]
        ax.set_title(
            f"Bridge USB Kinova (direct_bridge.log): {b_st['count']} intervalos · "
            f"media {b_st['mean']} tx/s · mín={b_st['min']} · máx={b_st['max']} (RTT web: no registrado en esta corrida)"
        )
        ax.set_ylabel("Tx SDK / s (Hz)")
        ax.set_xlabel("Tiempo en el bridge (s)")
        ax.set_ylim(max(0.0, float(np.min(bc)) - 10), float(np.max(bc)) + 10)
        ax.legend(loc="lower right", fontsize=9)
    else:
        ax.text(
            0.5, 0.5,
            "Sin muestras de RTT ni de intervalos del bridge.\nUsa los clientes web actualizados para capturar RTT en tiempo real.",
            ha="center", va="center", transform=ax.transAxes,
        )
        ax.set_xlabel("Tiempo (s)")

    p1 = out_dir / "01_frecuencia_y_latencia.png"
    fig.savefig(p1, dpi=300)
    plt.close(fig)
    out_paths.append(p1)

    # 2. 3D Trajectory + X/Y/Z + Linear speed
    fig = plt.figure(figsize=(12.5, 6.2), constrained_layout=True)
    fig.suptitle(f"Trayectoria cartesiana y velocidad lineal · {run_id}", fontsize=13, fontweight="bold")
    gs = fig.add_gridspec(2, 2, width_ratios=[1.1, 1.2])
    try:
        ax3d = fig.add_subplot(gs[:, 0], projection="3d")
        has_3d = True
    except ValueError:
        ax3d = fig.add_subplot(gs[:, 0])
        has_3d = False
    ax_xyz = fig.add_subplot(gs[0, 1])
    ax_spd = fig.add_subplot(gs[1, 1], sharex=ax_xyz)

    pos = series["pos"]
    pos_t = series["pos_t"]
    tel_t = series["tel_t"]
    if pos.shape[0] >= 2 and pos.shape[0] == pos_t.size:
        step = max(1, pos.shape[0] // 5000)
        p_sub = pos[::step]
        t_sub = pos_t[::step]
        kin = summary["kinematics"]
        if has_3d:
            sc = ax3d.scatter(p_sub[:, 0], p_sub[:, 1], p_sub[:, 2], c=t_sub, cmap="viridis", s=5, alpha=0.85)
            ax3d.plot(p_sub[:, 0], p_sub[:, 1], p_sub[:, 2], color="#64748b", linewidth=0.6, alpha=0.45)
            ax3d.scatter([pos[0, 0]], [pos[0, 1]], [pos[0, 2]], color="#16a34a", s=60, marker="o", label="Inicio")
            ax3d.scatter([pos[-1, 0]], [pos[-1, 1]], [pos[-1, 2]], color="#dc2626", s=70, marker="^", label="Fin")
            ax3d.set_zlabel("Z (m)")
            ax3d.set_title(
                f"Trayectoria 3D · recorrido {kin.get('path_length_m', 0):.3f} m · "
                f"desplaz. máx {kin.get('max_displacement_from_start_cm', 0):.1f} cm"
            )
        else:
            sc = ax3d.scatter(p_sub[:, 0], p_sub[:, 1], c=t_sub, cmap="viridis", s=6, alpha=0.85)
            ax3d.plot(p_sub[:, 0], p_sub[:, 1], color="#64748b", linewidth=0.7, alpha=0.45)
            ax3d.scatter([pos[0, 0]], [pos[0, 1]], color="#16a34a", s=60, marker="o", label="Inicio")
            ax3d.scatter([pos[-1, 0]], [pos[-1, 1]], color="#dc2626", s=70, marker="^", label="Fin")
            ax3d.set_aspect("equal", adjustable="datalim")
            ax3d.set_title(
                f"Trayectoria XY · recorrido 3D {kin.get('path_length_m', 0):.3f} m · "
                f"desplaz. máx {kin.get('max_displacement_from_start_cm', 0):.1f} cm"
            )
        fig.colorbar(sc, ax=ax3d, shrink=0.65, pad=0.08, label="Tiempo (s)")
        ax3d.set_xlabel("X (m)")
        ax3d.set_ylabel("Y (m)")
        ax3d.legend(loc="upper left", fontsize=8)

        tx, x_sub = _decimate(pos_t, pos[:, 0])
        _, y_sub = _decimate(pos_t, pos[:, 1])
        _, z_sub = _decimate(pos_t, pos[:, 2])
        ax_xyz.plot(tx, x_sub, label="X (m)", color="#ef4444", linewidth=1.2)
        ax_xyz.plot(tx, y_sub, label="Y (m)", color="#22c55e", linewidth=1.2)
        ax_xyz.plot(tx, z_sub, label="Z (m)", color="#3b82f6", linewidth=1.2)
        ax_xyz.set_ylabel("Posición (m)")
        ax_xyz.set_title("Coordenadas cartesianas X, Y, Z en el tiempo")
        ax_xyz.legend(loc="upper right", ncol=3, fontsize=9)

    if series["lin_mag_mms"].size == tel_t.size and tel_t.size > 0:
        tx, vs = _decimate(tel_t, series["lin_mag_mms"])
        ax_spd.plot(tx, vs, color="#7c3aed", linewidth=1.1, label="Velocidad comandada ||v||")
        ax_spd.fill_between(tx, 0, vs, color="#7c3aed", alpha=0.15)
        ax_spd.axhline(50.0, color="#dc2626", linestyle="--", linewidth=1.0, label="Límite seguridad (50 mm/s)")
        ax_spd.set_ylabel("|v| (mm/s)")
        ax_spd.set_xlabel("Tiempo (s)")
        ax_spd.set_ylim(0, max(58.0, float(np.max(vs)) * 1.15))
        ax_spd.set_title(
            f"Velocidad lineal · activo {summary['kinematics'].get('active_linear_motion_sec', 0)} s · "
            f"pico {summary['kinematics'].get('peak_linear_speed_mm_s', 0)} mm/s"
        )
        ax_spd.set_xlim(0, max(dur, 1.0))
        ax_spd.legend(loc="upper right", fontsize=8.5)

    p2 = out_dir / "02_trayectoria_3d_y_xyz.png"
    fig.savefig(p2, dpi=300)
    plt.close(fig)
    out_paths.append(p2)

    # 3. Joint angles J1..J6 + angular speed & simulated gripper
    fig, axes = plt.subplots(2, 1, figsize=(11.5, 7.6), sharex=True, constrained_layout=True)
    fig.suptitle(f"Articulaciones J1–J6, rotación comandada y pinza simulada · {run_id}", fontsize=13, fontweight="bold")

    ax = axes[0]
    joints_deg = series["joints_deg"]
    joints_t = series["joints_t"]
    if joints_deg.shape[0] > 0 and joints_deg.shape[0] == joints_t.size:
        colors_j = ["#ef4444", "#f97316", "#eab308", "#22c55e", "#06b6d4", "#6366f1"]
        for j in range(6):
            tx, jy = _decimate(joints_t, joints_deg[:, j])
            rng = summary["joints_deg"].get(f"J{j+1}", {}).get("range_deg", 0.0)
            ax.plot(tx, jy, label=f"J{j+1} (Δ={rng:.1f}°)", color=colors_j[j], linewidth=1.1)
        ax.set_ylabel("Ángulo articular (°)")
        ax.set_title("Trayectoria medida de articulaciones J1–J6")
        ax.legend(loc="upper right", ncol=3, fontsize=8.5)
    else:
        ax.text(0.5, 0.5, "Sin telemetría articular J1–J6", ha="center", va="center", transform=ax.transAxes)

    ax = axes[1]
    if series["ang_mag_degs"].size == tel_t.size and tel_t.size > 0:
        tx, wy = _decimate(tel_t, series["ang_mag_degs"])
        l1 = ax.plot(tx, wy, color="#db2777", linewidth=1.1, label="|ω_cmd| (°/s)")
        ax.fill_between(tx, 0, wy, color="#db2777", alpha=0.12)
        ax.set_ylabel("|ω| (°/s)", color="#db2777")
    else:
        l1 = []
    if series["fingers"].shape[0] > 0 and series["fingers"].shape[0] == series["finger_t"].size:
        ax_r = ax.twinx()
        tx, fy = _decimate(series["finger_t"], series["fingers"][:, 0])
        l2 = ax_r.plot(tx, fy, color="#0f766e", linestyle="--", linewidth=1.3, label="Pinza sim. F1 (rad)")
        ax_r.set_ylabel("Apertura pinza (rad)", color="#0f766e")
        ax_r.grid(False)
        lines = l1 + l2
        ax.legend(lines, [ln.get_label() for ln in lines], loc="upper right", fontsize=9)
    elif l1:
        ax.legend(loc="upper right", fontsize=9)
    ax.set_xlabel("Tiempo (s)")
    ax.set_xlim(0, max(dur, 1.0))
    ax.set_title(
        f"Velocidad angular comandada (pico {summary['kinematics'].get('peak_angular_speed_deg_s', 0)} °/s) y dedos simulados"
    )

    p3 = out_dir / "03_articulaciones_y_pinza.png"
    fig.savefig(p3, dpi=300)
    plt.close(fig)
    out_paths.append(p3)

    # 4. Cooperation timeline & Room states
    fig, axes = plt.subplots(2, 1, figsize=(11.5, 7.6), sharex=True, constrained_layout=True)
    fig.suptitle(f"Dinámica cooperativa, reservas de ejes y estados de sala · {run_id}", fontsize=13, fontweight="bold")

    ax = axes[0]
    state_to_y = {st: idx for idx, st in enumerate(ROOM_STATES_ORDER)}
    for span in series["state_spans"]:
        if span.end <= span.start:
            continue
        y = state_to_y.get(span.label, 0)
        color = ROOM_STATE_COLORS.get(span.label, "#64748b")
        ax.barh(y, span.end - span.start, left=span.start, height=0.6, color=color, alpha=0.88, edgecolor="none")
    ax.set_yticks(range(len(ROOM_STATES_ORDER)))
    ax.set_yticklabels(ROOM_STATES_ORDER)
    ax.set_ylim(-0.6, len(ROOM_STATES_ORDER) - 0.4)
    ax_ops = ax.twinx()
    if series["n_ops_t"].size > 0:
        ax_ops.step(series["n_ops_t"], series["n_ops_val"], where="post", color="#111827", linewidth=1.6, label="Operadores activos simultáneos")
    ax_ops.set_ylabel("# Operadores activos", color="#111827")
    ax_ops.set_ylim(-0.1, max(2.4, float(np.max(series["n_ops_val"]) if series["n_ops_val"].size else 2) + 0.5))
    ax_ops.set_yticks([0, 1, 2])
    ax_ops.grid(False)
    coop = summary["cooperation"]
    ax.set_title(
        f"Estados de la sala y concurrencia · operadores distintos: {coop['distinct_operators']} · "
        f"solapamiento ≥2 ops: {coop['multi_operator_overlap_sec']} s ({coop['multi_operator_overlap_pct_of_occupied']}%)"
    )
    seen_states = [st for st in ROOM_STATES_ORDER if summary["room_state_dwell_sec"].get(st, 0.0) > 0]
    top_handles = [mpatches.Patch(color=ROOM_STATE_COLORS[st], label=st) for st in seen_states]
    if series["n_ops_t"].size > 0:
        top_handles.append(plt.Line2D([0], [0], color="#111827", linewidth=1.6, label="# Operadores activos"))
    if top_handles:
        ax.legend(handles=top_handles, loc="upper left", ncol=min(len(top_handles), 4), fontsize=8.0, framealpha=0.92)

    ax = axes[1]
    palette = ["#6366f1", "#ec4899", "#14b8a6", "#f59e0b", "#3b82f6", "#8b5cf6"]
    op_colors = {op: palette[i % len(palette)] for i, op in enumerate(series["operators"])}
    for row_idx, axis_name in enumerate(AXES_ORDER):
        for span in series["axis_spans"][axis_name]:
            if span.end <= span.start:
                continue
            ax.barh(
                row_idx,
                span.end - span.start,
                left=span.start,
                height=0.62,
                color=op_colors.get(span.label, "#475569"),
                alpha=0.9,
            )
    ax.set_yticks(range(len(AXES_ORDER)))
    ax.set_yticklabels(AXES_ORDER)
    ax.invert_yaxis()
    ax.set_xlabel("Tiempo desde inicio (s)")
    ax.set_xlim(0, max(dur, 1.0))
    ax.set_title("Cronograma de propiedad exclusiva (leases) por componente")
    if op_colors:
        patches = [mpatches.Patch(color=col, label=op) for op, col in op_colors.items()]
        ax.legend(handles=patches, loc="upper right", fontsize=8.5, title="Operador")
    else:
        ax.text(0.5, 0.5, "Sin reservas de ejes registradas", ha="center", va="center", transform=ax.transAxes)

    p4 = out_dir / "04_cooperacion_y_estados.png"
    fig.savefig(p4, dpi=300)
    plt.close(fig)
    out_paths.append(p4)

    # 5. Slide-friendly executive summary (4 intuitive panels)
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 8.2), constrained_layout=True)
    fig.suptitle(f"Resumen visual para diapositiva · {run_id}", fontsize=14, fontweight="bold")

    # Top-left: Network RTT buckets
    ax = axes[0, 0]
    rtt_arr = series["rtt_ms"]
    if rtt_arr.size > 0:
        b_labels = [
            "< 20 ms\nInstantáneo",
            "20–50 ms\nMuy rápido",
            "50–100 ms\nFluido",
            "100–200 ms\nLento Wi-Fi",
            "> 200 ms\nPico Wi-Fi",
        ]
        b_counts = [
            float(np.mean(rtt_arr < 20.0) * 100.0),
            float(np.mean((rtt_arr >= 20.0) & (rtt_arr < 50.0)) * 100.0),
            float(np.mean((rtt_arr >= 50.0) & (rtt_arr < 100.0)) * 100.0),
            float(np.mean((rtt_arr >= 100.0) & (rtt_arr < 200.0)) * 100.0),
            float(np.mean(rtt_arr >= 200.0) * 100.0),
        ]
        b_colors = ["#16a34a", "#22c55e", "#eab308", "#f97316", "#ef4444"]
        bars = ax.bar(range(len(b_labels)), b_counts, color=b_colors, width=0.62, edgecolor="white")
        ax.set_xticks(range(len(b_labels)))
        ax.set_xticklabels(b_labels, fontsize=8.5)
        ax.set_ylim(0, max(b_counts) * 1.28 + 8.0)
        ax.set_ylabel("% de mensajes")
        for bar, val in zip(bars, b_counts):
            ax.text(
                bar.get_x() + bar.get_width() / 2.0,
                val + 1.2,
                f"{val:.1f}%",
                ha="center",
                va="bottom",
                fontsize=9,
                fontweight="bold",
            )
        rtt_cat_handles = [
            mpatches.Patch(color="#16a34a", label="Instantáneo (<20 ms)"),
            mpatches.Patch(color="#22c55e", label="Muy rápido (20–50 ms)"),
            mpatches.Patch(color="#eab308", label="Fluido (50–100 ms)"),
            mpatches.Patch(color="#ef4444", label="Lento / pico (>100 ms)"),
        ]
        ax.legend(handles=rtt_cat_handles, loc="upper right", fontsize=8.0, framealpha=0.92)
        ax.set_title(f"¿Qué tan rápida fue la red? (n={rtt_arr.size} pings)")
    else:
        ax.axis("off")
        ax.text(
            0.5,
            0.5,
            "Latencia RTT WebSocket no capturada\nen esta sesión (cliente anterior).",
            ha="center",
            va="center",
            fontsize=11,
            bbox={"boxstyle": "round,pad=0.6", "facecolor": "#f1f5f9", "edgecolor": "#cbd5e1"},
            transform=ax.transAxes,
        )
        ax.set_title("¿Qué tan rápida fue la red?")

    # Top-right: Per-client RTT (Median vs p95)
    ax = axes[0, 1]
    by_client = summary.get("network_rtt", {}).get("by_client", {})
    op_owners = set(coop["operators_axes"].keys())
    clients = [
        (name, st)
        for name, st in by_client.items()
        if st.get("count", 0) >= 20
    ]
    clients.sort(key=lambda item: (item[0] in op_owners, item[1]["count"]), reverse=True)
    clients = clients[:4]
    if clients:
        x_pos = np.arange(len(clients))
        w = 0.34
        meds = [st["p50"] for _, st in clients]
        p95s = [st["p95"] for _, st in clients]
        b1 = ax.bar(x_pos - w / 2, meds, width=w, color="#4f46e5", label="Mediana (típica)")
        b2 = ax.bar(x_pos + w / 2, p95s, width=w, color="#f59e0b", label="95% de mensajes (p95)")
        ax.axhline(100.0, color="#16a34a", linestyle="--", linewidth=1.2, label="Límite fluido (100 ms)")
        y_top = max(max(p95s), 100.0) * 1.22
        ax.set_ylim(0, y_top)
        ax.set_xticks(x_pos)
        ax.set_xticklabels([name for name, _ in clients], fontsize=9)
        ax.set_ylabel("Latencia ida y vuelta (ms)")
        for bar, val in list(zip(b1, meds)) + list(zip(b2, p95s)):
            ax.text(
                bar.get_x() + bar.get_width() / 2.0,
                val + y_top * 0.02,
                f"{val:.0f} ms",
                ha="center",
                va="bottom",
                fontsize=8.5,
                fontweight="bold",
            )
        ax.legend(loc="upper left", fontsize=8.5)
        ax.set_title("Latencia por dispositivo conectado")
    else:
        ax.axis("off")
        ax.text(
            0.5,
            0.5,
            "Sin suficientes muestras RTT por dispositivo\n(mínimo 20 muestras).",
            ha="center",
            va="center",
            fontsize=10.5,
            bbox={"boxstyle": "round,pad=0.6", "facecolor": "#f1f5f9", "edgecolor": "#cbd5e1"},
            transform=ax.transAxes,
        )
        ax.set_title("Latencia por dispositivo conectado")

    # Bottom-left: Active command time per axis stacked by operator
    ax = axes[1, 0]
    axis_times = kin.get("commanded_time_by_axis_sec", {})
    axis_owner_times = kin.get("commanded_time_by_axis_owner_sec", {})
    op_list = sorted(coop["operators_axes"].keys())
    op_palette = {op_name: palette[idx % len(palette)] for idx, op_name in enumerate(op_list)}
    y_ax = np.arange(len(AXES_ORDER))
    vals_ax = [float(axis_times.get(axis_name, 0.0)) for axis_name in AXES_ORDER]
    max_ax = max(vals_ax) if vals_ax else 1.0
    if op_list and any(axis_owner_times.get(ax) for ax in AXES_ORDER):
        lefts = np.zeros(len(AXES_ORDER), dtype=float)
        for op_name in op_list:
            seg = np.array([float((axis_owner_times.get(ax) or {}).get(op_name, 0.0)) for ax in AXES_ORDER])
            if np.any(seg > 0):
                ax.barh(y_ax, seg, left=lefts, color=op_palette[op_name], height=0.62, label=op_name)
                lefts += seg
        rem = np.maximum(0.0, np.array(vals_ax) - lefts)
        if np.any(rem > 0.05):
            ax.barh(y_ax, rem, left=lefts, color="#94a3b8", height=0.62, label="Otro / sin dueño")
    else:
        ax.barh(y_ax, vals_ax, color="#94a3b8", height=0.62, label="Sin dueño")
    ax.set_yticks(y_ax)
    ax.set_yticklabels(AXES_ORDER, fontweight="bold")
    ax.invert_yaxis()
    ax.set_xlim(0, max(max_ax * 1.26, 5.0))
    ax.set_xlabel("Tiempo en movimiento activo (s)")
    ax.set_title("Uso real por componente y quién lo controló")
    for row_i, val in enumerate(vals_ax):
        ax.text(
            val + max(max_ax * 0.02, 0.2),
            row_i,
            f"{val:.1f} s",
            va="center",
            ha="left",
            fontsize=9,
            fontweight="bold",
        )
    if op_palette:
        leg_patches = [mpatches.Patch(color=col, label=op_name) for op_name, col in op_palette.items()]
        ax.legend(handles=leg_patches, loc="lower right", fontsize=8.5, title="Operador", framealpha=0.92)

    # Bottom-right: 4 System KPI progress bars (0-100%)
    ax = axes[1, 1]
    br = summary["direct_bridge"]
    br_hz = float(br.get("effective_hz", {}).get("mean", 0.0)) if br.get("available") else 0.0
    br_pct = min(100.0, br_hz)
    tel_ok_pct = float(summary["pose_age_ms"].get("within_300ms_pct", 0.0))
    inp_ok = int(summary["protocol_outcomes"].get("input:input_accepted", 0))
    inp_tot = int(summary["protocol_messages"].get("input", 0))
    inp_pct = (inp_ok / inp_tot * 100.0) if inp_tot > 0 else 0.0
    coop_pct = float(coop.get("multi_operator_overlap_pct_of_occupied", 0.0))

    kpi_items = [
        ("USB Kinova\n(meta 100 Hz)", br_pct, f"{br_hz:.1f} Hz · {br.get('errors', 0)} err." if br.get("available") else "No disponible"),
        ("Telemetría a\ntiempo (≤300 ms)", tel_ok_pct, f"{tel_ok_pct:.1f}% · med {summary['pose_age_ms'].get('mean', 0):.0f} ms"),
        ("Comandos\naceptados", inp_pct, f"{inp_pct:.1f}% ({inp_ok}/{inp_tot})"),
        ("2 dispositivos\na la vez", coop_pct, f"{coop_pct:.1f}% ({coop['multi_operator_overlap_sec']:.1f} s)"),
    ]
    y_kpi = np.arange(len(kpi_items))
    ax.barh(y_kpi, [100.0] * len(kpi_items), color="#e2e8f0", height=0.56, label="Meta ideal (100%)")
    kpi_colors = ["#0d9488", "#0284c7", "#16a34a", "#6366f1"]
    ax.barh(y_kpi, [item[1] for item in kpi_items], color=kpi_colors, height=0.56)
    ax.set_yticks(y_kpi)
    ax.set_yticklabels([item[0] for item in kpi_items], fontsize=9.5, fontweight="bold")
    ax.invert_yaxis()
    ax.set_xlim(0, 172)
    ax.set_xticks([0, 25, 50, 75, 100])
    ax.set_xlabel("Cumplimiento (%)")
    ax.set_title("Indicadores de estabilidad y cooperación")
    for idx, (_, pct, detail) in enumerate(kpi_items):
        ax.text(max(pct, 2.0) + 3.0, idx, detail, va="center", ha="left", fontsize=9, fontweight="bold", color="#0f172a")
    kpi_leg = [
        mpatches.Patch(color="#0284c7", label="Valor medido"),
        mpatches.Patch(color="#e2e8f0", label="Meta 100%"),
    ]
    ax.legend(handles=kpi_leg, loc="lower right", fontsize=8.0, framealpha=0.92)

    p5 = out_dir / "05_resumen_para_diapositiva.png"
    fig.savefig(p5, dpi=300)
    plt.close(fig)
    out_paths.append(p5)

    # 6. Security states & summary KPI cards
    fig, axes = plt.subplots(1, 2, figsize=(12.0, 5.4), gridspec_kw={"width_ratios": [1.05, 1.15]}, constrained_layout=True)
    fig.suptitle(f"Seguridad, estados del sistema y logros de la prueba · {run_id}", fontsize=13.5, fontweight="bold")

    ax = axes[0]
    friendly_state = {
        "ACTIVE": "Operación cooperativa\n(ACTIVE)",
        "READY": "Listo para operar\n(READY)",
        "SIM_ONLY": "Solo pinza simulada\n(SIM_ONLY)",
        "PAUSED": "Pausa recuperación\n(PAUSED)",
        "FAULT": "Paro / Fallo seguro\n(FAULT)",
        "NOT_READY": "Iniciando puente\n(NOT_READY)",
    }
    dwell_items = [
        (st, float(summary["room_state_dwell_sec"].get(st, 0.0)))
        for st in ("ACTIVE", "READY", "SIM_ONLY", "PAUSED", "FAULT", "NOT_READY")
        if summary["room_state_dwell_sec"].get(st, 0.0) > 0
    ]
    if dwell_items:
        y_st = np.arange(len(dwell_items))
        st_vals = [sec for _, sec in dwell_items]
        st_cols = [ROOM_STATE_COLORS.get(st, "#64748b") for st, _ in dwell_items]
        bars_st = ax.barh(y_st, st_vals, color=st_cols, height=0.58)
        ax.set_yticks(y_st)
        ax.set_yticklabels([friendly_state.get(st, st) for st, _ in dwell_items], fontsize=9.5)
        ax.invert_yaxis()
        max_st = max(st_vals)
        ax.set_xlim(0, max_st * 1.38 + 1.0)
        ax.set_xlabel("Tiempo acumulado (s)")
        ax.set_title(f"Distribución del tiempo total ({dur:.1f} s)")
        st_patches = [mpatches.Patch(color=ROOM_STATE_COLORS.get(st, "#64748b"), label=st) for st, _ in dwell_items]
        ax.legend(handles=st_patches, loc="lower right", fontsize=8.0, title="Estado", framealpha=0.92)
        for bar, val in zip(bars_st, st_vals):
            pct = (val / dur * 100.0) if dur > 0 else 0.0
            ax.text(
                val + max_st * 0.025,
                bar.get_y() + bar.get_height() / 2.0,
                f"{val:.1f} s ({pct:.1f}%)",
                va="center",
                ha="left",
                fontsize=9,
                fontweight="bold",
            )
    else:
        ax.axis("off")

    ax_cards = axes[1]
    ax_cards.axis("off")
    ax_cards.set_xlim(0, 1)
    ax_cards.set_ylim(0, 1)
    ax_cards.set_title("Resultados clave de la sesión")

    vt = summary["virtual_task"]
    stops_n = int(summary["protocol_messages"].get("stop_global", 0))
    faults_n = sum(v for k, v in summary["state_transitions"].items() if k.endswith("->FAULT"))
    pauses_n = sum(v for k, v in summary["state_transitions"].items() if k.endswith("->PAUSED"))
    err_total = sum(summary["protocol_errors"].values()) if summary["protocol_errors"] else 0
    err_label = "1 error" if err_total == 1 else f"{err_total} errores"
    cards = [
        (0.03, 0.53, 0.44, 0.42, "#eef2ff", "#6366f1", "TRAYECTORIA 3D REAL",
         f"{kin.get('path_length_m', 0):.2f} m",
         f"Desplaz. máx: {kin.get('max_displacement_from_start_cm', 0):.1f} cm\nRango: ΔX {kin.get('excursion_cm', {}).get('X', 0):.0f} · ΔY {kin.get('excursion_cm', {}).get('Y', 0):.0f} · ΔZ {kin.get('excursion_cm', {}).get('Z', 0):.0f} cm"),
        (0.53, 0.53, 0.44, 0.42, "#ecfeff", "#0284c7", "VELOCIDADES PICO",
         f"{kin.get('peak_linear_speed_mm_s', 0):.0f} mm/s",
         f"Rotación máx: {kin.get('peak_angular_speed_deg_s', 0):.1f} °/s\nMov. activo: {kin.get('active_linear_motion_sec', 0):.1f} s lin · {kin.get('active_angular_motion_sec', 0):.1f} s ang"),
        (0.03, 0.05, 0.44, 0.42, "#f0fdf4", "#16a34a", "TAREA VIRTUAL (PYBULLET)",
         f"{vt.get('delivered_max', 0)} / {vt.get('total', 5)}",
         f"Muñecos entregados\nAgarres: {vt.get('grabs_detected', 0)} · Pinza: {summary['simulated_gripper'].get('max_rad', 0):.2f} rad"),
        (0.53, 0.05, 0.44, 0.42, "#fffbeb", "#d97706", "SEGURIDAD FAIL-SAFE",
         err_label,
         f"STOP: {stops_n} · Pausas PAUSED: {pauses_n}\nFallos FAULT: {faults_n} · SDK errs: {br.get('errors', 0)}"),
    ]
    for x0, y0, cw, ch, bg, border, heading, big_val, sub_txt in cards:
        patch = mpatches.FancyBboxPatch(
            (x0, y0), cw, ch,
            boxstyle="round,pad=0.02,rounding_size=0.04",
            facecolor=bg,
            edgecolor=border,
            linewidth=1.8,
        )
        ax_cards.add_patch(patch)
        ax_cards.text(x0 + cw / 2, y0 + ch * 0.80, heading, ha="center", va="center", fontsize=8.5, fontweight="bold", color=border)
        ax_cards.text(x0 + cw / 2, y0 + ch * 0.50, big_val, ha="center", va="center", fontsize=19, fontweight="bold", color="#0f172a")
        ax_cards.text(x0 + cw / 2, y0 + ch * 0.19, sub_txt, ha="center", va="center", fontsize=8.2, color="#334155")

    p6 = out_dir / "06_resumen_seguridad_y_tarea.png"
    fig.savefig(p6, dpi=300)
    plt.close(fig)
    out_paths.append(p6)

    # 7. IEEE-style RTT, CDF, Boxplot & Jitter Analysis (ultra clear with legends)
    fig, axes = plt.subplots(2, 2, figsize=(13.0, 8.8), constrained_layout=True)
    fig.suptitle(f"Análisis de Latencia de Red (RTT) y Jitter · {run_id}", fontsize=14, fontweight="bold")

    rtt_all = series["rtt_ms"]
    rtt_by_client = series.get("rtt_by_client", {})
    active_clients = [
        (name, arr)
        for name, arr in rtt_by_client.items()
        if arr.size >= 20
    ]
    active_clients.sort(key=lambda item: (item[0] in coop["operators_axes"], item[1].size), reverse=True)
    active_clients = active_clients[:4]
    client_colors = ["#4f46e5", "#ec4899", "#0d9488", "#f59e0b"]

    # Top-left: Cumulative Distribution Function (CDF) with color-coded zones
    ax = axes[0, 0]
    if rtt_all.size > 0:
        x_lim = max(120.0, float(np.percentile(rtt_all, 99)) * 1.18)
        ax.axvspan(0, 50, color="#dcfce7", alpha=0.55, label="Zona muy fluida (0–50 ms)")
        ax.axvspan(50, 100, color="#fef9c3", alpha=0.55, label="Zona fluida (50–100 ms)")
        ax.axvspan(100, x_lim, color="#fee2e2", alpha=0.45, label="Retardo Wi-Fi (>100 ms)")
        ax.axvline(100.0, color="#dc2626", linestyle="--", linewidth=1.2, label="Umbral fluido (100 ms)")
        ax.axhline(95.0, color="#64748b", linestyle=":", linewidth=1.0, label="Percentil 95%")

        sx = np.sort(rtt_all)
        sy = np.linspace(0.0, 100.0, sx.size)
        ax.plot(sx, sy, color="#0f172a", linewidth=2.0, label=f"Global (p50={np.percentile(rtt_all, 50):.1f}, p95={np.percentile(rtt_all, 95):.1f} ms)")
        for idx, (cname, carr) in enumerate(active_clients):
            cx = np.sort(carr)
            cy = np.linspace(0.0, 100.0, cx.size)
            ax.plot(cx, cy, color=client_colors[idx % len(client_colors)], linewidth=1.5, linestyle="-.", label=f"{cname} (p95={np.percentile(carr, 95):.1f} ms)")
        ax.set_xlim(0, x_lim)
        ax.set_ylim(0, 102)
        ax.set_xlabel("Latencia ida y vuelta RTT (ms)")
        ax.set_ylabel("% acumulado de mensajes")
        ax.set_title("Curva acumulada (CDF): ¿qué % llegó antes de X ms?")
        ax.legend(loc="lower right", fontsize=7.6, framealpha=0.94)
    else:
        ax.axis("off")
        ax.text(0.5, 0.5, "Sin datos de RTT WebSocket en esta sesión.", ha="center", va="center", transform=ax.transAxes)

    # Top-right: Per-client Boxplot + p95 markers
    ax = axes[0, 1]
    if active_clients:
        data_bp = [arr for _, arr in active_clients]
        bp = ax.boxplot(data_bp, patch_artist=True, showfliers=False, widths=0.5)
        for idx, box in enumerate(bp["boxes"]):
            c = client_colors[idx % len(client_colors)]
            box.set_facecolor(c)
            box.set_alpha(0.35)
            box.set_edgecolor(c)
            box.set_linewidth(1.5)
        for med in bp["medians"]:
            med.set_color("#0f172a")
            med.set_linewidth(2.0)
        p95_vals = [float(np.percentile(arr, 95)) for _, arr in active_clients]
        p50_vals = [float(np.percentile(arr, 50)) for _, arr in active_clients]
        xs_bp = np.arange(1, len(active_clients) + 1)
        ax.scatter(xs_bp, p95_vals, marker="D", s=45, color="#dc2626", zorder=5, label="Percentil 95 (p95)")
        ax.axhline(100.0, color="#16a34a", linestyle="--", linewidth=1.2, label="Umbral fluido (100 ms)")
        y_hi = max(max(p95_vals), 100.0) * 1.25
        ax.set_ylim(0, y_hi)
        for x_i, p50_v, p95_v in zip(xs_bp, p50_vals, p95_vals):
            ax.text(x_i, p95_v + y_hi * 0.03, f"p50={p50_v:.1f}\np95={p95_v:.1f}", ha="center", va="bottom", fontsize=8.0, fontweight="bold")
        ax.set_xticks(xs_bp)
        ax.set_xticklabels([f"{name}\n(n={arr.size})" for name, arr in active_clients], fontsize=8.5)
        ax.set_ylabel("RTT (ms)")
        ax.set_title("Dispersión de latencia por equipo (caja = 25%–75%)")
        bp_handles = [
            mpatches.Patch(color="#4f46e5", alpha=0.35, label="Rango central (25%–75%)"),
            plt.Line2D([0], [0], color="#0f172a", linewidth=2.0, label="Mediana (p50)"),
            plt.Line2D([0], [0], marker="D", color="w", markerfacecolor="#dc2626", markersize=7, label="Percentil 95 (p95)"),
            plt.Line2D([0], [0], color="#16a34a", linestyle="--", linewidth=1.2, label="Umbral fluido (100 ms)"),
        ]
        ax.legend(handles=bp_handles, loc="upper left", fontsize=8.0, framealpha=0.94)
    else:
        ax.axis("off")
        ax.text(0.5, 0.5, "Sin suficientes muestras RTT por dispositivo.", ha="center", va="center", transform=ax.transAxes)

    # Bottom-left: Jitter per client (mean |ΔRTT| and std σ)
    ax = axes[1, 0]
    j_by_client = summary.get("network_rtt", {}).get("jitter_inter_packet_ms", {}).get("by_client", {})
    if active_clients and j_by_client:
        xs_j = np.arange(len(active_clients))
        w = 0.34
        j_means = [float((j_by_client.get(name) or {}).get("mean", 0.0)) for name, _ in active_clients]
        j_stds = [float((summary["network_rtt"]["by_client"].get(name) or {}).get("std", 0.0)) for name, _ in active_clients]
        b_jm = ax.bar(xs_j - w / 2, j_means, width=w, color="#0284c7", label="Jitter entre paquetes |ΔRTT| medio")
        b_js = ax.bar(xs_j + w / 2, j_stds, width=w, color="#f59e0b", label="Desviación estándar σ(RTT)")
        y_jtop = max(max(j_means), max(j_stds), 10.0) * 1.28
        ax.set_ylim(0, y_jtop)
        for bar, val in list(zip(b_jm, j_means)) + list(zip(b_js, j_stds)):
            ax.text(bar.get_x() + bar.get_width() / 2.0, val + y_jtop * 0.02, f"{val:.1f} ms", ha="center", va="bottom", fontsize=8.5, fontweight="bold")
        ax.set_xticks(xs_j)
        ax.set_xticklabels([name for name, _ in active_clients], fontsize=9.0)
        ax.set_ylabel("Variación / Jitter (ms)")
        ax.set_title("Jitter por dispositivo (qué tanto varía el retardo)")
        ax.legend(loc="upper left", fontsize=8.2, framealpha=0.94)
    else:
        ax.axis("off")
        ax.text(0.5, 0.5, "Sin datos de jitter disponibles.", ha="center", va="center", transform=ax.transAxes)

    # Bottom-right: Internal gateway dt & pose_age stability summary
    ax = axes[1, 1]
    dt_st = summary["gateway_tick_dt_ms"]
    pa_st = summary["pose_age_ms"]
    if dt_st.get("count", 0) > 0 and pa_st.get("count", 0) > 0:
        metrics_lbl = ["Periodo Gateway\n(dt, meta 20 ms)", "Edad de postura\n(pose_age, ≤300 ms)"]
        xs_i = np.arange(len(metrics_lbl))
        w = 0.24
        meds_i = [dt_st["p50"], pa_st["p50"]]
        p95s_i = [dt_st["p95"], pa_st["p95"]]
        maxs_i = [dt_st["max"], pa_st["max"]]
        bi1 = ax.bar(xs_i - w, meds_i, width=w, color="#16a34a", label="Mediana (p50)")
        bi2 = ax.bar(xs_i, p95s_i, width=w, color="#4f46e5", label="Percentil 95 (p95)")
        bi3 = ax.bar(xs_i + w, maxs_i, width=w, color="#f97316", label="Máximo observado")
        ax.axhline(300.0, color="#dc2626", linestyle="--", linewidth=1.2, label="Límite seguridad watchdog (300 ms)")
        y_itop = max(max(maxs_i), 300.0) * 1.18
        ax.set_ylim(0, y_itop)
        for bar, val in list(zip(bi1, meds_i)) + list(zip(bi2, p95s_i)) + list(zip(bi3, maxs_i)):
            ax.text(bar.get_x() + bar.get_width() / 2.0, val + y_itop * 0.015, f"{val:.1f}", ha="center", va="bottom", fontsize=8.2, fontweight="bold")
        ax.set_xticks(xs_i)
        ax.set_xticklabels(metrics_lbl, fontsize=9.0, fontweight="bold")
        ax.set_ylabel("Tiempo (ms)")
        ax.set_title("Tiempos internos de la estación madre vs. umbral")
        ax.legend(loc="upper left", fontsize=8.0, framealpha=0.94)
    else:
        ax.axis("off")

    p7 = out_dir / "07_analisis_latencia_rtt_jitter.png"
    fig.savefig(p7, dpi=300)
    plt.close(fig)
    out_paths.append(p7)

    # 8. Course-style (1MTR74 plot_jitter.py) RTT & RFC 3550 Jitter plots
    def _plot_course_rtt_jitter_column(
        axes_col: tuple[Any, Any, Any],
        rtt_win: np.ndarray,
        rtt_full: np.ndarray,
        title_prefix: str = "",
        use_markers: bool = True,
    ) -> None:
        ax_rtt, ax_jit, ax_hist = axes_col
        inst_w, rfc_w = compute_rfc3550_jitter(rtt_win)
        idx = np.arange(rtt_win.size)

        # Row 0: Raw RTT over Samples
        ax_rtt.plot(
            idx,
            rtt_win,
            color="#3b82b6",
            linewidth=0.95 if use_markers else 0.55,
            alpha=0.82 if use_markers else 0.65,
            marker="o" if use_markers else None,
            markersize=2.8 if use_markers else 0.0,
            label="Raw RTT",
        )
        ax_rtt.grid(True, linestyle=":", alpha=0.55)
        ax_rtt.set_ylabel("RTT (ms)")
        ax_rtt.set_xlabel("Sample Index")
        ax_rtt.set_title(f"{title_prefix}Round-Trip Time (RTT) over Samples (n={rtt_win.size})")
        ax_rtt.legend(loc="upper right", fontsize=8.0, framealpha=0.92)

        # Row 1: Instantaneous Jitter |ΔRTT| + RFC 3550 Smoothed Jitter
        ax_jit.plot(
            idx,
            inst_w,
            color="#f59e0b",
            linewidth=0.85 if use_markers else 0.45,
            alpha=0.68 if use_markers else 0.45,
            marker="x" if use_markers else None,
            markersize=3.0 if use_markers else 0.0,
            label="Instantaneous Jitter |ΔRTT|",
        )
        ax_jit.plot(
            idx,
            rfc_w,
            color="#b91c1c",
            linewidth=1.9,
            label="RFC 3550 Smoothed Jitter",
        )
        ax_jit.grid(True, linestyle=":", alpha=0.55)
        ax_jit.set_ylabel("Jitter (ms)")
        ax_jit.set_xlabel("Sample Index")
        ax_jit.set_title(f"{title_prefix}Jitter (Latency Variance) over Samples")
        ax_jit.legend(loc="upper right", fontsize=8.0, framealpha=0.92)

        # Row 2: Latency Distribution (Histogram / Density) + Mean & +1σ
        mean_v = float(np.mean(rtt_full))
        std_v = float(np.std(rtt_full))
        plus1 = mean_v + std_v
        x_lo = max(0.0, float(np.min(rtt_full)) * 0.85)
        x_hi = max(float(np.percentile(rtt_full, 99)) * 1.15, plus1 * 1.25, 45.0)
        ax_hist.hist(
            rtt_full,
            bins=26,
            range=(x_lo, x_hi),
            density=True,
            color="#a78bfa",
            alpha=0.72,
            edgecolor="#4c3a6e",
            linewidth=0.75,
        )
        ax_hist.axvline(mean_v, color="#dc2626", linestyle="--", linewidth=1.6, label=f"Mean RTT: {mean_v:.1f} ms")
        ax_hist.axvline(plus1, color="#f59e0b", linestyle=":", linewidth=1.7, label=f"+1σ ({plus1:.1f} ms)")
        ax_hist.grid(True, linestyle=":", alpha=0.55)
        ax_hist.set_ylabel("Probability Density")
        ax_hist.set_xlabel("RTT (ms)")
        ax_hist.set_title(f"{title_prefix}Latency Distribution (Histogram / Density · n={rtt_full.size})")
        ax_hist.legend(loc="upper right", fontsize=8.0, framealpha=0.92)

    if active_clients:
        rtt_t_by_client = series.get("rtt_t_by_client", {})
        t_coop: float | None = None
        ops_t = series["n_ops_t"]
        ops_val = series["n_ops_val"]
        if ops_t.size > 0 and ops_val.size == ops_t.size:
            idxs_2op = np.flatnonzero(ops_val >= 2)
            if idxs_2op.size > 0:
                t_coop = float(ops_t[idxs_2op[0]]) + 15.0

        top_clients = active_clients[:2]
        client_windows: list[tuple[str, np.ndarray, np.ndarray]] = []
        for idx_c, (cname, carr) in enumerate(top_clients):
            t_arr = rtt_t_by_client.get(cname, np.zeros(0, dtype=float))
            win_len = min(120, int(carr.size))
            start_i = 0
            if t_coop is not None and t_arr.size == carr.size:
                match_idx = int(np.searchsorted(t_arr, t_coop, side="left"))
                if match_idx < carr.size:
                    start_i = min(match_idx, max(0, int(carr.size) - win_len))
            win_arr = carr[start_i : start_i + win_len]
            client_windows.append((cname, win_arr, carr))

            slug = re.sub(r"[^A-Za-z0-9_.-]+", "_", cname).strip("_") or f"cliente_{idx_c + 1}"
            prefix_letter = chr(ord("a") + idx_c)
            fig_c, axes_c = plt.subplots(3, 1, figsize=(10.0, 8.6), constrained_layout=True)
            fig_c.suptitle(f"Perfil de Latencia RTT y Jitter (RFC 3550) · {cname} · {run_id}", fontsize=13.5, fontweight="bold")
            _plot_course_rtt_jitter_column((axes_c[0], axes_c[1], axes_c[2]), win_arr, carr, title_prefix="", use_markers=True)
            p_c = out_dir / f"08{prefix_letter}_rtt_jitter_curso_{slug}.png"
            fig_c.savefig(p_c, dpi=300)
            plt.close(fig_c)
            out_paths.append(p_c)

        # Combined side-by-side comparison for active cooperative clients
        n_cols = len(client_windows)
        fig_cmp, axes_cmp = plt.subplots(
            3, n_cols, figsize=(7.0 * n_cols if n_cols > 1 else 10.0, 8.8), squeeze=False, constrained_layout=True
        )
        fig_cmp.suptitle(f"Latencia RTT y Jitter RFC 3550 por Equipo (ventana cooperativa n=120) · {run_id}", fontsize=13.5, fontweight="bold")
        for col_i, (cname, win_arr, carr) in enumerate(client_windows):
            _plot_course_rtt_jitter_column(
                (axes_cmp[0, col_i], axes_cmp[1, col_i], axes_cmp[2, col_i]),
                win_arr,
                carr,
                title_prefix=f"{cname} · ",
                use_markers=True,
            )
        p8 = out_dir / "08_rtt_jitter_curso_comparativo.png"
        fig_cmp.savefig(p8, dpi=300)
        plt.close(fig_cmp)
        out_paths.append(p8)

        # Full-session profile for the primary client
        prim_name, _, prim_full = client_windows[0]
        fig_full, axes_full = plt.subplots(3, 1, figsize=(10.5, 8.6), constrained_layout=True)
        fig_full.suptitle(f"Perfil Completo de Sesión RTT y Jitter RFC 3550 · {prim_name} · {run_id}", fontsize=13.5, fontweight="bold")
        _plot_course_rtt_jitter_column((axes_full[0], axes_full[1], axes_full[2]), prim_full, prim_full, title_prefix="", use_markers=False)
        p8c = out_dir / "08c_rtt_jitter_curso_sesion_completa.png"
        fig_full.savefig(p8c, dpi=300)
        plt.close(fig_full)
        out_paths.append(p8c)

    return out_paths


def build_markdown_report(summary: dict[str, Any], out_dir: Path) -> Path:
    """Write RESUMEN_PRESENTACION.md mapped directly to the 4 Slide-4 grading items."""
    run_id = summary["run_id"]
    dt = summary["gateway_tick_dt_ms"]
    pa = summary["pose_age_ms"]
    rtt = summary["network_rtt"]
    br = summary["direct_bridge"]
    kin = summary["kinematics"]
    coop = summary["cooperation"]
    vt = summary["virtual_task"]
    grip = summary["simulated_gripper"]
    proto = summary["protocol_messages"]
    outcomes = summary["protocol_outcomes"]
    errs = summary["protocol_errors"]

    sources_str = ", ".join(f"`{k}` ({v} muestras)" for k, v in summary["telemetry_sources"].items()) or "Simulación / sin fuente"
    rtt_line = (
        f"Media **{rtt['overall_ms']['mean']} ms** · p50 **{rtt['overall_ms']['p50']} ms** · "
        f"p95 **{rtt['overall_ms']['p95']} ms** · máx **{rtt['overall_ms']['max']} ms** ({rtt['overall_ms']['count']} muestras)"
        if rtt.get("available")
        else "No capturado en esta corrida (sesión iniciada antes de habilitar `client_sent_ms`/`rtt_ms` en `heartbeat`)"
    )
    bridge_line = (
        f"{'Simulación (mock)' if br.get('mock') else 'Hardware físico USB'} · "
        f"**{br['transmissions_per_interval']['mean']} tx/s** media "
        f"(mín {br['transmissions_per_interval']['min']} · máx {br['transmissions_per_interval']['max']} Hz en {br['intervals_count']} intervalos)"
        if br.get("available") and br.get("transmissions_per_interval", {}).get("count", 0) > 0
        else "Sin registros de intervalo en `direct_bridge.log`"
    )

    op_rows = []
    for op, axes in coop["operators_axes"].items():
        op_rows.append(f"| `{op}` | {', '.join(axes)} |")
    op_table = "\n".join(op_rows) if op_rows else "| — | Sin reservas registradas |"

    joint_rows = []
    for jname, jstats in summary["joints_deg"].items():
        joint_rows.append(f"| {jname} | {jstats['min_deg']}° | {jstats['max_deg']}° | {jstats['range_deg']}° |")
    joint_table = "\n".join(joint_rows) if joint_rows else "| — | — | — | — |"

    state_rows = "\n".join(
        f"| `{st}` | {sec} s ({round(sec / summary['duration_sec'] * 100.0, 1) if summary['duration_sec'] > 0 else 0}%) |"
        for st, sec in summary["room_state_dwell_sec"].items()
    ) or "| — | — |"

    axis_cmd_rows = "\n".join(
        f"| `{ax}` | {sec} s |" for ax, sec in kin.get("commanded_time_by_axis_sec", {}).items()
    ) or "| — | — |"

    md = f"""# Resumen de Presentación · JACO2 Teleoperación Cooperativa (`{run_id}`)

> Generado automáticamente a partir de `{summary['run_dir']}` ({summary['start_utc']} → {summary['end_utc']}, duración total **{summary['duration_sec']} s**).

---

## 1. Protocolos utilizados y arquitectura de red

| Capa / Enlace | Protocolo / Transporte | Frecuencia / Watchdog | Función |
| --- | --- | --- | --- |
| Cliente web ↔ Estación madre | **HTTPS + WSS (WebSocket sobre TLS)** en puerto `8443` (`lan_https_proxy.py` + CA local `ca.crt`) | Heartbeat **10 Hz** (100 ms), `input` **20 Hz** (50 ms), telemetría **50 Hz** | Autenticación por rol (`Operador`, `Observador`, `Administrador`), aislamiento de IP local vs. LAN y transporte cifrado. |
| Gateway cooperativo (`gateway.py`) | Protocolo JSON `teleop_cooperativo` (`v1`) sobre WebSocket + relé interno `127.0.0.1:8765` | Lazo autoritativo a **50 Hz** (tick nominal 20 ms) + watchdog de movimiento **400 ms** / *lease* **1.5 s** | Árbitro con propiedad exclusiva (*leases* con `lease_id` y `generation`), composición de twist 6-DOF y `STOP` global. |
| Gateway ↔ Bridge ROS 2 | **ROS 2 Jazzy** (`/teleop/cartesian_twist`, tópicos de pose/joints y servicios de armado/recuperación) | Lazo ROS + verificación `pose_age_sec ≤ 0.30 s` | Cinemática inversa cartesiana (`kinova_cartesian_adapter`) y control seguro de activación (`enable_output`, `recovery`). |
| Estación madre ↔ Kinova JACO2 | **SDK USB Kinova (`USBCommandLayerUbuntu.so`)** mediante `kinova_direct_node` (C++) | **100 Hz** (10 ms por paquete USB) | Lectura de ángulos articulares reales `J1–J6`, envío de velocidades articulares y parada de emergencia USB. |

---

## 2. Implementación de código y metodologías

- **Arbitraje cooperativo por componentes (`arbiter.py`, `lease_manager.py`):** Hasta 2 operadores remotos reservan de 1 a 3 componentes disjuntos entre `X, Y, Z, ROLL, PITCH, YAW` y `PINZA · SIMULADA`. Cada comando `input` valida número de secuencia monótono, `lease_id`, `generation` y estado de la sala (`READY` / `ACTIVE` / `SIM_ONLY`).
- **Seguridad multicapa (*Fail-Safe*):**
  1. Salida física **desarmada al arranque**; requiere armado explícito en consola madre (`desktop_console.py`).
  2. **Puesta a cero automática** ante liberación de botón, pérdida de foco (`blur`/`pagehide`), expiración de *lease* o falla de telemetría (`pose_age_sec > 0.30 s`).
  3. **Modo recuperación por articulaciones (`PAUSED`):** pausa el control web preservando los *leases* mientras la estación madre mueve ejes individuales a ≤5°/s.
- **Gemelo 3D + tarea virtual (`desktop_qt.py`, `virtual_task.py`, `simulated_gripper.py`):** Vista VTK/Qt de postura real reconstruida por FK + simulación PyBullet de pinza de 3 dedos y traslado de 5 muñecos (`Luna`, `Cubo`, `Sol`, `Orejas`, `Casco`) a zonas objetivo.
- **Trazabilidad en reloj único (`event_log.py`):** Bitácora `events.jsonl` (`schema teleop_cooperativo.run_event.v1`) sincronizada al reloj monotónico de la estación madre, más logs del bridge USB C++.

---

## 3. Métricas adquiridas (`{run_id}`)

### 3.1 Desempeño de control, red y hardware

| Métrica | Valor medido |
| --- | --- |
| Duración de sesión analizada | **{summary['duration_sec']} s** |
| Fuente de telemetría | {sources_str} |
| Eventos registrados | **{summary['event_counts'].get('telemetry', 0)}** telemetría · **{summary['event_counts'].get('protocol', 0)}** protocolo |
| Periodo del gateway (`dt` entre ticks) | Media **{dt.get('mean', '—')} ms** (**{dt.get('effective_hz', '—')} Hz**) · σ = **{dt.get('std', '—')} ms** · p95 = **{dt.get('p95', '—')} ms** · máx = **{dt.get('max', '—')} ms** ({dt.get('within_15_25_ms_pct', '—')}% dentro de 15–25 ms) |
| Frescura de postura (`pose_age_sec`) | Media **{pa.get('mean', '—')} ms** · p50 = **{pa.get('p50', '—')} ms** · p95 = **{pa.get('p95', '—')} ms** · máx = **{pa.get('max', '—')} ms** ({pa.get('within_300ms_pct', '—')}% ≤ 300 ms) |
| Latencia de red WebSocket (RTT) | {rtt_line} |
| Tasa del bridge USB Kinova | {bridge_line} |
| Mensajes de protocolo | `hello`: {proto.get('hello', 0)} · `claim`: {proto.get('claim', 0)} · `input`: {proto.get('input', 0)} · `heartbeat`: {proto.get('heartbeat', 0)} · `release`: {proto.get('release', 0)} · `stop_global`: {proto.get('stop_global', 0)} · `rearm`: {proto.get('rearm', 0)} |
| Errores / rechazos de protocolo | `{errs if errs else '0 errores registrados'}` |

### 3.2 Cinemática del efector final, articulaciones y pinza

| Métrica cinemática | Valor medido |
| --- | --- |
| Posición inicial `(X, Y, Z)` | `{kin.get('start_xyz_m', '—')}` m |
| Posición final `(X, Y, Z)` | `{kin.get('end_xyz_m', '—')}` m |
| Excursión cartesiana `(ΔX, ΔY, ΔZ)` | `X`: **{kin.get('excursion_cm', {}).get('X', '—')} cm** · `Y`: **{kin.get('excursion_cm', {}).get('Y', '—')} cm** · `Z`: **{kin.get('excursion_cm', {}).get('Z', '—')} cm** |
| Recorrido 3D total / desplaz. máx. | **{kin.get('path_length_m', '—')} m** recorrido · **{kin.get('max_displacement_from_start_cm', '—')} cm** desde inicio |
| Velocidad lineal comandada | Activa **{kin.get('active_linear_motion_sec', '—')} s** · pico **{kin.get('peak_linear_speed_mm_s', '—')} mm/s** |
| Velocidad angular comandada | Activa **{kin.get('active_angular_motion_sec', '—')} s** · pico **{kin.get('peak_angular_speed_deg_s', '—')} °/s** |
| Pinza simulada (`F1–F3`) | Activa: **{grip.get('active', False)}** · rango **{grip.get('min_rad', '—')} – {grip.get('max_rad', '—')} rad** |
| Tarea virtual cooperativa | Entregados **{vt.get('delivered_max', 0)} / {vt.get('total', 5)}** · agarres detectados: **{vt.get('grabs_detected', 0)}** · sueltas por impacto: **{vt.get('impact_releases', 0)}** |

#### Tiempo comandado por eje y rangos articulares

| Eje | Tiempo activo |
| --- | --- |
{axis_cmd_rows}

| Articulación | Mínimo | Máximo | Rango recorrido |
| --- | --- | --- | --- |
{joint_table}

### 3.3 Cooperación multi-operador y estados de sala

- Operadores distintos que reservaron ejes: **{coop['distinct_operators']}** (máx. simultáneos: **{coop['max_simultaneous_operators']}**, máx. ejes reservados a la vez: **{coop['max_simultaneous_reserved_axes']}**).
- Tiempo con 1 operador activo: **{coop['single_operator_sec']} s** · Tiempo en solapamiento cooperativo (≥2 operadores simultáneos): **{coop['multi_operator_overlap_sec']} s** (**{coop['multi_operator_overlap_pct_of_occupied']}%** del tiempo ocupado).

| Operador | Ejes reservados |
| --- | --- |
{op_table}

| Estado de sala | Tiempo acumulado |
| --- | --- |
{state_rows}

### 3.4 Gráficos generados para diapositivas

- `01_frecuencia_y_latencia.png` — Estabilidad del lazo de 50 Hz, `pose_age_sec` vs watchdog (300 ms) y tasa del bridge USB (100 Hz) / RTT WebSocket.
- `02_trayectoria_3d_y_xyz.png` — Trayectoria 3D del efector final, evolución `X/Y/Z` y perfil de velocidad lineal comandada.
- `03_articulaciones_y_pinza.png` — Ángulos medidos `J1–J6`, velocidad angular comandada y apertura de pinza simulada.
- `04_cooperacion_y_estados.png` — Línea de tiempo de estados de sala, número de operadores simultáneos y cronograma de *leases* por eje.
- `05_resumen_para_diapositiva.png` — Resumen ejecutivo en 4 paneles claros (histograma de rapidez de red, latencia por equipo, uso por eje y barras de cumplimiento).
- `06_resumen_seguridad_y_tarea.png` — Distribución de tiempo por estado y tarjetas KPI (trayectoria, velocidad, tarea virtual y seguridad).
- `07_analisis_latencia_rtt_jitter.png` — Curva acumulada CDF de RTT, boxplot por dispositivo, jitter entre paquetes y tiempos internos.
- `08_rtt_jitter_curso_comparativo.png` / `08a` / `08b` / `08c` — Series de RTT crudo, jitter instantáneo `|ΔRTT|` + jitter suavizado RFC 3550 e histograma de densidad (`Mean RTT` y `+1σ`) en el formato `plot_jitter.py` del curso `1MTR74`.

---

## 4. Resultados, análisis y conclusiones

1. **Estabilidad en tiempo real del control distribuido:** El gateway sostuvo una frecuencia efectiva cercana a **{dt.get('effective_hz', '—')} Hz** (media **{dt.get('mean', '—')} ms**, **{dt.get('within_15_25_ms_pct', '—')}%** dentro de 15–25 ms) y el bridge USB sostuvo **{br.get('transmissions_per_interval', {}).get('mean', '—')} tx/s**, cumpliendo la tasa nominal de 100 Hz del Kinova JACO2 sin desbordamientos ni errores críticos del SDK (`errors={br.get('errors', 0)}`).
2. **Frescura de telemetría bajo el umbral de seguridad:** El **{pa.get('within_300ms_pct', '—')}%** de las muestras de postura llegaron por debajo del watchdog de 300 ms (media **{pa.get('mean', '—')} ms**, p95 **{pa.get('p95', '—')} ms**), validando que el cierre automático de seguridad ante telemetría vencida no introduce falsos disparos en operación normal.
3. **Ejecución segura y cooperativa:** El aislamiento estricto de componentes por *leases* (máx. 3 por operador, total **{outcomes.get('input:input_accepted', 0)}** comandos `input` aceptados) y el desarme inicial protegen al manipulador físico durante las pruebas en red local.
4. **Siguiente corrida recomendada (para completar la foto 100% de exposición):**
   {"- Esta corrida no tuvo solapamiento de 2 operadores ni mediciones RTT porque se inició antes de habilitar el registro de RTT y con 1 operador. Con los cambios ya aplicados en el código, al correr una sesión corta con **2 equipos remotos conectados simultáneamente** y ejecutar `python3 scripts/analyze_session_metrics.py`, este mismo reporte incluirá automáticamente las curvas de **RTT en ms por cliente** y el **porcentaje de solapamiento cooperativo de ≥2 operadores**." if (not rtt.get("available") or coop["max_simultaneous_operators"] < 2) else "- Esta corrida ya incluye mediciones completas de **RTT WebSocket** y **cooperación simultánea**, listas para presentarse en las diapositivas."}
"""

    md_path = out_dir / "RESUMEN_PRESENTACION.md"
    md_path.write_text(md, encoding="utf-8")
    return md_path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Extrae métricas y gráficos de presentación desde una sesión de teleop_cooperativo.")
    parser.add_argument("run_dir", nargs="?", help="Ruta a /tmp/teleop-base.<ID> (si se omite, toma la más reciente).")
    parser.add_argument("--output-dir", help="Carpeta destino para JSON, Markdown y gráficos PNG.")
    args = parser.parse_args(argv)

    run_dir = Path(args.run_dir).resolve() if args.run_dir else find_latest_run_dir()
    summary, series = load_and_analyze(run_dir)
    repo_root = Path(__file__).resolve().parent.parent
    out_dir = Path(args.output_dir).resolve() if args.output_dir else (repo_root / "reports" / summary["run_id"])
    out_dir.mkdir(parents=True, exist_ok=True)

    json_path = out_dir / "metrics_summary.json"
    json_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    png_paths = generate_plots(summary, series, out_dir)
    md_path = build_markdown_report(summary, out_dir)

    print(f"Sesión analizada: {run_dir} (run_id={summary['run_id']}, duración={summary['duration_sec']} s)")
    print(f"Resumen JSON:     {json_path}")
    print(f"Resumen Slides:   {md_path}")
    for path in png_paths:
        print(f"Gráfico PNG:      {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
