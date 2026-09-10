# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Animaquina Core - KUKA text streaming runtime (Blender-independent)

import math
import time

from .kuka_krl_parser import parse_krl_program


def _execute_ptp_command(driver, cmd) -> str:
    """Execute one parsed PTP command using Dynamic Sync commands."""
    ctype = str(cmd.get("type", "")).upper()
    if ctype == "PTP_JOINT":
        joints = list(cmd.get("joints", ()))
        if len(joints) < 6:
            return "PTP_JOINT requires 6 joint values"
        e6axis = driver._format_e6axis(joints)
        err = driver.write_var("MQ_E6AXIS", e6axis)
        if err:
            return err
        err = driver.write_var("MQ_APO", "-1")
        if err:
            return err
        cmd_id = driver._next_cmd_id()
        err = driver.write_var("MQ_CMD_ID", str(cmd_id))
        if err:
            return err
        err = driver.write_var("MQ_ACTION", "1")
        if err:
            return err
        return driver._wait_cmd_done(cmd_id, timeout=30.0)

    if ctype == "PTP":
        pos = tuple(cmd.get("pos", ()))
        if len(pos) < 6:
            return "PTP requires 6 pose values"
        x, y, z, a, b, c = pos
        pos_m = (float(x) / 1000.0, float(y) / 1000.0, float(z) / 1000.0)
        # Driver canonical Euler is Blender XYZ=(C,B,A) for KUKA data.
        euler_rad = (math.radians(float(c)), math.radians(float(b)), math.radians(float(a)))
        return driver.move_to_pose_ptp(pos_m, euler_rad, 15.0, 100.0, 0.0, True)

    return ""


def _write_var_events_for_lin(driver, lin_idx: int, var_at_lin: dict, vel_at_lin: dict) -> str:
    for name, value in var_at_lin.get(lin_idx, ()):
        err = driver.write_var(name, value)
        if err:
            return err
    vel = vel_at_lin.get(lin_idx, None)
    if vel is not None:
        err = driver.write_var("MQ_LIN_VEL", f"{float(vel):.4f}")
        if err:
            return err
    return ""


def _classify_krl_commands(commands: list, default_lin_speed: float, default_advance: float):
    """Split parsed KRL commands into init/PTP-before/LIN/PTP-after phases."""
    lin_waypoints_m = []
    var_at_lin = {}
    vel_at_lin = {}
    ptp_before = []
    ptp_after = []
    init_vars = []
    current_vel = None
    pending_vars = []
    pending_vel = None
    found_first_lin = False

    for cmd in commands:
        ctype = cmd.get("type")
        if ctype == "VAR":
            if not found_first_lin:
                init_vars.append(cmd)
            else:
                pending_vars.append(cmd)
            continue
        if ctype == "VEL":
            if not found_first_lin:
                current_vel = cmd["value"]
                init_vars.append(cmd)
            else:
                pending_vel = cmd["value"]
            continue
        if ctype == "LIN":
            idx = len(lin_waypoints_m)
            found_first_lin = True
            if pending_vars:
                var_at_lin[idx] = [(v["name"], v["value"]) for v in pending_vars]
                pending_vars = []
            if pending_vel is not None:
                vel_at_lin[idx] = pending_vel
                current_vel = pending_vel
                pending_vel = None
            x, y, z, a, b, c = cmd["pos"]
            lin_waypoints_m.append(
                (
                    (x / 1000.0, y / 1000.0, z / 1000.0),
                    (math.radians(c), math.radians(b), math.radians(a)),
                )
            )
            continue
        if ctype in ("PTP", "PTP_JOINT"):
            if not found_first_lin:
                ptp_before.append(cmd)
            else:
                ptp_after.append(cmd)

    if not lin_waypoints_m:
        return (None, "No LIN moves found in KRL program")

    lin_speed = float(current_vel) if current_vel is not None else float(default_lin_speed)
    apo_mm = float(default_advance)
    return (
        {
            "lin_waypoints_m": lin_waypoints_m,
            "var_at_lin": var_at_lin,
            "vel_at_lin": vel_at_lin,
            "init_vars": init_vars,
            "ptp_before": ptp_before,
            "ptp_after": ptp_after,
            "lin_speed": lin_speed,
            "apo_mm": apo_mm,
        },
        "",
    )


def stream_krl_text(
    driver,
    src_text: str,
    *,
    default_lin_speed: float = 0.05,
    default_advance: float = 3.0,
    ring_size: int = 32,
    uploaded_ring_size: int = 0,
    stream_program_uploaded: bool = False,
    remote_path: str = "",
    base_no: int = 0,
    tool_no: int = 0,
    advance: int = None,
    startup_delay_s: float = 1.0,
    poll_period_s: float = 0.02,
    mark_stream_uploaded=None,
) -> str:
    """Parse and stream a KRL .src text through KUKA Dynamic Sync.

    This function is intentionally Blender-independent and can be reused
    from CLI/scripts by providing a connected KUKA driver instance.
    """
    if driver is None:
        return "Driver is not available"
    text = str(src_text or "")
    if not text.strip():
        return "KRL text is empty"

    commands = parse_krl_program(text)
    if not commands:
        return "No moves found in KRL program"

    plan, err = _classify_krl_commands(commands, default_lin_speed, default_advance)
    if err:
        return err

    cfg_ring = int(max(8, min(128, int(ring_size or 32))))
    uploaded = int(max(0, int(uploaded_ring_size or 0)))
    effective_ring_size = uploaded if uploaded > 0 else cfg_ring
    remote = str(remote_path or "")

    if not bool(stream_program_uploaded):
        err = driver.upload_stream_program(
            remote,
            base_no=int(base_no),
            tool_no=int(tool_no),
            ring_buffer_size=effective_ring_size,
            advance=advance,
        )
        if err:
            return err
        if callable(mark_stream_uploaded):
            try:
                mark_stream_uploaded(int(effective_ring_size))
            except Exception:
                pass

    err = driver.select_stream_program(remote)
    if err:
        return err
    err = driver.play_program()
    if err:
        return err

    if startup_delay_s and startup_delay_s > 0:
        time.sleep(float(startup_delay_s))

    for cmd in plan["init_vars"]:
        if cmd["type"] == "VAR":
            err = driver.write_var(cmd["name"], cmd["value"])
            if err:
                return err
        elif cmd["type"] == "VEL":
            err = driver.write_var("MQ_LIN_VEL", f"{float(cmd['value']):.4f}")
            if err:
                return err

    for cmd in plan["ptp_before"]:
        err = _execute_ptp_command(driver, cmd)
        if err:
            return err

    waypoints = plan["lin_waypoints_m"]
    total = len(waypoints)
    apo_m = plan["apo_mm"] / 1000.0 if plan["apo_mm"] > 0 else 0.0

    initial = min(effective_ring_size, total)
    for idx in range(initial):
        err = _write_var_events_for_lin(driver, idx, plan["var_at_lin"], plan["vel_at_lin"])
        if err:
            return err

    state = driver.stream_start(
        total,
        plan["lin_speed"],
        apo_m,
        waypoints[:effective_ring_size],
        effective_ring_size,
    )
    if isinstance(state, str):
        return state

    var_written_up_to = initial - 1
    while state["written"] < state["total"]:
        try:
            rd_idx = int(driver.read_var("MQ_RD_IDX"))
        except Exception:
            time.sleep(poll_period_s)
            continue

        up_to = rd_idx + effective_ring_size
        start = var_written_up_to + 1
        stop = min(up_to + 1, total)
        for idx in range(start, stop):
            err = _write_var_events_for_lin(driver, idx, plan["var_at_lin"], plan["vel_at_lin"])
            if err:
                return err
        var_written_up_to = max(var_written_up_to, up_to)

        remaining = waypoints[state["written"]:]
        if remaining:
            _, err = driver.stream_refill(state, rd_idx, remaining)
            if err:
                return err
        else:
            time.sleep(poll_period_s)

    done_deadline = time.time() + max(30.0, float(total) * 0.5)
    while time.time() < done_deadline:
        if driver.stream_is_done(state):
            break
        time.sleep(poll_period_s)
    else:
        return f"Timeout waiting streamed path completion (cmd_id={state.get('cmd_id', '?')})"

    for cmd in plan["ptp_after"]:
        err = _execute_ptp_command(driver, cmd)
        if err:
            return err

    return ""

