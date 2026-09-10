# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later

"""Pure-Python program generation helpers for closed beta/runtime packaging."""

from __future__ import annotations

import math
import os
import sys
from xml.sax.saxutils import escape

from .conversions import rpy2rv

_core_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_libs = os.path.join(_core_dir, "libs")
if _libs not in sys.path and os.path.isdir(_libs):
    sys.path.insert(0, _libs)

try:
    from kuka_krl_python import KRL
except ImportError:
    KRL = None

try:
    from ur_script_python import URScriptProgram
except ImportError:
    URScriptProgram = None


def normalize_kuka_program_name(raw_name: str, fallback: str = "animaquina") -> str:
    name = str(raw_name or "").strip()
    if name.lower().endswith(".src"):
        name = name[:-4]
    if name.lower().endswith(".dat"):
        name = name[:-4]
    if not name:
        name = str(fallback or "animaquina")
    safe = "".join(ch if (ch.isalnum() or ch == "_") else "_" for ch in name)
    if not safe:
        safe = "animaquina"
    if safe[0].isdigit():
        safe = f"P_{safe}"
    return safe


def normalize_ur_program_name(raw_name: str, fallback: str = "animaquina") -> str:
    name = str(raw_name or "").strip() or str(fallback or "animaquina")
    if name.lower().endswith(".script"):
        name = name[:-7]
    if name.lower().endswith(".urp"):
        name = name[:-4]
    safe = "".join(ch if ch.isalnum() or ch in {"_", "-", "."} else "_" for ch in name)
    return safe or "animaquina"


def make_ur_program_names(raw_name: str, fallback: str = "animaquina") -> dict[str, str]:
    base_name = normalize_ur_program_name(raw_name, fallback=fallback)
    return {
        "base_name": base_name,
        "program_name": base_name.replace(" ", "_").replace(".", "_"),
        "script_filename": f"{base_name}.script",
        "urp_filename": f"{base_name}.urp",
    }


def build_urp_wrapper(
    program_name: str,
    script_filename: str,
    script_content: str,
    *,
    installation_name: str = "default",
    polyscope_version: str = "5.11.11",
) -> str:
    """Build a basic URP wrapper XML that references a staged .script file."""
    prog = str(program_name or "animaquina")
    install = str(installation_name or "default")
    script_file = str(script_filename or "animaquina.script")
    cached = escape(str(script_content or ""))
    version = str(polyscope_version or "5.11.11")
    return (
        f'<URProgram name="{prog}" installation="{install}" '
        f'installationRelativePath="{install}" createdIn="{version}" lastSavedIn="{version}">\n'
        "  <children>\n"
        '    <MainProgram runOnlyOnce="true" InitVariablesNode="false">\n'
        "      <children>\n"
        '        <Script type="File">\n'
        f"          <cachedContents>{cached}</cachedContents>\n"
        f"          <file>{script_file}</file>\n"
        "        </Script>\n"
        "      </children>\n"
        "    </MainProgram>\n"
        "  </children>\n"
        "</URProgram>\n"
    )


def build_krl_program(
    *,
    program_name: str,
    positions: list[tuple[float, float, float, float, float, float]],
    home_joints: tuple[float, float, float, float, float, float],
    base_num: int = 0,
    tool_num: int = 0,
    speed: float = 15.0,
    acc: float = 100.0,
    lin_speed: float = 0.05,
    advance: int = 3,
    extrudes=None,
    enables=None,
    lin_speeds=None,
    fan_speeds=None,
    custom_vars: dict | None = None,
    index_var: str | None = None,
) -> str:
    if KRL is None:
        raise RuntimeError("kuka_krl_python not found (expected animaquina_core/libs on path)")
    if not positions:
        raise ValueError("No positions supplied for KRL export")

    index_var = (index_var or "").strip() or None

    krl = KRL(program_name=program_name)
    krl.change_variable("E_SPEED", 0, wait=False)
    krl.change_variable("E_ENABLE", False, wait=False)
    krl.change_variable("F_SPEED", 0, wait=False)
    if index_var:
        # Initialize + point 0 (the first PTP move below is waypoint index 0).
        krl.change_variable(index_var, 0, wait=False)
    krl.set_start_parameters(
        base_num,
        tool_num,
        speed,
        acc,
        lin_speed,
        advance,
        pos={
            "A1": home_joints[0],
            "A2": home_joints[1],
            "A3": home_joints[2],
            "A4": home_joints[3],
            "A5": home_joints[4],
            "A6": home_joints[5],
            "E1": 0,
            "E2": 0,
            "E3": 0,
            "E4": 0,
        },
    )

    first_position = positions[0]
    krl.add_move_ptp(
        first_position[0],
        first_position[1],
        first_position[2],
        first_position[3],
        first_position[4],
        first_position[5],
    )

    prev_extrude = prev_enable = prev_speed = prev_fan = None
    # Track previous values for custom variables (only emit on change)
    prev_custom = {}
    linear_positions = positions[1:]
    total_linear_moves = len(linear_positions)
    for i, pos in enumerate(linear_positions):
        extrude_val = extrudes[i + 1] if extrudes and i + 1 < len(extrudes) else None
        enable_val = enables[i + 1] if enables and i + 1 < len(enables) else None
        speed_val = lin_speeds[i + 1] if lin_speeds and i + 1 < len(lin_speeds) else None
        fan_val = fan_speeds[i + 1] if fan_speeds and i + 1 < len(fan_speeds) else None

        if speed_val is not None and speed_val != prev_speed:
            krl.set_linear_speed(speed_val)
            prev_speed = speed_val
        if extrude_val is not None and extrude_val != prev_extrude:
            krl.change_variable("E_SPEED", extrude_val)
            prev_extrude = extrude_val
        if enable_val is not None and enable_val != prev_enable:
            krl.change_variable("E_ENABLE", bool(enable_val))
            prev_enable = bool(enable_val)
        if fan_val is not None and fan_val != prev_fan:
            krl.change_variable("F_SPEED", fan_val)
            prev_fan = fan_val

        # Custom variables from tracked debug_vars with matching mesh attributes
        if custom_vars:
            for var_name, values in custom_vars.items():
                val = values[i + 1] if i + 1 < len(values) else None
                if val is not None and val != prev_custom.get(var_name):
                    krl.change_variable(var_name, val, wait=False)
                    prev_custom[var_name] = val

        # Per-point index: waypoint number for this move (point 0 is the PTP above).
        # wait=True emits TRIGGER WHEN DISTANCE=0 DELAY=0, so IDX updates in the
        # main run *at* the point (like E_SPEED/F_SPEED above) instead of early in
        # the advance run — the point-synchronized variable write we use on KUKA.
        if index_var:
            krl.change_variable(index_var, i + 1, wait=True)

        krl.add_linear_move(
            pos[0],
            pos[1],
            pos[2],
            pos[3],
            pos[4],
            pos[5],
            approximate=i < total_linear_moves - 1,
        )

    krl.add_move_ptp(*home_joints, joint=True)
    krl.end_program()
    return krl.program


# ── UR register / IO mapping for export ───────────────────────────
# Maps variable names (as shown in the Variables panel) to the correct
# URScript function call instead of a plain assignment.

_UR_REGISTER_PREFIXES = {
    "output_int_register_": ("write_output_integer_register", int),
    "output_double_register_": ("write_output_float_register", float),
    "input_int_register_": ("write_input_integer_register", int),
    "input_double_register_": ("write_input_float_register", float),
}

_UR_DIGITAL_PREFIXES = {
    "standard_digital_output_": "set_standard_digital_out",
    "tool_digital_output_": "set_tool_digital_out",
    "configurable_digital_output_": "set_configurable_digital_out",
}

_UR_ANALOG_PREFIXES = {
    "standard_analog_output_": "set_standard_analog_out",
    "tool_analog_output_": "set_tool_analog_out",
}


def _ur_write_var(ur, var_name: str, val):
    """Emit the correct URScript for a variable write.

    Register names (output_int_register_12, etc.) map to URScript built-in
    functions. Everything else falls back to a plain variable assignment.
    """
    for prefix, (func, cast) in _UR_REGISTER_PREFIXES.items():
        if var_name.startswith(prefix):
            idx = int(var_name[len(prefix):])
            ur._add_line(f"{func}({idx}, {cast(val)})")
            return

    for prefix, func in _UR_DIGITAL_PREFIXES.items():
        if var_name.startswith(prefix):
            pin = int(var_name[len(prefix):])
            ur._add_line(f"{func}({pin}, {bool(val)})")
            return

    for prefix, func in _UR_ANALOG_PREFIXES.items():
        if var_name.startswith(prefix):
            pin = int(var_name[len(prefix):])
            ur._add_line(f"{func}({pin}, {float(val)})")
            return

    # Fallback: plain URScript variable assignment
    ur.set_variable(var_name, val)


def build_ur_script(
    *,
    program_name: str,
    positions: list[tuple[float, float, float, float, float, float]],
    vel: float = 0.1,
    acc: float = 0.5,
    blend: float = 0.001,
    joint_vel: float = 1.05,
    joint_acc: float = 1.4,
    custom_tool: bool = False,
    payload_mass: float = 0.0,
    payload_cog=None,
    tcp=None,
    home_deg=None,
    extrudes=None,
    enables=None,
    lin_speeds=None,
    fan_speeds=None,
    custom_vars: dict | None = None,
    index_var: str | None = None,
) -> str:
    if URScriptProgram is None:
        raise RuntimeError("ur_script_python not found (expected animaquina_core/libs on path)")
    if not positions:
        raise ValueError("No positions supplied for UR export")

    index_var = (index_var or "").strip() or None

    payload_cog = list(payload_cog or (0, 0, 0))
    tcp = list(tcp or (0, 0, 0, 0, 0, 0))
    home_deg = list(home_deg or (0, -90, 0, 0, 0, 0))
    home_rad = [math.radians(d) for d in home_deg]

    ur = URScriptProgram(program_name=program_name)
    if custom_tool:
        ur.set_tcp(tcp[0], tcp[1], tcp[2], tcp[3], tcp[4], tcp[5])
        ur.set_payload(payload_mass, payload_cog)

    ur.add_popup("Press OK to start program", title=program_name, blocking=True)
    ur.add_movej(home_rad, a=joint_acc, v=joint_vel, r=0.0)
    ur.set_variable("E_SPEED", 0)
    ur.set_variable("E_ENABLE", False)
    ur.set_variable("F_SPEED", 0)

    # Set analog output domains to voltage (1) for any analog pins used
    if custom_vars:
        _analog_pins_init = set()
        for var_name in custom_vars:
            for prefix in _UR_ANALOG_PREFIXES:
                if var_name.startswith(prefix):
                    pin = int(var_name[len(prefix):])
                    if pin not in _analog_pins_init:
                        ur._add_line(f"set_analog_outputdomain({pin}, 1)")
                        _analog_pins_init.add(pin)

    first = positions[0]
    if index_var:
        _ur_write_var(ur, index_var, 0)  # point 0
    ur.add_movel(first[0], first[1], first[2], first[3], first[4], first[5], a=acc, v=vel, r=0.0)

    prev_extrude = prev_enable = prev_speed = prev_fan = None
    prev_custom = {}
    current_vel = vel

    for i, pos in enumerate(positions[1:]):
        extrude_val = extrudes[i + 1] if extrudes and i + 1 < len(extrudes) else None
        enable_val = enables[i + 1] if enables and i + 1 < len(enables) else None
        speed_val = lin_speeds[i + 1] if lin_speeds and i + 1 < len(lin_speeds) else None
        fan_val = fan_speeds[i + 1] if fan_speeds and i + 1 < len(fan_speeds) else None

        if speed_val is not None and speed_val != prev_speed:
            current_vel = speed_val
            prev_speed = speed_val
        if extrude_val is not None and extrude_val != prev_extrude:
            ur.set_variable("E_SPEED", extrude_val)
            prev_extrude = extrude_val
        if enable_val is not None and bool(enable_val) != prev_enable:
            ur.set_variable("E_ENABLE", bool(enable_val))
            prev_enable = bool(enable_val)
        if fan_val is not None and fan_val != prev_fan:
            ur.set_variable("F_SPEED", fan_val)
            prev_fan = fan_val

        # Custom variables from tracked debug_vars with matching mesh attributes
        if custom_vars:
            for var_name, values in custom_vars.items():
                val = values[i + 1] if i + 1 < len(values) else None
                if val is not None and val != prev_custom.get(var_name):
                    _ur_write_var(ur, var_name, val)
                    prev_custom[var_name] = val

        # Per-point index: waypoint number for this move (point 0 is the first movel)
        if index_var:
            _ur_write_var(ur, index_var, i + 1)

        ur.add_movel(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], a=acc, v=current_vel, r=blend)

    ur.add_movej(home_rad, a=joint_acc, v=joint_vel, r=0.0)
    ur.add_popup("Program complete", title=program_name, blocking=False)
    ur.end_program()
    return ur.get_program()


def waypoints_euler_to_ur_positions(
    waypoints: list[tuple[tuple[float, float, float], tuple[float, float, float]]]
) -> list[tuple[float, float, float, float, float, float]]:
    positions = []
    for (x_m, y_m, z_m), (a_rad, b_rad, c_rad) in waypoints:
        rx, ry, rz = rpy2rv(a_rad, b_rad, c_rad)
        positions.append((x_m, y_m, z_m, rx, ry, rz))
    return positions


def waypoints_euler_to_kuka_positions(
    waypoints: list[tuple[tuple[float, float, float], tuple[float, float, float]]]
) -> list[tuple[float, float, float, float, float, float]]:
    """Convert Blender-canonical waypoints (m, XYZ Euler rad) to KUKA (mm, ABC deg).

    Blender XYZ maps to KUKA (C, B, A), same reversal as kuka_driver._format_e6pos.
    """
    positions = []
    for (x_m, y_m, z_m), (x_rad, y_rad, z_rad) in waypoints:
        positions.append(
            (
                x_m * 1000.0,
                y_m * 1000.0,
                z_m * 1000.0,
                math.degrees(z_rad),   # KUKA A = Blender Z
                math.degrees(y_rad),   # KUKA B = Blender Y
                math.degrees(x_rad),   # KUKA C = Blender X
            )
        )
    return positions
