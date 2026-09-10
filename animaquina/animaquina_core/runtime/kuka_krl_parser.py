# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Animaquina Core - KUKA KRL parser (Blender-independent)

import re

# Parses an exported .src text into a list of streaming commands.
# Each command is a dict:
#   {"type": "LIN",  "pos": (x_mm, y_mm, z_mm, a_deg, b_deg, c_deg), "approx": True}
#   {"type": "PTP",  "pos": (x_mm, y_mm, z_mm, a_deg, b_deg, c_deg)}
#   {"type": "PTP_JOINT", "joints": (A1, A2, A3, A4, A5, A6)}
#   {"type": "VAR",  "name": "E_SPEED", "value": "123"}
#   {"type": "VEL",  "value": 0.05}

_RE_FRAME = re.compile(
    r"\{\s*X\s+([\-\d.]+)\s*,\s*Y\s+([\-\d.]+)\s*,\s*Z\s+([\-\d.]+)\s*,"
    r"\s*A\s+([\-\d.]+)\s*,\s*B\s+([\-\d.]+)\s*,\s*C\s+([\-\d.]+)\s*\}"
)
_RE_JOINT = re.compile(
    r"\{\s*A1\s+([\-\d.]+)\s*,\s*A2\s+([\-\d.]+)\s*,\s*A3\s+([\-\d.]+)\s*,"
    r"\s*A4\s+([\-\d.]+)\s*,\s*A5\s+([\-\d.]+)\s*,\s*A6\s+([\-\d.]+)"
)
_RE_VEL = re.compile(r"\$VEL\.CP\s*=\s*([\d.]+)")
_RE_TRIGGER_VAR = re.compile(
    r"TRIGGER\s+WHEN\s+DISTANCE\s*=\s*0\s+DELAY\s*=\s*0\s+DO\s+(\w+)\s*=\s*(.+)"
)
_RE_DIRECT_VAR = re.compile(r"^(E_SPEED|E_ENABLE|F_SPEED)\s*=\s*(.+)")


def parse_krl_program(src_text: str) -> list:
    """Parse exported KRL .src text into a list of streaming commands."""
    commands = []
    for raw_line in src_text.splitlines():
        line = raw_line.strip()
        if not line or line.startswith(";") or line.startswith("&"):
            continue

        if line.startswith("LIN "):
            m = _RE_FRAME.search(line)
            if m:
                pos = tuple(float(m.group(i)) for i in range(1, 7))
                approx = "C_DIS" in line
                commands.append({"type": "LIN", "pos": pos, "approx": approx})
            continue

        if line.startswith("PTP "):
            mj = _RE_JOINT.search(line)
            if mj:
                joints = tuple(float(mj.group(i)) for i in range(1, 7))
                commands.append({"type": "PTP_JOINT", "joints": joints})
                continue
            mf = _RE_FRAME.search(line)
            if mf:
                pos = tuple(float(mf.group(i)) for i in range(1, 7))
                commands.append({"type": "PTP", "pos": pos})
            continue

        mv = _RE_VEL.match(line)
        if mv:
            commands.append({"type": "VEL", "value": float(mv.group(1))})
            continue

        mt = _RE_TRIGGER_VAR.match(line)
        if mt:
            commands.append({"type": "VAR", "name": mt.group(1).strip(), "value": mt.group(2).strip()})
            continue

        md = _RE_DIRECT_VAR.match(line)
        if md:
            commands.append({"type": "VAR", "name": md.group(1).strip(), "value": md.group(2).strip()})
            continue

    return commands

