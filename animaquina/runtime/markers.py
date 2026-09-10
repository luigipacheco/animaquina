# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
# Animaquina — add marker at TCP (Section 10)

import bpy


def add_marker_at_tcp(pos_m: tuple, scene=None) -> None:
    """Add a PLAIN_AXES empty at the given world position (m)."""
    if scene is None:
        scene = bpy.context.scene
    bpy.ops.object.empty_add(
        type="PLAIN_AXES",
        radius=0.1,
        location=(float(pos_m[0]), float(pos_m[1]), float(pos_m[2])),
    )
