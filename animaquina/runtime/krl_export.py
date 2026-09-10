# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
# Animaquina â€” Blender-side KUKA export wrapper.

import math

import numpy as np

from . import export_points
from . import rig_apply
from animaquina_core.runtime.kuka_krl_parser import parse_krl_program as _parse_krl_program
from animaquina_core.runtime.program_exports import (
    build_krl_program,
    normalize_kuka_program_name,
)

KUKA_HOME_JOINTS_DEFAULT = (5, -90, 100, 5, -10, -5)

# Built-in attribute names handled separately (not passed as custom_vars)
_BUILTIN_ATTRIBUTES = {"E_SPEED", "E_ENABLE", "L_SPEED", "F_SPEED", "position"}


def _coerce_home_joints(home_values) -> tuple:
    raw = tuple(home_values or ())
    return tuple(float(raw[i]) if i < len(raw) else float(KUKA_HOME_JOINTS_DEFAULT[i]) for i in range(6))


def _resolve_kuka_home_joints(slot) -> tuple:
    slot_home = _coerce_home_joints(getattr(slot, "kuka_export_home", KUKA_HOME_JOINTS_DEFAULT))
    if slot is None or not bool(getattr(slot, "is_connected", False)):
        return slot_home
    try:
        from .. import manager

        driver = manager.get_driver_for_slot(slot)
        if driver is None:
            return slot_home
        return _coerce_home_joints(driver.read_home_joints())
    except Exception:
        return slot_home


def parse_krl_program(src_text: str) -> list:
    return _parse_krl_program(src_text)


def export_krl(slot, context) -> str:
    obj = getattr(context, "active_object", None)
    if not obj or obj.type != "MESH":
        return "Select a mesh object with a 'position' attribute"

    depsgraph = context.evaluated_depsgraph_get()
    eval_obj = obj.evaluated_get(depsgraph)
    if not hasattr(eval_obj.data, "attributes") or "position" not in eval_obj.data.attributes:
        return "No 'position' attribute on mesh"

    use_rotation_attr = bool(getattr(slot, "export_use_rotation_attribute", False))
    apply_rot_transform = bool(getattr(slot, "export_rotation_apply_transform", False))

    # export_custom_a/b/c are in KUKA ABC order; convert to Blender XYZ (reverse)
    # so the degree conversion below can reverse back consistently
    constant_euler_rad = (
        math.radians(getattr(slot, "export_custom_c", 0.0)),  # Blender X = KUKA C
        math.radians(getattr(slot, "export_custom_b", 0.0)),  # Blender Y = KUKA B
        math.radians(getattr(slot, "export_custom_a", 0.0)),  # Blender Z = KUKA A
    )

    blender_base_inv = rig_apply.get_slot_blender_base_world_matrix(slot).inverted()

    base_positions, base_eulers = export_points.compute_base_waypoints(
        eval_obj.data,
        obj.matrix_world,
        blender_base_inv,
        use_rotation_attr,
        apply_rot_transform,
        constant_euler_rad,
    )
    if len(base_positions) == 0:
        return "No positions in attribute"

    # Blender (m, XYZ rad) → KUKA (mm, ABC deg); A=Z, B=Y, C=X
    kuka_positions = np.empty((len(base_positions), 6), dtype=np.float64)
    kuka_positions[:, :3] = base_positions * 1000.0
    kuka_positions[:, 3:] = np.degrees(base_eulers[:, ::-1])

    program_name = normalize_kuka_program_name(
        getattr(slot, "program_name", "") or getattr(obj, "name", "") or "animaquina",
        fallback=str(getattr(obj, "name", "") or "animaquina"),
    )
    slot.program_name = program_name
    filename = f"{program_name}.src"

    extrudes = eval_obj.data.attributes.get("E_SPEED")
    enables = eval_obj.data.attributes.get("E_ENABLE")
    lin_speeds = eval_obj.data.attributes.get("L_SPEED")
    fan_speeds = eval_obj.data.attributes.get("F_SPEED")

    # Collect custom variable attributes from slot.debug_vars
    custom_vars = export_points.collect_custom_var_attributes(slot, eval_obj, _BUILTIN_ATTRIBUTES)

    try:
        krl_program = build_krl_program(
            program_name=program_name,
            positions=kuka_positions.tolist(),
            home_joints=_resolve_kuka_home_joints(slot),
            base_num=getattr(slot, "export_base_no", 0),
            tool_num=getattr(slot, "export_tool_no", 0),
            speed=getattr(slot, "export_speed", 15.0),
            acc=getattr(slot, "export_acc", 100),
            lin_speed=getattr(slot, "export_lin_speed", 0.05),
            advance=getattr(slot, "export_advance", 3),
            extrudes=export_points.read_scalar_attribute(extrudes) if extrudes else None,
            enables=export_points.read_scalar_attribute(enables, cast=bool) if enables else None,
            lin_speeds=export_points.read_scalar_attribute(lin_speeds) if lin_speeds else None,
            fan_speeds=export_points.read_scalar_attribute(fan_speeds) if fan_speeds else None,
            custom_vars=custom_vars,
            index_var=(
                getattr(slot, "export_point_index_var", "IDX")
                if getattr(slot, "export_write_point_index", False)
                else None
            ),
        )
    except RuntimeError as exc:
        return str(exc)

    import bpy

    if filename in bpy.data.texts:
        bpy.data.texts[filename].clear()
    else:
        bpy.data.texts.new(filename)
    bpy.data.texts[filename].write(krl_program)

    # Attribute future PROGRAM runs (polled IDX → run_state) to this toolpath
    slot.export_last_object = obj
    slot.export_last_count = len(kuka_positions)

    # Safety net: the property update callback tracks the index variable when
    # the option is toggled in the UI, but a .blend saved with it already on
    # never fires that. Re-assert it here so an exported program that writes the
    # index is always accompanied by a poll entry feeding run_idx.
    from ..properties import sync_point_index_debug_var

    sync_point_index_debug_var(slot, context)
    return ""
