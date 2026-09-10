# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
# Animaquina â€” Blender-side UR export wrapper.

import math

import numpy as np

from . import export_points
from . import rig_apply
from animaquina_core.runtime.program_exports import (
    build_ur_script,
    build_urp_wrapper as _build_urp_wrapper,
    make_ur_program_names,
)


def build_urp_wrapper(*args, **kwargs) -> str:
    return _build_urp_wrapper(*args, **kwargs)


# Built-in attribute names handled separately (not passed as custom_vars)
_BUILTIN_ATTRIBUTES = {"E_SPEED", "E_ENABLE", "L_SPEED", "F_SPEED", "position"}


def export_ur(slot, context) -> str:
    obj = getattr(context, "active_object", None)
    if not obj or obj.type != "MESH":
        return "Select a mesh object with a 'position' attribute"

    depsgraph = context.evaluated_depsgraph_get()
    eval_obj = obj.evaluated_get(depsgraph)
    if not hasattr(eval_obj.data, "attributes") or "position" not in eval_obj.data.attributes:
        return "No 'position' attribute on mesh"

    use_rotation_attr = bool(getattr(slot, "export_use_rotation_attribute", False))
    apply_rot_transform = bool(getattr(slot, "export_rotation_apply_transform", False))

    constant_euler_rad = (
        math.radians(getattr(slot, "export_custom_a", 0.0)),
        math.radians(getattr(slot, "export_custom_b", 0.0)),
        math.radians(getattr(slot, "export_custom_c", 0.0)),
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

    names = make_ur_program_names(getattr(slot, "ur_program_name", "") or str(obj.name), fallback=str(obj.name))
    slot.ur_program_name = names["base_name"]
    # UR pose = position (m) + axis-angle rotation vector
    positions = np.hstack((base_positions, export_points.eulers_xyz_to_rotvecs(base_eulers))).tolist()

    extrudes = eval_obj.data.attributes.get("E_SPEED")
    enables = eval_obj.data.attributes.get("E_ENABLE")
    lin_speeds = eval_obj.data.attributes.get("L_SPEED")
    fan_speeds = eval_obj.data.attributes.get("F_SPEED")

    # Collect custom variable attributes from slot.debug_vars
    custom_vars = export_points.collect_custom_var_attributes(slot, eval_obj, _BUILTIN_ATTRIBUTES)

    try:
        script_content = build_ur_script(
            program_name=names["program_name"],
            positions=positions,
            vel=getattr(slot, "ur_export_vel", 0.1),
            acc=getattr(slot, "ur_export_acc", 0.5),
            blend=getattr(slot, "ur_export_blend", 0.001),
            joint_vel=getattr(slot, "ur_export_joint_vel", 1.05),
            joint_acc=getattr(slot, "ur_export_joint_acc", 1.4),
            custom_tool=getattr(slot, "ur_export_custom_tool", False),
            payload_mass=getattr(slot, "ur_export_payload_mass", 0.0),
            payload_cog=list(getattr(slot, "ur_export_payload_cog", (0, 0, 0))),
            tcp=list(getattr(slot, "ur_export_tcp", (0, 0, 0, 0, 0, 0))),
            home_deg=list(getattr(slot, "ur_export_home", (0, -90, 0, 0, 0, 0))),
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

    urp_content = build_urp_wrapper(
        program_name=names["program_name"],
        script_filename=names["script_filename"],
        script_content=script_content,
    )

    import bpy

    if names["script_filename"] in bpy.data.texts:
        bpy.data.texts[names["script_filename"]].clear()
    else:
        bpy.data.texts.new(names["script_filename"])
    bpy.data.texts[names["script_filename"]].write(script_content)

    if names["urp_filename"] in bpy.data.texts:
        bpy.data.texts[names["urp_filename"]].clear()
    else:
        bpy.data.texts.new(names["urp_filename"])
    bpy.data.texts[names["urp_filename"]].write(urp_content)

    # Attribute future PROGRAM runs (polled IDX → run_state) to this toolpath
    slot.export_last_object = obj
    slot.export_last_count = len(positions)
    return ""
