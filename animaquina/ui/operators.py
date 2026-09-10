# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
# Animaquina Ã¢â‚¬â€ operators (Section 15, 25: keymappable IDs, meaningful polls)

import json
import math
import os
import importlib
import subprocess
import sys
import threading
import time
import uuid
import bpy
from mathutils import Matrix, Euler
from bpy.props import IntProperty, BoolProperty, StringProperty
from bpy.types import Operator

from .. import manager
from .. import preferences
from .. import run_state
from ..runtime import rig_apply, simulation, markers, newton_validator
from ..runtime import recorder
from animaquina_core.drivers.base import (
    CAP_MANUAL_MODE,
    CAP_MOVE_TO_TARGET,
    CAP_EXECUTE_PATH,
    CAP_HOME,
    CAP_UPLOAD_PROGRAM,
    CAP_SELECT_PROGRAM,
    CAP_RESET,
)
from animaquina_core.runtime.conversions import rad2deg, deg2rad


def get_active_slot(context):
    props = getattr(context.scene, "animaquina", None)
    if props is None:
        return None
    idx = props.active_robot_index
    if idx < 0 or idx >= len(props.robots):
        return None
    return props.robots[idx]



def _get_first_text_block(names):
    """Return first existing Blender text block for candidate names."""
    for name in names or ():
        key = str(name or "").strip()
        if not key:
            continue
        block = bpy.data.texts.get(key)
        if block is not None:
            return block, key
    return None, ""


def _normalize_kuka_program_name(raw_name: str, fallback: str = "animaquina") -> str:
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


def _ptp_motion_params_for_slot(slot):
    """Return (vel, acc) for move_to_pose_ptp in driver-native units."""
    if slot is not None and getattr(slot, "robot_type", "") == "KUKA":
        return float(getattr(slot, "kuka_ptp_speed_pct", 15.0)), float(getattr(slot, "kuka_ptp_acc_pct", 100.0))
    return float(getattr(slot, "joint_vel", 1.05)), float(getattr(slot, "joint_acc", 1.4))


def _capture_start_pose(slot, driver):
    """The pose a toolpath run should return to when it finishes.

    Read it from the robot *now* instead of trusting slot.tcp_pos_m. That
    cache is only refreshed by the poll timer, so with Polling disabled — or
    right after jogging/freedrive — it still holds the pose from connect time,
    and the return move would drive to a stale target instead of where the
    robot actually started. Falls back to the cache when the live read fails.
    """
    read_tcp = getattr(driver, "read_tcp", None)
    if callable(read_tcp):
        try:
            pos_m, euler_rad = read_tcp()
            if pos_m is not None and euler_rad is not None and len(pos_m) >= 3:
                return (tuple(float(v) for v in pos_m[:3]),
                        tuple(float(v) for v in euler_rad[:3]))
        except Exception as exc:
            print("[animaquina] start pose live read failed, using cache:", exc)
    return (tuple(slot.tcp_pos_m), tuple(slot.tcp_euler_rad))


def _slot_target_to_driver_pose(slot, target_obj):
    """Convert target world pose into driver base-frame pose using existing conventions."""
    target_world_loc = tuple(target_obj.matrix_world.to_translation())
    try:
        target_world_euler = tuple(target_obj.matrix_world.to_euler("XYZ"))
    except Exception:
        target_world_euler = tuple(target_obj.rotation_euler)
    world_pose = [(target_world_loc, target_world_euler)]
    if getattr(slot, "robot_type", "") == "KUKA":
        return rig_apply.world_waypoints_to_blender_base_frame(slot, world_pose)[0]
    return rig_apply.world_waypoints_to_base_frame(slot, world_pose)[0]


UR_SENDPATH_WARN_POINTS = 10000
UR_SENDPATH_HARD_LIMIT = 50000


def _get_preferred_newton_python(slot):
    preferred = str(getattr(slot, "newton_python_exe", "") or "").strip()
    if preferred:
        return preferred

    # Blender embedded Python executable path when available.
    py_path = getattr(getattr(bpy, "app", None), "binary_path_python", "") or ""
    if py_path:
        return py_path
    return sys.executable


def _write_newton_debug_text(context, slot, payload: dict) -> None:
    """
    Overwrite a Blender Text datablock with the latest Newton validation/probe debug data.
    Selectable/copyable from the Text Editor.
    """
    try:
        text_name = "ANIMAQUINA_NEWTON_DEBUG"
        text_block = bpy.data.texts.get(text_name)
        if text_block is None:
            text_block = bpy.data.texts.new(text_name)
        else:
            text_block.clear()
        text_block.write(json.dumps(payload, indent=2, ensure_ascii=False, default=str))
    except Exception:
        # Debug output should never break the operator workflow.
        pass


def _build_newton_validation_options(context, slot) -> dict:
    fps_base = context.scene.render.fps_base if context.scene and context.scene.render.fps_base else 1.0
    fps = float(context.scene.render.fps) / float(fps_base)
    return {
        "validation_step_dt": float(getattr(slot, "validation_step_dt", 0.02)),
        "validation_substeps": int(getattr(slot, "validation_substeps", 1)),
        "scene_fps": fps,
        "ik_check_contacts": bool(getattr(slot, "newton_check_contacts", True)),
        "ik_ignore_robot_self_contacts": bool(getattr(slot, "newton_ignore_robot_self_contacts", True)),
    }


def _apply_newton_validation_report(context, slot, report, *, points_count: int, waypoints_count: int, options: dict, op) -> set:
    is_valid = bool(report.get("is_valid"))
    first_failure = report.get("first_failure_index", None)
    failure_kind = report.get("failure_kind") or ""
    backend = report.get("backend") or "worker"
    base_msg = report.get("message") or ("Path valid" if is_valid else "Path invalid")
    stats = report.get("stats") or {}
    elapsed_ms = stats.get("elapsed_ms")
    checked = stats.get("waypoints_checked")

    slot.validation_last_status = "VALID" if is_valid else "INVALID"
    slot.validation_last_failure_index = int(first_failure) if isinstance(first_failure, int) else -1
    slot.validation_last_message = base_msg
    slot.validation_last_backend = str(backend)
    slot.validation_last_backend_message = str(report.get("backend_message") or (report.get("backend_info") or {}).get("message") or "")
    worker_stderr = str(report.get("worker_stderr") or "")
    slot.validation_last_worker_stderr = worker_stderr[:2000]
    contacts = report.get("contacts") or []
    slot.validation_last_contacts = str(contacts)[:2000]
    debug_block = report.get("debug") or {}
    slot.validation_last_debug = str(debug_block)[:2000]

    _write_newton_debug_text(context, slot, {
        "kind": "validate_path",
        "robot_label": getattr(slot, "label", ""),
        "robot_type": getattr(slot, "robot_type", ""),
        "newton_python_exe": getattr(slot, "newton_python_exe", ""),
        "options": options,
        "path_points_count": points_count,
        "waypoints_count": waypoints_count,
        "report": report,
    })

    suffix = []
    if isinstance(first_failure, int):
        suffix.append(f"idx={first_failure}")
    if failure_kind:
        suffix.append(failure_kind)
    if checked is not None:
        suffix.append(f"checked={checked}")
    if elapsed_ms is not None:
        suffix.append(f"{elapsed_ms}ms")
    suffix.append(f"backend={backend}")
    msg = base_msg if not suffix else f"{base_msg} ({', '.join(suffix)})"
    op.report({"INFO"} if is_valid else {"WARNING"}, msg)
    return {"FINISHED"}


def _find_armature_in_collection_recursive(coll):
    if coll is None:
        return None
    for obj in getattr(coll, "objects", []):
        if getattr(obj, "type", None) == "ARMATURE":
            return obj
    for child in getattr(coll, "children", []):
        found = _find_armature_in_collection_recursive(child)
        if found is not None:
            return found
    return None


def _get_newton_bake_target_armature(slot):
    # Prefer the simulation armature for validation playback; fall back to the rig armature.
    arm = _find_armature_in_collection_recursive(getattr(slot, "sim_collection", None))
    if arm is not None:
        return arm
    return getattr(slot, "rig_armature", None)


def _safe_obj_name(obj) -> str:
    try:
        return getattr(obj, "name", "") or ""
    except ReferenceError:
        return ""
    except Exception:
        return ""


def _set_object_world_pose(obj, pos_xyz, euler_xyz) -> None:
    """
    Set an object's world-space pose. Used by Blender IK path playback to snap the
    playback target to path index 0 before binding the Geometry Attribute constraint.
    """
    if obj is None:
        return
    try:
        mw = (
            Matrix.Translation((float(pos_xyz[0]), float(pos_xyz[1]), float(pos_xyz[2])))
            @ Euler((float(euler_xyz[0]), float(euler_xyz[1]), float(euler_xyz[2])), "XYZ").to_matrix().to_4x4()
        )
        obj.matrix_world = mw
    except Exception:
        try:
            obj.location = (float(pos_xyz[0]), float(pos_xyz[1]), float(pos_xyz[2]))
            obj.rotation_euler = (float(euler_xyz[0]), float(euler_xyz[1]), float(euler_xyz[2]))
        except Exception:
            pass


def _joint_axis_rotation_index(axis_str: str) -> int:
    axis = (axis_str or "Y").lstrip("-").upper()
    return {"X": 0, "Y": 1, "Z": 2}.get(axis, 1)


def _get_blender_sim_follow_path_constraint(target_obj):
    if target_obj is None:
        return None
    for c in getattr(target_obj, "constraints", []):
        if c.name == "animaquina_sim_follow_path" and c.type == "FOLLOW_PATH":
            return c
    return None


def _get_blender_sim_geo_attr_constraint(target_obj):
    if target_obj is None:
        return None
    for c in getattr(target_obj, "constraints", []):
        if c.name == "animaquina_sim_geo_attr" and c.type == "GEOMETRY_ATTRIBUTE":
            return c
    return None


def _iter_collection_objects_recursive(coll):
    if coll is None:
        return
    for obj in getattr(coll, "objects", []):
        yield obj
    for child in getattr(coll, "children", []):
        yield from _iter_collection_objects_recursive(child)


def _clear_blender_sim_path_animation_on_object(target_obj) -> dict:
    stats = {"constraint_removed": 0, "fcurves_removed": 0, "geo_constraint_removed": 0}
    if target_obj is None:
        return stats

    # New Blender 5 path playback: Geometry Attribute constraint (sample_index animation)
    geo_con = _get_blender_sim_geo_attr_constraint(target_obj)
    if geo_con is not None:
        try:
            ad = target_obj.animation_data
            action = ad.action if ad else None
            if action is not None:
                for dp in (
                    f'constraints["{geo_con.name}"].sample_index',
                    "rotation_euler",
                    "rotation_quaternion",
                    "rotation_axis_angle",
                    "location",
                ):
                    for fc in list(action.fcurves):
                        if fc.data_path == dp:
                            action.fcurves.remove(fc)
                            stats["fcurves_removed"] += 1
        except Exception:
            pass
        try:
            target_obj.constraints.remove(geo_con)
            stats["geo_constraint_removed"] += 1
        except Exception:
            pass

    # Backward compatibility cleanup: old Follow Path constraint workflow.
    con = _get_blender_sim_follow_path_constraint(target_obj)
    if con is not None:
        try:
            ad = target_obj.animation_data
            action = ad.action if ad else None
            if action is not None:
                for dp in (
                    f'constraints["{con.name}"].offset_factor',
                    "rotation_euler",
                    "rotation_quaternion",
                    "rotation_axis_angle",
                    "location",
                ):
                    for fc in list(action.fcurves):
                        if fc.data_path == dp:
                            action.fcurves.remove(fc)
                            stats["fcurves_removed"] += 1
        except Exception:
            pass
        try:
            target_obj.constraints.remove(con)
            stats["constraint_removed"] += 1
        except Exception:
            pass
    return stats


def _clear_object_transform_keyframes(target_obj) -> int:
    """
    Remove object transform keyframes commonly written by Blender sim validation playback.
    This is used as a final cleanup even if the playback constraint was already removed.
    """
    removed = 0
    if target_obj is None:
        return removed
    try:
        ad = target_obj.animation_data
        action = ad.action if ad else None
        if action is None:
            return 0
        for dp in ("location", "rotation_euler", "rotation_quaternion", "rotation_axis_angle"):
            for fc in list(action.fcurves):
                if fc.data_path == dp:
                    action.fcurves.remove(fc)
                    removed += 1
    except Exception:
        pass
    return removed


def _clear_named_sim_constraint_keyframes(target_obj) -> int:
    """
    Remove orphaned Animaquina path-playback constraint fcurves even if the corresponding
    constraint object was already removed. This ensures Clear Path / Clear Simulation fully
    resets the target playback action.
    """
    removed = 0
    if target_obj is None:
        return removed
    try:
        ad = target_obj.animation_data
        action = ad.action if ad else None
        if action is None:
            return 0
        prefixes = (
            'constraints["animaquina_sim_geo_attr"].',
            'constraints["animaquina_sim_follow_path"].',
        )
        for fc in list(action.fcurves):
            if any(str(fc.data_path).startswith(p) for p in prefixes):
                action.fcurves.remove(fc)
                removed += 1
    except Exception:
        pass
    return removed


def _iter_blender_sim_helper_targets(slot):
    for obj in _iter_collection_objects_recursive(getattr(slot, "sim_collection", None)):
        if str(getattr(obj, "name", "")).startswith("animaquina_sim_target_"):
            yield obj


def _clear_all_animation_on_object(obj) -> int:
    """
    Hard-clear all animation data on an object (used by Clear Path / Clear Simulation for
    playback targets like the user's Target empty). Returns removed fcurve count (best effort).
    """
    if obj is None:
        return 0
    removed = 0
    try:
        ad = obj.animation_data
        action = ad.action if ad else None
        if action is not None:
            removed += len(list(action.fcurves))
    except Exception:
        pass
    try:
        obj.animation_data_clear()
    except Exception:
        pass
    return removed


def _hard_clear_blender_sim_target_keyframes(slot) -> dict:
    """
    Clear all animation from the user's target object and any Animaquina sim helper targets.
    This is intentionally broader than path-only cleanup so Clear Path / Clear Simulation
    reliably remove leftover keyframes on the playback empty.
    """
    stats = {"objects": 0, "fcurves_removed": 0}
    seen = set()

    def _clear(obj):
        nonlocal stats
        if obj is None:
            return
        try:
            oid = obj.as_pointer()
        except Exception:
            oid = id(obj)
        if oid in seen:
            return
        n = _clear_all_animation_on_object(obj)
        if n > 0:
            stats["objects"] += 1
            stats["fcurves_removed"] += int(n)
        seen.add(oid)

    _clear(getattr(slot, "target_object", None))
    for helper in _iter_blender_sim_helper_targets(slot):
        _clear(helper)
    return stats


def _clear_blender_sim_path_animation(slot, scene) -> dict:
    """
    Remove Blender-sim follow-path constraint animation from the target object.
    Sim curve objects live in sim_collection and are removed by simulation.delete_sim_collection().
    """
    stats = {
        "constraint_removed": 0,
        "fcurves_removed": 0,
        "geo_constraint_removed": 0,
        "target_transform_fcurves_removed": 0,
        "orphan_constraint_fcurves_removed": 0,
        "objects_cleared": 0,
    }
    cleared_ids = set()

    def _accumulate(obj):
        nonlocal stats
        if obj is None:
            return
        try:
            obj_id = obj.as_pointer()
        except Exception:
            obj_id = id(obj)
        if obj_id in cleared_ids:
            return
        obj_stats = _clear_blender_sim_path_animation_on_object(obj)
        extra_removed = _clear_object_transform_keyframes(obj)
        orphan_removed = _clear_named_sim_constraint_keyframes(obj)
        stats["target_transform_fcurves_removed"] += int(extra_removed)
        stats["orphan_constraint_fcurves_removed"] += int(orphan_removed)
        if any(obj_stats.values()) or extra_removed or orphan_removed:
            stats["objects_cleared"] += 1
        for k, v in obj_stats.items():
            stats[k] += int(v)
        cleared_ids.add(obj_id)

    _accumulate(getattr(slot, "target_object", None))

    # Also clear helper playback targets created in the sim collection (and any other object
    # holding the Animaquina playback constraints), so Clear Simulation fully resets playback.
    for obj in _iter_collection_objects_recursive(getattr(slot, "sim_collection", None)):
        try:
            has_named_con = any(
                c.name in {"animaquina_sim_geo_attr", "animaquina_sim_follow_path"}
                for c in getattr(obj, "constraints", [])
            )
        except Exception:
            has_named_con = False
        if has_named_con or str(getattr(obj, "name", "")).startswith("animaquina_sim_target_"):
            _accumulate(obj)

    return stats


def _ensure_blender_sim_path_curve(slot, points_world):
    """
    Create/update a poly curve in the sim collection from world-space points.
    Returns (curve_obj, error_message).
    """
    sim_coll, err = simulation.ensure_sim_collection(slot)
    if err:
        return None, err
    if sim_coll is None:
        return None, "Could not create simulation collection"
    if not points_world:
        return None, "No path points"

    curve_obj_name = f"animaquina_sim_path_{(getattr(slot, 'uid', '') or 'slot')[:8]}"
    # Remove previous curve object of the same name in the sim collection.
    for obj in list(sim_coll.objects):
        if obj.name == curve_obj_name and obj.type == "CURVE":
            try:
                bpy.data.objects.remove(obj, do_unlink=True)
            except Exception:
                pass

    curve_data = bpy.data.curves.new(name=f"{curve_obj_name}_data", type="CURVE")
    curve_data.dimensions = "3D"
    try:
        curve_data.use_path = True
    except Exception:
        pass
    spline = curve_data.splines.new(type="POLY")
    spline.points.add(len(points_world) - 1)
    for i, p in enumerate(points_world):
        spline.points[i].co = (float(p[0]), float(p[1]), float(p[2]), 1.0)

    curve_obj = bpy.data.objects.new(curve_obj_name, curve_data)
    # Points are authored in world space, so keep the curve object transform at identity.
    try:
        curve_obj.location = (0.0, 0.0, 0.0)
        curve_obj.rotation_euler = (0.0, 0.0, 0.0)
        curve_obj.scale = (1.0, 1.0, 1.0)
    except Exception:
        pass
    sim_coll.objects.link(curve_obj)
    return curve_obj, ""


def _ensure_blender_sim_geometry_attribute_constraint(target_obj, source_obj):
    """
    Add/update Blender 5 Geometry Attribute constraint to sample the source object's
    `position` point attribute by sample_index.
    """
    con = _get_blender_sim_geo_attr_constraint(target_obj)
    if con is None:
        con = target_obj.constraints.new(type="GEOMETRY_ATTRIBUTE")
        con.name = "animaquina_sim_geo_attr"
    con.target = source_obj
    con.attribute_name = "position"
    con.data_type = "VECTOR"
    con.domain = "POINT"
    # Key setting for using the source object's transform (world-space path playback with local attr values).
    try:
        con.apply_target_transform = True
    except Exception:
        pass
    return con


def _ensure_blender_sim_driver_empty(slot):
    """
    Create/reuse an Empty in the sim collection to drive Blender IK playback when the user's
    target object is also the path mesh source.
    """
    sim_coll, err = simulation.ensure_sim_collection(slot)
    if err:
        return None, err
    if sim_coll is None:
        return None, "Could not create simulation collection"

    name = f"animaquina_sim_target_{(getattr(slot, 'uid', '') or 'slot')[:8]}"
    for obj in sim_coll.objects:
        if obj.name == name:
            return obj, ""
    empty = bpy.data.objects.new(name, None)
    try:
        empty.empty_display_type = "PLAIN_AXES"
        empty.empty_display_size = 0.05
    except Exception:
        pass
    sim_coll.objects.link(empty)
    return empty, ""


def _retarget_sim_tcp_constraints(slot, new_target_obj) -> int:
    """
    Retarget Blender-sim IK targets in sim armature(s) to a new object.
    IK constraint lives on the tcp bone (or joint_6 fallback) and may
    target an orientation helper empty parented to the user's target.
    Returns number of constraints updated.
    """
    if new_target_obj is None or slot.sim_collection is None:
        return 0

    def _walk_armatures(coll):
        for obj in coll.objects:
            if getattr(obj, "type", None) == "ARMATURE":
                yield obj
        for child in coll.children:
            yield from _walk_armatures(child)

    count = 0
    try:
        for arm in _walk_armatures(slot.sim_collection):
            if not getattr(arm, "pose", None):
                continue
            for bone_name in ("tcp", "joint_6"):
                bone = arm.pose.bones.get(bone_name)
                if bone is None:
                    continue
                for c in bone.constraints:
                    if c.type != "IK" or getattr(c, "target", None) is None:
                        continue
                    target = c.target
                    # If IK targets an orientation helper, re-parent it
                    # to the new target instead of replacing ik.target.
                    if target.name.startswith("animaquina_ik_orient_helper"):
                        target.parent = new_target_obj
                        target.matrix_parent_inverse = Matrix.Identity(4)
                    else:
                        c.target = new_target_obj
                    count += 1
    except Exception:
        pass
    return count


def _clear_newton_ik_keyframes_on_armature(scene, armature_obj) -> dict:
    """
    Remove rotation_euler keyframes for joint_1..joint_6 pose bones on the target armature,
    and remove timeline markers created by Newton keyframing.
    Returns small stats dict.
    """
    stats = {"fcurves_removed": 0, "markers_removed": 0}
    if armature_obj is None:
        return stats

    # Remove pose-bone rotation_euler fcurves for joint_1..joint_6.
    try:
        ad = armature_obj.animation_data
        action = ad.action if ad else None
        if action is not None:
            targets = {
                f'pose.bones["joint_{i}"].rotation_euler'
                for i in range(1, 7)
            }
            for fc in list(action.fcurves):
                if fc.data_path in targets:
                    action.fcurves.remove(fc)
                    stats["fcurves_removed"] += 1
    except Exception:
        pass

    # Remove Newton markers from timeline.
    try:
        markers = getattr(scene, "timeline_markers", None)
        if markers is not None:
            for m in list(markers):
                if str(getattr(m, "name", "")).startswith("NEWTON_"):
                    markers.remove(m)
                    stats["markers_removed"] += 1
    except Exception:
        pass
    return stats


def _remove_sim_ik_constraints(armature_obj) -> int:
    """
    Remove the simple Blender sim constraints used by setup_simulation() so Newton keyframing
    can drive joints directly without the IK chain overriding the pose.
    """
    removed = 0
    if armature_obj is None or getattr(armature_obj, "type", None) != "ARMATURE":
        return removed
    try:
        joint_6 = armature_obj.pose.bones.get("joint_6")
        if joint_6:
            for c in list(joint_6.constraints):
                if c.type == "IK":
                    joint_6.constraints.remove(c)
                    removed += 1
        tcp_bone = armature_obj.pose.bones.get("tcp")
        if tcp_bone:
            for c in list(tcp_bone.constraints):
                if c.type == "CHILD_OF":
                    tcp_bone.constraints.remove(c)
                    removed += 1
    except Exception:
        pass
    return removed


class ANIMAQUINA_OT_AddSlot(Operator):
    bl_idname = "object.animaquina_add_slot"
    bl_label = "Add Robot Slot"
    bl_description = "Add a new robot slot"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return context.scene is not None and hasattr(context.scene, "animaquina")

    def execute(self, context):
        props = context.scene.animaquina
        name = (props.new_robot_name or "").strip()
        if not name:
            name = f"Robot {len(props.robots) + 1}"
        props.robots.add()
        idx = len(props.robots) - 1
        props.active_robot_index = idx
        slot = props.robots[idx]
        slot.uid = str(uuid.uuid4())
        slot.label = name
        if context.area:
            context.area.tag_redraw()
        return {"FINISHED"}


class ANIMAQUINA_OT_RemoveSlot(Operator):
    bl_idname = "object.animaquina_remove_slot"
    bl_label = "Remove Robot Slot"
    bl_description = "Remove the active robot slot"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        return slot is not None

    def execute(self, context):
        props = context.scene.animaquina
        if props.active_robot_index < 0 or props.active_robot_index >= len(props.robots):
            return {"CANCELLED"}
        slot = props.robots[props.active_robot_index]
        if slot.is_connected:
            manager.disconnect_slot(slot)
        props.robots.remove(props.active_robot_index)
        props.active_robot_index = max(0, props.active_robot_index - 1)
        return {"FINISHED"}


class ANIMAQUINA_OT_ConnectRobot(Operator):
    bl_idname = "object.animaquina_connect_robot"
    bl_label = "Connect Robot"
    bl_description = "Connect to the robot at the given endpoint"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        return slot is not None and not slot.is_connected

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        err = manager.connect_slot(slot)
        if err:
            self.report({"ERROR"}, err)
            slot.last_error = err
            return {"CANCELLED"}
        if getattr(slot, "robot_type", "") == "UR":
            driver = manager.get_driver_for_slot(slot)
            backend = getattr(driver, "backend_name", "") if driver is not None else ""
            if backend:
                self.report({"INFO"}, f"Connected ({backend})")
            else:
                self.report({"INFO"}, "Connected")
        else:
            self.report({"INFO"}, "Connected")
        return {"FINISHED"}


class ANIMAQUINA_OT_DisconnectRobot(Operator):
    bl_idname = "object.animaquina_disconnect_robot"
    bl_label = "Disconnect Robot"
    bl_description = "Disconnect the active robot"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        return slot is not None and slot.is_connected

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        manager.disconnect_slot(slot)
        self.report({"INFO"}, "Disconnected")
        return {"FINISHED"}


class ANIMAQUINA_OT_URDebugStatus(Operator):
    bl_idname = "object.animaquina_ur_debug_status"
    bl_label = "UR Debug Status"
    bl_description = (
        "Print a UR / RTDE connection diagnostic to the system console — "
        "shows receive/control interface state and whether the control "
        "script is running on the robot"
    )
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        return slot is not None and getattr(slot, "robot_type", "") == "UR"

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        driver = manager.get_driver_for_slot(slot)
        if driver is None:
            self.report({"WARNING"}, "UR not connected — connect first")
            return {"CANCELLED"}
        fn = getattr(driver, "urcap_debug_status", None)
        if not callable(fn):
            self.report({"WARNING"}, "Debug status not supported by this backend")
            return {"CANCELLED"}
        status = fn()
        print("\n" + status + "\n")  # full dump to the system console
        # Surface the verdict line in the Blender status bar.
        verdict = ""
        for line in status.splitlines():
            if "==>" in line:
                verdict = line.strip()
                break
        self.report({"INFO"}, verdict or "UR debug status printed to console")
        return {"FINISHED"}


class ANIMAQUINA_OT_UpdatePose(Operator):
    bl_idname = "object.animaquina_update_pose"
    bl_label = "Update Pose"
    bl_description = "Read from robot and update twin, or apply manual values to twin when not connected"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return get_active_slot(context) is not None

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        driver = manager.get_driver_for_slot(slot)
        if driver is not None and slot.is_connected:
            try:
                pos_m, euler_rad = driver.read_tcp()
                slot.tcp_pos_m = pos_m
                slot.tcp_euler_rad = euler_rad
                slot.tcp_euler_deg = (rad2deg(euler_rad[0]), rad2deg(euler_rad[1]), rad2deg(euler_rad[2]))
                joints = driver.read_joints()
                for j in range(min(6, len(joints))):
                    slot.joints_deg[j] = joints[j]
                if hasattr(driver, "read_base") and (driver.capabilities() & (1 << 3)):
                    base_pos, base_euler = driver.read_base()
                    slot.base_pos_m = base_pos
                    slot.base_euler_rad = base_euler
                    slot.base_euler_deg = (rad2deg(base_euler[0]), rad2deg(base_euler[1]), rad2deg(base_euler[2]))
                    slot.base_frame_valid = True
                else:
                    slot.base_frame_valid = False
                if hasattr(driver, "read_tool"):
                    try:
                        tool_pos, tool_euler = driver.read_tool()
                        slot.tool_frame_pos_m = tool_pos
                        slot.tool_frame_euler_deg = (rad2deg(tool_euler[0]), rad2deg(tool_euler[1]), rad2deg(tool_euler[2]))
                        slot.tool_frame_valid = True
                    except Exception:
                        slot.tool_frame_valid = False
                else:
                    slot.tool_frame_valid = False
                manager.request_aux_refresh(slot)
            except Exception as e:
                slot.last_error = str(e)
                self.report({"ERROR"}, str(e))
                return {"CANCELLED"}
        else:
            # No robot: sync tcp_euler_rad from editable tcp_euler_deg then apply
            slot.tcp_euler_rad = (
                deg2rad(float(slot.tcp_euler_deg[0])),
                deg2rad(float(slot.tcp_euler_deg[1])),
                deg2rad(float(slot.tcp_euler_deg[2])),
            )
        rig_apply.apply_full_pose(slot)
        return {"FINISHED"}


class ANIMAQUINA_OT_ManualMode(Operator):
    bl_idname = "object.animaquina_manual_mode"
    bl_label = "Teach Mode"
    bl_description = "Toggle teach mode (freedrive)"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None or not slot.is_connected:
            return False
        driver = manager.get_driver_for_slot(slot)
        return driver and (driver.capabilities() & CAP_MANUAL_MODE)

    def execute(self, context):
        slot = get_active_slot(context)
        driver = manager.get_driver_for_slot(slot)
        if driver is None:
            return {"CANCELLED"}
        slot.freedrive_active = not slot.freedrive_active
        err = driver.set_manual_mode(slot.freedrive_active)
        if err:
            slot.freedrive_active = not slot.freedrive_active  # revert on error
            self.report({"WARNING"}, err)
        else:
            manager.request_aux_refresh(slot)
        return {"FINISHED"}


class ANIMAQUINA_OT_SnapTargetToTCP(Operator):
    bl_idname = "object.animaquina_snap_target_to_tcp"
    bl_label = "Snap Target to TCP"
    bl_description = "Copy current TCP transform to the Target object"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None:
            return False
        return slot.tcp_object is not None and slot.target_object is not None

    def execute(self, context):
        slot = get_active_slot(context)
        tcp = slot.tcp_object
        target = slot.target_object
        if tcp is None or target is None:
            return {"CANCELLED"}
        target.location = tcp.location.copy()
        target.rotation_euler = tcp.rotation_euler.copy()
        return {"FINISHED"}


class ANIMAQUINA_OT_MoveToTarget(Operator):
    bl_idname = "object.animaquina_move_to_target"
    bl_label = "Move to Target"
    bl_description = "Move robot TCP to the target object pose"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        props = getattr(getattr(context, "scene", None), "animaquina", None)
        selected_objects = set(getattr(context, "selected_objects", []) or [])
        if props is not None and selected_objects:
            for slot in getattr(props, "robots", []):
                if getattr(slot, "motion_active_label", ""):
                    continue
                if getattr(slot, "target_object", None) not in selected_objects:
                    continue
                if not getattr(slot, "is_connected", False):
                    continue
                driver = manager.get_driver_for_slot(slot)
                if driver and (driver.capabilities() & CAP_MOVE_TO_TARGET):
                    return True
        slot = get_active_slot(context)
        if slot is None or not slot.is_connected:
            return False
        if getattr(slot, "motion_active_label", ""):
            return False
        target = slot.target_object
        if not target:
            return False
        driver = manager.get_driver_for_slot(slot)
        return driver and (driver.capabilities() & CAP_MOVE_TO_TARGET)

    def execute(self, context):
        def _move_slot_to_target(slot, driver, target):
            target_world_loc = tuple(target.matrix_world.to_translation())
            try:
                target_world_euler = tuple(target.matrix_world.to_euler("XYZ"))
            except Exception:
                target_world_euler = tuple(target.rotation_euler)
            world_pose = [(target_world_loc, target_world_euler)]
            if slot.robot_type == "KUKA":
                pos_m, euler_rad = rig_apply.world_waypoints_to_blender_base_frame(slot, world_pose)[0]
            else:
                pos_m, euler_rad = rig_apply.world_waypoints_to_base_frame(slot, world_pose)[0]

            tcp_cache_pos = tuple(getattr(slot, "tcp_pos_m", (0.0, 0.0, 0.0)))
            tcp_cache_euler = tuple(getattr(slot, "tcp_euler_rad", (0.0, 0.0, 0.0)))
            tcp_obj = getattr(slot, "tcp_object", None)
            if tcp_obj is not None:
                tcp_world_loc = tuple(tcp_obj.matrix_world.to_translation())
                try:
                    tcp_world_euler = tuple(tcp_obj.matrix_world.to_euler("XYZ"))
                except Exception:
                    tcp_world_euler = tuple(tcp_obj.rotation_euler)
            else:
                tcp_world_loc = None
                tcp_world_euler = None

            debug_moves = bool(
                getattr(getattr(getattr(bpy.context, "scene", None), "animaquina", None), "global_debug", False)
                or getattr(slot, "ur_debug", False)
            )
            if debug_moves:
                print(
                    f"[Animaquina] MoveToTarget[{getattr(slot, 'label', 'Robot')}] "
                    f"type={getattr(slot, 'robot_type', '')} "
                    f"target_world_pos={target_world_loc} target_world_euler={target_world_euler}"
                )
                print(
                    f"[Animaquina] MoveToTarget[{getattr(slot, 'label', 'Robot')}] "
                    f"tcp_cache_pos={tcp_cache_pos} tcp_cache_euler={tcp_cache_euler} "
                    f"tcp_world_pos={tcp_world_loc} tcp_world_euler={tcp_world_euler}"
                )
                print(
                    f"[Animaquina] MoveToTarget[{getattr(slot, 'label', 'Robot')}] "
                    f"send_base_pos={pos_m} send_base_euler={euler_rad}"
                )

            if slot.robot_type == "KUKA" and hasattr(driver, "read_var") and debug_moves:
                e6pos_str = driver._format_e6pos(pos_m, euler_rad)
                print(f"[Animaquina] MoveToTarget: E6POS={e6pos_str}  speed={slot.speed}  radius={slot.radius}  wait=False")
                try:
                    state = driver.read_var("MQ_STATE")
                    action = driver.read_var("MQ_ACTION")
                    print(f"[Animaquina] Before send: MQ_STATE={state}  MQ_ACTION={action}")
                except Exception as ex:
                    print(f"[Animaquina] Could not read MQ vars: {ex}")

            if getattr(slot, "move_mode", "LINEAR") == "PTP":
                ptp_vel, ptp_acc = _ptp_motion_params_for_slot(slot)
                err = driver.move_to_pose_ptp(pos_m, euler_rad, ptp_vel, ptp_acc, slot.radius, False)
            else:
                err = driver.move_to_pose(pos_m, euler_rad, slot.speed, slot.acc, slot.radius, False)

            if slot.robot_type == "KUKA" and hasattr(driver, "read_var") and debug_moves:
                try:
                    state = driver.read_var("MQ_STATE")
                    action = driver.read_var("MQ_ACTION")
                    done = driver.read_var("MQ_DONE_ID")
                    print(f"[Animaquina] After send:  MQ_STATE={state}  MQ_ACTION={action}  MQ_DONE_ID={done}")
                except Exception as ex:
                    print(f"[Animaquina] Could not read MQ vars after: {ex}")
            return err

        props = getattr(getattr(context, "scene", None), "animaquina", None)
        selected_objects = set(getattr(context, "selected_objects", []) or [])
        selected_target_slots = []
        if props is not None and selected_objects:
            for slot in getattr(props, "robots", []):
                if getattr(slot, "target_object", None) in selected_objects:
                    selected_target_slots.append(slot)

        slots_to_process = selected_target_slots if selected_target_slots else [get_active_slot(context)]
        slots_to_process = [slot for slot in slots_to_process if slot is not None]
        if not slots_to_process:
            return {"CANCELLED"}

        moved_labels = []
        failures = []
        for slot in slots_to_process:
            target = slot.target_object
            if target is None:
                failures.append(f"{getattr(slot, 'label', 'Robot')}: no Target object")
                continue
            if not getattr(slot, "is_connected", False):
                failures.append(f"{getattr(slot, 'label', 'Robot')}: not connected")
                continue
            driver = manager.get_driver_for_slot(slot)
            if driver is None or not (driver.capabilities() & CAP_MOVE_TO_TARGET):
                failures.append(f"{getattr(slot, 'label', 'Robot')}: move not supported")
                continue

            err = _move_slot_to_target(slot, driver, target)
            if err:
                slot.last_error = err
                failures.append(f"{getattr(slot, 'label', 'Robot')}: {err}")
                continue
            manager.request_aux_refresh(slot)
            moved_labels.append(getattr(slot, "label", "Robot"))

        if failures and not moved_labels:
            self.report({"ERROR"}, failures[0][:220])
            return {"CANCELLED"}
        if failures:
            self.report({"WARNING"}, f"Move sent to {len(moved_labels)} robot(s); {len(failures)} failed")
            for msg in failures:
                print(f"[Animaquina] MoveToTarget skipped/failed: {msg}")
            return {"FINISHED"}
        if len(moved_labels) > 1:
            self.report({"INFO"}, f"Move sent to {len(moved_labels)} robots")
        return {"FINISHED"}


class ANIMAQUINA_OT_RealTimePuppetStart(Operator):
    bl_idname = "object.animaquina_realtime_puppet_start"
    bl_label = "Start Puppet Mode"
    bl_description = "Continuously stream target pose to robot in real time"
    bl_options = {"REGISTER"}

    _aq_timer = None
    _aq_slot_uid = ""
    _aq_last_target_mm = None
    # Spring state: position, velocity, and previous target per DOF (6 axes)
    _aq_spring_x = None
    _aq_spring_v = None
    _aq_spring_y_prev = None
    # perf_counter timestamp of the last spring integration (measured dt)
    _aq_last_tick_t = None

    @staticmethod
    def _spring_step(x, xd, y, yd, freq, zeta, r, dt):
        """Full second-order spring-damper (t3ssel8r formulation).

        x/xd  — spring position and velocity
        y/yd  — target position and velocity
        freq  — natural frequency (Hz)
        zeta  — damping ratio (1.0 = critical, <1 = overshoot, >1 = overdamped)
        r     — response: scales target velocity in the force term.
                0 = ignores target velocity; 1 = matches it; <0 = anticipation snap
        Returns (new_x, new_xd).
        """
        import math
        omega = 2.0 * math.pi * freq
        k1 = zeta / (math.pi * freq)
        k2 = 1.0 / (omega * omega)
        k3 = r * zeta / omega
        # Clamp k2 for numerical stability at large dt
        k2s = max(k2, max(dt * dt / 2.0 + dt * k1 / 2.0, dt * k1))
        x_new = x + dt * xd
        xd_new = xd + dt * (y + k3 * yd - x - k1 * xd) / k2s
        return x_new, xd_new


    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None or not slot.is_connected or slot.target_object is None:
            return False
        if bool(getattr(slot, "realtime_puppet_active", False)):
            return False
        if getattr(slot, "motion_active_label", ""):
            cls.poll_message_set("Wait for the toolpath to finish before starting Puppet Mode")
            return False
        driver = manager.get_driver_for_slot(slot)
        if driver and getattr(slot, "robot_type", "") == "UR":
            if str(getattr(driver, "backend_name", "") or "") != "ur_rtde":
                return False
        return bool(
            driver
            and hasattr(driver, "realtime_puppet_start")
            and hasattr(driver, "realtime_puppet_step")
            and hasattr(driver, "realtime_puppet_stop")
        )

    def _remove_timer(self, context):
        if getattr(self, "_aq_timer", None) is None:
            return
        try:
            context.window_manager.event_timer_remove(self._aq_timer)
        except Exception:
            pass
        self._aq_timer = None

    def _resolve_slot(self, context):
        props = getattr(getattr(context, "scene", None), "animaquina", None)
        uid = str(getattr(self, "_aq_slot_uid", "") or "")
        if props is None or not uid:
            return None
        for slot in getattr(props, "robots", []):
            if str(getattr(slot, "uid", "") or "") == uid:
                return slot
        return None

    def _stop_driver(self, slot, driver):
        if slot is None:
            return
        slot.realtime_puppet_active = False
        if driver is not None and hasattr(driver, "realtime_puppet_stop"):
            err = driver.realtime_puppet_stop() or ""
            if err:
                slot.realtime_puppet_status = err
                slot.last_error = err
                return err
        manager.request_aux_refresh(slot)
        return ""

    def execute(self, context):
        slot = get_active_slot(context)
        driver = manager.get_driver_for_slot(slot) if slot is not None else None
        if slot is None or driver is None:
            return {"CANCELLED"}
        if getattr(context, "window", None) is None:
            self.report({"ERROR"}, "Cannot start Puppet Mode without an active window")
            return {"CANCELLED"}

        # Safety: snap target to current TCP before starting puppet mode
        # to prevent the robot from jumping to a distant target position.
        tcp = getattr(slot, "tcp_object", None)
        target = getattr(slot, "target_object", None)
        if tcp is not None and target is not None:
            target.location = tcp.location.copy()
            target.rotation_euler = tcp.rotation_euler.copy()

        boundary = None
        if getattr(slot, "robot_type", "") == "XARM" and bool(getattr(slot, "xarm_puppet_use_boundary", False)):
            boundary = list(getattr(slot, "xarm_puppet_boundary_mm", (600.0, 205.0, 300.0, -300.0, 600.0, 100.0)))
        # KUKA: write puppet speed (separate property from regular PTP speed)
        if getattr(slot, "robot_type", "") == "KUKA" and hasattr(driver, "write_var"):
            ptp_pct = max(0.1, min(100.0, float(getattr(slot, "kuka_puppet_speed_pct", 50.0))))
            driver.write_var("MQ_PUPPET_SPEED", f"{ptp_pct:.1f}")
        err = driver.realtime_puppet_start(boundary_mm=boundary)
        if err:
            self.report({"ERROR"}, err[:240])
            slot.last_error = err
            slot.realtime_puppet_status = f"Error: {err[:180]}"
            slot.realtime_puppet_active = False
            return {"CANCELLED"}

        self._aq_slot_uid = str(getattr(slot, "uid", "") or "")
        self._aq_last_target_mm = None
        self._aq_spring_x = None
        self._aq_spring_v = None
        self._aq_spring_y_prev = None
        self._aq_last_tick_t = None
        slot.realtime_puppet_active = True
        slot.realtime_puppet_status = "Puppet Mode: running"

        try:
            pos_m, _ = _slot_target_to_driver_pose(slot, slot.target_object)
            self._aq_last_target_mm = [
                float(pos_m[0]) * 1000.0,
                float(pos_m[1]) * 1000.0,
                float(pos_m[2]) * 1000.0,
            ]
        except Exception:
            self._aq_last_target_mm = None

        rate_hz = max(1.0, float(getattr(slot, "realtime_puppet_rate_hz", 50.0)))
        wm = context.window_manager
        self._aq_timer = wm.event_timer_add(1.0 / rate_hz, window=context.window)
        wm.modal_handler_add(self)
        self.report({"INFO"}, "Puppet Mode started")
        return {"RUNNING_MODAL"}

    def modal(self, context, event):
        if event.type == "ESC":
            slot = self._resolve_slot(context)
            driver = manager.get_driver_for_slot(slot) if slot is not None else None
            self._remove_timer(context)
            err = self._stop_driver(slot, driver)
            if err:
                self.report({"WARNING"}, err[:240])
            else:
                if slot is not None:
                    slot.realtime_puppet_status = "Puppet Mode: stopped"
                self.report({"INFO"}, "Puppet Mode stopped")
            return {"CANCELLED"}

        if event.type != "TIMER":
            return {"PASS_THROUGH"}

        slot = self._resolve_slot(context)
        if slot is None:
            self._remove_timer(context)
            return {"CANCELLED"}

        # External stop request (button/operator) ends modal loop cleanly.
        if not bool(getattr(slot, "realtime_puppet_active", False)):
            self._remove_timer(context)
            slot.realtime_puppet_status = "Puppet Mode: stopped"
            return {"FINISHED"}

        if not bool(getattr(slot, "is_connected", False)):
            self._remove_timer(context)
            slot.realtime_puppet_active = False
            slot.realtime_puppet_status = "Puppet Mode: disconnected"
            return {"CANCELLED"}

        driver = manager.get_driver_for_slot(slot)
        if driver is None:
            self._remove_timer(context)
            slot.realtime_puppet_active = False
            slot.realtime_puppet_status = "Puppet Mode: driver unavailable"
            return {"CANCELLED"}

        target = getattr(slot, "target_object", None)
        if target is None:
            slot.realtime_puppet_status = "Puppet Mode: no target object"
            return {"PASS_THROUGH"}

        try:
            pos_m, euler_rad = _slot_target_to_driver_pose(slot, target)
        except Exception as ex:
            slot.realtime_puppet_status = f"Puppet Mode: target conversion error ({str(ex)[:120]})"
            return {"PASS_THROUGH"}

        vals = [float(pos_m[0]), float(pos_m[1]), float(pos_m[2]), float(euler_rad[0]), float(euler_rad[1]), float(euler_rad[2])]
        if not all(math.isfinite(v) for v in vals):
            slot.realtime_puppet_status = "Puppet Mode: invalid target values"
            return {"PASS_THROUGH"}

        cur_mm = [vals[0] * 1000.0, vals[1] * 1000.0, vals[2] * 1000.0]
        max_step = max(0.0, float(getattr(slot, "realtime_puppet_max_step_mm", 50.0)))
        if self._aq_last_target_mm is not None and max_step > 0.0:
            dx = cur_mm[0] - self._aq_last_target_mm[0]
            dy = cur_mm[1] - self._aq_last_target_mm[1]
            dz = cur_mm[2] - self._aq_last_target_mm[2]
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist > max_step:
                slot.realtime_puppet_status = (
                    f"Puppet Mode: jump too large ({dist:.1f}mm > {max_step:.1f}mm), update skipped"
                )
                return {"PASS_THROUGH"}

        # Spring-damper filter: smooth out stop-start jerkiness by having the
        # robot chase a physically simulated position rather than the raw target.
        rate_hz = max(1.0, float(getattr(slot, "realtime_puppet_rate_hz", 50.0)))
        # Measured dt, not the nominal timer interval. The timer does not fire on
        # schedule: each puppet step blocks this thread on a driver round-trip, so
        # on a slow transport (KUKA C3) ticks routinely run late. Integrating with
        # the nominal dt under-advances the spring exactly when it is behind, which
        # reads as the robot lagging and moving in steps. Clamped so a stalled or
        # double-fired tick cannot hand the integrator a wild dt (the lower bound
        # also protects the target-velocity division below).
        now = time.perf_counter()
        prev_tick_t = getattr(self, "_aq_last_tick_t", None)
        if prev_tick_t is None:
            dt = 1.0 / rate_hz
        else:
            dt = min(0.2, max(0.002, now - prev_tick_t))
        self._aq_last_tick_t = now
        if bool(getattr(slot, "puppet_spring_enabled", True)):
            freq = max(0.1, float(getattr(slot, "puppet_spring_freq", 4.0)))
            zeta = max(0.01, float(getattr(slot, "puppet_spring_damping", 1.0)))
            r = float(getattr(slot, "puppet_spring_response", 0.0))
            # Seed spring state from actual target on first tick
            if self._aq_spring_x is None:
                self._aq_spring_x = list(vals)
                self._aq_spring_v = [0.0] * 6
                self._aq_spring_y_prev = list(vals)
            spring_out = list(vals)
            for i in range(6):
                yd = (vals[i] - self._aq_spring_y_prev[i]) / dt
                nx, nv = self._spring_step(
                    self._aq_spring_x[i], self._aq_spring_v[i],
                    vals[i], yd, freq, zeta, r, dt,
                )
                self._aq_spring_x[i] = nx
                self._aq_spring_v[i] = nv
                spring_out[i] = nx
            self._aq_spring_y_prev = list(vals)
            vals = spring_out

        err = driver.realtime_puppet_step(
            (vals[0], vals[1], vals[2]),
            (vals[3], vals[4], vals[5]),
            float(getattr(slot, "speed", 0.1)),
            float(getattr(slot, "acc", 0.5)),
        )
        if err:
            slot.last_error = err
            slot.realtime_puppet_status = f"Puppet Mode error: {err[:140]}"
            return {"PASS_THROUGH"}

        self._aq_last_target_mm = cur_mm
        slot.realtime_puppet_status = (
            f"Puppet Mode: X:{cur_mm[0]:.1f} Y:{cur_mm[1]:.1f} Z:{cur_mm[2]:.1f} mm"
        )
        return {"PASS_THROUGH"}

    def cancel(self, context):
        slot = self._resolve_slot(context)
        driver = manager.get_driver_for_slot(slot) if slot is not None else None
        self._remove_timer(context)
        self._stop_driver(slot, driver)


class ANIMAQUINA_OT_RealTimePuppetStop(Operator):
    bl_idname = "object.animaquina_realtime_puppet_stop"
    bl_label = "Stop Puppet Mode"
    bl_description = "Stop real-time target streaming and return robot to normal control mode"
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None or not slot.is_connected:
            return False
        driver = manager.get_driver_for_slot(slot)
        if driver and getattr(slot, "robot_type", "") == "UR":
            if str(getattr(driver, "backend_name", "") or "") != "ur_rtde":
                return False
        return bool(driver and hasattr(driver, "realtime_puppet_stop"))

    def execute(self, context):
        slot = get_active_slot(context)
        driver = manager.get_driver_for_slot(slot) if slot is not None else None
        if slot is None or driver is None:
            return {"CANCELLED"}

        slot.realtime_puppet_active = False
        err = driver.realtime_puppet_stop() or ""
        if err:
            slot.last_error = err
            slot.realtime_puppet_status = f"Puppet Mode warning: {err[:140]}"
            self.report({"WARNING"}, err[:240])
            return {"FINISHED"}

        slot.realtime_puppet_status = "Puppet Mode: stopped"
        manager.request_aux_refresh(slot)
        self.report({"INFO"}, "Puppet Mode stopped")
        return {"FINISHED"}


class ANIMAQUINA_OT_GoHome(Operator):
    bl_idname = "object.animaquina_go_home"
    bl_label = "Go Home"
    bl_description = "Move robot to home (joint zero) if supported"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None or not slot.is_connected:
            return False
        if getattr(slot, "motion_active_label", ""):
            return False
        driver = manager.get_driver_for_slot(slot)
        return driver and (driver.capabilities() & CAP_HOME)

    def execute(self, context):
        slot = get_active_slot(context)
        driver = manager.get_driver_for_slot(slot)
        if driver is None:
            return {"CANCELLED"}
        # Use the slot's configured home joints and control joint vel/acc
        if slot.robot_type == "UR":
            home = list(slot.ur_export_home)
        elif slot.robot_type == "KUKA":
            home = list(slot.kuka_export_home)
        elif slot.robot_type == "XARM":
            home = list(slot.xarm_export_home)
        else:
            home = [0.0] * 6
        err = driver.go_home(joints_deg=home, vel=slot.home_vel, acc=slot.home_acc)
        if err:
            self.report({"ERROR"}, err)
            slot.last_error = err
            return {"CANCELLED"}
        manager.request_aux_refresh(slot)
        rig_apply.apply_full_pose(slot)
        self.report({"INFO"}, "Go home done")
        return {"FINISHED"}


class ANIMAQUINA_OT_Reset(Operator):
    bl_idname = "object.animaquina_reset"
    bl_label = "Reset"
    bl_description = "Clear errors and return to safe state (SDK home)"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None or not slot.is_connected:
            return False
        driver = manager.get_driver_for_slot(slot)
        return driver and (driver.capabilities() & CAP_RESET)

    def execute(self, context):
        slot = get_active_slot(context)
        driver = manager.get_driver_for_slot(slot)
        if driver is None:
            return {"CANCELLED"}
        err = driver.reset()
        if err:
            self.report({"ERROR"}, err)
            slot.last_error = err
            return {"CANCELLED"}
        manager.request_aux_refresh(slot)
        rig_apply.apply_full_pose(slot)
        self.report({"INFO"}, "Reset done")
        return {"FINISHED"}


class ANIMAQUINA_OT_AddMarker(Operator):
    bl_idname = "object.animaquina_add_marker"
    bl_label = "Add Marker"
    bl_description = "Add an empty at the current TCP position"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        return slot is not None

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        if getattr(slot, "tcp_object", None) is not None:
            pos_m = tuple(slot.tcp_object.matrix_world.to_translation())
        else:
            tcp_base_obj = getattr(slot, "base_object", None)
            if tcp_base_obj is not None and getattr(tcp_base_obj, "parent", None) is not None:
                tcp_base_obj = tcp_base_obj.parent
            world_pose = rig_apply._tcp_base_to_world_matrix(
                tuple(slot.tcp_pos_m),
                tuple(slot.tcp_euler_rad),
                tcp_base_obj,
            )
            pos_m = tuple(world_pose[0])
        markers.add_marker_at_tcp(pos_m, context.scene)
        return {"FINISHED"}


class ANIMAQUINA_OT_SetTool(Operator):
    bl_idname = "object.animaquina_set_tool"
    bl_label = "Set Tool"
    bl_description = "Constrain tool object to TCP"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        return slot is not None and slot.tcp_object and slot.tool_object

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        rig_apply.ensure_tool_constraint(slot.tool_object, slot.tcp_object)
        self.report({"INFO"}, "Tool constrained to TCP")
        return {"FINISHED"}


# Default joint axis map for Universal Robots (old animaquinaur: Y,Z,Z,Y,Y,Y)
UR_DEFAULT_AXES = ("Y", "Z", "Z", "Y", "Y", "Y")
# Default joint axis map for common KUKA rigs (kr10, kr30, kr120 from old addon)
KUKA_DEFAULT_AXES = ("-Y", "-X", "-X", "-Y", "-X", "-Y")
# xArm 6 from old animaquinauf (ufxarm6_twin); uf850 uses Y,Z,-Z,Y,Z,Y
XARM_DEFAULT_AXES = ("Y", "-Z", "-Z", "Y", "-Z", "Y")
XARM_850_AXES = ("Y", "Z", "-Z", "Y", "Z", "Y")

UR_AXIS_MAPS = {
    "UR_GENERIC": UR_DEFAULT_AXES,
    "UR3": UR_DEFAULT_AXES,
    "UR5": UR_DEFAULT_AXES,
    "UR10": UR_DEFAULT_AXES,
    "UR16": UR_DEFAULT_AXES,
    "UR20": UR_DEFAULT_AXES,
    "UR30": UR_DEFAULT_AXES,
}

KUKA_AXIS_MAPS = {
    "KUKA_GENERIC": KUKA_DEFAULT_AXES,
    "KR10": KUKA_DEFAULT_AXES,
    "KR30": KUKA_DEFAULT_AXES,
    "KR120": KUKA_DEFAULT_AXES,
}

XARM_AXIS_MAPS = {
    "XARM6": XARM_DEFAULT_AXES,
    "UF850": XARM_850_AXES,
}


class ANIMAQUINA_OT_SetURAxes(Operator):
    bl_idname = "object.animaquina_set_ur_axes"
    bl_label = "Set UR Axes"
    bl_description = "Set joint axis map to UR default (Y,Z,Z,Y,Y,Y)"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return get_active_slot(context) is not None

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        model = getattr(slot, "ur_model", "UR_GENERIC")
        axes = UR_AXIS_MAPS.get(model, UR_DEFAULT_AXES)
        for i, axis in enumerate(axes):
            setattr(slot, f"joint_axis_{i}", axis)
        self.report({"INFO"}, f"Joint axes set for {model}")
        return {"FINISHED"}


class ANIMAQUINA_OT_SetKukaAxes(Operator):
    bl_idname = "object.animaquina_set_kuka_axes"
    bl_label = "Set KUKA Axes"
    bl_description = "Set joint axis map to KUKA default (-Y,-X,-X,-Y,-X,-Y) for testing"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return get_active_slot(context) is not None

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        model = getattr(slot, "kuka_model", "KUKA_GENERIC")
        axes = KUKA_AXIS_MAPS.get(model, KUKA_DEFAULT_AXES)
        for i, axis in enumerate(axes):
            setattr(slot, f"joint_axis_{i}", axis)
        self.report({"INFO"}, f"Joint axes set for {model}")
        return {"FINISHED"}


class ANIMAQUINA_OT_SetXArmAxes(Operator):
    bl_idname = "object.animaquina_set_xarm_axes"
    bl_label = "Set xArm Axes"
    bl_description = "Set joint axis map to xArm 6 default (Y,-Z,-Z,Y,-Z,Y)"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return get_active_slot(context) is not None

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        model = getattr(slot, "xarm_model", "XARM6")
        axes = XARM_AXIS_MAPS.get(model, XARM_DEFAULT_AXES)
        for i, axis in enumerate(axes):
            setattr(slot, f"joint_axis_{i}", axis)
        self.report({"INFO"}, f"Joint axes set for {model}")
        return {"FINISHED"}


class ANIMAQUINA_OT_SetSimulation(Operator):
    bl_idname = "object.animaquina_set_simulation"
    bl_label = "Set Simulation"
    bl_description = "Setup simulation collection (rest pose, IK)"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        return slot is not None and slot.rig_collection

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        use_blender_ik = getattr(slot, "simulation_mode", "BLENDER_IK") == "BLENDER_IK"
        err = simulation.setup_simulation(slot, enable_blender_ik=use_blender_ik)
        if err:
            self.report({"ERROR"}, err)
            return {"CANCELLED"}
        mode_label = "Blender IK" if use_blender_ik else "Newton (no Blender IK)"
        self.report({"INFO"}, f"Simulation setup done ({mode_label})")
        return {"FINISHED"}


class ANIMAQUINA_OT_BlenderSimValidatePath(Operator):
    bl_idname = "object.animaquina_blender_validate_path"
    bl_label = "Validate Path"
    bl_description = "Animate the Target along a generated curve path to drive Blender IK simulation"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None:
            return False
        obj = getattr(context, "active_object", None)
        if not obj or obj.type != "MESH":
            return False
        if not getattr(obj.data, "attributes", None) or "position" not in obj.data.attributes:
            return False
        return slot.target_object is not None

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        if slot.target_object is None:
            self.report({"ERROR"}, "Set a Target object for Blender simulation")
            return {"CANCELLED"}

        # Capture the selected path mesh + points first because setup_simulation()
        # changes the active object to the sim armature.
        path_obj = context.active_object
        if path_obj is None or path_obj.type != "MESH":
            self.report({"ERROR"}, "Select a mesh with a 'position' attribute")
            return {"CANCELLED"}
        points, err_msg = _get_path_points_from_object(context)
        if points is None:
            self.report({"ERROR"}, err_msg)
            return {"CANCELLED"}
        # Use the tcp_object's actual world-space rotation (same source Newton
        # uses via target_obj.matrix_world).  slot.tcp_euler_rad is in robot base
        # frame and would be wrong if the armature has any world rotation.
        tcp_obj = slot.tcp_object
        if tcp_obj is not None:
            euler_rad = tuple(tcp_obj.matrix_world.to_euler("XYZ"))
        else:
            euler_rad = tuple(slot.tcp_euler_rad)

        # Ensure sim rig exists/configured (Blender IK path mode).
        err = simulation.setup_simulation(slot, enable_blender_ik=True)
        if err:
            self.report({"ERROR"}, err)
            return {"CANCELLED"}

        target_obj = slot.target_object
        playback_target_obj = target_obj

        # If the user's "target" is actually the path mesh, use a sim helper Empty as the
        # animated IK target and keep the path mesh as the geometry source.
        if target_obj == path_obj:
            helper_obj, helper_err = _ensure_blender_sim_driver_empty(slot)
            if helper_err or helper_obj is None:
                self.report({"ERROR"}, helper_err or "Could not create sim target helper")
                return {"CANCELLED"}
            playback_target_obj = helper_obj
            _retarget_sim_tcp_constraints(slot, helper_obj)

        clear_target_stats = _clear_blender_sim_path_animation(slot, context.scene)
        # If we switched to a helper target, clear any old helper/path constraints too.
        if playback_target_obj is not target_obj:
            helper_clear_stats = _clear_blender_sim_path_animation(type("Tmp", (), {"target_object": playback_target_obj})(), context.scene)
            clear_target_stats["helper_constraint_removed"] = helper_clear_stats.get("constraint_removed", 0)
            clear_target_stats["helper_geo_constraint_removed"] = helper_clear_stats.get("geo_constraint_removed", 0)
            clear_target_stats["helper_fcurves_removed"] = helper_clear_stats.get("fcurves_removed", 0)
        # Preserve TCP orientation during Blender-sim path playback (same orientation basis
        # used in Send Path / Newton validation flow).
        target_rot = (
            float(euler_rad[0]),
            float(euler_rad[1]),
            float(euler_rad[2]),
        )
        # Blender IK path workflow:
        # 1) clear old path playback
        # 2) move playback target to path point 0
        # 3) bind/apply the Geometry Attribute constraint
        # This avoids accumulating a bind-time offset between path iterations.
        p0 = None
        if points:
            try:
                p0 = points[0]
                _set_object_world_pose(playback_target_obj, p0, target_rot)
                try:
                    context.view_layer.update()
                except Exception:
                    pass
            except Exception:
                pass

        con = _ensure_blender_sim_geometry_attribute_constraint(playback_target_obj, path_obj)
        # Explicitly initialize to first point on bind to avoid start-offset drift.
        try:
            con.sample_index = 0
            try:
                context.view_layer.update()
            except Exception:
                pass
        except Exception:
            pass

        start_frame = int(context.scene.frame_current)
        frame_step = int(getattr(slot, "newton_keyframe_step", 1) or 1)
        num_pts = len(points)

        if num_pts <= 1:
            con.keyframe_insert(data_path="sample_index", frame=start_frame)
            playback_target_obj.rotation_euler = target_rot
            playback_target_obj.keyframe_insert(data_path="rotation_euler", frame=start_frame)
            end_frame = start_frame
        else:
            # First key at frame start is always point 0 (after explicit pre-bind snap).
            con.sample_index = 0
            con.keyframe_insert(data_path="sample_index", frame=start_frame)
            playback_target_obj.rotation_euler = target_rot
            playback_target_obj.keyframe_insert(data_path="rotation_euler", frame=start_frame)
            for i in range(1, num_pts):
                frame = start_frame + i * frame_step
                con.sample_index = i
                con.keyframe_insert(data_path="sample_index", frame=frame)
                playback_target_obj.rotation_euler = target_rot
                playback_target_obj.keyframe_insert(data_path="rotation_euler", frame=frame)
            end_frame = start_frame + (num_pts - 1) * frame_step

        # Make timeline convenient for inspection.
        try:
            context.scene.frame_start = min(int(context.scene.frame_start), start_frame)
            context.scene.frame_end = max(int(context.scene.frame_end), end_frame)
        except Exception:
            pass

        # Expose the run to the run_idx contract: timeline playback now drives
        # slot.run_idx via the frame_change handler (source of truth: the
        # keyframed sample_index on the playback constraint).
        run_state.start_run(slot, "SIM", path_obj, num_pts)

        _write_newton_debug_text(context, slot, {
            "kind": "blender_sim_validate_path",
            "robot_label": getattr(slot, "label", ""),
            "robot_type": getattr(slot, "robot_type", ""),
            "simulation_mode": getattr(slot, "simulation_mode", ""),
            "target_object": getattr(target_obj, "name", ""),
            "playback_target_object": getattr(playback_target_obj, "name", ""),
            "sim_collection": getattr(getattr(slot, "sim_collection", None), "name", ""),
            "path_source_object": getattr(path_obj, "name", ""),
            "constraint_type": "GEOMETRY_ATTRIBUTE",
            "path_points_count": num_pts,
            "target_rotation_euler_rad": list(target_rot),
            "start_frame": start_frame,
            "end_frame": end_frame,
            "frame_step": frame_step,
            "clear_target_stats": clear_target_stats,
        })

        self.report(
            {"INFO"},
            f"Blender sim path animated on '{playback_target_obj.name}' from path '{path_obj.name}' (pts={num_pts}, start={start_frame}, end={end_frame})",
        )
        return {"FINISHED"}


class ANIMAQUINA_OT_ClearSimulation(Operator):
    bl_idname = "object.animaquina_clear_simulation"
    bl_label = "Clear Simulation"
    bl_description = "Clear Newton-baked keys/markers and remove the sim collection recursively"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        return slot is not None and (slot.sim_collection is not None or _get_newton_bake_target_armature(slot) is not None)

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        armature_obj = _get_newton_bake_target_armature(slot)
        armature_name = _safe_obj_name(armature_obj)
        clear_stats = _clear_newton_ik_keyframes_on_armature(context.scene, armature_obj)
        blender_path_stats = _clear_blender_sim_path_animation(slot, context.scene)
        target_hard_clear_stats = _hard_clear_blender_sim_target_keyframes(slot)
        if getattr(slot, "run_source", "NONE") == "SIM":
            run_state.clear_run(slot)
        sim_name = getattr(getattr(slot, "sim_collection", None), "name", "")
        try:
            simulation.delete_sim_collection(slot)
            deleted = True
        except Exception as exc:
            deleted = False
            self.report({"WARNING"}, f"Could not delete sim collection: {exc}")

        _write_newton_debug_text(context, slot, {
            "kind": "clear_simulation",
            "robot_label": getattr(slot, "label", ""),
            "robot_type": getattr(slot, "robot_type", ""),
            "target_armature": armature_name,
            "sim_collection_name": sim_name,
            "deleted_sim_collection": deleted,
            "clear_stats": clear_stats,
            "blender_path_stats": blender_path_stats,
            "target_hard_clear_stats": target_hard_clear_stats,
        })
        self.report(
            {"INFO"},
            (
                f"Simulation cleared (newton_fc={clear_stats['fcurves_removed']}, "
                f"newton_markers={clear_stats['markers_removed']}, "
                f"follow_path_fc={blender_path_stats['fcurves_removed']}, "
                f"target_xform_fc={blender_path_stats.get('target_transform_fcurves_removed', 0)}, "
                f"target_hard_fc={target_hard_clear_stats.get('fcurves_removed', 0)}, "
                f"follow_path_constraints={blender_path_stats['constraint_removed']}, "
                f"sim_deleted={deleted})"
            ),
        )
        return {"FINISHED"}


class ANIMAQUINA_OT_BlenderSimClearPath(Operator):
    bl_idname = "object.animaquina_blender_clear_path"
    bl_label = "Clear Path"
    bl_description = "Clear Blender simulation path playback (constraints/keyframes) without deleting the sim collection"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        return slot is not None and (getattr(slot, "target_object", None) is not None or getattr(slot, "sim_collection", None) is not None)

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}

        stats = _clear_blender_sim_path_animation(slot, context.scene)
        target_hard_clear_stats = _hard_clear_blender_sim_target_keyframes(slot)
        if getattr(slot, "run_source", "NONE") == "SIM":
            run_state.clear_run(slot)
        restore_target = getattr(slot, "target_object", None) or getattr(slot, "tcp_object", None)
        restored_constraints = 0
        if getattr(slot, "sim_collection", None) is not None and restore_target is not None:
            restored_constraints = _retarget_sim_tcp_constraints(slot, restore_target)

        _write_newton_debug_text(context, slot, {
            "kind": "blender_sim_clear_path",
            "robot_label": getattr(slot, "label", ""),
            "robot_type": getattr(slot, "robot_type", ""),
            "simulation_mode": getattr(slot, "simulation_mode", ""),
            "restore_target": _safe_obj_name(restore_target),
            "restored_tcp_childof_constraints": restored_constraints,
            "clear_stats": stats,
            "target_hard_clear_stats": target_hard_clear_stats,
        })

        self.report(
            {"INFO"},
            (
                f"Blender sim path cleared (fc={stats.get('fcurves_removed', 0)}, "
                f"target_xform_fc={stats.get('target_transform_fcurves_removed', 0)}, "
                f"target_hard_fc={target_hard_clear_stats.get('fcurves_removed', 0)}, "
                f"geo_constraints={stats.get('geo_constraint_removed', 0)}, "
                f"restored_tcp_targets={restored_constraints})"
            )[:240],
        )
        return {"FINISHED"}


# Built-in attribute names that are handled separately (not custom vars)
_BUILTIN_ATTRIBUTES = {"position"}

# Background poll rate while a KUKA stream is running. Polling and the stream
# refill share a single C3 Bridge socket serialized by one driver lock, so the
# poll thread has to give way or it starves refill and blocks the UI thread.
# Low enough to leave the socket to refill, high enough to keep the twin live.
# 5 Hz left the twin visibly stepping during a buffered run (the UI timer redraws
# at poll_rate_hz, so each sample was held for ~5 frames). 10 Hz halves that while
# staying well under the 25 Hz full rate that starved refill in the first place.
_STREAM_POLL_RATE_HZ = 10.0


def _read_custom_var_attributes(slot, eval_obj) -> dict:
    """Read per-point values for each tracked debug variable that has a matching mesh attribute.

    Returns {var_name: [value_per_point, ...]}. Variables without a matching attribute
    are silently skipped (they're still monitored, just not streamed).
    """
    custom_vars = {}
    for item in getattr(slot, "debug_vars", []):
        if not bool(getattr(item, "enabled", True)):
            continue
        # Never drive an auto-added entry from a mesh attribute. The only one is
        # the point-index variable, which the robot writes itself from a
        # point-synchronized KRL TRIGGER - we poll it, we do not produce it.
        # Writing it back would fight the controller for the same variable and,
        # because a matching IDX attribute is exactly what the point-index
        # workflow puts on the toolpath mesh, would also rebuild a full
        # per-point list on the UI thread on every streaming tick.
        if bool(getattr(item, "auto_added", False)):
            continue
        var_name = str(getattr(item, "var_name", "") or "").strip()
        if not var_name or var_name in _BUILTIN_ATTRIBUTES:
            continue
        att = eval_obj.data.attributes.get(var_name)
        if att is None:
            continue
        custom_vars[var_name] = [getattr(v, "value", None) for v in att.data]
    return custom_vars


def _write_custom_vars_for_index(driver, custom_vars: dict, idx: int, prev_custom: dict):
    """Write custom variable values for a given waypoint index, only when changed.

    Updates prev_custom in place only on success so failed writes are retried.
    """
    for var_name, values in custom_vars.items():
        if idx < 0 or idx >= len(values):
            continue
        val = values[idx]
        if val is not None and val != prev_custom.get(var_name):
            try:
                err = driver.write_var(var_name, val)
                if err:
                    print(f"[Animaquina] var write failed: {var_name}={val} idx={idx}: {err}")
                else:
                    prev_custom[var_name] = val
            except Exception as exc:
                print(f"[Animaquina] var write exception: {var_name}={val} idx={idx}: {exc}")


def _get_path_points_from_object(context):
    """
    Get path points from active object. Uses mesh 'position' attribute on evaluated mesh
    (old addon: UR/UF/KUKA use position attribute, not raw vertices). Returns list of (pos_m, euler_rad) or None on error.
    """
    obj = getattr(context, "active_object", None)
    if not obj or obj.type != "MESH":
        return None, "Select a mesh with a 'position' attribute"
    depsgraph = context.evaluated_depsgraph_get()
    eval_obj = obj.evaluated_get(depsgraph)
    if not hasattr(eval_obj.data, "attributes") or "position" not in eval_obj.data.attributes:
        return None, "Mesh has no 'position' attribute (use Geometry Nodes or add attribute)"
    world_matrix = obj.matrix_world
    points = []
    for p in eval_obj.data.attributes["position"].data:
        local_pos = p.vector
        global_pos = world_matrix @ local_pos
        points.append((global_pos.x, global_pos.y, global_pos.z))
    if not points:
        return None, "No points in position attribute"
    return points, None


class ANIMAQUINA_OT_SendPath(Operator):
    bl_idname = "object.animaquina_send_path"
    bl_label = "Run Toolpath"
    bl_description = "Run toolpath from mesh 'position' attribute on the robot"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None or not slot.is_connected:
            return False
        if getattr(slot, "motion_active_label", ""):
            return False
        if bool(getattr(slot, "realtime_puppet_active", False)):
            cls.poll_message_set("Stop Puppet Mode before running a toolpath")
            return False
        if not context.active_object or context.active_object.type != "MESH":
            return False
        driver = manager.get_driver_for_slot(slot)
        return driver and (driver.capabilities() & CAP_EXECUTE_PATH)

    def execute(self, context):
        slot = get_active_slot(context)
        driver = manager.get_driver_for_slot(slot)
        if driver is None:
            return {"CANCELLED"}
        points, err_msg = _get_path_points_from_object(context)
        if points is None:
            self.report({"ERROR"}, err_msg)
            return {"CANCELLED"}
        # tcp_euler_rad is in J0/base frame; the world->base converters apply base_inv,
        # so pass the TCP orientation in world frame to avoid double-rotating a rotated base.
        euler_rad = rig_apply.tcp_world_euler(slot)
        if slot.robot_type == "KUKA":
            waypoints = rig_apply.world_points_to_blender_base_waypoints(slot, points, euler_rad)
        else:
            waypoints = rig_apply.world_points_to_base_waypoints(slot, points, euler_rad)

        if not waypoints:
            self.report({"WARNING"}, "No waypoints")
            return {"CANCELLED"}

        if getattr(slot, "robot_type", "") == "UR":
            count = len(waypoints)
            if count > UR_SENDPATH_HARD_LIMIT:
                msg = (
                    f"UR Send Path aborted: {count} points exceeds safe live-send limit "
                    f"({UR_SENDPATH_HARD_LIMIT}). Use Export URScript + Stage to Robot."
                )
                slot.last_error = msg
                self.report({"ERROR"}, msg[:240])
                return {"CANCELLED"}
            if count > UR_SENDPATH_WARN_POINTS:
                self.report(
                    {"WARNING"},
                    (
                        f"Large UR path ({count} points). Live send may fail; "
                        "prefer Export URScript + Stage to Robot."
                    )[:240],
                )

        wm = context.window_manager
        if getattr(context, "window", None) is None:
            self.report({"ERROR"}, "Cannot start non-blocking send without an active window")
            return {"CANCELLED"}

        self._aq_send_slot = slot
        self._aq_send_done = False
        self._aq_send_error = None

        # Clear abort flag at start so a stale flag doesn't cancel immediately.
        slot.motion_abort_requested = False
        slot.motion_active_label = "Run Toolpath"

        # KUKA: modal streaming with live mesh re-reading
        if slot.robot_type == "KUKA" and hasattr(driver, "stream_start"):
            total = len(waypoints)
            start_pose = _capture_start_pose(slot, driver)
            first_pose = waypoints[0]
            ring_size_cfg = int(max(4, min(128, int(getattr(slot, "kuka_ring_buffer_size", 5) or 5))))
            uploaded_ring_size = int(max(0, int(getattr(slot, "kuka_stream_uploaded_ring_size", 0) or 0)))
            effective_ring_size = ring_size_cfg
            if uploaded_ring_size > 0 and uploaded_ring_size != ring_size_cfg:
                effective_ring_size = uploaded_ring_size
                self.report(
                    {"WARNING"},
                    (
                        f"KUKA buffer mismatch (UI={ring_size_cfg}, uploaded={uploaded_ring_size}). "
                        f"Using uploaded={uploaded_ring_size}; run Upload Dynamic Sync to apply UI value."
                    )[:240],
                )
            elif not bool(getattr(slot, "stream_program_uploaded", False)):
                self.report(
                    {"WARNING"},
                    "Dynamic Sync may be outdated. Use Upload + Select Dynamic Sync after changing settings.",
                )

            # PTP move to first waypoint (blocking, in background thread)
            ptp_vel, ptp_acc = _ptp_motion_params_for_slot(slot)
            self._aq_kuka_phase = "move_to_first"
            self._aq_kuka_driver = driver
            self._aq_kuka_total = total
            self._aq_kuka_start_pose = start_pose
            self._aq_kuka_stream_state = None
            self._aq_kuka_obj = context.active_object
            self._aq_kuka_ring_size = effective_ring_size

            # Shared state dict — background thread writes here instead of to
            # self, which avoids ReferenceError if Blender frees the operator
            # while the thread is still running.
            self._aq_thread_state = {"done": False, "error": None}
            _ts = self._aq_thread_state

            def _move_first():
                err_local = driver.move_to_pose_ptp(
                    first_pose[0], first_pose[1], ptp_vel, ptp_acc, slot.radius, True
                )
                _ts["error"] = err_local
                _ts["done"] = True

            self._aq_send_thread = threading.Thread(target=_move_first, daemon=True)
            self._aq_send_thread.start()

            # Tight tick: refill must outpace the robot's advance run to keep the
            # ring buffer from draining mid-path. 0.02 s (~50 Hz) vs the old 0.05 s.
            self._aq_send_timer = wm.event_timer_add(0.02, window=context.window)
            wm.modal_handler_add(self)
            self.report({"INFO"}, f"KUKA streaming: moving to first point ({total} pts)")
            return {"RUNNING_MODAL"}

        # UR / xArm: background thread (static waypoints)
        start_pose = _capture_start_pose(slot, driver)
        first_pose = waypoints[0]

        # Shared state dict — background thread writes here instead of to
        # self, which avoids ReferenceError if Blender frees the operator
        # while the thread is still running.
        self._aq_thread_state = {"done": False, "error": None}
        _ts = self._aq_thread_state

        def _job():
            ptp_vel, ptp_acc = _ptp_motion_params_for_slot(slot)
            # Move to first waypoint (blocking — must complete before path)
            if getattr(slot, "robot_type", "") == "UR" and hasattr(driver, "move_to_pose"):
                # Keep UR path send speed consistent with Control > Linear Vel/Acc.
                err_local = driver.move_to_pose(
                    first_pose[0], first_pose[1], slot.speed, slot.acc, slot.radius, True
                )
            elif hasattr(driver, "move_to_pose_ptp"):
                err_local = driver.move_to_pose_ptp(
                    first_pose[0], first_pose[1], ptp_vel, ptp_acc, slot.radius, True
                )
            else:
                err_local = driver.move_to_pose(
                    first_pose[0], first_pose[1], slot.speed, slot.acc, slot.radius, True
                )
            # Execute path (blocking in worker thread so return PTP can run afterwards)
            if not err_local:
                err_local = driver.execute_ptp_path(
                    waypoints, slot.speed, slot.acc, slot.radius, True
                )
            # Return to start for UR/xArm after path completion
            if not err_local:
                if getattr(slot, "robot_type", "") == "UR" and hasattr(driver, "move_to_pose"):
                    err_local = driver.move_to_pose(
                        start_pose[0], start_pose[1], slot.speed, slot.acc, slot.radius, True
                    )
                elif hasattr(driver, "move_to_pose_ptp"):
                    err_local = driver.move_to_pose_ptp(
                        start_pose[0], start_pose[1], ptp_vel, ptp_acc, slot.radius, True
                    )
                else:
                    err_local = driver.move_to_pose(
                        start_pose[0], start_pose[1], slot.speed, slot.acc, slot.radius, True
                    )
            _ts["error"] = err_local
            _ts["done"] = True

        self._aq_send_thread = threading.Thread(target=_job, daemon=True)
        self._aq_send_thread.start()

        self._aq_send_timer = wm.event_timer_add(0.1, window=context.window)
        wm.modal_handler_add(self)
        self.report({"INFO"}, "Send started (Move -> Path -> Return)")
        return {"RUNNING_MODAL"}

    # Helper: re-read mesh positions and convert to KUKA base waypoints
    def _kuka_read_live_waypoints(self, context, start=0, count=None):
        """Re-evaluate mesh and return (total_points, waypoints_window).

        The window covers points [start, start+count); count=None means "to the
        end". Returns (0, None) if the mesh can't be read.

        Streaming re-reads the mesh every tick so edits during a run are picked
        up live, but it writes at most ring_size points per tick. Transforming
        the *whole* toolpath each time cost two matrix products and several
        object allocations per point, 50x a second, on Blender's UI thread - for
        a few thousand points that is millions of allocations per second and is
        on its own enough to freeze the viewport and balloon memory. Only the
        points that can actually be sent this tick are transformed now; the
        total is still reported so callers keep their topology-change check.
        """
        slot = self._aq_send_slot
        obj = self._aq_kuka_obj
        if obj is None or obj.type != "MESH":
            return 0, None
        try:
            depsgraph = context.evaluated_depsgraph_get()
            eval_obj = obj.evaluated_get(depsgraph)
            if "position" not in eval_obj.data.attributes:
                return 0, None
            pos_data = eval_obj.data.attributes["position"].data
            total = len(pos_data)

            lo = max(0, int(start))
            hi = total if count is None else min(total, lo + max(0, int(count)))

            world_matrix = obj.matrix_world
            points = []
            for i in range(lo, hi):
                gp = world_matrix @ pos_data[i].vector
                points.append((gp.x, gp.y, gp.z))
            euler_rad = rig_apply.tcp_world_euler(slot)
            # Also refresh custom var attributes while we have the eval_obj
            self._aq_kuka_custom_vars = _read_custom_var_attributes(slot, eval_obj)
            window = rig_apply.world_points_to_blender_base_waypoints(slot, points, euler_rad)
            return total, window
        except Exception:
            return 0, None

    def modal(self, context, event):
        if event.type != "TIMER":
            return {"PASS_THROUGH"}

        slot = getattr(self, "_aq_send_slot", None)
        if slot is None:
            return self._send_path_cleanup(context, None, "No slot")

        # Check abort flag — stop button sets this so we exit cleanly.
        if getattr(slot, "motion_abort_requested", False):
            return self._send_path_cleanup(context, slot, "Stopped by user")

        # KUKA streaming modal
        phase = getattr(self, "_aq_kuka_phase", None)
        if phase is not None:
            return self._kuka_modal_tick(context)

        # UR / xArm: wait for background thread
        _ts = getattr(self, "_aq_thread_state", None)
        if _ts is None or not _ts.get("done"):
            return {"PASS_THROUGH"}
        return self._send_path_cleanup(context, slot, _ts.get("error"))

    # IDX is read on its own cadence rather than every 0.02 s modal tick: it
    # shares the C3 socket with the ring-buffer refill writes, and refill is the
    # time-critical path during a stream.
    _KUKA_IDX_READ_INTERVAL = 0.05
    # How far the planner may run before a still-zero IDX means the controller is
    # running an mq_stream built before the TRIGGER was added.
    _KUKA_IDX_STALE_AFTER = 3

    def _kuka_read_sync_idx(self, driver, rd_idx):
        """Waypoint index synchronised to physical motion, from mq_stream's IDX.

        IDX is incremented by a TRIGGER, which executes in the controller's main
        run, so it tracks where the robot actually is. Falls back to rd_idx (the
        advance-run position) when the controller is running an mq_stream from
        before that TRIGGER existed: there IDX is declared but never written, so
        it would sit at 0 all run and freeze both end-effector dispatch and
        run_idx.
        """
        if getattr(self, "_aq_kuka_idx_stale", False):
            return rd_idx

        now = time.perf_counter()
        if now - getattr(self, "_aq_kuka_idx_t", 0.0) >= self._KUKA_IDX_READ_INTERVAL:
            self._aq_kuka_idx_t = now
            try:
                self._aq_kuka_idx = int(driver.read_var("IDX"))
            except Exception:
                pass

        idx = int(getattr(self, "_aq_kuka_idx", 0) or 0)
        if idx <= 0:
            # The planner has clearly moved on but IDX never left 0 -> old program.
            if rd_idx > self._KUKA_IDX_STALE_AFTER:
                self._aq_kuka_idx_stale = True
                print(
                    "[Animaquina] mq_stream on the controller never updates IDX "
                    "(built before the path-sync TRIGGER). Falling back to the "
                    "advance-run index; re-upload Dynamic Sync to get end-effector "
                    "switching in sync with the robot."
                )
            return rd_idx
        return idx

    def _kuka_modal_tick(self, context):
        slot = self._aq_send_slot
        driver = self._aq_kuka_driver
        phase = self._aq_kuka_phase

        # Phase 1: waiting for PTP move to first waypoint
        if phase == "move_to_first":
            _ts = getattr(self, "_aq_thread_state", None)
            if _ts is None or not _ts.get("done"):
                return {"PASS_THROUGH"}
            err = _ts.get("error")
            if err:
                return self._send_path_cleanup(context, slot, err)
            # Start streaming: read fresh waypoints (only the first ring's worth
            # is needed to prime - stream_start ignores the rest).
            ring_size = int(getattr(self, "_aq_kuka_ring_size", getattr(slot, "kuka_ring_buffer_size", 6)))
            total, head = self._kuka_read_live_waypoints(context, 0, ring_size)
            if not total or not head:
                return self._send_path_cleanup(context, slot, "Cannot read mesh positions")
            state = driver.stream_start(total, slot.speed, slot.radius,
                                        head, ring_size)
            if isinstance(state, str):
                return self._send_path_cleanup(context, slot, state)

            # Throttle background polling for the duration of the stream. The
            # poll thread and these refill writes share one C3 Bridge socket and
            # one driver lock: at the full poll rate the refill path (and with
            # it Blender's UI thread) blocks behind an in-flight read that can
            # sit in a 2 s socket timeout. The twin still updates several times
            # a second. Restored in _send_path_cleanup.
            manager.set_poll_rate(slot, _STREAM_POLL_RATE_HZ)

            # Prime the pipeline before handing control to the slower modal tick.
            # stream_start pre-fills the ring and sets MQ_ACTION=10, so the robot's
            # advance run starts draining the buffer immediately. If we waited for
            # the next ~0.02 s timer tick to refill, the planner can swallow the
            # whole buffer first and hit `WAIT FOR` — the start-of-path stall.
            # Here we top up freed slots in a tight, time-bounded loop so MQ_WR_IDX
            # gets ahead of the advance run before steady-state refilling begins.
            prime_deadline = time.time() + 0.06
            while state["written"] < total and time.time() < prime_deadline:
                try:
                    rd_idx = int(driver.read_var("MQ_RD_IDX"))
                except Exception:
                    break
                _, chunk = self._kuka_read_live_waypoints(context, state["written"], ring_size)
                if not chunk:
                    break
                refilled, err = driver.stream_refill(state, rd_idx, chunk)
                if err:
                    return self._send_path_cleanup(context, slot, err)
                if not refilled:
                    # Buffer full for now — let the advance run consume a slot.
                    time.sleep(0.002)

            self._aq_kuka_stream_state = state
            self._aq_kuka_phase = "streaming"
            self._aq_kuka_prev_custom = {}
            self._aq_kuka_idx = 0
            self._aq_kuka_idx_t = 0.0
            self._aq_kuka_idx_stale = False
            run_state.start_run(slot, "STREAM", self._aq_kuka_obj, total)
            self.report({"INFO"}, f"KUKA streaming: {total} points, buffer {ring_size} (live update active)")
            return {"PASS_THROUGH"}

        # Phase 2: streaming Ã¢â‚¬â€ refill ring buffer with live positions
        if phase == "streaming":
            state = self._aq_kuka_stream_state
            if state["written"] < state["total"]:
                try:
                    rd_idx = int(driver.read_var("MQ_RD_IDX"))
                except Exception:
                    return {"PASS_THROUGH"}
                # Re-read mesh for fresh positions
                # Only the slots writable this tick, not the whole toolpath.
                ring_size = int(getattr(self, "_aq_kuka_ring_size",
                                        getattr(slot, "kuka_ring_buffer_size", 6)))
                cur_total, chunk = self._kuka_read_live_waypoints(
                    context, state["written"], ring_size
                )
                if cur_total == state["total"] and chunk:
                    remaining = chunk
                else:
                    # Mesh topology changed or read failed - cannot continue live
                    remaining = []
                if remaining:
                    _, err = driver.stream_refill(state, rd_idx, remaining)
                    if err:
                        return self._send_path_cleanup(context, slot, err)
                # rd_idx is the PLANNER position: MQ_RD_IDX is a plain KRL
                # assignment, so it advances in the controller's advance run, up
                # to $ADVANCE motion blocks ahead of the robot. Dispatching the
                # end effector off it switches that many waypoints early. Refill
                # above deliberately keeps using rd_idx - a slot the planner has
                # consumed is already baked into the planned motion and is safe to
                # overwrite, and waiting for the physical index would shrink the
                # usable ring buffer.
                sync_idx = self._kuka_read_sync_idx(driver, rd_idx)

                # Write custom variable values for the waypoint the robot is currently at
                custom_vars = getattr(self, "_aq_kuka_custom_vars", {})
                if custom_vars:
                    prev_custom = getattr(self, "_aq_kuka_prev_custom", {})
                    _write_custom_vars_for_index(driver, custom_vars, sync_idx - 1, prev_custom)
                # run_idx contract: the waypoint currently in effect - sync_idx - 1
                # is the point the robot has physically started moving into (same
                # index basis as the custom var dispatch above).
                run_state.set_idx(slot, max(0, sync_idx - 1))
                return {"PASS_THROUGH"}

            # All points written Ã¢â‚¬â€ wait for robot to finish
            if not driver.stream_is_done(state):
                return {"PASS_THROUGH"}
            run_state.set_idx(slot, state["total"] - 1)

            # Phase 3: return to start position
            self._aq_kuka_phase = "returning"
            start_pose = self._aq_kuka_start_pose
            ptp_vel, ptp_acc = _ptp_motion_params_for_slot(slot)
            self._aq_thread_state = {"done": False, "error": None}
            _ts = self._aq_thread_state

            def _return_job():
                err_local = driver.move_to_pose_ptp(
                    start_pose[0], start_pose[1], ptp_vel, ptp_acc, slot.radius, True
                )
                _ts["error"] = err_local
                _ts["done"] = True

            threading.Thread(target=_return_job, daemon=True).start()
            return {"PASS_THROUGH"}

        # Phase 3: waiting for return move
        if phase == "returning":
            _ts = getattr(self, "_aq_thread_state", None)
            if _ts is None or not _ts.get("done"):
                return {"PASS_THROUGH"}
            return self._send_path_cleanup(
                context, slot, _ts.get("error")
            )

        return {"PASS_THROUGH"}

    def _send_path_cleanup(self, context, slot, err):
        """Remove timer and report result."""
        try:
            context.window_manager.event_timer_remove(self._aq_send_timer)
        except Exception:
            pass
        # Clear KUKA state
        self._aq_kuka_phase = None
        self._aq_kuka_ring_size = None
        self._aq_kuka_custom_vars = {}
        self._aq_kuka_prev_custom = {}
        # Undo the streaming poll throttle. Unconditional and before any early
        # return below: this also runs on the error and user-stop paths, and
        # leaving a slot pinned at the stream rate would silently degrade the
        # twin for the rest of the session.
        if slot:
            manager.restore_poll_rate(slot)
        # Clear active motion label
        if slot:
            slot.motion_active_label = ""
            if getattr(slot, "run_source", "NONE") == "STREAM":
                run_state.clear_run(slot)
        if err:
            if slot:
                slot.last_error = err
            self.report({"ERROR"}, err)
            return {"CANCELLED"}
        if slot:
            manager.request_aux_refresh(slot)
            rig_apply.apply_full_pose(slot)
        self.report({"INFO"}, "Path sent")
        return {"FINISHED"}


def _ur_read_live_waypoints(context, slot, obj, custom_vars_out=None, speeds_out=None):
    """Re-read mesh positions and convert to UR base waypoints (for live update).

    If custom_vars_out is a dict, it will be updated with {var_name: [values...]} for
    tracked debug variables that have matching mesh attributes.
    If speeds_out is a list, it is replaced with per-point linear speeds (m/s) from
    the slot's speed attribute (Per-Point Speed); left empty when the attribute is
    missing so callers can fall back to the constant speed.
    """
    if obj is None or obj.type != "MESH":
        return None
    try:
        depsgraph = context.evaluated_depsgraph_get()
        eval_obj = obj.evaluated_get(depsgraph)
        if "position" not in eval_obj.data.attributes:
            return None
        world_matrix = obj.matrix_world
        points = []
        for p in eval_obj.data.attributes["position"].data:
            gp = world_matrix @ p.vector
            points.append((gp.x, gp.y, gp.z))
        euler_rad = rig_apply.tcp_world_euler(slot)
        # Also refresh custom var attributes while we have the eval_obj
        if custom_vars_out is not None:
            custom_vars_out.update(_read_custom_var_attributes(slot, eval_obj))
        # Per-point speed attribute (values in m/s; range-validated in the driver).
        # Only accept a scalar point-domain attribute whose length matches the
        # waypoint count — anything else would silently misalign speeds.
        if speeds_out is not None:
            speeds_out[:] = []
            attr_name = str(getattr(slot, "speed_attribute", "") or "").strip()
            att = eval_obj.data.attributes.get(attr_name) if attr_name else None
            if (
                att is not None
                and att.domain == "POINT"
                and att.data_type in {"FLOAT", "INT"}
            ):
                try:
                    speeds_out[:] = [float(d.value) for d in att.data]
                except Exception:
                    speeds_out[:] = []
                if len(speeds_out) != len(points):
                    speeds_out[:] = []
        return rig_apply.world_points_to_base_waypoints(slot, points, euler_rad)
    except Exception:
        return None


class ANIMAQUINA_OT_SendPathQueueUR(Operator):
    """UR Dynamic Sync streaming — mirrors KUKA SendPath queue pattern.

    Uses RTDE input/output registers as a ring buffer (like KUKA MQ_PT[]).
    A URScript (mq_stream) runs on the controller, reads waypoints from
    input registers and executes movel() with blend. Blender refills the
    buffer as the robot consumes points — exactly like KUKA Dynamic Sync.

    Phases: move_to_first → streaming (register refill loop) → returning
    """
    bl_idname = "object.animaquina_send_path_queue_ur"
    bl_label = "Run Toolpath (Buffered)"
    bl_description = (
        "UR Dynamic Sync: stream path via RTDE register ring buffer "
        "with live mesh updates (like KUKA Dynamic Sync)"
    )
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None or not slot.is_connected or getattr(slot, "robot_type", "") != "UR":
            return False
        if getattr(slot, "motion_active_label", ""):
            return False
        if bool(getattr(slot, "realtime_puppet_active", False)):
            cls.poll_message_set("Stop Puppet Mode before running a toolpath")
            return False
        if not context.active_object or context.active_object.type != "MESH":
            return False
        driver = manager.get_driver_for_slot(slot)
        return driver and (driver.capabilities() & CAP_EXECUTE_PATH) and hasattr(driver, "stream_start")

    def execute(self, context):
        slot = get_active_slot(context)
        driver = manager.get_driver_for_slot(slot)
        if slot is None or driver is None:
            return {"CANCELLED"}

        points, err_msg = _get_path_points_from_object(context)
        if points is None:
            self.report({"ERROR"}, err_msg)
            return {"CANCELLED"}

        euler_rad = rig_apply.tcp_world_euler(slot)
        waypoints = rig_apply.world_points_to_base_waypoints(slot, points, euler_rad)
        if not waypoints:
            self.report({"WARNING"}, "No waypoints")
            return {"CANCELLED"}

        count = len(waypoints)

        wm = context.window_manager
        if getattr(context, "window", None) is None:
            self.report({"ERROR"}, "Cannot start non-blocking send without an active window")
            return {"CANCELLED"}

        ring_size = int(max(2, min(6, int(getattr(slot, "ur_queue_buffer_size", 6) or 6))))
        start_pose = _capture_start_pose(slot, driver)
        first_pose = waypoints[0]

        # State (mirrors KUKA SendPath modal state)
        self._aq_send_slot = slot
        self._aq_send_done = False
        self._aq_send_error = None
        self._aq_ur_phase = "move_to_first"

        # Clear abort flag and set active label for Program Controls display.
        slot.motion_abort_requested = False
        slot.motion_active_label = "Run Toolpath (Buffered)"
        self._aq_ur_driver = driver
        self._aq_ur_total = count
        self._aq_ur_start_pose = start_pose
        self._aq_ur_stream_state = None
        self._aq_ur_obj = context.active_object
        self._aq_ur_ring_size = ring_size
        self._aq_ur_waypoints = waypoints

        slot.ur_transfer_status = "RUNNING"
        slot.ur_transfer_log = f"UR Dynamic Sync started ({count} pts, ring {ring_size}) ..."
        slot.ur_queue_health = f"phase=move_to_first rd=0 written=0/{count} done=0"

        # Phase 1: PTP to first waypoint (blocking in worker thread)
        ptp_vel, ptp_acc = _ptp_motion_params_for_slot(slot)
        self._aq_thread_state = {"done": False, "error": None}
        _ts = self._aq_thread_state

        def _move_first():
            if hasattr(driver, "move_to_pose_ptp"):
                err = driver.move_to_pose_ptp(
                    first_pose[0], first_pose[1], ptp_vel, ptp_acc, slot.radius, True
                )
            else:
                err = driver.move_to_pose(
                    first_pose[0], first_pose[1], slot.speed, slot.acc, slot.radius, True
                )
            _ts["error"] = err
            _ts["done"] = True

        self._aq_send_thread = threading.Thread(target=_move_first, daemon=True)
        self._aq_send_thread.start()

        self._aq_send_timer = wm.event_timer_add(0.05, window=context.window)
        wm.modal_handler_add(self)
        self.report({"INFO"}, f"UR Dynamic Sync: moving to first point ({count} pts)")
        return {"RUNNING_MODAL"}

    def modal(self, context, event):
        if event.type != "TIMER":
            return {"PASS_THROUGH"}

        slot = getattr(self, "_aq_send_slot", None)
        if slot is None:
            return self._ur_queue_cleanup(context, None, "No slot")

        # Check abort flag — stop button sets this so we exit cleanly.
        if getattr(slot, "motion_abort_requested", False):
            return self._ur_queue_cleanup(context, slot, "Stopped by user")

        phase = getattr(self, "_aq_ur_phase", None)
        if phase is not None:
            return self._ur_modal_tick(context)

        # Fallback: wait for background thread (shouldn't reach here)
        if not getattr(self, "_aq_send_done", False):
            return {"PASS_THROUGH"}
        return self._ur_queue_cleanup(
            context, slot, getattr(self, "_aq_send_error", None)
        )

    # ── UR streaming modal (mirrors _kuka_modal_tick) ─────────────

    def _ur_modal_tick(self, context):
        slot = self._aq_send_slot
        driver = self._aq_ur_driver
        phase = self._aq_ur_phase

        # Phase 1: waiting for PTP move to first waypoint
        if phase == "move_to_first":
            _ts = getattr(self, "_aq_thread_state", None)
            if _ts is None or not _ts.get("done"):
                return {"PASS_THROUGH"}
            err = _ts.get("error")
            if err:
                return self._ur_queue_cleanup(context, slot, err)
            # Start streaming: read fresh waypoints + custom var attributes
            cv_out = {}
            sp_out = [] if getattr(slot, "use_speed_attribute", False) else None
            waypoints = _ur_read_live_waypoints(
                context, slot, self._aq_ur_obj, custom_vars_out=cv_out, speeds_out=sp_out
            )
            if not waypoints:
                return self._ur_queue_cleanup(context, slot, "Cannot read mesh positions")
            if cv_out:
                self._aq_ur_custom_vars = cv_out
            ring_size = self._aq_ur_ring_size
            total = len(waypoints)
            state = driver.stream_start(
                total,
                slot.speed,
                slot.radius,
                waypoints[:ring_size],
                ring_size,
                slot.acc,
                speeds=sp_out if sp_out else None,
            )
            if isinstance(state, str):
                return self._ur_queue_cleanup(context, slot, state)
            self._aq_ur_stream_state = state
            self._aq_ur_phase = "streaming"
            self._aq_ur_total = total
            self._aq_ur_waypoints = waypoints
            run_state.start_run(slot, "STREAM", self._aq_ur_obj, total)
            # custom_vars already set from cv_out above; only init prev tracking
            if not getattr(self, "_aq_ur_custom_vars", None):
                self._aq_ur_custom_vars = {}
            self._aq_ur_prev_custom = {}
            slot.ur_queue_health = (
                f"phase=streaming cmd={state.get('cmd_id', 0)} sent=0 "
                f"queued={state.get('written', 0)}/{state.get('total', 0)}"
            )
            slot.ur_transfer_log = f"UR servoL streaming: {total} points (live update active)"
            self.report({"INFO"}, f"UR servoL streaming: {total} points")
            return {"PASS_THROUGH"}

        # Phase 2: streaming — refill servoL queue with live positions
        if phase == "streaming":
            state = self._aq_ur_stream_state
            # Check for worker thread errors
            servo_err = state.get("error", "")
            if servo_err:
                return self._ur_queue_cleanup(context, slot, servo_err)

            if state["written"] < state["total"]:
                # Re-read mesh for fresh positions + custom var attributes
                cv_out = {}
                sp_out = [] if getattr(slot, "use_speed_attribute", False) else None
                live_waypoints = _ur_read_live_waypoints(
                    context, slot, self._aq_ur_obj, custom_vars_out=cv_out, speeds_out=sp_out
                )
                if cv_out:
                    self._aq_ur_custom_vars = cv_out
                # Live-update per-point speeds (worker reads state["speeds"])
                if sp_out is not None and len(sp_out) == state["total"]:
                    state["speeds"] = sp_out
                if live_waypoints and len(live_waypoints) == state["total"]:
                    self._aq_ur_waypoints = live_waypoints
                waypoints = getattr(self, "_aq_ur_waypoints", None)
                if waypoints and len(waypoints) == state["total"]:
                    remaining = waypoints[state["written"]:]
                else:
                    return self._ur_queue_cleanup(
                        context, slot, "UR queue aborted: live path point count changed during streaming"
                    )
                if remaining:
                    _, err = driver.stream_refill(state, 0, remaining)
                    if err:
                        return self._ur_queue_cleanup(context, slot, err)
                    slot.ur_transfer_log = f"UR streaming: {state['written']}/{state['total']} queued"
                # Write custom variable values for the waypoint the robot is currently at
                # Use waypoints_completed (source waypoints fully reached), NOT servo_sent
                # (which counts every ~8ms servoL substep and races ahead)
                wp_done = state.get("waypoints_completed", 0)
                custom_vars = getattr(self, "_aq_ur_custom_vars", {})
                if custom_vars and wp_done > 0:
                    prev_custom = getattr(self, "_aq_ur_prev_custom", {})
                    _write_custom_vars_for_index(driver, custom_vars, wp_done - 1, prev_custom)
                # run_idx contract: the waypoint currently in effect — the point
                # being interpolated toward (waypoints_completed segments are
                # done, so the robot is moving into point wp_done). set_idx
                # clamps to run_count - 1 for the final segment/finish.
                run_state.set_idx(slot, wp_done)
                slot.ur_queue_health = (
                    f"phase=streaming wp={wp_done}/{state.get('total', 0)} "
                    f"queued={state.get('written', 0)}/{state.get('total', 0)}"
                )
                return {"PASS_THROUGH"}

            # All points written — wait for robot to finish
            if not driver.stream_is_done(state):
                return {"PASS_THROUGH"}
            run_state.set_idx(slot, state["total"] - 1)

            # Phase 3: return to start position
            self._aq_ur_phase = "returning"
            start_pose = self._aq_ur_start_pose
            ptp_vel, ptp_acc = _ptp_motion_params_for_slot(slot)
            slot.ur_queue_health = (
                f"phase=returning sent={state.get('servo_sent', 0)} "
                f"queued={state.get('written', 0)}/{state.get('total', 0)}"
            )

            # Restore RTDE control script before return PTP
            driver.stream_restore_control()

            self._aq_thread_state = {"done": False, "error": None}
            _ts = self._aq_thread_state

            def _return_job():
                err_local = driver.move_to_pose_ptp(
                    start_pose[0], start_pose[1], ptp_vel, ptp_acc, slot.radius, True
                )
                _ts["error"] = err_local
                _ts["done"] = True

            threading.Thread(target=_return_job, daemon=True).start()
            slot.ur_transfer_log = "Returning to start position ..."
            return {"PASS_THROUGH"}

        # Phase 3: waiting for return move
        if phase == "returning":
            _ts = getattr(self, "_aq_thread_state", None)
            if _ts is None or not _ts.get("done"):
                return {"PASS_THROUGH"}
            return self._ur_queue_cleanup(
                context, slot, _ts.get("error")
            )

        return {"PASS_THROUGH"}

    def _ur_queue_cleanup(self, context, slot, err):
        """Remove timer, restore RTDE, report result."""
        try:
            context.window_manager.event_timer_remove(self._aq_send_timer)
        except Exception:
            pass

        # Restore RTDE control script
        driver = getattr(self, "_aq_ur_driver", None)
        if driver and hasattr(driver, "stream_restore_control"):
            try:
                driver.stream_restore_control()
            except Exception:
                pass

        self._aq_ur_phase = None
        self._aq_ur_stream_state = None
        self._aq_ur_waypoints = None
        self._aq_ur_custom_vars = {}
        self._aq_ur_prev_custom = {}

        # Clear active motion label
        if slot:
            slot.motion_active_label = ""
            if getattr(slot, "run_source", "NONE") == "STREAM":
                run_state.clear_run(slot)

        if err:
            if slot:
                slot.last_error = str(err)
                slot.ur_transfer_status = "ERROR"
                slot.ur_transfer_log = str(err)[:2000]
                slot.ur_queue_health = f"phase=error {str(err)[:120]}"
            self.report({"ERROR"}, str(err)[:240])
            return {"CANCELLED"}

        if slot:
            slot.ur_transfer_status = "OK"
            total = getattr(self, "_aq_ur_total", 0)
            msg = f"UR Dynamic Sync done ({total} pts)"
            slot.ur_transfer_log = msg
            slot.ur_queue_health = f"phase=done written={total}/{total}"
            manager.request_aux_refresh(slot)
            rig_apply.apply_full_pose(slot)
            self.report({"INFO"}, msg[:240])
        return {"FINISHED"}


class ANIMAQUINA_OT_ValidatePath(Operator):
    bl_idname = "object.animaquina_validate_path"
    bl_label = "Validate Path"
    bl_description = "Validate path from mesh 'position' attribute using Newton Phase 1 worker"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None:
            return False
        obj = getattr(context, "active_object", None)
        if not obj or obj.type != "MESH":
            return False
        return bool(obj.data.attributes.get("position"))

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}

        points, err_msg = _get_path_points_from_object(context)
        if points is None:
            self.report({"ERROR"}, err_msg)
            return {"CANCELLED"}

        # Build waypoints in armature/MuJoCo frame.
        # MuJoCo model is exported in armature-local space (armature origin = MuJoCo world origin).
        # Per-point rotation from mesh attribute (when enabled)
        _per_point_eulers_arm = None
        # Using rig_armature.matrix_world (not base_object) ensures correct frame alignment
        # when the armature has any local transform relative to J0 Ã¢â‚¬â€ e.g. a rest-pose rotation.
        # If the armature is directly at J0 with identity local transform, the two are identical.
        arm_inv = rig_apply.get_slot_armature_world_matrix(slot).inverted()
        target_obj = getattr(slot, "target_object", None)
        if target_obj is not None:
            euler_rad = tuple((arm_inv @ target_obj.matrix_world).to_euler("XYZ"))
        else:
            euler_rad = tuple(slot.tcp_euler_rad)

        if bool(getattr(slot, "export_use_rotation_attribute", False)):
            obj = context.active_object
            if obj and obj.type == "MESH":
                depsgraph = context.evaluated_depsgraph_get()
                eval_obj = obj.evaluated_get(depsgraph)
                rotation_att = eval_obj.data.attributes.get("rotation")
                if rotation_att is not None:
                    world_matrix = obj.matrix_world
                    apply_rot_transform = bool(getattr(slot, "export_rotation_apply_transform", False))
                    _per_point_eulers_arm = []
                    for r in rotation_att.data:
                        local_euler_deg = r.vector
                        local_euler = (math.radians(local_euler_deg[0]), math.radians(local_euler_deg[1]), math.radians(local_euler_deg[2]))
                        if apply_rot_transform:
                            world_rot = world_matrix.to_3x3().to_4x4() @ Euler(local_euler, "XYZ").to_matrix().to_4x4()
                            arm_rot = arm_inv @ world_rot
                            _per_point_eulers_arm.append(tuple(arm_rot.to_euler("XYZ")))
                        else:
                            _per_point_eulers_arm.append(local_euler)

        waypoints = []
        for i, pos_m in enumerate(points):
            loc = (arm_inv @ Matrix.Translation((float(pos_m[0]), float(pos_m[1]), float(pos_m[2])))).to_translation()
            wp_euler = _per_point_eulers_arm[i] if _per_point_eulers_arm and i < len(_per_point_eulers_arm) else euler_rad
            waypoints.append(((loc.x, loc.y, loc.z), wp_euler))

        options = _build_newton_validation_options(context, slot)
        request = newton_validator.build_validation_request(slot, waypoints, euler_rad, options=options)

        self._aq_slot = slot
        self._aq_points_count = len(points)
        self._aq_waypoints_count = len(waypoints)
        self._aq_options = options
        self._aq_job_done = False
        self._aq_job_error = None
        self._aq_job_report = None

        slot.validation_last_status = "RUNNING"
        slot.validation_last_message = "Newton validation running..."

        def _job():
            try:
                self._aq_job_report = newton_validator.validate_prebuilt_request(request, use_persistent=True)
            except Exception as exc:
                self._aq_job_error = exc
            finally:
                self._aq_job_done = True

        self._aq_thread = threading.Thread(target=_job, daemon=True)
        self._aq_thread.start()
        wm = context.window_manager
        self._aq_timer = wm.event_timer_add(0.1, window=context.window)
        wm.modal_handler_add(self)
        return {"RUNNING_MODAL"}

    def modal(self, context, event):
        if event.type != "TIMER":
            return {"PASS_THROUGH"}
        if not getattr(self, "_aq_job_done", False):
            return {"PASS_THROUGH"}

        try:
            context.window_manager.event_timer_remove(self._aq_timer)
        except Exception:
            pass

        slot = getattr(self, "_aq_slot", None)
        if slot is None:
            return {"CANCELLED"}

        if getattr(self, "_aq_job_error", None) is not None:
            exc = self._aq_job_error
            msg = f"Validation failed: {exc}"
            slot.validation_last_status = "ERROR"
            slot.validation_last_message = msg
            slot.validation_last_backend = ""
            slot.validation_last_backend_message = ""
            slot.validation_last_worker_stderr = ""
            slot.validation_last_contacts = ""
            slot.validation_last_debug = ""
            slot.validation_last_failure_index = -1
            slot.last_error = msg
            _write_newton_debug_text(context, slot, {
                "kind": "validate_path_error",
                "robot_label": getattr(slot, "label", ""),
                "robot_type": getattr(slot, "robot_type", ""),
                "newton_python_exe": getattr(slot, "newton_python_exe", ""),
                "options": getattr(self, "_aq_options", {}),
                "error": str(exc),
                "path_points_count": int(getattr(self, "_aq_points_count", 0)),
            })
            self.report({"ERROR"}, msg)
            return {"CANCELLED"}

        return _apply_newton_validation_report(
            context,
            slot,
            self._aq_job_report or {},
            points_count=int(getattr(self, "_aq_points_count", 0)),
            waypoints_count=int(getattr(self, "_aq_waypoints_count", 0)),
            options=dict(getattr(self, "_aq_options", {}) or {}),
            op=self,
        )


class ANIMAQUINA_OT_NewtonProbe(Operator):
    bl_idname = "object.animaquina_newton_probe"
    bl_label = "Probe Newton"
    bl_description = "Probe the configured Python environment and report Newton/MuJoCo import status"
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        return get_active_slot(context) is not None

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        try:
            backend_info = newton_validator.probe_worker(slot)
        except Exception as exc:
            msg = f"Probe failed: {exc}"
            slot.validation_last_status = "ERROR"
            slot.validation_last_backend = ""
            slot.validation_last_backend_message = msg
            slot.validation_last_worker_stderr = ""
            slot.validation_last_message = msg
            self.report({"ERROR"}, msg)
            return {"CANCELLED"}

        backend = str(backend_info.get("backend") or "unknown")
        backend_msg = str(backend_info.get("message") or "")
        worker_stderr = str(backend_info.get("worker_stderr") or "")
        pyexe = str(backend_info.get("python_executable") or _get_preferred_newton_python(slot))

        slot.validation_last_backend = backend
        slot.validation_last_backend_message = backend_msg
        slot.validation_last_worker_stderr = worker_stderr[:2000]
        slot.validation_last_message = f"Probe python: {pyexe}"
        slot.validation_last_status = "PROBE"
        _write_newton_debug_text(context, slot, {
            "kind": "probe",
            "robot_label": getattr(slot, "label", ""),
            "robot_type": getattr(slot, "robot_type", ""),
            "newton_python_exe": pyexe,
            "backend_info": backend_info,
        })

        summary = f"Probe backend={backend}"
        if backend_msg:
            summary += f" - {backend_msg}"
        self.report({"INFO"}, summary[:240])
        return {"FINISHED"}


class ANIMAQUINA_OT_NewtonInstallDeps(Operator):
    bl_idname = "object.animaquina_newton_install_deps"
    bl_label = "Install Newton Deps"
    bl_description = "Install Newton/MuJoCo packages into the selected Python (or Blender Python if empty)"
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        return get_active_slot(context) is not None

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}

        python_exe = _get_preferred_newton_python(slot)
        # Persist the effective interpreter for transparency (field is informational).
        slot.newton_python_exe = python_exe

        startupinfo = None
        creationflags = 0
        if sys.platform.startswith("win"):
            startupinfo = subprocess.STARTUPINFO()
            startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
            creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)

        commands = [
            [python_exe, "-m", "pip", "install", "--upgrade", "pip", "setuptools", "wheel"],
            [python_exe, "-m", "pip", "install", "--upgrade", "numpy"],
            [python_exe, "-m", "pip", "install", "--upgrade", "mujoco", "mujoco-warp"],
            [python_exe, "-m", "pip", "install", "--pre", "--upgrade", "warp-lang", "-f", "https://pypi.nvidia.com/warp-lang/"],
            [python_exe, "-m", "pip", "install", "--upgrade", "newton"],
        ]
        log_lines = [f"Python: {python_exe}"]
        slot.newton_install_status = "RUNNING"
        slot.newton_install_log = "Installing Newton dependencies..."

        pip_ok, pip_err = _bootstrap_pip(python_exe, log_lines)
        if not pip_ok:
            slot.newton_install_status = "ERROR"
            slot.newton_install_log = "\n".join(log_lines)[-4000:]
            self.report({"ERROR"}, pip_err[:240])
            return {"CANCELLED"}

        for cmd in commands:
            try:
                proc = subprocess.run(
                    cmd,
                    capture_output=True,
                    text=True,
                    encoding="utf-8",
                    errors="replace",
                    timeout=600,
                    startupinfo=startupinfo,
                    creationflags=creationflags,
                )
            except subprocess.TimeoutExpired:
                msg = f"Timeout running: {' '.join(cmd)}"
                log_lines.append(msg)
                slot.newton_install_status = "ERROR"
                slot.newton_install_log = "\n".join(log_lines)[-4000:]
                self.report({"ERROR"}, msg)
                return {"CANCELLED"}
            except Exception as exc:
                msg = f"Install failed starting command: {exc}"
                log_lines.append(msg)
                slot.newton_install_status = "ERROR"
                slot.newton_install_log = "\n".join(log_lines)[-4000:]
                self.report({"ERROR"}, msg[:240])
                return {"CANCELLED"}

            log_lines.append(f"$ {' '.join(cmd)}")
            if proc.stdout:
                log_lines.append(proc.stdout.strip())
            if proc.stderr:
                log_lines.append(proc.stderr.strip())
            if proc.returncode != 0:
                msg = f"Install command failed (code {proc.returncode}): {' '.join(cmd)}"
                slot.newton_install_status = "ERROR"
                slot.newton_install_log = "\n".join(log_lines)[-4000:]
                self.report({"ERROR"}, msg[:240])
                return {"CANCELLED"}

        slot.newton_install_status = "OK"
        slot.newton_install_log = "\n".join(log_lines)[-4000:]

        # Force validator workers to restart so backend detection is refreshed after install.
        try:
            newton_validator.shutdown_persistent_workers()
        except Exception:
            pass

        # Probe worker once so UI backend state updates immediately and validates imports.
        try:
            backend_info = newton_validator.probe_worker(slot)
            newton_ok = bool(backend_info.get("newton_available"))
            mujoco_ok = bool(backend_info.get("mujoco_available"))
            slot.validation_last_backend = str(backend_info.get("backend") or "")
            slot.validation_last_backend_message = str(backend_info.get("message") or "")
            slot.validation_last_worker_stderr = str(backend_info.get("worker_stderr") or "")[:2000]
            slot.validation_last_status = "PROBE"
            slot.validation_last_message = f"Probe python: {backend_info.get('python_executable') or python_exe}"
            log_lines.append(f"Probe backend: {slot.validation_last_backend}")
            if slot.validation_last_backend_message:
                log_lines.append(slot.validation_last_backend_message)
            if not (newton_ok and mujoco_ok):
                msg = "Install finished but worker probe reports Newton/MuJoCo unavailable"
                slot.newton_install_status = "ERROR"
                slot.newton_install_log = "\n".join(log_lines)[-4000:]
                self.report({"ERROR"}, msg)
                return {"CANCELLED"}
        except Exception as exc:
            msg = f"Probe after install failed: {exc}"
            log_lines.append(msg)
            slot.newton_install_status = "ERROR"
            slot.newton_install_log = "\n".join(log_lines)[-4000:]
            self.report({"ERROR"}, msg[:240])
            return {"CANCELLED"}

        slot.newton_install_status = "OK"
        slot.newton_install_log = "\n".join(log_lines)[-4000:]
        self.report({"INFO"}, "Newton dependencies install complete and verified")
        return {"FINISHED"}


# Per-package install operator

# Each entry: (pip_name, label, description of what needs it)
# NOTE: ur_rtde is deliberately NOT listed. Animaquina bundles a patched
# ur_rtde master build (cp313, in vendor_py) that is required for
# PolyScope X — the stock PyPI release (1.6.3) silently fails there.
# Installing ur_rtde via pip would overwrite the patched build.
DEP_PACKAGES = [
    ("paramiko",      "paramiko",      "UR — SFTP file transfer"),
    ("xarm-python-sdk", "xarm-python-sdk", "xArm — UFactory SDK (fallback if bundled fails)"),
    ("numpy",         "numpy",         "Newton — numerical computing"),
    ("mujoco",        "mujoco",        "Newton — MuJoCo physics engine"),
    ("mujoco-warp",   "mujoco-warp",   "Newton — MuJoCo Warp backend"),
    ("warp-lang",     "warp-lang",     "Newton — NVIDIA Warp (GPU sim)"),
    ("newton",        "newton",        "Newton — IK / simulation solver"),
]

# Packages that need special pip flags
_DEP_EXTRA_FLAGS = {
    "warp-lang": ["--pre", "-f", "https://pypi.nvidia.com/warp-lang/"],
}


def _is_package_installed(pip_name):
    """Check if a package exists in the addon vendor_py directory."""
    vendor_dir = _get_addon_vendor_python_dir()
    if not os.path.isdir(vendor_dir):
        return False
    # Look for the dist-info directory that pip creates on install.
    pkg_prefix = pip_name.replace("-", "_").lower()
    try:
        for entry in os.listdir(vendor_dir):
            if entry.lower().startswith(pkg_prefix) and entry.endswith(".dist-info"):
                return True
    except OSError:
        pass
    return False


class ANIMAQUINA_OT_InstallPackage(Operator):
    bl_idname = "object.animaquina_install_package"
    bl_label = "Install Package"
    bl_description = "Install a single Python package into the addon vendor directory"
    bl_options = {"REGISTER"}

    package: bpy.props.StringProperty(name="Package", default="")

    @classmethod
    def poll(cls, context):
        return get_active_slot(context) is not None

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None or not self.package:
            return {"CANCELLED"}

        pip_name = self.package
        python_exe = _get_preferred_ur_python()
        if not python_exe:
            self.report({"ERROR"}, "Cannot find Python executable")
            return {"CANCELLED"}

        vendor_dir = _ensure_current_session_vendor_path()
        log_lines = [f"Python: {python_exe}", f"Vendor dir: {vendor_dir}"]

        pip_ok, pip_err = _bootstrap_pip(python_exe, log_lines)
        if not pip_ok:
            self.report({"ERROR"}, pip_err[:240])
            return {"CANCELLED"}

        cmd = [python_exe, "-m", "pip", "install", "--upgrade", "--target", vendor_dir]
        cmd += _DEP_EXTRA_FLAGS.get(pip_name, [])
        cmd.append(pip_name)

        try:
            proc = _run_pip_command(cmd, timeout=600)
        except subprocess.TimeoutExpired:
            self.report({"ERROR"}, f"Timeout installing {pip_name}")
            return {"CANCELLED"}
        except Exception as exc:
            self.report({"ERROR"}, f"Install failed: {exc}"[:240])
            return {"CANCELLED"}

        log_lines.append(f"$ {' '.join(cmd)}")
        if proc.stdout:
            log_lines.append(proc.stdout.strip())
        if proc.stderr:
            log_lines.append(proc.stderr.strip())

        if proc.returncode != 0:
            # Show the actual pip error in the report, not just the code
            stderr_tail = (proc.stderr or "").strip().splitlines()
            detail = stderr_tail[-1] if stderr_tail else f"code {proc.returncode}"
            self.report({"ERROR"}, f"{pip_name}: {detail}"[:240])
            return {"CANCELLED"}

        importlib.invalidate_caches()
        self.report({"INFO"}, f"Installed {pip_name}")
        return {"FINISHED"}


def _get_preferred_ur_python():
    """Python executable for UR RTDE install — uses sys.prefix lookup (same strategy
    as blenderpipinstaller addon, reliable across Blender versions on Windows/macOS)."""
    for candidate in (
        os.path.join(sys.prefix, "bin", "python.exe"),
        os.path.join(sys.prefix, "python.exe"),
        os.path.join(sys.prefix, "Scripts", "python.exe"),
        os.path.join(sys.prefix, "bin", "python3"),
        os.path.join(sys.prefix, "bin", "python"),
    ):
        if os.path.exists(candidate):
            return candidate
    py_path = getattr(getattr(bpy, "app", None), "binary_path_python", "") or ""
    if py_path and os.path.exists(py_path):
        return py_path
    # Last resort: sys.executable may be blender.exe on Windows — avoid that.
    exe = sys.executable
    if exe and os.path.basename(exe).lower().startswith("blender"):
        return ""
    return exe


def _get_addon_vendor_python_dir() -> str:
    return os.path.join(os.path.dirname(os.path.dirname(__file__)), "vendor_py")


def _ensure_current_session_vendor_path() -> str:
    vendor_dir = _get_addon_vendor_python_dir()
    os.makedirs(vendor_dir, exist_ok=True)
    if vendor_dir not in sys.path:
        sys.path.insert(0, vendor_dir)
    importlib.invalidate_caches()
    return vendor_dir


def _refresh_current_session_python_paths(python_exe: str) -> tuple[list[str], str]:
    """Add the target interpreter's import paths into the current Blender session."""
    startupinfo = None
    creationflags = 0
    if sys.platform.startswith("win"):
        startupinfo = subprocess.STARTUPINFO()
        startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)

    probe = [
        python_exe,
        "-c",
        (
            "import json, sys; "
            "print(json.dumps([p for p in sys.path if isinstance(p, str)]))"
        ),
    ]
    try:
        proc = subprocess.run(
            probe,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=30,
            startupinfo=startupinfo,
            creationflags=creationflags,
        )
    except Exception as exc:
        return [], f"Failed to inspect Python paths for {python_exe}: {exc}"

    if proc.returncode != 0:
        details = proc.stderr.strip() or proc.stdout.strip() or f"exit code {proc.returncode}"
        return [], f"Failed to inspect Python paths for {python_exe}: {details}"

    try:
        candidate_paths = json.loads(proc.stdout.strip() or "[]")
    except Exception as exc:
        return [], f"Failed to parse Python paths for {python_exe}: {exc}"

    existing = {
        os.path.normcase(os.path.abspath(path))
        for path in sys.path
        if isinstance(path, str) and path and os.path.isdir(path)
    }
    pending = []
    for raw_path in candidate_paths:
        path = str(raw_path or "").strip()
        if not path or not os.path.isdir(path):
            continue
        norm = os.path.normcase(os.path.abspath(path))
        if norm in existing:
            continue
        existing.add(norm)
        pending.append(path)

    for path in reversed(pending):
        sys.path.insert(0, path)

    importlib.invalidate_caches()
    return pending, ""


def _pip_startup_options():
    startupinfo = None
    creationflags = 0
    if sys.platform.startswith("win"):
        startupinfo = subprocess.STARTUPINFO()
        startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    return startupinfo, creationflags


def _bootstrap_pip(python_exe, log_lines):
    """Ensure pip is available on python_exe using ensurepip (bundled with Python).

    Mirrors the strategy from the blenderpipinstaller addon which reliably
    works with Blender's embedded Python on Windows and macOS.
    Returns (ok: bool, error_msg: str).
    """
    startupinfo, creationflags = _pip_startup_options()

    # Check if pip is already present.
    probe = subprocess.run(
        [python_exe, "-m", "pip", "--version"],
        capture_output=True,
        startupinfo=startupinfo,
        creationflags=creationflags,
    )
    if probe.returncode == 0:
        log_lines.append("pip already available — skipping bootstrap")
        return True, ""

    log_lines.append("pip not found — bootstrapping via ensurepip...")
    try:
        proc = subprocess.run(
            [python_exe, "-m", "ensurepip", "--default-pip"],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=60,
            startupinfo=startupinfo,
            creationflags=creationflags,
        )
        log_lines.append(f"$ {python_exe} -m ensurepip --default-pip")
        if proc.stdout:
            log_lines.append(proc.stdout.strip())
        if proc.stderr:
            log_lines.append(proc.stderr.strip())
        if proc.returncode != 0:
            return False, f"ensurepip failed (code {proc.returncode})"
    except subprocess.TimeoutExpired:
        return False, "Timeout running ensurepip"
    except Exception as exc:
        return False, f"Error running ensurepip: {exc}"

    # Upgrade pip now that it's available.
    upgrade = subprocess.run(
        [python_exe, "-m", "pip", "install", "--upgrade", "pip"],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=120,
        startupinfo=startupinfo,
        creationflags=creationflags,
    )
    if upgrade.stdout:
        log_lines.append(upgrade.stdout.strip())
    if upgrade.stderr:
        log_lines.append(upgrade.stderr.strip())

    log_lines.append("pip bootstrap complete")
    return True, ""


def _run_pip_command(cmd, *, timeout: int):
    startupinfo, creationflags = _pip_startup_options()
    return subprocess.run(
        cmd,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=timeout,
        startupinfo=startupinfo,
        creationflags=creationflags,
    )


class ANIMAQUINA_OT_InstallURDeps(Operator):
    bl_idname = "object.animaquina_install_ur_deps"
    bl_label = "Install UR Dependencies"
    bl_description = (
        "Install UR dependencies (paramiko for SFTP) into the addon vendor "
        "directory. ur_rtde ships bundled with Animaquina (patched build "
        "required for PolyScope X) and is only pip-installed if the bundled "
        "copy is missing"
    )
    bl_options = {"REGISTER"}

    @staticmethod
    def _bundled_ur_rtde_present() -> bool:
        """True when the addon's vendor_py carries the bundled ur_rtde natives."""
        vendor_dir = _get_addon_vendor_python_dir()
        if not vendor_dir or not os.path.isdir(vendor_dir):
            return False
        try:
            return any(
                entry.startswith("rtde_control.") and entry.endswith(".pyd")
                for entry in os.listdir(vendor_dir)
            )
        except OSError:
            return False

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        return slot is not None and getattr(slot, "robot_type", "") == "UR"

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None or getattr(slot, "robot_type", "") != "UR":
            return {"CANCELLED"}

        python_exe = _get_preferred_ur_python()
        if not python_exe:
            slot.ur_install_status = "ERROR"
            slot.ur_install_log = (
                f"Could not locate Blender's Python executable.\n"
                f"sys.prefix={sys.prefix}  sys.executable={sys.executable}\n"
                f"Please run Blender as administrator or set a custom Python path."
            )
            self.report({"ERROR"}, "Cannot find Blender Python — see install log")
            return {"CANCELLED"}

        vendor_dir = _ensure_current_session_vendor_path()
        log_lines = [f"Python: {python_exe}"]
        log_lines.append(f"Vendor dir: {vendor_dir}")

        # Never overwrite the bundled patched ur_rtde (required for PolyScope X;
        # the stock PyPI 1.6.3 wheel silently fails there). Only pip-install
        # ur_rtde when no bundled copy exists at all.
        packages = ["paramiko"]
        if self._bundled_ur_rtde_present():
            log_lines.append(
                "Bundled ur_rtde detected (patched PolyScope X build) — "
                "skipping pip ur_rtde so it is not overwritten."
            )
        else:
            log_lines.append("No bundled ur_rtde found — installing from PyPI "
                             "(note: stock ur_rtde does not support PolyScope X).")
            packages.insert(0, "ur_rtde")

        slot.ur_install_status = "RUNNING"
        slot.ur_install_log = f"Installing {' + '.join(packages)}..."

        pip_ok, pip_err = _bootstrap_pip(python_exe, log_lines)
        if not pip_ok:
            slot.ur_install_status = "ERROR"
            slot.ur_install_log = "\n".join(log_lines)[-4000:]
            self.report({"ERROR"}, pip_err[:240])
            return {"CANCELLED"}

        cmd = [
            python_exe,
            "-m",
            "pip",
            "install",
            "--upgrade",
            "--target",
            vendor_dir,
        ] + packages
        try:
            proc = _run_pip_command(cmd, timeout=300)
        except subprocess.TimeoutExpired:
            msg = f"Timeout running: {' '.join(cmd)}"
            log_lines.append(msg)
            slot.ur_install_status = "ERROR"
            slot.ur_install_log = "\n".join(log_lines)[-4000:]
            self.report({"ERROR"}, msg)
            return {"CANCELLED"}
        except Exception as exc:
            msg = f"Install failed: {exc}"
            log_lines.append(msg)
            slot.ur_install_status = "ERROR"
            slot.ur_install_log = "\n".join(log_lines)[-4000:]
            self.report({"ERROR"}, msg[:240])
            return {"CANCELLED"}

        log_lines.append(f"$ {' '.join(cmd)}")
        if proc.stdout:
            log_lines.append(proc.stdout.strip())
        if proc.stderr:
            log_lines.append(proc.stderr.strip())
        if proc.returncode != 0:
            msg = f"pip install failed (code {proc.returncode})"
            slot.ur_install_status = "ERROR"
            slot.ur_install_log = "\n".join(log_lines)[-4000:]
            self.report({"ERROR"}, msg)
            return {"CANCELLED"}

        added_paths, path_err = _refresh_current_session_python_paths(python_exe)
        if path_err:
            log_lines.append(path_err)
        elif added_paths:
            log_lines.append("Added Python paths to current session:")
            log_lines.extend(added_paths)

        # Verify import works
        try:
            for module_name in ("rtde_control", "rtde_receive", "rtde_io", "paramiko"):
                sys.modules.pop(module_name, None)
            importlib.invalidate_caches()
            importlib.import_module("rtde_control")
            importlib.import_module("rtde_receive")
            importlib.import_module("paramiko")
        except ImportError as exc:
            msg = f"Install finished but import failed: {exc}"
            log_lines.append(msg)
            slot.ur_install_status = "ERROR"
            slot.ur_install_log = "\n".join(log_lines)[-4000:]
            self.report({"ERROR"}, msg)
            return {"CANCELLED"}

        # Refresh UR driver-side availability cache in this Blender session.
        try:
            from animaquina_core.drivers import ur_driver as _ur_driver
            if not _ur_driver.ur_rtde_available():
                msg = "Install finished but UR driver still reports ur_rtde unavailable"
                log_lines.append(msg)
                slot.ur_install_status = "ERROR"
                slot.ur_install_log = "\n".join(log_lines)[-4000:]
                self.report({"ERROR"}, msg)
                return {"CANCELLED"}
            if not _ur_driver.ur_sftp_available():
                msg = "Install finished but UR driver still reports paramiko unavailable"
                log_lines.append(msg)
                slot.ur_install_status = "ERROR"
                slot.ur_install_log = "\n".join(log_lines)[-4000:]
                self.report({"ERROR"}, msg)
                return {"CANCELLED"}
        except Exception as exc:
            msg = f"Install finished but could not refresh UR backend state: {exc}"
            log_lines.append(msg)
            slot.ur_install_status = "ERROR"
            slot.ur_install_log = "\n".join(log_lines)[-4000:]
            self.report({"ERROR"}, msg[:240])
            return {"CANCELLED"}

        slot.ur_install_status = "OK"
        slot.ur_install_log = "\n".join(log_lines)[-4000:]
        self.report({"INFO"}, "UR dependencies install complete and verified")
        return {"FINISHED"}


class ANIMAQUINA_OT_NewtonKeyframeIKPath(Operator):
    bl_idname = "object.animaquina_newton_keyframe_ik_path"
    bl_label = "Keyframe Newton IK Path"
    bl_description = "Run Newton/MuJoCo IK validation and keyframe each solved waypoint onto the timeline"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None:
            return False
        obj = getattr(context, "active_object", None)
        if not obj or obj.type != "MESH":
            return False
        if not getattr(obj.data, "attributes", None) or "position" not in obj.data.attributes:
            return False
        return bool(getattr(slot, "rig_collection", None) or getattr(slot, "sim_collection", None))

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        points, err_msg = _get_path_points_from_object(context)
        if points is None:
            self.report({"ERROR"}, err_msg)
            return {"CANCELLED"}

        # Newton playback should always bake onto the simulation duplicate, never the main twin/rig.
        sim_err = simulation.setup_simulation(slot, enable_blender_ik=False)
        if sim_err:
            self.report({"ERROR"}, f"Simulation setup failed: {sim_err}")
            return {"CANCELLED"}

        armature_obj = _find_armature_in_collection_recursive(getattr(slot, "sim_collection", None))
        if armature_obj is None:
            self.report({"ERROR"}, "No armature found in Sim Collection")
            return {"CANCELLED"}

        # Build waypoints in armature/MuJoCo frame (same as ValidatePath).
        arm_inv = rig_apply.get_slot_armature_world_matrix(slot).inverted()
        target_obj_kf = getattr(slot, "target_object", None)
        if target_obj_kf is not None:
            euler_rad = tuple((arm_inv @ target_obj_kf.matrix_world).to_euler("XYZ"))
        else:
            euler_rad = tuple(slot.tcp_euler_rad)

        # Per-point rotation from mesh attribute (when enabled)
        _per_point_eulers_arm = None
        if bool(getattr(slot, "export_use_rotation_attribute", False)):
            obj = context.active_object
            if obj and obj.type == "MESH":
                depsgraph = context.evaluated_depsgraph_get()
                eval_obj = obj.evaluated_get(depsgraph)
                rotation_att = eval_obj.data.attributes.get("rotation")
                if rotation_att is not None:
                    world_matrix = obj.matrix_world
                    apply_rot_transform = bool(getattr(slot, "export_rotation_apply_transform", False))
                    _per_point_eulers_arm = []
                    for r in rotation_att.data:
                        local_euler_deg = r.vector
                        local_euler = (math.radians(local_euler_deg[0]), math.radians(local_euler_deg[1]), math.radians(local_euler_deg[2]))
                        if apply_rot_transform:
                            world_rot = world_matrix.to_3x3().to_4x4() @ Euler(local_euler, "XYZ").to_matrix().to_4x4()
                            arm_rot = arm_inv @ world_rot
                            _per_point_eulers_arm.append(tuple(arm_rot.to_euler("XYZ")))
                        else:
                            _per_point_eulers_arm.append(local_euler)

        waypoints = []
        for i, pos_m in enumerate(points):
            loc = (arm_inv @ Matrix.Translation((float(pos_m[0]), float(pos_m[1]), float(pos_m[2])))).to_translation()
            wp_euler = _per_point_eulers_arm[i] if _per_point_eulers_arm and i < len(_per_point_eulers_arm) else euler_rad
            waypoints.append(((loc.x, loc.y, loc.z), wp_euler))

        options = _build_newton_validation_options(context, slot)
        request = newton_validator.build_validation_request(slot, waypoints, euler_rad, options=options)

        self._aq_slot = slot
        self._aq_armature_obj = armature_obj
        self._aq_points = points
        self._aq_waypoints = waypoints
        self._aq_options = options
        self._aq_request = request
        self._aq_job_done = False
        self._aq_job_error = None
        self._aq_job_report = None
        slot.validation_last_status = "RUNNING"
        slot.validation_last_message = "Newton keyframe solve running..."

        def _job():
            try:
                self._aq_job_report = newton_validator.validate_prebuilt_request(request, use_persistent=True)
            except Exception as exc:
                self._aq_job_error = exc
            finally:
                self._aq_job_done = True

        self._aq_thread = threading.Thread(target=_job, daemon=True)
        self._aq_thread.start()
        wm = context.window_manager
        self._aq_timer = wm.event_timer_add(0.1, window=context.window)
        wm.modal_handler_add(self)
        return {"RUNNING_MODAL"}

    def modal(self, context, event):
        if event.type != "TIMER":
            return {"PASS_THROUGH"}
        if not getattr(self, "_aq_job_done", False):
            return {"PASS_THROUGH"}
        try:
            context.window_manager.event_timer_remove(self._aq_timer)
        except Exception:
            pass

        slot = getattr(self, "_aq_slot", None)
        armature_obj = getattr(self, "_aq_armature_obj", None)
        points = list(getattr(self, "_aq_points", []) or [])
        waypoints = list(getattr(self, "_aq_waypoints", []) or [])
        options = dict(getattr(self, "_aq_options", {}) or {})
        if slot is None or armature_obj is None:
            return {"CANCELLED"}

        if getattr(self, "_aq_job_error", None) is not None:
            exc = self._aq_job_error
            msg = f"Newton IK keyframe failed: {exc}"
            slot.validation_last_status = "ERROR"
            slot.validation_last_message = msg
            _write_newton_debug_text(context, slot, {
                "kind": "keyframe_ik_path_error",
                "robot_label": getattr(slot, "label", ""),
                "robot_type": getattr(slot, "robot_type", ""),
                "newton_python_exe": getattr(slot, "newton_python_exe", ""),
                "options": options,
                "error": str(exc),
                "path_points_count": len(points),
            })
            self.report({"ERROR"}, msg)
            return {"CANCELLED"}

        report = dict(getattr(self, "_aq_job_report", {}) or {})
        debug_block = report.get("debug") or {}
        solutions_deg = debug_block.get("ik_waypoint_solutions_deg") or []
        if not solutions_deg:
            _write_newton_debug_text(context, slot, {
                "kind": "keyframe_ik_path_no_solutions",
                "report": report,
                "path_points_count": len(points),
                "waypoints_count": len(waypoints),
            })
            self.report({"ERROR"}, "No IK waypoint solutions returned by Newton")
            return {"CANCELLED"}

        # Also keep validation status/debug panels updated after the async keyframe solve.
        _apply_newton_validation_report(
            context, slot, report,
            points_count=len(points),
            waypoints_count=len(waypoints),
            options=options,
            op=self,
        )

        start_frame = int(context.scene.frame_current)
        frame_step = int(getattr(slot, "newton_keyframe_step", 1) or 1)
        joint_axis_map = [getattr(slot, f"joint_axis_{i}", "Y") for i in range(6)]
        clear_stats = _clear_newton_ik_keyframes_on_armature(context.scene, armature_obj)
        removed_constraints = _remove_sim_ik_constraints(armature_obj)

        for idx, joints_deg in enumerate(solutions_deg):
            frame = start_frame + idx * frame_step
            rig_apply.apply_joint_angles(armature_obj, list(joints_deg), joint_axis_map)
            for j in range(min(6, len(joints_deg))):
                bone = armature_obj.pose.bones.get(f"joint_{j + 1}")
                if bone is None:
                    continue
                axis_index = _joint_axis_rotation_index(joint_axis_map[j])
                bone.keyframe_insert(data_path="rotation_euler", index=axis_index, frame=frame)

        first_failure = report.get("first_failure_index")
        if isinstance(first_failure, int) and first_failure >= 0:
            marker_name = f"NEWTON_{str(report.get('failure_kind') or 'FAIL').upper()}"
            marker_frame = start_frame + first_failure * frame_step
            try:
                marker = context.scene.timeline_markers.new(marker_name, frame=marker_frame)
                marker.select = True
            except Exception:
                pass

        _write_newton_debug_text(context, slot, {
            "kind": "keyframe_ik_path",
            "robot_label": getattr(slot, "label", ""),
            "robot_type": getattr(slot, "robot_type", ""),
            "target_armature": getattr(armature_obj, "name", ""),
            "bake_target_kind": "sim_armature",
            "start_frame": start_frame,
            "frame_step": frame_step,
            "path_points_count": len(points),
            "waypoints_count": len(waypoints),
            "solutions_count": len(solutions_deg),
            "clear_stats": clear_stats,
            "removed_sim_constraints": removed_constraints,
            "report": report,
        })
        status = "VALID" if bool(report.get("is_valid")) else "INVALID"
        msg = (
            f"Keyframed {len(solutions_deg)} waypoint IK poses on '{armature_obj.name}' "
            f"(start={start_frame}, step={frame_step}, {status}, cleared_fc={clear_stats['fcurves_removed']}, rm_constraints={removed_constraints})"
        )
        self.report({"INFO"} if status == "VALID" else {"WARNING"}, msg[:240])
        return {"FINISHED"}


class ANIMAQUINA_OT_NewtonClearIKKeyframes(Operator):
    bl_idname = "object.animaquina_newton_clear_ik_keyframes"
    bl_label = "Clear IK Keyframes"
    bl_description = "Clear Newton-baked IK joint keyframes and Newton timeline markers from the simulation armature"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None:
            return False
        return _get_newton_bake_target_armature(slot) is not None

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        armature_obj = _get_newton_bake_target_armature(slot)
        if armature_obj is None:
            self.report({"ERROR"}, "No armature found in Sim Collection or Rig")
            return {"CANCELLED"}
        stats = _clear_newton_ik_keyframes_on_armature(context.scene, armature_obj)
        _write_newton_debug_text(context, slot, {
            "kind": "clear_keyframe_ik_path",
            "robot_label": getattr(slot, "label", ""),
            "robot_type": getattr(slot, "robot_type", ""),
            "target_armature": getattr(armature_obj, "name", ""),
            "clear_stats": stats,
        })
        self.report(
            {"INFO"},
            f"Cleared Newton IK keys on '{armature_obj.name}' (fcurves={stats['fcurves_removed']}, markers={stats['markers_removed']})",
        )
        return {"FINISHED"}


class ANIMAQUINA_OT_ExportKRL(Operator):
    bl_idname = "object.animaquina_export_krl"
    bl_label = "Export KRL"
    bl_description = "Export KRL program from mesh position attribute to Blender text (KUKA)"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        return slot is not None and slot.robot_type == "KUKA"

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        from ..runtime import krl_export
        err = krl_export.export_krl(slot, context)
        if err:
            self.report({"ERROR"}, err)
            return {"CANCELLED"}
        prog_name = _normalize_kuka_program_name(
            getattr(slot, "program_name", ""),
            fallback=(getattr(context.active_object, "name", "animaquina") if context.active_object else "animaquina"),
        )
        slot.program_name = prog_name
        self.report({"INFO"}, f"KRL saved as {prog_name}.src in Text Editor")
        return {"FINISHED"}


class ANIMAQUINA_OT_SaveProgramToRobot(Operator):
    bl_idname = "object.animaquina_save_program"
    bl_label = "Save Program to Robot"
    bl_description = "Upload the exported KRL program to the KUKA controller via C3 Bridge"
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None or not slot.is_connected:
            return False
        driver = manager.get_driver_for_slot(slot)
        return driver and (driver.capabilities() & CAP_UPLOAD_PROGRAM)

    def execute(self, context):
        slot = get_active_slot(context)
        driver = manager.get_driver_for_slot(slot)
        if slot is None or driver is None:
            return {"CANCELLED"}

        prog_name = (slot.program_name or "").strip()
        if not prog_name:
            self.report({"ERROR"}, "Set a program name first")
            return {"CANCELLED"}

        src_name = prog_name if prog_name.endswith(".src") else f"{prog_name}.src"
        text_block = None
        obj = context.active_object
        if obj:
            text_block = bpy.data.texts.get(f"{obj.name}.src")
        if text_block is None:
            text_block = bpy.data.texts.get(src_name)
        if text_block is None:
            hint = f"{obj.name}.src or {src_name}" if obj else src_name
            self.report({"ERROR"}, f"No text block found ({hint}) Ã¢â‚¬â€ export KRL first")
            return {"CANCELLED"}

        content = text_block.as_string()
        if not content.strip():
            self.report({"ERROR"}, "KRL text block is empty")
            return {"CANCELLED"}

        remote_path = getattr(slot, "remote_path", "") or ""
        err = driver.upload_program(content, prog_name, remote_path)
        if err:
            self.report({"ERROR"}, err)
            slot.last_error = err
            return {"CANCELLED"}

        self.report({"INFO"}, f"Program '{prog_name}' uploaded to robot")
        return {"FINISHED"}


class ANIMAQUINA_OT_StageProgramOnRobot(Operator):
    bl_idname = "object.animaquina_stage_program"
    bl_label = "Stage to Robot"
    bl_description = "Upload and select the exported KRL program on the KUKA controller via C3 Bridge"
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None or not slot.is_connected or slot.robot_type != "KUKA":
            return False
        driver = manager.get_driver_for_slot(slot)
        if driver is None:
            return False
        caps = driver.capabilities()
        return bool((caps & CAP_UPLOAD_PROGRAM) and (caps & CAP_SELECT_PROGRAM))

    def execute(self, context):
        slot = get_active_slot(context)
        driver = manager.get_driver_for_slot(slot)
        if slot is None or driver is None:
            return {"CANCELLED"}

        obj = context.active_object
        prog_name = _normalize_kuka_program_name(
            getattr(slot, "program_name", ""),
            fallback=(getattr(obj, "name", "animaquina") if obj else "animaquina"),
        )
        slot.program_name = prog_name
        src_name = f"{prog_name}.src"

        candidates = [src_name]
        if obj is not None:
            obj_src = f"{obj.name}.src"
            if obj_src not in candidates:
                candidates.append(obj_src)
        text_block, resolved_name = _get_first_text_block(candidates)
        if text_block is None:
            self.report({"ERROR"}, f"No text block found ({', '.join(candidates)}) - export KRL first")
            return {"CANCELLED"}

        content = text_block.as_string()
        if not content.strip():
            self.report({"ERROR"}, "KRL text block is empty")
            return {"CANCELLED"}

        remote_path = getattr(slot, "remote_path", "") or ""
        err = driver.upload_program(content, prog_name, remote_path)
        if err:
            self.report({"ERROR"}, err)
            slot.last_error = err
            return {"CANCELLED"}

        err = driver.select_program(prog_name, remote_path)
        if err:
            self.report({"ERROR"}, err)
            slot.last_error = err
            return {"CANCELLED"}

        self.report({"INFO"}, f"Staged '{prog_name}' (uploaded from {resolved_name} and selected)")
        return {"FINISHED"}


class ANIMAQUINA_OT_LoadProgramOnRobot(Operator):
    bl_idname = "object.animaquina_load_program"
    bl_label = "Load Program on Robot"
    bl_description = "Select/load a program on the KUKA controller via C3 Bridge"
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None or not slot.is_connected:
            return False
        driver = manager.get_driver_for_slot(slot)
        return driver and (driver.capabilities() & CAP_SELECT_PROGRAM)

    def execute(self, context):
        slot = get_active_slot(context)
        driver = manager.get_driver_for_slot(slot)
        if slot is None or driver is None:
            return {"CANCELLED"}

        prog_name = (slot.program_name or "").strip()
        if not prog_name:
            self.report({"ERROR"}, "Set a program name first")
            return {"CANCELLED"}

        remote_path = getattr(slot, "remote_path", "") or ""
        err = driver.select_program(prog_name, remote_path)
        if err:
            self.report({"ERROR"}, err)
            slot.last_error = err
            return {"CANCELLED"}

        self.report({"INFO"}, f"Program '{prog_name}' selected on robot")
        return {"FINISHED"}


class ANIMAQUINA_OT_UploadStreamProgram(Operator):
    bl_idname = "object.animaquina_upload_stream"
    bl_label = "Upload Dynamic Sync"
    bl_description = "Generate and upload the Dynamic Sync program (mq_stream) to the KUKA controller"
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None or not slot.is_connected:
            return False
        if slot.robot_type != "KUKA":
            return False
        driver = manager.get_driver_for_slot(slot)
        return driver and (driver.capabilities() & CAP_UPLOAD_PROGRAM)

    def execute(self, context):
        slot = get_active_slot(context)
        driver = manager.get_driver_for_slot(slot)
        if slot is None or driver is None:
            return {"CANCELLED"}
        remote_path = getattr(slot, "remote_path", "") or ""
        base_no = getattr(slot, "export_base_no", 0)
        tool_no = getattr(slot, "export_tool_no", 0)

        from animaquina_core.runtime import krl_stream
        ring_size = getattr(slot, "kuka_ring_buffer_size", krl_stream.RING_BUFFER_SIZE_DEFAULT)
        advance = getattr(slot, "kuka_advance", krl_stream.ADVANCE_DEFAULT)
        src_text = krl_stream.generate_src(base_no=base_no, tool_no=tool_no, ring_buffer_size=ring_size,
                                           advance=advance)
        dat_text = krl_stream.generate_dat()
        config_text = krl_stream.generate_config_snippet(base_no=base_no, tool_no=tool_no, ring_buffer_size=ring_size)
        for name, content in [
            ("mq_stream.src", src_text),
            ("mq_stream.dat", dat_text),
            ("MQ_CONFIG_SNIPPET.txt", config_text),
        ]:
            if name in bpy.data.texts:
                bpy.data.texts[name].clear()
            else:
                bpy.data.texts.new(name)
            bpy.data.texts[name].write(content)

        err = driver.upload_stream_program(remote_path, base_no=base_no, tool_no=tool_no,
                                           ring_buffer_size=ring_size, advance=advance)
        if err:
            self.report({"ERROR"}, err)
            slot.last_error = err
            return {"CANCELLED"}
        slot.stream_program_uploaded = True
        slot.kuka_stream_uploaded_ring_size = int(ring_size)
        self.report({"WARNING"}, "Program uploaded. Add MQ_ vars to $CONFIG.DAT (see MQ_CONFIG_SNIPPET.txt in Text Editor)")
        return {"FINISHED"}


class ANIMAQUINA_OT_SelectStreamProgram(Operator):
    bl_idname = "object.animaquina_select_stream"
    bl_label = "Select Dynamic Sync"
    bl_description = "Select the Dynamic Sync program (mq_stream) on the KUKA controller"
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None or not slot.is_connected:
            return False
        if slot.robot_type != "KUKA":
            return False
        driver = manager.get_driver_for_slot(slot)
        return driver and (driver.capabilities() & CAP_SELECT_PROGRAM)

    def execute(self, context):
        slot = get_active_slot(context)
        driver = manager.get_driver_for_slot(slot)
        if slot is None or driver is None:
            return {"CANCELLED"}
        remote_path = getattr(slot, "remote_path", "") or ""
        err = driver.select_stream_program(remote_path)
        if err:
            self.report({"ERROR"}, err)
            slot.last_error = err
            return {"CANCELLED"}
        self.report({"INFO"}, "Dynamic Sync program selected Ã¢â‚¬â€ start it from the pendant")
        return {"FINISHED"}


class ANIMAQUINA_OT_SetUROrientation(Operator):
    bl_idname = "object.animaquina_set_ur_orientation"
    bl_label = "Set Orientation from Current"
    bl_description = "Save the current TCP orientation as the export custom orientation"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        return slot is not None and slot.robot_type in {"UR", "KUKA"}

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        euler_deg = slot.tcp_euler_deg
        if all(v == 0.0 for v in euler_deg):
            self.report({"WARNING"}, "TCP orientation is all zero Ã¢â‚¬â€ is the robot connected?")
        if slot.robot_type == "KUKA":
            # tcp_euler_deg is Blender XYZ; KUKA ABC = ZYX reversed
            slot.export_custom_a = euler_deg[2]
            slot.export_custom_b = euler_deg[1]
            slot.export_custom_c = euler_deg[0]
            a_val, b_val, c_val = euler_deg[2], euler_deg[1], euler_deg[0]
        else:
            # UR/xArm: export_custom_a/b/c = Blender XYZ directly
            slot.export_custom_a = euler_deg[0]
            slot.export_custom_b = euler_deg[1]
            slot.export_custom_c = euler_deg[2]
            a_val, b_val, c_val = euler_deg[0], euler_deg[1], euler_deg[2]
        self.report({"INFO"}, f"{slot.robot_type} export orientation set to A={a_val:.1f} B={b_val:.1f} C={c_val:.1f}")
        return {"FINISHED"}


class ANIMAQUINA_OT_SetHome(Operator):
    bl_idname = "object.animaquina_set_home"
    bl_label = "Set Home from Current"
    bl_description = "Save the current joint angles as the home position"
    bl_options = {"REGISTER", "UNDO"}

    _HOME_PROPS = {"UR": "ur_export_home", "KUKA": "kuka_export_home", "XARM": "xarm_export_home"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        return slot is not None

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        prop = self._HOME_PROPS.get(slot.robot_type)
        if prop is None:
            self.report({"WARNING"}, f"No home property for {slot.robot_type}")
            return {"CANCELLED"}
        joints = slot.joints_deg
        if all(j == 0.0 for j in joints):
            self.report({"WARNING"}, "Joint angles are all zero Ã¢â‚¬â€ is the robot connected?")
        for i in range(6):
            getattr(slot, prop)[i] = joints[i]
        self.report({"INFO"}, f"Home set to [{', '.join(f'{j:.1f}' for j in joints)}]")
        return {"FINISHED"}


def _slot_model_name(slot) -> str:
    """Model enum for the slot's brand (ur_model / kuka_model / xarm_model)."""
    brand = str(getattr(slot, "robot_type", "") or "").lower()
    return str(getattr(slot, f"{brand}_model", "") or "")


def _record_meta(context, slot) -> dict:
    """Provenance for the dataset header. Everything here is cheap to capture
    now and impossible to reconstruct from the numbers later."""
    version = ".".join(str(v) for v in getattr(sys.modules.get("animaquina"), "bl_info", {}).get("version", ()))
    blend_path = bpy.data.filepath or "(unsaved)"
    props = getattr(context.scene, "animaquina", None)
    meta = {
        "robot_label": getattr(slot, "label", ""),
        "robot_type": getattr(slot, "robot_type", ""),
        "robot_model": _slot_model_name(slot),
        "poll_rate_hz": f"{float(getattr(props, 'poll_rate_hz', 0.0)):.3f}" if props else "",
        "joint_order": "j0..j5 = robot axes 1..6",
        "target_object": getattr(getattr(slot, "target_object", None), "name", "") or "(none)",
        "addon_version": version or "unknown",
        "blend_file": blend_path,
    }
    if getattr(slot, "base_frame_valid", False):
        meta["base_frame_m_deg"] = "%.4f,%.4f,%.4f,%.3f,%.3f,%.3f" % (
            tuple(slot.base_pos_m) + tuple(slot.base_euler_deg)
        )
    if getattr(slot, "tool_frame_valid", False):
        meta["tool_frame_m_deg"] = "%.4f,%.4f,%.4f,%.3f,%.3f,%.3f" % (
            tuple(slot.tool_frame_pos_m) + tuple(slot.tool_frame_euler_deg)
        )
    return meta


def _recorded_var_names(slot) -> list:
    """Enabled Variables-panel entries, in panel order.

    Same filter the poll timer uses to decide what to actually read, so a
    disabled entry never becomes an all-empty column.
    """
    names = []
    for item in getattr(slot, "debug_vars", []):
        if not bool(getattr(item, "enabled", True)):
            continue
        var_name = str(getattr(item, "var_name", "") or "").strip()
        if var_name and var_name not in names:
            names.append(var_name)
    return names


def _safe_dataset_name(raw: str) -> str:
    cleaned = "".join(c if (c.isalnum() or c in "-_") else "_" for c in str(raw or "").strip())
    return cleaned or "dataset"


def _write_dataset_text(session) -> str:
    """Write the session to a NEW Text datablock and return its name.

    Never reuses an existing block: a re-record must not silently destroy the
    take before it. bpy.data.texts.new() appends .001, .002 ... on collision.
    """
    name = f"AQ_DS_{_safe_dataset_name(session.name)}"
    text_block = bpy.data.texts.new(name)
    text_block.write(recorder.to_csv(session))
    return text_block.name


class ANIMAQUINA_OT_RecordStart(Operator):
    bl_idname = "object.animaquina_record_start"
    bl_label = "Record Dataset"
    bl_description = ("Start recording timestamped joint, TCP and commanded-target samples "
                      "at the poll rate, for building training datasets")
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        return slot is not None and slot.is_connected

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        if not slot.polling_enabled:
            self.report({"ERROR"}, "Enable Realtime polling first - samples come from the poll loop")
            return {"CANCELLED"}

        existing = recorder.get(slot.uid)
        if existing is not None and existing.active:
            self.report({"WARNING"}, "Already recording")
            return {"CANCELLED"}
        if existing is not None and existing.count() and not slot.record_text_name:
            self.report({"WARNING"}, "Previous recording was never written out - it will be discarded")

        name = _safe_dataset_name(slot.record_name)
        var_names = _recorded_var_names(slot)
        recorder.start(slot.uid, name, _record_meta(context, slot),
                       int(slot.record_max_samples), var_names)
        slot.record_active = True
        slot.record_sample_count = 0
        slot.record_elapsed_s = 0.0
        slot.record_text_name = ""

        extra = f" (+{len(var_names)} variables)" if var_names else ""
        if getattr(slot, "target_object", None) is None:
            self.report({"WARNING"}, f"Recording '{name}'{extra} - no Target object, cmd_* columns will be empty")
        else:
            self.report({"INFO"}, f"Recording dataset '{name}'{extra}")
        return {"FINISHED"}


class ANIMAQUINA_OT_RecordStop(Operator):
    bl_idname = "object.animaquina_record_stop"
    bl_label = "Stop Recording"
    bl_description = "Stop recording and write the dataset to a Text datablock"
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        return slot is not None and recorder.is_active(slot.uid)

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        session = recorder.stop(slot.uid)
        if session is None:
            return {"CANCELLED"}

        slot.record_active = False
        slot.record_sample_count = session.count()
        slot.record_elapsed_s = session.elapsed_s()

        if not session.count():
            self.report({"WARNING"}, "Stopped - no samples captured")
            return {"FINISHED"}

        try:
            slot.record_text_name = _write_dataset_text(session)
        except Exception as exc:
            self.report({"ERROR"}, f"Could not write Text datablock: {exc}"[:240])
            return {"CANCELLED"}

        note = " (hit sample cap)" if session.overflow else ""
        self.report({"INFO"},
                    f"{session.count()} samples in {session.elapsed_s():.1f}s -> {slot.record_text_name}{note}")
        return {"FINISHED"}


class ANIMAQUINA_OT_RecordSaveCSV(Operator):
    bl_idname = "object.animaquina_record_save_csv"
    bl_label = "Save Dataset .csv"
    bl_description = "Write the last recorded dataset to a .csv file on disk"
    bl_options = {"REGISTER"}

    filepath: StringProperty(subtype="FILE_PATH")
    filter_glob: StringProperty(default="*.csv", options={"HIDDEN"})

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None:
            return False
        session = recorder.get(slot.uid)
        return session is not None and session.count() > 0

    def invoke(self, context, _event):
        slot = get_active_slot(context)
        name = _safe_dataset_name(getattr(slot, "record_name", "")) if slot else "dataset"
        base = os.path.dirname(bpy.data.filepath) or os.path.expanduser("~")
        self.filepath = os.path.join(base, f"{name}.csv")
        context.window_manager.fileselect_add(self)
        return {"RUNNING_MODAL"}

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        session = recorder.get(slot.uid)
        if session is None or not session.count():
            self.report({"ERROR"}, "No recorded dataset to save")
            return {"CANCELLED"}

        path = bpy.path.abspath(self.filepath or "")
        if not path:
            self.report({"ERROR"}, "No file path given")
            return {"CANCELLED"}
        if not path.lower().endswith(".csv"):
            path += ".csv"

        try:
            with open(path, "w", encoding="utf-8", newline="") as fh:
                fh.write(recorder.to_csv(session))
        except OSError as exc:
            self.report({"ERROR"}, f"Could not write {path}: {exc}"[:240])
            return {"CANCELLED"}

        self.report({"INFO"}, f"Saved {session.count()} samples to {path}")
        return {"FINISHED"}


class ANIMAQUINA_OT_DebugAddVar(Operator):
    bl_idname = "object.animaquina_debug_add_var"
    bl_label = "Add Debug Variable"
    bl_description = "Add a controller variable to realtime debug polling"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        return slot is not None

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        name = str(getattr(slot, "debug_new_var", "") or "").strip()
        if not name:
            self.report({"WARNING"}, "Enter a variable name first")
            return {"CANCELLED"}

        for item in slot.debug_vars:
            if str(getattr(item, "var_name", "") or "").strip().upper() == name.upper():
                self.report({"INFO"}, f"Variable '{name}' is already in the list")
                return {"FINISHED"}

        entry = slot.debug_vars.add()
        entry.var_name = name
        entry.value = ""
        entry.error = ""
        entry.enabled = True
        slot.debug_new_var = ""
        self.report({"INFO"}, f"Added debug variable '{name}'")
        return {"FINISHED"}


# Legacy alias
ANIMAQUINA_OT_KukaDebugAddVar = ANIMAQUINA_OT_DebugAddVar


class ANIMAQUINA_OT_DebugRemoveVar(Operator):
    bl_idname = "object.animaquina_debug_remove_var"
    bl_label = "Remove Debug Variable"
    bl_description = "Remove a variable from realtime debug polling"
    bl_options = {"REGISTER", "UNDO"}

    index: IntProperty(name="Index", default=-1)

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        return slot is not None

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        idx = int(self.index)
        if idx < 0 or idx >= len(slot.debug_vars):
            return {"CANCELLED"}
        slot.debug_vars.remove(idx)
        return {"FINISHED"}


# Legacy alias
ANIMAQUINA_OT_KukaDebugRemoveVar = ANIMAQUINA_OT_DebugRemoveVar


class ANIMAQUINA_OT_ClearRunState(Operator):
    bl_idname = "object.animaquina_clear_run_state"
    bl_label = "Clear Run State"
    bl_description = (
        "Reset the slot's run_idx contract (run source, index, count, object). "
        "Manual escape hatch for a stuck PROGRAM/SIM run state; a live stream "
        "will keep updating run_idx until it finishes"
    )
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        return slot is not None and getattr(slot, "run_source", "NONE") != "NONE"

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        run_state.clear_run(slot)
        return {"FINISHED"}


class ANIMAQUINA_OT_StoreHomeToRobot(Operator):
    bl_idname = "object.animaquina_store_home_to_robot"
    bl_label = "Store Home to Robot"
    bl_description = "Write the robot's current joints into the controller home system variable"
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None or not slot.is_connected or slot.robot_type != "KUKA":
            return False
        driver = manager.get_driver_for_slot(slot)
        return driver is not None and hasattr(driver, "write_home_joints")

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self, width=480)

    def draw(self, context):
        layout = self.layout
        col = layout.column(align=True)
        col.alert = True
        col.label(text="This will overwrite the robot variable XHOME.", icon="ERROR")
        col.label(text="The current robot joint position will become the new Home.")
        col.label(text="Only continue if this is intended.")

    def execute(self, context):
        slot = get_active_slot(context)
        driver = manager.get_driver_for_slot(slot) if slot is not None else None
        if slot is None or driver is None:
            return {"CANCELLED"}
        try:
            joints = tuple(float(j) for j in driver.read_joints())
        except Exception as e:
            self.report({"ERROR"}, f"Could not read current joints: {e}")
            return {"CANCELLED"}

        err = driver.write_home_joints(joints)
        if err:
            self.report({"ERROR"}, err)
            slot.last_error = err
            return {"CANCELLED"}

        for i in range(min(6, len(joints))):
            slot.kuka_export_home[i] = joints[i]
        self.report({"INFO"}, f"Robot home (XHOME) stored to [{', '.join(f'{j:.1f}' for j in joints[:6])}]")
        return {"FINISHED"}


class ANIMAQUINA_OT_ExportUR(Operator):
    bl_idname = "object.animaquina_export_ur"
    bl_label = "Export URScript"
    bl_description = "Export URScript and URP wrapper from mesh position attribute to Blender texts (UR)"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        return slot is not None and slot.robot_type == "UR"

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        from ..runtime import ur_export
        err = ur_export.export_ur(slot, context)
        if err:
            self.report({"ERROR"}, err)
            return {"CANCELLED"}
        base_name = str(getattr(slot, "ur_program_name", "") or "").strip()
        if not base_name:
            base_name = getattr(context.active_object, "name", "object") if context.active_object else "object"
        self.report({"INFO"}, f"UR export saved as {base_name}.script and {base_name}.urp")
        return {"FINISHED"}


class ANIMAQUINA_OT_URSaveProgramToRobot(Operator):
    bl_idname = "object.animaquina_ur_save_program"
    bl_label = "Stage UR Program on Robot"
    bl_description = "Stage exported URScript on robot (SFTP full-program or legacy runtime transport)"
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None or not slot.is_connected or slot.robot_type != "UR":
            return False
        return manager.get_driver_for_slot(slot) is not None

    def execute(self, context):
        slot = get_active_slot(context)
        driver = manager.get_driver_for_slot(slot) if slot is not None else None
        if slot is None or driver is None:
            return {"CANCELLED"}

        obj = context.active_object
        obj_script_name = f"{obj.name}.script" if obj else ""
        prog_name = str(getattr(slot, "ur_program_name", "") or "").strip()
        if prog_name.lower().endswith(".script"):
            prog_name = prog_name[:-7]
        if prog_name.lower().endswith(".urp"):
            prog_name = prog_name[:-4]
        if not prog_name:
            prog_name = str(getattr(obj, "name", "") or "animaquina")
        prog_name = "".join(ch if ch.isalnum() or ch in {"_", "-", "."} else "_" for ch in prog_name)
        if not prog_name:
            prog_name = "animaquina"
        slot.ur_program_name = prog_name

        script_candidates = []
        if prog_name:
            script_candidates.append(f"{prog_name}.script")
        if obj_script_name and obj_script_name not in script_candidates:
            script_candidates.append(obj_script_name)
        text_block, resolved_script_name = _get_first_text_block(script_candidates)
        if text_block is None:
            hint = ", ".join(script_candidates) if script_candidates else "<object>.script"
            msg = f"No text block found ({hint}) - export URScript first"
            self.report({"ERROR"}, msg)
            slot.ur_transfer_status = "ERROR"
            slot.ur_transfer_log = msg
            return {"CANCELLED"}

        content = text_block.as_string()
        if not content.strip():
            msg = "URScript text block is empty"
            self.report({"ERROR"}, msg)
            slot.ur_transfer_status = "ERROR"
            slot.ur_transfer_log = msg
            return {"CANCELLED"}

        stage_transport = str(getattr(slot, "ur_stage_transport", "sftp") or "sftp").strip().lower()
        stage_remote_path = str(getattr(slot, "ur_remote_path", "/programs") or "/programs")
        stage_user = str(getattr(slot, "ur_sftp_user", "root") or "root")
        stage_password = str(getattr(slot, "ur_sftp_password", "") or "")
        stage_port = int(getattr(slot, "ur_sftp_port", 22))
        obj_urp_name = f"{obj.name}.urp" if obj else ""
        urp_candidates = []
        if prog_name:
            urp_candidates.append(f"{prog_name}.urp")
        if obj_urp_name and obj_urp_name not in urp_candidates:
            urp_candidates.append(obj_urp_name)
        urp_text_block, _ = _get_first_text_block(urp_candidates)
        urp_content = urp_text_block.as_string() if urp_text_block is not None else ""
        if stage_transport in {"sftp", "full_program_sftp"} and not urp_content.strip():
            # Build wrapper on the fly if no .urp text block was exported yet.
            from ..runtime import ur_export
            urp_content = ur_export.build_urp_wrapper(
                program_name=prog_name.replace(" ", "_").replace(".", "_"),
                script_filename=f"{prog_name}.script",
                script_content=content,
            )

        wm = context.window_manager
        if getattr(context, "window", None) is None:
            self.report({"ERROR"}, "Cannot start non-blocking stage without an active window")
            return {"CANCELLED"}

        self._aq_stage_slot = slot
        self._aq_stage_driver = driver
        self._aq_stage_prog_name = prog_name
        self._aq_stage_transport = stage_transport
        self._aq_stage_done = False
        self._aq_stage_error = None
        self._aq_stage_uploaded_urp = False

        slot.ur_transfer_status = "RUNNING"
        slot.ur_transfer_log = f"Staging {prog_name}.script/.urp via {stage_transport} ..."

        def _job():
            if hasattr(driver, "stage_program"):
                self._aq_stage_error = driver.stage_program(
                    content,
                    prog_name,
                    transport=stage_transport,
                    remote_path=stage_remote_path,
                    username=stage_user,
                    password=stage_password,
                    port=stage_port,
                )
            else:
                self._aq_stage_error = driver.send_program(content)
            if (
                not self._aq_stage_error
                and stage_transport in {"sftp", "full_program_sftp"}
                and hasattr(driver, "upload_urp_sftp")
                and urp_content.strip()
            ):
                self._aq_stage_error = driver.upload_urp_sftp(
                    urp_content,
                    prog_name,
                    remote_path=stage_remote_path,
                    username=stage_user,
                    password=stage_password,
                    port=stage_port,
                )
                if not self._aq_stage_error:
                    self._aq_stage_uploaded_urp = True
            self._aq_stage_done = True

        self._aq_stage_thread = threading.Thread(target=_job, daemon=True)
        self._aq_stage_thread.start()

        self._aq_stage_timer = wm.event_timer_add(0.1, window=context.window)
        wm.modal_handler_add(self)
        self.report({"INFO"}, f"Staging started: {resolved_script_name or f'{prog_name}.script'}")
        return {"RUNNING_MODAL"}

    def modal(self, context, event):
        if event.type != "TIMER":
            return {"PASS_THROUGH"}
        if not getattr(self, "_aq_stage_done", False):
            return {"PASS_THROUGH"}

        try:
            context.window_manager.event_timer_remove(self._aq_stage_timer)
        except Exception:
            pass

        slot = getattr(self, "_aq_stage_slot", None)
        driver = getattr(self, "_aq_stage_driver", None)
        prog_name = str(getattr(self, "_aq_stage_prog_name", "") or "animaquina")
        stage_transport = str(getattr(self, "_aq_stage_transport", "") or "")
        err = getattr(self, "_aq_stage_error", None)
        uploaded_urp = bool(getattr(self, "_aq_stage_uploaded_urp", False))
        if slot is None or driver is None:
            return {"CANCELLED"}

        if err:
            self.report({"ERROR"}, str(err)[:240])
            slot.last_error = str(err)
            slot.ur_transfer_status = "ERROR"
            slot.ur_transfer_log = str(err)[:2000]
            return {"CANCELLED"}

        transport = str(getattr(driver, "last_program_transport", "") or "").strip()
        remote_file = str(getattr(driver, "last_staged_remote_file", "") or "").strip()
        remote_urp = str(getattr(driver, "last_staged_remote_urp_file", "") or "").strip()
        if uploaded_urp:
            slot.ur_launcher_urp = f"{prog_name}.urp"
        if uploaded_urp and transport and remote_file and remote_urp:
            ok_msg = f"Staged '{prog_name}.script' and '{prog_name}.urp' via {transport}"
        elif transport and remote_file:
            ok_msg = f"Staged '{prog_name}.script' via {transport} -> {remote_file}"
        elif transport:
            ok_msg = f"Staged '{prog_name}.script' on robot via {transport}"
        else:
            ok_msg = f"Staged '{prog_name}.script' on robot via {stage_transport}"
        if uploaded_urp and remote_urp:
            ok_msg += f" (URP: {remote_urp})"
        slot.ur_transfer_status = "OK"
        slot.ur_transfer_log = ok_msg
        self.report({"INFO"}, ok_msg[:240])
        return {"FINISHED"}


class ANIMAQUINA_OT_KukaPlayProgram(Operator):
    bl_idname = "object.animaquina_kuka_play_program"
    bl_label = "Run KUKA Program"
    bl_description = "Start the selected program on the KUKA controller via C3 Bridge"
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None or not slot.is_connected or slot.robot_type != "KUKA":
            return False
        driver = manager.get_driver_for_slot(slot)
        return driver is not None and hasattr(driver, "play_program")

    def execute(self, context):
        slot = get_active_slot(context)
        driver = manager.get_driver_for_slot(slot) if slot is not None else None
        if slot is None or driver is None:
            return {"CANCELLED"}

        err = driver.play_program()
        if err:
            self.report({"ERROR"}, err[:240])
            slot.last_error = err
            return {"CANCELLED"}

        self.report({"INFO"}, "C3 Bridge: program started")
        return {"FINISHED"}


class ANIMAQUINA_OT_KukaStopProgram(Operator):
    bl_idname = "object.animaquina_kuka_stop_program"
    bl_label = "Stop KUKA Program"
    bl_description = "Stop the running program on the KUKA controller via C3 Bridge (non-blocking)"
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None or not slot.is_connected or slot.robot_type != "KUKA":
            return False
        driver = manager.get_driver_for_slot(slot)
        return driver is not None and hasattr(driver, "stop_program")

    def execute(self, context):
        slot = get_active_slot(context)
        driver = manager.get_driver_for_slot(slot) if slot is not None else None
        if slot is None or driver is None:
            return {"CANCELLED"}

        # Set abort flag FIRST so running modals exit cleanly.
        slot.motion_abort_requested = True

        # Keep Blender-side puppet state in sync if ProgramControl stops mq_stream.
        if bool(getattr(slot, "realtime_puppet_active", False)):
            slot.realtime_puppet_active = False
            slot.realtime_puppet_status = "Puppet Mode: stopped"

        def _stop_job():
            try:
                if hasattr(driver, "realtime_puppet_stop"):
                    driver.realtime_puppet_stop()
                driver.stop_program()
            except Exception:
                pass

        threading.Thread(target=_stop_job, daemon=True).start()
        slot.motion_active_label = ""
        self.report({"INFO"}, "KUKA stop requested")
        return {"FINISHED"}


class ANIMAQUINA_OT_KukaCancelProgram(Operator):
    bl_idname = "object.animaquina_kuka_cancel_program"
    bl_label = "Cancel KUKA Program"
    bl_description = "Cancel the running program on the KUKA controller via C3 Bridge (non-blocking)"
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None or not slot.is_connected or slot.robot_type != "KUKA":
            return False
        driver = manager.get_driver_for_slot(slot)
        return driver is not None and hasattr(driver, "cancel_program")

    def execute(self, context):
        slot = get_active_slot(context)
        driver = manager.get_driver_for_slot(slot) if slot is not None else None
        if slot is None or driver is None:
            return {"CANCELLED"}

        # Set abort flag FIRST so running modals exit cleanly.
        slot.motion_abort_requested = True

        # Clean up puppet mode state if active
        if bool(getattr(slot, "realtime_puppet_active", False)):
            slot.realtime_puppet_active = False
            slot.realtime_puppet_status = "Puppet Mode: cancelled"

        def _cancel_job():
            try:
                if hasattr(driver, "realtime_puppet_stop"):
                    driver.realtime_puppet_stop()
                driver.cancel_program()
            except Exception:
                pass

        threading.Thread(target=_cancel_job, daemon=True).start()
        slot.motion_active_label = ""
        self.report({"INFO"}, "KUKA cancel requested")
        return {"FINISHED"}


class ANIMAQUINA_OT_KukaStreamExportedProgram(Operator):
    """Stream a KRL .src text to KUKA Dynamic Sync (Blender wrapper)."""

    bl_idname = "object.animaquina_kuka_stream_program"
    bl_label = "Stream to Robot"
    bl_description = (
        "Parse the selected .src text and stream it via Dynamic Sync ring buffer"
    )
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None or not slot.is_connected or slot.robot_type != "KUKA":
            return False
        if getattr(slot, "motion_active_label", ""):
            return False
        if bool(getattr(slot, "realtime_puppet_active", False)):
            cls.poll_message_set("Stop Puppet Mode before streaming a toolpath")
            return False
        driver = manager.get_driver_for_slot(slot)
        return driver is not None and hasattr(driver, "stream_start")

    def execute(self, context):
        slot = get_active_slot(context)
        driver = manager.get_driver_for_slot(slot) if slot else None
        if slot is None or driver is None:
            return {"CANCELLED"}

        # Resolve source text: active Text Editor .src first, then program-name candidates.
        text_block = None
        st = getattr(context, "space_data", None)
        if st and getattr(st, "type", "") == "TEXT_EDITOR" and getattr(st, "text", None):
            active_text = st.text
            if str(getattr(active_text, "name", "")).lower().endswith(".src"):
                text_block = active_text

        obj = context.active_object
        prog_name = _normalize_kuka_program_name(
            getattr(slot, "program_name", ""),
            fallback=(getattr(obj, "name", "animaquina") if obj else "animaquina"),
        )
        slot.program_name = prog_name

        if text_block is None:
            candidates = [f"{prog_name}.src"]
            if obj is not None:
                obj_src = f"{obj.name}.src"
                if obj_src not in candidates:
                    candidates.append(obj_src)
            text_block, _ = _get_first_text_block(candidates)

        if text_block is None:
            self.report({"ERROR"}, "No .src text block found - open one in Text Editor or export KRL first")
            return {"CANCELLED"}

        src_text = text_block.as_string()
        if not src_text.strip():
            self.report({"ERROR"}, "KRL text block is empty")
            return {"CANCELLED"}

        wm = context.window_manager
        if getattr(context, "window", None) is None:
            self.report({"ERROR"}, "Cannot start non-blocking stream without an active window")
            return {"CANCELLED"}

        # Snapshot settings before thread starts (avoid reading Blender props in worker thread).
        remote_path = str(getattr(slot, "remote_path", "") or "")
        base_no = int(getattr(slot, "export_base_no", 0))
        tool_no = int(getattr(slot, "export_tool_no", 0))
        default_lin_speed = float(getattr(slot, "export_lin_speed", 0.05))
        default_advance = float(getattr(slot, "export_advance", 3.0))
        ring_size = int(max(4, min(128, int(getattr(slot, "kuka_ring_buffer_size", 6) or 6))))
        # $ADVANCE planner lookahead (distinct from default_advance/APO above).
        kuka_advance = int(getattr(slot, "kuka_advance", 3) or 3)
        uploaded_ring_size = int(max(0, int(getattr(slot, "kuka_stream_uploaded_ring_size", 0) or 0)))
        stream_program_uploaded = bool(getattr(slot, "stream_program_uploaded", False))

        self._aq_slot = slot
        self._aq_driver = driver
        self._aq_done = False
        self._aq_error = None
        self._aq_uploaded_ring_size = 0

        try:
            from animaquina_core.runtime.kuka_stream_runtime import stream_krl_text
        except Exception as exc:
            msg = f"Failed to load KUKA stream runtime: {exc}"
            slot.last_error = msg
            self.report({"ERROR"}, msg[:240])
            return {"CANCELLED"}

        def _job():
            try:
                def _mark_uploaded(size):
                    try:
                        self._aq_uploaded_ring_size = int(size)
                    except Exception:
                        self._aq_uploaded_ring_size = 0

                self._aq_error = stream_krl_text(
                    driver,
                    src_text,
                    default_lin_speed=default_lin_speed,
                    default_advance=default_advance,
                    ring_size=ring_size,
                    uploaded_ring_size=uploaded_ring_size,
                    stream_program_uploaded=stream_program_uploaded,
                    remote_path=remote_path,
                    base_no=base_no,
                    tool_no=tool_no,
                    advance=kuka_advance,
                    mark_stream_uploaded=_mark_uploaded,
                )
            except Exception as exc:
                self._aq_error = str(exc)
            finally:
                self._aq_done = True

        # Mark motion active so Puppet Mode (and other toolpath ops) stay disabled
        # while this stream runs. Cleared in modal() when the stream finishes.
        slot.motion_active_label = "Streaming"

        self._aq_thread = threading.Thread(target=_job, daemon=True)
        self._aq_thread.start()

        self._aq_timer = wm.event_timer_add(0.05, window=context.window)
        wm.modal_handler_add(self)
        self.report({"INFO"}, f"Streaming from '{text_block.name}' started")
        return {"RUNNING_MODAL"}

    def modal(self, context, event):
        if event.type != "TIMER":
            return {"PASS_THROUGH"}
        if not getattr(self, "_aq_done", False):
            return {"PASS_THROUGH"}

        try:
            context.window_manager.event_timer_remove(self._aq_timer)
        except Exception:
            pass

        slot = getattr(self, "_aq_slot", None)
        err = getattr(self, "_aq_error", None)
        uploaded_size = int(getattr(self, "_aq_uploaded_ring_size", 0) or 0)

        if slot is not None:
            slot.motion_active_label = ""

        if slot is not None and uploaded_size > 0:
            try:
                slot.stream_program_uploaded = True
                slot.kuka_stream_uploaded_ring_size = uploaded_size
            except Exception:
                pass

        if err:
            if slot is not None:
                slot.last_error = str(err)
            self.report({"ERROR"}, str(err)[:240])
            return {"CANCELLED"}

        if slot is not None:
            manager.request_aux_refresh(slot)
            rig_apply.apply_full_pose(slot)
        self.report({"INFO"}, "Stream program completed")
        return {"FINISHED"}


class ANIMAQUINA_OT_URLoadProgramOnRobot(Operator):
    bl_idname = "object.animaquina_ur_load_program"
    bl_label = "Run UR Program"
    bl_description = "Load configured launcher URP and send Dashboard run command"
    bl_options = {"REGISTER"}
    force_reload: BoolProperty(
        name="Force Reload",
        default=False,
        options={"SKIP_SAVE"},
        description="Stop current program and force launcher reload before play",
    )

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None or not slot.is_connected or slot.robot_type != "UR":
            return False
        driver = manager.get_driver_for_slot(slot)
        return driver is not None and hasattr(driver, "play_program")

    def execute(self, context):
        slot = get_active_slot(context)
        driver = manager.get_driver_for_slot(slot) if slot is not None else None
        if slot is None or driver is None:
            return {"CANCELLED"}

        launcher_urp = str(getattr(slot, "ur_launcher_urp", "") or "").strip()
        if launcher_urp and not launcher_urp.lower().endswith(".urp"):
            launcher_urp = f"{launcher_urp}.urp"
            slot.ur_launcher_urp = launcher_urp
        remote_path = str(getattr(slot, "ur_remote_path", "") or "").strip()
        dashboard_port = int(getattr(slot, "ur_dashboard_port", 29999))
        if not launcher_urp:
            msg = "LAUNCHER_URP_NOT_CONFIGURED: set Launcher URP before Run Program"
            self.report({"ERROR"}, msg[:240])
            slot.last_error = msg
            slot.ur_transfer_status = "ERROR"
            slot.ur_transfer_log = msg[:2000]
            return {"CANCELLED"}

        wm = context.window_manager
        if getattr(context, "window", None) is None:
            self.report({"ERROR"}, "Cannot start non-blocking run without an active window")
            return {"CANCELLED"}

        self._aq_play_slot = slot
        self._aq_play_driver = driver
        self._aq_play_launcher_urp = launcher_urp
        self._aq_play_remote_path = remote_path
        self._aq_play_force_reload = bool(getattr(self, "force_reload", False))
        self._aq_play_done = False
        self._aq_play_error = None

        slot.ur_transfer_status = "RUNNING"
        if self._aq_play_force_reload and remote_path:
            slot.ur_transfer_log = (
                f"Sending Dashboard reload+run (stop/reload '{launcher_urp}' from '{remote_path}') ..."
            )
        elif self._aq_play_force_reload:
            slot.ur_transfer_log = (
                f"Sending Dashboard reload+run (stop/reload launcher '{launcher_urp}') ..."
            )
        elif remote_path:
            slot.ur_transfer_log = (
                f"Sending Dashboard run (resume if paused, otherwise load '{launcher_urp}' from '{remote_path}') ..."
            )
        else:
            slot.ur_transfer_log = (
                f"Sending Dashboard run (resume if paused, otherwise load launcher '{launcher_urp}') ..."
            )

        def _job():
            self._aq_play_error = driver.play_program(
                dashboard_port,
                launcher_urp=launcher_urp,
                remote_path=remote_path,
                force_reload=bool(getattr(self, "_aq_play_force_reload", False)),
            )
            self._aq_play_done = True

        self._aq_play_thread = threading.Thread(target=_job, daemon=True)
        self._aq_play_thread.start()

        self._aq_play_timer = wm.event_timer_add(0.1, window=context.window)
        wm.modal_handler_add(self)
        if self._aq_play_force_reload:
            self.report({"INFO"}, f"Reload+Run started with launcher: {launcher_urp}")
        else:
            self.report({"INFO"}, f"Run started with launcher: {launcher_urp}")
        return {"RUNNING_MODAL"}

    def modal(self, context, event):
        if event.type != "TIMER":
            return {"PASS_THROUGH"}
        if not getattr(self, "_aq_play_done", False):
            return {"PASS_THROUGH"}

        try:
            context.window_manager.event_timer_remove(self._aq_play_timer)
        except Exception:
            pass

        slot = getattr(self, "_aq_play_slot", None)
        driver = getattr(self, "_aq_play_driver", None)
        launcher_urp = str(getattr(self, "_aq_play_launcher_urp", "") or "")
        remote_path = str(getattr(self, "_aq_play_remote_path", "") or "")
        force_reload = bool(getattr(self, "_aq_play_force_reload", False))
        err = getattr(self, "_aq_play_error", None)
        if slot is None or driver is None:
            return {"CANCELLED"}

        if err:
            self.report({"ERROR"}, str(err)[:240])
            slot.last_error = str(err)
            slot.ur_transfer_status = "ERROR"
            slot.ur_transfer_log = str(err)[:2000]
            return {"CANCELLED"}

        if force_reload and remote_path:
            ok_msg = f"Dashboard reload+run command sent (launcher={launcher_urp}, path={remote_path})"
        elif force_reload:
            ok_msg = f"Dashboard reload+run command sent (launcher={launcher_urp})"
        elif remote_path:
            ok_msg = f"Dashboard run command sent (launcher={launcher_urp}, path={remote_path})"
        else:
            ok_msg = f"Dashboard run command sent (launcher={launcher_urp})"
        slot.ur_transfer_status = "OK"
        slot.ur_transfer_log = ok_msg
        self.report({"INFO"}, ok_msg[:240])
        return {"FINISHED"}


class ANIMAQUINA_OT_URPauseProgram(Operator):
    bl_idname = "object.animaquina_ur_pause_program"
    bl_label = "Pause UR Program"
    bl_description = "Send Dashboard pause command (non-blocking)"
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None or not slot.is_connected or slot.robot_type != "UR":
            return False
        driver = manager.get_driver_for_slot(slot)
        return driver is not None and hasattr(driver, "pause_program")

    def execute(self, context):
        slot = get_active_slot(context)
        driver = manager.get_driver_for_slot(slot) if slot is not None else None
        if slot is None or driver is None:
            return {"CANCELLED"}

        slot.ur_transfer_status = "RUNNING"
        slot.ur_transfer_log = "Sending Dashboard pause ..."

        def _pause_job():
            try:
                driver.pause_program(int(getattr(slot, "ur_dashboard_port", 29999)))
            except Exception:
                pass

        threading.Thread(target=_pause_job, daemon=True).start()
        self.report({"INFO"}, "Pause requested")
        return {"FINISHED"}


class ANIMAQUINA_OT_URStopProgram(Operator):
    bl_idname = "object.animaquina_ur_stop_program"
    bl_label = "Stop UR Program"
    bl_description = "Stop UR motion/program (queue + dashboard). Sets abort flag first so running modals exit cleanly"
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None or not slot.is_connected or slot.robot_type != "UR":
            return False
        driver = manager.get_driver_for_slot(slot)
        return bool(
            driver
            and (
                hasattr(driver, "stop_program")
                or hasattr(driver, "stop_motion")
                or hasattr(driver, "stream_abort")
                or hasattr(driver, "stream_restore_control")
            )
        )

    def execute(self, context):
        slot = get_active_slot(context)
        driver = manager.get_driver_for_slot(slot) if slot is not None else None
        if slot is None or driver is None:
            return {"CANCELLED"}

        # Set abort flag FIRST — running modals check this each tick and
        # clean up on their own, preventing Blender freezes.
        slot.motion_abort_requested = True
        slot.ur_queue_health = "phase=stopped_by_control"

        # Clean up puppet mode state if active
        if bool(getattr(slot, "realtime_puppet_active", False)):
            slot.realtime_puppet_active = False
            slot.realtime_puppet_status = "Puppet Mode: stopped"

        slot.ur_transfer_status = "RUNNING"
        slot.ur_transfer_log = "Stopping UR motion/program ..."

        # Run driver stop calls in a background thread so they never
        # block the Blender UI (network I/O can be slow).
        def _stop_job():
            try:
                if hasattr(driver, "stream_abort"):
                    try:
                        driver.stream_abort("Stopped by user")
                    except Exception:
                        pass
                elif hasattr(driver, "stream_restore_control"):
                    try:
                        driver.stream_restore_control()
                    except Exception:
                        pass
                if hasattr(driver, "stop_motion"):
                    try:
                        driver.stop_motion()
                    except Exception:
                        pass
                if hasattr(driver, "stop_program"):
                    try:
                        driver.stop_program(int(getattr(slot, "ur_dashboard_port", 29999)))
                    except Exception:
                        pass
            except Exception:
                pass

        threading.Thread(target=_stop_job, daemon=True).start()

        slot.motion_active_label = ""
        self.report({"INFO"}, "Stop requested")
        return {"FINISHED"}


class ANIMAQUINA_OT_XArmStopMotion(Operator):
    bl_idname = "object.animaquina_xarm_stop_motion"
    bl_label = "Stop xArm Motion"
    bl_description = "Emergency stop — immediately halt all xArm motion"
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None or not slot.is_connected or slot.robot_type != "XARM":
            return False
        driver = manager.get_driver_for_slot(slot)
        return driver is not None and hasattr(driver, "stop_motion")

    def execute(self, context):
        slot = get_active_slot(context)
        driver = manager.get_driver_for_slot(slot) if slot is not None else None
        if slot is None or driver is None:
            return {"CANCELLED"}

        # Set abort flag FIRST so running modals exit cleanly.
        slot.motion_abort_requested = True

        if bool(getattr(slot, "realtime_puppet_active", False)):
            slot.realtime_puppet_active = False
            slot.realtime_puppet_status = "Puppet Mode: stopped"

        def _stop_job():
            try:
                driver.stop_motion()
            except Exception:
                pass

        threading.Thread(target=_stop_job, daemon=True).start()
        slot.motion_active_label = ""
        self.report({"INFO"}, "xArm stop requested")
        return {"FINISHED"}


class ANIMAQUINA_OT_SendURProgram(Operator):
    bl_idname = "object.animaquina_send_ur_program"
    bl_label = "Send URScript to Robot"
    bl_description = "Send the exported URScript program to the UR controller - the robot will execute immediately"
    bl_options = {"REGISTER"}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot(context)
        if slot is None or not slot.is_connected:
            return False
        return slot.robot_type == "UR"

    def execute(self, context):
        slot = get_active_slot(context)
        driver = manager.get_driver_for_slot(slot)
        if slot is None or driver is None:
            return {"CANCELLED"}

        obj = context.active_object
        obj_script_name = f"{obj.name}.script" if obj else ""
        prog_name = str(getattr(slot, "ur_program_name", "") or "").strip()
        if prog_name.lower().endswith(".script"):
            prog_name = prog_name[:-7]
        if prog_name.lower().endswith(".urp"):
            prog_name = prog_name[:-4]
        if not prog_name:
            prog_name = str(getattr(obj, "name", "") or "animaquina")
        prog_name = "".join(ch if ch.isalnum() or ch in {"_", "-", "."} else "_" for ch in prog_name)
        if not prog_name:
            prog_name = "animaquina"
        slot.ur_program_name = prog_name

        script_candidates = [f"{prog_name}.script"]
        if obj_script_name and obj_script_name not in script_candidates:
            script_candidates.append(obj_script_name)
        text_block, script_name = _get_first_text_block(script_candidates)
        if text_block is None:
            hint = ", ".join(script_candidates) if script_candidates else "<object>.script"
            self.report({"ERROR"}, f"No text block found ({hint}) - export URScript first")
            return {"CANCELLED"}

        content = text_block.as_string()
        if not content.strip():
            self.report({"ERROR"}, "URScript text block is empty")
            return {"CANCELLED"}

        wm = context.window_manager
        if getattr(context, "window", None) is None:
            self.report({"ERROR"}, "Cannot start non-blocking send without an active window")
            return {"CANCELLED"}

        self._aq_send_slot = slot
        self._aq_send_driver = driver
        self._aq_send_script_name = script_name
        self._aq_send_done = False
        self._aq_send_error = None

        slot.ur_transfer_status = "RUNNING"
        slot.ur_transfer_log = f"Sending {prog_name}.script ..."

        def _job():
            self._aq_send_error = driver.send_program(content)
            self._aq_send_done = True

        self._aq_send_thread = threading.Thread(target=_job, daemon=True)
        self._aq_send_thread.start()

        self._aq_send_timer = wm.event_timer_add(0.1, window=context.window)
        wm.modal_handler_add(self)
        self.report({"INFO"}, f"URScript send started: {script_name}")
        return {"RUNNING_MODAL"}

    def modal(self, context, event):
        if event.type != "TIMER":
            return {"PASS_THROUGH"}
        if not getattr(self, "_aq_send_done", False):
            return {"PASS_THROUGH"}

        try:
            context.window_manager.event_timer_remove(self._aq_send_timer)
        except Exception:
            pass

        slot = getattr(self, "_aq_send_slot", None)
        driver = getattr(self, "_aq_send_driver", None)
        script_name = str(getattr(self, "_aq_send_script_name", "") or "program.script")
        err = getattr(self, "_aq_send_error", None)
        if slot is None or driver is None:
            return {"CANCELLED"}

        if err:
            self.report({"ERROR"}, str(err)[:240])
            slot.last_error = str(err)
            slot.ur_transfer_status = "ERROR"
            slot.ur_transfer_log = str(err)[:2000]
            return {"CANCELLED"}

        transport = str(getattr(driver, "last_program_transport", "") or "").strip()
        msg = f"URScript '{script_name}' sent to robot"
        if transport:
            msg += f" via {transport}"
        msg += " - program running"
        slot.ur_transfer_status = "OK"
        slot.ur_transfer_log = msg
        self.report({"INFO"}, msg[:240])
        return {"FINISHED"}



class ANIMAQUINA_OT_RegisterRobotLibrary(Operator):
    bl_idname = "animaquina.register_robot_library"
    bl_label = "Register Robot Library"
    bl_description = (
        "Add the robot asset folder to Blender's Asset Libraries so robot rigs "
        "appear in the Asset Browser"
    )
    bl_options = {"REGISTER"}

    def execute(self, context):
        path = preferences.library_dir_from_prefs()
        if not path:
            self.report({"ERROR"}, "Set the Robot Library Folder in Add-on Preferences first")
            return {"CANCELLED"}
        if not preferences.library_dir_is_valid(path):
            self.report({"ERROR"}, f"No .blend file found in {path}")
            return {"CANCELLED"}

        existing = preferences.find_registered_library()
        if existing is not None:
            existing.path = path
            self.report({"INFO"}, f"Updated asset library '{preferences.ASSET_LIBRARY_NAME}'")
            return {"FINISHED"}

        try:
            bpy.ops.preferences.asset_library_add(directory=path)
        except Exception as exc:
            self.report({"ERROR"}, f"Could not add asset library: {str(exc)[:180]}")
            return {"CANCELLED"}

        libs = context.preferences.filepaths.asset_libraries
        if len(libs):
            libs[-1].name = preferences.ASSET_LIBRARY_NAME
        self.report({"INFO"}, f"Registered '{preferences.ASSET_LIBRARY_NAME}' - open an Asset Browser to use it")
        return {"FINISHED"}


class ANIMAQUINA_OT_UnregisterRobotLibrary(Operator):
    bl_idname = "animaquina.unregister_robot_library"
    bl_label = "Remove"
    bl_description = "Remove the Animaquina robot asset library from Blender's Asset Libraries"
    bl_options = {"REGISTER"}

    def execute(self, context):
        libs = context.preferences.filepaths.asset_libraries
        for i, lib in enumerate(libs):
            if lib.name == preferences.ASSET_LIBRARY_NAME:
                bpy.ops.preferences.asset_library_remove(index=i)
                self.report({"INFO"}, "Robot asset library removed")
                return {"FINISHED"}
        self.report({"WARNING"}, "Robot asset library was not registered")
        return {"CANCELLED"}


OPERATOR_CLASSES = [
    ANIMAQUINA_OT_RegisterRobotLibrary,
    ANIMAQUINA_OT_UnregisterRobotLibrary,
    ANIMAQUINA_OT_AddSlot,
    ANIMAQUINA_OT_RemoveSlot,
    ANIMAQUINA_OT_ConnectRobot,
    ANIMAQUINA_OT_DisconnectRobot,
    ANIMAQUINA_OT_URDebugStatus,
    ANIMAQUINA_OT_UpdatePose,
    ANIMAQUINA_OT_ManualMode,
    ANIMAQUINA_OT_SnapTargetToTCP,
    ANIMAQUINA_OT_MoveToTarget,
    ANIMAQUINA_OT_RealTimePuppetStart,
    ANIMAQUINA_OT_RealTimePuppetStop,
    ANIMAQUINA_OT_GoHome,
    ANIMAQUINA_OT_Reset,
    ANIMAQUINA_OT_AddMarker,
    ANIMAQUINA_OT_SetURAxes,
    ANIMAQUINA_OT_SetKukaAxes,
    ANIMAQUINA_OT_SetXArmAxes,
    ANIMAQUINA_OT_SetTool,
    ANIMAQUINA_OT_SetSimulation,
    ANIMAQUINA_OT_BlenderSimValidatePath,
    ANIMAQUINA_OT_BlenderSimClearPath,
    ANIMAQUINA_OT_ClearSimulation,
    ANIMAQUINA_OT_NewtonProbe,
    ANIMAQUINA_OT_NewtonInstallDeps,
    ANIMAQUINA_OT_InstallURDeps,
    ANIMAQUINA_OT_InstallPackage,
    ANIMAQUINA_OT_ValidatePath,
    ANIMAQUINA_OT_NewtonKeyframeIKPath,
    ANIMAQUINA_OT_NewtonClearIKKeyframes,
    ANIMAQUINA_OT_SendPath,
    ANIMAQUINA_OT_SendPathQueueUR,
    ANIMAQUINA_OT_ExportKRL,
    ANIMAQUINA_OT_SaveProgramToRobot,
    ANIMAQUINA_OT_StageProgramOnRobot,
    ANIMAQUINA_OT_LoadProgramOnRobot,
    ANIMAQUINA_OT_UploadStreamProgram,
    ANIMAQUINA_OT_SelectStreamProgram,
    ANIMAQUINA_OT_KukaPlayProgram,
    ANIMAQUINA_OT_KukaStopProgram,
    ANIMAQUINA_OT_KukaCancelProgram,
    ANIMAQUINA_OT_KukaStreamExportedProgram,
    ANIMAQUINA_OT_SetUROrientation,
    ANIMAQUINA_OT_SetHome,
    ANIMAQUINA_OT_DebugAddVar,
    ANIMAQUINA_OT_RecordStart,
    ANIMAQUINA_OT_RecordStop,
    ANIMAQUINA_OT_RecordSaveCSV,
    ANIMAQUINA_OT_DebugRemoveVar,
    ANIMAQUINA_OT_ClearRunState,
    ANIMAQUINA_OT_StoreHomeToRobot,
    ANIMAQUINA_OT_ExportUR,
    ANIMAQUINA_OT_URSaveProgramToRobot,
    ANIMAQUINA_OT_URLoadProgramOnRobot,
    ANIMAQUINA_OT_URPauseProgram,
    ANIMAQUINA_OT_URStopProgram,
    ANIMAQUINA_OT_XArmStopMotion,
    ANIMAQUINA_OT_SendURProgram,
]



