# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
# Animaquina — apply cache to rig, TCP, base, target, tool (Section 10)
# REFERENCE: oldversions/animaquinakuka_0.0.8/.../__init__.py update_twin_angles (joints), update_tcp (C,B,A euler), update_robotBase.

import math
import bpy
from mathutils import Matrix, Euler

from animaquina_core.runtime.conversions import rad2deg

# Dirty-check for base: keyed by base_obj.name → (pos_m_tuple, euler_rad_tuple).
# apply_base_object is skipped when values haven't changed — $BASE is static most of the time.
_last_applied_base: dict = {}

# Rotation-mode cache: set of (armature_name, bone_name) confirmed as XYZ.
# Avoids re-checking and re-setting rotation_mode on the hot path each tick.
_bone_rotation_mode_ok: set = set()


def _matrix_without_scale(mat: Matrix) -> Matrix:
    """Return a transform matrix with translation+rotation only (strip object scale)."""
    loc = mat.to_translation()
    rot = mat.to_euler("XYZ").to_matrix().to_4x4()
    return Matrix.Translation((loc.x, loc.y, loc.z)) @ rot


def apply_joint_angles(armature_obj, joints_deg: list, joint_axis_map: list) -> None:
    """Apply joint angles (deg) to armature. Old: update_twin_angles — rotation_axes e.g. -Y,-X,-X,-Y,-X,-Y; negate angle if '-'."""
    if armature_obj is None or armature_obj.type != "ARMATURE":
        return
    arm_name = armature_obj.name
    for i in range(min(6, len(joints_deg))):
        angle_deg = joints_deg[i]
        axis = joint_axis_map[i] if i < len(joint_axis_map) else "Y"
        if axis.startswith("-"):
            angle_deg = -angle_deg
            axis = axis[1:]
        bone_name = f"joint_{i + 1}"
        bone = armature_obj.pose.bones.get(bone_name)
        if not bone:
            continue
        # Set rotation_mode only on first encounter; skip the check on the hot path thereafter.
        cache_key = (arm_name, bone_name)
        if cache_key not in _bone_rotation_mode_ok:
            if bone.rotation_mode != "XYZ":
                bone.rotation_mode = "XYZ"
            _bone_rotation_mode_ok.add(cache_key)
        angle_rad = math.radians(angle_deg)
        if axis == "X":
            bone.rotation_euler[0] = angle_rad
        elif axis == "Y":
            bone.rotation_euler[1] = angle_rad
        elif axis == "Z":
            bone.rotation_euler[2] = angle_rad


def _tcp_base_to_world_matrix(pos_m: tuple, euler_rad: tuple, base_obj) -> tuple:
    """TCP in base frame (pos_m, euler_rad XYZ) → (world_location, world_euler_rad).
    Drivers return Blender-ready XYZ euler; no robot-specific conversion here.
    If base_obj is None, treat TCP as already in world (base at origin)."""
    trans = Matrix.Translation((float(pos_m[0]), float(pos_m[1]), float(pos_m[2])))
    rot = Euler((float(euler_rad[0]), float(euler_rad[1]), float(euler_rad[2])), "XYZ").to_matrix().to_4x4()
    tcp_in_base = trans @ rot
    if base_obj is not None and base_obj.name in bpy.data.objects:
        world_mat = base_obj.matrix_world @ tcp_in_base
    else:
        world_mat = tcp_in_base
    loc = world_mat.to_translation()
    euler = world_mat.to_euler("XYZ")
    return ((loc.x, loc.y, loc.z), (euler.x, euler.y, euler.z))


def get_slot_base_world_matrix(slot):
    """Return the J0 world matrix for this slot (robot's active base frame).
    Used for live moves — positions sent to the robot are relative to J0."""
    if slot.base_object is not None and slot.base_object.name in bpy.data.objects:
        return _matrix_without_scale(slot.base_object.matrix_world.copy())
    trans = Matrix.Translation((float(slot.base_pos_m[0]), float(slot.base_pos_m[1]), float(slot.base_pos_m[2])))
    rot = Euler((float(slot.base_euler_rad[0]), float(slot.base_euler_rad[1]), float(slot.base_euler_rad[2])), "XYZ").to_matrix().to_4x4()
    return trans @ rot


def tcp_world_euler(slot) -> tuple:
    """Current TCP orientation expressed in WORLD frame.

    slot.tcp_euler_rad is stored in the robot base (J0) frame. Toolpath senders
    must hand a world-frame orientation to world_points_to_base_waypoints /
    world_points_to_blender_base_waypoints (which apply base_inv), exactly like
    Move to Target reads target_obj.matrix_world.to_euler(). Passing the J0-frame
    euler directly would double-apply base_inv and rotate the tool incorrectly
    whenever the base object carries a rotation."""
    j0_rot = get_slot_base_world_matrix(slot).to_3x3().to_4x4()
    tcp_rot = Euler(
        (float(slot.tcp_euler_rad[0]), float(slot.tcp_euler_rad[1]), float(slot.tcp_euler_rad[2])),
        "XYZ",
    ).to_matrix().to_4x4()
    world_euler = (j0_rot @ tcp_rot).to_euler("XYZ")
    return (world_euler.x, world_euler.y, world_euler.z)


def get_slot_armature_world_matrix(slot):
    """Return the rig armature's world matrix (scale-stripped) for Newton/MuJoCo frame alignment.
    The MuJoCo model is exported in armature-local space (armature origin = MuJoCo world origin),
    so waypoints must be expressed in this frame — not the J0 (base_object) frame.
    Falls back to get_slot_base_world_matrix if no armature is available."""
    arm = getattr(slot, "rig_armature", None)
    if arm is not None and getattr(arm, "type", None) == "ARMATURE" and arm.name in bpy.data.objects:
        return _matrix_without_scale(arm.matrix_world.copy())
    return get_slot_base_world_matrix(slot)


def get_slot_blender_base_world_matrix(slot):
    """Return the Blender Reference Base world matrix (J0's parent empty).
    Used for KRL export — exported positions are relative to this frame, independent
    of the robot's $BASE offset. If J0 has no parent, falls back to J0's own matrix."""
    j0 = slot.base_object
    if j0 is not None and j0.name in bpy.data.objects:
        if j0.parent is not None and j0.parent.name in bpy.data.objects:
            return _matrix_without_scale(j0.parent.matrix_world.copy())
        return _matrix_without_scale(j0.matrix_world.copy())
    return Matrix.Identity(4)


def world_waypoints_to_base_frame(slot, waypoints_world):
    """Convert waypoints from world to this robot's base frame. waypoints_world: list of ((x,y,z), (euler_rad)). Returns list of ((x,y,z), (euler_rad)) in base frame."""
    base_inv = get_slot_base_world_matrix(slot).inverted()
    out = []
    for pos_m, euler_rad in waypoints_world:
        trans = Matrix.Translation((float(pos_m[0]), float(pos_m[1]), float(pos_m[2])))
        rot = Euler((float(euler_rad[0]), float(euler_rad[1]), float(euler_rad[2])), "XYZ").to_matrix().to_4x4()
        world_mat = trans @ rot
        base_mat = base_inv @ world_mat
        loc = base_mat.to_translation()
        euler = base_mat.to_euler("XYZ")
        out.append(((loc.x, loc.y, loc.z), (euler.x, euler.y, euler.z)))
    return out


def target_pose_in_robot_base_frame(slot, target_obj):
    """Commanded target pose expressed in the robot base (J0) frame.

    Deliberately uses world_waypoints_to_base_frame for every robot type,
    including KUKA: this is for logging next to the measured TCP, and
    slot.tcp_euler_rad / slot.tcp_pos_m are stored in the J0 frame. The
    machine-zero variant used by KUKA moves and KRL export would put the two
    poses in different frames and make the pair meaningless.
    """
    loc = tuple(target_obj.matrix_world.to_translation())
    try:
        euler = tuple(target_obj.matrix_world.to_euler("XYZ"))
    except Exception:
        euler = tuple(target_obj.rotation_euler)
    return world_waypoints_to_base_frame(slot, [(loc, euler)])[0]


def world_points_to_base_waypoints(slot, points_world, euler_rad):
    """
    Convert world points + constant world euler to base-frame waypoints in one pass.
    points_world: iterable[(x, y, z)] ; euler_rad: (rx, ry, rz)
    returns: list[((x, y, z), (rx, ry, rz))] in base frame
    """
    base_inv = get_slot_base_world_matrix(slot).inverted()
    rot = Euler((float(euler_rad[0]), float(euler_rad[1]), float(euler_rad[2])), "XYZ").to_matrix().to_4x4()
    out = []
    for pos_m in points_world:
        trans = Matrix.Translation((float(pos_m[0]), float(pos_m[1]), float(pos_m[2])))
        base_mat = base_inv @ (trans @ rot)
        loc = base_mat.to_translation()
        euler = base_mat.to_euler("XYZ")
        out.append(((loc.x, loc.y, loc.z), (euler.x, euler.y, euler.z)))
    return out


def world_waypoints_to_blender_base_frame(slot, waypoints_world):
    """Convert waypoints from world to Blender Base frame (J0's parent).
    Same frame used by KRL export — positions are relative to machine zero,
    independent of the robot's current $BASE offset.
    waypoints_world: list of ((x,y,z), (euler_rad)).
    Returns list of ((x,y,z), (euler_rad)) in Blender Base frame."""
    base_inv = get_slot_blender_base_world_matrix(slot).inverted()
    out = []
    for pos_m, euler_rad in waypoints_world:
        trans = Matrix.Translation((float(pos_m[0]), float(pos_m[1]), float(pos_m[2])))
        rot = Euler((float(euler_rad[0]), float(euler_rad[1]), float(euler_rad[2])), "XYZ").to_matrix().to_4x4()
        world_mat = trans @ rot
        base_mat = base_inv @ world_mat
        loc = base_mat.to_translation()
        euler = base_mat.to_euler("XYZ")
        out.append(((loc.x, loc.y, loc.z), (euler.x, euler.y, euler.z)))
    return out


def world_points_to_blender_base_waypoints(slot, points_world, euler_rad):
    """Convert world points + constant euler to Blender Base frame waypoints.
    Same frame used by KRL export.
    points_world: iterable[(x,y,z)] ; euler_rad: (rx, ry, rz)
    Returns list[((x,y,z), (rx, ry, rz))] in Blender Base frame."""
    base_inv = get_slot_blender_base_world_matrix(slot).inverted()
    rot = Euler((float(euler_rad[0]), float(euler_rad[1]), float(euler_rad[2])), "XYZ").to_matrix().to_4x4()
    out = []
    for pos_m in points_world:
        trans = Matrix.Translation((float(pos_m[0]), float(pos_m[1]), float(pos_m[2])))
        base_mat = base_inv @ (trans @ rot)
        loc = base_mat.to_translation()
        euler = base_mat.to_euler("XYZ")
        out.append(((loc.x, loc.y, loc.z), (euler.x, euler.y, euler.z)))
    return out


def world_tcp_to_base_frame(slot) -> None:
    """Convert slot.tcp_pos_m and tcp_euler_rad from world to base frame using slot.base_object. In-place. Call when switching to Manual base so TCP is stored relative to base and will move with the base."""
    if slot.base_object is None or slot.base_object.name not in bpy.data.objects:
        return
    base_inv = slot.base_object.matrix_world.inverted()
    trans = Matrix.Translation((float(slot.tcp_pos_m[0]), float(slot.tcp_pos_m[1]), float(slot.tcp_pos_m[2])))
    rot = Euler((float(slot.tcp_euler_rad[0]), float(slot.tcp_euler_rad[1]), float(slot.tcp_euler_rad[2])), "XYZ").to_matrix().to_4x4()
    world_mat = trans @ rot
    base_frame_mat = base_inv @ world_mat
    loc = base_frame_mat.to_translation()
    euler = base_frame_mat.to_euler("XYZ")
    slot.tcp_pos_m = (loc.x, loc.y, loc.z)
    slot.tcp_euler_rad = (euler.x, euler.y, euler.z)
    slot.tcp_euler_deg = (rad2deg(euler.x), rad2deg(euler.y), rad2deg(euler.z))


def apply_tcp_object(tcp_obj, pos_m: tuple, euler_rad: tuple, base_obj=None) -> None:
    """Set tcp_obj pose. If base_obj is set, pos_m/euler_rad are in base frame and are transformed to world."""
    if tcp_obj is None:
        return
    if base_obj is not None and base_obj.name in bpy.data.objects:
        world_loc, world_euler = _tcp_base_to_world_matrix(pos_m, euler_rad, base_obj)
        tcp_obj.location = world_loc
        tcp_obj.rotation_euler = world_euler
    else:
        tcp_obj.location = (float(pos_m[0]), float(pos_m[1]), float(pos_m[2]))
        tcp_obj.rotation_euler = (float(euler_rad[0]), float(euler_rad[1]), float(euler_rad[2]))


def apply_base_object(base_obj, pos_m: tuple, euler_rad: tuple) -> None:
    """Set J0 location and rotation. Values are Blender-ready — all robot-specific
    conversion (KUKA ZYX, inverse transform, etc.) is done in the driver's read_base()."""
    if base_obj is None:
        return
    base_obj.location = (float(pos_m[0]), float(pos_m[1]), float(pos_m[2]))
    base_obj.rotation_euler = (float(euler_rad[0]), float(euler_rad[1]), float(euler_rad[2]))


def apply_target(target_obj, tcp_obj) -> None:
    if target_obj is None or tcp_obj is None:
        return
    target_obj.location = tcp_obj.location.copy()
    target_obj.rotation_euler = tcp_obj.rotation_euler.copy()


def ensure_tool_constraint(tool_obj, tcp_obj) -> None:
    """Child Of tool to TCP; clear inverse only when constraint is new or target changed (keeps realtime path light)."""
    if tool_obj is None or tcp_obj is None:
        return
    constraint = next((c for c in tool_obj.constraints if c.name == "animaquina_tool_constraint"), None)
    if constraint is None:
        constraint = tool_obj.constraints.new(type="CHILD_OF")
        constraint.name = "animaquina_tool_constraint"
        constraint.target = tcp_obj
        _tool_constraint_clear_inverse(tool_obj)
        return
    if constraint.target == tcp_obj:
        return  # Hot path: no update, no ops
    constraint.target = tcp_obj
    _tool_constraint_clear_inverse(tool_obj)


def _tool_constraint_clear_inverse(tool_obj) -> None:
    """Run once when constraint is created or target changes; avoids doing this every poll."""
    try:
        if getattr(bpy.context, "view_layer", None):
            bpy.context.view_layer.update()
        bpy.ops.object.select_all(action="DESELECT")
        tool_obj.select_set(True)
        bpy.context.view_layer.objects.active = tool_obj
        bpy.ops.constraint.childof_clear_inverse(constraint="animaquina_tool_constraint", owner="OBJECT")
    except Exception:
        pass


def apply_full_pose(slot, update_view_layer: bool = True) -> None:
    """Update rig, TCP, base, target from slot cache; ensure tool constraint.

    All drivers (KUKA, UR, xArm) return TCP in robot-base frame (relative to J0).
    J0 is driven by $BASE when available. Apply J0 first so its matrix_world is
    as current as possible before TCP uses it (1-tick lag is acceptable at poll rates).
    """
    apply_joint_angles(
        slot.rig_armature,
        slot.joints_deg,
        slot.joint_axis_map,
    )
    # Drive J0 from robot's $BASE when the driver provides it.
    # Dirty check: $BASE is static during normal operation; skip the Blender property
    # write when values haven't changed to avoid triggering unnecessary depsgraph work.
    _base_obj = slot.base_object
    if _base_obj is not None:
        _base_key = _base_obj.name
        _new_base = (tuple(slot.base_pos_m), tuple(slot.base_euler_rad))
        if _last_applied_base.get(_base_key) != _new_base:
            apply_base_object(_base_obj, _new_base[0], _new_base[1])
            _last_applied_base[_base_key] = _new_base
    # TCP from $POS_ACT is in Blender Base frame (machine zero / reference frame).
    # Use J0's parent (the Blender Base empty) for world placement — do NOT include J0's
    # local $BASE offset here, as that offset is already reflected in the physical position
    # the robot reports. Falls back to J0 itself when no parent is set.
    _tcp_base_obj = _base_obj
    if _tcp_base_obj is not None and _tcp_base_obj.parent is not None:
        _tcp_base_obj = _tcp_base_obj.parent
    apply_tcp_object(
        slot.tcp_object,
        tuple(slot.tcp_pos_m),
        tuple(slot.tcp_euler_rad),
        base_obj=_tcp_base_obj,
    )
    if slot.tool_object and slot.tcp_object:
        ensure_tool_constraint(slot.tool_object, slot.tcp_object)
    if update_view_layer and bpy.context.view_layer:
        bpy.context.view_layer.update()
