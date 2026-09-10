# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later

"""Generates MuJoCo XML directly from the Blender robot rig."""

from __future__ import annotations

import math

try:
    from mathutils import Matrix, Vector
    _HAS_MATHUTILS = True
except ImportError:
    _HAS_MATHUTILS = False


def _axis_str_to_vec(axis_str: str) -> tuple[float, float, float]:
    sign = -1.0 if axis_str.startswith("-") else 1.0
    ax = axis_str.lstrip("-").upper()
    base = {"X": (1.0, 0.0, 0.0), "Y": (0.0, 1.0, 0.0), "Z": (0.0, 0.0, 1.0)}.get(ax, (0.0, 0.0, 1.0))
    return (sign * base[0], sign * base[1], sign * base[2])


def _bone_ik_limits_rad(bone, axis_str: str) -> tuple[float, float]:
    full = (-2.0 * math.pi, 2.0 * math.pi)
    ax = axis_str.lstrip("-").upper()
    negate = axis_str.startswith("-")
    if ax == "X" and getattr(bone, "use_ik_limit_x", False):
        lo, hi = bone.ik_min_x, bone.ik_max_x
    elif ax == "Y" and getattr(bone, "use_ik_limit_y", False):
        lo, hi = bone.ik_min_y, bone.ik_max_y
    elif ax == "Z" and getattr(bone, "use_ik_limit_z", False):
        lo, hi = bone.ik_min_z, bone.ik_max_z
    else:
        return full
    if negate:
        lo, hi = -hi, -lo
    return (lo, hi)


def _collect_bone_meshes(arm_obj, rig_collection) -> dict:
    result: dict[str, list] = {}
    if rig_collection is None:
        return result

    def _walk(coll):
        for obj in coll.objects:
            if (
                obj.parent is arm_obj
                and getattr(obj, "parent_type", "") == "BONE"
                and obj.parent_bone
                and obj.type == "MESH"
            ):
                result.setdefault(obj.parent_bone, []).append(obj)
        for child in coll.children:
            _walk(child)

    _walk(rig_collection)
    return result


def _bbox_geom_lines(mesh_obj, inv_body_world_mat, indent: str) -> list[str]:
    wm = mesh_obj.matrix_world
    corners = [inv_body_world_mat @ wm @ Vector(c) for c in mesh_obj.bound_box]
    min_x = min(c.x for c in corners)
    max_x = max(c.x for c in corners)
    min_y = min(c.y for c in corners)
    max_y = max(c.y for c in corners)
    min_z = min(c.z for c in corners)
    max_z = max(c.z for c in corners)
    cx = (min_x + max_x) / 2
    cy = (min_y + max_y) / 2
    cz = (min_z + max_z) / 2
    sx = max((max_x - min_x) / 2, 0.001)
    sy = max((max_y - min_y) / 2, 0.001)
    sz = max((max_z - min_z) / 2, 0.001)
    return [
        f'{indent}<geom type="box" pos="{cx:.5f} {cy:.5f} {cz:.5f}" '
        f'size="{sx:.5f} {sy:.5f} {sz:.5f}" rgba="0.6 0.6 0.6 0.4" contype="1" conaffinity="1"/>'
    ]


def export_slot_to_mujoco_xml(slot) -> str:
    if not _HAS_MATHUTILS:
        raise RuntimeError("mathutils not available â€” mujoco_export must run inside Blender Python")

    arm_obj = slot.rig_armature
    if arm_obj is None or arm_obj.type != "ARMATURE":
        raise RuntimeError("No armature set on slot; cannot export MuJoCo model")

    arm = arm_obj.data
    joint_axes = [getattr(slot, f"joint_axis_{i}", "Z") for i in range(6)]
    bone_meshes = _collect_bone_meshes(arm_obj, slot.rig_collection)
    j0_obj = slot.base_object

    def ind(level: int) -> str:
        return "  " * level

    lines: list[str] = [
        '<mujoco model="robot">',
        f'{ind(1)}<compiler angle="radian"/>',
        f'{ind(1)}<option gravity="0 0 -9.81"/>',
        f'{ind(1)}<worldbody>',
        f'{ind(2)}<body name="base_link" pos="0 0 0">',
    ]

    if j0_obj is not None and j0_obj.type == "MESH":
        inv_base = arm_obj.matrix_world.inverted()
        lines.extend(_bbox_geom_lines(j0_obj, inv_base, ind(3)))

    num_joints = 0
    parent_bone_mat = Matrix.Identity(4)
    for i in range(6):
        bone_name = f"joint_{i + 1}"
        bone = arm.bones.get(bone_name)
        if bone is None:
            break
        axis_str = joint_axes[i]
        ax, ay, az = _axis_str_to_vec(axis_str)
        lim_min, lim_max = _bone_ik_limits_rad(bone, axis_str)
        head_arm = bone.matrix_local.to_translation().to_4d()
        head_arm.w = 1.0
        pos_in_parent = (parent_bone_mat.inverted() @ head_arm).to_3d()
        bone_rot3 = bone.matrix_local.to_3x3()
        parent_rot3 = parent_bone_mat.to_3x3()
        rot_in_parent = parent_rot3.inverted() @ bone_rot3
        q = rot_in_parent.to_quaternion()
        body_level = 3 + i
        geom_level = body_level + 1
        lines.append(
            f'{ind(body_level)}<body name="link_{i + 1}" '
            f'pos="{pos_in_parent.x:.5f} {pos_in_parent.y:.5f} {pos_in_parent.z:.5f}" '
            f'quat="{q.w:.5f} {q.x:.5f} {q.y:.5f} {q.z:.5f}">'
        )
        lines.append(
            f'{ind(geom_level)}<joint name="{bone_name}" type="hinge" '
            f'axis="{ax:.4f} {ay:.4f} {az:.4f}" range="{lim_min:.4f} {lim_max:.4f}" damping="0.01"/>'
        )
        inv_body_world = (arm_obj.matrix_world @ bone.matrix_local).inverted()
        for mesh_obj in bone_meshes.get(bone_name, []):
            lines.extend(_bbox_geom_lines(mesh_obj, inv_body_world, ind(geom_level)))
        parent_bone_mat = bone.matrix_local
        num_joints += 1

    if num_joints == 0:
        raise RuntimeError("No joint_1..joint_6 bones found in armature â€” cannot export MuJoCo model.")

    last_bone_name = f"joint_{num_joints}"
    last_bone = arm.bones.get(last_bone_name)
    tcp_bone_rest = arm.bones.get("tcp")
    if last_bone is not None:
        inv_body = last_bone.matrix_local.inverted()
        has_tool = getattr(slot, "has_tool", False)
        if not has_tool and tcp_bone_rest is not None:
            pos_off = (tcp_bone_rest.head_local - last_bone.tail_local).length
            rot_off = last_bone.matrix_local.to_quaternion().rotation_difference(
                tcp_bone_rest.matrix_local.to_quaternion()
            ).angle
            has_tool = pos_off > 1e-6 or rot_off > 1e-4
        # Check offline tool frame offset (set via the Info panel)
        _tool_frame_active = (
            getattr(slot, "tool_frame_valid", False)
            and (
                any(abs(v) > 1e-6 for v in getattr(slot, "tool_frame_pos_m", (0, 0, 0)))
                or any(abs(v) > 1e-6 for v in getattr(slot, "tool_frame_euler_deg", (0, 0, 0)))
            )
        )
        if not has_tool and _tool_frame_active:
            has_tool = True
        if has_tool and tcp_bone_rest is not None and not _tool_frame_active:
            site_pos = (inv_body @ tcp_bone_rest.tail_local.to_4d()).to_3d()
        elif _tool_frame_active:
            # Compute tcp site from flange + tool offset.
            # Tool frame values are in KUKA flange frame (Z = tool direction).
            # In bone-local space, Y is along the bone (= tool direction).
            # Mapping: KUKA (X,Y,Z) -> bone-local (X, Z, -Y).
            flange_pos = (inv_body @ last_bone.tail_local.to_4d()).to_3d()
            tp = getattr(slot, "tool_frame_pos_m", (0, 0, 0))
            tool_offset = Vector((tp[0], tp[2], -tp[1]))
            site_pos = flange_pos + tool_offset
        else:
            site_pos = (inv_body @ last_bone.tail_local.to_4d()).to_3d()
        tcp_obj = getattr(slot, "tcp_object", None)
        j6_pb = arm_obj.pose.bones.get(last_bone_name)
        site_quat = None
        if tcp_obj is not None and j6_pb is not None:
            try:
                arm_inv = arm_obj.matrix_world.inverted()
                tcp_arm_rot = (arm_inv @ tcp_obj.matrix_world).to_3x3()
                j6_posed_rot = j6_pb.matrix.to_3x3()
                site_rot = j6_posed_rot.inverted() @ tcp_arm_rot
                site_quat = site_rot.to_quaternion()
            except Exception:
                site_quat = None
        # Offline tool rotation: KUKA A,B,C (intrinsic ZYX) stored as
        # raw degrees.  Convert to a quaternion in bone-local space and
        # use it as the site orientation when no tcp_object is available.
        if site_quat is None and _tool_frame_active:
            te = getattr(slot, "tool_frame_euler_deg", (0, 0, 0))
            a_r = math.radians(te[0])
            b_r = math.radians(te[1])
            c_r = math.radians(te[2])
            if abs(a_r) > 1e-6 or abs(b_r) > 1e-6 or abs(c_r) > 1e-6:
                from mathutils import Quaternion
                rz = Quaternion((0, 0, 1), a_r).to_matrix()
                ry = Quaternion((0, 1, 0), b_r).to_matrix()
                rx = Quaternion((1, 0, 0), c_r).to_matrix()
                kuka_rot = rz @ ry @ rx  # intrinsic ZYX
                # Remap KUKA axes to bone-local: X→X, Y→-Z, Z→Y
                P = Matrix(((1, 0, 0), (0, 0, 1), (0, -1, 0)))
                tool_rot_bone = P @ kuka_rot @ P.transposed()
                site_quat = tool_rot_bone.to_quaternion()
        if site_quat is None and has_tool and tcp_bone_rest is not None:
            tcp_rot = last_bone.matrix_local.to_3x3().inverted() @ tcp_bone_rest.matrix_local.to_3x3()
            site_quat = tcp_rot.to_quaternion()
        if site_quat is not None:
            site_attr = (
                f'pos="{site_pos.x:.5f} {site_pos.y:.5f} {site_pos.z:.5f}" '
                f'quat="{site_quat.w:.5f} {site_quat.x:.5f} {site_quat.y:.5f} {site_quat.z:.5f}"'
            )
        else:
            site_attr = f'pos="{site_pos.x:.5f} {site_pos.y:.5f} {site_pos.z:.5f}"'
        site_level = 3 + num_joints
        lines.append(f'{ind(site_level)}<site name="tcp_site" {site_attr}/>')

    for i in range(num_joints - 1, -1, -1):
        lines.append(f'{ind(3 + i)}</body>')
    lines.extend([f'{ind(2)}</body>', f'{ind(1)}</worldbody>', "</mujoco>"])
    return "\n".join(lines)
