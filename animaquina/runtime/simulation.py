# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
# Animaquina — simulation setup: duplicate, rename, rest pose, IK (Section 10, 18)

import math

import bpy
from mathutils import Euler, Matrix, Vector


def _log(msg: str) -> None:
    """Console log for the sim pipeline — silent unless scene Debug is on."""
    try:
        if not bpy.context.scene.animaquina.global_debug:
            return
    except Exception:
        pass
    print(f"[animaquina] sim: {msg}")


def _iter_collection_objects_recursive(coll):
    if coll is None:
        return
    for obj in getattr(coll, "objects", []):
        yield obj
    for child in getattr(coll, "children", []):
        yield from _iter_collection_objects_recursive(child)


def _axis_to_ik_locks(axis_str: str) -> tuple[bool, bool, bool]:
    """
    Convert a configured joint axis (e.g. 'Y', '-Z') into Blender IK axis locks.
    Returns (lock_ik_x, lock_ik_y, lock_ik_z), where only the configured axis is unlocked.
    """
    axis = (axis_str or "Y").lstrip("-").upper()
    if axis == "X":
        return (False, True, True)
    if axis == "Y":
        return (True, False, True)
    if axis == "Z":
        return (True, True, False)
    return (True, False, True)


def _apply_joint_ik_axis_locks(arm_obj, slot) -> None:
    """Apply IK axis locks to sim armature joints based on slot.joint_axis_0..5."""
    if arm_obj is None or arm_obj.type != "ARMATURE":
        return
    for i in range(6):
        pb = arm_obj.pose.bones.get(f"joint_{i + 1}")
        if pb is None:
            continue
        axis_str = getattr(slot, f"joint_axis_{i}", "Y")
        lock_x, lock_y, lock_z = _axis_to_ik_locks(axis_str)
        try:
            pb.lock_ik_x = lock_x
            pb.lock_ik_y = lock_y
            pb.lock_ik_z = lock_z
        except Exception as e:
            _log(f"could not set IK axis locks on joint_{i + 1}: {e}")


def _safe_set_attr(obj, name: str, value) -> None:
    """Best-effort attribute setter for Blender version compatibility — intentionally silent."""
    try:
        if hasattr(obj, name):
            setattr(obj, name, value)
    except Exception:
        pass


def _configure_itasc_for_sim(arm_obj) -> None:
    """
    Configure Blender iTaSC solver on the sim armature to reduce flipping when moving/rotating
    the target interactively.

    Tuning rationale (from Blender docs + practical defaults):
    - Use stateful Simulation mode to preserve continuity from previous pose.
    - Use SDLS for automatic damping near singularities (more stable, less reactive).
    - Reiterate on all frames for better convergence when dragging/rotating targets.
    - Use bounded auto substeps (max ~10 ms) for stability in simulation mode.
    - Moderate feedback / velocity limit to avoid overshoot and sudden branch jumps.
    """
    if arm_obj is None or arm_obj.type != "ARMATURE" or getattr(arm_obj, "pose", None) is None:
        return
    pose = arm_obj.pose
    _safe_set_attr(pose, "ik_solver", "ITASC")
    itasc = getattr(pose, "ik_param", None)
    if itasc is None:
        return

    # Solver behavior (stability first).
    _safe_set_attr(itasc, "mode", "SIMULATION")
    _safe_set_attr(itasc, "solver", "SDLS")
    _safe_set_attr(itasc, "reiteration_method", "ALWAYS")

    # Convergence / tracking.
    _safe_set_attr(itasc, "precision", 0.001)     # ~1 mm end-effector precision target
    _safe_set_attr(itasc, "iterations", 200)      # enough for reiterated solve while dragging
    _safe_set_attr(itasc, "feedback", 20.0)       # Blender manual default and stable range start
    _safe_set_attr(itasc, "velocity_max", 12.0)   # lower than default to reduce snap/flip tendency

    # Time stepping: docs recommend substeps <= ~10 ms for stability.
    _safe_set_attr(itasc, "use_auto_step", True)
    _safe_set_attr(itasc, "step_min", 0.001)      # 1 ms lower bound
    _safe_set_attr(itasc, "step_max", 0.01)       # 10 ms upper bound
    _safe_set_attr(itasc, "step_count", 4)        # fallback if auto-step is disabled by Blender/version

    # If the user switches solver to DLS manually later, these values are sane starting points.
    _safe_set_attr(itasc, "damping_epsilon", 0.1)
    _safe_set_attr(itasc, "damping_max", 0.5)

    # Keep root translation off for fixed-base industrial robot rigs.
    _safe_set_attr(itasc, "translate_root_bones", False)


def _sim_name(name: str) -> str:
    """Normalize names for simulation duplicates: prefer `_sim` over `_twin`."""
    if not name:
        return name
    if name.endswith("_twin"):
        return name[:-5] + "_sim"
    if name.endswith(".twin"):
        return name[:-5] + ".sim"
    return name.replace("twin", "sim")


def _unique_collection_name(base_name: str) -> str:
    if base_name not in bpy.data.collections:
        return base_name
    i = 1
    while True:
        cand = f"{base_name}.{i:03d}"
        if cand not in bpy.data.collections:
            return cand
        i += 1


def _duplicate_collection_recursive(src_coll, object_map: dict):
    """
    Deep-copy a collection tree and its objects. Mesh/armature data is copied so
    simulation edits do not mutate the source rig.
    """
    new_coll = bpy.data.collections.new(_unique_collection_name(_sim_name(src_coll.name)))
    for obj in src_coll.objects:
        dup = obj.copy()
        if getattr(obj, "data", None) is not None:
            try:
                dup.data = obj.data.copy()
            except Exception as e:
                _log(f"could not copy data for '{obj.name}', sharing original: {e}")
                dup.data = obj.data
        if dup.animation_data:
            try:
                dup.animation_data_clear()
            except Exception as e:
                _log(f"could not clear animation data on '{obj.name}': {e}")
        new_coll.objects.link(dup)
        object_map[obj] = dup
    for child in src_coll.children:
        new_child = _duplicate_collection_recursive(child, object_map)
        new_coll.children.link(new_child)
    return new_coll


def _relink_parenting(object_map: dict) -> None:
    """Restore parenting between duplicated objects after the copy pass."""
    for src_obj, dup_obj in object_map.items():
        if src_obj.parent in object_map:
            dup_obj.parent = object_map[src_obj.parent]
            dup_obj.parent_type = src_obj.parent_type
            dup_obj.parent_bone = src_obj.parent_bone
            try:
                dup_obj.matrix_parent_inverse = src_obj.matrix_parent_inverse.copy()
            except Exception as e:
                _log(f"could not copy matrix_parent_inverse for '{src_obj.name}': {e}")


def _find_first_armature_recursive(coll):
    if coll is None:
        return None
    for obj in coll.objects:
        if obj.type == "ARMATURE":
            return obj
    for child in coll.children:
        found = _find_first_armature_recursive(child)
        if found is not None:
            return found
    return None


def _set_material_sim_transparency(mat) -> None:
    """Apply 50% alpha to a sim material. Uses multiple fallback approaches for Blender version compat."""
    if mat is None:
        return
    try:
        rgba = list(getattr(mat, "diffuse_color", (1.0, 1.0, 1.0, 1.0)))
        while len(rgba) < 4:
            rgba.append(1.0)
        rgba[3] = 0.5
        mat.diffuse_color = rgba
    except Exception:
        pass
    # Enable alpha blending so viewport/material preview can display transparency.
    for attr_name in ("blend_method", "surface_render_method"):
        try:
            if hasattr(mat, attr_name):
                setattr(mat, attr_name, "BLEND")
        except Exception:
            pass
    for attr_name, value in (
        ("shadow_method", "HASHED"),   # Eevee (older)
        ("transparent_shadow_method", "HASHED"),  # some versions
    ):
        try:
            if hasattr(mat, attr_name):
                setattr(mat, attr_name, value)
        except Exception:
            pass
    # Update shader-node alpha too (Eevee/Cycles) when using nodes.
    try:
        if getattr(mat, "use_nodes", False) and getattr(mat, "node_tree", None):
            for node in mat.node_tree.nodes:
                if getattr(node, "type", "") == "BSDF_PRINCIPLED":
                    alpha_input = node.inputs.get("Alpha") if hasattr(node, "inputs") else None
                    if alpha_input is not None:
                        alpha_input.default_value = 0.5
    except Exception:
        pass
    try:
        if hasattr(mat, "show_transparent_back"):
            mat.show_transparent_back = False
    except Exception:
        pass
    try:
        mat["animaquina_sim_material"] = True
    except Exception:
        pass


def _ensure_sim_collection_material_transparency(sim_coll) -> None:
    """
    Make all mesh materials in the sim collection 50% transparent without affecting
    source rig materials. Mesh data is already duplicated; materials are copied lazily here.
    """
    if sim_coll is None:
        return
    copied_cache = {}
    for obj in _iter_collection_objects_recursive(sim_coll):
        if getattr(obj, "type", None) != "MESH":
            continue
        # Viewport object color alpha as an extra hint in Solid mode.
        try:
            color = list(getattr(obj, "color", (1.0, 1.0, 1.0, 1.0)))
            while len(color) < 4:
                color.append(1.0)
            color[3] = 0.5
            obj.color = color
        except Exception:
            pass

        mats = getattr(getattr(obj, "data", None), "materials", None)
        if mats is None:
            continue
        for i in range(len(mats)):
            mat = mats[i]
            if mat is None:
                continue
            if bool(getattr(mat, "get", lambda *_: False)("animaquina_sim_material", False)):
                _set_material_sim_transparency(mat)
                continue
            if mat in copied_cache:
                sim_mat = copied_cache[mat]
            else:
                try:
                    sim_mat = mat.copy()
                except Exception as e:
                    _log(f"could not copy material '{mat.name}' for sim, sharing original: {e}")
                    sim_mat = mat
                _set_material_sim_transparency(sim_mat)
                copied_cache[mat] = sim_mat
            try:
                mats[i] = sim_mat
            except Exception as e:
                _log(f"could not assign sim material to '{obj.name}' slot {i}: {e}")


def ensure_sim_collection(slot):
    """
    Ensure slot.sim_collection exists. If missing, duplicate slot.rig_collection and link
    it to the scene root. Returns (sim_coll, error_message).
    """
    if slot.sim_collection and slot.sim_collection.name in bpy.data.collections:
        return bpy.data.collections[slot.sim_collection.name], ""
    if not slot.rig_collection:
        return None, "No rig collection set"
    src_name = slot.rig_collection.name
    if src_name not in bpy.data.collections:
        return None, f"Collection '{src_name}' not found"
    src_coll = bpy.data.collections[src_name]

    object_map = {}
    sim_coll = _duplicate_collection_recursive(src_coll, object_map)
    _relink_parenting(object_map)

    scene = bpy.context.scene
    if scene is None:
        return None, "No active scene"
    scene.collection.children.link(sim_coll)
    slot.sim_collection = sim_coll

    # Best-effort auto-pick duplicated armature as the sim rig target.
    sim_arm = _find_first_armature_recursive(sim_coll)
    if sim_arm is not None:
        try:
            slot.rig_armature = slot.rig_armature or sim_arm
        except Exception as e:
            _log(f"could not auto-assign sim armature to slot: {e}")
    return sim_coll, ""


def _remove_existing_sim_constraints(arm_obj) -> None:
    """Avoid stacking duplicate IK/CHILD_OF constraints on repeated Set Simulation."""
    if arm_obj is None or arm_obj.type != "ARMATURE":
        return
    joint_6 = arm_obj.pose.bones.get("joint_6")
    tcp_bone = arm_obj.pose.bones.get("tcp")
    if joint_6:
        for c in list(joint_6.constraints):
            if c.type == "IK":
                joint_6.constraints.remove(c)
    if tcp_bone:
        for c in list(tcp_bone.constraints):
            if c.type == "IK":
                tcp_bone.constraints.remove(c)
    if tcp_bone:
        for c in list(tcp_bone.constraints):
            if c.type == "CHILD_OF":
                tcp_bone.constraints.remove(c)


def _tcp_has_offset_or_rotation(arm_obj, pos_eps: float = 1e-6, rot_eps: float = 1e-4) -> bool:
    """
    Return True when tcp differs from the joint_6 flange frame by translation and/or rotation.
    If False, tcp is effectively neutral and should be ignored for Blender IK targeting.
    """
    if arm_obj is None or arm_obj.type != "ARMATURE" or getattr(arm_obj, "data", None) is None:
        return False
    try:
        arm = arm_obj.data
        j6 = arm.bones.get("joint_6")
        tcp = arm.bones.get("tcp")
        if j6 is None or tcp is None:
            return False
        # Offset from flange (joint_6 tail) to tcp head in armature space.
        pos_off = (tcp.head_local - j6.tail_local).length
        # Orientation delta between flange frame and tcp frame.
        rot_off = j6.matrix_local.to_quaternion().rotation_difference(
            tcp.matrix_local.to_quaternion()
        ).angle
        # Ignore tcp bone length — the default armature placeholder has a small
        # non-zero length (~0.1) that does not indicate a real tool.  Only a
        # positional offset from j6.tail or a rotation difference counts.
        return bool(pos_off > pos_eps or rot_off > rot_eps)
    except Exception:
        return False


def setup_simulation(slot, enable_blender_ik: bool = True) -> str:
    """
    Duplicate rig collection, rename twin→sim, apply current pose as rest, add IK.
    Expects slot.rig_collection (source) and slot.sim_collection (target to create/fill).
    """
    if not slot.rig_collection:
        return "No rig collection set"
    setup_profile = "BLENDER_IK" if enable_blender_ik else "NEWTON_IK"

    # If the existing sim collection was prepared for a different simulation mode,
    # rebuild it so Blender IK rest-pose baking does not leak into Newton playback.
    existing_sim = getattr(slot, "sim_collection", None)
    if existing_sim and existing_sim.name in bpy.data.collections:
        existing_coll = bpy.data.collections[existing_sim.name]
        existing_profile = str(existing_coll.get("animaquina_setup_profile", ""))
        if existing_profile and existing_profile != setup_profile:
            _log(f"sim profile changed ({existing_profile} → {setup_profile}), rebuilding sim collection")
            delete_sim_collection(slot)

    src_name = slot.rig_collection.name
    if src_name not in bpy.data.collections:
        return f"Collection '{src_name}' not found"
    _collection = bpy.data.collections[src_name]

    sim_coll, err = ensure_sim_collection(slot)
    if err:
        return err
    if sim_coll is None:
        return "Could not create simulation collection"
    _ensure_sim_collection_material_transparency(sim_coll)

    # Remove orientation helper empties BEFORE iterating sim_coll.objects,
    # because bpy.data.objects.remove() frees the StructRNA and would
    # invalidate any snapshot list that still holds a reference.
    for obj in list(sim_coll.objects):
        if obj.name.startswith("animaquina_ik_orient_helper"):
            try:
                bpy.data.objects.remove(obj, do_unlink=True)
            except Exception:
                pass

    for obj in list(sim_coll.objects):
        new_name = _sim_name(obj.name).replace(".001", "")
        if new_name != obj.name:
            obj.name = new_name
        if obj.type == "ARMATURE":
            for bone in obj.data.bones:
                bone.name = _sim_name(bone.name).replace(".001", "")
            for pb in obj.pose.bones:
                pb.name = _sim_name(pb.name).replace(".001", "")
            _apply_joint_ik_axis_locks(obj, slot)
            _configure_itasc_for_sim(obj)
            _remove_existing_sim_constraints(obj)
            joint_6 = obj.pose.bones.get("joint_6")
            tcp_bone = obj.pose.bones.get("tcp")
            ik_target_obj = (slot.target_object or slot.tcp_object) if enable_blender_ik else None

            # Determine tool mode and compute orientation correction
            # BEFORE armature_apply (FK pose is still active).
            # Also check offline tool frame: if the user set a tool offset
            # and it is non-zero, treat as having a tool.
            _tool_frame_active = (
                getattr(slot, "tool_frame_valid", False)
                and bool(tcp_bone)
                and (
                    any(abs(v) > 1e-6 for v in slot.tool_frame_pos_m)
                    or any(abs(v) > 1e-6 for v in slot.tool_frame_euler_deg)
                )
            )
            has_tool = getattr(slot, "has_tool", False) or (
                bool(tcp_bone) and _tcp_has_offset_or_rotation(obj)
            ) or _tool_frame_active
            # Compute orientation correction from the ORIGINAL rig
            # armature and tcp_object — same approach as mujoco_export.
            # C = j6_posed⁻¹ @ tcp_object  (in armature space)
            # This captures the constant frame convention difference
            # between the bone FK frame and the robot TCP frame.
            correction_quat = None
            src_arm = slot.rig_armature
            tcp_obj = slot.tcp_object
            if enable_blender_ik and src_arm is not None and tcp_obj is not None:
                j6_pb_src = src_arm.pose.bones.get("joint_6")
                if j6_pb_src is not None:
                    try:
                        arm_inv = src_arm.matrix_world.inverted()
                        tcp_arm_rot = (arm_inv @ tcp_obj.matrix_world).to_3x3()
                        j6_posed_rot = j6_pb_src.matrix.to_3x3()
                        site_rot = j6_posed_rot.inverted() @ tcp_arm_rot
                        correction_quat = site_rot.to_quaternion()
                    except Exception as e:
                        _log(f"could not compute orientation correction: {e}")

            if enable_blender_ik:
                bpy.context.view_layer.objects.active = obj
                bpy.ops.object.mode_set(mode="POSE")
                bpy.ops.pose.armature_apply()
                bpy.ops.object.mode_set(mode="OBJECT")

            # Apply offline tool offset to tcp edit bone so IK targets the
            # tool tip rather than the flange.  Tool frame values are in the
            # KUKA flange coordinate system (Z = tool direction).  In the
            # Blender bone, Y is along head→tail (= tool direction), so we
            # remap: KUKA (X,Y,Z) → bone-local (X, Z, -Y).
            # Tool rotation (A,B,C in KUKA = intrinsic ZYX, stored as raw
            # degrees) is converted and applied to the tcp bone direction.
            if enable_blender_ik and _tool_frame_active and tcp_bone:
                try:
                    bpy.ops.object.mode_set(mode="EDIT")
                    eb_j6 = obj.data.edit_bones.get("joint_6")
                    eb_tcp = obj.data.edit_bones.get("tcp")
                    if eb_j6 and eb_tcp:
                        tp = slot.tool_frame_pos_m
                        tool_bone = Vector((tp[0], tp[2], -tp[1]))
                        j6_rot = eb_j6.matrix.to_3x3()
                        tcp_pos = eb_j6.tail + j6_rot @ tool_bone
                        tcp_len = max((eb_tcp.tail - eb_tcp.head).length, 0.05)

                        # Tool rotation: stored as KUKA A,B,C degrees
                        # (intrinsic ZYX).  Build a rotation matrix in bone-
                        # local space, then transform the bone Y direction.
                        te = slot.tool_frame_euler_deg
                        a_r = math.radians(te[0])
                        b_r = math.radians(te[1])
                        c_r = math.radians(te[2])
                        if abs(a_r) > 1e-6 or abs(b_r) > 1e-6 or abs(c_r) > 1e-6:
                            # KUKA ABC intrinsic ZYX → rotation matrix,
                            # then remap axes the same way as position.
                            from mathutils import Quaternion
                            rz = Quaternion((0, 0, 1), a_r).to_matrix()
                            ry = Quaternion((0, 1, 0), b_r).to_matrix()
                            rx = Quaternion((1, 0, 0), c_r).to_matrix()
                            kuka_rot = rz @ ry @ rx  # intrinsic ZYX
                            # Remap KUKA frame axes to bone-local axes:
                            # bone_x=kuka_x, bone_y=kuka_z, bone_z=-kuka_y
                            # Swap matrix rows/cols accordingly.
                            # R_bone = P @ R_kuka @ P^T  where P swaps Y↔Z
                            # and negates new Z.
                            from mathutils import Matrix as M3
                            P = M3(((1, 0, 0), (0, 0, 1), (0, -1, 0)))
                            tool_rot_bone = P @ kuka_rot @ P.transposed()
                            tcp_dir = j6_rot @ (tool_rot_bone @ Vector((0, 1, 0)))
                        else:
                            tcp_dir = j6_rot @ Vector((0, 1, 0))

                        eb_tcp.head = tcp_pos
                        eb_tcp.tail = tcp_pos + tcp_dir.normalized() * tcp_len
                    bpy.ops.object.mode_set(mode="OBJECT")
                except Exception as e:
                    _log(f"could not apply tool offset to tcp bone: {e}")
                    try:
                        bpy.ops.object.mode_set(mode="OBJECT")
                    except Exception:
                        pass

            if enable_blender_ik:
                if joint_6 is None:
                    _log("joint_6 bone not found on sim armature — IK constraint not added")
                elif ik_target_obj is None:
                    _log("No target/tcp object set — IK constraint not added")
                else:
                    # Create orientation helper: parented to target with
                    # inverse correction so the IK bone frame matches the
                    # robot TCP frame convention.
                    actual_ik_target = ik_target_obj
                    if correction_quat is not None:
                        try:
                            helper = bpy.data.objects.new(
                                f"animaquina_ik_orient_helper_{obj.name}", None
                            )
                            sim_coll.objects.link(helper)
                            helper.parent = ik_target_obj
                            helper.matrix_parent_inverse = Matrix.Identity(4)
                            helper.rotation_mode = "QUATERNION"
                            helper.rotation_quaternion = correction_quat.inverted()
                            helper.empty_display_size = 0.01
                            actual_ik_target = helper
                        except Exception as e:
                            _log(f"could not create orientation helper: {e}")

                    if has_tool and tcp_bone:
                        # Tool present: IK on tcp so the solver drives the
                        # tool tip (tcp tail) to the target.  tcp IK axes
                        # are locked — only joints 1-6 rotate.
                        ik = tcp_bone.constraints.new(type="IK")
                        ik.target = actual_ik_target
                        ik.subtarget = ""
                        ik.use_rotation = True
                        ik.chain_count = 7
                        try:
                            ik.use_stretch = False
                        except Exception as e:
                            _log(f"could not set ik.use_stretch: {e}")
                        tcp_bone.lock_ik_x = True
                        tcp_bone.lock_ik_y = True
                        tcp_bone.lock_ik_z = True
                    else:
                        # No tool: IK on j6 — solver drives j6 tail
                        # (flange) directly to the target.
                        ik = joint_6.constraints.new(type="IK")
                        ik.target = actual_ik_target
                        ik.subtarget = ""
                        ik.use_rotation = True
                        ik.chain_count = 6
                        try:
                            ik.use_stretch = False
                        except Exception as e:
                            _log(f"could not set ik.use_stretch: {e}")
    try:
        sim_coll["animaquina_setup_profile"] = setup_profile
    except Exception as e:
        _log(f"could not write setup_profile to sim collection: {e}")
    return ""


def delete_sim_collection(slot) -> None:
    """Unlink and remove sim collection and its objects."""
    if not slot.sim_collection:
        return
    name = slot.sim_collection.name
    if name not in bpy.data.collections:
        return
    coll = bpy.data.collections[name]

    def _delete_coll_recursive(c):
        for child in list(c.children):
            _delete_coll_recursive(child)
        for obj in list(c.objects):
            try:
                bpy.data.objects.remove(obj, do_unlink=True)
            except Exception as e:
                _log(f"could not remove object '{obj.name}': {e}")
        try:
            bpy.data.collections.remove(c)
        except Exception as e:
            _log(f"could not remove collection '{c.name}': {e}")

    _delete_coll_recursive(coll)
    try:
        slot.sim_collection = None
    except Exception as e:
        _log(f"could not clear slot.sim_collection: {e}")
