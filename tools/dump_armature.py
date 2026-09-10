"""Dump full collection structure + armature bone data.

Run in Blender with a COLLECTION selected in the outliner,
or set COLLECTION_NAME below. Writes output to:
  <project>/tools/rig_dump.txt

Captures:
  - Full object tree (hierarchy, types, parent chains)
  - Mesh parent info (parent type, parent bone)
  - Empties (display type, location, rotation)
  - Armature bones (head, tail, roll, axes, matrix, IK limits)
  - Pose bone state (rotation mode, euler)
"""

import bpy
import os
import json
from mathutils import Vector, Euler

# ── Change this if you want to target a specific collection ─────────
COLLECTION_NAME = ""  # leave empty → uses active collection from outliner
# ────────────────────────────────────────────────────────────────────

OUTPUT_PATH = os.path.join(os.path.dirname(bpy.data.filepath) if bpy.data.filepath else os.path.expanduser("~"),
                           "rig_dump.txt")
# Override: always write next to this script if run from project
_script_dir = os.path.dirname(os.path.abspath(__file__)) if "__file__" in dir() else None
if _script_dir and os.path.isdir(_script_dir):
    OUTPUT_PATH = os.path.join(_script_dir, "rig_dump.txt")


def _r(v, n=6):
    """Round a value or iterable (handles Blender Vector/Euler/etc.)."""
    if hasattr(v, "to_tuple"):
        return [round(float(x), n) for x in v.to_tuple()]
    if isinstance(v, (list, tuple)):
        return [round(float(x), n) for x in v]
    try:
        return round(float(v), n)
    except (TypeError, ValueError):
        return str(v)


def _indent(depth):
    return "  " * depth


def dump_object(obj, lines, depth=0):
    """Recursively dump an object and its children."""
    prefix = _indent(depth)
    pinfo = ""
    if obj.parent:
        pinfo = f"  parent={obj.parent.name}"
        if obj.parent_type == "BONE":
            pinfo += f"  parent_bone={obj.parent_bone}"
        elif obj.parent_type != "OBJECT":
            pinfo += f"  parent_type={obj.parent_type}"

    lines.append(f"{prefix}[{obj.type}] {obj.name}{pinfo}")

    if obj.type == "EMPTY":
        lines.append(f"{prefix}  display: {obj.empty_display_type}  size: {obj.empty_display_size:.3f}")
        lines.append(f"{prefix}  location: {_r(obj.location)}")
        lines.append(f"{prefix}  rotation: {_r(obj.rotation_euler)}  mode: {obj.rotation_mode}")

    elif obj.type == "MESH":
        lines.append(f"{prefix}  location: {_r(obj.location)}")
        lines.append(f"{prefix}  rotation: {_r(obj.rotation_euler)}  mode: {obj.rotation_mode}")
        lines.append(f"{prefix}  scale: {_r(obj.scale, 4)}")
        if obj.data:
            lines.append(f"{prefix}  verts: {len(obj.data.vertices)}  faces: {len(obj.data.polygons)}")
        if obj.parent_type == "BONE":
            mat_inv = obj.matrix_parent_inverse
            for ri in range(4):
                lines.append(f"{prefix}  matrix_parent_inverse row{ri}: {_r(mat_inv[ri])}")
        for ri in range(4):
            lines.append(f"{prefix}  matrix_world row{ri}: {_r(obj.matrix_world[ri])}")

    elif obj.type == "ARMATURE":
        lines.append(f"{prefix}  location: {_r(obj.location)}")
        lines.append(f"{prefix}  rotation: {_r(obj.rotation_euler)}  mode: {obj.rotation_mode}")
        lines.append(f"{prefix}  display_type: {obj.data.display_type}")
        lines.append(f"{prefix}  show_in_front: {obj.show_in_front}")
        dump_armature_bones(obj, lines, depth + 1)

    # Recurse into children
    for child in sorted(obj.children, key=lambda o: o.name):
        dump_object(child, lines, depth + 1)


def dump_armature_bones(arm_obj, lines, depth):
    """Dump all bones from an armature."""
    arm_data = arm_obj.data
    prefix = _indent(depth)
    lines.append(f"{prefix}BONES ({len(arm_data.bones)}):")

    for bone in arm_data.bones:
        bp = _indent(depth + 1)
        lines.append(f"{bp}--- {bone.name} ---")
        lines.append(f"{bp}  head_local:  {_r(bone.head_local)}")
        lines.append(f"{bp}  tail_local:  {_r(bone.tail_local)}")
        lines.append(f"{bp}  length:      {_r(bone.length)}")
        lines.append(f"{bp}  parent:      {bone.parent.name if bone.parent else None}")
        lines.append(f"{bp}  connected:   {bone.use_connect}")

        y_axis = bone.matrix_local.to_3x3() @ Vector((0, 1, 0))
        z_axis = bone.matrix_local.to_3x3() @ Vector((0, 0, 1))
        x_axis = bone.matrix_local.to_3x3() @ Vector((1, 0, 0))
        lines.append(f"{bp}  x_axis:      {_r(x_axis, 4)}")
        lines.append(f"{bp}  y_axis:      {_r(y_axis, 4)}")
        lines.append(f"{bp}  z_axis:      {_r(z_axis, 4)}")

        lines.append(f"{bp}  matrix_local:")
        for row_i in range(4):
            lines.append(f"{bp}    {_r(bone.matrix_local[row_i])}")

        # Pose bone
        pb = arm_obj.pose.bones.get(bone.name)
        if pb:
            lines.append(f"{bp}  pose rotation_mode: {pb.rotation_mode}")
            lines.append(f"{bp}  pose rotation_euler: {_r(pb.rotation_euler)}")
            lines.append(f"{bp}  pose matrix_basis:")
            for row_i in range(4):
                lines.append(f"{bp}    {_r(pb.matrix_basis[row_i])}")

            ik_parts = []
            if pb.use_ik_limit_x:
                ik_parts.append(f"X=[{_r(pb.ik_min_x,4)}, {_r(pb.ik_max_x,4)}]")
            if pb.use_ik_limit_y:
                ik_parts.append(f"Y=[{_r(pb.ik_min_y,4)}, {_r(pb.ik_max_y,4)}]")
            if pb.use_ik_limit_z:
                ik_parts.append(f"Z=[{_r(pb.ik_min_z,4)}, {_r(pb.ik_max_z,4)}]")
            if ik_parts:
                lines.append(f"{bp}  ik_limits: {', '.join(ik_parts)}")

        # Custom props
        custom = {k: bone[k] for k in bone.keys() if not k.startswith("_RNA")}
        if custom:
            lines.append(f"{bp}  custom_props: {custom}")


def find_collection(name):
    """Find collection by name, or use active."""
    if name:
        coll = bpy.data.collections.get(name)
        if coll:
            return coll
    # Try active collection from view layer
    vl_coll = bpy.context.view_layer.active_layer_collection
    if vl_coll and vl_coll.collection != bpy.context.scene.collection:
        return vl_coll.collection
    return None


def dump_collection(coll, lines, depth=0):
    """Dump collection hierarchy."""
    prefix = _indent(depth)
    lines.append(f"{prefix}[COLLECTION] {coll.name}  ({len(coll.objects)} objects)")

    # Objects at this level (only root objects — no parent, or parent outside collection)
    coll_obj_names = {o.name for o in coll.objects}
    root_objs = [o for o in coll.objects
                 if o.parent is None or o.parent.name not in coll_obj_names]
    root_objs.sort(key=lambda o: o.name)

    for obj in root_objs:
        dump_object(obj, lines, depth + 1)

    # Child collections
    for child_coll in sorted(coll.children, key=lambda c: c.name):
        dump_collection(child_coll, lines, depth + 1)


# ── Main ────────────────────────────────────────────────────────────
coll = find_collection(COLLECTION_NAME)
if coll is None:
    msg = "No collection found! Either:\n  1. Set COLLECTION_NAME at the top of the script\n  2. Select a collection in the outliner before running"
    print(msg)
else:
    lines = []
    lines.append(f"RIG DUMP — {coll.name}")
    lines.append(f"Blender {bpy.app.version_string}")
    lines.append(f"File: {bpy.data.filepath or '(unsaved)'}")
    lines.append("=" * 70)
    dump_collection(coll, lines)
    lines.append("=" * 70)

    output = "\n".join(lines)

    # Write to Blender text editor
    txt_name = f"rig_dump_{coll.name}"
    if txt_name in bpy.data.texts:
        bpy.data.texts.remove(bpy.data.texts[txt_name])
    txt = bpy.data.texts.new(txt_name)
    txt.write(output)

    print(output)
    print(f"\nSaved to text block: {txt_name}")
