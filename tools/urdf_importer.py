# Animaquina Robot Importer — Developer Tool
# Run in Blender: Text Editor > Open > Run Script  (or install as addon via Edit > Preferences)
#
# Two import modes:
#   1. URDF  — full kinematic chain + mesh references from a .urdf file
#   2. DH XML — Robots-style DH parameters (a, d) from a .xml + a mesh folder
#
# Both create robots following Animaquina conventions:
#   blender_base (Empty)  >  j0 (base mesh)  >  armature  (joint_1 .. joint_6)
#   + tcp (Empty) at end-effector
#
# Auto-detects joint_axis_map.

bl_info = {
    "name": "Animaquina Robot Importer",
    "author": "Animaquina",
    "version": (0, 2, 0),
    "blender": (4, 0, 0),
    "location": "View3D > Sidebar > Robot Import",
    "description": "Import robots from URDF or DH-parameter XML (Robots format)",
    "category": "Import-Export",
}

import bpy
from bpy.props import StringProperty, PointerProperty, EnumProperty
from bpy.types import Operator, Panel, PropertyGroup
import os
import math
import re
import xml.etree.ElementTree as ET
from mathutils import Matrix, Vector, Euler, Quaternion


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  Shared constants
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

_MIN_BONE = 0.02          # 2 cm — minimum bone length
_MAX_JOINTS = 6           # Animaquina rig cap
_MESH_EXTS = {".stl", ".obj", ".dae", ".zae", ".ply", ".fbx"}


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  Xacro preprocessor (lightweight — covers common ROS xacro patterns)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _preprocess_xacro(filepath):
    """Expand a .xacro file into parseable URDF XML string.

    Handles the most common patterns:
      - ``${prefix}``  /  ``${package_name}``  → stripped (replaced with empty / dirname)
      - ``$(find pkg)`` → stripped to empty
      - ``$(arg name)`` → default from same-file xacro:arg, or empty
      - ``<xacro:include>`` of sibling files (one level, non-recursive)
      - ``<xacro:macro>`` blocks → their inner XML is kept
      - xacro namespace tags/attributes are removed so ET can parse the result
    """
    with open(filepath, "r", encoding="utf-8") as f:
        text = f.read()

    base_dir = os.path.dirname(os.path.abspath(filepath))

    # collect xacro:arg defaults from the file
    arg_defaults: dict[str, str] = {}
    for m in re.finditer(r'<xacro:arg\s+name="([^"]+)"\s+default="([^"]*)"', text):
        arg_defaults[m.group(1)] = m.group(2)

    # inline xacro:include files (siblings only)
    def _inline_include(match):
        fname_expr = match.group(1)
        # strip $(find ...) wrapper
        fname = re.sub(r'\$\(find\s+[^)]*\)/?', '', fname_expr).strip().strip('/')
        inc_path = os.path.join(base_dir, os.path.basename(fname))
        if not os.path.isfile(inc_path):
            inc_path = os.path.join(base_dir, fname)
        if os.path.isfile(inc_path):
            with open(inc_path, "r", encoding="utf-8") as f2:
                inc = f2.read()
            # strip XML declaration and outer <robot> tags from included file
            inc = re.sub(r'<\?xml[^?]*\?>', '', inc)
            inc = re.sub(r'<robot[^>]*>', '', inc)
            inc = re.sub(r'</robot\s*>', '', inc)
            return inc
        return ""

    text = re.sub(r'<xacro:include\s+filename="([^"]+)"\s*/?\s*>', _inline_include, text)

    # expand $(arg name) with defaults
    def _expand_arg(match):
        name = match.group(1)
        return arg_defaults.get(name, "")

    text = re.sub(r'\$\(arg\s+(\w+)\)', _expand_arg, text)

    # strip $(find ...) expressions
    text = re.sub(r'\$\(find\s+[^)]*\)', '', text)

    # strip ${prefix} and ${...} variables
    text = re.sub(r'\$\{prefix\}', '', text)
    text = re.sub(r'\$\{package_name\}', os.path.basename(base_dir), text)
    text = re.sub(r'\$\{[^}]*\}', '', text)

    # remove xacro-specific elements and attributes that would confuse ET
    # 1) unwrap <xacro:macro> — keep inner content, strip the wrapper tags
    text = re.sub(r'<xacro:macro[^>]*>', '', text)
    text = re.sub(r'</xacro:macro\s*>', '', text)

    # 2) iteratively strip remaining xacro-namespaced elements
    #    (self-closing first, then tag pairs with any nested content)
    for _ in range(5):
        prev = text
        text = re.sub(r'<xacro:\w+[^>]*/>', '', text)
        text = re.sub(r'<xacro:(\w+)[^>]*>.*?</xacro:\1\s*>', '', text, flags=re.DOTALL)
        if text == prev:
            break

    # 3) strip xmlns:xacro attribute
    text = re.sub(r'\s+xmlns:xacro="[^"]*"', '', text)

    return text


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  URDF Parser
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _parse_origin(elem):
    """<origin xyz="x y z" rpy="r p y"/> → 4×4 Matrix.
    URDF rpy = fixed-axis X Y Z rotation = Blender Euler('XYZ')."""
    if elem is None:
        return Matrix.Identity(4)
    xyz = elem.get("xyz", "0 0 0").split()
    rpy = elem.get("rpy", "0 0 0").split()
    t = Matrix.Translation((float(xyz[0]), float(xyz[1]), float(xyz[2])))
    r = Euler((float(rpy[0]), float(rpy[1]), float(rpy[2])), "XYZ").to_matrix().to_4x4()
    return t @ r


def _parse_urdf(filepath):
    """Parse a URDF or xacro file.

    For ``.xacro`` files a lightweight preprocessor expands variables,
    inlines sibling includes, and strips xacro namespace elements before
    parsing.  Returns  (robot_name, links_dict, joints_list).
    """
    if filepath.lower().endswith(".xacro"):
        xml_str = _preprocess_xacro(filepath)
        root = ET.fromstring(xml_str)
    else:
        tree = ET.parse(filepath)
        root = tree.getroot()
    robot_name = root.get("name", "robot")

    links: dict = {}
    for lel in root.findall("link"):
        name = lel.get("name")
        visuals = []
        for vel in lel.findall("visual"):
            origin = _parse_origin(vel.find("origin"))
            mesh_file = None
            scale = (1.0, 1.0, 1.0)
            gel = vel.find("geometry")
            if gel is not None:
                mel = gel.find("mesh")
                if mel is not None:
                    mesh_file = mel.get("filename")
                    sc = mel.get("scale")
                    if sc:
                        p = sc.split()
                        sx = float(p[0])
                        sy = float(p[1]) if len(p) > 1 else sx
                        sz = float(p[2]) if len(p) > 2 else sx
                        scale = (sx, sy, sz)
            visuals.append({"origin": origin, "mesh_file": mesh_file, "scale": scale})
        links[name] = {"visuals": visuals}

    joints: list = []
    for jel in root.findall("joint"):
        jname = jel.get("name")
        jtype = jel.get("type", "fixed")
        parent = jel.find("parent").get("link")
        child = jel.find("child").get("link")
        origin = _parse_origin(jel.find("origin"))

        ael = jel.find("axis")
        if ael is not None:
            ax = tuple(float(v) for v in ael.get("xyz", "0 0 1").split())
        else:
            ax = (0.0, 0.0, 1.0)

        lel2 = jel.find("limit")
        limits = None
        if lel2 is not None:
            lo = float(lel2.get("lower", "0"))
            hi = float(lel2.get("upper", "0"))
            if lo != 0.0 or hi != 0.0:
                limits = (lo, hi)

        joints.append({
            "name": jname, "type": jtype,
            "parent": parent, "child": child,
            "origin": origin, "axis": ax, "limits": limits,
        })

    return robot_name, links, joints


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  DH Parameter XML Parser  (Robots / visose format)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

# Standard DH alpha presets (radians) per manufacturer.
# These complete the DH table when the XML only provides a and d.
_DH_ALPHA = {
    "UR": [math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0],
    "KUKA": [-math.pi / 2, 0.0, math.pi / 2, -math.pi / 2, math.pi / 2, 0.0],
    "ABB": [-math.pi / 2, 0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, 0.0],
    "Staubli": [-math.pi / 2, 0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, 0.0],
    "Stäubli": [-math.pi / 2, 0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, 0.0],
    "Fanuc": [-math.pi / 2, 0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, 0.0],
}

# Fallback for unknown manufacturers (common anthropomorphic 6R)
_DH_ALPHA_DEFAULT = [-math.pi / 2, 0.0, math.pi / 2, -math.pi / 2, math.pi / 2, 0.0]


def _dh_rest_tf(a_m, d_m, alpha):
    """Standard DH transform at rest (theta=0):  Rz(0) @ Tz(d) @ Tx(a) @ Rx(alpha).

    Parameters in **meters**.  Returns 4×4 Matrix.
    """
    ca, sa = math.cos(alpha), math.sin(alpha)
    return Matrix((
        (1.0, 0.0, 0.0, a_m),
        (0.0, ca, -sa, 0.0),
        (0.0, sa, ca, d_m),
        (0.0, 0.0, 0.0, 1.0),
    ))


def _parse_robots_xml(filepath):
    """Parse a Robots-format DH XML.

    Returns dict with keys:
        name, manufacturer, model, payload,
        base_tf (4×4 Matrix from <Base>),
        joints: list[{a_mm, d_mm, minrange_deg, maxrange_deg, maxspeed}]
    """
    tree = ET.parse(filepath)
    root = tree.getroot()

    # Handle both <RobotSystems> wrapper and bare <RobotCell>
    cell = root.find(".//RobotCell")
    if cell is None:
        cell = root
    arm = cell.find(".//RobotArm")
    if arm is None:
        raise ValueError("No <RobotArm> found in XML.")

    name = cell.get("name", arm.get("model", "robot"))
    manufacturer = arm.get("manufacturer", cell.get("manufacturer", ""))
    model = arm.get("model", name)
    payload = arm.get("payload", "")

    # Base frame
    base_el = arm.find("Base")
    if base_el is not None:
        bx = float(base_el.get("x", "0"))
        by = float(base_el.get("y", "0"))
        bz = float(base_el.get("z", "0"))
        qw = float(base_el.get("q1", "1"))
        qx = float(base_el.get("q2", "0"))
        qy = float(base_el.get("q3", "0"))
        qz = float(base_el.get("q4", "0"))
        base_tf = Matrix.Translation((bx * 0.001, by * 0.001, bz * 0.001))
        base_tf = base_tf @ Quaternion((qw, qx, qy, qz)).to_matrix().to_4x4()
    else:
        base_tf = Matrix.Identity(4)

    # Joints
    joints_el = arm.find("Joints")
    if joints_el is None:
        raise ValueError("No <Joints> element found in XML.")

    jlist = []
    for rev in joints_el.findall("Revolute"):
        jlist.append({
            "number": int(rev.get("number", len(jlist) + 1)),
            "a_mm": float(rev.get("a", "0")),
            "d_mm": float(rev.get("d", "0")),
            "alpha": float(rev.get("alpha", "NaN")),
            "minrange_deg": float(rev.get("minrange", "-360")),
            "maxrange_deg": float(rev.get("maxrange", "360")),
            "maxspeed": float(rev.get("maxspeed", "180")),
        })
    jlist.sort(key=lambda j: j["number"])

    return {
        "name": name, "manufacturer": manufacturer, "model": model,
        "payload": payload, "base_tf": base_tf, "joints": jlist,
    }


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  Mesh-path resolution (URDF)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _resolve_mesh(ref: str | None, urdf_dir: str) -> str | None:
    """Resolve a URDF mesh filename to an absolute path.

    Search strategy (first match wins):
      1. Exact relative / absolute / file:// path
      2. package:// → strip prefix, try relative to urdf_dir and parents
      3. Strip package-name component and retry
      4. **Filename-only fallback** — search urdf_dir and one level of sub-dirs
         for a file whose name matches (handles reorganised mesh folders)
    """
    if ref is None:
        return None
    if ref.startswith("file://"):
        p = ref[7:]
        if os.path.isfile(p):
            return p

    basename = os.path.basename(ref)
    bases = [urdf_dir, os.path.dirname(urdf_dir), os.path.dirname(os.path.dirname(urdf_dir))]

    if ref.startswith("package://"):
        rel = ref[len("package://"):]
        for b in bases:
            c = os.path.join(b, rel)
            if os.path.isfile(c):
                return os.path.normpath(c)
        parts = rel.split("/", 1)
        if len(parts) > 1:
            for b in bases:
                c = os.path.join(b, parts[1])
                if os.path.isfile(c):
                    return os.path.normpath(c)
    elif not ref.startswith("file://"):
        c = os.path.normpath(os.path.join(urdf_dir, ref))
        if os.path.isfile(c):
            return c

    # filename-only fallback: search urdf_dir tree (1 level of subdirs)
    for search_dir in bases:
        if not os.path.isdir(search_dir):
            continue
        candidate = os.path.join(search_dir, basename)
        if os.path.isfile(candidate):
            return os.path.normpath(candidate)
        for sub in os.listdir(search_dir):
            sub_path = os.path.join(search_dir, sub)
            if os.path.isdir(sub_path):
                candidate = os.path.join(sub_path, basename)
                if os.path.isfile(candidate):
                    return os.path.normpath(candidate)
    return None


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  Mesh folder collector (DH import)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _natural_sort_key(s):
    """Sort key that orders 'link_2' before 'link_10'."""
    return [int(c) if c.isdigit() else c.lower() for c in re.split(r'(\d+)', s)]


def _collect_mesh_folder(folder):
    """Return mesh file paths sorted naturally (link_0 < link_1 < ... < link_10).

    First file → base/j0, then one per joint in order.
    """
    if not folder or not os.path.isdir(folder):
        return []
    files = []
    for f in os.listdir(folder):
        if os.path.splitext(f)[1].lower() in _MESH_EXTS:
            files.append(os.path.join(folder, f))
    files.sort(key=lambda p: _natural_sort_key(os.path.basename(p)))
    return files


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  URDF kinematic chain builder
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _build_chain(links, joints):
    """Walk the URDF joint tree from root link to tip.

    Returns (root_link, rev_chain, link_tfs).
    """
    parent_set = {j["parent"] for j in joints}
    child_set = {j["child"] for j in joints}
    roots = parent_set - child_set

    root_link = "base_link"
    if roots:
        root_link = sorted(roots)[0]
        if "world" in roots:
            for j in joints:
                if j["parent"] == "world":
                    root_link = j["child"]
                    break

    p2j: dict[str, list] = {}
    for j in joints:
        p2j.setdefault(j["parent"], []).append(j)

    link_tfs: dict[str, Matrix] = {root_link: Matrix.Identity(4)}
    rev_chain: list = []
    visited: set = set()

    def _walk(link):
        if link in visited or link not in p2j:
            return
        visited.add(link)
        for j in p2j[link]:
            child_tf = link_tfs[link] @ j["origin"]
            link_tfs[j["child"]] = child_tf
            if j["type"] in ("revolute", "continuous"):
                j["cumulative_tf"] = child_tf.copy()
                rev_chain.append(j)
            _walk(j["child"])

    _walk(root_link)
    if root_link == "world":
        for j in joints:
            if j["parent"] == "world" and j["child"] not in visited:
                link_tfs.setdefault(j["child"], j["origin"].copy())
                _walk(j["child"])

    return root_link, rev_chain, link_tfs


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  Mesh file importer (shared)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

_import_warnings: list[str] = []


def _import_mesh(filepath: str, name: str):
    """Import STL / OBJ / DAE → Blender mesh object (or None).

    Resets selection state before import to avoid context issues.
    Collada imports that produce armatures/empties are handled: only MESH
    objects are kept, extras are removed.
    """
    ext = os.path.splitext(filepath)[1].lower()

    # Ensure clean selection state — some importers behave badly
    # if an armature is active or we're in a weird mode.
    try:
        bpy.ops.object.mode_set(mode="OBJECT")
    except RuntimeError:
        pass
    bpy.ops.object.select_all(action="DESELECT")

    before = set(bpy.data.objects)
    try:
        if ext == ".stl":
            if bpy.app.version >= (4, 0, 0):
                bpy.ops.wm.stl_import(filepath=filepath)
            else:
                bpy.ops.import_mesh.stl(filepath=filepath)
        elif ext == ".obj":
            bpy.ops.wm.obj_import(filepath=filepath)
        elif ext in (".dae", ".zae"):
            try:
                bpy.ops.wm.collada_import(filepath=filepath)
            except (RuntimeError, AttributeError):
                _import_warnings.append(
                    f"DAE not supported in Blender {bpy.app.version_string} — "
                    f"convert {os.path.basename(filepath)} to STL/OBJ/FBX")
                return None
        elif ext == ".ply":
            bpy.ops.wm.ply_import(filepath=filepath)
        elif ext == ".fbx":
            bpy.ops.import_scene.fbx(filepath=filepath)
        else:
            _import_warnings.append(f"Unsupported format: {ext}")
            return None
    except Exception as exc:
        _import_warnings.append(f"Import failed: {os.path.basename(filepath)} — {exc}")
        print(f"[Importer] Import failed: {filepath} — {exc}")
        return None

    all_new = [o for o in bpy.data.objects if o not in before]
    new_meshes = [o for o in all_new if o.type == "MESH"]

    if not new_meshes:
        types = [o.type for o in all_new]
        if all_new:
            _import_warnings.append(f"{os.path.basename(filepath)}: imported {len(all_new)} objects but no meshes (types: {types})")
        else:
            _import_warnings.append(f"{os.path.basename(filepath)}: importer produced no objects")
        # Clean up non-mesh leftovers from Collada (armatures, empties, etc.)
        for o in all_new:
            bpy.data.objects.remove(o, do_unlink=True)
        return None

    # Remove non-mesh leftovers (Collada often creates empties/armatures)
    for o in all_new:
        if o.type != "MESH":
            bpy.data.objects.remove(o, do_unlink=True)

    if len(new_meshes) > 1:
        bpy.ops.object.select_all(action="DESELECT")
        for o in new_meshes:
            o.select_set(True)
        bpy.context.view_layer.objects.active = new_meshes[0]
        bpy.ops.object.join()
        result = bpy.context.active_object
    else:
        result = new_meshes[0]

    result.name = name
    if result.data:
        result.data.name = name
    return result


def _move_to_collection(obj, coll):
    """Ensure *obj* lives only in *coll*."""
    for c in list(obj.users_collection):
        c.objects.unlink(obj)
    if obj.name not in coll.objects:
        coll.objects.link(obj)


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  Axis-map derivation (shared)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _axis_map_entry(bone, axis_armature):
    """Map an armature-space axis vector to the Animaquina axis string."""
    local = bone.matrix_local.to_3x3().inverted() @ axis_armature
    comps = (abs(local.x), abs(local.y), abs(local.z))
    idx = comps.index(max(comps))
    signs = (local.x, local.y, local.z)
    labels = ("X", "Y", "Z")
    return f"-{labels[idx]}" if signs[idx] < 0 else labels[idx]


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  Armature + TCP builder (shared)
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _build_armature(context, coll, j0, robot_name, joint_heads, joint_axes,
                    joint_limits, tcp_pos):
    """Create armature with joint_1..joint_N bones following Animaquina conventions.

    Matched against manually-built rigs (e.g. kr10r1100-2_twin in robots.blend):
      - Hierarchy: blender_base (ARROWS Empty) > j0 (mesh) > armature
      - Armature display: WIRE, show_in_front=False
      - Bones: joint_1 .. joint_6, parent chain; use_connect when head==parent tail
      - tcp bone: child of joint_6, QUATERNION mode, length 0.1
      - Pose bones: rotation_mode = 'XYZ'
      - TCP object: ARROWS Empty named '{robot_name}_tcp', not parented

    Returns (arm_obj, axis_map_list, tcp_obj)
    """
    n = len(joint_heads)
    _CONNECT_TOL = 1e-4

    arm_data = bpy.data.armatures.new(f"{robot_name}_rig")
    arm_obj = bpy.data.objects.new(robot_name, arm_data)
    coll.objects.link(arm_obj)
    arm_obj.parent = j0
    arm_obj.show_in_front = False
    arm_data.display_type = "WIRE"

    context.view_layer.objects.active = arm_obj
    bpy.ops.object.select_all(action="DESELECT")
    arm_obj.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")

    for i in range(n):
        eb = arm_data.edit_bones.new(f"joint_{i + 1}")
        head = joint_heads[i].copy()

        if i + 1 < n:
            tail = joint_heads[i + 1].copy()
        else:
            tail = tcp_pos.copy()

        if (tail - head).length < _MIN_BONE:
            tail = head + joint_axes[i].normalized() * 0.08

        eb.head = head
        eb.tail = tail

        if i > 0:
            parent_eb = arm_data.edit_bones[f"joint_{i}"]
            eb.parent = parent_eb
            eb.use_connect = (eb.head - parent_eb.tail).length < _CONNECT_TOL
        else:
            eb.use_connect = False

        if joint_limits[i] is not None:
            eb["_urdf_limit_lo"] = joint_limits[i][0]
            eb["_urdf_limit_hi"] = joint_limits[i][1]

    # tcp bone inside armature — child of last joint, QUATERNION, length 0.1
    last_bone_name = f"joint_{n}"
    tcp_eb = arm_data.edit_bones.new("tcp")
    tcp_eb.head = tcp_pos.copy()
    tcp_dir = (tcp_pos - joint_heads[-1]).normalized() if (tcp_pos - joint_heads[-1]).length > 1e-6 else Vector((1, 0, 0))
    tcp_eb.tail = tcp_pos + tcp_dir * 0.1
    tcp_eb.parent = arm_data.edit_bones[last_bone_name]
    tcp_eb.use_connect = False

    bpy.ops.object.mode_set(mode="OBJECT")

    # Pose bone setup: rotation_mode + axis map + IK limits
    axis_map: list[str] = []
    for i in range(n):
        bone = arm_data.bones[f"joint_{i + 1}"]
        pbone = arm_obj.pose.bones[f"joint_{i + 1}"]

        pbone.rotation_mode = "XYZ"

        entry = _axis_map_entry(bone, joint_axes[i].normalized())
        axis_map.append(entry)

        lo = bone.get("_urdf_limit_lo")
        hi = bone.get("_urdf_limit_hi")
        if lo is not None and hi is not None:
            ax_letter = entry.lstrip("-")
            negate = entry.startswith("-")
            real_lo, real_hi = (-hi, -lo) if negate else (lo, hi)
            if ax_letter == "X":
                pbone.use_ik_limit_x = True
                pbone.ik_min_x = real_lo
                pbone.ik_max_x = real_hi
            elif ax_letter == "Y":
                pbone.use_ik_limit_y = True
                pbone.ik_min_y = real_lo
                pbone.ik_max_y = real_hi
            else:
                pbone.use_ik_limit_z = True
                pbone.ik_min_z = real_lo
                pbone.ik_max_z = real_hi

    # tcp bone: QUATERNION rotation mode
    tcp_pbone = arm_obj.pose.bones.get("tcp")
    if tcp_pbone:
        tcp_pbone.rotation_mode = "QUATERNION"

    # TCP empty — ARROWS, named for auto-discovery by _find_tcp_in_collection
    tcp = bpy.data.objects.new(f"{robot_name}_tcp", None)
    tcp.empty_display_type = "ARROWS"
    tcp.empty_display_size = 0.1
    coll.objects.link(tcp)

    return arm_obj, axis_map, tcp


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  URDF import
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

_MESH_PRIORITY = (".stl", ".obj", ".fbx", ".ply", ".dae", ".zae")


def _match_mesh_by_name(link_name, mesh_folder):
    """Find a mesh file in *mesh_folder* whose stem matches *link_name*.

    Prioritises STL > OBJ > FBX > PLY > DAE so Blender 5.x (no Collada) works.
    """
    if not mesh_folder or not os.path.isdir(mesh_folder):
        return None
    target = link_name.lower()
    candidates = {}
    for f in os.listdir(mesh_folder):
        stem, ext = os.path.splitext(f)
        if ext.lower() in _MESH_EXTS and stem.lower() == target:
            candidates[ext.lower()] = os.path.join(mesh_folder, f)
    for ext in _MESH_PRIORITY:
        if ext in candidates:
            return candidates[ext]
    return None


def _do_import_urdf(context, filepath, mesh_folder=""):
    """Parse a URDF and build the full Animaquina-convention rig.

    Meshes are matched by **link name** → filename in *mesh_folder*
    (e.g. link named ``base_link`` matches ``base_link.dae``).
    """
    _import_warnings.clear()
    mesh_folder = mesh_folder.strip()
    if mesh_folder and not os.path.isdir(mesh_folder):
        mesh_folder = ""

    try:
        robot_name, links, joints = _parse_urdf(filepath)
    except ET.ParseError as e:
        return {"CANCELLED"}, f"XML parse error: {e}"
    except Exception as e:
        return {"CANCELLED"}, f"Failed to parse file: {e}"

    root_link, rev_chain, link_tfs = _build_chain(links, joints)
    if not rev_chain:
        return {"CANCELLED"}, "No revolute joints found in URDF."

    n = min(len(rev_chain), _MAX_JOINTS)
    rev_chain = rev_chain[:n]

    # ── collection + base ──────────────────────────────────────────────────
    coll = bpy.data.collections.new(robot_name)
    context.scene.collection.children.link(coll)

    blender_base = bpy.data.objects.new(f"{robot_name}_base", None)
    blender_base.empty_display_type = "ARROWS"
    blender_base.empty_display_size = 0.1
    coll.objects.link(blender_base)

    # ── j0 (base link mesh) ───────────────────────────────────────────────
    j0 = None
    mp = _match_mesh_by_name(root_link, mesh_folder)
    if mp:
        obj = _import_mesh(mp, f"{robot_name}_j0")
        if obj is not None:
            # Apply URDF visual origin (CAD frame → link frame) + link transform
            vis_origin = Matrix.Identity(4)
            if root_link in links and links[root_link]["visuals"]:
                vis_origin = links[root_link]["visuals"][0]["origin"]
            link_tf = link_tfs.get(root_link, Matrix.Identity(4))
            obj.matrix_world = link_tf @ vis_origin
            _move_to_collection(obj, coll)
            j0 = obj

    if j0 is None:
        j0 = bpy.data.objects.new(f"{robot_name}_j0", None)
        j0.empty_display_type = "CUBE"
        j0.empty_display_size = 0.05
        coll.objects.link(j0)
    j0.parent = blender_base

    # ── armature via shared builder ────────────────────────────────────────
    heads = [jinfo["cumulative_tf"].translation.copy() for jinfo in rev_chain]
    axes = [(jinfo["cumulative_tf"].to_3x3() @ Vector(jinfo["axis"])).normalized()
            for jinfo in rev_chain]
    limits = [jinfo["limits"] for jinfo in rev_chain]

    # TCP: walk fixed joints past last revolute
    last_child = rev_chain[-1]["child"]
    ee_tf = link_tfs.get(last_child, rev_chain[-1]["cumulative_tf"])
    child_fixed = {}
    for j in joints:
        if j["type"] == "fixed":
            child_fixed.setdefault(j["parent"], []).append(j)
    walk = last_child
    for _ in range(10):
        if walk in child_fixed:
            fj = child_fixed[walk][0]
            if fj["child"] in link_tfs:
                ee_tf = link_tfs[fj["child"]]
            walk = fj["child"]
        else:
            break
    tcp_pos = ee_tf.translation.copy()

    # Clean bone positions: project into XZ plane (Y=0) and start joint_1
    # from the armature origin.  This matches the manual-rig convention
    # (all bones in XZ, first joint at base).  Axes are already computed from
    # the original URDF cumulative transforms and remain correct.
    if heads:
        heads[0] = Vector((0, 0, 0))
    for h in heads:
        h.y = 0.0
    tcp_pos.y = 0.0

    arm_obj, axis_map, tcp = _build_armature(
        context, coll, j0, robot_name, heads, axes, limits, tcp_pos)
    arm_data = arm_obj.data

    # Flush bone matrices before parenting meshes
    context.view_layer.update()

    # ── link meshes → parent to bones (matched by link name) ─────────────
    # Position each mesh at the bone HEAD (cleaned XZ-plane position).
    # Rotation comes from the URDF: (link_tf @ visual_origin).to_3x3(),
    # which orients the CAD geometry correctly in world space.
    mesh_miss: list[str] = []
    mesh_ok = 0
    for i, jinfo in enumerate(rev_chain):
        child_link = jinfo["child"]
        bone_name = f"joint_{i + 1}"

        mp = _match_mesh_by_name(child_link, mesh_folder)
        if not mp:
            mesh_miss.append(child_link)
            continue
        obj = _import_mesh(mp, f"{robot_name}_j{i + 1}")
        if obj is None:
            continue

        vis_origin = Matrix.Identity(4)
        if child_link in links and links[child_link]["visuals"]:
            vis_origin = links[child_link]["visuals"][0]["origin"]
        link_tf = link_tfs.get(child_link, Matrix.Identity(4))

        urdf_rot = (link_tf @ vis_origin).to_3x3().to_4x4()
        bone_head_pos = heads[i]
        mesh_world = Matrix.Translation(bone_head_pos) @ urdf_rot

        _move_to_collection(obj, coll)
        obj.matrix_world = mesh_world
        context.view_layer.update()

        # Parent to bone — operator handles matrix_parent_inverse correctly
        bpy.ops.object.select_all(action="DESELECT")
        obj.select_set(True)
        arm_obj.select_set(True)
        context.view_layer.objects.active = arm_obj
        arm_data.bones.active = arm_data.bones[bone_name]
        try:
            bpy.ops.object.parent_set(type="BONE", keep_transform=True)
        except RuntimeError as e:
            _import_warnings.append(f"Parenting {obj.name} → {bone_name}: {e}")

        mesh_ok += 1

    # ── report ─────────────────────────────────────────────────────────────
    map_str = ", ".join(axis_map)
    msg_lines = [
        f"Robot: {robot_name}  ({n} joints, URDF)",
        f"Axis map: [{map_str}]",
        f"Hierarchy: {blender_base.name} > {j0.name} > {arm_obj.name}",
        f"TCP: {tcp.name}  pos {tuple(round(v, 4) for v in tcp_pos)}",
        f"Meshes: {mesh_ok} imported" + (f", {len(mesh_miss)} missing" if mesh_miss else ""),
    ]
    total_rev = sum(1 for j in joints if j["type"] in ("revolute", "continuous"))
    if total_rev > _MAX_JOINTS:
        msg_lines.append(f"WARNING: {total_rev - n} revolute joint(s) beyond 6 skipped")
    if mesh_miss:
        msg_lines.append(f"No mesh for links: {', '.join(mesh_miss)}")
        if mesh_folder:
            avail = [os.path.splitext(f)[0] for f in os.listdir(mesh_folder)
                     if os.path.splitext(f)[1].lower() in _MESH_EXTS]
            msg_lines.append(f"  Available in folder: {', '.join(sorted(avail))}")
    if not mesh_folder:
        msg_lines.append("No mesh folder selected — skeleton only")
    if _import_warnings:
        for w in _import_warnings:
            msg_lines.append(f"WARNING: {w}")

    msg = "\n".join(msg_lines)
    _print_report(msg)
    _select(context, arm_obj)
    return {"FINISHED"}, msg


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  DH XML import
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _do_import_dh(context, xml_path, mesh_folder):
    """Parse a Robots-format DH XML and build the Animaquina rig.

    Mesh folder:  sorted mesh files → first = j0, then one per joint.
    """
    _import_warnings.clear()
    data = _parse_robots_xml(xml_path)
    robot_name = data["name"]
    manufacturer = data["manufacturer"].upper().strip()
    dh_joints = data["joints"]
    n = min(len(dh_joints), _MAX_JOINTS)
    dh_joints = dh_joints[:n]

    # resolve alpha values
    alpha_preset = _DH_ALPHA.get(manufacturer)
    alpha_warn = ""
    if alpha_preset is None:
        alpha_preset = _DH_ALPHA_DEFAULT
        if manufacturer:
            alpha_warn = f"WARNING: No DH alpha preset for '{manufacturer}', using generic anthropomorphic"
        else:
            alpha_warn = "WARNING: No manufacturer in XML, using generic DH alpha"

    # compute cumulative DH transforms (rest pose, theta=0)
    # frame[i] = T_1 @ T_2 @ ... @ T_i  (in base coordinates)
    # Joint i rotates at frame[i-1] origin about frame[i-1] Z axis.
    frames = [Matrix.Identity(4)]  # frame[0] = base
    for i, jd in enumerate(dh_joints):
        a_m = jd["a_mm"] * 0.001
        d_m = jd["d_mm"] * 0.001
        alpha_i = jd["alpha"] if not math.isnan(jd["alpha"]) else alpha_preset[i] if i < len(alpha_preset) else 0.0
        frames.append(frames[-1] @ _dh_rest_tf(a_m, d_m, alpha_i))

    # joint i acts at frame[i-1], axis = frame[i-1] Z
    joint_heads = [frames[i].translation.copy() for i in range(n)]  # frame 0..n-1
    joint_axes = [(frames[i].to_3x3() @ Vector((0, 0, 1))).normalized() for i in range(n)]
    joint_limits = []
    for jd in dh_joints:
        lo = math.radians(jd["minrange_deg"])
        hi = math.radians(jd["maxrange_deg"])
        joint_limits.append((lo, hi) if (lo != 0.0 or hi != 0.0) else None)

    tcp_pos = frames[n].translation.copy()

    # ── collection + base ──────────────────────────────────────────────────
    coll = bpy.data.collections.new(robot_name)
    context.scene.collection.children.link(coll)

    blender_base = bpy.data.objects.new(f"{robot_name}_base", None)
    blender_base.empty_display_type = "ARROWS"
    blender_base.empty_display_size = 0.1
    coll.objects.link(blender_base)

    # ── meshes from folder ─────────────────────────────────────────────────
    mesh_files = _collect_mesh_folder(mesh_folder)

    # j0: first mesh, or empty fallback
    j0 = None
    if mesh_files:
        obj = _import_mesh(mesh_files[0], f"{robot_name}_j0")
        if obj is not None:
            _move_to_collection(obj, coll)
            j0 = obj
    if j0 is None:
        j0 = bpy.data.objects.new(f"{robot_name}_j0", None)
        j0.empty_display_type = "CUBE"
        j0.empty_display_size = 0.05
        coll.objects.link(j0)
    j0.parent = blender_base

    # ── armature ───────────────────────────────────────────────────────────
    arm_obj, axis_map, tcp = _build_armature(
        context, coll, j0, robot_name, joint_heads, joint_axes, joint_limits, tcp_pos)
    arm_data = arm_obj.data

    # ── parent remaining meshes to bones ───────────────────────────────────
    # mesh_files[1] → joint_1, mesh_files[2] → joint_2, etc.
    for i in range(n):
        mi = i + 1  # mesh index (0 was j0)
        if mi >= len(mesh_files):
            break
        bone_name = f"joint_{i + 1}"
        bone_rest = arm_data.bones[bone_name]
        parent_effect = arm_obj.matrix_world @ bone_rest.matrix_local

        fname = os.path.basename(mesh_files[mi])
        obj = _import_mesh(mesh_files[mi], os.path.splitext(fname)[0])
        if obj is None:
            continue

        # The mesh is assumed to be in the link's local frame.
        # Place it at the link frame (= frames[i+1]) in world, then bone-parent.
        desired = frames[i + 1]

        _move_to_collection(obj, coll)
        obj.parent = arm_obj
        obj.parent_type = "BONE"
        obj.parent_bone = bone_name
        obj.matrix_parent_inverse = parent_effect.inverted() @ desired
        obj.matrix_basis = Matrix.Identity(4)

    # ── report ─────────────────────────────────────────────────────────────
    map_str = ", ".join(axis_map)
    alpha_src = f"preset ({manufacturer})" if manufacturer in _DH_ALPHA else "generic default"
    msg_lines = [
        f"Robot: {robot_name}  ({n} joints, DH XML)",
        f"Manufacturer: {data['manufacturer']}  Model: {data['model']}",
        f"DH alpha source: {alpha_src}",
        f"Axis map: [{map_str}]",
        f"Hierarchy: {blender_base.name} > {j0.name} > {arm_obj.name}",
        f"TCP: {tcp.name}  pos {tuple(round(v, 4) for v in tcp_pos)}",
        f"Meshes found: {len(mesh_files)} files in folder",
    ]
    if alpha_warn:
        msg_lines.append(alpha_warn)
    if len(dh_joints) > _MAX_JOINTS:
        msg_lines.append(f"WARNING: {len(data['joints']) - n} joint(s) beyond 6 skipped")

    msg = "\n".join(msg_lines)
    _print_report(msg)
    _select(context, arm_obj)
    return {"FINISHED"}, msg


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  Helpers
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _print_report(msg):
    print("\n" + "=" * 64)
    print(msg)
    print("=" * 64 + "\n")


def _select(context, obj):
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    context.view_layer.objects.active = obj


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  Operators — URDF
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

class URDF_OT_FileBrowser(Operator):
    """Open a file browser and import a URDF / xacro file"""
    bl_idname = "import_robot.urdf"
    bl_label = "Import URDF"
    bl_options = {"REGISTER", "UNDO"}

    filepath: StringProperty(subtype="FILE_PATH")
    filter_glob: StringProperty(default="*.urdf;*.URDF;*.xacro;*.XACRO", options={"HIDDEN"})

    def execute(self, context):
        if not self.filepath or not os.path.isfile(self.filepath):
            self.report({"ERROR"}, "Select a valid URDF / xacro file.")
            return {"CANCELLED"}
        props = context.scene.robot_importer
        props.urdf_filepath = self.filepath
        mf = bpy.path.abspath(props.urdf_mesh_folder)
        result, msg = _do_import_urdf(context, self.filepath, mf)
        props.last_result = msg
        for line in msg.split("\n"):
            self.report({"INFO"} if result == {"FINISHED"} else {"ERROR"}, line)
        return result

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {"RUNNING_MODAL"}


class URDF_OT_ImportPanel(Operator):
    """Import the URDF from the paths set in the panel"""
    bl_idname = "import_robot.urdf_panel"
    bl_label = "Import URDF"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        props = context.scene.robot_importer
        fp = bpy.path.abspath(props.urdf_filepath)
        mf = bpy.path.abspath(props.urdf_mesh_folder)
        if not fp or not os.path.isfile(fp):
            self.report({"ERROR"}, f"File not found: {fp}")
            return {"CANCELLED"}
        result, msg = _do_import_urdf(context, fp, mf)
        props.last_result = msg
        for line in msg.split("\n"):
            self.report({"INFO"} if result == {"FINISHED"} else {"ERROR"}, line)
        return result


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  Operators — DH XML
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

class DH_OT_FileBrowser(Operator):
    """Open a file browser and import a Robots-format DH XML"""
    bl_idname = "import_robot.dh_xml"
    bl_label = "Import DH XML"
    bl_options = {"REGISTER", "UNDO"}

    filepath: StringProperty(subtype="FILE_PATH")
    filter_glob: StringProperty(default="*.xml;*.XML", options={"HIDDEN"})

    def execute(self, context):
        if not self.filepath or not os.path.isfile(self.filepath):
            self.report({"ERROR"}, "Select a valid XML file.")
            return {"CANCELLED"}
        props = context.scene.robot_importer
        mf = bpy.path.abspath(props.dh_mesh_folder)
        result, msg = _do_import_dh(context, self.filepath, mf)
        props.last_result = msg
        for line in msg.split("\n"):
            self.report({"INFO"} if result == {"FINISHED"} else {"ERROR"}, line)
        return result

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)
        return {"RUNNING_MODAL"}


class DH_OT_ImportPanel(Operator):
    """Import DH XML from the paths set in the panel"""
    bl_idname = "import_robot.dh_panel"
    bl_label = "Import DH XML"
    bl_options = {"REGISTER", "UNDO"}

    def execute(self, context):
        props = context.scene.robot_importer
        fp = bpy.path.abspath(props.dh_filepath)
        mf = bpy.path.abspath(props.dh_mesh_folder)
        if not fp or not os.path.isfile(fp):
            self.report({"ERROR"}, f"XML not found: {fp}")
            return {"CANCELLED"}
        result, msg = _do_import_dh(context, fp, mf)
        props.last_result = msg
        for line in msg.split("\n"):
            self.report({"INFO"} if result == {"FINISHED"} else {"ERROR"}, line)
        return result


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  Properties
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

class RobotImporterProps(PropertyGroup):
    # URDF
    urdf_filepath: StringProperty(
        name="URDF / Xacro",
        description="Path to the .urdf or .xacro file",
        subtype="FILE_PATH",
    )
    urdf_mesh_folder: StringProperty(
        name="Meshes Folder",
        description="Folder with mesh files — matched to URDF links by filename (e.g. base_link.dae, link_1.stl)",
        subtype="DIR_PATH",
    )
    # DH XML
    dh_filepath: StringProperty(
        name="DH XML File",
        description="Path to the Robots-format DH parameter .xml file",
        subtype="FILE_PATH",
    )
    dh_mesh_folder: StringProperty(
        name="Mesh Folder",
        description="Folder with mesh files (sorted: first=base, then one per joint)",
        subtype="DIR_PATH",
    )
    # Shared
    last_result: StringProperty(name="Result", default="")


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  Panels
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

class ROBOT_PT_Main(Panel):
    bl_label = "Robot Import"
    bl_idname = "ROBOT_PT_main"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Robot Import"

    def draw(self, context):
        layout = self.layout
        # conventions summary
        box = layout.box()
        col = box.column(align=True)
        col.label(text="Animaquina conventions:", icon="ARMATURE_DATA")
        col.label(text="  base (Empty) > j0 (mesh) > armature")
        col.label(text="  Bones: joint_1 .. joint_6")
        col.label(text="  TCP empty at end-effector")
        col.label(text="  Axis map auto-detected")
        col.label(text="  Scale: meters")

        # result
        props = context.scene.robot_importer
        if props.last_result:
            layout.separator()
            box2 = layout.box()
            for line in props.last_result.split("\n"):
                if "WARNING" in line:
                    box2.label(text=line, icon="ERROR")
                else:
                    box2.label(text=line)


class ROBOT_PT_URDF(Panel):
    bl_label = "URDF Import"
    bl_idname = "ROBOT_PT_urdf"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Robot Import"
    bl_parent_id = "ROBOT_PT_main"

    def draw(self, context):
        layout = self.layout
        props = context.scene.robot_importer

        box = layout.box()
        col = box.column(align=True)
        col.label(text="1. Pick a URDF or xacro file", icon="FILE")
        col.label(text="2. Pick the meshes folder", icon="FILE_FOLDER")
        col.separator(factor=0.3)
        col.label(text="Meshes matched by URDF link name:")
        col.label(text="  base_link → base_link.dae/.stl/.obj")
        col.label(text="  link_1 → link_1.dae/.stl/.obj  etc.")

        layout.separator()
        layout.prop(props, "urdf_filepath")
        layout.prop(props, "urdf_mesh_folder")
        row = layout.row(align=True)
        row.scale_y = 1.4
        row.operator("import_robot.urdf_panel", icon="IMPORT")
        row.operator("import_robot.urdf", text="", icon="FILEBROWSER")


class ROBOT_PT_DH(Panel):
    bl_label = "DH XML Import (Robots Format)"
    bl_idname = "ROBOT_PT_dh"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Robot Import"
    bl_parent_id = "ROBOT_PT_main"

    def draw(self, context):
        layout = self.layout
        props = context.scene.robot_importer

        box = layout.box()
        col = box.column(align=True)
        col.label(text="DH XML  (visose/Robots format):", icon="INFO")
        col.separator(factor=0.3)
        col.label(text="  <RobotCell> <RobotArm>")
        col.label(text="    <Revolute a=\"0\" d=\"151\" .../>")
        col.separator(factor=0.3)
        col.label(text="Mesh folder — files sorted naturally:")
        col.label(text="  link_0.stl  (base / j0)")
        col.label(text="  link_1.stl  (→ joint_1)")
        col.label(text="  link_2.stl  (→ joint_2)  ...")
        col.separator(factor=0.3)
        col.label(text="Known DH alpha: UR, KUKA, ABB,")
        col.label(text="  Staubli, Fanuc.  Others: generic.")

        layout.separator()
        layout.prop(props, "dh_filepath")
        layout.prop(props, "dh_mesh_folder")
        row = layout.row(align=True)
        row.scale_y = 1.4
        row.operator("import_robot.dh_panel", icon="IMPORT")
        row.operator("import_robot.dh_xml", text="", icon="FILEBROWSER")


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  File > Import menu
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

def _menu_import(self, context):
    self.layout.operator(URDF_OT_FileBrowser.bl_idname, text="URDF Robot (.urdf / .xacro)")
    self.layout.operator(DH_OT_FileBrowser.bl_idname, text="DH XML Robot (.xml)")


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  Registration
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

_classes = (
    RobotImporterProps,
    URDF_OT_FileBrowser,
    URDF_OT_ImportPanel,
    DH_OT_FileBrowser,
    DH_OT_ImportPanel,
    ROBOT_PT_Main,
    ROBOT_PT_URDF,
    ROBOT_PT_DH,
)


def register():
    for cls in _classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.robot_importer = PointerProperty(type=RobotImporterProps)
    bpy.types.TOPBAR_MT_file_import.append(_menu_import)


def unregister():
    bpy.types.TOPBAR_MT_file_import.remove(_menu_import)
    del bpy.types.Scene.robot_importer
    for cls in reversed(_classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
