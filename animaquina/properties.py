# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
# Animaquina — properties (single PropertyGroup, RobotSlot)

import bpy
from bpy.props import (
    BoolProperty,
    CollectionProperty,
    EnumProperty,
    FloatProperty,
    FloatVectorProperty,
    IntProperty,
    PointerProperty,
    StringProperty,
)
from bpy.types import Collection, Object, PropertyGroup, Scene

from .runtime import recorder


_TARGET_UPDATE_GUARD = False
_ACTIVE_ROBOT_SELECTION_GUARD = False


# Joint axis for rig mapping (Section 6, 17)
JOINT_AXIS_ITEMS = [
    ("X", "X", ""),
    ("Y", "Y", ""),
    ("Z", "Z", ""),
    ("-X", "-X", ""),
    ("-Y", "-Y", ""),
    ("-Z", "-Z", ""),
]

ROBOT_TYPE_ITEMS = [
    ("UR", "Universal Robots", ""),
    ("KUKA", "KUKA", ""),
    ("XARM", "xArm / UF", ""),
]

UR_MODEL_ITEMS = [
    ("UR_GENERIC", "Generic UR", "Use the standard UR axis map"),
    ("UR3", "UR3", "Universal Robots UR3 axis map"),
    ("UR5", "UR5", "Universal Robots UR5 axis map"),
    ("UR10", "UR10", "Universal Robots UR10 axis map"),
    ("UR16", "UR16", "Universal Robots UR16 axis map"),
    ("UR20", "UR20", "Universal Robots UR20 axis map"),
    ("UR30", "UR30", "Universal Robots UR30 axis map"),
]

KUKA_MODEL_ITEMS = [
    ("KUKA_GENERIC", "Generic KUKA", "Use the common KUKA axis map"),
    ("KR10", "KR10", "KUKA KR10 axis map"),
    ("KR30", "KR30", "KUKA KR30 axis map"),
    ("KR120", "KR120", "KUKA KR120 axis map"),
]

XARM_MODEL_ITEMS = [
    ("XARM6", "xArm 6", "xArm 6 / UFactory xArm6"),
    ("UF850", "UF850", "UFactory 850"),
]

SIMULATION_MODE_ITEMS = [
    ("BLENDER_IK", "Blender IK", "Use Blender simulation setup (IK constraints on sim rig)"),
    ("NEWTON_IK", "Newton IK", "Use Newton/MuJoCo IK validation and keyframed simulation"),
]

# When to update the Base object transform: manual = never overwrite; from_robot = use robot base when available
BASE_SOURCE_ITEMS = [
    ("MANUAL", "Manual", "Base position is set by you; never overwritten by the robot"),
    ("FROM_ROBOT", "From robot", "Base position is updated from the robot when the driver provides it"),
]


def _find_armature_in_collection(coll):
    """Return the first ARMATURE object in coll or any nested child collection."""
    if coll is None:
        return None
    for obj in coll.objects:
        if obj.type == "ARMATURE":
            return obj
    for child in coll.children:
        found = _find_armature_in_collection(child)
        if found is not None:
            return found
    return None


def _iter_collection_objects_recursive(coll):
    if coll is None:
        return
    for obj in coll.objects:
        yield obj
    for child in coll.children:
        yield from _iter_collection_objects_recursive(child)


def _find_tcp_in_collection(coll):
    """
    Best-effort TCP object detection inside the robot rig collection.
    Priority:
    1) exact 'tcp'
    2) names ending with '_tcp' / '.tcp'
    3) any object containing 'tcp' (excluding sim helper names)
    """
    if coll is None:
        return None
    objs = list(_iter_collection_objects_recursive(coll))
    if not objs:
        return None

    def _clean(n: str) -> str:
        return (n or "").strip().lower()

    exact = []
    suffix = []
    contains = []
    for obj in objs:
        name = _clean(getattr(obj, "name", ""))
        if not name or "animaquina_sim_target_" in name:
            continue
        if name == "tcp":
            exact.append(obj)
        elif name.endswith("_tcp") or name.endswith(".tcp") or name.endswith("twin_tcp"):
            suffix.append(obj)
        elif "tcp" in name:
            contains.append(obj)
    return (exact or suffix or contains or [None])[0]


def _find_tagged_object_in_collection(coll, token, exclude_substr=()):
    """First object in coll (recursive) whose cleaned name matches `token`.
    Priority: exact `token` > name ending `_token`/`.token` > name containing `token`.
    `exclude_substr`: skip objects whose name contains any of these substrings."""
    if coll is None:
        return None
    exact, suffix, contains = [], [], []
    for obj in _iter_collection_objects_recursive(coll):
        name = (getattr(obj, "name", "") or "").strip().lower()
        if not name or any(x in name for x in exclude_substr):
            continue
        if name == token:
            exact.append(obj)
        elif name.endswith("_" + token) or name.endswith("." + token):
            suffix.append(obj)
        elif token in name:
            contains.append(obj)
    return (exact or suffix or contains or [None])[0]


def _find_target_in_collection(coll):
    """Best-effort Target empty detection inside the robot rig collection (`*_target`).
    Excludes the addon's own sim-target helpers."""
    return _find_tagged_object_in_collection(
        coll, "target", exclude_substr=("animaquina_sim_target_",)
    )


def _object_is_in_collection_recursive(obj, coll) -> bool:
    if obj is None or coll is None:
        return False
    for cobj in _iter_collection_objects_recursive(coll):
        if cobj == obj:
            return True
    return False


def _update_rig_from_collection(slot, _context):
    """When rig_collection changes, auto-set rig_armature and base_object (J0) from the collection.
    J0 is the direct parent of the armature in the hierarchy: blender_base (empty) > j0 (mesh) > armature."""
    arm = _find_armature_in_collection(slot.rig_collection)
    slot.rig_armature = arm
    if arm is not None and arm.parent is not None:
        slot.base_object = arm.parent
    else:
        slot.base_object = None
    # Auto-pick TCP when it lives inside the rig collection (common rig convention: *_tcp).
    # Preserve a manual override if it points outside the rig collection.
    try:
        if slot.tcp_object is None or _object_is_in_collection_recursive(slot.tcp_object, slot.rig_collection):
            tcp_obj = _find_tcp_in_collection(slot.rig_collection)
            if tcp_obj is not None:
                slot.tcp_object = tcp_obj
    except Exception:
        pass
    # Auto-pick Target empty (common rig convention: *_target). Same override rule as TCP:
    # only replace when unset or when the current target still lives in this rig collection.
    # Assigning target_object runs _update_target_object, which enforces cross-slot uniqueness.
    try:
        if slot.target_object is None or _object_is_in_collection_recursive(slot.target_object, slot.rig_collection):
            target_obj = _find_target_in_collection(slot.rig_collection)
            if target_obj is not None and target_obj != slot.target_object:
                slot.target_object = target_obj
    except Exception:
        pass


def _update_base_source(slot, _context):
    """When switching to Manual base, convert TCP from world to base frame so it moves with the base."""
    if slot.base_source == "MANUAL":
        from .runtime import rig_apply
        rig_apply.world_tcp_to_base_frame(slot)


def _update_kuka_ring_buffer_size(slot, _context):
    """Changing ring size requires regenerating/re-uploading mq_stream."""
    try:
        slot.stream_program_uploaded = False
    except Exception:
        pass


def _update_joints_offline(slot, _context):
    """Apply joint angles to rig in real-time while editing offline (not connected)."""
    if slot.is_connected:
        return
    from .runtime.rig_apply import apply_full_pose
    apply_full_pose(slot)


def _update_base_offset_offline(slot, _context):
    """Apply base offset to rig when editing offline (not connected)."""
    if slot.is_connected:
        return
    slot.base_frame_valid = True
    from .runtime.rig_apply import apply_full_pose
    apply_full_pose(slot)


_BASE_EULER_DEG_GUARD = False


def _update_base_euler_deg_offline(slot, _context):
    """Sync base_euler_deg -> base_euler_rad and apply when editing offline."""
    global _BASE_EULER_DEG_GUARD
    if slot.is_connected or _BASE_EULER_DEG_GUARD:
        return
    import math
    _BASE_EULER_DEG_GUARD = True
    try:
        slot.base_euler_rad = tuple(math.radians(d) for d in slot.base_euler_deg)
        slot.base_frame_valid = True
        from .runtime.rig_apply import apply_full_pose
        apply_full_pose(slot)
    finally:
        _BASE_EULER_DEG_GUARD = False


def _update_tool_frame_offline(slot, _context):
    """Mark tool frame as valid when editing offline."""
    if slot.is_connected:
        return
    slot.tool_frame_valid = True


def _find_slot_index(props, slot):
    if props is None:
        return -1
    slot_uid = getattr(slot, "uid", "")
    for i, candidate in enumerate(getattr(props, "robots", [])):
        if slot_uid and getattr(candidate, "uid", "") == slot_uid:
            return i
        if candidate == slot:
            return i
    return -1


def _update_target_object(slot, context):
    """Keep target ownership unique and make the edited slot active."""
    global _TARGET_UPDATE_GUARD
    if _TARGET_UPDATE_GUARD:
        return

    scene = getattr(slot, "id_data", None) or getattr(context, "scene", None)
    props = getattr(scene, "animaquina", None)
    if props is None:
        return

    this_idx = _find_slot_index(props, slot)
    if this_idx >= 0 and props.active_robot_index != this_idx:
        props.active_robot_index = this_idx

    target_obj = getattr(slot, "target_object", None)
    if target_obj is None:
        return

    for i, other in enumerate(props.robots):
        if i == this_idx:
            continue
        if getattr(other, "target_object", None) == target_obj:
            _TARGET_UPDATE_GUARD = True
            try:
                slot.target_object = None
                props.active_robot_index = i
            finally:
                _TARGET_UPDATE_GUARD = False
            owner = getattr(other, "label", f"Robot {i + 1}")
            print(f"[Animaquina] Target '{target_obj.name}' already belongs to '{owner}'. Assignment cancelled.")
            break


def sync_active_robot_from_selected_target():
    """If the selected object is a robot target, make that robot the active slot."""
    try:
        ctx = bpy.context
        scene = getattr(ctx, "scene", None)
        obj = getattr(ctx, "active_object", None)
        if scene is None or obj is None:
            return
        props = getattr(scene, "animaquina", None)
        if props is None:
            return
        for i, slot in enumerate(props.robots):
            if getattr(slot, "target_object", None) == obj:
                if props.active_robot_index != i:
                    props.active_robot_index = i
                return
    except Exception:
        # Selection sync is best-effort and should never break the addon.
        pass


def _update_active_robot_index(_scene_props, context):
    """When switching active robot in the UI, select that robot's target (or TCP) object."""
    global _ACTIVE_ROBOT_SELECTION_GUARD
    if _ACTIVE_ROBOT_SELECTION_GUARD:
        return
    try:
        scene = getattr(context, "scene", None)
        view_layer = getattr(context, "view_layer", None)
        if scene is None or view_layer is None:
            return
        props = getattr(scene, "animaquina", None)
        if props is None:
            return
        idx = int(getattr(props, "active_robot_index", -1))
        if idx < 0 or idx >= len(props.robots):
            return
        slot = props.robots[idx]
        obj = getattr(slot, "target_object", None) or getattr(slot, "tcp_object", None)
        if obj is None:
            return
        if getattr(obj, "name", None) not in view_layer.objects:
            return
        _ACTIVE_ROBOT_SELECTION_GUARD = True
        try:
            try:
                obj.select_set(True)
            except Exception:
                pass
            try:
                view_layer.objects.active = obj
            except Exception:
                pass
        finally:
            _ACTIVE_ROBOT_SELECTION_GUARD = False
    except Exception:
        pass


class ANIMAQUINA_DebugVar(PropertyGroup):
    """One controller variable to poll in realtime (works with any robot that exposes read_var)."""

    enabled: BoolProperty(name="Enabled", default=True)
    var_name: StringProperty(name="Variable", default="")
    value: StringProperty(name="Value", default="")
    error: StringProperty(name="Error", default="")
    # Added by the addon (point-index tracking), not typed in by the user.
    # Only auto-added entries are removed automatically; manual ones are never
    # touched. Defaults to False so entries in existing .blend files are
    # treated as user-owned.
    auto_added: BoolProperty(name="Auto Added", default=False)


# Legacy alias so existing .blend files with the old class name still load
ANIMAQUINA_KukaDebugVar = ANIMAQUINA_DebugVar


def find_debug_var(slot, name) -> int:
    """Index of a polled debug variable by name, or -1. Case-insensitive: KRL
    variable names are, and the add-variable operator matches the same way."""
    target = str(name or "").strip().upper()
    if not target:
        return -1
    for i, item in enumerate(slot.debug_vars):
        if str(getattr(item, "var_name", "") or "").strip().upper() == target:
            return i
    return -1


def sync_point_index_debug_var(self, context):
    """Keep the point-index variable present in the polled debug list (KUKA).

    run_idx in PROGRAM mode is fed by manager._poll_all, which only reads names
    listed in slot.debug_vars. So enabling "Write Point Index" alone changes the
    exported program but produces no live index — the variable has to be tracked
    too. This closes that gap: the index var is the one value MQTT/PhyNodes
    consumers need, so it should never depend on remembering a second step.

    Entries added here are tagged auto_added and are removed again when the
    option is disabled, the name changes, or the slot stops being a KUKA. A name
    the user added by hand is left alone in both directions: never duplicated,
    never auto-removed.

    Cleanup below runs for every robot type on purpose — only the *adding* is
    KUKA-gated. Switching a slot KUKA -> UR must not strand an auto-added IDX,
    which would then be polled forever against a controller that has no such
    variable and just report errors.
    """
    wanted = ""
    if (
        getattr(self, "robot_type", "") == "KUKA"
        and getattr(self, "export_write_point_index", False)
    ):
        wanted = str(getattr(self, "export_point_index_var", "") or "").strip()

    # Drop stale auto-added entries (option turned off, or variable renamed).
    for i in range(len(self.debug_vars) - 1, -1, -1):
        item = self.debug_vars[i]
        if not getattr(item, "auto_added", False):
            continue
        if str(getattr(item, "var_name", "") or "").strip().upper() != wanted.upper():
            self.debug_vars.remove(i)

    if not wanted or find_debug_var(self, wanted) >= 0:
        return

    entry = self.debug_vars.add()
    entry.var_name = wanted
    entry.value = ""
    entry.error = ""
    entry.enabled = True
    entry.auto_added = True


class ANIMAQUINA_RobotSlot(PropertyGroup):
    """One robot slot: rig refs, connection, cache, motion/export settings."""

    uid: StringProperty(name="UID", default="")
    label: StringProperty(name="Label", default="Robot")

    robot_type: EnumProperty(
        name="Robot Type",
        items=ROBOT_TYPE_ITEMS,
        default="UR",
        # Adds the point-index var when a slot becomes KUKA with the option
        # already on, and clears the auto-added one when it stops being KUKA.
        update=sync_point_index_debug_var,
    )
    ur_model: EnumProperty(
        name="UR Model",
        items=UR_MODEL_ITEMS,
        default="UR_GENERIC",
    )
    kuka_model: EnumProperty(
        name="KUKA Model",
        items=KUKA_MODEL_ITEMS,
        default="KUKA_GENERIC",
    )
    xarm_model: EnumProperty(
        name="xArm Model",
        items=XARM_MODEL_ITEMS,
        default="XARM6",
    )

    # Rig references (rig_armature is auto-set from rig_collection)
    rig_armature: PointerProperty(name="Rig", type=Object, poll=lambda self, obj: obj.type == "ARMATURE")
    rig_collection: PointerProperty(
        name="Rig Collection",
        type=Collection,
        update=_update_rig_from_collection,
    )
    base_object: PointerProperty(
        name="J0 (Robot Base)",
        type=Object,
        description="Robot base mesh (j0).g Driven by the robot's base frame when base_source is From Robot. Armature and meshes should be parented to this.",
    )
    base_source: EnumProperty(
        name="Base source",
        items=BASE_SOURCE_ITEMS,
        default="FROM_ROBOT",
        description="Manual: you place the base; From robot: base is driven by robot when available",
        update=_update_base_source,
    )
    tcp_object: PointerProperty(name="TCP", type=Object)
    tool_object: PointerProperty(name="Tool", type=Object)
    target_object: PointerProperty(name="Target", type=Object, update=_update_target_object)
    sim_collection: PointerProperty(name="Sim Collection", type=Collection)
    collision_collection: PointerProperty(
        name="Collision Collection",
        type=Collection,
        description="Optional obstacle collection used by Newton path validation",
    )

    # Connection
    endpoint: StringProperty(name="IP / Host", default="")
    port: IntProperty(name="Port", default=0, min=0, max=65535)
    is_connected: BoolProperty(name="Connected", default=False)
    last_error: StringProperty(name="Last Error", default="")
    polling_enabled: BoolProperty(name="Polling", default=True)
    freedrive_active: BoolProperty(name="Freedrive", default=False, description="Freedrive (teach by hand) is on for this robot")
    ur_backend: EnumProperty(
        name="UR Backend",
        items=[
            ("ur_rtde", "ur_rtde (RTDE)", "Modern RTDE protocol — requires pip install"),
            ("urx", "URX (legacy)", "Bundled legacy library"),
        ],
        default="ur_rtde",
        description="Connection backend for Universal Robots",
    )
    ur_debug: BoolProperty(
        name="UR Debug Logging",
        default=False,
        description=(
            "Verbose animaquina UR logging to the console. Use with the "
            "'UR Debug Status' button to diagnose connection problems"
        ),
    )
    ur_install_status: StringProperty(name="UR Install Status", default="")
    ur_install_log: StringProperty(name="UR Install Log", default="")
    ur_transfer_status: StringProperty(name="UR Transfer Status", default="")
    ur_transfer_log: StringProperty(name="UR Transfer Log", default="")
    ur_queue_health: StringProperty(
        name="UR Queue Health",
        default="",
        description="Live Dynamic Sync queue state (phase, read index, written count, done id)",
    )

    # Motion
    move_mode: EnumProperty(
        name="Move Mode",
        items=[
            ("LINEAR", "Linear", "Linear (movel / LIN) — straight-line TCP path"),
            ("PTP", "PTP", "Point-to-point (movej / PTP) — fastest joint path"),
        ],
        default="LINEAR",
        description="Movement type for Move to Target",
    )
    speed: FloatProperty(name="Speed (m/s)", default=0.05, min=0.0, max=2.0, description="TCP velocity in m/s (conservative default)")
    acc: FloatProperty(name="Acc (m/s²)", default=0.2, min=0.0, max=5.0, description="TCP acceleration in m/s² (conservative default)")
    radius: FloatProperty(name="Radius (m)", default=0.001, min=0.0, max=0.5, description="Blend radius between waypoints in m")
    # Per-point speed from a mesh attribute (same pattern as Per-Point Orientation):
    # each toolpath point can carry its own linear speed. Attribute name is user
    # chosen (l_speed, speed, ...); values in m/s; points where the attribute is
    # missing or <= 0 fall back to the constant Speed above.
    use_speed_attribute: BoolProperty(
        name="Per-Point Speed",
        default=False,
        description="Read per-point linear speed (m/s) from a float mesh attribute during Run Toolpath; points without a valid value use the constant Speed",
    )
    speed_attribute: StringProperty(
        name="Speed Attribute",
        default="speed",
        description="Name of the float point attribute holding per-point linear speed in m/s (e.g. speed, l_speed)",
    )
    # Motion — joint (Go Home, etc.)
    joint_vel: FloatProperty(name="Joint Vel (rad/s)", default=0.5, min=0.01, max=6.28, soft_max=3.14, description="Joint velocity for joint moves (rad/s)")
    joint_acc: FloatProperty(name="Joint Acc (rad/s²)", default=0.8, min=0.01, max=10.0, soft_max=3.14, description="Joint acceleration for joint moves (rad/s²)")
    # Motion — home
    home_vel: FloatProperty(name="Home Vel (rad/s)", default=0.5, min=0.01, max=6.28, soft_max=3.14, description="Joint velocity for Go Home move (rad/s)")
    home_acc: FloatProperty(name="Home Acc (rad/s²)", default=0.8, min=0.01, max=10.0, soft_max=3.14, description="Joint acceleration for Go Home move (rad/s²)")
    kuka_ptp_speed_pct: FloatProperty(
        name="PTP Speed (%)",
        default=10.0,
        min=0.1,
        max=100.0,
        soft_max=30.0,
        description="KUKA PTP speed for joint/cartesian PTP moves (%)",
    )
    kuka_ptp_acc_pct: FloatProperty(
        name="PTP Acc (%)",
        default=100.0,
        min=0.0,
        max=100.0,
        description="KUKA PTP acceleration percentage",
    )
    kuka_puppet_speed_pct: FloatProperty(
        name="Puppet Speed (%)",
        default=10.0,
        min=0.1,
        max=100.0,
        soft_max=80.0,
        description="KUKA PTP speed for Puppet Mode (separate from regular PTP speed)",
    )
    wait: BoolProperty(name="Wait", default=False)
    motion_abort_requested: BoolProperty(
        name="Motion Abort Requested",
        default=False,
        description="Flag set by Program Controls stop — checked by running modals to abort gracefully",
    )
    motion_active_label: StringProperty(
        name="Motion Active Label",
        default="",
        description="Human label for the currently running motion (shown in Program Controls)",
    )
    realtime_puppet_active: BoolProperty(
        name="Puppet Mode Active",
        default=False,
        description="Runtime flag while Puppet Mode modal streaming is running",
    )
    realtime_puppet_status: StringProperty(
        name="Puppet Status",
        default="Idle",
        description="Latest Puppet Mode status",
    )
    realtime_puppet_rate_hz: FloatProperty(
        name="Puppet Rate (Hz)",
        default=50.0,
        min=1.0,
        max=250.0,
        soft_min=10.0,
        soft_max=100.0,
        description="Update rate for Puppet Mode setpoints",
    )
    realtime_puppet_max_step_mm: FloatProperty(
        name="Max Step (mm)",
        default=20.0,
        min=1.0,
        max=1000.0,
        soft_min=10.0,
        soft_max=200.0,
        description="Maximum allowed target movement per update in Puppet Mode; larger jumps are ignored for safety",
    )
    puppet_spring_enabled: BoolProperty(
        name="Spring Follow",
        default=True,
        description="Smooth robot motion using a spring-damper: the robot follows the target with natural acceleration and deceleration instead of jumping directly to each new position",
    )
    puppet_spring_freq: FloatProperty(
        name="Frequency (Hz)",
        default=4.0,
        min=0.5,
        max=20.0,
        soft_max=10.0,
        description="Spring responsiveness — higher values make the robot react faster and track more tightly; lower values produce slower, more fluid motion",
    )
    puppet_spring_damping: FloatProperty(
        name="Damping",
        default=1.0,
        min=0.1,
        max=2.0,
        soft_min=0.3,
        soft_max=1.5,
        description="1.0 = critically damped (no overshoot, smoothly settles); below 1.0 adds overshoot and springiness; above 1.0 is overdamped (sluggish). Default 1.0 is safest for robots",
    )
    puppet_spring_response: FloatProperty(
        name="Response",
        default=0.0,
        min=-5.0,
        max=5.0,
        soft_min=-2.0,
        soft_max=2.0,
        description="How the spring reacts to target velocity. 0 = starts from rest (neutral); 1 = immediately matches target speed; >1 = anticipates/leads the motion; <0 = briefly moves away first before following (organic anticipation snap)",
    )
    xarm_puppet_use_boundary: BoolProperty(
        name="Use xArm Safety Boundary",
        default=False,
        description="Apply reduced TCP boundary when starting Puppet Mode (xArm only)",
    )
    xarm_puppet_boundary_mm: FloatVectorProperty(
        name="xArm Boundary (mm)",
        size=6,
        default=(600.0, 205.0, 300.0, -300.0, 600.0, 100.0),
        description="xArm reduced-mode TCP boundary [x_max, x_min, y_max, y_min, z_max, z_min] in mm",
    )
    validation_step_dt: FloatProperty(
        name="Validation dt (s)",
        default=0.02,
        min=0.001,
        max=0.5,
        soft_min=0.005,
        soft_max=0.05,
        description="Sample step used by Newton path validation (Phase 1)",
    )
    validation_substeps: IntProperty(
        name="Validation Substeps",
        default=1,
        min=1,
        max=32,
        soft_max=8,
        description="Extra interpolation substeps for validation between path points",
    )
    newton_python_exe: StringProperty(
        name="Newton Python",
        default="",
        description="Optional Python executable for Newton worker (leave empty to use environment or Blender Python)",
    )
    newton_check_contacts: BoolProperty(
        name="Check Contacts",
        default=True,
        description="Enable MuJoCo contact checks during Newton validation",
    )
    newton_ignore_robot_self_contacts: BoolProperty(
        name="Ignore Self Contacts",
        default=True,
        description="Ignore contacts between robot bodies (useful while tuning exported collision geoms)",
    )
    newton_keyframe_step: IntProperty(
        name="Keyframe Step",
        default=1,
        min=1,
        max=1000,
        description="Frame step between Newton IK waypoint keyframes",
    )
    simulation_mode: EnumProperty(
        name="Simulation Mode",
        items=SIMULATION_MODE_ITEMS,
        default="BLENDER_IK",
        description="Choose the simulation backend workflow for the sim rig",
    )

    # KUKA KRL export (old addon: sna_basen, sna_tooln, sna_ca/cb/cc, sna_speed, sna_acc, sna_lspeed, sna_advance)
    export_base_no: IntProperty(name="Base #", default=0, min=0)
    export_tool_no: IntProperty(name="Tool #", default=0, min=0)
    export_custom_a: FloatProperty(name="Custom A (deg)", default=0.0)
    export_custom_b: FloatProperty(name="Custom B (deg)", default=0.0)
    export_custom_c: FloatProperty(name="Custom C (deg)", default=0.0)
    export_use_rotation_attribute: BoolProperty(
        name="Per-Point Orientation",
        default=False,
        description="Use 'rotation' mesh attribute (degrees, XYZ Euler) for per-point orientation instead of the constant custom A/B/C",
    )
    export_rotation_apply_transform: BoolProperty(
        name="Apply Object Transform",
        default=False,
        description="Apply the object's local transform to the rotation attribute values. When off, raw attribute values are used directly",
    )
    export_speed: FloatProperty(name="Speed %", default=15.0, min=0.0, max=100.0, soft_max=15.0)
    export_acc: IntProperty(name="Acc", default=100, min=0, max=100)
    export_lin_speed: FloatProperty(name="Lin Speed (m/s)", default=0.05, min=0.0, max=1.0, soft_max=0.2)
    export_advance: IntProperty(name="Advance", default=3, min=1, max=5)
    # Per-point index variable in exported programs. When on, the generated
    # program writes the current waypoint index into a variable at every move,
    # so a standalone program run reports progress. Poll it as a debug variable
    # (run_state PROGRAM mode mirrors it into run_idx) and bridge to PhyNodes.
    export_write_point_index: BoolProperty(
        name="Write Point Index",
        default=False,
        description=(
            "Emit a variable write at every waypoint holding its point index, so a "
            "standalone program run reports progress. Poll it as a debug variable and "
            "bridge it to PhyNodes/MQTT. KUKA writes it as a point-synchronized "
            "TRIGGER so it stays aligned with physical motion, and tracks the "
            "variable for polling automatically"
        ),
        update=sync_point_index_debug_var,
    )
    export_point_index_var: StringProperty(
        name="Index Variable",
        default="IDX",
        description=(
            "Variable the exported program writes the current point index to. "
            "KUKA: a name like IDX. UR: use an output register such as "
            "output_int_register_0 (plain script variables can't be read back over RTDE)"
        ),
        update=sync_point_index_debug_var,
    )
    # Recorded on every successful program export. Lets run_state attribute a
    # polled PROGRAM run (mirror_program_idx) to a toolpath object/point count
    # even though exports are driven by the active object at export time.
    export_last_object: PointerProperty(
        name="Last Exported Object",
        type=Object,
        description="Toolpath object of the most recent program export",
    )
    export_last_count: IntProperty(
        name="Last Exported Count",
        default=0,
        min=0,
        description="Waypoint count of the most recent program export",
    )

    # UR Script export — linear (movel)
    ur_export_vel: FloatProperty(
        name="Linear Vel (m/s)", default=0.05, min=0.001, max=2.0, soft_max=0.25,
        description="Tool velocity for movel commands (m/s)",
    )
    ur_export_acc: FloatProperty(
        name="Linear Acc (m/s²)", default=0.2, min=0.01, max=5.0, soft_max=1.5,
        description="Tool acceleration for movel commands (m/s²)",
    )
    ur_export_blend: FloatProperty(
        name="Blend Radius (m)", default=0.001, min=0.0, max=0.5, soft_max=0.05,
        description="Blend radius between waypoints (m)",
    )
    # UR Script export — joint (movej)
    ur_export_joint_vel: FloatProperty(
        name="Joint Vel (rad/s)", default=1.05, min=0.01, max=6.28, soft_max=3.14,
        description="Joint velocity for movej commands (rad/s)",
    )
    ur_export_joint_acc: FloatProperty(
        name="Joint Acc (rad/s²)", default=1.4, min=0.01, max=10.0, soft_max=3.14,
        description="Joint acceleration for movej commands (rad/s²)",
    )
    ur_export_custom_tool: BoolProperty(
        name="Custom Tool",
        default=False,
        description="Override pendant TCP and payload with custom values below",
    )
    ur_export_payload_mass: FloatProperty(
        name="Payload (kg)", default=0.0, min=0.0, max=35.0,
        description="Payload mass attached to tool flange (kg)",
    )
    ur_export_payload_cog: FloatVectorProperty(
        name="Payload CoG", size=3, default=(0.0, 0.0, 0.0),
        description="Payload center of gravity relative to tool flange (m)",
    )
    ur_export_tcp: FloatVectorProperty(
        name="TCP", size=6, default=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        description="Tool Center Point: x, y, z (m), rx, ry, rz (rad)",
    )
    ur_export_home: FloatVectorProperty(
        name="Home Joints (deg)", size=6, default=(0.0, -90.0, 0.0, 0.0, 0.0, 0.0),
        description="Home joint configuration in degrees (J0-J5)",
    )
    ur_program_name: StringProperty(
        name="Program Name",
        default="animaquina",
        description="Program filename on the UR controller (.script is added automatically)",
    )
    ur_stage_transport: EnumProperty(
        name="Stage Transport",
        items=[
            ("sftp", "SFTP (Full Program)", "Upload .script file to controller filesystem over SSH/SFTP"),
            ("runtime_script", "Runtime Script", "Legacy runtime send over secondary/RTDE channels"),
        ],
        default="sftp",
        description="Transport used by Stage to Robot",
    )
    ur_remote_path: StringProperty(
        name="Remote Path",
        default="/programs",
        description="Remote directory on UR controller for uploaded scripts",
    )
    ur_sftp_port: IntProperty(
        name="SFTP Port",
        default=22,
        min=1,
        max=65535,
        description="UR file transfer port (SFTP over SSH)",
    )
    ur_dashboard_port: IntProperty(
        name="Dashboard Port",
        default=29999,
        min=1,
        max=65535,
        description="UR Dashboard server port used for load/play commands",
    )
    ur_launcher_urp: StringProperty(
        name="Launcher URP",
        default="",
        description="Dashboard-loadable URP used by Run Program to execute staged content",
    )
    ur_queue_buffer_size: IntProperty(
        name="Ring Buffer Size",
        default=6,
        min=2,
        max=6,
        description="Number of waypoint slots in the RTDE register ring buffer "
                    "(max 6; each slot uses 6 input double registers and 36-38 are reserved for motion params)",
    )
    ur_sftp_user: StringProperty(
        name="SFTP User",
        default="root",
        description="Username for UR SFTP upload",
    )
    ur_sftp_password: StringProperty(
        name="SFTP Password",
        default="easybot",
        subtype="PASSWORD",
        description="Password for UR SFTP upload",
    )
    ur_auto_play_after_load: BoolProperty(
        name="Auto Run After Load",
        default=False,
        description="Send Dashboard run command right after successful load",
    )

    # UI foldouts (per robot slot)
    ui_ctrl_show_motion_settings: BoolProperty(
        name="Show Motion Settings",
        default=False,
    )
    ui_ctrl_show_puppet_settings: BoolProperty(
        name="Show Puppet Settings",
        default=False,
    )
    ui_ctrl_show_advanced_streaming: BoolProperty(
        name="Show Advanced Streaming",
        default=False,
    )
    ui_ur_show_export_settings: BoolProperty(
        name="Show UR Export Settings",
        default=False,
    )
    ui_ur_show_stage_settings: BoolProperty(
        name="Show UR Stage Settings",
        default=False,
    )
    ui_ur_show_play_settings: BoolProperty(
        name="Show UR Run Settings",
        default=False,
    )
    ui_kuka_show_export_settings: BoolProperty(
        name="Show KUKA Export Settings",
        default=True,
    )
    ui_kuka_show_stage_settings: BoolProperty(
        name="Show KUKA Stage Settings",
        default=False,
    )
    ui_kuka_show_play_settings: BoolProperty(
        name="Show KUKA Run Settings",
        default=False,
    )

    # KUKA export home position (A1-A6 deg)
    kuka_export_home: FloatVectorProperty(
        name="Home Joints (deg)", size=6, default=(5.0, -90.0, 100.0, 5.0, -10.0, -5.0),
        description="Home joint configuration in degrees (A1-A6)",
    )

    # xArm export home position (J1-J6 deg)
    xarm_export_home: FloatVectorProperty(
        name="Home Joints (deg)", size=6, default=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        description="Home joint configuration in degrees (J1-J6)",
    )

    # Dynamic Sync (KUKA): track whether mq_stream has been uploaded
    stream_program_uploaded: BoolProperty(
        name="Stream Program Uploaded",
        default=False,
        description="Whether the Dynamic Sync program (mq_stream) has been uploaded to this robot",
    )
    kuka_stream_uploaded_ring_size: IntProperty(
        name="Uploaded Ring Size",
        default=0,
        min=0,
        max=128,
        description="Ring buffer size baked into the last uploaded mq_stream program (0 means unknown)",
    )

    # C3 Bridge: program name and path on robot controller (Save/Load Program)
    program_name: StringProperty(
        name="Program Name",
        default="animaquina",
        description="Program name on the KUKA controller (for upload/select via C3 Bridge)",
    )
    remote_path: StringProperty(
        name="Remote Path",
        default="KRC:\\R1\\Program\\",
        description="KRC path on the controller where programs are uploaded (e.g. KRC:\\R1\\Program\\)",
    )
    kuka_ring_buffer_size: IntProperty(
        name="Ring Buffer Size",
        default=6,
        min=4,
        max=128,
        update=_update_kuka_ring_buffer_size,
        description="Number of waypoint slots in the Dynamic Sync ring buffer (MQ_PT[]). "
                    "Keep it above Advance Lookahead so the planner can't drain the buffer. "
                    "Changing this requires re-uploading the stream program and updating $CONFIG.DAT",
    )
    kuka_advance: IntProperty(
        name="Advance Lookahead",
        default=3,
        min=1,
        max=5,
        update=_update_kuka_ring_buffer_size,
        description="$ADVANCE planner lookahead in the Dynamic Sync program. Lower values leave more "
                    "ring-buffer margin for the producer (reduces start-of-path stalls); 3 is the KUKA "
                    "default and fine for blending. Clamped below Ring Buffer Size. "
                    "Changing this requires re-uploading the stream program",
    )
    debug_new_var: StringProperty(
        name="Variable",
        default="",
        description="Controller variable to read (KUKA: $OV_PRO; UR: output_double_register_0; xArm: cgpio_input_0)",
    )
    debug_vars: CollectionProperty(type=ANIMAQUINA_DebugVar, name="Debug Vars")

    # --- Dataset recorder -------------------------------------------------
    # Samples are held in animaquina.runtime.recorder (not in the .blend); these
    # properties are the UI mirror, refreshed by the poll timer.
    record_name: StringProperty(
        name="Dataset",
        default="dataset",
        description="Name for the recorded dataset. Used for the Text datablock and the exported .csv",
    )
    record_max_samples: IntProperty(
        name="Max Samples",
        default=50000,
        min=100,
        max=recorder.MAX_SAMPLES_HARD,
        description="Recording auto-stops at this many samples. At 25 Hz, 50000 samples is about 33 minutes. "
                    "The capture lives in a Blender Text datablock, so an uncapped recording would bloat the .blend",
    )
    record_active: BoolProperty(
        name="Recording",
        default=False,
        description="A dataset recording is running on this slot",
    )
    record_sample_count: IntProperty(name="Recorded Samples", default=0, min=0)
    record_elapsed_s: FloatProperty(name="Recorded Duration (s)", default=0.0, min=0.0)
    record_text_name: StringProperty(
        name="Dataset Text Block",
        default="",
        description="Text datablock the last recording was written to",
    )

    # Toolpath run state (canonical contract for external consumers, e.g. PhyNodes).
    # run_idx is the current waypoint index of whatever is "running" on this slot:
    # sim playback, live streaming, or a polled exported program. -1 = idle.
    # Read from: bpy.context.scene.animaquina.robots[<i>].run_idx
    run_idx: IntProperty(
        name="Run Index",
        default=-1,
        min=-1,
        description="Current waypoint index of the running toolpath (-1 when idle)",
    )
    run_count: IntProperty(
        name="Run Count",
        default=0,
        min=0,
        description="Total waypoint count of the running toolpath",
    )
    run_source: EnumProperty(
        name="Run Source",
        items=[
            ("NONE", "None", "No toolpath running"),
            ("SIM", "Simulation", "Blender sim playback drives run_idx"),
            ("STREAM", "Streaming", "Live streaming drives run_idx"),
            ("PROGRAM", "Program", "Polled IDX variable from an exported program drives run_idx"),
        ],
        default="NONE",
    )
    run_object: PointerProperty(
        name="Run Object",
        type=Object,
        description="Toolpath object currently running (attribute lookups index into this mesh)",
    )

    # Runtime cache (canonical: m, rad, deg)
    tcp_pos_m: FloatVectorProperty(name="TCP Pos (m)", size=3, subtype="TRANSLATION", unit="LENGTH")
    tcp_euler_rad: FloatVectorProperty(name="TCP Euler (rad)", size=3, subtype="EULER", unit="ROTATION")
    tcp_euler_deg: FloatVectorProperty(name="A, B, C (deg)", size=3, subtype="NONE")  # display only, kept in sync for UI
    joints_deg: FloatVectorProperty(name="Joints (deg)", size=6, subtype="NONE", update=_update_joints_offline)
    base_pos_m: FloatVectorProperty(name="Base Pos (m)", size=3, subtype="TRANSLATION", unit="LENGTH", update=_update_base_offset_offline)
    base_euler_rad: FloatVectorProperty(name="Base Euler (rad)", size=3, subtype="EULER", unit="ROTATION")
    base_euler_deg: FloatVectorProperty(name="Base Euler (deg)", size=3, subtype="NONE", update=_update_base_euler_deg_offline)

    # KUKA $TOOL / $BASE parsed (read-only display; frame parser in kuka_driver)
    tool_frame_pos_m: FloatVectorProperty(name="$TOOL Pos (m)", size=3, subtype="TRANSLATION", unit="LENGTH", update=_update_tool_frame_offline)
    tool_frame_euler_deg: FloatVectorProperty(name="$TOOL Euler (deg)", size=3, subtype="NONE", update=_update_tool_frame_offline)
    tool_frame_valid: BoolProperty(name="$TOOL valid", default=False)
    base_frame_valid: BoolProperty(name="$BASE valid", default=False)
    has_tool: BoolProperty(name="Has Tool", default=False,
        description="Explicitly mark this slot as having a tool attached to the flange. "
                    "When True, IK targets the tcp bone tail (tool tip). "
                    "When False, auto-detection is used as fallback")
    validation_last_status: StringProperty(name="Validation Status", default="")
    validation_last_message: StringProperty(name="Validation Message", default="")
    validation_last_backend: StringProperty(name="Validation Backend", default="")
    validation_last_backend_message: StringProperty(name="Validation Backend Message", default="")
    validation_last_worker_stderr: StringProperty(name="Validation Worker Stderr", default="")
    validation_last_contacts: StringProperty(name="Validation Contacts", default="")
    validation_last_debug: StringProperty(name="Validation Debug", default="")
    validation_last_failure_index: IntProperty(name="Validation Failure Index", default=-1, min=-1)
    newton_install_status: StringProperty(name="Newton Install Status", default="")
    newton_install_log: StringProperty(name="Newton Install Log", default="")

    # Joint axis map (Section 17): which Blender bone axis per robot joint
    joint_axis_0: EnumProperty(name="Joint 0 Axis", items=JOINT_AXIS_ITEMS, default="Y")
    joint_axis_1: EnumProperty(name="Joint 1 Axis", items=JOINT_AXIS_ITEMS, default="Z")
    joint_axis_2: EnumProperty(name="Joint 2 Axis", items=JOINT_AXIS_ITEMS, default="Z")
    joint_axis_3: EnumProperty(name="Joint 3 Axis", items=JOINT_AXIS_ITEMS, default="Y")
    joint_axis_4: EnumProperty(name="Joint 4 Axis", items=JOINT_AXIS_ITEMS, default="Y")
    joint_axis_5: EnumProperty(name="Joint 5 Axis", items=JOINT_AXIS_ITEMS, default="Y")

    @property
    def joint_axis_map(self):
        return [
            getattr(self, f"joint_axis_{i}") for i in range(6)
        ]


class ANIMAQUINA_SceneProperties(PropertyGroup):
    """Single addon state on Scene. Section 6, 25."""

    robots: CollectionProperty(type=ANIMAQUINA_RobotSlot, name="Robots")
    active_robot_index: IntProperty(name="Active Robot", default=0, min=0, update=_update_active_robot_index)
    global_debug: BoolProperty(name="Debug", default=False)
    # Name to use when adding a new robot slot (user types then clicks Add)
    new_robot_name: StringProperty(name="New Robot Name", default="Robot")
    # Single poll rate for all realtime slots (Hz)
    poll_rate_hz: FloatProperty(name="Poll Rate (Hz)", default=25.0, min=1.0, max=50.0, soft_max=50.0)


def register_properties():
    Scene.animaquina = PointerProperty(type=ANIMAQUINA_SceneProperties, name="Animaquina")


def unregister_properties():
    del Scene.animaquina
