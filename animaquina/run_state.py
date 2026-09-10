# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
# Animaquina — canonical toolpath run state (the run_idx contract).
#
# slot.run_idx is the single property external consumers (PhyNodes, UI,
# drivers) read to know which waypoint the current run is at.
#
# Semantics (the contract): run_idx is the index of the waypoint currently in
# effect — the point the motion is heading toward, whose per-point attributes
# (speed, custom vars) apply to the segment being executed. -1 = idle. All
# producers use this meaning: exported programs write the index before the
# move into that point, UR streaming reports waypoints_completed (= the point
# being interpolated toward), KUKA streaming reports the last ring-buffer
# point read by the robot, sim playback reports the sampled point.
#
# Three producers, all writing from the main thread
# (see plans/run-idx-ecosystem-glue.md):
#   SIM     — frame_change_post handler evaluates the sim playback fcurve
#   STREAM  — the streaming modal ticks write the consumed index
#   PROGRAM — the poll timer mirrors a polled IDX / IDX_RT debug variable
#
# External data path (stable contract):
#   bpy.context.scene.animaquina.robots[<i>].run_idx

import bpy
from bpy.app.handlers import persistent

_SIM_CONSTRAINT = "animaquina_sim_geo_attr"
_SIM_DATA_PATH = f'constraints["{_SIM_CONSTRAINT}"].sample_index'

# Fallback debug variables that drive run_idx in PROGRAM mode, first match
# wins. IDX_RT (main-run TRIGGER variable) beats IDX (advance-run assignment).
# The slot's own export_point_index_var is tried ahead of these — on UR the
# index must go to an RTDE-readable register (output_int_register_0), so the
# name is never the KUKA-style "IDX".
PROGRAM_IDX_VARS = ("IDX_RT", "IDX")


def program_idx_var_names(slot):
    """Debug variable names to check for the run index, highest priority first.

    IDX_RT stays on top: where it exists it is the main-run TRIGGER variable,
    so it is more accurate than any advance-run assignment. The slot's
    configured export variable comes next (this is what the exported program
    actually writes — on UR an RTDE-readable register like
    output_int_register_0), with the legacy IDX as the last fallback."""
    names = ["IDX_RT"]
    configured = str(getattr(slot, "export_point_index_var", "") or "").strip()
    if configured and configured not in names:
        names.append(configured)
    names.extend(n for n in PROGRAM_IDX_VARS if n not in names)
    return names


def start_run(slot, source, obj=None, count=0):
    """Mark a run active on this slot. source: 'SIM' | 'STREAM' | 'PROGRAM'."""
    slot.run_source = source
    slot.run_object = obj
    slot.run_count = int(count)
    slot.run_idx = 0 if count > 0 else -1


def set_idx(slot, idx):
    """Update run_idx (clamped to the run's point range), write only on change."""
    idx = int(idx)
    if slot.run_count > 0:
        idx = min(idx, slot.run_count - 1)
    idx = max(-1, idx)
    if slot.run_idx != idx:
        slot.run_idx = idx


def clear_run(slot):
    if slot.run_source != "NONE" or slot.run_idx != -1 or slot.run_count != 0:
        slot.run_source = "NONE"
        slot.run_object = None
        slot.run_idx = -1
        slot.run_count = 0


# PROGRAM producer — called from manager._poll_all with the freshly polled
# debug variable values. Streaming/sim own run_idx while they are active.

def mirror_program_idx(slot, debug_values):
    if slot.run_source in ("SIM", "STREAM"):
        return
    for name in program_idx_var_names(slot):
        raw = debug_values.get(name)
        if raw in (None, ""):
            continue
        try:
            idx = int(float(str(raw)))
        except (TypeError, ValueError):
            continue
        if slot.run_source != "PROGRAM":
            # Borrow the last exported toolpath so clamping works and PhyNodes
            # attribute lookups have an object to index into.
            slot.run_source = "PROGRAM"
            slot.run_object = getattr(slot, "export_last_object", None)
            slot.run_count = int(getattr(slot, "export_last_count", 0))
        set_idx(slot, idx)
        return


# SIM producer — evaluate the Geometry Attribute constraint's animated
# sample_index at the current frame. The constraint lives on the playback
# target (slot.target_object or a helper Empty in the sim collection).

def _iter_collection_objects_recursive(coll):
    if coll is None:
        return
    for obj in getattr(coll, "objects", []):
        yield obj
    for child in getattr(coll, "children", []):
        yield from _iter_collection_objects_recursive(child)


def _sim_sample_index_fcurve(slot):
    candidates = []
    if slot.target_object is not None:
        candidates.append(slot.target_object)
    candidates.extend(_iter_collection_objects_recursive(slot.sim_collection))
    for obj in candidates:
        ad = getattr(obj, "animation_data", None)
        action = getattr(ad, "action", None)
        if action is None:
            continue
        for fc in action.fcurves:
            if fc.data_path == _SIM_DATA_PATH:
                return fc
    return None


@persistent
def _on_frame_change(scene, _depsgraph=None):
    props = getattr(scene, "animaquina", None)
    if props is None:
        return
    for slot in props.robots:
        if slot.run_source != "SIM":
            continue
        fc = _sim_sample_index_fcurve(slot)
        if fc is None:
            continue
        try:
            set_idx(slot, round(fc.evaluate(scene.frame_current)))
        except Exception:
            pass


@persistent
def _on_load_post(_filepath=None):
    # Run state is transient: a file saved mid-run must not reopen with a
    # phantom STREAM/PROGRAM run (which would also block mirror_program_idx).
    #
    # Connection state is transient for the same reason: is_connected is saved
    # in the .blend, but the driver behind it lives in manager._drivers, which
    # is runtime-only. Reopening a file saved while connected would otherwise
    # show "Connected" with no driver — enabling motion operators that then
    # fail, and hiding the fact that nothing is actually attached.
    for scene in bpy.data.scenes:
        props = getattr(scene, "animaquina", None)
        if props is None:
            continue
        for slot in props.robots:
            clear_run(slot)
            if slot.is_connected:
                slot.is_connected = False
            if getattr(slot, "realtime_puppet_active", False):
                slot.realtime_puppet_active = False
            if getattr(slot, "motion_active_label", ""):
                slot.motion_active_label = ""


def register():
    if _on_frame_change not in bpy.app.handlers.frame_change_post:
        bpy.app.handlers.frame_change_post.append(_on_frame_change)
    if _on_load_post not in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.append(_on_load_post)


def unregister():
    if _on_frame_change in bpy.app.handlers.frame_change_post:
        bpy.app.handlers.frame_change_post.remove(_on_frame_change)
    if _on_load_post in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(_on_load_post)
