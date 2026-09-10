# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
# Animaquina - UI panels (hierarchical sub-panel layout)

import bpy
from bpy.types import Panel, UIList, Header

from .operators import get_active_slot
from .. import manager
from animaquina_core.drivers.base import CAP_MANUAL_MODE, CAP_MOVE_TO_TARGET, CAP_EXECUTE_PATH, CAP_HOME, CAP_RESET


def _ur_rtde_available():
    try:
        from animaquina_core.drivers.ur_driver import ur_rtde_available
        return ur_rtde_available()
    except ImportError:
        return False


def _ur_sftp_available():
    try:
        from animaquina_core.drivers.ur_driver import ur_sftp_available
        return ur_sftp_available()
    except ImportError:
        return False


def get_active_slot_from_context(context):
    return get_active_slot(context)


# Robot list widget

class ANIMAQUINA_UL_RobotList(UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        if layout is None or item is None:
            return
        try:
            raw = getattr(item, "label", None)
            name = (str(raw).strip() if raw else "") or f"Robot {index + 1}"
            if self.layout_type in ("DEFAULT", "COMPACT"):
                row = layout.row(align=True)
                row.label(text=name)
            else:
                layout.alignment = "CENTER"
                layout.label(text=name)
        except Exception as e:
            import traceback
            print("ANIMAQUINA_UL_RobotList.draw_item:", e)
            traceback.print_exc()
            try:
                layout.label(text="Error")
            except Exception:
                pass


# 1. Robot Registry (top-level, bl_order=0)

class ANIMAQUINA_PT_Registry(Panel):
    bl_label = "Robot Registry"
    bl_idname = "ANIMAQUINA_PT_registry"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Animaquina"
    bl_order = 0
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        return context.scene is not None and hasattr(context.scene, "animaquina")

    def draw_header_preset(self, context):
        # In the header so the mode stays reachable while the panel is collapsed.
        self.layout.prop(context.scene.animaquina, "ui_complexity", text="")

    def draw(self, context):
        try:
            if context.scene is None:
                return
            props = getattr(context.scene, "animaquina", None)
            if props is None:
                return
            layout = getattr(self, "layout", None)
            if layout is None:
                return
            layout.prop(props, "new_robot_name", text="Name")
            row = layout.row(align=True)
            row.operator("object.animaquina_add_slot", text="Add Robot")
            row.operator("object.animaquina_remove_slot", text="Remove")
            layout.separator()
            if getattr(props, "robots", None) is not None:
                layout.template_list(
                    "ANIMAQUINA_UL_RobotList",
                    "robots",
                    props,
                    "robots",
                    props,
                    "active_robot_index",
                    rows=4,
                )
            if len(getattr(props, "robots", [])) > 0:
                slot = get_active_slot_from_context(context)
                if slot is None:
                    layout.label(text="Select a robot above")
        except Exception as e:
            import traceback
            print("ANIMAQUINA_PT_registry.draw:", e)
            traceback.print_exc()
            try:
                if self.layout:
                    self.layout.label(text="Animaquina: error (see console)")
            except Exception:
                pass


# 2. Setup (top-level, bl_order=1) - robot type, model, rig

class ANIMAQUINA_PT_Setup(Panel):
    bl_label = "Setup"
    bl_idname = "ANIMAQUINA_PT_setup"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Animaquina"
    bl_order = 1
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        return get_active_slot_from_context(context) is not None

    def draw(self, context):
        try:
            if context.scene is None:
                return
            layout = getattr(self, "layout", None)
            if layout is None:
                return
            slot = get_active_slot_from_context(context)
            if slot is None:
                return
            col = layout.column(align=True)
            col.prop(slot, "robot_type")
            col.prop(slot, "rig_collection", text="Rig Collection")
            if slot.rig_armature:
                col.label(text=f"Rig: {slot.rig_armature.name}")
            elif slot.rig_collection:
                col.label(text="Rig: (no armature in collection)")
            if slot.robot_type == "UR":
                col.prop(slot, "ur_model", text="Model")
                col.operator("object.animaquina_set_ur_axes", text="Set UR axes (model)")
            if slot.robot_type == "KUKA":
                col.prop(slot, "kuka_model", text="Model")
                col.operator("object.animaquina_set_kuka_axes", text="Set KUKA axes (model)")
            if slot.robot_type == "XARM":
                col.prop(slot, "xarm_model", text="Model")
                col.operator("object.animaquina_set_xarm_axes", text="Set xArm axes (model)")
        except Exception as e:
            import traceback
            print("ANIMAQUINA_PT_setup.draw:", e)
            traceback.print_exc()
            try:
                if self.layout:
                    self.layout.label(text="Setup: error (see console)")
            except Exception:
                pass


class ANIMAQUINA_PT_Connection(Panel):
    bl_label = "Connection"
    bl_idname = "ANIMAQUINA_PT_connection"
    bl_parent_id = "ANIMAQUINA_PT_setup"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Animaquina"

    @classmethod
    def poll(cls, context):
        return get_active_slot_from_context(context) is not None

    def draw(self, context):
        layout = self.layout
        slot = get_active_slot_from_context(context)
        if slot is None:
            return
        if slot.robot_type == "UR":
            layout.prop(slot, "ur_backend", text="Backend")
            if slot.ur_backend == "ur_rtde":
                layout.prop(slot, "ur_debug")
            if slot.is_connected:
                driver = manager.get_driver_for_slot(slot)
                backend = getattr(driver, "backend_name", "") if driver is not None else ""
                if backend:
                    layout.label(text=f"Connected via: {backend}")
            if slot.ur_backend == "ur_rtde":
                layout.operator("object.animaquina_ur_debug_status", icon="CONSOLE")
        layout.prop(slot, "endpoint", text="IP")
        row = layout.row(align=True)
        row.operator("object.animaquina_connect_robot", text="Connect")
        row.operator("object.animaquina_disconnect_robot", text="Disconnect")
        if slot.is_connected:
            row = layout.row(align=True)
            row.prop(slot, "polling_enabled", text="Realtime")
            row.prop(context.scene.animaquina, "poll_rate_hz", text="Hz")


# --- Setup > Scene Objects (sub-panel) ---

class ANIMAQUINA_PT_SceneObjects(Panel):
    bl_label = "Scene Objects"
    bl_idname = "ANIMAQUINA_PT_scene_objects"
    bl_parent_id = "ANIMAQUINA_PT_setup"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Animaquina"

    @classmethod
    def poll(cls, context):
        return get_active_slot_from_context(context) is not None

    def draw(self, context):
        layout = self.layout
        slot = get_active_slot_from_context(context)
        if slot is None:
            return
        col = layout.column(align=True)
        col.prop(slot, "tcp_object", text="TCP")
        col.prop(slot, "tool_object", text="Tool")
        col.prop(slot, "target_object", text="Target")
        if slot.tcp_object:
            col.label(text=f"TCP auto: {slot.tcp_object.name}")
        row = layout.row(align=True)
        row.operator("object.animaquina_set_tool", text="Set Tool")
        if getattr(slot, "last_error", None):
            layout.label(text=slot.last_error)


# 3. Control (top-level, bl_order=2) - merges Commands + Motion

def _is_basic(context):
    """True when the UI is in Basic mode."""
    return getattr(context.scene.animaquina, "ui_complexity", "BASIC") == "BASIC"


def _foldout(container, context, ui, prop_name, label):
    """Draw a collapsible section header; return True when the body should draw.

    These sections hold tuning - speeds, spring constants, transport settings -
    so in Basic they are absent rather than collapsed. A chevron that reveals
    what the mode exists to hide is worse than no chevron.
    """
    if _is_basic(context):
        return False
    row = container.row(align=True)
    open_now = bool(getattr(ui, prop_name, False))
    row.prop(
        ui, prop_name, text=label, emboss=False,
        icon=("TRIA_DOWN" if open_now else "TRIA_RIGHT"),
    )
    return open_now


def _slot_driver_caps(context):
    """(slot, driver, caps) for the active slot. caps is 0 when disconnected."""
    slot = get_active_slot_from_context(context)
    if slot is None:
        return None, None, 0
    driver = manager.get_driver_for_slot(slot)
    return slot, driver, (driver.capabilities() if driver else 0)


TOOLPATH_CAPS = (
    CAP_MANUAL_MODE | CAP_HOME | CAP_RESET | CAP_MOVE_TO_TARGET | CAP_EXECUTE_PATH
)


def _puppet_capable(slot, driver):
    return bool(
        driver
        and hasattr(driver, "realtime_puppet_start")
        and hasattr(driver, "realtime_puppet_step")
        and hasattr(driver, "realtime_puppet_stop")
        and slot is not None
        and slot.is_connected
    )


def _streaming_available(slot):
    if slot is None:
        return False
    return (slot.robot_type == "KUKA") or (
        slot.robot_type == "UR" and slot.ur_backend == "ur_rtde"
    )


def _draw_motion_settings(container, slot):
        if slot.robot_type in {"UR", "KUKA", "XARM"}:
            container.label(text="Linear (movel)")
            row = container.row(align=True)
            row.label(text="Vel (m/s)")
            row.label(text="Acc (m/s^2)")
            row.label(text="Radius (m)")
            row = container.row(align=True)
            row.prop(slot, "speed", text="")
            row.prop(slot, "acc", text="")
            row.prop(slot, "radius", text="")
            if slot.robot_type == "UR":
                # Per-point speed from attribute (Run Toolpath Buffered)
                row = container.row(align=True)
                row.prop(slot, "use_speed_attribute", text="Per-Point Speed")
                sub = row.row(align=True)
                sub.active = slot.use_speed_attribute
                sub.prop(slot, "speed_attribute", text="")
        container.label(text=("Joint (PTP)" if slot.robot_type == "KUKA" else "Joint (movej)"))
        row = container.row(align=True)
        row.label(text=("Speed (%)" if slot.robot_type == "KUKA" else "Vel (rad/s)"))
        row.label(text=("Acc (%)" if slot.robot_type == "KUKA" else "Acc (rad/s^2)"))
        row = container.row(align=True)
        row.prop(slot, ("kuka_ptp_speed_pct" if slot.robot_type == "KUKA" else "joint_vel"), text="")
        row.prop(slot, ("kuka_ptp_acc_pct" if slot.robot_type == "KUKA" else "joint_acc"), text="")
        if slot.robot_type != "KUKA":
            container.label(text="Home")
            row = container.row(align=True)
            row.label(text="Vel (rad/s)")
            row.label(text="Acc (rad/s^2)")
            row = container.row(align=True)
            row.prop(slot, "home_vel", text="")
            row.prop(slot, "home_acc", text="")


class ANIMAQUINA_PT_Control(Panel):
    bl_label = "Control"
    bl_idname = "ANIMAQUINA_PT_control"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Animaquina"
    bl_order = 2
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        return get_active_slot_from_context(context) is not None

    def draw(self, context):
        pass  # parent header only; content lives in the sub-panels below


class ANIMAQUINA_PT_ControlProgram(Panel):
    bl_label = "Program"
    bl_idname = "ANIMAQUINA_PT_controlprogram"
    bl_parent_id = "ANIMAQUINA_PT_control"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Animaquina"

    @classmethod
    def poll(cls, context):
        slot = get_active_slot_from_context(context)
        return slot is not None and slot.is_connected

    def draw(self, context):
        layout = self.layout
        ui = context.scene.animaquina
        slot, driver, caps = _slot_driver_caps(context)
        if slot is None:
            return
        # --- Program Controls (always visible when connected) ---
        ctrl_box = layout.box()
        ctrl_box.label(text="Program Controls")

        # Status line — show what is currently running
        active_label = str(getattr(slot, "motion_active_label", "") or "")
        if active_label:
            status_row = ctrl_box.row()
            status_row.label(text=active_label, icon="PLAY")

        if slot.robot_type == "UR":
            row = ctrl_box.row(align=True)
            row.operator("object.animaquina_ur_pause_program", text="Pause", icon="PAUSE")
            stop_row = row.row(align=True)
            stop_row.alert = True
            stop_row.operator("object.animaquina_ur_stop_program", text="Stop", icon="CANCEL")
        elif slot.robot_type == "KUKA":
            has_stop = hasattr(driver, "stop_program")
            has_cancel = hasattr(driver, "cancel_program")
            if has_stop or has_cancel:
                row = ctrl_box.row(align=True)
                if has_stop:
                    stop_sub = row.row(align=True)
                    stop_sub.alert = True
                    stop_sub.operator("object.animaquina_kuka_stop_program", text="Stop", icon="PAUSE")
                if has_cancel:
                    cancel_sub = row.row(align=True)
                    cancel_sub.alert = True
                    cancel_sub.operator("object.animaquina_kuka_cancel_program", text="Cancel", icon="CANCEL")
        elif slot.robot_type == "XARM" and hasattr(driver, "stop_motion"):
            row = ctrl_box.row(align=True)
            stop_row = row.row(align=True)
            stop_row.alert = True
            stop_row.operator("object.animaquina_xarm_stop_motion", text="Stop", icon="CANCEL")


class ANIMAQUINA_PT_ControlToolpath(Panel):
    bl_label = "Toolpath"
    bl_idname = "ANIMAQUINA_PT_controltoolpath"
    bl_parent_id = "ANIMAQUINA_PT_control"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Animaquina"

    @classmethod
    def poll(cls, context):
        _, _, caps = _slot_driver_caps(context)
        return bool(caps & TOOLPATH_CAPS)

    def draw(self, context):
        layout = self.layout
        ui = context.scene.animaquina
        slot, driver, caps = _slot_driver_caps(context)
        if slot is None:
            return
        # --- Interactive Toolpathing ---
        # caps is 0 while disconnected, so build the box only when the driver
        # actually offers something - otherwise this drew an empty section.
        _toolpath_caps = (
            CAP_MANUAL_MODE | CAP_HOME | CAP_RESET | CAP_MOVE_TO_TARGET | CAP_EXECUTE_PATH
        )
        toolpath_box = layout.box()
        toolpath_box.label(text="Interactive Toolpathing")
        toolpath_box.operator("object.animaquina_update_pose", text="Sync From Robot")
        if caps & CAP_MANUAL_MODE:
            toolpath_box.operator("object.animaquina_manual_mode", text="Teach Mode", depress=slot.freedrive_active)
        if caps & CAP_HOME:
            toolpath_box.operator("object.animaquina_go_home", text="Go Home")
        if caps & CAP_RESET:
            toolpath_box.operator("object.animaquina_reset", text="Reset")
        if caps & CAP_MOVE_TO_TARGET:
            row = toolpath_box.row(align=True)
            row.operator("object.animaquina_snap_target_to_tcp", text="", icon="SNAP_ON")
            row.operator("object.animaquina_move_to_target", text="Move to Target")
        if caps & CAP_EXECUTE_PATH:
            row = toolpath_box.row(align=True)
            row.operator("object.animaquina_send_path", text="Run Toolpath")
            if slot.robot_type == "UR" and slot.ur_backend == "ur_rtde":
                row.operator("object.animaquina_send_path_queue_ur", text="Run Toolpath (Buffered)")
        toolpath_box.operator("object.animaquina_add_marker", text="Add Marker")

        # --- Shared Motion Settings ---
        if (caps & CAP_MOVE_TO_TARGET) or (caps & CAP_EXECUTE_PATH):
            if _foldout(layout, context, ui, "ui_ctrl_show_motion_settings", "Motion Settings"):
                box = layout.box()
                box.prop(slot, "move_mode", text="Move")
                _draw_motion_settings(box, slot)


class ANIMAQUINA_PT_ControlPuppet(Panel):
    bl_label = "Puppet Mode"
    bl_idname = "ANIMAQUINA_PT_controlpuppet"
    bl_parent_id = "ANIMAQUINA_PT_control"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Animaquina"

    @classmethod
    def poll(cls, context):
        slot, driver, _ = _slot_driver_caps(context)
        return _puppet_capable(slot, driver)

    def draw(self, context):
        layout = self.layout
        ui = context.scene.animaquina
        slot, driver, caps = _slot_driver_caps(context)
        if slot is None:
            return
        # --- Puppet Mode ---
        # Only drivers that implement the puppet trio can do this at all, so a
        # driver lacking them gets no section rather than an explanation.
        _puppet_capable = (
            driver
            and hasattr(driver, "realtime_puppet_start")
            and hasattr(driver, "realtime_puppet_step")
            and hasattr(driver, "realtime_puppet_stop")
            and slot.is_connected
        )
        puppet_box = layout.box()
        puppet_box.label(text="Puppet Mode")
        # ur_backend is a setting the user can change, so this one stays
        # visible as a hint rather than being hidden.
        if slot.robot_type == "UR" and str(getattr(driver, "backend_name", "") or "") != "ur_rtde":
            puppet_box.label(text="Requires UR backend: ur_rtde", icon="INFO")
        else:
            # Basic requires a work boundary before streaming live targets.
            # Advanced can run without one, deliberately.
            _needs_bounds = _is_basic(context) and slot.puppet_bounds_object is None
            row = puppet_box.row(align=True)
            start_sub = row.row(align=True)
            start_sub.enabled = not _needs_bounds
            start_sub.operator("object.animaquina_realtime_puppet_start", text="Start Puppet Mode")
            row.operator("object.animaquina_realtime_puppet_stop", text="Stop Puppet Mode")
            if _needs_bounds:
                puppet_box.label(text="Set a work boundary to start", icon="ERROR")

            # Work boundary. Sits above the status line because whether one
            # is set changes how to read that status.
            bounds_row = puppet_box.row(align=True)
            bounds_row.prop(slot, "puppet_bounds_enabled", text="")
            sub = bounds_row.row(align=True)
            sub.enabled = slot.puppet_bounds_enabled
            sub.prop(slot, "puppet_bounds_object", text="Boundary")
            if slot.puppet_bounds_object is None:
                bounds_row.operator(
                    "animaquina.create_puppet_bounds", text="", icon="MESH_CUBE"
                )
            if slot.puppet_bounds_enabled and slot.puppet_bounds_object is not None                         and getattr(slot, "puppet_bounds_outside", False):
                puppet_box.label(text="Target outside boundary - holding", icon="ERROR")

            if getattr(slot, "realtime_puppet_status", ""):
                status_box = puppet_box.box()
                status_box.label(text="Puppet Status")
                status_box.label(text=(slot.realtime_puppet_status or "")[:180])
            if _foldout(puppet_box, context, ui, "ui_ctrl_show_puppet_settings", "Puppet Settings"):
                settings_box = puppet_box.box()
                row = settings_box.row(align=True)
                row.prop(slot, "realtime_puppet_rate_hz", text="Rate (Hz)")
                row.prop(slot, "realtime_puppet_max_step_mm", text="Max Step (mm)")
                # Spring follow
                spring_row = settings_box.row(align=True)
                spring_row.prop(slot, "puppet_spring_enabled", text="Spring Follow")
                if slot.puppet_spring_enabled:
                    sub = settings_box.column(align=True)
                    sub.prop(slot, "puppet_spring_freq", text="Frequency (Hz)")
                    sub.prop(slot, "puppet_spring_damping", text="Damping")
                    sub.prop(slot, "puppet_spring_response", text="Response")
                if slot.robot_type == "KUKA":
                    settings_box.prop(slot, "kuka_puppet_speed_pct", text="Puppet Speed (%)")
                if slot.robot_type == "XARM":
                    settings_box.prop(slot, "xarm_puppet_use_boundary", text="Use xArm Safety Boundary")
                    if slot.xarm_puppet_use_boundary:
                        b = settings_box.box()
                        b.label(text="Boundary [x_max, x_min, y_max, y_min, z_max, z_min] mm")
                        row = b.row(align=True)
                        for i in range(6):
                            row.prop(slot, "xarm_puppet_boundary_mm", index=i, text="")


class ANIMAQUINA_PT_ControlStreaming(Panel):
    bl_label = "Streaming"
    bl_idname = "ANIMAQUINA_PT_controlstreaming"
    bl_parent_id = "ANIMAQUINA_PT_control"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Animaquina"
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        # Advanced only: buffered streaming and Dynamic Sync are not part
        # of the basic connect-jog-run-export path.
        if _is_basic(context):
            return False
        return _streaming_available(get_active_slot_from_context(context))

    def draw(self, context):
        layout = self.layout
        ui = context.scene.animaquina
        slot, driver, caps = _slot_driver_caps(context)
        if slot is None:
            return
        # --- Advanced Streaming ---
        _show_streaming = (slot.robot_type == "KUKA") or (slot.robot_type == "UR" and slot.ur_backend == "ur_rtde")
        row = layout.row(align=True)
        row.prop(
            ui,
            "ui_ctrl_show_advanced_streaming",
            text="Advanced Streaming",
            emboss=False,
            icon=("TRIA_DOWN" if ui.ui_ctrl_show_advanced_streaming else "TRIA_RIGHT"),
        )
        if ui.ui_ctrl_show_advanced_streaming:
            box = layout.box()
            if slot.robot_type == "UR":
                box.label(text="UR Buffered Toolpath")
                box.prop(slot, "ur_queue_buffer_size", text="Ring Buffer Size")
                if getattr(slot, "ur_queue_health", ""):
                    st = box.box()
                    st.label(text="Stream Status")
                    st.label(text=(slot.ur_queue_health or "")[-180:])
            elif slot.robot_type == "KUKA":
                box.label(text="KUKA Dynamic Sync")
                info = box.box()
                info.label(text="Buffer Size is baked into mq_stream.", icon="INFO")
                info.label(text="After changing it: Upload, then Select Dynamic Sync.")
                row = box.row(align=True)
                row.prop(slot, "export_base_no", text="Base #")
                row.prop(slot, "export_tool_no", text="Tool #")
                box.prop(slot, "remote_path", text="Path")
                box.prop(slot, "kuka_ring_buffer_size", text="Buffer Size")
                box.prop(slot, "kuka_advance", text="Advance Lookahead")
                current_size = int(getattr(slot, "kuka_ring_buffer_size", 6) or 6)
                uploaded_size = int(getattr(slot, "kuka_stream_uploaded_ring_size", 0) or 0)
                if uploaded_size > 0 and uploaded_size != current_size:
                    warn = box.box()
                    warn.alert = True
                    warn.label(text=f"Uploaded buffer is {uploaded_size}; re-upload to apply {current_size}.")
                elif uploaded_size > 0 and bool(getattr(slot, "stream_program_uploaded", False)):
                    ok = box.box()
                    ok.label(text=f"Uploaded program buffer: {uploaded_size}")
                elif not bool(getattr(slot, "stream_program_uploaded", False)):
                    warn = box.box()
                    warn.alert = True
                    warn.label(text="Upload + Select Dynamic Sync after changing settings.")
                row = box.row(align=True)
                row.operator("object.animaquina_upload_stream", text="Upload Dynamic Sync")
                row.operator("object.animaquina_select_stream", text="Select Dynamic Sync")


# 4. Simulation (top-level, bl_order=3)

class ANIMAQUINA_PT_Newton(Panel):
    bl_label = "Simulation"
    bl_idname = "ANIMAQUINA_PT_simulation"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Animaquina"
    bl_order = 3
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        # Advanced only: validation is a tuning/verification tool.
        if _is_basic(context):
            return False
        return get_active_slot_from_context(context) is not None

    def draw(self, context):
        layout = self.layout
        slot = get_active_slot_from_context(context)
        if slot is None:
            return
            layout.enabled = False
            return
        layout.prop(slot, "simulation_mode", text="Mode")
        row = layout.row(align=True)
        if slot.simulation_mode == "BLENDER_IK":
            row.operator("object.animaquina_set_simulation", text="Set Simulation")
            row.operator("object.animaquina_clear_simulation", text="Clear Simulation")
            row = layout.row(align=True)
            row.operator("object.animaquina_blender_validate_path", text="Validate Path")
            row.operator("object.animaquina_blender_clear_path", text="Clear Path")
            layout.prop(slot, "newton_keyframe_step", text="Path Frame Step")
            layout.label(text="Uses Blender IK playback on duplicated _sim rig")
            box = layout.box()
            box.label(text="Blender IK Mode")
            box.label(text="Validate Path animates Target along the selected path.")
            box.label(text="Clear Path removes playback constraints and keyframes.")
        else:
            row.operator("object.animaquina_validate_path", text="Validate Path")
            row = layout.row(align=True)
            row.operator("object.animaquina_newton_keyframe_ik_path", text="Keyframe IK Path")
            row.operator("object.animaquina_clear_simulation", text="Clear Simulation")
            layout.prop(slot, "export_use_rotation_attribute", text="Per-Point Orientation (rotation attr)")
            if slot.export_use_rotation_attribute:
                layout.prop(slot, "export_rotation_apply_transform", text="Apply Object Transform")
            layout.prop(slot, "collision_collection", text="Collision Coll")
            row = layout.row(align=True)
            row.prop(slot, "validation_step_dt", text="dt")
            row.prop(slot, "validation_substeps", text="Substeps")
            row = layout.row(align=True)
            row.prop(slot, "newton_check_contacts", text="Check Contacts")
            row.prop(slot, "newton_ignore_robot_self_contacts", text="Ignore Self")
            layout.prop(slot, "newton_keyframe_step", text="Keyframe Step")
            layout.prop(slot, "newton_python_exe", text="Newton Python")


# --- Simulation > Validation Results (sub-panel, DEFAULT_CLOSED) ---

class ANIMAQUINA_PT_ValidationResults(Panel):
    bl_label = "Validation Results"
    bl_idname = "ANIMAQUINA_PT_validation_results"
    bl_parent_id = "ANIMAQUINA_PT_simulation"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Animaquina"
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot_from_context(context)
        if slot is None:
            return False
        return slot.simulation_mode != "BLENDER_IK"

    def draw(self, context):
        layout = self.layout
        slot = get_active_slot_from_context(context)
        if slot is None:
            return
        if getattr(slot, "validation_last_status", ""):
            box = layout.box()
            status = getattr(slot, "validation_last_status", "")
            if status in {"INVALID", "ERROR"}:
                box.alert = True
            box.label(text=f"Status: {status}")
            backend = getattr(slot, "validation_last_backend", "")
            if backend:
                box.label(text=f"Backend: {backend}")
            backend_msg = getattr(slot, "validation_last_backend_message", "")
            if backend_msg:
                box.label(text=backend_msg[:180])
            if getattr(slot, "validation_last_failure_index", -1) >= 0:
                box.label(text=f"Failure idx: {slot.validation_last_failure_index}")
            if getattr(slot, "validation_last_message", ""):
                box.label(text=slot.validation_last_message)
            worker_stderr = getattr(slot, "validation_last_worker_stderr", "")
            if worker_stderr:
                box.label(text="Worker stderr captured (see console/log)")
            contacts = getattr(slot, "validation_last_contacts", "")
            if contacts:
                box.label(text=f"Contacts: {contacts[:160]}")
            debug_txt = getattr(slot, "validation_last_debug", "")
            if debug_txt:
                box.label(text=f"Debug: {debug_txt[:160]}")
        else:
            layout.label(text="No validation run yet")

        if getattr(slot, "newton_install_status", ""):
            box = layout.box()
            status = getattr(slot, "newton_install_status", "")
            if status == "ERROR":
                box.alert = True
            box.label(text=f"Install: {status}")
            if getattr(slot, "newton_install_log", ""):
                box.label(text=(slot.newton_install_log or "")[-180:])


# 5. Export (top-level parent, bl_order=4)

class ANIMAQUINA_PT_Export(Panel):
    bl_label = "Export"
    bl_idname = "ANIMAQUINA_PT_export"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Animaquina"
    bl_order = 4
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        slot = get_active_slot_from_context(context)
        return slot is not None and slot.robot_type in {"UR", "KUKA"}

    def draw(self, context):
        pass  # parent header only; content lives in the per-brand sub-panels


# --- Export > Universal Robots (sub-panel) ---

class ANIMAQUINA_PT_ExportUR(Panel):
    bl_label = "Universal Robots"
    bl_idname = "ANIMAQUINA_PT_export_ur"
    bl_parent_id = "ANIMAQUINA_PT_export"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Animaquina"

    @classmethod
    def poll(cls, context):
        slot = get_active_slot_from_context(context)
        return slot is not None and slot.robot_type == "UR"

    def draw(self, context):
        layout = self.layout
        ui = context.scene.animaquina
        slot = get_active_slot_from_context(context)
        if slot is None:
            return

        # Step 1: Export
        layout.label(text="Step 1: Export")
        layout.prop(slot, "ur_program_name", text="Program Name")
        layout.operator("object.animaquina_export_ur", text="Export Program (.script + .urp)")
        # Program frame: base/tool, orientation, TCP and home decide where the
        # program drives the robot, so they are visible in every mode. Hiding them
        # in Basic would ship invisible defaults on exactly the values that matter
        # most. None of this needs a connected robot.
        box = layout.box()
        box.label(text="Program Frame", icon="ORIENTATION_GLOBAL")
        if slot.base_object is None:
            # get_slot_blender_base_world_matrix falls back to identity here.
            box.label(text="No robot base bound - positions relative to world origin", icon="ERROR")

        box.prop(slot, "export_use_rotation_attribute", text="Per-Point Orientation (rotation attr)")
        if slot.export_use_rotation_attribute:
            box.prop(slot, "export_rotation_apply_transform", text="Apply Object Transform")
        if not slot.export_use_rotation_attribute:
            box.label(text="Custom orientation (deg)")
            row = box.row(align=True)
            row.label(text="A")
            row.label(text="B")
            row.label(text="C")
            row = box.row(align=True)
            row.prop(slot, "export_custom_a", text="")
            row.prop(slot, "export_custom_b", text="")
            row.prop(slot, "export_custom_c", text="")
            box.operator("object.animaquina_set_ur_orientation", text="Set Orientation from Current")

        box.prop(slot, "ur_export_custom_tool", text="Custom Tool")
        if slot.ur_export_custom_tool:
            tcp_box = box.box()
            tcp_box.label(text="TCP (x, y, z, rx, ry, rz)")
            row = tcp_box.row(align=True)
            for i in range(6):
                row.prop(slot, "ur_export_tcp", index=i, text="")
            tcp_box.label(text="Payload")
            tcp_box.prop(slot, "ur_export_payload_mass", text="Mass (kg)")
            row = tcp_box.row(align=True)
            row.label(text="CoG X")
            row.label(text="CoG Y")
            row.label(text="CoG Z")
            row = tcp_box.row(align=True)
            for i in range(3):
                row.prop(slot, "ur_export_payload_cog", index=i, text="")

        box.label(text="Home Joints (deg)")
        row = box.row(align=True)
        for i in range(6):
            row.prop(slot, "ur_export_home", index=i, text=f"J{i + 1}")
        box.operator("object.animaquina_set_home", text="Set Home from Current")

        # Basic hides the speed fields, so show what will be exported instead of
        # letting a motion value go unseen.
        if _is_basic(context):
            row = layout.row()
            row.alert = slot.ur_export_vel > 0.25
            row.label(
                text=f"Motion: lin {slot.ur_export_vel:.2f} m/s, joint {slot.ur_export_joint_vel:.2f} rad/s",
                icon="INFO",
            )
        if _foldout(layout, context, ui, "ui_ur_show_export_settings", "Motion Settings"):
            box = layout.box()

            box.label(text="Linear (movel)")
            row = box.row(align=True)
            row.label(text="Vel (m/s)")
            row.label(text="Acc (m/s^2)")
            row.label(text="Blend (m)")
            row = box.row(align=True)
            if slot.ur_export_vel > 0.25:
                row.alert = True
            row.prop(slot, "ur_export_vel", text="")
            row.prop(slot, "ur_export_acc", text="")
            row.prop(slot, "ur_export_blend", text="")

            box.label(text="Joint (movej)")
            row = box.row(align=True)
            row.label(text="Vel (rad/s)")
            row.label(text="Acc (rad/s^2)")
            row = box.row(align=True)
            row.prop(slot, "ur_export_joint_vel", text="")
            row.prop(slot, "ur_export_joint_acc", text="")

            # Per-point index variable (poll it → PhyNodes/MQTT)
            row = box.row(align=True)
            row.prop(slot, "export_write_point_index", text="Write Point Index")
            sub = row.row(align=True)
            sub.active = slot.export_write_point_index
            sub.prop(slot, "export_point_index_var", text="")
            if slot.export_write_point_index and not slot.export_point_index_var.startswith("output_"):
                box.label(text="UR: use output_int_register_0 to read it back over RTDE", icon="INFO")

            if slot.ur_export_vel > 0.25:
                warn = box.box()
                warn.alert = True
                warn.label(text="Velocity > 0.25 m/s (250 mm/s)", icon="ERROR")

        # Step 2: Stage
        layout.separator()
        layout.label(text="Step 2: Stage")
        layout.operator("object.animaquina_ur_save_program", text="Stage to Robot")
        if _foldout(layout, context, ui, "ui_ur_show_stage_settings", "Stage Settings"):
            box = layout.box()
            box.prop(slot, "ur_stage_transport", text="Stage Via")
            if slot.ur_stage_transport == "sftp":
                box.prop(slot, "ur_remote_path", text="Remote Path")
                row = box.row(align=True)
                row.prop(slot, "ur_sftp_user", text="User")
                row.prop(slot, "ur_sftp_port", text="Port")
                box.prop(slot, "ur_sftp_password", text="Password")
            else:
                box.label(text="Runtime Script")
                box.label(text="No file-transfer settings are needed.")

        # Step 3: Run
        layout.separator()
        layout.label(text="Step 3: Run Program")
        layout.prop(slot, "ur_launcher_urp", text="Launcher URP")
        row = layout.row(align=True)
        row.operator("object.animaquina_ur_load_program", text="Run Program")
        reload_op = row.operator("object.animaquina_ur_load_program", text="Reload + Run")
        reload_op.force_reload = True
        row = layout.row(align=True)
        row.operator("object.animaquina_ur_pause_program", text="Pause Program")
        row.operator("object.animaquina_ur_stop_program", text="Stop Program")
        if _foldout(layout, context, ui, "ui_ur_show_play_settings", "Run Settings"):
            box = layout.box()
            box.prop(slot, "ur_dashboard_port", text="Dash Port")
            if slot.ur_stage_transport != "sftp":
                box.label(text="Run uses Dashboard load/play on a launcher URP.")
                box.label(text="Use SFTP stage to upload/update launcher files.")

        if getattr(slot, "ur_transfer_status", ""):
            box = layout.box()
            status = getattr(slot, "ur_transfer_status", "")
            if status == "ERROR":
                box.alert = True
            box.label(text=f"Program Transfer: {status}")
            if getattr(slot, "ur_transfer_log", ""):
                box.label(text="Program Log")
                box.label(text=(slot.ur_transfer_log or "")[-180:])


# --- Export > KUKA (sub-panel) ---

class ANIMAQUINA_PT_ExportKUKA(Panel):
    bl_label = "KUKA"
    bl_idname = "ANIMAQUINA_PT_export_kuka"
    bl_parent_id = "ANIMAQUINA_PT_export"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Animaquina"

    @classmethod
    def poll(cls, context):
        slot = get_active_slot_from_context(context)
        return slot is not None and slot.robot_type == "KUKA"

    def draw(self, context):
        layout = self.layout
        ui = context.scene.animaquina
        slot = get_active_slot_from_context(context)
        if slot is None:
            return
        # Step 1: Export
        layout.label(text="Step 1: Export")
        layout.prop(slot, "program_name", text="Program Name")
        layout.operator("object.animaquina_export_krl", text="Export Program (.src)")
        # Program frame: base/tool, orientation, TCP and home decide where the
        # program drives the robot, so they are visible in every mode. Hiding them
        # in Basic would ship invisible defaults on exactly the values that matter
        # most. None of this needs a connected robot.
        box = layout.box()
        box.label(text="Program Frame", icon="ORIENTATION_GLOBAL")
        if slot.base_object is None:
            # get_slot_blender_base_world_matrix falls back to identity here.
            box.label(text="No robot base bound - positions relative to world origin", icon="ERROR")

        row = box.row(align=True)
        row.prop(slot, "export_base_no", text="Base #")
        row.prop(slot, "export_tool_no", text="Tool #")

        box.prop(slot, "export_use_rotation_attribute", text="Per-Point Orientation (rotation attr)")
        if slot.export_use_rotation_attribute:
            box.prop(slot, "export_rotation_apply_transform", text="Apply Object Transform")
        if not slot.export_use_rotation_attribute:
            box.label(text="Custom orientation (deg)")
            row = box.row(align=True)
            row.label(text="A")
            row.label(text="B")
            row.label(text="C")
            row = box.row(align=True)
            row.prop(slot, "export_custom_a", text="")
            row.prop(slot, "export_custom_b", text="")
            row.prop(slot, "export_custom_c", text="")
            box.operator("object.animaquina_set_ur_orientation", text="Set Orientation from Current")

        box.label(text="Home Joints (deg)")
        row = box.row(align=True)
        for i in range(6):
            row.prop(slot, "kuka_export_home", index=i, text=f"A{i + 1}")
        box.operator("object.animaquina_set_home", text="Set Home from Current")
        if bool(getattr(slot, "is_connected", False)):
            box.operator("object.animaquina_store_home_to_robot", text="Store Home to Robot")

        # Basic hides the speed fields, so show what will be exported instead of
        # letting a motion value go unseen.
        if _is_basic(context):
            row = layout.row()
            row.alert = slot.export_speed > 15 or slot.export_lin_speed > 0.2
            row.label(
                text=f"Motion: PTP {slot.export_speed:.0f}%, lin {slot.export_lin_speed:.2f} m/s",
                icon="INFO",
            )
        if _foldout(layout, context, ui, "ui_kuka_show_export_settings", "Motion Settings"):
            box = layout.box()

            box.label(text="KUKA motion")
            row = box.row(align=True)
            row.label(text="Speed (%)")
            row.label(text="Acc (%)")
            row.label(text="Lin (m/s)")
            row.label(text="Advance")
            row = box.row(align=True)
            if slot.export_speed > 15 or slot.export_lin_speed > 0.2:
                row.alert = True
            row.prop(slot, "export_speed", text="")
            row.prop(slot, "export_acc", text="")
            row.prop(slot, "export_lin_speed", text="")
            row.prop(slot, "export_advance", text="")

            # Per-point index variable (poll it → PhyNodes/MQTT)
            row = box.row(align=True)
            row.prop(slot, "export_write_point_index", text="Write Point Index")
            sub = row.row(align=True)
            sub.active = slot.export_write_point_index
            sub.prop(slot, "export_point_index_var", text="")

            if slot.export_speed > 15 or slot.export_lin_speed > 0.2:
                warn = box.box()
                warn.alert = True
                if slot.export_speed > 15 and slot.export_lin_speed > 0.2:
                    warn.label(text="Speed >15% and Lin speed >0.2 m/s (200 mm/s)", icon="ERROR")
                elif slot.export_speed > 15:
                    warn.label(text="Speed >15%", icon="ERROR")
                else:
                    warn.label(text="Lin speed >0.2 m/s (200 mm/s)", icon="ERROR")

        # Step 2a: Stage to Robot (upload .src + select + play from controller)
        layout.separator()
        layout.label(text="Step 2: Run Program")
        row = layout.row(align=True)
        row.operator("object.animaquina_stage_program", text="Stage to Robot")
        row = layout.row(align=True)
        row.operator("object.animaquina_kuka_play_program", text="Run Program")
        row.operator("object.animaquina_kuka_stop_program", text="Stop Program")
        cancel_row = layout.row(align=True)
        cancel_row.alert = bool(getattr(slot, "realtime_puppet_active", False))
        cancel_row.operator("object.animaquina_kuka_cancel_program", text="Cancel Program", icon="CANCEL")
        if _foldout(layout, context, ui, "ui_kuka_show_stage_settings", "Stage Settings"):
            box = layout.box()
            box.prop(slot, "remote_path", text="Path")
            box.label(text="Stage uploads .src and selects it on the controller.")
            box.label(text="Run/Stop/Cancel use C3 Bridge ProgramControl.")
            box.label(text="Robot must be in AUT or AUT EXT mode.")
            row = box.row(align=True)
            row.operator("object.animaquina_save_program", text="Upload Only")
            row.operator("object.animaquina_load_program", text="Select Only")

class ANIMAQUINA_PT_Debug(Panel):
    bl_label = "Debug"
    bl_idname = "ANIMAQUINA_PT_debug"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Animaquina"
    bl_order = 6
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        # Advanced only: developer diagnostics.
        if _is_basic(context):
            return False
        return get_active_slot_from_context(context) is not None

    def draw(self, context):
        pass  # parent header only; content lives in sub-panels


class ANIMAQUINA_PT_DependencyDebug(Panel):
    bl_label = "Dependencies"
    bl_idname = "ANIMAQUINA_PT_dependency_debug"
    bl_parent_id = "ANIMAQUINA_PT_debug"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Animaquina"

    @classmethod
    def poll(cls, context):
        slot = get_active_slot_from_context(context)
        return slot is not None

    def draw(self, context):
        from .operators import DEP_PACKAGES, _is_package_installed

        layout = self.layout
        slot = get_active_slot_from_context(context)
        if slot is None:
            return

        for pip_name, label, description in DEP_PACKAGES:
            row = layout.row(align=True)
            installed = _is_package_installed(pip_name)
            icon = "CHECKMARK" if installed else "CANCEL"
            row.label(text=label, icon=icon)
            row.label(text=description)
            btn_text = "Reinstall" if installed else "Install"
            op = row.operator("object.animaquina_install_package", text=btn_text, icon="IMPORT")
            op.package = pip_name


class ANIMAQUINA_PT_DatasetRecorder(Panel):
    bl_label = "Dataset Recorder"
    bl_idname = "ANIMAQUINA_PT_dataset_recorder"
    bl_parent_id = "ANIMAQUINA_PT_debug"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Animaquina"
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        return get_active_slot_from_context(context) is not None

    def draw(self, context):
        from ..runtime import recorder

        layout = self.layout
        slot = get_active_slot_from_context(context)
        if slot is None:
            return

        session = recorder.get(slot.uid)
        active = session is not None and session.active

        col = layout.column(align=True)
        col.enabled = not active   # locked while running: they go into the header
        col.prop(slot, "record_name", text="Name")
        col.prop(slot, "record_max_samples", text="Max Samples")

        if active:
            row = layout.row()
            row.alert = True
            row.scale_y = 1.3
            row.operator("object.animaquina_record_stop", text="Stop Recording", icon="CANCEL")
            layout.label(
                text=f"{slot.record_sample_count} samples · {slot.record_elapsed_s:.1f} s",
                icon="RADIOBUT_ON",
            )
            if getattr(slot, "target_object", None) is None:
                layout.label(text="No Target: cmd_* columns empty", icon="INFO")
            return

        row = layout.row()
        row.scale_y = 1.3
        row.operator("object.animaquina_record_start", text="Record", icon="REC")
        var_count = sum(
            1 for item in getattr(slot, "debug_vars", [])
            if bool(getattr(item, "enabled", True)) and str(getattr(item, "var_name", "") or "").strip()
        )
        if var_count:
            layout.label(text=f"+ {var_count} polled variable column(s)", icon="LINENUMBERS_ON")
        if not slot.is_connected:
            layout.label(text="Connect the robot to record", icon="INFO")
        elif not slot.polling_enabled:
            layout.label(text="Enable Realtime polling to record", icon="INFO")

        if session is None or not session.count():
            return

        box = layout.box()
        note = " (cap reached)" if session.overflow else ""
        box.label(text=f"Last: {session.count()} samples · {session.elapsed_s():.1f} s{note}")
        if slot.record_text_name:
            box.label(text=slot.record_text_name, icon="TEXT")
        box.operator("object.animaquina_record_save_csv", text="Save .csv", icon="EXPORT")


class ANIMAQUINA_PT_AdvancedDebug(Panel):
    bl_label = "Variables"
    bl_idname = "ANIMAQUINA_PT_advanced_debug"
    bl_parent_id = "ANIMAQUINA_PT_debug"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Animaquina"
    bl_options = {'DEFAULT_CLOSED'}

    # Variable-name hints per robot type
    _VAR_HINTS = {
        "KUKA": "KUKA variable (e.g. $OV_PRO, $MODE_OP)",
        "UR": "UR variable (e.g. output_double_register_0, robot_mode)",
        "XARM": "xArm variable (e.g. cgpio_digital_input_0, state)",
    }

    @classmethod
    def poll(cls, context):
        slot = get_active_slot_from_context(context)
        return slot is not None

    def draw(self, context):
        layout = self.layout
        slot = get_active_slot_from_context(context)
        if slot is None:
            return

        # Canonical run state (run_idx contract — what PhyNodes reads)
        run_source = getattr(slot, "run_source", "NONE")
        if run_source != "NONE":
            box = layout.box()
            row = box.row(align=True)
            row.label(text=f"Run [{run_source}]", icon="PLAY")
            count = int(getattr(slot, "run_count", 0))
            idx = int(getattr(slot, "run_idx", -1))
            row.label(text=f"IDX {idx}" + (f" / {count - 1}" if count > 0 else ""))
            row.operator("object.animaquina_clear_run_state", text="", icon="X")
            obj = getattr(slot, "run_object", None)
            if obj is not None:
                box.label(text=f"Path: {obj.name}", icon="MESH_DATA")

        hint = self._VAR_HINTS.get(getattr(slot, "robot_type", ""), "Add variable names to poll")
        layout.label(text=hint)
        row = layout.row(align=True)
        row.prop(slot, "debug_new_var", text="Var")
        row.operator("object.animaquina_debug_add_var", text="", icon="ADD")

        if not slot.debug_vars:
            layout.label(text="No debug variables added")
            return

        if not slot.is_connected:
            box = layout.box()
            box.alert = True
            box.label(text="Connect robot for realtime values", icon="INFO")

        for i, item in enumerate(slot.debug_vars):
            box = layout.box()
            row = box.row(align=True)
            row.prop(item, "enabled", text="")
            row.prop(item, "var_name", text="")
            if getattr(item, "auto_added", False):
                # Tracked automatically from Export > Write Point Index, so the
                # index reaches run_idx without adding it here by hand.
                row.label(text="", icon="AUTO")
            op = row.operator("object.animaquina_debug_remove_var", text="", icon="X")
            op.index = i

            val_row = box.row()
            val_row.enabled = False
            val_row.prop(item, "value", text="Value")
            if getattr(item, "error", ""):
                err_row = box.row()
                err_row.alert = True
                err_row.label(text=item.error[:140], icon="ERROR")


# Legacy alias
ANIMAQUINA_PT_KukaAdvancedDebug = ANIMAQUINA_PT_AdvancedDebug


# 6. Info (top-level, bl_order=5, DEFAULT_CLOSED)

class ANIMAQUINA_PT_Info(Panel):
    bl_label = "Info"
    bl_idname = "ANIMAQUINA_PT_info"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Animaquina"
    bl_order = 5
    bl_options = {'DEFAULT_CLOSED'}

    @classmethod
    def poll(cls, context):
        return get_active_slot_from_context(context) is not None

    def draw(self, context):
        layout = self.layout
        slot = get_active_slot_from_context(context)
        if slot is None:
            return
        offline = not slot.is_connected
        col = layout.column(align=True)

        def _read_only_prop(row, slot, prop_name, index, text):
            sub = row.row()
            sub.enabled = False
            sub.prop(slot, prop_name, index=index, text=text)

        col.label(text="TCP")
        for i, axis in enumerate(("X", "Y", "Z")):
            row = col.row()
            _read_only_prop(row, slot, "tcp_pos_m", i, axis)
        for i, axis in enumerate(("A", "B", "C")):
            row = col.row()
            _read_only_prop(row, slot, "tcp_euler_deg", (2, 1, 0)[i], axis)

        col.label(text="Joints (deg)")
        for i in range(6):
            row = col.row()
            if offline:
                row.prop(slot, "joints_deg", index=i, text="J%d" % (i + 1))
            else:
                _read_only_prop(row, slot, "joints_deg", i, "J%d" % (i + 1))

        # Home joints (used by Go Home and export)
        col.separator()
        _home_props = {"UR": ("ur_export_home", "J"), "KUKA": ("kuka_export_home", "A"), "XARM": ("xarm_export_home", "J")}
        hp = _home_props.get(slot.robot_type)
        if hp:
            prop_name, prefix = hp
            col.label(text="Home Joints (deg)")
            for i in range(6):
                col.prop(slot, prop_name, index=i, text=f"{prefix}{i + 1}")
            col.operator("object.animaquina_set_home", text="Set Home from Current")
            if slot.robot_type == "KUKA":
                row = col.row()
                row.enabled = bool(slot.is_connected)
                row.operator("object.animaquina_store_home_to_robot", text="Store Home to Robot")

        # Tool/Base offset — editable when offline, read-only when connected
        if offline or slot.robot_type == "KUKA":
            col.separator()
            col.label(text="Base offset")
            if offline:
                for i, axis in enumerate(("X", "Y", "Z")):
                    row = col.row()
                    row.prop(slot, "base_pos_m", index=i, text=axis)
                for i, axis in enumerate(("A", "B", "C")):
                    row = col.row()
                    row.prop(slot, "base_euler_deg", index=i, text=axis)
            elif getattr(slot, "base_frame_valid", False):
                for i, axis in enumerate(("X", "Y", "Z")):
                    row = col.row()
                    _read_only_prop(row, slot, "base_pos_m", i, axis)
                for i, axis in enumerate(("A", "B", "C")):
                    row = col.row()
                    _read_only_prop(row, slot, "base_euler_deg", i, axis)
            else:
                col.label(text="(not available)")

            col.label(text="Tool offset")
            if offline:
                for i, axis in enumerate(("X", "Y", "Z")):
                    row = col.row()
                    row.prop(slot, "tool_frame_pos_m", index=i, text=axis)
                for i, axis in enumerate(("A", "B", "C")):
                    row = col.row()
                    row.prop(slot, "tool_frame_euler_deg", index=i, text=axis)
            elif getattr(slot, "tool_frame_valid", False):
                for i, axis in enumerate(("X", "Y", "Z")):
                    row = col.row()
                    _read_only_prop(row, slot, "tool_frame_pos_m", i, axis)
                for i, axis in enumerate(("A", "B", "C")):
                    row = col.row()
                    _read_only_prop(row, slot, "tool_frame_euler_deg", i, axis)
            else:
                col.label(text="(not available)")


# Text Editor panel — stream any .src via Dynamic Sync

class ANIMAQUINA_PT_TextEditor(Panel):
    bl_label = "Animaquina"
    bl_idname = "ANIMAQUINA_PT_text_editor"
    bl_space_type = "TEXT_EDITOR"
    bl_region_type = "UI"
    bl_category = "Animaquina"

    @classmethod
    def poll(cls, context):
        slot = get_active_slot_from_context(context)
        return slot is not None and slot.robot_type == "KUKA" and slot.is_connected

    def draw(self, context):
        layout = self.layout
        slot = get_active_slot_from_context(context)
        if slot is None:
            return

        # Show active text block
        st = context.space_data
        if st and getattr(st, "text", None):
            layout.label(text=f"Text: {st.text.name}", icon="TEXT")
        else:
            layout.label(text="No text block open", icon="INFO")

        # Dynamic Sync setup
        layout.separator()
        layout.label(text="Dynamic Sync")
        row = layout.row(align=True)
        row.operator("object.animaquina_upload_stream", text="Upload")
        row.operator("object.animaquina_select_stream", text="Select")

        # Stream controls
        layout.separator()
        row = layout.row(align=True)
        row.operator("object.animaquina_kuka_stream_program", text="Run Program")
        row.operator("object.animaquina_kuka_stop_program", text="Stop Program")
        cancel_row = layout.row(align=True)
        cancel_row.alert = bool(getattr(slot, "realtime_puppet_active", False))
        cancel_row.operator("object.animaquina_kuka_cancel_program", text="Cancel Program", icon="CANCEL")
        layout.label(text="Run parses active .src text and starts streaming.")


# Registration list (parents MUST come before their children)

PANEL_CLASSES = [
    ANIMAQUINA_UL_RobotList,
    ANIMAQUINA_PT_Registry,
    ANIMAQUINA_PT_Setup,
    ANIMAQUINA_PT_Connection,        # child of Setup
    ANIMAQUINA_PT_SceneObjects,      # child of Setup
    ANIMAQUINA_PT_Control,           # parent header
    ANIMAQUINA_PT_ControlProgram,    # child of Control
    ANIMAQUINA_PT_ControlToolpath,   # child of Control
    ANIMAQUINA_PT_ControlPuppet,     # child of Control
    ANIMAQUINA_PT_ControlStreaming,  # child of Control (Advanced only)
    ANIMAQUINA_PT_Newton,
    ANIMAQUINA_PT_ValidationResults, # child of Simulation
    ANIMAQUINA_PT_Export,            # parent header
    ANIMAQUINA_PT_ExportUR,          # child of Export
    ANIMAQUINA_PT_ExportKUKA,        # child of Export
    ANIMAQUINA_PT_Info,
    ANIMAQUINA_PT_TextEditor,        # Text Editor sidebar
    ANIMAQUINA_PT_Debug,             # parent header
    ANIMAQUINA_PT_DependencyDebug,   # child of Debug
    ANIMAQUINA_PT_AdvancedDebug, # child of Debug
    ANIMAQUINA_PT_DatasetRecorder,   # child of Debug
]
