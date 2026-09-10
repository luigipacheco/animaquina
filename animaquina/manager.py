# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
# Animaquina — Robot Manager: slot.uid → driver, polling (Section 9)
#
# Threading model: each connected slot runs a _SlotPollThread that calls driver.read_*()
# in the background. The Blender timer (_poll_all) only reads the thread cache — no I/O
# on the main thread — then writes slot properties and calls apply_full_pose().

import math
import threading
import time
import bpy

from .runtime import recorder
from .runtime import rig_apply
from . import run_state
from animaquina_core.drivers.base import DriverBase, CAP_MANUAL_MODE, CAP_READ_BASE
from .runtime.rig_apply import apply_full_pose
from animaquina_core.runtime.conversions import rad2deg

# Lazy driver imports — only loaded when the robot brand is actually used.
# This allows distributing animaquina_core without all driver files.
_URDriver = None
_KUKADriver = None
_XArmDriver = None


def _get_ur_driver_class():
    global _URDriver
    if _URDriver is None:
        try:
            from animaquina_core.drivers.ur_driver import URDriver
            _URDriver = URDriver
        except ImportError:
            _URDriver = False
    return _URDriver if _URDriver is not False else None


def _get_kuka_driver_class():
    global _KUKADriver
    if _KUKADriver is None:
        try:
            from animaquina_core.drivers.kuka_driver import KUKADriver
            _KUKADriver = KUKADriver
        except ImportError:
            _KUKADriver = False
    return _KUKADriver if _KUKADriver is not False else None


def _get_xarm_driver_class():
    global _XArmDriver
    if _XArmDriver is None:
        try:
            from animaquina_core.drivers.xarm_driver import XArmDriver
            _XArmDriver = XArmDriver
        except ImportError:
            _XArmDriver = False
    return _XArmDriver if _XArmDriver is not False else None

# Robot slot property that stores export/home joints (deg)
_HOME_PROP_BY_TYPE = {
    "UR": "ur_export_home",
    "KUKA": "kuka_export_home",
    "XARM": "xarm_export_home",
}

# Driver instances — kept for disconnect/capability checks
_drivers = {}        # uid → driver instance
# Background poll threads
_poll_threads = {}   # uid → _SlotPollThread
# Thread-populated data cache; written by threads, read by timer on main thread.
# Dict per uid; fields: tcp_pos_m, tcp_euler_rad, joints_deg, base_pos_m,
# base_euler_rad, base_frame_valid, tool_frame_pos_m, tool_frame_euler_rad,
# tool_frame_valid, error.
_thread_cache = {}   # uid → dict
_cache_lock = threading.Lock()

_timer_registered = False

# Debug-only instrumentation (kept off by default; enable to log timer callback ms)
_DEBUG_POLL = False
_DEBUG_LOG_EVERY = 100
_debug_poll_counter = 0


# Background poll thread

class _SlotPollThread(threading.Thread):
    """Reads robot data in a background thread so Blender's main thread is never blocked by network I/O."""

    # Back-off ceiling after consecutive failed cycles. The loop never exits on
    # error, so without this a dead or saturated link is polled at full rate
    # forever - each failed read tearing down and rebuilding a socket.
    MAX_ERROR_BACKOFF = 2.0

    # Consecutive failures before a debug variable is dropped from the poll
    # rotation. An undeclared variable never starts working on its own, and
    # retrying it forever costs one socket round-trip per cycle.
    MAX_VAR_FAILURES = 5

    def __init__(self, driver: DriverBase, uid: str, rate_hz: float):
        super().__init__(daemon=True, name=f"animaquina-{uid[:8]}")
        self._driver = driver
        self._uid = uid
        self._base_rate_hz = max(1.0, min(50.0, float(rate_hz)))
        self._rate_hz = self._base_rate_hz
        self._stop_event = threading.Event()
        self._force_aux = threading.Event()
        self._debug_var_names = []
        self._debug_lock = threading.Lock()
        self._rate_lock = threading.Lock()

    def force_aux_refresh(self) -> None:
        """Request an aux (base/tool) read on the next iteration regardless of the normal cadence."""
        self._force_aux.set()

    def set_rate(self, rate_hz: float) -> None:
        """Change poll cadence while running.

        Used to throttle polling during streaming: the poll thread and the
        streaming refill on the main thread share one socket and one driver
        lock, so a full-rate poll starves refill and blocks Blender's UI thread
        behind an in-flight 2 s socket timeout.
        """
        with self._rate_lock:
            self._rate_hz = max(1.0, min(50.0, float(rate_hz)))

    def restore_rate(self) -> None:
        """Return to the cadence this thread was created with."""
        with self._rate_lock:
            self._rate_hz = self._base_rate_hz

    def set_debug_var_names(self, names) -> None:
        """Set list of controller variables to poll each cycle."""
        cleaned = []
        for n in names or []:
            s = str(n or "").strip()
            if s and s not in cleaned:
                cleaned.append(s)
        with self._debug_lock:
            self._debug_var_names = cleaned

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        driver = self._driver
        uid = self._uid
        has_tcp = hasattr(driver, "read_tcp")
        has_joints = hasattr(driver, "read_joints")
        has_base = hasattr(driver, "read_base") and bool(driver.capabilities() & CAP_READ_BASE)
        has_tool = hasattr(driver, "read_tool")
        aux_every_n = 3
        tick = 0
        consecutive_errors = 0
        var_failures = {}   # debug var name -> consecutive read failures

        while not self._stop_event.is_set():
            t_start = time.perf_counter()
            # Re-read each cycle: set_rate() can change this mid-run (streaming
            # throttle). The old code computed the interval once, before the
            # loop, so any later rate change had no effect.
            with self._rate_lock:
                interval = 1.0 / self._rate_hz
            tick += 1
            force_aux = self._force_aux.is_set()
            read_aux = force_aux or (tick % aux_every_n == 0)

            # Per-read error isolation: a failed joints parse shouldn't discard good TCP data
            data: dict = {}
            errors: list = []
            # Tracked separately from `data`, which is never empty: the debug-var
            # keys below are written on every cycle, so `not data` was always
            # False and the error branch could never fire.
            pose_ok = False

            if has_tcp:
                try:
                    pos_m, euler_rad = driver.read_tcp()
                    data["tcp_pos_m"] = pos_m
                    data["tcp_euler_rad"] = euler_rad
                    pose_ok = True
                except Exception as e:
                    errors.append(f"TCP: {e}")

            if has_joints:
                try:
                    data["joints_deg"] = driver.read_joints()
                    pose_ok = True
                except Exception as e:
                    errors.append(f"Joints: {e}")

            if has_base and read_aux:
                try:
                    base_pos, base_euler = driver.read_base()
                    data["base_pos_m"] = base_pos
                    data["base_euler_rad"] = base_euler
                    data["base_frame_valid"] = True
                except Exception as e:
                    errors.append(f"Base: {e}")

            if has_tool and read_aux:
                try:
                    tool_pos, tool_euler = driver.read_tool()
                    data["tool_frame_pos_m"] = tool_pos
                    data["tool_frame_euler_rad"] = tool_euler
                    data["tool_frame_valid"] = True
                except Exception:
                    data["tool_frame_valid"] = False

            with self._debug_lock:
                debug_vars = list(self._debug_var_names)
            if debug_vars and hasattr(driver, "read_var"):
                debug_values = {}
                debug_errors = {}
                for var_name in debug_vars:
                    # Quarantine a variable that keeps failing. A name that is
                    # not declared in the controller's $CONFIG.DAT fails on
                    # every single cycle, and each attempt is a full round-trip
                    # on the same socket the streaming refill needs. Stop paying
                    # for it, but keep reporting why so the UI still shows the
                    # cause instead of going quiet.
                    if var_failures.get(var_name, 0) >= self.MAX_VAR_FAILURES:
                        debug_errors[var_name] = (
                            f"not readable after {self.MAX_VAR_FAILURES} attempts "
                            "- declare it in $CONFIG.DAT, then reconnect"
                        )
                        continue
                    try:
                        debug_values[var_name] = str(driver.read_var(var_name))
                        var_failures.pop(var_name, None)
                    except Exception as e:
                        var_failures[var_name] = var_failures.get(var_name, 0) + 1
                        debug_errors[var_name] = str(e)
                data["debug_var_values"] = debug_values
                data["debug_var_errors"] = debug_errors
            else:
                data["debug_var_values"] = {}
                data["debug_var_errors"] = {}

            if force_aux:
                self._force_aux.clear()

            with _cache_lock:
                entry = _thread_cache.setdefault(uid, {})
                entry.update(data)
                if errors and not pose_ok:
                    entry["error"] = "; ".join(errors)
                else:
                    entry.pop("error", None)

            # Dataset recording happens here, not in the timer callback: this is
            # the moment the values were actually read off the controller, so
            # timer jitter and error back-off never contaminate the timestamps.
            # A failed cycle is skipped rather than logged as a stale repeat.
            if pose_ok:
                try:
                    recorder.record(
                        uid,
                        data.get("joints_deg"),
                        data.get("tcp_pos_m"),
                        data.get("tcp_euler_rad"),
                        data.get("debug_var_values"),
                    )
                except Exception:
                    pass

            # Exponential back-off while the link is failing. Every failed read
            # on the KUKA transport marks the connection dead and makes the next
            # one reconnect, so retrying at full rate is what turns a single
            # fault into a socket storm that stalls the whole machine.
            if errors and not pose_ok:
                consecutive_errors += 1
            else:
                consecutive_errors = 0

            if consecutive_errors:
                backoff = min(self.MAX_ERROR_BACKOFF, interval * (2 ** consecutive_errors))
            else:
                backoff = interval

            elapsed = time.perf_counter() - t_start
            self._stop_event.wait(timeout=max(0.0, backoff - elapsed))


# Slot lifecycle helpers

def _reset_slot_twin(slot):
    """On disconnect: reset rig, cache, TCP, base. REFERENCE: oldversions/.../animaquinakuka __init__.py disconnect() ~131."""
    try:
        # Zero cache (old: sna_x/y/z/a/b/c, sna_j1..j6, sna_base_*)
        slot.tcp_pos_m = (0.0, 0.0, 0.0)
        slot.tcp_euler_rad = (0.0, 0.0, 0.0)
        slot.tcp_euler_deg = (0.0, 0.0, 0.0)
        slot.base_pos_m = (0.0, 0.0, 0.0)
        slot.base_euler_rad = (0.0, 0.0, 0.0)
        slot.base_euler_deg = (0.0, 0.0, 0.0)
        slot.joints_deg = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        slot.last_error = ""
        for item in getattr(slot, "debug_vars", []):
            item.value = ""
            item.error = ""

        # Reset rig armature pose bones to identity (old: armature_obj.pose.bones matrix_basis.identity())
        arm = getattr(slot, "rig_armature", None)
        if arm is not None and getattr(arm, "type", None) == "ARMATURE" and getattr(arm, "pose", None):
            try:
                for pb in arm.pose.bones:
                    pb.matrix_basis.identity()
            except (ReferenceError, AttributeError):
                pass

        # Reset TCP object (old: location [0,0,0], rotation_euler [-pi/2, pi, pi] to avoid gimbal lock)
        tcp = getattr(slot, "tcp_object", None)
        if tcp is not None and tcp.name in bpy.data.objects:
            try:
                tcp.location = (0.0, 0.0, 0.0)
                tcp.rotation_euler = (-math.pi / 2, math.pi, math.pi)
            except (ReferenceError, AttributeError):
                pass

        # Reset base object (old: robot_name + "_base", location/rotation [0,0,0])
        base = getattr(slot, "base_object", None)
        if base is not None and base.name in bpy.data.objects:
            try:
                _last_applied_base.pop(base.name, None)
                base.location = (0.0, 0.0, 0.0)
                base.rotation_euler = (0.0, 0.0, 0.0)
            except (ReferenceError, AttributeError):
                pass

        if getattr(bpy.context, "view_layer", None):
            bpy.context.view_layer.update()
    except Exception as e:
        print("Animaquina reset_slot_twin:", e)


def get_driver_for_slot(slot):
    if slot is None:
        return None
    return _drivers.get(slot.uid)


def request_aux_refresh(slot) -> None:
    """Force base/tool reads on the next thread iteration for this slot."""
    if slot is None:
        return
    uid = getattr(slot, "uid", "")
    thread = _poll_threads.get(uid)
    if thread is not None:
        thread.force_aux_refresh()


def create_driver(robot_type: str) -> DriverBase:
    if robot_type == "UR":
        cls = _get_ur_driver_class()
        return cls() if cls else None
    if robot_type == "KUKA":
        cls = _get_kuka_driver_class()
        return cls() if cls else None
    if robot_type == "XARM":
        cls = _get_xarm_driver_class()
        return cls() if cls else None
    return None


def _teardown_slot_io(uid: str) -> None:
    """Stop and drop the driver + poll thread registered for a uid.

    Kept separate from disconnect_slot() because it touches only the runtime
    maps, never slot properties or the twin - connect_slot() uses it to clear a
    stale session before installing a new one.
    """
    driver = _drivers.pop(uid, None)
    if driver is not None:
        try:
            driver.disconnect()
        except Exception:
            pass
    thread = _poll_threads.pop(uid, None)
    if thread is not None:
        thread.stop()
        thread.join(timeout=1.0)
    with _cache_lock:
        _thread_cache.pop(uid, None)


def set_poll_rate(slot, rate_hz: float) -> None:
    """Throttle this slot's background polling (see _SlotPollThread.set_rate)."""
    if slot is None:
        return
    thread = _poll_threads.get(getattr(slot, "uid", ""))
    if thread is not None:
        thread.set_rate(rate_hz)


def restore_poll_rate(slot) -> None:
    """Undo set_poll_rate() - back to the scene's configured poll rate."""
    if slot is None:
        return
    thread = _poll_threads.get(getattr(slot, "uid", ""))
    if thread is not None:
        thread.restore_rate()


def connect_slot(slot) -> str:
    """Create driver, connect, start poll thread, register timer. Returns '' or error."""
    global _drivers, _poll_threads, _timer_registered
    # Never orphan a previous session for this uid. _drivers/_poll_threads are
    # keyed by uid, so a second connect_slot() for the same slot used to assign
    # over both entries: the old poll thread kept running forever (it only exits
    # on its stop event, never on error), holding its own socket open and
    # polling the robot at full rate for the life of the Blender session.
    if slot.uid in _drivers or slot.uid in _poll_threads:
        _teardown_slot_io(slot.uid)

    driver = create_driver(slot.robot_type)
    if driver is None:
        return f"Unsupported robot type: {slot.robot_type}"
    err = driver.connect(slot.endpoint or "", slot.port or 0, slot=slot)
    if err:
        return err

    _drivers[slot.uid] = driver
    slot.is_connected = True
    slot.last_error = ""

    # Seed cache immediately with live robot values so first move/path never starts from zeros.
    try:
        if hasattr(driver, "read_tcp"):
            pos_m, euler_rad = driver.read_tcp()
            slot.tcp_pos_m = pos_m
            slot.tcp_euler_rad = euler_rad
            slot.tcp_euler_deg = (rad2deg(euler_rad[0]), rad2deg(euler_rad[1]), rad2deg(euler_rad[2]))
        if hasattr(driver, "read_joints"):
            joints = driver.read_joints()
            for j in range(min(6, len(joints))):
                slot.joints_deg[j] = joints[j]
        try:
            home_joints = driver.read_home_joints()
            prop_name = _HOME_PROP_BY_TYPE.get(getattr(slot, "robot_type", ""))
            if prop_name and hasattr(slot, prop_name):
                home_prop = getattr(slot, prop_name)
                for j in range(min(6, len(home_joints))):
                    home_prop[j] = float(home_joints[j])
        except NotImplementedError:
            pass
        except Exception:
            pass
        if hasattr(driver, "read_base") and bool(driver.capabilities() & CAP_READ_BASE):
            try:
                base_pos, base_euler = driver.read_base()
                slot.base_pos_m = base_pos
                slot.base_euler_rad = base_euler
                slot.base_euler_deg = (rad2deg(base_euler[0]), rad2deg(base_euler[1]), rad2deg(base_euler[2]))
                slot.base_frame_valid = True
            except Exception:
                slot.base_frame_valid = False
        if hasattr(driver, "read_tool"):
            try:
                tool_pos, tool_euler = driver.read_tool()
                slot.tool_frame_pos_m = tool_pos
                slot.tool_frame_euler_deg = tuple(rad2deg(v) for v in tool_euler)
                slot.tool_frame_valid = True
            except Exception:
                slot.tool_frame_valid = False
        apply_full_pose(slot, update_view_layer=False)
        # Safety: initialize the user target to current TCP on connect.
        if getattr(slot, "target_object", None) is not None and getattr(slot, "tcp_object", None) is not None:
            slot.target_object.location = slot.tcp_object.location.copy()
            slot.target_object.rotation_euler = slot.tcp_object.rotation_euler.copy()
    except Exception:
        # Best-effort seed only; polling thread will continue updating live values.
        pass

    # Determine poll rate from scene if available
    rate_hz = 25.0
    scene = bpy.context.scene
    if scene and hasattr(scene, "animaquina"):
        rate_hz = max(1.0, min(50.0, float(scene.animaquina.poll_rate_hz)))

    thread = _SlotPollThread(driver, slot.uid, rate_hz)
    thread.force_aux_refresh()  # read base/tool immediately on first connect
    thread.start()
    _poll_threads[slot.uid] = thread

    if not _timer_registered:
        if not bpy.app.timers.is_registered(_poll_all):
            bpy.app.timers.register(_poll_all, first_interval=0.1, persistent=True)
        _timer_registered = True
    return ""


def disconnect_slot(slot) -> None:
    global _drivers, _poll_threads, _timer_registered

    # Mark disconnected immediately so the timer skips this slot
    slot.is_connected = False
    slot.freedrive_active = False

    # End any recording: with the poll thread gone nothing would feed it, and a
    # panel still showing "recording" would be a lie. The session itself is kept
    # so whatever was captured can still be written out.
    session = recorder.stop(slot.uid)
    if session is not None:
        try:
            slot.record_active = False
            slot.record_sample_count = session.count()
            slot.record_elapsed_s = session.elapsed_s()
        except Exception:
            pass
    try:
        slot.realtime_puppet_active = False
        slot.realtime_puppet_status = "Idle"
    except Exception:
        pass

    # Disconnect driver first so the thread's next read raises and exits quickly
    driver = _drivers.pop(slot.uid, None)
    if driver is not None:
        if slot.freedrive_active and (driver.capabilities() & CAP_MANUAL_MODE):
            try:
                driver.set_manual_mode(False)
            except Exception:
                pass
        try:
            driver.disconnect()
        except Exception:
            pass

    # Now stop and join the thread (it should exit fast since the socket is closed)
    thread = _poll_threads.pop(slot.uid, None)
    if thread is not None:
        thread.stop()
        thread.join(timeout=1.0)

    # Remove stale cache
    with _cache_lock:
        _thread_cache.pop(slot.uid, None)

    _reset_slot_twin(slot)

    if not _drivers and _timer_registered:
        try:
            if bpy.app.timers.is_registered(_poll_all):
                bpy.app.timers.unregister(_poll_all)
        except Exception:
            pass
        _timer_registered = False


# Timer callback — main thread only, no I/O

# Dirty-check cache for base: skip apply_base_object when $BASE hasn't changed.
_last_applied_base = {}  # uid → (pos_m_tuple, euler_rad_tuple)


def _publish_record_command(slot) -> None:
    """Feed the active recording with the main-thread-only values - target pose
    and run state - and refresh the slot's live counters (writing those is also
    what makes the panel redraw)."""
    session = recorder.get(slot.uid)
    if session is None:
        return
    if session.active:
        run_idx = int(getattr(slot, "run_idx", -1))
        run_source = str(getattr(slot, "run_source", "NONE"))
        pos_m = euler_rad = None
        target = getattr(slot, "target_object", None)
        if target is not None and target.name in bpy.data.objects:
            try:
                pos_m, euler_rad = rig_apply.target_pose_in_robot_base_frame(slot, target)
            except Exception:
                pos_m = euler_rad = None
        session.set_context(pos_m, euler_rad, run_idx, run_source)
    # Only write on change: these run every tick and a finished session would
    # otherwise keep tagging UI redraws with values that never move again.
    count = session.count()
    if slot.record_active != session.active:
        slot.record_active = session.active
    if slot.record_sample_count != count:
        slot.record_sample_count = count
        slot.record_elapsed_s = session.elapsed_s()


def _poll_all() -> float:
    """Timer callback: read thread cache and apply rig for all connected+polling slots.
    No driver I/O happens here — all reads are done in background threads."""
    global _debug_poll_counter
    scene = bpy.context.scene
    if not scene or not hasattr(scene, "animaquina"):
        return 0.1
    props = scene.animaquina
    rate_hz = max(1.0, min(50.0, float(props.poll_rate_hz)))
    interval = 1.0 / rate_hz
    had_polling_slot = False
    did_update = False

    debug_started = time.perf_counter() if _DEBUG_POLL else 0.0

    # Snapshot the cache under lock so we don't hold it during Blender API calls
    with _cache_lock:
        cache_snapshot = {uid: dict(entry) for uid, entry in _thread_cache.items()}

    for slot in props.robots:
        if not slot.is_connected or not slot.polling_enabled:
            continue
        had_polling_slot = True

        thread = _poll_threads.get(slot.uid)
        if thread is not None:
            debug_names = []
            for item in getattr(slot, "debug_vars", []):
                if not bool(getattr(item, "enabled", True)):
                    continue
                var_name = str(getattr(item, "var_name", "") or "").strip()
                if var_name:
                    debug_names.append(var_name)
            try:
                thread.set_debug_var_names(debug_names)
            except Exception:
                pass

        data = cache_snapshot.get(slot.uid)
        if not data:
            continue

        # Surface thread errors to the slot
        if "error" in data:
            slot.last_error = data["error"]
            continue

        slot.last_error = ""

        if "tcp_pos_m" in data:
            pos_m = data["tcp_pos_m"]
            euler_rad = data["tcp_euler_rad"]
            slot.tcp_pos_m = pos_m
            slot.tcp_euler_rad = euler_rad
            slot.tcp_euler_deg = (rad2deg(euler_rad[0]), rad2deg(euler_rad[1]), rad2deg(euler_rad[2]))

        if "joints_deg" in data:
            joints = data["joints_deg"]
            for j in range(min(6, len(joints))):
                slot.joints_deg[j] = joints[j]

        if "base_pos_m" in data:
            base_pos = data["base_pos_m"]
            base_euler = data["base_euler_rad"]
            slot.base_pos_m = base_pos
            slot.base_euler_rad = base_euler
            slot.base_euler_deg = (rad2deg(base_euler[0]), rad2deg(base_euler[1]), rad2deg(base_euler[2]))
            slot.base_frame_valid = data.get("base_frame_valid", False)

        if "tool_frame_pos_m" in data:
            slot.tool_frame_pos_m = data["tool_frame_pos_m"]
            slot.tool_frame_euler_deg = tuple(
                rad2deg(v) for v in data["tool_frame_euler_rad"]
            )
            slot.tool_frame_valid = data.get("tool_frame_valid", False)
        else:
            slot.tool_frame_valid = data.get("tool_frame_valid", False)

        if "base_frame_valid" not in data:
            slot.base_frame_valid = False

        debug_values = data.get("debug_var_values", {})
        debug_errors = data.get("debug_var_errors", {})
        for item in getattr(slot, "debug_vars", []):
            var_name = str(getattr(item, "var_name", "") or "").strip()
            if not var_name:
                item.value = ""
                item.error = ""
                continue
            item.value = str(debug_values.get(var_name, ""))
            item.error = str(debug_errors.get(var_name, ""))

        # Mirror a polled IDX / IDX_RT into the canonical run_idx contract
        # (no-op while sim playback or streaming own the run state).
        run_state.mirror_program_idx(slot, debug_values)

        # Dataset recording: publish the commanded target pose for the poll
        # thread to pair with its next measured sample. Blender data can only
        # be read here on the main thread, so the command a sample carries is
        # at most one timer interval old.
        _publish_record_command(slot)

        apply_full_pose(slot, update_view_layer=False)
        did_update = True

    if not had_polling_slot:
        return 0.2  # No one polling: wake occasionally to react when Realtime is re-enabled

    # Only trigger view_layer.update() when context has a window (avoids depsgraph_update_post
    # handlers from other addons running with None context when timer fires in background).
    if did_update:
        vl = getattr(bpy.context, "view_layer", None)
        if vl and getattr(bpy.context, "window", None) and getattr(bpy.context, "screen", None):
            try:
                vl.update()
            except Exception:
                pass

    if _DEBUG_POLL:
        _debug_poll_counter += 1
        if _debug_poll_counter % _DEBUG_LOG_EVERY == 0:
            elapsed_ms = (time.perf_counter() - debug_started) * 1000.0
            print(f"[animaquina] poll={_debug_poll_counter} timer_callback={elapsed_ms:.2f}ms (no I/O)")

    return interval
