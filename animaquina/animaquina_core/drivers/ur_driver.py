# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Animaquina Core - Universal Robots driver (ur_rtde with URX fallback)

import os
import importlib
import math
import posixpath
import queue
import socket
import tempfile
import threading
import time as _time

# Import paths are set up centrally by animaquina.__init__._bootstrap_import_paths()

from .base import (
    DriverBase,
    CAP_CONNECT, CAP_READ_TCP, CAP_READ_JOINTS,
    CAP_MANUAL_MODE, CAP_MOVE_TO_TARGET, CAP_EXECUTE_PATH, CAP_HOME,
)
from ..runtime.conversions import rv2rpy, rpy2rv, rad2deg, deg2rad

# Try ur_rtde first (preferred), fall back to bundled urx
try:
    import rtde_control
    import rtde_receive
    try:
        import rtde_io
    except ImportError:
        rtde_io = None
    _HAS_RTDE = True
except ImportError:
    rtde_control = None
    rtde_receive = None
    rtde_io = None
    _HAS_RTDE = False

try:
    import urx
except ImportError:
    urx = None

try:
    import paramiko
    _HAS_PARAMIKO = True
except ImportError:
    paramiko = None
    _HAS_PARAMIKO = False


def _refresh_rtde_imports() -> bool:
    """Refresh RTDE modules in case they were installed after addon import."""
    global rtde_control, rtde_receive, rtde_io, _HAS_RTDE
    importlib.invalidate_caches()
    try:
        rtde_control = importlib.import_module("rtde_control")
        rtde_receive = importlib.import_module("rtde_receive")
        try:
            rtde_io = importlib.import_module("rtde_io")
        except ImportError:
            rtde_io = None
        _HAS_RTDE = True
    except ImportError:
        rtde_control = None
        rtde_receive = None
        rtde_io = None
        _HAS_RTDE = False
    return _HAS_RTDE


def ur_rtde_available() -> bool:
    """Check if ur_rtde is installed and importable."""
    return _refresh_rtde_imports()


def urx_available() -> bool:
    """Check if bundled URX is available."""
    return urx is not None


def _refresh_paramiko_import() -> bool:
    """Refresh paramiko import in case it was installed after addon import."""
    global paramiko, _HAS_PARAMIKO
    importlib.invalidate_caches()
    try:
        paramiko = importlib.import_module("paramiko")
        _HAS_PARAMIKO = True
    except ImportError:
        paramiko = None
        _HAS_PARAMIKO = False
    return _HAS_PARAMIKO


def ur_sftp_available() -> bool:
    """Check if paramiko is installed and importable."""
    return _refresh_paramiko_import()


# â”€â”€ Backend: ur_rtde â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

class _RtdeBackend:
    """ur_rtde implementation of UR communication."""

    def __init__(self):
        self._control = None
        self._receive = None
        self._io = None
        self._endpoint = ""
        self._debug = False  # verbose animaquina logging
        self._input_reg_cache = {}  # {name: last_written_value} for input register readback

    def _ensure_rtde_control(self) -> str:
        """Ensure RTDEControlInterface is available; returns "" or an error string.

        Some Blender / ur_rtde / Windows combinations crash (native segfault) when
        RTDEControlInterface is created during Connect. Receive + IO are opened first;
        control is created only when motion, script send, or register write needs it.

        Works identically for PolyScope 5 / CB3 and PolyScope X (10.10+): ur_rtde
        uploads the control script over the secondary interface. The robot must
        be in Remote Control mode, otherwise the uploaded script never runs and
        construction fails after a ~60 s timeout.
        """
        if self._control is not None:
            return ""
        if self._receive is None:
            return "RTDE receive interface not connected"
        ep = (self._endpoint or "").strip()
        if not ep:
            return "RTDE endpoint not set"

        # NOTE: never pass FLAG_VERBOSE to the *control* interface. ur_rtde's
        # verbose script-injection log has an unsigned-underflow bug
        # (substr(n - 100) with n < 100) that crashes with a native access
        # violation during the script upload. Fixed in our vendored build,
        # but keep this defensive for stock wheels.
        try:
            if self._debug:
                print(f"[Animaquina][UR debug] creating RTDEControlInterface(ep={ep!r})")
            self._control = rtde_control.RTDEControlInterface(ep)
            if self._debug:
                print(f"[Animaquina][UR debug] RTDEControlInterface created: {self._control!r}")
            return ""
        except Exception as e:
            msg = str(e)
            if "not running" in msg.lower() or "timeout" in msg.lower():
                msg += (" — make sure the robot is in Remote Control mode "
                        "(PolyScope X: Settings > System > Remote Control, then "
                        "switch to Remote; PolyScope 5: Remote Control in Settings), "
                        "then retry.")
            return msg

    def connect(self, endpoint: str, *, debug: bool = False) -> str:
        try:
            self._endpoint = (endpoint or "").strip()
            self._debug = bool(debug)
            if self._debug:
                print(f"[Animaquina][UR debug] connect: endpoint={self._endpoint!r}")
            self._receive = rtde_receive.RTDEReceiveInterface(endpoint)
            # RTDEControlInterface deferred — see _ensure_rtde_control()
            self._control = None
            if rtde_io is not None:
                try:
                    self._io = rtde_io.RTDEIOInterface(endpoint)
                except Exception:
                    self._io = None
            self._receive.getActualQ()  # test read
            io_status = "available" if self._io is not None else "NOT available"
            print(f"[Animaquina] RTDE connected (receive + IO); control interface is lazy: IO={io_status}")
            return ""
        except Exception as e:
            self.disconnect()
            return str(e)

    def disconnect(self):
        # RTDEControlInterface uploads a control script that runs a loop on the
        # robot for as long as the session lives. ur_rtde requires stopScript()
        # before disconnecting: without it the script keeps running after we are
        # gone, holding the robot's side of the RTDE link open and continuing to
        # push traffic at this machine (up to 500 Hz on e-Series) toward a socket
        # that no longer exists. That shows up as system-wide network/interrupt
        # load which survives closing Blender and is invisible in Task Manager's
        # per-process view — so always stop the script first, best effort.
        ctrl = self._control
        if ctrl is not None:
            for name in ("stopScript", "stop_script"):
                fn = getattr(ctrl, name, None)
                if callable(fn):
                    try:
                        fn()
                    except Exception:
                        pass
                    break
        for obj in (self._control, self._receive, self._io):
            if obj is not None:
                try:
                    obj.disconnect()
                except Exception:
                    pass
        self._control = None
        self._receive = None
        self._io = None
        self._endpoint = ""

    @property
    def connected(self):
        return self._receive is not None

    def urcap_debug_status(self) -> str:
        """Human-readable status dump for diagnosing the UR / RTDE link.

        ur_rtde uploads the control script over the secondary interface on
        both PolyScope 5 and PolyScope X (10.10+); the robot must be in
        Remote Control mode for the script to run.
        """
        def _safe(obj, attr):
            fn = getattr(obj, attr, None)
            if not callable(fn):
                return "n/a"
            try:
                return fn()
            except Exception as e:
                return f"<error: {e}>"

        lines = ["── UR / RTDE debug status ──────────────────────────"]
        lines.append(f"  endpoint (robot IP) : {self._endpoint or '<none>'}")
        lines.append(f"  debug logging       : {'ON' if self._debug else 'off'}")

        # animaquina ← robot: receive interface
        recv = self._receive
        lines.append("")
        lines.append("  [animaquina ← robot]  RTDE receive interface")
        if recv is None:
            lines.append("    receive interface : NOT created")
        else:
            lines.append(f"    isConnected()     : {_safe(recv, 'isConnected')}")
            lines.append(f"    robot mode        : {_safe(recv, 'getRobotMode')}")
            lines.append(f"    safety mode       : {_safe(recv, 'getSafetyMode')}")
            lines.append(f"    runtime state     : {_safe(recv, 'getRuntimeState')}")

        # animaquina → robot: control interface (script upload)
        ctrl = self._control
        lines.append("")
        lines.append("  [animaquina → robot]  RTDE control interface (script upload)")
        if ctrl is None:
            lines.append("    control interface : not created yet — it is built lazily on")
            lines.append("                        the first motion command / script send.")
        else:
            lines.append(f"    isConnected()     : {_safe(ctrl, 'isConnected')}")
            lines.append(f"    isProgramRunning(): {_safe(ctrl, 'isProgramRunning')}")
            lines.append(f"    isSteady()        : {_safe(ctrl, 'isSteady')}")

        # Verdict
        lines.append("")
        prog_up = False
        if ctrl is not None:
            fn = getattr(ctrl, "isProgramRunning", None)
            if callable(fn):
                try:
                    prog_up = bool(fn())
                except Exception:
                    prog_up = False
        if ctrl is not None and prog_up:
            lines.append("  ==> Control script is running — robot accepts motion commands.")
        elif ctrl is not None:
            lines.append("  ==> Control interface exists but the control script is NOT")
            lines.append("      running. Make sure the robot is in Remote Control mode,")
            lines.append("      then send a motion command to re-upload the script.")
        else:
            lines.append("  ==> Control interface not built yet. Checklist: robot powered")
            lines.append("      on (Normal) and Remote Control mode active. Then send any")
            lines.append("      motion command to establish control.")
        lines.append("────────────────────────────────────────────────────")
        return "\n".join(str(x) for x in lines)

    def read_tcp(self):
        pose = self._receive.getActualTCPPose()
        return pose

    def read_joints(self):
        return self._receive.getActualQ()

    # ── Variable reading via RTDE registers / receiver methods ────
    # Supported variable names:
    #   output_double_register_N   (0-47, written by URScript write_output_float_register)
    #   output_int_register_N      (0-47, written by URScript write_output_integer_register)
    #   input_double_register_N    (0-47, set by external RTDE client)
    #   input_int_register_N       (0-47, set by external RTDE client)
    #   standard_digital_input_N   (0-7)
    #   standard_digital_output_N  (0-7)
    #   analog_input_N             (0-1)
    #   analog_output_N            (0-1)
    #   robot_mode, safety_mode, runtime_state
    #   actual_tcp_speed, actual_tcp_force
    #   target_q_d (target joint velocities as list)

    _RTDE_OUTPUT_REGISTER_READ_MAP = {
        "output_double_register_": "getOutputDoubleRegister",
        "output_int_register_": "getOutputIntRegister",
    }

    _RTDE_INPUT_REGISTER_READ_MAP = {
        "input_double_register_": "getInputDoubleRegister",
        "input_int_register_": "getInputIntRegister",
    }

    _RTDE_DIGITAL_MAP = {
        "standard_digital_input_": "getDigitalInState",
        "standard_digital_output_": "getDigitalOutState",
    }

    # Configurable / tool digital I/O — read via output bit masks,
    # write via RTDEIOInterface methods.
    # Bit layout in getActualDigitalOutputBits():
    #   bits 0-7: standard digital outputs
    #   bits 8-15: configurable digital outputs
    #   bits 16-17: tool digital outputs
    _DIGITAL_OUTPUT_BIT_MAP = {
        "configurable_digital_output_": (8, 8, "setConfigurableDigitalOut"),   # offset=8, count=8
        "tool_digital_output_": (16, 2, "setToolDigitalOut"),                  # offset=16, count=2
    }
    _DIGITAL_INPUT_BIT_MAP = {
        "configurable_digital_input_": (8, 8),   # offset=8, count=8
        "tool_digital_input_": (16, 2),           # offset=16, count=2
    }

    _RTDE_NAMED_MAP = {
        "robot_mode": "getRobotMode",
        "safety_mode": "getSafetyMode",
        "runtime_state": "getRuntimeState",
        "actual_tcp_speed": "getActualTCPSpeed",
        "actual_tcp_force": "getActualTCPForce",
        "target_q_d": "getTargetQd",
    }

    def read_var(self, name: str) -> str:
        recv = self._receive
        if recv is None:
            raise RuntimeError("RTDE receive interface not connected")

        # Output register reads (output_double_register_0, etc.)
        for prefix, method_name in self._RTDE_OUTPUT_REGISTER_READ_MAP.items():
            if name.startswith(prefix):
                idx = int(name[len(prefix):])
                fn = getattr(recv, method_name, None)
                if fn is None:
                    raise RuntimeError(f"RTDE method {method_name} not available")
                return str(fn(idx))

        # Input register reads — these are external→robot, so the receive
        # interface may not support reading them back.  Try recv first,
        # fall back to the last value we wrote (cached on write_var).
        for prefix, method_name in self._RTDE_INPUT_REGISTER_READ_MAP.items():
            if name.startswith(prefix):
                idx = int(name[len(prefix):])
                fn = getattr(recv, method_name, None)
                if fn is not None:
                    try:
                        return str(fn(idx))
                    except Exception:
                        pass
                # Fallback: return cached value from our last write
                cached = self._input_reg_cache.get(name)
                if cached is not None:
                    return str(cached)
                return "0"

        # Digital I/O reads (standard)
        for prefix, method_name in self._RTDE_DIGITAL_MAP.items():
            if name.startswith(prefix):
                pin = int(name[len(prefix):])
                fn = getattr(recv, method_name, None)
                if fn is None:
                    raise RuntimeError(f"RTDE method {method_name} not available")
                return str(fn(pin))

        # Configurable / tool digital output reads (via bit mask)
        for prefix, (bit_offset, count, _) in self._DIGITAL_OUTPUT_BIT_MAP.items():
            if name.startswith(prefix):
                pin = int(name[len(prefix):])
                if pin >= count:
                    raise ValueError(f"{prefix} pin {pin} out of range (0-{count - 1})")
                fn = getattr(recv, "getActualDigitalOutputBits", None)
                if fn is None:
                    raise RuntimeError("RTDE method getActualDigitalOutputBits not available")
                bits = fn()
                return str(bool(bits & (1 << (bit_offset + pin))))

        # Configurable / tool digital input reads (via bit mask)
        for prefix, (bit_offset, count) in self._DIGITAL_INPUT_BIT_MAP.items():
            if name.startswith(prefix):
                pin = int(name[len(prefix):])
                if pin >= count:
                    raise ValueError(f"{prefix} pin {pin} out of range (0-{count - 1})")
                fn = getattr(recv, "getActualDigitalInputBits", None)
                if fn is None:
                    raise RuntimeError("RTDE method getActualDigitalInputBits not available")
                bits = fn()
                return str(bool(bits & (1 << (bit_offset + pin))))

        # Analog reads — try multiple method name patterns across ur_rtde versions:
        #   v1.5+: getStandardAnalogOutput0() (per-pin, no arg)
        #   some builds: getStandardAnalogOutput(pin) (pin arg)
        for sa_prefix, short_prefix, method_base in (
            ("standard_analog_input_", "analog_input_", "getStandardAnalogInput"),
            ("standard_analog_output_", "analog_output_", "getStandardAnalogOutput"),
        ):
            matched = None
            if name.startswith(sa_prefix):
                matched = sa_prefix
            elif name.startswith(short_prefix):
                matched = short_prefix
            if matched is not None:
                pin = int(name[len(matched):])
                # Try per-pin method first: getStandardAnalogOutput0()
                fn = getattr(recv, f"{method_base}{pin}", None)
                if fn is not None:
                    return str(fn())
                # Try parameterized method: getStandardAnalogOutput(pin)
                fn = getattr(recv, method_base, None)
                if fn is not None:
                    return str(fn(pin))
                raise RuntimeError(
                    f"No analog read method found (tried {method_base}{pin} and {method_base})"
                )

        # Named state reads
        if name in self._RTDE_NAMED_MAP:
            method_name = self._RTDE_NAMED_MAP[name]
            fn = getattr(recv, method_name, None)
            if fn is None:
                raise RuntimeError(f"RTDE method {method_name} not available")
            result = fn()
            if isinstance(result, (list, tuple)):
                return ", ".join(f"{v:.4f}" for v in result)
            return str(result)

        raise ValueError(
            f"Unknown UR variable '{name}'. Use: output_double_register_N, "
            f"output_int_register_N, standard_digital_input_N, analog_input_N, "
            f"robot_mode, safety_mode, actual_tcp_speed, etc."
        )

    # ── Variable writing via RTDE IO or inline URScript ───────────
    # Input registers (Blender → robot): written via RTDEIOInterface
    # Output registers (robot → Blender): written via inline URScript
    # Digital/analog outputs: written via RTDEIOInterface

    _RTDE_INPUT_REGISTER_MAP = {
        "input_int_register_": ("setInputIntRegister", int),
        "input_double_register_": ("setInputDoubleRegister", float),
    }

    _RTDE_OUTPUT_REGISTER_MAP = {
        "output_int_register_": ("write_output_integer_register", int),
        "output_double_register_": ("write_output_float_register", float),
    }

    def write_var(self, name: str, value) -> str:
        """Write a variable when NOT streaming (direct call, any interface)."""
        io = self._io
        ctrl = self._control

        # Input registers — IO has setInputIntRegister/setInputDoubleRegister
        for prefix, (method_name, cast) in self._RTDE_INPUT_REGISTER_MAP.items():
            if name.startswith(prefix):
                idx = int(name[len(prefix):])
                casted = cast(value)
                for obj in (io, ctrl):
                    fn = getattr(obj, method_name, None) if obj is not None else None
                    if fn is not None:
                        try:
                            fn(idx, casted)
                            self._input_reg_cache[name] = casted
                            return ""
                        except Exception as e:
                            continue  # try next interface
                err = self._ensure_rtde_control()
                if err:
                    return err
                ctrl = self._control
                fn = getattr(ctrl, method_name, None) if ctrl is not None else None
                if fn is not None:
                    try:
                        fn(idx, casted)
                        self._input_reg_cache[name] = casted
                        return ""
                    except Exception as e:
                        return str(e)
                return f"Neither IO nor ctrl has {method_name} (io={'yes' if io else 'None'})"

        # Output registers — inline URScript
        for prefix, (ur_func, cast) in self._RTDE_OUTPUT_REGISTER_MAP.items():
            if name.startswith(prefix):
                err = self._ensure_rtde_control()
                if err:
                    return err
                ctrl = self._control
                idx = int(name[len(prefix):])
                script = f"{ur_func}({idx}, {cast(value)})"
                fn = getattr(ctrl, "sendCustomScript", None) or getattr(ctrl, "sendCustomScriptFunction", None)
                if fn is None:
                    return "No custom script send method on RTDE control"
                try:
                    fn(script)
                    return ""
                except Exception as e:
                    return str(e)

        # Digital outputs — via IO
        if name.startswith("standard_digital_output_"):
            if io is None:
                return "RTDE IO interface not available for digital output"
            try:
                pin = int(name[len("standard_digital_output_"):])
                io.setStandardDigitalOut(pin, bool(value))
                return ""
            except Exception as e:
                return str(e)

        # Configurable digital outputs — via IO
        if name.startswith("configurable_digital_output_"):
            if io is None:
                return "RTDE IO interface not available for configurable digital output"
            try:
                pin = int(name[len("configurable_digital_output_"):])
                io.setConfigurableDigitalOut(pin, bool(value))
                return ""
            except Exception as e:
                return str(e)

        # Tool digital outputs — via IO
        if name.startswith("tool_digital_output_"):
            if io is None:
                return "RTDE IO interface not available for tool digital output"
            try:
                pin = int(name[len("tool_digital_output_"):])
                io.setToolDigitalOut(pin, bool(value))
                return ""
            except Exception as e:
                return str(e)

        # Analog outputs — via IO voltage method
        # setAnalogOutputVoltage expects volts (0-10V), pass value directly
        if name.startswith("standard_analog_output_"):
            if io is None:
                return "RTDE IO interface not available for analog output"
            try:
                pin = int(name[len("standard_analog_output_"):])
                voltage = float(value)
                io.setAnalogOutputVoltage(pin, voltage)
                return ""
            except Exception as e:
                return str(e)

        return f"Cannot write UR variable '{name}' — use digital/analog outputs"

    @staticmethod
    def dispatch_streaming_write(ctrl, io, name: str, value) -> str:
        """Write a variable from the servo worker thread during streaming.

        Called on the servo thread — safe to use ctrl directly (no cross-thread).
        Uses ctrl for input registers, io for digital/analog outputs.
        Output registers are rejected (they need sendCustomScript which kills servoL).
        """
        # Input registers — only RTDEIOInterface has setInputIntRegister/setInputDoubleRegister
        for prefix, (method_name, cast) in _RtdeBackend._RTDE_INPUT_REGISTER_MAP.items():
            if name.startswith(prefix):
                idx = int(name[len(prefix):])
                casted = cast(value)
                # Try direct call on io first
                if io is not None:
                    fn = getattr(io, method_name, None)
                    if fn is not None:
                        try:
                            fn(idx, casted)
                            return ""
                        except Exception as e:
                            return f"IO.{method_name}({idx}, {casted}) failed: {e}"
                    else:
                        return (
                            f"IO object type={type(io).__name__} has no '{method_name}'. "
                            f"Available: {[m for m in dir(io) if 'nput' in m.lower() or 'egist' in m.lower()]}"
                        )
                if ctrl is not None:
                    fn = getattr(ctrl, method_name, None)
                    if fn is not None:
                        try:
                            fn(idx, casted)
                            return ""
                        except Exception as e:
                            return f"ctrl.{method_name}({idx}, {casted}) failed: {e}"
                return f"No IO or ctrl available for {method_name}"

        # Output registers — NOT possible during streaming
        for prefix in _RtdeBackend._RTDE_OUTPUT_REGISTER_MAP:
            if name.startswith(prefix):
                return f"Cannot write '{name}' during streaming — use input registers instead"

        # Digital outputs — via IO
        if name.startswith("standard_digital_output_"):
            if io is None:
                return "RTDE IO not available for digital output"
            try:
                pin = int(name[len("standard_digital_output_"):])
                io.setStandardDigitalOut(pin, bool(value))
                return ""
            except Exception as e:
                return str(e)

        # Configurable digital outputs — via IO
        if name.startswith("configurable_digital_output_"):
            if io is None:
                return "RTDE IO not available for configurable digital output"
            try:
                pin = int(name[len("configurable_digital_output_"):])
                io.setConfigurableDigitalOut(pin, bool(value))
                return ""
            except Exception as e:
                return str(e)

        # Tool digital outputs — via IO
        if name.startswith("tool_digital_output_"):
            if io is None:
                return "RTDE IO not available for tool digital output"
            try:
                pin = int(name[len("tool_digital_output_"):])
                io.setToolDigitalOut(pin, bool(value))
                return ""
            except Exception as e:
                return str(e)

        # Analog outputs — via IO voltage method
        if name.startswith("standard_analog_output_"):
            if io is None:
                return "RTDE IO not available for analog output"
            try:
                pin = int(name[len("standard_analog_output_"):])
                voltage = float(value)
                io.setAnalogOutputVoltage(pin, voltage)
                return ""
            except Exception as e:
                return str(e)

        return f"Unknown variable '{name}' for streaming write"

    def _ensure_control_program(self):
        err = self._ensure_rtde_control()
        if err:
            raise RuntimeError(err)
        ctrl = self._control

        is_running_fn = getattr(ctrl, "isProgramRunning", None)
        running = None
        if callable(is_running_fn):
            try:
                running = bool(is_running_fn())
            except Exception:
                running = None

        if running is False:
            reupload_fn = getattr(ctrl, "reuploadScript", None)
            reupload_ok = False
            if callable(reupload_fn):
                ok = reupload_fn()
                reupload_ok = not (isinstance(ok, bool) and not ok)
            if not reupload_ok:
                # Most common cause: the robot left Remote Control mode (a
                # local UI interaction switches it back to Manual/Local) —
                # the controller then refuses the control script.
                raise RuntimeError(
                    "UR control script is not running and could not be "
                    "restarted. Check that the robot is in Remote Control "
                    "mode, then Disconnect/Connect in Animaquina. If it "
                    "keeps failing, restart Blender to clear the stale "
                    "RTDE session."
                )

    def _run_motion(self, fn):
        self._ensure_control_program()
        try:
            fn()
            return
        except Exception as e:
            msg = str(e).lower()
            if "script is not running" not in msg and "rtde control script" not in msg:
                raise
            reupload_fn = getattr(self._control, "reuploadScript", None)
            if not callable(reupload_fn):
                raise
            ok = reupload_fn()
            if isinstance(ok, bool) and not ok:
                raise
            fn()

    def movel(self, target, speed, acc, wait):
        self._run_motion(lambda: self._control.moveL(target, speed, acc, asynchronous=not wait))

    def movej(self, joints_rad, vel, acc, wait):
        self._run_motion(lambda: self._control.moveJ(joints_rad, vel, acc, asynchronous=not wait))

    def movej_ik(self, target, vel, acc, wait):
        self._run_motion(lambda: self._control.moveJ_IK(target, vel, acc, asynchronous=not wait))

    def move_path(self, path_9elem, wait: bool = False):
        """Execute multi-waypoint path. Each entry: [x,y,z,rx,ry,rz, vel, acc, blend].
        wait=False: return immediately (robot executes in background), matches old URX behavior."""
        self._run_motion(lambda: self._control.moveL(path_9elem, asynchronous=not wait))

    def get_async_progress(self) -> int:
        """Return async operation progress. <0 = idle, >=0 = in progress."""
        if self._ensure_rtde_control():
            return -1
        ctrl = self._control
        if ctrl is None:
            return -1
        fn = getattr(ctrl, "getAsyncOperationProgress", None)
        if callable(fn):
            try:
                return int(fn())
            except Exception:
                return -1
        return -1

    def stop_l(self, decel: float = 10.0):
        """Decelerate to stop in Cartesian space."""
        if self._ensure_rtde_control():
            return
        ctrl = self._control
        if ctrl is not None:
            fn = getattr(ctrl, "stopL", None)
            if callable(fn):
                try:
                    fn(decel)
                except Exception:
                    pass

    def stop_j(self, decel: float = 10.0):
        """Decelerate to stop in joint space."""
        if self._ensure_rtde_control():
            return
        ctrl = self._control
        if ctrl is not None:
            fn = getattr(ctrl, "stopJ", None)
            if callable(fn):
                try:
                    fn(decel)
                except Exception:
                    pass

    def freedrive(self, enabled):
        err = self._ensure_rtde_control()
        if err:
            raise RuntimeError(err)
        ctrl = self._control
        if enabled:
            ctrl.freedriveMode()
        else:
            ctrl.endFreedriveMode()

    def send_program(self, content):
        err = self._ensure_rtde_control()
        if err:
            raise RuntimeError(err)
        ctrl = self._control

        # ur_rtde script-send API differs by version; prefer direct custom script send.
        fn = getattr(ctrl, "sendCustomScript", None)
        if callable(fn):
            fn(content)
            return

        # Fallback for older builds exposing only function-based custom script API.
        fn = getattr(ctrl, "sendCustomScriptFunction", None)
        if callable(fn):
            try:
                fn("animaquina_program", content)
                return
            except TypeError:
                fn(content)
                return

        raise RuntimeError("ur_rtde control has no supported custom-script send method")

    def send_program_file(self, content, program_name: str = "animaquina"):
        """Send script preferring file transport (better for large payloads)."""
        err = self._ensure_rtde_control()
        if err:
            raise RuntimeError(err)
        ctrl = self._control

        file_send = getattr(ctrl, "sendCustomScriptFile", None)
        if callable(file_send):
            safe = "".join(c if c.isalnum() or c in {"_", "-"} else "_" for c in (program_name or "animaquina"))
            script_path = ""
            try:
                with tempfile.NamedTemporaryFile(
                    mode="w",
                    encoding="utf-8",
                    suffix=".script",
                    prefix=f"{safe}_",
                    newline="\n",
                    delete=False,
                ) as tmp:
                    tmp.write(content)
                    script_path = tmp.name
                try:
                    file_send(script_path)
                    return "rtde_custom_file"
                except TypeError:
                    # Some builds expose a different signature; fall back to inline.
                    pass
            finally:
                if script_path:
                    try:
                        os.unlink(script_path)
                    except Exception:
                        pass

        self.send_program(content)
        return "rtde_custom_inline"


# â”€â”€ Backend: URX (legacy fallback) â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

class _UrxBackend:
    """URX implementation of UR communication."""

    def __init__(self):
        self._robot = None

    def connect(self, endpoint: str) -> str:
        try:
            self._robot = urx.Robot(endpoint)
            self._robot.getj()  # test read
            return ""
        except Exception as e:
            self._robot = None
            return str(e)

    def disconnect(self):
        if self._robot is not None:
            try:
                self._robot.close()
            except Exception:
                pass
            self._robot = None

    @property
    def connected(self):
        return self._robot is not None

    def read_tcp(self):
        return self._robot.getl()

    def read_joints(self):
        return self._robot.getj()

    # URX variable reading — limited compared to RTDE but covers secmon data
    _URX_SECMON_MAP = {
        "robot_mode": "RobotModeData",
        "actual_tcp_speed": "CartesianInfo",
    }

    def read_var(self, name: str) -> str:
        if self._robot is None:
            raise RuntimeError("URX not connected")
        # Try secmon.get_all_data() for known keys
        secmon = getattr(self._robot, "secmon", None)
        if secmon is not None:
            try:
                data = secmon.get_all_data()
            except Exception:
                data = {}
            if name in data:
                val = data[name]
                if isinstance(val, (list, tuple)):
                    return ", ".join(str(v) for v in val)
                return str(val)
        raise ValueError(
            f"URX variable '{name}' not available. URX has limited variable access. "
            f"Use ur_rtde backend for register reads (output_double_register_N, etc.)."
        )

    def movel(self, target, speed, acc, wait):
        self._robot.movel(target, acc=acc, vel=speed, wait=wait)

    def movej(self, joints_rad, vel, acc, wait):
        self._robot.movej(joints_rad, acc=acc, vel=vel, wait=wait)

    def movej_ik(self, target, vel, acc, wait):
        # URX has no moveJ_IK â€” send raw URScript with p[] prefix
        vals = [round(v, 6) for v in target]
        prog = "movej(p[{},{},{},{},{},{}], a={}, v={})".format(
            *vals, round(acc, 4), round(vel, 4)
        )
        self._robot.send_program(prog)

    def move_path(self, path_9elem, wait: bool = False):
        # URX uses movexs with separate radius â€” extract from 9-element vectors
        poses = []
        radius = 0.0
        for wp in path_9elem:
            poses.append(wp[:6])
            if len(wp) > 8:
                radius = max(radius, wp[8])
        acc = path_9elem[0][7] if path_9elem and len(path_9elem[0]) > 7 else 0.5
        vel = path_9elem[0][6] if path_9elem and len(path_9elem[0]) > 6 else 0.1
        self._robot.movexs("movel", poses, acc=acc, vel=vel, radius=radius, wait=wait)

    def stop_l(self, decel: float = 10.0):
        self._robot.stopl(decel)

    def stop_j(self, decel: float = 10.0):
        self._robot.stopj(decel)

    def freedrive(self, enabled):
        if enabled:
            self._robot.send_program("def myProg():\n\tfreedrive_mode()\n\tsleep(60)\nend")
        else:
            self._robot.send_program("def myProg():\n\tend_freedrive_mode()\nend")

    def send_program(self, content):
        self._robot.send_program(content)

    def send_program_file(self, content, program_name: str = "animaquina"):
        self._robot.send_program(content)
        return "urx_secondary_inline"


# â”€â”€ URDriver â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€â”€

class URDriver(DriverBase):
    """Universal Robots driver. Uses ur_rtde if available, falls back to URX."""

    def __init__(self):
        self._backend = None
        self._backend_name = ""
        self._endpoint = ""
        self._last_program_transport = ""
        self._last_staged_program_content = ""
        self._last_staged_program_name = ""
        self._last_staged_remote_file = ""
        self._last_staged_remote_urp_file = ""
        self._rt_puppet_active = False

    @property
    def backend_name(self):
        return self._backend_name

    @property
    def last_program_transport(self):
        return self._last_program_transport

    @property
    def last_staged_remote_file(self):
        return self._last_staged_remote_file

    @property
    def last_staged_remote_urp_file(self):
        return self._last_staged_remote_urp_file

    def capabilities(self):
        return (
            CAP_CONNECT | CAP_READ_TCP | CAP_READ_JOINTS
            | CAP_MANUAL_MODE | CAP_MOVE_TO_TARGET | CAP_EXECUTE_PATH | CAP_HOME
        )

    def urcap_debug_status(self) -> str:
        """Return a diagnostic status dump (PolyScope X / URCap connection)."""
        backend = self._backend
        if backend is None:
            return "UR not connected — no backend."
        fn = getattr(backend, "urcap_debug_status", None)
        if not callable(fn):
            return f"Debug status not available for backend '{self._backend_name}'."
        try:
            return fn()
        except Exception as e:
            return f"Debug status error: {e}"

    def connect(self, endpoint: str, port: int = 0, *, slot=None) -> str:
        # Disconnect previous
        self.disconnect()

        # User preference: ur_rtde or urx (from slot.ur_backend)
        preferred = "ur_rtde"
        debug = False
        if slot is not None:
            preferred = getattr(slot, "ur_backend", "ur_rtde") or "ur_rtde"
            debug = bool(getattr(slot, "ur_debug", False))
        has_rtde = ur_rtde_available()

        ur_rtde_failed = False
        # Try preferred backend first
        if preferred == "ur_rtde" and has_rtde:
            backend = _RtdeBackend()
            err = backend.connect(endpoint, debug=debug)
            if not err:
                self._backend = backend
                self._backend_name = "ur_rtde"
                self._endpoint = endpoint or ""
                return ""
            ur_rtde_failed = True
            if urx is None:
                return err
            print(f"[Animaquina] ur_rtde connect failed ({err}), trying URX fallback...")

        # Try URX when preferred or as fallback after ur_rtde failed
        if urx is not None and (preferred == "urx" or ur_rtde_failed):
            backend = _UrxBackend()
            err = backend.connect(endpoint)
            if not err:
                self._backend = backend
                self._backend_name = "urx"
                self._endpoint = endpoint or ""
                return ""
            return err

        # If preferred was ur_rtde but not installed, try URX
        if preferred == "ur_rtde" and not has_rtde and urx is not None:
            backend = _UrxBackend()
            err = backend.connect(endpoint)
            if not err:
                self._backend = backend
                self._backend_name = "urx"
                self._endpoint = endpoint or ""
                return ""
            return err

        if preferred == "ur_rtde" and not has_rtde:
            return ("ur_rtde not available — reinstall the Animaquina addon zip "
                    "(it ships bundled) or use 'Install UR Dependencies'")
        return "Neither ur_rtde nor urx is installed"

    def disconnect(self) -> None:
        try:
            if self._rt_puppet_active:
                self.realtime_puppet_stop()
        except Exception:
            pass
        try:
            self._servo_cleanup(reason="Disconnect")
        except Exception:
            pass
        if self._backend is not None:
            self._backend.disconnect()
            self._backend = None
            self._backend_name = ""
        self._endpoint = ""
        self._rt_puppet_active = False

    def _ensure(self):
        if self._backend is None or not self._backend.connected:
            raise RuntimeError("UR robot not connected")

    def _send_secondary_program(self, content: str, port: int = 30002, timeout: float = 2.0) -> str:
        """Send URScript directly to UR secondary interface (URX-style)."""
        endpoint = self._endpoint or ""
        if not endpoint:
            return "UR endpoint unavailable for secondary script send"
        payload = str(content or "")
        if not payload.endswith("\n"):
            payload += "\n"
        try:
            with socket.create_connection((endpoint, int(port)), timeout=timeout) as sock:
                sock.settimeout(timeout)
                sock.sendall(payload.encode("utf-8"))
            return ""
        except Exception as e:
            return f"Secondary interface send failed: {e}"

    def read_tcp(self) -> tuple:
        self._ensure()
        pose = self._backend.read_tcp()
        pos_m = (float(pose[0]), float(pose[1]), float(pose[2]))
        roll, pitch, yaw = rv2rpy(float(pose[3]), float(pose[4]), float(pose[5]))
        return (pos_m, (roll, pitch, yaw))

    def read_joints(self) -> tuple:
        self._ensure()
        j_rad = self._backend.read_joints()
        return tuple(rad2deg(float(j)) for j in j_rad[:6])

    def read_base(self) -> tuple:
        return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    def read_var(self, name: str) -> str:
        """Read a controller variable. Delegates to backend (RTDE registers or URX secmon)."""
        self._ensure()
        return self._backend.read_var(name)

    def write_var(self, name: str, value) -> str:
        """Write a controller variable.

        During active servoL streaming, writes are enqueued and executed by
        the servo worker thread. This prevents cross-thread access to the
        RTDE control interface (not thread-safe) and avoids sendCustomScript
        (which kills the servoL control script).
        """
        self._ensure()
        if not hasattr(self._backend, "write_var"):
            return f"{self._backend_name} backend does not support variable writing"

        # During streaming: enqueue for the servo worker thread
        servo_state = getattr(self, "_servo_state", None)
        if servo_state and not servo_state.get("stop"):
            # Reject output registers immediately (need sendCustomScript)
            for prefix in _RtdeBackend._RTDE_OUTPUT_REGISTER_MAP:
                if name.startswith(prefix):
                    return f"Cannot write '{name}' during streaming — use input registers instead"
            var_q = servo_state.get("var_queue")
            if var_q is not None:
                try:
                    var_q.put_nowait((name, value))
                    return ""
                except queue.Full:
                    return "Variable write queue full"
            return "No var_queue in servo state"

        # Not streaming: direct write
        return self._backend.write_var(name, value)


    def move_to_pose(self, pos_m, euler_rad, speed: float, acc: float, radius: float, wait: bool) -> str:
        self._ensure()
        try:
            rx, ry, rz = rpy2rv(float(euler_rad[0]), float(euler_rad[1]), float(euler_rad[2]))
            target = [float(pos_m[0]), float(pos_m[1]), float(pos_m[2]), rx, ry, rz]
            self._backend.movel(target, speed, acc, wait)
            return ""
        except Exception as e:
            return str(e)

    def move_to_pose_ptp(self, pos_m, euler_rad, vel: float, acc: float, radius: float, wait: bool) -> str:
        """Joint-space move to cartesian pose target."""
        self._ensure()
        try:
            rx, ry, rz = rpy2rv(float(euler_rad[0]), float(euler_rad[1]), float(euler_rad[2]))
            target = [float(pos_m[0]), float(pos_m[1]), float(pos_m[2]), rx, ry, rz]
            self._backend.movej_ik(target, vel, acc, wait)
            return ""
        except Exception as e:
            return str(e)

    def execute_ptp_path(self, waypoints: list, speed: float, acc: float, radius: float, wait: bool) -> str:
        self._ensure()
        try:
            path = []
            for wp in waypoints:
                pos_m = wp[0] if len(wp) > 0 else (0, 0, 0)
                euler_rad = wp[1] if len(wp) > 1 else (0, 0, 0)
                rx, ry, rz = rpy2rv(float(euler_rad[0]), float(euler_rad[1]), float(euler_rad[2]))
                path.append([
                    float(pos_m[0]), float(pos_m[1]), float(pos_m[2]), rx, ry, rz,
                    speed, acc, radius,
                ])
            if not path:
                return ""
            # Last waypoint must have blend=0
            path[-1][8] = 0.0
            self._backend.move_path(path, wait=wait)
            return ""
        except Exception as e:
            return str(e)

    def get_async_progress(self) -> int:
        """Return async operation progress. <0 = idle/done, >=0 = executing."""
        self._ensure()
        if hasattr(self._backend, "get_async_progress"):
            return self._backend.get_async_progress()
        return -1

    def stop_motion(self) -> str:
        """Emergency stop — decelerate robot in both joint and Cartesian space."""
        self._ensure()
        errors = []
        for method in ("stop_j", "stop_l"):
            if hasattr(self._backend, method):
                try:
                    getattr(self._backend, method)()
                except Exception as e:
                    errors.append(str(e))
        return "; ".join(errors) if errors else ""

    def _get_rtde_servo_fns(self):
        if not isinstance(self._backend, _RtdeBackend):
            return None, None, "UR Real Time Puppet Mode requires ur_rtde backend"
        err = self._backend._ensure_rtde_control()
        if err:
            return None, None, err
        ctrl = getattr(self._backend, "_control", None)
        if ctrl is None:
            return None, None, "RTDE control not connected"
        servo_fn = getattr(ctrl, "servoL", None)
        if not callable(servo_fn):
            servo_fn = getattr(ctrl, "servo_l", None)
        if not callable(servo_fn):
            return None, None, "servoL not available on this ur_rtde build"
        stop_fn = getattr(ctrl, "servoStop", None)
        if not callable(stop_fn):
            stop_fn = getattr(ctrl, "servo_stop", None)
        if not callable(stop_fn):
            return None, None, "servoStop not available on this ur_rtde build"
        return ctrl, servo_fn, ""

    def realtime_puppet_start(self, boundary_mm=None) -> str:
        """Start direct realtime target-follow mode (servoL) for UR via ur_rtde."""
        self._ensure()
        ctrl, _servo_fn, err = self._get_rtde_servo_fns()
        if err:
            return err

        # Cannot run queue streaming and puppet mode at the same time.
        self._servo_cleanup(reason="Starting Real Time Puppet Mode")

        ensure_fn = getattr(self._backend, "_ensure_control_program", None)
        if callable(ensure_fn):
            try:
                ensure_fn()
            except Exception as e:
                return str(e)

        # boundary_mm reserved for cross-robot API parity (xArm uses it).
        _ = boundary_mm
        self._rt_puppet_active = True
        return ""

    def realtime_puppet_step(self, pos_m, euler_rad, speed: float, acc: float) -> str:
        """Send one realtime cartesian setpoint using servoL."""
        self._ensure()
        if not self._rt_puppet_active:
            err = self.realtime_puppet_start()
            if err:
                return err

        ctrl, servo_fn, err = self._get_rtde_servo_fns()
        if err:
            return err

        def _servo_call(target, v, a):
            try:
                servo_fn(target, v, a, 0.008, 0.1, 300)
            except TypeError:
                try:
                    servo_fn(target, v, a, 0.008, 0.1)
                except TypeError:
                    servo_fn(target, v, a, 0.008)

        try:
            rx, ry, rz = rpy2rv(float(euler_rad[0]), float(euler_rad[1]), float(euler_rad[2]))
            target = [float(pos_m[0]), float(pos_m[1]), float(pos_m[2]), rx, ry, rz]
            if not all(math.isfinite(v) for v in target):
                return "UR realtime puppet step rejected: non-finite target values"
            v = max(0.001, float(speed))
            a = max(0.001, float(acc))
            _servo_call(target, v, a)
            return ""
        except Exception as e:
            msg = str(e)
            low = msg.lower()
            # Recover if RTDE control script dropped.
            if "script is not running" in low or "rtde control script" in low:
                reupload_fn = getattr(ctrl, "reuploadScript", None)
                if callable(reupload_fn):
                    try:
                        ok = reupload_fn()
                        if isinstance(ok, bool) and not ok:
                            return msg
                        _servo_call(target, v, a)
                        return ""
                    except Exception:
                        return msg
            return msg

    def realtime_puppet_stop(self) -> str:
        """Stop realtime puppet servo mode cleanly."""
        self._rt_puppet_active = False
        # Also stop queue worker if active, since both use servoL.
        self._servo_cleanup(reason="Real Time Puppet Mode stopped")

        if not isinstance(self._backend, _RtdeBackend):
            return ""
        ctrl = getattr(self._backend, "_control", None)
        if ctrl is None:
            return ""

        issues = []
        stop_fn = getattr(ctrl, "servoStop", None)
        if not callable(stop_fn):
            stop_fn = getattr(ctrl, "servo_stop", None)
        if callable(stop_fn):
            try:
                stop_fn(10.0)
            except TypeError:
                try:
                    stop_fn()
                except Exception as ex:
                    issues.append(str(ex))
            except Exception as ex:
                issues.append(str(ex))
        if hasattr(self._backend, "stop_l"):
            try:
                self._backend.stop_l()
            except Exception as ex:
                issues.append(str(ex))
        if issues:
            return f"UR realtime puppet stop warning: {issues[0]}"
        return ""

    # ── Dynamic Sync streaming (servoL worker thread — KUKA parity) ──

    _stream_cmd_id = 0
    _servo_state = None

    def _stream_next_cmd_id(self) -> int:
        URDriver._stream_cmd_id += 1
        return URDriver._stream_cmd_id

    @staticmethod
    def _interp_pose(start_pose, end_pose, alpha: float):
        """Linearly interpolate two 6D UR poses."""
        a = max(0.0, min(1.0, float(alpha)))
        return [
            float(start_pose[i]) + (float(end_pose[i]) - float(start_pose[i])) * a
            for i in range(6)
        ]

    @staticmethod
    def _segment_steps(start_pose, end_pose, speed: float, dt: float) -> int:
        """Estimate how many servo periods are needed for a segment.

        servoL expects a steady stream of targets. When we only have sparse path
        points, expand them into dense substeps so the robot does not settle at
        every source waypoint.
        """
        try:
            dx = float(end_pose[0]) - float(start_pose[0])
            dy = float(end_pose[1]) - float(start_pose[1])
            dz = float(end_pose[2]) - float(start_pose[2])
            linear_dist = math.sqrt(dx * dx + dy * dy + dz * dz)

            drx = float(end_pose[3]) - float(start_pose[3])
            dry = float(end_pose[4]) - float(start_pose[4])
            drz = float(end_pose[5]) - float(start_pose[5])
            angular_dist = math.sqrt(drx * drx + dry * dry + drz * drz)

            linear_speed = max(0.001, float(speed))
            angular_speed = max(0.25, linear_speed * 8.0)
            duration = max(linear_dist / linear_speed, angular_dist / angular_speed, float(dt))
            return max(1, int(math.ceil(duration / max(0.001, float(dt)))))
        except Exception:
            return 1

    def _servo_worker(self, state: dict):
        """Background thread: consume sparse waypoints and drive servoL at fixed cadence."""
        ctrl = getattr(self._backend, "_control", None)
        if ctrl is None:
            state["error"] = "RTDE control not connected"
            return
        servo_fn = getattr(ctrl, "servoL", None)
        if not callable(servo_fn):
            servo_fn = getattr(ctrl, "servo_l", None)
        if not callable(servo_fn):
            state["error"] = "servoL not available on this ur_rtde build"
            return
        # Use the controller's real cycle period, not the assumed default.
        # waitPeriod() paces this loop at the RTDE control frequency (500 Hz on
        # e-Series = 2 ms, 125 Hz on CB3 = 8 ms). The substep math below divides
        # each segment into duration/dt steps, so if dt disagrees with the real
        # cadence the TCP runs proportionally faster than the commanded speed
        # (4x on e-Series with the old fixed 8 ms assumption).
        step_time_fn = getattr(ctrl, "getStepTime", None)
        if callable(step_time_fn):
            try:
                real_dt = float(step_time_fn())
                if 0.001 <= real_dt <= 0.1:
                    state["dt"] = real_dt
            except Exception:
                pass
        dt = state["dt"]
        lookahead = state["lookahead"]
        gain = state["gain"]
        speed = max(0.001, float(state.get("speed", 0.1)))
        acc = max(0.001, float(state.get("acc", 0.5)))
        seg_speed = speed  # per-segment speed; overridden per waypoint when state["speeds"] is set
        last_cmd_target = None
        active_target = None
        active_start = None
        active_step = 0
        active_steps = 0

        init_period_fn = getattr(ctrl, "initPeriod", None)
        if not callable(init_period_fn):
            init_period_fn = None
        wait_period_fn = getattr(ctrl, "waitPeriod", None)
        if not callable(wait_period_fn):
            wait_period_fn = None

        receive = getattr(self._backend, "_receive", None)
        if receive is not None:
            actual_pose_fn = getattr(receive, "getActualTCPPose", None)
            if callable(actual_pose_fn):
                try:
                    pose = actual_pose_fn()
                    if pose and len(pose) >= 6:
                        last_cmd_target = [float(v) for v in pose[:6]]
                except Exception:
                    last_cmd_target = None

        while not state.get("stop"):
            cycle_token = None
            if init_period_fn is not None:
                try:
                    cycle_token = init_period_fn()
                except Exception:
                    cycle_token = None

            if active_target is None:
                try:
                    active_target = state["queue"].get(timeout=dt)
                    state["waypoints_popped"] = state.get("waypoints_popped", 0) + 1
                    active_start = list(last_cmd_target or active_target)
                    active_step = 0
                    # Per-point speed: the segment *into* waypoint i uses speeds[i]
                    seg_speed = speed
                    sp_list = state.get("speeds")
                    if sp_list:
                        i = state["waypoints_popped"] - 1
                        if 0 <= i < len(sp_list):
                            try:
                                v = float(sp_list[i])
                                if v > 0.0005:
                                    seg_speed = min(v, 2.0)
                            except (TypeError, ValueError):
                                pass
                    active_steps = self._segment_steps(active_start, active_target, seg_speed, dt)
                except queue.Empty:
                    active_target = None
                    active_start = None

            if active_target is not None:
                active_step += 1
                alpha = 1.0 if active_steps <= 1 else (float(active_step) / float(active_steps))
                target = self._interp_pose(active_start, active_target, alpha)
            else:
                target = list(last_cmd_target) if last_cmd_target is not None else None

            if target is None:
                if wait_period_fn is not None and cycle_token is not None:
                    try:
                        wait_period_fn(cycle_token)
                    except Exception:
                        _time.sleep(dt)
                else:
                    _time.sleep(dt)
                continue

            try:
                try:
                    servo_fn(target, seg_speed, acc, dt, lookahead, gain)
                except TypeError:
                    # Older/newer bindings may expose reduced positional signatures.
                    try:
                        servo_fn(target, seg_speed, acc, dt, lookahead)
                    except TypeError:
                        servo_fn(target, seg_speed, acc, dt)
                last_cmd_target = target
                state["servo_sent"] = state.get("servo_sent", 0) + 1
            except Exception as e:
                state["error"] = str(e)
                break

            if active_target is not None and active_step >= active_steps:
                last_cmd_target = list(active_target)
                active_target = None
                active_start = None
                active_step = 0
                active_steps = 0
                state["waypoints_completed"] = state.get("waypoints_completed", 0) + 1

            # Drain queued variable writes (enqueued by main thread)
            var_q = state.get("var_queue")
            if var_q is not None:
                io = getattr(self._backend, "_io", None)
                while not var_q.empty():
                    try:
                        vname, vval = var_q.get_nowait()
                    except queue.Empty:
                        break
                    try:
                        err = _RtdeBackend.dispatch_streaming_write(ctrl, io, vname, vval)
                        if err:
                            print(f"[Animaquina] streaming var write '{vname}'={vval}: {err}")
                    except Exception as exc:
                        print(f"[Animaquina] streaming var write '{vname}' exception: {exc}")

            if wait_period_fn is not None and cycle_token is not None:
                try:
                    wait_period_fn(cycle_token)
                except Exception:
                    _time.sleep(dt)
            else:
                _time.sleep(dt)

        # Clean stop
        stop_fn = getattr(ctrl, "servoStop", None)
        if not callable(stop_fn):
            stop_fn = getattr(ctrl, "servo_stop", None)
        if callable(stop_fn):
            try:
                stop_fn(10.0)
            except TypeError:
                try:
                    stop_fn()
                except Exception:
                    pass
            except Exception:
                pass

    def stream_start(self, total_points: int, speed: float, radius: float,
                     initial_waypoints: list, ring_buffer_size: int = 6,
                     acc_value: float = 0.5, speeds: list = None):
        """Start servoL-based path streaming (mirrors KUKADriver.stream_start).
        Uses a worker thread that calls servoL at ~125Hz while the modal refills
        a queue — exactly like the KUKA ring buffer pattern.

        speeds: optional per-point linear speeds (m/s), indexed like the full
        waypoint list ("Per-Point Speed" attribute). Points with a missing or
        non-positive entry fall back to the constant `speed`. The caller may
        replace state["speeds"] wholesale while streaming (live attribute
        updates); the worker only reads it.

        Returns a state dict on success, or an error string."""
        self._ensure()
        if not isinstance(self._backend, _RtdeBackend):
            return "servoL streaming requires ur_rtde backend"
        if self._rt_puppet_active:
            self.realtime_puppet_stop()
        err_rtde = self._backend._ensure_rtde_control()
        if err_rtde:
            return err_rtde
        ctrl = self._backend._control
        if ctrl is None:
            return "RTDE control not connected"
        has_servo = callable(getattr(ctrl, "servoL", None)) or callable(getattr(ctrl, "servo_l", None))
        if not has_servo:
            return "servoL not available — upgrade ur_rtde"
        has_servo_stop = callable(getattr(ctrl, "servoStop", None)) or callable(getattr(ctrl, "servo_stop", None))
        if not has_servo_stop:
            return "servoStop not available — upgrade ur_rtde"

        # Stop any previous servo session
        self._servo_cleanup()

        try:
            ring_buffer_size = int(max(2, min(6, int(ring_buffer_size or 6))))
            cmd_id = self._stream_next_cmd_id()

            state = {
                "cmd_id": cmd_id,
                "written": 0,
                "total": total_points,
                "ring_size": ring_buffer_size,
                "speed": max(0.001, float(speed)),
                "acc": max(0.001, float(acc_value)),
                "dt": 0.008,
                "lookahead": 0.1,
                "gain": 300,
                "speeds": list(speeds) if speeds else None,
                "queue": queue.Queue(maxsize=ring_buffer_size * 2),
                "var_queue": queue.Queue(maxsize=64),
                "stop": False,
                "error": "",
                "servo_sent": 0,
                "waypoints_popped": 0,
                "waypoints_completed": 0,
            }

            # Pre-fill queue with initial waypoints
            for wp in initial_waypoints[:ring_buffer_size]:
                pos_m, euler_rad = wp
                rx, ry, rz = rpy2rv(float(euler_rad[0]), float(euler_rad[1]), float(euler_rad[2]))
                target = [float(pos_m[0]), float(pos_m[1]), float(pos_m[2]), rx, ry, rz]
                state["queue"].put(target)
                state["written"] += 1

            # Start worker thread
            t = threading.Thread(target=self._servo_worker, args=(state,), daemon=True)
            state["thread"] = t
            self._servo_state = state
            t.start()

            return state
        except Exception as e:
            return str(e)

    def stream_refill(self, state: dict, rd_idx: int, waypoints: list):
        """Add waypoints to the servoL queue (mirrors KUKADriver.stream_refill).
        Returns (count_written, error_or_empty)."""
        err = state.get("error", "")
        if err:
            return 0, err

        q = state.get("queue")
        if q is None:
            return 0, "No servo queue"

        added = 0
        for wp in waypoints:
            if q.full():
                break
            try:
                pos_m, euler_rad = wp
                rx, ry, rz = rpy2rv(float(euler_rad[0]), float(euler_rad[1]), float(euler_rad[2]))
                target = [float(pos_m[0]), float(pos_m[1]), float(pos_m[2]), rx, ry, rz]
                q.put_nowait(target)
                state["written"] += 1
                added += 1
            except queue.Full:
                break
            except Exception as e:
                return added, str(e)

        return added, state.get("error", "")

    def stream_read_rd_idx(self) -> int:
        """Return how many source waypoints the servo worker has completed."""
        st = self._servo_state
        if st:
            return st.get("waypoints_completed", 0)
        return 0

    def stream_is_done(self, state: dict) -> bool:
        """True when all waypoints are written and fully executed."""
        if state.get("error"):
            return True
        return (state["written"] >= state["total"]
                and state.get("waypoints_completed", 0) >= state["total"])

    def stream_read_done_id(self) -> int:
        """Return cmd_id once servo is done (for health display)."""
        st = self._servo_state
        if st and st.get("waypoints_completed", 0) >= st.get("total", 1):
            return st.get("cmd_id", 0)
        return 0

    def _servo_cleanup(self, reason: str = ""):
        """Stop any running servo worker thread."""
        st = self._servo_state
        if st is not None:
            if reason and not st.get("error"):
                st["error"] = str(reason)
            st["stop"] = True
            t = st.get("thread")
            if t and t.is_alive():
                t.join(timeout=2.0)
            self._servo_state = None

    def stream_restore_control(self):
        """Stop servoL and restore normal RTDE control."""
        self._servo_cleanup()

    def stream_abort(self, reason: str = "Stopped by user"):
        """Abort active servo streaming session with an explicit reason."""
        self._servo_cleanup(reason=reason)

    def set_manual_mode(self, enabled: bool) -> str:
        self._ensure()
        try:
            self._backend.freedrive(enabled)
            return ""
        except Exception as e:
            return str(e)

    def go_home(self, joints_deg=None, vel=1.05, acc=1.4) -> str:
        """Move to home joints (degrees). vel in rad/s, acc in rad/s^2."""
        self._ensure()
        try:
            if joints_deg is None:
                joints_deg = [0.0] * 6
            joints_rad = [deg2rad(d) for d in joints_deg]
            self._backend.movej(joints_rad, vel, acc, wait=False)
            return ""
        except Exception as e:
            return str(e)

    def send_program(self, content: str) -> str:
        """Send URScript to robot for immediate execution."""
        self._ensure()
        secondary_err = self._send_secondary_program(content, port=30002, timeout=2.0)
        if not secondary_err:
            self._last_program_transport = "secondary_socket_30002"
            self._last_staged_program_content = str(content or "")
            self._last_staged_program_name = "send_program"
            self._last_staged_remote_file = ""
            self._last_staged_remote_urp_file = ""
            return ""

        try:
            self._backend.send_program(content)
            self._last_program_transport = (
                "rtde_custom_inline" if self._backend_name == "ur_rtde" else "urx_secondary_inline"
            )
            self._last_staged_remote_file = ""
            self._last_staged_remote_urp_file = ""
            return ""
        except Exception as e:
            return f"{secondary_err}; fallback send failed: {e}"

    def _as_filename(self, program_name: str, extension: str) -> str:
        base = str(program_name or "animaquina").strip() or "animaquina"
        safe = "".join(ch if ch.isalnum() or ch in {"_", "-", "."} else "_" for ch in base)
        if not safe:
            safe = "animaquina"
        ext = str(extension or "").strip() or ".script"
        if not ext.startswith("."):
            ext = "." + ext
        if not safe.lower().endswith(ext.lower()):
            safe += ext
        return safe

    def _upload_text_file_sftp(
        self,
        *,
        content: str,
        remote_filename: str,
        remote_path: str = "/programs",
        username: str = "root",
        password: str = "easybot",
        port: int = 22,
        timeout: float = 8.0,
    ) -> tuple:
        """Upload arbitrary text content as one file via SFTP. Returns (err, remote_file)."""
        self._ensure()

        if not ur_sftp_available():
            return ("SFTP_CONNECT_FAILED: paramiko is not installed (run Install UR Dependencies)", "")

        endpoint = self._endpoint or ""
        if not endpoint:
            return ("SFTP_CONNECT_FAILED: UR endpoint unavailable for SFTP upload", "")

        remote_dir = str(remote_path or "/programs").strip().replace("\\", "/")
        if not remote_dir:
            remote_dir = "/programs"
        if not remote_dir.startswith("/"):
            remote_dir = "/" + remote_dir.lstrip("/")

        remote_file = posixpath.join(remote_dir.rstrip("/"), str(remote_filename or "").strip())
        payload = str(content or "")

        ssh = None
        sftp = None
        local_path = ""
        try:
            with tempfile.NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                suffix=os.path.splitext(remote_filename)[1] or ".txt",
                prefix="animaquina_",
                newline="\n",
                delete=False,
            ) as tmp:
                tmp.write(payload)
                local_path = tmp.name

            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            ssh.connect(
                hostname=endpoint,
                port=int(port),
                username=str(username or "root"),
                password=str(password or ""),
                timeout=float(timeout),
                banner_timeout=float(timeout),
                auth_timeout=float(timeout),
                look_for_keys=False,
                allow_agent=False,
            )
            sftp = ssh.open_sftp()
            try:
                sftp.chdir(remote_dir)
            except Exception as e:
                return (f"REMOTE_PATH_INVALID: {remote_dir} ({e})", "")

            sftp.put(local_path, remote_file, confirm=True)
            try:
                attrs = sftp.stat(remote_file)
                if int(getattr(attrs, "st_size", 0)) <= 0:
                    return (f"SFTP_CONNECT_FAILED: uploaded file has invalid size ({remote_file})", "")
            except Exception:
                pass

            return ("", remote_file)
        except paramiko.AuthenticationException as e:
            return (f"SFTP_AUTH_FAILED: {e}", "")
        except (paramiko.SSHException, socket.timeout, TimeoutError, OSError) as e:
            return (f"SFTP_CONNECT_FAILED: {e}", "")
        except Exception as e:
            return (f"SFTP_CONNECT_FAILED: {e}", "")
        finally:
            if sftp is not None:
                try:
                    sftp.close()
                except Exception:
                    pass
            if ssh is not None:
                try:
                    ssh.close()
                except Exception:
                    pass
            if local_path:
                try:
                    os.unlink(local_path)
                except Exception:
                    pass

    def upload_program_sftp(
        self,
        content: str,
        program_name: str,
        remote_path: str = "/programs",
        username: str = "root",
        password: str = "easybot",
        port: int = 22,
        timeout: float = 8.0,
    ) -> str:
        """Upload script file to UR controller over SFTP."""
        script_name = self._as_filename(program_name, ".script")
        payload = str(content or "")
        if not payload.endswith("\n"):
            payload += "\n"
        err, remote_file = self._upload_text_file_sftp(
            content=payload,
            remote_filename=script_name,
            remote_path=remote_path,
            username=username,
            password=password,
            port=port,
            timeout=timeout,
        )
        if err:
            return err

        self._last_program_transport = "sftp"
        self._last_staged_program_content = payload
        self._last_staged_program_name = script_name
        self._last_staged_remote_file = remote_file
        self._last_staged_remote_urp_file = ""
        return ""

    def upload_urp_sftp(
        self,
        content: str,
        program_name: str,
        remote_path: str = "/programs",
        username: str = "root",
        password: str = "easybot",
        port: int = 22,
        timeout: float = 8.0,
    ) -> str:
        """Upload launcher URP file to UR controller over SFTP."""
        urp_name = self._as_filename(program_name, ".urp")
        err, remote_file = self._upload_text_file_sftp(
            content=str(content or ""),
            remote_filename=urp_name,
            remote_path=remote_path,
            username=username,
            password=password,
            port=port,
            timeout=timeout,
        )
        if err:
            return err
        self._last_program_transport = "sftp"
        self._last_staged_remote_urp_file = remote_file
        return ""

    def _stage_program_runtime_script(self, content: str, program_name: str = "animaquina") -> str:
        """Legacy runtime staging over script channels (no controller filesystem)."""
        self._ensure()

        secondary_err = self._send_secondary_program(content, port=30002, timeout=2.0)
        if not secondary_err:
            self._last_program_transport = "secondary_socket_30002"
            self._last_staged_program_content = str(content or "")
            self._last_staged_program_name = str(program_name or "animaquina")
            self._last_staged_remote_file = ""
            self._last_staged_remote_urp_file = ""
            return ""

        try:
            if hasattr(self._backend, "send_program_file"):
                self._last_program_transport = self._backend.send_program_file(content, program_name) or ""
            else:
                self._backend.send_program(content)
                self._last_program_transport = (
                    "rtde_custom_inline" if self._backend_name == "ur_rtde" else "urx_secondary_inline"
                )
            self._last_staged_program_content = str(content or "")
            self._last_staged_program_name = str(program_name or "animaquina")
            self._last_staged_remote_file = ""
            self._last_staged_remote_urp_file = ""
            return ""
        except Exception as e:
            return f"{secondary_err}; stage fallback failed: {e}"

    def stage_program(
        self,
        content: str,
        program_name: str = "animaquina",
        *,
        transport: str = "sftp",
        remote_path: str = "/programs",
        username: str = "root",
        password: str = "easybot",
        port: int = 22,
    ) -> str:
        """Stage exported URScript using selected transport."""
        mode = str(transport or "sftp").strip().lower()
        if mode in {"sftp", "full_program_sftp"}:
            return self.upload_program_sftp(
                content,
                program_name,
                remote_path=remote_path,
                username=username,
                password=password,
                port=port,
            )
        if mode in {"runtime_script", "runtime", "secondary"}:
            return self._stage_program_runtime_script(content, program_name)
        return f"REMOTE_PATH_INVALID: unknown stage transport '{transport}'"

    def upload_program(
        self,
        content: str,
        program_name: str,
        remote_path: str = "",
        username: str = "root",
        password: str = "easybot",
        port: int = 22,
    ) -> str:
        """Backward-compat wrapper: upload full program to controller filesystem."""
        return self.stage_program(
            content,
            program_name,
            transport="sftp",
            remote_path=remote_path,
            username=username,
            password=password,
            port=port,
        )

    def _dashboard_command(self, command: str, port: int = 29999, timeout: float = 4.0) -> tuple:
        endpoint = self._endpoint or ""
        if not endpoint:
            return ("", "UR endpoint unavailable for Dashboard command")
        try:
            with socket.create_connection((endpoint, int(port)), timeout=timeout) as sock:
                sock.settimeout(timeout)
                try:
                    sock.recv(4096)  # Banner
                except Exception:
                    pass
                sock.sendall((str(command).strip() + "\n").encode("utf-8"))
                resp = sock.recv(4096).decode("utf-8", errors="replace").strip()
                return (resp, "")
        except Exception as e:
            return ("", f"Dashboard command failed: {e}")

    def _dashboard_response_ok(self, response: str) -> bool:
        txt = (response or "").strip().lower()
        if not txt:
            return False
        for bad in ("failed", "error", "not found", "cannot", "unable"):
            if bad in txt:
                return False
        return True

    def select_program(self, program_name: str, remote_path: str = "", dashboard_port: int = 29999) -> str:
        """Backward-compat wrapper: Dashboard supports URP load, not script file load."""
        self._ensure()
        return "Dashboard load for .script is not used; stage/send the script directly instead"

    def dashboard_is_remote_control(self, dashboard_port: int = 29999) -> tuple:
        """Return (is_remote_control, err)."""
        self._ensure()
        resp, err = self._dashboard_command("is in remote control", port=dashboard_port)
        if err:
            return (False, err)
        txt = (resp or "").strip().lower()
        if "true" in txt:
            return (True, "")
        if "false" in txt:
            return (False, "")
        return (False, f"Dashboard remote-control check failed: {resp}")

    def dashboard_program_state(self, dashboard_port: int = 29999) -> tuple:
        """Return (state, err), where state is paused|playing|stopped|unknown."""
        self._ensure()

        resp, err = self._dashboard_command("programState", port=dashboard_port)
        if not err:
            txt = (resp or "").strip().lower()
            if "paused" in txt:
                return ("paused", "")
            if "playing" in txt:
                return ("playing", "")
            if "stopped" in txt:
                return ("stopped", "")

        # Fallback for controllers with limited Dashboard commands.
        resp, err = self._dashboard_command("running", port=dashboard_port)
        if err:
            return ("unknown", "")
        txt = (resp or "").strip().lower()
        if "true" in txt:
            return ("playing", "")
        if "false" in txt:
            return ("stopped", "")
        return ("unknown", "")

    def play_program(
        self,
        dashboard_port: int = 29999,
        launcher_urp: str = "",
        remote_path: str = "",
        force_reload: bool = False,
    ) -> str:
        """Start staged program using dashboard load/play on a launcher URP."""
        self._ensure()

        launcher = str(launcher_urp or "").strip()
        if launcher:
            is_remote, remote_err = self.dashboard_is_remote_control(dashboard_port)
            if remote_err:
                return f"DASHBOARD_PLAY_FAILED: {remote_err}"
            if not is_remote:
                return "REMOTE_CONTROL_DISABLED: enable Remote Control to use dashboard load/play"

            state, _ = self.dashboard_program_state(dashboard_port)
            if force_reload and state in {"paused", "playing"}:
                stop_resp, stop_err = self._dashboard_command("stop", port=dashboard_port)
                if stop_err:
                    return f"DASHBOARD_PLAY_FAILED: {stop_err}"
                if not self._dashboard_response_ok(stop_resp):
                    return f"DASHBOARD_PLAY_FAILED: {stop_resp}"
                state = "stopped"

            # Normal Play resumes paused programs; force_reload always reloads launcher.
            should_load = bool(force_reload) or state not in {"paused", "playing"}

            if should_load:
                # When remote_path is configured, load strictly from that directory to avoid
                # accidentally running an older launcher from a different folder.
                remote_dir = str(remote_path or "").strip().replace("\\", "/")
                load_target = launcher
                if remote_dir:
                    if not remote_dir.startswith("/"):
                        remote_dir = "/" + remote_dir.lstrip("/")
                    remote_dir = remote_dir.rstrip("/")
                    if remote_dir:
                        load_target = f"{remote_dir}/{launcher}"

                resp, err = self._dashboard_command(f"load {load_target}", port=dashboard_port)
                if err:
                    return f"DASHBOARD_PLAY_FAILED: {err}"
                if not self._dashboard_response_ok(resp):
                    return f"DASHBOARD_PLAY_FAILED: {resp}"

        resp, err = self._dashboard_command("play", port=dashboard_port)
        if err:
            return f"DASHBOARD_PLAY_FAILED: {err}"
        if not self._dashboard_response_ok(resp):
            return f"DASHBOARD_PLAY_FAILED: {resp}"
        return ""

    def pause_program(self, dashboard_port: int = 29999) -> str:
        """Pause running program using Dashboard server."""
        self._ensure()
        resp, err = self._dashboard_command("pause", port=dashboard_port)
        if err:
            return err
        if not self._dashboard_response_ok(resp):
            return f"Dashboard pause failed: {resp}"
        return ""

    def stop_program(self, dashboard_port: int = 29999) -> str:
        """Stop running program using Dashboard server."""
        self._ensure()
        resp, err = self._dashboard_command("stop", port=dashboard_port)
        if err:
            return err
        if not self._dashboard_response_ok(resp):
            return f"Dashboard stop failed: {resp}"
        return ""
