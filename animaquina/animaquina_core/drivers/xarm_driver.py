# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Animaquina Core — xArm / UFactory driver (xarm Python SDK)
#
# REFERENCE: oldversions/animaquinauf_0.0.9 — XArmAPI(ip), get_position (mm, deg/rad),
#   get_servo_angle(is_radian=True), set_position(x,y,z,roll,pitch,yaw mm/deg), set_mode(0), set_state(0).
#   Joint axes xarm6: Y,-Z,-Z,Y,-Z,Y; uf850: Y,Z,-Z,Y,Z,Y.

import math
import importlib

# Import paths are set up centrally by animaquina.__init__._bootstrap_import_paths()

from .base import (
    DriverBase,
    CAP_CONNECT,
    CAP_READ_TCP,
    CAP_READ_JOINTS,
    CAP_MANUAL_MODE,
    CAP_MOVE_TO_TARGET,
    CAP_EXECUTE_PATH,
    CAP_HOME,
    CAP_RESET,
)
from ..runtime.conversions import rad2deg

try:
    from xarm.wrapper import XArmAPI
except ImportError:
    XArmAPI = None


def _refresh_xarm_import() -> bool:
    """Refresh xArm SDK import in case it was installed after addon import."""
    global XArmAPI
    importlib.invalidate_caches()
    try:
        mod = importlib.import_module("xarm.wrapper")
        XArmAPI = getattr(mod, "XArmAPI", None)
    except ImportError:
        XArmAPI = None
    return XArmAPI is not None


def xarm_sdk_available() -> bool:
    """Check if xArm SDK is installed and importable."""
    return _refresh_xarm_import()


class XArmDriver(DriverBase):
    """xArm / UFactory via xarm Python SDK. Pose: position mm→m, Euler rad (roll,pitch,yaw); joints: deg."""

    def __init__(self):
        self._arm = None
        self._rt_puppet_active = False
        self._rt_puppet_reduced_mode = False

    def capabilities(self):
        return (
            CAP_CONNECT | CAP_READ_TCP | CAP_READ_JOINTS
            | CAP_MANUAL_MODE | CAP_MOVE_TO_TARGET | CAP_EXECUTE_PATH | CAP_HOME | CAP_RESET
        )

    def connect(self, endpoint: str, port: int = 0, *, slot=None) -> str:
        if not xarm_sdk_available():
            return "xarm SDK not found - use 'Install xArm Dependencies'"
        try:
            if self._arm is not None:
                try:
                    self._arm.disconnect()
                except Exception:
                    pass
                self._arm = None
            # port = IP string; SDK connects in constructor when do_not_open=False
            self._arm = XArmAPI(endpoint, is_radian=False, do_not_open=True)
            self._arm.connect(port=endpoint)
            self._arm.motion_enable(enable=True)
            self._arm.set_mode(0)
            self._arm.set_state(state=0)
            # Test read
            self._arm.get_position(is_radian=True)
            return ""
        except Exception as e:
            self._arm = None
            return str(e)

    def disconnect(self) -> None:
        if self._arm is not None:
            try:
                if self._rt_puppet_active:
                    try:
                        self.realtime_puppet_stop()
                    except Exception:
                        pass
                self._arm.disconnect()
            except Exception:
                pass
            self._arm = None
            self._rt_puppet_active = False
            self._rt_puppet_reduced_mode = False

    def _ensure(self):
        if self._arm is None:
            raise RuntimeError("xArm not connected")

    def read_tcp(self) -> tuple:
        self._ensure()
        code, pos = self._arm.get_position(is_radian=True)
        if code != 0:
            raise RuntimeError(f"xArm get_position code {code}")
        # pos: [x_mm, y_mm, z_mm, roll_rad, pitch_rad, yaw_rad]
        pos_m = (pos[0] * 0.001, pos[1] * 0.001, pos[2] * 0.001)
        euler_rad = (float(pos[3]), float(pos[4]), float(pos[5]))
        return (pos_m, euler_rad)

    def read_joints(self) -> tuple:
        self._ensure()
        code, angles = self._arm.get_servo_angle(is_radian=True)
        if code != 0:
            raise RuntimeError(f"xArm get_servo_angle code {code}")
        # angles can be 6 or 7; we need first 6 in deg
        return tuple(rad2deg(float(angles[i])) for i in range(min(6, len(angles))))

    def read_base(self) -> tuple:
        return ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    # ── Variable reading via xArm SDK ────────────────────────────
    # Supported variable names:
    #   state, mode, error_code                — arm state / mode / error
    #   cgpio_digital_input_N                  — controller GPIO digital input (0-15)
    #   cgpio_digital_output_N                 — controller GPIO digital output (0-15)
    #   cgpio_analog_input_N                   — controller GPIO analog input (0-1)
    #   cgpio_analog_output_N                  — controller GPIO analog output (0-1)
    #   tgpio_digital_input_N                  — tool GPIO digital input (0-1)
    #   tgpio_digital_output_N                 — tool GPIO digital output (0-1)
    #   tgpio_analog_input_N                   — tool GPIO analog input (0-1)
    #   joint_temperature_N                    — servo temperature for joint N (0-5)
    #   tcp_load                               — TCP payload [mass, cx, cy, cz]

    def read_var(self, name: str) -> str:
        self._ensure()

        # Simple state reads
        if name == "state":
            return str(self._arm.state)
        if name == "mode":
            return str(self._arm.mode)
        if name == "error_code":
            return str(self._arm.error_code)

        # Controller GPIO digital
        if name.startswith("cgpio_digital_input_") or name.startswith("cgpio_digital_output_"):
            code, states = self._arm.get_cgpio_state()
            if code != 0:
                raise RuntimeError(f"xArm get_cgpio_state code {code}")
            # states: [cgpio_state, cgpio_code, di, do, ai0, ai1, ao0, ao1, ...]
            # di / do are bitmasks in states[2] / states[3]
            if name.startswith("cgpio_digital_input_"):
                pin = int(name[len("cgpio_digital_input_"):])
                return str((int(states[2]) >> pin) & 1)
            else:
                pin = int(name[len("cgpio_digital_output_"):])
                return str((int(states[3]) >> pin) & 1)

        # Controller GPIO analog
        if name.startswith("cgpio_analog_input_"):
            pin = int(name[len("cgpio_analog_input_"):])
            code, val = self._arm.get_cgpio_analog(pin)
            if code != 0:
                raise RuntimeError(f"xArm get_cgpio_analog code {code}")
            return str(val)
        if name.startswith("cgpio_analog_output_"):
            pin = int(name[len("cgpio_analog_output_"):])
            code, val = self._arm.get_cgpio_analog(pin + 10)  # SDK uses offset for outputs
            if code != 0:
                raise RuntimeError(f"xArm get_cgpio_analog code {code}")
            return str(val)

        # Tool GPIO digital
        if name.startswith("tgpio_digital_input_") or name.startswith("tgpio_digital_output_"):
            code, di0, di1, do0, do1 = self._arm.get_tgpio_digital()
            if code != 0:
                raise RuntimeError(f"xArm get_tgpio_digital code {code}")
            if name.startswith("tgpio_digital_input_"):
                pin = int(name[len("tgpio_digital_input_"):])
                return str([di0, di1][pin] if pin < 2 else 0)
            else:
                pin = int(name[len("tgpio_digital_output_"):])
                return str([do0, do1][pin] if pin < 2 else 0)

        # Tool GPIO analog
        if name.startswith("tgpio_analog_input_"):
            pin = int(name[len("tgpio_analog_input_"):])
            code, val = self._arm.get_tgpio_analog(pin)
            if code != 0:
                raise RuntimeError(f"xArm get_tgpio_analog code {code}")
            return str(val)

        # Joint temperatures
        if name.startswith("joint_temperature_"):
            idx = int(name[len("joint_temperature_"):])
            code, temps = self._arm.get_servo_temperature()
            if code != 0:
                raise RuntimeError(f"xArm get_servo_temperature code {code}")
            if idx < len(temps):
                return str(temps[idx])
            raise ValueError(f"Joint index {idx} out of range")

        # TCP payload
        if name == "tcp_load":
            code, payload = self._arm.get_tcp_load()
            if code != 0:
                raise RuntimeError(f"xArm get_tcp_load code {code}")
            return str(payload)

        raise ValueError(
            f"Unknown xArm variable '{name}'. Use: state, mode, error_code, "
            f"cgpio_digital_input_N, cgpio_analog_input_N, tgpio_digital_input_N, "
            f"joint_temperature_N, tcp_load, etc."
        )

    def move_to_pose(self, pos_m, euler_rad, speed: float, acc: float, radius: float, wait: bool) -> str:
        self._ensure()
        try:
            x_mm = float(pos_m[0]) * 1000.0
            y_mm = float(pos_m[1]) * 1000.0
            z_mm = float(pos_m[2]) * 1000.0
            roll_deg = rad2deg(float(euler_rad[0]))
            pitch_deg = rad2deg(float(euler_rad[1]))
            yaw_deg = rad2deg(float(euler_rad[2]))
            speed_mm_s = max(1.0, min(1000.0, speed * 500.0))  # map 0..1 to reasonable mm/s
            code = self._arm.set_position(
                x=x_mm, y=y_mm, z=z_mm,
                roll=roll_deg, pitch=pitch_deg, yaw=yaw_deg,
                is_radian=False, speed=speed_mm_s, wait=wait,
                radius=radius if radius > 0 else None,
            )
            if code != 0:
                return f"xArm set_position code {code}"
            return ""
        except Exception as e:
            return str(e)

    def _map_linear_speed_mm_s(self, speed_m_s: float) -> float:
        return max(1.0, min(1000.0, float(speed_m_s) * 500.0))

    def _map_linear_acc_mm_s2(self, acc_m_s2: float) -> float:
        return max(10.0, min(50000.0, float(acc_m_s2) * 5000.0))

    def realtime_puppet_start(self, boundary_mm=None) -> str:
        """Enter servo-cartesian mode for continuous target following."""
        self._ensure()
        try:
            code = self._arm.motion_enable(enable=True)
            if code != 0:
                return f"xArm motion_enable code {code}"
            # Optional reduced boundary (best effort; only applied when supported by firmware).
            self._rt_puppet_reduced_mode = False
            if boundary_mm is not None and len(boundary_mm) == 6 and hasattr(self._arm, "set_reduced_tcp_boundary"):
                try:
                    vals = [float(v) for v in boundary_mm]
                    bcode = self._arm.set_reduced_tcp_boundary(vals)
                    if bcode == 0 and hasattr(self._arm, "set_reduced_mode"):
                        rcode = self._arm.set_reduced_mode(True)
                        if rcode == 0:
                            self._rt_puppet_reduced_mode = True
                except Exception:
                    pass
            code = self._arm.set_mode(1)  # servo motion mode
            if code != 0:
                return f"xArm set_mode(1) code {code}"
            code = self._arm.set_state(state=0)
            if code != 0:
                return f"xArm set_state(0) code {code}"
            self._rt_puppet_active = True
            return ""
        except Exception as e:
            return str(e)

    def realtime_puppet_step(self, pos_m, euler_rad, speed: float, acc: float) -> str:
        """Stream one cartesian servo setpoint (x/y/z in m, euler XYZ in rad)."""
        self._ensure()
        if not self._rt_puppet_active:
            err = self.realtime_puppet_start()
            if err:
                return err
        try:
            x_mm = float(pos_m[0]) * 1000.0
            y_mm = float(pos_m[1]) * 1000.0
            z_mm = float(pos_m[2]) * 1000.0
            roll_deg = rad2deg(float(euler_rad[0]))
            pitch_deg = rad2deg(float(euler_rad[1]))
            yaw_deg = rad2deg(float(euler_rad[2]))
            vals = [x_mm, y_mm, z_mm, roll_deg, pitch_deg, yaw_deg]
            if not all(math.isfinite(v) for v in vals):
                return "xArm realtime step rejected: non-finite target values"
            code = self._arm.set_servo_cartesian(
                vals,
                speed=self._map_linear_speed_mm_s(speed),
                mvacc=self._map_linear_acc_mm_s2(acc),
                is_radian=False,
            )
            if code != 0:
                return f"xArm set_servo_cartesian code {code}"
            return ""
        except Exception as e:
            return str(e)

    def realtime_puppet_stop(self) -> str:
        """Leave servo mode and restore normal position mode."""
        self._ensure()
        issues = []
        try:
            # Deceleration stop (best effort).
            code = self._arm.set_state(4)
            if code != 0:
                issues.append(f"set_state(4) code {code}")
        except Exception as ex:
            issues.append(str(ex))
        if self._rt_puppet_reduced_mode and hasattr(self._arm, "set_reduced_mode"):
            try:
                code = self._arm.set_reduced_mode(False)
                if code != 0:
                    issues.append(f"set_reduced_mode(False) code {code}")
            except Exception as ex:
                issues.append(str(ex))
            self._rt_puppet_reduced_mode = False
        try:
            code = self._arm.set_mode(0)
            if code != 0:
                issues.append(f"set_mode(0) code {code}")
        except Exception as ex:
            issues.append(str(ex))
        try:
            code = self._arm.set_state(state=0)
            if code != 0:
                issues.append(f"set_state(0) code {code}")
        except Exception as ex:
            issues.append(str(ex))
        self._rt_puppet_active = False
        if issues:
            return "xArm realtime stop warning: " + issues[0]
        return ""

    def move_to_pose_ptp(self, pos_m, euler_rad, vel: float, acc: float, radius: float, wait: bool) -> str:
        self._ensure()
        try:
            x_mm = float(pos_m[0]) * 1000.0
            y_mm = float(pos_m[1]) * 1000.0
            z_mm = float(pos_m[2]) * 1000.0
            roll_deg = rad2deg(float(euler_rad[0]))
            pitch_deg = rad2deg(float(euler_rad[1]))
            yaw_deg = rad2deg(float(euler_rad[2]))
            speed_deg_s = max(1.0, min(180.0, rad2deg(vel)))
            code = self._arm.set_position(
                x=x_mm, y=y_mm, z=z_mm,
                roll=roll_deg, pitch=pitch_deg, yaw=yaw_deg,
                is_radian=False, speed=speed_deg_s, wait=wait,
                radius=radius if radius > 0 else None,
                motion_type=1,  # 1 = joint motion (PTP)
            )
            if code != 0:
                return f"xArm set_position(ptp) code {code}"
            return ""
        except Exception as e:
            return str(e)

    def execute_ptp_path(self, waypoints: list, speed: float, acc: float, radius: float, wait: bool) -> str:
        self._ensure()
        try:
            speed_mm_s = max(1.0, min(1000.0, speed * 500.0))
            for wp in waypoints:
                pos_m = wp[0] if len(wp) > 0 else (0, 0, 0)
                euler_rad = wp[1] if len(wp) > 1 else (0, 0, 0)
                code = self._arm.set_position(
                    x=float(pos_m[0]) * 1000.0,
                    y=float(pos_m[1]) * 1000.0,
                    z=float(pos_m[2]) * 1000.0,
                    roll=rad2deg(float(euler_rad[0])),
                    pitch=rad2deg(float(euler_rad[1])),
                    yaw=rad2deg(float(euler_rad[2])),
                    is_radian=False,
                    speed=speed_mm_s,
                    wait=wait,
                    radius=radius if radius > 0 else None,
                )
                if code != 0:
                    return f"xArm set_position code {code}"
            return ""
        except Exception as e:
            return str(e)

    def set_manual_mode(self, enabled: bool) -> str:
        self._ensure()
        try:
            if enabled:
                # Joint teaching mode (freedrive-like)
                code = self._arm.set_mode(2)
                if code != 0:
                    return f"xArm set_mode(2) code {code}"
                code = self._arm.set_state(state=0)
            else:
                code = self._arm.set_mode(0)
                if code != 0:
                    return f"xArm set_mode(0) code {code}"
                code = self._arm.set_state(state=0)
            if code != 0:
                return f"xArm set_state code {code}"
            return ""
        except Exception as e:
            return str(e)

    def go_home(self, joints_deg=None, vel=0.5, acc=0.5) -> str:
        self._ensure()
        try:
            if joints_deg is None:
                joints_deg = [0.0] * 6
            # vel/acc arrive in rad/s and rad/s²; SDK expects °/s and °/s² when is_radian=False
            speed_deg = max(1.0, min(180.0, rad2deg(vel)))
            acc_deg = max(1.0, min(1500.0, rad2deg(acc)))
            code = self._arm.set_servo_angle(
                angle=list(joints_deg),
                speed=speed_deg,
                mvacc=acc_deg,
                is_radian=False,
                wait=True,
            )
            if code != 0:
                return f"xArm set_servo_angle(home) code {code}"
            return ""
        except Exception as e:
            return str(e)

    def stop_motion(self) -> str:
        """Emergency stop — immediately halt all motion."""
        self._ensure()
        try:
            if self._rt_puppet_active:
                self._rt_puppet_active = False
            self._arm.emergency_stop()
            return ""
        except Exception as e:
            return str(e)

    def reset(self) -> str:
        """Clear errors and move to SDK home (xArm move_gohome clears errors/warnings)."""
        self._ensure()
        try:
            self._arm.clean_error()
            self._arm.clean_warn()
            self._arm.set_mode(0)
            self._arm.set_state(state=0)
            if hasattr(self._arm, "move_gohome"):
                code = self._arm.move_gohome(wait=True)
                if code != 0:
                    return f"xArm move_gohome code {code}"
                return ""
            return ""
        except Exception as e:
            return str(e)
