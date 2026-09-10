# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Animaquina Core — driver interface (Section 5 Layer 1, Section 8)

# Capability flags (Section 8)
CAP_CONNECT = 1 << 0
CAP_READ_TCP = 1 << 1
CAP_READ_JOINTS = 1 << 2
CAP_READ_BASE = 1 << 3
CAP_MANUAL_MODE = 1 << 4
CAP_MOVE_TO_TARGET = 1 << 5
CAP_EXECUTE_PATH = 1 << 6
CAP_EXPORT_PROGRAM = 1 << 7
CAP_HOME = 1 << 8
CAP_UPLOAD_PROGRAM = 1 << 9
CAP_SELECT_PROGRAM = 1 << 10
CAP_RESET = 1 << 11


class DriverBase:
    """
    Robot driver interface. Hardware/API only; never touch Blender data.
    Canonical units: position m, orientation Euler XYZ rad, joints deg.
    Robot-specific coordinate conversions (e.g. KUKA ZYX→XYZ, $BASE inversion)
    MUST be done inside the driver before returning — the runtime layer applies
    values directly without any robot-specific math.
    """

    def capabilities(self):
        """Bitmask of CAP_* flags."""
        return 0

    def connect(self, endpoint: str, port: int = 0, *, slot=None) -> str:
        """Connect to robot. Returns empty on success, else error message. slot= for backend hints (UR)."""
        return "Not implemented"

    def disconnect(self) -> None:
        """Close connection."""
        pass

    def read_tcp(self) -> tuple:
        """Return (pos_m[3], euler_rad[3]) Blender-ready XYZ, or raise."""
        raise NotImplementedError

    def read_joints(self) -> tuple:
        """Return (j1_deg, ..., j6_deg)."""
        raise NotImplementedError

    def read_home_joints(self) -> tuple:
        """Return robot home joints from controller/system variables as (j1_deg, ..., j6_deg)."""
        raise NotImplementedError

    def write_home_joints(self, joints_deg) -> str:
        """Store robot home joints into controller/system variables. Returns '' or error."""
        raise NotImplementedError

    def read_base(self) -> tuple:
        """Return Blender-ready (location[3], rotation_euler[3]) for J0.
        All robot-specific conversions (coord system, inversion, etc.) must be
        done here so the runtime can apply values directly."""
        raise NotImplementedError

    def read_var(self, name: str) -> str:
        """Read a named controller variable. Returns string value. Raises on error.
        Variable names are robot-specific (KUKA: $OV_PRO; UR: output_double_register_0; xArm: state)."""
        raise NotImplementedError(f"{type(self).__name__} does not support variable reading")

    def write_var(self, name: str, value) -> str:
        """Write a named controller variable. Returns '' on success, else error string."""
        return f"{type(self).__name__} does not support variable writing"

    def move_to_pose(self, pos_m, euler_rad, speed: float, acc: float, radius: float, wait: bool) -> str:
        """Linear move TCP to pose. Returns '' or error."""
        return "Not implemented"

    def move_to_pose_ptp(self, pos_m, euler_rad, vel: float, acc: float, radius: float, wait: bool) -> str:
        """PTP (joint-space) move TCP to pose. Returns '' or error."""
        return "Not implemented"

    def execute_ptp_path(self, waypoints: list, speed: float, acc: float, radius: float, wait: bool) -> str:
        """Execute path. Each waypoint (pos_m[3], euler_rad[3]) or (pos_m, euler_rad, params). Returns '' or error."""
        return "Not implemented"

    def set_manual_mode(self, enabled: bool) -> str:
        """Enable/disable manual (freedrive) mode. Returns '' or error."""
        return "Not implemented"

    def go_home(self, joints_deg=None, vel=0.5, acc=0.5) -> str:
        """Move to home if CAP_HOME. joints_deg in degrees, vel/acc in driver-native units. Returns '' or error."""
        return "Not implemented"

    def reset(self) -> str:
        """Clear errors and return to safe state (SDK home). Returns '' or error."""
        return "Not implemented"

    def send_program(self, content: str) -> str:
        """Send a script/program to the robot for immediate execution. Returns '' or error."""
        return "Not implemented"

    def upload_program(self, content: str, program_name: str) -> str:
        """Upload program text to robot controller. Returns '' or error."""
        return "Not implemented"

    def select_program(self, program_name: str) -> str:
        """Select/load a program on the robot controller. Returns '' or error."""
        return "Not implemented"
