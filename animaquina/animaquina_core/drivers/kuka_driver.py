# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Animaquina Core — KUKA driver (kukaproxydriver)
#
# REFERENCE: oldversions/animaquinakuka_0.0.8/animaquinakuka/__init__.py
# Always match old parsing logic to avoid regressions. Key snippets:
#
# $POS_ACT (get_robotTCP, ~171): robotscale=0.001; parts = currentPos.split(',');
#   sna_x = float(parts[0].split('X')[1].strip())*robotscale  (same Y,Z);
#   sna_a = float(parts[3].split('A')[1].strip())  (deg, same B,C).
# TCP → Blender (update_tcp, ~266): rotation_euler[2]=rpy[0], [1]=rpy[1], [0]=rpy[2]  (C,B,A rad).
#
# $AXIS_ACT (get_robotPose, ~191): parts = currentPose.split(',');
#   sna_j1 = float(parts[0].split('A1')[1].strip())  … parts[5].split('A6')[1].
#   Old assumes first 6 segments are A1..A6; controller may send {E6AXIS: A1 x, A2 y, ...}
#   so parts[0] can be "{E6AXIS: A1 21.75". We search for "A1".."A6" in any part (handles E1..E4 too).
#
# $BASE (get_robotBase, ~309; parse_kuka_frame, ~276): replace("{FRAME:", "").replace("}", "").strip();
#   parts by comma; base_values[i] = float(parts[i].split('X'|'Y'|'Z'|'A'|'B'|'C')[1].strip());
#   base x,y,z = values[0:3] * -robotscale; base a,b,c = d2r(values[3:6]).

import math
import os
import threading

# Import paths are set up centrally by animaquina.__init__._bootstrap_import_paths()

from .base import (
    DriverBase,
    CAP_CONNECT,
    CAP_READ_TCP,
    CAP_READ_JOINTS,
    CAP_READ_BASE,
    CAP_MOVE_TO_TARGET,
    CAP_EXECUTE_PATH,
    CAP_EXPORT_PROGRAM,
    CAP_UPLOAD_PROGRAM,
    CAP_SELECT_PROGRAM,
)
from ..runtime.conversions import deg2rad, rad2deg

try:
    from kukaproxydriver import KUKA
except ImportError:
    KUKA = None


def _parse_pos_act(s: str) -> tuple:
    """Parse $POS_ACT. Old: get_robotTCP — parts=split(','); X,Y,Z * 0.001; A,B,C deg; Blender euler (C,B,A) rad."""
    if not s or not s.strip():
        raise ValueError("Empty $POS_ACT")
    parts = [p.strip() for p in s.split(",")]
    if len(parts) < 6:
        raise ValueError(f"$POS_ACT needs 6 parts, got {len(parts)}")
    # Old: float(parts[i].split('X'|'Y'|'Z'|'A'|'B'|'C')[1].strip())
    x_mm = float(parts[0].split("X")[1].strip())
    y_mm = float(parts[1].split("Y")[1].strip())
    z_mm = float(parts[2].split("Z")[1].strip())
    a_deg = float(parts[3].split("A")[1].strip())
    b_deg = float(parts[4].split("B")[1].strip())
    c_deg = float(parts[5].split("C")[1].strip())
    pos_m = (x_mm * 0.001, y_mm * 0.001, z_mm * 0.001)
    # Old update_tcp: rotation_euler[2]=rpy[0], [1]=rpy[1], [0]=rpy[2]  (C,B,A)
    euler_rad = (deg2rad(c_deg), deg2rad(b_deg), deg2rad(a_deg))
    return (pos_m, euler_rad)


def _parse_axis_act(s: str) -> tuple:
    """Parse $AXIS_ACT. Old: get_robotPose — parts[i].split('A1'..'A6')[1].strip(); first part can be '{E6AXIS: A1 21.75'."""
    if not s or not s.strip():
        raise ValueError("Empty $AXIS_ACT")
    parts = [p.strip() for p in s.split(",")]
    out = []
    for i in range(1, 7):
        prefix = f"A{i}"
        found = False
        for p in parts:
            if prefix in p:
                # Old: float(parts[i].split('A1')[1].strip()) — we search by axis name so {E6AXIS: A1 x} and E1..E4 work
                rest = p.split(prefix, 1)[1].strip()
                val_str = rest.split()[0] if rest.split() else rest
                out.append(float(val_str))
                found = True
                break
        if not found:
            raise ValueError(f"$AXIS_ACT missing {prefix} (got: {s[:80]!r}...)")
    return tuple(out)


def _parse_kuka_frame(s: str, pos_scale: float = 0.001) -> tuple:
    """Parse KUKA frame string ($BASE, $TOOL). Accepts {FRAME: X _, Y _, Z _, A _, B _, C _} or {X _, Y _, ...}.
    X,Y,Z in mm * pos_scale -> m; A,B,C deg -> rad (A,B,C order). Robot-specific conversions
    (ZYX→XYZ, inversion) happen after this, in the calling method."""
    if not s or not s.strip():
        raise ValueError("Empty frame string")
    # Strip braces and optional FRAME: prefix (same as old parse_kuka_frame / get_robotBase)
    s = s.strip().replace("{FRAME:", "").replace("}", "").replace("{", "").strip()
    parts = [p.strip() for p in s.split(",")]
    if len(parts) < 6:
        raise ValueError(f"Frame needs 6 parts, got {len(parts)}")
    # Extract X,Y,Z,A,B,C (order fixed in KUKA system variables)
    x_mm = float(parts[0].split("X")[1].strip())
    y_mm = float(parts[1].split("Y")[1].strip())
    z_mm = float(parts[2].split("Z")[1].strip())
    a_deg = float(parts[3].split("A")[1].strip())
    b_deg = float(parts[4].split("B")[1].strip())
    c_deg = float(parts[5].split("C")[1].strip())
    pos_m = (x_mm * pos_scale, y_mm * pos_scale, z_mm * pos_scale)
    # A, B, C in KUKA order; caller (_kuka_base_to_blender / etc.) handles conversion to Blender XYZ.
    euler_rad = (deg2rad(a_deg), deg2rad(b_deg), deg2rad(c_deg))
    return (pos_m, euler_rad)


def _kuka_base_to_blender(pos_m: tuple, euler_abc: tuple) -> tuple:
    """Convert raw $BASE frame (pos_m in meters, euler (A,B,C) in rad) to
    Blender-ready (location, rotation_euler) for J0.
    Builds the full rigid transform and inverts it so position and rotation
    are coupled — exactly like Blender parenting.
    Pure-math implementation (no mathutils dependency)."""
    from ..runtime.conversions import kuka_base_to_blender
    return kuka_base_to_blender(pos_m, euler_abc)


class KUKADriver(DriverBase):
    """KUKA via kukaproxydriver. TCP: $POS_ACT (mm, deg); joints: $AXIS_ACT (deg); base: $BASE.
    All socket access is serialized through _lock to prevent interleaving when the background
    poll thread and operators on the main thread both use the driver."""

    def __init__(self):
        self._robot = None
        self._lock = threading.Lock()

    # Command counter for MQ_CMD_ID handshake with mq_stream on the robot
    _cmd_id = 0

    def capabilities(self):
        return (
            CAP_CONNECT
            | CAP_READ_TCP
            | CAP_READ_JOINTS
            | CAP_READ_BASE
            | CAP_MOVE_TO_TARGET
            | CAP_EXECUTE_PATH
            | CAP_EXPORT_PROGRAM
            | CAP_UPLOAD_PROGRAM
            | CAP_SELECT_PROGRAM
        )

    def connect(self, endpoint: str, port: int = 0, *, slot=None) -> str:
        if KUKA is None:
            return "kukaproxydriver not found (expected animaquina_core/libs on path)"
        try:
            with self._lock:
                if self._robot is not None:
                    try:
                        self._robot.disconnect()
                    except Exception:
                        pass
                    self._robot = None
                self._robot = KUKA(endpoint)
                self._robot.read("$POS_ACT")
            return ""
        except Exception as e:
            self._robot = None
            return str(e)

    def disconnect(self) -> None:
        with self._lock:
            if self._robot is not None:
                try:
                    self._robot.disconnect()
                except Exception:
                    pass
                self._robot = None

    def _ensure(self):
        if self._robot is None:
            raise RuntimeError("KUKA not connected")

    def read_tcp(self) -> tuple:
        self._ensure()
        with self._lock:
            s = self._robot.read("$POS_ACT")
        return _parse_pos_act(s)

    def read_joints(self) -> tuple:
        self._ensure()
        with self._lock:
            s = self._robot.read("$AXIS_ACT")
        return _parse_axis_act(s)

    def read_home_joints(self) -> tuple:
        """Read KUKA home joints from XHOME (E6AXIS, degrees)."""
        self._ensure()
        with self._lock:
            s = self._robot.read("XHOME")
        return _parse_axis_act(s)

    def write_home_joints(self, joints_deg) -> str:
        """Store KUKA home joints to XHOME (E6AXIS, degrees)."""
        self._ensure()
        try:
            e6axis = self._format_e6axis(joints_deg)
            with self._lock:
                self._robot.write("XHOME", e6axis)
            return ""
        except Exception as e:
            return str(e)

    def read_base(self) -> tuple:
        """Return Blender-ready (location, rotation_euler) for J0.
        Builds the full $BASE rigid transform and inverts it so J0 acts as parent:
        position and rotation are coupled (inverse position is rotated)."""
        self._ensure()
        with self._lock:
            s = self._robot.read("$BASE")
        pos_m, euler_abc = _parse_kuka_frame(s, pos_scale=0.001)
        return _kuka_base_to_blender(pos_m, euler_abc)

    def read_tool(self) -> tuple:
        """Return parsed $TOOL frame (pos_m, euler_rad). Same FRAME format as $BASE; position scale +0.001."""
        self._ensure()
        with self._lock:
            s = self._robot.read("$TOOL")
        return _parse_kuka_frame(s, pos_scale=0.001)

    def upload_program(self, content: str, program_name: str, remote_path: str = "") -> str:
        """Upload KRL file text (.src or .dat) to the robot controller via C3 Bridge.
        Holds the lock for the entire multi-step transfer to prevent
        interleaving with background poll reads on the shared socket."""
        self._ensure()
        import tempfile
        tmp_path = None
        try:
            has_ext = program_name.endswith(".src") or program_name.endswith(".dat")
            file_name = program_name if has_ext else f"{program_name}.src"
            suffix = os.path.splitext(file_name)[1]
            with tempfile.NamedTemporaryFile(
                mode="w", suffix=suffix, delete=False, encoding="utf-8"
            ) as tmp:
                tmp.write(content)
                tmp_path = tmp.name
            if remote_path and not file_name.startswith("KRC:"):
                remote_full = remote_path.rstrip("\\/ ") + "\\" + file_name
            else:
                remote_full = file_name
            with self._lock:
                err = self._robot.upload_file(tmp_path, remote_full, copy_flags=64)
            return err
        except Exception as e:
            return str(e)
        finally:
            if tmp_path:
                try:
                    os.remove(tmp_path)
                except OSError:
                    pass

    def select_program(self, program_name: str, remote_path: str = "") -> str:
        """Select/load a program on the KUKA controller via C3 Bridge."""
        self._ensure()
        try:
            src_name = program_name if program_name.endswith(".src") else f"{program_name}.src"
            if remote_path and not src_name.startswith("KRC:"):
                remote_full = remote_path.rstrip("\\/ ") + "\\" + src_name
            else:
                remote_full = src_name
            with self._lock:
                err = self._robot.select_program(remote_full)
            return err
        except Exception as e:
            return str(e)

    # ── Dynamic Sync helpers (MQ_ variable protocol) ─────────────────

    def write_var(self, name: str, value) -> str:
        """Thread-safe variable write via C3 Bridge. Returns '' or error."""
        self._ensure()
        try:
            with self._lock:
                self._robot.write(name, str(value))
            return ""
        except Exception as e:
            return str(e)

    def read_var(self, name: str) -> str:
        """Thread-safe variable read via C3 Bridge."""
        self._ensure()
        with self._lock:
            return self._robot.read(name)

    @staticmethod
    def _format_e6pos(pos_m, euler_rad) -> str:
        """Blender canonical (m, XYZ Euler rad) -> KUKA E6POS string.
        Blender XYZ = (C, B, A) in KUKA terms; reverse to KUKA (A, B, C)."""
        x_mm = float(pos_m[0]) * 1000.0
        y_mm = float(pos_m[1]) * 1000.0
        z_mm = float(pos_m[2]) * 1000.0
        # euler_rad is Blender XYZ = (C_rad, B_rad, A_rad)
        a_deg = rad2deg(float(euler_rad[2]))
        b_deg = rad2deg(float(euler_rad[1]))
        c_deg = rad2deg(float(euler_rad[0]))
        return (
            f"{{X {x_mm:.3f}, Y {y_mm:.3f}, Z {z_mm:.3f}, "
            f"A {a_deg:.3f}, B {b_deg:.3f}, C {c_deg:.3f}}}"
        )

    @staticmethod
    def _format_e6axis(joints_deg) -> str:
        """Joint angles (deg) -> KUKA E6AXIS string."""
        vals = [float(joints_deg[i]) for i in range(6)]
        return (
            f"{{A1 {vals[0]:.3f}, A2 {vals[1]:.3f}, A3 {vals[2]:.3f}, "
            f"A4 {vals[3]:.3f}, A5 {vals[4]:.3f}, A6 {vals[5]:.3f}}}"
        )

    @staticmethod
    def _ptp_speed_percent(vel: float) -> float:
        """Clamp PTP speed to KUKA percent range (0.1–100)."""
        return max(0.1, min(100.0, float(vel)))

    def _next_cmd_id(self) -> int:
        self._cmd_id += 1
        return self._cmd_id

    def _wait_cmd_done(self, cmd_id: int, timeout: float = 30.0) -> str:
        """Poll MQ_DONE_ID until it matches cmd_id. Returns '' or timeout error."""
        import time
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                done = int(self.read_var("MQ_DONE_ID"))
                if done >= cmd_id:
                    return ""
            except Exception:
                pass
            time.sleep(0.05)
        return f"Timeout waiting for robot (MQ_DONE_ID != {cmd_id})"

    # ── Dynamic Sync move commands ─────────────────────────────────

    def move_to_pose(self, pos_m, euler_rad, speed: float, acc: float, radius: float, wait: bool) -> str:
        """LIN move via Dynamic Sync: write MQ_E6POS, speed, then MQ_ACTION=2."""
        self._ensure()
        try:
            e6pos = self._format_e6pos(pos_m, euler_rad)
            err = self.write_var("MQ_E6POS", e6pos)
            if err:
                return err
            lin_vel = max(0.001, min(2.0, speed))
            err = self.write_var("MQ_LIN_VEL", f"{lin_vel:.4f}")
            if err:
                return err
            # Single LIN command cannot be approximated reliably (no guaranteed next block in advance run).
            apo = -1
            err = self.write_var("MQ_APO", f"{apo:.1f}")
            if err:
                return err
            cmd_id = self._next_cmd_id()
            err = self.write_var("MQ_CMD_ID", str(cmd_id))
            if err:
                return err
            err = self.write_var("MQ_ACTION", "2")
            if err:
                return err
            if wait:
                return self._wait_cmd_done(cmd_id)
            return ""
        except Exception as e:
            return str(e)

    def move_to_pose_ptp(self, pos_m, euler_rad, vel: float, acc: float, radius: float, wait: bool) -> str:
        """PTP move via Dynamic Sync: write MQ_E6POS, speed(%), then MQ_ACTION=3."""
        self._ensure()
        try:
            e6pos = self._format_e6pos(pos_m, euler_rad)
            err = self.write_var("MQ_E6POS", e6pos)
            if err:
                return err
            ptp_speed = self._ptp_speed_percent(vel)
            err = self.write_var("MQ_SPEED", f"{ptp_speed:.1f}")
            if err:
                return err
            # Single PTP command cannot be approximated reliably (no guaranteed next block in advance run).
            apo = -1
            err = self.write_var("MQ_APO", f"{apo:.1f}")
            if err:
                return err
            cmd_id = self._next_cmd_id()
            err = self.write_var("MQ_CMD_ID", str(cmd_id))
            if err:
                return err
            err = self.write_var("MQ_ACTION", "3")
            if err:
                return err
            if wait:
                return self._wait_cmd_done(cmd_id)
            return ""
        except Exception as e:
            return str(e)

    def execute_ptp_path(self, waypoints: list, speed: float, acc: float, radius: float, wait: bool,
                         ring_buffer_size: int = 32) -> str:
        """Stream path via Dynamic Sync ring buffer (MQ_PT[]).
        Fills buffer in batches, sets MQ_ACTION=10 (LIN queue), monitors
        MQ_RD_IDX and refills ahead of the robot."""
        import time

        self._ensure()
        if not waypoints:
            return "No waypoints"
        try:
            total = len(waypoints)
            state = self.stream_start(total, speed, radius,
                                      waypoints[:ring_buffer_size], ring_buffer_size)
            if isinstance(state, str):
                return state  # error

            # Continue filling buffer as robot consumes points
            while state["written"] < total:
                try:
                    rd_idx = int(self.read_var("MQ_RD_IDX"))
                except Exception:
                    time.sleep(0.02)
                    continue

                remaining = waypoints[state["written"]:]
                refilled, err = self.stream_refill(state, rd_idx, remaining)
                if err:
                    return err
                if not refilled:
                    time.sleep(0.02)

            if wait:
                return self._wait_cmd_done(state["cmd_id"], timeout=max(30.0, total * 0.5))
            return ""
        except Exception as e:
            return str(e)

    # ── Streaming helpers (for modal / live-update path) ──────────

    def stream_start(self, total_points: int, speed: float, radius: float,
                     initial_waypoints: list, ring_buffer_size: int = 32):
        """Initialize a streaming path on the ring buffer.
        Returns a state dict on success, or an error string."""
        self._ensure()
        try:
            lin_vel = max(0.001, min(2.0, speed))
            apo = radius * 1000.0 if radius > 0 else -1

            err = self.write_var("MQ_LIN_VEL", f"{lin_vel:.4f}")
            if err:
                return err
            err = self.write_var("MQ_APO", f"{apo:.1f}")
            if err:
                return err
            err = self.write_var("MQ_PT_CNT", str(total_points))
            if err:
                return err
            err = self.write_var("MQ_RD_IDX", "1")
            if err:
                return err

            written = 0
            for i, wp in enumerate(initial_waypoints):
                pos_m, euler_rad = wp
                e6pos = self._format_e6pos(pos_m, euler_rad)
                buf_idx = (i % ring_buffer_size) + 1
                err = self.write_var(f"MQ_PT[{buf_idx}]", e6pos)
                if err:
                    return err
                written += 1

            err = self.write_var("MQ_WR_IDX", str(written + 1))
            if err:
                return err

            cmd_id = self._next_cmd_id()
            err = self.write_var("MQ_CMD_ID", str(cmd_id))
            if err:
                return err
            err = self.write_var("MQ_ACTION", "10")
            if err:
                return err

            return {
                "cmd_id": cmd_id,
                "written": written,
                "total": total_points,
                "ring_size": ring_buffer_size,
            }
        except Exception as e:
            return str(e)

    def stream_refill(self, state: dict, rd_idx: int, waypoints: list):
        """Fill free ring buffer slots from waypoints list.
        waypoints[0] corresponds to state['written'].
        Returns (count_written, error_or_empty)."""
        ring_size = state["ring_size"]
        wr_pos = state["written"] + 1
        used = max(0, wr_pos - rd_idx)
        free = max(0, ring_size - used)
        if free <= 0:
            return 0, ""
        to_write = min(free, len(waypoints))
        try:
            for i in range(to_write):
                pos_m, euler_rad = waypoints[i]
                e6pos = self._format_e6pos(pos_m, euler_rad)
                buf_idx = (state["written"] % ring_size) + 1
                err = self.write_var(f"MQ_PT[{buf_idx}]", e6pos)
                if err:
                    return 0, err
                state["written"] += 1
            err = self.write_var("MQ_WR_IDX", str(state["written"] + 1))
            if err:
                return 0, err
            return to_write, ""
        except Exception as e:
            return 0, str(e)

    def stream_is_done(self, state: dict) -> bool:
        """Check if the robot finished executing all queued points."""
        try:
            done_id = int(self.read_var("MQ_DONE_ID"))
            return done_id >= state["cmd_id"]
        except Exception:
            return False

    # ── Dynamic Sync program management ────────────────────────────

    def upload_stream_program(self, remote_path: str = "", base_no: int = 0, tool_no: int = 0,
                              ring_buffer_size: int = 32, advance: int = None) -> str:
        """Generate mq_stream.src/.dat and upload both to the robot."""
        from ..runtime import krl_stream
        if advance is None:
            advance = krl_stream.ADVANCE_DEFAULT
        src_content = krl_stream.generate_src(base_no=base_no, tool_no=tool_no,
                                              ring_buffer_size=ring_buffer_size,
                                              advance=advance)
        dat_content = krl_stream.generate_dat()

        err = self.upload_program(src_content, krl_stream.PROGRAM_NAME, remote_path)
        if err:
            return f"SRC upload failed: {err}"

        dat_name = f"{krl_stream.PROGRAM_NAME}.dat"
        err_dat = self.upload_program(dat_content, dat_name, remote_path)
        if err_dat:
            return f"DAT upload failed: {err_dat}"

        return ""

    def play_program(self) -> str:
        """Start the currently selected program on the KUKA controller via C3 Bridge."""
        self._ensure()
        try:
            with self._lock:
                err = self._robot.start_program()
            return err
        except Exception as e:
            return str(e)

    def stop_program(self) -> str:
        """Stop the running program on the KUKA controller via C3 Bridge."""
        self._ensure()
        try:
            with self._lock:
                err = self._robot.stop_program()
            return err
        except Exception as e:
            return str(e)

    def cancel_program(self) -> str:
        """Cancel the running program on the KUKA controller via C3 Bridge."""
        self._ensure()
        try:
            with self._lock:
                err = self._robot.cancel_program()
            return err
        except Exception as e:
            return str(e)

    def select_stream_program(self, remote_path: str = "") -> str:
        """Select mq_stream on the controller so user can start it."""
        from ..runtime import krl_stream
        return self.select_program(krl_stream.PROGRAM_NAME, remote_path)

    # ── Real-time Puppet Mode (tight PTP loop via MQ_ACTION=20) ──

    _rt_puppet_active = False

    def realtime_puppet_start(self, boundary_mm=None) -> str:
        """Enter puppet mode: write current TCP pose to MQ_E6POS and set MQ_ACTION=20.
        Assumes mq_stream is already running on the controller (main WHILE loop
        is idle at MQ_ACTION==0).  The robot-side runs a tight
        CONTINUE / PTP MQ_E6POS C_PTP loop.  Safe on connection loss because
        absolute PTP to the same frame = robot stays in place."""
        self._ensure()
        try:
            # Seed MQ_E6POS with current position so robot doesn't jump
            pos_m, euler_rad = self.read_tcp()
            e6pos = self._format_e6pos(pos_m, euler_rad)
            err = self.write_var("MQ_E6POS", e6pos)
            if err:
                return err
            err = self.write_var("MQ_ACTION", "20")
            if err:
                return err
            self._rt_puppet_active = True
            return ""
        except Exception as e:
            return str(e)

    def realtime_puppet_step(self, pos_m, euler_rad, speed: float, acc: float) -> str:
        """Stream one cartesian target. Single variable write — the robot-side
        tight loop picks it up on the next CONTINUE cycle."""
        self._ensure()
        if not self._rt_puppet_active:
            err = self.realtime_puppet_start()
            if err:
                return err
        try:
            e6pos = self._format_e6pos(pos_m, euler_rad)
            return self.write_var("MQ_E6POS", e6pos)
        except Exception as e:
            return str(e)

    def realtime_puppet_stop(self) -> str:
        """Exit puppet mode: set MQ_ACTION=0. The robot-side WHILE exits
        and returns to the main idle loop."""
        self._rt_puppet_active = False
        self._ensure()
        try:
            return self.write_var("MQ_ACTION", "0")
        except Exception as e:
            return str(e)
