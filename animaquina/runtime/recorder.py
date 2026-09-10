# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
# Animaquina — dataset recorder: timestamped (measured, commanded) samples.
#
# Threading model (mirrors manager.py): samples are appended by the background
# poll thread, so the timestamp is the moment of the driver read rather than the
# Blender timer tick — timer jitter and back-off never end up in the data. The
# commanded target lives in Blender data and may only be touched on the main
# thread, so the poll timer publishes it here (set_command) and the poll thread
# attaches the most recent value to each sample.
#
# This module stays free of bpy: it collects rows and serialises CSV. Writing
# the Text datablock / file is the operator's job.

import math
import threading
import time
from datetime import datetime, timezone

# Ceiling on what a single session may collect. A forgotten recording at 25 Hz
# would otherwise grow a Text datablock until the .blend is unopenable, so a
# session auto-stops here instead of taking the file down with it.
MAX_SAMPLES_HARD = 500000

BASE_COLUMNS = (
    "t_s", "dt_s",
    "j0_deg", "j1_deg", "j2_deg", "j3_deg", "j4_deg", "j5_deg",
    "tcp_x_m", "tcp_y_m", "tcp_z_m", "tcp_a_deg", "tcp_b_deg", "tcp_c_deg",
    "cmd_x_m", "cmd_y_m", "cmd_z_m", "cmd_a_deg", "cmd_b_deg", "cmd_c_deg",
    "cmd_valid",
    "run_idx", "run_source",
)


def var_column(var_name: str) -> str:
    """Column name for a polled controller variable ($OV_PRO -> var_OV_PRO)."""
    cleaned = "".join(c if (c.isalnum() or c == "_") else "_" for c in str(var_name or "").strip())
    return f"var_{cleaned.strip('_') or 'unnamed'}"


_RAD2DEG = 180.0 / math.pi
_MUST_QUOTE = ',"\n\r'   # chars that force CSV quoting

_sessions = {}          # uid → Session (active or finished, kept until replaced)
_sessions_lock = threading.Lock()


class Session:
    """One recording. Appended to from the poll thread, read from the main thread."""

    def __init__(self, uid: str, name: str, meta: dict, max_samples: int, var_names=()):
        self.uid = uid
        self.name = name
        self.meta = dict(meta or {})
        # Frozen at start: the CSV schema cannot change halfway through a file,
        # so variables added to the Variables panel mid-recording are ignored
        # until the next take.
        self.var_names = tuple(str(v) for v in (var_names or ()))
        self.max_samples = max(1, min(int(max_samples), MAX_SAMPLES_HARD))
        self.started_utc = datetime.now(timezone.utc).replace(microsecond=0).isoformat()
        self.active = True
        self.overflow = False
        self.text_name = ""
        self._rows = []
        self._command = None          # ((x, y, z), (a, b, c)) in rad, robot base frame
        self._run = (-1, "NONE")      # canonical run_idx contract, published by the timer
        self._t0 = time.perf_counter()
        self._t_last = None
        self._lock = threading.Lock()

    def columns(self) -> tuple:
        return BASE_COLUMNS + tuple(var_column(v) for v in self.var_names)

    # -- main thread -------------------------------------------------------
    def set_context(self, pos_m, euler_rad, run_idx=-1, run_source="NONE") -> None:
        """Publish the main-thread-only values the next sample should carry:
        the commanded target pose and the canonical run state."""
        if pos_m is None or euler_rad is None:
            command = None
        else:
            command = (
                (float(pos_m[0]), float(pos_m[1]), float(pos_m[2])),
                (float(euler_rad[0]), float(euler_rad[1]), float(euler_rad[2])),
            )
        run = (int(run_idx), str(run_source or "NONE"))
        with self._lock:
            self._command = command
            self._run = run

    def count(self) -> int:
        with self._lock:
            return len(self._rows)

    def elapsed_s(self) -> float:
        with self._lock:
            return float(self._t_last or 0.0)

    def rows(self) -> list:
        with self._lock:
            return list(self._rows)

    def stop(self) -> None:
        with self._lock:
            self.active = False

    # -- poll thread -------------------------------------------------------
    def add(self, joints_deg, tcp_pos_m, tcp_euler_rad, debug_values=None) -> None:
        """Append one sample. Silently ignored once the session is stopped or full."""
        t = time.perf_counter() - self._t0
        with self._lock:
            if not self.active:
                return
            if len(self._rows) >= self.max_samples:
                self.active = False
                self.overflow = True
                return
            dt = 0.0 if self._t_last is None else (t - self._t_last)
            self._t_last = t
            command = self._command
            run_idx, run_source = self._run

            row = [t, dt]
            for j in range(6):
                row.append(_num(joints_deg, j))
            for i in range(3):
                row.append(_num(tcp_pos_m, i))
            for i in range(3):
                row.append(_deg(tcp_euler_rad, i))
            if command is None:
                row.extend([None] * 6)
                row.append(0)
            else:
                cmd_pos, cmd_euler = command
                row.extend(cmd_pos)
                row.extend(v * _RAD2DEG for v in cmd_euler)
                row.append(1)
            row.append(run_idx)
            row.append(run_source)
            # Polled controller variables come straight off the poll cycle that
            # produced this sample, so they stay in step with the pose - unlike
            # the command, which the main thread can only publish between ticks.
            values = debug_values or {}
            for var_name in self.var_names:
                row.append(values.get(var_name, ""))
            self._rows.append(tuple(row))


def _num(seq, i):
    try:
        return float(seq[i])
    except (TypeError, IndexError, ValueError):
        return None


def _deg(seq, i):
    v = _num(seq, i)
    return None if v is None else v * _RAD2DEG


# Module-level session registry

def start(uid: str, name: str, meta: dict, max_samples: int, var_names=()) -> Session:
    """Begin a recording for this slot, replacing any previous session."""
    session = Session(uid, name, meta, max_samples, var_names)
    with _sessions_lock:
        _sessions[uid] = session
    return session


def stop(uid: str):
    """Stop recording. The session is kept so it can still be written out."""
    session = get(uid)
    if session is not None:
        session.stop()
    return session


def get(uid: str):
    with _sessions_lock:
        return _sessions.get(uid)


def discard(uid: str) -> None:
    with _sessions_lock:
        _sessions.pop(uid, None)


def is_active(uid: str) -> bool:
    session = get(uid)
    return session is not None and session.active


def record(uid: str, joints_deg, tcp_pos_m, tcp_euler_rad, debug_values=None) -> None:
    """Called from the poll thread once per successful read cycle."""
    session = get(uid)
    if session is not None and session.active:
        session.add(joints_deg, tcp_pos_m, tcp_euler_rad, debug_values)


def set_context(uid: str, pos_m, euler_rad, run_idx=-1, run_source="NONE") -> None:
    """Called from the main thread with the commanded pose and run state."""
    session = get(uid)
    if session is not None and session.active:
        session.set_context(pos_m, euler_rad, run_idx, run_source)


# Serialisation

def _fmt(v) -> str:
    if v is None:
        return ""
    if isinstance(v, str):
        return _quote(v)
    if isinstance(v, int):
        return str(v)
    if v != v:  # NaN
        return ""
    return f"{v:.6f}"


def _quote(text: str) -> str:
    """CSV-quote a controller variable value.

    read_var returns whatever the controller prints, and a KUKA frame comes
    back as '{X 1.0, Y 2.0, ...}' - unquoted, those commas would silently shift
    every column to its right.
    """
    if any(c in text for c in _MUST_QUOTE):
        return '"' + text.replace('"', '""') + '"'
    return text


def header_lines(session: Session, rows=None) -> list:
    """Comment block describing the dataset.

    Units and frame belong in the file, not in someone's memory — without them
    the numbers are unusable a few months from now. '#' keeps them skippable
    (pandas: read_csv(path, comment='#')).

    `rows` is the already-taken row snapshot, so the reported sample count can
    never disagree with the rows that follow it when a session is written out
    while it is still recording.
    """
    count = session.count() if rows is None else len(rows)
    duration = session.elapsed_s() if rows is None else (rows[-1][0] if rows else 0.0)
    lines = [
        "# Animaquina dataset",
        f"# name: {session.name}",
        f"# started_utc: {session.started_utc}",
        f"# samples: {count}",
        f"# duration_s: {duration:.3f}",
    ]
    for key, value in session.meta.items():
        lines.append(f"# {key}: {value}")
    lines.append("# units: position m, angles deg, time s")
    lines.append("# frame: robot base (J0); measured TCP and commanded target are both in this frame")
    lines.append("# cmd_valid: 1 when a target object pose was available for that sample, else 0")
    lines.append("# run_idx: waypoint the run is at (-1 idle); run_source: NONE|SIM|STREAM|PROGRAM")
    if session.var_names:
        lines.append("# polled_variables: " + ", ".join(
            f"{var_column(v)}={v}" for v in session.var_names))
    if session.overflow:
        lines.append(f"# NOTE: recording auto-stopped at the {session.max_samples} sample cap")
    return lines


def to_csv(session: Session) -> str:
    """Full CSV text: comment header, column names, one row per sample."""
    rows = session.rows()
    out = header_lines(session, rows)
    out.append(",".join(session.columns()))
    for row in rows:
        out.append(",".join(_fmt(v) for v in row))
    return "\n".join(out) + "\n"
