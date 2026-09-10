# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
# Animaquina -- Newton validator bridge (persistent worker + one-shot fallback)

from __future__ import annotations

import atexit
import hashlib
import importlib
import json
import os
import queue
import subprocess
import sys
import threading
import time
from pathlib import Path

import bpy
from . import mujoco_export

_WORKER_TIMEOUT_S = 30
_WORKER_LINE_TIMEOUT_S = 30


def _try_export_mujoco_xml(slot) -> tuple[str | None, str | None]:
    """
    Attempt to export the robot rig to MuJoCo XML.
    Returns (xml_str, None) on success or (None, error_msg) on failure.
    Never raises -- the validator can proceed in stub mode if export fails.
    """
    try:
        return mujoco_export.export_slot_to_mujoco_xml(slot), None
    except Exception as exc:
        return None, str(exc)


def _worker_script_path() -> str:
    try:
        worker_module = importlib.import_module("animaquina_core.runtime.newton_worker")
        worker_file = getattr(worker_module, "__file__", "") or ""
        if worker_file:
            return str(Path(worker_file))
    except Exception:
        pass
    return str(Path(__file__).with_name("newton_worker.py"))


def _python_executable(slot) -> str:
    preferred = str(getattr(slot, "newton_python_exe", "") or "").strip()
    if preferred:
        return preferred

    # Default to Blender's embedded Python when available.
    py_path = getattr(getattr(bpy, "app", None), "binary_path_python", "") or ""
    if py_path:
        return py_path
    return sys.executable


def _parse_json_responses(text: str) -> tuple[list, list]:
    responses = []
    skipped = []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            responses.append(json.loads(line))
        except Exception:
            skipped.append(line)
    return responses, skipped


def _collect_collision_objects(slot):
    coll = getattr(slot, "collision_collection", None)
    if coll is None:
        return []
    out = []
    for obj in getattr(coll, "objects", []):
        try:
            out.append(
                {
                    "name": obj.name,
                    "type": obj.type,
                    "location": [float(v) for v in obj.location],
                    "rotation_euler": [float(v) for v in obj.rotation_euler],
                    "scale": [float(v) for v in obj.scale],
                }
            )
        except Exception:
            continue
    return out


def _build_validation_payload(slot, waypoints_base, euler_rad, options=None) -> dict:
    options = dict(options or {})
    return {
        "robot_type": getattr(slot, "robot_type", "UR"),
        "joint_axis_map": [getattr(slot, f"joint_axis_{i}", "Y") for i in range(6)],
        "current_joints_deg": [
            float(getattr(slot, "joints_deg")[i])
            for i in range(min(6, len(getattr(slot, "joints_deg", []))))
        ],
        "tool_orientation_rad": [float(v) for v in euler_rad],
        "waypoints": [
            {
                "position_m": [float(v) for v in pos_m],
                "euler_rad": [float(v) for v in eul],
            }
            for (pos_m, eul) in waypoints_base
        ],
        "collision_objects": _collect_collision_objects(slot),
        "options": options,
    }


def _build_model_cache_key(slot, mujoco_xml: str | None, phase: str = "phase1_validation") -> str:
    xml_hash = hashlib.sha256((mujoco_xml or "").encode("utf-8", "replace")).hexdigest()
    # Newton path playback uses the Newton-specific sim setup profile (no Blender IK constraints)
    # even if the UI mode is switched later.
    sim_profile = "NEWTON_IK"
    return "|".join(
        [
            str(getattr(slot, "uid", "") or ""),
            str(getattr(slot, "robot_type", "") or ""),
            sim_profile,
            phase,
            xml_hash,
        ]
    )


def build_validation_request(slot, waypoints_base, euler_rad, options=None) -> dict:
    """
    Build a plain-Python request object. Call on the main thread (reads Blender RNA and exports XML).
    Safe to pass to a background thread for execution.
    """
    validate_payload = _build_validation_payload(slot, waypoints_base, euler_rad, options=options)
    mujoco_xml, xml_error = _try_export_mujoco_xml(slot)
    init_payload = {"phase": "phase1_validation"}
    if mujoco_xml is not None:
        init_payload["mujoco_xml"] = mujoco_xml
    elif xml_error:
        init_payload["mujoco_xml_error"] = xml_error
    return {
        "slot_uid": str(getattr(slot, "uid", "") or ""),
        "python_exe": _python_executable(slot),
        "worker_script": _worker_script_path(),
        "init_payload": init_payload,
        "validate_payload": validate_payload,
        "model_cache_key": _build_model_cache_key(slot, mujoco_xml, phase=str(init_payload.get("phase") or "phase1_validation")),
    }


def _startup_kwargs():
    startupinfo = None
    creationflags = 0
    if os.name == "nt":
        startupinfo = subprocess.STARTUPINFO()
        startupinfo.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    return startupinfo, creationflags


class _PersistentWorker:
    def __init__(self, python_exe: str, worker_script: str):
        self.python_exe = python_exe
        self.worker_script = worker_script
        self.proc: subprocess.Popen | None = None
        self._stdout_q: queue.Queue | None = None
        self._stderr_lines: list[str] = []
        self._stderr_lock = threading.Lock()
        self._stdout_noise: list[str] = []
        self._io_lock = threading.Lock()
        self._backend_info: dict | None = None
        self._init_model_cache_key: str | None = None

    def _spawn(self):
        startupinfo, creationflags = _startup_kwargs()
        self.proc = subprocess.Popen(
            [self.python_exe, self.worker_script, "--stdio"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="replace",
            startupinfo=startupinfo,
            creationflags=creationflags,
        )
        self._stdout_q = queue.Queue()
        self._stderr_lines = []
        self._stdout_noise = []
        self._backend_info = None
        self._init_model_cache_key = None

        def _pump_stdout():
            try:
                for line in self.proc.stdout:
                    self._stdout_q.put(line)
            except Exception:
                pass
            finally:
                self._stdout_q.put(None)

        def _pump_stderr():
            try:
                for line in self.proc.stderr:
                    with self._stderr_lock:
                        self._stderr_lines.append(line.rstrip("\n"))
                        if len(self._stderr_lines) > 2000:
                            self._stderr_lines = self._stderr_lines[-2000:]
            except Exception:
                pass

        threading.Thread(target=_pump_stdout, daemon=True).start()
        threading.Thread(target=_pump_stderr, daemon=True).start()

    def _ensure_alive(self):
        if self.proc is None or self.proc.poll() is not None:
            self.close()
            self._spawn()

    def _stderr_text(self) -> str:
        with self._stderr_lock:
            return "\n".join(self._stderr_lines).strip()

    def _read_json_response(self, timeout_s: float = _WORKER_LINE_TIMEOUT_S) -> dict:
        if self._stdout_q is None:
            raise RuntimeError("Newton worker stdout queue not initialized")
        deadline = time.time() + float(timeout_s)
        while True:
            remaining = max(0.0, deadline - time.time())
            if remaining <= 0.0:
                raise TimeoutError(f"Newton worker response timeout (> {timeout_s}s)")
            try:
                line = self._stdout_q.get(timeout=remaining)
            except queue.Empty:
                raise TimeoutError(f"Newton worker response timeout (> {timeout_s}s)")
            if line is None:
                raise RuntimeError("Newton worker stdout closed unexpectedly")
            line = str(line).strip()
            if not line:
                continue
            try:
                return json.loads(line)
            except Exception:
                self._stdout_noise.append(line)
                if len(self._stdout_noise) > 100:
                    self._stdout_noise = self._stdout_noise[-100:]

    def _send_cmd(self, cmd: str, payload: dict):
        if self.proc is None or self.proc.stdin is None:
            raise RuntimeError("Newton worker is not running")
        self.proc.stdin.write(json.dumps({"cmd": cmd, "payload": payload}) + "\n")
        self.proc.stdin.flush()

    def validate(self, *, init_payload: dict, validate_payload: dict, model_cache_key: str | None = None) -> dict:
        with self._io_lock:
            self._ensure_alive()
            if self._backend_info is None or self._init_model_cache_key != model_cache_key:
                self._send_cmd("init", init_payload)
                init_resp = self._read_json_response()
                if init_resp.get("type") != "ok":
                    raise RuntimeError(init_resp.get("message") or "Newton worker init failed")
                self._backend_info = dict(init_resp.get("backend_info") or {})
                self._init_model_cache_key = model_cache_key

            backend_info = dict(self._backend_info or {})
            self._send_cmd("validate_path", validate_payload)
            result = self._read_json_response()
            if result.get("type") != "validation_report":
                raise RuntimeError(result.get("message") or "Newton worker returned unexpected response")
            report = dict(result.get("report") or {})
            if backend_info and not report.get("backend_info"):
                report["backend_info"] = backend_info
            if backend_info and not report.get("backend"):
                report["backend"] = backend_info.get("backend") or "worker"
            stderr_text = self._stderr_text()
            if stderr_text and not report.get("worker_stderr"):
                report["worker_stderr"] = stderr_text
            if self._stdout_noise and not report.get("worker_stdout_noise"):
                report["worker_stdout_noise"] = list(self._stdout_noise[-50:])
            return report

    def close(self):
        proc = self.proc
        self.proc = None
        self._backend_info = None
        self._init_model_cache_key = None
        if proc is None:
            return
        try:
            if proc.stdin:
                proc.stdin.write(json.dumps({"cmd": "shutdown", "payload": {}}) + "\n")
                proc.stdin.flush()
        except Exception:
            pass
        try:
            proc.terminate()
        except Exception:
            pass
        try:
            proc.wait(timeout=1.5)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass
            try:
                proc.wait(timeout=1.0)
            except Exception:
                pass


_PERSISTENT_WORKERS: dict[str, _PersistentWorker] = {}
_PERSISTENT_WORKERS_LOCK = threading.Lock()


def _persistent_worker_key(request: dict) -> str:
    return "|".join(
        [
            str(request.get("slot_uid") or ""),
            str(request.get("python_exe") or ""),
            str(request.get("worker_script") or ""),
        ]
    )


def _get_persistent_worker(request: dict) -> _PersistentWorker:
    key = _persistent_worker_key(request)
    with _PERSISTENT_WORKERS_LOCK:
        worker = _PERSISTENT_WORKERS.get(key)
        if worker is None:
            worker = _PersistentWorker(
                python_exe=str(request.get("python_exe") or sys.executable),
                worker_script=str(request.get("worker_script") or _worker_script_path()),
            )
            _PERSISTENT_WORKERS[key] = worker
        return worker


def shutdown_persistent_workers():
    with _PERSISTENT_WORKERS_LOCK:
        items = list(_PERSISTENT_WORKERS.values())
        _PERSISTENT_WORKERS.clear()
    for worker in items:
        try:
            worker.close()
        except Exception:
            pass


atexit.register(shutdown_persistent_workers)


def _validate_prebuilt_request_one_shot(request: dict):
    """
    One-shot validation request (legacy path). Used as fallback or when persistence is disabled.
    """
    python_exe = str(request.get("python_exe") or sys.executable)
    worker_script = str(request.get("worker_script") or _worker_script_path())
    cmd = [python_exe, worker_script, "--stdio"]

    startupinfo, creationflags = _startup_kwargs()

    init_payload = dict(request.get("init_payload") or {})
    payload = dict(request.get("validate_payload") or {})
    input_text = (
        json.dumps({"cmd": "init", "payload": init_payload}) + "\n"
        + json.dumps({"cmd": "validate_path", "payload": payload}) + "\n"
        + json.dumps({"cmd": "shutdown", "payload": {}}) + "\n"
    )

    try:
        proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="replace",
            startupinfo=startupinfo,
            creationflags=creationflags,
        )
    except FileNotFoundError as exc:
        raise RuntimeError(
            f"Python executable not found for Newton worker: {python_exe!r}. "
            "Could not resolve Blender Python executable."
        ) from exc

    try:
        try:
            stdout_text, stderr_text = proc.communicate(input=input_text, timeout=_WORKER_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
            raise RuntimeError(
                f"Newton worker timed out (> {_WORKER_TIMEOUT_S} s). "
                "Check that the worker env is valid or increase _WORKER_TIMEOUT_S."
            )

        responses, noise = _parse_json_responses(stdout_text)
        if len(responses) < 2:
            snippet = (stderr_text or "")[:400]
            raise RuntimeError(
                f"Newton worker returned {len(responses)} response(s), expected at least 2. "
                f"Worker stderr: {snippet!r}"
            )

        init_resp = responses[0]
        if init_resp.get("type") != "ok":
            raise RuntimeError(init_resp.get("message") or "Newton worker init failed")
        backend_info = init_resp.get("backend_info") or {}

        result = responses[1]
        if result.get("type") != "validation_report":
            raise RuntimeError(result.get("message") or "Newton worker returned unexpected response")

        report = result.get("report") or {}
        if backend_info and not report.get("backend_info"):
            report["backend_info"] = backend_info
        if backend_info and not report.get("backend"):
            report["backend"] = backend_info.get("backend") or "worker"
        stderr_text = (stderr_text or "").strip()
        if stderr_text and not report.get("worker_stderr"):
            report["worker_stderr"] = stderr_text
        if noise and not report.get("worker_stdout_noise"):
            report["worker_stdout_noise"] = noise
        return report
    finally:
        try:
            proc.kill()
        except Exception:
            pass
        try:
            proc.wait(timeout=2.0)
        except Exception:
            pass


def validate_prebuilt_request(request: dict, use_persistent: bool = True):
    """
    Execute a validation request created by build_validation_request().
    Safe to call from background threads.
    """
    if not use_persistent:
        return _validate_prebuilt_request_one_shot(request)

    worker = _get_persistent_worker(request)
    try:
        return worker.validate(
            init_payload=dict(request.get("init_payload") or {}),
            validate_payload=dict(request.get("validate_payload") or {}),
            model_cache_key=str(request.get("model_cache_key") or ""),
        )
    except Exception:
        # One retry after worker restart (handles crashes/closed pipes).
        worker.close()
        return worker.validate(
            init_payload=dict(request.get("init_payload") or {}),
            validate_payload=dict(request.get("validate_payload") or {}),
            model_cache_key=str(request.get("model_cache_key") or ""),
        )


def validate_toolpath(slot, waypoints_base, euler_rad, options=None):
    request = build_validation_request(slot, waypoints_base, euler_rad, options=options)
    return validate_prebuilt_request(request, use_persistent=True)


def probe_worker(slot):
    """
    Run only the worker init/shutdown handshake and return backend info + stderr.
    Uses one-shot communicate(); probe should not affect persistent worker state.
    """
    python_exe = _python_executable(slot)
    cmd = [python_exe, _worker_script_path(), "--stdio"]
    startupinfo, creationflags = _startup_kwargs()

    input_text = (
        json.dumps({"cmd": "init", "payload": {"phase": "probe"}}) + "\n"
        + json.dumps({"cmd": "shutdown", "payload": {}}) + "\n"
    )

    try:
        proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
            errors="replace",
            startupinfo=startupinfo,
            creationflags=creationflags,
        )
    except FileNotFoundError as exc:
        raise RuntimeError(f"Python executable not found for Newton worker: {python_exe!r}") from exc

    try:
        try:
            stdout_text, stderr_text = proc.communicate(input=input_text, timeout=_WORKER_TIMEOUT_S)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
            raise RuntimeError(f"Newton worker probe timed out (> {_WORKER_TIMEOUT_S} s)")

        responses, noise = _parse_json_responses(stdout_text)
        if not responses:
            raise RuntimeError(f"No JSON response from Newton worker. stderr={((stderr_text or '')[:400])!r}")
        init_resp = responses[0]
        if init_resp.get("type") != "ok":
            raise RuntimeError(init_resp.get("message") or "Newton worker init failed during probe")
        backend_info = dict(init_resp.get("backend_info") or {})
        backend_info.setdefault("python_executable", python_exe)
        stderr_text = (stderr_text or "").strip()
        if stderr_text:
            backend_info["worker_stderr"] = stderr_text
        if noise:
            backend_info["worker_stdout_noise"] = noise
        return backend_info
    finally:
        try:
            proc.kill()
        except Exception:
            pass
        try:
            proc.wait(timeout=2.0)
        except Exception:
            pass
