# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Animaquina Core -- Newton worker (Phase 1 stub protocol over stdin/stdout)

from __future__ import annotations

import argparse
import contextlib
import json
import math
import sys
import time

try:
    import numpy as _np
except Exception:
    _np = None


def _send(obj):
    sys.stdout.write(json.dumps(obj) + "\n")
    sys.stdout.flush()


def _detect_backend():
    """
    Probe runtime availability. Phase 1 still uses the stub validator implementation,
    but this reports whether Newton/MuJoCo imports appear available in the worker env.
    """
    info = {
        "backend": "stub",
        "mode": "stub",
        "newton_available": False,
        "mujoco_available": False,
        "python_executable": sys.executable,
        "message": "Newton not imported; running stub validator",
    }
    try:
        with contextlib.redirect_stdout(sys.stderr):
            import newton  # noqa: F401

        info["newton_available"] = True
    except Exception as exc:
        info["message"] = f"Newton import unavailable in worker env ({exc.__class__.__name__}: {exc})"
        return info

    try:
        with contextlib.redirect_stdout(sys.stderr):
            import mujoco  # noqa: F401

        info["mujoco_available"] = True
    except Exception as exc:
        info["message"] = f"Newton available, MuJoCo import unavailable ({exc.__class__.__name__}: {exc}); using stub validator"
        return info

    info["backend"] = "newton_stub"
    info["mode"] = "stub"
    info["message"] = "Newton+MuJoCo imports available; validator still using stub implementation"
    return info


def _stub_validate_path(payload):
    """
    Phase 1 stub behavior so Blender-side validation flow can be tested before
    Newton/MuJoCo is installed. Replace internals with real Newton calls later.
    """
    t0 = time.perf_counter()
    waypoints = payload.get("waypoints") or []
    collision_objects = payload.get("collision_objects") or []

    if not waypoints:
        return {
            "is_valid": False,
            "first_failure_index": None,
            "failure_kind": "solver_failure",
            "message": "No waypoints received",
            "contacts": [],
            "stats": {"elapsed_ms": 0.0, "waypoints_checked": 0, "substeps": 0},
            "backend": "stub",
        }

    first_failure = None
    failure_kind = None
    message = "Path validated (stub worker)"
    contacts = []

    for idx, wp in enumerate(waypoints):
        pos = wp.get("position_m") or [0.0, 0.0, 0.0]
        if any((not isinstance(v, (int, float))) or not math.isfinite(float(v)) for v in pos):
            first_failure = idx
            failure_kind = "solver_failure"
            message = "Non-finite waypoint data"
            break

        x, y, z = (float(pos[0]), float(pos[1]), float(pos[2]))
        reach = math.sqrt(x * x + y * y + z * z)

        # Simple fake checks for end-to-end testing:
        if z < 0.0:
            first_failure = idx
            failure_kind = "collision"
            message = "Stub collision: waypoint below Z=0 floor"
            contacts = [{"name": "floor", "point": [x, y, z]}]
            break
        if reach > 2.0:
            first_failure = idx
            failure_kind = "joint_limit"
            message = "Stub reach limit exceeded (> 2.0 m from base)"
            break

    if first_failure is not None and failure_kind == "collision" and collision_objects:
        # Mention first obstacle to prove payload plumbing.
        message += f" (collision collection loaded: {collision_objects[0].get('name', 'obstacle')})"

    elapsed_ms = (time.perf_counter() - t0) * 1000.0
    return {
        "is_valid": first_failure is None,
        "first_failure_index": first_failure,
        "failure_kind": failure_kind,
        "message": message,
        "contacts": contacts,
        "stats": {
            "elapsed_ms": round(elapsed_ms, 3),
            "waypoints_checked": len(waypoints) if first_failure is None else first_failure + 1,
            "substeps": int((payload.get("options") or {}).get("validation_substeps", 1)),
        },
        "backend": "stub",
    }


def _log_model_info(payload: dict) -> None:
    """
    Log MuJoCo model XML info to stderr for Phase 1A inspection.
    Phase 1B will replace this with mujoco.MjModel.from_xml_string(xml).
    """
    xml = payload.get("mujoco_xml")
    xml_error = payload.get("mujoco_xml_error")
    if xml:
        print(
            f"[newton_worker] mujoco_xml received: {len(xml)} chars, "
            f"{xml.count(chr(10))+1} lines",
            file=sys.stderr,
        )
        # Phase 1A helper: print first 400 chars so you can verify the structure.
        print(f"[newton_worker] mujoco_xml preview:\n{xml[:400]}", file=sys.stderr)
    elif xml_error:
        print(f"[newton_worker] mujoco_xml export failed: {xml_error}", file=sys.stderr)
    else:
        print("[newton_worker] no mujoco_xml in init payload (stub mode only)", file=sys.stderr)


def _compile_mujoco_session(mujoco_xml: str):
    """Compile the exported MuJoCo XML and cache joint/body metadata."""
    if _np is None:
        return None, "numpy not available in worker environment"
    try:
        with contextlib.redirect_stdout(sys.stderr):
            import mujoco
    except Exception as exc:
        return None, f"mujoco import failed: {exc.__class__.__name__}: {exc}"

    try:
        model = mujoco.MjModel.from_xml_string(mujoco_xml)
        data = mujoco.MjData(model)
    except Exception as exc:
        return None, f"MuJoCo XML compile failed: {exc.__class__.__name__}: {exc}"

    joint_ids = []
    qpos_ids = []
    jnt_ranges = []
    for i in range(1, 7):
        jname = f"joint_{i}"
        j_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname))
        if j_id < 0:
            break
        if int(model.jnt_type[j_id]) != int(mujoco.mjtJoint.mjJNT_HINGE):
            return None, f"{jname} is not a hinge joint"
        qpos_ids.append(int(model.jnt_qposadr[j_id]))
        lo, hi = float(model.jnt_range[j_id][0]), float(model.jnt_range[j_id][1])
        jnt_ranges.append((lo, hi))
        joint_ids.append(j_id)
    if not joint_ids:
        return None, "No joint_1..joint_6 joints found in MuJoCo XML"

    ee_name = f"link_{len(joint_ids)}"
    ee_body_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, ee_name))
    if ee_body_id < 0:
        return None, f"End effector body {ee_name} not found"

    # Use tcp_site if present (flange or tool tip); otherwise fall back to
    # link_N body origin (joint pivot).
    tcp_site_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp_site"))
    ee_use_site = tcp_site_id >= 0

    mujoco.mj_forward(model, data)
    return {
        "mujoco": mujoco,
        "model": model,
        "data": data,
        "joint_ids": joint_ids,
        "qpos_ids": qpos_ids,
        "jnt_ranges": jnt_ranges,
        "ee_body_id": ee_body_id,
        "ee_site_id": tcp_site_id,
        "ee_use_site": ee_use_site,
    }, None


def _geom_body_name(mj, model, geom_id: int) -> str:
    try:
        body_id = int(model.geom_bodyid[geom_id])
    except Exception:
        return ""
    return mj.mj_id2name(model, mj.mjtObj.mjOBJ_BODY, body_id) or f"body_{body_id}"


def _contact_summary(mj, model, data, max_items=10):
    out = []
    ncon = int(getattr(data, "ncon", 0))
    for i in range(min(ncon, max_items)):
        c = data.contact[i]
        g1 = int(c.geom1)
        g2 = int(c.geom2)
        n1 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, g1) or f"geom_{g1}"
        n2 = mj.mj_id2name(model, mj.mjtObj.mjOBJ_GEOM, g2) or f"geom_{g2}"
        b1 = _geom_body_name(mj, model, g1)
        b2 = _geom_body_name(mj, model, g2)
        out.append({"geom1": n1, "geom2": n2, "body1": b1, "body2": b2})
    return out


def _euler_xyz_to_rotmat(rx: float, ry: float, rz: float):
    """Return 3x3 rotation matrix for XYZ intrinsic Euler (matching Blender/MuJoCo use here)."""
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    # Blender Euler('XYZ') / intrinsic XYZ equals matrix composition Rz @ Ry @ Rx.
    # Using Rx @ Ry @ Rz swaps axes in orientation targets (observed flange axis mismatch).
    return _np.asarray(
        [
            [cy * cz, cz * sx * sy - cx * sz, sx * sz + cx * cz * sy],
            [cy * sz, cx * cz + sx * sy * sz, cx * sy * sz - cz * sx],
            [-sy, cy * sx, cx * cy],
        ],
        dtype=float,
    )


def _orientation_error_vec(r_cur, r_tgt):
    """
    Small-angle orientation error vector in world frame.
    0.5 * sum_i (r_cur_i x r_tgt_i)
    """
    return 0.5 * (
        _np.cross(r_cur[:, 0], r_tgt[:, 0])
        + _np.cross(r_cur[:, 1], r_tgt[:, 1])
        + _np.cross(r_cur[:, 2], r_tgt[:, 2])
    )


def _filter_contacts_for_validation(contacts: list, ignore_robot_self_contacts: bool) -> tuple[list, list]:
    """
    Returns (effective_contacts, ignored_contacts).
    For now, optionally ignore any contact where both bodies belong to the robot chain/base.
    """
    if not contacts:
        return [], []
    if not ignore_robot_self_contacts:
        return list(contacts), []

    def _is_robot_body(name: str) -> bool:
        if not name:
            return False
        return name == "base_link" or name.startswith("link_")

    effective = []
    ignored = []
    for c in contacts:
        b1 = str(c.get("body1") or "")
        b2 = str(c.get("body2") or "")
        if _is_robot_body(b1) and _is_robot_body(b2):
            ignored.append(c)
        else:
            effective.append(c)
    return effective, ignored


def _mujoco_ik_validate_path(session: dict, payload: dict):
    """
    Minimal real validator: position-only IK per waypoint + joint limit checks.
    Uses MuJoCo contacts as a basic collision signal on the solved pose.
    """
    if _np is None:
        raise RuntimeError("numpy unavailable")

    t0 = time.perf_counter()
    mj = session["mujoco"]
    model = session["model"]
    data = session["data"]
    joint_ids = session["joint_ids"]
    qpos_ids = session["qpos_ids"]
    jnt_ranges = session["jnt_ranges"]
    ee_body_id = session["ee_body_id"]
    ee_site_id = session.get("ee_site_id", -1)
    ee_use_site = session.get("ee_use_site", False)
    waypoints = payload.get("waypoints") or []
    if not waypoints:
        return {
            "is_valid": False,
            "first_failure_index": None,
            "failure_kind": "solver_failure",
            "message": "No waypoints received",
            "contacts": [],
            "stats": {"elapsed_ms": 0.0, "waypoints_checked": 0, "substeps": 0},
            "backend": "mujoco_ik",
            "backend_mode": "real",
        }

    opts = payload.get("options") or {}
    max_iters = max(1, int(opts.get("ik_max_iters", 80)))
    pos_tol = max(1e-5, float(opts.get("ik_pos_tol_m", 0.005)))
    rot_tol = max(1e-5, float(opts.get("ik_rot_tol_rad", 0.08)))  # ~4.6 deg default
    damping = max(1e-9, float(opts.get("ik_damping", 1e-4)))
    max_step = max(1e-4, float(opts.get("ik_max_step_rad", 0.2)))
    collision_check = bool(opts.get("ik_check_contacts", True))
    ignore_robot_self_contacts = bool(opts.get("ik_ignore_robot_self_contacts", True))
    keep_orientation = bool(opts.get("ik_keep_orientation", True))
    pos_weight = max(1e-6, float(opts.get("ik_pos_weight", 1.0)))
    rot_weight = max(1e-6, float(opts.get("ik_rot_weight", 0.35)))

    init_joints_deg = payload.get("current_joints_deg") or []
    if init_joints_deg:
        for j, qidx in enumerate(qpos_ids):
            if j >= len(init_joints_deg):
                break
            q = math.radians(float(init_joints_deg[j]))
            lo, hi = jnt_ranges[j]
            data.qpos[qidx] = min(max(q, lo), hi)
    for qidx, (lo, hi) in zip(qpos_ids, jnt_ranges):
        data.qpos[qidx] = min(max(float(data.qpos[qidx]), lo), hi)
    mj.mj_forward(model, data)

    jacp = _np.zeros((3, model.nv), dtype=float)
    jacr = _np.zeros((3, model.nv), dtype=float)
    identity3 = _np.eye(3, dtype=float)
    identity6 = _np.eye(6, dtype=float)

    first_failure = None
    failure_kind = None
    message = "Path validated (MuJoCo IK)"
    contacts = []
    ignored_self_contacts_total = 0
    first_waypoint_solution_rad = None
    first_waypoint_solution_deg = None
    waypoint_solutions_rad = []
    waypoint_solutions_deg = []
    first_waypoint_target_euler_rad = None
    first_waypoint_final_pos_err_m = None
    first_waypoint_final_rot_err_rad = None

    def _ee_pos():
        if ee_use_site:
            return _np.asarray(data.site_xpos[ee_site_id], dtype=float).copy()
        return _np.asarray(data.xpos[ee_body_id], dtype=float).copy()

    def _ee_rotmat():
        if ee_use_site:
            return _np.asarray(data.site_xmat[ee_site_id], dtype=float).reshape(3, 3)
        return _np.asarray(data.xmat[ee_body_id], dtype=float).reshape(3, 3)

    def _ee_jac():
        jacp.fill(0.0)
        jacr.fill(0.0)
        if ee_use_site:
            mj.mj_jacSite(model, data, jacp, jacr, ee_site_id)
        else:
            mj.mj_jacBody(model, data, jacp, jacr, ee_body_id)

    for idx, wp in enumerate(waypoints):
        pos = wp.get("position_m") or [0.0, 0.0, 0.0]
        target = _np.asarray([float(pos[0]), float(pos[1]), float(pos[2])], dtype=float)
        target_euler = wp.get("euler_rad") or payload.get("tool_orientation_rad") or [0.0, 0.0, 0.0]
        target_rot = _euler_xyz_to_rotmat(
            float(target_euler[0]), float(target_euler[1]), float(target_euler[2])
        ) if keep_orientation else None
        if idx == 0 and first_waypoint_target_euler_rad is None:
            first_waypoint_target_euler_rad = [float(target_euler[0]), float(target_euler[1]), float(target_euler[2])]

        solved = False
        for _ in range(max_iters):
            mj.mj_forward(model, data)
            cur = _ee_pos()
            err = target - cur
            pos_err_norm = float(_np.linalg.norm(err))

            rot_err = _np.zeros(3, dtype=float)
            rot_err_norm = 0.0
            if keep_orientation:
                r_cur = _ee_rotmat()
                rot_err = _orientation_error_vec(r_cur, target_rot)
                rot_err_norm = float(_np.linalg.norm(rot_err))

            if pos_err_norm <= pos_tol and (not keep_orientation or rot_err_norm <= rot_tol):
                solved = True
                break

            _ee_jac()

            if keep_orientation:
                Jp = jacp[:, qpos_ids]
                Jr = jacr[:, qpos_ids]
                J = _np.vstack((pos_weight * Jp, rot_weight * Jr))
                task_err = _np.concatenate((pos_weight * err, rot_weight * rot_err))
                JT = J.T
                A = (J @ JT) + (damping * identity6)
            else:
                J = jacp[:, qpos_ids]
                task_err = err
                JT = J.T
                A = (J @ JT) + (damping * identity3)
            try:
                dq = JT @ _np.linalg.solve(A, task_err)
            except _np.linalg.LinAlgError:
                break
            dq = _np.clip(dq, -max_step, max_step)
            for j, qidx in enumerate(qpos_ids):
                q = float(data.qpos[qidx]) + float(dq[j])
                if int(model.jnt_limited[joint_ids[j]]):
                    lo, hi = jnt_ranges[j]
                    q = min(max(q, lo), hi)
                data.qpos[qidx] = q

        mj.mj_forward(model, data)

        if not solved:
            first_failure = idx
            failure_kind = "solver_failure"
            cur = _ee_pos()
            err_norm = float(_np.linalg.norm(target - cur))
            if keep_orientation:
                r_cur = _ee_rotmat()
                rot_err_norm = float(_np.linalg.norm(_orientation_error_vec(r_cur, target_rot)))
                message = f"MuJoCo IK did not converge (pos {err_norm:.4f} m, rot {rot_err_norm:.4f} rad)"
            else:
                message = f"MuJoCo IK did not converge (pos error {err_norm:.4f} m)"
            break

        for j, qidx in enumerate(qpos_ids):
            lo, hi = jnt_ranges[j]
            q = float(data.qpos[qidx])
            if q < lo - 1e-6 or q > hi + 1e-6:
                first_failure = idx
                failure_kind = "joint_limit"
                message = f"Joint {j+1} exceeds range"
                break
        if first_failure is not None:
            break

        solved_q_rad = [float(data.qpos[qidx]) for qidx in qpos_ids]
        solved_q_deg = [float(math.degrees(v)) for v in solved_q_rad]
        waypoint_solutions_rad.append(solved_q_rad)
        waypoint_solutions_deg.append(solved_q_deg)
        if idx == 0 and first_waypoint_solution_rad is None:
            first_waypoint_solution_rad = list(solved_q_rad)
            first_waypoint_solution_deg = list(solved_q_deg)
            cur = _ee_pos()
            first_waypoint_final_pos_err_m = float(_np.linalg.norm(target - cur))
            if keep_orientation:
                r_cur = _ee_rotmat()
                first_waypoint_final_rot_err_rad = float(_np.linalg.norm(_orientation_error_vec(r_cur, target_rot)))

        if collision_check and int(getattr(data, "ncon", 0)) > 0:
            raw_contacts = _contact_summary(mj, model, data)
            effective_contacts, ignored_contacts = _filter_contacts_for_validation(
                raw_contacts, ignore_robot_self_contacts=ignore_robot_self_contacts
            )
            ignored_self_contacts_total += len(ignored_contacts)
            if effective_contacts:
                first_failure = idx
                failure_kind = "collision"
                contacts = effective_contacts
                message = "MuJoCo reported contact(s) at solved waypoint"
                if ignored_contacts:
                    message += f" ({len(ignored_contacts)} self-contact(s) ignored)"
                break

    elapsed_ms = (time.perf_counter() - t0) * 1000.0
    return {
        "is_valid": first_failure is None,
        "first_failure_index": first_failure,
        "failure_kind": failure_kind,
        "message": message,
        "contacts": contacts,
        "debug": {
            "ik_check_contacts": collision_check,
            "ik_ignore_robot_self_contacts": ignore_robot_self_contacts,
            "ik_keep_orientation": keep_orientation,
            "ik_pos_tol_m": pos_tol,
            "ik_rot_tol_rad": rot_tol,
            "ik_pos_weight": pos_weight,
            "ik_rot_weight": rot_weight,
            "ignored_self_contacts_total": ignored_self_contacts_total,
            "ik_first_waypoint_solution_index": 0 if first_waypoint_solution_rad is not None else None,
            "ik_first_waypoint_solution_rad": first_waypoint_solution_rad,
            "ik_first_waypoint_solution_deg": first_waypoint_solution_deg,
            "ik_first_waypoint_target_euler_rad": first_waypoint_target_euler_rad,
            "ik_first_waypoint_final_pos_err_m": first_waypoint_final_pos_err_m,
            "ik_first_waypoint_final_rot_err_rad": first_waypoint_final_rot_err_rad,
            "ik_waypoint_solutions_count": len(waypoint_solutions_deg),
            "ik_waypoint_solutions_rad": waypoint_solutions_rad,
            "ik_waypoint_solutions_deg": waypoint_solutions_deg,
        },
        "stats": {
            "elapsed_ms": round(elapsed_ms, 3),
            "waypoints_checked": len(waypoints) if first_failure is None else first_failure + 1,
            "substeps": int((opts or {}).get("validation_substeps", 1)),
        },
        "backend": "mujoco_ik",
        "backend_mode": "real",
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stdio", action="store_true", help="Run JSON line protocol over stdin/stdout")
    _ = parser.parse_args()

    backend_info = _detect_backend()
    # Session state — populated by init, used by validate_path.
    session: dict = {"mujoco_xml": None, "mujoco_session": None, "runtime_backend_info": dict(backend_info)}

    for raw in sys.stdin:
        raw = raw.strip()
        if not raw:
            continue
        try:
            msg = json.loads(raw)
        except Exception:
            _send({"type": "error", "message": "Invalid JSON"})
            continue

        cmd = msg.get("cmd")
        payload = msg.get("payload") or {}

        if cmd == "init":
            _log_model_info(payload)
            session["mujoco_xml"] = payload.get("mujoco_xml")
            runtime_backend_info = dict(backend_info)
            session["mujoco_session"] = None
            mujoco_xml = payload.get("mujoco_xml")
            if mujoco_xml and backend_info.get("mujoco_available"):
                compiled, compile_err = _compile_mujoco_session(mujoco_xml)
                if compiled is not None:
                    session["mujoco_session"] = compiled
                    runtime_backend_info["backend"] = "mujoco_ik"
                    runtime_backend_info["mode"] = "real"
                    runtime_backend_info["message"] = "MuJoCo model compiled; position IK validator enabled"
                else:
                    runtime_backend_info["backend"] = "newton_stub"
                    runtime_backend_info["mode"] = "stub"
                    runtime_backend_info["message"] = f"MuJoCo model compile failed; using stub validator ({compile_err})"
            elif payload.get("mujoco_xml_error"):
                runtime_backend_info["message"] = f"MuJoCo XML export failed in Blender: {payload.get('mujoco_xml_error')}"
            session["runtime_backend_info"] = runtime_backend_info
            _send({"type": "ok", "message": "Worker initialized", "backend_info": runtime_backend_info})
            continue
        if cmd == "validate_path":
            runtime_backend_info = dict(session.get("runtime_backend_info") or backend_info)
            if session.get("mujoco_session") is not None and runtime_backend_info.get("mode") == "real":
                try:
                    report = _mujoco_ik_validate_path(session["mujoco_session"], payload)
                except Exception as exc:
                    report = _stub_validate_path(payload)
                    runtime_backend_info["backend"] = "newton_stub"
                    runtime_backend_info["mode"] = "stub"
                    runtime_backend_info["message"] = (
                        f"MuJoCo IK validator crashed; using stub ({exc.__class__.__name__}: {exc})"
                    )
            else:
                report = _stub_validate_path(payload)
            report.setdefault("backend", runtime_backend_info.get("backend", "stub"))
            report["backend_mode"] = runtime_backend_info.get("mode", "stub")
            report["backend_message"] = runtime_backend_info.get("message", "")
            _send({"type": "validation_report", "report": report})
            continue
        if cmd == "shutdown":
            _send({"type": "ok", "message": "Shutting down"})
            return 0

        _send({"type": "error", "message": f"Unknown command: {cmd!r}"})

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
