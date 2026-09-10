# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
# Animaquina — numpy-accelerated waypoint/attribute readers shared by the
# robot program exporters (KUKA KRL, UR script).

import numpy as np

# Mesh attribute data_type → numpy dtype for foreach_get fast reads.
_SCALAR_ATTR_DTYPES = {
    "FLOAT": np.float32,
    "INT": np.int32,
    "BOOLEAN": bool,
}


def read_scalar_attribute(att, cast=None) -> list:
    """Read a scalar mesh attribute into a plain Python list.

    Uses foreach_get into a numpy buffer when the attribute type allows it;
    falls back to per-item RNA access otherwise. `cast=bool` coerces values
    the same way the old per-point list comprehensions did.
    """
    dtype = _SCALAR_ATTR_DTYPES.get(getattr(att, "data_type", None))
    if dtype is not None:
        try:
            buf = np.empty(len(att.data), dtype=dtype)
            att.data.foreach_get("value", buf)
            if cast is bool:
                buf = buf.astype(bool)
            return buf.tolist()
        except Exception:
            pass
    values = [getattr(v, "value", None) for v in att.data]
    if cast is bool:
        values = [bool(v) for v in values]
    return values


def collect_custom_var_attributes(slot, eval_obj, builtin_names) -> dict:
    """Collect per-point attribute values for each tracked debug variable.

    Returns {var_name: [value_per_point, ...]} for variables that have a matching
    mesh attribute. Variables without a matching attribute are silently skipped
    (they're still monitored, just not exported).
    """
    custom_vars = {}
    for item in getattr(slot, "debug_vars", []):
        if not bool(getattr(item, "enabled", True)):
            continue
        var_name = str(getattr(item, "var_name", "") or "").strip()
        if not var_name or var_name in builtin_names:
            continue
        att = eval_obj.data.attributes.get(var_name)
        if att is None:
            continue
        custom_vars[var_name] = read_scalar_attribute(att)
    return custom_vars


def eulers_xyz_to_matrices(eulers: np.ndarray) -> np.ndarray:
    """(N,3) XYZ Euler angles (rad) → (N,3,3) rotation matrices.

    Blender 'XYZ' Euler convention: R = Rz @ Ry @ Rx (same layout as
    conversions.kuka_base_to_blender / conversions.rpy2rv).
    """
    x, y, z = eulers[:, 0], eulers[:, 1], eulers[:, 2]
    cx, sx = np.cos(x), np.sin(x)
    cy, sy = np.cos(y), np.sin(y)
    cz, sz = np.cos(z), np.sin(z)
    m = np.empty((eulers.shape[0], 3, 3), dtype=np.float64)
    m[:, 0, 0] = cy * cz
    m[:, 0, 1] = cz * sx * sy - cx * sz
    m[:, 0, 2] = sx * sz + cx * cz * sy
    m[:, 1, 0] = cy * sz
    m[:, 1, 1] = cx * cz + sx * sy * sz
    m[:, 1, 2] = cx * sy * sz - cz * sx
    m[:, 2, 0] = -sy
    m[:, 2, 1] = cy * sx
    m[:, 2, 2] = cx * cy
    return m


def matrices_to_eulers_xyz(mats: np.ndarray) -> np.ndarray:
    """(N,3,3) rotation matrices → (N,3) XYZ Euler angles (rad).

    Any gimbal-locked matrix resolves with ez = 0 (both branches encode the
    same rotation, so the exported pose is unchanged).
    """
    sy = np.clip(-mats[:, 2, 0], -1.0, 1.0)
    ey = np.arcsin(sy)
    ex = np.arctan2(mats[:, 2, 1], mats[:, 2, 2])
    ez = np.arctan2(mats[:, 1, 0], mats[:, 0, 0])
    lock = np.abs(sy) > 1.0 - 1e-8
    if np.any(lock):
        ex = np.where(lock, np.arctan2(-mats[:, 1, 2], mats[:, 1, 1]), ex)
        ez = np.where(lock, 0.0, ez)
    return np.stack((ex, ey, ez), axis=1)


def eulers_xyz_to_rotvecs(eulers: np.ndarray) -> np.ndarray:
    """(N,3) XYZ Euler angles (rad) → (N,3) axis-angle rotation vectors.

    Vectorized equivalent of conversions.rpy2rv applied per waypoint.
    """
    m = eulers_xyz_to_matrices(eulers)
    trace = m[:, 0, 0] + m[:, 1, 1] + m[:, 2, 2]
    theta = np.arccos(np.clip((trace - 1.0) / 2.0, -1.0, 1.0))
    scale = np.zeros_like(theta)
    ok = np.abs(theta) >= 1e-10
    with np.errstate(divide="ignore", invalid="ignore"):
        scale[ok] = theta[ok] / (2.0 * np.sin(theta[ok]))
    rv = np.empty_like(eulers)
    rv[:, 0] = (m[:, 2, 1] - m[:, 1, 2]) * scale
    rv[:, 1] = (m[:, 0, 2] - m[:, 2, 0]) * scale
    rv[:, 2] = (m[:, 1, 0] - m[:, 0, 1]) * scale
    return rv


def compute_base_waypoints(
    eval_mesh,
    world_matrix,
    base_inv,
    use_rotation_attr: bool,
    apply_rot_transform: bool,
    constant_euler_rad,
):
    """Batch-transform the 'position' attribute into the robot base frame.

    Returns (positions (N,3) metres, eulers (N,3) rad Blender XYZ), both float64.
    Points beyond the length of the 'rotation' attribute (or all points when it
    is absent/disabled) get constant_euler_rad, matching the old per-point loop.
    """
    pos_data = eval_mesh.attributes["position"].data
    n = len(pos_data)
    if n == 0:
        return np.empty((0, 3)), np.empty((0, 3))

    local = np.empty(n * 3, dtype=np.float32)
    pos_data.foreach_get("vector", local)
    local = local.reshape(n, 3).astype(np.float64)

    m = np.array(base_inv @ world_matrix, dtype=np.float64)
    positions = local @ m[:3, :3].T + m[:3, 3]

    eulers = np.tile(np.asarray(constant_euler_rad, dtype=np.float64), (n, 1))
    rotation_att = eval_mesh.attributes.get("rotation") if use_rotation_attr else None
    if rotation_att is not None:
        rn = min(len(rotation_att.data), n)
        if rn:
            buf = np.empty(len(rotation_att.data) * 3, dtype=np.float32)
            rotation_att.data.foreach_get("vector", buf)
            local_eulers = np.radians(buf.reshape(-1, 3)[:rn].astype(np.float64))
            if apply_rot_transform:
                rot = m[:3, :3].copy()
                norms = np.linalg.norm(rot, axis=0)
                rot /= np.where(norms == 0.0, 1.0, norms)
                eulers[:rn] = matrices_to_eulers_xyz(rot @ eulers_xyz_to_matrices(local_eulers))
            else:
                eulers[:rn] = local_eulers
    return positions, eulers
