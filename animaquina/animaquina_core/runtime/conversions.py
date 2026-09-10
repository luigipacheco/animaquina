# Copyright (C) 2026 Luis Arturo Pacheco
# SPDX-License-Identifier: GPL-3.0-or-later
#
# Animaquina Core — canonical conversions (Section 10)

import math


def deg2rad(deg: float) -> float:
    return deg * math.pi / 180.0


def rad2deg(rad: float) -> float:
    return rad * 180.0 / math.pi


def rpy2rv(roll: float, pitch: float, yaw: float) -> tuple:
    """Euler (roll, pitch, yaw) in rad → rotation vector (rx, ry, rz)."""
    ca, cb, cg = math.cos(yaw), math.cos(pitch), math.cos(roll)
    sa, sb, sg = math.sin(yaw), math.sin(pitch), math.sin(roll)
    r11 = ca * cb
    r12 = ca * sb * sg - sa * cg
    r13 = ca * sb * cg + sa * sg
    r21 = sa * cb
    r22 = sa * sb * sg + ca * cg
    r23 = sa * sb * cg - ca * sg
    r31 = -sb
    r32 = cb * sg
    r33 = cb * cg
    theta = math.acos(max(-1, min(1, (r11 + r22 + r33 - 1) / 2)))
    if abs(theta) < 1e-10:
        return (0.0, 0.0, 0.0)
    sth = math.sin(theta)
    kx = (r32 - r23) / (2 * sth)
    ky = (r13 - r31) / (2 * sth)
    kz = (r21 - r12) / (2 * sth)
    return (theta * kx, theta * ky, theta * kz)


def rv2rpy(rx: float, ry: float, rz: float) -> tuple:
    """Rotation vector (rx, ry, rz) → Euler (roll, pitch, yaw) in rad."""
    theta = math.sqrt(rx * rx + ry * ry + rz * rz)
    if theta == 0:
        return (0.0, 0.0, 0.0)
    kx, ky, kz = rx / theta, ry / theta, rz / theta
    cth = math.cos(theta)
    sth = math.sin(theta)
    vth = 1 - cth
    r11 = kx * kx * vth + cth
    r12 = kx * ky * vth - kz * sth
    r13 = kx * kz * vth + ky * sth
    r21 = kx * ky * vth + kz * sth
    r22 = ky * ky * vth + cth
    r23 = ky * kz * vth - kx * sth
    r31 = kx * kz * vth - ky * sth
    r32 = ky * kz * vth + kx * sth
    r33 = kz * kz * vth + cth
    beta = math.atan2(-r31, math.sqrt(r11 * r11 + r21 * r21))
    d89 = deg2rad(89.99)
    if beta > d89:
        beta = d89
        alpha = 0
        gamma = math.atan2(r12, r22)
    elif beta < -d89:
        beta = -d89
        alpha = 0
        gamma = -math.atan2(r12, r22)
    else:
        cb = math.cos(beta)
        alpha = math.atan2(r21 / cb, r11 / cb)
        gamma = math.atan2(r32 / cb, r33 / cb)
    return (gamma, beta, alpha)  # roll, pitch, yaw


def kuka_base_to_blender(pos_m: tuple, euler_abc: tuple) -> tuple:
    """Convert raw KUKA $BASE frame to Blender-ready (location, rotation_euler).

    Pure-math equivalent of the old mathutils-based _kuka_base_to_blender.
    Builds the full rigid transform and inverts it so position and rotation
    are coupled — exactly like Blender parenting.

    Args:
        pos_m: (x, y, z) in metres.
        euler_abc: (A, B, C) in radians (KUKA intrinsic ZYX order).

    Returns:
        ((loc_x, loc_y, loc_z), (euler_x, euler_y, euler_z)) — Blender XYZ Euler.
    """
    a_rad = float(euler_abc[0])
    b_rad = float(euler_abc[1])
    c_rad = float(euler_abc[2])

    # KUKA intrinsic ZYX(A,B,C) = Blender XYZ(C,B,A)
    rx, ry, rz = c_rad, b_rad, a_rad

    # Build rotation matrix for intrinsic XYZ = extrinsic ZYX:
    # R = Rz(rz) @ Ry(ry) @ Rx(rx)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)

    r00 = cy * cz
    r01 = cz * sx * sy - cx * sz
    r02 = sx * sz + cx * cz * sy
    r10 = cy * sz
    r11 = cx * cz + sx * sy * sz
    r12 = cx * sy * sz - cz * sx
    r20 = -sy
    r21 = cy * sx
    r22 = cx * cy

    tx = float(pos_m[0])
    ty = float(pos_m[1])
    tz = float(pos_m[2])

    # Inverse of rigid transform [R|t; 0|1] = [R^T | -R^T*t; 0|1]
    ir00, ir01, ir02 = r00, r10, r20
    ir10, ir11, ir12 = r01, r11, r21
    ir20, ir21, ir22 = r02, r12, r22

    itx = -(ir00 * tx + ir01 * ty + ir02 * tz)
    ity = -(ir10 * tx + ir11 * ty + ir12 * tz)
    itz = -(ir20 * tx + ir21 * ty + ir22 * tz)

    # Extract Euler XYZ from inverse rotation matrix [ir]:
    # R_xyz layout: r20 = -sin(ey)
    val = max(-1.0, min(1.0, -ir20))
    ey = math.asin(val)

    if abs(abs(val) - 1.0) < 1e-8:
        # Gimbal lock
        ex = math.atan2(-ir12, ir11)
        ez = 0.0
    else:
        ex = math.atan2(ir21, ir22)
        ez = math.atan2(ir10, ir00)

    return ((itx, ity, itz), (ex, ey, ez))
