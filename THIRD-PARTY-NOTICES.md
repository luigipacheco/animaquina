# Third-party components

Animaquina itself is GPL-3.0-or-later. It bundles or builds against the
components below. Every one must stay GPL-3-compatible; if you add a dependency,
check it here first.

## Bundled in `animaquina/libs/` (shipped in this repository)

| Component | Upstream | Licence | Notes |
|---|---|---|---|
| `math3d` (PyMath3D) | Morten Lind | LGPL-3.0 | Compatible: LGPL-3 may be used inside a GPL-3 work. |
| `urx` | Olivier Roulet-Dubonnet | LGPL-3.0 | Legacy UR fallback backend. `ur_rtde` is preferred. |
| `xarm` (xArm-Python-SDK) | UFACTORY, Inc. | BSD-3-Clause | Per-file headers read "Software License Agreement (BSD License), Copyright (c) 2018, UFACTORY, Inc." The vendored copy is missing the top-level `LICENSE` file - copy it from upstream into `animaquina/libs/xarm/` to complete the attribution. |

## Built into `animaquina/vendor_py/` (not committed — see `tools/vendor-build/`)

| Component | Licence | Notes |
|---|---|---|
| `ur_rtde` | MIT | Shipped as a **patched master build**, not a PyPI release. The patch is archived at `tools/vendor-build/patches/ur_rtde-animaquina.patch` and is a candidate for upstream merge. See `tools/vendor-build/README.md`. |
| `paramiko` | LGPL-2.1 | SFTP upload for UR. |
| `cryptography`, `bcrypt`, `PyNaCl`, `cffi`, `pycparser`, `invoke` | Apache-2.0 / BSD / MIT | Transitive dependencies of paramiko. |

`vendor_py/` contains compiled binaries and is intentionally **not** committed.
Rebuild it with the scripts in `tools/vendor-build/`. The long-term plan is to
ship these as Blender extension wheels declared in `blender_manifest.toml`.

## Robot models and geometry — not yet included

Robot meshes, rigs (`robots.blend`), and the KUKA URDF/xacro descriptions from
the previous private repository are **deliberately absent from this repository**
pending a provenance and redistribution review.

They derive from manufacturer CAD and from ROS-Industrial packages
(`kuka_agilus_support` and related). Shipping them to a handful of labs under a
beta agreement is a different act from publishing them worldwide. Before adding
them here, confirm for each asset:

- the upstream package and its licence (ROS-Industrial packages are typically
  Apache-2.0 or BSD-3-Clause — record which, and include the upstream
  `LICENSE` and `package.xml`),
- whether the manufacturer permits redistribution of the underlying CAD-derived
  geometry, and
- attribution for anything modified.

Once cleared, add them under `assets/` with a per-source `README` recording
origin, licence, and any modifications.

## Protocols

The KUKA driver speaks a variable-read/write protocol on TCP port 7000 that is
served by third-party software running on the controller (KUKAVARPROXY /
C3 Bridge). That server is **not** part of Animaquina, is not distributed here,
and has its own licence and terms. You must obtain and install it yourself.
