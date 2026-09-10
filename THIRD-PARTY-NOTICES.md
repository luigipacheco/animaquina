# Third-party components

Animaquina itself is GPL-3.0-or-later. It bundles or builds against the
components below. Every one must stay GPL-3-compatible; if you add a dependency,
check it here first.

## Bundled in `animaquina/libs/` (shipped in this repository)

| Component | Upstream | Licence | Notes |
|---|---|---|---|
| `math3d` (PyMath3D) | Morten Lind | LGPL-3.0 | Licence text at `animaquina/libs/math3d/LICENSE`. Compatible: LGPL-3 may be used inside a GPL-3 work. |
| `urx` | Olivier Roulet-Dubonnet | LGPL-3.0 | Licence text at `animaquina/libs/urx/LICENSE`. Legacy UR fallback backend; `ur_rtde` is preferred. |
| `xarm` (xArm-Python-SDK) v1.17.6 | UFACTORY, Inc. | BSD-3-Clause | Licence text at `animaquina/libs/xarm/LICENSE`, taken from [upstream](https://github.com/xArm-Developer/xArm-Python-SDK). Per-file headers carry the copyright notice, satisfying clause 1. |

## Built into `animaquina/vendor_py/` (not committed — see `tools/vendor-build/`)

| Component | Licence | Notes |
|---|---|---|
| `ur_rtde` | MIT | Shipped as a **patched master build**, not a PyPI release. The patch is archived at `tools/vendor-build/patches/ur_rtde-animaquina.patch` and is a candidate for upstream merge. See `tools/vendor-build/README.md`. |
| `paramiko` | LGPL-2.1 | SFTP upload for UR. |
| `cryptography`, `bcrypt`, `PyNaCl`, `cffi`, `pycparser`, `invoke` | Apache-2.0 / BSD / MIT | Transitive dependencies of paramiko. |

`vendor_py/` contains compiled binaries and is intentionally **not** committed.
Rebuild it with the scripts in `tools/vendor-build/`. The long-term plan is to
ship these as Blender extension wheels declared in `blender_manifest.toml`.

## Robot models and geometry

Robot meshes and rigs live in `assets/robots/` — beside the add-on package, not
inside it. They are committed here so the rigs are versioned with the code they
match, but they are **excluded from the add-on zip** and ship as a separate
release asset. A GPL-3 add-on zip should not imply a GPL-3 grant over geometry
the project does not own outright.

### Origin

The KUKA, xArm and UFACTORY models originate as **STEP/CAD files downloaded from
the manufacturers' own websites**. For each robot they were exported to mesh,
cleaned up, and rigged as a Blender armature (`joint_1`..`joint_6`, with the
Animaquina axis mapping) by Luis Arturo Pacheco.

They are **not** derived from ROS-Industrial or any other open-source robot
description package, so no third-party open-source licence obligations attach to
them.

### Licence

The rig work — the export and cleanup, the armatures, the `joint_1`..`joint_6`
bone convention, and the per-joint axis mapping — is original work of this
project and is licensed **[CC-BY-4.0](LICENSES/CC-BY-4.0.txt)**.

Attribute it as:

> Animaquina robot rigs by Luis Arturo Pacheco, licensed CC BY 4.0.
> https://github.com/luigipacheco/animaquina

The underlying geometry remains the manufacturers'. Original work on top of a CAD
model does not extinguish rights in the model it derives from, so the CC-BY-4.0
grant covers this project's contribution and cannot grant more than the project
holds. Each vendor's CAD terms of use still govern the geometry itself.

Redistributing vendor robot geometry is common across the ecosystem — ROS
description packages, offline programming suites and CAD plugins all ship it, and
vendors generally welcome what makes their robots easier to specify and buy.
A written confirmation from KUKA and UFACTORY is still worth having on file;
record it here if you obtain it.

## Protocols

The KUKA driver speaks a variable-read/write protocol on TCP port 7000 that is
served by third-party software running on the controller (KUKAVARPROXY /
C3 Bridge). That server is **not** part of Animaquina, is not distributed here,
and has its own licence and terms. You must obtain and install it yourself.
