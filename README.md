# Animaquina

A Blender add-on for multi-robot control, digital twin visualization, interactive toolpathing, and offline program export. Connect to industrial and collaborative robots directly from Blender's 3D viewport.

> ### ⚠️ This software drives industrial robots
> It performs **no safety function** and is **not** a substitute for your robot's
> safety-rated systems, guarding, or a risk assessment. It is provided with
> **absolutely no warranty**. Path validation has known gaps that can report a
> pass on motion that is not safe. **Read [SAFETY.md](SAFETY.md) before connecting
> to any machine.**

**Version:** 0.1.0 (Beta) · **Blender:** 5.2 LTS · **Licence:** [GPL-3.0-or-later](LICENSE) · **Author:** Luis Arturo Pacheco

Works with GeoSlicer for attributed toolpaths and PhyNodes for sensor and
end-effector signals. See the [user guide](USER-README.md) for setup and
examples.

## Supported Robots

| Brand | Models | Protocol |
|-------|--------|----------|
| **Universal Robots** | UR3/5/10/16/20/30 | RTDE (preferred), URX (fallback) |
| **KUKA** | KR10, KR30, KR120 | Socket KRL (kukaproxydriver) |
| **xArm / UFactory** | xArm 6, UF850 | xArm Python SDK |

## Features

### Digital Twin & Live Monitoring
- Real-time joint angle and TCP pose synchronization via background polling threads
- Non-blocking I/O: all robot communication runs off-thread; the Blender UI stays responsive
- Per-robot polling toggle with configurable rate (1-50 Hz)
- Live display of TCP position, orientation, joint angles, and base frame in the Info panel

### Multi-Robot Support
- Slot-based architecture: add, configure, and control multiple robots in a single scene
- Each slot holds its own connection, rig references, motion settings, and cached state
- Independent connect/disconnect per robot

### Interactive Motion Control
- **Move to Target** -- send the robot to a Blender empty's world position (Linear or PTP)
- **Run Toolpath** -- extract waypoints from a mesh's **`position`** attribute (evaluated mesh, e.g. from Geometry Nodes) or curve control points and stream them to the robot
- **Buffered Toolpath** -- ring-buffer streaming for KUKA Dynamic Sync and UR RTDE registers
- **Teach Mode (Freedrive)** -- enable manual guidance on UR and xArm
- **Puppet Mode** -- continuous real-time target following with configurable rate and safety limits
- **Snap Target** -- snap the target empty to the current TCP pose
- **Go Home / Reset** -- return to a stored home position or clear controller errors

### Rig Binding & Simulation
- Bind any Blender armature (bones `joint_1`..`joint_6`) as the robot's visual rig
- Per-joint axis mapping (X/Y/Z/-X/-Y/-Z) to match each robot's kinematic convention
- Automatic IK simulation rig: duplicate the armature, add IK constraints, and preview paths before sending
- TCP, Tool, Base, and Target object references per slot

### Offline Program Export
- **KRL Export (KUKA)** -- generate complete KRL programs from mesh waypoints, including motion parameters, base/tool numbers, and home positions
- **URScript Export (UR)** -- generate URScript programs with rotation vector poses, speed/acceleration, and blend radius
- Programs stored as Blender text blocks for review before saving
- **Stage & Upload** -- SFTP upload for UR; C3 Bridge upload for KUKA
- **Run Program** -- trigger execution via UR Dashboard or KUKA program selection

### Path Validation (Experimental)
- Newton/MuJoCo-based collision and IK validation
- Background worker process for non-blocking checks
- Reports failure index, contact details, and timing

### End-Effector Streaming (KUKA)

During path execution or puppet mode, the robot controller exposes a single **`IDX`** variable — the current waypoint index. Blender reads `IDX` from the controller and writes the corresponding end-effector attributes for that point:

| Variable | Type | Purpose |
|----------|------|---------|
| `IDX` | INT | Current waypoint index (robot writes, Blender reads) |
| `E_ENABLE` | BOOL | End-effector enable/disable |
| `E_SPEED` | REAL | Extrusion / end-effector speed |
| `E_FLOW` | REAL | Flow rate |
| `E_TEMP` | REAL | Temperature setpoint |
| `F_SPEED` | REAL | Fan speed |

This index-based approach avoids per-attribute synchronization — one read of `IDX` tells Blender which waypoint the robot is executing, and Blender streams the matching attributes from the mesh data (Geometry Nodes `position`, `E_SPEED`, `E_ENABLE`, `F_SPEED`, etc.) in a single update.

All variables are declared in `$CONFIG.DAT` (global scope) so they are accessible via C3 Bridge.

### Markers
- Drop PLAIN_AXES empties at the current TCP position to record waypoints visually

## Installation

### 1. Install the Add-on

1. Download a release `.zip`, or build one from source (see **Building** below).
2. Open Blender **5.2 LTS** (the bundled native dependencies are built for its Python 3.13).
3. Go to **Edit > Preferences > Add-ons** (or **Get Extensions**).
4. Click **Install from Disk** (top-right dropdown) and select the `.zip` file.
5. Enable **Animaquina** in the add-on list.

There is no licence key, activation, or feature gate. Every feature is enabled.

### 2. Install Robot Dependencies

Open the Animaquina sidebar (**N** key in 3D Viewport > **Animaquina** tab), then in the **Setup** panel:

- **Universal Robots:** Click **Install UR Dependencies** — this installs `ur_rtde` and `paramiko` via pip into Blender's Python.
- **KUKA:** No extra dependencies. The robot must be running the `kukaproxydriver` KRL program (provided separately).
- **xArm / UFactory:** The xArm SDK is bundled with the add-on.

### 3. Load Robot Assets

Animaquina needs 3D robot models (armatures) to visualize the digital twin:

1. Open or append the **robots.blend** asset file (provided with the add-on or separately).
2. The file contains collections for each robot model (e.g. `UR10e`, `KR10`, `xArm6`), each with a properly rigged armature with bones named `joint_1` through `joint_6`.
3. You can **Append** a robot collection into your scene: **File > Append > robots.blend > Collection > UR10e** (or your model).
4. Alternatively, keep robots.blend as a linked asset library.

### 4. Network Setup

Your computer and the robot must be on the **same network** (or connected directly via Ethernet):

**Universal Robots:**
- On the teach pendant: go to **Settings > System > Network** and note the robot's IP address (e.g. `192.168.1.100`).
- On your PC, set a static IP on the same subnet (e.g. `192.168.1.10`, subnet mask `255.255.255.0`).
- Verify connectivity: open a terminal and run `ping 192.168.1.100`.
- Make sure RTDE is enabled on the controller (it is by default on e-Series and newer).

**KUKA:**
- The KUKA controller must be running the `kukaproxydriver` KRL program on port `7000` (default).
- On your PC, set a static IP on the KUKA network subnet.
- Verify connectivity with `ping`.

**xArm / UFactory:**
- The xArm controller defaults to `192.168.1.xxx`. Connect your PC to the controller's Ethernet port or the same network.
- Verify with `ping`.

### 5. Connect in Blender

1. Open the 3D Viewport sidebar (**N** key) and switch to the **Animaquina** tab.
2. In the **Robot Registry** panel, click **Add** and give the slot a name (e.g. "UR10e Lab").
3. In **Setup**:
   - Choose the **Robot Type** (UR, KUKA, or xArm) and **Model**.
   - Assign the **Rig Collection** containing the robot armature you appended in step 3.
   - Enter the robot's **IP address**.
4. Click **Connect**. The status should change to "Connected".
5. Enable **Polling** to start the digital twin — you should see the 3D model match the real robot's pose.
6. Assign scene objects:
   - **TCP** — an empty at the tool center point
   - **Target** — an empty for motion commands (Move to Target, Puppet Mode)
   - Optionally: **Tool**, **Base**, **Sim Collection**

### 6. First Moves

- **Move to Target:** Position the Target empty in the viewport, then click **Move to Target** (Linear or PTP) in the **Control** panel.
- **Run Toolpath:** Select a mesh with waypoints stored in its `position` attribute (from Geometry Nodes or manual), then click **Run Toolpath**.
- **Export:** In the **Export** panel, generate an offline URScript/KRL program from the mesh waypoints.

### Optional Dependencies

- **Newton validation:** requires `mujoco` (install separately via pip).

### Developer Installation

For development (working from source):

1. Clone or download the repository.
2. Either:
   - Copy both `animaquina/` and `animaquina_core/` into Blender's addons/extensions directory, or
   - Stage `animaquina_core/` inside `animaquina/` for local source-folder development
3. Enable the add-on in Blender.

For building beta packages, see the **Beta Packaging** section in `DEVELOPER.md`.

## UI Overview

The sidebar is organized into these panels:

| Panel | Purpose |
|-------|---------|
| **Robot Registry** | Add/remove robot slots, select the active robot |
| **Setup** | Robot type, model, rig, connection, scene objects, driver-specific settings |
| **Control** | Interactive toolpathing, puppet mode, motion settings, streaming |
| **Newton** | Path validation (collision/IK check via MuJoCo), results |
| **Export** | Offline program generation, staging/upload, remote execution |
| **Info** | Read-only diagnostics: TCP, joints, base, connection status, errors |

## Variables Reference

The **Variables** panel lets you read (poll) and write controller variables in real time. Variable names also double as mesh attribute names for per-waypoint export and streaming.

### KUKA (C3 Bridge)

KUKA supports reading and writing **any KRL variable** by name — system variables, user-defined variables, and `$CONFIG.DAT` globals.

| Example | Type | Description |
|---------|------|-------------|
| `$OV_PRO` | INT | Program override speed (0–100 %) |
| `$MODE_OP` | INT | Operating mode |
| `$POS_ACT` | E6POS | Current TCP position |
| `$AXIS_ACT` | E6AXIS | Current joint angles |
| `$VEL.CP` | REAL | Cartesian velocity (m/s) |
| `$TORQUE_AXIS_ACT[1]` | REAL | Joint 1 torque |
| `E_SPEED` | REAL | End-effector speed (user-defined) |
| `E_ENABLE` | BOOL | End-effector enable (user-defined) |
| `F_SPEED` | REAL | Fan speed (user-defined) |
| `IDX` | INT | Current waypoint index (user-defined) |

Any string accepted by `KUKA.CrossComm` `read()` / `write()` works. User-defined variables must be declared in `$CONFIG.DAT` or the running program scope.

### Universal Robots (RTDE)

UR variables follow a fixed naming convention. The variable name determines which RTDE interface and method is used. Not all variables work in all contexts — see the **Export vs Streaming** table below.

**Digital I/O (read & write)**

| Variable | Pins | Read | Write | Description |
|----------|------|------|-------|-------------|
| `standard_digital_input_N` | 0–7 | yes | — | Standard digital input |
| `standard_digital_output_N` | 0–7 | yes | yes | Standard digital output |
| `configurable_digital_input_N` | 0–7 | yes | — | Configurable digital input |
| `configurable_digital_output_N` | 0–7 | yes | yes | Configurable digital output |
| `tool_digital_input_N` | 0–1 | yes | — | Tool flange digital input |
| `tool_digital_output_N` | 0–1 | yes | yes | Tool flange digital output |

**Analog I/O (read & write)**

| Variable | Pins | Read | Write | Description |
|----------|------|------|-------|-------------|
| `standard_analog_input_N` | 0–1 | yes | — | Standard analog input |
| `standard_analog_output_N` | 0–1 | yes | yes | Standard analog output (volts 0–10 V) |
| `tool_analog_output_N` | 0–1 | — | export only | Tool analog output (export only, 0.0–1.0) |

**Registers (read & write)**

| Variable | Pins | Read | Write | Description |
|----------|------|------|-------|-------------|
| `output_int_register_N` | 0–47 | yes | export only | Integer register (robot → external) |
| `output_double_register_N` | 0–47 | yes | export only | Float register (robot → external) |
| `input_int_register_N` | 0–47 | cached | export only | Integer register (external → robot) |
| `input_double_register_N` | 0–47 | cached | export only | Float register (external → robot) |

**Named state (read only)**

| Variable | Description |
|----------|-------------|
| `robot_mode` | Robot mode (running, idle, etc.) |
| `safety_mode` | Safety mode |
| `runtime_state` | Runtime state |
| `actual_tcp_speed` | Current TCP speed |
| `actual_tcp_force` | Current TCP force |
| `target_q_d` | Target joint velocities |

#### Export vs Buffered Streaming

Export generates a URScript program that runs on the controller — it has full access to all URScript built-in functions. Buffered streaming uses servoL at ~125 Hz with per-waypoint variable writes via the RTDE IO interface, which only supports a subset of outputs.

| Variable type | Export | Buffered Streaming | Why |
|---------------|--------|--------------------|-----|
| `standard_digital_output_N` | yes | **yes** | IO: `setStandardDigitalOut` |
| `configurable_digital_output_N` | yes | **yes** | IO: `setConfigurableDigitalOut` |
| `tool_digital_output_N` | yes | **yes** | IO: `setToolDigitalOut` |
| `standard_analog_output_N` | yes | **yes** | IO: `setAnalogOutputVoltage` |
| `tool_analog_output_N` | yes | no | No RTDE IO method for tool analog |
| `output_int_register_N` | yes | **no** | Needs `sendCustomScript` which kills servoL |
| `output_double_register_N` | yes | **no** | Same reason |
| `input_int_register_N` | yes | no effect | Writes succeed but no program reads them during servoL |
| `input_double_register_N` | yes | no effect | Same reason |

**Recommended for streaming:** use `standard_digital_output_N` (bool), `configurable_digital_output_N` (bool), `tool_digital_output_N` (bool), or `standard_analog_output_N` (float, volts 0–10 V).

**Recommended for export:** all variable types work. Use `output_int_register_N` / `output_double_register_N` for numeric values that the robot program can read back.

### xArm / UFactory

| Variable | Pins | Description |
|----------|------|-------------|
| `state` | — | Arm state (1=moving, 2=sleep, 3=pause, 4=error) |
| `mode` | — | Control mode |
| `error_code` | — | Current error code |
| `cgpio_digital_input_N` | 0–15 | Controller GPIO digital input |
| `cgpio_digital_output_N` | 0–15 | Controller GPIO digital output |
| `cgpio_analog_input_N` | 0–1 | Controller GPIO analog input |
| `cgpio_analog_output_N` | 0–1 | Controller GPIO analog output |
| `tgpio_digital_input_N` | 0–1 | Tool GPIO digital input |
| `tgpio_digital_output_N` | 0–1 | Tool GPIO digital output |
| `tgpio_analog_input_N` | 0–1 | Tool GPIO analog input |
| `joint_temperature_N` | 0–6 | Joint servo temperature |
| `tcp_load` | — | TCP payload (mass + CoG) |

### Using Variables with Toolpaths

When a tracked variable name matches a **point-domain mesh attribute**, the attribute values are automatically included in exported programs and buffered streaming:

1. Add the variable name to the Variables panel (e.g. `standard_digital_output_0`)
2. On your toolpath mesh, add a point-domain attribute with the same name
3. On **Export** or **Run Toolpath Buffered**, values are written per waypoint (only when the value changes)

For **UR buffered streaming**, only digital outputs, configurable outputs, tool outputs, and analog outputs work (see Export vs Streaming table above). For **export**, all variable types work including registers.

Built-in attributes (`E_SPEED`, `E_ENABLE`, `L_SPEED`, `F_SPEED`) are handled separately and don't need to be added to the Variables panel.

## For developers

- **Architecture and conventions:** see **DEVELOPER.md** at project root (layers, data model, adding a new robot, coordinate frames).
- **Design and compliance:** see **ANIMAQUINA v1.0 .md** (goals, slot model, Blender add-on rules).
- This beta implements: single PropertyGroup on Scene, slot-based Robot Manager with timer polling, UR/KUKA/xArm drivers, shared runtime, and UI with capability gating.
- `animaquina_core/` holds the Blender-independent core: robot drivers, streaming runtimes, and program generators. It never imports `bpy`, so the same code can back a non-Blender client later. It ships as plain source inside the add-on.
- Blender-facing GPL wrappers remain in `animaquina/` for modules that need `bpy`, `mathutils`, Blender text blocks, or Blender scene data access.
- **Planned:** IK simulation path preview with optional per-point rotation/orientation attributes (in addition to constant TCP orientation).

## Troubleshooting

- **`No module named 'animaquina_core'`:** Blender can see the GPL add-on package but not the core package. Either install/package `animaquina/` and `animaquina_core/` side-by-side, or stage `animaquina_core/` inside `animaquina/` for local development.
- **Compiled beta core is stale:** If Blender is still running old core behavior after you changed `animaquina_core/`, rerun `tools\sync-blender-addon.ps1` and then reload/restart Blender.
- **Compiled beta core missing `.pyc` files:** Re-run the staging script from the repo root and confirm `animaquina/animaquina_core/` contains `.pyc` files plus `NOTICE.txt`.
- **"Invalid value" on addon load:** Blender may cache an old manifest. Delete the cached addon folder (e.g. `%APPDATA%\Blender Foundation\Blender\5.0\extensions\vscode_development\animaquina` or replace `5.0` with your Blender version) and restart.
- **Panel draw errors** (e.g. error in `ANIMAQUINA_UL_RobotList.draw_item` or in Registry/Setup panels): Ensure the addon is enabled after a full Blender restart and that the addon folder Blender loads contains the latest panel code (with defensive None checks).
- **Connection failures:** Verify the robot's IP is reachable and the correct port is configured. For UR, ensure RTDE is enabled on the controller. For KUKA, the kukaproxydriver KRL program must be running.
- **Slow polling:** Reduce the poll rate or disable polling on robots you're not actively monitoring.

## Building

```bash
python tools/build-addon.py
```

Produces `dist/animaquina-<version>.zip`, installable via **Preferences > Add-ons >
Install from Disk**. The script validates the manifest, parses every module, and
checks nothing stray ends up in the zip.

| Flag | Use |
|---|---|
| `--vendor-py <dir>` | bundle a compiled native dependency folder (see below) |
| `--blender-min X.Y.Z` | build for a different Blender/Python series than the manifest targets |
| `--suffix <name>` | tag the filename, e.g. `--suffix uitest` |
| `--out <dir>` | output directory (default `dist/`) |

Without `--vendor-py` the build is **pure Python**: KUKA and xArm work fully, UR
works on the bundled `urx` backend, and `ur_rtde` features are unavailable.

### Native dependencies (`vendor_py`)

`animaquina/vendor_py/` holds compiled extensions — the patched `ur_rtde`, plus
`paramiko` and its dependencies. It is **not committed**: it must be built per
Python version, and Blender pins one Python per release (4.2–4.5 → 3.11/`cp311`,
5.x → 3.13/`cp313`).

This project's native UR backend uses a separately built `cp313` package for
Blender 5.2. Supply an artifact matching the target interpreter. See [`tools/vendor-build/README.md`](tools/vendor-build/README.md)
— note that the build targets a **python.org** interpreter, because Blender's
bundled Python ships no `Python.h`. The resulting `cp313` `.pyd` loads in Blender
regardless.

`build-addon.py` compares the ABI tags in `--vendor-py` against the manifest's
Blender floor and refuses mismatches, which otherwise fail at import with
`bad magic number`.

Moving these to Blender extension wheels declared in `blender_manifest.toml` is
open work — see [Contributing](#contributing).

## Robot assets

Robot rigs live in [`assets/robots/`](assets/robots) in this repository, but they
are **not bundled into the add-on zip** — at ~70 MB they would bloat every update,
and they carry different licence terms from the code. They ship as a separate
release asset instead.

To use them:

1. Download `animaquina-robots-<version>.zip` from the releases page and unzip it
   anywhere you like — or, if you cloned the repo, just point at `assets/robots/`.
2. **Edit > Preferences > Add-ons > Animaquina**, set **Robot Library Folder** to
   that folder.
3. Click **Register Robot Library**. The rigs then appear in any Asset Browser
   under *Animaquina Robots*, and you can drag them into a scene.

You can also skip the library entirely: bind any armature whose bones are named
`joint_1`..`joint_6` and set the per-joint axis mapping in the Rig panel.

The rigs were built from manufacturer STEP/CAD downloads, exported to mesh,
cleaned up and rigged for this project. That rig work is licensed
**[CC-BY-4.0](LICENSES/CC-BY-4.0.txt)** — attribute it as *"Animaquina robot rigs
by Luis Arturo Pacheco, licensed CC BY 4.0"*. The underlying geometry remains the
manufacturers'; see [THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md).

## Contributing

Contributions are welcome, especially:

- **Robot drivers** — Fanuc, ABB, Staubli, Yaskawa. The driver interface is
  `animaquina_core/drivers/base.py` and is deliberately small.
- **Blender extension wheels** — replacing the runtime pip install and the
  hand-built `vendor_py/` bundle.
- **Path validation** — the current validator has documented gaps; see
  [SAFETY.md](SAFETY.md).
- **Old controllers.** If you have a KRC2, KRC4, or anything else the vendors
  stopped caring about, bug reports from real hardware are the most valuable
  thing you can send.

By contributing you agree your work is licensed GPL-3.0-or-later.

## Licences

Copyright (C) 2026 Luis Arturo Pacheco.

Animaquina is free software under the [GNU General Public License v3.0 or
later](LICENSE). It comes with **ABSOLUTELY NO WARRANTY** — see sections 15 and 16
of the licence, and [SAFETY.md](SAFETY.md).

| What | Licence | Text |
|---|---|---|
| Animaquina add-on and core | GPL-3.0-or-later | [LICENSE](LICENSE) |
| `math3d` (PyMath3D) — Morten Lind | LGPL-3.0 | [`animaquina/libs/math3d/LICENSE`](animaquina/libs/math3d/LICENSE) |
| `urx` — Olivier Roulet-Dubonnet | LGPL-3.0 | [`animaquina/libs/urx/LICENSE`](animaquina/libs/urx/LICENSE) |
| [xArm-Python-SDK](https://github.com/xArm-Developer/xArm-Python-SDK) — UFACTORY, Inc. | BSD-3-Clause | [`animaquina/libs/xarm/LICENSE`](animaquina/libs/xarm/LICENSE) |
| `ur_rtde` (built into `vendor_py/`, not committed) | MIT | see [`tools/vendor-build/`](tools/vendor-build/README.md) |
| `paramiko` and its dependencies (built into `vendor_py/`) | LGPL-2.1 / Apache-2.0 / BSD / MIT | shipped `*.dist-info/LICENSE` |
| Robot rigs and cleanup (separate download) | CC-BY-4.0, over manufacturer CAD | [`LICENSES/CC-BY-4.0.txt`](LICENSES/CC-BY-4.0.txt) |

Full details, including the provenance of the robot geometry, are in
[THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md).

The KUKA driver talks to a variable server (KUKAVARPROXY / C3 Bridge) that runs on
the robot controller. That server is **not** part of Animaquina, is not
distributed here, and carries its own licence — you install it yourself.
