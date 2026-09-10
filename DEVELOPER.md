# Animaquina -- Developer Guide

This document explains the codebase architecture, data flow, and conventions to help contributors understand the implementation. For high-level goals, slot model, and Blender add-on compliance (e.g. Section 25), see **ANIMAQUINA v1.0 .md** at project root.

## Architecture Overview

The addon is organized into three layers. Each layer has strict boundaries:

```
Layer 3 — UI (panels.py, operators.py)
  Draws panels, dispatches operator logic. Never talks to hardware.

Layer 2 — Runtime (runtime/)
  Shared logic: rig updates, conversions, simulation, export, validation.
  Operates on Blender data and slot properties. Never imports drivers.

Layer 1 — Drivers (drivers/)
  Hardware communication only. Never imports bpy or touches Blender data.
```

The **Robot Manager** (`manager.py`) sits between the layers, owning driver lifecycle and the polling bridge that moves data from drivers into Blender properties.

```
┌─────────────────────────────────────────┐
│  UI  (panels.py / operators.py)         │
│  reads slot properties, calls operators │
└────────────────┬────────────────────────┘
                 │ operator execute()
┌────────────────▼────────────────────────┐
│  Manager  (manager.py)                  │
│  owns drivers, poll threads, timer      │
│  connect_slot / disconnect_slot         │
└──┬─────────────────────────────┬────────┘
   │ driver.read_*()             │ rig_apply.apply_full_pose()
   │ driver.move_*()             │ conversions, export, sim
┌──▼──────────┐   ┌─────────────▼─────────┐
│  Drivers    │   │  Runtime               │
│  ur_driver  │   │  rig_apply, simulation │
│  kuka_driver│   │  krl_export, ur_export │
│  xarm_driver│   │  conversions, markers  │
└─────────────┘   └────────────────────────┘
```

## Directory Structure

```
animaquina/
├── __init__.py            # bl_info, register/unregister, persistent handlers
├── properties.py          # All PropertyGroups (Scene, RobotSlot, KukaDebugVar)
├── manager.py             # Robot Manager, poll threads, timer
├── drivers/
│   ├── base.py            # DriverBase ABC + capability flags
│   ├── ur_driver.py       # Universal Robots (RTDE / URX)
│   ├── kuka_driver.py     # KUKA (kukaproxydriver socket)
│   └── xarm_driver.py     # xArm / UFactory SDK
├── runtime/
│   ├── __init__.py        # Runtime package init
│   ├── conversions.py     # Unit helpers: deg↔rad, rpy↔rotation_vector
│   ├── rig_apply.py       # Apply cached state to armature, TCP, base, target
│   ├── simulation.py      # IK rig setup (duplicate, constrain, iTaSC tuning)
│   ├── markers.py         # Add TCP marker empties
│   ├── krl_export.py      # Mesh waypoints → KRL program text
│   ├── ur_export.py       # Mesh waypoints → URScript program text
│   ├── krl_stream.py      # KUKA Dynamic Sync ring-buffer KRL generator
│   ├── kuka_stream_runtime.py  # Runtime handler for Dynamic Sync uploads
│   ├── kuka_krl_parser.py # Parse KRL program text
│   ├── ur_stream.py       # UR RTDE register streaming
│   ├── mujoco_export.py   # MuJoCo model export for Newton validation
│   ├── newton_validator.py # Path collision/IK checking
│   └── newton_worker.py   # Background subprocess for MuJoCo checks
├── ui/
│   ├── __init__.py        # Exports PANEL_CLASSES, OPERATOR_CLASSES
│   ├── panels.py          # All PT_ panel classes
│   └── operators.py       # All OT_ operator classes
├── libs/                  # Bundled third-party robot SDKs
│   ├── urx/               # Legacy UR interface
│   ├── ur_script_python.py
│   ├── kukaproxydriver.py  # KUKA socket interface
│   ├── kuka_krl_python.py  # KRL code generator
│   ├── xarm/              # UFactory xArm SDK
│   └── math3d/            # Transform math, quaternions, interpolation
└── assets/robots/         # Blender asset library (.blend) with robot rigs
```

## Data Model

All addon state lives in a **single PropertyGroup on `bpy.types.Scene`**, accessed as `scene.animaquina`. This follows Blender add-on compliance rules (no stray properties on unrelated types).

## Building native dependencies for a specific Blender/Python (vendor_py)

Animaquina bundles its native runtime dependencies in `animaquina/vendor_py/`, which the
add-on adds to `sys.path` at startup. These are compiled C-extensions (`.pyd`) whose
**ABI is locked to one CPython minor version** (the `cpXYZ` tag in the filename). A folder
built for `cp311` (Blender 5.0) **will not load** under `cp313` (Blender 5.2) — you get
`ImportError: DLL load failed`. So each Blender major that bumps Python needs its own
`vendor_py` rebuilt with the matching interpreter.

Blender → Python mapping:
- Blender 5.0 → Python 3.11 (`cp311`)
- Blender 5.2 beta → Python 3.13 (`cp313`)

Most deps ship `cpXYZ` wheels on PyPI (numpy, mujoco, cryptography, cffi, bcrypt, pynacl,
paramiko, glfw, pyopengl) — rebuild those with a simple `pip install --target`. **The
exception is `ur_rtde`**: it must be **compiled from source** — and not from PyPI.

> **⚠️ ur_rtde is a patched MASTER build — never `pip install ur-rtde` into vendor_py.**
> The latest PyPI release (1.6.3, as of July 2026) cannot control PolyScope X: it
> connects, claims success, and silently no-ops every motion. PolyScope X support
> (direct script upload with the robot in Remote Control mode) only exists on ur_rtde
> master. We additionally carry one local patch (a verbose-logging crash fix in
> `script_client.cpp`, upstream-MR candidate), archived at
> `tools/vendor-build/patches/ur_rtde-animaquina.patch`. Build with
> `tools/vendor-build/build_urrtde_master.bat` from the clone at
> `%USERPROFILE%\animaquina-build\ur_rtde_master`, then validate with
> `tools/vendor-build/smoke-test-urrtde.py` against URSim. Full rationale +
> upgrade checklist: `tools/vendor-build/README.md`.

> Blender's bundled Python has **no** dev headers/libs (`Python.h`, `pythonXYZ.lib`), so you
> cannot link a C-extension against it directly. Build with a **standalone python.org
> CPython of the same minor version** instead — the `cpXYZ` ABI is stable across patch
> releases, so a `.pyd` built against python.org 3.13.x loads fine in Blender's 3.13.y.

### One-time toolchain (Windows)

| Tool | Why | Install |
|------|-----|---------|
| VS 2022 C++ Build Tools (MSVC v143) | compiler/linker | `winget install --id Microsoft.VisualStudio.2022.BuildTools -e --override "--quiet --wait --norestart --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"` |
| CMake ≥ 3.11 | ur_rtde build driver | `winget install --id Kitware.CMake -e` |
| python.org CPython (matching Blender's minor, e.g. 3.13) | provides `Python.h` + `pythonXYZ.lib` | `winget install --id Python.Python.3.13 -e` |
| Boost (static libs: `system`, `thread`, `program_options`) | ur_rtde dependency | build from source, see below |

`ur_rtde` 1.6.3 build requirements (from its `CMakeLists.txt`):
`cmake_minimum_required(VERSION 3.11)`, C++11, `find_package(Boost REQUIRED COMPONENTS
system thread program_options)`, and on Windows `Boost_USE_STATIC_LIBS ON`. pybind11 is
pulled automatically by pip build isolation.

### Build Boost (only the needed components)

```powershell
# from a folder outside the repo, e.g. %USERPROFILE%\animaquina-build
# 1) download + extract boost_1_86_0 source
# 2) bootstrap + build the 3 static libs in a VS x64 dev shell:
& "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
cd boost_1_86_0
.\bootstrap.bat
.\b2 --with-system --with-thread --with-program_options link=static runtime-link=shared `
     threading=multi address-model=64 architecture=x86 -j8 stage
# static libs land in .\stage\lib ; headers are in .\boost
$env:BOOST_ROOT = "<...>\boost_1_86_0"
```

### Build ur_rtde for the target Python

Use the repo script — it builds the **patched master clone**, not PyPI:

```powershell
# clone once (or refresh): includes PolyScope X support absent from 1.6.3
git clone --recursive https://gitlab.com/sdurobotics/ur_rtde.git %USERPROFILE%\animaquina-build\ur_rtde_master
# apply the animaquina patches:
cd %USERPROFILE%\animaquina-build\ur_rtde_master
git apply <repo>\tools\vendor-build\patches\ur_rtde-animaquina.patch
# build the cp313 wheel:
<repo>\tools\vendor-build\build_urrtde_master.bat
# unzip the resulting .whl; it contains the cp313 .pyd files + rtde.dll + urcl/
```

### Assemble and verify vendor_py

1. Build the cp313 wheel deps into a fresh folder (**without** ur_rtde — pip would
   fetch the broken stock release):
   `<blender python.exe> -m pip install --target vendor_py_cp313 paramiko mujoco numpy glfw pyopengl`
   then copy the patched ur_rtde `.pyd`/`rtde.dll`/`urcl/`/`ur_rtde-*.dist-info` from the
   master wheel (`animaquina-build\urrtde_wheel_master\`) into the folder.
2. Keep the cp313 build as a folder (e.g. `vendor_py_cp313`), keeping each package's
   `*.dist-info/LICENSE` — ur_rtde is MIT, redistribution is fine.
3. Verify under Blender's interpreter:
   `<blender python.exe> -c "import sys; sys.path.insert(0,'vendor_py_cp313'); import rtde_control, rtde_receive, paramiko; print('ok')"`
4. **Ship it without touching the repo's cp311 folder** — point your packaging step at
   the new folder when staging the add-on zip:
   ```powershell
   Copy-Item -Recurse <...>endor_py_cp313 animaquinaendor_py
   python -c "import shutil; shutil.make_archive('dist/animaquina','zip','.','animaquina')"
   ```

Ready-to-run build scripts + a step-by-step runbook live in `tools/vendor-build/`
(`build_boost.bat`, `build_urrtde.bat`, `README.md`). The toolchain/Boost/ur_rtde
compile is **one-time per Python version** — once `vendor_py_cp313` exists, new zips
are just the one command above.

> Do **not** ship a `cpXYZ`-mismatched `vendor_py` (e.g. the cp311 folder in a Blender 5.2
> build). On the path under the wrong interpreter it shadows Blender's own numpy and breaks
> unrelated add-ons (`No module named 'numpy._core._multiarray_umath'`).

### Key Types

**`ANIMAQUINA_SceneProperties`** (on `Scene.animaquina`)
- `robots` -- `CollectionProperty` of `ANIMAQUINA_RobotSlot`
- `active_robot_index` -- index of the selected slot
- `poll_rate_hz` -- global polling frequency
- `global_debug` -- optional debug flag

**`ANIMAQUINA_RobotSlot`** -- one per robot. Contains everything:
- **Identity:** `uid`, `label`, `robot_type` (UR/KUKA/XARM), model enum
- **Rig references:** `rig_armature`, `rig_collection`, `base_object`, `tcp_object`, `tool_object`, `target_object`, `sim_collection`, `collision_collection`
- **Connection:** `endpoint`, `port`, `is_connected`, `polling_enabled`, `last_error`
- **Motion settings:** `move_mode` (LINEAR/PTP), `speed`, `acc`, `radius`, joint parameters
- **Runtime cache:** `tcp_pos_m`, `tcp_euler_rad`, `tcp_euler_deg`, `joints_deg`, `base_pos_m`, `base_euler_rad` -- always in canonical units
- **Export settings:** per-robot-type velocity, acceleration, tool/payload, staging config
- **UI state:** `ui_ctrl_show_*` booleans for collapsible panel sections
- **Joint axis map:** `joint_axis_0`..`joint_axis_5` enums (X/Y/Z/-X/-Y/-Z)

**`ANIMAQUINA_DebugVar`** -- items in a slot's debug variable list (works with all robot types).

### Canonical Units

All cached values and inter-layer communication use these units:
- **Position:** meters
- **Orientation:** radians (XYZ Euler order)
- **Joints:** degrees
- **Display:** the Info panel converts to human-readable (mm, deg) for display only

Robot-specific conversions (e.g., KUKA ZYX→XYZ, mm→m) happen **inside the driver**, never in runtime or UI code.

## Robot Manager & Threading

`manager.py` manages the full driver lifecycle:

### Connection Flow
```
connect_slot(slot)
  1. create_driver(slot.robot_type) → DriverBase subclass
  2. driver.connect(endpoint, port)
  3. Start _SlotPollThread (daemon) for this slot
  4. Register Blender timer (_poll_all) if not already running
  5. Set slot.is_connected = True
```

### Polling Architecture

Each connected slot has a **`_SlotPollThread`** (daemon thread):
- Calls `driver.read_tcp()`, `driver.read_joints()`, `driver.read_base()` in a loop
- Writes results to `_thread_cache` dict under a threading lock
- Sleeps based on `poll_rate_hz`
- **Never touches bpy** -- threads cannot safely access Blender data

The **main-thread timer** `_poll_all()` runs at ~20-50ms intervals:
- Reads from `_thread_cache` under lock (fast, no I/O)
- Writes values to slot properties (`tcp_pos_m`, `joints_deg`, etc.)
- Calls `rig_apply.apply_full_pose(slot)` to update Blender objects
- Tags view layer for redraw

### Disconnection
```
disconnect_slot(slot)
  1. Signal _SlotPollThread to stop → join thread
  2. driver.disconnect()
  3. Reset rig to rest pose
  4. Unregister timer if no connected slots remain
  5. Set slot.is_connected = False
```

## Driver Interface

All drivers extend `DriverBase` from `drivers/base.py`.

### Capability Flags

Drivers declare which features they support via bitwise capability flags:

```python
CAP_CONNECT        = 1 << 0    # Connect/disconnect
CAP_READ_TCP       = 1 << 1    # Read TCP pose
CAP_READ_JOINTS    = 1 << 2    # Read joint angles
CAP_READ_BASE      = 1 << 3    # Read base frame
CAP_MANUAL_MODE    = 1 << 4    # Freedrive / teach mode
CAP_MOVE_TO_TARGET = 1 << 5    # Single-pose motion command
CAP_EXECUTE_PATH   = 1 << 6    # Multi-waypoint path execution
CAP_EXPORT_PROGRAM = 1 << 7    # Offline program generation
CAP_HOME           = 1 << 8    # Go-home command
CAP_UPLOAD_PROGRAM = 1 << 9    # Upload to controller filesystem
CAP_SELECT_PROGRAM = 1 << 10   # Load/select program on controller
CAP_RESET          = 1 << 11   # Clear errors / reset state
```

The UI uses these flags to **hide buttons** for unsupported features. Operators check driver capabilities in their `poll()` methods.

### Required Methods

```python
# Core — every driver must implement these
def capabilities(self) -> int                            # Bitmask of CAP_* flags
def connect(self, endpoint, port=0, *, slot=None) -> str
def disconnect(self) -> None
def read_tcp(self) -> tuple[list[float], list[float]]    # (pos_m[3], euler_rad[3])
def read_joints(self) -> tuple[float, ...]               # (j1_deg, ..., j6_deg)
def read_base(self) -> tuple[list[float], list[float]]   # (pos_m[3], euler_rad[3])

# Motion
def move_to_pose(self, pos_m, euler_rad, speed, acc, radius, wait) -> str
def move_to_pose_ptp(self, pos_m, euler_rad, vel, acc, radius, wait) -> str
def execute_ptp_path(self, waypoints, speed, acc, radius, wait) -> str

# Home & reset
def read_home_joints(self) -> tuple[float, ...]          # (j1_deg, ..., j6_deg)
def write_home_joints(self, joints_deg) -> str
def go_home(self, joints_deg=None, vel=0.5, acc=0.5) -> str
def reset(self) -> str                                   # Clear errors / safe state

# Teach mode
def set_manual_mode(self, enabled: bool) -> str

# Program management
def send_program(self, content: str) -> str              # Immediate execution
def upload_program(self, content: str, program_name: str) -> str
def select_program(self, program_name: str) -> str
```

Each method returns a status string (empty on success, error message on failure). Drivers convert between robot-native units and canonical units internally. Override only the methods matching your declared `CAP_*` flags; the base class defaults return `"Not implemented"`.

### Adding a New Robot -- Step-by-Step

This walkthrough uses a hypothetical **ABB** robot as an example. Every touch-point is listed so nothing is missed.

#### Naming Conventions

Follow the existing pattern exactly:

| Thing | Convention | Example |
|-------|-----------|---------|
| Robot type enum value | `UPPERCASE` short name | `"ABB"` |
| Driver class | `<Brand>Driver` | `ABBDriver` |
| Driver file | `<brand>_driver.py` | `abb_driver.py` |
| Model enum items list | `<BRAND>_MODEL_ITEMS` | `ABB_MODEL_ITEMS` |
| Model property on slot | `<brand>_model` | `abb_model` |
| Default axes constant | `<BRAND>_DEFAULT_AXES` | `ABB_DEFAULT_AXES` |
| Axis map dict | `<BRAND>_AXIS_MAPS` | `ABB_AXIS_MAPS` |
| Set-axes operator class | `ANIMAQUINA_OT_Set<Brand>Axes` | `ANIMAQUINA_OT_SetABBAxes` |
| Set-axes operator ID | `object.animaquina_set_<brand>_axes` | `object.animaquina_set_abb_axes` |
| Export module (if needed) | `runtime/<brand>_export.py` | `runtime/abb_export.py` |
| Brand-specific properties | Prefix with `<brand>_` | `abb_speed_override` |
| Brand-specific sub-panel | `ANIMAQUINA_PT_<Brand>Settings` | `ANIMAQUINA_PT_ABBSettings` |

#### Step 1 -- properties.py

Add the type and model enums at the top of the file, alongside the existing ones:

```python
# After XARM_MODEL_ITEMS:
ABB_MODEL_ITEMS = [
    ("ABB_GENERIC", "Generic ABB", ""),
    ("IRB_1200", "IRB 1200", ""),
    ("IRB_6700", "IRB 6700", ""),
]
```

Add `"ABB"` to the `ROBOT_TYPE_ITEMS` list:

```python
ROBOT_TYPE_ITEMS = [
    ("UR", "Universal Robots", ""),
    ("KUKA", "KUKA", ""),
    ("XARM", "xArm / UF", ""),
    ("ABB", "ABB", ""),              # ← new
]
```

Add the model property inside `ANIMAQUINA_RobotSlot`:

```python
abb_model: EnumProperty(
    name="ABB Model",
    items=ABB_MODEL_ITEMS,
    default="ABB_GENERIC",
)
```

Add any brand-specific properties the driver or UI will need (connection options, speed overrides, etc.), prefixed with `abb_`.

#### Step 2 -- drivers/abb_driver.py

Create the driver file. The driver **must not import bpy**. It only talks to hardware and converts to/from canonical units.

```python
from .base import DriverBase, CAP_CONNECT, CAP_READ_TCP, CAP_READ_JOINTS, ...

class ABBDriver(DriverBase):
    _CAPS = CAP_CONNECT | CAP_READ_TCP | CAP_READ_JOINTS | ...

    def capabilities(self):
        return self._CAPS

    def connect(self, endpoint, port=0, *, slot=None) -> str:
        # Connect to the ABB controller (e.g. via Robot Web Services or EGM)
        # Return "" on success, error string on failure

    def disconnect(self) -> None:
        # Clean up connection

    def read_tcp(self) -> tuple:
        # Read current TCP from controller
        # Convert from ABB native units to canonical: (pos_m[3], euler_rad_xyz[3])

    def read_joints(self) -> tuple:
        # Read joint angles, convert to degrees
        # Return (j1_deg, j2_deg, ..., j6_deg)

    # Implement other methods matching your declared CAP_* flags
```

**Key rule:** all unit conversions (mm→m, ABB quaternion→XYZ Euler, etc.) happen here, not in runtime or UI.

#### Step 3 -- manager.py

Add the import and the factory branch in `create_driver()`:

```python
from .drivers.abb_driver import ABBDriver

def create_driver(robot_type: str) -> DriverBase:
    if robot_type == "UR":
        return URDriver()
    if robot_type == "KUKA":
        return KUKADriver()
    if robot_type == "XARM":
        return XArmDriver()
    if robot_type == "ABB":             # ← new
        return ABBDriver()
    return None
```

That's it for the manager -- polling, threading, and rig updates are robot-agnostic and work automatically.

#### Step 4 -- ui/operators.py (axis presets)

Add the axis map constants and the Set Axes operator. Determine the correct axis map by testing with your robot's Blender rig (which rotation channel of each bone matches the physical joint).

```python
ABB_DEFAULT_AXES = ("Y", "-Z", "-Z", "Y", "-Z", "Y")  # ← adjust per your rig

ABB_AXIS_MAPS = {
    "ABB_GENERIC": ABB_DEFAULT_AXES,
    "IRB_1200": ABB_DEFAULT_AXES,
    "IRB_6700": ABB_DEFAULT_AXES,
}

class ANIMAQUINA_OT_SetABBAxes(Operator):
    bl_idname = "object.animaquina_set_abb_axes"
    bl_label = "Set ABB Axes"
    bl_description = "Set joint axis map to ABB default"
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        return get_active_slot(context) is not None

    def execute(self, context):
        slot = get_active_slot(context)
        if slot is None:
            return {"CANCELLED"}
        model = getattr(slot, "abb_model", "ABB_GENERIC")
        axes = ABB_AXIS_MAPS.get(model, ABB_DEFAULT_AXES)
        for i, axis in enumerate(axes):
            setattr(slot, f"joint_axis_{i}", axis)
        self.report({"INFO"}, f"Joint axes set for {model}")
        return {"FINISHED"}
```

Add the class to the `OPERATOR_CLASSES` list at the bottom of the file.

#### Step 5 -- ui/panels.py (model selector + axis button)

In `ANIMAQUINA_PT_setup.draw()`, add a block for the new type alongside the existing ones:

```python
if slot.robot_type == "ABB":
    col.prop(slot, "abb_model", text="Model")
    col.operator("object.animaquina_set_abb_axes", text="Set ABB axes (model)")
```

If the robot needs brand-specific connection UI, export settings, or control options, add conditional blocks gated on `slot.robot_type == "ABB"` in the relevant panels -- or create a sub-panel with `bl_parent_id` pointing to the appropriate parent.

#### Step 6 -- Runtime (optional)

If the robot needs its own export format (e.g. RAPID for ABB) or streaming protocol:
- Add `runtime/abb_export.py` following the pattern of `krl_export.py` / `ur_export.py`
- Add an export operator in `operators.py` gated on `slot.robot_type == "ABB"`
- The export module reads mesh waypoints, converts frames via `world_waypoints_to_base_frame()` or `world_waypoints_to_blender_base_frame()`, and generates program text

#### Step 7 -- Test

1. Load the addon, add a slot, select the new robot type
2. Verify the model selector and axis preset button appear
3. Connect to the robot (or a simulator) and confirm TCP/joint polling works
4. Test Move to Target, Run Toolpath, and export if implemented
5. Verify the Info panel displays correct values in canonical units

#### Checklist

- [ ] `ROBOT_TYPE_ITEMS` has the new entry
- [ ] `<BRAND>_MODEL_ITEMS` defined
- [ ] `<brand>_model` property on `ANIMAQUINA_RobotSlot`
- [ ] Driver class in `drivers/<brand>_driver.py` with correct `capabilities()`
- [ ] `create_driver()` handles the new type
- [ ] Axis constants + `Set<Brand>Axes` operator + registered in `OPERATOR_CLASSES`
- [ ] `panels.py` shows model selector and axis button for the new type
- [ ] Any brand-specific properties prefixed with `<brand>_`
- [ ] Driver converts all values to/from canonical units internally
- [ ] Driver does NOT import `bpy`

## UR RTDE Variable Interface — Implementation Notes

This section documents the correct ur_rtde methods for reading and writing variables during live streaming (servoL) and export. Verified against the [ur_rtde 1.6.3 API](https://sdurobotics.gitlab.io/ur_rtde/api/api.html).

### RTDE Interfaces (3 separate connections)

| Interface | Class | Purpose |
|-----------|-------|---------|
| **Control** | `RTDEControlInterface` | Motion (servoL, moveL, moveJ), sendCustomScript |
| **Receive** | `RTDEReceiveInterface` | Read robot state, registers, I/O |
| **IO** | `RTDEIOInterface` | Write digital outputs, analog outputs, input registers, speed slider |

### RTDEIOInterface — Available Methods

Confirmed via `dir()` on live connection:

```
disconnect, reconnect, isConnected,
setStandardDigitalOut, setConfigurableDigitalOut, setToolDigitalOut,
setAnalogOutputVoltage, setAnalogOutputCurrent,
setInputIntRegister, setInputDoubleRegister,
setSpeedSlider
```

### Reading Variables (RTDEReceiveInterface)

| Variable pattern | Method | Notes |
|-----------------|--------|-------|
| `output_int_register_N` | `recv.getOutputIntRegister(N)` | N = 0-47, works |
| `output_double_register_N` | `recv.getOutputDoubleRegister(N)` | N = 0-47, works |
| `input_int_register_N` | fallback to cached value | `recv.getInputIntRegister` may not exist; returns last written value or "0" |
| `input_double_register_N` | fallback to cached value | Same as above |
| `standard_digital_input_N` | `recv.getDigitalInState(N)` | N = 0-7 |
| `standard_digital_output_N` | `recv.getDigitalOutState(N)` | N = 0-7 |
| `configurable_digital_output_N` | `recv.getActualDigitalOutputBits()` | Bit 8+N from output bitmask |
| `configurable_digital_input_N` | `recv.getActualDigitalInputBits()` | Bit 8+N from input bitmask |
| `tool_digital_output_N` | `recv.getActualDigitalOutputBits()` | Bit 16+N from output bitmask |
| `tool_digital_input_N` | `recv.getActualDigitalInputBits()` | Bit 16+N from input bitmask |
| `standard_analog_input_N` | `recv.getStandardAnalogInputN()` | Per-pin method, no argument |
| `standard_analog_output_N` | `recv.getStandardAnalogOutputN()` | Per-pin method, no argument |
| `robot_mode` | `recv.getRobotMode()` | |
| `safety_mode` | `recv.getSafetyMode()` | |

**Important:** Analog read methods are per-pin with NO parameter: `getStandardAnalogOutput0()`, `getStandardAnalogOutput1()`. There is no `getStandardAnalogOutput(pin)`.

**Digital output bitmask layout** (`getActualDigitalOutputBits()`):
- Bits 0-7: standard digital outputs
- Bits 8-15: configurable digital outputs
- Bits 16-17: tool digital outputs

### Writing Variables — What Works Where

| Variable pattern | IO method | Export (URScript) | Buffered streaming | Direct write |
|-----------------|-----------|------|---------|------|
| `standard_digital_output_N` | `io.setStandardDigitalOut(N, bool)` | `set_standard_digital_out` | **yes** | yes |
| `configurable_digital_output_N` | `io.setConfigurableDigitalOut(N, bool)` | `set_configurable_digital_out` | **yes** | yes |
| `tool_digital_output_N` | `io.setToolDigitalOut(N, bool)` | `set_tool_digital_out` | **yes** | yes |
| `standard_analog_output_N` | `io.setAnalogOutputVoltage(N, V)` | `set_standard_analog_out` | **yes** | yes |
| `tool_analog_output_N` | — | `set_tool_analog_out` | no | no |
| `output_int_register_N` | — | `write_output_integer_register` | **no** (needs sendCustomScript) | yes (sendCustomScript) |
| `output_double_register_N` | — | `write_output_float_register` | **no** (needs sendCustomScript) | yes (sendCustomScript) |
| `input_int_register_N` | `io.setInputIntRegister(N, val)` | `write_input_integer_register` | writes succeed but **no visible effect** | yes |
| `input_double_register_N` | `io.setInputDoubleRegister(N, val)` | `write_input_float_register` | writes succeed but **no visible effect** | yes |

**Why registers don't work during streaming:**
- **Output registers** (`output_int_register_N`): Can only be written via `sendCustomScript` (inline URScript). `sendCustomScript` kills the active servoL control program — cannot use during streaming.
- **Input registers** (`input_int_register_N`): The RTDE write succeeds, but input registers are "external → robot" — they're only useful when a URScript program calls `read_input_integer_register(N)`. During servoL streaming, no such program is running, so the written values have no visible effect on the teach pendant or robot behavior.

**Recommended for streaming:** `standard_digital_output_N`, `configurable_digital_output_N`, `tool_digital_output_N`, `standard_analog_output_N`

### Methods That Do NOT Exist on RTDEIOInterface

- ~~`setStandardAnalogOut`~~ — use `setAnalogOutputVoltage` or `setAnalogOutputCurrent`
- ~~`setAnalogOutputDomain`~~ — voltage vs current is selected by which method you call
- ~~`setOutputIntRegister`~~ — output registers are robot → external, no write method

### Thread Safety During servoL Streaming

The servo worker thread calls `ctrl.servoL()` at ~125Hz. The RTDE C++ bindings are **not thread-safe**. All writes during streaming must go through a `var_queue` (thread-safe `queue.Queue`):

```
Main Thread (Blender modal)              Servo Worker Thread (125Hz)
───────────────────────────              ──────────────────────────
driver.write_var(name, val)
  → enqueues (name, val)                 → drains var_queue each cycle
     into state["var_queue"]             → dispatch_streaming_write(ctrl, io, name, val)
  → returns "" immediately                  → io.setStandardDigitalOut(N, val)
                                            → io.setConfigurableDigitalOut(N, val)
                                            → io.setToolDigitalOut(N, val)
                                            → io.setAnalogOutputVoltage(N, val)
```

**Rules:**
1. Never call `ctrl` methods from the main thread during streaming
2. Never call `sendCustomScript` during streaming (kills servoL)
3. All IO writes are routed through the servo thread via var_queue for safety
4. Variable values are dispatched per source waypoint using `waypoints_completed`, not per servoL substep

### Export — URScript Functions

In exported URScript programs, variable writes use URScript built-in functions (not RTDE):

| Variable pattern | URScript function |
|-----------------|-------------------|
| `output_int_register_N` | `write_output_integer_register(N, val)` |
| `output_double_register_N` | `write_output_float_register(N, val)` |
| `input_int_register_N` | `write_input_integer_register(N, val)` |
| `input_double_register_N` | `write_input_float_register(N, val)` |
| `standard_digital_output_N` | `set_standard_digital_out(N, val)` |
| `configurable_digital_output_N` | `set_configurable_digital_out(N, val)` |
| `tool_digital_output_N` | `set_tool_digital_out(N, val)` |
| `standard_analog_output_N` | `set_standard_analog_out(N, val)` — normalized 0.0-1.0 |
| `tool_analog_output_N` | `set_tool_analog_out(N, val)` — normalized 0.0-1.0 |

Export also emits `set_analog_outputdomain(pin, 1)` at program start to set voltage mode for analog outputs.

### Analog Output Values

- **RTDE IO (streaming/direct):** `setAnalogOutputVoltage(pin, voltage)` — value is in volts (0.0-10.0)
- **URScript (export):** `set_standard_analog_out(pin, ratio)` — value is normalized (0.0-1.0, maps to 0-10V in voltage domain)
- **Reading:** `getStandardAnalogOutputN()` returns the normalized ratio (0.0-1.0)

## Future Work

### URDF Import

Currently, adding a new robot requires manually creating the Blender armature with correctly named bones (`joint_1`..`joint_6`) and determining the joint axis map by trial and error. A URDF importer would automate this:

- Parse the URDF XML to extract joint names, types, axes, and limits
- Build the Blender armature automatically with correct bone lengths and rest positions
- Auto-generate the joint axis map from `<axis xyz="..."/>` tags
- Import visual meshes (STL/DAE) and parent them to the corresponding bones
- Store joint limits for runtime safety checks

This would make the addon truly extensible -- any robot with a URDF file could be set up without manual rig work.

## Coordinate Frames & Base-to-Tool Conversions

This is the most important spatial concept in the addon. Every robot's TCP and target are positioned **relative to its J0 (base object)** in Blender, mirroring how real robots report poses relative to their base frame.

### Scene Object Hierarchy

The expected Blender object hierarchy for each robot slot:

```
Blender Base (Empty)          ← J0's parent; represents machine zero / reference frame
  └── J0 / base_object       ← the robot's physical base mesh; driven by $BASE (KUKA) or static
        └── (armature)        ← joint bones rotate here
TCP (Empty)                   ← positioned in world by converting base-relative pose
  └── Tool (mesh, optional)   ← attached via Child Of constraint to TCP
Target (Empty)                ← user-placed; converted to base frame before sending to robot
```

### How TCP Positioning Works

Drivers return TCP pose **in the robot's base frame** (relative to J0) -- all robots work this way:
- UR: `getActualTCPPose()` returns pose relative to the robot's base
- KUKA: `$POS_ACT` is relative to `$BASE`
- xArm: `get_position()` returns pose relative to base

The function `apply_full_pose()` in `rig_apply.py` converts this to Blender world space:

```
1. Apply J0 position first (from $BASE or manual placement)
2. Find the TCP's reference frame:
   - If J0 has a parent → use J0's parent (the "Blender Base" empty)
   - Otherwise → use J0 itself
3. Build TCP matrix = reference_frame.matrix_world @ tcp_in_base_matrix
4. Set tcp_object.location and .rotation_euler from the resulting world matrix
```

**Why J0's parent?** On KUKA robots, J0 is driven by `$BASE` (which may include an offset from machine zero). The TCP position the robot reports already accounts for `$BASE`, so applying TCP relative to J0 would double-count the offset. Using J0's parent (the static "Blender Base" empty at machine zero) avoids this.

### Sending Positions to the Robot

When the user clicks **Move to Target** or **Run Toolpath**, world-space positions must be converted back to the robot's base frame:

```python
# In rig_apply.py:
def world_waypoints_to_base_frame(slot, waypoints_world):
    base_inv = get_slot_base_world_matrix(slot).inverted()
    # For each waypoint: base_frame_pose = base_inv @ world_pose
```

`get_slot_base_world_matrix(slot)` returns J0's world matrix (scale-stripped). This is the live robot base -- positions converted through this matrix match what the driver expects.

### Export Frame (KRL / URScript)

Offline export uses a **different reference frame**: the Blender Base (J0's parent), not J0 itself. This makes exported programs independent of the robot's runtime `$BASE` setting:

```python
def world_waypoints_to_blender_base_frame(slot, waypoints_world):
    base_inv = get_slot_blender_base_world_matrix(slot).inverted()
    # Blender Base = J0's parent, or J0 itself if no parent
```

### Frame Summary

| Frame | What it is | Used for |
|-------|-----------|----------|
| **World** | Blender world origin | Internal Blender positioning of empties/meshes |
| **J0 (base_object)** | Robot base, driven by `$BASE` | Live motion commands (`move_to_pose`, `execute_path`) |
| **Blender Base** | J0's parent empty (machine zero) | Offline program export (KRL, URScript) |
| **Armature** | Rig armature origin | Newton/MuJoCo validation (model exported in armature space) |

### Joint Axis Mapping

Each robot model has different joint rotation conventions. The slot stores a per-joint axis map (`joint_axis_0`..`joint_axis_5`) with values like `X`, `Y`, `Z`, `-X`, `-Y`, `-Z`. When applying joint angles to the armature:

- The axis determines which `rotation_euler` channel to write (0=X, 1=Y, 2=Z)
- A `-` prefix negates the angle before applying
- Preset buttons (e.g., "Set UR Axes", "Set KUKA Axes") configure the correct map for each robot family

### Unit Conversions in Drivers

All robot-specific unit conversions happen **inside the driver**, never in runtime or UI:

| Robot | Native TCP units | Native joint units | Driver converts to |
|-------|-----------------|-------------------|-------------------|
| UR | meters, rotation vector (rad) | radians | m, XYZ euler (rad), degrees |
| KUKA | mm, ABC euler (ZYX, deg) | degrees (A1-A6) | m, XYZ euler (rad), degrees |
| xArm | mm, RPY (deg) | degrees | m, XYZ euler (rad), degrees |

### Offline Base & Tool Offsets

When the robot is **not connected** (offline mode), the user can manually set base and tool offsets in the Info panel. This mirrors the joint sliders that are already editable offline.

**Properties involved:**

| Property | Online (connected) | Offline |
|----------|-------------------|---------|
| `joints_deg` | Read-only, driven by poll thread | Editable, applies FK via `apply_full_pose()` |
| `base_pos_m` / `base_euler_deg` | Read-only, driven by `$BASE` | Editable, moves J0 via `apply_full_pose()` |
| `tool_frame_pos_m` / `tool_frame_euler_deg` | Read-only, driven by `$TOOL` | Editable, used by simulation & export |

**Update callbacks** (`properties.py`):
- `_update_base_offset_offline` -- on `base_pos_m` change: sets `base_frame_valid = True`, calls `apply_full_pose()`. Guarded by `if slot.is_connected: return`.
- `_update_base_euler_deg_offline` -- on `base_euler_deg` change: syncs degrees → `base_euler_rad`, then same as above. Has a reentrancy guard (`_BASE_EULER_DEG_GUARD`) to prevent circular updates.
- `_update_tool_frame_offline` -- on `tool_frame_pos_m` or `tool_frame_euler_deg` change: sets `tool_frame_valid = True`.

When connected, the poll thread overwrites these properties each cycle, and the `is_connected` guard prevents the offline callbacks from doing extra work.

**Tool offset in simulation (KUKA axis convention):**

The tool frame values are stored in **KUKA flange convention** (same format as `$TOOL`):
- Position: X, Y, Z in meters, where **Z is along the tool direction** (perpendicular to flange, pointing outward)
- Rotation: A, B, C in degrees, **KUKA intrinsic ZYX** (not Blender XYZ)

Both the Blender IK simulation (`simulation.py`) and MuJoCo export (`mujoco_export.py`) need to convert these values to **bone-local space**, where the bone **Y axis** is along the bone (= tool direction):

```
KUKA flange frame        Blender bone-local
  X (lateral)       →      X
  Y (perpendicular) →     -Z
  Z (tool direction) →     Y

Remapping matrix P:
  P = [[1, 0, 0],
       [0, 0, 1],
       [0,-1, 0]]

Position:  bone_local = (kuka_x, kuka_z, -kuka_y)
Rotation:  R_bone = P @ R_kuka @ P^T
           where R_kuka = Rz(A) @ Ry(B) @ Rx(C)  (intrinsic ZYX)
```

**Blender IK** (`simulation.py`, `setup_simulation`):
1. `_tool_frame_active` is True when `tool_frame_valid` and any values are non-zero
2. `has_tool` now includes `_tool_frame_active` (in addition to `slot.has_tool` and physical tcp bone offset)
3. After `armature_apply`, enters edit mode on the sim armature and repositions the tcp bone: head at flange + remapped position, tail direction from remapped rotation
4. IK is placed on tcp bone with `chain_count=7` so the solver drives the tool tip to the target

**Newton / MuJoCo** (`mujoco_export.py`):
1. Same `_tool_frame_active` check
2. `site_pos` is computed as flange position (in bone-local space) + remapped tool offset
3. `site_quat` is computed from the remapped rotation matrix when no `tcp_object` orientation is available

## Runtime Modules

### rig_apply.py -- Rig Binding

The core function is `apply_full_pose(slot)`:
1. Reads cached values from slot properties (`tcp_pos_m`, `joints_deg`, etc.)
2. Rotates armature bones per the slot's joint axis map
3. Positions the TCP empty relative to the base object
4. Updates the base object if `base_source == FROM_ROBOT`
5. Tags the view layer for redraw

**Frame conversion helpers:**
- `get_slot_base_world_matrix(slot)` -- the robot's J0 world matrix
- `world_waypoints_to_base_frame(slot, waypoints)` -- convert world-space waypoints into the robot's base frame (for sending to the driver)
- `world_waypoints_to_blender_base_frame(slot, waypoints)` -- convert into the Blender Reference Base frame (for KRL export, which is independent of `$BASE`)

### conversions.py

- `deg2rad(d)` / `rad2deg(r)` -- angle conversion
- `rpy2rv(roll, pitch, yaw)` / `rv2rpy(rx, ry, rz)` -- Euler ↔ rotation vector (needed for UR communication)

### simulation.py

`setup_ik_simulation(slot)`:
1. Duplicates the robot armature into `sim_collection`
2. Adds an IK constraint to the last bone targeting the slot's target empty
3. Locks IK axes to match the slot's joint axis map
4. Configures iTaSC solver parameters (SDLS, feedback, substeps)

### krl_export.py / ur_export.py

Both follow the same pattern:
1. Get the selected mesh object and its **evaluated** mesh (after modifiers / Geometry Nodes).
2. Read waypoints from the mesh **`position`** float attribute (required; see project rule "Toolpath / Send Path" and old addons in `oldversions/`).
3. Convert world-space points to the appropriate robot frame
4. Generate program text using the bundled code generators (`kuka_krl_python` / `ur_script_python`)
5. Store the result in a Blender text block

### Streaming Modules

Both KUKA and UR support real-time waypoint streaming via ring-buffer protocols, allowing Blender to feed waypoints to the robot during execution rather than uploading a complete program upfront.

- `krl_stream.py` -- generates the KRL wrapper program for KUKA Dynamic Sync (ring-buffer protocol). The generated program runs on the controller and reads waypoints from a shared array (`MQ_PT[]`) that Blender fills via socket writes.
- `kuka_stream_runtime.py` -- runtime handler that feeds waypoints into the ring buffer during execution. Monitors the controller's read index and writes ahead to keep the buffer populated.
- `ur_stream.py` -- UR RTDE register-based streaming for real-time waypoint injection. Uses RTDE input registers to pass waypoint data and motion parameters to a running URScript program on the controller.

### Puppet Mode

Puppet Mode is a modal operator (`StartRealTimePuppet` / `StopRealTimePuppet`) that continuously sends the target empty's position to the robot at a configurable rate. Key properties on the slot:

- `realtime_puppet_rate_hz` -- update frequency (1–250 Hz)
- `realtime_puppet_max_step_mm` -- safety limit; jumps larger than this are ignored
- `kuka_puppet_speed_pct` -- KUKA PTP speed % for puppet loop (separate from `kuka_ptp_speed_pct` used for regular moves), written to `MQ_PUPPET_SPEED` before entering the puppet loop
- `xarm_puppet_use_boundary` / `xarm_puppet_boundary_mm` -- optional TCP boundary for xArm

The modal operator runs on Blender's main thread via a timer, reads the target empty's world position each tick, converts it to the robot's base frame, and sends a motion command to the driver.

#### UR servoL tuning (not exposed in UI — defaults are good)

UR puppet mode calls `servoL(target, speed, acc, dt=0.008, lookahead=0.1, gain=300)`.  
These defaults work well in practice and were intentionally **not** exposed in the UI to avoid clutter.  
If jerkiness is observed on UR in the future, the approach to re-expose them is:

1. **`animaquina/properties.py`** — add three `FloatProperty` fields to `AnimaquinaRobotSlot`:
   ```python
   servo_dt: FloatProperty(name="Servo dt (s)", default=0.008, min=0.004, max=0.032)
   servo_lookahead: FloatProperty(name="Lookahead (s)", default=0.1, min=0.03, max=0.2)
   servo_gain: FloatProperty(name="Gain", default=300.0, min=100.0, max=2000.0)
   ```
2. **`animaquina/ui/panels.py`** — inside `if slot.ui_ctrl_show_puppet_settings` add:
   ```python
   if slot.robot_type == "UR":
       row = settings_box.row(align=True)
       row.prop(slot, "servo_lookahead", text="Lookahead (s)")
       row.prop(slot, "servo_gain", text="Gain")
       settings_box.prop(slot, "servo_dt", text="Servo dt (s)")
   ```
3. **`animaquina_core/drivers/ur_driver.py`** — add `_servo_dt/lookahead/gain` to `_RtdeBackend.__init__`, update `realtime_puppet_start(boundary_mm, servo_dt, servo_lookahead, servo_gain)` to store them, and replace hardcoded values in `_servo_call` inside `realtime_puppet_step`.
4. **`animaquina/ui/operators.py`** — before `driver.realtime_puppet_start(...)`, read the three slot properties for UR and pass as kwargs.

Tuning guidance: `lookahead` 0.1–0.2 s (longer = smoother, more lag); `gain` 100–300 (lower = smoother); `dt` should match actual Blender update rate (0.008 at 125 Hz, 0.016 at ~60 fps).

## UI Structure

### Panels (panels.py)

Panels are registered in the 3D Viewport sidebar under the "Animaquina" tab. Order:

1. **ANIMAQUINA_PT_Registry** (`bl_order=0`) -- robot slot list (UIList), add/remove
2. **ANIMAQUINA_PT_Setup** (`bl_order=1`) -- robot config, with sub-panels for Connection, Scene Objects, and driver-specific settings
3. **ANIMAQUINA_PT_Control** (`bl_order=2`) -- motion commands, puppet mode, collapsible motion/streaming settings
4. **ANIMAQUINA_PT_Newton** (`bl_order=3`) -- path validation (Newton/MuJoCo), results sub-panel
5. **ANIMAQUINA_PT_Export** (`bl_order=4`) -- program generation, staging, remote execution
6. **ANIMAQUINA_PT_Info** (`bl_order=5`) -- read-only diagnostics
7. **ANIMAQUINA_PT_Debug** (`bl_order=6`) -- debug/dependency helpers (when `global_debug` enabled)

### Operators (operators.py)

All operators use the `ANIMAQUINA_OT_` prefix. Each operator:
- Has a `poll()` that checks `slot.is_connected` and relevant capability flags
- Calls the manager or runtime functions in `execute()`
- Returns `{'FINISHED'}` or `{'CANCELLED'}` with an error report

Key operators:
- `AddSlot` / `RemoveSlot` -- slot management
- `ConnectRobot` / `DisconnectRobot` -- driver lifecycle via manager
- `UpdatePose` -- one-shot sync from robot
- `MoveToTarget` -- single-pose motion
- `SendPath` -- path execution from mesh/curve
- `StartRealTimePuppet` / `StopRealTimePuppet` -- puppet mode
- `ExportKRL` / `ExportUR` -- offline program generation
- `SetFreedrive` -- teach mode toggle
- `ValidatePath` -- Newton collision check
- `InstallURDeps` -- pip install ur_rtde/paramiko

### Capability Gating

The UI never shows buttons for features the active driver doesn't support. This is implemented by checking `driver.caps & CAP_*` in panel `draw()` methods and operator `poll()` methods.

## Bundled Libraries (libs/)

These are vendored to avoid requiring users to pip-install robot SDKs:

| Library | Purpose |
|---------|---------|
| `urx/` | Legacy UR communication (fallback when ur_rtde unavailable) |
| `ur_script_python.py` | URScript code generation for export |
| `kukaproxydriver.py` | KUKA socket interface (sends/receives KRL commands) |
| `kuka_krl_python.py` | KRL program code generation |
| `xarm/` | UFactory xArm Python SDK |
| `math3d/` | Transform math, quaternions, vector operations |

## Registration & Lifecycle

`__init__.py` handles addon registration:

**Register order:**
1. Property classes: `DebugVar` → `RobotSlot` → `SceneProperties`
2. `Scene.animaquina` pointer property
3. All operator classes
4. All panel classes
5. Persistent depsgraph handler (for selection sync)

**Unregister (cleanup):**
1. Disconnect all connected slots (stops threads, releases sockets)
2. Remove timer and depsgraph handler
3. Unregister panels → operators → property classes

## Conventions

- **Naming:** all Blender classes use `ANIMAQUINA_` prefix (`PT_`, `OT_`, `UL_`)
- **No stray properties:** all state on `Scene.animaquina`, never on Object or other types
- **Operator IDs:** `animaquina.*` (keymappable)
- **Context access:** `context.scene` only, never `bpy.data.scenes[0]`
- **Thread safety:** poll threads write to `_thread_cache` under lock; only the main-thread timer writes to bpy
- **Error reporting:** operators use `self.report({'ERROR'}, msg)` and return `{'CANCELLED'}`
