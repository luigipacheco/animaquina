---
name: Dependency Strategy — Review & Recommendations
overview: Audit of every external dependency, where it lives, how it's installed, and the recommended strategy for each.
todos: []
isProject: false
---

# Dependency Strategy

## Current Architecture

The addon uses a **hybrid model**:
- **Bundled** libs ship inside the `.zip` (zero user action needed)
- **Runtime-installed** libs are pip-installed on-demand into `vendor_py/`
- **Core** (`animaquina_core`) is GPL like the rest and ships as plain source inside the add-on

Import path bootstrapping (`__init__.py:_bootstrap_import_paths`) adds these to `sys.path`:
```
animaquina/              (addon dir)
../                      (parent — for side-by-side animaquina_core)
animaquina/vendor_py/    (runtime-installed deps)
animaquina_core/vendor_py/  (if core has its own vendors)
```

Each driver also independently adds `libs/` and `vendor_py/` to `sys.path` at import time.

---

## Dependency-by-Dependency Review

### 1. `kukaproxydriver` — KUKA socket protocol

| | |
|---|---|
| **Location** | `animaquina_core/libs/kukaproxydriver.py` |
| **External deps** | None (pure Python: `socket`, `struct`) |
| **License** | GPL-3.0-or-later (ours) |
| **Install method** | Bundled in core |
| **Used by** | `kuka_driver.py` → `from kukaproxydriver import KUKA` |

**Status: Correct.** Already in core. No changes needed.

---

### 2. `kuka_krl_python` — KRL code generator

| | |
|---|---|
| **Location** | `animaquina_core/libs/kuka_krl_python.py` |
| **External deps** | None |
| **License** | GPL-3.0-or-later (ours) |
| **Install method** | Bundled in core |

**Status: Correct.** Already in core.

---

### 3. `ur_script_python` — URScript code generator

| | |
|---|---|
| **Location** | `animaquina_core/libs/ur_script_python.py` |
| **External deps** | None |
| **License** | GPL-3.0-or-later (ours) |
| **Install method** | Bundled in core |

**Status: Correct.** Already in core.

---

### 4. `urx` — Legacy UR driver (bundled fallback)

| | |
|---|---|
| **Location** | `animaquina/libs/urx/` |
| **External deps** | `math3d` (also bundled) |
| **License** | GPL-compatible open source |
| **Install method** | Bundled in addon |
| **Used by** | `ur_driver.py` when `ur_rtde` is unavailable |

**Strategy: Keep bundled.** This is the zero-config fallback that lets UR users
connect without installing anything. It's small, stable, and GPL-compatible.
The `math3d` lib it depends on is also bundled and only used by `urx`.

**Consideration:** As `ur_rtde` becomes the standard, `urx` could eventually be
deprecated and removed. But for now it provides essential out-of-box UR support.

---

### 5. `math3d` — 3D math library

| | |
|---|---|
| **Location** | `animaquina/libs/math3d/` |
| **External deps** | `numpy` (optional) |
| **License** | Open source |
| **Install method** | Bundled in addon |
| **Used by** | Only `urx` |

**Strategy: Keep bundled.** Lives and dies with `urx`. If `urx` is removed, `math3d` goes too.

---

### 6. `xarm` — xArm Python SDK (bundled copy)

| | |
|---|---|
| **Location** | `animaquina/libs/xarm/` (~80 files) |
| **External deps** | None significant |
| **License** | BSD-3-Clause (xArm-Developer/xArm-Python-SDK) |
| **Install method** | Bundled in addon **AND** runtime pip installable |
| **Used by** | `xarm_driver.py` → `from xarm.wrapper import XArmAPI` |

**Problem: Dual-path redundancy.** The SDK is both:
1. Bundled in `animaquina/libs/xarm/` (always available)
2. Installable via "Install xArm Dependencies" button into `vendor_py/`

The bundled copy means xArm works out-of-the-box. The install button overwrites
it with a potentially newer version from GitHub master.

**Recommendation: Keep bundled only, remove the pip installer.**
- The bundled copy already works — no user action needed.
- Installing from GitHub master is unpredictable (breaking changes, API drift).
- To update: vendor a new snapshot manually during release prep.
- This eliminates the "Install xArm Dependencies" button and simplifies the
  Debug > Dependencies panel for xArm users.

**Alternative:** If you want users to be able to update, keep the install button
but make it clear it's "Update SDK" not "Install SDK" (since bundled already works).

---

### 7. `ur_rtde` — Modern UR RTDE protocol

| | |
|---|---|
| **Location** | `vendor_py/` (runtime installed) |
| **External deps** | C++ compiled wheel (platform-specific) |
| **License** | MIT |
| **Install method** | pip install via "Install UR Dependencies" button |
| **Used by** | `ur_driver.py` → `import rtde_control, rtde_receive` |

**Strategy: Keep as runtime install — cannot be bundled.**
- `ur_rtde` ships compiled C++ extensions (`.pyd`/`.so`) that are platform-specific.
- Cannot vendor a single copy that works on Windows + Linux + macOS.
- The current install-on-demand approach is correct.
- Fallback to `urx` when not installed ensures the addon always works.

**No changes needed.**

---

### 8. `paramiko` — SSH/SFTP for UR program upload

| | |
|---|---|
| **Location** | `vendor_py/` (runtime installed) |
| **External deps** | `cryptography`, `bcrypt`, `pynacl` (C extensions) |
| **License** | LGPL-2.1 |
| **Install method** | pip install via "Install UR Dependencies" button |
| **Used by** | `ur_driver.py` for SFTP file transfer to UR controller |

**Strategy: Keep as runtime install.**
- Has native C dependencies (`cryptography`) — cannot be cross-platform bundled.
- Only needed for SFTP upload workflow, not basic control.
- Correctly installed alongside `ur_rtde`.

**Consideration:** `paramiko` pulls in heavy transitive deps (`cryptography` ~30MB).
Could explore lighter SFTP alternatives in the future, but for now this works.

---

### 9. Newton / MuJoCo stack

| | |
|---|---|
| **Location** | Separate Python environment (user-configurable) |
| **Packages** | `numpy`, `mujoco`, `mujoco-warp`, `warp-lang`, `newton` |
| **Install method** | pip install via "Install Newton Dependencies" button |
| **Used by** | `newton_worker.py` (runs as subprocess, not in Blender Python) |

**Strategy: Keep as-is — correctly isolated.**
- These packages are large (hundreds of MB) and require GPU support.
- They run in a **separate Python process**, not Blender's Python.
- The current subprocess + separate interpreter approach is correct.
- The user-configurable Python path is essential (Blender Python may lack GPU support).

**No changes needed.**

---

## Recommended Changes Summary

| Dependency | Current | Recommendation | Action |
|---|---|---|---|
| `kukaproxydriver` | Core (bundled) | Keep in core | None |
| `kuka_krl_python` | Core (bundled) | Keep in core | None |
| `ur_script_python` | Core (bundled) | Keep in core | None |
| `urx` + `math3d` | Addon libs (bundled) | Keep bundled | None |
| `xarm` SDK | Bundled + pip install | **Bundled only** | Remove install button, keep bundled |
| `ur_rtde` | pip install on-demand | Keep pip install | None |
| `paramiko` | pip install on-demand | Keep pip install | None |
| Newton stack | pip install (separate Python) | Keep isolated | None |

### The one actionable change: xArm SDK

The xArm SDK is the only dependency with a redundant dual-path. Options:

**Option A (Recommended): Remove the pip installer, keep bundled only.**
- Delete `ANIMAQUINA_OT_InstallXArmDeps` operator
- Remove the xArm section from Debug > Dependencies panel
- The bundled `libs/xarm/` is the single source of truth
- Update the bundled copy during release prep when needed
- Simplest for users: xArm just works, nothing to install

**Option B: Keep both, rename button to "Update xArm SDK".**
- Bundled copy is the default, install button lets power users get latest
- Less disruptive but maintains the redundancy

---

## Code Review Notes

### sys.path manipulation (medium concern)

Every driver file independently manipulates `sys.path` at import time:
```python
# ur_driver.py, kuka_driver.py, xarm_driver.py — all do this:
for _libs in (multiple paths...):
    if _libs not in sys.path:
        sys.path.insert(0, _libs)
```

Plus `__init__.py:_bootstrap_import_paths()` does its own set.

**Issue:** Multiple `sys.path.insert(0, ...)` calls create import priority
conflicts. The first driver to import "wins" the path order.

**Recommendation:** Centralize path setup in `_bootstrap_import_paths()` only.
Remove the per-driver path manipulation. The bootstrap already covers all
needed paths (`libs/`, `vendor_py/`, parent dir). Drivers should just import
and trust the paths are set up.

### Import refresh pattern (low concern)

`_refresh_rtde_imports()` and `_refresh_xarm_import()` use
`importlib.invalidate_caches()` + `importlib.import_module()` to hot-reload
after pip install. This works but is fragile with Blender's module caching.

**No change needed** — it works in practice and only runs after explicit install.

### vendor_py target directory (low concern)

Both UR and xArm installers use `--target vendor_py` which installs packages
flat into the vendor directory. This can cause conflicts if two packages ship
the same transitive dependency at different versions.

**No change needed now** — the current set of packages doesn't conflict.
Worth monitoring if more runtime-installed deps are added.
