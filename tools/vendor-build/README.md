# Building `vendor_py` for a new Blender / Python version

## ⚠️ CRITICAL: the shipped ur_rtde is a patched MASTER build, not a release

**Do not replace it with `pip install ur-rtde`.** As of July 2026 the latest
PyPI release (1.6.3) **cannot control PolyScope X robots** — it connects, claims
success, and every motion silently no-ops. PolyScope X support (direct script
upload when the robot is in Remote Control mode) only exists on ur_rtde
**master** (commit `68ac4e1`+, after the 2026-03-17 commit
*"Added support for script upload + remote control mode for PSX versions > 10.10"*).

We ship master **plus one local patch** (marked `ANIMAQUINA PATCH` in the
source, archived in [`patches/ur_rtde-animaquina.patch`](patches/ur_rtde-animaquina.patch)):

- **`src/script_client.cpp` — verbose-logging crash fix.** The verbose
  script-injection log does `ur_script.substr(n - 100, ...)`; the injection
  point sits at position ~65, so `n - 100` underflows (unsigned) and the
  process dies with a **native access violation** during script upload.
  This is why animaquina never passes `FLAG_VERBOSE` to the control
  interface (defensive, in case a stock wheel sneaks in). This is a plain
  upstream bug — **candidate for an upstream merge request**; once merged
  and released, we can build unpatched.

(History: an earlier second patch trimmed the PolyScope X torque-control
register recipe on the theory it caused the crash. Verified unnecessary on
2026-07-06 — the full recipe works once the verbose bug is avoided — and
removed. Direct torque control registers are intact.)

Source clone: `%USERPROFILE%\animaquina-build\ur_rtde_master` (patch applied,
not committed). Build script: `build_urrtde_master.bat` (same toolchain as the
stock script, points at the local clone instead of PyPI).

### When upgrading to a newer upstream ur_rtde (> 1.6.3 release or newer master)

1. Check whether the patch is still needed: the `substr(n - 100)` underflow in
   `ScriptClient::scanAndInjectAdditionalScriptCode`. If upstream fixed it,
   build unpatched; otherwise `git apply patches/ur_rtde-animaquina.patch`.
2. Rebuild with `build_urrtde_master.bat` (update the source path if you clone fresh).
3. **Run the smoke test against URSim PolyScope X** before shipping — it covers
   import/flags, receive, script auto-upload + program running (the stock-1.6.3
   failure mode), a real motion round-trip, and a verbose-flag crash probe:
   ```powershell
   tools\ursim-start.ps1               # if the sim is not already running
   # sim: power on, brakes released, Remote Control mode ACTIVE, Blender disconnected
   <blender python.exe> tools\vendor-build\smoke-test-urrtde.py --vendor <candidate folder>
   ```
4. Also sanity-test against PolyScope 5 if available — the same binary serves both.

---

Animaquina bundles its native deps in `animaquina/vendor_py/` (compiled `.pyd`,
ABI-locked to one CPython minor version — the `cpXYZ` tag). Each Blender major
that bumps Python needs its own `vendor_py`:

| Blender | Python | tag    |
|---------|--------|--------|
| 5.0     | 3.11   | cp311  |
| 5.2 β   | 3.13   | cp313  |

Most deps (numpy, mujoco, cryptography, cffi, bcrypt, pynacl, paramiko, glfw,
pyopengl) have `cpXYZ` wheels on PyPI — trivial `pip install --target`. **Only
`ur_rtde` must be compiled from source** (no Windows wheels exist).

## You do NOT re-run this for every zip

This whole procedure is **one-time per Python version**. Once you have a
`vendor_py_<tag>` folder, making a new installable zip is a single command:

```powershell
<blender python.exe> tools\generate-license.py --all-features `
  --target-python-version 3.13 `
  --vendor-py <...>\vendor_py_cp313 `
  --bundle-output dist\animaquina-beta-<name>.zip
```

Re-run the build below only when: Blender bumps Python again, or you upgrade ur_rtde.

## One-time toolchain (winget)

```powershell
winget install --id Microsoft.VisualStudio.2022.BuildTools -e --override "--quiet --wait --norestart --add Microsoft.VisualStudio.Workload.VCTools --includeRecommended"
winget install --id Python.Python.3.13 -e      # matches Blender 5.2's minor; gives Python.h/.lib
```

Standalone **CMake 3.29.6** (must be < 3.30 — see note in build_urrtde.bat) and
**Boost 1.86 source**, both unzipped under `%USERPROFILE%\animaquina-build\`:
- CMake: https://github.com/Kitware/CMake/releases/download/v3.29.6/cmake-3.29.6-windows-x86_64.zip
- Boost: https://archives.boost.io/release/1.86.0/source/boost_1_86_0.zip

## Steps

1. **Boost** (one-time): `tools\vendor-build\build_boost.bat`
   → static libs in `animaquina-build\boost_1_86_0\stage\lib`.
2. **ur_rtde**: `tools\vendor-build\build_urrtde_master.bat` — builds the
   **patched master clone** (see the critical section at the top; required for
   PolyScope X). The old `build_urrtde.bat` builds stock 1.6.3 from PyPI and is
   kept only for reference — do **not** ship its output.
   → `animaquina-build\urrtde_wheel_master\ur_rtde-<ver>-cp313-cp313-win_amd64.whl`.
3. **Assemble `vendor_py_cp313`** (run with the *Blender* python so you get cp313 wheels):
   ```powershell
   <blender python.exe> -m pip install --target vendor_py_cp313 --only-binary=:all: `
     mujoco==3.6.0 paramiko==4.0.0 glfw==2.10.0 PyOpenGL==3.1.10 invoke==2.2.1 numpy==2.4.4
   # then overlay the compiled ur_rtde files from the wheel:
   #   *.cp313-win_amd64.pyd, rtde.dll, urcl/, ur_rtde-*.dist-info/   ->  vendor_py_cp313/
   ```
4. **Verify** under Blender's interpreter:
   ```powershell
   <blender python.exe> -c "import sys; sys.path.insert(0,'vendor_py_cp313'); import rtde_control, rtde_receive, paramiko, mujoco, numpy; print('ok')"
   ```
5. **Ship**: pass `--vendor-py vendor_py_cp313` to `generate-license.py` (see above).

## Gotchas already solved (don't rediscover them)

- `NoDefaultCurrentDirectoryInExePath=1` is set on this machine → breaks Boost
  bootstrap. The .bat clears it.
- CMake ≥ 3.30 drops the legacy `FindBoost` module (CMP0167) that ur_rtde needs
  → use CMake 3.29. The pip `cmake` wheel shim fails as a subprocess → use the
  real Kitware zip.
- MSBuild FileTracker hits MAX_PATH in pip's deep temp dir → the .bat sets
  `TMP/TEMP=C:\b`.
- Never ship a `cpXYZ`-mismatched `vendor_py` — under the wrong interpreter it
  shadows Blender's numpy and breaks unrelated add-ons.
