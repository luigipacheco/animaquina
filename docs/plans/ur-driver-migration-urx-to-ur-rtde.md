---
name: UR Driver Migration — Parity + Modular Completion
overview: Keep UR feature parity while finalizing the URX -> ur_rtde migration in a modular, multi-robot-safe way. Mark completed work, identify remaining gaps, and define implementation/verification steps.
todos: []
isProject: false
---

# UR Driver Migration (URX -> ur_rtde) — Reviewed Plan

## Documentation Findings (URX + ur_rtde, reviewed 2026-03-05)

- URX legacy behavior (bundled `animaquina/libs/urx`):
  - `URRobot.send_program()` sends URScript immediately through `SecondaryMonitor`.
  - `SecondaryMonitor` connects to UR secondary client interface port `30002` and pushes script bytes directly.
  - This confirms the old path is direct script transport/execution, not file upload + dashboard load.
- SDU ur_rtde API:
  - `RTDEControlInterface` exposes `sendCustomScript()`, `sendCustomScriptFile()`, `sendCustomScriptFunction()`, and `setCustomScriptFile()`.
  - `ScriptClient` exposes `sendScript(file)`, `sendScriptCommand()`, and script injection helpers.
  - `DashboardClient` exposes `loadURP()`, `play()`, `stop()`, `pause()`, `running()` on dashboard port `29999`.
  - Important constraint: Dashboard API documents `loadURP()` (URP), not direct `.script` loading.
- Version status check:
  - PyPI latest published package is `ur_rtde 1.6.2` (Sep 4, 2025).
  - SDU docs currently show API pages generated as `1.6.3`; treat this as docs head/newer than latest PyPI in some sections.

## Execution Model Clarification (URX parity)

- `Send Path` and `Export URScript + Send` are different workflows and should both remain:
  - `Send Path`:
    - live network-driven execution from Blender
    - simple operator flow, but payload/size sensitive for very large paths
    - intended for quick iteration and shorter paths
  - `Export URScript + Send to Robot`:
    - generates full standalone URScript in Blender text block
    - script can be reviewed/edited before send
    - robot executes controller-side after send; Blender can disconnect
    - supports per-vertex overrides (`E_SPEED`, `E_ENABLE`, `L_SPEED`, `F_SPEED`)
    - intended for large/production paths
- Freeze root-cause captured:
  - blocking path send (`wait=True` / synchronous RTDE path call) kept thread busy and made Blender appear stuck.
  - parity target is non-blocking UR path execution (`wait=False`) plus no automatic return-to-start for UR in the same operator call.

## Audit Snapshot (current codebase)

### Completed

- [x] Driver connection hint path is modularized:
  - `DriverBase.connect(..., slot=None)` supports backend hints.
  - `manager.connect_slot()` passes `slot` into `driver.connect(...)`.
- [x] UR backend selection is implemented:
  - `ANIMAQUINA_RobotSlot.ur_backend` exists (`ur_rtde` / `urx`).
- [x] UR dependency install state is implemented:
  - `ur_install_status` and `ur_install_log` exist on slot.
  - `ANIMAQUINA_OT_InstallURDeps` exists and runs pip install + import verify.
- [x] Connection panel supports UR backend + install UI:
  - Backend selector is shown.
  - Install button/status appears when `ur_rtde` is unavailable.
- [x] Send-to-robot path is backend-agnostic:
  - `ANIMAQUINA_OT_SendURProgram` uses `driver.send_program(content)`.
- [x] RTDE->URX fallback bug in `URDriver.connect()` is fixed:
  - RTDE failure now allows URX fallback path.
- [x] UR `Send Path` non-blocking parity restored:
  - UR path execution uses async/non-blocking path send.
  - UR send-path operator does not force return-to-start after dispatch.

## Remaining Gaps To Reach Full UR RTDE Parity

### 1) Critical: Runtime RTDE availability is cached and can stay stale after install

Current behavior:

- `ur_driver.py` sets `_HAS_RTDE` at module import.
- `ur_rtde_available()` and `URDriver.connect()` rely on that cached flag.

Impact:

- User can run `Install UR RTDE` successfully, but current Blender session may still behave as if RTDE is unavailable.
- Panel install state and backend connect path can remain wrong until addon reload/restart.

Required fix:

- Replace static `_HAS_RTDE` checks with a dynamic import check helper.
- Ensure both UI availability check and connect logic use the same fresh check.
- After install success, force refresh of UR driver availability state.

### 2) High: RTDE send-program compatibility hardening

Current behavior:

- `_RtdeBackend.send_program()` now prefers `sendCustomScript(...)` and falls back to `sendCustomScriptFunction(...)`.
- `_RtdeBackend.send_program_file()` uses `sendCustomScriptFile(...)` first and then inline fallback.

Risk:

- Some `ur_rtde` builds may still require `ScriptClient.sendScript(...)` fallback for maximum compatibility.

Required fix:

- Keep compatibility wrapper and add `ScriptClient` fallback path if field testing shows gaps.

### 3) High: Connected backend visibility in UI

Current behavior:

- Connect operator reports generic `"Connected"`.
- No explicit user-visible confirmation of actual backend used (`ur_rtde` vs `urx` fallback).

Required fix:

- On successful connect, show backend in report (for UR slots).
- Optionally show active backend label in Connection panel while connected.

### 4) Medium: Documentation parity

Current behavior:

- README still states UR uses bundled `urx` with no extra install.

Required fix:

- Update README install section:
  - UR supports `ur_rtde` (preferred) with installer button.
  - URX remains legacy fallback.

## Modular Implementation Plan

### Phase 1 — Runtime correctness (blocker)

- [x] Add dynamic RTDE availability helper in `animaquina/drivers/ur_driver.py`.
- [x] Update `URDriver.connect()` to use dynamic availability (not import-time constant only).
- [x] Refresh UR driver availability after `ANIMAQUINA_OT_InstallURDeps` success.
- [x] Keep URX fallback behavior unchanged for compatibility.

### Phase 2 — UR RTDE behavior parity hardening

- [x] Add RTDE script-send compatibility wrapper in `_RtdeBackend.send_program()`.
- [x] Ensure errors from unsupported RTDE script-send API are explicit and user-facing.

### Phase 3 — UI + operator parity

- [x] Report backend used on connect (`ur_rtde` or `urx`) for UR slots.
- [x] Show active backend in Connection panel when connected.
- [x] Keep install UI logic non-blocking (Connect/Disconnect always available).

### Phase 4 — Modular cleanup shared with other robots

- [ ] Factor pip install/status boilerplate into shared operator helper used by:
  - Newton deps installer
  - UR RTDE installer
- [ ] Preserve per-robot package lists and per-slot status fields.

### Phase 5 — Docs and validation

- [x] Update `animaquina/README.md` UR install notes.
- [ ] Run/confirm checklist below in Blender.

### Phase 6 — UR Export -> Upload -> Load Workflow

Goal:

- Keep current `Send to Robot` (immediate execution) path.
- Add a second UR flow similar to the KUKA UX:
  - Export script
  - Prepare/queue for execution from Blender
  - Optional explicit start/stop from Blender

Legacy reference note:

- Old URX addon behavior was immediate command/script send (no controller file transfer dependency).
- This phase now prioritizes URTDE-native script transport and removes SFTP dependency.

#### 6.1 Protocol decision (required first)

- [x] Lock target for v1:
  - Use URTDE/URX direct script transport (no SFTP).
  - Keep Dashboard only for run-state commands (`play/stop/pause/running`) and URP-only control when explicitly needed.
- [x] Explicitly de-scope `*.script` Dashboard load as primary path (Dashboard API is `loadURP()`).

#### 6.2 Slot properties (UR only)

- [ ] Remove/deprecate SFTP-only properties:
  - `ur_sftp_port`
  - `ur_sftp_user`
  - `ur_sftp_password`
  - `ur_remote_path` (only keep if still needed for URP workflows)
- [x] Keep/retain generic runtime properties:
  - `ur_program_name`
  - `ur_dashboard_port`
  - `ur_transfer_status`, `ur_transfer_log`
  - Optional: add `ur_script_transport` enum (`rtde_custom`, `script_client`, `legacy_socket`) for explicit diagnostics.

#### 6.3 Driver capabilities + methods

- [x] Replace SFTP upload API in `URDriver` with transport-first API:
  - `stage_program(content, program_name)` or `send_program_file(local_file)` (name TBD)
  - Implementation order:
    1) RTDE: `sendCustomScriptFile(...)` when available
    2) RTDE: `sendCustomScript(...)` fallback for smaller inline scripts
    3) RTDE: `ScriptClient.sendScript(...)` fallback
    4) URX fallback: legacy `send_program(...)` via secondary interface
- [x] Keep `send_program(content)` as immediate execution path for parity.
- [x] Keep Dashboard helper only for `play/stop/pause/running` and URP operations.
- [x] Keep transport independent from motion backend selection (`ur_rtde` preferred, `urx` fallback).

#### 6.4 Operators and UI

- [x] Replace current UR `Save/Load` buttons with URTDE-native actions:
  - `Stage Program` (uses file/in-memory script transport, no controller filesystem dependency)
  - `Run Staged Program` (optional, uses dashboard play if applicable)
  - `Stop Program` / `Pause Program` (dashboard)
- [x] Keep existing `Send to Robot` button unchanged as immediate fallback.
- [x] Remove SFTP credential fields from `Export > UR` panel for cleaner UX.

#### 6.5 Error handling and non-blocking UX

- [ ] All stage/run operators must:
  - report clear controller errors (remote mode off, protective stop, script rejected, play failed)
  - update `slot.last_error` and UR transfer log fields
  - avoid freezing Blender UI on network timeout
- [ ] Explicitly detect and message Remote/Local mode constraints.
- [ ] Add hard timeouts and deterministic cancel path when pendant program is canceled/stopped.
- [ ] Ensure all UR long-running actions are fire-and-return from Blender UI thread (no blocking waits tied to robot completion).

#### 6.6 Verification checklist (UR upload/load)

- [ ] Export URScript creates expected text block.
- [x] Stage succeeds without SFTP/paramiko installed.
- [ ] Large scripts use file-based RTDE/ScriptClient transport path (not inline-only).
- [ ] Run/Stop/Pause state updates are reflected in UI without Blender freeze.
- [ ] Send immediate path still works exactly as before.
- [ ] Works with both backends selected (`ur_rtde` / `urx`) for the stage/run path.
- [ ] UR `Send Path` remains non-blocking and does not auto-return to start.

## Flow Split Plan (refined)

1. Keep both UR flows explicit in UI/docs
- [ ] Label `Send Path` as "live/quick path" with practical size warning.
- [ ] Label `Export URScript + Send` as "large/production path" flow.

2. Add size-aware guardrail
- [x] Add waypoint count heuristic warning in `Send Path` (e.g., warn strongly above configurable threshold).
- [x] Optional: soft redirect message: "For large paths, use Export URScript + Send to Robot".

3. Large-path robustness (primary target)
- [ ] Prioritize `Export URScript + Send` reliability for 50k+ points.
- [ ] Keep transport URTDE-native (no SFTP dependency) per Phase 6 decisions.

4. Cancellation resilience
- [ ] If pendant cancels/stops program, operator state must finalize cleanly with timeout and clear status update.
- [ ] No lingering modal/timer state after robot-side cancel/abort.

## Large Program + Queue Strategy (new)

### Source-backed constraints (reviewed 2026-03-06)

- UR Dashboard `play/pause/stop` semantics are URP-centric and Remote-Control-gated:
  - UR manual dashboard table documents:
    - `play` can return `"Failed to execute: play"`.
    - dashboard control is `Only Remote Control` for `load/play/stop/pause`.
  - UR forums show practical mismatch when execution started from raw script (`30002`) instead of loaded URP:
    - pause/stop can work, but replay via dashboard `play` may fail for script-only runs.
- UR secondary/script channels are not equivalent to "loaded dashboard program":
  - Local URX behavior and current addon behavior both use secondary script transport (`30002`) for immediate execution.
  - This is useful for send-now, but it does not guarantee dashboard resumability.
- Large script/program size can hit controller-side limits:
  - UR forum reports include `valueStack is full, capacity = 1048576...`.
  - URScript interpreter documentation warns program size/complexity grows and explicitly says too-large programs should be avoided.
- Secondary programs are intentionally limited:
  - UR script manual: secondary program should not contain move/sleep or blocking operations.
  - Therefore queue/motion execution must be in primary/interpreter/main control flow, not secondary `sec` programs.
- SDU ur_rtde API supports the primitives we need:
  - `moveL(path, asynchronous=...)` and `movePath(path, asynchronous=...)`.
  - `sendCustomScript(...)` and `sendCustomScriptFile(...)`.
  - `ScriptClient.sendScript(...)`.
  - Dashboard `loadURP()/play()/stop()/pause()/running()/isInRemoteControl()`.
- RoboDK docs align with dual-flow model:
  - Use script+URP wrapper for larger programs.
  - Script-only execution has different runtime behavior than directly loaded URP.

### Reference Index (URLs + why it matters)

1. UR Dashboard command table (official)
- URL: https://www.universal-robots.com/manuals/EN/HTML/SW5_23/Content/prod-dashboard/Dashboard_table.htm
- Use for:
  - expected command responses (`play`, `pause`, `stop`, `running`, `load`),
  - known `"Failed to execute: play"` behavior.

2. UR Remote Control mode (official)
- URL: https://www.universal-robots.com/manuals/EN/HTML/SW5_21/Content/prod-usr-man/software/PolyScope/content/hamburger_menu_g5/System_remote_en.htm
- Use for:
  - preflight rule before dashboard control commands,
  - user-facing error when remote mode is not enabled.

3. UR Script manual (official PDF, e-Series)
- URL: https://s3-eu-west-1.amazonaws.com/ur-support-site/225539/scriptmanualG5_.pdf
- Use for:
  - interpreter mode behavior and lifecycle (`clear_interpreter()` context),
  - guidance that very large program growth should be avoided.

4. UR secondary program constraints (official)
- URL: https://www.universal-robots.com/manuals/EN/HTML/SW5_20/Content/prod-scriptmanual/G5/SecondaryPrograms.htm
- Use for:
  - why `sec` should not contain blocking or motion semantics we depend on,
  - separating secondary send from queue execution model.

5. SDU ur_rtde API docs (official library docs)
- URL: https://sdurobotics.gitlab.io/ur_rtde/api/api.html
- Use for:
  - `RTDEControlInterface` methods (`moveL`, `movePath`, `sendCustomScript`, `sendCustomScriptFile`),
  - `ScriptClient` and `DashboardClient` method availability.

6. SDU ur_rtde examples
- URL: https://sdurobotics.gitlab.io/ur_rtde/examples/examples.html
- Use for:
  - async motion patterns,
  - baseline usage for compatibility testing by ur_rtde version.

7. UR forum: dashboard play failures
- URL: https://forum.universal-robots.com/t/dashboard-server-returns-failed-to-execute-play/7147
- Use for:
  - real-world context of `play` failure handling and operator messaging.

8. UR forum: remote control requirement in rtde usage
- URL: https://forum.universal-robots.com/t/universal-robots-rtde-c-python-interface/4228
- Use for:
  - user support text when connection/control works in simulator but fails on robot.

9. UR forum: large program/value stack failure
- URL: https://forum.universal-robots.com/t/unknown-error-valuestack-is-full/18368
- Use for:
  - practical upper-bound warning and rationale for non-monolithic execution.

10. RoboDK URP/script flow note
- URL: https://robodk.com/doc/en/Robots-Universal-Robots-How-load-URP-program-run-UR-controller.html
- Use for:
  - design of `Stage + Play` UX and when URP launcher flow is preferred.

11. URX reference (bundled behavior mirror)
- Local files:
  - `animaquina/libs/urx/urrobot.py`
  - `animaquina/libs/urx/ursecmon.py`
- Use for:
  - legacy parity decisions (`send_program` over `30002`, `movexs` semantics).

### Implementation Notes (for future coding)

#### A) Queue session model (UR) - concrete skeleton

- Add a per-slot session state object:
  - `session_id`, `total_points`, `sent_points`, `acked_points`, `queue_target`, `queue_low_watermark`, `started_at`, `last_activity_at`, `status`.
- New driver interface (UR):
  - `start_live_queue(initial_chunk, speed, acc, blend) -> state`
  - `refill_live_queue(state, next_chunk) -> updated_state`
  - `query_live_queue_state(state) -> progress/status`
  - `stop_live_queue(state) -> err`
- Operator modal loop strategy:
  - On start:
    - PTP to first point (blocking in worker thread).
    - start queue with bounded prefill.
  - On each timer tick:
    - query queue depth/progress.
    - if depth <= low watermark, refill with next chunk.
    - if pendant cancel/stop detected, end operator cleanly.
  - On completion:
    - optional PTP return to start.

#### B) Suggested defaults for first real-hardware pass

- `ur_queue_buffer_size`: 6 (expose 2-6 UI range; 36-38 are reserved for motion params).
- refill chunk size: 32 points.
- modal tick: 50 ms.
- queue low watermark: 30% of target depth.
- per network command timeout: 1.5 s.
- session no-progress timeout: 5.0 s (triggers fail-fast with clear error).

#### C) Export path split (recommended UI labels)

- Keep both buttons:
  - `Send to Robot (Immediate)`:
    - one-shot script send; fastest; not dashboard-resumable by default.
  - `Stage to Robot` + `Play`:
    - robust production flow using dashboard-compatible loaded program strategy.
- Add transfer log suffix:
  - `mode=immediate|staged`
  - `transport=secondary_socket_30002|rtde_custom_inline|rtde_custom_file|script_client`
  - `bytes=<N>`

#### D) Error taxonomy (map to user-facing status)

- `REMOTE_CONTROL_DISABLED`
  - message: "Enable Remote Control on the robot to use dashboard play/load."
- `NO_DASHBOARD_PROGRAM_LOADED`
  - message: "No dashboard-loadable program is loaded; script replay mode only."
- `SCRIPT_TOO_LARGE`
  - message: "Program is too large for reliable one-shot execution; use queue or staged flow."
- `QUEUE_STALLED`
  - message: "No queue progress detected within timeout; check robot state and network."
- `PENDANT_ABORTED`
  - message: "Program canceled from pendant; Blender state has been cleaned up."

#### E) File-level implementation map

- Driver changes:
  - `animaquina/drivers/ur_driver.py`
- Queue operator changes:
  - `animaquina/ui/operators.py` (`ANIMAQUINA_OT_SendPathQueueUR`)
- Export control labels and status UI:
  - `animaquina/ui/panels.py`
- Slot properties for diagnostics and queue tuning:
  - `animaquina/properties.py`

#### F) Regression checklist add-on (quick smoke order)

1. Connect with `ur_rtde`, verify backend label and remote-control preflight.
2. `Send Path` small path (<2k) still works with begin/end PTP.
3. `Send Path (Queue)` medium path (10k) remains responsive and refill loop progresses.
4. Export 50k+:
   - immediate path warns with recommendation,
   - staged path completes without Blender freeze.
5. Pendant cancel during run:
   - operator exits,
   - timer/thread cleaned,
   - `slot.last_error` and transfer log updated.

### Problem statement (current addon)

- We currently have:
  - `Send Path` and `Send Path (Queue)` for UR, but both are chunked serial dispatch from Blender threads, not a controller-resident ring buffer queue like KUKA.
  - `Export + Send` sends full generated script over network transport; large payloads can still freeze workflow or trip controller limits on real hardware.
  - Dashboard `play` is used in a context where a dashboard-loadable URP may not exist, producing recurring `Failed to execute: play`.
- We need:
  - feature parity with legacy URX behavior,
  - dynamic queue updates (KUKA-like),
  - robust handling above ~50k points on real robot,
  - no Blender UI hangs.

### Target architecture (two explicit UR execution modes)

1) Live Queue Mode (dynamic / on-the-fly updates; Control panel)
- Goal:
  - true buffered queue behavior (append/update future points while robot is moving),
  - deterministic cancel/stop handling,
  - no one-shot giant script payload.
- Implementation direction:
  - Keep initial PTP-to-start behavior (user-requested parity).
  - Replace current "chunked blocking `execute_ptp_path` loop" with queue session model:
    - start queue session (`ur_queue_start`) with bounded prefill.
    - periodic refill (`ur_queue_refill`) from modal timer using queue depth thresholds.
    - completion detection (`ur_queue_is_done`) and optional return-to-start PTP.
  - Transport choice for queue commands:
    - v1: ur_rtde asynchronous `moveL(path, async)` / `movePath(async)` in bounded chunks + async-progress polling.
    - v2 (preferred for true live updates): interpreter-mode command feed (port `30020`) with bounded queue depth and periodic `clear_interpreter()` to avoid program growth.
- Hard requirement:
  - every queue operation must have finite timeout + cancel path; never wait indefinitely on robot completion in Blender worker threads.

2) Exported Program Mode (very large static jobs; Export panel)
- Goal:
  - run very large jobs controller-side without giant one-shot inline script execution behavior.
- Implementation direction:
  - Keep `Export URScript` text generation.
  - Distinguish two send paths:
    - `Send Immediate` (current behavior): send script for immediate execution.
    - `Stage + Play URP` (new robust path): use a stable URP launcher flow and dashboard `load/play`.
  - Practical robust pattern:
    - maintain a small launcher URP on controller that calls the staged script payload.
    - dashboard controls operate on loaded URP (not raw script-only run), improving `play` semantics.
  - If launcher URP is unavailable, report explicit fallback mode and disable misleading replay messaging.
- Guardrails:
  - for very large exports, warn before inline send and recommend Stage+Play mode.
  - keep point-count + byte-size heuristics in operator reports/logs.

### API and driver changes required

- URDriver:
  - Add explicit mode APIs:
    - `start_live_queue(...)`
    - `refill_live_queue(...)`
    - `stop_live_queue(...)`
    - `query_live_queue_state(...)`
  - Keep existing:
    - `send_program(content)` for immediate one-shot.
    - `stage_program(content, name)` for staging transport.
  - Add transport diagnostics fields:
    - script bytes sent,
    - chunk count,
    - average send latency,
    - last command response.
  - Add dashboard preflight helpers:
    - `dashboard_is_remote_control()`
    - `dashboard_loaded_program()`
    - `dashboard_program_state()`
- Operators:
  - `Send Path (Queue)` becomes canonical UR queue operator (not experimental).
  - export send/stage operators:
    - run fully non-blocking,
    - enforce hard timeout and cancellation cleanup,
    - never leave modal timer alive after failure paths.

### UI/UX behavior contract

- Control panel:
  - `Send Path`: quick path (small/medium).
  - `Send Path (Queue)`: dynamic queue mode (preferred for long/live-updated paths).
  - Both keep PTP at beginning/end where configured.
- Export panel:
  - `Send to Robot (Immediate)` for quick one-shot.
  - `Stage to Robot` + `Play` for large/production jobs.
  - show active transport and dashboard readiness state.

### Real-Time Puppet Mode (separate track, not replacement)

Scope boundary (explicit):
- Keep current long-program strategy:
  - for long static jobs, export/stage/run on controller remains primary.
- Keep existing flows working:
  - `Send Path` and `Send to Robot (Immediate)` stay as-is.
- New real-time mode applies to:
  - `Send Path (Queue)` evolution into low-latency streaming mode.

Intent:
- Robot behaves like a "puppet" driven by Blender timeline/procedural motion.
- Support interactive and effectively unbounded trajectories.
- Decouple process control (MQTT/FabNodes) from motion transport.

#### Real-time architecture (v1 target)

- Layer 1: Blender/Animaquina
  - target generation (scene, GN, animation, sim)
  - operator controls (start/stop stream, arm/disarm)
  - visualization (lag, queue depth, speed scaling)
- Layer 2: external streaming daemon (Python process outside Blender UI thread)
  - fixed-rate loop
  - trajectory ring buffer
  - RTDE feedback polling
  - transport to robot
- Layer 3: UR controller
  - persistent control execution path for servo targets
  - stop/hold behavior on underrun/fault

#### Motion transport options

- Option A (preferred first implementation):
  - ur_rtde servo streaming (`servoJ`) from daemon with fixed cadence.
  - avoids giant script payloads and aligns with puppet behavior.
- Option B (advanced fallback):
  - persistent URScript socket receiver loop on robot, daemon feeds joint targets.
  - useful if we need custom protocol control beyond Option A.

#### Real-time performance targets (initial)

- Control rate:
  - v1 target: 125 Hz stable on Windows/Linux.
  - v2 stretch: 250 Hz where network/CPU permit.
- End-to-end latency target:
  - <20 ms typical.
- Buffer depth:
  - 1-2 seconds of targets.
- Control primitive:
  - `servoJ` for joint-space puppet mode.

#### Safety and failure behavior (real-time mode)

- Buffer underrun:
  - hold last safe target briefly, then controlled stop (`stopj`/equivalent).
- Network interruption:
  - pause stream state, surface clear UI error, require explicit resume.
- Protective stop / mode change:
  - immediate stream halt, clear operator state, keep Blender responsive.
- No-progress watchdog:
  - terminate stream session if no ack/progress within timeout.

#### Real-time integration points in addon

- New UR stream APIs (driver facade):
  - `stream_open(...)`, `stream_push(...)`, `stream_status(...)`, `stream_close(...)`.
- Operator path:
  - rework `ANIMAQUINA_OT_SendPathQueueUR` to use stream session and refill.
- Optional new operator:
  - `ANIMAQUINA_OT_URRealtimePuppet` (Start/Stop live control from timeline).
- Properties/UI:
  - stream rate, buffer seconds, watchdog timeout, mode indicator.

### Phased implementation plan for this strategy

#### Phase 7 — UR Queue Streaming (smooth continuous motion, KUKA parity)

##### Problem analysis

The current `ANIMAQUINA_OT_SendPathQueueUR` sends chunks via `execute_ptp_path(chunk, wait=True)` in a serial loop. Each chunk becomes a separate `moveL(path)` RTDE call. The robot decelerates to zero velocity at the end of each chunk because the controller has no lookahead into the next chunk. This causes the brief stop between chunks.

The KUKA streaming model avoids this because:
- A KRL program (`mq_stream.src`) runs on the controller and reads from a ring buffer (`MQ_PT[]`).
- Blender refills the buffer via variable writes as the robot consumes points.
- The KRL `LIN` command with `$APO.CDIS > 0` (approximation) blends through points continuously.
- The controller's motion planner has lookahead into the buffer — it always sees upcoming points.

The UR controller behaves differently: each `moveL(path)` call is an independent trajectory command. The controller blends within a single `moveL(path)` (when blend > 0 on intermediate waypoints) but decelerates at the command boundary because it doesn't know another command is coming.

##### ur_rtde primitives available (reviewed 2026-03-06)

From SDU ur_rtde API (`RTDEControlInterface`):

1. **`moveL(path, async)`** — Send multiple waypoints `[x,y,z,rx,ry,rz,vel,acc,blend]`. Blend > 0 on intermediate points = smooth. Cannot append to a running path. Limited by controller memory.
2. **`movePath(path, async)`** — Same as moveL path but supports mixed moveJ/moveL entries.
3. **`servoJ(q, speed, acc, dt, lookahead, gain)`** — Real-time joint position streaming. Requires fixed-rate loop (125–500 Hz). Uses `initPeriod()` + `waitPeriod()` for timing. Smooth but no velocity/blend profile — we control exact position at each timestep.
4. **`servoL(pose, speed, acc, dt, lookahead, gain)`** — Same as servoJ but in Cartesian space.
5. **`getAsyncOperationProgress()` / `getAsyncOperationProgressEx()`** — Monitor progress of async moveL/moveJ. Returns <0 when idle, ≥0 during execution.
6. **`servoStop(a)` / `stopL(a)` / `stopJ(a)`** — Clean stop primitives.

None of these allow "appending waypoints to a running moveL". For continuous motion without stops, we need controller-side buffering.

##### Implementation: RTDE Register Ring Buffer (KUKA Dynamic Sync parity)

**Status: IMPLEMENTED (2026-03-06)**

Exact mirror of KUKA Dynamic Sync but using RTDE input/output registers instead of C3 Bridge variables.

Architecture:
```
┌─────────────────────┐    RTDE input registers     ┌────────────────────────┐
│  Blender (modal)    │ ──── waypoint writes ──────▶ │  UR Controller         │
│                     │    (setInputDoubleRegister)   │  (URScript mq_stream)  │
│  - reads mesh       │                              │                        │
│  - fills ring buf   │ ◀── progress feedback ────── │  loop:                 │
│  - monitors rd_idx  │    (getOutputIntRegister)     │   read_input_register  │
│                     │                              │   movel(p, v, a, r)    │
└─────────────────────┘                              └────────────────────────┘
```

Register layout (ring_buffer_size<=6, 6 floats per waypoint):
- Input double registers 0–35: up to 6 waypoint slots × 6 floats (x,y,z,rx,ry,rz)
- Input double registers 36–38: vel, acc, blend (reserved motion params)
- Input int registers 0–3: action, cmd_id, pt_cnt, wr_idx
- Output int registers 0–1: rd_idx (robot read cursor), done_id

URScript (`mq_stream` in `animaquina/runtime/ur_stream.py`):
- Reads waypoints from input float registers in a ring buffer loop
- Executes `movel()` with blend radius for smooth continuous motion
- Updates `output_integer_register(0)` with read cursor so Blender can refill
- Sets `output_integer_register(1)` to cmd_id when complete
- Sent on-the-fly via `send_program()` (no pre-upload needed)

Driver methods (mirror KUKA exactly):
- [x] `URDriver.stream_start(total, speed, radius, initial_waypoints, ring_size)` → state dict
- [x] `URDriver.stream_refill(state, rd_idx, waypoints)` → (count, error)
- [x] `URDriver.stream_read_rd_idx()` → int
- [x] `URDriver.stream_is_done(state)` → bool
- [x] `URDriver.stream_restore_control()` — reupload RTDE control script after streaming

Operator (`ANIMAQUINA_OT_SendPathQueueUR`):
- [x] Phased modal identical to KUKA SendPath streaming:
  - `move_to_first`: PTP to first waypoint (worker thread)
  - `streaming`: modal timer (50ms) reads rd_idx, refills ring buffer, re-reads mesh
  - `returning`: PTP back to start (worker thread)
- [x] Live mesh re-read on every tick (same as KUKA)
- [x] Cleanup restores RTDE control script

Key differences from KUKA:

| Aspect | KUKA | UR |
|--------|------|-----|
| Transport | C3 Bridge `write_var("MQ_PT[i]")` | RTDE `setInputDoubleRegister(reg, val)` |
| Controller program | KRL `mq_stream.src` (uploaded+selected) | URScript sent via `send_program()` on-the-fly |
| Buffer size | 32–128 slots (KRL array) | 2–6 slots (36 doubles for waypoints + 3 reserved params) |
| Read cursor | `read_var("MQ_RD_IDX")` | `getOutputIntRegister(0)` |
| Done signal | `read_var("MQ_DONE_ID")` | `getOutputIntRegister(1)` |
| Blend | `$APO.CDIS` (mm) | `movel(..., r=blend)` (meters) |
| Upload requirement | Must upload+select mq_stream | No pre-upload; script sent each time |
| Post-stream cleanup | N/A (KRL keeps running) | `reuploadScript()` to restore RTDE control |

##### Future: servoL real-time streaming (puppet mode)

For interactive / timeline-driven motion where Blender controls exact robot position in real time.
- Uses `servoL` at 125–500 Hz from a dedicated daemon thread.
- Blender modal timer feeds target poses into a thread-safe deque.
- Best for: timeline scrubbing, procedural motion, real-time puppet control.

This tier is deferred to Phase 11 (Real-Time Puppet Mode).

#### Phase 8 — Large program robust run path
- [ ] Split export send into `Immediate` vs `Stage+Play URP`.
- [ ] Add launcher-URP flow (with clear setup preconditions).
- [ ] Disable/annotate dashboard replay when no URP is loaded.
- [ ] Add large-payload heuristics (point + byte thresholds) and explicit recommendations.

#### Phase 9 — Error-state hardening
- [ ] Unify dashboard error parsing (`Failed to execute: play`, remote-control not allowed, no program loaded, automove/safety preconditions).
- [ ] Add pendant-cancel detection and finalize operator state without hanging Blender.
- [ ] Ensure all UR network operations have strict timeout + thread-safe completion flags.
- [ ] Socket streaming specific errors:
  - `STREAM_SOCKET_BIND_FAILED`: Blender can't open server socket on configured port.
  - `STREAM_CONNECT_TIMEOUT`: Controller didn't connect back within timeout.
  - `STREAM_BROKEN_PIPE`: Socket closed mid-stream (robot stopped or network failure).
  - `STREAM_UNDERRUN`: All points sent but robot hasn't finished (normal completion path).
  - `STREAM_CANCELLED`: User cancelled from Blender side.

#### Phase 10 — Validation on real hardware
- [ ] Test matrix for 10k / 50k / 100k waypoint scenarios:
  - `Send Path` (single moveL)
  - `Send Path (Queue)` — Tier 1 (single moveL, no chunk)
  - `Send Path (Queue)` — Tier 2 (socket streaming)
  - `Export -> Send Immediate`
  - `Export -> Stage+Play`
- [ ] Validate smooth motion (no stops between chunks) for Tier 1 and Tier 2.
- [ ] Validate pause/stop/play semantics separately for:
  - raw script execution
  - loaded URP execution
- [ ] Confirm no Blender freeze and no orphaned modal timers in all failure/cancel scenarios.
- [ ] Validate live mesh re-read during Tier 2 streaming (mesh position changes reflected in remaining path).
- [ ] Socket streaming stress test:
  - 50k+ points continuous stream.
  - Cancel mid-stream from Blender.
  - Network disconnect mid-stream.
  - Pendant stop mid-stream.

#### Phase 11 — Real-Time Puppet Mode (UR, Tier 3)
- [ ] Finalize mode contract in UI/docs:
  - long program flow remains export/stage/run,
  - puppet mode is separate and does not replace it.
- [ ] Implement servoL streaming daemon:
  - dedicated Python thread with `initPeriod()`/`waitPeriod()` timing loop at 125 Hz.
  - thread-safe deque for target poses.
  - RTDE feedback polling for actual position.
  - health watchdog (no-progress timeout, buffer underrun detection).
- [ ] Add UR driver servo stream methods:
  - `servo_stream_open(dt, lookahead, gain)` → state.
  - `servo_stream_push(state, pose)` → error.
  - `servo_stream_status(state)` → progress/health.
  - `servo_stream_close(state)` → cleanup.
- [ ] Add waypoint interpolation layer:
  - sparse waypoints (from mesh) → dense targets at servo rate.
  - configurable interpolation (linear / cubic).
- [ ] Rewire `Send Path (Queue)` to optionally use servo stream (Tier 3 mode selector).
- [ ] Add realtime status HUD fields:
  - queue depth, loop rate, lag ms, speed scaling, fault code.
- [ ] Add hard stop and cleanup guarantees:
  - no orphan thread/timer/process on cancel/disconnect/robot fault.
  - `servoStop()` always called on exit.
- [ ] Real hardware acceptance tests:
  - timeline scrubbing,
  - interactive target updates,
  - 10+ minute continuous stream without freeze.

## Decision Update (2026-03-06) — Full Program Stage via SSH/SFTP

This update supersedes earlier "no SFTP" transport assumptions for large static programs.

Transport contract moving forward:

1. `Send to Robot (Immediate)`:
- Keep current direct execution transport (`30002` / RTDE custom script) for quick iteration.

2. `Stage to Robot (Full Program)`:
- Use SSH/SFTP file transfer to place the generated `.script` on controller storage (for large jobs).
- Target path default: `/programs` (configurable per slot).

3. `Play Staged Program`:
- Dashboard remains URP-oriented; do not assume direct `.script` load support via dashboard.
- Preferred run model:
  - load/play a known launcher URP that executes the staged script.
- If launcher URP is not configured:
  - keep explicit fallback behavior and user messaging.

Why:
- Large monolithic script sends over runtime channels can fail on real robots.
- File staging is more robust for very large jobs and aligns with production execution workflows.

## Concrete Action Plan (implementation-ready)

### Step 0 — Scope lock (start here)
- [ ] Lock flow boundaries in code/docs:
  - long static programs: `Export -> Stage (SFTP) -> Play`.
  - quick jobs: `Send to Robot (Immediate)`.
  - `Send Path` remains as-is.
  - `Send Path (Queue)` continues as queue track / future realtime track.
- [ ] Mark old "SFTP removal" section as legacy/superseded.

### Step 1 — Transport settings + UI wiring
- [ ] Re-enable and surface UR transfer settings in `Export > UR` panel:
  - `ur_remote_path`, `ur_sftp_port`, `ur_sftp_user`, `ur_sftp_password`.
- [ ] Add optional launcher URP name property (for dashboard load/play path).
- [ ] Keep credentials hidden by default in UI and masked for password fields.
- Files:
  - `animaquina/properties.py`
  - `animaquina/ui/panels.py`

### Step 2 — Driver SSH/SFTP staging implementation
- [ ] Add `URDriver.upload_program_sftp(content, program_name, remote_path, user, password, port)`:
  - write temp local file,
  - transfer via SFTP,
  - verify remote file exists/size where possible,
  - return explicit structured error on failure.
- [ ] Keep existing `send_program(content)` unchanged for immediate execution.
- [ ] Update `stage_program(...)` to select transfer strategy:
  - prefer SFTP for full-program stage,
  - fallback to existing transport only when user explicitly selects fallback mode.
- Files:
  - `animaquina/drivers/ur_driver.py`

### Step 3 — Operator migration to explicit modes
- [ ] `Stage to Robot` operator:
  - call SFTP stage path,
  - never block UI thread,
  - report `transport=sftp` and remote destination.
- [ ] `Play` operator:
  - if launcher URP configured -> dashboard load/play.
  - else -> clear actionable error (no launcher configured).
- [ ] Keep `Send to Robot` operator as current immediate path.
- Files:
  - `animaquina/ui/operators.py`

### Step 4 — Error handling hardening
- [ ] Normalize transfer/play error codes:
  - `SFTP_AUTH_FAILED`
  - `SFTP_CONNECT_FAILED`
  - `REMOTE_PATH_INVALID`
  - `LAUNCHER_URP_NOT_CONFIGURED`
  - `DASHBOARD_PLAY_FAILED`
  - `REMOTE_CONTROL_DISABLED`
- [ ] Ensure all network operations have explicit connect/read/write timeouts.
- [ ] Ensure modal timers/threads always clean up on every failure path.
- Files:
  - `animaquina/drivers/ur_driver.py`
  - `animaquina/ui/operators.py`

### Step 5 — Validation matrix (minimum executable set)
- [ ] Small script (<2k points):
  - immediate send works as today.
- [ ] Large script (~50k+ points):
  - SFTP stage succeeds,
  - launcher URP load/play runs without Blender freeze.
- [ ] Pendant cancel mid-run:
  - Blender stays responsive,
  - operator exits with deterministic status.
- [ ] RTDE and URX backend selection does not break stage/play controls.

### Step 6 — Documentation + recovery playbook
- [ ] Add "UR full-program workflow" docs:
  - how to configure SFTP fields,
  - launcher URP prerequisite,
  - troubleshooting by error code.
- [ ] Add "fallback strategy" docs:
  - when to use immediate send vs staged flow.
- Files:
  - `animaquina/README.md`
  - `plans/ur-driver-migration-urx-to-ur-rtde.md`

## Legacy Sequence (SFTP removal) — superseded

1. Driver transport refactor
- [x] Add `send_program_from_file()`/`stage_program()` in `_RtdeBackend` using `sendCustomScriptFile` first, then fallbacks.
- [x] Keep `_UrxBackend.send_program()` unchanged for legacy parity.
- [x] Remove `paramiko` dependency from driver and installer path.

2. Operator migration
- [x] Replace `ANIMAQUINA_OT_URSaveProgramToRobot` + `ANIMAQUINA_OT_URLoadProgramOnRobot` logic with Stage/Run/Stop/Pause operators.
- [ ] Keep operator reports and transfer logs explicit about which transport was used.

3. UI/property cleanup
- [ ] Remove SFTP fields from properties/panels and clean stale migration fields safely.
- [ ] Keep dashboard port and program name fields.

4. Regression + stress
- [ ] Validate export -> stage -> run for long paths (50k+ points scenario).
- [ ] Validate `Send Path` quick-flow remains responsive and exits immediately on UR.
- [ ] Confirm no blocking waits in Blender modal/operators during run-state polling.

## Verification Matrix (Definition of Done)

### Backend selection and fallback

- [ ] `ur_backend = ur_rtde`, RTDE available -> connects with backend `ur_rtde`.
- [ ] `ur_backend = ur_rtde`, RTDE connect fails, URX available -> falls back to `urx`.
- [ ] `ur_backend = ur_rtde`, RTDE unavailable, URX available -> connects with `urx`.
- [ ] `ur_backend = urx`, URX available -> connects with `urx`.

### Install flow

- [ ] Press Install UR RTDE -> status transitions RUNNING -> OK/ERROR.
- [ ] After successful install, panel availability updates in same session (no restart required).

### Motion / command parity

- [ ] Update Pose (TCP + joints) works on RTDE backend.
- [ ] Move to Target (LINEAR/PTP), Send Path, Go Home, Freedrive work on RTDE backend.
- [ ] Send URScript to Robot works on RTDE backend via `driver.send_program()`.

### UX / modularity

- [ ] Connect report clearly indicates backend in use.
- [ ] No regressions for KUKA/XARM connect flow.
- [ ] Manager/runtime remain backend-agnostic (driver interface boundary preserved).
