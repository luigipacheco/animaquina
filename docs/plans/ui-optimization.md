---
name: UI Optimization — Control Clarity, Safety Defaults, and Naming
overview: Redesign panel structure and labels to reduce cognitive load, keep common actions front-and-center, hide advanced parameters behind collapsible sections, and set safer startup defaults.
todos: []
isProject: false
---

# UI Optimization Plan

## Objectives

- Make day-to-day control fast and obvious.
- Separate two workflows clearly:
  - Interactive Toolpathing (discrete robot actions).
  - Puppet Mode (continuous realtime target following).
- Keep advanced and robot-specific tuning available but hidden by default.
- Shift defaults toward safer motion (slower first-use behavior).
- Unify naming across UR, KUKA, xArm so labels describe behavior, not implementation details.

## Current UX Findings (Audit)

- `Control` currently mixes many action types in one vertical stream:
  - state/utility (`Update pose`, `Freedrive`, `Reset`)
  - motion actions (`Move to Target`, `Send Path`)
  - realtime mode (`Start Real Time Puppet`)
  - queue/program stop controls
- Some parameters are globally visible even when only needed in specific workflows.
- Label vocabulary is mixed (`Send`, `Play`, `Load`, `Stage`, `Queue`, `Dynamic Sync`) and creates ambiguity.
- There are strong technical options that should be advanced-only for most users.
- Safety defaults are reasonable but can be made more conservative for first run.

## Design Principles

- Primary-first: show only actions needed 80% of the time.
- Progressive disclosure: hide risk/rare settings in foldouts.
- Consistent verbs:
  - `Move`, `Run`, `Stop`, `Stage`, `Sync`, `Set`.
- One meaning per button:
  - no overloaded `Play` in contexts where it may actually load/reload/select.
- Safety by default:
  - low initial speed/acceleration, explicit opt-in for aggressive tuning.

## Proposed Information Architecture

## 1) Control Panel Structure

- Section A: `Interactive Toolpathing`
  - `Sync From Robot` (rename from `Update pose`)
  - `Teach Mode` (rename from `Freedrive`)
  - `Go Home`
  - `Reset`
  - `Snap Target` + `Move to Target`
  - `Run Toolpath` (rename from `Send Path`)
  - `Run Toolpath (Buffered)` (robot-specific where applicable)
  - `Stop Motion`
  - `Add Marker`
- Section B: `Puppet Mode`
  - `Start Puppet Mode`
  - `Stop Puppet Mode`
  - Compact live status line
- Section C: `Motion Settings` (collapsed by default)
  - Linear and joint settings shared by both sections.
  - Robot-specific advanced controls nested inside.
- Section D: `Advanced Streaming` (collapsed by default, robot-specific)
  - UR ring buffer, queue health.
  - KUKA dynamic sync upload/select and ring buffer.

## 2) Export Panel Structure

- Keep frequently changed fields always visible:
  - `Program Name`
  - Main action buttons (`Export`, `Stage`, `Run`).
- Move all tuning/config to foldouts:
  - `Export Settings` (orientation, motion params, tool/payload, home joints)
  - `Stage Settings` (transport/path/auth)
  - `Run Settings` (dashboard/control ports, launcher details)
- Keep status/log blocks compact and last.

## Visibility Policy (What Should Be Hidden by Default)

- Collapsed by default:
  - Motion tuning
  - Buffer sizes / ring size
  - Dashboard/SFTP/C3 transport details
  - Orientation/custom TCP payload blocks
  - Debug/dependency details
- Always visible:
  - Connect/Disconnect state
  - Target-critical actions (`Move to Target`, `Run Toolpath`, `Stop`)
  - Program name in Export
  - Puppet start/stop in Control when supported

## Safety Defaults (Proposed)

These are proposed default values for first-use safety:

- Shared linear motion:
  - `speed`: `0.05 m/s` (from `0.1`)
  - `acc`: `0.2 m/s^2` (from `0.5`)
  - `radius`: `0.001 m` (from `0.02`) for less unexpected blending
- Shared joint motion:
  - `joint_vel`: `0.5 rad/s` (from `1.05`)
  - `joint_acc`: `0.8 rad/s^2` (from `1.4`)
- KUKA:
  - `kuka_ptp_speed_pct`: `10%` (from `15%`)
- Puppet Mode:
  - `realtime_puppet_rate_hz`: keep `50 Hz`
  - `realtime_puppet_max_step_mm`: `20 mm` (from `50 mm`)
- UR export:
  - `ur_export_vel`: `0.05 m/s` (from `0.1`)
  - `ur_export_acc`: `0.2 m/s^2` (from `0.5`)

Note: these changes prioritize safe startup behavior; advanced users can quickly raise values.

## Naming Convention Proposal

- `Update pose` -> `Sync From Robot`
- `Freedrive` -> `Teach Mode`
- `Send Path` -> `Run Toolpath`
- `Send Path (Queue)` -> `Run Toolpath (Buffered)`
- `Start Real Time Puppet` -> `Start Puppet Mode`
- `Stop` (contextual) -> `Stop Motion` (Control) / `Stop Program` (Export run block)
- `Play` -> `Run Program` (when pendant-side program execution is meant)
- `Reload + Play` -> `Reload + Run`

Rule: use `Run Program` only for controller-side program execution; use `Run Toolpath` for direct Blender-driven motion.

## Implementation Plan

### Phase 1 — Control IA Split (Interactive Toolpathing vs Puppet Mode)

- [x] Reorganize `ANIMAQUINA_PT_Control` into clear labeled groups.
- [x] Keep common actions visible; move tuning under collapsed sections.
- [x] Normalize action labels per naming map.
- [x] Keep robot-specific functionality gated by capabilities/backend.

### Phase 2 — Safer Defaults

- [x] Update default values in `ANIMAQUINA_RobotSlot` motion properties.
- [x] Add concise help text indicating values are conservative by default.
- [x] Ensure no hidden regression in existing operators using these values.

### Phase 3 — Export UI Cleanup

- [x] Keep Program Name outside foldouts for UR and KUKA.
- [x] Standardize button labels (`Export`, `Stage`, `Run Program`, `Stop Program`).
- [x] Move transport/network details into collapsed settings.

### Phase 4 — Status and Feedback Consistency

- [x] Standardize status fields (`Program Transfer`, `Stream Status`, `Puppet Status`).
- [x] Keep single-line summaries in Control; detailed diagnostics in Debug.

### Phase 5 — Context-Aware Clutter Reduction

Hide UI elements that are irrelevant to the current robot type, backend, or connection state.

#### 5a — UR Backend-Aware Gating (Control Panel)

- [x] **"Run Toolpath (Buffered)" button**: only show when `ur_backend == "ur_rtde"`.
  URX does not support the RTDE ring-buffer queue; showing it is confusing.
- [x] **"Advanced Streaming" foldout for UR**: only show when `ur_backend == "ur_rtde"`.
  The entire UR buffered-toolpath section is RTDE-only.

#### 5b — Hide Irrelevant Panels by Robot Type

- [x] **Export parent panel draw**: hidden via `poll()` for xArm (no export workflow).
  Only shows for UR and KUKA.
- [x] **KUKA Debug sub-panel**: already gated by `robot_type == "KUKA"` — OK.
- [x] **Dependencies sub-panel**: only show UR deps box for UR, xArm deps box for xArm — already done. OK.

#### 5c — Connection-State Gating

- [x] **Polling controls** (`Realtime` toggle + `Hz`): already gated by `is_connected` — OK.
- [x] **Stop Motion / Stop Program / Cancel**: already gated — OK.
- [x] **Puppet Mode info message** ("Connect a supported robot…"): reviewed — shows only
  when disconnected or driver lacks puppet methods, which is correct behavior.

#### 5d — Reduce Redundant Controls

- [x] **KUKA Stop/Cancel buttons appear in 3 places**: consolidated into a single
  alert-red Stop/Cancel bar at the **top** of Control panel (visible whenever
  connected). Removed from Toolpath section, Puppet section, and Dynamic Sync
  section. Export panel keeps its own copy next to Run Program for that workflow.
  UR Stop Motion also moved to the same top-level bar.
- [x] **"Set Orientation from Current" in KUKA export**: verified — operator polls for
  both UR and KUKA, works correctly. Name is misleading but functional.
- [x] **Info panel Tool/Base offset**: now only shown for KUKA. UR and xArm no longer
  see the perpetual "(not available)" labels.

#### 5e — Collapse All Panels by Default

- [x] Set `bl_options = {'DEFAULT_CLOSED'}` on all top-level panels so the sidebar
  starts clean. Users expand what they need.

### Phase 6 — Validation

- [ ] Verify with UR (`ur_rtde`), UR (`urx`), KUKA, xArm in connected/disconnected states.
- [ ] Verify no feature loss in staging/play/queue/puppet workflows.
- [ ] Confirm panel remains usable on smaller viewport heights.
- [ ] Confirm "Run Toolpath (Buffered)" and streaming section hidden when URX selected.

## Files Expected to Change (when implementation starts)

- `animaquina/ui/panels.py`
- `animaquina/ui/operators.py` (label text / status wording)
- `animaquina/properties.py` (default values + any new foldout flags)
- Optional: `animaquina/README.md` (terminology updates)

## Acceptance Criteria

- User can immediately identify:
  - where to do discrete toolpath operations,
  - where to run realtime puppet mode.
- Default control values are conservative and safe.
- Advanced/technical options do not clutter primary workflow.
- Labeling is consistent and robot-agnostic where possible.
