# Safety

**Animaquina drives industrial robots. Industrial robots kill people.**

Read this before you connect the add-on to any machine.

## No warranty, no liability

Animaquina is licensed under the **GNU General Public License v3.0 or later**.
Sections 15, 16 and 17 of that licence — reproduced in full in [LICENSE](LICENSE) —
state that the program is provided **"AS IS" WITHOUT WARRANTY OF ANY KIND**, and
that no copyright holder or contributor is liable for any damages arising from
its use, including damages to property, third parties, or from the program's
failure to operate with any other software.

Those terms apply to every user, every fork, and every deployment. If you run
this software on a robot, **you** are the integrator and **you** carry the risk.

## Animaquina is not a safety system

Animaquina performs **no safety function whatsoever**. Specifically, it is not
and must never be treated as:

- a safety-rated controller, PLC, or safety monitor
- a substitute for the robot's own safety configuration, safety-rated limits,
  reduced-speed modes, or emergency stop circuit
- a substitute for physical guarding, light curtains, fencing, or interlocks
- a substitute for a risk assessment under ISO 10218, ISO/TS 15066,
  ANSI/RIA R15.06, the Machinery Directive, or whatever regime applies to you
- evidence that a motion is safe to execute

The robot controller and the cell's independent safety systems are the only
things standing between the software and a person. Animaquina sits entirely
outside that boundary and can fail in ways it cannot detect.

## Known limitations that can produce unsafe motion

These are real, current, and documented deliberately. See
[docs/plans/robot-ide-roadmap-2026-09-09.md](docs/plans/robot-ide-roadmap-2026-09-09.md)
for the full engineering review.

- **Path validation can report `VALID` without a real solver.** If MuJoCo or its
  dependencies are missing or fail to load, a stub validator may be selected and
  the UI does not currently require a real backend before showing a pass.
  **Never treat a validation pass as certification of anything.**
- **Validation checks waypoints, not the motion between them.** Interpolated
  segments, approach and retreat moves, travel to home, and controller blends
  are not checked. A path whose endpoints are clear can collide in the middle.
- **Collision geometry is approximate.** Robot links use visual bounding boxes.
  There is no dedicated tool collision model. Self-collision is off by default.
  Cell objects are metadata, not collision geometry.
- **Puppet Mode streams targets without validation.** Live targets are filtered
  and sent straight to the driver. The controller may choose a different joint
  configuration than the on-screen preview.
- **Preview is not execution.** UR Puppet Mode commands Cartesian `servoL`; the
  controller solves its own IK. What you saw in Blender is not necessarily what
  the arm will do.
- **Joint limits may fall back to ±2π** when an imported model's limits are not
  read correctly.

## Before you connect to real hardware

1. Complete a risk assessment for your cell. Animaquina does not do this for you.
2. Configure the robot's own safety-rated limits — speed, zones, payload —
   on the controller, not in Blender.
3. Keep an enabling device and an emergency stop within reach of an operator
   whose only job is the emergency stop.
4. Test every new path in simulation, then at reduced speed with the cell clear,
   before running at production speed.
5. Never stand inside the working envelope while the program is executing.
6. Assume any streaming or Puppet Mode session can send an unexpected pose.

## Reporting a safety-relevant defect

Open an issue and label it `safety`. If you believe a defect could cause
unexpected motion, say so in the title. Safety-relevant reports are triaged
ahead of features.
