# IDX-Based Attribute Dispatch

## Problem

Currently we write every custom variable to the robot per waypoint (E_SPEED, F_SPEED, etc.).
This means:
- The robot program is complex (variable writes interleaved with motion)
- Every new attribute requires changes to the export/streaming code
- The robot must support writing arbitrary variables (not all do cleanly)
- No easy path to forwarding attribute values to external peripherals (extruder, fan, MQTT)

## Concept

Invert the flow. The robot only tracks **one variable: `IDX`** (current waypoint index).
Blender becomes the attribute dispatcher.

```
Robot side (simple):
  for each waypoint:
    IDX = point_index
    move to waypoint

Blender side (reads IDX, dispatches attributes):
  poll IDX from robot
  look up point IDX in mesh attributes
  set properties on slot / object
  forward to MQTT / peripherals
```

## How It Works

### 1. Export / Streaming — Write IDX Only

During toolpath export or live streaming, the generated program sets a single
variable `IDX` to the current waypoint index at each move:

**KRL:**
```krl
IDX = 0
LIN pos_0
IDX = 1
LIN pos_1
...
```

**URScript:**
```urscript
write_output_integer_register(0, 0)
movel(pos_0, ...)
write_output_integer_register(0, 1)
movel(pos_1, ...)
...
```

**xArm:** Use CGPIO or a custom variable mechanism.

The robot program stays dead simple — just motion + IDX update. No per-point
variable writes for E_SPEED, F_SPEED, etc.

### 2. Blender Reads IDX Back

The poll thread already reads variables via `driver.read_var("IDX")` (or
`output_int_register_0` for UR). This is the same debug_vars / Variables panel
system we just built.

When IDX changes, Blender knows the robot has moved to a new waypoint.

### 3. Attribute Lookup in Blender

When IDX changes, Blender looks up the mesh attributes for that point index:

```python
# In the poll timer callback (manager.py or a dedicated handler)
idx = int(slot.debug_vars["IDX"].value)  # or however we read it
eval_obj = obj.evaluated_get(depsgraph)

for attr_name in eval_obj.data.attributes:
    if attr_name == "position":
        continue
    att = eval_obj.data.attributes[attr_name]
    value = att.data[idx].value
    # Store on slot or object as custom property
    slot[attr_name] = value
```

This means **any** attribute on the mesh automatically becomes available in
Blender as a property — no export code changes needed when adding new attributes.

### 4. Forward to MQTT / Peripherals

The looked-up attribute values (now stored as properties) can be forwarded:

```
IDX changes → lookup attributes → set properties → MQTT publish
                                                  → UI display
                                                  → Blender drivers
                                                  → custom callbacks
```

MQTT integration becomes a simple subscriber to property changes:
- Topic: `animaquina/{robot_uid}/{attr_name}`
- Payload: attribute value at current IDX

The extruder controller subscribes to `animaquina/robot1/E_SPEED` and gets
real-time speed values as the robot moves through the path.

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│ Blender (Animaquina)                                    │
│                                                         │
│  Mesh Object                                            │
│  ├── position    [p0, p1, p2, ...]                      │
│  ├── E_SPEED     [0,  5,  8, ...]     ◄── attributes    │
│  ├── F_SPEED     [0,  0, 50, ...]         per point     │
│  ├── E_ENABLE    [0,  1,  1, ...]                       │
│  └── my_custom   [.., .., .., ...]                      │
│                                                         │
│  Poll Thread                                            │
│  └── reads IDX from robot ──┐                           │
│                              ▼                          │
│  IDX Dispatch Handler                                   │
│  ├── looks up attributes[IDX]                           │
│  ├── sets slot properties                               │
│  └── publishes to MQTT ─────────► Extruder / Fan / etc  │
│                                                         │
└──────────────┬──────────────────────────────────────────┘
               │ (only IDX variable)
               ▼
┌──────────────────────┐
│ Robot Controller     │
│                      │
│  IDX = 0             │
│  LIN pos_0           │
│  IDX = 1             │
│  LIN pos_1           │
│  ...                 │
└──────────────────────┘
```

## What Needs to Change

### Export (program_exports.py)
- Add IDX variable write before each move in `build_krl_program()` and `build_ur_script()`
- For KUKA: `krl.change_variable("IDX", i)` before each LIN
- For UR: `ur.set_variable("IDX", i)` or `write_output_integer_register(0, i)` before each movel
- Keep existing E_SPEED/F_SPEED writes as optional (backwards compat), but IDX becomes the primary mechanism

### Live Streaming (operators.py)
- KUKA: `driver.write_var("IDX", rd_idx)` each tick (already have rd_idx)
- UR: `driver.write_var("IDX", servo_sent)` each tick (already have servo_sent)
- This is simpler than writing N custom vars — just one write per tick

### IDX Dispatch Handler (new, in manager.py or dedicated module)
- Watches for IDX changes in the poll data
- On change: evaluates mesh, reads all attributes at that index
- Stores values as slot custom properties or a dedicated dict
- Fires callbacks (MQTT publish, UI update, etc.)

### MQTT Integration (future, new module)
- Subscribes to IDX change events
- Publishes attribute values to configurable MQTT topics
- Broker address / topic pattern configurable in slot properties

## ⚠️ Timing precision: advance run vs main run (critical for extruder sync)

This is the make-or-break detail for forwarding per-point attributes (extruder
speed, flow, fan) in sync with **physical** robot position.

In KRL, a plain assignment like `IDX = i` or `MQ_RD_IDX = MQ_RD_IDX + 1` runs in
the **advance run** (the planner's lookahead), *not* the main run (physical
execution). The advance run is up to `$ADVANCE` motion blocks ahead of where the
robot actually is. So:

- `MQ_RD_IDX` / `IDX` as written today is **early by up to `$ADVANCE` points**
  (currently 3 — see [kuka-stream-stall-fix](kuka-stream-stall-fix.md)).
- If Blender publishes `E_SPEED` for the point at `MQ_RD_IDX`, the extruder gets
  the new value ~3 segments **before** the nozzle physically arrives there.
- At high `$ADVANCE` / short segments the lead can be tens of mm — visible as
  early/late extrusion, blobbing at corners, etc.

### How precise is `MQ_RD_IDX` right now?

- **Granularity:** one waypoint (we only know "which segment", not sub-segment).
- **Phase error:** leads physical motion by 0..`$ADVANCE` segments (non-constant —
  depends on planner state, blending, speed).
- **Read latency:** + one C3 Bridge round-trip (~5–20 ms) + poll cadence.

Good enough for a coarse digital-twin highlight; **not** good enough for tight
extruder sync without correction.

### Fix for true main-run sync

Emit the index from the **main run** so it updates exactly when the robot passes
the point. Options, best first:

1. **`TRIGGER WHEN PATH=0 DELAY=0 DO IDX_RT = i`** — fires a fast assignment at the
   main-run instant the block starts/ends; can even offset by distance/time to
   pre-compensate extruder dead time. This is the proper KUKA mechanism for
   I/O-synchronous-to-path actions. Keep `MQ_RD_IDX` for the *producer* refill
   logic, add a separate `IDX_RT` for *consumer* attribute dispatch.
2. **Drop `$ADVANCE` to 1** during attribute-critical paths — removes the lead but
   re-introduces the start/segment stalls we just fixed; only viable at low speed.
3. **Compensate in Blender** — publish attributes for `MQ_RD_IDX + lead`/`- lead`
   with a tuned offset. Fragile (lead isn't constant), but zero robot-side change.

Recommendation: add an `IDX_RT` driven by `TRIGGER WHEN PATH` to the queue loop in
[krl_stream.py](../animaquina_core/runtime/krl_stream.py), poll **that** for MQTT
dispatch, and keep `MQ_RD_IDX` purely for buffer accounting. Allow a configurable
distance pre-trigger for extruder dead-time compensation.

## Benefits

1. **Robot program stays simple** — only motion + IDX, no variable soup
2. **Attribute-agnostic** — add any attribute to the mesh, it automatically flows through
3. **Single variable to poll** — less network traffic than reading N variables
4. **Clean separation** — robot does motion, Blender does dispatch, MQTT does delivery
5. **Works with any robot** — every robot can write one integer variable
6. **Backwards compatible** — existing E_SPEED/F_SPEED exports still work alongside IDX

## Open Questions

- [ ] Should IDX be a dedicated system variable or just another debug_var?
- [ ] For UR RTDE: use `output_int_register_0` (standardized) or a named variable?
- [ ] Should the dispatch handler run in the poll thread or on the main thread timer?
- [ ] MQTT: paho-mqtt dependency — bundle or require user install?
- [ ] Rate limiting: if robot moves fast, should we throttle MQTT publishes?
- [ ] Should we store dispatched values as Blender custom properties (keyframeable) or just in-memory?

## Relationship to Current Work

The debug_vars / Variables panel system we just built is the foundation:
- `read_var("IDX")` already works for all robot types
- The poll thread + timer callback already propagates variable values to Blender
- The IDX dispatch handler hooks into the same data flow

The custom_vars export integration we just added (writing per-point attributes
to the robot) remains useful as a fallback for robots/setups where Blender
can't poll in real time. But for the primary use case (live streaming with MQTT),
IDX dispatch is cleaner.
