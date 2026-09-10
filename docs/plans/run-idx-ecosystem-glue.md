# RUN_IDX Ecosystem Glue — Animaquina → PhyNodes → FabNodes

> Strategy plan. This is where the three projects meet: Animaquina runs a
> toolpath, PhyNodes publishes the current point's attributes to the network,
> FabNodes hardware consumes them, and FabFlow shows Blender as one more node
> in the flow. Builds on [idx-attribute-dispatch](idx-attribute-dispatch.md)
> (robot-side IDX mechanics) — this plan covers the Blender-side contract and
> the network-side manifest.

## User story

1. I select a toolpath object and hit Run (sim playback *or* live stream).
2. As the run advances, Blender always knows the **current point index**.
3. A PhyNodes graph reads that index, looks up any per-point mesh attribute
   (`E_SPEED`, `F_SPEED`, `fan`, `my_custom`…) at that index, and publishes it
   over MQTT with fabnodes-flat topics.
4. A fab-struder / fab-servo / any FabNode reacts in real time.
5. In FabFlow I see a **Blender** node with those signals as ports — I can
   inspect it, wire it, and monitor it like any ESP32 node.

## Division of responsibilities (the core decision)

| Layer | Owns | Explicitly does NOT own |
|---|---|---|
| **Animaquina** | Producing a canonical `run_idx` per robot slot, for every run mode | Networking. No MQTT dependency, no paho. |
| **PhyNodes** | Attribute lookup at `run_idx`, value shaping (map/clamp/curve), MQTT publish, the FabNodes manifest | Robot control, toolpath semantics |
| **FabNodes / FabFlow** | Consuming signals, discovery/monitoring UI | Anything Blender-specific beyond rendering the manifest |

Rationale: keeps Animaquina lean (its poll thread + properties are the whole
contribution), keeps all transport code in PhyNodes where the connector layer
already exists (MQTT/OSC done, Zenoh next), and makes Blender look like just
another node to the hub — no special-casing in FabFlow.

**Strategy change vs. idx-attribute-dispatch:** that plan sketched an "IDX
Dispatch Handler" inside Animaquina that looks up attributes and fires MQTT.
We move the lookup + publish into the PhyNodes graph instead. Animaquina only
exposes the index. The in-Animaquina dispatch handler is deferred (still
useful later for driving slot properties / UI without PhyNodes installed).

## 1. Animaquina — the `run_idx` contract

Add to `ANIMAQUINA_RobotSlot` (properties.py):

```python
run_idx: IntProperty(name="Run Index", default=-1)     # -1 = not running
run_source: EnumProperty(items=[('NONE',...), ('SIM',...),
                                ('STREAM',...), ('PROGRAM',...)], default='NONE')
run_object: PointerProperty(type=bpy.types.Object)     # the toolpath being run
run_count: IntProperty(default=0)                      # total points (for % / progress)
```

`run_idx` is the **single canonical property** — the stable data path PhyNodes
reads:

```
bpy.context.scene.animaquina.robots[0].run_idx
```

Three producers, all normalizing into the same property:

### a) Sim playback (no robot needed — this makes the whole workflow testable offline)
The Blender-sim path playback already animates `sample_index` on the
`animaquina_sim_geo_attr` constraint (operators.py ~line 290). On run, set
`run_object`/`run_count`/`run_source='SIM'`; a lightweight
`frame_change_post` handler (or the existing playback machinery) copies the
current sample_index into `slot.run_idx`. On Clear Path, reset to -1/'NONE'.

### b) Live streaming
- KUKA: the stream tick already reads `MQ_RD_IDX` (operators.py ~2281). Write
  it into `slot.run_idx` each tick. For main-run accuracy, poll `IDX_RT`
  (the `TRIGGER WHEN PATH=0` variable from
  [idx-attribute-dispatch §timing](idx-attribute-dispatch.md)) once it lands —
  the property contract doesn't change, only which robot variable feeds it.
- UR: feed `waypoints_completed` (not `servo_sent` — same distinction the
  return-phase logic already makes) into `slot.run_idx`.

### c) Program run (exported program, robot polled)
The exported program writes `IDX` (already in the KRL template,
krl_stream.py:67). The poll thread reads it as a debug var; the `_poll_all`
timer callback additionally mirrors it into `slot.run_idx` when
`run_source == 'PROGRAM'`.

Notes:
- Update `run_idx` **only from the main thread** (timer callback / frame
  handler), same rule as the rest of the poll pipeline.
- Only write when the value changed (avoid depsgraph churn at 50 Hz).
- Progress % and "running" state for UIs derive from `run_idx`/`run_count` —
  also fixes the UI question "how far along is the run" for free.

## 2. PhyNodes — index-driven attribute lookup

### a) Geometry Attribute node: promote Index to an input socket
`nodes/attribute_node.py` has `index: IntProperty` but no socket. Add an
`Index` input socket (INT/ANY); when connected it overrides the property
(standard GN behavior — unconnected socket shows the property value). That's
the only change needed for the core loop:

```
[Property In: ...robots[0].run_idx] ──► [Geometry Attribute: toolpath obj,
                                        "E_SPEED", Index socket]
                                            │
                                            ▼ (optional Map / Clamp / Curve)
                                       [MQTT PUB: fab-struder1/feed/speed]
```

- Clamp negative index (run_idx = -1 when idle) → hold 0 / last value —
  decide behavior (see open questions).
- `Property In` returns strings for string props; `run_idx` is an int so the
  existing path works untouched.

### b) Optional sugar (later): a "Toolpath Run" node
One node with an object picker + slot picker outputting `Index`, `Running`,
`Progress` — wraps the data-path fiddliness. Pure convenience over (a); (a)
ships first.

## 3. PhyNodes — Blender as a FabNode (manifest)

Make a PhyNodes MQTT connection able to announce itself under
**fabnodes/1.1** so FabFlow discovers Blender like any ESP32:

- Per-MQTT-connection toggle in the N-panel: **"FabNode identity"** +
  `nodeName` (e.g. `blender1`), `nodeType: "fab-blender"`.
- On connect, walk the node tree(s) using this connection and derive signals:
  - each **MQTT PUB** node → `{"topic": ..., "dir": "pub", "dtype": ...}`
  - each **MQTT SUB** node → `{"topic": ..., "dir": "sub", "dtype": ...}`
  - dtype inferred from the connected socket type; fall back to "string".
- Publish retained `blender1/$info` + mirror `fabnodes/manifest/blender1`,
  `$state` via LWT (`online`/`offline`), diag heartbeat on the standard 15 s
  cadence (FabFlow greys out nodes without it).
- Re-publish the manifest when the graph topology changes (cheap: hash the
  topic list each evaluator sweep, publish on change).
- **E-stop:** the connection subscribes `system/estop`; while latched, PUB
  nodes suppress outbound control values (protocol rule: nodes fail safe).
  Surface the latch in the N-panel header. Optionally (later) also trigger
  Animaquina's stop operator.
- FabFlow needs zero changes for discovery; a `fab-blender` icon/style is a
  nice-to-have in `hub/flow`.

This honors the protocol constraints: control topics not retained, flat
topics, additive v1.x change (a manifest from a non-ESP32 node is just
another manifest).

## 3b. Attribute wiring model — arbitrary, user-named attributes

A toolpath is `position` + **any number of user-named per-point attributes**
(`l_speed`, `led_color`, `fan_state`, `my_thing`…). Nothing in the stack may
hardcode attribute names. The mental model: *every attribute gets wired to a
consumer*, and there are exactly three consumer types:

| Consumer | Example | Wiring surface | Status |
|---|---|---|---|
| **1. Network signal** (fabnode actuator) | `led_color` → `fab-led1/colors`, `fan_state` → `fab-struder1/fan/speed` | **PhyNodes graph**: one Geometry Attribute node per attribute (dropdown = any name), all fed by the same `run_idx` → shaping nodes → MQTT PUB | ✅ works (Phase 2) |
| **2. Controller variable** (robot program reads it) | `E_SPEED` attribute → KRL variable `E_SPEED` | **Name match**: an attribute named like an enabled debug variable is streamed to the robot per point (`_read_custom_var_attributes` → `_write_custom_vars_for_index`) | ✅ exists |
| **3. Motion parameter** (drives the run itself) | `l_speed` → per-point linear velocity, `blend` → per-point radius | **Per-channel "from attribute"** (decided — mirrors Per-Point Orientation): a toggle + attribute-name field per motion channel | ✅ speed on UR (see below) |
| | | | |

Notes on each:

1. **Network signals** need no convention: the user picks the attribute in the
   node and the topic in the PUB — the graph *is* the patch bay. Vector/color
   attributes already flow as arrays (`[r,g,b,a]`), which matches the
   fabnodes array payload format. Multiple attributes = multiple Attribute
   nodes sharing one run_idx wire. Later sugar (Phase 5): a **Toolpath Run**
   node with one dynamic output socket per attribute of the picked object, so
   N attributes don't need N nodes.
2. **Controller variables** stay as-is — it's the robot-side analog of a
   subscription, and it's already name-agnostic (name the attribute after the
   variable).
3. **Motion channels — decided: per-channel "from attribute"** (not a generic
   bindings table). Mirrors the existing *Per-Point Orientation* pattern the
   user already knows: each motion channel gets a toggle + an editable
   attribute-name field (so `l_speed`, `speed`, anything works — the name is
   the user's, the *channel* is Animaquina's). A generic mapping table is
   overkill for the ~3 channels that exist (speed, acc, blend) and harder to
   discover; revisit only if channels multiply.

   **Per-Point Speed — ✅ implemented (UR buffered).** Motion panel →
   Linear (movel) → *Per-Point Speed* checkbox + attribute name (default
   `speed`). Values are m/s; the segment *into* waypoint i uses `speeds[i]`;
   missing / non-positive entries fall back to the constant Speed; capped at
   2.0 m/s. Live-updates while streaming (attribute edits mid-run take
   effect, same as positions). Implementation: `_ur_read_live_waypoints`
   reads the attribute → `stream_start(..., speeds=...)` →
   `state["speeds"]` consumed per segment by `_servo_worker`
   (`_segment_steps` + servoL args). Exact, no robot-side change.

   Still open per channel:
   - **KUKA Dynamic Sync speed**: needs per-point VEL in the ring buffer
     (extend `MQ_PT[]` schema) or `$VEL.CP` writes in the queue loop —
     advance-run caveats apply; do after `IDX_RT`.
   - **Blend / acc from attribute**: same toggle+name pattern when needed.
   - **Exports** (URScript movel v=, KRL $VEL.CP): wire the same attribute
     into program generation so exported programs match streamed behavior.
   - Bound attributes are consumed *by* the run; unbound attributes remain
     available to consumers 1 and 2. The same attribute may feed several
     consumers at once (e.g. `l_speed` drives motion *and* is published for
     monitoring).

An attribute's *name* is thus never magic — only its *binding* is: bind it in
Animaquina (motion), name-match it (controller var), or wire it in PhyNodes
(network). A future "Toolpath Attributes" panel could list every attribute of
the selected path with its current wiring (motion / var / graph / unbound) —
the patch-panel overview.

## 4. Phases

**Phase 1 — the contract (Animaquina only). ✅ implemented.**
(`run_state.py`; producers wired in sim Validate Path, KUKA/UR streaming
modals, and `_poll_all` for polled IDX/IDX_RT; run state shown in the
Variables panel.)
`run_idx`/`run_source`/`run_object`/`run_count` on the slot; wire sim
playback + KUKA stream + UR stream + program-poll producers. Show
`run_idx` in the robot panel. *Deliverable: scrub the sim timeline and watch
run_idx move.*

**Phase 2 — the loop (PhyNodes). ✅ implemented.**
(Geometry Attribute node: `Index` input socket, enabled when All Elements is
off; legacy nodes without the socket keep using the old property.)
Index input socket on Geometry Attribute. *Deliverable: sim playback in
Blender live-drives a fab-struder over MQTT with zero robot hardware.*

**Phase 3 — the manifest (PhyNodes + hub).**
FabNode identity on the MQTT connector; retained manifest + $state + heartbeat
+ e-stop latch. *Deliverable: Blender appears in FabFlow, wireable and
monitored.*

**Phase 4 — accuracy (Animaquina, robot-side).**
`IDX_RT` main-run trigger for KUKA per idx-attribute-dispatch; configurable
distance pre-trigger for extruder dead time. *Deliverable: extrusion changes
land where the nozzle is, not $ADVANCE segments early.*

**Phase 4b — Motion channels from attributes (Animaquina).**
Per-channel toggle + attribute name (§3b). ✅ *Per-Point Speed on UR buffered
implemented* (`use_speed_attribute` / `speed_attribute` on the slot, branch
`fabnode-connector`). Remaining: KUKA per-point VEL after `IDX_RT`; blend/acc
channels; same attributes honored in program exports.

**Phase 5 — sugar.**
"Toolpath Run" node with dynamic per-attribute output sockets; "Toolpath
Attributes" patch-panel overview (§3b); in-Animaquina dispatch handler
(attributes → slot custom props) for PhyNodes-less setups; FabFlow styling
for fab-blender.

## 5. How to use it (Phases 1–2, current state)

### A. Offline first — sim playback drives the loop (no robot needed)

1. Reload both addons (Animaquina and PhyNodes) in Blender.
2. Select your toolpath mesh (needs a `position` attribute; per-point
   attributes like `E_SPEED` come along automatically).
3. In Animaquina: **Validate Path** (Blender IK sim). This keyframes the
   playback and marks the run: `run_source = SIM`, `run_object` = the path,
   `run_count` = point count.
4. Press play / scrub the timeline. The **Variables** panel (Debug section)
   now shows `Run [SIM]  IDX n / total`. That IDX is the contract property:
   `bpy.context.scene.animaquina.robots[0].run_idx`
   (swap `[0]` for the slot index of that robot).
5. **Clear Path** or **Clear Simulation** resets it to -1 / NONE.

### B. The PhyNodes graph (attribute → MQTT)

1. PhyNodes N-panel: add an MQTT connection (broker = the fabnodes hub,
   `fabnodes`/`fabnodes`) and connect.
2. In a PhyNodes tree, add:
   - **Property In** — data path `bpy.context.scene.animaquina.robots[0].run_idx`
   - **Geometry Attribute** — pick the toolpath object and the attribute
     (e.g. `E_SPEED`), **uncheck All Elements** — an `Index` input socket
     appears — and wire Property In → Index.
   - **MQTT PUB** — topic e.g. `fab-struder1/feed/speed`, wire Attribute → PUB.
   - Insert Map / Clamp / Float Curve between Attribute and PUB to shape
     values into the actuator's range.
3. Play the timeline: the fab node follows the toolpath attributes point by
   point. Idle (`run_idx = -1`) clamps to point 0 in the Attribute node.
4. Old files: Geometry Attribute nodes saved before the socket existed keep
   their fixed Index property; re-add the node to get the socket.

### C. Live robot runs

- **UR (Run Toolpath (Buffered))**: `run_idx` follows `waypoints_completed`
  automatically — nothing to configure. Same PhyNodes graph as above.
- **KUKA (Dynamic Sync streaming)**: `run_idx` follows `MQ_RD_IDX - 1`
  automatically. Advance-run lead applies until Phase 4 (`IDX_RT`).
- **Exported program on the controller**: add a debug variable named `IDX`
  (or `IDX_RT`) in the Variables panel with Realtime polling on; the poll
  loop mirrors it as `run_source = PROGRAM`. (UR exports: the register write
  + `output_int_register_0` mapping is still Phase 4 work.)

### D. Per-point speed (UR buffered)

1. Add a float point attribute to the toolpath mesh (any name — `speed`,
   `l_speed`…) with values in **m/s** (e.g. via Geometry Nodes Store Named
   Attribute, or the Attribute panel).
2. Motion panel → Linear (movel): enable **Per-Point Speed** and type the
   attribute name.
3. Run Toolpath (Buffered): each segment moves at the attribute value of the
   waypoint it targets. Points with a missing / zero / negative value use the
   constant Speed; values are capped at 2.0 m/s. Edits to the attribute
   during the run take effect live (like position edits).

### Speed settings for Run Toolpath (Buffered) — shared linear params

The buffered UR run takes its speed from the **same** linear motion settings
as Move to Target: Motion panel → *Linear (movel)* → **Vel (m/s)** /
**Acc (m/s²)** / **Radius (m)** (`slot.speed` / `slot.acc` / `slot.radius`).
There is no separate parameter to sync — `ur_export_vel` / `ur_export_acc`
only affect *exported* URScript files, not streaming.

> **Bug fixed (2026-07):** buffered runs ran ~4× the commanded speed on
> e-Series. The servoL worker assumed an 8 ms cycle (125 Hz) when slicing
> segments into substeps, but `waitPeriod()` paces the loop at the real RTDE
> control frequency — 500 Hz (2 ms) on e-Series — so substeps fired 4× too
> fast. `_servo_worker` now reads `ctrl.getStepTime()` and uses the
> controller's true period for both the substep math and the servoL time
> argument. CB3 robots (native 125 Hz) were unaffected.

## 6. Open questions

- [ ] `run_idx` when idle: hold last value or reset to -1? (-1 proposed; PUB
      graphs must then decide hold-last vs. publish-safe-value — maybe the
      Attribute node's clamp handles it, maybe a dedicated "Latch" node.)
- [ ] Multiple slots running simultaneously: fine (per-slot property), but the
      manifest/nodeName is per-connection — one Blender identity per broker,
      or per scene?
- [ ] Publish cadence: evaluator is 50 ms pull; publish only on `run_idx`
      change (natural rate limit) or every sweep?
- [ ] Should the manifest also advertise `run_idx`/progress themselves as pub
      signals (`blender1/run/idx`) so FabFlow can show run progress without
      any user graph? (Cheap and probably yes.)
- [ ] Sim playback source of truth: read the constraint's animated
      sample_index vs. compute from frame + point count — pick whichever
      survives Clear Path cleanup most robustly.

## Relationship to existing plans

- [idx-attribute-dispatch](idx-attribute-dispatch.md) — robot-side IDX
  mechanics and the advance-run timing analysis; its "IDX Dispatch Handler"
  is superseded by the PhyNodes graph (kept as Phase 5 option).
- [kuka-stream-stall-fix](kuka-stream-stall-fix.md) — $ADVANCE constraints
  that motivate `IDX_RT`.
- `phynodes/plan.md` §2.5 ("Animaquina-ready") and §5 (Zenoh next) — when
  Zenoh lands, the same graph publishes over Zenoh unchanged; the manifest
  concept can follow (`fabnodes/manifest` key expressions map 1:1).
- `ix-nodes/README.md` §5 — manifest format this plan targets (fabnodes/1.1).
