# Animaquina — Frame Primitive Implementation Plan

## Goal
Add a **Frame** primitive to Animaquina: a reference coordinate system you register into the scene and drive from one or more external live sources (OpenCV, mocap, ARKit, anything). v1 ships the simplest useful case — one source, a single rigid pose — but the data model is shaped so multiple sources, many-point sets, and points-with-attributes are **additive extensions, not rewrites.** It establishes the primitive scaffold that Robot and later primitives reuse.

## Design principle
A primitive is **assembled from native Blender mechanisms, not a custom runtime.** For the v1 pose case, three native pieces carry it:

- **Parenting** = frame resolution (world pose falls out of the transform hierarchy)
- **Custom property** = the input socket the bridge writes into
- **Driver** = the wiring from property to transform channel

The addon never runs a simulation loop. It constructs these pieces, wires them, and shows a minimal UI. Live data arrives through the existing MQTT→property bridge — no new transport. Convention conversion and smoothing happen **upstream in the bridge**, so the input always holds a clean, Blender-frame value by the time a driver (or Geometry Nodes) reads it.

## The general model (what v1 is a slice of)
A **Frame is a reference coordinate system.** What flows into it is, in the general case, *a set of points carrying named attributes*. A single 6DOF pose is the **degenerate case** — one point that also carries orientation. Holding this model up front is what keeps v1 from painting us into a corner.

Two independent axes have to be respected by the model, even if v1 only exercises the corner of each:

- **Cardinality** — a source may yield one point or many (mocap markers, a hand skeleton, fiducials, a sparse cloud).
- **Attributes** — each point may carry more than position: `id`, `confidence`, `velocity`, `orientation`, a label.

Blender answers "what carries the samples" with **two native carriers**, and the split is real:

- **Pose carrier** — an empty. One rigid pose, driven, and crucially *parentable* (hang a camera, robot base, or geometry off it). Right when the job is "align my scene to this one tracked thing." **This is v1.**
- **Points carrier** — a point cloud / mesh with named attributes consumed by Geometry Nodes. Right for many points, variable-length sets, or points-with-data. This is the FabNodes attribute paradigm pointed at *input* instead of tool output. **Reserved extension.**

They compose under one Frame rather than competing:

```
Frame (base — reference coordinate system, parentable, user-positioned)
 ├─ Source "flange"    → Pose carrier    (child empty, driven, parentable)   [v1]
 ├─ Source "hand"      → Points carrier  (points + attributes → GN)          [future]
 └─ Source "fiducials" → Points carrier  (points + attributes → GN)          [future]
```

This mirrors USD/Isaac deliberately: a single tracked pose is an Xform prim; a point set with per-point data is a Points prim / PointInstancer with primvars — and primvars *are* named attributes. Native stacks keep both carriers because parentability and point-sets are genuinely different needs.

## Vocabulary (settled)
- **Frame** — the primitive: a **base** reference frame plus **one or more sources**
- **base** — the parent empty; the registered reference the user positions (dropped in scene, or parented to the robot flange)
- **source** — a named binding of one external stream to one carrier, parented to base
- **carrier** — what holds a source's samples; kind is **Pose** (v1) or **Points** (future)
- **pose carrier** — a child empty, driven by the stream, parentable — the single-6DOF degenerate case

Namespacing note: "frame" is overloaded in Blender (timeline `frame_current`, UI frames). Disambiguate in surfaced strings, not in the concept — operator id is namespaced (`animaquina.add_frame`), enum value is `'FRAME'`, and the add menu entry reads **"Add Frame (Tracked)"** or lives under the Animaquina submenu to carry context.

---

## Architecture — the primitive pattern (shared scaffold)
Every Animaquina primitive has the same anatomy, differing only in what sits between stream and geometry:

| Part | Role |
|------|------|
| Native datablocks | The actual Blender objects (Frame → base empty + carriers; Robot → armature) |
| Marker | One enum every object carries, declaring what it is |
| Type-specific property group | Fields the primitive needs + source bindings |
| Add operator | Constructor: builds datablocks, wires bindings in one step |
| Conditional panel | N-panel that branches on the marker to show only relevant controls |

Frame is the first instance because it exercises the entire pattern with the least machinery.

---

## Data model
The extensibility lives here: **source is a first-class, repeatable binding, and carrier kind is an enum** — so "more sources" and "the points carrier" are data/branch additions, not schema changes.

### Shared marker (Phase 0)
```python
class AnimaquinaPrimitive(PropertyGroup):
    primitive_type: EnumProperty(items=[
        ('NONE',  "None",  ""),
        ('ROBOT', "Robot", ""),
        ('FRAME', "Frame", ""),
    ], default='NONE')
    role: EnumProperty(items=[            # generalized: a carrier may be pose or points
        ('NONE',    "None",    ""),
        ('BASE',    "Base",    ""),
        ('CARRIER', "Carrier", ""),
    ], default='NONE')
# bpy.types.Object.animaquina = PointerProperty(type=AnimaquinaPrimitive)
```

### Frame + sources (Phase 1) — lives on the base
```python
class SourceBinding(PropertyGroup):
    name: StringProperty(name="Source")               # topic / stream key
    carrier_kind: EnumProperty(items=[
        ('POSE',   "Pose",   ""),                      # v1
        ('POINTS', "Points", ""),                      # reserved extension
    ], default='POSE')
    convention: EnumProperty(items=[                   # recorded; flip applied upstream in bridge
        ('BLENDER', "Blender (native)", ""),
        ('OPENCV',  "OpenCV",  ""),
        ('ROS',     "ROS/TF",  ""),
        ('ARKIT',   "ARKit",   ""),
    ], default='BLENDER')
    carrier: PointerProperty(type=bpy.types.Object)    # the empty (pose) or points object

class FrameProperties(PropertyGroup):
    sources: CollectionProperty(type=SourceBinding)    # v1 constructs one; N is a data change
    active_source: IntProperty(default=0)
# bpy.types.Object.animaquina_frame = PointerProperty(type=FrameProperties)
```

### Pose carrier input (Phase 1) — lives on the pose carrier empty
Driving `location` from a custom prop on the same object is not a cycle — driven channel and source prop are different data.
```python
class PoseInput(PropertyGroup):
    input_location: FloatVectorProperty(size=3, subtype='TRANSLATION')
    input_rotation: FloatVectorProperty(size=4, default=(1,0,0,0))   # quaternion w,x,y,z
# bpy.types.Object.animaquina_pose = PointerProperty(type=PoseInput)
```

The points carrier needs no analog here — its samples live in named point-domain **attributes** on the geometry, written directly by the bridge and consumed by GN. That's the whole reason it's a separate carrier.

Convention lives in the bridge by default (a driver expression is stateless, so smoothing there is awkward; an axis flip *could* be stateless but is cleaner upstream). The `convention` field is a record of what the source is — not a second place the math happens.

---

## Construction — `animaquina.add_frame` (Phase 2, v1 = pose carrier)
1. Create **base** empty. Mark `FRAME` / `BASE`, attach `FrameProperties`. This is what the user positions.
2. Append one `SourceBinding` to `base.animaquina_frame.sources` (default name, `carrier_kind='POSE'`).
3. Create **pose carrier** empty. Mark `FRAME` / `CARRIER`, attach `PoseInput`.
4. Parent carrier to base.
5. **Clear the parent inverse:** `carrier.matrix_parent_inverse = Matrix.Identity(4)`. Without this, carrier-local is relative to wherever base sat at parent time, not to base itself — wrong for a streamed pose. After clearing, `carrier.matrix_world == base.matrix_world @ carrier.matrix_basis`.
6. Set `carrier.rotation_mode = 'QUATERNION'`.
7. Add the transform drivers (Phase 3).
8. Point `source.carrier` at the pose empty.
9. Select base, make it active — user is immediately positioning the reference frame.

Empties should be visually distinguishable (base = larger `PLAIN_AXES`, pose carrier = `ARROWS`) so the pair reads at a glance.

A separate **`animaquina.add_source`** operator (append a `SourceBinding` + build its carrier and parent it to base) makes N-sources trivial. v1 can ship with or without exposing it in UI, but writing the constructor source-wise from the start is what keeps multi-source additive.

---

## Base is the handle — incoming data is always base-relative
The core behavior: the user moves base like any Blender object (grab, rotate, snap, or parent it to the robot flange) and everything the sources feed in rides along automatically. This is a property of the composition, not a feature to maintain:

- Incoming data writes into **base-local space** — the carrier's `matrix_basis` (pose) or the point positions (points). Base-local is a fixed relationship to base by definition.
- World position is only ever `base.matrix_world @ (local)`, and `base.matrix_world` is wherever the user last left it.
- Blender's depsgraph recomposes children whenever a parent's transform changes, so moving base re-lands all its data through the new matrix with **no recompute step to trigger.** You get it for free precisely because there is no custom world-space cache.

This is also why the cleared `matrix_parent_inverse` matters: it makes "base-local" mean *actually relative to base*, so the source origin coincides with base's origin. Uncleared, moving base would still work, but the stream's zero would sit at a frozen parent-at-parent-time offset and calibration would be off by that transform.

Two edges to handle so it behaves as pictured:

- **Base scale is a footgun.** Data is relative to base's *full* transform, so scaling base scales a points cloud with it. Sometimes intended (blow up the whole rig), often not (they scaled base just to enlarge the gizmo and warped their tracking). Recommendation: keep base scale at 1 and give the empty its own display-size property, so "make the gizmo bigger" never means "rescale my tracked data."
- **The carrier is output-only.** "Any object" means base — the thing they move — or an ancestor of it. Hand-moving the *carrier* fights the drivers, which overwrite the edit next tick. So in the UI, base is the grabbable reference and the carrier reads as "driven, don't touch." This is the one place to be opinionated.

---

## Driver wiring (Phase 3, pose carrier only)
Seven single-property drivers bind the input props onto the pose carrier's transform. Each expression is just the variable — pure passthrough.

- `location[0..2]` ← `animaquina_pose.input_location[0..2]`
- `rotation_quaternion[0..3]` ← `animaquina_pose.input_rotation[0..3]` (Blender normalizes on eval; raw components fine)

```python
def bind(obj, data_path, index, src_path, src_index):
    fc = obj.driver_add(data_path, index)
    drv = fc.driver
    drv.type = 'SCRIPTED'
    var = drv.variables.new()
    var.name = 'v'
    var.type = 'SINGLE_PROP'
    tgt = var.targets[0]
    tgt.id = obj                                # same object; not a cycle
    tgt.data_path = f'{src_path}[{src_index}]'
    drv.expression = 'v'
```

Depsgraph note: drivers only re-evaluate when something tags an update — the **same** consideration the MQTT bridge already handles for its properties. If the bridge tags the ID / runs on its timer tick when new data lands, the drivers fire. No new mechanism.

The points carrier has **no drivers** — the bridge writes attributes onto the point domain and tags the update; GN re-evaluates. That's simpler wiring but means the bridge manages geometry (see Phase 5).

---

## UI panel (Phase 4)
N-panel (Animaquina tab) branching on `obj.animaquina.primitive_type`. For a `'FRAME'` base, show:

- The source list (v1: one row). Each row: name, carrier kind (Pose), convention.
- Per-source live readout — current `input_location` / `input_rotation` so the user sees the stream is alive.
- A "re-register" affordance: snap base to the current world pose of a target (selected object / robot flange) for quick alignment.

For a carrier object, show which Frame and source it belongs to, and a link back to base. Keep it minimal — inspection + binding, not a control surface.

---

## Bridge integration (Phase 5)
Reuse the existing MQTT→property bridge. It now **iterates a frame's sources** rather than assuming one:

- Resolve each `source.name` → topic.
- **Pose carrier:** apply convention flip + smoothing upstream, then write `input_location` / `input_rotation` on the carrier empty; tag the ID.
- **Points carrier (extension):** write named attributes onto the point domain; tag the ID. This is where the bridge grows from "write a property" to "manage geometry" — allocation and count changes (below).

Data flow (v1, pose):
```
source (OpenCV / mocap / ...) → MQTT → bridge
    → [convention flip] → [smoothing]        # upstream, stateful, in the bridge
    → input property (clean Blender-frame pose)
    → driver (stateless passthrough)
    → carrier local transform
    → parenting (base.matrix_world @ carrier.matrix_basis)
    → world pose
```

---

## Polish (Phase 6)
- Menu entry under Add → Animaquina → **Frame (Tracked)**; keep the qualified label near Blender's native controls.
- Icons / empty display sizes for legibility.
- Save/load robustness: native datablocks + drivers + props survive save/load for free — verify anyway, especially driver targets and `carrier` pointers after append/link.
- Guard the operator against re-running on an existing primitive; handle undo cleanly.

---

## Extensibility map — designed-in vs. deferred
| Axis | v1 ships | Extension path (no schema rewrite) |
|------|----------|-----------------------------------|
| Sources per frame | One | `sources` is a collection + `add_source` operator; add rows |
| Carrier kind | Pose (empty, drivers) | Add `POINTS` branch in operator + bridge; new `PointsInput`/attributes, GN consumption |
| Cardinality | One point | Points carrier holds N via point domain |
| Point attributes | position + orientation | Named attributes on the point domain (`id`, `confidence`, `velocity`, label…) |

Two things that become load-bearing the moment the points carrier lands, noted now so the attribute contract is designed with them in mind:

- **`id`** — correspondence across ticks (point 3 now = point 3 last tick) is what lets you track, per-point smooth, or drive a specific instance. Past a single point, `id` is not optional.
- **Variable count** — sources that gain/lose points shouldn't reallocate geometry every tick. Fixed capacity + a `validity`/`id` mask is the stable realtime choice and plays nicer with GN than resizing.

---

## Open decisions to nail before building
1. **Where pose input props live** — on the pose carrier empty (assumed) vs. a dedicated data object. Carrier empty is simpler and inspectable; confirm.
2. **Live vs. bakeable** — Frame is real-time alignment by default. Add a "bake to keyframes" path (sampling inputs per frame) for toolpath-synced record/playback? Not required for v1; the model supports adding it without change.
3. **`add_source` in v1 UI** — build the source-wise constructor now (yes), but expose multi-source in the panel in v1 or defer the UI?
4. **Points carrier attribute contract** — when it lands, fix the required/optional attribute set (`position`, `id`, `validity`, then `orientation`/`confidence`/`velocity`) so bridge and GN agree. Draft now, implement later.

---

## Why this generalizes (Robot follows)
Robot is the *same* anatomy — marker, property group, add operator, panel — with an armature and an IK solve between stream and geometry instead of a passthrough. Frame locks the pattern; Robot becomes a variation on one construction path, one UX, one mental model. And the points carrier ties the input side back to FabNodes' attribute paradigm, so input and tool-output share one way of thinking. That convergence — Frame, Robot, and FabNodes as members of one attribute-and-transform family — is the payoff of the primitive framing.
