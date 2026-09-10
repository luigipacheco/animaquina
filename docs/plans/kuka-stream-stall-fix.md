# KUKA Dynamic Sync — toolpath stall fix

## Symptom

When running a toolpath in the KUKA Dynamic Sync streaming mode, the robot
sometimes stalls for ~1 second as if waiting to load points. Notably it tends to
**stall at the very beginning and then run smoothly** afterwards.

## Root cause

Producer/consumer ring buffer (`MQ_PT[]`) over the C3 Bridge / KukavarProxy:

- Robot (consumer) walks `MQ_PT[]`, incrementing `MQ_RD_IDX`
  ([krl_stream.py](../animaquina_core/runtime/krl_stream.py), `MQ_ACTION==10/11`).
- Blender (producer) refills free slots, bumps `MQ_WR_IDX`
  (`stream_refill` in [kuka_driver.py](../animaquina_core/drivers/kuka_driver.py),
  driven by the modal tick in [operators.py](../animaquina/ui/operators.py)).

When `MQ_RD_IDX` catches `MQ_WR_IDX` the KRL hits
`WAIT FOR MQ_WR_IDX > MQ_RD_IDX`, which forces an **advance-run stop** — the robot
decelerates to a full standstill, then re-accelerates. That is the visible stutter.

Why it starves, especially at the start:

1. **1:1 advance:buffer ratio.** `$ADVANCE = 5` with buffer 5 → the planner's
   lookahead alone swallows the entire buffer the instant motion begins, so
   `MQ_RD_IDX` races to `MQ_WR_IDX` and hits `WAIT FOR` once, at the start, before
   the producer primes the pipeline. After priming, steady-state production
   (~1 point per C3 round-trip) outpaces consumption → smooth. → "stall then fine".
2. **First refill latency.** Refill only happened on the next modal tick (0.05 s)
   after `MQ_ACTION=10`, giving the planner time to drain the buffer first.
3. **Per-point round-trips + poll contention.** Each `MQ_PT[i]` write is one
   synchronous KukavarProxy transaction; the background poll thread shares the same
   socket/lock (see item 3 below) — adds intermittency ("sometimes").

## Done (items 1 & 2) — 2026-06

- **Advance/buffer margin.** `$ADVANCE` is now configurable per slot
  (`kuka_advance`, default **3**) and the ring buffer default is **6**, giving a
  2:1 margin. `generate_src(advance=...)` clamps advance to `[1,5]` and to
  `buffer-1` so the planner can never drain the whole ring.
  Files: krl_stream.py (`ADVANCE_DEFAULT`, `generate_src`), kuka_driver.py
  (`upload_stream_program(advance=)`), kuka_stream_runtime.py
  (`stream_krl_text(advance=)`), properties.py (`kuka_advance`, buffer default 6),
  panels.py (UI), operators.py (upload + both streaming paths).
  *Requires re-upload of `mq_stream` and updating `MQ_PT[]` size in `$CONFIG.DAT`
  when buffer changes — existing mismatch warning covers this.*
- **Prime before motion.** After `stream_start`, a short time-bounded loop
  (~60 ms) tops up freed ring slots so `MQ_WR_IDX` gets ahead of the advance run
  before steady-state refilling begins → removes the start-of-path stall.
  Also lowered the KUKA streaming modal tick 0.05 s → 0.02 s.
  File: operators.py `_kuka_modal_tick` streaming transition.

## Done (item 3 + host-side stall) — 2026-08-23

Item 3 is done, and investigating it turned up a second, much worse failure that
shares the same cause. Two different symptoms were being conflated:

- **Robot stalls** (~1 s hesitation mid-path) — the ring-buffer starvation above.
- **Host stalls** (Blender freezes, then the whole machine's networking gets
  sluggish and stays that way) — this section. Same class of bug as the UR
  `stopScript()` leak in 0f029d1: a connection that is never actually healthy
  being rebuilt over and over.

### Root cause of the host stall

`kukaproxydriver.KUKA.read()`/`write()` set `self.connected = False` on **any**
exception, then the next call transparently reconnects. But a variable that is
simply **not declared in `$CONFIG.DAT`** raises exactly the same way as a dead
socket. So one undeclared name poisoned a perfectly healthy link:

1. Poll thread reads the variable → fails → link marked dead.
2. Next read → `connect()` → **a brand-new TCP socket**.
3. Poll thread reads the same variable again next cycle → repeat, at the poll
   rate, forever.

Every cycle tore down and rebuilt the C3 Bridge connection. On Windows each
closed socket sits in `TIME_WAIT` for ~4 minutes, so ephemeral ports run out and
*all* networking on the machine degrades — which is why it reads as "the
computer is stalling" rather than "the add-on is slow", and why it outlives the
toolpath run.

**What made it start happening now:** `ff6a524` auto-added the point-index
variable to `slot.debug_vars` so `run_idx` would work. That was correct on its
own, but it means a KUKA slot with "Write Point Index" enabled now polls `IDX`
every cycle — and if `IDX` is not declared in `$CONFIG.DAT`, that is precisely
the undeclared-variable case above. Before that commit `debug_vars` was empty
unless the user added a name by hand, so the trigger was rarely pulled.

`connect()` also assigned over `self.client` without closing the previous
socket, and `send()` used `send()` rather than `sendall()` with a single
`recv(1024)` — a reply split across TCP segments came back truncated, failed to
parse, and fed the same reconnect loop.

### Fixes

- **Error classification** (kukaproxydriver.py). New `VariableError` for
  variable-level failures; only a real `socket.error` marks the link dead.
  An undeclared variable is now reported and stepped over, never reconnected.
- **Transport hygiene** (kukaproxydriver.py). `connect()` closes any existing
  socket before replacing it and refuses to retry faster than
  `RECONNECT_MIN_INTERVAL` (0.5 s). `send()` uses `sendall()` and reads the
  reply framed by its own length header, so segmented replies no longer desync.
- **Variable quarantine** (manager.py). A debug variable that fails
  `MAX_VAR_FAILURES` (5) times in a row leaves the poll rotation, still
  reporting why. An undeclared name costs 5 round-trips, not one per cycle.
- **Poll back-off** (manager.py). Consecutive failed cycles back off
  exponentially to `MAX_ERROR_BACKOFF` (2 s) instead of retrying at full rate.
  Also fixes the guard that gated this: `data` is never empty (the debug-var
  keys are written every cycle), so `not data` was always False and the error
  branch could never fire — errors were never even surfaced. Now tracked with
  an explicit `pose_ok`.
- **Poll throttle during streaming** (item 3 proper). `set_poll_rate` /
  `restore_poll_rate` in manager.py, `_SlotPollThread.set_rate()`; the KUKA
  streaming operator drops polling to `_STREAM_POLL_RATE_HZ` (5 Hz) for the
  duration of a run and restores it in `_send_path_cleanup` — including on the
  error and user-stop paths. The twin stays live; refill wins the socket.
  Chose "throttle", not "drive the twin from MQ_RD_IDX": it keeps showing the
  *actual* pose, and it is the smaller change.
  The poll interval is now re-read each cycle — it was computed once before the
  loop, so any rate change would have had no effect.
- **No orphaned sessions** (manager.py). `connect_slot()` tears down an existing
  driver/thread for the same uid instead of assigning over the map entries. The
  poll loop only ever exits on its stop event, so an orphaned thread used to
  keep polling with its own socket for the rest of the Blender session.
- **Don't write the index variable back** (operators.py). The auto-added
  point-index entry is skipped by `_read_custom_var_attributes`. The robot
  writes `IDX` itself from a point-synchronized `TRIGGER`; the add-on driving it
  from a mesh attribute of the same name fought the controller for the variable
  and rebuilt a full per-point list on the UI thread every tick.
- **Windowed live re-read** (operators.py). `_kuka_read_live_waypoints` takes
  `(start, count)` and returns `(total, window)`. Streaming re-read the entire
  toolpath every 0.02 s — two matrix products and several allocations per point,
  50×/s, on Blender's UI thread — to then send at most `ring_size` points. For a
  few thousand points that is millions of allocations per second on its own:
  the viewport freeze and the "memory leak" feel. `total` is still returned so
  the topology-change check is unchanged.

### Note for users

If `run_idx` / MQTT dispatch is wanted, `IDX` (or whatever
`export_point_index_var` is set to) **must be declared in `$CONFIG.DAT`**.
It is no longer harmful if it isn't — the variable is just reported as not
readable — but the index will not update until it is declared.

### Still open

`_ur_read_live_waypoints` has the same full-mesh-per-tick shape on the UR
streaming path. Its tick is 0.1 s rather than 0.02 s so the cost is ~5× lower,
but it is worth the same windowing treatment.

## Original TODO (item 3) — throttle poll, keep the digital twin live

Decided to defer; test items 1 & 2 first. The remaining mid-path intermittency
comes from the background `_SlotPollThread` ([manager.py](../animaquina/manager.py))
reading `$POS_ACT`/`$AXIS_ACT` on the **same socket/lock** as refill writes.

**Constraint:** the digital twin must stay updated during streaming — do **not**
fully pause polling.

Two options (can combine):

- **Throttle, don't pause.** While a stream is active, drop the poll rate
  (e.g. 50 → ~10 Hz) and restore on completion. Twin still updates ~10×/s; refill
  wins the socket far more often. Add `pause_poll`/`resume_poll` (really
  `set_stream_rate`) hooks in manager.py; call from the streaming operator
  start/cleanup.
- **Drive the twin from `MQ_RD_IDX`.** The refill loop already reads `MQ_RD_IDX`
  every cycle — update the twin to that commanded waypoint pose (zero extra socket
  reads) during streaming, resume `$POS_ACT`/`$AXIS_ACT` polling when done. Lowest
  latency twin, removes contention entirely. Caveat: shows *commanded* not *actual*
  pose, and `MQ_RD_IDX` leads physical motion (see precision note below).

## Related: MQTT per-point attribute streaming

User wants to use the current point index to stream per-point attributes (e.g.
extruder speed) to peripherals over MQTT. This is the existing
[idx-attribute-dispatch](idx-attribute-dispatch.md) plan.

**Precision caveat (important):** `MQ_RD_IDX` (and a plain `IDX = i`) advances in
the **advance run**, so it leads physical robot position by up to `$ADVANCE`
points (now 3). Fine for a coarse twin highlight; for tight extruder sync use a
main-run `TRIGGER WHEN PATH` index (`IDX_RT`) with optional distance pre-trigger
for extruder dead-time compensation. Full details added to the
[idx-attribute-dispatch](idx-attribute-dispatch.md) plan ("Timing precision"
section).
