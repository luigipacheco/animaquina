# Animaquina — User Guide

**Animaquina** is a Blender add-on for controlling industrial robots directly from the 3D viewport. It gives you:

- a **live digital twin** — the 3D robot model mirrors the real robot in real time
- **interactive motion** — move the robot to a target empty, or follow it continuously (Puppet Mode)
- **toolpaths from geometry** — turn any mesh or Geometry Nodes output into robot motion
- **offline program export** — generate URScript (UR) or KRL (KUKA) programs, review them, and upload them to the controller

Supported robots: **Universal Robots** (UR3–UR30, e-Series / PolyScope 5 / PolyScope X), **KUKA** (via C3 Bridge), and **UFactory xArm / UF850**.

> **This is a beta build for Blender 5.2 LTS.** Expect rough edges. Read the [Safety & Beta Terms](#safety--beta-terms) section before connecting to real hardware.

## 💬 Support & Community

Join the Animaquina Discord for setup help, bug reports, and beta discussion:

**👉 https://discord.gg/pM2cauqadZ**

This is the fastest way to get help and the preferred channel for beta feedback.

---

## Quick Start (the 10-minute version)

1. **Install** the Animaquina `.zip` in Blender 5.2 LTS (**Edit > Preferences > Add-ons > Install from Disk...**), enable it, restart Blender.
2. Press **N** in the 3D Viewport and open the **Animaquina** tab.
3. **Append a robot rig** from the bundled `robots.blend` (see [Robot Asset Library](#robot-asset-library)).
4. Put your PC on the **same network** as the robot and confirm you can `ping` its IP.
5. In **Robot Registry**, click **Add**, then in **Setup** choose the robot type/model, assign the rig collection, enter the IP, and click **Connect**.
6. Enable **Polling** — the 3D model should now mirror the real robot.
7. Place the **Target** empty somewhere reachable and click **Move to Target**. 🎉

Each step is explained in detail below.

---

## Requirements

- **Windows**
- **Blender 5.2 LTS** — the bundled robot drivers are built for its Python 3.13; older Blender versions will not load them
- Network access to the robot controller (same subnet or direct Ethernet)

## What's In Your Package

You should have received:

- the **Animaquina beta zip** (do not unzip it — Blender installs the zip directly)

Every feature and every supported robot brand is enabled. There is no licence key or activation.

---

## Installation

### 1. Install the add-on

1. Open Blender 5.2 LTS.
2. Go to **Edit > Preferences > Add-ons**.
3. Click **Install from Disk...** and select the Animaquina beta zip.
4. Enable the **Animaquina** add-on if Blender doesn't enable it automatically.
5. Restart Blender once after install.

### 2. Verify the add-on loaded

Press **N** in the 3D Viewport and open the **Animaquina** tab. If the panel is
there, you are ready. If the add-on fails to enable, see [Troubleshooting](#troubleshooting).

### 3. Install robot dependencies (only if needed)

| Robot | What to do |
|-------|------------|
| **Universal Robots** | The real-time driver (`ur_rtde`) is **bundled — do not install it yourself** (the bundled build is patched for PolyScope X; the stock pip package breaks it). Only if you use SFTP program upload: open **Debug > Dependencies** and click **Install UR Dependencies** (installs `paramiko`), then restart Blender. |
| **KUKA** | Nothing to install in Blender. The controller runs the C3 Bridge server (see [KUKA Setup](#kuka-setup)). |
| **xArm / UFactory** | Open **Debug > Dependencies**, click **Install xArm Dependencies**, then restart Blender. |

---

## Robot Asset Library

The robot rigs are a **separate download** from the add-on - they are large and
update on their own schedule.

1. Download `animaquina-robots-<version>.zip` from the releases page.
2. Unzip it anywhere (e.g. `Documents\Animaquinaobots\`).
3. **Edit > Preferences > Add-ons > Animaquina** -> set **Robot Library Folder**
   to that folder -> click **Register Robot Library**.
4. Open an **Asset Browser**, choose **Animaquina Robots**, and drag your robot
   into the scene.

Registering just adds an entry under **Preferences > File Paths > Asset
Libraries**. Click **Remove** in the same panel to undo it.

**Prefer to append manually?** *File > Append > robots.blend > Collection >*
your model works exactly as before.

**No library at all?** Any armature with bones named `joint_1`..`joint_6` can be
bound directly in the Rig panel - set the per-joint axis mapping to match.

## Network Setup

Your PC and the robot controller must be on the **same network** (or connected directly with an Ethernet cable). In all cases, verify with `ping <robot-ip>` from Command Prompt or PowerShell before trying to connect in Blender.

### Universal Robots

1. On the teach pendant: **Settings > System > Network** — note the robot's IP (e.g. `192.168.1.100`).
2. On your PC, set a static IP on the same subnet (e.g. `192.168.1.10`, mask `255.255.255.0`).
3. RTDE must be enabled on the controller (it is by default on e-Series and newer).

### KUKA

1. The controller must be running the **C3 Bridge** server program (port `7000` by default) — see [KUKA Setup](#kuka-setup).
2. On your PC, set a static IP on the KUKA network subnet.

### xArm / UFactory

1. The controller defaults to `192.168.1.xxx`. Connect to its Ethernet port or the same switch.
2. Set your PC to a static IP on the same subnet.

---

## Robot-Specific Setup

### UR Setup

**Controller requirements:**

- e-Series or newer, running **PolyScope 5** or **PolyScope X 10.10+** (CB3 works, but e-Series is recommended for full RTDE support)
- RTDE enabled (default)
- Robot in **Remote Control** mode for motion commands

**PolyScope X (10.10+)** needs no URCap and no pendant program — Animaquina uploads its control script automatically. One-time setup:

1. **Settings > System > Remote Control** — enable Remote Control (simulators often use password `operator`).
2. Switch the robot to **Remote** using the Local/Remote toggle in the top bar.
3. Power on and release brakes as usual.

Then connect from Animaquina with the robot's IP. The first motion command uploads and starts the control script (takes about a second).

⚠️ **Any touch on the pendant UI can flip the robot back to Local/Manual mode.** Motion commands will then fail until you switch back to Remote and reconnect (see [Troubleshooting](#troubleshooting)).

### KUKA Setup

KUKA robots communicate through the **C3 Bridge** — a server program running on the KUKA controller that exposes a socket interface for reading/writing variables, uploading programs, and streaming waypoints.

1. Copy the C3 Bridge files to the controller (provided with your Animaquina package or by your integrator), into `KRC:\R1\Program\`.
2. On the teach pendant, select and **start** the C3 Bridge program. It opens a socket server on port **7000** (default) and stays running in the background.
3. C3 Bridge must be running before Animaquina can connect.

**Custom per-waypoint variables** (e.g. `MY_TEMP`, `FLOW_RATE`) must be declared in `$CONFIG.DAT` on the controller:

```krl
DECL REAL MY_TEMP = 0.0
DECL REAL FLOW_RATE = 0.0
```

Variables not declared on the controller fail **silently** when written. The built-in end-effector variables (`E_SPEED`, `E_ENABLE`, `F_SPEED`, `IDX`, etc.) are already declared by the C3 Bridge setup.

### xArm / UFactory Setup

Install the SDK once via **Debug > Dependencies > Install xArm Dependencies**, restart Blender, and connect with the controller's IP. No controller-side program is needed.

---

## Connecting To A Robot

1. In the **Animaquina** sidebar, open **Robot Registry** and click **Add**. Name the slot (e.g. "UR10e Lab").
2. In the **Setup** panel:
   - choose the **Robot Type** (UR, KUKA, or xArm) and **Model**
   - assign the **Rig Collection** — the collection you appended from `robots.blend`
   - enter the robot's **IP address**
3. Click **Connect**. The status should change to "Connected".
4. Enable **Polling** — the 3D model should now mirror the real robot's pose in real time.

You can add multiple robot slots and connect/disconnect each independently.

### Scene Objects

Assign these in the **Setup** panel:

| Object | What it does | Required? |
|--------|-------------|-----------|
| **TCP** | Empty at the tool center point; updated live during polling | Recommended |
| **Target** | Empty used by Move to Target and Puppet Mode | Recommended |
| **Tool** | Tool/end-effector object reference | Optional |
| **Base** | Base frame reference | Optional |
| **Sim Collection** | IK simulation rig for path preview | Optional |

---

## Using The Robot

### Digital twin (monitoring)

With **Polling** enabled, the rig follows the real robot. The **Info** panel shows TCP position, joint angles, and connection status. Reduce the poll rate (or disable polling on idle robots) if the viewport feels slow.

### Move to Target

1. Position the **Target** empty where you want the robot to go.
2. In the **Control** panel, click **Move to Target** (Linear or PTP).

💡 **Snap Target** snaps the Target empty to the current TCP pose — useful as a safe starting point.

### Run Toolpath

1. Create a mesh whose vertices are your waypoints (e.g. from Geometry Nodes) — waypoints are read from the evaluated mesh's `position` attribute, or from curve control points.
2. In the **Control** panel, select the toolpath object and click **Run Toolpath**.
3. **Buffered** mode streams waypoints continuously at high frequency (servoL on UR, Dynamic Sync on KUKA) for smooth, uninterrupted motion.

### Puppet Mode / Teach Mode

- **Puppet Mode** — the robot continuously follows the Target empty in real time, with configurable rate and safety limits. Move the empty slowly and keep the e-stop within reach.
- **Teach Mode (Freedrive)** — enables manual guidance on UR and xArm so you can physically move the arm.

### Export a program (offline)

1. In the **Export** panel, select the toolpath mesh.
2. Click **Export** — Animaquina generates a URScript (UR) or KRL (KUKA) program as a Blender text block.
3. Review it in the Text Editor, then save to disk, or use **Stage & Upload** to send it to the controller (SFTP on UR, C3 Bridge on KUKA) and **Run Program** to execute it.

### Variables panel

The **Variables** panel reads and writes controller I/O and registers live. Add a variable by name (e.g. `standard_digital_output_0` on UR, `$OV_PRO` on KUKA) and it is polled continuously.

Variable names also double as **mesh attribute names**: if your toolpath mesh has a point-domain attribute with the same name, its values are written per waypoint during export and buffered streaming — this is how you drive extruders, grippers, or fans along a path. See the main README for the full per-brand variable reference.

### Per-Point Speed (UR, buffered streaming)

Each waypoint can carry its own linear speed instead of one constant value:

1. Add a **float point attribute** to the toolpath mesh (default name `speed`; any name works) holding the speed in **m/s** for the segment *into* each point.
2. In the **Control** panel (Linear section, UR only), enable **Per-Point Speed** and type the attribute name.
3. Run the toolpath in **Buffered** mode. Points with a missing or non-positive value fall back to the constant Speed; values are capped at 2.0 m/s.

The attribute is re-read live while streaming, so editing it mid-run (e.g. from Geometry Nodes) takes effect just like editing positions does. The attribute must be on the **point domain** and match the waypoint count, otherwise it is ignored and the constant Speed is used.

### Run Index — knowing which point the robot is at

Whenever a toolpath runs, Animaquina tracks the **current waypoint index** on the robot slot — the point whose per-point attributes are currently in effect (`-1` = idle). It works in all three run modes:

- **Simulation** — timeline playback of a validated sim path drives the index.
- **Buffered streaming** — the stream itself reports each point as it is consumed.
- **Exported program** — the program writes its own index and Animaquina polls it back (see below).

The **Debug** panel shows the active run (`Run [SIM|STREAM|PROGRAM] IDX n / total` plus the path object) with an **X** button to clear a stale run state manually.

For scripting and external tools (e.g. PhyNodes), the index is exposed at a stable data path:

```
bpy.context.scene.animaquina.robots[<slot>].run_idx
```

together with `run_count`, `run_source`, and `run_object`.

#### Streaming per-point attributes to hardware (PhyNodes)

With the **PhyNodes** add-on installed you can broadcast any per-point attribute of the running toolpath as it executes — extruder rates, LED colors, fan states — without writing anything to the robot controller:

1. Store one value per waypoint on the toolpath mesh as a **Point-domain attribute** (any name, e.g. `PAR`).
2. In a PhyNodes graph, add **Input → Animaquina Index** and pick this robot slot.
3. Add a **Geometry Attribute** node, choose the toolpath object and attribute, **untick "All Elements"**, and wire the Index output into its Index socket.
4. Wire the result into an **MQTT PUB** node.

One index drives any number of attributes — add another Geometry Attribute + PUB pair per attribute. Gate the PUB with the index node's **Running** output so nothing is published while idle. Enable **Announce as FabNode** on the PhyNodes connection and Blender appears in FabFlow as a `fab-blender` node with each topic as a port. See the PhyNodes README for the full walkthrough.

### Write Point Index — progress from an exported program

A program exported and started on the controller runs on its own — to get the run index back into Blender:

1. In the **Export** panel, enable **Write Point Index** before exporting.
   - **KUKA** — keep a plain variable name like `IDX`.
   - **UR** — use an output register such as `output_int_register_0` (plain URScript variables cannot be read back over RTDE; the panel reminds you).
2. Export, stage, and run the program as usual.
3. Stay connected with **Polling** on, and add the same name (`IDX` / `output_int_register_0`) to the **Variables** panel — that is what tells Animaquina to poll it.
4. The Debug panel now shows `Run [PROGRAM]` with the live index, and `run_idx` updates for PhyNodes and scripts.

💡 The index is sampled at the poll rate (default 25 Hz), so on fast or dense paths some points may be skipped between samples. For exact per-point behavior — including attribute dispatch on every waypoint — prefer **Run Toolpath Buffered**, where Animaquina itself feeds each point.

⚠️ **KUKA note:** a plain assignment executes in the controller's *advance run*, so it can lead the physical TCP by a few points. Animaquina writes the index as a point-synchronized `TRIGGER`, which keeps it aligned with the motion.

---

## Safety & Licence

⚠️ **You are commanding real industrial machinery.** Before every real-world run:

- verify all generated motion, exports, and toolpaths (use the simulation rig or a dry run at low speed)
- keep the emergency stop within reach and follow your site's safety rules
- start with reduced speed/override until a path is proven
- never stand inside the working envelope while a program is executing

**Animaquina performs no safety function.** It is not a safety-rated controller and
is not a substitute for your robot's own safety systems, guarding, or a risk
assessment. Its path validation has known gaps and can report a pass on motion
that is not safe. Read **[SAFETY.md](SAFETY.md)** in full before connecting to a machine.

**No warranty.** Animaquina is free software under the
[GNU General Public License v3.0 or later](LICENSE). As stated in sections 15 and 16
of that licence, the program is provided **"AS IS" WITHOUT WARRANTY OF ANY KIND**,
and to the maximum extent permitted by applicable law no copyright holder or
contributor is liable for any injury, death, loss, or damage of any kind arising
from your installation or use of it, or from any motion, programs, toolpaths,
exports, uploads, or other outputs produced with it — including harm to people,
damage to robots, tooling, facilities, or other property, loss of data or profits,
business interruption, production errors, or accidents involving industrial
equipment — even if caused by defects, errors, or omissions in the software or
documentation.

**You assume all risk** associated with connecting to and commanding real hardware,
and you must follow your site's safety rules, integrator guidance, and applicable
laws and standards.

---

## Troubleshooting

### UR connects and reads joints, but motion commands fail
Most common cause: the robot is not in **Remote Control** mode (on PolyScope X, touching the pendant UI switches it back to Local/Manual).

1. Switch the robot back to **Remote** mode.
2. In Animaquina, click **Disconnect**, then **Connect** again.
3. If motion still fails, restart Blender — a stale RTDE session can hold the controller's registers until the process exits.

The **UR Debug Status** button (Connection panel) prints a full diagnostic to the system console (**Window > Toggle System Console**) — include it in bug reports.

### UR options are unavailable / SFTP upload fails
- Run **Install UR Dependencies** (Debug > Dependencies), restart Blender, try again.
- **Never** pip-install `ur_rtde` yourself — the bundled patched build is required.

### SFTP password note
The **Stage & Upload** password field defaults to UR's factory default (`easybot`) and is stored **as plain text in your .blend file**. Change it if your robot uses a custom password, and don't share .blend files containing production credentials.

### KUKA won't connect
- Confirm the C3 Bridge program is **running** on the controller (not just selected).
- Confirm port 7000 is reachable: `ping` the controller, check the port in the Setup panel.
- Custom variables that were never declared in `$CONFIG.DAT` fail silently — declare them first.

### Blender shows old behavior after an update
- Fully close and reopen Blender.
- If needed, remove the old Animaquina install in Preferences > Add-ons and reinstall the new zip.

### Add-on fails to load after install
- Install the **zip file directly** — do not unzip it first.
- Confirm you are on **Blender 5.2 LTS** (older versions can't load the bundled drivers).

### Slow viewport while connected
- Lower the poll rate, or disable Polling on robots you're not actively watching.

---

## Reporting Bugs & Giving Feedback

Post in the Discord: **https://discord.gg/pM2cauqadZ**

Please include:

1. Blender version and robot type/model
2. a screenshot of the **License** panel
3. the exact error message (check **Window > Toggle System Console** for the full traceback)
4. for UR issues: the **UR Debug Status** output
5. what you were doing when it happened (steps to reproduce, if possible)

Workflow notes are just as valuable as bug reports — if something felt confusing or clunky, tell us.

## Research Citation

If you use Animaquina in your research, please cite:

Pacheco, Luis (2024). *Animaquina*. In *ACADIA 2024: Designing Change* [Volume 2: Proceedings of the 44th Annual Conference for the Association for Computer Aided Design in Architecture (ACADIA), ISBN 979-8-9891764-8-9]. Calgary, 11–16 November 2024. Edited by Alicia Nahmad-Vazquez, Jason Johnson, Joshua Taron, Jinmo Rhee, Daniel Hapton. pp. 591–600.

https://papers.cumincad.org/cgi-bin/works/Show?acadia24_v2_89
