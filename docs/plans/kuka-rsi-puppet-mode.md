# KUKA RSI vs MxAutomation for Puppet Mode

## TL;DR

**RSI is the clear winner** for puppet mode. It was designed for exactly this — external sensor-guided real-time position control at the interpolation cycle level.

MxAutomation is a PLC-style sequenced motion interface. It queues commands through the motion planner, making it unsuitable for real-time streaming.

## Comparison

| | RSI | MxAutomation |
|---|---|---|
| **Update rate** | 83Hz (12ms) or 250Hz (4ms IPO_FAST) | Commands processed at planner level, << 50Hz effective |
| **Latency** | 4-12ms cycle + ~120ms tracking delay | 50-200ms+ command-to-motion |
| **Protocol** | Simple UDP, XML strings | TCP port 1336 (Python API) or fieldbus (EtherCAT/PROFINET) |
| **Python impl** | ~50 lines UDP socket + XML parse | Complex proprietary protocol |
| **Position control** | Direct injection at interpolation cycle | Queued through motion planner |
| **Disconnect safety** | Robot faults and stops within 1 cycle | Depends on watchdog config |
| **Licensing** | 1 option package on KRC | KRC option + PLC library or Python API |
| **Best for** | Real-time streaming, sensor guidance | PLC sequenced programs, industrial automation |

## How RSI Works

### Architecture

```
Blender (Python)          Network           KRC4
┌─────────────┐          UDP/IP          ┌──────────────┐
│ UDP Server   │◄────────────────────────│ RSI Engine   │
│ port 49152   │  Robot sends state      │ (12ms cycle) │
│              │  every 4-12ms           │              │
│ Compute      │                         │ Applies      │
│ correction   │────────────────────────►│ correction   │
│              │  Server responds with   │ to path      │
│              │  position correction    │              │
└─────────────┘                          └──────────────┘
```

### Data Exchange (every cycle)

**Robot sends** its state as XML:
```xml
<Rob Type="KUKA">
  <RIst X="500.0" Y="0.0" Z="800.0" A="0.0" B="90.0" C="0.0" />
  <RSol X="500.0" Y="0.0" Z="800.0" A="0.0" B="90.0" C="0.0" />
  <AIPos A1="0.0" A2="-90.0" A3="90.0" A4="0.0" A5="0.0" A6="0.0" />
  <Delay D="0" />
  <IPOC>123456789</IPOC>
</Rob>
```

- `RIst` — actual cartesian position (mm, deg)
- `RSol` — commanded cartesian position
- `AIPos` — actual joint positions (deg)
- `IPOC` — timestamp, **must be echoed back**

**Server responds** with corrections:
```xml
<Sen Type="ImFree">
  <EStr>Animaquina Puppet</EStr>
  <RKorr X="10.0" Y="5.0" Z="0.0" A="0.0" B="0.0" C="0.0" />
  <IPOC>123456789</IPOC>
</Sen>
```

- `RKorr` — cartesian correction (mm, deg). In absolute mode: offset from position when RSI_ON was called.
- `IPOC` — **must match** the received IPOC

### Correction Modes

**Absolute mode** (best for puppet): Correction = offset from start position. Robot stays put if server stops updating.

**Relative mode** (dangerous): Each correction is added incrementally. If server stops, last delta keeps being applied — robot drifts.

### Safety Limits (configurable in XML)

| Limit | Default | Notes |
|-------|---------|-------|
| Per-cycle correction | ±5mm / ±5° | Clamped silently if exceeded |
| Total correction | ±6mm / ±6° | RSI stops, robot halts |
| Response deadline | ~8ms | Robot faults if missed |

For puppet mode with large workspace, increase total correction limits in the XML config. Per-cycle limit of 5mm @ 12ms = **416mm/s max velocity**, sufficient for smooth puppet motion.

## Implementation Plan

### 1. KRL Program (on robot)

```krl
DEF RSI_Puppet()
  DECL INT ret

  BAS(#INITMOV, 0)

  ; Move to current position (required before RSI)
  PTP $AXIS_ACT

  ; Load RSI XML configuration
  ret = RSI_CREATE("AnimaquinaPuppet", 1)

  ; Activate RSI in ABSOLUTE mode
  ; Robot is now under external position control
  ret = RSI_ON(#ABSOLUTE)

  ; Pure sensor-guided motion — no programmed end point
  ; Robot follows corrections until RSI_OFF or fault
  RSI_MOVECORR()

  ; Deactivate (reached if RSI faults or is stopped)
  ret = RSI_OFF()
END
```

### 2. RSI XML Config (`AnimaquinaPuppet.xml`)

Place in `C:\KRC\ROBOTER\Config\User\Common\SensorInterface\`:

```xml
<?xml version="1.0" encoding="utf-8"?>
<ROOT>
  <CONFIG>
    <IP_NUMBER>192.168.1.100</IP_NUMBER>  <!-- Blender PC IP -->
    <PORT>49152</PORT>                      <!-- UDP port -->
    <SENTYPE>ImFree</SENTYPE>
    <ONLYSEND>FALSE</ONLYSEND>
  </CONFIG>

  <!-- Send actual position to external server -->
  <SEND>
    <ELEMENT TAG="RIst" TYPE="CARTESIAN" INDX="INTERNAL" />
    <ELEMENT TAG="RSol" TYPE="CARTESIAN" INDX="INTERNAL" />
    <ELEMENT TAG="AIPos" TYPE="JOINT" INDX="INTERNAL" />
    <ELEMENT TAG="Delay" TYPE="INT" INDX="INTERNAL" />
  </SEND>

  <!-- Receive cartesian corrections from external server -->
  <RECEIVE>
    <ELEMENT TAG="RKorr" TYPE="CARTESIAN" INDX="POSCORR" />
  </RECEIVE>

  <!-- Correction limits (increase for large workspace puppet mode) -->
  <POSCORRMON>
    <UPPER LIM="1000.0" />   <!-- max total correction mm -->
    <LOWER LIM="-1000.0" />
  </POSCORRMON>
</ROOT>
```

### 3. Python UDP Server (Blender side)

```python
import socket
import re

class RSIPuppetServer:
    """UDP server for KUKA RSI puppet mode.
    Receives robot state every 4-12ms, responds with position correction."""

    def __init__(self, host="0.0.0.0", port=49152):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((host, port))
        self._sock.settimeout(0.1)
        self._target_offset = [0.0] * 6  # X,Y,Z,A,B,C correction (mm, deg)
        self._robot_addr = None
        self._start_pos = None  # captured on first packet

    def set_target(self, pos_m, euler_rad):
        """Set absolute target position (Blender canonical units).
        Internally converted to offset from RSI start position."""
        if self._start_pos is None:
            return
        # Convert m -> mm, rad -> deg for KUKA
        target_mm = [p * 1000.0 for p in pos_m]
        target_deg = [math.degrees(e) for e in euler_rad]  # already ABC order
        # Correction = target - start_pos (absolute mode)
        self._target_offset = [
            target_mm[0] - self._start_pos[0],
            target_mm[1] - self._start_pos[1],
            target_mm[2] - self._start_pos[2],
            target_deg[0] - self._start_pos[3],
            target_deg[1] - self._start_pos[4],
            target_deg[2] - self._start_pos[5],
        ]

    def spin_once(self):
        """Receive one robot packet, respond with correction. Non-blocking."""
        try:
            data, addr = self._sock.recvfrom(4096)
            self._robot_addr = addr
        except socket.timeout:
            return None

        xml_str = data.decode("utf-8", errors="replace")

        # Parse IPOC (must echo back)
        ipoc_match = re.search(r"<IPOC>(\d+)</IPOC>", xml_str)
        ipoc = ipoc_match.group(1) if ipoc_match else "0"

        # Parse actual position (for start_pos capture)
        if self._start_pos is None:
            rist = re.search(
                r'<RIst\s+X="([^"]+)"\s+Y="([^"]+)"\s+Z="([^"]+)"'
                r'\s+A="([^"]+)"\s+B="([^"]+)"\s+C="([^"]+)"', xml_str)
            if rist:
                self._start_pos = [float(rist.group(i)) for i in range(1, 7)]

        # Build response
        o = self._target_offset
        response = (
            '<Sen Type="ImFree">'
            '<EStr>AnimaquinaPuppet</EStr>'
            f'<RKorr X="{o[0]:.4f}" Y="{o[1]:.4f}" Z="{o[2]:.4f}" '
            f'A="{o[3]:.4f}" B="{o[4]:.4f}" C="{o[5]:.4f}" />'
            f'<IPOC>{ipoc}</IPOC>'
            '</Sen>'
        )
        self._sock.sendto(response.encode("utf-8"), addr)
        return self._start_pos

    def close(self):
        self._sock.close()
```

### 4. Integration with Animaquina Driver

The RSI puppet mode would be a separate code path in `kuka_driver.py`:

- `realtime_puppet_start()` — Start the UDP server thread, upload & start the RSI KRL program
- `realtime_puppet_step()` — Just call `server.set_target(pos_m, euler_rad)` — no socket write, instant (the UDP thread handles the response)
- `realtime_puppet_stop()` — Stop KRL program, shut down UDP server

Key advantage: **`realtime_puppet_step()` becomes zero-latency** — it just updates a target variable in memory. The UDP server thread running at 83-250Hz handles the actual communication independently.

## What You Need on the Controller

1. **RSI option package** licensed and installed on the KRC
2. **Network setup**: Blender PC must be on the KLI (KUKA Line Interface) network, typically `192.168.1.x`
3. **XML config file** placed in `C:\KRC\ROBOTER\Config\User\Common\SensorInterface\`
4. **RSI KRL program** uploaded and selected

## Current Approach (C3 Bridge) vs RSI

| | C3 Bridge (current) | RSI (proposed) |
|---|---|---|
| Step latency | ~5ms (TCP variable write) | ~0ms (memory update, UDP async) |
| Robot-side cycle | KRL interpreter (~12ms + planner) | Interpolation cycle (4-12ms, no planner) |
| Total lag | ~50-100ms+ (T1 speed limit adds more) | ~12-120ms (cycle + tracking filter) |
| Requires | kukaproxydriver only | RSI option package |
| T1 mode | Speed-limited by KRC | Same T1 speed limit applies |

Note: T1 speed limit (~250mm/s) applies regardless of the communication method. RSI won't bypass T1 safety limits — it just removes the communication and planner overhead.

## Future improvement: extend Puppet ↔ motion interlock to "Run Program" ops

**Status:** TODO (deferred). Not urgent.

We added a mutual-exclusion interlock so Puppet Mode and toolpath streaming can't
run at the same time (robot-agnostic, via `slot.realtime_puppet_active` +
`slot.motion_active_label`). Covered today:

- `ANIMAQUINA_OT_SendPath` (Run Toolpath — universal, all robots)
- `ANIMAQUINA_OT_SendPathQueueUR` (UR buffered)
- `ANIMAQUINA_OT_KukaStreamProgram` (KUKA `.src` Stream to Robot)
- `ANIMAQUINA_OT_RealTimePuppetStart` (blocks when a toolpath/stream is active)

**Not yet covered** — the two "play a pre-loaded program on the controller"
operators, which also move the robot but don't participate in the interlock
(they clear `motion_active_label` but never set it, and have no puppet guard):

- `ANIMAQUINA_OT_RunKukaProgram` — "Run KUKA Program" ([operators.py:4365](../animaquina/ui/operators.py#L4365))
- `ANIMAQUINA_OT_URLoadProgramOnRobot` — "Run UR Program" ([operators.py:4650](../animaquina/ui/operators.py#L4650))

**To do later:** decide whether starting these while Puppet Mode is active should
be blocked. Nuance for KUKA: puppet mode *requires* `mq_stream` to be running, so
"Run KUKA Program" interacts with it differently than a plain toolpath — it may
need to *stop* puppet rather than simply be disabled. Define the intended behavior
per robot, then add `poll()` guards (with `poll_message_set`) and/or set
`motion_active_label` while these programs run.

## References

- [KUKA RSI 3.1 Manual](http://supportwop.com/IntegrationRobot/content/6-Systèmes_intégrations/RobotSensorInterface/KST_RSI_31_en.pdf)
- [KUKA Ethernet RSI XML 1.1](http://www.wtech.com.tw/public/download/manual/kuka/krc2ed05/KUKA%20Ethernet%20RSI_XML.pdf)
- [RSIPI Python library (PyPI)](https://pypi.org/project/RSIPI/)
- [kuka-rsi3-communicator (GitHub)](https://github.com/erensezener/kuka-rsi3-communicator)
- [KUKA RSI Python Server](https://github.com/pawankumardev/kukarsiserver)
