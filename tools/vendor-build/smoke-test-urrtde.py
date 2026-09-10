# Smoke test for a candidate ur_rtde build against URSim (or a real robot).
#
# Run this before shipping any new ur_rtde in vendor_py:
#   <blender python.exe> tools/vendor-build/smoke-test-urrtde.py [--vendor DIR] [--ip 127.0.0.1]
#
# Prerequisites: URSim PolyScope X running (tools/ursim-start.ps1), robot
# powered on with brakes released, Remote Control mode ACTIVE, and no other
# RTDE client connected (disconnect Blender first — a live session holds the
# controller's RTDE input registers and the control ctor will fail).
#
# Checks:
#   1. import + PolyScope X capability flags present
#   2. receive interface connects and reads joints
#   3. control interface constructs (script auto-upload) and the control
#      program is RUNNING — this is the check stock 1.6.3 fails on PolyScope X
#      (it "succeeds" but the program never runs)
#   4. a small moveJ actually changes the joints (and is reverted)
#   5. verbose-flag crash probe in a subprocess (the substr(n-100) underflow
#      that crashed builds before the animaquina patch / upstream fix)

import argparse
import os
import subprocess
import sys
import time

DEFAULT_VENDOR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
    "animaquina", "vendor_py",
)


def fail(msg):
    print(f"FAIL  {msg}")
    sys.exit(1)


def ok(msg):
    print(f"OK    {msg}")


def verbose_probe(ip):
    """Construct the control interface WITH FLAG_VERBOSE. Crashes (native AV)
    on builds that still carry the script-injection substr underflow bug."""
    import rtde_control
    iface = rtde_control.RTDEControlInterface
    flags = int(iface.FLAGS_DEFAULT) | int(iface.FLAG_VERBOSE)
    c = rtde_control.RTDEControlInterface(ip, 125.0, flags)
    running = c.isProgramRunning()
    c.disconnect()
    print(f"verbose-probe isProgramRunning={running}")
    sys.exit(0 if running else 3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--vendor", default=DEFAULT_VENDOR, help="vendor_py folder to test")
    ap.add_argument("--ip", default="127.0.0.1", help="robot / URSim IP")
    ap.add_argument("--verbose-probe", action="store_true", help=argparse.SUPPRESS)
    args = ap.parse_args()

    sys.path.insert(0, args.vendor)

    if args.verbose_probe:
        verbose_probe(args.ip)
        return

    print(f"vendor_py : {args.vendor}")
    print(f"robot ip  : {args.ip}")

    # 1. import + flags
    try:
        import rtde_control, rtde_receive  # noqa: E401
    except Exception as e:
        fail(f"import: {e}")
    iface = rtde_control.RTDEControlInterface
    if getattr(iface, "FLAG_USE_EXT_UR_CAP", None) is None:
        fail("FLAG_USE_EXT_UR_CAP missing — ur_rtde too old")
    ok("import + capability flags")

    # 2. receive
    try:
        recv = rtde_receive.RTDEReceiveInterface(args.ip, 125.0)
        q0 = recv.getActualQ()
        assert len(q0) == 6
    except Exception as e:
        fail(f"receive interface: {e}")
    ok(f"receive connects, joints: {[round(v, 3) for v in q0]}")

    # 3. control ctor + program running (no verbose)
    try:
        ctrl = rtde_control.RTDEControlInterface(args.ip, 125.0, int(iface.FLAGS_DEFAULT))
    except Exception as e:
        fail(f"control ctor: {e} (robot on? Remote mode? another client connected?)")
    running = ctrl.isProgramRunning()
    if not running:
        ctrl.disconnect()
        fail("control program NOT running after construct — this is the stock-1.6.3 "
             "PolyScope X failure mode (or the robot is not in Remote Control mode)")
    ok("control script uploaded and running")

    # 4. motion round-trip
    q1 = list(recv.getActualQ())
    q_target = list(q1)
    q_target[5] += 0.1
    if not ctrl.moveJ(q_target, 0.5, 0.5):
        ctrl.disconnect()
        fail("moveJ returned False")
    time.sleep(0.3)
    moved = abs(recv.getActualQ()[5] - q1[5]) > 0.05
    ctrl.moveJ(q1, 0.5, 0.5)  # revert
    time.sleep(0.3)
    if not moved:
        ctrl.disconnect()
        fail("moveJ returned True but joints did not change")
    ok("moveJ moved the robot (and reverted)")
    ctrl.disconnect()
    time.sleep(1.0)  # let the controller release the session before the probe

    # 5. verbose crash probe (subprocess so a native crash doesn't kill us)
    proc = subprocess.run(
        [sys.executable, os.path.abspath(__file__), "--vendor", args.vendor,
         "--ip", args.ip, "--verbose-probe"],
        capture_output=True, text=True, timeout=120,
    )
    if proc.returncode == 0:
        ok("verbose-flag probe (no crash)")
    elif proc.returncode == 3:
        print("WARN  verbose probe ran but program not running (check Remote mode) — no crash though")
    else:
        fail(f"verbose-flag probe crashed/failed (exit {proc.returncode}) — "
             f"the substr underflow bug is present in this build\n{proc.stdout}{proc.stderr}")

    print("\nALL CHECKS PASSED — this ur_rtde build is good to ship.")


if __name__ == "__main__":
    main()
