"""
System-identification flight campaign - entry point.

Runs an automated, crash-tolerant battery of flight tests against the simulator to map the
drone's true dynamic envelope, then compiles a Markdown engineering report. The output feeds
the trajectory planner so a future vision pilot can fly at the airframe's real limits instead
of the conservative hand-tuned constants in dynamics.py.

Usage (sim must be running and listening, like main.py):
    python sysid_campaign.py --all                 # every battery, then the report
    python sysid_campaign.py --rotational --drag   # just these batteries
    python sysid_campaign.py --report-only          # regenerate the report from the latest run
    python sysid_campaign.py --report-only <dir>    # ... from a specific sysid_* directory

Press Ctrl-C at any time to abort cleanly. Each battery writes one tab CSV into
datasets/sysid_<timestamp>/, and the report lands beside them as SYSID_REPORT.md.
"""

import argparse
import os
import sys
import time

# Allow running this file directly (python flight_test/sysid_campaign.py) by putting
# the src/ package root on sys.path, not just this subpackage's folder.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pymavlink import mavutil

from common.paths import DATASETS_DIR
from comms.mavlink_rx import MAVLinkRX
from flight_test import sysid_report
from flight_test.sysid_batteries import (run_attitude, run_drag, run_feasibility, run_point_tracking,
                             run_recovery, run_rotational)
from flight_test.sysid_runner import TrialRunner
DEFAULT_IP = "127.0.0.1"
DEFAULT_PORT = 14550


def latest_sysid_dir():
    if not os.path.isdir(DATASETS_DIR):
        return None
    cands = sorted(d for d in os.listdir(DATASETS_DIR)
                   if d.startswith("sysid_") and os.path.isdir(os.path.join(DATASETS_DIR, d)))
    return os.path.join(DATASETS_DIR, cands[-1]) if cands else None


def connect(ip, port):
    """Trimmed setup: just the MAVLink connection + the telemetry rx thread (no vision/logger)."""
    shared_data = {"running": True, "quiet_track": True}   # suppress per-reset track-rx spam
    print(f"Connecting to sim on udpin:{ip}:{port} ...", flush=True)
    conn = mavutil.mavlink_connection(f"udpin:{ip}:{port}")
    conn.wait_heartbeat()
    print(f"Connected to system {conn.target_system}.", flush=True)
    MAVLinkRX.create_mavlink_rx(conn, shared_data, logger=None)
    system_boot_ms = int(time.time() * 1000)
    return conn, shared_data, system_boot_ms


def main():
    ap = argparse.ArgumentParser(description="6-DOF system identification flight campaign")
    ap.add_argument("--rotational", action="store_true", help="Tab 1: rotational dynamics sweep")
    ap.add_argument("--drag", action="store_true", help="Tab 2: aerodynamic drag envelope")
    ap.add_argument("--recovery", action="store_true", help="Tab 3: recovery phase-plane (aerobatic)")
    ap.add_argument("--feasibility", action="store_true", help="Tab 4: kinematic feasibility cone")
    ap.add_argument("--point-tracking", action="store_true",
                    help="Tab 5: closed-loop point tracking & station hold (Round-1 ODOMETRY)")
    ap.add_argument("--attitude", action="store_true",
                    help="Tab 6: attitude-setpoint interface signs/gains (send_attitude_setpoint)")
    ap.add_argument("--all", action="store_true", help="run every battery")
    ap.add_argument("--report-only", nargs="?", const="__latest__", default=None,
                    metavar="DIR", help="skip flying; (re)generate the report from DIR or the latest run")
    ap.add_argument("--alt", type=float, default=45.0, help="default test altitude (m)")
    ap.add_argument("--ip", default=DEFAULT_IP)
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = ap.parse_args()

    # ---- report-only path: no sim needed ----
    if args.report_only is not None:
        out_dir = latest_sysid_dir() if args.report_only == "__latest__" else args.report_only
        if not out_dir or not os.path.isdir(out_dir):
            print("No sysid_* directory found to report on.", flush=True)
            return 1
        sysid_report.generate(out_dir)
        return 0

    do_rot = args.rotational or args.all
    do_drag = args.drag or args.all
    do_rec = args.recovery or args.all
    do_feas = args.feasibility or args.all
    do_pt = args.point_tracking or args.all
    do_att = args.attitude or args.all
    if not (do_rot or do_drag or do_rec or do_feas or do_pt or do_att):
        print("Nothing to do. Pass a battery flag (--rotational/--drag/--recovery/--feasibility/"
              "--point-tracking/--attitude) or --all. See --help.", flush=True)
        return 1

    out_dir = os.path.join(DATASETS_DIR, time.strftime("sysid_%Y%m%d_%H%M%S"))
    os.makedirs(out_dir, exist_ok=True)
    print(f"Campaign output -> {out_dir}", flush=True)

    conn, data, boot_ms = connect(args.ip, args.port)
    runner = TrialRunner(conn, data, boot_ms)
    if not runner.wait_for_telemetry(timeout=15.0):
        print("No telemetry (odometry) received - is the sim running and a race loaded?", flush=True)
        return 1

    print("\nStarting campaign. Press Ctrl-C to abort. The drone WILL fly aggressively and crash; "
          "that's expected - it resets between trials.\n", flush=True)
    # Each battery is guarded by data["running"] so an ESC mid-campaign stops cleanly instead of
    # falling through and creating empty CSVs for the remaining batteries.
    try:
        if do_rot and data.get("running"):
            run_rotational(runner, out_dir, setup_alt=args.alt)
        if do_drag and data.get("running"):
            run_drag(runner, out_dir, setup_alt=args.alt)
        if do_rec and data.get("running"):
            run_recovery(runner, out_dir)
        if do_feas and data.get("running"):
            run_feasibility(runner, out_dir, setup_alt=args.alt)
        if do_pt and data.get("running"):
            run_point_tracking(runner, out_dir, setup_alt=args.alt)
        if do_att and data.get("running"):
            run_attitude(runner, out_dir, setup_alt=args.alt)
    except KeyboardInterrupt:
        print("\nInterrupted.", flush=True)
        data["running"] = False

    print("\nGenerating report ...", flush=True)
    sysid_report.generate(out_dir)
    print(f"\nDone. See {os.path.join(out_dir, 'SYSID_REPORT.md')}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
