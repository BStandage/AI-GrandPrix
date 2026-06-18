"""
Closed-loop offline harness: run a REAL pilot (vision or oracle) against the measured-dynamics
DroneModel + spec CameraModel, and score how it threads the gates - no real simulator needed.

Validate the model first:  python -m sim.pilot_sim --oracle      (must pass all gates - it's the
gold pilot on true state; if it doesn't, the MODEL is wrong, fix that before trusting vision)
Then debug the pilot:       python -m sim.pilot_sim                (vision pilot, perfect perception)
                            python -m sim.pilot_sim --noise       (vision pilot, log-calibrated noise)

Gate pass = the flown path crosses the gate plane within the 1.5 m square opening (half-width 0.75).
"""

import argparse
import glob
import json
import math
import os
import random

from common.dynamics import CONTROL_HZ
from common.gate_geometry import relative_gate
from common.paths import DATASETS_DIR
from sim.drone_model import DroneModel
from sim.camera_model import render

HALF_OPENING = 0.75
CAM_EVERY = max(1, CONTROL_HZ // 30)     # ~30 Hz camera vs 250 Hz control
MAX_TIME = 40.0


class _FakeConn:
    """Captures the pilot's rate+thrust command (the last 4 args of set_attitude_target_send)."""
    def __init__(self):
        self.cmd = (0.0, 0.0, 0.0, 0.0)
        self.target_system = self.target_component = 1
        self.mav = self

    def set_attitude_target_send(self, *a, **k):
        self.cmd = (a[-4], a[-3], a[-2], a[-1])   # roll_rate, pitch_rate, yaw_rate, thrust


def _load_gates(session_dir=None):
    if session_dir is None:
        files = sorted(glob.glob(os.path.join(DATASETS_DIR, "*", "gates.json")), key=os.path.getmtime)
        if not files:
            raise SystemExit("no gates.json found in any session")
        session_dir = os.path.dirname(files[-1])
    gates = json.load(open(os.path.join(session_dir, "gates.json")))["gates"]
    return sorted(gates, key=lambda g: g.get("gate_id", 0))


def run(mode="vision", noise=None, verbose=False, seed=1):
    import keyboard
    keyboard.is_pressed = lambda k: False          # headless

    gates = _load_gates()
    model = DroneModel()
    conn = _FakeConn()
    rng = random.Random(seed)

    if mode == "oracle":
        from pilots.oracle_pilot.oracle_pilot import update_trajectory_control as control
    else:
        from pilots.vision_pilot.vision_pilot import update_vision_control as control

    data = {
        "running": True,
        "gates": gates,                            # truth: camera + oracle use it; vision ignores it
        "race_status": {"race_start_boot_time_ms": 0, "sim_boot_time_ms": 100,
                        "race_finish_time_ns": -1, "active_gate_index": 0},
        "_control_mode": mode,
        "latest_frame_id": 0,
    }

    dt = 1.0 / CONTROL_HZ
    n = int(MAX_TIME * CONTROL_HZ)
    active = 0
    prev_rel = None
    results = {}                                   # gate_id -> (miss, right, down)
    traj = []

    for i in range(n):
        data["odometry"] = model.odometry()
        data["attitude"] = model.attitude()
        if i % CAM_EVERY == 0:
            data["vision_gates"] = render((model.pos[0], model.pos[1], model.pos[2]),
                                          model.quat(), gates, noise=noise, rng=rng)
            data["vision_target"] = data["vision_gates"][0] if data["vision_gates"] else None
            data["latest_frame_id"] += 1

        control(conn, 0, data)
        if verbose and i % 50 == 0:
            ndet = len(data.get("vision_gates") or [])
            curset = data.get("_vg_cur") is not None
            nxtset = data.get("_vg_next") is not None
            print(f"  t={i*dt:5.2f} alt={-model.pos[2]:6.1f} N={model.pos[0]:6.1f} "
                  f"pitch={math.degrees(model.pitch):5.1f} ndet={ndet} cur={curset} nxt={nxtset} "
                  f"reg={data.get('vision_regime')}", flush=True)
        model.step(*conn.cmd, dt)

        # crash / divergence guard. The course DESCENDS to ~-26 m alt, so only flag well beyond it
        # (too far below the lowest gate, or climbing above spawn).
        alt = -model.pos[2]
        lowest_gate_alt = min(-g["position_ned"][2] for g in gates)
        if not math.isfinite(model.pos[0]) or alt < lowest_gate_alt - 6 or alt > 12:
            if verbose:
                print(f"  [t={i*dt:.1f}] diverged: alt={alt:.1f}", flush=True)
            break

        # gate-plane crossing for the active gate (true geometry)
        if active < len(gates):
            rel = relative_gate(model.pos, model.quat(), gates[active])
            if prev_rel is not None and prev_rel["forward"] > 0 >= rel["forward"]:
                span = prev_rel["forward"] - rel["forward"]
                t = prev_rel["forward"] / span if span > 1e-9 else 0.0
                r = prev_rel["right"] + (rel["right"] - prev_rel["right"]) * t
                d = prev_rel["down"] + (rel["down"] - prev_rel["down"]) * t
                results[gates[active]["gate_id"]] = (math.hypot(r, d), r, d)
                active += 1
                data["race_status"]["active_gate_index"] = active
                prev_rel = None
                continue
            prev_rel = rel
        traj.append((model.pos[0], model.pos[1], alt))
        if active >= len(gates):
            break

    # report
    print(f"\n=== pilot_sim [{mode}{' +noise' if noise else ''}]  ({len(gates)} gates) ===")
    print(f"{'gate':>4} {'pass':>5} {'miss_m':>7} {'dir':>16}   gate_alt")
    passed = 0
    for g in gates:
        gid = g["gate_id"]
        ga = -g["position_ned"][2]
        if gid in results:
            miss, r, d = results[gid]
            ok = abs(r) <= HALF_OPENING and abs(d) <= HALF_OPENING
            passed += ok
            side = f"{'R' if r >= 0 else 'L'}{abs(r):.1f} {'DN' if d >= 0 else 'UP'}{abs(d):.1f}"
            print(f"{gid:>4} {'YES' if ok else 'NO ':>5} {miss:>7.2f} {side:>16}   {ga:>6.1f}")
        else:
            print(f"{gid:>4} {'--':>5} {'(not reached)':>24}   {ga:>6.1f}")
    print(f"\nthreaded {passed}/{len(gates)} gates")
    if traj:
        print(f"flight: {len(traj)} pts, final pos N={traj[-1][0]:.0f} E={traj[-1][1]:.0f} alt={traj[-1][2]:.1f}")
    return passed, len(gates)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--oracle", action="store_true", help="run the oracle (validates the model)")
    ap.add_argument("--noise", action="store_true", help="add log-calibrated perception noise")
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument("--trials", type=int, default=1, help="run N seeds and report the pass rate")
    args = ap.parse_args()
    mode = "oracle" if args.oracle else "vision"
    if args.trials > 1:
        from sim.perception_noise import PerceptionNoise
        total = 0
        for s in range(args.trials):
            p, n = run(mode=mode, noise=PerceptionNoise() if args.noise else None, seed=s + 1)
            total += p
        print(f"\n==== {args.trials} trials: mean {total / args.trials:.1f}/{n} gates "
              f"({100 * total / (args.trials * n):.0f}%) ====")
    else:
        noise = None
        if args.noise:
            from sim.perception_noise import PerceptionNoise
            noise = PerceptionNoise()
        run(mode=mode, noise=noise, verbose=args.verbose)
