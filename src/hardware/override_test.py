"""MSP override bench test: can the human pilot take the sticks back?

One command. Watch one screen. It tells you pass or fail.

    python3 -m hardware.override_test --port /dev/ttyTHS1

PROPS OFF. The test never raises throttle above minimum and never asks the FC
to arm, but a wrong mask is exactly what this is looking for, so take them off.

What it does: sends a distinctive stick pattern over MSP continuously, reads
back what the flight controller actually has on channels 1-4, and says who owns
them. Then it watches for the three things that have to happen:

    1. The JETSON takes the sticks      - flip MSP OVERRIDE on
    2. The TRANSMITTER gets them back   - flip MSP OVERRIDE off   <- THE TEST
    3. The TRANSMITTER can disarm       - ARM on, then ARM off

Step 2 is why this exists. With the default mask of 11 the Jetson can write the
switch channel itself and hold override on, and the transmitter stops working.
Mask 15 covers the four sticks and no AUX channel. This proves it took.

Only one program may hold the serial port, so close other sessions first.
"""

from __future__ import annotations

import argparse
import time

from hardware import msp
from hardware.bridge import FcBridge

# Distinctive values: far from stick centre, far from each other, so "the FC
# has our numbers" is unambiguous. Throttle stays at minimum throughout and is
# NOT used to decide ownership: a pilot's throttle at rest reads ~988 and ours
# reads 1000, which is inside any sane tolerance, so it cannot tell them apart.
# That throttle is covered at all is proven by the mask reading 15, not here.
SEND = {"roll": 1123, "pitch": 1234, "yaw": 1345}
THROTTLE = 1000
TOL = 25          # PWM: RC noise and the FC's own filtering
HOLD_S = 0.4      # a state must persist this long before it counts


def _owner(echo):
    """Who has channels 1-4 right now: the JETSON ('msp'), the TRANSMITTER
    ('pilot'), or a broken mix of both."""
    if not echo or len(echo) < 4:
        return None
    # MSP_RC echoes in Betaflight's INTERNAL order: roll, pitch, yaw, throttle
    got = {"roll": echo[msp.RC_ECHO_ROLL], "pitch": echo[msp.RC_ECHO_PITCH],
           "yaw": echo[msp.RC_ECHO_YAW]}
    hits = sum(abs(got[k] - SEND[k]) <= TOL for k in SEND)
    if hits == len(SEND):
        return "msp"
    if hits == 0:
        return "pilot"
    return "mixed"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--port", default="/dev/ttyTHS1")
    ap.add_argument("--tcp", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=180.0)
    args = ap.parse_args(argv)

    print(__doc__.split("What it does:")[0])
    print("PROPS OFF. Press Ctrl+C to stop.\n")
    print(" who has the sticks         | armed | override | ANGLE | roll pitch  yaw  thr | 1 2 3 |")
    print(" " + "-" * 92)

    br = FcBridge.open(port=args.port, tcp=args.tcp, baud=args.baud)
    br.start()

    took = False          # 1. MSP took the sticks
    gave_back = False     # 2. the pilot got them back, with MSP still sending
    disarmed = False      # 3. the pilot disarmed, with MSP still sending
    saw_armed = False
    mixed_seen = False
    seen_modes = set()
    since = time.monotonic()
    last_owner = None
    t0 = time.monotonic()

    try:
        while time.monotonic() - t0 < args.seconds:
            # keep sending our pattern the whole time; throttle stays at minimum
            # and arm stays low - the FC is never asked to arm by us
            br.set_rc(throttle=THROTTLE, roll=SEND["roll"],
                      pitch=SEND["pitch"], yaw=SEND["yaw"], arm=msp.RC_MIN)
            time.sleep(0.1)
            st = br.state()
            echo = st.rc_echo
            own = _owner(echo)
            if own != last_owner:
                last_owner, since = own, time.monotonic()
            held = time.monotonic() - since

            modes = (st.status.active_modes or []) if st.status else []
            seen_modes.update(modes)
            armed = bool(st.status and st.status.armed)
            override = "MSP OVERRIDE" in modes
            # ANGLE must be on: the follower sends ANGLE-mode sticks, which in
            # ACRO are read as RATE demands and the aircraft does something
            # entirely different from what the plan asked for.
            angle = "ANGLE" in modes
            saw_armed = saw_armed or armed
            if own == "mixed":
                mixed_seen = True
            if own == "msp" and held > HOLD_S:
                took = True
            if took and own == "pilot" and held > HOLD_S:
                gave_back = True
            if saw_armed and not armed:
                disarmed = True

            who = {"msp": "JETSON      has the sticks", "pilot": "TRANSMITTER has the sticks",
                   "mixed": "MIXED - WRONG MASK!       ",
                   None: "no RC echo yet            "}[own]
            ch = " ".join(f"{v:4d}" for v in (echo[:4] if echo else []))
            print(f"\r {who} | {'YES' if armed else 'no ':5} "
                  f"| {'ON ' if override else 'off':8} "
                  f"| {'ON ' if angle else 'OFF':5} "
                  f"| {ch} "
                  f"| {'X' if took else '_'} {'X' if gave_back else '_'} "
                  f"{'X' if disarmed else '_'} |", end="", flush=True)

            if took and gave_back and disarmed:
                break
    except KeyboardInterrupt:
        pass
    finally:
        br.set_rc(**dict(zip(("throttle", "roll", "pitch", "yaw"),
                             (msp.RC_MIN, msp.RC_CENTER, msp.RC_CENTER, msp.RC_CENTER))),
                  arm=msp.RC_MIN)
        time.sleep(0.2)
        br.stop()

    print("\n")
    print("  1. JETSON took the sticks     ", "PASS" if took else "not seen")
    print("  2. TRANSMITTER got them back  ", "PASS" if gave_back else "NOT SEEN")
    print("  3. TRANSMITTER disarmed       ", "PASS" if disarmed else "not seen")
    print(f"  flight modes seen              {', '.join(sorted(seen_modes)) or 'none'}")
    if "ANGLE" not in seen_modes:
        print("\n  WARNING: ANGLE was never active. The follower sends ANGLE-mode")
        print("  sticks; in ACRO they are read as rates and the aircraft will not")
        print("  do what the plan asks. Check the always-on aux row took.")
    if mixed_seen:
        print("\n  WARNING: channels 1-4 were only PARTLY ours at some point.")
        print("  That is what a wrong msp_override_channels_mask looks like.")
        print("  Check `get msp_override_channels_mask` reads 15.")
    ok = took and gave_back
    print("\n" + ("  RESULT: PASS - the TRANSMITTER can take control back." if ok else
                  "  RESULT: INCOMPLETE - step 2 never happened. Do not fly."))
    if ok and not disarmed:
        print("  (step 3 not exercised: arm and disarm with this running to check it)")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
