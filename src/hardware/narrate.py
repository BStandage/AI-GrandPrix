"""What the autopilot was thinking, in sentences, while it flew.

The CSV says what the numbers were. This says what the aircraft BELIEVED, so
that after a run you can put it next to what you saw with your own eyes and
find the place where the two stop agreeing. d44 flight 4 on 2026-09-21 is the
case that earned this: the log had every number needed to explain the climb
into gate 1's top bar, and it still took an evening of reading CSV columns to
see it. In sentences it would have been one line -

    18.6  gate 1 at 2.4 m: 0.35 m ABOVE me -> CLIMBING   <-- and she was not

EVENT DRIVEN, NOT PER TICK. The control loop runs at rc_hz and nobody can read
50 lines a second. A line appears when something CHANGES: a gate is acquired
or lost, the vertical verdict flips, a range band is crossed, the aircraft
commits, a gate is crossed. A quiet approach is a few lines; a bad one is
noisy, which is itself the signal.

Every verdict carries a HYSTERESIS band so a number sitting on a threshold
does not chatter. Everything goes to stdout and to a .log file beside the CSV.
"""

from __future__ import annotations

import math

# Vertical verdict bands, metres of commanded height offset. LEVEL is anything
# inside ENTER; once out, it takes coming back inside EXIT to read LEVEL again.
# 0.08 m is well under the 0.75 m gate half-opening but well over the 0.02 m
# of jitter a clean approach shows, so it speaks when it matters and not
# otherwise.
LEVEL_ENTER_M = 0.08
LEVEL_EXIT_M = 0.05
# A verdict must PERSIST this long before it is spoken. Without it, replaying
# d44 flight 4 produced nine verdict flips in eight seconds and buried the one
# that mattered. The real signal - a runaway - lasts seconds, not frames.
VERDICT_DWELL_S = 0.4
# Reject a detection whose range jumps absurdly: after the flight-4 bar strike
# the detector reported 154 m, then 6 m, then 12.6 m on consecutive frames, and
# a narrator with no plausibility filter announces each one as a gate sighting.
RANGE_JUMP_MAX_MPS = 12.0
# Consecutive plausible frames before a sighting is announced. The detector
# found a gate in 99.6 % of bench frames and also found one with the lens
# covered, so a single frame is not evidence. After the flight-4 strike it
# produced a "gate" on most frames at 20-60 m while sitting on the ground.
ACQUIRE_CONFIRM = 3
# Beyond this there is no gate worth narrating: the longest leg on the course
# is well inside it, and a reading past it is the detector finding scenery.
RANGE_SANE_MAX_M = 30.0
RANGE_BANDS_M = (8.0, 6.0, 5.0, 4.0, 3.0, 2.0)
LOST_AFTER_S = 0.5
THROTTLE_LOUD = 1380          # near the plan's 1410 worst case: say so

WHY_PLAIN = {
    "size": "ring fills the frame",
    "both_edges": "gate overfills the frame, top and bottom both cut",
    "width_clipped": "gate is wider than the frame",
    "no_ring": "no ring box",
    "detector": "detector says the height is unusable",
    "lost": "gate lost",
}


def _verdict(el_deg, dz_cmd):
    """Plain words, in the units the aircraft actually flies.

    NOT in metres of gate height. Metres would need range*sin(elevation), and
    range is the one camera signal this whole design refuses to depend on - it
    read 12.6, 22.9 and 59.5 m on a single approach. Narrating in metres would
    import that error into the record you use to judge the flight, and a
    narration that lies is worse than none.

    ELEVATION IN DEGREES is what gate_dz returns and what set_gate_elevation
    consumes. The command in metres beside it is VERT_EL_GAIN * degrees,
    capped. Both are exact; neither needs a range."""
    d = el_deg if el_deg is not None else dz_cmd
    word = "ABOVE me -> CLIMBING" if d > 0 else "BELOW me -> EASING DOWN"
    if el_deg is None:
        return f"{abs(dz_cmd):.2f} m {word}"
    return f"{abs(el_deg):.1f} deg {word} (commanding {dz_cmd:+.2f} m)"


class Narrator:
    """Call tick() every control cycle. It speaks only when something changes."""

    def __init__(self, gate_h_m=None, path=None, sink=print):
        self.gate_h = gate_h_m
        self.sink = sink
        self.fh = open(path, "w", encoding="utf-8") if path else None
        self.gate = None            # the event we are currently working
        self.seen = False           # gate in view right now
        self.t_lastdet = None
        self.band = None            # last range band announced for this gate
        self.level = True           # current vertical verdict
        self.committed = False
        self.said_clipped = False
        self.airborne = False
        self.loud = False
        self.last_cross = None
        self.pending = None         # (is_level, t_first_seen) awaiting dwell
        self.last_rng = None
        self.last_rng_t = None
        self.just_acquired = False
        self.confirm = 0

    def say(self, t, msg):
        line = f"{t:6.1f}  {msg}"
        self.sink(line)
        if self.fh:
            self.fh.write(line + "\n")
            self.fh.flush()

    def close(self):
        if self.fh:
            self.fh.close()
            self.fh = None

    def _new_gate(self, ev):
        self.gate, self.band, self.committed = ev, None, False
        self.level, self.said_clipped = True, False
        self.pending, self.last_rng, self.last_rng_t = None, None, None
        self.confirm = 0

    def tick(self, t, *, event=None, det=None, rng=None, dz=None, z=None,
             throttle=None, airborne=None, committed=False, why=None,
             cross_ev=None, cross_lat=None, cross_dz=None, el_deg=None):
        # PLAUSIBILITY. Range is the camera's weakest signal and it does not
        # fail gracefully - it fails by orders of magnitude. A gate cannot
        # recede at 12 m/s while we fly at 1.5, so a jump like that is the
        # detector finding something that is not the gate. Drop the frame
        # rather than narrate it.
        if det is not None and rng is not None and rng > RANGE_SANE_MAX_M:
            self.confirm = 0
            det = None                      # scenery, not a gate
        if det is not None and rng is not None and self.last_rng is not None:
            dt_r = max(1e-3, t - self.last_rng_t)
            if abs(rng - self.last_rng) / dt_r > RANGE_JUMP_MAX_MPS:
                self.confirm = 0
                if self.seen:
                    self.say(t, f"gate {self.gate}: detection IMPLAUSIBLE "
                                f"({self.last_rng:.1f} m -> {rng:.1f} m), ignoring it")
                    self.seen = False
                self.last_rng, self.last_rng_t = rng, t
                return
        if det is not None and rng is not None:
            self.last_rng, self.last_rng_t = rng, t
        # ---- airborne
        if airborne and not self.airborne:
            self.airborne = True
            self.say(t, f"AIRBORNE at {z:.2f} m" if z is not None else "AIRBORNE")

        # ---- a gate was crossed. cross_* are what the aircraft believes its
        # own miss distance was; it fitted through a 1.5 m opening, so anything
        # over 0.75 m here is position error it cannot see.
        if cross_ev is not None and cross_ev != self.last_cross:
            if self.last_cross is not None:
                side = "right" if (cross_lat or 0) > 0 else "left"
                extra = ""
                if cross_lat is not None:
                    extra = f", {abs(cross_lat):.2f} m to the {side}"
                    if abs(cross_lat) > 0.75:
                        extra += " (OUTSIDE the opening - that is position error)"
                if cross_dz is not None:
                    extra += f", {abs(cross_dz):.2f} m {'high' if cross_dz > 0 else 'low'}"
                self.say(t, f"gate {self.last_cross} CROSSED{extra}")
            self.last_cross = cross_ev

        # ---- which gate are we working
        if event is not None and event != self.gate:
            self._new_gate(event)

        # ---- acquired / lost
        fresh = det is not None
        if fresh:
            self.t_lastdet = t
            self.confirm += 1
            if not self.seen and self.confirm >= ACQUIRE_CONFIRM:
                self.seen = True
                self.just_acquired = True
                r = f"{rng:.1f} m" if rng else "range unknown"
                v = ("LEVEL with me" if dz is None or abs(dz) <= LEVEL_ENTER_M
                     else _verdict(el_deg, dz))
                self.say(t, f"gate {self.gate} IN SIGHT at {r}, {v}")
                # the bands are per-sighting: a re-acquire should count down again
                self.band = None
        elif self.t_lastdet is not None and t - self.t_lastdet > LOST_AFTER_S:
            self.confirm = 0
            if not self.seen:
                return
            self.seen = False
            self.say(t, f"gate {self.gate} LOST - fading the height reference out, "
                        f"holding vertical speed")

        if fresh and self.seen:
            # ---- range bands, counting down
            if rng:
                for b in RANGE_BANDS_M:
                    if rng <= b and (self.band is None or b < self.band):
                        was, self.band = self.band, b
                        if was is None and self.just_acquired:
                            break          # the IN SIGHT line just said this
                        v = ("LEVEL" if abs(dz or 0.0) <= LEVEL_ENTER_M
                             else _verdict(el_deg, dz))
                        # a height before takeoff is the barometer talking to
                        # itself; do not dress it up as an altitude
                        h = f", holding {z:.2f} m" if (z is not None and self.airborne) else ""
                        self.say(t, f"gate {self.gate} at {rng:.1f} m: {v}{h}")
                        break

            # ---- vertical verdict flips, with hysteresis
            if dz is not None and not self.committed:
                want_level = self.level
                if self.level and abs(dz) > LEVEL_ENTER_M:
                    want_level = False
                elif not self.level and abs(dz) < LEVEL_EXIT_M:
                    want_level = True
                if want_level == self.level:
                    self.pending = None
                else:
                    if self.pending is None or self.pending[0] != want_level:
                        self.pending = (want_level, t)
                    elif t - self.pending[1] >= VERDICT_DWELL_S:
                        self.level, self.pending = want_level, None
                        where = f" (at {rng:.1f} m)" if rng else ""
                        self.say(t, f"gate {self.gate} back to LEVEL{where}" if want_level
                                 else f"gate {self.gate} is {_verdict(el_deg, dz)}{where}")

            self.just_acquired = False

            # ---- the reconstruction kicked in
            if getattr(det, "clipped_v", False) and not self.said_clipped:
                self.said_clipped = True
                self.say(t, f"gate {self.gate}: ring is cut off by the frame edge, "
                            f"rebuilding its centre from the width")

            # ---- COMMIT
            if committed and not self.committed:
                self.committed = True
                r = f"{rng:.1f} m" if rng else "close range"
                held = f", holding {z:.2f} m" if z is not None else ""
                self.say(t, f"gate {self.gate} COMMIT at {r} ({WHY_PLAIN.get(why, why)}) "
                            f"- height reference released{held}, flying through")

        # ---- throttle near its ceiling is worth hearing about while it happens
        if throttle is not None:
            if throttle >= THROTTLE_LOUD and not self.loud:
                self.loud = True
                self.say(t, f"THROTTLE {throttle} - near the {THROTTLE_LOUD}+ ceiling, "
                            f"the loop is asking for a lot of lift")
            elif throttle < THROTTLE_LOUD - 40 and self.loud:
                self.loud = False
                self.say(t, f"throttle back to {throttle}")
