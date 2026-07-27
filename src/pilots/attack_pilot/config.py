"""
Attack pilot v2 params - an attitude-setpoint visual servo. The sim balances; we steer.

Same Phase-2 constraints as always (no ATTITUDE / ODOMETRY / LOCAL_POSITION_NED / GATE_INFO): a pure
visual servo on the HSV detector, sending ATTITUDE setpoints that the sim's stabilised controller
holds (sysid tab 6: attitude gain 1.0, ABSOLUTE yaw tracks the command within 1 deg). ROLL is parked
at 0 forever - yaw is the only steering, exactly like hover_pilot (flight-proven interface + signs).

The mission is COMPLETION: creep through every gate, speed is nobody's problem.

Image convention:  offset_x  -1 = left edge,  +1 = right edge  (+x = aim point is to your RIGHT)
                   offset_y  -1 = top,        +1 = bottom      (+y = aim point is LOW in the frame)
"""

# =============================================================================================
# VERTICAL - thrust holds the AIM POINT on the fly-at-gate-height image row.
# The row is COMPUTED, not tuned: a gate at drone height sits tan(uptilt - pitch_cmd) below the
# optical axis (camera 20 deg up; our own commanded pitch is known exactly - the sim holds it).
# v1 died here twice by servoing "elevation" through an ESTIMATED attitude: biased el read the gate
# below the horizon while flying 1 m UNDER the opening (run 234900, alt_est 0.39 m).
# =============================================================================================
ATK_HOVER      = 0.30   # baseline hover thrust GUESS - the integral trim below finds the real one.
                        #   (Sysid said 0.299, v1 flights acted ~0.32, run 092333 hovered at 0.261 -
                        #   it varies, so a fixed feedforward ALWAYS leaves a P-droop.)
ATK_KP_VERT    = 0.16   # thrust per unit of row error. RAISED from 0.10 (run 094047: arrived LOW at
                        #   gate 3, and after a crash the row error alone couldn't lift it off the
                        #   floor) - assertive both ways, per flight feedback.
ATK_KI_VERT    = 0.02   # INTEGRAL TRIM (thrust/s per unit of row error): learns the true hover
                        #   thrust so the row error goes to ZERO (run 092333 hovered 0.42 of oy high
                        #   without it). ONLY integrates when |vz_est| < ATK_TRIM_VZ_GATE - i.e. near
                        #   vertical equilibrium. Run 094047 taught why: during a long descending
                        #   approach the row error is GEOMETRY, not hover bias, and the ungated trim
                        #   wound down to -0.048 and dragged a sink through the whole gate-3 commit.
ATK_TRIM_VZ_GATE = 0.35 # trim learns only when |vz_est| is below this (m/s) - equilibrium only
ATK_TRIM_MAX   = 0.06   # clamp on the learned trim (anti-windup; +/-0.06 covers 0.24..0.36 hover)
ATK_KD_VZ      = 0.03   # thrust per m/s of IMU-integrated climb - quells the bob P-only rows had
ATK_KD_VZ_COMMIT = 0.10 # stronger vz-arrest INSIDE the commit window: the row servo freezes there,
                        #   and any leftover climb/sink coasts straight into a gate bar (run 101632
                        #   bumped gate 1's TOP arriving from below with residual climb). This term
                        #   nulls the vertical drift while the aim is frozen.
ATK_COMMIT_UP_OFFY = 0.25 # commit escape clause for the TILTED gate: if the hole climbs beyond this
                        #   far ABOVE image centre during the coast, follow it up. A straight-gate
                        #   commit never breaches this (aim settles near 0 by the pass, every logged
                        #   run); the forward-tilted gate's hole marched +0.15 -> -0.61 while the
                        #   frozen-level coast clipped its BOTTOM bar (run 122217, t=68.7).
ATK_KP_COMMIT_UP = 0.08 # thrust per unit of aim-above-deadband during commit (climb-only, damped by
                        #   the vz-arrest; ~+0.03 max - a nudge up onto the tilted passage line)
ATK_COMMIT_SIDE_OFFX = 0.30 # LATERAL twin of the commit escape clause: if the aim escapes sideways
                        #   beyond this mid-coast, nudge the roll after it. Run 130224 coasted the
                        #   latched commit wings-level while the aim walked to -0.41 and scraped the
                        #   LEFT pillar at the pass. Normal commits stay inside +/-0.2 to the end.
ATK_KP_COMMIT_SIDE = 0.5 # roll (rad) per unit of aim beyond the deadband, capped at ATK_ROLL_BLIND
                        #   (3.4 deg) - a drift correction, never a carve, inside a gate's throat
ATK_THRUST_UP  = 0.18   # climb authority above hover (0.48 ceiling). RAISED from 0.12: post-crash,
                        #   grounded with the gate in view above, base+0.12 was barely over the real
                        #   hover thrust - it sat there while the trim crawled. 0.18 lifts it NOW.
ATK_THRUST_DN  = 0.12   # descent authority below hover (0.18 floor - assertive, never a dive)
ATK_OFFY_BIAS  = -0.06  # aim row offset: hold the opening slightly ABOVE the at-height row = fly a
                        #   touch BELOW the gate line. FoV margin: the up-tilted camera sees only
                        #   ~9 deg below the horizon, so losing a gate out the frame BOTTOM (too
                        #   high) is unrecoverable; losing it out the top is impossible (+49 deg).
ATK_CRUISE_UP  = 0.12   # EXTRA row bias while the gate is FAR: hold it LOWER in frame = TRANSIT
                        #   HIGH (~1 m above the gate line at 15 m), then glide down as it nears.
                        #   The pilot cannot see non-gate obstacles (detector is gate-orange only),
                        #   but every obstacle on this course is parked ON THE FLOOR - run 121021
                        #   sagged to wing height mid-transit and hung up on a fighter's wing 3.4 s
                        #   after gate 4. Fly over what you cannot see.
ATK_CRUISE_FADE_AREA = 0.05  # the cruise-up bias fades to zero by this blob size (~approach range),
                        #   so ALIGN/COMMIT geometry is completely unchanged
ATK_VZ_TAU     = 4.0    # leak time-constant (s) of the IMU vertical-velocity integrator

# =============================================================================================
# YAW - the steering. Integrated ABSOLUTE heading, nudged toward the aim point once per frame.
# Nudge numbers are hover_pilot's, verified in flight ON THIS INTERFACE (incl. the sign:
# +yaw = clockwise/right here - opposite the rate-mode convention).
# =============================================================================================
ATK_YAW_SPAWN = 1.694     # the SPAWN FACING in the sim's yaw frame (rad, ~+97 deg) - the heading
                          #   command that means "keep facing the way we started". SOLVED from two
                          #   calibration flights (the frame is CCW-POSITIVE, standard math sign):
                          #     cmd 0    -> snapped 97 deg CLOCKWISE  (spawn is +97, going to 0)
                          #     cmd -97  -> snapped ~180 deg COUNTER-clockwise (predicted 166:
                          #                 -97 - (+97) = -194 -> shortest path +166 CCW)
                          #   Only spawn=+97 in a CCW-positive frame fits BOTH. MEASURED interface
                          #   calibration, like YAW_SIGN. Re-measure if the spawn ever changes: fly
                          #   once commanding 0, integrate zgyro over the snap. Small residual error
                          #   is fine - vision steers it out while the gate is in the FoV.
                          #   (hover_pilot has this same latent bug - it seeds heading at 0, AND its
                          #   "+yaw = right" config note is falsified by these flights.)
ATK_KP_YAW       = -0.10  # heading nudge per unit of offset_x, per camera frame. NEGATIVE: the
                          #   setpoint frame is CCW-POSITIVE (proven by the two yaw-calibration
                          #   flights above), so a gate to the RIGHT (offset_x > 0) needs the
                          #   heading command to DECREASE. (hover_pilot's +0.30 note assumed the
                          #   opposite frame and is falsified by the same flights.)
                          #   MAGNITUDE 0.10, down from 0.30: the sim's yaw actuation lags ~150 ms
                          #   (~5 camera frames), and at 0.30 the per-frame integrator exceeded unit
                          #   loop gain per lag -> a ±5 deg limit cycle (run 092333, heading hunting
                          #   85<->93 deg at ~1.3 s period). 0.10 keeps the loop well under it.
                          #     hunts side to side -> LOWER      too slow onto an off gate -> RAISE
ATK_YAW_ON       = 0.25   # yaw only recenters when the aim point drifts beyond this - COARSE FoV
                          #   keeping. Inside it the heading FREEZES and ROLL does the lateral work:
                          #   at creep speed yaw doesn't change the direction of travel (a quad
                          #   translates by TILTING; the nose is just a camera mount), so fine yaw
                          #   corrections only skidded the airframe under a fixed velocity vector.

# =============================================================================================
# ROLL - the LATERAL translation, pitch-creep style: small bank, strafe, level again.
# offx -> roll is position -> ACCELERATION (double integrator - v1's teeter-totter, at 6x this
# authority), so it is DAMPED on the aim point's measured lateral rate, not P-only.
# Sign: +roll = drone RIGHT (+y body), gain 1.0 - sysid tab 6, measured on THIS interface.
# =============================================================================================
# AUTHORITY PHILOSOPHY (Brian's bang/counter-bang theory, and it is correct): for tilt->acceleration
# the time-optimal move is a hard bank to build lateral speed and a symmetric counter-bank to kill
# it. This PD IS that maneuver in continuous form - KP is the bang, KD is the counter - and the caps
# below choose the point on the timid<->crisp spectrum. The stability limit is MEASUREMENT LAG: the
# counter needs closure speed, which arrives through 30 Hz vision + smoothing ~100-150 ms late, so
# authority can rise only as far as a late counter stays safe (v1 proved 28 deg with bad damping is
# past the cliff). Stepped 5.7 -> 10 deg after gate 3 went wide right (run 101632); if the logs stay
# clean-damped, another step up is justified; if it ever wags, back off ROLL_MAX first.
ATK_KP_LAT   = 0.50   # roll (rad) per unit of aim offset_x (saturates the cap at |offx| ~ 0.34).
ATK_KD_LAT   = 1.10   # roll (rad) per unit/s of aim lateral RATE - the COUNTER-BANG: rolls back
                      #   level (or past it) BEFORE the centre is crossed. Scaled with sqrt(KP).
ATK_OXRATE_MAX  = 0.30  # clamp on the measured lateral rate - the damping's ceiling. MUST rise with
                        #   authority: at higher strafe speeds a saturated damping signal = a late,
                        #   weak counter (the parallax spike this clamp was born for is now handled
                        #   by the OYRATE gate below, so it can afford to be loose).
ATK_OYRATE_GATE = 0.25  # when the aim is moving VERTICALLY faster than this (unit/s - climb or
                        #   descent transient), the lateral-rate sample is discarded entirely: fast
                        #   vertical apparent motion pollutes the x-rate with parallax.
ATK_ROLL_BLIND  = 0.06  # roll cap (rad ~= 3.4 deg) while lateral-rate samples are being DISCARDED
                        #   (climb/descent transients). Run 105441's climb gate: 2.5 s of discarded
                        #   samples left the roll loop undamped-P at full cap on a constant-bearing
                        #   pursuit - bearing never moved, so it silently built ~3 m/s of sideways
                        #   momentum and sailed past the gate. No hard banking on an unmeasured axis.
ATK_CLIMB_GATE  = 0.25  # row error beyond which the forward creep STOPS (the vertical twin of the
                        #   lateral ALIGN rule): a big climb is flown IN PLACE, at gate height first,
                        #   THEN advance. Creeping forward during the 2.5 s climb is what turned a
                        #   small drift into "already past the gate" (run 105441, t=41.6-44).
ATK_ROLL_MAX = 0.17   # bank cap (rad ~= 10 deg) -> ~1.7 m/s^2 lateral. Assertive, still recoverable
                      #   within the vision lag envelope.
ATK_ROLL_SLEW = 0.60  # max roll-command slew (rad/s) - the bang AND the counter must both develop
                      #   fast enough to matter (0.30 took 0.57 s just to reach the old cap).
ATK_YAW_STEP_MAX = 0.03   # cap per frame (~52 deg/s at 30 fps)
ATK_YAW_MAX_OFF  = 0.95   # anti-windup: stop turning once the aim point is AT the frame edge.
                          #   RAISED from 0.85: a gate acquired mid-SEEK-sweep appears at the edge
                          #   (~0.9) and must still get pulled toward centre, not ignored.

# =============================================================================================
# SEEK - sharp corners. At a ~90 deg turn the next gate leaves the FoV before the pass finishes
# (gate 3 -> 4 goes hard LEFT; later gates go RIGHT). While approaching gate N the pilot NOTES the
# absolute heading of the biggest OTHER detection (gate N+1) - perception-derived memory, works for
# either direction. Post-pass and blind, SEEK holds station and ROTATES toward/past that heading
# until vision acquires; the FoV catches the gate up to 45 deg before the nose does. Stale or absent
# memory -> same rotation, default direction: a slow pirouette scan costs seconds we have.
# =============================================================================================
ATK_NEXT_MIN_AREA = 0.004 # candidate floor for "that's the next gate, note its bearing"
ATK_NEXT_HOLDOFF  = 1.5   # NO memory writes for this long after a pass: those frames are full of
                          #   the receding gate's fragments sliding off-frame past the single-blob
                          #   avoid zone. Run 124505: 0.19 s after pass 9, a fragment on the RIGHT
                          #   overwrote a correct LEFT memory (+69.8 -> -15.5) and the tilted-gate
                          #   corner turned into a 300-degree wrong-way orbit. The pre-pass memory
                          #   rides through the holdoff and still points the right way.
ATK_NEXT_MEM_S    = 6.0   # memory freshness at pass time - older sightings only pick the direction
ATK_SEEK_S        = 12.0  # post-pass window in which blind = SEEK (rotate), not plain HOLD
ATK_SEEK_RATE     = 0.5   # SEEK rotation rate (rad/s ~= 29 deg/s) - a deliberate pan, FoV does the
                          #   finding; vision interrupts the sweep the moment a gate shows
# The next-gate memory is a SIGHTING TRACK, not a bearing: WHERE (absolute heading), HOW HIGH
# (vertical offset RELATIVE to the then-tracked gate - attitude/altitude invariant, Brian's
# formulation: "it was at the same offset as the one we were tracking"), and HOW BIG. During the
# post-pass window, acquisition must satisfy ALL THREE or the candidate is refused and the sweep
# keeps turning. This corner (post-slanted) is the acid test: elevated exit, course doubling back,
# cross-floor gates littering the cone - bearing alone kept accepting the wrong one.
ATK_SEEK_CONE     = 0.45  # bearing tolerance (rad ~= 26 deg) around the remembered heading
ATK_SEEK_OY_TOL   = 0.40  # vertical tolerance: candidate offset_y vs (current row target + the
                          #   remembered relative elevation). Rejects the down-low cross-floor gate
                          #   that a bearing-only cone accepted.
ATK_SEEK_AREA_RATIO = 0.25 # candidate must be at least this fraction of the remembered size - we
                          #   are CLOSER now than when it was sighted; a distant speck is not it.
ATK_PREEMPT_RATIO = 1.6   # YOUNG-LOCK PREEMPTION (post-pass window only, fail-open): a candidate
                          #   this many times BIGGER than the current lock steals it. The corner
                          #   sweep meets gates in the wrong order - the far N+2 enters frame before
                          #   the close true-next N+1 and got locked first (descending for it); N+1
                          #   is ~half the distance, so ~4x bigger the moment it appears. Switch,
                          #   never refuse. Straight handoffs unaffected (their lock IS the biggest).
ATK_POSTPASS_GENTLE_S = 4.0 # for this long after a pass, descent authority is capped - a wrong
                          #   mid-corner lock cannot spend much altitude before preemption fixes it
ATK_POSTPASS_DN   = 0.05  # the capped below-hover thrust band during that window (~0.5 m/s descent)
ATK_POSTPASS_UP   = 0.06  # ...and the capped ABOVE-hover band: a surviving junk lock fired FULL
                          #   climb (thr 0.45) at the gate plane 0.2 s after a pass and popped the
                          #   drone into the INSIDE of the top bar on exit (run 164646, t=53.0)
ATK_JUNK_OY       = -0.5  # post-pass sanity: a candidate higher in frame than this (~50 deg above
                          #   the horizon) cannot be a course gate - it is the just-passed gate's
                          #   own top bar or ceiling junk. Excluded from acquisition/preemption
                          #   during the post-pass window only; fail-open if it would empty the list.
ATK_SEEK_SINK     = 0.4   # during SEEK, sink at this rate (m/s) back toward PASS altitude when more
                          #   than ATK_SEEK_ALT_TOL above it. The pass just happened THROUGH a gate,
                          #   so altitude-at-pass is a trustworthy gate-line reference even with
                          #   integrator drift. Run 115646: a junk post-pass lock climbed it to 3 m,
                          #   SEEK held 3 m faithfully, and from there the NEAR next gate is below
                          #   the camera's 9-deg down-limit - it locked a FAR gate and flew over the
                          #   one it should have taken.
ATK_SEEK_ALT_TOL  = 0.4   # dead-band (m) above pass altitude before the SEEK sink engages
ATK_LOCK_CONFIRM  = 4     # a fresh lock gets NO vertical authority until it survives this many
                          #   consecutive frames - the junk blob that caused the 3 m climb lived for
                          #   ~2. Real gates confirm in 4 frames (~0.13 s) and lose nothing.

# =============================================================================================
# PITCH - a small constant forward creep, faded while off-aim. ~0.05 rad => ~1 m/s.
# The 8-min cap over a ~140 m course needs only ~0.4 m/s average - never rush a gate.
# =============================================================================================
ATK_PITCH_FWD  = 0.05   # forward lean (rad) when aimed
ATK_AIM_REF    = 0.40   # creep fades to 0 as |offset_x| approaches this (turn first, then close)
ATK_PITCH_SLEW = 0.15   # max pitch-command slew (rad/s) - eases the creep in/out, no lurches
ATK_HOLD_KP = 0.15      # STATION-HOLD gain for near-gate climbs (hover_pilot's proven range-hold):
                        #   pitch per unit of range error (range ~ 1/sqrt(area)), anchored to the
                        #   range where the climb began. A CONSTANT back-pitch brake there kept
                        #   accelerating backward over the multi-second climb (backed away, clipped
                        #   the top, no pass) - a servo brakes only as much as needed.
ATK_ALIGN_BRAKE = 0.035 # BACK-pitch (rad) held during ALIGN - zeroing the creep does NOT stop the
                        #   momentum already carried (run 111346: coasted from area 0.20 to 0.57 -
                        #   inside the gate's throat - before alignment finished, then clipped the
                        #   top bar entering banked and climbing). A gentle backlean actually
                        #   arrests, so "stop and slide onto the axis" happens AT the commit
                        #   boundary, with the whole gate still ahead. (hover_pilot's proven brake.)

# =============================================================================================
# COMMIT + PASS - near the gate the aim point balloons; stop chasing it and fly straight through.
# Pass = tracked blob peaked then receded (proven across every pilot in this repo).
# =============================================================================================
ATK_ALIGN_AREA  = 0.10  # blob fraction where a MISALIGNED approach starts braking + strafing (~4-5 m
                        #   out). Waiting until COMMIT range left no room: run 113145 entered ALIGN
                        #   ~2 m from gate 3 carrying speed and 0.3 of offset, and scraped down the
                        #   gate's side pillar (the "invisible obstacle" - a point-blank pillar is
                        #   just a featureless orange wall on camera). Aligned approaches ignore
                        #   this band and cruise straight through to commit.
ATK_COMMIT_AREA = 0.20  # blob fraction where the gate is NEAR: if ALIGNED we commit
ATK_COMMIT_ALIGN = 0.15 # |aim offset_x| required to actually COMMIT. Run 095624 committed on area
                        #   alone with the aim 0.28 LEFT of centre, slewed the mid-correction roll
                        #   (-7.5 deg) back to level, and coasted wide right into the gate edge. Near
                        #   but NOT aligned = the ALIGN regime: forward creep OFF (this pilot can
                        #   simply STOP), roll + row servos stay on - slide onto the axis, THEN go.
ATK_COMMIT_VZ   = 0.7   # max |vertical speed| (m/s) to ENTER commit. Run 132724 committed while
                        #   still descending -0.97 off the cruise glide and sagged into the bottom
                        #   bar; every healthy commit entered under 0.5. While hot, the ALIGN brake
                        #   holds station until the vertical settles - then commit, level.
ATK_COMMIT_ROW  = 0.35  # VERTICAL twin of the alignment requirement: |aim_oy - row target| must be
                        #   inside this to commit. Run 122735: the big-climb gate COMMITTED at
                        #   aim_oy -0.36 with vz +2.97 (mid-climb, hole far above centre); the
                        #   commit arrest then chopped the climb and it sagged into the bottom bar.
                        #   Healthy commits run err < 0.15; blocked ones climb-in-place (existing
                        #   climb-first branch) until level with the line, THEN commit.
ATK_PASS_AREA   = 0.30  # arms a pass - ONLY while committed (aligned + close). Area receding from a
                        #   lateral scrape used to count as a "pass" (run 095624 counted 6).
ATK_PASS_DROP   = 0.7   # counted when area falls to this * peak
ATK_PASS_REARM  = 0.15  # refractory: no re-arm until the passed gate shrinks below this

# =============================================================================================
# TRACKING - follow ONE gate across frames (v1's tracker, kept: the data showed it working).
# Acquisition prefers detections with a LOCATED OPENING - the az+40 station-sign decoy that stole
# two flights never has one.
# =============================================================================================
ATK_MIN_AREA     = 0.002  # ignore smaller blobs (next gate after a pass reads ~0.0025)
ATK_TRACK_RADIUS = 0.20   # max in-frame jump to still be the SAME gate
ATK_TRACK_MAX_MISS = 5    # frames unmatched before re-acquiring
ATK_HOP_DIST     = 0.25   # bigger tracked jump = lock hop -> reseed the smoother
ATK_AVOID_RADIUS = 0.30   # post-pass: exclude detections near the flown-through gate
ATK_AVOID_MIN_AREA = 0.10 # a detection only KEEPS the avoid lock if it is still BIG - the genuinely
                          #   just-passed gate filled the frame moments ago. Without this the avoid
                          #   marker transferred onto the NEXT gate when it drifted near the stale
                          #   position (run 110222: the pitched gate at 11 m wore the avoid ellipse
                          #   while the tracker locked a 0.007 speck at 22 m and flew at that).
ATK_AVOID_S      = 2.5    # hard expiry (s after the pass) - the avoid's whole job is the 1-2 s
                          #   handoff window; a stale marker is only a liability after that.
ATK_OFF_ALPHA    = 0.4    # EMA on the aim offsets
ATK_MAX_STALE    = 6      # frames without a fresh measurement before flying HOLD (hover, no steer)
