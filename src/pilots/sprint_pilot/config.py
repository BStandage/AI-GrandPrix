"""
Sprint pilot v1 params - steady_pilot's stop-and-align machinery plus a bang/counter-bang transit.

FORK of steady_pilot v2 (the 2:10 VQ2 qualifier, byte-preserved next door - it stays the reliable
qualifier). Same Phase-2 constraints as always (no ATTITUDE / ODOMETRY / LOCAL_POSITION_NED /
GATE_INFO): a pure visual servo on the HSV detector, sending ATTITUDE setpoints that the sim's
stabilised controller holds (sysid tab 6: attitude gain 1.0, ABSOLUTE yaw within 1 deg).

The mission is TIME: steady banked a huge completion margin, now we spend it where it is free - the
transits. Everything within align range of a gate is steady's proven code, untouched; the TRANSIT
section at the bottom of this file is the only new physics. F0 budget (steady qual log 180905):
TRANSIT 70 s (54%), TURNING 23 s, COMMIT 18 s, ALIGN 14 s, BLIND 5 s of the 130 s lap.

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
SPRINT_HOVER      = 0.27   # baseline hover thrust - MEASURED, no longer steady's 0.30 guess. Steady's
                        #   full qual lap converged its trim to -0.034 (= real hover 0.266) and F1b
                        #   (run 185331) proved sprint can't afford the learning time: trim was only
                        #   -0.007 at the first commit, and the leftover +0.027 bias over the 0.10
                        #   vz-arrest gain = the observed +0.27 m/s climb through the whole commit
                        #   coast - straight into gate 1's top bar. Trim still owns the residual.
SPRINT_KP_VERT    = 0.16   # thrust per unit of row error. RAISED from 0.10 (run 094047: arrived LOW at
                        #   gate 3, and after a crash the row error alone couldn't lift it off the
                        #   floor) - assertive both ways, per flight feedback.
SPRINT_KI_VERT    = 0.02   # INTEGRAL TRIM (thrust/s per unit of row error): learns the true hover
                        #   thrust so the row error goes to ZERO (run 092333 hovered 0.42 of oy high
                        #   without it). ONLY integrates when |vz_est| < SPRINT_TRIM_VZ_GATE - i.e. near
                        #   vertical equilibrium. Run 094047 taught why: during a long descending
                        #   approach the row error is GEOMETRY, not hover bias, and the ungated trim
                        #   wound down to -0.048 and dragged a sink through the whole gate-3 commit.
SPRINT_TRIM_VZ_GATE = 0.35 # trim learns only when |vz_est| is below this (m/s) - equilibrium only
SPRINT_TRIM_MAX   = 0.06   # clamp on the learned trim (anti-windup; +/-0.06 covers 0.24..0.36 hover)
# ---- CASCADED VERTICAL (run 221609): row error -> bounded climb-rate target -> thrust. ------
# The direct KP_VERT*err - KD_VZ*vz law (KD 0.03) implied an equilibrium climb of ~5.9 m/s per
# unit of row error - a structural runaway: err +1.11 was chased to vz +2.9 with thrust pinned
# at the band top, then slammed to 0.076 to arrest ("thrust to 0" - Brian). The cascade caps the
# climb rate regardless of how big the row step is, and arrests automatically as the row closes.
SPRINT_VZ_PER_ERR = 2.0    # climb-rate target (m/s) per unit of row error (0.25 err -> 0.5 m/s)
SPRINT_VZ_MAX     = 1.2    # hard cap on the commanded climb/sink rate (m/s) - a big vertical
                           #   step is flown as a CONTROLLED 1.2 m/s elevator ride, not a slam
SPRINT_KP_VZ      = 0.15   # thrust per m/s of climb-rate error (0.15 * 1.2 = full band at the
                           #   worst rate error - saturates exactly at the authority limit)
SPRINT_KD_VZ      = 0.03   # RETIRED in the servo branch (cascade owns it); thrust per m/s of
                           #   IMU-integrated climb - quells the bob P-only rows had
SPRINT_KD_VZ_COMMIT = 0.10 # stronger vz-arrest INSIDE the commit window: the row servo freezes there,
                        #   and any leftover climb/sink coasts straight into a gate bar (run 101632
                        #   bumped gate 1's TOP arriving from below with residual climb). This term
                        #   nulls the vertical drift while the aim is frozen.
SPRINT_COMMIT_UP_OFFY = 0.25 # commit escape clause for the TILTED gate: if the hole climbs beyond this
                        #   far ABOVE image centre during the coast, follow it up. A straight-gate
                        #   commit never breaches this (aim settles near 0 by the pass, every logged
                        #   run); the forward-tilted gate's hole marched +0.15 -> -0.61 while the
                        #   frozen-level coast clipped its BOTTOM bar (run 122217, t=68.7).
SPRINT_KP_COMMIT_UP = 0.08 # thrust per unit of aim-above-deadband during commit (climb-only, damped by
                        #   the vz-arrest; ~+0.03 max - a nudge up onto the tilted passage line)
SPRINT_COMMIT_SIDE_OFFX = 0.30 # LATERAL twin of the commit escape clause: if the aim escapes sideways
                        #   beyond this mid-coast, nudge the roll after it. Run 130224 coasted the
                        #   latched commit wings-level while the aim walked to -0.41 and scraped the
                        #   LEFT pillar at the pass. Normal commits stay inside +/-0.2 to the end.
SPRINT_KP_COMMIT_SIDE = 0.5 # roll (rad) per unit of aim beyond the deadband, capped at SPRINT_ROLL_BLIND
                        #   (3.4 deg) - a drift correction, never a carve, inside a gate's throat
SPRINT_THRUST_UP  = 0.18   # climb authority above hover (0.48 ceiling). RAISED from 0.12: post-crash,
                        #   grounded with the gate in view above, base+0.12 was barely over the real
                        #   hover thrust - it sat there while the trim crawled. 0.18 lifts it NOW.
SPRINT_THRUST_DN  = 0.12   # descent authority below hover (0.18 floor - assertive, never a dive)
SPRINT_OFFY_BIAS  = -0.06  # aim row offset: hold the opening slightly ABOVE the at-height row = fly a
                        #   touch BELOW the gate line. FoV margin: the up-tilted camera sees only
                        #   ~9 deg below the horizon, so losing a gate out the frame BOTTOM (too
                        #   high) is unrecoverable; losing it out the top is impossible (+49 deg).
SPRINT_CRUISE_UP  = 0.12   # EXTRA row bias while the gate is FAR: hold it LOWER in frame = TRANSIT
                        #   HIGH (~1 m above the gate line at 15 m), then glide down as it nears.
                        #   The pilot cannot see non-gate obstacles (detector is gate-orange only),
                        #   but every obstacle on this course is parked ON THE FLOOR - run 121021
                        #   sagged to wing height mid-transit and hung up on a fighter's wing 3.4 s
                        #   after gate 4. Fly over what you cannot see.
SPRINT_CRUISE_FADE_AREA = 0.05  # the cruise-up bias fades to zero by this blob size (~approach range),
                        #   so ALIGN/COMMIT geometry is completely unchanged
SPRINT_VZ_TAU     = 4.0    # leak time-constant (s) of the IMU vertical-velocity integrator
SPRINT_ROW_CAP_TICKS = 40  # ROW-CAPTURE LATCH (~0.45 s of sustained small row error): until the
                        #   initial climb has genuinely captured the gate row, the pilot has NO
                        #   roll authority and FLOW stays closed. The drone SPAWNS aligned; during
                        #   the ascent aim_ox is viewpoint parallax (run 205713: +0.006 -> +0.111
                        #   while measured roll was still 0.1 deg - zero real displacement), and
                        #   the P-term chasing it rolled 6 deg and MANUFACTURED the takeoff
                        #   misalignment. Climb straight, capture the row, then steer from truth.
                        #   Latches once per run; mid-course recoveries keep normal authority.
SPRINT_ROW_CAP_ERR = 0.20  # |row error| that counts as captured while the latch is arming

# =============================================================================================
# YAW - the steering. Integrated ABSOLUTE heading, nudged toward the aim point once per frame.
# Nudge numbers are hover_pilot's, verified in flight ON THIS INTERFACE (incl. the sign:
# +yaw = clockwise/right here - opposite the rate-mode convention).
# =============================================================================================
SPRINT_YAW_SPAWN = 1.694     # the SPAWN FACING in the sim's yaw frame (rad, ~+97 deg) - the heading
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
SPRINT_KP_YAW       = -0.10  # heading nudge per unit of offset_x, per camera frame. NEGATIVE: the
                          #   setpoint frame is CCW-POSITIVE (proven by the two yaw-calibration
                          #   flights above), so a gate to the RIGHT (offset_x > 0) needs the
                          #   heading command to DECREASE. (hover_pilot's +0.30 note assumed the
                          #   opposite frame and is falsified by the same flights.)
                          #   MAGNITUDE 0.10, down from 0.30: the sim's yaw actuation lags ~150 ms
                          #   (~5 camera frames), and at 0.30 the per-frame integrator exceeded unit
                          #   loop gain per lag -> a Ã‚Â±5 deg limit cycle (run 092333, heading hunting
                          #   85<->93 deg at ~1.3 s period). 0.10 keeps the loop well under it.
                          #     hunts side to side -> LOWER      too slow onto an off gate -> RAISE
SPRINT_YAW_ON       = 0.12   # yaw recenters when the aim drifts beyond this (FAR ranges only - the
                          #   servo now also requires area < ALIGN_AREA, so the ballooning near
                          #   aim still cannot whip the nose). LOWERED from 0.25 (run 223110, the
                          #   holistic review): with the deadband at 0.25, a standing offx of
                          #   0.15-0.30 froze the heading for NINE SECONDS while roll alone
                          #   chased a walking bearing - the stern-chase that seeded every gate-2
                          #   failure. The nose must track the gate on approach; roll works
                          #   around a CENTERED aim, not a standing offset.
                          #   Inside the deadband the heading freezes and ROLL does the lateral work:
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
SPRINT_KP_LAT   = 0.50   # roll (rad) per unit of aim offset_x (saturates the cap at |offx| ~ 0.34).
SPRINT_KD_VLAT  = 0.15   # roll (rad) per m/s of DEAD-RECKONED lateral velocity - the damper.
                      #   EASED from 0.20 (run 222247): the chase equilibrium KP*|offx|/KD was
                      #   ~1 m/s at offx 0.4 - too slow to close a walking bearing (the 3 s
                      #   stern-chase into gate 2). 0.15 keeps zeta ~0.74 at 5 m range.
                      #   Replaces the vision aim-rate damper (run 220336): those samples are
                      #   discarded on any fast vertical aim motion, which in FLOW is always, so
                      #   at transit speed the roll loop was structurally undamped-P - it chased
                      #   the launch-parallax phantom for 2.3 s, built ~1.3 m/s sideways, and hit
                      #   gate 1's right post. Sized for ~critical damping at 5 m range
                      #   (zeta = g*KD/(2*sqrt(g*KP/rng))); also bounds any phantom chase to an
                      #   equilibrium drift of KP*|offx|/KD (~0.3 m/s for the 0.13 launch bias).
SPRINT_KD_LAT   = 1.10   # RETIRED with the vision-rate damper (see SPRINT_KD_VLAT).
SPRINT_OXRATE_MAX  = 0.30  # clamp on the measured lateral rate (LOGGING-ONLY since the v_lat damper).
SPRINT_OYRATE_GATE = 0.25  # when the aim is moving VERTICALLY faster than this (unit/s - climb or
                        #   descent transient), the lateral-rate sample is discarded entirely: fast
                        #   vertical apparent motion pollutes the x-rate with parallax. (The sample
                        #   is LOGGING-ONLY now - this gate is why it could never damp the roll in
                        #   FLOW, and why the damper moved to dead-reckoned v_lat.)
SPRINT_ROLL_BLIND  = 0.06  # roll cap (rad ~= 3.4 deg) for the COMMIT lateral escape clause only -
                        #   the servo-branch blind downgrade is retired (v_lat damping is never
                        #   blind, so the full ROLL_MAX + real damping applies everywhere).
SPRINT_CLIMB_GATE  = 0.25  # row error beyond which the forward creep STOPS (the vertical twin of the
                        #   lateral ALIGN rule): a big climb is flown IN PLACE, at gate height first,
                        #   THEN advance. Creeping forward during the 2.5 s climb is what turned a
                        #   small drift into "already past the gate" (run 105441, t=41.6-44).
SPRINT_ROLL_MAX = 0.17   # bank cap (rad ~= 10 deg) -> ~1.7 m/s^2 lateral. Assertive, still recoverable
                      #   within the vision lag envelope.
SPRINT_ROLL_SLEW = 0.60  # max roll-command slew (rad/s) - the bang AND the counter must both develop
                      #   fast enough to matter (0.30 took 0.57 s just to reach the old cap).
SPRINT_YAW_STEP_MAX = 0.03   # cap per frame (~52 deg/s at 30 fps)
SPRINT_YAW_MAX_OFF  = 0.95   # anti-windup: stop turning once the aim point is AT the frame edge.
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
SPRINT_NEXT_MIN_AREA = 0.004 # candidate floor for "that's the next gate, note its bearing"
SPRINT_NEXT_HOLDOFF  = 1.5   # NO memory writes for this long after a pass: those frames are full of
                          #   the receding gate's fragments sliding off-frame past the single-blob
                          #   avoid zone. Run 124505: 0.19 s after pass 9, a fragment on the RIGHT
                          #   overwrote a correct LEFT memory (+69.8 -> -15.5) and the tilted-gate
                          #   corner turned into a 300-degree wrong-way orbit. The pre-pass memory
                          #   rides through the holdoff and still points the right way.
SPRINT_NEXT_MEM_S    = 6.0   # memory freshness at pass time - older sightings only pick the direction
SPRINT_SEEK_S        = 12.0  # post-pass window in which blind = SEEK (rotate), not plain HOLD
SPRINT_SEEK_RATE     = 0.5   # SEEK rotation rate (rad/s ~= 29 deg/s) - a deliberate pan, FoV does the
                          #   finding; vision interrupts the sweep the moment a gate shows
# The next-gate memory is a SIGHTING TRACK, not a bearing: WHERE (absolute heading), HOW HIGH
# (vertical offset RELATIVE to the then-tracked gate - attitude/altitude invariant, Brian's
# formulation: "it was at the same offset as the one we were tracking"), and HOW BIG. During the
# post-pass window, acquisition must satisfy ALL THREE or the candidate is refused and the sweep
# keeps turning. This corner (post-slanted) is the acid test: elevated exit, course doubling back,
# cross-floor gates littering the cone - bearing alone kept accepting the wrong one.
SPRINT_SEEK_CONE     = 0.45  # bearing tolerance (rad ~= 26 deg) around the remembered heading
SPRINT_SEEK_OY_TOL   = 0.40  # vertical tolerance: candidate offset_y vs (current row target + the
                          #   remembered relative elevation). Rejects the down-low cross-floor gate
                          #   that a bearing-only cone accepted.
SPRINT_SEEK_AREA_RATIO = 0.25 # candidate must be at least this fraction of the remembered size - we
                          #   are CLOSER now than when it was sighted; a distant speck is not it.
SPRINT_PREEMPT_RATIO = 1.6   # YOUNG-LOCK PREEMPTION (post-pass window only, fail-open): a candidate
                          #   this many times BIGGER than the current lock steals it. The corner
                          #   sweep meets gates in the wrong order - the far N+2 enters frame before
                          #   the close true-next N+1 and got locked first (descending for it); N+1
                          #   is ~half the distance, so ~4x bigger the moment it appears. Switch,
                          #   never refuse. Straight handoffs unaffected (their lock IS the biggest).
SPRINT_POSTPASS_GENTLE_S = 4.0 # for this long after a pass, descent authority is capped - a wrong
                          #   mid-corner lock cannot spend much altitude before preemption fixes it
SPRINT_POSTPASS_DN   = 0.05  # the capped below-hover thrust band during that window (~0.5 m/s descent)
SPRINT_POSTPASS_UP   = 0.06  # ...and the capped ABOVE-hover band: a surviving junk lock fired FULL
                          #   climb (thr 0.45) at the gate plane 0.2 s after a pass and popped the
                          #   drone into the INSIDE of the top bar on exit (run 164646, t=53.0)
SPRINT_JUNK_OY       = -0.5  # post-pass sanity: a candidate higher in frame than this (~50 deg above
                          #   the horizon) cannot be a course gate - it is the just-passed gate's
                          #   own top bar or ceiling junk. Excluded from acquisition/preemption
                          #   during the post-pass window only; fail-open if it would empty the list.
SPRINT_SEEK_SINK     = 0.4   # during SEEK, sink at this rate (m/s) back toward PASS altitude when more
                          #   than SPRINT_SEEK_ALT_TOL above it. The pass just happened THROUGH a gate,
                          #   so altitude-at-pass is a trustworthy gate-line reference even with
                          #   integrator drift. Run 115646: a junk post-pass lock climbed it to 3 m,
                          #   SEEK held 3 m faithfully, and from there the NEAR next gate is below
                          #   the camera's 9-deg down-limit - it locked a FAR gate and flew over the
                          #   one it should have taken.
SPRINT_SEEK_ALT_TOL  = 0.4   # dead-band (m) above pass altitude before the SEEK sink engages
SPRINT_LOCK_CONFIRM  = 4     # a fresh lock gets NO vertical authority until it survives this many
                          #   consecutive frames - the junk blob that caused the 3 m climb lived for
                          #   ~2. Real gates confirm in 4 frames (~0.13 s) and lose nothing.

# =============================================================================================
# PITCH - a small constant forward creep, faded while off-aim. ~0.05 rad => ~1 m/s.
# The 8-min cap over a ~140 m course needs only ~0.4 m/s average - never rush a gate.
# =============================================================================================
SPRINT_PITCH_FWD  = 0.05   # forward lean (rad) when aimed
SPRINT_AIM_REF    = 0.40   # creep fades to 0 as |offset_x| approaches this (turn first, then close)
SPRINT_PITCH_SLEW = 0.15   # max pitch-command slew (rad/s) - eases the creep in/out, no lurches
SPRINT_HOLD_KP = 0.15      # STATION-HOLD gain for near-gate climbs (hover_pilot's proven range-hold):
                        #   pitch per unit of range error (range ~ 1/sqrt(area)), anchored to the
                        #   range where the climb began. A CONSTANT back-pitch brake there kept
                        #   accelerating backward over the multi-second climb (backed away, clipped
                        #   the top, no pass) - a servo brakes only as much as needed.
SPRINT_ALIGN_BRAKE = 0.035 # BACK-pitch (rad) held during ALIGN - zeroing the creep does NOT stop the
                        #   momentum already carried (run 111346: coasted from area 0.20 to 0.57 -
                        #   inside the gate's throat - before alignment finished, then clipped the
                        #   top bar entering banked and climbing). A gentle backlean actually
                        #   arrests, so "stop and slide onto the axis" happens AT the commit
                        #   boundary, with the whole gate still ahead. (hover_pilot's proven brake.)

# =============================================================================================
# COMMIT + PASS - near the gate the aim point balloons; stop chasing it and fly straight through.
# Pass = tracked blob peaked then receded (proven across every pilot in this repo).
# =============================================================================================
SPRINT_ALIGN_AREA  = 0.10  # blob fraction where a MISALIGNED approach starts braking + strafing (~4-5 m
                        #   out). Waiting until COMMIT range left no room: run 113145 entered ALIGN
                        #   ~2 m from gate 3 carrying speed and 0.3 of offset, and scraped down the
                        #   gate's side pillar (the "invisible obstacle" - a point-blank pillar is
                        #   just a featureless orange wall on camera). Aligned approaches ignore
                        #   this band and cruise straight through to commit.
SPRINT_COMMIT_AREA = 0.20  # blob fraction where the gate is NEAR: if ALIGNED we commit
SPRINT_COMMIT_ALIGN = 0.15 # |aim offset_x| required to actually COMMIT. Run 095624 committed on area
                        #   alone with the aim 0.28 LEFT of centre, slewed the mid-correction roll
                        #   (-7.5 deg) back to level, and coasted wide right into the gate edge. Near
                        #   but NOT aligned = the ALIGN regime: forward creep OFF (this pilot can
                        #   simply STOP), roll + row servos stay on - slide onto the axis, THEN go.
SPRINT_COMMIT_VZ   = 0.7   # max |vertical speed| (m/s) to ENTER commit. Run 132724 committed while
                        #   still descending -0.97 off the cruise glide and sagged into the bottom
                        #   bar; every healthy commit entered under 0.5. While hot, the ALIGN brake
                        #   holds station until the vertical settles - then commit, level.
SPRINT_COMMIT_ROW  = 0.35  # VERTICAL twin of the alignment requirement: |aim_oy - row target| must be
                        #   inside this to commit. Run 122735: the big-climb gate COMMITTED at
                        #   aim_oy -0.36 with vz +2.97 (mid-climb, hole far above centre); the
                        #   commit arrest then chopped the climb and it sagged into the bottom bar.
                        #   Healthy commits run err < 0.15; blocked ones climb-in-place (existing
                        #   climb-first branch) until level with the line, THEN commit.
SPRINT_PASS_AREA   = 0.30  # arms a pass - ONLY while committed (aligned + close). Area receding from a
                        #   lateral scrape used to count as a "pass" (run 095624 counted 6).
SPRINT_PASS_DROP   = 0.7   # counted when area falls to this * peak
SPRINT_PASS_REARM  = 0.15  # refractory: no re-arm until the passed gate shrinks below this

# =============================================================================================
# TRACKING - follow ONE gate across frames (v1's tracker, kept: the data showed it working).
# Acquisition prefers detections with a LOCATED OPENING - the az+40 station-sign decoy that stole
# two flights never has one.
# =============================================================================================
SPRINT_MIN_AREA     = 0.002  # ignore smaller blobs (next gate after a pass reads ~0.0025)
SPRINT_TRACK_RADIUS = 0.20   # max in-frame jump to still be the SAME gate
SPRINT_TRACK_MAX_MISS = 5    # frames unmatched before re-acquiring
SPRINT_HOP_DIST     = 0.25   # bigger tracked jump = lock hop -> reseed the smoother
SPRINT_AVOID_RADIUS = 0.30   # post-pass: exclude detections near the flown-through gate
SPRINT_AVOID_MIN_AREA = 0.10 # a detection only KEEPS the avoid lock if it is still BIG - the genuinely
                          #   just-passed gate filled the frame moments ago. Without this the avoid
                          #   marker transferred onto the NEXT gate when it drifted near the stale
                          #   position (run 110222: the pitched gate at 11 m wore the avoid ellipse
                          #   while the tracker locked a 0.007 speck at 22 m and flew at that).
SPRINT_AVOID_S      = 2.5    # hard expiry (s after the pass) - the avoid's whole job is the 1-2 s
                          #   handoff window; a stale marker is only a liability after that.
SPRINT_OFF_ALPHA    = 0.4    # EMA on the aim offsets
SPRINT_MAX_STALE    = 6      # frames without a fresh measurement before flying HOLD (hover, no steer)

# =============================================================================================
# TRANSIT - the sprint layer: bang / cruise / counter-bang between gates. THE ONLY NEW PHYSICS.
# Time-optimal tilt->acceleration is a hard lean to build speed and a symmetric counter-lean to
# kill it (Brian's bang/counter-bang, already proven in roll by the KP/KD strafe). This applies it
# to PITCH on the transits, where the F0 budget says 54% of the lap lives. All gains from sysid:
# drag k=0.0343 (a = k*v^2), attitude lag 96 ms (tab 1), terminal fwd 9.38 m/s. The layer can only
# ADD pitch in the far field while commit-grade aligned; every abort path lands back on steady's
# creep machinery (fail-open by construction - it has no veto over locks or commits).
# =============================================================================================
SPRINT_LEAN        = 0.22   # max forward lean (rad ~12.6 deg, ~2.2 m/s^2). RAISED from 0.15 after
                            #   run 194551: the forward bang was never the failure mode - every
                            #   crash was the nose-UP side - and at 0.15 half the transit went to
                            #   building speed. v_est vs v_meas matched at cruise (drag model
                            #   validated in-flight), so the accel side can carry authority.
SPRINT_V_CRUISE    = 5.0    # transit cruise ceiling (m/s). RAISED from 3.0 (run 194551 matched
                            #   steady's gate times to the second - the caps strangled the law).
                            #   The taper owns arrival regardless of cruise; at 15 m spacing the
                            #   profile is taper-bound most of the way, this cap binds ~14 m out.
# ---- THE DISCRETE BRAKE IS RETIRED (Brian's verdict, 5 flights, 2026-07-27 evening). ----------
# The counter-bang slammed the nose up, the pitch transient tripped the OYRATE gate and blinded
# the roll damping at max speed (the design review's day-one watch item), the drone drifted and
# hit gate frames. Speed is now a CONTINUOUS profile - no brake state exists to slam:
SPRINT_V_PASS      = 3.0    # RETIRED - superseded by the lookahead-priced SPRINT_V_LAND_* law below; kept only so old notes still parse. Brian's doctrine after run
                            #   194551: THE BRAKING HAPPENS AFTER THE GATE, NOT BEFORE. Every
                            #   crash of this campaign lived in the slow loiter in front of the
                            #   plane (stop -> drift/ascend -> hit a bar). At 3 m/s the last 2 m
                            #   take 0.7 s - no time to drift - and the momentum carries the
                            #   drone THROUGH the plane during the blob-collapse handoff instead
                            #   of stalling short of it. Post-pass, the blind backpressure sheds
                            #   the speed on the FAR side of the gate.
SPRINT_V_SETTLED   = 1.5    # "slow" for handoff purposes: below this the creep machinery owns it
SPRINT_R_LAND      = 2.0    # range (m) where the taper bottoms out at V_PASS
SPRINT_COMMIT_LEAN = 0.12   # cap on the commit-window lean (rad ~6.9 deg): commit holds V_PASS
                            #   via the flow servo, but a full bang inside a gate's throat is
                            #   never warranted
SPRINT_TAPER_K     = 0.35   # RETIRED - superseded by the constant-decel profile (SPRINT_A_BRAKE).
                            #   Run 210832's verdict: a LINEAR taper demands its hardest decel at
                            #   the highest speed and eases off as the gate nears - backwards -
                            #   and its onset at 4.5 m/s was 10.6 m out. With v_land dead at the
                            #   1.5 floor the brake bubble swallowed whole transits: vmax never
                            #   passed 4.7, 36-62% of every segment crawled under 2 m/s.
SPRINT_A_BRAKE     = 1.2    # profile deceleration (m/s^2): v_tgt = sqrt(v_land^2 + 2*A*room).
                            #   Sized to what the airframe delivers inside the backlean cap WITH
                            #   drag's help (at 3 m/s: drag 0.31 + 6.9-deg tilt 1.2 = ~1.5 - the
                            #   cap has servo headroom above the feedforward). Onset from 4.5 m/s
                            #   to a priced 2.5 landing: ~2 + 5.8 = ~7.8 m instead of 10.6.
SPRINT_TAPER_BACK  = 0.12   # HARD cap on backlean (rad, 6.9 deg - still gentle: the transient is
                            #   brief at the pitch slew, far too small to blind the roll damping
                            #   the way the retired 11.5-deg brake state did, and the row target
                            #   stays in frame. RAISED from 0.10 with the decel feedforward: the
                            #   cap must clear atan((A_BRAKE - drag)/g) ~ 6.3 deg at low speed or
                            #   the profile can't deliver its own decel and the shed drags past
                            #   R_LAND. This remains the maximum nose-up the transit layer can
                            #   ever command - a slam remains impossible.
SPRINT_ROW_TGT_LAG_S = 0.25 # the row target's pitch term is LAGGED by this (s) to stay in phase
                            #   with the measurement it is compared against (~96 ms attitude lag
                            #   + ~150 ms vision latency). Run 215329: computing oy_tgt from the
                            #   instantaneous pitch command made the target lead the stale aim_oy
                            #   by ~0.3 s at every brake onset - phantom row error +-0.3, thrust
                            #   whipped 0.22<->0.31, vz to +1.1 m/s, a 1 m balloon in front of
                            #   gate 2. In steady state the lag changes nothing.
SPRINT_BRAKE_LOW_OY = 0.15  # BRAKE LOW (Brian's law): while the FLOW profile still has speed to
                            #   shed, hold the aim row this far BELOW the gate line (image units;
                            #   ~0.5-0.7 m at 4-5 m range), fading to zero as v_est reaches
                            #   v_land. Whatever vertical disturbance braking still causes then
                            #   climbs INTO the gate line instead of through it into the top bar
                            #   (run 215329 committed 1.1 m high after the brake balloon).
SPRINT_KP_V        = 0.25   # pitch (rad) per m/s of speed error around the drag-equilibrium
                            #   feedforward; clamped [-TAPER_BACK, LEAN]. RAISED from 0.06 (run
                            #   194551: the weak servo approached its target asymptotically and
                            #   never reached even the 3 m/s cap - pitch sat at 1-4 deg all
                            #   transit). At 0.25, an error over ~0.9 m/s saturates to the full
                            #   lean: the bang is back, but continuous and self-easing.
SPRINT_PITCH_SLEW_T = 0.60  # pitch slew (rad/s) while the transit layer drives (and until the
                            #   command is back inside the creep envelope): the bang AND the counter
                            #   must develop fast - steady's 0.15 would take a full second to reach
                            #   the brake lean, a 4 m coast at speed. Matches the roll-slew logic.
SPRINT_K_DRAG      = 0.0343 # quadratic drag (1/m), sysid tab 2: a = k * v^2
SPRINT_ATT_TAU     = 0.096  # attitude first-order lag (s), sysid tab 1 - the v_est observer runs
                            #   the commanded pitch through this before integrating
SPRINT_V_LEAK_TAU  = 3.0    # v_est leak (s) toward zero, ONLY while CREEP and near-level: at hover
                            #   the k*v^2 model has no low-speed drag and would hold a phantom
                            #   0.5 m/s forever; the leak re-anchors it through every stop-and-align.
                            #   NEVER leaks mid-transit (an under-read v_est brakes LATE - unsafe).
SPRINT_C_RNG       = 1.4    # range proxy: rng_est (m) = C / sqrt(area). Anchored to steady's proven
                            #   climb-hold calibration (area 0.10 ~ 4.5 m). Known to go optimistic
                            #   beyond ~8 m (area is sub-1/r^2 far out) - fine for braking, which
                            #   happens inside the calibrated band; F1 logs it for recalibration.
SPRINT_ENTRY_OFFX  = 0.15   # commit-grade lateral alignment required to open the throttle (same
                            #   bar as SPRINT_COMMIT_ALIGN: sprint only at gates we'd commit to)
SPRINT_ENTRY_ROW   = 0.25   # row-error ceiling to open the throttle (vertical twin; also inside
                            #   the CLIMB_GATE band, so a sprint never coexists with climb-first)
SPRINT_MIN_FRAMES  = 8      # consecutive same-lock frames before sprinting (2x LOCK_CONFIRM: junk
                            #   blobs die in ~2 frames, real gates confirm in 4 - 8 is cheap paranoia
                            #   before pointing 9 m/s of authority at something)
SPRINT_ABORT_OFFX  = 0.30   # aim escape that aborts a sprint into BRAKE (hysteresis vs ENTRY_OFFX)
SPRINT_STALE_ABORT = 2      # stale frames tolerated at speed before shedding it - steady waits 6,
                            #   but 6 frames blind at 4 m/s is nearly a metre of unmeasured coasting
SPRINT_AREA_GROW   = 3.0    # headroom multiplier on the KINEMATIC area growth rate (da/a = 2*v/r)
                            #   for the TRIGGER-AREA limiter. The detector FRAGMENTS the near gate
                            #   (banner/checkerboard breaks the blob): raw area swings +/-40% frame
                            #   to frame with over-read episodes lasting ~0.5 s - runs 190500 and
                            #   191030 BOTH read 0.073->0.105->0.065 at true ~0.05 (~6 m by
                            #   dead-reckoned distance) and fired the brake ~2 m early. No debounce
                            #   survives a 0.5 s episode; physics does: a real approach cannot grow
                            #   area faster than closure allows. Drops are followed instantly (the
                            #   lower envelope tracked truth in both flights). Raw area stays raw
                            #   everywhere else - pass peaks NEED the spikes.
SPRINT_COMMIT_V    = 3.5    # RETIRED - the latch now admits v_land + SPRINT_COMMIT_V_SLACK. Was: RAISED from 1.5 with the
                            #   brake-after-the-gate doctrine: commits are now flown AT V_PASS -
                            #   this gate only rejects arrivals hotter than the taper's promise
                            #   (taper failure / rng_trig lying), where the ALIGN backpressure
                            #   bleeds it first. Vertical readiness (vz gate + row check) unchanged.
SPRINT_THR_FF_MAX  = 0.04   # cap on the 1/cos(pitch) hover feed-forward: a lean scales vertical
                            #   thrust by cos(pitch) and the transit line must not sag (the fighter-
                            #   wing scar, run 121021, lives exactly on the transit line)
# (The brake-window row-target clamp is retired with the brake itself: at the 3.4-deg backlean cap
# the computed row target tops out ~0.71, comfortably in frame, so the normal row servo runs
# through the ENTIRE approach - the vertical never freezes, which is what the active_gate_index
# audit showed was keeping the drone off the gate line.)

# ---- CORNER RUNG (OFF for F1 - its own flight, one mechanism per flight) --------------------
# F0 found 28 s/lap of corner overhead: only 5 s is blind SEEK panning; 23 s is TURNING - tracking
# the next gate but with the creep faded by |offx| while the nose comes around. Both mechanisms
# below attack it with machinery that already exists (the next-gate sighting memory is live DURING
# the commit; the frozen commit heading is doctrine written for a pilot with no speed to lose).
SPRINT_PRETURN     = True   # ON (the racing-line flight, with post-pass priced carry): start the
                            #   turn toward the next gate INSIDE the current gate's throat instead
                            #   of after the blind window - run 220921 spent the whole 1.8 s leg
                            #   to a close gate 2 crawling because every degree of turning waited
                            #   for the pass
SPRINT_PRETURN_RATE = 0.8   # rad/s of commit-window yaw toward the remembered next-gate heading,
                            #   starting once the area has PEAKED in the throat (>= PASS_AREA -
                            #   NOT the armed-by-range path: run 221609 fired 3 m short of the
                            #   plane and swung the nose 27-31 deg before crossing it)
SPRINT_PRETURN_MAX_PRE = 0.30 # cap (rad ~17 deg) on the total PRE-pass sweep: the lean is body-
                            #   frame, so every degree the nose is off the flight path at the
                            #   plane pushes the drone sideways through it (the run 221609 edge
                            #   hits). The rest of the turn belongs to the post-pass SEEK.
SPRINT_SEEK_RATE_FAST = 0.8 # SEEK sweep rate when the corner rung is on (steady pans at 0.5; the
                            #   FoV finds gates 45 deg before the nose, so a brisker pan is cheap)

# =============================================================================================
# LOOKAHEAD-PRICED LANDING SPEED - the pass speed each gate must EARN (Brian's law).
# A flat V_PASS carried 3 m/s into 100-deg hairpins and gate-2 bumps alike. The corner-angle
# histogram of the completion lap (18 gates) is NOT bimodal: 2 straight, 7 gentle (10-20 deg),
# 1 mid, 7 at 35-60, 3 hairpins - so the landing speed is a CONTINUOUS function of the live
# next-gate sighting: v_land = MIN + (MAX-MIN) * (1 - dpsi/PSI_REF - |doy|/DOY_REF), floored.
# Robustness rules from the design review:
#   * sightings from THIS APPROACH only (gate-count keyed) - the remembered-only memory fails
#     DISCRETELY (poisoning/decoys) and may not price speed;
#   * measured bearing UNDERSTATES the turn at the pass (SEEK-era scar), so dpsi is INFLATED;
#   * PSI_REF sized so the 0-20 deg bins (9 gates) price above the floor and 35+ prices AT the
#     floor - widen toward the 35-60 bin on the evidence ladder, not optimism.
# No sighting -> floor = settled speed = steady-grade gate handling. The law can only ADD
# speed above a proven-safe floor; it can never subtract safety.
# RECALIBRATED after run 210832, where the law was a NO-OP - v_land sat at 1.50 for all 2718
# ticks. Two independent kills: (a) the score went negative on every one of 72 sighting writes
# (PSI_REF 0.55 + DOY_REF 0.25 priced even a straight-ahead next gate to the floor - typical
# |doy| is 0.12-0.19 in normalized image units, eating half the score by itself), and (b) the
# 1 s wall-clock liveness failed at most brake onsets anyway (sightings arrive in bursts early
# in the approach; age at brake time was 2-11 s). Liveness now lives in _v_land as a gate-count
# check; the refs below make the intended ladder actually price: straight ~2.6, gentle 15 deg
# ~2.2, 35 deg ~floor, 45+ floor.
SPRINT_V_LAND_MIN  = 1.5    # floor (= V_SETTLED): corners, climbs, blind or stale lookahead
SPRINT_V_LAND_MAX  = 3.0    # ceiling (= the old flat V_PASS); raise only on clean-lap evidence
SPRINT_PSI_INFLATE = 1.4    # counter the systematic understate of the turn measured mid-approach
SPRINT_PSI_REF     = 1.00   # score knee (rad, post-inflation): raw ~41 deg turn -> floor
SPRINT_DOY_REF     = 0.60   # elevation knee: a strongly climbing/descending next leg -> floor
SPRINT_NEXT_LIVE_S = 10.0   # loose wall-clock BACKSTOP only (an approach lasts 5-11 s); the real
                            #   liveness is the gate-count key in _v_land
SPRINT_COMMIT_VLAT = 0.35   # RETIRED after ONE flight (run 223110): the gate blocked a perfectly
                            #   positioned commit on a PHANTOM v_lat (+0.81 integrated from a
                            #   1.7-deg roll command held 4 s - small-angle dead reckoning is
                            #   bias-dominated), and the blocked pilot loitered 2 s in the throat
                            #   and hit the top bar. Lesson twinned with fail-open acquisition:
                            #   an OPEN-LOOP estimate may DAMP (bounded effect) but never GATE
                            #   (unbounded loiter). Drift is handled by the commit drift-kill.
SPRINT_VLAT_LEVEL  = 0.06   # commanded roll (rad, ~3.4 deg) below which v_lat LEAKS: lateral
                            #   rolls live at 1-3 deg where command-vs-actual bias dominates the
                            #   integral (pitch leans are 12+ deg - v_est never had this problem)
SPRINT_VLAT_LEAK_TAU = 2.0  # leak time constant (s) for small-angle v_lat
SPRINT_COMMIT_V_SLACK = 0.7 # commit latch admits v_est up to v_land + this (replaces the flat
                            #   SPRINT_COMMIT_V gate - the taper is already landing AT v_land)
SPRINT_R_ARM       = 1.5    # velocity-aware pass arming: also arm inside this range (m) while
                            #   committed - a fast pass may give the camera only 2-3 frames above
                            #   the area threshold
