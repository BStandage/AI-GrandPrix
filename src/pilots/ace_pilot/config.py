"""ace_pilot - map-based racer. Fly the solved trajectory, correct with vision + IMU.

Flight-1 numbers are deliberately conservative (~2x steady's pace, ~0.25x the theoretical
ceiling): the map may carry a uniform scale error (drag model unvalidated at creep speeds), and
the first flight's job is to measure tracking error, not set a time. The crank order after a
clean lap: V_MAX -> A_LAT -> A_ACC/A_BRK -> LEAN caps.
"""

ACE_MODE         = "tape"  # "tape" = pure open-loop command replay (NO vision anywhere - the
                           #   deterministic-sim strategy: solve offline, replay, iterate the
                           #   per-gate offsets from each attempt's result). "vision" = the
                           #   closed-loop follower (retired from the race path by Brian's call).

# ---- offline solver (trajectory.py) ---------------------------------------------------------
# Measured ceiling (envelope sweep): 45 deg PER AXIS -> the accel envelope is a SQUARE, 9.8
# m/s^2 per body axis, 13.9 on the diagonal; terminal ~29-31 m/s. Flight-1 numbers stay well
# inside it - the unproven part is the ESTIMATOR at speed, not the airframe.
ACE_V_MAX        = 14.0 # cruise ceiling (m/s); measured terminal is ~29-31
ACE_V_FINISH     = 7.5 # carry speed through the finish gate - the clock stops there
ACE_PITCH_UP_MAX = 0.0   # NO NOSE-UP AT ALL (Brian, repeatedly). Feasible now that A_BRK is matched to physics (0.85): the plan no longer demands decelerations that require pitching back, so the follower never needs it. 5 deg was tried and the follower simply used all of it.
                          # tape now share this exact number so the sim verifies what flies
ACE_A_LAT        = 8.0 # max lateral (centripetal) accel (m/s^2); measured ceiling 9.8/axis. 6.0 = 31 deg bank vs the 45 deg clamp. 7.5 was tried and made the profile worse (the limiter was not the binding constraint - a near-stop at s=167 was).
ACE_A_ACC        = 7.5     # max along-track acceleration (m/s^2)
ACE_A_BRK        = 0.85 # max along-track braking (m/s^2). PHYSICS-MATCHED: a quad brakes by pitching UP, and the nose-up cap is ACE_PITCH_UP_MAX (5 deg) = g*tan(5) = 0.86 m/s^2. The old 2.8 demanded 3.3x more deceleration than the pilot can produce, so the follower pinned the nose up, fell behind the profile and recovered with alternating +-30 deg banks - Brian: pitch up and bank, then loop off path.
ACE_GATE_NORMAL_OFF = 0.0 # RETIRED to 0 (flight 2, run 103722): the pre/post waypoint triplets
                          #   put curvature SPIKES at every gate - the speed profile braked to
                          #   0.9-1.9 m/s and the yaw tangent whipped +-60-100 deg between the
                          #   micro-segments, which wrecked the dead-reckoned heading, which
                          #   made the anchor filter reject every real detection (30/s seen,
                          #   0 accepted from t=12 on). Centers-only is smooth; the +-0.61 m
                          #   opening tolerates the entry angles this course's legs produce.
ACE_SAMPLE_DS    = 0.25   # path sampling step (m)
ACE_V_HAIRPIN    = 2.6    # local speed cap (m/s) within 5 m of a course-reversal (>90 deg)
                          #   gate - the global curvature profile under-slows hairpin apexes
ACE_Z_EXEC_TRIM  = 0.0    # EXECUTION altitude trim (m), the one z knob: reality consistently
                          #   flies ~half a gate above the model-flown plan (Brian, attempts
                          #   4/6/7). Applied to the follower's altitude target during tape
                          #   generation; refine from each attempt's altimetry.
ACE_Z_FLOOR      = 0.25    # hard minimum plan altitude (m): the map places gates 4-5 near 0.1
                          #   (leak-era z artifact) and an open-loop tape has no vision to save
                          #   it from the floor; real-attempt feedback corrects per-gate later
ACE_TRANSIT_UP   = 0.6    # cruise-high lift (m) on transits, fading to gate z within ~3 m of
                          #   each gate (steady's proven doctrine; the map z is relative to the
                          #   pad and gates 4-5 sit near 0 - flying the raw spline z dragged
                          #   the floor at 10 m/s in the crash-realistic sim)

# ---- runtime follower (ace_pilot.py) --------------------------------------------------------
ACE_HOVER        = 0.27   # collective at hover (steady/sprint's measured value; trim learns rest)
ACE_LEAN_MAX     = 0.70   # cap on commanded pitch/roll (rad ~40 deg), PER AXIS. The sim clamps
                          #   each axis at 45 deg (envelope sweep sysid_20260728_102241: pitch
                          #   AND roll, 60/75/90 all held 45.0; diagonals hold 45/45 = 54.7 deg
                          #   total tilt). NEVER command past 45: beyond the clamp the held
                          #   attitude is UNPREDICTABLE (cmd pitch+90 held pitch45 + ROLL+44.7 -
                          #   a corner nobody asked for). We clip ourselves at 40, always.
ACE_K_V          = 1.6    # accel (m/s^2) per m/s of velocity error
ACE_A_FF_GAIN    = 0.5    # fraction of the trajectory accel feedforward blended in (1.0
                          #   double-counts the turn the pursuit already encodes)
ACE_K_POS        = 1.5    # velocity (m/s) per m of cross-track position error (blended into
                          #   the pursuit target - bounded by the lookahead geometry)
ACE_LOOKAHEAD_K  = 0.25   # lookahead distance = clamp(K * v, MIN, MAX)
ACE_LOOKAHEAD_MIN = 1.0
ACE_LOOKAHEAD_MAX = 4.0
ACE_A_CMD_MAX    = 12.0    # clamp on commanded horizontal accel (m/s^2; envelope 13.9)
ACE_K_VZ         = 2.4    # vertical accel demand (m/s^2) per m/s of climb-rate error
                          #   (-> ~0.083 thrust per m/s via HOVER/g - steady-grade authority)
ACE_K_Z          = 1.5    # climb-rate target (m/s) per m of altitude error
ACE_VZ_MAX       = 3.5    # cap on commanded climb rate (m/s)
ACE_YAW_SLEW     = 2.5    # max yaw-command slew (rad/s) - the camera must lead the path
ACE_ATT_SLEW     = 3.0    # max pitch/roll command slew (rad/s)
ACE_ATT_TAU      = 0.096  # measured attitude lag (sysid tab 1) - drives the estimator
ACE_YAW_TAU      = 0.15   # yaw-response lag for the HEADING ESTIMATE (the sim tracks yaw
                          #   through a lag like everything else; offline sim showed ~0.3 rad
                          #   of heading error at hairpin yaw rates when treated as instant)
# Two-term drag (Tab 7 ace-envelope, sysid_20260728_095038, odometry ground truth):
# a_drag = C1*v + C2*v^2. Replaces quadratic-only k=0.0343, which under-dragged creep ~1.8x
# (the map stretch) and over-dragged 45-deg flight. Max +-14% over 2-20 m/s.
# Also measured there: the sim CLAMPS attitude at 45 deg (60 commanded -> 45 held), and thrust
# 0.80 sustains +29 m/s climb - the 45-deg clamp, not thrust, is the physics ceiling.
ACE_DRAG_C1      = 0.1141
ACE_DRAG_C2      = 0.0192
# VERTICAL THRUST MODEL (replaces the IMU accel integrator entirely - validated offline against
# Tab 7 odometry truth, analysis/validate_estimator.py + fit, sysid_20260728_110105):
#   vz' = g*(thr*cos(pitch)*cos(roll)/T0V - 1) - (C1V + C2V*|vz|)*vz
# Mean |vz| error 1.09 m/s across 252 s of the FULL envelope (max-thrust climbs, 45/45
# diagonals, knife-edge commands, inversions) vs 13-26 m/s for every IMU-integration variant -
# the IMU integrator's impact-rejection gate throws away real race accelerations and its
# rotation degrades at big tilts. No IMU, no gates, no leak; drift ~0.07 m/s open loop, and the
# vision z-anchor owns the low frequency.
ACE_T0V          = 0.265 # effective hover collective in the vertical response. PER-SIM: the
                          #   Tab 7 fit found 0.235 - that is VQ1's thrust calibration (it is
                          #   why the battery climbed at tilt-comp 0.299). VQ2's measured hover
                          #   is ~0.27 (steady's trim). Flying VQ2 with 0.235 made the model
                          #   read phantom climb at hover -> servo cut thrust -> the real drone
                          #   settled into the FLOOR (the ground-scraping flight). The drag fit
                          #   c1v/c2v transfers; the hover constant must match the sim flown.
ACE_THR_TAU      = 0.03   # motor/thrust response lag (s) modeled in the estimator - without
                          #   MEASURED ~instant (tab7 step fit, tau 0.02) - kept tiny for
                          #   smoothness; run 115245's oscillation was the AIM-POINT bug
ACE_THR_EXP      = 1.45    # MEASURED thrust curve (vq2_probe_124051, hover bracket + brake
                          #   pulses; REFINED against V1 odometry truth, joint fit err 0.22 m/s:
                          #   T0=0.265 p=1.45 c1v=0.35 c2v=0.01) - SUPER-LINEAR, exactly Brian's
                          #   "pitch back -> climb" diagnosis: linear tilt-comp raises
                          #   collective 1/cos but the sim pays back (1/cos)^1.9 of lift
                          #   (+1.6 m/s^2 balloon measured at a 25-deg pulse, both directions)
ACE_GE_GAIN      = 0.05 # GROUND EFFECT: lift multiplier (1+GAIN) at the floor, fading by
                          #   GE_H. Brian's call: ground effect is a pressure cushion that ADDS
                          #   lift -> LESS throttle near the floor. +0.4 was too strong
                          #   (floor-stick), the -0.15 sign flip was backwards (3x high).
                          #   +0.12 = the physically-correct sign at a modest gain.
ACE_GE_H         = 3.0
ACE_C1V          = 0.35   # linear vertical drag (fit)
ACE_C2V          = 0.010  # quadratic vertical drag (fit)
ACE_VZ_TAU       = 45.0   # vertical integrator leak - EFFECTIVELY DISABLED (was steady's 4.0).
                          #   Flight 3 (run 104353): a leaky altimeter is a HIGH-PASS - during
                          #   the sustained course climb it bled the altitude away, the vertical
                          #   servo chased the phantom deficit forever, and the drone flew into
                          #   the CEILING while its own estimate read +1 m. Steady could afford
                          #   the leak because it servos VISION rows, not absolute altitude; a
                          #   trajectory pilot cannot. Low-frequency truth now comes from the
                          #   vision z-anchor instead of a leak.
ACE_LAUNCH_S     = 1.4     # after GO: hold LEVEL at hover+ for this long before the follower
                          #   engages. Flight 3's estimate looped BACKWARD at launch: the
                          #   follower pitched 20 deg while the drone still sat on the pad, and
                          #   the estimator integrated phantom forward motion ground contact
                          #   never allowed. Clean vertical liftoff first, then race.
ACE_SPOOL_S      = 0.4    # motor spool after GO (floor start): commands produce no thrust yet;
                          #   modeled so the estimator does not imagine a climb that has not
                          #   happened (offline: unmodeled spool = floor strike at follower
                          #   handoff)
ACE_LAUNCH_VZ    = 1.2    # climb-rate target (m/s) during the launch hop - SERVOED, not a fixed
                          #   collective (0.40 open-loop rocketed way past gate height, flight 4)

ACE_HOME_RNG     = 8.0    # terminal visual homing engages inside this range (m) of the gate
ACE_K_HOME       = 0.45   # lateral velocity per unit of measured offset_x (scaled by speed)

# ---- vision anchoring -----------------------------------------------------------------------
ACE_ANCHOR_MIN_AREA = 0.003  # detection must be at least this to correct the estimate
ACE_ANCHOR_MAX_AREA = 0.30   # ...and below this (point-blank blobs balloon, geometry lies)
ACE_ANCHOR_OX_TOL   = 0.35   # measured-vs-predicted bearing must agree this well to be OUR gate
                             #   (fail-open lesson does not apply: a wrong anchor CORRUPTS the
                             #   estimate, so anchoring is allowed to be choosy - flying blind
                             #   on the model is the safe default here, unlike acquisition)
ACE_ANCHOR_TOL_GROW = 0.08   # the tolerance WIDENS by this per second since the last accepted
                             #   anchor (anti-starvation, flight 2's death spiral: once the
                             #   estimate drifted, the fixed tolerance rejected every real
                             #   detection forever - choosy must not mean unforgiving; the
                             #   step cap bounds the damage a wrong re-anchor can do)
ACE_ANCHOR_TOL_MAX  = 0.90
ACE_ANCHOR_GAIN_LAT = 0.25   # fraction of the lateral innovation applied per frame
ACE_ANCHOR_GAIN_Z   = 0.30   # fraction of the vertical innovation applied per frame. RAISED
                             #   from 0.15: with the integrator leak disabled, vision OWNS the
                             #   low-frequency altitude - and the map's z column is compressed
                             #   (it inherited steady's leaky altimeter), so the racer must
                             #   servo altitude off the LIVE gate elevation, not the map's z.
ACE_ANCHOR_GAIN_RNG = 0.30   # fraction of the range innovation applied per frame (weakest -
                             #   the area->range proxy is the noisiest measurement)
ACE_VIS_LAT_S       = 0.15   # assumed camera latency (s): anchors judge each frame against the
                             #   estimate at FRAME TIME via the pose history (uncompensated, the
                             #   anchors injected ~5 m of error at speed - offline-sim ablation)
ACE_VSCALE_K        = 0.008  # velocity-scale learning rate per accepted anchor (bounded
                             #   [0.6,1.4]): absorbs VQ1-fit-vs-VQ2-reality dynamics mismatch
ACE_ANCHOR_STEP_MAX = 0.40   # HARD cap (m) on any single anchor correction, per axis per frame.
                             #   Flight 1 (run 103152): an inverted vertical-anchor sign ran the
                             #   altitude estimate to +174 m in ten frames. With the cap, even a
                             #   wrong-signed correction moves <= 12 m/s of estimate - slow
                             #   enough to see in the log and survive.
ACE_C_RNG           = 1.4    # range proxy: rng = C / sqrt(area_frac) (sprint's calibration)

# ---- IMU vertical damper (tape mode) -------------------------------------------------------
# Kills run-to-run vertical drift (measured ~5 cm/s divergence, VQ1 3-flight overlay 20260730)
# by trimming thrust toward the tape's OWN median vertical-velocity profile (vz_ref.json,
# built by analysis/make_vz_ref.py from flown sessions). A run flying the median gets zero
# correction - the tuned trajectory is unchanged by construction. K=0 disables entirely.
ACE_DAMPER_K         = 0.0    # thrust per m/s of vz error (start 0; bracket 0.01-0.04)
ACE_DAMPER_CLAMP     = 0.02   # max |thrust trim| - an order below any tuned tape value
ACE_DAMPER_ROLL_GATE = 0.40   # rad; damper off when |roll_cmd| exceeds this (whips - body
                              # -> world rotation error is worst there, drift is slow anyway)
ACE_DAMPER_TAU       = 4.0    # s; leaky-integrator horizon for the IMU vz estimate
