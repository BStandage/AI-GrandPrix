# How the drone flies the course, in detail

Same structure as the slides (`docs/how_it_flies.pptx`) and the short version (`docs/HOW_IT_FLIES.md`). Every section starts with the one-line explanation, then the detail, then links into the code where it happens.

Each section opens with an ELI5 line. Acronyms are spelled out the first time they appear. Links are relative to this file and carry a line number; they were checked on 2026-09-19 against the working tree at commit `eeb0f8d`. A line may drift by a few as the code changes, the function names will not.

## Contents

1. [The one-minute version](#1-the-one-minute-version)
2. [The map](#2-the-map)
3. [The levers](#3-the-levers)
4. [The planner: the line](#4-the-planner-the-line)
5. [The planner: the speed profile](#5-the-planner-the-speed-profile)
6. [What the flight controller gives us](#6-what-the-flight-controller-gives-us)
7. [Dead reckoning: position from the accelerometer](#7-dead-reckoning-position-from-the-accelerometer)
8. [Altitude: accelerometer plus barometer](#8-altitude-accelerometer-plus-barometer)
9. [The camera: from pixels to a bearing and a range](#9-the-camera-from-pixels-to-a-bearing-and-a-range)
10. [Which gate is that: association](#10-which-gate-is-that-association)
11. [Applying a fix](#11-applying-a-fix)
12. [Counting crossings and the crossing fix](#12-counting-crossings-and-the-crossing-fix)
13. [Blind stretches](#13-blind-stretches)
14. [The follower: the carrot](#14-the-follower-the-carrot)
15. [The follower: from acceleration to sticks](#15-the-follower-from-acceleration-to-sticks)
16. [The nose and the camera](#16-the-nose-and-the-camera)
17. [The runtime on the Orin, one tick](#17-the-runtime-on-the-orin-one-tick)
18. [The human and MSP override](#18-the-human-and-msp-override)
19. [The sim: what is real and what is a stand-in](#19-the-sim-what-is-real-and-what-is-a-stand-in)
20. [Day 0 and race day](#20-day-0-and-race-day)
21. [The fallback: the seeker](#21-the-fallback-the-seeker)
22. [What has been proven](#22-what-has-been-proven)

---

## 1. The one-minute version

**ELI5.** We know where the gates are. Before the race a computer draws the fastest line through them that our drone and camera can handle. In the air the drone counts its steps with eyes closed, opens its eyes whenever the camera sees the next gate where the map says it must be, and a follower steers it along the line. A human on the radio arms it and can take over.

**In detail.** Two names used throughout. The **FC** is the flight controller, the small board running Betaflight that drives the motors. **MSP** is the MultiWii Serial Protocol, the serial command language the FC speaks; `rc_backend` writes RC (radio control) stick values into it.

Three programs, in order:

| Program | Runs where | Input | Output |
|---|---|---|---|
| Planner (`raceline.ladder`, `raceline.planner`) | laptop, before the race | map + levers | the plan JSON: line, speed, 23 crossings |
| Estimator (`seeker.dr_estimator`) | Orin, 50 Hz | FC attitude, accel, baro; camera blobs; the map | position, velocity, which gate is next |
| Follower (`solvers.follower`, `raceline.rc_backend`) | Orin, 50 Hz | estimate + plan | sticks over MSP |

The diagram: `docs/how_it_flies.png`.

## 2. The map

**ELI5.** The organizers told us exactly where every gate stands. We believe them.

**In detail.** `data/course_map.json` is the organizer's published table converted to metres in a map frame where +y is "north" on their PDF. Each gate has a position, a crossing heading (the direction you fly through it) and a height. Gate 9 is one physical gate crossed twice: south through the top opening at 4.05 m, then a U-turn and north through the low opening at 1.35 m. Code labels are traversal order: `g0` is the organizer's gate 1, `g8-top` and `g8-low` are gate 9, `g9` is gate 10. The start is a dashed line 7.3 m behind gate 1 and is the origin of the sim and of the estimator.

Each gate also carries an `either_direction` flag. It exists because the organizers were asked whether gate 6 may be crossed backwards; the answer on 2026-09-17 was no, the published direction only (what may pass either side of it is the *bypass* on the way round to its entry). So the flag is false on every gate today, and the code that honours it (section 4) is dormant but kept.

**Where.**
- The gate table: [data/course_map.json:14](../data/course_map.json#L14); the start line: [course_map.json:334](../data/course_map.json#L334).
- Loader, crossing sequence for 2 laps, the sim origin: [src/raceline/course.py:41](../src/raceline/course.py#L41) (`load_course`), falling back to the vendored [src/raceline/pq_course.py:232](../src/raceline/pq_course.py#L232) when the sim repo is absent (the Orin).
- The facts as received: [src/PQ_SPECS_INTAKE.md:126](../src/PQ_SPECS_INTAKE.md#L126) (the published track).

## 3. The levers

**ELI5.** One file says how the camera is mounted and how hard the drone may fly. One number k turns all the "how hard" limits together, from timid (0) to race (1).

**In detail.** `config/vehicle_cam35_120.toml` is the race config plus two camera keys. The planner reads five limits from it: `max_tilt_deg` (how far it may lean, which sets lateral acceleration, g times tan of the tilt, where g is one gravity, 9.81 m/s^2), `a_lat_rate_max` (how fast lateral acceleration may change, the attitude slew), `a_lat_margin` (how much of the tilt budget the plan may use in turns), `v_max_mps` (top speed) and `vz_up_max` (climb rate). The ladder interpolates the first four between a floor and the config's values with one scalar k and re-solves. The climb rate is not a lever: above about 1.25 m/s a climbing turn crosses the stacked gate off centre even on ground truth, so it stays capped.

The camera keys (`cam_tilt_deg`, and `cam_hfov_deg`, the horizontal field of view) do two things at config load: the plan's tilt is clamped so that a gate at the drone's own height stays inside the frame (mount + half the vertical field of view - 8 deg margin), and a look window is switched on (section 5). For the 35 deg mount and the 120 deg lens that clamp is 71.3 deg, well under the config's own `max_tilt_deg` of 89, so on the race rung it is the camera, not the airframe, that decides how far the drone may lean. `centred_crossings = true` tells the planner to ignore the hand-searched per-gate pose knobs and cross every gate through its centre.

**Where.**
- The config schema and optional keys, and where the camera clamp is applied at load: [src/raceline/config.py:100](../src/raceline/config.py#L100).
- The centred-crossings switch: [config.py:68](../src/raceline/config.py#L68).
- Ladder floor and the four levers: [src/raceline/ladder.py:64](../src/raceline/ladder.py#L64); the camera tilt cap: [ladder.py:83](../src/raceline/ladder.py#L83) (`camera_tilt_cap`); the interpolation: [ladder.py:88](../src/raceline/ladder.py#L88) (`envelope`).
- The per-gate knobs that centred crossings switch off (they were searched for 29 s on ground truth): [src/raceline/planner.py:182](../src/raceline/planner.py#L182).
- The config itself: the camera keys at [config/vehicle_cam35_120.toml:62](../config/vehicle_cam35_120.toml#L62), the climb cap at [vehicle_cam35_120.toml:189](../config/vehicle_cam35_120.toml#L189), centred crossings at [vehicle_cam35_120.toml:216](../config/vehicle_cam35_120.toml#L216).

## 4. The planner: the line

**ELI5.** Draw a smooth curve through the middle of every gate, in order, that does not clip a frame.

**In detail.** `build_anchors` places anchor points at each crossing (the gate centre, plus short straight stubs before and after so the line goes through square), decides for each pair of gates whether the drone travels straight between them or has to loop (the g4 to g5 hairpin, the stacked gate reversal), and sets heights. `sample_spline` fits a smooth curve through the anchors and resamples it every few centimetres. A frame-contact check rejects any line that passes through a gate frame; the ladder steps k down until contacts are zero.

`plan` wraps that solve in a search over crossing direction: every gate the map marks `either_direction` is solved both ways and the fastest contact-free combination wins. While it was live for gate 6 the published direction won by 43 s, and the map now marks no gate either-direction (section 2), so the search collapses to a single solve. The sim's referee accepts a backwards crossing under the same flag.

**Where.**
- Anchors, stubs, reversal detection: [src/raceline/planner.py:469](../src/raceline/planner.py#L469) (`build_anchors`).
- The spline: [planner.py:1156](../src/raceline/planner.py#L1156) (`sample_spline`).
- The entry point with the direction search, and the centred-crossings wrapper: [planner.py:1859](../src/raceline/planner.py#L1859) (`plan`), [planner.py:1843](../src/raceline/planner.py#L1843) (`centred_crossings`).
- The referee's either-direction rule: [src/raceline/pq_course.py:295](../src/raceline/pq_course.py#L295).

## 5. The planner: the speed profile

**ELI5.** Now decide how fast to be at every point on the line: as fast as the tilt, the turn radius, the motors and the camera allow, braking early enough for every corner.

**In detail.** `_speed_profile` builds speed ceilings per sample (curvature against lateral acceleration, climb and descent rates, a uniform speed cap inside a window around every gate, the run-out after the finish), then runs a forward pass (accelerate as hard as the forward budget allows, minus drag, respecting a friction circle with the lateral acceleration in use) and a backward pass (brake as hard as the brake budget allows). Gravity along the path is included, so a climb costs and a descent pays. The look window, on when a camera is configured, caps the forward budget in the last 8 m before each crossing so the pitch cannot point the camera off the gate on the approach.

**Where.**
- The function: [src/raceline/planner.py:1474](../src/raceline/planner.py#L1474) (`_speed_profile`).
- Gate-window ceiling: [planner.py:1648](../src/raceline/planner.py#L1648).
- Look window: [planner.py:1750](../src/raceline/planner.py#L1750).
- Friction circle and the two passes: [planner.py:1725](../src/raceline/planner.py#L1725), forward at [planner.py:1770](../src/raceline/planner.py#L1770), backward at [planner.py:1778](../src/raceline/planner.py#L1778).
- Blind-turn speed cap (exists, off by default, it made the hairpin worse): `v_blind_turn_mps` in [config.py:76](../src/raceline/config.py#L76).

## 6. What the flight controller gives us

**ELI5.** The flight controller is a little computer that keeps the drone level and knows which way it is tilted and pointing. We ask it 50 times a second.

**In detail.** The Archer runs Betaflight 4.4.3 (its own blackbox says so). We talk MSP version 1 over a UART (universal asynchronous receiver/transmitter, a plain serial port) at 115200 baud. Requested in a loop: `MSP_ATTITUDE` (roll, pitch, heading), `MSP_RAW_IMU` (the raw IMU, the inertial measurement unit: accelerometer and gyroscope, accel at 2048 counts per g), `MSP_ALTITUDE` (barometric altitude and vario), `MSP_STATUS_EX` (armed, active modes, arming blockers), `MSP_BATTERY_STATE`. We send `MSP_SET_RAW_RC` at 50 Hz. Heading is gyro-integrated (no magnetometer in the log), so the runtime reads it on the start line and calls that direction map north. The accelerometer scale is measured at rest before takeoff.

**Where.**
- MSP client, message ids, decoding: [src/hardware/msp.py:46](../src/hardware/msp.py#L46) (`MSP_SET_RAW_RC`), [msp.py:293](../src/hardware/msp.py#L293) (`decode_status`).
- The 50 Hz bridge, stale-command rule: [src/hardware/bridge.py:17](../src/hardware/bridge.py#L17), [bridge.py:89](../src/hardware/bridge.py#L89) (`set_rc`).
- FC state into our frame: [src/hardware/state.py:82](../src/hardware/state.py#L82) (`FcStateSource.estimate`), heading to map yaw: [state.py:75](../src/hardware/state.py#L75).
- Map north from the start line: [src/hardware/runtime.py:212](../src/hardware/runtime.py#L212). Accel scale at rest: [runtime.py:232](../src/hardware/runtime.py#L232).
- Heading drift measurement: [src/hardware/bench.py:172](../src/hardware/bench.py#L172) (`cmd_drift`).
- The blackbox facts: [src/PQ_SPECS_INTAKE.md:34](../src/PQ_SPECS_INTAKE.md#L34).

## 7. Dead reckoning: position from the accelerometer

**ELI5.** Close your eyes and count steps. You know roughly where you are, and the error grows the longer your eyes stay shut.

**In detail.** Every tick the body accelerometer reading is rotated into the world frame with the attitude and gravity is subtracted, leaving the true acceleration. Velocity is the integral of that, position the integral of velocity. A weak decay on velocity (time constant 120 s) keeps a bad bias from running away. The estimator also keeps a half-second history of positions so a camera frame can be compared with where the drone was when the frame was taken, not where it is now. Run on the organizers' real 82 s flight log this integration drifts 0.14 m/s^2, about 0.6 m over a 3 s blind leg.

**Where.**
- The integration: [src/seeker/dr_estimator.py:184](../src/seeker/dr_estimator.py#L184) (`integrate`); gravity removed at [dr_estimator.py:191](../src/seeker/dr_estimator.py#L191).
- Position history for latency: [dr_estimator.py:209](../src/seeker/dr_estimator.py#L209) (`p_at`), the attitude with it: [dr_estimator.py:213](../src/seeker/dr_estimator.py#L213) (`state_at`).
- Fed from the FC on the Orin: [src/hardware/runtime.py:291](../src/hardware/runtime.py#L291).
- Test: [tests/test_dr_estimator.py:35](../tests/test_dr_estimator.py#L35) (`TestIntegration`).

## 8. Altitude: accelerometer plus barometer

**ELI5.** The barometer knows the height but jitters; the accelerometer is smooth but drifts. Blend them and you get both.

**In detail.** A critically damped observer at 3 rad/s: vertical velocity integrates the vertical acceleration, altitude integrates the velocity, and every fresh barometer sample pulls both toward the measurement, with a slow accelerometer bias state. On the ground the accelerometer is ignored (the sim reads free fall while settling on the deck) until the barometer has read above 0.25 m for three samples. On the drone the input is the FC's altitude; in the sim it is the raw sim barometer with 10 cm of noise, four times worse than the real one.

**Where.**
- [src/seeker/dr_estimator.py:40](../src/seeker/dr_estimator.py#L40) (`VerticalFilter`), the update at [dr_estimator.py:57](../src/seeker/dr_estimator.py#L57).
- Test: [tests/test_dr_estimator.py:58](../tests/test_dr_estimator.py#L58) (`TestVerticalFilter`).

## 9. The camera: from pixels to a bearing and a range

**ELI5.** Find the red thing. Where it is in the picture says which way it is; how big it is says how far.

**In detail.** The detector thresholds the frame in HSV for the gate red, finds blobs, and reports for each: the outer box, the opening (hole) box, and offsets of the centre from the image centre normalised to -1..+1. The runtime hands the three biggest blobs to the estimator. Range is the focal length times the gate's real width divided by the blob's pixel width; the width, not the height, because the real gate carries a header board. The offsets and the camera's mount tilt and field of view turn into a unit vector in the body frame. Focal length, field of view and mount tilt are measured on the drone with `camcal` against a real gate at a taped distance.

**Where.**
- Thresholds: [src/perception/detectors/hsv_classic.py:39](../src/perception/detectors/hsv_classic.py#L39), the mask: [hsv_classic.py:45](../src/perception/detectors/hsv_classic.py#L45).
- Blobs to boxes and offsets: [src/perception/gate_detection.py:71](../src/perception/gate_detection.py#L71) (`mask_to_detections`).
- Camera thread on the Orin, top three blobs, range from width: [src/hardware/runtime.py:50](../src/hardware/runtime.py#L50) (`CameraThread`), [runtime.py:111](../src/hardware/runtime.py#L111).
- Offsets to a body direction: [src/seeker/synthetic_camera.py:68](../src/seeker/synthetic_camera.py#L68) (`direction_body`).
- Measuring the camera: [src/hardware/camcal.py:92](../src/hardware/camcal.py#L92).
- Judging a detector on recorded video, including the width jitter that becomes range error: [src/perception/video_probe.py:113](../src/perception/video_probe.py#L113).

## 10. Which gate is that: association

**ELI5.** We know which gate should be in front of us and where it should appear. If the red thing is there, it is the gate. If not, ignore it.

**In detail.** For each allowed gate the estimator predicts its direction from the current position estimate and the attitude, and its range. A blob matches a gate when the angle between the observed and predicted directions is inside a tolerance that grows with time since the last fix (1.5 m plus 0.5 m per second, over the predicted range, clamped to 6 to 40 deg), and when its measured range fits the predicted one within 1 m plus 25 percent. Gates beyond 15 m are never candidates. If two gates match about equally the blob is refused. Only the gate just passed and the next one are allowed at all, because the map and the crossing count say nothing else can be the one in view. Among several blobs the best-scoring match wins, so the biggest blob need not be the gate.

**Where.**
- Allowed gates from the crossing count: [src/seeker/dr_estimator.py:135](../src/seeker/dr_estimator.py#L135) (`set_landmarks`), [dr_estimator.py:144](../src/seeker/dr_estimator.py#L144) (`allowed_now`).
- The scoring: [dr_estimator.py:253](../src/seeker/dr_estimator.py#L253) (`associate_scored`); tolerance at [dr_estimator.py:278](../src/seeker/dr_estimator.py#L278); range gate at [dr_estimator.py:287](../src/seeker/dr_estimator.py#L287); the ambiguity refusal at [dr_estimator.py:296](../src/seeker/dr_estimator.py#L296); the 15 m limit at [dr_estimator.py:126](../src/seeker/dr_estimator.py#L126).
- Several blobs: [dr_estimator.py:302](../src/seeker/dr_estimator.py#L302) (`observe_any`).
- Called on the Orin: [src/hardware/runtime.py:297](../src/hardware/runtime.py#L297); in the sim: [src/solvers/follower.py:537](../src/solvers/follower.py#L537).
- Tests: [tests/test_dr_estimator.py:97](../tests/test_dr_estimator.py#L97) (`TestAssociation`), [test_dr_estimator.py:145](../tests/test_dr_estimator.py#L145) (`TestSeveralBlobs`), [test_dr_estimator.py:162](../tests/test_dr_estimator.py#L162) (`TestMapPrior`).

## 11. Applying a fix

**ELI5.** Open your eyes, see where the gate really is, and nudge your idea of where you stand. Trust the sideways part of what you see more than the distance part.

**In detail.** The fix position is the gate's map position minus the range along the observed direction. The residual against the estimate at the frame's time is split across and along the line of sight. Across (the bearing, precise to a few centimetres) is applied at gain 0.6, scaled down with range. Along (the range, about 10 percent) is applied at 0.6 times 0.17, and not at all beyond 20 m. The across residual also corrects velocity, capped at 0.3 m/s per fix. A fix that would move the estimate more than 4 m, or whose raw residual is over 8 m, is undone: that is a wrong gate.

**Where.**
- [src/seeker/dr_estimator.py:318](../src/seeker/dr_estimator.py#L318) (`apply_fix`); the range weighting at [dr_estimator.py:342](../src/seeker/dr_estimator.py#L342); the gains themselves at [dr_estimator.py:84](../src/seeker/dr_estimator.py#L84).
- The undo: [dr_estimator.py:371](../src/seeker/dr_estimator.py#L371) (`observe`), the rejection test at [dr_estimator.py:382](../src/seeker/dr_estimator.py#L382).
- Test: [tests/test_dr_estimator.py:179](../tests/test_dr_estimator.py#L179) (`TestFixes`).

## 12. Counting crossings and the crossing fix

**ELI5.** Nobody on the drone tells it that it went through a gate. It notices by itself when its own position crosses the gate's plane, and at that moment it knows it was inside the opening.

**In detail.** Every tick the estimator checks whether the segment from the previous to the current position crosses the plane of the next expected gate in the right direction. Inside the opening (1.5 m across, 1.2 m vertically) it counts a clean crossing; up to 4 m off centre it still advances the count (a miss, but the gate is behind us; staying on it would circle forever). A fix can carry the estimate across a plane too, so the check also runs after every fix. On a clean crossing the lateral position is pulled 60 percent of the way to the gate centre: the drone was inside the opening, the map says where that is. This anchors every blind turn at its start.

**Where.**
- [src/seeker/dr_estimator.py:153](../src/seeker/dr_estimator.py#L153) (`_count_crossings`); the crossing fix at [dr_estimator.py:178](../src/seeker/dr_estimator.py#L178).
- The follower reads the estimator's count, not the sim referee: [src/solvers/follower.py:552](../src/solvers/follower.py#L552).
- Tests: [tests/test_dr_estimator.py:214](../tests/test_dr_estimator.py#L214) (`TestGateCounting`), [test_dr_estimator.py:232](../tests/test_dr_estimator.py#L232) (`TestCrossingByFix`).

## 13. Blind stretches

**ELI5.** Through the hairpin and at the stacked gate the next gate is not in the picture for a second or two. The drone flies the turn it knows from the map with eyes closed.

**In detail.** Two places per lap: the 125 deg hairpin from g4 into g5, and the stacked gate (top, U-turn with a 2.7 m descent, low). During them no fix arrives. The estimate drifts by the velocity error times the blind time, 0.2 to 0.5 m. Three things keep that survivable: the crossing fix at the start of the turn, the plan's climb rate cap at the stack, and the nose pointing at the next gate so a fix arrives as early as geometry allows. The estimator trace `out/flightlogs/dr_NNN.csv` (sim only) shows the error against truth every 10 ms.

**Where.**
- The trace writer: [src/solvers/follower.py:493](../src/solvers/follower.py#L493) (`_dr_trace`, sim only).
- The climb cap: `vz_up_max` in [config/vehicle_cam35_120.toml:189](../config/vehicle_cam35_120.toml#L189).

## 14. The follower: the carrot

**ELI5.** Find the point on the line that is right beside you, look a little further along it, and steer toward that.

**In detail.** `Tracker.step` first advances its progress along the plan: it projects the estimated velocity onto the path tangent and refines with a local nearest-point search (a global search would jump lanes where laps overlap). Progress may not run past the next uncounted gate. The carrot sits a lookahead ahead (max of a distance and a time times speed). Desired acceleration is the plan's feedforward acceleration at the carrot plus a proportional term on the cross-track offset and a damping term on the velocity error, with caps so a rejoin never demands a lunge. Missed gates get a bounded retry.

**Where.**
- [src/solvers/follower.py:111](../src/solvers/follower.py#L111) (`Tracker`); progress: [follower.py:153](../src/solvers/follower.py#L153) (`_advance`); the step with the feedback law: [follower.py:168](../src/solvers/follower.py#L168), gains at [follower.py:181](../src/solvers/follower.py#L181), lookahead at [follower.py:277](../src/solvers/follower.py#L277).
- Gains and lookaheads: the `[follower]` section of the flown config, [config/vehicle_cam35_120.toml:293](../config/vehicle_cam35_120.toml#L293), commented at length in the base [config/vehicle.toml:288](../config/vehicle.toml#L288).

## 15. The follower: from acceleration to sticks

**ELI5.** To go sideways a drone leans. The follower turns "accelerate this much, this way" into "lean this many degrees", plus a throttle to hold height.

**In detail.** The altitude loop turns the height error and vertical speed into a vertical acceleration command, then a throttle through the measured thrust curve, divided by the cosine of the tilt so leaning does not lose height. A thrust budget gives the vertical need priority and the horizontal what is left. In ANGLE mode (the FC holds whatever lean angle the sticks ask for) the horizontal acceleration becomes roll and pitch angles (atan of a over g, rotated into the body by the yaw) scaled to Betaflight's `angle_limit`; the FC closes the attitude loop itself. In ACRO (rate mode: the sticks ask for turn rates, not angles) a thrust-vector loop turns the same demand into body rates. ACRO is the sim's default, because the attitude estimate of the SITL (software in the loop: the Betaflight firmware compiled to run on a laptop) drifts in ANGLE.

**Where.**
- The pure step, sensor-source agnostic, used by both the sim and the Orin: [src/solvers/follower.py:556](../src/solvers/follower.py#L556) (`step`); the thrust budget comment at [follower.py:593](../src/solvers/follower.py#L593); the stick call at [follower.py:656](../src/solvers/follower.py#L656).
- Altitude loop: [src/raceline/rc_backend.py:61](../src/raceline/rc_backend.py#L61), throttle at [rc_backend.py:100](../src/raceline/rc_backend.py#L100).
- Angle sticks: [rc_backend.py:169](../src/raceline/rc_backend.py#L169) (`angle_sticks`); rate sticks: [rc_backend.py:130](../src/raceline/rc_backend.py#L130) (`attitude_sticks`); yaw: [rc_backend.py:202](../src/raceline/rc_backend.py#L202) (`YawLoop`).
- Test: [tests/test_angle_mode.py:18](../tests/test_angle_mode.py#L18) (`TestAngleSticks`).

## 16. The nose and the camera

**ELI5.** The camera is bolted to the nose, so point the nose at the next gate whenever you are not about to fly through one.

**In detail.** Under dead reckoning the follower replaces the plan's yaw (which follows the path tangent, i.e. away from the gate through a hairpin) with the map azimuth of the next gate, and hands back to the crossing heading 2 m before the gate. Thrust-vector control does not care where the nose points, so this costs nothing in tracking. The handoff distance and the policy are environment knobs.

**Where.**
- [src/solvers/follower.py:563](../src/solvers/follower.py#L563); the policy switch `AIGP_YAW_AT_GATE` at [follower.py:428](../src/solvers/follower.py#L428) and `AIGP_AIM_HANDOFF_M` at [follower.py:429](../src/solvers/follower.py#L429).

## 17. The runtime on the Orin, one tick

**ELI5.** Fifty times a second: read the flight controller, take the newest camera blobs, update where we are, ask the follower for sticks, send them.

**In detail.** `hardware.runtime` opens the MSP bridge and the camera thread, zeroes the barometer, measures the accelerometer scale at rest, reads map north from the start-line heading, and, in the live mode, waits until the FC reports ARMED and MSP OVERRIDE. Then each tick: FC state to attitude and body accel; `integrate`; if blobs are fresh, `observe_any`; `fol.step` with the estimator's own gate count; sticks to the bridge; a CSV row. It stops on a stale link, on the pilot switching override off, or after `--max-s`. `--dry-run` runs all of it with neutral sticks and the FC disarmed.

**Where.**
- [src/hardware/runtime.py:124](../src/hardware/runtime.py#L124) (`main`): the arming gate at [runtime.py:259](../src/hardware/runtime.py#L259), the tick at [runtime.py:291](../src/hardware/runtime.py#L291) to [runtime.py:301](../src/hardware/runtime.py#L301).
- The same dataclasses without the sim repo: [src/hardware/api_shim.py:14](../src/hardware/api_shim.py#L14).
- Rehearsal against the sim's flight controller: `python -m hardware.runtime --tcp 127.0.0.1:5761 ...` (README).

## 18. The human and MSP override

**ELI5.** The radio is the boss. MSP moves the four sticks only; arming, the override switch and the flight mode stay with the pilot.

**In detail.** Betaflight replaces only the receiver channels in `msp_override_channels_mask` while the MSP OVERRIDE mode is on. The qualifier FC ships with 11 (roll, pitch, yaw, no throttle); it must be 15 (the four sticks) and never an AUX (auxiliary, i.e. switch) channel, or the human pilot cannot take control back. The runtime decodes the active modes by name from `MSP_BOXNAMES`, waits for ARM and MSP OVERRIDE, warns if ANGLE is off, and stops when override goes off. Abort is the human pilot's switch.

**Where.**
- Mode decoding: [src/hardware/msp.py:246](../src/hardware/msp.py#L246) (`msp_override`).
- The wait: [src/hardware/runtime.py:259](../src/hardware/runtime.py#L259).
- The procedure, including the one-off setting in Betaflight's CLI (command line interface, its configuration console): [src/PQ_PROCEDURE.md:19](../src/PQ_PROCEDURE.md#L19), day 0 step 4.

## 19. The sim: what is real and what is a stand-in

**ELI5.** The sim is the same drone code with a pretend camera. The pretend camera is made deliberately bad so the real one has to be better, not worse.

**In detail.** The sim (elodin plus a real Betaflight SITL in lockstep) renders no camera frames headless, so a synthetic camera projects every gate through the configured mount and lens from the true pose and returns up to three unlabeled blobs, biggest first, one frame old, with 15 percent dropouts, 0.5 deg offset noise, 10 percent range noise and a 2 percent chance that the biggest blob is junk. The estimator never sees the true pose; the sim's raw barometer (10 cm noise) feeds the altitude filter; attitude comes from the sim's attitude, as it would from the FC. Ground truth is read only to write the `dr_NNN.csv` error trace. The one thing the sim cannot model is the real detector's blob quality, hence `video_probe --still` on real video.

**Where.**
- [src/seeker/synthetic_camera.py:34](../src/seeker/synthetic_camera.py#L34) (`detect`), the noise model at [synthetic_camera.py:85](../src/seeker/synthetic_camera.py#L85), the unlabeled blob list at [synthetic_camera.py:96](../src/seeker/synthetic_camera.py#L96) (`detect_all`).
- The sim wrapper around the same `step`: [src/solvers/follower.py:518](../src/solvers/follower.py#L518) (`autopilot`).
- Flying a plan in the sim: [src/raceline/batch_fly.py:64](../src/raceline/batch_fly.py#L64) (`fly`); the grid: [src/raceline/benchmark.py:59](../src/raceline/benchmark.py#L59).

## 20. Day 0 and race day

**ELI5.** Measure the handful of numbers on the bench, dry-run on the start line, fly the slowest plan first.

**In detail.** In [src/PQ_PROCEDURE.md:13](../src/PQ_PROCEDURE.md#L13): the CLI mask, the pilot's switches, `bench info` / `rc-test` / `arm-test` / `drift`, `camcal` at three distances, the dry run, then [the ladder](../src/PQ_PROCEDURE.md#L90): lowest clean k first, clean twice, jump to the fastest k brought, fail, midpoint.

## 21. The fallback: the seeker

**ELI5.** If the estimator cannot be trusted, a simpler pilot flies gate to gate on the camera alone, slowly.

**In detail.** `seeker.brain` is a state machine (takeoff, seek, track, commit, transit, stop, turn, hold, land) that uses heading, altitude and the detection of the next gate only, with the map giving each leg's heading and distance. Slow by design. As last measured, on 2026-09-16 and before the estimator work in sections 10 to 12: 215 s clean with a perfect detector and true altitude, and 9 of 23 crossings with the noisy one, hitting the low stacked gate. It has not been re-run since, so those two numbers are the oldest on this page. Selected with `--pilot seeker`.

**Where.** [src/seeker/brain.py:124](../src/seeker/brain.py#L124) (`SeekerBrain`), [src/seeker/pilot.py:55](../src/seeker/pilot.py#L55) (`SeekerPilot`), sim adapter [src/solvers/seeker.py:139](../src/solvers/seeker.py#L139).

## 22. What has been proven

Everything below is the sim with the noisy synthetic detector and no ground truth in the loop, and the 35 deg mount with the 120 deg lens, unless it says otherwise.

**The ladder, 2026-09-17, three seeds per rung.** Flown after each camera fix began using the attitude at the frame's own time (a frame one period old at 100 deg/s of yaw was 0.45 m of sideways error at 8 m, every fix through a turn leaning the same way). Raw rows: [docs/benchmark_2026-09-17_fast.csv](benchmark_2026-09-17_fast.csv) and [docs/benchmark_2026-09-17_fast3.csv](benchmark_2026-09-17_fast3.csv).

| k | model time | flown on vision |
|---|---|---|
| 0.5 | 48.7 s | 3 of 3 clean, 47.6 s |
| 0.65 | 43.2 s | 3 of 3 clean, 43.9 s |
| 0.8 | 38.4 s | 3 of 3 clean, 39.9 s |
| 0.9 | 35.3 s | 0 of 3: the line contacts gate 5, every seed stops after 5 crossings |
| 1.0 | 32.2 s | 3 of 3 clean, 35.4 s |

- k = 1.0 is the race config's own limits under the camera's tilt cap (section 3), so the whole envelope this camera allows has now been flown clean on vision. The older claim that the 30 s class of plan does not complete is superseded; the fastest proven rung is 35 s.
- k = 0.9 failing between two clean rungs is why the ladder is flown rung by rung and never interpolated: fly what was flown.
- The five proven rungs travel with the repo, config and plan together: `config/ladder/vehicle_k{050,065,080,100}_cam35_120.toml` and `config/ladder/vehicle_k033_cam20_90.toml`, with their plans under `config/ladder/plans/`. Those are the files the Archer command lines in the README take.

**The mount and lens grid, 2026-09-17, three seeds at k 0.33 and k 0.5**, flown before that fix: [docs/benchmark_2026-09-17.csv](benchmark_2026-09-17.csv). Only two combinations were clean on every seed, 35/120 at k 0.5 (48 s) and 20/90 at k 0.33 (67 s); everything else is 0 to 2 of 3 (45/90 at k 0.5 was 2 of 2, with one seed erroring out). That grid is the argument for the 35 deg mount and the 120 deg lens, and 20/90 at k 0.33 is the rung to fly if a narrower camera is all we have.

**Off the sim.**

- The dead-reckoning method run on the organizers' real 82 s flight log: 0.14 m/s^2 drift, about 0.6 m over a 3 s blind leg.
- The real detector on the Orin recording with the repo thresholds: 11 percent blob-width jitter and occasional merges with the next gate. Width is range, so the target for thresholds tuned on site is under 3 percent.

**Not proven: never flown on the real drone.** No rung has met the real airframe, the real detector, the real barometer or the real gates. The follower gains, the ladder and the camera geometry are sim numbers until day 1.
