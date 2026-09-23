// Every word on the site lives here. Edit this file, not the components.
// Anything that starts with "[BRIAN:" renders as a highlighted note until it
// is replaced. House style: the aircraft are D43, D44 and D45; names are
// introduced once on their cards; "it", never he or she. Report prose.

export const site = {
  title: "AI Grand Prix 2026: Physical Qualifier",
  subtitle: "Technical report: the autonomy stack we built, how it performed on the competition aircraft, and what we learned",
  byline: "by Brian Standage, Team Lead",
  supported: "Supported by: Cristhian Prado, Reese Haven",
  heroImage: "photos/aigp/aigp_header.png",
  thanks: "Our team extends a special thank you to Anduril, Neros, the Drone Champions League, and JobsOhio for an excellent event and incredible support. We would not have been able to do any of this without their teams, who worked non-stop to develop and support the event.",
  tagline:
    "Between June and September 2026 we built a complete autonomy stack for the AI Grand Prix: gate perception, vision-aided state estimation, trajectory planning, and a flight controller interface, developed against a fork of an open-source Elodin simulator with the Betaflight firmware in the loop. The physical qualifier in Costa Mesa was the first time the software ran on the competition aircraft. This report documents each subsystem, the hardware bring-up, fourteen autonomous course flights with their logs, and the analysis of what failed.",
};

// Names that render as bold red links wherever they appear in the text.
export const people_links = {
  "Brian Standage": "https://www.linkedin.com/in/brian-standage-22835912a/",
  "Cristhian Prado": "https://www.linkedin.com/in/pradocristhian/",
  "Reese Haven": "https://www.linkedin.com/in/reese-haven-6a57a1224/",
};

export const stats = [
  { value: "3,300+", label: "teams entered worldwide" },
  { value: "Top 15", label: "virtual qualifier 2 result" },
  { value: "3", label: "months, from nothing" },
  { value: "3.5 of 8", label: "days on site" },
  { value: "3", label: "aircraft" },
  { value: "14", label: "autonomous flights on the course" },
  { value: "100%", label: "gate detection rate on the FPV lap" },
  { value: "1", label: "handshake with Palmer Luckey" },
];

// ---------------------------------------------------------------- the team

export const team = {
  intro:
    "I built the autonomy stack on my own from June. The two virtual qualifiers were flown in the organizers' simulator, with our stack connected to it over MAVLink: I qualified through VQ1, then placed in the top 15 of more than 3,300 teams worldwide in VQ2, which earned an invitation to the physical qualifiers. For the physical qualifier I forked an open-source Elodin drone simulator that runs the Betaflight flight-controller firmware in the loop, and built the published course into it, so the same code could be developed against the real course before it ever flew. My teammates Cristhian Prado and Reese Haven joined me for the physical qualifier at Anduril in Costa Mesa, CA.",
  members: [
    { name: "Brian Standage", role: ["Team Lead", "AI/ML Engineer"], photo: "photos/team/brian.jpeg", blurb: "[BRIAN: a line or two.]", linkedin: "https://www.linkedin.com/in/brian-standage-22835912a/" },
    { name: "Cristhian Prado", role: "AI/ML Engineer", photo: "photos/team/cristhian.jpg", blurb: "[BRIAN: a line or two.]", linkedin: "https://www.linkedin.com/in/pradocristhian/" },
    { name: "Reese Haven", role: "Electrical Engineer", photo: "photos/team/reese.jpg", blurb: "[BRIAN: a line or two.]", linkedin: "https://www.linkedin.com/in/reese-haven-6a57a1224/" },
  ],
  groupPhoto: { src: "photos/team/group.jpg", caption: "[BRIAN: the team at the venue.]" },
};

// ------------------------------------------------------------ the competition

export const aigp = {
  title: "What is the Anduril AI Grand Prix",
  intro: [
    "The AI Grand Prix is an autonomous drone racing competition run by the Drone Champions League with Anduril. Every team flies the same aircraft: a DCL racing quad with a Betaflight flight controller, an NVIDIA Jetson Orin, and a single forward camera. The aircraft has to fly the course by itself on its own sensors; a human pilot arms it and can take over, and any pilot input ends the run.",
  ],
  stagesImage: "photos/aigp/aigp_stages.jpg",
  pqImage: "photos/aigp/pq_aigp.jpg",
  stages: [
    { n: "1", name: "Virtual Qualifier 1", note: "The cut: fly the organizers' course in their simulator, with the team's software connected over MAVLink. Qualified." },
    { n: "2", name: "Virtual Qualifier 2", note: "The ranking round. Our simulator run placed in the top 15 of more than 3,300 teams worldwide and earned the invitation to the physical qualifier." },
    { n: "3", name: "Physical Qualifier", note: "Anduril, Costa Mesa, California, 15 to 22 September. Real aircraft on a real course, timed slots, a referee counting gates. Ten teams advance. This report covers this stage; we were on site for 3.5 of the 8 days." },
    { n: "4", name: "Grand Prix, Ohio", note: "The final, for the ten teams that qualified from the physical qualifier. We did not place in the top 10 and did not advance." },
  ],
};

// ---------------------------------------------------------------- aircraft

export const aircraft = [
  {
    id: "D45",
    name: "Randy the Vanguard",
    art: "d45",
    role: "First autonomous flight of the effort; first damage.",
    story:
      "D45 flew the first autonomous flight on 20 September and climbed to the ceiling: the vertical-speed estimate, derived from a 10 Hz barometer, read zero for the first 0.3 s after launch, so the climb was never braked. The pilot's abort recovered it; the camera fell 2.25 m and was replaced and recalibrated. That evening on the track it flew once with the barometer out of the loop, climbed to 5.1 m while the controller commanded full descent, drifted into the net and broke an arm. Its log identified the thrust-model error described under Control.",
    fate: "Arm repaired. Did not fly on race day.",
  },
  {
    id: "D44",
    name: "Sally the Brave",
    art: "d44",
    role: "Undamaged through the event.",
    story:
      "D44 hovered cleanly on the first flight with the corrected thrust curve. On 21 September it made four course flights in fourteen minutes, the first autonomous gate approaches of the effort, tracking the planned line from 7.4 m to 2.5 m from gate 0 with 0.22 m of cross-track error before striking the top bar. On race day it flew once, unpadded, and did not leave the start line: the start hold did not release.",
    fate: "No damage. [BRIAN: why D44 flew without padding.]",
  },
  {
    id: "D43",
    name: "King Julian",
    art: "d43",
    role: "Arrived last; flew ten of the fourteen course flights.",
    story:
      "D43 arrived without the correct WiFi antennas and was brought up over the serial console. It flew four times the night before race day and six times in the 27-minute slot. Each race-day attempt exposed one defect that the next attempt fixed: accelerometer drift, a hover-throttle constant 23 µs high, a commit on the pad, a start-hold lurch, and a barometer-driven hold. Attempt 6 reached the commit point on the centre line, 0.2 m below the gate centre, and struck the top bar in the blind segment after commit.",
    fate: "Four gate strikes; padded and reflown each time. [BRIAN: the padding: what, who, how long.]",
  },
];

// ---------------------------------------------------- the technical sections
// Each: id, title, blocks. A block is a paragraph string, or
// { list: [...] }, { figure: {src, caption} }, { video: {src, caption} },
// { table: { head: [...], rows: [[...], ...] } }, { h: "subheading" }.

export const report = [
  {
    id: "system",
    title: "System overview",
    blocks: [
      { figure: { src: "photos/system/how_it_flies.png", caption: "Data flow. Before flight: the published map and a vehicle config go into the planner, which writes a plan. In flight: the flight controller's attitude, accelerometer and barometer feed the estimator over MSP; the camera's detections correct it; the follower tracks the plan and sends stick commands at 50 Hz." } },
      "The stack runs on the Jetson Orin in Python. The flight controller is stock Betaflight, flown in ANGLE mode; our process is the pilot, sending roll, pitch, yaw and throttle stick values over the MSP serial link with MSP override enabled. A human on the radio arms the aircraft and can take back control at any moment by switching override off.",
      { list: [
        "Perception: an HSV colour segmentation of the red gate panel, from which the ring, the opening, the image offsets and a width-based range are derived.",
        "State estimation: vision-aided dead reckoning. Accelerometer rotated by the flight controller's attitude, gravity subtracted, integrated; corrected by each gate detection against the published map.",
        "Planning: an offline planner that solves the fastest line through the published gates within the vehicle's limits and writes a time-parameterised plan with 23 crossings per two laps.",
        "Control: a trajectory tracker that follows the plan and a vertical channel that steers on the gate's elevation angle; both output stick values through a thrust model measured from flight data.",
        "Simulation: Elodin physics with the Betaflight SITL firmware in Docker, flying the same follower code against a synthetic camera.",
      ] },
      "Two constraints shaped every design choice. There is no position sensor on the aircraft: no GPS, no rangefinder, no optical flow. And the only link to the flight controller is a 32 Hz serial channel, which limits how the IMU can be used.",
    ],
  },
  {
    id: "perception",
    title: "Perception",
    blocks: [
      "The gate panels are bright red-orange on a grey background, so the detector is classic computer vision rather than a learned model: an HSV threshold, morphological cleanup, and contour analysis at the flight resolution of 1280 by 720.",
      { list: [
        "Segmentation: two HSV ranges, hue 0 to 12 and 169 to 180 with saturation above 85 and value above 75, OR-ed into one mask. The upper hue cap at 12 was tuned to drop the spill from the venue's orange floor markings.",
        "Ring and opening: the largest red contour above 0.015 percent of the frame is the panel; its bounding box is the ring. The opening is located inside it and its centre gives the image offsets, normalised to -1 to +1 in each axis.",
        "Range: from the ring's apparent width against the known 2.7 m outer size and the calibrated focal length. Documented in the code as the weak signal; it is used only to nudge the along-track estimate.",
        "Clipped rings: when the panel runs off the top or bottom of the frame, the vertical centre is rebuilt from the width, since the panel is square. This was added after the first track session, where a 20-degree camera tilt cut off the lower half of every gate and the centroid read high.",
        "Commit rule: when the ring spans 85 percent of the frame height for three consecutive frames, or both side edges clip, the aircraft is inside the range where the elevation can no longer be trusted and the vertical reference is frozen for the crossing.",
      ] },
      { video: { src: "video/hsv_lap.mp4", caption: "The flight detector run over the organizers' FPV lap of the course, resized to the aircraft's 1280 by 720. Green box: the ring. Cross: the opening's centre. Amber line: the offset from the image centre that the controller steers on. The box turns red and the label reads COMMIT where the commit rule fires. Frames without a detection are left unmarked. Over 1,744 frames the detector found a gate on 1,736." } },
      "Measured on that lap: 100 percent detection over 80 sampled frames at 4 Hz, centre within ±0.1 of the frame on every approach, and an automatic switch to the next gate the frame after passing one. The stacked gate reads as one tall blob for six seconds; a stack-aware branch splits it into two ring centres by aspect ratio.",
      { h: "Limitations that mattered" },
      { list: [
        "A centroid is not a pose. The panel has white lettering, logos and checker patterns inside the red that the mask sees as holes, so the blob's centre sits tens of centimetres from the hole's centre, and the offset changes with viewing angle. The opening is about 1.4 m square, so that error is a large fraction of the clearance.",
        "The width-based range fails when the ring clips: it read 12 m at 3 m on the race-day approaches, which forced a plausibility filter and the commit rule.",
        "Neither problem exists for a detector that returns the four corners of the opening, from which a calibrated camera gives a full pose by PnP. That is the first item in the next-steps list.",
      ] },
    ],
  },
  {
    id: "estimation",
    title: "State estimation",
    blocks: [
      "With no position sensor, the estimator is vision-aided dead reckoning, one code path for the simulator and the aircraft.",
      { list: [
        "Prediction: the flight controller's attitude rotates the body-frame accelerometer into the world frame; gravity is subtracted; the result is integrated to velocity and position. An accelerometer bias is learned while the aircraft sits on the pad and subtracted in flight.",
        "Correction: each gate detection is associated with the map gate whose predicted bearing from the current estimate is closest. The fix is the gate's map position minus the detected range along the observed bearing, blended in across the line of sight at the fix gain and along it at a fraction of that, because the range is the least trusted number. A fix that implies an implausible jump is rejected.",
        "Crossings: the estimator counts its own gate crossings from the position track, 0.75 m past the plane, never early, so a missed gate is counted and the run continues.",
        "Heading: no magnetometer. The map frame is fixed to the flight controller's heading at arming, with the aircraft on the start line pointed down gate 0's line, and a per-aircraft drift rate compensated open loop.",
      ] },
      { h: "The vertical channel, and what failed" },
      "Altitude was the problem the entire event. The barometer reads about 1 m high after takeoff and its derived vertical speed carries ±0.4 m/s of noise with the propellers running, so the design steered height on the gate's elevation angle instead: the camera's pitch to the ring centre, corrected for body attitude, drives a slew-limited height reference at 0.09 m per degree, capped at 0.20 m per step before gate 0 and 0.35 m after.",
      "Elevation is a good height signal until the ring clips the frame at about 3.5 m from the gate. From there to the crossing, roughly three seconds, the aircraft holds height on a vertical-speed estimate. Three sources were tried on race day:",
      { table: { head: ["source", "what the logs showed", "outcome"], rows: [
        ["Accelerometer integrated from arming", "reads about 0.35 m/s² low in flight over the 32 Hz link; the integrated speed drifted 0.3 m/s every second", "attempts 1 and 2: climbed into the top bar"],
        ["Barometer-fused speed", "±0.4 m/s of noise; one phantom 0.43 m/s sink reading pushed +1.4 m/s² of throttle", "attempt 6: rose 0.7 m in the blind segment"],
        ["Vision fit of the elevation history", "correct until the ring clips, which is exactly when it is needed", "not usable through the crossing"],
      ] } },
      "The conclusion is structural rather than a tuning matter: the design needs a state that persists through the crossing, which means an EKF over the IMU with gate-pose measurements from the four corners of the opening. Every team that cleared the course was, as far as we could learn, flying some form of that.",
    ],
  },
  {
    id: "planning",
    title: "Planning",
    blocks: [
      "The organizers publish the course as a table: ten gates, positions, crossing directions and opening heights, with one stacked gate flown through the top opening at 4.05 m, reversed, and back through the low opening at 1.35 m. Two laps is 23 crossings.",
      { list: [
        "The vehicle config holds the camera model (mount tilt, field of view, focal length) and the flight limits: maximum lean, roll rate into a turn, lateral margin, top speed and climb rate. All limits are global; nothing is tuned per gate.",
        "The planner solves the fastest line through the crossings inside those limits and refuses any line that would lean the aircraft far enough for the camera to lose the next gate. It writes the line as time-stamped samples with speed and heading at every point, and the 23 crossings in order.",
        "A ladder parameter k scales every limit together between a conservative floor and the race values, so a whole family of plans exists and the flight-day choice is one number. The race-day plan was flown at 1.5 m/s with an 8-degree lean cap.",
        "A collision check refuses plans that contact any gate frame, since a frame contact invalidates the run under the rules.",
      ] },
      "The plan frame is the start line: the aircraft begins at the origin pointed down gate 0's line, 7.3 m out. The course drawing in the flight-testing section is this plan.",
    ],
  },
  {
    id: "control",
    title: "Control and the flight controller interface",
    blocks: [
      "The follower tracks the plan at 50 Hz: a position and velocity error against the moving plan point gives a desired world-frame acceleration, which is converted into ANGLE-mode roll and pitch stick values through the lean limit, with yaw held on the next gate. The last six metres to an aligned gate are flown on the gate's centre line rather than the plan's spline.",
      { h: "The thrust model" },
      "Throttle is computed from a desired vertical acceleration through a thrust curve. The first curve came from the organizers' blackbox logs and was 30 percent high for our airframe, because it was measured per kilogram of a heavier aircraft: commanding 1 g delivered about 1.45 g, and every early flight climbed away. The curve was re-fitted from D45's crash log on 20 September (a constant vertical speed at 1228 µs and 2.79 m/s reached in 0.45 s at 1350 µs), after which D44 hovered on the first attempt.",
      "The hover point itself was the second lesson: 1228 µs measured on D45 was carried in the config for D43, whose real hover is 1205 µs. That 23 µs turned every 'hold' into a slow climb for the first four race-day attempts. Hover throttle has to be measured on the aircraft that flies.",
      { h: "The start and the crossing" },
      { list: [
        "Takeoff: an open-loop punch until airborne, then the height loop on the elevation. Release from the start hold two seconds after the first tick, on time alone; a position-based release lost two attempts.",
        "Commit: inside 6 m by the map, aligned within 35 degrees of the crossing heading and 1.0 m of the gate line, elevation level within 0.12 m, then the size rule. Committed means no position fixes, height reference frozen, a gentle lateral steer toward the gate line, nose on the crossing heading. Forced at 3.2 m regardless.",
        "The hold: the mean throttle of the last four seconds, trimmed by the vertical-speed estimate. This is the piece that depends on a vertical speed the aircraft does not have; see State estimation.",
        "Abort: the pilot switches MSP override off and has the aircraft instantly. It worked every time it was used, and it is why the event cost two cameras and an arm and no airframes.",
      ] },
    ],
  },
  {
    id: "simulation",
    title: "Simulation",
    blocks: [
      "The virtual qualifiers used the organizers' simulator directly. For the physical qualifier we forked an open-source Elodin simulator that runs the Betaflight SITL firmware in the loop, built the published course into it, run in Docker, flying the same follower code that runs on the aircraft against a synthetic camera that returns gate detections with dropout, offset noise, 10 percent range noise, false positives and one frame of latency.",
      { list: [
        "ANGLE mode: the SITL had only ever flown rate mode. Getting it to level in ANGLE mode came down to the attitude quaternion convention the firmware expects from the physics bridge, (w, x, -y, -z), plus a softened rate tune so the mixer did not saturate at hover.",
        "Sensor models: accelerometer noise at the level measured on the aircraft, and a barometer with the takeoff transient and drift seen in flight.",
        "Tooling: a seed sweep that flies N runs and tabulates gates passed, strike locations and commit ranges; a synthetic pad calibration; a referee that scores crossings and freezes on frame contact, matching the rules.",
      ] },
      "In this simulator the race-day build cleared the full course, both laps, and the sweeps were used to fix real problems at the stacked gate and the hairpin. None of that transferred to the vertical channel, because the simulator's camera never clips a ring and its IMU has no aliasing over a serial link. The simulator tested the parts of the stack that did not fail.",
      "Teams that cleared the physical course reportedly built the course in Unity or Gazebo, rendered the panels through the camera model, trained a perception model on the renders, and ran the real perception in the loop. Each failure documented on this page would be visible in that kind of simulation.",
    ],
  },
  {
    id: "hardware",
    title: "Hardware bring-up and troubleshooting",
    blocks: [
      "The aircraft arrived as sealed units with no network configured and no credentials known beyond the defaults. Everything below was done on site, most of it on the first day.",
      { list: [
        "Access: ssh over the Jetson's micro-USB cable would not hold a session. The serial console on the same cable did: PuTTY on the COM port at 115200, log in, then configure WiFi with nmcli. Both aircraft answer on the same USB address with different host keys, so swapping aircraft trips a host-key warning that has to be cleared. D43 arrived without the correct WiFi antennas and was brought up the same way.",
        "Identity: every Jetson reports the same hostname, so each got a shell prompt carrying its number and a paint-pen label on the frame.",
        "Clock: the Jetsons boot believing it is 2023, which corrupts every log filename until NTP is switched on.",
        "UART: the Linux console owns the flight controller's UART by default; a one-time script frees it for MSP.",
        "MSP override: the default mask (11) leaves throttle on the radio and lets MSP write its own override switch; it was set to 15 on every board. ANGLE mode was hard-assigned, since none of the boards had it on a switch.",
        "Camera: IMX477 over GStreamer at 1920 by 1080, 60 fps, resized to 1280 by 720 for the detector. Verified at 100 frames captured before any flight.",
        "Calibration: focal length, field of view, principal point and mount tilt measured per camera with a 25 mm checkerboard, 20 views, RMS 0.165 px on D45. A 5-degree error in mount tilt misplaces a gate by 0.7 m at 8 m, and none of the numbers survive a camera swap; D45's had to be redone after its crash. Betaflight on this firmware reports pitch positive nose-down, the opposite of what the code assumed for months; a tilt check tool now verifies the sign before calibration.",
        "Thrust model: measured from 76,000 airborne samples in a recovered blackbox log, then corrected again from D45's crash log. See Control.",
        "Damage and repair: D45's camera replaced and recalibrated after the ceiling strike; its arm repaired after the net. D43 padded after its first gate strike and reflown four times. [BRIAN: what the padding was and who did it.]",
      ] },
    ],
  },
];

export const attempts = [
  { n: "1", who: "D43", result: "Way high over gate 0", cause: "Accelerometer vertical speed drifted 0.3 m/s every second in flight. The loop's damping fought the elevation and won." },
  { n: "2", who: "D43", result: "High left corner", cause: "Same drift. A size commit fired half a metre low because a clipped ring has no elevation, and the hold latched a climbing throttle." },
  { n: "3", who: "D43", result: "Top bar", cause: "Commanded 'down' for four seconds and never descended. Configured hover 1228, real hover 1205." },
  { n: "4", who: "D43", result: "Blind, high", cause: "Something red 1.8 m ahead on the pad read as gate 0 and committed at t=0. No fixes, no elevation, the whole approach." },
  { n: "5", who: "D43", result: "Height correct, lateral lurch", cause: "Placed 0.9 m from the plan's start point; the start hold rolled hard both ways. Pilot took over at 4 s." },
  { n: "6", who: "D43", result: "Lateral correct, top bar", cause: "0.2 m low at commit. Barometer speed hold read a phantom 0.43 m/s sink, pushed +1.4 m/s², rose 0.7 m in three seconds." },
  { n: "7", who: "D44", result: "Never moved", cause: "Start hold captured its point with three fixes, the estimate moved 0.8 m, held for 20 s drifting backwards." },
];

export const whyWeFailed = [
  {
    title: "No vertical speed the aircraft could trust, and a design that needed one",
    body: "The vertical channel steers on the gate's elevation until the ring clips the frame at about 3.5 m, then holds height blind for three seconds on a vertical speed. The barometer gives ±0.4 m/s of noise. The accelerometer over the serial link reads 0.35 m/s² low under the props. The vision fit is correct until the ring clips, which is exactly when it is needed. None of them survives three seconds. Every attempt that reached commit ended there.",
  },
  {
    title: "A wrong hover throttle for four attempts",
    body: "1228 µs was measured on D45 two days earlier. D43 hovers at 1205 µs. Every 'hold' was a climb and every 'descend' was a hover. A ten-minute hover test would have found it. There was no place to fly one.",
  },
  {
    title: "Blob perception instead of pose",
    body: "The detector returns a centroid and a width. The panel is a wide red square with white lettering and logos that the mask sees as holes, around a 1.4 m opening. The centroid is not the hole's centre, the width-based range reads 12 m at 3 m when the ring clips, and both fail in the last three metres of every approach. Four corners plus a calibrated camera would have given a pose in metres all the way through.",
  },
  {
    title: "No practice, no gate, no bench",
    body: "The start line was ours only inside the slots. There was no gate of our own, no hover flights, no way to fly an approach outside the clock. Everything learned about the aircraft was learned inside 27-minute windows, one fact per flight.",
  },
  {
    title: "Every attempt flew code that had never flown",
    body: "Between attempts the build changed, sometimes by several things at once. Two of seven attempts were lost to new defects that a bench run would have caught. The others each proved one thing and could not prove the next.",
  },
];

export const whatWorked = [
  "The abort. MSP override off gave the pilot the aircraft back instantly, every time.",
  "The detector found the gates on 100 percent of frames of a real lap and switched to the next gate on its own.",
  "The thrust model, once measured from our own flight data. D44 hovered on the first try.",
  "Takeoff and release, the approach height on the elevation, and the lateral onto the gate's line each worked on a real flight in the final slot.",
  "The debrief pipeline: every flight pulled off the aircraft and narrated in two minutes, so each attempt fixed the previous one's cause rather than a guess.",
  "The simulator flies ANGLE mode with the real firmware in the loop.",
];

export const lessons = [
  "Measure the hover throttle on the aircraft that is going to fly. Not a sibling, not a blackbox from someone else's airframe.",
  "A state estimate is not optional. An EKF over the IMU with gate-pose updates is what every team that cleared gates was flying. We built everything downstream of a position we did not have.",
  "Detect the hole, not the red. Corners and a calibrated camera give a pose. A blob gives a bearing and a guess.",
  "The simulator only tests what it models. Real perception in the loop, sensor models from the aircraft's own logs, or it will clear the course and mean nothing.",
  "Build a mock gate. A red panel with a 1.4 m hole in a parking lot would have taught us in an afternoon what the slots taught us in a week.",
  "Never fly code for score that has not flown for practice. One fact per flight is too expensive when there are seven flights.",
  "Bring the right antennas and know the serial-console route before you need it: ssh over the USB cable will not work, PuTTY on the COM port will.",
  "Read the PAD line before arming. The check that would have saved attempt 4 was on the card. It was not the habit.",
];

export const nextSteps = [
  "Corner detection for the opening (a keypoint model trained on rendered panels and the FPV lap), camera intrinsics from a checkerboard, PnP for a metric gate pose every frame.",
  "An EKF over the IMU at the link rate with the gate pose as the measurement, carrying position and velocity through the crossing. Noise parameters from the numbers measured this week.",
  "The course rendered in Gazebo or Unity through the camera model, with the real perception in the loop, so clipping, lettering and lighting show up before a flight.",
  "A mock gate and a measured hover on each aircraft before any flight for score.",
];

export const people = {
  title: "The people",
  body: [
    "[BRIAN: the other teams. Who helped, who lent what, the conversations in the pits.]",
    "[BRIAN: the organizers, the venue, how the slots were run.]",
    "[BRIAN: shaking Palmer Luckey's hand.]",
  ],
};

export const photos = [
  { src: "photos/g0-strike-right.png", caption: "D43 at the right inner edge of gate 0, a third of the way down the opening. The hole is about 1.4 m square inside a 2.7 m panel: half a metre of clearance each side." },
  { src: "photos/g0-strike-top.png", caption: "D43 at the top edge of the opening, near the left corner." },
];

export const links = [
  { label: "The repository", href: "https://github.com/BStandage/AI-GrandPrix" },
  { label: "Debrief", href: "https://github.com/BStandage/AI-GrandPrix/blob/main/docs/DEBRIEF_2026-09-22.md" },
  { label: "Handoff: every flight, every cause", href: "https://github.com/BStandage/AI-GrandPrix/blob/main/docs/HANDOFF_2026-09-22.md" },
  { label: "Track session 2 debrief", href: "https://github.com/BStandage/AI-GrandPrix/blob/main/docs/TRACK_2_DEBRIEF.md" },
  { label: "Camera calibration procedure", href: "https://github.com/BStandage/AI-GrandPrix/blob/main/docs/CAMERA_CALIBRATION.md" },
  { label: "Simulator setup", href: "https://github.com/BStandage/AI-GrandPrix/blob/main/docs/ELODIN_SIM_SETUP.md" },
];
