// Every word on the site lives here. Edit this file, not the components.
// Anything that starts with "[BRIAN:" renders as a highlighted note so you can
// see what still needs your voice. Delete the marker when you replace it.
//
// House style: the aircraft are D43, D44 and D45. Their names are introduced
// once on their cards. They are "it", never he or she.

export const site = {
  title: "Top 15 of 3,300",
  subtitle: "Building an autonomous racing drone from scratch for the AI Grand Prix, June to September 2026",
  tagline:
    "Three months, a small team, a simulator, and three drones we met five days before the race. We built the whole stack ourselves: perception, mapping, planning, control, and a Betaflight simulator that flies the course. We took it to the start line fourteen times. This is the story of what we built, what it did, and what we learned.",
  repo: "https://github.com/BStandage/AI-GrandPrix",
};

export const stats = [
  { value: "3,300", label: "teams entered" },
  { value: "Top 15", label: "where we finished" },
  { value: "3", label: "months, from nothing" },
  { value: "3.5 of 8", label: "days we could be there" },
  { value: "3", label: "drones" },
  { value: "14", label: "autonomous flights on the course" },
  { value: "23 / 23", label: "gates in our own simulator" },
  { value: "1", label: "handshake with Palmer Luckey" },
];

// The team. Photos go in site/public/photos/team/ and are referenced here.
// A missing photo shows a placeholder tile until you add it.
export const team = {
  intro: "I built the autonomy stack and the simulator on my own from June, and qualified in the top 15 of 3,300 in the second virtual round. That is when this became a team: I asked Reese and Cristhian to come to the race, and they did. [BRIAN: one line on where you're from, if you want it.]",
  members: [
    { name: "Brian Standage", role: ["Team Lead", "AI/ML Engineer"], photo: "photos/team/brian.jpg", blurb: "[BRIAN: a line or two.]", linkedin: "https://www.linkedin.com/in/brian-standage-22835912a/" },
    { name: "Cristhian Prado", role: "AI/ML Engineer", photo: "photos/team/cristhian.jpg", blurb: "[BRIAN: a line or two.]" },
    { name: "Reese Haven", role: "Electrical Engineer", photo: "photos/team/reese.jpg", blurb: "[BRIAN: a line or two.]" },
  ],
  groupPhoto: { src: "photos/team/group.jpg", caption: "[BRIAN: the team at the venue.]" },
};

// art: site/src/art/d43.txt etc., shown on the cards
export const aircraft = [
  {
    id: "D45",
    name: "Randy the Vanguard",
    art: "d45",
    role: "First to fly. First to break.",
    story:
      "D45 flew the first autonomous flight of the whole effort on 20 September and went to the ceiling: the speedometer read zero for the first third of a second after launch, the brake never came on, and the pilot took it back. The camera fell 2.25 m. That evening on the track it flew once with the barometer out of the loop, climbed to 5.1 m against a controller asking for full descent, drifted into the net and broke an arm. Its crash log held the finding that fixed the whole team's thrust model.",
    fate: "Arm repaired. Sat out the final day.",
  },
  {
    id: "D44",
    name: "Sally the Brave",
    art: "d44",
    role: "The one that never got hurt.",
    story:
      "D44 hovered cleanly the first time the corrected thrust curve flew. On 21 September it made four course flights in fourteen minutes, the first autonomous gate approaches this team ever flew, tracking the line from 7.4 m to 2.5 m before the top bar. In the final slot it flew once, with no padding at all, and never left the start line: a start hold that would not release.",
    fate: "Never damaged. [BRIAN: why D44 flew without padding.]",
  },
  {
    id: "D43",
    name: "King Julian",
    art: "d43",
    role: "Arrived last, flew the most.",
    story:
      "D43 turned up without the right WiFi antennas and had to be reached over a serial console before it had a network at all. Four flights the night before the slot, none through a gate. Six attempts in the final 27 minutes, each one fixing what the last one found: the accelerometer that lied, the hover number that was 23 microseconds high, the pad commit, the start hold, the barometer's phantom sink. On attempt 6 it arrived on the centre line, 0.2 m low, and rose into the top bar in the last three metres.",
    fate: "Hit the gate four times, padded. Flew again every time. [BRIAN: the antennas, and the padding job: what, who, how long.]",
  },
];

export const timeline = [
  {
    date: "Before",
    title: "June to September: a race stack with no race track",
    body: [
      "We started in June with a simulator, the organizers' course map, and no aircraft. By September the repository had a classic-vision gate detector, a course map and planner, a trajectory tracker, dead reckoning with vision fixes, and a Betaflight software-in-the-loop sim that could fly the whole course.",
      "The bet was sim-to-real: get everything right in simulation, then transfer. The bet was reasonable. It needed the sim to model the aircraft's sensors honestly, and it did not. That sentence is most of this story.",
      "The virtual qualifier came first: our simulator flew the organizers' course well enough to place in the top 15 of 3,300 entries, which earned the trip. [BRIAN: why 3.5 days of the 8: work, travel, cost, whatever it was.]",
    ],
  },
  {
    date: "19 Sep",
    title: "Day one: two drones in boxes and no way in",
    body: [
      "Nobody was logged in to anything. The Jetsons had no WiFi configured, the USB network gadget that was supposed to be the way in dropped constantly, and both aircraft reported the same hostname, so a fix applied to one was tested on the other for an hour before anyone noticed. Three USB ports on the bench looked identical: the flight controller's, the Jetson's, and the wrong laptop's. Betaflight's 'Connect (Virtual)' connected to nothing and looked exactly like success for forty minutes.",
      "The way in turned out to be the serial console on the same USB cable: PuTTY on a COM port at 115200, a login prompt, and from there the WiFi could be configured by hand. Once both aircraft were on the venue network there were ssh shortcuts, shell aliases, and a prompt that said which drone you were on. D43, when it arrived, did not have the right antennas and went through the same serial-console route before it had a network at all. [BRIAN: the antenna detail and how it got sorted.]",
      "With a way in, the day got productive: the MSP override mask fixed on both flight controllers, ANGLE mode assigned, the camera verified at 1080p60, a real gate detected on 100 percent of frames at 6 m, and the thrust model re-measured from 76,000 airborne samples in a recovered blackbox. The config's hover throttle had been 1240, from the simulator. The blackbox said 1291. The first takeoff would have been under-thrusted by a quarter, and it was fixed without flying.",
      "[BRIAN: arrival, the venue, the pits, first impressions.]",
    ],
  },
  {
    date: "20 Sep",
    title: "First autonomous flight, first crash, and the finding of the week",
    body: [
      "D45 flew itself for the first time in a 2 by 2 m cage and went to the ceiling. The speedometer came from a barometer that reports ten times a second, so for a third of a second after launch it read zero, the brake never came on, and the aircraft kept the speed the punch had given it. The pilot's abort worked instantly. The camera did not survive the fall.",
      "That evening, track session one. D45 flew once with the barometer out of the loop, climbed steadily to 5.1 m while the controller asked for full descent, drifted into the net and broke an arm. Its log showed why: the thrust curve had come from the organizers' heavier aircraft. Commanding 1 g on ours delivered about 1.45 g. Every flight all day had been told to climb, and each time an instrument had taken the blame. Once the curve was corrected from that crash, D44 hovered on the first try with no oscillation.",
    ],
  },
  {
    date: "21 Sep",
    title: "The first gate approaches",
    body: [
      "Track session two: fourteen minutes, D44, four flights, no damage. Every flight failed vertically and succeeded laterally, and the four failures had four different causes, each visible only once the previous one was fixed: a camera tilted 20 degrees up that could not see a gate at its own height, a clipped ring with a false centre, an altitude limit cycle, an airborne flag that flickered.",
      "Flight four tracked the line from 7.4 m out to 2.5 m from the gate with 0.22 m of cross-track error and clipped the top bar on the final metre. Camera fixes went from 15 on the first flight to 256 on the last. It was the closest anyone on this team had come.",
      "That night D43 flew four times: hover forever, top bar, and two right-edge strikes. None of it was the controller. All of it was bookkeeping that had never been checked in flight.",
    ],
  },
  {
    date: "22 Sep",
    title: "The slot",
    body: [
      "One 27-minute slot. Two aircraft. The bar to advance was four gates. Seven attempts.",
      "Attempts 1 and 2 lost the vertical to an accelerometer that reads 0.35 m/s² low under the props. Attempt 3 commanded 'down' for four seconds and never descended because the configured hover throttle was 23 microseconds above D43's real hover. Attempt 4 committed to something red on the pad at t=0 and flew gate 0 blind. Attempt 5 had perfect height and a start hold that lurched. Attempt 6 had perfect lateral, arrived 0.2 m low, and a barometer read a phantom sink and pushed the aircraft into the top bar. Attempt 7, D44, never released its start hold.",
      "By attempt 6 every piece had worked on some flight: takeoff, release, approach height, lateral onto the line, the size commit. Never all on one flight, and never the three seconds after commit.",
      "[BRIAN: what it was like on the line, the pilot, the two-person setup, watching each one.]",
    ],
  },
  {
    date: "After",
    title: "Ten teams advanced. We finished fifteenth, and we know the one thing that separates the two.",
    body: [
      "Attempt 6 arrived on the centre line, three and a half metres from gate 0, at the right height. The three seconds after that are the whole gap between us and the teams that went on, and they have a name: a state estimator that carries the aircraft through the crossing. Everything else on this page worked on a real flight.",
      "[BRIAN: the end of the day, the other teams, the handshake, the drive home.]",
    ],
  },
];

export const attempts = [
  { n: "1", who: "D43", result: "Way high over gate 0", cause: "Accelerometer vertical speed drifted 0.3 m/s every second in flight. The loop's damping fought the elevation and won." },
  { n: "2", who: "D43", result: "High left corner", cause: "Same drift. A size commit fired half a metre low because a clipped ring has no elevation, and the hold latched a climbing throttle." },
  { n: "3", who: "D43", result: "Top bar", cause: "Commanded 'down' for four seconds and never descended. Configured hover 1228, real hover 1205." },
  { n: "4", who: "D43", result: "Blind, high", cause: "Something red 1.8 m ahead on the pad read as gate 0 and committed at t=0. No fixes, no elevation, the whole approach." },
  { n: "5", who: "D43", result: "Height perfect, lurch", cause: "Placed 0.9 m from the plan's start point; the start hold rolled hard both ways. Pilot took over at 4 s." },
  { n: "6", who: "D43", result: "Lateral perfect, top bar", cause: "0.2 m low at commit. Barometer speed hold read a phantom 0.43 m/s sink, pushed +1.4 m/s², rose 0.7 m in three seconds." },
  { n: "7", who: "D44", result: "Never moved", cause: "Start hold captured its point with three fixes, the estimate moved 0.8 m, held for 20 s drifting backwards." },
];

export const simStory = {
  title: "The simulator that could fly the whole course and see none of it",
  body: [
    "The day before the slot went into the Betaflight software-in-the-loop sim. It had never flown ANGLE mode, the mode the real aircraft fly. The fix turned out to be a quaternion convention: the SITL wanted the Gazebo plugin's (w, x, -y, -z). After that came a softened rate tune, sensor noise, a barometer model and a seed sweep. It reached 23 of 23 gates, both laps, in 183 seconds.",
    "It could not model accelerometer aliasing over a 32 Hz serial link, the barometer's takeoff transient, a ring clipped by the frame edge, or the airframe's hover point. It found and fixed real problems at the stacked gate and the hairpin. The real failure was the first gate's vertical, and the sim handed the controller a perfect detection every frame, so it never saw it.",
    "The teams that cleared the course most likely had the other kind of sim: the panel rendered through the camera model, a corner-detection model trained on the renders, the real perception in the loop. Every strike in our week would have shown up in that loop in an hour.",
  ],
};

export const whyWeFailed = [
  {
    title: "No vertical speed the aircraft could trust, and a design that needed one",
    body: "The vertical channel steers on the gate's elevation until the ring clips the frame at about 3.5 m, then holds height blind for three seconds on a vertical speed. The barometer gives ±0.4 m/s of noise. The accelerometer over the serial link reads 0.35 m/s² low under the props. The vision fit is correct until the ring clips, which is exactly when it is needed. None of them survives three seconds. Every attempt that reached commit died there.",
  },
  {
    title: "A wrong hover throttle for four attempts",
    body: "1228 was measured on D45 two days earlier. D43 hovers at 1205. Every 'hold' was a climb and every 'descend' was a hover. A ten-minute hover test would have found it. We never had a place to fly one.",
  },
  {
    title: "Blob perception instead of pose",
    body: "Our detector finds the red panel and returns a centroid and a width. The panel is a wide red square with white lettering and logos that the mask sees as holes, around a 1.4 m opening. The centroid is not the hole's centre, the width-based range reads 12 m at 3 m when the ring clips, and both go wrong in the last three metres of every approach. Four corners plus a calibrated camera would have given a pose in metres all the way through.",
  },
  {
    title: "No practice, no gate, no bench",
    body: "The start line was ours only inside the slots. We had no gate of our own, no hover flights, no way to fly an approach outside the clock. Everything we knew about the aircraft, we learned inside 27-minute windows, one fact per flight.",
  },
  {
    title: "Every attempt flew code that had never flown",
    body: "Between attempts the build changed, sometimes by several things at once. Two of seven attempts were lost to brand-new bugs that a bench run would have caught. The others each proved one thing and could not prove the next.",
  },
];

export const whatWorked = [
  "The abort. MSP override off gave the pilot the aircraft back instantly, every time. It is the reason the week cost cameras and an arm and not airframes.",
  "The detector found the gates on 100 percent of frames of a real lap and switched to the next gate on its own.",
  "The thrust model, once measured from our own flight data. D44 hovered on the first try.",
  "Takeoff and release, the approach height on the elevation, and the lateral onto the gate's line each worked on a real flight in the final slot.",
  "The debrief pipeline. Every flight pulled off the aircraft and narrated in two minutes, so each attempt fixed the previous one's cause, not a guess.",
  "The sim flies ANGLE mode now, which nobody had managed before this week.",
];

export const lessons = [
  "Measure the hover throttle on the aircraft that is going to fly. Not a sibling, not a blackbox from someone else's airframe.",
  "A state estimate is not optional. An EKF over the IMU with gate-pose updates is what every team that cleared gates was flying. We built everything downstream of a position we did not have.",
  "Detect the hole, not the red. Corners and a calibrated camera give a pose. A blob gives a bearing and a guess.",
  "The sim only tests what it models. Real perception in the loop, sensor models from the aircraft's own logs, or it will tell you 23 of 23 and mean nothing.",
  "Build a mock gate. A red panel with a 1.4 m hole in a parking lot would have taught us in an afternoon what the slots taught us in a week.",
  "Never fly code for score that has not flown for practice. One fact per flight is too expensive when you have seven flights.",
  "Label the hardware, bring the right antennas, and know the serial-console route before you need it. Day one is lost to whatever you did not check.",
  "Read the PAD line before arming. The check that would have saved attempt 4 was on the card. It was not the habit.",
];

export const people = {
  title: "The people",
  body: [
    "[BRIAN: the other teams. Who helped, who lent what, the conversations in the pits.]",
    "[BRIAN: the organizers, the venue, how the slots were run.]",
    "[BRIAN: shaking Palmer Luckey's hand. What he said, what you said.]",
  ],
};

export const photos = [
  {
    src: "photos/g0-strike-right.png",
    caption: "D43 at the right inner edge of gate 0, a third of the way down the opening. The hole is about 1.4 m square inside a 2.7 m panel: half a metre of clearance each side.",
  },
  {
    src: "photos/g0-strike-top.png",
    caption: "The top edge, near the left corner. Same story from the other axis.",
  },
  // Add more: drop the file in site/public/photos and add { src, caption } here.
];

export const links = [
  { label: "The repository", href: "https://github.com/BStandage/AI-GrandPrix" },
  { label: "Debrief: why we did not pass a gate", href: "https://github.com/BStandage/AI-GrandPrix/blob/main/docs/DEBRIEF_2026-09-22.md" },
  { label: "Handoff: every flight, every cause", href: "https://github.com/BStandage/AI-GrandPrix/blob/main/docs/HANDOFF_2026-09-22.md" },
  { label: "Track session 2 debrief: the first gate approaches", href: "https://github.com/BStandage/AI-GrandPrix/blob/main/docs/TRACK_2_DEBRIEF.md" },
];
