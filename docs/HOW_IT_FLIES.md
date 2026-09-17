# How the drone flies the course

Written for anyone on the team. No prior knowledge assumed. Diagram: `docs/how_it_flies.png`, slides: `docs/how_it_flies.pptx`, the detailed version with links into the code: `docs/HOW_IT_FLIES_DETAILED.md`.

## The one-paragraph version

The organizers told us where every gate stands. Before the race a planner draws the fastest line through them that our camera and drone can handle, and writes down the speed at every point. In the air the drone has no GPS. It counts its steps with the flight controller's motion sensors, like walking with eyes closed, and every time the camera sees the next gate where the map says it should be, it opens its eyes and corrects. A follower steers the drone along the planned line and sends stick commands to the flight controller fifty times a second. A human on the radio arms it and can take over at any moment.

## 1. Before the race: the map and the plan

**The map.** `data/course_map.json` is the organizer's published table: 10 gates, their positions, which way you fly through each one, and the heights. Gate 9 is a double: through the top, U-turn, back through the bottom. Two laps is 23 crossings.

**The levers.** `config/vehicle_cam35_120.toml` holds two things:

- The camera: mount angle and lens field of view, as measured on the drone with `hardware.camcal`.
- The flight limits: how hard the drone may tilt, how fast it may roll into a turn, how much lateral margin it keeps, its top speed, and how fast it may climb.

**The planner.** `python -m raceline.ladder --k 0.5 --config ../config/vehicle_cam35_120.toml` moves all the limits together between a safe floor (k = 0) and the race values (k = 1), solves the fastest line inside them, and prints the time that comes out. It also refuses to lean the drone so far that the camera would lose the gate.

**The plan.** `out/plans/plan_LADDER_k050.json`: the line as a list of points, the speed at every point, and the 23 crossings in order. Every crossing goes through the centre of its gate.

## 2. In the air: knowing where you are

The flight controller is a Betaflight board. Over a serial link (MSP) it gives us, about 50 times a second:

- attitude: how the drone is tilted
- heading: which way the nose points
- accelerometer: how hard it is being pushed
- barometer: altitude

There is no GPS and no position sensor. So:

**Dead reckoning.** Rotate the accelerometer into the world using the attitude, subtract gravity, integrate twice. That gives velocity and position. It drifts: a few tenths of a metre every few seconds.

**Altitude.** Accelerometer and barometer blended. The barometer alone is noisy, the accelerometer alone drifts, together they are good.

**Fixes from the camera.** The detector finds red blobs. The estimator already knows, from the map and its own position, exactly where the next gate must appear in the image. A blob that sits at that azimuth and looks the right size is the gate; its bearing and apparent size give a position fix. A blob anywhere else is ignored. Only the gate just passed and the next one are ever allowed to match.

**Fixes from crossings.** When the estimator's own position crosses a gate plane, it counts the crossing and pulls its lateral position toward the gate centre, because the drone was inside a 1.5 m opening at that instant.

**Blind stretches.** Through the hairpin and the stacked gate the next gate is out of view for a second or two. The follower flies the known turn from the plan on dead reckoning and the crossing fix at the end resets the error.

## 3. In the air: steering

The follower takes the estimate and the plan:

1. Finds where on the line it is.
2. Aims at a carrot a little ahead on the line.
3. Computes the acceleration needed to get there, plus the plan's own acceleration as feedforward.
4. Turns that into a tilt angle (ANGLE mode: the flight controller levels itself), a throttle for the altitude, and a yaw that keeps the nose, and the camera, on the next gate.
5. Sends the sticks over MSP.

## 4. The human

MSP override covers the four sticks only. The pilot on the radio:

- arms the drone and flips MSP OVERRIDE and ANGLE on,
- can flip override off at any moment and have the sticks back,
- can disarm at any moment.

The runtime waits until the flight controller reports both switches before the plan clock starts.

## 5. Race day

1. Bench: `bench info`, `rc-test`, `arm-test`, `drift`; `camcal` on a real gate; a dry run on the start line.
2. Fly the lowest k that was clean in the sim. Clean twice, jump to the fastest k brought. Fail, fly the midpoint.
3. A complete slow run scores above any incomplete fast one.

## 6. What has been proven and what has not

- Sim, with a noisy synthetic detector and no ground truth: k around 0.4 to 0.5 completes about two runs in three at 47 to 50 s; slower rungs complete nearly always; the 30 s plan does not.
- The estimator's method run on the organizers' real flight log drifts 0.14 m/s^2, the same regime the sim tolerates.
- Never flown on the real drone. The follower gains have never met the real airframe.
