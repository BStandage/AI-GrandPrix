// The system architecture as it flew on race day, drawn as an SVG so it
// matches the page and stays editable. Source of truth for the boxes:
// src/hardware/runtime.py, src/solvers/follower.py, src/seeker/dr_estimator.py,
// src/perception/, src/raceline/planner.py. Mirrored in docs/how_it_flies.puml.

const W = 1180
const H = 820

function Box({ x, y, w, h, title, lines = [], accent }) {
  return (
    <g transform={`translate(${x} ${y})`}>
      <rect width={w} height={h} rx={12} className={`abox ${accent || ''}`} />
      <text x={14} y={26} className="atitle">
        {title}
      </text>
      {lines.map((l, i) => (
        <text key={i} x={14} y={48 + i * 17} className="aline">
          {l}
        </text>
      ))}
    </g>
  )
}

function Arrow({ d, label, lx, ly, dashed }) {
  return (
    <g>
      <path d={d} className={`aarrow ${dashed ? 'dashed' : ''}`} markerEnd="url(#ahead)" />
      {label && (
        <text x={lx} y={ly} className="alabel">
          {label}
        </text>
      )}
    </g>
  )
}

export default function Architecture() {
  return (
    <figure className="fig arch">
      <svg viewBox={`0 0 ${W} ${H}`} className="archsvg" role="img" aria-label="system architecture">
        <defs>
          <marker id="ahead" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="8" markerHeight="8" orient="auto-start-reverse">
            <path d="M 0 0 L 10 5 L 0 10 z" className="ahead" />
          </marker>
        </defs>

        {/* lanes */}
        <rect x={16} y={16} width={1148} height={160} rx={16} className="alane" />
        <text x={32} y={40} className="alanetitle">
          BEFORE FLIGHT · laptop
        </text>
        <rect x={16} y={196} width={1148} height={440} rx={16} className="alane" />
        <text x={32} y={220} className="alanetitle">
          IN FLIGHT · Jetson Orin, 50 Hz control loop, Python
        </text>
        <rect x={16} y={656} width={1148} height={148} rx={16} className="alane" />
        <text x={32} y={680} className="alanetitle">
          AROUND IT
        </text>

        {/* before flight */}
        <Box x={40} y={56} w={250} h={104} title="Published course map" lines={['10 gates: position, crossing heading,', 'opening height; one stacked gate', '(4.05 m top, 1.35 m low)']} />
        <Box x={330} y={56} w={250} h={104} title="Vehicle config" lines={['camera: tilt, FOV, focal length', 'limits: lean 8°, speed 1.5 m/s,', 'climb rate, lateral margin; ladder k']} />
        <Box x={620} y={56} w={230} h={104} title="Planner" lines={['fastest line through the crossings', 'inside the limits; refuses lines', 'that lose the camera or hit a frame']} accent="red" />
        <Box x={890} y={56} w={250} h={104} title="Plan" lines={['505 samples: x y z, speed, heading', '23 crossings per 2 laps, 251 m', 'frame: start line, +y down gate 0']} />
        <Arrow d="M 290 108 L 328 108" />
        <Arrow d="M 580 108 L 618 108" />
        <Arrow d="M 850 108 L 888 108" />

        {/* in flight: sensors */}
        <Box x={40} y={244} w={260} h={150} title="Flight controller · Betaflight" lines={['MSP serial link, ~32 Hz', 'attitude, heading', 'accelerometer (body frame)', 'barometer', 'ANGLE mode; MSP override on']} />
        <Box x={40} y={430} w={260} h={186} title="Camera · IMX477" lines={['1920×1080 @ 60 fps, GStreamer', 'resized to 1280×720', '', 'HSV detector', 'ring bbox · opening centre', 'offsets (−1..+1) · width range', 'clipped-ring rebuild · commit rule']} accent="amber" />

        {/* estimator */}
        <Box x={360} y={244} w={300} h={210} title="Estimator · vision-aided dead reckoning" lines={['accel rotated by attitude, gravity removed,', 'integrated → velocity, position', 'pad-learned accel bias subtracted', 'gate fix: map gate − range along bearing,', 'strong across the bearing, weak along it', 'counts crossings 0.75 m past the plane', 'heading: FC yaw at arm + drift rate']} accent="red" />

        {/* follower */}
        <Box x={720} y={244} w={420} h={372} title="Follower" lines={[
          'TRACKER  plan point ahead → position/velocity error',
          '  → world acceleration → ANGLE roll/pitch (lean cap 8°)',
          '  last 6 m to an aligned gate: on the gate’s centre line',
          '  yaw: nose on the next gate',
          '',
          'VERTICAL  gate elevation angle (attitude-corrected)',
          '  → slew-limited height reference, 0.09 m/deg',
          '  cap 0.20 m before gate 0, 0.35 m after',
          '  damped on a vertical-speed estimate (see report)',
          '',
          'COMMIT  ring ≥ 85 % of frame ×3, or both edges clip;',
          '  needs: within 6 m, aligned, level',
          '  → no fixes, height frozen, hold throttle,',
          '  gentle steer to the gate line; forced at 3.2 m',
          '',
          'THRUST MODEL  acceleration → throttle µs',
          '  hover measured in flight (D43: 1205 µs)',
        ]} />

        {/* sticks out */}
        <Box x={360} y={500} w={300} h={116} title="Stick commands · 50 Hz" lines={['roll, pitch, yaw, throttle', 'over MSP to the flight controller', 'trace + narration logged every tick']} />

        <Arrow d="M 300 300 L 358 300" label="attitude, accel, baro" lx={200} ly={232} />
        <Arrow d="M 300 500 C 330 500, 330 400, 358 400" label="detections" lx={305} ly={470} />
        <Arrow d="M 660 340 L 718 340" label="state" lx={670} ly={330} />
        <Arrow d="M 1015 160 L 1015 242" label="plan" lx={1024} ly={210} />
        <Arrow d="M 718 560 L 662 560" label="sticks" lx={672} ly={550} />
        <Arrow d="M 358 560 C 320 560, 320 420, 300 394" label="MSP override" lx={210} ly={412} />
        <Arrow d="M 500 454 L 500 498" dashed />

        {/* around it */}
        <Box x={40} y={696} w={340} h={92} title="Pilot · radio" lines={['throttle low → ARM → MSP OVERRIDE → ANGLE', 'abort = MSP OVERRIDE off: instant handback', 'any pilot input ends the scored run']} />
        <Box x={420} y={696} w={360} h={92} title="Simulator · Elodin + Betaflight SITL, Docker" lines={['same estimator and follower code', 'synthetic camera: dropout, noise, latency, false positives', 'referee: crossings, frame contact = run void']} accent="amber" />
        <Box x={820} y={696} w={320} h={92} title="Debrief tools" lines={['pull_flight: log + narration off the aircraft', 'triage: runaway / commit / link checks', 'this page: traces from the same logs']} />
        <Arrow d="M 210 694 L 170 396" dashed />
      </svg>
      <figcaption>
        The race-day architecture. Before flight, the published map and the vehicle config produce a plan on the laptop. In
        flight, the estimator fuses the flight controller's IMU with gate detections against the map, the follower tracks the
        plan and steers height on the gate's elevation, and stick commands go back to the flight controller at 50 Hz. The
        pilot arms and can take back control at any moment; the simulator runs the same estimator and follower code.
      </figcaption>
    </figure>
  )
}
