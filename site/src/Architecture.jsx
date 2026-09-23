// The system architecture as it flew on race day, drawn as an SVG so it
// matches the page and stays editable. Source of truth for the boxes:
// src/hardware/runtime.py, src/solvers/follower.py, src/seeker/dr_estimator.py,
// src/perception/, src/raceline/planner.py. Mirrored in docs/how_it_flies.puml.
//
// Layout rules: boxes are sized from their longest line (mono 12 px ≈ 7.3 px
// per character); arrows run in the gaps between boxes, never across one.

const CH = 7.3 // px per character, 12 px JetBrains Mono
const PAD = 16

function Box({ x, y, title, lines = [], accent, minW = 0 }) {
  const longest = Math.max(title.length * 8.6, ...lines.map((l) => l.length * CH))
  const w = Math.max(minW, Math.ceil(longest) + 2 * PAD)
  const h = 44 + lines.length * 17
  return (
    <g transform={`translate(${x} ${y})`}>
      <rect width={w} height={h} rx={12} className={`abox ${accent || ''}`} />
      <text x={PAD} y={26} className="atitle">
        {title}
      </text>
      {lines.map((l, i) => (
        <text key={i} x={PAD} y={48 + i * 17} className="aline">
          {l}
        </text>
      ))}
    </g>
  )
}

function Arrow({ d, label, lx, ly, dashed, anchor }) {
  return (
    <g>
      <path d={d} className={`aarrow ${dashed ? 'dashed' : ''}`} markerEnd="url(#ahead)" />
      {label && (
        <text x={lx} y={ly} className="alabel" textAnchor={anchor || 'start'}>
          {label}
        </text>
      )}
    </g>
  )
}

// Column and row positions. Widths below are the auto-sized widths of each
// box, so the gaps between columns are known and the arrows stay in them.
const W = 1400
const H = 900

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
        <rect x={16} y={16} width={1368} height={174} rx={16} className="alane" />
        <text x={32} y={40} className="alanetitle">
          BEFORE FLIGHT · laptop
        </text>
        <rect x={16} y={210} width={1368} height={470} rx={16} className="alane" />
        <text x={32} y={234} className="alanetitle">
          IN FLIGHT · Jetson Orin, 50 Hz control loop, Python
        </text>
        <rect x={16} y={700} width={1368} height={184} rx={16} className="alane" />
        <text x={32} y={724} className="alanetitle">
          AROUND IT
        </text>

        {/* before flight: four boxes, widths 280 / 300 / 290 / 330 at x 40 / 350 / 680 / 1000 */}
        <Box x={40} y={56} minW={280} title="Published course map" lines={['10 gates: position, crossing', 'heading, opening height; one', 'stacked gate (4.05 m / 1.35 m)']} />
        <Box x={350} y={56} minW={300} title="Vehicle config" lines={['camera: tilt, FOV, focal length', 'limits: lean 8°, speed 1.5 m/s,', 'climb rate, margin; ladder k']} />
        <Box x={680} y={56} minW={290} title="Planner" lines={['fastest line through the', 'crossings inside the limits;', 'refuses lines that lose the', 'camera or touch a frame']} accent="red" />
        <Box x={1000} y={56} minW={330} title="Plan" lines={['505 samples: x y z, speed,', 'heading; 23 crossings / 2 laps', '251 m; frame: start line,', '+y down gate 0']} />
        <Arrow d="M 320 110 L 348 110" />
        <Arrow d="M 650 110 L 678 110" />
        <Arrow d="M 970 110 L 998 110" />

        {/* in flight, left column: FC (x 40..330) and camera (x 40..330) */}
        <Box x={40} y={256} minW={290} title="Flight controller" lines={['Betaflight, MSP serial, ~32 Hz', 'attitude, heading', 'accelerometer (body frame)', 'barometer', 'ANGLE mode, MSP override on']} />
        <Box x={40} y={430} minW={290} title="Camera · IMX477" lines={['1920×1080 @ 60 fps, GStreamer', 'resized to 1280×720', 'HSV detector:', '  ring bbox, opening centre', '  offsets −1..+1, width range', '  clipped-ring rebuild', '  commit rule']} accent="amber" />

        {/* middle column: estimator (x 400..740) */}
        <Box
          x={400}
          y={256}
          minW={340}
          title="Estimator"
          lines={[
            'vision-aided dead reckoning',
            'accel on attitude, gravity removed,',
            '  integrated → velocity, position',
            'pad-learned accel bias subtracted',
            'gate fix: map gate − range along',
            '  bearing; strong across, weak along',
            'crossings counted 0.75 m past plane',
            'heading: FC yaw at arm + drift rate',
          ]}
          accent="red"
        />

        {/* right column: follower (x 800..1360) and sticks (x 800..1360) */}
        <Box
          x={800}
          y={256}
          minW={560}
          title="Follower"
          lines={[
            'TRACKER   plan point ahead → position/velocity',
            '  error → world acceleration → ANGLE roll/pitch',
            '  (lean cap 8°); last 6 m on the gate centre line',
            'VERTICAL  gate elevation (attitude-corrected)',
            '  → height reference 0.09 m/deg, cap 0.20 m',
            '  before gate 0, 0.35 m after; damped on a vz',
            'COMMIT    ring ≥ 85 % ×3 or both edges clip;',
            '  within 6 m, aligned, level → no fixes, height',
            '  frozen, hold throttle; forced at 3.2 m',
            'THRUST    acceleration → throttle µs; hover',
            '  measured in flight (D43: 1205 µs)',
          ]}
        />
        <Box x={800} y={560} minW={560} title="Stick commands · 50 Hz" lines={['roll, pitch, yaw, throttle over MSP', 'trace and narration logged every tick']} />

        {/* arrows, all in the gaps */}
        <Arrow d="M 330 300 L 398 300" label="attitude · accel · baro" lx={364} ly={290} anchor="middle" />
        <Arrow d="M 330 480 L 372 480 L 372 400 L 398 400" label="detections" lx={340} ly={500} />
        <Arrow d="M 740 330 L 798 330" label="state" lx={769} ly={320} anchor="middle" />
        <Arrow d="M 1165 152 L 1165 254" label="plan" lx={1174} ly={210} />
        <Arrow d="M 1080 484 L 1080 558" />
        <Arrow d="M 800 610 L 358 610 L 358 350 L 332 350" label="stick values, MSP override" lx={560} ly={630} />

        {/* around it */}
        <Box x={40} y={740} minW={400} title="Pilot · radio" lines={['throttle low → ARM → MSP OVERRIDE → ANGLE', 'abort: MSP OVERRIDE off, instant handback', 'any pilot input ends the scored run']} />
        <Box x={470} y={740} minW={470} title="Simulator" lines={['Elodin physics + Betaflight SITL, Docker', 'same estimator and follower code', 'synthetic camera: dropout, noise, latency,', 'false positives; referee scores crossings']} accent="amber" />
        <Box x={970} y={740} minW={390} title="Debrief tools" lines={['pull_flight: log + narration off the aircraft', 'triage: runaway, commit and link checks', 'this page: traces drawn from the same logs']} />
      </svg>
      <figcaption>
        The race-day architecture. Before flight, the published map and the vehicle config produce a plan on the laptop. In
        flight, the estimator fuses the flight controller's IMU with gate detections against the map, the follower tracks the
        plan and steers height on the gate's elevation, and stick values go back to the flight controller at 50 Hz. The pilot
        arms over the radio and can take back control at any moment; the simulator runs the same estimator and follower code.
      </figcaption>
    </figure>
  )
}
