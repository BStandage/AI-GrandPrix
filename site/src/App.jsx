import { useEffect, useState } from 'react'
import * as C from './content.js'
import data from './data.json'
import artD43 from './art/d43.txt?raw'
import artD44 from './art/d44.txt?raw'
import artD45 from './art/d45.txt?raw'
import './App.css'

const ART = { d43: artD43, d44: artD44, d45: artD45 }
const BASE = import.meta.env.BASE_URL
const GATE_Z = 1.35
const TOP_BAR_Z = 1.35 + 0.7 // the opening is ~1.4 m: the bar is ~0.7 m above centre

// ---------- helpers ----------

function Text({ children }) {
  // "[BRIAN: ...]" renders as a visible note until it is replaced
  if (typeof children === 'string' && children.includes('[BRIAN:')) {
    const i = children.indexOf('[BRIAN:')
    const j = children.indexOf(']', i)
    return (
      <>
        {children.slice(0, i)}
        <mark className="todo">{children.slice(i + 1, j)}</mark>
        {children.slice(j + 1)}
      </>
    )
  }
  return <>{children}</>
}

function Paras({ items }) {
  return items.map((p, i) => (
    <p key={i}>
      <Text>{p}</Text>
    </p>
  ))
}

function Photo({ src, alt, className }) {
  // a placeholder tile until the file exists in site/public/photos
  const [ok, setOk] = useState(true)
  return ok ? (
    <img src={BASE + src} alt={alt} className={className} loading="lazy" onError={() => setOk(false)} />
  ) : (
    <div className={`placeholder ${className || ''}`}>
      <span>photo</span>
      <code>{src}</code>
    </div>
  )
}

// ---------- chapter bar ----------

const CHAPTERS = [
  ['team', 'Team'],
  ['aigp', 'The AI-GP'],
  ['aircraft', 'Aircraft'],
  ['system', 'System'],
  ['perception', 'Perception'],
  ['estimation', 'Estimation'],
  ['planning', 'Planning'],
  ['control', 'Control'],
  ['simulation', 'Simulation'],
  ['hardware', 'Hardware'],
  ['testing', 'Flight testing'],
  ['raceday', 'Race day'],
  ['analysis', 'Analysis'],
  ['lessons', 'Lessons'],
  ['people', 'People'],
  ['photos', 'Photos'],
]

function ChapterBar() {
  const [active, setActive] = useState('team')
  useEffect(() => {
    const obs = new IntersectionObserver(
      (entries) => entries.forEach((e) => e.isIntersecting && setActive(e.target.id)),
      { rootMargin: '-30% 0px -60% 0px' },
    )
    CHAPTERS.forEach(([id]) => {
      const el = document.getElementById(id)
      if (el) obs.observe(el)
    })
    return () => obs.disconnect()
  }, [])
  return (
    <nav className="chapters">
      {CHAPTERS.map(([id, label]) => (
        <a key={id} href={`#${id}`} className={active === id ? 'on' : ''}>
          {label}
        </a>
      ))}
    </nav>
  )
}

// ---------- hero ----------

function Hero() {
  return (
    <header className="hero">
      <div className="herotext">
        <p className="kicker">Autonomous drone racing · Anduril · DCL</p>
        <h1>{C.site.title}</h1>
        <p className="sub">{C.site.subtitle}</p>
        <p className="byline">{C.site.byline}</p>
        <p className="lede">{C.site.tagline}</p>
      </div>
      <img className="heroimg" src={BASE + C.site.heroImage} alt="AI Grand Prix 2026" />
      <div className="stats">
        {C.stats.map((s) => (
          <div key={s.label} className="stat">
            <div className="v">{s.value}</div>
            <div className="l">{s.label}</div>
          </div>
        ))}
      </div>
    </header>
  )
}

// ---------- the team ----------

function Team() {
  const t = C.team
  return (
    <section id="team">
      <h2>The team</h2>
      <p className="lede">
        <Text>{t.intro}</Text>
      </p>
      <figure className="group">
        <Photo src={t.groupPhoto.src} alt="the team" />
        <figcaption>
          <Text>{t.groupPhoto.caption}</Text>
        </figcaption>
      </figure>
      <div className="members">
        {t.members.map((m, i) => (
          <article key={i} className="member">
            <Photo src={m.photo} alt={m.name} className="face" />
            <h3>
              <Text>{m.name}</Text>
            </h3>
            <p className="role">
              {(Array.isArray(m.role) ? m.role : [m.role]).map((r, k) => (
                <span key={k}>
                  {k > 0 && <br />}
                  <Text>{r}</Text>
                </span>
              ))}
            </p>
            <p>
              <Text>{m.blurb}</Text>
            </p>
            {m.linkedin && (
              <a className="linkedin" href={m.linkedin} target="_blank" rel="noreferrer">
                LinkedIn ↗
              </a>
            )}
          </article>
        ))}
      </div>
    </section>
  )
}

// ---------- the competition ----------

function AIGP() {
  const a = C.aigp
  return (
    <section id="aigp">
      <h2>{a.title}</h2>
      <Paras items={a.intro} />
      <div className="aigp">
        <img src={BASE + a.stagesImage} alt="the four stages of the AI Grand Prix" />
        <ol className="stages">
          {a.stages.map((st) => (
            <li key={st.n}>
              <div className="n">{st.n}</div>
              <div>
                <h3>{st.name}</h3>
                <p>
                  <Text>{st.note}</Text>
                </p>
              </div>
            </li>
          ))}
        </ol>
      </div>
    </section>
  )
}

// ---------- aircraft ----------

function Aircraft() {
  return (
    <section id="aircraft">
      <h2>The aircraft</h2>
      <div className="cards three">
        {C.aircraft.map((a) => (
          <article key={a.id} className="card">
            <pre className="art" aria-hidden="true">
              {ART[a.art]}
            </pre>
            <div className="tag">{a.id}</div>
            <h3>{a.name}</h3>
            <p className="role">{a.role}</p>
            <p>{a.story}</p>
            <p className="fate">
              <Text>{a.fate}</Text>
            </p>
          </article>
        ))}
      </div>
    </section>
  )
}

// ---------- the technical report sections ----------

function Block({ b }) {
  if (typeof b === 'string')
    return (
      <p>
        <Text>{b}</Text>
      </p>
    )
  if (b.h) return <h3 className="mt">{b.h}</h3>
  if (b.list)
    return (
      <ul className="report">
        {b.list.map((x, i) => (
          <li key={i}>
            <Text>{x}</Text>
          </li>
        ))}
      </ul>
    )
  if (b.figure)
    return (
      <figure className="fig">
        <img src={BASE + b.figure.src} alt={b.figure.caption} loading="lazy" />
        <figcaption>{b.figure.caption}</figcaption>
      </figure>
    )
  if (b.video)
    return (
      <figure className="fig">
        <video src={BASE + b.video.src} controls muted playsInline preload="metadata" />
        <figcaption>{b.video.caption}</figcaption>
      </figure>
    )
  if (b.table)
    return (
      <table className="report">
        <thead>
          <tr>
            {b.table.head.map((h) => (
              <th key={h}>{h}</th>
            ))}
          </tr>
        </thead>
        <tbody>
          {b.table.rows.map((r, i) => (
            <tr key={i}>
              {r.map((c, j) => (
                <td key={j}>{c}</td>
              ))}
            </tr>
          ))}
        </tbody>
      </table>
    )
  return null
}

function Report() {
  return C.report.map((sec) => (
    <section key={sec.id} id={sec.id}>
      <h2>{sec.title}</h2>
      {sec.blocks.map((b, i) => (
        <Block key={i} b={b} />
      ))}
    </section>
  ))
}

// ---------- course map ----------

function CourseMap({ selected }) {
  const { gates, path } = data.course
  const flight = selected != null ? data.flights[selected] : null
  const pts = [...path, ...gates.map((g) => [g.x, g.y])]
  const xs = pts.map((p) => p[0])
  const ys = pts.map((p) => p[1])
  const pad = 2.5
  const x0 = Math.min(...xs) - pad
  const x1 = Math.max(...xs) + pad
  const y1 = Math.max(...ys) + pad
  const y0 = Math.min(...ys) - pad
  const W = x1 - x0
  const H = y1 - y0
  const X = (x) => x - x0
  const Y = (y) => y1 - y
  const poly = (arr) => arr.map(([x, y]) => `${X(x).toFixed(2)},${Y(y).toFixed(2)}`).join(' ')
  const seen = new Set()
  return (
    <div className="coursewrap">
      <svg viewBox={`0 0 ${W} ${H}`} className="course" preserveAspectRatio="xMidYMid meet">
        <polyline points={poly(path)} className="plan" />
        {gates.map((g, i) => {
          if (seen.has(g.label)) return null
          seen.add(g.label)
          const s = 1.35
          const deg = -((g.heading * 180) / Math.PI + 90)
          const stacked = g.label.startsWith('g8')
          return (
            <g key={i} transform={`translate(${X(g.x)} ${Y(g.y)}) rotate(${deg})`}>
              <rect x={-s} y={-0.18} width={2 * s} height={0.36} className={stacked ? 'gate stacked' : 'gate'} />
              <line x1={0} y1={0} x2={0} y2={-1.1} className="dir" transform="rotate(90)" />
              <text y={-0.6} className="glabel" transform={`rotate(${-deg})`}>
                {g.label.replace('-top', ' (stack)').replace('-low', '')}
              </text>
            </g>
          )
        })}
        <circle cx={X(0)} cy={Y(0)} r={0.45} className="start" />
        <text x={X(0) + 0.7} y={Y(0) + 0.3} className="glabel">
          start
        </text>
        {flight && (
          <>
            <polyline points={poly(flight.series.map((p) => [p.x, p.y]))} className="trace" />
            {(() => {
              const last = flight.series[flight.series.length - 1]
              return <circle cx={X(last.x)} cy={Y(last.y)} r={0.4} className="tracehead" />
            })()}
          </>
        )}
      </svg>
      <p className="caption">
        The published course in the plan frame: gate panels in red, the stacked gate in amber, the planned line dashed, the
        crossing direction as a tick.
        {flight
          ? ` In red: ${flight.aircraft}'s estimated position, ${flight.local}. Every flight of the event ended before gate 1.`
          : ' Select a flight below to draw its estimated track.'}
      </p>
    </div>
  )
}

// ---------- flight deck ----------

function AltChart({ fl }) {
  const s = fl.series
  const tmax = Math.max(s[s.length - 1].t, 5)
  const zmax = Math.max(...s.map((p) => p.z), 3.5)
  const W = 100
  const H = 46
  const X = (t) => (W * t) / tmax
  const Y = (z) => H - (H * z) / zmax
  const line = s.map((p) => `${X(p.t).toFixed(1)},${Y(p.z).toFixed(1)}`).join(' ')
  const rs = s.filter((p) => p.r != null && p.r < 20)
  const rmax = Math.max(...rs.map((p) => p.r), 8)
  const rline = rs.map((p) => `${X(p.t).toFixed(1)},${(H - (H * p.r) / rmax).toFixed(1)}`).join(' ')
  return (
    <div className="charts">
      <svg viewBox={`-6 -2 ${W + 8} ${H + 8}`} className="chart">
        <line x1={0} x2={W} y1={Y(GATE_Z)} y2={Y(GATE_Z)} className="ref" />
        <text x={W + 0.5} y={Y(GATE_Z) + 1} className="reflabel">
          centre
        </text>
        <line x1={0} x2={W} y1={Y(TOP_BAR_Z)} y2={Y(TOP_BAR_Z)} className="ref bar" />
        <text x={W + 0.5} y={Y(TOP_BAR_Z) + 1} className="reflabel">
          top bar
        </text>
        {fl.commit_t != null && (
          <>
            <line x1={X(fl.commit_t)} x2={X(fl.commit_t)} y1={0} y2={H} className="commit" />
            <text x={X(fl.commit_t) + 0.8} y={3} className="reflabel">
              commit
            </text>
          </>
        )}
        <polyline points={line} className="z" />
        <text x={-5.5} y={Y(0) + 1} className="axis">
          0
        </text>
        <text x={-5.5} y={Y(zmax) + 3} className="axis">
          {zmax.toFixed(0)} m
        </text>
        <text x={W - 6} y={H + 5} className="axis">
          {tmax.toFixed(0)} s
        </text>
        <text x={0} y={H + 5} className="axis">
          height (barometer)
        </text>
      </svg>
      <svg viewBox={`-6 -2 ${W + 8} ${H + 8}`} className="chart">
        <polyline points={rline} className="r" />
        <text x={-5.5} y={H + 1} className="axis">
          0
        </text>
        <text x={-5.5} y={3} className="axis">
          {rmax.toFixed(0)} m
        </text>
        <text x={0} y={H + 5} className="axis">
          range to the gate, from the camera
        </text>
        <text x={W - 6} y={H + 5} className="axis">
          {tmax.toFixed(0)} s
        </text>
      </svg>
    </div>
  )
}

function FlightDeck({ selected, setSelected }) {
  const fls = data.flights
  const fl = fls[selected]
  const prev = () => setSelected((selected + fls.length - 1) % fls.length)
  const next = () => setSelected((selected + 1) % fls.length)
  useEffect(() => {
    const onKey = (e) => {
      if (e.key === 'ArrowLeft') prev()
      if (e.key === 'ArrowRight') next()
    }
    window.addEventListener('keydown', onKey)
    return () => window.removeEventListener('keydown', onKey)
  })
  return (
    <>
      <h3 className="mt">Every flight, from its log</h3>
      <p>
        Thirteen of the fourteen autonomous course flights, as recorded on the aircraft. Arrows or arrow keys move between
        flights; the tiles jump.
      </p>
      <div className="strip">
        {fls.map((f, i) => (
          <button key={f.stamp} className={i === selected ? 'on' : ''} onClick={() => setSelected(i)} title={f.result}>
            <span className="who">{f.aircraft.replace('D', '')}</span>
            <span className="when">{f.local.split(', ')[1]}</span>
          </button>
        ))}
      </div>
      <div className="deck">
        <button className="arrow" onClick={prev} aria-label="previous flight">
          ‹
        </button>
        <div className="flight">
          <div className="head">
            <div>
              <div className="when">
                {fl.local} · {fl.session}
                {fl.attempt ? ` · attempt ${fl.attempt}` : ''}
              </div>
              <h3>
                {fl.aircraft}: {fl.result}
              </h3>
            </div>
            <div className="nums">
              <div>
                <b>{fl.duration_s}</b> s
              </div>
              <div>
                <b>{fl.max_z}</b> m max
              </div>
              <div>
                <b>{fl.min_range ?? '–'}</b> m closest
              </div>
              <div>
                <b>{fl.fixes ?? '–'}</b> fixes
              </div>
            </div>
          </div>
          <AltChart fl={fl} />
          <p className="cause">{fl.note}</p>
          {fl.events.length > 0 && (
            <ul className="events">
              {fl.events.map((e, i) => (
                <li key={i}>
                  <span>{e.t.toFixed(1)} s</span> {e.msg}
                </li>
              ))}
            </ul>
          )}
        </div>
        <button className="arrow" onClick={next} aria-label="next flight">
          ›
        </button>
      </div>
      <CourseMap selected={selected} />
    </>
  )
}

function Testing({ selected, setSelected }) {
  return (
    <section id="testing">
      <h2>Flight testing: Anduril, Costa Mesa, 19 to 22 September 2026</h2>
      <figure className="pq">
        <img src={BASE + C.aigp.pqImage} alt="Physical Qualifier poster" />
      </figure>
      <ol className="timeline">
        {C.timeline.map((t) => (
          <li key={t.title}>
            <div className="date">{t.date}</div>
            <div className="body">
              <h3>{t.title}</h3>
              <Paras items={t.body} />
            </div>
          </li>
        ))}
      </ol>
      <FlightDeck selected={selected} setSelected={setSelected} />
    </section>
  )
}

// ---------- race day ----------

function RaceDay({ setSelected }) {
  const byAttempt = Object.fromEntries(data.flights.map((f, i) => [f.attempt, i]))
  return (
    <section id="raceday">
      <h2>Race day: 27 minutes, seven attempts</h2>
      <table className="attempts">
        <thead>
          <tr>
            <th>#</th>
            <th>aircraft</th>
            <th>result</th>
            <th>cause, from the log</th>
          </tr>
        </thead>
        <tbody>
          {C.attempts.map((a) => (
            <tr
              key={a.n}
              className={byAttempt[a.n] != null ? 'link' : ''}
              onClick={() => {
                const i = byAttempt[a.n]
                if (i != null) {
                  setSelected(i)
                  document.getElementById('testing').scrollIntoView({ behavior: 'smooth' })
                }
              }}
            >
              <td>{a.n}</td>
              <td>{a.who}</td>
              <td>
                <b>{a.result}</b>
              </td>
              <td>{a.cause}</td>
            </tr>
          ))}
        </tbody>
      </table>
      <p className="caption">
        Rows with a log open that flight in the flight-testing section. Attempts 1 and 6 were not pulled off the aircraft
        before the slot ended; their numbers come from the console output.
      </p>
    </section>
  )
}

// ---------- analysis / lessons ----------

function Analysis() {
  return (
    <section id="analysis">
      <h2>Analysis</h2>
      <h3>What failed, in order of cost</h3>
      <div className="cards">
        {C.whyWeFailed.map((w, i) => (
          <article key={w.title} className="card">
            <div className="tag">{i + 1}</div>
            <h3>{w.title}</h3>
            <p>{w.body}</p>
          </article>
        ))}
      </div>
      <h3 className="mt">What worked</h3>
      <ul className="checks">
        {C.whatWorked.map((w) => (
          <li key={w}>{w}</li>
        ))}
      </ul>
    </section>
  )
}

function Lessons() {
  return (
    <section id="lessons">
      <h2>Lessons and next steps</h2>
      <ol className="lessons">
        {C.lessons.map((l) => (
          <li key={l}>{l}</li>
        ))}
      </ol>
      <h3 className="mt">Next steps, in order</h3>
      <ol className="lessons">
        {C.nextSteps.map((l) => (
          <li key={l}>{l}</li>
        ))}
      </ol>
    </section>
  )
}

// ---------- people / photos / footer ----------

function People() {
  return (
    <section id="people">
      <h2>{C.people.title}</h2>
      <Paras items={C.people.body} />
    </section>
  )
}

function Photos() {
  return (
    <section id="photos">
      <h2>Photos</h2>
      <div className="gallery">
        {C.photos.map((p) => (
          <figure key={p.src}>
            <img src={BASE + p.src} alt={p.caption} loading="lazy" />
            <figcaption>{p.caption}</figcaption>
          </figure>
        ))}
      </div>
    </section>
  )
}

function Footer() {
  return (
    <footer>
      <ul>
        {C.links.map((l) => (
          <li key={l.href}>
            <a href={l.href}>{l.label}</a>
          </li>
        ))}
      </ul>
      <p className="caption">
        Flight traces and numbers on this page are computed from the logs in the repository. Generated {data.generated}.
      </p>
    </footer>
  )
}

// ---------- app ----------

export default function App() {
  const [selected, setSelected] = useState(() => {
    const i = data.flights.findIndex((f) => f.stamp === '20260921_173440')
    return i >= 0 ? i : 0
  })
  return (
    <>
      <ChapterBar />
      <main>
        <Hero />
        <Team />
        <AIGP />
        <Aircraft />
        <Report />
        <Testing selected={selected} setSelected={setSelected} />
        <RaceDay setSelected={setSelected} />
        <Analysis />
        <Lessons />
        <People />
        <Photos />
        <Footer />
      </main>
    </>
  )
}
