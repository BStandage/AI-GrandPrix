import { useEffect, useMemo, useState } from 'react'
import * as C from './content.js'
import data from './data.json'
import artD43 from './art/d43.txt?raw'
import artD44 from './art/d44.txt?raw'
import artD45 from './art/d45.txt?raw'
const ART = { d43: artD43, d44: artD44, d45: artD45 }
import './App.css'

// ---------- helpers ----------

function Text({ children }) {
  // "[BRIAN: ...]" renders as a visible note until it is replaced
  if (typeof children === 'string' && children.startsWith('[BRIAN:')) {
    return <mark className="todo">{children.slice(1, -1)}</mark>
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

const GATE_Z = 1.35
const TOP_BAR_Z = 1.35 + 0.7 // opening is ~1.4 m: the bar is ~0.7 m above centre

// ---------- chapter bar ----------

const CHAPTERS = [
  ['week', 'The week'],
  ['aircraft', 'Aircraft'],
  ['course', 'Course'],
  ['flights', 'Flights'],
  ['slot', 'The slot'],
  ['why', 'Why'],
  ['worked', 'What worked'],
  ['lessons', 'Lessons'],
  ['people', 'People'],
  ['photos', 'Photos'],
]

function ChapterBar() {
  const [active, setActive] = useState('week')
  useEffect(() => {
    const obs = new IntersectionObserver(
      (entries) => {
        entries.forEach((e) => e.isIntersecting && setActive(e.target.id))
      },
      { rootMargin: '-40% 0px -55% 0px' },
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
      <p className="kicker">AI Grand Prix · September 2026</p>
      <h1>{C.site.title}</h1>
      <p className="sub">{C.site.subtitle}</p>
      <p className="lede">{C.site.tagline}</p>
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

// ---------- the week ----------

function Week() {
  return (
    <section id="week">
      <h2>The week</h2>
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
            <pre className="art" aria-hidden="true">{ART[a.art]}</pre>
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
  const y0 = Math.min(...ys) - pad
  const y1 = Math.max(...ys) + pad
  const W = x1 - x0
  const H = y1 - y0
  // world y is "down the course"; draw it upward on screen
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
          const deg = (-g.heading * 180) / Math.PI
          const stacked = g.label.startsWith('g8')
          return (
            <g key={i} transform={`translate(${X(g.x)} ${Y(g.y)}) rotate(${deg})`}>
              <rect x={-s} y={-0.18} width={2 * s} height={0.36} className={stacked ? 'gate stacked' : 'gate'} />
              <text y={-0.6} className="glabel" transform={`rotate(${-deg})`}>
                {g.label.replace('-top', ' (stack)').replace('-low', '')}
              </text>
            </g>
          )
        })}
        <circle cx={X(0)} cy={Y(0)} r={0.45} className="start" />
        <text x={X(0) + 0.7} y={Y(0) + 0.3} className="glabel">start</text>
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
        The course as the aircraft knew it: gates from the published map in the flight frame, the planned line in grey.
        {flight
          ? ` In red: where ${flight.aircraft} estimated it was, ${flight.local}. Every flight of the week ends before gate 1.`
          : ' Pick a flight below to draw where it went.'}
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
          height (baro)
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
    <section id="flights">
      <h2>Every flight, from its own log</h2>
      <p className="lede">
        Thirteen of the fourteen autonomous course flights, pulled off the aircraft and drawn as they were recorded. Use the
        arrows or the arrow keys.
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
    </section>
  )
}

// ---------- the slot ----------

function Slot({ setSelected }) {
  const byAttempt = Object.fromEntries(data.flights.map((f, i) => [f.attempt, i]))
  return (
    <section id="slot">
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
                  document.getElementById('flights').scrollIntoView({ behavior: 'smooth' })
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
        Click a row to open that flight above. Attempts 1 and 6 were not pulled off the aircraft before the slot ended; their
        numbers come from the console.
      </p>
    </section>
  )
}

// ---------- why / worked / lessons ----------

function Why() {
  return (
    <section id="why">
      <h2>What went wrong, and why</h2>
      <div className="cards">
        {C.whyWeFailed.map((w, i) => (
          <article key={w.title} className="card">
            <div className="tag">{i + 1}</div>
            <h3>{w.title}</h3>
            <p>{w.body}</p>
          </article>
        ))}
      </div>
      <h3 className="mt">{C.simStory.title}</h3>
      <Paras items={C.simStory.body} />
    </section>
  )
}

function Worked() {
  return (
    <section id="worked">
      <h2>What worked</h2>
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
      <h2>Lessons</h2>
      <ol className="lessons">
        {C.lessons.map((l) => (
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
            <img src={import.meta.env.BASE_URL + p.src} alt={p.caption} loading="lazy" />
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
        Every number on this page is computed from the flight logs and the git history in the repository. Generated{' '}
        {data.generated}.
      </p>
    </footer>
  )
}

// ---------- app ----------

export default function App() {
  const [selected, setSelected] = useState(() => {
    // open on the best flight of the week
    const i = data.flights.findIndex((f) => f.stamp === '20260921_173440')
    return i >= 0 ? i : 0
  })
  return (
    <>
      <ChapterBar />
      <main>
        <Hero />
        <Week />
        <Aircraft />
        <section id="course">
          <h2>The course</h2>
          <CourseMap selected={null} />
        </section>
        <FlightDeck selected={selected} setSelected={setSelected} />
        <Slot setSelected={setSelected} />
        <Why />
        <Worked />
        <Lessons />
        <People />
        <Photos />
        <Footer />
      </main>
    </>
  )
}
