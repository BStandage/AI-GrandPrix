// Render the architecture SVG to a standalone file for the README.
import { createServer } from 'vite'
import { renderToStaticMarkup } from 'react-dom/server'
import { createElement } from 'react'
import { writeFileSync } from 'node:fs'

const vite = await createServer({ server: { middlewareMode: true }, appType: 'custom', logLevel: 'error' })
const { default: Architecture } = await vite.ssrLoadModule('/src/Architecture.jsx')
let html = renderToStaticMarkup(createElement(Architecture))
const svg = html.slice(html.indexOf('<svg'), html.indexOf('</svg>') + 6)
const style = `<style>
  svg { background: #0d0f12; font-family: Inter, system-ui, sans-serif; }
  .abox { fill: #15181d; stroke: #2a2f38; stroke-width: 1.2; }
  .abox.red { stroke: #e5322d; } .abox.amber { stroke: #f2b134; }
  .atitle { fill: #e9ecf1; font-size: 15px; font-weight: 700; }
  .aline { fill: #a4abb8; font-size: 12.5px; font-family: 'JetBrains Mono', ui-monospace, Consolas, monospace; }
  .alane { fill: none; stroke: #2a2f38; stroke-dasharray: 6 5; }
  .alanetitle { fill: #ff6b66; font-size: 12px; letter-spacing: 0.08em; font-family: ui-monospace, Consolas, monospace; }
  .aarrow { fill: none; stroke: #a4abb8; stroke-width: 1.6; } .aarrow.dashed { stroke-dasharray: 5 4; }
  .ahead { fill: #a4abb8; } .alabel { fill: #a4abb8; font-size: 11.5px; font-family: ui-monospace, Consolas, monospace; }
</style>`
const out = svg.replace(/<svg([^>]*)>/, (m, a) => `<svg xmlns="http://www.w3.org/2000/svg"${a.replace(' class="archsvg"', '')}>${style}`)
writeFileSync('../docs/architecture.svg', out)
console.log('wrote docs/architecture.svg', out.length, 'bytes')
await vite.close()
