# viz/ — PQ1OHGP 3D course viewer

`course_viewer.html` is a fully self-contained page (no build step, no
dependencies beyond Google Fonts): a hand-rolled canvas 3D renderer with
orbit/pan/zoom, gate frames at true dimensions, the double-gate
out-and-back, and an animated drone dot flying the racing line.

Open it by double-clicking the file, or serve it from anywhere that hosts
static files.

## Where the data inside it comes from

The gate constants (`GATES`), racing line (`PATH`), and cones (`CONES`)
near the top of the `<script>` block are **copied from data**, not
hand-authored:

1. `GATES` - the organizer's **published** coordinates (2026-09-15),
   i.e. `data/course_map.json` with `source: published`: 10 gates on an
   85 x 165 ft floor, ids = organizer gate numbers, order = traversal.
   Gate numbers shown in the viewer are **traversal order** (g0 = start
   = organizer gate 1, gK = gate K+1, g8 = the double gate 9). The
   stacked gate's top-opening height (4.05 m) is NOT published; it is the
   overhead estimate.

2. `PATH` / `CONES` - the racing line traced from the 2026-08-27 overhead
   image and its cone centroids, rigid-aligned onto the published frame
   (+0.34, +0.24 m, 0.06 deg; the estimate and the extractor that made
   it were retired on 2026-09-15 - git history has both, and the
   alignment record lives in the published map's `meta`). The line is an
   illustration - the
   flown line is `out/plans/plan_RACE.png`. It is drawn at opening height
   (1.35 m); near the double gate the traced points within 4.5 m are cut
   and bridged by a Catmull-Rom spline that flies the known maneuver:
   south through the top opening (4.05 m), U-turn, back north through the
   bottom (1.35 m). The cones were eyeballed from the PDF figure
   (`meta.cones_xy`, +-1.5 m).

To refresh after a map change: paste the order-sorted gates from
`data/course_map.json` (as `{id, order, x, y, yaw, entry, type, conf}`)
into `GATES`, update `GS` (the stacked gate's XY) and the HUD counts.
Everything else (camera, spline bridge, HUD) reads from those constants.

## Deploying (e.g. Vercel)

It is one static file — rename to `index.html` in a folder and
`vercel deploy`, or point a Vercel project at this directory with no
framework/build command. No server code, no env vars.

**Before making it public: this is our course intel for a live
competition.** Keep the deployment private/unlisted (Vercel deployment
protection) until after the September qualifier.

## Provenance / honesty

Gate positions are the organizer's published table (feet, converted:
x = X ft * 0.3048, y = (165 - Y ft) * 0.3048, heading = 90 - rot). The
overhead estimate that preceded it (327x547 phone screenshot, ~8.4 px/m,
grid-verified RMS 0.14 m) agreed to < 0.5 m after a rigid fit, which is
why its traced line and cones are still usable here. Opening heights and
the cone positions are estimates; everything else is published data.
