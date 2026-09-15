# viz/ — PQ1OHGP 3D course viewer

`course_viewer.html` is a fully self-contained page (no build step, no
dependencies beyond Google Fonts): a hand-rolled canvas 3D renderer with
orbit/pan/zoom, gate frames at true dimensions, the double-gate
out-and-back, and an animated drone dot flying the racing line.

Open it by double-clicking the file, or serve it from anywhere that hosts
static files.

## Where the data inside it comes from

The gate constants (`GATES`), racing line (`PATH`), and cones (`CONES`)
near the top of the `<script>` block are **copied from extractor output**,
not hand-authored:

1. `GATES` — from `data/course_map.json`, produced by:

   ```
   python scripts/extract_course_map.py assets/course_overhead.png \
       --corner 69 91 0 50  --corner 237 91 20 50 \
       --corner 237 508 20 0 --corner 69 508 0 0 \
       --start-px 100 369 --roi -1 0 23 56
   ```

   Gate numbers shown in the viewer are **traversal order** (g0 = start,
   g8 = double gate), not extraction ids. **Since 2026-09-15 `GATES` holds
   the organizer's PUBLISHED coordinates** (`data/course_map.json`, source
   `published`; the overhead estimate moved to
   `data/course_map_overhead_estimate.json`): 10 gates, g0 = organizer gate
   1, gK = gate K+1. `PATH`/`CONES` are the old extraction rigid-aligned
   onto the published frame (+0.34, +0.24 m, 0.06 deg) - the line is still
   an illustration, and the cones were eyeballed from the PDF figure.

2. `PATH` / `CONES` — the traced racing line and cone centroids, dumped
   from the same extraction (see `trace_line` / the `blob` shapes in
   `scripts/extract_course_map.py`). The line is drawn at opening height
   (1.35 m); near the double gate the traced points within 4.5 m are cut
   and bridged by a Catmull-Rom spline that flies the known maneuver:
   south through the top opening (4.05 m), U-turn, back north through the
   bottom (1.35 m). The climb/dive *shape* is illustrative — only the
   crossings and directions are data.

To refresh after a re-extraction: re-run the command above, paste the new
`gates` (converted to order-sorted `{id, order, x, y, yaw, entry, type,
conf}`) into `GATES`, and the new path/cone dumps into `PATH` / `CONES`.
Everything else (camera, spline bridge, HUD) reads from those constants.

## Deploying (e.g. Vercel)

It is one static file — rename to `index.html` in a folder and
`vercel deploy`, or point a Vercel project at this directory with no
framework/build command. No server code, no env vars.

**Before making it public: this is our course intel for a live
competition.** Keep the deployment private/unlisted (Vercel deployment
protection) until after the September qualifier.

## Provenance / honesty

Positions are meter-scale estimates from a 327×547 phone screenshot
(~8.4 px/m), homography-verified against the floor grid (RMS 0.14 m).
`data/course_map.json` carries `source: estimated_from_overhead_image`;
swap in `published` or `survey_refined` coordinates via
`src/common/course_map.py` when better data lands.
