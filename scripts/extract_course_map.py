"""
Extract a metric gate map from a top-down course render (PQ overhead image).

    python scripts/extract_course_map.py assets/course_overhead.png \
        --corner 69 91 0 50  --corner 236 91 20 50 \
        --corner 236 509 20 0 --corner 69 509 0 0

    python scripts/extract_course_map.py assets/course_overhead.png --click

Pipeline:
  1. Homography from 4 user-supplied pixel<->metric floor correspondences.
  2. Grid verification: detect floor grid lines, project to metric, report
     max/RMS residual vs the grid pitch. Bad residual => REFUSE to emit.
  3. Segment saturated strokes off the gray floor; erosion splits thick
     gate bars from the thin racing line even where they touch.
     Classification is SHAPE/SIZE ONLY (colors are display styling).
  4. Bars co-located in XY (< STACK_MERGE_M) merge into the one stacked gate.
  5. Skeleton-trace the racing line (straightest-continuation through
     junctions) to get traversal order and entry headings. If tracing
     fails, order is null — never guessed.
  6. Emit data/course_map.json (schema in repo brief) + out/ overlays.

Standalone: cv2 + numpy only, no repo imports. The consumer-side loader
lives in src/common/course_map.py.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import cv2
import numpy as np

# --- gate geometry (published spec VADR-TS-004; NOT inferred from image) ----
GATE_OUTER_M = 2.70
GATE_OPENING_M = 1.50
GATE_DEPTH_M = 0.26
# frame sits flush on the deck, opening centered in the square outer frame
OPENING_CENTER_Z_M = round(GATE_OUTER_M / 2.0, 3)             # 1.35
STACK_TOP_Z_M = round(OPENING_CENTER_Z_M + GATE_OUTER_M, 3)   # 4.05

STACK_MERGE_M = 1.5      # bar centroids closer than this = the stacked gate
MIN_GATE_SPACING_M = 2.7  # distinct gates closer than this is implausible
GRID_TOL_M = 0.25         # max inlier RMS before the homography is rejected
PASS_RADIUS_M = 2.5       # line must come this close to count as a gate pass

# bar/blob classification bands, metric (bar stroke length ~= 2.7 m frame).
# Gate bars are drawn as THICK strokes; the racing line is a thin stroke —
# stroke thickness (distance transform), not PCA width, separates a bar
# from a chopped/curved racing-line fragment.
BAR_LEN_M = (1.6, 4.2)
BAR_MIN_ASPECT = 2.0
LINE_MAX_THICK_M = 0.55
CROSS_MAX_SOLIDITY = 0.75   # plus/X of two bars is concave; discs are ~1.0


# =========================================================================
# homography
# =========================================================================

def compute_homography(px_pts, m_pts):
    """px_pts, m_pts: (4,2) arrays. Returns H mapping image px -> metric m."""
    px = np.asarray(px_pts, np.float64).reshape(-1, 2)
    mm = np.asarray(m_pts, np.float64).reshape(-1, 2)
    if px.shape[0] < 4:
        raise ValueError("need at least 4 correspondences")
    H, _ = cv2.findHomography(px, mm, 0)
    if H is None or abs(np.linalg.det(H)) < 1e-12:
        raise ValueError("findHomography failed (degenerate points?)")
    back = cv2.perspectiveTransform(px.reshape(-1, 1, 2), H).reshape(-1, 2)
    if np.linalg.norm(back - mm, axis=1).max() > 1e-3:
        raise ValueError("homography does not fit the given correspondences "
                         "(collinear or inconsistent points?)")
    return H


def px_to_m(H, pts):
    p = np.asarray(pts, np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(p, H).reshape(-1, 2)


def m_to_px(H, pts):
    return px_to_m(np.linalg.inv(H), pts)


# =========================================================================
# grid verification
# =========================================================================

def _profile_peaks(prof, min_dist):
    prof = cv2.GaussianBlur(prof.reshape(-1, 1).astype(np.float32),
                            (1, 5), 0).ravel()
    # robust threshold: percentile floor + fraction of a robust peak level,
    # so one huge spurious spike cannot mask the real grid lines
    base = np.percentile(prof, 60)
    peak_level = np.percentile(prof[prof > base], 90) if (prof > base).any() else 0
    thr = base + 0.35 * max(peak_level - base, 1e-6)
    idx = []
    for i in range(2, len(prof) - 2):
        if prof[i] > thr and prof[i] >= prof[i - 2:i + 3].max():
            if not idx or i - idx[-1] >= min_dist:
                idx.append(i)
            elif prof[i] > prof[idx[-1]]:
                idx[-1] = i
    return idx


def verify_grid(gray, fg_mask, H, pitch=5.0, scale=8):
    """Detect grid lines in rectified metric space; residuals vs pitch grid.

    Returns dict with max/RMS inlier residuals and ok flag. Lines further
    than 1 m from any pitch multiple are reported as outliers (arena
    borders etc), not silently mixed into the score.
    """
    h, w = gray.shape
    corners_m = px_to_m(H, [[0, 0], [w, 0], [w, h], [0, h]])
    x0, y0 = corners_m.min(0)
    x1, y1 = corners_m.max(0)
    T = np.array([[scale, 0, -x0 * scale],
                  [0, -scale, y1 * scale],   # metric +y up -> canvas row down
                  [0, 0, 1]], np.float64)
    W = int((x1 - x0) * scale) + 1
    Hc = int((y1 - y0) * scale) + 1
    if W * Hc > 4e7:
        raise ValueError("rectified canvas too large; check correspondences")
    warped = cv2.warpPerspective(gray, T @ H, (W, Hc))
    keep = cv2.warpPerspective(
        (fg_mask == 0).astype(np.uint8) * 255, T @ H, (W, Hc))
    keep = cv2.erode(keep, np.ones((5, 5), np.uint8))
    # ignore the warp border: pixels outside the source image read as black
    # and would swamp the edge profiles with a fake mega-edge
    valid = cv2.warpPerspective(np.full_like(gray, 255), T @ H, (W, Hc))
    # borderValue=0 matters: the default treats canvas edges as white and
    # the mask would never shrink away from the warp boundary
    valid = cv2.erode(valid, np.ones((11, 11), np.uint8),
                      borderType=cv2.BORDER_CONSTANT, borderValue=0)
    good = (keep > 0) & (valid > 0)

    gx = np.abs(cv2.Sobel(warped, cv2.CV_32F, 1, 0, 3)) * good
    gy = np.abs(cv2.Sobel(warped, cv2.CV_32F, 0, 1, 3)) * good
    cols = _profile_peaks(gx.sum(0), min_dist=int(1.5 * scale))
    rows = _profile_peaks(gy.sum(1), min_dist=int(1.5 * scale))

    def residuals(px_positions, origin, sign):
        out = []
        for p in px_positions:
            m = origin + sign * p / scale
            r = m - pitch * round(m / pitch)
            out.append((m, r))
        return out

    rx = residuals(cols, x0, +1)
    ry = residuals(rows, y1, -1)
    # a line >0.5 m off-pitch is not a grid line (arena borders, fences)
    inl = [r for _, r in rx + ry if abs(r) <= 0.5]
    outl = [(m, r) for m, r in rx + ry if abs(r) > 0.5]
    res = {
        "n_lines_x": len(rx), "n_lines_y": len(ry),
        "lines_x_m": [round(m, 2) for m, _ in rx],
        "lines_y_m": [round(m, 2) for m, _ in ry],
        "n_inliers": len(inl), "n_outliers": len(outl),
        "outliers_m": [round(m, 2) for m, _ in outl],
    }
    if len(inl) >= 6 and len(rx) >= 3 and len(ry) >= 3:
        res["max_err_m"] = round(float(np.max(np.abs(inl))), 3)
        res["rms_err_m"] = round(float(np.sqrt(np.mean(np.square(inl)))), 3)
        res["ok"] = res["rms_err_m"] <= GRID_TOL_M and res["max_err_m"] <= 2 * GRID_TOL_M
        res["status"] = "verified" if res["ok"] else "FAILED"
    else:
        res["max_err_m"] = res["rms_err_m"] = None
        res["ok"] = False
        res["status"] = "inconclusive (too few grid lines found)"
    return res


# =========================================================================
# segmentation — shape/size only; hue bands exist ONLY to split touching
# strokes of different styling into separate components
# =========================================================================

def _cross_axes(d):
    """Estimate the two stroke axes of a plus/X component from its centered
    metric pixel cloud: score candidate axes by how many pixels fall in a
    narrow band along them, keep the two best (>=30 deg apart)."""
    scored = []
    for deg in range(0, 180, 3):
        a = math.radians(deg)
        perp = np.abs(d[:, 0] * math.sin(a) - d[:, 1] * math.cos(a))
        scored.append((int((perp < 0.35).sum()), deg))
    picked = []
    for s, deg in sorted(scored, reverse=True):
        if all(min(abs(deg - p[0]), 180 - abs(deg - p[0])) > 30
               for p in picked):
            picked.append((deg, s))
        if len(picked) == 2:
            break
    return [(math.radians(deg), s) for deg, s in picked]


def segment(img, H, sat_thr=60, val_thr=60):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hch = hsv[..., 0].astype(int)
    fg = ((hsv[..., 1].astype(int) > sat_thr)
          & (hsv[..., 2].astype(int) > val_thr)).astype(np.uint8)

    bands = [
        (hch < 25) | (hch > 160),                       # warm strokes
        (hch >= 85) & (hch <= 135),                     # cool strokes
        ((hch >= 25) & (hch < 85)) | ((hch > 135) & (hch <= 160)),
    ]
    k3 = np.ones((3, 3), np.uint8)
    shapes = []
    shape_mask = np.zeros_like(fg)
    for band in bands:
        bmask = (fg & band).astype(np.uint8)
        cores = cv2.erode(bmask, k3, iterations=1)
        n, lab = cv2.connectedComponents(cores, 8)
        for i in range(1, n):
            core = (lab == i).astype(np.uint8)
            if core.sum() < 12:
                continue
            # undo the erosion without crawling far along the thin line
            full = cv2.dilate(core, k3, iterations=2) & bmask
            # measure on the CORE: the erosion already stripped any thin
            # racing-line stroke that touches the shape, so core stats are
            # clean where full-shape stats would be contaminated
            ys, xs = np.nonzero(core)
            met = px_to_m(H, np.column_stack([xs, ys]).astype(np.float64))
            c = met.mean(0)
            d = met - c
            cov = d.T @ d / len(d)
            evals, evecs = np.linalg.eigh(cov)
            axis = evecs[:, 1]
            lo = d @ axis
            sh = d @ evecs[:, 0]
            length = float(lo.max() - lo.min())
            width = float(max(sh.max() - sh.min(), 1e-3))
            yaw = math.atan2(axis[1], axis[0])
            aspect = length / width
            # stroke thickness in meters (local scale at the centroid);
            # +2 px compensates the 3x3 erosion (1 px per side)
            t_px = 2.0 * float(cv2.distanceTransform(core * 255,
                                                     cv2.DIST_L2, 3).max()) + 2.0
            cx, cy = float(xs.mean()), float(ys.mean())
            j = px_to_m(H, [[cx, cy], [cx + 1, cy], [cx, cy + 1]])
            m_per_px = 0.5 * (np.linalg.norm(j[1] - j[0])
                              + np.linalg.norm(j[2] - j[0]))
            thick = t_px * m_per_px
            # compensate the erosion in the extents as well
            length += 2.0 * m_per_px
            width += 2.0 * m_per_px
            aspect = length / width
            hull = cv2.convexHull(np.column_stack([xs, ys]).astype(np.float32))
            solidity = float(core.sum()) / max(cv2.contourArea(hull), 1e-6)
            if (thick < LINE_MAX_THICK_M
                    and (aspect >= BAR_MIN_ASPECT or length > 4.5)):
                kind = "line-piece"   # thin stroke = racing-line fragment
            elif (BAR_LEN_M[0] <= length <= BAR_LEN_M[1]
                    and aspect >= BAR_MIN_ASPECT and thick <= 1.3):
                kind = "bar"
            elif (BAR_LEN_M[0] <= length <= BAR_LEN_M[1]
                    and 1.0 <= width <= BAR_LEN_M[1]
                    and solidity < CROSS_MAX_SOLIDITY):
                kind = "cross"        # two overlapped bars = stacked gate
            elif length <= 4.5 and aspect < BAR_MIN_ASPECT:
                kind = "blob"    # cones / cone clusters / markers -> excluded
            else:
                kind = "unknown"
            shape = {
                "kind": kind, "xy": [float(c[0]), float(c[1])],
                "yaw": yaw, "length_m": length, "width_m": width,
                "thick_m": round(thick, 2), "solidity": round(solidity, 2),
                "px": [cx, cy],
                "area_px": int(full.sum()),
            }
            if kind == "cross":
                shape["axes"] = _cross_axes(d)
            shapes.append(shape)
            if kind != "line-piece":   # line fragments stay in the line mask
                shape_mask |= full
    line_mask = (fg & ~cv2.dilate(shape_mask, k3, 1)).astype(np.uint8)
    # drop tiny specks, keep sizeable strokes
    n, lab, stats, _ = cv2.connectedComponentsWithStats(line_mask, 8)
    clean = np.zeros_like(line_mask)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] >= 20:
            clean[lab == i] = 1
    return shapes, clean, fg


def group_gates(shapes):
    """Merge co-located bars into the stacked gate; everything else single.

    A "cross" shape (two bars overlapped into one plus/X component) is a
    stacked gate on its own, with an ambiguous bar axis.
    """
    bars = [s for s in shapes if s["kind"] == "bar"]
    used = [False] * len(bars)
    gates = []
    for i, a in enumerate(bars):
        if used[i]:
            continue
        members = [a]
        used[i] = True
        for j in range(i + 1, len(bars)):
            if used[j]:
                continue
            b = bars[j]
            if math.dist(a["xy"], b["xy"]) < STACK_MERGE_M:
                members.append(b)
                used[j] = True
        xy = np.mean([m["xy"] for m in members], axis=0)
        crossed = (len(members) > 1 and abs(wrap_pi(
            members[0]["yaw"] - members[1]["yaw"])) > math.radians(30))
        gates.append({
            "xy": [float(xy[0]), float(xy[1])],
            "members": members,
            "type": "stacked" if len(members) > 1 else "single",
            "crossed": crossed,
        })
    for x in (s for s in shapes if s["kind"] == "cross"):
        gates.append({
            "xy": list(x["xy"]),
            "members": [x],
            "type": "stacked",
            "crossed": True,
        })
    return gates


# =========================================================================
# racing-line tracing
# =========================================================================

def _thin(mask):
    """Zhang-Suen thinning (no cv2.ximgproc in this env)."""
    img = (mask > 0).astype(np.uint8)
    while True:
        changed = 0
        for step in (0, 1):
            p = np.pad(img, 1)
            P2 = p[:-2, 1:-1]; P3 = p[:-2, 2:]; P4 = p[1:-1, 2:]
            P5 = p[2:, 2:]; P6 = p[2:, 1:-1]; P7 = p[2:, :-2]
            P8 = p[1:-1, :-2]; P9 = p[:-2, :-2]
            B = (P2 + P3 + P4 + P5 + P6 + P7 + P8 + P9)
            seq = [P2, P3, P4, P5, P6, P7, P8, P9, P2]
            A = sum(((seq[k] == 0) & (seq[k + 1] == 1)).astype(np.uint8)
                    for k in range(8))
            if step == 0:
                c1 = (P2 * P4 * P6 == 0) & (P4 * P6 * P8 == 0)
            else:
                c1 = (P2 * P4 * P8 == 0) & (P2 * P6 * P8 == 0)
            kill = (img == 1) & (A == 1) & (B >= 2) & (B <= 6) & c1
            img[kill] = 0
            changed += int(kill.sum())
        if not changed:
            return img


def _skel_endpoint_dirs(skel):
    """Endpoints of a thinned mask + the direction each tail points OUT of
    the curve (unit vector in (col,row) = image xy)."""
    pset = {tuple(p) for p in map(tuple, np.column_stack(np.nonzero(skel)))}

    def nb(p):
        r, c = p
        return [(r + dr, c + dc) for dr in (-1, 0, 1) for dc in (-1, 0, 1)
                if (dr or dc) and (r + dr, c + dc) in pset]

    out = []
    for p in pset:
        if len(nb(p)) != 1:
            continue
        # walk a few px inward to estimate the tail direction
        prev, cur = None, p
        for _ in range(6):
            nxt = [q for q in nb(cur) if q != prev]
            if len(nxt) != 1:
                break
            prev, cur = cur, nxt[0]
        v = np.array([p[1] - cur[1], p[0] - cur[0]], float)
        n = np.linalg.norm(v)
        if n > 1e-6:
            out.append((p, v / n))
    return out


def _stitch_skeleton(skel, H, meta, max_gap_m=4.6):
    """Connect facing skeleton endpoints across gaps where the racing line
    ran underneath a gate bar (the bar stroke swallowed the line there).
    Only stitches pairs whose tails point at each other — parallel nearby
    strands are left alone."""
    eps = _skel_endpoint_dirs(skel)
    stitched = 0
    used = set()
    for i in range(len(eps)):
        for j in range(i + 1, len(eps)):
            (pa, da), (pb, db) = eps[i], eps[j]
            if pa in used or pb in used:
                continue
            gap_px = np.array([pb[1] - pa[1], pb[0] - pa[0]], float)
            d_px = np.linalg.norm(gap_px)
            if d_px < 2:
                continue
            mm = px_to_m(H, [[pa[1], pa[0]], [pb[1], pb[0]]])
            if np.linalg.norm(mm[1] - mm[0]) > max_gap_m:
                continue
            u = gap_px / d_px
            if np.dot(da, u) < 0.5 or np.dot(db, -u) < 0.5:
                continue
            cv2.line(skel, (pa[1], pa[0]), (pb[1], pb[0]), 1, 1)
            used.update((pa, pb))
            stitched += 1
    if stitched:
        meta["stitched_gaps"] = stitched
        skel = _thin(skel)
    return skel


def trace_line(line_mask, H, start_px=None):
    """Order the racing-line pixels into one path. Returns (path_m, meta).

    path_m is (N,2) metric, resampled at 0.25 m; None when tracing failed.
    """
    meta = {}
    # bridge small gaps left where gate bars crossed the line
    closed = cv2.morphologyEx(line_mask, cv2.MORPH_CLOSE,
                              cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)))
    skel = _thin(closed)
    skel = _stitch_skeleton(skel, H, meta)
    pts = np.column_stack(np.nonzero(skel))          # (row, col)
    if len(pts) < 30:
        meta["fail"] = "skeleton too small"
        return None, meta
    pset = {tuple(p) for p in map(tuple, pts)}
    nbrs = {}
    for r, c in pset:
        nbrs[(r, c)] = [(r + dr, c + dc)
                        for dr in (-1, 0, 1) for dc in (-1, 0, 1)
                        if (dr or dc) and (r + dr, c + dc) in pset]
    ends = [p for p, nn in nbrs.items() if len(nn) == 1]
    meta["n_skel_px"] = len(pset)
    meta["n_endpoints"] = len(ends)

    if ends:
        if start_px is not None:
            sp = np.array([start_px[1], start_px[0]])   # (row, col)
            byd = sorted(ends, key=lambda p: np.hypot(*(np.array(p) - sp)))
            start = byd[0]
            if len(byd) > 1:
                mm = px_to_m(H, [[byd[0][1], byd[0][0]],
                                 [byd[1][1], byd[1][0]]])
                if np.linalg.norm(mm[1] - mm[0]) < 3.0:
                    # both curve ends sit at the start/finish V: picking
                    # either is a direction coin flip, not a resolution
                    meta["direction_ambiguous"] = True
        else:
            # no hint: take the endpoint lowest in the image (start/finish
            # gates sit at the near edge in this render family) — flagged
            start = max(ends, key=lambda p: p[0])
            meta["start_assumed"] = True
    else:
        meta["closed_loop"] = True
        if start_px is not None:
            sp = np.array([start_px[1], start_px[0]])   # (row, col)
            start = min(pset, key=lambda p: np.hypot(*(np.array(p) - sp)))
            # start point pinned, but loop direction is still a coin flip
            meta["direction_ambiguous"] = True
        else:
            start = max(pset, key=lambda p: p[0])
            meta["start_assumed"] = True

    visited = set()
    path = [start]
    dirv = np.zeros(2)
    cur = start
    while True:
        cands = [q for q in nbrs[cur] if (cur, q) not in visited]
        if not cands:
            break
        if np.linalg.norm(dirv) < 1e-6:
            nxt = cands[0]
        else:
            nxt = max(cands, key=lambda q: float(
                np.dot(dirv, np.array(q, float) - np.array(cur, float))
                / (np.linalg.norm(np.array(q, float) - np.array(cur, float)))))
        visited.add((cur, nxt))
        visited.add((nxt, cur))
        step = np.array(nxt, float) - np.array(cur, float)
        step /= np.linalg.norm(step)
        dirv = 0.7 * dirv + 0.3 * step
        n = np.linalg.norm(dirv)
        if n > 1e-6:
            dirv /= n
        path.append(nxt)
        cur = nxt

    coverage = len({p for p in path}) / len(pset)
    meta["coverage"] = round(coverage, 3)
    if coverage < 0.6:
        meta["fail"] = f"trace covered only {coverage:.0%} of skeleton"
        return None, meta

    px = np.array([(c, r) for r, c in path], np.float64)
    met = px_to_m(H, px)
    # resample at uniform arc step, light smoothing
    seg = np.linalg.norm(np.diff(met, axis=0), axis=1)
    s = np.concatenate([[0], np.cumsum(seg)])
    total = s[-1]
    meta["arc_len_m"] = round(float(total), 1)
    if total < 10:
        meta["fail"] = "traced path too short"
        return None, meta
    step_m = 0.25
    su = np.arange(0, total, step_m)
    res = np.column_stack([np.interp(su, s, met[:, 0]),
                           np.interp(su, s, met[:, 1])])
    kernel = np.ones(5) / 5
    for k in range(2):
        res[:, k] = np.convolve(np.pad(res[:, k], 2, mode="edge"),
                                kernel, mode="valid")
    return res, meta


def assign_order(gates, path, reverse=False):
    """Find each gate's pass(es) along the path; rank by arc position."""
    if path is None:
        for g in gates:
            g["order"] = None
            g["entry_heading_rad"] = None
            g["passes"] = []
        return False
    if reverse:
        path = path[::-1]
    step_m = 0.25
    win = int(4.0 / step_m)
    for g in gates:
        d = np.linalg.norm(path - np.array(g["xy"]), axis=1)
        passes = []
        for i in range(len(d)):
            lo, hi = max(0, i - win), min(len(d), i + win + 1)
            if d[i] <= PASS_RADIUS_M and d[i] == d[lo:hi].min():
                if not passes or (i - passes[-1]) > win:
                    passes.append(i)
        g["passes"] = passes
        if passes:
            i = passes[0]
            a, b = max(0, i - 4), min(len(path) - 1, i + 4)
            t = path[b] - path[a]
            g["entry_heading_rad"] = math.atan2(t[1], t[0])
        else:
            g["entry_heading_rad"] = None
    hit = [g for g in gates if g["passes"]]
    hit.sort(key=lambda g: g["passes"][0])
    for rank, g in enumerate(hit):
        g["order"] = rank
    for g in gates:
        if not g["passes"]:
            g["order"] = None
    return True


# =========================================================================
# assembly / emission
# =========================================================================

def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def build_map(gates, grid_res, trace_meta, source_image, warnings):
    out_gates = []
    for gid, g in enumerate(gates):
        members = g["members"]
        heading = g["entry_heading_rad"]
        crossed = g.get("crossed", False)
        if crossed and heading is not None:
            # crossed rendering makes the shared axis ambiguous; the
            # opening must face travel, so of the two stroke axes take
            # the one most perpendicular to the entry heading
            if len(members) > 1:
                cands = [(m["yaw"], 1) for m in members]
            else:
                cands = (members[0].get("axes")
                         or [(wrap_pi(heading + math.pi / 2), 1)])
            if len(cands) > 1 and cands[0][1] >= 1.15 * cands[1][1]:
                # one stroke clearly dominates: that's the drawn gate bar
                yaw = cands[0][0]
            else:
                yaw = min((c[0] for c in cands), key=lambda a: abs(
                    abs(wrap_pi(heading - a)) - math.pi / 2))
        else:
            yaw = members[0]["yaw"]
        conf = "med"
        lens = [m["length_m"] for m in members]
        if g["type"] == "stacked" or crossed:
            conf = "low"
        if not (2.1 <= np.mean(lens) <= 3.3):
            conf = "low"
        if heading is not None:
            perp = abs(wrap_pi(heading - yaw))
            if not (math.radians(45) <= perp <= math.radians(135)):
                conf = "low"
                warnings.append(
                    f"gate {gid}: racing line nearly parallel to bar axis")
        else:
            conf = "low"
        if grid_res.get("rms_err_m") is None or not grid_res.get("ok"):
            conf = "low"
        entry = None
        if heading is not None:
            # snap heading to the gate normal nearer the line tangent
            n1 = wrap_pi(yaw + math.pi / 2)
            n2 = wrap_pi(yaw - math.pi / 2)
            entry = n1 if abs(wrap_pi(heading - n1)) < abs(wrap_pi(heading - n2)) else n2
        openings = [{"z": OPENING_CENTER_Z_M}]
        if g["type"] == "stacked":
            # double-gate procedure: through the TOP opening heading away
            # from the course, U-turn, back through the BOTTOM opening
            # along the lap direction — top entry = reverse of bottom
            openings = [{"z": STACK_TOP_Z_M}, {"z": OPENING_CENTER_Z_M}]
            if entry is not None:
                openings[0]["entry_heading_rad"] = round(
                    wrap_pi(entry + math.pi), 3)
                openings[1]["entry_heading_rad"] = round(entry, 3)
        entry_out = {
            "id": gid,
            "order": g["order"],
            "xy": [round(g["xy"][0], 2), round(g["xy"][1], 2)],
            "yaw_rad": round(wrap_pi(yaw), 3),
            "entry_heading_rad": None if entry is None else round(entry, 3),
            "type": g["type"],
            "openings": openings,
            "confidence": conf,
            "bar_length_m": [round(l, 2) for l in lens],
        }
        if g["type"] == "stacked" and len(g["passes"]) > 1:
            entry_out["order_second_pass_rank_hint"] = int(g["passes"][1])
        out_gates.append(entry_out)

    # spacing plausibility
    for i in range(len(out_gates)):
        for j in range(i + 1, len(out_gates)):
            dd = math.dist(out_gates[i]["xy"], out_gates[j]["xy"])
            if dd < MIN_GATE_SPACING_M:
                warnings.append(
                    f"gates {i} and {j} only {dd:.1f} m apart (<{MIN_GATE_SPACING_M}) "
                    "— should they be the stacked pair?")

    order_ok = all(g["order"] is not None for g in out_gates)
    return {
        "frame": "ENU, origin at SW corner of footprint, +X east, +Y north",
        "source": "estimated_from_overhead_image",
        "footprint_m": [60.0, 21.0],
        "gate_geometry": {
            "outer_m": GATE_OUTER_M, "opening_m": GATE_OPENING_M,
            "depth_m": GATE_DEPTH_M,
            "opening_center_height_m": OPENING_CENTER_Z_M,
        },
        "gates": out_gates,
        "meta": {
            "source_image": source_image,
            "grid_verification": grid_res,
            "line_trace": trace_meta,
            "traversal_order_verified": (
                order_ok and not trace_meta.get("start_assumed")
                and not trace_meta.get("direction_ambiguous")),
            "direction_assumed": bool(trace_meta.get("start_assumed")
                                      or trace_meta.get("direction_ambiguous")),
            "warnings": warnings,
            "note": ("overhead-render estimate; positions are meter-scale at "
                     "best — refine with survey flight data"),
        },
    }


# =========================================================================
# overlays
# =========================================================================

def render_overlay(img, H, mapd, path_m, cones, out_path, scale=3):
    big = cv2.resize(img, (img.shape[1] * scale, img.shape[0] * scale),
                     interpolation=cv2.INTER_NEAREST)

    def to_px(pts_m):
        return (m_to_px(H, pts_m) * scale).astype(int)

    # reprojected 5 m grid
    h, w = img.shape[:2]
    corners_m = px_to_m(H, [[0, 0], [w, 0], [w, h], [0, h]])
    x0, y0 = corners_m.min(0) - 1
    x1, y1 = corners_m.max(0) + 1
    for gx in np.arange(math.floor(x0 / 5) * 5, x1, 5.0):
        p = to_px([[gx, y0], [gx, y1]])
        cv2.line(big, tuple(p[0]), tuple(p[1]), (180, 120, 180), 1)
    for gy in np.arange(math.floor(y0 / 5) * 5, y1, 5.0):
        p = to_px([[x0, gy], [x1, gy]])
        cv2.line(big, tuple(p[0]), tuple(p[1]), (180, 120, 180), 1)

    if path_m is not None:
        pp = to_px(path_m)
        cv2.polylines(big, [pp.reshape(-1, 1, 2)], False, (230, 180, 30), 2)

    for c in cones:
        p = to_px([c["xy"]])[0]
        cv2.drawMarker(big, tuple(p), (140, 140, 140), cv2.MARKER_TILTED_CROSS, 14, 2)

    for g in mapd["gates"]:
        cxy = np.array(g["xy"])
        p = to_px([cxy])[0]
        half = GATE_OUTER_M / 2
        ax = np.array([math.cos(g["yaw_rad"]), math.sin(g["yaw_rad"])])
        e = to_px([cxy - ax * half, cxy + ax * half])
        col = (0, 0, 255) if g["type"] == "stacked" else (0, 180, 0)
        cv2.line(big, tuple(e[0]), tuple(e[1]), col, 3)
        if g["entry_heading_rad"] is not None:
            hd = np.array([math.cos(g["entry_heading_rad"]),
                           math.sin(g["entry_heading_rad"])])
            a = to_px([cxy, cxy + hd * 2.5])
            cv2.arrowedLine(big, tuple(a[0]), tuple(a[1]), (255, 0, 0), 2,
                            tipLength=0.35)
        label = "?" if g["order"] is None else str(g["order"])
        if g["type"] == "stacked":
            label += "S"
        cv2.putText(big, label, (p[0] + 8, p[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
        cv2.putText(big, label, (p[0] + 8, p[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

    gr = mapd["meta"]["grid_verification"]
    txt = (f"grid {gr['status']}  rms={gr.get('rms_err_m')}m "
           f"max={gr.get('max_err_m')}m | "
           f"{sum(1 for g in mapd['gates'] if g['type'] == 'single')} singles + "
           f"{sum(1 for g in mapd['gates'] if g['type'] == 'stacked')} stacked")
    cv2.putText(big, txt, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0, 0, 0), 3)
    cv2.putText(big, txt, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    cv2.imwrite(out_path, big)


def render_seg_debug(img, shapes, line_mask, out_path, scale=3):
    dbg = img.copy()
    dbg[line_mask > 0] = (255, 160, 0)
    for s in shapes:
        p = (int(s["px"][0]), int(s["px"][1]))
        col = {"bar": (0, 200, 0), "blob": (200, 0, 200),
               "cross": (0, 0, 255), "line-piece": (230, 160, 20),
               }.get(s["kind"], (0, 220, 220))
        cv2.circle(dbg, p, 9, col, 2)
        cv2.putText(dbg, s["kind"][0].upper(), (p[0] - 4, p[1] + 4),
                    cv2.FONT_HERSHEY_PLAIN, 0.8, col, 1)
    big = cv2.resize(dbg, (img.shape[1] * scale, img.shape[0] * scale),
                     interpolation=cv2.INTER_NEAREST)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    cv2.imwrite(out_path, big)


# =========================================================================
# CLI
# =========================================================================

def click_corners(img):
    pts = []
    disp = img.copy()
    names = ["SW", "SE", "NE", "NW"]

    def cb(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(pts) < 4:
            pts.append((x, y))
            cv2.circle(disp, (x, y), 4, (0, 0, 255), -1)
            cv2.putText(disp, names[len(pts) - 1], (x + 6, y),
                        cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 255), 1)

    print("click 4 known floor points in order SW, SE, NE, NW; "
          "u=undo, q=done")
    cv2.namedWindow("corners")
    cv2.setMouseCallback("corners", cb)
    while True:
        cv2.imshow("corners", disp)
        k = cv2.waitKey(30) & 0xFF
        if k == ord("u") and pts:
            pts.pop()
            disp = img.copy()
            for i, p in enumerate(pts):
                cv2.circle(disp, p, 4, (0, 0, 255), -1)
        if k == ord("q") or len(pts) == 4:
            break
    cv2.destroyAllWindows()
    if len(pts) != 4:
        sys.exit("need exactly 4 points")
    metric = []
    for i, p in enumerate(pts):
        raw = input(f"metric X Y for {names[i]} point {p}: ").split()
        metric.append((float(raw[0]), float(raw[1])))
    return pts, metric


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("image")
    ap.add_argument("--corner", nargs=4, action="append", type=float,
                    metavar=("PX", "PY", "MX", "MY"),
                    help="pixel + metric correspondence (give 4+)")
    ap.add_argument("--click", action="store_true")
    ap.add_argument("--grid-pitch", type=float, default=5.0)
    ap.add_argument("--out", default=os.path.join("data", "course_map.json"))
    ap.add_argument("--overlay",
                    default=os.path.join("out", "course_map_overlay.png"))
    ap.add_argument("--debug-seg",
                    default=os.path.join("out", "segmentation_debug.png"))
    ap.add_argument("--start-px", nargs=2, type=float, default=None,
                    help="pixel near the traversal START (resolves direction)")
    ap.add_argument("--roi", nargs=4, type=float, default=None,
                    metavar=("X0", "Y0", "X1", "Y1"),
                    help="metric region containing the course; shapes "
                         "outside are ignored (UI icons, legend art)")
    ap.add_argument("--reverse", action="store_true",
                    help="flip traced traversal direction")
    ap.add_argument("--sat-thr", type=int, default=60)
    ap.add_argument("--val-thr", type=int, default=60)
    ap.add_argument("--force", action="store_true",
                    help="emit even if grid verification fails (marked)")
    ap.add_argument("--no-write", action="store_true")
    args = ap.parse_args(argv)

    img = cv2.imread(args.image)
    if img is None:
        sys.exit(f"cannot read {args.image}")

    if args.click:
        px, mm = click_corners(img)
    elif args.corner and len(args.corner) >= 4:
        px = [(c[0], c[1]) for c in args.corner]
        mm = [(c[2], c[3]) for c in args.corner]
    else:
        sys.exit("give 4 --corner PX PY MX MY, or --click")

    H = compute_homography(px, mm)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    all_shapes, line_mask, fg = segment(img, H, args.sat_thr, args.val_thr)
    shapes = all_shapes
    n_roi_dropped = 0
    if args.roi:
        rx0, ry0, rx1, ry1 = args.roi
        shapes = [s for s in all_shapes
                  if rx0 <= s["xy"][0] <= rx1 and ry0 <= s["xy"][1] <= ry1]
        n_roi_dropped = len(all_shapes) - len(shapes)
        if n_roi_dropped:
            print(f"roi: ignored {n_roi_dropped} shapes outside "
                  f"[{rx0},{ry0}]..[{rx1},{ry1}]")
    grid_res = verify_grid(gray, fg, H, pitch=args.grid_pitch)
    print(f"grid: {grid_res['status']}  "
          f"rms={grid_res.get('rms_err_m')} m  max={grid_res.get('max_err_m')} m  "
          f"lines x/y={grid_res['n_lines_x']}/{grid_res['n_lines_y']}  "
          f"outliers at {grid_res.get('outliers_m')}")
    if not grid_res["ok"] and not args.force:
        render_seg_debug(img, all_shapes, line_mask, args.debug_seg)
        sys.exit("\n*** HOMOGRAPHY NOT VERIFIED — refusing to emit a map. ***\n"
                 "Grid lines do not land on the expected pitch. Re-check the\n"
                 "corner correspondences (seg debug written for inspection),\n"
                 "or --force to emit a map marked unverified.")

    gates = group_gates(shapes)
    cones = [s for s in shapes if s["kind"] == "blob"]
    unknowns = [s for s in shapes if s["kind"] == "unknown"]

    path_m, trace_meta = trace_line(line_mask, H, args.start_px)
    if args.reverse and path_m is not None:
        trace_meta["reversed"] = True
    assign_order(gates, path_m, reverse=args.reverse)

    warnings = []
    if not grid_res["ok"]:
        warnings.append("grid verification FAILED/inconclusive — emitted "
                        "under --force; every value is unverified")
    if unknowns:
        warnings.append(f"{len(unknowns)} unclassified shapes (see debug)")
    if path_m is None:
        warnings.append(f"racing-line trace failed: {trace_meta.get('fail')} "
                        "— traversal order NOT emitted")
    elif trace_meta.get("start_assumed") or trace_meta.get("direction_ambiguous"):
        warnings.append("traversal DIRECTION is a guess (line start/finish "
                        "is ambiguous) — check overlay arrows, re-run with "
                        "--reverse if wrong")

    stacked = [g for g in gates if g["type"] == "stacked"]
    if len(stacked) != 1:
        warnings.append(f"expected exactly 1 stacked gate, found {len(stacked)}")

    mapd = build_map(gates, grid_res, trace_meta, os.path.basename(args.image),
                     warnings)

    n_single = sum(1 for g in mapd["gates"] if g["type"] == "single")
    n_stacked = len(stacked)
    print(f"\ngates: {len(mapd['gates'])} locations = {n_single} singles + "
          f"{n_stacked} stacked ({n_single + 2 * n_stacked} physical gates)")
    for g in sorted(mapd["gates"],
                    key=lambda g: (g["order"] is None, g["order"] or 0)):
        hd = (f"{math.degrees(g['entry_heading_rad']):+6.1f} deg"
              if g["entry_heading_rad"] is not None else "   ?  ")
        print(f"  order={g['order'] if g['order'] is not None else '?':>2} "
              f"id={g['id']:2d} ({g['xy'][0]:5.1f},{g['xy'][1]:5.1f})  "
              f"yaw={math.degrees(g['yaw_rad']):+6.1f}  entry={hd}  "
              f"{g['type']:7s} conf={g['confidence']}")
    for w in warnings:
        print(f"  WARN: {w}")

    if n_roi_dropped:
        mapd["meta"]["roi"] = args.roi
        mapd["meta"]["roi_dropped_shapes"] = n_roi_dropped
    # course furniture: cone/marker centroids (excluded from gates, but
    # downstream sims place them as obstacles)
    mapd["meta"]["cones_xy"] = [
        [round(s["xy"][0], 2), round(s["xy"][1], 2)] for s in cones]
    render_seg_debug(img, all_shapes, line_mask, args.debug_seg)
    render_overlay(img, H, mapd, path_m, cones, args.overlay)
    print(f"\noverlay -> {args.overlay}\nseg dbg -> {args.debug_seg}")

    if not args.no_write:
        os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
        with open(args.out, "w") as f:
            json.dump(mapd, f, indent=1)
        print(f"map     -> {args.out}")
    return mapd


if __name__ == "__main__":
    main()
