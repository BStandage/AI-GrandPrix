"""Build site/src/data.json from the repo: every hardware flight's trace and
summary from flightlogs/, the narration's key moments from the .log files, and
the week's git activity. Run from anywhere:

    python site/scripts/extract.py
"""
import csv
import json
import os
import re
import subprocess
from collections import OrderedDict, defaultdict
from datetime import datetime, timedelta, timezone

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
OUT = os.path.join(ROOT, "site", "src", "data.json")
PACIFIC = timezone(timedelta(hours=-7))

# What each flight was, from the debriefs and the ledger. Keyed by log stamp.
FLIGHTS = OrderedDict([
    ("20260921_172033", dict(aircraft="D44", session="Track session 2", result="Vertical runaway", note="Camera tilted 20 deg up could not see a gate at its own height; the clipped ring's false centre said 'climb' the whole flight.")),
    ("20260921_172846", dict(aircraft="D44", session="Track session 2", result="Held centre 4 s, then climbed", note="Clipped centre rebuilt from the width. Climbed once the gate overfilled the frame.")),
    ("20260921_173201", dict(aircraft="D44", session="Track session 2", result="Altitude limit cycle", note="Airborne flag flickered; bounced off the ground and reclimbed.")),
    ("20260921_173440", dict(aircraft="D44", session="Track session 2", result="Top bar at 2.5 m", note="Best flight of the week to that point: tracked the line from 7.4 m to 2.5 m with 0.22 m cross-track.")),
    ("20260922_010031", dict(aircraft="D43", session="Night before", result="Hovered, never released", note="Release compared 3D distance to a plan point at 0.23 m altitude; under vision that never closes.")),
    ("20260922_010056", dict(aircraft="D43", session="Night before", result="Top bar", note="The plan's climb feedforward kept running after commit.")),
    ("20260922_010201", dict(aircraft="D43", session="Night before", result="Right edge", note="Phantom velocity from an uncorrected accelerometer bias; a bad position fix at 2 m lunged him sideways.")),
    ("20260922_010604", dict(aircraft="D43", session="Night before", result="Right edge", note="Same mechanism; wobbled on the lateral, never committed cleanly.")),
    ("20260922_184859", dict(aircraft="D43", session="The slot", attempt=1, result="Way high", note="Accelerometer vertical speed drifted 0.3 m/s every second; the damping term fought the elevation and won.")),
    ("20260922_185233", dict(aircraft="D43", session="The slot", attempt=2, result="High left corner", note="Size commit fired half a metre low on a clipped ring with no elevation; the hold latched a climbing throttle.")),
    ("20260922_190008", dict(aircraft="D43", session="The slot", attempt=3, result="Top bar", note="Commanded down for four seconds and never descended: configured hover 1228, real hover 1205.")),
    ("20260922_190320", dict(aircraft="D43", session="The slot", attempt=4, result="Blind, high", note="Something red 1.8 m ahead on the pad read as g0 and committed at t=0.")),
    ("20260922_190910", dict(aircraft="D43", session="The slot", attempt=5, result="Height perfect, lateral lurch", note="Start hold tried to fly to a plan point 0.9 m away; hard roll both ways; pilot took over at 4 s.")),
    ("20260922_191550", dict(aircraft="D44", session="The slot", attempt=7, result="Never moved", note="Start hold captured its point with three fixes, the estimate moved 0.8 m, held 20 s drifting backwards.")),
])
# Attempt 6 (perfect lateral, top bar) was not pulled off the aircraft before
# the slot ended; its numbers come from the console and live in the prose.


def f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def load_flight(stamp, day_dir):
    csv_path = os.path.join(ROOT, "flightlogs", day_dir, f"hw_follower_{stamp}.csv")
    log_path = os.path.join(ROOT, "flightlogs", day_dir, f"hw_follower_{stamp}.log")
    if not os.path.exists(csv_path):
        return None
    rows = list(csv.DictReader(open(csv_path, encoding="utf-8", errors="replace")))
    if not rows:
        return None
    cols = rows[0].keys()
    zc = "z"
    rc = "det_range" if "det_range" in cols else None
    ec = "el_deg" if "el_deg" in cols else None
    fc = "fixes" if "fixes" in cols else None
    series = []
    last_t = -1.0
    for r in rows:
        t = f(r.get("t"))
        if t is None or t - last_t < 0.45:
            continue
        last_t = t
        series.append({
            "t": round(t, 1),
            "x": round(f(r.get("x")) or 0.0, 2),
            "y": round(f(r.get("y")) or 0.0, 2),
            "z": round(f(r.get(zc)) or 0.0, 2),
            "r": (round(f(r.get(rc)), 1) if rc and f(r.get(rc)) is not None else None),
            "el": (round(f(r.get(ec)), 1) if ec and f(r.get(ec)) is not None else None),
        })
    zs = [f(r.get(zc)) for r in rows if f(r.get(zc)) is not None]
    rs = [f(r.get(rc)) for r in rows if rc and f(r.get(rc)) is not None and f(r.get(rc)) > 0.5]
    fixes = max((f(r.get(fc)) or 0) for r in rows) if fc else None
    duration = f(rows[-1].get("t")) or 0.0
    commit_t = None
    events = []
    if os.path.exists(log_path):
        for line in open(log_path, encoding="utf-8", errors="replace"):
            m = re.match(r"\s*([0-9.]+)\s+(.*)", line)
            if not m:
                continue
            t, msg = float(m.group(1)), m.group(2).strip()
            if "COMMIT" in msg and commit_t is None:
                commit_t = t
            if any(k in msg for k in ("COMMIT", "CROSSED", "AIRBORNE", "MISSED", "IN SIGHT at")) and len(events) < 8:
                events.append({"t": t, "msg": msg[:110]})
    dt = datetime.strptime(stamp, "%Y%m%d_%H%M%S").replace(tzinfo=timezone.utc).astimezone(PACIFIC)
    meta = FLIGHTS.get(stamp, {})
    return {
        "stamp": stamp,
        "local": dt.strftime("%a %d %b, %H:%M"),
        "aircraft": meta.get("aircraft", "?"),
        "session": meta.get("session", ""),
        "attempt": meta.get("attempt"),
        "result": meta.get("result", ""),
        "note": meta.get("note", ""),
        "duration_s": round(duration, 1),
        "max_z": round(max(zs), 2) if zs else None,
        "min_range": round(min(rs), 1) if rs else None,
        "fixes": int(fixes) if fixes is not None else None,
        "commit_t": commit_t,
        "events": events,
        "series": series,
    }


def flights():
    out = []
    for day in ("2026-09-21", "2026-09-22"):
        d = os.path.join(ROOT, "flightlogs", day)
        if not os.path.isdir(d):
            continue
        for name in sorted(os.listdir(d)):
            m = re.match(r"hw_follower_(\d{8}_\d{6})\.csv$", name)
            if m and m.group(1) in FLIGHTS:
                fl = load_flight(m.group(1), day)
                if fl:
                    out.append(fl)
    out.sort(key=lambda x: x["stamp"])
    return out


def git_week():
    """Commits and lines per day over the event, from main."""
    log = subprocess.run(
        ["git", "log", "main", "--no-merges", "--since=2026-09-15", "--until=2026-09-24",
         "--format=%H|%ad|%s", "--date=format:%Y-%m-%d", "--numstat"],
        cwd=ROOT, capture_output=True, text=True).stdout
    days = defaultdict(lambda: {"commits": 0, "added": 0, "removed": 0, "subjects": []})
    cur = None
    for line in log.splitlines():
        if "|" in line and re.match(r"^[0-9a-f]{40}\|", line):
            _, day, subj = line.split("|", 2)
            cur = day
            days[cur]["commits"] += 1
            if len(days[cur]["subjects"]) < 6:
                days[cur]["subjects"].append(subj[:90])
        elif cur and re.match(r"^\d+\t\d+\t", line):
            a, r, _ = line.split("\t", 2)
            days[cur]["added"] += int(a)
            days[cur]["removed"] += int(r)
    return [dict(day=d, **v) for d, v in sorted(days.items())]


def authorship():
    out = subprocess.run(["git", "shortlog", "-sn", "--no-merges", "main"], cwd=ROOT,
                         capture_output=True, text=True).stdout
    rows = []
    for line in out.splitlines():
        m = re.match(r"\s*(\d+)\s+(.*)", line)
        if m:
            rows.append({"commits": int(m.group(1)), "name": m.group(2)})
    return rows


def course():
    """Gates and the planned path in the flight frame (start at the origin,
    +y down gate 0's line), from the STACK plan."""
    p = json.load(open(os.path.join(ROOT, "out", "plans", "plan_STACK_s15_cam20_75.json")))
    gates = [{"label": e["label"], "x": e["x"], "y": e["y"], "z": e["z"], "heading": e["heading_rad"]}
             for e in p["events"] if e.get("lap", 0) == 0]
    # the dense path: the first list of x/y records that is not the events
    path = []
    for key, v in p.items():
        if key == "events" or not (isinstance(v, list) and len(v) > 50):
            continue
        q0 = v[0]
        if isinstance(q0, dict) and "x" in q0 and "y" in q0:
            path = [[round(q["x"], 2), round(q["y"], 2)] for q in v[::2]]
        elif isinstance(q0, (list, tuple)) and len(q0) >= 4:
            # plan samples are [s, x, y, z, vx, vy, vz, ax, ay, az]
            path = [[round(q[1], 2), round(q[2], 2)] for q in v[::2]]
        if path:
            break
    return {"gates": gates, "path": path, "path_key": key if path else None}


def main():
    data = {
        "course": course(),
        "generated": datetime.now(PACIFIC).strftime("%Y-%m-%d"),
        "flights": flights(),
        "git_week": git_week(),
        "authorship": authorship(),
    }
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as fh:
        json.dump(data, fh, indent=1)
    print(f"{OUT}: {len(data['flights'])} flights, {len(data['git_week'])} days of commits, "
          f"{os.path.getsize(OUT) // 1024} KB")


if __name__ == "__main__":
    main()
