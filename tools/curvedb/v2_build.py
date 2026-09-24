#!/usr/bin/env python3
"""curvedb v2, step 3 -- build the road table from extracted qlog tracks.

  tracks (v2_extract.py) -> per route: fuse -> resample every step_m -> passes -> anchors (+1 obs/pass)

Scope (owner, 2026-09-23: "repeat roads first") -- a NEW anchor is only created at an in-scope point:
  * the I-5 corridor Seattle-Corvallis: mapd's wayRef or road name has an `I 5` or `I 405` token and
    44.3 <= lat <= 48.0 (Portland's I-405 loop is on the test set; WA I-405 is inside the latitude band
    and is kept -- it is a repeat road in the Seattle area);
  * the Seattle home area: SEATTLE_BOX below (the city, SR 99 tunnel approach included).
Passes of either car, any steering mode, are attached (owner decision). Refusals are counted by reason.

Outputs (in --out):
  table.json.gz  every anchor and every admitted pass -- the input to v2_replay.py (LODO is a filter)
  rows.json      the (anchor, branch) rows that have AUTHORITY on all dates: the file a car would load
  build.txt      counts, coverage, refusal reasons

Run:
  PYTHONPATH=<worktree> python tools/curvedb/v2_build.py --tracks <dir> --out <dir>
"""
from __future__ import annotations

import argparse
import gzip
import json
import os
import sys
from collections import Counter
from dataclasses import asdict

from openpilot.tools.curvedb import roadtable as rt
from openpilot.tools.curvedb import v2_io

# PROVISIONAL: Seattle city limits, rounded outward (lat_min, lat_max, lon_min, lon_max).
SEATTLE_BOX = (47.49, 47.74, -122.44, -122.24)
CORRIDOR_ROADS = ("I 5", "I 405")
CORRIDOR_LAT = (44.3, 48.0)


def region(lat: float, lon: float, road: str) -> str | None:
  """'i5' | 'home' | None. Corridor wins where both apply (I-5 through Seattle)."""
  toks = {x.strip() for x in (road or "").replace("|", ";").split(";")}
  if CORRIDOR_LAT[0] <= lat <= CORRIDOR_LAT[1] and toks & set(CORRIDOR_ROADS):
    return "i5"
  if SEATTLE_BOX[0] <= lat <= SEATTLE_BOX[1] and SEATTLE_BOX[2] <= lon <= SEATTLE_BOX[3]:
    return "home"
  return None


def in_scope(pt) -> bool:
  return region(pt.lat, pt.lon, pt.road) is not None


def car_of(fp) -> str:
  return fp or "unknown"


def build(tracks: str, params: rt.V2Params, tally: Counter):
  index = rt.AnchorIndex(params)
  routes = v2_io.list_routes(tracks, "qlog")
  loaded = []
  for route, files in routes.items():
    doc = v2_io.load_route(files, tally)
    if doc is None:
      continue
    samples = rt.fuse(doc, doc["off_s"], params)
    if not samples:
      tally["route: no samples (no pose/carState)"] += 1
      continue
    loaded.append((samples[0].t, route, car_of(doc["fp"]), samples))
  loaded.sort()            # canonical order: anchors are an accident of nothing
  for _t0, route, car, samples in loaded:
    tally[f"route loaded ({'tesla' if car.startswith(rt.TESLA_PREFIX) else car})"] += 1
    for pts in rt.resample(samples, params, tally=tally):
      rt.add_pass(index, pts, date=v2_io.pt_date(pts[0].t), drive=route, car=car, in_scope=in_scope,
                  tally=tally)
  return index


def dump(index: rt.AnchorIndex, params: rt.V2Params, out: str, tally: Counter) -> list[str]:
  os.makedirs(out, exist_ok=True)
  doc = {"params": asdict(params), "anchors": [
    {"lat": a.lat, "lon": a.lon, "brg": a.brg, "obs": [asdict(o) for o in a.obs]} for a in index.anchors]}
  with gzip.open(os.path.join(out, "table.json.gz"), "wt") as f:
    json.dump(doc, f, separators=(",", ":"))
  rows, lines = [], []
  reasons, by_region = Counter(), Counter()
  mode_c, car_dates = Counter(), Counter()
  for a in index.anchors:
    road = rt.majority([o.road for o in a.obs], "")
    reg = region(a.lat, a.lon, road) or "out"
    by_region[(reg, "anchors")] += 1
    for o in a.obs:
      mode_c[o.mode] += 1
    brs = rt.branches(a, params.branch_radius_m)
    car_dates[f"anchors with {min(len(brs), 3)}{'+' if len(brs) >= 3 else ''} branch(es)"] += 1
    if not brs:
      reasons["no pass with an extent end"] += 1
      continue
    best, granted_any = None, False
    for br in brs:
      v = rt.row_verdict(a, params, branch=br)
      reasons[v.reason if v.granted else v.reason.split(" (")[0]] += 1
      best = v if best is None or v.n_dates > best.n_dates else best
      if v.granted:
        granted_any = True
        rows.append({"lat": round(a.lat, 6), "lon": round(a.lon, 6), "brg": round(a.brg, 1),
                     "end_lat": round(br[0], 6), "end_lon": round(br[1], 6),
                     "k": round(v.k, 6), "n_dates": v.n_dates, "n_passes": v.n_passes, "hwy": v.hwy,
                     "spl": v.spl, "region": reg})
        car_dates["rows with a tesla pass"] += any(o.car.startswith(rt.TESLA_PREFIX) for o in a.obs)
    by_region[(reg, ">=2 dates")] += best.n_dates >= 2
    by_region[(reg, ">=3 dates")] += best.n_dates >= 3
    by_region[(reg, "authority")] += granted_any
  with open(os.path.join(out, "rows.json"), "w") as f:
    json.dump({"params": asdict(params), "rows": rows}, f, separators=(",", ":"))
  lines.append(f"anchors: {len(index.anchors)}   (anchor, branch) rows with authority (all dates): {len(rows)}")
  lines.append("by region:")
  for reg in ("i5", "home", "out"):
    lines.append("  " + reg + ": " + ", ".join(f"{k} {by_region[(reg, k)]}" for k in
                                               ("anchors", ">=2 dates", ">=3 dates", "authority")))
  lines.append("(anchor, branch) verdicts (all dates): " + ", ".join(f"{k} {v}" for k, v in reasons.most_common()))
  lines.append("pass modes admitted: " + ", ".join(f"{k} {v}" for k, v in mode_c.most_common()))
  lines.append(", ".join(f"{k} {v}" for k, v in car_dates.items()))
  lines.append("tally: " + ", ".join(f"{k} {v}" for k, v in sorted(tally.items())))
  with open(os.path.join(out, "build.txt"), "w") as f:
    f.write("\n".join(lines) + "\n")
  return lines


def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--tracks", required=True)
  ap.add_argument("--out", required=True)
  a = ap.parse_args(argv)
  tally: Counter = Counter()
  index = build(a.tracks, rt.PROVISIONAL_V2, tally)
  if not index.anchors:
    print("ZERO anchors built -- the scope or the tracks are wrong; refusing to write an empty table")
    return 2
  print("\n".join(dump(index, rt.PROVISIONAL_V2, a.out, tally)))
  return 0


if __name__ == "__main__":
  sys.exit(main())
