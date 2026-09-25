#!/usr/bin/env python3
"""curvedb v2, step 3 -- build the road table from extracted qlog tracks.

  tracks (v2_extract.py) -> per route: fuse -> resample every step_m -> passes -> anchors (+1 obs/pass)

Scope (owner 2026-09-24, replacing the I-5/I-405 corridor + home-box rule of 09-23; data-driven, no
geography in the code) -- a NEW anchor is only created at a point that is:
  * on a way mapd classes motorway, trunk or primary (links/ramps and an unknown class are separate
    classes, so they never create anchors -- the same classes the car's live gate refuses);
  * driven at >= SCOPE_MIN_V_MS on that pass (a city arterial crawled at 25 mph does not seed a table).
Authority is unchanged (roadtable.row_verdict): >= 2 distinct PT dates, never Tesla-only evidence, no ramp
in the extent, a known class, dates that agree. A road driven once gets anchors but no row.
Anchors are SEEDED over every pass first and passes ATTACHED second (roadtable.seed_anchors): MEASURED, no
point before 2026-08-15 carries a highwayClass, so a class scope would otherwise drop those dates' passes
wherever a later date creates the anchor.
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

SCOPE_CLASSES = ("motorway", "trunk", "primary")   # mapd highwayClass values that may seed anchors
SCOPE_MIN_V_MS = 40 * 0.44704                      # owner 2026-09-24: "driven at >= 40 mph on the pass"


def scope_class(hwy: str) -> bool:
  """Is this a class the table covers? Links and unknown are not in the set (mirrors the live gate)."""
  return hwy in SCOPE_CLASSES


def in_scope(pt) -> bool:
  """May this point CREATE an anchor: a scope class, driven at >= SCOPE_MIN_V_MS."""
  return scope_class(pt.hwy) and pt.v >= SCOPE_MIN_V_MS


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
  passes = []
  for _t0, route, car, samples in loaded:
    tally[f"route loaded ({'tesla' if car.startswith(rt.TESLA_PREFIX) else car})"] += 1
    for pts in rt.resample(samples, params, tally=tally):
      passes.append((route, car, pts))
      tally["anchors seeded"] += rt.seed_anchors(index, pts, in_scope=in_scope)
  for route, car, pts in passes:     # phase 2: attach only -- every anchor already exists
    rt.add_pass(index, pts, date=v2_io.pt_date(pts[0].t), drive=route, car=car, in_scope=lambda _pt: False,
                tally=tally)
  return index


def dump(index: rt.AnchorIndex, params: rt.V2Params, out: str, tally: Counter) -> list[str]:
  os.makedirs(out, exist_ok=True)
  doc = {"params": asdict(params), "anchors": [
    {"lat": a.lat, "lon": a.lon, "brg": a.brg, "obs": [asdict(o) for o in a.obs]} for a in index.anchors]}
  with gzip.open(os.path.join(out, "table.json.gz"), "wt") as f:
    json.dump(doc, f, separators=(",", ":"))
  rows, lines = [], []
  reasons, by_class = Counter(), Counter()
  mode_c, car_dates = Counter(), Counter()
  for a in index.anchors:
    cls = rt.majority([o.hwy for o in a.obs if o.hwy not in rt.UNKNOWN_CLASSES], "unknown")
    by_class[(cls, "anchors")] += 1
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
                     "spl": v.spl})
        car_dates["rows with a tesla pass"] += any(o.car.startswith(rt.TESLA_PREFIX) for o in a.obs)
    by_class[(cls, ">=2 dates")] += best.n_dates >= 2
    by_class[(cls, ">=3 dates")] += best.n_dates >= 3
    by_class[(cls, "authority")] += granted_any
  with open(os.path.join(out, "rows.json"), "w") as f:
    json.dump({"params": asdict(params), "rows": rows}, f, separators=(",", ":"))
  lines.append(f"anchors: {len(index.anchors)}   (anchor, branch) rows with authority (all dates): {len(rows)}")
  lines.append("anchors by majority class:")
  for cls in sorted({c for c, _ in by_class}, key=lambda c: -by_class[(c, "anchors")]):
    lines.append("  " + cls + ": " + ", ".join(f"{k} {by_class[(cls, k)]}" for k in
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
