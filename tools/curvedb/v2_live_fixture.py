#!/usr/bin/env python3
"""curvedb v2 LIVE -- build the PRIVATE replay fixture the car-side replay test runs on.

For every episode the offline replay ADJUDICATED (v2_replay.py's `--out` rows, status "adjudicated") and
every real ADD it found, this writes:

  * the episode's OWN pass (the truck's resampled track, +-800 m around the site) -- standing in for mapd's
    path, so the car's keying runs on the geometry the offline query ran on;
  * the table exported LEAVE-ONE-DATE-OUT on the episode's date (v2_live_export.export_doc), cut to the
    anchors within 1.5 km -- the same rows the offline replay judged the episode with;
  * the episode's numbers (ref, ICBM's target, posted, k_truth, the offline k_row and outcome).

The fixture contains the owner's driven positions: it is written to the private scratch area and must
never be committed to pnw-pilot. The test that reads it (selfdrive/controls/lib/ces_pnw/tests/
test_curvedblive2pnw_replay.py) skips, saying so, where it is absent.

Run:
  PYTHONPATH=<worktree> python tools/curvedb/v2_live_fixture.py --table <v2>/db/table.json.gz \\
      --tracks <v2>/tracks --episodes <v2>/arch_eps.jsonl <more eps> \\
      --replay <v2>/replay_proposed.A2.2.jsonl --adds <v2>/replay_proposed.A2.2.adds.jsonl --out <fixture.json.gz>
"""
from __future__ import annotations

import argparse
import gzip
import json
import sys

from openpilot.tools.curvedb import roadtable as rt
from openpilot.tools.curvedb.store import haversine_m
from openpilot.tools.curvedb.v2_live_export import export_doc
from openpilot.tools.curvedb.v2_replay import Episode, load_episodes, load_passes, load_table, own_point

WINDOW_M = 800.0
ROW_RADIUS_M = 1500.0


def cut(pts, i, window_m=WINDOW_M):
  s0 = pts[i].s
  return [[p.lat, p.lon] for p in pts if abs(p.s - s0) <= window_m]


def local_rows(idx, exclude_date, lat, lon):
  doc, _ = export_doc(idx, exclude_date)
  doc["anchors"] = [a for a in doc["anchors"] if haversine_m(lat, lon, a[0], a[1]) <= ROW_RADIUS_M]
  return doc


def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--table", required=True)
  ap.add_argument("--tracks", required=True)
  ap.add_argument("--episodes", nargs="+", required=True)
  ap.add_argument("--replay", required=True, help="v2_replay.py --out rows for ONE A (the adjudicated set)")
  ap.add_argument("--adds", required=True, help="the matching .adds.jsonl")
  ap.add_argument("--out", required=True)
  a = ap.parse_args(argv)
  idx = load_table(a.table)
  eps, _ = load_episodes(a.episodes)
  by_t = {round(e.t, 1): e for e in eps}
  adj = [json.loads(ln) for ln in open(a.replay)]
  adj = [r for r in adj if r.get("status") == "adjudicated"]
  adds = [json.loads(ln) for ln in open(a.adds)]
  adds = [x for x in adds if x.get("cls", "").startswith("real")]
  if not adj or not adds:
    print(f"nothing to write: {len(adj)} adjudicated, {len(adds)} real adds")
    return 2
  ranges = [(r["t"] - 30, r["t"] + 240) for r in adj]
  passes = load_passes(a.tracks, idx.p, ranges)   # the real add (09-21 Tumwater) is on an episode route
  out = {"episodes": [], "adds": []}
  for r in adj:
    ep: Episode = by_t[round(r["t"], 1)]
    own = own_point(passes, ep)
    if isinstance(own, str):
      print(f"{r['pt']}: {own}")
      return 2
    _d, pts, i, _route = own
    out["episodes"].append({
      "pt": r["pt"], "date": ep.date, "site": list(ep.site), "ref": ep.ref, "icbm": ep.icbm, "posted": ep.posted,
      "k_truth": r["k_truth"], "k_row_offline": r["k_row"], "cls": r["cls"], "outcome_offline": r["outcome"],
      "target_mph_offline": r["target_mph"], "path": cut(pts, i),
      "rows": local_rows(idx, ep.date, ep.site[0], ep.site[1])})
  for x in adds:
    best = None
    for _t0, _t1, route, _car, pts in passes:
      if route != x["route"]:
        continue
      for i, p in enumerate(pts):
        d = haversine_m(p.lat, p.lon, x["lat"], x["lon"])
        if d < 5.0 and (best is None or d < best[0]):
          best = (d, pts, i)
    if best is None:
      print(f"add {x['pt']}: its pass is not in the tracks")
      return 2
    _d, pts, i = best
    before = [[p.lat, p.lon] for p in pts if pts[i].s - WINDOW_M <= p.s <= pts[i].s + WINDOW_M]
    out["adds"].append({"pt": x["pt"], "date": x["date"], "site": [x["lat"], x["lon"]], "v_app_mph": x["v_app_mph"],
                        "k_row_offline": x["k_row"], "k_truth": x["k_truth"], "v_db_mph_offline": x["v_db_mph"],
                        "path": before, "rows": local_rows(idx, x["date"], x["lat"], x["lon"])})
  out["params"] = {k: getattr(idx.p, k) for k in ("site_radius_m", "heading_tol_deg", "extent_back_m",
                                                  "extent_fwd_m", "branch_radius_m")}
  assert isinstance(idx.p, rt.V2Params)
  with gzip.open(a.out, "wt") as f:
    json.dump(out, f)
  print(f"wrote {a.out}: {len(out['episodes'])} episodes, {len(out['adds'])} adds")
  return 0


if __name__ == "__main__":
  sys.exit(main())
