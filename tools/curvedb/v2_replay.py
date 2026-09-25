#!/usr/bin/env python3
"""curvedb v2, step 4 -- replay the road table against every ICBM slowdown episode, LEAVE-ONE-DATE-OUT.

For each episode (ingest.py's episode records: ces_events 2026-09-16..23 archive + the drives/ corpora):

  1. the episode's OWN pass is found in the qlog tracks (it is on the held-out date, so it never builds
     a row that judges it). Its measured curvature over the site's extent is the TRUTH, k_truth.
  2. the row at the episode's candidate site (heading = the own pass's heading there) is evaluated with
     the episode's PT date excluded. `roadtable.row_verdict` applies the owner's rules.
  3. v2's target = the curve DB's curvature with MAPD'S lateral target: v_db = sqrt(A_mapd / k_row).
     Owner rule 2026-09-23: a measurement may override the map in BOTH directions. Lowering is free
     (safe direction); raising is bounded -- never above ICBM + RAISE_CAP, never above posted + margin,
     and never above the driver's own reference speed (store.cancel_target's asymmetry).
  4. the verdict uses the owner's cut-offs on a@ref = k_truth * ref^2 (the counterfactual, I1):
     UNWANTED <= 2.2 < MARGINAL < 2.8 <= WANTED, and the design's REAL = a@ref >= 2.5 m/s^2.

SAFETY LIST: every REAL episode where v2's target is above ICBM's and the truth curvature at v2's target
is >= 2.5 m/s^2. That list must be empty before any authority is discussed.

Run:
  PYTHONPATH=<worktree> python tools/curvedb/v2_replay.py --table <out>/table.json.gz --tracks <dir> \\
      --episodes eps1.jsonl [eps2.jsonl ...] [--a-mapd 2.2] [--adds]
"""
from __future__ import annotations

import argparse
import bisect
import gzip
import json
import math
import sys
from collections import Counter
from dataclasses import dataclass

from openpilot.tools.curvedb import roadtable as rt
from openpilot.tools.curvedb import v2_io
from openpilot.tools.curvedb.store import haversine_m
from openpilot.tools.curvedb.v2_build import scope_class

MPH = 0.44704
UNWANTED_MAX = 2.2     # owner's own cut (09-21 drive report s3): a@set <= 2.2 is unwanted
WANTED_MIN = 2.8       # owner's own cut: a@set >= 2.8 is wanted
REAL_MIN = 2.5         # CURVEDB2PNW.md s3.9-6 / R4: "real" at the counterfactual speed
MIN_RED_MS = 1.0       # v1 MIN_REDUCTION_MS: below this a "slowdown" is SET-button rounding
ACT_EPS_MS = 0.5       # a target change smaller than this is not a change
RAISE_CAP_MS = 15 * MPH      # PROVISIONAL: P1's "at most +15 mph above mapd in the first cut"
POSTED_MARGIN_MS = 10 * MPH  # PROVISIONAL: P1's "never above the posted limit + margin"
PASSAGE_MAX_M = 80.0   # v1: how close the own pass must come to the site


def v2_target(*, k_row: float, a_mapd: float, icbm_ms: float, ref_ms: float, posted_ms,
              raise_cap_ms: float = RAISE_CAP_MS, posted_margin_ms: float = POSTED_MARGIN_MS,
              raise_margin: float = 1.0) -> float:
  """The speed ICBM would have aimed for with the DB. Pure; see the module docstring for the rules.

  `raise_margin` (>= 1) inflates the row's curvature ONLY when the DB would raise ICBM's target -- the
  unsafe direction. A lowering (the DB adds slowing) uses the row as measured: a margin there would make
  the DB a phantom source of its own (MEASURED: 9 unwanted adds at 1.25x)."""
  if not (k_row > 0 and a_mapd > 0 and raise_margin >= 1.0 and math.isfinite(icbm_ms) and math.isfinite(ref_ms)):
    raise ValueError(f"v2_target: bad inputs k={k_row} a={a_mapd} m={raise_margin} icbm={icbm_ms} ref={ref_ms}")
  v = math.sqrt(a_mapd / k_row)
  if v > icbm_ms:                                   # a RAISE: margin'd, then bounded
    v = max(icbm_ms, math.sqrt(a_mapd / (k_row * raise_margin)))
    cap = icbm_ms + raise_cap_ms
    if posted_ms is not None and posted_ms > 0:
      cap = min(cap, posted_ms + posted_margin_ms)
    v = max(icbm_ms, min(v, cap))
  return min(v, ref_ms)


def classify(a_ref: float) -> str:
  if a_ref <= UNWANTED_MAX:
    return "unwanted"
  if a_ref >= WANTED_MIN:
    return "wanted"
  return "marginal"


def outcome(cls: str, *, ref: float, icbm: float, target: float, a_target: float) -> str:
  """What v2 would have done to this episode."""
  raised = target > icbm + ACT_EPS_MS
  lowered = target < icbm - ACT_EPS_MS
  gone = target >= ref - MIN_RED_MS
  if cls == "unwanted":
    return "REMOVED" if gone else "reduced" if raised else "WORSENED" if lowered else "unchanged"
  if not raised:
    return "kept (stronger)" if lowered else "kept"
  if a_target >= REAL_MIN:
    return "LOST" if gone else "WEAKENED"
  return "raised, still < 2.5 at target"


@dataclass
class Episode:
  raw: dict
  date: str
  t: float
  car: str
  site: tuple
  ref: float
  icbm: float
  posted: float | None
  hwy: str | None


def load_episodes(paths) -> tuple[list[Episode], Counter]:
  tally: Counter = Counter()
  out: list[Episode] = []
  seen: list[tuple[str, float]] = []
  for p in paths:
    for line in open(p):
      e = json.loads(line)
      tally["episode records read"] += 1
      if any(c == e["car"] and abs(t - e["t"]) < 5.0 for c, t in seen):
        tally["duplicate (same car, within 5 s) dropped"] += 1
        continue
      seen.append((e["car"], e["t"]))
      if e.get("site_lat") is None or e.get("ref_ms") is None or e.get("icbm_target_ms") is None:
        tally["episode without site/ref/target dropped"] += 1
        continue
      out.append(Episode(raw=e, date=e["date"], t=e["t"], car=e["car"], site=(e["site_lat"], e["site_lon"]),
                         ref=e["ref_ms"], icbm=e["icbm_target_ms"], posted=e.get("posted_ms"),
                         hwy=e.get("highway_class")))
  out.sort(key=lambda e: e.t)
  return out, tally


def load_table(path: str) -> rt.AnchorIndex:
  with gzip.open(path, "rt") as f:
    doc = json.load(f)
  params = rt.V2Params(**doc["params"])
  idx = rt.AnchorIndex(params)
  for a in doc["anchors"]:
    ai = idx.add(a["lat"], a["lon"], a["brg"])
    idx.anchors[ai].obs = [rt.PassObs(**o) for o in a["obs"]]
  return idx


def load_passes(tracks: str, params: rt.V2Params, t_ranges) -> list[tuple[float, float, str, str, list]]:
  """(t0, t1, route, car, pts) for every pass of every route that overlaps one of t_ranges."""
  out = []
  tally: Counter = Counter()
  for route, files in v2_io.list_routes(tracks, "qlog").items():
    doc = v2_io.load_route(files, tally)
    if doc is None:
      continue
    samples = rt.fuse(doc, doc["off_s"], params)
    if not samples or not any(samples[0].t - 60 <= b and a <= samples[-1].t + 60 for a, b in t_ranges):
      continue
    for pts in rt.resample(samples, params):
      out.append((pts[0].t, pts[-1].t, route, doc["fp"] or "unknown", pts))
  out.sort(key=lambda x: x[0])
  return out


def own_point(passes, ep: Episode):
  """The own pass's point nearest the site, after the decision, within PASSAGE_MAX_M.
  Returns (d, pts, i, route) or a string saying WHY there is none (Rule 2: never a bare None)."""
  best = None
  overlap = False
  for t0, t1, route, _car, pts in passes:
    if t1 < ep.t - 5 or t0 > ep.t + 180:
      continue
    overlap = True
    for i, p in enumerate(pts):
      if p.t < ep.t - 5 or p.t > ep.t + 180:
        continue
      d = haversine_m(p.lat, p.lon, ep.site[0], ep.site[1])
      if d <= PASSAGE_MAX_M and (best is None or d < best[0]):
        best = (d, pts, i, route)
  if best is None:
    return "no own pass: track exists, none within 80 m of the site" if overlap else \
      "no own pass: no moving (>= v_min) qlog track at that time"
  return best


def replay(idx: rt.AnchorIndex, passes, eps: list[Episode], a_mapd: float, tally: Counter,
           raise_cap_ms=RAISE_CAP_MS, posted_margin_ms=POSTED_MARGIN_MS, k_scale: float = 1.0,
           raise_margin: float = 1.0) -> list[dict]:
  """`k_scale` multiplies BOTH the row and the truth curvature (the kPeak-equivalent sensitivity: v2's
  1 s-smoothed estimator reads ~0.89x kPeak on sustained clean curves). a@target = A * k_truth / k_row is
  invariant to it; the classification (a@ref) and the targets are not. `raise_margin`: see v2_target."""
  p = idx.p
  rows = []
  for ep in eps:
    r = {"date": ep.date, "pt": v2_io.pt_str(ep.t), "t": ep.t, "car": ep.car, "site": ep.site,
         "ref_mph": ep.ref / MPH, "icbm_mph": ep.icbm / MPH, "hwy": ep.hwy, "src": ep.raw.get("icbm_src"),
         "site_group": ep.raw.get("site_group")}
    own = own_point(passes, ep)
    if isinstance(own, str):
      r["status"] = own
      rows.append(r)
      continue
    d, pts, i, route = own
    pt = pts[i]
    r["route"], r["road"] = route, pt.road
    r["region"] = pt.hwy
    # a KNOWN class the table does not cover; an unknown class (pre-08-15 passes) goes on to the lookup,
    # whose class gate refuses it unless the row itself carries a class
    if not scope_class(pt.hwy) and pt.hwy not in rt.UNKNOWN_CLASSES:
      r["status"] = "out of scope"
      rows.append(r)
      continue
    k_truth, why = rt.extent_peak(pts, i, p.extent_back_m, p.extent_fwd_m)
    if k_truth is None:
      r["status"] = f"no truth ({why})"
      rows.append(r)
      continue
    k_truth *= k_scale
    a_ref = k_truth * ep.ref ** 2
    r.update(k_truth=k_truth, a_ref=a_ref, a_icbm=k_truth * ep.icbm ** 2, cls=classify(a_ref),
             real=a_ref >= REAL_MIN)
    ai, dd = idx.nearest(ep.site[0], ep.site[1], pt.brg)
    if ai is None:
      r["status"] = "no row"
      rows.append(r)
      continue
    br = rt.branch_point(pts, i, idx.anchors[ai], p.extent_fwd_m)
    v = rt.row_verdict(idx.anchors[ai], p, exclude_date=ep.date, branch=br) if br is not None else \
      rt.RowVerdict(False, None, 0, 0, "query has no branch point (own pass ends)", "", 0.0)
    r.update(k_row=v.k, n_dates=v.n_dates, row_why=v.reason)
    if v.granted and p.way_check and pt.way:
      w = rt.majority([o.way for o in idx.anchors[ai].obs if o.date != ep.date])
      if w not in (None, pt.way):
        v = rt.RowVerdict(False, v.k, v.n_dates, v.n_passes, "way mismatch at query", v.hwy, v.spl)
    hc = ep.hwy if ep.hwy is not None else v.hwy
    if v.granted and (hc in rt.UNKNOWN_CLASSES or hc in rt.RAMP_CLASSES):
      v = rt.RowVerdict(False, v.k, v.n_dates, v.n_passes, f"episode class {hc!r}", v.hwy, v.spl)
    if v.granted and not (ep.posted and ep.posted > 0):
      v = rt.RowVerdict(False, v.k, v.n_dates, v.n_passes, "no posted limit", v.hwy, v.spl)
    if not v.granted:
      r["status"] = f"no authority: {v.reason}"
      rows.append(r)
      continue
    k_row = v.k * k_scale
    r["k_row"] = k_row
    tgt = v2_target(k_row=k_row, a_mapd=a_mapd, icbm_ms=ep.icbm, ref_ms=ep.ref, posted_ms=ep.posted,
                    raise_cap_ms=raise_cap_ms, posted_margin_ms=posted_margin_ms, raise_margin=raise_margin)
    a_t = k_truth * tgt ** 2
    r.update(status="adjudicated", v_db_mph=math.sqrt(a_mapd / k_row) / MPH, target_mph=tgt / MPH, a_target=a_t,
             outcome=outcome(r["cls"], ref=ep.ref, icbm=ep.icbm, target=tgt, a_target=a_t))
    rows.append(r)
  return rows


def scan_adds(idx: rt.AnchorIndex, passes, eps: list[Episode], a_mapd: float, dates: set,
              in_scope) -> list[dict]:
  """Curves where v2 would ADD a slowdown ICBM never made: on Lightning passes of `dates` (which must be
  dates whose ICBM record is CONTINUOUS -- the ces_events archive -- or "ICBM did not act" is unknowable),
  every point whose row (LODO) derives v_db more than MIN_RED below the approach speed (max v over the
  previous 150 m), unless an ICBM episode was active there [t - 5 s, t_end + 10 s] with a target at or
  below v_db + MIN_RED. Consecutive hits are one event. The truth curvature at the approach speed decides
  whether the add is a real curve (>= 2.5) or a new unwanted slowdown (<= 2.2)."""
  p = idx.p
  windows = sorted((e.t - 5.0, (e.raw.get("t_end") or e.t) + 10.0, e.icbm) for e in eps)
  w_t = [w[0] for w in windows]
  events = []
  for _t0, _t1, route, car, pts in passes:
    if car.startswith(rt.TESLA_PREFIX) or car == "unknown":
      continue
    cur = None
    for i, pt in enumerate(pts):
      date = v2_io.pt_date(pt.t)
      hit = None
      if date in dates and in_scope(pt) and math.isfinite(pt.brg):
        j = bisect.bisect_right(w_t, pt.t)
        icbm_here = [w[2] for w in windows[max(0, j - 50):j] if w[0] <= pt.t <= w[1]]
        ai, _ = idx.nearest(pt.lat, pt.lon, pt.brg)
        br = rt.branch_point(pts, i, idx.anchors[ai], p.extent_fwd_m) if ai is not None else None
        if ai is not None and br is not None:
          v = rt.row_verdict(idx.anchors[ai], p, exclude_date=date, branch=br)
          v_db = math.sqrt(a_mapd / v.k) if v.granted else None       # a LOWERING: no margin
          if v_db is not None and any(x <= v_db + MIN_RED_MS for x in icbm_here):
            v_db = None                      # ICBM was already slowing at least this much here
          if v_db is not None:
            v_app = max(q.v for q in pts if pt.s - 150 <= q.s <= pt.s)
            if v_db < v_app - MIN_RED_MS:
              k_truth, _ = rt.extent_peak(pts, i, p.extent_back_m, p.extent_fwd_m)
              hit = {"pt": v2_io.pt_str(pt.t), "date": date, "route": route, "lat": pt.lat, "lon": pt.lon,
                     "road": pt.road, "v_app_mph": v_app / MPH, "v_db_mph": v_db / MPH, "k_row": v.k,
                     "icbm_active": bool(icbm_here),
                     "k_truth": k_truth, "a_app": None if k_truth is None else k_truth * v_app ** 2}
      if hit and cur and pt.s - cur["_s"] <= 200:
        if (hit["a_app"] or 0) > (cur["a_app"] or 0) or hit["v_db_mph"] < cur["v_db_mph"]:
          hit["_s"] = pt.s
          cur.update({k: hit[k] for k in hit if k not in ("pt",)})
        cur["_s"] = pt.s
      elif hit:
        cur = dict(hit, _s=pt.s)
        events.append(cur)
  for e in events:
    e.pop("_s", None)
    a = e["a_app"]
    e["cls"] = "no truth" if a is None else "real (>=2.5)" if a >= REAL_MIN else \
      "unwanted (<=2.2)" if a <= UNWANTED_MAX else "marginal"
  return events


def row_accuracy(idx: rt.AnchorIndex, *, k_min: float, a_mapd: float,
                 k_margin: float = 1.0) -> tuple[list[dict], Counter]:
  """The geometry error rate, independent of ICBM: for every pass over every anchor, build the row WITHOUT
  that pass's date (LODO), on that pass's own branch, and compare the pass against it. A held-out pass whose
  curvature at the DB's own speed sqrt(A / k_row) gives a lateral acceleration >= REAL_MIN is a case where
  the DB alone would have let the truck into that curve too fast. Only rows with k_row >= k_min (curves
  where the DB speed can bind) are scored. Returns (one dict per held-out pass, refusal tally)."""
  out, tally = [], Counter()
  for ai, a in enumerate(idx.anchors):
    for o in a.obs:
      d = o.date
      br = (o.end_lat, o.end_lon) if math.isfinite(o.end_lat) else None
      v = rt.row_verdict(a, idx.p, exclude_date=d, branch=br)
      if not v.granted:
        tally[v.reason.split(" (")[0]] += 1
        continue
      if v.k < k_min:
        tally["row k below k_min"] += 1
        continue
      ratio = o.k_ext / (v.k * k_margin)
      out.append({"anchor": ai, "date": d, "car": o.car, "mode": o.mode, "k_row": v.k, "k_pass": o.k_ext,
                  "ratio": ratio, "a_at_vdb": a_mapd * ratio, "lat": a.lat, "lon": a.lon, "road": o.road,
                  "n_dates": v.n_dates})
  return out, tally


def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--table", required=True)
  ap.add_argument("--tracks", required=True)
  ap.add_argument("--episodes", nargs="+", required=True)
  ap.add_argument("--a-mapd", type=float, nargs="+", default=[2.2])
  ap.add_argument("--no-raise-cap", action="store_true", help="sensitivity: no +15 mph / posted caps")
  ap.add_argument("--adds", action="store_true", help="also scan for slowdowns v2 would ADD")
  ap.add_argument("--adds-from", default="2026-09-16",
                  help="first PT date whose ICBM record is continuous (the ces_events archive)")
  ap.add_argument("--k-scale", type=float, default=1.0, help="sensitivity: scale row AND truth curvature")
  ap.add_argument("--raise-margin", type=float, default=1.0,
                  help="inflate the row curvature when the DB RAISES ICBM's target (the unsafe direction)")
  ap.add_argument("--out", help="write per-episode rows (JSONL) here")
  ap.add_argument("--row-accuracy", action="store_true",
                  help="also score every held-out pass against its LODO row (the ICBM-independent error rate)")
  a = ap.parse_args(argv)

  idx = load_table(a.table)
  if a.row_accuracy:
    for amap in a.a_mapd:
      k_min = amap / (35.0 ** 2)              # rows whose DB speed is below 35 m/s (78 mph): where it binds
      acc, rtal = row_accuracy(idx, k_min=k_min, a_mapd=amap, k_margin=a.raise_margin)
      print(f"ROW ACCURACY (LODO, A={amap}, rows with k >= {k_min:.5f}): {len(acc)} held-out passes over " +
            f"{len({(x['anchor']) for x in acc})} anchors; refused: " +
            ", ".join(f"{k} {v}" for k, v in rtal.most_common()))
      for name, sel in (("all", acc), ("lightning", [x for x in acc if not x["car"].startswith(rt.TESLA_PREFIX)]),
                        ("tesla", [x for x in acc if x["car"].startswith(rt.TESLA_PREFIX)]),
                        ("op-steered", [x for x in acc if x["mode"] == "op"]),
                        ("manual/drv", [x for x in acc if x["mode"] in ("manual", "drv")])):
        if not sel:
          print(f"  {name:<11} n=0")
          continue
        r = [x["ratio"] for x in sel]
        bad = [x for x in sel if x["a_at_vdb"] >= REAL_MIN]
        print(f"  {name:<11} n={len(r):>6}  k_pass/k_row p50 {rt.percentile(r, 50):.3f} p90 {rt.percentile(r, 90):.3f} " +
              f"p99 {rt.percentile(r, 99):.3f}  a@v_db >= {REAL_MIN}: {len(bad)} ({100 * len(bad) / len(r):.2f} %) " +
              f"at {len({x['anchor'] for x in bad})} anchors")
      if a.out:
        with open(a.out.replace(".jsonl", f".A{amap}.rowacc.jsonl"), "w") as f:
          for x in acc:
            f.write(json.dumps(x) + "\n")
  eps, tally = load_episodes(a.episodes)
  print(f"table: {len(idx.anchors)} anchors;  episodes: {len(eps)}  ({dict(tally)})")
  if not eps or not idx.anchors:
    print("NOTHING TO REPLAY -- refusing to report on an empty set")
    return 2
  ranges = [(e.t - 30, e.t + 240) for e in eps]
  if a.adds:
    ranges.append((min(e.t for e in eps) - 86400, max(e.t for e in eps) + 86400))
  passes = load_passes(a.tracks, idx.p, ranges)
  print(f"own-pass candidates: {len(passes)} passes")
  rc = 0
  for amap in a.a_mapd:
    kw = {"raise_cap_ms": 1e9, "posted_margin_ms": 1e9} if a.no_raise_cap else {}
    rows = replay(idx, passes, eps, amap, tally, k_scale=a.k_scale, raise_margin=a.raise_margin, **kw)
    print(f"\n==================== A_mapd = {amap} m/s^2{'  (NO raise caps)' if a.no_raise_cap else ''}" +
          f"{f'  k_scale {a.k_scale}' if a.k_scale != 1.0 else ''}{f'  raise_margin {a.raise_margin}' if a.raise_margin != 1.0 else ''}")
    st = Counter(r["status"] if not r["status"].startswith("no authority") else "no authority" for r in rows)
    print("funnel: " + ", ".join(f"{k} {v}" for k, v in st.most_common()))
    na = Counter(r["status"][len("no authority: "):].split(" (")[0] for r in rows
                 if r["status"].startswith("no authority"))
    print("  no-authority reasons: " + ", ".join(f"{k} {v}" for k, v in na.most_common()))
    adj = [r for r in rows if r["status"] == "adjudicated"]
    grid = Counter((r["cls"], r["outcome"]) for r in adj)
    for cls in ("unwanted", "marginal", "wanted"):
      print(f"  {cls:<9}: " + ", ".join(f"{o} {n}" for (c, o), n in sorted(grid.items()) if c == cls))
    groups = {r["site_group"] for r in adj if r["real"]}
    print(f"  adjudicable REAL episodes: {sum(r['real'] for r in adj)} ({len(groups)} distinct site groups)")
    unsafe = [r for r in adj if r["real"] and r["outcome"] in ("LOST", "WEAKENED")]
    print(f"  SAFETY LIST (real, raised, truth >= {REAL_MIN} at v2 target): {len(unsafe)}")
    for r in unsafe + [r for r in adj if r["real"] and r["outcome"].startswith("raised")]:
      print(f"    {r['pt']} {r['car'][:5]} {r['road'][:18]:<18} ref {r['ref_mph']:.0f} icbm {r['icbm_mph']:.1f} " +
            f"-> v2 {r['target_mph']:.1f} mph  k_truth {r['k_truth']:.5f} k_row {r['k_row']:.5f} " +
            f"a@ref {r['a_ref']:.2f} a@v2 {r['a_target']:.2f}  {r['outcome']}")
    rc = rc or (1 if unsafe else 0)
    if a.out:
      with open(a.out.replace(".jsonl", f".A{amap}.jsonl"), "w") as f:
        for r in rows:
          f.write(json.dumps(r) + "\n")
    if a.adds:
      dates = {e.date for e in eps if not e.car.startswith(rt.TESLA_PREFIX) and e.date >= a.adds_from}
      adds = scan_adds(idx, passes, eps, amap, dates, lambda pt: scope_class(pt.hwy))
      print(f"  ADDS (v2 would slow where ICBM did not; Lightning passes on {len(dates)} episode dates): "
            + ", ".join(f"{k} {v}" for k, v in Counter(e["cls"] for e in adds).most_common()))
      for e in sorted(adds, key=lambda e: -(e["a_app"] or 0))[:40]:
        print(f"    {e['pt']} {e['road'][:22]:<22} ({e['lat']:.4f},{e['lon']:.4f}) v_app {e['v_app_mph']:.0f} -> " +
              f"v_db {e['v_db_mph']:.0f} mph  k_row {e['k_row']:.5f} k_truth " +
              f"{(e['k_truth'] or 0):.5f} a_app {(e['a_app'] or 0):.2f}  {e['cls']}")
      if a.out:
        with open(a.out.replace(".jsonl", f".A{amap}.adds.jsonl"), "w") as f:
          for e in adds:
            f.write(json.dumps(e) + "\n")
  return rc


if __name__ == "__main__":
  sys.exit(main())
