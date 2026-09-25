#!/usr/bin/env python3
"""curveshape2pnw 3/3 -- replay the measured-shape stage in LIVE mode over the Lightning's ces_events corpora and judge
the design's gates G1-G7 (~/gh/comma/docs/CURVE-MEASURED-SHAPE-DESIGN.md s7).

WHAT IS REPLAYED. Every moving Lightning stock-ACC tick with a NEAR map candidate (mapV / mapDist as logged) is priced
twice with the REAL functions: ICBM as shipped (icbm_curve_target + icbm_penalise, no pricer) and ICBM with the stage
LIVE (the same functions with a real ShapePricer, the tick's own icbmK/icbmKD/icbmKN/icbmKAhead, a real ShapeLatch run
tick by tick, tick_gate on the logged waySel / hwyClass). No constant is transcribed.

WHAT IS NOT, stated so a pass is not over-read:
  * the FAR candidate and the curve DB's decision need mapd's whole path, which was never logged before
    mapdpathlog2pnw -- so a curve enters the replay only inside the 10 s near window (it does on every approach);
  * the start gates (in-curve, map-first), the passed-point gate, the posted-limit floor and lead pacing are not
    applied (the passed point is approximated as Part C did: the same mapV with a growing mapDist);
  * no pitch and no path -> no descent or left extra (left_factor is 1.0 on the channel anyway);
  * mapd's A is the truck's device-verified 2.0 (--a-mapd).

JUDGING A CURVE (s7). Achieved lateral accel at the speed driven = peak 3-tick |kPoseP| (else |slKActl|) x v^2 within
+-8 s of arrival at the candidate. REAL >= 2.3, GREY 1.8-2.3, PHANTOM < 1.8 with no lead and no gas/steer input
(< 1.8 WITH input is counted as GREY, conservatively), NO-OP if the target >= the minimum speed driven - 0.6 mph,
UNJUDGED with no curvature witness (listed, never counted as a pass).

OUTPUT goes to --out (default ~/gh/comma/drives/2026-09-24/curveshape-replay/), NEVER into this repo: the tick cache
and the episode lists carry positions. This file holds only times and rules.

Run (from a pnw-pilot worktree):
  PYTHONPATH=$PWD:$PWD/opendbc_repo python tools/curveshape/replay.py [--rescan] [--out DIR] [--a-mapd 2.0]
Exit code 0 = every gate PASS, 1 = a gate FAILED or is UNJUDGED, 2 = the harness itself failed.
"""
from __future__ import annotations

import argparse
import bisect
import glob
import gzip
import io
import json
import math
import os
import sys
from collections import Counter
from datetime import datetime
from zoneinfo import ZoneInfo

from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw import curvedb_live as cl
from openpilot.selfdrive.controls.lib.ces_pnw import icbm_shape as shp

PT = ZoneInfo("America/Los_Angeles")
MPH = 0.44704
WB = os.path.expanduser("~/gh/comma")
ROOTS = [os.path.join(WB, "drives"), os.path.join(WB, "_scratch/curvedb-v2/pnwlogs")]
OUT = os.path.join(WB, "drives/2026-09-24/curveshape-replay")
GAP_S = 3.0                 # ticks further apart than this start a new episode
NOOP_MPH = 0.6              # the executor deadband
KEYS = ["car", "ev", "vEgo", "stockSet", "stockOn", "mapV", "mapDist", "icbmT", "icbmC", "icbmSrc", "kPoseP", "slKActl",
        "gas", "strPrs", "hasLead", "lat", "lon", "bearing", "spdLim", "hwyClass", "waySel", "icbmK", "icbmKD", "icbmKN",
        "icbmKAhead", "cdb2Dir", "icbmFlrHit"]
# pre-`car`-field corpora (Part C's list, each from its own DRIVE_REPORT header)
PRE_CAR = {"2026-07-10/lightning-oplong-first-lap": "FORD", "2026-07-11/lightning-icbm-nofire": "FORD",
           "2026-07-12/ellensburg-snoqualmie-westbound": "FORD", "2026-07-12/snoqualmie-ellensburg-icbm": "FORD"}


class CP:
  carFingerprint = "FORD_F_150_LIGHTNING_MK1"
  brand = "ford"
  openpilotLongitudinalControl = False


def f(x):
  try:
    x = float(x)
  except (TypeError, ValueError):
    return None
  return x if math.isfinite(x) else None


def pt_epoch(date, hms):
  return datetime.fromisoformat(f"{date}T{hms}").replace(tzinfo=PT).timestamp()


def pt(t):
  return datetime.fromtimestamp(t, PT).strftime("%Y-%m-%d %H:%M:%S")


# ---------------------------------------------------------------------------------------------------------------------
# scan
# ---------------------------------------------------------------------------------------------------------------------
def _open(p):
  if p.endswith(".gz"):
    return gzip.open(p, "rt", errors="replace")
  if p.endswith(".zst"):
    import zstandard
    return io.TextIOWrapper(zstandard.ZstdDecompressor().stream_reader(open(p, "rb")), errors="replace")
  return open(p, errors="replace")


def scan(roots):
  """Every moving Lightning tick from every ces_events file. Rule 2: every file is reported; an unreadable one is an
  ERROR row, never a zero. Duplicate copies (.zst next to plain) are de-duplicated on the tick time."""
  files = set()
  for r in roots:
    for pat in ("ces*.jsonl", "ces*.jsonl.*", "ces_events*"):
      files |= set(glob.glob(os.path.join(r, "**", pat), recursive=True))
  files = sorted(x for x in files if not x.endswith((".py", ".txt", ".md", ".json")) and os.path.isfile(x))
  keep, inv = {}, []
  for fn in files:
    rel = os.path.relpath(fn, WB)
    row = {"file": rel, "rec": 0, "perr": 0, "ford": 0, "err": None}
    pre = next((c for k, c in PRE_CAR.items() if k in rel), None)
    try:
      with _open(fn) as fh:
        for line in fh:
          line = line.strip()
          if not line:
            continue
          try:
            r = json.loads(line)
          except ValueError:
            row["perr"] += 1
            continue
          if not isinstance(r, dict):
            row["perr"] += 1
            continue
          row["rec"] += 1
          car = str(r.get("car") or pre or "")
          t = r.get("t")
          if not car.startswith("FORD") or not isinstance(t, (int, float)) or r.get("ev") not in ("tick", None):
            continue
          v = f(r.get("vEgo"))
          if v is None or v < 4.0:
            continue
          row["ford"] += 1
          d = {k: r.get(k) for k in KEYS}
          d["_hasK"] = "icbmK" in r            # a field that is ABSENT (July) is not a field that reads 0
          d["t"], d["_src"] = t, rel
          keep[round(t, 2)] = d
    except Exception as e:
      row["err"] = f"{type(e).__name__}: {e}"
    inv.append(row)
  return [keep[k] for k in sorted(keep)], inv


# ---------------------------------------------------------------------------------------------------------------------
# replay: the real functions, tick by tick
# ---------------------------------------------------------------------------------------------------------------------
def make_vehicle():
  saved, pv.CURVE_CONFIG_PATH = pv.CURVE_CONFIG_PATH, "/nonexistent/curve.json"   # the shipped defaults, not a file
  try:
    v = pv.PnwVehicle(CP())                             # curve.json is read here, once
  finally:
    pv.CURVE_CONFIG_PATH = saved
  assert v.lightning_curve_slow, "the replay vehicle must be the Lightning"
  return v


def price_tick(veh, r, ref, pricer):
  """ICBM's near-map target for one tick through the real functions; pricer None = as shipped."""
  v, map_v, map_d = f(r["vEgo"]), f(r["mapV"]), f(r["mapDist"])
  t, _, src = m.icbm_curve_target(v, ref, map_v, map_d, None, m.icbm_map_eff_scale, map_scale=veh.icbm_map_scale,
                                  firm_decel=veh.icbm_firm_decel, track=True, eff_fn=pricer)
  if t is None:
    return None
  sig = {"map_target_v": map_v, "map_target_dist": map_d, "curve_lat_accel_vision": 0.0, "pitch": None}
  return m.icbm_penalise(veh, [], t, src, sig, None, None, float("inf"), 0.0, pricer=pricer)[0]


def replay(ticks, veh, a_mapd):
  why = Counter()
  res = []
  latch = shp.ShapeLatch()
  for i, r in enumerate(ticks):
    v, ss = f(r.get("vEgo")), f(r.get("stockSet"))
    # the latch sees EVERY tick (a refresh per tick), replayable or not, exactly as the car's does
    k_t = r["t"] if r.get("_hasK") else None
    stable = latch.update(k_t, r.get("icbmK"), r.get("icbmKD"), r.get("icbmKN"), r.get("icbmKAhead"), v)
    if r.get("stockOn") is not True:
      why["stock ACC off"] += 1
      continue
    if not ss or ss <= 0:
      why["no stockSet"] += 1
      continue
    map_v, map_d = f(r.get("mapV")), f(r.get("mapDist"))
    if not map_v or map_v <= 0 or map_d is None or map_d <= 0:
      why["no near map candidate"] += 1
      continue
    pr = ticks[i - 1] if i else None
    if (pr is not None and r["t"] - pr["t"] <= 2.0 and f(pr.get("mapV")) == map_v and f(pr.get("mapDist")) is not None
        and map_d > f(pr.get("mapDist"))):
      why["passed point (mapDist growing)"] += 1
      continue
    ic = f(r.get("icbmC"))
    ref = ic if (ic and r.get("icbmT") is not None) else ss
    gate = shp.tick_gate(r.get("waySel"), r.get("hwyClass"), "raw", cl.RAMP_CLASSES, cl.UNKNOWN_CLASSES)
    reading = (r.get("icbmK"), r.get("icbmKD"), r.get("icbmKN"), r.get("icbmKAhead"))
    pricer = None
    dec = (None, gate or "noPolylineFields", "none", None, None, None)
    if gate is None and r.get("_hasK"):
      posted = f(r.get("spdLim")) or 0.0
      pricer = shp.ShapePricer(m.icbm_map_eff_scale, veh.icbm_map_scale, a_mapd, reading, stable, posted, ref,
                               veh.icbm_shape_lat_a, veh.icbm_shape_lat_a_70)
      dec = pricer.decide(map_v, map_d)
    old = price_tick(veh, r, ref, None)
    new = price_tick(veh, r, ref, pricer) if pricer is not None else old
    why["replayed"] += 1
    res.append({"i": i, "t": r["t"], "v": v, "ref": ref, "mapV": map_v, "mapD": map_d, "old": old, "new": new,
                "why": dec[1], "dir": dec[2], "k": dec[3], "km": dec[4], "src": r["_src"]})
  return res, why


def validate(ticks, res, veh):
  """The forward model (as shipped) against the logged icbmT on map-sourced ticks with nothing else acting."""
  val = Counter()
  for x in res:
    r = ticks[x["i"]]
    if r.get("icbmSrc") != "map" or r.get("icbmT") is None or x["old"] is None:
      continue
    if r.get("cdb2Dir") not in (None, "none") or r.get("icbmFlrHit"):
      val["skip: DB / posted floor acted"] += 1
      continue
    d = abs(f(r["icbmT"]) - x["old"])
    val["within 0.3 m/s" if d <= 0.3 else ("within 1.0" if d <= 1.0 else "off > 1.0 m/s")] += 1
  return val


# ---------------------------------------------------------------------------------------------------------------------
# episodes and judging
# ---------------------------------------------------------------------------------------------------------------------
def episodes(res, key):
  eps, cur = [], None
  for x in res:
    if x[key] is None:
      continue
    if cur and x["t"] - cur["t1"] <= GAP_S and x["src"].split("/")[:2] == cur["src"]:
      cur["t1"] = x["t"]
      cur["xs"].append(x)
    else:
      cur = {"t0": x["t"], "t1": x["t"], "xs": [x], "src": x["src"].split("/")[:2]}
      eps.append(cur)
  return eps


def overlaps(e, others):
  return [o for o in others if o["t0"] - GAP_S <= e["t1"] and e["t0"] <= o["t1"] + GAP_S]


def kwit(r):
  k = f(r.get("kPoseP"))
  if k is not None:
    return abs(k)
  k = f(r.get("slKActl"))
  return abs(k) if k is not None else None


class Measure:
  def __init__(self, ticks):
    self.ticks = ticks
    self.T = [r["t"] for r in ticks]

  def __call__(self, t0, t1):
    a, b = bisect.bisect_left(self.T, t0), bisect.bisect_right(self.T, t1)
    w = self.ticks[a:b]
    ks = [(kwit(r), f(r.get("vEgo"))) for r in w]
    best = None
    for j in range(1, len(ks) - 1):
      tri = [ks[j - 1][0], ks[j][0], ks[j + 1][0]]
      if None in tri or ks[j][1] is None:
        continue
      km = sum(tri) / 3.0
      if best is None or km > best[0]:
        best = (km, ks[j][1])
    vmin = min((f(r.get("vEgo")) for r in w if f(r.get("vEgo")) is not None), default=None)
    return {"k": best[0] if best else None, "v_at_k": best[1] if best else None, "vmin": vmin,
            "input": any(r.get("gas") or r.get("strPrs") for r in w),
            "lead": any(r.get("hasLead") for r in w[:max(1, len(w) // 2)])}


def judge(e, key, measure):
  """One episode at its lowest `key` target: what the truck did at that curve."""
  mn = min(e["xs"], key=lambda x: x[key])
  t_curve = mn["t"] + mn["mapD"] / max(mn["v"], 1.0)
  ms = measure(t_curve - 8.0, t_curve + 8.0)
  k = ms["k"]
  s = {"when": pt(e["t0"]), "src": "/".join(e["src"]), "n": len(e["xs"]), "mapV": mn["mapV"] / MPH,
       "ref": mn["ref"] / MPH, "tgt": mn[key] / MPH, "old": mn["old"] / MPH if mn["old"] is not None else None,
       "why": mn["why"], "vmin": ms["vmin"] / MPH if ms["vmin"] else None, "k": k,
       "a_drv": k * ms["v_at_k"] ** 2 if k else None, "a_tgt": k * mn[key] ** 2 if k else None,
       "a_ref": k * mn["ref"] ** 2 if k else None, "v_need": math.sqrt(2.5 / k) / MPH if k else None,
       "input": ms["input"], "lead": ms["lead"]}
  s["noop"] = s["vmin"] is not None and s["tgt"] >= s["vmin"] - NOOP_MPH
  return s


def classify(s):
  if s["k"] is None:
    return "UNJUDGED"
  if s["noop"]:
    return "NO-OP"
  if s["a_drv"] >= 2.3:
    return "REAL"
  if s["a_drv"] >= 1.8:
    return "GREY"
  if s["lead"] or s["input"]:
    return "GREY"               # < 1.8 but the driver or a lead confounds the measurement: counted as GREY, not hidden
  return "PHANTOM"


def too_deep(s):
  """s7 G7: a change more than 3 mph below the measured need (at 2.5 m/s^2), without a lead."""
  return s["k"] is not None and not s["noop"] and not s["lead"] and s["tgt"] < s["v_need"] - 3.0


# ---------------------------------------------------------------------------------------------------------------------
# gates
# ---------------------------------------------------------------------------------------------------------------------
def window(res, date, a, b):
  t0, t1 = pt_epoch(date, a), pt_epoch(date, b)
  return [x for x in res if t0 <= x["t"] <= t1]


def _mph(x):
  return None if x is None else round(x / MPH, 1)


def gates(res, ticks, E_new, added, removed, lowered):
  g = {}

  w = window(res, "2026-09-24", "11:17:50", "11:18:45")
  b = [x for x in w if x["new"] is not None]
  if not w:
    g["G1"] = ("UNJUDGED", "no replayable tick in 11:17:50-11:18:45 PT 09-24")
  else:
    first_d = b[0]["mapD"] if b else None
    tmin = min(x["new"] for x in b) if b else None
    ok = bool(b) and first_d >= 150.0 and tmin / MPH <= 72.5
    g["G1"] = ("PASS" if ok else "FAIL",
               f"{len(w)} ticks; binds {len(b)}; first bind at {first_d and round(first_d)} m; min target " +
               f"{_mph(tmin)} mph (today's min {_mph(min((x['old'] for x in w if x['old'] is not None), default=None))})")

  w = window(res, "2026-09-24", "11:18:55", "11:19:45")
  b = [x["new"] for x in w if x["new"] is not None]
  if not w:
    g["G2"] = ("UNJUDGED", "no replayable tick in 11:18:55-11:19:45 PT 09-24")
  else:
    tmin = min(b) if b else None
    old = min((x["old"] for x in w if x["old"] is not None), default=None)
    ok = tmin is None or tmin / MPH >= 67.6 - 0.6
    g["G2"] = ("PASS" if ok else "FAIL", f"{len(w)} ticks; map min {_mph(tmin)} mph (today {_mph(old)}); bar >= 67.0")

  parts, bad, n = [], 0, 0
  for date, a, bb in (("2026-09-17", "14:51:00", "14:54:00"), ("2026-09-24", "12:57:00", "13:00:00")):
    w = window(res, date, a, bb)
    n += len(w)
    new_bind = [x for x in w if x["new"] is not None and x["old"] is None]
    lower = [x for x in w if x["new"] is not None and x["old"] is not None and x["new"] < x["old"] - 0.05]
    bad += len(new_bind) + len(lower)
    parts.append(f"{date}: {len(w)} ticks, whys {dict(Counter(x['why'] for x in w))}, new binds {len(new_bind)}, " +
                 f"lowered {len(lower)}")
  g["G3"] = ("UNJUDGED" if n == 0 else ("PASS" if bad == 0 else "FAIL"), "; ".join(parts))

  w = [x for x in window(res, "2026-09-21", "21:21:40", "21:22:15") if abs(x["mapV"] / MPH - 59.9) <= 0.5]
  b = [x["new"] for x in w if x["new"] is not None]
  if not w:
    g["G4"] = ("UNJUDGED", "no tick naming the 59.9 mph point in 21:21:40-21:22:15 PT 09-21")
  else:
    tmin = min(b) if b else None
    ok = tmin is not None and 60.0 <= tmin / MPH <= 66.0
    g["G4"] = ("PASS" if ok else "FAIL", f"{len(w)} ticks on the 59.9 point; shape binds {len(b)}; min {_mph(tmin)} mph " +
               f"(today binds {sum(x['old'] is not None for x in w)}); whys {dict(Counter(x['why'] for x in w))}")

  w = window(res, "2026-09-24", "12:35:00", "12:36:40")
  ch = [x for x in w if x["new"] != x["old"]]
  g["G5"] = ("UNJUDGED" if not w else ("PASS" if not ch else "FAIL"),
             f"{len(w)} ticks; changed {len(ch)}; whys {dict(Counter(x['why'] for x in w))}")

  w = [x for x in res if x["src"].startswith(("drives/2026-07-11", "drives/2026-07-12"))]
  ch = [x for x in w if x["new"] != x["old"]]
  g["G6"] = ("UNJUDGED" if not w else ("PASS" if not ch else "FAIL"),
             f"{len(w)} replayable July I-90 ticks; changed {len(ch)}; whys {dict(Counter(x['why'] for x in w))}")

  cls = Counter(classify(s) for s in added)
  rem_real = [s for s in removed if s["k"] is not None and not s["noop"] and s["a_ref"] >= 2.3]
  deep = [s for s in added + lowered if too_deep(s)]
  ok = (cls["PHANTOM"] == 0 and cls["GREY"] <= 2 and len(added) <= 25 and not rem_real and not deep)
  g["G7"] = ("PASS" if ok else "FAIL",
             f"added {len(added)} ({dict(cls)}); removed {len(removed)} (real {len(rem_real)}); " +
             f"lowered {len(lowered)}; > 3 mph below need without a lead: {len(deep)}")
  return g


def fmt(s):
  def g(x, n=1):
    return "   -" if x is None else f"{x:5.{n}f}"
  return (f"{s['when']}  {s['src'][:40]:40s} mapV {g(s['mapV'])} ref {g(s['ref'])} today {g(s['old'])} " +
          f"shape {g(s['tgt'])} ({s['why']}) | drove min {g(s['vmin'])} k {g(s['k'], 5)} a@drv {g(s['a_drv'], 2)} " +
          f"a@tgt {g(s['a_tgt'], 2)} need@2.5 {g(s['v_need'])} {'INPUT ' if s['input'] else ''}" +
          f"{'LEAD' if s['lead'] else ''}")


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--out", default=OUT)
  ap.add_argument("--rescan", action="store_true")
  ap.add_argument("--a-mapd", type=float, default=2.0)
  args = ap.parse_args(argv)
  os.makedirs(args.out, exist_ok=True)
  cache = os.path.join(args.out, "ticks.jsonl")
  if args.rescan or not os.path.exists(cache):
    ticks, inv = scan(ROOTS)
    with open(cache, "w") as fo:
      for r in ticks:
        fo.write(json.dumps(r) + "\n")
    json.dump(inv, open(os.path.join(args.out, "inventory.json"), "w"), indent=1)
    errs = [r for r in inv if r["err"]]
    print(f"scanned {len(inv)} files, {sum(r['rec'] for r in inv)} records, {len(ticks)} unique moving Lightning ticks, " +
          f"{sum(r['perr'] for r in inv)} parse errors, {len(errs)} file errors")
    for r in errs:
      print("  FILE ERROR", r["file"], r["err"])
  else:
    ticks = [json.loads(ln) for ln in open(cache)]
    print(f"cached {len(ticks)} ticks from {cache} (--rescan to rebuild)")
  if not ticks:
    print("HARNESS FAILURE: no ticks at all -- the corpus or the scan is broken, not a result")
    return 2
  veh = make_vehicle()
  res, why = replay(ticks, veh, args.a_mapd)
  val = validate(ticks, res, veh)
  measure = Measure(ticks)
  E_old, E_new = episodes(res, "old"), episodes(res, "new")
  added, removed, lowered, raised = [], [], [], []
  for e in E_new:
    ov = overlaps(e, E_old)
    s = judge(e, "new", measure)
    if not ov:
      added.append(s)
      continue
    to = min(x["old"] for o in ov for x in o["xs"]) / MPH
    s["old"] = to
    if s["tgt"] < to - 0.5:
      lowered.append(s)
    elif s["tgt"] > to + 0.5:
      raised.append(s)
  for e in E_old:
    if not overlaps(e, E_new):
      removed.append(judge(e, "old", measure))
  G = gates(res, ticks, E_new, added, removed, lowered)

  L = []
  P = L.append
  P("# curveshape2pnw replay (LIVE mode, real functions)")
  P(f"ticks {len(ticks)}; accounting {dict(why)}")
  P(f"forward-model check (as shipped vs logged icbmT, map-sourced): {dict(val)}")
  P(f"stage verdicts on replayed ticks: {dict(Counter(x['why'] for x in res))}")
  P(f"episodes: today {len(E_old)}, live {len(E_new)}; ADDED {len(added)}, REMOVED {len(removed)}, " +
    f"LOWERED {len(lowered)}, RAISED {len(raised)}")
  P("\n## Gates")
  P("| gate | verdict | evidence |")
  P("|---|---|---|")
  for k in sorted(G):
    P(f"| {k} | {G[k][0]} | {G[k][1]} |")
  for name, lst in (("ADDED", added), ("REMOVED", removed), ("LOWERED", lowered), ("RAISED", raised)):
    P(f"\n## {name} ({len(lst)})")
    for s in sorted(lst, key=lambda s: s["when"]):
      tag = classify(s) if name != "REMOVED" else ("REAL-LOST" if s["k"] and not s["noop"] and s["a_ref"] >= 2.3 else
                                                   ("NO-OP" if s["noop"] else ("UNJUDGED" if s["k"] is None else "ok")))
      P(f"  [{tag}]{' TOO-DEEP' if too_deep(s) else ''} " + fmt(s))
  with open(os.path.join(args.out, "report.md"), "w") as fo:
    fo.write("\n".join(L) + "\n")
  json.dump({k: {"verdict": v[0], "evidence": v[1]} for k, v in G.items()},
            open(os.path.join(args.out, "gates.json"), "w"), indent=1)
  print("\n".join(L[:12 + len(G)]))
  print(f"\nfull report: {os.path.join(args.out, 'report.md')}")
  return 0 if all(v[0] == "PASS" for v in G.values()) else 1


if __name__ == "__main__":
  sys.exit(main())
