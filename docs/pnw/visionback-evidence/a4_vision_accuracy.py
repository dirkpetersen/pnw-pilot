#!/usr/bin/env python3
"""A4: re-derive vision's curvature accuracy vs distance to the candidate, instead of inheriting
docs/CURVEDB2PNW.md Sec 1.

That table came from drives/2026-09-12/central-oregon-weekend/vis_vs_dist.py with
    vision curvature = |visLat| / vEgo^2
    truth            = max |achLat|/vEgo^2 from HERE until travelled >= d + 40 m
and carries its own caveat: the truth is a max over a stretch and is biased HIGH, so every ratio is
biased LOW. This script varies BOTH terms so the size of each bias is measured rather than argued.

VISION TERM
  v1 `visLat/vEgo^2`  -- the Sec 1 method. `visLat` is vision_curve_lat_accel()'s
     `orientationRate.z[i] * velocity.x[i]`: the lateral accel at the model's OWN PLANNED speed at
     that point. Its curvature is therefore lat/velocity_x[i]^2, not lat/v_ego^2. Because the model
     plans to SLOW for a curve it sees (velocity_x[i] <= v_ego), dividing by v_ego^2 UNDER-states the
     curvature -- on exactly the curves that matter. ces_pnw.icbm_vision_curvature's docstring says
     this in as many words. So Sec 1's vision term carries a bias of its own, in the same direction
     as the truth bias but on the other side of the ratio.
  v2 `icbmKVis`       -- ces_pnw.icbm_vision_curvature = max |z_i| / max(v_i, 1.0) over the horizon:
     the SAME quantity computed correctly. Available only where curvelead2pnw telemetry exists.

TRUTH TERM
  t1 `runout40`  -- Sec 1's: max curvature from the current tick to (d + 40 m) travelled.
  t2 `site50`    -- max over travelled in [d - 25 m, d + 25 m] only: the peak AT the matched site,
                    which is the less-biased estimator the brief asks about.
  t3 `site50_kpeak` -- the same window using max(kPeak, |slKActl|) (per-second peak; 2026-09-17 on).

Curvature source for truth is |slKActl| (= CS.yawRate / vEgo, the same signal achLat is built from);
an exact 0.0 is treated as NO MEASUREMENT, never as a straight road (docs/CURVEDB2PNW.md D1).
"""
import json
import math
from collections import defaultdict

TICKS = "/home/dp/gh/comma/_scratch/visionback/ticks.jsonl"
MIN_V = 8.0
BUCKETS = [(0, 50), (50, 100), (100, 150), (150, 200), (200, 300), (300, 9999)]


def num(x):
  try:
    v = float(x)
  except (TypeError, ValueError):
    return None
  return v if math.isfinite(v) else None


def k_true(r, use_kpeak):
  """curvature the truck MEASURED on this tick, or None. Exact 0.0 -> None (dead-sensor rule)."""
  best = None
  ka = num(r.get("slKActl"))
  if ka is not None and abs(ka) > 1e-9:
    best = abs(ka)
  if use_kpeak:
    kp = num(r.get("kPeak"))
    if kp is not None and abs(kp) > 1e-9:
      best = abs(kp) if best is None else max(best, abs(kp))
  return best


def main():
  rows = [json.loads(x) for x in open(TICKS)]
  rows.sort(key=lambda r: r["t"])
  print(f"ford moving ticks: {len(rows)}")

  # index by drive so the forward walk never crosses a corpus boundary
  runs = defaultdict(list)
  for r in rows:
    runs[r["_src"]].append(r)

  stats = defaultdict(list)          # (vision_mode, truth_mode) -> [(d, kv, truth)]
  loss = defaultdict(int)
  for _, seq in runs.items():
    for i, r in enumerate(seq):
      if r.get("icbmSrc") not in ("map", "far"):
        continue
      d = num(r.get("mapDist"))
      if d is None or d <= 0:
        loss["noCandidateDistance"] += 1
        continue
      v_ego = num(r.get("vEgo"))
      if v_ego is None or v_ego <= MIN_V:
        loss["belowMinSpeed"] += 1
        continue
      vis = {}
      vl = num(r.get("visLat"))
      if vl is not None:
        vis["v1_visLat_over_vego2"] = abs(vl) / (v_ego * v_ego)
      kv = num(r.get("icbmKVis"))
      if kv is not None:
        vis["v2_icbmKVis"] = kv
      if not vis:
        loss["noVisionWitness"] += 1
        continue
      # forward walk: accumulate travelled distance, collect the truth in each window
      travelled = 0.0
      t_runout = t_site = t_site_kp = 0.0
      reached = False
      for j in range(i, min(i + 200, len(seq))):
        v = num(seq[j].get("vEgo")) or 0.0
        dt = (seq[j]["t"] - seq[j - 1]["t"]) if j > i else 0.0
        travelled += v * (dt if 0.0 < dt < 5.0 else 1.0)
        kt = k_true(seq[j], False)
        kp = k_true(seq[j], True)
        if kt is not None:
          t_runout = max(t_runout, kt)
          if d - 25.0 <= travelled <= d + 25.0:
            t_site = max(t_site, kt)
        if kp is not None and d - 25.0 <= travelled <= d + 25.0:
          t_site_kp = max(t_site_kp, kp)
        if travelled >= d + 40.0:
          reached = True
          break
      if not reached:
        loss["corpusEndedBeforeTheCandidate"] += 1
        continue
      for vname, kvv in vis.items():
        for tname, tv in (("t1_runout40", t_runout), ("t2_site50", t_site),
                          ("t3_site50_kpeak", t_site_kp)):
          if tv <= 1e-9:
            loss[f"noTruth:{tname}"] += 1
            continue
          stats[(vname, tname)].append((d, kvv, tv))

  print(f"dropped: {dict(sorted(loss.items(), key=lambda kv: -kv[1]))}\n")
  for key in sorted(stats):
    vname, tname = key
    s = stats[key]
    print(f"=== vision={vname}   truth={tname}   n={len(s)} ===")
    print(f"    {'distance':<14}{'n':>6}{'vis/truth med':>15}{'under-reads':>13}{'badly<0.5x':>12}"
          f"{'p10 ratio':>11}{'p90 ratio':>11}")
    for lo, hi in BUCKETS:
      b = [x for x in s if lo <= x[0] < hi]
      if not b:
        continue
      rt = sorted(kv / tv for _, kv, tv in b)
      n = len(rt)
      print(f"    {f'{lo}-{hi} m':<14}{n:>6}{rt[n // 2]:>15.2f}"
            f"{sum(1 for x in rt if x < 1.0) / n * 100:>12.0f}%"
            f"{sum(1 for x in rt if x < 0.5) / n * 100:>11.0f}%"
            f"{rt[int(.1 * n)]:>11.2f}{rt[min(int(.9 * n), n - 1)]:>11.2f}")
    print()


if __name__ == "__main__":
  main()
