#!/usr/bin/env python3
"""A7: two diagnostics the verdict rests on.
 (1) How often was ICBM's commanded target ABOVE the speed the truck actually met at the apex? On
     those episodes something else (a lead, the driver, op-long) was the binding constraint, so
     max(v_apex, T) over-states what a give-back could deliver -- stated rather than hidden.
 (2) Vision's verdict on the in-window map ICBM ticks whose model horizon covers the candidate:
     no-curve-at-all / gentler-than-the-map-target / tighter. This is the number that decides
     whether "only where vision is trustworthy" is a gate or a constant."""
import json, sys, statistics as st
sys.path.insert(0, "/home/dp/gh/comma/_scratch/visionback")
import a3_giveback as A3                                                         # noqa: E402

MPH, fnum = A3.MPH, A3.fnum
V = A3.veh()
recs = sorted((json.loads(x) for x in open(A3.TICKS)), key=lambda r: r["t"])
by_t = {r["t"]: r for r in recs}
times = sorted(by_t)
eps = A3.build_episodes(recs)

for cruise_only in (True, False):
  rows = []
  for e in eps:
    ticks = e["ticks"]
    v0 = fnum(ticks[0].get("vEgo")) or 0.0
    if v0 < 25 * MPH:
      continue
    tmin = min(ticks, key=lambda r: fnum(r.get("icbmT")) or 1e9)
    md = fnum(tmin.get("mapDist")) or 0.0
    reach = md if md > 0 else 150.0
    k, v_apex, _, seen, _ = A3.apex_witness(
      times, by_t, e["t0"], e["t_end"] + min(reach / max(v0, 1.0) + 10.0, 40.0), A3.MIN_V, True)
    if seen == 0 or k <= 0 or 1.0 / k < A3.MIN_ROAD_R:
      continue
    pt = []
    for r in ticks:
      if r.get("icbmSrc") != "map" or (cruise_only and not r.get("stockOn")):
        continue
      mv, d, ve = fnum(r.get("mapV")), fnum(r.get("mapDist")), fnum(r.get("vEgo"))
      ref = fnum(r.get("icbmC")) or fnum(r.get("stockSet")) or fnum(r.get("vSet"))
      if not mv or not d or d <= 0 or not ve or not ref:
        continue
      p = A3.model_target(V, mv, d, ref, ve, None, False, floor_raw=True)
      if p is not None:
        pt.append(p)
    if pt:
      rows.append((min(pt), v_apex or 0.0, k))
  lab = "cruise_ON" if cruise_only else "ALL"
  n = len(rows)
  above = sum(1 for T, va, _ in rows if T > va + 1e-6)
  d = [(T - va) / MPH for T, va, _ in rows]
  al_m = [k * va * va for T, va, k in rows]
  al_c = [k * T * T for T, va, k in rows]
  print(f"\n[{lab}] n={n} episodes >25 mph")
  print(f"   ICBM's target ABOVE the measured apex speed: {above}/{n} = {above / n * 100:.0f}%")
  print(f"   (T - v_apex) mph: median {st.median(d):+.1f} p90 {sorted(d)[int(.9 * n)]:+.1f} max {max(d):+.1f}")
  print(f"   a_lat AT THE MEASURED APEX SPEED: median {st.median(al_m):.2f} "
        f"p90 {sorted(al_m)[int(.9 * n)]:.2f} max {max(al_m):.2f}")
  print(f"   a_lat AT ICBM'S COMMANDED TARGET: median {st.median(al_c):.2f} "
        f"p90 {sorted(al_c)[int(.9 * n)]:.2f} max {max(al_c):.2f}")

print("\nVISION'S VERDICT on in-window (50-150 m) map ICBM ticks whose mdlEndX covers the candidate:")
for cruise_only in (True, False):
  c = {"noCurveAtAll": 0, "gentlerThanMapTarget": 0, "tighterOrEqual": 0}
  for e in eps:
    for r in e["ticks"]:
      if r.get("icbmSrc") != "map" or (cruise_only and not r.get("stockOn")):
        continue
      d = fnum(r.get("mapDist"))
      if d is None or not (50 <= d <= 150):
        continue
      mr = fnum(r.get("mdlEndX"))
      if mr is None or d > mr:
        continue
      ve, mv = fnum(r.get("vEgo")), fnum(r.get("mapV"))
      ref = fnum(r.get("icbmC")) or fnum(r.get("stockSet")) or fnum(r.get("vSet"))
      if not (ve and mv and ref):
        continue
      p = A3.model_target(V, mv, d, ref, ve, None, False, floor_raw=True)
      if p is None:
        continue
      vv, _ = A3.vision_speed(r, "vislat_strict", V.icbm_lead_lat_accel)
      c["noCurveAtAll" if vv is None else
        ("gentlerThanMapTarget" if vv > p + 1e-9 else "tighterOrEqual")] += 1
  tot = sum(c.values())
  print(f"  [{'cruise_ON' if cruise_only else 'ALL'}] n={tot}: "
        + ", ".join(f"{k} {v} ({v / max(tot, 1) * 100:.1f}%)" for k, v in c.items()))
