#!/usr/bin/env python3
"""A9: does the verdict hinge on the APEX-WITNESS window? Repeats the sweep's maximum under the
three window definitions _scratch/icbmslow/a5_replay.py used (tight / base / wide)."""
import json, sys, statistics as st
sys.path.insert(0, "/home/dp/gh/comma/_scratch/visionback")
import a3_giveback as A3                                                         # noqa: E402

MPH, fnum, CP = A3.MPH, A3.fnum, A3.CP
V = A3.veh()
a_lat_vis = V.icbm_lead_lat_accel
recs = sorted((json.loads(x) for x in open(A3.TICKS)), key=lambda r: r["t"])
by_t = {r["t"]: r for r in recs}
times = sorted(by_t)
eps = A3.build_episodes(recs)
N_LIST = [0, 2, 5, 10, 15]

for cruise_only in (True, False):
  for wname in ("tight", "base", "wide"):
    rows = []
    for e in eps:
      ticks = e["ticks"]
      v0 = fnum(ticks[0].get("vEgo")) or 0.0
      if v0 < 25 * MPH:
        continue
      tmin = min(ticks, key=lambda r: fnum(r.get("icbmT")) or 1e9)
      md = fnum(tmin.get("mapDist")) or 0.0
      reach = md if md > 0 else 150.0
      win = {"tight": (e["t0"], e["t_end"] + min(reach / max(v0, 1.0), 12.0) + 4.0, 0.6 * v0),
             "base": (e["t0"], e["t_end"] + min(reach / max(v0, 1.0) + 10.0, 40.0), A3.MIN_V),
             "wide": (e["t0"] - 5.0, e["t_end"] + min(reach / max(v0, 1.0) + 20.0, 60.0), A3.MIN_V)}[wname]
      k, v_apex, _, seen, _ = A3.apex_witness(times, by_t, win[0], win[1], win[2], True)
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
          pt.append((d, p, ref, fnum(r.get("spdLim")) or 0.0, r))
      if not pt:
        continue
      T = min(p for _, p, _, _, _ in pt)
      entry = [r for d, _, _, _, r in pt if d <= A3.HI]
      s_entry = ((fnum(entry[0].get("stockSet")) or fnum(entry[0].get("vEgo")) or float("inf"))
                 if entry else float("inf"))
      elig = []
      for d, p, ref, posted, r in pt:
        if not (A3.LO <= d <= A3.HI):
          continue
        mr = fnum(r.get("mdlEndX"))
        if mr is None or d > mr:
          continue
        vv, _ = A3.vision_speed(r, "vislat_bold", a_lat_vis)
        if vv is None or vv <= p + 1e-9:
          continue
        elig.append((p, ref, posted, vv))
      cf = {}
      for N in N_LIST:
        if N == 0 or not elig:
          cf[N] = T
          continue
        cand = []
        for p, ref, posted, vv in elig:
          cap = ref - CP.ICBM_MIN_DROP_MS
          if posted > 0.0:
            cap = min(cap, posted)
          cand.append(min(p + min(N * MPH, vv - p), max(cap, p)))
        cf[N] = max(T, min(s_entry, min(cand), T + N * MPH))
      rows.append((k, v_apex or 0.0, T, cf))
    lab = "cruise_ON" if cruise_only else "ALL"
    out = []
    for N in N_LIST:
      al = [k * max(va, cf[N]) ** 2 for k, va, T, cf in rows]
      out.append(f"N{N}: med {st.median(al):.2f} max {max(al):.2f}")
    print(f"[{lab:<9}] window={wname:<6} n={len(rows):>3}  " + " | ".join(out))
