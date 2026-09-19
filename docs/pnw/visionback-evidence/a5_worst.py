#!/usr/bin/env python3
"""A5: name the episodes the sweep puts over the line, and measure how often the rule would fire
in real driving terms. Also sweeps the WINDOW (the 50-150 m bound is the owner's, not a measurement).

Reuses a3's model and definitions by import so the two cannot drift.
"""
import datetime
import json
import math
import statistics as st
import sys
import zoneinfo
from collections import defaultdict

sys.path.insert(0, "/home/dp/gh/comma/_scratch/visionback")
import a3_giveback as A3                                                         # noqa: E402

PT = zoneinfo.ZoneInfo("America/Los_Angeles")
MPH = A3.MPH
CP, fnum = A3.CP, A3.fnum


def analyse(recs, times, by_t, eps, V, a_lat_vis, lo, hi, mode, N_list):
  """One (window, vision-mode) configuration -> per-episode counterfactual rows."""
  out = []
  for e in eps:
    ticks = e["ticks"]
    v0 = fnum(ticks[0].get("vEgo")) or 0.0
    tg = [fnum(r.get("icbmT")) for r in ticks if fnum(r.get("icbmT")) is not None]
    if not tg or v0 < A3.MIN_V:
      continue
    tmin_rec = min(ticks, key=lambda r: fnum(r.get("icbmT")) or 1e9)
    map_d = fnum(tmin_rec.get("mapDist")) or 0.0
    reach = map_d if map_d > 0 else 150.0
    k, v_apex, t_at, seen, ksrc = A3.apex_witness(
      times, by_t, e["t0"], e["t_end"] + min(reach / max(v0, 1.0) + 10.0, 40.0), A3.MIN_V, True)
    if seen == 0 or k <= 0.0 or 1.0 / k < A3.MIN_ROAD_R:
      continue
    per_tick = []
    for r in ticks:
      if r.get("icbmSrc") != "map":
        continue
      mv, md, ve = fnum(r.get("mapV")), fnum(r.get("mapDist")), fnum(r.get("vEgo"))
      ref = fnum(r.get("icbmC")) or fnum(r.get("stockSet")) or fnum(r.get("vSet"))
      if not mv or not md or md <= 0 or not ve or not ref:
        continue
      p = A3.model_target(V, mv, md, ref, ve, None, False, floor_raw=True)
      if p is not None:
        per_tick.append((md, p, ref, fnum(r.get("spdLim")) or 0.0, r))
    if not per_tick:
      continue
    T = min(p for _, p, _, _, _ in per_tick)
    entry = [(d, r) for d, _, _, _, r in per_tick if d <= hi]
    s_entry = ((fnum(entry[0][1].get("stockSet")) or fnum(entry[0][1].get("vEgo")) or float("inf"))
               if entry else float("inf"))
    elig = []
    for d, p, ref, posted, r in per_tick:
      if not (lo <= d <= hi):
        continue
      reach_m = fnum(r.get("mdlEndX"))
      if reach_m is None or d > reach_m:
        continue
      v_vis, _ = A3.vision_speed(r, mode, a_lat_vis)
      if v_vis is None or v_vis <= p + 1e-9:
        continue
      elig.append((p, ref, posted, v_vis, d, r))
    row = {"t": e["t0"], "drive": A3.drive_of(ticks[0]["_src"]), "k": k, "v0": v0,
           "v_apex": v_apex or 0.0, "T": T, "s_entry": s_entry, "ksrc": ksrc,
           "n_elig": len(elig), "map_v": fnum(tmin_rec.get("mapV")) or 0.0,
           "ref": max(x[2] for x in per_tick), "posted": max(x[3] for x in per_tick),
           "dists": [round(d) for d, _, _, _, _ in per_tick], "cf": {}}
    for N in N_list:
      if N == 0 or not elig:
        row["cf"][N] = T
        continue
      cand = []
      for p, ref, posted, v_vis, _, _ in elig:
        cap = ref - CP.ICBM_MIN_DROP_MS
        if posted > 0.0:
          cap = min(cap, posted)
        cand.append(min(p + min(N * MPH, v_vis - p), max(cap, p)))
      row["cf"][N] = max(T, min(s_entry, min(cand), T + N * MPH))
    out.append(row)
  return out


def main():
  V = A3.veh()
  a_lat_vis = V.icbm_lead_lat_accel
  recs = sorted((json.loads(x) for x in open(A3.TICKS)), key=lambda r: r["t"])
  by_t = {r["t"]: r for r in recs}
  times = sorted(by_t)
  eps = A3.build_episodes(recs)
  N_list = [0, 2, 5, 10]

  # ---- exposure: how much driving is behind these episodes -----------------------------------
  drive_s = defaultdict(float)
  dist_m = defaultdict(float)
  prev = {}
  for r in recs:
    src = r["_src"]
    v = fnum(r.get("vEgo")) or 0.0
    if src in prev:
      dt = r["t"] - prev[src]
      if 0.0 < dt < 5.0:
        drive_s[src] += dt
        dist_m[src] += v * dt
    prev[src] = r["t"]
  # one file per generation -> de-duplicate by drive folder using the max, not the sum
  by_drive_s, by_drive_m = defaultdict(float), defaultdict(float)
  for src in drive_s:
    by_drive_s[A3.drive_of(src)] += drive_s[src]
    by_drive_m[A3.drive_of(src)] += dist_m[src]
  tot_h = sum(by_drive_s.values()) / 3600.0
  tot_mi = sum(by_drive_m.values()) / 1609.34
  print(f"EXPOSURE (Ford, moving ticks, duplicate generations summed -> an OVER-count of exposure, "
        f"i.e. a conservative UNDER-count of the firing rate):")
  print(f"   {tot_h:.1f} driving hours, {tot_mi:.0f} miles across {len(by_drive_s)} corpora")

  print(f"\n{'=' * 110}\nWINDOW SENSITIVITY (vision witness = vislat_bold; a_lat at "
        f"max(measured apex speed, counterfactual target))")
  print(f"{'window':<14}{'N':>4}{'episodes':>10}{'engaged':>9}{'dV med':>9}{'a_lat med':>11}"
        f"{'p90':>8}{'max':>8}{'>4.5':>6}{'>5.0':>6}")
  for lo, hi in ((50.0, 150.0), (50.0, 100.0), (100.0, 150.0), (0.0, 50.0), (0.0, 300.0)):
    rows = [r for r in analyse(recs, times, by_t, eps, V, a_lat_vis, lo, hi, "vislat_bold", N_list)
            if r["v0"] > 25 * MPH]
    for N in N_list:
      al, dv, eng = [], [], 0
      for r in rows:
        vcf = max(r["v_apex"], r["cf"][N])
        vb = max(r["v_apex"], r["T"])
        if vcf > vb + 1e-6:
          eng += 1
          dv.append((vcf - vb) / MPH)
        al.append(r["k"] * vcf * vcf)
      s = sorted(al)
      print(f"{f'{lo:.0f}-{hi:.0f} m':<14}{N:>4}{len(rows):>10}{eng:>9}"
            f"{(st.median(dv) if dv else 0):>9.2f}{st.median(al):>11.2f}"
            f"{s[min(int(.9 * len(s)), len(s) - 1)]:>8.2f}{max(al):>8.2f}"
            f"{sum(1 for x in al if x > 4.5):>6}{sum(1 for x in al if x > 5.0):>6}")

  # ---- the episodes that go over, named ------------------------------------------------------
  for mode in ("vislat_bold", "vislat_strict", "kvis"):
    rows = [r for r in analyse(recs, times, by_t, eps, V, a_lat_vis, 50.0, 150.0, mode, N_list)
            if r["v0"] > 25 * MPH]
    print(f"\n{'=' * 110}\nEPISODES OVER 3.0 m/s^2 AT ANY N  --  vision witness = {mode}  "
          f"(n={len(rows)} episodes)")
    print(f"{'PT time':<20}{'corpus':<38}{'R m':>6}{'mapV':>6}{'set':>5}{'post':>5}"
          + "".join(f"{f'a@N{N}':>8}" for N in N_list)
          + "".join(f"{f'mph{N}':>7}" for N in N_list) + "  ksrc  candDists")
    flagged = [r for r in rows if max(r["k"] * max(r["v_apex"], r["cf"][N]) ** 2
                                      for N in N_list) > 3.0]
    for r in sorted(flagged, key=lambda r: -r["k"] * max(r["v_apex"], r["cf"][10]) ** 2):
      print(f"{datetime.datetime.fromtimestamp(r['t'], PT):%m-%d %H:%M:%S}    "
            f"{r['drive'][:36]:<38}{1.0 / r['k']:>6.0f}{r['map_v'] / MPH:>6.0f}"
            f"{r['ref'] / MPH:>5.0f}{r['posted'] / MPH:>5.0f}"
            + "".join(f"{r['k'] * max(r['v_apex'], r['cf'][N]) ** 2:>8.2f}" for N in N_list)
            + "".join(f"{max(r['v_apex'], r['cf'][N]) / MPH:>7.1f}" for N in N_list)
            + f"  {r['ksrc']:<8}{r['dists'][:12]}")
    # engagement rate in real terms
    for N in (2, 5, 10):
      eng = sum(1 for r in rows
                if max(r["v_apex"], r["cf"][N]) > max(r["v_apex"], r["T"]) + 1e-6)
      gains = [(max(r["v_apex"], r["cf"][N]) - max(r["v_apex"], r["T"])) / MPH for r in rows
               if max(r["v_apex"], r["cf"][N]) > max(r["v_apex"], r["T"]) + 1e-6]
      print(f"   N={N:<3} engages on {eng}/{len(rows)} episodes = "
            f"{eng / max(tot_h, 1e-9):.2f}/driving-hour, {eng / max(tot_mi, 1e-9) * 100:.2f}/100 mi; "
            f"median gain {st.median(gains) if gains else 0:.2f} mph, "
            f"total {sum(gains) if gains else 0:.0f} mph-episodes")


if __name__ == "__main__":
  main()
