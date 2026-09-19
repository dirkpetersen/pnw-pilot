#!/usr/bin/env python3
"""A3: the BOUNDED VISION GIVE-BACK counterfactual, swept over N.

THE RULE UNDER TEST (owner, 2026-09-19)
---------------------------------------
On a MAP-sourced ICBM curve target, while the candidate sits between 50 m and 150 m ahead, and only
where vision is measurably trustworthy, let vision hand back at most N mph of the slowdown. Never
cancel it, never above the driver's set, never above the posted limit, never turn a reduction into
an increase.

    eligible tick:  src == "map"  and  50 <= mapDist <= 150
                    and a vision witness exists
                    and the candidate is inside the model's own horizon  (mapDist <= mdlEndX)
                    and vision's implied safe speed v_vis > the map target P_i
    give_i       =  min(N mph, v_vis - P_i)
    P'_i         =  min(P_i + give_i, ref - ICBM_MIN_DROP_MS, posted)      # posted only if known
    (P'_i >= P_i always: a give-back can only RAISE a target, never lower one)

THE FORWARD MODEL IS NOT NEW. `model_target()` is copied verbatim from
_scratch/icbmslow/a5_replay.py (the model validated to a median +0.03..+0.10 m/s residual against
logged `icbmT` on every corpus from 2026-08-12 on), run with floor_raw=True so the baseline is the
SHIPPED map-rating floor (`icbm_map_floor_frac` 1.0, live on origin/3devpnw @ d0a6b08abc), not the
pre-floor behaviour.

TWO EXTENSIONS, both stated rather than smuggled in:

 1. EPISODE BINDING LEVEL WITH THE EXECUTOR'S RATCHET. ICBM caps are DEC-only -- once the SET- walk
    has taken the stock set down, only the separate guarded restore can raise it (ces_pnw.py
    IcbmEpisode.step: "once a low value is tapped, only the ... RESTORE can undo it"). So a
    give-back at 120 m cannot recover speed the walk already gave away before 150 m. Modelled from
    MEASUREMENT, not simulation:

        T          = min over the episode's map ticks of P_i            (baseline, as icbmslow)
        T'_window  = min over ELIGIBLE ticks of P'_i
        S_entry    = the MEASURED stock set at the first tick with mapDist <= 150 m
        T'         = max(T, min(S_entry, T'_window))

    `min(S_entry, ...)` is the ratchet: the give-back can only arrest a walk that has not already
    passed it. This is a LATCHED give-back -- it is assumed to hold for the rest of the approach.
    That is the PERMISSIVE reading and therefore the right one for a safety question; the unlatched
    version (the target drops back inside 50 m and the walk resumes at 1 mph / 0.4 s) recovers
    strictly less, and the erosion is quantified separately.

 2. SCORING AT THE SPEED THE TRUCK WOULD ACTUALLY HAVE REACHED. icbmslow scored a_lat = k * target^2.
    That is the speed ICBM asked for, not the one the truck met. Here:

        v_cf = max(v_apex_measured, T')

    which is a rigorous UPPER bound: the walk is monotone, so the set at the apex under the
    counterfactual lies in [v_apex_measured, T'] whenever T' > v_apex_measured, and equals
    v_apex_measured otherwise. At N=0 this returns the measured apex speed exactly, so the baseline
    row is what the truck did, not a model of it. a_lat = k_truth * v_cf^2.
    The icbmslow-comparable a_lat = k * T'^2 is reported alongside.

TRUTH CURVATURE: max(kPeak, |slKActl|) over the apex window, with kPeak (the per-second peak,
2026-09-17 on) preferred where present -- it is >= the 1 Hz |slKActl| sample on 99.2 % of the ticks
that carry both. The slKActl-only variant is reported for comparability with ICBMSLOW2PNW.md.
Tesla is excluded at the scan (ICBM does not exist there and slKActl is 0 % live).

Rule 2: every episode that drops out is counted under a named reason; a variant that never engages
is reported as engaging zero times rather than averaged away.
"""
import bisect
import json
import math
import statistics as st
import sys
from collections import defaultdict

WT = "/home/dp/gh/comma/pnw/wt-icbmslow"     # origin/3devpnw + icbmslow2pnw == what the car runs
sys.path.insert(0, WT)
sys.path.insert(0, WT + "/opendbc_repo")

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as CP   # noqa: E402
from openpilot.selfdrive.controls.lib import pnw_vehicle as pv       # noqa: E402

TICKS = "/home/dp/gh/comma/_scratch/visionback/ticks.jsonl"
MPH = 0.44704
GAP_S = 3.0
MIN_V = 8.0
HANDS_OFF_LIMIT = 4.5          # docs/LIGHTNING-STEERING-LIMITS.md measured hands-off lateral ceiling
A_LAT_DESIGN = 2.5
MIN_ROAD_R = 30.0
LO, HI = 50.0, 150.0           # the give-back window (m to the candidate)
N_SWEEP = [0, 2, 5, 10, 15]    # mph; 0 is the shipped baseline, 15 an over-range probe


class FakeCP:
  carFingerprint = "FORD_F_150_LIGHTNING_MK1"
  brand = "ford"
  openpilotLongitudinalControl = False


def veh():
  pv.CURVE_CONFIG_PATH = "/nonexistent/curve.json"     # defaults only -> icbm_map_floor_frac = 1.0
  return pv.PnwVehicle(FakeCP())


def fnum(x):
  try:
    v = float(x)
  except (TypeError, ValueError):
    return None
  return v if math.isfinite(v) else None


def drive_of(src):
  if src.startswith("arch/"):
    return "ARCHIVE(continuous)"
  p = src.split("/")
  return "/".join(p[:2]) if len(p) > 2 else p[0]


# --- VERBATIM from _scratch/icbmslow/a5_replay.py (the validated forward model) -------------------
def model_target(v, map_v, map_d, ref, v_ego, pitch, is_left, floor_raw=True):
  t, _, _ = CP.icbm_curve_target(v_ego, ref, map_v, map_d, None, CP.icbm_map_eff_scale,
                                 0.0, float("inf"), map_scale=v.icbm_map_scale,
                                 firm_decel=v.icbm_firm_decel, track=True)
  if t is None:
    return None
  if not floor_raw:
    return max(t - v.curve_speed_penalty_ms(t, pitch_rad=pitch, is_left=is_left), 0.0)
  pen_base = v.curve_speed_penalty_ms(t)
  pen_full = v.curve_speed_penalty_ms(t, pitch_rad=pitch, is_left=is_left)
  floor = v.icbm_map_floor_ms(map_v)
  return max(max(t - pen_base, min(floor, t)) - max(pen_full - pen_base, 0.0), 0.0)
# -------------------------------------------------------------------------------------------------


def build_episodes(recs):
  out, cur = [], None
  for r in recs:
    t = r["t"]
    dec = (r.get("icbmT") is not None and r.get("icbmSrc") in ("map", "far", "vis"))
    if dec and cur is not None and t - cur["t_end"] <= GAP_S:
      cur["ticks"].append(r)
      cur["t_end"] = t
    elif dec:
      if cur is not None:
        out.append(cur)
      cur = {"t0": t, "t_end": t, "ticks": [r]}
  if cur is not None:
    out.append(cur)
  return out


def apex_witness(times, by_t, t_lo, t_hi, v_floor, use_kpeak):
  """Tightest curvature the truck MEASURED over the window, and the speed it was doing there."""
  k_best, v_at, t_at, seen, src = 0.0, None, None, 0, None
  i = bisect.bisect_left(times, t_lo)
  while i < len(times) and times[i] <= t_hi:
    r = by_t[times[i]]
    i += 1
    v = fnum(r.get("vEgo"))
    if v is None or v < v_floor:
      continue
    cands = []
    ka = fnum(r.get("slKActl"))
    if ka is not None and abs(ka) > 1e-9:
      cands.append((abs(ka), "slKActl"))
    if use_kpeak:
      kp = fnum(r.get("kPeak"))
      if kp is not None and abs(kp) > 1e-9:
        cands.append((abs(kp), "kPeak"))
    if not cands:
      continue
    seen += 1
    k, s = max(cands)
    if k > k_best:
      k_best, v_at, t_at, src = k, v, r["t"], s
  return k_best, v_at, t_at, seen, src


def vision_speed(r, mode, a_lat):
  """vision's implied SAFE SPEED at this tick (m/s), or None = vision refuses / not usable.
  Returns (v_vis, why). float('inf') means 'vision sees no curve at all here'."""
  v_ego = fnum(r.get("vEgo"))
  if v_ego is None or v_ego <= 0:
    return None, "noVego"
  if mode == "kvis":
    k = fnum(r.get("icbmKVis"))
    if k is None:
      return None, "noKvis"
    if k <= 1e-9:
      return float("inf"), "straight"
    return math.sqrt(a_lat / k), "ok"
  # visLat modes -- ICBM's own vision candidate generator
  vl = fnum(r.get("visLat"))
  ttc = fnum(r.get("visTtc"))
  if vl is None or ttc is None:
    return None, "noVisLat"
  vis_v, _ = CP.icbm_vision_apex(v_ego, vl, ttc)
  if vis_v <= 0.0:                       # |visLat| <= ICBM_VISION_ENTER -> "not a curve"
    if mode == "vislat_bold":
      return float("inf"), "straight"
    return None, "notACurve"             # vislat_strict: no measurement -> no give-back
  return vis_v, "ok"


def main():
  V = veh()
  a_lat_vis = V.icbm_lead_lat_accel          # 2.5 m/s^2 -- what icbm_map_sanity uses
  print(f"vehicle: FORD_F_150_LIGHTNING_MK1  map_scale={V.icbm_map_scale}  "
        f"firm_decel={V.icbm_firm_decel}  map_floor_frac={V._curve_cfg['icbm_map_floor_frac']}  "
        f"vision a_lat={a_lat_vis}")
  print(f"ICBM_MIN_DROP_MS={CP.ICBM_MIN_DROP_MS}  window={LO:.0f}-{HI:.0f} m\n")

  recs = sorted((json.loads(x) for x in open(TICKS)), key=lambda r: r["t"])
  by_t = {r["t"]: r for r in recs}
  times = sorted(by_t)
  eps = build_episodes(recs)
  print(f"ford moving ticks: {len(recs)}   DEC episodes: {len(eps)}")

  modes = ["kvis", "vislat_bold", "vislat_strict"]
  # POPULATION. `stockOn` is car_state.cruiseState.enabled (ces_pnw.py:3842). With the stock ACC
  # DISENGAGED, ICBM's SET- taps reach nothing -- icbm_pnw's executor gates every press on cruise --
  # so the published target is advisory and a give-back could not have changed the truck's speed.
  # Measured: 1,882 of 3,087 ICBM ticks (61 %) have stockOn False, and 1,310 of 1,522 (86 %) on the
  # 2026-09-12 weekend that dominates the sample. Scoring "what the truck would ACTUALLY have
  # pulled" over those is meaningless, so the two populations are reported separately rather than
  # merged.
  for pop in ("cruise_ON (ICBM could actuate)", "ALL ICBM ticks (incl. cruise off = advisory)"):
   cruise_only = pop.startswith("cruise_ON")
   for truth_mode in ("kpeak_or_slk", "slk_only"):
     rows, skip = [], defaultdict(int)
     use_kpeak = truth_mode == "kpeak_or_slk"
     for e in eps:
       ticks = e["ticks"]
       v0 = fnum(ticks[0].get("vEgo")) or 0.0
       tgts = [fnum(r.get("icbmT")) for r in ticks if fnum(r.get("icbmT")) is not None]
       if not tgts or v0 < MIN_V:
         skip["below 8 m/s at episode start, or no logged target"] += 1
         continue
       tmin_rec = min(ticks, key=lambda r: fnum(r.get("icbmT")) or 1e9)
       map_d = fnum(tmin_rec.get("mapDist")) or 0.0
       reach = map_d if map_d > 0 else 150.0
       wins = {
         "tight": (e["t0"], e["t_end"] + min(reach / max(v0, 1.0), 12.0) + 4.0, 0.6 * v0),
         "base": (e["t0"], e["t_end"] + min(reach / max(v0, 1.0) + 10.0, 40.0), MIN_V),
         "wide": (e["t0"] - 5.0, e["t_end"] + min(reach / max(v0, 1.0) + 20.0, 60.0), MIN_V),
       }
       wit = {w: dict(zip(("k", "v", "t", "seen", "src"),
                          apex_witness(times, by_t, lo, hi, vf, use_kpeak), strict=True))
              for w, (lo, hi, vf) in wins.items()}
       if wit["base"]["seen"] == 0:
         skip["no curvature witness in the apex window (corpus predates 2026-08-11)"] += 1
         continue
       k = wit["base"]["k"]
       if k <= 0.0:
         skip["witness present but curvature identically 0"] += 1
         continue
       if 1.0 / k < MIN_ROAD_R:
         skip[f"apex radius < {MIN_ROAD_R:.0f} m (junction/parking turn, not a road curve)"] += 1
         continue

       # ---- baseline + counterfactual, per tick -------------------------------------------------
       per_tick = []        # (dist, P_i, ref, posted, record)
       n_off = 0
       for r in ticks:
         if r.get("icbmSrc") != "map":
           continue
         if cruise_only and not r.get("stockOn"):
           n_off += 1
           continue
         mv, md, ve = fnum(r.get("mapV")), fnum(r.get("mapDist")), fnum(r.get("vEgo"))
         ref = fnum(r.get("icbmC")) or fnum(r.get("stockSet")) or fnum(r.get("vSet"))
         if not mv or not md or md <= 0 or not ve or not ref:
           continue
         p = model_target(V, mv, md, ref, ve, None, False, floor_raw=True)
         if p is None:
           continue
         per_tick.append((md, p, ref, fnum(r.get("spdLim")) or 0.0, r))
       if not per_tick:
         skip["no replayable MAP tick with ICBM authority (far/vision-sourced, missing inputs, "
              "or stock ACC disengaged)" if cruise_only else
              "no replayable MAP tick (far/vision-sourced, or missing mapV/mapDist/ref)"] += 1
         continue
       T = min(p for _, p, _, _, _ in per_tick)

       # measured stock set at window entry (first tick with the candidate <= HI m out)
       entry = [(d, r) for d, _, _, _, r in per_tick if d <= HI]
       if entry:
         r0 = entry[0][1]
         s_entry = fnum(r0.get("stockSet")) or fnum(r0.get("vEgo")) or float("inf")
       else:
         s_entry = float("inf")

       v_apex = wit["base"]["v"] or 0.0
       row = {"t": e["t0"], "drive": drive_of(ticks[0]["_src"]), "v0": v0, "k": k,
              "wit": wit, "T": T, "v_apex": v_apex, "s_entry": s_entry,
              "n_map": len(per_tick), "cf": {}}
       # eligibility + give-back, per vision mode and per N
       for mode in modes:
         elig, why_ct = [], defaultdict(int)
         for d, p, ref, posted, r in per_tick:
           if not (LO <= d <= HI):
             why_ct["outOfWindow"] += 1
             continue
           reach_m = fnum(r.get("mdlEndX"))
           if reach_m is None:
             why_ct["noModelReach"] += 1
             continue
           if d > reach_m:
             why_ct["candidateBeyondModelHorizon"] += 1
             continue
           v_vis, why = vision_speed(r, mode, a_lat_vis)
           if v_vis is None:
             why_ct[why] += 1
             continue
           if v_vis <= p + 1e-9:
             why_ct["visionAgreesOrTighter"] += 1
             continue
           elig.append((p, ref, posted, v_vis))
           why_ct["eligible"] += 1
         row["cf"][mode] = {"why": dict(why_ct)}
         for N in N_SWEEP:
           late = False
           if N == 0 or not elig:
             tprime = T
           else:
             cand = []
             for p, ref, posted, v_vis in elig:
               give = min(N * MPH, v_vis - p)
               cap = ref - CP.ICBM_MIN_DROP_MS
               if posted > 0.0:
                 cap = min(cap, posted)
               cand.append(min(p + give, max(cap, p)))    # max(cap,p): never LOWER the target
             tw = min(cand)
             # THE BOUND. "at most N mph of a map-commanded slowdown" is a bound on the EPISODE's
             # binding level, not only on the per-tick arithmetic: the episode minimum T can come
             # from a tick outside the window, and without this clamp holding the set high inside
             # the window would hand back far more than N (measured: up to +27 mph). Clamping here
             # is what makes the swept N mean what the question says it means.
             tprime = max(T, min(s_entry, tw, T + N * MPH))
             late = (s_entry < min(tw, T + N * MPH) - 1e-9)   # the SET- walk had already passed it
           row["cf"][mode][N] = tprime
           row["cf"][mode][("late", N)] = late
       rows.append(row)

     # ---- report ------------------------------------------------------------------------------
     print(f"\n{'=' * 118}\nPOPULATION = {pop}   TRUTH = {truth_mode}   "
           f"episodes analysed: {len(rows)}")
     for kk, n in sorted(skip.items(), key=lambda kv: -kv[1]):
       print(f"   excluded {n:4d}: {kk}")
     if not rows:
       continue
     srcs = defaultdict(int)
     for r in rows:
       srcs[r["wit"]["base"]["src"]] += 1
     print(f"   apex witness source: {dict(srcs)}")
     hi_rows = [r for r in rows if r["v0"] > 25 * MPH]
     print(f"   episodes above 25 mph: {len(hi_rows)}")
     by_drive = defaultdict(int)
     for r in hi_rows:
       by_drive[r["drive"]] += 1
     print(f"   by corpus: {dict(sorted(by_drive.items(), key=lambda kv: -kv[1]))}")

     def qs(v, p):
       v = sorted(v)
       return v[min(int(p * len(v)), len(v) - 1)] if v else float("nan")

     for mode in modes:
       print(f"\n--- vision witness = {mode} ---")
       wt = defaultdict(int)
       for r in hi_rows:
         for kk, n in r["cf"][mode]["why"].items():
           wt[kk] += n
       print(f"    in-window tick verdicts: {dict(sorted(wt.items(), key=lambda kv: -kv[1]))}")
       hdr = (f"    {'N mph':>6}{'engaged':>9}{'dV med':>9}{'dV p90':>9}{'dV max':>9}"
              f"{'alat med':>10}{'alat p90':>10}{'alat p99':>10}{'alat MAX':>10}"
              f"{'>2.5':>6}{'>4.5':>6}{'>5.0':>6}   worst site")
       print(hdr)
       import datetime
       import zoneinfo
       PT = zoneinfo.ZoneInfo("America/Los_Angeles")
       for N in N_SWEEP:
         al, dv, eng, late = [], [], 0, 0
         worst = None
         for r in hi_rows:
           tp = r["cf"][mode][N]
           v_cf = max(r["v_apex"], tp)
           v_base = max(r["v_apex"], r["T"])
           if v_cf > v_base + 1e-6:
             eng += 1
             dv.append((v_cf - v_base) / MPH)
           if r["cf"][mode][("late", N)]:
             late += 1
           a = r["k"] * v_cf * v_cf
           al.append(a)
           if worst is None or a > worst[0]:
             worst = (a, r)
         w = worst[1]
         wlbl = (f"{datetime.datetime.fromtimestamp(w['t'], PT):%m-%d %H:%M:%S} "
                 f"{w['drive']} R={1.0 / w['k']:.0f}m v={max(w['v_apex'], w['cf'][mode][N]) / MPH:.0f}mph")
         print(f"    {N:>6}{eng:>9}"
               f"{(st.median(dv) if dv else 0.0):>9.2f}{qs(dv, .9) if dv else 0.0:>9.2f}"
               f"{(max(dv) if dv else 0.0):>9.2f}"
               f"{st.median(al):>10.2f}{qs(al, .9):>10.2f}{qs(al, .99):>10.2f}{max(al):>10.2f}"
               f"{sum(1 for x in al if x > A_LAT_DESIGN):>6}"
               f"{sum(1 for x in al if x > HANDS_OFF_LIMIT):>6}"
               f"{sum(1 for x in al if x > 5.0):>6}  late{late:>3}  {wlbl}")
       # every episode whose counterfactual lateral accel clears the design target, named
       for N in (2, 5, 10):
         bad = sorted(((r["k"] * max(r["v_apex"], r["cf"][mode][N]) ** 2, r) for r in hi_rows),
                      key=lambda x: -x[0])[:5]
         print(f"    N={N:<3} top-5 a_lat: " + " | ".join(
           f"{a:.2f} @{datetime.datetime.fromtimestamp(r['t'], PT):%m-%d %H:%M:%S} R={1.0 / r['k']:.0f}m "
           f"{max(r['v_apex'], r['cf'][mode][N]) / MPH:.0f}mph(+{(max(r['v_apex'], r['cf'][mode][N]) - max(r['v_apex'], r['T'])) / MPH:.1f})"
           for a, r in bad))
       # UNLATCHED sensitivity: the give-back stops at the window's near edge (50 m) and the SET-
       # walk resumes at the executor's 1 mph / 0.4 s = 2.5 mph/s for the remaining 50 m.
       print(f"    {'N mph':>6}  UNLATCHED (give-back released at 50 m, walk resumes 2.5 mph/s):")
       for N in N_SWEEP:
         al, dv, eng = [], [], 0
         for r in hi_rows:
           v_ref = max(r["v_apex"], 1.0)
           erosion = 2.5 * MPH * (50.0 / v_ref)
           tp = max(r["T"], r["cf"][mode][N] - erosion)
           v_cf = max(r["v_apex"], tp)
           v_base = max(r["v_apex"], r["T"])
           if v_cf > v_base + 1e-6:
             eng += 1
             dv.append((v_cf - v_base) / MPH)
           al.append(r["k"] * v_cf * v_cf)
         print(f"    {N:>6}  engaged {eng:>3}  dV med {(st.median(dv) if dv else 0.0):.2f} "
               f"max {(max(dv) if dv else 0.0):.2f} mph | a_lat med {st.median(al):.2f} "
               f"p90 {qs(al, .9):.2f} max {max(al):.2f} | >4.5 {sum(1 for x in al if x > HANDS_OFF_LIMIT)}")
       # the icbmslow-comparable scoring: a_lat at the COMMANDED target
       print(f"    {'N mph':>6}{'--- a_lat at the COMMANDED target (icbmslow-comparable) ---':>62}")
       for N in N_SWEEP:
         al = [r["k"] * r["cf"][mode][N] ** 2 for r in hi_rows]
         dt = [(r["cf"][mode][N] - r["T"]) / MPH for r in hi_rows]
         print(f"    {N:>6}  dT med {st.median(dt):+6.2f} max {max(dt):+6.2f} mph | "
               f"alat med {st.median(al):.2f} p90 {qs(al, .9):.2f} max {max(al):.2f} | "
               f">2.5 {sum(1 for x in al if x > A_LAT_DESIGN)} >4.5 "
               f"{sum(1 for x in al if x > HANDS_OFF_LIMIT)}")

     if truth_mode == "kpeak_or_slk" and cruise_only:
       with open("/home/dp/gh/comma/_scratch/visionback/episodes.json", "w") as f:
         json.dump([{str(kk): vv for kk, vv in r.items() if kk != "cf"} for r in rows], f)


if __name__ == "__main__":
  main()
