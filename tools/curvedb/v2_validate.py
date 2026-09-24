#!/usr/bin/env python3
"""curvedb v2, step 2 -- VALIDATE the yaw-derived curvature against the car's own `kPeak` before any row
is built from it. If the two disagree badly the road table is not built (the caller is told to STOP).

What is compared (every ces_events tick record with `kPeakN` > 0 and vEgo >= v_min, i.e. 2026-09-17 on):

  kPoseP  per-record PEAK of |livePose yaw| / v at the control rate -- the SAME sensor v2 reads, so this
          isolates what the log rate (qlog 5 Hz / rlog 20 Hz) and the smoothing cost.
  kPeak   per-record PEAK of max(|carState.yawRate/v|, |desiredCurvature|) -- v1's estimator and the
          design's reference. It includes the planner's COMMAND and 100 Hz wiggle, so a smoothed
          measurement is EXPECTED to read below it on straights; on curves the two should agree.

  and, per 175 m extent (what a row actually stores): v2's extent peak against the max kPeak over the
  records whose time falls inside that extent.

Time alignment is MEASURED, not assumed: the signed instantaneous `kPose` is cross-correlated against
v2's raw k over +-2 s, and the best lag is reported and applied.

Run:
  PYTHONPATH=<worktree> python tools/curvedb/v2_validate.py --tracks <dir> --ces <ces files...> [--kind qlog]
"""
from __future__ import annotations

import argparse
import bisect
import sys
from collections import Counter

from openpilot.tools.curvedb import roadtable as rt
from openpilot.tools.curvedb import v2_io

CURVE_K = 1e-3        # "on a curve": R <= 1 km. Below this the ratio is dominated by noise.


def window_peak(ts, vals, t0: float, t1: float):
  """max |v| over samples with t0 < t <= t1 (None values skipped); None if the window is empty."""
  i = bisect.bisect_right(ts, t0)
  best = None
  while i < len(ts) and ts[i] <= t1:
    if vals[i] is not None:
      a = abs(vals[i])
      best = a if best is None else max(best, a)
    i += 1
  return best


def interp(ts, vals, t: float):
  """Linear interpolation of vals at t (None if outside, or a neighbor is None)."""
  i = bisect.bisect_left(ts, t)
  if i == 0 or i >= len(ts) or vals[i] is None or vals[i - 1] is None:
    return None
  a = (t - ts[i - 1]) / (ts[i] - ts[i - 1]) if ts[i] > ts[i - 1] else 0.0
  return vals[i - 1] + a * (vals[i] - vals[i - 1])


def ratio_stats(ref, est) -> dict:
  """Agreement of est against ref: n, bias = median(est/ref), p50/p90 of |est/ref - 1|, and the share
  of pairs where est reads more than 20 % LOW (the unsafe direction for a row)."""
  r = [e / f for f, e in zip(ref, est, strict=True) if f and f > 0 and e is not None]
  if not r:
    return {"n": 0}
  err = [abs(x - 1.0) for x in r]
  return {"n": len(r), "bias": rt.percentile(r, 50), "ratio_p10": rt.percentile(r, 10),
          "ratio_p90": rt.percentile(r, 90), "abs_err_p50": rt.percentile(err, 50),
          "abs_err_p90": rt.percentile(err, 90), "low20": sum(x < 0.8 for x in r) / len(r)}


def best_lag(ces_t, ces_k, ts, ks, lags) -> tuple[float, float, int]:
  """(lag, median |diff|, n) minimizing the median abs difference of signed k on curves."""
  best = (0.0, float("inf"), 0)
  for lag in lags:
    d = []
    for t, k in zip(ces_t, ces_k, strict=True):
      if k is None or abs(k) < CURVE_K:
        continue
      m = interp(ts, ks, t + lag)
      if m is not None:
        d.append(abs(m - k))
    if d:
      med = rt.percentile(d, 50)
      if med < best[1]:
        best = (lag, med, len(d))
  return best


def fmt(name, s):
  if not s.get("n"):
    return f"  {name:<46} n=0  <- NOTHING PAIRED: suspect the method, not the data"
  return (f"  {name:<46} n={s['n']:>6}  bias(median ratio) {s['bias']:.3f}  ratio p10/p90 " +
          f"{s['ratio_p10']:.3f}/{s['ratio_p90']:.3f}  |err| p50 {s['abs_err_p50']:.3f} p90 {s['abs_err_p90']:.3f}" +
          f"  >20% low {100 * s['low20']:.1f}%")


def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--tracks", required=True)
  ap.add_argument("--kind", default="qlog")
  ap.add_argument("--ces", nargs="+", required=True)
  ap.add_argument("--smooth-s", type=float, nargs="*", default=[rt.PROVISIONAL_V2.smooth_s])
  a = ap.parse_args(argv)

  ces = [r for r in v2_io.load_ces(a.ces) if r.get("ev") == "tick" and r.get("kPeakN")
         and (r.get("vEgo") or 0) >= rt.PROVISIONAL_V2.v_min_ms]
  print(f"ces tick records with kPeakN>0 and vEgo>={rt.PROVISIONAL_V2.v_min_ms}: {len(ces)}")
  if not ces:
    print("NOTHING TO VALIDATE AGAINST -- refusing to report agreement on an empty set")
    return 2
  t_lo, t_hi = ces[0]["t"] - 3600, ces[-1]["t"] + 3600
  routes = v2_io.list_routes(a.tracks, a.kind)
  tally: Counter = Counter()
  exit_code = 0
  for smooth in a.smooth_s:
    params = rt.V2Params(**{**rt.PROVISIONAL_V2.__dict__, "smooth_s": smooth})
    pairs = {"kPoseP~raw": ([], []), "kPoseP~smooth": ([], []), "kPeak~raw": ([], []),
             "kPeak~smooth": ([], []), "extent kPeak~smooth": ([], []),
             "extent kPoseP~smooth": ([], []),
             "extent kPeak~smooth, SUSTAINED curve": ([], []),
             "extent kPeak~smooth, SUSTAINED + CLEAN (no drv/lc/blnk)": ([], []),
             "extent CAN yawRate (smoothed)~v2 [independent sensor]": ([], [])}
    lag_rows = []
    n_routes = 0
    for route, files in sorted(routes.items()):
      doc = v2_io.load_route(files, tally)
      if doc is None or not doc["gps"]:
        continue
      samples = rt.fuse(doc, doc["off_s"], params)
      if not samples or samples[-1].t < t_lo or samples[0].t > t_hi:
        continue
      rec = [r for r in ces if samples[0].t - 1 <= r["t"] <= samples[-1].t + 1]
      if not rec:
        continue
      n_routes += 1
      ts = [s.t for s in samples]
      kraw = [s.k_raw for s in samples]
      ksm = [s.k for s in samples]
      lag, med, n = best_lag([r["t"] for r in rec], [r.get("kPose") for r in rec], ts, kraw,
                             [x / 10.0 for x in range(-20, 21)])
      lag_rows.append((route, lag, med, n))
      if n < 20:
        lag = 0.0        # too few curve witnesses to trust a lag; use none and count it
        tally["route: lag not measurable (<20 curve witnesses), 0 used"] += 1
      for r in rec:
        t1 = r["t"] + lag
        kr = window_peak(ts, kraw, t1 - 1.0, t1)
        ks = window_peak(ts, ksm, t1 - 1.0, t1)
        if r.get("kPoseP") and r["kPoseP"] >= CURVE_K:
          pairs["kPoseP~raw"][0].append(r["kPoseP"])
          pairs["kPoseP~raw"][1].append(kr)
          pairs["kPoseP~smooth"][0].append(r["kPoseP"])
          pairs["kPoseP~smooth"][1].append(ks)
        if r.get("kPeak") and r["kPeak"] >= CURVE_K:
          pairs["kPeak~raw"][0].append(r["kPeak"])
          pairs["kPeak~raw"][1].append(kr)
          pairs["kPeak~smooth"][0].append(r["kPeak"])
          pairs["kPeak~smooth"][1].append(ks)
      # extent level: what a ROW stores
      rt_ = [r["t"] + lag for r in rec]
      # third witness: carState.yawRate (CAN; alive on the Lightning only), same fusion + smoothing
      cs_t = [c[0] + doc["off_s"] for c in doc["cs"]]
      cs_k = rt.smooth_signed(cs_t, [rt.curvature(c[6], c[1], params.v_min_ms) if c[6] else None
                                     for c in doc["cs"]], params.smooth_s)
      for pts in rt.resample(samples, params):
        for i in range(0, len(pts), 4):          # every 100 m, to keep extents roughly independent
          kx, why = rt.extent_peak(pts, i, params.extent_back_m, params.extent_fwd_m)
          if kx is None:
            continue
          s0 = pts[i].s
          ext = [p for p in pts if s0 - params.extent_back_m <= p.s <= s0 + params.extent_fwd_m]
          j0 = bisect.bisect_left(rt_, ext[0].t)
          j1 = bisect.bisect_right(rt_, ext[-1].t)
          kp = [rec[j]["kPeak"] for j in range(j0, j1) if rec[j].get("kPeak")]
          if len(kp) >= 3 and max(kp) >= CURVE_K:
            pairs["extent kPeak~smooth"][0].append(max(kp))
            pairs["extent kPeak~smooth"][1].append(kx)
          if len(kp) >= 3 and max(kp) >= CURVE_K:
            m = max(kp)
            sustained = sum(k >= 0.7 * m for k in kp) >= 3
            clean = not any(any(x in (rec[j].get("dqWhy") or "") for x in ("drv", "lc", "blnk"))
                            for j in range(j0, j1))
            if sustained:
              pairs["extent kPeak~smooth, SUSTAINED curve"][0].append(m)
              pairs["extent kPeak~smooth, SUSTAINED curve"][1].append(kx)
              if clean:
                pairs["extent kPeak~smooth, SUSTAINED + CLEAN (no drv/lc/blnk)"][0].append(m)
                pairs["extent kPeak~smooth, SUSTAINED + CLEAN (no drv/lc/blnk)"][1].append(kx)
          kc = window_peak(cs_t, cs_k, ext[0].t - 1e-6, ext[-1].t)
          if kc is not None and kc >= CURVE_K:
            pairs["extent CAN yawRate (smoothed)~v2 [independent sensor]"][0].append(kc)
            pairs["extent CAN yawRate (smoothed)~v2 [independent sensor]"][1].append(kx)
          kq = [rec[j]["kPoseP"] for j in range(j0, j1) if rec[j].get("kPoseP")]
          if len(kq) >= 3 and max(kq) >= CURVE_K:
            pairs["extent kPoseP~smooth"][0].append(max(kq))
            pairs["extent kPoseP~smooth"][1].append(kx)
    lags = [x[1] for x in lag_rows if x[3] >= 20]
    print(f"\n=== smooth_s={smooth}  routes paired: {n_routes}")
    if lags:
      print(f"  measured clock lag (v2 minus ces), over {len(lags)} routes with >=20 curve witnesses: " +
            f"median {rt.percentile(lags, 50):+.1f} s, range {min(lags):+.1f}..{max(lags):+.1f} s")
    for name, (ref, est) in pairs.items():
      s = ratio_stats(ref, est)
      print(fmt(name, s))
      if not s.get("n"):
        exit_code = 1
    if smooth == a.smooth_s[0]:
      print("  per-route lag:", ", ".join(f"{r[:8]} {lg:+.1f}s(n={n})" for r, lg, _, n in lag_rows))
  for k, v in sorted(tally.items()):
    print(f"  [tally] {k}: {v}")
  return exit_code


if __name__ == "__main__":
  sys.exit(main())
