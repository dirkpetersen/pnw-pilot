#!/usr/bin/env python3
"""curvedb calibration -- the two numbers the README quotes about its own PROVISIONAL constants.

    PYTHONPATH=. python3 tools/curvedb/calib.py drives/**/ces_events*.jsonl

**This file exists because those numbers previously came from an uncommitted scratch script
(Fable 2026-09-19, and it is the same D12 defect this project has been bitten by before).** A
measurement quoted in a document and a review, with no runnable thing behind it, cannot be checked,
cannot be re-run on a new corpus, and cannot be shown to be wrong. Two of them were carrying real
weight: section 3.3's "a 1 Hz row under-reads k by p50 1.10x" is the argument for logging a 100 Hz
peak at all, and section 6.3's "22 % of sites move further than the matching tolerance" is the
argument that `approach_bearing_ref_m` is not a free parameter.

Both are measured through the SAME primitives `ingest.py` uses -- the same extents, the same
estimator ladder, the same approach-bearing sampler -- so they describe the rows the pipeline
actually builds rather than a second, similar-looking pipeline written for a report.

## 1. What a 1 Hz-sampled row costs, versus the 100 Hz peak

Only the 2026-09-17 corpora carry `kPeak`, so this can only be measured where both exist. Reported
two ways, because they answer different questions:

* **per tick** -- how much a single 1 Hz sample misses. Noisy and unbounded (a sample taken between
  two peaks can miss by any factor).
* **per extent** -- how much the max over a whole section 6.3 extent misses, which is what a ROW is
  built from and therefore the number that matters.

Since `v = sqrt(a/k)`, a row under-reading k by a factor f permits a speed sqrt(f) too high. That is
the UNSAFE direction: too fast for the bend.

## 2. How far the approach bearing moves across ICBM's decision range

ICBM decides anywhere from ~150 m to its 500 m far-source horizon. If the truck's bearing at those
distances differs by more than `heading_tol_deg`, then ingest (which samples at
`approach_bearing_ref_m`) and a lookup (which would sample wherever the car happens to ask) can
disagree about which direction "this way" is, and the row is simply never found.
"""
from __future__ import annotations

import argparse
import statistics
import sys
from collections import Counter

from openpilot.tools.curvedb.ingest import (
  DQ_ALL,
  PROVISIONAL_PARAMS,
  approach_bearing,
  drive_odo_gps_ok,
  find_sites,
  load_corpus,
  measure_passage,
  split_drives,
)
from openpilot.tools.curvedb.store import CurveDBParams, bearing_diff_deg

# PROVISIONAL. The distances ICBM's own decision is taken across (`MAP_SOURCE_HORIZON_M` = 500 m
# down to the ~150 m near source). The spread across these is the cost of leaving
# `approach_bearing_ref_m` unpinned.
BEARING_REFS_M = (500.0, 300.0, 150.0)
# A 1 Hz sample missing this much of the peak is the threshold the README quotes. "Under-reads by
# more than 20 %" means sample < 0.8 * peak, i.e. peak/sample > 1.25.
UNDER_READ_RATIO = 1.25


def _sample_1hz(tk) -> float | None:
  """The 1 Hz estimator of section 3.3: max(commanded, achieved) over whichever columns exist.

  Deliberately NOT `_tick_k`, which prefers `kPeak` where it exists -- the whole point here is to
  compare the two, so this is the ladder with the 100 Hz rung removed."""
  ks = [k for k in (tk.k_actl, tk.k_cmd) if k is not None]
  return max(ks) if ks else None


def measure(ticks, params: CurveDBParams):
  """Every ratio and every bearing spread the corpus can support, with its own denominator."""
  per_tick: list[float] = []
  per_extent: list[float] = []
  spreads: list[float] = []
  vs_ref: list[float] = []
  why: Counter = Counter()
  drives = split_drives(ticks)

  for tk in ticks:
    if tk.k_peak is None:
      why["tick: no kPeak (corpus predates 2026-09-17)"] += 1
      continue
    s = _sample_1hz(tk)
    if s is None:
      why["tick: kPeak but no 1 Hz column to compare it with"] += 1
      continue
    per_tick.append(tk.k_peak / s)

  # The unit is the PASSAGE, not the site: `measure_passage` is what ingest builds a row out of, so
  # a site it rejects (the drive never reached it, no curvature anywhere in the extent, no approach
  # bearing) is not a row whose provenance or keying could ever have mattered. Every rejection is
  # counted by `ingest`'s own named reason in `drop`.
  drop: Counter = Counter()
  for d in drives:
    if not drive_odo_gps_ok(d):
      # The SAME gate ingest applies. Both measurements below are odometer-indexed -- the extent
      # boundaries and the 500/300/150 m reference points -- so a drive whose odometer and GPS
      # disagree is precisely the input that corrupts them.
      why["ingest: drive_dropped_odometer_disagrees_with_gps"] += 1
      continue
    for site in find_sites(d, params, drop):
      p = measure_passage(d, site, params, drop, DQ_ALL)
      if p is None:
        continue
      lo, hi = p.extent
      ext = [t for t in d.ticks[lo:hi] if t.v_ego >= params.min_speed_ms]
      peaks = [t.k_peak for t in ext if t.k_peak is not None]
      samples = [s for s in (_sample_1hz(t) for t in ext) if s is not None]
      if peaks and samples:
        per_extent.append(max(peaks) / max(samples))
      else:
        why["extent: no kPeak/1 Hz pair inside it"] += 1

      brgs = {r: b for r, b in ((r, approach_bearing(d, d.s[p.i_passage], r)[0])
                                for r in BEARING_REFS_M) if b is not None}
      if len(brgs) == len(BEARING_REFS_M):
        vals = list(brgs.values())
        spreads.append(max(bearing_diff_deg(a, b) for a in vals for b in vals))
        # The disagreement that can actually lose a row is between the reference INGEST samples at
        # and wherever the car happens to ask, so it is measured against `approach_bearing_ref_m`
        # specifically. The pairwise max above is an UPPER BOUND on it, not the same number
        # (Fable 2026-09-19).
        here = brgs.get(params.approach_bearing_ref_m)
        if here is None:
          why["bearing: approach_bearing_ref_m is not one of the reference distances"] += 1
        else:
          vs_ref.append(max(bearing_diff_deg(here, b) for b in vals))
      else:
        why[f"bearing: only {len(brgs)} of {len(BEARING_REFS_M)} reference points on the drive"] += 1
  # Every one of ingest's own counters except the two that say what KIND of site it was rather than
  # that it was rejected. Passed through by name instead of re-derived, so a new drop reason in
  # ingest shows up here automatically rather than vanishing into the denominator.
  for k, v in drop.items():
    if k not in ("site_track", "site_logged"):
      why[f"ingest: {k}"] += v
  return per_tick, per_extent, spreads, vs_ref, why, drives


def _stats(xs: list[float]) -> str:
  if not xs:
    return "NO DATA"
  s = sorted(xs)
  return (f"n={len(s)}  p50={statistics.median(s):.3f}  p90={s[int(0.9 * (len(s) - 1))]:.3f}  " +
          f"max={s[-1]:.3f}")


def report(per_tick, per_extent, spreads, vs_ref, why, params: CurveDBParams, n_ticks, n_drives,
           log=print):
  log("")
  log("=" * 100)
  log("CALIBRATION -- what a 1 Hz row costs, and how far the approach bearing moves")
  log("=" * 100)
  log(f"  corpus: {n_ticks} ticks, {n_drives} drives")
  log("")
  log("  1. kPeak (100 Hz) / max(1 Hz sample) -- only measurable where BOTH exist")
  log(f"     per tick    {_stats(per_tick)}")
  if per_tick:
    over = sum(1 for r in per_tick if r > UNDER_READ_RATIO)
    log(f"       the 1 Hz sample under-read by more than 20% on {over} of {len(per_tick)} ticks " +
        f"({100.0 * over / len(per_tick):.1f}%)")
  log(f"     per extent  {_stats(per_extent)}   <- THIS is what a row is built from")
  if per_extent:
    s = sorted(per_extent)
    p50, p90 = statistics.median(s), s[int(0.9 * (len(s) - 1))]
    log(f"       v = sqrt(a/k), so a row built from 1 Hz permits a speed {p50 ** 0.5:.3f}x too " +
        f"high at the median and {p90 ** 0.5:.3f}x at p90 -- the UNSAFE direction")
  log("")
  log(f"  2. approach-bearing spread across {'/'.join(f'{r:.0f}' for r in BEARING_REFS_M)} m " +
      "before a site")
  log(f"     vs the {params.approach_bearing_ref_m:.0f} m reference INGEST samples at   " +
      f"{_stats(vs_ref)} degrees   <- the disagreement that loses a row")
  if vs_ref:
    over = sum(1 for x in vs_ref if x > params.heading_tol_deg)
    log(f"       exceeding the {params.heading_tol_deg:.0f} deg matching tolerance: {over} of " +
        f"{len(vs_ref)} ({100.0 * over / len(vs_ref):.1f}%)")
    log("       on those, ingest and the car can disagree about which direction 'this way' is, " +
        "and the row is never found")
  log(f"     pairwise max over all three (an UPPER BOUND, not the same number)   {_stats(spreads)}")
  if spreads:
    over = sum(1 for x in spreads if x > params.heading_tol_deg)
    log(f"       exceeding {params.heading_tol_deg:.0f} deg: {over} of {len(spreads)} " +
        f"({100.0 * over / len(spreads):.1f}%)")
  log("")
  log("  what was NOT measurable, and why (Rule 2: an absent number is a claim, not a silence):")
  for k, v in why.most_common():
    log(f"    {v:>8}  {k}")
  if not per_tick and not per_extent:
    log("")
    log("  !! NO kPeak ANYWHERE IN THIS CORPUS. Section 1 above is unmeasured, not measured at " +
        "1.0 -- kPeak exists only from 2026-09-17.")


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__,
                               formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("paths", nargs="+", help="the same ces_events corpora ingest.py was run over")
  ap.add_argument("--quiet", action="store_true")
  args = ap.parse_args(argv)

  ticks, _ = load_corpus(args.paths, log=lambda *a: None)
  if not ticks:
    raise SystemExit("no usable ticks in any of those files -- that is a statement about the " +
                     "input, and no ratio should be read out of it")
  params = PROVISIONAL_PARAMS
  per_tick, per_extent, spreads, vs_ref, why, drives = measure(ticks, params)
  report(per_tick, per_extent, spreads, vs_ref, why, params, len(ticks), len(drives),
         log=(lambda *a: None) if args.quiet else print)
  return 0


if __name__ == "__main__":
  sys.exit(main())
