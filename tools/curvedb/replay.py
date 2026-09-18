#!/usr/bin/env python3
"""curvedb replay -- CURVEDB2PNW.md section 7's go/no-go gate. THE deliverable of Phase 2 offline.

    PYTHONPATH=. python3 tools/curvedb/replay.py --observations obs.jsonl --episodes eps.jsonl

Section 7, in full: *"Rebuild the database retrospectively from the Phase-1 corpus and replay against
every logged ICBM episode, counting BOTH phantoms refuted AND real slowdowns wrongly cancelled. Ship
only on zero of the latter."* Section 3.9 adds the two conditions without which "zero" is not an
honest claim: **a stated N** (item 5) and **no circularity** (item 6).

## The three things that would make a flattering result meaningless, and what is done about each

**1. Circularity.** If the row that refutes an episode was built from that same episode's pass, the
replay is asking the data whether it agrees with itself. Section 3.9 item 6 requires
**leave-one-date-out**, so `_lodo_db` rebuilds the database from every observation EXCEPT those
sharing the episode's PT date, and `_assert_lodo` re-checks that property on every single episode
rather than trusting the construction.

**2. A missing denominator.** "Zero false cancels" over three episodes is not a result. Every count
here is printed with its N, and a zero is converted to the bound it actually supports: with no
events in N trials the 95 % upper bound on the rate is ~3/N (the rule of three). A bound is stated
even when it is embarrassing.

**3. A gate that cannot tell its success path from its failure path.** If the matcher never matched,
every episode would be "not acted on" and the false-cancel count would be a clean zero -- the shape
of the `getfattr` failure, where a uniform result was read as a finding instead of as a broken
method. Three defences:

* the funnel is printed in full, so "0 false cancels" is always read next to "0 episodes the
  database could act on at all";
* `--self-match` replays WITHOUT leave-one-date-out. That must find far more rows than the LODO
  run. If it does not, the keying is broken rather than the corpus being thin, and those are
  completely different conclusions;
* `--control k-shuffle` / `--control site-shuffle` deliberately corrupt the database and re-run. A
  measurement that is doing work must get WORSE when its content is scrambled. If scrambling
  changes nothing, nothing was being measured.

## "Real slowdown", defined numerically -- and the one place this file interprets the design

Section 3.9 item 6 defines a real slowdown as measured `achLat >= 2.5 m/s^2` at the speed driven,
hands-off. **Read literally that is circular in the opposite direction:** ICBM's whole job is to
reduce the speed driven, so a slowdown that worked leaves a LOW measured `achLat` and would be
scored a phantom. Every successful slowdown would be evidence for cancelling it.

The non-circular form of the same test is the COUNTERFACTUAL, and it is available precisely because
section 4 stores curvature and not speed: curvature is a property of the road, so

    a_counterfactual = k_truth * v_allowed^2

is the lateral acceleration the truck WOULD have pulled at the speed the database would have let it
hold. `k_truth` comes from the episode's own pass (ground truth, and not circular -- it is not in
the database, leave-one-date-out removed it). Both readings are reported:

* `real_cf`   -- counterfactual >= `real_slowdown_a_lat_ms2`. THE headline number.
* `real_meas` -- the literal reading, max hands-off |achLat| measured over the site extent.

They are printed side by side and neither is hidden.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import statistics
import sys
from collections import Counter
from dataclasses import dataclass, replace

from openpilot.tools.curvedb.ingest import load_episodes, load_observations
from openpilot.tools.curvedb.store import (
  PROVISIONAL_ENVELOPES,
  PROVISIONAL_PARAMS,
  CarEnvelope,
  CurveDB,
  CurveDBParams,
  Observation,
  authority,
  cancel_target,
)

# --- PROVISIONAL replay constants ------------------------------------------------------------
# The design's own comfort target, and section 3.9 item 6's definition of a real slowdown.
REAL_SLOWDOWN_A_LAT_MS2 = 2.5
# PROVISIONAL. Below this the road was not asking for a slowdown at any speed in play, so cancelling
# it refuted a phantom. Between this and REAL it is a GREY band, reported separately and never
# quietly counted as a win -- the design does not define this number and this file will not pretend
# it does.
PHANTOM_A_LAT_MS2 = 1.5
# A target raised by less than this is arithmetic noise, not an action.
ACT_EPS_MS = 0.1


@dataclass(frozen=True)
class Outcome:
  episode: dict
  matched: bool
  auth_reason: str
  v_row_ms: float | None
  new_target_ms: float
  acted: bool
  a_cf: float | None            # counterfactual lateral accel at the speed the DB would allow
  a_at_icbm: float | None       # what ICBM's own target implied
  a_at_ref: float | None        # what the driver's set speed implied
  k_verdict: float | None       # the curvature the verdict was taken on
  k_v_ego: float                # the speed that curvature was MEASURED at (see _verdict_curvature)
  verdict: str                  # see VERDICTS


VERDICTS = ("no_row", "no_authority", "no_action", "false_cancel", "grey", "phantom_refuted",
            "no_ground_truth")


def _verdict_curvature(episode: dict) -> tuple[float | None, float]:
  """The curvature a cancel is judged against, and the speed it was measured at.

  **The MAX of the site's own extent and the whole lookahead window, deliberately.** Attributing an
  episode to a map node is imprecise (mapd targets are dense and ICBM decides 300-500 m out), so
  judging a cancel only on the node that happened to be chosen would let a genuine curve 200 m
  further on go unpunished. If there was a demanding bend anywhere in the 500 m the truck was about
  to drive, cancelling the slowdown was wrong regardless of which node the database keyed on. This
  makes the gate STRICTER and removes the site-attribution choice from the verdict.

  The measuring speed travels with it, because k and the speed it was seen at are only meaningful
  together: k = 0.14 (R = 7 m) at 6 m/s is an intersection turn, and multiplying it by a 29 m/s
  counterfactual invents a 118 m/s^2 curve that nothing ever drove."""
  best, best_v = None, 0.0
  for k_key, v_key in (("k_truth", "k_v_ego"), ("k_ahead_max", "k_ahead_v_ego")):
    k = episode.get(k_key)
    if isinstance(k, int | float) and math.isfinite(k) and k > 0.0 and (best is None or k > best):
      v = episode.get(v_key)
      best, best_v = k, (float(v) if isinstance(v, int | float) and math.isfinite(v) else 0.0)
  return best, best_v


def rule_of_three(n: int) -> float | None:
  """95 % upper bound on a rate given ZERO events in n trials. None for n = 0, because "no trials"
  bounds nothing at all and printing 0 there would be a lie with a percent sign on it."""
  return None if n <= 0 else 3.0 / n


class _DBCache:
  """Leave-one-date-out databases, built once per key. Rebuilding is the honest way to do this --
  removing rows from a built DB would leave anchors that a removed observation had placed.

  **The key is (date, drive), not date alone** (Fable 2026-09-17). Section 3.9 item 6 asks for
  leave-one-DATE-out, but an observation's date is the PT date of the *passage* while an episode's
  is the PT date of the *decision*. A drive crossing PT midnight between the two puts the episode's
  own pass on the far side of the boundary and straight back into the database that judges it.
  Measured: zero such episodes in this corpus -- which is exactly the kind of "it happens not to
  bite today" that must not be load-bearing."""

  def __init__(self, observations: list[Observation], params: CurveDBParams):
    self.observations = observations
    self.params = params
    self._cache: dict[tuple[str, str] | None, CurveDB] = {}

  def get(self, exclude: tuple[str, str] | None) -> CurveDB:
    if exclude not in self._cache:
      obs = (self.observations if exclude is None else
             [o for o in self.observations
              if o.date != exclude[0] and o.drive_id != exclude[1]])
      self._cache[exclude] = CurveDB.build(obs, self.params)
    return self._cache[exclude]


def _assert_lodo(db: CurveDB, date: str, drive_id: str):
  """Re-check the property rather than trusting the construction that was supposed to provide it.

  Cheap, and it is the single assumption the whole non-circularity claim rests on."""
  for row in db.rows:
    for o in row.observations:
      if o.date == date or o.drive_id == drive_id:
        raise AssertionError(f"leave-one-date-out violated: observation {o.source} (date " +
                             f"{o.date}, drive {o.drive_id}) is in the database judging the " +
                             f"episode of date {date} / drive {drive_id}")


def evaluate(episode: dict, db: CurveDB, params: CurveDBParams,
             envelopes: dict[str, CarEnvelope], *, real_a_lat: float,
             phantom_a_lat: float) -> Outcome:
  """One episode against one database. No I/O, no globals -- so a test can drive it directly."""
  site = (episode.get("site_lat"), episode.get("site_lon"), episode.get("approach_bearing"))
  row = None
  if all(isinstance(x, int | float) and math.isfinite(x) for x in site):
    row = db.match(site[0], site[1], site[2])
  auth = authority(row, posted_limit_ms=episode.get("posted_ms"),
                   highway_class=episode.get("highway_class"), platform=episode.get("car") or "",
                   params=params, envelopes=envelopes)
  ref, icbm_t = episode["ref_ms"], episode["icbm_target_ms"]
  new_target = cancel_target(ref, icbm_t, auth.v_row_ms if auth.granted else None)
  acted = new_target > icbm_t + ACT_EPS_MS

  k, k_v = _verdict_curvature(episode)
  has_k = k is not None
  a_cf = k * new_target * new_target if has_k else None
  a_icbm = k * icbm_t * icbm_t if has_k else None
  a_ref = k * ref * ref if has_k else None

  if row is None:
    verdict = "no_row"
  elif not auth.granted:
    verdict = "no_authority"
  elif not acted:
    verdict = "no_action"
  elif not has_k:
    # Acting with no measurement of what the road turned out to be is not a win and not a loss --
    # it is an un-adjudicable action, and lumping it with the wins is how a replay flatters itself.
    verdict = "no_ground_truth"
  elif a_cf >= real_a_lat:
    verdict = "false_cancel"
  elif a_cf >= phantom_a_lat:
    verdict = "grey"
  else:
    verdict = "phantom_refuted"
  return Outcome(episode=episode, matched=row is not None, auth_reason=auth.reason,
                 v_row_ms=auth.v_row_ms, new_target_ms=new_target, acted=acted,
                 a_cf=a_cf, a_at_icbm=a_icbm, a_at_ref=a_ref, k_verdict=k, k_v_ego=k_v,
                 verdict=verdict)


def run(observations: list[Observation], episodes: list[dict], params: CurveDBParams,
        envelopes: dict[str, CarEnvelope], *, lodo: bool = True,
        real_a_lat: float = REAL_SLOWDOWN_A_LAT_MS2,
        phantom_a_lat: float = PHANTOM_A_LAT_MS2) -> list[Outcome]:
  cache = _DBCache(observations, params)
  out = []
  for e in episodes:
    key = (e.get("date") or "", e.get("drive_id") or "") if lodo else None
    db = cache.get(key)
    if key is not None:
      _assert_lodo(db, key[0], key[1])
    out.append(evaluate(e, db, params, envelopes, real_a_lat=real_a_lat,
                        phantom_a_lat=phantom_a_lat))
  return out


# ---------------------------------------------------------------------------------------------
# adversarial controls
# ---------------------------------------------------------------------------------------------

def shuffle_k(observations: list[Observation], seed: int) -> list[Observation]:
  """Right places, wrong curvatures. If the false-cancel count does NOT rise, the curvature
  measurement is not doing any work and the whole premise of section 4 is unsupported here."""
  ks = [o.k for o in observations]
  random.Random(seed).shuffle(ks)
  return [replace(o, k=k) for o, k in zip(observations, ks, strict=True)]


def shuffle_sites(observations: list[Observation], seed: int) -> list[Observation]:
  """Right curvatures, wrong places. If the number of episodes the DB acts on does NOT drop, the
  matcher is not selecting anything and every "match" was an accident of density.

  **Shuffled WITHIN each date** (Fable 2026-09-17). A global shuffle reassigns sites across dates
  and thereby MANUFACTURES multi-date rows the real corpus does not have, so the corrupted database
  looked MORE capable than the real one and the control's own alarm text fired for a reason
  unrelated to what it asserts. Permuting inside a date preserves the site-to-date coupling, so a
  row that survives is one the matcher built out of genuinely unrelated places -- which is the
  thing being tested."""
  by_date: dict[str, list[int]] = {}
  for i, o in enumerate(observations):
    by_date.setdefault(o.date, []).append(i)
  out = list(observations)
  rng = random.Random(seed)
  for idxs in by_date.values():
    pts = [(observations[i].site_lat, observations[i].site_lon, observations[i].bearing_deg)
           for i in idxs]
    rng.shuffle(pts)
    for i, pt in zip(idxs, pts, strict=True):
      out[i] = replace(observations[i], site_lat=pt[0], site_lon=pt[1], bearing_deg=pt[2])
  return out


# ---------------------------------------------------------------------------------------------
# reporting
# ---------------------------------------------------------------------------------------------

def _pct(n, d):
  return "--" if not d else f"{100.0 * n / d:.1f}%"


def summarise(outcomes: list[Outcome], title: str, log=print, *, real_a_lat: float):
  n = len(outcomes)
  v = Counter(o.verdict for o in outcomes)
  acted = [o for o in outcomes if o.acted]
  adjudicable = [o for o in acted if o.a_cf is not None]
  false_cancels = [o for o in acted if o.verdict == "false_cancel"]

  log("")
  log("=" * 100)
  log(f"{title}")
  log("=" * 100)
  log(f"  episodes replayed                       {n}")
  log(f"  ... matched a row                       {sum(1 for o in outcomes if o.matched)}" +
      f"  ({_pct(sum(1 for o in outcomes if o.matched), n)})")
  for key in VERDICTS:
    if v.get(key):
      log(f"  ... verdict {key:<28} {v[key]}  ({_pct(v[key], n)})")
  log("")
  log(f"  THE GATE (section 7): episodes the database would have cancelled or reduced = {len(acted)}")
  log(f"    of those, adjudicable (k_truth present)   {len(adjudicable)}")
  log(f"    REAL SLOWDOWNS WRONGLY CANCELLED          {len(false_cancels)}" +
      "   <- section 7 ships ONLY on zero")
  log(f"    phantoms refuted                          {v.get('phantom_refuted', 0)}")
  log(f"    grey band                                 {v.get('grey', 0)}")

  bound = rule_of_three(len(adjudicable))
  if not false_cancels:
    if bound is None:
      log("    bound on the false-cancel rate            NONE -- zero adjudicable actions bounds " +
          "nothing. 0-of-0 is not evidence.")
    else:
      log(f"    bound on the false-cancel rate            <= {100 * bound:.1f}% at 95% " +
          f"(rule of three, N={len(adjudicable)})")
      log(f"    section 3.9 item 5 wants N >= 60          {'MET' if len(adjudicable) >= 60 else 'NOT MET'}")
  else:
    log(f"    false-cancel rate (point estimate)        {_pct(len(false_cancels), len(adjudicable))}" +
        f" of {len(adjudicable)} -- NOT a bound, an observed failure rate")

  if adjudicable:
    a = sorted(o.a_cf for o in adjudicable)
    log("")
    # Both readings of section 3.9 item 6, side by side, so the interpretation argued for in this
    # module's docstring can be CHECKED rather than taken on trust. (Fable 2026-09-17: the
    # docstring promised this and the code did not do it -- `a_lat_measured` was written by ingest
    # and never read here, which is the visK shape: a field that looks like evidence and is not.)
    meas = sorted(o.episode["a_lat_measured"] for o in adjudicable
                  if isinstance(o.episode.get("a_lat_measured"), int | float))
    if meas:
      log(f"  MEASURED |achLat| hands-off over the extent -- the LITERAL 3.9-6 reading (n={len(meas)}): " +
          f"p50 {statistics.median(meas):.2f} max {meas[-1]:.2f} m/s^2; " +
          f"{sum(1 for x in meas if x >= real_a_lat)} at or above {real_a_lat:.2f}")
      log("    (low BY CONSTRUCTION -- ICBM had already slowed the truck, which is exactly why " +
          "the counterfactual below is the non-circular test)")
    else:
      log("  MEASURED |achLat|: not available on any adjudicable episode")
    log(f"  counterfactual lateral accel at the speed the DB would allow (n={len(a)}):")
    log(f"    p50 {statistics.median(a):.2f}   p90 {a[int(0.9 * (len(a) - 1))]:.2f}   " +
        f"max {a[-1]:.2f}  m/s^2   (threshold {real_a_lat:.2f})")
    near = sum(1 for x in a if real_a_lat * 0.8 <= x < real_a_lat)
    log(f"    within 20% below the threshold            {near}" +
        "   <- near misses; a zero count with near misses stacked at the line is not a safe zero")

  if false_cancels:
    log("")
    log("  EVERY FALSE CANCEL, in full (this is the list that stops the project):")
    for o in false_cancels[:40]:
      e = o.episode
      log(f"    {e['date']} {e['car']:<24} k={o.k_verdict:.5f}@{o.k_v_ego:.0f}m/s " +
          f"({e['k_estimator']}) icbm {e['icbm_target_ms']:.1f} -> db {o.new_target_ms:.1f} m/s  " +
          f"a_cf={o.a_cf:.2f}  {e['source']}")
    if len(false_cancels) > 40:
      log(f"    ... and {len(false_cancels) - 40} more")
    slow = sum(1 for o in false_cancels if o.k_v_ego < 0.6 * o.new_target_ms)
    log(f"    of these, {slow} had their curvature measured below 60 % of the counterfactual " +
        "speed -- a junction or turn, not a through-road curve the truck would have taken at speed")

  reasons = Counter(o.auth_reason.split(" (")[0].split(",")[0] for o in outcomes if not o.acted)
  if reasons:
    log("")
    log("  why the database did NOT act (the denominator's own breakdown):")
    for r, c in reasons.most_common(12):
      log(f"    {c:>6}  {r}")
  return {"n": n, "acted": len(acted), "adjudicable": len(adjudicable),
          "false_cancels": len(false_cancels), "phantoms": v.get("phantom_refuted", 0),
          "grey": v.get("grey", 0), "matched": sum(1 for o in outcomes if o.matched)}


def provenance_table(observations: list[Observation], episodes: list[dict], log=print):
  log("")
  log("=" * 100)
  log("PROVENANCE -- what the rows and the ground truth are actually MADE OF")
  log("=" * 100)
  for name, counter in (
    ("observation estimator", Counter(o.estimator for o in observations)),
    ("observation site source", Counter(o.site_src for o in observations)),
    ("observation dq state", Counter(o.dq_state for o in observations)),
    ("observation car", Counter(o.car for o in observations)),
    ("episode k estimator", Counter(e.get("k_estimator") for e in episodes)),
    ("episode site source", Counter(e.get("site_src") for e in episodes)),
    ("episode car", Counter(e.get("car") for e in episodes)),
    ("episode highway class", Counter(str(e.get("highway_class")) for e in episodes)),
  ):
    log(f"  {name}:")
    for k, c in counter.most_common():
      log(f"    {c:>7}  {k}")
  dates = sorted({o.date for o in observations})
  log(f"  observation dates: {len(dates)}  ({dates[0]} .. {dates[-1]})" if dates
      else "  observation dates: NONE")


def row_table(observations: list[Observation], params: CurveDBParams, log=print):
  db = CurveDB.build(observations, params)
  multi = [r for r in db.rows if len(r.dates) >= params.min_dates
           and r.n_passes >= params.min_passes]
  log("")
  log("=" * 100)
  log("ROWS -- and how many could ever earn authority (D6: >=2 passes on >=2 dates)")
  log("=" * 100)
  log(f"  rows                                    {len(db.rows)}")
  log(f"  rows with >={params.min_passes} passes on >={params.min_dates} dates       {len(multi)}")
  if multi:
    per = Counter(len(r.dates) for r in multi)
    log(f"  dates per qualifying row                {dict(sorted(per.items()))}")
    log("  a sample of qualifying rows:")
    for r in sorted(multi, key=lambda x: -x.n_passes)[:10]:
      log(f"    ({r.site_lat:.5f},{r.site_lon:.5f}) brg {r.bearing_deg:5.1f}  " +
          f"k_up={r.k_up} k_down={r.k_down}  n={r.n_passes} dates={len(r.dates)} " +
          f"cars={','.join(c.split('_')[0] for c in r.cars)}")
  return db, multi


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__,
                               formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--observations", required=True)
  ap.add_argument("--episodes", required=True)
  ap.add_argument("--dq", choices=("clean", "any"), default="clean",
                  help="'clean' uses only passes whose disqualifier roll-up said clean; 'any' also " +
                       "admits pre-2026-09-17 passes where it is UNKNOWN (reported, never assumed)")
  ap.add_argument("--self-match", action="store_true",
                  help="also replay WITHOUT leave-one-date-out -- the diagnostic that tells a thin " +
                       "corpus apart from a broken matcher")
  ap.add_argument("--control", choices=("k-shuffle", "site-shuffle"), action="append", default=[],
                  help="corrupt the database deliberately and re-run; a measurement that is doing " +
                       "work must get WORSE")
  ap.add_argument("--seed", type=int, default=20260917)
  ap.add_argument("--site-radius-m", type=float)
  ap.add_argument("--heading-tol-deg", type=float)
  ap.add_argument("--highway-only", action="store_true",
                  help="restrict to motorway/trunk episodes. The phantom problem this project is " +
                       "about is a HIGHWAY problem; the corpus is half city driving where an ICBM " +
                       "slowdown is for a real intersection")
  ap.add_argument("--min-ref-ms", type=float,
                  help="restrict to episodes whose reference speed was at least this (m/s)")
  ap.add_argument("--allow-unknown-highway-class", action="store_true",
                  help="sensitivity run: treat an unknown highway class as not-a-ramp. This is " +
                       "LESS SAFE than the default and exists only to show what the strict rule " +
                       "costs on corpora that predate hwyClass")
  ap.add_argument("--json-out")
  args = ap.parse_args(argv)

  params = PROVISIONAL_PARAMS
  if args.site_radius_m:
    params = replace(params, site_radius_m=args.site_radius_m)
  if args.heading_tol_deg:
    params = replace(params, heading_tol_deg=args.heading_tol_deg)

  observations = load_observations(args.observations)
  episodes = load_episodes(args.episodes)
  print(f"curvedb replay -- {len(observations)} observations, {len(episodes)} episodes")
  print(f"  params: site_radius_m={params.site_radius_m} heading_tol_deg={params.heading_tol_deg} " +
        f"min_passes={params.min_passes} min_dates={params.min_dates} " +
        f"a_lat_comfort={params.a_lat_comfort_ms2}")
  print(f"  real slowdown >= {REAL_SLOWDOWN_A_LAT_MS2} m/s^2, phantom < {PHANTOM_A_LAT_MS2} m/s^2")

  if args.dq == "clean":
    kept = [o for o in observations if o.dq_state == "clean"]
    print(f"  dq filter 'clean': {len(kept)} of {len(observations)} observations " +
          f"({len(observations) - len(kept)} had an UNKNOWN disqualifier and are excluded)")
    observations = kept

  if args.highway_only:
    before = len(episodes)
    episodes = [e for e in episodes if e.get("highway_class") in ("motorway", "trunk")]
    print(f"  --highway-only: {len(episodes)} of {before} episodes are motorway/trunk")
  if args.min_ref_ms:
    before = len(episodes)
    episodes = [e for e in episodes if (e.get("ref_ms") or 0.0) >= args.min_ref_ms]
    print(f"  --min-ref-ms {args.min_ref_ms}: {len(episodes)} of {before} episodes kept")

  envelopes = PROVISIONAL_ENVELOPES
  if args.allow_unknown_highway_class:
    episodes = [dict(e, highway_class=(e.get("highway_class") or "motorway")) for e in episodes]
    print("  !! --allow-unknown-highway-class: every unknown class is being treated as motorway. " +
          "This is a SENSITIVITY RUN, not a result.")

  provenance_table(observations, episodes)
  row_table(observations, params)

  results = {}
  outcomes = run(observations, episodes, params, envelopes)
  results["lodo"] = summarise(outcomes, "LEAVE-ONE-DATE-OUT (section 3.9 item 6) -- THE RESULT",
                              real_a_lat=REAL_SLOWDOWN_A_LAT_MS2)

  if args.self_match:
    sm = run(observations, episodes, params, envelopes, lodo=False)
    results["self_match"] = summarise(
      sm, "SELF-MATCH (NO leave-one-date-out) -- DIAGNOSTIC ONLY, circular by construction",
      real_a_lat=REAL_SLOWDOWN_A_LAT_MS2)
    print("")
    print(f"  DIAGNOSTIC: self-match found {results['self_match']['matched']} rows vs " +
          f"{results['lodo']['matched']} under LODO.")
    if results["self_match"]["matched"] <= results["lodo"]["matched"]:
      print("  !! Self-match did not find MORE rows than LODO. The keying is not working -- this " +
            "is a broken matcher, not a thin corpus. Do not read the LODO numbers as a result.")

  for ctrl in args.control:
    obs2 = (shuffle_k(observations, args.seed) if ctrl == "k-shuffle"
            else shuffle_sites(observations, args.seed))
    res = summarise(run(obs2, episodes, params, envelopes),
                    f"CONTROL '{ctrl}' -- the database deliberately corrupted",
                    real_a_lat=REAL_SLOWDOWN_A_LAT_MS2)
    results[f"control_{ctrl}"] = res
    print("")
    if ctrl == "k-shuffle" and res["false_cancels"] <= results["lodo"]["false_cancels"]:
      print("  !! k-shuffle did not produce MORE false cancels than the real database. The " +
            "curvature measurement is not doing work here -- treat the headline as unsupported.")
    if ctrl == "site-shuffle" and results["lodo"]["matched"] > 0 \
       and res["matched"] >= results["lodo"]["matched"]:
      print("  !! site-shuffle matched as many rows as the real database, with positions permuted " +
            "WITHIN each date. The matcher is not selecting anything; matches are an artifact of " +
            "row density rather than of the truck having driven that place before.")

  if args.json_out:
    with open(args.json_out, "w") as f:
      json.dump(results, f, indent=2)

  print("")
  lodo = results["lodo"]
  if lodo["adjudicable"] == 0:
    print("VERDICT: NO RESULT. The database never acted on an adjudicable episode, so section 7 " +
          "has not been evaluated -- neither passed nor failed. Read the funnel above: this is a " +
          "statement about the corpus and the method, NOT about whether the idea works.")
    return 2
  if lodo["false_cancels"]:
    print(f"VERDICT: FAILS section 7 on this corpus -- {lodo['false_cancels']} real slowdown(s) " +
          f"wrongly cancelled out of {lodo['adjudicable']} adjudicable actions.")
    return 1
  b = rule_of_three(lodo["adjudicable"])
  print(f"VERDICT: zero false cancels over N={lodo['adjudicable']} adjudicable actions " +
        f"(95% upper bound {100 * b:.1f}%). Section 3.9 item 5 asks for N >= 60: " +
        f"{'MET' if lodo['adjudicable'] >= 60 else 'NOT MET'}.")
  return 0


if __name__ == "__main__":
  sys.exit(main())
