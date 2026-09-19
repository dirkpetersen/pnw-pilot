#!/usr/bin/env python3
"""curvedb recurrence -- does the truck actually RE-DRIVE the roads the phantoms are on?

    PYTHONPATH=. python3 tools/curvedb/recurrence.py --episodes eps.jsonl \\
        --observations obs.jsonl drives/**/ces_events*.jsonl

CURVEDB2PNW.md's whole premise is that a curve worth refuting gets driven repeatedly, and D6 turns
that into a rule: **>=2 admissible passes on >=2 different dates** before a row may act. This script
measures the premise directly, and it exists because measuring it *through* `ingest.py` gave the
wrong answer.

**THE DISTINCTION THIS SCRIPT EXISTS TO MAKE.** Counting episode sites that have an *observation*
from another date conflates two completely different failures:

  a. the truck never drove that road again          -> more driving is the only fix
  b. it did, and our admissibility rules binned it  -> a rule we chose is the fix

Through the pipeline the answer looked like (a). Measured from **raw GPS ticks**, bypassing
observations, passages, extents and disqualifiers entirely, it is substantially (b). Those lead to
opposite recommendations, so the measurement has to be made without the pipeline in it -- which is
also why this is a separate file rather than another counter inside `ingest.py`.

It additionally reports the number D6 actually needs and which the raw count does not give: how many
sites were revisited on **two or more** other dates. Under leave-one-out a site with exactly one
other date can never reach `min_dates=2`, so it can never produce an action however clean the
pipeline becomes -- and quoting the revisit count without that split overstates what is reachable.

Every count is printed with its denominator, and the attribution of the lost revisits is by the
disqualifier's OWN named cause, not by inference.
"""
from __future__ import annotations

import argparse
import sys
from collections import Counter

from openpilot.tools.curvedb.ingest import (
  _tick_k,
  dq_names,
  load_corpus,
  load_episodes,
  load_observations,
  pt_date,
)
from openpilot.tools.curvedb.store import (
  PROVISIONAL_PARAMS,
  CurveDBParams,
  bearing_diff_deg,
  haversine_m,
)

# Coarse spatial bucket, so this is not (episodes x ticks) haversines. Must be comfortably wider
# than any `site_radius_m` the caller passes, and the 3x3 neighbourhood is scanned, so the usable
# radius is up to ~110 m at this cell size.
CELL_DEG = 0.002        # ~220 m of latitude


def _index(ticks, params: CurveDBParams):
  grid: dict[tuple[int, int], list] = {}
  for tk in ticks:
    if tk.v_ego < params.min_speed_ms:
      continue            # a parked tick is not a pass over the site
    grid.setdefault((round(tk.lat / CELL_DEG), round(tk.lon / CELL_DEG)), []).append(tk)
  return grid


def _near(grid, lat, lon, radius_m):
  out = []
  for dla in (-1, 0, 1):
    for dlo in (-1, 0, 1):
      out += grid.get((round(lat / CELL_DEG) + dla, round(lon / CELL_DEG) + dlo), [])
  return [tk for tk in out if haversine_m(tk.lat, tk.lon, lat, lon) <= radius_m]


def analyse(ticks, episodes, observations, params: CurveDBParams):
  grid = _index(ticks, params)
  if not grid:
    raise SystemExit("no moving ticks in the corpus -- refusing to report a recurrence rate of 0 " +
                     "from an empty index (that would be a statement about the input, not the road)")
  tally = Counter()
  why = Counter()
  other_date_hist = Counter()
  # The same count per DISTINCT episode-site. ICBM re-firing at one junction inside one drive asks
  # this question twice about one road (`ingest.find_episodes` tags it `site_group`), so a raw
  # episode count overstates how many roads were actually looked at. Both are reported; neither is
  # silently substituted for the other.
  groups: dict[str, set] = {}

  def hit_group(key, e):
    g = e.get("site_group")
    if g is not None:
      groups.setdefault(key, set()).add(g)

  for e in episodes:
    la, lo, brg, d0 = e["site_lat"], e["site_lon"], e["approach_bearing"], e["date"]
    hit = _near(grid, la, lo, params.site_radius_m)
    if hit:
      tally["came_within_radius_at_all"] += 1
      hit_group("came_within_radius_at_all", e)
    dates = {pt_date(tk.t) for tk in hit} - {d0}
    if dates:
      tally["revisited_on_another_date"] += 1
      hit_group("revisited_on_another_date", e)
    # Same direction, because a row is keyed on (site, approach bearing): the opposite carriageway
    # is a different road and must not be counted as a revisit.
    same_dir = [tk for tk in hit if tk.bearing is not None
                and bearing_diff_deg(tk.bearing, brg) <= params.heading_tol_deg]
    dirs = {pt_date(tk.t) for tk in same_dir} - {d0}
    if not dirs:
      continue
    tally["revisited_same_direction"] += 1
    hit_group("revisited_same_direction", e)
    other_date_hist[len(dirs)] += 1
    if len(dirs) >= params.min_dates:
      # What D6 actually needs: leave-one-out removes the episode's own date, so a site with only
      # ONE other date can never reach min_dates however clean the pipeline gets.
      tally["revisited_on_enough_other_dates_for_D6"] += 1
      hit_group("revisited_on_enough_other_dates_for_D6", e)

    # SAME key as `CurveDB._nearest`: position AND approach bearing (Fable 2026-09-19). Omitting
    # the bearing counted a row on the opposite carriageway as a row the lookup would find, which
    # is why this reported 7 where the replay's matcher saw 4 -- two numbers for one question,
    # differing for a reason nobody had written down.
    has_row = any(o.date != d0
                  and haversine_m(o.site_lat, o.site_lon, la, lo) <= params.site_radius_m
                  and bearing_diff_deg(o.bearing_deg, brg) <= params.heading_tol_deg
                  for o in observations)
    if has_row:
      why["a row exists"] += 1
    elif not any(_tick_k(tk)[0] is not None for tk in same_dir):
      why["no curvature column on the revisit (pre-August corpus)"] += 1
    elif any(tk.dq_bits for tk in same_dir):
      worst = max((tk.dq_bits for tk in same_dir), key=lambda b: bin(b).count("1"))
      why[f"disqualified: {dq_names(worst)}"] += 1
    else:
      why["no map candidate / no approach bearing / never became a site"] += 1
  return tally, why, other_date_hist, groups


def report(tally, why, hist, n_eps, params, log=print, groups=None, n_groups=None):
  log("")
  log("=" * 100)
  log("RECURRENCE -- measured from RAW GPS TICKS, with the observation pipeline OUT of the loop")
  log("=" * 100)
  log(f"  matching radius {params.site_radius_m:.0f} m, heading tolerance " +
      f"{params.heading_tol_deg:.0f} deg, speed floor {params.min_speed_ms:.1f} m/s")
  if n_groups is None:
    log("  per-episode counts only: this episodes file predates the `site_group` tag, so ICBM " +
        "re-firing at one junction is counted more than once. Re-run ingest for both columns.")
  else:
    log(f"  two denominators: {n_eps} EPISODES at {n_groups} DISTINCT episode-sites " +
        f"({n_eps - n_groups} are the same junction firing again inside one drive)")
  for key, label in (
    ("came_within_radius_at_all", "episode sites the truck came within the radius of, ever"),
    ("revisited_on_another_date", "... on a DIFFERENT date"),
    ("revisited_same_direction", "... on a different date AND a compatible approach bearing"),
    ("revisited_on_enough_other_dates_for_D6",
     f"... on >= {params.min_dates} other dates (what D6 needs under leave-one-out)"),
  ):
    sites = ("" if groups is None or n_groups is None
             else f"   |  {len(groups.get(key, ())):>4} / {n_groups} distinct sites")
    log(f"  {label:<70} {tally[key]:>4} / {n_eps}{sites}")
  log(f"  distinct OTHER dates per revisited site: {dict(sorted(hist.items()))}")
  log("")
  log("  of the revisits, why each did or did not become a usable row:")
  for k, v in why.most_common():
    log(f"    {v:>4}  {k}")
  lost = sum(v for k, v in why.items() if k != "a row exists")
  got = why.get("a row exists", 0)
  log("")
  if got + lost == 0:
    log("  NO REVISITS AT ALL. That is a statement about the driving, and the only fix is more of " +
        "it -- but check the radius and heading tolerance above before believing it.")
  else:
    log(f"  {lost} of {got + lost} genuine revisits produced no row. If that number dominates, " +
        "the blocker is a rule we chose (section 6.1), not the driving pattern.")


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__,
                               formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("paths", nargs="+", help="the same ces_events corpora ingest.py was run over")
  ap.add_argument("--episodes", required=True)
  ap.add_argument("--observations", required=True)
  ap.add_argument("--site-radius-m", type=float)
  ap.add_argument("--heading-tol-deg", type=float)
  args = ap.parse_args(argv)

  params = PROVISIONAL_PARAMS
  if args.site_radius_m:
    params = CurveDBParams(**{**vars(params), "site_radius_m": args.site_radius_m})
  if args.heading_tol_deg:
    params = CurveDBParams(**{**vars(params), "heading_tol_deg": args.heading_tol_deg})
  if params.site_radius_m > 110.0:
    raise SystemExit(f"site_radius_m={params.site_radius_m} exceeds what the {CELL_DEG} deg grid's " +
                     "3x3 neighbourhood covers; widen CELL_DEG rather than under-counting silently")

  episodes = load_episodes(args.episodes)
  observations = load_observations(args.observations)
  ticks, _ = load_corpus(args.paths, log=lambda *a: None)
  print(f"curvedb recurrence -- {len(ticks)} ticks, {len(episodes)} episodes, " +
        f"{len(observations)} observations")
  if not episodes:
    raise SystemExit("no episodes -- nothing to measure recurrence FOR")
  tally, why, hist, groups = analyse(ticks, episodes, observations, params)
  tags = [e.get("site_group") for e in episodes]
  n_groups = None if any(t is None for t in tags) else len(set(tags))
  report(tally, why, hist, len(episodes), params, groups=groups, n_groups=n_groups)
  return 0


if __name__ == "__main__":
  sys.exit(main())
