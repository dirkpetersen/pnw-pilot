#!/usr/bin/env python3
"""curvedb ingest -- ces_events corpora into curvedb observations and ICBM episodes.

CURVEDB2PNW.md sections 6.1/6.2/6.3 (what a pass measures) plus the raw material section 7's replay
needs. NOT ON THE CONTROL PATH: this is an offline `tools/` script, it imports only `store.py` from
this package, and nothing the car runs imports it.

    PYTHONPATH=. python3 tools/curvedb/ingest.py drives/**/ces_events*.jsonl \\
        --out-observations obs.jsonl --out-episodes eps.jsonl

## The thing this file exists to not do

**A corpus that silently contributes nothing is the exact Rule 2 failure this project keeps
hitting.** The fields Phase 1 added -- `kPeak` `kPeakN` `kPose` `kPoseP` `achLatPose` `dq` `strTq`
`mapLat` `mapLon` `mapCandD` -- exist only from 2026-09-17. Every file before that has a strictly
smaller vocabulary, and several have no curvature at all. So ingest **detects what each file
actually carries, says so per file, and names the estimator it fell back to**; a file that produces
zero observations prints why, with counts, never a blank line.

Five traps, each of which produced or nearly produced a wrong number while this was written:

1. **Duplicate corpora.** `drives/` holds the same rotated generation under several names (a `.gz`
   beside its `.jsonl`, one `ces_tail.jsonl` filed under two incident folders, overlapping pulls).
   Left in, the same pass counts twice and manufactures the "2 passes" half of the D6 authority rule
   out of one drive. Ticks are deduplicated globally on `(car, round(t, 2))` and the collision count
   is printed per file.
2. **Dead-RTC timestamps.** The 3X's RTC battery is dead, so a cold boot stamps records with a stale
   clock until NTP lands; several corpora open in November 2025 or July 2028. The PT date is the
   leave-one-date-out key, so a misdated record leaks a pass into the wrong fold. Records further
   than `CLOCK_SKEW_MAX_S` from their file's median are dropped and counted.
3. **`slKActl` is DEAD on the Tesla** -- exactly 0.0 on essentially every moving tick, which reads
   as *perfectly straight road*, the most dangerous possible default for anything that cancels a
   slowdown (D1). An exact 0.0 in the curvature family is therefore treated as NULL here, on every
   corpus, whether or not the log that produced it had P1-B.
4. **The pose sign inversion.** `kPose`/`achLatPose` recorded before `36f914a17c` are negated.
   Detected per file against an independent witness and reported. The blast radius is stated
   honestly in `_PoseSign`: the store keys on curvature MAGNITUDE and on the GPS approach bearing,
   so a flipped sign corrupts the signed telemetry columns and not the rows.
5. **Reconstructed candidate positions.** Before 2026-09-17 nothing logged where the curve was
   (D3), only how far ahead it was. `_reconstruct_site` recovers the point from the truck's own
   later track, and every row it builds is tagged `site_src="track"`, so the replay reports the mix
   instead of it being inferable only by reading this file.
"""
from __future__ import annotations

import argparse
import bisect
import dataclasses
import datetime
import gzip
import json
import math
import os
import statistics
import sys
import zoneinfo
from collections import Counter
from dataclasses import dataclass

from openpilot.tools.curvedb.store import (
  PROVISIONAL_PARAMS,
  CurveDBParams,
  Observation,
  haversine_m,
  initial_bearing_deg,
)

PT = zoneinfo.ZoneInfo("America/Los_Angeles")   # the device runs UTC; the driver lives in Pacific

TICK_EVENTS = ("tick", "adopt", "steer")

# ---------------------------------------------------------------------------------------------
# PROVISIONAL ingest constants. None is defended by data yet; all are listed in the report.
# ---------------------------------------------------------------------------------------------
# Wider than any single drive, narrower than the multi-day spread a dead-RTC boot produces.
# Applied against the FILE's median timestamp, never against a wall clock.
CLOCK_SKEW_MAX_S = 36 * 3600.0
# A gap this long is a new drive (ignition cycle, or the seam between rotated generations).
# Longer than a traffic light, shorter than a charging stop.
DRIVE_GAP_S = 300.0
# dt between ticks is nominally 1 s; clamp so one missing second cannot integrate a phantom
# kilometre of odometer.
MAX_TICK_DT_S = 5.0
# How close the truck must actually have come to a candidate for the pass to count as having driven
# it. mapd's point can sit 56-125 m from the bend (section 6.3) and lane offset adds a few more.
# Generous on purpose: a pass that never reached the point is DROPPED with a count, never measured.
PASSAGE_MAX_M = 80.0
# How far past the naming tick to look for the passage. ICBM's own far-source reach is 500 m, so a
# candidate is never named more than that ahead; 1500 m is three times that and bounds the search.
PASSAGE_SCAN_M = 1500.0
# Tolerance on where the approach bearing is sampled. Section 6.3 makes the direction the APPROACH
# direction; sampling it at a fixed distance back is what makes ingest and a lookup agree.
APPROACH_TOL_M = 60.0
# Two fixes this far apart is a real direction; two fixes a metre apart are noise wearing a bearing.
BEARING_BASELINE_M = 50.0
# How far ahead of an episode's start the curve it is reacting to may be (MAP_SOURCE_HORIZON_M).
EPISODE_LOOKAHEAD_M = 500.0
# Ticks can be missing inside an episode; a gap longer than this ends it.
EPISODE_GAP_S = 3.0
# How many ticks from the decision to look in for ICBM's own candidate (its coordinates, then its
# distance). ICBM latches a point per tick; a handful covers a latch that lands a tick or two late.
EPISODE_CAND_SCAN_TICKS = 5
# Below this reduction it is not a slowdown, it is rounding on the SET button.
MIN_REDUCTION_MS = 1.0
# How far back to look for the pre-curve reference speed when `icbmC` is absent.
REF_LOOKBACK_S = 20.0
# Curvature below this is a straight road at every speed we drive (R > 10 km; reaching 2.5 m/s^2 at
# k = 1e-4 needs 158 m/s). Used ONLY to reject a degenerate row -- never to call a road straight.
K_MIN_USABLE = 1e-4
# The odometer and the GPS track must agree to within this, or the extents on that drive are
# measured against a distance the two sources disagree about.
ODO_GPS_RATIO_BAND = (0.8, 1.25)

ICBM_DECEL_SOURCES = ("map", "far", "vis", "gpsHold")

# Every drop/keep counter this module can emit. Seeded to 0 so the tally always PRINTS a
# reason that never fired, instead of omitting it -- "both counters are 0" must be read,
# not inferred from an absence. The f-string-built keys (site_*, ep_site_attrib_*,
# obs_up_*, obs_dropped_dq_*) are listed explicitly for the same reason.
TALLY_KEYS = (
  "down_dropped_no_kcmd", "down_no_lateral_accel_witness",
  "drive_dropped_odometer_disagrees_with_gps", "drives",
  "ep_dropped_no_reference_speed", "ep_dropped_no_site_ahead", "ep_dropped_reduction_too_small",
  "ep_kept", "ep_restore_only", "ep_site_attrib_logged_candidate",
  "ep_site_attrib_logged_candidate_far", "ep_site_attrib_map_dist",
  "ep_site_attrib_nearest_ahead", "obs_down", "obs_dropped_disqualified",
  "obs_k_below_usable_floor", "obs_up_clean", "obs_up_unknown", "pass_extent_has_no_moving_tick",
  "pass_never_reached_site", "pass_no_approach_bearing", "pass_no_curvature_anywhere_in_extent",
  "passages", "site_logged", "site_reconstruct_past_end_of_drive", "site_track",
)


# Section 3.5's disqualifier roll-up, in ces_pnw's OWN four names (`_DQ_NAMES`), so the 100 Hz
# logged `dqWhy` and the 1 Hz sampled flags of an older corpus are directly comparable instead of
# being two vocabularies that only look like one.
DQ_SAT, DQ_DRV, DQ_LC, DQ_BLNK = 1, 2, 4, 8
# `dq` said true but named no cause. Deliberately outside every relaxation mask: a disqualifier we
# cannot attribute must not be relaxable, because "we do not know why" is not "it does not count".
DQ_UNATTRIBUTED = 16
DQ_BIT = {"sat": DQ_SAT, "drv": DQ_DRV, "lc": DQ_LC, "blnk": DQ_BLNK}
DQ_NAME = {DQ_SAT: "sat", DQ_DRV: "drv", DQ_LC: "lc", DQ_BLNK: "blnk",
           DQ_UNATTRIBUTED: "unattributed"}
DQ_ALL = DQ_SAT | DQ_DRV | DQ_LC | DQ_BLNK | DQ_UNATTRIBUTED


def dq_names(bits: int) -> str:
  return ",".join(n for b, n in sorted(DQ_NAME.items()) if bits & b)


def dq_mask_from_names(names: str) -> int:
  """Parse a comma-separated relaxation mask. An unrecognised name RAISES rather than being
  ignored: silently dropping a flag the caller asked for would relax the rule further than asked,
  in the unsafe direction."""
  mask = DQ_UNATTRIBUTED
  for n in [x.strip() for x in names.split(",") if x.strip()]:
    if n not in DQ_BIT:
      raise ValueError(f"unknown disqualifier {n!r}; known: {sorted(DQ_BIT)}")
    mask |= DQ_BIT[n]
  return mask


# ---------------------------------------------------------------------------------------------
# normalised tick
# ---------------------------------------------------------------------------------------------

@dataclass(slots=True)
class Tick:
  t: float
  car: str
  lat: float
  lon: float
  bearing: float | None
  v_ego: float
  k_peak: float | None        # kPeak: the 100 Hz per-second max of max(|achieved|, |commanded|)
  k_actl: float | None        # slKActl: CAN/pose achieved curvature (DEAD on the Tesla)
  k_cmd: float | None         # slKCmd: commanded curvature
  ach_lat: float | None
  dq_bits: int                # section 3.5 causes: logged `dqWhy` where it exists, else the 1 Hz OR
  dq_known: bool              # was ANY disqualifier evidence present in this record at all
  dq_rollup: bool             # was that evidence section 3.5's 100 Hz OR, or a 1 Hz sample
  str_prs: bool
  ev: str
  posted: float | None
  hwy: str | None
  map_lat: float | None
  map_lon: float | None
  map_dist: float | None
  icbm_t: float | None
  icbm_src: str | None
  icbm_c: float | None
  v_set: float | None
  src: str
  s: float = 0.0              # cumulative distance along this drive, metres


def _num(v):
  """A finite number, or None. Booleans are not numbers here -- `True` is not 1.0 m/s."""
  if isinstance(v, bool) or not isinstance(v, int | float):
    return None
  f = float(v)
  return f if math.isfinite(f) else None


def _k(v):
  """A curvature MAGNITUDE, or None.

  **An exact 0.0 is a dead sensor, not a straight road** (P1-B / D1). `slKActl` reads exactly 0.0 on
  7,218 of 7,221 moving Tesla ticks; admitting those as measurements is what made v1's proof of
  concept a Tesla with a dead sensor on the opposite carriageway. Applied here rather than trusting
  the log, because every corpus before 2026-09-17 predates P1-B and logs the dead reading as 0.0."""
  f = _num(v)
  if f is None or f == 0.0:
    return None
  return abs(f)


def _signed(v):
  f = _num(v)
  return None if f is None or f == 0.0 else f


def normalise(r: dict, src: str) -> Tick | None:
  """One ces_events record -> a Tick, or None if it cannot anchor anything.

  Requires a timestamp, a fix and a speed: a record without those cannot contribute to a
  position-keyed database, and is counted as `no_fix` rather than silently skipped."""
  t = _num(r.get("t"))
  lat, lon = _num(r.get("lat")), _num(r.get("lon"))
  v = _num(r.get("vEgo"))
  if t is None or lat is None or lon is None or v is None:
    return None
  if not -90.0 <= lat <= 90.0 or not -180.0 <= lon <= 180.0:
    return None
  bearing = _num(r.get("bearing"))
  if bearing is None:
    bearing = _num(r.get("heading"))
  if bearing is not None:
    bearing %= 360.0

  # Section 3.5. The logged roll-up is the 100 Hz OR and is strictly better than the 1 Hz sample,
  # so it wins where it exists -- but only its NAMED causes are carried, because an unattributed
  # disqualifier must not be relaxable later.
  bits, known, rollup = 0, False, False
  logged = r.get("dq")
  if isinstance(logged, bool):
    known = rollup = True
    why = r.get("dqWhy")
    if why:
      for n in str(why).split(","):
        bits |= DQ_BIT.get(n.strip(), DQ_UNATTRIBUTED)
    elif logged:
      bits |= DQ_UNATTRIBUTED
  else:
    for name, bit in (("strPrs", DQ_DRV), ("blnk", DQ_BLNK), ("slAngSat", DQ_SAT),
                      ("slSat", DQ_SAT), ("slCurvLim", DQ_SAT)):
      if name in r:
        known = True
        if r[name]:
          bits |= bit
    if "lcGate" in r:
      known = True
      if r["lcGate"] == "lanechange":
        bits |= DQ_LC

  return Tick(
    t=t, car=str(r.get("car") or "") or "UNKNOWN", lat=lat, lon=lon, bearing=bearing, v_ego=v,
    k_peak=_k(r.get("kPeak")), k_actl=_k(r.get("slKActl")), k_cmd=_k(r.get("slKCmd")),
    ach_lat=_signed(r.get("achLat")), dq_bits=bits, dq_known=known, dq_rollup=rollup,
    str_prs=bool(r.get("strPrs")), ev=str(r.get("ev") or ""),
    posted=_num(r.get("spdLim")), hwy=(str(r["hwyClass"]) if r.get("hwyClass") else None),
    map_lat=_num(r.get("mapLat")), map_lon=_num(r.get("mapLon")), map_dist=_num(r.get("mapDist")),
    icbm_t=_num(r.get("icbmT")), icbm_src=(str(r["icbmSrc"]) if r.get("icbmSrc") else None),
    icbm_c=_num(r.get("icbmC")), v_set=_num(r.get("vSet")), src=src,
  )


# ---------------------------------------------------------------------------------------------
# pose sign (trap 4)
# ---------------------------------------------------------------------------------------------

class _PoseSign:
  """Was `kPose` recorded before the 2026-09-17 sign fix `36f914a17c`?

  Two independent witnesses, in order of strength:

  * `slKActl` -- the CAN/Ford source, correct and signed positive-left. Only usable where it is
    ALIVE, which excludes every Tesla record (D1).
  * `strAng` -- the steering wheel. The 2026-09-17 drive report measured this third witness
    directly: on 213 records with |strAng| > 8 degrees, `slKActl` agreed 95.3 % and the unfixed
    `kPose` agreed 0.5 %. It is the only witness a Tesla corpus has.

  **Blast radius, stated rather than implied.** `Observation.k` is a magnitude and a row's direction
  is the GPS approach bearing, so an inverted `kPose` does not flip a row -- it corrupts the signed
  telemetry columns, which is exactly why the defect survived a whole drive on the Lightning. This
  check reports the state of a corpus and justifies using |kPose| unconditionally; it is NOT the
  thing standing between an undetected flip and a poisoned database."""

  MIN_WITNESSES = 20

  def __init__(self):
    self.ratios: list[float] = []
    self.angle_agree: list[bool] = []
    self.kpose_records = 0

  def observe(self, r: dict):
    kp = _signed(r.get("kPose"))
    if "kPose" in r:
      self.kpose_records += 1
    if kp is None:
      return
    ka = _signed(r.get("slKActl"))
    if ka is not None and abs(ka) > 1e-4:
      self.ratios.append(kp / ka)
    sa = _num(r.get("strAng"))
    if sa is not None and abs(sa) > 8.0:
      self.angle_agree.append((kp > 0) == (sa > 0))

  def verdict(self) -> tuple[str, str]:
    if self.kpose_records == 0:
      return "absent", "kPose not in this corpus (pre-2026-09-17)"
    if len(self.ratios) >= self.MIN_WITNESSES:
      med = statistics.median(self.ratios)
      n = len(self.ratios)
      if med <= -0.5:
        return "INVERTED", f"median kPose/slKActl = {med:+.3f} over n={n} (pre-36f914a17c)"
      if med >= 0.5:
        return "normal", f"median kPose/slKActl = {med:+.3f} over n={n}"
      return "ambiguous", f"median kPose/slKActl = {med:+.3f} over n={n} -- neither sign"
    if len(self.angle_agree) >= self.MIN_WITNESSES:
      agree = sum(self.angle_agree) / len(self.angle_agree)
      n = len(self.angle_agree)
      if agree <= 0.2:
        return "INVERTED", f"sign(kPose)==sign(strAng) on {100 * agree:.1f}% of n={n}"
      if agree >= 0.8:
        return "normal", f"sign(kPose)==sign(strAng) on {100 * agree:.1f}% of n={n}"
      return "ambiguous", f"sign(kPose)==sign(strAng) on {100 * agree:.1f}% of n={n} -- neither"
    return ("no witness",
            f"kPose on {self.kpose_records} records but only {len(self.ratios)} slKActl / " +
            f"{len(self.angle_agree)} strAng witnesses (need {self.MIN_WITNESSES})")


# ---------------------------------------------------------------------------------------------
# loading
# ---------------------------------------------------------------------------------------------

AUDIT_FIELDS = ("kPeak", "kPeakN", "kPose", "kPoseP", "achLatPose", "dq", "strTq", "mapLat",
                "mapCandD", "achLat", "slKActl", "slKCmd", "icbmT", "icbmSrc", "icbmC", "mapV",
                "mapDist", "spdLim", "hwyClass", "bearing", "vEgo", "strAng")


@dataclass
class FileReport:
  path: str
  records: int = 0
  unparsable: int = 0
  ticks: int = 0
  no_fix: int = 0
  duplicates: int = 0
  clock_dropped: int = 0
  kept: int = 0
  cars: Counter = dataclasses.field(default_factory=Counter)
  fields_present: Counter = dataclasses.field(default_factory=Counter)
  fields_alive: Counter = dataclasses.field(default_factory=Counter)
  pose_sign: str = "not checked"
  span_pt: str = "-"

  def line(self) -> str:
    return (f"{os.path.relpath(self.path):<70} recs={self.records:>7} kept={self.kept:>7} " +
            f"dup={self.duplicates:>6} clock={self.clock_dropped:>5} nofix={self.no_fix:>5} " +
            f"bad={self.unparsable:>4} {','.join(sorted(self.cars)) or '-'}")


def _read_json_lines(path):
  op = gzip.open if str(path).endswith(".gz") else open
  with op(path, "rt", errors="replace") as f:
    for lineno, line in enumerate(f, 1):
      line = line.strip()
      if not line:
        continue
      try:
        r = json.loads(line)
      except Exception:
        yield lineno, None
        continue
      yield lineno, (r if isinstance(r, dict) else None)


def load_corpus(paths, log=print) -> tuple[list[Tick], list[FileReport]]:
  """Every usable tick from every file, deduplicated, with a per-file report.

  Files are processed in SORTED order so the dedup keeps a deterministic first-seen copy; which
  file "owns" a shared record is then a property of the path list, not of filesystem ordering.

  One streaming pass per file: the raw records are never all held at once (the largest corpus here
  is 46 MB of ~2.5 kB records), only the normalised Ticks are."""
  seen: set[tuple[str, int]] = set()
  ticks: list[Tick] = []
  reports: list[FileReport] = []
  for path in sorted(paths):
    rep = FileReport(path=path)
    pose = _PoseSign()
    pending: list[Tick] = []
    for lineno, r in _read_json_lines(path):
      rep.records += 1
      if r is None:
        rep.unparsable += 1
        continue
      if r.get("ev") not in TICK_EVENTS:
        continue
      rep.ticks += 1
      for f in AUDIT_FIELDS:
        if f in r:
          rep.fields_present[f] += 1
          v = r[f]
          if v is not None and v != 0.0 and v != "" and v is not False:
            rep.fields_alive[f] += 1
      pose.observe(r)
      tk = normalise(r, f"{os.path.relpath(path)}:{lineno}")
      if tk is None:
        rep.no_fix += 1
        continue
      pending.append(tk)

    verdict, detail = pose.verdict()
    rep.pose_sign = f"{verdict}: {detail}"
    if not pending:
      reports.append(rep)
      log("  " + rep.line() + "   <-- NOTHING USABLE")
      continue

    median_t = statistics.median([tk.t for tk in pending])
    for tk in pending:
      if abs(tk.t - median_t) > CLOCK_SKEW_MAX_S:
        rep.clock_dropped += 1
        continue
      # `ev` is part of the key (Fable 2026-09-17). A `tick` and an `adopt` can share a
      # timestamp -- 3 of 51 adopt records in a 6,000-line sample of the 2026-09-17 corpus do --
      # and the adopt record is the one carrying icbmT/icbmSrc/mapLat at the decision instant.
      # Keyed on (car, t) alone, whichever arrived second was silently counted as a duplicate of
      # the other, which is a real record thrown away wearing a deduplication's costume.
      key = (tk.car, tk.ev, int(round(tk.t * 100.0)))
      if key in seen:
        rep.duplicates += 1
        continue
      seen.add(key)
      rep.cars[tk.car] += 1
      rep.kept += 1
      ticks.append(tk)

    fmt = "%Y-%m-%d %H:%M %Z"
    lo = datetime.datetime.fromtimestamp(min(tk.t for tk in pending), PT)
    hi = datetime.datetime.fromtimestamp(max(tk.t for tk in pending), PT)
    rep.span_pt = f"{lo:{fmt}} -> {hi:{fmt}}"
    reports.append(rep)
    log("  " + rep.line())
  ticks.sort(key=lambda x: (x.car, x.t))
  return ticks, reports


# ---------------------------------------------------------------------------------------------
# drives
# ---------------------------------------------------------------------------------------------

@dataclass(slots=True)
class Drive:
  car: str
  ticks: list[Tick]
  s: list[float]
  date: str
  drive_id: str
  gps_vs_odo: float | None      # median ratio of GPS step to v*dt step -- a units cross-check

  def index_at(self, s_target: float) -> int | None:
    """Index of the tick nearest `s_target` along this drive, or None when out of range."""
    if not self.s:
      return None
    i = bisect.bisect_left(self.s, s_target)
    if i <= 0:
      return 0
    if i >= len(self.s):
      return len(self.s) - 1
    return i if (self.s[i] - s_target) < (s_target - self.s[i - 1]) else i - 1


def pt_date(t: float) -> str:
  """The PT calendar date of an epoch. The leave-one-date-out key, and the `drives/` folder name:
  a late-evening PT drive is already the next day in UTC, and filing it under the UTC date would
  put the same road in two different folds."""
  return f"{datetime.datetime.fromtimestamp(t, PT):%Y-%m-%d}"


def split_drives(ticks: list[Tick]) -> list[Drive]:
  """Contiguous per-car runs, each with the cumulative odometer every site measurement indexes by.

  `s` integrates `v_ego * dt` rather than summing GPS steps: at 1 Hz the GPS step is ~25 m of signal
  on a few metres of fix noise. But the GPS track is also the only INDEPENDENT witness that vEgo is
  in m/s at all, so both are computed and their median ratio is reported. A corpus where that is not
  ~1.0 has a units problem -- the `units2pnw` failure mode, non-zero and wrong."""
  drives: list[Drive] = []
  run: list[Tick] = []

  def flush():
    if len(run) < 2:
      run.clear()
      return
    s, gps, odo = [0.0], [], []
    for a, b in zip(run, run[1:], strict=False):
      dt = min(max(b.t - a.t, 0.0), MAX_TICK_DT_S)
      step = 0.5 * (a.v_ego + b.v_ego) * dt
      s.append(s[-1] + step)
      g = haversine_m(a.lat, a.lon, b.lat, b.lon)
      if step > 5.0 and g > 0.0:
        gps.append(g)
        odo.append(step)
    ratio = statistics.median([g / o for g, o in zip(gps, odo, strict=True)]) if gps else None
    for tk, sv in zip(run, s, strict=True):
      tk.s = sv
    # The id is (car, first tick's epoch) rendered stably: unique per ignition cycle, stable
    # across re-ingests, and readable in a row's provenance.
    drives.append(Drive(car=run[0].car, ticks=list(run), s=s, gps_vs_odo=ratio,
                        date=pt_date(statistics.median([x.t for x in run])),
                        drive_id=f"{run[0].car}:{run[0].t:.2f}"))
    run.clear()

  for tk in ticks:
    if run and (tk.car != run[-1].car or not 0.0 <= tk.t - run[-1].t <= DRIVE_GAP_S):
      flush()
    run.append(tk)
  flush()
  return drives


# ---------------------------------------------------------------------------------------------
# sites: where the CURVE is (D3), not where the truck is
# ---------------------------------------------------------------------------------------------

@dataclass(slots=True)
class Site:
  lat: float
  lon: float
  src: str            # "logged" (mapLat/mapLon) | "track" (reconstructed)
  first_i: int        # the tick that named it


def _reconstruct_site(drive: Drive, i: int, dist_m: float):
  """Where is a candidate `dist_m` ahead? **At the truck's own position `dist_m` later.**

  The truck drives the road, so its future track IS the road, to within lane offset and fix noise.
  Projecting `dist_m` along the instantaneous bearing instead puts the point off the road exactly
  where the road bends -- i.e. on the entire population this database is about. Returns None when
  the drive ended before the candidate was reached, which is a real and counted skip, not a zero."""
  target = drive.s[i] + dist_m
  if target > drive.s[-1]:
    return None
  j = bisect.bisect_left(drive.s, target)
  if j <= i:
    return None
  a, b = drive.ticks[j - 1], drive.ticks[j]
  span = drive.s[j] - drive.s[j - 1]
  f = 0.0 if span <= 0.0 else (target - drive.s[j - 1]) / span
  return (a.lat + f * (b.lat - a.lat), a.lon + f * (b.lon - a.lon))


def find_sites(drive: Drive, params: CurveDBParams, stats: Counter) -> list[Site]:
  """Every distinct map candidate this drive named, deduplicated at the matcher's own radius.

  Deduplicating with `site_radius_m` -- the radius the LOOKUP uses -- is deliberate: two points the
  matcher would treat as one row must contribute ONE pass, or a single drive manufactures the
  multiple passes D6 requires. Only moving ticks may name a site: below `min_speed_ms` the truck is
  parked or crawling and the curvature that would be measured there is 1/v noise (the 2026-09-17
  corpus's tightest measured radius was 8 m -- a parking-lot turn, not a road curve)."""
  sites: list[Site] = []
  for i, tk in enumerate(drive.ticks):
    if tk.v_ego < params.min_speed_ms:
      continue
    if tk.map_lat is not None and tk.map_lon is not None:
      pt, src = (tk.map_lat, tk.map_lon), "logged"
    elif tk.map_dist is not None and tk.map_dist > 0.0:
      pt = _reconstruct_site(drive, i, tk.map_dist)
      src = "track"
      if pt is None:
        stats["site_reconstruct_past_end_of_drive"] += 1
        continue
    else:
      continue
    for s in sites:
      if haversine_m(s.lat, s.lon, pt[0], pt[1]) <= params.site_radius_m:
        break
    else:
      sites.append(Site(lat=pt[0], lon=pt[1], src=src, first_i=i))
      stats[f"site_{src}"] += 1
  return sites


# ---------------------------------------------------------------------------------------------
# measuring one pass over one site
# ---------------------------------------------------------------------------------------------

@dataclass(slots=True)
class Passage:
  """What one drive measured at one site: the raw material for an Observation and, when an ICBM
  episode points at this site, for the replay's ground truth."""
  site: Site
  i_passage: int
  s_passage: float
  approach_bearing: float
  bearing_src: str
  k: float
  k_estimator: str
  k_n: int
  k_v_ego: float                   # the speed at which that peak curvature was measured
  a_lat_max: float | None          # max |achLat| over the extent, hands-off ticks only
  a_lat_src: str
  dq_state: str                    # "clean" | "dirty" | "unknown"
  dq_src: str                      # "rollup100" | "sampled1hz" | "none" -- how strong that state is
  dq_why: str                      # ALL causes seen in the extent, even ones the mask relaxed
  posted: float | None
  hwy: str | None
  n_ticks: int
  t: float
  extent: tuple[int, int]
  passage_err_m: float


def _tick_k(tk: Tick) -> tuple[float | None, str]:
  """This tick's best curvature reading, and which estimator produced it.

  Section 3.3: `max(commanded, achieved)`, never achieved alone -- achieved is bounded by steering
  authority, and where the truck cannot follow it UNDER-reads the road, precisely on the curves that
  matter (D2). `kPeak` already is that max, taken at 100 Hz. Without it the 1 Hz sample is the same
  max over whichever of the two columns exist, and it is a WEAKER estimator in the unsafe direction:
  an under-read k derives too high a speed."""
  if tk.k_peak is not None:
    return tk.k_peak, "kPeak100"
  if tk.k_actl is not None and tk.k_cmd is not None:
    return max(tk.k_actl, tk.k_cmd), "sample1hz_cmd_actl"
  if tk.k_actl is not None:
    return tk.k_actl, "sample1hz_actl"
  if tk.k_cmd is not None:
    return tk.k_cmd, "sample1hz_cmd"
  return None, "none"


def _weakest_estimator(ests: Counter) -> str:
  """Name a window by its WEAKEST estimator, not the one that happened to hold the peak.

  A row whose peak came from `kPeak` but whose window was mostly 1 Hz samples is a 1 Hz row: the
  peak could have been anywhere, and naming it by the strongest reading present would overstate the
  row's provenance in exactly the direction that matters."""
  if not ests:
    return "none"
  for name in ("sample1hz_cmd", "sample1hz_actl", "sample1hz_cmd_actl"):
    if ests.get(name):
      return name + ("_mixed" if ests.get("kPeak100") else "")
  return "kPeak100"


def measure_passage(drive: Drive, site: Site, params: CurveDBParams, stats: Counter,
                    dq_mask: int = DQ_ALL) -> Passage | None:
  """Section 6.3's extent, measured on one pass. None -- with a counted reason -- when the pass
  cannot support a measurement at all.

  The passage search is bounded forward from the tick that NAMED the candidate: the candidate is by
  construction ahead of its naming tick, and bounding the scan is what keeps this from being
  quadratic in the length of a four-hour drive."""
  lo_i = site.first_i
  s0 = drive.s[lo_i]
  hi_i = bisect.bisect_right(drive.s, s0 + PASSAGE_SCAN_M)
  best_i, best_d = None, float("inf")
  for i in range(lo_i, max(hi_i, lo_i + 1)):
    tk = drive.ticks[i]
    d = haversine_m(tk.lat, tk.lon, site.lat, site.lon)
    if d < best_d:
      best_i, best_d = i, d
  if best_i is None or best_d > PASSAGE_MAX_M:
    stats["pass_never_reached_site"] += 1
    return None
  s_p = drive.s[best_i]
  lo = bisect.bisect_left(drive.s, s_p - params.extent_back_m)
  hi = bisect.bisect_right(drive.s, s_p + params.extent_fwd_m)
  extent = [tk for tk in drive.ticks[lo:hi] if tk.v_ego >= params.min_speed_ms]
  if not extent:
    stats["pass_extent_has_no_moving_tick"] += 1
    return None

  # -- approach bearing, sampled where the CAR will later ask (section 6.3) --------------------
  bearing, bsrc = None, "none"
  s_ref = s_p - params.approach_bearing_ref_m
  i_ref = drive.index_at(s_ref)
  if i_ref is not None and abs(drive.s[i_ref] - s_ref) <= APPROACH_TOL_M:
    ref = drive.ticks[i_ref]
    if ref.bearing is not None:
      bearing, bsrc = ref.bearing, "logged"
    else:
      j = drive.index_at(drive.s[i_ref] + BEARING_BASELINE_M)
      if j is not None and j > i_ref:
        bearing = initial_bearing_deg(ref.lat, ref.lon, drive.ticks[j].lat, drive.ticks[j].lon)
        bsrc = "track"
  if bearing is None:
    # Not a silent skip: a pass whose approach was never recorded (the drive started inside the
    # extent, or the fix was lost) cannot be keyed by direction, and direction is half the key.
    stats["pass_no_approach_bearing"] += 1
    return None

  # -- curvature over the extent (section 6.1) ------------------------------------------------
  k_best, k_n, k_v = None, 0, 0.0
  ests: Counter = Counter()
  for tk in extent:
    k, est = _tick_k(tk)
    if k is None:
      continue
    k_n += 1
    ests[est] += 1
    if k_best is None or k > k_best:
      # The speed the peak was measured AT is carried with it. A k of 0.14 (R = 7 m) measured at
      # 6 m/s is an intersection turn, not a through-road curve, and a replay that multiplies it by
      # a 29 m/s counterfactual speed invents a 118 m/s^2 "curve" nothing ever drove.
      k_best, k_v = k, tk.v_ego
  if k_best is None:
    stats["pass_no_curvature_anywhere_in_extent"] += 1
    return None

  # -- disqualifiers (sections 3.5 / 6.1) ------------------------------------------------------
  bits, known, rollup = 0, False, False
  for tk in extent:
    bits |= tk.dq_bits
    known = known or tk.dq_known
    rollup = rollup or tk.dq_rollup
  if bits & dq_mask:
    dq_state = "dirty"
  elif known:
    dq_state = "clean"
  else:
    # Rule 2: no flags in this corpus means the disqualifier is UNKNOWN. "No evidence of a lane
    # change" is not "no lane change", and reading it as clean is how an override-contaminated pass
    # becomes a row claiming the road is tighter -- or straighter -- than it is.
    dq_state = "unknown"

  # -- lateral acceleration actually experienced, hands-off -------------------------------------
  a_vals, used_achlat, used_kv2 = [], False, False
  for tk in extent:
    if tk.str_prs:
      continue
    if tk.ach_lat is not None:
      a_vals.append(abs(tk.ach_lat))
      used_achlat = True
    else:
      k, _ = _tick_k(tk)
      if k is not None:
        a_vals.append(k * tk.v_ego * tk.v_ego)
        used_kv2 = True
  a_src = ("mixed" if used_achlat and used_kv2 else
           "achLat" if used_achlat else "k*v^2" if used_kv2 else "none")

  posted = [tk.posted for tk in extent if tk.posted]
  hwys = Counter(tk.hwy for tk in extent if tk.hwy)
  return Passage(
    site=site, i_passage=best_i, s_passage=s_p, approach_bearing=bearing, bearing_src=bsrc,
    k=k_best, k_estimator=_weakest_estimator(ests), k_n=k_n, k_v_ego=k_v,
    a_lat_max=max(a_vals) if a_vals else None, a_lat_src=a_src,
    dq_state=dq_state, dq_src=("rollup100" if rollup else "sampled1hz" if known else "none"),
    dq_why=dq_names(bits),
    posted=statistics.median(posted) if posted else None,
    hwy=hwys.most_common(1)[0][0] if hwys else None, n_ticks=len(extent),
    t=drive.ticks[best_i].t, extent=(lo, hi), passage_err_m=best_d,
  )


def observations_for_drive(drive: Drive, passages: list[Passage], params: CurveDBParams,
                           stats: Counter) -> list[Observation]:
  """UP (6.1) and DOWN (6.2) observations from one drive.

  **No lead-car gate on UP (D7):** curvature does not depend on a lead car, and v1 wrongly discarded
  the 2026-09-08 pass for that reason. The lead confound belongs to down-by-braking only -- which
  this corpus cannot measure at all, since `brakePressed` fired 0 of 15,884 carState messages above
  27 mph on this truck. DOWN here is therefore steering-override only, exactly as section 6.2
  specifies."""
  out: list[Observation] = []
  for p in passages:
    if p.k < K_MIN_USABLE:
      stats["obs_k_below_usable_floor"] += 1
      continue
    if p.dq_state == "dirty":
      stats["obs_dropped_disqualified"] += 1
      # WHICH cause, not just how many. Section 6.1 rejects a pass for any of four reasons and they
      # are not equally load-bearing: `sat` is steering saturation, which section 3.3's
      # max(commanded, achieved) already handles, while `blnk` is a signalled exit that says
      # nothing about the road's curvature. Without this breakdown the rule's cost is a single
      # number with no way to argue about it.
      for name in (p.dq_why.split(",") if p.dq_why else ["none"]):
        stats[f"obs_dropped_dq_{name}"] += 1
      continue
    tk = drive.ticks[p.i_passage]
    out.append(Observation(
      date=pt_date(tk.t), t=tk.t, car=drive.car, drive_id=drive.drive_id,
      site_lat=p.site.lat, site_lon=p.site.lon,
      bearing_deg=p.approach_bearing, k=p.k, kind="up", estimator=p.k_estimator,
      site_src=p.site.src, source=tk.src, posted_ms=p.posted, highway_class=p.hwy,
      n_ticks=p.n_ticks, dq_state=p.dq_state, dq_src=p.dq_src,
    ))
    stats[f"obs_up_{p.dq_state}"] += 1

    # -- DOWN: a steering override taken while the road was actually loading the truck ----------
    lo, hi = p.extent
    for t2 in drive.ticks[lo:hi]:
      if not t2.str_prs or t2.v_ego < params.min_speed_ms:
        continue
      if t2.ach_lat is not None:
        a = abs(t2.ach_lat)
      else:
        k2, _ = _tick_k(t2)
        a = None if k2 is None else k2 * t2.v_ego * t2.v_ego
      if a is None:
        stats["down_no_lateral_accel_witness"] += 1
        continue
      if a < params.down_trigger_a_lat_ms2:
        continue
      if t2.k_cmd is None or t2.k_cmd < K_MIN_USABLE:
        # Section 6.2's magnitude is sqrt(A_LAT/|slKCmd|). With no commanded curvature there is no
        # magnitude to compute, so the intervention is COUNTED and dropped, never guessed at.
        stats["down_dropped_no_kcmd"] += 1
        continue
      out.append(Observation(
        date=pt_date(t2.t), t=t2.t, car=drive.car, drive_id=drive.drive_id,
        site_lat=p.site.lat, site_lon=p.site.lon,
        bearing_deg=p.approach_bearing, k=t2.k_cmd, kind="down", estimator="slKCmd_at_override",
        site_src=p.site.src, source=t2.src, posted_ms=p.posted, highway_class=p.hwy,
        n_ticks=1, dq_state="clean", dq_src=p.dq_src,
      ))
      stats["obs_down"] += 1
      break          # one DOWN per pass: the driver's "too fast" is one judgement, not N ticks
  return out


# ---------------------------------------------------------------------------------------------
# ICBM episodes -- the thing the replay adjudicates
# ---------------------------------------------------------------------------------------------

def find_episodes(drive: Drive, passages: list[Passage], params: CurveDBParams,
                  stats: Counter) -> list[dict]:
  """Every ICBM slowdown this drive performed, with ground truth for the road it slowed for.

  An episode is a contiguous run of ticks carrying a published `icbmT`. The RESTORE tail belongs to
  the same run (it is how the truck gets its speed back) but is excluded from the depth, because a
  restore is by construction an increase."""
  eps: list[dict] = []
  i, n = 0, len(drive.ticks)
  while i < n:
    if drive.ticks[i].icbm_t is None:
      i += 1
      continue
    j = i
    while (j + 1 < n and drive.ticks[j + 1].icbm_t is not None
           and 0.0 <= drive.ticks[j + 1].t - drive.ticks[j].t <= EPISODE_GAP_S):
      j += 1
    lo_run, hi_run = i, j + 1
    i = j + 1

    i_start = next((x for x in range(lo_run, hi_run)
                    if drive.ticks[x].icbm_src in ICBM_DECEL_SOURCES), None)
    if i_start is None:
      stats["ep_restore_only"] += 1
      continue
    start = drive.ticks[i_start]
    run = drive.ticks[lo_run:hi_run]
    target = min(tk.icbm_t for tk in run if tk.icbm_src in ICBM_DECEL_SOURCES)

    # Reference: the speed the truck would have held with no curve slowdown. `icbmC` IS that -- the
    # episode's own pre-curve latch. Falling back to vSet DURING the episode would be circular:
    # ICBM taps the SET button down, so vSet mid-episode IS the slowdown, not the reference.
    ceils = [tk.icbm_c for tk in run if tk.icbm_c]
    if ceils:
      ref, ref_src = max(ceils), "icbmC"
    else:
      back = [tk.v_set for tk in drive.ticks[:i_start + 1]
              if tk.v_set and 0.0 <= start.t - tk.t <= REF_LOOKBACK_S]
      ref, ref_src = (max(back), "vSet_prewindow") if back else (None, "none")
    if ref is None:
      stats["ep_dropped_no_reference_speed"] += 1
      continue
    if ref - target < MIN_REDUCTION_MS:
      stats["ep_dropped_reduction_too_small"] += 1
      continue

    # Which site was it reacting to? "Ahead" is always along the drive's odometer, so a candidate
    # already driven past (the behindgate2pnw failure) can never be attributed to this episode.
    #
    # ⚠️ NEAREST-AHEAD IS THE LAST RESORT, NOT THE RULE. mapd targets are dense -- the first
    # measured run put the median attributed site 34 m ahead of a decision ICBM takes 300-500 m out,
    # i.e. it was naming a node the truck was about to pass rather than the curve being slowed for.
    # So: use the candidate ICBM actually logged where it exists, then its logged DISTANCE, and only
    # then fall back -- recording which, because the three are not equally trustworthy.
    s0 = drive.s[i_start]
    ahead = [p for p in passages if 0.0 <= p.s_passage - s0 <= EPISODE_LOOKAHEAD_M]
    if not ahead:
      stats["ep_dropped_no_site_ahead"] += 1
      continue
    head = drive.ticks[i_start:min(i_start + EPISODE_CAND_SCAN_TICKS, hi_run)]
    cand_pt = next((( tk.map_lat, tk.map_lon) for tk in head
                    if tk.map_lat is not None and tk.map_lon is not None), None)
    cand_d = next((tk.map_dist for tk in head if tk.map_dist), None)
    if cand_pt is not None:
      p = min(ahead, key=lambda q: haversine_m(q.site.lat, q.site.lon, cand_pt[0], cand_pt[1]))
      attrib = ("logged_candidate"
                if haversine_m(p.site.lat, p.site.lon, cand_pt[0], cand_pt[1]) <= params.site_radius_m
                else "logged_candidate_far")
    elif cand_d:
      p = min(ahead, key=lambda q: abs((q.s_passage - s0) - cand_d))
      attrib = "map_dist"
    else:
      p = min(ahead, key=lambda q: q.s_passage - s0)
      attrib = "nearest_ahead"
    stats[f"ep_site_attrib_{attrib}"] += 1

    # A second, independent read of "was there a curve": the max curvature anywhere in the lookahead
    # window, not only inside the chosen site's extent. Where the two disagree the site attribution
    # is suspect, and the replay can see that instead of trusting one number.
    hi_ahead = bisect.bisect_right(drive.s, s0 + EPISODE_LOOKAHEAD_M)
    # SAME speed floor as the passage extent (Fable 2026-09-17). Without it, 36 of 168 verdicts
    # were taken on a tick below 5 m/s -- a parking-lot or junction turn, whose k multiplied by a
    # highway counterfactual invents a curve nothing ever drove. That is noise in the FAIL
    # direction: it manufactures false cancels and the natural reaction to those is to loosen the
    # rule. A speed floor is the fix; a warning label on the output was not.
    k_ahead, k_ahead_v = None, 0.0
    for tk in drive.ticks[i_start:hi_ahead]:
      if tk.v_ego < params.min_speed_ms:
        continue
      k, _ = _tick_k(tk)
      if k is not None and (k_ahead is None or k > k_ahead):
        k_ahead, k_ahead_v = k, tk.v_ego

    eps.append({
      "date": pt_date(start.t), "t": start.t, "t_end": run[-1].t, "car": drive.car,
      "drive_id": drive.drive_id,
      "start_lat": start.lat, "start_lon": start.lon, "start_bearing": start.bearing,
      "v_ego_start": start.v_ego,
      "ref_ms": ref, "ref_src": ref_src, "icbm_target_ms": target, "reduction_ms": ref - target,
      "icbm_src": start.icbm_src,
      "site_lat": p.site.lat, "site_lon": p.site.lon, "site_src": p.site.src,
      "site_attrib": attrib,
      "site_dist_m": p.s_passage - s0, "passage_err_m": p.passage_err_m,
      "approach_bearing": p.approach_bearing, "bearing_src": p.bearing_src,
      "k_truth": p.k, "k_estimator": p.k_estimator, "k_n": p.k_n, "k_v_ego": p.k_v_ego,
      "k_ahead_max": k_ahead, "k_ahead_v_ego": k_ahead_v,
      "a_lat_measured": p.a_lat_max, "a_lat_src": p.a_lat_src,
      "dq_state": p.dq_state, "dq_src": p.dq_src, "dq_why": p.dq_why,
      "posted_ms": p.posted if p.posted else start.posted,
      "highway_class": p.hwy or start.hwy,
      "n_ticks": p.n_ticks, "source": start.src,
    })
    stats["ep_kept"] += 1
  return eps


# ---------------------------------------------------------------------------------------------
# driver
# ---------------------------------------------------------------------------------------------

def _run_pipeline(drives, params: CurveDBParams, dq_mask: int, log, reports):
  """Drives -> observations + episodes. The one place the per-drive gates live, so the
  file-driven and record-driven entry points cannot diverge."""
  # Pre-seeded so a drop reason that never fired prints `0` instead of being ABSENT from the tally
  # (Fable 2026-09-17). "Both counters are 0" was being INFERRED from their absence, which is the
  # same silence Rule 2 bans: a never-incremented Counter key and a never-reached code path look
  # identical from the output.
  stats: Counter = Counter(dict.fromkeys(TALLY_KEYS, 0))
  observations: list[Observation] = []
  episodes: list[dict] = []
  for d in drives:
    stats["drives"] += 1
    if d.gps_vs_odo is not None and not ODO_GPS_RATIO_BAND[0] <= d.gps_vs_odo <= ODO_GPS_RATIO_BAND[1]:
      # DROPPED, not merely reported (Fable 2026-09-17). vEgo in the wrong unit, or a fix not
      # moving with the truck, shifts every extent, the approach-bearing reference and the
      # lookahead window by an unknown factor -- the 2026-08-26 Tesla drive read 0.61, i.e. ~1.6x
      # off. Being loud satisfies Rule 2; letting the rows in anyway does not, because downstream
      # they are indistinguishable from good ones.
      stats["drive_dropped_odometer_disagrees_with_gps"] += 1
      log(f"  !! {d.date} {d.car}: GPS/odometer ratio {d.gps_vs_odo:.2f} -- DROPPING this drive; " +
          "its extents would be measured against a distance the two sources disagree about")
      continue
    sites = find_sites(d, params, stats)
    passages = [p for p in (measure_passage(d, s, params, stats, dq_mask) for s in sites)
                if p is not None]
    stats["passages"] += len(passages)
    observations.extend(observations_for_drive(d, passages, params, stats))
    episodes.extend(find_episodes(d, passages, params, stats))
  return observations, episodes, reports, stats, drives


def ingest(paths, params: CurveDBParams, log=print, dq_mask: int = DQ_ALL):
  log(f"curvedb ingest -- {len(paths)} file(s), disqualifiers in force: {dq_names(dq_mask)}")
  log("")
  log("PER-FILE LOAD (dup = already seen on (car, ev, t); clock = outside file median +-36 h)")
  ticks, reports = load_corpus(paths, log=log)
  log("")
  log(f"{len(ticks)} unique ticks after dedup")
  return _run_pipeline(split_drives(ticks), params, dq_mask, log, reports)


def ingest_records(records, params: CurveDBParams = PROVISIONAL_PARAMS, dq_mask: int = DQ_ALL):
  """The same pipeline as `ingest()`, driven from already-parsed records instead of files.

  Exists so a test can exercise the DRIVER loop -- the odometer/GPS drop and the pre-seeded tally
  live there and nowhere else, and a test that reimplemented the loop would not be testing it."""
  ticks = [t for t in (normalise(r, f"mem:{i}") for i, r in enumerate(records)) if t is not None]
  ticks.sort(key=lambda x: (x.car, x.t))
  return _run_pipeline(split_drives(ticks), params, dq_mask, log=lambda *a: None, reports=[])


def print_reports(reports, stats, drives, log=print):
  log("")
  log("=" * 110)
  log("PER-FILE CAPABILITY -- which curvedb fields a corpus actually carries, and its pose sign")
  log("=" * 110)
  for r in reports:
    if not r.kept:
      log(f"{os.path.relpath(r.path)}\n    CONTRIBUTES NOTHING: {r.records} records, {r.ticks} " +
          f"ticks, {r.duplicates} duplicates, {r.clock_dropped} clock-dropped, {r.no_fix} no fix")
      continue
    alive = [f for f in AUDIT_FIELDS if r.fields_alive.get(f)]
    dead = [f for f in AUDIT_FIELDS if r.fields_present.get(f) and not r.fields_alive.get(f)]
    log(f"{os.path.relpath(r.path)}   [{r.span_pt}]  kept={r.kept} " +
        f"cars={','.join(sorted(r.cars))}")
    log(f"    alive : {','.join(alive) or '-- NONE --'}")
    if dead:
      log(f"    PRESENT BUT ALWAYS NULL/ZERO : {','.join(dead)}")
    log(f"    pose sign: {r.pose_sign}")
  log("")
  log("=" * 110)
  log("INGEST TALLY -- every drop has a count; a zero below is a claim, not a silence")
  log("=" * 110)
  for k in sorted(stats):
    log(f"  {k:<44} {stats[k]}")
  ratios = [d.gps_vs_odo for d in drives if d.gps_vs_odo is not None]
  if ratios:
    log(f"  {'gps/odometer ratio (median over drives)':<44} {statistics.median(ratios):.3f}" +
        "   <- must be ~1.0; the only independent witness that vEgo is in m/s")


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__,
                               formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("paths", nargs="+")
  ap.add_argument("--out-observations", required=True)
  ap.add_argument("--out-episodes", required=True)
  ap.add_argument("--dq-flags", default="sat,drv,lc,blnk",
                  help="which section 3.5 causes disqualify a pass. The default is the design's " +
                       "own list; a SHORTER list is a sensitivity run showing what the rule costs, " +
                       "and is less faithful, not more correct. An unattributed `dq` always counts")
  ap.add_argument("--quiet", action="store_true")
  args = ap.parse_args(argv)

  def log(*a):
    if not args.quiet:
      print(*a)

  dq_mask = dq_mask_from_names(args.dq_flags)
  if dq_mask != DQ_ALL:
    log(f"!! RELAXED disqualifier set: {args.dq_flags}. This is a sensitivity run, not a result.")
  observations, episodes, reports, stats, drives = ingest(args.paths, PROVISIONAL_PARAMS, log=log,
                                                          dq_mask=dq_mask)
  print_reports(reports, stats, drives, log=log)

  with open(args.out_observations, "w") as f:
    for o in observations:
      f.write(json.dumps(dataclasses.asdict(o)) + "\n")
  with open(args.out_episodes, "w") as f:
    for e in episodes:
      f.write(json.dumps(e) + "\n")
  log("")
  log(f"WROTE {len(observations)} observations -> {args.out_observations}")
  log(f"WROTE {len(episodes)} episodes     -> {args.out_episodes}")
  if not observations:
    log("!! ZERO OBSERVATIONS. That is a result about the METHOD or the corpus, not about the road " +
        "-- read the tally above before reporting it as a finding.")
    return 1
  return 0


def load_observations(path) -> list[Observation]:
  out = []
  with open(path) as f:
    for lineno, line in enumerate(f, 1):
      line = line.strip()
      if not line:
        continue
      try:
        out.append(Observation(**json.loads(line)))
      except Exception as e:
        raise ValueError(f"{path}:{lineno}: not a curvedb observation: {e}") from e
  return out


def load_episodes(path) -> list[dict]:
  out = []
  with open(path) as f:
    for lineno, line in enumerate(f, 1):
      line = line.strip()
      if not line:
        continue
      try:
        out.append(json.loads(line))
      except Exception as e:
        raise ValueError(f"{path}:{lineno}: not JSON: {e}") from e
  return out


if __name__ == "__main__":
  sys.exit(main())
