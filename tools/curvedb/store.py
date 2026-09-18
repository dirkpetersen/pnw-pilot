"""curvedb C1 -- the row store, the matcher, the update rules and the authority gate.

CURVEDB2PNW.md sections 4, 5, 6 and 8.2. **This is the module a future on-car consumer imports
unchanged.** If the section 7 replay validated one implementation and the car later ran a different
one, the replay would have proved nothing -- so the matching, the update rules and the authority
gate live here and nowhere else. `ingest.py` and `replay.py` are callers, not re-implementations.

Contract, per section 8.3's C1 row ("drops into any fork, any path, any 0.11.x"):

* **zero openpilot imports** -- stdlib only, so it resolves on 0.11.1 and 0.11.2 alike and cannot
  drag `cereal` (which is NOT import-stable across those versions) in behind it;
* **no I/O** -- `match()` is pure geometry; snapshots are handed in as already-parsed objects;
* **no clock** -- every time is passed in. `Observation.date` is a *PT calendar date string*, and it
  is the leave-one-date-out key, so it must be computed once, by the caller, from the record's own
  epoch (the device runs UTC, the driver lives in Pacific -- rendering it here would bake a timezone
  into the core).

**UNITS ARE SI AND UNLABELLED FIELD NAMES ARE BANNED.** Curvature `k` is 1/m, speeds are m/s,
lateral acceleration m/s^2, bearings degrees clockwise from true north, distances metres.

## What is stored, and what is not

**Curvature, never a speed** (section 4). If a row held "the speed that worked", auto-editing closes
a loop: the system slows to 45, records "45 worked", and next time has more reason to slow. Speed is
always derived at read time: `v = sqrt(a_lat / k)`.

That rule also decides how the DOWN rule (section 6.2) is represented here. The design writes DOWN as
`v_row = sqrt(A_LAT_car / |slKCmd|)` -- a speed. Storing the **curvature** `|slKCmd|` instead is
algebraically the same number and keeps section 4 intact, and it makes "DOWN is never overwritten by
UP" (D8) fall out of a `max` instead of needing precedence bookkeeping: both rules can only ever
*raise* `k_eff`, and raising `k` only ever *lowers* the derived speed. Recorded as a deliberate
restatement, not a silent change.

## What this module deliberately does NOT do

`authority()` REFUSES rather than defaulting, in four places where the design or the data leaves a
hole. Each refusal returns a reason string; none of them is silent (Rule 2):

* **no posted limit -> no authority** (section 6.4, explicit -- "cap by the driver's set speed" is
  full cancel by another name);
* **no known highway class -> no authority.** Section 6.4 only excludes *ramps*. This module treats
  an absent/unknown class as "might be a ramp", because absence of evidence is not evidence of
  absence and a ramp is where an under-brake costs most. STRICTER THAN THE DESIGN -- flagged.
* **no car envelope for the platform -> no authority.** Section 8.2 suggests "a `CarSpecs` physics
  fallback for unknown keys", but Fable's own correction in that same section is that **mass does not
  predict a lateral ceiling** (the Lightning's ~4.5 m/s^2 is PSCM slew, not grip). A physics fallback
  would therefore be an invented number wearing a derivation. DEVIATES FROM THE DESIGN -- flagged.
* **fewer than `min_passes` on fewer than `min_dates` dates -> no authority** (D6).

The asymmetry that justifies any of this: today's failures are over-braking -- annoying, safe. A
wrong row produces an UNDER-brake. So `cancel_target()` can only ever raise a commanded target
*toward* the reference the driver already chose, never above it.
"""
from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass, field, replace

__all__ = [
  "PROVISIONAL_ENVELOPES", "PROVISIONAL_PARAMS", "UNKNOWN_HIGHWAY_CLASSES", "Authority",
  "CarEnvelope", "CurveDB", "CurveDBError", "CurveDBParams", "CurveRow", "Observation",
  "authority", "bearing_diff_deg", "cancel_target", "haversine_m", "initial_bearing_deg",
  "load_snapshot_or_empty", "speed_for_curvature", "tighten", "with_params",
]

R_EARTH_M = 6371000.0
SNAPSHOT_VERSION = 1

# Spellings that mean "the map did not tell us what road this is". `unknown` is a real member of
# cereal/custom.capnp's HighwayClass (the way's highway tag was not one of the listed values, or the
# tiles predate the field), and `str()` of the capnp enum renders it as the bare name.
UNKNOWN_HIGHWAY_CLASSES = ("", "unknown", "None", "none")


class CurveDBError(Exception):
  """A row store that cannot be trusted. Section 8.1: the consumer must fail SAFE to *no database*
  -- but it must SAY SO. Never catch this and continue with an empty DB without logging."""


# ---------------------------------------------------------------------------------------------
# geometry -- pure, and the single implementation ingest/replay/the car all share
# ---------------------------------------------------------------------------------------------

def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
  p1, p2 = math.radians(lat1), math.radians(lat2)
  dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
  a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
  return 2.0 * R_EARTH_M * math.asin(min(1.0, math.sqrt(a)))


def initial_bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
  """Great-circle initial bearing, degrees clockwise from true north, in [0, 360)."""
  p1, p2 = math.radians(lat1), math.radians(lat2)
  dl = math.radians(lon2 - lon1)
  y = math.sin(dl) * math.cos(p2)
  x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
  return math.degrees(math.atan2(y, x)) % 360.0


def bearing_diff_deg(a: float, b: float) -> float:
  """Smallest unsigned angle between two bearings, in [0, 180].

  D4 is why this is a CONTINUOUS tolerance and not a bucket index: v1's 45-degree buckets straddle
  47 % of real curves, so a pass's tight fragment landed in a different bucket than the lookup and
  the row was never found. A bucket has edges wherever the modulus falls; an angular distance has
  none."""
  return abs((a - b + 180.0) % 360.0 - 180.0)


def speed_for_curvature(a_lat_ms2: float, k: float) -> float:
  """v = sqrt(a_lat / k). The ONLY place a speed is ever produced from a row (section 5)."""
  if not (math.isfinite(a_lat_ms2) and math.isfinite(k)) or a_lat_ms2 <= 0.0 or k <= 0.0:
    raise CurveDBError(f"speed_for_curvature needs positive finite inputs, got a_lat={a_lat_ms2!r} k={k!r}")
  return math.sqrt(a_lat_ms2 / k)


# ---------------------------------------------------------------------------------------------
# parameters -- NO DEFAULTS. every value is a decision someone has to sign.
# ---------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class CurveDBParams:
  """Every threshold this feature has, in one place, with **no default values**.

  A default here would be an invented constant wearing the authority of code. The corpus chooses
  these; until it has, callers pass `PROVISIONAL_PARAMS` *by name*, which is greppable.

  The first group defines what a row MEANS (used by ingest when measuring a pass); the second
  defines how a row is FOUND and whether it may act (used by the car at lookup time). They are one
  object because the two halves must agree -- a row measured with one approach-bearing convention
  and looked up with another is a row that is never found.
  """
  # --- measurement (ingest) ---------------------------------------------------------------
  # Section 6.3: extent = [candidate - 25 m, candidate + >=150 m run-out]. mapd's point can sit
  # 56-125 m from the bend, so the window is asymmetric and generous forwards.
  extent_back_m: float
  extent_fwd_m: float
  # Where along the approach the pass's direction is measured. This must be the distance at which
  # the CAR will later ask, or ingest and lookup disagree about what "the approach bearing" is.
  approach_bearing_ref_m: float
  # Section 6.2: the override-trigger filter. Of 2,986 steering overrides above 27 mph only ~113
  # ticks were plausibly about curve speed; the rest are lane changes, exits and repositioning.
  down_trigger_a_lat_ms2: float
  # Below this the truck is parked/crawling and the yaw-derived curvature is 1/v noise.
  min_speed_ms: float

  # --- lookup + authority (the car) -------------------------------------------------------
  site_radius_m: float
  heading_tol_deg: float
  min_passes: int
  min_dates: int
  # Section 8.2: the *policy* half of the envelope -- a comfort target, shared across cars. The
  # per-car hard ceiling lives in CarEnvelope.
  a_lat_comfort_ms2: float
  # Section 6.4: OSM ramp speed limits are routinely inherited from the mainline, so a wrong posted
  # limit is the one thing the never-above-posted cap cannot see.
  ramp_highway_classes: tuple[str, ...]

  def __post_init__(self):
    bad = []
    for name in ("extent_back_m", "extent_fwd_m", "approach_bearing_ref_m",
                 "down_trigger_a_lat_ms2", "min_speed_ms", "site_radius_m",
                 "heading_tol_deg", "a_lat_comfort_ms2"):
      v = getattr(self, name)
      if not isinstance(v, int | float) or not math.isfinite(v) or v <= 0.0:
        bad.append(f"{name}={v!r}")
    for name in ("min_passes", "min_dates"):
      v = getattr(self, name)
      if not isinstance(v, int) or v < 1:
        bad.append(f"{name}={v!r}")
    if self.heading_tol_deg >= 180.0:
      bad.append(f"heading_tol_deg={self.heading_tol_deg!r} matches every direction")
    if bad:
      raise CurveDBError("CurveDBParams: " + ", ".join(bad))


# ⚠️ PROVISIONAL. Not one of these is a value anyone has defended with data yet; they exist so the
# replay can RUN, and every one of them is listed in the report that ships with this branch. The
# name is deliberately shouty so `grep PROVISIONAL` finds every call site.
PROVISIONAL_PARAMS = CurveDBParams(
  # cited, not invented: section 6.3 states both extents.
  extent_back_m=25.0,
  extent_fwd_m=150.0,
  # PROVISIONAL. ICBM's far source reaches MAP_SOURCE_HORIZON_M (500 m) and CES's own 10 s horizon is
  # ~308 m at 90 mph, so the decision is taken somewhere in 150-500 m. 300 m is the middle of that
  # and nothing more. Sensitivity is measured in the report.
  approach_bearing_ref_m=300.0,
  # cited: section 6.2's own trigger analysis, and the same 2.5 the design uses for "real".
  down_trigger_a_lat_ms2=2.5,
  # cited: tools/curvedb_telemetry_check.py already treats 5 m/s as the parked/crawling floor.
  min_speed_ms=5.0,
  # PROVISIONAL. Must cover GPS fuzz plus the spread between the node ICBM matched on one pass and
  # the node it matched on another. Section 6.3 notes mapd publishes no identity, so this radius is
  # standing in for an OSM way/node ID we do not have. Measured in the report.
  site_radius_m=40.0,
  # PROVISIONAL. Section 6.3 killed 45-degree BUCKETS (D4); it did not say what tolerance replaces
  # them. 35 degrees is under one bucket width and above the 19-degree median curve sweep.
  heading_tol_deg=35.0,
  # cited: section 6.4 / D6.
  min_passes=2,
  min_dates=2,
  # cited: VTSC's A_LAT_TARGET, retuned to 2.5 on 2026-07-01 and used as "the design's own comfort
  # target" throughout CURVEDB2PNW.md and the 2026-09-17 drive report.
  a_lat_comfort_ms2=2.5,
  # cited: the *_link members of cereal/custom.capnp's HighwayClass, spelled exactly as ces_events
  # logs them (camelCase -- `motorwayLink`, not the OSM `motorway_link`; getting this wrong would
  # silently grant authority on every ramp, which is precisely where an under-brake costs most).
  # Section 6.4 excludes ramps from authority. `unknown` is not listed here because it is handled
  # one step earlier, by the no-known-class refusal -- a ramp we cannot see is still a ramp.
  ramp_highway_classes=("motorwayLink", "trunkLink", "primaryLink", "secondaryLink",
                        "tertiaryLink"),
)


@dataclass(frozen=True)
class CarEnvelope:
  """Section 8.2's car table, keyed by an OPAQUE platform string.

  Opaque on purpose: the Tesla legacy platform names do not exist upstream, so the identifier is
  fork-local. Keeping it a plain string (rather than an enum imported from opendbc) is what lets
  C1 drop into another fork. Per this project's capability-view rule, feature code reads a
  capability -- it never branches on the fingerprint itself."""
  platform: str
  hard_ceiling_a_lat_ms2: float          # a CAR property (the Lightning's ~4.5 is PSCM slew)
  provenance: str                        # where the number came from, in words. never empty.

  def __post_init__(self):
    if not self.platform:
      raise CurveDBError("CarEnvelope needs a platform string")
    if not (isinstance(self.hard_ceiling_a_lat_ms2, int | float)
            and math.isfinite(self.hard_ceiling_a_lat_ms2) and self.hard_ceiling_a_lat_ms2 > 0.0):
      raise CurveDBError(f"CarEnvelope({self.platform}): bad ceiling {self.hard_ceiling_a_lat_ms2!r}")
    if not self.provenance:
      raise CurveDBError(f"CarEnvelope({self.platform}): a ceiling with no stated provenance is an " +
                         "invented constant")


# ⚠️ ONE ENTRY, and the Tesla is deliberately absent. `LIGHTNING-STEERING-LIMITS.md` measured the
# truck; nothing has measured the Raven, and section 8.2's suggested CarSpecs fallback is exactly the
# mass-predicts-grip reasoning Fable rejected in that same section. An absent platform gets NO
# AUTHORITY and a reason string -- which is the honest state of knowledge, and is visible in the
# replay's refusal table instead of hiding as a plausible default.
PROVISIONAL_ENVELOPES: dict[str, CarEnvelope] = {
  "FORD_F_150_LIGHTNING_MK1": CarEnvelope(
    platform="FORD_F_150_LIGHTNING_MK1",
    hard_ceiling_a_lat_ms2=4.5,
    provenance="LIGHTNING-STEERING-LIMITS.md, measured at Crown Hill: ~4.5 m/s^2 ceiling, " +
               "binding limit is ~18-20 deg/s PSCM slew",
  ),
}


# ---------------------------------------------------------------------------------------------
# rows
# ---------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Observation:
  """One admissible pass over one site, reduced to the only things a row is allowed to remember.

  `k` is a MAGNITUDE (1/m). The store never keys on the sign of curvature -- direction is the GPS
  approach bearing, which is measured, unambiguous, and was not affected by the 2026-09-17 pose
  sign inversion (`36f914a17c`). That is deliberate: it is what bounds the blast radius of that
  defect to the signed telemetry columns.

  `estimator` and `site_src` are PROVENANCE and are never optional. A row built from a 1 Hz sample
  of a quantity section 3.3 argues must be peaked is a *weaker* row than one built from `kPeak`, and
  it is weak in the UNSAFE direction (an under-read k derives too high a speed). A row built from a
  reconstructed candidate position is weaker than one keyed on the coordinates the car logged. The
  replay reports the mix; it must never be inferable only by reading the ingest source.
  """
  date: str                       # PT calendar date, YYYY-MM-DD -- the leave-one-date-out key
  t: float                        # epoch seconds of the pass, for audit
  car: str                        # opaque platform string
  # The drive this pass belongs to. Leave-one-DATE-out is what section 3.9 item 6 asks for, but a
  # drive that crosses PT midnight between the ICBM decision and the site would put the episode's
  # OWN pass on the other side of the date boundary and back into the database that judges it
  # (Fable 2026-09-17). Measured: zero such episodes in the 2026-09-17 corpus -- but the whole
  # non-circularity claim should not rest on the owner never driving at midnight, so the replay
  # excludes the drive as well as the date and `_assert_lodo` checks both.
  drive_id: str
  site_lat: float
  site_lon: float
  bearing_deg: float              # approach bearing (section 6.3), NOT the bearing in the bend
  k: float                        # curvature magnitude, 1/m, > 0
  kind: str                       # "up" (section 6.1) | "down" (section 6.2)
  estimator: str                  # e.g. "kPeak100" | "sample1hz_cmd_actl" | "sample1hz_cmd"
  site_src: str                   # "logged" (mapLat/mapLon) | "track" (reconstructed)
  source: str                     # file:line, for audit
  posted_ms: float | None         # posted limit seen on this pass (context, NOT part of the key)
  highway_class: str | None
  n_ticks: int                    # ticks the extent actually contained
  # "clean"  -- section 3.5's roll-up (or the 1 Hz sampled flags) said no disqualifier;
  # "unknown" -- the corpus predates those flags entirely. NOT the same thing, and conflating them
  #             is how an override-contaminated pass becomes a row. Callers filter; the store
  #             records. A "dirty" pass is dropped at ingest and never reaches an Observation.
  dq_state: str
  # WHICH disqualifier evidence produced that state -- `rollup100` is section 3.5's 100 Hz OR,
  # `sampled1hz` is the older corpora's instantaneous flags, `none` is no evidence at all. Section
  # 3.5 exists precisely because a 1 Hz sample "may have been between events at the sample
  # instant", so a `clean` from a sample is weaker than a `clean` from the roll-up -- the same
  # discipline `estimator` applies to curvature, which this field used to be missing (Fable).
  dq_src: str

  def __post_init__(self):
    bad = []
    if not self.date or len(self.date) != 10 or self.date[4] != "-":
      bad.append(f"date={self.date!r} is not a YYYY-MM-DD PT date")
    if not self.drive_id:
      bad.append("drive_id is mandatory -- it is half of the non-circularity key")
    if self.kind not in ("up", "down"):
      bad.append(f"kind={self.kind!r}")
    if self.dq_state not in ("clean", "unknown"):
      bad.append(f"dq_state={self.dq_state!r}")
    if self.dq_src not in ("rollup100", "sampled1hz", "none"):
      bad.append(f"dq_src={self.dq_src!r}")
    if not self.estimator or not self.site_src:
      bad.append("estimator/site_src provenance is mandatory")
    if not (isinstance(self.k, int | float) and math.isfinite(self.k) and self.k > 0.0):
      # P1-B: an exact 0.0 in the curvature family is a dead sensor, not a straight road.
      bad.append(f"k={self.k!r} -- zero/negative/non-finite curvature is never a measurement")
    for name in ("site_lat", "site_lon", "bearing_deg"):
      v = getattr(self, name)
      if not (isinstance(v, int | float) and math.isfinite(v)):
        bad.append(f"{name}={v!r}")
    if not bad:
      if not -90.0 <= self.site_lat <= 90.0 or not -180.0 <= self.site_lon <= 180.0:
        bad.append(f"site ({self.site_lat}, {self.site_lon}) is not on the planet")
      if not 0.0 <= self.bearing_deg < 360.0:
        bad.append(f"bearing_deg={self.bearing_deg!r} outside [0, 360)")
    if bad:
      raise CurveDBError("Observation: " + ", ".join(bad))


@dataclass
class CurveRow:
  """One (site, approach direction) with the curvature the road was measured to have.

  The anchor (`site_lat`/`site_lon`/`bearing_deg`) is the FIRST observation's and never moves.
  Letting it drift to a running mean would let a chain of passes each just inside `site_radius_m`
  walk a row down the road -- the classic single-link-clustering failure, and one that would show up
  as a row whose position no longer names any curve."""
  site_lat: float
  site_lon: float
  bearing_deg: float
  observations: list[Observation] = field(default_factory=list)

  # -- section 6.1 UP: max across passes. One genuinely tight pass must never be averaged away by
  #    ten gentle ones, so this is a max and not a mean or a quantile.
  @property
  def k_up(self) -> float | None:
    ks = [o.k for o in self.observations if o.kind == "up"]
    return max(ks) if ks else None

  # -- section 6.2 DOWN: the driver said "too fast". Stored as the curvature that implies the lower
  #    speed, so it can only ever raise k_eff (see the module docstring).
  @property
  def k_down(self) -> float | None:
    ks = [o.k for o in self.observations if o.kind == "down"]
    return max(ks) if ks else None

  @property
  def k_eff(self) -> float | None:
    ks = [k for k in (self.k_up, self.k_down) if k is not None]
    return max(ks) if ks else None

  @property
  def dates(self) -> tuple[str, ...]:
    return tuple(sorted({o.date for o in self.observations}))

  @property
  def cars(self) -> tuple[str, ...]:
    return tuple(sorted({o.car for o in self.observations}))

  @property
  def estimators(self) -> tuple[str, ...]:
    return tuple(sorted({o.estimator for o in self.observations}))

  @property
  def site_srcs(self) -> tuple[str, ...]:
    return tuple(sorted({o.site_src for o in self.observations}))

  @property
  def n_passes(self) -> int:
    """Distinct DRIVES, not observations (Fable 2026-09-17).

    One pass can produce an UP and a DOWN, and counting observations would let a single drive
    satisfy half of D6's ">=2 admissible passes" on its own. `min_dates` happens to dominate today
    and DOWN has never fired, so this is currently a distinction without a difference -- which is
    exactly when it is cheapest to get right."""
    return len(self.drives)

  @property
  def drives(self) -> tuple[str, ...]:
    return tuple(sorted({o.drive_id for o in self.observations}))

  @property
  def n_observations(self) -> int:
    return len(self.observations)

  def to_json(self) -> dict:
    return {
      "site_lat": self.site_lat, "site_lon": self.site_lon, "bearing_deg": self.bearing_deg,
      "k_up": self.k_up, "k_down": self.k_down,
      "n_passes": self.n_passes, "dates": list(self.dates), "cars": list(self.cars),
      "estimators": list(self.estimators), "site_srcs": list(self.site_srcs),
      "observations": [vars(o) for o in self.observations],
    }


# ---------------------------------------------------------------------------------------------
# the database
# ---------------------------------------------------------------------------------------------

class CurveDB:
  """The road table (section 5) plus the matcher (section 6.3).

  Built by GROUPING an observation list, not by incremental merge, because that is also what
  section 8.1's compaction is: the append-only JSONL holds observations, the snapshot holds the
  grouped rows, and rebuilding from the log must give byte-identical rows or the snapshot is a
  second source of truth. `build()` therefore sorts its input into a canonical order first, so the
  same observations always produce the same rows regardless of the order they were appended in.
  """

  def __init__(self, params: CurveDBParams, rows: list[CurveRow] | None = None):
    if not isinstance(params, CurveDBParams):
      raise CurveDBError(f"CurveDB needs CurveDBParams, got {type(params).__name__}")
    self.params = params
    self.rows: list[CurveRow] = list(rows or [])

  # ---- construction --------------------------------------------------------------------

  @classmethod
  def build(cls, observations: Iterable[Observation], params: CurveDBParams) -> CurveDB:
    obs = list(observations)
    for o in obs:
      if not isinstance(o, Observation):
        raise CurveDBError(f"build() takes Observations, got {type(o).__name__}")
    # Canonical order. Without it the anchor of every row is an accident of file-read order.
    obs.sort(key=lambda o: (o.date, o.t, o.site_lat, o.site_lon, o.bearing_deg, o.kind, o.source))
    db = cls(params)
    for o in obs:
      row = db._nearest(o.site_lat, o.site_lon, o.bearing_deg)
      if row is None:
        row = CurveRow(site_lat=o.site_lat, site_lon=o.site_lon, bearing_deg=o.bearing_deg)
        db.rows.append(row)
      row.observations.append(o)
    return db

  # ---- the matcher ---------------------------------------------------------------------

  def _nearest(self, lat: float, lon: float, bearing_deg: float) -> CurveRow | None:
    """Nearest row whose anchor is within `site_radius_m` AND whose approach bearing is within
    `heading_tol_deg`. Pure geometry, no I/O, no clock.

    A flat scan with a cheap bounding-box pre-filter, deliberately: section 8.1 puts the whole table
    at 2,000-5,000 rows, so this is a few thousand float compares at the 1 Hz the consumer would ask
    at. A spatial index would be a correctness surface (cell size vs. radius vs. the cos(lat)
    longitude scale) bought for no measurable gain."""
    for name, v in (("lat", lat), ("lon", lon), ("bearing_deg", bearing_deg)):
      if not (isinstance(v, int | float) and math.isfinite(v)):
        raise CurveDBError(f"match(): {name}={v!r} is not a usable fix")
    r = self.params.site_radius_m
    dlat_max = r / 110574.0 * 1.05                      # metres -> degrees latitude, 5 % slack
    coslat = max(math.cos(math.radians(lat)), 1e-6)
    dlon_max = r / (111320.0 * coslat) * 1.05
    best, best_d = None, float("inf")
    for row in self.rows:
      if abs(row.site_lat - lat) > dlat_max or abs(row.site_lon - lon) > dlon_max:
        continue
      if bearing_diff_deg(row.bearing_deg, bearing_deg) > self.params.heading_tol_deg:
        continue
      d = haversine_m(lat, lon, row.site_lat, row.site_lon)
      if d <= r and d < best_d:
        best, best_d = row, d
    return best

  def match(self, lat: float, lon: float, bearing_deg: float) -> CurveRow | None:
    """The car's lookup: given where the candidate is and which way we are approaching it, the row.

    Identical to the function that decides row identity at build time -- on purpose. If matching and
    merging could disagree, a pass could build a row the lookup can never find."""
    return self._nearest(lat, lon, bearing_deg)

  # ---- snapshot (section 8.1) ----------------------------------------------------------

  def to_snapshot(self) -> dict:
    return {
      "version": SNAPSHOT_VERSION,
      "params": vars(self.params),
      "rows": [r.to_json() for r in self.rows],
    }

  @classmethod
  def from_snapshot(cls, obj: dict) -> CurveDB:
    """Rebuild from a snapshot. Raises CurveDBError on anything it cannot vouch for.

    Section 8.1 says the consumer must fail SAFE to "no database" -- but silently substituting an
    empty DB for a corrupt one is the exact Rule 2 failure this project keeps paying for, so the
    failure is raised here and the CALLER is required to log it (see `load_snapshot_or_empty`)."""
    if not isinstance(obj, dict):
      raise CurveDBError(f"snapshot is a {type(obj).__name__}, not an object")
    if obj.get("version") != SNAPSHOT_VERSION:
      raise CurveDBError(f"snapshot version {obj.get('version')!r} != {SNAPSHOT_VERSION}")
    praw = obj.get("params")
    if not isinstance(praw, dict):
      raise CurveDBError("snapshot carries no params -- rows built with one matching tolerance and " +
                         "read with another are rows the lookup cannot find")
    try:
      praw = dict(praw)
      praw["ramp_highway_classes"] = tuple(praw.get("ramp_highway_classes") or ())
      params = CurveDBParams(**praw)
    except TypeError as e:
      raise CurveDBError(f"snapshot params do not match CurveDBParams: {e}") from e
    rows = []
    for raw in obj.get("rows") or []:
      try:
        obs = [Observation(**o) for o in raw["observations"]]
        rows.append(CurveRow(site_lat=raw["site_lat"], site_lon=raw["site_lon"],
                             bearing_deg=raw["bearing_deg"], observations=obs))
      except (KeyError, TypeError) as e:
        raise CurveDBError(f"snapshot row is malformed: {e}") from e
    return cls(params, rows)


def load_snapshot_or_empty(obj, on_error) -> CurveDB | None:
  """Section 8.1's fail-safe, with the logging made non-optional.

  `on_error` has NO DEFAULT and is called with the exception before an empty result is returned.
  A caller that wants to swallow the failure has to write the swallow down."""
  if not callable(on_error):
    raise CurveDBError("load_snapshot_or_empty needs an on_error callback -- a corrupt database " +
                       "that reports nothing is worse than no database")
  try:
    return CurveDB.from_snapshot(obj)
  except CurveDBError as e:
    on_error(e)
    return None


# ---------------------------------------------------------------------------------------------
# authority + the cancel formula (section 6.4)
# ---------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Authority:
  """The answer to "may this row act, and if so what speed does it justify".

  `reason` is populated on GRANT as well as on refusal, so the replay can tabulate why rows do not
  act without re-deriving the gate. A refusal that does not say why is a check whose success path
  cannot be told from its failure path."""
  granted: bool
  v_row_ms: float | None
  reason: str


def authority(row: CurveRow | None, *, posted_limit_ms, highway_class, platform: str,
              params: CurveDBParams, envelopes: dict[str, CarEnvelope]) -> Authority:
  """Section 6.4. Keyword-only past `row` because every one of these is a gate, and a positional
  mix-up between a speed and a class string would fail open."""
  if row is None:
    return Authority(False, None, "no row")
  k = row.k_eff
  if k is None or not math.isfinite(k) or k <= 0.0:
    return Authority(False, None, "row has no usable curvature")
  if row.n_passes < params.min_passes:
    return Authority(False, None, f"only {row.n_passes} pass(es), need {params.min_passes}")
  if len(row.dates) < params.min_dates:
    return Authority(False, None, f"only {len(row.dates)} date(s), need {params.min_dates}")
  if not isinstance(posted_limit_ms, int | float) or not math.isfinite(posted_limit_ms) \
     or posted_limit_ms <= 0.0:
    # Section 6.4, verbatim: "No posted limit -> no authority."
    return Authority(False, None, "no posted limit")
  # ⚠️ NORMALISE ONCE, then compare the normalised value everywhere (Fable 2026-09-17).
  # The unknown test used to `str()` and the ramp test did not. Offline both work, because the
  # value arrives from JSON as a string -- but this module is declared "imported by the car
  # unchanged", and a consumer handing in the capnp enum would make the ramp test fail **open**
  # (`HighwayClass.motorwayLink not in ("motorwayLink", ...)` is True), granting authority on
  # exactly the road section 6.4 says an under-brake costs most on. Same shape as the capnp
  # `str()` enum trap that silently killed the Pro Power feature.
  hc = "" if highway_class is None else str(highway_class)
  if not hc or hc in UNKNOWN_HIGHWAY_CLASSES:
    # `unknown` is a REAL member of cereal's HighwayClass (tag not recognised, or tiles that
    # predate the field) and it arrives as the literal string "unknown", which is truthy. Testing
    # only `not highway_class` would have granted authority on every one of them -- 990 of 7,813
    # records on the 2026-09-12 weekend corpus alone.
    return Authority(False, None, f"highway class unknown ({highway_class!r}; treated as maybe-ramp)")
  if hc in params.ramp_highway_classes:
    return Authority(False, None, f"ramp ({hc})")
  env = envelopes.get(platform)
  if env is None:
    return Authority(False, None, f"no measured lateral envelope for platform {platform!r}")
  a_lat = min(params.a_lat_comfort_ms2, env.hard_ceiling_a_lat_ms2)
  v = speed_for_curvature(a_lat, k)
  v = min(v, float(posted_limit_ms))                   # never above posted, section 6.4
  return Authority(True, v, f"{row.n_passes} passes on {len(row.dates)} dates, k={k:.6f}, " +
                            f"a_lat={a_lat:.2f}, capped at posted {posted_limit_ms:.1f}")


def cancel_target(ref_ms: float, icbm_target_ms: float, v_row_ms: float | None) -> float:
  """`target = min(ref, max(icbm_target, v_row))` -- section 6.4, unchanged.

  `ref` is the speed the truck would hold with no curve slowdown at all (the driver's set / the
  episode ceiling). The result is clamped to `ref` on the way out, which is the whole asymmetry:
  the database may only ever CANCEL OR REDUCE a slowdown. It is never a reason to go faster than
  the driver already asked for; only a reason not to slow."""
  for name, v in (("ref_ms", ref_ms), ("icbm_target_ms", icbm_target_ms)):
    if not isinstance(v, int | float) or not math.isfinite(v):
      raise CurveDBError(f"cancel_target: {name}={v!r}")
  if v_row_ms is None:
    return float(icbm_target_ms)
  if not isinstance(v_row_ms, int | float) or not math.isfinite(v_row_ms):
    raise CurveDBError(f"cancel_target: v_row_ms={v_row_ms!r}")
  return min(float(ref_ms), max(float(icbm_target_ms), float(v_row_ms)))


def with_params(db: CurveDB, params: CurveDBParams) -> CurveDB:
  """Re-group an existing DB's observations under different matching parameters.

  The parameter sweep in the report needs this, and doing it by rebuilding from the observations
  (rather than by editing rows) is what keeps "the snapshot is derivable from the log" true."""
  return CurveDB.build([o for r in db.rows for o in r.observations], params)


def tighten(params: CurveDBParams, **changes) -> CurveDBParams:
  """`dataclasses.replace` with the name this codebase can grep for in a sweep."""
  return replace(params, **changes)
