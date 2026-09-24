"""curvedb v2 -- the ROAD TABLE, built from drive logs (qlog/rlog) instead of `ces_events`.

Pure functions only: no I/O, no clock, no openpilot imports (same contract as `store.py`). Input is the
stream dict `v2_extract.py` writes; output is a list of anchors, each holding one value per admissible
pass. Everything a later step decides (a row's curvature, whether it may act) is computed from that list
at query time, so leave-one-date-out is a filter, not a rebuild.

WHAT CHANGED FROM v1, AND WHY (owner decisions 2026-09-23, docs/work-pending/...learned-curve-database...)
------------------------------------------------------------------------------------------------------
* **Curvature is measured, not commanded:** k = |livePose yaw rate| / vEgo, on BOTH cars. v1 used
  `kPeak` = max(|achieved|, |commanded|), which only exists from 2026-09-17 and only on the Lightning,
  and the Tesla half of v1 was the planner's COMMAND (`slKCmd`, deviation I8), not a measurement.
* **Every pass counts** -- openpilot steering, driver steering (`drv`), fully manual, both cars. The
  owner's safeguard replaces v1's disqualifier: a row's curvature is a MIDDLE PERCENTILE ACROSS DATES,
  never one pass, so a lane-wiggle or a lane change on one date cannot set the row.
* **Tesla is never sole evidence:** a row needs >= 1 non-Tesla date to have authority.
* **GPS quality (R9):** a pass is refused at an anchor when any fix inside its extent has no fix or a
  `bearingAccuracyDeg` above the limit. `horizontalAccuracy` is 0 on every fix on this device, so it
  cannot be used.
* **wayId cross-check:** a pass whose mapd wayId at the anchor disagrees with the anchor's majority way
  is not used for that row (the "parallel ramp matched by GPS error" objection).

Units: SI. k in 1/m, v in m/s, distances in m, bearings in degrees clockwise from true north.
"""
from __future__ import annotations

import bisect
import math
from collections import Counter
from dataclasses import dataclass, field

from openpilot.tools.curvedb.store import bearing_diff_deg, haversine_m, initial_bearing_deg

TESLA_PREFIX = "TESLA"
RAMP_CLASSES = ("motorwayLink", "trunkLink", "primaryLink", "secondaryLink", "tertiaryLink")
UNKNOWN_CLASSES = ("", "unknown", "None", "none")


@dataclass(frozen=True)
class V2Params:
  """Every threshold v2 has. NO DEFAULTS -- callers pass `PROVISIONAL_V2` by name (greppable)."""
  v_min_ms: float            # below this yaw/v is noise-dominated; no curvature sample
  smooth_s: float            # centered SIGNED moving mean window for k (suppresses lane-keeping wiggle)
  step_m: float              # resample spacing along each pass
  max_gps_gap_s: float       # interpolate position only between fixes closer than this
  bacc_max_deg: float        # R9: reject a pass at an anchor if any fix in the extent exceeds this
  site_radius_m: float       # anchor match radius (v1's provisional 40 m)
  heading_tol_deg: float     # anchor match heading tolerance (v1's provisional 35 deg)
  extent_back_m: float       # section 6.3 extent, as v1
  extent_fwd_m: float
  percentile: float          # the "middle percentile" across DATES (owner rule)
  min_dates: int             # owner rule / D6: >= 2 distinct PT dates
  max_date_spread: float     # refuse authority when the dates disagree by more than this ratio ...
  spread_floor_k: float      # ... AND by more than this absolute curvature (1/m)
  way_check: bool            # drop passes whose wayId at the anchor disagrees with the anchor majority
  branch_radius_m: float     # a row pools only passes whose extent ENDS within this of the query's end point


PROVISIONAL_V2 = V2Params(
  v_min_ms=5.0,          # cited: v1's min_speed_ms (tools/curvedb_telemetry_check.py's floor). MEASURED: at
                         # 8 m/s most city episodes lost their truth (the pass ends at the junction).
                         # A 0.005 rad/s gyro bias is k=0.001 at 5 m/s -- small against city curves.
  smooth_s=1.0,          # PROVISIONAL, chosen by the kPeak validation (docs/CURVEDB-V2-BUILD.md s2)
  step_m=25.0,           # PROVISIONAL. Well under the 40 m radius so every anchor sees every pass.
  max_gps_gap_s=2.5,     # qlog GPS is 1 Hz; one missing fix is tolerated, two are not.
  bacc_max_deg=10.0,     # the assessment's R9 proposal, taken as the task specifies
  site_radius_m=40.0,    # v1 PROVISIONAL, unchanged so v1 and v2 are comparable
  heading_tol_deg=35.0,  # v1 PROVISIONAL, unchanged
  extent_back_m=25.0,    # cited: CURVEDB2PNW.md s6.3
  extent_fwd_m=150.0,    # cited: CURVEDB2PNW.md s6.3
  percentile=50.0,       # owner: "a middle percentile across passes on >= 2 distinct dates"
  min_dates=2,           # owner rule + D6
  max_date_spread=1.5,   # PROVISIONAL. A site whose dates disagree by >1.5x in k is not one road ...
  spread_floor_k=5e-4,   # ... unless the gap is < 5e-4 1/m: near-straight noise (0.0003 vs 0.0006 is 2x).
                         # Worst case the row under-reads the high date by 5e-4 (>= 3 dates, median) =
                         # 0.56 m/s^2 at 75 mph; by 2.5e-4 = 0.28 m/s^2 with 2 dates (mean). MEASURED:
                         # with a 2e-4 floor 2,878 of 3,619 refusals were rows with k < 1e-3.
  # OFF, MEASURED: on 1,168 of 18,047 anchors with wayIds on >= 2 dates, each date carries ONE way id and the
  # dates disagree -- OSM way ids change between map downloads and at way joins. The check dropped whole
  # genuine dates (09-17 Tumwater NB: 2 Tesla dates -> 1). Different-road mixing is caught by
  # max_date_spread instead.
  way_check=False,
  # PROVISIONAL. MEASURED: without it the worst LODO errors were all ROAD SPLITS -- a held-out pass that took
  # the Aurora surface street at the SR 99 portal (k 0.0099 vs a mainline row of 0.0021), and I-405 vs I-5 at
  # the N Portland split. A position+heading key cannot separate two branches that diverge INSIDE the 150 m
  # extent; where the pass is at the extent's end can. 15 m: > 3 lanes of offset plus GPS error, < the
  # separation of a ramp 150 m after its gore.
  branch_radius_m=15.0,
)


# ---------------------------------------------------------------------------------------------
# small numeric helpers
# ---------------------------------------------------------------------------------------------

def percentile(xs, p: float) -> float:
  """Linear-interpolated percentile of a non-empty sequence (numpy's default 'linear' method)."""
  s = sorted(xs)
  if not s:
    raise ValueError("percentile of an empty sequence")
  if len(s) == 1:
    return float(s[0])
  pos = (len(s) - 1) * p / 100.0
  lo = int(math.floor(pos))
  hi = min(lo + 1, len(s) - 1)
  return float(s[lo] + (s[hi] - s[lo]) * (pos - lo))


def _nearest_idx(ts, t: float, max_dt: float):
  """Index of the element of sorted `ts` nearest to t, or None if none within max_dt."""
  if not ts:
    return None
  i = bisect.bisect_left(ts, t)
  best = None
  for j in (i - 1, i):
    if 0 <= j < len(ts) and abs(ts[j] - t) <= max_dt and (best is None or abs(ts[j] - t) < abs(ts[best] - t)):
      best = j
  return best


def curvature(yaw_rate: float, v: float, v_min: float):
  """Signed path curvature from yaw rate, or None below v_min / on non-finite input."""
  if v is None or yaw_rate is None or not (math.isfinite(v) and math.isfinite(yaw_rate)) or v < v_min:
    return None
  return yaw_rate / v


def smooth_signed(ts, ks, window_s: float):
  """Centered moving mean of SIGNED k over +-window_s/2, skipping None. Averaging the SIGNED value is the
  point: lane-keeping wiggle alternates sign and cancels, a real bend does not. Returns a list aligned to
  ks (None where the input was None)."""
  n = len(ts)
  out = [None] * n
  half = window_s / 2.0
  lo = hi = 0
  acc, cnt = 0.0, 0
  for i in range(n):
    while hi < n and ts[hi] <= ts[i] + half:
      if ks[hi] is not None:
        acc += ks[hi]
        cnt += 1
      hi += 1
    while ts[lo] < ts[i] - half:
      if ks[lo] is not None:
        acc -= ks[lo]
        cnt -= 1
      lo += 1
    if ks[i] is not None and cnt:
      out[i] = acc / cnt
  return out


# ---------------------------------------------------------------------------------------------
# fusion: streams -> per-pose-tick samples
# ---------------------------------------------------------------------------------------------

@dataclass
class Sample:
  t: float                  # UNIX seconds
  v: float
  k: float | None           # smoothed signed curvature
  k_raw: float | None       # unsmoothed signed curvature (for the validation only)
  drv: bool                 # steeringPressed
  blk: bool
  lat_active: bool | None   # carControl.latActive (None = no carControl nearby)
  lat: float | None = None
  lon: float | None = None
  fix_ok: bool = False      # both bracketing fixes hasFix and within bacc limit
  bacc: float | None = None
  way: int = 0
  hwy: str = ""
  road: str = ""
  spl: float = 0.0
  mcs: float = 0.0


def fuse(doc: dict, off_s: float, params: V2Params) -> list[Sample]:
  """One segment's (or a concatenated route's) streams -> Samples at livePose times.

  `off_s` converts monotonic to UNIX seconds (route median of the GPS offsets; the caller decides).
  A pose tick with no carState within 0.3 s is dropped -- no speed, no curvature."""
  cs, cc, gps, mapd = doc["cs"], doc["cc"], doc["gps"], doc["mapd"]
  cs_t = [r[0] for r in cs]
  cc_t = [r[0] for r in cc]
  fixes = [r for r in gps if r[1]]
  fx_t = [r[0] for r in fixes]
  md_t = [r[0] for r in mapd]
  pose = doc["pose"]
  ts, ks, rows = [], [], []
  for t, yaw in pose:
    i = _nearest_idx(cs_t, t, 0.3)
    if i is None:
      continue
    c = cs[i]
    v = c[1]
    ts.append(t)
    ks.append(curvature(-yaw, v, params.v_min_ms))   # device z is opposite to vehicle yaw (ces_pnw)
    j = _nearest_idx(cc_t, t, 0.5)
    rows.append((t, v, bool(c[2]), bool(c[3]), None if j is None else bool(cc[j][1])))
  sm = smooth_signed(ts, ks, params.smooth_s)
  out = []
  for n, (t, v, drv, blk, la) in enumerate(rows):
    s = Sample(t=t + off_s, v=v, k=sm[n], k_raw=ks[n], drv=drv, blk=blk, lat_active=la)
    # position: linear interpolation between the bracketing FIXES, never extrapolated
    b = bisect.bisect_right(fx_t, t)
    if 0 < b < len(fixes):
      f0, f1 = fixes[b - 1], fixes[b]
      if f1[0] - f0[0] <= params.max_gps_gap_s:
        a = (t - f0[0]) / (f1[0] - f0[0]) if f1[0] > f0[0] else 0.0
        s.lat = f0[2] + a * (f1[2] - f0[2])
        s.lon = f0[3] + a * (f1[3] - f0[3])
        s.bacc = max(f0[5], f1[5])
        s.fix_ok = s.bacc <= params.bacc_max_deg
    m = _nearest_idx(md_t, t, 1.5)
    if m is not None:
      r = mapd[m]
      s.way, s.hwy, s.spl, s.mcs = r[1], r[2], r[3], r[4]
      # road = "<wayRef>|<roadName>": the ref ("I 5") survives where the way has its own name
      # ("Terwilliger Curves", "Marquam Bridge"), which is exactly where the corridor's curves are.
      s.road = f"{r[6] if len(r) > 6 else ''}|{r[5]}"
    out.append(s)
  return out


# ---------------------------------------------------------------------------------------------
# resampling by distance -> Points (one pass = one contiguous run)
# ---------------------------------------------------------------------------------------------

@dataclass
class Point:
  s: float                 # odometer along the pass, m
  t: float
  lat: float
  lon: float
  brg: float               # track bearing at this point
  v: float
  k: float | None          # |smoothed k| at this point (None = no speed / no sample)
  fix_ok: bool
  drv: bool
  lat_active: bool | None
  way: int
  hwy: str
  road: str
  spl: float
  mcs: float


def resample(samples: list[Sample], params: V2Params, max_dt_s: float = 1.0,
             tally: Counter | None = None) -> list[list[Point]]:
  """Samples -> passes of Points every `step_m` of odometer (integrated vEgo). A pass breaks on a time
  gap > max_dt_s, on a missing position, or on v < v_min. Bearing is the track bearing between the
  neighboring points (GPS bearing is noisier at the fix level and absent on a fix gap)."""
  passes: list[list[Point]] = []
  cur: list[Point] = []
  s = 0.0
  next_s = 0.0
  prev = None
  for smp in samples:
    ok = smp.lat is not None and smp.v >= params.v_min_ms
    if prev is not None and (smp.t - prev.t > max_dt_s or not ok):
      if tally is not None:
        tally["pass break: " + ("time gap" if smp.t - prev.t > max_dt_s else
                                "no position (GPS gap)" if smp.lat is None else "below v_min")] += 1
      if len(cur) >= 2:
        passes.append(cur)
      cur, prev = [], None
    if not ok:
      continue
    if prev is not None:
      s += 0.5 * (smp.v + prev.v) * (smp.t - prev.t)
    else:
      s, next_s = 0.0, 0.0
    if s >= next_s:
      cur.append(Point(s=s, t=smp.t, lat=smp.lat, lon=smp.lon, brg=0.0, v=smp.v,
                       k=None if smp.k is None else abs(smp.k), fix_ok=smp.fix_ok, drv=smp.drv,
                       lat_active=smp.lat_active, way=smp.way, hwy=smp.hwy, road=smp.road, spl=smp.spl,
                       mcs=smp.mcs))
      next_s += params.step_m
      if next_s <= s:              # a big jump in one tick: re-anchor rather than emit a burst
        next_s = s + params.step_m
    prev = smp
  if len(cur) >= 2:
    passes.append(cur)
  for p in passes:
    for i, pt in enumerate(p):
      a, b = p[max(i - 1, 0)], p[min(i + 1, len(p) - 1)]
      if a is not b and haversine_m(a.lat, a.lon, b.lat, b.lon) > 1.0:
        pt.brg = initial_bearing_deg(a.lat, a.lon, b.lat, b.lon)
      else:
        pt.brg = float("nan")
  return passes


def extent_end(pts: list[Point], i: int, fwd_m: float):
  """(lat, lon) of the pass at exactly s_i + fwd_m (linear between the bracketing points -- snapping to the
  nearest 25 m point would add up to 12.5 m of along-track error to a 15 m branch test), or None past the
  pass end."""
  target = pts[i].s + fwd_m
  if pts[-1].s < target or target <= pts[0].s:
    return None
  j = next(n for n in range(len(pts)) if pts[n].s >= target)
  a, b = pts[j - 1], pts[j]
  f = (target - a.s) / (b.s - a.s) if b.s > a.s else 0.0
  return a.lat + f * (b.lat - a.lat), a.lon + f * (b.lon - a.lon)


def branch_point(pts: list[Point], i: int, anchor: Anchor, fwd_m: float):
  """Where this pass is fwd_m beyond the ANCHOR (the anchor projected onto the pass at point i).

  THE ONE implementation the build and every query share. Measuring from the pass's own nearest point
  instead -- which can sit up to site_radius_m away along the road -- shifts the end point by as much:
  MEASURED, 6,428 of 22,398 anchors showed a phantom second branch at build time, and the replay's query
  side then found "no passes on this branch" on 13 episodes, when the two sides were computed differently."""
  pt = pts[i]
  along = 0.0
  d = haversine_m(pt.lat, pt.lon, anchor.lat, anchor.lon)
  if d > 0.5 and math.isfinite(pt.brg):
    along = d * math.cos(math.radians(initial_bearing_deg(pt.lat, pt.lon, anchor.lat, anchor.lon) - pt.brg))
  return extent_end(pts, i, fwd_m + along)


def extent_classes(pts: list[Point], i: int, back_m: float, fwd_m: float) -> tuple[bool, bool]:
  """(any ramp class, every point's class known) over the extent [s_i - back, s_i + fwd]."""
  s0 = pts[i].s
  hs = [p.hwy for p in pts if s0 - back_m <= p.s <= s0 + fwd_m]
  return any(h in RAMP_CLASSES for h in hs), not any(h in UNKNOWN_CLASSES for h in hs)


def extent_peak(pts: list[Point], i: int, back_m: float, fwd_m: float):
  """(peak |k| over [s_i - back, s_i + fwd], reason). Refuses (None, why) when the extent is not fully
  covered by the pass, a sample has no curvature, or any fix in it failed the GPS gate."""
  s0 = pts[i].s
  if s0 - pts[0].s < back_m:
    return None, "extent starts before the pass"
  if pts[-1].s - s0 < fwd_m:
    return None, "extent runs past the end of the pass"
  peak = 0.0
  for p in pts:
    if p.s < s0 - back_m or p.s > s0 + fwd_m:
      continue
    if not p.fix_ok:
      return None, "gps"
    if p.k is None:
      return None, "no k"
    peak = max(peak, p.k)
  return peak, "ok"


# ---------------------------------------------------------------------------------------------
# anchors: the road table's keys
# ---------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class PassObs:
  """One pass over one anchor. `k_ext` is the extent peak (the value a candidate lookup uses), `k_loc`
  the local value at the anchor (the value a path lookup uses)."""
  date: str                # PT date -- the leave-one-date-out key
  drive: str               # route name
  car: str
  t: float
  k_ext: float
  k_loc: float
  way: int
  hwy: str
  road: str
  spl: float
  mode: str                # "op" (openpilot steering, no driver input) | "drv" | "manual" | "unknown"
  d_m: float               # distance from the anchor
  end_lat: float = float("nan")    # where this pass was at the END of the extent (s + extent_fwd): the branch
  end_lon: float = float("nan")
  # R8 over the EXTENT, not just the anchor: MEASURED, both "real curves the DB would add" in the first
  # replay were exit ramps -- a mainline anchor just before the gore whose 175 m extent ran into the
  # motorwayLink curve. The anchor said "motorway"; the curvature belonged to the ramp.
  ext_ramp: bool = False           # any point in the extent on a ramp class
  # False = at least one point in the extent had no class. MEASURED bimodal on I-5 (2,558 of 3,023 extents
  # fully known, 461 fully unknown, 4 mixed): whole passes predate mapd v2.2.0's highwayClass. Such a pass
  # still measures curvature, but it is not CLASS evidence -- see row_verdict.
  ext_known: bool = True


@dataclass
class Anchor:
  lat: float
  lon: float
  brg: float
  obs: list[PassObs] = field(default_factory=list)


def pass_mode(p: Point) -> str:
  if p.lat_active is None:
    return "unknown"
  if not p.lat_active:
    return "manual"
  return "drv" if p.drv else "op"


class AnchorIndex:
  """Greedy single-anchor clustering with a spatial hash. The anchor is the FIRST point's position and
  never moves (v1's CurveRow rule: a drifting anchor walks down the road). Canonical build order is the
  caller's job (sort passes by time) so the table is reproducible."""

  def __init__(self, params: V2Params):
    self.p = params
    self.anchors: list[Anchor] = []
    self._cell = params.site_radius_m / 111000.0
    self._grid: dict[tuple[int, int], list[int]] = {}

  def _key(self, lat, lon):
    return (int(math.floor(lat / self._cell)), int(math.floor(lon / (self._cell / max(math.cos(math.radians(lat)), 1e-6)))))

  def nearest(self, lat: float, lon: float, brg: float):
    """(index, distance) of the nearest anchor within radius and heading tolerance, else (None, None)."""
    if not math.isfinite(brg):
      return None, None
    ky, kx = self._key(lat, lon)
    best, bd = None, None
    for dy in (-1, 0, 1):
      for dx in (-1, 0, 1):
        for ai in self._grid.get((ky + dy, kx + dx), ()):
          a = self.anchors[ai]
          if bearing_diff_deg(a.brg, brg) > self.p.heading_tol_deg:
            continue
          d = haversine_m(lat, lon, a.lat, a.lon)
          if d <= self.p.site_radius_m and (bd is None or d < bd):
            best, bd = ai, d
    return best, bd

  def add(self, lat, lon, brg) -> int:
    self.anchors.append(Anchor(lat, lon, brg))
    ai = len(self.anchors) - 1
    self._grid.setdefault(self._key(lat, lon), []).append(ai)
    return ai


def add_pass(index: AnchorIndex, pts: list[Point], *, date: str, drive: str, car: str, in_scope,
             tally: Counter) -> None:
  """Attach one pass to the table. Each anchor takes at most ONE observation per pass (the nearest
  point). New anchors are created only for in-scope points (`in_scope(point) -> bool`)."""
  p = index.p
  best: dict[int, tuple[float, int]] = {}
  for i, pt in enumerate(pts):
    if not math.isfinite(pt.brg):
      continue
    ai, d = index.nearest(pt.lat, pt.lon, pt.brg)
    if ai is None:
      if not in_scope(pt):
        continue
      ai, d = index.add(pt.lat, pt.lon, pt.brg), 0.0
    if ai not in best or d < best[ai][0]:
      best[ai] = (d, i)
  for ai, (d, i) in best.items():
    k_ext, why = extent_peak(pts, i, p.extent_back_m, p.extent_fwd_m)
    if k_ext is None or pts[i].k is None:
      tally[f"pass refused: {why if k_ext is None else 'no k'}"] += 1
      continue
    pt = pts[i]
    end = branch_point(pts, i, index.anchors[ai], p.extent_fwd_m)
    if end is None:                     # the projected end is past the pass: no branch, counted
      tally["pass admitted without a branch point (projected end past the pass)"] += 1
      end = (float("nan"), float("nan"))
    ramp, known = extent_classes(pts, i, p.extent_back_m, p.extent_fwd_m)
    index.anchors[ai].obs.append(PassObs(date=date, drive=drive, car=car, t=pt.t, k_ext=k_ext, k_loc=pt.k,
                                         way=pt.way, hwy=pt.hwy, road=pt.road, spl=pt.spl,
                                         mode=pass_mode(pt), d_m=d, end_lat=end[0], end_lon=end[1],
                                         ext_ramp=ramp, ext_known=known))
    tally["pass admitted"] += 1


def branches(anchor: Anchor, radius_m: float) -> list[tuple[float, float]]:
  """Greedy clusters of the anchor's pass END points (the first member is the center, as for anchors).
  One per road branch leaving the anchor within the extent: a plain road has one, a split two."""
  cents: list[tuple[float, float]] = []
  for o in sorted(anchor.obs, key=lambda o: (o.date, o.t)):
    if not math.isfinite(o.end_lat):
      continue
    if not any(haversine_m(o.end_lat, o.end_lon, c[0], c[1]) <= radius_m for c in cents):
      cents.append((o.end_lat, o.end_lon))
  return cents


# ---------------------------------------------------------------------------------------------
# a row's value and authority -- computed at query time so LODO is a filter
# ---------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class RowVerdict:
  granted: bool
  k: float | None          # the row curvature (percentile across dates) -- set even when refused
  n_dates: int
  n_passes: int
  reason: str
  hwy: str
  spl: float


def majority(xs, default=None):
  c = Counter(x for x in xs if x not in (None, "", 0))
  return c.most_common(1)[0][0] if c else default


def row_verdict(anchor: Anchor, params: V2Params, *, exclude_date: str | None = None,
                which: str = "k_ext", branch: tuple[float, float] | None = None) -> RowVerdict:
  """The owner's rule set, in order. `exclude_date` is leave-one-date-out. `branch` is where the QUERY's
  path is at the end of the extent; only passes that ended within `branch_radius_m` of it are pooled.
  With no branch (the table export) every pass is pooled and a split shows up as `dates disagree`."""
  obs = [o for o in anchor.obs if o.date != exclude_date]
  if branch is not None:
    obs = [o for o in obs if math.isfinite(o.end_lat) and
           haversine_m(o.end_lat, o.end_lon, branch[0], branch[1]) <= params.branch_radius_m]
    if not obs:
      return RowVerdict(False, None, 0, 0, "no passes on this branch", "", 0.0)
  if not obs:
    return RowVerdict(False, None, 0, 0, "no passes", "", 0.0)
  if params.way_check:
    w = majority([o.way for o in obs])
    if w is not None:
      obs = [o for o in obs if o.way in (0, w)]
  hwy = majority([o.hwy for o in obs if o.hwy not in UNKNOWN_CLASSES], "")
  spl = majority([round(o.spl, 1) for o in obs], 0.0)
  by_date: dict[str, list[float]] = {}
  cars: dict[str, set] = {}
  for o in obs:
    by_date.setdefault(o.date, []).append(getattr(o, which))
    cars.setdefault(o.date, set()).add(o.car)
  per_date = {d: percentile(v, 50.0) for d, v in by_date.items()}
  k = percentile(list(per_date.values()), params.percentile)
  n_dates, n_passes = len(per_date), len(obs)
  if n_dates < params.min_dates:
    return RowVerdict(False, k, n_dates, n_passes, f"only {n_dates} date(s)", hwy, spl)
  # Owner: Tesla passes may build rows but must never be the ONLY evidence. A pass whose car is not
  # known (no carParams in the route) is not evidence of the other car either.
  if not any(c != "unknown" and not c.startswith(TESLA_PREFIX) for cs in cars.values() for c in cs):
    return RowVerdict(False, k, n_dates, n_passes, "tesla-only evidence", hwy, spl)
  if hwy in UNKNOWN_CLASSES:
    return RowVerdict(False, k, n_dates, n_passes, "highway class unknown", hwy, spl)
  if hwy in RAMP_CLASSES:
    return RowVerdict(False, k, n_dates, n_passes, f"ramp ({hwy})", hwy, spl)
  # R8 over the extent (see PassObs.ext_ramp). ANY pass pooled into this row reaching a ramp refuses it:
  # the branch key already separates mainline from exit, so what is left here is a genuinely mixed site.
  # Class EVIDENCE comes only from passes whose whole extent was classified; a row with none is refused.
  if any(o.ext_ramp for o in obs):
    return RowVerdict(False, k, n_dates, n_passes, "extent reaches a ramp", hwy, spl)
  if not any(o.ext_known for o in obs):
    return RowVerdict(False, k, n_dates, n_passes, "no pass with a fully classified extent", hwy, spl)
  lo, hi = min(per_date.values()), max(per_date.values())
  if lo <= 0 or hi / lo > params.max_date_spread and hi - lo > params.spread_floor_k:
    return RowVerdict(False, k, n_dates, n_passes, f"dates disagree ({lo:.5f}..{hi:.5f})", hwy, spl)
  return RowVerdict(True, k, n_dates, n_passes, "ok", hwy, spl)
