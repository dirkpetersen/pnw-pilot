"""curveshape2pnw -- price an ICBM map candidate from the curve's MEASURED SHAPE (docs: ~/gh/comma/docs/
CURVE-MEASURED-SHAPE-DESIGN.md; owner answers 2026-09-24 "yes, defaults").

THE PROBLEM. A map candidate's speed is mapd's rating x icbm_map_eff_scale (1.10 -> 1.35 between 50 and 60 mph raw)
x map_scale 0.92: 1.012x on tight curves but 1.242x on sweepers, an effective ~3.1 m/s^2 (mapd rates at A = 2). The
OR-34 11:18 curve (mapd 68.0, about right) became 84.5, above the 78 set, and was discarded; the truck took it at
2.92 m/s^2 with the PSCM at limit level 2. Trusting mapd's number alone at 2.5 (Part C) added a phantom on I-5 45.72,
where mapd is wrong.

THE RULE (v1). Two readings of the same curve, both from mapd's own OSM nodes:
  k_poly  the map polyline's horizon-peak curvature (icbmK, 3-point circle fits, legs gated to [25, 300] m);
  k_mapd  mapd's own curvature, recovered from its rating: k_mapd = A_mapd / v_mapd^2 (A_mapd read live).
Where they AGREE within x1.5 (either way) the candidate is priced at v = sqrt(A / mean(k_poly, k_mapd)), A = 2.5
m/s^2. Where they disagree NOTHING changes -- the reading that disagrees is the one that has been wrong in the corpus
(s2.2: I-5 45.72 is mapSharper, I-5 46.64 / 44.85 are polySharper), and neither may veto the other (s2.3). Measured on
210 mainline curves: the agreeing mean is within +-25 % of the truck's own curvature on 80 % (86 % for k >= 0.002).

CONFIDENCE, all required, else the candidate keeps TODAY's price and the reason is logged (shpWhy):
  mapd's rating >= 50 mph (owner answer 3: tight curves stay as today / curve DB)        rawLow
  the polyline measured something (icbmKN >= 3, k > 0)                                   sparse
  that peak lies ahead of the truck (icbmKAhead)                                         notAhead
  it is >= 60 m away (a peak under the truck is the curve being driven, not approached) near
  it is within 150 m of THIS candidate (it describes the same curve)                     farApart
  the same peak on 2 consecutive refreshes (ShapeLatch)                                  unstable
  mapd's A readable                                                                       noA
plus the per-tick gates in tick_gate (waySel, road class, GPS).

RAISES (v above today's price) are capped at +8 mph, posted + 10 mph (no posted limit -> no raise), and the set.
A lowering is exact. The Lightning's left/descent extras still subtract downstream (owner answer 5); the base hump
is given back (the shape price becomes the map floor, as Part B does for vision).

This module is PURE except for ShapeStage (commit 2), which holds the latch / waySel hold / counters and reads A.
Nothing here imports tools/curvedb.
"""
from __future__ import annotations

import math

MPH = 0.44704

SHAPE_BAND = 1.5                  # owner answer 2: agreement band, k_poly / k_mapd in [1/1.5, 1.5]
SHAPE_RAW_MIN_MS = 50.0 * MPH     # owner answer 3: v1 only for mapd ratings >= 50 mph
SHAPE_KN_MIN = 3                  # icbmKN: triplets that passed the spacing gate
SHAPE_KD_MIN_M = 60.0             # the peak must be at least this far ahead
SHAPE_KD_TOL_M = 150.0            # |icbmKD - candidate distance| <= this: the same curve
LATCH_DK_REL = 0.35               # consecutive refreshes: |dk| / k below this ...
LATCH_DKD_M = 40.0                # ... and the peak moved closer by v*dt within this
LATCH_MAX_DT_S = 2.5              # refreshes are ~1 s apart; further apart is not "consecutive"
RAISE_CAP_MS = 8.0 * MPH          # a raise never exceeds today's price + 8 mph ...
POSTED_MARGIN_MS = 10.0 * MPH     # ... nor posted + 10 mph (no posted limit -> no raise)
HI_V_MS = 70.0 * MPH              # at or above this the high-speed lateral target applies (owner answer 4)
ACT_EPS_MS = 0.05                 # smaller than this is not a change

MODES = ("off", "shadow", "live")


def _num(x):
  """A finite float, else None (bools are not numbers here)."""
  if x is None or isinstance(x, bool):
    return None
  try:
    x = float(x)
  except (TypeError, ValueError):
    return None
  return x if math.isfinite(x) else None


class ShapeLatch:
  """The 2-refresh persistence rule. The polyline is re-measured once per _read_map refresh (~1 Hz), stamped with the
  refresh time; ICBM ticks at ~4 Hz and calls update() every tick, so a repeated stamp is the SAME reading and changes
  nothing. A reading is STABLE when the previous refresh's reading was valid, came <= LATCH_MAX_DT_S before, had
  nearly the same curvature (|dk|/k < LATCH_DK_REL) and sat further away by what the truck drove since
  (|(kd_prev - kd) - v*dt| <= LATCH_DKD_M). An invalid reading (no stamp, KN < SHAPE_KN_MIN, k <= 0, behind) resets
  the latch. Pure state machine; never raises on bad input."""

  def __init__(self):
    self.reset()

  def reset(self) -> None:
    self.t = None                  # stamp of the last reading seen
    self.prev = None               # (t, k, kd) of the last VALID reading
    self.stable = False

  def update(self, t, k, kd, kn, ahead, v_ego) -> bool:
    t, k, kd, v = _num(t), _num(k), _num(kd), _num(v_ego)
    kn = _num(kn)
    if t is None:
      self.reset()
      return False
    if t == self.t:
      return self.stable
    self.t = t
    valid = k is not None and k > 0.0 and kd is not None and kn is not None and kn >= SHAPE_KN_MIN and ahead is True
    if not valid:
      self.prev, self.stable = None, False
      return False
    stable = False
    if self.prev is not None and v is not None:
      pt, pk, pkd = self.prev
      dt = t - pt
      stable = (0.0 < dt <= LATCH_MAX_DT_S and abs(k - pk) / pk < LATCH_DK_REL
                and abs((pkd - kd) - max(v, 0.0) * dt) <= LATCH_DKD_M)
    self.prev, self.stable = (t, k, kd), stable
    return stable


def tick_gate(way_sel, hwy, gps_state, ramp_classes, unknown_classes) -> str | None:
  """Why the stage may not price ANY candidate this tick, or None. The curve DB's own exclusions: mapd not on the
  current way (after the caller's 2 s flicker hold), a ramp or unknown road class, no fresh GPS fix."""
  if gps_state not in ("raw", "proj"):
    return "gps"
  if way_sel != "current":
    return "waySel"
  if hwy is None or hwy in unknown_classes or hwy in ramp_classes:
    return "class"
  return None


def shape_price(raw_v, cand_d, a_mapd, reading, stable, legacy_eff, posted, v_set, a_tgt, a_tgt_hi):
  """ONE map/far candidate -> (v, why, dir, k_used, k_mapd).

  raw_v       mapd's rating for the candidate (m/s)          cand_d     its distance (m)
  a_mapd      mapd's lateral target (m/s^2), None = unreadable
  reading     (k_poly, kd, kn, ahead) -- icbmK / icbmKD / icbmKN / icbmKAhead
  stable      ShapeLatch.stable for that reading
  legacy_eff  TODAY's price, icbm_map_eff_scale(raw) * raw * map_scale (m/s)
  posted      the posted limit (m/s; 0 / None = none)      v_set  the set / episode ceiling (m/s)
  a_tgt       the lateral target (2.5); a_tgt_hi applies when the price is >= 70 mph (owner answer 4)

  v is the shape price (m/s) or None = keep legacy_eff. dir: "lower" / "raise" / "held" (a raise the caps withheld
  entirely) / "none". why: "ok", "noPosted" (a raise withheld for want of a posted limit), or the confidence rule that
  refused (module docstring). Pure; never raises."""
  raw, d, leg = _num(raw_v), _num(cand_d), _num(legacy_eff)
  if raw is None or raw <= 0.0 or d is None or leg is None or leg <= 0.0:
    return None, "badInput", "none", None, None
  if raw < SHAPE_RAW_MIN_MS:
    return None, "rawLow", "none", None, None
  a = _num(a_mapd)
  if a is None or a <= 0.0:
    return None, "noA", "none", None, None
  km = a / (raw * raw)
  try:
    kp, kd, kn, ahead = reading
  except (TypeError, ValueError):
    return None, "badInput", "none", None, km
  kp, kd, kn = _num(kp), _num(kd), _num(kn)
  if kp is None or kp <= 0.0 or kn is None or kn < SHAPE_KN_MIN or kd is None:
    return None, "sparse", "none", None, km
  if ahead is not True:
    return None, "notAhead", "none", None, km
  if kd < SHAPE_KD_MIN_M:
    return None, "near", "none", None, km
  if abs(kd - d) > SHAPE_KD_TOL_M:
    return None, "farApart", "none", None, km
  if stable is not True:
    return None, "unstable", "none", None, km
  ratio = kp / km
  if ratio < 1.0 / SHAPE_BAND:
    return None, "mapSharper", "none", None, km
  if ratio > SHAPE_BAND:
    return None, "polySharper", "none", None, km
  at, at_hi = _num(a_tgt), _num(a_tgt_hi)
  if at is None or at <= 0.0:
    return None, "badInput", "none", None, km
  k = 0.5 * (kp + km)                                   # owner answer 1: the MEAN of the two readings
  v = math.sqrt(at / k)
  if v >= HI_V_MS and at_hi is not None and at_hi > 0.0:
    v = math.sqrt(at_hi / k)
  if v > leg + ACT_EPS_MS:
    p = _num(posted)
    if p is None or p <= 0.0:
      return leg, "noPosted", "held", k, km
    cap = min(leg + RAISE_CAP_MS, p + POSTED_MARGIN_MS)
    vs = _num(v_set)
    if vs is not None and vs > 0.0:
      cap = min(cap, vs)
    r = max(leg, min(v, cap))
    return r, "ok", ("raise" if r > leg + ACT_EPS_MS else "held"), k, km
  if v < leg - ACT_EPS_MS:
    return v, "ok", "lower", k, km
  return leg, "ok", "none", k, km


class ShapePricer:
  """The per-tick pricing function handed to ICBM's candidate functions as `eff_fn(tv, dist)`. Returns the shape price
  where shape_price gives one, else TODAY's price computed exactly as before: scale_fn(tv) * tv * map_scale, the same
  multiplication order, so an unpriced candidate is bit-identical. Decisions are cached per (tv, dist): the far scan,
  the passed-point gate and the curve DB all ask about the same points.

  NEVER RAISES. A defect in the pricing costs that candidate its shape price, never the ICBM publish: the legacy
  product is returned and the failure is recorded in `err` (the caller logs it and reports shpOn=err)."""

  def __init__(self, scale_fn, map_scale, a_mapd, reading, stable, posted, v_set, a_tgt, a_tgt_hi):
    self._scale_fn, self._map_scale = scale_fn, map_scale
    self._ctx = (a_mapd, reading, stable, posted, v_set, a_tgt, a_tgt_hi)
    self._cache: dict = {}
    self.err = None

  def legacy(self, tv):
    return self._scale_fn(tv) * tv * self._map_scale

  def decide(self, tv, dist):
    """(v, why, dir, k_used, k_mapd, legacy) for one candidate, cached. Raises only if the legacy product itself
    raises -- the same exposure as before this stage existed."""
    key = (tv, dist)
    got = self._cache.get(key)
    if got is not None:
      return got
    leg = self.legacy(tv)
    try:
      a, reading, stable, posted, v_set, at, at_hi = self._ctx
      v, why, dr, k, km = shape_price(tv, dist, a, reading, stable, leg, posted, v_set, at, at_hi)
    except Exception as e:           # a defect here must cost the shape price, not the candidate
      self.err = type(e).__name__
      v, why, dr, k, km = None, "err", "none", None, None
    got = (v, why, dr, k, km, leg)
    self._cache[key] = got
    return got

  def __call__(self, tv, dist):
    d = self.decide(tv, dist)
    return d[0] if d[0] is not None else d[5]

  def priced(self, tv, dist):
    """The shape price for this candidate, or None when it keeps today's price. Never raises."""
    try:
      return self.decide(tv, dist)[0]
    except Exception as e:
      self.err = type(e).__name__
      return None

  @property
  def any_priced(self) -> bool:
    return any(d[0] is not None for d in self._cache.values())
