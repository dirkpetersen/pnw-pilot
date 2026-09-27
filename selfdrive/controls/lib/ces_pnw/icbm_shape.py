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
Where they AGREE within x1.35 (either way) the candidate is priced at v = sqrt(A / mean(k_poly, k_mapd)), A = 2.5
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

MODES (curve.json {"lightning": {"icbm_shape": ...}}, owner answer 7: SHADOW FIRST):
  off     the stage does nothing; ICBM exactly as before.
  shadow  (default) every decision is computed and logged (shp* fields: what it WOULD do) and ICBM's real pipeline is
          handed NO pricing function at all, so its targets and taps are those of "off" by construction.
  live    ICBM's candidates are priced by the stage; the curve DB still overrides afterwards where it has a row.
Lightning only (PnwVehicle.icbm_shape); every other car reads "off".

Everything above ShapeStage is PURE. ShapeStage holds the latch / waySel hold / counters, reads mapd's A and builds
the telemetry. Nothing here imports tools/curvedb.
"""
from __future__ import annotations

import math

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.ces_pnw import curvedb_live as cl

MPH = 0.44704

# Agreement band, k_poly / k_mapd in [1/1.35, 1.35]. Owner 2026-09-24 answered x1.5; owner 2026-09-26 approved x1.35
# after the live replay's G7 failed at x1.5: OR-34 WB 09-22 09:43:58, polyline 1.34x the measured curvature and mapd
# 0.91x (ratio 1.46) priced 66.3 mph where the curve needed 70.2. Between 1.35 and 1.5 sits the polyline's over-read
# tail (s2.3 p90 1.55); the 09-22 09:22 Salem latent phantom (ratio 1.37, needed ~113 mph) sits there too.
SHAPE_BAND = 1.35
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


# ---------------------------------------------------------------------------------------------------------------------
# the stateful stage (curveshape2pnw 2/3)
# ---------------------------------------------------------------------------------------------------------------------
A_POLL_S = 5.0            # mapd's A is re-read this often (a Params file read, off the 4 Hz hot path)
A_MAX_AGE_S = 30.0        # a reading older than this is no reading
DECISION_FRESH_S = 1.0    # a record shows the latest decision only if ICBM made one this recently
ERR_LOG_S = 30.0          # Rule 2: throttle for the failure log (the stage runs at ~4 Hz)

# Every key tele() emits, pinned by the tests (a key added here and not emitted, or vice versa, is a silently-null
# column -- the VTSCStatus trap).
TELE_KEYS = ("shpOn", "shpWhy", "shpDir", "shpSrc", "shpKP", "shpKM", "shpK", "shpV", "shpLeg", "shpD", "shpBase",
             "shpT", "shpA", "shpStable", "shpWayHold", "shpN", "shpNA", "shpNL", "shpNR", "shpErr")


def _r(x, nd):
  return None if x is None else round(float(x), nd)


class ShapeStage:
  """One per controller. tick() is called once per ICBM decision (~4 Hz) with a `core(eff_fn)` callback that runs
  ICBM's candidate + penalty core (icbm_far_map_candidate -> icbm_curve_target -> icbm_penalise) and returns
  (target, src, (raw, dist) of the candidate to describe, or None). The stage runs it WITH its pricer (shpT) and,
  when anything was priced, WITHOUT (shpBase), so shpDir is the stage's effect on that core alone -- before the start
  gates, the passed-point gate, the curve DB, the posted-limit floor and lead pacing, which are unchanged by it.
  Returns the pricer for the REAL pipeline in live mode, else None (shadow and off hand ICBM nothing)."""

  def __init__(self, mode, why, a_tgt, a_tgt_hi, read_params=None):
    self.mode = mode if mode in MODES else "off"
    self.why_cfg = why
    self.a_tgt, self.a_tgt_hi = a_tgt, a_tgt_hi
    self._read_params = read_params
    self.latch = ShapeLatch()
    self._a = (None, None, "not read yet")
    self._a_t = -1e9                  # monotonic time of the last A read attempt
    self._a_ok_t = -1e9               # ... and of the last GOOD read
    self._a_logged = None
    self._way_cur_t = -1e9
    self.n = self.n_add = self.n_lower = self.n_raise = 0
    self.n_err = 0
    self._err_t = -1e9
    self._last: dict = {}
    self._last_t = -1e9

  @property
  def enabled(self) -> bool:
    return self.mode in ("shadow", "live")

  # -- mapd's A ---------------------------------------------------------------------------------------------------
  def poll_a(self, now) -> None:
    if now - self._a_t < A_POLL_S:
      return
    self._a_t = now
    try:
      raw_s, raw_p = (self._read_params or cl.READ_PARAMS[0])()
      a, src, why = cl.parse_a(raw_s, raw_p)
    except Exception as e:           # a failed read is an unreadable A (logged below), never a default
      a, src, why = None, None, f"read failed ({type(e).__name__})"
    if a is not None:
      self._a_ok_t = now
      self._a = (a, src, why)
    elif now - self._a_ok_t > A_MAX_AGE_S:
      self._a = (None, src, why)
    key = (a, src, why)
    if key != self._a_logged:        # change-only, both directions
      if a is None:
        cloudlog.error(f"icbm_shape: mapd's lateral target A is UNREADABLE ({why}) -- the measured-shape stage " +
                       "prices nothing until it reads")
      else:
        cloudlog.event("icbm_shape_a", a_lat=a, source=src)
      self._a_logged = key

  def a_mapd(self, now):
    a = self._a[0]
    return a if a is not None and now - self._a_ok_t <= A_MAX_AGE_S else None

  # -- waySel flicker hold (the curve DB's WAYSEL_HOLD_S, tracked independently so neither disturbs the other) -----
  def way_held(self, way_sel, now):
    if way_sel == "current":
      self._way_cur_t = now
      return "current", False
    hold = now - self._way_cur_t <= cl.WAYSEL_HOLD_S
    return ("current" if hold else way_sel), hold

  # -- one decision ----------------------------------------------------------------------------------------------
  def _record(self, now, **kw) -> None:
    rec = dict.fromkeys(TELE_KEYS)
    rec.update(shpOn=self.mode, **kw)
    self._last, self._last_t = rec, now

  def tick(self, now, *, gps_state, way_sel, hwy, reading, reading_t, v_ego, posted, v_set, scale_fn, map_scale,
           core):
    if not self.enabled:
      self._record(now, shpWhy=self.why_cfg)
      return None
    self.poll_a(now)
    way, hold = self.way_held(way_sel, now)
    stable = self.latch.update(reading_t, *reading, v_ego)
    a = self.a_mapd(now)
    base_rec = {"shpKP": _r(reading[0], 6), "shpA": a, "shpStable": stable, "shpWayHold": hold}
    why = tick_gate(way, hwy, gps_state, cl.RAMP_CLASSES, cl.UNKNOWN_CLASSES) or (None if a is not None else "noA")
    if why is not None:
      self._record(now, shpWhy=why, shpDir="none", **base_rec)
      return None
    pricer = ShapePricer(scale_fn, map_scale, a, reading, stable, posted, v_set, self.a_tgt, self.a_tgt_hi)
    t_shape, src, cand = core(pricer)
    t_base = core(None)[0] if pricer.any_priced else t_shape
    if t_base is None and t_shape is None:
      dr = "none"
    elif t_base is None:
      dr = "add"
    elif t_shape is None or t_shape > t_base + ACT_EPS_MS:
      dr = "raise"
    elif t_shape < t_base - ACT_EPS_MS:
      dr = "lower"
    else:
      dr = "none"
    self.n += 1
    if dr == "add":
      self.n_add += 1
    elif dr == "lower":
      self.n_lower += 1
    elif dr == "raise":
      self.n_raise += 1
    rec = dict(base_rec, shpWhy="noCand", shpDir=dr, shpBase=_r(t_base, 2), shpT=_r(t_shape, 2))
    if cand is not None:
      v, cwhy, _cdir, k, km, leg = pricer.decide(*cand)
      rec.update(shpWhy=cwhy, shpSrc=src if src in ("map", "far") else "map", shpKM=_r(km, 6), shpK=_r(k, 6),
                 shpV=_r(v, 2), shpLeg=_r(leg, 2), shpD=_r(cand[1], 0))
    if pricer.err is not None:
      self.fail(now, pricer.err, rec)
      return None                      # a defective pricer is not handed to ICBM, even in live mode
    self._record(now, **rec)
    return pricer if self.mode == "live" else None

  def fail(self, now, err, rec=None) -> None:
    """Rule 2: the stage failed this tick. ICBM runs WITHOUT it (the caller passes no pricer); the record says
    shpOn=err with the exception type, and the log says so (throttled, with a count)."""
    self.n_err += 1
    name = err if isinstance(err, str) else type(err).__name__
    r = dict(rec or {})
    r.update(shpErr=name)
    self._record(now, **r)
    self._last["shpOn"] = "err"
    if now - self._err_t > ERR_LOG_S:
      self._err_t = now
      msg = (f"icbm_shape: measured-shape stage FAILED ({name}) -- ICBM runs WITHOUT it this tick " +
             f"({self.n_err} failure(s) so far)")
      if isinstance(err, BaseException):
        cloudlog.exception(msg)
      else:
        cloudlog.error(msg)

  def tele(self, now) -> dict:
    out = dict.fromkeys(TELE_KEYS)
    if self._last and now - self._last_t <= DECISION_FRESH_S:
      out.update(self._last)
    else:
      out.update(shpOn=self.mode, shpWhy=self.why_cfg if not self.enabled else "idle")
    out.update(shpN=self.n, shpNA=self.n_add, shpNL=self.n_lower, shpNR=self.n_raise)
    return out
