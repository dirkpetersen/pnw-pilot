"""
CES — Conditional Experimental Switching (xnor)  ⚠️ NOT WIRED / NOT DEPLOYED

Decides per-cycle whether the longitudinal planner should run Chill (ACC/MPC) or Experimental
(blended e2e), keeping the car in Chill for steady cruising and flipping to Experimental only for
curves, stop lights/signs, low-speed/complex (incl. city), and closing on a slow/stopped lead.

Design + decisions: see /home/dp/gh/comma/CES.md. Key properties:
  - Default Chill; ANY condition -> Experimental; return to Chill only when ALL clear + sustained +
    min-dwell (hysteresis on every threshold).
  - Per-condition FirstOrderFilter debounce (THRESHOLD ~ 1 s) — no flapping.
  - Tesla-only, longitudinal-only (Experimental does NOT change steering), default OFF.
  - 3-state top-right button override: CES / forced-Chill / forced-Experimental.

SAFETY: this module is PURE DECISION LOGIC. It does not command the car. It must be wired into the
effective-experimental computation (selfdrived) only after review + on-road verification. It never
touches panda safety. The decision core (`decide_active`) takes primitives and is unit-tested.
"""
import json
import math
import os
import time

from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle
# curveslow-lightning: ICBM's vision apex uses the SAME lateral-accel target as the VTSC vision path
# (v_safe = v_ego*sqrt(A_LAT/|lat|)) so the two subsystems agree on what a camera-seen curve "means".
# descentcurve2pnw: MAP_SOURCE_HORIZON_M is mapd's hard 500 m path cap — ICBM's full-horizon map scan
# uses the same constant family as VTSC/MTSC so both scan exactly what mapd publishes.
from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_constants import (A_LAT_TARGET as VTSC_A_LAT,
                                                                      MAP_SOURCE_HORIZON_M)

# Persistent, append-only "each adoption" trail. Lives OUTSIDE /data/openpilot so it survives the
# boot overlay-swap AND swaglog rotation (a long drive rotates swaglog and would lose early events).
# One JSON line per CES mode transition, with GPS so we can map where each adoption happened.
CES_EVENT_LOG = "/data/dirk/ces_events.jsonl"
CES_EVENT_LOG_MAX_BYTES = 20 * 1024 * 1024   # rotate at 20 MB; one .1 generation kept -> ~40 MB cap
                                             # (was unbounded — 43 MB and growing on a 90%-full disk)


def vision_curve_lat_accel(orientation_rate_z, velocity_x, timebase, v_ego):
  """FrogPilot-style vision curve detector: predicted lateral accel + time-to-curve over the model
  horizon. Returns (predicted_lat_accel m/s^2, time_to_curve s). Pure; lists must be equal length."""
  if not orientation_rate_z or not velocity_x or not timebase:
    return 0.0, 1.0
  n = min(len(orientation_rate_z), len(velocity_x), len(timebase))
  best_acc, best_t, best_abs = 0.0, 1.0, -1.0
  for i in range(n):
    lat = orientation_rate_z[i] * velocity_x[i]   # yaw_rate * speed = lateral accel
    if abs(lat) > best_abs:
      best_abs, best_acc, best_t = abs(lat), lat, timebase[i]
  return best_acc, max(best_t, 1.0)


# icbm2pnw: comfort decel used to decide WHEN a curve starts binding the stock-ACC set speed. The
# stock ACC does the actual braking to the new set point; this only times the hand-off (gentle,
# truck-profile). Reduce-only: targets never exceed the driver's own latched set speed.
ICBM_A_DECEL = 0.8          # m/s^2 comfort approach decel
ICBM_MARGIN_M = 30.0        # start a little early — button steps + ACC response add latency
ICBM_MIN_DROP_MS = 1.0      # ignore caps within ~2 mph of the set speed (not worth taps)
# curveslow-lightning: floor on |predicted lateral accel| before a vision curve is even a candidate
# (div-by-zero guard + straightaway rejector). Reuses the CES vision-enter threshold so a curve the
# camera calls "not a curve" for the CES trip is also not one for ICBM.
ICBM_VISION_ENTER = C.CURVE_LAT_ACCEL_ENTER   # m/s^2 (1.9)
ICBM_VISION_EPS = 0.05                          # m/s^2 floor inside the sqrt (never divide by ~0)
# descentcurve2pnw: at 90 mph the drive's map candidate came from a 10 s time window (~400 m) while
# shedding 25 mph at the 0.8 comfort decel needs ~510 m — the curve became visible already-late.
# The full-horizon scan (ICBM_MAP_HORIZON_M = mapd's 500 m cap) closes that gap. For very LARGE
# drops the assumed approach decel firms from ICBM_A_DECEL toward the Lightning's tunable
# icbm_firm_decel (~1.4; stock ACC does the actual braking — this only shapes the tap-start
# envelope, and the ramp starts at 30 mph of drop so the field case, a 25 mph drop, still uses the
# full comfort envelope and binds at first sight).
ICBM_MAP_HORIZON_M = MAP_SOURCE_HORIZON_M   # m; scan the FULL published map path
ICBM_FIRM_DROP_LO = 13.4    # m/s (~30 mph) required drop where the approach decel starts firming
ICBM_FIRM_DROP_HI = 26.8    # m/s (~60 mph) required drop where it reaches the firm ceiling


def icbm_approach_decel(v_ego, apex, firm_decel=0.0, a_base=ICBM_A_DECEL,
                        drop_lo=ICBM_FIRM_DROP_LO, drop_hi=ICBM_FIRM_DROP_HI):
  """descentcurve2pnw: assumed approach decel for the binding envelope — the base comfort decel for
  normal drops, ramping linearly toward `firm_decel` for very large (v_ego - apex) drops.
  firm_decel 0 / None / <= a_base (the non-Lightning default) -> a_base exactly (byte-identical to
  the pre-descentcurve behavior). Monotonic non-decreasing in the drop; always within
  [a_base, firm_decel]. Pure."""
  if not firm_decel or firm_decel <= a_base:
    return a_base
  drop = max(float(v_ego) - float(apex), 0.0)
  if drop <= drop_lo:
    return a_base
  if drop >= drop_hi:
    return firm_decel
  return a_base + (firm_decel - a_base) * (drop - drop_lo) / (drop_hi - drop_lo)


def icbm_vision_apex(v_ego, curve_lat_accel_vision, time_to_curve, a_lat=VTSC_A_LAT):
  """curveslow-lightning: turn the model's predicted lateral accel into a vision curve candidate for
  ICBM, mirroring the VTSC vision path. From lat_accel = v^2 * kappa the safe speed holding a_lat is
  vis_apex = v_ego*sqrt(a_lat/|lat|); distance = time_to_curve*v_ego. Returns (apex_v m/s, dist m), or
  (0.0, inf) when it is NOT a candidate (too straight / no speed). Pure; never raises."""
  try:
    lat = abs(float(curve_lat_accel_vision))
    v = float(v_ego)
    if v <= 0.0 or lat <= ICBM_VISION_ENTER:
      return 0.0, float('inf')
    apex = v * math.sqrt(a_lat / max(lat, ICBM_VISION_EPS))
    dist = max(float(time_to_curve) * v, 0.0)
    return max(apex, 0.0), dist
  except (TypeError, ValueError):
    return 0.0, float('inf')


def _icbm_binding_apex(v_ego, ref, apex, dist, a_decel=ICBM_A_DECEL):
  """One curve candidate -> the apex speed it commands IF it both (a) is reduce-only (apex sits
  ICBM_MIN_DROP_MS below the ceiling `ref`) and (b) has entered the brake envelope at `a_decel`
  (default = the comfort decel; descentcurve2pnw passes the drop-scaled icbm_approach_decel).
  Else None. Identical envelope math to the original map-only path. Pure."""
  if apex is None or dist == float('inf') or apex >= ref - ICBM_MIN_DROP_MS:
    return None
  brake_dist = max(v_ego * v_ego - apex * apex, 0.0) / (2.0 * a_decel) + ICBM_MARGIN_M
  if dist <= brake_dist:
    return max(apex, 0.0)
  return None


def icbm_far_map_candidate(points, cur_lat, cur_lon, v_ego, ref, scale_fn, map_scale=1.0,
                           firm_decel=0.0, horizon_m=ICBM_MAP_HORIZON_M):
  """descentcurve2pnw: FULL-horizon map candidate for ICBM. Scans every mapd path point out to
  `horizon_m` (mapd's 500 m publish cap — vs the old 10 s time window, ~400 m at 90 mph) and
  returns (apex_eff m/s, dist m) of the MOST-BINDING candidate — the one whose decel-limited brake
  cap is lowest right now (same selection idea as VTSC's most_binding_map_curve, so a far sharp
  curve can't shadow a nearer curve that needs action first). Candidates apply the shared tiered
  scale (scale_fn) AND the Lightning map-speed discount `map_scale` (<= 1.0, from PnwVehicle — OSM
  curve speeds are calibrated for stronger-steering cars) BEFORE the reduce-only test, so selection
  and use can't disagree. The per-point envelope uses the same drop-scaled icbm_approach_decel the
  downstream binding test uses. Returns (0.0, inf) if none. The actual DEC-only binding decision
  stays in icbm_curve_target/_icbm_binding_apex. NaN-guarded like upcoming_curve. Pure."""
  if not points or cur_lat is None or cur_lon is None or ref <= 0.0:
    return 0.0, float('inf')
  best_cap = float('inf')
  best_v, best_d = 0.0, float('inf')
  for p in points:
    try:
      d = _haversine_m(cur_lat, cur_lon, p["latitude"], p["longitude"])
      tv = float(p["velocity"])
    except (KeyError, TypeError, ValueError):
      continue
    if tv != tv or d != d:                      # NaN guard (mapd emits non-finite velocities, seen live)
      continue
    if tv <= 0.0 or not (0.0 < d <= horizon_m):
      continue
    eff = scale_fn(tv) * tv * map_scale
    if eff >= ref - ICBM_MIN_DROP_MS:
      continue                                  # reduce-only: not meaningfully below the ceiling
    a = icbm_approach_decel(v_ego, eff, firm_decel)
    cap = math.sqrt(eff * eff + 2.0 * a * max(d - ICBM_MARGIN_M, 0.0))   # decel envelope from here
    if cap < best_cap:
      best_cap, best_v, best_d = cap, eff, d
  return best_v, best_d


def icbm_curve_target(v_ego, v_set, map_v, map_dist, ceiling, scale_fn,
                      vis_v=0.0, vis_dist=float('inf'),
                      map_scale=1.0, firm_decel=0.0,
                      far_v=0.0, far_dist=float('inf')):
  """Pure ICBM brain step (unit-tested). Returns (target_ms or None, new_ceiling or None, src or None)
  where src is "map" / "vis" / "far". Considers a MAP candidate (pfeiferj target, tiered-scaled like
  VTSC/MTSC), a VISION candidate (from icbm_vision_apex), and descentcurve2pnw's FAR-MAP candidate
  (full-horizon scan, already effective/scaled — from icbm_far_map_candidate) and returns the BINDING
  one with the LOWEST target (most slowing). Same DEC-ONLY, reduce-only, ceiling-latch semantics as
  before — with the descentcurve defaults (map_scale=1.0, firm_decel=0.0, no far candidate) this is
  byte-equivalent to the original.

  descentcurve2pnw knobs (all neutral by default, Lightning-supplied via PnwVehicle):
    map_scale  <= 1.0 discount on the map candidate's suggested speed (OSM speeds too generous for
               the Lightning's weak EPS) — applied BEFORE the reduce-only/binding tests;
    firm_decel assumed approach decel for very LARGE drops (icbm_approach_decel ramp; stock ACC does
               the actual braking — this only shapes the tap-start envelope).

  ceiling = the driver's own set speed latched when a cap first engages (None when uncapped); while
  capped, v_set follows the button-lowered stock set, so the latched ceiling is the only memory of
  what to restore to. Reduce-only: target is never above the ceiling."""
  if v_set <= 0:
    return None, None, None                # no valid set speed -> hands off
  ref = ceiling if ceiling is not None else v_set
  best_apex, best_src = None, None
  # MAP candidate — same tiered scaling as VTSC/MTSC, plus the Lightning map-speed discount.
  # Deliberately KEEPS the base comfort-decel envelope (NOT the drop-scaled firm decel): a firmer
  # assumed decel SHRINKS the envelope, i.e. starts taps LATER — inside the near window that would
  # be an under-brake regression vs pre-descentcurve behavior (Gemini review catch 2026-07-11).
  # The firm decel applies only to the FAR candidate below, where pre-diff there was NO braking at
  # all, so it can only ever ADD slowing.
  if map_v and map_v > 0 and map_dist != float('inf'):
    eff = scale_fn(map_v) * map_v * map_scale
    a = _icbm_binding_apex(v_ego, ref, eff, map_dist)
    if a is not None:
      best_apex, best_src = a, "map"
  # VISION candidate — already a safe speed (icbm_vision_apex). Lowest binding target wins (most slowing).
  if vis_v and vis_v > 0 and vis_dist != float('inf'):
    a = _icbm_binding_apex(v_ego, ref, vis_v, vis_dist)
    if a is not None and (best_apex is None or a < best_apex):
      best_apex, best_src = a, "vis"
  # FAR-MAP candidate — full-horizon scan (already effective: tiered scale + map_scale applied inside
  # icbm_far_map_candidate). Same binding envelope; catches curves beyond the 10 s window at speed.
  if far_v and far_v > 0 and far_dist != float('inf'):
    a = _icbm_binding_apex(v_ego, ref, far_v, far_dist, icbm_approach_decel(v_ego, far_v, firm_decel))
    if a is not None and (best_apex is None or a < best_apex):
      best_apex, best_src = a, "far"
  if best_apex is not None:
    new_ceiling = ceiling if ceiling is not None else v_set
    return best_apex, new_ceiling, best_src
  # curve cleared (or none): go silent and unlatch immediately. DEC-ONLY design (Gemini-hardened
  # 2026-07-11): there is no restore path — the executor can only lower the set speed, and the
  # driver restores it themselves. This kills the restore-vs-driver-intent fight entirely.
  return None, None, None


def _haversine_m(lat1, lon1, lat2, lon2) -> float:
  """Great-circle distance in metres (pure)."""
  import math
  r = 6371000.0
  p1, p2 = math.radians(lat1), math.radians(lat2)
  dp = math.radians(lat2 - lat1); dl = math.radians(lon2 - lon1)
  a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
  return 2 * r * math.asin(min(1.0, a ** 0.5))


def map_turn_direction(points, cur_lat, cur_lon, target_dist, tol_m: float = 60.0) -> int:
  """icbmalign2pnw: turn DIRECTION of the map path at ~`target_dist` metres ahead: +1 = LEFT,
  -1 = right, 0 = straight/unknown. Finds the path point whose distance from the current position
  is closest to `target_dist` (the ICBM candidate's recorded distance — same haversine metric, so
  the match is exact for map/far candidates), then sums the signed turn (2-D cross product of
  successive segment vectors in a local east/north frame) over a small window around it.
  Sign: with x = east, y = north, cross(v0, v1) > 0 = counterclockwise viewed from above = LEFT —
  the same left-positive convention as vtsc_pnw.apex_turn_direction / openpilot steering.
  Ambiguous / too few points / no match within tol_m -> 0 (neutral: the left factor is simply not
  applied — fail-safe is 'no extra penalty', never a wrong-direction penalty). Pure."""
  if not points or cur_lat is None or cur_lon is None or target_dist == float('inf'):
    return 0
  pts = []
  for p in points:
    try:
      la, lo = float(p["latitude"]), float(p["longitude"])
    except (KeyError, TypeError, ValueError):
      continue
    if la != la or lo != lo:                      # NaN guard
      continue
    pts.append((la, lo))
  if len(pts) < 3:
    return 0
  # nearest path point to the candidate distance
  best_i, best_err = -1, float('inf')
  for i, (la, lo) in enumerate(pts):
    err = abs(_haversine_m(cur_lat, cur_lon, la, lo) - target_dist)
    if err < best_err:
      best_i, best_err = i, err
  if best_err > tol_m:
    return 0
  # signed turn accumulated over up to 2 corners each side of the candidate point (local EN frame;
  # per-corner cross is normalized = sin(turn angle), so GPS point spacing doesn't bias the sum)
  coslat = math.cos(math.radians(cur_lat))
  total = 0.0
  for j in range(max(1, best_i - 2), min(len(pts) - 1, best_i + 3)):
    ax = (pts[j][1] - pts[j - 1][1]) * coslat
    ay = pts[j][0] - pts[j - 1][0]
    bx = (pts[j + 1][1] - pts[j][1]) * coslat
    by = pts[j + 1][0] - pts[j][0]
    na, nb = math.hypot(ax, ay), math.hypot(bx, by)
    if na <= 0.0 or nb <= 0.0:
      continue
    total += (ax * by - ay * bx) / (na * nb)
  if abs(total) < 0.02:                           # ~1 deg total bend: too straight to call
    return 0
  return 1 if total > 0.0 else -1


def upcoming_curve(target_velocities, cur_lat, cur_lon, v_ego, lookahead_s) -> tuple[float, float]:
  """From pfeiferj's MapTargetVelocities (list of {latitude, longitude, velocity}) + current
  position, return (min_target_velocity, distance) of the most-binding upcoming curve within the
  lookahead distance (v_ego * lookahead_s). Returns (0.0, inf) if none / no data. Pure & testable."""
  if not target_velocities or cur_lat is None or cur_lon is None:
    return 0.0, float('inf')
  horizon = max(v_ego, 1.0) * lookahead_s
  best_v, best_d = 0.0, float('inf')
  for p in target_velocities:
    try:
      d = _haversine_m(cur_lat, cur_lon, p["latitude"], p["longitude"])
      tv = float(p["velocity"])
    except (KeyError, TypeError, ValueError):
      continue
    if tv != tv or d != d:   # NaN guard: mapd occasionally emits non-finite velocities (seen live
      continue               # 2026-07-11) — a NaN silently falsifies every comparison downstream
    if 0.0 < d <= horizon:
      # most-binding = lowest target speed ahead within the horizon
      if best_v == 0.0 or tv < best_v:
        best_v, best_d = tv, d
  return best_v, best_d


class Condition:
  """A debounced boolean signal: raw bool -> filtered -> compared to THRESHOLD. The filter is driven
  by the MEASURED loop dt (selfdrived runs at 100 Hz / DT_CTRL, not the model rate) so FILTER_TAU is a
  real time constant — using a fixed DT_MDL here made the debounce run 5x too fast (instant flapping)."""
  def __init__(self):
    self.f = FirstOrderFilter(0.0, C.FILTER_TAU, DT_CTRL)
    self.active = False

  def update(self, raw: bool, dt: float = DT_CTRL) -> bool:
    # tolerance, not equality: the measured dt jitters every cycle, and an exact != recomputed the
    # filter alpha (an exp()) at 100 Hz per condition for no behavioral gain
    if abs(dt - self.f.dt) > 1e-3:
      self.f.dt = dt
      self.f.update_alpha(C.FILTER_TAU)
    self.f.update(1.0 if raw else 0.0)
    self.active = self.f.x >= C.THRESHOLD
    return self.active

  def reset(self):
    self.f.x = 0.0
    self.active = False


def _accelerate_zone(s) -> bool:
  """PURE: True when we're slow but should be ACCELERATING into open road, so Experimental's timid
  e2e acceleration would hurt — keep Chill instead. Covers the two cases:
    - highway on-ramp merge (open road ahead, set speed = highway >> ramp speed)
    - stop&go where the lead has pulled away leaving a big gap (catch back up at Chill briskness)
  Requires open road ahead AND a set speed meaningfully above current speed. Only gates `lowSpeed`.

  redlight2pnw (SAFETY, driver report 2026-07-11, confirmed in ces_events — exp->chill at 0-5 mph
  with az=True): approaching a red light with no lead and a high set speed has the SAME signals as an
  on-ramp merge (open road + set >> ego), so this gate suppressed the low-speed Experimental hold at
  the last moment and Chill's MPC accelerated toward the set speed -> lurch through the light. Two
  fail-safe guards (both only ever KEEP Experimental, which DOES stop for lights):
    (a) defer to the model's stop prediction — if it says stop, this is never an accelerate-zone;
    (b) the NO-LEAD branch (pure open road = could be a red light) requires a real merge speed floor;
        near a stop you are never 'merging onto a highway'. The has-lead-far branch (a genuine lead
        pull-away, where a lead is present so it can't be a clear red light) is unchanged."""
  if s.get("model_should_stop"):
    return False
  no_lead = not s["has_lead"]
  if no_lead and s["v_ego"] < C.ACCEL_ZONE_MIN_V:
    return False
  open_ahead = no_lead or (s["lead_drel"] > C.GAP_OPEN_M
                           and s["lead_vlead"] >= s["v_ego"] - C.LEAD_PULLAWAY_MARGIN)
  want_faster = s["v_set"] > 0.0 and (s["v_set"] - s["v_ego"]) > C.ACCEL_ZONE_DV
  return open_ahead and want_faster


def decide_active(s) -> tuple[bool, str]:
  """PURE decision core (no state, no filtering): given a signals dict-like `s`, return
  (any_condition_active, status). Used by both the live controller (post-filter) and the unit tests.

  Expected keys (all SI, primitives):
    v_ego, has_lead, lead_vlead, lead_drel, blinker,
    map_target_v, map_target_dist, curve_lat_accel_vision, time_to_curve,
    model_should_stop,
    toggles: curves/stops/low_speed/lead (bool enables)
  """
  t = s["toggles"]
  v = s["v_ego"]

  # 1) curve — map (primary, ~10 s) OR vision (fallback, ~3.5 s)
  # Freeway gate: on a known-freeway (OSM spd_lim >= CURVE_HWY_GATE) hand MODERATE curves to VTSC+MTSC
  # (bounded, decel-limited) and DON'T trip Experimental e2e curve braking — it stacks below the VTSC floor
  # and over-slows (driver gas-override on the 2026-06-28 Snoqualmie sweepers). spd_lim 0/unknown => not
  # gated (keep tripping, safe default). Only the curve reason is gated; stop/lead/radar are intact.
  # SHARP-curve exception: a genuinely sharp curve (map target < CURVE_SHARP_MAP_V) keeps the trip even on
  # a freeway — that's where steering-limit/EPS risk is (2026-06-28 North Bend descent take-control), so we
  # want maximum braking authority (e2e + VTSC + MTSC), not just the bounded cap.
  freeway_gated = s["spd_lim"] >= C.CURVE_HWY_GATE
  # Compare the SCALED map target (what MTSC actually drives to), not the raw one: mapd's raw safe-speeds
  # run systematically low (~1.5-1.8x), so the raw compare tripped Experimental on I-84 sweepers the driver
  # takes at 79-86 mph (raw 24-30 m/s, 2026-07-06 09:42-09:48 cluster). Genuinely sharp curves (scaled
  # target still < CURVE_SHARP_MAP_V) keep the exception and full braking authority.
  # sharpcurve2pnw iter2: use the TIERED effective scale (shared helper) so this classification and the
  # MTSC fold agree — a flat 1.8 here would inflate a tight curve's target past CURVE_SHARP_MAP_V and
  # drop its sharp flag exactly where full braking authority matters most.
  sharp_curve = 0.0 < s["map_target_v"] * C.tiered_map_scale(s["map_target_v"]) < C.CURVE_SHARP_MAP_V
  # ces2pnw lead-pacing gate (2026-07-08 02:12:57Z, lebowski first drive): a curve-triggered
  # Experimental HELD through an extended 100%-map winding stretch while the (faster) lead pulled away
  # 43->126 m — e2e crawls, and the driver ruled it unacceptable ("the lead car cannot pull away").
  # When a lead within CURVE_LEAD_PACE_DREL is pacing us (not slower than us minus the margin), the
  # curve trip is suppressed ENTIRELY — including sharp curves: the pacing lead is live evidence of a
  # drivable line, and VTSC+MTSC (tiered scale + decel envelope + sharp-curve firmer rate-limit) remain
  # the independent physical cap either way. A lead that brakes/slows flips to the slowLead trigger
  # below; a lead beyond the range (or lost) re-arms the curve trip immediately.
  lead_pacing = (s["has_lead"] and s["lead_drel"] < C.CURVE_LEAD_PACE_DREL
                 and s["lead_vlead"] >= v - C.LEAD_PULLAWAY_MARGIN)
  # ces2pnw accel-zone curve gate (2026-07-09 03:01-03:15Z on-ramp): a highway on-ramp IS a curve, so
  # the curve trip pinned the merge at 38-39 mph (set 90, no lead, aEgo ~0 for 7 s) while the driver
  # rode the gas — _accelerate_zone (already gating lowSpeed for exactly this merge case) now gates the
  # curve trip too. VTSC (vision) + tiered MTSC still cap the curved portion physically; suppressing
  # only the Experimental e2e layer lets Chill's MPC pull to the merge speed the moment VTSC releases.
  # Solo-cruising-at-set into a curve (v_set ~ v_ego, e.g. Terwilliger) keeps the trip: az is False there.
  if t["curves"] and v > C.CRUISING_SPEED and (not freeway_gated or sharp_curve) \
     and not lead_pacing and not _accelerate_zone(s):
    # MAP: pfeiferj MapTargetVelocities gives a safe curve speed ahead. Trip when an upcoming
    # target speed within the lookahead is meaningfully (>MIN_SLOWDOWN) below current speed.
    map_curve = (s["map_target_v"] > 0.0
                 and (v - s["map_target_v"]) > C.CURVE_MAP_MIN_SLOWDOWN
                 and 0.0 < s["map_target_dist"] / max(v, 1.0) < C.CURVE_MAP_LOOKAHEAD_S)
    # VISION fallback: predicted lateral accel over the (short) model horizon.
    vision_curve = (abs(s["curve_lat_accel_vision"]) > C.CURVE_LAT_ACCEL_ENTER
                    and s["time_to_curve"] < C.CURVE_VISION_LOOKAHEAD_S
                    and not s["blinker"])
    if map_curve or vision_curve:
      return True, "curve"

  # 2) stop light / stop sign — model predicts a stop, not currently following a lead
  if t["stops"] and s["model_should_stop"] and not s["has_lead"]:
    return True, "stop"

  # 3) low speed (city / complex / construction) — lead-aware threshold. TWO exceptions, both
  #    learned from the drive log (only ever REMOVE Experimental, so safe):
  #    (a) highway gate: skip on a road whose OSM speed limit is high — slow-but-following on a
  #        highway is normal Chill cruising, not a complex zone.
  #    (b) accelerate-zone: skip when we should be accelerating into open road (on-ramp merge /
  #        lead pulled away) — Experimental's timid e2e acceleration is bad there.
  thr = C.CES_SPEED_LEAD if s["has_lead"] else C.CES_SPEED
  on_highway = s["spd_lim"] >= C.LOWSPEED_HWY_GATE
  if t["low_speed"] and 1.0 <= v < thr and not on_highway and not _accelerate_zone(s):
    return True, "lowSpeed"

  # 4) slow / stopped lead — closing on a slower/stopped lead -> let e2e do the smooth decel
  if t["lead"] and s["has_lead"]:
    if (v - s["lead_vlead"]) > C.SLOW_LEAD_DV or s["lead_vlead"] < C.STOPPED_LEAD_V:
      return True, "slowLead"

  return False, "chill"


def _clamp01(x: float) -> float:
  return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


def curve_closeness(s) -> tuple[float, str]:
  """PURE, display-only: 'how close are we to tripping Experimental for a curve', 0.0..1.0, plus
  which half drives it ('map' / 'vision' / ''). 1.0 == at/over the entry threshold (switch imminent).
  Mirrors the curve branch of `decide_active` but as a continuous ratio for the on-screen feedback —
  it does NOT make the decision. 0.80 ~= "very close", 0.99 ~= "about to switch", >=1.0 == tripping."""
  t = s["toggles"]
  v = s["v_ego"]
  if not t["curves"] or v <= C.CRUISING_SPEED:
    return 0.0, ""
  # MAP half: how far the upcoming safe curve speed sits below us vs the slowdown that trips it,
  # but only while that curve is within the lookahead time.
  map_close = 0.0
  mv, md = s["map_target_v"], s["map_target_dist"]
  if mv > 0.0 and 0.0 < md / max(v, 1.0) < C.CURVE_MAP_LOOKAHEAD_S:
    map_close = _clamp01((v - mv) / C.CURVE_MAP_MIN_SLOWDOWN)
  # VISION half: predicted lateral accel vs the entry threshold, within the (short) vision horizon.
  vis_close = 0.0
  if s["time_to_curve"] < C.CURVE_VISION_LOOKAHEAD_S and not s["blinker"]:
    vis_close = _clamp01(abs(s["curve_lat_accel_vision"]) / C.CURVE_LAT_ACCEL_ENTER)
  if map_close >= vis_close:
    return map_close, ("map" if map_close > 0.0 else "")
  return vis_close, "vision"


def decision_telemetry(s) -> dict:
  """PURE, display-only: a compact snapshot for the on-screen CES overlay. Reports the binding
  reason, the curve 'closeness' as a 0..100 %, and the upcoming map-curve preview (target speed +
  distance). Built from the SAME signals dict `decide_active` consumes, so the overlay can never
  disagree with the live decision."""
  raw_active, reason = decide_active(s)
  cpct, csrc = curve_closeness(s)
  md = s["map_target_dist"]
  return {
    "rawActive": bool(raw_active),
    "reason": reason,
    "curvePct": int(round(cpct * 100)),
    "curveSrc": csrc,
    "mapV": round(float(s["map_target_v"]), 1),
    "mapDist": round(float(md), 0) if md != float('inf') else 0.0,
    "vEgo": round(float(s["v_ego"]), 1),
    # accelerate-zone + the signals that drive it (also logged per event for later tuning)
    "accelZone": _accelerate_zone(s),
    "hwyGate": s.get("spd_lim", 0.0) >= C.LOWSPEED_HWY_GATE,   # lowSpeed suppressed: on a highway
    "vSet": round(float(s["v_set"]), 1),
    "dRel": round(float(s["lead_drel"]), 0),
    "vLead": round(float(s["lead_vlead"]), 1),
    "aEgo": round(float(s.get("a_ego", 0.0)), 2),
    "gas": bool(s.get("gas", False)),
  }


class ConditionalExperimentalSwitching:
  """Live controller. Owns the per-condition filters + the mode state machine (min-dwell + sustained
  clear). `mode()` returns 'experimental'/'chill'; `update(sm, toggles)` is called each cycle."""

  def __init__(self, exp_min_dwell: float = C.EXP_MIN_DWELL_S, chill_min_dwell: float = C.CHILL_MIN_DWELL_S):
    # one debounce filter per condition (entry) + one for the all-clear (exit)
    self._cond = Condition()        # "any condition active" (debounced)
    self._is_experimental = False
    self._dwell = 0.0               # s in current mode
    self._status = "chill"
    self._exp_min = exp_min_dwell   # min dwell in Experimental (gentle profile lengthens this)
    self._chill_min = chill_min_dwell  # min dwell in Chill / re-entry cooldown

  def reset(self):
    self._cond.reset()
    self._is_experimental = False
    self._dwell = 0.0
    self._status = "chill"

  def mode(self) -> str:
    return "experimental" if self._is_experimental else "chill"

  def status(self) -> str:
    return self._status

  def update_decision(self, signals: dict, dt: float = DT_CTRL) -> str:
    """Advance the state machine one cycle from an extracted `signals` dict (see decide_active).
    `dt` is the MEASURED loop period (selfdrived runs at 100 Hz) so the dwell/debounce are real
    seconds. Separated from `update(sm)` so it is unit-testable without cereal messages."""
    raw_active, status = decide_active(signals)
    cond_active = self._cond.update(raw_active, dt)   # debounced (real-time)
    self._dwell += dt

    if not self._is_experimental:
      # enter Experimental once the debounced condition is active AND we've been in Chill at least
      # the re-entry cooldown (de-flap: stops the instant snap-back that caused the stop&go sawtooth)
      if cond_active and self._dwell >= self._chill_min:
        self._is_experimental = True
        self._status = status
        self._dwell = 0.0
    else:
      # stay Experimental; return to Chill only when the condition cleared (sustained via filter)
      # AND we've held Experimental at least EXP_MIN_DWELL_S
      if status != "chill":
        self._status = status      # keep showing the active reason
      if not cond_active and self._dwell >= self._exp_min:
        self._is_experimental = False
        self._status = "chill"
        self._dwell = 0.0
    return self.mode()


# ---------------------------------------------------------------------------
# Phase 2/3 — live wiring. Runs in selfdrived (which publishes the effective
# experimentalMode → both the planner AND the top-right icon follow it).
# Behavior-neutral: experimental_request() returns False whenever CES is
# disabled/non-Tesla, so selfdrived's `manual OR request` == manual == upstream.
# ---------------------------------------------------------------------------

def _toggles_from_params(params) -> dict:
  """Per-condition enables; default ON (the master switch is the real gate)."""
  def gb(k):
    try:
      return params.get_bool(k)
    except Exception:
      return True
  return {"curves": gb("CESCurves"), "stops": gb("CESStops"),
          "low_speed": gb("CESLowSpeed"), "lead": gb("CESLead")}


def _signals_from(car_state, lead, model, toggles: dict, map_target_v: float, map_target_dist: float,
                  spd_lim: float = 0.0) -> dict:
  """Build the decision primitives from STOCK messages (carState, radarState.leadOne, modelV2)
  plus the map-curve result (map_target_v/dist) and the OSM speed limit (spd_lim, m/s, for the
  lowSpeed highway gate). Defensive: missing/odd data falls back to 'nothing happening' (stay Chill)."""
  v_ego = float(car_state.vEgo)

  has_lead = bool(getattr(lead, 'status', False))
  lead_vlead = float(getattr(lead, 'vLead', 0.0)) if has_lead else 0.0
  lead_drel = float(getattr(lead, 'dRel', 0.0)) if has_lead else 0.0

  try:
    orz = list(model.orientationRate.z); vx = list(model.velocity.x); tb = list(model.orientationRate.t)
    vis_acc, ttc = vision_curve_lat_accel(orz, vx, tb, v_ego)
  except Exception:
    vis_acc, ttc = 0.0, 10.0
  try:
    model_should_stop = bool(model.action.shouldStop)
  except Exception:
    model_should_stop = False

  # set speed (openpilot's v_cruise, km/h on carState.vCruise) -> m/s; 255 is the unset sentinel.
  v_set_kph = float(getattr(car_state, 'vCruise', 0.0))
  v_set = v_set_kph * CV.KPH_TO_MS if 0.0 < v_set_kph < C.V_SET_MAX_KPH else 0.0

  return {
    "v_ego": v_ego, "has_lead": has_lead, "lead_vlead": lead_vlead, "lead_drel": lead_drel,
    "blinker": bool(car_state.leftBlinker or car_state.rightBlinker),
    "map_target_v": map_target_v, "map_target_dist": map_target_dist,   # map half (MapTargetVelocities)
    "curve_lat_accel_vision": vis_acc, "time_to_curve": ttc,            # vision fallback
    "model_should_stop": model_should_stop, "toggles": toggles,
    "v_set": v_set,                                                     # accelerate-zone (set-speed gap)
    "spd_lim": float(spd_lim),                                         # OSM speed limit (lowSpeed highway gate)
    "a_ego": float(getattr(car_state, 'aEgo', 0.0)),                   # logged for verification
    "gas": bool(getattr(car_state, 'gasPressed', False)),             # logged for verification
  }


class CESController:
  """Live wrapper used by selfdrived. Owns the state machine + ~1 Hz param refresh + the 3-state
  button (CESButtonState: 0=CES, 1=forced Chill, 2=forced Experimental) + the map-curve read.
  Gated on openpilotLongitudinalControl (NOT brand — available on every car, like the stock
  Experimental toggle). experimental_request() returns False when disabled → behavior-neutral."""
  def __init__(self, CP, params=None):
    import platform
    from openpilot.common.params import Params
    self.CP = CP
    self.params = params or Params()
    # pfeiferj mapd writes MapTargetVelocities/LastGPSPosition to the in-memory param store
    try:
      self.mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else self.params
    except Exception:
      self.mem_params = None
    # light-ces-gentle: the gentle profile is now USER-SELECTED via CESMode (1=Light), NOT gated on
    # carFingerprint. Light = longer dwell + VTSC owns curves (no curve->Experimental) on ANY car;
    # Standard = default tune. _read_params() rebuilds the state machine if the mode changes at runtime.
    self._mode = C.CES_MODE_OFF
    self._gentle = False
    self._sm = ConditionalExperimentalSwitching()
    self._enabled = False
    self._button = C.BTN_CES
    self._toggles = {"curves": True, "stops": True, "low_speed": True, "lead": True}
    self._map_targets = []          # cached MapTargetVelocities (refreshed ~1 Hz)
    self._cur_lat = self._cur_lon = self._cur_bearing = None
    self._vtsc_cap = self._vtsc_state = None
    self._speed_limit = 0.0         # OSM speed limit (m/s, 0 = none) from mapd
    self._frame = 0
    # telemetry / logging (display + diagnostics only — never gates control)
    self._last_mode = "off"         # last logged mode: off / chill / experimental
    self._tele_last = 0.0           # monotonic stamp of last CESStatus publish
    self._tick_last = 0.0           # monotonic stamp of last breadcrumb tick
    self._last_decide_t = None      # monotonic stamp of last state-machine step (for real dt)
    self._bs_l = False              # bsm2pnw: last-seen blind-spot booleans (telemetry only —
    self._bs_r = False              #   proves BSM liveness in ces_events; never gates control here)
    self._event_log_ok = False      # persistent "each adoption" trail (CES_EVENT_LOG)
    try:
      os.makedirs(os.path.dirname(CES_EVENT_LOG), exist_ok=True)
      self._event_log_ok = True
    except Exception:
      self._event_log_ok = False
    # capability view (driver directive 2026-07-11: check CAPABILITIES, never fingerprints here —
    # pnw_vehicle.PnwVehicle is the one place that maps cars to features):
    #   _long_ok -> openpilot owns longitudinal (planner actuation)
    #   _shadow  -> CES runs shadow with ICBM as the actuator (stock-ACC buttons, no op-long)
    veh = PnwVehicle(CP)
    self._veh = veh                        # curveslow-lightning: per-car curve-speed penalty (ICBM apex)
    self._long_ok = veh.op_long
    self._shadow = veh.ces_shadow
    # icbm2pnw: latched driver set speed while a curve cap is active (see icbm_curve_target), a
    # publish throttle for the IcbmTarget mem-param heartbeat, and the last published target +
    # stock-ACC readings for the ces_events closed-loop trace.
    self._icbm_ceiling = None
    self._icbm_last_pub = 0.0
    self._icbm_last_target = None
    self._icbm_src = None                  # curveslow-lightning: "map"/"vis"/None for the drive log
    self._stock_set = 0.0
    self._stock_on = False

  def _set_mode(self, mode: int):
    """Apply a CESMode change: pick the gentle vs default dwell and (re)build the state machine only
    when the gentle flag actually flips, so we don't reset the dwell every ~1 Hz read."""
    gentle = C.ces_is_gentle(mode)
    if mode != self._mode or gentle != self._gentle:
      if gentle != self._gentle:
        if gentle:
          self._sm = ConditionalExperimentalSwitching(C.GENTLE_EXP_MIN_DWELL_S, C.GENTLE_CHILL_MIN_DWELL_S)
        else:
          self._sm = ConditionalExperimentalSwitching()
      self._mode = mode
      self._gentle = gentle

  def _read_params(self):
    if self._frame % max(1, int(1.0 / DT_CTRL)) == 0:   # ~1 Hz (selfdrived steps at 100 Hz / DT_CTRL)
      try:
        mode = C.read_ces_mode(self.params)
      except Exception:
        mode = C.CES_MODE_OFF
      self._set_mode(mode)
      # CES is meaningful only when openpilot owns longitudinal (same gate as ExperimentalMode) —
      # except in Lightning shadow mode, where the pipeline runs for telemetry/display only.
      self._enabled = (self._long_ok or self._shadow) and C.ces_enabled(self._mode)
      if self._enabled:
        self._toggles = _toggles_from_params(self.params)
        try:
          self._button = int(self.params.get("CESButtonState", return_default=True) or 0)
        except Exception:
          self._button = C.BTN_CES
        self._read_map()
    self._frame += 1

  def _read_map(self):
    """Refresh map-curve inputs + GPS + OSM speed limit from the pfeiferj mem params (defensive — any
    failure => no map curve, vision fallback still works). GPS + speed limit are read regardless of
    the curves toggle because the event log wants them at all times."""
    if self.mem_params is None:
      self._map_targets = []
      return
    # map curve targets — only when the curve condition is enabled
    if self._toggles.get("curves", True):
      try:
        self._map_targets = self.mem_params.get("MapTargetVelocities", return_default=True) or []
      except Exception:
        self._map_targets = []
    else:
      self._map_targets = []
    # GPS (lat/lon/bearing) — always (map-curve distance + logging)
    try:
      pos = self.mem_params.get("LastGPSPosition", return_default=True)
      if isinstance(pos, (bytes, str)):
        pos = json.loads(pos)
      self._cur_lat = float(pos["latitude"]); self._cur_lon = float(pos["longitude"])
      self._cur_bearing = float(pos.get("bearing", 0.0))
    except Exception:
      self._cur_lat = self._cur_lon = self._cur_bearing = None
    # OSM speed limit (m/s; 0 = none) — for the coarse highway guess in the log
    try:
      sl = self.mem_params.get("MapSpeedLimit", return_default=True)
      self._speed_limit = float(sl) if sl not in (None, "", b"") else 0.0
    except Exception:
      self._speed_limit = 0.0
    # VTSC applied cap + state — logging only (see _event_record)
    try:
      vt = self.mem_params.get("VTSCStatus", return_default=True)
      if isinstance(vt, (bytes, str)):
        vt = json.loads(vt)
      self._vtsc_cap = round(float(vt["cap"]), 1) if vt.get("engaged") else None
      self._vtsc_state = vt.get("state")
    except Exception:
      self._vtsc_cap = self._vtsc_state = None

  def enabled(self) -> bool:
    return self._enabled

  def status(self) -> str:
    return self._sm.status()

  def experimental_request(self, car_state, sm) -> bool:
    """True if CES wants Experimental this cycle. Reads params; advances the state machine.
    Safe to call always — returns False whenever CES is disabled (behavior-neutral)."""
    self._read_params()
    # bsm2pnw: sample the blind-spot booleans every cycle (cheap), so adopt/tick records carry them
    # even while CES is disabled — the point is drive-log evidence that BSM flips with passing cars.
    self._bs_l = bool(getattr(car_state, 'leftBlindspot', False))
    self._bs_r = bool(getattr(car_state, 'rightBlindspot', False))
    # icbm2pnw/lateral telemetry (driver req 2026-07-11 "more good data"): steering angle + driver
    # override per record — quantifies left-pull, curve-tracking failures and override clusters.
    self._str_ang = round(float(getattr(car_state, 'steeringAngleDeg', 0.0)), 1)
    self._str_prs = bool(getattr(car_state, 'steeringPressed', False))
    # icbm2pnw closed-loop trace: the STOCK ACC's reported set speed + engagement — with the
    # published target (icbmT below) this shows every executor tap landing (set stepping down).
    try:
      self._stock_set = round(float(car_state.cruiseState.speed), 2)
      self._stock_on = bool(car_state.cruiseState.enabled)
    except Exception:
      self._stock_set, self._stock_on = 0.0, False
    if not self._enabled:
      if self._last_mode != "off":
        cloudlog.info("CES disabled (master OFF / no openpilot long) -> Chill baseline")
        self._last_mode = "off"
      self._sm.reset()
      return False

    # Build the decision signals every cycle while enabled — even in the forced button modes —
    # so the on-screen overlay always reflects what CES sees (curve %, upcoming curve preview).
    sig = None
    try:
      lead = sm['radarState'].leadOne
      model = sm['modelV2']
      v_ego = float(car_state.vEgo)
      mtv, mtd = upcoming_curve(self._map_targets, self._cur_lat, self._cur_lon, v_ego, C.CURVE_MAP_LOOKAHEAD_S)
      # gentle profile: VTSC handles curve speed (smooth, decel-limited), so CES does NOT trip
      # Experimental for curves on the truck — removes the chill<->experimental planner-mode flapping.
      toggles = {**self._toggles, "curves": False} if self._gentle else self._toggles
      sig = _signals_from(car_state, lead, model, toggles, mtv, mtd, self._speed_limit)
      # icbmalign2pnw: road pitch for the ICBM descent guard — the SAME message/field VTSC reads
      # (carControl.orientationNED[1], rad, < 0 = downhill; selfdrived's SubMaster subscribes
      # carControl). Own inner try: a carControl hiccup must not cost the whole signals dict.
      try:
        ned = sm['carControl'].orientationNED
        sig["pitch"] = float(ned[1]) if len(ned) == 3 else None
      except Exception:
        sig["pitch"] = None
    except Exception:
      sig = None

    # measured loop period — selfdrived steps at ~100 Hz; never assume a fixed DT (was the 5x bug)
    now_t = time.monotonic()
    dt = (now_t - self._last_decide_t) if self._last_decide_t is not None else DT_CTRL
    self._last_decide_t = now_t
    dt = min(max(dt, 1e-3), 0.5)           # clamp first call / scheduling hiccups

    if self._button == C.BTN_CHILL:        # forced Chill
      self._sm.reset()
      want = False
    elif self._button == C.BTN_EXP:        # forced full Experimental
      want = True
    elif sig is not None:                  # BTN_CES: condition ladder decides
      want = self._sm.update_decision(sig, dt) == "experimental"
    else:
      want = False

    self._publish_status(sig, want)
    # icbm2pnw: in Lightning shadow mode the CES/planner path never actuates, but the ICBM brain
    # publishes a stock-ACC set-speed target the ford carcontroller executor follows (curve
    # slow-down, dec-only against the driver's own set — see icbm_curve_target). ICBM mirrors the
    # button: ACTIVE only in the CES state, SILENT in forced Chill (the truck's button flips only
    # CES<->Chill — forced Exp is unreachable there). Publishing empty in Chill stops the executor.
    if self._shadow:
      self._icbm_step(sig, active=(sig is not None and self._button == C.BTN_CES))
    return want and self._long_ok

  def _icbm_step(self, sig, active: bool) -> None:
    """Publish the IcbmTarget mem-param at ~4 Hz (executor treats >2 s silence as stale-stop).
    Best-effort: never raises into the control path. `active` False -> publish empty (ICBM off)."""
    now = time.monotonic()
    if now - self._icbm_last_pub < 0.25 or self.mem_params is None:
      return
    self._icbm_last_pub = now
    try:
      if not active:
        self._icbm_ceiling = None
        self._icbm_last_target = None
        self._icbm_src = None
        self.mem_params.put_nonblocking("IcbmTarget", {})
        return
      # curveslow-lightning: vision curve candidate (the 493-curve gap: ICBM was MAP-ONLY and blind to
      # camera-seen curves). icbm_curve_target picks the more-binding of map / vision / far-map.
      vis_v, vis_dist = icbm_vision_apex(sig["v_ego"], sig.get("curve_lat_accel_vision", 0.0),
                                         sig.get("time_to_curve", float("inf")))
      # descentcurve2pnw: full-horizon map candidate — at highway speed the 10 s window (~400 m at
      # 90 mph) hid curves that need the full 500 m mapd publishes (the 2026-07-11 silent-ICBM run).
      # Capability-supplied knobs: map_scale (<=1 OSM discount) + firm_decel (large-drop envelope);
      # both neutral (1.0 / 0.0) on any non-Lightning.
      ref = self._icbm_ceiling if self._icbm_ceiling is not None else sig["v_set"]
      far_v, far_dist = icbm_far_map_candidate(self._map_targets, self._cur_lat, self._cur_lon,
                                               sig["v_ego"], ref, C.tiered_map_scale,
                                               self._veh.icbm_map_scale, self._veh.icbm_firm_decel)
      target, self._icbm_ceiling, self._icbm_src = icbm_curve_target(
        sig["v_ego"], sig["v_set"], sig.get("map_target_v", 0.0),
        sig.get("map_target_dist", float("inf")), self._icbm_ceiling, C.tiered_map_scale,
        vis_v, vis_dist,
        map_scale=self._veh.icbm_map_scale, firm_decel=self._veh.icbm_firm_decel,
        far_v=far_v, far_dist=far_dist)
      # curveslow-lightning: lower the chosen apex on the Lightning (weaker EPS -> enter curves slower).
      # Penalty is >= 0 (never a speed-up), only lowers -> still reduce-only vs the ceiling; floor 0.
      # icbmalign2pnw: ICBM now applies the SAME descent + left-curve multipliers as VTSC — literally
      # the same shared function (PnwVehicle.curve_speed_penalty_ms, knobs from /data/pnw/curve.json),
      # so stock-ACC and op-long behavior stay aligned. Direction per source:
      #   vis      -> sign of the model's predicted lateral accel (lat = orientationRate.z * v, and
      #               z > 0 = LEFT — the convention verified for apex_turn_direction);
      #   map/far  -> map path geometry at the candidate's distance (map_turn_direction, same
      #               left-positive convention). Unknown direction / no pitch -> neutral (no-op).
      # Multipliers only ever RAISE the penalty (>= 1, clamped), so the target only moves DOWN:
      # DEC-only/ceiling semantics untouched.
      if target is not None:
        is_left = False
        try:
          if self._icbm_src == "vis":
            is_left = float(sig.get("curve_lat_accel_vision", 0.0) or 0.0) > 0.0
          elif self._icbm_src == "far":
            is_left = map_turn_direction(self._map_targets, self._cur_lat, self._cur_lon, far_dist) > 0
          elif self._icbm_src == "map":
            is_left = map_turn_direction(self._map_targets, self._cur_lat, self._cur_lon,
                                         sig.get("map_target_dist", float("inf"))) > 0
        except Exception:
          is_left = False
        target = max(target - self._veh.curve_speed_penalty_ms(target, pitch_rad=sig.get("pitch"),
                                                               is_left=is_left), 0.0)
      self._icbm_last_target = round(target, 2) if target is not None else None
      if target is not None:
        # JSON params take a DICT (params_pyx serializes it; a pre-dumped string raises TypeError —
        # Gemini review catch that would have silently killed every publish)
        self.mem_params.put_nonblocking("IcbmTarget", {"target": round(target, 2), "ceiling": round(self._icbm_ceiling, 2), "ts": time.time()})  # noqa: TID251 -- wall clock heartbeat shared with the executor
      else:
        self.mem_params.put_nonblocking("IcbmTarget", {})
    except Exception:
      pass

  def _publish_status(self, sig, want: bool) -> None:
    """Log mode transitions and publish a throttled CESStatus snapshot to the in-memory param store
    for the on-screen overlay. Display/diagnostics only — never affects the returned decision."""
    mode = "experimental" if want else "chill"
    tele = decision_telemetry(sig) if sig is not None else {
      "reason": "noData", "curvePct": 0, "curveSrc": "", "mapV": 0.0, "mapDist": 0.0, "vEgo": 0.0,
    }
    tele["mode"] = mode
    tele["button"] = int(self._button)
    tele["enabled"] = True
    # mapd diagnostics so the overlay can always show what mapd is up to (curve half is map-driven):
    tele["mapPts"] = len(self._map_targets)                       # MapTargetVelocities points cached
    tele["gps"] = self._cur_lat is not None and self._cur_lon is not None  # LastGPSPosition fix present
    # icbm2pnw overlay feed (driver req 2026-07-11): current button-management target + the truck's
    # reported stock set speed, so the debug box can show "ICBM 24>18" while taps are stepping it down.
    # The shadow flag itself MUST be in the overlay feed too — the renderer keys the grey SHADOW
    # labels and the ICBM line on it (it was only in the ces_events record; the overlay kept showing
    # orange EXPERIMENTAL in shadow — driver caught it twice, 2026-07-11).
    tele["shadow"] = self._shadow
    if self._shadow:
      tele["icbmT"] = self._icbm_last_target
      tele["icbmSrc"] = self._icbm_src           # curveslow-lightning: "map"/"vis" source of the target
      tele["icbmSet"] = self._stock_set
      tele["icbmOn"] = self._stock_on

    # (a) transition ("adopt") — one record per chill<->experimental change, cloudlog + event file.
    if mode != self._last_mode:
      cloudlog.info("CES %s->%s button=%d reason=%s curve=%d%%(%s) vEgo=%.1f vSet=%.1f az=%s mapV=%.1f",
                    self._last_mode, mode, self._button, tele.get("reason"),
                    tele.get("curvePct", 0), tele.get("curveSrc", ""), tele.get("vEgo", 0.0),
                    tele.get("vSet", 0.0), tele.get("accelZone"), tele.get("mapV", 0.0))
      rec = self._event_record("adopt", tele)
      rec["from"], rec["to"] = self._last_mode, mode
      self._append_event(rec)
      self._last_mode = mode
    else:
      # (b) heartbeat ("tick") — ~1 Hz breadcrumb so the WHOLE drive's GPS track + state is captured
      # (lets us place every adoption on the route and apply the highway / 300 ft buffer in analysis).
      now2 = time.monotonic()
      if now2 - self._tick_last >= C.TICK_S:
        self._tick_last = now2
        self._append_event(self._event_record("tick", tele))

    # ~5 Hz publish to /dev/shm/params (put a dict -> JSON; nonblocking so the safety loop never waits)
    if self.mem_params is None:
      return
    now = time.monotonic()
    if now - self._tele_last < 0.2:
      return
    self._tele_last = now
    try:
      self.mem_params.put_nonblocking("CESStatus", tele)
    except Exception:
      pass

  def _event_record(self, kind: str, tele: dict) -> dict:
    """Build one rich, flat record for the persistent CES_EVENT_LOG. `kind` is "adopt" (a CES mode
    transition) or "tick" (a ~1 Hz breadcrumb). Includes GPS (lat/lon/bearing), OSM speed limit, a
    coarse highway guess, the accelerate-zone decision + its inputs (vSet/dRel/vLead/aEgo/gas), and
    the curve/map diagnostics — everything needed to verify behavior against the route later."""
    vego = float(tele.get("vEgo") or 0.0)
    hwy = (self._speed_limit >= C.HWY_SPEED_LIMIT) or (vego >= C.HWY_VEGO)  # coarse; authoritative = GPS+OSM+300ft in analysis
    return {
      "t": round(time.time(), 1),  # noqa: TID251 -- wall clock, for route/time correlation
      "ev": kind, "mode": tele.get("mode"), "reason": tele.get("reason"), "button": int(self._button),
      "vEgo": tele.get("vEgo"), "vSet": tele.get("vSet"), "aEgo": tele.get("aEgo"), "gas": tele.get("gas"),
      "accelZone": tele.get("accelZone"),
      "curvePct": tele.get("curvePct"), "curveSrc": tele.get("curveSrc"),
      "mapV": tele.get("mapV"), "mapDist": tele.get("mapDist"), "mapPts": tele.get("mapPts"),
      "dRel": tele.get("dRel"), "vLead": tele.get("vLead"),
      "gps": tele.get("gps"), "lat": self._cur_lat, "lon": self._cur_lon, "bearing": self._cur_bearing,
      "spdLim": round(self._speed_limit, 1), "hwy": bool(hwy),
      # VTSC applied cap + state (from the VTSCStatus mem param) — without this channel the 2026-07-06
      # I-84 gas-override cluster couldn't be attributed (VTSC/MTSC vs CES) from the log alone.
      "vtscCap": self._vtsc_cap, "vtscState": self._vtsc_state,
      # bsm2pnw: blind-spot booleans (carState.left/rightBlindspot) — liveness evidence for the
      # lane-change BSM gate; expect these to flip as traffic passes on real drives.
      "bsL": self._bs_l, "bsR": self._bs_r,
      # icbm2pnw: steering angle + driver-override flag (lateral quality forensics), and the shadow
      # marker — True on the Lightning where the planner path never actuates (ICBM may).
      "strAng": self._str_ang, "strPrs": self._str_prs, "shadow": self._shadow,
      # icbm2pnw closed-loop trace: published curve target (m/s, None = ICBM idle), latched driver
      # ceiling, the truck's reported stock set speed + engagement. icbmT stepping the stockSet down
      # in consecutive ticks = executor taps landing.
      "icbmT": self._icbm_last_target, "icbmC": self._icbm_ceiling, "icbmSrc": self._icbm_src,
      "stockSet": self._stock_set, "stockOn": self._stock_on,
    }

  def _append_event(self, rec: dict) -> None:
    """Append one JSON line to the persistent CES_EVENT_LOG (append-only, outside the overlay so it
    survives reboot + swaglog rotation). Best-effort; never breaks control."""
    if not self._event_log_ok:
      return
    try:
      try:
        if os.path.getsize(CES_EVENT_LOG) > CES_EVENT_LOG_MAX_BYTES:
          os.replace(CES_EVENT_LOG, CES_EVENT_LOG + ".1")   # atomic rotate; overwrites the previous .1
      except OSError:
        pass                                                # no file yet / stat race -> just append
      with open(CES_EVENT_LOG, "a") as f:
        f.write(json.dumps(rec) + "\n")
    except Exception:
      pass
