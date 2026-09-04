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
from collections import deque

from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
# greenlight2pnw/greenlead2pnw: pure standstill->release detector + release-cause classifier
# (sunnypilot mechanics + FrogPilot arming/lead rules — attribution in green_light.py).
# Display/sound only; never gates control.
from openpilot.selfdrive.controls.lib.ces_pnw.green_light import GreenLightDetector, GL_EV_GREEN, GL_EV_LEAD
# ces2core2pnw: the CES2 decision core (CES2-STUDY.md adoptions) — runs SHADOW every tick, decides
# live only when the Ces2Core param is set (default OFF => v1 path below is byte-identical).
from openpilot.selfdrive.controls.lib.ces_pnw.ces2_core import Ces2Core, DivergenceCounter
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle
# curveslow-lightning: ICBM's vision apex uses the SAME lateral-accel target as the VTSC vision path
# (v_safe = v_ego*sqrt(A_LAT/|lat|)) so the two subsystems agree on what a camera-seen curve "means".
# descentcurve2pnw: MAP_SOURCE_HORIZON_M is mapd's hard 500 m path cap — ICBM's full-horizon map scan
# uses the same constant family as VTSC/MTSC so both scan exactly what mapd publishes.
from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_constants import (A_LAT_TARGET as VTSC_A_LAT,
                                                                      MAP_SOURCE_HORIZON_M,
                                                                      MAP_SCALE_MIN)

# Persistent, append-only "each adoption" trail. Lives OUTSIDE /data/openpilot so it survives the
# boot overlay-swap AND swaglog rotation (a long drive rotates swaglog and would lose early events).
# One JSON line per CES mode transition, with GPS so we can map where each adoption happened.
CES_EVENT_LOG = "/data/pnw/ces_events.jsonl"
CES_EVENT_LOG_MAX_BYTES = 20 * 1024 * 1024   # rotate at 20 MB per generation
# cesretain2pnw: keep N rotated generations (.1 .. .N), not one. A single .1 gave a ~1-day window:
# the 2026-08-26 Olympic Peninsula trip (261 mi, two cars) had ALREADY rotated past by the time it
# was analysed — .1 started AFTER the driving ended, and the whole trip had to be reconstructed from
# S3 qlogs instead. Heavy driving writes ~21 MB/day (measured on that trip), so a full week needs
# >=147 MB of ROTATED history: 8 generations x 20 MB = 160 MB clears it, 7 would NOT (140 MB).
# Worst case 180 MB including the live file — 0.2% of the 89 GB /data.
CES_EVENT_LOG_GENERATIONS = 8
# stophold2pnw (D): the comma 3X RTC battery is dead (RTC reads 1970) — every cold boot writes
# event records with a garbage wall clock until NTP/GPS sync (a 2025-11-25-stamped record polluted
# the 2026-07-12 gap analysis). Records written before the clock is plausibly valid are MARKED
# (never dropped — the data is still real, only the timestamp is not).
CLOCK_VALID_EPOCH = 1577836800.0   # 2020-01-01T00:00Z


def clock_bad(t_wall: float) -> bool:
  """True when the wall clock is obviously pre-sync (dead-RTC boot). Pure."""
  try:
    return float(t_wall) < CLOCK_VALID_EPOCH
  except (TypeError, ValueError):
    return True


def rotate_event_log(path: str, generations: int) -> None:
  """cesretain2pnw: shift path.1..path.(N-1) down one and move path -> path.1, keeping `generations`
  rotated files. Oldest (path.N) is dropped. Each step is an atomic os.replace, so a crash mid-rotate
  loses at most one generation and never the live file. Caller has already checked the size."""
  for i in range(max(int(generations), 1) - 1, 0, -1):
    try:
      os.replace(f"{path}.{i}", f"{path}.{i + 1}")
    except OSError:
      pass          # that generation doesn't exist yet -> nothing to shift
  os.replace(path, f"{path}.1")


# steerpower2pnw: LOGGING ONLY -- measure the truck's true hands-off steering capability by direction.
# achLat = achieved curvature (kActl, yaw-rate-derived, already logged as slKActl/kActl elsewhere) *
# vEgo^2, signed -- the delivered lateral accel this tick. Grouping the steerEvent peakAchLat by the
# compass heading below (offline) yields max(peakAchLat) per direction -> a capability map ->
# slowdown target v=sqrt(cap*R). Both helpers are PURE, no I/O, never raise -- controlsd.py carries an
# independent copy of _ach_lat (as `_ach_lat_ms2`) since the two processes don't share code, only the
# formula (see that function's docstring for why it's duplicated, not imported).
def _ach_lat(k_actl, v_ego):
  """Delivered lateral accel this tick = k_actl * v_ego**2 (signed, m/s^2). None/non-finite k_actl or
  v_ego degrades to None, never raises."""
  try:
    if k_actl is None or v_ego is None:
      return None
    ach = float(k_actl) * float(v_ego) ** 2
    return ach if math.isfinite(ach) else None
  except (TypeError, ValueError):
    return None


_COMPASS_PTS = ("N", "NE", "E", "SE", "S", "SW", "W", "NW")


def _compass(bearing):
  """8-point compass heading from a GPS bearing (deg, 0=N, clockwise). None/non-finite bearing (no
  GPS fix yet) degrades to None, never raises."""
  try:
    if bearing is None:
      return None
    b = float(bearing)
    if not math.isfinite(b):
      return None
    return _COMPASS_PTS[int((b + 22.5) // 45) % 8]
  except (TypeError, ValueError):
    return None


# steerpower2pnw I4 review fix: a GPS fix that has lat/lon but no computed course (standstill, or the
# capnp bearingDeg field simply defaulting to 0.0 before the GPS stack has ever computed one) is
# indistinguishable from a genuine true-north 0.0 reading once it reaches _compass() -- silently
# biasing the "N" bucket with what are actually no-course-yet samples. Gate at the LOGGING site (not
# in _read_map -- other consumers of self._cur_bearing still want its 0.0 default) on the record's own
# `gps` boolean (lat AND lon both present -- the same test every record already uses to populate its
# "gps" field). A frozen/stale bearing while gps_valid is True is accepted (still logged, not nulled)
# -- only a genuinely absent fix degrades to None.
def _heading_if_fixed(bearing, gps_valid: bool):
  """8-pt compass heading, or None if `gps_valid` is False (no current lat/lon fix). Pure."""
  return _compass(bearing) if gps_valid else None


# steerpower2pnw I3 review fix: a steerEvent is emitted 0.75-5 s AFTER the saturation onset (the
# post-hold debounce in controlsd's flight-recorder state machine -- see FLIGHT_POST_HOLD_S /
# FLIGHT_MAX_EVENT_S), but heading was being stamped from self._cur_bearing, the ~1 Hz-refreshed
# value AT EMIT TIME. Through a curve, heading rotates ~15-20 deg/s, so the peak's true direction can
# be 45-90 deg off from the emit-time heading -- mis-filing peakAchLat under the wrong compass
# direction, defeating the by-direction capability map this whole feature exists to build. Fix: keep a
# small bounded ring of (wall_time, bearing, gps_valid) samples -- appended once per _read_map()
# refresh (~1 Hz, see its tail) -- and look up the sample NEAREST the episode's actual onset time
# (controlsd's "onsetT", see the _flight_start_wall comment there) instead of using the live value.
_BEARING_HIST_MAXLEN = 30   # ~30 s of ~1 Hz samples -- comfortably covers the 0.75-5 s emit lag


def _nearest_bearing(hist, t_wall):
  """Return (bearing, gps_valid) from the (wall_time, bearing, gps_valid) tuple in `hist` whose
  wall_time is closest to `t_wall`. (None, False) if `hist` is empty or `t_wall` isn't a real number.
  Pure; never raises."""
  if not hist:
    return None, False
  try:
    t_wall = float(t_wall)
  except (TypeError, ValueError):
    return None, False
  best = min(hist, key=lambda s: abs(s[0] - t_wall))
  return best[1], best[2]


# icbmonset: mapd's curvature calc (Heron's formula on near-collinear OSM nodes, see
# system/mapd/mapd_configd.py) occasionally emits a FINITE but physically-implausible high target
# velocity instead of NaN -- the existing NaN guards below don't catch it. Field-observed live
# 2026-07-18 (Ballard/Shilshole tight city curves, drive_report `drives/2026-07-18/
# lightning-icbm-curve/`): raw reads of 46.5-128.6 m/s (104-288 mph) at curve entry, settling to the
# real target (e.g. 14.6 m/s) 3-4 s later. The ceiling below is set ABOVE the highest RAW target ever
# exercised in this file's own test suite (110 mph / 49.2 m/s, `test_far_map_dec_only_above_ceiling_
# ignored` -- a genuine generous-sweeper reading, not noise) so it can never reclassify a previously
# real value as noise.
#
# SCOPE (Fable review, 2026-07-18): `_map_v_sane` is wired ONLY into `upcoming_curve` and
# `icbm_far_map_candidate` -- the two CANDIDATE scanners, where a rejected point can only ever REMOVE
# a value that would already have failed the reduce-only/binding test on its own (provably a no-op
# for every existing decision downstream: v_ego - tv is always very negative, tv*scale is always far
# above any sharp-curve/binding threshold). It is deliberately NOT wired into `icbm_map_reach`
# (reverted -- see that function's own docstring): pfeiferj's targetVelocity is a `sqrt(2/kappa)`-
# style function of curve RADIUS, so a wide/near-straight node (radius > ~1.7 km) legitimately
# reports tv > this ceiling, and a straight/unmatched node legitimately carries the capnp default
# 0.0 -- both are genuine "no slowdown needed" map verdicts on ordinary straight road, not noise.
# Gating REACH (map "coverage") on tv would have zeroed out coverage on straights and wrongly opened
# the MAP-FIRST gate (icbm_vision_may_start) for vision there -- the exact vision-over-slow class the
# 2026-07-12 driver rule ("vis=60 dec ticks") exists to suppress. `tv` alone can't distinguish "Heron
# glitch at a curve entry" from "legitimate gentle/straight road" -- reach stays position-based only.
#
# KNOWN PARTIAL COVERAGE (LOW, Fable finding 2): the ceiling can't be set below the legitimate 49.2
# m/s (110 mph) sweeper reading above without reclassifying real data as noise, so the observed
# 46.5-58 m/s slice of the garbage band (below this ceiling) still passes `_map_v_sane` and can still
# mask a real curve candidate in `upcoming_curve`/`icbm_far_map_candidate` the same way the >=60 m/s
# spikes used to. This is a PARTIAL fix for the primary late-onset stall, not a complete one -- if the
# stall recurs with an observed garbage read in the 46.5-58 m/s range, that is this known gap, not a
# new regression.
MAP_CURVE_V_SANITY_MAX_MS = 58.0   # m/s (~130 mph)


def _map_v_sane(tv) -> bool:
  """icbmonset: True when a raw mapd curve-target velocity (m/s) is finite, positive, and under the
  implausibility ceiling above. Wired into the two candidate scanners (upcoming_curve,
  icbm_far_map_candidate) so a curvature-noise spike is rejected the same way NaN already is, instead
  of being treated as a legitimate candidate. Deliberately NOT used by icbm_map_reach -- see the
  SCOPE note above. Pure; never raises."""
  try:
    tv = float(tv)
  except (TypeError, ValueError):
    return False
  return tv == tv and 0.0 < tv <= MAP_CURVE_V_SANITY_MAX_MS   # tv==tv rejects NaN


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
  stays in icbm_curve_target/_icbm_binding_apex. NaN- and curvature-noise-guarded like upcoming_curve
  (icbmonset: `_map_v_sane` rejects implausible finite reads, not just NaN). Pure."""
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
    if not _map_v_sane(tv) or d != d:    # icbmonset: NaN + curvature-noise guard (mapd emits both live)
      continue
    if not (0.0 < d <= horizon_m):
      continue
    eff = scale_fn(tv) * tv * map_scale
    if eff >= ref - ICBM_MIN_DROP_MS:
      continue                                  # reduce-only: not meaningfully below the ceiling
    a = icbm_approach_decel(v_ego, eff, firm_decel)
    cap = math.sqrt(eff * eff + 2.0 * a * max(d - ICBM_MARGIN_M, 0.0))   # decel envelope from here
    if cap < best_cap:
      best_cap, best_v, best_d = cap, eff, d
  return best_v, best_d


def icbm_map_reach(points, cur_lat, cur_lon, horizon_m=ICBM_MAP_HORIZON_M) -> float:
  """icbmmapfirst2pnw: how far ahead the published mapd path COVERS the road (m) — the farthest
  valid path point within mapd's horizon. 0.0 = no usable coverage (mapd down / no data / GPS lost /
  a stale path we have driven > horizon_m away from). Used by the MAP-FIRST start gate: a vision
  candidate INSIDE this reach is on a stretch the map has judged, so the map verdict (including "no
  slowdown needed") wins for STARTING episodes. Distances are recomputed from the CURRENT position
  every call, so a dead mapd's last path decays out of coverage as we drive on (mapd-liveness
  fallback: vision regains the right to initiate). NaN-guarded like the other scanners.

  icbmonset (POSITION-based only, deliberately NOT `_map_v_sane`-gated — see the reversion note at
  MAP_CURVE_V_SANITY_MAX_MS): reach means "mapd published a point here", independent of what its
  velocity says. pfeiferj's targetVelocity is `sqrt(2/kappa)`-derived: a wide/near-straight node
  (radius > ~1.7 km) legitimately reports a HIGH target velocity, and a straight/unmatched node
  legitimately carries the capnp default 0.0 — both are genuine "no slowdown needed here" verdicts,
  not noise, and gating reach on velocity would silently zero out coverage on ordinary straight
  road, wrongly opening icbm_vision_may_start there (the exact vision-over-slow class the
  2026-07-12 driver rule exists to suppress). Pure."""
  if not points or cur_lat is None or cur_lon is None:
    return 0.0
  reach = 0.0
  for p in points:
    try:
      d = _haversine_m(cur_lat, cur_lon, p["latitude"], p["longitude"])
    except (KeyError, TypeError, ValueError):
      continue
    if d != d:                                    # NaN guard
      continue
    if reach < d <= horizon_m:
      reach = d
  return reach


def icbm_in_curve(lat_accel_now, curve_lat_accel_vision, time_to_curve) -> bool:
  """icbmmapfirst2pnw (driver rule 2): True when the vehicle is ALREADY loaded in a curve — either
  the measured-now lateral accel (model yaw_rate*speed at t~0) is above the CES curve-exit
  hysteresis, or the camera's binding curve is effectively under us (a real vision curve with less
  than the act-window left). While True, no NEW dec episode may start (hold the current set), a
  restore may not BEGIN, and a running restore PAUSES. Defensive: bad input -> False (never blocks
  on garbage — the pre-mapfirst behavior). Pure."""
  try:
    if abs(float(lat_accel_now)) >= ICBM_IN_CURVE_LAT:
      return True
    return (abs(float(curve_lat_accel_vision)) > ICBM_VISION_ENTER
            and float(time_to_curve) < ICBM_VIS_MIN_TTC_S)
  except (TypeError, ValueError):
    return False


def icbm_vision_may_start(vis_dist, time_to_curve, map_reach) -> bool:
  """icbmmapfirst2pnw (driver rule 1): vision may INITIATE a new slow-down episode only when
  (a) the map does NOT cover that stretch (candidate beyond the map's coverage reach — includes
  mapd dead/blind, reach 0.0) AND (b) there is still time to act BEFORE the curve. Running
  episodes are not gated by this (callers apply it only when starting). Pure."""
  try:
    return float(vis_dist) > float(map_reach) and float(time_to_curve) >= ICBM_VIS_MIN_TTC_S
  except (TypeError, ValueError):
    return False


def icbm_curve_target(v_ego, v_set, map_v, map_dist, ceiling, scale_fn,
                      vis_v=0.0, vis_dist=float('inf'),
                      map_scale=1.0, firm_decel=0.0,
                      far_v=0.0, far_dist=float('inf'),
                      track=False):
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

  icbmtrack2pnw: track=True additionally lets MAP/FAR candidates START via the tracking window
  (_icbm_track_apex — set walks down early, e.g. while lead-bound) instead of only the v_ego decel
  envelope. VISION candidates are never tracked. track=False (default) is byte-identical to before.

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
    if a is None and track:
      # icbmtrack2pnw: continuous set-tracking — a binding-RATED map curve within the tracking
      # window starts the walk-down even before the v_ego decel envelope binds (lead-bound case).
      a = _icbm_track_apex(v_ego, ref, eff, map_dist)
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
    if a is None and track:
      a = _icbm_track_apex(v_ego, ref, far_v, far_dist)   # icbmtrack2pnw (far_v already effective)
    if a is not None and (best_apex is None or a < best_apex):
      best_apex, best_src = a, "far"
  if best_apex is not None:
    new_ceiling = ceiling if ceiling is not None else v_set
    return best_apex, new_ceiling, best_src
  # curve cleared (or none): go silent and unlatch immediately. Caps remain DEC-ONLY
  # (Gemini-hardened 2026-07-11); icbmrestore2pnw layers the GUARDED restore as a separate
  # episode phase in IcbmEpisode below — this function itself never commands an increase.
  return None, None, None


# ---------------------------------------------------------------------------
# icbmrestore2pnw — guarded restore episode (driver-requested 2026-07-12)
# ---------------------------------------------------------------------------
ICBM_RESTORE_WINDOW_S = 45.0            # restore may run at most this long after the curve clears
ICBM_EXEC_STEP_MS = 1.0 * 0.44704       # one executor tap = 1 mph (mirror of ford icbm_pnw.STEP_MS)
ICBM_RESTORE_DONE_TOL = 0.6 * ICBM_EXEC_STEP_MS   # within this of the ceiling = restored (matches DEADBAND)
ICBM_DRIVER_LOWER_TOL = 1.7 * ICBM_EXEC_STEP_MS   # stock set below our lowest commanded target by more
                                                  # than this = the DRIVER lowered it -> never restore.
                                                  # icbmmapfirst2pnw: WIDENED 0.6 -> 1.7 steps on field
                                                  # forensics (2026-07-12 18:08:26Z, Snoqualmie->Ellensburg):
                                                  # the truck REPORTS the set speed with ~1 s of lag, so
                                                  # while chasing a falling vision target the executor lands
                                                  # one EXTRA tap after the reported set already met the
                                                  # target (min_target 77.3 mph, own floor 76.0 = 1.3 steps
                                                  # below). The old 0.6 tol misread our OWN late tap as a
                                                  # driver SET- and silently killed the restore — the
                                                  # driver's "no re-acceleration on a straight" complaint
                                                  # (stuck at 76 vs ceiling 85 for 33 s, manual gas+SET+).
                                                  # Worst legit self-overshoot = executor DEADBAND (0.6
                                                  # steps) + ONE report-latency tap (1.0) = 1.6 steps; 1.7
                                                  # covers it with margin. Residual (documented, mirrors
                                                  # RestoreGuard's SET+ residual): ONE driver SET- tap
                                                  # landing inside that band is indistinguishable from our
                                                  # own latency tap and now restores — still bounded by the
                                                  # driver's OWN ceiling, and every abort guard (incl. any
                                                  # set decrease DURING the restore) stays live. Two taps
                                                  # (>= 2 steps) still abort.
ICBM_TAP_PERIOD_S = 0.4                 # executor completes at most one tap per this (PRESS+GAP frames)
ICBM_RESTORE_DELAY_S = 3.0              # the curve must stay CLEAR this long before restore begins —
                                        # flicker-proofing: a 1-tick detection dropout must not start
                                        # pressing SET+ and then re-latch a lower ceiling when the
                                        # curve re-binds (S-curve gaps keep the ORIGINAL ceiling)

# --- icbmmapfirst2pnw (driver-directed rework, drive 2026-07-12 Snoqualmie->Ellensburg) -------------
# Field verdict: vision initiated 60/72 dec ticks (map 9, far 3), slowed too much and INSIDE curves,
# and the map's 500 m anticipatory horizon was under-used. New start policy (mirrors the VTSC
# sharpcurve2pnw shape): MAP-FIRST — with live map coverage over a stretch, the map verdict
# (including "no slowdown needed") is authoritative for STARTING slow-down episodes; vision may only
# INITIATE where the map is blind (beyond its coverage reach, or mapd down — the reach is computed
# from the CURRENT gps distance to the published path points, so a stale path left behind decays out
# of coverage and vision automatically takes back over) AND while there is still time to act. No new
# episode may START while the vehicle is already lateral-loaded in a curve — hold the current set. A
# RUNNING episode is untouched by all of this (it may continue steering the set, any source).
ICBM_VIS_MIN_TTC_S = 2.75        # s; vision may only START an episode with >= this much time to the
                                 #    curve (driver spec: 2.5-3.0 s) — never begin taps at/inside it
ICBM_IN_CURVE_LAT = C.CURVE_LAT_ACCEL_EXIT   # m/s^2 (1.3); measured-now |lat accel| above the CES
                                             #    curve-exit hysteresis = still loaded in a curve
ICBM_APEX_PASS_TTA_S = 1.0       # s; the binding candidate cleared within MARGIN + v*this of us =>
                                 #    we PASSED it (apex behind) — not a detection dropout
ICBM_RESTORE_DELAY_FAST_S = 1.0  # s; early-restore debounce when the curve is provably behind us
                                 #    (drive-out promptly instead of the full 3 s silent hold)
ICBM_HOLD_MAX_S = 10.0           # s; still lateral-loaded this long after the cap cleared with no
                                 #    re-bind -> give up the restore silently (no stale ceiling latch)
ICBM_LATE_TAP_GRACE_S = 1.5      # s after going silent in which the executor's final in-flight tap
                                 #    may still land on the reported set (~1 s report lag + margin)
ICBM_LATE_TAP_TOL = 1.6 * ICBM_EXEC_STEP_MS  # one full late tap + the executor deadband

# --- icbmtrack2pnw (driver-approved follow-up; field event 2026-07-12 19:58:31-59Z) -----------------
# Continuous curve-profile SET-TRACKING for MAP candidates. Driver design, verbatim intent: "adjust
# the target even when I'm behind a slow car; if I'm slower anyway it has no impact; if I go faster
# it slows me; and it gives great debugging in traffic." The field event: following a lead at ~70
# with set 90, a rated map curve 300 m ahead; driver changed lanes, the lead vanished and the stock
# ACC accelerated 72->89 INTO the curve — by then the start was correctly in-curve-suppressed and
# the driver tapped down manually. With tracking, the set would already have been walked down to the
# curve apex while still lead-bound, so losing the lead could only accelerate TO THE APEX.
#   - MAP/FAR candidates: a binding-RATED curve (effective apex < ref - MIN_DROP) starts the cap
#     episode when within the TRACKING WINDOW even if the v_ego decel envelope does not bind yet
#     (drop the v_ego brake-distance precondition; the window is sized so the executor can walk the
#     set down at tap cadence before the curve, computed at the WORST-CASE travel speed = ref, i.e.
#     the speed the ACC would reach if the lead vanished — exactly the protection case).
#   - VISION candidates keep ALL existing stricter gates (short horizon; in-curve / too-late / map-
#     first suppression unchanged). Restore machinery and episode/ceiling semantics unchanged —
#     this only changes WHEN a map cap may start, not what it does. Cap phases have NO expiry (only
#     the RESTORE phase carries the 45 s window), so long lead-bound tracking episodes are safe by
#     construction; the executor governor/stale-stop cadence is episode-length-agnostic.
ICBM_TRACK_MARGIN_S = 4.0     # s of slack beyond the pure tap-walk time (publish latency + set-report lag)
ICBM_TRACK_MAX_M = 350.0      # m hard cap on the tracking window — bounds exit/route-divergence
                              #   tracking (mapd re-matches the path after a divergence and the
                              #   candidates are re-derived from CURRENT GPS every tick, so a wrong
                              #   walk-down self-heals: curve clears -> guarded restore to ceiling)
# 19:58:37Z ROOT CAUSE (the "missing bind"): ICBM map candidates used the raw tiered_map_scale, whose
# SWEEPER end (raw >= 29 m/s -> x1.8, calibrated for VTSC/MTSC where binding causes real braking)
# inflated the event's raw 64.9 mph curve to an effective 107 mph — "not binding vs set 90" was
# computed CORRECTLY on an absurd target, so no cap and no gate ever showed. For ICBM (reduce-only
# SET-walking, no braking below the walked set) cap the scale at the tiered ramp's TIGHT end
# (MAP_SCALE_MIN, 1.35): field-calibrated against BOTH 2026-07-12 legs — 19:58 curve raw 64.9 ->
# eff 80.6 mph (driver manually chose 80-82 there) => binds at set 90; morning sweepers raw 70.9 at
# set 85 -> eff 88 => still silent (the morning over-slow complaint stays fixed). VTSC/MTSC/CES
# classification keep the full tiered scale — this cap is ICBM-only.
#
# icbmcurve2pnw (2026-08-11, docs/ICBM-CURVE-LATE.md Root Cause A / "first cut"): the flat 1.35 cap
# above was itself too COARSE — it applied the SAME ~35% inflation to every map curve, tight or
# moderate, not just sweepers. Field replay of a 2026-08-10 drive found a genuine ~50 mph-rated curve
# (131 m out, 55 mph cruise) that the flat cap inflated to an effective ~62 mph (50 * 1.35 * 0.92
# Lightning map_scale discount = 1.242 net) — ABOVE the 55 mph cruise, so the reduce-only candidacy
# test (icbm_curve_target/_icbm_binding_apex: "eff >= ref - ICBM_MIN_DROP_MS -> reject") discarded the
# candidate outright, before distance/window logic ever ran. All 131 m of published map lead time went
# unused — not a late trigger, a non-trigger. Driver direction (2026-08-10 follow-up): the curve
# TARGET should approximate the real physics limit v = sqrt(a_lat_limit / kappa), not a padded number
# — no separate safety-margin term on top. The fix below restores a genuine TWO-POINT ramp (same
# tight->sweeper SHAPE as the shared tiered_map_scale, per ICBM-CURVE-LATE.md Sec 7.A/7.E) instead of
# a flat cap, but stays ICBM-ONLY — it does NOT modify vtsc_pnw.MAP_SCALE_MIN/MAP_SPEED_SCALE/
# tiered_map_scale, so VTSC/MTSC/Tesla are byte-unchanged by this function:
#   - raw <= ICBM_MAP_SCALE_LO_MPH (50 mph): ICBM_MAP_SCALE_MIN (1.10) — NEAR-RAW. A small, documented
#     correction for mapd/GPS curvature-estimate noise (within the doc's Option-1-recommended
#     1.05-1.1x range), not a padding margin. This is what lets a genuine moderate-cut curve (the
#     50 mph / 55 mph field event) qualify as a candidate AND be tracked/targeted close to its real
#     rating instead of being inflated past the driver's cruise speed.
#   - raw >= ICBM_MAP_SCALE_HI_MPH (60 mph): ICBM_MAP_EFF_SCALE_CAP, still == MAP_SCALE_MIN (1.35),
#     UNCHANGED from the flat cap this replaces. The breakpoint is deliberately kept BELOW both
#     2026-07-12 field-calibrated events so their outcomes stay byte-identical: 64.9 mph raw
#     (29.02 m/s) and 70.9 mph raw (31.7 m/s) both sit above 60 mph -> flat 1.35, exactly as before
#     (64.9 -> eff 80.6 mph, binds at set 90; 70.9 -> eff 88 mph, stays silent at set 85).
#   - linear between 50-60 mph raw.
# Net effect vs. the flat cap: tight/moderate curves (<=50 mph raw) now track close to raw (both
# candidacy AND the published target use this SAME eff value — there is no separate candidacy-only
# scale in this first cut, per ICBM-CURVE-LATE.md Sec 7.A Option 1's "smallest diff, easiest to
# review" recommendation); both field-calibrated sweeper/binding events are unchanged; nothing at or
# above 60 mph raw changes at all. CONSERVATIVE FIRST CUT — pending on-road validation against the new
# steerlimit-log2pnw telemetry (docs/STEERING-LIMITS.md); iterate the two MPH breakpoints and
# ICBM_MAP_SCALE_MIN from there, not by guessing further from a desk analysis.
ICBM_MAP_EFF_SCALE_CAP = MAP_SCALE_MIN            # 1.35 — UNCHANGED, still the sweeper-end ceiling
ICBM_MAP_SCALE_MIN = 1.10                         # near-raw floor for tight/moderate curves (<= LO)
ICBM_MAP_SCALE_LO_MPH = 50.0 * CV.MPH_TO_MS       # ~22.35 m/s — ramp start (near-raw at/below this)
ICBM_MAP_SCALE_HI_MPH = 60.0 * CV.MPH_TO_MS       # ~26.82 m/s — ramp end (flat 1.35 at/above this;
                                                   #   below both 64.9/70.9 mph field-calibrated events)


def icbm_map_eff_scale(tv_raw: float) -> float:
  """icbmtrack2pnw + icbmcurve2pnw: the ICBM-only effective scale for a RAW map target speed (m/s).
  NOT the shared vtsc_pnw.tiered_map_scale (VTSC/MTSC/Tesla are untouched by this function) — a
  two-point linear ramp from ICBM_MAP_SCALE_MIN (near-raw, tight/moderate curves) up to
  ICBM_MAP_EFF_SCALE_CAP (1.35, unchanged sweeper-end cap) between ICBM_MAP_SCALE_LO_MPH and
  ICBM_MAP_SCALE_HI_MPH. See the icbmcurve2pnw root-cause note above. Pure."""
  if tv_raw <= ICBM_MAP_SCALE_LO_MPH:
    return ICBM_MAP_SCALE_MIN
  if tv_raw >= ICBM_MAP_SCALE_HI_MPH:
    return ICBM_MAP_EFF_SCALE_CAP
  frac = (tv_raw - ICBM_MAP_SCALE_LO_MPH) / (ICBM_MAP_SCALE_HI_MPH - ICBM_MAP_SCALE_LO_MPH)
  return ICBM_MAP_SCALE_MIN + (ICBM_MAP_EFF_SCALE_CAP - ICBM_MAP_SCALE_MIN) * frac


def icbm_track_window_m(v_ego, ref, apex_eff) -> float:
  """icbmtrack2pnw: how far ahead a binding-RATED map curve may START the set walk-down. Sized from
  what the executor physically needs: one tap per ICBM_TAP_PERIOD_S walks the set (ref - apex_eff)
  down in steps, plus margin — converted to distance at the WORST-CASE travel speed max(ref, v_ego)
  (after a lead vanishes the ACC accelerates toward ref). Capped at ICBM_TRACK_MAX_M. The window
  self-scales: small set-to-apex drops or low set speeds give short windows (no premature city
  tracking); big highway drops use the full cap. Pure; never negative."""
  try:
    steps = max((float(ref) - float(apex_eff)) / ICBM_EXEC_STEP_MS, 0.0)
    t = steps * ICBM_TAP_PERIOD_S + ICBM_TRACK_MARGIN_S
    return min(max(float(ref), float(v_ego), 0.0) * t, ICBM_TRACK_MAX_M)
  except (TypeError, ValueError):
    return 0.0


def _icbm_track_apex(v_ego, ref, eff, dist):
  """icbmtrack2pnw: TRACKING qualification for a MAP candidate that the v_ego decel envelope does
  not (yet) bind: reduce-only rated (eff meaningfully below ref) AND within the tracking window ->
  the apex speed to walk the set toward. Else None. Same reduce-only/apex semantics as
  _icbm_binding_apex — only the start precondition differs. Pure."""
  if eff is None or dist == float('inf') or eff >= ref - ICBM_MIN_DROP_MS:
    return None
  if dist <= icbm_track_window_m(v_ego, ref, eff):
    return max(eff, 0.0)
  return None


# --- icbmratchet2pnw (root-cause fix, field event 2026-08-10: computed ~31 mph target / ~13.86 m/s,
# ~15 mph delivered) --------------------------------------------------------------------------------
# ROOT CAUSE: IcbmEpisode._min_target only ever ratchets DOWN for the life of a cap episode
# (min(_min_target, cap_target) every tick), so a single transient low tick — e.g. a vision-
# curvature reading (orientationRate.z / icbm_vision_apex) distorted for one ~0.25 s brain tick by
# the truck being momentarily off-line/understeering under the ISO 3.0 clip — was treated exactly
# like a genuine, sustained curve reading and floored the whole episode at that outlier value. The
# VERY NEXT tick's recomputed (correct, ~31 mph) target could never undo it: caps are DEC-only by
# design (icbm_curve_target never commands an increase), and the only path back up is the bounded,
# guarded RESTORE — which only fires once the curve clears ENTIRELY, all the way to the driver's
# original pre-curve ceiling, never to an intermediate "that reading was noise, the real target is
# 31" value. So one bad tick permanently pinned the truck at the outlier speed for the rest of the
# curve.
#
# FIX: a tick that wants to drop the ratchet by more than ICBM_RATCHET_OUTLIER_DROP_MS below the
# LAST CONFIRMED target must PERSIST for ICBM_RATCHET_CONFIRM_S before it is adopted (updates
# _min_target) or published. While a drop is pending confirmation, the episode keeps commanding the
# last CONFIRMED target — still braking normally for whatever curve was already recognized, never
# silently going fully hands-off — just not lurching to the unconfirmed outlier. A tick that
# recovers back within the outlier band of the last confirmed target cancels the pending candidate
# outright: proven transient, never adopted, never published, never tapped toward.
#
# Only large SINGLE-tick drops are gated — ordinary continuous refinement (a curve's estimate
# tightening smoothly tick to tick as distance/geometry closes) is always small and passes straight
# through with no delay, so genuine sustained curves brake exactly as before. A drop this large
# across one ~0.25 s tick (icbm_curve_target/_icbm_step run at ~4 Hz, matching IcbmEpisode.step's
# docstring) is not something real road geometry produces — a candidate curve's RATED speed does
# not change tick to tick, only whether/when it starts binding — so this is a deliberately
# conservative "obviously not a real curve reading" gate, not a general smoothing filter that would
# blunt a genuine hard brake. See IcbmEpisode._ratchet_confirm for the mechanism.
ICBM_RATCHET_OUTLIER_DROP_MS = 3.0   # m/s (~6.7 mph). Defined directly in m/s, NOT as a multiple of
                                      # ICBM_EXEC_STEP_MS (that constant is itself already 1 mph in
                                      # m/s — an earlier draft of this fix wrote "3.0 * ICBM_EXEC_STEP_MS"
                                      # intending 3 m/s and got 3 mph instead, a Fable review catch).
                                      # The field glitch was a 16 mph (7.2 m/s) one-tick drop (31->15),
                                      # far above this — comfortably gates it while staying well clear
                                      # of ordinary tick-to-tick source/tiering jitter (a few mph at
                                      # most).
ICBM_RATCHET_CONFIRM_S = 0.6                             # s; ~2-3 ticks at 4 Hz. Worst-case added
                                                          # travel before a genuine large drop is
                                                          # honored: v_ego * this (e.g. ~24 m at
                                                          # 90 mph) — a small fraction of the existing
                                                          # ICBM_MARGIN_M(30 m) start-early buffer and
                                                          # the (typically hundreds-of-metres) comfort-
                                                          # decel envelope; negligible against the many
                                                          # taps (ICBM_TAP_PERIOD_S=0.4 s each) a real
                                                          # multi-mph slowdown needs anyway.


class IcbmEpisode:
  """Cap -> clear -> RESTORE -> done state machine (pure, unit-tested; owned by CESController).

  DEC remains the rule for CAPS. The restore phase ONLY returns the stock set speed to the driver's
  OWN latched set (the ceiling ICBM itself latched when the first cap of the episode engaged) —
  restore ONLY what ICBM took, never fight the driver:
    - the ceiling is latched exclusively by an ICBM cap engaging (a driver-lowered set never arms it)
    - a restore never publishes a target above the latched ceiling
    - HARD ABORTS (reset -> silent -> executor stale-stops): driver gas/brake; stock ACC off;
      window expiry (ICBM_RESTORE_WINDOW_S); reaching the ceiling; the stock set moving in a way
      our taps can't explain (any decrease, or a rise faster than the executor tap cadence);
      a NEW cap engaging — DEC ALWAYS WINS: the restore episode is cancelled entirely and a fresh
      cap episode latches at the CURRENT set (after a partial restore the new ceiling is the
      partially-restored speed, never the old higher one — conservative on purpose)
    - if the driver lowered the set below anything we commanded during the cap phase, the episode
      ends WITHOUT any restore (their intent, not ours).
  The executor enforces its own independent envelope on top (ceiling clamp, stale heartbeat,
  cruise/override gates, RestoreGuard human-detection latch)."""

  def __init__(self, window_s: float = ICBM_RESTORE_WINDOW_S, clear_delay_s: float = ICBM_RESTORE_DELAY_S):
    self._window_s = window_s
    self._clear_delay_s = clear_delay_s
    self.phase = "idle"                 # idle | cap | restore
    self.ceiling = None                 # driver's own set (m/s), latched while an episode is active
    self._min_target = None             # lowest CONFIRMED cap target commanded this episode
    # icbmratchet2pnw: a candidate that would drop _min_target by more than an outlier-sized step
    # in one tick, awaiting confirmation (see _ratchet_confirm) before it is adopted/published.
    self._pending_low_target = None
    self._pending_low_t0 = None         # monotonic time the pending candidate was first seen
    # icbmratchet2pnw: when the CURRENT ceiling/_min_target was (re)latched at engage (Gemini review
    # catch) — an engage that goes silent again before this proves itself for ICBM_RATCHET_CONFIRM_S
    # is treated as unconfirmed noise, not a real curve; see step()'s clear-debounce entry.
    self._engage_t0 = None
    self._t0 = None                     # restore start (monotonic)
    self._clear_t0 = None               # first tick the curve was clear while capping (debounce)
    self._hold_set0 = None              # stock set snapshot at hold entry (movement = human)
    self._last_stock = None
    self._last_t = None
    # icbmmapfirst2pnw: where the binding candidate was when it last bound — a candidate that clears
    # while CLOSE to us was PASSED (apex behind -> early restore); one that clears while far ahead
    # was a detection dropout (keep the full flicker-proof debounce).
    self._last_cap_dist = None
    self._last_cap_vego = 0.0
    self._apex_passed = False
    self._late_tap_set = None           # the ONE absorbed late-tap baseline (anchored, never walked)

  def reset(self) -> None:
    self.phase = "idle"
    self.ceiling = None
    self._min_target = None
    self._pending_low_target = None
    self._pending_low_t0 = None
    self._engage_t0 = None
    self._t0 = None
    self._clear_t0 = None
    self._hold_set0 = None
    self._last_stock = None
    self._last_t = None
    self._last_cap_dist = None
    self._last_cap_vego = 0.0
    self._apex_passed = False
    self._late_tap_set = None

  def _ratchet_confirm(self, now: float, cap_target: float, baseline: float) -> float:
    """icbmratchet2pnw: the robustness gate on the DOWNWARD ratchet. `baseline` is the reference an
    outlier-sized drop is measured against — self._min_target (the last CONFIRMED target) for an
    already-active cap episode, or the driver's own v_set for the very first tick of a fresh
    episode (there is no confirmed target yet; a Gemini/Fable review catch on the first version of
    this fix found that skipping confirmation entirely at engage let a single bad engage tick latch
    a ceiling/min_target that a subsequent immediate clear-and-reset (see step()'s clear-debounce
    handling of self._engage_t0) could not always undo before it reached the executor). Returns the
    target THIS tick should actually command/ratchet toward:
      - cap_target itself, immediately, when it is not an outlier-sized drop below `baseline` — the
        common case: flat, rising, or an ordinary small/gradual decrease. _min_target is updated
        (ratcheted down, never up; initialized on first use) exactly as before the fix.
      - the last CONFIRMED target (self._min_target, or `baseline` if nothing confirmed yet), while
        an outlier-sized drop is pending confirmation — never act on an unconfirmed low reading.
      - the pending candidate (the lowest value seen while it was pending), once an outlier-sized
        drop has persisted for >= ICBM_RATCHET_CONFIRM_S — a genuinely sustained lower target is
        adopted and _min_target ratchets down to it, same as the pre-fix behavior but proven, not
        assumed.
    A recovery back within the outlier band of `baseline` (checked every tick via the first branch)
    discards any pending candidate outright — a transient never gets a "grace" tap. A reading that
    ITSELF deviates from the currently-pending candidate by more than the outlier band (Gemini
    review catch: an extreme one-tick outlier landing inside an otherwise-legitimate pending window
    must not "hide" there and get adopted via the worst-seen min()) restarts the confirmation window
    at the new value instead of blending into the old one — noisy/unstable readings simply take
    longer to confirm, they never let one wild tick hijack a window opened by a different value."""
    held = self._min_target if self._min_target is not None else baseline
    if cap_target >= baseline - ICBM_RATCHET_OUTLIER_DROP_MS:
      self._pending_low_target = None
      self._pending_low_t0 = None
      self._min_target = cap_target if self._min_target is None else min(self._min_target, cap_target)
      return cap_target
    # outlier-sized drop vs baseline: hold at the last confirmed (or baseline) value until this (or
    # a lower) reading persists ICBM_RATCHET_CONFIRM_S. Track the WORST (lowest) value seen among
    # readings CONSISTENT with each other during the window — conservative on purpose (never less
    # braking than warranted), consistent with the rest of the file's reduce-only bias.
    if (self._pending_low_t0 is None
        or abs(cap_target - self._pending_low_target) > ICBM_RATCHET_OUTLIER_DROP_MS):
      self._pending_low_target = cap_target
      self._pending_low_t0 = now
      return held
    self._pending_low_target = min(self._pending_low_target, cap_target)
    if now - self._pending_low_t0 >= ICBM_RATCHET_CONFIRM_S:
      confirmed = self._pending_low_target
      self._min_target = confirmed
      self._pending_low_target = None
      self._pending_low_t0 = None
      return confirmed
    return held

  def step(self, now, cap_target, v_set, stock_set, stock_on, driver_pedal,
           cap_dist=None, v_ego=0.0, in_curve=False):
    """One brain tick (~4 Hz). All inputs SI primitives; cap_target is the (penalty-applied) cap
    from icbm_curve_target or None. Returns (publish_target or None, direction 'dec'/'inc'/None).

    icbmmapfirst2pnw optional inputs (defaults keep the pre-mapfirst behavior identical):
      cap_dist  distance (m) of the binding candidate this tick (None/inf = unknown) — feeds the
                apex-passage detection for the EARLY restore (curve provably behind -> 1 s debounce
                instead of 3 s);
      v_ego     current speed (m/s), for the apex-passage window;
      in_curve  vehicle currently lateral-loaded (icbm_in_curve): a restore may not BEGIN and a
                running restore PAUSES (silent — executor stale-stops — WITHOUT resetting, so it
                resumes when the load clears) while True. Never raise the set mid-curve. All abort
                guards stay live throughout."""
    if v_set is None or v_set <= 0.0:
      self.reset()                      # no valid driver set -> hands off everything
      return None, None
    if not stock_on or driver_pedal:
      # Gemini adversarial catch (ACC off/on survival): ANY pedal press or ACC-off in ANY phase
      # kills the episode entirely — no episode may exist while the driver is braking/gassing or
      # the ACC is disengaged. A cap present right now is still FORWARDED (dec-only; the executor
      # independently gates presses on cruise/pedals), but WITHOUT an episode/latch: when
      # conditions return, the next cap tick starts a FRESH episode latched at the THEN-current
      # set — so a post-brake re-engage at a lower set can never be "restored" to the old ceiling.
      self.reset()
      if cap_target is not None:
        return cap_target, "dec"
      return None, None
    if cap_target is not None:
      cap_target = float(cap_target)
      # DEC ALWAYS WINS. A cap during RESTORE cancels the restore episode entirely and re-latches
      # at the CURRENT set; a cap during CAP just continues the episode (ceiling untouched).
      if self.phase != "cap":
        self.reset()
        self.phase = "cap"
        self.ceiling = float(v_set)
        self._min_target = cap_target
        self._engage_t0 = now             # icbmratchet2pnw: see the clear-debounce handling below
        publish_target = cap_target
      else:
        # icbmratchet2pnw: an outlier-sized single-tick drop is held pending confirmation instead
        # of ratcheting (and publishing) immediately — see _ratchet_confirm / the constants above.
        # baseline: the last CONFIRMED target, or (nothing confirmed yet — a fresh episode whose
        # very first tick was itself never validated) the driver's current set.
        baseline = self._min_target if self._min_target is not None else float(v_set)
        publish_target = self._ratchet_confirm(now, cap_target, baseline)
      self._clear_t0 = None             # curve (re)bound: reset the clear debounce
      # icbmmapfirst2pnw: remember where the binding candidate sits — used at clear to tell
      # "passed the curve" (early restore) from "detection dropout" (full debounce).
      try:
        self._last_cap_dist = float(cap_dist) if (cap_dist is not None and cap_dist == cap_dist
                                                  and cap_dist != float('inf')) else None
        self._last_cap_vego = max(float(v_ego), 0.0)
      except (TypeError, ValueError):
        self._last_cap_dist = None
        self._last_cap_vego = 0.0
      return publish_target, "dec"

    if self.phase == "cap":
      # clear DEBOUNCE: hold silent (ceiling retained, executor stale-stops within 2 s) until the
      # curve has stayed clear for clear_delay_s. A detection flicker or an S-curve gap therefore
      # keeps the ORIGINAL ceiling instead of starting a restore and re-latching lower.
      if self._clear_t0 is None:
        # icbmratchet2pnw (Gemini review catch): the engage that latched the CURRENT ceiling/
        # _min_target never had a chance to prove itself for ICBM_RATCHET_CONFIRM_S before the
        # candidate vanished again — an engage-then-immediate-clear is exactly the single-tick-
        # outlier signature this whole fix targets, just at the FIRST tick instead of a later one
        # (where _ratchet_confirm already catches it). Without this, a bad engage tick's ceiling/
        # _min_target could survive the clear and get reused — including by a later, unrelated,
        # genuinely-binding candidate reusing this SAME (unconfirmed, wrong) episode — because caps
        # are DEC-only: once a low value is tapped, only the fully separate, much slower guarded
        # RESTORE can undo it, never an in-episode recompute. Wipe the episode outright so any later
        # rebind starts completely fresh instead of resuming an unproven one.
        if self._engage_t0 is not None and (now - self._engage_t0) < ICBM_RATCHET_CONFIRM_S:
          self.reset()
          return None, None
        self._clear_t0 = now
        self._hold_set0 = stock_set     # snapshot: we go SILENT now, so nothing of ours moves the set
        self._late_tap_set = None       # fresh clear window: any previous absorbed tap is void
        # icbmratchet2pnw (Fable review catch): a candidate awaiting confirmation must NOT survive
        # a gap in cap ticks. _ratchet_confirm measures WALL TIME, not contiguous low readings, so
        # without this an old pending value (e.g. a glitch tick just before a brief detection
        # dropout) could sit stale through the gap and then get instantly "confirmed" — using
        # elapsed time that includes the silent gap — the moment ANY new, unrelated, merely
        # outlier-sized-but-legitimate candidate rebinds, via the worst-seen min(). Void it here so
        # a rebind after any gap always starts its own fresh confirmation window.
        self._pending_low_target = None
        self._pending_low_t0 = None
        # icbmmapfirst2pnw: apex passage — the binding candidate vanished while within the tap
        # margin + ~1 s of travel of us => we drove past it (curve behind), not a dropout.
        self._apex_passed = (self._last_cap_dist is not None
                             and self._last_cap_dist <= ICBM_MARGIN_M + self._last_cap_vego * ICBM_APEX_PASS_TTA_S)
      # icbmmapfirst2pnw late-tap absorption (field false positive, 2026-07-12 18:08:26Z): the truck
      # reports the set with ~1 s lag, so the executor's FINAL in-flight tap can land AFTER we went
      # silent. Within the grace window, ONE small DOWNWARD move (<= one tap + deadband, measured
      # from the ORIGINAL snapshot) is our own tap, not a human — record it as an ALTERNATE
      # baseline. ANCHORED, adopted at most once, never walked (Gemini adversarial catch: walking
      # the baseline would let a driver's repeated SET- taps be absorbed one step at a time).
      # Upward movement is never absorbed; a second downward step lands below BOTH baselines and
      # the movement guard blocks the restore exactly as before.
      if (stock_set is not None and self._hold_set0 is not None
          and self._late_tap_set is None
          and (now - self._clear_t0) <= ICBM_LATE_TAP_GRACE_S
          and 0.0 < self._hold_set0 - stock_set <= ICBM_LATE_TAP_TOL):
        self._late_tap_set = stock_set
      # icbmmapfirst2pnw early restore (driver rule 3): when the curve is provably BEHIND us, begin
      # the restore after a short drive-out instead of the full flicker-proof hold. A dropout-style
      # clear (candidate still far ahead) keeps the original 3 s debounce unchanged.
      delay = min(self._clear_delay_s, ICBM_RESTORE_DELAY_FAST_S) if self._apex_passed else self._clear_delay_s
      if (now - self._clear_t0) < delay:
        return None, None
      if in_curve:
        # still lateral-loaded (e.g. long curve, or the NEXT bend of an S): hold silently — never
        # BEGIN raising the set mid-curve. Bounded: loaded too long with no re-bind -> give up.
        if (now - self._clear_t0) > ICBM_HOLD_MAX_S:
          self.reset()
        return None, None
      # curve cleared (sustained) -> enter RESTORE only when the episode is cleanly ours:
      eligible = (stock_on and not driver_pedal and self.ceiling is not None
                  and stock_set is not None and stock_set > 0.0
                  # the current set is explainable by OUR taps — if the driver went lower than the
                  # lowest target we ever commanded, restoring would fight their intent: don't.
                  and (self._min_target is None or stock_set >= self._min_target - ICBM_DRIVER_LOWER_TOL)
                  # Gemini adversarial catch (the 3 s blind spot): the brain is SILENT through the
                  # hold, so the executor does nothing — ANY set movement across the hold window is
                  # a HUMAN choosing a speed. Movement (either direction) -> no restore at all.
                  # icbmmapfirst2pnw: the set may alternatively match the ONE absorbed late-tap
                  # baseline (its own executor tap landing after silence) — nothing else.
                  and self._hold_set0 is not None
                  and (abs(stock_set - self._hold_set0) <= ICBM_RESTORE_DONE_TOL
                       or (self._late_tap_set is not None
                           and abs(stock_set - self._late_tap_set) <= ICBM_RESTORE_DONE_TOL))
                  # something to restore (not already at/above the ceiling)
                  and stock_set < self.ceiling - ICBM_RESTORE_DONE_TOL)
      if eligible:
        self.phase = "restore"
        self._t0 = now
        self._last_stock = stock_set
        self._last_t = now
        return self.ceiling, "inc"
      self.reset()
      return None, None

    if self.phase == "restore":
      # hard aborts / completion — any of these ends the episode entirely (unlatch, go silent)
      if (not stock_on or driver_pedal or stock_set is None or stock_set <= 0.0
          or (now - self._t0) > self._window_s
          or stock_set >= self.ceiling - ICBM_RESTORE_DONE_TOL):
        self.reset()
        return None, None
      # brain-side manual-intervention detection (the executor's RestoreGuard is the fine-grained
      # one; this catches it independently at the 4 Hz brain cadence):
      if self._last_stock is not None:
        dt = max(now - (self._last_t if self._last_t is not None else now), 0.0)
        if stock_set < self._last_stock - ICBM_RESTORE_DONE_TOL:
          self.reset()                  # set went DOWN: only a human does that during restore
          return None, None
        if stock_set > self._last_stock + ICBM_EXEC_STEP_MS * (dt / ICBM_TAP_PERIOD_S + 1.6):
          self.reset()                  # rose faster than our taps can: driver holding SET+
          return None, None
      self._last_stock = stock_set
      self._last_t = now
      if in_curve:
        # icbmmapfirst2pnw: PAUSE while lateral-loaded (a late-seen next bend) — go silent so the
        # executor stale-stops, but keep the episode so the restore resumes once the load clears.
        # All the aborts above (pedal/ACC/window/decrease/fast-rise) ran this tick and stay live.
        return None, None
      return self.ceiling, "inc"

    return None, None                   # idle, no cap


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
  lookahead distance (v_ego * lookahead_s). Returns (0.0, inf) if none / no data. Pure & testable.

  icbmonset: a point failing `_map_v_sane` (NaN, seen live 2026-07-11, OR a physically-implausible
  finite spike, seen live 2026-07-18 — mapd's curvature calc misfiring at curve entry) is skipped
  exactly like the pre-existing NaN case. Shared by CES's own curve trip (decide_active/
  curve_closeness, both CES1 and the ces2_core shadow, all cars) as well as ICBM's near-window
  candidate: provably a no-op for every existing decision there, since a value this high already
  fails every 'is this curve binding' comparison identically to 'no candidate' (v_ego - tv is always
  very negative, tv*scale is always far above any sharp-curve/binding threshold) — only the raw
  telemetry (mapV/curvePct) stops showing the nonsense number."""
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
    if not _map_v_sane(tv) or d != d:   # icbmonset: NaN + curvature-noise guard
      continue
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

  def force(self):
    """stopintent2pnw: charge the debounce to 'fully active' — used ONLY by the stop-intent fast
    path so the EXIT side behaves exactly as after a normal entry (the condition must genuinely
    clear and decay before Chill is considered; the return path keeps its full anti-flap dwell)."""
    self.f.x = 1.0
    self.active = True


def _lead_pull_away(s) -> bool:
  """pullaway2pnw (PURE): evidence-gated pull-away exception BELOW the redlight2pnw floor
  (ACCEL_ZONE_MIN_V). True only when ALL hold — any failure means the floor stands exactly as
  before (incident 2026-07-12 ~14:0x PT: ~17 mph behind a pulling-away lead, CES held Experimental
  and the truck lost the lead; driver rule: "the lead car cannot pull away"):
    (a) a lead is PRESENT with dRel in the sane 5-60 m band,
    (b) the lead is genuinely OPENING: at least PULLAWAY_DV faster than ego AND s["lead_opening"]
        (the PullAwayTracker's 3-spaced-sample monotonic dRel rise, which also enforces the
        model-stop recency guard — the yellow-light trap),
    (c) the model does NOT want to stop this cycle (the exact signal the red-light guard keys on),
    (d) ego is actually moving (>= PULLAWAY_MIN_V, never from standstill) and below the floor
        (above it, nothing changes — byte-identical).
  Callers that do not supply "lead_opening" (older pure tests) get False -> today's behavior."""
  return (bool(s["has_lead"])
          and not s.get("model_should_stop")
          and C.PULLAWAY_MIN_V <= s["v_ego"] < C.ACCEL_ZONE_MIN_V
          and C.PULLAWAY_DREL_LO <= s["lead_drel"] <= C.PULLAWAY_DREL_HI
          and (s["lead_vlead"] - s["v_ego"]) >= C.PULLAWAY_DV
          and bool(s.get("lead_opening", False)))


class PullAwayTracker:
  """pullaway2pnw: the STATEFUL evidence half of the pull-away exception (pure, unit-tested;
  owned by CESController). Produces the s["lead_opening"] bool from per-cycle observations:
    - dRel must RISE monotonically (>= PULLAWAY_OPEN_EPS per step) across PULLAWAY_SAMPLES
      samples spaced >= PULLAWAY_SAMPLE_GAP_S apart (a real, sustained opening — not radar noise);
    - a lead loss, dRel jump (> PULLAWAY_JUMP_M, lead swap / radar reacquire) restarts the
      evidence from scratch;
    - the model must not have wanted to stop within PULLAWAY_STOP_CLEAR_S (recency guard: a
      shouldStop flicker while a lead clears a yellow light must keep blocking after it clears)."""

  def __init__(self):
    self._hist = []            # [(t, dRel)] newest last, at most PULLAWAY_SAMPLES entries
    self._last_stop_t = None   # last time the model wanted to stop

  def update(self, now, has_lead, d_rel, model_should_stop) -> bool:
    if model_should_stop:
      self._last_stop_t = now
    if not has_lead or d_rel is None or d_rel <= 0.0:
      self._hist = []          # no (valid) lead: evidence dies with it
      return False
    if self._hist and abs(float(d_rel) - self._hist[-1][1]) > C.PULLAWAY_JUMP_M:
      self._hist = []          # discontinuity: different lead / radar reacquire
    if not self._hist or (now - self._hist[-1][0]) >= C.PULLAWAY_SAMPLE_GAP_S:
      self._hist.append((now, float(d_rel)))
      self._hist = self._hist[-C.PULLAWAY_SAMPLES:]
    opening = (len(self._hist) == C.PULLAWAY_SAMPLES
               and all(self._hist[i + 1][1] - self._hist[i][1] >= C.PULLAWAY_OPEN_EPS
                       for i in range(C.PULLAWAY_SAMPLES - 1)))
    stop_recent = (self._last_stop_t is not None
                   and (now - self._last_stop_t) < C.PULLAWAY_STOP_CLEAR_S)
    return opening and not stop_recent


def _accelerate_zone_base(s) -> bool:
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


def _accelerate_zone(s) -> bool:
  """pullaway2pnw: the accelerate-zone is the UNCHANGED base (redlight2pnw semantics, floor and
  all) OR the evidence-gated lead-pull-away exception below the floor. Above the floor and in
  every no-lead case this is byte-identical to _accelerate_zone_base."""
  return _accelerate_zone_base(s) or _lead_pull_away(s)


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

  # 2) stop light / stop sign — model predicts a stop, not currently following a lead.
  # stophold2pnw (A1, red-light lurch 2026-07-12 21:47:08Z): below STOP_HOLD_MAX_V the `not
  # has_lead` mask is LIFTED — stopped/creeping behind a lead at a light, the model's stop intent
  # must count (the original mask exists so lead-following decel AT SPEED doesn't trip
  # Experimental; at a creep the LIGHT governs, not the lead). This also re-arms the stopIntent
  # fast path in exactly the lurch geometry (raw_active becomes True, so a Chill machine re-enters
  # in one cycle when shouldStop asserts). Fail-safe: only ever KEEPS/ENTERS Experimental.
  if t["stops"] and s["model_should_stop"] and (not s["has_lead"] or v < C.STOP_HOLD_MAX_V):
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

  # pullaway2pnw telemetry: when the ONLY thing keeping us out of the lowSpeed Experimental hold
  # is the pull-away exception (base accel-zone would NOT fire), name the reason so field
  # validation reads straight off ces_events / the overlay `why` line.
  if (t["low_speed"] and 1.0 <= v < thr and not on_highway
      and not _accelerate_zone_base(s) and _lead_pull_away(s)):
    return False, "pullAway"
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


def lead_metrics(has_lead: bool, d_rel: float, v_lead: float, v_ego: float) -> tuple[float, float]:
  """PURE (vtsctele2pnw): lead-follow metrics for telemetry — (gap seconds, lead speed delta m/s).
  gap = dRel/vEgo rounded to 1 decimal (0.0 when no lead or near-stopped, so a logged 0.0 always
  means 'not following'); delta = vLead - vEgo (positive = lead pulling away, negative = closing).
  Display/logging only — never gates control."""
  if not has_lead:
    return 0.0, 0.0
  gap = round(float(d_rel) / float(v_ego), 1) if float(v_ego) > 0.5 else 0.0
  return gap, round(float(v_lead) - float(v_ego), 1)


def decision_telemetry(s) -> dict:
  """PURE, display-only: a compact snapshot for the on-screen CES overlay. Reports the binding
  reason, the curve 'closeness' as a 0..100 %, and the upcoming map-curve preview (target speed +
  distance). Built from the SAME signals dict `decide_active` consumes, so the overlay can never
  disagree with the live decision."""
  raw_active, reason = decide_active(s)
  cpct, csrc = curve_closeness(s)
  md = s["map_target_dist"]
  gap_s, d_v = lead_metrics(s["has_lead"], s["lead_drel"], s["lead_vlead"], s["v_ego"])
  return {
    "rawActive": bool(raw_active),
    "reason": reason,
    "mdlEndX": round(float(s.get("mdl_end_x", 0.0)), 1),   # ces2-study replay dataset (log-only)
    "curvePct": int(round(cpct * 100)),
    "curveSrc": csrc,
    "mapV": round(float(s["map_target_v"]), 1),
    "mapDist": round(float(md), 0) if md != float('inf') else 0.0,
    "vEgo": round(float(s["v_ego"]), 1),
    # speedlimitdebug2pnw (driver req 2026-07-16): surface the OSM speed limit in the overlay feed
    # too (it was already logged per-event via self._speed_limit, just never in the live CESStatus
    # dict the on-screen box reads) so the overlay can flash it on a real change.
    "spdLim": round(float(s.get("spd_lim", 0.0)), 1),
    # accelerate-zone + the signals that drive it (also logged per event for later tuning)
    "accelZone": _accelerate_zone(s),
    "hwyGate": s.get("spd_lim", 0.0) >= C.LOWSPEED_HWY_GATE,   # lowSpeed suppressed: on a highway
    "vSet": round(float(s["v_set"]), 1),
    "dRel": round(float(s["lead_drel"]), 0),
    "vLead": round(float(s["lead_vlead"]), 1),
    # vtsctele2pnw: explicit lead-present bool (radarState.leadOne.status — a logged dRel of 0.0 is
    # ambiguous: 'no lead' vs 'lead at 0 m'), gap time (s) and lead speed delta (m/s), so
    # LongitudinalExt / curve-entry forensics stop inferring lead state from dRel>0.
    "lead": bool(s["has_lead"]),
    "gapS": gap_s,
    "dV": d_v,
    "aEgo": round(float(s.get("a_ego", 0.0)), 2),
    "gas": bool(s.get("gas", False)),
    # lowspeedcurve2pnw telemetry-first discovery (Hwy 99 2026-07-13 field issue #5: curvePct
    # stuck at 0 all drive): the RAW vision-curve inputs, so the next city drive shows WHICH gate
    # held it — weak predicted lat-accel (visLat vs CURVE_LAT_ACCEL_ENTER), the lookahead window
    # (visTtc vs CURVE_VISION_LOOKAHEAD_S), or blinker suppression (blnk True on signaled turns).
    "visLat": round(float(s.get("curve_lat_accel_vision", 0.0) or 0.0), 2),
    "visTtc": round(float(s.get("time_to_curve", 0.0) or 0.0), 1),
    "blnk": bool(s.get("blinker", False)),
    # stophold2pnw (B): the RAW per-cycle model stop intent (modelV2.action.shouldStop), NOT
    # debounced — this is the exact signal A1/A2/stopIntent/pullaway key on, and its absence from
    # the breadcrumb is why the 21:47:08Z lurch forensics could not tell red from green.
    "stp": bool(s.get("model_should_stop", False)),
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
    # stophold2pnw (A2): seconds model_should_stop has been CONTINUOUSLY clear. Initialized to the
    # hold threshold ("clear long ago") so a fresh machine never spuriously holds.
    self._stop_clear_s = C.STOP_CLEAR_HOLD_S
    # standstill2pnw (see the constants block for the field basis + design):
    self._ss_lead_s = 0.0        # s of continuous lead presence at standstill (promotion debounce)
    self._at_standstill = False  # True while Experimental at v < STANDSTILL_LATCH_V (latch active)
    self._ss_close_lead = False  # a lead was within STANDSTILL_RELEASE_DREL during the standstill
                                 #   (sticky across radar dropouts — the field flip ticks all show
                                 #   lead=False; only a SEEN gap > CLEAR_DREL clears it)
    self._release_hold = False   # armed at the release tick when _ss_close_lead; holds Experimental
                                 #   until v > STANDSTILL_RELEASE_V or the gap opens past CLEAR_DREL
    # cesnochill2pnw: True while the pure-v_ego Schmitt-trigger latch is armed (see the constants
    # block) — starts False (a fresh machine at cruise is not "stopping"; the first genuine
    # decel-to-a-stop arms it on its own from live v_ego).
    self._nochill_armed = False

  def reset(self):
    self._cond.reset()
    self._is_experimental = False
    self._dwell = 0.0
    self._status = "chill"
    self._stop_clear_s = C.STOP_CLEAR_HOLD_S  # stophold2pnw (A2)
    self._ss_lead_s = 0.0                     # standstill2pnw
    self._at_standstill = False
    self._ss_close_lead = False
    self._release_hold = False
    self._nochill_armed = False               # cesnochill2pnw

  def mode(self) -> str:
    return "experimental" if self._is_experimental else "chill"

  def status(self) -> str:
    return self._status

  def update_decision(self, signals: dict, dt: float = DT_CTRL) -> str:
    """Public entry point: run the full decision core, then apply the cesnochill2pnw hard latch as
    a FINAL override so it wins over every internal path (dwell expiry, A2, filter decay, a
    model_should_stop flicker, or any future addition) — closing the ordering gap that let a
    transient `chill` decision through anywhere in the stopping/stopped speed band (see the
    constants block for the field incident, the round-2 a_ego revert, and the wedge-impossibility
    proof). Behavior-neutral once genuinely moving away: the latch is a no-op there and the
    unmodified core governs exactly as before."""
    self._update_decision_core(signals, dt)
    self._apply_nochill_latch(float(signals.get("v_ego", 0.0)))
    return self.mode()

  def _apply_nochill_latch(self, v_now: float) -> None:
    """cesnochill2pnw: PURE v_ego Schmitt-trigger latch — no acceleration term (round 2's a_ego
    direction gate was reverted: Gemini review found it could wedge Experimental permanently on a
    gentle/leveling-off launch, and flapped on a_ego noise near the release band). One bit of
    armed/not memory, re-evaluated every tick from live v_ego alone — see the constants block for
    the full ARM/RELEASE spec and the wedge-impossibility proof (RELEASE depends on v_ego ALONE, so
    any tick above NOCHILL_RELEASE_V clears it unconditionally — a real launch cannot avoid producing
    such a tick). While armed, status is UNCONDITIONALLY "stopLatch" for the whole episode (Gemini
    review, round 1: avoids flapping between a stale core-computed reason and the latch tag every
    time the core's own dwell machinery cycles underneath) — `_dwell` is never touched here."""
    if self._nochill_armed:
      if v_now > C.NOCHILL_RELEASE_V:
        self._nochill_armed = False
    elif v_now < C.NOCHILL_ARM_V:
      self._nochill_armed = True
    if self._nochill_armed:
      self._is_experimental = True
      self._status = "stopLatch"   # cesnochill2pnw telemetry tag: unconditional for the whole hold

  def _update_decision_core(self, signals: dict, dt: float = DT_CTRL) -> str:
    """Advance the state machine one cycle from an extracted `signals` dict (see decide_active).
    `dt` is the MEASURED loop period (selfdrived runs at 100 Hz) so the dwell/debounce are real
    seconds. Separated from `update(sm)` so it is unit-testable without cereal messages."""
    raw_active, status = decide_active(signals)
    v_now = float(signals.get("v_ego", 0.0))
    has_lead = bool(signals.get("has_lead", False))
    d_rel = float(signals.get("lead_drel", 0.0) or 0.0)
    # stophold2pnw (A2): track how long the model's stop intent has been continuously clear —
    # updated FIRST so the timer is correct on every path out of this function (incl. the
    # stopIntent fast-path early return below). Capped at the threshold (no unbounded float).
    if bool(signals.get("model_should_stop")):
      self._stop_clear_s = 0.0
    else:
      self._stop_clear_s = min(self._stop_clear_s + dt, C.STOP_CLEAR_HOLD_S)
    # standstill2pnw: sustained-lead evidence at standstill (the promotion debounce) — updated
    # FIRST like _stop_clear_s so it is correct on every path. A lead DROPOUT resets it (strict
    # continuity: a single-tick radar ghost can never charge it). Capped at the threshold.
    if v_now < C.STANDSTILL_LATCH_V and has_lead:
      self._ss_lead_s = min(self._ss_lead_s + dt, C.STANDSTILL_PROMOTE_LEAD_S)
    else:
      self._ss_lead_s = 0.0
    # stopintent2pnw (driver-approved): ABSOLUTE stop-intent fast path. When the model's stop
    # intent (model_should_stop — the exact signal the red-light guard keys on) asserts AND the
    # decision ladder wants Experimental, entering Experimental bypasses EVERYTHING on the entry
    # side: the CHILL_MIN_DWELL_S re-entry cooldown, the ~1 s condition filter charge, and any
    # accel-zone adoption (the az is already dead while shouldStop by the existing gate). Churn
    # TOWARD stopping is the safe direction and is exempt from anti-flap; the RETURN to Chill
    # keeps the full normal dwell (filter force() + untouched exit path = the asymmetry).
    # This closes the pullaway2pnw occlusion trap (Gemini STOP, 2026-07-12: a departing lead
    # occludes a red light; shouldStop asserts only AFTER a pull-away Chill adoption — the 5 s
    # cooldown then held Chill through the intersection) and the PRE-EXISTING 5 s stop-blind
    # window after ANY accel-zone adoption. Respects the per-condition "stops" toggle; the
    # driver's forced-Chill button still wins upstream (it never reaches this state machine).
    if (not self._is_experimental and raw_active
        and bool(signals.get("model_should_stop"))
        and bool(signals.get("toggles", {}).get("stops", True))):
      self._is_experimental = True
      self._status = "stopIntent"      # telemetry tag: every fast-path preemption is visible
      self._dwell = 0.0
      self._cond.force()               # exit side sees a fully-charged condition (normal semantics)
      self._at_standstill = v_now < C.STANDSTILL_LATCH_V   # standstill2pnw: latch state from entry
      self._release_hold = False
      self._ss_close_lead = False
      return self.mode()
    # standstill2pnw PROMOTE: at standstill in Chill with the ladder wanting Experimental (only
    # slowLead / stop can be raw-active at v~0 — lowSpeed has a 1.0 floor, curve needs
    # CRUISING_SPEED), enter WITHOUT the CHILL_MIN_DWELL_S cooldown or the ~1 s filter charge —
    # at 0 mph those anti-flap timers were FIGHTING the trigger (the 11:34-11:38Z flapping), and
    # Experimental at standstill is strictly safer. Gated on STANDSTILL_PROMOTE_LEAD_S of
    # CONTINUOUS lead presence instead (radar-ghost debounce); the latch below then makes the
    # promoted state absorbing at standstill, so chill<->exp oscillation is structurally impossible
    # there. Exit semantics stay normal (force() = fully-charged condition, full return dwell).
    if (not self._is_experimental and raw_active
        and v_now < C.STANDSTILL_LATCH_V
        and self._ss_lead_s >= C.STANDSTILL_PROMOTE_LEAD_S):
      self._is_experimental = True
      self._status = status            # the real trigger (slowLead/stop) — meaningful in the logs
      self._dwell = 0.0
      self._cond.force()
      self._at_standstill = True
      self._release_hold = False
      self._ss_close_lead = has_lead and d_rel <= C.STANDSTILL_RELEASE_DREL
      return self.mode()
    cond_active = self._cond.update(raw_active, dt)   # debounced (real-time)
    self._dwell += dt

    if not self._is_experimental:
      # enter Experimental once the debounced condition is active AND we've been in Chill at least
      # the re-entry cooldown (de-flap: stops the instant snap-back that caused the stop&go sawtooth)
      if cond_active and self._dwell >= self._chill_min:
        self._is_experimental = True
        self._status = status
        self._dwell = 0.0
        self._at_standstill = v_now < C.STANDSTILL_LATCH_V   # standstill2pnw: latch state from entry
        self._release_hold = False
        self._ss_close_lead = has_lead and d_rel <= C.STANDSTILL_RELEASE_DREL and self._at_standstill
    else:
      # standstill2pnw: maintain the latch / release-hold state EVERY Experimental tick (not only
      # when an exit is eligible — the release tick can land while a condition is still charged).
      if v_now < C.STANDSTILL_LATCH_V:
        self._at_standstill = True
        self._release_hold = False     # moot while stopped — the latch owns standstill
        if has_lead and d_rel <= C.STANDSTILL_RELEASE_DREL:
          self._ss_close_lead = True
        elif has_lead and d_rel > C.STANDSTILL_RELEASE_CLEAR_DREL:
          self._ss_close_lead = False  # a SEEN open gap clears it; lead-absent ticks keep the last
                                       # value (radar dropouts at standstill are the field norm —
                                       # every 11:34-11:38Z flip tick logged lead=False)
      else:
        if self._at_standstill:
          # the RELEASE tick: leaving standstill with a close lead arms the hold — the launch into
          # a short gap stays model-governed instead of a Chill MPC launch into a 9-14 m gap.
          self._release_hold = self._ss_close_lead
          self._at_standstill = False
          self._ss_close_lead = False
        if self._release_hold and (v_now > C.STANDSTILL_RELEASE_V
                                   or (has_lead and d_rel > C.STANDSTILL_RELEASE_CLEAR_DREL)):
          self._release_hold = False   # launch complete / gap opened: hand back to the ladder.
                                       # Lead LOSS deliberately does not disarm (a dropout mid-launch
                                       # must not re-open the jolt); the hold is bounded by v > 5.
      # stay Experimental; return to Chill only when the condition cleared (sustained via filter)
      # AND we've held Experimental at least EXP_MIN_DWELL_S
      if status != "chill":
        self._status = status      # keep showing the active reason
      if not cond_active and self._dwell >= self._exp_min:
        # standstill2pnw LATCH: at standstill (or during the close-lead release hold) an
        # Experimental machine may not demote — zero benefit to Chill at 0 mph, and every demotion
        # there sets up a lurch. A pure demotion GATE: no timer pauses (dwell keeps accumulating,
        # nothing can leak "frozen"), and it releases the instant v_ego rises / the hold disarms —
        # both re-checked from live signals every tick. NOT gated on the stops toggle: this is
        # mode-flap hygiene at 0 mph, not stop machinery. Fail-safe direction only (like A2): it
        # can never enter Experimental, only delay leaving it.
        if v_now < C.STANDSTILL_LATCH_V or self._release_hold:
          self._status = "standstillHold"   # telemetry: the hold is all that keeps Experimental
        # stophold2pnw (A2): standstill-departure hold. Below STANDSTILL_HOLD_V, "the conditions
        # cleared" (typically: the lead crept and slowLead dropped) is NOT sufficient to hand the
        # launch to Chill's MPC — the model must also have agreed GO (shouldStop continuously
        # clear for STOP_CLEAR_HOLD_S). Respects the per-condition "stops" toggle, like the
        # stopIntent fast path. Tagged "stopHold" so field logs show every hold explicitly.
        # Fail-safe direction only: this can never enter Experimental, only delay leaving it.
        elif (v_now < C.STANDSTILL_HOLD_V
              and self._stop_clear_s < C.STOP_CLEAR_HOLD_S
              and bool(signals.get("toggles", {}).get("stops", True))):
          self._status = "stopHold"   # telemetry: the hold is the only thing keeping Experimental
        else:
          self._is_experimental = False
          self._status = "chill"
          self._dwell = 0.0
          self._at_standstill = False   # standstill2pnw: clean slate for the next episode
          self._release_hold = False
          self._ss_close_lead = False
    return self.mode()


# ---------------------------------------------------------------------------
# Phase 2/3 — live wiring. Runs in selfdrived (which publishes the effective
# experimentalMode → both the planner AND the top-right icon follow it).
# Behavior-neutral: experimental_request() returns False whenever CES is
# disabled/non-Tesla, so selfdrived's `manual OR request` == manual == upstream.
# ---------------------------------------------------------------------------

def _toggles_from_params(params) -> dict:
  """Per-condition enables; default ON (the master switch is the real gate)."""
  def gb(k, default=True):
    try:
      return params.get_bool(k)
    except Exception:
      return default
  # ces2core2pnw: "turns" (CESTurns) is the CES2 turn-signal condition — default OFF (study §5.2
  # rule 3: ships dark for the first drives), unlike the four v1 conditions which default ON.
  # decide_active (v1) ignores the key entirely.
  return {"curves": gb("CESCurves"), "stops": gb("CESStops"),
          "low_speed": gb("CESLowSpeed"), "lead": gb("CESLead"),
          "turns": gb("CESTurns", default=False)}


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
    # ces2-study: the model's trajectory ENDPOINT (position.x[-1], meters ahead) — the signal the
    # graded stop-urgency in the CES2 design keys on (DEC/CEM both use it). Logged from now so the
    # replay acceptance dataset accrues with every drive; NOT used in any decision yet.
    try:
      mdl_end_x = float(model.position.x[-1]) if len(model.position.x) else 0.0
    except Exception:
      mdl_end_x = 0.0
    vis_acc, ttc = vision_curve_lat_accel(orz, vx, tb, v_ego)
    # icbmmapfirst2pnw: lateral accel AT t~0 (yaw_rate*speed at the first model point) — the
    # "currently loaded in a curve" signal for the ICBM start/restore gates (icbm_in_curve).
    lat_now = float(orz[0]) * float(vx[0]) if (orz and vx) else 0.0
  except Exception:
    vis_acc, ttc = 0.0, 10.0
    lat_now = 0.0
  try:
    model_should_stop = bool(model.action.shouldStop)
  except Exception:
    model_should_stop = False
  # ces2core2pnw: lane-change intent (modelV2.meta.laneChangeState != off) — the CES2 TURN
  # condition's "signaling a TURN, not a lane change" test (CEM F2's lane detection, v1 form).
  try:
    lane_change_intent = str(model.meta.laneChangeState) != "off"
  except Exception:
    lane_change_intent = False

  # set speed (openpilot's v_cruise, km/h on carState.vCruise) -> m/s; 255 is the unset sentinel.
  v_set_kph = float(getattr(car_state, 'vCruise', 0.0))
  v_set = v_set_kph * CV.KPH_TO_MS if 0.0 < v_set_kph < C.V_SET_MAX_KPH else 0.0

  return {
    "v_ego": v_ego, "has_lead": has_lead, "lead_vlead": lead_vlead, "lead_drel": lead_drel,
    "blinker": bool(car_state.leftBlinker or car_state.rightBlinker),
    "map_target_v": map_target_v, "map_target_dist": map_target_dist,   # map half (MapTargetVelocities)
    "curve_lat_accel_vision": vis_acc, "time_to_curve": ttc,            # vision fallback
    "mdl_end_x": mdl_end_x,                                             # ces2-study replay dataset
    "lat_accel_now": lat_now,                                           # icbmmapfirst2pnw: in-curve gate
    "model_should_stop": model_should_stop, "toggles": toggles,
    "v_set": v_set,                                                     # accelerate-zone (set-speed gap)
    "spd_lim": float(spd_lim),                                         # OSM speed limit (lowSpeed highway gate)
    "a_ego": float(getattr(car_state, 'aEgo', 0.0)),                   # logged for verification
    "gas": bool(getattr(car_state, 'gasPressed', False)),             # logged for verification
    "brake": bool(getattr(car_state, 'brakePressed', False)),         # icbmrestore2pnw: episode abort
    # ces2core2pnw: CES2-only inputs (v1 decide_active ignores both keys)
    "standstill": bool(getattr(car_state, 'standstill', False)),      # CEM standstill hold
    "lane_change_intent": lane_change_intent,                         # TURN condition lane test
  }


class CESStub:
  """stophold2pnw (C): inert fallback selfdrived installs when CESController CONSTRUCTION raises —
  a CES bug must degrade to stock behavior (never Experimental, never publish), never take
  selfdrived (safety-critical) down. Mirrors the CESController surface selfdrived touches."""

  # greenlead2pnw: selfdrived reads these UNCONDITIONALLY every cycle — without them the stub
  # itself would crash selfdrived with AttributeError (latent since greenlight2pnw shipped).
  green_light = False
  lead_departing = False

  def experimental_request(self, car_state, sm) -> bool:
    return False

  def enabled(self) -> bool:
    return False

  def status(self) -> str:
    return "chill"

  def log_take_control_alert(self, payload) -> None:
    # takecontrol2pnw: no event log writer exists in this fallback (CESController construction
    # already failed) -- consistent with every other telemetry method missing here, this is a no-op.
    pass


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
    # ces2core2pnw: the CES2 core runs EVERY tick (pure functions on the same sig dict — cheap).
    # Flag OFF (default): v1 decides, CES2 shadows; its would-be mode/reason/urgency + a cumulative
    # divergence-edge counter go into every ces_events record (ces2Mode/ces2Reason/ces2Urgency/
    # ces2Div). Flag ON (Ces2Core=1): CES2 decides and records mark ces2Live=true.
    self._ces2 = Ces2Core()
    self._ces2_live = False
    self._ces2_mode = None
    self._ces2_reason = None
    self._ces2_urg = 0.0
    self._ces2_div = DivergenceCounter()  # cumulative v1-vs-CES2 divergence EDGES this session
    self._enabled = False
    self._button = C.BTN_CES
    self._toggles = {"curves": True, "stops": True, "low_speed": True, "lead": True}
    self._map_targets = []          # cached MapTargetVelocities (refreshed ~1 Hz)
    self._cur_lat = self._cur_lon = self._cur_bearing = None
    # steerpower2pnw I3 review fix: bounded (wall_time, bearing, gps_valid) history, appended once per
    # _read_map() refresh (~1 Hz) -- see _nearest_bearing() above. Lets a steerEvent record look up
    # the bearing at its actual saturation ONSET instead of the live value at emit time.
    self._bearing_hist: deque = deque(maxlen=_BEARING_HIST_MAXLEN)
    self._vtsc_cap = self._vtsc_state = None
    # vtsctele2pnw: VTSC penalty components actually applied (from VTSCStatus) — logging only
    self._vtsc_pen = self._vtsc_pitch = None
    self._vtsc_dir = ""
    self._vtsc_tele: dict = {}   # curve-source forensics from VTSCStatus
    self._sa_tele: dict = {}     # satele2pnw: speedadjust forensics from SpeedAdjustStatus
    # lanecenter2pnw telemetry: lane-centering trim status (from LaneCenterStatus, published by
    # controlsd — see selfdrive/controls/lib/lane_centering.py) — logging only, never gates control
    # here. Defaulted so a missing/never-published param (feature disabled, or before the first
    # controlsd tick lands) reads as a clean "no data" row rather than an AttributeError.
    self._lc_corr = None     # applied correction (1/m)
    self._lc_act = False     # actively nudging this tick
    self._lc_gate = None     # why not acting (or "ok")
    self._lc_err = None      # center error at lookahead (m)
    self._lc_p1 = self._lc_p2 = None    # laneLineProbs[1]/[2]
    self._lc_s1 = self._lc_s2 = None    # laneLineStds[1]/[2] (m)
    self._lc_ystd = None     # E2E path position.yStd at lookahead (m)
    self._lc_w = None        # apparent lane width at lookahead (m)
    # steerlimit-log2pnw telemetry: steering-limit status (from SteerLimitStatus, published by
    # controlsd — see docs/STEERING-LIMITS.md) — logging only, never gates control here. Defaulted
    # so a missing/never-published param (before the first controlsd tick lands) reads as a clean
    # "no data" row rather than an AttributeError.
    self._sl_curv_lim = False    # curvature_limited: clip_curvature's ISO jerk/accel/max-curv ceiling bound this tick
    self._sl_safe_lim = False    # steer_limited_by_safety: carcontroller/panda had to override the commanded angle
    self._sl_ang_des = None      # commanded steering angle (deg)
    self._sl_ang_act = None      # measured steering angle (deg) -- also already logged as strAng
    self._sl_ang_err = None      # angDes - angActual (deg)
    self._sl_sat = False         # LatControlAngle's own time-integrated saturation flag
    self._sl_lat_dem = None      # pre-clip lateral accel demand (m/s^2)
    self._sl_lat_max = None      # live ISO lateral-accel ceiling this tick (m/s^2), varies with roll
    self._sl_curv_max = None     # live "how tight a curve could we even ask for right now" (1/m)
    # fordkappalog2pnw: commanded vs achieved curvature (1/m), Ford wire convention (positive=left) —
    # see docs/STEERING-LIMITS.md "Ford curvature interface" and the kCmd/kActl/kErr comment block in
    # controlsd.py for the full derivation. Pure observation, same defaulting rationale as the sl*
    # fields above.
    self._sl_k_cmd = None        # commanded curvature this tick (-self.desired_curvature)
    self._sl_k_actl = None       # achieved/measured curvature, derived from CS.yawRate / vEgo
    self._sl_k_err = None        # kCmd - kActl -- sustained large + hands-off = real saturation
    # steertele2pnw: capability-analysis additions — see the steer_limit_status comment block in
    # controlsd.py for the full derivation of each. Same defaulting rationale as the sl* fields above.
    self._sl_lat_active = False  # CC.latActive this tick -- False means angDes/angAct froze to manual steering, not an openpilot capability signal
    self._sl_ang_sat = False     # un-fused angle-only saturation half (curvLim already isolates the curvature half of the fused "sat" flag)
    self._speed_limit = 0.0         # OSM speed limit (m/s, 0 = none) from mapd
    # mapd220-2pnw PHASE 1: mapd v2.2.0 mapdOut fields (@24/@26), bridged via mem params
    # (MapHighwayClass/MapConditionalSpeedLimit — see mapd_configd.py). PURE OBSERVATION: logged
    # only, never gates any curve/speed-limit decision here. See docs/MAPD-V220-UPGRADE.md.
    self._hwy_class = None          # HighwayClass enum name, e.g. "motorway" (None = no data yet)
    self._cond_spd_lim = ""         # raw OSM maxspeed:conditional text; "" = none
    # waysel2pnw (PURE OBSERVATION): how confident mapd is about WHICH way we are on, and how far off
    # its centreline. Answers "was the map even looking at our road?" when a curve target is absurd.
    self._way_sel = None            # current / predicted / possible / extended / fail (None = no data)
    self._way_off = None            # metres from the selected way's centreline (None = no data)
    self._frame = 0
    # telemetry / logging (display + diagnostics only — never gates control)
    self._last_mode = "off"         # last logged mode: off / chill / experimental
    self._tele_last = 0.0           # monotonic stamp of last CESStatus publish
    self._tick_last = 0.0           # monotonic stamp of last breadcrumb tick
    self._steer_tick_last = 0.0     # cessteerlog2pnw: monotonic stamp of last CES-off steer breadcrumb
    self._steer_event_seen_id = None  # steerevent2pnw: last SteerEvent.evId already appended (edge dedup;
                                       #   evId is now a per-process-salted STRING, see I2 review fix)
    self._steer_event_frame = 0       # steerevent2pnw B1: call counter -- throttles the mem-param GET
                                       #   itself to ~5 Hz instead of every ~100 Hz call
    self._steer_event_raw_last = None  # steerevent2pnw B1: last raw SteerEvent bytes seen -- skip
                                        #   json.loads entirely when unchanged since the last GET
    self._last_decide_t = None      # monotonic stamp of last state-machine step (for real dt)
    self._bs_l = False              # bsm2pnw: last-seen blind-spot booleans (telemetry only —
    self._bs_r = False              #   proves BSM liveness in ces_events; never gates control here)
    self._event_log_ok = False      # persistent "each adoption" trail (CES_EVENT_LOG)
    self._append_fail = 0           # stophold2pnw (C): consecutive _append_event failures (0 = healthy)
    try:
      os.makedirs(os.path.dirname(CES_EVENT_LOG), exist_ok=True)
      self._event_log_ok = True
    except Exception:
      self._event_log_ok = False
    # stophold2pnw (D): car identity in every record — `shadow` stopped being a car discriminator
    # the day Alpha-Long became an A/B switch on the Lightning (2026-07-12 session misattribution).
    self._car = str(getattr(CP, 'carFingerprint', '') or '') if CP is not None else ''
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
    # icbmrestore2pnw: the cap->clear->restore episode machine + the current direction for telemetry
    self._icbm_ep = IcbmEpisode()
    self._icbm_dir = None                  # "dec" while capping, "inc" while restoring, None idle
    self._stock_set = 0.0
    self._stock_on = False
    # pullaway2pnw: stateful evidence for the below-floor lead-pull-away exception (monotonic
    # dRel rise + model-stop recency). Feeds sig["lead_opening"]; pure logic stays in decide_active.
    self._pullaway_trk = PullAwayTracker()
    # greenlight2pnw/greenlead2pnw: ALWAYS-ON standstill dings (driver decision 2026-07-12: no
    # toggle). Runs every cycle BEFORE the CES-enabled gate so it works with CESMode off too.
    # selfdrived reads .green_light / .lead_departing (each True for exactly one cycle per firing)
    # and raises the matching alert event. _gl_ev_pending is the one-shot "glEv" marker the next
    # ces_events record consumes (records are ~1 Hz, firings are one 100 Hz tick — a latch, not a
    # per-tick flag, or every event would be invisible in the breadcrumb).
    self._gl = GreenLightDetector()
    self._gl_last_t = None
    self.green_light = False
    self.lead_departing = False
    self._gl_ev_pending = None
    # icbmmapfirst2pnw: start-gate telemetry — WHY a would-be new episode was suppressed this tick
    # ("inCurve" / "visCovered" / "visLate" / None) + the map coverage reach (m; 0 = mapd blind/dead,
    # the mapd-liveness evidence for the field logs). Display/log only — never gates control here.
    self._icbm_gate = None
    self._icbm_map_reach = None

  def _set_mode(self, mode: int):
    """Apply a CESMode change: pick the gentle vs default dwell and (re)build the state machine only
    when the gentle flag actually flips, so we don't reset the dwell every ~1 Hz read."""
    gentle = C.ces_is_gentle(mode)
    if mode != self._mode or gentle != self._gentle:
      if gentle != self._gentle:
        if gentle:
          self._sm = ConditionalExperimentalSwitching(C.GENTLE_EXP_MIN_DWELL_S, C.GENTLE_CHILL_MIN_DWELL_S)
          self._ces2 = Ces2Core(C.GENTLE_EXP_MIN_DWELL_S, C.GENTLE_CHILL_MIN_DWELL_S)
        else:
          self._sm = ConditionalExperimentalSwitching()
          self._ces2 = Ces2Core()
      self._mode = mode
      self._gentle = gentle

  def _read_params(self):
    if self._frame % max(1, int(1.0 / DT_CTRL)) == 0:   # ~1 Hz (selfdrived steps at 100 Hz / DT_CTRL)
      try:
        mode = C.read_ces_mode(self.params)
      except Exception:
        mode = C.CES_MODE_OFF
      self._set_mode(mode)
      # rain2pnw: push the live wet-weather tier into the capability view (used by the ICBM curve
      # target below; applies in shadow too). Defensive — never let a param hiccup break _read_params.
      try:
        self._veh.set_rain_tier(self.params.get("RainMode", return_default=True))
      except Exception:
        pass
      # CES is meaningful only when openpilot owns longitudinal (same gate as ExperimentalMode) —
      # except in Lightning shadow mode, where the pipeline runs for telemetry/display only.
      self._enabled = (self._long_ok or self._shadow) and C.ces_enabled(self._mode)
      if self._enabled:
        self._toggles = _toggles_from_params(self.params)
        try:
          self._button = int(self.params.get("CESButtonState", return_default=True) or 0)
        except Exception:
          self._button = C.BTN_CES
        # ces2core2pnw: CES2 live flag (default OFF = shadow-only; any read failure -> OFF)
        try:
          self._ces2_live = bool(self.params.get_bool("Ces2Core"))
        except Exception:
          self._ces2_live = False
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
    # mapd220-2pnw PHASE 1: mapd v2.2.0 highwayClass/conditionalSpeedLimit — logging only (see
    # _event_record). Same cross-process mem-param read pattern as MapSpeedLimit just above.
    try:
      hc = self.mem_params.get("MapHighwayClass", return_default=True)
      self._hwy_class = str(hc) if hc not in (None, "", b"") else None
    except Exception:
      self._hwy_class = None
    try:
      csl = self.mem_params.get("MapConditionalSpeedLimit", return_default=True)
      self._cond_spd_lim = str(csl) if csl not in (None, b"") else ""
    except Exception:
      self._cond_spd_lim = ""
    # waysel2pnw: same cross-process mem-param pattern; both stay None when mapd is dead or old, so a
    # missing value is visibly "no data" in the log rather than a plausible-looking default.
    try:
      ws = self.mem_params.get("MapWaySel", return_default=True)
      self._way_sel = str(ws) if ws not in (None, "", b"") else None
    except Exception:
      self._way_sel = None
    try:
      wo = self.mem_params.get("MapWayOffset", return_default=True)
      self._way_off = float(wo) if wo not in (None, "", b"") else None
    except Exception:
      self._way_off = None
    # VTSC applied cap + state — logging only (see _event_record)
    try:
      vt = self.mem_params.get("VTSCStatus", return_default=True)
      if isinstance(vt, (bytes, str)):
        vt = json.loads(vt)
      self._vtsc_cap = round(float(vt["cap"]), 1) if vt.get("engaged") else None
      self._vtsc_state = vt.get("state")
      # vtsctele2pnw: penalty components VTSC actually applied this cycle (Lightning hump penalty
      # m/s, road pitch rad it used, apex turn direction "L"/"R"/"") — so over/under-slow curve
      # forensics read the real inputs instead of inferring descent/left multipliers after the fact.
      pen, pitch = vt.get("pen"), vt.get("pitch")
      self._vtsc_pen = round(float(pen), 2) if pen is not None else None
      self._vtsc_pitch = round(float(pitch), 4) if pitch is not None else None
      self._vtsc_dir = str(vt.get("dir") or "")
      # curvefloor2pnw / mapcurv2pnw: WHY each curve source did or didn't bind. Without ingesting
      # these here they never leave /dev/shm -- VTSCStatus is volatile and overwritten at 5 Hz, and
      # this cherry-picking read was the ONLY consumer, so the fields published by 62a51a4772 were
      # being computed and thrown away. That made the drive-replay they exist for impossible.
      # waysel2pnw: apexCurvature/apexDist/vCurveSafe are the FINAL post-fold values that actually set
      # the cap -- they were already being published here and discarded by this very list, so the cap's
      # own curvature had to be back-computed from the cap. curveWin/rsnMap/rsnVis say which source won.
      for k in ("mapRaw", "mapEff", "mapD", "mapFlr", "visK", "visD", "visV",
                "mapK", "mapKD", "mapKV", "mapKN", "mapKAhead",
                "apexCurvature", "apexDist", "vCurveSafe", "curveWin", "rsnMap", "rsnVis"):
        self._vtsc_tele[k] = vt.get(k)
    except Exception:
      self._vtsc_cap = self._vtsc_state = None
      self._vtsc_pen = self._vtsc_pitch = None
      self._vtsc_dir = ""
      self._vtsc_tele = {}
    # satele2pnw: speedadjust (police cap + limit-drop trim) internals — logging only. Same volatile
    # /dev/shm channel and same cherry-pick requirement as VTSCStatus above: without this read the
    # fields never leave the mem-param. Prefixed "sa" so they cannot collide with VTSC's keys in the
    # flattened tick record. Driver directive 2026-08-21 (the undiagnosable stuck-at-47-mph episode).
    try:
      st = self.mem_params.get("SpeedAdjustStatus", return_default=True)
      if isinstance(st, (bytes, str)):
        st = json.loads(st)
      # Fable review 2026-08-21: this list MUST cover every key _publish_status() emits, or the
      # missing ones evaporate in /dev/shm -- the exact trap satele2pnw's own commit message named,
      # and it had already caught vCruise + polKey (published, never picked up, while
      # params_keys.h advertised polKey as reaching ces_events).
      self._sa_tele = {"sa" + k[0].upper() + k[1:]: st.get(k) for k in
                       ("mode", "sl", "slRef", "ratio", "cap", "out", "vSet", "vCruise", "lastSet",
                        "ovr", "eng", "polLatch", "polSupp", "polKey")}
    except Exception:
      self._sa_tele = {}
    # lanecenter2pnw telemetry: lane-centering trim status — logging only (see _event_record).
    # Same cross-process read as VTSCStatus just above: controlsd (100 Hz) publishes to
    # /dev/shm/params at ~5 Hz, this reads it at ~1 Hz. Fully defensive — any missing key, wrong
    # type, or malformed JSON degrades to the same None/"off" defaults set in __init__ rather than
    # raising; a stale telemetry read can never affect CES's own decisions (this method's output is
    # display/log-only throughout).
    try:
      lc = self.mem_params.get("LaneCenterStatus", return_default=True)
      if isinstance(lc, (bytes, str)):
        lc = json.loads(lc)
      # Before controlsd's first publish (feature/CES just started) the key reads back as None; treat
      # anything that isn't a dict as an empty "no data yet" row so every field cleanly defaults to
      # None below, instead of throwing an AttributeError into the except once per second until live.
      if not isinstance(lc, dict):
        lc = {}
      corr = lc.get("corr")
      self._lc_corr = round(float(corr), 5) if corr is not None else None
      self._lc_act = bool(lc.get("act", False))
      self._lc_gate = str(lc.get("gate")) if lc.get("gate") is not None else None
      err = lc.get("err")
      self._lc_err = round(float(err), 2) if err is not None else None
      p1, p2 = lc.get("p1"), lc.get("p2")
      self._lc_p1 = round(float(p1), 2) if p1 is not None else None
      self._lc_p2 = round(float(p2), 2) if p2 is not None else None
      s1, s2 = lc.get("s1"), lc.get("s2")
      self._lc_s1 = round(float(s1), 2) if s1 is not None else None
      self._lc_s2 = round(float(s2), 2) if s2 is not None else None
      ystd = lc.get("yStd")
      self._lc_ystd = round(float(ystd), 2) if ystd is not None else None
      w = lc.get("w")
      self._lc_w = round(float(w), 2) if w is not None else None
    except Exception:
      self._lc_corr = self._lc_err = None
      self._lc_p1 = self._lc_p2 = self._lc_s1 = self._lc_s2 = None
      self._lc_ystd = self._lc_w = None
      self._lc_act = False
      self._lc_gate = None
    # steerlimit-log2pnw telemetry: steering-limit status — logging only (see _event_record). Same
    # cross-process read as LaneCenterStatus just above: controlsd (100 Hz) publishes to
    # /dev/shm/params at ~5 Hz, this reads it at ~1 Hz. Fully defensive — any missing key, wrong
    # type, or malformed JSON degrades to the same None/False defaults set in __init__ rather than
    # raising; a stale telemetry read can never affect CES's own decisions (this method's output is
    # display/log-only throughout). See docs/STEERING-LIMITS.md.
    try:
      sl = self.mem_params.get("SteerLimitStatus", return_default=True)
      if isinstance(sl, (bytes, str)):
        sl = json.loads(sl)
      # Before controlsd's first publish the key reads back as None; treat anything that isn't a dict
      # as an empty "no data yet" row so every field cleanly defaults below, same pattern as lc above.
      if not isinstance(sl, dict):
        sl = {}
      self._sl_curv_lim = bool(sl.get("curvLim", False))
      self._sl_safe_lim = bool(sl.get("safeLim", False))
      ang_des = sl.get("angDes")
      self._sl_ang_des = round(float(ang_des), 2) if ang_des is not None else None
      ang_act = sl.get("angAct")
      self._sl_ang_act = round(float(ang_act), 2) if ang_act is not None else None
      ang_err = sl.get("angErr")
      self._sl_ang_err = round(float(ang_err), 2) if ang_err is not None else None
      self._sl_sat = bool(sl.get("sat", False))
      lat_dem = sl.get("latDem")
      self._sl_lat_dem = round(float(lat_dem), 3) if lat_dem is not None else None
      lat_max = sl.get("latMax")
      self._sl_lat_max = round(float(lat_max), 3) if lat_max is not None else None
      curv_max = sl.get("curvMax")
      self._sl_curv_max = round(float(curv_max), 5) if curv_max is not None else None
      # fordkappalog2pnw: commanded vs achieved curvature — same read/default pattern as the sl*
      # fields directly above, same dict, no separate publish/read cycle.
      k_cmd = sl.get("kCmd")
      self._sl_k_cmd = round(float(k_cmd), 6) if k_cmd is not None else None
      k_actl = sl.get("kActl")
      self._sl_k_actl = round(float(k_actl), 6) if k_actl is not None else None
      k_err = sl.get("kErr")
      self._sl_k_err = round(float(k_err), 6) if k_err is not None else None
      # steertele2pnw: same defensive get/default pattern as the sl* fields above, same dict, no
      # separate publish/read cycle.
      self._sl_lat_active = bool(sl.get("latActive", False))
      self._sl_ang_sat = bool(sl.get("angSat", False))
    except Exception:
      self._sl_curv_lim = self._sl_safe_lim = self._sl_sat = False
      self._sl_ang_des = self._sl_ang_act = self._sl_ang_err = None
      self._sl_lat_dem = self._sl_lat_max = self._sl_curv_max = None
      self._sl_k_cmd = self._sl_k_actl = self._sl_k_err = None
      self._sl_lat_active = self._sl_ang_sat = False
    # steerpower2pnw I3 review fix: append this refresh's (wall_time, bearing, gps_valid) sample to
    # the bounded history — see _nearest_bearing()/_BEARING_HIST_MAXLEN above. gps_valid mirrors the
    # exact "gps" test every record already uses (lat AND lon present); a no-fix sample is still
    # appended (bearing=None, gps_valid=False) so a lookup landing near it correctly resolves to "no
    # heading data at that time" instead of silently falling through to some other sample.
    self._bearing_hist.append((time.time(), self._cur_bearing,   # noqa: TID251 -- wall clock, ~1 Hz
                               self._cur_lat is not None and self._cur_lon is not None))

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
    # greenlight2pnw: always-on (independent of CESMode/_enabled — display/sound only)
    self._green_light_step(car_state, sm)
    # cessteerlog2pnw: unconditional steer/lane-centering breadcrumb — no-ops once _enabled is True
    # (the normal tick/adopt path below already logs the same fields), so this only ever adds the
    # CES-off records that were previously missing from the log entirely.
    self._steer_log_step(car_state, sm)
    # steerevent2pnw: edge-triggered mirror of controlsd's flight-recorder burst into ces_events —
    # called unconditionally every cycle (NOT gated on C.TICK_S like _steer_log_step above),
    # independent of CESMode/_enabled since the underlying saturation edge is a controlsd/steering
    # fact, not a CES decision. The method throttles its OWN mem-param GET internally to ~5 Hz and
    # dedups on the raw payload before parsing (B1 review fix) — see its docstring.
    self._steer_event_step()
    if not self._enabled:
      if self._last_mode != "off":
        cloudlog.info("CES disabled (master OFF / no openpilot long) -> Chill baseline")
        self._last_mode = "off"
      self._sm.reset()
      self._ces2.reset()               # ces2core2pnw: shadow state resets with the live one
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
    # pullaway2pnw: advance the pull-away evidence every cycle and inject it into the signals dict
    # BEFORE the decision (decision_telemetry consumes the same dict, so the overlay/why agrees).
    if sig is not None:
      try:
        sig["lead_opening"] = self._pullaway_trk.update(now_t, sig["has_lead"], sig["lead_drel"],
                                                        sig["model_should_stop"])
      except Exception:
        sig["lead_opening"] = False
    dt = (now_t - self._last_decide_t) if self._last_decide_t is not None else DT_CTRL
    self._last_decide_t = now_t
    dt = min(max(dt, 1e-3), 0.5)           # clamp first call / scheduling hiccups

    # ces2core2pnw: advance the CES2 core every tick a sig is available — SHADOW when the flag is
    # OFF (v1 below stays the byte-identical decider), LIVE when Ces2Core=1. Pure functions on the
    # same dict (Ces2Core copies it, never mutates); any CES2 exception degrades to v1.
    ces2_want = None
    if sig is not None and self._button != C.BTN_CHILL:
      try:
        ces2_want = self._ces2.update_decision(sig, dt) == "experimental"
        self._ces2_mode = "experimental" if ces2_want else "chill"
        self._ces2_reason = self._ces2.status()
        self._ces2_urg = self._ces2.urgency
      except Exception:
        ces2_want = None
        self._ces2_mode = self._ces2_reason = None

    if self._button == C.BTN_CHILL:        # forced Chill
      self._sm.reset()
      self._ces2.reset()                   # ces2core2pnw: shadow state resets with the live one
      self._ces2_mode = self._ces2_reason = None
      want = False
    elif self._button == C.BTN_EXP:        # forced full Experimental
      want = True
    elif sig is not None:                  # BTN_CES: condition ladder decides
      # v1 ALWAYS advances (it is the live decider when the flag is OFF, and the reverse-shadow
      # divergence reference when the flag is ON).
      want_v1 = self._sm.update_decision(sig, dt) == "experimental"
      self._ces2_div.update(want_v1, ces2_want)   # divergence EDGES, not per-tick spam
      want = ces2_want if (self._ces2_live and ces2_want is not None) else want_v1
    else:
      want = False

    self._publish_status(sig, want)
    # icbm2pnw: in Lightning shadow mode the CES/planner path never actuates, but the ICBM brain
    # publishes a stock-ACC set-speed target the ford carcontroller executor follows (curve
    # slow-down, dec-only against the driver's own set — see icbm_curve_target). ICBM mirrors the
    # button: ACTIVE only in the CES state, SILENT in forced Chill. This is the driver's kill switch
    # for ICBM, so it must stay reachable in ONE tap from the default boot state (CES) — verified by
    # oplongexp2pnw's select_ces_cycle ordering (CES -> Chill -> Exp-confirm) in exp_button.py.
    # CESButtonState itself still never stores BTN_EXP while op-long is off (exp_button.py forces the
    # landing state back to CES and gates the actual enable behind a separate confirm tap), so forced
    # Exp stays structurally unreachable HERE regardless of the onroad button's own multi-tap UI flow
    # to reach it. Publishing empty in Chill stops the executor.
    if self._shadow:
      self._icbm_step(sig, active=(sig is not None and self._button == C.BTN_CES))
    return want and self._long_ok

  def _green_light_step(self, car_state, sm) -> None:
    """greenlight2pnw/greenlead2pnw: advance the pure GreenLightDetector one cycle and latch the
    per-cause alert flags for selfdrived (each True for exactly the firing cycle):
      "green"      -> .green_light    (no lead, path opens: the greenLight alert)
      "lead"       -> .lead_departing (stopped lead pulls away from OUR standstill: leadDeparting)
      "leadMoving" -> telemetry record only, NO alert (driver rule #3: never ding while rolling)
    Best-effort: any message hiccup means 'no ding this cycle', never an exception into
    selfdrived's control loop. Model-based and car-agnostic — reads only vEgo/gasPressed,
    modelV2 (endpoint + shouldStop) and radarState.leadOne; no fingerprints, no CAN, no control
    output."""
    now = time.monotonic()
    dt = (now - self._gl_last_t) if self._gl_last_t is not None else DT_CTRL
    self._gl_last_t = now
    ev = None
    try:
      model = sm['modelV2']
      lead = sm['radarState'].leadOne
      try:
        mdl_end_x = float(model.position.x[-1]) if len(model.position.x) else 0.0
      except Exception:
        mdl_end_x = 0.0
      try:
        should_stop = bool(model.action.shouldStop)
      except Exception:
        should_stop = False
      has_lead = bool(getattr(lead, 'status', False))
      ev = self._gl.update(dt, float(car_state.vEgo), bool(getattr(car_state, 'gasPressed', False)),
                           should_stop, mdl_end_x, has_lead,
                           float(getattr(lead, 'dRel', 0.0)) if has_lead else 0.0,
                           float(getattr(lead, 'vLead', 0.0)) if has_lead else 0.0)
      if ev is not None:
        cloudlog.info("greenlead2pnw: %s mdlEndX=%.1f lead=%s", ev, mdl_end_x, has_lead)
        self._gl_ev_pending = ev   # one-shot marker for the next ces_events record ("glEv")
        # dedicated ces_events record (works even with CES off — log-validation channel)
        self._append_event({
          "t": round(time.time(), 1),  # noqa: TID251 -- wall clock, for route/time correlation
          "ev": "greenLight", "cls": ev, "mdlEndX": round(mdl_end_x, 1), "lead": has_lead,
          "dRel": round(float(getattr(lead, 'dRel', 0.0)), 1) if has_lead else 0.0,
          "vEgo": round(float(car_state.vEgo), 2),
          "lat": self._cur_lat, "lon": self._cur_lon,
        })
    except Exception:
      ev = None
    self.green_light = ev == GL_EV_GREEN
    self.lead_departing = ev == GL_EV_LEAD

  def _steer_log_step(self, car_state, sm) -> None:
    """cessteerlog2pnw: LOGGING ONLY. The sl*/lc*/vtsc* steering/lane-centering diagnostics in the
    normal tick/adopt records (_event_record) are only ever written while CES is enabled
    (CESMode>0) — _read_params() only calls _read_map() (which refreshes those fields from the
    SteerLimitStatus/LaneCenterStatus/VTSCStatus mem-params) inside `if self._enabled:`. On a
    CES-off drive that leaves steering behavior completely invisible in ces_events.jsonl. This
    method closes that gap with its own throttled (~C.TICK_S, same cadence as the normal tick)
    breadcrumb, mirroring _green_light_step's "always-on, log-validation channel" pattern:
      - runs BEFORE the `if not self._enabled: return False` gate in experimental_request, but
        no-ops immediately once _enabled is True, so it NEVER duplicates the existing tick/adopt
        record and never runs while CES is on;
      - when CES is off, calls _read_map() itself (the thing _read_params() skips) — that method
        is a pure, defensive read against self.mem_params (published by controlsd independent of
        CES) with try/except around every field, so it is safe to call regardless of _enabled;
      - does NOT call experimental_request's own decision logic, _publish_status, the ICBM
        executor, CES2, or any mode/actuator path — only reads mem-params and appends one JSONL
        record via the same best-effort _append_event used everywhere else in this file.
    Any exception here is swallowed; a broken breadcrumb must never affect control.

    leadrate2pnw: `sm` is the SAME SubMaster experimental_request() already holds (it's passed
    through from there, this call site adds no new subscription) — used ONLY to read
    sm['radarState'].leadOne, exactly like _green_light_step/experimental_request already do
    elsewhere in this file, so the CES-off "steer" breadcrumb can carry hasLead/dRel/vLead too
    (previously only the enabled tick/adopt path had lead telemetry)."""
    if self._enabled:
      return   # the enabled tick/adopt path already logs sl*/lc*/vtsc* once per ~1 Hz — no duplicate
    now = time.monotonic()
    if now - self._steer_tick_last < C.TICK_S:
      return
    self._steer_tick_last = now
    try:
      self._read_map()   # refresh sl*/lc*/vtsc*/GPS fields that _read_params() skips while CES is off
      # N1 review fix: keep the RAW read separate from the display-friendly v_ego -- a failed/absent
      # read must degrade achLat to None (not the false "driving straight" of k_actl * 0.0). raw_vego
      # is None exactly when the read failed; v_ego (the logged field) still defaults to 0.0 for
      # display continuity, matching controlsd's _ach_lat_ms2 pattern (None in, None out).
      try:
        raw_vego = getattr(car_state, 'vEgo', None)
        v_ego = round(float(raw_vego), 2) if raw_vego is not None else 0.0
      except Exception:
        raw_vego = None
        v_ego = 0.0
      now_wall = time.time()  # noqa: TID251 -- wall clock, for route/time correlation
      gps_valid = self._cur_lat is not None and self._cur_lon is not None
      # steerpower2pnw: pure functions, computed from fields already read just above by _read_map().
      ach_lat = _ach_lat(self._sl_k_actl, raw_vego)
      # leadrate2pnw: lead state, read directly from radarState.leadOne (same message CES already
      # subscribes to and reads elsewhere -- see experimental_request/_green_light_step). Defensive:
      # any failure here degrades to "no lead" telemetry-wise, never raises into this breadcrumb.
      # N-2 (Fable review): hasLead is genuinely THREE-STATE, not a plain bool -- False means "radar
      # read fine, no lead"; None means "the read itself failed / radarState unavailable this tick"
      # (the except below). Offline consumers filtering on the hasLead boolean should treat None as
      # "unknown", not silently coerce it to False -- a read failure is not the same fact as "no lead".
      has_lead = d_rel = v_lead = None
      try:
        lead = sm['radarState'].leadOne
        has_lead = bool(getattr(lead, 'status', False))
        if has_lead:
          d_rel_raw = float(getattr(lead, 'dRel', 0.0))
          v_lead_raw = float(getattr(lead, 'vLead', 0.0))
          # N-1 (Fable review): a NaN dRel/vLead from a corrupted radarState message would otherwise
          # round-trip straight through round()/float() into a bare, invalid-JSON NaN token -- same
          # guard as the existing vEgo NaN guard in the "alert" record above (search "takecontrol2pnw"
          # in this file). Gemini review note: this guards dRel/vLead INDEPENDENTLY of has_lead --
          # a genuinely-present lead (radar's own status bit True) with a corrupted distance reading
          # logs as hasLead=True, dRel=None (not hasLead=None/False) -- the lead itself is real, only
          # the poisoned field nulls. Offline consumers must not assume hasLead=True implies dRel is
          # non-null.
          d_rel = round(d_rel_raw, 1) if math.isfinite(d_rel_raw) else None
          v_lead = round(v_lead_raw, 1) if math.isfinite(v_lead_raw) else None
      except Exception:
        has_lead = d_rel = v_lead = None
      rec = {
        "t": round(now_wall, 1),
        # cesOff: True means self._enabled is False here — this can include CESMode 1/2 (Light/
        # Standard) when the car has neither op-long nor shadow (see _enabled's definition).
        "ev": "steer", "cesOff": True, "cesMode": self._mode, "car": self._car, "vEgo": v_ego,
        "gps": gps_valid,
        "lat": self._cur_lat, "lon": self._cur_lon, "bearing": self._cur_bearing,
        "spdLim": round(self._speed_limit, 1) if self._speed_limit else 0.0,
        # VTSC applied cap + state (from VTSCStatus) — same fields as the enabled-path tick record.
        "vtscCap": self._vtsc_cap, "vtscState": self._vtsc_state, **getattr(self, "_vtsc_tele", {}),
        **getattr(self, "_sa_tele", {}),
        # lanecenter2pnw fields (from LaneCenterStatus) — same subset the enabled-path tick logs.
        "lcCorr": self._lc_corr, "lcAct": self._lc_act, "lcGate": self._lc_gate, "lcErr": self._lc_err,
        # steerlimit-log2pnw / steertele2pnw / fordkappalog2pnw fields (from SteerLimitStatus).
        "slCurvLim": self._sl_curv_lim, "slSafetyLim": self._sl_safe_lim,
        "slAngDes": self._sl_ang_des, "slAngAct": self._sl_ang_act, "slAngErr": self._sl_ang_err,
        "slLatDem": self._sl_lat_dem, "slLatMax": self._sl_lat_max, "slCurvMax": self._sl_curv_max,
        "slSat": self._sl_sat, "slLatAct": self._sl_lat_active, "slAngSat": self._sl_ang_sat,
        "slKCmd": self._sl_k_cmd, "slKActl": self._sl_k_actl, "slKErr": self._sl_k_err,
        # steerpower2pnw: LOGGING ONLY — delivered lateral accel (m/s^2, signed) + 8-pt compass
        # heading, to measure the truck's true hands-off steering capability by direction. I4 review
        # fix: heading nulls (not "N") when there's no current GPS fix, rather than _compass()
        # silently reading a no-fix-defaulted 0.0 bearing as true north.
        "achLat": round(ach_lat, 3) if ach_lat is not None else None,
        "heading": _heading_if_fixed(self._cur_bearing, gps_valid),
        # leadrate2pnw: LOGGING ONLY — lead-car state alongside this breadcrumb, so offline analysis
        # can separate "driver holding a speed by choice" from "speed forced by a slow lead" even on
        # a CES-off drive (previously only the enabled tick/adopt path carried lead telemetry).
        # dRel/vLead null (not 0.0) when there is no lead -- 0.0 would be indistinguishable from a
        # genuine lead sitting right at the bumper.
        "hasLead": has_lead, "dRel": d_rel, "vLead": v_lead,
      }
      if clock_bad(now_wall):
        rec["clockBad"] = True
      self._append_event(rec)
    except Exception:
      pass

  def _steer_event_step(self) -> None:
    """steerevent2pnw: edge-triggered mirror of controlsd's SteerEvent flight-recorder burst
    (docs/pnw/LANE-DEPARTURE-LOGGING-PROPOSALS.md Proposal 1) into ces_events.jsonl.

    Unlike _steer_log_step (throttled to ~C.TICK_S, CES-off only), this method is CALLED
    unconditionally every cycle from experimental_request() (selfdrived's ~100 Hz loop). It runs
    regardless of CESMode/_enabled: the underlying saturation episode is a controlsd/steering fact,
    independent of what CES itself is deciding.

    B1 review fix -- the mem-param GET itself is throttled internally, it is NOT done every call:
    Params.get() in this fork is a real file read + json.loads on every invocation (not a cached
    lookup, unlike the pattern _read_map's other fields might suggest -- those are also real reads,
    just made at _read_map's own ~1 Hz call cadence, not 100 Hz). Two layers keep this cheap:
      1. self._steer_event_frame counts calls; only every 20th (~5 Hz at this ~100 Hz call site)
         touches the param store at all. A rare event tolerates the <=~200 ms added latency -- the
         record carries its own srcT/t so nothing about the event's own timing is lost.
      2. Even at 5 Hz, the RAW bytes are read directly (mem_params.get_param_path + a plain file
         read) and compared against self._steer_event_raw_last BEFORE any json.loads -- an unchanged
         payload (the overwhelming majority of throttled reads, since a new event is rare) returns
         immediately without ever parsing JSON.
    Cost when idle: one file stat+read every ~200 ms + a bytes comparison. No JSON parsing and no
    _append_event write happens unless the raw payload actually changed.

    I3 review fix -- mem params are NOT cleared between drives (CLEAR_ON_MANAGER_START only fires on
    a manager restart, not on ignition), so a SteerEvent left over from a PREVIOUS drive would
    otherwise be picked up here on the next ignition and appended stamped with the new drive's wall
    time + current GPS -- wrong on both counts. The event's own wall-clock `t` (when controlsd
    actually emitted it) is checked against `now`; anything older than ~45 s is marked seen (so it's
    never reconsidered) but is NOT appended.

    Defensive: a missing mem store, a missing/never-published SteerEvent file, a malformed payload,
    or any field access failure is treated as 'nothing to log' and swallowed — this runs inside
    selfdrived's 100 Hz experimental_request() call and must never raise into it."""
    if self.mem_params is None:
      return
    # B1.1: throttle the GET itself to ~5 Hz -- see docstring.
    self._steer_event_frame += 1
    if self._steer_event_frame % 20 != 0:
      return
    try:
      path = self.mem_params.get_param_path("SteerEvent")
      try:
        with open(path, "rb") as f:
          raw = f.read()
      except FileNotFoundError:
        raw = b""   # never published yet (or a store that predates this key)
      # B1.2: dedup on the RAW bytes before parsing -- an unchanged payload means nothing new to log.
      if raw == self._steer_event_raw_last:
        return
      self._steer_event_raw_last = raw
      if not raw:
        return
      ev = json.loads(raw)
      if not isinstance(ev, dict) or not ev:
        return
      ev_id = ev.get("evId")
      if ev_id is None or ev_id == self._steer_event_seen_id:
        return   # either malformed (no evId) or already appended — edge dedup (evId is a string,
                 # salted per controlsd process -- see I2 review fix -- but plain `==` still works)
      self._steer_event_seen_id = ev_id   # mark seen now: a stale event below must never be
                                           # reconsidered even though it isn't appended
      now_wall = time.time()  # noqa: TID251 -- wall clock, rare edge-triggered append only
      # I3 review fix: skip (but keep marked-seen) a leftover event from a previous drive.
      src_t = ev.get("t")
      try:
        stale = src_t is not None and (now_wall - float(src_t)) > 45.0
      except Exception:
        stale = False
      if stale:
        return
      # steerpower2pnw I3 review fix: anchor the heading lookup on the episode's ONSET time
      # (controlsd's "onsetT", the idle->armed edge -- see the _flight_start_wall comment there), not
      # the emit time `src_t` used for staleness above. Falls back to src_t for an event emitted by a
      # pre-fix controlsd build that doesn't carry onsetT yet -- never worse than the old emit-time
      # behavior, and _nearest_bearing degrades to (None, False) if bearing_anchor_t is also None or
      # the history is empty (e.g. right at boot before _read_map has run once).
      onset_t = ev.get("onsetT")
      bearing_anchor_t = onset_t if onset_t is not None else src_t
      bearing_at_onset, gps_at_onset = _nearest_bearing(self._bearing_hist, bearing_anchor_t)
      rec = {
        "t": round(now_wall, 1),
        "ev": "steerEvent", "evId": ev_id, "car": self._car, "cesMode": self._mode,
        "durationS": ev.get("durationS"), "peakAngErr": ev.get("peakAngErr"),
        # steerpower2pnw: THE capability number — pass through controlsd's 100 Hz peak |achLat|
        # (m/s^2) for this episode as-is (already computed/rounded there — no re-derivation here).
        # Meaningful for offline direction-of-travel capability analysis when driverOverride is False.
        "peakAchLat": ev.get("peakAchLat"),
        # leadrate2pnw: pass through controlsd's 100 Hz peak steering-RATE accumulators as-is (already
        # computed/rounded there, same non-re-derivation contract as peakAchLat above). The S-curve
        # reversal that motivated this showed the binding limit is a SLEW RATE, not lateral accel --
        # peakCmdRate is how fast the plan asked to turn, peakActRate is how fast the wheel actually
        # followed; the gap between them (hands-off, driverOverride False) is the EPS slew ceiling.
        "peakCmdRate": ev.get("peakCmdRate"), "peakActRate": ev.get("peakActRate"),
        "peakLaneOff": ev.get("peakLaneOff"), "minLaneMargin": ev.get("minLaneMargin"),
        "minLaneConf": ev.get("minLaneConf"),
        # Never silently trust a low-confidence excursion: default True (flagged) if the source
        # event is missing the field for any reason, rather than defaulting to "confident".
        "laneLowConf": bool(ev.get("laneLowConf", True)),
        # I1 review fix: whether a driver steering override happened anywhere in the episode's own
        # window -- lets offline analysis separate genuine "openpilot couldn't steer" departures from
        # override-adjacent ones without silently dropping the latter.
        "driverOverride": bool(ev.get("driverOverride", False)),
        # steertrig2pnw: WHICH condition armed this episode ("sat"/"angSat"/"angErr"/"undershoot",
        # comma-joined) plus the two undershoot_turn inputs at the arming edge. Without these the
        # 33-events-in-19-min cluster of 2026-08-21 could not be attributed to any trigger: sat /
        # angSat / undershoot are evaluated at 100 Hz in controlsd while these records are ~1 Hz, so
        # offline reconstruction could only establish a negative (peakAngErr never reached the 8 deg
        # threshold, so it wasn't that branch). Pass-through, no re-derivation here.
        "trigWhy": ev.get("trigWhy"),
        "trigRatio": ev.get("trigRatio"),
        "trigDesLat": ev.get("trigDesLat"),
        # N5 review fix: True only when controlsd's 5 s defensive ceiling force-emitted this event
        # (still armed, no clean clear) -- durationS is then a known-truncated lower bound.
        "capped": bool(ev.get("capped", False)),
        "frameId": ev.get("frameId"), "modelLogMonoTime": ev.get("modelLogMonoTime"),
        "srcT": ev.get("t"),
        "trace": ev.get("trace") or [],
        # lat/lon/bearing stay the CURRENT (emit-time) GPS position -- same "close-tick snapshot"
        # convention as log_take_control_alert's "alert" records (see that docstring's NOTE), fine
        # for PLACING the episode on the map since position barely moves in a few seconds. heading is
        # the one field that needs the ONSET-time value (bearing_at_onset above) -- direction, unlike
        # position, can rotate 45-90 deg in that same window. I4 review fix: null (not "N") when no
        # GPS fix was current at the buffered onset sample (gps_at_onset).
        "gps": self._cur_lat is not None and self._cur_lon is not None,
        "lat": self._cur_lat, "lon": self._cur_lon, "bearing": self._cur_bearing,
        "heading": _heading_if_fixed(bearing_at_onset, gps_at_onset),
      }
      if clock_bad(now_wall):
        rec["clockBad"] = True
      self._append_event(rec)
    except Exception:
      pass

  # takecontrol2pnw: as of this feature, ces_events.jsonl carries THREE overlapping steering-related
  # record families for one physical wheel-saturation episode — each with its own trigger and its own
  # notion of "duration", so don't treat them as contradictory when digging through the log:
  #   "ev":"steer"      — steerlimit-log2pnw's ~1 Hz breadcrumb (see _steer_log_step above), samples
  #                        steer-limit state on a fixed clock regardless of any alert/event.
  #   "ev":"steerEvent" — steerevent2pnw's controlsd-side flight recorder, its OWN edge trigger
  #                        (slAngSat-based) sampled at up to 100 Hz around the peak.
  #   "ev":"alert","name":"steerSaturated" — THIS method: selfdrived's actual "Take Control" / "Turn
  #                        Exceeds Steering Limit" alert (the undershoot+turning+lac.saturated block
  #                        in selfdrived.py's update_events()) — the real driver-facing event, edge-
  #                        triggered + hold-off-debounced (see selfdrived._log_take_control_edge).
  def log_take_control_alert(self, payload: dict) -> None:
    """takecontrol2pnw: append one discrete {"ev":"alert","name":"steerSaturated",...} record to
    ces_events.jsonl. Called ONLY by selfdrived (selfdrive/selfdrived/selfdrived.py
    _log_take_control_edge/_emit_take_control_alert), on the rare rising/closing edge of ITS OWN
    steerSaturated ("Take Control" / "Turn Exceeds Steering Limit") decision — never every tick, and
    unconditionally regardless of CESMode/_enabled (the alert firing is a selfdrived fact, not a CES
    decision).

    `payload` is plain Python (str/float/bool/list/None only — no cereal/capnp objects), already
    built by the caller; this method does NOT decide anything and does NOT re-derive the trigger. It
    only enriches the record with fields ces already has cached from its own ~1 Hz _read_map()
    refresh (kept fresh regardless of CESMode by _steer_log_step/_read_params — steerlimit-log2pnw/
    steertele2pnw) and appends via the existing _append_event writer: GPS (self._cur_lat/_cur_lon/
    _cur_bearing) and steer-limit state (self._sl_ang_des/_sl_ang_act/_sl_ang_err/_sl_lat_dem/
    _sl_lat_max). No fresh mem-param read happens here.

    NOTE on phase="end" records: vEgo/gps/lat/lon/bearing/slAng*/slLat*/otherEvents here are all a
    CLOSE-TICK snapshot (~1 s after the episode, delayed by selfdrived's hold-off) — they are NOT the
    episode's peak/trigger-moment state. durationS (the one field that IS about the episode itself)
    is computed by the caller from the last frame the alert actually fired, not the close tick.

    See also the "three overlapping steering record families" note above CESController (or search
    ces_events.jsonl for '"ev":"steer"' / '"ev":"steerEvent"' / '"ev":"alert"') — this "alert" record
    is one of three different steering-related record types that can appear for one physical
    saturation episode; they have different triggers/durations and are not contradictory.

    Defensive: a malformed/partial payload or any field-access failure degrades to 'nothing logged'
    rather than raising — the caller already wraps this call in try/except as well (belt + braces),
    since this ultimately runs from selfdrived's ~100 Hz control loop."""
    try:
      now_wall = time.time()  # noqa: TID251 -- wall clock, rare edge-triggered append only
      v_ego = payload.get("vEgo")
      if v_ego is not None and not math.isfinite(v_ego):
        v_ego = None  # takecontrol2pnw: NaN/inf would serialize to a bare (invalid-JSON) NaN token
      rec = {
        "t": round(now_wall, 1),
        "ev": "alert", "name": payload.get("name", "steerSaturated"), "phase": payload.get("phase"),
        "car": self._car, "cesMode": self._mode,
        "vEgo": v_ego,
        "gps": self._cur_lat is not None and self._cur_lon is not None,
        "lat": self._cur_lat, "lon": self._cur_lon, "bearing": self._cur_bearing,
        # leadrate2pnw: heading was simply MISSING from this record (not gated too strictly -- there
        # was no "heading" key at all), so it always read back as None even on a valid fix, unlike the
        # "steer"/"tick"/"adopt" records taken the same moment. Same helper/gate those use: null only
        # on a genuine no-fix, not a blanket None.
        "heading": _heading_if_fixed(self._cur_bearing, self._cur_lat is not None and self._cur_lon is not None),
        # steerlimit-log2pnw fields — already-cached, no fresh read here (see docstring above).
        "slAngDes": self._sl_ang_des, "slAngAct": self._sl_ang_act, "slAngErr": self._sl_ang_err,
        "slLatDem": self._sl_lat_dem, "slLatMax": self._sl_lat_max,
        # co-active onroadEvents this frame (e.g. steerTempUnavailable/ldw), for correlation.
        "otherEvents": payload.get("otherEvents") or [],
      }
      if payload.get("durationS") is not None:
        rec["durationS"] = payload["durationS"]
      if clock_bad(now_wall):
        rec["clockBad"] = True
      self._append_event(rec)
    except Exception:
      pass

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
        self._icbm_dir = None
        self._icbm_gate = None          # icbmmapfirst2pnw
        self._icbm_map_reach = None
        self._icbm_ep.reset()           # icbmrestore2pnw: forced Chill / no data ends any episode
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
      # icbmrestore2pnw: the episode machine owns the ceiling latch now. While a CAP episode is
      # active, curve binding is judged against ITS latched ceiling (v_set follows the tapped-down
      # stock set); during restore/idle a fresh cap latches at the current set.
      ep_ceiling = self._icbm_ep.ceiling if self._icbm_ep.phase == "cap" else None
      ref = ep_ceiling if ep_ceiling is not None else sig["v_set"]
      # icbmtrack2pnw: ICBM-only capped scale (19:58:37Z root cause — the tiered sweeper end
      # inflated a raw 64.9 mph curve to an effective 107 mph, so it never bound vs set 90).
      far_v, far_dist = icbm_far_map_candidate(self._map_targets, self._cur_lat, self._cur_lon,
                                               sig["v_ego"], ref, icbm_map_eff_scale,
                                               self._veh.icbm_map_scale, self._veh.icbm_firm_decel)
      # icbmmapfirst2pnw start-policy gates (drive 2026-07-12): apply ONLY when a decision would
      # START a new episode — a running cap episode (phase 'cap', incl. its S-gap clear debounce)
      # continues with the full candidate set exactly as before ("an episode may continue").
      ttc = float(sig.get("time_to_curve", float("inf")) or float("inf"))
      in_curve = icbm_in_curve(sig.get("lat_accel_now", 0.0), sig.get("curve_lat_accel_vision", 0.0), ttc)
      starting = self._icbm_ep.phase != "cap"
      self._icbm_gate = None
      self._icbm_map_reach = None
      if starting:
        map_reach = icbm_map_reach(self._map_targets, self._cur_lat, self._cur_lon)
        self._icbm_map_reach = round(map_reach, 0)
      target, _, self._icbm_src = icbm_curve_target(
        sig["v_ego"], sig["v_set"], sig.get("map_target_v", 0.0),
        sig.get("map_target_dist", float("inf")), ep_ceiling, icbm_map_eff_scale,
        vis_v, vis_dist,
        map_scale=self._veh.icbm_map_scale, firm_decel=self._veh.icbm_firm_decel,
        far_v=far_v, far_dist=far_dist, track=True)
      if target is not None and starting:
        if in_curve:
          # driver rule 2: NEVER begin a new dec episode while already loaded in the curve — hold
          # the current set (any source: braking mid-curve was the field complaint).
          self._icbm_gate = "inCurve"
          target, self._icbm_src = None, None
        elif self._icbm_src == "vis" and not icbm_vision_may_start(vis_dist, ttc, map_reach):
          # driver rule 1 (MAP-FIRST): live map coverage over this stretch -> the map verdict
          # (incl. "no slowdown needed") is authoritative for anticipatory slowing; and a too-late
          # vision curve must not start taps at the curve. mapd-dead fallback: reach 0.0 -> only
          # the time gate applies, vision keeps initiating where the map is blind.
          self._icbm_gate = "visCovered" if vis_dist <= map_reach else "visLate"
          # a still-binding map/far candidate may start the episode instead of the gated vision one
          target, _, self._icbm_src = icbm_curve_target(
            sig["v_ego"], sig["v_set"], sig.get("map_target_v", 0.0),
            sig.get("map_target_dist", float("inf")), ep_ceiling, icbm_map_eff_scale,
            0.0, float("inf"),
            map_scale=self._veh.icbm_map_scale, firm_decel=self._veh.icbm_firm_decel,
            far_v=far_v, far_dist=far_dist, track=True)
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
        # rain2pnw: driver-selected wet-weather curve margin (same reduction the Tesla/VTSC gets).
        target = max(target - self._veh.rain_penalty_ms(), 0.0)
      # icbmrestore2pnw: run the episode machine — it forwards caps unchanged ('dec'), enters the
      # bounded GUARDED restore when the curve clears, and hard-aborts on any driver-intent signal.
      driver_pedal = bool(sig.get("gas")) or bool(sig.get("brake"))
      # icbmmapfirst2pnw: hand the episode the binding candidate's DISTANCE (apex-passage detection
      # for the early restore) and the in-curve flag (restore entry deferral / restore pause).
      src_dist = {"map": sig.get("map_target_dist", float("inf")),
                  "vis": vis_dist, "far": far_dist}.get(self._icbm_src)
      pub_target, direction = self._icbm_ep.step(now, target, sig["v_set"],
                                                 self._stock_set, self._stock_on, driver_pedal,
                                                 cap_dist=src_dist, v_ego=sig["v_ego"], in_curve=in_curve)
      self._icbm_ceiling = self._icbm_ep.ceiling
      self._icbm_dir = direction
      if direction == "inc":
        self._icbm_src = "restore"      # telemetry: the restore phase is its own source label
      self._icbm_last_target = round(pub_target, 2) if pub_target is not None else None
      if pub_target is not None:
        # JSON params take a DICT (params_pyx serializes it; a pre-dumped string raises TypeError —
        # Gemini review catch that would have silently killed every publish)
        # ceiling: the episode latch, or — for an episode-less forwarded cap (ACC off / pedal
        # pressed, episode reset) — the driver's current set: the exact pre-restore reduce-only
        # semantics. The executor clamps target <= ceiling either way.
        ceil_pub = self._icbm_ceiling if self._icbm_ceiling is not None else sig["v_set"]
        payload = {"target": round(pub_target, 2), "ceiling": round(ceil_pub, 2), "ts": time.time()}  # noqa: TID251 -- wall clock heartbeat shared with the executor
        if direction == "inc":
          payload["dir"] = "inc"        # explicit marker: executor's inc path is ONLY reachable via this
        self.mem_params.put_nonblocking("IcbmTarget", payload)
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
      "stp": False,   # stophold2pnw (B): keep the key present on the noData path too
    }
    tele["mode"] = mode
    # stopintent2pnw: the adopt record must show WHICH entry path fired — decide_active's reason
    # cannot know the state machine took the fast path, so override from the sm status (it holds
    # "stopIntent" exactly for the cycle the preemption happened).
    # ces2core2pnw: when CES2 is LIVE, the CES2 core's status is the authoritative reason instead.
    if mode == "experimental":
      if self._ces2_live and self._ces2_reason:
        tele["reason"] = self._ces2_reason
      # standstill2pnw: the hold tags too — while a hold is the ONLY thing keeping Experimental,
      # decide_active's reason reads "chill", which made the 11:34 flapping forensics blind to WHY
      # the mode was held. Overlay/log now shows the state machine's authoritative reason.
      # cesnochill2pnw: "stopLatch" added — the hard latch's own override tag, so field telemetry
      # can never again show "reason": "chill" while mode is actually experimental.
      elif self._sm.status() in ("stopIntent", "stopHold", "standstillHold", "stopLatch"):
        tele["reason"] = self._sm.status()
    # ces2core2pnw shadow A/B channel: CES2's would-be decision + graded urgency + the cumulative
    # divergence-edge counter, on EVERY record (the replay/acceptance dataset).
    tele["ces2Mode"] = self._ces2_mode
    tele["ces2Reason"] = self._ces2_reason
    tele["ces2Urgency"] = round(float(self._ces2_urg), 3)
    tele["ces2Div"] = self._ces2_div.count
    tele["ces2Live"] = self._ces2_live
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
      tele["icbmSrc"] = self._icbm_src           # curveslow-lightning: "map"/"vis"/"restore" source
      tele["icbmDir"] = self._icbm_dir           # icbmrestore2pnw: "dec" capping / "inc" restoring
      tele["icbmSet"] = self._stock_set
      tele["icbmOn"] = self._stock_on
      tele["icbmGate"] = self._icbm_gate         # icbmmapfirst2pnw: start suppressed & why (or None)
      tele["mapReach"] = self._icbm_map_reach    # icbmmapfirst2pnw: map coverage m (0/None = blind)

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
      # stophold2pnw (C): wall-clock heartbeat — the overlay's NO-SIGNAL dead-man compares this
      # against its own clock, so a dead/silent publisher becomes VISIBLE (the driver's "silence
      # must be loud" rule, born of the 2026-07-12 false-silence investigation).
      tele["ts"] = round(time.time(), 2)  # noqa: TID251 -- wall clock heartbeat shared with the overlay
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
    now_wall = time.time()  # noqa: TID251 -- wall clock, for route/time correlation
    # steerpower2pnw: pure functions, computed from fields already read by _read_map() (see __init__).
    # N1 review fix: `vego` above is the "or 0.0"-defaulted value the `hwy` coarse guess intentionally
    # tolerates -- but that same fallback fed straight into achLat made a genuine no-data record
    # (_publish_status's "noData" sentinel tele, or any tele missing "vEgo") indistinguishable from
    # "driving perfectly straight" (k_actl * 0**2 = 0.0). achLat instead uses the RAW reading, None
    # when there isn't one (missing key, or the noData sentinel) -- matching controlsd's
    # _ach_lat_ms2(k_actl, CS.vEgo), which never fakes a 0.0 v_ego in the first place.
    raw_vego = tele.get("vEgo")
    no_data = tele.get("reason") == "noData"
    ach_lat = None if (no_data or raw_vego is None) else _ach_lat(self._sl_k_actl, raw_vego)
    rec = {
      "t": round(now_wall, 1),
      "ev": kind, "mode": tele.get("mode"), "reason": tele.get("reason"), "button": int(self._button),
      "vEgo": tele.get("vEgo"), "vSet": tele.get("vSet"), "aEgo": tele.get("aEgo"), "gas": tele.get("gas"),
      "accelZone": tele.get("accelZone"),
      # stophold2pnw (B): raw model stop intent — red-vs-green is decidable from the breadcrumb now
      "stp": tele.get("stp"),
      # stophold2pnw (D): car identity (shadow is no longer a car discriminator — alpha-long A/B)
      "car": self._car,
      "curvePct": tele.get("curvePct"), "curveSrc": tele.get("curveSrc"),
      # lowspeedcurve2pnw: raw vision-curve trigger inputs (why did curvePct stay 0? — Hwy 99
      # 2026-07-13). None on the noData path, same as every other tele passthrough.
      "visLat": tele.get("visLat"), "visTtc": tele.get("visTtc"), "blnk": tele.get("blnk"),
      "mapV": tele.get("mapV"), "mapDist": tele.get("mapDist"), "mapPts": tele.get("mapPts"),
      # mapd220-2pnw PHASE 1: mapd v2.2.0 highwayClass/conditionalSpeedLimit telemetry. PURE
      # OBSERVATION — this is the dataset a later phase validates curve-tiering/freeway-floor/
      # posted-limit gating against (docs/MAPD-V220-UPGRADE.md); nothing here changes any target.
      # condSpdLim truncated to bound the JSONL row size (raw OSM text, unbounded upstream).
      "hwyClass": self._hwy_class,
      "condSpdLim": (self._cond_spd_lim[:80] if self._cond_spd_lim else ""),
      "waySel": self._way_sel,          # waysel2pnw
      "wayOff": self._way_off,          # waysel2pnw
      "dRel": tele.get("dRel"), "vLead": tele.get("vLead"),
      # vtsctele2pnw: explicit lead-present bool + gap time (s) + lead speed delta (m/s)
      # leadrate2pnw: "hasLead" is an ALIAS of the same value as "lead" below (both come straight from
      # s["has_lead"] via decision_telemetry's tele dict) -- added under this name so tick/adopt and
      # the CES-off "steer" record (_steer_log_step) share one field name for the same fact.
      "lead": tele.get("lead"), "hasLead": tele.get("lead"), "gapS": tele.get("gapS"), "dV": tele.get("dV"),
      "gps": tele.get("gps"), "lat": self._cur_lat, "lon": self._cur_lon, "bearing": self._cur_bearing,
      "spdLim": round(self._speed_limit, 1), "hwy": bool(hwy),
      # VTSC applied cap + state (from the VTSCStatus mem param) — without this channel the 2026-07-06
      # I-84 gas-override cluster couldn't be attributed (VTSC/MTSC vs CES) from the log alone.
      "vtscCap": self._vtsc_cap, "vtscState": self._vtsc_state, **getattr(self, "_vtsc_tele", {}),
        **getattr(self, "_sa_tele", {}),
      # vtsctele2pnw: the penalty components VTSC actually applied (Lightning penalty m/s, the road
      # pitch it used, apex turn direction L/R) — 2026-07-12 westbound over-slow forensics needed
      # these and had to infer them.
      "vtscPen": self._vtsc_pen, "vtscPitch": self._vtsc_pitch, "vtscDir": self._vtsc_dir,
      # bsm2pnw: blind-spot booleans (carState.left/rightBlindspot) — liveness evidence for the
      # lane-change BSM gate; expect these to flip as traffic passes on real drives.
      "bsL": self._bs_l, "bsR": self._bs_r,
      # lanecenter2pnw telemetry: lane-centering trim status (from LaneCenterStatus) — display/log
      # only, same as vtscCap/vtscState above. lcGate explains why lcCorr isn't (fully) applied on
      # any given tick ("ok" = acting; see lane_centering.py's _finish_status for the full code list).
      "lcCorr": self._lc_corr, "lcAct": self._lc_act, "lcGate": self._lc_gate, "lcErr": self._lc_err,
      "lcP1": self._lc_p1, "lcP2": self._lc_p2, "lcS1": self._lc_s1, "lcS2": self._lc_s2,
      "lcYStd": self._lc_ystd, "lcW": self._lc_w,
      # steerlimit-log2pnw telemetry: steering-limit status (from SteerLimitStatus) — display/log
      # only, same as lc* above. PURE OBSERVATION: never gates or alters any control value. See
      # docs/STEERING-LIMITS.md for what each field means and how to read them together.
      "slCurvLim": self._sl_curv_lim, "slSafetyLim": self._sl_safe_lim,
      "slAngDes": self._sl_ang_des, "slAngAct": self._sl_ang_act, "slAngErr": self._sl_ang_err,
      "slLatDem": self._sl_lat_dem, "slLatMax": self._sl_lat_max, "slCurvMax": self._sl_curv_max,
      "slSat": self._sl_sat,
      # steertele2pnw: capability-analysis pair — slLatAct disambiguates openpilot-engaged steering
      # from manual maneuvering (angDes/angAct freeze together when False), slAngSat is the un-fused
      # angle-only half of slSat (slCurvLim already isolates the curvature half). See the
      # steer_limit_status comment block in controlsd.py for the exact derivation of each.
      "slLatAct": self._sl_lat_active, "slAngSat": self._sl_ang_sat,
      # fordkappalog2pnw: commanded vs achieved curvature (1/m, Ford wire convention positive=left) —
      # the empirical saturation signal to characterize this truck's real curvature limit. Display/log
      # only, same as sl* above. See docs/STEERING-LIMITS.md "Ford curvature interface" section.
      "slKCmd": self._sl_k_cmd, "slKActl": self._sl_k_actl, "slKErr": self._sl_k_err,
      # steerpower2pnw: LOGGING ONLY — delivered lateral accel (m/s^2, signed) + 8-pt compass heading,
      # to measure the truck's true hands-off steering capability by direction (see module docstring
      # near _ach_lat/_compass).
      # I4 review fix: null (not "N") when there's no current GPS fix, rather than _compass() reading
      # a no-fix-defaulted 0.0 bearing as true north.
      "achLat": round(ach_lat, 3) if ach_lat is not None else None,
      "heading": _heading_if_fixed(self._cur_bearing, self._cur_lat is not None and self._cur_lon is not None),
      # icbm2pnw: steering angle + driver-override flag (lateral quality forensics), and the shadow
      # marker — True on the Lightning where the planner path never actuates (ICBM may).
      "strAng": self._str_ang, "strPrs": self._str_prs, "shadow": self._shadow,
      "mdlEndX": round(float(tele.get("mdlEndX") or 0.0), 1),
      # ces2core2pnw shadow A/B: CES2 would-be mode/reason, graded stop urgency, cumulative
      # divergence edges vs v1, and whether CES2 was LIVE (deciding) for this record.
      "ces2Mode": tele.get("ces2Mode"), "ces2Reason": tele.get("ces2Reason"),
      "ces2Urg": tele.get("ces2Urgency"), "ces2Div": tele.get("ces2Div"),
      "ces2Live": tele.get("ces2Live"),
      # icbm2pnw closed-loop trace: published curve target (m/s, None = ICBM idle), latched driver
      # ceiling, the truck's reported stock set speed + engagement. icbmT stepping the stockSet down
      # in consecutive ticks = executor taps landing.
      "icbmT": self._icbm_last_target, "icbmC": self._icbm_ceiling, "icbmSrc": self._icbm_src,
      "icbmDir": self._icbm_dir,   # icbmrestore2pnw: "inc" rows in ces_events = restore taps
      "stockSet": self._stock_set, "stockOn": self._stock_on,
      # icbmmapfirst2pnw: start-gate + map coverage forensics (why vision did NOT initiate; whether
      # mapd was alive — mapReach 0/None with mapPts 0 = the mapd-outage signature).
      "icbmGate": self._icbm_gate, "mapReach": self._icbm_map_reach,
      # greenlead2pnw: detector state per record ("idle"/"armed"/"fired" — the arming trail) plus
      # a ONE-SHOT event marker ("green"/"lead"/"leadMoving") on the record that follows an actual
      # firing, else None. Replaces the old "greenLight" field, which logged the (always-truthy)
      # state and read as true in 13,427/13,433 records on the 2026-07-13 drive — the arming trail
      # and the event are now separate, meaningful channels.
      "glSt": self._gl.state,
      "glEv": self._gl_ev_pending,
    }
    self._gl_ev_pending = None   # consumed by exactly one record (adopt or tick, whichever is next)
    # stophold2pnw (D): mark (never drop) records written before the clock is plausibly synced —
    # the dead-RTC boot wrote a 2025-11-25-stamped record that corrupted the 07-12 gap analysis.
    if clock_bad(now_wall):
      rec["clockBad"] = True
    return rec

  def _append_event(self, rec: dict) -> None:
    """Append one JSON line to the persistent CES_EVENT_LOG (append-only, outside the overlay so it
    survives reboot + swaglog rotation). Best-effort; never breaks control — but repeated failure
    is no longer SILENT (stophold2pnw C: the 07-12 investigation burned hours proving a silence
    that wasn't; a genuinely dying writer must announce itself in swaglog)."""
    if not self._event_log_ok:
      return
    try:
      try:
        if os.path.getsize(CES_EVENT_LOG) > CES_EVENT_LOG_MAX_BYTES:
          rotate_event_log(CES_EVENT_LOG, CES_EVENT_LOG_GENERATIONS)
      except OSError:
        pass                                                # no file yet / stat race -> just append
      with open(CES_EVENT_LOG, "a") as f:
        f.write(json.dumps(rec) + "\n")
      self._append_fail = 0
    except Exception:
      # ~1-2 writes/s: warn once at 10 consecutive failures, then re-warn every ~600 (~5-10 min) —
      # loud enough to see, throttled enough to never flood swaglog. The write stays best-effort.
      self._append_fail += 1
      if self._append_fail == 10 or self._append_fail % 600 == 0:
        try:
          cloudlog.error(f"ces_pnw: ces_events append FAILING ({self._append_fail} consecutive) — telemetry trail is dark ({CES_EVENT_LOG})")
        except Exception:
          pass
