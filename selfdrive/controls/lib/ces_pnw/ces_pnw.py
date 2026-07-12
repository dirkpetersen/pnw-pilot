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
                                                                      MAP_SOURCE_HORIZON_M,
                                                                      MAP_SCALE_MIN)

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


def icbm_map_reach(points, cur_lat, cur_lon, horizon_m=ICBM_MAP_HORIZON_M) -> float:
  """icbmmapfirst2pnw: how far ahead the published mapd path COVERS the road (m) — the farthest
  valid path point within mapd's horizon. 0.0 = no usable coverage (mapd down / no data / GPS lost /
  a stale path we have driven > horizon_m away from). Used by the MAP-FIRST start gate: a vision
  candidate INSIDE this reach is on a stretch the map has judged, so the map verdict (including "no
  slowdown needed") wins for STARTING episodes. Distances are recomputed from the CURRENT position
  every call, so a dead mapd's last path decays out of coverage as we drive on (mapd-liveness
  fallback: vision regains the right to initiate). NaN-guarded like the other scanners. Pure."""
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
ICBM_MAP_EFF_SCALE_CAP = MAP_SCALE_MIN


def icbm_map_eff_scale(tv_raw: float) -> float:
  """icbmtrack2pnw: the ICBM-only effective scale for a RAW map target speed — the shared tiered
  ramp, capped at its tight-curve end (see ICBM_MAP_EFF_SCALE_CAP root-cause note). Pure."""
  return min(C.tiered_map_scale(tv_raw), ICBM_MAP_EFF_SCALE_CAP)


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
    self._min_target = None             # lowest cap target commanded this episode
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
    self._t0 = None
    self._clear_t0 = None
    self._hold_set0 = None
    self._last_stock = None
    self._last_t = None
    self._last_cap_dist = None
    self._last_cap_vego = 0.0
    self._apex_passed = False
    self._late_tap_set = None

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
      # DEC ALWAYS WINS. A cap during RESTORE cancels the restore episode entirely and re-latches
      # at the CURRENT set; a cap during CAP just continues the episode (ceiling untouched).
      if self.phase != "cap":
        self.reset()
        self.phase = "cap"
        self.ceiling = float(v_set)
        self._min_target = float(cap_target)
      else:
        self._min_target = float(cap_target) if self._min_target is None else min(self._min_target, float(cap_target))
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
      return cap_target, "dec"

    if self.phase == "cap":
      # clear DEBOUNCE: hold silent (ceiling retained, executor stale-stops within 2 s) until the
      # curve has stayed clear for clear_delay_s. A detection flicker or an S-curve gap therefore
      # keeps the ORIGINAL ceiling instead of starting a restore and re-latching lower.
      if self._clear_t0 is None:
        self._clear_t0 = now
        self._hold_set0 = stock_set     # snapshot: we go SILENT now, so nothing of ours moves the set
        self._late_tap_set = None       # fresh clear window: any previous absorbed tap is void
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
    # vtsctele2pnw: VTSC penalty components actually applied (from VTSCStatus) — logging only
    self._vtsc_pen = self._vtsc_pitch = None
    self._vtsc_dir = ""
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
    # icbmrestore2pnw: the cap->clear->restore episode machine + the current direction for telemetry
    self._icbm_ep = IcbmEpisode()
    self._icbm_dir = None                  # "dec" while capping, "inc" while restoring, None idle
    self._stock_set = 0.0
    self._stock_on = False
    # pullaway2pnw: stateful evidence for the below-floor lead-pull-away exception (monotonic
    # dRel rise + model-stop recency). Feeds sig["lead_opening"]; pure logic stays in decide_active.
    self._pullaway_trk = PullAwayTracker()
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
      # vtsctele2pnw: penalty components VTSC actually applied this cycle (Lightning hump penalty
      # m/s, road pitch rad it used, apex turn direction "L"/"R"/"") — so over/under-slow curve
      # forensics read the real inputs instead of inferring descent/left multipliers after the fact.
      pen, pitch = vt.get("pen"), vt.get("pitch")
      self._vtsc_pen = round(float(pen), 2) if pen is not None else None
      self._vtsc_pitch = round(float(pitch), 4) if pitch is not None else None
      self._vtsc_dir = str(vt.get("dir") or "")
    except Exception:
      self._vtsc_cap = self._vtsc_state = None
      self._vtsc_pen = self._vtsc_pitch = None
      self._vtsc_dir = ""

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
    }
    tele["mode"] = mode
    # stopintent2pnw: the adopt record must show WHICH entry path fired — decide_active's reason
    # cannot know the state machine took the fast path, so override from the sm status (it holds
    # "stopIntent" exactly for the cycle the preemption happened).
    if mode == "experimental" and self._sm.status() == "stopIntent":
      tele["reason"] = "stopIntent"
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
      # vtsctele2pnw: explicit lead-present bool + gap time (s) + lead speed delta (m/s)
      "lead": tele.get("lead"), "gapS": tele.get("gapS"), "dV": tele.get("dV"),
      "gps": tele.get("gps"), "lat": self._cur_lat, "lon": self._cur_lon, "bearing": self._cur_bearing,
      "spdLim": round(self._speed_limit, 1), "hwy": bool(hwy),
      # VTSC applied cap + state (from the VTSCStatus mem param) — without this channel the 2026-07-06
      # I-84 gas-override cluster couldn't be attributed (VTSC/MTSC vs CES) from the log alone.
      "vtscCap": self._vtsc_cap, "vtscState": self._vtsc_state,
      # vtsctele2pnw: the penalty components VTSC actually applied (Lightning penalty m/s, the road
      # pitch it used, apex turn direction L/R) — 2026-07-12 westbound over-slow forensics needed
      # these and had to infer them.
      "vtscPen": self._vtsc_pen, "vtscPitch": self._vtsc_pitch, "vtscDir": self._vtsc_dir,
      # bsm2pnw: blind-spot booleans (carState.left/rightBlindspot) — liveness evidence for the
      # lane-change BSM gate; expect these to flip as traffic passes on real drives.
      "bsL": self._bs_l, "bsR": self._bs_r,
      # icbm2pnw: steering angle + driver-override flag (lateral quality forensics), and the shadow
      # marker — True on the Lightning where the planner path never actuates (ICBM may).
      "strAng": self._str_ang, "strPrs": self._str_prs, "shadow": self._shadow,
      "mdlEndX": round(float(tele.get("mdlEndX") or 0.0), 1),
      # icbm2pnw closed-loop trace: published curve target (m/s, None = ICBM idle), latched driver
      # ceiling, the truck's reported stock set speed + engagement. icbmT stepping the stockSet down
      # in consecutive ticks = executor taps landing.
      "icbmT": self._icbm_last_target, "icbmC": self._icbm_ceiling, "icbmSrc": self._icbm_src,
      "icbmDir": self._icbm_dir,   # icbmrestore2pnw: "inc" rows in ces_events = restore taps
      "stockSet": self._stock_set, "stockOn": self._stock_on,
      # icbmmapfirst2pnw: start-gate + map coverage forensics (why vision did NOT initiate; whether
      # mapd was alive — mapReach 0/None with mapPts 0 = the mapd-outage signature).
      "icbmGate": self._icbm_gate, "mapReach": self._icbm_map_reach,
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
