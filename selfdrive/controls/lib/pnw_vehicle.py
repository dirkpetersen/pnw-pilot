"""
pnw_vehicle — the PNW fleet's capability view over CarParams.

ONE place that knows which cars support which pnw features. Feature code asks about CAPABILITIES
(veh.ces_shadow, veh.stock_acc_buttons, ...), never about fingerprints — adding a car means adding
it here, not hunting string comparisons across the tree (driver directive 2026-07-11).

Pure and defensive: works with a capnp CarParams reader, the structs dataclass, or None (returns
all-False capabilities), so UI code can call it before a car is fingerprinted.
"""
import json
import math
import os
import stat
import time

from cereal import log  # tightfollow2pnw: LongitudinalPersonality enum for the aggressive-only check
from openpilot.common.swaglog import cloudlog  # rule2fixes2pnw: _load_curve_config failures are logged
from openpilot.selfdrive.controls.lib.ces_pnw.icbm_shape import MODES as SHAPE_MODES   # curveshape2pnw
from openpilot.selfdrive.controls.lib.drive_helpers import lat_accel_target_source   # curvebrain2pnw: curve_lat_a's clip

# curveslow-lightning: mph<->m/s (no numpy — a plain float; numpy leaked into a capnp setter and
# crash-looped card, 2026-07-11).
_MPH_TO_MS = 0.44704
# tightfollow2pnw v2: Lightning-only Aggressive T_FOLLOW override (s), vs. the shared upstream 1.25s.
# v1 used a FLAT 1.0 and was reverted after on-road measurement made things worse (see the
# tight_aggressive_follow comment in __init__). v2 is deliberately shallower (1.15 = 0.10s tighter
# than baseline, not 0.25s), and is applied only through the stability gate + slew below.
_TIGHT_AGGRESSIVE_T_FOLLOW = 1.15
# The upstream Aggressive T_FOLLOW, used only as a fallback when the caller doesn't pass the live
# baseline (keeps this module free of a long_mpc import -- pnw_vehicle is imported by UI code too).
_TIGHT_AGGRESSIVE_BASELINE = 1.25
# Stability gate. The lead must look CALM for this long before any tightening starts; ANY excursion
# restarts the clock from zero (fail toward the looser baseline, never toward the tighter target).
_TIGHT_STABLE_MIN_S = 5.0
# Thresholds are applied to the KALMAN-FILTERED lead fields only. This is the whole lesson of the v1
# revert: radard's `vRel` is RAW (get_RadarState_from_vision sets it from lead_v_rel_pred, unfiltered)
# while `vLeadK`/`aLeadK` come out of Track's KF1D or radarless2pnw's VisionLeadFilter. Gating on raw
# vRel would chatter on exactly the per-frame model noise v1 tripped over, so relative speed is
# DERIVED here as (vLeadK - v_ego) rather than read from lead.vRel.
_TIGHT_MAX_ABS_VREL = 2.0            # m/s, |vLeadK - v_ego|
_TIGHT_MAX_ABS_ALEAD = 0.5           # m/s^2, |aLeadK| (same 0.5 rule Track uses to call accel constant)
# A step in dRel means a DIFFERENT car (cut-in / lead swap), not a manoeuvre by the car we were
# tracking -- restart the stability clock rather than carrying credit across two vehicles.
_TIGHT_LEAD_JUMP_M = 8.0
# Slew the override rather than stepping it, so the gate flipping can never hand the MPC a
# discontinuous target (a step in T_FOLLOW is a step in desired following distance).
_TIGHT_SLEW_S_PER_S = 0.05
_TIGHT_DT_DEFAULT = 0.05             # planner tick (DT_MDL) when the caller doesn't pass one

# standstillsoft2pnw: the launch accel ramp rises to this ceiling by launch_v (slightly above the stock
# ~2.0 m/s^2 max so the cap never binds the normal accel envelope at/after launch_v).
_LAUNCH_TOP = 2.5

# Persistent, outside-git tunable for the Lightning curve-speed penalty (survives auto-update, same
# discipline as dm_config.py on the dm-variable branch). Missing/malformed -> hardcoded defaults.
# Schema: {"lightning": {<any subset of the _CURVE_DEFAULTS keys below>}} — unknown keys ignored,
# every known key clamped to _CURVE_BOUNDS. descentcurve2pnw extends the schema with the descent
# guard / left factor / overspeed margin / ICBM map scale + firm decel; all pre-existing keys keep
# working unchanged.
CURVE_CONFIG_PATH = "/data/pnw/curve.json"
_CURVE_CONFIG_MAX_BYTES = 64 * 1024

# Hardcoded defaults = the INTENDED Lightning ramp (this is a wanted behavior change, not neutral).
# Direction (field-calibrated, drives/2026-07-11 op-long/VTSC takeover analysis): the EPS torque
# demand to hold a curve scales with v^2, so the Lightning's steering-authority DEFICIT vs the Tesla
# appears at SPEED — it washes out of FAST highway sweepers, not slow tight corners (ample authority
# when slow). So the penalty grows WITH the binding curve target speed:
#   target <= low_v_mph  -> penalty_min_mph slower (slow tight corners barely need it)
#   target >= high_v_mph -> penalty_max_mph slower (fast sweepers need the full ~10 mph)
#   linear between. The three takeovers analyzed had VTSC binding caps 29.1 / 32.1 / 33.0 m/s
#   (65 / 72 / 74 mph) — high_v_mph=65 puts the full penalty in force across that whole band.
# Applies ONLY to the Lightning (Tesla path returns 0.0, byte-unchanged).
_CURVE_DEFAULTS = {
  "penalty_min_mph": 1.0,     # slow corners: ample steering authority
  "penalty_max_mph": 5.0,     # the mid-speed washout zone (driver-approved 2026-07-11 iteration 3)
  "penalty_taper_mph": 1.5,   # long fast gentle sweepers: carry speed again (iteration 3 feedback)
  "low_v_mph": 30.0,
  "peak_lo_v_mph": 45.0,
  "peak_hi_v_mph": 62.0,
  "taper_v_mph": 75.0,
  # descentcurve2pnw (2026-07-11 evening: two DOWNHILL LEFT-curve washouts under op-long; stock-ACC
  # 90 mph silent-ICBM run -- the LEFT labels were inverted, see left_factor). All Lightning-only via lightning_curve_slow; Tesla path returns 0.0 /
  # neutral values from every accessor below.
  "descent_gain": 8.0,          # per rad of downhill pitch: a 5% grade (~0.05 rad) -> +40% penalty.
                                #   Physics: on a descent gravity eats the regen decel budget, so the
                                #   truck arrives at/above the cap — enter the curve slower instead.
  "descent_pitch_cap": 0.12,    # rad (~12% grade); |pitch| beyond this adds no more (IMU-noise bound)
  "penalty_cap_mph": 15.0,      # hard cap on the TOTAL penalty after all multipliers (never more)
  "left_factor": 1.0,           # multiplier on LEFT curves only. NEUTRAL (1.0) since 2026-09-24: the
                                #   owner removed the left-only penalty -- left and right curves are
                                #   treated the same. It was 1.15, justified by "both 2026-07-11
                                #   washouts were downhill LEFTS" (adverse US road crown + weak EPS).
                                #   That premise was WRONG: tools/washouts.py labelled direction with
                                #   strAng < 0 = left, but on this truck steeringAngleDeg > 0 = LEFT,
                                #   so 28 of the 35 binding washouts carried the wrong `dir`, and the
                                #   18:12 PT washout cited as a downhill left was a right-hander
                                #   (drives/2026-09-24/vision-left-flag-check.md). Knob kept so
                                #   curve.json can still set it (bounds [1.0, 1.5]); the direction
                                #   plumbing + telemetry (icbmLeft / icbmLeftSrc, vtscDir) stay.
  "overspeed_margin_mph": 2.0,  # VTSC: v_ego above the applied cap by this -> friction-brake escalation
  "map_scale": 0.92,            # ICBM: scale mapd's suggested speeds DOWN before the binding test —
                                #   OSM curve speeds (mapV 99-112 mph on the I-90 sweepers) are
                                #   calibrated for stronger-steering cars; Tesla-appropriate, too
                                #   generous for the Lightning.
  "icbm_firm_decel": 1.4,       # m/s^2 assumed approach decel for LARGE speed drops (stock ACC does
                                #   the actual braking; this only shapes the tap-start envelope)
  # icbmslow2pnw (driver 2026-09-17 "button control management also took me down to 38 mph... it's
  # going too slow"): the fraction of a MAP candidate's OWN rated speed that the Lightning curve
  # penalty may not push the ICBM target below. 1.0 = "ICBM never commands below mapd's rating for
  # this curve"; 0.0 = floor off (the pre-icbmslow2pnw behaviour).
  #
  # WHY: the penalty above was field-calibrated 2026-07-11 when the ICBM map path used a FLAT 1.35
  # effective scale, so a map candidate arrived as 1.35 * 0.92 = 1.242x its raw rating and the
  # penalty ate part of that inflation -- on tight/moderate curves the July target still landed
  # ABOVE the raw rating. icbmcurve2pnw (2026-08-11) then dropped the tight end of that scale to
  # 1.10 for a different and correct reason (a 50 mph curve inflated past a 55 mph cruise was being
  # rejected as a candidate), which makes the composite 1.10 * 0.92 = 1.012 -- essentially raw. The
  # penalty was never re-calibrated against that, so since 2026-08-11 it has been subtracting a
  # 5 mph inflation margin from a number that no longer carries one. Measured across 88 replayable
  # ICBM episodes (drives 2026-08-12..09-17, curvature from slKActl): the truck was commanded to a
  # median 1.41 m/s^2 of lateral accel where mapd's own rating implies 1.67 and the design target is
  # 2.50. This floor restores the pre-08-11 property WITHOUT restoring the pre-08-11 candidacy bug.
  "icbm_map_floor_frac": 1.0,
  # curvefix2pnw Part B (owner 2026-09-24, OR-34 11:19 "slowed down a little bit too much"): the same floor for a
  # VISION candidate -- the fraction of vision's OWN speed for the curve (icbm_vision_apex, solved at A_LAT_TARGET
  # 2.5 m/s^2, the Lightning's lateral target) that the curve penalty may not push the ICBM target below. 1.0 = floor
  # on (default); 0.0 = the pre-curvefix behaviour. On OR-34 the camera read the curve correctly (visLat 2.72-2.79 at
  # 72-73 mph = k 0.0026, measured 0.00256), its 2.5 m/s^2 speed was ~69.2 mph, and the hump took ~3 mph more off
  # (66, while the curve needed 69.9). The owner's rule: slow for a sharp curve, but only to the level required.
  # Like the map floor it bounds the BASE hump only: the descent and left-factor extras still come off below it.
  "icbm_vis_floor_frac": 1.0,
  # curvelead2pnw: the lateral load ICBM may let a tracked lead car pace the truck to through a curve
  # (v <= sqrt(this / curvature), curvature = the TIGHTER of map geometry and vision). 2.5 is what the
  # driver himself chose on the 2026-09-13 ramps (2.32 / 2.96 m/s^2 measured). 0.0 turns lead pacing off.
  "icbm_lead_lat_accel": 2.5,
  # restorehold2pnw (2026-09-21 21:21 PT, Tumwater S-bend): an ICBM RESTORE (SET+ back to the driver's set) may not
  # rise above the curve ahead's safe speed sqrt(this / curvature). 2.5 = VTSC_A_LAT, the curve model ICBM already
  # solves every map curve with (icbmKV is sqrt(2.5/k)). It is also VISION's trigger bar (0 % false triggers at 2.5 in
  # the 2026-09-24 calibration). Holding only withholds acceleration: never a SET-, never a lower set. 0.0 = the whole
  # hold off (polyline included).
  "icbm_restore_hold_lat_accel": 2.5,
  # restorehold2pnw (owner 2026-09-24, fewer false holds): the MAP POLYLINE triggers a hold only at this higher load --
  # it is the noisier witness (31 % of its fires at 2.8 land on roads measured below 2.0 m/s^2). 0.0 = polyline off.
  "icbm_restore_hold_poly_lat_accel": 2.8,
  # standstillsoft2pnw (2026-07-14): gentle standstill LAUNCH accel ramp — the red-light follow-launch
  # "lurch" fix. Cap the accel out of a dead stop to launch_accel, ramping to the normal envelope by
  # launch_v. (Root cause: a lead crept forward at a red, op-long launched to follow at ~2.0 m/s^2.)
  "launch_accel": 0.6,          # m/s^2 accel ceiling at a dead stop (soft; stock jumped to ~2.0 = lurch)
  "launch_v_mph": 9.0,          # by this speed the launch cap has lifted to the normal accel envelope
  # curvedblive2pnw: the learned curve database (curvedb v2) supplies ICBM's map/far curve target LIVE, both
  # directions (ces_pnw/curvedb_live.py). ON by default on the Lightning -- owner decision 2026-09-24, which
  # overrides the design's N >= 60 shadow gate. 0 = OFF: ICBM exactly as without the DB. Read at selfdrived start.
  "curvedb_v2_live": 1.0,
  # curvedblive2pnw: the lateral accel (m/s^2) the curve DB turns a row's curvature into a speed with,
  # v = sqrt(this / k). DEFAULT 2.5 -- owner decision 2026-09-24 11:25 PT after the Corvallis->Albany drive ("it slowed
  # down a little bit too much in one of the curves"), up from 2.2 set earlier the same day. Known consequence: at 2.5
  # the DB alone does not slow the 09-21 Tumwater left curve (the restore hold still limits it).
  # 0 = mapd's own map_curve_target_lat_a, read live from MapdSettings (2 on the truck). A number in [1.0, 3.5] =
  # use that. Read at selfdrived start; logged as curvedb_v2_cfg and in every record (cdb2A / cdb2ASrc).
  "curvedb_v2_lat_a": 2.5,
  # curveshape2pnw (owner answer 4, 2026-09-24): the lateral target (m/s^2) a map candidate is priced at when the
  # polyline and mapd's own curvature agree (ces_pnw/icbm_shape.py): v = sqrt(this / mean(k_poly, k_mapd)).
  "icbm_shape_lat_a": 2.5,
  # ... and for a shape price at or above 70 mph. The PSCM reached limit level 2 at 2.46 (OR-34 11:19, 73 mph) and
  # 2.92 (11:18, 77 mph); the owner kept 2.5 there too, as its own knob so it can be lowered alone.
  "icbm_shape_lat_a_70": 2.5,
}
# curveshape2pnw: the measured-shape stage's MODE is a string, so it is NOT in _CURVE_DEFAULTS (whose loop float()s every
# key -- a string there would throw the WHOLE curve.json back to defaults). "shadow" = compute and log, change nothing
# (owner answer 7: shadow first). "off" or 0 = the stage does nothing. "live" = it prices ICBM's map candidates.
SHAPE_MODE_DEFAULT = "shadow"


def _parse_shape_mode(raw) -> tuple[str, str]:
  """(mode, why) for curve.json's lightning.icbm_shape. An unrecognised value is NOT silently read as anything the
  owner did not write: it falls back to the inert default (shadow) and `why` says so -- CESController logs it as an
  error at start."""
  if isinstance(raw, str) and raw.strip().lower() in SHAPE_MODES:
    return raw.strip().lower(), "curve.json"
  if isinstance(raw, (int, float)) and not isinstance(raw, bool) and raw == 0:
    return "off", "curve.json"
  return SHAPE_MODE_DEFAULT, f"INVALID curve.json icbm_shape {raw!r:.24} -> {SHAPE_MODE_DEFAULT}"
# sane clamp bounds per key (penalties [0,15] mph so a penalty can NEVER invert to a speed-up; speeds
# [10,80] mph). A bad config can only ever land inside these -> control code stays safe.
_CURVE_BOUNDS = {
  "penalty_min_mph": (0.0, 15.0),
  "penalty_max_mph": (0.0, 15.0),
  "penalty_taper_mph": (0.0, 15.0),
  "low_v_mph": (10.0, 80.0),
  "peak_lo_v_mph": (10.0, 80.0),
  "peak_hi_v_mph": (10.0, 80.0),
  "taper_v_mph": (10.0, 90.0),
  # descentcurve2pnw: every bound chosen so a bad config can only DEGRADE toward neutral, never
  # invert. descent_gain >= 0 + left_factor >= 1.0 -> the multipliers are always >= 1 (never shrink
  # the base penalty into a speed-up); map_scale <= 1.0 -> ICBM can never inflate a map speed;
  # icbm_firm_decel <= 1.5 -> the assumed approach decel stays a gentle-braking assumption.
  "descent_gain": (0.0, 20.0),
  "descent_pitch_cap": (0.0, 0.20),
  "penalty_cap_mph": (0.0, 15.0),
  "left_factor": (1.0, 1.5),
  "overspeed_margin_mph": (0.5, 10.0),
  "map_scale": (0.5, 1.0),
  "icbm_firm_decel": (0.8, 1.5),
  # icbmslow2pnw: [0, 1] -- the floor can never sit ABOVE the map's own rating for the curve (1.0),
  # and 0.0 is the documented off switch. A bad config therefore degrades toward "more slowing",
  # never toward "less than mapd asked for".
  "icbm_map_floor_frac": (0.0, 1.0),
  "icbm_vis_floor_frac": (0.0, 1.0),   # curvefix2pnw: same reasoning -- never above vision's own 2.5 m/s^2 speed
  # curvelead2pnw: [0, 3.0] -- 0 disables lead pacing; the ceiling sits at the driver's own p90 on country
  # roads (2.96 m/s^2): a bad config can never let a lead pace the truck above his own p90.
  "icbm_lead_lat_accel": (0.0, 3.0),
  # restorehold2pnw: [0, 3.2] -- 0 turns the hold off; a LOWER value holds MORE (never a slowdown either way). The
  # ceiling is openpilot's own live lateral limit at 73 mph on the Tumwater curve (slLatMax 3.21): a config can
  # never let a restore accelerate toward a curve predicted past the point where steering saturates.
  "icbm_restore_hold_lat_accel": (0.0, 3.2),
  "icbm_restore_hold_poly_lat_accel": (0.0, 3.2),   # same ceiling, same reason
  # launch_accel in [0.2, 2.0]: never so low the truck can't move, never above the stock ~2.0 max ->
  # this cap can only ever SOFTEN a launch, never make it harsher. launch_v [3, 25] mph.
  "launch_accel": (0.2, 2.0),
  "launch_v_mph": (3.0, 25.0),
  "curvedb_v2_live": (0.0, 1.0),   # curvedblive2pnw: a switch; >= 0.5 is ON
  "curvedb_v2_lat_a": (0.0, 3.5),  # curvedblive2pnw: 0 = mapd's A; else clamped to [1.0, 3.5] by the property
  # curveshape2pnw: never below 1.5 (a price far slower than any measured need) nor above openpilot's own 3.2 lateral
  # ceiling (the restorehold2pnw bound).
  "icbm_shape_lat_a": (1.5, 3.2),
  "icbm_shape_lat_a_70": (1.5, 3.2),
}


# rain2pnw: driver-selected wet-weather curve margin — an ADDITIVE speed reduction applied in curves,
# on TOP of each car's base curve behavior. UNLIKE the Lightning EPS penalty below, rain reduces grip
# on EVERY car, so it applies to BOTH the Tesla and the Lightning with the SAME reduction (driver
# directive 2026-07-12: "they can have the same rain reduction for now"; the cars' DRY curve tuning
# stays separate and untouched — this is purely a driver-opted margin the driver dials in when it
# rains). Live tier = UI param RainMode (0=None, 1=Light, 2=Heavy); the magnitudes are the device-
# local tunable below (defaults 3 / 5 mph), same discipline as curve.json / dm.json.
RAIN_CONFIG_PATH = "/data/pnw/rain.json"
_RAIN_CONFIG_MAX_BYTES = 64 * 1024
_RAIN_DEFAULTS = {"light_mph": 3.0, "heavy_mph": 5.0}
# clamp [0,15] mph: a bad config can only ever be a small reduction — never a speed-up, never a huge brake.
_RAIN_BOUNDS = {"light_mph": (0.0, 15.0), "heavy_mph": (0.0, 15.0)}


def _clamp(x: float, lo: float, hi: float) -> float:
  return lo if x < lo else (hi if x > hi else x)


def _load_rain_config() -> dict:
  """Read /data/pnw/rain.json defensively (mirror of _load_curve_config): a flat
  {"light_mph": .., "heavy_mph": ..}, unknown keys ignored, each clamped to [0,15] mph. NEVER raises
  (runs in control code) — missing/malformed -> the hardcoded 3/5 mph defaults.

  deleterrain2pnw (Rule 2, same fix as rule2fixes2pnw's _load_curve_config): fallbacks unchanged; a MISSING
  file stays silent (the documented default), an unusable/unreadable/malformed file is a cloudlog.error naming
  the path and the error, and a clamped or NaN value is one cloudlog.warning per load naming the keys."""
  cfg = dict(_RAIN_DEFAULTS)
  path = RAIN_CONFIG_PATH
  try:
    st = os.stat(path)
    if not stat.S_ISREG(st.st_mode) or st.st_size > _RAIN_CONFIG_MAX_BYTES:
      cloudlog.error(f"pnw_vehicle: {path} IGNORED (not a regular file, or {st.st_size} B > " +
                     f"{_RAIN_CONFIG_MAX_BYTES} B) -- using the default rain margins")
      return cfg
    with open(path) as f:
      data = json.load(f)
    if not isinstance(data, dict):
      cloudlog.error(f"pnw_vehicle: {path} IGNORED (expected a JSON object, got {type(data).__name__}) " +
                     "-- using the default rain margins")
    nan_keys = []
    if isinstance(data, dict):
      for k in cfg:
        if k in data:
          v = float(data[k])
          if v == v:                          # NaN guard (NaN != NaN)
            cfg[k] = v
          else:
            nan_keys.append(k)
    if nan_keys:
      cloudlog.warning(f"pnw_vehicle: {path}: NaN ignored for {nan_keys} -- those keys keep their defaults")
  except FileNotFoundError:
    return dict(_RAIN_DEFAULTS)                # no file = the documented default margins; not an error
  except Exception as e:
    cloudlog.error(f"pnw_vehicle: {path} unreadable/malformed ({type(e).__name__}: {e}) -- the WHOLE file is " +
                   "ignored, using the default rain margins")
    return dict(_RAIN_DEFAULTS)                # any failure -> defaults, never raise
  clamped = []
  for k, (lo, hi) in _RAIN_BOUNDS.items():
    v = _clamp(cfg[k], lo, hi)
    if v != cfg[k]:
      clamped.append(f"{k}={cfg[k]}->{v}")
    cfg[k] = v
  if clamped:
    cloudlog.warning(f"pnw_vehicle: {path}: out-of-bounds value(s) clamped: {', '.join(clamped)}")
  return cfg


def _load_curve_config() -> dict:
  """Read /data/pnw/curve.json defensively (os.stat gate: regular file, <= 64 KiB), overlaying only
  known numeric keys onto the defaults, clamping each to sane bounds. NEVER raises (runs in control
  code) — any missing/bad/malformed input -> the hardcoded default ramp.

  rule2fixes2pnw (Rule 2): every fallback used to be silent. Fallbacks unchanged; now a MISSING file stays
  silent (it is the documented default), while an unusable/unreadable/malformed file is a cloudlog.error
  naming the path and the error, and a clamped or NaN value is one cloudlog.warning per load naming the keys.

  curveshape2pnw: the string knob icbm_shape is parsed OUTSIDE the numeric loop (which float()s every key it
  visits -- a string there would throw the whole file back to defaults) and carries its own icbm_shape_why."""
  cfg = dict(_CURVE_DEFAULTS, icbm_shape=SHAPE_MODE_DEFAULT, icbm_shape_why="default")
  path = CURVE_CONFIG_PATH
  try:
    st = os.stat(path)
    if not stat.S_ISREG(st.st_mode) or st.st_size > _CURVE_CONFIG_MAX_BYTES:
      cloudlog.error(f"pnw_vehicle: {path} IGNORED (not a regular file, or {st.st_size} B > " +
                     f"{_CURVE_CONFIG_MAX_BYTES} B) -- using the default curve ramp")
      return cfg
    with open(path) as f:
      data = json.load(f)
    light = data.get("lightning", {}) if isinstance(data, dict) else {}
    if not isinstance(data, dict) or not isinstance(light, dict):
      cloudlog.error(f"pnw_vehicle: {path} IGNORED (expected {{\"lightning\": {{...}}}}, got {type(data).__name__}" +
                     (f" with lightning={type(light).__name__}" if isinstance(data, dict) else "") +
                     ") -- using the default curve ramp")
    nan_keys = []
    if isinstance(light, dict):
      for k in _CURVE_DEFAULTS:           # the numeric keys only (curveshape2pnw: icbm_shape is a string)
        if k in light:
          v = float(light[k])
          if v == v:                      # NaN guard (NaN != NaN)
            cfg[k] = v
          else:
            nan_keys.append(k)
      if "icbm_shape" in light:
        cfg["icbm_shape"], cfg["icbm_shape_why"] = _parse_shape_mode(light["icbm_shape"])
    if nan_keys:
      cloudlog.warning(f"pnw_vehicle: {path}: NaN ignored for {nan_keys} -- those keys keep their defaults")
  except FileNotFoundError:
    return dict(_CURVE_DEFAULTS, icbm_shape=SHAPE_MODE_DEFAULT, icbm_shape_why="default")   # no curve.json: normal
  except Exception as e:
    cloudlog.error(f"pnw_vehicle: {path} unreadable/malformed ({type(e).__name__}: {e}) -- the WHOLE file is " +
                   "ignored, using the default curve ramp")
    # any failure -> defaults, never raise. curveshape2pnw: the stage mode says it came from a failed read.
    return dict(_CURVE_DEFAULTS, icbm_shape=SHAPE_MODE_DEFAULT,
                icbm_shape_why=f"default (curve.json unreadable: {type(e).__name__})")
  clamped = []
  for k, (lo, hi) in _CURVE_BOUNDS.items():
    v = _clamp(cfg[k], lo, hi)
    if v != cfg[k]:
      clamped.append(f"{k}={cfg[k]}->{v}")
    cfg[k] = v
  if clamped:
    cloudlog.warning(f"pnw_vehicle: {path}: out-of-bounds value(s) clamped: {', '.join(clamped)}")
  return cfg


# curvebrain2pnw 1/8 (docs/SHARED-CURVE-BRAIN-DESIGN.md s3.4, s5.2): the shared curve brain's per-car lateral target and
# the Tesla VTSC's consumption mode. NOTHING CONSUMES EITHER YET -- stage 1 only reads, bounds and logs them.
# The Tesla's knobs live in their own curve.json section, {"tesla": {"curve_lat_a": .., "curve_brain": ..}}, parsed by
# _load_tesla_curve_config() and only on the car that has the capability. The "lightning" section and its loader are
# untouched, and _load_curve_config never reads "tesla": one device serves both cars, and nothing written for the Tesla
# may move a Lightning value.
CURVE_LAT_A_DEFAULT = 2.5          # m/s^2: today's VTSC_A_LAT -- the Lightning's target, and any car without its own
# curvebrain2pnw A (owner decision 2026-09-28, design D1): the Tesla's curve SPEED target is 4.0 m/s^2 ("the tesla can do
# 4.0"; a human reached 5.16, openpilot-steered 3.61 verified). THIS IS A SPEED TARGET, NOT A STEERING CAPABILITY. Steering
# stays bounded elsewhere and a higher target can run into those limits: (1) the lataccel2pnw lateral cap, which
# curve_lat_a() keeps as min(target, lat_accel_target(v) - 0.3), so on the SHARED schedule 4.0 is EFFECTIVELY 3.7 at 70 mph
# and 2.7 at >= 80 mph (and flat 3.0 - 0.3 = 2.7 without a valid schedule file). latcar2pnw/latmargin2pnw: the Tesla has its
# own "cars" entry in lataccel_limits.json ([[50,5],[60,5],[70,4],[80,3.6]]); a value from a valid per-car entry is NOT reduced
# by the 0.3 margin (the entry is chosen at the clamp), so with it (1) never binds below the steering clamp;
# (2) the Tesla vehicle model's steering-angle limit (~14.4
# deg at 70 mph in the Terwilliger left curve: 2026-09-28 22:35 the applied angle stalled at the model limit, the car
# drifted wide, "Turn Exceeds Steering Limit"); (3) the EPS torque abort (2.7-3.8 Nm). Nothing here may command a speed
# that assumes more lateral acceleration than those allow -- STRICTLY, only (1) is kept: curve_lat_a()'s min() clips at (1) and at
# (2) = the model clamp itself (owner 2026-09-29: zero margin, see CURVE_STEER_CEILING_OWNER), so (2) still lets the car reach the
# angle limit, and on adverse camber that limit delivers only ~3.0; known bad curves are lowered by the per-curve overrides
# (ces_pnw/curve_brain.py). (3) is a limit nothing here can see.
TESLA_CURVE_LAT_A_DEFAULT = 4.0    # m/s^2 (was 2.8 until 2026-09-28)
_TESLA_CURVE_LAT_A_BOUNDS = (2.0, 4.5)
CURVE_LAT_CLIP_MARGIN = 0.3        # m/s^2 the Tesla's target stays below openpilot's own SHARED lateral clip (lat_accel_target);
#                                    not applied to a valid per-car entry (latmargin2pnw)
CURVE_BRAIN_MODES = ("off", "shadow", "lower", "raise")   # "raise" includes "lower" (design s5.2)
# curvebrain2b2pnw: the Tesla's DEFAULT mode is ACTING -- "lower" (T2: lower-only against VTSC's own cap, at rows of the curve DB
# with authority). Owner, 2026-09-28: "i don't want shadow, make the tesla use the curve db". This overrides the design's
# "shadow first" (SHARED-CURVE-BRAIN-DESIGN s5.2 / D3) and CLAUDE.md's "new toggles default OFF": the G-T2 gate (a Tesla shadow
# drive, D4) has NOT been run, which the commit message and the report say. curve.json is the override and the kill switch:
# {"tesla": {"curve_brain": "shadow"}} (compute + log, change nothing) or "off" take effect within ~1-2 s, hot-reloaded. A
# MISSING file or section gets this acting default (logged at start: curve_brain_cfg); a PRESENT-but-corrupt file or invalid mode
# at start gets CURVE_BRAIN_CORRUPT (shadow) + cloudlog.error; a bad file DURING a drive is not applied (last config kept).
# To take the acting default back out, change this one constant to "shadow".
CURVE_BRAIN_DEFAULT = "lower"
# A PRESENT but corrupt tesla section / unreadable file / invalid mode at start is NOT "no config": it falls to this (compute + log,
# change nothing) with the cloudlog.error and `why`. Only a MISSING file or MISSING section gets the acting default. A file DELETED
# mid-drive reverts to the acting default (event curve_brain_cfg_reload file=absent) -- a documented footgun.
CURVE_BRAIN_CORRUPT = "shadow"
# vtscfloor2pnw release-later (owner 2026-09-29): on the Raven, VTSC keeps its cap frozen instead of climbing while the curve target is
# still tightening after the state machine released. DEFAULT ON. Kill switch: {"tesla": {"vtsc_release_later": false}} in curve.json
# (hot-reloaded with the rest of the tesla section). A present-but-unreadable file at start turns it OFF (today's behaviour).
VTSC_RELEASE_LATER_DEFAULT = True
# vtscnotch2pnw (owner-approved 2026-09-30): on the Raven, VTSC measures the map-curve minimum-slowdown notch (and the gate that uses it) from
# min(set, vEgo) instead of the set speed alone, so a car below its set speed (just after a raise, or in traffic) is asked for the full ~10 mph
# of slowdown, not ~5 (2026-09-30 22:32:27 bend: set 85->90, vEgo 85, the cap bottomed at vEgo and the car did not slow). DEFAULT ON.
# Kill switch: {"tesla": {"vtsc_notch_vego": false}} in curve.json (hot-reloaded with the rest of the tesla section); same fail-safe as
# release-later: a present-but-unreadable file / bad value turns it OFF = today's set-relative notch.
VTSC_NOTCH_VEGO_DEFAULT = True
# vtschold2pnw (owner-approved 2026-10-01): on the Raven, VTSC's apex HOLD no longer freezes the cap against a curve that needs a lower one: while
# holding, the cap follows the decel envelope of the binding curve when the car is above that curve's safe speed, and is lowered to the car's own
# speed (never below it, so it never brakes) when the car is at safe speed. It only ever LOWERS the held cap. DEFAULT ON. Kill switch:
# {"tesla": {"vtsc_hold_envelope": false}} in curve.json (hot-reloaded with the rest of the tesla section); same fail-safe as release-later: a
# present-but-unreadable file / bad value turns it OFF = today's frozen hold.
VTSC_HOLD_ENVELOPE_DEFAULT = True
# dbfirst2pnw (owner design 2026-10-01): on the Raven, VTSC's map minimum-slowdown notch (set - 4.5 m/s on ANY OSM-flagged curve) is replaced by
# "the curve DB first". Where the curve DB COVERS the flagged point in the driving direction (a row with authority: driven, measured), the OSM
# fold is not applied there -- the DB (the curve brain's need) and vision decide. Where it does not, the flat notch becomes a cap of
# vtsc_constants.UNCOVERED_CURVE_LIMIT_RATIO x the posted limit, relaxed when the polyline and the camera both measure the curve mild. Needs the
# curve brain ACTING (curve_brain lower/raise): in shadow/off, or with no fresh coverage, today's notch stands. DEFAULT ON. Kill switch:
# {"tesla": {"vtsc_db_first": false}} in curve.json (hot-reloaded with the rest of the tesla section); same fail-safe as the other switches: a
# present-but-unreadable file / bad value turns it OFF = today's flat notch.
VTSC_DB_FIRST_DEFAULT = True
# stopgo2pnw (owner 2026-10-03): on the Raven with op-long, while stopped behind a lead that is demonstrably pulling away, the planner lets the
# lead-aware MPC decide the stop instead of the e2e model's (often stuck) shouldStop -- see stopgo_pnw.py. DEFAULT ON. Kill switch:
# {"tesla": {"stop_go_handoff": false}} in curve.json (hot-reloaded with the rest of the tesla section); same fail-safe as the other switches: a
# present-but-unreadable file / bad value turns it OFF = today's behaviour (byte-identical planner path).
STOP_GO_DEFAULT = True
CURVE_CFG_POLL_S = 1.0             # curve.json's tesla section is re-checked (one os.stat) at most this often

# curvebrain2b2pnw A2..A5: the STEERING ceiling on the lateral acceleration a curve speed may assume. The Tesla's angle is
# clipped in opendbc/car/lateral.py apply_steer_angle_limits_vm at get_max_angle_vm(v), the angle the vehicle model needs for
# opendbc.car.tesla.values.CarControllerParams.ANGLE_LIMITS.MAX_LATERAL_ACCEL = ISO 3.0 + g * AVERAGE_ROAD_ROLL(0.06) = 3.5886
# m/s^2 at every speed. OWNER DECISION 2026-09-29: "set it to 3.6, no panda flash needed for now" -> the ceiling is
# min(CURVE_STEER_CEILING_OWNER 3.6, that clamp) = the clamp itself, ZERO margin. The owner was told what that means: the vehicle-
# model angle limit clips commands at exactly this point; delivered lateral acceleration is ~3.6 on flat roads but only ~3.0 on
# adverse camber (2026-09-28 22:35: the applied angle stalled at the clamp, 14.42 deg at 70.3 mph, and the car delivered 3.0-3.1,
# running wide). (Fable F1 had asked for 2.8 = ISO - 0.2; the owner overrode it.) That is only acceptable because KNOWN bad curves
# get their own lower limit: /data/pnw/curve_overrides.json (curve_brain.Overrides), and if that file is missing or invalid the
# brain drops to CURVE_STEER_FALLBACK (2.8) for everything until it is valid. The owner's 4.0 (tesla.curve_lat_a) stays his
# REQUESTED target and this clip makes it <= 3.6. The import is done ONCE, at PnwVehicle construction on the Tesla, never in a control
# loop (a ~30 ms import in selfdrived's 100 Hz loop is a commIssue). Unreadable constant -> CURVE_STEER_FALLBACK + cloudlog.error.
CURVE_STEER_CEILING_OWNER = 3.6
CURVE_STEER_FALLBACK = 2.8
_STEER_LAT_CEILING_FALLBACK = CURVE_STEER_FALLBACK

# ovrcar2pnw (OWNER DECISION 2026-09-29): the per-curve override file /data/pnw/curve_overrides.json (schema v2, curve_brain.Overrides)
# holds PER-CAR limits and is used ONLY by these two cars, named by their opendbc platform ("according to the car database":
# opendbc.car.tesla.values.CAR.TESLA_MODEL_S_HW3, opendbc.car.ford.values.CAR.FORD_F_150_LIGHTNING_MK1 -- a test pins both against
# opendbc). An entry naming any other platform is invalid; a car not in this tuple never opens the file (curve_override_platform
# is None). This tuple is the ONLY place the names are listed: feature code reads PnwVehicle.curve_override_platform.
CURVE_OVERRIDE_PLATFORMS = ("TESLA_MODEL_S_HW3", "FORD_F_150_LIGHTNING_MK1")
CURVE_OVERRIDE_V1_PLATFORM = CURVE_OVERRIDE_PLATFORMS[0]   # a schema-v1 entry (top-level a_max, no `cars`) is read as Tesla-only
_steer_lat_ceiling_cache: float | None = None


def _tesla_steer_lat_ceiling() -> float:
  """min(CURVE_STEER_CEILING_OWNER, the Tesla vehicle-model clamp's MAX_LATERAL_ACCEL) in m/s^2. Cached."""
  global _steer_lat_ceiling_cache
  if _steer_lat_ceiling_cache is None:
    v = _STEER_LAT_CEILING_FALLBACK
    try:
      from opendbc.car.tesla.values import CarControllerParams
      lim = CarControllerParams.ANGLE_LIMITS.MAX_LATERAL_ACCEL
      if not (isinstance(lim, (int, float)) and math.isfinite(lim) and 2.0 <= lim <= 5.0):
        raise ValueError(f"MAX_LATERAL_ACCEL {lim!r} is not plausible")
      v = min(CURVE_STEER_CEILING_OWNER, float(lim))
    except Exception as e:
      cloudlog.error("pnw_vehicle: the Tesla steering lateral ceiling could not be read from opendbc " +
                     f"({type(e).__name__}: {e}) -- using the fixed {_STEER_LAT_CEILING_FALLBACK} m/s^2")
    _steer_lat_ceiling_cache = v
  return _steer_lat_ceiling_cache


def _parse_curve_brain_mode(raw) -> str | None:
  """curve.json's tesla.curve_brain -> a mode, or None when it is not one (the caller says so)."""
  if isinstance(raw, str) and raw.strip().lower() in CURVE_BRAIN_MODES:
    return raw.strip().lower()
  if isinstance(raw, (int, float)) and not isinstance(raw, bool) and raw == 0:
    return "off"
  return None


def _load_tesla_curve_config() -> dict:
  """curvebrain2pnw: curve.json's "tesla" section -> {"curve_lat_a", "curve_brain", "why"}. NEVER raises.

  Its own parse of the same file, key by key: a bad value costs that key its default, never the section. Rule 2, the
  _load_curve_config discipline: a missing file or no "tesla" section is the documented default and silent; an
  unusable / unreadable / malformed file, a section that is not an object, a value that is not a finite number and a
  mode that is not one are each a cloudlog.error naming the path; a value clamped into bounds is a cloudlog.warning.
  `why` says where the values came from ("default", "curve.json", or "INVALID ..." -- CESController logs that as an
  error at start too)."""
  cfg = {"curve_lat_a": TESLA_CURVE_LAT_A_DEFAULT, "curve_brain": CURVE_BRAIN_DEFAULT, "why": "default",
         "release_later": VTSC_RELEASE_LATER_DEFAULT, "notch_vego": VTSC_NOTCH_VEGO_DEFAULT,
         "hold_envelope": VTSC_HOLD_ENVELOPE_DEFAULT, "db_first": VTSC_DB_FIRST_DEFAULT,
         "stop_go": STOP_GO_DEFAULT}
  path = CURVE_CONFIG_PATH
  try:
    st = os.stat(path)
    if not stat.S_ISREG(st.st_mode) or st.st_size > _CURVE_CONFIG_MAX_BYTES:
      cloudlog.error(f"pnw_vehicle: {path} IGNORED for the tesla section (not a regular file, or {st.st_size} B > " +
                     f"{_CURVE_CONFIG_MAX_BYTES} B) -- the curve brain uses its defaults")
      cfg["why"] = "default (curve.json unusable)"
      cfg["curve_brain"] = CURVE_BRAIN_CORRUPT
      cfg["release_later"] = False   # vtscfloor2pnw: unreadable file -> release-later OFF
      cfg["notch_vego"] = False      # vtscnotch2pnw: unreadable file -> set-relative notch (today's)
      cfg["hold_envelope"] = False   # vtschold2pnw: unreadable file -> frozen hold (today's)
      cfg["db_first"] = False      # dbfirst2pnw: unreadable file -> flat notch (today's)
      cfg["stop_go"] = False       # stopgo2pnw: unreadable file -> e2e stop authoritative (today's)
      return cfg
    with open(path) as f:
      data = json.load(f)
  except FileNotFoundError:
    return cfg                                   # no curve.json: normal
  except Exception as e:
    cloudlog.error(f"pnw_vehicle: {path} unreadable/malformed ({type(e).__name__}: {e}) -- the curve brain's tesla " +
                   "section uses its defaults")
    cfg["why"] = f"default (curve.json unreadable: {type(e).__name__})"
    cfg["curve_brain"] = CURVE_BRAIN_CORRUPT
    cfg["release_later"] = False   # vtscfloor2pnw: unreadable file -> release-later OFF
    cfg["notch_vego"] = False      # vtscnotch2pnw: unreadable file -> set-relative notch (today's)
    cfg["hold_envelope"] = False   # vtschold2pnw: unreadable file -> frozen hold (today's)
    cfg["db_first"] = False      # dbfirst2pnw: unreadable file -> flat notch (today's)
    cfg["stop_go"] = False       # stopgo2pnw: unreadable file -> e2e stop authoritative (today's)
    return cfg
  # The key parse is guarded like the read above (Fable F1): json.load turns a 309+ digit literal into an int that
  # math.isfinite / float() cannot convert (OverflowError), and a raise here would take down EVERY process that builds
  # a Tesla PnwVehicle. Any failure is logged and the whole section falls back to its defaults.
  try:
    tesla = data.get("tesla") if isinstance(data, dict) else None
    if tesla is None:
      return cfg                                   # no section: normal
    if not isinstance(tesla, dict):
      cloudlog.error(f"pnw_vehicle: {path} tesla section IGNORED (expected an object, got {type(tesla).__name__}) -- " +
                     "the curve brain uses its defaults")
      cfg["why"] = f"INVALID curve.json tesla section ({type(tesla).__name__}) -> defaults"
      cfg["curve_brain"] = CURVE_BRAIN_CORRUPT
      cfg["release_later"] = False   # vtscfloor2pnw: unreadable file -> release-later OFF
      cfg["notch_vego"] = False      # vtscnotch2pnw: unreadable file -> set-relative notch (today's)
      cfg["hold_envelope"] = False   # vtschold2pnw: unreadable file -> frozen hold (today's)
      cfg["db_first"] = False      # dbfirst2pnw: unreadable file -> flat notch (today's)
      cfg["stop_go"] = False       # stopgo2pnw: unreadable file -> e2e stop authoritative (today's)
      return cfg
    bad, clamped = [], []
    if "curve_lat_a" in tesla:
      raw = tesla["curve_lat_a"]
      if isinstance(raw, (int, float)) and not isinstance(raw, bool) and math.isfinite(raw):
        v = _clamp(float(raw), *_TESLA_CURVE_LAT_A_BOUNDS)
        if v != raw:
          clamped.append(f"curve_lat_a={raw}->{v}")
        cfg["curve_lat_a"] = v
      else:
        bad.append(f"curve_lat_a {raw!r:.24} is not a finite number -> {TESLA_CURVE_LAT_A_DEFAULT}")
    if "curve_brain" in tesla:
      mode = _parse_curve_brain_mode(tesla["curve_brain"])
      if mode is None:
        bad.append(f"curve_brain {tesla['curve_brain']!r:.24} is not one of {'/'.join(CURVE_BRAIN_MODES)} -> " +
                   CURVE_BRAIN_CORRUPT)
        cfg["curve_brain"] = CURVE_BRAIN_CORRUPT
      else:
        cfg["curve_brain"] = mode
    if "vtsc_release_later" in tesla:
      # vtscfloor2pnw kill switch: true / false (0 / 1 accepted). Anything else is a `bad` key like any other: at START the section
      # is not honoured (curve_brain falls to CURVE_BRAIN_CORRUPT) and release-later is OFF; MID-DRIVE the reload is rejected and the
      # LAST GOOD config (including this switch) is kept -- see refresh_curve_brain_cfg.
      raw = tesla["vtsc_release_later"]
      if isinstance(raw, bool):
        cfg["release_later"] = raw
      elif isinstance(raw, (int, float)) and raw in (0, 1):
        cfg["release_later"] = bool(raw)
      else:
        cfg["release_later"] = False
        bad.append(f"vtsc_release_later {raw!r:.24} is not true/false -> OFF")
    if "vtsc_notch_vego" in tesla:
      # vtscnotch2pnw kill switch, same contract as vtsc_release_later above: true / false (0 / 1); anything else is a `bad` key,
      # the switch goes OFF (today's behaviour) and the section is not honoured (mid-drive: last good config kept).
      raw = tesla["vtsc_notch_vego"]
      if isinstance(raw, bool):
        cfg["notch_vego"] = raw
      elif isinstance(raw, (int, float)) and raw in (0, 1):
        cfg["notch_vego"] = bool(raw)
      else:
        cfg["notch_vego"] = False
        bad.append(f"vtsc_notch_vego {raw!r:.24} is not true/false -> OFF")
    if "vtsc_hold_envelope" in tesla:
      # vtschold2pnw kill switch, same contract as vtsc_release_later / vtsc_notch_vego above: true / false (0 / 1); anything else is a
      # `bad` key, the switch goes OFF (today's frozen hold) and the section is not honoured (mid-drive: last good config kept).
      raw = tesla["vtsc_hold_envelope"]
      if isinstance(raw, bool):
        cfg["hold_envelope"] = raw
      elif isinstance(raw, (int, float)) and raw in (0, 1):
        cfg["hold_envelope"] = bool(raw)
      else:
        cfg["hold_envelope"] = False
        bad.append(f"vtsc_hold_envelope {raw!r:.24} is not true/false -> OFF")
    if "vtsc_db_first" in tesla:
      # dbfirst2pnw kill switch, same contract as the other vtsc_* switches above: true / false (0 / 1); anything else is a `bad` key, the switch
      # goes OFF (today's flat notch) and the section is not honoured (mid-drive: last good config kept).
      raw = tesla["vtsc_db_first"]
      if isinstance(raw, bool):
        cfg["db_first"] = raw
      elif isinstance(raw, (int, float)) and raw in (0, 1):
        cfg["db_first"] = bool(raw)
      else:
        cfg["db_first"] = False
        bad.append(f"vtsc_db_first {raw!r:.24} is not true/false -> OFF")
    if "stop_go_handoff" in tesla:
      # stopgo2pnw kill switch, same contract as the vtsc_* switches above: true / false (0 / 1); anything else is a `bad` key, the switch goes
      # OFF (today's behaviour) and the section is not honoured (mid-drive: last good config kept).
      raw = tesla["stop_go_handoff"]
      if isinstance(raw, bool):
        cfg["stop_go"] = raw
      elif isinstance(raw, (int, float)) and raw in (0, 1):
        cfg["stop_go"] = bool(raw)
      else:
        cfg["stop_go"] = False
        bad.append(f"stop_go_handoff {raw!r:.24} is not true/false -> OFF")
    if clamped:
      cloudlog.warning(f"pnw_vehicle: {path}: tesla value(s) clamped into bounds: {', '.join(clamped)}")
    if bad:
      if "curve_brain" not in tesla:
        cfg["curve_brain"] = CURVE_BRAIN_CORRUPT      # a corrupt section with no explicit mode is not "no config"
      cloudlog.error(f"pnw_vehicle: {path}: tesla value(s) NOT honored: {'; '.join(bad)}")
      cfg["why"] = "INVALID curve.json tesla: " + "; ".join(bad)
    else:
      cfg["why"] = "curve.json"
  except Exception as e:
    cloudlog.error(f"pnw_vehicle: {path} tesla section unparsable ({type(e).__name__}) -- the curve brain uses its defaults")
    return {"curve_lat_a": TESLA_CURVE_LAT_A_DEFAULT, "curve_brain": CURVE_BRAIN_CORRUPT, "release_later": False,
            "notch_vego": False, "hold_envelope": False, "db_first": False, "stop_go": False,
            "why": f"default (curve.json tesla unparsable: {type(e).__name__})"}
  return cfg


def _curve_file_sig():
  """(mtime_ns, size) of curve.json, None when it does not exist, ("err", errno) when it cannot be stat'ed."""
  try:
    st = os.stat(CURVE_CONFIG_PATH)
    return (st.st_mtime_ns, st.st_size)
  except FileNotFoundError:
    return None
  except OSError as e:
    return ("err", e.errno)


def _tesla_cfg_is_honored(cfg: dict) -> bool:
  """True when the tesla section was read as written (or is the documented default), False for an unreadable file, a
  section that is not an object, or a key that was not honoured -- the cases a hot reload must not act on."""
  return not str(cfg["why"]).startswith(("INVALID", "default (curve.json"))


# fpsidebar2pnw: fingerprint -> short driver-facing display name (FINGERPRINT2XNOR.md /
# pending-work "Fingerprint sidebar"). This is the ONE place that maps a carFingerprint string to a
# friendly name — same capability-view discipline as the rest of this module: feature/UI code asks
# display_name(CP), it never string-compares carFingerprint itself. Used by the offroad sidebar to
# show e.g. "F-150 Lightning" for the last-known car instead of a bare "dashcam"-looking home screen
# when the shared device is parked (truck/car off). DISPLAY-ONLY: purely cosmetic, never read by any
# control code and never influences fingerprinting — `card` (selfdrive/car/card.py) is only_onroad
# and always re-fingerprints authoritatively the moment a car powers on.
#
# Deliberately module-level data + a plain function, NOT a PnwVehicle capability: PnwVehicle.__init__
# reads curve.json/rain.json off disk on every construction (real drive-control tunables), which the
# sidebar has no use for and shouldn't pay the I/O cost of on every offroad redraw.
#
# WIDTH BUDGET (code review 2026-07-18): the sidebar metric box is METRIC_WIDTH=240px, and
# _draw_metric centers the value text UNCLIPPED in ~218px usable (240 - 22px left inset) at
# FONT_SIZE 35 * FONT_SCALE (Inter SemiBold) — nothing scissors the text itself (only the colored
# left edge is scissored), so an over-length value crosses the metric border / can bleed past the
# sidebar edge. "F-150 Lightning" measured ~255-300px and overflowed; keep every value here short
# enough to clear ~218px (roughly <= 10-11 chars at this font) — "Lightning" / "Model S" both fit.
# Longest pre-existing sidebar value is "UPLOADING" (9 chars), a reasonable ceiling to match.
_DISPLAY_NAMES = {
  "FORD_F_150_LIGHTNING_MK1": "Lightning",
  "TESLA_MODEL_S_HW3": "Model S",          # the fleet's Raven (HW3) — see CLAUDE.md Cars & Devices
}


def display_name(CP) -> str | None:
  """Friendly driver-facing name for a fingerprinted car, for DISPLAY ONLY.

  Returns None (never a raw fingerprint string, and never "MOCK") whenever there's nothing safe to
  show: CP absent, MOCK/unrecognized brand, dashcamOnly, or a fingerprint not in the map above — the
  caller's job is to fall back to its own default text in every one of those cases. Defensive:
  accepts a capnp CarParams reader, the structs dataclass, or None; NEVER raises, so a future edit
  here can't crash-loop the UI (selfdrive/ui is restart_if_crash)."""
  try:
    if CP is None:
      return None
    fp = str(getattr(CP, 'carFingerprint', '') or '')
    brand = str(getattr(CP, 'brand', '') or '')
    if not fp or brand == 'mock' or bool(getattr(CP, 'dashcamOnly', False)):
      return None
    return _DISPLAY_NAMES.get(fp)
  except Exception:
    return None


class PnwVehicle:
  def __init__(self, CP, live_op_long=None):
    """live_op_long: UI contexts pass ui_state.has_longitudinal_control here — the PERSISTENT
    CarParams the UI reads keeps the PREVIOUS session's opLong until the next onroad fingerprint,
    so after toggling Alpha Long the capability view would lag a full drive behind (greyed the CES
    selector right after the driver turned alpha off, 2026-07-11). Controller contexts omit it
    (their CP is fresh at construction)."""
    fp = str(getattr(CP, 'carFingerprint', '') or '') if CP is not None else ''
    brand = str(getattr(CP, 'brand', '') or '') if CP is not None else ''

    # openpilot owns gas/brake (op-long / alpha-long active)
    if live_op_long is not None:
      self.op_long: bool = bool(live_op_long)
    else:
      self.op_long = bool(getattr(CP, 'openpilotLongitudinalControl', False)) if CP is not None else False

    # oplongfix2pnw (docs/pnw/op-long-features.md §6, correcting the oplongui2pnw bug): True when
    # this platform's openpilot longitudinal is UNCONDITIONAL / native -- independent of the
    # AlphaLongitudinalEnabled toggle. Today only the Tesla Raven: opendbc's tesla interface.py
    # ::_get_params_sx (the legacy/HW3 path) sets openpilotLongitudinalControl=True with NO
    # `if alpha_long:` gate around it -- the AP computer's driving role is fully replaced, so op-long
    # isn't a per-session opt-in there. Contrast the Ford Lightning, where alphaLongitudinalAvailable
    # IS the master A/B switch (opendbc/car/ford/interface.py: `alphaLongitudinalAvailable =
    # radarUnavailable`, and openpilotLongitudinalControl only becomes True once the driver opts in).
    #
    # CRITICAL: alphaLongitudinalAvailable is ALSO True for the Tesla -- opendbc's tesla
    # _get_params_sx sets `ret.alphaLongitudinalAvailable = True` right alongside the unconditional
    # openpilotLongitudinalControl=True (both unconditional, both True). So
    # "openpilotLongitudinalControl and not alphaLongitudinalAvailable" is NOT a valid native-op-long
    # test -- that expression is False for the Tesla, and was the shipped oplongui2pnw bug: the Tesla
    # fell through to the alpha branch, so its UI toggle / exp-button capability wrongly mirrored
    # AlphaLongitudinalEnabled instead of being unconditionally True. brand == "tesla" is the correct
    # capability-view test (matching the `nudgeless` line below), and callers (ui_state.py,
    # developer.py's compute_alpha_long_toggle_state) must check op_long_native BEFORE
    # alphaLongitudinalAvailable.
    self.op_long_native: bool = brand == "tesla"

    # stock-ACC set-speed steering via SET +/- button taps on the SCCM stream (ICBM executor lives
    # in the ford carcontroller; 0x083 is TX-allowlisted). Today: the 2025 F-150 Lightning.
    self.stock_acc_buttons: bool = fp == "FORD_F_150_LIGHTNING_MK1"

    # uimax2pnw (display only): the comma screen's MAX SPEED shows the driver's own latched ICBM ceiling while a
    # slowdown lowers the truck's stock set. Exactly the cars whose set speed ICBM steers by button taps.
    self.icbm_display_ceiling: bool = self.stock_acc_buttons

    # speedadjust-exec2pnw: the ONE generic capability that gates the shared stock-ACC button-tap
    # executor (opendbc/car/ford/icbm_pnw.py) — mirrors opendbc/car/pnw_vehicle.py's identically-
    # named field (kept in sync there; opendbc cannot import this openpilot-side module). True
    # whenever this car has stock-ACC buttons AND openpilot does NOT own longitudinal. Car-agnostic
    # by construction: ANY car-agnostic pnw brain that publishes a {target, ceiling, ts, dir?}
    # mem-param (today: ces_pnw.py's IcbmTarget curve brain, speedadjust_controller.py's
    # SpeedAdjustTarget police/limit brain) gets slowdowns for free on any car declaring this
    # capability — the executor arbitrates every live brain's command down to one target and has no
    # notion of "which feature". Neither brain checks carFingerprint/brand; this is the ONE place
    # that does.
    self.button_management: bool = self.stock_acc_buttons and not self.op_long

    # accdroplog2pnw (LOGGING ONLY, selfdrive/car/accdrop_pnw.py, run by card): the CAN messages whose
    # every <=4-bit status signal is recorded around each stock-ACC state change, as (parser bus, DBC
    # message name), plus continuous (bus, message, signal) trace values. Empty on every other car,
    # which makes card skip the logger entirely (the Tesla is untouched). The second group is NOT
    # registered by opendbc's Ford carstate today; it is listed on purpose so every record names it
    # under "notRegistered" instead of silently leaving it out, and so the logger picks it up with no
    # openpilot change once an opendbc pin bump registers it. Evidence for each:
    # drives/2026-09-12/central-oregon-weekend/DRIVE_REPORT.md (4 no-input Active->Standby drops).
    if self.stock_acc_buttons:
      self.acc_drop_status_msgs: tuple = (
        # registered by carstate today -> decoded every frame, recorded now
        ("pt", "EngBrakeData"),            # 0x165 PCM: CcStat/CcMde/CcOvrrdActv/AccEngStat/PrplTqMnSat/brake applied
        ("pt", "DesiredTorqBrk"),          # 0x213 ABS: CcDis_B_Cmd (cruise disable cmd), AccBrkDeny/Dis, TCS/ABS/ESC active
        ("pt", "BrakeSysFeatures"),        # 0x415 ABS: VehStab_D_Stat, speed quality
        ("pt", "Cluster_Info1_FD1"),       # 0x430 IPC: AccDeny_B_RqIpc, AccEnbl_B_RqDrv, ManRgen_D_Rq, slip-control mode
        ("pt", "EPAS_INFO"),               # 0x082 PSCM: EPAS_Failure, SteMdule_D_Stat
        ("pt", "Lane_Assist_Data3_FD1"),   # 0x3CC PSCM: LatCtlSte/Cpblty/Lim, LaHandsOff, LaActDeny, hands-on confidence
        ("pt", "Steering_Data_FD1"),       # 0x083 SCCM: every driver button bit, turn stalk
        ("cam", "ACCDATA"),                # 0x186 IPMA: AccCancl/AccDeny/CmbbDeny/AccResumEnbl (only without op-long)
        ("cam", "ACCDATA_2"),              # 0x187 IPMA: CMbB brake requests
        ("cam", "ACCDATA_3"),              # 0x18A IPMA: Tja_D_Stat, AccMsgTxt, AccWarn, radar blocked / misaligned
        ("cam", "IPMA_Data"),              # 0x3D8 IPMA: LaActvStats, LaDenyStats, LaHandsOff, camera status
        # NOT registered by carstate today -> reported as notRegistered until opendbc decodes them
        ("pt", "BrakeSysFeatures_2"),      # 0x416 ABS: BpedMove_D_Actl, Abs_B_Falt, TCMode, slip-control indicator
        ("pt", "BrakeSnData_5"),           # 0x076 ABS: StopLamp_B_RqBrk
        ("pt", "DesiredTorqBrk_2"),        # 0x214 ABS: RgenTqFalt_B_Actl (regen torque fault)
        ("pt", "VehicleOperatingModes"),   # 0x167 PCM: PwPckTq_D_Stat, ElPw_D_Stat
        ("pt", "HEV_Powertrain_Data2"),    # 0x25B PCM: PrplTqMnRgen_B_Actl
        ("pt", "TorqueDataEngFlags"),      # 0x200 PCM: PtDrvMde_D_Stat (drive mode)
        ("pt", "Powertrain_Data_4"),       # 0x424 PCM: SelDrvMdePt_D_Stat, BpedDrvMsgTxt_B_Dsply
        ("pt", "SelectDriveModeData"),     # 0x420 ABS: SelDrvMde_D_Stat
        ("pt", "Low_Voltage_Power_Data_FD1"),  # 0x43D PCM: 12 V source fault/disconnect (the rail reads ~11.5 V)
      )
      self.acc_drop_trace: tuple = (
        ("pt", "EngVehicleSpThrottle", "ApedPos_Pc_ActlArb"),   # accelerator %, registered
        ("cam", "ACCDATA", "AccPrpl_A_Rq"),                     # IPMA's ACC accel request, registered
        ("cam", "ACCDATA", "AccBrkTot_A_Rq"),                   # IPMA's ACC brake request, registered
      )
    else:
      self.acc_drop_status_msgs = ()
      self.acc_drop_trace = ()

    # pscmlimlog2pnw (LOGGING ONLY, selfdrive/car/pscmlim_pnw.py, run by card): where the steering rack reports its own
    # lateral limit, as (CAN parser bus, DBC message, limit signal, capability signal). The Lightning's PSCM sends
    # LimitClose/LimitReached in angle mode (drives/2026-09-12/central-oregon-weekend/PSCM_LIMITREACHED.md). The message
    # must already be registered by opendbc carstate; the logger never registers one. () on every other car, and card
    # then never builds the logger.
    self.pscm_limit_report: tuple = (("pt", "Lane_Assist_Data3_FD1", "LatCtlLim_D_Stat", "LatCtlCpblty_D_Stat")
                                     if fp == "FORD_F_150_LIGHTNING_MK1" else ())

    # CES runs in SHADOW (decisions/telemetry/overlay, planner never actuates) with ICBM as the
    # actuator — exactly when the car has ACC buttons to steer and openpilot does NOT own long.
    self.ces_shadow: bool = self.button_management

    # CES can act on this car at all (planner via op-long, or ICBM via buttons)
    self.ces_capable: bool = self.op_long or self.ces_shadow

    # speedadjust-exec2pnw: same capability under the feature's own name, for any call site that
    # wants to name the feature rather than the umbrella mechanism (e.g. future UI/telemetry).
    self.speedadjust_buttons: bool = self.button_management

    # mads2pnw: "lateral survives a brake press" (sunnypilot MADS's controls_allowed_lateral).
    # The Lightning runs STOCK ACC, so steering is the ONLY thing openpilot does for it and a brake
    # tap today takes away everything. Every other car: False.
    #
    # teslamads2pnw: the Raven now has it too. The earlier exclusion ("its EPS inhibits itself
    # (EAC_INHIBITED) on a brake press") is NOT supported by the rlogs: after 9/9 sampled brake
    # disengages eacStatus goes ACTIVE -> AVAILABLE with error 0 (the inhibit is the handsOnLevel-3
    # case), and brake is ~55% of Raven disengages (drives/2026-09-28/tesla-brake-census). What the
    # data does NOT show is the EPS accepting angle commands while DI_cruiseState is STANDBY -- never
    # observed on this car; the first on-car test must check it. mads_resume stays False below (the
    # Raven has no stock-ACC button path).
    self.mads_lateral: bool = fp in ("FORD_F_150_LIGHTNING_MK1", "TESLA_MODEL_S_HW3")

    # madsresume2pnw: openpilot may tap the stock ACC's RESUME button once, on the driver's behalf,
    # to give back the speed the driver had ALREADY SET, after a brake press left MADS steering
    # alone. Needs BOTH halves and neither implies the other: the MADS lateral authority (which is
    # what creates the state this feature completes) and the stock-ACC button path the tap rides on
    # (button_management -- the same 0x083 SCCM frame as the SET+/- taps; a car with op-long has no
    # stock ACC to resume). Mirrors opendbc/car/pnw_vehicle.py's identically-named field. The brain
    # additionally refuses to publish unless madsState.available is true at runtime, so a stock
    # (non-MADS) panda makes this inert even on a car that declares the capability.
    self.mads_resume: bool = self.mads_lateral and self.button_management

    # teslastalk2pnw: mirrors opendbc/car/pnw_vehicle.py. The Raven's stalk (0x45) is read into ButtonEvents:
    # FWD push -> mainCruise, RWD pull -> resumeCruise. selfdrived reads this to (a) let a stalk cancel end
    # steering-only even with a foot on the accelerator (the onoffgas2pnw gate protects a THUMB on the wheel
    # button, not a deliberate stalk push). eps_refusal_alert: card surfaces carstate's EPS-refusal verdict.
    self.stalk_cruise_buttons: bool = fp == "TESLA_MODEL_S_HW3"
    self.eps_refusal_alert: bool = fp == "TESLA_MODEL_S_HW3"

    # nudgeless (blinker-hold) lane change support — BSM-gated in DesireHelper
    self.nudgeless: bool = brand == "tesla" or fp == "FORD_F_150_LIGHTNING_MK1"

    # gpssel2pnw: the car broadcasts its own GPS fix on CAN (GWM APIMGPS 0x462/0x464, 1 Hz), which the
    # opendbc ford carstate decodes into the /dev/shm CarGps mem-param; mapd_configd may then select
    # it for LastGPSPosition. Verified only on our 2025 Lightning (2026-09-05 probe, 2026-09-12
    # weekend comparison: 1.6 m vs 3.0 m median cross-track). The Tesla has no such broadcast.
    # Not mirrored in opendbc/car/pnw_vehicle.py: nothing on the opendbc side consumes it (the
    # publisher already self-gates on the DBC carrying the messages).
    self.car_gps: bool = fp == "FORD_F_150_LIGHTNING_MK1"

    # fordtsr2pnw: the car's own camera reports the posted speed limit, decoded by opendbc into
    # carState.cruiseState.speedLimitSign(/Status) (Ford CAN FD 0x3CD Traffic_RecognitnData). Measured only on our
    # 2025 Lightning (drives/2026-09-24/ford-tsr-measure: a limit 99.5 % of driving time, 94 % agreement with mapd).
    # speedadjust weighs it against the map limit only on a car declaring this; everywhere else the carState fields
    # read `unavailable` and are never looked at (the Tesla is untouched). Not mirrored in opendbc/car/pnw_vehicle.py:
    # the decode self-gates on CAN FD + the DBC carrying the message.
    self.camera_speed_limit: bool = fp == "FORD_F_150_LIGHTNING_MK1"

    # teslayaw2pnw: carState.yawRate is a real CAN yaw-rate sensor on this car (positive = left). Every Ford
    # (opendbc ford/carstate.py, Yaw_Data_FD1 -- unconditional for the brand) and the Raven HW3 (tesla/carstate.py,
    # BrakeMessage 0x20a, pnw-opendbc teslayaw2pnw). On any other car the carstate never assigns it, so it reads the
    # capnp default 0.0 -- a confident "driving straight". Telemetry derived from it (controlsd kActl/kErr/achLat/
    # peakAchLat, ces_pnw's CAN half of kPeak) must publish None there instead. Telemetry only: nothing that
    # actuates reads this.
    self.yaw_rate_source: bool = brand == "ford" or fp == "TESLA_MODEL_S_HW3"

    # gearparkcan2pnw: this car's carstate reports gearShifter == park ONLY from a gear frame that actually
    # arrived -- a never-received gear message decodes `unknown` -- and gear_source_bus names the CANParser
    # (card's CI.can_parsers key) that carries it. That lets selfdrive/car/gear_park.py confirm Park while
    # another bus is asleep (the Lightning charging with its camera bus quiet). Verified through the pinned
    # opendbc parser (selfdrive/car/tests/test_gear_park.py): the Lightning since pnw-opendbc gearunknown2pnw
    # (PowertrainData_10 on "pt"); the Raven HW3 by its DBC (DI_torque2.DI_gear 0 = DI_GEAR_INVALID, on
    # "chassis"). Every other car: False -- 74 platforms in that opendbc decode park from a silent bus.
    self.gear_source_bus: str = {"FORD_F_150_LIGHTNING_MK1": "pt", "TESLA_MODEL_S_HW3": "chassis"}.get(fp, "")
    self.gear_unknown_until_seen: bool = self.gear_source_bus != ""

    # coopsteer-shadow2pnw / coopsteer2pnw: Penduras "cooperative steering" sub-threshold torque nudge
    # (selfdrive/controls/lib/coopsteer_pnw.py). controlsd always computes and logs it (cp*) and ADDS it
    # to the actuator unless the DisableCoopSteer opt-out toggle is on. Gated to the exact car the
    # sign convention was road-confirmed on (09-07 and 09-26 Raven rlogs, >= 99 %); extending to
    # another Tesla class needs its own confirmation. LOAD-BEARING, not tidiness: the Ford also runs LatControlAngle, so without this
    # gate the shadow would run there too and pollute the Lightning's cp* telemetry (actuation-inert
    # either way -- the Ford carcontroller consumes actuators.curvature, never steeringAngleDeg).
    self.coop_steer: bool = fp == "TESLA_MODEL_S_HW3"

    # toggles2pnw: does the capability view know what car this is? False when nothing was ever fingerprinted (CP None), the
    # fingerprint is empty, or it is the MOCK brand. The settings UI uses it to leave EVERY car-gated toggle enabled in
    # that case (the owner must be able to set troubleshooting toggles while parked at home with no last-known car).
    self.car_known: bool = bool(fp) and brand != 'mock'

    # toggles2pnw: the Lightning is the car whose non-driving CAN writes (Pro Power re-arm) the "Disable Ford Convenience Features"
    # toggle gates. Mirrors opendbc/car/pnw_vehicle.py's pro_power_onboard; used by the settings UI to grey the toggle elsewhere.
    self.ford_convenience: bool = fp == "FORD_F_150_LIGHTNING_MK1"

    # toggles2pnw (DISPLAY ONLY): the Lightning has an EverDrive charger box whose status the UI can show. Nothing actuates on it.
    self.everdrive: bool = fp == "FORD_F_150_LIGHTNING_MK1"

    # curveslow-lightning: the Lightning's EPS is physically weaker than the Tesla's and washes out of
    # curves the Tesla holds, so it must enter curves SLOWER. This is a steering-authority FACT, not a
    # config choice, so it applies in BOTH op-long (VTSC) and stock-ACC (ICBM) — both produce a curve
    # target SPEED that curve_speed_penalty_ms() lowers. Tesla (and every other car) -> False -> 0.0.
    self.lightning_curve_slow: bool = fp == "FORD_F_150_LIGHTNING_MK1"
    # standstillsoft2pnw: the Lightning's EV torque + pure-integrator longitudinal makes a launch out of
    # a stop (following a lead that pulled away at a red) a hard LURCH; gentle the launch accel. The
    # Tesla launches smoothly and is NOT affected (capability, not a fingerprint check in feature code).
    self.gentle_launch: bool = fp == "FORD_F_150_LIGHTNING_MK1"
    # tightfollow2pnw (driver req 2026-07-16): DISABLED 2026-07-16 after on-road measurement — a flat
    # 1.0s T_FOLLOW made things measurably WORSE, not better: avg gapS actually rose to 1.92s (looser
    # than the pre-fix 1.84s) with the gap hunting 1.20-3.80s, and aEgo stdev rose 0.221->0.347 (audibly
    # rougher ride). Root cause (Fable design review): the Lightning has no radar (radarUnavailable=True
    # in ford/interface.py), so radard.py's vision-only lead path is completely unfiltered (no KF1D,
    # unlike the radar Track path) -- tightening the target just asked the MPC to chase raw per-frame
    # model noise harder. Fix needs to filter the vision lead in radard.py FIRST (a general fix, not
    # Lightning-specific), then reintroduce tightening as a smaller (~1.15s), lead-stability-gated,
    # slewed target -- not a flat constant. See PENDING-WORK.md. Left as an inert capability (False) so
    # the plumbing (aggressive_t_follow / t_follow_override) stays in place for that follow-up.
    # tightfollow2pnw v2 (2026-08-31): RE-ENABLED for the Lightning now that the v1 root cause is
    # actually fixed upstream of here -- radarless2pnw (a46d8749b8) added VisionLeadFilter, so the
    # vision-only lead's vLead/aLeadK are Kalman-filtered instead of raw per-frame model output.
    # v2 differs from the reverted v1 in three ways, all required: a shallower target (1.15 vs 1.0),
    # a lead-stability gate, and a slew. Kept a Lightning capability rather than keying on
    # CP.radarUnavailable: this is the DRIVER's preference for this truck, not a physics property,
    # and radarUnavailable would silently opt in any future radarless car nobody asked for.
    self.tight_aggressive_follow: bool = fp == "FORD_F_150_LIGHTNING_MK1"
    # slew/stability state for aggressive_t_follow (reset whenever the override is not applicable)
    self._tight_stable_s: float = 0.0
    self._tight_prev_drel: float | None = None
    self._tight_t_follow: float | None = None
    # read the tunable ramp ONCE at construction (defensive; defaults when absent = the intended ramp)
    self._curve_cfg = _load_curve_config()

    # curvebrain2pnw 1/8: VTSC on this car may consume the shared curve brain (design s5; nothing does yet). The Raven
    # HW3 only: its lateral target was measured on this car (design s5.4), and curve.json's "tesla" section is read
    # ONLY here, so on every other car -- the Lightning above all -- it is never even opened.
    self.curve_brain_vtsc: bool = fp == "TESLA_MODEL_S_HW3"
    # ovrcar2pnw: the platform key this car's per-curve overrides are filed under (curve_brain.Overrides), or None = this car
    # never reads /data/pnw/curve_overrides.json (owner 2026-09-29: only the Tesla and the Lightning use it).
    self.curve_override_platform: str | None = fp if fp in CURVE_OVERRIDE_PLATFORMS else None
    self._tesla_cfg_sig = _curve_file_sig() if self.curve_brain_vtsc else None   # curvebrain2b2pnw: hot-reload state
    self._tesla_curve_cfg = _load_tesla_curve_config() if self.curve_brain_vtsc else None
    self._tesla_cfg_good = bool(self.curve_brain_vtsc and _tesla_cfg_is_honored(self._tesla_curve_cfg))
    if self.curve_brain_vtsc:
      _tesla_steer_lat_ceiling()          # curvebrain2b2pnw: pay the one-time opendbc import here, not in a control loop
    self._tesla_cfg_poll = time.monotonic()

    # rain2pnw: wet-weather curve margin — applies to BOTH cars, SAME reduction (not the Lightning-only
    # curve penalty). Magnitudes from the device-local tunable (defaults 3/5 mph, read once here); the
    # live tier is pushed in by the controllers each ~1 Hz via set_rain_tier so a mid-drive change (it
    # starts raining) takes effect with no restart.
    self._rain_cfg = _load_rain_config()
    self._rain_tier = 0

  def set_rain_tier(self, tier) -> None:
    """rain2pnw: set the live wet-weather tier (0=None, 1=Light, 2=Heavy). Controllers push the
    RainMode param here each ~1 Hz. Defensive: accepts int / str / bytes (Params.get yields bytes);
    anything that isn't 1 or 2 -> 0 (off)."""
    try:
      if isinstance(tier, bytes):
        tier = tier.decode()
      t = int(tier)
    except (TypeError, ValueError):
      t = 0
    self._rain_tier = t if t in (1, 2) else 0

  def rain_penalty_ms(self) -> float:
    """rain2pnw: extra m/s to SUBTRACT from a curve target speed for the current rain tier — the SAME
    reduction on every car (each car's dry curve tuning is separate and untouched; this is a driver-
    opted additive margin only). 0.0 when the tier is None. Reduce-only, bounded by the config."""
    if self._rain_tier == 1:
      return self._rain_cfg["light_mph"] * _MPH_TO_MS
    if self._rain_tier == 2:
      return self._rain_cfg["heavy_mph"] * _MPH_TO_MS
    return 0.0

  def curve_speed_penalty_ms(self, v_target_ms: float, pitch_rad=None, is_left: bool = False) -> float:
    """Extra m/s to SUBTRACT from a curve target speed so the Lightning enters curves slower than the
    Tesla. Shape (driver-calibrated on I-90, 2026-07-11 evening — three field iterations):
    a HUMP, not a ramp. The EPS deficit bites hardest in the MID-speed "tight" highway curves
    (binding targets ~45-62 mph — the washout zone); slow corners need almost nothing (ample
    steering authority when slow), and LONG FAST sweepers (high binding targets = gentle curvature)
    can carry speed again ("towards the end of the drive it was too slow, needs to accelerate
    more" — driver, on the monotonic ramp). Piecewise-linear over the target speed in mph:
      <= low_v (30)            -> penalty_min (1.0)
      peak_lo..peak_hi (45..62)-> penalty_max (5.0)   (the approved tight-curve cut)
      >= taper_v (75)          -> penalty_taper (1.5) (fast gentle sweepers keep their speed)
      linear between the knots.

    descentcurve2pnw multipliers on top of the hump (both optional args -> existing callers are
    byte-identical):
      pitch_rad: road pitch (rad, carControl.orientationNED[1]; < 0 = downhill). On a DESCENT the
        penalty scales UP: pen *= 1 + descent_gain * min(|pitch|, descent_pitch_cap) — gravity eats
        the regen decel budget, so the truck must enter the curve slower (2026-07-11 washouts at
        18:12 / 19:17 were "accelerating downhill into the curve"). None / NaN / uphill -> no-op.
      is_left: LEFT curve -> pen *= left_factor. Neutral (1.0) by default since 2026-09-24 -- see
        the left_factor default comment for why the 1.15 was removed.
    The TOTAL is clamped to penalty_cap_mph (<= 15 mph) after all multipliers.
    Returns 0.0 for any non-Lightning (Tesla path untouched). Pure Python (no numpy); never
    negative (a bad config can't invert this into a speed-up)."""
    if not self.lightning_curve_slow:
      return 0.0
    cfg = self._curve_cfg
    v_mph = float(v_target_ms) / _MPH_TO_MS
    # knots, sanitized: enforce ordering so a bad config degrades to a flat safe shape, never crashes
    x0 = cfg["low_v_mph"]
    x1 = max(cfg["peak_lo_v_mph"], x0 + 1.0)
    x2 = max(cfg["peak_hi_v_mph"], x1)
    x3 = max(cfg["taper_v_mph"], x2 + 1.0)
    y0, y1, y3 = cfg["penalty_min_mph"], cfg["penalty_max_mph"], cfg["penalty_taper_mph"]
    if v_mph <= x0:
      pen_mph = y0
    elif v_mph < x1:
      pen_mph = y0 + (y1 - y0) * (v_mph - x0) / (x1 - x0)
    elif v_mph <= x2:
      pen_mph = y1                                   # the peak plateau: the washout zone
    elif v_mph < x3:
      pen_mph = y1 + (y3 - y1) * (v_mph - x2) / (x3 - x2)
    else:
      pen_mph = y3                                   # fast gentle sweepers: nearly free again
    # descentcurve2pnw: descent guard — scale the penalty UP with downhill grade. Defensive on the
    # pitch input (None / non-numeric / NaN -> skip); gain >= 0 and |pitch| clamped >= 0, so the
    # multiplier is always >= 1 (monotonic in |pitch| up to the cap, never a reduction).
    if pitch_rad is not None:
      try:
        p = float(pitch_rad)
      except (TypeError, ValueError):
        p = 0.0
      if p == p and p < 0.0:                         # finite (NaN != NaN) AND downhill
        pen_mph *= 1.0 + cfg["descent_gain"] * min(-p, cfg["descent_pitch_cap"])
    # descentcurve2pnw: left-curve factor — clamped >= 1.0; 1.0 (no-op) by default since 2026-09-24
    if is_left:
      pen_mph *= cfg["left_factor"]
    pen_mph = min(pen_mph, cfg["penalty_cap_mph"])   # hard total cap after all multipliers
    return max(0.0, pen_mph * _MPH_TO_MS)

  # ---- descentcurve2pnw accessors (all neutral on non-Lightning: 0.0 / 1.0 / 0.0) ----------------
  @property
  def overspeed_margin_ms(self) -> float:
    """VTSC overspeed-into-curve escalation margin (m/s): v_ego above the applied cap by more than
    this (with a binding curve regen can't make) unlocks the existing SHARP_A_DECEL_MAX friction
    ceiling. 0.0 on non-Lightning (callers also gate on lightning_curve_slow)."""
    return self._curve_cfg["overspeed_margin_mph"] * _MPH_TO_MS if self.lightning_curve_slow else 0.0

  @property
  def icbm_map_scale(self) -> float:
    """Lightning discount on mapd's suggested curve speeds BEFORE the ICBM binding test (<= 1.0 by
    the config bounds — can never inflate a map speed). 1.0 (identity) on non-Lightning."""
    return self._curve_cfg["map_scale"] if self.lightning_curve_slow else 1.0

  @property
  def icbm_firm_decel(self) -> float:
    """Assumed approach decel (m/s^2) ICBM may plan with for LARGE speed drops (stock ACC does the
    actual braking). 0.0 on non-Lightning -> callers fall back to the base comfort decel."""
    return self._curve_cfg["icbm_firm_decel"] if self.lightning_curve_slow else 0.0

  def icbm_map_floor_ms(self, raw_map_v) -> float:
    """icbmslow2pnw: the speed (m/s) the Lightning curve penalty may not push an ICBM MAP/FAR curve
    target below -- `icbm_map_floor_frac` times that candidate's OWN raw mapd rating.

    Returns 0.0 (no floor) for every non-Lightning car, for a missing/NaN/non-positive rating, and
    when the fraction is configured to 0. 0.0 is safe by construction: the caller applies it with
    max(), so "no floor" is exactly the pre-icbmslow2pnw behaviour.

    Provably inert where the penalty is 0 anyway: a non-Lightning car has
    curve_speed_penalty_ms() == 0.0 AND icbm_map_scale == 1.0, so its target is
    icbm_map_eff_scale(raw) * raw >= 1.10 * raw, already above any floor this could return. The
    lightning_curve_slow gate is kept regardless so the Tesla path stays byte-identical."""
    if not self.lightning_curve_slow:
      return 0.0
    try:
      v = float(raw_map_v)
    except (TypeError, ValueError):
      return 0.0
    if not math.isfinite(v) or v <= 0.0:
      return 0.0
    return v * self._curve_cfg["icbm_map_floor_frac"]

  def icbm_vis_floor_ms(self, vis_apex) -> float:
    """curvefix2pnw Part B: the speed (m/s) the Lightning curve penalty may not push an ICBM VISION curve target
    below -- `icbm_vis_floor_frac` times the vision candidate's OWN pre-penalty speed (icbm_vision_apex, at
    A_LAT_TARGET 2.5 m/s^2). Same contract as icbm_map_floor_ms: 0.0 (no floor) on every non-Lightning car, for a
    missing / NaN / non-positive speed, and when the fraction is 0; the caller applies it with min(floor, target)
    and max(), so it can only give penalty back, never raise a target above the candidate."""
    if not self.lightning_curve_slow:
      return 0.0
    try:
      v = float(vis_apex)
    except (TypeError, ValueError):
      return 0.0
    if not math.isfinite(v) or v <= 0.0:
      return 0.0
    return v * self._curve_cfg["icbm_vis_floor_frac"]

  @property
  def curvedb_v2_live(self) -> bool:
    """curvedblive2pnw: the learned curve database may set ICBM's curve target. Lightning only (ICBM is its
    stock-ACC path), and only while curve.json's `curvedb_v2_live` kill switch is on (default on)."""
    return self.lightning_curve_slow and self._curve_cfg["curvedb_v2_live"] >= 0.5

  @property
  def curvedb_v2_lat_a(self) -> float | None:
    """curvedblive2pnw: curve.json's own A for the curve DB, or None = use mapd's A (the default, and every
    non-Lightning car). A value below 1.0 other than 0 is raised to 1.0 (the DB's plausibility floor), never read
    as "mapd's"."""
    v = self._curve_cfg["curvedb_v2_lat_a"]
    if not self.lightning_curve_slow or v <= 0.0:
      return None
    return max(v, 1.0)

  @property
  def icbm_shape(self) -> str:
    """curveshape2pnw: the measured-shape stage's mode, "off" / "shadow" / "live". Lightning only (ICBM is its
    stock-ACC path); every other car reads "off"."""
    return self._curve_cfg.get("icbm_shape", SHAPE_MODE_DEFAULT) if self.lightning_curve_slow else "off"

  @property
  def icbm_shape_why(self) -> str:
    """curveshape2pnw: where icbm_shape came from ("default", "curve.json", "noCapability", or why a value was
    refused)."""
    return self._curve_cfg.get("icbm_shape_why", "default") if self.lightning_curve_slow else "noCapability"

  @property
  def icbm_shape_lat_a(self) -> float:
    return self._curve_cfg["icbm_shape_lat_a"]

  @property
  def icbm_shape_lat_a_70(self) -> float:
    return self._curve_cfg["icbm_shape_lat_a_70"]

  @property
  def icbm_lead_lat_accel(self) -> float:
    """curvelead2pnw: the lateral accel (m/s^2) ICBM may let a tracked lead pace this truck to through a
    curve. It is the truck's steering capability, not the lead's: a car corners harder than a 7,000 lb
    pickup with a weaker EPS. 0.0 = lead pacing OFF -- every non-Lightning car, and the Lightning when
    curve.json sets it to 0."""
    return self._curve_cfg["icbm_lead_lat_accel"] if self.lightning_curve_slow else 0.0

  @property
  def icbm_restore_hold_lat_accel(self) -> float:
    """restorehold2pnw: the curve target (m/s^2) a held ICBM restore may rise to (v_safe = sqrt(this / k)), and
    vision's trigger bar. 0.0 = hold OFF -- every non-Lightning car, and the Lightning when curve.json sets it to 0."""
    return self._curve_cfg["icbm_restore_hold_lat_accel"] if self.lightning_curve_slow else 0.0

  @property
  def icbm_restore_hold_poly_lat_accel(self) -> float:
    """restorehold2pnw: the map polyline's own (higher) trigger bar for the restore hold (m/s^2). 0.0 = polyline off
    -- every non-Lightning car, and the Lightning when curve.json sets it to 0."""
    return self._curve_cfg["icbm_restore_hold_poly_lat_accel"] if self.lightning_curve_slow else 0.0

  # ---- curvebrain2pnw 1/8: the shared curve brain's per-car config. NOTHING CONSUMES THESE YET. ----------------
  def curve_lat_a(self, v_ego) -> float:
    """The lateral accel (m/s^2) the shared curve brain prices a curve at on this car (design s3.4).

    Tesla (curve_brain_vtsc): curve.json tesla.curve_lat_a (default 4.0, bounds [2.0, 4.5]), capped at openpilot's own
    lateral clip at this speed -- the UNSLEWED lataccel2pnw schedule -- and at the steering ceiling. Two cases for the clip:
    the Tesla's own valid "cars" entry in lataccel_limits.json (latcar2pnw; owner 2026-09-30: [[50,5],[60,5],[70,4],[80,3.6]],
    3.6 = the vehicle-model clamp, so controlsd's clip_curvature -- and the 'Turn Exceeds Steering Limit' alert -- act at the
    car's real limit) is used AS IS, no margin (latmargin2pnw: the value is chosen at the clamp; the ceiling below still
    bounds it); the shared schedule (no/invalid per-car entry, the Lightning, other cars) keeps the margin exactly as before:
    lat_accel_target(v) - CURVE_LAT_CLIP_MARGIN (shared 4.0 at 70 mph, 3.0 at >= 80 mph; flat 3.0 without a valid file).
    ALSO capped at the steering ceiling (_tesla_steer_lat_ceiling: the vehicle-model angle clamp's lateral acceleration
    min(3.6 owner, MAX_LATERAL_ACCEL 3.5886), zero margin -- see the comment above it), so the owner's requested 4.0 is
    clipped to 3.59 while the schedule allows it: with the Tesla's per-car entry that is at EVERY speed (A = 3.5886).
    Without that entry (shared device schedule [[50,5],[60,5],[70,4],[80,3]]) up to ~71 mph, then
    3.5 at 72 mph ... 2.7 from 80 mph; without a schedule file the clip is a flat 2.7.
    Every other car: CURVE_LAT_A_DEFAULT (2.5). On the
    Lightning it is for the brain's future need layer only: ICBM keeps its own knobs (VTSC_A_LAT, curvedb_v2_lat_a,
    icbm_shape_lat_a(_70), the restore-hold bars)."""
    if not self.curve_brain_vtsc:
      return CURVE_LAT_A_DEFAULT
    clip, per_car = lat_accel_target_source(v_ego, self.curve_override_platform)
    return min(self._tesla_curve_cfg["curve_lat_a"], clip if per_car else clip - CURVE_LAT_CLIP_MARGIN,
               _tesla_steer_lat_ceiling())

  def refresh_curve_brain_cfg(self, now: float | None = None) -> bool:
    """curvebrain2b2pnw: hot-reload curve.json's "tesla" section (the owner's kill switch: {"tesla": {"curve_brain":
    "shadow"}} or "off" takes effect within CURVE_CFG_POLL_S plus the caller's own cadence, no restart). One os.stat
    per poll; the file is re-parsed only when its (mtime, size) changed. Returns True when the live config changed.
    Never raises. A no-op on a car without the capability.

    A file that is now unreadable, malformed, not an object, or holds a key that cannot be honoured is NOT applied:
    the last config is kept and it is a cloudlog.error -- a typo typed mid-drive must not silently swap "shadow" for
    the acting default. A file that is simply GONE reverts to the documented default (acting), and says so."""
    if not self.curve_brain_vtsc:
      return False
    now = time.monotonic() if now is None else now
    if now - self._tesla_cfg_poll < CURVE_CFG_POLL_S:
      return False
    self._tesla_cfg_poll = now
    try:
      sig = _curve_file_sig()
      if sig == self._tesla_cfg_sig:
        return False
      self._tesla_cfg_sig = sig
      old = self._tesla_curve_cfg
      new = _load_tesla_curve_config()
      if not _tesla_cfg_is_honored(new) and self._tesla_cfg_good:
        cloudlog.error(f"pnw_vehicle: curve.json changed but its tesla section was NOT applied ({new['why']}) -- " +
                       f"keeping mode={old['curve_brain']} lat_a={old['curve_lat_a']} " +
                       f"release_later={old['release_later']} notch_vego={old['notch_vego']} hold_envelope={old['hold_envelope']} " +
                       f"db_first={old['db_first']} stop_go={old['stop_go']}")
        self._tesla_curve_cfg = dict(old, why=f"{old['why']} | reload rejected: {new['why']}")
        return False
      self._tesla_curve_cfg = new
      self._tesla_cfg_good = _tesla_cfg_is_honored(new)
      changed = ((new["curve_brain"], new["curve_lat_a"], new["release_later"], new["notch_vego"], new["hold_envelope"],
                  new["db_first"], new["stop_go"])
                 != (old["curve_brain"], old["curve_lat_a"], old["release_later"], old["notch_vego"], old["hold_envelope"], old["db_first"],
                     old["stop_go"]))
      cloudlog.event("curve_brain_cfg_reload", mode=new["curve_brain"], lat_a=new["curve_lat_a"], why=new["why"],
                     prev_mode=old["curve_brain"], prev_lat_a=old["curve_lat_a"], changed=changed,
                     release_later=new["release_later"], prev_release_later=old["release_later"],
                     notch_vego=new["notch_vego"], prev_notch_vego=old["notch_vego"],
                     hold_envelope=new["hold_envelope"], prev_hold_envelope=old["hold_envelope"],
                     db_first=new["db_first"], prev_db_first=old["db_first"],
                     stop_go=new["stop_go"], prev_stop_go=old["stop_go"],
                     file="absent" if sig is None else "present")
      return changed
    except Exception as e:
      cloudlog.error(f"pnw_vehicle: curve.json hot-reload FAILED ({type(e).__name__}: {e}) -- keeping the last config")
      return False

  @property
  def curve_lat_a_cfg(self) -> float:
    """curve_lat_a before the lateral-clip cap: the configured value (Tesla) or CURVE_LAT_A_DEFAULT."""
    return self._tesla_curve_cfg["curve_lat_a"] if self.curve_brain_vtsc else CURVE_LAT_A_DEFAULT

  @property
  def curve_brain(self) -> str:
    """What VTSC may do with the shared curve brain: "off" / "shadow" / "lower" / "raise" (design s5.2). The Tesla
    defaults to "shadow"; every other car reads "off" (the Lightning's VTSC never consumes it)."""
    return self._tesla_curve_cfg["curve_brain"] if self.curve_brain_vtsc else "off"

  @property
  def vtsc_release_later(self) -> bool:
    """vtscfloor2pnw: may VTSC freeze its cap instead of climbing while the curve target is still tightening (release-later,
    vtscRelDefer)? The Raven (curve_brain_vtsc) only, and only while curve.json's tesla.vtsc_release_later is not switched off; False
    on every other car, so the Lightning's VTSC is byte-unchanged."""
    return bool(self.curve_brain_vtsc and self._tesla_curve_cfg["release_later"])

  @property
  def vtsc_notch_from_vego(self) -> bool:
    """vtscnotch2pnw: may VTSC measure the map-curve minimum-slowdown notch from min(set, vEgo) rather than the set speed? The Raven
    (curve_brain_vtsc) only, and only while curve.json's tesla.vtsc_notch_vego is not switched off; False on every other car, so the
    Lightning's VTSC (inert anyway: no op-long) is byte-unchanged."""
    return bool(self.curve_brain_vtsc and self._tesla_curve_cfg["notch_vego"])

  @property
  def vtsc_hold_envelope(self) -> bool:
    """vtschold2pnw: may VTSC's apex hold lower its frozen cap to the envelope of a binding curve that needs less (and to the car's own speed
    when the car is at safe speed)? The Raven (curve_brain_vtsc) only, and only while curve.json's tesla.vtsc_hold_envelope is not switched
    off; False on every other car, so the Lightning's VTSC (inert anyway: no op-long) is byte-unchanged."""
    return bool(self.curve_brain_vtsc and self._tesla_curve_cfg["hold_envelope"])

  @property
  def vtsc_db_first(self) -> bool:
    """dbfirst2pnw: is the curve DB first for VTSC's map curve notch (DB-covered points skip the OSM fold, uncovered ones get the posted-limit
    cap instead of the flat notch)? The Raven (curve_brain_vtsc) only, and only while curve.json's tesla.vtsc_db_first is not switched off;
    False on every other car, so the Lightning's VTSC (inert anyway: no op-long) is byte-unchanged."""
    return bool(self.curve_brain_vtsc and self._tesla_curve_cfg["db_first"])

  @property
  def stop_go_handoff(self) -> bool:
    """stopgo2pnw: may the planner hand the stop decision to the lead-aware MPC while a lead pulls away from a standstill (stopgo_pnw)? The
    Raven (curve_brain_vtsc) only -- the Lightning has no op-long path through this and is never affected -- and only while curve.json's
    tesla.stop_go_handoff is not switched off. False on every other car."""
    return bool(self.curve_brain_vtsc and self._tesla_curve_cfg["stop_go"])

  @property
  def curve_brain_why(self) -> str:
    """Where curve_brain / curve_lat_a came from: "default", "curve.json", "INVALID ...", "default (curve.json ...)",
    or "noCapability"."""
    return self._tesla_curve_cfg["why"] if self.curve_brain_vtsc else "noCapability"

  def gentle_launch_accel(self, v_ego: float) -> float:
    """standstillsoft2pnw: a soft accel CEILING (m/s^2) out of a standstill so a follow-launch behind a
    departing lead is a smooth ramp, not a ~2.0 m/s^2 lurch. Ramps linearly from launch_accel at v=0 up
    to _LAUNCH_TOP (just above the stock max, so it stops binding) by launch_v; +inf above launch_v and
    on every non-Lightning car -> NO cap there. The caller applies it REDUCE-ONLY to the accel ceiling
    (min), so this can never RAISE the accel limit / speed the car up — it can only soften the launch.
    Only the positive-accel ceiling is touched; braking authority (accel_clip[0]) is untouched."""
    if not self.gentle_launch:
      return float('inf')
    v_lift = self._curve_cfg["launch_v_mph"] * _MPH_TO_MS
    if v_lift <= 0.0 or v_ego >= v_lift:
      return float('inf')
    a0 = self._curve_cfg["launch_accel"]
    frac = _clamp(v_ego / v_lift, 0.0, 1.0)
    return a0 + frac * (_LAUNCH_TOP - a0)

  def _tight_reset(self) -> None:
    """Drop all tight-follow state. Called whenever the override cannot apply, so a later re-entry
    starts from the baseline with a fresh stability clock instead of resuming mid-slew."""
    self._tight_stable_s = 0.0
    self._tight_prev_drel = None
    self._tight_t_follow = None

  def _tight_lead_is_calm(self, lead, v_ego: float, dt: float) -> bool:
    """Advance the stability clock; True once this lead has been calm for _TIGHT_STABLE_MIN_S.

    Fail-closed by construction: a missing lead, a malformed field, a lead SWAP (dRel step) or any
    excursion past the vRel/aLead thresholds zeroes the clock, so the answer on anything unexpected
    is False -> slew back toward the looser baseline."""
    status = False
    try:
      status = bool(lead.status)
    except AttributeError:
      status = False
    if lead is None or not status:
      self._tight_stable_s = 0.0
      self._tight_prev_drel = None
      return False
    try:
      d_rel = float(lead.dRel)
      v_lead_k = float(lead.vLeadK)
      a_lead_k = float(lead.aLeadK)
    except (AttributeError, TypeError, ValueError):
      self._tight_stable_s = 0.0
      self._tight_prev_drel = None
      return False
    if not (math.isfinite(d_rel) and math.isfinite(v_lead_k) and math.isfinite(a_lead_k)):
      self._tight_stable_s = 0.0
      self._tight_prev_drel = None
      return False

    prev_drel = self._tight_prev_drel
    self._tight_prev_drel = d_rel
    if prev_drel is not None and abs(d_rel - prev_drel) > _TIGHT_LEAD_JUMP_M:
      self._tight_stable_s = 0.0            # different car -> no credit carried over
      return False
    # DERIVED from the filtered lead speed, never lead.vRel (which is raw) -- see the constants above.
    if abs(v_lead_k - v_ego) > _TIGHT_MAX_ABS_VREL or abs(a_lead_k) > _TIGHT_MAX_ABS_ALEAD:
      self._tight_stable_s = 0.0
      return False
    self._tight_stable_s += dt
    return self._tight_stable_s >= _TIGHT_STABLE_MIN_S

  def aggressive_t_follow(self, personality, lead=None, v_ego: float = 0.0,
                          baseline: float | None = None, dt: float | None = None) -> float | None:
    """tightfollow2pnw v2: a tighter T_FOLLOW (s) for the Lightning's Aggressive personality only —
    None (unchanged upstream behavior) for every other car and every other personality, including
    the Tesla's own Aggressive. The caller passes this straight through as long_mpc's optional
    t_follow_override; None there means "use the shared get_T_FOLLOW(personality) as before".

    v2 is gated and slewed rather than flat (v1's flat 1.0 was reverted — it asked the MPC to chase
    unfiltered vision-lead noise). Tightening applies only while the lead has been CALM for
    _TIGHT_STABLE_MIN_S, and the target is slewed at _TIGHT_SLEW_S_PER_S so neither entering nor
    leaving the gate can step the MPC's desired following distance.

    Directionality is deliberate and asymmetric in the safe sense: `target` is clamped into
    [tight, baseline], so this can only ever ask for a gap between the two — it can never tighten
    past _TIGHT_AGGRESSIVE_T_FOLLOW nor loosen beyond the personality's own baseline."""
    if not self.tight_aggressive_follow or personality != log.LongitudinalPersonality.aggressive:
      self._tight_reset()
      return None

    step_dt = _TIGHT_DT_DEFAULT if dt is None else float(dt)
    if not math.isfinite(step_dt) or step_dt <= 0.0:
      step_dt = _TIGHT_DT_DEFAULT
    base = _TIGHT_AGGRESSIVE_BASELINE if baseline is None else float(baseline)
    if not math.isfinite(base):
      base = _TIGHT_AGGRESSIVE_BASELINE
    v_ego_f = float(v_ego) if math.isfinite(float(v_ego)) else 0.0

    lo, hi = min(_TIGHT_AGGRESSIVE_T_FOLLOW, base), max(_TIGHT_AGGRESSIVE_T_FOLLOW, base)
    target = _TIGHT_AGGRESSIVE_T_FOLLOW if self._tight_lead_is_calm(lead, v_ego_f, step_dt) else base
    target = _clamp(target, lo, hi)

    cur = base if self._tight_t_follow is None else self._tight_t_follow
    cur = _clamp(cur, lo, hi)                       # a baseline change (personality edit) can't strand us
    step = _TIGHT_SLEW_S_PER_S * step_dt
    cur = min(cur + step, target) if cur < target else max(cur - step, target)
    self._tight_t_follow = cur

    # Indistinguishable from the shared baseline -> hand back None so the untouched upstream path
    # runs (keeps the override genuinely inert whenever it isn't doing anything).
    if abs(cur - base) < 1e-3:
      return None
    return cur
