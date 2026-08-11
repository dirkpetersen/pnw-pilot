import json
import math
import os
import time

import numpy as np
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY, CV
from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.common.swaglog import cloudlog

MIN_SPEED = 1.0
CONTROL_N = 17
CAR_ROTATION_RADIUS = 0.0
# This is a turn radius smaller than most cars can achieve
MAX_CURVATURE = 0.2
MAX_VEL_ERR = 5.0  # m/s
MIN_STABLE_DELAY = 0.3

# EU guidelines
MAX_LATERAL_JERK = 5.0  # m/s^3
# ISO baseline. Also lat_accel_limit()'s hardcoded fail-safe fallback -- see the lataccel2pnw block
# below for the speed-scheduled, JSON-tunable cap that clip_curvature() actually uses.
MAX_LATERAL_ACCEL_NO_ROLL = 3.0  # m/s^2

# --------------------------------------------------------------------------------------------------
# lataccel2pnw: speed-scheduled, hot-reloaded max-lateral-accel cap for clip_curvature().
#
# WHAT THIS IS
#   Upstream clip_curvature() bounds curvature to a single fixed MAX_LATERAL_ACCEL_NO_ROLL (3.0 m/s^2,
#   the ISO baseline) at every speed. That is unnecessarily conservative at low speed (parking lots,
#   tight residential/campus curves, U-turns) where more lateral authority is safe and helpful, and
#   it's exactly right at highway speed where a hard-cornering response to a misjudged curve is the
#   WRONG fix anyway -- the correct response to a fast curve you can't hold at cruise speed is to slow
#   down beforehand (that's what VTSC / ICBM do), not to steer harder through it. So this schedule
#   tapers the cap DOWN as speed goes UP: more authority at low speed, ISO 3.0 by highway speed.
#
#   This is a shared control-envelope limit, not a feature -- it applies globally, unconditioned on
#   car/fingerprint (see the pnw capability-view rule: control limits like this either apply to every
#   car or don't belong here at all). Raising the low-speed cap never impairs the Tesla or any other
#   car; it only ever gives clip_curvature more headroom to work with at low speed.
#
# THE SCHEDULE (mph -> m/s^2, linearly interpolated, held flat outside the ends)
#   <=30 mph -> 5.0, tapering to 4.0 by 45 mph, tapering to 3.0 (ISO) by 60 mph, flat 3.0 above that.
#   See docs/LATACCEL2PNW.md for the full writeup.
#
# HOW TUNING WORKS (mirrors lane_centering.py's LaneCenteringController hot-reload pattern)
#   The schedule lives in /data/pnw/lataccel_limits.json (breakpoints in MPH so it's easy to edit on
#   the road), OUTSIDE the git tree so an auto-update's `git clean` can't delete a driver's tuning (see
#   docs/mapd-binary-wiped-by-autoupdate.md for the incident this convention is copied from). It is
#   re-stat'd at most once every _LAT_ACCEL_RELOAD_INTERVAL_S -- clip_curvature() runs at 100 Hz, so
#   this keeps steady-state cost to one cheap os.path.getmtime() call every few seconds, not a JSON
#   parse every tick. Any problem loading it (missing file, malformed JSON, non-finite/out-of-range
#   values, wrong shape) is fully absorbed here: the schedule falls back to the last-good parse, or to
#   DEFAULT_LAT_ACCEL_BREAKPOINTS_MPH if nothing has ever loaded successfully. Loading can NEVER raise
#   out of this module, and lat_accel_limit() always returns a finite float in _LAT_ACCEL_CAP_CLAMP.
# --------------------------------------------------------------------------------------------------

LAT_ACCEL_LIMITS_PATH = "/data/pnw/lataccel_limits.json"

# Control-loop hygiene limit only (keeps a 100 Hz caller from doing disk I/O 100x/sec) -- not a safety
# limit. A stale-by-a-few-seconds schedule is harmless; a 100 Hz stat() storm is not.
_LAT_ACCEL_RELOAD_INTERVAL_S = 5.0

# Hard ceiling on any cap value the JSON file can produce, fixed here and NOT reachable from the JSON.
# Below 1.0 m/s^2 lateral authority would be unusably weak even in a parking lot; above 6.0 m/s^2 is
# well past what ISO guidance (and this platform's validated tuning) supports -- a typo or a
# decimal-point slip in the on-road file must not be able to command a harder turn than that.
_LAT_ACCEL_CAP_CLAMP = (1.0, 6.0)

# Built-in schedule -- used until/unless a valid JSON file is found, used again the moment the on-disk
# file becomes unreadable/invalid, and also written out as the on-disk default the first time
# LAT_ACCEL_LIMITS_PATH is missing. [speed_mph, accel_mps2] pairs, ascending by speed.
DEFAULT_LAT_ACCEL_BREAKPOINTS_MPH: list[list[float]] = [[30, 5.0], [45, 4.0], [60, 3.0]]


class _LatAccelSchedule:
  """Caches the parsed, sanitized speed(m/s)->cap(m/s^2) schedule loaded from LAT_ACCEL_LIMITS_PATH,
  hot-reloaded at most every _LAT_ACCEL_RELOAD_INTERVAL_S. Mirrors the hot-reload/sanitize pattern in
  lane_centering.py's LaneCenteringController._refresh_tuning() / _sanitize_tuning()."""

  def __init__(self) -> None:
    self._breakpoints_ms = self._to_ms(DEFAULT_LAT_ACCEL_BREAKPOINTS_MPH)
    self._mtime: float | None = None
    self._last_check_mono = 0.0
    self._last_load_error: str | None = None
    self._wrote_default = False

  @staticmethod
  def _to_ms(breakpoints_mph) -> list[tuple[float, float]]:
    return [(speed_mph * CV.MPH_TO_MS, float(accel)) for speed_mph, accel in breakpoints_mph]

  def _write_default_once(self) -> None:
    # Best-effort only, and only ever attempted once per process: /data/pnw/ may not exist yet, may be
    # read-only (tests/CI), or may race another process creating it -- all of that is fine, we simply
    # keep running on the in-memory DEFAULT_LAT_ACCEL_BREAKPOINTS_MPH like any other missing-file case.
    if self._wrote_default:
      return
    self._wrote_default = True
    try:
      os.makedirs(os.path.dirname(LAT_ACCEL_LIMITS_PATH), exist_ok=True)
      if not os.path.exists(LAT_ACCEL_LIMITS_PATH):
        tmp_path = LAT_ACCEL_LIMITS_PATH + ".tmp"
        with open(tmp_path, "w") as f:
          json.dump({
            "_comment": ("Speed-scheduled max lateral-accel cap for the curvature clip. " +
                         "breakpoints=[[speed_mph, accel_mps2],...], linearly interpolated, held flat " +
                         "outside the ends. ISO baseline 3.0. Hot-reloaded (~every few seconds). " +
                         "Corrupt/non-finite -> falls back to flat 3.0."),
            "breakpoints": DEFAULT_LAT_ACCEL_BREAKPOINTS_MPH,
          }, f, indent=2)
        os.rename(tmp_path, LAT_ACCEL_LIMITS_PATH)  # atomic, avoids a reader ever seeing a partial write
    except OSError:
      pass

  @staticmethod
  def _sanitize(raw) -> list[tuple[float, float]] | None:
    """Validate raw["breakpoints"]: >=2 entries of [speed_mph, accel_mps2], both finite numbers,
    accel within _LAT_ACCEL_CAP_CLAMP, speed non-negative. All-or-nothing -- a schedule is a shape,
    not independently tunable fields, so any single bad entry discards the whole file rather than
    silently distorting the curve; the caller falls back to the last-good (or default) schedule.
    Rejects non-finite values explicitly: json.load() parses bare NaN/Infinity/-Infinity tokens (and
    overflowing literals like 1e999) into float nan/inf WITHOUT raising, so without this isfinite gate
    a single bad on-road edit could smuggle a NaN into the curvature clamp."""
    if not isinstance(raw, dict):
      return None
    breakpoints = raw.get("breakpoints")
    if not isinstance(breakpoints, list) or len(breakpoints) < 2:
      return None
    parsed = []
    for entry in breakpoints:
      if not isinstance(entry, (list, tuple)) or len(entry) != 2:
        return None
      try:
        speed_mph = float(entry[0])
        accel = float(entry[1])
      except (TypeError, ValueError):
        return None
      if not (math.isfinite(speed_mph) and math.isfinite(accel)):
        return None
      if speed_mph < 0.0 or not (_LAT_ACCEL_CAP_CLAMP[0] <= accel <= _LAT_ACCEL_CAP_CLAMP[1]):
        return None
      parsed.append((speed_mph * CV.MPH_TO_MS, accel))
    parsed.sort(key=lambda p: p[0])
    return parsed

  def _refresh(self) -> None:
    """Reload self._breakpoints_ms from disk if enough wall-clock time has passed AND the file's
    mtime changed since the last successful parse. Never raises: any failure leaves the schedule
    exactly as it was (last-good, or the built-in default if nothing has ever loaded)."""
    now = time.monotonic()
    if now - self._last_check_mono < _LAT_ACCEL_RELOAD_INTERVAL_S:
      return
    self._last_check_mono = now

    try:
      mtime = os.path.getmtime(LAT_ACCEL_LIMITS_PATH)
    except OSError:
      # Missing file (or /data/pnw/ not there yet) is a normal state, not an error -- try to seed the
      # default once so a driver has something to edit, then move on.
      self._write_default_once()
      return

    if mtime == self._mtime:
      return  # unchanged since our last successful parse

    try:
      with open(LAT_ACCEL_LIMITS_PATH) as f:
        raw = json.load(f)
      parsed = self._sanitize(raw)
      if parsed is None:
        raise ValueError("invalid or out-of-range breakpoints")
      self._breakpoints_ms = parsed
      self._mtime = mtime
      self._last_load_error = None
    except Exception as e:
      # Malformed JSON, wrong shape, non-finite/out-of-range values, permission error, anything --
      # keep the last-good schedule and move on. Log only when the error message is new, so a
      # persistently-broken file doesn't spam the log for an entire drive.
      msg = f"{type(e).__name__}: {e}"
      if msg != self._last_load_error:
        cloudlog.error(f"drive_helpers: failed to load {LAT_ACCEL_LIMITS_PATH}, keeping last-good lat-accel schedule ({msg})")
        self._last_load_error = msg
      # Deliberately do NOT update self._mtime here, so a subsequent fix is retried next check even if
      # it happens to land on a mtime we've already seen.

  def limit(self, v_ego: float) -> float:
    self._refresh()
    try:
      v_ego_f = float(v_ego)
    except (TypeError, ValueError):
      return MAX_LATERAL_ACCEL_NO_ROLL
    if not math.isfinite(v_ego_f):
      return MAX_LATERAL_ACCEL_NO_ROLL
    xs = [p[0] for p in self._breakpoints_ms]
    ys = [p[1] for p in self._breakpoints_ms]
    cap = float(np.interp(v_ego_f, xs, ys))
    if not math.isfinite(cap):
      return MAX_LATERAL_ACCEL_NO_ROLL
    return float(np.clip(cap, *_LAT_ACCEL_CAP_CLAMP))


_lat_accel_schedule = _LatAccelSchedule()


def lat_accel_limit(v_ego: float) -> float:
  """Speed-scheduled maximum lateral acceleration (m/s^2), used by clip_curvature() in place of the
  fixed MAX_LATERAL_ACCEL_NO_ROLL constant. Hot-reloaded from LAT_ACCEL_LIMITS_PATH; see the
  lataccel2pnw module docstring above and docs/LATACCEL2PNW.md for the schedule and rationale.
  Always returns a finite float in _LAT_ACCEL_CAP_CLAMP -- never raises, never returns NaN/Inf."""
  return _lat_accel_schedule.limit(v_ego)


def clamp(val, min_val, max_val):
  clamped_val = float(np.clip(val, min_val, max_val))
  return clamped_val, clamped_val != val

def smooth_value(val, prev_val, tau, dt=DT_MDL):
  alpha = 1 - np.exp(-dt/tau) if tau > 0 else 1
  return alpha * val + (1 - alpha) * prev_val

def clip_curvature(v_ego, prev_curvature, new_curvature, roll) -> tuple[float, bool]:
  # This function respects ISO lateral jerk and acceleration limits + a max curvature
  v_ego = max(v_ego, MIN_SPEED)
  max_curvature_rate = MAX_LATERAL_JERK / (v_ego ** 2)  # inexact calculation, check https://github.com/commaai/openpilot/pull/24755
  new_curvature = np.clip(new_curvature,
                          prev_curvature - max_curvature_rate * DT_CTRL,
                          prev_curvature + max_curvature_rate * DT_CTRL)

  roll_compensation = roll * ACCELERATION_DUE_TO_GRAVITY
  # lataccel2pnw: speed-scheduled + JSON-tunable cap in place of the fixed MAX_LATERAL_ACCEL_NO_ROLL.
  lat_accel_cap = lat_accel_limit(v_ego)
  max_lat_accel = lat_accel_cap + roll_compensation
  min_lat_accel = -lat_accel_cap + roll_compensation
  new_curvature, limited_accel = clamp(new_curvature, min_lat_accel / v_ego ** 2, max_lat_accel / v_ego ** 2)

  new_curvature, limited_max_curv = clamp(new_curvature, -MAX_CURVATURE, MAX_CURVATURE)
  return float(new_curvature), limited_accel or limited_max_curv


def get_accel_from_plan(speeds, accels, t_idxs, action_t=DT_MDL, vEgoStopping=0.05):
  if len(speeds) == len(t_idxs):
    v_now = speeds[0]
    a_now = accels[0]
    if action_t < MIN_STABLE_DELAY:
      v_target = v_now + (action_t / MIN_STABLE_DELAY) * (np.interp(MIN_STABLE_DELAY, t_idxs, speeds) - v_now)
    else:
      v_target = np.interp(action_t, t_idxs, speeds)
    a_target = 2 * (v_target - v_now) / (action_t) - a_now
    v_target_1sec = np.interp(action_t + 1.0, t_idxs, speeds)
  else:
    v_target = 0.0
    v_target_1sec = 0.0
    a_target = 0.0
  should_stop = (v_target < vEgoStopping and
                 v_target_1sec < vEgoStopping)
  return a_target, should_stop

def curv_from_psis(psi_target, psi_rate, vego, action_t):
  vego = np.clip(vego, MIN_SPEED, np.inf)
  curv_from_psi = psi_target / (vego * action_t)
  return 2*curv_from_psi - psi_rate / vego

def get_curvature_from_plan(yaws, yaw_rates, t_idxs, vego, action_t):
  if action_t < MIN_STABLE_DELAY:
    psi_target = (action_t / MIN_STABLE_DELAY) * np.interp(MIN_STABLE_DELAY, t_idxs, yaws)
  else:
    psi_target = np.interp(action_t, t_idxs, yaws)
  psi_rate = yaw_rates[0]
  return curv_from_psis(psi_target, psi_rate, vego, action_t)
