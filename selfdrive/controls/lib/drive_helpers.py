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
# THE SCHEDULE (mph -> m/s^2, linearly interpolated, held flat outside the ends) -- ONLY ACTIVE ONCE A
# VALID FILE IS LOADED FROM DISK
#   <=50 mph -> 5.0, tapering to 4.0 by 60 mph, tapering to 3.0 (ISO) by 70 mph, flat 3.0 above that.
#   LAYER-2 safety envelope (see docs/pnw/LATACCEL2PNW.md for the full writeup, incl. the 2026-08-11
#   10:37 PDT Crown Hill left-curve finding that set these anchors): curve-speed slowdown (VTSC/CES)
#   is layer 1; if it doesn't fire before a curve, this cap is what lets openpilot use the truck's
#   real steering authority to hold the lane instead of under-turning and departing.
#
# FAIL-SAFE DIRECTION: flat ISO 3.0, not the 5/4/3 schedule
#   Until a VALID lataccel_limits.json has been parsed from disk -- at boot, if the file is missing, or
#   any time it becomes unreadable/malformed/non-finite/out-of-range -- lat_accel_limit() returns the
#   flat MAX_LATERAL_ACCEL_NO_ROLL (3.0 m/s^2) at every speed, i.e. plain upstream behavior. It does
#   NOT fall back to DEFAULT_LAT_ACCEL_BREAKPOINTS_MPH (the looser 5/4/3 schedule) -- a bad/missing file
#   must never leave the car with a LOOSER cap than a driver's own stricter tune, and a deleted file
#   must revert immediately rather than latching the last-good schedule forever.
#   DEFAULT_LAT_ACCEL_BREAKPOINTS_MPH exists only as the content _write_default_once() seeds to
#   LAT_ACCEL_LIMITS_PATH the first time it's missing -- on the real device that seed write succeeds,
#   so the 5/4/3 schedule activates within ~_LAT_ACCEL_RELOAD_INTERVAL_S of boot; in a read-only
#   test/CI environment the seed write silently fails and the cap stays flat 3.0 (safe, matches stock).
#
# HOW TUNING WORKS (mirrors lane_centering.py's LaneCenteringController hot-reload pattern)
#   The schedule lives in /data/pnw/lataccel_limits.json (breakpoints in MPH so it's easy to edit on
#   the road), OUTSIDE the git tree so an auto-update's `git clean` can't delete a driver's tuning (see
#   docs/mapd-binary-wiped-by-autoupdate.md for the incident this convention is copied from). It is
#   re-stat'd at most once every _LAT_ACCEL_RELOAD_INTERVAL_S -- clip_curvature() runs at 100 Hz, so
#   this keeps steady-state cost to one cheap os.stat() call every few seconds, not a JSON parse every
#   tick. The parsed breakpoint arrays are built ONCE per successful reload (not per limit() call), and
#   a file identity (mtime_ns, size) that failed to parse is memoized so a persistently-broken file is
#   still cheaply stat'd every interval but never re-opened/re-read. Any problem loading it (missing
#   file, malformed JSON, non-finite/out-of-range values, wrong shape, more than 32 breakpoints,
#   non-strictly-increasing speeds) is fully absorbed here and RESETS to the flat-3.0 fail-safe (see
#   above) -- loading can NEVER raise out of this module, and lat_accel_limit() always returns a finite
#   float in _LAT_ACCEL_CAP_CLAMP. The effective cap returned is additionally rate-limited (see
#   LAT_ACCEL_SLEW_RATE below) so a schedule swap -- including the fail-safe revert -- can never step
#   the cap in a single control tick.
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

# Hard ceiling on breakpoint count. A JSON file is untrusted, attacker-or-typo-controlled input read
# every _LAT_ACCEL_RELOAD_INTERVAL_S by a 100 Hz control-loop caller -- without a cap, an arbitrarily
# large "breakpoints" array would mean an arbitrarily large per-reload parse/sort AND (absent the
# precomputed-array fix below) a per-tick list rebuild. 32 is far more resolution than a piecewise
# speed schedule needs; reject (not truncate) anything larger, same all-or-nothing policy as every
# other _sanitize check.
_LAT_ACCEL_MAX_BREAKPOINTS = 32

# Effective-cap slew rate: the value limit() returns can move at most this many m/s^2 per second of
# wall-clock time, regardless of how much the underlying target schedule/value just changed. This is
# what makes a schedule hot-swap (or the fail-safe revert to flat 3.0 below) a smooth transition
# instead of a single-tick step in the curvature clamp -- clip_curvature() applies the jerk/rate limit
# BEFORE this cap, so an instant cap drop while cornering would otherwise step the commanded curvature
# in one 10 ms tick. Speed-driven target changes are already gradual (v_ego doesn't jump), so this
# barely engages on them; it's here for the abrupt cases (a hot-reloaded file, or losing/regaining a
# valid file).
#
# 2026-08-11 (Crown Hill ~50 mph right curve, see drives/2026-08-11): 0.5 was too low -- during hard
# braking into a curve the scheduled cap RISES fast (speed drops -> schedule interpolates toward the
# looser low-speed end) and 0.5 m/s^2/s couldn't keep up, so the slewed cap pinned near its 3.0 floor
# and clipped a legitimate ~3.8 m/s^2 cornering demand mid-brake. The slew was only ever meant to
# soften an ABRUPT schedule swap (a hot-reload edit, or the fail-safe revert), not to throttle the
# normal speed-driven cap change, which needs only ~0.5-1.0 m/s^2/s here. Bound: at saturation, the
# lateral jerk a cap-change rate R (m/s^2/s) induces equals R (m/s^3), so any R <= MAX_LATERAL_JERK
# (5.0 m/s^3, ISO baseline) stays in the ISO jerk envelope. 4.0 gives ample margin under 5.0 while
# easily tracking real deceleration (4-8x headroom), and still softens the abrupt-swap case: a
# worst-case full-range swap (cap 6.0 -> 1.0, the [1.0, 6.0] clamp span) now ramps over ~1.25 s instead
# of a single 10 ms tick, well short of a one-tick snap.
LAT_ACCEL_SLEW_RATE = 4.0  # m/s^2 per second

# Built-in schedule -- used ONLY as the content _write_default_once() seeds to LAT_ACCEL_LIMITS_PATH
# the first time it's missing, so a driver has something to edit. It is NOT an in-memory fallback: see
# the "FAIL-SAFE DIRECTION" note above -- absent/invalid file means flat MAX_LATERAL_ACCEL_NO_ROLL, not
# this schedule. [speed_mph, accel_mps2] pairs, ascending by speed.
DEFAULT_LAT_ACCEL_BREAKPOINTS_MPH: list[list[float]] = [[50, 6.0], [60, 5.0], [70, 4.0]]


class _LatAccelSchedule:
  """Caches the parsed, sanitized speed(m/s)->cap(m/s^2) schedule loaded from LAT_ACCEL_LIMITS_PATH,
  hot-reloaded at most every _LAT_ACCEL_RELOAD_INTERVAL_S. Mirrors the hot-reload/sanitize pattern in
  lane_centering.py's LaneCenteringController._refresh_tuning() / _sanitize_tuning().

  self._xs/self._ys (precomputed once per successful reload, never rebuilt per-tick) are the ONLY
  state limit() reads for the schedule itself. Both None means "no validly-loaded schedule" -- the
  state at boot, and the state any load failure resets to -- in which case limit()'s target is the
  flat MAX_LATERAL_ACCEL_NO_ROLL fail-safe rather than any speed-scheduled value."""

  def __init__(self) -> None:
    self._xs: np.ndarray | None = None
    self._ys: np.ndarray | None = None
    self._file_id: tuple[int, int] | None = None       # (st_mtime_ns, st_size) of the last file we successfully parsed
    self._failed_file_id: tuple[int, int] | None = None  # (st_mtime_ns, st_size) of the last file that FAILED to parse
    self._last_check_mono = 0.0
    self._wrote_default = False
    self._eff_cap = MAX_LATERAL_ACCEL_NO_ROLL           # slewed value limit() returns; see LAT_ACCEL_SLEW_RATE
    self._last_limit_mono: float | None = None

  def _write_default_once(self) -> None:
    # Best-effort only, and only ever attempted once per process: /data/pnw/ may not exist yet, may be
    # read-only (tests/CI), or may race another process creating it -- all of that is fine, we simply
    # keep running on the flat MAX_LATERAL_ACCEL_NO_ROLL fail-safe like any other missing-file case.
    # O_CREAT|O_EXCL on the FINAL path (not a shared ".tmp" + rename) so we never clobber a file a
    # driver is mid-way through scp'ing into place, and never leave a stranded .tmp behind.
    if self._wrote_default:
      return
    self._wrote_default = True
    try:
      os.makedirs(os.path.dirname(LAT_ACCEL_LIMITS_PATH), exist_ok=True)
      fd = os.open(LAT_ACCEL_LIMITS_PATH, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
      with os.fdopen(fd, "w") as f:
        json.dump({
          "_comment": ("Speed-scheduled max lateral-accel cap for the curvature clip -- LAYER-2 safety " +
                       "envelope: curve-speed slowdown (VTSC/CES) is layer 1, this is the steering-" +
                       "authority backstop for when it doesn't fire before a curve. ACTIVE ONLY " +
                       "WHILE THIS FILE VALIDLY PARSES. breakpoints=[[speed_mph, accel_mps2],...], " +
                       "linearly interpolated, held flat outside the ends. Missing/corrupt/non-finite " +
                       "-> falls back to flat ISO 3.0 (NOT this schedule), gently (rate-limited, not a " +
                       "step). Hot-reloaded (~every few seconds)."),
          "breakpoints": DEFAULT_LAT_ACCEL_BREAKPOINTS_MPH,
        }, f, indent=2)
    except OSError:
      pass

  @staticmethod
  def _sanitize(raw) -> tuple[np.ndarray, np.ndarray] | None:
    """Validate raw["breakpoints"]: 2-32 entries of [speed_mph, accel_mps2], both real (non-bool)
    numbers, finite, accel within _LAT_ACCEL_CAP_CLAMP, speed non-negative, and STRICTLY increasing
    speeds (no duplicate/non-increasing breakpoints -- matches the np.all(np.diff(x) > 0) contract
    lane_centering.py's _valid_path() enforces, so np.interp behaves and "the cap at this speed" is
    unambiguous). All-or-nothing -- a schedule is a shape, not independently tunable fields, so any
    single bad entry discards the whole file rather than silently distorting the curve; the caller
    resets to the flat fail-safe on any rejection (see module docstring FAIL-SAFE DIRECTION note).
    Rejects non-finite values explicitly: json.load() parses bare NaN/Infinity/-Infinity tokens (and
    overflowing literals like 1e999) into float nan/inf WITHOUT raising, so without this isfinite gate
    a single bad on-road edit could smuggle a NaN into the curvature clamp. Rejects bool/str explicitly
    rather than relying on float()'s coercion (float("30") and float(True) both succeed silently)."""
    if not isinstance(raw, dict):
      return None
    breakpoints = raw.get("breakpoints")
    if not isinstance(breakpoints, list) or not (2 <= len(breakpoints) <= _LAT_ACCEL_MAX_BREAKPOINTS):
      return None
    parsed = []
    for entry in breakpoints:
      if not isinstance(entry, (list, tuple)) or len(entry) != 2:
        return None
      speed_mph, accel = entry[0], entry[1]
      if not (isinstance(speed_mph, (int, float)) and not isinstance(speed_mph, bool)):
        return None
      if not (isinstance(accel, (int, float)) and not isinstance(accel, bool)):
        return None
      speed_mph = float(speed_mph)
      accel = float(accel)
      if not (math.isfinite(speed_mph) and math.isfinite(accel)):
        return None
      if speed_mph < 0.0 or not (_LAT_ACCEL_CAP_CLAMP[0] <= accel <= _LAT_ACCEL_CAP_CLAMP[1]):
        return None
      parsed.append((speed_mph * CV.MPH_TO_MS, accel))
    parsed.sort(key=lambda p: p[0])
    xs = np.array([p[0] for p in parsed], dtype=float)
    ys = np.array([p[1] for p in parsed], dtype=float)
    if not np.all(np.diff(xs) > 0):
      return None  # duplicate or non-increasing speed breakpoints
    return xs, ys

  def _refresh(self) -> None:
    """Reload self._xs/self._ys from disk if enough wall-clock time has passed AND the file's
    identity (mtime_ns, size) changed since our last successful parse. Never raises. On ANY
    error/missing-file/parse-failure this RESETS to the no-schedule state (self._xs = self._ys =
    None) rather than keeping the last-good schedule -- see the module docstring's FAIL-SAFE
    DIRECTION note for why: a deleted/broken file must revert immediately, not latch."""
    now = time.monotonic()
    if now - self._last_check_mono < _LAT_ACCEL_RELOAD_INTERVAL_S:
      return
    self._last_check_mono = now

    try:
      st = os.stat(LAT_ACCEL_LIMITS_PATH)
    except OSError:
      # Missing file (or /data/pnw/ not there yet) is a normal state, not an error -- reset to the
      # fail-safe (in case a previously-valid file was just deleted), try to seed the default once so
      # a driver has something to edit, then move on.
      self._xs = self._ys = None
      self._file_id = None
      self._failed_file_id = None
      self._write_default_once()
      return

    file_id = (st.st_mtime_ns, st.st_size)

    if file_id == self._file_id:
      return  # unchanged since our last successful parse
    if file_id == self._failed_file_id:
      return  # unchanged since our last failed parse -- cheap to stat, not worth re-opening/re-reading

    try:
      with open(LAT_ACCEL_LIMITS_PATH) as f:
        raw = json.load(f)
      parsed = self._sanitize(raw)
      if parsed is None:
        raise ValueError("invalid or out-of-range breakpoints")
      self._xs, self._ys = parsed
      self._file_id = file_id
      self._failed_file_id = None
    except Exception as e:
      # Malformed JSON, wrong shape, non-finite/out-of-range values, permission error, anything --
      # RESET to the flat fail-safe (do NOT keep last-good; see FAIL-SAFE DIRECTION above) and memoize
      # this exact file identity so we don't keep re-opening/re-reading a persistently-broken file
      # every reload interval (we still cheaply stat() it every interval, so a fix is picked up as
      # soon as its mtime/size actually changes). Logging is keyed on file identity, not the error
      # message string, so two different bad edits in a row are BOTH logged -- a driver must not be
      # able to conclude a second bad edit "took" just because the message happened to repeat.
      self._xs = self._ys = None
      self._file_id = None
      self._failed_file_id = file_id
      cloudlog.error(f"drive_helpers: failed to load {LAT_ACCEL_LIMITS_PATH}, reverting to flat " +
                     f"{MAX_LATERAL_ACCEL_NO_ROLL} m/s^2 fail-safe ({type(e).__name__}: {e})")

  def limit(self, v_ego: float) -> float:
    """Returns the slewed effective cap. LAT_ACCEL_SLEW_RATE-limits the move toward the freshly
    computed target so a schedule swap (hot-reload, or the fail-safe revert to flat 3.0) is a gentle
    ramp rather than a single-tick step in the curvature clamp; the target itself is unslewed."""
    self._refresh()

    try:
      v_ego_f = float(v_ego)
    except (TypeError, ValueError):
      v_ego_f = float("nan")

    if self._xs is None or not math.isfinite(v_ego_f):
      target = MAX_LATERAL_ACCEL_NO_ROLL
    else:
      target = float(np.interp(v_ego_f, self._xs, self._ys))
      if not math.isfinite(target):
        target = MAX_LATERAL_ACCEL_NO_ROLL
    target = float(np.clip(target, *_LAT_ACCEL_CAP_CLAMP))

    now = time.monotonic()
    dt = 0.0 if self._last_limit_mono is None else float(np.clip(now - self._last_limit_mono, 0.0, 0.1))
    self._last_limit_mono = now

    max_step = LAT_ACCEL_SLEW_RATE * dt
    eff_cap = self._eff_cap + float(np.clip(target - self._eff_cap, -max_step, max_step))
    if not math.isfinite(eff_cap):
      eff_cap = MAX_LATERAL_ACCEL_NO_ROLL
    self._eff_cap = float(np.clip(eff_cap, *_LAT_ACCEL_CAP_CLAMP))
    return self._eff_cap


_lat_accel_schedule = _LatAccelSchedule()


def lat_accel_limit(v_ego: float) -> float:
  """Speed-scheduled maximum lateral acceleration (m/s^2), used by clip_curvature() in place of the
  fixed MAX_LATERAL_ACCEL_NO_ROLL constant. Hot-reloaded from LAT_ACCEL_LIMITS_PATH when a valid file
  is present; falls back to flat MAX_LATERAL_ACCEL_NO_ROLL (not the 5/4/3 schedule) otherwise -- see
  the lataccel2pnw module docstring above and docs/pnw/LATACCEL2PNW.md for the schedule and rationale.
  The return value is additionally slew-rate-limited (LAT_ACCEL_SLEW_RATE) so a schedule swap or the
  fail-safe revert can never step in a single call. Always returns a finite float in
  _LAT_ACCEL_CAP_CLAMP -- never raises, never returns NaN/Inf."""
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
