"""
Lane Centering — a small, bounded steering micro-correction toward lane-line center.

WHAT THIS IS
  The end-to-end driving model's desired curvature already tries to follow a sensible path, but it
  is not always centered between the visible lane lines (it can hug one side, especially on wide or
  worn-marking roads). This controller compares where the model's own path lands, at a short
  lookahead distance, against the midpoint of the left/right lane lines it also reports — and adds a
  small correction to the curvature to nudge the car toward that midpoint. It is a *trim*, not a
  replacement path planner: every path the raw correction can produce is hard-clamped to a tiny
  curvature delta (see `max_raw_correction` below) long before it reaches the actuator.

  This is a straight port of StarPilot's `LaneCenteringController`
  (selfdrive/controls/lib/lane_centering.py in the frog/StarPilot tree). The correction math, the
  confidence gates, the graceful smooth-release behavior, and the end-to-end (E2E) authority blend
  are preserved EXACTLY — this file changes only how the controller is *tuned*, not what it computes.
  If you are auditing this port for correctness, diff `_raw_correction()` and `update()` against the
  StarPilot source line-by-line; they should match modulo the tuning-source indirection described
  next.

HOW TUNING WORKS HERE (the one real change from StarPilot)
  StarPilot hardcoded its tuning as module-level constants, with `offset`/`e2e_authority` passed in
  by the caller. Here, ALL tunables (including offset and e2e_authority) instead live in a JSON file
  on disk, `/data/pnw/lanecenter_tuning.json`, so they can be tweaked and *hot-reloaded* while
  driving — no code change, no reboot, no manager restart. `/data/pnw/` is used (not the openpilot
  git tree) for the same reason `system/mapd`'s downloaded binary was moved out of the tree: an
  auto-update `git clean` wipes untracked files inside the repo, and this tuning file must survive
  that (see docs/mapd-binary-wiped-by-autoupdate.md for the incident this pattern is copied from).

  The file is re-read at most once per second (`_RELOAD_INTERVAL_S`), and only re-parsed if its
  mtime actually changed — so a steady-state drive costs one cheap `os.path.getmtime()` stat call
  per second, not a JSON parse on every 100 Hz control tick. Any problem loading it — file absent,
  malformed JSON, wrong shape, `/data/pnw/` not yet created, a permissions error, anything — is fully
  absorbed here: the controller falls back to (a) the last tuning it successfully loaded, or (b) the
  hardcoded `DEFAULT_TUNING` if nothing has ever loaded successfully. Reading tuning can NEVER raise
  out of this module and NEVER stalls or crashes the 100 Hz control loop that calls `update()`.

THE SAFETY-ENVELOPE CONTRACT (read this before editing `_CLAMPS`)
  The JSON file is a live, on-road-editable input — effectively a lightweight remote-tuning channel.
  That means it must be treated as UNTRUSTED input to a lateral-control component, exactly like any
  other steering-adjacent value: a bad edit (typo, decimal-point slip, copy-paste from a different
  key) must never be able to command something the control loop wasn't designed to survive.

  `_sanitize_tuning()` is where that trust boundary lives. After loading and before ANY value from
  the JSON is used, every field is clamped into a hardcoded `(lo, hi)` envelope in `_CLAMPS` — the
  JSON can tune *within* that envelope, but the envelope itself is fixed in this file and is not
  reachable from the JSON. Each clamp below is commented with the specific failure it exists to
  prevent. A handful of fields also have cross-field invariants checked afterward (e.g. lane-width
  min must stay below lane-width max) — if an invariant is violated the whole tuning load is
  considered untrustworthy and DEFAULT_TUNING is used instead, rather than trying to guess a
  "reasonable" repair.

  In short: the JSON is advisory WITHIN a safety envelope that is fixed in this source file. Nothing
  written to `/data/pnw/lanecenter_tuning.json` can widen that envelope.
"""

import json
import math
import os
import time

import numpy as np
from cereal import log

from openpilot.common.realtime import DT_CTRL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.drive_helpers import smooth_value

# Live, hot-reloaded tuning file. Deliberately OUTSIDE the git tree (see module docstring) so an
# auto-update's `git clean` cannot delete it out from under a driver who has tuned it.
TUNING_PATH = "/data/pnw/lanecenter_tuning.json"

# How often (seconds) we're willing to even stat() the tuning file. This is a control-loop hygiene
# limit, not a safety limit — its only job is to keep a 100 Hz loop from doing disk I/O 100x/sec.
_RELOAD_INTERVAL_S = 1.0

# Every tunable, defaulted to StarPilot's original hardcoded module-constant values (see the module
# docstring). These are the values used until/unless a valid tuning file is found on disk, and they
# are also the fallback the moment the on-disk file becomes unreadable or invalid.
DEFAULT_TUNING: dict[str, float | bool] = {
  "offset": 0.0,                  # lateral bias from lane-midpoint, +right/-left (m)
  "e2e_authority": 1.0,           # how much the E2E model path can veto the lane-line correction (0-1)
  "deadband": 0.08,               # center-error deadband before any correction starts (m)
  "max_gain": 0.30,               # final scalar applied to the clamped raw correction
  "max_raw_correction": 0.004,    # hard clamp on the raw (pre-gain) curvature correction (1/m)
  "min_lane_width": 2.6,          # reject the lane-line read if apparent width is below this (m)
  "max_lane_width": 4.8,          # reject the lane-line read if apparent width is above this (m)
  "min_center_to_line": 1.1,      # required clearance kept between target point and either line (m)
  "max_offset": 0.3,              # hard clamp on `offset` (m)
  "lookahead_min": 8.0,           # lower bound of the speed-derived lookahead distance (m)
  "lookahead_max": 35.0,          # upper bound of the speed-derived lookahead distance (m)
  "smooth_tau": 0.4,              # time constant easing the correction toward its target (s)
  "signal_release_tau": 0.20,     # time constant releasing the correction during a turn signal (s)
  "confidence_release_tau": 0.20, # time constant releasing the correction when confidence is lost (s)
  "min_lane_prob": 0.6,           # minimum model confidence required in both adjacent lane lines
  "max_lane_std": 0.3,            # maximum model uncertainty (std) allowed in either lane line (m)
  "min_v_ego": 5.0,               # minimum speed before the correction is allowed to act (m/s)
  "e2e_max_path_std": 0.35,       # E2E path uncertainty ceiling for the E2E-authority blend to apply
  "e2e_break_in_start": 0.15,     # center-error magnitude where E2E authority starts reducing the correction (m)
  "e2e_break_in_full": 0.50,      # center-error magnitude where E2E authority is fully applied (m)
}

# --------------------------------------------------------------------------------------------------
# THE SAFETY ENVELOPE. Hardcoded (lo, hi) bounds every tuning value is clamped into, no matter what
# the JSON file says. See the "SAFETY-ENVELOPE CONTRACT" section of the module docstring.
# --------------------------------------------------------------------------------------------------
_CLAMPS: dict[str, tuple[float, float]] = {
  # +/-0.3 m is StarPilot's original hard offset ceiling. Beyond this the car would be deliberately
  # aimed at a meaningful fraction of a lane width away from center — well past "centering".
  "offset": (-0.3, 0.3),
  # e2e_authority blends the correction against the E2E model's own path; it's a mix fraction and is
  # only ever meaningful in [0, 1] — anything outside that would extrapolate the blend, not mix it.
  "e2e_authority": (0.0, 1.0),
  # Deadband beyond half a typical lane width would be nonsensical (correction could never start);
  # letting it go to 0 is fine (just noisier, still bounded by max_raw_correction downstream).
  "deadband": (0.0, 0.5),
  # max_gain is the last scalar multiplier before the correction is added to desired curvature.
  # 0.5 is a hard ceiling well above the 0.30 default — chosen so a bad edit can dial the effect up
  # but never past "still visibly a trim, not a replan" (max_raw_correction is the real hard stop).
  "max_gain": (0.0, 0.5),
  # THE key safety number: max_raw_correction bounds the raw (pre-gain) curvature nudge in 1/m. At
  # 0.01 1/m and highway speed this is still a gentle nudge, not a maneuver — this is the ceiling a
  # JSON edit can never exceed even if every other multiplier (max_gain) is also maxed out.
  "max_raw_correction": (0.0, 0.01),
  # Lane width bounds gate whether a lane-line read is trusted at all. 2.0-6.0 m covers everything
  # from a narrow work-zone lane to a wide multi-purpose lane; outside that the "lane" the model
  # reported is probably not a real lane and the correction should refuse to trust it (handled by
  # the invariant check below, and by _raw_correction's own width-range rejection).
  "min_lane_width": (2.0, 6.0),
  "max_lane_width": (2.0, 6.0),
  # Minimum clearance the target point must keep from either lane line. Floored above 0 so a bad
  # edit can't let the "centering" target land ON a lane line; ceiling keeps it from swallowing the
  # entire lane (which would silently zero out max_offset's effective range).
  "min_center_to_line": (0.3, 2.0),
  "max_offset": (0.0, 0.3),
  # Lookahead is a distance we interpolate the model's lane-line/path arrays at. Below ~5 m the
  # model's own near-field geometry is noisy; above ~60 m we would be reading (and correcting
  # curvature toward) something well beyond where lane-line detection is reliable.
  "lookahead_min": (5.0, 60.0),
  "lookahead_max": (5.0, 60.0),
  # Every tau feeds `smooth_value()`'s exponential blend. A tau of exactly 0 means "apply instantly,
  # no smoothing at all" in that helper — which would defeat the entire point of the graceful
  # release behavior this controller relies on for safety (a lost-confidence or turn-signal event
  # must decay the correction, not step it). Floor just above zero; 5 s ceiling keeps a "smoothed"
  # release from effectively becoming "never releases".
  "smooth_tau": (0.01, 5.0),
  "signal_release_tau": (0.01, 5.0),
  "confidence_release_tau": (0.01, 5.0),
  # min_lane_prob is a model-reported probability — [0, 1] is its entire valid domain.
  "min_lane_prob": (0.0, 1.0),
  # max_lane_std (and e2e_max_path_std below) are model uncertainty in METERS, NOT a normalized
  # [0, 1] quantity. The 1.0 m ceiling is a conservative cap: a lane line / path with more than ~1 m
  # of predicted std is far too uncertain to trust for a centering trim, so clamping the ceiling to
  # 1.0 m can only ever tighten this confidence gate, never loosen it below a usable value.
  "max_lane_std": (0.0, 1.0),
  # Minimum speed gate. Floored at 2.0 m/s (~4.5 mph) so a JSON edit can't enable the correction at
  # near-parking-lot speeds, where lane geometry and the lookahead math (which divides by
  # lookahead**2) get unstable. Capped at 40 m/s (~90 mph) purely so a fat-fingered huge value
  # can't silently read as "feature disabled" without anything showing why.
  "min_v_ego": (2.0, 40.0),
  "e2e_max_path_std": (0.0, 1.0),    # E2E path uncertainty ceiling, METERS (see max_lane_std note)
  "e2e_break_in_start": (0.0, 1.0),  # center-error magnitude, METERS, where the E2E veto starts
  "e2e_break_in_full": (0.0, 1.0),   # center-error magnitude, METERS, where the E2E veto is full
}


def _tele_float(value) -> float | None:
  """
  Coerce `value` to a plain finite float for the TELEMETRY-ONLY `status` dict, or None if it isn't
  one. Used only to populate `LaneCenteringController.status` fields — never read back by any
  control-path code, so it is deliberately more permissive/lossy than `_clamp` (no envelope, no
  fallback-to-default; unusable values just become None, meaning "not available this tick").
  """
  try:
    v = float(value)
  except (TypeError, ValueError):
    return None
  return v if math.isfinite(v) else None


def _clamp(value, lo: float, hi: float, default: float) -> float:
  """
  Clamp a single scalar into [lo, hi], falling back to `default` for anything unusable.

  `default` is this field's DEFAULT_TUNING value (already inside the envelope), NOT `lo` — a bad
  `offset` must fall back to 0.0 (centered), not to -0.3 (hard left), so the fallback has to be the
  field's own safe default rather than the low edge of its clamp range.

  Rejects NON-FINITE values explicitly. This is a safety boundary, not a nicety: json.load() parses
  bare `NaN`/`Infinity`/`-Infinity` tokens (and overflowing literals like 1e999) into float
  nan/inf WITHOUT raising, and np.clip(nan, lo, hi) returns nan — so without this isfinite gate a
  single bad on-road JSON edit could smuggle a NaN past the entire clamp envelope and on into the
  curvature command. Any nan/inf here is treated as an unusable edit and replaced by the default.
  """
  try:
    v = float(value)
  except (TypeError, ValueError):
    return default
  if not math.isfinite(v):
    return default
  return float(np.clip(v, lo, hi))


def _sanitize_tuning(raw: dict) -> dict[str, float | bool]:
  """
  Turn an arbitrary (untrusted) JSON object into a fully safe tuning dict.

  Every key defaults to DEFAULT_TUNING's value. Any key present in `raw` is taken, coerced to the
  right type, and clamped into `_CLAMPS`. Keys absent from `raw`, or present with a value that can't
  be coerced, silently fall back to the default for that one key (a partially-filled tuning file is
  fine — you don't have to specify every tunable to override a couple of them).

  After per-field clamping, a small set of CROSS-FIELD invariants are checked. If one is violated,
  the entire pair of fields involved is reset to defaults rather than guessing a fix — a violated
  invariant (e.g. min width >= max width) usually means the file was hand-edited into a nonsensical
  state, and refusing to "creatively" repair it is safer than silently reinterpreting the driver's
  intent.
  """
  out = dict(DEFAULT_TUNING)

  for key, default in DEFAULT_TUNING.items():
    if key not in raw:
      continue
    value = raw[key]
    if isinstance(default, bool):
      # bool is a subclass of int in Python, so this must be checked before the float branch.
      out[key] = bool(value)
    else:
      lo, hi = _CLAMPS[key]
      out[key] = _clamp(value, lo, hi, float(default))

  # Invariant: lane-width band must be non-empty, or every lane-line read gets rejected forever.
  if out["min_lane_width"] >= out["max_lane_width"]:
    out["min_lane_width"] = DEFAULT_TUNING["min_lane_width"]
    out["max_lane_width"] = DEFAULT_TUNING["max_lane_width"]

  # Invariant: lookahead band must be non-empty for the same reason.
  if out["lookahead_min"] >= out["lookahead_max"]:
    out["lookahead_min"] = DEFAULT_TUNING["lookahead_min"]
    out["lookahead_max"] = DEFAULT_TUNING["lookahead_max"]

  # Invariant: the E2E break-in ramp divides by (full - start); start must stay strictly below full
  # or that division blows up (and the ramp direction would be backwards).
  if out["e2e_break_in_start"] >= out["e2e_break_in_full"]:
    out["e2e_break_in_start"] = DEFAULT_TUNING["e2e_break_in_start"]
    out["e2e_break_in_full"] = DEFAULT_TUNING["e2e_break_in_full"]

  return out


class LaneCenteringController:
  """
  Computes a small, bounded curvature correction that nudges the car toward the midpoint of the
  model's own left/right lane-line detections.

  Call `update()` once per control-loop tick (100 Hz) with the model's desired curvature; it returns
  a (possibly) adjusted curvature. Call `reset()` whenever lateral control disengages, so the
  correction doesn't "jump" on re-engagement — the caller (controlsd) is expected to do this from the
  same place it resets the other lateral/longitudinal controllers.

  This class holds two independent pieces of state: `self._correction` (the smoothed correction
  itself, carried tick-to-tick so it can ramp instead of stepping) and `self._tuning` (the current
  sanitized tuning dict, refreshed from disk at most once a second). Nothing else persists across
  calls.

  `self.status` is a THIRD, TELEMETRY-ONLY piece of state: a plain dict snapshot of "what happened
  this tick" (the applied correction, whether it's actively acting, why not if it isn't, and the
  lane-line/path geometry the gates just checked). It exists purely so an external process (the CES
  event logger, over a cross-process mem-param — see controlsd.py/ces_pnw.py) can observe this
  controller for drive analysis. It is written continuously by `update()`/`_raw_correction()` but is
  NEVER read by any control-path code in this class — it cannot feed back into `self._correction` or
  any gate, by construction. Treat it as write-only from this class's own perspective.
  """

  def __init__(self) -> None:
    self._correction = 0.0
    self._tuning: dict[str, float | bool] = dict(DEFAULT_TUNING)
    self._tuning_mtime: float | None = None
    self._last_check_mono = 0.0
    # Remembers the last error string we logged, so a persistently-broken tuning file logs once
    # (when the failure starts, and again if the failure message changes) instead of once a second
    # for an entire drive.
    self._last_load_error: str | None = None
    # Telemetry-only snapshot of the current tick's state (see class docstring). Never read by
    # control logic.
    self.status: dict = self._new_status()

    # Write-path convenience only: make sure /data/pnw/ exists so a driver (or a future settings
    # UI) that wants to drop a tuning file there doesn't have to create the directory by hand first.
    # Guarded and best-effort — the READ path (_refresh_tuning, via os.path.getmtime's OSError catch)
    # does NOT depend on this succeeding; if /data/pnw can't be created (read-only rootfs, race with
    # another process, whatever) we simply keep running on DEFAULT_TUNING like any other
    # missing-file case.
    try:
      os.makedirs(os.path.dirname(TUNING_PATH), exist_ok=True)
    except OSError:
      pass

  def reset(self) -> None:
    """
    Zero the carried correction. Call this on every lateral disengage (see controlsd.py).

    Also resets `self.status` (telemetry-only, see class docstring) to a neutral "not acting"
    snapshot. `update()` overwrites the `gate`/`corr`/`act`/`v` fields with the specific reason on
    every one of its own internal reset() calls, so this default is only ever visibly "final" when
    a caller invokes reset() directly (i.e. on disengage, between update() calls).
    """
    self._correction = 0.0
    self.status = self._new_status()

  @staticmethod
  def _new_status() -> dict:
    """
    Fresh telemetry scaffold for one tick. Geometry fields (`err`/`p1`/`p2`/`s1`/`s2`/`yStd`/`w`)
    start as None ("not computed this tick") and are filled in by `_raw_correction()` only on the
    code paths that actually reach that math; any tick gated out before then correctly reports them
    as None rather than carrying over a stale value from a previous tick. Telemetry-only — see the
    class docstring; nothing here is ever read by control logic.
    """
    return {
      "corr": 0.0,     # applied correction this tick (1/m)
      "act": False,    # whether that correction is nonzero (actively nudging)
      "gate": "off",   # why it's not fully acting; see _finish_status for the reason codes
      "err": None,     # center error at lookahead (m)
      "p1": None,      # laneLineProbs[1] (left ego line)
      "p2": None,      # laneLineProbs[2] (right ego line)
      "s1": None,      # laneLineStds[1] (m)
      "s2": None,      # laneLineStds[2] (m)
      "yStd": None,    # E2E path position.yStd at lookahead (m)
      "w": None,       # apparent lane width at lookahead (m)
      "v": 0.0,        # v_ego (m/s)
    }

  def _finish_status(self, gate: str, v_ego) -> None:
    """
    Finalize `self.status` for this tick: record the gate reason and the correction actually
    applied, using whatever `self._correction`/geometry fields are already in place by this point.
    Called at every `update()` return point, right before returning. Telemetry-only side effect —
    `self._correction` is already final by the time this runs, so this can never feed back into
    control. Wrapped so it can never raise into the 100 Hz caller no matter what `v_ego` is.

    Gate reason codes:
      "ok"       - actively correcting
      "off"      - feature disabled (not `enabled`)
      "slow"     - v_ego below min_v_ego
      "nolat"    - lateral control not active, or modelV2 not valid/fresh
      "signal"   - turn signal on; correction smoothly releasing
      "lanechange" - an automatic lane change is in progress (or its state couldn't be read)
      "lowconf"  - lane-line confidence/geometry failed _raw_correction's checks
      "err"      - a defensive fallback fired (malformed/non-finite input); should not happen live
    """
    try:
      self.status["gate"] = gate
      self.status["corr"] = float(self._correction)
      self.status["act"] = bool(self._correction != 0.0)
      v = float(v_ego)
      if math.isfinite(v):
        self.status["v"] = v
    except Exception:
      pass

  def _refresh_tuning(self) -> None:
    """
    Reload `self._tuning` from TUNING_PATH if (a) enough wall-clock time has passed since the last
    check, and (b) the file's mtime has actually changed since the last successful load. Never
    raises: any failure leaves `self._tuning` exactly as it was (last-good, or DEFAULT_TUNING if
    nothing has ever loaded).
    """
    now = time.monotonic()
    if now - self._last_check_mono < _RELOAD_INTERVAL_S:
      return
    self._last_check_mono = now

    try:
      mtime = os.path.getmtime(TUNING_PATH)
    except OSError:
      # File doesn't exist (yet, or ever) or /data/pnw isn't there — that's a completely normal
      # state (nobody has tuned this feature), not an error worth logging every second.
      return

    if mtime == self._tuning_mtime:
      return  # unchanged since our last successful parse; nothing to do

    try:
      with open(TUNING_PATH) as f:
        raw = json.load(f)
      if not isinstance(raw, dict):
        raise ValueError(f"tuning file root must be a JSON object, got {type(raw).__name__}")
      self._tuning = _sanitize_tuning(raw)
      self._tuning_mtime = mtime
      self._last_load_error = None
    except Exception as e:
      # Malformed JSON, wrong shape, permission error, anything — keep the last-good tuning (or
      # DEFAULT_TUNING if we've never had a good one) and move on. Log only when the error is new,
      # so a broken file doesn't spam the log for an entire drive.
      msg = f"{type(e).__name__}: {e}"
      if msg != self._last_load_error:
        cloudlog.error(f"lane_centering: failed to load {TUNING_PATH}, keeping last-good tuning ({msg})")
        self._last_load_error = msg
      # Deliberately do NOT update self._tuning_mtime here: if the file gets fixed (same mtime as
      # some earlier broken write is astronomically unlikely, but if it doesn't change we'd want to
      # keep retrying) we still want a real edit to be picked up.

  def update(self, model_curvature, model_v2, v_ego, enabled, lat_active, model_valid, turn_signal_active) -> float:
    """
    Return `model_curvature`, optionally nudged toward lane-line center.

    Args:
      model_curvature: the curvature the caller would otherwise use (1/m).
      model_v2: the modelV2 message (lane lines, path, meta.laneChangeState).
      v_ego: current vehicle speed (m/s).
      enabled: master enable — the caller's derived "feature should run" flag (already reflects the
        UI toggle; see controlsd.py for how that's computed. This function does not read Params.)
      lat_active: whether lateral control is currently active (CC.latActive).
      model_valid: whether modelV2 is currently valid/fresh (caller's freshness check).
      turn_signal_active: whether either turn signal is currently on.

    Every early-return path below either returns the correction smoothly decaying toward zero, or
    calls reset() and returns the correction as exactly zero — never a discontinuous jump. This
    function never raises: any unexpected input (NaN, wrong types, malformed model_v2) is treated as
    "can't trust this tick" and degrades to returning model_curvature unchanged.

    Also refreshes `self.status`, a telemetry-only snapshot of this tick's gate/correction/geometry
    (see class docstring). This is a pure observation side effect: it is written from values already
    computed for the control decision above, never the other way around, and its own failure modes
    are fully self-contained (wrapped so they can never raise into this 100 Hz loop).
    """
    # Fresh telemetry scaffold for this tick (see _new_status). Every return path below finalizes
    # it via self.reset() and/or self._finish_status() before returning.
    self.status = self._new_status()

    try:
      model_curvature = float(model_curvature)
    except (TypeError, ValueError):
      # The caller (controlsd) always passes a float, but honor the "never raises" contract even so:
      # if the input curvature is somehow un-floatable there is nothing to correct, so drop out.
      self.reset()
      self._finish_status("err", v_ego)
      return 0.0
    if not math.isfinite(model_curvature):
      # A non-finite input curvature is an upstream problem we can't fix by adding to it; don't try.
      self.reset()
      self._finish_status("err", v_ego)
      return model_curvature

    # Pull current tuning before anything else. This can't fail or block (see _refresh_tuning).
    self._refresh_tuning()
    t = self._tuning

    try:
      v_ego = float(v_ego)
      offset = float(t["offset"])
      e2e_authority = float(t["e2e_authority"])
    except (TypeError, ValueError, KeyError):
      # Shouldn't happen (t is always a sanitized dict with every key), but if it ever does, fail
      # to the safest state: no correction.
      self.reset()
      self._finish_status("err", v_ego)
      return model_curvature

    if not np.isfinite([v_ego, offset, e2e_authority]).all():
      self.reset()
      self._finish_status("err", v_ego)
      return model_curvature

    # Master gate: not enabled, not actively steering, model not fresh, or too slow to trust the
    # lookahead geometry -> no correction, and don't carry stale state into the next activation.
    if not model_valid or not enabled or not lat_active or v_ego < t["min_v_ego"]:
      self.reset()
      # Telemetry only: pick the single most-specific reason, in priority order below. This
      # doesn't change behavior (the gate above already fired) — it just labels *why*.
      if not enabled:
        gate = "off"
      elif not model_valid or not lat_active:
        gate = "nolat"
      else:
        gate = "slow"
      self._finish_status(gate, v_ego)
      return model_curvature

    # Turn signal in progress (about to change lanes, or just indicating): smoothly RELEASE any
    # existing correction rather than fighting the driver's/planner's intent to move laterally.
    # This does not reset() — a release is a smooth decay, a reset is an instant snap to zero; the
    # decay is the point here (see signal_release_tau in the safety-envelope comments above).
    # This release is ALWAYS active and is deliberately NOT tunable from the JSON: it is a promised
    # safety behavior (the UI toggle text guarantees the trim "fades off … if you signal a turn"),
    # and the safety-envelope contract forbids the JSON from being able to disable a safety gate.
    # (StarPilot exposed a `pause_on_signal` switch here; we intentionally dropped it.)
    if turn_signal_active:
      self._correction = float(smooth_value(0.0, self._correction, t["signal_release_tau"], dt=DT_CTRL))
      self._finish_status("signal", v_ego)
      return model_curvature + self._correction

    try:
      if model_v2.meta.laneChangeState != log.LaneChangeState.off:
        # An automatic lane change is in progress (nudgeless or otherwise) — never fight that path.
        self.reset()
        self._finish_status("lanechange", v_ego)
        return model_curvature
    except (AttributeError, TypeError, ValueError):
      # Malformed modelV2.meta -> can't tell if a lane change is happening -> assume the worst and
      # disable rather than risk correcting during an actual lane change.
      self.reset()
      self._finish_status("lanechange", v_ego)
      return model_curvature

    valid, raw_correction = self._raw_correction(
      model_v2,
      v_ego,
      float(np.clip(offset, -t["max_offset"], t["max_offset"])),
      float(np.clip(e2e_authority, 0.0, 1.0)),
      t,
    )
    if not valid:
      # Couldn't compute a trustworthy correction this tick (low confidence, bad geometry, stale
      # arrays, etc.) — smoothly release toward zero rather than either holding the last correction
      # forever or snapping it off.
      self._correction = float(smooth_value(0.0, self._correction, t["confidence_release_tau"], dt=DT_CTRL))
      self._finish_status("lowconf", v_ego)
      return model_curvature + self._correction

    # Clamp the raw (pre-gain) correction to the hard safety ceiling, THEN apply the gain. Doing the
    # clamp before the gain means max_gain can never be used to smuggle a bigger raw signal through —
    # the two multipliers are independent hard stops, not one combined budget.
    target = float(np.clip(raw_correction, -t["max_raw_correction"], t["max_raw_correction"])) * t["max_gain"]
    self._correction = float(smooth_value(target, self._correction, t["smooth_tau"], dt=DT_CTRL))
    # Final backstop: the correction is provably finite here given the sanitized tuning and the
    # finite-geometry gates above, but this is a lateral-adjacent command — never let a non-finite
    # value reach clip_curvature/the actuator. If it somehow is, drop the correction entirely.
    if not math.isfinite(self._correction):
      self._correction = 0.0
      self._finish_status("err", v_ego)
      return model_curvature
    self._finish_status("ok", v_ego)
    return model_curvature + self._correction

  @staticmethod
  def _valid_path(x, y) -> bool:
    """A path/line is usable only if x/y are equal-length, finite, and x is strictly increasing
    (so np.interp behaves and "lookahead distance" is unambiguous)."""
    return x.size >= 2 and x.size == y.size and np.isfinite(x).all() and np.isfinite(y).all() and np.all(np.diff(x) > 0)

  @staticmethod
  def _covers(x, distance: float) -> bool:
    """Whether `distance` falls within the range the array `x` actually reports (no extrapolation)."""
    return bool(x[0] <= distance <= x[-1])

  def _raw_correction(self, model_v2, v_ego: float, offset: float, e2e_authority: float, t: dict) -> tuple[bool, float]:
    """
    Compute the raw (pre-clamp, pre-gain) curvature correction from the model's lane lines and path.

    Returns (valid, raw_correction). `valid=False` means "don't trust this tick's geometry" — the
    caller releases the correction smoothly rather than using a garbage value. Every failure mode
    (missing arrays, low confidence, geometry out of a sane lane-shape range, lookahead distance not
    actually covered by the reported arrays) returns (False, 0.0) rather than raising, and the whole
    method is wrapped in a catch-all so a single malformed model_v2 field can never propagate an
    exception into the 100 Hz control loop.
    """
    try:
      lane_lines = model_v2.laneLines
      probs = np.asarray(model_v2.laneLineProbs, dtype=float)
      stds = np.asarray(model_v2.laneLineStds, dtype=float)
      if len(lane_lines) < 3 or probs.size < 3 or stds.size < 3:
        return False, 0.0

      # Telemetry only (see class docstring): record the confidence values the gates immediately
      # below are about to check, BEFORE those gates run — so a "lowconf" tick in the CES log still
      # shows the p1/p2/s1/s2 that caused the gate to fire, not just that it fired. Wrapped so this
      # can never influence, or itself throw into, the control decision that follows.
      try:
        self.status["p1"] = _tele_float(probs[1])
        self.status["p2"] = _tele_float(probs[2])
        self.status["s1"] = _tele_float(stds[1])
        self.status["s2"] = _tele_float(stds[2])
      except Exception:
        pass

      # Indices 1 and 2 are the ego lane's immediate left/right lines in openpilot's laneLines
      # convention (0=far-left .. 3=far-right of a 4-line report).
      if not np.isfinite(probs[[1, 2]]).all() or not np.isfinite(stds[[1, 2]]).all():
        return False, 0.0
      if np.any(probs[[1, 2]] < t["min_lane_prob"]) or np.any(probs[[1, 2]] > 1.0):
        return False, 0.0
      if np.any(stds[[1, 2]] < 0.0) or np.any(stds[[1, 2]] > t["max_lane_std"]):
        return False, 0.0

      left_x = np.asarray(lane_lines[1].x, dtype=float)
      left_y = np.asarray(lane_lines[1].y, dtype=float)
      right_x = np.asarray(lane_lines[2].x, dtype=float)
      right_y = np.asarray(lane_lines[2].y, dtype=float)
      pos_x = np.asarray(model_v2.position.x, dtype=float)
      pos_y = np.asarray(model_v2.position.y, dtype=float)
      if not (self._valid_path(left_x, left_y) and self._valid_path(right_x, right_y) and self._valid_path(pos_x, pos_y)):
        return False, 0.0

      # Lookahead grows with speed (look further ahead when moving faster), bounded to the tuned
      # [lookahead_min, lookahead_max] band.
      lookahead = float(np.clip(v_ego, t["lookahead_min"], t["lookahead_max"]))
      if not all(self._covers(x, lookahead) for x in (left_x, right_x, pos_x)):
        return False, 0.0

      left = float(np.interp(lookahead, left_x, left_y))
      right = float(np.interp(lookahead, right_x, right_y))
      width = right - left
      # Telemetry only: record the apparent width even if the range check below rejects it — that's
      # the useful diagnostic case (e.g. seeing width collapse to ~0 at a merge/exit ramp).
      try:
        self.status["w"] = _tele_float(width)
      except Exception:
        pass
      if not t["min_lane_width"] <= width <= t["max_lane_width"]:
        # Apparent lane width outside a plausible range -> the "lane lines" probably aren't a real
        # lane (merge, exit ramp, adjacent-lane line briefly tracked as ours, etc.) -> don't correct.
        return False, 0.0

      # The offset is further tightened so the target point never gets closer than
      # min_center_to_line to either physical line, no matter how large `offset`/max_offset are.
      max_safe_offset = min(t["max_offset"], max(0.0, width * 0.5 - t["min_center_to_line"]))
      target_y = 0.5 * (left + right) + float(np.clip(offset, -max_safe_offset, max_safe_offset))
      model_y = float(np.interp(lookahead, pos_x, pos_y))
      error = target_y - model_y
      error_abs = abs(error)
      if error_abs <= t["deadband"]:
        error = 0.0
      else:
        # Subtract the deadband from the magnitude (not just clip to 0) so the correction ramps up
        # smoothly starting from the deadband edge instead of jumping the instant it's crossed.
        error = np.copysign(error_abs - t["deadband"], error)

      # E2E-authority blend: if the model's own end-to-end path is confident (low yStd) and it
      # disagrees with the lane-line-derived target by more than a little, let the model's path win
      # (partially or fully) instead of fighting it — the E2E path may be avoiding something the
      # simple lane-line midpoint doesn't know about (e.g. a parked car hugging one line).
      try:
        pos_y_std = np.asarray(model_v2.position.yStd, dtype=float)
        if self._valid_path(pos_x, pos_y_std):
          path_std = float(np.interp(lookahead, pos_x, pos_y_std))
          # Telemetry only: record regardless of whether the E2E blend below actually engages.
          try:
            self.status["yStd"] = _tele_float(path_std)
          except Exception:
            pass
          if 0.0 <= path_std <= t["e2e_max_path_std"]:
            break_in = np.clip(
              (error_abs - t["e2e_break_in_start"]) / (t["e2e_break_in_full"] - t["e2e_break_in_start"]),
              0.0,
              1.0,
            )
            error *= 1.0 - e2e_authority * float(break_in)
      except (AttributeError, TypeError, ValueError):
        # No/invalid yStd -> skip the E2E blend, keep the lane-line-only error. Not a failure.
        pass

      # Telemetry only: the final (post-deadband, post-E2E-blend) center error that the curvature
      # conversion below is about to use.
      try:
        self.status["err"] = _tele_float(error)
      except Exception:
        pass

      # Small-angle lookahead geometry: for a small lateral error at distance `lookahead`, the
      # curvature needed to close it over that distance is approximately 2*error/lookahead**2
      # (standard "pure pursuit"-style relation for a short correction, not a full path replan).
      return True, float(2.0 * error / lookahead ** 2)
    except (AttributeError, IndexError, TypeError, ValueError):
      return False, 0.0
