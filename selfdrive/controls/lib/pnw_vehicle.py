"""
pnw_vehicle — the PNW fleet's capability view over CarParams.

ONE place that knows which cars support which pnw features. Feature code asks about CAPABILITIES
(veh.ces_shadow, veh.stock_acc_buttons, ...), never about fingerprints — adding a car means adding
it here, not hunting string comparisons across the tree (driver directive 2026-07-11).

Pure and defensive: works with a capnp CarParams reader, the structs dataclass, or None (returns
all-False capabilities), so UI code can call it before a car is fingerprinted.
"""
import json
import os
import stat

# curveslow-lightning: mph<->m/s (no numpy — a plain float; numpy leaked into a capnp setter and
# crash-looped card, 2026-07-11).
_MPH_TO_MS = 0.44704

# Persistent, outside-git tunable for the Lightning curve-speed penalty (survives auto-update, same
# discipline as dm_config.py on the dm-variable branch). Missing/malformed -> hardcoded defaults.
# Schema: {"lightning": {"penalty_min_mph": 1.5, "penalty_max_mph": 10, "low_v_mph": 30, "high_v_mph": 65}}
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
}
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
}


def _clamp(x: float, lo: float, hi: float) -> float:
  return lo if x < lo else (hi if x > hi else x)


def _load_curve_config() -> dict:
  """Read /data/pnw/curve.json defensively (os.stat gate: regular file, <= 64 KiB), overlaying only
  known numeric keys onto the defaults, clamping each to sane bounds. NEVER raises (runs in control
  code) — any missing/bad/malformed input -> the hardcoded default ramp."""
  cfg = dict(_CURVE_DEFAULTS)
  try:
    st = os.stat(CURVE_CONFIG_PATH)
    if not stat.S_ISREG(st.st_mode) or st.st_size > _CURVE_CONFIG_MAX_BYTES:
      return cfg
    with open(CURVE_CONFIG_PATH) as f:
      data = json.load(f)
    light = data.get("lightning", {}) if isinstance(data, dict) else {}
    if isinstance(light, dict):
      for k in cfg:
        if k in light:
          v = float(light[k])
          if v == v:                      # NaN guard (NaN != NaN)
            cfg[k] = v
  except Exception:
    return dict(_CURVE_DEFAULTS)          # any failure -> defaults, never raise
  for k, (lo, hi) in _CURVE_BOUNDS.items():
    cfg[k] = _clamp(cfg[k], lo, hi)
  return cfg


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

    # stock-ACC set-speed steering via SET +/- button taps on the SCCM stream (ICBM executor lives
    # in the ford carcontroller; 0x083 is TX-allowlisted). Today: the 2025 F-150 Lightning.
    self.stock_acc_buttons: bool = fp == "FORD_F_150_LIGHTNING_MK1"

    # CES runs in SHADOW (decisions/telemetry/overlay, planner never actuates) with ICBM as the
    # actuator — exactly when the car has ACC buttons to steer and openpilot does NOT own long.
    self.ces_shadow: bool = self.stock_acc_buttons and not self.op_long

    # CES can act on this car at all (planner via op-long, or ICBM via buttons)
    self.ces_capable: bool = self.op_long or self.ces_shadow

    # nudgeless (blinker-hold) lane change support — BSM-gated in DesireHelper
    self.nudgeless: bool = brand == "tesla" or fp == "FORD_F_150_LIGHTNING_MK1"

    # curveslow-lightning: the Lightning's EPS is physically weaker than the Tesla's and washes out of
    # curves the Tesla holds, so it must enter curves SLOWER. This is a steering-authority FACT, not a
    # config choice, so it applies in BOTH op-long (VTSC) and stock-ACC (ICBM) — both produce a curve
    # target SPEED that curve_speed_penalty_ms() lowers. Tesla (and every other car) -> False -> 0.0.
    self.lightning_curve_slow: bool = fp == "FORD_F_150_LIGHTNING_MK1"
    # read the tunable ramp ONCE at construction (defensive; defaults when absent = the intended ramp)
    self._curve_cfg = _load_curve_config()

  def curve_speed_penalty_ms(self, v_target_ms: float) -> float:
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
    return max(0.0, pen_mph * _MPH_TO_MS)
