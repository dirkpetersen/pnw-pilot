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

from cereal import log  # tightfollow2pnw: LongitudinalPersonality enum for the aggressive-only check

# curveslow-lightning: mph<->m/s (no numpy — a plain float; numpy leaked into a capnp setter and
# crash-looped card, 2026-07-11).
_MPH_TO_MS = 0.44704
# tightfollow2pnw: Lightning-only Aggressive T_FOLLOW override (s), vs. the shared upstream 1.25s.
# Deliberately a MODEST first cut for a heavy work truck, not an attempt to match the stock ACC's
# tightest bar setting outright — validate on the road and retune from here.
_TIGHT_AGGRESSIVE_T_FOLLOW = 1.0

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
  # 90 mph silent-ICBM run). All Lightning-only via lightning_curve_slow; Tesla path returns 0.0 /
  # neutral values from every accessor below.
  "descent_gain": 8.0,          # per rad of downhill pitch: a 5% grade (~0.05 rad) -> +40% penalty.
                                #   Physics: on a descent gravity eats the regen decel budget, so the
                                #   truck arrives at/above the cap — enter the curve slower instead.
  "descent_pitch_cap": 0.12,    # rad (~12% grade); |pitch| beyond this adds no more (IMU-noise bound)
  "penalty_cap_mph": 15.0,      # hard cap on the TOTAL penalty after all multipliers (never more)
  "left_factor": 1.15,          # extra multiplier on LEFT curves only: US road crown drains right, so
                                #   a left curve banks ADVERSELY (negative superelevation) — the same
                                #   curvature needs more lateral grip + more EPS torque, and the
                                #   Lightning's weak EPS washes out of lefts first (both 2026-07-11
                                #   washouts were downhill LEFTS).
  "overspeed_margin_mph": 2.0,  # VTSC: v_ego above the applied cap by this -> friction-brake escalation
  "map_scale": 0.92,            # ICBM: scale mapd's suggested speeds DOWN before the binding test —
                                #   OSM curve speeds (mapV 99-112 mph on the I-90 sweepers) are
                                #   calibrated for stronger-steering cars; Tesla-appropriate, too
                                #   generous for the Lightning.
  "icbm_firm_decel": 1.4,       # m/s^2 assumed approach decel for LARGE speed drops (stock ACC does
                                #   the actual braking; this only shapes the tap-start envelope)
  # standstillsoft2pnw (2026-07-14): gentle standstill LAUNCH accel ramp — the red-light follow-launch
  # "lurch" fix. Cap the accel out of a dead stop to launch_accel, ramping to the normal envelope by
  # launch_v. (Root cause: a lead crept forward at a red, op-long launched to follow at ~2.0 m/s^2.)
  "launch_accel": 0.6,          # m/s^2 accel ceiling at a dead stop (soft; stock jumped to ~2.0 = lurch)
  "launch_v_mph": 9.0,          # by this speed the launch cap has lifted to the normal accel envelope
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
  # launch_accel in [0.2, 2.0]: never so low the truck can't move, never above the stock ~2.0 max ->
  # this cap can only ever SOFTEN a launch, never make it harsher. launch_v [3, 25] mph.
  "launch_accel": (0.2, 2.0),
  "launch_v_mph": (3.0, 25.0),
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
  (runs in control code) — missing/malformed -> the hardcoded 3/5 mph defaults."""
  cfg = dict(_RAIN_DEFAULTS)
  try:
    st = os.stat(RAIN_CONFIG_PATH)
    if not stat.S_ISREG(st.st_mode) or st.st_size > _RAIN_CONFIG_MAX_BYTES:
      return cfg
    with open(RAIN_CONFIG_PATH) as f:
      data = json.load(f)
    if isinstance(data, dict):
      for k in cfg:
        if k in data:
          v = float(data[k])
          if v == v:                          # NaN guard (NaN != NaN)
            cfg[k] = v
  except Exception:
    return dict(_RAIN_DEFAULTS)                # any failure -> defaults, never raise
  for k, (lo, hi) in _RAIN_BOUNDS.items():
    cfg[k] = _clamp(cfg[k], lo, hi)
  return cfg


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
    self.tight_aggressive_follow: bool = False
    # read the tunable ramp ONCE at construction (defensive; defaults when absent = the intended ramp)
    self._curve_cfg = _load_curve_config()

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
      is_left: LEFT curve -> pen *= left_factor (adverse US road crown + the weak EPS; see the
        left_factor default comment).
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
    # descentcurve2pnw: left-curve factor (adverse crown + weak EPS) — factor is clamped >= 1.0
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

  def aggressive_t_follow(self, personality) -> float | None:
    """tightfollow2pnw: a tighter T_FOLLOW (s) for the Lightning's Aggressive personality only —
    None (unchanged upstream behavior) for every other car and every other personality, including
    the Tesla's own Aggressive. The caller passes this straight through as long_mpc's optional
    t_follow_override; None there means "use the shared get_T_FOLLOW(personality) as before"."""
    if not self.tight_aggressive_follow:
      return None
    if personality != log.LongitudinalPersonality.aggressive:
      return None
    return _TIGHT_AGGRESSIVE_T_FOLLOW
