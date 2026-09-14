"""
coopsteer_pnw -- Penduras "cooperative steering" torque nudge, ported for the Tesla Raven in
SHADOW-LOG MODE ONLY.

WHAT THIS IS. On angle-controlled cars the EPAS servos toward the commanded angle, so any driver
torque BELOW the steeringPressed threshold (1.0 Nm on Tesla, opendbc/car/tesla/values.py
STEER_THRESHOLD) does nothing at all: a light corrective push is either ignored or has to be
escalated into a full override. Penduras (github.com/Penduras/openpilot, latcontrol_angle.py,
commits 482d6ed28 / 1422c80e3 / 3b5f67c42) converts that sub-threshold torque into a small,
bounded angle OFFSET added to the model's commanded angle. This module is the pure, unit-testable
brain of that idea.

WHAT THIS IS NOT (this round). Nothing here reaches the actuators. controlsd computes the offset
this module WOULD apply and publishes it to telemetry (SteerLimitStatus -> ces_events "cp*" fields)
so a real drive can settle two things the source never did:
  1. THE SIGN. Penduras' own commit 482d6ed28 says the steeringTorque-vs-steeringAngleDeg sign
     convention was "inference from code, not confirmed on the road". docs/TESLA-MADS-FEASIBILITY.md
     s3 records that their "~7 h / 27 drives" validation claim is not in their repo (the commits
     show ~88 min of rlog REPLAY). A wrong sign is a wrong-way 12 deg nudge. So: log first.
  2. THE FEEL CONSTANTS (deadzone, washout tau, slew) -- tune from our own log, not theirs.

PURITY CONTRACT. No I/O, no cereal, no Params, no imports from the control stack. Every bound
lives in this file so it can be tested without a car. The only injected dependency is
`deg_for_curvature(curvature_1pm, v_ego) -> steering-wheel deg`, so the call site can hand in the
real VehicleModel (whose understeer term matters above ~40 mph) while tests use the linear bicycle
default below. Both are pure functions.

BOUNDS (and why each number):

* COOP_DEADZONE_NM = 0.3, COOP_FULL_NM = 1.0  [carried from Penduras]
  Torsion-bar torque under 0.3 Nm is wheel weight / bias noise; 1.0 Nm is where the Tesla
  carstate's steeringPressed (full override) takes over, so the nudge reaches its own maximum
  exactly where the override begins. The nudge lives ONLY in the 0.3-1.0 Nm dead band under the
  override threshold. Torque above COOP_FULL_NM zeroes it on the same tick, as does steeringPressed
  (which is debounced 50 ms and so latches later); see `update`.

* COOP_ABS_MAX_DEG = 12.0 (flat, speed-independent)  [carried from Penduras 3b5f67c42]
  Physics check (TESLA-MADS-FEASIBILITY.md s3, Raven sR 15.0, wheelbase 2.96 m): 12 deg at the
  steering WHEEL is 12/15 = 0.8 deg at the road wheel, i.e. a turning radius R = 2.96 / tan(0.8 deg)
  ~= 212 m. Lateral accel v^2/R: 0.01 m/s^2 at 3 mph, 0.09 at 10 mph, 0.36 at 20, 0.77 at 30,
  ~1.3 at 40 mph. Over 10 m of parking-lot travel it is ~0.24 m of lateral displacement --
  genuinely light-touch. Without this flat cap the speed-scaled bound below reached -64.8 deg at
  <2 m/s in Penduras' replay (the low-speed defect PENDURAS-ANALYSIS.md notes); with it, the flat
  cap binds below ~45 mph and the lat-accel cap binds above, so the nudge never exceeds 12 deg
  anywhere and never exceeds ~1.5 m/s^2 anywhere.

* COOP_MAX_LAT_ACCEL = 1.5 m/s^2  [carried from Penduras]
  The speed-scaled half of the cap: max offset = angle that produces 1.5 m/s^2 at this speed
  (curvature = a/v^2). Deliberately well under the ~3.6 m/s^2 the panda enforces on the SUM of
  model + nudge -- it is a nudge, not a takeover. v is floored at COOP_V_FLOOR to keep the
  division finite; below that floor the flat 12 deg cap is what binds anyway.

* COOP_WASHOUT_TAU_S = 5.0  [NEW -- PENDURAS-ANALYSIS.md: "no washout on a sustained input"]
  A first-order high-pass (target minus its own low-pass) so a sustained push cannot hold an
  offset indefinitely: a 0.5 s tap keeps ~90 % of its effect, a 5 s lean is down to 37 %, a 15 s
  lean is under 5 % and the car is back on the model's line. The washed target is SIGN-CLAMPED
  to [0, target] (or [target, 0]) so the filter can only ever REDUCE the offset's magnitude: on
  release (target -> 0) the classic high-pass undershoot would otherwise push the OPPOSITE way,
  and on a torque reversal the filter must not add to the fresh input. The low-pass state is a
  filtered copy of a +-12 deg-bounded signal, so it is itself bounded -- no wind-up is possible.

* COOP_JERK_REF_MS3 = 3.6 m/s^3, COOP_SLEW_FRAC_SAME = 0.5, COOP_SLEW_FRAC_OPPOSING = 1.0,
  COOP_SLEW_MAX_DEG_S = 60  [REPLACES Penduras' 30 / 150 deg/s constants]
  Penduras rate-limited the offset at 30 deg/s (150 deg/s on reversal), constants chosen against
  Tesla's MAX_ANGLE_RATE (250 deg/s) -- but the BINDING limit on the wire is the lateral-JERK
  bound the carcontroller and panda apply to the whole command: max angle rate = angle for
  curvature-rate 3.6/v^2, i.e. ~15 deg/s at 30 m/s (VM), so their 150 deg/s could never actually
  happen at highway speed and 30 deg/s was already over the limit above ~40 mph. Here the offset's
  own slew is a FRACTION of that jerk-limited rate at the current speed: 0.5 when the driver
  pushes the same way the offset already points (leave half the jerk budget to the model's own
  path, so the nudge never forces the carcontroller to clip the model), 1.0 when the driver's
  torque OPPOSES the held offset (Penduras 1422c80e3's finding: a stale offset fighting a fresh
  reversal is the worst feel; spend the whole budget getting out of the way). COOP_JERK_REF_MS3
  mirrors opendbc/car/tesla/values.py ANGLE_LIMITS.MAX_LATERAL_JERK (3.0 + g*0.06 ~= 3.59) --
  not imported, to keep this module pure; test_coopsteer_pnw pins the two together. The 60 deg/s
  ceiling is for low-speed comfort only: the jerk-derived rate is hundreds of deg/s below 15 mph,
  and a 12 deg nudge arriving in 50 ms reads as a snap. 60 deg/s = the full cap in 0.2 s, about
  the rise time of the hand input itself.

WORST CASE WITH A WRONG SIGN. |offset| <= min(12 deg, angle for 1.5 m/s^2) on every tick, by
construction (target is clipped, washout only reduces, slew only interpolates toward it). Even
were this ever applied with the sign inverted, the request stays inside what the model itself may
command and inside every panda bound. That is the whole reason a shadow drive is cheap.
"""
import math
from collections.abc import Callable
from typing import NamedTuple

# --- torque band ---------------------------------------------------------------------------------
COOP_DEADZONE_NM = 0.3
COOP_FULL_NM = 1.0            # == Tesla STEER_THRESHOLD; steeringPressed (full override) starts here
# --- magnitude caps ------------------------------------------------------------------------------
COOP_ABS_MAX_DEG = 12.0
COOP_MAX_LAT_ACCEL = 1.5      # m/s^2
COOP_V_FLOOR = 1.0            # m/s, keeps a/v^2 finite; 12 deg cap binds long before this matters
# --- washout -------------------------------------------------------------------------------------
COOP_WASHOUT_TAU_S = 5.0
# --- slew (tied to the lateral-jerk bound, NOT to MAX_ANGLE_RATE) --------------------------------
COOP_JERK_REF_MS3 = 3.6       # mirrors tesla ANGLE_LIMITS.MAX_LATERAL_JERK; pinned by the unit test
COOP_SLEW_FRAC_SAME = 0.5
COOP_SLEW_FRAC_OPPOSING = 1.0
COOP_SLEW_MAX_DEG_S = 60.0
# --- default (test) steering geometry: Raven, opendbc/car/tesla/values.py TESLA_MODEL_S_HW3 -------
RAVEN_STEER_RATIO = 15.0
RAVEN_WHEELBASE_M = 2.96

# Reason codes. "active" is the ONLY code under which the offset can be non-zero; every other code
# means the module decided on zero and says why. None (absent) in telemetry means the car has no
# coop_steer capability at all -- see telemetry_fields().
REASON_INACTIVE = "inactive"    # lateral not active: state reset, offset 0
REASON_OVERRIDE = "override"    # steeringPressed (debounced) OR |torque| > 1 Nm this tick: the driver owns the wheel
REASON_DEADZONE = "deadzone"    # |torque| <= 0.3 Nm: nothing to respond to
REASON_BAD_INPUT = "badInput"   # NaN/inf on an input: refuse to compute rather than guess
REASON_ACTIVE = "active"


class CoopSteerResult(NamedTuple):
  offset_deg: float      # the offset that WOULD be added to the commanded angle this tick
  target_deg: float      # pre-washout, pre-slew target (what the raw torque asked for)
  washed_deg: float      # target after the sign-clamped washout (|washed| <= |target|, same sign or 0)
  cap_deg: float         # the magnitude bound in force this tick (min of flat and lat-accel caps)
  reason: str            # one of the REASON_* codes
  would_cmd_deg: float   # angle_cmd_deg + offset_deg -- what the wire WOULD have carried


def linear_bicycle_deg(curvature: float, v_ego: float,
                       steer_ratio: float = RAVEN_STEER_RATIO, wheelbase: float = RAVEN_WHEELBASE_M) -> float:
  """Steering-wheel degrees for a curvature under the small-angle kinematic bicycle model:
  road-wheel angle ~= wheelbase * curvature, times the steering ratio. Pure; used as the default
  when no VehicleModel is injected. Understeer-free, so it under-reads the real VM by ~15 % above
  40 mph (FEASIBILITY s3 table: 14.1 deg VM vs 11.9 deg here at 40 mph) -- conservative, and
  irrelevant below ~40 mph where the flat 12 deg cap binds. `v_ego` is accepted for signature
  parity with VehicleModel.get_steer_from_curvature and unused here."""
  return abs(math.degrees(wheelbase * curvature) * steer_ratio)


def _clamp(x: float, lo: float, hi: float) -> float:
  return lo if x < lo else hi if x > hi else x


class CoopSteerShadow:
  def __init__(self, dt: float, deg_for_curvature: Callable[[float, float], float] | None = None):
    if not (isinstance(dt, (int, float)) and math.isfinite(dt) and dt > 0):
      raise ValueError(f"coopsteer_pnw: dt must be a positive finite number, got {dt!r}")
    self.dt = float(dt)
    self._deg = deg_for_curvature if deg_for_curvature is not None else linear_bicycle_deg
    self._offset = 0.0     # held offset (deg), the slewed output
    self._lp = 0.0         # low-pass of the raw target (deg), the washout's memory

  @classmethod
  def for_vehicle(cls, veh, dt: float, deg_for_curvature=None):
    """THE capability gate. Returns None unless `veh.coop_steer` is truthy (PnwVehicle), so a car
    without the capability -- the Ford, which also runs LatControlAngle -- never even owns an
    instance and its telemetry carries the all-None fragment from telemetry_fields(None). Duck-typed
    on purpose: this module must not import pnw_vehicle (purity contract)."""
    if not bool(getattr(veh, "coop_steer", False)):
      return None
    return cls(dt, deg_for_curvature)

  def reset(self) -> None:
    self._offset = 0.0
    self._lp = 0.0

  @property
  def offset_deg(self) -> float:
    return self._offset

  def cap_deg(self, v_ego: float) -> float:
    """Magnitude bound at this speed: min(flat 12 deg, angle producing 1.5 m/s^2)."""
    v = max(float(v_ego), COOP_V_FLOOR)
    return min(abs(self._deg(COOP_MAX_LAT_ACCEL / (v * v), v)), COOP_ABS_MAX_DEG)

  def jerk_rate_deg_s(self, v_ego: float) -> float:
    """The binding lateral-jerk angle rate at this speed (what the carcontroller/panda enforce on
    the whole command): angle for a curvature-RATE of 3.6/v^2 per second."""
    v = max(float(v_ego), COOP_V_FLOOR)
    return abs(self._deg(COOP_JERK_REF_MS3 / (v * v), v))

  def update(self, lat_active: bool, steering_pressed: bool, torque_nm: float, v_ego: float,
             angle_cmd_deg: float = 0.0) -> CoopSteerResult:
    if not lat_active:
      self.reset()
      return CoopSteerResult(0.0, 0.0, 0.0, 0.0, REASON_INACTIVE, float(angle_cmd_deg) if _finite(angle_cmd_deg) else 0.0)

    # coopsteerfix2pnw: override is decided from the torque ITSELF as well as from steeringPressed.
    # Tesla's steeringPressed is debounced (update_steering_pressed(|tq| > 1.0, 5): 6 consecutive
    # frames), so for ~50 ms after the driver crosses 1.0 Nm it is still False. Without this the
    # module stayed "active" in that window and the ratio saturated at 1.0 -> the FULL 12 deg cap,
    # emitted exactly as the driver takes over (drive 2026-09-07: 77 of 1585 active ticks, up to
    # 2.98 Nm). Strict `>` matches the carstate's own `abs(torque) > STEER_THRESHOLD`. The debounced
    # flag is kept too: its hysteresis holds the override for a few frames after torque dips under
    # 1.0 Nm, which is the conservative side. Do NOT shorten the carstate debounce instead -- that
    # flag is shared with disengagement logic.
    override = bool(steering_pressed) or (_finite(torque_nm) and abs(float(torque_nm)) > COOP_FULL_NM)

    if not (_finite(torque_nm) and _finite(v_ego) and _finite(angle_cmd_deg)):
      # A NaN torque is a broken input, not "no torque": target zero (the held offset then decays at
      # the normal slew), and SAY SO via the reason code.
      v_eff = float(v_ego) if _finite(v_ego) else COOP_V_FLOOR
      target, cap, reason = 0.0, self.cap_deg(v_eff), REASON_BAD_INPUT
    elif override:
      target, cap, reason = 0.0, self.cap_deg(v_ego), REASON_OVERRIDE
      v_eff = float(v_ego)
    else:
      v_eff = float(v_ego)
      cap = self.cap_deg(v_eff)
      tq = float(torque_nm)
      dz = math.copysign(max(0.0, abs(tq) - COOP_DEADZONE_NM), tq)
      if dz == 0.0:
        target, reason = 0.0, REASON_DEADZONE
      else:
        ratio = _clamp(dz / (COOP_FULL_NM - COOP_DEADZONE_NM), -1.0, 1.0)
        target, reason = ratio * cap, REASON_ACTIVE

    # Fable review (2026-09-07) must-fix: the injected deg_for_curvature is the one thing this module
    # does not control. A NaN/inf cap would otherwise latch _lp/_offset at NaN for the rest of the
    # engagement with reason still "active" -- the headline "|offset| <= cap on every tick" would be
    # false and json would carry a bare NaN. Refuse, reset, and say so.
    if not _finite(cap):
      self.reset()
      target, cap, reason = 0.0, 0.0, REASON_BAD_INPUT

    # --- washout: high-pass the raw target, then sign-clamp so it can only shrink the magnitude ---
    self._lp += (target - self._lp) * min(1.0, self.dt / COOP_WASHOUT_TAU_S)
    washed = target - self._lp
    washed = _clamp(washed, 0.0, target) if target >= 0.0 else _clamp(washed, target, 0.0)

    # --- slew toward the washed target at a fraction of the jerk-limited rate ---------------------
    # Full jerk budget when the driver's fresh torque OPPOSES the held offset (Penduras 1422c80e3), and
    # ALSO during a full override (Fable should-fix 3): steeringPressed means the driver owns the wheel
    # and any held offset is by definition stale -- sheding it at half rate would be the exact "stale
    # offset fights the driver" case that finding was about. A plain release (torque -> 0, not
    # pressed) keeps the gentle half rate so the return to the model's line is not a snap.
    # coopsteerfix2pnw: a torque-derived override is the same override, so it sheds at the same rate.
    opposing = (target != 0.0 and self._offset != 0.0 and (target * self._offset) < 0.0) or override
    frac = COOP_SLEW_FRAC_OPPOSING if opposing else COOP_SLEW_FRAC_SAME
    rate = min(frac * self.jerk_rate_deg_s(v_eff), COOP_SLEW_MAX_DEG_S)
    if not _finite(rate):
      rate = 0.0    # same injected-callable failure as the cap guard above; hold rather than propagate NaN
    step = rate * self.dt
    self._offset = _clamp(washed, self._offset - step, self._offset + step)
    # Per-tick bound, independent of everything above: |offset| <= cap(v) <= 12 deg on THIS tick. The
    # slew alone would let an offset earned under a larger cap (lower speed) outlive that cap for a
    # few ticks as speed rises; cap(v) falls smoothly with v so this clamp is continuous in practice
    # (a speed STEP is not physical). This is the invariant a reviewer should be able to trust
    # without reading the rest of this method.
    self._offset = _clamp(self._offset, -cap, cap)

    return CoopSteerResult(self._offset, target, washed, cap, reason, float(angle_cmd_deg) + self._offset)


def _finite(x) -> bool:
  try:
    return math.isfinite(float(x))
  except (TypeError, ValueError):
    return False


# Telemetry fragment carried inside controlsd's SteerLimitStatus dict (-> ces_pnw._read_map ->
# ces_events). The KEY NAMES here are the single source of truth: controlsd publishes exactly this
# dict, and test_coopsteer_pnw drives the real ces_pnw._read_map with exactly this dict, so a key
# that evaporates on the way to ces_events (memory: vtscstatus-telemetry-not-logged) fails a test
# instead of a drive.
COOP_TELEMETRY_KEYS = ("cpOff", "cpTgt", "cpCap", "cpWhy", "cpTq", "cpRate", "cpCmd")


def telemetry_fields(res: CoopSteerResult | None, torque_nm=None, steering_rate_deg_s=None) -> dict:
  """cp* fragment for SteerLimitStatus. `res is None` == the car has no coop_steer capability:
  every key present, every value None, so a Ford row is distinguishable from a Tesla row whose
  module produced zero (cpWhy carries a reason string there)."""
  if res is None:
    return dict.fromkeys(COOP_TELEMETRY_KEYS)
  return {
    # Every value _finite-guarded (Fable must-fix 1): a bare NaN token is invalid JSON and would poison
    # the ces_events row; None + the reason code is the honest rendering of "not a number".
    "cpOff": round(res.offset_deg, 3) if _finite(res.offset_deg) else None,
    "cpTgt": round(res.target_deg, 3) if _finite(res.target_deg) else None,
    "cpCap": round(res.cap_deg, 2) if _finite(res.cap_deg) else None,
    "cpWhy": res.reason,
    "cpTq": round(float(torque_nm), 3) if _finite(torque_nm) else None,
    "cpRate": round(float(steering_rate_deg_s), 2) if _finite(steering_rate_deg_s) else None,
    # Gemini review (2026-09-07) must-fix: a NaN CS.steeringAngleDeg reaches the shadow BEFORE controlsd's
    # own "Ensure no NaNs" pass, so would_cmd_deg can be NaN on a badInput tick; json.dumps would then
    # write a bare `NaN` token (invalid JSON) into ces_events.jsonl. None here, with cpWhy="badInput"
    # saying why -- same guard cpTq/cpRate already have.
    "cpCmd": round(res.would_cmd_deg, 3) if _finite(res.would_cmd_deg) else None,
  }
