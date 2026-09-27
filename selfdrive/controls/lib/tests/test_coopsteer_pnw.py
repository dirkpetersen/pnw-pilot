"""
coopsteer-shadow2pnw tests. Three layers, each with a stated mutation that MUST make it fail:

  1. the pure module's bounds (12 deg cap, override, deadzone, washout, jerk-tied slew, sign symmetry,
     bad input, worst-case boundedness under fuzz);
  2. the capability gate (Raven on, Ford/unknown/None off, and for_vehicle() returning None);
  3. telemetry ARRIVAL: the exact dict controlsd publishes is pushed through the REAL
     ces_pnw._read_map and the REAL ces_pnw._event_record / _steer_log_step, and the cp* values are
     asserted on the record that would be appended to ces_events.jsonl. This is the layer that the
     vtscstatus-telemetry-not-logged failure (a mem-param field that ces_pnw never cherry-picked)
     would have failed at.
"""
import inspect
import json
import math
import random
import re
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls import controlsd as ctl
from openpilot.selfdrive.controls.controlsd import Controls
from openpilot.selfdrive.controls.lib import coopsteer_pnw as cs
from openpilot.selfdrive.controls.lib.coopsteer_pnw import CoopSteer, telemetry_fields, COOP_TELEMETRY_KEYS
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw import park_tick_gate
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C

DT = 0.01
MPH = 0.44704
RAVEN = "TESLA_MODEL_S_HW3"
LIGHTNING = "FORD_F_150_LIGHTNING_MK1"


class FakeCP:
  def __init__(self, fp="", brand="", op_long=False):
    self.carFingerprint = fp
    self.brand = brand
    self.openpilotLongitudinalControl = op_long
    self.dashcamOnly = False


@pytest.fixture
def unfiltered(monkeypatch):
  """Turn the coopsteer2pnw torque low-pass off (tau 0 -> alpha 1) for tests that pin single-tick
    arithmetic of the ratio/slew/override logic. Must be requested BEFORE the instance is built (the
    filter coefficient is fixed at construction). The filter itself is tested with the shipped value."""
  monkeypatch.setattr(cs, "COOP_TQ_TAU_S", 0.0)


def _run(shadow, torque, v, n, pressed=False, active=True, cmd=0.0):
  res = None
  for _ in range(n):
    res = shadow.update(active, pressed, torque, v, cmd)
  return res


# =================================================================================================
# 1. pure module bounds
# =================================================================================================
class TestFlatCap:
  def test_12_deg_cap_binds_at_low_speed_with_full_torque(self):
    """Mutation: COOP_ABS_MAX_DEG = 64.8 (Penduras' pre-fix low-speed defect) -> offset ~ 190 deg at 10 mph."""
    s = CoopSteer(DT)
    r = _run(s, 1.0, 10 * MPH, 500)           # 5 s at 10 mph: speed-scaled cap would be ~190 deg
    assert r.reason == cs.REASON_ACTIVE
    assert r.cap_deg == pytest.approx(12.0)
    assert 0 < r.offset_deg <= 12.0

  def test_cap_never_exceeded_even_by_direct_state_corruption(self):
    """Belt-and-braces clamp on the held offset. Mutation: delete the final _clamp(self._offset, ...)."""
    s = CoopSteer(DT)
    s._offset = 40.0                          # corrupt the held state
    r = s.update(True, False, 0.9, 5.0, 0.0)
    assert abs(r.offset_deg) <= 12.0

  def test_lat_accel_cap_binds_above_45_mph(self):
    """Above ~45 mph the 1.5 m/s^2 bound is smaller than 12 deg (FEASIBILITY s3 table; linear bicycle
    here). Mutation: COOP_MAX_LAT_ACCEL = 3.6 -> cap at 70 mph ~ 9.4 deg, not ~3.9."""
    s = CoopSteer(DT)
    cap70 = s.cap_deg(70 * MPH)
    assert cap70 < 12.0
    assert cap70 == pytest.approx(cs.linear_bicycle_deg(1.5 / (70 * MPH) ** 2, 70 * MPH), abs=1e-6)
    assert cap70 == pytest.approx(3.9, abs=0.2)

  def test_cap_arithmetic_matches_the_docstring(self):
    """DOC PIN, not a behaviour test (Fable should-fix 7): re-derives the docstring's physics from the
    module's geometry constants so the comment cannot drift from them silently. It fails on a
    constant mutation (RAVEN_STEER_RATIO = 12 -> R ~ 170 m), not on a logic mutation."""
    road_rad = math.radians(12.0 / cs.RAVEN_STEER_RATIO)
    R = cs.RAVEN_WHEELBASE_M / math.tan(road_rad)
    assert R == pytest.approx(212, abs=2)
    assert (3 * MPH) ** 2 / R == pytest.approx(0.0085, abs=0.001)
    assert 1.2 < (40 * MPH) ** 2 / R < 1.6


class TestOverrideAndDeadzone:
  def test_override_against_the_held_offset_sheds_it(self):
    """steeringPressed with torque OPPOSING the held offset zeroes the target and sheds the offset.
    Mutation: drop the override branch (the -1.5 would read as an active full-cap target the other way)."""
    s = CoopSteer(DT)
    _run(s, 0.9, 20 * MPH, 300)               # build up an offset first
    assert s.offset_deg > 1.0
    r = _run(s, -1.5, 20 * MPH, 400, pressed=True)
    assert r.reason == cs.REASON_OVERRIDE
    assert r.target_deg == 0.0
    assert r.offset_deg == 0.0

  def test_override_with_the_held_offset_holds_it_never_pulls_back(self):
    """coopsteer2pnw: on the Raven lateral stays ACTIVE from 1 Nm up to the EPS's own hands-on abort,
    so shedding a held offset while the driver pushes the SAME way drives the wheel back against his
    hands (the base module did that for 35.7 deg in total over the 09-26 morning replay). The offset
    must be frozen -- not shed, and not grown past what the light-touch band earned. Both signs, both
    override sources (debounced flag, raw torque). Mutation: delete the HOLD branch -> REASON_OVERRIDE
    and the offset falls every tick."""
    for sign in (1.0, -1.0):
      for pressed, tq in ((True, 1.5), (False, 2.5), (True, 0.8)):
        s = CoopSteer(DT)
        _run(s, sign * 0.9, 20 * MPH, 300)
        held = s.offset_deg
        assert abs(held) > 1.0
        for _ in range(300):
          r = s.update(True, pressed, sign * tq, 20 * MPH, 0.0)
          assert r.reason == cs.REASON_HOLD, (sign, pressed, tq, r)
          assert r.offset_deg == held
        # ...and it is released gently once the driver releases the wheel (no snap, no wrong-way undershoot)
        prev = abs(s.offset_deg)
        for _ in range(300):
          r = s.update(True, False, 0.0, 20 * MPH, 0.0)
          assert 0.0 <= abs(r.offset_deg) <= prev and r.offset_deg * sign >= 0.0
          prev = abs(r.offset_deg)
        assert r.offset_deg == 0.0

  def test_hold_still_respects_the_speed_cap(self):
    """A HOLD earned at low speed (12 deg flat cap) must not outlive a lower cap as speed rises:
    the per-tick |offset| <= cap(v) invariant covers the HOLD branch too. Mutation: drop the clamp in
    the HOLD branch."""
    s = CoopSteer(DT)
    _run(s, 1.0, 10 * MPH, 400)
    assert s.offset_deg > 5.0
    r = s.update(True, True, 1.5, 70 * MPH, 0.0)
    assert r.reason == cs.REASON_HOLD
    assert r.offset_deg == pytest.approx(s.cap_deg(70 * MPH)) and r.offset_deg < 5.0

  def test_debounced_pressed_still_overrides_below_full_torque(self):
    """coopsteerfix2pnw: the debounced flag must keep its own authority. Its hysteresis holds it True
    for a few frames after the torque dips back under 1.0 Nm; those frames must stay override, not
    flip to a 0.9 Nm (near-full) active nudge. Mutation: `override = <torque test only>` (drop
    `bool(steering_pressed)`) -> reason active, target ~ 0.86 cap."""
    s = CoopSteer(DT)
    r = s.update(True, True, 0.9, 20 * MPH, 0.0)
    assert r.reason == cs.REASON_OVERRIDE and r.target_deg == 0.0 and r.offset_deg == 0.0

  def test_zero_torque_is_exactly_zero(self):
    """Mutation: COOP_DEADZONE_NM = -0.1 -> zero torque produces a non-zero ratio... it does not,
    copysign(0)=0 -- so the real mutation is `dz == 0.0` -> `dz == 0.01`, which mis-labels the
    reason. Both checked."""
    s = CoopSteer(DT)
    r = _run(s, 0.0, 30 * MPH, 200)
    assert r.offset_deg == 0.0 and r.target_deg == 0.0
    assert r.reason == cs.REASON_DEADZONE

  def test_inside_deadzone_is_exactly_zero(self):
    """0.49 Nm held (just under the 0.5 deadzone) must produce exactly 0 -- including the 0.3-0.5 band
    Penduras' 0.3 used to act on, which the 09-26 drive shows is the resting hand dragging against the
    EPS (84 % against the wheel's motion), not intent. Mutation: COOP_DEADZONE_NM = 0.3."""
    s = CoopSteer(DT)
    r = _run(s, 0.49, 30 * MPH, 300)
    assert r.offset_deg == 0.0 and r.reason == cs.REASON_DEADZONE
    r = _run(s, -0.49, 30 * MPH, 300)
    assert r.offset_deg == 0.0 and r.reason == cs.REASON_DEADZONE

  def test_ratio_is_proportional_between_deadzone_and_full(self, unfiltered):
    """0.75 Nm is the midpoint of 0.5-1.0 -> target = cap/2. Mutation: COOP_FULL_NM = 2.0."""
    s = CoopSteer(DT)
    r = s.update(True, False, 0.75, 20 * MPH, 0.0)
    assert r.target_deg == pytest.approx(0.5 * r.cap_deg, rel=1e-6)

  def test_inactive_resets_state(self):
    """Mutation: remove self.reset() from the not-active branch."""
    s = CoopSteer(DT)
    _run(s, 0.9, 20 * MPH, 300)
    assert s.offset_deg > 1.0
    r = s.update(False, False, 0.9, 20 * MPH, 5.0)
    assert r.reason == cs.REASON_INACTIVE and r.offset_deg == 0.0 and s._lp == 0.0


class TestTorqueOverrideBeforeDebounce:
  """coopsteerfix2pnw -- the 2026-09-07 100 Hz replay defect. Tesla's steeringPressed needs 6
  consecutive frames of |tq| > 1.0 Nm, so for ~50 ms after the driver crosses 1.0 Nm the flag reads
  False. The module must call that override from the torque itself, not emit the saturated 12 deg
  cap in the window (77 of 1585 active ticks on the drive, up to 2.98 Nm)."""

  def test_2nm_without_pressed_is_override_not_saturated_active(self):
    """THE defect. Mutation: `elif override:` -> `elif steering_pressed:` (the pre-fix branch) ->
    reason active, target = the full 12 deg cap."""
    for tq in (2.0, -2.0):
      s = CoopSteer(DT)
      r = s.update(True, False, tq, 10 * MPH, 0.0)
      assert r.reason == cs.REASON_OVERRIDE, (tq, r)
      assert r.target_deg == 0.0 and r.offset_deg == 0.0

  def test_exact_boundary_is_strict_greater_than_1nm(self, unfiltered):
    """1.0 Nm exactly is still the top of the nudge band (ratio 1 -> target == cap), matching the
    carstate's own strict `abs(torque) > STEER_THRESHOLD`; the next representable float above it is
    override. Literal 1.0 on purpose, not COOP_FULL_NM. Mutations: `>` -> `>=` (1.0 becomes
    override); `abs(...)` dropped (-1.0000000000000002 stays active)."""
    above = math.nextafter(1.0, math.inf)
    for sign in (1.0, -1.0):
      s = CoopSteer(DT)
      r = s.update(True, False, sign * 1.0, 20 * MPH, 0.0)
      assert r.reason == cs.REASON_ACTIVE, (sign, r)
      assert r.target_deg == pytest.approx(sign * r.cap_deg, rel=1e-9)
      s = CoopSteer(DT)
      r = s.update(True, False, sign * above, 20 * MPH, 0.0)
      assert r.reason == cs.REASON_OVERRIDE, (sign, r)
      assert r.target_deg == 0.0

  def test_real_tesla_debounce_window_never_reads_active_above_1nm(self):
    """End to end against the REAL opendbc debouncer (CarStateBase.update_steering_pressed with the
    Tesla carstate's arguments: > STEER_THRESHOLD, min count 5): a driver ramping from a held 0.9 Nm
    push to 2.98 Nm, and a fresh 2.98 Nm grab with no offset held. On every tick where |tq| > 1.0 the
    module must be in override (HOLD of what the light band already earned, or a zero target) -- never
    'active', never growing -- and the window where steeringPressed is still False must actually
    exist (else the test is vacuous). Mutation: the pre-fix branch (`elif steering_pressed:`) ->
    active ticks with |tq| > 1.0 and the offset climbing toward the 12 deg cap."""
    from opendbc.car.interfaces import CarStateBase
    from opendbc.car.tesla.values import STEER_THRESHOLD
    for lead_in in (300, 0):
      deb = type("Deb", (), {"steering_pressed_cnt": 0})()
      s = CoopSteer(DT)
      trace = [0.9] * lead_in + [1.2, 1.6, 2.1, 2.6] + [2.98] * 10
      window, held = 0, None
      for tq in trace:
        pressed = CarStateBase.update_steering_pressed(deb, abs(tq) > STEER_THRESHOLD, 5)
        before = s.offset_deg
        r = s.update(True, pressed, tq, 4.6, 0.0)
        if abs(tq) > 1.0:
          window += 0 if pressed else 1
          held = before if held is None else held
          assert r.reason in (cs.REASON_OVERRIDE, cs.REASON_HOLD), (tq, pressed, r)
          assert abs(r.offset_deg) <= abs(held) + 1e-12, (tq, pressed, r)
          if held == 0.0:
            assert r.reason == cs.REASON_OVERRIDE and r.target_deg == 0.0 and r.offset_deg == 0.0
      assert window >= 5, f"debounce window not exercised ({window} unpressed ticks above 1 Nm)"

  def test_torque_override_sheds_held_offset_at_the_full_jerk_rate(self):
    """A torque-derived override AGAINST the held offset is the same override: the offset sheds at the
    full jerk budget (Fable should-fix 3), not the gentle release rate. Mutation: slew `or override`
    -> `or bool(steering_pressed)` -> sheds at half rate in the debounce window."""
    s = CoopSteer(DT)
    _run(s, 0.9, 30.0, 400)
    before = s.offset_deg
    assert before > 0.5
    r = s.update(True, False, -2.0, 30.0, 0.0)
    assert r.reason == cs.REASON_OVERRIDE
    assert before - r.offset_deg == pytest.approx(1.0 * s.jerk_rate_deg_s(30.0) * DT, rel=1e-6)

  def test_nonfinite_torque_is_still_bad_input_at_the_release_rate(self):
    """inf torque is a broken input, not a 'huge push': reason badInput and the held offset decays at
    the pre-fix (half) rate, i.e. the badInput path is unchanged by this fix. Mutation: drop the
    `_finite(torque_nm) and` guard in the override test -> abs(inf) > 1 -> full-rate shed."""
    s = CoopSteer(DT)
    _run(s, 0.9, 30.0, 400)
    before = s.offset_deg
    assert before > 0.5
    r = s.update(True, False, float("inf"), 30.0, 0.0)
    assert r.reason == cs.REASON_BAD_INPUT
    assert before - r.offset_deg == pytest.approx(0.5 * s.jerk_rate_deg_s(30.0) * DT, rel=1e-6)


class TestWashout:
  def test_sustained_input_decays(self):
    """A held 0.9 Nm must NOT hold its offset: ~3 tau later (16 s) it is under 5 % of its 0.5 s peak.
    Mutation: COOP_WASHOUT_TAU_S = 1e9 (i.e. no washout) -> offset stays at the cap."""
    s = CoopSteer(DT)
    peak = _run(s, 0.9, 20 * MPH, 50).offset_deg
    assert peak > 1.0
    late = _run(s, 0.9, 20 * MPH, 1600).offset_deg
    assert late < 0.05 * peak

  def test_tau_is_five_seconds(self):
    """Pins the constant through behaviour, not by reading it back: after exactly one tau of a
    held input the RAW washed target is e^-1 of the target (37 %). Mutation: COOP_WASHOUT_TAU_S=2."""
    s = CoopSteer(DT)
    r = _run(s, 1.0, 20 * MPH, int(5.0 / DT))
    assert s._lp == pytest.approx(r.target_deg * (1 - math.exp(-1)), rel=0.02)

  def test_release_never_undershoots_the_other_way(self):
    """THE sign-clamp: on release the plain high-pass would go negative (a wrong-way nudge). Mutation:
    delete the sign-clamp line -> offset goes negative after release."""
    s = CoopSteer(DT)
    _run(s, 0.9, 20 * MPH, 300)
    for _ in range(600):
      r = s.update(True, False, 0.0, 20 * MPH, 0.0)
      assert r.offset_deg >= 0.0, "washout undershoot: offset flipped sign on release"
    assert r.offset_deg == 0.0

  def test_washout_only_ever_reduces_magnitude(self):
    """|washed target| <= |raw target| on every tick of a random torque trace, both signs.
    Mutation: same as above."""
    s = CoopSteer(DT)
    rng = random.Random(7)
    for _ in range(3000):
      tq = rng.uniform(-1.0, 1.0)
      r = s.update(True, False, tq, 15.0, 0.0)
      assert abs(r.washed_deg) <= abs(r.target_deg) + 1e-12
      assert r.washed_deg == 0.0 or (r.washed_deg * r.target_deg) > 0.0

  def test_lowpass_state_is_bounded_no_windup(self):
    """The washout memory is a filtered copy of a +-12-bounded signal. Mutation: feed the low-pass
    the un-clipped ratio (e.g. `self._lp += (tq * 100 - self._lp) * ...`)."""
    s = CoopSteer(DT)
    for _ in range(5000):
      # coopsteerfix2pnw: was an absurd 50 Nm, which is now an override (target 0) and would make this
      # test vacuous. 1.0 Nm is the largest torque that still drives the target to the full cap.
      s.update(True, False, 1.0, 5.0, 0.0)
    assert s._lp == pytest.approx(12.0, abs=1e-3)   # it DID wind up to the cap (the test is live)...
    assert abs(s._lp) <= 12.0 + 1e-9                 # ...and no further

  def test_reversal_is_not_attenuated_by_stale_memory(self, unfiltered):
    """A fresh torque reversal must be felt at full raw strength (the clamp caps washed at the raw
    target, never above, and stale opposite-sign memory must not shrink it either)."""
    s = CoopSteer(DT)
    _run(s, 0.9, 20 * MPH, 200)
    r = s.update(True, False, -0.9, 20 * MPH, 0.0)
    assert r.washed_deg == pytest.approx(r.target_deg)


class TestSlewTiedToJerk:
  def test_same_direction_slew_is_half_the_jerk_rate_at_highway_speed(self, unfiltered):
    """At 30 m/s the jerk-limited rate is ~10 deg/s (linear bicycle) -> same-dir slew 5 deg/s -> first
    tick moves exactly 0.05 deg. Mutation: COOP_SLEW_FRAC_SAME = 1.0, or a fixed 30 deg/s constant."""
    s = CoopSteer(DT)
    jr = s.jerk_rate_deg_s(30.0)
    assert jr == pytest.approx(cs.linear_bicycle_deg(3.6 / 900.0, 30.0))
    r = s.update(True, False, 1.0, 30.0, 0.0)
    assert r.offset_deg == pytest.approx(0.5 * jr * DT, rel=1e-6)
    assert r.offset_deg < 30.0 * DT          # i.e. strictly slower than Penduras' constant

  def test_opposing_torque_unwinds_at_the_full_jerk_rate(self, unfiltered):
    """Mutation: COOP_SLEW_FRAC_OPPOSING = 0.5."""
    s = CoopSteer(DT)
    _run(s, 0.9, 30.0, 400)
    before = s.offset_deg
    assert before > 0.5
    r = s.update(True, False, -0.9, 30.0, 0.0)
    assert before - r.offset_deg == pytest.approx(1.0 * s.jerk_rate_deg_s(30.0) * DT, rel=1e-6)

  def test_low_speed_slew_is_capped_at_60_deg_s(self, unfiltered):
    """At 10 mph the jerk rate is hundreds of deg/s; the comfort ceiling binds. Mutation:
    COOP_SLEW_MAX_DEG_S = 250 (MAX_ANGLE_RATE) -> first tick moves ~0.94 deg, not 0.6."""
    s = CoopSteer(DT)
    assert 0.5 * s.jerk_rate_deg_s(10 * MPH) > 60.0
    r = s.update(True, False, 1.0, 10 * MPH, 0.0)
    assert r.offset_deg == pytest.approx(60.0 * DT, rel=1e-6)

  def test_jerk_reference_matches_opendbc_tesla_limit(self):
    """COOP_JERK_REF_MS3 must track the binding tesla ANGLE_LIMITS.MAX_LATERAL_JERK (3.0 + g*0.06).
    Mutation: COOP_JERK_REF_MS3 = 5.0 (ISO_LATERAL_JERK, the wrong constant)."""
    from opendbc.car.tesla.values import CarControllerParams
    assert cs.COOP_JERK_REF_MS3 == pytest.approx(CarControllerParams.ANGLE_LIMITS.MAX_LATERAL_JERK, abs=0.05)


class TestWrongSignWorstCase:
  def test_sign_symmetry(self):
    """+tq and -tq produce mirror offsets. Mutation: `dz = max(0, |tq| - dz)` without copysign."""
    a, b = CoopSteer(DT), CoopSteer(DT)
    ra = _run(a, 0.8, 20 * MPH, 100)
    rb = _run(b, -0.8, 20 * MPH, 100)
    assert ra.offset_deg == pytest.approx(-rb.offset_deg) and ra.offset_deg > 0

  def test_fuzz_offset_bounded_by_cap_everywhere(self):
    """Whatever the sign convention turns out to be, |offset| <= min(12, cap(v)) on every tick of a
    random trace (torque, speed, pressed, active all random). Mutation: any of the caps, or deleting
    the per-tick `_clamp(self._offset, -cap, cap)` (a speed step lets a low-speed offset outlive its
    cap -- this test found that gap in the first draft)."""
    s = CoopSteer(DT)
    rng = random.Random(42)
    ticks = 0
    while ticks < 60000:
      # piecewise-constant inputs held for 0.5-3 s: a per-tick re-roll never lets the offset build up,
      # which let the 64.8 deg cap mutation survive the first draft of this test.
      hold = rng.randint(50, 300)
      v = rng.choice([0.0, 0.5, 2.0, 5.0, 13.4, 20.0, 30.0, 40.0])
      tq = rng.uniform(-3.0, 3.0)
      active, pressed = rng.random() > 0.05, rng.random() < 0.1
      for _ in range(hold):
        r = s.update(active, pressed, tq, v, rng.uniform(-90, 90))
        assert abs(r.offset_deg) <= 12.0 + 1e-9
        assert abs(r.offset_deg) <= s.cap_deg(v) + 1e-9 or r.reason == cs.REASON_INACTIVE
        assert math.isfinite(r.would_cmd_deg)
      ticks += hold

  def test_nonfinite_cap_from_injected_model_is_refused_and_reset(self):
    """Fable must-fix 1: a NaN/inf from the injected deg_for_curvature must not latch NaN into the
    state with reason "active". Expect badInput, offset exactly 0, state reset, finite+JSON-safe
    telemetry, and full recovery once the model is sane again. Mutation: delete the
    `if not _finite(cap)` guard."""
    bad = {"nan": False}
    def deg(k, v):
      return float("nan") if bad["nan"] else cs.linear_bicycle_deg(k, v)
    s = CoopSteer(DT, deg_for_curvature=deg)
    _run(s, 0.9, 20 * MPH, 100)
    assert s.offset_deg > 0.5
    bad["nan"] = True
    r = s.update(True, False, 0.9, 20 * MPH, 1.0)
    assert r.reason == cs.REASON_BAD_INPUT and r.offset_deg == 0.0 and r.cap_deg == 0.0 and s._lp == 0.0
    frag = telemetry_fields(r, 0.9, 1.0)
    json.dumps(frag, allow_nan=False)
    for _ in range(50):
      r = s.update(True, False, 0.9, 20 * MPH, 1.0)
      assert math.isfinite(r.offset_deg) and r.offset_deg == 0.0
    bad["nan"] = False
    r = _run(s, 0.9, 20 * MPH, 100)
    assert r.reason == cs.REASON_ACTIVE and r.offset_deg > 0.5
    # telemetry_fields is the LAST line of defence and must hold on its own: a hand-built result with
    # NaN in every numeric slot (unreachable through update() while the cap guard exists) still
    # serialises. Mutation: drop the _finite guard on cpOff/cpTgt/cpCap.
    nan = float("nan")
    frag = telemetry_fields(cs.CoopSteerResult(nan, nan, nan, nan, cs.REASON_ACTIVE, nan), nan, nan)
    json.dumps(frag, allow_nan=False)
    assert frag["cpOff"] is None and frag["cpTgt"] is None and frag["cpCap"] is None

  def test_nonfinite_jerk_rate_holds_instead_of_jumping(self, unfiltered):
    """Fable note: the model can be sane for the cap curvature (1.5/v^2) yet NaN for the larger jerk
    curvature (3.6/v^2). Without the rate guard the offset would take a one-tick jump to `washed`
    (bounded, but a discontinuity). With it the offset holds. Mutation: delete `rate = 0.0`."""
    v = 20 * MPH
    def deg(k, vv):
      return float("nan") if k > 2.0 / (v * v) else cs.linear_bicycle_deg(k, vv)
    s = CoopSteer(DT, deg_for_curvature=deg)
    r = s.update(True, False, 1.0, v, 0.0)
    assert r.reason == cs.REASON_ACTIVE and r.cap_deg == pytest.approx(12.0)
    assert r.offset_deg == 0.0                     # held, not jumped to the 12 deg washed target

  def test_override_unwinds_at_the_full_jerk_rate(self, unfiltered):
    """Fable should-fix 3: a held offset the driver pushes AGAINST (steeringPressed) must shed at the
    full jerk budget, not the gentle half rate. Mutation: drop `or override` in the slew."""
    s = CoopSteer(DT)
    _run(s, 0.9, 30.0, 400)
    before = s.offset_deg
    assert before > 0.5
    r = s.update(True, True, -1.5, 30.0, 0.0)
    assert r.reason == cs.REASON_OVERRIDE
    assert before - r.offset_deg == pytest.approx(1.0 * s.jerk_rate_deg_s(30.0) * DT, rel=1e-6)
    # ...while a plain release (not pressed) keeps the gentle half rate
    s2 = CoopSteer(DT)
    _run(s2, 0.9, 30.0, 400)
    b2 = s2.offset_deg
    r2 = s2.update(True, False, 0.0, 30.0, 0.0)
    assert b2 - r2.offset_deg == pytest.approx(0.5 * s2.jerk_rate_deg_s(30.0) * DT, rel=1e-6)

  def test_bad_input_is_named_not_guessed(self):
    """NaN torque -> reason badInput, target 0, no crash. Mutation: replace the _finite guard with
    `torque_nm = torque_nm or 0.0` (silently treats NaN as no torque)."""
    s = CoopSteer(DT)
    r = s.update(True, False, float("nan"), 20.0, 0.0)
    assert r.reason == cs.REASON_BAD_INPUT and r.target_deg == 0.0 and math.isfinite(r.offset_deg)
    r = s.update(True, False, 0.9, float("inf"), 0.0)
    assert r.reason == cs.REASON_BAD_INPUT

  def test_nan_command_never_reaches_the_json_row(self):
    """Gemini must-fix: NaN steeringAngleDeg -> would_cmd_deg NaN -> json.dumps writes a bare NaN token
    (invalid JSON) into ces_events. Every cp* value must survive json.dumps(allow_nan=False).
    Mutation: drop the _finite guard on cpCmd in telemetry_fields."""
    s = CoopSteer(DT)
    for cmd, tq, v in ((float("nan"), 0.9, 20.0), (5.0, float("nan"), 20.0), (5.0, 0.9, float("inf"))):
      r = s.update(True, False, tq, v, cmd)
      assert r.reason == cs.REASON_BAD_INPUT
      frag = telemetry_fields(r, tq, float("nan"))
      json.dumps(frag, allow_nan=False)            # raises ValueError on any NaN/inf
      assert frag["cpWhy"] == cs.REASON_BAD_INPUT
    assert telemetry_fields(s.update(True, False, 0.9, 20.0, float("nan")), 0.9, 1.0)["cpCmd"] is None

  def test_would_cmd_is_cmd_plus_offset_and_nothing_else(self):
    s = CoopSteer(DT)
    r = _run(s, 0.9, 20 * MPH, 50, cmd=-7.25)
    assert r.would_cmd_deg == pytest.approx(-7.25 + r.offset_deg)

  def test_dt_must_be_sane(self):
    for bad in (0.0, -0.01, float("nan"), None):
      with pytest.raises((ValueError, TypeError)):
        CoopSteer(bad)


# =================================================================================================
# 1b. coopsteer2pnw: the sign on REAL frames, and the torque filter
# =================================================================================================
# Random samples (seeded, not hand-picked) of the Raven's own carState on 2026-09-26, manual driving
# (latActive False, vEgo > 5 m/s). drives/2026-09-26/coopsteer-calibration/frames.py made them; the
# populations they come from agree 99.5 % / 99.8 % (primary), 98.0 / 98.7 % (turning-in) and 93-95 %
# (onsets), so a sample of 44 all agreeing is what those rates predict, not a curated result.
# (steeringTorque Nm, steeringAngleDeg, steeringRateDeg, vEgo) -- |tq| >= 0.5, |angle| >= 15 deg
SIGN_0926_PRIMARY = (
  (1.93, 33.5, -6.5, 12.27),
  (1.85, 55.9, -38.5, 8.99),
  (1.62, 29.0, 11.5, 8.7),
  (1.9, 32.4, -21.0, 13.69),
  (1.54, 27.0, -6.5, 9.96),
  (2.2, 107.7, -39.0, 7.04),
  (-1.7, -103.8, 91.5, 5.6),
  (-1.69, -56.5, 44.0, 9.8),
  (-1.69, -105.7, 90.0, 5.57),
  (-3.04, -32.6, -56.0, 11.79),
  (-2.56, -58.0, -24.0, 8.32),
  (-1.17, -55.1, 20.0, 6.3),
  (1.15, 15.5, -0.5, 14.17),
  (1.06, 15.6, -0.5, 6.58),
  (1.8, 22.8, -5.5, 17.27),
  (1.24, 15.5, -0.0, 14.18),
  (1.48, 20.5, -0.5, 14.32),
  (2.72, 25.6, 10.5, 17.08),
  (-1.87, -25.3, 1.0, 12.91),
  (-2.22, -28.5, -0.0, 12.96),
  (-2.15, -25.5, -3.0, 12.9),
  (-2.18, -23.6, -3.5, 13.41),
  (-1.45, -17.2, 0.5, 10.9),
  (-2.01, -22.5, -1.5, 13.35),
)
# the same, turning IN (rate has the angle's sign): the wheel is moving the way the driver pushes
SIGN_0926_TURN_IN = (
  (2.74, 96.3, 122.5, 5.49),
  (2.97, 29.0, 40.0, 15.45),
  (0.74, 1.9, 12.0, 29.44),
  (1.88, 14.0, 49.0, 13.03),
  (1.57, 10.4, 13.0, 10.97),
  (-1.47, -5.5, -10.0, 13.96),
  (-2.03, -23.3, -18.5, 10.81),
  (1.74, 12.5, 23.0, 14.7),
  (1.89, 12.3, 12.0, 17.65),
  (-1.74, -17.1, -18.5, 14.08),
  (2.35, 23.1, 11.0, 15.77),
  (1.69, 9.7, 12.5, 17.65),
)
# torque onsets from near-straight (|angle| < 5): (torque every 30 ms for 300 ms, angle at onset,
# angle 300 ms later, vEgo) -- the causal witness: push first, the wheel follows the same way
SIGN_0926_ONSETS = (
  ([0.54, 0.61, 0.59, 0.4, 0.71, 0.59, 0.81, 0.65, 0.66, 0.78, 1.0], 1.5, 2.2, 12.26),
  ([-0.52, -0.58, -0.46, -0.62, -0.66, -0.78, -0.99, -0.95, -1.15, -1.07, -1.01], 2.1, 1.4, 29.25),
  ([0.53, 0.53, 0.59, 0.6, 0.66, 0.7, 0.63, 0.76, 0.83, 0.85, 0.86], 0.4, 1.4, 10.07),
  ([-0.51, -0.47, -0.45, -0.43, -0.42, -0.42, -0.56, -0.56, -0.56, -0.57, -0.52], 1.4, 0.9, 13.17),
  ([-0.7, -0.39, -0.4, -0.46, -0.55, -0.59, -0.68, -0.91, -0.92, -0.84, -1.29], -1.7, -2.6, 14.93),
  ([0.59, 0.41, 0.58, 0.63, 0.68, 0.47, 0.49, 0.44, 0.56, 0.62, 0.58], 1.7, 2.6, 13.12),
  ([0.61, 0.57, 0.61, 0.59, 0.74, 0.79, 0.8, 1.18, 1.22, 1.19, 1.43], 2.5, 3.6, 14.77),
  ([-0.55, -0.41, -0.93, -0.73, -0.66, -0.72, -0.57, -0.47, -0.6, -0.7, -0.42], 2.6, -0.1, 7.0),
)


class TestSignOnRealFrames:
  """THE sign question, settled on the car's own data rather than inferred from code. +torque goes with
  +angle, so the module's offset (same sign as torque) must steer TOWARD where the driver is turning.
  Each witness is checked on the frames themselves AND fed through the module. Mutation: invert the
  target (`target, reason = -ratio * cap, ...`) -> every frame fails; or flip the deadzone copysign."""

  @staticmethod
  def _offset_for(tq_sign, v):
    s = CoopSteer(DT)
    return _run(s, 0.9 * tq_sign, v, 50).offset_deg

  def test_primary_witness_torque_with_angle(self):
    assert all(math.copysign(1, tq) == math.copysign(1, angle) for tq, angle, _, _ in SIGN_0926_PRIMARY)
    for tq, angle, _, v in SIGN_0926_PRIMARY:
      off = self._offset_for(math.copysign(1, tq), v)
      assert off != 0.0 and math.copysign(1, off) == math.copysign(1, angle), (tq, angle, v, off)

  def test_rate_witness_torque_with_wheel_motion_when_turning_in(self):
    assert all(math.copysign(1, tq) == math.copysign(1, rate) for tq, _, rate, _ in SIGN_0926_TURN_IN)
    for tq, _, rate, v in SIGN_0926_TURN_IN:
      off = self._offset_for(math.copysign(1, tq), v)
      assert off != 0.0 and math.copysign(1, off) == math.copysign(1, rate), (tq, rate, v, off)

  def test_causal_witness_the_wheel_follows_the_push(self):
    for tqs, a0, a1, v in SIGN_0926_ONSETS:
      assert all(math.copysign(1, t) == math.copysign(1, tqs[0]) for t in tqs)
      assert math.copysign(1, a1 - a0) == math.copysign(1, tqs[0])
      off = self._offset_for(math.copysign(1, tqs[0]), v)
      assert off != 0.0 and math.copysign(1, off) == math.copysign(1, a1 - a0), (tqs[0], a0, a1, off)

  def test_both_signs_are_present(self):
    """A fixture of one sign only would pass a module that ignores the torque's sign."""
    for frames in (SIGN_0926_PRIMARY, SIGN_0926_TURN_IN):
      assert {math.copysign(1, f[0]) for f in frames} == {1.0, -1.0}


class TestTorqueFilter:
  """coopsteer2pnw: COOP_TQ_TAU_S = 0.1 s on the torque that feeds the target. 09-26: 18,665 separate
  excursions into 0.3-1.0 Nm while engaged, 99.6 % under 100 ms."""

  def test_a_30ms_flick_does_not_nudge(self):
    """Mutation: COOP_TQ_TAU_S = 0 -> the flick reaches ratio ~0.9 and moves the wheel."""
    s = CoopSteer(DT)
    for tq in [0.95] * 3 + [0.1] * 100:
      r = s.update(True, False, tq, 10.0, 0.0)
      assert r.offset_deg == 0.0, r

  def test_a_real_push_still_nudges_within_300ms(self):
    """The filter must not swallow intent: a held 0.8 Nm push moves the wheel within 0.3 s at 20 mph.
    Mutation: COOP_TQ_TAU_S = 5.0 -> nothing within 0.3 s."""
    s = CoopSteer(DT)
    r = _run(s, 0.8, 20 * MPH, 30)
    assert r.reason == cs.REASON_ACTIVE and r.offset_deg > 0.5

  def test_filter_never_delays_the_override(self):
    """The override test reads the RAW torque: a 2 Nm grab against a held offset sheds it on the very
    first tick. Mutation: test `abs(self._tq_f) > COOP_FULL_NM` for the override."""
    s = CoopSteer(DT)
    _run(s, 0.9, 20 * MPH, 100)
    r = s.update(True, False, -2.0, 20 * MPH, 0.0)
    assert r.reason == cs.REASON_OVERRIDE and r.target_deg == 0.0

  def test_filter_state_is_clipped_to_the_band(self):
    """A long 3 Nm override must not leave the filter at 3 Nm (it would linger as a full-cap target after
    release). Mutation: drop the _clamp on the filter input."""
    s = CoopSteer(DT)
    _run(s, 3.0, 20 * MPH, 500)
    assert abs(s._tq_f) <= cs.COOP_FULL_NM + 1e-12


# =================================================================================================
# 2. capability gate
# =================================================================================================
class TestCapabilityGate:
  def test_raven_has_coop_steer(self):
    assert PnwVehicle(FakeCP(RAVEN, "tesla", op_long=True)).coop_steer is True

  def test_ford_has_no_coop_steer_and_no_instance(self):
    """THE Ford gate. Mutation: `self.coop_steer = brand in ("tesla", "ford")` or
    for_vehicle() returning cls(...) unconditionally."""
    v = PnwVehicle(FakeCP(LIGHTNING, "ford", op_long=False))
    assert v.coop_steer is False
    assert CoopSteer.for_vehicle(v, DT) is None

  def test_other_tesla_class_and_unknown_and_none_are_off(self):
    """Gated to the ONE car the sign will be confirmed on; brand-level gating is the mutation."""
    assert PnwVehicle(FakeCP("TESLA_MODEL_3", "tesla", op_long=True)).coop_steer is False
    assert PnwVehicle(FakeCP("SOME_OTHER_CAR", "hyundai")).coop_steer is False
    assert PnwVehicle(None).coop_steer is False
    assert CoopSteer.for_vehicle(PnwVehicle(None), DT) is None
    assert CoopSteer.for_vehicle(object(), DT) is None

  def test_ford_telemetry_is_all_none(self):
    """No instance -> telemetry_fields(None): every key present, every value None (distinct from a
    Raven zero, whose cpWhy carries a reason). Mutation: `return {}` for the None case."""
    frag = telemetry_fields(None)
    assert set(frag) == set(COOP_TELEMETRY_KEYS)
    assert all(v is None for v in frag.values())

  def test_raven_instance_uses_injected_vehicle_model(self):
    calls = []
    def deg(k, v):
      calls.append((k, v))
      return 100.0
    s = CoopSteer.for_vehicle(PnwVehicle(FakeCP(RAVEN, "tesla")), DT, deg_for_curvature=deg)
    assert s is not None and s.cap_deg(20.0) == 12.0 and calls


# =================================================================================================
# 3. telemetry ARRIVAL -- the real ces_pnw read + record paths, driven by the real publish dict
# =================================================================================================
class _MemParams:
  """Stub for the /dev/shm Params handle: returns the SteerLimitStatus dict controlsd would publish
  and None for every other key, exactly like the real store before those keys' first publish."""
  def __init__(self, sl):
    self.sl = sl
  def get(self, key, return_default=False):
    return self.sl if key == "SteerLimitStatus" else None


def _ces_cls():
  return next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_event_record"))


class _Permissive:
  def __getattr__(self, n):
    return None


def _stub_after_read_map(sl_dict):
  """Run the REAL _read_map against a stub whose mem_params returns `sl_dict` for SteerLimitStatus."""
  g = _Permissive()
  g.mem_params = _MemParams(sl_dict)
  g._toggles = {}
  g._bearing_hist = deque(maxlen=30)
  g._map_targets = []
  g._enabled = True
  _ces_cls()._read_map.__get__(g)()
  return g


def _publish_dict(res, tq, rate):
  """The controlsd side: the sl dict already carries other keys; the cp* fragment is merged in."""
  d = {"angErr": 1.0, "latActive": True}
  d.update(telemetry_fields(res, tq, rate))
  return d


class TestTelemetryArrival:
  def test_cp_fields_survive_read_map_cherry_pick(self):
    """Mutation: delete the cp* block in ces_pnw._read_map -> attributes stay None."""
    s = CoopSteer(DT)
    res = _run(s, 0.7, 20 * MPH, 100, cmd=3.5)
    g = _stub_after_read_map(_publish_dict(res, 0.7, -4.2))
    assert g._cp_off == pytest.approx(res.offset_deg, abs=1e-3) and g._cp_off > 0
    assert g._cp_tgt == pytest.approx(res.target_deg, abs=1e-3)
    assert g._cp_cap == pytest.approx(12.0)
    assert g._cp_why == cs.REASON_ACTIVE
    assert g._cp_tq == pytest.approx(0.7) and g._cp_rate == pytest.approx(-4.2)
    assert g._cp_cmd == pytest.approx(3.5 + res.offset_deg, abs=1e-3)

  def test_cp_fields_reach_the_enabled_tick_record(self):
    """Real _event_record("tick"). Mutation: delete the cp* lines from the enabled record."""
    s = CoopSteer(DT)
    res = _run(s, -0.8, 20 * MPH, 100)
    g = _stub_after_read_map(_publish_dict(res, -0.8, 2.0))
    g._vtsc_tele = {}
    g._sa_tele = {}
    g._speed_limit = 11.2
    g._button = C.BTN_CES
    g._ces2_urg = 0.0
    g._ces2_div = type("D", (), {"count": 0})()
    g._gl = type("G", (), {"state": None, "status": lambda s: None})()
    g._sm = type("S", (), {"state": None, "status": lambda s: None})()
    g._icbm_floor_lim = 0.0
    g._icbm_k = 0.0
    g._icbm_k_dist = 0.0
    g._icbm_k_v = 0.0
    g._icbm_k_n = 0
    g._icbm_k_ahead = False
    # icbmconsist2pnw added these to CES __init__ (and to the enabled record, as float()/int()); _Permissive predates
    # them and answered None, so the record raised TypeError before ever reaching the cp* fields under test.
    g._icbm_k_at = 0.0
    g._icbm_k_at_d = 0.0
    g._icbm_k_at_n = 0
    g._icbm_k_at_gap = 0.0
    # parkgate2pnw added "gear"/"park" to the enabled record; `park` comes off a real ParkTickGate
    # (the _Permissive stub's None would raise before reaching the cp* fields under test).
    g._gear_name = "drive"
    g._park_gate = park_tick_gate.ParkTickGate()
    rec = _ces_cls()._event_record.__get__(g)("tick", {"vEgo": 11.0})
    assert rec["cpOff"] == pytest.approx(res.offset_deg, abs=1e-3) and rec["cpOff"] < 0
    assert rec["cpWhy"] == cs.REASON_ACTIVE and rec["cpTq"] == pytest.approx(-0.8)
    assert rec["cpRate"] == pytest.approx(2.0)
    for k in COOP_TELEMETRY_KEYS:
      assert k in rec

  def test_cp_fields_reach_the_ces_off_steer_breadcrumb(self):
    """Real _steer_log_step (the CES-off path -- the Raven drives with CES off, so this is THE record
    the sign question will be read from). Mutation: delete the cp* lines from that record."""
    s = CoopSteer(DT)
    res = _run(s, 0.55, 25.0, 100)
    g = _stub_after_read_map(_publish_dict(res, 0.55, 1.1))
    g._enabled = False
    g._steer_tick_last = -1e9
    g._mode = 0
    g._car = "TESLA_MODEL_S_HW3"
    g._cur_lat = g._cur_lon = g._cur_bearing = None
    g._speed_limit = 0.0
    g._vtsc_tele = {}
    g._sa_tele = {}
    captured = []
    g._append_event = captured.append
    g._read_map = lambda: None          # already run above; the real one would re-read the same stub
    # parkgate2pnw: _steer_log_step now asks the park gate first, via the controller's _park_decision
    # wrapper. Seeded with no gear at all -> the FAIL-OPEN case, so the breadcrumb is written exactly
    # as before and every cp* assertion below is unaffected.
    g._gear = g._gear_name = g._v_ego_raw = None
    g._park_gate = park_tick_gate.ParkTickGate()
    g._park_gate_on = True
    g._park_decision = _ces_cls()._park_decision.__get__(g)
    car_state = type("CS", (), {"vEgo": 25.0})()
    sm = {"radarState": type("R", (), {"leadOne": type("L", (), {"status": False})()})()}
    _ces_cls()._steer_log_step.__get__(g)(car_state, sm)
    assert len(captured) == 1, "breadcrumb not appended"
    rec = captured[0]
    assert rec["ev"] == "steer"
    assert rec["cpOff"] == pytest.approx(res.offset_deg, abs=1e-3) and rec["cpOff"] > 0
    assert rec["cpTgt"] == pytest.approx(res.target_deg, abs=1e-3)
    assert rec["cpWhy"] == cs.REASON_ACTIVE
    assert rec["cpTq"] == pytest.approx(0.55) and rec["cpRate"] == pytest.approx(1.1)

  def test_ford_row_is_none_not_zero(self):
    """A car without the capability publishes the all-None fragment, and None must survive as None
    (a 0.0 would read as 'the Raven module computed zero')."""
    g = _stub_after_read_map({"angErr": 0.0, **telemetry_fields(None)})
    assert g._cp_off is None and g._cp_why is None and g._cp_tq is None

  def test_unpublished_fragment_reads_none(self):
    """Before controlsd's first publish (or if the merge were dropped) the keys are absent and read
    None -- on the Raven that is the 'silently evaporated' signature the drive log must show, not a 0."""
    g = _stub_after_read_map({"angErr": 0.0})
    assert g._cp_off is None and g._cp_why is None

  def test_controlsd_merges_the_fragment_before_publishing(self):
    """Structural pin on the publish side: the cp* merge (with the actuation fields) sits between the
    steer_limit_status literal and the put_nonblocking("SteerLimitStatus") call, and the module update
    runs AFTER LaC's angle is on the actuator. Mutation: delete the merge, or move the coop block above
    the actuators assignment."""
    src = (Path(m.__file__).resolve().parents[2] / "controlsd.py").read_text()
    publish = src.index('put_nonblocking("SteerLimitStatus"')
    literal = src.index("steer_limit_status = {")
    merge = src.index("coop_telemetry_fields(self._coop_res")
    assert literal < merge < publish
    assert "actuating=self._coop_enabled, applied_deg=self._coop_applied" in src[merge:publish]
    act = src.index("actuators.steeringAngleDeg = float(steeringAngleDeg)")
    upd = src.index("self._coop_res = self._coop_shadow.update(")
    assert act < upd

  def test_cp_act_and_cp_app_reach_the_record(self):
    """coopsteer2pnw: the two new fields go through the REAL _read_map and the REAL enabled-tick /
    CES-off records. Mutation: drop them from _read_map, or from either record."""
    s = CoopSteer(DT)
    res = _run(s, 0.8, 20 * MPH, 100)
    sl = {"angErr": 1.0, "latActive": True}
    sl.update(telemetry_fields(res, 0.8, 1.0, actuating=True, applied_deg=res.offset_deg))
    g = _stub_after_read_map(sl)
    assert g._cp_act is True and g._cp_app == pytest.approx(res.offset_deg, abs=1e-3) and g._cp_app > 0
    src = Path(m.__file__).read_text()
    assert src.count('"cpAct": self._cp_act, "cpApp": self._cp_app') == 2   # enabled tick + CES-off steer record
    off = telemetry_fields(res, 0.8, 1.0)                                    # toggle OFF: computed, not applied
    assert off["cpAct"] is False and off["cpApp"] == 0.0 and off["cpOff"] > 0
    assert telemetry_fields(None)["cpAct"] is None and telemetry_fields(None)["cpApp"] is None


# =================================================================================================
# 4. coopsteer2pnw: actuation in controlsd (the real methods, on a stub instance)
# =================================================================================================


def _ctrl(enabled, res, shadow=None):
  c = object.__new__(Controls)
  c._coop_enabled = enabled
  c._coop_res = res
  c._coop_applied = 0.0
  c._coop_shadow = shadow if shadow is not None else CoopSteer(DT)
  c._coop_frame = 0
  c._coop_param_err_logged = False
  return c


def _res(off):
  return cs.CoopSteerResult(off, off, off, 12.0, cs.REASON_ACTIVE, 3.0 + off)


class _Log:
  def __init__(self):
    self.calls = []
  def __getattr__(self, name):
    return lambda *a, **k: self.calls.append((name, a, k))


class TestActuation:
  LAC_ANGLE = 3.0000000000000004     # a value whose float bits would show any arithmetic done on it

  def _apply(self, c, lat_active=True):
    acts = SimpleNamespace(steeringAngleDeg=self.LAC_ANGLE)
    Controls._coop_apply(c, SimpleNamespace(latActive=lat_active), acts, self.LAC_ANGLE)
    return acts

  def test_toggle_off_leaves_the_actuator_bit_identical(self):
    """THE Tesla-with-toggle-OFF guarantee: LaC's angle goes to the wire untouched, the bits and the very
    object, whatever the module computed. Mutation: drop `self._coop_enabled and` from the gate."""
    c = _ctrl(False, _res(4.0))
    acts = self._apply(c)
    assert acts.steeringAngleDeg is self.LAC_ANGLE and c._coop_applied == 0.0

  def test_toggle_on_adds_exactly_the_offset(self):
    c = _ctrl(True, _res(-2.5))
    acts = self._apply(c)
    assert acts.steeringAngleDeg == self.LAC_ANGLE - 2.5 and c._coop_applied == -2.5

  def test_lateral_inactive_or_no_result_applies_nothing(self):
    """Mutation: drop `CC.latActive and` / `res is not None and`."""
    acts = self._apply(_ctrl(True, _res(4.0)), lat_active=False)
    assert acts.steeringAngleDeg is self.LAC_ANGLE
    c = _ctrl(True, None)                         # the module raised this tick (Rule 2 path)
    acts = self._apply(c)
    assert acts.steeringAngleDeg is self.LAC_ANGLE and c._coop_applied == 0.0

  def test_nonfinite_offset_is_logged_and_not_applied(self, monkeypatch):
    """Rule 2: say so, apply nothing. Mutation: drop the isfinite guard (NaN reaches the actuator), or
    the cloudlog.error line."""
    log = _Log()
    monkeypatch.setattr(ctl, "cloudlog", log)
    c = _ctrl(True, _res(float("nan")))
    acts = self._apply(c)
    assert acts.steeringAngleDeg is self.LAC_ANGLE and c._coop_applied == 0.0
    assert any(n == "error" and "coopsteer2pnw" in a[0] for n, a, _ in log.calls)


class _Params:
  def __init__(self, value=None, exc=None):
    self.value, self.exc, self.reads = value, exc, 0
  def get_bool(self, key):
    assert key == "CoopSteer"
    self.reads += 1
    if self.exc:
      raise self.exc
    return self.value


class TestToggleRead:
  def test_param_error_is_off_and_logged_once(self, monkeypatch):
    """Fail-safe OFF, said once (Rule 2). Mutation: `except: pass` keeping the last state, or no log."""
    log = _Log()
    monkeypatch.setattr(ctl, "cloudlog", log)
    c = _ctrl(True, None)
    c.params = _Params(exc=RuntimeError("store gone"))
    for _ in range(300):
      Controls._read_coop_enabled(c)
    assert c._coop_enabled is False
    assert sum(1 for n, _, _ in log.calls if n == "exception") == 1

  def test_read_at_1hz_and_a_flip_resets_the_module(self):
    """Mutation: drop the reset on a flip -> the offset the shadow was holding steps onto the wire."""
    shadow = CoopSteer(DT)
    _run(shadow, 0.9, 20 * MPH, 100)
    assert shadow.offset_deg > 0.5
    c = _ctrl(False, None, shadow=shadow)
    c.params = _Params(value=True)
    for _ in range(250):
      Controls._read_coop_enabled(c)
    assert c.params.reads == 3 and c._coop_enabled is True
    assert shadow.offset_deg == 0.0

  def test_default_is_off(self):
    """New toggle, default OFF (CLAUDE.md). Mutation: params_keys.h default "1"."""
    from openpilot.common.params import Params
    assert Params().get("CoopSteer", return_default=True) is False


class TestLightningUntouched:
  def test_the_whole_coop_path_is_behind_the_capability_instance(self):
    """On a car without PnwVehicle.coop_steer, _coop_shadow is None (TestCapabilityGate) and NOTHING of
    the coop path runs: the toggle read, the update and the apply all sit inside
    `if self._coop_shadow is not None:` in state_control, and steeringAngleDeg is assigned in exactly two
    places (LaC's line, and _coop_apply). Mutation: move `self._coop_apply(...)` out of the if-block."""
    import ast
    import inspect
    import textwrap
    tree = ast.parse(textwrap.dedent(inspect.getsource(Controls.state_control)))
    gated = set()
    for node in ast.walk(tree):
      if isinstance(node, ast.If) and "self._coop_shadow is not None" in ast.unparse(node.test):
        for sub in ast.walk(node):
          if isinstance(sub, ast.Call) and isinstance(sub.func, ast.Attribute):
            gated.add(sub.func.attr)
    assert {"_read_coop_enabled", "update", "_coop_apply"} <= gated
    for node in ast.walk(tree):
      if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and \
         node.func.attr in ("_read_coop_enabled", "_coop_apply"):
        assert any(node in list(ast.walk(n)) for n in ast.walk(tree)
                   if isinstance(n, ast.If) and "self._coop_shadow is not None" in ast.unparse(n.test))
    src = Path(ctl.__file__).read_text()
    assert len(re.findall(r"actuators\.steeringAngleDeg\s*=", src)) == 2

  def test_ford_controls_owns_no_instance(self):
    ford = PnwVehicle(FakeCP(LIGHTNING, "ford"))
    assert CoopSteer.for_vehicle(ford, DT) is None
