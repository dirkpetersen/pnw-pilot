"""
Tests for the Lane Centering correction rate-of-change cap (`correction_roc`).

Scope is deliberately narrow: these cover the per-tick growth limiter added in lcroc2pnw and the
safety-envelope guarantees around it. They do NOT attempt to re-test the whole StarPilot port.

The model messages are hand-built stubs, not real cereal messages -- LaneCenteringController only
ever reads attributes off `model_v2`, so a small duck-typed object exercises the exact same code
path without needing a msgq/cereal fixture.
"""

import json
import numpy as np
import pytest

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib import lane_centering as lc
from openpilot.selfdrive.controls.lib.lane_centering import DEFAULT_TUNING, LaneCenteringController


class _Line:
  def __init__(self, y: float, n: int = 40):
    self.x = np.arange(n, dtype=float)   # 0 .. n-1 m, strictly increasing
    self.y = np.full(n, y, dtype=float)


class _Position:
  def __init__(self, y: float, y_std: float, n: int = 40):
    self.x = np.arange(n, dtype=float)
    self.y = np.full(n, y, dtype=float)
    self.yStd = np.full(n, y_std, dtype=float)


class _Meta:
  laneChangeState = 0  # log.LaneChangeState.off


class _Model:
  """A modelV2 stub with confident, plausible lane lines placed symmetrically about `lane_center`."""

  def __init__(self, lane_center: float = 0.0, model_y: float = 0.0, width: float = 3.6,
               prob: float = 0.99, std: float = 0.05, path_std: float = 1.0):
    self.laneLines = [_Line(lane_center - width), _Line(lane_center - width / 2.0),
                      _Line(lane_center + width / 2.0), _Line(lane_center + width)]
    self.laneLineProbs = [prob] * 4
    self.laneLineStds = [std] * 4
    # path_std defaults ABOVE e2e_max_path_std (0.35) so the E2E veto stays out of these tests.
    self.position = _Position(model_y, path_std)
    self.meta = _Meta()


V_EGO = 20.0            # m/s -- above min_v_ego, inside the lookahead band
FULL = DEFAULT_TUNING["max_raw_correction"] * DEFAULT_TUNING["max_gain"]   # 0.0012 1/m
STEP = DEFAULT_TUNING["correction_roc"] * DT_CTRL                          # 9.0e-6 1/m per tick


def _ctl(tmp_path, monkeypatch, tuning: dict | None = None) -> LaneCenteringController:
  """Controller pointed at a temp tuning file, so tests never touch the real /data/pnw one."""
  path = tmp_path / "lanecenter_tuning.json"
  if tuning is not None:
    path.write_text(json.dumps(tuning))
  monkeypatch.setattr(lc, "TUNING_PATH", str(path))
  return LaneCenteringController()


def _run(ctl, model, n: int, v_ego: float = V_EGO) -> float:
  out = 0.0
  for _ in range(n):
    out = ctl.update(0.0, model, v_ego, True, True, True, False)
  return out


# --- the limiter itself -----------------------------------------------------------------------

def test_growth_is_capped_at_one_roc_step_per_tick(tmp_path, monkeypatch):
  """A cold start into a full-scale target must move by at most one ROC step per tick."""
  ctl = _ctl(tmp_path, monkeypatch)
  # lane center 2 m to the right of the model path -> error far past the deadband -> target pinned
  # at +FULL by max_raw_correction. Unfiltered, the first tick would be alpha*FULL = 2.96e-5.
  model = _Model(lane_center=2.0, model_y=0.0)
  prev = 0.0
  for _ in range(200):
    ctl.update(0.0, model, V_EGO, True, True, True, False)
    assert ctl._correction - prev <= STEP + 1e-15, "correction grew faster than correction_roc"
    prev = ctl._correction
  assert prev > 0.0


def test_confidence_flip_at_curve_exit_does_not_snap(tmp_path, monkeypatch):
  """The driver-reported case: correction released through a curve, lane lines re-acquired at the
  exit with a full-scale error. The re-application must be paced, not front-loaded."""
  ctl = _ctl(tmp_path, monkeypatch)
  good = _Model(lane_center=2.0, model_y=0.0)
  # Low-confidence tick(s): probs below min_lane_prob -> the "lowconf" release path.
  bad = _Model(lane_center=2.0, model_y=0.0, prob=0.1)

  _run(ctl, bad, 50)
  assert ctl._correction == pytest.approx(0.0, abs=1e-9)

  before = ctl._correction
  ctl.update(0.0, good, V_EGO, True, True, True, False)
  assert ctl._correction - before <= STEP + 1e-15


def test_limiter_paces_but_still_converges(tmp_path, monkeypatch):
  """The cap must delay the correction, never prevent it from reaching its target."""
  ctl = _ctl(tmp_path, monkeypatch)
  model = _Model(lane_center=2.0, model_y=0.0)
  _run(ctl, model, 1000)
  assert ctl._correction == pytest.approx(FULL, rel=1e-3)


def test_steady_state_is_bit_identical_to_the_unlimited_filter(tmp_path, monkeypatch):
  """Once converged (a straight, or a steady curve), the limiter must be doing NOTHING. Proven by
  running a second controller whose limiter is stubbed out and asserting the two agree bit for bit
  every tick -- not by re-deriving the expected value, and not by the weaker "the step was small"
  claim (a frozen correction would satisfy that too)."""
  ctl = _ctl(tmp_path, monkeypatch)
  ref = _ctl(tmp_path, monkeypatch)
  # `ref` is the pre-change controller: same code, limiter neutered by an unreachable ROC.
  ref._roc_limited_ticks = 0
  monkeypatch.setattr(ref, "_refresh_tuning", lambda: None)
  ref._tuning = dict(DEFAULT_TUNING, correction_roc=1e9)

  # A small center error, well inside the linear range (raw << max_raw_correction), so the target
  # genuinely tracks the geometry instead of sitting pinned at the clamp.
  model = _Model(lane_center=0.20, model_y=0.0)
  _run(ctl, model, 2000)
  _run(ref, model, 2000)
  settled = ctl._correction
  assert 0.0 < settled < FULL, f"fixture is not in the linear range: {settled}"

  clips_after_warmup = ctl._roc_limited_ticks
  # Small, continuous target motion -- the regime the limiter must stay out of.
  moved = False
  for lane_center in (0.20, 0.21, 0.22, 0.21, 0.20):
    m = _Model(lane_center=lane_center, model_y=0.0)
    for _ in range(20):
      prev = ctl._correction
      out = ctl.update(0.0, m, V_EGO, True, True, True, False)
      ref_out = ref.update(0.0, m, V_EGO, True, True, True, False)
      assert out == pytest.approx(ref_out, abs=1e-12), \
        "limiter altered the output during steady-state tracking"
      if ctl._correction != prev:
        moved = True
  assert moved, "the correction never moved -- this test would pass against a frozen controller"
  assert ctl._roc_limited_ticks == clips_after_warmup, "the cap clipped a steady-state tick"


def test_sign_flip_is_bounded_through_zero(tmp_path, monkeypatch):
  """The single biggest per-tick jump this limiter exists to bound: a converged correction whose
  target flips to the opposite full-scale value. A magnitude-only growth test would classify the
  zero-crossing tick as a "shrink" and let it through at ~6.6x the cap."""
  ctl = _ctl(tmp_path, monkeypatch)
  _run(ctl, _Model(lane_center=2.0, model_y=0.0), 2000)
  assert ctl._correction == pytest.approx(FULL, rel=1e-3)

  flipped = _Model(lane_center=-2.0, model_y=0.0)
  crossed = False
  for _ in range(600):
    prev = ctl._correction
    ctl.update(0.0, flipped, V_EGO, True, True, True, False)
    if prev > 0.0 >= ctl._correction or prev < 0.0 <= ctl._correction:
      crossed = True
      assert abs(ctl._correction - prev) <= STEP + 1e-15, \
        "the zero-crossing tick moved faster than one ROC step"
    if abs(ctl._correction) > abs(prev):
      assert abs(ctl._correction) - abs(prev) <= STEP + 1e-15, "growth exceeded one ROC step"
  assert crossed, "the correction never crossed zero -- fixture did not exercise the flip"
  assert ctl._correction == pytest.approx(-FULL, rel=1e-3), "the flip never completed"


def test_limiter_never_reverses_a_de_authorizing_step(tmp_path, monkeypatch):
  """Belt and braces on the growth-only rule: across a long shrink the limited controller must
  never hold MORE authority than the unlimited one would at the same tick."""
  ctl = _ctl(tmp_path, monkeypatch)
  ref = _ctl(tmp_path, monkeypatch)
  monkeypatch.setattr(ref, "_refresh_tuning", lambda: None)
  ref._tuning = dict(DEFAULT_TUNING, correction_roc=1e9)

  strong = _Model(lane_center=2.0, model_y=0.0)
  _run(ctl, strong, 2000)
  _run(ref, strong, 2000)
  centered = _Model(lane_center=0.0, model_y=0.0)   # target back inside the deadband -> 0
  for _ in range(400):
    ctl.update(0.0, centered, V_EGO, True, True, True, False)
    ref.update(0.0, centered, V_EGO, True, True, True, False)
    assert abs(ctl._correction) <= abs(ref._correction) + 1e-15, \
      "the limiter held more authority than the unlimited filter would have"


def test_shrinking_correction_is_not_slowed(tmp_path, monkeypatch):
  """Growth-only: reducing authority must never be paced. With the target back at zero the
  correction must decay at the exponential filter's own rate, faster than one ROC step per tick."""
  ctl = _ctl(tmp_path, monkeypatch)
  _run(ctl, _Model(lane_center=2.0, model_y=0.0), 2000)
  assert ctl._correction == pytest.approx(FULL, rel=1e-3)

  # Perfectly centered lane -> error inside the deadband -> valid tick with target 0.
  centered = _Model(lane_center=0.0, model_y=0.0)
  prev = ctl._correction
  ctl.update(0.0, centered, V_EGO, True, True, True, False)
  drop = prev - ctl._correction
  assert drop > STEP, "the limiter slowed a de-authorizing step"
  expected_alpha = 1.0 - np.exp(-DT_CTRL / DEFAULT_TUNING["smooth_tau"])
  assert drop == pytest.approx(expected_alpha * prev, rel=1e-6)


def test_release_paths_stay_instantaneous_relative_to_the_cap(tmp_path, monkeypatch):
  """Signal / low-confidence releases and reset() bypass the growth cap entirely."""
  ctl = _ctl(tmp_path, monkeypatch)
  model = _Model(lane_center=2.0, model_y=0.0)
  _run(ctl, model, 2000)
  held = ctl._correction

  # turn signal -> smooth release at signal_release_tau, unimpeded by the growth cap
  ctl.update(0.0, model, V_EGO, True, True, True, True)
  assert held - ctl._correction > STEP

  _run(ctl, model, 2000)
  ctl.reset()
  assert ctl._correction == 0.0


# --- safety-envelope guarantees ---------------------------------------------------------------

def test_json_cannot_disable_the_limiter(tmp_path, monkeypatch):
  """A JSON edit far outside the envelope (or a NaN) must clamp back into _CLAMPS, and even the
  envelope ceiling must still bind on a full-scale flip."""
  lo, hi = lc._CLAMPS["correction_roc"]
  for attempt in (1e9, 0.0, -5.0, "nonsense", None, float("nan")):
    ctl = _ctl(tmp_path, monkeypatch, {"correction_roc": attempt})
    ctl.update(0.0, _Model(), V_EGO, True, True, True, False)
    assert lo <= ctl._tuning["correction_roc"] <= hi

  # At the ceiling, one tick of a cold start must still be capped below the unlimited filter step.
  ctl = _ctl(tmp_path, monkeypatch, {"correction_roc": hi})
  ctl.update(0.0, _Model(lane_center=2.0, model_y=0.0), V_EGO, True, True, True, False)
  alpha = 1.0 - np.exp(-DT_CTRL / DEFAULT_TUNING["smooth_tau"])
  assert ctl._correction <= hi * DT_CTRL + 1e-15
  assert ctl._correction < alpha * FULL, "at the envelope ceiling the limiter no longer binds"


def test_correction_never_exceeds_the_existing_hard_ceiling(tmp_path, monkeypatch):
  """The pre-existing authority bound is untouched: the limiter only ever paces toward it."""
  ctl = _ctl(tmp_path, monkeypatch)
  model = _Model(lane_center=3.0, model_y=0.0)  # error way past anything reachable
  _run(ctl, model, 5000)
  assert abs(ctl._correction) <= FULL + 1e-12


def test_default_roc_matches_its_documented_derivation(tmp_path, monkeypatch):
  """Pins the derivation in lane_centering.py's use-site comment, so a future edit to the default
  can't quietly drift away from the arithmetic that justifies it."""
  roc = DEFAULT_TUNING["correction_roc"]
  lo, hi = lc._CLAMPS["correction_roc"]
  assert roc == 0.0009
  assert lo <= roc <= hi
  # Same fractional pacing as BluePilot af4bc410c9: ~2.7 s to traverse the full correction span.
  assert (2.0 * FULL) / roc == pytest.approx(2.7, abs=0.15)
  # And it must actually bind: one tick of it is well under the exponential filter's own largest
  # single-tick step when re-applying a full-scale correction from zero.
  alpha = 1.0 - np.exp(-DT_CTRL / DEFAULT_TUNING["smooth_tau"])
  assert roc * DT_CTRL < 0.5 * alpha * FULL


def test_limit_counter_is_telemetry_only_and_counts_real_clips(tmp_path, monkeypatch):
  """`limN` must count exactly the ticks the cap clipped, be exposed on status, and never be read
  back by control logic (it is not reset by reset(), which control state always is)."""
  ctl = _ctl(tmp_path, monkeypatch)
  assert ctl.status["limN"] == 0

  ctl.update(0.0, _Model(lane_center=2.0, model_y=0.0), V_EGO, True, True, True, False)
  assert ctl._roc_limited_ticks == 1
  assert ctl.status["limN"] == 1

  # A gated-out tick must not count.
  ctl.update(0.0, _Model(), 0.0, True, True, True, False)   # below min_v_ego
  assert ctl._roc_limited_ticks == 1

  # reset() zeroes control state but must NOT zero the odometer.
  before = ctl._roc_limited_ticks
  ctl.reset()
  assert ctl._correction == 0.0
  assert ctl._roc_limited_ticks == before
