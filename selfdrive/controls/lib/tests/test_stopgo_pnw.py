"""stopgo2pnw -- the lead-pulling-away-from-a-stop hand-off of the stop decision to the lead-aware MPC.

Pure-logic gate (StopGoHandoff), the planner combine (combine_stop_plan), the planner wiring (fail-closed, telemetry keys) and the
07:54:25 2026-10-02 regression (drives/2026-10-02/tesla-resume-from-stop): the lead creeps 6 -> 14 m at 1-1.5 m/s, the MPC wants to go,
the e2e model keeps shouldStop=1 for ~7 s.
"""
import importlib
import itertools
import logging
import sys
import types
from types import SimpleNamespace

import pytest

from cereal import car
from openpilot.selfdrive.controls.lib import stopgo_pnw as sg

DT = 0.05
_ACADOS = "openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx"


def lead(d=10.0, vl=1.0, y=0.0, mp=0.9, radar=True, status=True):
  return SimpleNamespace(status=status, dRel=d, vLead=vl, yRel=y, modelProb=mp, radar=radar)


def kw(**o):
  base = dict(enabled=True, active=True, v_ego=0.0, lead=None, radar_expected=True, driver_braking=False, force_decel=False, dt=DT)
  base.update(o)
  return base


def run(g, seq, **fixed):
  """seq: iterable of (d, vl) or lead objects -> list of active flags."""
  out = []
  for item in seq:
    ld = item if isinstance(item, SimpleNamespace) or item is None else lead(*item)
    out.append(g.update(**kw(lead=ld, **fixed)))
  return out


def opening(d0=6.0, vl=1.2, secs=3.0):
  """A lead pulling away at constant vl from d0, one sample per planner tick."""
  n = int(secs / DT)
  return [(d0 + vl * i * DT, vl) for i in range(n)]


# ---------------------------------------------------------------- the gate: positive path

def test_opening_lead_engages_after_the_dwell_gap_and_gain():
  g = sg.StopGoHandoff()
  flags = run(g, opening(d0=6.0, vl=1.2, secs=4.0))
  assert any(flags)
  i = flags.index(True)
  d_i = 6.0 + 1.2 * i * DT
  assert d_i >= sg.MIN_GAP                          # 8 m
  assert i * DT >= sg.OPEN_S - DT                    # >= 1 s of opening
  assert d_i - 6.0 >= sg.MIN_GAIN - 1e-9             # gap grew >= 1 m
  assert not any(flags[:i])                          # and not a tick earlier
  assert all(flags[i:])                              # then holds


def test_a_far_lead_engages_on_the_dwell_alone():
  g = sg.StopGoHandoff()
  flags = run(g, opening(d0=12.0, vl=1.5, secs=3.0))
  i = flags.index(True)
  assert (i + 1) * DT >= sg.OPEN_S - 1e-9 and i * DT < 1.2    # dwell is the binding condition


# ---------------------------------------------------------------- the gate: every refusal

def test_no_lead_never_hands_off():                # red light / stop sign / crosswalk
  g = sg.StopGoHandoff()
  assert not any(run(g, [None] * 200))
  assert g.why == "noLead"
  assert not any(run(sg.StopGoHandoff(), [lead(status=False)] * 200))


def test_a_stationary_lead_never_hands_off():
  assert not any(run(sg.StopGoHandoff(), [(20.0, 0.0)] * 200))
  assert not any(run(sg.StopGoHandoff(), [(20.0, sg.OPEN_V - 0.01)] * 200))


def test_a_shrinking_gap_never_arms():
  g = sg.StopGoHandoff()
  seq = [(12.0 - 0.1 * i * DT * 20, 1.0) for i in range(100)]      # lead "moving" per vLead but the gap closes 2 m/s
  assert not any(run(g, seq))


def test_not_enough_gain_never_engages():
  g = sg.StopGoHandoff()
  assert not any(run(g, [(10.0 + 0.2 * i * DT, 1.0) for i in range(100)]))   # 0.2 m/s of gap growth: <1 m in 5 s


def test_a_lead_below_the_min_gap_does_not_engage():
  g = sg.StopGoHandoff()
  seq = [(6.0 + 0.9 * i * DT, 0.9) for i in range(int(2.1 / DT))]    # opens for 2.1 s and gains 1.9 m, but ends at 7.9 m < MIN_GAP
  assert seq[-1][0] < sg.MIN_GAP and len(seq) * DT > sg.OPEN_S
  assert not any(run(g, seq))


def test_the_ego_must_be_stopped_to_arm():
  g = sg.StopGoHandoff()
  assert not any(run(g, opening(secs=4.0), v_ego=sg.START_V + 0.01))
  assert g.why == "moving"


def test_driver_braking_blocks_arming_and_ends_a_handoff():
  assert not any(run(sg.StopGoHandoff(), opening(secs=4.0), driver_braking=True))
  g = sg.StopGoHandoff()
  run(g, opening(secs=3.0))
  assert g.active
  assert g.update(**kw(lead=lead(), driver_braking=True)) is False


def test_force_decel_blocks():
  assert not any(run(sg.StopGoHandoff(), opening(secs=4.0), force_decel=True))
  g = sg.StopGoHandoff()
  run(g, opening(secs=3.0))
  assert g.update(**kw(lead=lead(), force_decel=True)) is False


@pytest.mark.parametrize("ld,why", [
  (lead(y=2.0), "offLane"), (lead(y=-1.6), "offLane"),
  (lead(mp=0.0), "noVision"), (lead(mp=0.49), "noVision"),
  (lead(radar=False), "noRadar"),
  (lead(d=float("nan")), "badLead"), (lead(vl=float("inf")), "badLead"), (lead(y=float("nan")), "badLead"), (lead(mp=float("nan")), "badLead"),
])
def test_lead_quality_refusals(ld, why):
  g = sg.StopGoHandoff()
  assert not any(run(g, [ld] * 120))
  assert g.why == why


def test_a_radarless_car_accepts_a_vision_only_lead():
  g = sg.StopGoHandoff()
  flags = [g.update(**kw(lead=lead(d=d, vl=v, radar=False), radar_expected=False)) for d, v in opening(secs=3.0)]
  assert any(flags)


def test_a_single_glitch_tick_resets_the_dwell():
  g = sg.StopGoHandoff()
  seq = opening(d0=6.0, vl=1.5, secs=3.0)
  seq[10] = None                                                      # radar blink at t=0.5 s
  flags = run(g, seq)
  assert flags.index(True) > 10 + int(sg.OPEN_S / DT) - 2             # the clock restarted after the blink


# ---------------------------------------------------------------- re-latch

def _engaged(d0=6.0, vl=1.5):
  g = sg.StopGoHandoff()
  run(g, opening(d0=d0, vl=vl, secs=3.0))
  assert g.active
  return g, d0 + vl * 3.0


def test_losing_the_lead_relatches_at_once_and_a_second_handoff_needs_a_new_context():
  g, d = _engaged()
  assert g.update(**kw(lead=None, v_ego=0.4)) is False and g.why == "noLead"
  # a one-tick radar blink is NOT a new context: the hand-off is used up
  flags = run(g, [(d + 0.05 * i, 1.5) for i in range(1, 80)], v_ego=0.0)
  assert not any(flags) and g.why == "used"
  # nor is a long dropout (radar flicker on the same moving lead): still used
  g, d = _engaged()
  for _ in range(int(3.0 / DT)):
    g.update(**kw(lead=None, v_ego=0.0))
  flags = run(g, [(d + 0.05 * i, 1.5) for i in range(1, 80)], v_ego=0.0)
  assert not any(flags) and g.why == "used"


def test_a_shrinking_gap_relatches():
  g, d = _engaged()
  assert g.update(**kw(lead=lead(d=d + 0.5, vl=1.5), v_ego=0.6)) is True
  assert g.update(**kw(lead=lead(d=d + 0.5 - sg.KEEP_SHRINK_TOL - 0.05, vl=1.5), v_ego=0.6)) is False
  assert g.why == "gapShrinking"


def test_gap_dip_within_radar_noise_does_not_relatch():
  g, d = _engaged()
  assert g.update(**kw(lead=lead(d=d - 0.5, vl=1.5), v_ego=0.6)) is True


def test_a_lead_that_stops_or_reverses_relatches():
  g, d = _engaged()
  assert g.update(**kw(lead=lead(d=d, vl=sg.KEEP_V - 0.05), v_ego=0.6)) is False and g.why == "leadStopped"
  g, d = _engaged()
  assert g.update(**kw(lead=lead(d=d, vl=-0.5), v_ego=0.6)) is False


def test_vision_or_lane_disagreement_while_engaged_relatches():
  g, d = _engaged()
  assert g.update(**kw(lead=lead(d=d, mp=0.1), v_ego=0.6)) is True        # a single dropout is inside the vision grace ...
  n = int(sg.VISION_GRACE_S / DT) + 2
  flags = [g.update(**kw(lead=lead(d=d, mp=0.1), v_ego=0.6)) for _ in range(n)]
  assert flags[-1] is False and g.why == "noVision"                       # ... but vision that stays away past it is a disagreement
  g, d = _engaged()
  assert g.update(**kw(lead=lead(d=d, y=2.5), v_ego=0.6)) is False and g.why == "offLane"
  g, d = _engaged()
  assert g.update(**kw(lead=lead(d=d, radar=False), v_ego=0.6)) is False and g.why == "noRadar"


def test_the_handoff_sustains_through_the_launch_and_ends_at_the_stoplatch_release():
  g, d = _engaged()
  for v in (0.2, 0.6, 0.9, 1.2, sg.RELEASE_V - 0.01):
    d += 0.05
    assert g.update(**kw(lead=lead(d=d, vl=1.5), v_ego=v)) is True    # above START_V it must NOT fall back to the e2e veto mid-launch
  assert g.update(**kw(lead=lead(d=d, vl=1.5), v_ego=sg.RELEASE_V)) is False
  assert g.why == "inactive"
  # after the release it cannot restart from a rolling car
  assert g.update(**kw(lead=lead(d=d, vl=1.5), v_ego=0.9)) is False and g.why == "used"


def test_the_release_speed_is_the_stoplatch_release():
  from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
  assert sg.RELEASE_V == C.NOCHILL_RELEASE_V == 1.3


# ---------------------------------------------------------------- switch OFF / not op-long

def test_switch_off_and_inactive_are_inert():
  assert not any(run(sg.StopGoHandoff(), opening(secs=4.0), enabled=False))
  assert not any(run(sg.StopGoHandoff(), opening(secs=4.0), active=False))
  g, _ = _engaged()
  assert g.update(**kw(lead=lead(), enabled=False)) is False and g.why == "off"
  assert g.update(**kw(lead=lead(), v_ego=float("nan"))) is False


# ---------------------------------------------------------------- the combine

def _stock(experimental, a_e2e, stop_e2e, a_mpc, stop_mpc):
  if experimental:
    return min(a_e2e, a_mpc), bool(stop_e2e or stop_mpc), a_e2e < a_mpc
  return a_mpc, stop_mpc, False


@pytest.mark.parametrize("exp,ae,se,am,sm", list(itertools.product((True, False), (-1.0, 0.02, 0.8), (False, True), (-0.5, 0.15, 1.5), (False, True))))
def test_combine_is_exactly_stock_when_the_handoff_is_off(exp, ae, se, am, sm):
  assert sg.combine_stop_plan(exp, False, ae, se, am, sm) == _stock(exp, ae, se, am, sm)


def test_combine_hands_the_stop_to_the_mpc_and_caps_the_launch():
  a, stop, e2e = sg.combine_stop_plan(True, True, 0.02, True, 1.5, False)
  assert (a, stop, e2e) == (sg.ACCEL_CAP, False, False)                       # e2e's shouldStop=1 no longer vetoes; launch capped
  a, stop, _ = sg.combine_stop_plan(True, True, 0.02, False, 0.15, False)
  assert a == 0.15                                                           # below the cap: the MPC's own value
  a, stop, _ = sg.combine_stop_plan(True, True, 0.02, False, -0.7, True)
  assert (a, stop) == (-0.7, True)                                           # the MPC can still say stop, and braking is not capped


def test_chill_ignores_the_handoff():
  assert sg.combine_stop_plan(False, True, 0.0, True, 0.4, False) == (0.4, False, False)


# ---------------------------------------------------------------- 2026-10-02 07:54:25 regression

# (t s from 07:54:25.0, dRel m, vLead m/s) -- from the report's 10 Hz table; linear between rows
_TRACE = [(0.0, 6.1, 0.50), (0.5, 6.6, 0.88), (2.0, 8.5, 1.50), (5.0, 12.1, 0.88), (7.0, 14.6, 1.31)]


def _interp(t):
  for (t0, d0, v0), (t1, d1, v1) in zip(_TRACE, _TRACE[1:], strict=False):
    if t0 <= t <= t1:
      f = (t - t0) / (t1 - t0)
      return d0 + f * (d1 - d0), v0 + f * (v1 - v0)
  raise ValueError(t)


def test_regression_0754_creeping_lead_releases_the_e2e_hold_about_two_seconds_in():
  g = sg.StopGoHandoff()
  t_on = None
  for i in range(int(7.0 / DT)):
    t = i * DT
    d, v = _interp(t)
    if g.update(**kw(lead=lead(d=d, vl=v), v_ego=0.0)) and t_on is None:
      t_on = t
  assert t_on is not None and 1.0 <= t_on <= 2.5      # the report's driver had to press the gas at t=6.5 s
  # the launch decision then comes from the MPC (0.15 -> 0.69 m/s^2 "go"), not from the e2e shouldStop=1 / a=0.02
  a, stop, _ = sg.combine_stop_plan(True, True, 0.026, True, 0.55, False)
  assert stop is False and a == 0.55


def test_regression_a_red_light_with_no_lead_never_releases():
  g = sg.StopGoHandoff()
  assert not any(g.update(**kw(lead=None)) for _ in range(int(60 / DT)))


# ---------------------------------------------------------------- planner wiring

@pytest.fixture
def lp(monkeypatch):
  try:
    importlib.import_module(_ACADOS)
  except ImportError:
    stub = types.ModuleType(_ACADOS)
    stub.AcadosOcpSolverCython = object
    monkeypatch.setitem(sys.modules, _ACADOS, stub)
  mod = importlib.import_module("openpilot.selfdrive.controls.lib.longitudinal_planner")
  monkeypatch.setattr(mod, "LongitudinalMpc", lambda *a, **k: object())
  return mod


class _Records(logging.Handler):
  def __init__(self):
    super().__init__(level=logging.DEBUG)
    self.records = []

  def emit(self, record):
    self.records.append(record)


@pytest.fixture
def logs():
  from openpilot.common.swaglog import cloudlog
  h = _Records()
  cloudlog.addHandler(h)
  yield h
  cloudlog.removeHandler(h)


class _SM(dict):
  def __init__(self, *a, alive=True, **k):
    super().__init__(*a, **k)
    self.alive = {"radarState": alive}


def _sm(ld, brake=False, enabled=True, long_active=True, alive=True, gas=False):
  return _SM({"radarState": SimpleNamespace(leadOne=ld), "carState": SimpleNamespace(brakePressed=brake, gasPressed=gas),
              "selfdriveState": SimpleNamespace(enabled=enabled), "carControl": SimpleNamespace(longActive=long_active)}, alive=alive)


def _planner(lp, op_long=True, radar_unavailable=False, switch=True):
  cp = car.CarParams.new_message(openpilotLongitudinalControl=op_long, radarUnavailable=radar_unavailable)
  p = lp.LongitudinalPlanner(cp)
  p.veh = SimpleNamespace(stop_go_handoff=switch, refresh_curve_brain_cfg=lambda **k: False)    # the planner only reads this capability + reloads it
  return p


def test_planner_step_engages_on_an_opening_lead(lp):
  p = _planner(lp)
  flags = [p._stopgo_step(_sm(lead(d=d, vl=v)), 0.0, False, False) for d, v in opening(secs=3.0)]
  assert any(flags)


@pytest.mark.parametrize("kwargs,reset", [(dict(op_long=False), False), (dict(switch=False), False), ({}, True)])
def test_planner_step_is_inert_without_op_long_without_the_capability_or_when_reset(lp, kwargs, reset):
  p = _planner(lp, **kwargs)
  assert not any(p._stopgo_step(_sm(lead(d=d, vl=v)), 0.0, reset, False) for d, v in opening(secs=3.0))


def test_planner_step_brake_pressed_blocks(lp):
  p = _planner(lp)
  assert not any(p._stopgo_step(_sm(lead(d=d, vl=v), brake=True), 0.0, False, False) for d, v in opening(secs=3.0))


def test_planner_step_dead_radarstate_is_no_lead(lp):
  p = _planner(lp)
  assert not any(p._stopgo_step(_sm(lead(d=d, vl=v), alive=False), 0.0, False, False) for d, v in opening(secs=3.0))


def test_planner_radarless_car_uses_vision_only_lead(lp):
  p = _planner(lp, radar_unavailable=True)
  assert any(p._stopgo_step(_sm(lead(d=d, vl=v, radar=False)), 0.0, False, False) for d, v in opening(secs=3.0))
  p = _planner(lp, radar_unavailable=False)
  assert not any(p._stopgo_step(_sm(lead(d=d, vl=v, radar=False)), 0.0, False, False) for d, v in opening(secs=3.0))


def test_a_raising_gate_fails_closed_and_says_so_once(lp, logs, monkeypatch):
  p = _planner(lp)

  def boom(**k):
    raise RuntimeError("gate exploded")
  p.stopgo.update = boom
  p.stopgo.active = True
  assert p._stopgo_step(_sm(lead()), 0.0, False, False) is False
  assert p.stopgo.active is False
  msgs = [r for r in logs.records if "stopgo2pnw" in r.getMessage()]
  assert len(msgs) == 1 and msgs[0].levelno >= logging.ERROR and msgs[0].exc_info is not None
  p._stopgo_step(_sm(lead()), 0.0, False, False)
  assert len([r for r in logs.records if "stopgo2pnw" in r.getMessage()]) == 1     # rate-limited


def test_the_handoff_is_logged_change_only_with_the_lead_numbers(logs):
  from openpilot.common.swaglog import cloudlog
  seen = []
  orig = cloudlog.event
  cloudlog.event = lambda name, **kv: seen.append((name, kv))
  try:
    g = sg.StopGoHandoff()
    run(g, opening(secs=3.0))
    n_on = len(seen)
    run(g, opening(d0=12.0, secs=1.0))             # still engaged: no new event
    assert len(seen) == n_on == 1
    g.update(**kw(lead=None))
  finally:
    cloudlog.event = orig
  assert [(n, kv["on"]) for n, kv in seen] == [("stop_go_handoff", True), ("stop_go_handoff", False)]
  assert {"dRel", "vLead", "gain", "radar", "modelProb", "v_ego", "why"} <= set(seen[0][1])
  assert seen[1][1]["why"] == "noLead"


# ---------------------------------------------------------------- telemetry reaches ces_events

def test_published_keys_equal_the_ces_cherry_pick_list(lp):
  from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
  p = _planner(lp)
  put = []
  p._stopgo_mem = SimpleNamespace(put_nonblocking=lambda k, v: put.append((k, v)))
  p._stopgo_publish(_sm(lead()), 0.15, False, True)
  assert put and put[0][0] == "StopGoStatus"
  assert set(put[0][1]) == set(m.STOPGO_TELE_KEYS)                          # a key published but not listed would evaporate
  assert put[0][1]["e2eStop"] is True and put[0][1]["mpcStop"] is False and put[0][1]["lAct"] is True and put[0][1]["eng"] is True


def test_a_failing_publish_is_logged_not_swallowed(lp, logs):
  p = _planner(lp)

  def boom(k, v):
    raise OSError("shm gone")
  p._stopgo_mem = SimpleNamespace(put_nonblocking=boom)
  p._stopgo_publish(_sm(lead()), 0.0, False, False)
  assert any("StopGoStatus publish" in r.getMessage() and r.levelno >= logging.ERROR for r in logs.records)


def test_the_param_key_is_registered():
  import pathlib
  h = (pathlib.Path(__file__).parents[4] / "common" / "params_keys.h").read_text()
  assert h.count('{"StopGoStatus", {CLEAR_ON_MANAGER_START, JSON}}') == 1


def test_the_planner_reloads_its_own_vehicle_config_each_tick(lp):
  """VTSC owns a separate PnwVehicle; without this call the kill switch would only ever be read at plannerd start."""
  p = _planner(lp)
  calls = []
  p.veh = SimpleNamespace(stop_go_handoff=True, refresh_curve_brain_cfg=lambda **k: calls.append(k))
  p._stopgo_step(_sm(lead()), 0.0, False, False)
  assert calls == [{"log_event": False}]                 # and quietly: VTSC's instance owns the reload event


def test_vision_flicker_on_a_radar_tracked_lead_does_not_break_the_gate():
  """07:54:26 / 09-30 07:50: modelProb alternates ~1.0 <-> 0.0 for up to ~1.5 s on a lead the radar tracks the whole time."""
  g = sg.StopGoHandoff()
  flags = []
  for i in range(int(5.0 / DT)):
    d = 6.0 + 1.3 * i * DT
    mp = 0.0 if 0.8 <= (i * DT) % 2.0 < 1.6 else 0.99          # 0.8 s dropouts every 2 s
    flags.append(g.update(**kw(lead=lead(d=d, vl=1.3, mp=mp))))
  assert flags[-1] is True and flags.index(True) * DT < 2.5


def test_a_lead_vision_never_confirmed_is_never_handed_off():
  g = sg.StopGoHandoff()
  assert not any(run(g, [lead(d=6.0 + 1.3 * i * DT, vl=1.3, mp=0.0) for i in range(100)]))
  assert g.why == "noVision"


def test_a_gap_dip_while_arming_restarts_the_dwell():
  """The lead's vLead says 'opening' but the measured gap dipped more than the radar-noise tolerance: the dwell restarts (not just a refusal at the end)."""
  g = sg.StopGoHandoff()
  seq = [(9.0 + 1.5 * i * DT, 1.5) for i in range(12)]                       # 0.6 s of clean opening, already past MIN_GAP
  d_dip = seq[-1][0] - sg.SHRINK_TOL - 0.2
  seq.append((d_dip, 1.5))                                                   # one tick with the gap 0.7 m below its running max
  seq += [(d_dip + 1.5 * (i + 1) * DT, 1.5) for i in range(60)]
  flags = run(g, seq)
  assert flags.index(True) >= 13 + int(sg.OPEN_S / DT) - 1                   # the clock restarted at the dip, so >= 1 s AFTER it


def test_a_not_opening_tick_restarts_the_dwell():
  g = sg.StopGoHandoff()
  seq = [(9.0 + 1.5 * i * DT, 1.5) for i in range(16)]                       # 0.8 s opening, already past MIN_GAP
  d = seq[-1][0]
  seq.append((d + 0.05, sg.OPEN_V - 0.2))                                    # one tick where the lead is not (yet) moving away
  seq += [(d + 0.05 + 1.5 * (i + 1) * DT, 1.5) for i in range(60)]
  flags = run(g, seq)
  assert flags.index(True) >= 17 + int(sg.OPEN_S / DT) - 1


# ---------------------------------------------------------------- one hand-off per stop (review F1), e2e braking (F2), time cap (F3), shrink pin (F5)

def _used_up(how):
  """An engaged gate that has just ended for the given reason; the lead keeps pulling away (the e2e model is 'right')."""
  g, d = _engaged(d0=9.0, vl=1.5)
  if how == "e2eBraking":
    assert g.update(**kw(lead=lead(d=d), v_ego=0.6, e2e_accel=-1.5)) is False and g.why == "e2eBraking"
  elif how == "release":
    assert g.update(**kw(lead=lead(d=d), v_ego=sg.RELEASE_V + 0.1)) is False
  elif how == "timeCap":
    for _ in range(int(sg.MAX_HANDOFF_S / DT) + 5):
      d += 1.0 * DT
      g.update(**kw(lead=lead(d=d, vl=1.0), v_ego=1.0))
    assert not g.active and g.why in ("timeCap", "used")
  elif how == "gap":
    assert g.update(**kw(lead=lead(d=d - 2.0), v_ego=0.6)) is False and g.why == "gapShrinking"
  return g, d


@pytest.mark.parametrize("how", ["e2eBraking", "release", "timeCap", "gap"])
def test_one_handoff_per_stop_the_lead_moving_away_is_not_a_new_context(how):
  g, d = _used_up(how)
  # ego back down to a stop, lead still opening: the e2e model was right to hold; no second lunge, however long it lasts
  flags = [g.update(**kw(lead=lead(d=d + 1.5 * i * DT, vl=1.5), v_ego=0.0)) for i in range(int(30 / DT))]
  assert not any(flags) and g.why == "used"


def _new_context_then_opening(g, d, action):
  if action == "queue":                       # the lead itself stops for QUEUE_S
    for _ in range(int(sg.QUEUE_S / DT) + 1):
      g.update(**kw(lead=lead(d=d, vl=0.0), v_ego=0.0))
  elif action == "fast":
    g.update(**kw(lead=lead(d=d, vl=1.5), v_ego=sg.REARM_V + 0.5))
  elif action == "gas":
    g.update(**kw(lead=lead(d=d, vl=1.5), v_ego=0.0, driver_gas=True))
  return [g.update(**kw(lead=lead(d=d + 1.5 * i * DT, vl=1.5), v_ego=0.0)) for i in range(int(4 / DT))]


@pytest.mark.parametrize("action", ["queue", "fast", "gas"])
def test_a_new_context_allows_the_next_handoff(action):
  g, d = _used_up("e2eBraking")
  flags = _new_context_then_opening(g, d, action)
  assert any(flags), action


@pytest.mark.parametrize("action", ["queue", "fast", "gas"])
def test_a_context_change_is_needed_to_clear_used_not_just_any_tick(action):
  """The partial versions (a too-short stop / absence, a small jump, a slow ego, no gas) must NOT clear it."""
  g, d = _used_up("e2eBraking")
  if action == "queue":
    g.update(**kw(lead=lead(d=d, vl=0.0), v_ego=0.0))
  elif action == "fast":
    g.update(**kw(lead=lead(d=d, vl=1.5), v_ego=sg.REARM_V - 0.5))
  flags = [g.update(**kw(lead=lead(d=d + 1.5 * i * DT, vl=1.5), v_ego=0.0)) for i in range(int(4 / DT))]
  assert not any(flags), action


def test_op_long_off_or_switch_off_is_a_new_context():
  for kwargs in (dict(active=False), dict(enabled=False)):
    g, d = _used_up("e2eBraking")
    g.update(**kw(lead=lead(d=d), **kwargs))
    assert any(run(g, [(d + 1.5 * i * DT, 1.5) for i in range(int(4 / DT))])), kwargs


def test_e2e_braking_ends_the_handoff_only_when_moving_and_hard_enough():
  g, d = _engaged()
  assert g.update(**kw(lead=lead(d=d), v_ego=0.6, e2e_accel=-0.29)) is True      # mild: the stuck cases had +0.02..+0.05
  assert g.update(**kw(lead=lead(d=d), v_ego=sg.E2E_BRAKE_V - 0.05, e2e_accel=-2.0)) is True   # below 0.3 m/s the model speaks via shouldStop, not accel
  assert g.update(**kw(lead=lead(d=d), v_ego=0.6, e2e_accel=-0.31)) is False and g.why == "e2eBraking"
  assert sg.E2E_BRAKE_A == -0.3 and sg.E2E_BRAKE_V == 0.3


def test_e2e_braking_blocks_arming_too():
  g = sg.StopGoHandoff()
  flags = [g.update(**kw(lead=lead(d=d, vl=v), v_ego=0.4, e2e_accel=-1.0)) for d, v in opening(secs=4.0)]
  assert not any(flags)


def test_the_time_cap_ends_a_slow_creep_handoff():
  g, d = _engaged()
  already = g._dur
  n = 0
  while g.active and n < int(10 / DT):
    d += 1.0 * DT
    g.update(**kw(lead=lead(d=d, vl=1.0), v_ego=1.0))
    n += 1
  assert not g.active and g.why == "timeCap" and abs(already + n * DT - sg.MAX_HANDOFF_S) <= 2 * DT and sg.MAX_HANDOFF_S == 5.0


def test_chill_ends_the_handoff_without_resetting_context():
  g, d = _engaged()
  assert g.update(**kw(lead=lead(d=d), v_ego=0.6, experimental=False)) is False and g.why == "chill"
  assert not any(g.update(**kw(lead=lead(d=d + 1.5 * i * DT, vl=1.5))) for i in range(int(4 / DT)))


def test_the_engaged_shrink_tolerance_is_one_metre():
  """Literal numbers (the constant is also asserted): 3.0 m would let a closing lead run the hand-off on."""
  assert sg.KEEP_SHRINK_TOL == 1.0 and sg.SHRINK_TOL == 0.5
  g, d = _engaged()
  assert g.update(**kw(lead=lead(d=d - 0.9, vl=1.5), v_ego=0.6)) is True
  g, d = _engaged()
  assert g.update(**kw(lead=lead(d=d - 1.5, vl=1.5), v_ego=0.6)) is False and g.why == "gapShrinking"


def test_planner_passes_gas_e2e_accel_and_mode_through(lp):
  p = _planner(lp)
  seen = []
  p.stopgo.update = lambda **k: seen.append(k) or False
  p._stopgo_step(_sm(lead(), gas=True), 0.2, False, False, False, -0.7)
  k = seen[0]
  assert k["driver_gas"] is True and k["e2e_accel"] == -0.7 and k["experimental"] is False and k["active"] is True


def test_radar_noise_on_the_same_moving_lead_is_never_a_new_context():
  """Review F1-noise: a 0.2 s out-and-back dRel glitch (10-01 09:41:56.9: 25.9 -> 21.5 -> 28.9 m) and a 1.5 s dropout while the lead keeps
  moving away must not re-arm a second hand-off (they used to: a lunge toward a lead ~40 m away)."""
  g, d = _used_up("e2eBraking")
  flags = []
  for i in range(int(40 / DT)):
    t = i * DT
    dd = d + 1.5 * t
    ld = lead(d=dd, vl=1.5)
    if 10.0 <= t < 10.2:
      ld = lead(d=dd - 4.4, vl=1.5)           # out ...
    elif 10.2 <= t < 10.4:
      ld = lead(d=dd + 3.0, vl=1.5)           # ... and back
    elif 20.0 <= t < 21.5:
      ld = None                               # a 1.5 s dropout
    elif 30.0 <= t < 33.0:
      ld = lead(d=dd + 12.0, vl=1.5)          # a 3 s wrong-track stretch
    flags.append(g.update(**kw(lead=ld, v_ego=0.0)))
  assert not any(flags) and g.why == "used"


def test_the_queue_stop_must_be_continuous():
  """Two 0.3 s stretches of a stopped lead around a dropout are not a 0.5 s queue stop."""
  g, d = _used_up("e2eBraking")
  n = int(0.35 / DT)                                                         # 0.35 s each: < QUEUE_S alone, > QUEUE_S together
  for _ in range(n):
    g.update(**kw(lead=lead(d=d, vl=0.0), v_ego=0.0))
  g.update(**kw(lead=None, v_ego=0.0))
  for _ in range(n):
    g.update(**kw(lead=lead(d=d, vl=0.0), v_ego=0.0))
  flags = [g.update(**kw(lead=lead(d=d + 1.5 * i * DT, vl=1.5), v_ego=0.0)) for i in range(int(4 / DT))]
  assert not any(flags) and g.why == "used"
