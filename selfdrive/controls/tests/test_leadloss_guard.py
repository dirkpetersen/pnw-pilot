"""leadlossr2pnw — the planner must survive a failing lead-loss shadow detector, and must SAY SO (Rule 2).

The call used to be wrapped in `except Exception: pass`: a broken detector would have silently stopped the
`lead_loss_hold_shadow` telemetry that the lead-loss-hold decision is based on. These tests pin both halves:
the planner keeps running, and one cloudlog line appears (first failure at once, then at most one a minute).
"""
import importlib
import logging
import sys
import types

import pytest

from cereal import car
from openpilot.common.swaglog import cloudlog

_ACADOS = "openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx"


class _Records(logging.Handler):
  def __init__(self):
    super().__init__(level=logging.DEBUG)
    self.records = []

  def emit(self, record):
    self.records.append(record)

  def leadloss(self):
    return [r for r in self.records if "leadloss2pnw" in r.getMessage()]


class _RaisingDetector:
  def __init__(self):
    self.calls = 0

  def update(self, lead, v_ego, a_ego):
    self.calls += 1
    raise RuntimeError("detector exploded")


class _StopAfterShadow(Exception):
  """Raised by the fake carState on the first read AFTER the shadow step, to end update() there."""


class _CarState:
  vEgo = 12.0
  aEgo = -0.5

  @property
  def vCruise(self):
    raise _StopAfterShadow


@pytest.fixture
def lp(monkeypatch):
  # The compiled MPC solver is only present on a built tree; the guard under test never touches it.
  try:
    importlib.import_module(_ACADOS)
  except ImportError:
    stub = types.ModuleType(_ACADOS)
    stub.AcadosOcpSolverCython = object
    monkeypatch.setitem(sys.modules, _ACADOS, stub)
  mod = importlib.import_module("openpilot.selfdrive.controls.lib.longitudinal_planner")
  monkeypatch.setattr(mod, "LongitudinalMpc", lambda *a, **k: object())
  return mod


@pytest.fixture
def logs():
  h = _Records()
  cloudlog.addHandler(h)
  yield h
  cloudlog.removeHandler(h)


@pytest.fixture
def clock(lp, monkeypatch):
  t = [0.0]   # monotonic time can be ~0 right after boot; the first failure must still be logged
  monkeypatch.setattr(lp, "time", types.SimpleNamespace(monotonic=lambda: t[0]))
  return t


def _planner(lp):
  p = lp.LongitudinalPlanner(car.CarParams.new_message())
  p.leadloss = _RaisingDetector()
  return p


def test_update_survives_raising_detector_and_logs_once(lp, logs, clock):
  p = _planner(lp)
  sm = {"carControl": types.SimpleNamespace(orientationNED=[]), "carState": _CarState(),
        "radarState": types.SimpleNamespace(leadOne=object())}
  # update() must get PAST the shadow step: the next statement reads carState.vCruise, which stops the test.
  # Without the guard the detector's RuntimeError escapes instead.
  with pytest.raises(_StopAfterShadow):
    p.update(sm)
  assert p.leadloss.calls == 1
  lines = logs.leadloss()
  assert len(lines) == 1
  assert lines[0].levelno >= logging.ERROR
  assert lines[0].exc_info is not None            # traceback attached
  assert "NOT being recorded" in lines[0].getMessage()


def test_failure_log_is_rate_limited_to_once_a_minute(lp, logs, clock):
  p = _planner(lp)
  sm = {"carState": _CarState(), "radarState": types.SimpleNamespace(leadOne=object())}

  for _ in range(5):                               # 20 Hz burst at t=0: one line, not five
    p._leadloss_shadow_step(sm, 12.0)
  assert len(logs.leadloss()) == 1
  assert "(1 failure(s)" in logs.leadloss()[0].getMessage()

  clock[0] = 59.9
  p._leadloss_shadow_step(sm, 12.0)
  assert len(logs.leadloss()) == 1                 # still inside the minute

  clock[0] = 60.0
  p._leadloss_shadow_step(sm, 12.0)
  assert len(logs.leadloss()) == 2                 # a minute later it speaks again ...
  assert "(6 failure(s)" in logs.leadloss()[1].getMessage()   # ... counting the 5 it suppressed + this one

  clock[0] = 90.0
  p._leadloss_shadow_step(sm, 12.0)
  assert len(logs.leadloss()) == 2
  clock[0] = 120.0
  p._leadloss_shadow_step(sm, 12.0)
  assert len(logs.leadloss()) == 3
  assert "(2 failure(s)" in logs.leadloss()[2].getMessage()


def test_sm_access_errors_are_caught_too(lp, logs, clock):
  p = _planner(lp)
  p._leadloss_shadow_step({}, 12.0)                # no radarState at all -> KeyError inside the guard
  assert len(logs.leadloss()) == 1
