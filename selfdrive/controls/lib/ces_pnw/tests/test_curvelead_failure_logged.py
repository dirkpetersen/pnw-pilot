"""silentexc2pnw -- a failing ICBM lead-pacing block must SAY SO with its exception type and a count (Rule 2), and must
fall back exactly as before (ICBM's own curve target, icbmLeadWhy "error").

In _icbm_step the try around icbm_lead_pace / icbm_map_sanity / icbm_path_behind / _curvelead_note already fell back to
`target = own_t` and logged through _curvelead_failed, throttled to ICBM_ERR_LOG_S -- but the line named neither the
exception nor how often it had failed. It now does, in the twistyr2pnw pattern: the first failure at once, then at most
one line per CURVELEAD_ERR_LOG_S (60 s), counting the failures since the previous line. The log call cannot escape into
_icbm_step's own except (which would lose the whole IcbmTarget publish).
"""
import logging

import pytest

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvelead2pnw import MPH, _controller, _lightning, _tick

LEAD = (50.0, 22.0)                                      # 49 mph, 50 m ahead: lead pacing engages on this curve
PATCHED = ("icbm_lead_pace", "icbm_map_sanity", "icbm_path_behind", "_curvelead_note")
ORIG = {n: getattr(m, n) for n in PATCHED}
REAL_EXCEPTION = cloudlog.exception


def _restore(monkeypatch):
  """Put the real helpers back for a reference run (monkeypatch.undo would also drop the curve-config fixture)."""
  for n in PATCHED:
    monkeypatch.setattr(m, n, ORIG[n])
  monkeypatch.setattr(m.cloudlog, "exception", REAL_EXCEPTION)


class _Records(logging.Handler):
  def __init__(self):
    super().__init__(level=logging.DEBUG)
    self.records = []

  def emit(self, record):
    self.records.append(record)

  def lead(self):
    return [r for r in self.records if isinstance(r.msg, str) and "lead pacing FAILED" in r.msg]

  def step(self):
    return [r for r in self.records if isinstance(r.msg, str) and "_icbm_step FAILED" in r.msg]


@pytest.fixture(autouse=True)
def _default_curve_cfg(tmp_path, monkeypatch):
  """As test_curvelead2pnw's default_curve_cfg: the Lightning reads the built-in curve.json defaults, never a host file."""
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "absent.json"))
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))


@pytest.fixture
def logs():
  h = _Records()
  cloudlog.addHandler(h)
  yield h
  cloudlog.removeHandler(h)


def _raise(exc):
  def f(*a, **k):
    raise exc
  return f


def _refused(*a, **k):
  """Lead pacing declining on its own: ICBM's own target passes through."""
  return None, "noLead", None


def _drive(monkeypatch, ticks=24, patch_at=None, name=None, fn=None, lead=LEAD, start=1700.0):
  """ICBM at exactly 4 Hz on the curvelead2pnw scene. From tick `patch_at`, m.<name> is replaced by `fn`.
  Returns the published IcbmTarget payloads (ts stripped) and the controller."""
  clock = [start]
  c, step = _controller(monkeypatch, clock, _lightning())
  pubs = []
  for i in range(ticks):
    if patch_at is not None and i == patch_at:
      monkeypatch.setattr(m, name, fn)
    p = _tick(c, step, 140.0, 24.0, 60 * MPH, lead)
    pubs.append({k: v for k, v in p.items() if k != "ts"} if isinstance(p, dict) else p)
    clock[0] = start + (i + 1) * 0.25
  return pubs, c


def test_positive_control_lead_pacing_raises_the_published_target(monkeypatch, logs):
  """Without this, the equalities below could pass because pacing never changed the target at all."""
  paced, c = _drive(monkeypatch)
  assert c._icbm_lead_why == "ok" and logs.lead() == []
  _restore(monkeypatch)
  own, _ = _drive(monkeypatch, patch_at=0, name="icbm_lead_pace", fn=_refused)
  assert all(p and p.get("target") is not None for p in paced + own)
  assert paced[-1]["target"] > own[-1]["target"] + 1 * MPH


CALLS = [("icbm_lead_pace", RuntimeError("code defect")), ("icbm_map_sanity", TypeError("bad")),
         ("icbm_path_behind", ValueError("bad")), ("_curvelead_note", KeyError("icbm_lead_log"))]


@pytest.mark.parametrize("name, exc", CALLS, ids=[c[0] for c in CALLS])
def test_a_failure_is_logged_with_its_type_and_publishes_icbms_own_target(monkeypatch, logs, name, exc):
  got, c = _drive(monkeypatch, patch_at=0, name=name, fn=_raise(exc))
  lines = logs.lead()
  assert len(lines) == 1                                  # 24 failing ticks at 4 Hz: one line, inside the minute
  assert lines[0].levelno >= logging.ERROR and lines[0].exc_info is not None and lines[0].exc_info[0] is type(exc)
  assert f"({type(exc).__name__})" in lines[0].msg and "(1 failure(s)" in lines[0].msg
  assert c._icbm_lead_why == "error" and logs.step() == []
  _restore(monkeypatch)
  want, _ = _drive(monkeypatch, patch_at=0, name="icbm_lead_pace", fn=_refused)
  assert got == want                                      # tick for tick: exactly ICBM's own target, never lost


def test_a_failure_mid_pacing_reverts_exactly_like_pacing_being_refused(monkeypatch, logs):
  got, c = _drive(monkeypatch, ticks=40, patch_at=20, name="icbm_map_sanity", fn=_raise(RuntimeError("boom")))
  assert c._icbm_lead_why == "error"
  _restore(monkeypatch)
  want, _ = _drive(monkeypatch, ticks=40, patch_at=20, name="icbm_lead_pace", fn=_refused)
  assert got[19]["target"] > got[-1]["target"] + 1 * MPH  # pacing really was raising the target before the failure
  assert got == want
  assert len(logs.lead()) == 1


def test_failure_log_is_rate_limited_to_once_a_minute(monkeypatch, logs):
  _drive(monkeypatch, ticks=240, patch_at=0, name="icbm_lead_pace", fn=_raise(RuntimeError("boom")))
  assert len(logs.lead()) == 1                            # ticks at +0 .. +59.75 s: still inside the minute
  _restore(monkeypatch)
  logs.records.clear()
  _drive(monkeypatch, ticks=241, patch_at=0, name="icbm_lead_pace", fn=_raise(RuntimeError("boom")))
  lines = logs.lead()
  assert len(lines) == 2                                  # ... and the tick at exactly +60 s logs again
  assert "(1 failure(s)" in lines[0].msg and "(240 failure(s)" in lines[1].msg
  assert m.CURVELEAD_ERR_LOG_S == 60.0


def test_a_raising_logger_cannot_escape_into_the_publish(monkeypatch, logs):
  """_icbm_step's own except would drop the whole IcbmTarget publish: the failure log must not be able to reach it."""
  monkeypatch.setattr(m.cloudlog, "exception", _raise(RuntimeError("logger down")))
  got, c = _drive(monkeypatch, patch_at=0, name="icbm_lead_pace", fn=_raise(TypeError("bad")))
  assert c._icbm_lead_why == "error"
  _restore(monkeypatch)
  want, _ = _drive(monkeypatch, patch_at=0, name="icbm_lead_pace", fn=_refused)
  assert all(p and p.get("target") is not None for p in got)
  assert got == want


def test_the_first_failure_after_a_quiet_minute_logs_at_once(monkeypatch, logs):
  calls = {"n": 0}
  real = m.icbm_lead_pace

  def flaky(*a, **k):
    calls["n"] += 1
    if calls["n"] in (1, 300):                             # t = 0 s and t = +74.75 s, working in between
      raise RuntimeError("boom")
    return real(*a, **k)
  _drive(monkeypatch, ticks=301, patch_at=0, name="icbm_lead_pace", fn=flaky)
  lines = logs.lead()
  assert len(lines) == 2 and "(1 failure(s)" in lines[0].msg and "(1 failure(s)" in lines[1].msg
