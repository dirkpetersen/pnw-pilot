"""teslastalk2pnw: the EPS-refusal log record (selfdrive/car/epsref_pnw.py) and its wiring in card.

The verdict is opendbc carstate's (tests in opendbc/car/tesla/tests/test_stalk_pnw.py); this is the ces_events record.
"""
import inspect
import json
from types import SimpleNamespace

from openpilot.selfdrive.car import epsref_pnw
from openpilot.selfdrive.car.card import Car

CC = SimpleNamespace(latActive=True, enabled=False)
CS = SimpleNamespace(vEgo=13.4567, steeringPressed=False)


class Clock:
  def __init__(self):
    self.t = 100.0

  def __call__(self):
    return self.t


def _logger(lines, route=lambda t: {"route": "r", "seg": 3}, clock=None):
  return epsref_pnw.EpsRefusalLogger(write_fn=lines.append, route_fn=route, threaded=False, clock=clock or Clock())


def test_first_frame_writes_a_baseline_record_so_silence_is_distinguishable():
  lines = []
  lg = _logger(lines)
  assert lg.update(False, 2, 0, CC, CS)
  rec = json.loads(lines[0])
  assert rec["ev"] == "epsRef" and rec["ref"] is False and rec["from"] is None and rec["eac"] == 2 and rec["seg"] == 3


def test_only_changes_are_recorded():
  lines = []
  clk = Clock()
  lg = _logger(lines, clock=clk)
  lg.update(False, 2, 0, CC, CS)
  assert not any(lg.update(False, 2, 0, CC, CS) for _ in range(500))
  clk.t += 10
  assert lg.update(True, 1, 0, CC, CS)
  assert not lg.update(True, 1, 0, CC, CS)
  clk.t += 10
  assert lg.update(False, 2, 0, CC, CS)
  recs = [json.loads(x) for x in lines]
  assert [r["ref"] for r in recs] == [False, True, False]
  assert recs[1]["from"] is False and recs[1]["eac"] == 1 and recs[1]["latActive"] is True and recs[1]["enabled"] is False
  assert recs[1]["vEgo"] == 13.46


def test_a_failed_write_is_counted_and_never_raises():
  def boom(_):
    raise OSError("disk full")
  lg = epsref_pnw.EpsRefusalLogger(write_fn=boom, route_fn=lambda t: {}, threaded=False)
  assert lg.update(True, 1, 0, CC, CS)      # must not raise into card
  assert lg.write_errors == 1


def test_card_builds_and_drives_it():
  src = inspect.getsource(Car)
  assert "eps_refusal_alert" in src and "EpsRefusalLogger()" in src
  assert "self._epsref.update(_cs.eps_refused, _cs.eac_status_raw, _cs.eac_error_raw" in src


def _flap(lg, clk, seconds, step=0.51):
  """The wheel-touched shape: refused / not refused alternating every 0.51 s. Returns the number of records emitted."""
  n, refused, t_end = 0, True, clk.t + seconds
  while clk.t < t_end:
    n += lg.update(refused, 1, 0, CC, CS)
    refused = not refused
    clk.t += step
  return n


def test_a_flapping_verdict_is_recorded_once_per_holdoff_and_the_rest_counted():
  lines = []
  clk = Clock()
  lg = _logger(lines, clock=clk)
  lg.update(False, 2, 0, CC, CS)
  clk.t += 30
  _flap(lg, clk, 20.0)                                    # ~39 flips; ~78 records before the hold-off (2 per 0.51 s cycle)
  recs = [json.loads(x) for x in lines]
  assert len(recs) <= 1 + 1 + int(20.0 / epsref_pnw.HOLDOFF_S) + 1, len(recs)
  assert recs[1]["ref"] is True and recs[1]["suppressed"] == 0           # the FIRST onset is recorded at once
  assert sum(r["suppressed"] for r in recs) + (len(recs) - 1) >= 35       # ~every flip is accounted for (the last few may still be pending)


def test_the_final_clear_is_recorded_after_a_flap_and_reports_the_count():
  lines = []
  clk = Clock()
  lg = _logger(lines, clock=clk)
  lg.update(False, 2, 0, CC, CS)
  clk.t += 30
  lg.update(True, 1, 0, CC, CS)                           # onset
  clk.t += 0.3
  lg.update(False, 2, 0, CC, CS)                          # clear, inside the hold-off: held
  clk.t += 0.3
  lg.update(True, 1, 0, CC, CS)                           # re-fire: held
  clk.t += 0.3
  lg.update(False, 2, 0, CC, CS)                          # the final clear: held
  assert len(lines) == 2
  clk.t += epsref_pnw.HOLDOFF_S
  assert lg.update(False, 2, 0, CC, CS)                   # hold-off over: the settled state is recorded
  last = json.loads(lines[-1])
  assert last["ref"] is False and last["from"] is True and last["suppressed"] == 3
  assert not lg.update(False, 2, 0, CC, CS)               # and then quiet


def test_a_flap_that_ends_refused_is_recorded_as_still_refused():
  lines = []
  clk = Clock()
  lg = _logger(lines, clock=clk)
  lg.update(False, 2, 0, CC, CS)
  clk.t += 30
  lg.update(True, 1, 0, CC, CS)
  clk.t += 0.3
  lg.update(False, 2, 0, CC, CS)
  clk.t += 0.3
  lg.update(True, 1, 0, CC, CS)
  clk.t += epsref_pnw.HOLDOFF_S
  assert lg.update(True, 1, 0, CC, CS)
  last = json.loads(lines[-1])
  assert last["ref"] is True and last["from"] is True and last["suppressed"] == 2
