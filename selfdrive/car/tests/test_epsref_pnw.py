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


def _logger(lines, route=lambda t: {"route": "r", "seg": 3}):
  return epsref_pnw.EpsRefusalLogger(write_fn=lines.append, route_fn=route, threaded=False)


def test_first_frame_writes_a_baseline_record_so_silence_is_distinguishable():
  lines = []
  lg = _logger(lines)
  assert lg.update(False, 2, 0, CC, CS)
  rec = json.loads(lines[0])
  assert rec["ev"] == "epsRef" and rec["ref"] is False and rec["from"] is None and rec["eac"] == 2 and rec["seg"] == 3


def test_only_changes_are_recorded():
  lines = []
  lg = _logger(lines)
  lg.update(False, 2, 0, CC, CS)
  assert not any(lg.update(False, 2, 0, CC, CS) for _ in range(500))
  assert lg.update(True, 1, 0, CC, CS)
  assert not lg.update(True, 1, 0, CC, CS)
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
