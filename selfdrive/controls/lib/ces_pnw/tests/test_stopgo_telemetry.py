"""stopgo2pnw: the planner's StopGoStatus fields (eng, lAct, e2eStop, mpcA, mpcStop, stopHand, stopHandWhy) must reach ces_events.

ces_pnw cherry-picks mem-param keys, so a field that is published but not read here evaporates (VTSCStatus / waysel2pnw lesson). Pinned at both
ends: _read_map ingests exactly STOPGO_TELE_KEYS, and the real _event_record carries them on the tick AND the CES-off steer record.
"""
import inspect
import json
import logging
from collections import deque

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_ces_record_fields import _record

BLOB = {"eng": True, "lAct": True, "e2eStop": True, "mpcA": 0.55, "mpcStop": False, "stopHand": True, "stopHandWhy": "handoff", "junk": 1}


class _Mem:
  def __init__(self, blob):
    self._d = {"StopGoStatus": blob}

  def get(self, key, return_default=False):
    return self._d.get(key)


def _read_map(blob):
  cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_read_map"))

  class Stub:
    def __getattr__(self, n):
      return None

  g = Stub()
  g.mem_params = _Mem(blob)
  g._toggles = {"curves": True}
  g._vtsc_tele = {}
  g._bearing_hist = deque(maxlen=8)
  cls._read_map.__get__(g)()
  return g


def test_read_map_ingests_exactly_the_listed_keys():
  g = _read_map(json.dumps(BLOB))
  assert set(g._stopgo_tele) == set(m.STOPGO_TELE_KEYS)
  assert g._stopgo_tele["mpcA"] == 0.55 and g._stopgo_tele["stopHand"] is True and g._stopgo_tele["stopHandWhy"] == "handoff"


def test_absent_blob_is_null_telemetry_not_an_error(caplog):
  g = _read_map(None)
  assert g._stopgo_tele == {}


def test_an_unreadable_blob_is_logged():
  from openpilot.common.swaglog import cloudlog

  class H(logging.Handler):
    def __init__(self):
      super().__init__(level=logging.DEBUG)
      self.r = []

    def emit(self, rec):
      self.r.append(rec)
  h = H()
  cloudlog.addHandler(h)
  try:
    g = _read_map("{not json")
  finally:
    cloudlog.removeHandler(h)
  assert g._stopgo_tele == {}
  assert any("StopGoStatus read FAILED" in r.getMessage() for r in h.r)


def test_the_tick_record_carries_the_fields():
  rec = _record(_stopgo_tele={k: BLOB[k] for k in m.STOPGO_TELE_KEYS})
  for k in m.STOPGO_TELE_KEYS:
    assert rec[k] == BLOB[k], k


def test_the_steer_record_source_also_carries_them():
  src = inspect.getsource(m)
  assert src.count('**(getattr(self, "_stopgo_tele", None) or {})') == 2     # tick/adopt record + CES-off steer record
