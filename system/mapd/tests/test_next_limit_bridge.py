"""limitahead2pnw: mapd_configd bridges mapdOut's UPCOMING limit to speedadjust as the NextMapSpeedLimit mem-param.

Runs the REAL mapd_configd.main() loop (configd_replay.py). The contract: every mapdOut that writes MapSpeedLimit also
writes NextMapSpeedLimit from the SAME message, stamped with the loop's monotonic time -- so speedadjust can tell a live
"nothing announced" from a dead bridge (stale ts).
"""
from openpilot.system.mapd import mapd_configd as M
from openpilot.system.mapd.tests import configd_replay as R

MPH = 0.44704


def _writes(res, key):
  return [(t, v) for t, k, v in res.mem.writes if k == key]


def test_every_mapdout_writes_the_upcoming_limit_from_the_same_message(monkeypatch):
  steps = [
    (0.0, [("mapdOut", {"speedLimit": 60 * MPH, "nextSpeedLimit": 40 * MPH, "nextSpeedLimitDistance": 1078.0})]),
    (0.05, [("mapdOut", {"speedLimit": 60 * MPH, "nextSpeedLimit": 40 * MPH, "nextSpeedLimitDistance": 1076.3})]),
    (0.10, [("mapdOut", {"speedLimit": 40 * MPH, "nextSpeedLimit": 0.0, "nextSpeedLimitDistance": 0.0})]),
  ]
  res = R.run(monkeypatch, steps)
  cur, nxt = _writes(res, "MapSpeedLimit"), _writes(res, "NextMapSpeedLimit")
  assert [t for t, _ in cur] == [t for t, _ in nxt] == [0.0, 0.05, 0.10]
  assert nxt[0][1] == {"sl": round(40 * MPH, 3), "d": 1078.0, "ts": 0.0}
  assert nxt[1][1] == {"sl": round(40 * MPH, 3), "d": 1076.3, "ts": 0.05}
  assert nxt[2][1] == {"sl": 0.0, "d": 0.0, "ts": 0.10}          # an explicit, fresh "nothing announced"


def test_a_silent_mapd_stops_the_writes_so_the_stamp_goes_stale(monkeypatch):
  """mapd dies: no more NextMapSpeedLimit writes, so the last one's ts ages past speedadjust's LA_INPUT_STALE_S and the
  look-ahead reads "no announcement" (and says so) instead of acting on the last value forever."""
  msg = ("mapdOut", {"speedLimit": 60 * MPH, "nextSpeedLimit": 40 * MPH, "nextSpeedLimitDistance": 500.0})
  steps = [(round(k * 0.05, 2), [msg]) for k in range(20)] + [(round(1.0 + k * 0.5, 2), []) for k in range(1, 20)]
  res = R.run(monkeypatch, steps)
  nxt = _writes(res, "NextMapSpeedLimit")
  assert nxt and [t for t, _ in nxt] == [t for t, _ in _writes(res, "MapSpeedLimit")]
  last_t, last = nxt[-1]
  assert last["ts"] == last_t and last_t < 3.0, f"still writing at {last_t} s with mapd silent since 0.95 s"


def test_the_payload_normalises_garbage_to_none():
  assert M.next_limit_payload(17.8816, 1078.04, 5.0) == {"sl": 17.882, "d": 1078.0, "ts": 5.0}
  for n, d in ((0.0, 500.0), (17.9, 0.0), (float("nan"), 10.0), (17.9, float("inf")), (-1.0, 5.0)):
    assert M.next_limit_payload(n, d, 7.0) == {"sl": 0.0, "d": 0.0, "ts": 7.0}
