"""mapsl2pnw: a dead mapd must not leave its LAST posted limit in MapSpeedLimit.

mapd has restart_if_crash=False, so before this the last limit stayed in force until reboot for every consumer of the
mem-param (speedadjust rules 1/1b + police, CES zone rules, VTSC freeway floor). Runs the REAL mapd_configd.main()
loop (configd_replay.py). "unknown" is "0.0" -- exactly what mapd itself publishes for "no limit" -- never "" (VTSC
float()s the value and would log a read failure on "").
"""
import struct

from openpilot.system.mapd.tests import configd_replay as R

MPH = 0.44704
LIM = str(float(struct.unpack("f", struct.pack("f", 60 * MPH))[0]))   # capnp stores float32
LIVE = ("mapdOut", {"speedLimit": 60 * MPH, "nextSpeedLimit": 0.0, "nextSpeedLimitDistance": 0.0,
                    "conditionalSpeedLimit": "45 @ (Mo-Fr 07:00-09:00)"})


def _live(t0, n=4):
  return [(round(t0 + k * 0.05, 2), [LIVE]) for k in range(n)]


def _silent(t0, n):
  """n polls that timed out, 1 s apart (mapdOut ages out of `alive` after 0.5 s)."""
  return [(round(t0 + k, 2), []) for k in range(1, n + 1)]


def _w(res, key):
  return [(t, v) for t, k, v in res.mem.writes if k == key]


def _clears(res):
  return [(t, v) for t, v in _w(res, "MapSpeedLimit") if v == "0.0"]


def _warns(res):
  return [kw["msg"] for name, kw in res.log.events if name == "line" and "MapSpeedLimit" in kw["msg"]]


def test_alive_publishes_and_never_clears(monkeypatch):
  res = R.run(monkeypatch, _live(0.0, 20))
  assert [v for _, v in _w(res, "MapSpeedLimit")] == [LIM] * 20
  assert _clears(res) == [] and _warns(res) == []


def test_mapd_dies_clears_once_after_the_debounce_with_one_log(monkeypatch):
  res = R.run(monkeypatch, _live(0.0) + _silent(0.15, 12))
  assert len(_clears(res)) == 1, "exactly one clear however long mapd stays dead"
  t_clear = _clears(res)[0][0]
  assert t_clear == 5.15, f"cleared at {t_clear}: the existing 5-loop debounce, not the first silent poll"
  assert res.mem.store["MapSpeedLimit"] == "0.0"
  assert res.mem.store["MapConditionalSpeedLimit"] == ""
  w = _warns(res)
  assert len(w) == 1 and "cleared to unknown" in w[0]


def test_a_short_hiccup_does_not_flap_the_limit(monkeypatch):
  """3 silent loops (< the 5-loop debounce) then mapd is back: the limit is never cleared."""
  res = R.run(monkeypatch, _live(0.0) + _silent(0.15, 3) + _live(3.5))
  assert _clears(res) == [] and _warns(res) == []
  assert res.mem.store["MapSpeedLimit"] == LIM


def test_mapd_comes_back_and_the_normal_path_republishes(monkeypatch):
  res = R.run(monkeypatch, _live(0.0) + _silent(0.15, 8) + _live(9.0))
  vals = [v for _, v in _w(res, "MapSpeedLimit")]
  assert vals == [LIM] * 4 + ["0.0"] + [LIM] * 4
  assert res.mem.store["MapConditionalSpeedLimit"] == LIVE[1]["conditionalSpeedLimit"]


def test_a_second_death_clears_again_with_its_own_log(monkeypatch):
  steps = _live(0.0) + _silent(0.15, 7) + _live(8.0) + _silent(8.15, 7)
  res = R.run(monkeypatch, steps)
  assert len(_clears(res)) == 2 and len(_warns(res)) == 2


def test_mapd_that_never_started_writes_unknown_but_logs_nothing(monkeypatch):
  """No stale value existed (or the key was unset): nothing was cleared, so there is nothing to announce; the
  write itself is idempotent and happens once."""
  res = R.run(monkeypatch, _silent(0.0, 12))
  assert len(_clears(res)) == 1 and _warns(res) == []


def test_a_stale_limit_left_in_shm_by_a_previous_configd_is_cleared_and_logged(monkeypatch):
  """mapd_configd restarts (restart_if_crash=True) while mapd is dead: its own bookkeeping starts empty, but the
  mem store still holds the old limit. The clear reads the store, not a local flag."""
  res = R.run(monkeypatch, _silent(0.0, 12), mem={"MapSpeedLimit": LIM})
  assert len(_clears(res)) == 1 and len(_warns(res)) == 1


def test_mapd_away_on_purpose_is_cleared_too_and_recovers(monkeypatch):
  """A binary swap / .override leaves mapd away for minutes: it looks exactly like a dead mapd to this loop, so the
  limit is cleared (a stale limit on a road that moved on is the bug), and comes back with mapd."""
  res = R.run(monkeypatch, _live(0.0) + _silent(0.15, 60) + _live(61.0))
  assert len(_clears(res)) == 1 and len(_warns(res)) == 1
  assert res.mem.store["MapSpeedLimit"] == LIM


def test_next_limit_is_not_rewritten_by_the_clear(monkeypatch):
  """A fresh-stamped NextMapSpeedLimit "none" would read as 'mapd is publishing' (map_fresh) -- it must age out."""
  res = R.run(monkeypatch, _live(0.0) + _silent(0.15, 12))
  nxt = _w(res, "NextMapSpeedLimit")
  assert len(nxt) == 4 and all(t < 0.2 for t, _ in nxt)


def test_a_failing_clear_is_logged_not_swallowed(monkeypatch):
  orig = R.FakeParams.put

  def boom(self, key, value):
    if key == "MapSpeedLimit" and value == "0.0":
      raise OSError("shm full")
    return orig(self, key, value)

  monkeypatch.setattr(R.FakeParams, "put", boom)
  monkeypatch.setattr(R.FakeParams, "put_nonblocking", boom)
  res = R.run(monkeypatch, _live(0.0) + _silent(0.15, 8))
  assert any("could not clear MapSpeedLimit" in kw.get("msg", "") for _, kw in res.log.events)
