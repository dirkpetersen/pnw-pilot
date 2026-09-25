"""tools/curveshape/replay.py -- the harness's own rules, on synthetic ticks (no positions)."""
import math

import pytest

from openpilot.tools.curveshape import replay as rp

MPH = rp.MPH


def _tick(t, **kw):
  r = {"t": t, "_src": "drives/2099-01-01/x/ces_events.jsonl", "_hasK": True, "vEgo": 33.0, "stockSet": 80 * MPH,
       "stockOn": True, "mapV": 68.0 * MPH, "mapDist": 250.0 - 33.0 * (t - 1000.0), "icbmT": None, "icbmC": None,
       "icbmK": 0.00271, "icbmKD": 250.0 - 33.0 * (t - 1000.0), "icbmKN": 6, "icbmKAhead": True, "spdLim": 65 * MPH,
       "hwyClass": "primary", "waySel": "current"}
  r.update(kw)
  return r


@pytest.fixture(scope="module")
def veh():
  return rp.make_vehicle()


def test_the_replay_uses_the_real_rule_and_the_latch(veh):
  ticks = [_tick(1000.0), _tick(1001.0), _tick(1002.0)]
  res, why = rp.replay(ticks, veh, 2.0)
  assert why["replayed"] == 3
  assert res[0]["why"] == "unstable" and res[0]["new"] == res[0]["old"]     # one refresh is not persistence
  assert res[1]["why"] == "ok"
  km = 2.0 / (68.0 * MPH) ** 2
  want = math.sqrt(2.5 / (0.5 * (0.00271 + km)))
  assert res[1]["new"] == pytest.approx(want, abs=1e-6)                       # the floor gives the hump back
  assert res[1]["old"] is None                                                # today: 84.5 mph, above the 80 set


def test_a_tick_without_polyline_fields_is_unchanged(veh):
  ticks = [_tick(1000.0, _hasK=False), _tick(1001.0, _hasK=False)]
  res, _ = rp.replay(ticks, veh, 2.0)
  assert all(x["new"] == x["old"] for x in res)
  assert {x["why"] for x in res} == {"noPolylineFields"}


def _s(**kw):
  s = {"k": 0.0025, "noop": False, "a_drv": 2.5, "lead": False, "input": False, "tgt": 70.0, "v_need": 70.0}
  s.update(kw)
  return s


@pytest.mark.parametrize("kw,cls", [({"k": None}, "UNJUDGED"), ({"noop": True}, "NO-OP"), ({"a_drv": 2.3}, "REAL"),
                                    ({"a_drv": 2.0}, "GREY"), ({"a_drv": 1.7}, "PHANTOM"),
                                    ({"a_drv": 1.7, "lead": True}, "GREY"), ({"a_drv": 1.7, "input": True}, "GREY")])
def test_classify(kw, cls):
  assert rp.classify(_s(**kw)) == cls


def test_too_deep():
  assert rp.too_deep(_s(tgt=66.9, v_need=70.0))
  assert not rp.too_deep(_s(tgt=67.1, v_need=70.0))
  assert not rp.too_deep(_s(tgt=60.0, v_need=70.0, lead=True))
  assert not rp.too_deep(_s(tgt=60.0, v_need=70.0, noop=True))


def test_no_data_is_unjudged_never_a_pass():
  g = rp.gates([], [], [], [], [], [])
  for k in ("G1", "G2", "G3", "G4", "G5", "G6"):
    assert g[k][0] == "UNJUDGED", k
