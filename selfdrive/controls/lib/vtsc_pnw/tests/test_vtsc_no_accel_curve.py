"""noaccelcurve2pnw (owner-approved 2026-10-26): on the Raven, VTSC does not let the car ACCELERATE toward the set speed while a curve within
NO_ACCEL_CURVE_AHEAD_M (200 m) would be uncomfortable at the SET speed (set^2 x k > A_LAT_TARGET), k = the sharpest of the curve brain's row and
mapd's polyline curvature. The cap holds max(the car's speed at the start of the hold, the car's speed now): lower-only, never below vEgo.

Drive 2026-10-26 20:09 PT (drives/2026-10-26/tesla-i5-olympia-grand-mound): the lead left at 72 mph, the Tesla accelerated at ~1.3 m/s^2 to
88 mph toward set 90 into a left curve (DB k 0.00175, mapd k 0.0015). VTSC and the brain priced it above the set speed, so neither acted.

Branch tests call _no_accel_curve() on a real controller; the replay drives the REAL cap() closed loop with the 20:09 numbers."""
import json
import types

import pytest

from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import VTSC_TELE_KEYS
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_constants as C
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as vc
from openpilot.selfdrive.controls.lib.vtsc_pnw.tests.test_curvebrain_vtsc_pnw import _CLOCK, _CP, _Mem, _Params, _sm, LIGHTNING, MPH, TESLA


@pytest.fixture(autouse=True)
def env(monkeypatch, tmp_path):
  """The fake clock and a private curve.json (as in test_curvebrain_vtsc_pnw)."""
  _CLOCK[0] = 1000.0
  monkeypatch.setattr(vc, "time", types.SimpleNamespace(monotonic=lambda: _CLOCK[0]))
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "curve.json"))
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))
  return tmp_path / "curve.json"


SET = 90 * MPH           # 40.2 m/s
K_DB = 0.00175           # the 20:09 curve DB row: 40.2^2 x 0.00175 = 2.83 m/s^2 > 2.5 -> uncomfortable at the set speed
K_MAPD = 0.0015          # the 20:09 mapd value: 2.43 m/s^2 < 2.5 -> comfortable on its own


def _ctrl(env_path, cfg=None, car=TESLA):
  env_path.write_text(json.dumps({"tesla": dict({"curve_brain": "lower"}, **(cfg or {}))}))
  c = vc.VTSCController(_CP(*car), params=_Params())
  c.mem_params = _Mem()
  return c


def _db(c, k=K_DB, d=150.0, v=45.0, mode="lower"):
  """Put a fresh curve-brain entry in place, as _apply_brain's _read_brain leaves it."""
  c._cb_entry = {"mode": mode, "v": v, "d": d, "ev": "measured", "src": "db", "row": "1:0", "a": 3.59, "k": k}
  c._cb_age, c._cb_problem = 0.0, None


def _mapd(c, k, d=150.0, n=3, ahead=True):
  c._tele_mapk, c._tele_mapk_d, c._tele_mapk_n, c._tele_mapk_ahead = k, d, n, ahead


def _logs(monkeypatch):
  lines = []
  for lvl in ("info", "warning", "error", "exception"):
    monkeypatch.setattr(vc.cloudlog, lvl, lambda msg, *a, _l=lvl, **k: lines.append((_l, msg % a if a else msg)))
  return lines


# ------------------------------------------------------------------------------------------- the branch

def test_a_db_curve_uncomfortable_at_the_set_speed_holds_the_cap_at_vego(env):
  c = _ctrl(env)
  _db(c)
  assert c._no_accel_curve(SET, 32.0, SET) == pytest.approx(32.0)
  assert c._tele_nc == ("hold", 32.0)
  # the car slows (a lead): the cap stays at the start speed, it may come back to it but not above it
  assert c._no_accel_curve(SET, 30.0, SET) == pytest.approx(32.0)
  # the car is faster than the start speed (downhill): the cap follows the car, it never brakes it
  assert c._no_accel_curve(SET, 33.0, SET) == pytest.approx(33.0)
  # lower-only: a lower cap from VTSC / the brain passes through
  assert c._no_accel_curve(SET, 33.0, 28.0) == pytest.approx(28.0)


def test_a_mapd_curve_alone_also_holds(env):
  c = _ctrl(env, {"curve_brain": "off"})
  _mapd(c, 0.002)
  assert c._no_accel_curve(SET, 32.0, SET) == pytest.approx(32.0) and c._tele_nc[0] == "hold"


def test_the_sharpest_of_db_and_mapd_decides(env):
  c = _ctrl(env)
  _db(c, k=0.0010)                     # comfortable
  _mapd(c, K_DB)                       # uncomfortable
  assert c._no_accel_curve(SET, 32.0, SET) == pytest.approx(32.0)


@pytest.mark.parametrize("why, db, mapd", [
  ("comfortable at the set speed (the 20:09 mapd k)", None, (K_MAPD, 150.0)),
  ("comfortable DB row", (0.0015, 150.0), None),
  ("DB row beyond 200 m", (K_DB, 201.0), None),
  ("mapd curve beyond 200 m", None, (K_DB, 201.0)),
  ("mapd point behind the car", None, (K_DB, 50.0, 3, False)),
  ("the DB has no row ahead", "noNeed", None),
])
def test_no_uncomfortable_curve_ahead_changes_nothing(env, why, db, mapd):
  c = _ctrl(env)
  if db == "noNeed":
    c._cb_entry, c._cb_age, c._cb_problem = {"mode": "lower", "v": None}, 0.0, "noNeed"
  elif db:
    _db(c, k=db[0], d=db[1])
  if mapd:
    _mapd(c, *mapd)
  assert c._no_accel_curve(SET, 32.0, SET) == SET, why
  assert c._tele_nc == ("clear", None), why


def test_the_hold_ends_when_the_curve_is_past_and_restarts_fresh(env):
  c = _ctrl(env)
  _db(c)
  c._no_accel_curve(SET, 32.0, SET)
  _db(c, k=0.0005)                     # past the curve: the next row is mild
  assert c._no_accel_curve(SET, 33.0, SET) == SET and c._nc_hold is None
  _db(c)
  assert c._no_accel_curve(SET, 35.0, SET) == pytest.approx(35.0)     # a new hold starts at the speed of then


def test_the_dead_reckoned_distance_of_an_aged_entry_is_used(env):
  c = _ctrl(env)
  _db(c, d=230.0)
  c._cb_age = 1.0                      # 230 - 32 x 1.0 = 198 m now
  assert c._no_accel_curve(SET, 32.0, SET) == pytest.approx(32.0)


# ------------------------------------------------------------------------------------------- inactive: unchanged, and said

def test_the_lightning_is_unchanged_and_publishes_null(env):
  c = _ctrl(env, car=LIGHTNING)
  _db(c)
  _mapd(c, K_DB)
  assert c._no_accel_curve(SET, 32.0, SET) == SET
  assert c._tele_nc == (None, None) and c.overlay_payload()["ncOn"] is None


def test_the_opt_out_switch_turns_it_off(env, monkeypatch):
  lines = _logs(monkeypatch)
  c = _ctrl(env, {"vtsc_no_accel_curve_off": True})
  _db(c)
  assert c._no_accel_curve(SET, 32.0, SET) == SET and c._tele_nc == ("off", None)
  assert any("no-accel-curve: None -> off" in m for _, m in lines)


def test_a_bad_switch_value_turns_it_off_and_is_an_error(env, monkeypatch):
  lines = _logs(monkeypatch)
  c = _ctrl(env, {"vtsc_no_accel_curve_off": "maybe"})
  assert not c.veh.vtsc_no_accel_curve
  assert any(lvl == "error" and "vtsc_no_accel_curve_off" in m for lvl, m in lines)


def test_by_default_it_is_on_for_the_tesla_only(env):
  assert _ctrl(env).veh.vtsc_no_accel_curve
  assert not _ctrl(env, car=LIGHTNING).veh.vtsc_no_accel_curve


@pytest.mark.parametrize("why, mode, entry_mode", [
  ("brain in shadow, no map", "shadow", "shadow"),
  ("brain off, no map", "off", "lower"),
  ("brain and VTSC disagree, no map", "lower", "shadow"),
])
def test_missing_input_is_inactive_and_logged(env, monkeypatch, why, mode, entry_mode):
  lines = _logs(monkeypatch)
  c = _ctrl(env, {"curve_brain": mode})
  _db(c, mode=entry_mode)
  assert c._no_accel_curve(SET, 32.0, SET) == SET, why
  assert c._tele_nc == ("noInput", None), why
  assert [lvl for lvl, m in lines if "no-accel-curve" in m and "-> noInput" in m] == ["info"], why
  c._no_accel_curve(SET, 32.0, SET)
  assert len([m for _, m in lines if "no-accel-curve" in m]) == 1, why          # change-only: not 20 lines a second


def test_a_db_row_without_a_usable_k_is_missing_input(env, monkeypatch):
  lines = _logs(monkeypatch)
  c = _ctrl(env)
  _db(c, k=None)
  assert c._no_accel_curve(SET, 32.0, SET) == SET and c._tele_nc[0] == "noInput"
  assert any("-> noInput" in m for _, m in lines)


def test_an_exception_is_inactive_and_logged(env, monkeypatch):
  lines = _logs(monkeypatch)
  c = _ctrl(env)

  class Boom(dict):
    def get(self, *a):
      raise RuntimeError("boom")
  c._cb_entry = Boom()
  c._nc_hold = 30.0
  assert c._no_accel_curve(SET, 32.0, SET) == SET
  assert c._tele_nc == ("err", None) and c._nc_hold is None
  assert any(lvl == "exception" and "no-accel-curve check FAILED" in m for lvl, m in lines)


def test_cruise_off_drops_the_hold(env):
  c = _ctrl(env)
  _db(c)
  c._no_accel_curve(SET, 32.0, SET)
  c._drop_latched()
  assert c._nc_hold is None


def test_the_telemetry_keys_reach_ces_events(env):
  assert {"ncOn", "ncHold"} <= set(VTSC_TELE_KEYS)
  c = _ctrl(env)
  _db(c)
  c._no_accel_curve(SET, 32.0, SET)
  pay = json.loads(json.dumps(c.overlay_payload()))
  assert pay["ncOn"] == "hold" and pay["ncHold"] == pytest.approx(32.0)


# ------------------------------------------------------------------------------------------- the 20:09 replay, through the real cap()

def _replay_2009(env_path, cfg):
  """Closed loop from 20:09:01 (the lead left at 72 mph, set 90): the car follows min(cap, set) at most +1.3 m/s^2 (what it did that night).
  The curve DB row (k 0.00175, priced at 101 mph so the brain itself never binds) starts 190 m ahead and comes closer as the car drives."""
  env_path.write_text(json.dumps({"tesla": dict({"curve_brain": "lower"}, **cfg)}))
  c = vc.VTSCController(_CP(*TESLA), params=_Params())
  pos = [0.0]
  c.mem_params = _Mem(lambda t: {"ts": t, "seq": 1, "mode": "lower", "v": 101 * MPH, "d": max(190.0 - pos[0], 0.0), "src": "db",
                                 "ev": "measured", "a": 3.59, "k": K_DB, "row": "4700:0"})
  v, out = 72 * MPH, []
  for i in range(8 * 20):
    _CLOCK[0] = 1000.0 + i / 20.0
    cap = c.cap(_sm(), SET, v)
    out.append((i / 20.0, v, cap))
    v += max(min(min(cap, SET) - v, 1.3), -1.5) / 20.0
    pos[0] += v / 20.0
  return out


def test_replay_2009_the_cap_holds_at_72_mph_instead_of_rising(env):
  run = _replay_2009(env, {})
  at_0902 = [r for r in run if r[0] == pytest.approx(1.0)][0]
  assert at_0902[2] == pytest.approx(72 * MPH, abs=0.05) and at_0902[1] == pytest.approx(72 * MPH, abs=0.05)
  assert max(r[1] for r in run) < 72.2 * MPH             # 20:09:08 was 88 mph; now it stays at 72
  # today's behavior, for contrast (and the proof the scenario has teeth): with the switch off it accelerates as it did that night
  off = _replay_2009(env, {"vtsc_no_accel_curve_off": True})
  assert max(r[1] for r in off) > 85 * MPH
  assert all(r[2] >= SET - 0.05 for r in off)            # neither VTSC nor the brain acted on this curve
  assert C.NO_ACCEL_CURVE_AHEAD_M == 200.0
