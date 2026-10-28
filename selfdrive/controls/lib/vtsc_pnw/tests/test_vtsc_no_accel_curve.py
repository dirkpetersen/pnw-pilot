"""noaccelcurve2pnw (owner-approved 2026-10-26): on the Raven, VTSC does not let the car ACCELERATE toward the set speed while a curve within
NO_ACCEL_CURVE_AHEAD_M (200 m) would be uncomfortable at the SET speed (set^2 x k > A_LAT_TARGET), k = the sharpest of the curve brain's row and
mapd's polyline curvature. The cap is max(v_comf, v_hold): v_comf = sqrt(A_LAT_TARGET / k), the curve's own comfortable speed, and
v_hold = max(the car's speed at the start of the hold, the car's speed now). Lower-only, never below vEgo.

Drive 2026-10-26 20:09 PT (drives/2026-10-26/tesla-i5-olympia-grand-mound): the lead left at 72 mph, the Tesla accelerated at ~1.3 m/s^2 to
88 mph toward set 90 into a left curve (DB k 0.00175, mapd k 0.0015). VTSC and the brain priced it above the set speed, so neither acted.
Drive 2026-10-27 (drives/2026-10-27/tesla-us34-corvallis): the first rule held the car's own speed, so near Corvallis the Tesla stayed at
41 mph in a 55 on bends (k up to 0.0044) that allow 53 mph and more. Now the car may accelerate up to v_comf.

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
K_SHARP = 0.0044         # the sharpest 10-27 Corvallis bend: v_comf = sqrt(2.5 / 0.0044) = 23.8 m/s, below the 32 m/s of the branch tests,
                         #   so those tests see the v_hold part of the cap
DT = 0.05


def _ctrl(env_path, cfg=None, car=TESLA):
  env_path.write_text(json.dumps({"tesla": dict({"curve_brain": "lower"}, **(cfg or {}))}))
  c = vc.VTSCController(_CP(*car), params=_Params())
  c.mem_params = _Mem()
  return c


def _db(c, k=K_SHARP, d=150.0, v=45.0, mode="lower"):
  """Put a fresh curve-brain entry in place, as _apply_brain's _read_brain leaves it."""
  c._cb_entry = {"mode": mode, "v": v, "d": d, "ev": "measured", "src": "db", "row": "1:0", "a": 3.59, "k": k}
  c._cb_age, c._cb_problem = 0.0, None


def _mapd(c, monkeypatch, k, d=150.0, n=3, ahead=True, far=None):
  """mapd's path as polyline_curvature measures it: (k, d) is the sharpest point over a horizon that reaches d; `far` = (k, d) of a sharper
  curve further on, which a longer horizon would return instead (the function gives ONE maximum per horizon)."""
  c._map_targets = [{"latitude": 0.0, "longitude": 0.0}]       # non-empty; the fake below ignores the geometry
  asked = []

  def fake(points, lat, lon, horizon, a_lat, bearing=None):
    asked.append(horizon)
    if far is not None and horizon >= far[1]:
      return far[0], far[1], 0.0, n, True
    return (k, d, 0.0, n, ahead) if horizon >= d else (0.0, 0.0, float("inf"), n, True)
  monkeypatch.setattr(vc, "polyline_curvature", fake)
  return asked


def _logs(monkeypatch):
  lines = []
  for lvl in ("info", "warning", "error", "exception"):
    monkeypatch.setattr(vc.cloudlog, lvl, lambda msg, *a, _l=lvl, **k: lines.append((_l, msg % a if a else msg)))
  return lines


# ------------------------------------------------------------------------------------------- the branch

def test_a_db_curve_uncomfortable_at_the_set_speed_holds_the_cap_at_vego(env):
  c = _ctrl(env)
  _db(c)
  assert c._no_accel_curve(SET, 32.0, SET, DT) == pytest.approx(32.0)
  assert c._tele_nc == ("hold", 32.0)
  # the car slows (a lead): the cap stays at the start speed, it may come back to it but not above it
  assert c._no_accel_curve(SET, 30.0, SET, DT) == pytest.approx(32.0)
  # the car is faster than the start speed (downhill): the cap follows the car, it never brakes it
  assert c._no_accel_curve(SET, 33.0, SET, DT) == pytest.approx(33.0)
  # lower-only: a lower cap from VTSC / the brain passes through
  assert c._no_accel_curve(SET, 33.0, 28.0, DT) == pytest.approx(28.0)


def test_corvallis_the_cap_is_the_curve_s_comfortable_speed_not_the_car_s(env):
  """10-27 Corvallis: 41 mph, set 90, k 0.0044 -> the cap is v_comf = 53 mph, not the 41 mph the car had."""
  c = _ctrl(env)
  _db(c, k=0.0044)
  v_comf = (c.tune["A_LAT_TARGET"] / 0.0044) ** 0.5
  assert v_comf / MPH == pytest.approx(53.3, abs=0.1)
  assert c._no_accel_curve(SET, 41 * MPH, SET, DT) == pytest.approx(v_comf)
  assert c._tele_nc == ("hold", round(v_comf, 2))          # ncHold reports the applied cap


def test_the_comfortable_speed_comes_from_the_sharpest_k(env, monkeypatch):
  c = _ctrl(env)
  _db(c, k=0.0030)
  _mapd(c, monkeypatch, 0.0044)
  assert c._no_accel_curve(SET, 18.0, SET, DT) == pytest.approx((2.5 / 0.0044) ** 0.5)


def test_a_mapd_curve_alone_also_holds(env, monkeypatch):
  c = _ctrl(env, {"curve_brain": "off"})
  _mapd(c, monkeypatch, K_SHARP)
  assert c._no_accel_curve(SET, 32.0, SET, DT) == pytest.approx(32.0) and c._tele_nc[0] == "hold"


def test_the_sharpest_of_db_and_mapd_decides(env, monkeypatch):
  c = _ctrl(env)
  _db(c, k=0.0010)                     # comfortable
  _mapd(c, monkeypatch, K_SHARP)       # uncomfortable
  assert c._no_accel_curve(SET, 32.0, SET, DT) == pytest.approx(32.0)


@pytest.mark.parametrize("why, db, mapd", [
  ("comfortable at the set speed (the 20:09 mapd k)", None, (K_MAPD, 150.0)),
  ("comfortable DB row", (0.0015, 150.0), None),
  ("DB row beyond 200 m", (K_DB, 201.0), None),
  ("mapd curve beyond 200 m", None, (K_DB, 201.0)),
  ("mapd point behind the car", None, (K_DB, 50.0, 3, False)),
  ("the DB has no row ahead", "noNeed", None),
])
def test_no_uncomfortable_curve_ahead_changes_nothing(env, monkeypatch, why, db, mapd):
  c = _ctrl(env)
  if db == "noNeed":
    c._cb_entry, c._cb_age, c._cb_problem = {"mode": "lower", "v": None}, 0.0, "noNeed"
  elif db:
    _db(c, k=db[0], d=db[1])
  if mapd:
    _mapd(c, monkeypatch, *mapd)
  assert c._no_accel_curve(SET, 32.0, SET, DT) == SET, why
  assert c._tele_nc == ("clear", None), why


def test_the_hold_ends_when_the_curve_is_past_and_restarts_fresh(env):
  c = _ctrl(env)
  _db(c)
  c._no_accel_curve(SET, 32.0, SET, DT)
  _db(c, k=0.0005)                     # past the curve: the next row is mild
  assert c._no_accel_curve(SET, 33.0, SET, DT) < SET and c._nc_hold is None     # easing up, not a snap (see the ramp test)
  _db(c)
  assert c._no_accel_curve(SET, 35.0, SET, DT) == pytest.approx(35.0)     # a new hold starts at the speed of then


def test_the_release_eases_up_at_a_relax_from_the_held_cap(env):
  """Gemini 2026-10-27 (1): VTSC's and the brain's own caps sit at the set speed through the hold, so a plain release snapped the cap
  32 -> 40 m/s in one tick. The term now eases up at A_RELAX from the held cap, then stops acting."""
  c = _ctrl(env)
  _db(c)
  c._no_accel_curve(SET, 32.0, SET, DT)
  _db(c, k=0.0005)
  caps = [c._no_accel_curve(SET, 32.0, SET, DT) for _ in range(400)]
  steps = [b - a for a, b in zip([32.0] + caps, caps, strict=False)]
  assert max(steps) <= c.tune["A_RELAX"] * DT + 1e-9
  assert steps[0] == pytest.approx(c.tune["A_RELAX"] * DT)
  assert caps[-1] == SET and c._nc_cap is None and c._tele_nc == ("clear", None)
  assert c._tele_nc[0] == "clear"


def test_the_ramp_payload_names_the_easing_cap(env):
  c = _ctrl(env)
  _db(c)
  c._no_accel_curve(SET, 32.0, SET, DT)
  _db(c, k=0.0005)
  out = c._no_accel_curve(SET, 32.0, SET, DT)
  assert c._tele_nc == ("clear", round(out, 2))


def test_the_ramp_follows_its_own_held_cap_not_a_lower_cap_from_the_brain(env):
  """A brain cap of 22 during the hold must not become the ramp's start: when the brain lets go (kill switch), the cap goes back to the
  hold's own value at once, then eases."""
  c = _ctrl(env)
  _db(c)
  assert c._no_accel_curve(SET, 32.0, 22.0, DT) == 22.0
  _db(c, k=0.0005)
  assert c._no_accel_curve(SET, 32.0, SET, DT) == pytest.approx(32.0 + c.tune["A_RELAX"] * DT)


def test_the_ramp_never_asks_for_less_than_the_car_s_speed(env):
  c = _ctrl(env)
  _db(c)
  c._no_accel_curve(SET, 32.0, SET, DT)
  _db(c, k=0.0005)
  assert c._no_accel_curve(SET, 36.0, SET, DT) == pytest.approx(36.0)     # the driver pressed the accelerator: no brake to the ramp


def test_the_opt_out_switch_stops_the_ramp_at_once(env):
  c = _ctrl(env)
  _db(c)
  c._no_accel_curve(SET, 32.0, SET, DT)
  c.veh._tesla_curve_cfg["no_accel_curve"] = False
  assert c._no_accel_curve(SET, 32.0, SET, DT) == SET and c._nc_cap is None


def test_a_lower_cap_from_vtsc_ends_the_ramp(env):
  c = _ctrl(env)
  _db(c)
  c._no_accel_curve(SET, 32.0, SET, DT)
  _db(c, k=0.0005)
  assert c._no_accel_curve(SET, 32.0, 30.0, DT) == 30.0 and c._nc_cap is None


def test_a_sharper_far_curve_does_not_hide_a_near_one(env, monkeypatch):
  """Gemini 2026-10-27 (2): the ~500 m mapK maximum (k 0.0025 at 350 m) hid an uncomfortable curve at 120 m (k 0.0020: 3.24 m/s^2 at 90 mph).
  The feature measures the polyline over its own 200 m window."""
  c = _ctrl(env, {"curve_brain": "off"})
  asked = _mapd(c, monkeypatch, 0.0020, d=120.0, far=(0.0025, 350.0))
  # v_comf of the NEAR curve (35.4 m/s); the far one would give 31.6, so the cap would be the car's 32
  assert c._no_accel_curve(SET, 32.0, SET, DT) == pytest.approx((2.5 / 0.0020) ** 0.5) and c._tele_nc[0] == "hold"
  assert asked == [C.NO_ACCEL_CURVE_AHEAD_M]


def test_a_sharp_curve_beyond_200_m_with_a_mild_near_one_does_not_hold(env, monkeypatch):
  c = _ctrl(env, {"curve_brain": "off"})
  _mapd(c, monkeypatch, 0.0010, d=120.0, far=(0.0025, 350.0))
  assert c._no_accel_curve(SET, 32.0, SET, DT) == SET and c._tele_nc == ("clear", None)


def test_an_unmeasurable_mapd_path_is_missing_input_and_logged(env, monkeypatch):
  lines = _logs(monkeypatch)
  c = _ctrl(env, {"curve_brain": "off"})
  _mapd(c, monkeypatch, 0.0, n=0)
  assert c._no_accel_curve(SET, 32.0, SET, DT) == SET and c._tele_nc == ("noInput", None)
  assert any("-> noInput" in m and "mapd triplets in 200.0 m: 0" in m for _, m in lines)


def test_a_non_number_speed_is_an_error_and_never_raises(env, monkeypatch):
  """Gemini 2026-10-27 (3): the state-change log line is outside the try; it must not raise out of cap() on a non-number."""
  lines = _logs(monkeypatch)
  c = _ctrl(env)
  _db(c)
  assert c._no_accel_curve(None, 32.0, SET, DT) == SET and c._tele_nc == ("err", None)
  assert any(lvl == "exception" for lvl, _ in lines) and any("-> err" in m for _, m in lines)


def test_the_dead_reckoned_distance_of_an_aged_entry_is_used(env):
  c = _ctrl(env)
  _db(c, d=230.0)
  c._cb_age = 1.0                      # 230 - 32 x 1.0 = 198 m now
  assert c._no_accel_curve(SET, 32.0, SET, DT) == pytest.approx(32.0)


# ------------------------------------------------------------------------------------------- inactive: unchanged, and said

def test_the_lightning_is_unchanged_and_publishes_null(env, monkeypatch):
  c = _ctrl(env, car=LIGHTNING)
  _db(c)
  _mapd(c, monkeypatch, K_DB)
  assert c._no_accel_curve(SET, 32.0, SET, DT) == SET
  assert c._tele_nc == (None, None) and c.overlay_payload()["ncOn"] is None


def test_the_opt_out_switch_turns_it_off(env, monkeypatch):
  lines = _logs(monkeypatch)
  c = _ctrl(env, {"vtsc_no_accel_curve_off": True})
  _db(c)
  assert c._no_accel_curve(SET, 32.0, SET, DT) == SET and c._tele_nc == ("off", None)
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
  assert c._no_accel_curve(SET, 32.0, SET, DT) == SET, why
  assert c._tele_nc == ("noInput", None), why
  assert [lvl for lvl, m in lines if "no-accel-curve" in m and "-> noInput" in m] == ["info"], why
  c._no_accel_curve(SET, 32.0, SET, DT)
  assert len([m for _, m in lines if "no-accel-curve" in m]) == 1, why          # change-only: not 20 lines a second


def test_a_db_row_without_a_usable_k_is_missing_input(env, monkeypatch):
  lines = _logs(monkeypatch)
  c = _ctrl(env)
  _db(c, k=None)
  assert c._no_accel_curve(SET, 32.0, SET, DT) == SET and c._tele_nc[0] == "noInput"
  assert any("-> noInput" in m for _, m in lines)


def test_an_exception_is_inactive_and_logged(env, monkeypatch):
  lines = _logs(monkeypatch)
  c = _ctrl(env)

  class Boom(dict):
    def get(self, *a):
      raise RuntimeError("boom")
  c._cb_entry = Boom()
  c._nc_hold = 30.0
  assert c._no_accel_curve(SET, 32.0, SET, DT) == SET
  assert c._tele_nc == ("err", None) and c._nc_hold is None
  assert any(lvl == "exception" and "no-accel-curve check FAILED" in m for lvl, m in lines)


def test_cruise_off_drops_the_hold(env):
  c = _ctrl(env)
  _db(c)
  c._no_accel_curve(SET, 32.0, SET, DT)
  c._drop_latched()
  assert c._nc_hold is None


def test_the_telemetry_keys_reach_ces_events(env):
  assert {"ncOn", "ncHold"} <= set(VTSC_TELE_KEYS)
  c = _ctrl(env)
  _db(c)
  c._no_accel_curve(SET, 32.0, SET, DT)
  pay = json.loads(json.dumps(c.overlay_payload()))
  assert pay["ncOn"] == "hold" and pay["ncHold"] == pytest.approx(32.0)


# ------------------------------------------------------------------------------------------- the 20:09 replay, through the real cap()

def _replay_2009(env_path, cfg, secs=8, past_m=None):
  """Closed loop from 20:09:01 (the lead left at 72 mph, set 90): the car follows min(cap, set) at most +1.3 m/s^2 (what it did that night).
  The curve DB row (k 0.00175, priced at 101 mph so the brain itself never binds) starts 190 m ahead and comes closer as the car drives."""
  env_path.write_text(json.dumps({"tesla": dict({"curve_brain": "lower"}, **cfg)}))
  c = vc.VTSCController(_CP(*TESLA), params=_Params())
  pos = [0.0]
  c.mem_params = _Mem(lambda t: {"ts": t, "seq": 1, "mode": "lower", "v": None} if past_m is not None and pos[0] > past_m else
                      {"ts": t, "seq": 1, "mode": "lower", "v": 101 * MPH, "d": max(190.0 - pos[0], 0.0), "src": "db",
                       "ev": "measured", "a": 3.59, "k": K_DB, "row": "4700:0"})
  v, out = 72 * MPH, []
  for i in range(secs * 20):
    _CLOCK[0] = 1000.0 + i / 20.0
    cap = c.cap(_sm(), SET, v)
    out.append((i / 20.0, v, cap))
    v += max(min(min(cap, SET) - v, 1.3), -1.5) / 20.0
    pos[0] += v / 20.0
  return out, c


def test_replay_2009_the_cap_rises_to_the_curve_s_comfortable_speed_and_no_higher(env):
  v_comf = (2.5 / K_DB) ** 0.5                            # 37.8 m/s = 84.6 mph
  assert v_comf / MPH == pytest.approx(84.6, abs=0.1)
  run, _ = _replay_2009(env, {}, secs=10)
  assert run[0][2] == pytest.approx(v_comf, abs=0.01)    # at once the cap is v_comf, not the car's 72 mph
  assert max(r[2] for r in run) < v_comf + 0.01
  assert max(r[1] for r in run) == pytest.approx(v_comf, abs=0.05)   # the car accelerates up to v_comf (20:09:08 was 88 mph)
  # today's behavior, for contrast (and the proof the scenario has teeth): with the switch off it accelerates as it did that night
  off, _ = _replay_2009(env, {"vtsc_no_accel_curve_off": True})
  assert max(r[1] for r in off) > 85 * MPH
  assert all(r[2] >= SET - 0.05 for r in off)            # neither VTSC nor the brain acted on this curve
  assert C.NO_ACCEL_CURVE_AHEAD_M == 200.0


def test_replay_2009_after_the_curve_the_cap_eases_up_at_a_relax(env):
  run, c = _replay_2009(env, {}, secs=20, past_m=340.0)            # the row's extent ends ~150 m past its anchor 190 m ahead
  caps = [r[2] for r in run]
  release = next(i for i, r in enumerate(caps) if r > (2.5 / K_DB) ** 0.5 + 0.01)
  assert 8.0 < run[release][0] < 11.0                           # 340 m at 72 -> 84.6 mph
  assert max(b - a for a, b in zip(caps, caps[1:], strict=False)) <= c.tune["A_RELAX"] * DT + 1e-6       # never a snap to the set speed
  assert caps[-1] > caps[release]
