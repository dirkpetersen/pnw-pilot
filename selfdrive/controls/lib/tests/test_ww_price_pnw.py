"""wheelweight2pnw stage 2: curves toward a detected wheel weight are priced so the angle they need fits under the Raven's car-side
angle clamp after the learned offset (selfdrive/controls/lib/ww_price_pnw.py), lower-only, in the curve brain (DB rows) and in
VTSC (mapd curves). Owner numbers 2026-10-26 (computed on the device with this method, no margin): k 0.0018 / offset 4.1 -> 81.2 mph,
k 0.0017 -> about 83, offset 2.3 -> about 88.7."""
import json
import math
import types

import pytest

from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib import ww_price_pnw as W
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw import curve_brain as cb
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import VTSC_TELE_KEYS
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as vc
from openpilot.selfdrive.controls.lib.vtsc_pnw.tests.test_curvebrain_vtsc_pnw import _CP, _Mem, _Params, _sm, _CLOCK, LIGHTNING, TESLA
from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_pnw import ww_map_need
from openpilot.selfdrive.selfdrived.wheel_weight_pnw import WheelWeight

MPH = 0.44704
VM = W.raven_clamp_vm()
LAT0, LON0 = 47.0, -122.9


# ---------------------------------------------------------------------------------------------------------------- the price
@pytest.mark.parametrize("k,off,mph", [(0.0018, 4.1, 81.2), (0.0017, 4.1, 83.0), (0.0018, 2.3, 88.75)])
def test_owner_numbers_without_margin(k, off, mph):
  assert W.offset_speed(VM, k, off, margin=0.0) / MPH == pytest.approx(mph, abs=0.15)


@pytest.mark.parametrize("k,off,mph", [(0.0018, 4.1, 79.9), (0.0017, 4.1, 81.7), (0.0018, 2.3, 87.4)])
def test_with_the_owner_margin_of_0_1(k, off, mph):
  assert W.offset_speed(VM, k, off) / MPH == pytest.approx(mph, abs=0.15)


def test_the_price_leaves_exactly_the_margin_under_the_clamp():
  from opendbc.car.lateral import get_max_angle_vm
  v = W.offset_speed(VM, 0.0018, 4.1)
  need = math.degrees(VM.get_steer_from_curvature(0.0018, v, 0)) + 4.1
  margin = math.degrees(VM.get_steer_from_curvature(0.1 / v ** 2, v, 0))
  assert need == pytest.approx(get_max_angle_vm(v, VM, W.RAVEN_LIMITS) - margin, abs=0.01)


def test_the_offset_sign_does_not_matter_and_a_larger_offset_is_slower():
  assert W.offset_speed(VM, 0.0018, -4.1) == W.offset_speed(VM, 0.0018, 4.1)
  assert W.offset_speed(VM, 0.0018, 5.0) < W.offset_speed(VM, 0.0018, 4.1) < W.offset_speed(VM, 0.0018, 0.0)


def test_bounds_and_bad_input():
  assert W.offset_speed(VM, 1e-5, 4.1) == W.V_HI
  assert W.offset_speed(VM, 0.05, 4.1) == W.V_LO
  for k, off in [(0.0, 4.1), (-0.001, 4.1), (float("nan"), 4.1), (0.0018, float("nan"))]:
    assert W.offset_speed(VM, k, off) is None


def test_turn_side():
  assert W.turn_side(7.0) == "right" and W.turn_side(-7.0) == "left"
  assert W.turn_side(0.5) is None and W.turn_side(float("nan")) is None


# ---------------------------------------------------------------------------------------------------------------- curve brain (DB rows)
def _ll(x, y):
  return LAT0 + y / 111320.0, LON0 + x / (111320.0 * math.cos(math.radians(LAT0)))


class _Idx:
  """One anchor heading north at y = 0; its branch ends 150 m on, `x` m to the east (negative = a left bend)."""
  back = 25.0

  def __init__(self, x):
    self.anchors = [[*_ll(0.0, 0.0), 0.0, [[*_ll(x, 150.0), 0.0018, 3]]]]


class _Row:
  anchor, branch, row_id, lat, lon = 0, 0, "0:0", LAT0, LON0

  def __init__(self, k=0.0018, s_anchor=300.0):
    self.k, self.s_anchor = k, s_anchor


class _Veh:
  def curve_lat_a(self, v):
    return 3.5886


V_BASE = math.sqrt(3.5886 / 0.0018)    # 44.65 m/s = 99.9 mph: what the brain asks without a weight


def _need(x, ww):
  need, n = cb.most_binding_row(_Idx(x), [_Row()], 0.0, _Veh(), 40.0, 1.2, ww=ww)
  assert n == 1
  return need


def test_brain_left_curve_with_the_weight_left_is_offset_priced():
  need = _need(-20.0, ("left", 4.1, VM))
  assert need["v"] == pytest.approx(W.offset_speed(VM, 0.0018, 4.1)) and need["v"] / MPH == pytest.approx(79.9, abs=0.15)
  assert need["ww_use"] is True and need["ww_dir"] == "left" and need["ww_price"] == need["v"]


def test_brain_right_curve_with_the_weight_left_is_unchanged():
  need = _need(20.0, ("left", 4.1, VM))
  assert need["v"] == pytest.approx(V_BASE) and need["ww_use"] is False and need["ww_dir"] == "right" and need["ww_price"] is None


def test_brain_mirror_weight_right_prices_the_right_curve():
  assert _need(20.0, ("right", 4.1, VM))["v"] == pytest.approx(W.offset_speed(VM, 0.0018, 4.1))


def test_brain_no_weight_is_unchanged():
  need = _need(-20.0, None)
  assert need["v"] == pytest.approx(V_BASE) and need["ww_dir"] is None and need["ww_use"] is False


def test_brain_unknown_bend_side_is_not_offset_priced_and_counted():
  need = _need(0.0, ("left", 4.1, VM))
  assert need["v"] == pytest.approx(V_BASE) and need["ww_dir"] is None and need["ww_unknown"] == 1


def test_brain_offset_price_is_lower_only():
  """A row the brain already prices below the offset price keeps its own speed."""
  need, _ = cb.most_binding_row(_Idx(-20.0), [_Row(k=0.0018)], 0.0, types.SimpleNamespace(curve_lat_a=lambda v: 1.0), 40.0, 1.2,
                                ww=("left", 4.1, VM))
  assert need["v"] == pytest.approx(math.sqrt(1.0 / 0.0018)) and need["ww_use"] is False


def test_the_payload_carries_the_weight_for_vtsc_and_parse_ww_reads_it():
  b = cb.CurveBrain.__new__(cb.CurveBrain)
  b.veh = types.SimpleNamespace(refresh_curve_brain_cfg=lambda now: None, curve_brain="off")
  b.overrides = types.SimpleNamespace(refresh=lambda now: None)
  b._seq, b._ww_unknown_logged = 0, None
  b.db = types.SimpleNamespace(way_sel_held=lambda w, n: (w,))
  out = b.step(100.0, v_ego=40.0, points=[], plat=None, plon=None, gps_state=None, way_sel=None, hwy=None, a_decel=1.2,
               ww=("left", 4.123, VM))
  assert out["ww"] == ["left", 4.12]
  assert cb.parse_ww(json.dumps(out), 100.1) == (("left", 4.12), "ok")
  assert "ww" not in b.step(100.0, v_ego=40.0, points=[], plat=None, plon=None, gps_state=None, way_sel=None, hwy=None, a_decel=1.2)


@pytest.mark.parametrize("raw,why", [(None, "absent"), ({"ts": 100.0}, "absent"), ({"ts": 90.0, "ww": ["left", 4.1]}, "stale"),
                                     ({"ts": 100.0, "ww": ["up", 4.1]}, "bad"), ({"ts": 100.0, "ww": ["left", float("nan")]}, "bad"),
                                     ({"ts": 100.0, "ww": ["left", True]}, "bad"), ("{not json", "bad")])
def test_parse_ww_refuses_what_it_cannot_use(raw, why):
  assert cb.parse_ww(raw, 100.0) == (None, why)


def test_ces_controller_hands_the_brain_the_detector_state():
  det = WheelWeight()
  ctl = types.SimpleNamespace(wheel_weight=det, _ww_vm=VM)
  for side, off, want in [("unknown", None, None), ("none", 4.1, None), ("left", 4.1, ("left", 4.1, VM)), ("right", -4.1, ("right", -4.1, VM))]:
    det.side, det.off = side, off
    assert m._ww_pricing(ctl) == want
  assert m._ww_pricing(types.SimpleNamespace(wheel_weight=det, _ww_vm=None)) is None   # a car without the capability


def test_capability_tesla_only():
  from opendbc.car import structs
  def cp(brand, fp):
    c = structs.CarParams.new_message()
    c.brand, c.carFingerprint = brand, fp
    return c
  assert pv.PnwVehicle(cp(*TESLA[::-1])).wheel_weight_detect and not pv.PnwVehicle(cp(*LIGHTNING[::-1])).wheel_weight_detect


# ---------------------------------------------------------------------------------------------------------------- VTSC (mapd curves)
def _arc(k_signed, start=50.0, length=400.0, step=50.0):
  """mapd-style path due north from the car: straight to `start` m, then a constant-curvature bend (k < 0 = left)."""
  pts, x, y, brg, s = [], 0.0, -100.0, 0.0, -100.0
  ds = 1.0
  next_pt = s
  while s <= start + length:
    if s >= next_pt - 1e-6:
      la, lo = _ll(x, y)
      pts.append({"latitude": la, "longitude": lo, "velocity": 0.0})
      next_pt += step
    if s >= start:
      brg += math.degrees(k_signed * ds)
    x += math.sin(math.radians(brg)) * ds
    y += math.cos(math.radians(brg)) * ds
    s += ds
  return pts


def test_map_need_left_bend_with_the_weight_left():
  price = lambda k: W.offset_speed(VM, k, 4.1)   # noqa: E731
  need = ww_map_need(_arc(-0.0018), LAT0, LON0, 0.0, 500.0, "left", price, 40.0, 0.8)
  assert need is not None and need[0] / MPH == pytest.approx(79.9, abs=0.5)    # the polyline measures k within a few %


def test_map_need_ignores_the_bend_away_from_the_weight_and_behind_the_car():
  price = lambda k: W.offset_speed(VM, k, 4.1)   # noqa: E731
  assert ww_map_need(_arc(-0.0018), LAT0, LON0, 0.0, 500.0, "right", price, 40.0, 0.8) is None
  assert ww_map_need(_arc(0.0018), LAT0, LON0, 0.0, 500.0, "right", price, 40.0, 0.8) is not None
  assert ww_map_need(_arc(-0.0018), LAT0, LON0, 180.0, 500.0, "left", price, 40.0, 0.8) is None   # heading the other way
  assert ww_map_need(_arc(-0.0018), LAT0, LON0, None, 500.0, "left", price, 40.0, 0.8) is None    # no bearing: nothing is ahead
  assert ww_map_need(_arc(0.0, length=400.0), LAT0, LON0, 0.0, 500.0, "left", price, 40.0, 0.8) is None   # straight


class _MapParams(_Params):
  def get_bool(self, k):
    return k == "VtscMapCurves"


class _MapMem(_Mem):
  def __init__(self, entry, points):
    super().__init__(entry)
    self.points = points

  def get(self, k, return_default=False):
    if k == "MapTargetVelocities":
      return self.points
    if k == "LastGPSPosition":
      return {"latitude": LAT0, "longitude": LON0, "bearing": 0.0}
    return super().get(k, return_default)


V_SET = 40.0   # 89.5 mph


def _vtsc(env, points, ww, car=TESLA, mode="lower", ticks=200):
  env.write_text(json.dumps({"tesla": {"curve_brain": mode}}))
  _CLOCK[0] = 1000.0
  c = vc.VTSCController(_CP(*car), params=_MapParams())
  c.mem_params = _MapMem(lambda t: {"ts": t, "seq": 1, "mode": mode, "v": None, **({"ww": ww} if ww else {})}, points)
  out = []
  for i in range(ticks):
    _CLOCK[0] = 1000.0 + i / 20.0
    out.append((c.cap(_sm(), V_SET, V_SET), c.overlay_payload()))
  return out


@pytest.fixture
def env(monkeypatch, tmp_path):
  monkeypatch.setattr(vc, "time", types.SimpleNamespace(monotonic=lambda: _CLOCK[0]))
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "curve.json"))
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))
  return tmp_path / "curve.json"


def test_vtsc_lowers_the_cap_for_a_left_bend_with_the_weight_left(env):
  base = [c for c, _ in _vtsc(env, _arc(-0.0018), None)]
  run = _vtsc(env, _arc(-0.0018), ["left", 4.1])
  assert all(c <= b + 1e-9 for (c, _), b in zip(run, base, strict=True))           # lower-only
  assert min(c for c, _ in run) < min(base) - 1.0
  last = run[-1][1]
  assert last["wwMapDir"] == "left" and last["wwMapPrice"] / MPH == pytest.approx(79.9, abs=0.5) and last["wwMapUse"] is True


@pytest.mark.parametrize("pts,ww,car,mode", [
  (_arc(0.0018), ["left", 4.1], TESLA, "lower"),     # bend away from the weight
  (_arc(-0.0018), None, TESLA, "lower"),             # detector none / unknown: no "ww" in the entry
  (_arc(-0.0018), ["left", 4.1], TESLA, "off"),      # the brain's switch is this term's switch
  (_arc(-0.0018), ["left", 4.1], LIGHTNING, "lower"),  # the Lightning never runs it
])
def test_vtsc_unchanged(env, pts, ww, car, mode):
  base = [c for c, _ in _vtsc(env, pts, None, car=car, mode=mode)]
  run = _vtsc(env, pts, ww, car=car, mode=mode)
  assert [c for c, _ in run] == base
  assert run[-1][1]["wwMapUse"] in (None, False)


def test_vtsc_ww_keys_reach_ces_events():
  assert {"wwMapPrice", "wwMapDir", "wwMapUse"} <= set(VTSC_TELE_KEYS)
  assert {"wwPrice", "wwDir", "wwUse"} <= set(cb.TELE_KEYS)


def test_brain_ww_fields_reach_the_ces_tick_record(monkeypatch):
  from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_parkgate2pnw import _publish_controller, _run_publish
  b = cb.CurveBrain.__new__(cb.CurveBrain)
  b.veh = types.SimpleNamespace(curve_brain="lower", curve_brain_why="ok")
  b.db = types.SimpleNamespace(state="ok", err=None, index=None)
  b.overrides = types.SimpleNamespace(entries=[])
  b._last_t = 1000.0
  b._last = {"cbWhy": "ok", "v": 35.71, "d": 100.0, "src": "db", "ev": "measured", "a": 3.59, "k": 0.0018, "row": "0:0", "cbN": 1,
             "seq": 1, "cbOvr": None, "cbCov": None, "cbErr": None, "wwPrice": 35.71, "wwDir": "left", "wwUse": True}
  c = _publish_controller(monkeypatch)
  c._curve_brain = b
  caps = _run_publish(monkeypatch, c, 1, g="drive", v_ego=40.0, t0=1000.0)
  assert caps and (caps[0]["wwPrice"], caps[0]["wwDir"], caps[0]["wwUse"]) == (35.71, "left", True)


def test_vtsc_ww_cap_does_not_survive_cruise_off(env):
  """Gemini 2026-10-27: the term's rate-limited cap must be dropped while the driver drives (cruise off), like the brain's and the
  no-accel hold's, so a re-engage after the curve does not apply a stale low cap."""
  env.write_text(json.dumps({"tesla": {"curve_brain": "lower"}}))
  _CLOCK[0] = 1000.0
  c = vc.VTSCController(_CP(*TESLA), params=_MapParams())
  mem = _MapMem(lambda t: {"ts": t, "seq": 1, "mode": "lower", "v": None, "ww": ["left", 4.1]}, _arc(-0.0018))
  c.mem_params = mem
  i = 0

  def tick(enabled):
    nonlocal i
    _CLOCK[0] = 1000.0 + i / 20.0
    i += 1
    sm = _sm()
    sm["carControl"].enabled = enabled
    return c.cap(sm, V_SET, V_SET)

  held = [tick(True) for _ in range(200)][-1]
  assert held < V_SET - 1.0 and c._ww_applied < V_SET - 1.0          # the term is holding a low cap
  mem.points = _arc(0.0)                                              # the driver drives the curve by hand; the road is straight now
  for _ in range(20):
    tick(False)
  assert tick(True) == pytest.approx(V_SET)                           # re-engage: no stale cap
