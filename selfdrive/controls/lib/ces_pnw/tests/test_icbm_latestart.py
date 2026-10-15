"""icbmlatestart2pnw -- a large-drop curve on an open road starts its slowdown LATE (envelope only), never by the tracking window.

Owner 2026-10-15: "the roundabout should slow later and arrive at an appropriate speed, not crawl". Root cause: the
icbmtrack2pnw tracking-window START rule fires on an open road too, ~50-60 m before the 0.8 m/s^2 + 30 m envelope would, and
the executor then jumps the set to the apex. The truck cannot slow faster than ~0.8-0.9 m/s^2 through set taps, so the fix
is WHEN the cap starts, not how fast it walks. Synthetic geometry only (no private coordinates).
"""
import inspect

import pytest

from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import (
  ICBM_LATE_BOUND_MS, ICBM_LATE_LEAD_WAIT_S, icbm_curve_target, icbm_late_start_gate, icbm_map_eff_scale,
  icbm_track_window_m, upcoming_curve)
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvelead2pnw import FakeCP, _controller, _lightning, _scene
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle
from opendbc.car.ford.icbm_pnw import IcbmCommand, PressGovernor, STEP_MS, arbitrate, decide_press

MPH = 0.44704
LAT0, LON0 = 47.0, -122.0
LATE = 18.0 * MPH            # the default knob, m/s


@pytest.fixture
def default_curve_cfg(tmp_path, monkeypatch):
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "absent.json"))
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))


def _far(v_ego, ref, far_v, far_dist, firm=0.0, late=0.0, track=True):
  return icbm_curve_target(v_ego, ref, 0.0, float("inf"), None, icbm_map_eff_scale, far_v=far_v, far_dist=far_dist,
                           firm_decel=firm, track=track, late_drop=late)[0]


def _near(v_ego, ref, eff_v, dist, late=0.0, track=True):
  """map candidate: scale_fn is the identity so `eff_v` is the effective apex."""
  return icbm_curve_target(v_ego, ref, eff_v, dist, None, lambda v: 1.0, track=track, late_drop=late)[0]


# ---- 1 / 2: the pure function --------------------------------------------------------------------------------------
class TestPureFunction:
  V, REF, EFF = 20.2, 20.6, 6.9          # the roundabout geometry from the design

  def test_far_candidate_waits_for_the_envelope(self):
    assert icbm_track_window_m(self.V, self.REF, self.EFF) > 300.0, "premise: the tracking window reaches 300 m"
    assert _far(self.V, self.REF, self.EFF, 300.0, late=0.0) == pytest.approx(self.EFF), "regression: today starts at 300 m"
    assert _far(self.V, self.REF, self.EFF, 300.0, late=LATE) is None, "late start must NOT start at 300 m"
    assert _far(self.V, self.REF, self.EFF, 250.0, late=LATE) == pytest.approx(self.EFF), "envelope start at 250 m"
    assert _far(self.V, self.REF, self.EFF, 250.0, late=0.0) == pytest.approx(self.EFF)

  def test_map_candidate_waits_for_the_envelope(self):
    assert _near(self.V, self.REF, self.EFF, 300.0) == pytest.approx(self.EFF)
    assert _near(self.V, self.REF, self.EFF, 300.0, late=LATE) is None
    assert _near(self.V, self.REF, self.EFF, 250.0, late=LATE) == pytest.approx(self.EFF)

  def test_default_zero_is_byte_identical_over_a_grid(self):
    for v in (10.0, 20.2, 31.0):
      for drop in (1.5, 5.0, 9.0, 14.0):
        for d in range(20, 520, 10):
          for track in (False, True):
            a = icbm_curve_target(v, v + 0.4, 0.0, float("inf"), None, icbm_map_eff_scale,
                                  far_v=v - drop, far_dist=float(d), firm_decel=1.4, track=track)
            b = icbm_curve_target(v, v + 0.4, 0.0, float("inf"), None, icbm_map_eff_scale,
                                  far_v=v - drop, far_dist=float(d), firm_decel=1.4, track=track, late_drop=0.0)
            assert a == b

  def test_drops_under_the_threshold_change_nothing(self):
    v, ref = 20.2, 20.6
    for drop_mph in (3.0, 8.0, 12.0, 14.8, 17.5):
      eff = ref - drop_mph * MPH
      for d in range(20, 400, 5):
        for fn in (_far, _near):
          assert fn(v, ref, eff, float(d), late=LATE) == fn(v, ref, eff, float(d), late=0.0), (drop_mph, d, fn.__name__)
    # and at least one of those really is a tracking start (the grid is not vacuous)
    eff = ref - 14.8 * MPH
    assert any(_far(v, ref, eff, float(d), track=False) is None and _far(v, ref, eff, float(d), late=LATE) is not None
               for d in range(20, 400, 5))

  def test_the_threshold_is_on_the_drop_from_the_ceiling(self):
    v, ref = 20.2, 20.6

    def tracking_only(eff):      # distances only the tracking window (not the envelope) reaches
      return [float(d) for d in range(20, 500, 5)
              if _far(v, ref, eff, float(d), track=False) is None and _far(v, ref, eff, float(d)) is not None]
    under, over = tracking_only(ref - (LATE - 0.05)), tracking_only(ref - (LATE + 0.05))
    assert under and over, "premise: both drops have a tracking-only band"
    assert all(_far(v, ref, ref - (LATE - 0.05), d, late=LATE) is not None for d in under)
    assert all(_far(v, ref, ref - (LATE + 0.05), d, late=LATE) is None for d in over)


class TestGate:
  def test_ok_only_on_a_start_tick_open_road_at_set_speed(self):
    ref = 20.0
    assert icbm_late_start_gate(True, None, ref, ref) == (True, None)
    assert icbm_late_start_gate(True, ICBM_LATE_LEAD_WAIT_S + 0.1, ref, ref) == (True, None)
    assert icbm_late_start_gate(False, None, ref, ref) == (False, "running")
    assert icbm_late_start_gate(True, 0.0, ref, ref) == (False, "lead")
    assert icbm_late_start_gate(True, ICBM_LATE_LEAD_WAIT_S - 0.1, ref, ref) == (False, "lead")
    assert icbm_late_start_gate(True, None, ref - ICBM_LATE_BOUND_MS - 0.01, ref) == (False, "bound")
    assert icbm_late_start_gate(True, None, ref - ICBM_LATE_BOUND_MS + 0.01, ref) == (True, None)
    assert icbm_late_start_gate(True, None, float("nan"), ref) == (False, "bound")
    assert icbm_late_start_gate(True, None, "x", ref) == (False, "bound")


# ---- step-level harness --------------------------------------------------------------------------------------------
def _veh(knob_on=True):
  v = _lightning()
  if not knob_on:
    v._curve_cfg["icbm_late_start_drop_mph"] = 0.0
  return v


def _sig(c, d, v, v_set, lead=False, map_v=6.8):
  pts = _scene(d, 60.0, map_v)
  c._map_targets = pts
  mtv, mtd = upcoming_curve(pts, LAT0, LON0, v, 10.0)
  return {"v_ego": v, "v_set": v_set, "map_target_v": mtv, "map_target_dist": mtd, "curve_lat_accel_vision": 0.0,
          "time_to_curve": float("inf"), "lat_accel_now": 0.0, "has_lead": lead, "lead_drel": 50.0, "lead_vlead": 20.0,
          "gas": False, "brake": False, "spd_lim": 0.0, "pitch": None}


def _tick(c, step, clock, d, v, v_set, lead=False, map_v=6.8):
  n = len(c.mem_params.log)
  step(_sig(c, d, v, v_set, lead, map_v), active=True)
  new = c.mem_params.log[n:]
  clock[0] += 0.25
  return new[-1] if new else None


def _one(monkeypatch, d, v=20.2, v_set=20.6, knob_on=True, lead_hist=(), map_v=6.8):
  """One tick at distance d; lead_hist = [(seconds_before, has_lead), ...] earlier ticks on the same controller."""
  clock = [1000.0]
  c, step = _controller(monkeypatch, clock, _veh(knob_on))
  for _ in lead_hist:
    _tick(c, step, clock, 600.0, v, v_set, lead=True, map_v=map_v)       # nothing to do at 600 m, but the lead is SEEN
  if lead_hist:
    clock[0] += lead_hist[0]
  pub = _tick(c, step, clock, d, v, v_set, lead=False if not lead_hist or lead_hist[0] else True, map_v=map_v)
  return pub, c


def _start_distance(monkeypatch, **kw):
  for d in range(500, 100, -5):
    pub, c = _one(monkeypatch, float(d), **kw)
    if pub and pub.get("target") is not None:
      return d, c
  return None, None


# ---- 3: the 51 -> 14 mph geometry, firm-ramp vs envelope -----------------------------------------------------------
class TestFirmRampIsOffOnLateTicks:
  V = 51 * MPH
  EFF = 14 * MPH

  def test_far_scan_and_start_use_the_comfort_envelope(self, monkeypatch):
    # pure: at 310 m the 0.8 envelope (~330 m) has started but the firm ramp (~286 m) has not
    assert _far(self.V, self.V, self.EFF, 310.0, firm=0.0, late=LATE) is not None
    assert _far(self.V, self.V, self.EFF, 310.0, firm=1.4, late=LATE) is None, "premise: firm would start later"
    recorded = []
    real = m.icbm_far_map_candidate

    def spy(*a, **kw):
      recorded.append(a[7] if len(a) > 7 else kw.get("firm_decel"))
      return real(*a, **kw)
    monkeypatch.setattr(m, "icbm_far_map_candidate", spy)
    pub, c = _one(monkeypatch, 310.0, v=self.V, v_set=self.V, map_v=self.EFF / (1.1 * 0.92))
    assert pub and pub.get("target") is not None, "late tick must start at 310 m on the 0.8 envelope"
    assert recorded and set(recorded) == {0.0}, f"far scan ran with firm={recorded} on a late tick"
    assert c._icbm_start_why == "env"

  def test_without_the_knob_the_firm_ramp_still_applies(self, monkeypatch):
    recorded = []
    real = m.icbm_far_map_candidate
    monkeypatch.setattr(m, "icbm_far_map_candidate",
                        lambda *a, **kw: (recorded.append(a[7] if len(a) > 7 else kw.get("firm_decel")), real(*a, **kw))[1])
    _one(monkeypatch, 310.0, v=self.V, v_set=self.V, knob_on=False, map_v=self.EFF / (1.1 * 0.92))
    assert set(recorded) == {1.4}


# ---- 4: step gating -------------------------------------------------------------------------------------------------
class TestStepGating:
  D = 300.0          # between the 0.8 envelope (~255 m) and the tracking window (~335 m) for the 20.2 -> 6.8 geometry

  def test_open_road_is_withheld_with_the_knob_on_and_starts_with_it_off(self, monkeypatch):
    on, c_on = _one(monkeypatch, self.D)
    off, c_off = _one(monkeypatch, self.D, knob_on=False)
    assert on == {} or on.get("target") is None
    assert off and off.get("target") is not None
    assert c_on._icbm_start_why == "lateWait" and c_off._icbm_start_why == "track"

  @pytest.mark.parametrize("name,kw,why", [
    ("lead now", dict(lead_hist=(0.0,)), "trackLead"),
    ("lead 3 s ago", dict(lead_hist=(3.0,)), "trackLead"),
    ("lead-bound", dict(v=20.6 - 1.5 * MPH - 0.2), "trackBound"),
  ])
  def test_lead_and_lead_bound_starts_are_byte_identical_to_today(self, monkeypatch, name, kw, why):
    on, c_on = _one(monkeypatch, self.D, **kw)
    off, c_off = _one(monkeypatch, self.D, knob_on=False, **kw)
    assert off and off.get("target") is not None, f"{name}: premise (today starts here)"
    assert on == off, f"{name}: publish differs from today"
    assert c_on._icbm_start_why == why

  def test_a_lead_seen_long_enough_ago_is_late_again(self, monkeypatch):
    pub, c = _one(monkeypatch, self.D, lead_hist=(ICBM_LATE_LEAD_WAIT_S + 1.0,))
    assert (pub or {}).get("target") is None and c._icbm_start_why == "lateWait"

  def test_a_running_cap_keeps_the_tracking_window(self, monkeypatch):
    """The exemption that matters: a truck decelerating faster than the 0.8 envelope would un-bind its own cap (and then
    restore mid-approach) if a RUNNING episode were held to the envelope too. Tick 1 starts on the envelope edge; tick 2
    is the same truck 4 m on, 0.25 m/s slower (still at set speed), where the envelope alone no longer binds."""
    clock = [1000.0]
    c, step = _controller(monkeypatch, clock, _veh())
    p1 = _tick(c, step, clock, 254.0, 20.2, 20.6)
    assert p1 and p1.get("target") is not None and c._icbm_ep.phase == "cap", "premise: the late start fired"
    v2, d2 = 19.95, 250.0
    assert _far(v2, 20.6, 6.8, d2, firm=0.0, track=False) is None, "premise: the envelope alone has un-bound"
    p2 = _tick(c, step, clock, d2, v2, 20.6)
    assert c._icbm_src == "far" and p2 and p2.get("target") is not None, "the running cap lost its binding"
    assert c._icbm_start_why is None, "icbmStartWhy describes START ticks only"

  def test_chill_clears_the_per_tick_state(self, monkeypatch):
    pub, c = _one(monkeypatch, self.D)
    assert c._icbm_start_why == "lateWait"
    cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_icbm_step"))
    c._icbm_last_pub = -1e9
    cls._icbm_step.__get__(c)({"v_ego": 20.0}, active=False)
    assert c._icbm_start_why is None and c._icbm_firm_t is None and c._icbm_late_t is None

  def test_telemetry_values(self, monkeypatch):
    whys = {}
    whys["env"] = _one(monkeypatch, 250.0)[1]._icbm_start_why
    whys["track"] = _one(monkeypatch, self.D, knob_on=False)[1]._icbm_start_why
    whys["trackLead"] = _one(monkeypatch, self.D, lead_hist=(0.0,))[1]._icbm_start_why
    whys["trackBound"] = _one(monkeypatch, self.D, v=19.0)[1]._icbm_start_why
    whys["lateWait"] = _one(monkeypatch, self.D)[1]._icbm_start_why
    assert whys == {k: k for k in ("env", "track", "trackLead", "trackBound", "lateWait")}
    assert _one(monkeypatch, 450.0)[1]._icbm_start_why is None, "nothing to explain when there is no candidate"


# ---- 5: closed loop with the Ford executor --------------------------------------------------------------------------
class TestClosedLoop:
  def _drive(self, monkeypatch, knob_on=True):
    clock = [6000.0]
    c, step = _controller(monkeypatch, clock, _veh(knob_on))
    v, v_set0 = 20.2, 20.6
    stock, gov, frame, pressing = v_set0, PressGovernor(), 0, False
    d, trace = 330.0, []
    while d > 40.0 and len(trace) < 400:
      c._stock_set = stock
      pub = _tick(c, step, clock, d, v, stock, map_v=6.8)
      clock[0] -= 0.25                       # _tick advanced the clock; the executor loop below advances it itself
      cmd = None
      if pub and pub.get("target") is not None:
        cmd = IcbmCommand(target_ms=pub["target"], ceiling_ms=pub["ceiling"], ts=pub["ts"], dir=pub.get("dir", "dec"))
      for _ in range(25):
        chosen = arbitrate([cmd], clock[0])
        btn = gov.update(frame, decide_press(stock, chosen, clock[0], True, False))
        if pressing and btn is None:
          stock -= STEP_MS
        pressing = btn == "dec"
        assert btn != "inc", "a curve cap pressed SET+"
        frame += 1
        clock[0] += 0.01
      # the truck follows the stock set at up to 1.2 m/s^2 -- faster than the 0.8 the envelope assumes
      v = max(stock if stock < v else v, v - 1.2 * 0.25) if stock < v else v
      d -= v * 0.25
      trace.append((d, v, stock, pub["target"] if pub and pub.get("target") is not None else None, c._icbm_ep.phase))
    return trace

  def test_a_truck_decelerating_faster_than_the_envelope_keeps_its_cap_bound(self, monkeypatch):
    tr = self._drive(monkeypatch)
    started = next((i for i, t in enumerate(tr) if t[3] is not None), None)
    assert started is not None, "the late start never fired"
    assert tr[started][0] < 262.0, f"the late start fired at {tr[started][0]:.0f} m, not on the 0.8 envelope"
    after = tr[started:]
    assert all(t[3] is not None for t in after), \
      f"the cap un-bound mid-approach at {next(t[0] for t in after if t[3] is None):.0f} m"
    assert all(t[4] == "cap" for t in after), "the episode left the cap phase before the curve"
    assert min(t[2] for t in after) < 12.0, "the set never walked toward the apex"


# ---- 6: every call site of a tick uses the SAME inputs ---------------------------------------------------------------
class TestConsistency:
  def _spy(self, monkeypatch):
    calls = {"target": [], "far": []}
    rt, rf = m.icbm_curve_target, m.icbm_far_map_candidate

    def spy_t(*a, **kw):
      calls["target"].append((kw.get("firm_decel"), kw.get("late_drop")))
      return rt(*a, **kw)

    def spy_f(*a, **kw):
      calls["far"].append(a[7] if len(a) > 7 else kw.get("firm_decel"))
      return rf(*a, **kw)
    monkeypatch.setattr(m, "icbm_curve_target", spy_t)
    monkeypatch.setattr(m, "icbm_far_map_candidate", spy_f)
    return calls

  def _stubs(self, c):
    class Shape:
      def tick(self, now, **kw):
        kw["core"](None)
        return None

    class Db:
      enabled = True

      def decide(self, **kw):
        cands, pts = kw["cands_fn"]()
        kw["recand_fn"]("map", pts["map"] if isinstance(pts.get("map"), list) else [])
        kw["recand_fn"]("far", [])
        kw["bind_fn"](5.0, 200.0)
        return kw["today"], kw["src"], None
    c._icbm_shape, c._roaddb, c._way_sel, c._hwy_class = Shape(), Db(), None, None
    cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_icbm_step"))
    c._icbm_roaddb = cls._icbm_roaddb.__get__(c)

  @pytest.mark.parametrize("late_tick", [True, False])
  @pytest.mark.parametrize("gate_src", ["map", "far"])
  def test_all_sites_agree_on_one_tick(self, monkeypatch, late_tick, gate_src):
    calls = self._spy(monkeypatch)
    clock = [1000.0]
    c, step = _controller(monkeypatch, clock, _veh())
    self._stubs(c)
    _tick(c, step, clock, 250.0, 20.2, 20.6, lead=not late_tick)
    firm, late = (0.0, LATE) if late_tick else (1.4, 0.0)
    assert calls["target"] and calls["far"]
    assert len(calls["target"]) >= 4 and len(calls["far"]) >= 3, (calls, "a call site was not exercised")
    assert all(t == (firm, pytest.approx(late)) for t in calls["target"]), calls["target"]
    assert all(f == firm for f in calls["far"]), calls["far"]
    # the passed-point gate re-takes the decision on its own: drive it directly with this tick's stored values
    calls["target"].clear()
    calls["far"].clear()
    monkeypatch.setattr(m, "icbm_passed_points", lambda pts, *a, **k: ([True] * len(pts), "ok"))
    c._icbm_src = gate_src
    sig = _sig(c, 150.0, 20.2, 20.6)          # inside the 10 s window, so a near candidate exists
    m._icbm_passed_gate(c, 1.0, 6.9, sig, LAT0, LON0, 20.6, 6.9, 250.0, 6.9, (0.0, float("inf")))
    assert calls["target"] and all(t == (firm, pytest.approx(late)) for t in calls["target"]), calls["target"]
    assert calls["far"] and all(f == firm for f in calls["far"]), calls["far"]

  def test_helper_falls_back_to_todays_values_on_a_bare_stub(self):
    class Bare:
      pass
    veh = _lightning()
    assert m._icbm_tick_params(Bare(), veh) == (veh.icbm_firm_decel, 0.0)


# ---- 7: the knob ---------------------------------------------------------------------------------------------------
class TestKnob:
  def _veh_with(self, tmp_path, monkeypatch, body, fp="FORD_F_150_LIGHTNING_MK1", brand="ford"):
    p = tmp_path / "curve.json"
    p.write_text(body)
    monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(p))
    monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))
    return PnwVehicle(FakeCP(fp, brand, False))

  def test_default_is_18_mph(self, default_curve_cfg):
    assert _lightning().icbm_late_start_drop_ms == pytest.approx(18 * MPH)

  def test_zero_is_off(self, tmp_path, monkeypatch):
    v = self._veh_with(tmp_path, monkeypatch, '{"lightning": {"icbm_late_start_drop_mph": 0}}')
    assert v.icbm_late_start_drop_ms == 0.0

  @pytest.mark.parametrize("val", [0.5, 5, 10, 17.9])
  def test_values_between_zero_and_default_clamp_up(self, tmp_path, monkeypatch, val):
    warned = []
    monkeypatch.setattr(pv.cloudlog, "warning", lambda msg, *a, **k: warned.append(msg))
    v = self._veh_with(tmp_path, monkeypatch, '{"lightning": {"icbm_late_start_drop_mph": ' + str(val) + '}}')
    assert v.icbm_late_start_drop_ms == pytest.approx(18 * MPH)
    assert any("icbm_late_start_drop_mph" in w for w in warned), "the clamp was silent"

  def test_larger_values_are_kept_up_to_the_bound(self, tmp_path, monkeypatch):
    assert self._veh_with(tmp_path, monkeypatch, '{"lightning": {"icbm_late_start_drop_mph": 25}}'
                          ).icbm_late_start_drop_ms == pytest.approx(25 * MPH)
    assert self._veh_with(tmp_path, monkeypatch, '{"lightning": {"icbm_late_start_drop_mph": 500}}'
                          ).icbm_late_start_drop_ms == pytest.approx(60 * MPH)

  def test_negative_degrades_to_off(self, tmp_path, monkeypatch):
    assert self._veh_with(tmp_path, monkeypatch, '{"lightning": {"icbm_late_start_drop_mph": -4}}'
                          ).icbm_late_start_drop_ms == 0.0

  def test_non_lightning_is_zero(self, default_curve_cfg):
    assert PnwVehicle(FakeCP("TESLA_MODEL_S_RAVEN", "tesla", True)).icbm_late_start_drop_ms == 0.0
    assert PnwVehicle(None).icbm_late_start_drop_ms == 0.0

  def test_unreadable_file_is_logged_and_falls_back(self, tmp_path, monkeypatch):
    errs = []
    monkeypatch.setattr(pv.cloudlog, "error", lambda msg, *a, **k: errs.append(msg))
    v = self._veh_with(tmp_path, monkeypatch, '{"lightning": {"icbm_late_start_drop_mph": ')
    assert errs, "an unreadable curve.json was not logged"
    assert v.icbm_late_start_drop_ms == pytest.approx(18 * MPH), "fallback is the default"

  def test_a_non_numeric_value_is_logged_and_falls_back(self, tmp_path, monkeypatch):
    errs = []
    monkeypatch.setattr(pv.cloudlog, "error", lambda msg, *a, **k: errs.append(msg))
    v = self._veh_with(tmp_path, monkeypatch, '{"lightning": {"icbm_late_start_drop_mph": "soon"}}')
    assert errs and v.icbm_late_start_drop_ms == pytest.approx(18 * MPH)


# ---- 10: the roundabout fixture --------------------------------------------------------------------------------------
class TestRoundaboutFixture:
  def test_first_start_moves_from_the_tracking_window_to_the_envelope(self, monkeypatch):
    off, _ = _start_distance(monkeypatch, knob_on=False)
    on, c = _start_distance(monkeypatch, knob_on=True)
    assert off is not None and on is not None
    assert off >= 320, f"today starts at {off} m (tracking window)"
    assert 240 <= on <= 262, f"late start at {on} m, expected the 240-260 m onset"
    assert off - on >= 60, "the start must move at least 60 m closer"
    assert c._icbm_start_why == "env"
