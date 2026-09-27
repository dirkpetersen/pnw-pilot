"""curveshape2pnw 2/3 -- the measured-shape stage wired into ICBM, on the REAL controller (CURVE-MEASURED-SHAPE-DESIGN.md
s5-s6; owner answers 2026-09-24).

Pinned here:
  * SHADOW IS INERT: the published IcbmTarget sequence is byte-identical to "off", while the shp* fields say what the
    stage WOULD do;
  * live mode changes the target, exactly to the stage's own verdict, and the curve DB still overrides afterwards;
  * the Tesla has no stage (capability) and its ICBM output is unchanged;
  * a failing stage costs the stage, not ICBM: legacy target, shpOn=err, a throttled cloudlog.exception;
  * the fields reach the ces_events line on disk;
  * curve.json's string switch cannot knock the numeric knobs back to defaults.
All geometry is synthetic.
"""
from __future__ import annotations

import copy
import json
import math
import types

import pytest

from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
from openpilot.selfdrive.controls.lib.ces_pnw import curvedb_live as cl
from openpilot.selfdrive.controls.lib.ces_pnw import icbm_shape as shp
from openpilot.selfdrive.controls.lib.ces_pnw.tests.roaddb_fixture import reader, write_db
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_ces_mode_read_failure_logged import LIGHTNING, _P
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvelead2pnw import LAT0, LON0, FakeCP, _model

NS = types.SimpleNamespace
MPH = cl.MPH
TESLA = "TESLA_MODEL_S_HW3"
A_MAPD = 2.2                    # the fixture's mapd A (standard personality)
R = 400.0                       # the arc's radius: k = 0.0025


def _ll(x, y):
  return LAT0 + y / 111320.0, LON0 + x / (111320.0 * math.cos(math.radians(LAT0)))


def arc_path(k_ratio=1.0, y_turn=200.0, step=40.0, n_arc=12, r=R):
  """North through the truck (y = 0) to y_turn, then a right-hand arc of radius R, a node every 40 m. mapd rates the
  arc nodes at sqrt(A_mapd / (k * k_ratio)): k_ratio 1 = mapd agrees with the geometry exactly; 2 = mapd claims a curve
  twice as sharp (the I-5 45.72 phantom shape)."""
  pts, y = [], -180.0
  while y < y_turn:
    la, lo = _ll(0.0, y)
    pts.append({"latitude": la, "longitude": lo, "velocity": 0.0})
    y += step
  v_arc = math.sqrt(A_MAPD / (k_ratio / r))
  th = step / r
  for i in range(n_arc + 1):
    a = i * th
    x, yy = r * (1.0 - math.cos(a)), y_turn + r * math.sin(a)
    la, lo = _ll(x, yy)
    pts.append({"latitude": la, "longitude": lo, "velocity": v_arc if 0 < i < n_arc else 0.0})
  return pts


def _run(monkeypatch, tmp_path, *, points, shape=None, anchors=None, fp=LIGHTNING, brand="ford", op_long=False,
         stock_mph=90.0, posted_mph=70.0, v_ego=30.0, ticks=400, log_file=None, extra_cfg=None):
  tmp_path.mkdir(parents=True, exist_ok=True)
  cfg = tmp_path / "curve.json"
  light = dict(extra_cfg or {})
  if shape is not None:
    light["icbm_shape"] = shape
  if light:
    cfg.write_text(json.dumps({"lightning": light}))
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(cfg))
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))
  d = tmp_path / "roaddb"
  d.mkdir(exist_ok=True)
  write_db(str(d), anchors or [[0.5, 0.5, 0.0, [[0.5013, 0.5, 0.001, 3]]]])
  monkeypatch.setattr(cl, "DATA_DIR", str(d))
  monkeypatch.setattr(cl, "READ_PARAMS", [reader()])

  clock = [5000.0]
  ns = types.SimpleNamespace(monotonic=lambda: clock[0], time=lambda: clock[0])
  for mod in (C, m, cl):
    monkeypatch.setattr(mod, "time", ns)
  C._ces_mode_hold_st.clear()

  class Mem:
    def __init__(self):
      self.puts = []

    def get(self, k, return_default=False):
      return {"MapTargetVelocities": points, "MapSpeedLimit": str(posted_mph * MPH), "MapHighwayClass": "motorway",
              "MapWaySel": "current",
              "LastGPSPosition": json.dumps({"latitude": LAT0, "longitude": LON0, "bearing": 0.0, "src": "device",
                                             "ts": clock[0], "fix_ts": clock[0] - 0.3})}.get(k)

    def put_nonblocking(self, k, v):
      self.puts.append((k, copy.deepcopy(v)))

  params = _P(clock, mode=lambda t: 2, extra={"CESButtonState": "0"})
  c = m.CESController(FakeCP(fp, brand, op_long), params=params)
  c.mem_params = Mem()
  recs = []
  c._event_log_ok = True
  if log_file is not None:
    monkeypatch.setattr(m, "CES_EVENT_LOG", str(log_file))
  else:
    c._append_event = lambda rec: None if rec.get("ev") == "mapdPath" else recs.append(copy.deepcopy(rec))
  stock = stock_mph * MPH
  for i in range(ticks):
    clock[0] = 5000.0 + (i + 1) * 0.01
    orz, vx, px, ts = _model(v_ego, 5000.0, 1e6)          # vision: a straight road (the map is on its own here)
    model = NS(orientationRate=NS(z=orz, t=ts), velocity=NS(x=vx), position=NS(x=px),
               action=NS(shouldStop=False), meta=NS(laneChangeState="off"))
    sm = {"radarState": NS(leadOne=NS(status=False, vLead=0.0, dRel=0.0, aLeadK=0.0, vLeadK=0.0)),
          "modelV2": model, "carControl": NS(orientationNED=[0.0, 0.0, 0.0]),
          "livePose": NS(angularVelocityDevice=NS(x=0.0, y=0.0, z=0.0, valid=True)),
          "controlsState": NS(desiredCurvature=0.0,
                              lateralControlState=NS(which=lambda: "angleState", angleState=NS(saturated=False)))}
    cstate = NS(vEgo=v_ego, aEgo=0.0, gasPressed=False, brakePressed=False, leftBlinker=False, rightBlinker=False,
                vCruise=stock * 3.6, standstill=False, steeringAngleDeg=0.0, steeringPressed=False,
                leftBlindspot=False, rightBlindspot=False, cruiseState=NS(speed=stock, enabled=True),
                yawRate=0.0, steeringTorque=0.0)
    c.experimental_request(cstate, sm)
  icbm = [v for k, v in c.mem_params.puts if k == "IcbmTarget"]
  return icbm, recs, c


def _targets(icbm):
  return [p.get("target") for p in icbm]


AGREE = arc_path(1.0)          # polyline and mapd agree exactly
MAP_SHARPER = arc_path(2.0, r=800.0)   # mapd claims twice the curvature the geometry has (rated 66 mph)


class TestShadowIsInert:
  def test_shadow_publishes_byte_identical_targets_and_logs_what_it_would_do(self, monkeypatch, tmp_path):
    off, recs_off, c_off = _run(monkeypatch, tmp_path / "off", points=AGREE, shape="off")
    sh, recs, c = _run(monkeypatch, tmp_path / "shadow", points=AGREE, shape="shadow")
    assert c.__dict__["_icbm_shape"].mode == "shadow" and c_off._icbm_shape.mode == "off"
    assert any(t is not None for t in _targets(off)), "ICBM never slowed: the scenario proves nothing"
    assert json.dumps(sh, sort_keys=True) == json.dumps(off, sort_keys=True)          # the taps' input, byte for byte
    # the record's own control fields are identical too (only shp* may differ)
    ctl = ("icbmT", "icbmC", "icbmSrc", "icbmDir", "icbmPhase", "icbmMapFlr", "cdb2Tgt")
    assert [{k: r[k] for k in ctl} for r in recs] == [{k: r[k] for k in ctl} for r in recs_off]
    live_recs = [r for r in recs if r["shpWhy"] == "ok"]
    assert live_recs, {r["shpWhy"] for r in recs}
    r = live_recs[-1]
    assert r["shpOn"] == "shadow" and r["shpDir"] == "lower" and r["shpStable"] is True
    assert r["shpT"] < r["shpBase"]
    assert r["shpK"] == pytest.approx(1.0 / R, rel=0.02) and r["shpA"] == A_MAPD
    assert r["shpV"] == pytest.approx(math.sqrt(2.5 / (1.0 / R)), rel=0.02)
    assert r["shpN"] > 0 and r["shpNL"] > 0
    assert {r["shpOn"] for r in recs_off} == {"off"} and {r["shpWhy"] for r in recs_off} == {"curve.json"}

  def test_the_default_is_shadow(self, monkeypatch, tmp_path):
    _, recs, c = _run(monkeypatch, tmp_path / "d", points=AGREE)
    assert c._icbm_shape.mode == "shadow" and c._veh.icbm_shape_why == "default"
    assert {r["shpOn"] for r in recs} == {"shadow"}


class TestLive:
  def test_live_lowers_to_the_stages_own_verdict(self, monkeypatch, tmp_path):
    off, _, _ = _run(monkeypatch, tmp_path / "off", points=AGREE, shape="off")
    on, recs, _ = _run(monkeypatch, tmp_path / "live", points=AGREE, shape="live")
    t_off = [t for t in _targets(off) if t is not None][-1]
    t_on = [t for t in _targets(on) if t is not None][-1]
    assert t_on < t_off - 1.0
    r = [x for x in recs if x["shpWhy"] == "ok"][-1]
    assert r["shpOn"] == "live" and r["icbmT"] == pytest.approx(r["shpT"], abs=0.011)
    # the base hump is given back: the published target IS the shape price (no left / descent extra on this road)
    assert t_on == pytest.approx(math.sqrt(2.5 * R), abs=0.3)

  def test_live_raises_within_the_caps(self, monkeypatch, tmp_path):
    """mapd reads the curve 1.3x sharper than the geometry (inside the x1.35 band) at ~51 mph: today's price (~53 mph)
    is below the agreed mean's ~58 mph, so live RAISES -- and every candidate that names the curve (near AND far) must
    be priced, or the unpriced one keeps the old, lower number."""
    pts = arc_path(1.3, r=310.0, n_arc=5)      # a 200 m arc: the polyline peak sits within 150 m of the candidate
    off, _, _ = _run(monkeypatch, tmp_path / "off", points=pts, shape="off")
    on, recs, _ = _run(monkeypatch, tmp_path / "live", points=pts, shape="live")
    t_off = [t for t in _targets(off) if t is not None][-1]
    t_on = [t for t in _targets(on) if t is not None][-1]
    raw = pts[next(j for j, p in enumerate(pts) if p["velocity"] > 0)]["velocity"]
    kp, km = 1.0 / 310.0, A_MAPD / raw ** 2
    want = math.sqrt(2.5 / (0.5 * (kp + km)))
    assert raw / MPH > 50.0 and t_on > t_off + 1.0
    assert t_on == pytest.approx(want, abs=0.3)
    r = [x for x in recs if x["shpWhy"] == "ok"][-1]
    assert r["shpDir"] == "raise" and r["shpNR"] > 0

  def test_a_disagreeing_mapd_changes_nothing_even_live(self, monkeypatch, tmp_path):
    off, _, _ = _run(monkeypatch, tmp_path / "off", points=MAP_SHARPER, shape="off")
    on, recs, _ = _run(monkeypatch, tmp_path / "live", points=MAP_SHARPER, shape="live")
    assert any(t is not None for t in _targets(off)), "mapd's claim must bind today, or 'unchanged' proves nothing"
    assert on == off
    assert "mapSharper" in {r["shpWhy"] for r in recs}
    assert {r["shpDir"] for r in recs} <= {"none", None}

  def test_the_curve_db_still_overrides_afterwards(self, monkeypatch, tmp_path):
    poly = cl.Polyline(AGREE)
    i = next(j for j, p in enumerate(AGREE) if p["velocity"] > 0)
    s_a = poly.s[poly.src_idx.index(i)]
    la, lo = AGREE[i]["latitude"], AGREE[i]["longitude"]
    brg = poly.heading(s_a)
    e_la, e_lo = poly.at(s_a + 150.0)
    row = [[la, lo, brg, [[e_la, e_lo, 0.006, 3]]]]                  # the road is sharper than both readings
    on, recs, _ = _run(monkeypatch, tmp_path / "live", points=AGREE, shape="live", anchors=row)
    t_on = [t for t in _targets(on) if t is not None][-1]
    assert t_on == pytest.approx(math.sqrt(A_MAPD / 0.006), abs=0.02)
    assert any(r["cdb2Dir"] == "lower" and r["shpOn"] == "live" for r in recs)


class TestCapability:
  def test_the_tesla_has_no_stage_and_unchanged_output(self, monkeypatch, tmp_path):
    a, recs_a, c = _run(monkeypatch, tmp_path / "a", points=AGREE, fp=TESLA, brand="tesla", op_long=True, shape="off")
    b, recs_b, c2 = _run(monkeypatch, tmp_path / "b", points=AGREE, fp=TESLA, brand="tesla", op_long=True, shape="live")
    assert c._veh.icbm_shape == "off" and c2._veh.icbm_shape == "off" and c2._veh.icbm_shape_why == "noCapability"
    assert a == b
    assert {r["shpOn"] for r in recs_b} == {"off"} and {r["shpWhy"] for r in recs_b} == {"noCapability"}

  @pytest.mark.parametrize("raw,mode,why", [("live", "live", "curve.json"), ("LIVE ", "live", "curve.json"),
                                            ("off", "off", "curve.json"), (0, "off", "curve.json"),
                                            ("shadow", "shadow", "curve.json"), ("on", "shadow", "INVALID"),
                                            (1, "shadow", "INVALID"), (None, "shadow", "INVALID")])
  def test_the_switch(self, tmp_path, monkeypatch, raw, mode, why):
    f = tmp_path / "curve.json"
    f.write_text(json.dumps({"lightning": {"icbm_shape": raw, "map_scale": 0.8}}))
    monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(f))
    v = pv.PnwVehicle(FakeCP(LIGHTNING, "ford", False))
    assert v.icbm_shape == mode and v.icbm_shape_why.startswith(why)
    assert v.icbm_map_scale == 0.8, "the string switch must not throw the numeric knobs back to their defaults"

  def test_an_invalid_switch_is_an_error_at_start(self, monkeypatch, tmp_path):
    errs = []
    monkeypatch.setattr(m.cloudlog, "error", lambda msg, *a, **k: errs.append(msg))
    _run(monkeypatch, tmp_path / "bad", points=AGREE, shape="yes please", ticks=2)
    assert any("icbm_shape" in e and "INVALID" in e for e in errs)

  @pytest.mark.parametrize("cfg,want", [(None, 2.5), (2.2, 2.2), (0.3, 1.5), (9.0, 3.2)])
  def test_the_lateral_knobs(self, tmp_path, monkeypatch, cfg, want):
    f = tmp_path / "curve.json"
    if cfg is not None:
      f.write_text(json.dumps({"lightning": {"icbm_shape_lat_a": cfg, "icbm_shape_lat_a_70": cfg}}))
    monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(f))
    v = pv.PnwVehicle(FakeCP(LIGHTNING, "ford", False))
    assert v.icbm_shape_lat_a == want and v.icbm_shape_lat_a_70 == want


class TestFailure:
  def test_a_crashing_stage_costs_the_stage_not_icbm(self, monkeypatch, tmp_path):
    off, _, _ = _run(monkeypatch, tmp_path / "off", points=AGREE, shape="off")
    calls = []
    monkeypatch.setattr(m.cloudlog, "exception", lambda msg, *a, **k: calls.append(msg))
    monkeypatch.setattr(shp, "cloudlog", m.cloudlog)

    def boom(self, *a, **k):
      raise RuntimeError("stage on fire")
    monkeypatch.setattr(shp.ShapeStage, "tick", boom)
    on, recs, c = _run(monkeypatch, tmp_path / "live", points=AGREE, shape="live")
    assert on == off
    assert {r["shpOn"] for r in recs if r["icbmT"] is not None} == {"err"}
    assert {r["shpErr"] for r in recs if r["icbmT"] is not None} == {"RuntimeError"}
    shape_logs = [x for x in calls if "icbm_shape" in x]
    assert 1 <= len(shape_logs) <= 2, shape_logs          # throttled (30 s) over the 4 s run
    assert c._icbm_shape.n_err > 5

  def test_a_crashing_pricer_is_not_handed_to_icbm(self, monkeypatch, tmp_path):
    off, _, _ = _run(monkeypatch, tmp_path / "off", points=AGREE, shape="off")
    errs = []
    monkeypatch.setattr(shp.cloudlog, "error", lambda msg, *a, **k: errs.append(msg))

    def boom(*a, **k):
      raise ZeroDivisionError("bug")
    monkeypatch.setattr(shp, "shape_price", boom)
    on, recs, c = _run(monkeypatch, tmp_path / "live", points=AGREE, shape="live")
    assert on == off
    assert "err" in {r["shpOn"] for r in recs} and any("FAILED" in e for e in errs)
    assert c._icbm_eff_fn is None, "a pricer that failed must not be handed to ICBM"

  def test_a_stub_controller_reads_absent(self):
    class Stub:
      pass
    assert m._shape_tele(Stub()) == {**dict.fromkeys(shp.TELE_KEYS), "shpOn": "absent"}


class TestOnDisk:
  def test_the_shp_fields_reach_the_ces_events_line_on_disk(self, monkeypatch, tmp_path):
    log = tmp_path / "ces_events.jsonl"
    _run(monkeypatch, tmp_path / "rec", points=AGREE, shape="shadow", log_file=log)
    lines = [json.loads(ln) for ln in log.read_text().splitlines()]
    ticks = [r for r in lines if r.get("ev") in ("tick", "adopt")]
    assert ticks, "nothing was written"
    for r in ticks:
      assert set(shp.TELE_KEYS) <= set(r), set(shp.TELE_KEYS) - set(r)
    ok = [r for r in ticks if r["shpWhy"] == "ok"]
    assert ok
    r = ok[-1]
    for k in ("shpKP", "shpKM", "shpK", "shpV", "shpLeg", "shpD", "shpBase", "shpT", "shpA", "shpN", "shpNL"):
      assert r[k] is not None, k
    assert r["shpOn"] == "shadow" and r["shpDir"] == "lower" and r["shpSrc"] in ("map", "far")

  def test_tele_keys_are_exactly_what_tele_emits(self):
    st = shp.ShapeStage("shadow", "default", 2.5, 2.5, read_params=reader())
    assert set(st.tele(0.0)) == set(shp.TELE_KEYS)
    st._record(0.0, shpWhy="x")
    assert set(st.tele(0.0)) == set(shp.TELE_KEYS)
