"""icbmvisraise2pnw: a RUNNING map/far ICBM cap may be lifted by VISION (drive 2026-10-21 PT, I-5 northbound Portland, posted 50).

The drive: mapd rated the curves 43-45 mph, ICBM walked the stock set to 42-43, while the camera measured k 0.0047-0.0051 1/m
(~50 mph at 2.5 m/s^2). The curve DB had no row there. Vision now lifts such a cap through the DB raise's own path
(IcbmEpisode._db_raise), confirmed for 1.0 s, never above the pre-cap set nor the posted limit + 5 mph.

Pinned here, each by a test that fails if the rule is broken:
  * the Portland replay: map 43, vision k 0.0049 covering the apex, limit 50 -> the set rises to ~50, not earlier than 1.0 s;
  * apex beyond vision's reach, low confidence, vision sharper than the map, a straight reading, a tightening curve: no raise,
    the cap stays at mapd's number (and an acted raise is walked back down);
  * the pre-cap set bounds the raise;
  * the controller: labels + icbmVisV reach the record, the published marker is visRaise, the Tesla is "off".
"""
import math

import pytest

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import ICBM_VIS_RAISE_CONFIRM_S, ICBM_RATCHET_CONFIRM_S, IcbmEpisode, icbm_vis_raise
from openpilot.selfdrive.controls.lib.ces_pnw.tests import test_curvedblive2pnw as cdb
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvelead2pnw import _model

MPH = 0.44704
DT = 0.25
A_LAT = 2.5
K_PORTLAND = 0.0049                         # camera, 2026-10-21 I-5 NB Portland: 0.0047-0.0051 1/m
V_EGO = 43 * MPH


def _v(k):
  return math.sqrt(A_LAT / k)


class Truck:
  """IcbmEpisode at 4 Hz, an executor tapping the stock set 1 mph per tick toward the published target, and the controller's
  vision-raise glue (CESController._icbm_step, icbmvisraise2pnw block) in front of it."""

  def __init__(self, set_mph, limit_mph=50.0):
    self.ep = IcbmEpisode()
    self.t = 1000.0
    self.set = set_mph * MPH
    self.limit = limit_mph * MPH
    self.k0 = None
    self.dirs, self.sets, self.whys = [], [], []
    self.first_ok_t = self.first_inc_t = None

  def tick(self, map_mph, vis_k=K_PORTLAND, apex=100.0, reach=190.0, n=1):
    for _ in range(n):
      own = map_mph * MPH
      ref = self.ep.bind_ceiling if self.ep.phase == "cap" else self.set
      v, why = icbm_vis_raise(self.ep.phase == "cap", "map", own, apex, V_EGO, vis_k, reach, A_LAT, 0.0, self.k0, self.limit)
      cap, db, conf = own, None, ICBM_RATCHET_CONFIRM_S
      if v is not None:
        self.k0 = vis_k if self.k0 is None else min(self.k0, vis_k)
        cap = db = min(v, ref)
        conf = ICBM_VIS_RAISE_CONFIRM_S
        if self.first_ok_t is None:
          self.first_ok_t = self.t
      else:
        self.k0 = None
      self.ep.note_limit(self.limit, proportional=False)
      tgt, d = self.ep.step(self.t, cap, self.set, self.set, True, False, limit_now=self.limit, db_raise_t=db,
                            raise_confirm_s=conf)
      self.whys.append(why if v is None else (self.ep.db_raise or "held"))
      self.dirs.append(d)
      if d == "inc" and self.first_inc_t is None:
        self.first_inc_t = self.t
      if tgt is not None and d == "dec" and self.set > tgt + 0.5 * MPH:
        self.set -= MPH
      elif tgt is not None and d == "inc" and self.set < tgt - 0.5 * MPH:
        self.set += MPH
      self.sets.append(round(self.set / MPH, 6))
      self.t += DT


def _capped(set_mph=55.0, map_mph=43.0, limit_mph=50.0):
  """mapd's 43 walks the set down while the camera does not reach the apex yet."""
  tr = Truck(set_mph, limit_mph)
  tr.tick(map_mph, apex=300.0, n=24)
  assert round(tr.set / MPH) == round(map_mph) and tr.ep.phase == "cap" and "inc" not in tr.dirs
  assert set(tr.whys[1:]) == {"noReach"}
  return tr


class TestPortland:
  def test_the_set_rises_to_visions_speed(self):
    tr = _capped()
    n0 = len(tr.sets)
    tr.tick(43.0, n=40)
    assert "inc" in tr.dirs[n0:]
    assert tr.sets[-1] == pytest.approx(50.0, abs=0.01)                   # v_vis 50.5 mph, the executor's 1 mph steps
    assert max(tr.sets) <= 55.0 + 1e-6 and max(tr.sets[n0:]) <= _v(K_PORTLAND) / MPH
    assert tr.whys[-1] in ("held", "acted")

  def test_the_raise_waits_one_second(self):
    tr = _capped()
    tr.tick(43.0, n=12)
    assert tr.first_inc_t - tr.first_ok_t >= ICBM_VIS_RAISE_CONFIRM_S - 1e-9
    assert tr.first_inc_t - tr.first_ok_t < ICBM_VIS_RAISE_CONFIRM_S + 2 * DT

  def test_never_above_the_pre_cap_set(self):
    tr = _capped(set_mph=49.0, map_mph=43.0)
    tr.tick(43.0, n=40)
    assert "inc" in tr.dirs and max(tr.sets) <= 49.0 + 1e-6 and tr.sets[-1] == pytest.approx(49.0, abs=0.01)


class TestNoRaise:
  @pytest.mark.parametrize("kw, why", [
    ({"apex": 200.0, "reach": 190.0}, "noReach"),                           # mapd may see a sharper curve past the camera
    ({"apex": 160.0, "reach": 400.0}, "noReach"),                           # inside the model reach, beyond 8 s at 43 mph
    ({"vis_k": None}, "lowConf"),
    ({"reach": 0.0}, "lowConf"),
    ({"vis_k": float("nan")}, "lowConf"),
    ({"vis_k": 0.0070}, "noGain"),                                          # vision SHARPER than the map (42 mph)
    ({"vis_k": 0.0060}, "noGain"),                                          # 45.7 mph: within 5 mph of mapd's 43
    ({"vis_k": 0.0001}, "noCurve"),                                         # a straight reading is not evidence
  ])
  def test_the_cap_stays_at_mapds_number(self, kw, why):
    tr = _capped()
    n0 = len(tr.sets)
    tr.tick(43.0, n=20, **kw)
    assert "inc" not in tr.dirs and set(tr.sets[n0:]) == {43.0} and set(tr.whys[n0:]) == {why}

  def test_a_tightening_curve_drops_an_acted_raise(self):
    tr = _capped()
    tr.tick(43.0, n=40)
    assert tr.sets[-1] == pytest.approx(50.0, abs=0.01)
    n0 = len(tr.sets)
    tr.tick(43.0, vis_k=K_PORTLAND * 1.25)                                  # the curve tightens: drop at once
    assert tr.whys[-1] == "kRise" and tr.dirs[-1] == "dec"
    tr.tick(43.0, vis_k=K_PORTLAND * 1.25, n=20)                           # 45.2 mph: no longer 5 mph over mapd
    assert tr.sets[-1] == pytest.approx(43.0, abs=0.01) and "inc" not in tr.dirs[n0:]

  def test_a_dropout_gives_mapds_cap_back_at_once(self):
    tr = _capped()
    tr.tick(43.0, n=40)
    n0 = len(tr.sets)
    tr.tick(43.0, vis_k=None, n=12)
    assert tr.whys[n0] == "lowConf" and tr.dirs[n0] == "dec" and tr.sets[-1] == pytest.approx(43.0, abs=0.01)

  def test_an_unknown_limit_is_no_raise(self):
    tr = _capped()
    tr.limit = None
    tr.tick(43.0, n=12)
    assert "inc" not in tr.dirs and set(tr.whys[-12:]) == {"noLimit"}


class TestGates:
  def test_labels(self):
    base = dict(capping=True, src="map", own=43 * MPH, cand_dist=100.0, v_ego=V_EGO, vis_k=K_PORTLAND, vis_reach=190.0,
                a_lat=A_LAT, rain_ms=0.0, k_ref=None, limit=50 * MPH)
    assert icbm_vis_raise(**base) == (pytest.approx(_v(K_PORTLAND)), "ok")
    assert icbm_vis_raise(**{**base, "src": "far"})[1] == "ok"
    for kw, why in (({"a_lat": 0.0}, "off"), ({"capping": False}, "noCap"), ({"src": "vis"}, "notMap"),
                    ({"src": "gpsHold"}, "notMap"), ({"own": None}, "notMap"), ({"cand_dist": float("inf")}, "noReach"),
                    ({"k_ref": K_PORTLAND / 1.2}, "kRise"), ({"limit": 0.0}, "noLimit"), ({"limit": 44 * MPH}, "noCurve")):
      assert icbm_vis_raise(**{**base, **kw}) == (None, why), kw
    assert icbm_vis_raise(**{**base, "k_ref": K_PORTLAND / 1.05})[1] == "ok"   # inside the noise band
    assert icbm_vis_raise(**{**base, "rain_ms": 1.0})[0] == pytest.approx(_v(K_PORTLAND) - 1.0)


def _curve_later(monkeypatch, k=K_PORTLAND, horizon_s=10.0, after=150, k_late=None, late=300):
  """The camera sees a straight road while the cap starts (a cap never STARTS in a curve), then the curve 40 m ahead
  (curvature k; k_late from model call `late` on)."""
  n = [0]

  def model(v, _at, _r):
    n[0] += 1
    kk = k_late if (k_late is not None and n[0] > late) else k
    return _model(v, 40.0, 1.0 / kk, horizon_s=horizon_s) if n[0] > after else _model(v, 5000.0, 1e6)
  monkeypatch.setattr(cdb, "_model", model)


def _drive(monkeypatch, tmp_path, **kw):
  return cdb._drive(monkeypatch, tmp_path, points=cdb._path({140.0: 43 * MPH}), anchors=[cdb.FAR_ANCHOR], stock_mph=55.0,
                    posted_mph=50.0, v_ego=24.0, **kw)


class TestController:
  def test_portland_lifts_the_published_cap_to_visions_speed(self, monkeypatch, tmp_path):
    _curve_later(monkeypatch)
    icbm, recs, _ = _drive(monkeypatch, tmp_path)
    tg = [t for t in cdb._targets(icbm) if t is not None]
    assert tg[0] == pytest.approx(43 * MPH, abs=0.01) and tg[-1] == pytest.approx(_v(K_PORTLAND), abs=0.01)
    last = recs[-1]
    assert last["icbmSrc"] == "map" and last["icbmVisV"] == pytest.approx(_v(K_PORTLAND), abs=0.01)
    assert last["icbmVisRaise"] == "held" and last["icbmDbRaise"] is None   # the set (55) is above it: the dec cap holds there
    assert {r["icbmVisRaise"] for r in recs if r["icbmPhase"] == "cap"} <= {"noCap", "noCurve", "held"}

  def test_the_controller_passes_the_one_second_confirm(self, monkeypatch, tmp_path):
    _curve_later(monkeypatch)
    seen = []
    real = IcbmEpisode.step

    def spy(self, *a, **k):
      seen.append((k.get("db_raise_t"), k.get("raise_confirm_s")))
      return real(self, *a, **k)
    monkeypatch.setattr(IcbmEpisode, "step", spy)
    _drive(monkeypatch, tmp_path)
    fed = [c for d, c in seen if d is not None]
    assert fed and set(fed) == {ICBM_VIS_RAISE_CONFIRM_S}

  def test_a_tightening_camera_reading_is_kRise(self, monkeypatch, tmp_path):
    _curve_later(monkeypatch, k_late=K_PORTLAND * 1.25)
    icbm, recs, _ = _drive(monkeypatch, tmp_path)
    whys = [r["icbmVisRaise"] for r in recs if r["icbmPhase"] == "cap"]
    assert "held" in whys and whys.index("kRise") > whys.index("held")
    assert cdb._targets(icbm)[-1] == pytest.approx(43 * MPH, abs=0.01)    # mapd's cap is back

  def test_apex_beyond_the_camera_is_no_raise(self, monkeypatch, tmp_path):
    _curve_later(monkeypatch, horizon_s=4.0)                                # reach 96 m < the 140 m apex
    icbm, recs, _ = _drive(monkeypatch, tmp_path)
    assert {round(t, 2) for t in cdb._targets(icbm) if t is not None} == {round(43 * MPH, 2)}
    assert recs[-1]["icbmVisRaise"] == "noReach" and recs[-1]["icbmVisV"] is None

  def test_vision_sharper_than_the_map_keeps_the_cap(self, monkeypatch, tmp_path):
    _curve_later(monkeypatch, k=0.0070)
    icbm, recs, _ = _drive(monkeypatch, tmp_path)
    assert max(t for t in cdb._targets(icbm) if t is not None) <= 43 * MPH + 0.01
    # the camera's own sharper curve now binds: a vision cap, which vision never raises
    assert recs[-1]["icbmSrc"] == "vis" and recs[-1]["icbmVisRaise"] == "notMap" and recs[-1]["icbmVisV"] is None

  def test_a_target_the_db_changed_is_never_raised_by_vision(self, monkeypatch, tmp_path):
    _curve_later(monkeypatch)
    real = m.CESController._icbm_roaddb

    def db_lowers(self, target, *a, **k):
      t, fd = real(self, target, *a, **k)
      return (None if t is None else t - 0.5), fd                         # the DB's own (lower) number binds
    monkeypatch.setattr(m.CESController, "_icbm_roaddb", db_lowers)
    icbm, recs, _ = _drive(monkeypatch, tmp_path)
    assert max(t for t in cdb._targets(icbm) if t is not None) < 43 * MPH
    assert {r["icbmVisRaise"] for r in recs if r["icbmPhase"] == "cap"} == {"notMap"}

  def test_the_tesla_is_off(self, monkeypatch, tmp_path):
    _curve_later(monkeypatch)
    _, recs, c = _drive(monkeypatch, tmp_path, fp=cdb.TESLA, brand="tesla")
    assert c._veh.icbm_restore_hold_lat_accel == 0.0
    assert recs and {r.get("icbmVisRaise") for r in recs} <= {None, "off"} and {r.get("icbmVisV") for r in recs} == {None}


def test_a_vision_raise_is_published_with_its_own_marker(monkeypatch, tmp_path):
  """speedadjust reads both markers as ICBM's own SET+ (speedadjust_controller._icbm_why); the DB's stays dbRaise."""
  from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvedbtel2pnw import _icbm_stub_at, _run, _sig
  mgr, step = _icbm_stub_at(monkeypatch, tmp_path)
  _run(mgr, step, _sig(25.0, 60 * MPH))
  monkeypatch.setattr(mgr._icbm_ep, "step", lambda *a, **k: (20.0, "inc"))
  for vis, want in (((22.0, "ok"), "visRaise"), ((None, "noGain"), "dbRaise")):
    monkeypatch.setattr(m, "icbm_vis_raise", lambda *a, _r=vis, **k: _r)
    mgr._icbm_ep.db_raise = "acted"
    _run(mgr, step, {**_sig(25.0, 60 * MPH), "vis_k_max": K_PORTLAND})
    assert mgr.mem_params.last.get("dir") == "inc" and mgr.mem_params.last.get("why") == want
