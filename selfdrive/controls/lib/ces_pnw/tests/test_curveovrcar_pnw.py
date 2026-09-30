"""ovrcar2pnw -- the per-curve override file is PER-CAR (schema v2), used ONLY by the Tesla Model S (Raven HW3) and the Ford F-150
Lightning, and every entry has a mandatory DIRECTION (owner 2026-09-29). Pinned here, each by a test that fails when the rule breaks:

  * schema: unknown platform, missing / oversized heading tolerance, missing cars, both forms at once, the v1 compat read;
  * capability: PnwVehicle.curve_override_platform, the opendbc platform names, a car outside the two never reads the file;
  * per-car application: a Tesla entry never affects the Lightning and vice versa (loader AND both consumers);
  * lower-only on both cars; fail-safe per car (Tesla 2.8 everywhere, Lightning no overrides), a mid-drive typo keeps the list;
  * direction and the three real seed entries against the REAL table, with the speed each car is priced at.
"""
from __future__ import annotations

import json
import math
import os
import pathlib
import types

import pytest

from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw import curve_brain as cb
from openpilot.selfdrive.controls.lib.ces_pnw import curvedb_live as cl
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvebrain_need_pnw import (
  _SEED_SKIP, _brain, _seed_is_v2, _step, cfgpath, schedule, tesla)  # noqa: F401
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvedblive2pnw import (
  ADD_ROW, LAT0, LON0, STRAIGHT, _anchor, _drive, _ll, _path)
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvelead2pnw import FakeCP

TESLA_P = "TESLA_MODEL_S_HW3"
LIGHT_P = "FORD_F_150_LIGHTNING_MK1"
MPH = 0.44704
K_ADD = 0.004          # ADD_ROW's curvature


class _Log:
  def __init__(self):
    self.errors, self.warnings, self.events, self.exceptions = [], [], [], []

  def error(self, msg, *a, **k):
    self.errors.append(msg)

  def warning(self, msg, *a, **k):
    self.warnings.append(msg)

  def exception(self, msg, *a, **k):
    self.exceptions.append(msg)

  def event(self, name, **k):
    self.events.append((name, k))

  def info(self, *a, **k):
    pass

  def debug(self, *a, **k):
    pass


@pytest.fixture
def log(monkeypatch):
  lg = _Log()
  for mod in (cb, cl, pv):
    monkeypatch.setattr(mod, "cloudlog", lg)
  return lg


def _e(cars, **kw):
  e = {"lat": 45.0, "lon": -122.0, "radius_m": 100.0, "heading_deg": 90.0, "heading_tol_deg": 30.0, "cars": cars, "note": "t"}
  e.update(kw)
  return e


def _doc(entries, version=2):
  return {"version": version, "overrides": entries}


def _load(platform, doc=None, raw=None):
  pathlib.Path(cb.OVERRIDES_PATH).write_text(raw if raw is not None else json.dumps(doc))
  return cb.Overrides(platform=platform)


# =====================================================================================================
# the capability and the platform names
# =====================================================================================================
class TestCapability:
  def test_the_two_names_are_opendbc_platforms(self):
    from opendbc.car.ford.values import CAR as FORD
    from opendbc.car.tesla.values import CAR as TES
    assert pv.CURVE_OVERRIDE_PLATFORMS == (str(TES.TESLA_MODEL_S_HW3), str(FORD.FORD_F_150_LIGHTNING_MK1))
    assert pv.CURVE_OVERRIDE_PLATFORMS == (TESLA_P, LIGHT_P) and pv.CURVE_OVERRIDE_V1_PLATFORM == TESLA_P

  def test_the_capability_names_the_platform_of_exactly_the_two_cars(self):
    from opendbc.car.toyota.values import CAR as TOY
    assert tesla().curve_override_platform == TESLA_P
    assert pv.PnwVehicle(FakeCP(LIGHT_P, "ford", False)).curve_override_platform == LIGHT_P
    for fp in (str(TOY.TOYOTA_COROLLA), "TESLA_MODEL_3", "TESLA_MODEL_S_HW2", "FORD_F_150_MK14", ""):
      assert pv.PnwVehicle(FakeCP(fp, "x", False)).curve_override_platform is None, fp

  def test_the_loader_refuses_a_platform_outside_the_two(self):
    with pytest.raises(ValueError):
      cb.Overrides(platform="TOYOTA_COROLLA")

  def test_a_car_outside_the_two_never_reads_the_file(self, monkeypatch):
    """A Toyota's CESController builds no Overrides (neither the DB's nor the brain's) -- the file is never opened."""
    built = []
    real = cb.Overrides

    class Spy(real):
      def __init__(self, *a, **k):
        built.append(k.get("platform"))
        super().__init__(*a, **k)
    monkeypatch.setattr(m, "Overrides", Spy)
    monkeypatch.setattr(cb, "Overrides", Spy)
    c = m.CESController(FakeCP("TOYOTA_COROLLA", "toyota", False))
    assert built == [] and c._roaddb.overrides is None and c._curve_brain is None
    m.CESController(FakeCP(LIGHT_P, "ford", False))
    assert built == [LIGHT_P]
    m.CESController(FakeCP(TESLA_P, "tesla", True))
    assert built == [LIGHT_P, TESLA_P]        # the Tesla's brain; its DB (disabled on the Tesla) builds none


# =====================================================================================================
# schema v2
# =====================================================================================================
class TestSchema:
  def test_a_valid_file_gives_each_car_only_its_own_entries(self, log):
    doc = _doc([_e({TESLA_P: {"a_max": 2.8}}), _e({LIGHT_P: {"a_max": 2.2}}, lat=45.1),
                _e({TESLA_P: {"a_max": 2.6}, LIGHT_P: {"a_max": 2.4}}, lat=45.2)])
    t, li = _load(TESLA_P, doc), _load(LIGHT_P, doc)
    assert not t.failsafe and not li.failsafe and log.errors == []
    assert [x["a_max"] for x in t.entries] == [2.8, 2.6] and [x["a_max"] for x in li.entries] == [2.2, 2.4]
    assert [x["lat"] for x in t.entries] == [45.0, 45.2] and [x["lat"] for x in li.entries] == [45.1, 45.2]

  @pytest.mark.parametrize("plat", ["TOYOTA_COROLLA", "TESLA_MODEL_3", "tesla_model_s_hw3", "", None, 3])
  @pytest.mark.parametrize("car", [TESLA_P, LIGHT_P])
  def test_an_unknown_platform_invalidates_the_file_loudly_for_both_cars(self, log, car, plat):
    doc = _doc([_e({TESLA_P: {"a_max": 2.8}}), _e({plat: {"a_max": 2.5}})])
    o = _load(car, doc)
    assert o.failsafe and len(log.errors) == 1 and "not one of" in o.why

  @pytest.mark.parametrize("drop", ["heading_deg", "heading_tol_deg"])
  def test_direction_is_mandatory(self, log, drop):
    e = _e({TESLA_P: {"a_max": 2.8}})
    del e[drop]
    assert _load(TESLA_P, _doc([e])).failsafe and log.errors

  @pytest.mark.parametrize("tol, ok", [(0.0, False), (-5.0, False), (1.0, True), (60.0, True), (60.01, False), (90.0, False),
                                       (180.0, False), (360.0, False), ("30", False), (None, False)])
  def test_the_heading_tolerance_is_positive_and_at_most_60(self, log, tol, ok):
    o = _load(LIGHT_P, _doc([_e({LIGHT_P: {"a_max": 2.2}}, heading_tol_deg=tol)]))
    assert (not o.failsafe) is ok, o.why

  @pytest.mark.parametrize("cars", [None, {}, [], "x", {TESLA_P: None}, {TESLA_P: {}}, {TESLA_P: {"a_max": "2"}},
                                    {TESLA_P: {"a_max": 0.5}}, {TESLA_P: {"a_max": 6.0}}, {TESLA_P: {"a_max": True}}])
  def test_a_bad_cars_object_is_invalid(self, log, cars):
    e = _e(cars)
    o = _load(TESLA_P, _doc([e]))
    assert o.failsafe and log.errors

  def test_missing_cars_in_a_v2_file_is_invalid(self, log):
    e = _e({})
    del e["cars"]
    e["a_max"] = 2.8
    assert _load(TESLA_P, _doc([e])).failsafe

  def test_both_forms_in_one_entry_is_invalid(self, log):
    assert _load(TESLA_P, _doc([_e({TESLA_P: {"a_max": 2.8}}, a_max=2.8)])).failsafe

  def test_cars_without_a_version_2_is_invalid_and_so_is_version_3(self, log):
    assert _load(TESLA_P, _doc([_e({TESLA_P: {"a_max": 2.8}})], version=1)).failsafe
    assert _load(TESLA_P, _doc([_e({TESLA_P: {"a_max": 2.8}})], version=3)).failsafe
    assert _load(TESLA_P, _doc([_e({TESLA_P: {"a_max": 2.8}})], version=True)).failsafe

  def test_an_empty_v2_list_is_valid(self, log):
    assert _load(LIGHT_P, _doc([])).entries == [] and log.errors == []

  def test_a_bad_entry_for_the_other_car_still_invalidates_the_file(self, log):
    """One safety document: a typo in the Tesla's entry is an error on the Lightning too (and it falls back to no overrides)."""
    doc = _doc([_e({LIGHT_P: {"a_max": 2.2}}), _e({TESLA_P: {"a_max": 99.0}})])
    assert _load(LIGHT_P, doc).failsafe


class TestV1Compat:
  V1 = {"overrides": [{"lat": 45.0, "lon": -122.0, "radius_m": 100.0, "heading_deg": 90.0, "heading_tol_deg": 30.0,
                       "a_max": 2.8, "note": "old"}]}

  def test_a_v1_entry_is_tesla_only_and_says_so(self, log):
    t = _load(TESLA_P, self.V1)
    assert not t.failsafe and [x["a_max"] for x in t.entries] == [2.8]
    assert any("v1 entry: Tesla only" in w for w in log.warnings) and log.errors == []

  def test_the_lightning_reads_a_v1_file_as_no_overrides_not_failsafe(self, log):
    li = _load(LIGHT_P, self.V1)
    assert not li.failsafe and li.entries == [] and log.errors == []

  def test_v1_is_still_bound_by_the_direction_rules(self, log):
    v1 = json.loads(json.dumps(self.V1))
    v1["overrides"][0]["heading_tol_deg"] = 180.0
    assert _load(TESLA_P, v1).failsafe

  def test_a_v2_file_raises_no_v1_warning(self, log):
    _load(TESLA_P, _doc([_e({TESLA_P: {"a_max": 2.8}})]))
    assert log.warnings == []


# =====================================================================================================
# the Lightning consumer (CurveDbLive.row_a and the ICBM chain end to end)
# =====================================================================================================
def _ovr_at_add_row(cars, **kw):
  """An entry sitting on ADD_ROW's anchor (north 300 m... ADD_ROW is at y=150), heading 0 +- 30."""
  la, lo = _ll(0.0, 150.0)
  return _e(cars, **{"lat": la, "lon": lo, "radius_m": 50.0, "heading_deg": 0.0, "heading_tol_deg": 30.0, **kw})


def _lightning_target(monkeypatch, tmp_path, doc, *, raw=None, lat_a=2.5):
    tmp_path.mkdir(parents=True, exist_ok=True)
    (tmp_path / "curve.json").write_text(json.dumps({"lightning": {"curvedb_v2_lat_a": lat_a}}))
    pathlib.Path(cb.OVERRIDES_PATH).write_text(raw if raw is not None else json.dumps(doc))
    icbm, recs, c = _drive(monkeypatch, tmp_path, points=STRAIGHT, anchors=ADD_ROW, read=lambda: (None, 1))
    got = [t for t in (p.get("target") for p in icbm) if t is not None]
    assert got, "ICBM never targeted: the scenario proves nothing"
    return got[-1], recs, c


class TestLightningConsumer:
  def test_the_baseline_is_sqrt_a_over_k_at_curve_json_a(self, monkeypatch, tmp_path):
    t, recs, _ = _lightning_target(monkeypatch, tmp_path, _doc([]))
    assert t == pytest.approx(math.sqrt(2.5 / K_ADD), abs=0.01)
    assert {r["cdb2OvrN"] for r in recs} == {0} and {r["cdb2Ovr"] for r in recs if r["cdb2Why"] == "ok"} == {None}

  def test_a_lightning_entry_lowers_the_row_speed_and_is_reported(self, monkeypatch, tmp_path):
    t, recs, _ = _lightning_target(monkeypatch, tmp_path, _doc([_ovr_at_add_row({LIGHT_P: {"a_max": 2.0}}, note="ovr note")]))
    assert t == pytest.approx(math.sqrt(2.0 / K_ADD), abs=0.01)
    ok = [r for r in recs if r["cdb2Why"] == "ok"]
    assert ok and {r["cdb2Ovr"] for r in ok} == {"ovr note"} and {r["cdb2OvrN"] for r in recs} == {1}
    assert all(r["cdb2VDb"] == pytest.approx(math.sqrt(2.0 / K_ADD), abs=0.01) for r in ok)

  def test_a_lightning_entry_lowers_a_mapd_rated_candidate_too(self, monkeypatch, tmp_path):
    """The replace() path (mapd rates a 20 m/s curve at y=300, the DB row there is sharper: k 0.008) as well as the scan path."""
    la, lo = _ll(0.0, 300.0)

    def target(doc, sub):
      (tmp_path / sub).mkdir(parents=True, exist_ok=True)
      (tmp_path / sub / "curve.json").write_text(json.dumps({"lightning": {"curvedb_v2_lat_a": 2.5}}))
      pathlib.Path(cb.OVERRIDES_PATH).write_text(json.dumps(doc))
      icbm, _, _ = _drive(monkeypatch, tmp_path / sub, points=_path({300.0: 20.0}), anchors=[_anchor(300.0, 0.008)],
                          read=lambda: (None, 1))
      got = [t for t in (p.get("target") for p in icbm) if t is not None]
      assert got
      return got[-1]
    assert target(_doc([]), "a") == pytest.approx(math.sqrt(2.5 / 0.008), abs=0.01)
    e = _e({LIGHT_P: {"a_max": 2.0}}, lat=la, lon=lo, radius_m=50.0, heading_deg=0.0, heading_tol_deg=30.0)
    assert target(_doc([e]), "b") == pytest.approx(math.sqrt(2.0 / 0.008), abs=0.01)

  def test_a_tesla_entry_never_affects_the_lightning(self, monkeypatch, tmp_path):
    t, recs, _ = _lightning_target(monkeypatch, tmp_path, _doc([_ovr_at_add_row({TESLA_P: {"a_max": 1.5}})]))
    assert t == pytest.approx(math.sqrt(2.5 / K_ADD), abs=0.01)
    assert {r["cdb2OvrN"] for r in recs} == {0}

  def test_it_only_lowers_never_raises(self, monkeypatch, tmp_path):
    t, recs, _ = _lightning_target(monkeypatch, tmp_path, _doc([_ovr_at_add_row({LIGHT_P: {"a_max": 4.0}})]))
    assert t == pytest.approx(math.sqrt(2.5 / K_ADD), abs=0.01)          # the A of curve.json (2.5) stays
    assert {r["cdb2Ovr"] for r in recs if r["cdb2Why"] == "ok"} == {None}

  def test_the_wrong_direction_does_not_match(self, monkeypatch, tmp_path):
    e = _ovr_at_add_row({LIGHT_P: {"a_max": 2.0}}, heading_deg=180.0)     # the row is northbound (brg 0)
    t, _, _ = _lightning_target(monkeypatch, tmp_path, _doc([e]))
    assert t == pytest.approx(math.sqrt(2.5 / K_ADD), abs=0.01)

  def test_a_missing_file_applies_no_overrides_says_so_and_does_not_lower_the_lightning(self, monkeypatch, tmp_path, log):
    monkeypatch.setattr(cb, "OVERRIDES_PATH", str(tmp_path / "gone.json"))
    tmp_path.mkdir(exist_ok=True)
    (tmp_path / "curve.json").write_text(json.dumps({"lightning": {"curvedb_v2_lat_a": 2.5}}))
    icbm, recs, c = _drive(monkeypatch, tmp_path, points=STRAIGHT, anchors=ADD_ROW, read=lambda: (None, 1))
    got = [t for t in (p.get("target") for p in icbm) if t is not None]
    assert got[-1] == pytest.approx(math.sqrt(2.5 / K_ADD), abs=0.01)      # NOT 2.8-capped, NOT lowered
    assert c._roaddb.overrides.failsafe and any("INVALID/MISSING" in e and LIGHT_P in e for e in log.errors)
    assert {r["cdb2OvrN"] for r in recs} == {None}                         # fail-safe is visible in telemetry

  def test_a_corrupt_file_is_the_same(self, monkeypatch, tmp_path, log):
    t, recs, c = _lightning_target(monkeypatch, tmp_path, None, raw="{not json")
    assert t == pytest.approx(math.sqrt(2.5 / K_ADD), abs=0.01) and c._roaddb.overrides.failsafe and log.errors

  def test_a_typo_mid_drive_keeps_the_last_valid_list(self, tmp_path, log):
    o = _load(LIGHT_P, _doc([_e({LIGHT_P: {"a_max": 2.2}})]))
    db = cl.CurveDbLive(True, start=False, overrides=o)
    st = os.stat(cb.OVERRIDES_PATH)
    pathlib.Path(cb.OVERRIDES_PATH).write_text('{"version": 2, "overrides": [{"lat": ')
    os.utime(cb.OVERRIDES_PATH, ns=(st.st_atime_ns, st.st_mtime_ns + 5_000_000_000))
    db.overrides.refresh(2.0)
    assert not o.failsafe and len(o.entries) == 1 and any("NOT applied" in e for e in log.errors)

  def test_row_a_is_lower_only_and_unchanged_without_overrides(self):
    o = _load(LIGHT_P, _doc([_e({LIGHT_P: {"a_max": 2.2}})]))
    mm = types.SimpleNamespace(lat=45.0, lon=-122.0, anchor=0)
    idx = types.SimpleNamespace(anchors=[(45.0, -122.0, 90.0, ())])
    db = cl.CurveDbLive(True, start=False, overrides=o)
    assert db.row_a(idx, mm, 2.5) == (2.2, "t")
    assert db.row_a(idx, mm, 2.0) == (2.0, None) and db.row_a(idx, mm, 2.2) == (2.2, None)
    assert cl.CurveDbLive(True, start=False).row_a(idx, mm, 2.5) == (2.5, None)


# =====================================================================================================
# the Tesla consumer
# =====================================================================================================
def _tesla_step(tmp_path, monkeypatch, doc, raw=None):
  pathlib.Path(cb.OVERRIDES_PATH).write_text(raw if raw is not None else json.dumps(doc))
  b = _brain(tmp_path, monkeypatch)
  return b, _step(b)


class TestTeslaConsumer:
  ENTRY_AT_ROW = dict(lat=LAT0 + 300.0 / 111320.0, lon=LON0, radius_m=200.0, heading_deg=0.0, heading_tol_deg=40.0)

  def test_a_tesla_entry_lowers_a_and_a_lightning_entry_does_nothing(self, tmp_path, monkeypatch, log):
    base = _tesla_step(tmp_path / "a", monkeypatch, _doc([]))[1]
    lowered = _tesla_step(tmp_path / "b", monkeypatch, _doc([_e({TESLA_P: {"a_max": 2.0}}, **self.ENTRY_AT_ROW)]))[1]
    other = _tesla_step(tmp_path / "c", monkeypatch, _doc([_e({LIGHT_P: {"a_max": 1.5}}, **self.ENTRY_AT_ROW)]))[1]
    assert lowered["a"] == pytest.approx(2.0) and lowered["v"] < base["v"]
    assert other["a"] == base["a"] and other["v"] == base["v"]

  def test_a_tesla_entry_above_the_current_a_never_raises(self, tmp_path, monkeypatch, log):
    base = _tesla_step(tmp_path / "a", monkeypatch, _doc([]))[1]
    hi = _tesla_step(tmp_path / "b", monkeypatch, _doc([_e({TESLA_P: {"a_max": 4.9}}, **self.ENTRY_AT_ROW)]))[1]
    assert hi["a"] == base["a"] and hi["v"] == base["v"]

  def test_the_wrong_heading_does_not_match(self, tmp_path, monkeypatch, log):
    base = _tesla_step(tmp_path / "a", monkeypatch, _doc([]))[1]
    e = _e({TESLA_P: {"a_max": 2.0}}, **{**self.ENTRY_AT_ROW, "heading_deg": 180.0})
    assert _tesla_step(tmp_path / "b", monkeypatch, _doc([e]))[1]["v"] == base["v"]

  def test_tesla_failsafe_is_2_8_everywhere_on_a_missing_or_invalid_file(self, tmp_path, monkeypatch, log):
    out = _tesla_step(tmp_path / "a", monkeypatch, None, raw='{"version": 2, "overrides": [{"cars": {"TOYOTA_COROLLA": {"a_max": 2}}}]}')[1]
    assert out["a"] <= cb.FAILSAFE_A + 1e-9 and log.errors

  def test_the_v1_seed_form_still_keeps_the_tesla_out_of_failsafe(self, tmp_path, monkeypatch, log):
    v1 = {"overrides": [{**self.ENTRY_AT_ROW, "a_max": 2.0, "note": "n"}]}
    b, out = _tesla_step(tmp_path, monkeypatch, v1)
    assert out["a"] == pytest.approx(2.0) and not b.overrides.failsafe and log.errors == []
    assert any("v1 entry: Tesla only" in w for w in log.warnings)

  def test_the_brain_is_refused_for_a_car_without_the_capability(self, tmp_path, monkeypatch):
    with pytest.raises(ValueError):
      cb.CurveBrain(pv.PnwVehicle(FakeCP(LIGHT_P.replace("FORD", "XFORD"), "ford", False)))


# =====================================================================================================
# the real seed file against the real table
# =====================================================================================================
_SEED = pathlib.Path(os.path.expanduser("~/gh/comma/workdir/data/curve_overrides.json"))
_TABLE = pathlib.Path(os.path.expanduser("~/gh/comma/workdir/data/curvedb_v2"))
_OR34_WB = (44.5604, -123.1210)
_OR34_EB = (44.5606, -123.1217)
# The Lightning OR-34 EB right: 2.2 was measured to sit INSIDE the PSCM limit band (achLat 2.37-2.46 at k 0.00256 -> ~2.39), so the
# seed must stay strictly BELOW 2.2 (and, lower-only, at or under the curve-DB A of 2.5). The exact private value is read from the seed.
_LIGHT_KNOWN_BAD_A = 2.2
_TESLA_DEFAULT_A = 2.8


def _seed_a(plat):
  """The a_max values the private seed holds for one platform, in file order (read, never hard-coded)."""
  return [e["cars"][plat]["a_max"] for e in json.loads(_SEED.read_text())["overrides"] if plat in e["cars"]]


@pytest.mark.skipif(not (_SEED.exists() and _TABLE.exists() and _seed_is_v2()), reason=_SEED_SKIP)
class TestTheSeedAgainstTheRealTable:
  @pytest.fixture(scope="class")
  def idx(self):
    return cl.load_rows(str(_TABLE))[0]

  def _lim(self, plat, a):
    return cb.Overrides(str(_SEED), platform=plat).limit(a[0], a[1], a[2])

  def test_the_seed_is_v2_valid_for_both_cars_with_the_expected_entry_counts(self, log):
    doc = json.loads(_SEED.read_text())
    assert doc["version"] == 2 and len(doc["overrides"]) == 3
    assert all(0.0 < e["heading_tol_deg"] <= 60.0 and set(e["cars"]) <= set(pv.CURVE_OVERRIDE_PLATFORMS) for e in doc["overrides"])
    t, li = cb.Overrides(str(_SEED), platform=TESLA_P), cb.Overrides(str(_SEED), platform=LIGHT_P)
    assert not t.failsafe and not li.failsafe and log.errors == [] and log.warnings == []
    assert [x["a_max"] for x in t.entries] == _seed_a(TESLA_P) and len(t.entries) == 2
    assert [x["a_max"] for x in li.entries] == _seed_a(LIGHT_P) and len(li.entries) == 1

  def test_the_lightning_a_max_is_below_the_known_bad_value_and_every_entry_is_lower_only(self):
    (a_l,) = _seed_a(LIGHT_P)
    assert 0.0 < a_l < _LIGHT_KNOWN_BAD_A          # 2.2 ~ 2.39 m/s2 at the measured k: inside the PSCM limit band
    assert all(0.0 < a <= _TESLA_DEFAULT_A for a in _seed_a(TESLA_P))
    assert any(a < _TESLA_DEFAULT_A for a in _seed_a(TESLA_P))

  def test_no_row_is_limited_for_both_cars_and_each_car_has_its_own_rows(self, idx):
    tes, lig = set(), set()
    tes_a, (lig_a,) = set(_seed_a(TESLA_P)), _seed_a(LIGHT_P)
    for i, a in enumerate(idx.anchors):
      t, li = self._lim(TESLA_P, a)[0], self._lim(LIGHT_P, a)[0]
      assert t is None or li is None, (a[0], a[1], a[2])
      if t is not None:
        assert t in tes_a
        tes.add(i)
      if li is not None:
        assert li == lig_a
        lig.add(i)
    assert len(tes) == 19 and len(lig) == 9           # 11 Terwilliger + 8 OR-34 WB (Tesla); the 9 OR-34 EB rows (Lightning)

  def test_the_lightning_entry_matches_only_the_eastbound_bend(self, idx):
    lig = cb.Overrides(str(_SEED), platform=LIGHT_P)
    hit = [a for a in idx.anchors if lig.limit(a[0], a[1], a[2])[0] is not None]
    assert len(hit) == 9
    assert all(80.0 <= a[2] <= 140.0 and cb._dist_m(a[0], a[1], *_OR34_EB) <= 220.0 for a in hit)
    # DIRECTION: every westbound row of the same curve is left alone (they share the circle)
    wb = [a for a in idx.anchors if cb._dist_m(a[0], a[1], *_OR34_EB) <= 300.0 and 250.0 <= a[2] <= 330.0]
    assert len(wb) >= 8 and all(lig.limit(a[0], a[1], a[2])[0] is None for a in wb)

  def test_the_westbound_tesla_entries_never_match_an_eastbound_row(self, idx):
    tes = cb.Overrides(str(_SEED), platform=TESLA_P)
    eb = [a for a in idx.anchors if cb._dist_m(a[0], a[1], *_OR34_WB) <= 400.0 and 80.0 <= a[2] <= 140.0]
    assert len(eb) >= 8 and all(tes.limit(a[0], a[1], a[2])[0] is None for a in eb)

  def test_terwilliger_and_or34_wb_are_tesla_only_and_or34_eb_is_lightning_only(self, idx):
    tes, lig = cb.Overrides(str(_SEED), platform=TESLA_P), cb.Overrides(str(_SEED), platform=LIGHT_P)
    for e in tes.entries:
      assert lig.limit(e["lat"], e["lon"], e["heading_deg"]) == (None, None)
    (e,) = lig.entries
    assert tes.limit(e["lat"], e["lon"], e["heading_deg"]) == (None, None)

  @pytest.mark.usefixtures("cfgpath", "schedule")
  def test_priced_speeds_per_car(self, idx):
    """The speed each car is priced at, per matched row: the Lightning at its curve-DB A 2.5 lowered to the seed's a_max
    (sqrt(A/k), the expected speed computed from the a_max READ from the seed), the Tesla at min(its A, the entry's a_max)
    through row_speed (the same call the brain makes)."""
    (lig_a,) = _seed_a(LIGHT_P)
    lig = cb.Overrides(str(_SEED), platform=LIGHT_P)
    db = cl.CurveDbLive(True, start=False, overrides=lig)
    got = {}
    for i, a in enumerate(idx.anchors):
      k = max((b[2] for b in a[3] if b[2]), default=None)
      if lig.limit(a[0], a[1], a[2])[0] is None or k is None:
        continue
      a_row, note = db.row_a(idx, types.SimpleNamespace(lat=a[0], lon=a[1], anchor=i), 2.5)
      assert a_row == lig_a and note and "OR-34 EB" in note
      got[round(a[0], 5)] = (round(cl.v_db(2.5, k) / MPH, 1), round(cl.v_db(a_row, k) / MPH, 1), k)
    assert len(got) == 8
    for v0, v1, k in got.values():
      assert v1 == round(math.sqrt(lig_a / k) / MPH, 1) and v1 < v0          # priced at sqrt(a_max / k), never above the baseline
    worst = max(got.values(), key=lambda t: t[2])                            # the tightest covered row, read from the table
    assert worst[0] == round(math.sqrt(2.5 / worst[2]) / MPH, 1)             # baseline = sqrt(2.5 / worst k), not a pinned number
    assert worst[1] == round(math.sqrt(lig_a / worst[2]) / MPH, 1)
    assert min(v for _, v, _ in got.values()) == worst[1]
    assert max(v for v, _, _ in got.values()) > 100

    tes, veh = cb.Overrides(str(_SEED), platform=TESLA_P), tesla()
    v_ego, seen, kmax = 31.0, {}, {}
    for a in idx.anchors:
      cap, note = tes.limit(a[0], a[1], a[2])
      k = max((b[2] for b in a[3] if b[2]), default=None)
      if cap is None or k is None:
        continue
      v, a_used = cb.row_speed(veh, k, v_ego, cap)
      v0, _ = cb.row_speed(veh, k, v_ego)
      assert a_used <= cap + 1e-9 and v <= v0 + 1e-9     # lower-only in A AND in speed (row_speed guards the second)
      seen.setdefault(cap, []).append(round(v / MPH, 1))
      kmax[cap] = max(kmax.get(cap, 0.0), k)
    lo, hi = min(seen), max(seen)
    assert hi == _TESLA_DEFAULT_A
    # the slowest priced speed of each cap group is sqrt(cap / its tightest k), derived from the table (the schedule fixture's
    # A at v_ego and at the row speed is >= cap, so the override binds)
    for cap in (hi, lo):
      assert min(seen[cap]) == round(math.sqrt(cap / kmax[cap]) / MPH, 1), cap
