"""icbmdbraise2pnw: a RUNNING ICBM cap may be lifted back up by a curve-DB raise (drive 2026-10-19, Terwilliger right curve).

The drive: the DB read `branchAmbiguous` for 7 s, the target fell to mapd's own 46.9 mph and the set was walked 52 -> 47; the DB
then came back with raise rows (52.6 / 53.6 / 56.5) and the running cap, which is dec-only, left the set at 47 for 15 s more.

Pinned here, each by a test that fails if the rule is broken:
  * that sequence, replayed through IcbmEpisode with an executor that taps the set: the set rises to the DB target, never above
    the ceiling C nor the posted limit + 5 mph;
  * only the DB's own raise/add lifts a cap: mapd's or vision's raise during a cap is still refused;
  * the left-curve sequence (no DB number, mapd 50.6 throughout) is unchanged: dec only;
  * a flapping DB raise never taps (it must read stable for ICBM_RATCHET_CONFIRM_S); vision can still lower it;
  * an unknown limit is no raise, and says so (icbmDbRaise "noLimit");
  * the controller flags only a DB-won decision (CurveDbLive.backed); the Tesla has no DB, so no raise.
"""
import pytest

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw import curvedb_live as cl
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import ICBM_DB_RAISE_OVER_MS, IcbmEpisode
from openpilot.selfdrive.controls.lib.ces_pnw.tests.roaddb_fixture import FAR_ANCHOR
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvedblive2pnw import (ADD_ROW, LOWER_ROW, PHANTOM, RAISE_ROW, STRAIGHT, TESLA,
                                                                                 _drive)

MPH = 0.44704
DT = 0.25
T0 = 1000.0


class Truck:
  """An IcbmEpisode driven at 4 Hz with an executor that taps the stock set 1 mph per tick toward the published target."""

  def __init__(self, set_mph, limit_mph=50.0):
    self.ep = IcbmEpisode()
    self.t = T0
    self.set = set_mph * MPH
    self.limit = limit_mph * MPH
    self.dirs = []
    self.incs = []                      # every published raise target (mph)
    self.sets = []

  def tick(self, cap_mph, db_mph=None, n=1, limit=...):
    lim = self.limit if limit is ... else limit
    for _ in range(n):
      cap = None if cap_mph is None else cap_mph * MPH
      db = None if db_mph is None else db_mph * MPH
      self.ep.note_limit(lim, proportional=False)
      tgt, d = self.ep.step(self.t, cap, self.set, self.set, True, False, limit_now=lim, db_raise_t=db)
      self.dirs.append(d)
      if d == "inc":
        self.incs.append(tgt / MPH)
      if tgt is not None and d == "dec" and self.set > tgt + 0.5 * MPH:
        self.set -= MPH
      elif tgt is not None and d == "inc" and self.set < tgt - 0.5 * MPH:
        self.set += MPH
      self.sets.append(self.set / MPH)
      self.t += DT


def _terwilliger(limit_mph=50.0, ceiling=61.0, low=47.0):
  """The right-curve sequence up to the end of the branchAmbiguous window: set 61 walked to 47 by mapd's 46.9."""
  tr = Truck(ceiling, limit_mph)
  tr.tick(56.2)
  tr.tick(52.3, n=8)
  tr.tick(low - 0.1, n=28)                # branchAmbiguous: the DB number is gone, mapd's own rating binds (no db_raise_t)
  assert round(tr.set / MPH) == round(low) and tr.ep.phase == "cap"
  return tr


class TestTerwilliger:
  def test_the_set_rises_to_the_db_target_and_stops_at_limit_plus_5(self):
    tr = _terwilliger()
    n0 = len(tr.sets)
    tr.tick(52.6, db_mph=52.6, n=12)
    assert 52 <= tr.sets[-1] <= 53 + 1e-6 and "inc" in tr.dirs[n0:]
    tr.tick(53.6, db_mph=53.6, n=8)
    tr.tick(56.5, db_mph=56.5, n=12)
    assert tr.sets[-1] == pytest.approx(55.0, abs=0.01)          # limit 50 + 5, not the DB's 56.5
    assert max(tr.sets[n0:]) <= 50.0 + ICBM_DB_RAISE_OVER_MS / MPH + 1e-6
    assert tr.ep.db_raise == "capped"                            # at the bound: the DB wants more, the bound says no

  def test_never_above_the_episode_ceiling(self):
    tr = _terwilliger(limit_mph=60.0, ceiling=50.0)              # C = 50 < DB 56 < limit + 5
    tr.tick(56.0, db_mph=56.0, n=20)
    assert tr.incs and max(tr.incs) <= 50.0 + 1e-6      # the PUBLISHED target, not what a clamping executor would do with it
    assert tr.sets[-1] == pytest.approx(50.0, abs=0.01)

  def test_limit_plus_5_binds_below_the_ceiling(self):
    tr = _terwilliger(limit_mph=40.0, ceiling=61.0, low=38.0)
    tr.tick(56.0, db_mph=56.0, n=20)
    assert tr.incs and max(tr.incs) <= 45.0 + 1e-6 and tr.sets[-1] == pytest.approx(45.0, abs=0.01)

  def test_the_raise_waits_for_the_confirmation(self):
    tr = _terwilliger()
    tr.tick(52.6, db_mph=52.6)
    assert tr.dirs[-1] == "dec" and tr.ep.db_raise == "wait"
    tr.tick(52.6, db_mph=52.6, n=4)
    assert "inc" in tr.dirs[-4:]


class TestOnlyTheDbRaises:
  def test_a_mapd_or_vision_raise_during_a_cap_is_still_refused(self):
    tr = _terwilliger()
    tr.tick(52.6, db_mph=None, n=40)                             # the target rose, but no DB number made it
    assert tr.dirs[-1] == "dec" and "inc" not in tr.dirs[36:] and round(tr.set / MPH) == 47
    assert tr.ep.db_raise is None

  def test_the_left_curve_is_unchanged(self):
    tr = Truck(61.0)
    tr.tick(55.8)
    tr.tick(50.6, n=30)                                          # branchAmbiguous throughout, mapd 50.6, no DB number
    assert set(tr.dirs) == {"dec"} and round(tr.set / MPH) == 51

  def test_vision_can_still_lower_a_raise(self):
    tr = _terwilliger()
    tr.tick(49.0, db_mph=52.6, n=12)                             # the final cap (vision-lowered) is 49
    assert tr.sets[-1] == pytest.approx(49.0, abs=0.01)

  def test_flapping_never_taps(self):
    tr = _terwilliger()
    for _ in range(10):
      tr.tick(52.6, db_mph=52.6)
      tr.tick(52.6, db_mph=None)
    for _ in range(5):
      tr.tick(52.6, db_mph=52.6, n=2)
      tr.tick(52.6, db_mph=None, n=2)
    assert "inc" not in tr.dirs and round(tr.set / MPH) == 47

  def test_an_unknown_limit_is_no_raise_and_says_so(self):
    tr = _terwilliger()
    tr.tick(52.6, db_mph=52.6, n=12, limit=None)
    assert "inc" not in tr.dirs and tr.ep.db_raise == "noLimit"

  def test_a_new_episode_never_raises_on_its_first_tick(self):
    tr = Truck(61.0)
    tr.tick(46.9, db_mph=52.6, n=1)
    assert tr.dirs == ["dec"]

  def test_no_cap_no_raise_state(self):
    tr = _terwilliger()
    tr.tick(52.6, db_mph=52.6, n=3)
    tr.tick(None, n=1)
    assert tr.ep.db_raise is None and tr.ep._dbr_t0 is None


def _flags(monkeypatch, tmp_path, **kw):
  """(cdb2Dir, backed) of every DB decision of a drive."""
  seen = []
  real = cl.CurveDbLive.decide

  def spy(self, **k):
    out = real(self, **k)
    seen.append((self._last["cdb2Dir"], self.backed))
    return out
  monkeypatch.setattr(cl.CurveDbLive, "decide", spy)
  _drive(monkeypatch, tmp_path, **kw)
  return seen


class TestController:
  def test_only_a_db_won_decision_is_flagged(self, monkeypatch, tmp_path):
    raise_ = _flags(monkeypatch, tmp_path / "raise", points=PHANTOM, anchors=RAISE_ROW)
    assert ("raise", True) in raise_ and all(b == (d == "raise") for d, b in raise_ if d != "none")
    add = _flags(monkeypatch, tmp_path / "add", points=STRAIGHT, anchors=ADD_ROW)
    assert ("add", True) in add
    lower = _flags(monkeypatch, tmp_path / "lower", points=PHANTOM, anchors=LOWER_ROW)
    assert ("lower", False) in lower and not any(b for d, b in lower if d == "lower")   # a lowering is not a raise
    none = _flags(monkeypatch, tmp_path / "none", points=PHANTOM, anchors=[FAR_ANCHOR])
    assert none and not any(b for _, b in none)                   # no row: mapd's own number

  def test_the_tesla_has_no_db_so_no_raise(self, monkeypatch, tmp_path):
    _, recs, c = _drive(monkeypatch, tmp_path / "t", points=PHANTOM, anchors=RAISE_ROW, fp=TESLA, brand="tesla")
    assert not c._roaddb.enabled and not c._roaddb.backed
    assert {r.get("icbmDbRaise") for r in recs} <= {None}

  def test_the_field_reaches_the_record(self, monkeypatch, tmp_path):
    _, recs, _ = _drive(monkeypatch, tmp_path / "f", points=PHANTOM, anchors=RAISE_ROW)
    assert recs and all("icbmDbRaise" in r for r in recs)
    assert m.IcbmEpisode().db_raise is None
