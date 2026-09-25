"""curveshape2pnw 1/3 -- the pure measured-shape pricing (icbm_shape.py). CURVE-MEASURED-SHAPE-DESIGN.md s3-s4.

The named curves are the design's s2.2 table: mapd's rating, the polyline's peak curvature, and what the truck measured
(truth). No coordinates -- only the numbers.
"""
import math

import pytest

from openpilot.selfdrive.controls.lib.ces_pnw import icbm_shape as s
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import icbm_map_eff_scale

MPH = s.MPH
A_MAPD = 2.0            # the truck's mapd (device-verified, CURVEDB-V2-LIVE)
A = 2.5                 # owner answer 4
MAP_SCALE = 0.92        # the Lightning's shipped map_scale


def legacy(raw_mph):
  raw = raw_mph * MPH
  return icbm_map_eff_scale(raw) * raw * MAP_SCALE


def price(raw_mph, k_poly, *, d=300.0, kd=None, kn=6, ahead=True, stable=True, a_mapd=A_MAPD, posted_mph=70.0,
          set_mph=85.0, a=A, a_hi=A):
  raw = raw_mph * MPH
  return s.shape_price(raw, d, a_mapd, (k_poly, d if kd is None else kd, kn, ahead), stable, legacy(raw_mph),
                       posted_mph * MPH if posted_mph else posted_mph, set_mph * MPH, a, a_hi)


# (name, mapd rating mph, polyline k, truth k, expected why, expected shape speed mph or None)
S22 = [
  ("OR-34 11:18 left", 68.0, 0.00271, 0.00261, "ok", 71.6),
  ("OR-34 11:19 right (60.4 point)", 60.4, 0.00275, 0.00256, "ok", 67.5),
  ("OR-34 11:19:38 left", 67.1, 0.00242, 0.00248, "ok", 73.4),
  ("Terwilliger A right", 47.4, 0.00329, 0.00391, "rawLow", None),
  ("Terwilliger B left", 49.0, 0.00453, 0.00429, "rawLow", None),
  ("Tumwater 09-21 left", 59.9, 0.00316, 0.0031, "ok", 64.9),
  ("I-5 46.12 09-24", 65.5, 0.00204, 0.00215, "ok", 75.6),
  ("I-5 45.72 phantom (low poly)", 61.1, 0.00097, 0.0011, "mapSharper", None),
  ("I-5 45.72 phantom (high poly)", 61.1, 0.00136, 0.0011, "mapSharper", None),
  ("I-5 46.64 09-21 (poly wrong)", 73.8, 0.00433, 0.00075, "polySharper", None),
  ("I-5 44.85 09-24 (poly wrong)", 95.3, 0.00281, 0.00125, "polySharper", None),
]


@pytest.mark.parametrize("name,raw,kp,truth,why,want", S22, ids=[r[0] for r in S22])
def test_the_design_table(name, raw, kp, truth, why, want):
  v, got_why, dr, k, km = price(raw, kp)
  assert got_why == why, name
  if want is None:
    assert v is None and dr == "none"
    return
  assert v / MPH == pytest.approx(want, abs=0.1), name
  assert dr == "lower" and v < legacy(raw)             # every agreeing s2.2 curve LOWERS today's inflated price
  assert k == pytest.approx(0.5 * (kp + A_MAPD / (raw * MPH) ** 2), rel=1e-12)   # the MEAN (owner answer 1)
  # and it lands within the design's +-25 % of what the truck measured
  assert 0.75 <= k / truth <= 1.34, name


def test_or34_1118_now_binds_below_the_set_it_used_to_be_discarded_at():
  """G1's mechanism: today's 84.5 mph is above the 78 set (discarded); the shape price, 71.6, binds."""
  assert legacy(68.0) / MPH == pytest.approx(84.5, abs=0.1)
  v, *_ = price(68.0, 0.00271, set_mph=78.0)
  assert v / MPH <= 72.5


def test_the_mean_is_neither_the_sharper_nor_mapd_alone():
  v, _, _, k, km = price(60.4, 0.00300)
  mean = math.sqrt(A / (0.5 * (0.00300 + km)))
  assert v == pytest.approx(mean, rel=1e-12)
  assert v > math.sqrt(A / max(0.00300, km)) + 0.1 and abs(v - math.sqrt(A / km)) > 0.1


class TestBand:
  @pytest.mark.parametrize("ratio,why", [(1.49, "ok"), (1.51, "polySharper"), (1 / 1.49, "ok"), (1 / 1.51, "mapSharper")])
  def test_the_edges(self, ratio, why):
    km = A_MAPD / (65.0 * MPH) ** 2
    assert price(65.0, km * ratio)[1] == why

  def test_the_band_is_x1_5_not_x1_35(self):
    km = A_MAPD / (65.0 * MPH) ** 2
    assert price(65.0, km * 1.4)[1] == "ok"
    assert price(65.0, km / 1.4)[1] == "ok"


class TestConfidence:
  def test_every_why_is_reachable(self):
    ok = 0.5 * A_MAPD / (65.0 * MPH) ** 2 * 2.0          # agrees exactly
    seen = {
      price(65.0, ok)[1],
      price(49.9, ok)[1],                                 # rawLow
      price(65.0, ok, a_mapd=None)[1],                    # noA
      price(65.0, ok, kn=2)[1],                           # sparse
      price(65.0, 0.0)[1],                                # sparse (nothing measured)
      price(65.0, ok, ahead=False)[1],                    # notAhead
      price(65.0, ok, d=50.0, kd=50.0)[1],                # near
      price(65.0, ok, d=300.0, kd=460.0)[1],              # farApart
      price(65.0, ok, stable=False)[1],                   # unstable
      price(61.1, 0.001)[1],                              # mapSharper
      price(73.8, 0.00433)[1],                            # polySharper
      price(52.0, A_MAPD / (52.0 * MPH) ** 2 * 0.7, posted_mph=0.0)[1],   # noPosted (a raise, no posted limit)
      s.shape_price(float("nan"), 300.0, 2.0, (0.002, 300.0, 5, True), True, 30.0, 30.0, 35.0, A, A)[1],  # badInput
    }
    assert seen == {"ok", "rawLow", "noA", "sparse", "notAhead", "near", "farApart", "unstable", "mapSharper",
                    "polySharper", "noPosted", "badInput"}

  def test_the_50_mph_gate(self):
    km = A_MAPD / (50.0 * MPH) ** 2
    assert price(50.0, km)[1] == "ok"
    assert price(49.95, km)[1] == "rawLow"

  @pytest.mark.parametrize("d,kd,why", [(300.0, 150.0, "ok"), (300.0, 149.0, "farApart"), (300.0, 451.0, "farApart"),
                                        (60.0, 60.0, "ok"), (59.0, 59.0, "near")])
  def test_the_distance_rules(self, d, kd, why):
    km = A_MAPD / (65.0 * MPH) ** 2
    assert price(65.0, km, d=d, kd=kd)[1] == why

  @pytest.mark.parametrize("bad", [None, float("nan"), float("inf"), "x", True])
  def test_bad_input_keeps_todays_price(self, bad):
    for args in ((bad, 300.0, 2.0, (0.002, 300.0, 5, True), True, 30.0),
                 (30.0, bad, 2.0, (0.002, 300.0, 5, True), True, 30.0),
                 (30.0, 300.0, 2.0, (0.002, 300.0, 5, True), True, bad),
                 (30.0, 300.0, 2.0, (bad, 300.0, 5, True), True, 30.0),
                 (30.0, 300.0, 2.0, bad, True, 30.0)):
      v, why, dr, _, _ = s.shape_price(*args, 30.0, 35.0, A, A)
      assert v is None and dr == "none" and why in ("badInput", "sparse"), args


class TestRaise:
  """mapd sharper than the polyline but inside the band: the price RISES, capped."""

  def km(self, raw_mph):
    return A_MAPD / (raw_mph * MPH) ** 2

  def test_the_8_mph_cap(self):
    # raw 62, poly reads 0.7x mapd: v = sqrt(2.5 / (0.85 km)) = 75.2 mph; today 62 * 1.35 * 0.92 = 77.0 -> that is a
    # lowering. Use a tight-ish rating where today's price is near raw: 52 mph -> today 52 * 1.144 * 0.92 = 54.7.
    kp = self.km(52.0) * 0.7
    v, why, dr, _, _ = price(52.0, kp, posted_mph=80.0, set_mph=90.0)
    want = math.sqrt(A / (0.5 * (kp + self.km(52.0))))
    assert want > legacy(52.0) + s.RAISE_CAP_MS
    assert (why, dr) == ("ok", "raise") and v == pytest.approx(legacy(52.0) + s.RAISE_CAP_MS, abs=1e-9)

  def test_the_posted_plus_10_cap(self):
    kp = self.km(52.0) * 0.7
    v, _, dr, _, _ = price(52.0, kp, posted_mph=50.0, set_mph=90.0)
    assert dr == "raise" and v == pytest.approx(60.0 * MPH, abs=1e-9)

  def test_the_set_caps_it(self):
    kp = self.km(52.0) * 0.7
    v, _, dr, _, _ = price(52.0, kp, posted_mph=80.0, set_mph=57.0)
    assert dr == "raise" and v == pytest.approx(57.0 * MPH, abs=1e-9)

  def test_no_posted_limit_no_raise(self):
    kp = self.km(52.0) * 0.7
    for posted in (0.0, None):
      v, why, dr, _, _ = price(52.0, kp, posted_mph=posted)
      assert (why, dr) == ("noPosted", "held") and v == pytest.approx(legacy(52.0), abs=1e-12)

  def test_a_posted_limit_below_todays_price_holds_it(self):
    kp = self.km(52.0) * 0.7
    v, why, dr, _, _ = price(52.0, kp, posted_mph=40.0)       # posted + 10 = 50 < today's 54.7
    assert dr == "held" and v == pytest.approx(legacy(52.0), abs=1e-12)

  def test_a_lowering_is_exact_and_uncapped(self):
    kp = self.km(68.0) * 1.25
    v, _, dr, k, _ = price(68.0, kp, posted_mph=0.0)          # no posted limit does not block a LOWERING
    assert dr == "lower" and v == pytest.approx(math.sqrt(A / k), rel=1e-12)


class TestHighSpeedTarget:
  def test_a_hi_applies_only_at_or_above_70(self):
    kp = A_MAPD / (68.0 * MPH) ** 2 * 1.2
    v_std = price(68.0, kp)[0]
    assert v_std / MPH > 70.0
    v_hi = price(68.0, kp, a_hi=2.2)[0]
    assert v_hi == pytest.approx(math.sqrt(2.2 / price(68.0, kp)[3]), rel=1e-12)
    kp_slow = A_MAPD / (58.0 * MPH) ** 2
    assert price(58.0, kp_slow, a_hi=2.2)[0] == price(58.0, kp_slow)[0]      # below 70: a_hi not used


class TestLatch:
  def test_stable_only_on_the_second_consistent_refresh(self):
    lt = s.ShapeLatch()
    assert lt.update(100.0, 0.0025, 400.0, 6, True, 30.0) is False
    assert lt.update(100.0, 0.0025, 400.0, 6, True, 30.0) is False        # the same refresh again: nothing new
    assert lt.update(101.0, 0.0026, 372.0, 6, True, 30.0) is True         # 28 m closer at 30 m/s: consistent
    assert lt.update(101.0, 0.0026, 372.0, 6, True, 30.0) is True

  @pytest.mark.parametrize("k2,kd2,dt,why", [
    (0.0025 * 1.36, 370.0, 1.0, "curvature jumped"),
    (0.0025, 400.0 - 30.0 + 41.0, 1.0, "peak did not approach"),
    (0.0025, 400.0 - 30.0 - 41.0, 1.0, "peak jumped closer"),
    (0.0025, 400.0 - 90.0, 3.0, "refreshes too far apart"),
  ])
  def test_an_inconsistent_second_reading_is_unstable(self, k2, kd2, dt, why):
    lt = s.ShapeLatch()
    lt.update(100.0, 0.0025, 400.0, 6, True, 30.0)
    assert lt.update(100.0 + dt, k2, kd2, 6, True, 30.0) is False, why

  def test_the_band_edges_hold(self):
    lt = s.ShapeLatch()
    lt.update(100.0, 0.0025, 400.0, 6, True, 30.0)
    assert lt.update(101.0, 0.0025 * 1.34, 370.0 + 39.0, 6, True, 30.0) is True

  @pytest.mark.parametrize("bad", [dict(kn=2), dict(ahead=False), dict(k=0.0), dict(t=None), dict(k=float("nan"))])
  def test_an_invalid_reading_resets(self, bad):
    lt = s.ShapeLatch()
    lt.update(100.0, 0.0025, 400.0, 6, True, 30.0)
    assert lt.update(101.0, 0.0025, 370.0, 6, True, 30.0) is True
    r = dict(t=102.0, k=0.0025, kd=340.0, kn=6, ahead=True)
    r.update(bad)
    assert lt.update(r["t"], r["k"], r["kd"], r["kn"], r["ahead"], 30.0) is False
    assert lt.update(103.0, 0.0025, 310.0, 6, True, 30.0) is False, "the invalid reading must break the chain"


class TestPricer:
  def mk(self, stable=True, a=A_MAPD, reading=(0.00271, 300.0, 6, True)):
    return s.ShapePricer(icbm_map_eff_scale, MAP_SCALE, a, reading, stable, 70.0 * MPH, 85.0 * MPH, A, A)

  def test_an_unpriced_candidate_is_bit_identical_to_todays_product(self):
    p = self.mk(stable=False)
    for tv in (15.0, 22.35, 25.0, 26.82, 30.4, 44.0):
      assert p(tv, 300.0) == icbm_map_eff_scale(tv) * tv * MAP_SCALE      # ==, not approx
      assert p.priced(tv, 300.0) is None
    assert not p.any_priced

  def test_a_priced_candidate_and_its_floor(self):
    p = self.mk()
    raw = 68.0 * MPH
    assert p(raw, 300.0) / MPH == pytest.approx(71.6, abs=0.1)
    assert p.priced(raw, 300.0) == p(raw, 300.0) and p.any_priced

  def test_a_defect_costs_the_shape_price_not_the_candidate(self, monkeypatch):
    p = self.mk()

    def boom(*a, **k):
      raise ZeroDivisionError("bug")
    monkeypatch.setattr(s, "shape_price", boom)
    raw = 68.0 * MPH
    assert p(raw, 300.0) == icbm_map_eff_scale(raw) * raw * MAP_SCALE
    assert p.priced(raw, 300.0) is None and p.err == "ZeroDivisionError"

  def test_decisions_are_cached(self, monkeypatch):
    p = self.mk()
    calls = []
    real = s.shape_price
    monkeypatch.setattr(s, "shape_price", lambda *a: calls.append(a) or real(*a))
    for _ in range(3):
      p(68.0 * MPH, 300.0)
      p.priced(68.0 * MPH, 300.0)
    assert len(calls) == 1


def test_tick_gate():
  ramps, unknown = ("motorwayLink",), ("", "unknown")
  assert s.tick_gate("current", "motorway", "raw", ramps, unknown) is None
  assert s.tick_gate("current", "motorway", "proj", ramps, unknown) is None
  assert s.tick_gate("current", "motorway", "stale", ramps, unknown) == "gps"
  assert s.tick_gate("current", "motorway", "none", ramps, unknown) == "gps"
  assert s.tick_gate("predicted", "motorway", "raw", ramps, unknown) == "waySel"
  assert s.tick_gate("current", "motorwayLink", "raw", ramps, unknown) == "class"
  assert s.tick_gate("current", None, "raw", ramps, unknown) == "class"
  assert s.tick_gate("current", "unknown", "raw", ramps, unknown) == "class"
