"""curvelead2pnw -- ICBM follows a tracked lead through a curve (A); the map-claim sanity checks stay telemetry (B).

THE DRIVER, 2026-09-13: "if there's a lead car that takes a curve at a certain speed, why don't you just follow
that instead of making up your own mind ... we trust that the lead car is typically not suicidal." And: "going
like 50 plus on a country road and I'm heading towards a curve and the car suddenly loses 20 mph."

What is pinned here, and why each group exists:
  * the pure gates of icbm_lead_pace, one test per guard, each written so the guard's removal FAILS it;
  * the real weekend telemetry (tests/data/curvelead_2026-09.json), including Sun 13:57 -- a lead tracked
    toward an R~36 m ramp and lost at its entry, where ANY one of three gates must refuse on its own;
  * the wiring through CESController._icbm_step, and a closed loop through the real Ford executor
    (icbm_pnw.arbitrate / decide_press / PressGovernor): the set is held at the lead pace, and after the
    lead is lost it is walked back down to ICBM's own target;
  * B stays out of control: its verdicts reach the record and never the published target;
  * the Tesla (op-long, no ICBM) never computes or reads any of it.
"""
import inspect
import json
import math
import os
import types

import pytest

from opendbc.car.ford.icbm_pnw import (IcbmCommand, PressGovernor, STEP_MS, arbitrate, decide_press)
from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import (
  CURVELEAD_TELE_KEYS, ICBM_LEAD_CONT_S, ICBM_RATCHET_CONFIRM_S, IcbmEpisode, IcbmLeadTrack,
  icbm_lead_pace, icbm_map_sanity, icbm_path_behind, icbm_vision_curvature, upcoming_curve,
  vision_curve_lat_accel)
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle
from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_pnw import polyline_curvature

MPH = 0.44704
LIGHTNING = "FORD_F_150_LIGHTNING_MK1"
FIXTURE = os.path.join(os.path.dirname(__file__), "data", "curvelead_2026-09.json")


class FakeCP:
  def __init__(self, fp, brand, op_long):
    self.carFingerprint = fp
    self.brand = brand
    self.openpilotLongitudinalControl = op_long
    self.dashcamOnly = False


@pytest.fixture
def default_curve_cfg(tmp_path, monkeypatch):
  """Every Lightning here reads the built-in curve.json defaults, never a file on the dev host."""
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "absent.json"))
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))


def _lightning():
  return PnwVehicle(FakeCP(LIGHTNING, "ford", False))


# ---------------------------------------------------------------------------------------------------------
# icbm_lead_pace: one guard per test
# ---------------------------------------------------------------------------------------------------------
BASE = dict(own=40 * MPH, ref=60 * MPH, v_ego=24.0, src="map", cand_dist=120.0, lead_v=49 * MPH,
            lead_cont_s=5.0, lead_why="ok", k_max=0.004, k_max_n=6, k_at=0.0035, k_at_n=6, k_at_gap=5.0,
            vis_k=0.004, vis_reach=220.0, a_lat=2.5, rain_ms=0.0)


def pace(**over):
  return icbm_lead_pace(**{**BASE, **over})


class TestTheGates:
  def test_a_tracked_lead_on_a_measured_curve_paces_icbm(self):
    t, why, k = pace()
    assert why == "ok" and t == pytest.approx(49 * MPH), "the lead is slower than the truck bound -> follow it"
    assert k == pytest.approx(0.004)

  def test_continuity_a_lead_tracked_under_3_s_does_not_pace(self):
    assert pace(lead_cont_s=2.9)[:2] == (None, "cont")
    assert pace(lead_cont_s=0.0, lead_why="jump")[:2] == (None, "jump")
    assert pace(lead_cont_s=3.0)[1] == "ok"

  def test_the_continuity_hold_is_three_seconds_not_merely_its_constant(self):
    """Absolute pin (the icbmrestorecap2pnw M5 lesson): a lead tracked for 2.5 s does not pace."""
    assert pace(lead_cont_s=2.5)[0] is None

  @pytest.mark.parametrize("tight_side", ["vis_k", "k_at", "k_max"])
  def test_the_TIGHTER_curvature_decides_never_the_looser(self, tight_side):
    """One source says R 1 km, the other R 100 m: the bound must come from R 100 m. With the looser reading
    the truck would be paced to 49 mph through a curve that allows 35."""
    loose = dict(vis_k=0.001, k_at=0.001, k_max=0.001)
    loose[tight_side] = 0.01
    t, why, k = pace(**loose)
    assert k == pytest.approx(0.01)
    assert t is None and why == "slower", f"{tight_side} was the tight reading and was ignored"

  def test_the_lateral_bound_caps_a_lead_faster_than_the_truck(self):
    """A car corners harder than the truck. 70 mph lead on k=0.004 (R 250 m): the truck gets sqrt(2.5*250)."""
    t, why, k = pace(lead_v=70 * MPH)
    assert why == "ok" and t == pytest.approx(math.sqrt(2.5 / 0.004))
    assert k * t * t == pytest.approx(2.5), "the paced speed must sit AT the bound, not above it"
    assert t < 56 * MPH, "absolute pin: 56 mph on R 250 m would be 2.5 m/s^2 -- anything faster is wrong"

  def test_the_bound_is_the_capability_and_zero_turns_it_off(self):
    assert pace(a_lat=0.0)[:2] == (None, "off")
    lo, _, _ = pace(lead_v=70 * MPH, a_lat=1.5)
    hi, _, _ = pace(lead_v=70 * MPH, a_lat=2.5)
    assert lo < hi

  def test_never_above_the_drivers_set(self):
    t, why, _ = pace(lead_v=80 * MPH, k_max=1e-5, k_at=1e-5, vis_k=1e-5, ref=60 * MPH)
    assert why == "ok" and t == pytest.approx(60 * MPH), "DEC-only: the pace may never exceed the driver's set"

  def test_a_curve_icbm_rates_below_25_mph_is_never_relaxed(self):
    assert pace(own=24 * MPH)[:2] == (None, "tight")
    assert pace(own=25.5 * MPH)[1] == "ok"

  @pytest.mark.parametrize("over,why", [
    (dict(k_max_n=0), "noGeom"),
    (dict(k_at_n=0), "gap"),
    (dict(k_at_gap=31.0), "gap"),
    (dict(k_at_gap=float("nan")), "gap"),
    (dict(vis_k=None), "noVis"),
    (dict(vis_k="x"), "noVis"),
    (dict(cand_dist=230.0), "visReach"),                   # beyond the model's reach
    (dict(cand_dist=200.0), "visReach"),                   # inside the 220 m reach, beyond 8 s at 24 m/s
    (dict(cand_dist=float("inf")), "badInput"),
  ])
  def test_an_unmeasurable_curve_means_no_relaxation(self, over, why):
    assert pace(**over)[:2] == (None, why)

  def test_a_vision_candidate_needs_no_map_point_match(self):
    assert pace(src="vis", k_at_n=0, k_at_gap=400.0)[1] == "ok"

  def test_rain_comes_off_the_truck_bound(self):
    dry, _, _ = pace(lead_v=70 * MPH)
    wet, _, _ = pace(lead_v=70 * MPH, rain_ms=3 * MPH)
    assert wet == pytest.approx(dry - 3 * MPH)

  def test_it_only_ever_raises(self):
    assert pace(lead_v=35 * MPH)[:2] == (None, "slower"), "a lead slower than ICBM's own target changes nothing"

  @pytest.mark.parametrize("bad", [None, "x", float("nan")])
  def test_garbage_refuses_and_never_raises(self, bad):
    for key in ("own", "ref", "v_ego", "lead_v", "cand_dist", "k_max", "k_max_n", "k_at", "k_at_gap", "vis_reach",
                "lead_cont_s", "a_lat"):
      t, why, _ = pace(**{key: bad})
      assert t is None, f"{key}={bad!r} relaxed the target"


class TestTheLeadClock:
  def test_it_accrues_while_one_lead_is_followed(self):
    trk = IcbmLeadTrack()
    for i in range(14):
      s, why = trk.update(i * 0.25, True, 50.0, 22.0, 22.0)
    assert why == "ok" and s == pytest.approx(3.25)

  def test_losing_the_lead_restarts_it(self):
    trk = IcbmLeadTrack()
    for i in range(20):
      trk.update(i * 0.25, True, 50.0, 22.0, 22.0)
    assert trk.update(5.0, False, 0.0, 0.0, 22.0) == (0.0, "noLead")
    s, _ = trk.update(5.25, True, 50.0, 22.0, 22.0)
    assert s == 0.0, "a re-acquired lead inherited the old clock"

  def test_a_one_tick_flicker_costs_the_full_three_seconds(self):
    trk = IcbmLeadTrack()
    for i in range(20):
      trk.update(i * 0.25, True, 50.0, 22.0, 22.0)
    trk.update(5.0, False, 0.0, 0.0, 22.0)
    for i in range(1, 12):
      s, _ = trk.update(5.0 + i * 0.25, True, 50.0, 22.0, 22.0)
    assert s < ICBM_LEAD_CONT_S

  def test_a_silent_brain_is_a_gap(self):
    trk = IcbmLeadTrack()
    for i in range(20):
      trk.update(i * 0.25, True, 50.0, 22.0, 22.0)
    assert trk.update(4.75 + 1.1, True, 50.0, 22.0, 22.0) == (0.0, "gap")

  def test_a_different_car_is_a_jump_but_a_closing_lead_is_not(self):
    trk = IcbmLeadTrack()
    for i in range(20):                                   # closing at 4 m/s: dRel falls 1 m per tick
      s, why = trk.update(i * 0.25, True, 60.0 - i * 1.0, 18.0, 22.0)
    assert why == "ok" and s > 4.0, "a lead explained by its own relative speed was treated as a new car"
    assert trk.update(5.0, True, 20.0, 18.0, 22.0) == (0.0, "jump"), "a 21 m cut-in kept the old clock"

  @pytest.mark.parametrize("bad", [float("nan"), None, "x", -5.0, 0.0])
  def test_garbage_is_no_lead(self, bad):
    trk = IcbmLeadTrack()
    trk.update(0.0, True, 50.0, 22.0, 22.0)
    assert trk.update(0.25, True, bad, 22.0, 22.0) == (0.0, "noLead")


class TestVisionCurvature:
  def test_it_divides_by_the_models_planned_speed_not_current_speed(self):
    """The model slows for the curve it sees. Yaw rate 0.5 rad/s at a planned 10 m/s is R 20 m -- dividing its
    lateral accel by the CURRENT 25 m/s would read R 125 m and pace the truck into it."""
    k, reach = icbm_vision_curvature([0.0, 0.1, 0.5], [25.0, 20.0, 10.0], [0.0, 40.0, 110.0])
    assert k == pytest.approx(0.05) and reach == 110.0

  @pytest.mark.parametrize("args", [([], [], []), ([0.1], [20.0], [30.0]), ([0.1, float("nan")], [20.0, 20.0], [0.0, 30.0]),
                                    ([0.1, 0.1], [20.0, 20.0], [0.0, 0.0]), (None, None, None)])
  def test_unusable_is_None_not_straight(self, args):
    assert icbm_vision_curvature(*args) == (None, 0.0)


class TestTheCapability:
  def test_lightning_gets_the_driver_bound_every_other_car_none(self, default_curve_cfg):
    assert _lightning().icbm_lead_lat_accel == 2.5
    assert PnwVehicle(FakeCP("TESLA_MODEL_S_HW3", "tesla", True)).icbm_lead_lat_accel == 0.0
    assert PnwVehicle(None).icbm_lead_lat_accel == 0.0

  def test_a_bad_config_cannot_exceed_the_drivers_own_p90(self, tmp_path, monkeypatch):
    cfg = tmp_path / "curve.json"
    cfg.write_text(json.dumps({"lightning": {"icbm_lead_lat_accel": 9.0}}))
    monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(cfg))
    assert _lightning().icbm_lead_lat_accel == 3.0


# ---------------------------------------------------------------------------------------------------------
# The real weekend
# ---------------------------------------------------------------------------------------------------------
def _pt(y, mo, d, h, mi, sec):
  import datetime
  import zoneinfo
  return datetime.datetime(y, mo, d, h, mi, sec, tzinfo=zoneinfo.ZoneInfo("America/Los_Angeles")).timestamp()


def _window(name):
  with open(FIXTURE) as f:
    fx = json.load(f)
  return [dict(zip(fx["_cols"], row, strict=True)) for row in fx[name]]


def _lead_clock(recs):
  """4 brain ticks per ~1 Hz record; dRel/vLead/vEgo interpolated between two lead-present records."""
  trk, out = IcbmLeadTrack(), []
  for i, r in enumerate(recs):
    nxt = recs[i + 1] if i + 1 < len(recs) else None
    span = (nxt["t"] - r["t"]) if nxt is not None and nxt["t"] - r["t"] <= 1.6 else 1.0
    both = nxt is not None and span <= 1.6 and r["lead"] and nxt["lead"]
    for q in range(4):
      f = q / 4.0
      if q == 0:
        s = trk.update(r["t"], bool(r["lead"]), r["dRel"] or 0.0, r["vLead"] or 0.0, r["vEgo"] or 0.0)
        out.append(s)
      elif both:
        lerp = [(r[k] or 0.0) + f * ((nxt[k] or 0.0) - (r[k] or 0.0)) for k in ("dRel", "vLead", "vEgo")]
        trk.update(r["t"] + f * span, True, *lerp)
      else:
        trk.update(r["t"] + f * span, False, 0.0, 0.0, r["vEgo"] or 0.0)
  return out


def _replay(name, a_lat=2.5):
  """icbm_lead_pace on every ICBM target tick of a window. Vision curvature is |visLat| / vEgo^2, which
  UNDER-reads a curve the model slows for -- so it raises the truck bound and these safety pins are
  pessimistic, not optimistic."""
  recs = _window(name)
  clock = _lead_clock(recs)
  rows = []
  for i, r in enumerate(recs):
    if r["icbmSrc"] not in ("map", "far", "vis") or r["icbmT"] is None or (r["vEgo"] or 0.0) < 1.0:
      continue
    v = r["vEgo"]
    ref = r["icbmC"] or max(r["vSet"] or 0.0, r["stockSet"] or 0.0)
    dist = {"map": r["mapDist"], "far": r["icbmKAtD"], "vis": (r["visTtc"] or 0.0) * v}[r["icbmSrc"]]
    t, why, _ = icbm_lead_pace(r["icbmT"], ref, v, r["icbmSrc"], dist, r["vLead"] or 0.0, *clock[i],
                               r["icbmK"], r["icbmKN"], r["icbmKAt"], r["icbmKAtN"], r["icbmKAtGap"],
                               abs(r["visLat"] or 0.0) / v ** 2, r["mdlEndX"], a_lat)
    rows.append((i, r, t, why))
  return recs, rows


def _truck_measured_k(recs, i, dist):
  s, k = 0.0, 0.0
  for j in range(i + 1, len(recs)):
    s += 0.5 * ((recs[j]["vEgo"] or 0.0) + (recs[j - 1]["vEgo"] or 0.0)) * (recs[j]["t"] - recs[j - 1]["t"])
    if (recs[j]["vEgo"] or 0.0) > 4.0 and recs[j]["achLat"] is not None:
      k = max(k, abs(recs[j]["achLat"]) / recs[j]["vEgo"] ** 2)
    if s > dist + 40.0:
      break
  return k


class TestTheWeekend:
  def test_sun_1221_trunk_the_lead_paces_icbm_within_the_truck_bound(self):
    """Stock ACC on, lead tracked 45-59 m ahead slowing 52 -> 41 mph through an R~180 m curve. ICBM tapped the
    set 60 -> 38 and the driver pressed the gas. The pace follows the lead, and at every paced speed the
    curvature the truck then MEASURED stays at or under 2.5 m/s^2."""
    recs, rows = _replay("sun_1221_trunk_lead")
    paced = [(i, r, t) for i, r, t, why in rows if t is not None]
    assert len(paced) >= 4, f"lead pacing did not engage on the one real lead-paced curve: {[w for *_, w in rows]}"
    for i, r, t in paced:
      assert t > r["icbmT"] + 1 * MPH
      assert t <= (r["vLead"] or 0.0) + 1e-6, "paced above the lead"
      k = _truck_measured_k(recs, i, r["mapDist"])
      assert k > 0.004, "the window lost the curve -- the safety pin below would prove nothing"
      assert k * t * t <= 2.5 + 0.05, f"paced to {t / MPH:.0f} mph = {k * t * t:.2f} m/s^2 measured"
    assert max(t for *_, t in paced) / MPH > 45.0, "absolute pin: the pace reached ~48 mph against ICBM's 38-40"

  def test_sun_1357_lead_lost_at_a_tight_ramp_is_refused_by_EACH_gate_alone(self, monkeypatch):
    """The case every gate exists for (stock ACC on): a lead tracked for 24+ s toward an R~36 m ramp that ICBM
    had correctly rated 18-24 mph; polyline read R 1-10 km, 56-125 m off mapd's point; the model saw a straight
    road at 165 m; the lead vanished at 13:57:09. Each of the three gates must refuse ON ITS OWN, and with all
    three removed the replay must reproduce the danger -- otherwise this test cannot fail."""
    gates = {"ICBM_LEAD_MIN_OWN_MS": 0.0, "ICBM_KAT_GAP_MAX_M": 1e9, "ICBM_VIS_TRUST_S": 1e9}
    ramp_t0 = _pt(2026, 9, 13, 13, 57, 0)                 # 13:56:52 is an earlier, gentle bend: pacing it is fine

    def ramp_rows():
      recs, rows = _replay("sun_1357_tight_ramp_lead_lost")
      return recs, [(i, r, t, why) for i, r, t, why in rows if r["t"] >= ramp_t0]

    _, rows = ramp_rows()
    assert len(rows) >= 8, "the window lost the ramp approach -- proves nothing"
    assert all(t is None for *_, t, _ in rows), "lead pacing relaxed the ramp ICBM was right about"
    for keep in gates:
      for name, off in gates.items():
        if name != keep:
          monkeypatch.setattr(m, name, off)
      _, rows = ramp_rows()
      assert all(t is None for *_, t, _ in rows), f"{keep} alone did not refuse 13:57"
      monkeypatch.undo()
    for name, off in gates.items():
      monkeypatch.setattr(m, name, off)
    recs, rows = ramp_rows()
    worst = max((_truck_measured_k(recs, i, r["mapDist"]) * t * t for i, r, t, _ in rows if t is not None), default=0.0)
    assert worst > 5.0, f"without the gates the replay should raise 13:57 into ~6 m/s^2, got {worst:.2f}"

  @pytest.mark.parametrize("name", ["sat_1259_primary", "sep08_2028"])
  def test_no_lead_no_change(self, name):
    """Sat 12:59 primary 61 -> 44 and the 2026-09-08 20:28 motorway phantom had no lead: A cannot touch them."""
    _, rows = _replay(name)
    assert rows, "window has no ICBM ticks -- proves nothing"
    assert all(t is None for *_, t, _ in rows)

  def test_sep08_1937_a_20_mph_ramp_target_is_never_relaxed(self):
    _, rows = _replay("sep08_1937")
    assert rows and all(t is None for *_, t, _ in rows)
    assert "tight" in {why for *_, why in rows}

  @pytest.mark.parametrize("name,when", [
    ("sun_1243_ramp", (2026, 9, 13, 12, 43, 54)),       # report section 3: "the geometry reads straight at a real curve"
    ("sun_1318_ramp", (2026, 9, 13, 13, 18, 37)),       # report section 3: "mapd contradicts its own geometry"
    ("sun_1355_ramp", (2026, 9, 13, 13, 55, 29)),       # report section 4: the second tracked-lead ramp
    ("sun_1355_ramp", (2026, 9, 13, 13, 55, 52)),       # a REAL tap-down, stock ACC on, set 64 -> 62
    ("sat_1247_behind", (2026, 9, 12, 12, 47, 25)),     # a REAL tap-down, stock ACC on, set 40 -> 28
    ("sep08_1937", (2026, 9, 8, 19, 37, 47)),           # icbmconsist2pnw's "correct mapd claim"
  ])
  def test_the_report_ramp_targets_were_for_points_BEHIND_the_truck(self, name, when):
    """B-behind's evidence, in the log itself: ICBM held a map target while mapd's claimed point got FARTHER
    away at about the truck's own speed for the three seconds before it, with mapd's velocity unchanged. That
    is a point the truck has already passed -- which is why neither lead pacing nor a geometry veto is the fix
    for these ramps."""
    t_end = _pt(*when) + 0.99                             # records carry sub-second stamps inside the named second
    recs = [r for r in _window(name) if t_end - 3.2 <= r["t"] <= t_end]
    assert len(recs) >= 3 and recs[-1]["icbmSrc"] == "map" and recs[-1]["icbmT"] is not None
    for p, r in zip(recs, recs[1:], strict=False):
      assert abs((r["mapV"] or 0.0) - (p["mapV"] or 0.0)) < 0.05, "mapd switched points -- not one claim"
      rate = (r["mapDist"] - p["mapDist"]) / (r["t"] - p["t"])
      assert rate > 0.5 * r["vEgo"], f"mapDist {p['mapDist']} -> {r['mapDist']} is not receding at v={r['vEgo']}"


class TestTheSanityRuleItself:
  """icbm_map_sanity's own gates, so a replay test cannot be the only thing standing behind them."""
  S = dict(own=20 * MPH, ref=60 * MPH, v_ego=20.0, src="map", cand_dist=100.0, k_at=0.003, k_at_n=5, k_at_gap=4.0,
           vis_k=0.004, vis_reach=200.0, a_lat=2.5)

  def sane(self, **over):
    return icbm_map_sanity(**{**self.S, **over})

  def test_much_gentler_on_both_readings_would_raise_to_the_geometry_speed(self):
    t, why = self.sane()
    assert why == "gentler" and t == pytest.approx(math.sqrt(2.5 / 0.004)), "the TIGHTER of the two readings sets it"
    assert self.sane(k_at=1e-5, vis_k=1e-5)[0] == pytest.approx(60 * MPH), "never above the driver's set"

  def test_consistent_geometry_leaves_it_alone(self):
    """Geometry allows 1.3x ICBM's speed -- inside the 1.5x 'much gentler' band: nothing to say."""
    k = 2.5 / (1.3 * 20 * MPH) ** 2
    assert self.sane(k_at=k, vis_k=k) == (None, "consistent")

  @pytest.mark.parametrize("over,why", [(dict(own=55 * MPH), "small"), (dict(k_at_gap=40.0), "gap"), (dict(k_at_n=0), "gap"),
                                        (dict(k_at=None), "gap"), (dict(vis_k=None), "noVis"), (dict(cand_dist=170.0), "visReach"),
                                        (dict(src="vis"), None), (dict(a_lat=0.0), "off")])
  def test_refusals(self, over, why):
    assert self.sane(**over) == (None, why)


class TestBIsTelemetryOnly:
  def _sane(self, name, hms_pred):
    out = []
    for r in _window(name):
      if r["icbmSrc"] in ("map", "far") and r["icbmT"] is not None and (r["vEgo"] or 0.0) > 1.0 and hms_pred(r):
        v = r["vEgo"]
        dist = r["mapDist"] if r["icbmSrc"] == "map" else r["icbmKAtD"]
        ref = r["icbmC"] or max(r["vSet"] or 0.0, r["stockSet"] or 0.0)
        out.append((r, *icbm_map_sanity(r["icbmT"], ref, v, r["icbmSrc"], dist, r["icbmKAt"], r["icbmKAtN"],
                                         r["icbmKAtGap"], abs(r["visLat"] or 0.0) / v ** 2, r["mdlEndX"], 2.5)))
    return out

  def test_1243_is_spared_only_by_distance_geometry_and_vision_both_read_gentle(self, monkeypatch):
    """12:43:54 was required not to be suppressed. The shipped rule refuses it -- but only because mapd's point
    sat 135-163 m out at 35-38 mph, beyond the 8 s vision-trust horizon. Point-matched geometry (R 4.8 km, gap 2 m)
    and vision BOTH read gentle; move the point inside the horizon and the rule suppresses it."""
    got = self._sane("sun_1243_ramp", lambda r: True)
    assert got and all(why == "visReach" for _, _, why in got), [w for *_, w in got]
    monkeypatch.setattr(m, "ICBM_VIS_TRUST_S", 1e9)
    got = self._sane("sun_1243_ramp", lambda r: True)
    assert any(why == "gentler" for _, _, why in got), f"12:43:54 no longer reads gentle: {[w for *_, w in got]}"

  def test_the_requested_rule_WOULD_raise_sat_1500_into_4_m_s2(self):
    """Sat 15:00:15-16: point-matched geometry and vision both under-read a real curve; the rule raises ICBM
    29 -> 50-56 mph where the truck measured ~4 m/s^2. This is why B is not wired."""
    recs = _window("sat_1500_underread")
    worst = 0.0
    for i, r in enumerate(recs):
      if r["icbmSrc"] != "map" or r["icbmT"] is None:
        continue
      v = r["vEgo"]
      ref = r["icbmC"] or max(r["vSet"] or 0.0, r["stockSet"] or 0.0)
      t, why = icbm_map_sanity(r["icbmT"], ref, v, "map", r["mapDist"], r["icbmKAt"], r["icbmKAtN"], r["icbmKAtGap"],
                               abs(r["visLat"] or 0.0) / v ** 2, r["mdlEndX"], 2.5)
      if t is not None:
        worst = max(worst, _truck_measured_k(recs, i, r["mapDist"]) * t * t)
    assert worst > 3.5

  def test_the_sep08_cases_have_no_point_matched_geometry_to_judge(self):
    for name in ("sep08_1937", "sep08_2028"):
      got = self._sane(name, lambda r: True)
      assert got and all(t is None for _, t, _ in got)
      assert not {why for *_, why in got} & {"gentler", "consistent"}, "judged without point-matched geometry"


class TestPathBehind:
  @staticmethod
  def _pts(xy):
    c = math.cos(math.radians(47.0))
    return [{"latitude": 47.0 + y / 111320.0, "longitude": -122.0 + x / (111320.0 * c), "velocity": 10.0} for x, y in xy]

  @staticmethod
  def _dist(p):
    return m._haversine_m(47.0, -122.0, p["latitude"], p["longitude"])

  def test_a_passed_point_is_behind_and_the_same_distance_ahead_is_not(self):
    pts = self._pts([(0, y) for y in range(-190, 211, 40)])          # no two points at the same distance
    behind = next(p for p in pts if p["latitude"] < 47.0 - 100 / 111320.0)
    ahead = next(p for p in pts if p["latitude"] > 47.0 + 100 / 111320.0)
    assert icbm_path_behind(pts, 47.0, -122.0, self._dist(behind)) is True
    assert icbm_path_behind(pts, 47.0, -122.0, self._dist(ahead)) is False

  def test_the_far_side_of_a_loop_ramp_is_AHEAD_though_its_bearing_points_back(self):
    r = 60.0
    loop = [(r * (1 - math.cos(a)), r * math.sin(a)) for a in [i * math.radians(30) for i in range(10)]]
    pts = self._pts([(0, -80), (0, -40)] + loop)                  # 270 deg loop starting at the truck
    far_side = pts[-2]                                             # ~240 deg round: south-east of the truck
    assert icbm_path_behind(pts, 47.0, -122.0, self._dist(far_side)) is False

  def test_ambiguity_resolves_toward_ahead(self):
    """A hairpin whose legs pass 6 m apart: the truck projects onto both. Taking the later leg would call the
    whole approach 'behind' and drop the real curve."""
    pts = self._pts([(0, -115), (0, -75), (0, -35), (0, 5), (0, 45), (3, 65), (6, 42), (6, 2), (6, -38), (6, -78)])
    approach_ahead = pts[4]                                        # 45 m out on the approach; no other point at 45 m
    assert sum(abs(self._dist(p) - 45.0) <= 0.5 for p in pts) == 1, "the geometry lost its unique candidate"
    assert icbm_path_behind(pts, 47.0, -122.0, self._dist(approach_ahead)) is False

  def test_two_points_at_the_same_distance_one_ahead_is_not_behind(self):
    """Distances are unsigned: a passed point and a point ahead can match the same candidate distance. That must
    resolve toward AHEAD -- calling it behind would drop a real curve if this is ever wired."""
    pts = self._pts([(0, y) for y in range(-200, 201, 40)])
    assert icbm_path_behind(pts, 47.0, -122.0, 200.0) is False

  def test_off_the_path_or_unmatched_is_unknown(self):
    pts = self._pts([(200, y) for y in range(-200, 201, 40)])
    assert icbm_path_behind(pts, 47.0, -122.0, 250.0) is None
    near = self._pts([(0, y) for y in range(-200, 201, 40)])
    assert icbm_path_behind(near, 47.0, -122.0, 123.4) is None

  @pytest.mark.parametrize("pts", [None, [], [{"latitude": "x"}], [{"latitude": float("nan"), "longitude": 0.0}] * 3])
  def test_garbage_is_unknown(self, pts):
    assert icbm_path_behind(pts, 47.0, -122.0, 40.0) is None


# ---------------------------------------------------------------------------------------------------------
# Through the controller, and through the Ford executor
# ---------------------------------------------------------------------------------------------------------
LAT0, LON0 = 47.0, -122.0


def _ll(x, y):
  return LAT0 + y / 111320.0, LON0 + x / (111320.0 * math.cos(math.radians(LAT0)))


def _scene(curve_at, radius, map_v, step=40.0):
  """mapd-style path in travel order: straight from 200 m behind to the curve, then a right-hand 90 deg arc."""
  pts, y = [], -200.0
  while y < curve_at - 25.0:
    la, lo = _ll(0.0, y)
    pts.append({"latitude": la, "longitude": lo, "velocity": 0.0})
    y += step
  for i in range(int(radius * math.pi / 2 / step) + 1):
    th = i * step / radius
    la, lo = _ll(radius * (1 - math.cos(th)), curve_at + radius * math.sin(th))
    pts.append({"latitude": la, "longitude": lo, "velocity": map_v})
  return pts


def _model(v, curve_at, radius, n=33, horizon_s=10.0):
  ts = [horizon_s * (i / (n - 1)) ** 2 for i in range(n)]
  px = [v * t for t in ts]
  orz = [(-v / radius) if curve_at <= x <= curve_at + radius * math.pi / 2 else 0.0 for x in px]
  return orz, [v] * n, px, ts


class _Mem:
  def __init__(self):
    self.log = []

  def put_nonblocking(self, k, v):
    if k == "IcbmTarget":
      self.log.append(v)


def _controller(monkeypatch, clock, veh):
  monkeypatch.setattr(m.time, "monotonic", lambda: clock[0])
  monkeypatch.setattr(m.time, "time", lambda: clock[0])
  cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_icbm_step"))

  class Stub:
    pass
  c = Stub()
  c.mem_params = _Mem()
  c._veh = veh
  c._icbm_ep = IcbmEpisode()
  for k, v in dict(_icbm_ceiling=None, _icbm_dir=None, _map_targets=[], _cur_lat=LAT0, _cur_lon=LON0,
                   _cur_bearing=0.0, _icbm_floor_lim=0.0, _icbm_floor_pend=None, _icbm_floor_hit=False,
                   _icbm_k=0.0, _icbm_k_n=0, _icbm_k_ahead=True, _icbm_k_at=0.0, _icbm_k_at_d=0.0,
                   _icbm_k_at_n=0, _icbm_k_at_gap=0.0, _stock_set=60 * MPH, _stock_on=True,
                   _icbm_last_pub=-1e9).items():
    setattr(c, k, v)
  return c, cls._icbm_step.__get__(c)


def _tick(c, step, curve_at, v, v_set, lead, radius=180.0, map_v=20.0):
  """One brain tick with the truck at the origin heading north and the curve `curve_at` m ahead."""
  pts = _scene(curve_at, radius, map_v)
  c._map_targets = pts
  k, _, _, kn, _ = polyline_curvature(pts, LAT0, LON0, 500.0, 2.5, 0.0)
  c._icbm_k, c._icbm_k_n = k, kn
  mtv, mtd = upcoming_curve(pts, LAT0, LON0, v, 10.0)
  orz, vx, px, ts = _model(v, curve_at, radius)
  vis_acc, ttc = vision_curve_lat_accel(orz, vx, ts, v)
  vis_k, reach = icbm_vision_curvature(orz, vx, px)
  sig = {"v_ego": v, "v_set": v_set, "map_target_v": mtv, "map_target_dist": mtd, "curve_lat_accel_vision": vis_acc,
         "time_to_curve": ttc, "lat_accel_now": 0.0, "vis_k_max": vis_k, "vis_reach": reach,
         "has_lead": lead is not None, "lead_drel": lead[0] if lead else 0.0, "lead_vlead": lead[1] if lead else 0.0,
         "gas": False, "brake": False, "spd_lim": 0.0, "pitch": None}
  n = len(c.mem_params.log)
  step(sig, active=True)
  new = c.mem_params.log[n:]
  return new[-1] if new else None


class TestThroughTheController:
  def test_a_tracked_lead_raises_the_published_cap_and_losing_it_reverts(self, monkeypatch, default_curve_cfg):
    clock = [1000.0]
    c, step = _controller(monkeypatch, clock, _lightning())
    lead = (50.0, 22.0)                                            # 49 mph, 50 m ahead
    for _ in range(20):                                            # 5 s of approach with the lead
      pub = _tick(c, step, 140.0, 24.0, 60 * MPH, lead)
      clock[0] += 0.25
    assert pub and pub.get("target") is not None, "no curve cap published -- proves nothing"
    assert c._icbm_lead_why == "ok", f"lead pacing did not engage: {c._icbm_lead_why}"
    own, paced = c._icbm_own_t, pub["target"]
    assert paced == pytest.approx(c._icbm_lead_t, abs=0.01) and paced > own + 1 * MPH
    assert paced == pytest.approx(math.sqrt(2.5 * 180.0), abs=0.05), "pace must be the R 180 m truck bound (lead faster)"
    # the lead vanishes: the brain drops pacing THIS tick ...
    pub = _tick(c, step, 134.0, 24.0, 60 * MPH, None)
    assert c._icbm_lead_t is None and c._icbm_lead_why == "noLead", "pacing survived the lead"
    assert c._icbm_own_t == pytest.approx(own, abs=0.3)
    # ... and the published cap follows within the ratchet's confirm window
    reverted_at = None
    for i in range(1, 8):
      clock[0] += 0.25
      pub = _tick(c, step, 134.0 - 6.0 * i, 24.0, 60 * MPH, None)
      if pub and pub["target"] <= c._icbm_own_t + 0.05 and reverted_at is None:
        reverted_at = i * 0.25
    assert reverted_at is not None and reverted_at <= ICBM_RATCHET_CONFIRM_S + 0.26, \
      f"published cap did not revert to ICBM's own target in time ({reverted_at})"

  def test_chill_clears_the_telemetry_and_restarts_the_lead_clock(self, monkeypatch, default_curve_cfg):
    clock = [1500.0]
    c, step = _controller(monkeypatch, clock, _lightning())
    for _ in range(20):
      _tick(c, step, 140.0, 24.0, 60 * MPH, (50.0, 22.0))
      clock[0] += 0.25
    assert c._icbm_lead_why == "ok"
    step({"v_ego": 24.0}, active=False)
    clock[0] += 0.25
    assert all(v is None for v in m._curvelead_tele(c).values()), "forced Chill left lead-pace telemetry standing"
    _tick(c, step, 140.0, 24.0, 60 * MPH, (50.0, 22.0))
    assert c._icbm_lead_why == "cont", "the lead clock survived a Chill interlude"

  def test_a_crash_in_the_new_code_falls_back_to_icbms_own_target_and_says_so(self, monkeypatch, default_curve_cfg):
    """Raise AFTER lead pacing has already raised the target (in the B telemetry): the published cap must be
    ICBM's own, ICBM must keep publishing, and the failure must be logged -- once, not at 4 Hz."""
    logged = []
    monkeypatch.setattr(m.cloudlog, "exception", lambda msg, *a, **k: logged.append(msg))
    clock = [1700.0]
    c, step = _controller(monkeypatch, clock, _lightning())
    for _ in range(20):
      _tick(c, step, 140.0, 24.0, 60 * MPH, (50.0, 22.0))
      clock[0] += 0.25
    assert c._icbm_lead_why == "ok"
    own = c._icbm_own_t
    monkeypatch.setattr(m, "icbm_map_sanity", lambda *a, **k: (_ for _ in ()).throw(RuntimeError("boom")))
    pubs = []
    for _ in range(8):
      pubs.append(_tick(c, step, 140.0, 24.0, 60 * MPH, (50.0, 22.0)))
      clock[0] += 0.25
    assert all(p and p.get("target") is not None for p in pubs), "a curvelead crash stopped ICBM publishing"
    assert pubs[-1]["target"] == pytest.approx(own, abs=0.05), "a curvelead crash left the lead-paced target standing"
    assert c._icbm_lead_why == "error"
    assert sum("lead pacing FAILED" in msg for msg in logged) == 1

  def test_B_verdicts_never_reach_the_published_target(self, monkeypatch, default_curve_cfg):
    """Force both B verdicts to fire, with no lead: the published cap must equal the one without B."""
    def run(force_b):
      clock = [2000.0]
      c, step = _controller(monkeypatch, clock, _lightning())
      if force_b:
        monkeypatch.setattr(m, "icbm_map_sanity", lambda *a, **k: (59 * MPH, "gentler"))
        monkeypatch.setattr(m, "icbm_path_behind", lambda *a, **k: True)
      out = []
      for i in range(12):
        out.append(_tick(c, step, 200.0 - 5.0 * i, 24.0, 60 * MPH, None))
        clock[0] += 0.25
      return out, c
    base, _ = run(False)
    forced, c = run(True)
    assert any(p and p.get("target") for p in base), "no cap published -- proves nothing"
    assert c._icbm_sane_t == pytest.approx(59 * MPH) and c._icbm_behind is True, "the forced verdicts never ran"
    assert forced == base

  def test_B_logs_when_it_would_have_acted(self, monkeypatch, default_curve_cfg):
    logged = []
    monkeypatch.setattr(m.cloudlog, "info", lambda msg, *a, **k: logged.append(msg % a if a else msg))
    monkeypatch.setattr(m, "icbm_path_behind", lambda *a, **k: True)
    clock = [3000.0]
    c, step = _controller(monkeypatch, clock, _lightning())
    for _ in range(8):
      _tick(c, step, 160.0, 24.0, 60 * MPH, None)
      clock[0] += 0.25
    assert sum("BEHIND the truck" in s for s in logged) == 1, "the behind verdict edge was not logged exactly once"

  def test_engage_and_end_are_logged(self, monkeypatch, default_curve_cfg):
    logged = []
    monkeypatch.setattr(m.cloudlog, "info", lambda msg, *a, **k: logged.append(msg % a if a else msg))
    clock = [4000.0]
    c, step = _controller(monkeypatch, clock, _lightning())
    for _ in range(20):
      _tick(c, step, 140.0, 24.0, 60 * MPH, (50.0, 22.0))
      clock[0] += 0.25
    _tick(c, step, 134.0, 24.0, 60 * MPH, None)
    assert sum("lead pacing ENGAGED" in s for s in logged) == 1
    assert sum("lead pacing ENDED (noLead)" in s for s in logged) == 1

  def test_telemetry_reaches_the_record(self, monkeypatch, default_curve_cfg):
    clock = [5000.0]
    c, step = _controller(monkeypatch, clock, _lightning())
    for _ in range(20):
      _tick(c, step, 140.0, 24.0, 60 * MPH, (50.0, 22.0))
      clock[0] += 0.25
    cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_event_record"))

    class Rec:
      def __getattr__(self, n):
        return getattr(c, n) if n in vars(c) else None
    r = Rec()
    for k, v in dict(_vtsc_tele={}, _sa_tele={}, _speed_limit=0.0, _button=0, _ces2_urg=0.0, _icbm_k_dist=0.0,
                     _icbm_k_v=0.0, _ces2_div=types.SimpleNamespace(count=0), _gl=types.SimpleNamespace(state=None)).items():
      object.__setattr__(r, k, v)
    rec = cls._event_record.__get__(r)("tick", {"vEgo": 24.0})
    assert set(CURVELEAD_TELE_KEYS) <= set(rec), "a curvelead key never reached the ces_events record"
    assert rec["icbmLeadWhy"] == "ok" and rec["icbmLeadT"] == pytest.approx(c._icbm_lead_t, abs=0.01)
    assert rec["icbmOwnT"] == pytest.approx(c._icbm_own_t, abs=0.01) and rec["icbmLeadS"] >= 3.0
    assert rec["icbmKVis"] == pytest.approx(1 / 180.0, abs=1e-4)


class TestClosedLoopThroughTheFordExecutor:
  """Brain -> IcbmTarget -> icbm_pnw.arbitrate -> decide_press -> PressGovernor at 100 Hz -> the truck's set
  steps 1 mph per completed tap. Modelled on Sun 12:21: 60 mph set, a curve ICBM rates ~40 mph, a lead at
  49 mph 50 m ahead, the curve 300 m out at 24 m/s."""

  APPEAR_S = 3.5          # the lead is tracked this long before the curve enters mapd's path

  def _drive(self, monkeypatch, veh, lose_lead_at=None, seconds=11.0):
    clock = [6000.0]
    c, step = _controller(monkeypatch, clock, veh)
    stock, gov, frame, pressing = 60 * MPH, PressGovernor(), 0, False
    trace = []
    t = 0.0
    while t < seconds:
      # 700 m out (beyond mapd's 500 m path: no candidate) while the lead clock accrues, then the curve
      # appears 185 m out -- inside the 8 s vision-trust horizon at 24 m/s -- and approaches.
      curve_at = 700.0 if t < self.APPEAR_S else 185.0 - 24.0 * (t - self.APPEAR_S)
      lead = None if (lose_lead_at is not None and t >= lose_lead_at) else (50.0, 22.0)
      c._stock_set = stock
      # icbmslow2pnw: map_v 18.0 (was the 20.0 default). The map-rating floor means ICBM's own
      # target is now the candidate's OWN rating rather than rating-minus-penalty, so the raw
      # rating that makes this scenario "a curve ICBM rates ~40 mph" is 18.0 m/s, not 20.0.
      # The scenario (and the gap to the 47.4 mph lead pace these tests exist to show) is
      # preserved; only the input that produces it moved.
      pub = _tick(c, step, max(curve_at, 70.0), 24.0, stock, lead, map_v=18.0)
      cmd = None
      if pub and pub.get("target") is not None:
        cmd = IcbmCommand(target_ms=pub["target"], ceiling_ms=pub["ceiling"], ts=pub["ts"], dir=pub.get("dir", "dec"))
      for _ in range(25):                                        # 0.25 s of executor frames
        chosen = arbitrate([cmd], clock[0])
        btn = gov.update(frame, decide_press(stock, chosen, clock[0], True, False))
        if pressing and btn is None:
          stock -= STEP_MS                                       # a completed SET- tap
        pressing = btn == "dec"
        assert btn != "inc", "a curve cap pressed SET+"
        frame += 1
        clock[0] += 0.01
      trace.append((t, stock, pub["target"] if pub and pub.get("target") is not None else None,
                    c._icbm_own_t, c._icbm_lead_t))
      t += 0.25
    return trace

  def test_with_the_lead_the_set_is_held_at_the_pace_not_tapped_to_icbms_own_target(self, monkeypatch, default_curve_cfg):
    paced = self._drive(monkeypatch, _lightning())
    off_veh = _lightning()
    monkeypatch.setattr(type(off_veh), "icbm_lead_lat_accel", property(lambda s: 0.0))
    own = self._drive(monkeypatch, off_veh)
    own_floor = min(s for _, s, *_ in own)
    paced_floor = min(s for _, s, *_ in paced)
    pace_v = math.sqrt(2.5 * 180.0)
    engaged = [t for t, _, _, _, lead_t in paced if lead_t is not None]
    assert engaged and engaged[0] <= self.APPEAR_S + 0.3, f"lead pacing never engaged from the first curve tick: {engaged[:3]}"
    assert own_floor < 42 * MPH, f"without pacing ICBM should walk the set to ~40 mph (got {own_floor / MPH:.1f}) -- proves nothing"
    assert paced_floor >= pace_v - STEP_MS - 1e-6, f"set tapped to {paced_floor / MPH:.1f} mph below the {pace_v / MPH:.1f} pace"
    assert paced_floor <= pace_v + 1e-6 + STEP_MS, "the set was never brought down to the pace at all"

  def test_losing_the_lead_walks_the_set_down_to_icbms_own_target(self, monkeypatch, default_curve_cfg):
    loss = 8.75                                           # curve 70 m out; the set has settled at the pace
    trace = self._drive(monkeypatch, _lightning(), lose_lead_at=loss, seconds=loss + 7.0)
    pace_v = math.sqrt(2.5 * 180.0)
    before = [(s, lead_t) for t, s, _, _, lead_t in trace if self.APPEAR_S + 0.5 <= t < loss]
    after = [(t, s, own) for t, s, _, own, _ in trace if t >= loss]
    assert before and all(lead_t is not None for _, lead_t in before), "pacing was not engaged before the loss -- proves nothing"
    held = before[-1][0]
    assert abs(held - pace_v) <= STEP_MS, f"set {held / MPH:.1f} mph was not settled at the {pace_v / MPH:.1f} pace"
    own = min(o for *_, o in after if o is not None)
    assert own < held - 5 * MPH, "ICBM's own target is too close to the pace to show a revert"
    first_tap = next((t for t, s, _ in after if s < held - 1e-6), None)
    assert first_tap is not None and first_tap - loss <= ICBM_RATCHET_CONFIRM_S + 0.75, \
      f"no tap within {ICBM_RATCHET_CONFIRM_S + 0.75:.2f} s of losing the lead (first at {first_tap})"
    assert after[-1][1] <= own + STEP_MS, f"set stuck at {after[-1][1] / MPH:.1f} mph above ICBM's own {own / MPH:.1f}"


# ---------------------------------------------------------------------------------------------------------
# The Tesla
# ---------------------------------------------------------------------------------------------------------
class TestTheTeslaIsUntouched:
  """End to end through the REAL CESController.experimental_request: the Tesla (op-long, no ICBM) never computes
  the vision curvature, never runs the ICBM brain, and its decision + overlay feed are identical when every
  curvelead control helper is replaced by one that raises. The Lightning run is the control that makes the
  spy able to fail."""

  @staticmethod
  def _run(monkeypatch, fp, brand, op_long, poison):
    NS = types.SimpleNamespace

    class P:
      def get(self, k, return_default=False):
        return {"CESMode": "2", "CESButtonState": "0"}.get(k)

      def get_bool(self, k):
        return False

    class Mem:
      def __init__(self):
        self.puts = []

      def get(self, k, return_default=False):
        return None

      def put_nonblocking(self, k, v):
        self.puts.append((k, {kk: vv for kk, vv in v.items() if kk != "ts"} if isinstance(v, dict) else v))

    calls = {"vis": 0, "pace": 0}
    real_vis, real_pace = m.icbm_vision_curvature, m.icbm_lead_pace
    if poison:
      def boom(*a, **k):
        raise AssertionError("curvelead helper called")
      monkeypatch.setattr(m, "icbm_vision_curvature", boom)
      monkeypatch.setattr(m, "icbm_lead_pace", boom)
    else:
      monkeypatch.setattr(m, "icbm_vision_curvature", lambda *a: (calls.__setitem__("vis", calls["vis"] + 1), real_vis(*a))[1])
      monkeypatch.setattr(m, "icbm_lead_pace", lambda *a, **k: (calls.__setitem__("pace", calls["pace"] + 1), real_pace(*a, **k))[1])
    clock = [7000.0]
    monkeypatch.setattr(m.time, "monotonic", lambda: clock[0])
    c = m.CESController(FakeCP(fp, brand, op_long), params=P())
    c.mem_params = Mem()
    c._event_log_ok = False
    n = 33
    model = NS(orientationRate=NS(z=[-0.12] * n, t=[i * 0.3 for i in range(n)]), velocity=NS(x=[22.0] * n),
               position=NS(x=[22.0 * i * 0.3 for i in range(n)]), action=NS(shouldStop=False), meta=NS(laneChangeState="off"))
    sm = {"radarState": NS(leadOne=NS(status=True, vLead=21.0, dRel=45.0)), "modelV2": model,
          "carControl": NS(orientationNED=[0.0, 0.0, 0.0])}
    cs = NS(vEgo=22.0, aEgo=0.0, gasPressed=False, brakePressed=False, leftBlinker=False, rightBlinker=False, vCruise=96.0,
            standstill=False, steeringAngleDeg=0.0, steeringPressed=False, leftBlindspot=False, rightBlindspot=False,
            cruiseState=NS(speed=26.8, enabled=True))
    decisions = []
    for _ in range(30):
      decisions.append(c.experimental_request(cs, sm))
      clock[0] += 0.3
    monkeypatch.undo()
    return decisions, c.mem_params.puts, calls

  def test_tesla_decisions_and_overlay_are_identical_with_every_helper_poisoned(self, monkeypatch, default_curve_cfg):
    d_real, puts_real, calls = self._run(monkeypatch, "TESLA_MODEL_S_HW3", "tesla", True, poison=False)
    d_poison, puts_poison, _ = self._run(monkeypatch, "TESLA_MODEL_S_HW3", "tesla", True, poison=True)
    assert calls == {"vis": 0, "pace": 0}, f"the Tesla ran curvelead code: {calls}"
    assert not any(k == "IcbmTarget" for k, _ in puts_real), "the Tesla published an ICBM target"
    assert d_real == d_poison and puts_real == puts_poison
    assert not any(k.startswith("icbm") for k, v in puts_real if isinstance(v, dict) for k in v), \
      "curvelead telemetry leaked into the Tesla overlay feed"

  def test_the_lightning_control_run_does_call_it(self, monkeypatch, default_curve_cfg):
    _, puts, calls = self._run(monkeypatch, LIGHTNING, "ford", False, poison=False)
    assert calls["vis"] > 0 and calls["pace"] > 0, f"the spy cannot see the Lightning either -- test is blind: {calls}"
    assert any(k == "IcbmTarget" for k, _ in puts)
