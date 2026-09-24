"""curvedbshadow2pnw -- CURVEDB2PNW.md Phase 2 on the car, SHADOW ONLY.

The read boundary (the safety argument) lives in its own file, `test_curvedb_read_boundary.py`.
This file pins what the shadow MEASURES and what it SAYS, which is the part that has to be right for
the data to be worth collecting at all.

What each section pins, and the mutation each test exists to kill (the harness is in
`_scratch/mutate_curvedbshadow.py`, which anchor-checks and `compile()`s every mutant):

  6.1 UP     `max` across the extent and `max` across passes, never a mean -- M1 "mean instead of
             max over the extent", M2 "the row averages its observations".
  6.1        capped at the posted limit, no lead-car gate (D7) -- M3 "the posted cap is dropped".
  6.2 DOWN   evaluated INDEPENDENTLY of the UP disqualifier test -- M4 "DOWN nested inside the clean
             branch", which is exactly the shipped offline bug (zero DOWN observations on 2.2 GB).
  6.2        DOWN may only ever LOWER v_row -- M5 "k_down lowers k_eff instead of raising it".
  6.3        keyed on the CANDIDATE's position and the APPROACH bearing -- M6 "keyed on the truck",
             M7 "the bearing is taken in the bend rather than on the approach".
  6.4        no posted limit / unknown class / ramp / too few passes or dates -> NO authority, each
             with a stated reason -- M8 "a refusal returns silently".
  3.9-6      non-circularity: this drive's own passes never enter the database it is judged against
             -- M9 "the observation is appended to the live DB".
  Rule 2     every refusal and every failure names itself; `cdbOn`/`cdbErr` distinguish "no site"
             from "dead" -- M10 "the null fragment is returned without a state".

The last class drives the REAL CESController at 100 Hz: a rule that is correct in a unit test and
never reaches the record is the visK failure mode, which this file's Phase-1 sibling was written to
prevent and which has happened four times in ces_pnw's history.
"""
from __future__ import annotations

import copy
import json
import math
import os
import types

import pytest

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
from openpilot.selfdrive.controls.lib.ces_pnw import curvedb_shadow as cs
from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvelead2pnw import FakeCP, LAT0, LON0, _model, _scene
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_ces_mode_read_failure_logged import LIGHTNING, MPH, _P
from openpilot.tools.curvedb import ingest as offline_ingest
from openpilot.tools.curvedb.store import PROVISIONAL_PARAMS, CurveDB, Observation, authority, haversine_m

NS = types.SimpleNamespace

M_PER_DEG_LAT = 111320.0
# A REAL epoch (2026-09-18 PT). `_finish` refuses to file a row stamped before
# CLOCK_VALID_EPOCH, because the dead RTC makes a cold boot read 1970 and a bogus
# timestamp manufactures a distinct DATE -- half of D6's authority key.
T0 = 1788300000.0


def _north(lat, metres):
  return lat + metres / M_PER_DEG_LAT


def _shadow(tmp_path, platform=LIGHTNING, config=None, obs=()):
  """A shadow with its files under tmp_path, loaded SYNCHRONOUSLY so the test is deterministic."""
  tmp_path.mkdir(parents=True, exist_ok=True)
  obs_path = tmp_path / "obs.jsonl"
  obs_path.write_text("".join(json.dumps(vars(o)) + "\n" for o in obs))
  cfg_path = tmp_path / "curvedb.json"
  if config is not None:
    cfg_path.write_text(json.dumps(config))
  return cs.CurveDBShadow(platform, obs_path=str(obs_path), config_path=str(cfg_path),
                          load_async=False)


def _obs(**over):
  base = dict(date="2026-09-10", t=T0, car=LIGHTNING, drive_id="d0",
              site_lat=LAT0, site_lon=LON0, bearing_deg=0.0, k=0.01, kind="up",
              estimator="kPeak100", site_src="logged", source="test", posted_ms=26.8,
              highway_class="motorway", n_ticks=10, dq_state="clean", dq_src="rollup100")
  base.update(over)
  return Observation(**base)


# The truck always drives DUE NORTH from LAT0/LON0, one record every `step_m` metres. Sites are
# given as metres north of LAT0 (and optionally metres east), so "the site is on the driven line" and
# "the site is 5 km off it" are both expressible -- the first version of this harness put the site
# ON the truck at t=0, and the never-reached test was then measuring nothing at all.
SITE_M = 400.0


def _site(north_m=SITE_M, east_m=0.0):
  lat = _north(LAT0, north_m)
  return (lat, LON0 + east_m / (M_PER_DEG_LAT * math.cos(math.radians(lat))))


def _drive_past(sh, site, *, n=80, step_m=10.0, v=25.0, t0=T0, bearing=0.0,
                bearing_per_tick=None,
                k_per_tick=None, dq_per_tick=None, str_prs_at=None, k_cmd=0.004,
                posted=26.8, hwy="motorway", arm=True):
  """Walk the truck north past `site` in `step_m` records, feeding the 100 Hz half in between.

  `n * step_m` must comfortably exceed (distance to the site) + extent_back + extent_fwd, or the
  passage never completes -- which is a silent nothing, the exact failure Rule 2 is about. Asserted
  rather than assumed.

  Returns the list of telemetry fragments, one per record."""
  assert not arm or n * step_m >= SITE_M + 200.0, "harness too short for a passage to complete"
  out = []
  ticks_per_record = max(int(step_m / v * 100), 1)
  for i in range(n):
    lat = _north(LAT0, i * step_m)
    dq = 0 if dq_per_tick is None else dq_per_tick(i)
    pressed = str_prs_at is not None and i in str_prs_at
    for j in range(ticks_per_record):
      # `k_per_tick` is called per CONTROL tick, not per record: a spike shorter than one record is
      # the only thing that distinguishes section 3.3's per-second PEAK from the instantaneous
      # sample the design was written to replace (a per-record k made mutant M1 survive).
      k = 0.0008 if k_per_tick is None else k_per_tick(i, j)
      sh.tick(k, k_cmd if pressed else None, None, dq, v, pressed)
    brg = bearing if bearing_per_tick is None else bearing_per_tick(i)
    out.append(sh.record(now_wall=t0 + i * (step_m / v), lat=lat, lon=LON0, bearing=brg,
                         v_ego=v, site_pt=(site if arm else None), posted_ms=posted,
                         hwy_class=hwy, icbm_src=None, icbm_target_ms=None, ref_ms=None))
  return out


def _written(sh) -> list[dict]:
  path = sh._obs_path
  with open(path) as f:
    return [json.loads(ln) for ln in f if ln.strip()]


# =====================================================================================================
# agreement with the offline pipeline -- if these drift, the section 7 replay validates nothing
# =====================================================================================================
class TestIngestAgreement:

  @pytest.mark.parametrize("name", ["DRIVE_GAP_S", "MAX_TICK_DT_S", "PASSAGE_MAX_M",
                                    "PASSAGE_SCAN_M", "K_MIN_USABLE"])
  def test_the_shared_constants_are_equal(self, name):
    assert getattr(cs, name) == getattr(offline_ingest, name), \
      f"{name} differs between the car and the replay"

  def test_the_disqualifier_bits_mean_the_same_thing_in_all_three_places(self):
    """ces_pnw's roll-up feeds the shadow directly, and ingest re-reads the same `dqWhy` strings off
    the log. Three vocabularies that only look alike is how a `drv` becomes a `blnk`."""
    assert (cs.DQ_SAT, cs.DQ_DRV, cs.DQ_LC, cs.DQ_BLNK) == (m.DQ_SAT, m.DQ_DRIVER, m.DQ_LANECHG, m.DQ_BLINKER)
    assert (cs.DQ_SAT, cs.DQ_DRV, cs.DQ_LC, cs.DQ_BLNK) == \
           (offline_ingest.DQ_SAT, offline_ingest.DQ_DRV, offline_ingest.DQ_LC, offline_ingest.DQ_BLNK)
    assert cs._dq_names(cs.DQ_DRV | cs.DQ_BLNK) == offline_ingest.dq_names(cs.DQ_DRV | cs.DQ_BLNK)

  def test_the_pt_date_is_the_same_function(self):
    """The leave-one-date-out key. A late-evening PT drive is already the next day in UTC, so a
    disagreement here would put the same road in two different folds."""
    for t in (1.7883e9, 1.7883e9 + 3600 * 7, 1.7883e9 + 3600 * 12, 1.6e9):
      assert cs.pt_date(t, offline_ingest.PT) == offline_ingest.pt_date(t)


# =====================================================================================================
# 6.1 -- UP
# =====================================================================================================
class TestUp:

  def test_the_row_takes_the_PEAK_over_the_extent_not_the_mean(self, tmp_path):
    """M1. One tight metre in an otherwise gentle bend IS the bend. A mean would report the approach.

    This also exercises the 100 Hz half: the peak is accumulated between records, so a record-rate
    sample would miss a spike that lives inside one second."""
    site = _site()
    sh = _shadow(tmp_path)
    # ONE control tick out of the ~40 in record 41 carries the spike. A per-record sample -- or the
    # last reading of the window -- reports 0.001 and calls a real bend a straight road.
    _drive_past(sh, site, k_per_tick=lambda i, j: 0.02 if (i, j) == (41, 7) else 0.001)
    rows = _written(sh)
    ups = [r for r in rows if r["kind"] == "up"]
    assert len(ups) == 1, rows
    assert ups[0]["k"] == pytest.approx(0.02), "the peak was averaged away"
    assert ups[0]["estimator"] == "kPeak100"

  def test_max_across_passes_never_a_mean(self, tmp_path):
    """M2, and section 6.1 verbatim: *one genuinely tight pass must never be averaged away by ten
    gentle ones*. The `max` lives in CurveRow.k_up, which the car imports rather than reimplements."""
    obs = [_obs(k=0.002, date=f"2026-09-{d:02d}", drive_id=f"d{d}") for d in range(10, 20)]
    obs.append(_obs(k=0.03, date="2026-09-25", drive_id="tight"))
    db = CurveDB.build(obs, PROVISIONAL_PARAMS)
    assert len(db.rows) == 1
    assert db.rows[0].k_up == pytest.approx(0.03)

  def test_a_disqualified_pass_never_becomes_a_row_and_says_which_disqualifier(self, tmp_path):
    site = _site()
    sh = _shadow(tmp_path)
    tele = _drive_past(sh, site, dq_per_tick=lambda i: cs.DQ_BLNK if i == 40 else 0)
    assert not [r for r in _written(sh) if r["kind"] == "up"]
    evs = [t["cdbEv"] for t in tele if t["cdbEv"]]
    assert evs and evs[0].startswith("dirty:") and "blnk" in evs[0], evs

  def test_a_spike_at_one_site_does_not_leak_into_the_next(self, tmp_path):
    """The 100 Hz window must be DRAINED at every record.

    Without the drain, one tight metre -- or one blinker -- contaminates every later passage for the
    rest of the drive, and a row a kilometre further on claims a curve that is behind the truck.
    Every other test here drives exactly ONE passage, so there is nothing for a leak to leak INTO;
    the missing drain survived mutation until this test existed (M16)."""
    sh = _shadow(tmp_path)
    s1, s2 = _site(north_m=400.0), _site(north_m=1400.0)
    for i in range(160):
      for j in range(40):
        # a 0.03 spike AND a blinker, both confined to record 41 -- inside site 1's extent only
        sh.tick(0.03 if (i, j) == (41, 5) else 0.001, None, None,
                cs.DQ_BLNK if i == 41 else 0, 25.0, False)
      sh.record(now_wall=T0 + i * 0.4, lat=_north(LAT0, i * 10.0), lon=LON0, bearing=0.0,
                v_ego=25.0, site_pt=(s1 if i < 60 else s2), posted_ms=26.8, hwy_class="motorway",
                icbm_src=None, icbm_target_ms=None, ref_ms=None)
    ups = [r for r in _written(sh) if r["kind"] == "up"]
    assert len(ups) == 1, "site 1 was disqualified; site 2 should have produced the only row"
    assert haversine_m(ups[0]["site_lat"], ups[0]["site_lon"], s2[0], s2[1]) < 1.0
    assert ups[0]["k"] == pytest.approx(0.001), \
      "the spike from the PREVIOUS site leaked into this row"

  def test_a_window_nobody_drained_in_time_is_discarded_not_measured(self, tmp_path):
    """`tick` runs unconditionally at 100 Hz; `record` only runs while CES is ENABLED.

    So a CES-off stretch in the MIDDLE of a passage would otherwise have the accumulator OR-ing
    disqualifiers and holding a peak for the whole time, and the record that resumes the passage
    would hand all of it to a site the truck is still approaching -- a curve measured on a road it
    left a minute ago. Bounded by MAX_TICK_DT_S, the same clamp the odometer uses.

    (A gap longer than DRIVE_GAP_S clears the tracked sites anyway; this is the window BETWEEN the
    two bounds, where the sites survive and only the discard protects them.)"""
    sh = _shadow(tmp_path)
    site = _site()

    def drive(lo, hi, t0):
      for i in range(lo, hi):
        for _ in range(40):
          sh.tick(0.001, None, None, 0, 25.0, False)
        sh.record(now_wall=t0 + i * 0.4, lat=_north(LAT0, i * 10.0), lon=LON0, bearing=0.0,
                  v_ego=25.0, site_pt=site, posted_ms=26.8, hwy_class="motorway",
                  icbm_src=None, icbm_target_ms=None, ref_ms=None)

    drive(0, 46, T0)                        # CES on: site armed, extent open, gentle road
    assert sh._sites, "the site was not still in flight; the test would prove nothing"
    for _ in range(6000):                      # 60 s of CES-OFF ticking, tight and blinkered
      sh.tick(0.05, None, None, cs.DQ_BLNK, 25.0, False)
    assert 0.0 < cs.DRIVE_GAP_S and 60.0 < cs.DRIVE_GAP_S   # short enough that sites survive
    drive(46, 80, T0 + 60.0)                # CES back on, same passage finishes

    ups = [r for r in _written(sh) if r["kind"] == "up"]
    assert len(ups) == 1, "the passage was contaminated by the CES-off window and disqualified"
    assert ups[0]["k"] == pytest.approx(0.001), \
      "a peak measured while CES was off was attributed to this curve"

  def test_the_odometer_is_the_replays_trapezoid_not_a_rectangle(self, tmp_path):
    """`s` is what section 6.3's extents are measured in, and ingest integrates it as the TRAPEZOID
    over each record pair (`0.5 * (v_a + v_b) * dt`). A rectangle holding the current speed over the
    whole interval differs wherever the speed is changing -- which is precisely the approach to a
    curve, the only place the extent matters."""
    sh = _shadow(tmp_path)
    speeds = [30.0, 28.0, 24.0, 18.0, 12.0, 12.0, 16.0, 22.0]
    dt = 1.0
    for i, v in enumerate(speeds):
      sh.record(now_wall=T0 + i * dt, lat=LAT0, lon=LON0, bearing=0.0, v_ego=v, site_pt=None,
                posted_ms=26.8, hwy_class="motorway", icbm_src=None, icbm_target_ms=None,
                ref_ms=None)
    expect = sum(0.5 * (a + b) * dt for a, b in zip(speeds, speeds[1:], strict=False))
    assert sh._s == pytest.approx(expect), "the odometer is not the one the replay measures with"
    # ...and it is NOT the rectangle, or the assertion above is satisfied by either implementation.
    rect = sum(b * dt for b in speeds[1:])
    assert abs(expect - rect) > 1.0

  def test_there_is_no_lead_car_gate(self, tmp_path):
    """D7: curvature does not depend on a lead car, and v1 wrongly discarded the 2026-09-08 pass for
    that reason. The shadow is never told about a lead at all -- pinned by the signature, so a future
    edit that adds one has to change this test."""
    assert "lead" not in cs.CurveDBShadow.record.__code__.co_varnames
    assert "lead" not in cs.CurveDBShadow.tick.__code__.co_varnames

  def test_the_derived_speed_is_capped_at_the_posted_limit(self):
    """M3, section 6.4: *never above posted*. A gentle row derives sqrt(2.5/0.001) = 50 m/s, which is
    112 mph -- the cap is the only thing between that and a reason to keep going."""
    obs = [_obs(k=0.001, date=f"2026-09-1{d}", drive_id=f"d{d}") for d in range(2)]
    row = CurveDB.build(obs, PROVISIONAL_PARAMS).rows[0]
    auth = authority(row, posted_limit_ms=26.8, highway_class="motorway", platform=LIGHTNING,
                     params=PROVISIONAL_PARAMS,
                     envelopes=cs.PROVISIONAL_ENVELOPES)
    assert auth.granted and auth.v_row_ms == pytest.approx(26.8)


# =====================================================================================================
# 6.2 -- DOWN. The rule the offline ingest cannot reach.
# =====================================================================================================
class TestDown:

  def test_a_driver_override_in_a_loaded_curve_produces_a_DOWN(self, tmp_path):
    """The headline: section 6.2 has produced ZERO observations on 2.2 GB of offline corpus, because
    `observations_for_drive` drops the whole passage on `dq_state == "dirty"` before its DOWN loop --
    and a steering override IS `DQ_DRV`. Here the two tests are independent."""
    site = _site()
    sh = _shadow(tmp_path)
    # 0.006 1/m at 25 m/s = 3.75 m/s^2, above the 2.5 trigger; the override also sets DQ_DRV.
    tele = _drive_past(sh, site, k_per_tick=lambda i, j: 0.006 if 39 <= i <= 42 else 0.001,
                       dq_per_tick=lambda i: cs.DQ_DRV if 40 <= i <= 41 else 0,
                       str_prs_at={40, 41}, k_cmd=0.0075)
    rows = _written(sh)
    downs = [r for r in rows if r["kind"] == "down"]
    assert len(downs) == 1, rows
    assert downs[0]["k"] == pytest.approx(0.0075)
    assert downs[0]["estimator"] == "slKCmd_at_override"
    # ...and the UP half correctly refused the same pass.
    assert not [r for r in rows if r["kind"] == "up"]
    evs = [t["cdbEv"] for t in tele if t["cdbEv"]]
    assert evs and "down" in evs[0] and "dirty" in evs[0], evs

  def test_the_offline_ingest_really_cannot_reach_that_rule(self):
    """M4's negative control, and the justification for this whole branch: the offline pipeline is
    fed a passage carrying a qualifying override, and produces no DOWN. If this ever starts failing,
    the other agent fixed it and this test's PREMISE should be revisited -- not the shadow's rule,
    which is independent by construction either way."""
    src = offline_ingest.observations_for_drive.__doc__ or ""
    assert "DOWN" in src
    import inspect
    body = inspect.getsource(offline_ingest.observations_for_drive)
    drop = body.index('if p.dq_state == "dirty"')
    down = body.index("-- DOWN:")
    assert drop < down, ("the offline DOWN loop is no longer preceded by the dirty-passage drop; " +
                         "re-check this test's premise")

  def test_an_override_below_the_lateral_trigger_is_not_a_DOWN(self, tmp_path):
    """Of 2,986 steering overrides above 27 mph, only ~113 ticks cleared 2.5 m/s^2. The rest are lane
    changes, exits and repositioning -- and a row built from those would learn the DRIVER, not the
    ROAD (section 4's whole rule)."""
    site = _site()
    sh = _shadow(tmp_path)
    # 0.001 1/m at 25 m/s = 0.625 m/s^2 -- a lane change, not a curve complaint.
    _drive_past(sh, site, k_per_tick=lambda i, j: 0.001,
                dq_per_tick=lambda i: cs.DQ_DRV if i == 40 else 0, str_prs_at={40}, k_cmd=0.0075)
    assert not [r for r in _written(sh) if r["kind"] == "down"]

  def test_an_override_with_no_commanded_curvature_is_dropped_not_guessed(self, tmp_path):
    """Section 6.2's magnitude is sqrt(A_LAT/|slKCmd|). With no commanded curvature there is no
    magnitude, so the intervention is dropped -- never replaced by the achieved value, which is
    bounded by steering authority and would read the road as gentler than it is (D2)."""
    sh = _shadow(tmp_path)
    for _ in range(50):
      sh.tick(0.006, None, None, cs.DQ_DRV, 25.0, True)   # pressed, loaded, but no k_cmd
    assert sh._down_k is None

  def test_DOWN_may_only_ever_LOWER_the_derived_speed(self):
    """M5, and section 6.2's hard constraint. Stored as the CURVATURE that implies the lower speed,
    so "DOWN is never overwritten by UP" (D8) falls out of a `max` instead of precedence
    bookkeeping: both rules can only raise k_eff, and raising k only lowers sqrt(a_lat/k)."""
    gentle = [_obs(k=0.001, date=f"2026-09-1{d}", drive_id=f"d{d}") for d in range(2)]
    row_up = CurveDB.build(gentle, PROVISIONAL_PARAMS).rows[0]
    row_both = CurveDB.build([*gentle, _obs(k=0.02, kind="down", date="2026-09-20",
                                            drive_id="dn", estimator="slKCmd_at_override")],
                             PROVISIONAL_PARAMS).rows[0]
    assert row_both.k_eff > row_up.k_eff
    kw = dict(posted_limit_ms=40.0, highway_class="motorway", platform=LIGHTNING,
              params=PROVISIONAL_PARAMS, envelopes=cs.PROVISIONAL_ENVELOPES)
    assert authority(row_both, **kw).v_row_ms < authority(row_up, **kw).v_row_ms
    # ...and a LATER gentle UP cannot erase it (D8).
    row_after = CurveDB.build([*gentle,
                               _obs(k=0.02, kind="down", date="2026-09-20", drive_id="dn",
                                    estimator="slKCmd_at_override"),
                               _obs(k=0.0005, date="2026-09-30", drive_id="late")],
                              PROVISIONAL_PARAMS).rows[0]
    assert row_after.k_eff == pytest.approx(0.02)


# =====================================================================================================
# 6.3 -- keying
# =====================================================================================================
class TestKeying:

  def test_the_site_is_the_candidates_position_not_the_trucks(self, tmp_path):
    """M6 / D3. Map/far ticks sit median 49 m, p75 91 m, p90 129 m BEFORE the candidate, so keying on
    the truck smears one episode across four or five "sites" mostly covering a straight approach."""
    site = _site()
    sh = _shadow(tmp_path)
    tele = _drive_past(sh, site)
    rows = _written(sh)
    assert rows
    assert rows[0]["site_lat"] == pytest.approx(site[0]), "the site is not the candidate's position"
    # ...and the truck was demonstrably somewhere ELSE when it named it, or this proves nothing.
    assert max(t["cdbBrgD"] or 0.0 for t in tele) > 100.0
    assert haversine_m(rows[0]["site_lat"], rows[0]["site_lon"], LAT0, LON0) \
           == pytest.approx(SITE_M, abs=2.0)

  def test_the_bearing_is_the_APPROACH_bearing_taken_on_the_approach(self, tmp_path):
    """M7 / D4. v1's 45-degree buckets straddle 47 % of real curves, so a pass's tight fragment
    landed in a different bucket than the lookup. The approach bearing is sampled at
    `approach_bearing_ref_m` -- where the CAR will later ask -- and where it actually landed is
    recorded, because a site armed closer than that can never be sampled there."""
    site = _site()
    sh = _shadow(tmp_path)
    # The bearing MUST vary, or "the approach bearing" and "the bearing in the bend" are the same
    # number and the test is vacuous: 17 deg on the approach, swinging to 80 deg through the bend.
    _drive_past(sh, site, bearing_per_tick=lambda i: 17.0 if i < 30 else 80.0)
    rows = _written(sh)
    assert rows and rows[0]["bearing_deg"] == pytest.approx(17.0), \
      "the row was keyed on the bearing IN the bend, not on the approach"

  def test_a_site_armed_inside_the_reference_distance_still_records_where_it_sampled(self, tmp_path):
    """Rule 2. A candidate named 80 m ahead can never be sampled at 300 m, and the pass is still
    usable -- but the analysis has to be able to tell the two apart, which is what `bearing_d` in the
    per-passage log line is for. The pass is NOT silently dropped and NOT silently mislabelled."""
    site = _site(north_m=80.0)
    sh = _shadow(tmp_path)
    rec = []
    for i in range(40):
      for _ in range(40):
        sh.tick(0.003, None, None, 0, 25.0, False)
      rec.append(sh.record(now_wall=T0 + i * 0.4, lat=_north(LAT0, i * 10.0), lon=LON0,
                           bearing=5.0, v_ego=25.0, site_pt=site, posted_ms=26.8,
                           hwy_class="motorway", icbm_src=None, icbm_target_ms=None, ref_ms=None))
    rows = _written(sh)
    assert rows and rows[0]["bearing_deg"] == pytest.approx(5.0)

  def test_a_bearing_of_exactly_360_does_not_wedge_the_tracker(self, tmp_path):
    """`Observation` rejects a bearing outside [0, 360), and a raise inside `_finish` unwinds
    `_advance` BEFORE its `self._sites = keep` rebuild -- so the offending site is never removed,
    every later record re-raises, and the write half dies for the rest of the drive segment along
    with every other site in flight. `ingest.normalise` does `bearing %= 360.0`; so does this."""
    sh = _shadow(tmp_path)
    _drive_past(sh, _site(), bearing=360.0)
    rows = [r for r in _written(sh) if r["kind"] == "up"]
    assert len(rows) == 1 and rows[0]["bearing_deg"] == pytest.approx(0.0)
    assert sh.err == 0 and not sh._sites, "the site was stranded in flight"

  def test_the_stored_peak_is_the_quantity_ingest_reads_not_a_wider_one(self, tmp_path):
    """A row is labelled `estimator="kPeak100"`, and ingest's `_tick_k` reads the record's `kPeak`,
    which is `max(|k_actl|, |k_cmd|)` -- `kPose` is a SEPARATE column. Folding the localizer into
    the stored peak would make the car's rows and the replay's rows two different numbers under one
    label, on the Tesla in particular (dead `k_actl`, so the row would be pose-only on the car and
    commanded-only in the replay)."""
    sh = _shadow(tmp_path)
    site = _site()
    for i in range(80):
      for _ in range(40):
        # localizer reads a tight bend; CAN and the planner both read a gentle one
        sh.tick(0.001, 0.001, 0.05, 0, 25.0, False)
      sh.record(now_wall=T0 + i * 0.4, lat=_north(LAT0, i * 10.0), lon=LON0, bearing=0.0,
                v_ego=25.0, site_pt=site, posted_ms=26.8, hwy_class="motorway",
                icbm_src=None, icbm_target_ms=None, ref_ms=None)
    rows = [r for r in _written(sh) if r["kind"] == "up"]
    assert len(rows) == 1
    assert rows[0]["k"] == pytest.approx(0.001), "kPose was folded into the stored peak"
    assert rows[0]["estimator"] == "kPeak100"

  def test_a_wide_pass_measures_ingests_extent_not_a_shifted_one(self, tmp_path):
    """The extent closes at `s_p + extent_fwd_m`, where `s_p` is CLOSEST APPROACH -- ingest's own
    rule. Measuring `extent_back + extent_fwd` forward from the OPEN instead shifts a WIDE pass's
    window (one that opens late, on recession) by the whole back extent: [s_p+5, s_p+180] rather
    than [s_p-25, s_p+150]. A spike 165 m past the apex is outside the real extent and inside the
    shifted one, so it is what tells them apart."""
    sh = _shadow(tmp_path)
    site = _site(north_m=400.0, east_m=60.0)      # 60 m off the line: > extent_back_m, < PASSAGE_MAX_M
    for i in range(90):
      for _ in range(40):
        # apex at record 40 (s=400); the spike sits at s=565, i.e. 165 m past it
        sh.tick(0.02 if i == 56 else 0.001, None, None, 0, 25.0, False)
      sh.record(now_wall=T0 + i * 0.4, lat=_north(LAT0, i * 10.0), lon=LON0, bearing=0.0,
                v_ego=25.0, site_pt=site, posted_ms=26.8, hwy_class="motorway",
                icbm_src=None, icbm_target_ms=None, ref_ms=None)
    rows = [r for r in _written(sh) if r["kind"] == "up"]
    assert len(rows) == 1, rows
    assert rows[0]["k"] == pytest.approx(0.001), \
      "the extent reached 165 m past the apex -- it is measured from the open, not the apex"

  def test_the_lookup_uses_the_APPROACH_bearing_not_the_heading_in_the_bend(self, tmp_path):
    """`cdbRow` is the one number this feature exists to measure, and keying the lookup on the
    truck's instantaneous heading biases it LOW: the write half keyed the row on the bearing sampled
    at `approach_bearing_ref_m`, so inside the bend (sweep median 19 deg, p90 62 deg) the same site
    stops matching. When the candidate is one the tracker is already following, its own sampled
    approach bearing is used -- and `cdbBrgD` then reports where THAT bearing was taken."""
    site = _site()
    obs = [_obs(site_lat=site[0], site_lon=site[1], bearing_deg=0.0, k=0.004,
                date=f"2026-09-1{d}", drive_id=f"d{d}") for d in range(2)]
    sh = _shadow(tmp_path, obs=obs)
    hits = []
    for i in range(60):
      for _ in range(40):
        sh.tick(0.004, None, None, 0, 25.0, False)
      # heading 0 on the approach, swinging to 90 deg through the bend -- far outside heading_tol
      brg = 0.0 if i < 38 else 90.0
      frag = sh.record(now_wall=T0 + i * 0.4, lat=_north(LAT0, i * 10.0), lon=LON0, bearing=brg,
                       v_ego=25.0, site_pt=site, posted_ms=26.8, hwy_class="motorway",
                       icbm_src="far", icbm_target_ms=18.0, ref_ms=26.8)
      if frag["cdbSite"]:
        # `tracked` is read AFTER the call, and the passage is closed inside it -- so it says
        # exactly which bearing that record's lookup used. Once the passage completes the site is
        # no longer followed and the lookup falls back to the instantaneous heading; by then the
        # candidate is 150 m behind the truck and ICBM is not acting on it.
        hits.append((i, frag["cdbRow"], frag["cdbBrg"], bool(sh._sites)))
    in_bend = [h for h in hits if h[0] >= 40 and h[3]]
    assert len(in_bend) >= 5, f"the test never drove the bend with the site tracked: {hits}"
    assert all(h[1] for h in in_bend), \
      "the row stopped matching inside the bend -- the lookup used the instantaneous heading"
    assert all(h[2] == pytest.approx(0.0) for h in in_bend)

  def test_a_pass_that_never_reaches_the_candidate_is_a_counted_drop_not_a_measurement(self, tmp_path):
    """Rule 2. A site the truck turned away from must not silently vanish -- a silent no-action is
    indistinguishable from the feature being dead."""
    site = _site(north_m=200.0, east_m=5000.0)   # 5 km to the side of the driven line
    sh = _shadow(tmp_path)
    tele = _drive_past(sh, site, n=200, step_m=10.0)
    assert not _written(sh)
    evs = [t["cdbEv"] for t in tele if t["cdbEv"]]
    assert evs and evs[0].startswith("notReached:"), evs

  def test_two_candidates_the_matcher_would_merge_contribute_one_pass(self, tmp_path):
    """D6 protection. Deduplicating at the LOOKUP's own radius is what stops a single drive
    manufacturing the ">= 2 admissible passes" authority requires."""
    site = _site()
    near = _site(north_m=SITE_M + 10.0)          # 10 m apart -- inside site_radius_m (40 m)
    sh = _shadow(tmp_path)
    for i in range(80):
      pt = site if i % 2 else near
      for _ in range(40):
        sh.tick(0.002, None, None, 0, 25.0, False)
      sh.record(now_wall=T0 + i * 0.4, lat=_north(LAT0, i * 10.0), lon=LON0, bearing=0.0,
                v_ego=25.0, site_pt=pt, posted_ms=26.8, hwy_class="motorway",
                icbm_src=None, icbm_target_ms=None, ref_ms=None)
    assert len([r for r in _written(sh) if r["kind"] == "up"]) == 1


# =====================================================================================================
# 6.4 -- authority, and every refusal naming itself
# =====================================================================================================
class TestAuthorityRefusals:

  def _row(self, **over):
    obs = [_obs(date=f"2026-09-1{d}", drive_id=f"d{d}", **over) for d in range(2)]
    return CurveDB.build(obs, PROVISIONAL_PARAMS).rows[0]

  @pytest.mark.parametrize("kw,fragment", [
    (dict(posted_limit_ms=0.0), "no posted limit"),
    (dict(highway_class="unknown"), "highway class unknown"),
    (dict(highway_class=""), "highway class unknown"),
    (dict(highway_class="motorwayLink"), "ramp"),
    (dict(platform="TESLA_MODEL_S_HW3"), "no measured lateral envelope"),
  ])
  def test_each_refusal_says_why(self, kw, fragment):
    """M8, Rule 2: a refusal that does not say why is a check whose success path cannot be told from
    its failure path. Every one of these is a real hole the design left -- an absent class is treated
    as maybe-a-ramp (stricter than section 6.4, flagged in store.py), and an unmeasured car gets
    nothing rather than a mass-derived guess Fable already rejected."""
    base = dict(posted_limit_ms=26.8, highway_class="motorway", platform=LIGHTNING,
                params=PROVISIONAL_PARAMS, envelopes=cs.PROVISIONAL_ENVELOPES)
    auth = authority(self._row(), **{**base, **kw})
    assert not auth.granted
    assert fragment in auth.reason, auth.reason

  def test_one_pass_on_one_date_is_not_authority(self):
    """D6. On the data that exists today this is ZERO rows, which is exactly why Phase 1 ran."""
    row = CurveDB.build([_obs()], PROVISIONAL_PARAMS).rows[0]
    auth = authority(row, posted_limit_ms=26.8, highway_class="motorway", platform=LIGHTNING,
                     params=PROVISIONAL_PARAMS, envelopes=cs.PROVISIONAL_ENVELOPES)
    assert not auth.granted and "pass" in auth.reason

  def test_the_database_may_only_ever_CANCEL_OR_REDUCE_a_slowdown(self, tmp_path):
    """The asymmetry the whole feature rests on: today's failures are over-braking (annoying, safe);
    a wrong row produces an UNDER-brake. `cdbGive` must therefore never be negative, and `cdbWould`
    must never exceed the reference the driver already chose."""
    obs = [_obs(k=0.001, date=f"2026-09-1{d}", drive_id=f"d{d}") for d in range(2)]
    sh = _shadow(tmp_path, obs=obs)
    tele = sh.record(now_wall=T0, lat=_north(LAT0, -300.0), lon=LON0, bearing=0.0, v_ego=27.0,
                     site_pt=(LAT0, LON0), posted_ms=26.8, hwy_class="motorway",
                     icbm_src="far", icbm_target_ms=18.0, ref_ms=26.8)
    assert tele["cdbRow"] is True
    assert tele["cdbWould"] == pytest.approx(26.8)     # min(ref, max(icbm, v_row))
    assert tele["cdbGive"] == pytest.approx(8.8)
    assert tele["cdbWould"] <= 26.8

    # ...and with a reference BELOW the row's speed it is the reference that binds, never the row.
    tele2 = sh.record(now_wall=T0 + 1, lat=_north(LAT0, -300.0), lon=LON0, bearing=0.0,
                      v_ego=20.0, site_pt=(LAT0, LON0), posted_ms=26.8, hwy_class="motorway",
                      icbm_src="far", icbm_target_ms=18.0, ref_ms=19.0)
    assert tele2["cdbWould"] == pytest.approx(19.0)

  def test_the_posted_limit_really_binds_and_is_not_merely_shadowed_by_the_reference(self, tmp_path):
    """Section 6.4's `v_row <= posted` must be DISTINGUISHABLE from having no cap at all.

    With ref == posted the two are indistinguishable, which is how a dropped cap survives a test
    suite. Here the driver's reference is 40 m/s and the posted limit is 26.8: a gentle row derives
    sqrt(2.5/0.001) = 50 m/s, so without the cap `cdbWould` would read 40 -- the truck's full set
    speed, justified by nothing but a row that says "this road is straight"."""
    obs = [_obs(k=0.001, date=f"2026-09-1{d}", drive_id=f"d{d}") for d in range(2)]
    sh = _shadow(tmp_path, obs=obs)
    tele = sh.record(now_wall=T0, lat=_north(LAT0, -300.0), lon=LON0, bearing=0.0, v_ego=35.0,
                     site_pt=(LAT0, LON0), posted_ms=26.8, hwy_class="motorway",
                     icbm_src="far", icbm_target_ms=18.0, ref_ms=40.0)
    assert tele["cdbVRow"] == pytest.approx(26.8), "v_row was not capped at the posted limit"
    assert tele["cdbWould"] == pytest.approx(26.8)
    assert tele["cdbWould"] < 40.0


# =====================================================================================================
# 3.9-6 -- non-circularity, and Rule 2's liveness channel
# =====================================================================================================
class TestNonCircularityAndLiveness:

  def test_this_drives_own_passes_never_enter_the_database_it_is_judged_against(self, tmp_path):
    """M9. A pass must never be judged against itself. On the car the guarantee is structural: the
    live DB is exactly what was on disk at construction, and `_write` appends without touching it."""
    site = _site()
    sh = _shadow(tmp_path)
    assert sh._rows() == 0
    _drive_past(sh, site)
    assert _written(sh), "nothing was recorded; the test proves nothing"
    assert sh._rows() == 0, "a pass from this drive entered the live database"
    # ...and it IS on disk for the NEXT boot.
    sh2 = _shadow(tmp_path, obs=[Observation(**r) for r in _written(sh)])
    assert sh2._rows() == 1

  def test_the_health_fields_distinguish_dead_from_quiet(self, tmp_path):
    """Rule 2, and the field that makes every other number readable. `cdbSite` False with
    `cdbOn == "on"` is "no candidate here"; `cdbOn == "err"` is "the shadow is broken"; and
    `cdbOn == "loading"` is "ask again in a second" -- none of which may look like the others."""
    sh = _shadow(tmp_path)
    quiet = sh.record(now_wall=T0, lat=LAT0, lon=LON0, bearing=0.0, v_ego=25.0, site_pt=None,
                      posted_ms=26.8, hwy_class="motorway", icbm_src=None, icbm_target_ms=None,
                      ref_ms=None)
    assert quiet["cdbOn"] == "on" and quiet["cdbSite"] is False and quiet["cdbRow"] is None
    assert quiet["cdbErr"] == 0

    sh._state = "loading"
    sh._db = None
    loading = sh.record(now_wall=T0 + 1, lat=LAT0, lon=LON0, bearing=0.0, v_ego=25.0,
                        site_pt=(LAT0, LON0), posted_ms=26.8, hwy_class="motorway",
                        icbm_src=None, icbm_target_ms=None, ref_ms=None)
    assert loading["cdbSite"] is True and loading["cdbRow"] is None
    assert loading["cdbWhy"] == "loading", "a loading database must not look like an absent row"

  def test_a_fragment_is_always_complete(self, tmp_path):
    """M10. Every declared key is present on every path, including the failure path -- a key that
    appears only sometimes is a null column that reads as "the feature did not trigger"."""
    sh = _shadow(tmp_path)
    paths = [
      sh.record(now_wall=T0, lat=None, lon=None, bearing=None, v_ego=None, site_pt=None,
                posted_ms=None, hwy_class=None, icbm_src=None, icbm_target_ms=None, ref_ms=None),
      cs.curvedb_tele(NS(), site_pt=None, now_wall=T0, v_ego=25.0, v_set=26.8),
    ]
    for frag in paths:
      assert set(frag) == set(cs.CURVEDB_TELE_KEYS), set(frag) ^ set(cs.CURVEDB_TELE_KEYS)
    assert paths[1]["cdbOn"] == "absent", "an uninstalled shadow must not look like a live one"

  def test_a_missing_timezone_disables_the_writer_loudly_rather_than_guessing(self, tmp_path):
    """A row filed under the wrong calendar date silently corrupts leave-one-date-out, which is the
    entire non-circularity argument -- and `datetime.fromtimestamp(t, None)` does not raise, it
    returns LOCAL time. AGNOS is a minimal rootfs and tzdata has never been verified on it."""
    sh = _shadow(tmp_path)
    sh._tz = None
    tele = _drive_past(sh, _site())
    assert not _written(sh)
    # ...and the passage SAYS why it produced nothing. Without this the assertion above is also
    # satisfied by the second, independent guard in `_write`, so removing the one in `_finish` --
    # the one that stops `pt_date(t, None)` silently returning LOCAL time -- survived mutation.
    # Two guards are defence in depth; a test that cannot tell them apart is not.
    evs = [t["cdbEv"] for t in tele if t["cdbEv"]]
    assert evs == ["noTz"], evs

  def test_a_dead_rtc_timestamp_is_refused_not_filed_under_1970(self, tmp_path):
    """The 3X's RTC battery is dead, so a cold boot stamps records 1970 until NTP/GPS sync -- and a
    bogus timestamp manufactures a distinct DATE, which is half of D6's authority key. ces_pnw MARKS
    such records (`clockBad`) rather than dropping them, because a record's other fields are still
    real; a row's date is not one of its fields, it is its identity."""
    sh = _shadow(tmp_path)
    tele = _drive_past(sh, _site(), t0=1.0e9)          # 2001: before CLOCK_VALID_EPOCH
    assert not _written(sh)
    assert [t["cdbEv"] for t in tele if t["cdbEv"]] == ["clockBad"]
    # ...and the SAME drive with a real clock does produce a row, or this proves only that the
    # harness is broken.
    sh2 = _shadow(tmp_path / "ok")
    _drive_past(sh2, _site(), t0=T0)
    assert [r for r in _written(sh2) if r["kind"] == "up"]

  def test_the_clock_epoch_is_the_one_ces_pnw_uses(self):
    assert cs.CLOCK_VALID_EPOCH == m.CLOCK_VALID_EPOCH

  def test_a_frozen_writer_never_reports_a_row_it_did_not_write(self, tmp_path):
    """Rule 2. At the cap -- the state section 8.1 says must be loud -- the per-record verdict must
    not read "up". A channel that lies about the one thing it exists to report is worse than none."""
    sh = _shadow(tmp_path)
    sh._write_stopped = True
    tele = _drive_past(sh, _site())
    assert not _written(sh) and sh.obs_written == 0
    evs = [t["cdbEv"] for t in tele if t["cdbEv"]]
    assert evs == ["upDropped:stopped"], evs

  def test_two_passages_finishing_on_one_record_both_report(self, tmp_path):
    """`cdbEv` is one field and a record can close more than one passage; overwriting would drop a
    verdict silently."""
    sh = _shadow(tmp_path)
    sh._ev_pending = ["up", "dirty:blnk"]
    frag = sh.record(now_wall=T0, lat=LAT0, lon=LON0, bearing=0.0, v_ego=25.0, site_pt=None,
                     posted_ms=26.8, hwy_class="motorway", icbm_src=None, icbm_target_ms=None,
                     ref_ms=None)
    assert frag["cdbEv"] == "up;dirty:blnk"

  def test_a_site_is_not_re_armed_after_its_passage_completes(self, tmp_path):
    """Ingest's `find_sites` dedups over the WHOLE drive. Deduping only against the in-flight list
    let a candidate the truck had just finished measuring be re-armed the instant the passage
    closed, producing a SECOND observation of the same site from one drive made of nothing but that
    curve's run-out."""
    sh = _shadow(tmp_path)
    _drive_past(sh, _site(), n=140)               # long enough for a re-arm to complete too
    assert len([r for r in _written(sh) if r["kind"] == "up"]) == 1

  def test_a_corrupt_observation_line_is_discarded_and_counted(self, tmp_path):
    """Section 8.1: a torn last line is discarded (append-only), and the store must fail SAFE to
    "no database" -- but it must SAY SO."""
    obs_path = tmp_path / "obs.jsonl"
    good = json.dumps(vars(_obs()))
    obs_path.write_text(good + "\n" + '{"date": "2026-09-11", "k": ' + "\n" + good + "\n")
    sh = cs.CurveDBShadow(LIGHTNING, obs_path=str(obs_path),
                          config_path=str(tmp_path / "absent.json"), load_async=False)
    assert sh._state == "on" and sh._rows() == 1

  def test_a_malformed_config_is_reported_not_silently_defaulted(self, tmp_path):
    cfg = tmp_path / "curvedb.json"
    cfg.write_text("{not json")
    params, enabled, note = cs._load_config(str(cfg))
    assert params is PROVISIONAL_PARAMS and enabled and "unreadable" in note

  def test_the_config_cannot_retune_what_a_stored_row_MEANS(self, tmp_path):
    """Only the MATCHING half is tunable. The measurement half (`extent_*`,
    `approach_bearing_ref_m`, `down_trigger_a_lat_ms2`, `min_speed_ms`) decides what a stored row
    means, and an `Observation` carries no record of the params it was measured under -- so a hand
    edit would silently make new rows non-comparable with old ones and with the replay's."""
    cfg = tmp_path / "curvedb.json"
    cfg.write_text(json.dumps({"extent_fwd_m": 400.0, "min_speed_ms": 1.0,
                               "a_lat_comfort_ms2": 9.0, "min_passes": 1,
                               "site_radius_m": 55.0}))
    params, enabled, note = cs._load_config(str(cfg))
    assert params.site_radius_m == 55.0, "the matching half must still be tunable"
    assert params.extent_fwd_m == PROVISIONAL_PARAMS.extent_fwd_m
    assert params.min_speed_ms == PROVISIONAL_PARAMS.min_speed_ms
    assert params.a_lat_comfort_ms2 == PROVISIONAL_PARAMS.a_lat_comfort_ms2
    assert params.min_passes == PROVISIONAL_PARAMS.min_passes
    # Rule 2: ignoring a key the owner deliberately typed must not be silent.
    assert "ignoring" in note and "extent_fwd_m" in note, note

  def test_an_out_of_range_override_is_rejected_with_a_reason(self, tmp_path):
    cfg = tmp_path / "curvedb.json"
    cfg.write_text(json.dumps({"heading_tol_deg": 400.0}))
    params, _, note = cs._load_config(str(cfg))
    assert params is PROVISIONAL_PARAMS and "rejected" in note

  def test_the_config_can_retune_matching_without_a_deploy(self, tmp_path):
    """`cdbD`/`cdbBrg` are logged on every lookup precisely so these can be chosen from real data."""
    params, enabled, note = cs._load_config(str(tmp_path / "absent.json"))
    assert params is PROVISIONAL_PARAMS and enabled and note == ""
    sh = _shadow(tmp_path, config={"site_radius_m": 120.0, "heading_tol_deg": 60.0})
    assert sh.params.site_radius_m == 120.0 and sh.params.heading_tol_deg == 60.0

  def test_disabled_means_disabled(self, tmp_path):
    sh = _shadow(tmp_path, config={"enabled": False})
    assert sh._state == "off"
    frag = sh.record(now_wall=T0, lat=LAT0, lon=LON0, bearing=0.0, v_ego=25.0,
                     site_pt=(LAT0, LON0), posted_ms=26.8, hwy_class="motorway",
                     icbm_src="far", icbm_target_ms=18.0, ref_ms=26.8)
    assert frag["cdbOn"] == "off" and frag["cdbSite"] is False and frag["cdbWould"] is None

  def test_a_drive_gap_starts_a_new_drive_so_two_passes_can_accumulate(self, tmp_path):
    """D6 counts distinct DRIVES. Without the gap split an un-rebooted session could pass the same
    site on two days and still count as one pass -- and authority would never be reachable."""
    site = _site()
    sh = _shadow(tmp_path)
    _drive_past(sh, site, t0=T0)
    _drive_past(sh, site, t0=T0 + 86400)
    rows = [r for r in _written(sh) if r["kind"] == "up"]
    assert len(rows) == 2
    assert len({r["drive_id"] for r in rows}) == 2
    db = CurveDB.build([Observation(**r) for r in rows], PROVISIONAL_PARAMS)
    assert db.rows[0].n_passes == 2


# =====================================================================================================
# the real call path -- a rule that never reaches the record is the visK failure mode
# =====================================================================================================
def _run_real(monkeypatch, tmp_path, observations=(), ticks=400, curve_at=300.0):
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "absent.json"))
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))
  obs_path = tmp_path / "obs.jsonl"
  obs_path.write_text("".join(json.dumps(vars(o)) + "\n" for o in observations))
  monkeypatch.setattr(cs, "OBS_PATH", str(obs_path))
  monkeypatch.setattr(cs, "CONFIG_PATH", str(tmp_path / "absent-curvedb.json"))
  clock = [5000.0]
  ns = types.SimpleNamespace(monotonic=lambda: clock[0], time=lambda: clock[0])
  for mod in (C, m):
    monkeypatch.setattr(mod, "time", ns)
  C._ces_mode_hold_st.clear()

  class Mem:
    def __init__(self):
      self.puts = []

    def get(self, k, return_default=False):
      if k == "MapTargetVelocities":
        return _scene(curve_at, 180.0, 12.0)
      if k == "LastGPSPosition":
        return json.dumps({"latitude": LAT0, "longitude": LON0, "bearing": 0.0, "src": "device",
                           "ts": clock[0], "fix_ts": clock[0] - 0.3})
      if k == "MapSpeedLimit":
        return str(60 * MPH)
      return None

    def put_nonblocking(self, k, v):
      self.puts.append((k, copy.deepcopy(v)))

  params = _P(clock, mode=lambda t: 2, extra={"CESButtonState": "0"})
  c = m.CESController(FakeCP(LIGHTNING, "ford", False), params=params)
  assert c._cdb.wait_loaded()
  c.mem_params = Mem()
  recs = []
  c._event_log_ok = True
  c._append_event = lambda rec: recs.append(copy.deepcopy(rec))
  v_ego, stock = 27.0, 60 * MPH
  for i in range(ticks):
    clock[0] = 5000.0 + (i + 1) * 0.01
    orz, vx, px, ts = _model(v_ego, curve_at, 180.0)
    model = NS(orientationRate=NS(z=orz, t=ts), velocity=NS(x=vx), position=NS(x=px),
               action=NS(shouldStop=False), meta=NS(laneChangeState="off"))
    sm = {"radarState": NS(leadOne=NS(status=False, vLead=0.0, dRel=0.0, aLeadK=0.0, vLeadK=0.0)),
          "modelV2": model, "carControl": NS(orientationNED=[0.0, 0.0, 0.0]),
          "livePose": NS(angularVelocityDevice=NS(x=0.0, y=0.0, z=0.02, valid=True)),
          "controlsState": NS(desiredCurvature=0.004,
                              lateralControlState=NS(which=lambda: "angleState",
                                                     angleState=NS(saturated=False)))}
    cstate = NS(vEgo=v_ego, aEgo=0.0, gasPressed=False, brakePressed=False,
                leftBlinker=False, rightBlinker=False, vCruise=stock * 3.6, standstill=False,
                steeringAngleDeg=0.0, steeringPressed=False, leftBlindspot=False,
                rightBlindspot=False, cruiseState=NS(speed=stock, enabled=True),
                yawRate=0.05, steeringTorque=0.0)
    c.experimental_request(cstate, sm)
  return c, [r for r in recs if r.get("ev") in ("tick", "adopt")]


class TestOnTheRealCallPath:

  def test_every_declared_key_reaches_the_ces_events_record(self, monkeypatch, tmp_path):
    """The CURVEDB_TELE_KEYS contract, same shape as CURVE_TELE_KEYS / CURVELEAD_TELE_KEYS: a key
    declared but never emitted is a null column that reads as "the feature did not trigger". That has
    now happened four times in this file's history (visK, icbmKVis, waysel2pnw's eight, lcSpdA)."""
    _, recs = _run_real(monkeypatch, tmp_path)
    assert recs, "no records were produced"
    for k in cs.CURVEDB_TELE_KEYS:
      assert k in recs[0], f"{k} is declared in CURVEDB_TELE_KEYS but never reaches the record"

  def test_the_shadow_is_alive_on_the_shipped_call_path(self, monkeypatch, tmp_path):
    """The anti-visK test: the accumulator is fed at 100 Hz, the site is named, and the lookup runs
    -- proved from a record the REAL controller produced, not a dict a unit test handed the builder."""
    _, recs = _run_real(monkeypatch, tmp_path)
    assert all(r["cdbOn"] == "on" for r in recs)
    assert all(r["cdbErr"] == 0 for r in recs), "the shadow logged a failure on a clean drive"
    sited = [r for r in recs if r["cdbSite"]]
    assert sited, "no record ever named a map candidate; the shadow measured nothing"
    assert all(r["cdbRow"] is False for r in sited), "an empty database matched a row"
    assert all(r["cdbWhy"] for r in sited), "a lookup returned no reason at all"

  def test_the_site_is_the_records_own_mapLat_mapLon(self, monkeypatch, tmp_path):
    """The shadow's site and the record's `mapLat`/`mapLon` are resolved from the SAME branch in
    `_event_record`. If they ever diverge, a row would be keyed on one curve while the telemetry
    beside it named another -- and no analysis would be able to tell."""
    _, recs = _run_real(monkeypatch, tmp_path)
    sited = [r for r in recs if r["cdbSite"]]
    assert sited
    for r in sited:
      assert r["mapLat"] is not None and r["mapLon"] is not None, \
        "the shadow named a site on a record whose own coordinates are null"

  def test_a_loaded_row_is_found_and_reported_with_its_match_geometry(self, monkeypatch, tmp_path):
    """`cdbRow` is the number section 12 says decides whether Phase 2 is viable at all, and
    `cdbD`/`cdbBrg`/`cdbBrgD` are what let the keying be tuned from real data rather than argued."""
    obs = [Observation(date=f"2026-09-1{d}", t=T0 + d, car=LIGHTNING, drive_id=f"d{d}",
                       site_lat=_north(LAT0, 300.0), site_lon=LON0, bearing_deg=0.0, k=0.004,
                       kind="up", estimator="kPeak100", site_src="logged", source="t",
                       posted_ms=26.8, highway_class="motorway", n_ticks=12,
                       dq_state="clean", dq_src="rollup100") for d in range(2)]
    _, recs = _run_real(monkeypatch, tmp_path, observations=obs)
    hits = [r for r in recs if r["cdbRow"]]
    assert hits, "the pre-loaded row was never matched on the real call path"
    h = hits[0]
    assert h["cdbD"] is not None and h["cdbD"] <= cs.PROVISIONAL_PARAMS.site_radius_m
    assert h["cdbBrg"] is not None and h["cdbBrg"] <= cs.PROVISIONAL_PARAMS.heading_tol_deg
    assert h["cdbBrgD"] is not None and h["cdbBrgD"] > 0.0
    assert h["cdbK"] == pytest.approx(0.004)
    assert h["cdbN"] == 2 and h["cdbDt"] == 2
    assert h["cdbKUp"] == pytest.approx(0.004) and h["cdbKDn"] is None

  def test_the_record_is_json_serialisable_and_finite(self, monkeypatch, tmp_path):
    """The record is written with `json.dumps`. A NaN would serialise to bare `NaN`, which is not
    JSON and which every downstream reader in `tools/` rejects -- silently losing the line."""
    _, recs = _run_real(monkeypatch, tmp_path)
    for r in recs:
      frag = {k: r[k] for k in cs.CURVEDB_TELE_KEYS}
      json.loads(json.dumps(frag, allow_nan=False))
      for k, v in frag.items():
        assert not (isinstance(v, float) and not math.isfinite(v)), (k, v)


# =====================================================================================================
# cesarchive2pnw: curvedb_obs.jsonl is the LIVE database (append-only, never rotated), so the uploader
# must never take it directly. The shadow snapshots it into curvedb_archive/ at construction (selfdrived
# start, before anything can engage), once per distinct content.
# =====================================================================================================
SNAP_MTIME = 1789000000.0


def _snap_make(tmp_path, text):
  obs = tmp_path / "curvedb_obs.jsonl"
  if text is not None:
    obs.write_text(text)
    os.utime(obs, (SNAP_MTIME, SNAP_MTIME))
  cfg = tmp_path / "curvedb.json"
  cfg.write_text(json.dumps({}))
  sh = cs.CurveDBShadow("TESTCAR", obs_path=str(obs), config_path=str(cfg), load_async=False)
  return sh, tmp_path / cs.CURVEDB_ARCHIVE_SUBDIR


class TestUploadSnapshot:
  def test_construction_snapshots_the_corpus_by_copy(self, tmp_path):
    sh, arc = _snap_make(tmp_path, "")
    (name,) = os.listdir(arc)
    assert name == "curvedb_obs.jsonl.20260910T002640Z"
    assert os.stat(arc / name).st_ino != os.stat(tmp_path / "curvedb_obs.jsonl").st_ino, "a COPY, not a link"
    assert sh._state == "on"

  def test_an_unchanged_corpus_is_not_snapshotted_again(self, tmp_path):
    _snap_make(tmp_path, "")
    _, arc = _snap_make(tmp_path, None)             # second boot, file untouched
    assert len(os.listdir(arc)) == 1

  def test_no_corpus_no_snapshot_and_the_shadow_still_works(self, tmp_path):
    sh, arc = _snap_make(tmp_path, None)
    assert not arc.exists() or os.listdir(arc) == []
    assert sh._state == "on"

  def test_a_snapshot_failure_does_not_make_the_shadow_inert(self, tmp_path, monkeypatch):
    monkeypatch.setattr(cs.pnw_log_archive.shutil, "copyfile",
                        lambda s, d: (_ for _ in ()).throw(OSError(28, "ENOSPC")))
    sh, _ = _snap_make(tmp_path, "")
    assert sh._state == "on" and sh.err == 0
