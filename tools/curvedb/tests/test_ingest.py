"""Unit tests for curvedb ingest -- normalisation, dedup, drives, sites, passages, episodes.

The synthetic drives here are exact: 1 Hz ticks, 25 m/s, due north, so one tick is 25 m of odometer
and every extent boundary lands on a tick this file can name. That is on purpose -- a test whose
expected value is computed the same way the code computes it proves only that the code is
self-consistent.
"""
import gzip
import json
import math

import pytest

from openpilot.tools.curvedb import ingest as I
from openpilot.tools.curvedb.store import PROVISIONAL_PARAMS, Observation, tighten

P = PROVISIONAL_PARAMS
LIGHTNING = "FORD_F_150_LIGHTNING_MK1"
TESLA = "TESLA_MODEL_S_HW3"
M_PER_DEG_LAT = math.pi * 6371000.0 / 180.0     # the same sphere haversine_m uses
T0 = 1789000000.0                                # 2026-09-09 17:26:40 PDT (2026-09-10 00:26 UTC)
LAT0, LON0 = 45.0, -122.0


def raw(i, *, v=25.0, car=LIGHTNING, t0=T0, **over):
  """One ces_events-shaped record, `i` ticks into a due-north drive at `v` m/s."""
  r = {"ev": "tick", "t": t0 + i, "car": car, "vEgo": v, "bearing": 0.0,
       "lat": LAT0 + (v * i) / M_PER_DEG_LAT, "lon": LON0,
       "strPrs": False, "blnk": False, "slAngSat": False, "slSat": False, "slCurvLim": False,
       "lcGate": "ok", "spdLim": 29.0, "hwyClass": "motorway"}
  r.update(over)
  return r


def drive_of(records):
  ticks = [t for t in (I.normalise(r, f"t:{i}") for i, r in enumerate(records)) if t is not None]
  drives = I.split_drives(ticks)
  assert len(drives) == 1, f"expected one drive, got {len(drives)}"
  return drives[0]


# ------------------------------------------------------------------------------------ primitives

@pytest.mark.parametrize(("v", "expected"), [
  (1.5, 1.5), (0, 0.0), (-2, -2.0), (True, None), (False, None), ("3", None), (None, None),
  (float("nan"), None), (float("inf"), None),
])
def test_num(v, expected):
  # True is not 1.0 m/s: a boolean sneaking into a numeric field is the capnp-enum trap's cousin.
  assert I._num(v) == expected


@pytest.mark.parametrize(("v", "expected"), [
  (0.004, 0.004), (-0.004, 0.004),
  (0.0, None),        # D1: an exact zero is a dead sensor, not a straight road
  (-0.0, None), (None, None), (True, None), (float("nan"), None),
])
def test_k_treats_exact_zero_as_dead(v, expected):
  assert I._k(v) == expected


@pytest.mark.parametrize(("v", "expected"), [(1.2, 1.2), (-1.2, -1.2), (0.0, None), (None, None)])
def test_signed(v, expected):
  assert I._signed(v) == expected


# ---------------------------------------------------------------------------------- normalisation

@pytest.mark.parametrize("missing", ["t", "lat", "lon", "vEgo"])
def test_normalise_needs_a_timestamp_a_fix_and_a_speed(missing):
  r = raw(0)
  del r[missing]
  assert I.normalise(r, "s") is None


@pytest.mark.parametrize(("lat", "lon"), [(91.0, -122.0), (-91.0, -122.0), (45.0, 181.0)])
def test_normalise_rejects_a_fix_off_the_planet(lat, lon):
  assert I.normalise(raw(0, lat=lat, lon=lon), "s") is None


def test_normalise_falls_back_to_heading_then_gives_up():
  r = raw(0)
  del r["bearing"]
  r["heading"] = 123.0
  assert I.normalise(r, "s").bearing == 123.0
  del r["heading"]
  assert I.normalise(r, "s").bearing is None


def test_normalise_wraps_a_bearing_into_range():
  assert I.normalise(raw(0, bearing=370.0), "s").bearing == pytest.approx(10.0)
  assert I.normalise(raw(0, bearing=-10.0), "s").bearing == pytest.approx(350.0)


def test_normalise_reads_the_logged_roll_up_in_preference_to_the_sampled_flags():
  # The logged dq is the 100 Hz OR; the sampled flags are one instant. Where both exist the logged
  # one wins, INCLUDING when the sample happens to look clean.
  tk = I.normalise(raw(0, dq=True, dqWhy="sat,blnk", strPrs=False), "s")
  assert tk.dq_bits == I.DQ_SAT | I.DQ_BLNK
  assert tk.dq_known


def test_normalise_marks_an_unattributed_disqualifier():
  tk = I.normalise(raw(0, dq=True), "s")
  assert tk.dq_bits == I.DQ_UNATTRIBUTED
  # and it survives every relaxation mask a user can ask for
  assert tk.dq_bits & I.dq_mask_from_names("sat,drv,lc,blnk")
  assert tk.dq_bits & I.dq_mask_from_names("")


def test_normalise_builds_bits_from_the_sampled_flags_when_there_is_no_roll_up():
  tk = I.normalise(raw(0, strPrs=True, slAngSat=True, lcGate="lanechange"), "s")
  assert tk.dq_bits == I.DQ_DRV | I.DQ_SAT | I.DQ_LC
  assert tk.dq_known


def test_normalise_reports_the_disqualifier_as_unknown_when_no_flag_exists():
  r = raw(0)
  for f in ("strPrs", "blnk", "slAngSat", "slSat", "slCurvLim", "lcGate"):
    del r[f]
  tk = I.normalise(r, "s")
  assert tk.dq_bits == 0
  assert tk.dq_known is False      # NOT "clean" -- absence of evidence is not evidence of absence


def test_normalise_does_not_confuse_a_non_lanechange_gate_with_one():
  assert I.normalise(raw(0, lcGate="lowconf"), "s").dq_bits == 0


# ----------------------------------------------------------------------------------- the dq names

def test_dq_names_round_trips():
  assert I.dq_names(I.DQ_SAT | I.DQ_BLNK) == "sat,blnk"
  assert I.dq_names(0) == ""


def test_dq_mask_from_names_rejects_an_unknown_flag():
  with pytest.raises(ValueError):
    I.dq_mask_from_names("sat,wipers")


def test_dq_mask_always_keeps_the_unattributed_bit():
  assert I.dq_mask_from_names("drv") == I.DQ_DRV | I.DQ_UNATTRIBUTED


# --------------------------------------------------------------------------------------- pose sign

def _pose(records):
  p = I._PoseSign()
  for r in records:
    p.observe(r)
  return p.verdict()


def test_pose_sign_absent_when_kpose_is_not_in_the_corpus():
  assert _pose([raw(i) for i in range(40)])[0] == "absent"


def test_pose_sign_normal_against_the_can_witness():
  recs = [raw(i, kPose=0.004, slKActl=0.004) for i in range(40)]
  assert _pose(recs)[0] == "normal"


def test_pose_sign_detects_the_2026_09_17_inversion():
  recs = [raw(i, kPose=-0.004, slKActl=0.004) for i in range(40)]
  verdict, detail = _pose(recs)
  assert verdict == "INVERTED"
  assert "36f914a17c" in detail


def test_pose_sign_uses_the_steering_wheel_when_the_can_source_is_dead():
  # The Tesla's only witness: slKActl reads exactly 0.0, so only strAng can speak.
  recs = [raw(i, car=TESLA, kPose=-0.004, slKActl=0.0, strAng=20.0) for i in range(40)]
  assert _pose(recs)[0] == "INVERTED"
  recs = [raw(i, car=TESLA, kPose=0.004, slKActl=0.0, strAng=20.0) for i in range(40)]
  assert _pose(recs)[0] == "normal"


def test_pose_sign_reports_ambiguity_rather_than_guessing():
  recs = ([raw(i, kPose=0.004, slKActl=0.004) for i in range(20)]
          + [raw(i, kPose=-0.004, slKActl=0.004) for i in range(20)])
  assert _pose(recs)[0] == "ambiguous"


def test_pose_sign_says_no_witness_rather_than_passing_vacuously():
  verdict, detail = _pose([raw(i, kPose=0.004) for i in range(40)])
  assert verdict == "no witness"
  assert "40" in detail


def test_pose_sign_ignores_a_small_steering_angle():
  # Near centre the sign of the wheel is noise; a witness must be unambiguous to be a witness.
  assert _pose([raw(i, car=TESLA, kPose=-0.004, strAng=1.0) for i in range(40)])[0] == "no witness"


# ------------------------------------------------------------------------------------ load_corpus

def write(tmp_path, name, records, gz=False):
  p = tmp_path / name
  data = "".join(json.dumps(r) + "\n" for r in records)
  if gz:
    with gzip.open(p, "wt") as f:
      f.write(data)
  else:
    p.write_text(data)
  return str(p)


def test_load_corpus_deduplicates_the_same_pass_filed_twice(tmp_path):
  recs = [raw(i) for i in range(10)]
  a = write(tmp_path, "a.jsonl", recs)
  b = write(tmp_path, "b.jsonl.gz", recs, gz=True)
  ticks, reports = I.load_corpus([a, b], log=lambda *x: None)
  assert len(ticks) == 10
  assert sum(r.duplicates for r in reports) == 10


def test_load_corpus_does_not_confuse_two_cars_at_the_same_instant(tmp_path):
  path = write(tmp_path, "a.jsonl", [raw(0, car=LIGHTNING), raw(0, car=TESLA)])
  ticks, _ = I.load_corpus([path], log=lambda *x: None)
  assert len(ticks) == 2


def test_load_corpus_drops_dead_rtc_records_and_counts_them(tmp_path):
  recs = [raw(i) for i in range(40)]
  recs[0]["t"] = T0 - 300 * 24 * 3600.0          # a cold boot with a stale clock
  path = write(tmp_path, "a.jsonl", recs)
  ticks, reports = I.load_corpus([path], log=lambda *x: None)
  assert len(ticks) == 39
  assert reports[0].clock_dropped == 1


def test_load_corpus_counts_an_unparsable_line_instead_of_dying(tmp_path):
  p = tmp_path / "a.jsonl"
  p.write_text(json.dumps(raw(0)) + "\n{ this is not json\n" + json.dumps(raw(1)) + "\n")
  ticks, reports = I.load_corpus([str(p)], log=lambda *x: None)
  assert len(ticks) == 2
  assert reports[0].unparsable == 1


def test_load_corpus_ignores_non_tick_events(tmp_path):
  path = write(tmp_path, "a.jsonl", [raw(0), {"ev": "pscmLim", "t": T0}, raw(1)])
  ticks, reports = I.load_corpus([path], log=lambda *x: None)
  assert len(ticks) == 2
  assert reports[0].ticks == 2


def test_load_corpus_reports_a_file_that_contributes_nothing(tmp_path):
  path = write(tmp_path, "a.jsonl", [{"ev": "alert", "t": T0}])
  ticks, reports = I.load_corpus([path], log=lambda *x: None)
  assert ticks == []
  assert reports[0].kept == 0


def test_load_corpus_counts_a_record_with_no_fix(tmp_path):
  r = raw(0)
  del r["lat"]
  path = write(tmp_path, "a.jsonl", [r, raw(1)])
  _, reports = I.load_corpus([path], log=lambda *x: None)
  assert reports[0].no_fix == 1


def test_load_corpus_separates_present_from_alive(tmp_path):
  # visK's failure: present on every record and always zero. Presence is not aliveness.
  path = write(tmp_path, "a.jsonl", [raw(i, slKActl=0.0, slKCmd=0.002) for i in range(5)])
  _, reports = I.load_corpus([path], log=lambda *x: None)
  assert reports[0].fields_present["slKActl"] == 5
  assert reports[0].fields_alive["slKActl"] == 0
  assert reports[0].fields_alive["slKCmd"] == 5


# ----------------------------------------------------------------------------------------- drives

def test_pt_date_is_pacific_not_utc():
  # 1789516800 is 2026-09-16 00:00 UTC and 2026-09-15 17:00 PT. Filing it under the UTC date would
  # put a late-evening drive in tomorrow's leave-one-date-out fold.
  assert I.pt_date(1789516800.0) == "2026-09-15"
  assert I.pt_date(1789549140.0) == "2026-09-16"


def test_split_drives_breaks_on_a_long_gap():
  ticks = [I.normalise(raw(i), "s") for i in range(10)]
  ticks += [I.normalise(raw(i, t0=T0 + 10 + I.DRIVE_GAP_S + 1), "s") for i in range(10)]
  assert len(I.split_drives(ticks)) == 2


def test_split_drives_does_not_break_on_a_short_stop():
  ticks = [I.normalise(raw(i), "s") for i in range(10)]
  ticks += [I.normalise(raw(i, t0=T0 + 10 + I.DRIVE_GAP_S - 1), "s") for i in range(10)]
  assert len(I.split_drives(ticks)) == 1


def test_split_drives_breaks_when_the_device_moves_to_the_other_car():
  ticks = [I.normalise(raw(i), "s") for i in range(10)]
  ticks += [I.normalise(raw(i, car=TESLA, t0=T0 + 11), "s") for i in range(10)]
  assert {d.car for d in I.split_drives(ticks)} == {LIGHTNING, TESLA}


def test_split_drives_discards_a_run_too_short_to_measure():
  assert I.split_drives([I.normalise(raw(0), "s")]) == []


def test_odometer_is_metres_and_agrees_with_the_gps_track():
  d = drive_of([raw(i) for i in range(20)])
  assert d.s[1] == pytest.approx(25.0, rel=1e-6)
  assert d.s[-1] == pytest.approx(25.0 * 19, rel=1e-6)
  assert d.gps_vs_odo == pytest.approx(1.0, rel=1e-3)


def test_odometer_disagreement_is_visible_when_the_speed_is_in_the_wrong_unit():
  # vEgo logged in mph while the fix moves in metres: the ratio is the only witness of that, and
  # the whole point of computing both. The POSITIONS stay on a 25 m/s track; only the speed lies.
  recs = [dict(raw(i), vEgo=25.0 * 2.237) for i in range(20)]
  assert drive_of(recs).gps_vs_odo < 0.5


def test_a_missing_second_does_not_integrate_a_phantom_kilometre():
  # A 100 s hole inside one drive (still under DRIVE_GAP_S) must not add 2.5 km of odometer.
  recs = [raw(0), dict(raw(1), t=T0 + 100.0)]
  d = drive_of(recs)
  assert d.s[-1] == pytest.approx(25.0 * I.MAX_TICK_DT_S)


def test_index_at_picks_the_nearer_tick():
  d = drive_of([raw(i) for i in range(20)])
  assert d.index_at(0.0) == 0
  assert d.index_at(49.0) == 2          # 50 m is tick 2; 49 is nearer to it than to tick 1
  assert d.index_at(1e9) == 19


# ------------------------------------------------------------------------------------------ sites

def test_reconstruct_site_lands_where_the_truck_later_was():
  d = drive_of([raw(i) for i in range(40)])
  pt = I._reconstruct_site(d, 0, 250.0)
  assert pt == pytest.approx((d.ticks[10].lat, d.ticks[10].lon))


def test_reconstruct_site_interpolates_between_ticks():
  d = drive_of([raw(i) for i in range(40)])
  lat, _ = I._reconstruct_site(d, 0, 262.5)
  assert lat == pytest.approx((d.ticks[10].lat + d.ticks[11].lat) / 2.0, rel=1e-9)


def test_reconstruct_site_returns_none_past_the_end_of_the_drive():
  d = drive_of([raw(i) for i in range(10)])
  assert I._reconstruct_site(d, 0, 5000.0) is None


def test_find_sites_prefers_the_logged_candidate_over_a_reconstruction():
  recs = [raw(i) for i in range(40)]
  from collections import Counter
  recs[0].update(mapLat=45.5, mapLon=-122.5, mapDist=250.0)
  sites = I.find_sites(drive_of(recs), P, Counter())
  assert (sites[0].lat, sites[0].src) == (45.5, "logged")


def test_find_sites_deduplicates_at_the_matcher_radius():
  from collections import Counter
  recs = [raw(i) for i in range(40)]
  for i in range(5):
    recs[i]["mapDist"] = 250.0 - 25.0 * i        # the same point, named five times as it nears
  sites = I.find_sites(drive_of(recs), P, Counter())
  assert len(sites) == 1


def test_find_sites_ignores_a_parked_tick():
  # A LOGGED candidate, so the parked tick would otherwise make a site with no reconstruction
  # needed -- and the rest of the drive moves, so this cannot pass for the wrong reason.
  from collections import Counter
  recs = [raw(i) for i in range(40)]
  recs[5].update(vEgo=1.0, mapLat=46.0, mapLon=-123.0)
  recs[20].update(mapLat=45.5, mapLon=-122.5)
  sites = I.find_sites(drive_of(recs), P, Counter())
  assert [(s.lat, s.lon) for s in sites] == [(45.5, -122.5)]


def test_find_sites_counts_a_candidate_the_drive_never_reached():
  from collections import Counter
  stats = Counter()
  recs = [raw(i) for i in range(10)]
  recs[0]["mapDist"] = 5000.0
  assert I.find_sites(drive_of(recs), P, stats) == []
  assert stats["site_reconstruct_past_end_of_drive"] == 1


# --------------------------------------------------------------------------------------- passages

def curve_drive(**over):
  """40 ticks north at 25 m/s with a bend at tick 20 (s = 500 m) and a map candidate naming it."""
  recs = [raw(i, **over) for i in range(40)]
  for i in range(19, 23):
    recs[i]["slKActl"] = 0.005
  recs[8]["mapDist"] = 300.0                      # tick 8 is s=200; +300 => s=500 => tick 20
  return recs


def one_passage(recs, dq_mask=I.DQ_ALL):
  from collections import Counter
  d = drive_of(recs)
  stats = Counter()
  sites = I.find_sites(d, P, stats)
  assert len(sites) == 1, sites
  return d, I.measure_passage(d, sites[0], P, stats, dq_mask), stats


def test_measure_passage_finds_the_bend_and_its_curvature():
  d, p, _ = one_passage(curve_drive())
  assert p is not None
  assert p.i_passage == 20
  assert p.k == 0.005
  assert p.k_v_ego == 25.0
  assert p.k_estimator == "sample1hz_actl"


def test_measure_passage_extent_is_the_designed_window_not_the_whole_drive():
  # Section 6.3: [candidate - 25 m, candidate + 150 m]. At 25 m/s that is tick 19 through tick 26.
  d, p, _ = one_passage(curve_drive())
  assert p.extent == (19, 27)
  assert p.n_ticks == 8


def test_measure_passage_takes_the_peak_of_the_extent_not_its_floor():
  # D5, and the owner's own instinct: adjust to the top, never average. Three different curvatures
  # inside one extent, so a min/first/last estimator cannot pass by coincidence.
  recs = curve_drive()
  recs[19]["slKActl"] = 0.002
  recs[20]["slKActl"] = 0.006
  recs[21]["slKActl"] = 0.003
  _, p, _ = one_passage(recs)
  assert p.k == 0.006


def test_measure_passage_ignores_a_spike_outside_the_extent():
  recs = curve_drive()
  recs[35]["slKActl"] = 0.05                      # 375 m past the site: a different curve
  _, p, _ = one_passage(recs)
  assert p.k == 0.005


def test_measure_passage_takes_the_max_of_commanded_and_achieved():
  # Section 3.3 / D2: achieved is bounded by steering authority and under-reads where it matters.
  recs = curve_drive()
  for i in range(19, 23):
    recs[i]["slKCmd"] = 0.007
  _, p, _ = one_passage(recs)
  assert p.k == 0.007
  assert p.k_estimator == "sample1hz_cmd_actl"


def test_measure_passage_prefers_kpeak_where_it_exists():
  recs = curve_drive()
  for i in range(19, 23):
    recs[i]["kPeak"] = 0.009
  _, p, _ = one_passage(recs)
  assert p.k == 0.009
  assert p.k_estimator == "kPeak100"


def test_measure_passage_names_a_window_by_its_weakest_estimator():
  recs = curve_drive()
  recs[20]["kPeak"] = 0.009                        # one 100 Hz tick among 1 Hz samples
  _, p, _ = one_passage(recs)
  assert p.k_estimator == "sample1hz_actl_mixed"


def test_measure_passage_samples_the_approach_bearing_at_the_design_distance():
  recs = curve_drive()
  for i, r in enumerate(recs):
    r["bearing"] = 111.0 if i == 8 else 222.0      # tick 8 is 300 m before the site
  _, p, _ = one_passage(recs)
  assert p.approach_bearing == 111.0
  assert p.bearing_src == "logged"


def test_measure_passage_derives_the_bearing_from_the_track_when_none_is_logged():
  recs = curve_drive()
  for r in recs:
    del r["bearing"]
  _, p, _ = one_passage(recs)
  assert p.bearing_src == "track"
  assert p.approach_bearing == pytest.approx(0.0, abs=0.5)     # the drive is due north


def test_measure_passage_refuses_a_pass_with_no_recorded_approach():
  from collections import Counter
  recs = [raw(i) for i in range(8)]               # 175 m of drive; the 300 m approach never happened
  recs[0]["mapDist"] = 150.0
  d = drive_of(recs)
  stats = Counter()
  sites = I.find_sites(d, P, stats)
  assert I.measure_passage(d, sites[0], P, stats) is None
  assert stats["pass_no_approach_bearing"] == 1


def test_measure_passage_refuses_a_site_the_truck_never_reached():
  from collections import Counter
  d = drive_of([raw(i) for i in range(40)])
  site = I.Site(lat=46.0, lon=-122.0, src="logged", first_i=0)
  stats = Counter()
  assert I.measure_passage(d, site, P, stats) is None
  assert stats["pass_never_reached_site"] == 1


def test_measure_passage_reports_no_curvature_rather_than_zero():
  from collections import Counter
  recs = [raw(i) for i in range(40)]              # no slKActl/slKCmd at all: a June corpus
  recs[8]["mapDist"] = 300.0
  d = drive_of(recs)
  stats = Counter()
  assert I.measure_passage(d, I.find_sites(d, P, stats)[0], P, stats) is None
  assert stats["pass_no_curvature_anywhere_in_extent"] == 1


@pytest.mark.parametrize(("flag", "value", "why"), [
  ("strPrs", True, "drv"), ("blnk", True, "blnk"), ("slAngSat", True, "sat"),
  ("slSat", True, "sat"), ("slCurvLim", True, "sat"), ("lcGate", "lanechange", "lc"),
])
def test_measure_passage_disqualifies_on_every_section_35_cause(flag, value, why):
  recs = curve_drive()
  recs[21][flag] = value
  _, p, _ = one_passage(recs)
  assert p.dq_state == "dirty"
  assert why in p.dq_why


def test_a_disqualifier_outside_the_extent_does_not_taint_the_pass():
  recs = curve_drive()
  recs[35]["strPrs"] = True
  _, p, _ = one_passage(recs)
  assert p.dq_state == "clean"


def test_a_relaxed_mask_still_names_the_cause_it_ignored():
  recs = curve_drive()
  recs[21]["slAngSat"] = True
  _, p, _ = one_passage(recs, dq_mask=I.dq_mask_from_names("drv,lc"))
  assert p.dq_state == "clean"
  assert "sat" in p.dq_why           # relaxed, not forgotten


def test_a_corpus_with_no_flags_reports_unknown_not_clean():
  recs = curve_drive()
  for r in recs:
    for f in ("strPrs", "blnk", "slAngSat", "slSat", "slCurvLim", "lcGate"):
      r.pop(f, None)
  _, p, _ = one_passage(recs)
  assert p.dq_state == "unknown"


def test_lateral_accel_is_measured_hands_off_only():
  recs = curve_drive()
  for r in recs:
    r["achLat"] = 0.2
  recs[20]["achLat"] = 4.0
  recs[20]["strPrs"] = True         # the driver's own steering, not the road
  recs[21]["achLat"] = 1.0
  _, p, _ = one_passage(recs, dq_mask=I.dq_mask_from_names("lc,blnk"))
  assert p.a_lat_src == "achLat"
  assert p.a_lat_max == 1.0


def test_lateral_accel_falls_back_to_k_times_v_squared_and_says_so():
  d, p, _ = one_passage(curve_drive())
  assert p.a_lat_src == "k*v^2"
  assert p.a_lat_max == pytest.approx(0.005 * 25.0 * 25.0)


# ----------------------------------------------------------------------------------- observations

def observations(recs, dq_mask=I.DQ_ALL):
  from collections import Counter
  d = drive_of(recs)
  stats = Counter()
  sites = I.find_sites(d, P, stats)
  ps = [p for p in (I.measure_passage(d, s, P, stats, dq_mask) for s in sites) if p is not None]
  return I.observations_for_drive(d, ps, P, stats), stats


def test_an_admissible_pass_becomes_one_up_observation():
  obs, _ = observations(curve_drive())
  assert len(obs) == 1
  o = obs[0]
  assert isinstance(o, Observation)
  assert (o.kind, o.k, o.dq_state, o.site_src) == ("up", 0.005, "clean", "track")
  assert o.date == "2026-09-09"          # PT, not the 2026-09-10 the UTC clock would give


def test_a_disqualified_pass_is_dropped_and_its_cause_counted():
  recs = curve_drive()
  recs[21]["slAngSat"] = True
  obs, stats = observations(recs)
  assert obs == []
  assert stats["obs_dropped_disqualified"] == 1
  assert stats["obs_dropped_dq_sat"] == 1


def test_a_straight_road_does_not_become_a_row():
  recs = curve_drive()
  for i in range(19, 23):
    recs[i]["slKActl"] = 1e-6
  obs, stats = observations(recs)
  assert obs == []
  assert stats["obs_k_below_usable_floor"] == 1


def test_a_driver_override_under_real_lateral_load_becomes_a_down_observation():
  recs = curve_drive()
  recs[20].update(strPrs=True, achLat=3.0, slKCmd=0.008)
  obs, _ = observations(recs, dq_mask=I.dq_mask_from_names("lc,blnk"))
  down = [o for o in obs if o.kind == "down"]
  assert len(down) == 1
  assert down[0].k == 0.008                     # section 6.2's |slKCmd| at the intervention
  assert down[0].estimator == "slKCmd_at_override"


def test_a_gentle_override_is_not_a_down_observation():
  # Section 6.2: of 2,986 overrides above 27 mph only ~113 ticks were plausibly about curve speed.
  recs = curve_drive()
  recs[20].update(strPrs=True, achLat=0.9, slKCmd=0.008)
  obs, _ = observations(recs, dq_mask=I.dq_mask_from_names("lc,blnk"))
  assert [o for o in obs if o.kind == "down"] == []


def test_an_override_with_no_commanded_curvature_is_counted_not_guessed():
  recs = curve_drive()
  recs[20].update(strPrs=True, achLat=3.0)
  obs, stats = observations(recs, dq_mask=I.dq_mask_from_names("lc,blnk"))
  assert [o for o in obs if o.kind == "down"] == []
  assert stats["down_dropped_no_kcmd"] == 1


def test_only_one_down_per_pass():
  recs = curve_drive()
  for i in (20, 21, 22):
    recs[i].update(strPrs=True, achLat=3.0, slKCmd=0.008)
  obs, _ = observations(recs, dq_mask=I.dq_mask_from_names("lc,blnk"))
  assert len([o for o in obs if o.kind == "down"]) == 1


# --------------------------------------------------------------------------------------- episodes

def episodes(recs):
  from collections import Counter
  d = drive_of(recs)
  stats = Counter()
  sites = I.find_sites(d, P, stats)
  ps = [p for p in (I.measure_passage(d, s, P, stats) for s in sites) if p is not None]
  return I.find_episodes(d, ps, P, stats), stats


def icbm_drive(**over):
  recs = curve_drive(**over)
  for i in range(8, 16):
    recs[i].update(icbmT=18.0, icbmSrc="map", icbmC=28.0)
  return recs


def test_an_icbm_slowdown_becomes_one_episode():
  eps, _ = episodes(icbm_drive())
  assert len(eps) == 1
  e = eps[0]
  assert e["icbm_target_ms"] == 18.0
  assert (e["ref_ms"], e["ref_src"]) == (28.0, "icbmC")
  assert e["k_truth"] == 0.005
  assert e["date"] == "2026-09-09"


def test_a_restore_only_run_is_not_a_slowdown():
  recs = curve_drive()
  for i in range(8, 16):
    recs[i].update(icbmT=26.0, icbmSrc="restore")
  eps, stats = episodes(recs)
  assert eps == []
  assert stats["ep_restore_only"] == 1


def test_the_reference_speed_falls_back_to_the_pre_episode_set():
  # vSet DURING an episode is the slowdown itself -- ICBM taps the SET button down -- so the
  # fallback must look BEFORE the episode or the reduction measures itself as zero.
  recs = curve_drive()
  for i in range(8):
    recs[i]["vSet"] = 28.0
  for i in range(8, 16):
    recs[i].update(icbmT=18.0, icbmSrc="map", vSet=18.0)
  eps, _ = episodes(recs)
  assert (eps[0]["ref_ms"], eps[0]["ref_src"]) == (28.0, "vSet_prewindow")


def test_an_episode_with_no_reference_speed_is_counted_not_invented():
  recs = curve_drive()
  for i in range(8, 16):
    recs[i].update(icbmT=18.0, icbmSrc="map")
  eps, stats = episodes(recs)
  assert eps == []
  assert stats["ep_dropped_no_reference_speed"] == 1


def test_a_rounding_sized_reduction_is_not_a_slowdown():
  recs = curve_drive()
  for i in range(8, 16):
    recs[i].update(icbmT=27.9, icbmSrc="map", icbmC=28.0)
  eps, stats = episodes(recs)
  assert eps == []
  assert stats["ep_dropped_reduction_too_small"] == 1


def test_an_episode_is_never_attributed_to_a_curve_already_driven_past():
  # behindgate2pnw's failure mode, as a test: the site is at tick 20 and the decision at tick 30.
  recs = curve_drive()
  for i in range(30, 36):
    recs[i].update(icbmT=18.0, icbmSrc="map", icbmC=28.0)
  eps, stats = episodes(recs)
  assert eps == []
  assert stats["ep_dropped_no_site_ahead"] == 1


def test_the_episode_site_uses_icbms_own_logged_candidate_when_there_is_one():
  recs = icbm_drive()
  d = drive_of(recs)
  target = (d.ticks[20].lat, d.ticks[20].lon)
  for i in range(8, 16):
    recs[i].update(mapLat=target[0], mapLon=target[1])
  eps, _ = episodes(recs)
  assert eps[0]["site_attrib"] == "logged_candidate"


def test_the_episode_site_falls_back_to_the_logged_distance():
  eps, _ = episodes(icbm_drive())
  assert eps[0]["site_attrib"] in ("map_dist", "nearest_ahead")


def test_the_episode_carries_the_speed_its_curvature_was_measured_at():
  # k = 0.14 at 6 m/s is an intersection; the replay cannot tell without this number.
  eps, _ = episodes(icbm_drive())
  assert eps[0]["k_v_ego"] == 25.0


def test_two_separated_slowdowns_are_two_episodes():
  recs = [raw(i) for i in range(80)]
  for i in range(19, 23):
    recs[i]["slKActl"] = 0.005
  for i in range(59, 63):
    recs[i]["slKActl"] = 0.005
  recs[8]["mapDist"] = 300.0
  recs[48]["mapDist"] = 300.0
  for i in range(8, 14):
    recs[i].update(icbmT=18.0, icbmSrc="map", icbmC=28.0)
  for i in range(48, 54):
    recs[i].update(icbmT=18.0, icbmSrc="map", icbmC=28.0)
  eps, _ = episodes(recs)
  assert len(eps) == 2


# ------------------------------------------------------------------------------------------- i/o

def test_load_observations_rejects_a_line_that_is_not_an_observation(tmp_path):
  p = tmp_path / "obs.jsonl"
  p.write_text('{"date": "2026-09-01"}\n')
  with pytest.raises(ValueError):
    I.load_observations(str(p))


def test_observations_round_trip_through_jsonl(tmp_path):
  import dataclasses
  obs, _ = observations(curve_drive())
  p = tmp_path / "obs.jsonl"
  p.write_text("".join(json.dumps(dataclasses.asdict(o)) + "\n" for o in obs))
  assert I.load_observations(str(p)) == obs


def test_main_reports_an_empty_result_as_a_failure(tmp_path, capsys):
  path = write(tmp_path, "a.jsonl", [raw(i) for i in range(40)])   # no curvature anywhere
  rc = I.main([path, "--out-observations", str(tmp_path / "o.jsonl"),
               "--out-episodes", str(tmp_path / "e.jsonl")])
  assert rc == 1
  assert "ZERO OBSERVATIONS" in capsys.readouterr().out


def test_main_rejects_an_unknown_disqualifier_name(tmp_path):
  path = write(tmp_path, "a.jsonl", [raw(0)])
  with pytest.raises(ValueError):
    I.main([path, "--dq-flags", "wipers", "--out-observations", str(tmp_path / "o.jsonl"),
            "--out-episodes", str(tmp_path / "e.jsonl")])


def test_provisional_ingest_constants_are_the_documented_values():
  assert I.PASSAGE_MAX_M == 80.0
  assert I.EPISODE_LOOKAHEAD_M == 500.0
  assert I.DRIVE_GAP_S == 300.0
  assert I.K_MIN_USABLE == 1e-4
  assert tighten(P, site_radius_m=40.0).site_radius_m == 40.0


# ------------------------------------------------- the 2026-09-17 review round (Fable findings)

def test_dedup_does_not_confuse_an_adopt_record_with_the_tick_at_the_same_instant(tmp_path):
  # 3 of 51 adopt records in a 6,000-line sample share a timestamp with a tick, and the ADOPT
  # record is the one carrying icbmT/icbmSrc/mapLat at the decision instant. Keyed on (car, t)
  # alone, one of the two was silently counted as a duplicate of the other.
  tick = raw(0)
  adopt = dict(raw(0), ev="adopt", icbmT=18.0, icbmSrc="map")
  path = write(tmp_path, "a.jsonl", [tick, adopt])
  ticks, reports = I.load_corpus([path], log=lambda *x: None)
  assert len(ticks) == 2
  assert reports[0].duplicates == 0
  assert {t.ev for t in ticks} == {"tick", "adopt"}


def test_dedup_still_catches_a_genuinely_duplicated_record(tmp_path):
  a = write(tmp_path, "a.jsonl", [raw(0), raw(1)])
  b = write(tmp_path, "b.jsonl", [raw(0), raw(1)])
  ticks, reports = I.load_corpus([a, b], log=lambda *x: None)
  assert len(ticks) == 2
  assert sum(r.duplicates for r in reports) == 2


def test_a_drive_id_is_stable_and_distinguishes_ignition_cycles():
  ticks = [I.normalise(raw(i), "s") for i in range(10)]
  ticks += [I.normalise(raw(i, t0=T0 + 10 + I.DRIVE_GAP_S + 1), "s") for i in range(10)]
  ids = [d.drive_id for d in I.split_drives(ticks)]
  assert len(set(ids)) == 2
  assert ids == [d.drive_id for d in I.split_drives(ticks)]     # stable across re-ingests
  assert all(LIGHTNING in i for i in ids)


def test_observations_carry_their_drive_id():
  obs, _ = observations(curve_drive())
  assert obs[0].drive_id
  assert obs[0].drive_id.startswith(LIGHTNING)


def test_dq_provenance_says_whether_the_state_came_from_the_roll_up_or_a_sample():
  # Section 3.5 exists because a 1 Hz sample "may have been between events at the sample instant",
  # so a `clean` from a sample is weaker than a `clean` from the 100 Hz OR. Recording which is the
  # same discipline `estimator` applies to curvature.
  obs, _ = observations(curve_drive())
  assert obs[0].dq_src == "sampled1hz"

  recs = curve_drive()
  for r in recs:
    r["dq"] = False
  obs2, _ = observations(recs)
  assert obs2[0].dq_src == "rollup100"

  recs = curve_drive()
  for r in recs:
    for f in ("strPrs", "blnk", "slAngSat", "slSat", "slCurvLim", "lcGate"):
      r.pop(f, None)
  obs3, _ = observations(recs)
  assert (obs3[0].dq_state, obs3[0].dq_src) == ("unknown", "none")


def test_the_lookahead_curvature_ignores_a_parking_lot_turn():
  # I7 judges a cancel on max(k_truth, k_ahead_max). Without the same speed floor the extent uses,
  # a 7 m-radius junction taken at 2 m/s enters the verdict and manufactures a false cancel.
  recs = icbm_drive()
  # tick 25 is s=625 m, inside the 500 m window measured from the decision at tick 8 (s=200).
  # Tick 30 would be at ~725 m and OUTSIDE it -- which is how this test first passed vacuously.
  recs[25].update(vEgo=2.0, slKActl=0.14)
  eps, _ = episodes(recs)
  assert eps[0]["k_truth"] == 0.005
  assert eps[0]["k_ahead_max"] == 0.005
  assert eps[0]["k_ahead_v_ego"] >= P.min_speed_ms


def test_a_drive_whose_odometer_disagrees_with_gps_contributes_nothing(monkeypatch):
  """Loud AND dropped. Its extents, approach reference and lookahead are all measured against a
  distance the two sources disagree about, and downstream those rows look like any other.

  The control half matters as much as the assertion: with the band opened up, this same drive DOES
  produce a row -- so the empty result below is the drop doing its job, not the drive being
  unusable for some other reason."""
  # vEgo in mph while the fix moves in metres: every distance-derived field scales with it.
  bad = [dict(r, vEgo=r["vEgo"] * 2.237) for r in icbm_drive()]
  for r in bad:
    if r.get("mapDist"):
      r["mapDist"] *= 2.237

  o1, e1, _, s1, _ = I.ingest_records(icbm_drive())
  assert o1 and e1 and s1["drive_dropped_odometer_disagrees_with_gps"] == 0

  o2, e2, _, s2, _ = I.ingest_records(bad)
  assert (o2, e2) == ([], [])
  assert s2["drive_dropped_odometer_disagrees_with_gps"] == 1

  monkeypatch.setattr(I, "ODO_GPS_RATIO_BAND", (0.0, 99.0))
  o3, _, _, s3, _ = I.ingest_records(bad)
  assert o3, ("the control failed: this drive yields nothing even WITHOUT the drop, so the test " +
              "above would pass with the drop deleted")
  assert s3["drive_dropped_odometer_disagrees_with_gps"] == 0


def test_the_tally_prints_a_reason_that_never_fired():
  # "Both counters are 0" must be READ, not inferred from their absence from the output.
  _, _, _, stats, _ = I.ingest_records(icbm_drive())
  for k in ("down_dropped_no_kcmd", "down_no_lateral_accel_witness", "obs_down",
            "ep_dropped_reduction_too_small", "drive_dropped_odometer_disagrees_with_gps"):
    assert k in stats
    assert stats[k] == 0
  assert set(I.TALLY_KEYS) <= set(stats)


# ------------------------------------------------------------------- recurrence (section 3.1)

def test_recurrence_separates_a_missing_revisit_from_a_discarded_one():
  """The distinction the whole recommendation turns on: did the truck not come back, or did it
  come back and our own rules bin the pass?"""
  from openpilot.tools.curvedb import recurrence as REC

  def pass_over(day, dirty=False, curvature=True):
    recs = []
    for i in range(40):
      r = raw(i, t0=T0 + 86400 * day)
      if curvature and 19 <= i <= 22:
        r["slKActl"] = 0.005
      if dirty and i == 20:
        r["strPrs"] = True
      recs.append(r)
    recs[8]["mapDist"] = 300.0
    return recs

  def run(days):
    recs = [r for d in days for r in pass_over(*d)]
    obs, eps, _, _, _ = I.ingest_records(recs)
    ticks = [t for t in (I.normalise(r, "m") for r in recs) if t is not None]
    d = drive_of(pass_over(0))
    ep = dict(site_lat=d.ticks[20].lat, site_lon=d.ticks[20].lon, approach_bearing=0.0,
              date=I.pt_date(T0))
    return REC.analyse(ticks, [ep], obs, P)

  # never came back
  tally, why, _ = run([(0,)])
  assert tally["revisited_same_direction"] == 0

  # came back on day 5, clean -> a row exists
  tally, why, _ = run([(0,), (5,)])
  assert tally["revisited_same_direction"] == 1
  assert why["a row exists"] == 1

  # came back on day 5, but the driver was steering -> the revisit is DISCARDED, not absent
  tally, why, _ = run([(0,), (5, True)])
  assert tally["revisited_same_direction"] == 1
  assert why["a row exists"] == 0
  assert why["disqualified: drv"] == 1


def test_recurrence_counts_only_revisits_in_the_same_direction():
  from openpilot.tools.curvedb import recurrence as REC
  recs = [raw(i) for i in range(40)]
  ticks = [t for t in (I.normalise(r, "m") for r in recs) if t is not None]
  d = drive_of(recs)
  here = dict(site_lat=d.ticks[20].lat, site_lon=d.ticks[20].lon, date="2000-01-01")
  # the drive is due north; an episode approached from the south matches, from the north does not
  assert REC.analyse(ticks, [dict(here, approach_bearing=0.0)], [], P)[0]["revisited_same_direction"] == 1
  assert REC.analyse(ticks, [dict(here, approach_bearing=180.0)], [], P)[0]["revisited_same_direction"] == 0


def test_recurrence_refuses_to_report_a_rate_from_an_empty_index():
  from openpilot.tools.curvedb import recurrence as REC
  parked = [t for t in (I.normalise(raw(i, v=1.0), "m") for i in range(40)) if t is not None]
  with pytest.raises(SystemExit):
    REC.analyse(parked, [dict(site_lat=45.0, site_lon=-122.0, approach_bearing=0.0,
                              date="2000-01-01")], [], P)
