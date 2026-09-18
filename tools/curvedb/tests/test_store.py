"""Unit tests for curvedb's C1 core -- geometry, rows, the matcher, authority, the cancel formula.

Every assertion here is an assertion about a DECISION, not about a shape: the values are chosen so
that flipping a comparison, dropping a gate, or swapping a `max` for a `min` changes a number this
file reads. That is the bar set by the mutation harness in `_scratch/curvedb/mutate.py`.
"""
import itertools
import math

import pytest

from openpilot.tools.curvedb.store import (
  PROVISIONAL_ENVELOPES,
  PROVISIONAL_PARAMS,
  CarEnvelope,
  CurveDB,
  CurveDBError,
  CurveRow,
  Observation,
  authority,
  bearing_diff_deg,
  cancel_target,
  haversine_m,
  initial_bearing_deg,
  load_snapshot_or_empty,
  speed_for_curvature,
  tighten,
  with_params,
)

LIGHTNING = "FORD_F_150_LIGHTNING_MK1"


_drive_seq = itertools.count()


def obs(**over):
  # Each call is a DIFFERENT drive unless the caller says otherwise, because `n_passes` counts
  # drives: two observations from one pass (an UP and a DOWN) must not read as two passes.
  base = dict(date="2026-09-01", t=1789000000.0, car=LIGHTNING,
              drive_id=f"drive-{next(_drive_seq)}", site_lat=45.0, site_lon=-122.0,
              bearing_deg=90.0, k=0.004, kind="up", estimator="kPeak100", site_src="logged",
              source="x.jsonl:1", posted_ms=29.0, highway_class="motorway", n_ticks=7,
              dq_state="clean", dq_src="rollup100")
  base.update(over)
  return Observation(**base)


# ---------------------------------------------------------------------------------------- geometry

def test_haversine_one_degree_of_latitude():
  # pi*R/180 with R = 6371 km is 111194.9 m. A wrong earth radius or a degrees/radians slip moves
  # this by more than the 10 m tolerance.
  assert haversine_m(45.0, -122.0, 46.0, -122.0) == pytest.approx(111194.9, abs=10.0)


def test_haversine_is_zero_for_the_same_point():
  assert haversine_m(45.0, -122.0, 45.0, -122.0) == 0.0


def test_haversine_shrinks_with_latitude_in_longitude():
  # One degree of longitude at 45 N is cos(45) of one at the equator. Catches a lat/lon swap.
  eq = haversine_m(0.0, 0.0, 0.0, 1.0)
  mid = haversine_m(45.0, 0.0, 45.0, 1.0)
  assert mid == pytest.approx(eq * math.cos(math.radians(45.0)), rel=1e-3)


@pytest.mark.parametrize(("dlat", "dlon", "expected"), [
  (0.01, 0.0, 0.0),        # due north
  (0.0, 0.01, 90.0),       # due east
  (-0.01, 0.0, 180.0),     # due south
  (0.0, -0.01, 270.0),     # due west
])
def test_initial_bearing_cardinals(dlat, dlon, expected):
  b = initial_bearing_deg(45.0, -122.0, 45.0 + dlat, -122.0 + dlon)
  assert b == pytest.approx(expected, abs=0.5)


def test_initial_bearing_is_always_in_range():
  assert 0.0 <= initial_bearing_deg(45.0, -122.0, 44.99, -122.01) < 360.0


@pytest.mark.parametrize(("a", "b", "expected"), [
  (0.0, 0.0, 0.0), (350.0, 10.0, 20.0), (10.0, 350.0, 20.0), (0.0, 180.0, 180.0),
  (0.0, 181.0, 179.0), (90.0, 271.0, 179.0),
])
def test_bearing_diff_wraps(a, b, expected):
  assert bearing_diff_deg(a, b) == pytest.approx(expected, abs=1e-9)


def test_bearing_diff_never_exceeds_180():
  for a in range(0, 360, 7):
    for b in range(0, 360, 11):
      assert 0.0 <= bearing_diff_deg(float(a), float(b)) <= 180.0


# ------------------------------------------------------------------------------- speed <- curvature

def test_speed_for_curvature_is_the_design_formula():
  # v = sqrt(a/k): 2.5 m/s^2 on k = 0.004 (R = 250 m) is 25 m/s.
  assert speed_for_curvature(2.5, 0.004) == pytest.approx(25.0, rel=1e-9)


@pytest.mark.parametrize(("a", "k"), [(2.5, 0.0), (2.5, -0.004), (0.0, 0.004), (-1.0, 0.004),
                                      (float("nan"), 0.004), (2.5, float("inf"))])
def test_speed_for_curvature_refuses_nonsense(a, k):
  with pytest.raises(CurveDBError):
    speed_for_curvature(a, k)


# ----------------------------------------------------------------------------------------- params

@pytest.mark.parametrize("field", ["extent_back_m", "extent_fwd_m", "approach_bearing_ref_m",
                                   "down_trigger_a_lat_ms2", "min_speed_ms", "site_radius_m",
                                   "heading_tol_deg", "a_lat_comfort_ms2"])
@pytest.mark.parametrize("bad", [0.0, -1.0, float("nan")])
def test_params_reject_non_positive(field, bad):
  with pytest.raises(CurveDBError):
    tighten(PROVISIONAL_PARAMS, **{field: bad})


@pytest.mark.parametrize("field", ["min_passes", "min_dates"])
def test_params_reject_counts_below_one(field):
  with pytest.raises(CurveDBError):
    tighten(PROVISIONAL_PARAMS, **{field: 0})


def test_params_reject_a_heading_tolerance_that_matches_everything():
  with pytest.raises(CurveDBError):
    tighten(PROVISIONAL_PARAMS, heading_tol_deg=180.0)


def test_provisional_params_are_the_documented_values():
  # These are quoted verbatim in the branch report; drift here silently invalidates it.
  p = PROVISIONAL_PARAMS
  assert (p.extent_back_m, p.extent_fwd_m) == (25.0, 150.0)
  assert p.min_passes == 2 and p.min_dates == 2
  assert p.a_lat_comfort_ms2 == 2.5
  assert "motorwayLink" in p.ramp_highway_classes
  assert "motorway" not in p.ramp_highway_classes


# ------------------------------------------------------------------------------------- envelopes

def test_car_envelope_requires_provenance():
  with pytest.raises(CurveDBError):
    CarEnvelope(platform=LIGHTNING, hard_ceiling_a_lat_ms2=4.5, provenance="")


@pytest.mark.parametrize(("platform", "ceiling"), [("", 4.5), (LIGHTNING, 0.0), (LIGHTNING, -1.0),
                                                   (LIGHTNING, float("nan"))])
def test_car_envelope_rejects_nonsense(platform, ceiling):
  with pytest.raises(CurveDBError):
    CarEnvelope(platform=platform, hard_ceiling_a_lat_ms2=ceiling, provenance="measured")


def test_only_the_lightning_has_a_measured_envelope():
  # The Raven's lateral ceiling has never been measured; section 8.2's CarSpecs fallback is the
  # mass-predicts-grip reasoning Fable rejected. An entry appearing here is a decision, not a typo.
  assert set(PROVISIONAL_ENVELOPES) == {LIGHTNING}


# ----------------------------------------------------------------------------------- observations

@pytest.mark.parametrize("bad", [
  {"k": 0.0}, {"k": -0.004}, {"k": float("nan")},
  {"kind": "sideways"}, {"dq_state": "dirty"}, {"dq_state": ""},
  {"dq_src": "guessed"}, {"dq_src": ""}, {"drive_id": ""},
  {"date": "2026-9-1"}, {"date": ""},
  {"estimator": ""}, {"site_src": ""},
  {"bearing_deg": 360.0}, {"bearing_deg": -1.0}, {"bearing_deg": float("inf")},
  {"site_lat": 91.0}, {"site_lon": 181.0},
])
def test_observation_rejects(bad):
  with pytest.raises(CurveDBError):
    obs(**bad)


def test_observation_accepts_the_boundary_bearings():
  assert obs(bearing_deg=0.0).bearing_deg == 0.0
  assert obs(bearing_deg=359.999).bearing_deg == 359.999


def test_observation_accepts_unknown_dq_state():
  assert obs(dq_state="unknown").dq_state == "unknown"


# ------------------------------------------------------------------------------------------- rows

def test_row_up_is_a_max_across_passes_not_a_mean():
  # Section 6.1: one genuinely tight pass must never be averaged away by gentle ones.
  r = CurveRow(45.0, -122.0, 90.0, [obs(k=0.001), obs(k=0.009), obs(k=0.002)])
  assert r.k_up == 0.009


def test_row_down_raises_k_and_therefore_lowers_the_derived_speed():
  r = CurveRow(45.0, -122.0, 90.0, [obs(k=0.002, kind="up"), obs(k=0.008, kind="down")])
  assert r.k_up == 0.002
  assert r.k_down == 0.008
  assert r.k_eff == 0.008
  assert speed_for_curvature(2.5, r.k_eff) < speed_for_curvature(2.5, r.k_up)


def test_row_down_keeps_the_firmest_intervention_not_the_gentlest():
  # Two drivers' "too fast" on one row: the one that asked for the most slowing must win, or a
  # single mild intervention on a later date quietly raises the speed the row permits.
  r = CurveRow(45.0, -122.0, 90.0, [obs(k=0.003, kind="down"),
                                    obs(k=0.009, kind="down", date="2026-09-02"),
                                    obs(k=0.005, kind="down", date="2026-09-03")])
  assert r.k_down == 0.009


def test_a_later_up_pass_can_never_undo_a_down():
  # D8: DOWN is never overwritten by UP. A gentler UP pass must not raise the derived speed.
  r = CurveRow(45.0, -122.0, 90.0, [obs(k=0.008, kind="down")])
  before = r.k_eff
  r.observations.append(obs(k=0.0005, kind="up", date="2026-09-02"))
  assert r.k_eff == before


def test_row_dates_and_cars_are_deduplicated_and_sorted():
  r = CurveRow(45.0, -122.0, 90.0, [obs(date="2026-09-02"), obs(date="2026-09-01"),
                                    obs(date="2026-09-02", car="TESLA_MODEL_S_HW3")])
  assert r.dates == ("2026-09-01", "2026-09-02")
  assert r.cars == ("FORD_F_150_LIGHTNING_MK1", "TESLA_MODEL_S_HW3")
  assert r.n_passes == 3
  assert r.n_observations == 3


def test_n_passes_counts_drives_not_observations():
  # An UP and a DOWN from the SAME pass are one pass. Counting observations would let a single
  # drive satisfy half of D6 on its own.
  one = CurveRow(45.0, -122.0, 90.0, [obs(drive_id="d1", kind="up"),
                                      obs(drive_id="d1", kind="down", k=0.009)])
  assert one.n_observations == 2
  assert one.n_passes == 1
  assert one.drives == ("d1",)


def test_row_with_no_observations_has_no_curvature():
  assert CurveRow(45.0, -122.0, 90.0, []).k_eff is None


# ---------------------------------------------------------------------------------------- matching

def east_of(lat, lon, metres):
  return lon + metres / (111320.0 * math.cos(math.radians(lat)))


def test_build_groups_two_passes_of_the_same_place():
  p = PROVISIONAL_PARAMS
  a = obs(date="2026-09-01")
  b = obs(date="2026-09-02", site_lon=east_of(45.0, -122.0, 20.0))
  db = CurveDB.build([a, b], p)
  assert len(db.rows) == 1
  assert db.rows[0].n_passes == 2


def test_build_separates_places_further_apart_than_the_radius():
  p = PROVISIONAL_PARAMS
  a = obs(date="2026-09-01")
  b = obs(date="2026-09-02", site_lon=east_of(45.0, -122.0, p.site_radius_m + 25.0))
  assert len(CurveDB.build([a, b], p).rows) == 2


def test_build_separates_opposite_directions_of_the_same_place():
  # The whole point of keying on (site, heading): northbound and southbound are different roads.
  db = CurveDB.build([obs(bearing_deg=90.0), obs(bearing_deg=270.0, date="2026-09-02")],
                     PROVISIONAL_PARAMS)
  assert len(db.rows) == 2


def test_heading_tolerance_is_a_continuous_angle_not_a_bucket():
  # D4: 45-degree buckets straddle 47 % of real curves. 350 and 10 degrees are 20 apart and must
  # match, even though they fall either side of every sensible bucket edge.
  db = CurveDB.build([obs(bearing_deg=350.0), obs(bearing_deg=10.0, date="2026-09-02")],
                     PROVISIONAL_PARAMS)
  assert len(db.rows) == 1


def test_match_respects_the_heading_tolerance_boundary():
  p = tighten(PROVISIONAL_PARAMS, heading_tol_deg=30.0)
  db = CurveDB.build([obs(bearing_deg=0.0)], p)
  assert db.match(45.0, -122.0, 29.0) is not None
  assert db.match(45.0, -122.0, 31.0) is None


def test_match_respects_the_radius_boundary():
  p = tighten(PROVISIONAL_PARAMS, site_radius_m=40.0)
  db = CurveDB.build([obs()], p)
  assert db.match(45.0, east_of(45.0, -122.0, 35.0), 90.0) is not None
  assert db.match(45.0, east_of(45.0, -122.0, 45.0), 90.0) is None


def test_the_radius_is_a_circle_and_not_the_bounding_box_that_pre_filters_it():
  # 30 m north AND 30 m east is 42.4 m away: inside a 40 m box on each axis, outside a 40 m circle.
  # Without this the bounding-box pre-filter would be doing the whole job and the radius check
  # could be deleted without a test noticing.
  p = tighten(PROVISIONAL_PARAMS, site_radius_m=40.0)
  db = CurveDB.build([obs()], p)
  lat = 45.0 + 30.0 / 110574.0
  assert haversine_m(45.0, -122.0, lat, east_of(45.0, -122.0, 30.0)) > 40.0
  assert db.match(lat, east_of(45.0, -122.0, 30.0), 90.0) is None


def test_match_returns_the_nearest_of_several_candidates():
  p = tighten(PROVISIONAL_PARAMS, site_radius_m=200.0)
  near = obs(k=0.001, site_lon=east_of(45.0, -122.0, 10.0))
  far = obs(k=0.009, site_lon=east_of(45.0, -122.0, 150.0), date="2026-09-02")
  db = CurveDB(p, [CurveRow(near.site_lat, near.site_lon, 90.0, [near]),
                   CurveRow(far.site_lat, far.site_lon, 90.0, [far])])
  assert db.match(45.0, -122.0, 90.0).k_eff == 0.001


def test_match_returns_none_on_an_empty_database():
  assert CurveDB(PROVISIONAL_PARAMS).match(45.0, -122.0, 90.0) is None


@pytest.mark.parametrize("bad", [(float("nan"), -122.0, 90.0), (45.0, float("inf"), 90.0),
                                 (45.0, -122.0, float("nan")), (None, -122.0, 90.0)])
def test_match_refuses_an_unusable_fix(bad):
  with pytest.raises(CurveDBError):
    CurveDB(PROVISIONAL_PARAMS).match(*bad)


def test_build_is_order_independent():
  p = PROVISIONAL_PARAMS
  a = obs(date="2026-09-01", k=0.002)
  b = obs(date="2026-09-02", k=0.006, site_lon=east_of(45.0, -122.0, 20.0))
  c = obs(date="2026-09-03", k=0.004, site_lon=east_of(45.0, -122.0, 300.0))
  one = CurveDB.build([a, b, c], p).to_snapshot()
  two = CurveDB.build([c, b, a], p).to_snapshot()
  assert one == two


def test_the_row_anchor_does_not_drift_with_later_passes():
  # Single-link clustering would let a chain of passes walk a row down the road until its position
  # names no curve at all.
  p = PROVISIONAL_PARAMS
  first = obs(date="2026-09-01")
  db = CurveDB.build([first, obs(date="2026-09-02", site_lon=east_of(45.0, -122.0, 30.0))], p)
  assert db.rows[0].site_lon == first.site_lon


def test_build_rejects_anything_that_is_not_an_observation():
  with pytest.raises(CurveDBError):
    CurveDB.build([{"k": 0.004}], PROVISIONAL_PARAMS)


def test_db_rejects_params_of_the_wrong_type():
  with pytest.raises(CurveDBError):
    CurveDB({"site_radius_m": 40.0})


def test_with_params_regroups_from_the_observations():
  wide = tighten(PROVISIONAL_PARAMS, site_radius_m=500.0)
  db = CurveDB.build([obs(), obs(date="2026-09-02", site_lon=east_of(45.0, -122.0, 200.0))],
                     PROVISIONAL_PARAMS)
  assert len(db.rows) == 2
  assert len(with_params(db, wide).rows) == 1


# --------------------------------------------------------------------------------------- snapshot

def test_snapshot_round_trips():
  db = CurveDB.build([obs(), obs(date="2026-09-02", k=0.006)], PROVISIONAL_PARAMS)
  back = CurveDB.from_snapshot(db.to_snapshot())
  assert back.to_snapshot() == db.to_snapshot()
  assert back.params == db.params
  assert back.rows[0].k_eff == 0.006


@pytest.mark.parametrize("mangle", [
  lambda s: {**s, "version": 999},
  lambda s: {k: v for k, v in s.items() if k != "params"},
  lambda s: {**s, "params": "not a dict"},
  lambda s: {**s, "params": {"site_radius_m": 40.0}},
  lambda s: {**s, "rows": [{"site_lat": 45.0}]},
  lambda s: [],
])
def test_from_snapshot_refuses_what_it_cannot_vouch_for(mangle):
  good = CurveDB.build([obs()], PROVISIONAL_PARAMS).to_snapshot()
  with pytest.raises(CurveDBError):
    CurveDB.from_snapshot(mangle(good))


def test_load_snapshot_or_empty_reports_before_it_degrades():
  seen = []
  assert load_snapshot_or_empty({"version": 999}, seen.append) is None
  assert len(seen) == 1 and isinstance(seen[0], CurveDBError)


def test_load_snapshot_or_empty_will_not_let_a_caller_swallow_silently():
  with pytest.raises(CurveDBError):
    load_snapshot_or_empty({"version": 999}, None)


# -------------------------------------------------------------------------------------- authority

def row_with(n_passes=2, n_dates=2, k=0.004):
  o = [obs(date=f"2026-09-{1 + (i % n_dates):02d}", k=k, drive_id=f"dr{i}")
       for i in range(n_passes)]
  return CurveRow(45.0, -122.0, 90.0, o)


def auth(row, **over):
  kw = dict(posted_limit_ms=29.0, highway_class="motorway", platform=LIGHTNING,
            params=PROVISIONAL_PARAMS, envelopes=PROVISIONAL_ENVELOPES)
  kw.update(over)
  return authority(row, **kw)


def test_authority_granted_on_a_qualifying_row():
  a = auth(row_with())
  assert a.granted
  # comfort 2.5 binds below the Lightning's 4.5 ceiling: sqrt(2.5/0.004) = 25.0
  assert a.v_row_ms == pytest.approx(25.0, rel=1e-9)
  assert a.reason


def test_authority_is_capped_at_the_posted_limit():
  a = auth(row_with(k=0.0002), posted_limit_ms=29.0)      # sqrt(2.5/0.0002) = 111.8 m/s
  assert a.granted and a.v_row_ms == 29.0


def test_authority_uses_the_car_ceiling_when_it_is_lower_than_comfort():
  env = {LIGHTNING: CarEnvelope(LIGHTNING, 1.0, "test")}
  a = auth(row_with(), envelopes=env, posted_limit_ms=99.0)
  assert a.v_row_ms == pytest.approx(math.sqrt(1.0 / 0.004), rel=1e-9)


@pytest.mark.parametrize(("over", "fragment"), [
  ({}, "no row"),
])
def test_authority_refuses_without_a_row(over, fragment):
  a = auth(None, **over)
  assert not a.granted and fragment in a.reason


def test_authority_refuses_a_row_with_no_curvature():
  a = auth(CurveRow(45.0, -122.0, 90.0, []))
  assert not a.granted and "curvature" in a.reason


def test_authority_refuses_a_single_pass():
  a = auth(row_with(n_passes=1, n_dates=1))
  assert not a.granted and "pass" in a.reason


def test_authority_refuses_two_observations_from_one_drive():
  # D6 again, via n_passes: the same drive contributing an UP and a DOWN is ONE pass.
  r = CurveRow(45.0, -122.0, 90.0, [obs(drive_id="d1", date="2026-09-01"),
                                    obs(drive_id="d1", date="2026-09-02", kind="down", k=0.009)])
  a = auth(r)
  assert not a.granted and "pass" in a.reason


class _FakeEnum:
  """What a capnp enum looks like to `in`: not equal to its own name, but `str()`s to it."""

  def __init__(self, name):
    self._name = name

  def __str__(self):
    return self._name


def test_authority_refuses_a_ramp_handed_in_as_an_enum_not_a_string():
  # The ramp gate used to compare the raw object against a tuple of strings, so a consumer passing
  # the capnp enum would have failed OPEN and granted authority on a ramp.
  a = auth(row_with(), highway_class=_FakeEnum("motorwayLink"))
  assert not a.granted and "ramp" in a.reason


def test_authority_refuses_an_unknown_class_handed_in_as_an_enum():
  a = auth(row_with(), highway_class=_FakeEnum("unknown"))
  assert not a.granted and "unknown" in a.reason


def test_authority_still_grants_on_a_through_road_handed_in_as_an_enum():
  # The normalisation must not break the grant path -- a check that only ever refuses is useless.
  a = auth(row_with(), highway_class=_FakeEnum("motorway"))
  assert a.granted


def test_authority_refuses_two_passes_on_one_date():
  # D6: unanimity was vacuous because 243/249 rows were n=1 and every n>=2 row was contaminated.
  a = auth(row_with(n_passes=2, n_dates=1))
  assert not a.granted and "date" in a.reason


@pytest.mark.parametrize("posted", [None, 0.0, -1.0, float("nan"), "55"])
def test_authority_refuses_without_a_posted_limit(posted):
  # Section 6.4, verbatim: no posted limit -> no authority. v1's "cap by the driver's set speed"
  # was full cancel by another name; every +574 mph prototype row had posted = 0.
  a = auth(row_with(), posted_limit_ms=posted)
  assert not a.granted and "posted" in a.reason


@pytest.mark.parametrize("cls", [None, "", "unknown", "none", "None"])
def test_authority_refuses_an_unknown_highway_class(cls):
  # `unknown` is a real HighwayClass member and arrives as a TRUTHY string.
  a = auth(row_with(), highway_class=cls)
  assert not a.granted and "unknown" in a.reason


@pytest.mark.parametrize("cls", ["motorwayLink", "trunkLink", "primaryLink", "secondaryLink",
                                 "tertiaryLink"])
def test_authority_refuses_every_ramp(cls):
  a = auth(row_with(), highway_class=cls)
  assert not a.granted and "ramp" in a.reason


def test_authority_refuses_a_platform_with_no_measured_envelope():
  a = auth(row_with(), platform="TESLA_MODEL_S_HW3")
  assert not a.granted and "envelope" in a.reason


def test_authority_refuses_an_empty_platform():
  a = auth(row_with(), platform="")
  assert not a.granted


# ----------------------------------------------------------------------------- the cancel formula

def test_cancel_target_is_min_ref_max_icbm_vrow():
  assert cancel_target(29.0, 19.0, 25.0) == 25.0     # the database raises the target
  assert cancel_target(29.0, 19.0, 15.0) == 19.0     # a lower v_row never lowers it further
  assert cancel_target(29.0, 19.0, 40.0) == 29.0     # and never above the reference


def test_cancel_target_without_a_row_changes_nothing():
  assert cancel_target(29.0, 19.0, None) == 19.0


def test_cancel_target_can_never_exceed_the_reference():
  for v in (0.0, 5.0, 19.0, 29.0, 100.0):
    assert cancel_target(29.0, 19.0, v) <= 29.0


@pytest.mark.parametrize("args", [(float("nan"), 19.0, 25.0), (29.0, float("inf"), 25.0),
                                  (29.0, 19.0, float("nan")), ("29", 19.0, 25.0)])
def test_cancel_target_refuses_nonsense(args):
  with pytest.raises(CurveDBError):
    cancel_target(*args)
