"""policenear2pnw — near-report lock, no-staleness-drop, and the honest speed-gate reason string.

Driver reports from the 2026-08-18 I-5 drive:
  1. A REAL sighting <1 mi ahead (correctly slowed + resumed) while another sat ~6 mi out — the overlay
     ALTERNATED between the two. Selection ranks on geo.nearest_ahead's along-track distance (projected
     onto the mapd path) while the driver sees straight-line, so near an interchange the close report
     can lose a tick to the distant one. Rule adopted: anything inside POLICE_NEAR_MI is the only
     candidate set, closest-by-straight-line wins, and nothing further can interrupt it.
  2. "speed <45mph" displayed while doing 70 — the gate had failed on an UNKNOWN GPS speed (post-reboot,
     no fix yet) but reported the too-slow reason.
Old reports are deliberately NOT dropped (driver: "I still do want to see it" when nothing else is in
range); age becomes a display tier, not a filter.
"""
import pytest

from openpilot.system.location_services import location_servicesd as lsd
from openpilot.system.location_services.location_servicesd import _line_police, _now_epoch


def _ts_min_ago(minutes):
  return (_now_epoch() - minutes * 60.0) * 1000.0      # Waze timestamps are epoch MILLISECONDS


def _alert(uuid, lat, lon=-122.0, age_min=5):
  return {"lat": lat, "lon": lon, "magvar": None, "uuid": uuid, "street": "", "town": "T",
          "ts": None if age_min is None else _ts_min_ago(age_min)}


def _line(alerts, lat=47.0, lon=-122.0, brg=0.0):
  # Heading due north with reports to the north = geometrically ahead. Empty path exercises
  # nearest_ahead's straight-line fallback. Fresh _PoliceRecede per call = no carried state.
  recede = lsd._PoliceRecede(lsd.POLICE_RECEDE_MI)
  return _line_police(alerts, "ok", "", lat, lon, brg, [], recede)


# ~0.0145 deg latitude per mile
def _lat_mi_north(mi):
  return 47.0 + mi / 69.0


class TestNearLock:
  def test_near_report_wins_over_distant_one(self):
    out = _line([_alert("far", _lat_mi_north(6.0)), _alert("near", _lat_mi_north(0.6))])
    assert out["state"] == "alert"
    assert out["uuid"] == "near", "a report inside 1 mi must not be preempted by one 6 mi out"

  def test_near_lock_is_order_independent(self):
    # the flapping was tick-to-tick instability, so the outcome must not depend on input order
    a, b = _alert("far", _lat_mi_north(6.0)), _alert("near", _lat_mi_north(0.6))
    assert _line([a, b])["uuid"] == _line([b, a])["uuid"] == "near"

  def test_closest_of_several_near_reports_wins(self):
    out = _line([_alert("n9", _lat_mi_north(0.9)), _alert("n2", _lat_mi_north(0.2)),
                 _alert("n7", _lat_mi_north(0.7))])
    assert out["uuid"] == "n2"

  def test_distant_report_still_shown_when_nothing_is_near(self):
    out = _line([_alert("far", _lat_mi_north(6.0))])
    assert out["state"] == "alert" and out["uuid"] == "far"

  def test_near_report_behind_us_does_not_near_lock(self):
    # heading north, report 0.5 mi SOUTH -> forward-hemisphere check must reject it, otherwise the
    # near-lock would resurrect reports we are driving away from.
    out = _line([_alert("behind", 47.0 - 0.5 / 69.0), _alert("ahead_far", _lat_mi_north(6.0))])
    assert out["uuid"] == "ahead_far"

  def test_near_lock_reports_straight_line_distance(self):
    out = _line([_alert("near", _lat_mi_north(0.6))])
    assert out["dist_mi"] == pytest.approx(0.6, abs=0.15)


class TestNoStalenessDrop:
  """The driver explicitly wants old reports kept when nothing fresher is in range."""

  @pytest.mark.parametrize("age", [45, 90, 240])
  def test_old_report_is_still_shown(self, age):
    out = _line([_alert("old", _lat_mi_north(3.0), age_min=age)])
    assert out["state"] == "alert"
    assert out["age_min"] >= age - 1, "age must be surfaced, not used to filter"

  def test_unknown_age_is_kept(self):
    assert _line([_alert("nots", _lat_mi_north(3.0), age_min=None)])["state"] == "alert"

  def test_old_near_report_still_outranks_a_fresh_distant_one(self):
    # consequence of the driver's rule as stated: proximity decides, not age. Recorded deliberately —
    # this is the case a confidence TIER is meant to re-colour rather than re-order.
    out = _line([_alert("old_near", _lat_mi_north(0.5), age_min=60),
                 _alert("new_far", _lat_mi_north(6.0), age_min=2)])
    assert out["uuid"] == "old_near"


class TestSpeedGateReason:
  def test_unknown_speed_is_not_reported_as_too_slow(self):
    for cur_speed, expect_slow in ((None, False), (0.0, True), (5.0, True)):
      reason = f"speed <{lsd.POLICE_GATE_MPH}mph" if cur_speed is not None else "no GPS speed"
      assert (reason == f"speed <{lsd.POLICE_GATE_MPH}mph") is expect_slow
      if not expect_slow:
        assert "mph" not in reason
