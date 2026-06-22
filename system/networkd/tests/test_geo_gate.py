"""Unit tests for the pure GPS geofence gate (network2xnor)."""
from openpilot.system.networkd.geo_gate import haversine_m, near_home


def test_haversine_zero():
  assert haversine_m(45.0, -122.0, 45.0, -122.0) == 0.0


def test_haversine_known_distance():
  # ~111 km per degree of latitude near the equator/mid-latitudes
  d = haversine_m(45.0, -122.0, 46.0, -122.0)
  assert 110_000 < d < 112_000


def test_near_home_true_when_close():
  home = (45.5230, -122.6810)
  cur = (45.5231, -122.6811)        # ~15 m away
  assert near_home(home, cur) is True


def test_near_home_false_when_far():
  home = (45.5230, -122.6810)
  cur = (45.5400, -122.6810)        # ~1.9 km north -> outside 250 m
  assert near_home(home, cur) is False


def test_geofence_boundary():
  home = (45.0, -122.0)
  # a point ~100 m away is inside the default 250 m fence; ~300 m is outside
  near = (45.0 + 100 / 111_320.0, -122.0)
  far = (45.0 + 300 / 111_320.0, -122.0)
  assert near_home(home, near) is True
  assert near_home(home, far) is False


def test_fail_open_when_home_unknown():
  # no learned home -> scan as before (don't suppress)
  assert near_home(None, (45.0, -122.0)) is True


def test_fail_open_when_gps_unknown():
  assert near_home((45.0, -122.0), None) is True


def test_fail_open_when_either_coord_none():
  assert near_home((None, -122.0), (45.0, -122.0)) is True
  assert near_home((45.0, -122.0), (45.0, None)) is True


def test_custom_radius():
  home = (45.0, -122.0)
  cur = (45.0 + 400 / 111_320.0, -122.0)   # ~400 m
  assert near_home(home, cur, radius_m=250.0) is False
  assert near_home(home, cur, radius_m=500.0) is True
