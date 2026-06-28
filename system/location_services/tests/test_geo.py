"""location2pnw: unit tests for the pure 'what's ahead' geometry (no openpilot deps needed)."""

from openpilot.system.location_services import geo

# a straight EAST path from (47.5, -122.0), ~5 points over ~1.8 km
PATH = [{"latitude": 47.5, "longitude": -122.0 + i * 0.005} for i in range(5)]


def test_haversine_and_bearing():
  # 0.01 deg lon at this latitude ~ 750 m; bearing due east ~ 90 deg
  d = geo.haversine_m(47.5, -122.0, 47.5, -121.99)
  assert 740 < d < 760, d
  assert abs(geo.bearing_deg(47.5, -122.0, 47.5, -121.99) - 90.0) < 1.0


def test_normalize180():
  assert geo.normalize180(190.0) == -170.0
  assert geo.normalize180(-190.0) == 170.0


def test_ahead_on_path():
  a = geo.ahead(PATH, 47.5, -122.0, 90.0, 47.5, -121.99)
  assert a is not None and a["source"] == "path"
  assert 700 < a["along_m"] < 800
  assert a["perp_m"] < 30


def test_behind_returns_none():
  assert geo.ahead(PATH, 47.5, -122.0, 90.0, 47.5, -122.02) is None


def test_cone_fallback_past_path_end():
  # POI ~3.7 km east, well past the ~1.8 km path -> cone branch
  a = geo.ahead(PATH, 47.5, -122.0, 90.0, 47.5, -121.95)
  assert a is not None and a["source"] == "cone"
  assert a["perp_m"] < 50


def test_no_bearing_no_path_is_none():
  assert geo.ahead([], 47.5, -122.0, None, 47.5, -121.95) is None


def test_perp_offset_in_cone():
  # POI off to the left ~ a few hundred meters perpendicular
  a = geo.ahead([], 47.5, -122.0, 90.0, 47.502, -121.97)
  assert a is not None
  assert a["perp_m"] > 100  # genuinely off the heading line


def test_nearest_ahead_and_perp_filter():
  pois = [{"lat": 47.5, "lon": -121.99, "id": "near"},
          {"lat": 47.5, "lon": -121.97, "id": "far"}]
  poi, a = geo.nearest_ahead(PATH, 47.5, -122.0, 90.0, pois)
  assert poi["id"] == "near"
  # tight perp filter excludes a sideways POI
  side = [{"lat": 47.55, "lon": -121.99, "id": "sideways"}]
  poi2, _ = geo.nearest_ahead(PATH, 47.5, -122.0, 90.0, side, max_perp_m=100.0)
  assert poi2 is None
