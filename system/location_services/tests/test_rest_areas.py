"""restarea2pnw — the bundled I-5 rest-area data loads whole and is direction-correct.

Found 2026-09-24: the SeaTac northbound rest area (I-5 MP 140, Federal Way; WSDOT "SeaTac - I-5
northbound") was missing (the OSM gatherer skipped relations, and SeaTac is mapped as one), and
Maytown / Scatter Creek / Silver Lake carried dir "" although each serves only one direction
(WSDOT official names), so the southbound-only Maytown showed to northbound traffic.
"""

import json
import os

from openpilot.system.location_services import location_servicesd as lsd

I5_FILE = os.path.join(lsd.REST_DIR, "i5_rest_areas.json")


def _i5_items():
  return [r for r in lsd.StaticData().rest if "I 5" in r["refs"]]


def _pick(lat, lon, brg):
  return lsd._line_rest_corridor(_i5_items(), lat, lon, brg, "I 5")


def test_every_i5_entry_loads_and_has_a_direction():
  # The loader skips a malformed entry without a word; count so a bad edit cannot vanish quietly.
  with open(I5_FILE) as f:
    raw = json.load(f)
  items = _i5_items()
  assert len(items) == len(raw)
  # Every I-5 rest area in WA/OR serves one side of the freeway; "" would show it in both directions.
  assert all(r["dir"] in ("N", "S") for r in items), [r["name"] for r in items if r["dir"] not in ("N", "S")]


def test_seatac_shows_northbound():
  # I-5 northbound near Fife/Milton, ~4 mi south of the rest area, heading NNE.
  r = _pick(47.22, -122.36, 30.0)
  assert r is not None
  poi, mi = r
  assert poi["name"] == "SeaTac"
  assert poi["dir"] == "N"
  assert 3.0 < mi < 5.5
  assert lsd.geo.haversine_m(poi["lat"], poi["lon"], 47.271134, -122.314557) < 300.0  # WSDOT coordinates


def test_seatac_hidden_southbound():
  # I-5 southbound just north of it, heading SSW: SeaTac is on the other side; nothing else within 15 mi.
  assert _pick(47.30, -122.29, 210.0) is None


def test_maytown_is_southbound_only():
  # Northbound between Scatter Creek (MP 90) and Maytown (MP 93): Maytown is SB-only, SeaTac is ~29 mi on.
  assert _pick(46.85, -122.98, 20.0) is None
  # Southbound north of Maytown: it shows.
  r = _pick(46.90, -122.955, 200.0)
  assert r is not None and r[0]["name"] == "Maytown"
