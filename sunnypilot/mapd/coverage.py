"""
mapd2pnw: map-coverage helpers for the "Get map for this location" on-demand download.

PURE geometry over the region bounding-box table extracted from the bundled pfeiferj mapd binary
(`regions.json`: {"states": {US 2-letter -> {full_name, bbox}}, "nations": {2-letter -> ...}}).
bbox = [min_lon, min_lat, max_lon, max_lat].

Used by mapd_manager + the UI toggle to answer two questions:
  - region_for_gps(lat, lon)      -> the region code covering this point (US state preferred over nation)
  - is_covered(region, downloaded)-> is that region already in the downloaded set?

No I/O beyond loading the bundled JSON once. The binary keys US states and nations BOTH by 2-letter
code and they COLLIDE (e.g. "ID" = Idaho the state AND Indonesia the nation), so the two are kept in
separate sub-tables and a US state is always preferred when a point falls in both.
"""
from __future__ import annotations

import json
import os

_REGIONS_PATH = os.path.join(os.path.dirname(__file__), "regions.json")
_regions_cache: dict | None = None


def _regions() -> dict:
  global _regions_cache
  if _regions_cache is None:
    try:
      with open(_REGIONS_PATH) as f:
        _regions_cache = json.load(f)
    except Exception:
      _regions_cache = {"states": {}, "nations": {}}
  return _regions_cache


def _in_bbox(lat: float, lon: float, bbox: list) -> bool:
  min_lon, min_lat, max_lon, max_lat = bbox
  if not (min_lat <= lat <= max_lat):
    return False
  # antimeridian-crossing box (e.g. Alaska: min_lon ~+179, max_lon ~-179): the lon range wraps through
  # 180, so a plain min<=lon<=max test fails. When min_lon > max_lon, accept points on EITHER side of 180.
  if min_lon <= max_lon:
    return min_lon <= lon <= max_lon
  return lon >= min_lon or lon <= max_lon


def _bbox_area(bbox: list) -> float:
  """Approx lon*lat area; handles antimeridian-crossing boxes (else span goes negative)."""
  min_lon, min_lat, max_lon, max_lat = bbox
  lon_span = (max_lon - min_lon) if min_lon <= max_lon else (360.0 - (min_lon - max_lon))
  return lon_span * (max_lat - min_lat)


def region_for_gps(lat: float | None, lon: float | None) -> str | None:
  """Region code covering (lat, lon), or None if no fix / not in any table region.
  US state is preferred over a nation when a point falls in both (states are the granular download)."""
  if lat is None or lon is None:
    return None
  reg = _regions()
  # smallest-area US state first (states overlap less; pick the tightest box)
  best, best_area = None, None
  for code, e in reg.get("states", {}).items():
    b = e.get("bbox")
    if b and _in_bbox(lat, lon, b):
      area = _bbox_area(b)
      if best_area is None or area < best_area:
        best, best_area = code, area
  if best is not None:
    return best
  # No US state covers it -> fall back to a nation (e.g. BC -> "CA"). Pick the tightest box.
  # EXCLUDE the "US" nation here: US coverage IS the 50 state boxes, so a point not in any state box
  # is not really in the US — and the US national box spuriously overflows north into southern Canada
  # (top lat 49.5), which would otherwise mis-resolve Vancouver BC (49.3) to "US" instead of Canada.
  best, best_area = None, None
  for code, e in reg.get("nations", {}).items():
    if code == "US":
      continue
    b = e.get("bbox")
    if b and _in_bbox(lat, lon, b):
      area = _bbox_area(b)
      if best_area is None or area < best_area:
        best, best_area = code, area
  return best


def is_us_state(code: str | None) -> bool:
  return bool(code) and code in _regions().get("states", {})


def region_full_name(code: str | None) -> str:
  if not code:
    return ""
  reg = _regions()
  return (reg.get("states", {}).get(code) or reg.get("nations", {}).get(code) or {}).get("full_name", code)


def is_covered(region: str | None, downloaded_regions: list[str]) -> bool:
  """True if `region` (the code under current GPS) is already in the downloaded set.
  None region (no fix / unknown) is treated as covered = True, so the toggle stays inactive rather
  than prompting a download we can't place."""
  if region is None:
    return True
  return region in set(downloaded_regions or [])
