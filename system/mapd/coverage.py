"""
mapdstate2pnw: map-coverage helpers for the GPS-driven, on-demand, whole-STATE mapd download.

PURE geometry over the region bounding-box table extracted from the bundled pfeiferj mapd binary
(`regions.json`: {"states": {US 2-letter -> {full_name, bbox}}, "nations": {2-letter -> ...}}).
bbox = [min_lon, min_lat, max_lon, max_lat].

Ported (2026-08-15) from the sunnypilot-derived `sunnypilot/mapd/coverage.py` that was deleted in
cd1bc37b0e alongside the rest of the old disabled mapd runtime — this module (and regions.json) is
pure and had no dependency on that runtime, so it's resurrected verbatim here plus two small
additions (`region_bbox` / `download_key`) needed to drive mapd's `mapdIn` download menu and to find
which offline tile directories belong to a region (for the "Refresh this location map" delete-then-
redownload action).

Used by system/mapd/mapd_configd.py to answer:
  - region_for_gps(lat, lon)      -> the region code covering this point (US state preferred over nation)
  - download_key(region)          -> the mapdIn.str download-menu key for that region ("us_state.WA" /
                                      "nation.CA")
  - region_bbox(region)           -> that region's [min_lon, min_lat, max_lon, max_lat], for locating
                                      its offline tile directories on disk

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


def _locate(lat: float | None, lon: float | None) -> tuple[str | None, bool]:
  """(code, is_us_state) for the region covering (lat, lon), or (None, False). US state is preferred
  over a nation when a point falls in both (states are the granular download); ties within a table
  broken by the tightest (smallest-area) bbox.

  Factored out of region_for_gps() so a caller that needs to know WHICH table matched can get that
  from the SAME lookup pass — the states/nations tables COLLIDE on 20 two-letter codes (e.g. "CA" =
  California the US state AND Canada the nation; "ID" = Idaho AND Indonesia), so re-deriving
  "is this a US state?" from a bare code after the fact (e.g. a naive `code in states table` check)
  silently picks the wrong one whenever a nation-side match happens to share a state's code."""
  if lat is None or lon is None:
    return None, False
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
    return best, True
  # No US state covers it -> fall back to a nation (e.g. BC -> "CA" = Canada). Pick the tightest box.
  # EXCLUDE the "US" nation here: US coverage IS the 50 state boxes, so a point not in any state box
  # is not really in the US — and the US national box spuriously overflows north into southern Canada
  # (top lat 49.5), which would otherwise mis-resolve Vancouver BC (49.3) to "US" instead of Canada.
  # This is also what guarantees mapd_configd never sends the whole-US "nation.US" download key: a
  # region code of "US" is structurally impossible out of this function.
  best, best_area = None, None
  for code, e in reg.get("nations", {}).items():
    if code == "US":
      continue
    b = e.get("bbox")
    if b and _in_bbox(lat, lon, b):
      area = _bbox_area(b)
      if best_area is None or area < best_area:
        best, best_area = code, area
  return best, False


def region_for_gps(lat: float | None, lon: float | None) -> str | None:
  """Region code covering (lat, lon), or None if no fix / not in any table region.
  US state is preferred over a nation when a point falls in both (states are the granular download).

  NOTE the states/nations tables collide on 20 codes (CA/ID/AZ/... — see _locate's docstring): this
  bare code alone does not say which table it came from. Callers that need to act differently for a
  state vs. a nation (e.g. building mapd's download-menu key) MUST use region_and_key_for_gps()
  instead of re-deriving it from this return value with is_us_state()."""
  return _locate(lat, lon)[0]


def is_us_state(code: str | None) -> bool:
  """True if `code` names a US state in the table. AMBIGUOUS for the 20 collision codes (a bare code
  can't say whether it was really matched as a state or a nation) — do not use this to interpret a
  code that came out of region_for_gps(); use region_and_key_for_gps() instead, which resolves the
  ambiguity during the same lookup pass that found the match."""
  return bool(code) and code in _regions().get("states", {})


def region_full_name(code: str | None) -> str:
  if not code:
    return ""
  reg = _regions()
  return (reg.get("states", {}).get(code) or reg.get("nations", {}).get(code) or {}).get("full_name", code)


def region_bbox(code: str | None, is_us_state: bool | None = None) -> list[float] | None:
  """[min_lon, min_lat, max_lon, max_lat] for `code`, or None if unknown. Used to locate the offline
  tile directories on disk for a "Refresh this location map" delete.

  `is_us_state`, when given, picks the table directly instead of guessing states-first — REQUIRED for
  correctness on any of the 20 collision codes (e.g. "CA" = California the US state AND Canada the
  nation): a states-first guess silently returns California's bbox for a fix that was actually
  resolved as Canada, which is exactly what made "Refresh this location map" delete the wrong
  region's tiles for a fix in Whistler BC. Callers that resolved the code via _locate() /
  region_and_key_for_gps() have this flag already (region_and_key_for_gps()'s key prefix
  "us_state."/"nation." tells you which) and MUST pass it through. Omitting it falls back to the old
  states-first guess — only safe for a code the caller already knows is unambiguous."""
  if not code:
    return None
  reg = _regions()
  if is_us_state is True:
    e = reg.get("states", {}).get(code)
  elif is_us_state is False:
    e = reg.get("nations", {}).get(code)
  else:
    e = reg.get("states", {}).get(code) or reg.get("nations", {}).get(code)
  return e.get("bbox") if e else None


def region_and_key_for_gps(lat: float | None, lon: float | None) -> tuple[str | None, str | None]:
  """(region code, mapdIn.str download-menu key) for (lat, lon), or (None, None).

  This is the function mapd_configd.py should call to build a download request — it resolves the
  state-vs-nation collision correctly because it reads which table matched off the SAME lookup pass
  that found the region, instead of re-guessing from a bare code afterward (region_for_gps() +
  is_us_state(code) would silently mis-key all 20 collision codes, e.g. sending "us_state.CA"
  (California) for a fix in Canada). Key format matches the live mapd binary's
  settings/download_menu.json top-level tables: "us_state.<CODE>" / "nation.<CODE>" (singular, not
  this module's "states"/"nations" table names)."""
  code, is_state = _locate(lat, lon)
  if code is None:
    return None, None
  return code, (f"us_state.{code}" if is_state else f"nation.{code}")


def is_covered(region: str | None, downloaded_regions: list[str]) -> bool:
  """True if `region` (the code under current GPS) is already in the downloaded set.
  None region (no fix / unknown) is treated as covered = True, so a caller stays inactive rather
  than acting on a region we can't place."""
  if region is None:
    return True
  return region in set(downloaded_regions or [])
