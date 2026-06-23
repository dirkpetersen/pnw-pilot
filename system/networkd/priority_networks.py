"""
network2xnor: shared PURE data model for multiple priority/home WiFi networks.

Generalizes the original single (TetheringPriorityWifi + TetheringHomeLocation) pair into a LIST of
location+SSID pairs, stored as JSON in the `TetheringPriorityNetworks` param. Each entry:

    {"label": "Home", "ssid": "MyWifi", "lat": 45.512, "lon": -122.681, "portal": null}

- label  : human name shown in the UI list (free text; defaults to the ssid).
- ssid   : the WiFi SSID the arbiter switches to when in range (and a saved NM connection exists).
- lat/lon: GPS center of this network's geofence (auto-learned when connected; or "record" button).
            May be null until learned -> that entry simply isn't geo-gated yet (fail-open).
- portal : optional captive-portal handler id (see captive_portal.py), e.g. "peak". null = none.

PURE: parsing, migration, and selection only — no params/NM/requests I/O. The daemon and the UI both
import these helpers so the schema lives in exactly one place.
"""
from __future__ import annotations

import json


def _coerce_entry(d: dict) -> dict | None:
  """Validate/normalize one raw entry. Returns a clean dict or None if unusable (no ssid)."""
  if not isinstance(d, dict):
    return None
  ssid = str(d.get("ssid", "")).strip()
  if not ssid:
    return None
  label = str(d.get("label", "")).strip() or ssid
  portal = d.get("portal")
  portal = str(portal).strip() or None if portal else None

  def _f(v):
    try:
      return float(v)
    except (TypeError, ValueError):
      return None
  return {"label": label, "ssid": ssid, "lat": _f(d.get("lat")), "lon": _f(d.get("lon")), "portal": portal}


def parse(raw: str | bytes | None,
          legacy_ssid: str | None = None,
          legacy_home_raw: str | bytes | None = None) -> list[dict]:
  """Parse the TetheringPriorityNetworks JSON list into clean entries.

  Backward-compat MIGRATION: ONLY when the new param has never been set (raw is None/empty) do we
  synthesize a one-entry list from the OLD single-network params (legacy_ssid = TetheringPriorityWifi,
  legacy_home_raw = TetheringHomeLocation JSON [lat,lon]), so existing setups keep working with no user
  action. Once TetheringPriorityNetworks holds a valid JSON list — INCLUDING an empty `[]` (the user
  deleted all networks via the UI) — that list is authoritative and we do NOT resurrect the legacy
  network. (Resurrecting on an empty list would make a deleted legacy network un-removable.)
  """
  out: list[dict] = []
  parsed_a_list = False
  if raw:
    try:
      data = json.loads(raw) if isinstance(raw, (str, bytes, bytearray)) else raw
      if isinstance(data, list):
        parsed_a_list = True
        for d in data:
          e = _coerce_entry(d)
          if e is not None and not any(e["ssid"] == x["ssid"] for x in out):
            out.append(e)
    except (ValueError, TypeError):
      out = []

  # migrate ONLY if the new param was never a valid list (never set / corrupt) — not when it's [].
  if not out and not parsed_a_list and legacy_ssid and legacy_ssid.strip():
    lat = lon = None
    if legacy_home_raw:
      try:
        h = json.loads(legacy_home_raw) if isinstance(legacy_home_raw, (str, bytes, bytearray)) else legacy_home_raw
        lat, lon = float(h[0]), float(h[1])
      except (ValueError, TypeError, IndexError, KeyError):
        lat = lon = None
    out.append({"label": legacy_ssid.strip(), "ssid": legacy_ssid.strip(),
                "lat": lat, "lon": lon, "portal": None})
  return out


def dumps(nets: list[dict]) -> str:
  """Serialize entries back to the JSON stored in the param (clean/normalized)."""
  clean = [e for e in (_coerce_entry(n) for n in nets) if e is not None]
  return json.dumps(clean)


def ssids(nets: list[dict]) -> list[str]:
  """All configured SSIDs."""
  return [e["ssid"] for e in nets]


def locations(nets: list[dict]) -> list[tuple[float, float]]:
  """All learned (lat, lon) centers (entries without a fix are skipped)."""
  return [(e["lat"], e["lon"]) for e in nets if e.get("lat") is not None and e.get("lon") is not None]


def select_available(nets: list[dict], scan_ssids: list[str], saved_connections: list[str],
                     priority_connection_id) -> dict | None:
  """The first configured network that is BOTH visible in the scan AND has a saved NM connection.
  `priority_connection_id` is the id-builder from network_arbiter (kept as a param to stay pure)."""
  scan = set(scan_ssids)
  saved = set(saved_connections)
  for e in nets:
    if e["ssid"] in scan and priority_connection_id(e["ssid"]) in saved:
      return e
  return None


def entry_for_ssid(nets: list[dict], ssid: str) -> dict | None:
  """The configured entry whose ssid matches (used for auto-learn + captive-portal lookup)."""
  if not ssid:
    return None
  for e in nets:
    if e["ssid"] == ssid:
      return e
  return None
