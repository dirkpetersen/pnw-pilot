"""
network2xnor: GPS geofence gate for priority-WiFi scanning (PURE).

The comma has ONE WiFi radio. While tethering (AP mode), the arbiter periodically SCANS for the
priority ("home") SSID so it can switch back when you arrive home. But scanning forces the radio
off-channel (confirmed by NetworkManager's maintainer — scans induce lag/drops), which on a single
radio competes with the hotspot and can blip connectivity. There's no point scanning for home WiFi
when you're nowhere near home.

So: gate the scan on GPS. Record the home location (where the priority WiFi lives) and only scan/switch
when the device is within HOME_GEOFENCE_M of it. Far from home -> never scan, just hold the hotspot.

PURE: haversine + the near-home decision, no I/O. `network_arbiterd` supplies GPS (LastGPSPosition)
and the stored home location. SI: metres.
"""
import math

HOME_GEOFENCE_M = 250.0   # within this of the saved home location -> allow scan/switch to priority WiFi
# auto-capture: when we're actually connected to the priority WiFi, that IS home -> remember the spot.


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
  """Great-circle distance in metres between two lat/lon points. Pure."""
  r = 6371000.0
  p1, p2 = math.radians(lat1), math.radians(lat2)
  dp = math.radians(lat2 - lat1)
  dl = math.radians(lon2 - lon1)
  a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
  return 2 * r * math.asin(min(1.0, math.sqrt(a)))


def near_home(home: tuple[float, float] | None, cur: tuple[float, float] | None,
              radius_m: float = HOME_GEOFENCE_M) -> bool:
  """True if current GPS is within radius of the saved home location.

  Returns True (fail-OPEN) when we DON'T have both fixes — so a missing/!learned home location or a
  GPS dropout never *prevents* finding home WiFi; it just falls back to the old always-scan behavior.
  Only a confident "we are far from a KNOWN home" suppresses scanning.
  """
  if home is None or cur is None:
    return True
  hlat, hlon = home
  clat, clon = cur
  if hlat is None or hlon is None or clat is None or clon is None:
    return True
  return haversine_m(hlat, hlon, clat, clon) <= radius_m


def near_any_home(homes: list[tuple[float, float]], cur: tuple[float, float] | None,
                  radius_m: float = HOME_GEOFENCE_M) -> bool:
  """network2xnor (multi-location): True if current GPS is within radius of ANY saved location.

  Same fail-OPEN contract as near_home: returns True when we have no current fix, or when NO
  locations are learned yet (so scanning falls back to always-on rather than being suppressed). Only
  a confident "we are far from EVERY known location" returns False.
  """
  if cur is None or not homes:
    return True
  return any(near_home(h, cur, radius_m) for h in homes)
