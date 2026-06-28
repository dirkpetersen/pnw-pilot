#!/usr/bin/env python3
"""
mapd2pnw: one-shot PNW map auto-download for the official pfeiferj mapd binary.

The mapd binary ships with every speed/curve CONTROL disabled (safe default) and downloads no map
data on its own. This tiny daemon asks mapd — once, on the first unmetered Wi-Fi connection — to
download the Pacific Northwest map set (Washington, Oregon, Idaho) via a `mapdIn` download message.

It is guarded by the MapdPnwMapsRequested param so it only fires once, and it keeps re-sending until
mapd reports the download started (the message can be missed if mapd's socket isn't up yet), then
stops. Speed-limit DISPLAY works as soon as the maps are present; the user opts into speed/curve
CONTROL later via MapdSettings — this daemon never enables control.
"""
import json
import cereal.messaging as messaging
from cereal import log
from openpilot.common.gps import get_gps_location_service
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog

# Download-menu paths are period-delimited keys from mapd's download_menu.json. The US states
# table is "us_state" (SINGULAR), e.g. "us_state.WA". Comma-join multiple areas.
PNW_DOWNLOAD = "us_state.WA,us_state.OR,us_state.ID"
NetworkType = log.DeviceState.NetworkType


def _on_unmetered_wifi(ds) -> bool:
  return ds.networkType == NetworkType.wifi and not ds.networkMetered


def main():
  params = Params()
  mem = Params("/dev/shm/params")   # CES + the on-road overlay read the legacy map params from here
  pm = messaging.PubMaster(['mapdIn'])
  gps_service = get_gps_location_service(params)
  sm = messaging.SubMaster(['deviceState', 'mapdExtendedOut', 'mapdOut', gps_service])

  if params.get_bool("MapdPnwMapsRequested"):
    cloudlog.info("mapd_configd: PNW maps already requested; idle (re-checks the param each loop)")
  last_covered = None

  while True:
    sm.update(1000)  # paces the loop (blocks up to 1 s); no extra sleep

    # mapd2pnw bridge: the official pfeiferj mapd v2.0.6 publishes everything over CEREAL
    # (mapdOut / mapdExtendedOut), but CES (selfdrive/controls/lib/ces_pnw) and the on-road CES
    # overlay still read the legacy in-memory params the OLD mapd binary used to write directly
    # (MapTargetVelocities / LastGPSPosition / MapSpeedLimit). Translate the cereal output into those
    # mem params so CES's map-curve trigger + the overlay "map" line come alive. Display/decision only;
    # actual map braking is the longitudinal_planner mapdOut.suggestedSpeed cap (separate, gated OFF).
    try:
      if sm.alive[gps_service]:
        g = sm[gps_service]
        mem.put_nonblocking("LastGPSPosition", json.dumps({
          "latitude": float(g.latitude), "longitude": float(g.longitude),
          "bearing": float(getattr(g, "bearingDeg", 0.0))}))
      if sm.alive['mapdOut']:
        mem.put_nonblocking("MapSpeedLimit", str(float(sm['mapdOut'].speedLimit)))  # m/s; 0 = none
      if sm.alive['mapdExtendedOut']:
        # mapdExtendedOut.path = List(MapdPathPoint{latitude, longitude, curvature, targetVelocity});
        # CES's upcoming_curve() wants a list of {latitude, longitude, velocity} (m/s).
        mem.put_nonblocking("MapTargetVelocities", [
          {"latitude": float(p.latitude), "longitude": float(p.longitude), "velocity": float(p.targetVelocity)}
          for p in sm['mapdExtendedOut'].path])
    except Exception:
      cloudlog.exception("mapd_configd: mapd->CES bridge write failed")

    # mapd2pnw: drive the "Get map for this location" toggle grey-out (param MapForLocationCovered).
    # The toggle should be GREYED (covered) unless we KNOW we're somewhere with no downloaded map.
    # covered = no GPS fix (can't tell where we are, e.g. parked offroad) OR mapd has a map tile
    # loaded for the current position. Only a fix in an uncovered area enables the toggle. Replaces
    # the deleted sunnypilot coverage writer; written only on change to avoid churning the param.
    has_fix = sm.alive[gps_service]
    tile_here = sm.alive['mapdOut'] and sm['mapdOut'].tileLoaded
    covered = (not has_fix) or tile_here
    if covered != last_covered:
      params.put_bool("MapForLocationCovered", covered)
      last_covered = covered

    # Re-read the guard each loop (not once at startup) so re-arming the download — resetting
    # MapdPnwMapsRequested to 0 — takes effect without restarting this daemon.
    if params.get_bool("MapdPnwMapsRequested"):
      continue

    if not sm.alive['mapdExtendedOut']:
      continue  # mapd not up yet (binary still downloading at launch, or not started)

    # Check FRESH state at the top of the loop: did our request already start a download? If so, the
    # one-shot is complete. (Checking right after send() would read pre-send state and re-spam.)
    prog = sm['mapdExtendedOut'].downloadProgress
    if prog.active or prog.totalFiles > 0:
      params.put_bool("MapdPnwMapsRequested", True)  # one-shot guard; re-read at the top of the loop
      cloudlog.warning("mapd_configd: PNW download started; one-shot guard set")
      continue

    # Not started yet: on unmetered Wi-Fi, (re)send the download request. Resends each loop until mapd
    # picks it up (a message can be missed before mapd's mapdIn socket is ready), then the check above
    # ends it. mapd_configd is the only mapdIn publisher.
    if sm.alive['deviceState'] and _on_unmetered_wifi(sm['deviceState']):
      msg = messaging.new_message('mapdIn')
      msg.mapdIn.type = 'download'
      msg.mapdIn.str = PNW_DOWNLOAD
      pm.send('mapdIn', msg)
      cloudlog.warning(f"mapd_configd: requested PNW map download: {PNW_DOWNLOAD}")


if __name__ == "__main__":
  main()
