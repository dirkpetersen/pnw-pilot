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
import cereal.messaging as messaging
from cereal import log
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
  pm = messaging.PubMaster(['mapdIn'])
  sm = messaging.SubMaster(['deviceState', 'mapdExtendedOut'])

  if params.get_bool("MapdPnwMapsRequested"):
    cloudlog.info("mapd_configd: PNW maps already requested; idle (re-checks the param each loop)")

  while True:
    sm.update(1000)  # paces the loop (blocks up to 1 s on deviceState/mapdExtendedOut); no extra sleep
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
