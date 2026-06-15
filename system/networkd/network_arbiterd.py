#!/usr/bin/env python3
"""
network2xnor: arbitration supervisor for perpetual tethering + priority WiFi.

The comma 3X has ONE WiFi radio: it can run the hotspot (AP) OR connect to a client network, never
both. NetworkManager scans even while in AP mode (it sees other SSIDs) but will NOT auto-switch off
an active hotspot to a higher-priority client. This always-on process closes that gap:

  - When tethering is enabled (param TetheringEnabled), the hotspot is kept up so it survives reboot
    ("perpetual tethering") and re-asserts itself if knocked down.
  - When a single named "priority" SSID (param TetheringPriorityWifi) comes into range AND we have a
    saved connection for it, we switch the radio over to that client network (dropping the hotspot).
    When it leaves range, we bring the hotspot back.

All the *decisions* live in the pure, unit-tested `decide(...)` in `network_arbiter.py`. This module
is only the thin, dangerous I/O shell: it polls NetworkManager via `nmcli`, feeds a snapshot to
`decide(...)`, and runs exactly the one nmcli action it returns. It is robust to nmcli failures
(log + continue) and idempotent (never re-`up`s the already-active connection).

Behavior-neutral when TetheringEnabled is unset/0 (its default): the only action it can take then is
tearing the hotspot down if it somehow finds it up, which matches the user's "tethering off" intent.
"""
import subprocess
import time
from typing import NoReturn

from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.system.networkd.network_arbiter import (
  HOTSPOT_CONNECTION_ID,
  decide,
  priority_connection_id,
)
from openpilot.system.networkd.lte_guard import decide_lte_guard

POLL_INTERVAL_S = 20.0
NMCLI_TIMEOUT_S = 15.0

LTE_CONNECTION_ID = "lte"
# the carrier/modem PDN throttle string NM logs when a speed test (or any rapid PDN churn) gets the
# IMEI flagged. NM then hammers retries ~1/s, which keeps the carrier timer from ever aging out, so
# LTE never recovers. The guard parks (autoconnect off + down) on this so the carrier timer ages out.
PDN_THROTTLE_TOKEN = "pdn-ipv4-call-throttled"


def _nmcli(args: list[str]) -> str | None:
  """Run an nmcli query/command. Returns stdout on success, None on any failure (logged)."""
  try:
    proc = subprocess.run(
      ["nmcli", *args],
      capture_output=True,
      text=True,
      timeout=NMCLI_TIMEOUT_S,
      check=False,
    )
  except (OSError, subprocess.TimeoutExpired):
    cloudlog.exception(f"network_arbiterd: nmcli {args} failed to run")
    return None
  if proc.returncode != 0:
    cloudlog.warning(f"network_arbiterd: nmcli {args} rc={proc.returncode} err={proc.stderr.strip()}")
    return None
  return proc.stdout


def _scan_ssids() -> list[str]:
  """SSIDs currently visible to NM. `nmcli -t -f SSID dev wifi list` (works even in AP mode)."""
  out = _nmcli(["-t", "-f", "SSID", "dev", "wifi", "list"])
  if out is None:
    return []
  return [line for line in (raw.strip() for raw in out.splitlines()) if line]


def _saved_connections() -> list[str]:
  """All NM connection ids that exist. `nmcli -t -f NAME con show`."""
  out = _nmcli(["-t", "-f", "NAME", "con", "show"])
  if out is None:
    return []
  return [line for line in (raw.strip() for raw in out.splitlines()) if line]


def _active_wifi_connection() -> str | None:
  """
  The NM connection id currently active on the wlan device, or None.

  `nmcli -t -f NAME,TYPE,DEVICE con show --active` gives active connections; we pick the wifi one.
  """
  out = _nmcli(["-t", "-f", "NAME,TYPE,DEVICE", "con", "show", "--active"])
  if out is None:
    return None
  for raw in out.splitlines():
    # fields are colon-separated: NAME:TYPE:DEVICE. NAME may itself contain escaped colons, but our
    # connection ids ("Hotspot", "openpilot connection <ssid>") and NM wifi type "802-11-wireless"
    # don't, so a simple split is sufficient here.
    parts = raw.split(":")
    if len(parts) < 2:
      continue
    name, conn_type = parts[0], parts[1]
    if "wireless" in conn_type:
      return name
  return None


def _apply(action: str, priority_ssid: str) -> None:
  """Run the one nmcli action chosen by decide(). All failures are logged, never raised."""
  if action == "noop":
    return

  if action == "up_priority":
    conn_id = priority_connection_id(priority_ssid.strip())
    cloudlog.info(f"network_arbiterd: priority wifi '{priority_ssid}' in range -> {conn_id} (dropping hotspot)")
    _nmcli(["con", "up", conn_id])
  elif action == "up_hotspot":
    cloudlog.info("network_arbiterd: bringing hotspot up")
    _nmcli(["con", "up", HOTSPOT_CONNECTION_ID])
  elif action == "down_hotspot":
    cloudlog.info("network_arbiterd: tethering disabled -> bringing hotspot down")
    _nmcli(["con", "down", HOTSPOT_CONNECTION_ID])
  else:
    cloudlog.error(f"network_arbiterd: unknown action {action!r}")


def _lte_has_ip() -> bool:
  """True if wwan0 has an IPv4 address (LTE data is up). `ip -4 -o addr show wwan0`."""
  try:
    proc = subprocess.run(["ip", "-4", "-o", "addr", "show", "wwan0"],
                          capture_output=True, text=True, timeout=NMCLI_TIMEOUT_S, check=False)
  except (OSError, subprocess.TimeoutExpired):
    return False
  return proc.returncode == 0 and "inet " in proc.stdout


def _lte_throttled_recently() -> bool:
  """True if NM logged the PDN throttle in the last minute (still hammering / freshly tripped).
  `journalctl -u NetworkManager --since -60s`."""
  try:
    proc = subprocess.run(
      ["sudo", "journalctl", "-u", "NetworkManager", "--no-pager", "--since", "-60s"],
      capture_output=True, text=True, timeout=NMCLI_TIMEOUT_S, check=False)
  except (OSError, subprocess.TimeoutExpired):
    return False
  return proc.returncode == 0 and PDN_THROTTLE_TOKEN in proc.stdout


def _park_lte() -> None:
  """Stop NM hammering the modem: autoconnect off + down. Lets the carrier PDN timer age out."""
  cloudlog.warning("network_arbiterd: LTE PDN-throttled -> parking lte (autoconnect off) to let carrier timer clear")
  _nmcli(["con", "modify", LTE_CONNECTION_ID, "connection.autoconnect", "no"])
  _nmcli(["con", "down", LTE_CONNECTION_ID])


def _unpark_lte() -> None:
  """Backoff elapsed: re-enable autoconnect + one clean bring-up attempt."""
  cloudlog.info("network_arbiterd: LTE backoff elapsed -> re-enabling autoconnect + single bring-up")
  _nmcli(["con", "modify", LTE_CONNECTION_ID, "connection.autoconnect", "yes"])
  _nmcli(["con", "up", LTE_CONNECTION_ID])


def main() -> NoReturn:
  params = Params()
  cloudlog.info("network_arbiterd: started")

  # LTE throttle-guard state (carried across loop iterations)
  lte_parked = False
  lte_parked_until = 0.0
  lte_throttle_count = 0

  while True:
    try:
      tethering_enabled = params.get_bool("TetheringEnabled")
      priority_ssid = params.get("TetheringPriorityWifi") or ""

      action = decide(
        tethering_enabled=tethering_enabled,
        priority_ssid=priority_ssid,
        scan_ssids=_scan_ssids(),
        saved_connections=_saved_connections(),
        current_active=_active_wifi_connection(),
      )
      _apply(action, priority_ssid)
    except Exception:
      # never let a transient error kill the supervisor
      cloudlog.exception("network_arbiterd: unhandled error in loop")

    # LTE PDN-throttle backoff guard (independent of the wifi arbitration above)
    try:
      lte_action, lte_parked, lte_parked_until, lte_throttle_count = decide_lte_guard(
        now=time.monotonic(),
        throttled=_lte_throttled_recently() if not lte_parked else False,  # don't probe while parked
        lte_has_ip=_lte_has_ip() if not lte_parked else False,
        parked=lte_parked,
        parked_until=lte_parked_until,
        throttle_count=lte_throttle_count,
      )
      if lte_action == "park":
        _park_lte()
      elif lte_action == "unpark":
        _unpark_lte()
    except Exception:
      cloudlog.exception("network_arbiterd: unhandled error in lte guard")

    time.sleep(POLL_INTERVAL_S)


if __name__ == "__main__":
  main()
