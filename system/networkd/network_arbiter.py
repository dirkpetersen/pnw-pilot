"""
network2xnor: pure arbitration logic for perpetual tethering + priority WiFi.

This module contains ONLY the decision function `decide(...)`. It performs no I/O — it takes a
snapshot of the world (params + NetworkManager scan/state) and returns exactly one action string.
The supervisor process (`network_arbiterd.py`) wraps it with the nmcli I/O.

Keeping the decision pure makes it trivially unit-testable and keeps the dangerous part (running
nmcli, dropping the hotspot) thin and obvious.

Actions:
  'up_priority'  -> bring the saved priority-wifi client connection up (drops the hotspot)
  'up_hotspot'   -> bring the Hotspot AP up
  'down_hotspot' -> tear the Hotspot AP down (tethering disabled but AP still up)
  'noop'         -> nothing to do; current state already matches the desired state
"""
from __future__ import annotations

# NM connection ids. The Hotspot connection is always named "Hotspot"; saved client networks are
# created by wifi_manager.connect_to_network as "openpilot connection <SSID>".
HOTSPOT_CONNECTION_ID = "Hotspot"


def priority_connection_id(ssid: str) -> str:
  """The NM connection id wifi_manager uses for a saved client network."""
  return f"openpilot connection {ssid}"


def decide(
  tethering_enabled: bool,
  priority_ssid: str,
  scan_ssids: list[str],
  saved_connections: list[str],
  current_active: str | None,
) -> str:
  """
  Decide the single nmcli action to take this tick.

  Args:
    tethering_enabled: value of the TetheringEnabled param.
    priority_ssid: value of the TetheringPriorityWifi param (blank if unset). Only THIS ssid may
                   interrupt the hotspot.
    scan_ssids: SSIDs currently visible to NM (`nmcli -t -f SSID dev wifi list`). NM scans even
                while in AP mode.
    saved_connections: NM connection ids that exist (`nmcli -t -f NAME con show`). Used to confirm
                       a saved client connection exists for the priority SSID before trying to
                       bring it up.
    current_active: NM connection id that is currently active on wlan0 (the Hotspot, an
                    "openpilot connection <ssid>", or None). Used to stay idempotent — we never
                    re-`up` what is already active.

  Returns one of: 'up_priority', 'up_hotspot', 'down_hotspot', 'noop'.
  """
  # --- Tethering OFF: only ever ensure the hotspot is DOWN. Never touch client wifi. ---
  if not tethering_enabled:
    if current_active == HOTSPOT_CONNECTION_ID:
      return "down_hotspot"
    return "noop"

  # --- Tethering ON ---
  priority_ssid = (priority_ssid or "").strip()
  priority_id = priority_connection_id(priority_ssid) if priority_ssid else None

  # A named priority SSID wins the radio whenever it is both in range AND has a saved connection.
  priority_available = bool(
    priority_ssid
    and priority_ssid in scan_ssids
    and priority_id in saved_connections
  )

  if priority_available:
    # WiFi wins. Connect to it (this drops the hotspot), unless already connected.
    if current_active == priority_id:
      return "noop"
    return "up_priority"

  # Priority SSID not in range (or none configured / not saved) -> hotspot should be up.
  if current_active == HOTSPOT_CONNECTION_ID:
    return "noop"
  return "up_hotspot"
