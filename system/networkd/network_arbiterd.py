#!/usr/bin/env python3
"""
network2xnor: arbitration supervisor for perpetual tethering + priority WiFi.

The comma 3X has ONE WiFi radio: it can run the hotspot (AP) OR connect to a client network, never
both. NetworkManager scans even while in AP mode (it sees other SSIDs) but will NOT auto-switch off
an active hotspot to a higher-priority client. This always-on process closes that gap:

  - When tethering is enabled (param TetheringEnabled), the hotspot is kept up so it survives reboot
    ("perpetual tethering") and re-asserts itself if knocked down. It ALSO installs the NAT (ip_forward
    + masquerade out LTE) whenever it raises the hotspot, so tethered clients actually get internet —
    the UI toggle isn't the only path that raises the AP.
  - When a single named "priority" SSID (param TetheringPriorityWifi) comes into range AND we have a
    saved connection for it, we switch the radio over to that client network (dropping the hotspot).
    When it leaves range, we bring the hotspot back.

GEO-GATED SCANNING: while tethering, we'd otherwise scan for the priority SSID every cycle — but on a
single radio a WiFi scan forces the radio off-channel (NetworkManager's own maintainer documents that
scans induce lag/drops), competing with the hotspot. There's no point scanning for home WiFi when
you're nowhere near home. So we record the home location (GPS, where the priority WiFi lives) and only
scan/switch when within HOME_GEOFENCE_M of it. Fail-open: if home isn't learned yet or GPS is missing,
we scan as before.

The *decisions* live in the pure, unit-tested `decide(...)` (network_arbiter.py) + `near_home(...)`
(geo_gate.py). This module is the thin I/O shell: it polls NM via `nmcli`, feeds snapshots to the pure
deciders, and runs the chosen action. Robust to nmcli failures (log + continue), idempotent.

IMPORTANT — netplan/keyfile crash: on this AGNOS the gsm profiles (lte/esim) are NETPLAN-managed.
`nmcli con modify` on a gsm profile triggers NM's keyfile-writer assertion -> NetworkManager ABRT
(crash loop) -> hardwared's network read goes blank. So we NEVER `nmcli con modify` a gsm profile here.
Persistence of the LTE profile (blank APN, autoconnect) is handled in /data/etc/netplan/ + the GsmApn
param, NOT by this daemon. The throttle guard parks LTE via the MODEM (mmcli --disable), not nmcli.

Behavior-neutral when TetheringEnabled is unset/0 (its default).
"""
import json
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
from openpilot.system.networkd.geo_gate import near_home

POLL_INTERVAL_S = 20.0
NMCLI_TIMEOUT_S = 15.0

LTE_CONNECTION_ID = "lte"
HOTSPOT_SUBNET = "192.168.43.0/24"   # AP client subnet, masqueraded out the LTE uplink
# the carrier/modem PDN throttle string NM logs when a speed test (or any rapid PDN churn) gets the
# IMEI flagged. NM then hammers retries ~1/s, which keeps the carrier timer from ever aging out.
PDN_THROTTLE_TOKEN = "pdn-ipv4-call-throttled"


def _nmcli(args: list[str]) -> str | None:
  """Run an nmcli query/command. Returns stdout on success, None on any failure (logged)."""
  try:
    proc = subprocess.run(["nmcli", *args], capture_output=True, text=True,
                          timeout=NMCLI_TIMEOUT_S, check=False)
  except (OSError, subprocess.TimeoutExpired):
    cloudlog.exception(f"network_arbiterd: nmcli {args} failed to run")
    return None
  if proc.returncode != 0:
    cloudlog.warning(f"network_arbiterd: nmcli {args} rc={proc.returncode} err={proc.stderr.strip()}")
    return None
  return proc.stdout


def _run(args: list[str]) -> subprocess.CompletedProcess | None:
  """Run an arbitrary command, swallow exec errors. Returns the CompletedProcess or None."""
  try:
    return subprocess.run(args, capture_output=True, text=True, timeout=NMCLI_TIMEOUT_S, check=False)
  except (OSError, subprocess.TimeoutExpired):
    cloudlog.exception(f"network_arbiterd: {args[:2]} failed to run")
    return None


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
  """The NM connection id currently active on the wlan device, or None."""
  out = _nmcli(["-t", "-f", "NAME,TYPE,DEVICE", "con", "show", "--active"])
  if out is None:
    return None
  for raw in out.splitlines():
    parts = raw.split(":")
    if len(parts) < 2:
      continue
    name, conn_type = parts[0], parts[1]
    if "wireless" in conn_type:
      return name
  return None


# --- hotspot NAT (so the arbiter-raised AP actually passes traffic, like the UI toggle does) --------

def _set_hotspot_nat(enabled: bool) -> None:
  """Forward + masquerade the AP subnet out the LTE uplink so tethered clients get internet. Mirrors
  wifi_manager._set_tethering_nat — the arbiter raises the hotspot on boot / after a WiFi-drop, paths
  the UI toggle never runs, so without this the hotspot is up but 'nothing happens' for clients.
  AGNOS uses iptables-LEGACY (the nft binary lacks the MASQUERADE module). Idempotent."""
  _run(["sudo", "sysctl", "-w", f"net.ipv4.ip_forward={1 if enabled else 0}"])
  rules = [
    ["-t", "nat", "POSTROUTING", "-s", HOTSPOT_SUBNET, "!", "-d", HOTSPOT_SUBNET, "-j", "MASQUERADE"],
    ["FORWARD", "-s", HOTSPOT_SUBNET, "-j", "ACCEPT"],
    ["FORWARD", "-d", HOTSPOT_SUBNET, "-j", "ACCEPT"],
  ]
  for rule in rules:
    pre = rule[:2] if rule[0] == "-t" else []
    chain = rule[2] if rule[0] == "-t" else rule[0]
    rest = rule[3:] if rule[0] == "-t" else rule[1:]
    base = ["sudo", "iptables-legacy", *pre]
    _run([*base, "-D", chain, *rest])           # delete first (idempotent)
    if enabled:
      _run([*base, "-I", chain, *rest])


def _apply(action: str, priority_ssid: str) -> None:
  """Run the one action chosen by decide(). All failures are logged, never raised."""
  if action == "noop":
    return
  if action == "up_priority":
    conn_id = priority_connection_id(priority_ssid.strip())
    cloudlog.info(f"network_arbiterd: priority wifi '{priority_ssid}' in range -> {conn_id} (dropping hotspot)")
    _set_hotspot_nat(False)                       # hotspot going away -> tear down its NAT
    _nmcli(["con", "up", conn_id])
  elif action == "up_hotspot":
    cloudlog.info("network_arbiterd: bringing hotspot up (+NAT)")
    _set_hotspot_nat(True)                         # install NAT BEFORE the AP so the first client packet routes
    _nmcli(["con", "up", HOTSPOT_CONNECTION_ID])
  elif action == "down_hotspot":
    cloudlog.info("network_arbiterd: tethering disabled -> bringing hotspot down (-NAT)")
    _nmcli(["con", "down", HOTSPOT_CONNECTION_ID])
    _set_hotspot_nat(False)
  else:
    cloudlog.error(f"network_arbiterd: unknown action {action!r}")


# --- GPS / home-location for the geo-gate ------------------------------------------------------------

def _read_gps(params: Params, mem_params: Params | None) -> tuple[float, float] | None:
  """Current (lat, lon) from LastGPSPosition (JSON {latitude,longitude}), or None. mapd writes it to
  the in-memory store; locationd to the persistent one — try both."""
  for store in (mem_params, params):
    if store is None:
      continue
    try:
      raw = store.get("LastGPSPosition")
      if not raw:
        continue
      d = json.loads(raw) if isinstance(raw, (str, bytes, bytearray)) else raw
      return float(d["latitude"]), float(d["longitude"])
    except Exception:
      continue
  return None


def _read_home(params: Params) -> tuple[float, float] | None:
  """Saved home location (TetheringHomeLocation JSON [lat, lon]), or None."""
  try:
    raw = params.get("TetheringHomeLocation")
    if not raw:
      return None
    d = json.loads(raw)
    return float(d[0]), float(d[1])
  except Exception:
    return None


def _save_home(params: Params, lat: float, lon: float) -> None:
  """Remember where the priority WiFi lives (we're connected to it right now == home)."""
  try:
    params.put("TetheringHomeLocation", json.dumps([round(lat, 6), round(lon, 6)]))
  except Exception:
    cloudlog.exception("network_arbiterd: failed to save home location")


# --- LTE PDN-throttle park/unpark (MODEM-level — never nmcli con modify a netplan gsm profile) -------

def _modem_index() -> str | None:
  """The modem's current ModemManager index (it RE-ENUMERATES on reset, so resolve it live)."""
  p = _run(["mmcli", "-L"])
  if p is None or p.returncode != 0:
    return None
  import re
  m = re.search(r"Modem/(\d+)", p.stdout)
  return m.group(1) if m else None


def _lte_throttled_recently() -> bool:
  p = _run(["sudo", "journalctl", "-u", "NetworkManager", "--no-pager", "--since", "-60s"])
  return p is not None and p.returncode == 0 and PDN_THROTTLE_TOKEN in p.stdout


def _lte_has_ip() -> bool:
  p = _run(["ip", "-4", "-o", "addr", "show", "wwan0"])
  return p is not None and p.returncode == 0 and "inet " in p.stdout


def _park_lte() -> None:
  """Stop the modem hammering the throttled PDN: disable the modem RF (mmcli) so it stops requesting a
  bearer and the carrier's tower-side timer can age out. NM can't activate a disabled modem, so the
  ~1/s retry loop stops — WITHOUT any nmcli con modify (which would crash NM on a netplan gsm profile)."""
  idx = _modem_index()
  cloudlog.warning(f"network_arbiterd: LTE PDN-throttled -> disabling modem {idx} to let carrier timer clear")
  _nmcli(["con", "down", LTE_CONNECTION_ID])
  if idx is not None:
    _run(["mmcli", "-m", idx, "--disable"])


def _unpark_lte() -> None:
  """Backoff elapsed: re-enable the modem RF; NM autoconnect (set in netplan) brings lte back up."""
  idx = _modem_index()
  cloudlog.info(f"network_arbiterd: LTE backoff elapsed -> re-enabling modem {idx}")
  if idx is not None:
    _run(["mmcli", "-m", idx, "--enable"])
  _nmcli(["con", "up", LTE_CONNECTION_ID])


def main() -> NoReturn:
  import platform
  params = Params()
  try:
    mem_params = Params("/dev/shm/params") if platform.system() != "Darwin" else None
  except Exception:
    mem_params = None
  cloudlog.info("network_arbiterd: started")

  lte_parked = False
  lte_parked_until = 0.0
  lte_throttle_count = 0

  while True:
    try:
      tethering_enabled = params.get_bool("TetheringEnabled")
      priority_ssid = (params.get("TetheringPriorityWifi") or "").strip()
      current_active = _active_wifi_connection()

      # auto-learn the home location: if we're connected to the priority WiFi, that spot IS home.
      if priority_ssid and current_active == priority_connection_id(priority_ssid):
        gps = _read_gps(params, mem_params)
        if gps is not None:
          _save_home(params, *gps)

      # geo-gate the scan: only scan for the priority SSID when near the learned home location (a WiFi
      # scan on the single radio competes with the hotspot). Fail-open when home/GPS unknown.
      allow_scan = near_home(_read_home(params), _read_gps(params, mem_params))
      scan = _scan_ssids() if (tethering_enabled and priority_ssid and allow_scan) else []

      action = decide(
        tethering_enabled=tethering_enabled,
        priority_ssid=priority_ssid,
        scan_ssids=scan,
        saved_connections=_saved_connections(),
        current_active=current_active,
      )
      _apply(action, priority_ssid)
    except Exception:
      cloudlog.exception("network_arbiterd: unhandled error in loop")

    # LTE PDN-throttle backoff guard
    try:
      lte_action, lte_parked, lte_parked_until, lte_throttle_count = decide_lte_guard(
        now=time.monotonic(),
        throttled=_lte_throttled_recently() if not lte_parked else False,
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
