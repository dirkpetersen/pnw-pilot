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
import threading
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
from openpilot.system.networkd.geo_gate import near_any_home, haversine_m
from openpilot.system.networkd import priority_networks as pn
from openpilot.system.networkd import captive_portal

POLL_INTERVAL_S = 20.0
NMCLI_TIMEOUT_S = 15.0
SIGNAL_EVERY_N = 3   # run LTE signal logging only every Nth loop (~60s) so its mmcli calls can't repeatedly stall WiFi recovery
LEARN_MIN_MOVE_M = 50.0   # only re-write a learned home location if GPS moved more than this (flash-wear guard)
PORTAL_MAX_TRIES = 6   # max captive-portal accept() attempts per SSID session (bounds loop time; resets on leaving)

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


# NOTE: the single-home _read_home/_save_home helpers were removed when this daemon moved to the
# multi-location model — per-entry locations now live in TetheringPriorityNetworks (priority_networks
# .py), auto-learned inline in main(). The legacy TetheringHomeLocation param is still read (only) by
# priority_networks.parse() for one-time migration of an old single-home setup.


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


# --- LTE signal-strength logging (network2xnor: log to qlog on change so slow spots are visible) -----

# coarse "bars" buckets from RSSI dBm (matches the spirit of deviceState.networkStrength on screen).
def _bars_from_dbm(dbm: float | None) -> int | None:
  if dbm is None:
    return None
  if dbm >= -75:
    return 4
  if dbm >= -85:
    return 3
  if dbm >= -95:
    return 2
  if dbm >= -105:
    return 1
  return 0


def _read_lte_operator(idx: str | None) -> str | None:
  """Carrier/provider name via `mmcli -m <idx> -J` (modem.3gpp.operator-name). None on failure.
  Changes rarely (roaming / tower handoff), so the caller caches it and only re-reads occasionally."""
  if idx is None:
    return None
  p = _run(["mmcli", "-m", idx, "-J"])
  if p is None or p.returncode != 0 or not p.stdout.strip():
    return None
  try:
    tgpp = json.loads(p.stdout).get("modem", {}).get("3gpp", {})
    op = (tgpp.get("operator-name") or "").strip()
    return op or None
  except (ValueError, AttributeError):
    return None


_signal_setup_done_for: str | None = None   # modem idx we've already enabled signal polling on


def _ensure_signal_setup(idx: str | None) -> None:
  """Enable the modem's periodic signal sampling ONCE per modem (not every loop).

  Hammering `mmcli --signal-setup` every cycle can wedge ModemManager during the initial carrier
  attach / PDN bearer activation (and blocks the loop up to NMCLI_TIMEOUT_S each time). The modem
  re-enumerates on reset, so we re-run setup only when the index changes. Mark done ONLY on success,
  so a failed setup during the busy boot/attach window is retried next time (else signal-get would
  silently return nothing forever for this session)."""
  global _signal_setup_done_for
  if idx is None or idx == _signal_setup_done_for:
    return
  p = _run(["mmcli", "-m", idx, "--signal-setup", "30"])   # 30 s polling; we only read on change anyway
  if p is not None and p.returncode == 0:
    _signal_setup_done_for = idx


def _read_lte_signal(idx: str | None) -> dict | None:
  """Modem signal metrics via `mmcli -m <idx> --signal-get -J` (RSSI/RSRP/RSRQ/SNR dBm). None on fail.
  Assumes signal polling was already enabled once via _ensure_signal_setup()."""
  if idx is None:
    return None
  p = _run(["mmcli", "-m", idx, "--signal-get", "-J"])
  if p is None or p.returncode != 0 or not p.stdout.strip():
    return None
  try:
    data = json.loads(p.stdout).get("modem", {}).get("signal", {})
  except (ValueError, AttributeError):
    return None

  def _g(*path):
    cur = data
    for k in path:
      if not isinstance(cur, dict):
        return None
      cur = cur.get(k)
    try:
      return round(float(cur), 1)
    except (TypeError, ValueError):
      return None

  # try the common access techs in order (lte, then 5g, then umts), first with a value wins.
  out: dict = {"access_tech": None}
  for tech in ("lte", "5g", "umts", "gsm"):
    rssi = _g(tech, "rssi")
    rsrp = _g(tech, "rsrp")
    if rssi is not None or rsrp is not None:
      out = {"access_tech": tech, "rssi": rssi, "rsrp": rsrp,
             "rsrq": _g(tech, "rsrq"), "snr": _g(tech, "snr") or _g(tech, "sinr")}
      break
  if out["access_tech"] is None:
    return None
  out["bars"] = _bars_from_dbm(out.get("rssi") if out.get("rssi") is not None else out.get("rsrp"))
  return out


def _log_signal_if_changed(idx: str | None, last: dict | None, operator: str | None) -> dict | None:
  """Read the modem signal and emit a qlog event ONLY when bars/dBm/operator changed since last tick.
  Returns the new reading (or `last` unchanged if nothing to report). `operator` = carrier name."""
  sig = _read_lte_signal(idx)
  if sig is None:
    return last
  sig["operator"] = operator
  # consider it "changed" if operator/access-tech changes, or any dBm moved >= 2. NOTE: we deliberately
  # do NOT treat a `bars` flip on its own as a change — bars is a bucketed dBm, so a 0.1 dBm jitter
  # across a bucket boundary (e.g. -75.0 -> -75.1) would flip bars every tick and spam qlog. Gating on
  # the >= 2 dBm move gives bars hysteresis for free: bars only effectively re-logs once the underlying
  # dBm has genuinely moved past the threshold.
  def _moved(a, b, key, thresh=2.0):
    va, vb = (a or {}).get(key), (b or {}).get(key)
    if va is None or vb is None:
      return va is not vb
    return abs(va - vb) >= thresh
  changed = (last is None
             or sig.get("access_tech") != (last or {}).get("access_tech")
             or sig.get("operator") != (last or {}).get("operator")
             or any(_moved(sig, last, k) for k in ("rssi", "rsrp", "rsrq", "snr")))
  if changed:
    cloudlog.event("network2xnor_lte_signal", operator=sig.get("operator"), bars=sig.get("bars"),
                   access_tech=sig.get("access_tech"), rssi=sig.get("rssi"), rsrp=sig.get("rsrp"),
                   rsrq=sig.get("rsrq"), snr=sig.get("snr"))
    return sig
  return last


def _has_connectivity() -> bool:
  """NM's view of internet reachability (used to decide whether a captive portal still needs poking)."""
  out = _nmcli(["-t", "-f", "CONNECTIVITY", "general"])
  return out is not None and out.strip() == "full"


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
  last_signal: dict | None = None
  operator: str | None = None
  signal_tick = 0
  portal_done_for: str | None = None   # ssid we've already satisfied a captive portal for this session
  portal_tries: dict[str, int] = {}    # ssid -> accept() attempts this session (bounded by PORTAL_MAX_TRIES)
  portal_result: dict[str, bool] = {}  # ssid -> last accept() online result (set by the worker thread)
  portal_thread: threading.Thread | None = None  # captive-portal accept() runs here so it NEVER blocks this loop

  while True:
    try:
      tethering_enabled = params.get_bool("TetheringEnabled")
      # network2xnor (multi-location): the list param is authoritative; fall back to the legacy single
      # params so existing setups keep working (parse() migrates them transparently).
      nets = pn.parse(params.get("TetheringPriorityNetworks"),
                      legacy_ssid=(params.get("TetheringPriorityWifi") or ""),
                      legacy_home_raw=params.get("TetheringHomeLocation"))
      net_ssids = pn.ssids(nets)
      current_active = _active_wifi_connection()
      gps = _read_gps(params, mem_params)

      # auto-learn each network's location: if we're connected to one of OUR priority SSIDs right now,
      # this spot IS that network's geofence center -> update only that entry.
      # FLASH-WEAR GUARD: GPS jitters every read, so only WRITE the param when the fix has actually
      # moved meaningfully (> LEARN_MIN_MOVE_M) from the stored location, or it was never learned.
      # Otherwise this would params.put() a new JSON blob every 20 s forever, wearing the flash.
      if gps is not None and current_active:
        for e in nets:
          if current_active == priority_connection_id(e["ssid"]):
            old = (e.get("lat"), e.get("lon"))
            moved = old[0] is None or old[1] is None or \
              haversine_m(old[0], old[1], gps[0], gps[1]) > LEARN_MIN_MOVE_M
            if moved:
              e["lat"], e["lon"] = round(gps[0], 6), round(gps[1], 6)
              params.put("TetheringPriorityNetworks", pn.dumps(nets))
            break

      # geo-gate: scan only when near ANY learned location (a scan competes with the hotspot on the
      # single radio). Fail-open when no locations learned yet or GPS missing.
      # ESCAPE HATCH (fix for the stale-GPS lockout): scan whenever we are NOT on a real client WiFi —
      # disconnected (current_active is None) OR sitting on our own hotspot. A stale "far from home"
      # GPS fix must never suppress the scan in those states, or the device can never find its home
      # WiFi to recover and sits stranded forever. Geo-gating only avoids needless scans once we're
      # already on a real client network; otherwise finding WiFi wins.
      on_client_wifi = current_active is not None and current_active != HOTSPOT_CONNECTION_ID
      allow_scan = (not on_client_wifi) or near_any_home(pn.locations(nets), gps)
      scan = _scan_ssids() if (tethering_enabled and net_ssids and allow_scan) else []

      # STICKY ACTIVE CONNECTION: if we're ALREADY connected to one of our priority SSIDs, keep it —
      # do NOT require it to re-appear in this tick's scan. Otherwise, when the geo-gate pauses
      # scanning (or a scan simply omits the connected AP, which is common), chosen_ssid would go ""
      # and decide() would tear down a perfectly good client WiFi to raise the hotspot. We seed the
      # scan+chosen with the active network so decide() sees it as available and returns noop.
      active_entry = None
      if on_client_wifi:
        for e in nets:
          if current_active == priority_connection_id(e["ssid"]):
            active_entry = e
            break
      if active_entry is not None and active_entry["ssid"] not in scan:
        scan = [*scan, active_entry["ssid"]]

      # pick the first configured network that is both in range and has a saved NM connection.
      chosen = pn.select_available(nets, scan, _saved_connections(), priority_connection_id)
      chosen_ssid = chosen["ssid"] if chosen else ""

      action = decide(
        tethering_enabled=tethering_enabled,
        priority_ssid=chosen_ssid,
        scan_ssids=scan,
        saved_connections=_saved_connections(),
        current_active=current_active,
      )
      _apply(action, chosen_ssid)

      # captive-portal auto-accept: when we're sitting on one of OUR SSIDs that declares a portal
      # handler and we don't yet have full connectivity, POST its accept form (once per session).
      active_portal_entry = pn.entry_for_ssid(
        nets, current_active.replace("openpilot connection ", "")) if current_active else None
      if active_portal_entry and active_portal_entry.get("portal"):
        ssid = active_portal_entry["ssid"]
        # Poke the portal even when NM reports global connectivity "full": the comma keeps LTE up
        # alongside WiFi, so LTE's connectivity masks a captive WiFi (the default route) and the old
        # `if not _has_connectivity()` guard never fired here. accept() is HTTP-ONLY — it never touches
        # NM, the LTE connection, the hotspot, or routing.
        #
        # CRITICAL: run accept() in a DAEMON THREAD, never inline. A DNS-blocking portal can make its
        # GETs time out (~tens of seconds), and blocking THIS loop would delay LTE-signal logging and
        # tethering/hotspot NAT upkeep. The thread stores its result in portal_result; we read it next
        # loop. One thread at a time; bounded to PORTAL_MAX_TRIES per session.
        if portal_result.get(ssid):                # worker confirmed we're actually online -> done
          portal_done_for = ssid
        if (portal_done_for != ssid and portal_tries.get(ssid, 0) < PORTAL_MAX_TRIES
            and (portal_thread is None or not portal_thread.is_alive())):
          portal_tries[ssid] = portal_tries.get(ssid, 0) + 1
          cloudlog.event("network2xnor_portal_try", ssid=ssid, portal=active_portal_entry["portal"],
                         current_active=current_active, connectivity_full=_has_connectivity(),
                         attempt=portal_tries[ssid])

          def _poke(handler=active_portal_entry["portal"], sid=ssid):
            try:
              portal_result[sid] = bool(captive_portal.accept(handler, already_online=False))
            except Exception:
              portal_result[sid] = False

          portal_thread = threading.Thread(target=_poke, name="captive_portal", daemon=True)
          portal_thread.start()
      else:
        # not on a configured portal SSID -> reset session state so re-entry re-tries cleanly. A still-
        # running daemon thread just finishes its HTTP harmlessly (it only writes its result file).
        # Mutate (.clear()) rather than rebind so the worker thread's closure always shares this object.
        if portal_done_for is not None or portal_tries or portal_result:
          portal_done_for = None
          portal_tries.clear()
          portal_result.clear()
    except Exception:
      cloudlog.exception("network_arbiterd: unhandled error in loop")

    # LTE signal-strength logging (operator + bars + dBm, only on change -> qlog timeline of slow
    # spots). Runs on a SLOWER cadence than the arbiter loop (every SIGNAL_EVERY_N ticks ~= 60 s), so
    # the up-to-4 sequential mmcli calls here can't repeatedly stall the WiFi-recovery logic that runs
    # earlier in the loop (each _run has a 15 s timeout). Operator name changes rarely -> re-read it
    # only occasionally. Skipped entirely while LTE is parked (modem RF off) — reads would just block.
    signal_tick += 1
    if not lte_parked and signal_tick % SIGNAL_EVERY_N == 0:
      try:
        idx = _modem_index()
        _ensure_signal_setup(idx)   # once per modem, NOT every loop (avoid wedging ModemManager)
        if operator is None or signal_tick % (SIGNAL_EVERY_N * 10) == 0:
          operator = _read_lte_operator(idx)
        last_signal = _log_signal_if_changed(idx, last_signal, operator)
      except Exception:
        cloudlog.exception("network_arbiterd: unhandled error in signal logging")

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
