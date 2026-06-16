# NETWORK2XNOR — Perpetual Tethering + Priority WiFi

Branch: `network2xnor` (off the tethering-fix lineage on `xnor/openpilot`).

## What this does

The comma 3X has **one WiFi radio**: it can run the hotspot (AP) **or** connect to a client network,
never both. NetworkManager (NM) scans even while in AP mode (it sees other SSIDs) but does **not**
auto-switch off an active hotspot to a higher-priority client. This feature adds two behaviors on top
of the existing tethering fixes (NAT-before-AP, autoconnect handling, blank-APN auto-negotiation):

1. **Perpetual tethering** — toggling "Enable Tethering" now also persists the intent to a param
   (`TetheringEnabled`). An always-on supervisor re-asserts the hotspot, so it survives reboot and
   comes back if knocked down.
2. **Priority WiFi over tethering** — a single named SSID (param `TetheringPriorityWifi`) is allowed
   to interrupt the hotspot. When that SSID is in range **and** has a saved connection, the radio
   switches to it (dropping the AP); when it leaves range, the hotspot comes back. Only that one
   named SSID can interrupt the hotspot — every other network is ignored.

**Behavior-neutral when off:** `TetheringEnabled` defaults to `0` and `TetheringPriorityWifi`
defaults to blank. With tethering off the supervisor's only possible action is tearing the hotspot
down if it somehow finds it up — which matches "tethering off". No panda / safety code is touched.

## Params (additive, in `common/params_keys.h`)

| Key | Type | Default | Meaning |
|-----|------|---------|---------|
| `TetheringEnabled` | `BOOL` | `"0"` | User's tethering intent; written by the UI tethering toggle. The supervisor keeps the hotspot up when this is set. |
| `TetheringPriorityWifi` | `STRING` | `""` | SSID that may interrupt the hotspot. Blank disables priority-switching. |

Both are `PERSISTENT` so they survive reboot.

## Components

### Pure decision: `system/networkd/network_arbiter.py`
`decide(tethering_enabled, priority_ssid, scan_ssids, saved_connections, current_active) -> str`
returns exactly one of `'up_priority' | 'up_hotspot' | 'down_hotspot' | 'noop'`. No I/O — fully
unit-tested in `system/networkd/tests/test_network_arbiter.py` (off / on+priority-in-range /
on+priority-absent / priority-not-saved / already-on-correct → noop / blank priority / only the named
SSID interrupts / switch from a wrong client). Logic:
- tethering **off** → `down_hotspot` if the hotspot is the active connection, else `noop` (never
  touches client wifi).
- tethering **on**, priority SSID **in `scan_ssids`** and `openpilot connection <ssid>` **in
  `saved_connections`** → `up_priority` (unless already active → `noop`).
- otherwise → `up_hotspot` (unless the hotspot is already active → `noop`).

### Supervisor: `system/networkd/network_arbiterd.py`
Always-on `PythonProcess` (registered in `system/manager/process_config.py` as `network_arbiterd`,
`always_run`, `enabled=TICI`). Every ~20 s it builds a snapshot from `nmcli`:
- `nmcli -t -f SSID dev wifi list` → visible SSIDs
- `nmcli -t -f NAME con show` → saved connection ids
- `nmcli -t -f NAME,TYPE,DEVICE con show --active` → the active wifi connection id

…calls `decide(...)`, and runs the single resulting action
(`nmcli con up "openpilot connection <ssid>"` / `nmcli con up Hotspot` / `nmcli con down Hotspot`).
Every nmcli call is wrapped: failures and timeouts are logged via `cloudlog` and the loop continues.
It is idempotent — it never re-`up`s the already-active connection.

NM connection ids (verified on device): the hotspot is `Hotspot`; saved client networks created by
`wifi_manager.connect_to_network` are `openpilot connection <SSID>`.

### UI
- `system/ui/widgets/network.py` (main raylib UI, `AdvancedNetworkSettings`): new ListItem
  **"Priority WiFi over tethering"** directly below "Tethering Password", following the exact
  ButtonAction + keyboard-dialog pattern. Blank input clears the param.
- `selfdrive/ui/mici/layouts/settings/network/network_layout.py` (mici variant): same field added
  below the tethering-password button using `BigButton` + `BigInputDialog`.
- `system/ui/lib/wifi_manager.py`: `set_tethering_active()` now persists `TetheringEnabled` (the
  perpetual-tethering intent) before the existing NAT/AP worker thread runs.

## Deploy (file overlay to `/data/openpilot`)

The car runs openpilot as a **file overlay**, not a git checkout. Copy the changed files in place,
register the new manager process, clear pyc, rebuild, set persistence guards, restart.

Files to overlay (relative to repo root):
```
common/params_keys.h
system/networkd/__init__.py
system/networkd/network_arbiter.py
system/networkd/network_arbiterd.py
system/manager/process_config.py
system/ui/widgets/network.py
system/ui/lib/wifi_manager.py
selfdrive/ui/mici/layouts/settings/network/network_layout.py
```
(`system/networkd/tests/` is dev-only; no need to ship it.)

On device:
```bash
source /usr/local/venv/bin/activate
export PYTHONPATH=/data/openpilot:/data/openpilot/opendbc_repo
# params_keys.h changed -> rebuild the params extension so TetheringEnabled/TetheringPriorityWifi register
PATH=/usr/local/venv/bin:$PATH scons -u -j$(nproc) common/params_pyx.so
find /data/openpilot -name '*.pyc' -delete
```

Persistence guards (REQUIRED so the reboot overlay-swap doesn't wipe the overlay / reflash panda):
```bash
sudo rm -rf /data/safe_staging/finalized
touch -d "2020-01-01" /data/openpilot/.overlay_init
touch /data/openpilot/prebuilt
# DisableUpdates=1 (param + UI "Allow auto updates" = OFF)
```
Then restart: `sudo systemctl restart comma` (or reboot).

## On-device validation (NOT yet done)

1. **Process up:** `network_arbiterd` appears in manager (`cat /dev/shm/params/...` not needed —
   check the manager process list / `ps aux | grep network_arbiterd`). With tethering off, confirm it
   is a no-op (does not bring any AP up). Watch `cloudlog` for `network_arbiterd: started`.
2. **Param wiring:** toggle "Enable Tethering" in Settings → Network → Advanced; confirm
   `Params().get_bool("TetheringEnabled")` flips, AP comes up, clients get internet.
3. **Perpetual tethering:** with tethering on, reboot; confirm the hotspot auto-starts.
4. **Priority field:** set "Priority WiFi over tethering" to a known home SSID that the device has a
   saved `openpilot connection <SSID>` for; confirm `TetheringPriorityWifi` is written.
5. **Switch-to-wifi:** with tethering on + priority set, bring that SSID in range → device should
   `nmcli con up "openpilot connection <SSID>"` and drop the AP within ~20 s.
6. **Switch-back:** take that SSID out of range → hotspot should come back within ~20 s.
7. **Isolation:** confirm a *different* in-range saved network does NOT drop the hotspot.
8. **Blank clears:** clear the priority field → param removed; only the hotspot runs.

## Risks / TODOs

- **`nmcli` field parsing:** `_active_wifi_connection()` splits `NAME:TYPE:DEVICE` on `:` and matches
  the wifi row by `"wireless" in TYPE`. Our connection ids contain no colons, so this is safe here,
  but a connection id with an escaped colon would mis-parse. Low risk given the fixed id scheme.
- **20 s latency:** the switch is poll-driven (`POLL_INTERVAL_S = 20`), so expect up to ~20 s before a
  priority network is picked up or the hotspot returns. Tunable.
- **`enabled=TICI`:** the supervisor only runs on device hardware (it shells out to `nmcli`), matching
  the other device-only processes. On PC it is disabled.
- **Interaction with NM autoconnect:** the existing tethering fix toggles other networks'
  autoconnect. The supervisor's explicit `nmcli con up` switches cleanly in both directions
  (verified manually per HANDOFF), but the combined long-running behavior has **not** been validated
  on device yet.
- **Hotspot id assumption:** assumes the AP connection is literally named `Hotspot`. If a future NM
  config renames it, `HOTSPOT_CONNECTION_ID` must be updated.
- Not yet driven / not yet validated live — code + unit tests only.

## Deviations from the original spec

- Supervisor module is `system/networkd/network_arbiterd.py` (the `*d` daemon naming convention),
  with the pure logic kept in `network_arbiter.py` as specified.
- Registered with `enabled=TICI` (not unconditional `always_run`) so it doesn't try to shell out to
  `nmcli` on a PC/CI host; it is still `always_run` (on+offroad) on the device.

---

## 2026-06-16 additions (live debugging on the 3X — DEPLOYED + reboot-verified)

A long on-device session reshaped this branch. New pieces:

### 1. Arbiter installs the hotspot NAT (`_set_hotspot_nat`)
The UI toggle (`wifi_manager.set_tethering_active`) installed `ip_forward` + an iptables-LEGACY
masquerade of the AP subnet (`192.168.43.0/24`) out the LTE uplink — but the **arbiter** raises the
hotspot on boot / after a WiFi-drop, paths the UI toggle never runs. Without NAT, clients got an IP but
**no internet ("nothing happens")**. `_apply()` now installs NAT on `up_hotspot` and tears it down on
`up_priority`/`down_hotspot`. Uses `iptables-legacy` (the nft binary lacks the MASQUERADE module on
AGNOS). **Test data with TCP — ICMP is carrier-blocked, so ping always shows 100% loss.**

### 2. GPS geo-gated scanning (`geo_gate.py` + `TetheringHomeLocation` + "Set Home Location" button)
A WiFi scan forces the single radio off-channel (competes with the hotspot). So while tethering we only
scan for the priority SSID when **near home**: pure `near_home()` (haversine, 250 m geofence) gates the
scan in `network_arbiterd`. Home is **auto-learned** (GPS captured whenever connected to the priority
WiFi) and/or set via the new **"Set Home Location"** button under "Priority WiFi over tethering"
(`network.py`, captures current GPS — press outdoors). Fail-open: unknown home/GPS → scan as before.

### 3. LTE PDN-throttle guard is now MODEM-level (`mmcli`, not nmcli)
An Ookla/burst speed test trips `pdn-ipv4-call-throttled`; NM's ~1/s retry perpetuates it. The guard
(`lte_guard.decide_lte_guard`, exponential backoff 30s→2m→5m→10m) now parks LTE via
`mmcli -m <idx> --disable` (RF off → carrier timer ages out) and unparks via `--enable` — **NOT**
`nmcli con modify`, see #4.

### 4. gsm-profile enforcer REMOVED (it crashed NetworkManager)
The earlier `gsm_profile.py` enforcer ran `nmcli con modify lte/esim ...` every loop. On this AGNOS the
gsm profiles are **netplan-managed**, and `nmcli con modify` on them trips NM's keyfile-writer
assertion → **NetworkManager ABRT crash-loop** → hardwared's network read goes blank (blank signal
bars). Removed entirely. **Rule: never `nmcli con modify` a gsm profile; `con up/down` is fine.**

### LTE config persistence (the right way)
LTE config (blank APN for carrier auto-negotiation, autoconnect) persists in **`/data/etc/netplan/`**
(bind-mounted over `/etc/netplan` on boot by `fs_setup.sh`) + the `GsmApn` param. Editing `/etc/netplan`
directly reverts on reboot. The phantom `esim` profile (SIM-pinned to an absent eSIM, was winning the
autoconnect race and failing to `lo`) was deleted; `lte` is the sole gsm profile.

### Still OPEN (separate issue)
Signal bars blank for ~8 s **at hotspot activation** — hardwared's cached NM/modem read disrupted at
AP-up; the geo-gate does not touch it. Connectivity itself is unaffected.
