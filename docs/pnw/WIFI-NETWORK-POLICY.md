---
updated: 2026-09-14          # audited against origin/3devpnw @ ed66cc6e04 (networkd/uploader last touched f65fdbdaf9); §3/§4/§6/§7 + networkd line citations updated for pinunmetered2pnw (branch, NOT shipped)
status: current
---

# WIFI-NETWORK-POLICY — how the comma 3X picks its network, and what runs on it

**Ground truth for this document is the code on `origin/3devpnw` only** — `system/networkd/*.py`,
`system/ui/lib/wifi_manager.py`, `system/ui/widgets/network.py`,
`selfdrive/ui/mici/layouts/settings/network/network_layout.py`, `system/loggerd/uploader.py`,
`system/updated/updated.py`, `system/mapd/mapd_configd.py`, `system/hardware/{hardwared.py,base.py,
tici/hardware.py}`, `common/params_keys.h`, and the `system/networkd/tests/*` suite (297+ tests as of
`f65fdbdaf9`; 360 on branch `pinunmetered2pnw`). Where an older doc (`docs/NETWORK2XNOR.md`) or a memory note disagrees with this code,
**the code wins** — the disagreement is called out explicitly below, not silently reconciled. Every
rule below cites `file:line` and, where one exists, the test that pins it as a spec.

This is a driver-readable reference, not a design doc: read §6 (FAQ) first if you just want an answer.

---

## 0. The one-paragraph model

The comma 3X has **one WiFi radio** (`wlan0`): it is either the comma's own hotspot (AP mode) or a
client on someone else's WiFi, never both at once. **LTE (`wwan0`) is a separate modem and is always
independently available** — it is not part of the radio-sharing constraint. An always-on daemon,
`network_arbiterd` (`system/networkd/network_arbiterd.py`), polls NetworkManager (NM) every **20 s**
(`POLL_INTERVAL_S = 20.0`, `network_arbiterd.py:68`) and — **only while `TetheringEnabled=1`** — ranks
every saved WiFi network currently in range by cost (unmetered beats unknown beats metered) and
switches `wlan0` to the cheapest usable one, falling back to the comma's own hotspot (bridged onto LTE
for any tethered clients) when nothing usable is in range. A network the driver taps by hand in the
Settings WiFi list is recorded as a **pin** and the arbiter will not move the radio off it for cost
reasons until the pin ends — which, since `pinunmetered2pnw` (branch, not shipped), includes an explicitly
unmetered saved network *arriving* while the pinned network is not itself explicitly unmetered (§4). With `TetheringEnabled=0` the arbiter only ever tears the hotspot down if
it finds it up — NetworkManager's own built-in autoconnect (with priority order asserted by the
arbiter) picks client WiFi on its own, and none of the cost-ladder/pin logic below runs at all.

---

## 1. The network kinds

| Kind | What it is | Where it lives |
|---|---|---|
| **Saved WiFi profile** | Any network the driver has joined once. NM connection id `openpilot connection <SSID>` (`priority_connection_id`, `network_arbiter.py:56-58`), created by `WifiManager.connect_to_network` / `.activate_connection` (`wifi_manager.py:655-746`). | NM config, not a param |
| **`TetheringPriorityNetworks`** | JSON list of configured entries the arbiter treats specially: `{"label","ssid","lat","lon","portal","mobile"}` (`priority_networks.py:1-17`, `_coerce_entry` `priority_networks.py:23-45`). `lat/lon` = the network's learned GPS geofence center (auto-learned, see §3); `portal` = an optional captive-portal handler key (§3); `mobile: true` marks an entry that **travels with the car** (e.g. the driver's iPhone hotspot) — it is exempt from geofence learning, never ends a pin as `home`, and its arrival since a pick can only be established by scans (§4). (Before `pinunmetered2pnw` it could never end a pin at all.) | Param `TetheringPriorityNetworks` (`params_keys.h:77`, `PERSISTENT, STRING`) |
| **Legacy single-priority params** | `TetheringPriorityWifi` (one SSID, `params_keys.h:58`) + `TetheringHomeLocation` (one `[lat,lon]`, `params_keys.h:76`). Migrated into a one-entry `TetheringPriorityNetworks` list **only** when the new param has never held a valid JSON list — an explicit `[]` (all entries deleted via UI) is authoritative and is **not** resurrected (`priority_networks.parse`, `priority_networks.py:48-85`; tests `test_legacy_migration`, `test_empty_list_is_respected_not_resurrected`, `priority_networks.py` tests). | — |
| **The comma's own hotspot** | NM connection id literally `Hotspot` (`HOTSPOT_CONNECTION_ID`, `network_arbiter.py:53`). Broadcasts SSID `weedle-<first 4 chars of DongleId>` (`wifi_manager.py:188-192`), default password `"swagswagcomma"` (`DEFAULT_TETHERING_PASSWORD`, `wifi_manager.py:36`), gateway `192.168.43.1` (`TETHERING_IP_ADDRESS`, `wifi_manager.py:34`), subnet `192.168.43.0/24` (`TETHERING_SUBNET`/`HOTSPOT_SUBNET`, `wifi_manager.py:35`, `network_arbiterd.py:75`). Raising it installs `ip_forward` + an iptables-legacy MASQUERADE of that subnet out the LTE uplink (`_set_hotspot_nat`, `network_arbiterd.py:395-413`; the UI toggle's own copy is `_set_tethering_nat`, `wifi_manager.py:846-868`) — NAT is installed *before* the AP comes up so the first client packet already routes. | Param `TetheringEnabled` (`params_keys.h:57`, `PERSISTENT, BOOL, default "0"`) |
| **LTE** | The modem, `wwan0`, a **netplan-managed** gsm profile named `lte`. Never touched with `nmcli con modify` here — that crashes NetworkManager on this AGNOS (keyfile-writer assertion on a netplan-owned profile); only `con up`/`con down` and modem-level `mmcli` calls are used (`network_arbiterd.py:28-32`, `_park_lte`/`_unpark_lte`, `network_arbiterd.py:593-610`). | Params `GsmApn`, `GsmRoaming`, `GsmMetered` (`params_keys.h:53-55`) |

**The single-radio constraint is specifically between AP mode and WiFi-client mode on `wlan0`.** LTE
runs on a separate interface and is not mutually exclusive with either. "Falls back to LTE" in the
commit history means *the comma's own hotspot comes up and bridges tethered clients onto LTE* (or, with
no client WiFi and tethering off, the comma's own traffic simply rides LTE as NM's normal default
route) — it does not mean the arbiter "chooses" LTE as a WiFi-radio state.

**The arbiter only arbitrates the radio while `TetheringEnabled=1`.** `decide()`'s first branch:
tethering off → `down_hotspot` if the hotspot happens to be up, else `noop`, and it "never touches
client wifi" (`network_arbiter.py:695-699`; tests `test_off_hotspot_up_tears_it_down`,
`test_off_never_touches_client_wifi`, `test_off_ignores_priority_config`). With tethering off, NM's own
built-in autoconnect picks client WiFi; the only thing the arbiter still does is repair
`autoconnect`/`autoconnect-priority` on saved profiles (`_wifi_autoconnect_repair`,
`network_arbiterd.py:104-145`, called only while tethering is off, `network_arbiterd.py:949-950`) — see
§7 for why this is a driver-facing gap worth flagging.

---

## 2. Metered

### NM's three-state `connection.metered`

NetworkManager's per-connection `connection.metered` property has **three** values, and the middle one
is not a synonym for either end (`network_arbiter.py:522-536`):

| Value | Meaning | Constant |
|---|---|---|
| `no` | asserted cheap (driver, or the OS, said so) | `COST_UNMETERED = 0` (`network_arbiter.py:523`) |
| `unknown` | **the default — nobody ever said** | `COST_UNKNOWN = 1` (`network_arbiter.py:524`) |
| `yes` | asserted expensive | `COST_METERED = 2` (`network_arbiter.py:525`) |

Measured on the truck 2026-09-10: of four saved client profiles only the iPhone and the home WiFi
carried an explicit `no` — the driver's mobile Starlink and "Visitor" were both `unknown`
(`network_arbiterd.py:181-184`). **Folding `unknown` into `unmetered` is explicitly rejected** — an
earlier cut did that and let an alphabetical tiebreak decide between a metered Starlink and an
unmetered iPhone (`network_arbiter.py:564-567`; test `test_unknown_metered_state_is_NOT_treated_as_metered`,
`test_case_insensitive_like_the_rest_of_the_module` family in `test_network_cost_tiers.py`).

### The driver's UI control

Settings → Network → Advanced → **"Wi-Fi Network Metered"** — a 3-way segmented control
(`default` / `metered` / `unmetered`) in the raylib UI (`system/ui/widgets/network.py:159-162,
258-261`) and its mici equivalent (`selfdrive/ui/mici/layouts/settings/network/network_layout.py:49-56`).
Selecting a value calls `WifiManager.set_current_network_metered(MeteredType)`
(`wifi_manager.py:894-917`), which writes NM's `connection.metered` integer property (0=unknown,
1=yes, 2=no, `MeteredType` enum, `wifi_manager.py:70-74`) on the **currently active** connection. It is
disabled while tethering is active or while there is no IPv4 address (`network.py:213-219`) — **there
is no way to mark a network the device is not currently joined to**.

**This control never pins.** `set_current_network_metered` never calls `_record_manual_pick`
(compare `wifi_manager.py:894-917` against `connect_to_network`/`activate_connection`,
`wifi_manager.py:655,724`, which do). Marking a network metered/unmetered and joining/pinning it are
two independent actions — see §4.

### `deviceState.networkMetered` / the `NetworkMetered` param

Refreshed every **10 s** in `hardwared`'s hw-state thread (`"# these are expensive calls. update every
10s"`, `system/hardware/hardwared.py:109-144`) via `HARDWARE.get_network_metered(network_type)`
(`system/hardware/tici/hardware.py:276-295`), then written to the param **every deviceState tick, not
change-gated** (`params.put_bool_nonblocking("NetworkMetered", ...)`, `hardwared.py:443`).

- **On WiFi**: `True` only if NM's per-*device* `Metered` D-Bus property is `NM_METERED_YES` or
  `NM_METERED_GUESS_YES` (`tici/hardware.py:286-288`).
- **On cellular** (`cell2G`/`cell3G`/`cell4G`/`cell5G`): `True` **unless** NM explicitly reports
  `NM_METERED_NO` for that device (`tici/hardware.py:289-291`) — i.e. LTE defaults to metered and NM
  essentially never asserts "no" for a modem, so **LTE reads as metered in practice, generally as
  NM's own guess** ("`GENERAL.METERED "yes (guessed)"`" — probed live and quoted in
  `system/loggerd/uploader.py:208`, `uploader.py:660-661` docstrings). The base-class fallback
  (`system/hardware/base.py:155-156`) is the same rule: `network_type not in (none, wifi, ethernet)`
  → metered.

`NetworkMetered` (`params_keys.h:290`, `PERSISTENT, BOOL`, no explicit default → reads `False`
until first written) is what `uploader.py` and `updated.py` read as the "is this link expensive" bit
— see §5.

---

## 3. The cost ladder and ranking (netcosttier2pnw → netrank2pnw)

### The loop

`network_arbiterd.main()` (`network_arbiterd.py:739-1256`) runs forever, polling `nmcli` every
`POLL_INTERVAL_S = 20.0` s (`network_arbiterd.py:68`, each `nmcli` call bounded by
`NMCLI_TIMEOUT_S = 15.0` s, `network_arbiterd.py:69`). Each tick it builds a snapshot — active
connection, scan results, saved connections, metered states, GPS, the manual pick — and calls the pure
function `decide(...)` (`network_arbiter.py:651-735`), then applies exactly one action.

### The ranking, exactly (`choose_wifi`, `network_arbiter.py:539-610`)

For **every saved WiFi profile currently in range** (in the scan, or the currently-active usable link —
"sticky", `network_arbiter.py:593-596`), sort by:

1. **cost class** — `COST_UNMETERED (0) < COST_UNKNOWN (1) < COST_METERED (2)` (`network_arbiter.py:523-536`)
2. **configured membership** — a `TetheringPriorityNetworks` entry beats a non-member; members keep
   the driver's own list order among themselves (`member_rank`, `network_arbiter.py:585-590`)
3. **SSID** — a stable tiebreak so two equal-cost, equal-membership networks never flap
   (`network_arbiter.py:608-609`; test `test_the_choice_is_stable_between_equal_cost_networks`)

Driver, verbatim (2026-09-13): *"we don't want a solution where it just scans the Starlink SSIDs that I
have configured; we need a generic solution where an unmetered network is always prioritized over a
metered network or a default setting … if I clearly have an unmetered network and the other network is
set to either metered or default, then I made a conscious choice that this network is a priority if
it's available."* (commit `43036409a4`). This is a reversal of the immediately-prior design
(`netcosttier2pnw`, commit `eedddfa617`), where a configured entry not known to be metered was always
tier 0 regardless of an unmetered non-member in range; cost now dominates membership. Test:
`test_an_explicitly_unmetered_NON_member_beats_a_DEFAULT_configured_network`.

The winner brings up `up_priority` (if it's a configured member) or `up_fallback` (if not) —
`network_arbiter.py:711-717`; both do the same `nmcli con up`, the name is purely about membership for
logging. `explain_fallback` (`network_arbiter.py:616-648`) computes, purely from `decide()`'s own
inputs, *why* each configured network lost (not in scan / no saved profile / in failure backoff /
outranked), logged only on an applied `up_fallback` (test
`test_a_configured_network_IN_RANGE_that_loses_on_cost_is_named_as_outranked`).

**Nothing usable → the comma's own hotspot** (`up_hotspot`, `network_arbiter.py:732-735`) — bridged
onto LTE via NAT for any tethered clients (§1). If tethering is off in the first place, this never
fires; see §1.

### The kill switch

`DisableNetworkCostLadder` (`params_keys.h:67`, `PERSISTENT, BOOL, default "0"` — **inverted polarity**,
0 = ladder ON) is re-read every tick (`network_arbiterd.py:789`). Set, it reverts to the **exact
pre-ladder binary behaviour**: the first reachable *configured* entry, or the hotspot, no notion of
cost at all — deliberately, "that is what the kill switch is for" (`network_arbiter.py:689-693`). The
**one** thing it still honors is the failure-backoff ledger (below) — a kill switch that lets a dead
router strand the device offline is not a safe kill switch. Manual pins and upgrade scans are **not**
honored in binary mode (test `test_with_the_ladder_disabled_the_behaviour_is_pre_ladder_binary`).

### GPS geo-gating of scans

A WiFi scan forces the radio off-channel, which competes with an active hotspot — so scanning is
gated on GPS. `HOME_GEOFENCE_M = 250.0` (`geo_gate.py:18`). `near_any_home(locations, gps)` is
**fail-open**: with no learned location or no GPS fix it returns `True` (scan as before) — only a
*confident* "far from every known location" suppresses scanning (`geo_gate.py:49-59`; tests
`test_fail_open_when_home_unknown`, `test_fail_open_when_gps_unknown`).

**Escape hatch**: geo-gating only ever suppresses a scan while already on a *real client* WiFi
(`on_client_wifi = current_active not in (None, Hotspot)`, `network_arbiterd.py:887`). Disconnected or
sitting on the comma's own hotspot, scanning is **never** suppressed by GPS — a stale "far from home"
fix must never strand the device with no way to find WiFi again (`network_arbiterd.py:882-887`).

Each network's `lat/lon` is **auto-learned**: while connected to one of its own configured (non-mobile)
SSIDs, the current GPS fix becomes that entry's geofence center, but only written when it has moved
more than `LEARN_MIN_MOVE_M = 50.0` m from the stored value (flash-wear guard,
`network_arbiterd.py:71,860-878`). A `mobile: true` entry (the iPhone) never gets a learned location —
"wherever it was last seen is not where it will be next" (`priority_networks.py:99-107`).

GPS itself comes from `LastGPSPosition` (JSON, written at ~1 Hz by `mapd_configd`'s CAN→bridge, `ts` =
`time.monotonic()`), rejected as stale after `GPS_MAX_AGE_S = 10.0` s, from the future (a previous
boot), or with no `ts` at all — all three are treated as **no GPS**, never as a stale-but-real fix
(`_read_gps`, `network_arbiterd.py:477,481-525`; 8 unit tests in `test_network_gps_freshness.py`).
While parked (`IsOnroad=0`, i.e. `deviceState.started`) with the ignition-gated GPS producer not
running, the last fresh fix is *carried* forward for geo-gate decisions (not for learning a location) —
`_carry_parked_fix`, `network_arbiterd.py:528-562` (`gpscarry2pnw`).

### Upgrade scans on a link that is "done enough" but not cheapest

The geo-gate above predates the cost ladder, when the only client WiFi the arbiter could be on was
already the cheapest thing available. Measured 2026-09-13: the truck sat on metered Starlink for **66
minutes** with the unmetered iPhone never even scanned for, because the geo-gate suppresses scanning on
*any* client WiFi away from a learned location (commit `67eacc66c3`). `upgrade_scan_due`
(`network_arbiter.py:156-192`) forces through **one scan per `UPGRADE_SCAN_S = 120.0` s**
(`network_arbiter.py:83`) whenever the ladder is on, we're on client WiFi, the active link is **not
explicitly unmetered**, the UI is not still joining a manual pick (`pin_joining`), and the link is not
still settling (a bring-up awaiting judgement, or one that just appeared — so the very first upgrade
scan can't land inside a DHCP window). Suppressed entirely on an explicitly-unmetered link, since
nothing can outrank it on cost. Test: `test_the_upgrade_scan_is_throttled`;
`test_no_scans_at_all_while_on_a_priority_network`.

**Change in `pinunmetered2pnw` (branch, not shipped):** a pin used to suppress the upgrade scan for as long
as it was in force. Now it suppresses it only while the pinned network is **not yet the active link**
(`pin_joining`, `network_arbiterd.py:899`) — a scan must not take the radio off-channel under the UI's
own join. Once the pick is up, a pinned link that is not explicitly unmetered gets the same 120 s scan as
any other, because an unmetered network *arriving* now ends the pin (§4) and away from a learned location
this scan is the only way to see it arrive. A pinned explicitly-unmetered link still gets none. Tests:
`test_they_run_every_UPGRADE_SCAN_S_on_a_pinned_METERED_link`, `test_they_do_not_run_on_a_pinned_UNMETERED_link`,
`test_they_do_not_run_while_the_UI_is_still_JOINING_the_pick` (`TestUpgradeScansWhilePinned`).

### Failure backoff (the association-failure ledger)

Bringing up a client link drops the hotspot **first**; if the association then fails, the device has no
uplink at all. A per-SSID ledger tracks consecutive failures and backs a network off:

`FAIL_BACKOFF_S = (60.0, 300.0, 900.0)` (`network_arbiterd.py:240`) — escalating, held at the 15-minute
cap thereafter. Deliberately not an hour: "retrying a genuinely dead network costs one hotspot blip
every 15 min … while exiling a network that has come back costs the driver an hour of LTE in his own
driveway" (`network_arbiterd.py:241-244`). A network absent from `ABSENT_SCANS_FOR_FRESH_START = 2`
(`network_arbiterd.py:307`) **consecutive real scans** and then seen again gets a clean slate
(`_forget_on_reappearance`, `network_arbiterd.py:310-340`) — a scan that did not run (geo-gate
suppressed, or `nmcli` failed) is **not** evidence of absence and changes nothing.

A link is judged usable via `_client_link_usable` (tri-state: `True`=has an IPv4 address,
`False`=associated with none, `None`=could not tell, `network_arbiterd.py:256-304`) plus NM's per-device
`GENERAL.IP4-CONNECTIVITY` check — an address alone is not enough (a dead-DHCP or portal-blocked AP
holds an address and black-holes the device's own traffic, since `wlan0`'s default route outranks
`wwan0`'s: metric 600 vs 1000, `network_arbiterd.py:279`). `portal` state is treated as usable **only**
for a configured entry that declares a captive-portal handler (§ below); any other portal network is a
black hole and is marked unusable.

`DHCP_GRACE_S = 60.0` s (`network_arbiterd.py:235`) — NM's own DHCP timeout is 45 s and the poll is
20 s, so a first look can legally land mid-activation; inside the grace window a not-yet-usable link
stays sticky rather than being blamed. `judge_link` (`network_arbiter.py:438-519`) is the pure function
that decides sticky/blame/pending each tick — tested directly in `test_network_link_judgement.py`
(26 tests) because four separate defects lived in this exact logic as loop glue before it was
extracted.

### The unreadable-read hold, and its bound

If `nmcli con show --active` itself fails, `current_active` reads as `None` — indistinguishable, without
care, from "nothing is active". Left unguarded this handed the radio to the hotspot on top of a
perfectly healthy link (measured: `[(40 s, Hotspot), (60 s, iPhone)]` from one bad `nmcli` call,
commit `ff811dea0e`). Fix: while a run of failed active-connection reads is under
`ACTIVE_UNREADABLE_HOLD_S = 120.0` s (`network_arbiterd.py:253` — chosen as 2× `DHCP_GRACE_S`, "so an
NM slow to answer at boot or mid-activation is ridden out with margin"), any `up_priority`/
`up_fallback`/`up_hotspot` is suppressed as `noop` and logged once (change-only) as
`network_arbiter_active_unreadable_hold`. **The hold is gated on there being something to hold onto** —
a link seen active at the last good read, or the arbiter's own unresolved bring-up
(`seen_active or requested_active`, `network_arbiterd.py:1130`) — so a cold boot with nothing active yet
is **not** delayed by up to 120 s (`unreadhold2pnw`, commit `a0dd8097ee`; test
`test_at_BOOT_with_nothing_to_hold_an_unreadable_read_does_not_delay_the_first_connection`). Past the
bound, the action goes through as before and `network_arbiter_active_unreadable_released` is logged
once at ERROR level.

### Deferred judgement of a pending bring-up (`arbiterfu2pnw`)

The same unreadable-read problem can hit the tick that verifies a bring-up the arbiter itself just
issued: a failed `nmcli con show --active` used to read as "the bring-up never took", blaming the
network and starting its backoff on **zero evidence**. `judge_link` now treats `active_ssid=None`
(could not be read) as "keep waiting" for a pending bring-up, with the DHCP grace still counted from
when it was raised; the daemon passes `None` only while inside the same `ACTIVE_UNREADABLE_HOLD_S`
window as the hold above, so both release together (`network_arbiterd.py:986-993`; commit `f65fdbdaf9`).
Past the bound, a new ERROR-level `netcosttier_blamed_unverified` event precedes the ordinary
`netcosttier_assoc_failed`, so an unverified blame is distinguishable in the logs from a confirmed one.

### Logging (event names, all via `cloudlog`)

| Event | Fires when |
|---|---|
| `network_arbiter_active_changed` | The active WiFi connection changed since the last read, including **NetworkManager's own** autoconnect switch — `by=arbiter`\|`external`\|`not_as_requested` (`network_arbiterd.py:801-817`, `smallfix0914pnw`) |
| `netcosttier_upgrade_scan` | An upgrade scan (§ above) was issued |
| `netcosttier_assoc_failed` / `netcosttier_recovered` / `netcosttier_ledger_cleared` | Failure-ledger transitions (`network_arbiterd.py:343-361,333-335`) |
| `netcosttier_blamed_unverified` / `netcosttier_verify_deferred` | Unverified-blame / deferred-judgement events (§ above) |
| `network_arbiter_active_unreadable_hold` / `_released` | Unreadable-read hold engaged/released |
| `netcosttier_active_cost_unreadable` | The active link's cost couldn't be read and nothing is cached — a cost-driven move is suppressed (§ "D1" below) |
| `netcosttier_pin_set` / `_cleared` / `_held` / `_unreadable` | Manual-pin lifecycle (§4) |
| `netcosttier_pin_kept` | *(`pinunmetered2pnw`)* An explicitly unmetered network that would otherwise end the pin did not — `reason=visible_since_pick` (it was in range when the pick was made) or `pinned_cost_unread`. Once per network per pick (§4) |
| `network2xnor_portal_try` / `network2xnor_captive_portal` | Captive-portal auto-accept attempts/results |
| `network2xnor_lte_signal` | LTE bars/dBm/operator changed (throttled to every `SIGNAL_EVERY_N=3` ticks ≈ 60 s, `network_arbiterd.py:70`) |

### A cost-driven move needs a cost that was actually read (D1)

If the **active** link's `connection.metered` read fails with nothing cached, its cost is `None` —
distinct from a successful read of "unknown". Left alone, `choose_wifi` would rank that link as
`unknown` and let an explicitly-unmetered candidate outrank it on a single bad `nmcli` call — measured:
at home on the unmetered home WiFi with the unmetered phone also in range, one failed first read tore
the home link down and rebuilt it, right at boot when NM is slowest to answer (commit `8546e17288`).
Fix: while the active link is **usable** and its cost is unread (`None`, not "unknown"), any `up_*`
action is suppressed (`network_arbiterd.py:1085-1104`) and logged once as
`netcosttier_active_cost_unreadable`. A link that is **not** usable is exempt — a failure-driven move
must never be held. The kill switch's binary mode has no notion of cost and is exempt too.

---

## 4. Hand-picking, in depth (netscanpin2pnw / netrank2pnw)

### What records a pick

`WifiManualPick` (`params_keys.h:75`, `CLEAR_ON_MANAGER_START, JSON`) — `{"ssid": <str>, "ts": <float>}`,
where `ts` is `time.monotonic()` used only as an **identity** to tell one pick of an SSID from a later
pick of the same SSID (never arithmetic across processes/boots). Written by
`WifiManager._record_manual_pick` (`wifi_manager.py:627-654`) via `put_nonblocking`, called from
**both** human-join code paths, which are genuinely different NM calls:

| UI action | Calls | Records a pick? |
|---|---|---|
| Tap a **saved** network in the WiFi list | `activate_connection` → NM `ActivateConnection` (`wifi_manager.py:723-746`) | **Yes** (`wifi_manager.py:724`) |
| Enter a password for a **new** network, or re-enter a wrong one | `connect_to_network` → NM `AddAndActivateConnection2` (`wifi_manager.py:655-705`) | **Yes** (`wifi_manager.py:656`) |
| "Hidden Network" dialog | → `connect_to_network` | **Yes** (same path) |
| "Wi-Fi Network Metered" toggle | `set_current_network_metered` (`wifi_manager.py:894-917`) | **No** — never calls `_record_manual_pick` |
| "Enable Tethering" | `set_tethering_active` → `activate_connection(self._tethering_ssid)` (`wifi_manager.py:870-892`) | **No** — excluded explicitly: "tethering is not a WiFi pick" (`wifi_manager.py:645`), and `_record_manual_pick` itself early-returns when `ssid == self._tethering_ssid` (`wifi_manager.py:645-646`) |
| "Add Network Here" / editing `TetheringPriorityNetworks` | Params writes only, no NM call | **No** |
| NetworkManager autoconnecting a saved network on its own | No UI call at all | **No** — logged only as `by=external` in `network_arbiter_active_changed` |

A failed write is **logged loudly, never swallowed**: `"netscanpin: FAILED to record manual WiFi pick …
-- the arbiter will not know the driver chose it"` (`wifi_manager.py:652-653`).

### What a pin holds and suppresses

The arbiter tracks the `(ssid, ts)` identity it last saw — it never writes the param itself, so a new
pick racing an ending old one can never be clobbered (`network_arbiterd.py:763-770`). While a pin is
**in force**, `decide()`'s ladder result is overridden to `noop` for any `up_priority`/`up_fallback`/
`up_hotspot` (`network_arbiterd.py:1106-1115`) — this holds through **both** the join itself (so the
arbiter never fights the UI's own activation mid-DHCP) and the time spent on it afterward. It requires
`tethering_enabled and fallback_enabled` — with the ladder kill-switched, pins are not honored at all
(§3). `down_hotspot` (tearing the hotspot down because tethering itself was disabled) is **never**
suppressed by a pin.

### Every way a pin ends (`judge_pin`, `network_arbiter.py:110-153`)

| Reason | Exact condition |
|---|---|
| `failed` | `judge_link` blamed the pinned network (associated, no usable link, past `DHCP_GRACE_S`) — "a pin must never hold the device offline"; the failure ledger then applies as normal (`network_arbiter.py:120-122`) |
| `dropped` | It **was** the active link (seen active at least once since the pick) and no longer is |
| `join_timeout` | Never became the active link within `PIN_JOIN_WINDOW_S = 90.0` s of the arbiter first seeing the pick (`network_arbiter.py:80-82`) — chosen against NM's 45 s DHCP timeout plus the UI password-retry path re-adding the profile |
| `home` | A **stationary, explicitly unmetered** configured network has *arrived* since the pick (see below) |
| `unmetered` | *(`pinunmetered2pnw`, branch, not shipped — owner decision 2026-09-14)* An **explicitly unmetered saved** network has *arrived* since the pick and the pinned network is **not** itself explicitly unmetered (see "The `unmetered` rule" below). Logged `netcosttier_pin_cleared reason=unmetered by=<ssid>`. `home` is checked first, so a home arrival keeps its own reason (`network_arbiter.py:143-146`) |
| `superseded` | A newer `(ssid, ts)` pick replaced this one — decided by the daemon comparing identities (`network_arbiterd.py:839-845`) |
| `param_removed` | `WifiManualPick` went absent (SSH release hatch), **or** is present but the value could not be decoded as JSON — `params_pyx` returns `None` for both, so this reason covers both (`network_arbiterd.py:846-851`) |
| `reboot` | `WifiManualPick` is `CLEAR_ON_MANAGER_START` — a manager restart clears it unconditionally |

An **unreadable** active-connection read (`nmcli` failed) is **no evidence** and neither ends nor
advances a pin (`judge_pin`, `network_arbiter.py:139-140`; test `test_an_UNREADABLE_active_read_is_no_evidence`
/ `test_an_unreadable_ACTIVE_read_does_not_end_the_pin`). Likewise a `WifiManualPick` read failure or a
damaged value is logged (`netcosttier_pin_unreadable`) and **changes nothing** — "only an ABSENT param
is a silent 'no pin'" (`network_arbiterd.py:830-851`).

### A mobile network (the iPhone) is never `home` — but since `pinunmetered2pnw` it can end a pin as `unmetered`

`home_to_yield_to` (`network_arbiter.py:259-299`) only considers **stationary** configured entries
(`not e.get("mobile")`, `network_arbiterd.py:1038`). Exempting mobile entries was deliberate: "the
iPhone is a mobile priority entry, and 'any `up_priority` ends the pin' would reopen exactly the
measured case of the driver picking Starlink while his phone is in range" (commit `43036409a4`). Test:
`test_the_MOBILE_phone_in_range_does_NOT_end_the_pin`.

**Superseded in part by the owner's 2026-09-14 decision** (branch `pinunmetered2pnw`, not shipped): the
phone still never ends a pin as `home`, but an *arriving* explicitly unmetered phone does end a pin on a
network that is not explicitly unmetered, as `unmetered` (next section). The measured case the exemption
protected — a pick made **while the phone is in range** — is still protected, by the arrival requirement:
a phone that stays in range never "arrives". Test (inverted from netrank2pnw):
`test_the_phone_going_away_and_coming_back_ends_a_METERED_pin_as_unmetered_not_as_home`.

### The `home_to_yield_to` / arrival rule (D2, `netrank2pnw`)

A pin ends as `reason="home"` **only** for a home network that has genuinely *arrived* since the pick
was made — not merely "is in range right now". `home_to_yield_to` requires ALL of: stationary
(not mobile), **explicitly unmetered** (`no`, not `unknown`), present in this tick's **real** scan, a
saved profile, not serving a failure backoff, not the pinned network itself, and **arrived**
(`network_arbiter.py:266-273,294-299`).

"Arrived" (`update_home_arrival`, `network_arbiter.py:198-256`) is established by evidence only — a
pick made deliberately while home is already visible must **stick**, not revert on the very next tick
(driver-approved case: "pick Starlink, drive home, and the truck stays on paid Starlink in the
driveway", commit `989427d339`):

- `PIN_HOME_ABSENT_SCANS = 3` (`network_arbiter.py:194`) consecutive **real** scans that do not list
  it — but **only** while GPS cannot place the truck within `PIN_HOME_FAR_M` of that network's learned
  location (a missing scan at home is a router reboot or a weak AP, not the truck leaving); or
- GPS more than `PIN_HOME_FAR_M = 2.0 × HOME_GEOFENCE_M = 500.0` m (`network_arbiter.py:195`) from the
  network's learned location, on a tick where no real scan listed it — this is how a road pick (where
  the geo-gate suppresses scanning until arrival) ever gets to "arrived" at all.

A scan that did not run is no evidence either way; a scan that *does* list the network resets the miss
count (flicker never adds up). Once established, absence persists for the life of the pin. Tests:
`test_the_home_network_qualifies_once_it_has_ARRIVED`, `test_a_pick_made_while_home_is_visible_sticks_
with_home_continuously_in_range`, `test_home_FLICKERING_in_scan_results_does_not_end_it`,
`test_home_missing_from_scans_while_GPS_says_the_truck_is_STILL_HOME_does_not_end_it`.

The `home` rule is **unchanged** by `pinunmetered2pnw` and is not folded into the `unmetered` rule: it
still ends a pin on an explicitly unmetered network (the phone picked on the road, then home arrives),
which the strictly-cheaper `unmetered` rule would not. Test:
`test_home_still_ends_a_pin_on_the_UNMETERED_phone`.

### The `unmetered` rule (`pinunmetered2pnw` — branch, NOT shipped)

**Owner decision, 2026-09-14 ~19:45 PT.** Asked *"Should a manual WiFi pick end on its own when you mark
that network metered, or when an unmetered network appears?"*, the owner answered, verbatim: *"when an
unmetered network appears !"* The case that evening: KarlMoik (the mobile Starlink) was picked by hand and
marked metered; turning on the iPhone hotspot did not take over, because only a stationary home could end
a pin and no upgrade scan ran while pinned. **Marking a network metered still does not end or change a
pin by itself.**

`unmetered_to_yield_to` (`network_arbiter.py:336-391`), called from `network_arbiterd.py:1046-1047`, ends
the pin (`reason="unmetered"`) when a network is **ALL** of:

1. a **saved** client profile (`openpilot connection <SSID>`) whose `connection.metered` is **explicitly
   `no`** — `unknown` does not qualify;
2. in this tick's **real** scan (a scan that did not run is no evidence; the list the daemon seeds with the
   active configured network is not used);
3. **arrived** since the pick — the same `update_home_arrival` evidence as `home`, now tracked for every
   candidate (`arrival_candidates`, `network_arbiter.py:302-327`);
4. not serving a failure backoff, and not the pinned network itself;
5. **strictly cheaper** than the pinned network: the pinned network's `connection.metered` was **read**
   and is not `no`. An explicitly unmetered pin is never ended by this rule; a pinned network whose cost
   was never successfully read is not assumed expensive (the D1 rule: a cost move needs a cost that was
   read) and the pin holds.

Then the ordinary cost ladder takes the cheapest network in range — **not necessarily the one that
arrived** (see the risk below).

**Configured or not.** Any saved profile qualifies, not only `TetheringPriorityNetworks` entries. The
ladder already ranks every saved profile by cost first (§3, the 2026-09-13 generic rule), and `no` is set
on a profile only by the driver marking it. Restricting the rule to configured entries would leave the
truck on a paid pin beside a network the ladder itself ranks above it. Test:
`test_a_NON_configured_saved_network_marked_unmetered_ends_it_too`.

**Arrival evidence by kind of network** (`arrival_candidates`):

| network | location used | how "absent since the pick" is established |
|---|---|---|
| stationary configured entry | its learned `lat/lon` | exactly as for `home` above — 3 real scans (not counted while GPS places the truck within 500 m), or GPS > 500 m |
| mobile configured entry (the iPhone) | **none**, even if one is stored ("Add Network Here" records one) | **3 consecutive real scans only** |
| saved profile, not configured | none | 3 consecutive real scans only |

**Scans while pinned.** See §3 — the upgrade scan now runs every 120 s on a joined pin that is not
explicitly unmetered, so the arrival can be seen away from home. Away from home that means a phone must
be **out of 3 consecutive upgrade scans (about 4–6 min of pin time)** before its appearance counts; near a
learned location scans run every 20 s, so about 1 min.

**Logging** (change-only): `netcosttier_pin_cleared reason=unmetered by=<ssid>` when it ends (`by` = the
network whose arrival ended it, like `home`'s `trigger`); `netcosttier_pin_kept ssid=<pinned> by=<ssid>
reason=visible_since_pick|pinned_cost_unread` once per network per pick when a cheaper, explicitly
unmetered network in range did **not** end it (`network_arbiterd.py:1062-1070`).

**A flapping hotspot.** No presence confirmation was added; one real-scan appearance after established
absence ends the pin. Reasons: (a) a pin ends at most once — it cannot oscillate; (b) the absence
threshold is the anti-flap guard — a hotspot that keeps dropping out of fewer than 3 consecutive scans
never counts as arriving (`test_a_phone_that_FLICKERS_after_the_pick_never_counts_as_arriving`); (c) the
join happens on the same tick as the scan that saw the network, when it is most certainly beaconing, while
a second confirming scan 120 s later could miss a hotspot that is only visible for a while; (d) if the join
fails anyway, the existing ledger blames it and the ladder takes its next choice on the next tick. That is the previous network unless a cheaper network (unmetered, or default-cost such as `Visitor`) is in range, and **the pick does not return** (Fable probe 2026-09-14)
(`test_a_ONE_SCAN_appearance_ends_the_pin_once_and_a_failed_join_costs_one_tick`). After a pin ends,
repeated retries of a flaky phone are the unpinned ladder's existing behaviour (the §8.5 watch item in
`docs/NETCOST-STARLINK-TO-HOTSPOT.md`), not something this rule adds.

**Residual risks, for the owner and the reviewer:**
- **The ladder may not join the network that arrived.** At home with Starlink picked over a weak home AP,
  the phone hotspot appearing ends the pin, and the ladder then takes whichever explicitly unmetered
  network ranks first — the home WiFi, if it is listed before the phone (measured in the harness:
  `ups=[Hannelore]` with the list order Hannelore, iPhone; `ups=[iPhone]` with it reversed).
- **"Visible at pick time" lasts only while the network keeps beaconing.** If an iPhone stops advertising
  its hotspot when no client is connected (not verified on this phone), a pick made *while* it was in range
  can still end later: the phone drops out of 3 scans after the truck leaves it, then reappears when the
  hotspot screen is opened.
- **A hotspot turned on soon after the pick does not end it.** Absence must be established first (≈4–6
  min of upgrade scans away from home); a phone switched on within that window counts as visible since the
  pick, and `netcosttier_pin_kept reason=visible_since_pick` says so.
- A saved, explicitly unmetered **non-configured** network has no learned location, so its AP dropping out of
  3 scans while the truck is parked beside it reads as absence (no GPS veto).

### Marking a network metered does **not** pin it

Confirmed directly above (§4 table): `set_current_network_metered` never calls
`_record_manual_pick`. **Only** tapping/activating/joining a network pins it.

### The 2026-09-14 ~18:00 PT observed case (Silbereisen → KarlMoik)

The driver tapped "Silbereisen" and then, shortly after, tapped "KarlMoik" in the Settings WiFi list.
Both taps are `activate_connection`/`connect_to_network` calls, so **both** record a
`WifiManualPick`. The second write (`KarlMoik`, a later `ts`) is a newer identity than the first, so
the arbiter's next tick sees `pick != pin_key` and treats it as a fresh pick: the Silbereisen pin is
logged `netcosttier_pin_cleared(reason="superseded", by="KarlMoik")` and KarlMoik becomes the pin in
force (`network_arbiterd.py:838-845`). Because KarlMoik is now pinned, `decide()`'s cost-driven
`up_priority`/`up_fallback`/`up_hotspot` actions are suppressed for as long as that pin holds (§ above)
— **and because the driver's iPhone hotspot entry is `mobile: true`, it can never end that pin** even
if it comes into range and outranks KarlMoik on cost (§ "a mobile network is never `home`" above).
That is exactly why the iPhone hotspot did not take over: the mechanism is the ordinary pin-supersede
path plus the mobile exemption, not a special case. (This specific sequence is not separately logged
or documented anywhere in the repo as of this audit — it is reconstructed here from the code paths
above, which fully account for the observed behavior.)

**With `pinunmetered2pnw` (branch, not shipped)** the same evening would go differently: upgrade scans keep
running on the pinned KarlMoik, the iPhone is missing from them while its hotspot is off, and once it is
turned on the next upgrade scan sees it arrive — the pin ends (`reason=unmetered by=<iPhone>`) and the
ladder joins the phone (`test_TONIGHT_karlmoik_pinned_and_marked_metered_then_the_iPhone_hotspot_turns_on`).
The separate 18:55 `WRONG_KEY` failure joining the iPhone is not addressed by this and would still block
the join.

---

## 5. What traffic runs on each connection

### The uploader (`system/loggerd/uploader.py`)

Two passes, both gated by **one** shared notion of cost, `effective_metered(network_type, metered,
at_home)` (`uploader.py:176-215`, `uploadgate3pnw`, commit `718079e75a` — written specifically because
the two passes had drifted and 2,642 MB went out over a metered link the driver had explicitly marked
metered while 1 MB qlogs were refused on the very same link):

```python
def effective_metered(network_type, metered, at_home):
    return metered and not (at_home and network_type in PASS2_NETWORK_TYPES)
```

- `network_type` — from `deviceState.networkType`; `PASS2_NETWORK_TYPES = {NetworkType.wifi}`
  (`uploader.py:81`) — the comma's own hotspot is `never-default` so it never reports `wifi` while
  active, meaning "wifi" here structurally excludes the comma's own AP.
- `metered` — `deviceState.networkMetered` (§2).
- `at_home` — the `OnPriorityNetwork` param (`params_keys.h:201`, `CLEAR_ON_MANAGER_START, BOOL`),
  written change-only by `network_arbiterd`'s `on_priority_network(active_ssid, configured_ssids,
  active_metered)` (`network_arbiter.py:394-416`, `network_arbiterd.py:938-941`): **True** iff the
  active client SSID is a configured `TetheringPriorityNetworks` entry (case-insensitive) **and** its
  `connection.metered` is not explicitly `yes`. An explicitly-metered configured network does **not**
  qualify — membership alone used to authorize uploads over a link the driver had explicitly marked
  expensive (the 2026-09-10 incident); `unknown` still qualifies, and so does a failed read with
  nothing cached.

`pass1_allowed = not effective_metered(...)` (`uploader.py:218-245`) gates the small files
(`qlog`/`qcam`). `pass2_allowed(network_type, metered, at_home, onroad, parked, defer_hd)`
(`uploader.py:254-300`) gates the large "firehose" files (`FIREHOSE_FILES = {rlog, rlog.zst,
fcamera.hevc, ecamera.hevc}`, `uploader.py:31`):

```
if network_type not in PASS2_NETWORK_TYPES: return False
if metered and not at_home: return False
return True
```

`onroad`/`parked` are accepted as parameters but **no longer consulted** — `uploadanywifi2pnw`
(commit `6ad65ca264`, driver spec 2026-09-05, verbatim: *"nothing should be blocked when on a wifi that
is (1) either GPS preferred location or (2) on unmetered wifi … car running or not should not play a
role"*) removed the onroad/parked gate entirely; pass 2 now runs **while driving** whenever WiFi
qualifies. This is a stated **accepted risk** (`uploader.py:287-295`): full HD video while driving has
never been measured for the `commIssue`/`selfdrivedLagging` regression the original onroad gate existed
to prevent; only the rlog-only shape (with `DeferHDVideoUpload` on) has been measured safe (8-10
uploads/min, 23 min, zero lag events). If that regression returns, the documented mitigation is turning
`DeferHDVideoUpload` **ON**, not restoring the onroad gate.

**`DeferHDVideoUpload`** (`params_keys.h:190`, default OFF) holds `fcamera`/`ecamera`/`dcamera` in
*either* pass (`HD_VIDEO_FILES`, `uploader.py:36`) while `qlog`/`rlog`/`qcam` keep flowing — a
temporary hold, never xattr-marked, so it uploads normally once toggled off. `SkipWideCameraUpload`
(`params_keys.h:228`, default OFF) is a **permanent** skip of just `ecamera.hevc` and, unlike defer,
the deleter stops protecting those segments while it's on.

**The locator ping**: `LOCATOR_PING_S = 300.0` s (`uploader.py:251`) — a bare `~1 KB` `upload_url` GET
(`path="locator/ping"`) on **every** connection type, including metered, so the CloudWatch device-locator
line keeps updating even when file uploads are blocked (`uploader.py:779-788`).

### The updater (`system/updated/updated.py`)

**Code updates run on ANY link, metered included** — `updatemetered2pnw` (driver directive
2026-07-11, reaffirmed 2026-09-13, `updated.py:491-494`): stock openpilot skips the automatic fetch
whenever `NetworkMetered` is set and the last fetch is under 3 days old; that gate is removed here.
Every fetch logs which link it's spending (`cloudlog.event("updated: fetching update", metered=...,
network_type=...)`, `updated.py:499-502`). Normal poll interval **1.5 h** (`wait = 1.5*60*60`,
`updated.py:553`); a failed fetch backs off `5 min × 2^n`, capped at 1.5 h
(`updated.py:554-556`). `SIGHUP` (`WaitTimeHelper.update_now`, `updated.py:53,56-59`) wakes the sleep
and forces an immediate fetch regardless of backoff. `SIGUSR1` forces a check-only pass (no fetch).
While `DisableUpdates=1` ("Pause Updates"), the loop idles entirely and fetches nothing
(`updated.py:463-467`).

### mapd (`system/mapd/mapd_configd.py`)

**Whole-region map downloads run on ANY network, metered or not — deliberately, no gate at all**
(`mapd_configd.py:1-19,713-719`): "the owner would rather burn cellular data on an interstate than be
stranded without maps in the back country." The old fixed WA/OR/ID unmetered-WiFi-gated download is
gone (superseded by `mapdstate2pnw`); `OsmStateName` (`params_keys.h:123`) is a **dead param** — nothing
in the current tree reads it (confirmed by grep; matches `pnw/CLAUDE.md`'s note). This is measured, not
theoretical: `drives/2026-09-14/lte-usage/DRIVE_REPORT.md` found ≈14–17 GB of the comma's own 18.35 GB
of September LTE traffic was whole-state mapd pulls, many of them going out over LTE **even while the
comma was on WiFi** (next item).

### IPv6 route preference sends traffic over LTE even while on WiFi

**Not controlled by any code in this doc's scope — device configuration, evidenced in
`drives/2026-09-14/lte-usage/DRIVE_REPORT.md`, not asserted as a rule of the arbiter.** On the device
as measured: the `lte` NM profile sets `ipv6.method auto` with `ipv6.route-metric 1000`; **every**
saved WiFi profile (KarlMoik, Hannelore, Visitor, the iPhone, and Hotspot) has `ipv6.method ignore`, so
the kernel accepts the router advertisement on `wlan0` at the Linux default IPv6 route metric **1024**.
LTE's 1000 beats WiFi's 1024, so **any IPv6-reachable host is dialed over LTE even while `IPv4`
correctly prefers WiFi** (`wlan0` IPv4 metric 600 vs `wwan0` 1000 — WiFi wins there,
`network_arbiterd.py:279`). `map-data.pfeifer.dev` (mapd's download host) and `gitlab.com` (the
updater's LFS host) are both IPv6-reachable and both leak this way; `github.com`, S3, the upload API
gateway, and athena are IPv4-only and correctly follow WiFi. **This is listed as an open item in §7**,
not something the code intentionally does.

### Police proxy / athena

- **Police proxy** (`system/location_services/location_servicesd.py`) — a paid Waze/OpenWebNinja proxy
  call, cost-gated by **speed, not by network/metered state at all**: armed at
  `POLICE_GATE_MPH = 45` mph (`location_servicesd.py:142`), disarmed 2 mph below (hysteresis,
  `POLICE_RESUME_SPEED_MS`, `location_servicesd.py:145`). No `NetworkMetered`/WiFi check anywhere in
  this file (confirmed by grep) — it runs on LTE or WiFi indifferently once the speed gate is armed.
- **athena** (`system/athena/athenad.py`) — comma's own remote-control/telemetry websocket to
  `ATHENA_HOST` (`athenad.py:45`, default `wss://athena.comma.ai`), independent of the arbiter and the
  `uploader.py` gates entirely. Its own file-upload handler aborts an in-flight transfer if the
  connection becomes metered mid-upload and honors a per-request `allow_cellular` flag
  (`athenad.py:253-291`) — this is comma's stock mechanism, not a PNW gate. Disabled entirely when
  `ConnectBackend=Offline` (index 3, `network.py:176-181`).

### Table: connection type × traffic class → allowed?

| | Comma's own hotspot (WiFi radio, up while tethering + nothing better) | Saved client WiFi, unmetered | Saved client WiFi, metered, **not** a priority network | Saved client WiFi, metered, **is** a priority (`OnPriorityNetwork=1`) | LTE |
|---|---|---|---|---|---|
| Uploader pass 1 (qlog/qcam) | n/a (device is on the hotspot's uplink = LTE, see below) | ✅ | ❌ | ✅ | ❌ (unless `NetworkMetered` somehow reads False) |
| Uploader pass 2 (rlog/HD video) | n/a | ✅ | ❌ | ✅ | ❌ |
| Updater code fetch | ✅ (device traffic rides the hotspot's own uplink = LTE) | ✅ | ✅ | ✅ | ✅ (always, metered or not) |
| mapd whole-state download | ✅ | ✅ | ✅ | ✅ | ✅ (always, metered or not; also leaks onto LTE via IPv6 even from WiFi, see above) |
| Locator ping (~1 KB/300 s) | ✅ | ✅ | ✅ | ✅ | ✅ (always) |
| Police proxy | ✅ | ✅ | ✅ | ✅ | ✅ (speed-gated only, not network-gated) |
| Athena (comma's own channel) | per-request `allow_cellular` | ✅ | per-request | ✅ | per-request |

*("The comma's own hotspot" column describes what the **device itself** does while its radio is in AP
mode — its own uplink in that state is whatever NM's default route is, normally LTE, since the AP
can't also be a WiFi client. Tethered *clients'* traffic is simply NAT'd through and is not gated by
any of this repo's code at all.)*

---

## 6. Driver FAQ

**How do I make the truck use the iPhone hotspot right now?**
Open Settings → Network, tap the iPhone's SSID in the WiFi list (or type its password if not yet
saved). This immediately records a `WifiManualPick` and pins it (§4) — the arbiter will not move off it
for cost reasons (though it can still be outranked if `DisableNetworkCostLadder` is set, since pins
aren't honored in binary mode, §3). If `TetheringEnabled=0`, the tap alone (an ordinary NM
`ActivateConnection`) is what does it — the arbiter isn't involved either way in that case (§1).

**The arbiter tried the iPhone hotspot but did not switch. Why?** (observed 2026-09-14 18:55–18:58 PT)
With tethering on and no pin, the cost-upgrade scan (every `UPGRADE_SCAN_S = 120 s` on a non-unmetered link)
found `Dirk's iPhone 13` and ran `nmcli con up` on it (log: `priority wifi 'Dirk's iPhone 13' in range ->
... (leaving openpilot connection KarlMoik)`). Two different join failures followed:
1. **Wrong key.** wpa_supplicant reported `CTRL-EVENT-SSID-TEMP-DISABLED ... reason=WRONG_KEY`. NM then
   asked for secrets, but no secret agent is available for the arbiter's `nmcli`, so it logged
   `Secrets were required, but not provided`. The saved PSK was present (system-stored, `psk-flags=0`)
   and had connected earlier that evening.
   - Likely cause, **unverified**: the iPhone hotspot was in WPA3 mode ("Maximize Compatibility" off),
     while the comma's profile is `key-mgmt=wpa-psk` (WPA2). A WPA3-only AP shows up as a wrong key.
   - Otherwise the stored password really is wrong.
2. **Not visible.** NM reported `ssid-not-found` ("association took too long"). iPhones hide the hotspot
   unless the Personal Hotspot screen is open or a device is already joined.

Each failure is blamed (`netcosttier_assoc_failed`) and starts the backoff `FAIL_BACKOFF_S = (60, 300, 900)`.
So after two failures the arbiter waits 5 min, then 15 min, before trying the phone again.
**Fix at the truck:**
1. iPhone → Settings → Personal Hotspot → turn **Maximize Compatibility ON** and keep that screen open.
2. On the comma, tap the iPhone once, re-entering the password if asked. The tap records a pin, and a
   successful join clears the backoff.

*With `pinunmetered2pnw` (2026-09-14):* if you are on a network you picked that is **not** marked
unmetered (e.g. KarlMoik), just turning the iPhone hotspot on is enough — **provided** the phone was out of
range for 3 consecutive scans since you picked (about 4–6 min away from home, about 1 min near a learned
location). The pin ends and the truck takes the cheapest network in range (§4, "The `unmetered` rule"). The
iPhone profile must be marked unmetered (it is, as of 2026-09-10).

**Why didn't my phone hotspot take over from a network I picked?** *(`pinunmetered2pnw`)*
Look for `netcosttier_pin_kept` in the log: `reason=visible_since_pick` means the phone was already in range
when you picked (or came on before 3 scans had missed it), so your pick stands; `reason=pinned_cost_unread`
means the picked network's metered setting could not be read. No `netcosttier_pin_kept` at all: the phone is
not in the scans, is not marked unmetered, is in failure backoff, or the network you picked is itself marked
unmetered (nothing is cheaper). To force it, tap the phone in Settings.

**How do I mark a network metered without pinning it?**
Already the case: Settings → Network → Advanced → "Wi-Fi Network Metered" (the 3-way `default`/
`metered`/`unmetered` control) **never** records a pin (§4) — it only writes NM's `connection.metered`
on the currently-active connection. It is however only usable while connected to that network (the
control is disabled with no IPv4 address, `network.py:213-219`), so marking a network you are *not*
currently on **is not possible in the current UI** — not determined to have any workaround in this
code.

**How do I clear a pin?**
It clears itself the moment any of the conditions in §4's table fire (most commonly: the network drops,
or you tap a different network; with `pinunmetered2pnw`, also an unmetered network arriving). To force it: tap a different network (supersedes it immediately), or
remove the `WifiManualPick` param directly (the "manual release hatch over SSH", pinned by
`test_removing_WifiManualPick_releases_the_pin_and_the_ladder_resumes`; *the earlier citation here,
`network_arbiter.py:462`, pointed at unrelated `choose_wifi` docstring text*) —
`Params().remove("WifiManualPick")`. It also clears on any manager restart
or reboot (`CLEAR_ON_MANAGER_START`).

**Why did uploads stop on a network I'm connected to?**
Check, in order: (1) is `TetheringEnabled`/the network actually your active WiFi connection (uploads
never run on LTE or the comma's own hotspot for pass 1/2, §5); (2) is the network's `connection.metered`
set to `metered`, and is it a `TetheringPriorityNetworks` member (`OnPriorityNetwork`) — if metered and
**not** a priority member, both passes are blocked outright; (3) check the `LastUploadError` param
(`params_keys.h:229`) for a recent HTTP-status upload failure — pure connection drops are not surfaced
there by design (`uploader.py:411-422`), only actionable server-side errors; (4) a just-failed file
serves a 15-minute cooldown (`RETRY_COOLDOWN_S = 900.0`, `uploader.py:101`) before it's retried again.

**What happens when driving away from home?**
`OnPriorityNetwork` drops False the moment the WLAN disassociates from the priority SSID, immediately
stopping pass-1/2 upload eligibility on that basis (pass 2 can still run if some *other* WiFi that
happens to be unmetered comes into range, per §5's independent-qualifiers gate). If you had **not**
manually pinned anything, the arbiter's cost ladder simply keeps looking for the next-cheapest saved
network in range as you drive, geo-gated scanning re-arms once you're off any client WiFi
(`on_client_wifi=False` → scan never suppressed, §3), and it falls back to the comma's own hotspot/LTE
once nothing in range is usable. If you **had** pinned a network, driving away from it ends the pin via
`dropped` (§4) as soon as `judge_link` sees it's no longer the active link, and the ladder resumes.

**How do I add a priority network?**
Settings → Network → Advanced → **"Add Network Here"** — enter the SSID; the current GPS fix is
captured as that entry's geofence center automatically (or learned later once connected, if GPS wasn't
available at the time, §3). "Priority Networks" lists and lets you remove existing entries one at a
time (`network.py:404-438`). There is no in-UI way to edit an existing entry's location or portal
handler other than removing and re-adding it (the SSID literally `visitor`/`Visitor` is auto-tagged
with the `peak` captive-portal handler, `network.py:388`).

---

## 7. Known gaps and open owner questions

- ~~**Should a pin end when the *pinned* network is marked metered, or when an unmetered network
  appears?**~~ **DECIDED by the owner 2026-09-14 ~19:45 PT: *"when an unmetered network appears !"*** —
  built on branch `pinunmetered2pnw` (NOT shipped; §4 "The `unmetered` rule"). Marking the pinned network
  metered still does not end a pin by itself. Open follow-ups from building it, for the owner: (1) after
  the pin ends the ladder may take a different unmetered network than the one that arrived (e.g. the home
  WiFi you picked away from); (2) a hotspot switched on within ~4–6 min of a pick (away from home) does not
  end it; (3) whether the iPhone keeps advertising its hotspot with no client connected is unverified, and
  decides how long "visible at pick time" protects a pick.
- **The IPv6 route metric** (§5): LTE's IPv6 route (metric 1000) beats WiFi's kernel-default IPv6 route
  (metric 1024) on every saved profile, so any IPv6-reachable host (mapd's download host, the updater's
  LFS host) is dialed over LTE even while correctly associated to WiFi for IPv4. This is a live,
  measured device-configuration fact (`drives/2026-09-14/lte-usage/DRIVE_REPORT.md`), not something any
  file in `system/networkd/` or elsewhere in this doc's scope currently controls or even reads. Fixes
  considered in that report (raise the LTE profile's IPv6 metric, or disable IPv6 on it) are explicitly
  **not implemented** as of this audit — flagged there as a network change with stranding risk on a
  live, roaming device.
- **Map downloads on metered links** (§5): `mapd_configd`'s whole-state download is deliberately
  ungated on metered/WiFi — the owner's stated preference ("burn cellular rather than be stranded"). The
  `drives/2026-09-14/lte-usage/DRIVE_REPORT.md` reconciliation flags this as the dominant cost driver of
  the device's own LTE bill (≈14–17 GB of 18.35 GB observed in a 2-week window) and lists un-adopted
  options (a startup grace period before calling a region "uncovered"; persisting the per-region retry
  ledger across reboots so a reboot storm doesn't re-pull the same state repeatedly; gating first-entry
  downloads to unmetered WiFi with an LTE-only fallback once truly stranded). None of these are
  implemented; the owner has not decided among them as of this audit.
- **`docs/NETWORK2XNOR.md` is stale and superseded on multiple points** (kept as a historical record,
  not corrected in place per this repo's mirror convention): it describes the pre-ladder binary
  `decide()` ("Only that one named SSID can interrupt the hotspot — every other network is ignored") —
  false since `netcosttier2pnw`/`netrank2pnw` (every saved network in range is now a candidate); it
  documents only the single `TetheringPriorityWifi`/`TetheringHomeLocation` params — superseded by the
  multi-location `TetheringPriorityNetworks` list; and it predates the manual-pin mechanism, the
  unreadable-read hold, GPS-fix carrying, and the captive-portal handler entirely. `POLL_INTERVAL_S=20`
  is the one constant that still matches current code.
- **`_wifi_autoconnect_repair` only runs while `TetheringEnabled=0`** (§1): while tethering is on, NM's
  own autoconnect on other saved networks is deliberately suppressed by the UI's
  `_set_others_autoconnect(False)` so nothing steals the radio from the AP — but this means the
  autoconnect-priority values the arbiter writes (from the driver's own `TetheringPriorityNetworks`
  list order) are only actually consulted by NM while tethering has been off long enough for the repair
  to run. **Not determined from code** whether this produces any driver-visible surprise in practice
  (e.g. immediately toggling tethering on/off); no test or commit message addresses the interaction
  directly.

---

## Appendix: constants cited, with `file:line`

| Constant | Value | `file:line` |
|---|---|---|
| `POLL_INTERVAL_S` | 20.0 s | `system/networkd/network_arbiterd.py:68` |
| `NMCLI_TIMEOUT_S` | 15.0 s | `system/networkd/network_arbiterd.py:69` |
| `SIGNAL_EVERY_N` | 3 ticks (≈60 s) | `system/networkd/network_arbiterd.py:70` |
| `LEARN_MIN_MOVE_M` | 50.0 m | `system/networkd/network_arbiterd.py:71` |
| `PORTAL_MAX_TRIES` | 6 attempts/session | `system/networkd/network_arbiterd.py:72` |
| `HOTSPOT_SUBNET` | `192.168.43.0/24` | `system/networkd/network_arbiterd.py:75` |
| `DHCP_GRACE_S` | 60.0 s | `system/networkd/network_arbiterd.py:235` |
| `FAIL_BACKOFF_S` | (60.0, 300.0, 900.0) s | `system/networkd/network_arbiterd.py:240` |
| `ACTIVE_UNREADABLE_HOLD_S` | 120.0 s | `system/networkd/network_arbiterd.py:253` |
| `ABSENT_SCANS_FOR_FRESH_START` | 2 real scans | `system/networkd/network_arbiterd.py:307` |
| `GPS_MAX_AGE_S` | 10.0 s | `system/networkd/network_arbiterd.py:477` |
| `HOME_GEOFENCE_M` | 250.0 m | `system/networkd/geo_gate.py:18` |
| `PIN_JOIN_WINDOW_S` | 90.0 s | `system/networkd/network_arbiter.py:80` |
| `UPGRADE_SCAN_S` | 120.0 s | `system/networkd/network_arbiter.py:83` |
| `PIN_HOME_ABSENT_SCANS` | 3 real scans | `system/networkd/network_arbiter.py:194` |
| `PIN_HOME_FAR_M` | 500.0 m (2× `HOME_GEOFENCE_M`) | `system/networkd/network_arbiter.py:195` |
| `COST_UNMETERED` / `COST_UNKNOWN` / `COST_METERED` | 0 / 1 / 2 | `system/networkd/network_arbiter.py:523-525` |
| `BACKOFF_SCHEDULE_S` (LTE PDN-throttle guard) | (30.0, 120.0, 300.0, 600.0) s | `system/networkd/lte_guard.py:20` |
| `PORTAL_TIMEOUT_S` | 8 s | `system/networkd/captive_portal.py:58` |
| `MAX_FORM_HOPS` | 3 | `system/networkd/captive_portal.py:59` |
| `LOCATOR_PING_S` | 300.0 s | `system/loggerd/uploader.py:251` |
| `RETRY_COOLDOWN_S` | 900.0 s (15 min) | `system/loggerd/uploader.py:101` |
| `PASS2_INTERLEAVE` | 4 successful pass-1 uploads | `system/loggerd/uploader.py:95` |
| `MAX_UPLOAD_SIZES` (qlog / qcam) | 25e6 / 5e6 bytes | `system/loggerd/uploader.py:169-173` |
| Updater normal poll interval | 5400 s (1.5 h) | `system/updated/updated.py:553` |
| Updater fetch-failure backoff | `5 min × 2^n`, capped at 1.5 h | `system/updated/updated.py:556` |
| `hardwared` network-state refresh cadence | every 10 s | `system/hardware/hardwared.py:109-110` |
| `wlan0`/`wwan0` IPv4 default-route metrics (measured, code comment) | 600 / 1000 | `system/networkd/network_arbiterd.py:279` |
| `wlan0`/`wwan0` IPv6 default-route metrics (measured, device evidence) | 1024 / 1000 | `drives/2026-09-14/lte-usage/DRIVE_REPORT.md` |
| `TETHERING_IP_ADDRESS` | `192.168.43.1` | `system/ui/lib/wifi_manager.py:34` |
| `DEFAULT_TETHERING_PASSWORD` | `"swagswagcomma"` | `system/ui/lib/wifi_manager.py:36` |
| `SCAN_PERIOD_SECONDS` (UI's own WiFi-list scan, distinct from the arbiter poll) | 5 s | `system/ui/lib/wifi_manager.py:38` |
| `POLICE_GATE_MPH` / resume | 45 mph / 43 mph | `system/location_services/location_servicesd.py:142,145` |

## Params cited

| Param | Type/flags | Default | `file:line` |
|---|---|---|---|
| `TetheringEnabled` | `PERSISTENT, BOOL` | `"0"` | `common/params_keys.h:57` |
| `TetheringPriorityWifi` (legacy) | `PERSISTENT, STRING` | `""` | `common/params_keys.h:58` |
| `TetheringHomeLocation` (legacy) | `PERSISTENT, STRING` | — | `common/params_keys.h:76` |
| `TetheringPriorityNetworks` | `PERSISTENT, STRING` (JSON) | — | `common/params_keys.h:77` |
| `DisableNetworkCostLadder` | `PERSISTENT, BOOL` | `"0"` (ladder ON) | `common/params_keys.h:67` |
| `WifiManualPick` | `CLEAR_ON_MANAGER_START, JSON` | — | `common/params_keys.h:75` |
| `OnPriorityNetwork` | `CLEAR_ON_MANAGER_START, BOOL` | `"0"` | `common/params_keys.h:201` |
| `NetworkMetered` | `PERSISTENT, BOOL` | — (reads False unset) | `common/params_keys.h:290` |
| `GsmMetered` | `PERSISTENT, BOOL` | `"1"` | `common/params_keys.h:54` |
| `GsmRoaming` | `PERSISTENT, BOOL` | — | `common/params_keys.h:55` |
| `GsmApn` | `PERSISTENT, STRING` | — | `common/params_keys.h:53` |
| `DeferHDVideoUpload` | `PERSISTENT, BOOL` | `"0"` | `common/params_keys.h:190` |
| `SkipWideCameraUpload` | `PERSISTENT, BOOL` | `"0"` | `common/params_keys.h:228` |
| `LastUploadError` | `CLEAR_ON_MANAGER_START, STRING` | — | `common/params_keys.h:229` |
| `DisableUpdates` | `PERSISTENT, BOOL` | — | `common/params_keys.h:34` |
| `OsmStateName` (dead) | `PERSISTENT, STRING` | `"WA,OR,ID"` | `common/params_keys.h:123` |
| `RefreshLocationMap` | `PERSISTENT, BOOL` | `"0"` | `common/params_keys.h:139` |
| `GearPark` | `CLEAR_ON_MANAGER_START, BOOL` | `"0"` | `common/params_keys.h:202` |
| `IsOnroad` | `PERSISTENT, BOOL` | — | `common/params_keys.h:86` |
| `ConnectBackend` | `PERSISTENT, INT` | `"0"` | `common/params_keys.h:195` |

## Test index (spec-by-test, non-exhaustive — see `system/networkd/tests/*` for the full ~297+ suite)

| Rule | Test(s) |
|---|---|
| Off tethering never touches client WiFi | `test_off_never_touches_client_wifi`, `test_off_ignores_priority_config` (`test_network_arbiter.py`) |
| Cost ranking: unmetered < unknown < metered, membership as tiebreak | `test_an_explicitly_UNMETERED_network_beats_a_DEFAULT_priority_member`, `test_tier1_unmetered_beats_tier2_metered`, `test_unknown_metered_state_is_NOT_treated_as_metered` (`test_network_cost_tiers.py`) |
| Kill switch reverts to binary, ignores pins/scans | `test_with_the_ladder_disabled_the_behaviour_is_pre_ladder_binary` (`test_network_arbiter_sequences.py`) |
| Geo-gate fails open | `test_fail_open_when_home_unknown`, `test_fail_open_when_gps_unknown` (`test_geo_gate.py`) |
| Upgrade scan due/suppressed | `test_the_upgrade_scan_is_throttled`, `test_no_scans_at_all_while_on_a_priority_network` (`test_network_arbiter_sequences.py`) |
| Failure ledger backoff + reappearance | `test_backoff_escalates_and_is_capped`, `test_a_reappearing_ssid_gets_a_clean_slate`, `test_a_SUPPRESSED_scan_is_not_absence` (`test_network_cost_daemon.py`) |
| Unreadable-active-read hold, bounded, no boot delay | `test_a_PERSISTENT_failure_acts_as_before_once_the_bound_passes_and_says_so_loudly`, `test_at_BOOT_with_nothing_to_hold_an_unreadable_read_does_not_delay_the_first_connection` (`test_network_arbiter_sequences.py`) |
| Deferred judgement of a pending bring-up | `test_a_deferral_never_happens_without_the_hold__double_fault` (`test_network_arbiter_sequences.py`) |
| Manual pick recorded on both join paths, not on metered/tethering | `system/ui/lib/tests/test_wifi_manual_pick.py` |
| Pin holds during join and after | `test_the_pin_holds_the_radio_DURING_the_join` (`test_network_arbiter_sequences.py`) |
| Pin ends: failed/dropped/join_timeout/superseded | `test_the_pin_ends_when_the_pinned_network_DROPS_and_the_ladder_resumes`, `test_a_pick_that_never_joins_ends_on_the_join_window`, `test_a_pinned_network_that_FAILS_does_not_hold_the_device_offline`, `test_a_newer_pick_supersedes_the_old_one` (`test_network_arbiter_sequences.py`) |
| Mobile entry never ends a pin as `home`; a phone in range at the pick does not end it | `test_the_MOBILE_phone_in_range_does_NOT_end_the_pin` (`test_network_arbiter_sequences.py`) |
| *(pinunmetered2pnw, branch)* An arriving explicitly unmetered network ends a pin on a non-unmetered network; the owner's 2026-09-14 case | `test_TONIGHT_karlmoik_pinned_and_marked_metered_then_the_iPhone_hotspot_turns_on`, `test_the_phone_going_away_and_coming_back_ends_a_METERED_pin_as_unmetered_not_as_home`, `test_a_NON_configured_saved_network_marked_unmetered_ends_it_too` (`test_network_arbiter_sequences.py`); `TestUnmeteredToYieldTo`, `TestArrivalCandidates` (`test_network_manual_pick.py`) |
| *(pinunmetered2pnw, branch)* What does not end it: unmetered pin, `unknown`, backoff, unread pinned cost, no real scan, visible since pick, flicker | `TestWhatDoesNotEndThePin`, `TestAFlappingHotspotDoesNotThrash`, `test_a_pick_made_while_the_iPhone_is_ALREADY_visible_sticks_and_the_log_says_why` (`test_network_arbiter_sequences.py`) |
| *(pinunmetered2pnw, branch)* Upgrade scans while pinned: 120 s on metered, none on unmetered, none during the join | `TestUpgradeScansWhilePinned` (`test_network_arbiter_sequences.py`) |
| *(pinunmetered2pnw, branch)* The `home` rule is unchanged | `test_home_still_ends_a_pin_on_the_UNMETERED_phone` + every `TestPinYieldsToHome` / `TestAPickMadeAtHomeSticks` test |
| Pin sticks if picked while home visible; ends only on arrival | `test_a_pick_made_while_home_is_visible_sticks_with_home_continuously_in_range`, `test_home_FLICKERING_in_scan_results_does_not_end_it`, `test_home_missing_from_scans_while_GPS_says_the_truck_is_STILL_HOME_does_not_end_it` (`test_network_arbiter_sequences.py`) |
| Uploader: one cost function, no divergence between passes | invariant test in `718079e75a`'s `TestUploadGate` (`test_uploader.py`) |
| Uploader: pass 2 no longer gated on onroad/parked | property test in `6ad65ca264`'s `TestUploadGate` (`test_uploader.py`) |

---

*Compiled 2026-09-14 against `origin/3devpnw` (checked out at `ed66cc6e04`; the networkd/uploader code
itself was last touched by `f65fdbdaf9`, `arbiterfu2pnw`). Mirror: `docs/WIFI-NETWORK-POLICY.md` at the
workbench root; canonical copy is this one, on the branch, per the doc-reorganization convention in
`../CLAUDE.md` / `../../CLAUDE.md`. On branch `pinunmetered2pnw` (NOT shipped) the `network_arbiter.py` /
`network_arbiterd.py` line citations throughout were re-mapped to that branch's code, and §0, §1, §3, §4, §6,
§7 and the test index describe the owner's 2026-09-14 "when an unmetered network appears" rule. The root
mirror is not updated until the branch ships.*
