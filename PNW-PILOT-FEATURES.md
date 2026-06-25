# PNW-PILOT — Feature List

**pnw-pilot** is the Pacific Northwest production distribution of openpilot, tuned for the **I-5
corridor between Seattle, WA and Corvallis, OR**. It is a fork of a fork:

```
commaai/openpilot              upstream
  └─ xnor-tech/openpilot       adds full legacy Tesla HW1/HW2/HW3 (Raven) support
       └─ dirkpetersen/pnw-pilot   ← THIS distribution (Pacific Northwest)
```

xnor is the base because it carries the **proven Tesla Raven** drivers; PNW layers an integrated
feature set on top and stays **panda-safe** (every new toggle defaults **OFF**, no panda-safety code
is weakened). One physical comma 3X is moved between two cars; car-specific code is fingerprint-gated
so it's inert on the other car.

> **Status legend:** ✅ deployed & in use · 🟡 deployed, behavior-neutral until enabled / not yet
> road-proven · 🧪 staged on a branch, not deployed. Param names in `code font` are the toggles in
> `common/params_keys.h`.

---

## 1. Cars supported

| Car | Class | Support |
|-----|-------|---------|
| **Tesla Model S Long Range Plus 2021** | Raven, HW3 (primary) | ✅ Full legacy support inherited from xnor: `tesla_legacy.h` panda safety, legacy CAN builder, Continental ARS4-B radar, `GTW_status` 0x348 ignition |
| **Ford F-150 Lightning Flash 2025** | 131 kWh (secondary) | ✅ Fingerprint + FW; 🟡 Tier-2: camera/radar lead + opt-in openpilot longitudinal |

Both cars **auto-fingerprint** when powered on; no manual pinning.

---

## 2. Longitudinal / lateral driving features

| Feature | What it does | Toggle(s) | Status |
|---------|--------------|-----------|--------|
| **CES — Conditional Experimental Switching** | Chill by default, auto-switches to Experimental for upcoming curves (map + vision), low speed, stop lights, and slow leads; 3-state top-right button | `ConditionalExperimentalSwitching`, `CESMode` (Off/Light/Standard), `CESCurves`/`CESStops`/`CESLowSpeed`/`CESLead`, `CESButtonState`/`CESStatus` | 🟡 deployed, default OFF |
| **VTSC — Vision Turn Speed Control** | Caps cruise speed in curves from the model's predicted path curvature (decel-limited, smooth); Terwilliger-calibrated. Rides the CES master toggle (no separate param) | (rides `ConditionalExperimentalSwitching`) | 🟡 deployed, not yet road-proven |
| **OSM speed-limit display + warnings** | Shows current/next posted speed limit; lower-limit warning | `ShowSpeedLimit`, `MapSpeedLimit`, `NextMapSpeedLimit` | ✅ |
| **Nudgeless lane change** | Lane change without a nudge once blinker is on (timed hold) | `NudgelessLaneChange` | 🟡 |
| **No disengage on brake / accelerator** | Tapping the brake (or gas) doesn't disengage | `NoDisengageOnBrake`, `DisengageOnAccelerator` | 🟡 |
| **Relaxed driver monitoring** | Dual-counter, longer attention timeouts | (DM policy) `DriverTooDistracted`, `Offroad_DriverMonitoringUncertain` | 🟡 |
| **Longitudinal personality** | Standard openpilot follow-distance personality | `LongitudinalPersonality` | ✅ |
| **Blind-spot gate (BSM)** | Tesla Raven blind-spot via `AutopilotStatus` 0x399 to block nudgeless lane changes into an occupied lane | (reuses `NudgelessLaneChange` gate) | 🧪 staged, on-car CAN check pending |

See `docs/CES.md`, `docs/VTSC.md`, `docs/MAPD2XNOR.md`, `docs/AUTO2XNOR.md`, `docs/DMON2XNOR.md`,
`docs/BSM2XNOR.md`, `docs/GLARE.md` in the workbench for the deep dives.

---

## 3. Connectivity & fleet — drive-data upload (connect2pnw)

Self-hosted: drives upload to **AWS S3 (`comma-connect` bucket)** via an API Gateway that issues
presigned PUTs — **not** comma connect.

| Feature | What it does | Toggle / signal | Status |
|---------|--------------|-----------------|--------|
| **Self-hosted upload gateway** | `API_HOST` pinned to the gateway (`…execute-api.us-west-2…`) so uploads can't silently fall back to comma and 412 (= data loss) | baked default + `launch_env.sh` | ✅ |
| **Two-pass upload** | Pass 1 = small files (qlog/qcamera) on any network; **pass 2 = HD video + rlog ("firehose") only on real, non-metered WiFi** | — | ✅ |
| **HD-interleave** | Forces an HD file after every N small uploads so video isn't starved behind a backlog | — | ✅ |
| **Firehose "uploading" indicator** | Green CONNECT→UPLOADING logo, lit **only while an HD transfer is actually in flight** | `FirehoseActive` | ✅ |
| **Prompt indicator clear on WiFi drop** | Daemon watcher clears the green within ~0.5 s when WiFi drops mid-transfer (the main loop is blocked in the PUT) | — | ✅ (this branch) |
| **Metered-aware firehose** | Marking a connection metered (incl. a phone hotspot) stops HD pass-2 and drops the green logo; pass-1 still runs throttled | `NetworkMetered`, `GsmMetered` | ✅ (this branch) |

---

## 4. Connectivity & fleet — networking (network2pnw)

| Feature | What it does | Toggle(s) | Status |
|---------|--------------|-----------|--------|
| **Perpetual tethering** | Device keeps its hotspot up | `TetheringEnabled` | ✅ |
| **Priority-WiFi switching** | Auto-join known priority networks when in range | `TetheringPriorityWifi`, `TetheringPriorityNetworks` | ✅ |
| **GPS geo-gated WiFi scanning + Set Home** | Only scan/join client WiFi near saved home location(s) | `TetheringHomeLocation` | ✅ |
| **Captive-portal auto-accept** | Walks a MikroTik TOS portal (Peak "Visitor") so uploads work behind it; never blocks LTE/tethering | (per-network `portal`) | ✅ |
| **LTE PDN-throttle guard** | Recovers a throttled/stuck LTE data connection | (lte_guard) | ✅ |
| **Tethering NAT fix** | iptables masquerade so tethered clients get internet | — | ✅ |

Lives in `system/networkd/` (`network_arbiterd.py`, `captive_portal.py`, `geo_gate.py`,
`lte_guard.py`, `priority_networks.py`). See `docs/NETWORK2XNOR.md`, `docs/CONNECT2XNOR.md`.

---

## 5. Maps (Pacific-Northwest defaults)

| Feature | Detail | Toggle(s) |
|---------|--------|-----------|
| **OSM data via pfeifer mapd** | Speed limits + curve speeds from OpenStreetMap (pfeiferj/mapd) | `MapdSettings`, `OsmLocal` |
| **PNW maps default** | Default coverage **Washington / Oregon / Idaho**; first download auto-arms on a fresh deploy | `OsmStateName`, `MapdPnwMapsRequested`, `OsmAutoRequested`, `OsmDbUpdatesCheck`, `OSMDownloadLocations`/`OSMDownloadBounds` |
| **mapd binary resilience** | mapd binary re-downloads after every update, network-agnostic (not Wi-Fi-gated) | — |

Curve/longitudinal behavior is calibrated to the **I-5 Terwilliger curve (Portland)**. See `pnw/CLAUDE.md`.

---

## 6. UI / device

- **Offroad sidebar car label** — shows last-known car (Raven / Lightning) even when offroad (`docs/FINGERPRINT2XNOR.md`). 🧪/🟡
- **Ford SecOC false-dashcam fix** — keeps the Lightning from showing a bare dashcam screen (`docs/SecOC.md`). 🟡
- **CES / VTSC status overlays** — top-right CES button + lower-right VTSC status.
- **Update controls** — `DisableUpdates`, `SnoozeUpdate`, `UpdateAvailable`, updater branch/description params.

---

## 7. Safety posture

- Priority order: **safety > stability > quality > features.**
- New feature toggles default **OFF**; none touch panda safety.
- Tesla legacy safety mode is `tesla_legacy.h` (counter + checksum validation load-bearing — never weakened).
- Tesla changes are intentionally **not** submitted upstream (too niche).

---

## 8. Where the detail lives

This file is a high-level index. The authoritative per-feature docs (deploy steps, pitfalls, param
registry) live in the workbench:

- `docs/INDEX.md` — catalog of every doc
- `docs/DEVICE-STATE.md` — source of truth for what's deployed on the 3X (full param registry)
- `pnw/CLAUDE.md` — the PNW distribution overview
- Per-effort: `docs/CES.md`, `docs/VTSC.md`, `docs/MAPD2XNOR.md`, `docs/AUTO2XNOR.md`,
  `docs/DMON2XNOR.md`, `docs/LIGHT2XNOR.md`, `docs/BSM2XNOR.md`, `docs/NETWORK2XNOR.md`,
  `docs/CONNECT2XNOR.md`, `docs/FINGERPRINT2XNOR.md`, `docs/SecOC.md`, `docs/GLARE.md`

> Branch note: this list reflects the `3devpnw` line (feature toggles present in `common/params_keys.h`
> + `selfdrive/controls/lib/{ces_xnor,vtsc_xnor}` + `system/networkd/`). Status tags are point-in-time;
> validate against `docs/DEVICE-STATE.md` before relying on "deployed" claims.
