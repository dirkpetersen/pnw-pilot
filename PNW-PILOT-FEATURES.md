# PNW-PILOT — Feature List

**pnw-pilot** is the Pacific Northwest production distribution of openpilot, tuned for the **I-5
corridor between Seattle, WA and Corvallis, OR** and driven day-to-day on a Tesla Model S (Raven) and
a Ford F-150 Lightning. It is a fork of a fork:

```
commaai/openpilot              upstream
  └─ xnor-tech/openpilot       adds full legacy Tesla HW1/HW2/HW3 (Raven) support
       └─ dirkpetersen/pnw-pilot   ← THIS distribution (Pacific Northwest)
```

xnor is the base because it carries the **proven Tesla Raven** drivers; PNW layers an integrated
feature set on top and stays **panda-safe** (every new toggle defaults **OFF**, no panda-safety code
is weakened). One physical comma 3X is moved between two cars; car-specific code is **capability-gated**
(`pnw_vehicle.py`, never `carFingerprint` string checks) so it's inert on the other car, and the Tesla
is never impaired by Ford work or vice-versa.

> **Status legend:** ✅ deployed & in use · 🟡 deployed, behavior-neutral until enabled / not yet
> road-proven · 🧪 shadow / staged, not acting. Param names in `code font` are the toggles in
> `common/params_keys.h`.
>
> **Deploy model (since 2026-07-09):** the device runs `/data/openpilot` as a **git checkout of
> `3devpnw` with auto-update enabled** — ship = push to `3devpnw`, the updater fetches and installs at
> the next reboot/ignition-off, build-on-boot handles rebuilds. (`4devpnw` = frozen known-good
> fallback; `3testpnw` = friends' channel.) The old file-overlay + patch-script era is gone.

---

## 1. Cars supported

| Car | Class | Support |
|-----|-------|---------|
| **Tesla Model S Long Range Plus 2021** | Raven, HW3 (primary) | ✅ Full legacy support inherited from xnor: `tesla_legacy.h` panda safety, legacy CAN builder, Continental ARS4-B radar, `GTW_status` 0x348 ignition. Blind-spot via `AutopilotStatus` 0x399 (BSM). |
| **Ford F-150 Lightning Flash 2025** | 131 kWh (secondary) | ✅ Fingerprint + FW; ✅ 4-signal BlueCruise-grade lateral (reflashed panda); 🟡 opt-in openpilot longitudinal (Alpha Long); ✅ ICBM stock-ACC curve slowdowns when Alpha Long is off. |

Both cars **auto-fingerprint** when powered on; no manual pinning. When the device is physically moved
between the two cars it **auto-recalibrates** (see §7).

---

## 2. Longitudinal / curves / speed control

The heart of the distribution. Chill-by-default driving that anticipates curves, stops, and leads —
tuned against real I-5 / I-90 / I-82 drive logs.

| Feature | What it does | Why it exists | Toggle(s) | Status |
|---------|--------------|---------------|-----------|--------|
| **CES — Conditional Experimental Switching** | Chill by default; auto-switches to Experimental for upcoming curves (map + vision), low speed, stop lights, and slow leads. 3-state top-right button (CES / Chill / Exp). | Chill is smooth on the highway but doesn't slow for curves/stops; Experimental does but is jerky everywhere. CES gets both by switching only when needed. | `CESMode` (0=Off / 1=Light / 2=Standard), `CESCurves`/`CESStops`/`CESLowSpeed`/`CESLead`, `CESButtonState`/`CESStatus` | 🟡 default OFF |
| **VTSC — Vision Turn Speed Control** | Caps cruise speed in curves from the model's predicted path curvature (decel-limited, smooth). Rides the CES master. | Keeps the car from carrying too much speed into a bend the driver can see coming. | (rides `CESMode`); status `VTSCStatus` | 🟡 |
| **Map-curve braking (MTSC)** | Folds pfeiferj OSM map-curve target speeds into VTSC so the car brakes for curves that are still **out of camera sight**. | Vision alone can't see a blind curve; the map can. Turned ON by default after the Snoqualmie I-90 over-brake analysis. | `VtscMapCurves` (default **ON**) | ✅ |
| **Sharp-curve full-horizon + regen-coast** | Scans the full ~500 m the map publishes (was ~370 m) and gets off the gas early so the curve **entrance** is the slowest point, then accelerates out. Braking is EV regen-coast (~0.2 g), not friction, unless a genuinely sharp curve needs a bounded firmer stop. | Recurring blind sharp-curve **"TAKE CONTROL" beep** on I-90 descents — the slowdown started too late because VTSC threw away ~130 m of the map's warning. | (rides `CESMode`) | ✅ (06-29) |
| **Twisty-descent base trim** | On a packed sequence of curves that is also descending, holds a lower base cruise through the section instead of re-accelerating to full set between blind curves. | Winding downhill stretches were surging between curves; flat twisty roads keep full speed (per-curve VTSC handles them). | (rides `CESMode`) | ✅ |
| **Highway speed-limit floor** | VTSC never trims below the posted highway limit (read from mapd). | A live Snoqualmie retune over-slowed and forced throttle overrides; the floor bounds the downside so a lower lateral target is safe. | (reads mapd `MapSpeedLimit`) | ✅ (07-01) |
| **Lightning curve penalty (per-car)** | The Lightning slows **more** for curves than the Tesla — a piecewise "hump" (≈1 mph below 30 mph → 5 mph across 45–62 mph → taper by 75+), plus extra on downhill and left-hand curves. | The Lightning's EPS is physically weaker; it washed out of fast sweepers the Tesla holds (91 steering-override clusters on one I-90 drive). Field-calibrated over three same-evening passes. | (rides `CESMode`; optional `/data/pnw/curve.json`) | ✅ (07-11) |
| **Rain slowdown** | Driver-selected wet-weather margin subtracted from curve targets (Light ≈ 3 mph, Heavy ≈ 5 mph). Applies to **both** cars equally. | Driver wants to dial in extra caution in the wet without touching the dry curve tuning. | `RainMode` (0=None/1=Light/2=Heavy, default None; `/data/pnw/rain.json`) | ✅ (07-12) |
| **Red-light stop-hold guards** | A1: the "stop" reason fires even with a lead present at a creep, so the light governs; A2: a standstill hold blocks the Experimental→Chill exit at ~0 mph. | Tesla **red-light lurch** — CES adopted Chill at 0.4 mph behind a creeping lead and the Chill MPC launched at up to 1.6 m/s² toward the set speed. | (internal CES) | ✅ (07-12) |
| **Standstill latch** | No Chill at 0 mph; launches from a standstill behind a close lead are model-governed, not a hard Chill MPC launch. | Stop-and-go **"horse bucking"** + red-light **jolt**: CES flapped Chill↔Experimental at a standstill (radar lead dropout → dwell expiry) and released in Chill into a 9–14 m gap. | (internal CES) | ✅ (07-13) |
| **Stop-intent fast path + evidence-gated pull-away** | An absolute fast path into Experimental on model stop-intent; a guarded exception lets the car follow a lead pulling away below the red-light floor. | Stop churn is the safe direction; but a genuinely departing lead shouldn't strand the car. | (internal CES) | ✅ (07-12) |
| **ICBM — stock-ACC curve slowdown (Lightning)** | When Alpha Long is OFF, gives the Lightning curve slow-downs on **stock Ford ACC** by tapping its own SET−/SET+ buttons over CAN (0x083, no panda change) to steer the set speed toward a curve apex target. Map-first (500 m), vision beyond map reach, guarded restore of the set speed after the curve. | The Lightning normally runs stock ACC with no curve awareness; ICBM adds VTSC-grade anticipation without taking longitudinal authority (Ford's radar keeps all braking/accel). The Alpha-Long toggle is the op-long-vs-ICBM A/B switch. | `CESMode` master; A/B = Alpha Longitudinal; `IcbmTarget` (mem) | ✅ (07-10→12) |
| **CES2 decision core (shadow)** | A redesigned decision core: graded stop-urgency from the model trajectory endpoint, precedence rules, ~19 rules → ~12. Runs **shadow-only** (logs `ces2*` telemetry; v1 still decides). | Groundwork for a cleaner, more predictable CES; not road-ready until the stop-distance table is re-anchored on lebowski-model replays. | `Ces2Core` (default OFF), `CESTurns` (default OFF) | 🧪 shadow (07-12) |
| **Green Light Alert** | Ding + green banner when a held stop releases with **no lead** ("Light is green"); a separate "Car ahead is leaving" note when a close stopped lead departs. Always on, both cars. | Driver request — parity with sunnypilot's green-light chime; refined so a lead blocks the pure green ding (the driver only wants it for actual traffic-light releases). | (no toggle — always on) | ✅ (07-12→13) |
| **Nudgeless lane change** | Lane change without a nudge once the blinker is on (timed hold). | Reduces the hand-torque ritual on long highway drives. | `NudgelessLaneChange` | 🟡 |
| **No disengage on brake** | Tapping the brake doesn't disengage. | Lets the driver dab the brake without dropping openpilot. | `NoDisengageOnBrake` | 🟡 |
| **Blind-spot gate (BSM)** | Tesla Raven blind-spot via `AutopilotStatus` 0x399, logged and available to block nudgeless lane changes into an occupied lane. | Groundwork toward safer automated lane changes on the Raven. | (reuses lane-change gate) | ✅ deployed 07-10 (walk-by flip test pending) |
| **Ford highway follow (LongitudinalExt)** | BluePilot's follow-aware longitudinal shaping (gaining/pacing/trailing lead states) on the Lightning's op-long path. | Smoother highway following when Alpha Long is on; **inert** until then. City stop-and-go "hopping" was fixed by gating the BP acc-message builder on the algorithm actually being in charge. | (Alpha Longitudinal) | 🟡 (07-11→12) |

See `docs/CES.md`, `docs/VTSC.md`, `SHARPCURVE2PNW.md`, `CES_I90.md`, `CURVESLOW2PNW.md`,
`ICBM2PNW.md`, `FORDLONG2PNW.md`, `docs/AUTO2XNOR.md`, `docs/BSM2XNOR.md`.

---

## 3. Driving model

| Feature | What it does | Why | Status |
|---------|--------------|-----|--------|
| **lebowski model + commaai master modeld stack** | Ported commaai/master's modeld stack and the "lebowski" driving model onto the PNW line (with USB-GPU / LFS handling for the large weights). | Keep the perception stack current with upstream; better model behavior. First drive 2026-07-08. | ✅ (07-01→08) |
| **Upstream cherry-picks** | Selected commaai/master fixes (locationd/lagd timestamps, radar-lead prob filter, camerad driver-cam BPS, cruise-fault-not-silent, athena RPC, etc.). | Fold in upstream stability/safety fixes without a full rebase. | ✅ (07-08) |

See `LEBOWSKI2PNW.md`, `UPSTREAM2PNW.md`.

---

## 4. Driver monitoring

| Feature | What it does | Why | Toggle | Status |
|---------|--------------|-----|--------|--------|
| **Glare desensitization (ungated)** | Keeps DM in its active mode through a glare burst instead of dropping to passive wheel-touch (higher POSESTD threshold, longer hi-std fallback, fast dcam-uncertain reset, face-loss grace). | Low sun / windshield glare on I-82/I-84 eastbound drives kept nagging the driver with false "pay attention" prompts. | (ungated) | ✅ (07-06) |
| **Road-gated DM timeout selector** | 3-way selector: **Default** (stock strict) / **Highway** (relaxed only on freeway/divided road) / **Relaxed**. "Off" was renamed **Default** so nobody reads it as "monitoring disabled" (0 = stock-strict; monitoring is never off). | Long highway stretches don't need the same attention cadence as city driving. | `DmMode` (0/1/2, default 0) | ✅ (07-08) |
| **Configurable DM tiers (device-local)** | The actual timeout magnitudes are strict in source; personal (longer) values live only in a device-local `/data/pnw/dm.json` set via a `dm` CLI, clamped [10 s, 4 h]. Relaxed tier is hidden in the UI unless unlocked. | Keeps the shipped/source values conservative and reviewable while letting the owner tune their own device — without baking permissive numbers into the public repo. | `DmMode` + `/data/pnw/dm.json` | ✅ (07-11→12) |

See `DM-CURRENT.md`, `GLARE.md`, `DMROAD2PNW.md`, `DM-VARIABLE.md`. (Note: the old
`SensitiveDriverMonitoring` gate **no longer exists** — DM is configured by `DmMode`, not that key.)

---

## 5. Maps, OSM & location services

| Feature | What it does | Why | Toggle(s) | Status |
|---------|--------------|-----|-----------|--------|
| **OSM data via pfeifer mapd** | Speed limits, road names, and curve speeds from OpenStreetMap (pfeiferj/mapd v2). | Feeds VTSC map-curve braking and the speed-limit display. | `MapdSettings`, `OsmLocal`, `ShowSpeedLimit`, `ShowRoadName` | ✅ |
| **PNW maps default + auto-download** | Default coverage **WA / OR / ID**; the first download auto-arms on a fresh deploy. | The device lives in the Pacific Northwest — no settings-page fiddling needed. | `OsmStateName` ("WA,OR,ID"), `MapdPnwMapsRequested`, `OsmAutoRequested`, `OsmDbUpdatesCheck` | ✅ |
| **"Get map for this location"** | On-demand region download under current GPS, with a covered/greyed indicator. | Pull coverage for a trip outside the default states. | `GetMapForLocation`, `OSMDownloadLocations`/`OSMDownloadBounds` | ✅ |
| **mapd binary persistence** | Installs the mapd Go binary to persistent `/data/mapd/mapd` (outside the git tree). | Auto-update's `git clean` was **deleting** the in-tree binary → silent "no map data" after every update. | — | ✅ (06-29) |
| **"Happening Ahead" overlay** | One display-only daemon merges police reports, highway rest areas, and EV DC-fast chargers (opt-in L2) into a right-side overlay. | Situational awareness for long corridor drives. | `LocationServicesEnabled` (ON), `EvIncludeLevel2` (OFF) | ✅ |
| **Police alerts — Waze parity + quieting** | Live distance, clears when passed, drops other-side-of-highway and stale (>20 min) reports; siren OFF, banner only inside 0.25 mi; real poll-error surfaced in red (e.g. `quota (429)`) instead of a false "Clear". | Field tuning — too many alerts / false "Clear" when the feed was down; siren was startling. | (in overlay) | ✅ (07-01→08) |
| **EV charger refinements** | Compass direction on each charger; near-field roadside chargers show despite the cone geometry; a slow L2 is suppressed when a DC-fast is within 5 mi of it. | Make the charger hints actually useful on the freeway. | `EvIncludeLevel2` | ✅ (07-08) |
| **Corridor rest-area data** | Stable 15 mi rest-area previews by corridor identity; added I-82 and US-12/US-95 data. | pfeifer mapd can't surface rest areas (POIs stripped at tile build), so they ship as curated corridor JSON. | — | ✅ (07-01→08) |
| **Keyless Waze police source** | The police feed now defaults to a keyless caching **proxy** (AWS Lambda + DynamoDB TTL cache behind API Gateway), with automatic fallback to a device-local direct key. | The shared RapidAPI key kept hitting its monthly quota (429) and doesn't scale per-device; the proxy dedups the whole fleet onto one upstream call per cell per TTL window. Also: the key no longer ships in the repo — it lives only in `/data/pnw/location/police_proxy.json`. | — | ✅ (07-12→13) |

See `MAPD-SYSTEM.md`, `LOCATION2PNW.md`, `REST_AREA_DATA.md`, `../comma-connect/WAZE-API.md`.

---

## 6. Connectivity — uploads & networking

Self-hosted: drives upload to **AWS S3 (`comma-connect` bucket)** via an API Gateway that issues
presigned PUTs — **not** comma connect.

| Feature | What it does | Why | Toggle / signal | Status |
|---------|--------------|-----|-----------------|--------|
| **Self-hosted upload gateway** | `API_HOST` pinned to the gateway so uploads can't silently fall back to comma and 412 (= data loss). | A reflash once dropped `API_HOST`; files got marked uploaded without ever reaching S3. | baked default + `launch_env.sh` | ✅ |
| **Two-pass upload** | Pass 1 = small files (qlog/qcamera) on any network; pass 2 = HD video + rlog only on real, non-metered WiFi. | Don't burn LTE/hotspot data on HD video. | — | ✅ |
| **HD-interleave + offroad-only pass 2** | Forces an HD file after every N small uploads; pass-2 HD uploads run **offroad only**. | Video was starved behind a small-file backlog; HD uploads while driving contended with the drive. | — | ✅ (07-08) |
| **Upload-phase indicators** | GREEN CONNECT logo during pass 1 (logs), BLUE during pass 2 (HD); Mbps shown next to CONNECT; clears within ~0.5 s on a WiFi drop. | At-a-glance sense of what's uploading and whether it stalled. | `FirehoseActive`, `FirehoseSpeed` | ✅ (07-09) |
| **Defer HD Video Upload** | ON = hold fcamera/ecamera/dcamera (qlog/rlog/qcam still flow). | Driver request — keep precious WiFi for logs, defer the big video. | `DeferHDVideoUpload` (default OFF) | ✅ (07-06) |
| **Device locator** | The uploader self-reports its LAN IP on each upload request (logged Lambda-side). | The roaming comma's IP changes by network; this makes it findable via CloudWatch. | — | ✅ (07-08) |
| **Perpetual tethering + priority-WiFi + geo-gate** | Device keeps its hotspot up; auto-joins known priority networks only near saved home location(s). | Reliable connectivity across home / hotspot / friends' WiFi without manual switching. | `TetheringEnabled`, `TetheringPriorityNetworks`, `TetheringPriorityWifi` | ✅ |
| **Captive-portal auto-accept** | Walks a MikroTik TOS portal (Peak "Visitor", OSU "osuvisitor" T-<MAC> variant) so uploads work behind it; never blocks LTE/tethering. | University/guest WiFi with a click-through was blocking uploads. | (per-network `portal`) | ✅ (07-08) |
| **LTE throttle guard + NAT fix** | Recovers a throttled/stuck LTE PDN; iptables masquerade so tethered clients get internet. | Keep the pipe alive; tethered phones need a route. | — | ✅ |

Lives in `system/networkd/`, `system/loggerd/`. See `docs/NETWORK2XNOR.md`, `docs/CONNECT2XNOR.md`,
`docs/DEFER_HD_UPLOAD.md`.

---

## 7. Cars, fingerprinting & Ford lateral

| Feature | What it does | Why | Status |
|---------|--------------|-----|--------|
| **4-signal Ford lateral + hardened panda safety** | Ports BluePilot's validated 4-signal steer message (curvature + curvature_rate + path_offset + path_angle) plus matching panda safety onto the Lightning; **requires a panda reflash**. Capability-gated to the Lightning; Tesla safety byte-identical. | Curvature-only control washed out of curves on the Lightning's weaker EPS — this is the difference between "holds the lane" and drifting. The reset-bypass latch was hardened so it can't bypass the disengaged-steering guarantee. | ✅ deployed 07-11 |
| **Ford lateral refinements** | Predicted-curvature blend (turn-exit wind-up fix), human-turn reset (a sustained manual turn flushes desired curvature cleanly), BlueCruise blue-cluster display on engagement. | Smoother turn exits, no lurch after a manual override, honest cluster feedback. All stock-panda-safe (display/curvature-only). | ✅ (07-11) |
| **Auto-recalibrate on car swap** | When the device is moved to a **different** car, `card` clears the 5 learned car-specific params (camera extrinsics + LiveParameters/Torque/Delay) before publishing CarParams — no manual Reset Calibration. Same-car reboots do nothing. | The truck "drove weird" after a Tesla→Lightning swap until a manual reset; this makes the swap seamless. | ✅ (07-13) |
| **Fingerprint hardening** | Never persist a MOCK fingerprint over a known-good car; fleet-identity fallback so an OTA that changes model pointers can't drop the fingerprint; 2025 Lightning fingerprint re-added; poisoned FW-cache DASHCAM-after-toggle fix. | A string of "car shows as MOCK / dashcam" incidents after swaps, toggles, and updates. | ✅ (07-10→11) |
| **card crash resilience** | Manager restarts `card` if it crashes (same policy as UI); fatal crashes are logged to swaglog before dying. | A `card` crash used to strand the drive silently. | ✅ (07-11) |

See `FORDSAFETY2PNW.md`, `FORDLONG2PNW.md`, `docs/RAVEN.md`, `docs/BSM2XNOR.md`, `docs/FORD.md`.

> **Known gap:** the 2025 Lightning fingerprint is present on `testing`/`3devpnw` but a clean build
> from a frozen `4devpnw` fingerprints the Lightning as MOCK/dashcam — verify before relying on it.

---

## 8. UI / overlay

- **Dark-cockpit CES overlay** — a tiered "moving" view with health dots + a full standstill diagnostic
  dump, regrouped into a cockpit card and narrowed (~2/3 width) per driver feedback that the box was too
  wide and the old dump "looked crude". Hardened against `None` values in the CES payload. (07-06→13)
- **Right-side context indicators** (Lightning) — **CALIBRATING x%** (explains why it won't engage,
  especially right after a car-swap recalibration), **STEER: STOCK / NO 4SIG PANDA** (surfaces a
  4-signal fallback or panda-flash mismatch that's otherwise invisible while driving), **RAIN −N**
  (rain-slowdown armed). All silent when healthy. (07-12)
- **Confidence ball** — a comma-4-style ball on the right edge showing the model's own confidence in the
  current plan (teal-green high / amber mid / red low). Display-only. (07-11)
- **LONG MISMATCH red warning** — flags the dangerous "Alpha Long toggled ON but the session is still in
  shadow" state (the persistent CarParams lags a session). (07-12)
- **Live 3-state CES button + Hide-CES-Debug toggle**, **speed-limit / road-name display**, and standard
  update controls (`DisableUpdates`, `SnoozeUpdate`, updater branch/description).

---

## 9. Device / state

- **Auto-update self-deploy** — `/data/openpilot` is a git checkout of `3devpnw`; the stock updater
  fetches (~1.5 h or via `SIGHUP`) and build-on-boot installs at the next reboot/ignition-off.
  opendbc/panda ride as SHA-pinned submodules (`master-pnw`). (07-09→10)
- **Updater runs onroad** — the update fetch was `only_offroad`; it now runs onroad so a download isn't
  stranded when the car is always in use. (07-10)
- **Self-hosted pairing/connect URLs** — the pairing QR and connect URLs point at the self-hosted
  endpoint. (07-13)
- **Persistence guards** — `rm -rf /data/safe_staging/finalized`, `prebuilt`, `DisableUpdates` handled
  by the deploy flow so a reboot can't revert the install or reflash the panda to stock.

---

## 10. Safety posture

- Priority order: **safety > stability > quality > features.**
- New feature toggles default **OFF**; none touch panda safety, with **one deliberate exception**: the
  4-signal Ford lateral **does** ship a custom panda safety mode (reviewed, latch-hardened, Tesla safety
  byte-identical, deployed only after a driver-authorized reflash — see `FORDSAFETY2PNW.md`).
- Feature code **never** branches on `carFingerprint` — it consumes capability views in `pnw_vehicle.py`
  so the Tesla and Lightning can't corrupt each other's behavior.
- Tesla legacy safety mode is `tesla_legacy.h` (counter + checksum validation load-bearing — never
  weakened). Tesla changes are intentionally **not** submitted upstream (too niche).

---

## Last two weeks (2026-06-29 → 2026-07-13) — the arc

A dense two weeks: the curve/CES work matured against real drives, the Lightning became a first-class
car (4-signal lateral + panda reflash, op-long, ICBM), and a lot of stop-and-go/red-light behavior was
root-caused from telemetry.

**Curves & CES (the through-line).**
- **06-29** `sharpcurve2pnw` — VTSC full 500 m lookahead + regen-coast, apex-timed so the curve entrance
  is slowest. Fixes the blind sharp-curve "TAKE CONTROL" beep on I-90 descents.
- **07-01** Live Snoqualmie I-90 retune — lateral target settled at `A_LAT=2.5` **bounded by a new
  highway speed-limit floor** (never trim below the posted limit); map-curve braking (`VtscMapCurves`)
  turned ON by default; CES `Light` mode recommended for the summit over-brake.
- **07-08** CES accel-zone gate (on-ramp merge fix) + lead-pacing gate for the curve trip.
- **07-11** `curveslow-lightning` — per-car curve penalty "hump" for the Lightning's weaker EPS, plus
  `descentcurve2pnw` downhill/left factors; `icbmalign2pnw` gives ICBM the same penalty as VTSC.
- **07-12** `rain2pnw` (`RainMode` selector); `stophold2pnw` red-light lurch guards; `stopintent` /
  `pullaway` fast paths; `ces2core2pnw` CES2 shadow core (`Ces2Core`/`CESTurns`, default OFF).
- **07-13** `standstill2pnw` (no Chill at 0 mph — kills the stop-and-go bucking + launch jolt);
  `greenlead2pnw` splits the green-light ding from a lead-departure note.

**The Lightning becomes first-class.**
- **07-10** ICBM phase 0 (stock-ACC curve slowdown via SET− taps, shadow first); BSM Raven blind-spot
  deployed; fingerprint hardening (MOCK protection, fleet fallback, 2025 fingerprint re-add).
- **07-11** 4-signal Ford lateral + hardened panda safety **deployed** (reflash); Ford `LongitudinalExt`
  highway follow (inert until Alpha Long); ICBM promoted (dec-only), then reworked all weekend
  (map-first, guarded set-speed restore, set-tracking); `fpcache2pnw` DASHCAM-after-toggle fix; `card`
  crash-restart.
- **07-12** ICBM continuous set-tracking + map-scale cap; Ford city-hop (BP brake-hysteresis) fix; the
  dark-cockpit CES overlay; `fordlatui2pnw` right-side indicators.
- **07-13** `calswap2pnw` auto-recalibrate on car swap; `dumpui`/`dmgate` overlay refinements.

**Model, maps, uploads, DM, plumbing.**
- **07-01→08** lebowski driving model + commaai-master modeld stack ported (first drive 07-08);
  upstream cherry-picks.
- **06-29** mapd binary moved to persistent `/data/mapd` (survives auto-update `git clean`).
- **07-01→08** location2pnw police/EV/rest-area tuning; **07-12→13** keyless Waze proxy.
- **07-06** glare2pnw round 2 (keep DM active under glare); **07-08** road-gated `DmMode` selector;
  **07-11→12** dm-variable (device-local `dm.json` tiers + `dm` CLI).
- **07-06** DeferHDVideoUpload; **07-08** captive-portal osuvisitor variant + offroad-only pass-2 +
  device locator; **07-09** upload-phase indicator colors.
- **07-09→10** auto-update self-deploy validated end-to-end; submodule `master-pnw` workflow; updater
  runs onroad. **07-11→12** CREDITS/README/RELEASES/SECURITY docs.

**Surveyed for this doc:** ~150 commits across `3devpnw` / `4devpnw` (the two channel tips), the
`*2pnw` feature branches (sharpcurve, curveslow, descentcurve, icbm* ×5, fordsafety/fordlat/fordlong,
standstill, greenlight/greenlead, stophold/stopintent/pullaway, rain, calswap, wazeproxy, ces2core,
location2pnw, dmroad/dm-variable, connect2pnw, lebowski, upstream2pnw, ball, dumpui/dmgate), plus the
companion repos `pnw-opendbc` / `pnw-panda` (branch `master-pnw`: BSM 0x399, 4-signal lateral +
safety, `FordLatStatus`, ICBM/LongitudinalExt executors), the `docs/` catalog, and the
2026-07-01 → 07-13 drive reports.

---

## Where the detail lives

This file is a high-level index. The authoritative per-feature docs (deploy steps, pitfalls, param
registry) live in the workbench and on the branch:

- `docs/INDEX.md` — complete tree-wide catalog of every doc
- `docs/DEVICE-STATE.md` — source of truth for what's deployed on the 3X (full param registry)
- `docs/CHANGELOG-2026-07-01.md` — the 06-29→07-06 changelog; `CHANGELOG-2026-07-12.md` — the 07-11/12
  "Ford weekend"
- `pnw/CLAUDE.md` — the PNW distribution overview
- Branch-canonical: `SHARPCURVE2PNW.md`, `CES_I90.md`, `CURVESLOW2PNW.md`, `ICBM2PNW.md`,
  `FORDSAFETY2PNW.md`, `FORDLONG2PNW.md`, `LOCATION2PNW.md`, `DMROAD2PNW.md`, `DM-VARIABLE.md`,
  `LEBOWSKI2PNW.md`, `UPSTREAM2PNW.md`, `CREDITS.md`
- Workbench per-effort: `docs/CES.md`, `docs/VTSC.md`, `docs/MAPD-SYSTEM.md`, `docs/AUTO2XNOR.md`,
  `docs/DM-CURRENT.md`, `docs/GLARE.md`, `docs/BSM2XNOR.md`, `docs/NETWORK2XNOR.md`,
  `docs/CONNECT2XNOR.md`, `docs/DEFER_HD_UPLOAD.md`, `docs/FORD.md`, `docs/RAVEN.md`

> **Discrepancy flags (for review):**
> - `DEVICE-STATE.md`'s param registry was last fully audited 2026-07-07 and its "weekend added only
>   `Ces2Core`+`CESTurns`" note (07-12) **omits `RainMode` and `CalibrationCar`**, which are present in
>   `3devpnw:common/params_keys.h` — the registry should be re-audited.
> - Green Light Alert ships with **no param** (always on) — intentional, per the 07-12 driver decision.
> - Status tags here are point-in-time; validate "deployed" claims against `docs/DEVICE-STATE.md` and
>   the live device before relying on them.
