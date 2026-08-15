# MAPD-SYSTEM.md — the AS-DEPLOYED mapd architecture (PNW production)

> # 🟢 DEPLOYED — `mapdstate2pnw` branch (`~/gh/comma/pnw/pnw-pilot`)
> Official **pfeiferj `mapd` v2.0.6** binary, downloaded-at-launch to a persistent path, publishing
> everything over **cereal** (`mapdOut` / `mapdExtendedOut`), with a `mapd_configd` bridge that
> re-exports the legacy in-memory params CES/VTSC/overlays still read AND drives the map-DOWNLOAD
> policy: GPS-driven, on-demand, whole-STATE. Speed-limit **display** works as soon as maps are on
> disk; speed/curve **control** is opt-in and defaults OFF.

## Download policy (mapdstate2pnw, supersedes the fixed-state auto-download below)

Coverage is no longer a fixed default state set. Each loop, `mapd_configd.py` checks whether mapd has
a tile loaded for the car's current GPS fix. As soon as it doesn't (a fresh device, or the car just
crossed into a state with no map data), it downloads the **whole state the car is currently in** — on
any network, metered or not. It never downloads a tiny per-tile area (that would leave the driver
stranded a few miles outside the tile) and never downloads the whole US as a single "country" (a US
fix always resolves to its enclosing state, never the national box). A fix outside the US falls back
to the enclosing **nation** (mapd has no province-level granularity, so e.g. British Columbia pulls
the whole-Canada nation download). Region resolution is done in openpilot
(`system/mapd/coverage.py` + `regions.json` — see below), not by the binary.

The old "Get map for this location" on-demand toggle is gone; its param is repurposed as **"Refresh
this location map"** — an action that deletes the current region's downloaded tiles so the automatic
uncovered-state download re-fetches them (for a stale/corrupted map), rather than a way to request a
download that now happens automatically.

## Supersedes `MAPD2XNOR.md` / `MAPD2PNW.md`

Those docs describe an **obsolete** architecture and should not be trusted for the deployed system:

| Old (MAPD2XNOR / MAPD2PNW) | Now (this doc) |
|---|---|
| `sunnypilot/mapd/mapd_manager.py` bridge + `coverage.py`/`regions.json` | `system/mapd/mapd_configd.py` bridge; `system/mapd/coverage.py` + `regions.json` (re-ported, mapdstate2pnw) resolve GPS -> region + download key in openpilot; the binary still owns tile storage/download protocol and `tileLoaded` |
| Bundled ~9.4 MB **v1** binary in `third_party/mapd_pfeiferj/mapd` (committed to git) | **v2.0.6** binary NOT in git; downloaded-at-launch, sha256-verified, pinned in `mapd_release.json` |
| Wrote `liveMapDataSP` (cereal `CustomReserved8 → LiveMapDataSP`) | Publishes `mapdOut` / `mapdExtendedOut`; **legacy `liveMapDataSP` removed** (`cereal/services.py:109`) |
| Path: `sunnypilot/mapd/` | Path: `system/mapd/` (symlinked into the `openpilot` package on-device) |
| Sunnypilot coverage writer computed map coverage in Python | `mapdOut.tileLoaded` still tells `mapd_configd` whether the current fix is covered; `mapd_configd` now ALSO decides (in Python, via `coverage.py`) which state/nation to request or delete when it isn't |

Everything below reflects the code actually on the branch. Where the code is ambiguous I say so.

---

## Architecture

```
mapd_release.json  ──(url+sha256)──►  installer.py  ──►  /data/mapd/mapd   (persistent binary)
                                                              │
                                        manager execs it (NativeProcess "mapd")
                                                              │  cereal pub/sub
                          mapdIn  ◄──────────────────────────┤────────────►  mapdOut  (20 Hz)
                        (settings/                            │              mapdExtendedOut (1 Hz)
                         download                             ▼
                         triggers)                     mapd_configd.py  (bridge daemon)
                                                              │
                        writes /dev/shm/params: LastGPSPosition, MapSpeedLimit,
                        RoadName, WayRef, RoadContext, MapTargetVelocities, MapDownloadStatus
                        writes /data/params:   MapForLocationCovered
                        reads system/mapd/coverage.py + regions.json (GPS -> region + download key)
                                                              │
   consumers ──►  speed_limit.py (UI, reads mapdOut)   ·   longitudinal_planner (mapdOut.suggestedSpeed cap)
                  vtsc_pnw (reads MapTargetVelocities)  ·   ces_pnw (reads mem map params)
```

### 1. Installer — `system/mapd/installer.py`

The ~20 MB binary is **not vendored** in git (keeps clones/updates small). `mapd_release.json` at the
repo root pins `version` / `url` / `sha256` / `size` / `install_path`. `ensure_mapd()`:

- Resolves paths from the file's **realpath**, not `BASEDIR`, because `system/` is a symlink into the
  `openpilot` package on-device (`installer.py:26-32`).
- Installs to a **persistent path OUTSIDE the git tree**: `MAPD_BINARY = /data/mapd/mapd` when `/data`
  is writable, else the legacy in-tree `selfdrive/mapd` fallback for dev/CI (`installer.py:54-58`).
  This is deliberate — an auto-update's `git clean -xdff` would delete an untracked in-tree binary and
  force a flaky boot re-download (the 2026-06-29 incident, `installer.py:40-47`).
- Is **atomic + idempotent**: download to `dest + ".download"`, sha256-verify against the pin, `chmod
  0o755`, `os.replace()` onto `dest`; a no-op when the pinned binary is already installed and valid
  (`installer.py:78-147`). If a valid binary already exists at the legacy in-tree path it is **copied
  (not re-downloaded)** to the persistent dir first — this is what lets the fix survive a boot with no
  network (`installer.py:93-108`). 3 retries; static temp name so a hard-kill leaves at most one stale
  temp, never accumulating 20 MB orphans.
- `python3 -m openpilot.system.mapd.installer --check` reports status without downloading.

**Who runs it:** gated in the manager by `mapd_running()` = `os.path.exists(MAPD_BINARY)`
(`process_config.py:53-54`) — manager never execs a missing file. (Where `ensure_mapd()` is invoked on
boot was not re-verified here; the design intent per the module docstring is download-at-launch.)

### 2. The binary — pfeiferj `mapd` v2.0.6

Registered as a `NativeProcess` (`process_config.py:121`):

```py
NativeProcess("mapd", "selfdrive", ["/usr/bin/env", "USE_MSGQ_PREFIX=true", MAPD_BINARY], mapd_running)
```

Self-contained Go binary (upstream source for context/link only: `~/gh/comma/mapd`,
`github.com/pfeiferj/mapd`). It owns OSM tile storage (`/data/media/0/osm/offline`) and the download
protocol (fetch + extract a `download_menu.json` region key, e.g. `us_state.WA` / `nation.CA`, into
2°-grid tile directories under `offline/<lat>/<lon>/`); openpilot's `coverage.py` (mapdstate2pnw) does
GPS -> region + download-key resolution and decides WHEN to ask for a download or delete. It publishes
`mapdOut` (primary driving output, 20 Hz, incl. `tileLoaded` — whether it has a tile for the current
fix) and `mapdExtendedOut` (download progress + curve path, 1 Hz), and subscribes `mapdIn` (settings +
download/trigger commands). All ship with speed/curve **control disabled** by default; it downloads no
maps on its own — every download is a `mapdIn` request from `mapd_configd`.

### 3. The bridge — `system/mapd/mapd_configd.py`

A small `always_run` daemon (`process_config.py:122`, TICI-only) that does three jobs:

1. **cereal → legacy mem params.** CES (`selfdrive/controls/lib/ces_pnw`) and the on-road overlay still
   read the in-memory params the OLD binary used to write. `mapd_configd` translates the v2 cereal
   output into `/dev/shm/params`: `LastGPSPosition` (from the GPS service), `MapSpeedLimit`
   (`mapdOut.speedLimit`, m/s), `RoadName` / `WayRef` / `RoadContext` (road identity/class), and
   `MapTargetVelocities` (from `mapdExtendedOut.path` — the per-point curve list VTSC/CES consume)
   (`mapd_configd.py:63-84`).
2. **Download status + coverage.** Publishes `MapDownloadStatus` (`"OK"`/`"downloading X/Y"`/`"incomplete
   X/Y"`/`"none"`) for the debug overlay, and `MapForLocationCovered` — now the greyout for the
   **"Refresh this location map"** toggle (enabled only when covered, since there's nothing to refresh
   otherwise): covered = no GPS fix OR `mapdOut.tileLoaded`, written only on change. **This replaces the
   deleted sunnypilot coverage writer.**
3. **GPS-driven, on-demand, whole-STATE auto-download (mapdstate2pnw).** Each loop, if there's a GPS
   fix but no tile loaded ("uncovered"), it resolves the region under the fix via `coverage.
   region_and_key_for_gps()` (`system/mapd/coverage.py` + `regions.json`, ported from the deleted
   sunnypilot module) and sends a `mapdIn` download for that region's key (`us_state.<CODE>` for a US
   state, `nation.<CODE>` for a non-US fix — **never** the whole US as a country: `region_for_gps`
   structurally excludes the "US" nation box). It requests on **any** network — no unmetered-Wi-Fi gate
   — and re-arms whenever a *different* region goes uncovered, or the region has been uncovered for over
   `REGION_RESEND_INTERVAL_S` (60 s, a bounded retry in case a request got missed), but never while a
   download is already `active`. It is the **only** `mapdIn` publisher, and it **never enables control**
   — only display/download. The **"Refresh this location map" toggle** (`RefreshLocationMap` param)
   deletes the current region's tile directories (via `_grid_cells_for_bbox` — mirrors the binary's own
   2°-grid tile layout) when triggered, so this same auto-download logic re-fetches it.

### 4. Consumers

| Consumer | Reads | Behavior |
|---|---|---|
| Speed-limit display | `mapdOut` directly (`selfdrive/ui/onroad/speed_limit.py`, subscribed via `ui_state.py:58`) | On-road speed-limit sign; gated by `ShowSpeedLimit` (`ui_state.py:73,186`) |
| Longitudinal planner | `mapdOut.suggestedSpeed` (`longitudinal_planner.py:154-157`) | Caps `v_cruise` down to the mapd suggested speed. **Inert by default** — `suggestedSpeed` is 0 unless a mapd control is enabled in `MapdSettings`. Only ever *reduces* speed, only when `openpilotLongitudinalControl`, gated on `alive` (not just `valid`) + a low-speed floor |
| VTSC (`vtsc_pnw`) | `MapTargetVelocities` mem param (`vtsc_controller.py:117-123`) | Optional map-curve fold (MTSC), gated by `VtscMapCurves` (default ON). Folds pfeiferj map curve speeds into vision turn-speed control for earlier braking |
| CES (`ces_pnw`) | mem map params (`MapTargetVelocities`, `MapSpeedLimit`, `LastGPSPosition`) | Map-curve trigger + overlay road line |

---

## Params (`common/params_keys.h`)

| Key | Type / default | Purpose |
|---|---|---|
| `MapdSettings` | JSON, no default | The v2 binary's settings store — **the binary reads/writes it directly** and reloads on a `mapdIn reloadSettings`. All speed/curve controls live here and default OFF (`params_keys.h:77-79`) |
| `RoadName` | STRING | mapd road name, bridged to mem params (`params_keys.h:85`) |
| `WayRef` | STRING | mapd road ref (e.g. `"I 5"`) bridged to mem (`params_keys.h:86`) |
| `RoadContext` | STRING | Road class `'freeway'`/`'city'`/`'unknown'` — freeway-gates location lookups (`params_keys.h:87`). NB: `mapd_configd` writes `str(mo.roadContext)`, i.e. the enum's numeric value, not the name — see gotcha |
| `MapDownloadStatus` | STRING, CLEAR_ON_MANAGER_START | Live OSM DB download state for the debug overlay (`params_keys.h:88`) |
| `OsmStateName` | STRING, **`"WA,OR,ID"`** | **Dead param** — leftover from the pre-v2.0.6 `mapd_manager` era; nothing in the current tree reads it. Coverage is now decided per-fix by `coverage.region_and_key_for_gps()`, not by a configured default list |
| `OSMDownloadLocations` | JSON | Requested OSM download locations (`params_keys.h:95`) |
| `ShowSpeedLimit` | BOOL, **`"0"`** | Speed-limit display toggle; default OFF (`params_keys.h:99`) |
| `RefreshLocationMap` | BOOL, **`"0"`** | "Refresh this location map" — ON deletes the current region's downloaded tiles so the auto-download re-fetches it. Repurposed from the old "Get map for this location" (`params_keys.h:113`) |
| `MapForLocationCovered` | BOOL, CLEAR_ON_MANAGER_START | True when current GPS is already covered → UI enables the Refresh toggle only then (inverted from the old on-demand-download greyout). Written by `mapd_configd` (`params_keys.h:116`) |
| `OsmLocationName` | STRING | Named OSM location (`params_keys.h:91`) |
| `OsmDbUpdatesCheck` | BOOL | OSM DB update check flag (`params_keys.h:89`) |
| `OsmDownloadedDate` | STRING | Last OSM download date (`params_keys.h:90`) |
| `OsmLocal` | BOOL | Local OSM flag (`params_keys.h:93`) |
| `OsmAutoRequested` | BOOL | Auto-download requested flag (`params_keys.h:94`) |
| `OSMDownloadBounds` | STRING | OSM download bounding box (`params_keys.h:96`) |
| `ShowRoadName` | BOOL, `"1"` | Road-name display; default ON (`params_keys.h:98`) |
| `Offroad_OSMUpdateRequired` | JSON, CLEAR_ON_MANAGER_START | Offroad alert that an OSM download is needed (`params_keys.h:104`) |
| `VtscMapCurves` | BOOL, `"1"` | Fold pfeiferj map curve speeds into VTSC (MTSC); default ON (`params_keys.h:121`) — a VTSC key, not owned by mapd, but the map-curve consumer switch |

> Several `Osm*` keys (`OsmDbUpdatesCheck`, `OsmDownloadedDate`, `OsmLocationName`, `OsmLocal`,
> `OsmAutoRequested`, `OSMDownloadBounds`, `OSMDownloadLocations`) are carried over from the mapd2xnor
> foundation. Which of them the v2 binary actually still reads was **not verified here** — treat as
> legacy/back-compat unless proven live.

---

## Cereal / message flow (`cereal/custom.capnp`, `cereal/services.py`)

The v2 cereal types are **copied VERBATIM (same `@`-IDs)** from
`github.com/pfeiferj/mapd cereal/custom/custom.capnp` so the prebuilt binary and openpilot agree on wire
layout. They **reuse the CustomReserved17/18/19 wire IDs** (`MapdExtendedOut` @0xa30662…,
`MapdIn` @0xc86a3d…, `MapdOut` @0xa4f1eb…); helper structs/enums are new named types
(`custom.capnp:76-201`).

- **`MapdOut`** (24 fields, `custom.capnp:176-201`): `wayName`, `wayRef`, `roadName`, `speedLimit`,
  `nextSpeedLimit`(+`Distance`), `advisorySpeed`, `tileLoaded`, `suggestedSpeed`,
  `speedLimitSuggestedSpeed`, `visionCurveSpeed`, `mapCurveSpeed`, `roadContext` (enum
  freeway/city/unknown), `waySelectionType`, `speedLimitAccepted`, hazards, lanes, etc.
- **`MapdExtendedOut`** (`custom.capnp:102-106`): `downloadProgress` (`MapdDownloadProgress`),
  `settings` (Text), `path` (List(`MapdPathPoint{latitude,longitude,curvature,targetVelocity}`)).
- **`MapdIn`** (`custom.capnp:163-168`): `type` (`MapdInputType` enum, ~39 commands incl. `download`,
  `reloadSettings`, `cancelDownload`, the `set*` control toggles), plus `float`/`str`/`bool` args.

Service registration (`cereal/services.py:106-112`) — **queue size MUST be MEDIUM (2 MB)** to match the
binary's `settings/const.go` `QUEUE_SIZE_MEDIUM`:

```py
"mapdOut":         (True, 20., 20, QueueSize.MEDIUM),   # logged, primary driving output
"mapdExtendedOut": (False, 1., None, QueueSize.MEDIUM), # not logged (download progress + path)
"mapdIn":          (True, 0., None, QueueSize.MEDIUM),  # event-driven, openpilot -> mapd
# Legacy sunnypilot liveMapDataSP removed.
```

---

## On-device deploy / persistence

- **The binary lives at `/data/mapd/mapd`** — persistent, outside the git working tree — so it survives
  every auto-update (`git clean -xdff`) and reboot. No re-download unless the pin changes. This is the
  central design fix vs. the old in-tree binary.
- **To upgrade mapd:** bump `version` + `url` + `sha256` + `size` in `mapd_release.json` (get the sha
  with `sha256sum mapd`); the installer picks up the new pin and atomically replaces the binary. Nothing
  else changes.
- **Arming control:** `MapdSettings` (JSON, all controls default OFF) is what enables speed-limit / map
  / vision curve *control*. Display (`ShowSpeedLimit`) and map curves in VTSC (`VtscMapCurves`) are
  independent of it.
- **Map download:** `mapd_configd` downloads the state (or nation) the car is currently in as soon as it
  detects it's uncovered — no settings-page action needed, on any network. Re-arms automatically on
  every fresh uncovered region; "Refresh this location map" (`RefreshLocationMap`, enabled only when
  `MapForLocationCovered`) deletes the current region's tiles to force a clean re-download.

---

## Known gotchas

1. **msgq-prefix boot race (fixed).** gomsgq auto-detects the `/dev/shm` naming by `stat`-ing
   `msgq_logMessage` at startup. If mapd starts **before** that segment exists, it falls back to
   UNPREFIXED names and is silently **deaf + mute all session** — no speed limits, no map curves, no road
   context (observed I-82, 2026-07-06). Fixed by forcing `USE_MSGQ_PREFIX=true` in the exec
   (`process_config.py:117-121`), which matches this tree's `msgq.cc` (`/dev/shm/msgq_<name>`). Fix
   commit: **`2fc78f0fbd`** ("glare2pnw + mapd msgq prefix: … force mapd shm prefix").
2. **Region resolution is back in openpilot (mapdstate2pnw).** `system/mapd/coverage.py` +
   `regions.json` (US states vs. nations, the BC-is-whole-of-Canada problem) are re-ported from the
   deleted sunnypilot module — the binary still owns tile storage/the download protocol/`tileLoaded`,
   but openpilot now decides WHICH region to request or delete. **Collision gotcha:** the states and
   nations tables share 20 two-letter codes (`CA` = California AND Canada, `ID` = Idaho AND Indonesia,
   …) — `coverage.region_and_key_for_gps()` resolves this correctly by tracking which table matched
   during the lookup; re-deriving "is this a US state?" from a bare region code afterward (e.g. via
   `is_us_state(code)`) silently picks the wrong one for those 20 codes and is deliberately documented
   as unsafe to use that way.
3. **`RoadContext` param is the enum's numeric value, not its name.** `mapd_configd` writes
   `str(mo.roadContext)` (`mapd_configd.py:76`), so the `RoadContext` param holds `"0"`/`"1"`/`"2"`
   (freeway/city/unknown), while the `params_keys.h:87` comment describes it as
   `'freeway'|'city'|'unknown'`. Consumers must map the number, or this is a latent mismatch — **flagged,
   not fixed** (out of scope).
4. **Control is inert unless armed.** `longitudinal_planner`'s `mapdOut.suggestedSpeed` cap and every
   `set*` control in `MapdIn` do nothing until enabled in `MapdSettings` (default OFF). Speed-limit
   *display* and VTSC map-curve fold are the only things live by default.
5. **`ensure_mapd()` invocation point not re-verified** in this pass — the module docstring and manager
   gating imply download-at-launch, but exactly which boot hook calls `ensure_mapd()` was not traced. If
   the binary is ever missing on a fresh device, check that call site first.
