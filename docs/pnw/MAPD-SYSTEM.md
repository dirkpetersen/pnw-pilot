# MAPD-SYSTEM.md — the AS-DEPLOYED mapd architecture (PNW production)

> # 🟢 DEPLOYED — `4devpnw` branch (`~/gh/comma/pnw/pnw-pilot`)
> Official **pfeiferj `mapd` v2.0.6** binary, downloaded-at-launch to a persistent path, publishing
> everything over **cereal** (`mapdOut` / `mapdExtendedOut`), with a thin `mapd_configd` bridge that
> re-exports the legacy in-memory params CES/VTSC/overlays still read. Speed-limit **display** works as
> soon as maps are on disk; speed/curve **control** is opt-in and defaults OFF.

## Supersedes `MAPD2XNOR.md` / `MAPD2PNW.md`

Those docs describe an **obsolete** architecture and should not be trusted for the deployed system:

| Old (MAPD2XNOR / MAPD2PNW) | Now (this doc) |
|---|---|
| `sunnypilot/mapd/mapd_manager.py` bridge + `coverage.py`/`regions.json` | `system/mapd/mapd_configd.py` bridge; region resolution moved **into** the v2 binary |
| Bundled ~9.4 MB **v1** binary in `third_party/mapd_pfeiferj/mapd` (committed to git) | **v2.0.6** binary NOT in git; downloaded-at-launch, sha256-verified, pinned in `mapd_release.json` |
| Wrote `liveMapDataSP` (cereal `CustomReserved8 → LiveMapDataSP`) | Publishes `mapdOut` / `mapdExtendedOut`; **legacy `liveMapDataSP` removed** (`cereal/services.py:109`) |
| Path: `sunnypilot/mapd/` | Path: `system/mapd/` (symlinked into the `openpilot` package on-device) |
| Sunnypilot coverage writer computed map coverage in Python | Coverage comes from the binary's `mapdOut.tileLoaded`; `mapd_configd` just relays it |

Everything below reflects the code actually on `4devpnw`. Where the code is ambiguous I say so.

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
                        writes /data/params:   MapForLocationCovered, MapdPnwMapsRequested
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
`github.com/pfeiferj/mapd`). It owns OSM tile storage (`/data/media/0/osm/offline`), the download
protocol, and **region resolution** (previously done in the deleted sunnypilot `coverage.py`). It
publishes `mapdOut` (primary driving output, 20 Hz) and `mapdExtendedOut` (download progress + curve
path, 1 Hz), and subscribes `mapdIn` (settings + download/trigger commands). All ship with speed/curve
**control disabled** by default; it downloads no maps on its own.

### 3. The bridge — `system/mapd/mapd_configd.py`

A small `always_run` daemon (`process_config.py:122`, TICI-only) that does three jobs:

1. **cereal → legacy mem params.** CES (`selfdrive/controls/lib/ces_pnw`) and the on-road overlay still
   read the in-memory params the OLD binary used to write. `mapd_configd` translates the v2 cereal
   output into `/dev/shm/params`: `LastGPSPosition` (from the GPS service), `MapSpeedLimit`
   (`mapdOut.speedLimit`, m/s), `RoadName` / `WayRef` / `RoadContext` (road identity/class), and
   `MapTargetVelocities` (from `mapdExtendedOut.path` — the per-point curve list VTSC/CES consume)
   (`mapd_configd.py:63-84`).
2. **Download status + coverage.** Publishes `MapDownloadStatus` (`"OK"`/`"downloading X/Y"`/`"incomplete
   X/Y"`/`"none"`) for the debug overlay (`mapd_configd.py:86-103`), and `MapForLocationCovered` — the
   greyout for the "Get map for this location" toggle: covered = no GPS fix OR `mapdOut.tileLoaded`,
   written only on change (`mapd_configd.py:105-116`). **This replaces the deleted sunnypilot coverage
   writer.**
3. **One-shot PNW map auto-download.** On the first unmetered Wi-Fi connection it sends a `mapdIn`
   download for `us_state.WA,us_state.OR,us_state.ID` (`PNW_DOWNLOAD`, `mapd_configd.py:24`), guarded by
   `MapdPnwMapsRequested` so it fires once. It re-sends each loop until `mapdExtendedOut.downloadProgress`
   shows the pull started (messages can be missed before mapd's `mapdIn` socket is up), then sets the
   guard (`mapd_configd.py:119-141`). It is the **only** `mapdIn` publisher, and it **never enables
   control** — only display/download.

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
| `MapdPnwMapsRequested` | BOOL, unset(false) | One-shot guard: the PNW WA/OR/ID auto-download is requested only once (`params_keys.h:80-81`) |
| `RoadName` | STRING | mapd road name, bridged to mem params (`params_keys.h:85`) |
| `WayRef` | STRING | mapd road ref (e.g. `"I 5"`) bridged to mem (`params_keys.h:86`) |
| `RoadContext` | STRING | Road class `'freeway'`/`'city'`/`'unknown'` — freeway-gates location lookups (`params_keys.h:87`). NB: `mapd_configd` writes `str(mo.roadContext)`, i.e. the enum's numeric value, not the name — see gotcha |
| `MapDownloadStatus` | STRING, CLEAR_ON_MANAGER_START | Live OSM DB download state for the debug overlay (`params_keys.h:88`) |
| `OsmStateName` | STRING, **`"WA,OR,ID"`** | Default coverage — **2-letter codes** (the binary's STATE_BOXES key), NOT full names (`params_keys.h:92`) |
| `OSMDownloadLocations` | JSON | Requested OSM download locations (`params_keys.h:95`) |
| `ShowSpeedLimit` | BOOL, **`"0"`** | Speed-limit display toggle; default OFF (`params_keys.h:99`) |
| `GetMapForLocation` | BOOL, **`"0"`** | "Get map for this location" — ON downloads the region under current GPS (`params_keys.h:101`) |
| `MapForLocationRegion` | STRING, CLEAR_ON_MANAGER_START | Region code under current GPS; `""` = covered/unknown (`params_keys.h:102`) |
| `MapForLocationCovered` | BOOL, CLEAR_ON_MANAGER_START | True when current GPS is already covered → UI greys the toggle. Written by `mapd_configd` (`params_keys.h:103`) |
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
- **First-run map download:** `mapd_configd` fires the WA/OR/ID one-shot on first unmetered Wi-Fi,
  guarded by `MapdPnwMapsRequested`. Re-arm by resetting that param to 0 (re-read each loop, no restart
  needed). On-demand region download is driven by `GetMapForLocation` (greyed via `MapForLocationCovered`
  when already covered).

---

## Known gotchas

1. **msgq-prefix boot race (fixed).** gomsgq auto-detects the `/dev/shm` naming by `stat`-ing
   `msgq_logMessage` at startup. If mapd starts **before** that segment exists, it falls back to
   UNPREFIXED names and is silently **deaf + mute all session** — no speed limits, no map curves, no road
   context (observed I-82, 2026-07-06). Fixed by forcing `USE_MSGQ_PREFIX=true` in the exec
   (`process_config.py:117-121`), which matches this tree's `msgq.cc` (`/dev/shm/msgq_<name>`). Fix
   commit: **`2fc78f0fbd`** ("glare2pnw + mapd msgq prefix: … force mapd shm prefix").
2. **Region resolution moved into the binary.** The old sunnypilot `coverage.py` / `regions.json` (US
   states vs. nations, the BC-is-whole-of-Canada problem) is gone from openpilot; the v2 binary owns the
   region table and download protocol. Openpilot only passes `us_state.XX` download keys and reads
   `tileLoaded`/`downloadProgress` back.
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
