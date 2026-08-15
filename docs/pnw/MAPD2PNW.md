# MAPD2PNW — foundational mapd infrastructure for PNW (on-demand per-location download)

> # ⛔ SUPERSEDED — see `MAPD-SYSTEM.md`
> Everything below is a **historical record** of the original `mapd2pnw` branch (v0.11.2 lineage,
> never deployed): the bundled v1 binary, the `sunnypilot/mapd/mapd_manager.py` bridge, its fixed
> `OsmStateName="WA,OR,ID"` default, and its Priority-WiFi-gated download. **None of that ships
> today.** The deployed system is the v2.0.6 binary + `system/mapd/mapd_configd.py` (see
> `MAPD-SYSTEM.md`), and as of `mapdstate2pnw` the download policy itself has also changed: coverage
> is no longer a fixed default state list — `mapd_configd` downloads whichever state (or, outside the
> US, nation) the car's GPS fix is currently uncovered in, on any network, every time it detects a new
> uncovered region. `GetMapForLocation` / `MapForLocationRegion` (the params this doc describes below)
> are gone; `RefreshLocationMap` (repurposed from `GetMapForLocation`) now means "delete my current
> map so it re-downloads", not "let me request a download". The `coverage.py` / `regions.json` design
> described here DID get re-ported (`system/mapd/coverage.py`), verbatim in its GPS-resolution logic —
> that part of this doc is still an accurate description of how region lookup works.

**Branch:** `mapd2pnw` (off `pnwtest3`, openpilot v0.11.2). **Foundational** — CES (map-curve half) and
the speed-limit warning **depend on this branch**. Built but NOT deployed; device untouched.

> **⚠️ Copied from sunnypilot / pfeiferj — may need maturing.** The OSM map engine is the bundled
> pfeiferj `mapd` binary (`third_party/mapd_pfeiferj/mapd`, ~9.4 MB) + the sunnypilot `mapd_manager`
> bridge, ported as-is. It works for the PNW use case but is not a from-scratch implementation; expect
> rough edges (notably Canadian-province granularity — see below) and treat the binary as a black box
> whose region table + download protocol we adapt to, not control.

## What this branch provides (the foundation)
- The mapd subsystem: `sunnypilot/mapd/` (manager + OSM map data) + the pfeiferj binary, registered as
  two managed processes (`mapd` native + `mapd_manager`), publishing `liveMapDataSP` (OSM speed limits
  + road name) and writing `MapTargetVelocities` (the CES map-curve input) to the in-memory param store.
- `cereal`: `CustomReserved8 → LiveMapDataSP`; `liveMapDataSP` service. `Paths.mapd_root()`.
- OSM/mapd params in `common/params_keys.h` (`OsmStateName="WA,OR,ID"`, `MapTargetVelocities`,
  `ShowSpeedLimit`, etc.).

## The two PNW behavior changes on top of the sunnypilot base

### 1. Default coverage = WA / OR / ID (US states), download gated on a Priority Network WiFi
- `PNW_STATES = ["WA","OR","ID"]` is the shipped default (`OsmStateName`).
- **Download only runs when connected to a configured Priority Network WiFi** (`network2xnor`'s
  `TetheringPriorityNetworks`/`TetheringPriorityWifi`). `mapd_manager._on_priority_wifi()` checks the
  active `nmcli` wireless connection against the saved priority SSIDs and **fails CLOSED** — so a
  multi-hundred-MB (or, for a nation, multi-GB) map download never burns cellular/hotspot data. This is
  why mapd2pnw depends on the network foundation in `pnwtest3`.
- Download is also purpose-gated: it only arms if a map is actually wanted (speed-limit display on, or
  the user has on-demand-added a region).

### 2. "Get map for this location" — on-demand per-location download (the BC solution)
**Why:** the pfeiferj binary's region table keys US **states** individually (WA/OR/ID each downloadable)
but has **no Canadian province** — Canada exists only as a whole-nation box (`"CA": "Canada"`,
~multi-GB). So British Columbia can't be a small default; downloading it means the whole of Canada. The
on-demand toggle sidesteps this: you download whatever region you're physically in, only when you ask.

- **`sunnypilot/mapd/coverage.py`** (pure): `regions.json` (extracted from the binary — 52 US states +
  175 nations, kept in **separate sub-tables** because state and nation 2-letter codes COLLIDE, e.g.
  `ID` = Idaho-the-state AND Indonesia-the-nation). `region_for_gps(lat,lon)` returns the covering
  region (US **state preferred**; the "US" nation box is excluded from the nation fallback because it
  overflows north into southern Canada and would mis-resolve Vancouver BC → US). `is_covered(region,
  downloaded)`.
- **`mapd_manager`** each loop: `update_location_coverage()` writes `MapForLocationRegion` (code under
  current GPS) + `MapForLocationCovered` (bool) for the UI. `on_demand_download()`: when the user flips
  `GetMapForLocation` ON and the current region isn't already downloaded, it appends the region to
  `OsmStateName` (persistent), logs it, and clears the toggle (one-shot). The normal arming loop then
  downloads it — still priority-WiFi-gated.
- **UI toggle** ("Get map for this location", `toggles.py`): **greyed out / inactive** when
  `MapForLocationCovered` (current GPS already covered, or no fix/unknown region); **enabled but OFF**
  when uncovered. Flipping it on triggers the download of the current region (a US state, or a whole
  nation like Canada for BC).

  Verified coverage logic: Seattle→WA, Portland→OR, Boise→ID(Idaho), Vancouver/Calgary→CA(Canada),
  mid-Pacific→None(inactive).

## Params added (historical — see the SUPERSEDED banner above for current names)
`GetMapForLocation` (BOOL, the toggle) · `MapForLocationRegion` (STRING, cleared on manager start) ·
`MapForLocationCovered` (BOOL, cleared on manager start) · `ShowSpeedLimit` (BOOL "0", registered here
as the foundation; consumed by the speed-limit display).

**Today:** `GetMapForLocation` and `MapForLocationRegion` no longer exist (removed, mapdstate2pnw —
nothing read `MapForLocationRegion` even before removal). `MapForLocationCovered` survives, repurposed
as the greyout for `RefreshLocationMap`. `ShowSpeedLimit` survives only as a migration source for its
opt-out successor `NoSpeedLimitDisplay`.

## Dependency chain
`network2xnor` (priority WiFi) → **`mapd2pnw`** (this) → CES map-curve half + speed-limit warning.

## Known immaturity / TODO
- **No Canadian-province granularity** — BC = whole-Canada download. A future mapd binary exposing CA
  provinces would let `coverage.py` + `on_demand_download` narrow it to BC. The binary's custom-bounds
  path (`OSMDownloadBounds`) reportedly nil-panics (per CES.md) — re-verify before trying a BC bbox.
- Bounding-box coverage is approximate near borders (rectangles overlap); the US-nation-exclusion
  heuristic handles the US/Canada case but other border regions may be coarse.
- Not deployed / not driven. Deploy via the installer (capnp + params_keys.h changed → needs a rebuild).
