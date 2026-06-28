# LOCATION2PNW.md — "Happening Ahead" location services (display-only)

**Branch:** `location2pnw` (off `4devpnw`). **Spec:** `~/gh/comma/other/waze-test/LOCATION_SERVICES_DESIGN.md`.
**Status:** v1 **code built + Gemini-reviewed (gemini-pro-latest) + geo unit-tested**; **NOT merged, NOT
deployed.** Display-only, panda-safe — never touches control/safety, never blocks engagement.

One daemon merges three "what's ahead on the highway" sources into one lower-left overlay:
**police** (live Waze proxy), **rest areas** (static), **EV fast chargers** (static).

## What was built (v1 code — design §8 items 1–3)

| File | Role |
|------|------|
| `system/mapd/mapd_configd.py` | **bridge extended** — publishes `RoadName`/`WayRef`/`RoadContext` to `/dev/shm` (no new msgq sub) |
| `common/params_keys.h` | `LocationServicesEnabled` (BOOL **default ON**), `LocationServices` (mem JSON), `WayRef`/`RoadContext` (mem) |
| `system/location_services/geo.py` | **pure** geometry: haversine/bearing + along-track/perp projection onto the mapd path + forward-cone fallback; "behind = None" |
| `system/location_services/location_servicesd.py` | the daemon: 3 isolated updaters (police network **thread**, static rest, static EV), merge → publish `LocationServices` |
| `system/location_services/tests/test_geo.py` | 8 geo unit tests (all pass standalone) |
| `selfdrive/ui/onroad/location_services_status.py` | lower-left "HAPPENING AHEAD" overlay (mirrors `ces_status.py`) |
| `selfdrive/ui/onroad/augmented_road_view.py` | instantiate + render the overlay |
| `system/manager/process_config.py` | `PythonProcess("location_servicesd", …, always_run, enabled=TICI)` |
| `selfdrive/selfdrived/selfdrived.py` | added to `NON_ESSENTIAL_PROCS` → **never blocks engagement** |

## Decisions honored (design §0/§5/§7)
- Master toggle **default ON**; police **polls always** (any connection); a failed poll shows **"—"/no-data**,
  never a false **"Clear"**. EV qualifies within **1 mi perpendicular**; kW/network from the local file.
- **Police direction hint only when Waze gives a real reporter-heading (`magvar`); else silent.** The tested
  RapidAPI proxy nulls `magvar`, so today the hint never shows — by design.
- **HARD RULE upheld:** the police network poll runs in its **own thread** (backoff + defensive parse); the
  static rest/EV lines update every tick regardless — a hung/403 poll can never blank the overlay.

## Deviations / choices (flagged for the user)
1. **API key IS shipped in-distribution (user-approved 2026-06-28, testing phase).** `DEFAULT_PROXY` in
   `location_servicesd.py` carries the RapidAPI key/host/url so police polling works out of the box; a
   **`/data/pnw/location/police_proxy.json`** file, if present, OVERRIDES it (picked up on daemon restart).
   It's a rotatable third-party proxy key, not a device/account secret. Longer term → override-file-only.
2. **Plain text labels, not emoji** (👮/🛏/⚡) — the openpilot Inter font has no emoji glyphs (would render as
   tofu). Swap to an icon atlas later if pictograms are wanted.

## Gemini review (gemini-pro-latest) — NEEDS CHANGES → fixed
- ✅ Confirmed: thread isolation correct (no race), geometry correct, defensive parsing robust, freeway gate +
  nodata-vs-clear correct, NON_ESSENTIAL (won't block engagement), key not leaked, Ratekeeper paces when off.
- 🔴 **Bug #1 (fixed):** the staleness check ran *after* nearest-ahead, so a near-but-ancient report could mask
  a fresh one further ahead → false "Clear". Now stale reports are filtered **before** nearest-ahead.
- ⚪ Bugs #2/#3 (MapTargetVelocities/LocationServices "remain bytes") were **false positives** — verified on the
  live device that JSON-typed params auto-deserialize (`get()` returns list/dict, like the live `CESStatus`).

## Remaining for deploy (design §8 items 4–5 — NOT done)
- Stage static data to **`/data/pnw/location/`** (`chargers/ev_dc_fast.geojson` from `other/places/ev-stations/`,
  `rest_areas/*.json` from `other/places/`) — persistent `/data`, not the wiped overlay.
- Drop `police_proxy.json` (if police polling is wanted).
- On-device deploy per **CLAUDE.md "On-device PUSH / DEPLOY"** (rebuild `params_pyx.so` for the 2 new params;
  `touch /tmp/booted` before restart; remove `finalized` + Pause Updates). Verify lower-left overlay + no crash.

## Open verification (design §9)
OpenWeb Ninja proxy `magvar`?; mapd path populates at highway speed on WA/OR tiles?; proxy free-tier rate
limit vs ~1/min; real POLICE lateral precision vs OSM carriageway separation; lower-left region free (it is).
