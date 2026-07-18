# REST_AREA_DATA — the location-services rest-area corridor data

**Status:** BUILT + DEPLOYED on `4devpnw` (`pnw/pnw-pilot`), live on the 3X. This is the shipped
implementation of **`REST_AREAS.md`**'s recommended *option (b)* — a standalone OSM lookup — since
the pfeiferj mapd binary strips POI nodes at tile-build time and cannot surface rest areas itself
(`MapdOut` has no rest-area field). See `LOCATION2PNW.md` for the consuming "Happening Ahead" overlay
and `MAPD-SYSTEM.md` for why mapd can't do this.

---

## Where the data lives

Static, version-controlled JSON, one file per corridor, under:

```
system/location_services/data/rest_areas/
  i5_rest_areas.json         # I-5 (WA/OR)
  i90_rest_areas.json        # I-90 (WA)
  i82_rest_areas.json        # I-82 (WA)
  us12_us95_rest_areas.json  # US-12 + US-95 (WA/ID)
```

The daemon merges **every `*.json` in this directory** (`REST_DIR`,
`location_servicesd.py:45`), so adding a new corridor = drop in another file; no code change.

## Schema (one JSON list per file, each entry a rest area)

```json
{
  "name":    "Custer Southbound Rest Area",   // full OSM name
  "display": "Custer",                          // short label shown in the overlay
  "town":    "Birch Bay",                       // nearest town (context only)
  "type":    "rest_area",                       // always "rest_area" here
  "dir":     "S",                               // travel direction served: N/S/E/W (or "" if both)
  "state":   "WA",
  "lat":     48.926833,
  "lon":     -122.645395,
  "osm":     "way/160676510"                     // OSM element id (provenance / re-query key)
}
```

The daemon is tolerant of shape: it accepts either a bare list or `{"rest_areas": [...]}`, and for
the label falls back `display → name → "Rest area"` (`location_servicesd.py:235-238`).

## How the daemon consumes it

- Loads + merges all corridor files once at startup (`location_servicesd.py:222-246`, logs
  `loaded %d rest areas`).
- Rest areas are **car-agnostic** (shown regardless of Tesla/Ford) — `location_servicesd.py:811`.
- **Perpendicular distance filter (the important gotcha):** the data is scoped per-corridor, but a
  rest area from a *crossing* corridor (e.g. an I-5 entry while driving I-90) can still be near the
  car. The daemon applies a ~1.5 mi perpendicular filter so only rest areas on the mainline you're
  actually driving surface (`location_servicesd.py:92-95`). The gatherer scopes to within ~2 km of
  the mainline, so 1.5 mi comfortably keeps the real ones and rejects the crossers.
- Nearest-ahead selection is the shared `_nearest_within(items, lat, lon, max_mi)` helper
  (`location_servicesd.py:582`).

## How the JSON is generated / maintained

The generators are **standalone dev-host scripts**, NOT part of the openpilot tree — they live in
**`~/gh/comma/other/places/`**:

```
other/places/i5_rest_areas.py
other/places/i90_rest_areas.py
other/places/i82_rest_areas.py
other/places/us12_us95_rest_areas.py
```

Each queries **OpenStreetMap via the Overpass API** for `highway=rest_area` nodes/ways within ~2 km
of the corridor's mainline (service plazas / truck stops excluded), then writes both the
`<corridor>_rest_areas.json` (deployed) and a `.csv` (review). Details:

- Overpass endpoints are tried in order with fallback mirrors
  (`overpass-api.de` → `overpass.kumi.systems` → `overpass.osm.ch`) — `i82_rest_areas.py:39-43`.
- Query shape: find the corridor ways, then `node(around.hw:2000)["highway"="rest_area"]` +
  `way(around.hw:2000)[...]` (`i82_rest_areas.py:56-57`).
- The US-12/US-95 gatherer differs slightly (US-highway ways instead of an interstate ref) —
  `us12_us95_rest_areas.py:7-13`.

**To refresh or add a corridor:** run the matching `other/places/*_rest_areas.py`, eyeball the
`.csv`, then copy the resulting `<corridor>_rest_areas.json` into
`pnw/pnw-pilot/system/location_services/data/rest_areas/` and commit on the pnw feature branch. No
device-code change is needed — the daemon auto-merges any `*.json` it finds there.

> These are small, hand-curated corridor files (I-5/I-90/I-82/US-12/US-95 = the PNW Seattle↔central western Oregon
> and eastern-WA driving envelope), not a live download — they ship in-tree and are refreshed manually
> when the route set changes.
