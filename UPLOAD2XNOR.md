# UPLOAD2XNOR — upload ONLY on home-WiFi geolocation, immediately, HD included

**Branch:** `upload2xnor` (off `integration2xnor`) · **Status:** implemented, **Gemini-reviewed & fixed**,
validated; **NOT deployed yet** — deploy when the device is next parked at home (one clean deploy +
verify). Deploy = restart the uploader process (no reboot, no build).

## Requirement (user)

> Uploading must **START as soon as** the device is on **home WiFi (home geo-location)**, and must
> **NEVER run** anywhere else — not cellular, not the comma hotspot, not away-from-home WiFi. This is
> the *only* criterion; onroad/offroad must not matter.

## Why this is safe by construction

You can only be associated to **home WiFi while parked at home** — drive away and you drop to cellular
(metered → skipped) or the hotspot (`networkType` reports `cell`, never `wifi`). So "on real WiFi at
the home geofence" is an exact proxy for "parked at home, safe to upload everything," and ignition /
onroad state is irrelevant.

## The change (`system/loggerd/uploader.py`)

1. **HOME-WIFI-GEOLOCATION GATE** — uploading (both passes) runs only when
   `on real WiFi AND near_home()`, reusing the existing network2xnor home feature:
   - `TetheringHomeLocation` (auto-learned home GPS, `params_keys.h`)
   - `near_home()` — 250 m geofence in `system/networkd/geo_gate.py`
   - GPS read from `LastGPSPosition` (mapd's in-memory store, locationd's persistent one).
   Fails **OPEN** only when home is unlearned or GPS is missing (being on home WiFi already implies
   home), so it never fails to upload *at* home; only a confident "far from a KNOWN home" suppresses
   it. Polls every **15 s** when gated out, so it starts promptly on arrival. `FORCEWIFI` bypasses.
2. **Interleaved PASS 1 + PASS 2** — each loop uploads one small file (qlog/qcamera) AND one large HD
   file (rlog/fcamera/ecamera), so HD never starves while a parked-but-ignition-on car keeps
   producing pass-1 work. (HD backend speed is ~0.5 MB/s → ~150 s per file; that's the real throughput
   limit, not this gate.)

Gate behavior (unit-tested): home-WiFi at home → upload; home-WiFi 5 km away → no; cellular at home →
no; other WiFi away from home → no; home-WiFi no-GPS → upload (fail-open); FORCEWIFI → upload.

## Crash-safety (Gemini-reviewed)

The uploader is a manager-supervised process — an unhandled exception crash-loops it and stops ALL
uploads. Gemini review (gemini-2.5-pro) flagged two issues; both fixed and re-confirmed safe:

- **`clear_locks()`**: now early-returns `if not os.path.isdir(root)` (was an unguarded
  `os.listdir` → `FileNotFoundError` if `realdata` is missing on a fresh/late-mounted FS).
- **Geofence call**: wrapped in `try/except` that **fails open** (`at_home = True`) on any exception,
  so a corrupted/out-of-range GPS value (which could make `haversine_m`'s `sqrt` raise `ValueError`)
  can never crash the loop. `_read_home`/`_read_gps` already catch all parse errors internally.

Gemini final verdict: *"Both issues fully resolved; the code is highly robust, crash-safe, and ready
for deployment."* No regressions.

## Known minor / deferred (Gemini suggestions, not blocking)

- Param reads (`IsOffroad`, home, GPS) happen each loop iteration. Cheap, and the loop blocks ~150 s
  per HD file anyway, so negligible — left minimal on purpose (less code = less crash surface). Could
  cache home coords + read `IsOffroad` only when idle later.
- Hardening `geo_gate.haversine_m` with `sqrt(max(0.0, a))` would protect *all* callers
  (incl. `network_arbiterd`); deferred to avoid touching shared network2xnor code (the try/except
  wrapper already protects this uploader).

## Deploy (when parked at home)

Pure Python, no build. md5-guarded copy of `uploader.py` + clear pyc + restart the uploader process
(`pkill -f system.loggerd.uploader`; manager respawns). Verify once: on home WiFi it uploads (pass 1 +
HD); confirm it does NOT upload when away. Rollback = restore `/data/dirk/org/upload2xnor/...`.

## Files
- `system/loggerd/uploader.py` — gate + interleave + crash-safety (only functional change)
- depends on `system/networkd/geo_gate.py` (`near_home`) + `TetheringHomeLocation` (unchanged)
