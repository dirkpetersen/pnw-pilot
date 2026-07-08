# DMROAD2PNW — road-gated 3-way driver-monitoring timeout selector

**STATUS:** built on branch **`dmroad2pnw`** (off `4devpnw`), NOT yet deployed. Adds a user-facing
3-way selector that makes the relaxed DM timeouts **conditional** instead of always-on. The GLARE
Layer-C knobs are **untouched** — this changes only the pose/phone *timeout magnitudes*.

Supersedes the "the relaxation is UNGATED — no toggle exists" statement in `DM-CURRENT.md` §"UNGATED".

## What it does — the `DmMode` param (INT, default `0` = Off)

| Mode | UI label | Pose timeout | Phone timeout | Where |
|---|---|---|---|---|
| `0` | **Off** (default) | `_DISTRACTED_TIME` = **11 s** | 11 s | everywhere (stock openpilot strict) |
| `1` | **Highway** | **900 s** (15 min) | **1800 s** (30 min) | **freeway OR divided-2-lane only**; stock-strict elsewhere |
| `2` | **Relaxed** | **10800 s** (3 h) | **3600 s** (1 h) | everywhere (the prior ungated behavior) |

> ⚠️ **Behavior change on deploy:** today the device runs the equivalent of **Relaxed** unconditionally.
> After this deploys, `DmMode` defaults to **Off (stock strict)** until the driver picks Highway or
> Relaxed in Settings. This is intentional per the spec ("disabled by default").

## Road detection (Highway mode only)

"Qualifying road" = **`RoadContext == "freeway"`** OR **(`oneWay` AND `lanes >= 2`)** — the latter
catches divided multi-lane carriageways OSM doesn't tag as motorway (divided highways are modeled as
two one-way ways, so `oneWay && lanes>=2` ≈ "2+ lanes in one direction, separated").

- Source: the pfeiferj mapd binary, bridged to mem params by `system/mapd/mapd_configd.py`:
  `RoadContext` (already bridged), plus **new** `MapOneWay` (`"1"`/`"0"`) and `MapLanes` (int-as-string).
- **90-second hold:** on `RoadContext == "unknown"` / no map (tunnel, unmapped stretch, GPS dropout)
  the last *definite* verdict is held for 90 s, then falls **strict**. A definite `"city"` drops to
  strict immediately; a definite `"freeway"`/divided goes relaxed immediately.
- `MapLanes` coverage in OSM is patchy on rural roads — where absent, the `oneWay && lanes>=2` leg is
  inert and the mode relies on `RoadContext == "freeway"`. **Verify on-road** that `MapOneWay`/`MapLanes`
  are actually populated by the deployed binary (they were unverified when this was built — the parked
  device had no matched way, so `WayRef`/`MapLanes` were empty).

## Implementation (all additive)

- **`common/params_keys.h`** — `DmMode` (PERSISTENT INT "0"), `MapOneWay` / `MapLanes` (PERSISTENT
  STRING). New keys ⇒ **`params_pyx.so` rebuild required**, and the `"0"` default seeds only at manager
  startup (read `cat /data/params/d/DmMode` post-restart to confirm).
- **`system/mapd/mapd_configd.py`** — publishes `MapOneWay`/`MapLanes` alongside the existing
  `RoadContext` bridge.
- **`selfdrive/monitoring/helpers.py`** — `DriverMonitoring`:
  - `_apply_dm_timeouts()` derives `_pose_step`/`_phone_step` + the four pre/prompt thresholds from the
    effective timeout (replaces the static `__init__` computation). Only the **ACTIVE dual-counter**
    path consumes these ⇒ passive wheel-touch and all glare knobs are unaffected.
  - `_refresh_dm_mode()` re-reads `DmMode` + road class **~1 Hz** (throttled; cheap tmpfs reads) with
    the 90 s hold, and recomputes only when the effective regime changes. Called from `run_step()`
    before the counters decay. **No restart needed** to change modes — picked up within ~1 s.
  - Counters are normalized 0..1 fractions that persist across a regime change, so a downshift
    (freeway→city, or Relaxed→Off) changes the *rate/thresholds* only — worst case is an immediate
    orange **prompt**, never a jump straight to terminal lockout. Snap-back to 1.0 after 2 s attentive
    still applies.
- **`selfdrive/ui/layouts/settings/toggles.py`** — 3-way `multiple_button_item` "Driver Monitoring"
  (Off/Highway/Relaxed), inserted after the Always-On DM toggle; `_set_dm_mode` writes `DmMode`.
  Not longitudinal-gated (unlike CES) — always available.

## Safety notes

- **Direction is a safety *improvement*:** Off/Highway are *stricter* than today's always-relaxed
  behavior. Pure Python in the monitoring layer — **touches no panda safety**.
- Glare Layer-C (POSESTD 0.45, face-loss grace, HI_STD, passive recovery) is unchanged in every mode.
- The 900/1800 highway numbers are the only new magic values; the stock (11/8/6) and relaxed
  (10800/3600, 60/30/120/60) numbers reuse the existing `DRIVER_MONITOR_SETTINGS` constants.

## Verify

1. `cat /data/params/d/DmMode` → `0` after a post-deploy manager restart.
2. Offline import test in the venv (params + helpers load) before restart.
3. Settings → the "Driver Monitoring" 3-way appears right under Always-On DM, tappable, persists.
4. **On-road (Highway mode):** confirm `MapOneWay`/`MapLanes`/`RoadContext` populate on the freeway,
   and that DM stays relaxed on the interstate but tightens on surface streets (with the 90 s hold
   over coverage gaps). Re-pull `ces_events` / mem params to confirm.

## See also
- `DM-CURRENT.md` — as-deployed DM values (update its "UNGATED" section once this ships).
- `GLARE.md` — the untouched glare Layer-C knobs.
- `LOCATION2PNW.md` — the `RoadContext` freeway-gate this reuses.
