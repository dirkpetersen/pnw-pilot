# CES2PNW — Conditional Experimental Switching + VTSC + mapd ported to pnwtest3

**Branch:** `ces2pnw` (off `pnwtest3`, openpilot v0.11.2) · pushed to `origin/dirkpetersen/pnw-pilot`.
**Status: IMPLEMENTED, Gemini-reviewed, pre-flight-verified on the device env — NOT DEPLOYED.**
Device stays on stable `pnwtest3 @ 290543e78`, frozen (`DisableUpdates=1`). `ces2pnw` (`efbc591b6`)
is fetched into the device's git, ready to deploy when supervised + in the car.

## What it is (the full ces2xnor feature, ported)
CES = chill by default, auto-flip to Experimental only for curves / low-speed-city / stop lights /
slow-lead, then back. VTSC caps cruise speed smoothly through curves. mapd (pfeiferj binary) feeds the
map-curve half + OSM speed limits. See `CES.md` (design) and `VTSC.md`.

### Decisions baked in (per Dirk, this session)
- **Replaced** the Experimental Mode settings toggle with the **CES master toggle**. Full Experimental
  is reachable via the **top-right 3-state button** (white-exp = CES auto / wheel = forced Chill /
  orange-exp = forced Experimental).
- **mapd ported** in full (it's the existing pfeiferj binary + `MapTargetVelocities` param, not new
  geometry code).
- **All cars** with `openpilotLongitudinalControl` (not brand-gated); CES greys out + forces off when
  op-long is unavailable.
- Default **OFF** (`ConditionalExperimentalSwitching=0`) → behavior-neutral / byte-identical baseline.

## Commit chain (ces2pnw)
1. `b7b0bb43a` Phase 1 — CES + VTSC self-contained cores (`ces_xnor/`, `vtsc_xnor/`); 26/28 unit tests
   pass on dev host (2 + the ces test module need capnp/full env → run on-device post-install).
2. `ef0b6802c` Phase 2 — mapd subsystem: `sunnypilot/mapd/` (6 py) + `third_party/mapd_pfeiferj/mapd`
   (9.37 MB binary) + 15 OSM/mapd params (incl. `MapTargetVelocities`, `OsmStateName="WA,OR,ID"`) +
   `CustomReserved8→LiveMapDataSP` + `liveMapDataSP` service + `Paths.mapd_root()` + 2 managed procs.
3. `1f298e139` Phase 3a — integration: selfdrived publishes effective `experimentalMode`; planner
   `v_cruise = vtsc.cap(...)`; 3-state `exp_button.py`; `ces_status.py` overlay; `CustomReserved9→
   VtscState` + `vtscState` service (+ plannerd PubMaster); CES params.
4. `efbc591b6` Phase 3b — **replace Exp Mode toggle with CES** + **all 5 Gemini safety fixes**.

## Gemini review (integration) — all VERIFIED real + FIXED
- VTSC can-only-reduce enforced in the planner WIRING: `v_cruise = max(0, min(v_cruise, cap_v))` +
  `isnan` guard (can't accelerate / NaN-crash the MPC).
- `experimental_request` wrapped in try/except in the selfdrived hot path → chill on any error.
- Effective `experimentalMode` gated on `CP.openpilotLongitudinalControl` → byte-identical when OFF.
- `vtscState` publish uses defensive `m.get(...)` → can't KeyError-crash plannerd.

## Pre-flight verified on the device env (non-destructive)
- `cereal/custom.capnp` (with `LiveMapDataSP`+`VtscState`) **compiles** via pycapnp on-device.
- mapd binary present in git (9371832 bytes). CES/mapd params all registered. No v0.11.2 params dropped.
- No leftover `CustomReserved8/9` struct defs; each wire ID used once.

## To deploy later (when supervised + in the car) — RECOMMENDED PATH
Use the **installer/updater** (clean full build), NOT a file-overlay SSH patch (the overlay/stale-pyc/
nested-`openpilot/openpilot/` traps caused repeated UI crashes this session — see the openpilot skill
`pitfalls.md`). capnp + `params_keys.h` changed → a rebuild is required regardless, which the updater
does cleanly.
1. Set `UpdaterTargetBranch=ces2pnw` (or install `dirkpetersen/ces2pnw`), `DisableUpdates=0`.
2. Let it fetch + `scons` build + finalize → reboot onto the clean build.
3. Post-deploy verify (the part deferred): UI healthy + stable (no crash-loop); `pgrep mapd
   mapd_manager`; run the full ces_xnor/vtsc_xnor test suite on-device; CES default OFF →
   behavior-neutral; toggle CES on (Settings) → 3-state button cycles; **drive attentively** —
   Terwilliger-class curve → VTSC eases speed / CES flips Experimental, returns after.
4. Persistence guards if file-overlaying instead (not recommended): see `CES.md` §F.

## Rollback
Device is on `290543e78` now; if a deploy misbehaves, set `UpdaterTargetBranch=pnwtest3` (or reinstall)
and reboot. `ces2pnw` never overwrote the running system this session.
