# UPSTREAM2PNW — commaai/master picks for a new dev branch

**STATUS:** survey only — nothing picked yet. Branch **`upstream2pnw`** (off `4devpnw`) is created to
receive the picks. Audited 2026-07-08.

## Baseline

- Our `4devpnw` lineage diverged from commaai at merge-base **`a68ea44a`** (2026-03-14).
- Surveyed: **406 commits** `a68ea44a..commaai/master` (tip `05cf8023a9`, 2026-07-07).
- Branch relationship (verified): `pnw-pilot:master` **is an ancestor of** `commaai/master` — the pnw
  and xnor masters track commaai but sit slightly behind the commaai tip. This survey targets the
  freshest point (`commaai/master`).

## Ground rules (from the request)

1. **No panda / opendbc changes** — the 22 upstream commits that bump the `panda`/`opendbc_repo`
   submodules are excluded wholesale, as is anything that only makes sense with a newer safety/CAN layer.
2. **No conflicts with existing PNW work** — every candidate's file list was diffed against the files
   `4devpnw` has modified since the merge-base. Files we own: `selfdrive/monitoring/helpers.py`,
   `system/loggerd/uploader.py`, `selfdrive/ui/layouts/settings/toggles.py`,
   `selfdrive/controls/lib/longitudinal_planner.py`, `common/params_keys.h`, `system/networkd/*`,
   `system/mapd/*`, `system/location_services/*`, `cereal/custom.capnp`, `common/api.py`,
   plus lighter touches in `selfdrived.py`, `modeld.py`, `realtime.py`, `ui_state.py`.
3. **The 2026-06-20/21 restructure is a hard boundary** — commaai moved the whole tree into a nested
   `openpilot/` dir (`5edc0bd89d`, `20e0f21b58`, `37eda06c95`, `160942dfde`…). Commits **after** that
   don't cherry-pick onto our layout without path rewriting; all Tier-1/2 picks below predate it
   (the two post-restructure candidates are marked).

## Tier 1 — clean picks (no overlap with our files; expected to apply near-clean)

Ordered by suggested pick order (dependencies/logical grouping).

| # | SHA | Date | What | Why for PNW | Files | Risk |
|---|-----|------|------|-------------|-------|------|
| 1 | `7fae59167e` | 03-20 | paramsd/torqued: use correct livePose timestamp | steering-torque learning uses right pose → better lateral | locationd/{paramsd,torqued}.py | low |
| 2 | `094591793a` | 06-03 | livePose: no timestamp from uninitialized Kalman | pose validity correctness | locationd/locationd.py | low |
| 3 | `900a896c63` | 06-04 | lagd: higher min speed | steering-lag estimate quality (lateral feel) | locationd/lagd.py | low |
| 4 | `5a7b710d90` | 06-11 | lagd: invalidate cached estimates on estimator update | stale-lag bug fix | lagd.py + **log.capnp (+1 field, additive)** | low; full rebuild |
| 5 | `38ffb324f8` | 05-11 | radard: filter lead prob | smoother lead tracking → steadier CES slow-lead trigger + Lightning Tier-2 radar | controls/radard.py | low |
| 6 | `f364110a36` | 05-01 | "Safer get accel" (reapply) | cruise-accel edge-case safety | controls/lib/drive_helpers.py | low |
| 7 | `8a80bd70e7` | 06-16 | cruise faults should not disable silently | driver-visible fault instead of silent disengage | selfdrived/state.py | low |
| 8 | `98e5f547ea` | 06-10 | athenad: outbound RPC stuck behind low-prio queue timeout | we see athenad reconnect churn in swaglog | athena/athenad.py | low |
| 9 | `ca04b70d0a` | 04-21 | **camerad: driver camera BPS magic** | **GLARE Layer A** — the driver-cam ISP fix `GLARE.md` wants ported; attacks washed-out DM images at the source | camerad/cameras/{spectra,cdm,bps_blobs} | med: C++, needs on-device scons + a real drive to confirm DM image quality; files verified present in our tree |
| 10 | `fd37cd1d03` | 05-08 | ui: prevent raylib sleep drifting from vblank | smoother raylib UI frame pacing | system/ui/lib/application.py | low |
| 11 | `ef94b134c3` | 05-14 | common: avoid shell in sudo_read | small hardening | common/utils.py | low |
| 12 | `0f17a98793` | 06-01 | modeld: fix capnp memory leak | long-drive stability | modeld/modeld.py (we touched this file — see Tier 2 note) | low-med |

## Tier 2 — we touched the same file (expect a trivial context merge, not a real conflict)

| SHA | Date | What | Our overlap | Verdict |
|-----|------|------|-------------|---------|
| `433c52f623` | 06-09 | radar errors: only gate enable if openpilot-long | `selfdrived.py` has 3 pnw commits (mapd non-essential, CES renames, location2pnw) — different regions | pick, resolve by hand if needed. Nice for the Lightning (radar quirks shouldn't block lateral-only) |
| `69e2c321e4` | 06-01 | frame drop: only allow 1% | same `selfdrived.py` | pick with above |
| `aa26dded8b` | 05-12 | ui: lower priority background threads | `realtime.py` (dmon2pnw) + `ui_state.py` (mapd2pnw) — additive lines | pick, verify our lines survive |
| `0f17a98793` | 06-01 | modeld capnp leak (from Tier 1) | `modeld.py` has auto2pnw nudgeless change — different function | pick |

## Tier 3 — high value but HAND-PORT only (touches `helpers.py` we own, or swaps the DM model)

Not cherry-pickable per rule 2 — port the *idea* manually, exactly like the GLARE work was done.

| SHA | Date | What | Why / caveat |
|-----|------|------|--------------|
| `5dcaf3bef8` | 04-01 | DM: fewer alerts during maneuvers | suppresses distraction alerts during active steering/maneuvers — pairs perfectly with our `DmMode`; ~25-line hand-port into our `helpers.py` |
| `19d56f685b` | 04-08 | DM: auto-reset audible alert coming to a stop | QoL: terminal alert clears at a stop; small hand-port |
| `2ed88a1dff` | 05-17 | **DM: sleep-prob logging + updated `dmonitoring_model.onnx`** | **GLARE Layer B** — the sleep-prob DM model. Swapping the model needs the runtime input/output layout to match our xnor-era `dmonitoringmodeld` (upstream had companion runtime changes, e.g. `c3b0f0d11a` frame-size-from-vipc). Treat as its own mini-effort; +1 additive `log.capnp` field |

## Explicitly EXCLUDED (and why)

- **All 22 panda/opendbc submodule bumps** — rule 1.
- **`3a764c0ae3` "Params: rm nonblocking funcs"** — we *depend* on `put_nonblocking` throughout
  (mapd bridge, CES, location services). Adopting it would touch dozens of our files for zero benefit.
- **The 2026-06-20/21 restructure chain** (nested `openpilot/`, HAL move, `car.capnp` unification,
  `IsOffroad` param rename) — wholesale tree reshaping; would conflict with everything we carry.
- **model16 / usbgpu / tinygrad-bump chain** (`f02d134f40`, `52e182611d`, `cd2c590d50`, `a40fa3a0b0`…) —
  new driving-model stack; deeply entangled, reverted-and-reapplied upstream, needs the new cereal
  (`drivingModelData` compression `bdac9efa1e`). A separate future effort at best.
- **webrtc/teleop chain** (`45d8bcd7f3`, `9844075bb2`, `aff9f9ffae`, +8 more) — depends on the newer
  athena/cereal stack; we don't use comma-connect teleop.
- **DM refactor chain** (`389b639ef2` DriverMonitoringState v2, `8b53f9158d` alert renames,
  `6996e87f8d` helpers→policy rename, `b29d0a17af` readability) — schema + rename churn that would
  collide head-on with our modified `helpers.py`/DmMode.
- **`4ecbdb0d7a` DCAM reset 2 s** — already ported (it's in our GLARE Layer C).
- **`4585e93066` dmonitoringmodeld YUV padding** — touches `compile_dm_warp.py`, which doesn't exist
  in our tree (newer DM pipeline); N/A.
- **`d79267fa22` modem DNS fallback** — file layout differs (upstream `hardware/tici/modem.py` split
  doesn't exist in our tree). LTE-relevant, so worth a **hand-port into our
  `system/hardware/tici/hardware.py`** later, but not a pick.
- **AGNOS bumps** (`8c533b14c0` 18.1, `c87f613659` 18.4…) — OS updates, separate risk domain.

## Suggested workflow on `upstream2pnw`

1. Cherry-pick Tier 1 in table order (each is independent; `5a7b710d90` + anything touching
   `log.capnp` ⇒ full on-device rebuild, not just a restart).
2. Then Tier 2, resolving trivial context conflicts by hand; re-run our offline import self-test.
3. `pytest selfdrive/locationd selfdrive/controls selfdrive/monitoring -m 'not slow'` on the host,
   plus `scons -u` build; then Gemini review of the accumulated diff (gemini-pro-latest).
4. Merge to `4devpnw` only after a parked deploy + drive verification; camerad BPS (`ca04b70d0a`)
   deserves its own deploy so DM-image changes are attributable.
5. Tier 3 items are separate hand-port mini-efforts (do `5dcaf3bef8` first — biggest DM QoL win).

## See also
- `GLARE.md` — Layers A (`ca04b70d0a`) and B (`2ed88a1dff`) are the two unported fixes it names.
- `DEVICE-STATE.md` — deploy checklist; `DMROAD2PNW.md` — the DmMode work Tier-3 ports must respect.
