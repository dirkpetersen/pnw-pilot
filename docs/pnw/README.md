---
updated: 2026-09-19          # git-derived; bump when you edit this file
status: current        # current | drifted | superseded | unreviewed
---

# PNW design docs

PNW-specific feature/effort design docs (moved here from the repo root 2026-07-13 to declutter).
The user-facing feature list is [`../../PNW-PILOT-FEATURES.md`](../../PNW-PILOT-FEATURES.md); these are
the deeper per-feature design/rationale docs.

| Doc | Covers |
|-----|--------|
| [CES_I90.md](CES_I90.md) | I-90/Snoqualmie curve-braking learnings; CES→Experimental over-brake fix; map-curve braking on by default |
| [CURVESLOW2PNW.md](CURVESLOW2PNW.md) | Low-speed / city curve slowdown work |
| [SHARPCURVE2PNW.md](SHARPCURVE2PNW.md) | Earlier/smoother blind-sharp-curve VTSC (full map lookahead, regen-coast, apex timing) |
| [LOCATION2PNW.md](LOCATION2PNW.md) | "Happening Ahead" display overlay — police / rest areas / EV chargers |
| [LEBOWSKI2PNW.md](LEBOWSKI2PNW.md) | Driving-model snapshot-port (lebowski) method + deploy |
| [DMROAD2PNW.md](DMROAD2PNW.md) | Road-gated driver-monitoring timeout selector (`DmMode`) |
| [DM-VARIABLE.md](DM-VARIABLE.md) | Variable driver-monitoring design |
| [FORDLONG2PNW.md](FORDLONG2PNW.md) | Ford Lightning longitudinal (op-long / ICBM) work |
| [FORDSAFETY2PNW.md](FORDSAFETY2PNW.md) | Ford panda safety-C changes (4-signal lateral, reset-latch hardening) |
| [UPSTREAM2PNW.md](UPSTREAM2PNW.md) | Upstream (commaai) sync / rebase notes |
| [CES2PNW.md](CES2PNW.md) | Initial CES+VTSC+mapd port onto the pnw line (+ its 5 Gemini safety fixes) — port history |
| [ICBM2PNW.md](ICBM2PNW.md) | Lightning stock-ACC curve slow-downs via SET−/SET+ taps (map-first, set-tracking, guarded restore) |
| [VISIONBACK2PNW.md](VISIONBACK2PNW.md) | ⛔ MEASUREMENT ONLY, verdict **dead** — can vision hand back a bounded N mph of a map slowdown between 50–150 m? N=10 reaches **5.80 m/s²**, N=5 reaches 4.70, only N=2 is safe and returns 2 mph. Not an accuracy problem: the trust gate is a pass-through (vision says "no real curve" on 87 % of ticks). Nine reproducible analysis scripts in `visionback-evidence/` |
| [ICBMSLOW2PNW.md](ICBMSLOW2PNW.md) | ✅ **SHIPPED** `a9329c6d75` (2026-09-18 19:27; the row used to read "NOT DEPLOYED") — ICBM over-slows for curves: the map-rating floor (the 2026-08-11 penalty/scale double count), + the widened n=3,087-tick evidence |
| [LATACCEL2PNW.md](LATACCEL2PNW.md) | Speed-scheduled, JSON-hot-reloadable max-lateral-accel cap for `clip_curvature()` (low-speed authority up, highway stays ISO 3.0) |
| [FORDREGEN2PNW.md](FORDREGEN2PNW.md) | EV regen over-decel design (Ford long PID damping Fix A + regen-bite Fix B) |
| [MAPD-SYSTEM.md](MAPD-SYSTEM.md) | **As-deployed mapd** — pfeiferj binary (stock pin v2.3.1; device runs override build `77bad867`), `mapd_configd`, `MapdOut`, full param table |
| [GPSSEL2PNW.md](GPSSEL2PNW.md) | Which GPS fix reaches `LastGPSPosition`: the device `hasFix` check + write-on-arrival `ts` (gpsfix2pnw), then the Lightning CAN GPS selection by capability (gpssel2pnw) |
| [MAPD2PNW.md](MAPD2PNW.md) | ⛔ Historical: the original pnw mapd foundation (fixed default state list, priority-WiFi gate); superseded by MAPD-SYSTEM's GPS-driven on-demand whole-state download |
| [DM-CURRENT.md](DM-CURRENT.md) | Source of truth for the as-deployed DM config (tiers + GLARE knobs) |
| [GLARE.md](GLARE.md) | Layer-C DM glare band-aid (deployed 2026-07-06) |
| [ONROAD-CHARGING.md](ONROAD-CHARGING.md) | EV parked-while-charging reads as onroad — `gearShifter=park` is the real parked signal |
| [PARKNOREC2PNW.md](PARKNOREC2PNW.md) | No route segments while the shifter is in Park: manager stops loggerd after 30 s of `GearPark`; GearPark writer hardened; kill switch `RecordWhileParked` |
| [PARKGATE2PNW.md](PARKGATE2PNW.md) | ✅ **SHIPPED** `1dd8a1a313` (2026-09-19) — the `ces_events` half of the same problem: the ~1 Hz breadcrumb stops while the shifter is in Park (93.7 % of the archived corpus was a parked truck). Gear-gated, fails open, 60 s marked heartbeat so a hold is never a silent gap; adds `gear`/`park` to every record. Corrects the 2 GB archive's retention from ~12.9 driving hours to ~140–150 |
| [REST_AREAS.md](REST_AREAS.md) | Feasibility: mapd can't surface rest-area POIs → option (b) built |
| [REST_AREA_DATA.md](REST_AREA_DATA.md) | The corridor rest-area JSON dataset + schema + generators |
| [DEFER_HD_UPLOAD.md](DEFER_HD_UPLOAD.md) | "Defer HD Video Upload" toggle |
| [UI-CPU-TRIM.md](UI-CPU-TRIM.md) | UI CPU investigation → uicpu2pnw (param reads 60→2 Hz) |
| [CHANGELOG-2026-07-01.md](CHANGELOG-2026-07-01.md) | PNW changelog 06-29→07-06 |
| [CHANGELOG-2026-07-12.md](CHANGELOG-2026-07-12.md) | PNW changelog 07-11→07-12 (the "Ford weekend") |
| [CHANGELOG-2026-07-18.md](CHANGELOG-2026-07-18.md) | PNW changelog 07-12(eve)→07-18 (red-light-lurch arc, commIssue cascade, speedadjust, tightfollow arc + revert) |
| [uploadtest-evidence/](uploadtest-evidence/README.md) | 🆕 Mutation proof for the restored `test_uploader.py` (`uploadtest2pnw` `1f9a352538`) — 4 of 39 tests had been RED on the channel for two weeks (bisected to `6ad65ca264`); 8 mutants / 8 killed, incl. one that reintroduces the silent-data-loss behaviour and two that ONLY the restored exact-sequence assertion catches |
| [CHANGELOG-2026-09-20.md](CHANGELOG-2026-09-20.md) | PNW changelog 09-20 — **Rule 9's first run on someone else's work.** `everdrive2pnw` (7 commits, `params_keys.h` + `opendbc_repo` pin + UI) came back GREEN, and both mechanisms proved out: opendbc auto-moved to the bumped pin, and `params_pyx.so` validated 219 keys (was 217) because it is BUILT rather than borrowed. ⚠️ **But it reported green without importing the feature's own tests** — `test_everdrive_status_pnw.py` landed in a directory `TEST_PATHS` did not cover. A scope that never GROWS is as blind as one that shrinks; +3 dirs, 3,320 → 3,466 tests |
| [CHANGELOG-2026-09-19.md](CHANGELOG-2026-09-19.md) | PNW changelog 09-19 — five ships: **`parkgate2pnw`** (`ces_events` was 93.7 % a parked truck; the 2 GB archive held 12.9 driving hours, now ~140–150), **`curvedbshadow2pnw`** (Phase 2's on-car half, SHADOW ONLY, read boundary enforced by a test), **`viskvis2pnw`** (the model's curvature on every tick, not only the ones ICBM acted on), and **`VISIONBACK2PNW`** — the bounded vision give-back MEASURED AND DEAD (N=10 → 5.80 m/s²; the trust gate is a pass-through, vision says "no curve" on 87 % of ticks). Plus: `icbmslow2pnw` actually shipped 09-18, §6.2's DOWN rule was structurally unreachable (0 → 127 observations), and the five checks this week that could not fail. Plus **`uploadtest2pnw`**: the uploader's test file had been RED for two weeks (the component whose API_HOST fallback once caused silent data loss), and the Fable review caught that my first fix made them pass by WEAKENING them. And the map-rating floor **verified LIVE on the car** via `initData.gitCommit` in a qlog from S3 — an install check that works through CGNAT |
| [CHANGELOG-2026-09-18.md](CHANGELOG-2026-09-18.md) | PNW changelog 09-18 — **curvedb Phase 2's §7 replay was built and RUN, and returned NO RESULT**: 170 ICBM episodes, acted on zero under leave-one-out; the blocker is SITE RECURRENCE (185 of 193 sites driven once) — but `drives/` is analysis windows, median 62 min, so that is a biased lower bound and the question has never really been asked. Also: the ICBM over-slow is a dated REGRESSION (began 2026-08-11 when `icbmcurve2pnw` changed the tier scale and nobody re-calibrated the penalty hump), a retraction of the `map_scale: 1.0` advice (it deletes 18 of 88 slowdowns), and the map-rating floor built to fix it |
| [CHANGELOG-2026-09-17.md](CHANGELOG-2026-09-17.md) | PNW changelog 09-17 — **curvedb Phase 1 shipped** (`4b901737b2`, 9 commits): curvature telemetry + `/data/pnw/ces_archive` retention + S3 upload of the corpus. Four Fable rounds, three of which caught a check that passed for the wrong reason. Could NOT be verified on the car (Starlink CGNAT — no inbound SSH). Also: the abort rule is preempted by `behindrun2pnw`, and the stale-branch audit (`lcroc2pnw` was already shipped) |
| [CHANGELOG-2026-09-16.md](CHANGELOG-2026-09-16.md) | PNW changelog 09-16 — mapd car-GPS relay shipped; the stale-branch survey ("nothing was ready", three would have reverted shipped work); corrections to the 0.11.2 claim, the "31 satellites" sentinel and the swaglog-grep trap |
| [CHANGELOG-2026-09-15.md](CHANGELOG-2026-09-15.md) | PNW changelog 09-15 — the nine-install night (incl. `behindrun2pnw`, which is what preempts the 09-16 abort rule) |
| [CHANGELOG-2026-09-14.md](CHANGELOG-2026-09-14.md) | PNW changelog 09-14 overnight (arbiter active-change logging, accdrop STALE_ROUTE_S; in progress: unreadable-read hold, behind-curve gate, police misses, Ford gear unknown, lead-loss review) — updated as items ship |
| [CHANGELOG-2026-09-13.md](CHANGELOG-2026-09-13.md) | PNW changelog 09-13 (ICBM restore cap, speed-limit zone no-restore, curve-with-lead, MADS quiet chimes, WiFi cost ladder + manual pick, metered updates + backoff + keep-staged, no recording in Park, mapd pin 2.3.1 + logging, ACC dropout diagnostics, lat-accel cap 5.0<50mph) |
| [op-long-features.md](op-long-features.md) | **Capability matrix**: every feature x Tesla / Lightning+AlphaLong-ON / Lightning+AlphaLong-OFF, per-car pros/cons, which-mode-to-drive guidance (Fable-written 2026-07-18) |
| [PNWTEST3-OVERNIGHT-2026-06-22.md](PNWTEST3-OVERNIGHT-2026-06-22.md) | Historical: network2xnor rebuilt as 4 increments on pnwtest3 |

(The 16 docs below the first group were consolidated here from `~/gh/comma/docs/` on 2026-07-18 —
that folder now keeps only workbench-wide and other-fork docs.)

See also the workbench-wide catalog at `~/gh/comma/docs/INDEX.md`.
