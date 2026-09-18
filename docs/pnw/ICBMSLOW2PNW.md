---
updated: 2026-09-17
status: unreviewed     # current | drifted | superseded | unreviewed
---

# ICBMSLOW2PNW — ICBM over-slows for curves: the map-rating floor

> **⛔ NOT DEPLOYED, NOT PUSHED, NOT REVIEWED.** Branch `icbmslow2pnw` off `origin/3devpnw`
> (`7c40d8003b`). This is control-path code that changes how the truck brakes. The owner decides.
> Nothing was written to `/data/pnw/curve.json` on the device.

## The complaint

Driver, **2026-09-17 14:25 PT**: *"button control management also took me down to 38 mph… it's going
too slow."* ICBM is the stock-ACC button-tapping curve slowdown on the Ford F-150 Lightning
(openpilot longitudinal OFF; ICBM taps SET− on `0x083`).

Measured on that event (`drives/2026-09-17/curvedb-first-capture/`): mapd rated the curve
**43.6 mph**, ICBM published **38.2 mph**.

## Root cause: two Lightning reductions that stopped being one design

A map candidate reaches the Lightning curve penalty as

```
eff = icbm_map_eff_scale(raw) * raw * map_scale          # map_scale = 0.92, Lightning-only
target = eff - curve_speed_penalty_ms(eff)               # the hump, up to 5 mph (×1.15 left, ×1.4 descent)
```

| date | `icbm_map_eff_scale` (tight/moderate) | composite with `map_scale` 0.92 | penalty on a 44 mph map curve | commanded |
|---|---|---|---|---|
| 2026-07-11 (`descentcurve2pnw`, the calibration) | **flat 1.35** | **1.242×** | ~5 mph off a 54.6 mph candidate | **49.6 mph** |
| 2026-08-11 (`icbmcurve2pnw`) → today | **1.10** below 50 mph raw | **1.012×** | ~4.9 mph off a 44.5 mph candidate | **39.6 mph** |

The 5 mph hump was field-calibrated on 2026-07-11 against a candidate that arrived **24 % inflated**
— so on tight and moderate curves the July target still landed *above* mapd's own rating.
`icbmcurve2pnw` then dropped the tight end of the scale to 1.10 for a different and entirely correct
reason (a genuine 50 mph curve inflated to 62 mph effective was being **rejected** as a candidate
against a 55 mph cruise — a non-trigger, not a late trigger). Nothing re-calibrated the hump against
that. Since 2026-08-11 the penalty has been subtracting an inflation margin that is no longer there.

**Corollary, and it matters for how the complaint reads:** commanding *below* mapd's own rating is a
behaviour that started on 2026-08-11. It was never the driver-approved July behaviour.

**A second correction, to the 2026-09-17 drive report.** That report decomposed the 14:25 event as
"44 × 0.92 = 40.5, minus a 3.8–4.4 mph penalty", i.e. `map_scale` and the hump contributing about
equally. That is wrong: it omits the 1.10 ICBM scale that `map_scale` is multiplied against. The
composite is **1.012×**, so `map_scale` contributes ~0.5 mph at that event and the hump contributes
essentially all of the rest.

## The evidence (widened from n=39 on one drive to n=3,087 ICBM ticks over 22 corpora)

Swept **every** `ces_events` corpus under `drives/` (92 files, 730,887 records, 0 unreadable,
70 unparseable lines). 373,244 Lightning records; **69,291 deduped Lightning moving ticks**;
**3,087 ticks where ICBM published a target** (1,882 map-sourced, 756 far, 192 vision, 179 restore).

Per-corpus field availability is reported in full (`_scratch/icbmslow/a1_capability.py`) precisely so
that a corpus contributing nothing is visible rather than silent:

| witness | first corpus that has it | Lightning moving ticks with it |
|---|---|---|
| `strAng` (steering angle) | 2026-07-10 | 63,743 |
| `slKActl` / `achLat` (yaw-rate curvature) | 2026-08-11 / 2026-08-12 | 44,187 / 42,077 |
| `icbmK` (polyline curvature) | 2026-09-08 | 34,433 |
| `kPeak` (curvedb) | 2026-09-17 | 6,930 |

Pre-2026-08-11 corpora (2026-07-10 … 2026-08-10, 68 episodes) have **no curvature witness usable for
a lateral-accel claim**. `strAng` → curvature through a bicycle model with a fitted understeer
gradient was measured against `slKActl` on 33,568 overlapping ticks: sign agreement 99.2 %, but
median relative error 0.15 and **p90 0.60**. That is too loose to adjudicate an individual event, so
it is used only as a coarse presence check and is labelled wherever it appears.

**251 ICBM DEC episodes** were extracted; **107** had a usable apex-curvature witness (70 excluded as
below 8 m/s, 68 as pre-witness corpora, 6 as a junction/parking turn with R < 30 m). **93** started
above 25 mph. The curvature is taken from the ticks *after* the decision, as the truck traverses the
curve — not the same tick. Three window definitions give medians 1.39 / 1.39 / 1.62 m/s², so the
result does not hang on the window.

| at what speed | lateral accel the truck met (n = 93) |
|---|---|
| **ICBM's commanded target** | p10 0.20 · **median 1.34** · p90 2.01 |
| what the truck actually drove | median 1.50 |
| mapd's own raw rating | median **1.67** · p90 2.47 |
| the driver's set (the ceiling) | median **2.78** · p90 7.59 |
| design target `A_LAT_TARGET` | 2.50 |

The ceiling row is why the slowdowns are needed at all. The first row is the complaint, and it
reproduces the 2026-09-17 single-drive finding (1.40 median) at 2.4× the sample across seven drives.

**What the driver himself drives at** (44,187 Lightning ticks with a lateral witness, ICBM silent,
in a bend i.e. |a_lat| > 0.5): median 1.01, p90 1.88 above 25 mph; p90 **1.63**, p99 2.52 above
45 mph. Only 0.73 % of all >25 mph ticks ever exceed 2.5 m/s². **2.5 is the top of this truck's
operating envelope on these roads, not a routine number** — which is the argument against "just aim
at `A_LAT_TARGET`".

## The candidates, replayed through the shipped code

The forward model was validated first: modelling `icbmT` from the logged `mapV`/`mapDist`/ref/`v_ego`
reproduces the log to a **median +0.03…+0.10 m/s** residual on every corpus from 2026-08-12 on
(p10/p90 within ±0.5 m/s). The pre-2026-08-11 corpora show −1.7…−6.0 m/s, which is exactly the
`icbmcurve2pnw` scale change and is what first exposed the root cause.

88 of the 93 episodes are replayable (the rest are far/vision-sourced with no logged map inputs).
`lost` = the curve stops binding at all, scored at the ceiling speed:

| variant | Δ target | a_lat median | a_lat p90 | > 2.5 | > 4.5 | **lost** |
|---|---|---|---|---|---|---|
| shipped | +0.00 | 1.41 | 2.06 | 4 | 0 | 0 |
| `map_scale` 0.92 → **1.00** | +2.36 | 1.78 | 2.69 | 12 | 0 | **18** |
| `penalty_max` 5 → 3 | +0.64 | 1.50 | 2.15 | 4 | 0 | 0 |
| `penalty_max` 5 → 2 | +0.96 | 1.55 | 2.21 | 5 | 0 | 0 |
| hump peak moved to 62–78 mph | +0.43 | 1.50 | 2.15 | 4 | 0 | 0 |
| **map-rating floor (this change)** | **+1.31** | **1.65** | 2.32 | 4 | 0 | **0** |
| floor + `penalty_max` 3 | +1.80 | 1.65 | 2.34 | 4 | 0 | 0 |
| the pre-2026-08-11 code (flat 1.35) | +6.51 | 2.19 | 3.07 | 31 | **1** | **32** |

Readings:

* **`map_scale` → 1.0 is not free.** It raises the *effective* speed, which is what the reduce-only
  candidacy test (`eff >= ref - ICBM_MIN_DROP_MS` → reject) is applied to, so **18 of 88 slowdowns
  disappear entirely** for a median +2.36 mph. Eleven of them are the same shape: a curve rated
  38 mph with the set at 42.
* **Reshaping the hump barely moves anything** (+0.4…+1.0 mph). ICBM's binding targets sit at a
  median 33 mph — on the hump's *ramp*, not on its 45–62 mph plateau — so `penalty_max` has little
  leverage there. And the hump is **shared with VTSC**: lowering it silently weakens the op-long
  washout margin the moment Alpha Long is switched back on.
* **Going back to the pre-2026-08-11 behaviour is not an option**: 32 of 88 slowdowns stop binding,
  which is the non-trigger bug `icbmcurve2pnw` was written to fix.
* **The floor gets the median to mapd's own number (1.65 ≈ 1.67) with no candidacy change and no
  episode lost.** It touches 85 of 88 episodes; median gain on those 1.54 mph, p90 4.36, max 4.46.
  At the 14:25 complaint it commands **43.6 mph instead of 39.4** (a_lat 1.98 vs 1.61 on the measured
  R ≈ 192 m; 1.52 vs 1.14 on the drive report's kPeak-derived R ≈ 254 m).

## What ships on this branch

**One rule, in `_icbm_step`, applied after candidacy and after the penalty:**

> The Lightning curve penalty may not push a MAP or FAR curve target below **that candidate's own raw
> mapd rating** (× `icbm_map_floor_frac`, default 1.0).

* `PnwVehicle.icbm_map_floor_ms(raw)` — new accessor; 0.0 on every non-Lightning car (and provably
  inert there anyway: with penalty 0 and `map_scale` 1.0 the target is already ≥ 1.10 × raw).
* `icbm_far_map_candidate()` now returns `(eff, dist, **raw**)` — the winning point's own rating.
  Taken where the point is selected, not re-derived by distance-matching afterwards (the
  neighbouring-node trap `map_candidate_point`'s docstring warns about) and not by inverting the
  scale (which would rot silently the next time `icbm_map_eff_scale` changes — it already has once).
  `_icbm_passed_gate` carries it through so a replaced candidate is never floored at the rating of
  the curve it replaced.
* **The descent guard and the left-curve factor still bite below the floor.** The floor bounds the
  *base* hump only; the multipliers' extra over the flat-right penalty is still subtracted. They
  model risk mapd's rating does not contain (adverse crown on a left; a descent eating the decel
  budget) and both came from the two downhill-LEFT washouts of 2026-07-11. Flooring the *multiplied*
  penalty would make `left_factor` and `descent_gain` silently do nothing on most map curves.
* **Vision candidates are never floored** — `icbm_vision_apex` is already a physics-derived safe
  speed with no map rating behind it. The pinned VTSC↔ICBM penalty parity
  (`test_parity_vtsc_and_icbm_apply_identical_penalty`) is on the vision path and is unchanged.
* **`curve_speed_penalty_ms` itself is untouched**, so VTSC / op-long is byte-identical and the
  washout regression registry is unaffected.
* Telemetry: **`icbmMapFlr`** (the floor used, m/s; 0.0 = none) and **`icbmMapFlrHit`** (it actually
  gave penalty back) in `ces_events` and on the `CESStatus` overlay feed, cleared every tick and on
  an inactive tick. Without both there is no way to tell "the floor never applied" from "it applied
  and changed nothing".

### `/data/pnw/curve.json` — new key

```json
{"lightning": {"icbm_map_floor_frac": 1.0}}
```

Clamped to `[0.0, 1.0]`: it can never floor **above** mapd's rating, and **`0.0` is the documented
off switch** that reproduces today's behaviour exactly. Read once at construction — a change needs a
reboot.

## Would this have re-created the 2026-07-11 washouts?

**No, and the reason is structural, not a margin argument.**

1. **ICBM was silent at every one of them.** Replaying the registry's 27 binding washout clusters
   against the telemetry recorded within ±12 s: ICBM was publishing a target at **0 of 27**.
   Independently corroborated by that drive's own report — *"ICBM published a target only 81 times
   all drive, all at 7–52 mph … 0/493 sharp vision curves at >55 mph."* Those washouts happened under
   **op-long/VTSC**, whose path this change does not touch.
2. **The shared penalty is unchanged**, so `test_washout_registry.py` passes unmodified: every
   binding washout still gets a cap ≥ 3 mph below the speed the truck actually carried in.
3. **Even replayed onto those sites with today's ICBM**, the floor moves the commanded target by a
   median **+0.85 mph** (max +4.5), on targets of 19–54 mph against entry speeds of **66–88 mph**.
   At 9 of the 27 sites today's ICBM publishes no target at all, and at 3 more (the candidates rated
   ≥ 60 mph raw, where the 1.35 end of the ICBM scale inflates far more than the penalty removes) the
   floor sits below the penalised target and never binds. 15 sites move, none by more than 4.5 mph.
4. **Where the floor acts is disjoint from where the washouts happened.** The floor only binds when
   `pen > raw·(scale − 1)`, i.e. for curves rated below **≈ 55 mph**. Every binding washout was an
   entry above 65 mph.

**Registry gap found and closed (independent of the fix).** The checked-in
`washouts_2026_07_11.json` stopped at **18:17 PT** because `ces_events_1917.jsonl` was pulled off the
device *after* it was generated. It therefore never contained the **19:16 / 19:17 PT downhill-LEFT
washouts that `descentcurve2pnw`'s `left_factor` and `descent_gain` were built from** — the
regression test has never been run against them. Regenerated with `tools/washouts.py` over the full
folder: **158 → 171 clusters, 27 → 35 binding**, and the shipped penalty pipeline passes all 35 with
no change. The coverage assertion now fails if the fixture ever stops short again.

## Verification

* `selfdrive/controls/lib`: **1,714 passed**, 0 failed (934 in `ces_pnw/tests`, of which 22 new).
* `selfdrive/car` (excluding the network-dependent `test_models.py`): 362 passed, **2 failed + 1
  import error that are PRE-EXISTING on `origin/3devpnw`** — verified by running the same three files
  in a clean worktree at `7c40d8003b` (`test_oplong_carswap.py::TestOpLongResetFailureRetry` ×2,
  `test_cruise_speed.py` ImportError). Not caused by this change.
* Mutation testing (`_scratch/icbmslow/mutate.py`, every mutant anchor-checked for exactly one match
  and `compile()`-checked before counting): **24 mutants, 23 killed, 0 invalid, 1 survivor** — M23
  ("outer zero clamp removed"), proven **equivalent**: the `rain2pnw` line two statements later
  re-applies `max(…, 0.0)` to the same variable. The clamp is kept as a local double-clamp and the
  verdict is recorded in the harness rather than the mutant deleted.
* `ruff`: no new findings (the 4 in `ces_pnw.py` and 3 in `test_icbm_bridge.py` are byte-identical
  to `origin/3devpnw`).

## Open, for the owner

1. **Is "mapd's own rating" the right ceiling, or should it be higher?** The floor lands the median
   at 1.65 m/s². Reaching the 2.50 design target needs ~1.22 × mapd's number, which is above the
   driver's own p99 in a bend (2.52) and has no field validation. Not proposed. If more is wanted the
   knob to add is a floor *fraction above* 1.0, and it should be earned on a drive, not a desk.
2. **The interim, zero-code option** is a `curve.json` — but measure expectations: `penalty_max` 5→3
   buys a median **+0.64 mph** and does not fix the 14:25 class of event, and it weakens op-long's
   washout margin as a side effect.
3. **`map_scale` 0.92 is now doing almost nothing at the tight end** (composite 1.012) and a 24 %
   inflation at the sweeper end. That is an accident of two independent changes, not a design. Worth
   folding into one number — deliberately **not** done here (it changes candidacy).

## Related

`docs/pnw/CURVESLOW2PNW.md` (the penalty and its 2026-07-11 calibration) · `docs/pnw/ICBM2PNW.md`
(the actuation path) · `docs/pnw/ICBMCURV2PNW.md` · `docs/pnw/SHARPCURVE2PNW.md` ·
`docs/LIGHTNING-STEERING-LIMITS.md` (the measured ~4.5 m/s² hands-off ceiling) ·
`drives/2026-09-17/curvedb-first-capture/DRIVE_REPORT.md` (the complaint and finding 2) ·
`drives/2026-07-11/lightning-icbm-nofire/DRIVE_REPORT.md` (the washouts).
