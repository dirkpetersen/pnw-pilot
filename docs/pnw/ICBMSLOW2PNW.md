---
updated: 2026-09-17
status: current     # current | drifted | superseded | unreviewed
---

# ICBMSLOW2PNW — ICBM over-slows for curves: the map-rating floor

> **⛔ NOT DEPLOYED, NOT PUSHED.** Fable-reviewed 2026-09-17 (**SHIP WITH CHANGES**; every
> required change is applied — see *Verification*). Branch `icbmslow2pnw` off `origin/3devpnw`
> (`cfcbf1cf4c`). This is control-path code that changes how the truck brakes. The owner decides.
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

Per-corpus field availability is reported in full — committed at
[`icbmslow-evidence/out_a1_capability.txt`](icbmslow-evidence/out_a1_capability.txt), produced by
`/home/dp/gh/comma/_scratch/icbmslow/a1_capability.py` — precisely so that a corpus contributing
nothing is visible rather than silent:

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

**No — but the first version of this section was wrong, and the corrected argument is narrower and
better.** (Two independent adversarial reviews, 2026-09-17, caught the same hole: the claim
"ICBM was publishing a target at 0 of 27" was computed against the **old** 158/27 fixture and never
re-run after this branch regenerated it to 171/35. It is true of those 27, and false of the 35.)

**32 of the 35 binding washouts: ICBM was silent.** Replaying every cluster against the telemetry
within ±12 s (full coverage at each — 23–26 records, `icbmT` present as a key in 100 % of them, and
non-null 119 times elsewhere in the drive, so a null is a null and not a missing field): 32 clusters
have no ICBM target at all, all `shadow: false`, i.e. **op-long/VTSC was the actor**. This change
does not touch that path.

**3 of the 35 were stock-ACC with ICBM live** — 19:04:43 / 19:05:30 / 19:06:52 PT, `shadow: true`,
in the evening `ces_events_1917.jsonl` session. `tools/washouts.py` records their `cap_ms` as *the
lowest ICBM target*, so by the tool's own definition these are ICBM washouts. Replayed tick by tick:

| cluster | entry | ICBM source | what the floor does |
|---|---|---|---|
| #165 19:04:43 | 86.3 mph | **vision on all 5 ticks** | nothing — the map candidate does not exist (`mapV` 72.9 raw → eff 90.5 mph ≥ the 88 mph set, so it is rejected as not reduce-only) |
| #166 19:05:30 | 88.1 mph | **vision on all 6 ticks** | nothing — same shape (`mapV` 73.1 → eff 90.8 ≥ 89) |
| #167 19:06:52 | 87.2 mph | **map on 1 tick, vision on 10** | on that one tick the candidate is 50.0 mph, shipped commands 45.0, floored commands **49.4** |

So the floor's entire effect across the washout evidence is **+4.4 mph, on one tick, at one site** —
and one second later vision takes that same curve over and demands 51 → 53 mph with **no floor
applied at all** (vision candidates are never floored, and the lowest binding candidate wins, so a
vision candidate that disagrees with the map always overrides it downward).

**At all three, the target was never the binding constraint.** The truck entered at 86–89 mph
against ICBM targets of 51–58: it was ~30–36 mph over, and what limits the approach there is the
executor's **1 mph per 0.4 s** SET− walk-down, not the level the target sits at. A 4.4 mph change to
one tick of that walk is not what decides whether the driver takes the wheel at 89 mph.

**Bounded, not argued.** `test_icbm_sourced_caps_survive_the_map_rating_floor` asserts it rather than
reasoning about it: for every ICBM-sourced binding washout, `cap + penalty_cap_mph` (15 mph — the
hard cap on the whole hump, hence an upper bound on anything the floor can give back whatever the
candidate's rating was) still sits ≥ 3 mph below the entry speed. All three pass with 11–19 mph of
margin.

**The shared penalty is unchanged**, so `test_new_pipeline_caps_at_least_3mph_below_every_recorded_entry`
passes unmodified over all 35, and VTSC / op-long is byte-identical.

**Where the floor can act is bounded by the scale.** It binds only when the base hump exceeds what
the ICBM scale inflated, i.e. for candidates rated below **53.6 mph raw** (pinned by
`test_the_floor_binding_range_is_bounded_by_the_scale`). Every binding washout is an entry above
65 mph; 32 of 35 have no ICBM candidate at all.

**A retracted corroboration.** The 2026-07-11 drive report's *"ICBM published a target only 81 times
all drive, all at 7–52 mph"* and *"0/493"* are **not** independent corroboration: that report was
written at 16:40 PT, before three of the four `ces_events` files existed, and recomputing its figure
from `ces_events.jsonl` alone reproduces it exactly (81 records, 6.9–51.7 mph). Twenty-nine ICBM
ticks that day carry targets above 52 mph, all in the file the report never saw. The claims above are
measured directly instead.

### Registry gap found and closed (independent of the fix)

The checked-in `washouts_2026_07_11.json` stopped at **18:17 PT** because `ces_events_1917.jsonl` was
pulled off the device *after* it was generated. It therefore never contained the **19:16 / 19:17 PT
downhill-LEFT washouts that `descentcurve2pnw`'s `left_factor` and `descent_gain` were built from** —
the regression test had never been run against them — nor the three ICBM-live clusters above.
Regenerated with `tools/washouts.py` over the full folder: **158 → 171 clusters, 27 → 35 binding**,
and the shipped penalty pipeline passes all 35 with no change. The coverage assertion now fails if
the fixture ever stops short again.

## Known limits of this analysis

* **One corpus dominates.** 64 of the 88 replayable episodes are the 2026-09-12 central-Oregon
  weekend. Excluding it (n = 24) the median lateral accel at the commanded target is 0.58 m/s² and
  the floor's median gain is +0.79 mph — those 24 are mostly near-straight urban episodes
  (a_lat at `mapV` 0.44 on Crown Hill). The two corpora with real curvature agree with each other:
  central Oregon 1.57 / 1.81 / 1.70 (ship / mapV / floor) and 2026-09-17 1.54 / 1.84 / 1.88.
* **The replay treats every curve as flat and right-handed.** `icbmDir` in `ces_events` is the
  episode direction (`dec`/`inc`), not the curve's, and `vtscPitch` is only published on the op-long
  path — so neither the descent guard nor the left factor is represented. The *delta* is unaffected
  (the floor gives back exactly the base hump either way), but the absolute lateral accels quoted are
  slight OVER-estimates on downhill lefts, i.e. conservative.
* **Pre-2026-08-11 corpora cannot be scored.** 68 of the 251 episodes have no curvature witness.
* **`penalty_min_mph` / `penalty_max_mph` lose most of their leverage on the ICBM map path** once
  this ships: below 53.6 mph raw the base hump is capped at `eff − raw` (~0.5 mph at a 44 mph rating),
  and only `left_factor` / `descent_gain` extras remain. Lowering `map_scale` does **not** restore
  base penalty there — below 0.909 the candidate simply drops under its own raw rating and the floor
  becomes the candidate. VTSC keeps the full hump. This is the intended effect; it is written down
  because a future tuning session that reaches for `penalty_max` on this path will find nothing.

## Verification

* `selfdrive/controls/lib`: **1,717 passed**, 0 failed, of which 956 in `ces_pnw/tests` and **24 new
  in `test_icbmslow2pnw.py`** plus 1 new in `test_washout_registry.py`.
* `selfdrive/car` (excluding the network-dependent `test_models.py`): 362 passed, **2 failed + 1
  import error that are PRE-EXISTING on `origin/3devpnw`** — verified by running the same three files
  in a clean worktree at `cfcbf1cf4c` (`test_oplong_carswap.py::TestOpLongResetFailureRetry` ×2,
  `test_cruise_speed.py` ImportError). Not caused by this change.
* Mutation testing (`_scratch/icbmslow/mutate.py`, every mutant anchor-checked for exactly one match
  and `compile()`-checked before counting; an inapplicable mutant is reported INVALID, never
  "killed"): **26 mutants, 25 killed, 0 invalid, 1 survivor** — M23 ("outer zero clamp removed"),
  proven **equivalent**: the `rain2pnw` line two statements later re-applies `max(…, 0.0)` to the
  same variable. The clamp is kept as a local double-clamp and the verdict is recorded in the
  harness rather than the mutant deleted.
* `ruff`: no new findings (the 4 in `ces_pnw.py` and 3 in `test_icbm_bridge.py` are byte-identical
  to `origin/3devpnw`).
* **Reviewed by Fable, 2026-09-17: SHIP WITH CHANGES.** No code defect found in `_icbm_step`; a
  numeric sweep over `map_scale` {0.5…1.0} × `penalty_max` {2,5,15} × raw 10–88 mph × pitch × left
  found 0 invariant violations. Its required change was to the washout evidence in this document
  (F1), which has been rewritten above; F2 became
  `test_icbm_sourced_caps_survive_the_map_rating_floor`, F3 the last bullet of *Known limits*, F4 the
  53.6 mph figure (now pinned by a test), F5 the evidence appendix below. F6 (a stale
  `icbmMapFlr` if `_icbm_step` raises between the floor and the publish) is noted and unchanged —
  it is the identical exposure `_icbm_floor_hit` already carries.
* **Evidence appendix:** the raw output of every analysis step is committed at
  [`icbmslow-evidence/`](icbmslow-evidence/) so the numbers above are auditable from the repo. The
  harness that produced them lives in the workbench at `/home/dp/gh/comma/_scratch/icbmslow/`
  (per the project's "harness in `_scratch/`" rule) with its own `README.md` — including the three
  method errors it caught in itself.

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
