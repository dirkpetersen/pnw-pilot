# CHANGELOG — 2026-09-18 (Friday)

Continues [`CHANGELOG-2026-09-17.md`](CHANGELOG-2026-09-17.md). Overnight work on two questions the
owner asked: build curvedb Phase 2 offline, and fix the ICBM over-slow.

**Channel tip:** `origin/3devpnw` = `a46b90f2b5`. `icbmslow2pnw` is built and in Fable re-review, **not
pushed** — it changes how the truck brakes and its post-review commit has not been reviewed.

---

## 🔴 curvedb Phase 2: the §7 replay was BUILT and RUN, and it returned NO RESULT

Branch `curvedb2pnw`, **pushed** (`a46b90f2b5`) — 4,852 lines, **100 % under `tools/curvedb/`**, zero
files outside it, so it cannot reach the car. 315 tests, **81/81 mutants**, Fable SHIP WITH CHANGES
(22 findings, 16 fixed, 6 deferred).

**Over 84 files / 400,583 unique ticks / 139 drives / 170 ICBM episodes, under leave-one-date-out the
DB matched a row on 4 and acted on ZERO.** "0 real slowdowns wrongly cancelled" over 0 actions bounds
nothing, and the replay exits non-zero rather than print a percentage of an empty set. Loosening the
match radius to 300 m finally produces 2 actions — **both false cancels**, both on junctions measured
at 5 m/s. The only parameterisation that acts produces only garbage.

### The blocker is SITE RECURRENCE, not data volume

Leave-one-date-out needs the same curve driven on **≥3 separate dates**. Measured twice, independently:

| measurement | sites | with ≥3 dates |
|---|---|---|
| the build pipeline, 170 episodes | 170 | **15** |
| independent re-measure from raw GPS, 150 m clustering | 193 | **4** (185 of 193 seen on ONE date) |

Even a perfect ingest tops out near 15 adjudicable actions against §3.9-5's **N ≥ 60**. More driving on
NEW roads does not help — new roads add one-visit sites.

### ⚠️ But both numbers are lower bounds on a BIASED sample, and that is the actionable part

`drives/` is not a log. It is a pile of **analysis windows** pulled when something interesting happened:
**median span 62 minutes; 1,697 of 3,450 files under one hour; 74 over six hours.** A road driven every
week appears once if a window was pulled once. So this measures *what we chose to pull*, not *what the
truck drives*.

**The question has never actually been asked** — and `/data/pnw/ces_archive`, which shipped yesterday,
is the first continuous unbiased record this project has ever had. Re-running
`tools/curvedb/recurrence.py` on a few weeks of it yields one number that decides whether Phase 2 is
viable at all. That retroactively justifies Phase 1's retention half beyond mere volume.

**Do not build Phase 2's on-car half.** If recurrence stays low on unbiased data, the premise is wrong
for this driver's pattern and Phase 2 should be **abandoned rather than tuned** — a legitimate §7 outcome.

### Other blockers, worth keeping even if Phase 2 dies
* **§6.1 rejects 147/170 episode passes**, `drv` on 136 — as written **no Tesla road can ever be learned**.
  The `drv` rule alone costs 19 of 33 genuine revisits. Decidable today, no new driving.
* **§6.3 never states at what distance the approach bearing is measured**; **13.6 %** of sites exceed the
  tolerance across ICBM's decision range. (Corrected 2026-09-19 from 22.2 % — that denominator matched
  nothing in the pipeline. Now reproducible: `tools/curvedb/calib.py`, committed for exactly that reason.)
* ~~**§6.2's DOWN rule produced ZERO observations** on 2.2 GB.~~ **WRONG — corrected 2026-09-19.** It was
  **structurally unreachable**: the passage was discarded as `drv`-dirty *before* the DOWN loop ran, and a
  DOWN **is** a `strPrs` tick. So the rule that exists to record the DRIVER'S OWN interventions had never
  recorded one, anywhere. Fixed: **127 observations**. It did NOT change the headline — the replay still
  acts on zero of 170 — but the zero was a bug, not a fact about the driving, and the old text explained
  it as "the interventions are simply below the trigger", which was the reverse of the truth.
* **A 1 Hz-built row under-reads curvature p50 1.10× / p90 1.51×** — permitting a speed 5–23 % too high,
  the UNSAFE direction. Exactly why §3.3 demands the per-second peak.
* **`slKCmd` is alive on the Tesla** — independently verified at **98.2 % live over 52,514 moving Tesla
  ticks**, against `slKActl` at **0 % live / 51,754 exact zeros**. So D1 is confirmed *exactly as written*
  (no achieved curvature from CAN), and what is alive is the planner's COMMAND, not a measurement of the
  road. Narrows D1; does not satisfy §3.9-1. Recorded as a deviation, not a free win.

## 🟠 The ICBM over-slow is a dated REGRESSION, not a tuning preference

Branch `icbmslow2pnw`, built, **in Fable re-review, NOT pushed**.

**Root cause.** `eff = icbm_map_eff_scale(raw) · raw · map_scale`. The 5 mph penalty hump was calibrated
**2026-07-11**, when the composite was a flat **1.35 × 0.92 = 1.242×** — so targets landed *above* mapd's
rating (the 09-17 14:25 curve would have been commanded at **49.6 mph**). On **2026-08-11**
`icbmcurve2pnw` dropped the tight-end tier to **1.10** for a different and correct reason, making the
composite **1.012×**. Nothing re-calibrated the hump.

> **ICBM began commanding below mapd's own number on 2026-08-11.**

### ⚠️ Correction to the 09-17 drive report's decomposition
It said "44 × 0.92 = 40.5", omitting `icbm_map_eff_scale`. On that curve **`map_scale` contributes ~0.5
mph; the hump contributes nearly all the rest.** The report has been corrected.

### And a retraction: DO NOT set `map_scale: 1.0`
Recommended to the owner on the evening of 09-17. **Replayed over 88 episodes it deletes 18 slowdowns
outright.** `map_scale` multiplies the *effective* speed that reduce-only candidacy is tested on, so
raising it pushes borderline candidates above the binding threshold and they stop binding at all. Verified
independently against the shipped `icbm_far_map_candidate`. It would not have softened curves; it would
have removed some.

### Evidence, at 24× yesterday's sample
92 corpora / 730,887 records / **n = 93** episodes above 25 mph with a curvature witness taken *as the
truck traverses the curve* (not same-tick), across 7 drives:

| at what speed | lateral accel |
|---|---|
| ICBM's command | **1.34 m/s²** |
| mapd's raw rating | 1.67 |
| the driver's set (ceiling) | 2.78 (p90 **7.59** — why slowdowns exist) |
| design `A_LAT_TARGET` | 2.50 |

**The driver's own revealed preference settles the "aim at 2.50?" question:** in a bend above 45 mph his
**p90 is 1.63 and p99 is 2.52**; only **0.73 %** of his >25 mph ticks ever exceed 2.5. Aiming at
`A_LAT_TARGET` would drive the truck harder than he ever drives it. **Not proposed.**

### The candidates, replayed through the shipped code

| variant | Δ target | a_lat median | **slowdowns LOST** |
|---|---|---|---|
| shipped | — | 1.41 | 0 |
| `map_scale` → 1.0 | +2.36 mph | 1.78 | **18 of 88** |
| `penalty_max` 5→3 | +0.64 mph | 1.50 | 0 |
| hump peak → 62–78 mph | +0.43 mph | 1.50 | 0 |
| **map-rating floor (built)** | **+1.31 mph** | **1.65** | **0** |
| pre-08-11 code | +6.51 mph | 2.19 | **32 of 88** |

**The built fix:** the Lightning penalty may not push a MAP/FAR target below `icbm_map_floor_frac` × that
candidate's own raw mapd rating. Applied after candidacy; vision never floored; `curve_speed_penalty_ms`
untouched so op-long is byte-identical. Telemetry `icbmMapFlr`/`icbmMapFlrHit` plus a startup log so a
curve.json off-switch cannot go silent. **1,717 tests, 26 mutants / 25 killed / 1 proven-equivalent.**

### The washout check — the agent corrected itself twice, which is why it is believable
First answer "0 of 27 binding washouts had ICBM live" was **wrong** — true of the stale fixture, false of
the 35 its own regeneration produced. Corrected: **32 of 35 had no ICBM target** (op-long/VTSC); **3 were
stock-ACC with ICBM live**, two of them vision-sourced on every tick. The floor's entire effect on the
washout evidence is **+4.4 mph, one tick, one site**, where the binding constraint was the executor's
1 mph/0.4 s walk-down. The floor structurally cannot bind above **53.6 mph raw**; every washout entry was
above 65.

**Side finding:** the checked-in washout registry **stopped at 18:17 PT and never contained the 19:16/19:17
downhill-LEFT washouts that `left_factor` and `descent_gain` were built from.** Regenerated 158→171
clusters, 27→35 binding; the shipped pipeline passes all 35.

### Open for the owner
1. `icbm_map_floor_frac` **defaults to 1.0 — the floor is ON at full strength on install.** Fable has been
   asked whether a restoration counts as "new" under the default-OFF convention.
2. Is mapd's rating the right ceiling? The floor lands the median at 1.65 m/s². Reaching 2.50 needs ~1.22×
   mapd's number — above the driver's own p99 in a bend, no field validation. **Not proposed.**
3. `map_scale` 0.92 now does ~nothing at the tight end and +24 % at the sweeper end — an accident of two
   changes, deliberately not folded into this branch because it changes candidacy.
4. `penalty_min/max_mph` lose their leverage on the ICBM map path below 53.6 mph raw once this ships —
   written down so a future tuning session does not reach for a dead knob.
