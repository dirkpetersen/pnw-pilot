# curvedb — the OFFLINE half of CURVEDB2PNW.md Phase 2

**Status: NOT DEPLOYED, NO AUTHORITY, NOT PUSHED.** Built 2026-09-17 on branch `curvedb2pnw` off
`3devpnw` @ `7c40d8003b`; reviewed by Fable the same night (SHIP WITH CHANGES) and this is the
post-review state. Everything lives under `tools/`; nothing in `selfdrive/`, `system/`, `cereal/`
or any submodule is touched, and nothing the car imports imports any of it. Phase 2 remains gated
on §3.9 and this branch does not change that.

| file | what it is |
|---|---|
| `store.py` | **C1** (§8.3): the row store, the matcher, the update rules, the authority gate, the cancel formula. Zero openpilot imports, no I/O, no clock. **This is the module a future on-car consumer would import unchanged** — if the replay validated one implementation and the car ran another, the replay proved nothing. |
| `ingest.py` | ces_events corpora → observations + ICBM episodes, with a per-file capability report. |
| `replay.py` | **§7's go/no-go gate.** Leave-one-date-out, a stated N, and three adversarial self-checks. |
| `recurrence.py` | Does the truck actually re-drive the phantom roads? Measured from **raw GPS**, with the observation pipeline deliberately out of the loop — because measuring it *through* the pipeline gave the wrong answer. |
| `tests/` | 315 tests. 81/81 mutants killed (`_scratch/curvedb/mutate.py`). |

```bash
PYTHONPATH=. python3 tools/curvedb/ingest.py drives/**/ces_events*.jsonl \
    --out-observations obs.jsonl --out-episodes eps.jsonl
PYTHONPATH=. python3 tools/curvedb/replay.py --observations obs.jsonl --episodes eps.jsonl \
    --dq any --self-match --control k-shuffle --control site-shuffle
PYTHONPATH=. python3 tools/curvedb/recurrence.py drives/**/ces_events*.jsonl \
    --episodes eps.jsonl --observations obs.jsonl
```

---

## 1. THE RESULT: **NO RESULT**, and that is the honest answer

Run over every `ces_events` corpus under `drives/` — 84 files, **400,583 unique ticks** after
deduplication, 139 drives (1 dropped, §4), 16 dates, both cars, **4,719 observations**:

```
  episodes replayed                       170
  ... matched a row                       4  (2.4%)
  ... verdict no_row                      166 (97.6%)
  ... verdict no_authority                4   (2.4%)

  THE GATE (§7): episodes the database would have cancelled or reduced = 0
    of those, adjudicable (k_truth present)   0
    REAL SLOWDOWNS WRONGLY CANCELLED          0
    bound on the false-cancel rate            NONE -- 0-of-0 is not evidence.
```

**The zero in "0 real slowdowns wrongly cancelled" is worth nothing**, because the database never
acted. §7 has not been passed and has not been failed; it has not been *evaluated*. Reporting the
zero without the denominator would be the `getfattr` failure again — a uniform result read as a
finding instead of as an un-exercised method.

**The 170 has its own denominator, and it is a selection effect worth seeing.** ICBM produced
**433** target runs in this corpus: 50 were restore-only (an increase, not a slowdown), **213 had
no measurable passage within 500 m ahead**, and 170 survived. Half the population is excluded
before the replay starts, for a reason (no candidate, or the drive ended) that is not random.

### The three adversarial checks

| check | result | reading |
|---|---|---|
| **self-match** (no LODO — circular by construction) | **26** rows vs 4 under LODO | Rows are found when the pass that built them is present, so the LODO 2.4 % is about the corpus, not a broken key. Caveat Fable raised, and it is fair: the episode's site/bearing and the observation's come from the *same* `Passage` object, so this asks whether a row anchored at x contains x. It proves `build()`/`match()` are self-consistent; it cannot detect an ingest-vs-lookup convention mismatch. |
| **`k-shuffle`** (right places, wrong curvatures) | identical funnel (4 matched, 0 actions) | Cannot discriminate — because nothing acts. Uninformative here *by construction*, and the tool says so. |
| **`site-shuffle`** (right curvatures, wrong places, permuted **within each date**) | 4 matched, 0 actions — same as the real DB | The 4 matches survive scrambling, i.e. they are row-density artifacts rather than evidence the truck drove that place before. The tool fires its own alarm saying exactly that. |

> The site-shuffle was **rebuilt after review**. It used to permute globally, which reassigns sites
> across dates and thereby *manufactures* multi-date rows the real corpus does not have — so the
> corrupted database came out looking **more** capable than the real one (21 matches vs 4), and the
> control's own alarm text fired for a reason unrelated to what it asserts. A control whose failure
> message is known-false is not a control.

### Loosening the match until it *can* act makes it worse, not better

At `--site-radius-m 300 --heading-tol-deg 45` the database finally acts on 2 episodes — and
**both are false cancels, a 100 % failure rate**:

```
2026-08-26 k=0.06654@5m/s (sample1hz_cmd) icbm 2.6 -> db 11.2 m/s  a_cf=8.35
2026-08-26 k=0.06654@5m/s (sample1hz_cmd) icbm 2.6 -> db 11.2 m/s  a_cf=8.35
  of these, 2 had their curvature measured below 60 % of the counterfactual speed
```

`k = 0.067` at **5 m/s** is a 15 m radius — a junction, not a road curve. So these are not credible
false cancels either; they are the matcher, at a 300 m radius, keying onto a different piece of
road. The honest summary is **"the only parameterisation that produces any action produces only
garbage"**, which is a stronger argument against loosening the match than any of the above.

### Stratified to the roads the phantoms are actually on

`--highway-only` (motorway/trunk) keeps **25 of 170** episodes, and on those the LODO match rate is
**4/25 = 16 %** rather than 2.4 % (self-match 12/25 = 48 %). Still zero actions — every match is a
single pass — but it says the corridor sites are considerably less sparse than the corpus average,
which is the population §1 of the design is about.

---

## 2. The retrospective corpora cannot certify anything, and here is exactly why

The Phase-1 fields (`kPeak` `kPeakN` `kPose` `kPoseP` `achLatPose` `dq` `dqWhy` `strTq` `mapLat`
`mapLon` `mapCandD`) exist only from 2026-09-17. Everything before is a reconstruction, and
`ingest.py` reports per file which vocabulary it had.

| provenance of the 4,719 observations | count | consequence |
|---|---|---|
| `kPeak100` — the real 100 Hz peak | **166** | the only rows built the way §3.3 specifies |
| `sample1hz_cmd` — commanded only (every Tesla pass) | 3,196 | a *model* estimate, not a CAN measurement (see §5 I8) |
| `sample1hz_cmd_actl` — max of both, 1 Hz | 1,338 | §3.3's estimator, at 1/100th the rate |
| `sample1hz_actl` — achieved only | 19 | D2: bounded by steering authority |
| `site_src="logged"` (`mapLat`/`mapLon`) | **166** | keyed on the candidate, as §6.3 requires |
| `site_src="track"` (reconstructed from the truck's later position) | 4,553 | keyed on a reconstruction of it |
| `dq_src="rollup100"` (§3.5's 100 Hz OR) | **166** | a `clean` that means what §3.5 says |
| `dq_src="sampled1hz"` (instantaneous flags) | 4,553 | a `clean` that may have been *between* events |

(Across all sites *considered*, not just those that became rows: 307 logged, 27,563 reconstructed.)

**Measured, not assumed — how bad is the 1 Hz estimator?** On the one corpus where both exist
(2026-09-17), per measurement extent, `max(kPeak) / max(1 Hz sample)` is **p50 1.10, p90 1.51,
max 2.88**. Since `v = sqrt(a/k)`, a 1 Hz-built row permits a speed **5 % too high at the median,
23 % at p90, 70 % at worst.** Per tick the sample under-reads by more than 20 % on **74 %** of
ticks. Every 1 Hz row is therefore biased in the direction that **under-brakes**.

---

## 3. What actually blocks the gate

### 3.1 The sites ARE re-driven — the admissibility pipeline throws the revisits away

This reversed under an adversarial check, so both numbers are given, and the check is now a
committed tool (`recurrence.py`) rather than a scratch script.

Via the observation pipeline, only 7 of 170 episodes have a row at their site from another date,
which reads as "the truck does not repeat these roads". **It does.** Measured from raw GPS ticks,
bypassing observations, passages and disqualifiers entirely:

| | episodes (of 170) |
|---|---|
| the truck came within 40 m of the site at all | **169** |
| ... on a **different date** | **39** |
| ... on a different date *and* a compatible approach bearing | **33** |
| ... on **≥2 other dates** — what D6 needs under leave-one-out | **15** |
| ... and a usable row actually exists there | **7** |

**26 of the 33 genuine revisits are lost inside our own pipeline.** Attributed by the
disqualifier's own named cause:

| why the revisit produced no row | count |
|---|---|
| **disqualified, `drv` involved** (`drv` 12, `sat,drv` 4, `drv,blnk` 2, `sat,drv,lc` 1) | **19** |
| disqualified, `sat`/`blnk` only | 4 |
| no map candidate / no approach bearing / never became a site | 3 |
| *(a row exists)* | 7 |

Two conclusions, and they must be kept apart:

* **For row EXISTENCE, the driving pattern is not the primary blocker — §6.1's disqualifier is.**
  19 of 33 revisits lost to `drv`-involved causes alone.
* **For the GATE, that is not enough.** Authority needs ≥2 passes on ≥2 dates, and under
  leave-one-out that means ≥2 *other* dates — true for only **15 of 170** sites
  (`{1 other date: 18, 2: 4, 4: 7, 5: 2, 12: 2}`). So even with a perfect pipeline this corpus
  tops out around 15 adjudicable actions, and **§3.9 item 5's N ≥ 60 is unreachable here under any
  dq rule.** Fixing the `drv` question raises the ceiling; it does not by itself reach the bar.

Secondary, still true: 3,899 rows exist and **762 have ≥2 passes on ≥2 dates** (~20 %), but those
are Seattle city loops with no ICBM slowdowns on them.

### 3.2 §6.1's disqualifier rejects 147 of 170 episode passes

`dq_state` on the 170 ICBM episodes: **147 dirty, 23 clean.** An extent can carry more than one
cause, so the episode column sums above 147:

| cause | disqualified passages (of 3,087) | episodes carrying it (of 147) |
|---|---|---|
| `drv` — driver steering (`strPrs`) | 2,413 | **136** |
| `sat` — steering saturation | 976 | 41 |
| `blnk` — blinker | 432 | 21 |
| `lc` — lane change | 113 | 1 |

**Driver steering, not saturation, is what disqualifies these passes.** Two consequences the design
does not discuss:

1. A pass is only admissible where openpilot was steering. On the **Tesla**, and on any drive with
   lateral disengaged, `strPrs` is true continuously — so those roads can never be learned at all.
2. An ICBM slowdown *provokes* the driver to steer. The rule therefore systematically excludes the
   population it exists to serve.

Re-running ingest with `sat` and `blnk` relaxed (`--dq-flags drv,lc`) recovers only ~600 of 3,087
passages (+13 % observations) and changes the replay verdict not at all. Relaxing those is not the
fix; the `drv` question is the one worth asking.

> **Open question for the owner (do not act on this unasked):** should a *brief* steering input
> anywhere in a 175 m extent disqualify a curvature measurement that is, after all, a property of
> the road? `ingest.py --dq-flags` exists so this can be answered with a number instead of an
> argument, and `recurrence.py` measures what the answer buys.

### 3.3 §6.3's "direction by approach bearing" is under-specified, and the gap is 22 %

§6.3 replaces v1's 45° buckets with "the approach bearing at lookup time" but does not say **at
what distance**. Measured over 2,074 site passages, the truck's own bearing at 500 m, 300 m and
150 m before a site spreads by **p50 17.7°, p90 59°, max 180°** — and **22.2 % of sites exceed the
35° matching tolerance.** ICBM decides anywhere from 150 m to its 500 m far-source horizon, so on
one site in five, ingest and the car can disagree about which direction "this way" is and the row
is simply never found. `approach_bearing_ref_m` is a real parameter with a real cost.

### 3.4 §6.2's DOWN rule produced **zero** observations on 2.2 GB of driving

Not one steering override inside a site extent met §6.2's own trigger (measured lateral accel
≥ 2.5 m/s² with a commanded curvature to compute the magnitude from). The tally prints
`down_dropped_no_kcmd 0` and `down_no_lateral_accel_witness 0` **explicitly** — the counters are
pre-seeded, so this is read rather than inferred from an absence — so it is not a plumbing failure;
the interventions are simply below the trigger. Consistent with §6.2's own measurement (113 of
2,986 overrides), but it means **the DOWN half of the design is entirely unexercised**.

### 3.5 Episode→site attribution is still short of where ICBM decides

With ICBM's own logged candidate used where it exists and its logged distance otherwise (156 of 170
use `map_dist`, 8 logged coordinates, 4 nearest-ahead), the attributed site sits p50 ~71 m ahead of
the decision — short of the 150–500 m ICBM reacts at, because `mapDist` is CES's own 10 s candidate
and not ICBM's far one. This is why the verdict is taken on `max(k_truth, k_ahead_max)` over the
whole 500 m window (I7) rather than on the chosen node.

---

## 4. Findings that are NOT about the blockers

* **The Tesla is not void after all — `slKCmd` is alive on it.** D1 says the Raven cannot teach the
  Lightning because `slKActl` is dead (confirmed: exactly 0.0 on essentially every moving Tesla
  tick, every corpus). But the **commanded** curvature is alive on **99.4–99.8 %** of moving Tesla
  ticks with the correct sign (agrees with `strAng` on 95–100 % of records where |strAng| > 8°).
  §3.3 itself argues commanded is the *better* half on curves. Result: **2,931 of the 4,719
  observations are Tesla passes** — more than the Lightning's 1,788. This is a **deviation from
  §3.9 item 1**, not a footnote; it is written up as I8 in §5.
* **The 2026-09-17 pose sign fix is confirmed live.** `ingest.py`'s per-file sign check reads
  **INVERTED** on the two 09-17 daytime corpora (median `kPose/slKActl` −0.85 and −0.71) and
  **normal: +0.950 over n=221** on the 18:29 PT lane-change extract. `36f914a17c` installed at the
  evening ignition, exactly as `DRIVE_REPORT.md` predicted.
* **Sign-inversion blast radius is smaller than it looks.** `Observation.k` is a magnitude and a
  row's direction is the GPS approach bearing, so an inverted `kPose` corrupts the signed telemetry
  columns and *not* the rows. Detection is reported per file, but it is not the thing standing
  between the defect and a poisoned database.
* **`drives/` is full of duplicate corpora.** 12 of 84 files contributed nothing but duplicates (a
  `.gz` beside its `.jsonl`, one `ces_tail.jsonl` filed under two incident folders, overlapping
  pulls). Left in, they would have manufactured the "2 passes" half of D6 out of a single drive.
  The dedup key is `(car, ev, round(t, 2))` — **`ev` matters**: a `tick` and an `adopt` can share a
  timestamp (3 of 51 adopts in a 6,000-line sample), and the `adopt` is the record carrying
  `icbmT`/`icbmSrc`/`mapLat` at the decision instant, so keying on `(car, t)` alone silently threw
  one of them away as a "duplicate".
* **Dead-RTC timestamps are real.** Several corpora open in November 2025 or July 2028 before NTP
  lands. Since the PT date is the leave-one-out key, a misdated record leaks a pass into the wrong
  fold. Dropped at 36 h from each file's own median, and counted.
* **`vEgo` is confirmed to be m/s** — the GPS track and the integrated odometer agree at a median
  ratio of **1.000** across 139 drives. Exactly one drive (2026-08-26, Tesla) read **0.61** and is
  **dropped**, not merely warned about: its extents, approach reference and lookahead would all be
  measured against a distance the two sources disagree about, and downstream those rows would be
  indistinguishable from good ones.
* **The episode population is mostly not the phantom problem.** Only **25 of 170** episodes are on
  `motorway`/`trunk` (48 `secondary`, 45 `tertiary`, 12 `primary`, 11 `unknown`). `v_ego` at the
  decision is p50 **14.3 m/s** (32 mph). §1's phantoms are a *highway* problem.

---

## 5. Where this implementation interprets or deviates from CURVEDB2PNW.md

Each is a decision a reviewer should ratify or overturn. None is silent. **Fable ratified I1–I7 on
2026-09-17** (I7's *direction*, with the estimator fixed — see §9); I8 was added at Fable's
instruction because it was being presented as a finding when it is a deviation.

| # | design says | this does | why |
|---|---|---|---|
| I1 | §3.9-6: "real slowdown = measured `achLat` ≥ 2.5 at the speed driven" | judges the **counterfactual** `k_truth · v_allowed²`, and prints the measured reading beside it | Read literally the design's test is circular in the opposite direction: ICBM's job is to reduce the speed driven, so every *successful* slowdown leaves a low `achLat` and scores as a phantom. The counterfactual is available only because §4 stores curvature. |
| I2 | §6.2: `v_row = sqrt(A_LAT/\|slKCmd\|)` — a speed | stores the **curvature** `\|slKCmd\|` and derives the speed | Algebraically identical, keeps §4 ("never store a speed") intact, and makes D8 fall out of a `max` instead of precedence bookkeeping. |
| I3 | §6.4 excludes **ramps** | also refuses when the highway class is **unknown** | `unknown` is a real `HighwayClass` member and arrives as a *truthy string*; 990 of 7,813 records on the 2026-09-12 corpus carry it. A ramp we cannot see is still a ramp. **Stricter than the design.** |
| I4 | §8.2: "a `CarSpecs` physics fallback for unknown keys" | **refuses authority** for any platform with no measured envelope (i.e. everything except the Lightning) | Fable's own correction in §8.2 is that mass does not predict a lateral ceiling. A physics fallback would be an invented number wearing a derivation. **Deviates from the design.** |
| I5 | §7 counts phantoms refuted and real slowdowns cancelled | adds a **grey band** (`phantom < 1.5 ≤ grey < 2.5 ≤ real`) | The design defines "real"; it does not define "phantom". Folding the middle into "refuted" would be the flattering choice. |
| I6 | §6.3 keys on mapd's candidate position | reconstructs the candidate from the **truck's own later track** when it was not logged, tagged `site_src="track"` | Nothing before 2026-09-17 logged where the curve was (D3). The truck drives the road, so its future position at `mapDist` further on *is* the candidate; projecting along the instantaneous bearing puts the point off the road exactly where the road bends. Note (Fable): a `track` site is the truck's *lane* position and a `logged` site is mapd's node — the same curve can carry both, and `site_radius_m` must absorb that too. |
| I7 | §7 verdict is per-episode | judges on `max(k_truth, k_ahead_max)` over the whole 500 m lookahead, **both with the `min_speed_ms` floor** | Attributing an episode to a map node is imprecise; if there was a demanding bend anywhere in the 500 m the truck was about to drive, cancelling was wrong regardless of which node was keyed on. **Stricter**, and it removes the attribution choice from the verdict. |
| **I8** | §3.9-1: achieved curvature alive on both cars, **or Tesla passes excluded by an explicit rule** | neither — Tesla passes are **admitted on `slKCmd` alone** (62 % of all observations) | §3.3 argues commanded is the better half on curves, and it is sampled at the truck (0 m), where §1's own vision-vs-distance table puts the model at median 1.01× truth. Defensible — but 62 % of the road table being a model estimate rather than a CAN measurement is a decision for the owner, not a footnote. **Deviates from the design.** |

---

## 6. Every PROVISIONAL constant

None has been chosen by data. `store.py`'s live in `PROVISIONAL_PARAMS` (the dataclass itself has
**no field defaults** — a caller must name the instance); `ingest.py`'s and `replay.py`'s are module
constants with the same comment discipline.

### Cited from the design — not invented here

| name | value | source |
|---|---|---|
| `extent_back_m` / `extent_fwd_m` | 25 / 150 m | §6.3 verbatim |
| `min_passes` / `min_dates` | 2 / 2 | §6.4, D6 |
| `a_lat_comfort_ms2` | 2.5 m/s² | VTSC `A_LAT_TARGET`, retuned 2026-07-01 |
| `down_trigger_a_lat_ms2` | 2.5 m/s² | §6.2's own trigger analysis |
| `REAL_SLOWDOWN_A_LAT_MS2` | 2.5 m/s² | §3.9 item 6 |
| `min_speed_ms` | 5.0 m/s | `tools/curvedb_telemetry_check.py`'s existing floor |
| `ramp_highway_classes` | the `*Link` members | `cereal/custom.capnp` `HighwayClass` — **camelCase** |
| `hard_ceiling_a_lat_ms2` (Lightning) | 4.5 m/s² | `LIGHTNING-STEERING-LIMITS.md`, measured at Crown Hill |

### PROVISIONAL — pick a value or a way to choose one

| name | value | why this value, and what is known about it |
|---|---|---|
| `site_radius_m` | **40 m** | Stands in for the OSM way/node ID §6.3 says mapd does not publish. **Measured cost:** at 300 m the only actions produced are garbage (§1). Nothing measured supports 40 specifically. |
| `heading_tol_deg` | **35°** | §6.3 killed 45° *buckets* (D4) but named no replacement. **Measured cost:** 22.2 % of sites have an approach-bearing spread wider than this across 500→150 m. |
| `approach_bearing_ref_m` | **300 m** | The midpoint of ICBM's 150–500 m decision range, and nothing more. §6.3 does not specify it. **§3.3 says this is not a free parameter.** |
| `PHANTOM_A_LAT_MS2` | **1.5 m/s²** | The design defines "real" and not "phantom". Everything between this and 2.5 is reported as grey. |
| `PASSAGE_MAX_M` | 80 m | How close the truck must have come for a pass to count. §6.3 notes mapd's point sits 56–125 m from the bend. |
| `EPISODE_LOOKAHEAD_M` | 500 m | ICBM's own `MAP_SOURCE_HORIZON_M`. |
| `EPISODE_GAP_S` | 3.0 s | Tolerance for a missing tick inside an episode. |
| `MIN_REDUCTION_MS` | 1.0 m/s | Below this a "slowdown" is rounding on the SET button. |
| `REF_LOOKBACK_S` | 20 s | How far back to find the pre-curve set speed when `icbmC` is absent. |
| `CLOCK_SKEW_MAX_S` | 36 h | Dead-RTC rejection, against each file's own median. |
| `DRIVE_GAP_S` | 300 s | Longer than a traffic light, shorter than a charging stop. |
| `MAX_TICK_DT_S` | 5 s | Odometer clamp across a missing second. |
| `PASSAGE_SCAN_M` | 1500 m | Bounds the passage search; 3× the far horizon. |
| `APPROACH_TOL_M` | 60 m | Tolerance on where the approach bearing is sampled. |
| `BEARING_BASELINE_M` | 50 m | Baseline for a track-derived bearing. |
| `K_MIN_USABLE` | 1e-4 1/m | R > 10 km. Rejects a degenerate row — **never** used to declare a road straight. **Open (Fable):** it also drops the 473 *straightest* passes, which §10 of the design says are the prime phantom refuters. Clamping instead (`k = max(k, K_MIN_USABLE)`, tagged) is the safe direction. A design call. |
| `ODO_GPS_RATIO_BAND` | 0.8–1.25 | Outside this band the drive is **dropped**. |
| `ACT_EPS_MS` | 0.1 m/s | Below this a raised target is arithmetic noise, not an action. |
| `EPISODE_CAND_SCAN_TICKS` | 5 | How many ticks from the decision to look in for ICBM's own candidate. |
| `_PoseSign.MIN_WITNESSES` | 20 | Minimum witnesses before a sign verdict is stated rather than "no witness". |
| `CELL_DEG` (recurrence) | 0.002° | Spatial bucket; the tool refuses a `site_radius_m` its 3×3 neighbourhood cannot cover. |
| `0.6` (false-cancel listing) | — | Below this ratio of measuring speed to counterfactual speed, a false cancel is flagged as a junction rather than a through-curve. |

---

## 7. Assumptions

1. **`icbmT`/`icbmC`/`vSet`/`spdLim`/`mapV` are m/s**, curvature 1/m, `strAng` degrees.
   Cross-checked: the odometer/GPS ratio is 1.000 across 139 drives, independently confirming
   `vEgo`; `icbmT` values (19.5 m/s ≈ 43.6 mph) match `DRIVE_REPORT.md`'s hand-analysed events.
2. **An ICBM episode is a contiguous run of ticks carrying a published `icbmT`**, restore tail
   excluded from the depth. There is no episode ID in `ces_events`.
3. **The episode's reference speed is `icbmC`** where present, else the max `vSet` in the 20 s
   *before* the decision. Using `vSet` during the episode would be circular: ICBM taps the SET
   button down, so mid-episode `vSet` *is* the slowdown.
4. **A row measured by one car may inform another.** The road table is car-independent; the
   envelope is keyed on the car *driving*. §8.2 implies this but does not state it, and it is what
   makes the Tesla's 2,931 observations usable at all (see I8).
5. **A drive is a per-car run with gaps < 300 s**, identified by `(car, first tick epoch)`.
   Rotated generations are stitched by timestamp, not by filename.
6. **Leave-one-out excludes the episode's date AND its drive.** §3.9-6 asks for
   leave-one-*date*-out, but an observation's date is the PT date of the *passage* while an
   episode's is the PT date of the *decision*; a drive crossing PT midnight between them would put
   the episode's own pass back into the database that judges it. Zero such episodes exist in this
   corpus — which is exactly why it must not be load-bearing.
7. **One site per drive** (candidates deduplicated at the matcher's own radius), and `n_passes`
   counts **drives**, not observations, so a single drive's UP and DOWN cannot satisfy half of D6.
8. `highway_class` is normalised with `str()` **once** and compared as a string everywhere — see
   the capnp `str()` enum trap.
9. **LODO includes future dates.** For cross-validating the idea that is standard; for the funnel
   numbers it is optimistic relative to what the car would have had at the time.

---

## 8. Honest read: does §7 suggest the idea holds up?

**Unproven. The premise survives; the gate is out of reach on this corpus.**

The design's *logic* survived contact with the data — nothing here contradicts curvature-not-speed,
road-not-policy, never-above-posted, or the cancel formula, and the store implements all of them
without needing to bend one. Everything testable mechanically works.

The premise it rests on — **that a curve worth refuting gets driven repeatedly** — survives, but
only after an adversarial check reversed the first reading. Through the pipeline it looked like 7
of 170 (4 %); measured from raw GPS it is 33 of 170 (19 %), and **26 of those 33 revisits are
discarded by our own admissibility rules, 19 of them by `drv`.** The truck repeats these roads
about five times more often than the database can currently see, and the gap is mostly a rule we
chose.

**But row existence is not the gate.** D6 needs ≥2 *other* dates under leave-one-out, and only
**15 of 170** sites have that. So even a perfect pipeline tops out around 15 adjudicable actions on
this corpus — against §3.9 item 5's ask for N ≥ 60. **No dq rule makes this corpus decide §7.** Six
to eight weeks of corridor driving is genuinely required; it is just no longer the *first* thing to
fix.

The one thing this corpus cannot speak to at all is the number §7 actually asks for — **the
false-cancel rate** — because that needs actions and there were none. Nothing here is evidence that
cancelling is safe. The single data point pointing the other way is that the only parameterisation
which produced any action produced 2 actions and 2 false cancels (§1) — an artifact of an over-wide
radius, but not a reassuring one.

**Recommendation: neither a green light nor a red light. In this order:**

1. **Decide the `drv` question (§3.2).** It costs 19 of the 33 genuine revisits — more than the
   driving pattern does. Answerable today with `ingest.py --dq-flags` and `recurrence.py`, no new
   driving required.
2. **Pin `approach_bearing_ref_m` (§3.3).** 22 % of sites move further than the matching tolerance
   across ICBM's own decision range, so ingest and the car can disagree about direction.
3. **Then** accumulate the corridor driving §3.9 item 3 asks for, with the pipeline no longer
   discarding four-fifths of the revisits it gets — and re-run this replay before building
   anything.

---

## 9. Review round 2026-09-17 (Fable: SHIP WITH CHANGES)

Fixed in this branch:

| # | finding | fix |
|---|---|---|
| 1 | the recurrence claim was produced by an uncommitted scratch script (D12 again) | `recurrence.py` is committed and tested, and reports the ≥2-other-dates split the gate actually needs |
| 1 | "15 by `drv` alone" overstated the author's own table | now "19 `drv`-involved"; the per-cause table is printed by the tool |
| 2 | README site-shuffle figures matched no run on disk | re-run and re-quoted from `replay_main.log` |
| 3 | the ramp gate compared the raw object while the unknown gate compared `str()` — fails **open** on a capnp enum | normalised once; three tests including a fake enum |
| 4 | leave-one-**date**-out leaks across PT midnight within one drive | key is now `(date, drive_id)`, `_assert_lodo` checks both |
| 5 | `k_ahead_max` had no speed floor — 36 of 170 verdicts taken below 5 m/s | same `min_speed_ms` floor as the extent |
| 6 | site-shuffle permuted globally, manufacturing multi-date rows | permutes **within** each date; alarm text corrected |
| 8 | the k-shuffle test never ran the shuffled DB | now asserts the **verdict** changes, and fails if no seed can change it |
| 9 | I1 claimed the measured reading was printed and it was not | `summarise` prints both readings |
| 10 | `dq_state="clean"` conflated the 100 Hz roll-up with a 1 Hz sample | `dq_src` records which |
| 11 | dedup key `(car, t)` discarded `adopt` records sharing a tick's timestamp | key includes `ev` |
| 12 | a drive failing the odometer/GPS band still contributed rows | dropped, loudly, with a count |
| 13 | the episode denominator (433 runs → 170) was unstated | §1 |
| 14 | the Tesla commanded-only admission was presented as a finding, not a deviation | I8 |
| 15 | zero-count stats were absent rather than printed as 0 | `TALLY_KEYS` pre-seeded |
| 17 | `n_passes` counted observations, not passes | counts distinct drives |

**Deferred for the owner, not fixed here** (recorded so they are not re-discovered):

* **16 — dead code.** `to_snapshot`/`from_snapshot`/`load_snapshot_or_empty`/`tighten`/
  `with_params` are used only by tests. They implement §8.1's "fail SAFE to no database, but say
  so", which is a Rule 2 exemplar, but §8.1 belongs to commit C3 of a gated phase. Delete or keep
  is a judgement call and the owner should make it. (`load_snapshot_or_empty` also returns `None`,
  not an empty DB — the name should change either way.)
* **20 — `K_MIN_USABLE` drops the 473 straightest passes**, which §10 of the design says matter
  most. Clamping instead of dropping is the safe direction and is a design call.
* **7 — `--self-match` is near-tautological** (episode and observation share the same `Passage`),
  so its docstring's "the matcher works" overstates. Noted in §1's table rather than reworded in
  code, because a non-tautological version needs an independent site source that does not exist
  before 2026-09-17.
* **18 — LODO includes future dates.** Standard for cross-validation, optimistic for the funnel.
* **19 — `track` vs `logged` sites are geometrically different things** (lane position vs mapd
  node); `site_radius_m` must absorb that offset too.
* **21 — `_PoseSign`** arguably belongs in `tools/curvedb_telemetry_check.py`.
* **22 — small Rule-2 items**: `no_fix` conflates four causes; `fields_alive` reports `dq=False` as
  "always null/zero" though `False` is a live clean reading; the clock filter counts drops without
  printing their dates; `"steer"` breadcrumb records may lack `strPrs` and so enter an extent as
  hands-off ticks.

---

## 10. Reproducing

Working files from the 2026-09-17 run are in `_scratch/curvedb/out/` (not committed):
`ingest_all.log`, `obs_all.jsonl` (4,719), `eps_all.jsonl` (170), `replay_main.log`,
`replay_r{80,150,300}.log`, `replay_highway.log`, `replay_anyclass.log`, `recurrence.txt`,
`calib.txt`, `mutate_full.log`.

```bash
# tests (the worktree needs opendbc_repo/opendbc symlinked and a borrowed common/params_pyx.so)
PYTHONPATH=$PWD:$PWD/opendbc_repo ../pnw-pilot/.venv/bin/python -m pytest tools/curvedb/tests -q
# mutation harness -- every mutant compile-checked and anchor-checked before it counts
python3 /home/dp/gh/comma/_scratch/curvedb/mutate.py
```
