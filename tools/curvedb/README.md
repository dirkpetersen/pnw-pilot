# curvedb — the OFFLINE half of CURVEDB2PNW.md Phase 2

**Status: NOT DEPLOYED, NO AUTHORITY, NOT PUSHED.** Built 2026-09-17 on branch `curvedb2pnw` off
`3devpnw` @ `7c40d8003b`; reviewed by Fable the same night (SHIP WITH CHANGES), **re-reviewed
2026-09-19** (section 10 -- one of the findings was a rule that could never fire and a README
paragraph that said the opposite), and this is the post-review state. Everything lives under `tools/`; nothing in `selfdrive/`, `system/`, `cereal/`
or any submodule is touched, and nothing the car imports imports any of it. Phase 2 remains gated
on §3.9 and this branch does not change that.

| file | what it is |
|---|---|
| `store.py` | **C1** (§8.3): the row store, the matcher, the update rules, the authority gate, the cancel formula. Zero openpilot imports, no I/O, no clock. **This is the module a future on-car consumer would import unchanged** — if the replay validated one implementation and the car ran another, the replay proved nothing. |
| `ingest.py` | ces_events corpora → observations + ICBM episodes, with a per-file capability report. |
| `replay.py` | **§7's go/no-go gate.** Leave-one-date-out, a stated N, and three adversarial self-checks. |
| `recurrence.py` | Does the truck actually re-drive the phantom roads? Measured from **raw GPS**, with the observation pipeline deliberately out of the loop — because measuring it *through* the pipeline gave the wrong answer. |
| `calib.py` | The two numbers this README quotes about its own PROVISIONAL constants: what a 1 Hz-built row costs against the 100 Hz peak, and how far the approach bearing moves across ICBM's decision range. **Committed because they used to come from an uncommitted scratch script** (§10). |
| `tests/` | 352 tests. **102/102 mutants killed**, 0 survived, 0 unbuilt (`_scratch/curvedb/mutate.py`). |

### v2 (2026-09-24, OFFLINE, NOT DEPLOYED, NO AUTHORITY): the road table from drive logs

Built per the owner's 2026-09-23 decisions (every pass counts; median across >= 2 dates; Tesla never sole
evidence; I-5 corridor + Seattle first). Method, validation, replay, gate and the on-car shadow proposal:
**`docs/CURVEDB-V2-BUILD.md`** in the workbench (`~/gh/comma/docs/`). v1's files above are unchanged.

| file | what it is |
|---|---|
| `v2_extract.py` | qlog/rlog -> per-segment streams (livePose yaw, carState, carControl, GPS, mapdOut incl. `wayRef`). I/O only |
| `roadtable.py` | the pure core: fusion, 1 s signed-mean curvature, 25 m resampling, anchors + **branch key**, R8 over the extent, `row_verdict` (the owner's rules). Imports only `store.py` |
| `v2_io.py` | route loading, PT dates, ces_events loading |
| `v2_validate.py` | step 2: yaw-derived k vs `kPeak`/`kPoseP`/CAN `yawRate`, clock lag measured per route |
| `v2_build.py` | step 3: the table (`table.json.gz`) and the exportable rows (`rows.json`) |
| `v2_replay.py` | step 4: LODO replay of ICBM episodes, adds scan, and the ICBM-independent **row accuracy** |
| `tests/test_v2.py` | the pure functions |

### v2 LIVE (curvedblive2pnw, 2026-09-24): the table sets ICBM's curve target on the Lightning

Owner decision 2026-09-24: LIVE, both directions, the N >= 60 shadow gate overridden. The car side is
`selfdrive/controls/lib/ces_pnw/curvedb_live.py`; how it works, the rules and the kill switch:
**`docs/CURVEDB-V2-LIVE.md`** in the workbench. **The rows file is PRIVATE (the owner's driven positions): it is
never committed here.** It lives at `/data/pnw/curvedb_v2/` on the device and in the private workdir repo.

| file | what it is |
|---|---|
| `v2_live_export.py` | `table.json.gz` -> `curvedb_v2_rows.json.zst` (zstd JSON) + `manifest.json` (SHA-256 of the .zst) |
| `v2_live_fixture.py` | the PRIVATE leave-one-date-out replay fixture `ces_pnw/tests/test_curvedblive2pnw_replay.py` runs on |
| `v2_live_report.py` | per drive from `ces_events`: what the live DB did (decisions, raises, lowers, mph, sites, why-not) |
| `tests/test_v2_live_report.py` | the report tool |

```bash
PYTHONPATH=. python3 tools/curvedb/ingest.py drives/**/ces_events*.jsonl \
    --out-observations obs.jsonl --out-episodes eps.jsonl
PYTHONPATH=. python3 tools/curvedb/replay.py --observations obs.jsonl --episodes eps.jsonl \
    --dq any --self-match --control k-shuffle --control site-shuffle
PYTHONPATH=. python3 tools/curvedb/recurrence.py drives/**/ces_events*.jsonl \
    --episodes eps.jsonl --observations obs.jsonl
PYTHONPATH=. python3 tools/curvedb/calib.py drives/**/ces_events*.jsonl
```

---

## 1. THE RESULT: **NO RESULT**, and that is the honest answer

Run over every `ces_events` corpus under `drives/` — 84 files, **400,583 unique ticks** after
deduplication, 139 drives (1 dropped, §4), 16 dates, both cars, **4,846 observations** (4,719 UP +
**127 DOWN**, the latter newly reachable — §10 finding 1):

```
  episodes replayed                       170  (146 distinct episode-sites)
  ... matched a row                       6  (3.5%)
  ... verdict no_row                      164 (96.5%)
  ... verdict no_authority                6   (3.5%)

  THE GATE (§7): episodes the database would have cancelled or reduced = 0  (0 distinct sites)
    of those, adjudicable (k_truth present)   0
    REAL SLOWDOWNS WRONGLY CANCELLED          0
    bound on the false-cancel rate            NONE -- 0-of-0 is not evidence.
```

**The zero in "0 real slowdowns wrongly cancelled" is worth nothing**, because the database never
acted. §7 has not been passed and has not been failed; it has not been *evaluated*. Reporting the
zero without the denominator would be the `getfattr` failure again — a uniform result read as a
finding instead of as an un-exercised method.

**Fixing the DOWN rule did not change that.** §6.2's DOWN observations were structurally
unreachable until 2026-09-19 (§3.4). With them the database has 4,023 rows instead of 3,899 and
matches **6** episodes instead of 4 — and still acts on **zero**. All 6 are refused authority by
D6 (one pass). This is the thing to check first if any of these numbers are re-quoted: the
headline is unchanged by the fix, not preserved by omission.

**170 episodes is 146 distinct episode-sites**, and the difference is not decoration: **24 of the
170 are ICBM re-firing at a junction it had already slowed for, later in the same drive** — one
road event logged twice (`site_group`, tagged by `ingest.py`, printed by every tool). The two
"false cancels" the 300 m run produces below are *one* junction, lines 4258 and 4294 of one file,
one `Passage`. Every count here now prints both denominators, and the rule-of-three bound is taken
on distinct sites, because a re-fire is not an independent trial.

**The 170 has its own denominator too, and it is a selection effect worth seeing.** ICBM produced
**433** target runs in this corpus: 50 were restore-only (an increase, not a slowdown), **213 had
no measurable passage within 500 m ahead**, and 170 survived. Half the population is excluded
before the replay starts, for a reason (no candidate, or the drive ended) that is not random.

### The three adversarial checks

| check | result | reading |
|---|---|---|
| **self-match** (no LODO — circular by construction) | **40** rows vs 6 under LODO; **34 of the 40 rows contain a pass from the episode's own drive** | This shows `build()` and `match()` are **SELF-CONSISTENT** — nothing more. It asks whether a row anchored at x contains x, because the episode's site/bearing and the observation's come from the *same* `Passage` object; the 34 is that tautology, measured. It would *not* detect an ingest-vs-lookup convention mismatch, so it is **not** evidence that "the keying works". What it does rule out: a matcher so broken that LODO's 3.5 % is an artefact of the key rather than of the corpus. |
| **`k-shuffle`** (right places, wrong curvatures) | identical funnel (6 matched, 0 actions) | **Uninformative, and now says so in those words** rather than firing an alarm. With zero actions there is no false-cancel count for scrambling to raise; this control can only speak once the DB acts. |
| **`site-shuffle`** (each episode looked up at **another episode's site**) | **29 matched vs 6** — the corrupted lookup matches *five times more often*; **23 of the 29 landed on a site whose own episode cannot match there under its own LODO key**, and the excess is 23 | An episode's **own** site is the hardest place for it to find a row. Not a broken matcher and not row density: leave-one-date-out removes the episode's own date, and the passes at its own site are overwhelmingly *from* that date, while another episode's site is built from dates LODO does not touch. **It is §3.1's single-visit-site blocker, measured from a second direction.** That explanation is *measured* (`confound_matches`), not asserted — and `confound ≥ excess` is an identity, so a run where it fails is a plumbing bug and the tool says so in those words. |

> **The site-shuffle was rebuilt again on 2026-09-19, because the previous version could not
> fail.** It permuted the *observations'* sites within each date. That preserves each date's
> multiset of positions exactly; LODO removes whole dates; so the surviving multiset — and
> therefore `matched` — was invariant **by construction**. Its alarm ("the matcher is not selecting
> anything") fired on every corpus no matter what, and this README's previous conclusion, that the
> 4 matches were row-density artefacts, did not follow from it. An alarm that always fires is worse
> than no alarm: it teaches the reader to skip it. (The version before *that* permuted globally,
> manufacturing multi-date rows the corpus does not have. Third time.)
>
> The replacement can land in any of four states — fewer matches (the lookup selects), equal
> (density), more (the LODO confound above), or "the real lookup matched nothing, so this is
> uninformative" — and `main` prints which. Its self-control is printed too: **3 of 170 episodes
> landed back within 40 m of their own site** (permutation fixed points and re-fires) and are
> therefore uncontrolled.

### Loosening the match until it *can* act makes it worse, not better

At `--site-radius-m 300 --heading-tol-deg 45` the database finally acts on 2 episodes — and
**both are false cancels, a 100 % failure rate**. They are, however, **ONE junction**: the same
site firing twice inside one drive, lines 4258 and 4294 of one file. The `!` is the tool marking
the re-fire, and "2 (1 distinct episode-site)" is how it is now counted:

```
 2026-08-26 k=0.06654@5m/s (sample1hz_cmd) icbm 2.6 -> db 11.2 m/s  a_cf=8.35  ...:4258
!2026-08-26 k=0.06654@5m/s (sample1hz_cmd) icbm 2.6 -> db 11.2 m/s  a_cf=8.35  ...:4294
  of these, 2 had their curvature measured below 60 % of the counterfactual speed
```

`k = 0.067` at **5 m/s** is a 15 m radius — a junction, not a road curve. So these are not credible
false cancels either; they are the matcher, at a 300 m radius, keying onto a different piece of
road. The honest summary is **"the only parameterisation that produces any action produces only
garbage"**, which is a stronger argument against loosening the match than any of the above.

### Stratified to the roads the phantoms are actually on

`--highway-only` (motorway/trunk) keeps **25 of 170** episodes (24 distinct sites), and on those
the LODO match rate is **4/25 = 16 %** rather than 3.5 % (self-match 12/25 = 48 %). Still zero
actions — every match is a single pass — but it says the corridor sites are considerably less
sparse than the corpus average, which is the population §1 of the design is about.

---

## 2. The retrospective corpora cannot certify anything, and here is exactly why

The Phase-1 fields (`kPeak` `kPeakN` `kPose` `kPoseP` `achLatPose` `dq` `dqWhy` `strTq` `mapLat`
`mapLon` `mapCandD`) exist only from 2026-09-17. Everything before is a reconstruction, and
`ingest.py` reports per file which vocabulary it had.

| provenance of the 4,846 observations | count | consequence |
|---|---|---|
| `kPeak100` — the real 100 Hz peak | **166** | the only rows built the way §3.3 specifies |
| `sample1hz_cmd` — commanded only (every Tesla pass) | 3,196 | a *model* estimate, not a CAN measurement (see §5 I8) |
| `sample1hz_cmd_actl` — max of both, 1 Hz | 1,338 | §3.3's estimator, at 1/100th the rate |
| `slKCmd_at_override` — §6.2's DOWN | **127** | the driver's own "too fast", newly reachable (§3.4) |
| `sample1hz_actl` — achieved only | 19 | D2: bounded by steering authority |
| `site_src="logged"` (`mapLat`/`mapLon`) | **167** | keyed on the candidate, as §6.3 requires |
| `site_src="track"` (reconstructed from the truck's later position) | 4,679 | keyed on a reconstruction of it |
| `dq_src="rollup100"` (§3.5's 100 Hz OR) | **167** | a `clean` that means what §3.5 says |
| `dq_src="sampled1hz"` (instantaneous flags) | 4,679 | a `clean` that may have been *between* events |

(Across all sites *considered*, not just those that became rows: 307 logged, 27,564 reconstructed.)
By car and kind: Tesla 2,931 UP + 31 DOWN, Lightning 1,788 UP + 96 DOWN.

**Measured, not assumed — how bad is the 1 Hz estimator?** `calib.py`, over the 259 measurement
extents where both a 100 Hz peak and a 1 Hz sample exist (only the 2026-09-17 corpora carry
`kPeak`): `max(kPeak) / max(1 Hz sample)` is **p50 1.101, p90 1.506, max 2.875**. Since
`v = sqrt(a/k)`, a 1 Hz-built row permits a speed **5 % too high at the median, 23 % at p90, 70 %
at worst.** Per tick (n=8,139) the sample under-reads by more than 20 % on **62 %** of ticks. Every
1 Hz row is therefore biased in the direction that **under-brakes**.

> These figures used to come from an uncommitted scratch script (§10 finding 4). `calib.py`
> reproduces the two that were quoted — p50 1.10, p90 1.51, max 2.875 — to three significant
> figures. The **per-tick** figures it produces differ (n=8,139 / 62 % against the scratch's
> n=6,451 / 74 %); the scratch's filter is not recoverable, so the committed script's numbers are
> the ones quoted here and the old per-tick numbers should be treated as unreproducible.

---

## 3. What actually blocks the gate

### 3.1 The sites ARE re-driven — the admissibility pipeline throws the revisits away

This reversed under an adversarial check, so both numbers are given, and the check is now a
committed tool (`recurrence.py`) rather than a scratch script.

Via the observation pipeline, only 6 of 170 episodes have a row at their site from another date,
which reads as "the truck does not repeat these roads". **It does.** Measured from raw GPS ticks,
bypassing observations, passages and disqualifiers entirely:

| | episodes (of 170) | distinct sites (of 146) |
|---|---|---|
| the truck came within 40 m of the site at all | **169** | 145 |
| ... on a **different date** | **39** | 36 |
| ... on a different date *and* a compatible approach bearing | **33** | 31 |
| ... on **≥2 other dates** — what D6 needs under leave-one-out | **15** | 13 |
| ... and a usable row actually exists there | **6** | — |

**27 of the 33 genuine revisits are lost inside our own pipeline.** Attributed by the
disqualifier's own named cause:

| why the revisit produced no row | count |
|---|---|
| **disqualified, `drv` involved** (`drv` 12, `sat,drv` 2, `drv,blnk` 2, `sat,drv,lc` 1) | **17** |
| disqualified, `sat`/`blnk` only | 5 |
| no map candidate / no approach bearing / never became a site | 5 |
| *(a row exists)* | 6 |

> **"A row exists" was 7 and is now 6, and the change is two separate corrections** (§10 findings
> 8 and 1). `has_row` was testing position only while the replay's matcher also tests the approach
> bearing, so it counted the opposite carriageway: keyed identically, the same corpus reads **4**,
> which is exactly what the replay's matcher saw. The DOWN fix then adds 2 real rows, taking both
> tools to **6**. They now agree at every stage, which is the point — two numbers for one question
> is how a discrepancy nobody wrote down becomes a finding.

Two conclusions, and they must be kept apart:

* **For row EXISTENCE, the driving pattern is not the primary blocker — §6.1's disqualifier is.**
  17 of 33 revisits lost to `drv`-involved causes alone.
* **For the GATE, that is not enough.** Authority needs ≥2 passes on ≥2 dates, and under
  leave-one-out that means ≥2 *other* dates — true for only **15 of 170** sites
  (`{1 other date: 18, 2: 4, 4: 7, 5: 2, 12: 2}`) — 13 of 146 distinct sites. So even with a
  perfect pipeline this corpus tops out around 15 adjudicable actions, and **§3.9 item 5's N ≥ 60
  is unreachable here under any dq rule.** Fixing the `drv` question raises the ceiling; it does
  not by itself reach the bar. **The DOWN fix did not move this ceiling** (still 15/170): DOWN
  observations land at the same sparse sites.

Secondary, still true: 4,023 rows exist and **765 have ≥2 passes on ≥2 dates** (~19 %), but those
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

**Driver steering, not saturation, is what disqualifies these passes.** (This is the UP rule.
Since 2026-09-19 §6.2's DOWN is judged on the same roll-up *without* `drv` — see §3.4 — so `drv`
costs the UP half only.) Two consequences the design does not discuss:

1. A pass is only admissible where openpilot was steering, and an extent is disqualified if `drv`
   fired **anywhere** in it — which is a far stronger rule than the per-tick rate suggests.
   Measured over ticks carrying the field above 5 m/s, `strPrs` is true on **7.7 %** of Tesla ticks
   and 16.5 % of Lightning ticks. (This section used to say "on the Tesla `strPrs` is true
   continuously". It is not; the 7.7 % is the measurement. What is true is that 136 of the 147
   disqualified episode passes carry `drv`.)
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
what distance**. ICBM decides anywhere from 150 m to its 500 m far-source horizon; ingest samples
at `approach_bearing_ref_m` = 300 m. `calib.py`, over the **8,244 measured passages** where all
three reference points exist:

| | p50 | p90 | max | over the 35° tolerance |
|---|---|---|---|---|
| **vs the 300 m reference ingest samples at** — the disagreement that loses a row | 10.6° | 43.4° | 179.6° | **13.6 %** |
| pairwise max over 500/300/150 — an **upper bound**, not the same number | 13.9° | 54.0° | 179.7° | 18.8 % |

**On one site in seven, ingest and the car can disagree about which direction "this way" is and
the row is simply never found.** `approach_bearing_ref_m` is a real parameter with a real cost.

> **The "22.2 % over 2,074 site passages" this section used to quote is NOT reproducible** (§10
> finding 4): it came from an uncommitted scratch script whose denominator of 2,074 matches nothing
> in the pipeline — there are 8,279 passages and 27,871 candidate sites. `calib.py` measures it
> through `measure_passage` itself, and applies the same odometer/GPS drive gate, so a passage here
> is a passage there. **The conclusion is unchanged and the number is not:** anything quoting
> 22.2 % (including `docs/CURVEDB2PNW.md` §12 and the 09-18 changelog, which this branch does not
> touch) should be corrected to **13.6 %**.

### 3.4 §6.2's DOWN rule was UNREACHABLE — this section used to say the opposite

**What this section said until 2026-09-19 was the reverse of the truth, and it is the worst defect
this branch has had.** It read: *"it is not a plumbing failure; the interventions are simply below
the trigger."* They were not. **`observations_for_drive` dropped every disqualified passage before
the DOWN loop ran, and a DOWN is BY DEFINITION a `strPrs` tick, which sets `DQ_DRV`, which makes
the passage disqualified.** The rule could not fire on any input whatsoever. Its zero was a
structural property of the code, and this README confidently explained it as a property of the
road — exactly what Rule 2 exists to prevent.

The pre-seeded counters were real and did what they were built for; they simply could not detect
this, because `down_dropped_no_kcmd 0` is equally consistent with "no overrides qualified" and
"the loop was never entered". The test suite could not detect it either: **every DOWN test relaxed
`drv` for itself** (`--dq-flags lc,blnk`), so the tests encoded the bug. They now run under the
design's own disqualifier set, and one of them exists only to assert reachability.

**Fixed:** UP is still judged on the full §6.1 roll-up (a driver's hands corrupt a *hands-off*
curvature measurement), while DOWN is judged on the same roll-up with `drv` — its own precondition
— cleared, and every other cause (`sat`, `blnk`, `lc`, unattributed) still binding.

| | before | after |
|---|---|---|
| `obs_down` | **0** | **127** |
| `down_no_lateral_accel_witness` | 0 | **2** |
| `down_dropped_dq_not_drv` (new counter) | — | 1,342 (`sat` 976, `blnk` 432, `lc` 113) |
| `down_dropped_no_kcmd` | 0 | 0 |

**What the 127 are, stated rather than rounded to a reassuring comparison.** They come from **75
distinct override ticks**: extents overlap, so one override lands inside several neighbouring
sites and becomes one observation at each (33 ticks feed 2–4 sites). **88 of the 127 are a single
Lightning drive on 2026-09-13**; 96 are the Lightning and 31 the Tesla. Median `k` is 0.0146
(R ≈ 69 m) and **46 of 127 are tighter than R = 50 m** — city corners, not corridor bends. The two
`k_down = 0.0807, n=2, dates=2` rows in the row table are *one* Tesla tick at R ≈ 12 m attributed
to two sites 60 m apart.

(§6.2's own "~113 of 2,986 overrides" is **not** a like-for-like check on this: it counted
Lightning ticks above 27 mph, these are site-attributed observations from both cars above 5 m/s.
The numbers being close is a coincidence, not corroboration.)

**They change the funnel (4,023 rows, 6 matched) and they do not change the result: still zero
actions** (§1). Nothing improper becomes a row — `n_passes` counts distinct drives, so one drive's
UP and DOWN cannot together satisfy D6, and a DOWN can only ever raise `k_eff`, which only ever
lowers the derived speed.

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
  §3.3 itself argues commanded is the *better* half on curves. Result: **2,962 of the 4,846
  observations are Tesla passes** — more than the Lightning's 1,884. This is a **deviation from
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
| `heading_tol_deg` | **35°** | §6.3 killed 45° *buckets* (D4) but named no replacement. **Measured cost (`calib.py`):** on 13.6 % of measured passages the bearing at 500 or 150 m differs from the 300 m reference by more than this (18.8 % by the pairwise-max upper bound). |
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
| `K_MIN_USABLE` | 1e-4 1/m | R > 10 km. Rejects a degenerate row — **never** used to declare a road straight. **Open (Fable):** it also drops the 473 *straightest* passes, which §10 of `CURVEDB2PNW.md` says are the prime phantom refuters. Clamping instead (`k = max(k, K_MIN_USABLE)`, tagged) is the safe direction. A design call. |
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
only after an adversarial check reversed the first reading. Through the pipeline it looked like 6
of 170 (4 %); measured from raw GPS it is 33 of 170 (19 %), and **27 of those 33 revisits are
discarded by our own admissibility rules, 17 of them by `drv`.** The truck repeats these roads
about five times more often than the database can currently see, and the gap is mostly a rule we
chose. The site-shuffle control now says the same thing from the other side: an episode's own site
is the *hardest* place for it to find a row, because its rows are its own date's (§1).

**But row existence is not the gate.** D6 needs ≥2 *other* dates under leave-one-out, and only
**15 of 170** sites have that. So even a perfect pipeline tops out around 15 adjudicable actions on
this corpus — against §3.9 item 5's ask for N ≥ 60. **No dq rule makes this corpus decide §7.** Six
to eight weeks of corridor driving is genuinely required; it is just no longer the *first* thing to
fix.

The one thing this corpus cannot speak to at all is the number §7 actually asks for — **the
false-cancel rate** — because that needs actions and there were none. Nothing here is evidence that
cancelling is safe. The single data point pointing the other way is that the only parameterisation
which produced any action produced 2 actions and 2 false cancels (§1) — one junction twice over,
an artifact of an over-wide radius, but not a reassuring one.

**Recommendation: neither a green light nor a red light. In this order:**

1. **Decide the `drv` question (§3.2).** It costs 17 of the 33 genuine revisits — more than the
   driving pattern does. Answerable today with `ingest.py --dq-flags` and `recurrence.py`, no new
   driving required. (§3.4 has now answered the DOWN half of it: `drv` cannot disqualify the rule
   it defines. The UP half is still open and still the owner's call.)
2. **Pin `approach_bearing_ref_m` (§3.3).** On 13.6 % of measured passages the bearing elsewhere
   in ICBM's decision range differs from the 300 m reference by more than the matching tolerance,
   so ingest and the car can disagree about direction. `calib.py` is the tool.
3. **Then** accumulate the corridor driving §3.9 item 3 asks for, with the pipeline no longer
   discarding four-fifths of the revisits it gets — and re-run this replay before building
   anything.

---

## 9. Review round 2026-09-17 (Fable: SHIP WITH CHANGES)

Fixed in this branch:

| # | finding | fix |
|---|---|---|
| 1 | the recurrence claim was produced by an uncommitted scratch script (D12 again) | `recurrence.py` is committed and tested, and reports the ≥2-other-dates split the gate actually needs |
| 1 | "15 by `drv` alone" overstated the author's own table | now "19 `drv`-involved" (17 after §10's finding 8 keyed `has_row` the same way the matcher does — both numbers are correct, for the before and after corpora); the per-cause table is printed by the tool |
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

**Deferred at that round, and their fate at the 2026-09-19 round (§10):**

* **16 — dead code.** `to_snapshot`/`from_snapshot`/`load_snapshot_or_empty`/`CurveRow.to_json`/
  `with_params` were reachable only from their own tests. **DELETED 2026-09-19** — §8.1's on-car
  half is not being built, so it was code carrying review cost for a consumer that does not exist.
  `git show a46b90f2b5:tools/curvedb/store.py` has it. `tighten` stays: it is `dataclasses.replace`
  under a greppable name and three test modules use it.
* **7 — `--self-match` is near-tautological** (episode and observation share the same `Passage`).
  **REWORDED 2026-09-19.** Both this README and the tool used to argue from it that "the keying
  works"; it shows only that `build()` and `match()` are self-consistent. The tool now measures and
  prints the tautology itself — **34 of the 40 self-matched rows contain a pass from the episode's
  own drive** — and says in those words what that can and cannot support.
* **20 — `K_MIN_USABLE` drops the 473 straightest passes**, which §10 of `CURVEDB2PNW.md` says
  matter most. Clamping instead of dropping is the safe direction and is a design call. **Still open**
  (the 2026-09-19 review verified independently that removing the floor changes the replay verdict
  not at all).
* **18 — LODO includes future dates.** Standard for cross-validation, optimistic for the funnel.
  **Still open**, deliberately.
* **19 — `track` vs `logged` sites are geometrically different things** (lane position vs mapd
  node); `site_radius_m` must absorb that offset too. **Still open**, deliberately.
* **21 — `_PoseSign`** arguably belongs in `tools/curvedb_telemetry_check.py`. **Still open** —
  a relocation with no behaviour attached.
* **22 — small Rule-2 items.** `fields_alive` reporting `dq=False` as "always null/zero" is
  **FIXED** (§10 finding 5). The rest are still open: `no_fix` conflates four causes; the clock
  filter counts drops without printing their dates; `"steer"` breadcrumb records may lack `strPrs`
  and so enter an extent as hands-off ticks.

---

## 10. Review round 2026-09-19 (Fable, second pass)

The headline — over 170 ICBM episodes the DB acted on ZERO under leave-one-date-out — was
**independently verified and holds**: the recurrence ceiling reproduces, LODO is leak-free, the
match radius is not the loss mechanism, and removing `K_MIN_USABLE` changes nothing. What sat
underneath it did not survive as well.

| # | finding | what was done |
|---|---|---|
| 1 | **§6.2's DOWN rule could never fire**, and §3.4 said the opposite in as many words | the disqualifier that *defines* DOWN no longer disqualifies it; §3.4 rewritten. `obs_down` **0 → 127**. The tests relaxed `drv` for themselves and so encoded the bug — they now run under the design's own set |
| 2 | **the site-shuffle control could not fail** — permuting within a date leaves `matched` invariant under LODO, so its alarm fired by construction and §1's conclusion from it was unsupported | rebuilt to permute the **episodes'** sites; four outcomes, each with its own reading; §1 rewritten. It now reports something real (29 vs 6) |
| 3 | **N = 170 is inflated by 24 re-fires**, and the two 300 m "false cancels" are one junction | episodes carry `site_group`; every count prints both denominators; the false-cancel listing marks re-fires with `!`; the rule-of-three bound is taken on distinct sites |
| 4 | **uncommitted evidence (D12 again)** — the 1 Hz under-read and bearing-spread figures had no script behind them | `calib.py`, committed and tested. The per-extent figures reproduce (p50 1.10 / p90 1.51); **the 22.2 % bearing figure does not** and is superseded by 13.6 % (§3.3) |
| 5 | **`fields_alive` reported a legitimately-`False` field as dead** (`False == 0.0` in Python) | `_is_live_reading` decides bools before the zero test |
| 6 | **dead snapshot code** (Rule 4) | deleted, with the tests that were its only caller |
| 7 | **"keying works" overstated what self-match shows** | reworded in §1, §9 and the tool; the tautology is now measured and printed |
| 8 | **`recurrence.has_row` omitted the bearing check** the replay's matcher applies, which is why it said 7 where the replay saw 4 | keyed identically; both now report the same number at every stage (4 before the DOWN fix, 6 after) |

**A second Fable pass on the fixes themselves** found eight more, all applied: `calib.py` skipped
the odometer/GPS drive gate ingest applies (it now shares ingest's own `drive_odo_gps_ok`
and reports the dropped drive); §3.4's "127 vs §6.2's ~113" was not
like-for-like and is now stated as 75 distinct ticks with 88 on one date; §3.2's "on the Tesla
`strPrs` is true continuously" was wrong and is now the measured 7.7 %; the site-shuffle's
confound explanation is now **measured** (`confound_matches`) instead of asserted; a "12.1 %"
figure no committed script produces was deleted; the bearing spread is split into the number that
matters and its upper bound; `site_group` now travels with the site through the shuffle; and §9's
19-vs-§3.1's-17 is explained rather than left as two numbers for one table.

**Not material, skipped by instruction:** 18 (LODO includes future dates), 19 (`track` vs `logged`
geometry), 21 (`_PoseSign`'s home).

**What did NOT change: the result.** Fixing DOWN adds 127 observations and 124 rows, and the
database still acts on **zero** episodes under LODO. §7 remains un-evaluated.

---

## 11. Reproducing

Working files are in `_scratch/curvedb/` (not committed): the 2026-09-17 run in `out/`, and the
2026-09-19 re-run in `new/` (`ingest.log`, `obs.jsonl` (4,846), `eps.jsonl` (170),
`replay_main.log`, `replay_{r300,highway}.log`, `recurrence.txt`, `calib.txt`). `base/` holds the
pre-fix run the before/after numbers in §3.4 and §10 are taken from.

```bash
# tests (the worktree needs opendbc_repo/opendbc symlinked and a borrowed common/params_pyx.so)
PYTHONPATH=$PWD:$PWD/opendbc_repo ../pnw-pilot/.venv/bin/python -m pytest tools/curvedb -q
# mutation harness -- every mutant compile-checked and anchor-checked before it counts
python3 /home/dp/gh/comma/_scratch/curvedb/mutate.py
```
