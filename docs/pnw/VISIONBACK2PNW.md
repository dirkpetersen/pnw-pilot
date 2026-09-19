---
updated: 2026-09-19
status: current     # MEASUREMENT ONLY -- no feature, no car code, nothing deployed, nothing pushed
---

# VISIONBACK2PNW — can vision hand back a bounded N mph of a map slowdown?

> **⛔ NOTHING WAS BUILT.** This is an offline measurement answering one question the owner asked on
> 2026-09-19. No control-path file was touched. No branch was pushed. The verdict below is a
> recommendation, not a decision.

## The question

Today `icbm_curve_target()` takes the **minimum** of the map, vision and far-map candidates, so vision
can only ever ADD slowing. A full vision VETO was built (`icbm_map_sanity`) and deliberately left
unwired: on the weekend corpus it raised 86 ticks, **7 on real curves needing > 3.0 m/s², one at 5.3**.

The owner asked about a different shape — a **bounded give-back**:

> If vision had been allowed to give back **at most N mph** of a map-commanded slowdown between
> **50 m and 150 m** from the candidate, and only where vision is measurably trustworthy, what
> lateral acceleration would the truck ACTUALLY have pulled on every real curve in the corpus?

Decision rule, his: worst case comfortably under the Lightning's measured **~4.5 m/s²** hands-off
ceiling *and* a meaningful typical recovery ⇒ real feature. Worst case reaching **5+** ⇒ dead.

## Verdict

**Dead — for the same reason the veto was, and the measurement says so at N = 5, not only at N = 10.**

| N (mph) | engaged | median recovery | **max resulting a_lat** | ≥ 4.5 | ≥ 5.0 |
|---|---|---|---|---|---|
| 0 (shipped) | 0 / 88 | — | **3.81** | 0 | 0 |
| **2** | 38 / 88 | +2.00 mph | **4.10** | 0 | 0 |
| **5** | 40 / 88 | +3.05 mph | **4.70** | 2 | 0 |
| **10** | 45 / 88 | +3.33 mph | **5.80** | 4 | 1 |
| 15 | 46 / 88 | +4.69 mph | **7.01** | 8 | 3 |

*(88 ICBM curve episodes above 25 mph, Ford only, truth = max(`kPeak`, `|slKActl|`) measured at the
apex, scored at `max(measured apex speed, counterfactual target)`. Full method in §3.)*

**N = 10 is dead on the owner's own rule: 5.80 m/s² at one real bend.** N = 5 reaches 4.70 — above the
ceiling, with no margin. Only **N = 2** stays under, and N = 2 returns **2 mph**, which is two taps of
the executor's own 1 mph step and inside its ~1 mph set-report latency.

Three findings matter more than the table:

1. **The "only where vision is trustworthy" gate is not a gate.** On the 690 in-window map ICBM ticks
   whose model horizon actually covers the candidate, vision read **"no curve here at all" on 601
   (87 %)** and "a curve, but gentler than the map target" on 87 (13 %). **Exactly 2 ticks (0.3 %) read
   tighter.** A condition satisfied 99.7 % of the time does not discriminate between a phantom and a
   real curve — it fires on both. All of the rule's safety is in the bound N, none of it in the
   evidence.
2. **It cannot fix the event it was invented for.** The 2026-09-08 20:28 phantom IS in the corpus
   (§6): 69.1 → 43.2 mph on a straight 60 mph motorway, and vision read `visLat` 1.36–1.48 m/s² —
   *below* the 1.9 curve-enter threshold — on 3 of the 4 in-window ticks. The rule engages, correctly.
   But **the already-shipped map-rating floor takes 4.4 of those 25.9 mph back on its own**, and the
   most a bounded give-back can safely add on top is **2 mph**. The N that would add 10 is the N that
   puts a real R = 87 m bend at 5.80 m/s².
3. **A give-back confined to 50–150 m delivers nothing.** Released at the 50 m edge, the executor's
   own 1 mph / 0.4 s SET− walk re-takes the whole give-back before the apex: **zero of 88 episodes
   engage at N ≤ 5** (§3.4). The numbers above therefore assume the give-back **latches** through the
   last 50 m — the most permissive reading, and the one that creates the risk.

**What the owner was right about:** vision *is* better than `docs/CURVEDB2PNW.md` §1 says. Both of that
table's biases are real and both flatter the pessimistic conclusion (§4). The median 50–100 m ratio
moves 0.83 → 0.98 once the truth is de-biased, and 100–150 m moves 0.68 → 0.93 once vision's own term
is computed correctly. **It does not change the answer, because the give-back is killed by the tail,
not the median**, and the tail barely moves: under every combination tried, roughly one in six
in-window readings still under-reads by more than 2×, and the 10th-percentile reading under-reads the
real curvature by **3–7×**.

---

## 1. Corpora, and what each one could contribute

98 files swept, **0 unreadable**, 773,805 records, 70 unparseable lines — `scan.py` reports every file
as its own row so a corpus contributing nothing is visible (full table:
[`visionback-evidence/out_a1_capability.txt`](visionback-evidence/out_a1_capability.txt)).

| corpus | files | records | Ford moving ticks | **ICBM ticks** |
|---|---|---|---|---|
| `drives/**` (47 corpora: **20 carry ICBM activity, 27 do not**) | 92 | 730,887 | 69,288 | **3,087** |
| **`/tmp/arch` continuous archive (2026-09-16/17)** | 6 | 42,918 | 1,848 | **0** |

*(ICBM ticks are DEDUPED by timestamp across rotated generations of the same drive; the raw per-file
sum is 8,719. The deduped 3,087 matches `_scratch/icbmslow` exactly, which is the first of several
cross-checks against that independently-written harness.)*

> ### ⚠️ The continuous archive contributed ZERO ICBM decisions, and the reason is Rule 3
> The six rotated generations are **96 % parked ticks**: 41,239 of 42,918 records sit below 5 m/s.
> Three whole generations (09-16 02:21–08:21 PT) are 100 % stationary — the truck charging with the
> ignition on. The 1,848 moving ticks are three short city legs, all at or below 22 m/s (50 mph),
> and **not one of them has a non-null `icbmT`**.
>
> This is exactly the behaviour `CLAUDE.md` Rule 3 describes (`IsOnroad` follows ignition; the device
> records ~750 MB/h parked), now measured on the first continuous corpus this project has. It also
> sharpens `CURVEDB2PNW.md` §12's point about `drives/` being a biased window-sample: the unbiased log
> is *mostly parked*, so "weeks of continuous archive" will accumulate ICBM episodes far more slowly
> than the 21 MB/day budget suggests.
>
> The archive **is** usable for the vision-accuracy half (§4) — it carries `visLat`, `visTtc`,
> `icbmKVis`, `mdlEndX` and `slKActl` — and it is included there.

### Field availability, and the one that decides everything

| witness | what it is | non-null records | corpora |
|---|---|---|---|
| `visLat` / `visTtc` | ICBM's own vision candidate input (`vision_curve_lat_accel`) | 326,627 | 2026-07-13 onward |
| `mdlEndX` | the model horizon reach — identical quantity to `icbm_vision_curvature`'s `vis_reach` | 355,551 | 2026-07-13 onward |
| **`icbmKVis`** | **the correctly-computed vision curvature** (`max\|z_i\|/max(v_i,1)`) | 54,693 | **only 3 corpora** |
| `slKActl` | achieved curvature (truth) | 311,035 | 2026-08-11 onward |
| `kPeak` | per-second curvature PEAK (truth) | 10,885 | only 2026-09-17 |
| `icbmKAt` / `icbmKAtGap` | point-matched map geometry | 145,681 | weekend, archive, 09-13, 09-17 |

**`icbmKVis` is non-null on 56 of the 896 in-window map ICBM ticks, and all 56 come from one drive**
(`drives/2026-09-17/curvedb-first-capture`). Of the other two corpora that carry it, one is the parked
archive (0 ICBM ticks) and one is a 143-record file. So the *technically correct* vision witness and
an *ICBM decision* coexist on a single drive. Everything with broad coverage has to use `visLat`,
which carries a known bias (§4).

### Where the decisions live

1,868 map-sourced ICBM ticks carry a candidate distance; **896 fall in the 50–150 m window** (48 %).
The distance histogram says the window is the right place to look:

```
   0- 50 m: 857      150-200 m:  94
  50-100 m: 607      200-250 m:  19
 100-150 m: 286      250-350 m:   5
```

**756 far-sourced ICBM ticks have no logged candidate distance of their own** (`mapDist` is CES's 10 s
candidate and reads 0.0 on a far tick), so they cannot be placed in or out of the window and are
excluded by name, not silently.

---

## 2. ⚠️ The stock ACC was DISENGAGED on 61 % of all ICBM ticks

`stockOn` is `carState.cruiseState.enabled` (`ces_pnw.py:3842`). With cruise off, ICBM's SET− taps
reach nothing — `icbm_pnw`'s executor gates every press on cruise — so the published `icbmT` is
advisory and **a give-back could not have changed the truck's speed at all**.

| | ICBM ticks | `stockOn` False |
|---|---|---|
| whole corpus | 3,087 | **1,882 (61 %)** |
| `2026-09-12/central-oregon-weekend` (which dominates the sample) | 1,522 | **1,310 (86 %)** |

This nearly produced a wrong headline. The first run's worst case was "09-12 14:59:36, R = 87 m,
5.80 m/s² at 50 mph" — and the tick dump shows the truck was doing **30–34 mph through that whole
stretch with cruise OFF**, peaking at 1.71 m/s² measured. ICBM was a spectator. Both populations are
therefore reported separately:

| population | episodes > 25 mph | what it answers |
|---|---|---|
| **cruise ON** (map ticks with `stockOn` true) | **33** | what the truck would ACTUALLY have pulled |
| ALL ICBM ticks | 88 | what it would pull *if* ICBM had had authority everywhere |

Even inside the cruise-ON population, ICBM's commanded target sat **above** the speed the truck
actually met at the apex on 16 of 33 episodes — a lead, the driver or op-long was the binding
constraint there — so every absolute figure below is an **upper bound**, deliberately.

---

## 3. The counterfactual

### 3.1 The rule under test

```
eligible tick:  icbmSrc == "map"  and  50 m <= mapDist <= 150 m
                and a vision witness exists on that tick
                and the candidate is inside the model's own horizon   (mapDist <= mdlEndX)
                and vision's implied safe speed v_vis > the map target P_i
give_i      =   min(N mph, v_vis - P_i)
P'_i        =   min(P_i + give_i, ref - ICBM_MIN_DROP_MS, posted)     # posted only where known
```

`ref` is the latched ceiling / driver's set; `ref - ICBM_MIN_DROP_MS` is what keeps it a reduction and
never a cancel. `P'_i >= P_i` always.

### 3.2 The forward model is NOT new

`model_target()` is copied **verbatim** from `_scratch/icbmslow/a5_replay.py` — the model validated to
a median **+0.03…+0.10 m/s** residual against logged `icbmT` on every corpus from 2026-08-12 on. It is
run with `floor_raw=True`, i.e. **on top of the shipped map-rating floor** (`icbm_map_floor_frac` 1.0,
live on `origin/3devpnw` @ `d0a6b08abc`), so the baseline is today's car, not the pre-floor behaviour.

That is confirmed by the baseline reproducing the independently-published table in
`docs/pnw/ICBMSLOW2PNW.md` without tuning:

| | this harness (N = 0, 88 episodes) | ICBMSLOW2PNW.md |
|---|---|---|
| a_lat at ICBM's commanded target | median **1.65**, p90 **2.34** | median **1.65**, p90 **2.32** |
| a_lat the truck actually pulled | median **1.53** | median **1.50** |

### 3.3 Two extensions, both stated

1. **The executor's ratchet, taken from measurement rather than simulated.** ICBM caps are DEC-only:
   once the SET− walk has taken the set down, only the separate guarded restore can raise it
   (`IcbmEpisode.step`: *"once a low value is tapped, only the … RESTORE can undo it"*). So a give-back
   at 120 m cannot recover speed the walk already gave away before 150 m:

   ```
   T         = min over the episode's map ticks of P_i             (baseline)
   T'_window = min over ELIGIBLE ticks of P'_i
   S_entry   = the MEASURED stock set at the first tick with mapDist <= 150 m
   T'        = max( T,  min( S_entry, T'_window, T + N mph ) )
   ```

   `S_entry` is read from the log, not modelled. The `T + N mph` clamp is load-bearing: without it,
   holding the set high inside the window hands back up to **+27 mph** on episodes whose minimum
   target came from a tick outside the window — which is not the question that was asked.

2. **Scoring at the speed the truck would have reached**, `v_cf = max(v_apex_measured, T')`. The walk
   is monotone, so the counterfactual apex set lies in `[v_apex_measured, T']`; this takes the top of
   that interval. At N = 0 it returns the measured apex speed exactly, so the baseline row is what the
   truck did rather than a model of it.

**Truth curvature** = `max(kPeak, |slKActl|)` over the apex window, preferring `kPeak` where present
(it is ≥ the 1 Hz `|slKActl|` sample on **99.2 %** of the 6,638 ticks carrying both, median **1.90×**).
The `slKActl`-only variant is reported alongside and moves nothing material. An exact 0.0 is treated
as *no measurement*, never as straight road (`CURVEDB2PNW.md` D1).

**Two known data hazards, both handled by construction rather than by correction:**

* **Tesla is excluded at the scan** (`scan.py` keeps only `car` = `FORD_*`, with the pre-2026-07-13
  corpora mapped from their own `DRIVE_REPORT.md` headers). ICBM does not exist on the Raven, and
  `slKActl` there is **0 % live / 51,754 exact zeros** — a Tesla row could contribute neither a
  decision nor a truth curvature. Tesla record counts are printed per file so the exclusion is
  visible, not silent.
* **`kPose` / `achLatPose` are not used at all.** Those are the fields that shipped sign-inverted
  before `36f914a17c`. The truth here is `slKActl` (from `CS.yawRate`, the Ford CAN signal) and
  `kPeak`, which is defined as *"the per-second PEAK of `max(|achieved|, |commanded|)`"* — absolute
  by construction and derived from a different source than the pose. A sign inversion cannot reach
  either. Everything downstream additionally uses `abs()`.

### 3.4 The sweep

**Population: cruise ON — 33 episodes. This is the answer to "what would the truck actually have pulled".**

| N | engaged | ΔV med | a_lat med | p90 | p99 | **max** | ≥2.5 | ≥4.5 | worst site |
|---|---|---|---|---|---|---|---|---|---|
| 0 | 0 | — | 1.66 | 2.38 | 3.81 | **3.81** | 2 | 0 | 09-13 13:56:52, R 36 m |
| 2 | 11 | +1.48 | 1.79 | 2.45 | 3.81 | **3.81** | 2 | 0 | — unchanged |
| 5 | 11 | +2.46 | 1.79 | 2.45 | 3.81 | **3.81** | 2 | 0 | — unchanged |
| 10 | 14 | +2.46 | 1.79 | 2.45 | 4.68 | **4.68** | 2 | 1 | 09-13 13:56:52, 26 → 29 mph |
| 15 | 14 | +2.46 | 1.79 | 2.45 | 4.68 | **4.68** | 2 | 1 | same |

**Population: ALL 88 episodes** (the table in the verdict). Max 3.81 → **4.10** (N 2) → **4.70** (N 5)
→ **5.80** (N 10) → **7.01** (N 15); ≥ 4.5 counts 0 / 0 / 2 / 4 / 8.

**Unlatched** (the give-back released at the 50 m edge, the walk resuming at the executor's 2.5 mph/s):

| N | engaged (of 88) | max a_lat |
|---|---|---|
| 2 | **0** | 3.81 |
| 5 | **0** | 3.81 |
| 10 | 7 | 3.81 |
| 15 | 15 | 4.58 |

A give-back that really stops at 50 m does nothing below N = 10. Everything the feature might buy
depends on it surviving the last 50 m — the range where the map was right and vision's earlier reading
has already been superseded by arrival.

### 3.5 Window sensitivity

Moving the window (50–100, 100–150, 0–50, 0–300 m) changes engagement counts by a few episodes and
**does not change the maximum at any N** (3.81 / 4.10 / 4.70 / 5.80). The worst case is set by *which*
curve gets the give-back, not by where in the approach it is granted.

### 3.6 Vision witness sensitivity

| witness | in-window eligible ticks | engaged (of 88) at N = 5 | max a_lat at N = 5 |
|---|---|---|---|
| `vislat_bold` — "vision sees no curve" ⇒ full give-back | 451 | 40 | **4.70** |
| `vislat_strict` — refuse unless vision actually measured a curve | 65 | 12 | 3.81 |
| `icbmKVis` — the correct curvature (one drive only) | 38 | 2 | **4.51** |

`vislat_strict` is safe and useless: it refuses on the 386 in-window ticks where vision reported *no
curve at all*, which is precisely the phantom case the give-back exists to catch. `icbmKVis`, the
technically correct witness, still produces a **4.51 m/s²** outcome at N = 5 from its 38 eligible
ticks, on an R = 40 m bend at 09-17 13:23:10 where a 30 mph posted limit was the only thing capping it.

### 3.7 The apex-witness window does not decide the answer

Repeating the sweep under the three window definitions `_scratch/icbmslow/a5_replay.py` uses
(`a9_window_sensitivity.py`), maximum resulting a_lat:

| window | population | N 0 | N 2 | N 5 | N 10 | N 15 |
|---|---|---|---|---|---|---|
| tight | ALL (n 90) | 3.81 | 4.10 | 4.70 | **5.80** | 7.01 |
| base | ALL (n 88) | 3.81 | 4.10 | 4.70 | **5.80** | 7.01 |
| wide | ALL (n 86) | 4.96 | 4.96 | 4.96 | **5.80** | 7.01 |
| tight / base | cruise ON (n 33) | 3.81 | 3.81 | 3.81 | **4.68** | 4.68 |
| wide | cruise ON (n 32) | 4.93 | 4.93 | 4.93 | 4.93 | 4.93 |

`tight` and `base` agree exactly. `wide` raises the **baseline** to 4.93–4.96 — i.e. it is catching a
different, tighter curve than the episode's own, which is the window-contamination artifact
`ICBMSLOW2PNW.md` documents; it is reported, not used. **N = 10 exceeds 5.0 m/s² under all three.**

### 3.8 "Never above the posted limit" is unenforceable where it matters

`spdLim` reads 0.0 — posted limit unknown — on **408 of the 896** in-window map ICBM ticks (46 %), and
on essentially the whole central-Oregon weekend, i.e. on exactly the curvy roads where a give-back
would fire most. On those ticks the only remaining cap is the driver's own set speed, which is
`CURVEDB2PNW.md` D9's *"cap by set speed = full cancel by another name"* in a smaller costume.

---

## 4. Re-derived vision accuracy — the owner's caveat is real, and it is not enough

`docs/CURVEDB2PNW.md` §1 was produced by `drives/2026-09-12/central-oregon-weekend/vis_vs_dist.py`
with vision curvature = `|visLat| / vEgo²` and truth = the max `|achLat|/vEgo²` from the current tick
until `d + 40 m` had been travelled. **Both terms are biased, in the same direction.**

* **The truth is biased HIGH** (its own stated caveat): a max over the whole approach, not the peak at
  the matched site.
* **The vision term is biased LOW, and that was not flagged.** `visLat` is
  `vision_curve_lat_accel()`'s `orientationRate.z[i] * velocity.x[i]` — the lateral accel at the
  model's **own planned speed** at that point. Its curvature is `lat / velocity_x[i]²`, not
  `lat / v_ego²`. The model plans to SLOW for a curve it sees, so `velocity_x[i] ≤ v_ego` and dividing
  by `v_ego²` under-states the curvature on exactly the curves that matter.
  `ces_pnw.icbm_vision_curvature`'s docstring says this in as many words; `vis_vs_dist.py` did it
  anyway. (The same arithmetic makes `icbm_vision_apex` **over**-estimate the safe speed whenever the
  model is planning a slowdown — harmless in today's `min()` usage, dangerous in a give-back.)

Re-derived over the whole Ford corpus (`a4_vision_accuracy.py`, output committed):

| vision term | truth term | 0–50 m | 50–100 m | 100–150 m | 150–200 m |
|---|---|---|---|---|---|
| §1 published | | 1.01 (7 % bad) | 0.90 (13 %) | 0.75 (28 %) | 0.69 (29 %) |
| `visLat/v_ego²` | §1's run-out-40 | **0.98** (11 %) | **0.83** (24 %) | **0.68** (37 %) | 0.61 (41 %) |
| `visLat/v_ego²` | peak ±25 m at the site | **1.06** (8 %) | **0.98** (17 %) | **0.80** (30 %) | 0.75 (36 %) |
| `icbmKVis` (correct) | §1's run-out-40 | 1.03 (7 %) | 0.80 (23 %) | **0.90** (33 %) | 0.96 (6 %) |
| `icbmKVis` (correct) | peak ±25 m at the site | 1.06 (7 %) | **0.95** (16 %) | **0.93** (24 %) | 1.17 (6 %) |

*(median vision/truth ratio; "bad" = fraction under-reading by more than 2×. n per bucket: 366–548 for
the `visLat` rows over the whole corpus; **21–35 for the `icbmKVis` rows**, which are one drive.)*

**Readings.**

* §1 replicates (my run is very slightly worse because it spans every Ford corpus, not one weekend).
* **De-biasing the truth is worth about +0.13 on the median ratio** at 50–150 m. The caveat was real.
* **Computing vision's own term correctly is worth roughly another +0.1…+0.2** at 100–150 m. So at
  100–150 m the honest median moves **0.75 → ~0.93**: vision is close to unbiased on a typical curve,
  much better than §1 implied, and the owner's instinct that vision has something to contribute is
  **supported on the median**.
* **And it changes nothing, because the give-back is killed by the tail.** Across every combination,
  the 10th-percentile ratio at 50–150 m is **0.14–0.37** — one reading in ten under-reads the real
  curvature by **3–7×** — and "badly under" stays at **16–30 %**. The reading the give-back acts on is
  "vision says gentle", and one in six of those is wrong by more than 2× in the unsafe direction.
  A rule whose safety rests on a bound of N mph can tolerate that; a rule that fires on a real R = 87 m
  bend with N = 10 mph cannot.

---

## 5. What the give-back would actually buy

Exposure behind the 88 episodes: **19.5 Ford driving hours, 808 miles** across 23 corpora (rotated
duplicate generations are summed, so exposure is over-counted and the rate below is an under-count).

| N | episodes engaged | rate | median recovery | total recovered |
|---|---|---|---|---|
| 2 | 38 / 88 | 1.95 / driving-hour, 4.7 / 100 mi | **+2.00 mph** | 66 mph-episodes |
| 5 | 40 / 88 | 2.06 / hour | +3.05 mph | 134 |
| 10 | 45 / 88 | 2.31 / hour | +3.33 mph | 229 |

Restricted to cruise-ON (where it could actually act): **11 of 33 episodes at N = 2**, ≈ 0.6 per
driving hour, median **+1.48 mph**.

So the honest shape of the offer is: **about one engagement per 1.7 driving hours, worth 1.5–2 mph**,
in exchange for control-path code in the ICBM decision that acts on a reading which is wrong by more
than 2× on one in six of the ticks it acts on.

---

## 6. The motivating phantom, tested directly

`drives/2026-09-08/…` does contain the 20:28 event. It also gives the forward model a spot check on a
live phantom: the logged `icbmT` held **44.1–44.9 mph** across the episode and the model with the floor
OFF returns **45.3**, a residual of **+0.4…+1.2 mph (+0.2…+0.5 m/s)**. That is larger than the
corpus-wide +0.03…+0.10 m/s `ICBMSLOW2PNW.md` reports, and it is stated rather than smoothed: `ref` is
reconstructed here from `icbmC`/`stockSet`, and the logged target reflects the episode's latched
ceiling rather than the current stock set. It biases the modelled target slightly HIGH, i.e. it makes
the shipped baseline look marginally better than it was, not worse.

| PT | `stockOn` | vEgo | stock set | `mapDist` | logged `icbmT` | model, floor ON | `visLat` | vision verdict |
|---|---|---|---|---|---|---|---|---|
| 20:28:52 | True | 68.5 | 65 | 300 m | 44.9 | 49.7 | — | *(outside the window)* |
| **20:28:57** | True | 59.9 | 53 | **144 m** | 44.2 | 49.7 | 2.06 | a curve, safe at 66 mph |
| **20:28:58** | True | 57.0 | 50 | **113 m** | 44.3 | 49.7 | **1.48** | **no curve at all** |
| **20:28:59** | True | 54.1 | 48 | **84 m** | 44.4 | 49.7 | **1.48** | **no curve at all** |
| **20:29:00** | True | 51.2 | 45 | **57 m** | 44.4 | 49.7 | **1.36** | **no curve at all** |
| 20:29:02 | True | 45.9 | 44 | 6 m | 44.3 | 49.7 | 1.05 | *(inside 50 m)* |

The rule engages, in the right direction, on the right event — vision was reading *below* the 1.9 m/s²
curve-enter threshold on 3 of the 4 in-window ticks while the map dragged the set 70 → 44.

**But the shipped map-rating floor has already taken the large bite.** On this exact event the floor
moves the modelled command **45.3 → 49.7 mph (+4.4)** — against the 44.4 the truck was actually
commanded that day, +5.3 — i.e. it alone removes about a fifth of the 25.9 mph the truck gave away
(69.1 → 43.2 mph). What a bounded vision give-back could add on top is
**2 mph at the only N the data permits** — a rounding error against what is left. The N that would
return 10 more is the N that puts a real bend at 5.80 m/s².

*(Aside, not this analysis's subject, but it is the sharper tool on this event: `mapDist` runs
**6 → 17 → 40 → 62 → 84 → 104 m** over 20:29:02–20:29:07 while `mapV` never moves — it GROWS at
`v_ego`. That is `icbm_path_behind`'s behind-the-truck signature, which `behindgate2pnw` /
`behindrun2pnw` already ship for, and unlike vision it has no under-read tail.)*

---

## 7. What this does and does not settle

**Settled by measurement:**

* N = 10 is dead (5.80 m/s²). N = 5 has no margin (4.70). N = 2 is safe and buys ~2 mph.
* Vision's "gentle" verdict is not a discriminator: 688 of 690 in-window readings say gentle-or-nothing.
* A give-back confined to 50–150 m delivers nothing; it only works if it latches through the last 50 m.
* `CURVEDB2PNW.md` §1 understates vision by roughly 0.2 on the median ratio at 100–150 m, from two
  separate biases. **§1 should be corrected.** Its conclusion survives, its numbers do not.

**Not settled, and stated as such:**

* **n is small.** 33 cruise-ON episodes, 13 of them from one weekend corpus. A worst case of 4.68
  from a single R = 36 m bend is one observation, not a distribution.
* **The correct vision witness has essentially no coverage.** `icbmKVis` + an ICBM decision coexist on
  one drive (56 in-window ticks). The broad-coverage results all lean on `visLat`, whose bias is now
  characterised but not removed — `velocity.x[i]` is not in `ces_events`, and recovering it needs
  rlogs, not this log. **If the owner wants this question re-asked properly, the cheapest move is to
  log `icbmKVis` unconditionally rather than only where curvelead2pnw telemetry exists.**
* **The absolute lateral accels are upper bounds**, by construction, and on 16 of 33 cruise-ON
  episodes ICBM's target sat above the speed the truck actually met.
* The counterfactual does not model the descent guard or the left-curve factor (`icbmDir` is the
  episode direction, `vtscPitch` is op-long-only) — same limitation `ICBMSLOW2PNW.md` records. The
  *delta* is unaffected; the absolute values are slight over-estimates on downhill lefts.

## 8. If the owner still wants vision in this loop

Not proposed, recorded because the measurement points at them:

1. **N = 2 as a pure comfort trim, gated to `icbmKVis` and a known posted limit.** Safe on this corpus
   (max unchanged at 3.81 cruise-ON), fires ~0.6×/hour, returns 1.5 mph. Honest description: it is not
   worth control-path risk.
2. **Log `icbmKVis` + `mdlEndX` on every tick** and re-ask in six weeks. Cost is bytes; it converts the
   one-drive column of §4 into a real measurement.
3. **The behind-the-truck test is the better lever on this corpus** — it fires on the 09-08 phantom
   with a signature (`mapDist` growing at `v_ego`) that has no under-read tail at all.

## Reproduce

Harness: `/home/dp/gh/comma/_scratch/visionback/` (also committed at
[`visionback-evidence/`](visionback-evidence/) with its saved outputs).

```bash
cd /home/dp/gh/comma/_scratch/visionback
PY=/home/dp/gh/comma/pnw/pnw-pilot/.venv/bin/python
PP=/home/dp/gh/comma/pnw/wt-icbmslow:/home/dp/gh/comma/pnw/wt-icbmslow/opendbc_repo

python3 scan.py                      # 98 files -> inventory.json + ticks.jsonl  (~70 s)
python3 a1_capability.py             # per-corpus field availability
python3 a2_sizing.py                 # how many ticks can carry a give-back decision
PYTHONPATH=$PP $PY a3_giveback.py    # THE SWEEP (both populations x both truths x 3 witnesses)
python3 a4_vision_accuracy.py        # vision accuracy vs distance, both terms de-biased
PYTHONPATH=$PP $PY a5_worst.py       # window sweep, named worst sites, firing rate
PYTHONPATH=$PP $PY a6_worstsite_dump.py   # tick-by-tick dump of the max-producing episodes
PYTHONPATH=$PP $PY a7_diag.py             # commanded-vs-measured split + vision's verdict
PYTHONPATH=$PP $PY a8_phantom.py          # the 2026-09-08 phantom, floor ON vs OFF
PYTHONPATH=$PP $PY a9_window_sensitivity.py   # does the apex window decide the answer? (no)
```

## Related

`docs/CURVEDB2PNW.md` §1 (the table this corrects) · `docs/pnw/ICBMSLOW2PNW.md` (the map-rating floor
this is measured on top of, and the forward model reused here) · `docs/pnw/ICBM2PNW.md` ·
`docs/LIGHTNING-STEERING-LIMITS.md` (the ~4.5 m/s² ceiling) ·
`ces_pnw.py::icbm_map_sanity` (the unwired veto) · `ces_pnw.py::icbm_path_behind`.
