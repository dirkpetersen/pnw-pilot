# AUTOTUNER-DESIGN — self-tuning for the bp-7.0 angle-control port (design review, no code)

**Date:** 2026-07-19 · **Author:** analysis session (Claude), on driver request
**Scope:** should a self-tuning loop for Alan Polk's Low/High Speed Factor exist; if so, its safest
useful first version; the estimator spec; interaction with the exit-unwind defect; the field-data
package for Alan Polk; upstreaming.
**Inputs:** `ALAN-POLK-TUNING-TUTORIAL.md` (his method, transcribed from his two videos),
`DRIVE_REPORT.md` (this folder, 2026-07-19), `ALAN-POLK-SPEC.md`, `ALAN-POLK-PORT-DEVIATIONS.md`,
`pnw-opendbc:angle2pnw-faithful2` (`lateral_angle_pnw.py`, `angle_tuning_pnw.py`,
`angle_tuning.reference.json`), and bp-7.0 source (`sunny/bluepilot`, tip `19858f2888`) —
specifically his own debug widgets, which resolve two design-critical unknowns (§C.0).

---

## A. Verdict: build it — as a SHADOW/ADVISORY post-drive analyzer. Do NOT build closed-loop auto-apply now, and NEVER mid-drive.

**Yes, this should exist**, because the driver's actual stated pain is **in-drive interaction**
("I have to currently interact way too much speaking to my laptop while driving"). The thing he is
doing by voice mid-drive is *measurement and judgment* — watching curve behavior, deciding
overshoot vs undershoot, dictating observations. A shadow tuner automates exactly that part:
during the drive there is **zero interaction**; between drives the system presents a fully-justified
recommendation and the driver accepts it with one command. Shadow mode alone removes ~100 % of the
in-drive burden. Auto-apply only removes the *between-drive* one-command review — which is not the
stated pain, and is where all of the risk lives.

**The failure mode of a bad auto-tuner, concretely:** the tuned parameters are multiplicative gains
on the actual steering command (`path_angle = κ·v·gain`). A wrong-direction adjustment does not
degrade gracefully — it changes how hard the truck steers in every curve. We do not have to
speculate about severity: **on 2026-07-19 this exact system put the truck into the oncoming lane**
after a *human plus an analyst*, using a plausible-looking metric (the delivery ratio), moved one
factor 20 % in the wrong direction (`DRIVE_REPORT.md` §Delivery-ratio verdict, §100 Hz anatomy).
The metric was structurally confounded (controller's own error correction + its own v>9 clip gate
read as "PSCM under-delivery") and every low-speed observation was ICBM-contaminated. An auto-tuner
encoding a wrong metric is that same mistake, automated, repeated every drive, without the human
skepticism that caught it after one day. **That failure mode is unacceptable for an autonomous
loop and acceptable for an advisory one** — a wrong shadow recommendation is a wrong sentence in a
report, reviewed while parked.

Two further reasons the closed loop is premature *today*, independent of principle:

1. **The estimator's input signal only becomes trustworthy tomorrow.** The green/yellow comparison
   (§C) was not logged on any drive to date; the wire command has never been observed; the one
   metric we did compute was refuted. An auto-tuner cannot be safer than its estimator, and the
   estimator has not produced one validated number yet.
2. **The dominant observed defect is not tunable** (§D). Until exit-unwind holds are either fixed
   or reliably excluded, a closed loop would be optimizing a gain against a failure the gain does
   not control.

Mid-drive auto-apply (tempting because the JSON overlay hot-reloads in ~5 s) is **rejected
permanently**, not just for v1: it changes the config mid-drive (destroying the matched-pair /
config-boundary analysis that today's report shows is already painful to reconstruct), it applies
changes at the moment of least human oversight, and a time-varying gain is effectively a new
control mechanism — a deviation from Alan Polk's design, not a use of it (§F, last paragraph).

---

## B. Safest useful first version: SHADOW mode, and what graduation would require

### The three candidates, on the metrics that matter

| | Shadow/advisory (recommended) | Auto-apply between drives | Auto-apply mid-drive |
|---|---|---|---|
| **Safety** | Wrong estimator output = wrong text. Human gate before any gain change. | Wrong estimator output = wrong steering gain next drive. Bounded by envelope (§C.5) but unreviewed. | Wrong output = wrong gain **while driving**, with no review at all. |
| **Data quality for Alan** | Best: one config per drive, clean matched pairs, recommendation-vs-outcome pairs (the most valuable field data an author can get: "your method said X, X was applied, here is the next drive"). | Good if strictly one change/drive; still loses the human-verdict column. | Worst: config boundaries mid-drive; today's report spent a whole section reconstructing exactly this. |
| **Driver's stated goal (no laptop while driving)** | **Fully met.** Measurement is automated; in-drive interaction = zero. One accept/reject command while parked. | Met, plus removes the parked command. | Met, same as previous. |
| **Marginal benefit over shadow** | — | Saves ~10 s per drive. | Saves nothing the driver asked for. |

The marginal benefit of auto-apply is tiny and its marginal risk is the entire failure mode of §A.
**Recommendation: shadow/advisory.**

### What shadow mode is, concretely

- **A post-drive offline analyzer** (Python, runs on the laptop over pulled logs; optionally
  on-device offroad, run explicitly over SSH). **No new onroad process, no UI, no live
  subscriptions of any kind** (§ Constraints below for where the data comes from).
- Input: the drive's rlog segments (primary) and/or the 100 Hz capture file, plus
  `ces_events.jsonl` and the `angle_tuning.json` history.
- Output, written next to the drive's report folder:
  1. `recommendation.json` — either `{"change": {"low_speed_curv_factor": 0.98}, ...}` with the
     full evidence chain (every curve sample, every gate decision, the update-law arithmetic), or
     `{"change": null, "reason": "..."}` (insufficient/contradictory evidence, exit-hold-dominant
     drive, incident freeze).
  2. `curves.jsonl` + `bins.json` + selected traces — the Alan Polk field package (§E), produced
     as a by-product of the same pass.
- Applying: the driver reviews and runs one command (`scp`/`ssh` writing
  `/data/pnw/angle_tuning.json`, appending to `/data/pnw/angle_tuning_history.jsonl`). The
  hot-reload makes it live in ~5 s; per the one-change-per-drive discipline it is done while
  parked, before the next drive.

### Graduation criteria — what must be true before auto-apply (between drives) is even discussed

1. **Backtest regression passes.** Run the analyzer over every historical capture. It must
   (a) recommend *no increase* from the 2026-07-19 S0 data that seduced the delivery-ratio
   analysis, (b) output REVERT-AND-FREEZE for the 15:06:37 departure drive, and (c) classify the
   15:34:35 excursion curve as exit-hold/incident, not as gain evidence. These three are frozen as
   permanent regression fixtures.
2. **≥ 5 consecutive drives** where the shadow recommendation was reviewed, applied (or its "no
   change" accepted), and the *following* drive's data confirmed the direction (no
   recommendation ever later judged wrong-direction).
3. **The exit-unwind defect is root-caused** (wire telemetry distinguishing clip / blend / PSCM
   lag) and either fixed upstream-style or demonstrably excluded by the classifier (§D) with zero
   leakage on the fixtures.
4. **Estimator cross-validation:** green/yellow peak verdicts spot-checked against the wire
   (`LatCtlPath_An_Actl`) chain on ≥ 3 drives; per-drive lag estimate stable and consistent with
   `liveDelay.lateralDelay` + measured ~300 ms.
5. **The envelope machinery (§C.5) exists and is tested offline** — tighter auto-clamps, one
   parameter × one bounded step per drive, append-only provenance log, incident auto-revert.
6. **ICBM policy settled:** tuning sessions with ICBM active are excluded by gate (already in
   §C.1); confirmed that enough clean samples survive on normal (non-session) drives to be useful.

Even after graduation: auto-apply happens **only offroad** (trigger: `gearShifter == park` per the
established Lightning convention — never `IsOnroad`), one parameter, one quantum, driver notified
in the drive report. Mid-drive stays forbidden.

---

## C. The estimator

### C.0 Two facts from bp-7.0 source that shape everything (verified today)

1. **His green and yellow lines are both steering-wheel degrees, and both already exist in our
   logs.** `bluepilot/ui/widgets/debug/lateral_debug_panel.py` (bp-7.0, lines 37–67): green
   ("Angle Desired") = `carControl.actuators.steeringAngleDeg`; yellow ("Angle Actual") =
   `carState.steeringAngleDeg`. Ford is `SteerControlType.angle` in both trees, so `controlsd` /
   `LatControlAngle` populates `actuators.steeringAngleDeg =
   degrees(VM.get_steer_from_curvature(-desired_curvature, vEgo, roll)) + angleOffsetDeg` —
   identical in our `pnw-pilot/selfdrive/controls/controlsd.py:128`. **No unit conversion problem,
   no new signal needed for the core method** — green is the planner's wish mapped through the
   vehicle model, i.e. *upstream of the entire angle pipeline*, so a yellow-vs-green peak
   comparison integrates blend + clip + gain + PSCM exactly the way his eyeball method does.
   (The wire `path_angle` from sendcan 982 is still wanted — as the *attribution* signal telling
   us which pipeline stage ate the difference, and for §D. It is not the green line.)
2. **A "click" is 0.01, and his UI splits it by the speed weight.**
   `angle_factor_adjuster.py`: `STEP = 0.01`, clamps 0.5–1.5, and one tap moves
   `Ford{Low,High}SpeedFactor_ang` *together*, split `w_low/w_high` where
   `w_high = clip((v−13.5)/(26.82−13.5), 0, 1)` — the "blue shaded band" in his video is exactly
   this interp position (`SPEED_HIGHLIGHT` sliding bar). His spoken "back off two or three clicks"
   = **0.02–0.03**. This anchors the update-law step size (§C.4) in his own practice, and confirms
   the attribution weighting he himself uses (§C.3).

### C.1 Sample-validity gate

Unit of evidence = a **curve event**: a contiguous window where |green| ≥ threshold (below) for
≥ 1.0 s. All gates evaluated over the window **± 1 s margin**. Every exclusion is logged with its
reason (the gate log is itself field data for Alan).

| Gate | Rule | Why / source |
|---|---|---|
| Engaged, hands-free | `latActive` throughout; `steeringPressed` false throughout ±1 s; no human-turn override; no stall/press blip (mode-0 pulse on wire, or strategy flags once logged) | A press mid-curve both contaminates the measurement and (via the blip machinery) resets the PSCM. |
| No lane change | `modelV2.meta.laneChangeState == off` throughout | Lane-change factor multiplies the command (`precision_type=0` path); different knob, excluded from v1 entirely. |
| Steady speed | max |a_long| ≤ 0.5 m/s² AND (v_max−v_min)/v_mean ≤ 10 % over the window | Kills ICBM taps, driver braking, stop approach *by measurement, not by flag* — `path_angle = κ·v·gain` scales with v, so any mid-curve Δv directly corrupts the peak (the entire (b) "runs wide" dataset died this way). |
| ICBM belt-and-braces | cross-tag `ces_events.jsonl`; any icbm activity in window → exclude | Redundant with the speed gate; keeps the exclusion reason explicit for the report. |
| Not near a stop | v ≥ 8 m/s throughout; v never < 5 m/s within ±5 s | Alan: "the model gets really squirrely" near stops. 8 m/s also stays clear of the v>9 clip boundary flapping. |
| Curve magnitude above noise | peak |green| ≥ interp(v, [8, 15, 25] m/s → [12°, 10°, 6°]) AND the peak sustains ≥ 90 % of max for ≥ 0.3 s | His ±5° road-noise floor, applied to the signal (peak must clear it decisively) and to the verdict (§C.2 deadband). High-speed curves legitimately use smaller wheel angles, hence the taper. |
| Pipeline not saturated at peak | deviation clip not binding (offline proxy: NOT(v>9 AND \|actuators.curvature − κ_meas\| > 0.002) at peak); soft-ROC not binding; `_in_hard_sat` not active | If a clamp bound at the peak, yellow-vs-green measures **the clamp**, not the gain — evidence would be attributed to the wrong mechanism. Once the strategy's `bp_*_limited` flags are logged, use them directly instead of proxies. |
| Peak-mismatch, not exit-hold | §D classifier says the curve's dominant error is at the peak | The load-bearing gate; see §D. |
| Entry/exit transients | **Excluded from gain evidence by construction** — we measure only at the sustained peak (next section). Entry/exit samples feed the §D classifier and the lag estimate, never the gain. | Alan: "judge at the peak of each curve, not the whole trace." |
| No incident | Any driver grab during a hands-free curve, emergency brake, or lane-position excursion beyond threshold anywhere in the drive → the whole drive's recommendation is REVERT/FREEZE (§C.5); no gain evidence is emitted at all | Incidents outrank statistics. |

### C.2 What to measure at a curve peak, and time alignment

**Do not align pointwise; compare peak magnitudes.** Alan is explicit that the lines never overlay
("there's some delay and lag… what we're really looking for is peaks in curves"), and pointwise
differencing of two lagged traces manufactures phantom overshoot on every entry and phantom
undershoot on every exit. Per curve event:

1. `t_g` = time of max |green| in the window; require the sustained-peak condition (§C.1).
2. Find yellow's local extremum of the **same sign** in `[t_g − 0.1 s, t_g + 1.0 s]`. The +1.0 s
   ceiling generously covers the measured ~300 ms command→yaw lag; the −0.1 s floor rejects
   pairing with a *previous* curve.
3. Per-curve verdict: `Δ = |yellow_peak| − |green_peak|` (degrees), plus ratio for the report.
   - `Δ > +3°` → **overshoot** sample (evidence to reduce).
   - `Δ < −3°` → **undershoot** sample — *suspect until speed is ruled out* (§C.4).
   - `|Δ| ≤ 3°` → in-family (within his ±5° noise philosophy; 3° on the *peak difference* is
     stricter than 5° on the raw trace, deliberately).
4. **Lag is estimated, reported, but not used to shift the verdict:** per drive, cross-correlate
   green vs yellow over all hands-free driving → one lag number; sanity-check against
   `liveDelay.lateralDelay` (which the strategy itself consumes as the VLT `t_base`) and against
   the ~300 ms measured on 2026-07-19. A drifting or bimodal lag estimate is itself a red flag
   that freezes tuning for the drive (it usually means mode-0 pulses or PSCM misbehavior).
   `liveDelay.lateralDelay` alone is *not* sufficient for alignment — it is the planner's
   pre-compensation input, not the end-to-end plant lag, and today's data showed calibration
   excursions to ~420 ms are possible.

### C.3 Attribution to `low_speed_curv_factor` vs `high_speed_curv_factor`

The gain the sample actually experienced (his line 445–446 / our 553–559):

```
w_hi  = clip((v_peak − gain_speed_lo_ms) / (gain_speed_hi_ms − gain_speed_lo_ms), 0, 1)   # his blue band
high_gain_calc = interp → (1.30·low_factor) at w_hi=0  …  (0.95·high_factor) at w_hi=1
curvature_factor = interp(|κ_cmd|, [0.0007, 0.001], [low_gain_calc, high_gain_calc])
```

- **Speed attribution = the blue band itself:** a sample's evidence is split
  `(1 − w_hi) → low_speed_curv_factor`, `w_hi → high_speed_curv_factor` — exactly the split his
  own adjuster applies to a click. A factor only becomes eligible for a recommendation when its
  **accumulated attributed weight ≥ 3.0 curve-equivalents** in the drive.
- **The sub-30 mph flat clamp is embraced, not fought:** every sample with `v_peak ≤
  gain_speed_lo_ms` (13.5 m/s as configured) carries `w_hi = 0` — ALL sub-30 mph evidence, from
  8 m/s up, bears on the single number `low_speed_curv_factor`, and the estimator must say so in
  its justification text (this is precisely how the 2026-07-19 session fixed 20 mph and broke
  30 mph with one knob). The attribution always uses the **currently configured** breakpoints from
  the tuning history, not hardcoded 13.5/26.82 — if the anchor experiment (`gain_speed_lo_ms →
  9.0`, DRIVE_REPORT Pass 2) runs, the weights follow it automatically.
- **Curvature-axis restriction (v1):** only samples with `|κ_cmd| ≥ curvature_factor_bp_hi`
  (0.001) are tuning-eligible — there `curvature_factor == high_gain_calc` exactly and attribution
  is purely the speed weight. Samples in the mixed band (0.0007–0.001) are excluded; samples below
  0.0007 (gentle sweepers, gain = `low_gain_calc` → `gain_lowC_highV` territory) are *logged* per
  speed bin for Alan but never tuned in v1. This closes the 2-D attribution problem by refusing
  the ambiguous cells rather than modeling them.
- **The auto-tuner's write set is exactly Alan's two factors** — the same two keys his own
  stepper widget moves. `low_speed_boost`, `gain_speed_lo/hi_ms`, `path_angle_blend_ratio`,
  `vlt_extra_max`, `curvature_factor_bp_*`, `lane_change_factor_high_ang`: **never auto-touched.**
  Those remain human, deliberate, one-at-a-time experiments (they are our overlay's extras or
  structural constants; several are the exit-defect suspects).

### C.4 Update law

Anchored in his practice: click = 0.01, "back off two or three clicks" on overshoot, skeptical of
undershoot, "you could spend forever trying to tune it perfect."

| Element | Rule |
|---|---|
| Quantum | 0.01 (his click). Recommended change = `−0.01 · ceil(median(Δ)/3°)` capped at ±limit below. |
| **Reduce on overshoot** | ≥ 3 corroborating curves (≥ 2 distinct locations, or both directions) with attributed-weight ≥ 3.0 and median Δ > +3°, and **no** valid undershoot cluster in the same drive → recommend down, max **−0.03/drive** (his "two or three clicks"). |
| **Increase on undershoot** | Stricter on every axis: ≥ 5 corroborating curves across ≥ 2 locations, **every** sample speed-vetted, **zero** overshoot samples anywhere in the drive → recommend up, max **+0.01/drive** (one click). Speed-vetting per sample: lateral accel at peak `a_lat = v²·|κ_meas| ≤ 2.0 m/s²` (below the fleet's tuned 2.5 comfort target — if the truck was near its lateral-accel envelope, "too fast" cannot be excluded, per Alan's own diagnostic), AND no PSCM/DBC/soft-ROC saturation at peak (Ford "cuts back" authority with speed — undershoot at the authority ceiling is not a gain deficit). |
| Mixed over+under in one drive | **No change.** Diagnosis output: "speed inconsistency — the undershot curves were likely taken too fast" (his exact diagnostic), with the per-curve a_lat table. |
| Hysteresis | A direction flip (last applied change was opposite sign) requires the full evidence bar met on **2 consecutive drives** before recommending. Prevents the tuner oscillating around plant noise. |
| Deadband | median |Δ| ≤ 3° → "in tune, no change" (explicitly a success output, not silence). |
| Per-drive budget | **One parameter, one recommendation.** If both factors qualify, recommend only the one with more attributed weight; note the other as pending. Preserves matched-pair analyzability — this discipline is load-bearing for everything in §E. |

### C.5 Safety envelope (tighter than his, because a loop has no judgment)

- **Auto-range:** `[0.85, 1.15]` for both factors — deliberately tighter than his human UI range
  (0.5–1.5) and just wide enough to cover the observed useful region (platform gain 0.95 ±
  realistic trim). The human keeps the full 0.5–1.5 by hand-editing the JSON; the *analyzer
  refuses to recommend* outside 0.85–1.15 and the (future) auto-apply path hard-clamps there.
- **No naive monotonic ratchet — because gain has no safe direction.** Undershoot runs wide
  (outside of the curve: on a right-hander that is the oncoming lane); overshoot cuts inside (on a
  left-hander that is the oncoming lane). Both directions can produce a departure. The honest
  ratchet is **toward the validated baseline**: steps that shrink |factor − baseline| (baseline =
  1.0 × platform default, the config that drove clean all of S0) get the normal evidence bar;
  steps that grow it get the stricter undershoot-grade bar regardless of direction, and the
  cumulative excursion from baseline is capped at **±0.05 until re-validated** by an incident-free
  review cycle.
- **Automatic revert-and-freeze:** the incident detector (driver grab during a hands-free curve
  + concurrent lane-position excursion from `modelV2.laneLines`, or emergency braking in-curve, or
  disengagement in-curve) → output REVERT to last-known-good tuning vector + set a `frozen` flag
  in the history log. While frozen, the analyzer emits diagnosis only. Only a human clears the
  freeze. (In shadow mode this is a recommendation with REVERT priority; in a graduated auto mode
  it is the one change that may be applied without review — reverting to a config that has already
  driven clean is the only intrinsically safe automatic action this system has.)
- **Provenance:** every applied change — human, shadow-accepted, or (future) auto — appends to
  `/data/pnw/angle_tuning_history.jsonl`: timestamp, actor, prior→new vector, evidence digest,
  drive id. This is simultaneously the audit trail, the config-boundary record that
  `DRIVE_REPORT.md` had to reconstruct forensically from file mtimes, and a section of Alan's
  data package.
- **Fidelity guard:** the tuner writes only the overlay JSON keys, inside their existing loader
  clamps (`angle_tuning_pnw.py` `_RANGES`), which themselves mirror his UI clips. It never touches
  code, never touches the non-exposed constants, never writes mid-drive. Within these rules the
  loop *is* Alan's method — his measurement (green/yellow at peaks), his verdict rules (reduce on
  overshoot, suspect undershoot, ±noise floor, never near stops), his step size (clicks), his
  knobs (the two factors), just with the eyeball replaced by the log.

---

## D. The exit-unwind hold: the tuner must detect it and refuse

**Would a peak-based tuner make it better, worse, or leave it untouched?** Mechanically it leaves
the defect untouched: the hold lives in the exit path (deviation clip pinning the command to
measured curvature above 9 m/s, and/or the effectively-dead `_desired_falling` blend collapse —
its 0.010-per-20 Hz-call threshold ≈ 0.2 κ/s is ~500× the real planner unwind rate, verified
identical in his source), and none of that is scaled by the speed factors. Lower gain does reduce
the *stored rotation* to shed at exit (consistent with 1.0 surviving the corner 1.2 departed), so a
correct tuner slightly shrinks the blast radius — but the trap remains at any gain.

**The active danger is misattribution, and it points the wrong way.** During a hold, yellow stays
pinned at the apex value while green collapses — for the >1 s the 15:06:37 trace shows, yellow
*exceeds* green by a growing margin. A naive estimator that samples anywhere past the apex reads
this as textbook overshoot and reduces the factor: it launders a structural defect into a gain
change, degrading entry tracking on every curve while fixing nothing at exit, then finds "overshoot"
again next drive. This is the auto-tuner death spiral for this specific plant.

**Therefore: yes, detect and refuse.** Per-curve classifier, run before any sample becomes gain
evidence:

- **Exit-hold signature:** after the green peak, green magnitude falls by ≥ 30 % of peak within
  1.0 s while yellow remains within 10 % of its peak for ≥ 0.7 s (thresholds calibrated so the
  15:06:37 100 Hz trace — green 0.0109→0.0028 in 1 s, yellow holding >1 s — classifies
  unambiguously; tighten on tomorrow's wire data). Such a curve is **excluded from gain evidence**
  and counted in a separate exit-hold defect tally with its trace saved.
- **Dominant-defect refusal:** if exit-hold curves ≥ 30 % of otherwise-eligible curves in a drive,
  the analyzer recommends **nothing** for the gains and outputs the exit-hold diagnosis instead
  ("the dominant error today is not tunable by Alan Polk's method; fix or root-cause the exit path
  first"), with the per-curve evidence and — once the wire is logged — which stage (clip vs blend
  vs PSCM lag) held the turn.
- The exit-hold tally, rate, and traces go into Alan's package (§E) — it is a finding about *his*
  shipping code (`_desired_falling` threshold; the v>9 clip trap at exit), reported with data, and
  he can check his Mach-E/F-150 fleet for the same signature.

---

## E. The data package for Alan Polk

One `tar.gz` per analyzed drive (target < 2 MB), schema-versioned `bp_angle_field_report_v1`.
Privacy: no GPS coordinates, no VIN, no route reconstruction possible — curves are identified by
opaque ids + geometry (κ, direction, length, speed), dates at day precision, timestamps relative to
drive start. Contents:

1. **`manifest.json`** — schema version; date (day); platform (`FORD_F_150_LIGHTNING_MK1` +
   his gain-pair `(0.95, 0.95)`); software identity (bp-7.0 tip `19858f2888`, our port branch +
   commit, hash of `ALAN-POLK-PORT-DEVIATIONS.md` so he knows exactly which code ran); the full
   12-key tuning vector(s) in force with relative-time boundaries; configured anchors
   (`gain_speed_lo/hi_ms`); per-drive lag estimate + `liveDelay.lateralDelay` stats; drive
   duration/distance bucket.
2. **`curves.jsonl`** — one record per curve event (including excluded ones):
   `{id, dir, v_peak, w_hi, kappa_cmd_peak, kappa_meas_peak, a_lat_peak, green_peak_deg,
   yellow_peak_deg, delta_deg, ratio, lag_s, duration_s, class, gates{...}, tuning_vector_id}`
   where `class ∈ {clean, overshoot, undershoot_speed_suspect, undershoot_valid, exit_hold,
   excluded:<reason>}` and `gates` records every §C.1 decision. ~1 KB/curve.
3. **`bins.json`** — aggregated per 2 m/s speed bin (6–36 m/s) × direction: n, median/IQR of Δ and
   ratio, and the **inferred gain multiplier** (the factor that would have made median Δ = 0).
   **This table is what answers his 15/70-vs-13.5/26.82 anchor question from field data:** if the
   inferred multiplier is flat below 13.4 m/s and inflects there, the code anchors (30/60 mph) are
   the physical truth; if it keeps trending down through 13.4 toward 6.7 m/s, his spoken 15 mph
   anchor has empirical support and `gain_speed_lo_ms` deserves to move. Attach the same table
   from any drive run with the ⚠️ `gain_speed_lo_ms=9.0` experiment (DRIVE_REPORT Pass 2) — a
   direct A/B on his open question.
4. **`traces/`** — for ≤ 10 selected curves (every incident/exit-hold + the best clean exemplars):
   20 Hz-resampled `{t_rel, green_deg, yellow_deg, v, kappa_cmd, kappa_meas, wire_path_angle_rad,
   wire_mode, lane_pos_m, sat/clip/roc flags}`, ±5 s around the curve. ~50–100 KB each. This is
   his green/yellow chart, reconstructable, with the wire layer his own debug screen doesn't show.
5. **`adjustments.jsonl`** — the tuning history for the drive: shadow recommendations with their
   evidence digests, and what the human (or, later, the auto path) actually applied. The
   recommendation→application→next-drive-outcome chain is the "adjustments the system made
   automatically" the driver wants sent back, in its most useful form.
6. **`findings.md`** — freeform: the exit-hold tally and mechanism evidence, the
   `_desired_falling` threshold analysis, any new discrepancies, questions (anchors first).

Transport: whatever he prefers — the tarball is small enough for a Discord/forum attachment or a
GitHub gist; offer the schema doc alongside so he can parse it programmatically.

---

## F. Upstream to his fork?

**Yes — the observer, not the actuator. In three stages, data first:**

1. **Now (no code): send the field package + two questions.** (a) The anchor discrepancy — spoken
   15/70 mph vs coded 13.5/26.82 m/s, with our `bins.json` evidence and the note that `interp`'s
   clamp makes everything below 30 mph one knob (this decides whether `gain_speed_lo_ms` should
   move for everyone); (b) the exit-unwind findings — `_desired_falling`'s unreachable threshold
   and the v>9 deviation-clip exit trap, with the 15:06:37 trace — asking whether his Mach-E/F-150
   data shows the same post-apex hold. Data-first establishes credibility and gives him something
   only a second vehicle in the field can give.
2. **Next: offer the shadow analyzer as a tool.** It consumes only standard signals every
   BluePilot user already logs (`carControl.actuators.steeringAngleDeg`,
   `carState.steeringAngleDeg`, `modelV2`, `liveDelay` — his own debug panel's inputs), it encodes
   *his published method* (peaks, ±5° floor, never-near-stops, reduce-on-overshoot,
   suspect-undershoot, clicks), and it addresses a burden every one of his users carries — today
   they tune by watching a live chart *while driving*, which is exactly the interaction problem
   our driver revolted against, with a screen instead of a laptop. A post-drive "here's what your
   drive says about your factors, and why" report is a genuine contribution. Shape: a standalone
   script (`tools/` PR to BluePilotDev/bluepilot, or a companion repo he can bless) — pure
   observer, zero control-path changes, so it cannot destabilize his fleet and the review burden
   on him is small. His bp_portal (device web UI) would be a natural eventual host, but that's his
   call, not part of the offer.
3. **The auto-apply loop stays downstream (PNW-only), permanently-ish.** It depends on our JSON
   overlay write path (his tree uses Params + UI), and on risk-acceptance decisions (envelope
   width, freeze policy) that an author should not ship fleet-wide on our say-so. If his fleet's
   field data later validates the estimator broadly, graduating it upstream is his decision, made
   with data we helped generate.

**Fidelity flag (per the constraint):** the analyzer is a pure observer — zero deviation. Shadow
recommendations applied by the human between drives are equivalent to his own UI stepper — zero
deviation. A graduated *auto-apply between drives* still only moves his two exposed factors within
his own clip ranges — no change to his math, but the *policy* (machine-written factors) should be
disclosed to him as part of stage 2. **Mid-drive automatic writes would be a real deviation** — a
time-varying gain schedule is a new control mechanism his design doesn't have — and are rejected
(§A), which conveniently keeps the fidelity ledger clean.

---

## Constraints compliance (where this runs)

- **No UI:** none anywhere. All outputs are files; all interaction is SSH/scp/JSON, per directive.
- **No carState subscription in background processes:** the analyzer is **offline, post-drive**,
  over rlog segments (primary) — rlog already carries `carState`/`carControl` at 100 Hz,
  `modelV2`, `liveDelay`, and `sendcan` (the 982 wire decode comes from there too). **Offline
  quality ≥ live quality here** — it is the same bus data at the same rates, minus all onroad
  risk — so per the "prefer offline if comparable" rule, offline is the design, not a fallback.
  The existing 100 Hz `/data/dirk` capture script remains a session-scoped convenience for
  tuning days (with the pidfile + wall-clock fixes from DRIVE_REPORT); the analyzer accepts either
  input, and the capture script can retire once the rlog path is proven.
- **Nothing in the 20 Hz control path:** the control code is not touched by this design at all.
  The only device-side writes are the overlay JSON (parked) and the append-only history log.
- **Faithful-port rule:** the tuner adjusts exactly the factors Alan exposes, within his clamps,
  via the overlay that already exists for the human. No code deviation is introduced by shadow
  mode; the one would-be deviation (mid-drive writes) is explicitly rejected.

## Prerequisites from tomorrow's drive (what makes any of this possible)

Everything in DRIVE_REPORT's Pass 0/1 stands; ranked for this design specifically:

1. **Clean, ICBM-disabled, steady-speed hands-free laps on both corners with the wire logged**
   (sendcan 982: `LatCtlPath_An_Actl`, `LatCtl_D2_Rq`) alongside green
   (`actuators.steeringAngleDeg`) and yellow (`steeringAngleDeg`) — the first uncontaminated
   green/yellow samples ever collected, and the data that both validates the estimator's peak
   verdicts and disambiguates the exit-hold mechanism (clip vs blend vs lag). This single capture
   feeds §C validation, §D calibration, and §E's first honest package.
2. Capture hygiene: pidfile single-instance guard, wall clock per row (config-boundary attribution
   depends on it).
3. Lane-position (`modelV2.laneLines`) in the capture — the incident detector's outcome signal.
4. If pass structure allows: 2–3 laps at ~28 mph *and* ~18 mph on the same corner — the first
   matched pair across the sub-30 flat-clamp region, seed data for the anchor question.
