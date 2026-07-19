# Drive report — 2026-07-19 — Lightning: bp-7.0 faithful angle port, first tuning day (closed-loop corner laps)

**Car:** Ford F-150 Lightning · **Lateral:** `angle2pnw-faithful2` (Alan Polk bp-7.0 1:1 port), `FordAngleLateral=1`,
openpilot `3f8df8d` + opendbc `5eea429f` · **Long:** op-long OFF (stock ACC + ICBM curve taps), `CESMode=2`.
**Road:** the same closed test loop as 2026-07-18 (Ballard/Shilshole, ~47.663,-122.369), two corners lapped
repeatedly — a **LEFT curve** (~47.6633,-122.3690) and a **RIGHT curve** (~47.6636,-122.3698). 25 mph zone
(`spdLim` 11.2 m/s). All times **UTC** (device clock).
**Raw data (this folder):** `angle_capture2.jsonl` (100 Hz carState/carControl) · `ces_events.jsonl` (1 Hz CES/ICBM).

## Headline

1. **The 1.2 Low-Speed-Factor change is refuted by a matched pair on the same corner**: the left curve at
   28–29 mph, hands-free, **no ICBM active in either pass** — clean on 1.0 (13:45:24), lane excursion on 1.2
   (15:34:35). The metric that motivated 1.2 (the "delivery ratio") is structurally unsound (§ Delivery-ratio
   verdict) — it measured the controller's own error correction and its own 9 m/s clip gate, not PSCM delivery.
2. **The 100 Hz trace of the departure (15:06:37) shows the failure is at curve EXIT, not entry**: tracking during
   entry was fine (ratio 0.79–0.99); at the apex the planner unwound (`curv_cmd` −0.0109 → −0.0028 in 1 s, then
   sign-flip) while the truck **kept turning at full curvature for >1 s** → deep drift into the oncoming lane →
   driver grab + brake 31→10 mph. Which internal stage held the turn (predicted-curvature blend, the v>9
   deviation clip, or PSCM lag) is **unobservable in today's data** — the entire strategy pipeline between
   `actuators.curvature` and the wire is unlogged.
3. **ICBM confounded every low-speed lateral observation**: it braked *inside* the curve on every right-corner
   pass (28→16 mph mid-corner), and at 15:36:12 a mapV noise spike (6.6→15.7 m/s) made it **throttle back UP
   mid-curve** exactly when `strAng` spiked to −48°. `AutoSpeedReduce` was **not** a factor (param 0; `vSet`
   never clamped toward `spdLim`).

---

## Timeline and config actually in force (derived, not assumed)

Two corrections to the working assumptions:

- **The tuning JSON is NOT read only at card init.** `lateral_angle_pnw.py` re-polls
  `/data/pnw/angle_tuning.json` every `_TUNING_RELOAD_CALLS=100` strategy calls (20 Hz) ≈ **every 5 s,
  mid-drive**, and `load_angle_tuning()` re-reads the file with no caching. Edits take effect ~5 s after the
  file write, no restart needed.
- Therefore the config boundary is the **file-write time**, not card start: `/data/pnw/` dir mtime shows
  `angle_tuning.json` was **created 14:58:40** (the 1.2 edit, while parked between S0 and S1) and
  **edited in place 15:36:47** (the revert to 1.0; file mtime 1784475407).

| Session | Time (UTC) | low_speed_curv_factor in force | Evidence | 100 Hz capture |
|---|---|---|---|---|
| **S0** | 13:22:15–14:59:16 | **1.0** (no overlay file → pure bp-7.0 defaults) | file created 14:58:40, after all S0 driving | **none** (capture started 15:02:31) |
| — | 14:58:40 | file created with 1.2 (parked) | `/data/pnw` dir mtime | — |
| **S1** | 15:00:36–15:29:46 | **1.2** | file predates session; ≤5 s reload | seg0, 15:02:31–15:29:46 |
| — | 15:30:46 | manager/reboot | `ps lstart` | — |
| **S2** | 15:31:04–15:42:49 | **1.2 until ~15:36:52**, then **1.0** | revert written 15:36:47 + ≤5 s reload | **GAP 15:29:46–15:38:18**, then doubled |
| — | 15:36:47 | revert to 1.0 written | file mtime | — |

Driver claim (d) — "the right curve was still on 1.2" — is **TRUE** (the (d) curve at 15:36:03–15 predates the
revert write by 44 s) but for the wrong reason: the revert would have applied within ~5 s of the write, restart
or not. Everything driven after ~15:36:52 was on 1.0 (only pressed/low-speed corners; no clean hands-free data).

**Gain actually commanded at the plateau** (sharp curves, |κ|>0.001): `1.30 × factor` flat below 13.5 m/s
(30.2 mph). On 1.0 → **1.30**; on 1.2 → **1.56**. At the 31 mph apex of the departure: 1.54 (vs 1.29 baseline)
— the 1.2 sessions commanded ~20 % more path_angle at exactly the speed where both incidents happened.

### Capture-tooling defects found (fix before tomorrow)
- **Gap**: no 100 Hz data 15:29:46→15:38:18 — covers both S2 incidents (c2, d2). 1 Hz ces only.
- **Duplicate writers**: from 15:38:18 and 15:39:04 TWO capture instances appended interleaved rows (t offset
  46 s). Single-instance guard needed (pidfile or `pgrep`).
- No wall clock per row (only a session-start marker) and none of the strategy internals (see § Telemetry).

---

## The two corners, all passes (GPS-matched to ±25 m)

### LEFT curve ~47.6633,-122.3690 (the departure corner)
| Time | Factor | Speed | Hands-free | ICBM | Outcome |
|---|---|---|---|---|---|
| 13:31:53 | 1.0 | 44→32 mph | yes (0/9) | active (braking in) | **clean** — driver's "fast pass, held up remarkably" (a) |
| 13:45:24 | 1.0 | **28 mph steady** | yes (0/10) | **none** | **clean** ← the matched baseline |
| 15:03:42 | 1.2 | ~37–46 mph | yes (0/9) | 1 row | clean (mostly straight run-up in capture) |
| 15:06:37 | 1.2 | 46→**31** mph | until grab | **braking through entry+apex** (icbmT from 15:06:35, 46→30) | **LANE DEPARTURE into oncoming** (c) — driver grab at 15:06:44, brake 31→10 mph |
| 15:34:35 | 1.2 | **29 mph steady** | yes (0/10) | **none** (curve never map-detected; vision only at 55 m) | **excursion into oncoming** (c2) — strAng ramp +15→+51°, collapse, light press after |

### RIGHT curve ~47.6636,-122.3698 (the "runs wide / too far right" corner)
| Time | Factor | Speed | Hands-free | ICBM | Outcome |
|---|---|---|---|---|---|
| 13:30:56 | 1.0 | 26→16 mph | mostly (3/13) | active, vSet→7.6 m/s | driver: ran wide (b) |
| 13:33:04 | 1.0 | 29→16 mph | yes (0/14) | active 13/14 s | (b) family |
| 13:46:38 | 1.0 | 27→19 mph | partial | active | (b) family |
| 15:04:55 | 1.2 | 31→16 mph | yes (0/14) | active 11/14 s | — |
| 15:08:11 | 1.2 | 22→16 mph | yes (1/14) | active 11/14 s | oscillatory tracking (ratio 0.57–1.18 @100 Hz) |
| 15:36:03 | 1.2 | 28→16 mph | yes (0/13) | active; **throttled UP mid-curve at 15:36:12 on a mapV 6.6→15.7 noise spike**, strAng spiked −48° at 15:36:13 | "too far right" (d) |

Every single right-corner pass had ICBM decelerating (or on d, re-accelerating) **inside** the curve. There is
no clean constant-speed low-speed pass in the whole day. The "wide at <20 mph" observation is therefore
**confounded**: `path_angle = κ·v·factor` scales with v, so mid-curve decel directly shrinks the command while
the planner re-inflates κ with ~300 ms round-trip lag (measured, § below).

---

## 100 Hz anatomy of the departure (c), 15:06:35–15:06:48, factor 1.2

```
   t(s) v_mph  strAng   curv_cmd  kappa_meas ratio  pressed
  245.0  45.1   +6.0   -0.0017   -0.0008    0.50   0     entry begins (ICBM braking underway)
  247.0  42.4  +28.5   -0.0069   -0.0055    0.80   0
  249.0  37.9  +37.2   -0.0090   -0.0074    0.83   0     entry tracking: lagged but stable
  251.0  33.7  +42.0   -0.0121   -0.0091    0.75   0
  252.0  31.7  +50.7   -0.0109   -0.0108    0.99   0     apex: caught up
  253.0  31.1  +44.3   -0.0028   -0.0110    3.98   0  ⛔ planner unwinds 4x; TRUCK KEEPS TURNING
  254.0  30.7  -23.6   +0.0047   +0.0011    —      1     command sign-flips right; driver grabs
  255.0  21.5  -21.2   +0.0057   +0.0052    —      1     driver emergency-brakes 31→10 mph
```

- **Entry**: no runaway; achieved curvature tracks command at 0.75–0.99 through the ramp. The 1.56/1.54 gain
  did not produce visible entry oversteer in this (closed-loop) view.
- **Exit**: at t=253 the planner's desired curvature collapsed 0.0109→0.0028 and then flipped sign, but measured
  curvature held at 0.0110 for >1 s. The car "plowed on" into the inside (= oncoming lane on a left curve).
- **Candidate mechanisms** (all between `actuators.curvature` and the wire, ALL UNLOGGED today):
  1. **The v>9 deviation clip** (`kappa_cmd = clip(requested, measured ± 0.002)`, bp-7.0's own code, line 432):
     at exit it becomes a trap — the command may not drop more than 0.002 below the *measured* curvature, the
     PSCM holds the turn because we still command it, measured stays high, the clip keeps the command high.
     Unwinding proceeds at ~0.002 curvature per plant-response interval; from κ=0.011 with ~300 ms lag that
     predicts ~1.2–1.5 s of continued turning — **matching the observed hold**. Note the clip's gate is
     `v_ego > 9`: below 9 m/s exits are free, above it they are rate-trapped.
  2. **Predicted-curvature blend b=0.5** with the exit-collapse effectively dead: `_desired_falling` requires a
     0.010 curvature drop **per 20 Hz call** (= 0.2 κ/s) — real planner unwinding is ~0.0004/call, so the
     collapse never triggers on real exits. Verified **identical in Alan Polk's bp-7.0 source** (line 395) —
     if this is a bug, it is *his* bug, faithfully ported; worth reporting to him.
  3. Plain PSCM lag (~300 ms measured lag, cross-correlation cmd→yaw, corr 0.958).
  The data cannot distinguish 1–3. The gain factor (1.2) plausibly worsened the outcome (20 % more commanded
  path_angle at the apex → more stored rotation to shed through whichever mechanism), which is consistent with
  the same corner surviving at 1.0 — but the *mechanism* is exit-unwind, not "too much steady-state gain".

---

## Delivery-ratio verdict: REFUTED as a measure of PSCM delivery

Replication on seg0 (hands-free, latActive, |curv_cmd|>0.002, N=42,950 frames):

| v band (m/s) | ratio median (IQR) | lag-compensated | cmd-vs-meas gap >0.002 |
|---|---|---|---|
| 6–9 | 0.82 (0.73–0.90) | 0.82 | 47 % of frames |
| 9–12 | 0.93 (0.85–1.04) | 0.95 | 5 % |
| 12–15 | 0.97 (0.87–1.08) | 1.02 | 7 % |
| 15–20 | 0.90 (0.80–1.01) | 0.97 | 18 % |

The banded pattern replicates — but it does not mean what it was taken to mean:

1. **The 9 m/s step is the controller's own gate, not a plant property.** The strategy clips its executed
   command to `measured ± 0.002` **only above 9 m/s** (`if v_ego > 9`, bp-7.0 line 432). Above 9, executed
   command and measurement are chained together by construction, so measured/desired converges to ~1 in any
   sustained curve. Below 9, the clip is off and the planner free-leads. The "PSCM under-delivers below 9 m/s"
   reading attributed the code's own speed gate to the actuator.
2. **Numerator and denominator are not cause and effect.** `curv_cmd` (= `actuators.curvature`) is the
   planner's *desired* curvature including live error correction — exactly the confusion Alan Polk warns about
   (desired-vs-predicted-curvature article). Whenever the car is mid-correction (all short city corners,
   always), desired deliberately leads achieved and the ratio reads <1 with a perfectly healthy actuator.
3. **The executed command is not even proportional to `curv_cmd`.** Between `actuators.curvature` and the wire
   sit the predicted-curvature blend (b=0.5 with modelV2 at VLT lookahead), the lane-change scaling, the
   deviation clip, the gain stage, the PSCM/DBC clamps and the soft ROC — **none of it logged**. A ratio built
   on the pipeline's *input* cannot distinguish "PSCM under-delivered" from "the pipeline commanded
   less/more than assumed". With b=0.5, a model prediction that unwinds earlier than the planner (normal at
   low-speed corner exits/entries) alone drags the executed command ~15–25 % away from `curv_cmd`.
4. Secondary confounds all push the same way at low speed: 300 ms lag over 3–5 s corner transients
   (entry-heavy sampling), yaw/v noise amplification, and ICBM changing v mid-curve in **every** low-speed pass.

**Conclusion: the ~0.8 low-speed ratio was never evidence for raising the low-speed gain.** The 1.2 change it
motivated made the same corner strictly worse (matched pair 13:45:24 vs 15:34:35). The right metrics, in order
of value: (i) wire `LatCtlPath_An_Actl` vs achieved steering angle (true actuator delivery), (ii) modelV2
lane-line lateral position (true outcome/lane error), (iii) steady-state-only mid-curve sampling with
lag alignment — all require the telemetry additions below.

## ICBM / speed-control findings (secondary but real)

- ICBM braked mid-curve (not before) on every low-speed pass — same late-onset failure documented in
  `drives/2026-07-18/lightning-icbm-curve/DRIVE_REPORT.md` (noisy map resolution at entry).
- **New failure mode 15:36:12 (d)**: mapV noise spike (6.6→15.7 m/s) mid-curve → icbmT jumped 7.7→12.96 →
  stock ACC **accelerated inside the corner** (vSet 7.6→12.1) at the moment of maximum steering (−48°). A
  speed step-up mid-curve demands a proportional path_angle step from the lateral controller. File as an ICBM
  bug: hold/floor the target until curve exit (curvePct decay), ignore upward mapV revisions mid-curve.
- On the blind left corner (c2 15:34:35) the map never flagged the curve (curvePct 0 until 55 m, vision-only
  from 5 s out) → ICBM never fired → 29 mph entry un-slowed. On (c) it fired late and braked through the apex.
- **AutoSpeedReduce: not involved.** Param is 0; `vSet` sat well above `spdLim` (up to 21 vs 11.2 m/s) all
  day and every reduction tracks `icbmT`/`stockSet` exactly. The driver's suspicion is refuted for today.

## Agree / refute / unprovable (the analyst's prior conclusions)

| Claim | Verdict |
|---|---|
| "Delivery ratio ~1.0 above 9 m/s, ~0.8 below" | **AGREE numerically** (replicated: 0.82 / 0.90–0.97) |
| "…therefore the PSCM under-delivers at low speed" | **REFUTE** — the 9 m/s step is the code's own deviation-clip gate; the ratio conflates controller correction, blend, gain, clip and lag with plant delivery (§ verdict) |
| "…therefore raise Low Speed Factor to 1.2" | **REFUTE** — matched-pair on the left corner (1.0 clean / 1.2 excursion, both ~29 mph, hands-free, no ICBM); both incidents occurred at the speed where 1.2's plateau bites hardest |
| "(d) was still on 1.2" | **AGREE** (predates the revert write by 44 s) — but the premise "edits need a truck restart" is **wrong**: the overlay re-polls every ~5 s |
| "(b)/(d) low-speed wide and (c) 30 mph oversteer are both the flat-below-30 mph plateau" | **PARTLY** — (c)/(c2) happened at the plateau's top edge (13.4–13.9 m/s) where 1.2 commands 1.54–1.56, and the same corner passes at 1.30; but the 100 Hz mechanism is **exit-unwind failure**, not steady oversteer, and (b) is **UNPROVABLE** today (every <20 mph pass ICBM-confounded, no 100 Hz for S0, no lane-position signal) |
| "PSCM under-delivery is real at low speed" | **UNPROVABLE without wire telemetry** — cannot separate command shortfall from delivery shortfall when the commanded wire value is unlogged |

## RECOMMENDATION — tomorrow's session (one change per pass)

**Config is already correct for pass 1: the revert to 1.0 is on the device and verified.** Do not touch the
gain surface until the wire is observable. Deviations from Alan Polk's literal bp-7.0 are marked ⚠️.

**Pass 0 — before driving (parked): fix the capture, add wire + lane telemetry.** No control-code change, no
reboot needed for the capture script (it is a standalone /data/dirk script):
- Single-instance guard (pidfile) + wall-clock timestamp on every row.
- Subscribe `sendcan`; decode addr **982 (`LateralMotionControl2`)** with `ford_lincoln_base_pt.dbc`; log
  `LatCtlPath_An_Actl` (rad; **wire value = −internal path_angle**, carcontroller negates all four signals),
  `LatCtl_D2_Rq` (mode; **0-pulses reveal human-turn overrides and stall/hand-off blips**, 300 ms),
  `LatCtlPathOffst_L_Actl`, `LatCtlCurv_NoRate_Actl` + `LatCtlCrv_NoRate2_Actl` (confirm c2/c3 stay 0).
- Subscribe `modelV2`; log `laneLines[1].y[0]`, `laneLines[2].y[0]` (inner line lateral offsets = direct lane
  position) and `meta.laneChangeState`.
- Subscribe `liveDelay`; log `lateralDelay` (VLT t_base input).
- Keep 1 Hz ces as is. Chain captured becomes: planner wish → wire path_angle → steering angle → lane position.
- **Disable ICBM for the lateral passes** (CESMode=0 for the session, or drive with ACC set manually at target
  speed): every low-speed observation today was speed-confounded. Re-enable afterwards.

**Pass 1 — baseline re-characterization on 1.0 (no tuning change).** Left corner at steady ~28 mph and the
right corner at steady ~18 mph, 2–3 laps each, hands hovering. Questions the new telemetry answers directly:
- Right corner "wide": is wire path_angle short of κ·v·1.30 (command-side) or is the wire full and steering
  short (PSCM delivery) or is steering full and the line still wide (planner/model)? Only then pick a knob.
- Left corner exit: watch wire path_angle at apex→exit — does it track the planner's unwind or hold (clip/blend
  trap)? Any `LatCtl_D2_Rq=0` pulses mid-curve = blips firing invisibly.

**Pass 2 — only if pass 1 shows genuine low-speed command shortfall:** ⚠️ `gain_speed_lo_ms: 13.5 → 9.0`
(single change). This tapers the plateau's top so 30 mph (13.4 m/s) drops 1.30→~1.21 while <20 mph keeps 1.30.
Deviation from Alan Polk's literal [13.5, 26.82] breakpoints — deliberate, reversible, protects the
demonstrated failure speed before any boost is added. Risk: slightly less authority at 30–45 mph (watch for
wide exits on the fast left pass; abort criterion = any hands-free pass needing correction that pass 1 didn't).

**Pass 3 — only if low-speed wide persists after pass 2:** ⚠️ `low_speed_boost: 1.30 → 1.40` (single change).
With lo_ms=9 this gives <20 mph ≈ 1.40 while 30 mph stays ≈ 1.29 (≈ today's clean baseline). Never raise
`low_speed_curv_factor` for this purpose again — it cannot boost <20 mph without also boosting 30 mph.

**Do NOT tomorrow:** re-raise `low_speed_curv_factor`; touch `path_angle_blend_ratio` or the deviation clip
(candidate exit-trap mechanisms — change nothing there until the wire data confirms which one holds the turn;
a clip change is also a ⚠️ deviation and near-safety-adjacent); run tuning passes with ICBM enabled.

**For the Alan Polk write-back** (deviations + findings to report): (1) faithful port tracked correctly all
day — no sign issues, entry tracking 0.75–1.0; (2) suspected exit-unwind hold on sharp low-speed corners —
his `_desired_falling` exit-blend collapse threshold (0.010/20 Hz-call) appears unreachable for real planner
unwind rates, and his v>9 deviation clip can pin the command to the measured curvature at exit; does his
Mach-E/F-150 data show the same? (3) our JSON overlay + [13.5→9.0] breakpoint experiment if pass 2 runs.

## Proven vs unproven (summary)

**Proven today:** faithful port steers the correct direction and tracks entries; 1.2 made the left corner
worse at constant speed with no ICBM (matched pair); the failure mode at 30 mph is exit-unwind hold; tuning
JSON hot-reloads in ~5 s; ICBM braked/accelerated mid-curve on every low-speed pass; AutoSpeedReduce inactive;
~300 ms command→yaw lag.
**Unproven / blocked on telemetry:** whether low-speed "wide" (b) survives without ICBM decel; whether the
PSCM under- or over-delivers commanded path_angle anywhere (wire unlogged); which mechanism (clip / blend /
PSCM lag) holds the turn at exit; whether stall/hand-off blips fired today at all.
