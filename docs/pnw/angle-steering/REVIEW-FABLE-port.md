# REVIEW-FABLE — adversarial review of `angle2pnw-faithful2` (pnw-opendbc 05938869)

Reviewer: Fable (first of two independent reviewers; Gemini is the second).
Scope: full diff `master-pnw..angle2pnw-faithful2` in `/home/dp/gh/comma/pnw/pnw-opendbc`, diffed
hunk-by-hunk against Alan Polk's bp-7.0 ground truth in `/home/dp/gh/comma/sunny/bluepilot`
(branch `bp-7.0`, tip `19858f2888`). Everything below marked **[EXECUTED]** was verified by
actually running code, not by reading it.

---

## 1. Sign convention — independently re-derived and EXECUTED

I did not trust `SIGN-CONVENTION-TRACE.md`; I re-derived the chain and then ran the port's actual
packer and decoded the bytes with both the DBC parser and a transcription of ford.h's bit
extraction.

**Derivation.** The file-internal convention is pinned by
`current_curvature = -CS.out.yawRate / max(v_ego, 0.1)`
(`lateral_angle_pnw.py:539`), which is byte-identical to his line 430 AND to the
already-proven upstream/curvature-mode Ford code (`apply_ford_curvature_limits`,
`apply_ford_curvature_limits_ext`). `CS.out.yawRate` is a raw passthrough of `VehYaw_W_Actl`
(`carstate.py:36`, identical in both trees, no negation). For the deviation clip to work — and it
demonstrably works on real Fords, upstream, for years — `actuators.curvature` and
`-yawRate/v` must share a sign. Everything downstream is sign-preserving:
blend (positive weights), lane-change factor (positive), deviation clip (clip, cannot flip sign),
`path_angle = kappa_cmd * v_ego * curvature_factor` with `curvature_factor` a positive gain,
saturation clamp and soft ROC (magnitude operations). So internally `path_angle` always shares
`kappa_cmd`'s sign, and `bp_kappa_cmd` shares `actuators.curvature`'s sign. **There is no negation
inside the strategy file — confirmed by a normalized full-body diff (see §2).**

Negation happens exactly twice, in `carcontroller.py`, both verbatim from his `carcontroller.py`
(his lines 200–208, 224): (1) all four LMC/LMC2 wire signals together
(`-path_offset, -path_angle, -apply_curvature, -curvature_rate` — never a subset, so c1 and c2 can
never diverge from each other), (2) `shadow_curvature = -bp_kappa_cmd`.

**[EXECUTED] Packer proof (right-hand curve, file-internal convention: desired = +0.01,
yawRate = −0.10 rad/s, v = 20 m/s, internal path_angle = +0.113 rad):**

- `create_lat_ctl2_msg(..., -0.113, ...)` produced bytes `16017d0c1a008006`.
- DBC round-trip decode: `LatCtlPath_An_Actl = -0.113`, `LatCtlCurv_No_Actl = 0.0`, mode = 1. ✓
- ford.h's own bit extraction (`raw_path_angle = ((data[3] & 0x1F) << 6) | (data[4] >> 2)`,
  branch ford.h line 799): desired_path_angle = **−226 CAN units = −0.113 rad** exactly;
  desired_curvature = 0. ✓ (Note: an earlier −613 figure in my session was my own transcription
  error of the shift widths, not a code defect.)
- LKA message: engaged bit = 1; shadow raw int16 = −5000 → ×0.05 = **−250 CAN units**.
  Panda's independent `angle_meas` for the same physical state (`raw_yaw/v × 50000` =
  −0.10/20×50000) = **−250 CAN units**. **Equal — the
  `ford_shadow_curvature_error_check` corroboration passes for a genuinely tracking car and would
  hard-fail (+250 vs −250) if the shadow negation were dropped or doubled.** ✓
- With angle mode off, `fordcan_pnw.create_lka_msg(..., False, 0.0)` is **byte-identical** to
  stock `fordcan.create_lka_msg` output. ✓

**Verdict on the trace doc:** I agree with `SIGN-CONVENTION-TRACE.md`'s conclusion and its two
negation points. One reservation (finding m3): its Step-0 *absolute* labels ("positive internal =
RIGHT") rest on assuming Ford's raw `VehYaw_W_Actl` follows ISO positive-left, which conflicts
with openpilot's ISO positive-LEFT convention for `actuators.curvature` — one of the two
assumptions must be wrong, and neither is verifiable from code alone. This does not affect
correctness: every stage is *relative*, anchored to the proven upstream curvature path and to Alan
Polk's shipping transform (identical code), and the shadow-vs-angle_meas equality holds label-free.

## 2. Fidelity vs his `lateral_angle_ext.py` — hunk-by-hunk

**[EXECUTED]** I mechanically normalized (comments/blanks stripped) his entire
`update_angle_strategy` body against the port's `update()` body and diffed. The ONLY differences:

1. method rename (manifest a5), 2. `_ensure_lateral_curv_initialized` → `update_angle_params(None)`
(a4/a7 — his shim is a literal `pass`, verified), 3. dropped dead `LP = self.lp` (see m2),
4. lane-change constants localized with identical values `[4.4, 40.23]` / `0.95` (a3 — verified
equal to `LateralCurvExt.__init__` lines 147–148 in bp-7.0), 5. gain-interp literals
`[13.5, 26.82]` / `1.30` / `[0.0007, 0.001]` replaced by tunable attributes **with identical
defaults** (e1/e2).

Everything else is line-identical, including:
- **all ~15 state resets in each of the three early-return paths** (latActive-false, human-turn,
  stall-blip) — checked attr-by-attr against his lines 223–249 / 260–291 / 310–333; complete,
  including `_desired_curvature_last` refresh and `precision_type = 1`;
- VLT incl. the 0.1–0.15 liveDelay clip, direction-aware `_kappa_entering`, `[25,55 mph]` speed
  taper, `0.005/0.020` kappa taper;
- exit-biased blend collapse `b * 0.25` with `_desired_falling` threshold 0.010, `_dbc_sat` at 90%
  of ±0.5/0.5235, `_pscm_lim` via `getattr(CS,'lat_ctl_lim_stat',0)` — **both trees lack that CS
  attr, so both evaluate to 0: exact parity** (verified by grep of both carstates);
- deviation clip (v>9 gate, CURVATURE_ERROR=0.002 both trees), PSCM sat clamp incl.
  `_PSCM_SAT_UNWIND_RATE=0.02` sign-preserving unwind, DBC clamp, soft ROC `[9,10,15,25] →
  [0.055,0.055,0.0425,0.009]`;
- **state machine**: human-turn detector (`human_turn_pnw.py` is textually identical to his
  `human_turn.py` in every constant and line: 45.0°/1.5 s/3.0 s, pre-turned discriminator,
  hold-on-no-update semantics); press-blip (0.5 s falling edge, cooldown+frames-left guards, timer
  zeroed after evaluation); stall blip (gap > 2×CURVATURE_ERROR, **off-frames hold the accumulator**
  — accumulate only when `bp_curvature_deviation_limited && cooldown<=0` but never reset inside
  `_stalled`, exactly his), 6-frame pulse, cooldown set on last pulse frame, 3-blip episode cap,
  episode ends on press or half-gap closure, **human turn resets the whole blip state including
  press_timer** (his "only press time after the latch releases earns a pulse" rule) — all identical.
- Constants table in `ALAN-POLK-PORT-DEVIATIONS.md` §3 spot-audited: all entries check out,
  including `BP_ANGLE_LIMITS` (identical tables in `values_pnw.py` vs his `values_ext.py`),
  STEER_STEP=5, LKA_STEP=3.

His stale docstring (`½·κ·d_ref`) vs actual code (`κ·v·factor`): confirmed — `d_ref` is computed
and never consumed in HIS file; the port reproduces the dead computation. The port implements his
code, correctly.

## 3. Findings

### BLOCKER — none.

### MAJOR

**M1 — UNLISTED deviation: LKA `angle_mode_engaged` semantics.**
`carcontroller.py` (angle branch, `self._angle_mode_engaged = lat_active`, i.e.
`CC.latActive && !human_turn && !stall_blip`) vs his line 223:
`angle_mode_engaged = (not disable_BP_lat_UI) and (primary == angle)` — **independent of latActive
and the overrides**. The port therefore drops the corroboration bit to 0 during every human-turn /
stall-blip episode and while disengaged; his keeps it 1 whenever angle mode is selected.
This difference appears NOWHERE in `ALAN-POLK-PORT-DEVIATIONS.md`, whose own contract says "every
difference not listed there is a bug, not a deliberate choice."
Concrete scenario: on the first mode-1 LMC2 frame after an override/blip release, panda still holds
flag=0 (LKA is 33 Hz, LMC2 20 Hz, and the LMC2 is packed before the LKA in the same can_sends), so
that frame is checked under CURVATURE-mode rules: `steer_angle_cmd_checks(0, enabled, ...)`
mid-curve is a guaranteed angle-error violation (desired 0 vs angle_meas beyond ±0.002). Today it
is absorbed ONLY because the reset-bypass latch is armed by the override's neutral frames and the
violation is classed ROC (latch-relaxable) — a **hidden coupling between this unlisted deviation
and the latch amnesty**. If the latch hardening is ever tightened further (it has been touched
twice already), this becomes 1–2 blocked steering frames at the exact moment control resumes
mid-curve. Direction of the deviation is strictly *tighter* than his (engaged=0 → stricter panda
checks), and mid-curve fresh engagement has the same 1–2-frame exposure under BOTH semantics
(ford.h's c6 hardening forces the flag false while disengaged regardless), so I found no unsafe
scenario — but it is a real, unlisted behavioral difference in the safety-corroboration channel.
**Required: either restore his semantics (`engaged = _latext_angle is not None`, gated only on the
builder being live) or add this deviation to the manifest with the latch-coupling analysis above
and a test pinning the release-frame behavior.**

### MINOR

**m2 — dropped `LP = self.lp` (his line 338) is unlisted**, and manifest a2's claim that his
strategy "never reads liveParameters" is technically wrong (it reads-and-discards). Zero behavior
delta; fix the manifest wording.

**m3 — `SIGN-CONVENTION-TRACE.md` Step-0 absolute left/right labels** (see §1). The conclusion is
unaffected, but the doc states "positive desired = RIGHT" as derived fact when it actually depends
on unverified Ford sensor polarity; openpilot's ISO positive-left convention for
`actuators.curvature` suggests the labels are flipped. Doc fix only.

**m4 — manifest e2 / module docstring claim bp-7.0 reads `FordPathAngleBlendRatio` /
`FordVLTExtraMax` from Params.** It does not — grep of bp-7.0 shows those names exist only in
comments; his `update_angle_params` reads exactly three keys. Defaults are identical (0.50 / 0.10)
so no behavior delta, but the manifest asserts a params-read that doesn't exist in his code.

### NOTE

**n1 —** shadow int16 at 1e-6 scale saturates at ±0.0328 1/m; inherited byte-for-byte from bp-7.0
(same scale, same clamp); unreachable in the checked regime (check only runs >9.9 m/s where the
Python clip binds kappa to measured ±0.002).
**n2 —** the three cereal-dependent engaged tests skip silently in the plain pnw-opendbc venv
(14 passed / 3 skipped there); the writer's "17 passed" holds only from the pnw-pilot venv. Deploy
gates must keep running them from that venv.
**n3 —** pnw-pilot side: the shipped opendbc pin in the deploy-batch commits (`8796ceaf`) predates
`05938869`; shipping this branch requires the companion push + a fresh pin bump, and the
`FordAngleLateral` key must be in the shipped params registry (present per the `angleenable`
branch's commit message — key presence **UNVERIFIED** by me directly). Deploy-process note, not a
branch defect (a missing key fails safe: `angle_lat` stays False).
**n4 —** `PnwVehicle` now does a guarded `Params().get_bool` on every construction for every Ford;
failure path leaves the feature off. Negligible, fails safe.

## 4. Claims verified [all EXECUTED unless noted]

- **ford.h byte-identical to branch `angle2pnw`'s ford.h**: `git diff angle2pnw..angle2pnw-faithful2
  -- opendbc/safety/modes/ford.h` is EMPTY. Claim TRUE. I additionally read the full
  master-pnw→branch diff: the value/ROC violation split, controls_allowed-gated flag update, and
  separate `FORD_PATH_ANGLE_LIMITS_ANGLE` table (his numbers ×1.02: 0.0561/0.04335/0.00918) are as
  described; curvature-mode behavior is preserved byte-for-byte when `angle_mode_active` is false.
- **Safety tests**: ran `opendbc/safety/tests/test_ford.py` on the branch —
  **123 passed, 96 skipped, 9000 subtests passed** (matches the writer's claim exactly); libsafety
  is compiled at test time from the branch's ford.h (fresh .os/.gcno artifacts confirm), so this
  genuinely exercised the reviewed C.
- **Car tests**: `opendbc/car/ford/tests/` — 44 passed, 7 skipped (opendbc venv); the angle-specific
  files — 17 passed under the pnw-pilot venv, including
  `test_angle_mode_takes_precedence_when_toggled_on` (the precedence fix) and
  `test_angle_mode_off_by_default`.
- **JSON overlay**: no-file path is provably default-equal — the defaults dataclass is constructed
  from the module constants + `PnwVehicle.angle_gain` (0.95, 0.95 Lightning) and every value equals
  his (checked against bp-7.0 source, not the manifest); loader never raises (all-exception guard,
  per-key clamp-or-default, NaN check, ordered-pair revert); reference JSON values all equal his
  constants; disk I/O is init + every 100th update() call (~5 s), nothing per-tick.
- **Gating / off-state**: `angle_lat` default OFF proven by test; with it off, `_latext_angle` is
  None, the dispatch/`_pcblend_enabled`/LKA call sites reduce to the master-pnw paths, and the LKA
  bytes are identical to stock (executed, §1). Construction-order precedence (angle before
  four_signal) verified in code and by its regression test.

## VERDICT

**PASS-WITH-FIXES** — must fix before ship: **M1** (restore his `angle_mode_engaged` semantics OR
add the deviation to `ALAN-POLK-PORT-DEVIATIONS.md` with the latch-coupling analysis and a pinning
test). m2/m3/m4 are documentation corrections that should ride along in the same commit; NOTEs need
no code change (n3 is a deploy-process reminder).
