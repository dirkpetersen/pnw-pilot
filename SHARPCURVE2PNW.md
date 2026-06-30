# SHARPCURVE2PNW — earlier + smoother curve slowdown (blind sharp-curve take-control fix)

Branch: **`sharpcurve2pnw`** (off `4devpnw`). Touches only `selfdrive/controls/lib/vtsc_pnw/`
(constants + pure core + controller + tests). **No new params** — rides the existing CES master
(`CESMode`); behavior-neutral when CES is Off. **All changes only ever REDUCE speed.**
**NOT yet driven** — build/tested + Gemini-reviewed (gemini-pro-latest, 2 rounds), pending an on-road drive.

## Problem
Recurring sharp-curve **"TAKE CONTROL"** beep on the blindest curves at high set speed (I-90 descents):
the slowdown started too late and the driver had to intervene. Root cause = lookahead, not braking:
pfeiferj **mapd publishes a fixed 500 m path** (`MIN_WAY_DIST`), but VTSC scanned only `v_ego*12 s`
(~370 m @ 70 mph) and CES ~310 m — throwing away ~130–190 m (~6 s) of available warning. The braking
distance to slow 70→30 mph at a gentle decel (~330 m) FITS in 500 m but NOT in the old ~370 m horizon.

## Driver model (EV / Model S)
"On the freeway braking should almost never be required; as soon as you decelerate, regen [recoup]
makes it much slower." Lifting the accelerator gives ~0.2 g regen. So the fix leans on **lookahead +
regen-coast**, not friction braking. Goal: get off the gas early so the curve **ENTRANCE is the
slowest point**, then accelerate out (sometimes before the apex).

## What changed (4 parts, all in VTSC)
1. **Distance-based lookahead.** Scan the FULL ~500 m mapd publishes (`MAP_SOURCE_HORIZON_M`) and pick
   the most-binding upcoming curve **by decel envelope** (`most_binding_map_curve`), not nearest /
   min-speed. A far sharp curve has a high (non-binding) envelope until close enough; the envelope gates
   when braking actually starts, so scanning farther only buys earlier detection, never premature slowing.
2. **Regen-coast deceleration.** Normal commanded decel is capped to EV regen authority
   `REGEN_A_DECEL = 2.0 m/s²` (coast/regen, no friction brake). A genuinely **sharp** map curve
   (`SHARP_CURVE_V = 30 m/s`) that regen alone can't reach before its **entrance**
   (`required_decel(v_ego, v_curve, d_entrance) > REGEN_A_DECEL`, where `d_entrance = d_apex −
   v_ego*APEX_FINISH_S`) raises the rate-limit ceiling to `SHARP_A_DECEL_MAX = 2.8` — **last resort
   only**, still bounded (never a slam).
3. **Apex timing → slowest at entrance, accelerate before apex.** `APEX_FINISH_S 1.2→2.5`,
   `HOLD_TTA_S 1.2→2.5`, `APEX_TTA_S 0.4→1.2`. The `at_safe` gate still guards RELEASE, so it only
   accelerates pre-apex once actually slowed to curve-safe speed (lateral margin preserved).
4. **Twisty-DESCENT base trim** (`twisty_section_cap`). ONLY when ≥3 packed binding curves within the
   horizon **AND** the road is descending (`pitch < TWISTY_DESCENT_PITCH`, from
   `carControl.orientationNED[1]`): hold a lower base cruise (floor `TWISTY_MIN_FACTOR = 0.82`) through
   the section so we don't re-accelerate to full set between blind curves. **Flat twisty keeps full
   speed** (per-curve VTSC handles it) → no speed lost where it isn't needed.

## Speed impact (driver asked: don't lose speed overall)
- Curve-safe (slowest) speed is UNCHANGED (`A_LAT_TARGET=2.2`) — never slows more at a curve than before.
- Earlier lookahead replaces late hard braking with early gentle regen → equal-or-faster, smoother.
- Accelerating before the apex regains speed sooner → faster out of curves.
- The ONLY deliberate reduction is the twisty-DESCENT trim (winding downhill) — exactly the requested
  "lower the set on twisty descents." Everywhere else: no speed loss.
- The ≥1 mph curve cue (`CONFIDENCE_CUT`) on every real bend is untouched — still applies.

## Gemini review (gemini-pro-latest)
Round 1 found 5 issues; #1 was a false positive (the `at_safe` release guard IS present). Fixed: map-fold
threshold now uses the true SET speed (not the twisty-trimmed cruise); last-resort trigger measures decel
to the ENTRANCE not the apex; full 500 m scan (removed the truncating helper); MTSC scale+clamp folded
into selection. Round 2 verified those safe and caught one new CRITICAL — `is_sharp` must be classified on
the RAW target, not the 1.12×-scaled one (else a 28 m/s sharp curve inflates to 31.4 and loses its sharp
flag) — fixed. The unscaled twisty-descent base is intentional (more conservative downhill).

## Test / deploy
`PYTHONPATH=<worktree> python3 -m pytest selfdrive/controls/lib/vtsc_pnw/tests/test_vtsc_pnw.py
-k "not state_machine and not disabled_is_neutral" --noconftest -o addopts=""` → 36 passed.
Pure-Python (no `params_keys.h` change → no `params_pyx` rebuild). Deploy when parked / `IsOnroad=0`;
needs `VtscMapCurves` ON + a non-Off `CESMode`. **Validate on a real twisty/descent drive before trusting.**
