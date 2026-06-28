# CES_I90.md — curve-braking learnings from the I-90 / Snoqualmie Pass drive

**Branch:** `ces-i90-2pnw` (off `4devpnw`, base `3d87fe68`).
**Status:** built + Gemini-reviewed (APPROVE WITH NITS), lint-clean, **committed to this branch only —
NOT merged into a dev/integration branch, NOT deployed to the 3X.** Default behavior is unchanged
(the new feature is gated default-OFF), so merging it later is behavior-neutral until the param is set.

This doc records what the I-90 eastbound drive (Seattle → Snoqualmie Pass summit, Tesla Raven) taught
us about VTSC / CES curve behavior, and exactly what was (and was **not**) changed in response.

---

## The drive feedback (what the driver observed)

A run of right/left curves on I-90 climbing to the Snoqualmie summit, set speed 90 mph:

- **"entering a right curve and you are braking way too hard"** — speed set 90, doing ~80-something,
  brake-down to ~70. Repeated across many curves (4:55, 4:56, 5:00, 5:11–5:20).
- **"4:56": the decel happened *in* the curve** — should have been *before* it (vision saw the curve
  too late to finish slowing before the entry).
- Driver's proposed rule: **"every curve should always start with a deceleration of at least one mile
  per hour"** (a small, definite "I see the curve" cue at curve onset).
- At the **summit long right curve**: dash said *"reduce speed 65"*, **"we're going 55 — way too slow."**
- A follow-on left curve into Hyak: *"going 66, really slow."*

So two *opposite* complaints in one drive: **too-hard / too-late braking on the climb curves**, and
**too-slow (over-braked) at the summit curve.** The telemetry explains why they're not contradictory.

---

## Key telemetry finding (this is the important part)

At the Snoqualmie **summit** curve, the two systems behaved very differently:

| System | What it did at the summit | Binding? |
|--------|---------------------------|----------|
| **VTSC** (vision curve cap) | cap ≈ **82 mph** — *above* the set/actual speed | **NO** — not the cause |
| **CES → Experimental** (e2e long) | end-to-end model braked to **~55 mph** | **YES** — this is the over-brake |

**Conclusion: the "too slow at the summit" over-braking was CES switching to Experimental mode**
(whose end-to-end longitudinal planner brakes for the curve on its own), **not VTSC.** VTSC's vision
cap wasn't even binding there. This reframes every fix below.

Two facts that were *already true* in the tree before this branch (don't re-do them):

- **`A_LAT_TARGET` is already `1.9`** (not the old `1.5`). VTSC's curve aggressiveness was already
  softened — see `vtsc_constants.py:11`.
- **The "≥1 mph at curve start" mechanism already exists** as `CONFIDENCE_CUT = 0.5 m/s` (~1.1 mph): the
  instant a *binding* curve is detected, VTSC applies an immediate ≥1 mph cap cut (`vtsc_constants.py:43`,
  applied on the `idle→brake` transition). **What this branch added (driver follow-up):** extend that cut
  to fire on **every** real bend, not only binding ones — see the cue section below.

---

## Resolution — what to change vs. what already covers it

| Drive complaint | Root cause | Resolution |
|-----------------|-----------|------------|
| Too-slow / over-braked at the summit curve | **CES→Experimental** e2e braking, not VTSC | **Use Light mode (`CESMode=1`).** Light hands curves *entirely to VTSC* and does **not** switch to Experimental for curves — so the e2e over-brake can't happen. This already exists (light-ces-gentle); it just needs to be the selected mode. |
| "≥1 mph dip at the start of *every* curve" | The cut only fired on *binding* curves (curve-safe speed below set speed); gentle bends got nothing | **Extended:** new `CUE_MIN_CURVATURE` (~5°/200 m ≈ R2300 m) makes a mild bend "count" so it gets the `CONFIDENCE_CUT` ≥1 mph dip too. See the cue section below. |
| Curve cap too aggressive | — | `A_LAT_TARGET` **already 1.9** (softened). No change. |
| **Decel happened *in* the curve / too late** (vision sees sharp curves late); summit curve under-read by vision (cap 82 while a 58 mph curve was real) | VTSC is **vision-only**, ~8 s horizon | **NEW feature: MTSC** — fold pfeiferj map curve safe-speeds (`MapTargetVelocities`, longer horizon) into VTSC so it can start braking earlier and catch sharp curves vision under-reads. **Gated default-OFF** (`VtscMapCurves`). |
| "road clear" shown with a car right in front | overlay only checked map curve | **Overlay fix:** show the lead gap (`lead Nm`) when a lead is tracked; only say "road clear" (now green) when there's neither an upcoming map curve nor a tracked lead. |

---

## Guaranteed ≥1 mph cue on every curve (driver follow-up)

The driver wanted a definite small "I see the curve" dip at the **start of every real curve**, including
the gentle bends VTSC would otherwise ignore. Today the ≥1 mph `CONFIDENCE_CUT` only fires when VTSC
decides to *brake* (curve-safe speed below the set speed); a gentle sweeper produced nothing.

**The fix is a one-predicate change**, not a new code path. `model_curve_state` already returns the real
apex curvature `k_apex` for *any* bend — even a gentle one whose curve-safe speed stays above cruise. So
a new threshold `CUE_MIN_CURVATURE` lets a mild bend "count" as a curve:

```python
# vtsc_constants.py — ~5° of heading change over the upcoming ~200 m == radius ~2300 m
CUE_MIN_CURVATURE = (5° in rad) / 200 m  ≈ 4.36e-4 1/m   (R ≈ 2292 m)

# vtsc_controller.py — a curve "counts" if it BINDS *or* is a mild bend past the cue threshold
has_curve = d_apex >= 0.0 and (v_curve < v_cruise - 0.1 or k_apex >= C.CUE_MIN_CURVATURE)
```

Why this is enough — the **existing** state machine does the rest:
- For a gentle bend, `brake_cap_for_apex` returns a speed well above cruise (no real slowing needed), so
  the brake target clamps to exactly `v_cruise − CONFIDENCE_CUT` → a single ~1 mph dip, held through the
  bend, then eased back out at the apex. It never brakes harder than the dip.
- **Binding (sharp) curves are byte-identical** — they already satisfied `v_curve < v_cruise − 0.1`, so
  the new OR term changes nothing for them; they still get graduated braking.
- **No stacking** on a winding road: the floor is `min(applied, v_cruise − CONFIDENCE_CUT)` against the
  *base* `v_cruise`, so repeated cue curves can never ratchet below `v_cruise − 0.5`.

**Threshold sanity:** at 90 mph the cue radius is ~0.07 g lateral — gentle, **above** straight-road / lane
noise (`MIN_CURVATURE`, R≈10 km) and **below** where VTSC already brakes for real (~R851 m @ 90 mph). It
catches sweeping highway bends without firing on straights. **Folded into normal behavior — no toggle**;
active whenever VTSC is on (`CESMode` ≥ 1). With maps ON it also fires *earlier* (before the bend).

Reviewed by Gemini (`gemini-pro-latest`): **APPROVE WITH NITS** — it traced the gentle-bend path, confirmed
no binding-curve regression, no stacking, and `≤ v_cruise` / `≥ V_MIN` bounds hold. The one nit (hardcoded
π vs `math.pi`) is intentional: `vtsc_constants.py` is deliberately import-free ("pure literals") so the
core stays unit-testable without the openpilot stack.

---

## What this branch changes (4 files, all default-OFF / display-only)

1. **`common/params_keys.h`** — new param `VtscMapCurves` (`PERSISTENT BOOL "0"`, **default OFF**).
2. **`selfdrive/controls/lib/vtsc_xnor/vtsc_constants.py`** — `MAP_LOOKAHEAD_S = 12.0`,
   `MAP_MIN_SLOWDOWN = 3.0` (+ a documentation block).
3. **`selfdrive/controls/lib/vtsc_xnor/vtsc_controller.py`** — the MTSC core:
   - `_read_map()` reads `MapTargetVelocities` + `LastGPSPosition` from `/dev/shm/params` (the **same**
     source CES uses), ~1 Hz, fully exception-guarded.
   - `_fold_map_curve()` compares the upcoming **map** curve against the **vision** curve via
     `brake_cap_for_apex` (required-speed-now) and uses **whichever is more binding**.
   - one-line hook in `cap()` after `model_curve_state`, only when `self._map_curves` is true.
   - **Safety:** the chosen curve (map or vision) feeds the **same** decel-limited brake/hold/release
     state machine → still rate-limited by `A_DECEL_MAX`, still floored at `V_MIN`. A wrong/low map
     speed therefore brakes **smoothly (never slams)** and stays bounded. When the param is OFF,
     `_map_curves` stays False and the fold is skipped → **behavior-neutral**.
4. **`selfdrive/ui/onroad/ces_status.py`** — overlay: `lead Nm` instead of a false "road clear"; "road
   clear" recolored green. `dRel` is published by `decision_telemetry` (`ces_xnor.py:211`), verified.

### MTSC = MAP Turn Speed Control (vs. VTSC = VISION)

VTSC reads the driving model's predicted path curvature (~8 s). MTSC reads pfeiferj mapd's
per-point OSM curve safe-speeds (`MapTargetVelocities`, longer horizon). Folding map in lets VTSC
**brake earlier** and **catch sharp curves vision under-reads**. Map was historically the MTSC
*deferral* reason ("until mapd safe-speeds are fixed"), which is exactly why it ships **default-OFF**
behind `VtscMapCurves` and rides the existing safety envelope rather than commanding speed directly.

---

## Review

- **Gemini (`gemini-2.5-pro`): APPROVE WITH NITS.** Confirmed: correct more-binding pick; behavior-
  neutral when OFF; map data goes through the same `A_DECEL_MAX` + `V_MIN` safety limits; robust
  None/exception/JSON handling; `k_map` back-out is logging-only (no control depends on `k_apex`).
  The one nit (import ordering) is a **false positive** — `import json`/`import time` are already
  grouped at the top of `vtsc_controller.py` in stdlib order; Gemini misread the diff's context lines.
- **Independent publisher-side check:** verified `dRel`/`mapV`/`mapDist` are actually published into
  `CESStatus` by `decision_telemetry` (something Gemini couldn't see from the diff alone), so the
  overlay change is live, not dead code.

---

## How to test on the next drive (NOT yet done)

1. Deploy this branch's 4 files (surgical overlay) + **rebuild `params_pyx.so`** on-device (a new param
   key requires it, else UI crash-loop on `UnknownKeyName`). Set persistence guards.
2. Set **`CESMode = 1` (Light)** so curves are VTSC-only (no Experimental over-brake at summits).
3. Set **`VtscMapCurves = 1`** to enable MTSC.
4. Re-drive I-90 to the Snoqualmie summit. Expect: earlier, gentler braking *before* the summit curve
   (map horizon), no e2e drop to 55, the ≥1 mph engage cut felt at each curve onset.

---

## Follow-ups (not in this branch)

- **Validate the map safe-speeds.** They are the reason MTSC is default-OFF. Compare
  `MapTargetVelocities` targets against real curve geometry on the I-90 / I-5 logs before flipping the
  default on by default.
- **Accelerate-zone tuning** so the car recovers speed promptly after a curve series (the "going 55,
  way too slow" lingered into the follow-on straight).
- **A_LAT_TARGET clean-data validation** on a non-summit curve where VTSC *is* the binding system, to
  confirm 1.9 feels right without the CES e2e confound.
- Consider a future MTSC drive recommending `CESMode=1` + `VtscMapCurves=1` together as the curve combo.
