# Sign-convention trace — angle2pnw-faithful2

**Purpose:** prove, frame by frame, that this port did not repeat whatever caused the previous
port's on-road failure regarding sign. Every number below except the two inputs (`desired_curvature`,
`yawRate`) was produced by actually running the ported code (`opendbc/car/ford/lateral_angle_pnw.py`
+ `carcontroller.py` on branch `angle2pnw-faithful2`), decoding the real CAN bytes it produced, not
hand-derived.

## Conclusion, up front

There is **no negation anywhere inside `LateralAngleExt`**. `self.bp_kappa_cmd` carries the same
sign as the planner's `desired_curvature` all the way through the blend, the lane-change scaling,
and the deviation clip, and `path_angle` is `kappa_cmd * v_ego * curvature_factor` with
`curvature_factor` always positive — so `path_angle` shares `kappa_cmd`'s sign too. Negation happens
**exactly twice**, both in `carcontroller.py`, both copied verbatim from Alan Polk's bp-7.0
`carcontroller.py`: (1) all four LMC/LMC2 wire signals are negated together
(`-path_offset, -path_angle, -apply_curvature, -curvature_rate`) — never a subset, so curvature and
path_angle can never end up on different sign conventions relative to each other; and (2)
`shadow_curvature = -bp_kappa_cmd` when angle mode is engaged, sent on the LKA message. We verified
by direct execution that `shadow_curvature`'s sign matches panda's independently-computed
`angle_meas` (raw yaw-rate-derived, never touched by any Python-side negation) for both a left and a
right curve — the exact cross-check `ford_shadow_curvature_error_check` performs on the device. This
is the single most important thing to get right per the task brief, and it checks out numerically,
not just by code inspection.

---

## Step 0 — establishing what "positive" means internally on this file

> **Label caveat (review finding m3):** the absolute LEFT/RIGHT names attached to "positive" below
> rest on the polarity of Ford's raw `VehYaw_W_Actl`, which is not verifiable from code alone —
> openpilot's ISO convention (positive = LEFT) for `actuators.curvature` suggests the labels may be
> flipped. This does not affect any conclusion in this document: every claim is **relative**
> (command vs measured, ours vs Alan Polk's identical transform), and the decisive check —
> wire `shadow_curvature` == panda's independently derived `angle_meas`, verified by executed-byte
> comparison in both directions — is label-free. Read "RIGHT" below as "the direction of positive
> internal κ", whichever physical side that is.

Neither `lateral_angle_ext.py` nor `lateral_angle_pnw.py` states outright whether
`actuators.curvature` (the planner's `desired_curvature`) is positive-left or positive-right. We
derive it from the file's own, already-deployed math:

```python
current_curvature = -CS.out.yawRate / max(v_ego, 0.1)
...
kappa_cmd = float(clip(kappa_cmd, current_curvature - CURVATURE_ERROR, current_curvature + CURVATURE_ERROR))
```

`current_curvature` is compared **directly** against `kappa_cmd` (which descends from
`actuators.curvature` with no sign change anywhere in between) — for this clip to do anything
sensible, `current_curvature` and `actuators.curvature` must share the same sign convention. This
exact formula (`current_curvature = -CS.out.yawRate / v`) is not new to this port — it's the same
formula in the already-shipping `lateral_curv_pnw.py` (`apply_ford_curvature_limits_ext`) and in
stock, non-BluePilot `carcontroller.py` (`apply_ford_curvature_limits`), both of which are live,
working code on real Fords. If the sign convention here were wrong, the deviation clip would reject
essentially all real steering the instant the car actually turned (current_curvature would have the
opposite sign of desired_curvature at any nonzero curvature above `CURVATURE_ERROR=0.002`), and Ford
lane-keeping would never have worked at all. Since it demonstrably does, this is solid:

- ISO/openpilot yaw-rate convention: **positive `yawRate` = turning left** (counterclockwise, viewed
  from above, z-axis up).
- Therefore `current_curvature = -yawRate/v` is **positive when `yawRate` is negative**, i.e.
  **positive `current_curvature` = turning right**.
- Since `desired_curvature` (`actuators.curvature`) shares that convention: **positive
  `desired_curvature` = a RIGHT curve.**

This convention is internal to this file (and to the sibling curvature-mode file) — it is *not*
claimed to be some universal openpilot-wide rule; `opendbc/car/vehicle_model.py`'s own
`calc_curvature()` docstring ("Multiplied by the speed this will give the yaw rate") describes the
*unnegated*, standard κ=yaw_rate/v relationship, i.e. positive-left, the opposite sense. What matters
for this trace is only the convention *this specific file* uses internally, which the clip above
pins down unambiguously and which the already-shipping stock/curvature-mode Ford code has been using
in production.

## Step 1 — a concrete right curve, traced through real execution

Test setup: F-150 Lightning, `v_ego = 20 m/s`, `yawRate = -0.10 rad/s` (negative ⇒ **right turn**,
per Step 0), `desired_curvature = actuators.curvature = +0.01` (positive ⇒ **right turn**, per Step
0 — consistent with the measured yaw rate, as it should be for a car that's actually mid-turn).
`FordAngleLateral` forced on. 60 frames run to let the soft ROC settle; below is the settled state.

| Quantity | Value | Sign / meaning |
|---|---|---|
| `actuators.curvature` (planner desired) | **+0.01** | RIGHT (input) |
| `current_curvature = -yawRate/v` | **+0.005** | RIGHT (measured, consistent with input) |
| `lat.bp_kappa_cmd` (post blend + deviation clip) | **+0.005** | RIGHT — same sign as input; deviation clip pulled the magnitude down (planner led measured by more than `CURVATURE_ERROR`), but **did not flip the sign** |
| `lat.path_angle_last` (internal, pre-negation) | **+0.1129 rad** | RIGHT — same sign as `bp_kappa_cmd`. No negation happened between `kappa_cmd` and `path_angle`; `curvature_factor` (the only other multiplicand) is a positive gain, always. |
| **Wire `LatCtlCurv_No_Actl` (c2)**, decoded from the actual `LateralMotionControl2` CAN bytes sent | **0.0** | c2 is pinned at the inactive sentinel in angle mode (`lat.apply_curvature = 0.0` always) — correct by design, not a sign question. |
| **Wire `LatCtlPath_An_Actl` (c1)**, decoded from the same frame | **-0.113 rad** | `= -path_angle_last`, i.e. the negation applied. |
| `CC._shadow_curvature` (sent on the LKA message) | **-0.005** | `= -bp_kappa_cmd`, same negation applied to the curvature-shaped side channel. |
| **Wire LKA `shadow_curvature`**, decoded from the actual `Lane_Assist_Data1` bytes sent | **-0.005** | Matches `CC._shadow_curvature` exactly (packing round-trips cleanly at this magnitude). |
| **panda's `angle_meas`** (independently: `ford_yaw_rate/speed`, **no Python-side negation involved** — panda decodes the raw `VehYaw_W_Actl` CAN signal directly, the same raw signal `CS.out.yawRate` is a passthrough of) | `-0.10 / 20 =` **-0.005** | Same raw yaw rate, same formula panda always uses, whether angle mode is on or not. |

**The cross-check that matters:** wire `shadow_curvature` (**-0.005**) and panda's independently
computed `angle_meas` (**-0.005**) **match** — same sign, same magnitude. `ford_shadow_curvature_error_check`
compares these two (`angle_meas.min - max_angle_error - 1 <= shadow_curvature_can <= angle_meas.max
+ max_angle_error + 1`); with them numerically equal, that check passes comfortably, exactly as it
should for a genuinely-tracking car. If the shadow_curvature negation were wrong (or missing, as in
the discarded `angle2pnw-faithful` draft that dropped it entirely), `shadow_curvature` would sit at
**+0.005** against panda's **-0.005** — a hard divergence that `ford_shadow_curvature_error_check`
would catch and block on, at any speed above `angle_error_min_speed` (9.9 m/s). This is precisely
the failure mode the task brief was worried about, and it is not present here.

## Step 2 — c2/c3 stay genuinely inactive

`lat.apply_curvature` and `lat.curvature_rate` are `0.0` unconditionally in angle mode (see
`lateral_angle_pnw.py`'s `update()` — every `LateralResult` it returns has `apply_curvature=0.0,
curvature_rate=0.0`). `-0.0 == 0.0`, so the negation at the wire is a no-op for these two signals;
they decode to exactly `0.0` in the captured frame above. This matches the task's hard rule 5
(c0/c2/c3 pinned to zero) and the ALAN-POLK-SPEC.md's summary of your published design (`path_angle
= κ·v·gain`, c2 = 0) — the code and the article agree on this specific point.

## Step 3 — the documented wire polarities, and an honest note on c1 vs c2

`fordcan.py` (stock, pre-dates BluePilot) and `fordcan_ext.py` / `fordcan_pnw.py` (identical
wording) carry this comment on the LMC/LMC2 builders:

```
c0 (path_offset): lateral offset between the vehicle and the centerline (positive is right)
c1 (path_angle): heading angle between the vehicle and the centerline (positive is right)
c2 (curvature): curvature of the centerline (positive is left)
c3: rate of change of curvature of the centerline
```

Using Step 0's convention (internal positive = right) and the **identical** negation this port
applies to both signals:

- **c2**: internal curvature positive=right → negated → wire **negative** for a right turn → per
  the comment above (wire positive=left, so wire negative=right) → **consistent**.
- **c1**: internal path_angle positive=right (same convention as curvature, since no negation
  happens between `kappa_cmd` and `path_angle`) → negated → wire **negative** for a right turn → per
  the comment above (wire positive=right, so wire negative=left) → **the comment's own stated
  polarity for c1 is the opposite sense of c2's.**

We flag this rather than paper over it. We see two honest possibilities: (1) Ford's PSCM genuinely
defines c0/c1 (a lane-position polynomial, `y(x) = c0 + c1·x + ...`) in a different axis convention
than c2/c3 (which may be sourced from/checked against yaw-rate-based curvature estimation
elsewhere in the PSCM) — entirely plausible for real reverse-engineered automotive protocol, and
the sort of asymmetry a firmware team could introduce without it being a bug; or (2) the comment
itself has a typo on one of the two lines and was never fully verified against real per-signal
polarity, similar to the stale docstring/comment issues already found and reported in
`ALAN-POLK-PORT-DEVIATIONS.md` section 5. **We cannot independently resolve which, and it is not
this port's job to** — what we *can* and did verify is that this port applies **the exact same
transform** you apply, to **the same signals, in the same places**, confirmed by diff against your
`carcontroller.py`. If your bp-7.0 negation is correct on real hardware (which you've stated it is,
having shipped it), ours is too, because it's the identical code path. If it's ever found to be
wrong on a Lightning specifically (a different PSCM firmware than your Mach-E), that would be a
finding about the *hardware*, not about whether this port faithfully reproduced your software — and
the fix, if one is ever needed, is one negation flip in `carcontroller.py`'s angle-mode dispatch, in
one place, easy to locate and undo.

## Step 4 — same trace for a left curve (symmetry check)

Same setup, `yawRate = +0.10`, `desired_curvature = -0.01`:

- `current_curvature = -0.10/20 = -0.005` (LEFT, consistent with input)
- `bp_kappa_cmd` settles to **-0.005** (LEFT, same sign as input, magnitude pulled in by the
  deviation clip exactly as in the right-curve case)
- `path_angle_last` settles to **-0.1129** (LEFT, same sign as `kappa_cmd`)
- Wire c1 (`LatCtlPath_An_Actl`) = **+0.113** (`-path_angle_last`)
- `shadow_curvature` sent = **+0.005** (`-bp_kappa_cmd`)
- panda's `angle_meas` = `+0.10/20 = +0.005`
- **Match**: wire shadow_curvature (+0.005) = panda angle_meas (+0.005). Same conclusion as Step 1,
  mirrored.

## Step 5 — what a broken port would have looked like, for contrast

We checked this because it's exactly the failure mode the brief was worried about. Three ways this
*could* have gone wrong, and why none of them are present here:

1. **Missing shadow_curvature entirely** (all zeros): this is what the discarded, never-driven
   `angle2pnw-faithful` draft did. `shadow_curvature_can` would sit at `0` against a nonzero
   `angle_meas` once the car actually turns — `ford_shadow_curvature_error_check` blocks the LMC2
   frame, and the car would appear to simply stop steering above ~10 m/s in any real curve. Not
   present here — we verified a nonzero, correctly-signed value is sent (Step 1/4).
2. **Double negation** (e.g. negating `bp_kappa_cmd` a second time inside the strategy before
   `carcontroller` negates it again): would put `shadow_curvature` back in-phase with the *internal*
   convention but **out of phase** with `angle_meas`, producing the exact same divergence-block
   failure as case 1, just via a different bug. We confirmed by direct inspection (and the smoke
   test's numeric trace) that `bp_kappa_cmd` is un-negated inside `lateral_angle_pnw.py` — the sign
   flip happens exactly once, in `carcontroller.py`.
3. **Negating path_angle/curvature but not shadow_curvature, or vice versa** (an asymmetric
   negation): would make the LMC2 command internally self-consistent but the LKA-message
   corroboration wrong, or the reverse. We verified both undergo the negation (Step 1/4 both show
   negated wire c1 *and* negated shadow_curvature, from the same un-negated `bp_kappa_cmd`/
   `path_angle_last` pair).

None of these three failure modes are present in `angle2pnw-faithful2`, verified by actually running
the code and decoding the CAN bytes it produced — not by re-reading the source and asserting it
looks right.
