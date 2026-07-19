# Alan Polk / BluePilot 7.0 angle control — THE AUTHORITATIVE SPEC (from his own writing)

**Source articles (archived verbatim in `alan-polk-articles/`; fetch fresh via the WordPress REST API,
the HTML pages are a JS "Loading posts…" shell):**

| Article | Date | URL |
|---|---|---|
| **BluePilot 7.0 – The Return of Angle Control (it wasn't the model's fault)** | 2026-07-15 | https://bluepilot.dev/announcements/?post=bluepilot-7-0-the-return-of-angle-control-it-wasnt-the-models-fault |
| How do Ford Lateral Controls Work and Why Are They Such a Challenge for OpenPilot? | 2025-07-13 | https://bluepilot.dev/2025/07/13/how-do-ford-lateral-controls-work-and-why-are-they-such-a-challenge-for-openpilot/ |
| Desired Curvature versus Predicted Curvature and why it matters for Ford | 2025-07-13 | https://bluepilot.dev/2025/07/13/desired-curvature-versus-predicted-curvature-and-why-it-matters-for-ford/ |

```bash
curl -s "https://bluepilot.dev/wp-json/wp/v2/posts?slug=<slug>" | python3 -c "import json,sys,re,html; print(html.unescape(re.sub('<[^>]+>','\n',json.load(sys.stdin)[0]['content']['rendered'])))"
```

**Alan Polk drives a Ford F-150 Lightning** — same vehicle as ours, so his base gain applies directly.

---

## 1. The architecture (why Ford is different)

Ford's lateral **planner lives in the PSCM** (power steering control module, inside the rack), not the
camera module. openpilot cannot command the wheel; it must **impersonate the IPMA's output** so the
PSCM's own planner executes what we want. Seven signals: curvature (c2), curvature rate (c3), path
offset (c0), **path angle (c1)**, ramp type, precision type, mode.

> "path_angle is actually the **steering wheel angle to correct path offset**, not the angle of the
> vehicle relative to centerline." — his own later correction.

## 2. THE DISCOVERY — "sticky c2" (why curvature control ping-pongs)

Reverse-engineered from Ford VBF firmware, **address `0x101B0B60`**: a **speed-indexed low-pass filter
on the PSCM's internal curvature state**.

| Speed | Filter time constant T | Settle time (5T) |
|---|---|---|
| 5 km/h | 328 ms | 1.64 s |
| 20 km/h | 195 ms | ~1.0 s |
| 40 km/h | 95 ms | ~0.5 s |
| 100 km/h | 35 ms | ~0.17 s |

The PSCM **integrates toward** a commanded curvature and **keeps commanding the old value for ~0.5–1.6 s
after we drop it**. That lag drives the oscillation loop: op commands → PSCM lags → op increases → PSCM
arrives late → op backs off → PSCM still "remembers" → overshoot the other way → repeat.

> **"What we had been calling 'model noise' was the model faithfully responding to oscillations the
> sticky curvature filter was creating. The model was never the problem."**

**Conclusion: don't fight sticky c2 — stop using c2 altogether.**

## 3. Why c1 dominates: the PSCM's lookahead is tiny

PSCM evaluates `y(x) = c0 + c1·x + ½·c2·x² + ⅙·c3·x³` at lookahead `d_ref` ≈ **1.4 m at highway speed**
(firmware lookup table). Effective contribution at 100 km/h:

| Signal | Weight | Contribution |
|---|---|---|
| c0 path_offset | 1.0 | full |
| **c1 path_angle** | **1.4** | **full** |
| c2 curvature | 0.98 | **centimeters** |
| c3 curvature_rate | 0.46 | **sub-millimeter** |

## 4. ⭐ THE FORMULA (this is the whole implementation)

> **Angle control sets `c2` and `c3` to ZERO.**
>
> ```
> path_angle = κ × v_ego × gain_factor
> ```
> where κ = the planner's **desired curvature**, v_ego = vehicle speed, gain_factor = small
> vehicle-specific calibration constant.

- **No lookahead term. No filter. No PID. No feedback. Pure feedforward geometry.**
  (bp-3.0's PID-feedback path_angle attempt is what he explicitly calls the failed "twitchy" version.)
- **Sign follows κ directly** — there is no negation anywhere in his description.
- "When we want to stop turning, we send zero, and the PSCM stops turning immediately and begins to
  unwind." Still subject to Ford + comma rate limits, but no sticky memory.

## 5. The four tuning parameters (the entire tuning surface)

| Param | Meaning | Default | Range |
|---|---|---|---|
| **Primary Control** | Curvature (old) ↔ **Angle** (new) | **Curvature** | toggle |
| **Low Speed Factor** | scales response **below ~30 mph / 50 km/h** | 1.0 | 0.5–1.5 |
| **High Speed Factor** | scales response **above ~70 mph / 115 km/h** | 1.0 | 0.5–1.5 |
| **Lane Change Factor** | steering authority during lane changes | 1.0 | 0.5–2.0 |

Base gain is **per model** (F-150, Mach-E, Escape…), then dialed in per vehicle. Why per-vehicle
tuning is unavoidable: the PSCM continuously compensates for yaw/sway/roll against a **factory model of
the vehicle** (trim, suspension, weight distribution). Lift kits, tire size, wheelbase, 2WD/4WD all
change the real vehicle but not the PSCM's internal model, so the gain must absorb the mismatch.

## 6. What he observed after switching

- Desired-curvature signal **got visibly smoother without touching the model** → angle control is
  tolerant of model swaps (no longer locked to 2–3 Ford-friendly models).
- **In-lane offset became unnecessary** — the chronic right bias was sticky-c2 integration, not calibration.
- **Lane changes became smooth** (command drops cleanly instead of the PSCM still "reaching").
- **Sharper curves became manageable** — no windup, can command sharp and unwind immediately.

## 7. Status / caveats in his own words

- Target launch **end of July 2026**; repo side "rough" — comma's upstream thermal changes make the
  **BluePilot UI overheat/crash on the comma 3X** (Comma4 mostly unaffected). Concessions likely.
- He shipped **defaults per model, not zero-tuning** — that hope "didn't survive contact with reality."

---

## 8. ⛔ WHERE OUR FAILED PORT DEVIATED (measured against §4)

| # | Our `angle2pnw` port | Alan Polk's spec | Verdict |
|---|---|---|---|
| 1 | kept a **shadow curvature** and **negated it**: `self._shadow_curvature = -self._latext_angle.bp_kappa_cmd` | **c2 = 0, c3 = 0** | ❌ **Both the negation AND the non-zero c2 are ours, not his.** Prime cause of the 100%-inverted commanded sign. |
| 2 | fixed **0.2 s lookahead** lookup (after dropping the VLT) | formula has **no lookahead at all** — `κ × v_ego × gain` | ❌ neither our fixed lookahead nor the VLT is in his spec |
| 3 | `_VLT_V_LOW_MS` = 25 mph, `_VLT_V_HIGH_MS` = 55 mph | speed breakpoints are **~30 mph and ~70 mph**, and they gate **gain factors**, not lookahead | ❌ wrong values, wrong role |
| 4 | PSCM saturation clamp + exit-biased blend collapse | not described | ⚠️ ours; must be justified or dropped |
| 5 | omitted stall-blip / `_PRESS_BLIP_MIN_S` | not in the article (may be in code) | ⚠️ verify against source |

**The corrected mental model:** angle control is *simpler* than what we built, not more complex. It is
three multiplications and two zeroed signals. Every extra mechanism we added is a deviation to justify
or delete before we can honestly tell Alan Polk "we ran your design."

**Open question for the re-port:** the article says c2=0; if his shipping code still writes a non-zero
c2 (e.g. for the PSCM's own health/limit checks), the **code** wins over the article — but the
deviation manifest must record which one we followed and why.
