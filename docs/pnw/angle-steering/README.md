# Angle steering (BluePilot bp-7.0 port) — document set

Everything on the faithful port of **Alan Polk's BluePilot 7.0 path-angle-primary lateral control**
into the PNW line, the two live-drive sessions, and the tooling built around it.

**Upstream source of truth:** `BluePilotDev/bluepilot` branch `bp-7.0`
(`opendbc_repo/opendbc/sunnypilot/car/ford/lateral_angle_ext.py`). **Our port:** `pnw-opendbc`
branch `angle2pnw-faithful2` → `opendbc/car/ford/lateral_angle_pnw.py`.
**Feature gate:** `FordAngleLateral` (default **0**) + `PnwVehicle.angle_lat`.
**Tuning overlay:** `/data/pnw/angle_tuning.json` — hot-reloads every ~5 s, no restart needed.
In-code defaults are Alan Polk's exact values; an absent file == his code.

## Read in this order

| Doc | What it is |
|---|---|
| `ALAN-POLK-SPEC.md` | ⭐ The authoritative design, recovered from his published articles. `path_angle = κ × v_ego × gain_factor` with **c2/c3 zeroed**; the "sticky c2" PSCM firmware filter (`0x101B0B60`) that motivates the whole scheme; the four tuning params. Verbatim articles in `alan-polk-articles/`. |
| `ALAN-POLK-TUNING-TUTORIAL.md` | ⭐ His **tuning method**, transcribed from his two videos: green (commanded steering angle) vs yellow (achieved), judge at curve **peaks**, "yellow must not exceed green", one click = **0.01**, ±5° is road noise, never tune near stops. Also documents the **15/70 vs 30/60 mph anchor discrepancy**. |
| `ALAN-POLK-PORT-DEVIATIONS.md` | Every deviation our port makes from his code, by category, with justification — the document to hand him alongside any result. |
| `SIGN-CONVENTION-TRACE.md` | Planner → kappa_cmd → path_angle → wire negation → DBC → panda `angle_meas`, verified by executing the packer and decoding real bytes. |
| `DRIVE_REPORT-2026-07-18.md` | First activation (the deviant port). Failed; the "sign inversion" conclusion in it was later **refuted** — read the 07-19 report. |
| `DRIVE_REPORT-2026-07-19.md` | ⭐ The faithful port on-road. Fast-curve success; the 1.2 tuning mistake and the resulting lane departure; the **exit-unwind hold** finding; ICBM confounds; the refuted delivery-ratio metric. |
| `AUTOTUNER-DESIGN.md` | Whether to automate his tuning method. Verdict: **shadow/advisory post-drive analyzer**, no mid-drive auto-apply. Includes the field-data package for Alan Polk. |
| `REVIEW-FABLE-port.md` / `REVIEW-GEMINI-port.md` | The two independent reviews of the port itself. |
| `REVIEW-CURVE-LOGGER.md` | Review of the curve logger (5 blockers found and fixed — the first version would have recorded zero curves). |
| `alan-polk-video{1,2}-*.txt` | Full transcripts of his tuning-tutorial and drive videos. |

## Tooling

`scripts/angle_curve_logger.py` — curve-triggered event recorder. Ring-buffers 20 Hz data and commits
a window (8 s pre-roll + curve + 8 s post-roll) only when a real curve completes, so a month of
passive collection costs ~100 MB instead of ~40 MB/hour. Emits one aggregatable summary line per
curve (peak green, peak yellow, verdict, validity flags) plus a full trace. Works on **both cars**
(both run `steerControlType = angle`); every record is tagged with brand/fingerprint.

## ✅ RESOLVED — gain anchors: **15 / 70 mph, confirmed by Alan Polk directly** (2026-07-19)

The driver asked him directly. **He confirmed the anchors are meant to be 15 and 70 mph, and that
30 / 60 "is not enough."** So the shipping `bp-7.0` constants are a **bug**, not intent:

```python
# lateral_angle_ext.py:445-446  and  angle_factor_adjuster.py:33-34   (BluePilotDev/bluepilot @ bp-7.0)
interp(v_ego, [13.5, 26.82], ...)     # 30.2 / 60.0 mph  — WRONG per the author
                                      # intended: 6.71 / 31.29 m/s = 15 / 70 mph
```

**Why it matters (this is the whole low-speed story):** `interp` **clamps**, so with the 13.5 anchor
the curve gain is **flat at `1.30 × low_speed_factor` from 30 mph all the way down to 0**. Every speed
below 30 mph gets an identical gain — which makes 18 mph and 30 mph behavior impossible to separate
with any knob. That is precisely how a 2026-07-19 attempt to cure sub-20 mph wide-running (raising
`low_speed_curv_factor` 1.0 → 1.2) instead produced a **lane departure at 30 mph**: the "low speed"
factor is at full strength *at* 30 mph.

With the corrected 15 / 70 anchors the taper begins at 15 mph:

| mph | gain @ 30/60 (shipped) | gain @ 15/70 (intended) | Δ |
|---|---|---|---|
| ≤15 | 1.300 | 1.300 | — |
| 20 | 1.300 | 1.268 | −2.4% |
| 25 | 1.300 | 1.236 | −4.9% |
| **30** | **1.300** | **1.205** | **−7.3%** |
| 40 | 1.185 | 1.141 | −3.7% |
| 60 | 0.950 | 1.014 | +6.7% |
| 70 | 0.950 | 0.950 | — |

The largest reduction lands exactly at the departure speed, while sub-20 mph is nearly untouched —
the two symptoms finally sit on different parts of the curve.

**Applied here** via the tuning overlay (`gain_speed_lo_ms = 6.71`, `gain_speed_hi_ms = 31.29`),
verified loaded on-device 2026-07-19. This is a **correction toward the author's stated intent**, not
a deviation from his design — recorded as such in `ALAN-POLK-PORT-DEVIATIONS.md`.

⚠️ **Upstream impact:** this affects **every BluePilot Ford user** on bp-7.0, not just us — the low-speed
factor they are told to tune at 15 mph is in fact pinned at full strength through 30 mph.

## Open questions for Alan Polk

1. **Exit-blend threshold** — `_desired_falling` (lateral_angle_ext.py:395) needs the planner to unwind
   faster than **0.2 (1/m)/s**. Measured on our roads: median **0.020**, p99 0.119, **1 of 1134 frames**
   exceeded it. His exit-biased blend collapse therefore never engages here — our leading explanation
   for the observed **exit-unwind hold**. His pre-scaling value (`0.002`/call = 0.04 (1/m)/s) would fire
   on genuine exits. Possibly the same class of bug as the anchors above.
2. Lane Change Factor range: article says 0.5–2.0, code clips 0.85–1.50.
