# Drive report — 2026-07-19 (evening) — Lightning: first instrumented angle-steering tuning session

**Car:** Ford F-150 Lightning · **Lateral:** Alan Polk bp-7.0 angle steering, faithful port
(`FordAngleLateral=1`, opendbc `angle2pnw-faithful2`) · **Longitudinal:** stock ACC; **CES turned OFF
mid-session** at the driver's initiative (removed the ICBM mid-curve braking that contaminated every
low-speed sample earlier in the day) · **Channel** `3devpnw` @ `71ad354551`.
**Data:** `angle_curves.tar.gz` (38 files — `curves.jsonl` summaries + per-curve 20 Hz traces),
captured by `scripts/angle_curve_logger.py`, first live use.

## Config timeline (each curve record embeds its own tuning snapshot — no reconstruction needed)

| Segment | `low_speed_curv_factor` | anchors `lo/hi` (m/s) | Note |
|---|---|---|---|
| Session A | 1.00 | **6.71 / 31.29** | anchors corrected to 15/70 mph — **confirmed by Alan Polk directly** |
| Session B | **1.10** | 6.71 / 31.29 | raised from the Session-A measurement below |

The anchor change is the day's headline: bp-7.0 ships `13.5 / 26.82` (30/60 mph); the author confirmed
the intent is **15/70 mph** and that 30/60 "is not enough". Because `interp` clamps, the shipped values
make curve gain **flat from 30 mph down to 0** — so no knob can separate 18 mph from 30 mph behavior.
That is the direct mechanism behind the morning's lane departure (see the sibling
`lightning-angle-faithful/DRIVE_REPORT.md`).

## What was driven

A twisty low-speed road plus city/parking maneuvering. **24 curves captured, but only 3 were real road
curves** (R > 40 m, engaged, green peak > 8°). The rest were R = 7–36 m — parking lots, intersection
turns, one very tight ~5 mph corner the driver explicitly excluded.

## Measurements

### Real road curves
| Curve | R | speed | factor | raw ratio | note |
|---|---|---|---|---|---|
| 0009 (A) | 102 m | 19.9 mph | 1.00 | 0.86 | cleanest sample of the day: hands-off, engaged, **speed swing 1.6 mph**, 29% saturated |
| 0005 (A) | 386 m | 21.7 mph | 1.00 | 0.86 | speed-unsteady (ICBM still on) |
| 0006 (B) | 94 m | 23.9 mph | 1.10 | 0.89 | driver touched briefly; **lag-corrected 0.942** |

### ⭐ Lag correction changes the conclusion
Cross-correlating green (commanded steering angle) against yellow (achieved) on curve 0006:

```
best lag                 : 2 samples = 0.10 s   (r = 0.988)
RAW peak ratio           : 0.890
LAG-CORRECTED peak ratio : 0.942      green 108.8° -> yellow 102.5°
```

**About half the apparent shortfall is lag, not under-delivery.** Alan Polk says exactly this in his
tuning video ("the lines don't overlay… there's some delay and lag"), and it means a raw peak-ratio
metric **systematically under-reads delivery**. Adding gain to compensate for lag cannot fix lag — it
just overshoots later in the curve, a plausible contributor to the morning's departure.

**Consequence: hold at 1.10. Do NOT raise to 1.20.** True deficit at 24 mph is ~6%, not ~11%.

### Tight curves are an envelope limit, not a tuning deficit
| Radius | ratio | evidence |
|---|---|---|
| ~94 m | 0.94 (corrected) | fine |
| 36 m | 0.79 | `steerSaturated` → **"Take Control"** |
| 12–19 m | 0.69–0.79 | alerts, human-turn override fired |

The driver's "it didn't steer enough" maps onto the **tight** curves. This matches Alan Polk's own
account of his F-150: a 15 mph advisory curve needs ~23 mph, traffic circles are impossible,
navigational turns unsupported. **More gain will not buy these curves** — it saturates harder and
alerts more. Below roughly **R = 20 m** the Lightning is out of authority.

## Hypotheses tested and KILLED this session
- **Deviation-clip trap** (`v_ego > 9` clip chaining command to measured yaw): **refuted** — only
  **5% of samples** sat at the clip bound on curve 0006. Not the binding constraint.
- **"Raise the low-speed factor to fix low-speed understeer"**: **refuted twice** — first by the
  morning's departure, then by the lag analysis showing most of the deficit isn't gain-related.

## 🆕 New finding for Alan Polk: `liveDelay` is 4× the real lag
`liveDelay.lateralDelay` reported **0.4 s**; measured lag by cross-correlation is **0.10 s**.
His own code comments on this failure mode and caps `liveDelay` at 0.15 s for VLT purposes precisely
because "liveDelay can calibrate up to ~420 ms on some runs", which "pushes the model lookahead 5 m
into the curve". **We are seeing his documented bug live, at 0.4 s** — it inflates the VLT lookahead
and is a candidate for curve-entry behavior.

## Instrumentation notes (first live use of the curve logger)
- Worked as designed: 24 curves recorded, correctly flagged; disengaged maneuvers show green == yellow
  by construction (openpilot sets desired = actual when lateral is inactive) and are excluded.
- **Driver catch, added mid-session:** openpilot's own lateral-limit alerts were not being logged.
  Now captured (`steerSaturated/warning`, text `"Take Control"`, plus `angleState.saturated` — which is
  a **tracking-error** flag, |desired − actual| > 2.5°, *not* a hardware limit; Ford is explicitly the
  branch openpilot marks as unable to detect true torque saturation).
- **Watch out:** `ratio > 1` with `driver_touched` is the DRIVER steering more than commanded, not
  oversteer (curve 0007, ratio 1.33). An auto-tuner must not read that as a reason to cut gain.
- Fixed mid-session: tuning snapshot was embedding ~2 kB of `_doc` prose per curve.

## Next session
1. **Hold 1.10.** Need road curves **R > 80 m, hands-off, steady speed, 15–25 mph** — this session
   produced only one, and it was driver-touched.
2. Apply **lag correction as standard** in all future verdicts; the raw peak ratio is not trustworthy.
3. Verdict logic needs a ratio test alongside the ±3° absolute deadband (it called several real
   shortfalls "ok" at small angles).
4. Still open: the **exit-unwind hold**, and its leading explanation — `_desired_falling` needs
   0.2 (1/m)/s unwind while measured median is 0.020 (1 frame in 1134 exceeded it), so Alan Polk's
   exit-biased blend collapse never engages on these roads.
