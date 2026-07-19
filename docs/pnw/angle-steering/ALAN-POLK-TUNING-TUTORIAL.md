# Alan Polk's angle-steering tuning method — from his own two videos

Source: `sunny/bluepilot-videos/videoplayback{1,2}.mp4` (transcribed 2026-07-19 with whisper;
full text + timestamped segments alongside as `.txt` / `.segments.json`).
Video 1 = "how to use the lateral debug menu to tune the angle steering factors in BluePilot 7" (7:20).
Video 2 = voiceover drive of his F-150 on his low-speed high-curvature torture route (~5:35).

---

## 1. The core method — two lines on a chart

His lateral debug screen plots two traces:

| Trace | Meaning |
|---|---|
| 🟢 **GREEN** | the steering wheel angle **the model wants** |
| 🟡 **YELLOW** | the steering wheel angle **actually achieved** |

> "In a perfect scenario the green line and yellow line would always look like one line… In reality
> there's some delay and lag… but what we're really looking for is **peaks in curves**."

**The tuning rule, in one line:**
> "What we want is for the yellow line to **not exceed** the green line, either going up or down."

- 🟡 **overshoots** 🟢 at the peak → **oversteering → reduce the factor** ("back off two or three clicks")
- 🟡 falls short of 🟢 → understeering → increase — **but first check you weren't just going too fast**
- Judge at the **peak** of each curve, not the whole trace. The lines never lie on top of each other.

**Sign convention (his words):** right curve trends **down** (right is negative in Ford's world);
left curve trends **up** (positive is left).

## 2. Where to tune

- **Ideal:** a curvy road at **15 mph** for the low-speed factor, then highway curves at **70 mph** for
  the high-speed factor. "Tune as slow as you can, tune as high as you can."
- **Acceptable:** 30 and 60 — "do the best you can", the debug menu weights the adjustment by speed.
- A **blue shaded band** shows how much weight your current speed gives to the low vs high factor.
- ⚠️ **Never tune at/near stops, red lights, intersections:** "the model tends to get really squirrely…
  it likes to move the wheel a lot to really try and center up." Tune only while *actually driving* —
  15–20 mph is fine, but not decelerating to or accelerating from a stop.
- **±5° of wheel movement is road noise**, not signal. Ignore it.
- **Diagnostic:** overshooting some curves but undershooting others ⇒ you were simply **too fast** in
  the ones that undershot. Don't chase it with the factor.

## 3. What angle control can and cannot do (video 2, on his own F-150)

- Hands-free through **6–7 consecutive curves that curvature control could not do at all**.
- **"We can take a lot bigger and tighter curves than we used to, and we can also unwind."**
- ⚠️ **Ford's own lateral limits are the ceiling, and they tighten as speed rises:** "as the speed
  increases Ford really cuts back on the amount of steering it will let you have."
- Practical speed rule: **get close to the yellow advisory-sign speed.** A 15 mph sign curve worked at
  25–26 mph but he "wished I'd gone down to about 23". Hands-on he'd take that same curve at 35–40.
- An **off-center warning flash** appears as you approach the limit — the curve still completes, but
  it's the signal to slow down further, not to add gain.
- **Traffic circles are impossible** — BlueCruise signals won't support the required steering torque.
- **Navigational turns: not supported.**

## 4. His live tuning session, as it played out

1. First curve: green peaked ~45°, yellow overshot to ~48–49° → **minus 2–3 clicks**.
2. Next two curves: peaks lined up well. "You could spend forever trying to tune it perfect."
3. A left curve reached **70°** and slightly *under*steered — he attributes this to **carrying too much
   speed**, not to tuning.
4. A later left curve overshot on a curve he knew *wasn't* speed-limited → **another click or two down**.

Note the asymmetry in his own practice: he reduces on overshoot readily, and treats undershoot as
suspect until speed is ruled out.

---

## 5. ⚠️ DISCREPANCY: his spoken anchors (15 / 70 mph) vs his code (30 / 60 mph)

He says in video 1: *"Low speed factor is at 15 miles an hour and high speed factor is at 70 miles per
hour and the steering factor gets literally interpolated between the two."*

But **both** his control code and his own debug widget use `13.5` and `26.82` m/s:

```python
# lateral_angle_ext.py:445-446
low_gain_calc  = interp(v_ego, [13.5, 26.82], [1.0, gain_lowC_highV])
high_gain_calc = interp(v_ego, [13.5, 26.82], [1.30*low_speed_curv_factor, gain_highC_highV*high_speed_curv_factor])
# angle_factor_adjuster.py:33-34   _SPEED_LOW_MS = 13.5 ; _SPEED_HIGH_MS = 26.82
```

**13.5 m/s = 30.2 mph** and **26.82 m/s = 60.0 mph** — not 15 and 70.

Consequences, and why this matters to us:
- Since `interp` **clamps**, curve gain is **flat from 30 mph all the way down to 0**. There is no
  low-speed taper at all. Tuning "at 15 mph" therefore adjusts the *identical* number that governs
  30 mph — which is exactly how our 2026-07-19 session produced a 30 mph lane departure while trying
  to fix sub-20 mph wide-running.
- His advice "tune as slow as you can" is sound *only if* the anchor really is at 15 mph. At 13.5 m/s
  it is not — the slow-speed observation and the 30 mph behavior are the same knob.
- **Question for Alan Polk:** are the spoken 15/70 the intended design (and 13.5/26.82 a stale
  constant), or did the video misstate the anchors? This is the single highest-value question we have
  for him, because it decides whether `gain_speed_lo_ms` should be moved.

This is also a **third** article/video-vs-code discrepancy, after the Lane Change Factor range
(article 0.5–2.0 vs code clip 0.85–1.50) and the High Speed Factor anchor (article "~70 mph" vs code
60 mph). Pattern so far: **the code is the truth; his prose describes intent.**

---

## 6. What this means for OUR telemetry (the gap we hit on 2026-07-19)

His green/yellow chart **is** the measurement we were missing. Green = commanded steering angle,
yellow = achieved steering angle. On 2026-07-19 we logged neither — only `actuators.curvature`
(the planner's *input* to the strategy), with the entire blend/VLT/clip/gain/ROC chain unobserved.

To reproduce his chart in our logs we need, per sample:
- 🟢 **commanded** `path_angle` — from the wire: `sendcan` addr **982** `LateralMotionControl2`,
  signal `LatCtlPath_An_Actl` (wire value = −internal, per the port's sign convention)
- 🟡 **achieved** `CarState.steeringAngleDeg`
- plus `v_ego`, `steeringPressed`, `latActive`, lane-position from `modelV2`, and the mode byte
  (`LatCtl_D2_Rq`) so human-turn/stall-blip mode-0 pulses are visible.

**Peak-matched comparison in the same units is the whole game** — green and yellow must be compared
at curve peaks, hands-free, away from stops, ignoring ±5° noise.
