# FORDREGEN2PNW — Highway follow jerkiness / regen over-decel on the F-150 Lightning (op-long)

**Status: DESIGN ONLY (2026-07-13). Nothing built, nothing deployed, device untouched.**
Ford-Lightning-only. Safety envelope (ACCEL_MIN/MAX, MIN_GAS, panda ACCDATA checks) must stay
unchanged in every option below.

Scope: the *actuation-layer* longitudinal jerk when op-long (Alpha Long ON) follows a lead on the
highway. This is the highway sibling of the city "hopping like a horse" incident already documented
in `pnw/FORDLONG2PNW.md` (2026-07-12) — same family (commanded-vs-actual accel mismatch), different
speed band and different dominant cause.

---

## 1. Measured symptom (2026-07-13, driver in the Lightning ~80 mph, following a lead)

| Signal | Observed |
|--------|----------|
| `longitudinalPlan` a_target (planner) | smooth, ~**+0.15 … +0.23 m/s²**, barely moving |
| `carState.aEgo` (actual) | swings **0.00 → +0.95 m/s²**, and on the decel side overshoots to **~−0.87 m/s²** while the command stayed mild |
| Driver feel | "when it needs to slow it's jolty — too much deceleration, then it accelerates again, jerky" |
| Personality | 0 = Aggressive during the measurement (then switched to 1 = Standard to test) |

Driver hypothesis (assessed correct as the *root plant behavior*): the EV's **regenerative braking
bites harder than the gentle commanded decel** when op-long lifts throttle → over-deceleration →
op-long re-adds throttle → oscillation.

---

## 2. Code trace: how a planner accel becomes truck throttle/regen

Path for the 80 mph follow case (op-long ON, Lightning → `bp_long_follow` capable, 4-signal lateral
live, so BOTH `_latext` and `_longext` are constructed and the BP acc message owns the frame).

### 2a. The closed-loop that produces `actuators.accel` (THE KEY FINDING)

`pnw/pnw-pilot/selfdrive/controls/lib/longcontrol.py:82-88` — in `LongCtrlState.pid`:

```python
error = a_target - CS.aEgo
output_accel = self.pid.update(error, speed=CS.vEgo, feedforward=a_target)
```

`actuators.accel` is **NOT** the smooth planner target. It is `feedforward=a_target` **plus a PID
correction on `(a_target − aEgo)`**. And the Ford PID gains
(`pnw/pnw-opendbc/opendbc/car/ford/interface.py:40-41`) are:

```python
ret.longitudinalTuning.kiBP = [0.]
ret.longitudinalTuning.kiV = [0.5]      # ki = 0.5 at all speeds
# kpV / kpBP are never set for Ford -> default [0.] (interfaces.py:215-218)
```

So **Ford longitudinal control is feed-forward + a PURE INTEGRATOR (kp = 0, ki = 0.5)**. When regen
drags `aEgo` to −0.87 while `a_target` = +0.2, the error is a sustained **+1.07**, the integrator
**winds up** and pushes `output_accel` (= gas) up → truck lunges → `aEgo` overshoots to +0.95 →
error flips negative → integrator unwinds → throttle drops → regen bites again. A pure integrator
against a punchy, low-damping EV plant is a textbook **limit cycle**. This integral term is literally
the "then it accelerates again" the driver feels.

(`get_pid_accel_limits`, interface.py:22-28, clamps the positive limit toward **0.2** as speed nears
the set point — so near set-speed the re-add is capped at 0.2 yet `aEgo` still reaches +0.95, i.e.
the +0.95 is the EV's *own* punchy response to a ≤0.2 gas request, not op-long over-commanding. Away
from set-speed the limit opens to 2.0. Which regime the 80 mph clip was in is an rlog item — §5.)

### 2b. The Ford actuation layer (`opendbc/car/ford/carcontroller.py`, ~340-408)

```
342  if openpilotLongitudinalControl and frame % ACC_CONTROL_STEP == 0:   # 50 Hz
343      accel = actuators.accel            # <- the PID output from 2a
344      gas   = accel
350      accel = apply_creep_compensation(accel, vEgo)   # ~0 above 3 m/s -> inert at highway
354      accel = max(accel, self.accel - (3.5 * ACC_CONTROL_STEP * DT_CTRL))  # 3.5 m/s^3 brake-down rate limit
356      accel = clip(accel, ACCEL_MIN=-3.5, ACCEL_MAX=2.0)
357      gas   = clip(gas,   ACCEL_MIN,      ACCEL_MAX)
360-361 if not longActive or gas < MIN_GAS(-0.5): gas = INACTIVE_GAS(-5.0)
364-372 accel_pitch_compensated = accel + g*sin(pitch)
        # stock brake-bit hysteresis: >0.3 -> brake off ; <0.0 -> brake on ; 0.0..0.3 holds
377  if self._longext and self._latext:                 # BP follow shaping
384      lng = self._longext.update(CC, CS, sm, accel, gas, pitch, vEgo*2.23694, stopping, V_CRUISE_MAX)
397  if lng is not None and lng.bp_long_used:            # >50 mph, lead present -> TRUE here
398      create_acc_msg(... lng.gas, lng.accel, ... lng.brake_actuate, lng.precharge_actuate ...)
405  else:                                               # stock path (byte-identical to pre-port)
```

The ACC CAN message (`fordcan_pnw.create_acc_msg`, fordcan_pnw.py:228-250) carries **both** a
propulsion request and a brake request every frame:
- `AccPrpl_A_Rq` = `gas` (the truck's throttle/coast command; near-zero/negative = lift = **regen**)
- `AccBrkTot_A_Rq` = `accel` (friction-brake total-accel request)
- `AccBrkDecel_B_Rq` = `brake_actuate` bit, `AccBrkPrchg_B_Rq` = `precharge_actuate` bit

So on a gentle command the truck receives a small/negative **gas** request with the **brake bit
OFF**, and the PCM's mapping of that gas request to actual wheel torque (regen) is what over-decels.

### 2c. The BP follow shaping (`opendbc/car/ford/longitudinal_ext_pnw.py`)

Active at 80 mph (`bpSpeedAllow` latches ON >50, OFF <45; lead present and >40 mph). Relevant to the
jerk:
- Lead classified **gaining / pacing / trailing** by `v_rel` with **hard ±0.1 m/s thresholds and NO
  hysteresis** (lines 139-145). At a matched-speed follow, noisy `vRel` flaps the state every scan.
- `pacing` caps `max_follow_gas = 0.2 + pitch`, `min_follow_gas = 0` (line 153); `trailing`/`gaining`
  leave gas at `op_gas`. So a flapping state **steps the gas clamp** between 0.2 and `op_gas` frame
  to frame — an independent jerk source layered on top of 2a.
- brake/precharge hysteresis: brake on `<−0.14`, off `>−0.06`; precharge on `<−0.12`, off `>−0.06`
  (lines 176-183), cross-scan latched (the port fixed BP's per-scan re-init chatter). For a command
  that never crosses −0.14 the brake bit stays OFF — consistent with "command stayed mild, brake not
  commanded, yet aEgo −0.87" ⇒ the −0.87 is **regen via the gas request, not the friction brake**.
- `following_accel_ROC = 0.002`/scan down-rate limit applies only lead-present with TTC>8 s & gap>0.5 s.

---

## 3. Mechanism analysis (Q1 / Q2)

**Q1 — does anything model or compensate the truck's regen?** **No.** Nowhere in
`longcontrol.py`, `carcontroller.py`, `longitudinal_ext_pnw.py`, `interface.py`, or the CAN builders
is there any term that accounts for an EV producing *more* deceleration than an ICE for the same
gas/coast request. `apply_creep_compensation` handles the opposite end (low-speed creep, ~0 above
3 m/s). The gas→wheel-torque map is treated as the PCM's problem and consumed open-loop as a
propulsion request. **This is the gap.** The only thing "fighting" the regen is the generic
longcontrol integrator (2a), which does it badly (limit cycle).

**Q2 — what produces the commanded-vs-actual mismatch?** Ranked by evidence:

- **(d) EV PCM/regen over-response to the gas request — ROOT CAUSE.** The Lightning's mapping of
  `AccPrpl_A_Rq` in the coast/near-zero region to actual wheel torque is steep and punchy in *both*
  directions (regen on lift → −0.87; torque on re-add → +0.95). openpilot has no model of it.
- **Amplifier: the Ford pure-integral loop (2a).** kp=0, ki=0.5 turns a plant nonlinearity into a
  sustained oscillation. This is the largest *in-scope* contributor and the cheapest to move — the
  integrator is exactly the "accelerates again" recovery the driver reports.
- **(b, partial) BP lead-state gas-clamp flap.** No hysteresis on the ±0.1 `v_rel` classifier ⇒
  `max_follow_gas` steps between 0.2 and `op_gas` at a matched-speed follow. Real, in-scope, but
  secondary to the loop dynamics.
- **(a) stock/BP brake-bit flap in the hysteresis band — likely NOT active here.** The measurement
  says the command stayed mild (never crossed −0.14), so the brake bit almost certainly stayed OFF;
  the −0.87 came through the gas/regen channel, not a flapping friction-brake bit. (Confirm in rlog;
  it *was* the dominant cause in the city-hop incident, which is why the `bp_long_used` gate exists.)
- **(c) the 3.5 m/s³ brake-down rate limit — not the highway-follow culprit.** It only shapes the
  downward step of `accel` (friction brake), which isn't being commanded here; its own comment admits
  step overshoot, but that path is inactive for a mild-command follow.

**Conclusion:** root plant behavior is EV regen (d), which openpilot can only *work around*; the
biggest thing we actually control is the **pure-integral Ford loop** turning (d) into a limit cycle,
with the **BP gas-clamp flap** (b) as a secondary contributor. A proper **regen feed-forward** (model
d so the loop doesn't have to fight it) is the physically-correct fix but needs the gas→aEgo map from
the rlog to size.

---

## 4. Fix options, ranked (Q3)

All Ford-only (they live in the ford brand files / Ford `CarParams`), none touch panda or the
ACCEL/MIN_GAS clips. Gate anything new behind the existing capability view
(`opendbc/car/pnw_vehicle.py`, e.g. reuse/extend `bp_long_follow`) — never a fingerprint string.

### Option A (RECOMMENDED FIRST) — damp the Ford longitudinal loop
**Change:** in `ford/interface.py:40-41`, lower the Ford integral gain and add a little proportional
damping, e.g. `kiV = [0.5] → [0.3]` and `kpV = [0.0] → [0.2]` (`kpBP=[0.]`). Ford-only; Tesla's tune
is in `tesla/interface.py` and is untouched.
**Why:** directly attacks the amplifier in §3. A pure integrator against a punchy plant *is* the
limit cycle; lowering ki slows windup so the loop stops chasing the regen transient, and a small kp
adds phase lead (damping) without steady-state pumping. `feedforward = a_target` still carries the
tracking, so speed-holding is preserved.
**Expected effect:** markedly less overshoot/oscillation; slightly slower to null a real speed error
(gentler catch-up). **Risk: LOW.** No safety surface; fully reversible one-liner; standard openpilot
per-car tuning. Validate that decel authority is unaffected (feedforward passes a_target straight
through; ACCEL_MIN unchanged).
**Caveat:** works around, doesn't cure, the regen nonlinearity — but it's the safest, data-free first
move and will visibly reduce the jerk.

### Option B — regen feed-forward (the physically-correct cure; needs rlog first)
**Change:** in the Ford long path (cleanest in `longitudinal_ext_pnw.py`, or a small helper in
`carcontroller.py` before the acc msg), add a **bounded positive bias to the gas request in the
coast band** so a near-zero commanded accel doesn't invoke strong regen — i.e. invert the empirical
`AccPrpl_A_Rq → aEgo` map (from §5) over, say, `−0.5 < gas < +0.3`, capped at a small offset
(order ~0.1–0.3 m/s², to be set from data), and **only** additive to gas, never to `AccBrkTot_A_Rq`
and never below where the brake bit takes over.
**Why:** removes the root mismatch so the loop (even the current one) has little to fight.
**Expected effect:** commanded ≈ actual in the gentle region; the oscillation loses its driving
nonlinearity. **Risk: MEDIUM** — an over-large bias delays/weakens genuine gentle decel (truck
coasts less when you want it to). Must be small, coast-band-only, speed-gated to the follow regime,
and leave the friction-brake path alone. **Do this AFTER Option A and AFTER the rlog calibration.**

### Option C — add hysteresis/smoothing to the BP lead-state classifier
**Change:** in `longitudinal_ext_pnw.py:139-145`, give the `gaining/pacing/trailing` `v_rel`
thresholds hysteresis (e.g. enter pacing at |v_rel|<0.1, leave at |v_rel|>0.3) or low-pass `v_rel`,
so `max_follow_gas` stops stepping between 0.2 and `op_gas` on a matched-speed follow.
**Why:** removes the secondary gas-clamp flap (§3b).
**Expected effect:** smoother gas in steady following; no effect on the regen root cause.
**Risk: LOW-MEDIUM.** Only shapes the BP gas clamp; ACCEL/brake untouched. Confirm it doesn't slow
legitimate gap responses.

### Option D — brake/precharge hysteresis / brake-rate retune (LOW priority here)
Widen the BP `−0.14/−0.06` band or retune the stock `3.5 m/s³` down-rate limit. **Only worth doing
if §5 shows the brake bit actually flapping at 80 mph** (expected NOT to, per §3a). Keep on the shelf;
it was the right fix for the *city* incident, not obviously this one.

**Recommended sequence:** ship **Option A** first (data-free, low-risk, directly damps the limit
cycle), confirm with a follow-up drive + rlog, then use the same rlog to calibrate **Option B** (the
real cure) and add **Option C** if the classifier flap is visible.

---

## 5. Rlog validation plan (Q4 — from the parked truck, driver will provide)

Pull a highway-following window (the ~80 mph segment) and **time-align these signals** to separate
"brake bit flapping" from "loop chasing regen" from "plant just over-regening":

| Source msg | Field | Purpose |
|-----------|-------|---------|
| `longitudinalPlan` | `accels[0]` (a_target) | the smooth planner command (baseline) |
| `carControl.actuators` | `.accel`, `.gas` | the **PID output actually sent** — does it swing while a_target is flat? (⇒ confirms integrator windup, §2a) |
| `sendcan` → ACCDATA (0x186) decoded | `AccPrpl_A_Rq`, `AccBrkTot_A_Rq`, `AccBrkDecel_B_Rq`, `AccBrkPrchg_B_Rq` | the CAN as the truck saw it — is the **brake/precharge bit toggling** (⇒ Option D) or steady OFF (⇒ regen via gas)? |
| `carState` | `aEgo`, `vEgo` | actual accel vs command; measure **lead/lag** of aEgo vs gas |
| `radarState` | `leadOne.dRel/vRel/vLead` | reconstruct the BP lead-state classification — does `vRel` cross ±0.1 repeatedly (⇒ classifier flap, Option C)? |
| `controlsState`/`carState` | `vEgo` vs set speed | which `get_pid_accel_limits` regime (pos-limit 0.2 vs 2.0) the clip was in |

**Analyses to produce:**
1. Overlay a_target vs `actuators.accel`/`gas` vs `aEgo` — quantify the oscillation amplitude/period
   and confirm the loop (not the planner) is driving it.
2. Check the brake bit trace — if OFF throughout, (a) is excluded and the −0.87 is confirmed as
   **regen through the gas channel** ⇒ prioritize A + B, shelve D.
3. Build the empirical **steady-state `AccPrpl_A_Rq → aEgo` map** in the coast region (bin gas
   requests where the brake bit is OFF, plot resulting aEgo) — this **sizes the Option B regen
   feed-forward** offset and its coast band.
4. Plot `vRel` vs the pacing/trailing boundary to confirm/deny the Option C classifier flap.

Every drive analysis goes under `drives/<date>/.../DRIVE_REPORT.md` (CLAUDE.md Rule 5) with the raw
rlog saved alongside.

---

## 6. Single recommended first change

**Option A: reduce the Ford longitudinal integral gain (kiV 0.5 → ~0.3) and add small proportional
damping (kpV ~0.2)** in `pnw-opendbc/opendbc/car/ford/interface.py`. It is Ford-scoped (Tesla
untouched), needs no new data, has no safety surface (ACCEL_MIN/MAX, MIN_GAS, panda ACCDATA all
unchanged), is a trivially reversible one-liner, and directly damps the pure-integral limit cycle
that turns the EV's regen nonlinearity into the felt jerk. Then use the parked rlog (§5) to calibrate
Option B (the actual regen cure) and decide on Option C.

## Related
`pnw/FORDLONG2PNW.md` (the BP follow controller this rides on; the city-hop sibling incident) ·
`docs/LONG.md` (longitudinal capability reference) · `docs/ICBM2PNW.md` (the stock-ACC arm of the
Alpha-Long A/B) · `docs/CURVESLOW2PNW.md` (Lightning EV curve-braking calibration) · `CREDITS.md`.
