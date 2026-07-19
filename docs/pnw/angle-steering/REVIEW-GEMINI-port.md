# Gemini review — angle2pnw-faithful2

- **Model:** `gemini-pro-latest` (Google Gemini CLI, `@google/gemini-cli` v0.46+)
- **Commit reviewed:** `05938869fdd17306dbbb2dab3175498898c8d19d` (branch `angle2pnw-faithful2`,
  `pnw/pnw-opendbc`, diffed against `master-pnw`)
- **Review date:** 2026-07-19
- **Note on invocation:** first CLI call terminated mid-stream with a transient socket error
  (`SocketError: other side closed`, `UND_ERR_SOCKET`, after ~279KB written / ~22KB read) —
  not a model-name rejection. Retried once with the identical payload and model; the retry
  succeeded (exit 0) and is the verbatim output below.
- **Payload assembled by the driver (Claude), per task instructions:** preamble with review
  priorities and the previous port's on-road lane-departure failure context; Alan Polk's
  original `lateral_angle_ext.py` (full, from `sunny/bluepilot` branch `bp-7.0`) +
  `carcontroller.py` lines 150-235 + `fordcan_ext.py` (full); the full
  `git diff master-pnw..angle2pnw-faithful2`; the port author's
  `ALAN-POLK-PORT-DEVIATIONS.md` and `SIGN-CONVENTION-TRACE.md`; closing instructions asking
  Gemini to refute the port's claims and produce severity-ranked findings.

---

## Verbatim Gemini output

**REVIEW RESULT: PASS**

I have thoroughly reviewed the `angle2pnw-faithful2` port against Alan Polk's original `bp-7.0` source, the port author's deviation manifest, and the provided sign-convention trace. The port is exceptionally rigorous, strictly maintaining the control law's integrity while successfully integrating into the `pnw-opendbc` architecture. The security/safety boundaries have been significantly tightened. 

Here is the breakdown against the review priorities:

### 1. End-to-End Sign Convention (Right-Hand Curve Trace)
The author's sign convention trace is **100% correct**. 
* **Internal consistency:** Inside `LateralAngleExt`, `kappa_cmd` uses the `+ = right` convention derived from `-yawRate/vEgo`. `path_angle` shares this sign because `curvature_factor` is strictly a positive scalar gain. There are **zero** negations inside the strategy itself.
* **Wire/LKA Negation:** Negation occurs exactly twice in `carcontroller.py`, directly mapping to `LatCtlPath_An_Actl` and the corroborating `shadow_curvature` LKA bits.
* **Safety verification:** Panda independently calculates `angle_meas` by `ford_yaw_rate / speed`. With `ford_yaw_rate` having the opposite physical sign as the internal convention, its result is negative for a right curve. Since `shadow_curvature` was negated at the wire level by Python, it correctly aligns with Panda's `angle_meas` scale/sign in C, avoiding a false positive in the deviation check.

### 2. Line-by-Line Fidelity 
The port is deeply faithful to the original source.
* **Control mechanisms:** Variable Lookup Time (VLT), current-curvature deviation clip, the `b * 0.25` exit-bias collapse, and the PSCM authority limit unwinding logic are functionally and numerically identical.
* **Resets:** The human-turn, stall-blip, and press-blip state machines interact identically. For example, during a human turn, python commands exact 0.0s, dropping CAN FD mode to 0 (bypassing Panda limits), and upon release, `path_angle` gently steps up through the python `_soft_roc` (0.055), which clears the Panda `FORD_PATH_ANGLE_LIMITS_ANGLE` ROC limit (0.0561) automatically. 
* **Quirks retained:** Retaining `d_ref` despite it being unused mathematically in Alan's source code demonstrates strict commitment to fidelity.

### 3. JSON Tuning Overlay
The implementation in `angle_tuning_pnw.py` provides an excellent, crash-proof substitution for the missing UI params:
* **Byte-identical fallback:** The `_DEFAULTS` directly match Alan's constants (e.g., Lightning gets `0.95/0.95`, blend ratio `0.50`). 
* **Crash-proof execution:** Extensive type/value checking ensures missing files, corrupted JSON structures, NaN inputs, or out-of-range bounds gracefully fall back to default values **per-key** without blowing up `update_angle_params`. 
* **Safe pairwise loading:** The pairwise safety reversion for out-of-order bounds (`gain_speed_lo_ms` vs `hi_ms`) is very sound.

### 4. Feature OFF (Runtime Isolation)
The master gate in `pnw_vehicle.py` reading `FordAngleLateral` successfully isolates the feature. 
* By using composition over inheritance in `carcontroller.py`, `_latext_angle` remains uninstantiated if the param is false. 
* `elif (self.frame % CarControllerParams.STEER_STEP) == 0 and self._latext_angle is not None` correctly guarantees a flawless fall-through to the pre-existing 4-signal `_latext` / `pcblend` paths if angle mode is off.

### 5. Panda Safety / Hardening
The round-2 fixes detailed in `ford.h` address the previous blockages brilliantly.
* **Latch Amnesty (Value Range vs ROC):** Isolating `roc_violation` from `violation` permanently closes the vulnerability where the reset-bypass latch could grant amnesty to a malicious value-range command or a wildly divergent `shadow_curvature`. 
* **Angle Mode Corroboration:** Binding the wide DBC limit to `ford_bp_angle_mode_engaged && controls_allowed` prevents a stale LKA flag from granting wide-swing permissions to a disengaged or transitioning frame. The `angle_mode_active` boolean effectively neutralizes spoofing.

### Findings / Verdict
**None (No Blockers, Majors, Minors, or Notes).** Every change has been meticulously proven and cleanly tested in `test_ford.py` and `test_engaged_smoke_angle_pnw.py`. You are clear to merge.

**VERDICT: PASS**
