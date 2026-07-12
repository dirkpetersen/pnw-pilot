# CES2-STUDY.md — CES vs sunnypilot DEC vs FrogPilot CEM: comparison + "CES2" redesign proposal

**Branch:** `decstudy2pnw` (cut from `origin/3devpnw` @ `fce2b03372`, which includes the
2026-07-12 stop-intent fast path + evidence-gated pull-away merges).
**Status: RESEARCH / DESIGN ONLY. Nothing here is wired, deployed, or behavior-changing.**
The deliverable is this document; no code was added or modified.

**Question (driver's words):** *"compare what we have now with what FrogPilot and sunnypilot have
built. What we have now works OK but maybe there's an improvement possible."* Our CES decision core
has accreted rules (red-light 8 m/s floor, pull-away evidence gates, stop-intent fast path,
chill/exp dwells, freeway gate, accel-zone, curve triggers with VTSC handoff). It works, but it
smells like a rule pile. Do DEC/CEM have fleet-refined heuristics that could replace parts of it?

**Sources examined (all local, exact versions):**

| System | Source | Version |
|---|---|---|
| **Ours (CES)** | `selfdrive/controls/lib/ces_pnw/ces_pnw.py` + `ces_pnw_constants.py` on this branch base | `3devpnw` @ `fce2b03372` (2026-07-12) |
| **sunnypilot DEC** | `sunny/sunnypilot/sunnypilot/selfdrive/controls/lib/dec/{dec,constants}.py` | `Version = 2025-6-30` — **byte-identical to `sunnypilot-upstream/master`** (fetched 2026-07-12), so this IS their newest |
| **FrogPilot CEM** | `FrogPilot/frogpilot/controls/lib/conditional_experimental_mode.py` + `frogpilot_planner.py`, `frogpilot_following.py`, `frogpilot_variables.py`, `frogpilot_utilities.py` | worktree `dirk`, tip 2026-02-28 (very current) |
| History | `docs/DEC.md` (our ORIGINAL 2026-06 pre-CES analysis of DEC/CEM), `docs/CES.md`, `CES_I90.md` (branch), `docs/LIGHT_CES.md` | — |
| Field evidence | `drives/2026-07-11/lightning-icbm-nofire/`, `drives/2026-07-12/{snoqualmie-ellensburg-icbm, ellensburg-snoqualmie-westbound}/` DRIVE_REPORTs + ces_events | — |

Note: `docs/DEC.md` already made the structural call in June — "CEM is the better template because
it defaults to Chill and switches INTO Experimental; DEC only downgrades an already-Experimental
session." That call was right and CES was built CEM-shaped. This study is the *second* pass: now
that CES has a month of field calibration, what do the other two still know that we don't?

---

## 1. Architecture in one paragraph each

**Ours (CES):** pure decision core `decide_active(signals) -> (bool, reason)` — a first-hit-wins
ladder (curve / stop / lowSpeed / slowLead) with suppression gates (freeway, lead-pacing,
accel-zone, highway lowSpeed gate) — wrapped by ONE aggregate `FirstOrderFilter` debounce
(τ=1.0 s, threshold 0.63) and an asymmetric-dwell state machine (Exp≥8 s before exit, Chill≥5 s
re-entry cooldown; Light profile 12/8 s) plus a stop-intent fast path that bypasses the entry side
entirely. Runs in `selfdrived`, upgrades Chill→Experimental (and back), per-condition toggles, 3-way
`CESMode`, 3-state button override, per-car capability gating via `PnwVehicle` (op-long vs Lightning
shadow/ICBM), persistent per-second telemetry (`ces_events.jsonl`).

**sunnypilot DEC:** runs in the longitudinal planner, **only active when Experimental Mode is
already ON** (`self._active = sm['selfdriveState'].experimentalMode and self._enabled`) — it
*downgrades* blended→ACC within an Experimental session, never upgrades Chill. Signals are smoothed
by bespoke `SmoothKalmanFilter`s feeding a `ModeTransitionManager` with confidence accumulation
(+0.1/hit, −0.05 others, ×0.98 decay), hysteresis thresholds (0.6 to change mode / 0.3 to keep it),
a min-mode-duration of 10 model frames (**0.5 s** @20 Hz), and an `emergency` bypass. Two ladders:
`_radar_mode()` and `_radarless_mode()`.

**FrogPilot CEM:** runs in FrogPilot's planner ecosystem; Chill-default, upgrades into Experimental
on a prioritized toggle-gated ladder, one `FirstOrderFilter` per *condition* (τ=1.0, stop-light
τ=**0.5**, threshold 0.63 ≈ "true ~1 s"), numeric `CEStatus` reason for the UI, a manual override
(`CEStatus 1/2` via screen press), and an explicit **standstill hold** (keep Experimental while
stopped and the model still says stopped). Everything is a user-tunable param; notable **fleet
defaults: almost every condition OFF** except stop lights (`CEStopLights=1`), navigation
(`CENavigation=1`), and turn-signal-below-55 (`CESignalSpeed=55`, with lane detection).

---

## 2. Normalized condition table (the complete trigger sets)

Legend: **→EXP** = condition requests Experimental/blended; **→CHILL** = condition forces/prefers
ACC. Speeds SI unless noted.

### 2.1 Ours — CES (`decide_active` + state machine), as of `fce2b03372`

| # | Trigger | Signal(s) | Threshold(s) | Hysteresis / dwell | Interactions |
|---|---|---|---|---|---|
| C1 | **curve →EXP** (map primary) | mapd `MapTargetVelocities` vs vEgo, GPS haversine | slowdown > 3 m/s within 10 s lookahead | aggregate filter + dwell (below) | gated by: v>5 m/s; **freeway gate** spd_lim ≥ 55 mph → hand to VTSC+MTSC UNLESS **sharp** (tiered-scaled map target < 30 m/s); **lead-pacing** (lead < 100 m, vLead ≥ vEgo−1) suppresses entirely; **accel-zone** suppresses (on-ramp is a curve); Light mode (`CESMode=1`) suppresses curve condition entirely |
| C2 | **curve →EXP** (vision fallback) | argmax \|orientationRate.z × velocity.x\| over model horizon | \|lat accel\| > 1.9 m/s², time-to-curve < 3.5 s, **blinker off** | same | same gates as C1 |
| C3 | **stop →EXP** | `modelV2.action.shouldStop` | boolean | **stop-intent fast path**: bypasses entry filter + Chill 5 s cooldown, `Condition.force()` charges exit side | requires **no lead**; respects `CESStops` toggle |
| C4 | **lowSpeed →EXP** | vEgo, lead status | 1 ≤ v < 40 mph (no lead) / 45 mph (lead); return +3 mph (43/48, via constants — hysteresis folded into the filter in practice) | aggregate filter + dwell | gated by: **highway gate** (OSM spd_lim ≥ 50 mph — 21 false trips fixed); **accel-zone** |
| C5 | **slowLead →EXP** | radarState.leadOne | closing dv > 5 m/s OR vLead < 1 m/s | aggregate filter + dwell | `CESLead` toggle |
| G1 | **accel-zone →CHILL** (suppresses C1, C4) | vSet−vEgo, dRel, vLead, shouldStop | open ahead (no lead OR dRel > 45 m ∧ vLead ≥ vEgo−1) ∧ vSet−vEgo > 6 m/s | — | killed by `model_should_stop`; **no-lead branch has the 8 m/s red-light floor** (`ACCEL_ZONE_MIN_V`) |
| G2 | **pull-away exception →CHILL** (below the 8 m/s floor) | dRel history, vLead−vEgo, shouldStop recency | lead in 5–60 m band, vLead ≥ vEgo+1, monotonic dRel rise ≥ 0.3 m × 3 samples ≥ 0.25 s apart, no shouldStop within 2 s, 2 ≤ vEgo < 8 m/s | evidence resets on lead loss / >10 m dRel jump | `PullAwayTracker` state; telemetry tag `pullAway` |
| SM | **state machine** | aggregate raw_active | filter τ=1.0, thr 0.63 | **Exp min-dwell 8 s / Chill re-entry 5 s** (Light: 12/8) | stop-intent fast path skips entry side only |
| UI | 3-state button (CES/Chill/Exp), 3-way `CESMode`, 4 per-condition toggles | | | | per-car: op-long OR Lightning shadow (ICBM executor) |

Standstill/resume: no explicit standstill rule — C3/C4 hold Experimental near stops; exit is
filter-decay + 8 s dwell.

### 2.2 sunnypilot DEC (2025-6-30, == upstream master)

| # | Trigger | Signal(s) | Threshold(s) | Hysteresis / dwell | Notes |
|---|---|---|---|---|---|
| D1 | **MPC FCW →BLENDED (emergency)** | `mpc.crash_cnt` via Kalman filter | filtered > 0.5 | bypasses min-duration entirely (`emergency=True`) | we have no equivalent (CES runs in selfdrived, no MPC handle) |
| D2 | **lead →ACC** (radar cars only) | `radarState.leadOne.status` Kalman-filtered | prob > 0.45 | confidence machine | **"lead present ⇒ ACC, always"** (unless standstill) — highest non-emergency priority in `_radar_mode` |
| D3 | **slow-down →BLENDED** | **trajectory endpoint** `md.position.x[32]` vs speed-indexed expected distance table | expected dist: 32 m @0 kph … 165 m @60+ kph; urgency = shortage ratio ×2 (×2 again under 0.3×expected; speed factor >25 kph); trips at filtered urgency > 0.24–0.3 | urgency > 0.7 ⇒ **emergency** (immediate); else confidence-weighted | *the* battle-tested piece: a graded, model-native "the model wants to slow/stop ahead" detector. Invalid/short trajectory at >20 kph also scores 0.3 urgency |
| D4 | **standstill →BLENDED** | carState.standstill counter | count > 3 | counter ramps 0–20 | keeps blended through stops (smooth resume) |
| D5 | **slowness →ACC** | vEgo ≤ vCruise × 1.025 | filtered prob > 0.55, hysteresis ×0.8 active / ×1.1 inactive | suppressed while slow-down active or standstill | "cruising below set speed ⇒ ACC accelerates better" — the generalized form of our accel-zone |
| D6 | default →ACC | — | confidence 0.7 | — | |
| SM | `ModeTransitionManager` | confidence per mode | change thr 0.6 / keep 0.3, ×0.98 decay | **min mode duration 0.5 s**; emergency bypass | far shorter dwell than ours; smoothing lives in the Kalman filters instead |
| Gate | **active only when ExperimentalMode already ON** | | | | can only downgrade — structural mismatch with our use (already adjudicated in docs/DEC.md) |

No curve condition at all. No map/OSM input. No turn-signal logic. Radarless ladder = same minus D2.

### 2.3 FrogPilot CEM (worktree tip 2026-02-28)

| # (CEStatus) | Trigger | Signal(s) | Threshold(s) | Hysteresis / dwell | Fleet default |
|---|---|---|---|---|---|
| F1 (3/4) | **below speed →EXP** | vEgo, `following_lead` (tracking_lead ∧ dRel < 2·t_follow·vEgo) | 1 ≤ v < `CESpeed` (no lead) / `CESpeedLead` (lead) | none (raw compare) | **both default 0 = OFF** |
| F2 (5) | **turn signal + no lane →EXP** | blinker, `calculate_lane_width` of target lane | v < 55 mph ∧ blinker ∧ adjacent lane < `LaneDetectionWidth` | none | **ON @55 mph** w/ lane detection |
| F3 (6/7) | **nav intersection/turn →EXP** | `frogpilotNavigation.approachingIntersection/Turn` | nav-supplied | none | **ON** (needs nav route) |
| F4 (8) | **curve →EXP** | vision: argmax \|orientationRate.z·velocity.x\|; also "in curve now" \|v²·curvature\| ≥ 1.3 | trip when v² · \|k\| > 1 m/s² (`(1/|k|)^0.5 < v`), v > 5 m/s, blinker off | per-condition filter τ=1.0, thr 0.63 | **OFF** |
| F5 (9/10) | **slow/stopped lead →EXP** | lead_one vs vEgo | closing > 5 m/s (`CRUISING_SPEED`!) or vLead < 1 | filter τ=1.0 | **OFF** |
| F6 (11/12) | **stop light/sign →EXP** | `model_length` (trajectory endpoint x[-1]) | endpoint < vEgo × `CEModelStopTime` (default **8 s**); OR `model_stopped` (endpoint < 50 m ∨ forcing_stop) | **fast filter τ=0.5**, thr 0.63; requires **not tracking_lead**; disabled in "traffic mode" | **ON** — the one big default-on condition |
| F7 (13) | speed-limit-controller wants EXP | SLC | — | — | SLC-coupled |
| SM | first-hit-wins ladder, per-condition filters, numeric CEStatus | | | **standstill hold:** while standstill, keep current Experimental if `model_stopped`; skip ladder | manual screen-press override (1=Chill, 2=Exp) |

---

## 3. Adjudication — condition by condition

Criteria: our two-car fleet (Tesla Raven vision-only op-long; Lightning stock-ACC + ICBM shadow),
and the driver's documented rules — **(a)** the lead car can NEVER pull away (2026-07-08 directive,
re-confirmed 2026-07-12 incident), **(b)** never lurch through red lights (redlight2pnw, 2026-07-11),
**(c)** freeway curves belong to VTSC/MTSC/ICBM (map-first), not CES-Experimental (CES_I90 summit
over-brake; icbmmapfirst2pnw), **(d)** brisk on-ramp accel (accel-zone origin, 2026-07-09),
**(e)** comfort over aggressiveness in-curve.

| Topic | Ours | DEC | CEM | **VERDICT** | Why (one line, grounded) |
|---|---|---|---|---|---|
| **Stop detection signal** | binary `shouldStop` + fast path | **graded trajectory-endpoint urgency** (speed-indexed expected-distance table, emergency >0.7) | endpoint < vEgo×8 s + model_stopped, fast τ=0.5 filter | **ADOPT-THEIRS (hybrid DEC+CEM)** | Both fleets converged on *trajectory-endpoint shortage* as the stop/slow detector; it is graded (DEC) and horizon-timed (CEM), where our binary `shouldStop` forced us to invent the 8 m/s floor + recency guard to disambiguate red-light vs on-ramp. This is the single biggest import. |
| **Stop entry latency** | fast path bypasses filter+cooldown | `emergency=True` bypasses min-duration | τ=0.5 stop filter (2× faster) | **KEEP-OURS** | Our fast path (2026-07-12) is the same idea as DEC's emergency flag, already field-motivated (occlusion trap); nothing to import beyond noting convergence. |
| **Stop while lead present** | stop requires `not has_lead` | n/a (lead ⇒ ACC outranks) | requires `not tracking_lead` | **KEEP-OURS** (= CEM) | All three agree: with a lead, follow the lead (MPC), don't vision-stop. |
| **Standstill / resume** | implicit (no rule) | standstill ⇒ blended (counter >3) | **hold Experimental at standstill while model_stopped** | **ADOPT-THEIRS (CEM)** | We have a gap: at a light, once vEgo=0 our conditions can decay and the 8 s dwell is the only thing holding Experimental; CEM's explicit "hold while stopped and model still says stopped" is the principled anti-lurch anchor at v=0 — directly serves rule (b). |
| **Lead present at speed** | 3 separate gates: lead-pacing (curve), accel-zone has-lead branch, pull-away exception | **one rule: lead ⇒ ACC** (radar mode, prob-filtered 0.45) | separate `CESpeedLead` threshold (default 0) | **HYBRID (adopt DEC's shape, keep our evidence)** | DEC's single "lead present and not being closed on ⇒ ACC" is the unified form of our three lead gates; our fleets' every lead complaint (2026-07-08 curve-hold, 2026-07-12 pull-away) was e2e-timid-behind-lead. But on the vision-only Raven a raw lead-prob rule is too noisy — keep our `PullAwayTracker`-style evidence for the *opening* determination. |
| **Slow/stopped lead** | dv > 5 OR vLead < 1 | none (lead ⇒ ACC even if slower!) | dv > 5 OR vLead < 1 (default OFF) | **KEEP-OURS** (= CEM values) | Identical thresholds as CEM — already battle-tested; DEC's "ACC even behind a braking lead" contradicts our smooth-decel goal on the Raven. |
| **Low speed / city** | 40/45 mph lead-aware + OSM highway gate | none (slowness only) | lead-aware thresholds, **fleet default OFF** | **KEEP-OURS** | CEM's fleet defaults ship this OFF — evidence the fleet found blanket low-speed Experimental unnecessary; ours survives because our OSM highway gate + accel-zone already removed its failure modes (21 false trips fixed). Keep, but see §5 (it shrinks). |
| **Want-to-accelerate ⇒ Chill** | accel-zone (vSet−vEgo > 6, open-ahead, floor 8 m/s, pull-away exception) | **slowness: vEgo ≤ vCruise×1.025 ⇒ ACC**, suppressed while slow-down active | none | **HYBRID** | DEC's slowness is our accel-zone generalized ("below set ⇒ ACC accelerates better") with exactly the right precedence (stop-urgency outranks it); adopting that *precedence* — not the 1.025 threshold — lets the 8 m/s floor retire (§4). Keep our open-ahead check (dRel > 45 ∧ not slower): blanket slowness would fight slowLead on the Raven. |
| **Curve trigger** | map primary (10 s) + vision fallback (3.5 s), freeway gate + sharp exception, tiered map scale, lead-pacing + accel-zone suppression, Light-mode handoff to VTSC | **none** | vision only (v²k > 1 m/s²), no map, **fleet default OFF** | **KEEP-OURS** | Neither fleet runs curve→Experimental by default — DEC never had it, CEM ships it OFF. That *validates* our direction of travel (CES_I90: hand curves to VTSC/MTSC/ICBM; Light mode suppresses the condition entirely). Our map-first machinery has no counterpart in either fork. |
| **Turn signal / turns** | blinker only suppresses vision-curve | none | **blinker + no-adjacent-lane < 55 mph ⇒ EXP (fleet default ON)** | **ADOPT-THEIRS (CEM), simplified** | A default-ON, fleet-proven condition we entirely lack: signaling a *turn* (no lane to change into) at city speed is a complex maneuver where e2e slows properly; cheap to add (blinker + lane-width from modelV2 laneLines, or blinker + v < threshold as the v1). |
| **Nav intersection/turn** | none | none | ON but requires an active nav route | **SKIP** | We have no navigation destination pipeline (mapd only); mapd's data could approximate intersections later, but that is new plumbing, not a port. |
| **MPC FCW ⇒ emergency blended** | none | crash_cnt Kalman ⇒ emergency | none | **SKIP (note)** | Right idea, wrong layer for us: CES runs in selfdrived without an MPC handle; on the Lightning-shadow config there is no op-long MPC at all. Revisit only if CES ever moves into the planner. |
| **Debounce machinery** | ONE aggregate FirstOrderFilter + asymmetric dwell 8/5 s | per-signal Kalman + confidence manager, 0.5 s min duration | per-condition FirstOrderFilter, stop 2× faster | **KEEP-OURS + one CEM refinement** | Our aggregate filter has a real flaw CEM avoids: one condition's decaying charge can mask another's rise (and vice versa — a curve's charge shortens the stop's ramp). Move to per-condition filters (CEM), keep our asymmetric dwell (field-tuned vs the 30 flips/min sawtooth; DEC's 0.5 s dwell would reintroduce it). DEC's Kalman/confidence stack adds complexity with no behavior we lack. |
| **Manual override** | 3-state button + CESMode | none | CEStatus screen-press 1/2 | **KEEP-OURS** | Equivalent; ours also has the 3-way profile selector. |
| **Per-car capability gating** | PnwVehicle (op-long vs shadow/ICBM), capability-view rule | none | none | **KEEP-OURS** | Unique to us; required by the two-car fleet. |
| **Telemetry** | persistent ces_events.jsonl with reason + counterfactual tags (`pullAway`) | none | numeric CEStatus (UI only) | **KEEP-OURS** | Our tuning loop depends on it; neither fork logs decisions persistently. |

### What they have that we entirely LACK
1. **Graded trajectory-endpoint slow-down urgency** (DEC D3 + CEM F6): a model-native analog
   stop/slow signal with a speed-indexed expected-distance table and an emergency tier. We only
   consume the binary `shouldStop`.
2. **Explicit standstill hold** (CEM, DEC D4): keep Experimental at v=0 while the model still says
   stopped.
3. **Turn-signal condition** (CEM F2, fleet default ON): blinker + no adjacent lane below ~55 mph
   ⇒ Experimental.
4. **Per-condition debounce filters** (CEM): independent charge/decay per trigger.
5. **Slowness precedence rule** (DEC D5): want-to-accelerate ⇒ ACC *except while slow-down evidence
   is active* — the clean priority we approximate with the 8 m/s floor.
6. (Not portable now) MPC FCW emergency; nav-route intersections.

### What we have that they LACK
1. **Map-based curve intelligence**: OSM curve targets, tiered scaling, freeway gate + sharp-curve
   exception, lead-pacing suppression, Light-mode VTSC handoff, map-first doctrine (icbmmapfirst2pnw).
2. **Evidence-gated pull-away** (PullAwayTracker: monotonic dRel rise, lead-swap reset, stop-recency
   guard) — both forks handle "lead pulling away" with blunt thresholds or not at all.
3. **Stop-intent fast path** with `force()` (asymmetric anti-flap: instant toward stopping, full
   dwell back).
4. **OSM speed-limit gates** (lowSpeed highway gate, curve freeway gate).
5. **Per-car capability layer** (PnwVehicle; shadow mode driving ICBM on stock ACC).
6. **Persistent decision telemetry** incl. counterfactual reason tags.
7. **Accel-zone with red-light guard** — neither fork protects on-ramp merges at all (DEC's slowness
   is close but unguarded by stop evidence horizon-forward; it only checks *current* slow-down state).

---

## 4. Can the 8 m/s red-light floor die?

**Yes — in principle it retires cleanly; in practice it retires after replay proof.**

Why the floor exists: with only a *binary* `shouldStop`, "no lead + open road + set ≫ ego" is
ambiguous between an on-ramp merge (want Chill) and a red-light approach *before the model commits
to stopping* (must stay Experimental). The floor (`ACCEL_ZONE_MIN_V = 8 m/s`) resolves the
ambiguity by speed band — and then needed the pull-away exception (7 constants + a tracker) to
punch a hole back through it for the legitimate below-floor lead-pull-away case, plus the
stop-recency guard to close the yellow-light trap that hole opened. That is the rule pile.

The DEC/CEM insight: the trajectory endpoint disambiguates *directly*. On a red-light approach the
model's plan endpoint contracts well before binary `shouldStop` asserts (CEM trips at endpoint <
vEgo × 8 s; DEC's urgency rises continuously against the expected-distance table). On an on-ramp
the endpoint is pinned at the far horizon. So the single principle becomes:

> **stop-evidence (graded, hysteretic) outranks accelerate-preference — at any speed.**

With that precedence (DEC D5's exact shape), the accel-zone no longer needs a speed floor: a
red-light approach at 5 mph *or* 25 mph shows contracting endpoint → accel-zone suppressed; an
on-ramp at 5 mph shows full endpoint → Chill allowed. The pull-away exception's speed band
(`PULLAWAY_MIN_V ≤ v < ACCEL_ZONE_MIN_V`) dissolves with the floor; the *evidence* half
(PullAwayTracker) survives as the general "lead genuinely opening" test at all speeds (it is what
makes the rule safe on the vision-only Raven). The stop-recency guard moves inside the graded
signal's own hysteresis (urgency decays over ~2 s instead of snapping to 0 — same effect,
no separate rule).

**Retirement condition (must hold before the floor is deleted):** offline replay (§6) over the
logged drives must show the graded stop-evidence active in **100% of the logged red-light-approach
windows** (specifically the 2026-07-11 exp→chill-at-0-5-mph incident window that created
redlight2pnw, and the 2026-07-12 ~14:0x pull-away incident where the floor mis-fired) **at or
before** the tick where today's floor+`shouldStop` combination acts, with zero new accel-zone
suppressions inside any on-ramp window (the 2026-07-09 03:01-03:15Z merge). Until then the floor
stays in as a belt-and-suspenders assert (log-only mismatch counter in shadow, §6 phase 2).

Caveat flagged honestly: DEC's `SLOW_DOWN_BP/DIST` table and CEM's 8 s horizon are calibrated on
*their* models; ours is lebowski. The table must be re-anchored on our own rlogs (the endpoint
statistics of lebowski on I-90/I-82 drives), not copied blind.

---

## 5. CES2 — proposed decision core

### 5.1 Structure (three layers, same wiring surface)

```
signals (per tick)                 evidence layer (stateful)            decision ladder (pure)
─────────────────                  ─────────────────────────            ─────────────────────
vEgo, vSet, spd_lim                StopEvidence:                        1. STOP        →EXP  (fast path)
lead {status,dRel,vLead}             graded urgency from trajectory     2. LEAD-OPEN   →CHILL-pref
modelV2 endpoint x[-1]               endpoint vs expected-dist table    3. TURN        →EXP
orientationRate/velocity             + shouldStop OR-in                 4. CURVE       →EXP  (Standard only)
blinker, standstill                  hysteretic decay (~2 s)            5. SLOW-LEAD   →EXP
map targets + GPS                  LeadOpening (= PullAwayTracker,      6. LOW-SPEED   →EXP
                                     band widened to all speeds)        7. default     →CHILL
                                   per-condition FirstOrderFilters
                                                                        state machine: asymmetric dwell
                                                                        8/5 s + stop fast path (unchanged)
```

### 5.2 The ladder, precisely

1. **STOP** — `stop_urgency ≥ HIGH` OR `model_should_stop`, and no tracked lead. HIGH urgency or
   shouldStop takes the existing fast path (force + bypass entry dwell). *New:* MEDIUM urgency
   (contracting endpoint, pre-shouldStop) charges the per-condition filter normally — earlier,
   smoother entries at red lights. **Standstill hold (new, from CEM):** while standstill ∧ endpoint
   still short, hold Experimental regardless of filter decay.
2. **LEAD-OPEN → Chill preference** — unified lead rule replacing three: if a lead is present
   within 100 m and `LeadOpening` evidence holds (vLead ≥ vEgo − 1 AND not closing), suppress
   CURVE and LOW-SPEED (never STOP or SLOW-LEAD). This is DEC's "lead ⇒ ACC" with our evidence
   gate; it subsumes lead-pacing (`CURVE_LEAD_PACE_DREL`), the accel-zone has-lead branch
   (`GAP_OPEN_M`), and the below-floor pull-away exception in one place. Driver rule (a) becomes
   ONE rule instead of three.
3. **TURN (new, from CEM)** — blinker ∧ vEgo < 25 mph ∧ not lane-change-available (v1: just
   blinker + speed; v2: lane-width from modelV2). Default-ON in FrogPilot's fleet; fills our only
   genuine coverage hole. Ships behind a new `CESTurns` toggle, default OFF first drive.
4. **CURVE** — unchanged machinery (map primary + vision fallback, freeway gate + sharp exception,
   tiered scale), minus the two lead/accel suppressions now owned by rule 2 and rule 7's
   want-faster check. Still suppressed entirely in Light mode.
5. **SLOW-LEAD** — unchanged (dv > 5 OR vLead < 1; CEM-identical, field-proven).
6. **LOW-SPEED** — unchanged thresholds + OSM highway gate; want-faster suppression now via rule 7.
7. **WANT-FASTER → Chill preference** (accel-zone v2) — `vSet − vEgo > 6` ∧ open-ahead
   (no lead OR rule-2 evidence) ∧ **stop_urgency LOW** (the DEC precedence). **No speed floor.**

### 5.3 Every rule the redesign deletes

| Deleted rule / constant | Today's job | Absorbed by |
|---|---|---|
| `ACCEL_ZONE_MIN_V` (8 m/s red-light floor) | disambiguate merge vs red-light below 18 mph | graded stop-urgency precedence (§4) — *after replay proof* |
| `PULLAWAY_MIN_V` / `PULLAWAY_DREL_LO/HI` speed-band carve-out | punch the pull-away hole through the floor | floor gone; LeadOpening evidence runs at all speeds |
| `PULLAWAY_STOP_CLEAR_S` recency guard (as a separate rule) | yellow-light trap | stop-urgency hysteretic decay (same 2 s, inside the signal) |
| `_lead_pull_away()` + `_accelerate_zone()` OR-wrapper | exception plumbing | single WANT-FASTER rule + LEAD-OPEN rule |
| accel-zone has-lead branch (`GAP_OPEN_M` = 45 m test) | lead-far = open road | LEAD-OPEN rule (dRel band already inside the tracker) |
| `CURVE_LEAD_PACE_DREL` as a curve-specific gate | lead paces the curve | LEAD-OPEN rule (same 100 m bound, one place) |
| accel-zone-suppresses-curve special case | on-ramp is a curve | WANT-FASTER precedence over CURVE in the ladder |
| aggregate single `Condition` filter | debounce | per-condition filters (CEM) — deletes the cross-condition masking artifact |

Net: today's core has ~19 independently-tuned trigger/suppression rules; CES2 has **12** (7 ladder
rules + freeway gate + sharp exception + OSM lowSpeed gate + Light-mode handoff + dwell pair), and
every deleted rule maps to a named absorber — nothing is dropped silently. The stop-intent fast
path, asymmetric dwells, CESMode/button/toggles, PnwVehicle gating, ICBM plumbing, and telemetry
are untouched. ICBM/IcbmEpisode is out of scope (it lives in the same file but is a separate
subsystem; CES2 should also be the excuse to split `ces_pnw.py` into `ces_core.py` + `icbm.py`).

### 5.4 New signals to build

- `StopEvidence`: consumes `modelV2.position.x[-1]` (endpoint) + vEgo + shouldStop. Expected-distance
  table re-anchored on lebowski rlogs (start from DEC's `SLOW_DOWN_BP/DIST`, verify on our logs).
  Output: LOW / MEDIUM / HIGH + hysteretic decay. Pure, unit-testable.
- `LeadOpening`: PullAwayTracker generalized (drop the speed band; keep monotonic-rise evidence,
  jump reset, and consume StopEvidence instead of raw shouldStop recency).
- Per-condition `Condition` instances (stop τ=0.5 like CEM, others τ=1.0).

---

## 6. Migration & testing plan

**Phase 0 — replay harness (the acceptance suite).** New
`selfdrive/controls/lib/ces_pnw/tests/replay_ces_events.py`:
- Input: any `ces_events.jsonl` (per-second `tick` records carry vEgo, vSet, dRel, vLead, lead,
  mapV, mapDist, curvePct, spdLim, gas, aEgo, mode, reason — everything `decide_active` needs
  except the model trajectory).
- Reconstruct the signals dict per record, run BOTH the current core and the CES2 core, emit a
  per-tick mode/reason diff. Assert: zero regressions on the labeled incident windows
  (2026-07-11 red-light lurch, 2026-07-12 pull-away, 2026-07-09 on-ramp pin, 2026-07-08 curve
  lead-hold, 2026-06-28 summit over-brake — all GPS+timestamped in the DRIVE_REPORTs).
- **Gap:** ces_events does NOT log the trajectory endpoint, so StopEvidence cannot be replayed from
  it alone. Two-track fix: (a) extract `modelV2.position.x[-1]` per second from the rlogs in
  `drives/2026-07-*/raw-logs/` via `tools/lib/logreader` (LogReader over rlog/qlog, join on wall
  clock to ces_events ticks — qlogs carry modelV2 at reduced rate, sufficient at 1 Hz join);
  (b) **add `mdlEndX` to the ces_events tick record now** (one telemetry field, behavior-neutral,
  can ship independently ahead of CES2) so every future drive is replayable without rlog surgery.
- Acceptance for the floor retirement is §4's condition, measured by this harness.

**Phase 1 — build CES2 core** as `decide_active2()` + evidence classes alongside the current core,
unit tests ported + new incident-window scenario tests (from real telemetry, per the tuning-loop
convention). No wiring change.

**Phase 2 — shadow A/B on-car.** Wire CES2 in *log-only* mode: `CESController` computes both cores
each tick and logs `mode2/reason2` next to the live decision in ces_events. Drive normally
(both cars). Diff after each drive; tune the StopEvidence table. This costs nothing behaviorally
and uses the existing telemetry loop.

**Phase 3 — flip.** When N drives show zero adverse diffs (and the §4 floor condition holds),
swap `decide_active2` in as the decider, keep the old core importable for one release, then delete
the absorbed rules. Gemini review at phase 1 and phase 3 (gemini-pro-latest, @ escaped), deploy per
the pnw-pilot-deploy skill.

**Effort estimate (rough):**

| Item | Effort |
|---|---|
| Replay harness + rlog endpoint extraction + `mdlEndX` telemetry field | ~1 day |
| StopEvidence + LeadOpening + per-condition filters + ladder (`decide_active2`) + unit/scenario tests | ~2 days |
| Shadow A/B wiring + first tuning pass on logged drives | ~1 day |
| On-road shadow validation | 3–5 normal drives (calendar, not effort) |
| Flip + rule deletion + doc/DEVICE-STATE updates | ~0.5 day |
| **Total** | **~4.5 focused days + a week of shadow drives** |

---

## 7. Bottom line

The suspicion was half right. Neither fork has a better *curve* or *lead* brain than ours — DEC has
no curve logic at all, CEM ships curves/lead/low-speed OFF by default, and neither knows about maps,
per-car capability, or evidence-gated pull-away. What they DO have, converged on independently by
two large fleets, is a better **stop/slow detector**: graded trajectory-endpoint urgency with the
precedence "stop evidence outranks accelerate preference." Importing that one primitive (plus CEM's
standstill hold and turn-signal condition, and per-condition filters) lets us delete the 8 m/s
red-light floor, the pull-away speed-band carve-out, and two of the three lead special cases —
a rule pile of ~19 becomes a ladder of 12 with our field-calibrated pieces (dwells, fast path,
freeway/sharp gates, OSM gates, trackers, telemetry) intact.
