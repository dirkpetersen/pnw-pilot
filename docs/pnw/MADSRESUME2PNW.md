---
updated: 2026-09-06          # git-derived; bump when you edit this file
status: current        # current | drifted | superseded | unreviewed
---

# MADSRESUME2PNW — bounded auto-resume of the driver's own set speed

**Branches:** `madsresume2pnw` in `dirkpetersen/pnw-pilot` (the brain, the wiring, the toggle) and
`madsresume-exec2pnw` in `dirkpetersen/pnw-opendbc` (the button executor). The **safety** half — the
`ford.h` resume-button gate that permits a resume press while lateral authority is held — is the
owner's own commit `32049e5f` on `pnw-opendbc/madsresume2pnw` and is **not touched here**;
`madsresume-exec2pnw` is stacked directly on top of it, so that one branch is the complete,
pinnable matched set (safety gate + executor). Nothing in this work modifies `opendbc/safety/` or
`panda/`.

**Status: BUILT, TESTED, NOT DRIVEN. Governed by `DisengageOnBrake` (default OFF = active).**

> **onetoggle2pnw (2026-09-06):** the separate auto-resume toggle is GONE at the driver's request.
> **"Disengage on brake" alone governs both halves** — keeping lateral through the brake and
> resuming afterwards. Not a loosening: `mads_pnw` sets
> `lateral_only = (not disengage_on_brake) and braking and not blocked`, and the brain can only ARM
> on the rising edge of `lateral_only`, so the toggle was already implied.
>
> **Two consequences, stated plainly:**
> 1. **Auto-resume is now effectively default-ON** on a `PandaMadsSafety=1` Lightning, because
>    `DisengageOnBrake` defaults OFF. A self-engagement feature now reaches the driver without a
>    second opt-in — a deliberate deviation from this fork's "new toggles default OFF" rule, made by
>    the owner with the objection on the table. It cannot reach the friends channel by accident
>    because `PandaMadsSafety` is hand-set as part of the flash.
> 2. **It is no longer an instant kill switch.** The old toggle was re-read at ~1 Hz, so flipping it
>    OFF stopped a pending resume within a second. `DisengageOnBrake` is baked into
>    `CP.alternativeExperience` at card start and is `needs_restart=True`, so flipping it mid-drive
>    triggers an **onroad cycle** — openpilot restarts and must be re-engaged. To abort a resume in
>    the moment, use the pedals: gas or brake cancels it. (Fable review 2026-09-06.) Depends on MADS being
flashed and `PandaMadsSafety=1`; it is a handful of boolean tests per tick otherwise.

## The problem

The Lightning runs stock ACC, so openpilot only steers. `mads2pnw` made steering survive a brake
press ("Steering only"), but the driver still has to reach for the cruise stalk to get speed back.
The owner asked openpilot to press it for them.

## This is self-engagement, and it was accepted as such

Lateral authority is being used to unlock a longitudinal re-engagement openpilot did not otherwise
have. The owner accepted that objection explicitly, on two conditions that are now the axioms:

1. **Resume ONLY to the speed the driver ALREADY SET.** Never higher, never a new speed.
2. **The brake is always under the driver's foot**, so they can always take it back.

> **⚠️ AXIOM 1 WAS AMENDED BY THE OWNER, 2026-09-06.** They asked for a second path — **gasset2pnw**
> — where the driver names the speed **with the accelerator** and openpilot taps **SET** at whatever
> speed they reached: *"if I have been braking and I then accelerate with the gas and when I stop
> accelerating can't that be the speed that is then set ... that would be the most natural."*
> That IS "a new speed", so axiom 1 as written no longer holds for SET mode, and this doc asserted
> the opposite until the 2026-09-07 review caught it.
>
> **What replaces it, and why their version is the safer one:** a SET establishes the speed the
> truck is **already doing**, so it commands **no acceleration at all**, where a RESUME hands speed
> back to ACC and lets it climb. The acceleration-bounding gates (`setFar`, `staleContext`, the
> headway requirement) therefore do not apply to SET mode. What still binds: `slowing` (see below),
> the speed floor, openpilot engageability, the lead **distance floor** and **TTC**, and the rule
> that stock cruise must not already be engaged. Axiom 2 is untouched and still carries the envelope.
>
> **`slowing` exists because of what this truck does not tell us:** `regenBraking` is **never set on
> Ford** (0 occurrences in `opendbc/car/ford/`), so on a Lightning with 1-Pedal Drive a lift-off to
> *slow down* is indistinguishable from a lift-off to *cruise* — except by speed. A SET is refused
> while the truck is still decelerating past `DECEL_REFUSE_MS2`.
>
> Read the two modes as **separate features sharing a state machine**, not one feature with a
> loophole. `ResumeDecision.mode` records which of them fired.

### `slowing` / `decelUnknown` — and which drive mode gasset2pnw is for

> **SUPERSEDED 2026-09-13 — owner decision "Ignore regen, set" (engagegoal2pnw).** The `slowing`
> refusal is **removed from the gas-set path**. The 2026-09-11..13 weekend measured 1.5–1.9 m/s² of
> regen within ~1 s of every steering-only lift-off, so the gate refused half of them, and a SET− to the
> current speed commands no acceleration. The RES path never had a decel gate and is unchanged (D3
> undecided). `decelUnknown` is kept: it still holds the tap until a post-lift decel window exists, so
> fire timing and the logged `decel` are unchanged. A real brake still wins: gate 2, the `reBrake`
> re-arm, and the executor's pedal gate. **Known consequence:** a lift-off to slow down followed by a
> brake more than ~0.5–0.8 s later can get the SET first (weekend: Sat 12:41:50). The text below is the
> history of the gate.

MEASURED on this truck, 2026-09-07, route `000000f9` — 7 gas-release events above 5 m/s, decel over
the 0.4 s after lift-off: **median −0.06, p90 1.33, max 1.33, min −0.67 m/s²**. This Lightning
**coasts** on lift-off in the mode the owner drives; it does not hard-regen. `DECEL_REFUSE_MS2` is
therefore **1.0** — ~16× the coasting median, and 25 % below the only genuine slowing event seen.
n=7 from one drive; `decel` / `decelAgeS` are logged so this can be re-derived, not re-argued.

**Under Ford 1-Pedal Drive the arithmetic changes and gasset2pnw becomes inert by construction.**
1PD lift-off regen is roughly 1.5–2 m/s², well past the threshold, so a SET would be refused every
time — and in 1PD the only way to "lift to cruise" is a partial pedal, which keeps `gasPressed` true
and never releases the arm at all. That is the correct outcome, not a bug: in 1PD, lifting off *is*
the brake, and setting cruise would cancel exactly the deceleration the driver asked for.

`decelUnknown` is the companion refusal and it is **load-bearing, not redundant**: the estimator
resamples on its own cadence, unaligned to the driver, so the most recent completed window can
straddle the accelerator — where the truck was speeding UP and `decel` reads negative. A measurement
not taken entirely after both pedals came up is refused. Do not delete it on the reasoning that
`DECEL_WINDOW_S < RELEASE_MIN_S` makes it unnecessary; it does not (see the constant's comment).

| refusal | means |
|---|---|
| `slowing` | *(removed 2026-09-13, owner "Ignore regen, set"; no longer emitted)* the truck was still decelerating past `DECEL_REFUSE_MS2` |
| `decelUnknown` | no deceleration measurement taken entirely after lift-off yet; fails **closed** |

### The honest limit of condition 1

The button we send is Ford's RESUME (`CcAsllButtnResPress` on `0x083`). **The PCM chooses the speed,
not openpilot** — Ford ACC resume returns to the PCM's own last set speed. openpilot therefore cannot
*command* a target at all, and any design that claims to is lying about the mechanism. What this
feature actually does is:

* **observe** the driver's set speed before the brake, and refuse to press at all if it never saw
  one (`noSet`), if the observation is stale (`noSet`), or if it is unreadable (`setUnknown`);
* **refuse** if the truck is currently reporting a set speed *above* the observed one (`setRaised`);
* **refuse to press while stock cruise is ENGAGED**, where RES is a `SET+` (+1 mph) on Ford —
  enforced in the brain *and* again, independently, in the executor (`decide_resume`);
* **verify after the fact**: once cruise comes back, compare and emit a LOUD `ces_events` record plus
  `cloudlog.error` if it came back higher (`"reason":"setHigher","loud":true`).

The residual — the PCM restoring something other than what the cluster displayed — is **unobservable
in principle** and is therefore made **impossible to miss after the fact** rather than claimed away.

## Architecture (the established brain/executor split)

Same shape as `icbm2pnw` / `speedadjust2pnw`: a brain publishes a JSON mem-param, a per-brand
executor arbitrates and presses. Deliberately **disjoint** from the SET+/− path at every layer,
because a resume happens exactly when cruise is OFF and a set-speed tap exactly when it is ON:

| | SET+/− (icbm2pnw, speedadjust2pnw) | RESUME (this) |
|---|---|---|
| mem-param | `IcbmTarget`, `SpeedAdjustTarget` | `MadsResumeTarget` |
| payload | `{target, ceiling, ts, dir:"dec"\|"inc"}` | `{dir:"res", ts, eid, set}` |
| parser | `_parse_button_cmd` (whitelists `dec`/`inc`) | `parse_resume_cmd` (requires `dir=="res"`) |
| decision | `arbitrate()` → `decide_press()` | `decide_resume()` |
| cadence | `PressGovernor` / `RestoreGuard` | `ResumePress` (one press per `eid`, ever) |
| cruise precondition | **enabled** | **NOT enabled** |
| freshness | 2.0 s | **0.5 s** |

Neither parser accepts the other's payload, and `decide_press()` now **hard-rejects any direction
outside `("dec","inc")`** — before this branch a `dir:"res"` command reaching it would have fallen
through to the DEC branch and pressed `SET−`. That was a latent bug; it is closed with a test.

`eid` (episode id) is separate from `ts` on purpose: `ts` is a re-published heartbeat (so the offer
can be withdrawn within 0.5 s of any gate dropping) while `eid` is constant for one offer (so the
one-shot press latch has a stable key). One field could not do both.

**Files**

| Repo | File | Role |
|---|---|---|
| pnw-pilot | `selfdrive/controls/lib/madsresume_pnw.py` | **the brain** — pure, no I/O, every gate |
| pnw-pilot | `selfdrive/selfdrived/selfdrived.py` | `_mads_resume_step()` — all the I/O |
| pnw-pilot | `selfdrive/controls/lib/ces_pnw/ces_pnw.py` | `log_mads_resume()` → `ces_events.jsonl` |
| pnw-pilot | `selfdrive/controls/lib/pnw_vehicle.py` | `mads_resume` capability |
| pnw-pilot | `common/params_keys.h` | `MadsResumeTarget` (the auto-resume key was removed in onetoggle2pnw) |
| pnw-pilot | `selfdrive/ui/layouts/settings/toggles.py` | "Auto-resume after brake" |
| pnw-opendbc | `opendbc/car/ford/icbm_pnw.py` | `ResumeCommand`/`parse_resume_cmd`/`decide_resume`/`ResumePress` |
| pnw-opendbc | `opendbc/car/ford/carcontroller.py` | `_resume_button()` + the `create_button_msg(resume=True)` send |
| pnw-opendbc | `opendbc/car/pnw_vehicle.py` | `mads_resume` capability (opendbc mirror) |

The brain lives in **selfdrived**, not plannerd, because selfdrived owns `self.mads` (the MADS state
machine that defines the whole trigger), `CS`, `self.events` and a `radarState` subscription it
already holds — no new subscription anywhere (the `feedback-no-carstate-sub-in-background-procs`
lesson).

## The gates, and why each threshold

Every one is a hard refusal; none is a "soft preference". `#` maps to the requirement list.

| # | Gate | Value | Why this value |
|---|---|---|---|
| 0 | `DisengageOnBrake` OFF | implied by gate 1 | `lateral_only` can only be true when this is OFF, so the arm edge already carries it. Removed as a separate control in onetoggle2pnw. NOT an instant kill switch (`needs_restart`) — use the pedals to abort a resume in the moment. |
| 8 | `madsState.available` | required | The panda's own contract (`alternativeExperience`), not a fingerprint. False on the Raven and on any unflashed panda → the brain does not even log. |
| 1 | rising edge of `madsState.lateralOnly` | — | The ONLY arm trigger. A plain disengage (no MADS latch) never arms; a `lateralOnly` that ends aborts (`latOff`). |
| 2 | `brakePressed` **and** `regenBraking` both false | — | "Fully released" includes regen: on an EV, foot-off-friction-brake is still deceleration. Enforced twice — the release clock only starts on full release, *and* a re-press after a release aborts (`reBrake`). ⚠️ **Honest caveat:** `regenBraking` is **never assigned in `opendbc/car/ford/carstate.py`** (only GM populates it), so on the Lightning — the only car this runs on — the regen half is **vacuous today**. It is kept because it is correct for any car that does report it and costs nothing, but the *effective* gate on this truck is `brakePressed` alone. |
| 1 | brake down at the arm tick | — | Defence in depth: `mads_pnw` only ever raises `lateralOnly` on a frame where `braking` is true (both the immediate and the brake-grace arm test it), so this is unreachable today. It exists so a *future* MADS arming path cannot silently hand this feature a non-brake episode (`noBrake`). |
| 3 | window after release | **0.5 – 3.0 s** | **0.5 s min**: the PCM's `CcStat_D_Actl` 4/5→3 and the driver's foot both need to settle, and firing on the release tick would fire on a brake *bounce*. **3.0 s max**: ~90 m at highway speed — the situation that caused the brake is still the same situation. Past it the driver has demonstrably chosen not to resume, and a resume becomes a surprise. |
| 3 | arm lifetime | **20 s** | `lateralOnly` can persist indefinitely; a pending resume must not. Bounds "a resume arriving minutes after the brake". engagegoal2pnw: once it expires, the **accelerator** may still open a SET-mode episode while steering-only (owner goal 2026-09-13, "hit the gas pedal once ... set the new speed"); RESUME stays bounded by it. **First press only** (owner decision 2026-09-13): no time limit, but only the FIRST accelerator press after a brake may set the speed. A press is used once its lift-off is judged (both pedals up ≥ `RELEASE_MIN_S`), whether it set or a gate refused it; shorter lifts are modulation within the same press. Only a new brake press re-arms it. |
| 4 | once per brake event | latch | Set at offer start, cleared **only** by a disarm, which requires `lateralOnly` to go False. A press that produced nothing gets **no retry**. Latched independently again in the executor on `eid`. |
| 5 | lead distance | **≥ 20 m** | Absolute floor: headway alone is far too permissive at low speed (2 s at 5 m/s is 10 m). |
| 5 | headway `dRel/vEgo` | **≥ 2.0 s** | Resuming hands speed back to stock ACC, which then *accelerates*. 2.0 s is at or better than stock ACC's own following distance, so re-engaging cannot ask ACC to close a gap it would not otherwise close. |
| 5 | TTC `dRel/(vEgo−vLead)` | **≥ 8.0 s** | At 30 m/s behind a 60 m lead, 8 s TTC means closing at under 7.5 m/s — we are not meaningfully overtaking. Faster closing is exactly what the driver braked for. |
| 5 | radar read failed | **refuse** (`leadUnknown`) | CLAUDE.md rule 2: an error is not a negative result. `has_lead` is three-state; `None` never means "no lead". |
| 6 | captured set speed | required, **≤ 0.75 s** old | Captured on every tick stock cruise reports enabled, so normally a few frames old at arm. Sized to the mechanism, not to a round second: MADS's brake-grace window is `MADS_BRAKE_GRACE_FRAMES` = 0.45 s (the *measured* pedal lead, during which cruise has dropped but `brakePressed` has not landed), + 0.30 s margin. Pinned to that constant by a test. |
| 6 | set speed range | **20 mph – 45 m/s** | 20 mph is Ford's own ACC minimum, so a lower "set speed" is not a real one; the ceiling is a decode-fault sanity bound. |
| 6 | live set > captured | **refuse** (`setRaised`) | Would risk resuming above what the driver set. A *lower* live reading is fine (resume can only reach the PCM's remembered set). Non-finite → `setUnknown`, also a refusal. |
| — | `vEgo` floor | **20 mph** | Same mechanism-grounded number: below Ford's ACC minimum there is no valid set speed to resume to, and a low-speed brake is stop-and-go, not the highway case this exists for. |
| 7 | aborts | `gas`, `blocked`, `opEngaged`, `accOff`, `ccOn`, `reBrake`, `armExpired`, `latOff` | Any further pedal input, any blocking event, ACC main off, cruise returning on its own, or lateral-only ending. Each ends the arm outright. |
| 7 | `selfdriveState.engageable` | required | **The gate that was missing until review.** A `NO_ENTRY` event (`resumeBlocked`, `tooDistracted`, `outOfSpace`, `stockLkas`, `speedTooHigh`, `selfdriveInitializing`) carries no DISABLE type, so it does *not* appear in `blocked` and MADS keeps holding lateral. But if our RES engages stock cruise while one stands: openpilot refuses to engage → `controlsd` sends `cruiseControl.cancel` → `mads_pnw` sees the cruise-engage edge and **revokes lateral**. Our press would have taken the driver's *steering* away. Refuses with `noEntry`. |
| — | executor: cruise engaged | **refuse** | **The critical one.** RES while ACC is engaged is a `SET+` on Ford. Enforced at the carcontroller off real CAN state, independently of the brain. |
| — | executor: offer freshness | **0.5 s**, monotonic | A *backstop*, not the withdrawal path — the executor re-reads the mem-param **every frame** while it holds a command, so a withdrawal lands within one 10 ms frame. `ts` is `time.monotonic()` (shared `CLOCK_MONOTONIC`), not wall clock: this device has a dead RTC and steps its clock on first sync, which would make a minutes-old offer look fresh. A heartbeat from the *future* (clocks disagreeing) is also a refusal. |
| 1 | brain: arm edge | three-state | The `lateralOnly` edge detector distinguishes "never observed" from "observed False", so a selfdrived restart mid-drive — or the toggle flipped on while already steering-only — cannot manufacture a rising edge and arm without a brake transition. |

## Telemetry — a silent no-resume and a silent wrong-resume are distinguishable

`ces_events.jsonl`, `"ev":"madsResume"`, written **unconditionally** (independent of `CESMode`, like
`cessteerlog2pnw`'s breadcrumb). Phases:

| `phase` | Meaning |
|---|---|
| `arm` | A brake press left MADS steering alone; the window is open. `setMs` null ⇒ gate 6 already failed. `reason:"gas"` (engagegoal2pnw) ⇒ no episode was open and the driver pressed the accelerator while steering-only: a **SET-mode** episode, never RESUME. `reason:"gasReopen"` ⇒ an episode that had already made its attempt was reopened by the accelerator. |
| `refuse` `gasSpent` | (engagegoal2pnw) A later accelerator press in the same steering-only stretch was ignored (first press only). `gate` names what refused the first press if its window was still open (then this is that arm's terminal record), else null. Every record carries `gasSpent`. |
| `lift` | (engagegoal2pnw, non-terminal) The driver went back on the accelerator while a lift-off window was open and refusing; `reason` is the gate that held it (`slowing`, `decelUnknown`, `slow`, a lead gate…), `liftS` how long the foot was up. Before this the refusal was overwritten by `gas` and left no record. |
| `fire` | Every gate passed; the offer is on the wire. **Exactly one per arm.** |
| `refuse` | The arm ended without a resume; `reason` names the binding gate. **Exactly one per arm**, mutually exclusive with `fire`. |
| `offerEnd` | The offer was withdrawn; `reason` is `expired` or the gate that cut it. |
| `verify` | What the set speed actually came back at. `setHigher` (+`"loud":true`, + `cloudlog.error`) is the one that matters; `setLower` (also loud) means the PCM did *not* restore its remembered set — not dangerous, but not what this design assumes either; `noCruise` means the press produced no re-engagement at all (+`cloudlog.warning`, because that is also the signature of the executor being pinned without the matching panda safety gate). |

Fields on every record: `setMs`, `setAgeS`, `stockSet`, `vEgo`, `lead`, `dRel`, `vLead`, `ttc`,
`hdwy`, `relS`, `armS`, `brk`, `regen`, `gas`, `ccOn`, `ccAvail`, `blocked`, `latOnly`, `opEn`,
`engbl`, `eid`, `fired`, plus the usual `car`/`cesMode`/GPS stamp.

**Reading it:** *no records at all* for a brake event ⇒ the brain never armed (MADS off, toggle off,
wrong car). A record with `"fired":false` and a `reason` ⇒ an **explained** no-resume. Those two can
never be confused, and neither can be confused with a resume that fired.

Failure paths are loud too: a repeated exception in `_mads_resume_step` escalates to
`cloudlog.exception` with a consecutive-failure count ("auto-resume is NOT deciding"), and a failed
that param read logged and fell back to OFF; the path is gone with the toggle (onetoggle2pnw).

## Verification

* **56** brain tests + **46** executor tests (+ 45 pre-existing `test_icbm_pnw.py` still green,
  113 for the whole collectible `opendbc/car/ford/tests` tree), all listed by name in the branch
  commit messages. Four `opendbc/car/ford/tests` modules cannot be collected on the dev host at all
  (missing `Crypto`/`hypothesis`) — verified identical on the base commit, i.e. pre-existing.
* **31 mutation cases**, one per gate (plus one that removes gate 2's *both* enforcement points at
  once): every mutant killed by the specifically-named test for that gate. Four first-pass results
  that killed the *wrong* tests were run down rather than papered over — one was a stale-`.pyc`
  artefact in the harness itself, one was a test whose baseline had absorbed the mutation, and two
  were gates that turned out to be genuinely double-guarded; the tests were strengthened and the
  redundancies documented rather than the expectations relaxed.
* A **cross-repo seam check**: the exact payload `selfdrived` publishes, run through the executor's
  real parser and decision, yields exactly one 10-frame press — and **zero** presses with cruise
  engaged. The seam is also pinned from both sides by tests
  (`test_the_published_wire_contract_matches_what_the_executor_parses` /
  `test_parses_a_well_formed_offer`), because a key rename would otherwise fail *silently*.
* **A real bug was found by the tests, not by review**: `isfinite(x) and x > threshold` silently
  *permitted* on a NaN/inf reported set speed, in **both** the brain and the executor. Both now fail
  closed (`setUnknown` / `False`).

### Review findings and what was done about them

Gemini (`gemini-flash-latest`) and Fable both reviewed the full diff adversarially, attacking the
four axes the owner named: firing outside the bounded state, a stale/wrong "previous set speed",
the once-per-event latch, and resuming above the driver's set speed.

| Finding | Verdict | Action |
|---|---|---|
| **Executor kept pressing off a CACHED command for up to 250 ms after the brain withdrew the offer** (flat 4 Hz mem-param poll vs a 0.5 s freshness bound) — defeats the brain's whole fast-abort design | **Real, the most serious finding** | Executor now re-reads `MadsResumeTarget` **every frame** whenever it holds a command (4 Hz only while idle). Withdrawal latency: 250 ms → 10 ms. Pinned by a source-level test + a mutation case. |
| **Wall-clock `ts` across the process boundary** — a dead-RTC clock step could make a stale offer look fresh, or kill a live one | **Real** | The resume heartbeat is now `time.monotonic()` on both sides, plus an explicit refusal on a *negative* age (clocks disagreeing is an unreadable input, not "extra fresh"). The dec/inc set-speed path keeps wall clock and is not touched. |
| **First-tick edge detector arms without a brake transition** (selfdrived restart mid-drive, or the toggle flipped on while already steering-only) | **Real** | `_lat_prev` is now three-state (`None` = never observed), and the inert branch *seeds* the detector from the current observation instead of forcing `False`. Two complementary fixes covering two different entry paths; each has its own test and its own mutation case. |
| **Set-speed capture race**: driver cancels cruise on the stalk, brakes a beat later, and we resume to the speed they just cancelled | **Already blocked one layer up** — a stalk CANCEL is not in `mads_pnw.MADS_TOLERATED_EVENTS`, so `blocked` is True at the falling edge and MADS refuses to arm at all. Reported here rather than dismissed, because the *bound* was still loose. | `SET_MAX_AGE_S` tightened 1.0 s → 0.75 s and re-derived from `MADS_BRAKE_GRACE_FRAMES` (0.45 s) + margin, pinned to that constant by a test so it cannot drift. |
| **`except Exception: pass` in `log_mads_resume`** would make the feature's only forensic channel fail silently | **Real** (CLAUDE.md rule 2) | Now `cloudlog.exception` with an explicit "this drive's auto-resume forensics are INCOMPLETE". Disk-level append failures were already loud via `_append_event`'s consecutive-failure escalation. |
| Payload confusion between the resume path and the dec/inc set-speed path; the RES-while-engaged SET+ hazard | **No finding** — both reviewers judged the separation watertight (dual independent `cruise_enabled` checks, disjoint parsers, disjoint decision functions) | — |
| Once-per-brake-event latch | **No finding** — brain `_done` + executor `_used_eid` judged robust against a double press | — |
| Redundant `_armed_set is None` re-check in `_gates()` | NIT, deliberate defence in depth | Kept, documented |
| **A standing `NO_ENTRY` was not checked at all** — our RES engaging cruise into one makes openpilot refuse, `controlsd` cancel, and MADS revoke lateral: *our press takes the driver's steering away* | **Real, and the highest-value finding of the whole review** | New `engageable` input and `noEntry` gate, aborting and withdrawing an offer in flight; `engbl` added to every record; own test + mutation case. |
| Brain gated only on `madsState.available`, not on `PnwVehicle.mads_resume` — with Alpha Long on the executor is structurally inert, so the brain would fire into a mem-param nobody reads and log `noCruise` every brake | **Real** | The brain is constructed only when `PnwVehicle(CP).mads_resume` holds; both halves now gate on the same capability. |
| `verify` only flagged a come-back *above* the capture; a come-back well *below* it (the PCM setting a new speed rather than restoring its remembered one) was logged as `"ok"` | **Real** | New `setLower` reason, also loud. |
| Executor construction / mem-param read failures were silent — a dead executor looks exactly like "the gates refused" | **Real** (rule 2) | `carlog.error`/`carlog.exception` at construction and on repeated read failures (throttled); pinned by a test. |
| `verify noCruise` had no log at all — and it is also the exact signature of the executor pinned *without* the panda safety gate | **Real** | `cloudlog.warning` naming that suspicion; the pin is now a hard deploy gate (see open items). |
| `CESStub.log_mads_resume` was a silent `pass`, so a CES construction failure would erase all resume forensics for the drive | **Real** | Warns once per process. |
| A `round(now, 3)` heartbeat can sit up to 0.5 ms in the *future*; a hard `dt < 0` refusal would burn the single press for the whole brake event | **Real** | 20 ms tolerance (`RESUME_FUTURE_TOL_S`), still far below any real clock disagreement. |
| `ResumePress` compared `eid ==`; `<=` is strictly stronger against a replayed older offer | NIT, free | Taken. |
| No check that the brake was actually down at the arm tick | LOW, unreachable through today's `mads_pnw` | `noBrake` guard added anyway, so a future arming path cannot bypass the premise. |
| Doc claimed regen handling that is vacuous on Ford; the `cancel` open item's reasoning was wrong; a row was mislabelled; a code comment and the test counts were stale | **Real** | All corrected; the regen caveat is now stated explicitly rather than implied. |

## Known open items

* **Not driven.** Every number above is reasoned, none is field-calibrated.
* **`radarState` liveness on the Lightning is unverified by me.** If it is not alive/valid, gate 5's
  input is missing and the feature refuses every time with `leadUnknown` — safe, and visible in the
  log, but it would mean the feature never fires. Check for `"reason":"leadUnknown"` on the first drive.
* **Whether `Veh_V_DsplyCcSet` holds the set speed in ACC standby is unverified.** The design works
  either way (0 ⇒ fall back to the latched capture; non-zero ⇒ additionally cross-checked), but which
  path is taken on the real truck is unknown until a drive. `stockSet` in the log answers it.
* **The pnw-pilot submodule pin is deliberately NOT bumped** (it is byte-identical to `3devpnw`'s:
  `opendbc_repo` `0dae2c85`, `panda` `c5e431e1`). Nothing runs on the car until the owner pushes
  `madsresume-exec2pnw` (which already carries his `32049e5f` safety gate as its parent) and bumps
  the pin himself — the deliberate seam, since bumping it is what puts a new panda safety build in
  front of the car.
* **HARD DEPLOY GATE:** the pin must be the *merged* set. `madsresume-exec2pnw` is stacked directly
  on `32049e5f`, so pinning that branch tip gets both halves. Pinning the executor **without** the
  `ford.h` gate would make **every** press a TX violation the panda drops silently — the only trace
  would be a `verify noCruise` record, which is why that record now also emits a `cloudlog.warning`
  naming exactly this suspicion.
* The `CC.cruiseControl.cancel` branch takes priority over the resume branch in the carcontroller's
  elif chain, so a cancel in the same frame burns the offer's `eid`. Conservative direction (no
  resume). **An earlier version of this bullet claimed such a cancel was unreachable; that was
  wrong** — the `noEntry` gate above exists precisely because it *is* reachable (our own press
  engaging cruise into a standing NO_ENTRY is what produces the cancel). With that gate in place the
  situation should no longer arise, and if it does the cancel wins and nothing is pressed.
* **Documented residual (bounded, one step):** if the driver presses the stalk RESUME themselves in
  the ~10–30 ms it takes `EngBrakeData`/`CcStat_D_Actl` to report cruise as engaged, both our gates
  are still reading the pre-engage value, so our press could land while ACC is *actually* engaged
  and act as a `SET+`. Bounded to a single +1 mph step, and caught after the fact by the `setHigher`
  verify record. Not mitigable from this side without a faster engagement signal.
* **Documented residual (inherent):** a `NO_ENTRY` that appears in the ~100–300 ms *after* the press
  has already reached the PCM cannot be un-pressed. openpilot will then refuse the engage,
  `controlsd` cancels, and MADS revokes lateral — the exact sequence the `noEntry` gate prevents
  *before* the press. The window is one CAN round-trip and cannot be closed from this side.

## Related

`MADS2PNW.md` (the lateral authority this completes) · `ICBM2PNW.md` + `SPEEDADJUST-EXECUTOR.md`
(the brain/executor pattern followed here) · `DEVICE-STATE.md` (params).
