# CHANGELOG — PNW distribution, 2026-07-11 → 2026-07-12 (the Ford weekend)

What changed on `dirkpetersen/pnw-pilot` **`3devpnw`** (+ `pnw-opendbc` **`master-pnw`**) over the
weekend, terse + pitfall-first. Tip at close: pnw-pilot **`f6db6bd816`** (ces2core shadow merge ←
`b62b87aa99` docs ← `38280ee117` greenlight); opendbc `1fb73dce`. New param keys all weekend:
**only `Ces2Core` + `CESTurns`** (the final merge; params_pyx rebuild via build-on-boot); one
**panda reflash** (fordsafety2pnw, driver-authorized, controlled first drive). Everything Lightning-facing is
capability-gated via PnwVehicle — Tesla provably unchanged/neutral at every step (pinned by tests).
Drive evidence: `../drives/2026-07-11/lightning-icbm-nofire/`, `../drives/2026-07-12/{snoqualmie-
ellensburg-icbm, ellensburg-snoqualmie-westbound, tesla-redlight}/`.

## TL;DR for the driver

- **The Lightning became a real openpilot car this weekend:** BluePilot 4-signal lateral (+ hardened
  panda safety), BlueCruise cluster, highway follow control, and — on the stock-ACC arm — ICBM that
  now slows for vision curves, tracks map curves through traffic, and **gives the set speed back**.
- **The Lightning slows MORE for curves than the Tesla** (weaker EPS): field-calibrated hump
  penalty + descent/left factors, shared by VTSC and ICBM.
- **CES got three safety-grade decision fixes** (red-light guard, evidence-gated pull-away,
  stop-intent fast path) and an always-on **Green Light Alert**.
- **DM personal values left the repo** (dm-variable: strict-in-source, `dm` CLI + `/data/pnw/dm.json`).
- **The Alpha-Long-toggle→DASHCAM bug is fixed** (poisoned FW cache).

## 2026-07-11 — fordsafety2pnw: 4-signal Ford lateral + panda safety (DEPLOYED, reflash)

BluePilot (alan-polk) port: curvature+curvature_rate+path_offset+path_angle LMC/LMC2 + BP's panda
safety (opendbc `14b0caac`→`58ef4868`). **Reset latch hardened** (`19ad2728`): BP's neutral-frame
bypass was permanently armed while disengaged = a controls_allowed bypass; now gated on
controls_allowed (ramp-in feature preserved bit-for-bit). BP shipped NO tests for its own safety —
suite re-derived, 99/9000 subtests, tesla*.h 0-line diff. Doctrine: *ported safety is not validated
safety*; tests pin the guarantee, not the hole. Branch doc: `FORDSAFETY2PNW.md`.

## 2026-07-11 — the numpy engage-crash saga + card resilience

card died at engage, masqueraded all day as a boot race. Chain: crash logger (`cc8d0c544c`, manager
discards child stderr) → root cause `deba3bd4`: LateralCurvExt returns numpy.float64, capnp setters
reject it; **BP's base carcontroller casts float(), stock doesn't** (fork-lineage trap). Fixes:
float casts; LateralCurvExt exceptions → permanent stock-lateral fallback, fault-injection proven
(`723e6b29`); card `restart_if_crash=True` (`47be478339`); **engaged-path smoke test** now a
mandatory gate for Ford lateral changes (motion-gated crashes false-pass at standstill).

## 2026-07-11 — icbm2pnw promoted + redlight2pnw SAFETY

ICBM promoted for the op-long vs ICBM A/B (Alpha-Long = the switch, `6ee4db4464`); honest shadow
display (grey `>> SHADOW`, always-visible ICBM status, `379b34c32c`); yellow CES-auto icon +
closed-loop telemetry `icbmT/icbmC/stockSet/stockOn` (`7a8248a0eb`). **redlight2pnw** (`c7ce79773f`):
a red light + no lead + high set looked like an on-ramp merge → CES fell to Chill and accelerated AT
the light (4× in 5 min live) — accel-zone now dead under model_should_stop and requires ≥8 m/s
no-lead. **pnw_vehicle capability view** (`4a059e5069`): no fingerprint checks in feature code.

## 2026-07-11 → 12 — curveslow-lightning + descentcurve2pnw + icbmalign2pnw (DEPLOYED)

91 takeovers analyzed → per-car curve penalty (`PnwVehicle.curve_speed_penalty_ms`), ICBM gains a
VISION curve source, speed-limit floor lowered by the car's own penalty, then iteration-3 field
calibration: **penalty is a HUMP not a ramp** (1 mph slow corners / 5 mph in the 45–62 mph washout
zone / taper 1.5 mph by 75+). Descent guard (5% grade = +40%), left factor 1.15, overspeed friction
escalation; ICBM full 500 m map horizon + firm-decel far candidate + map_scale 0.92. All knobs in
`/data/pnw/curve.json` (defensive loader; **calibrated values are now source defaults**, the device
file was removed). Washout regression registry (`tools/washouts.py`) pins ≥3 mph improvement at all
27 binding clusters. Branch doc: `CURVESLOW2PNW.md`.

## 2026-07-11 → 12 — fpcache2pnw: Alpha-Long-toggle→DASHCAM fixed

Poisoned-cache root cause (`DASHCAM_TOGGLE_BUG.md`): a sleepy-bus partial FW set rescued by the
fleet-VIN fallback was persisted to `CarParamsCache` → every later card restart went MOCK with the
truck ON. Fixes: fixed-source fingerprints never cached (`acbe87faea`); cached-miss → ONE live
requery (opendbc `a0781fac`, swap guard preserved); MOCK sessions can't wipe
AlphaLong/ExperimentalMode prefs. 11 new tests, Gemini clean.

## 2026-07-12 — dm-variable merged (+ UI lockdown)

Strict-in-source DM tiers (highway 30/60 road-gated, relaxed 60/120 opt-in); long personal values
ONLY in `/data/pnw/dm.json` via the `dm` CLI (clamp [10 s, 4 h]; source-purity test). UI: Relaxed
INVISIBLE unless unlocked via the CLI; help text = Default+Highway only, numbers printed from source
constants. `DmMode` param semantics changed accordingly. Docs: `DM-CURRENT.md` (rewritten),
`pnw-pilot:DM-VARIABLE.md`.

## 2026-07-12 — ICBM: restore → map-first → set-tracking (all merged same day)

Three field-driven iterations off the Snoqualmie↔Ellensburg legs:
1. **icbmrestore2pnw** — guarded SET+ restore (episode SM cap→hold→restore, hard-abort matrix, DEC
   always wins; executor `RestoreGuard`, no panda change). Gemini found 3 real intent conflicts →
   all fixed by STRENGTHENING guards.
2. **icbmmapfirst2pnw** — maps are the anticipatory authority (vision initiates only beyond
   `mapReach` + ≥2.75 s); no new episode while lat-loaded; apex-passed restore 3 s→1 s; the
   18:08:26Z failed-restore was a driver-lower guard FALSE POSITIVE from ~1 s set-reporting lag →
   anchored one-time late-tap absorption. Telemetry: `icbmGate`, `mapReach`.
3. **icbmtrack2pnw** — root cause of the 19:58:37Z lane-change-into-bend: tiered sweeper scale ×1.8
   inflated a rated 64.9 mph curve to eff 107 → **ICBM-only map-scale cap 1.35** (field-checked
   against both legs); **continuous set-tracking** walks the set down while lead-bound (window ≤
   350 m) so a lane change can never launch the truck into a bend; silence now means broken.
   153 tests, Gemini "airtight". Doc: `ICBM2PNW.md` (rewritten).

Also: `vtsctele2pnw` lead/penalty forensics in every tick (`lead/gapS/dV`, `vtscPen/Pitch/Dir`).

## 2026-07-12 — fordlong2pnw + the city-hop fix

BP LongitudinalExt (lead classification, follow-mode rate limit + TTC bypass, split brake/precharge
hysteresis, 50→45 mph deadband) — inert until Alpha Long; 3 Gemini divergences (mild-decel-only
suppression so VTSC/CES braking survives; cross-scan hysteresis latch; rate limiter lead-only).
Field: city "hopping like a horse" → the BP acc builder owned the brake bit at ALL speeds; fixed by
gating the BP message on `bp_long_used` (opendbc `1fb73dce`). Branch doc: `FORDLONG2PNW.md`.

## 2026-07-12 — CES decision core: pullaway + stop-intent fast path

**pullaway2pnw**: lead-pull-away exception below the 8 m/s floor (lead present + genuinely opening +
no stop intent + moving) — Gemini adversarial round found the occlusion+cooldown lockout → branch
held. **stopintent2pnw** closed it as driver-approved policy: model_should_stop + ladder-wants-Exp
preempts EVERY entry-side timer in one cycle (reason `stopIntent`); exits keep full dwell (churn
bounded toward stopping only); also closes the pre-existing 5 s stop-blind window. Combined 176
tests, Gemini clear-to-ship. Cross-car by design (decision core is generic).

## 2026-07-12 — greenlight2pnw + ball2pnw + CES2 (study → shadow core)

**Green Light Alert** (`94048354bf`, always-on, both cars): ding + green banner when a held stop
releases — sunnypilot mechanics + FrogPilot stop-context arming, display/sound only; own CREDITS
section. **ball2pnw** (`cc0c33d752`): comma-4-style model-confidence ball (sunnypilot C3X port).
**CES2-STUDY.md** (branch `decstudy2pnw`, study only): CES vs DEC vs CEM, graded stop-urgency
redesign + rule retire-list; `mdlEndX` now logged every tick for the replay acceptance dataset.
**ces2core2pnw MERGED late 07-12** (`aa9e93e322`/`f6db6bd816`): the CES2 decision core — graded
stop-urgency (mdlEndX vs speed-indexed DEC table, instant rise / ~2 s hysteretic decay), the
precedence principle (stop evidence outranks accelerate-preference at any speed; retires the 8 m/s
floor, pull-away band + accel-zone wrappers INSIDE the CES2 core, ~19 rules → 12, with a
blind-endpoint fallback restoring the floor where the signal can't see — Gemini Trap-1 catch), CEM
adoptions (standstill hold, `CESTurns` blinker condition default OFF, per-condition debounce) —
**SHADOW-ONLY behind `Ces2Core` default OFF**: v1 decides byte-identically (pinned by the 176-test
suite; 201 total), CES2 logs `ces2Mode/Reason/Urg/Div/Live` every tick; replay harness
`tools/ces2_replay.py` (three acceptance windows replayed, 0–5.9% divergence). Gemini verdict: NOT
road-ready ON until the stop table is re-anchored on lebowski mdlEndX shadow drives — the flag
stays OFF. **Still in-flight:** `stophold2pnw`, `wazeproxy2pnw` (branches cut, no commits yet).
**BRANCH-ONLY:** `decstudy2pnw` (the study itself).

## ⚠️ OPEN ISSUE — Tesla red-light lurch (2026-07-12 drive)

Reported as "CES went silent"; investigation proved otherwise
(`../drives/2026-07-12/tesla-redlight/CES_SILENCE_REPORT.md`): CES coverage was dense — the lurch is
CES-*mediated*, a structural hole (stop condition gated on `not has_lead` + the lowSpeed 1.0 m/s
floor) that adopted Chill at 0.4 m/s behind a creeping lead; Chill MPC launched toward the set. Fix
proposals written, NOT implemented. Bonus findings: device RTC battery dead (bogus clock until NTP
after cold boots); `CESController` constructed outside try/except in selfdrived (hardening candidate).

## Ops / deploy-discipline changes (pnw-pilot-deploy skill)

- **Pre-drive sync rule (OVERRIDING, driver directive):** before EVERY drive push + install latest,
  verified; metered gate is for video uploads only, never code; NEVER hot-patch — urgent path is
  commit→push→manual git install of the SHA (`e79c5d204f`).
- **CODE-vs-SETTINGS reload rule** (`63e46c14e8`): manager preimports at device boot ⇒ code changes
  need a reboot; settings/JSON reload on ignition cycle; version label lags files-only installs.
- **Aggressive-reboot policy** (`e694ca58bb`): pending state → auto-reboot at first gear==park,
  unprompted. UI shows red **LONG MISMATCH** when toggle vs running long path disagree.
- **Panda safety-C deploy discipline** documented (`ec8cfad29a`); 2026-07-11 field lessons
  (numpy/fork-lineage, motion-gated crashes, crash forensics) recorded (`4a576a0f06`).
