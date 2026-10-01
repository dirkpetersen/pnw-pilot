---
updated: 2026-09-30          # git-derived; bump when you edit this file
status: current        # current | drifted | superseded | unreviewed
---

# MADS2PNW — "lateral survives a brake press"

**Branches:** `mads2pnw` (panda safety C + the toggle), `madsop2pnw` (the openpilot-side
engagement state machine) and `madsheartbeat2pnw` (the lateral watchdog + the health-packet field)
in `dirkpetersen/pnw-pilot`; `mads2pnw` + `madsheartbeat2pnw` in `dirkpetersen/pnw-opendbc`;
`madsheartbeat2pnw` in `dirkpetersen/pnw-panda`.
**Status (updated 2026-09-30): MERGED TO `3devpnw` AND LIVE.** The text below was written 2026-09-05, when this
was built-but-inert pending a panda flash; the "do not merge", "pre-flash blocker" and flash-procedure sections
are history. Evidence: `madsheartbeat2pnw` merged to `3devpnw` as `b1a1a8fc90` (2026-09-06), `madsresume2pnw`
`7569781e31` (09-06), `madsbrake2pnw` `3c7592b616`/`668607cab5` (09-24); `PandaMadsSafety` is a key in
`common/params_keys.h` and `card.py` sets the MADS bits for cars with the `mads_lateral` capability (Lightning, and
the Tesla Raven since `teslamads2pnw` `c53bae6ed8`, 2026-09-28; tested by `test_mads_alternative_experience.py`).
On-car: the workbench `DEVICE-STATE.md` (not in this repo) records `alternativeExperience` 1024 verified after the
09-24 install and the Raven's internal panda reflashed 09-28; not re-verified by this edit.

## The problem

The F-150 Lightning runs **stock ACC**. Speed is the truck's; openpilot only steers. So a brake
tap disengages stock cruise, openpilot disengages with it, and the driver loses **everything** —
including the one thing openpilot was actually doing. On the Tesla Raven this doesn't arise the
same way (openpilot owns longitudinal there, and the Raven's EPS self-inhibits via
`EAC_INHIBITED` on a brake press anyway, so this feature would not help it).

## The design, and why it is not a weakening

Ported from sunnypilot's **MADS** (via `sunny/bluepilot vin-lightning-2024-25`,
`opendbc/safety/sunnypilot/mads.h`). The trick is that upstream did **not** touch the brake check.
They added a **second, parallel authority flag** next to it:

* `generic_rx_checks()` still clears `controls_allowed` on the rising edge of the brake pedal, in
  every path, for every car. **Untouched.**
* A new `controls_allowed_lateral` is consulted **only** by lateral tx gates, as
  `(controls_allowed || controls_allowed_lateral)`.
* Longitudinal, gas, resume-button, relay: all still read `controls_allowed` alone.

So a brake press still removes longitudinal authority; it just no longer removes lateral authority.

### Opt-in chain (all four must hold)

1. `PnwVehicle.mads_lateral` — capability view, today the Lightning only. Never a fingerprint test
   in feature code.
2. `PandaMadsSafety` — an explicit declaration that the **currently flashed** panda carries this
   safety build (see *Honest capability detection*).
3. `set_safety_hooks()` refuses the MADS bits in **every safety mode except `SAFETY_FORD`** — the
   panda does not trust the host's capability gate.
4. `m_update_control_state()` gates the write on `system_enabled`, so a build with MADS disabled
   can never flip the global on.

`mads_set_system_state()` re-inits the whole struct on every safety-mode change, so
`controls_allowed_lateral` is cleared on every mode change — including the panda's drop to
`SAFETY_SILENT` when the heartbeat is lost.

### The toggle — inverted polarity, deliberately

**"Disengage on brake"**, param `DisengageOnBrake`, default `"0"`. Same idiom as
`DisableLaneCentering` / `NoFordAngleSteering`:

| Toggle | Param | MADS mode | Behaviour |
|---|---|---|---|
| **OFF (default)** | `"0"` | REMAIN_ACTIVE | brake takes speed, **leaves steering** |
| ON | `"1"` | DISENGAGE | stock: brake takes everything |

The param ships `"0"` (satisfying this fork's default-OFF rule) and `"0"` is *also* the new
behaviour. **That is intentional, not a slip.** `PAUSE` exists in the safety code and is not
exposed.

The toggle is **greyed and painted ON (stock)** whenever the car lacks the capability *or*
`PandaMadsSafety` is `0`. The grey-out is display-only (no `put_bool` — this is ONE physical
device moved between two cars); the real forcing is in `card.py`, which sends
`alternativeExperience = 0` whenever either condition fails.

`needs_restart` is `True` on the toggle, so flipping it requests an onroad cycle. The panda only
latches the MADS bits at safety-mode init — without the cycle the flip would silently do nothing.

## What changed

### `pnw-opendbc` (branch `mads2pnw`)
| File | Change |
|---|---|
| `opendbc/safety/pnw/mads_declarations.h` | new — state machine types, `ALT_EXP_*` defines, and the full rationale + deviations |
| `opendbc/safety/pnw/mads.h` | new — the state machine |
| `opendbc/safety/lateral.h` | 9 lateral gates → `(controls_allowed \|\| controls_allowed_lateral)` |
| `opendbc/safety/safety.h` | `mads_state_update()` once per rx after `generic_rx_checks()`; `mads_exit_controls(LAG)` on lag and on an invalid rx message; MADS bits applied at `set_safety_hooks`, Ford-only |
| `opendbc/safety/modes/ford.h` | the two angle-mode disengaged-steering guards, the angle-mode corroboration gate and `angle_mode_active` → lateral authority; `acc_main_on` rx write ported |
| `opendbc/safety/__init__.py` | `ALTERNATIVE_EXPERIENCE.ENABLE_MADS / MADS_DISENGAGE_LATERAL_ON_BRAKE / MADS_PAUSE_LATERAL_ON_BRAKE` |
| tests | `mads_common.py` mixin + Ford/Toyota/Tesla tests, libsafety hooks, `mutation.py` pin re-base |

### `pnw-pilot` (branch `mads2pnw`)
| File | Change |
|---|---|
| `selfdrive/car/card.py` | `alternativeExperience` was **hardcoded to 0**; now computed by `_alternative_experience()` |
| `selfdrive/controls/lib/pnw_vehicle.py` | new `mads_lateral` capability |
| `common/params_keys.h` | `DisengageOnBrake`, `PandaMadsSafety` (both default `"0"`) |
| `selfdrive/ui/layouts/settings/toggles.py` | the "Disengage on brake" toggle + its capability clamp |
| `selfdrive/car/tests/test_mads_alternative_experience.py` | new |

### Deliberate deviations from upstream
* **No MADS button, no ACC-main-rising engage.** The *only* thing that ever sets
  `controls_allowed_lateral` is the rising edge of openpilot's own `controls_allowed`.
* **Added beyond upstream:** openpilot losing controls for any **non-brake** reason (CANCEL, a
  fault, anything) also drops lateral. Upstream can leave the latch standing because it has an
  openpilot-side MADS state machine and the `heartbeat_engaged_mads` watchdog; this tree has
  neither yet, so without this a CANCEL press would leave the panda permitting lateral forever.
* **`acc_main_on` rx write ported** (bp-7.0 line the angle2pnw port had skipped) so that turning
  ACC MAIN off is a real, driver-reachable revoke. A brake press drops `CcStat_D_Actl` 4/5 → 3
  (standby, main still on) and correctly does **not** revoke.
* **Also clears lateral on an invalid rx message** (`is_msg_valid`), where upstream clears only
  `controls_allowed`. Strictly narrower.
* **The Ford reset-bypass latch keeps its plain `controls_allowed` gate.** That latch grants
  *amnesty from rate-of-change checks*; it does not select limits. Extending it to the MADS flag
  would re-open the 2026-07-11 hole (openpilot streams neutral frames while longitudinally
  disengaged, which would keep the amnesty permanently armed). MADS therefore runs with the ROC
  checks fully enforced, every frame.
* **`mads_state_update()` placement.** Called once per rx message from `safety_rx_hook`, right
  after `generic_rx_checks()`. bluepilot calls it from inside `stock_ecu_check()`, which runs once
  per relay-checked tx_msg — a variable number of times per frame, which is wrong for an edge
  detector.
* **`mads_set_alternative_experience()` is actually called.** In bluepilot it is invoked only from
  the test harness, so MADS there is dead code in real firmware.
* **`heartbeat_engaged_mads` NOT ported.** See below — this is a hard blocker for flashing.

## Honest capability detection — and why it is a param

The panda stores `alternative_experience` verbatim and echoes it back regardless of whether its
safety build understands the MADS bits, so **the echo cannot distinguish a MADS panda from a stock
one**. There is no version, safety-param or health field in this tree that can either. The honest
signal would be a new `controls_allowed_lateral` field in the panda health packet →
`PandaState` (as sunnypilot does), which is itself a panda-firmware change and therefore only
available *after* the flash.

So `PandaMadsSafety` is an **explicit, conservative, default-OFF declaration** set by hand as the
last step of the flash procedure. It is not inferred from anything. Until it is set, `card.py`
sends `alternativeExperience = 0` — the stock contract — and the toggle is greyed to stock.

**Note the half-state hazard does not arise in this port**: the greyed `NoDisengageOnBrake` toggle
exists because suppressing openpilot's *own* disengage against a panda that still clears
`controls_allowed` produces "UI says engaged, car does nothing", ending in `controlsMismatch`
(`ET.IMMEDIATE_DISABLE`) after 2 s. **This port does not suppress openpilot's disengage at all**
(see below), so that state is unreachable. `PandaMadsSafety` guards the converse — sending MADS
bits to a panda that cannot honour them.

## ⚠️ The three pre-flash blockers — all now CLOSED in code (the flash itself is not)

### 1. ~~The openpilot-side lateral engagement state machine is NOT ported~~ — DONE, on `madsop2pnw`
See *The openpilot side* below. Items 2 and 3 remain, and item 2 is still a hard pre-flash blocker.

## The openpilot side (`madsop2pnw`)

Ported from sunnypilot `sunnypilot/mads/{mads,state,helpers}.py` (via
`sunny/bluepilot vin-lightning-2024-25`). Without it the panda's new permission goes unused:
when the Ford PCM drops cruise on the brake, openpilot disengages, `selfdriveState.active` goes
False, `controlsd` sets `CC.latActive = False`, and no steering command is sent.

### What it is — and what it deliberately is not

`selfdrive/selfdrived/mads_pnw.py` is a SECOND, parallel engagement state, published on its own
message and consulted by its own branch in `controlsd`. It **never edits openpilot's own
disengage**:

* `selfdrived`'s state machine runs FIRST and is untouched. On a brake press openpilot still
  disengages: `selfdriveState.enabled`/`.active` both go False, `CC.enabled` goes False,
  longitudinal stops, and `mismatch_counter` — keyed on `self.enabled` — is *reset*, so
  `controlsMismatch` can never fire because of this feature.
* `mads.update()` runs immediately AFTER that, and only ever reads the frame's events. A test
  parses the module with `ast` and fails if it ever calls `.remove()` or names `latActive`.

The refused shortcut was to latch `CC.latActive`, or to suppress openpilot's own `pedalPressed`.
Either gives "UI says engaged, car is not actuating": `mismatch_counter` climbs at 100 Hz and
`controlsMismatch` hard-disables at 2.0 s (`selfdrived.py`, `events.py`). **Do not reintroduce it.**

### The gate is `CarParams.alternativeExperience`, not a param

`MadsPnw` is constructed from `self.CP.alternativeExperience` — the exact bitfield `card.py`
handed the panda — and reads **no params at all**. openpilot therefore cannot hold an opinion the
panda does not share, and the whole opt-in chain (`PnwVehicle.mads_lateral` + `PandaMadsSafety` +
the `DisengageOnBrake` polarity) is inherited from `card.py` rather than re-derived. With
`PandaMadsSafety=0` that bitfield is `0`, `available` is False, and every consumer falls back to
`selfdriveState` — today's behaviour exactly.

`MADS_PAUSE_LATERAL_ON_BRAKE` is **refused**: if the bit is ever set, MADS logs an error and
declares itself unavailable rather than run a policy it does not model.

### The state machine — a mirror of `opendbc/safety/pnw/mads.h`

| panda | openpilot |
|---|---|
| latches on the RISING edge of `controls_allowed` | latches on the rising edge of `selfdrived.enabled` |
| no MADS button, no ACC-main engage | same — openpilot's own engage is the only source |
| clears on `controls_allowed` FALLING while `!braking.current` | clears on `enabled` falling unless `CS.brakePressed or CS.regenBraking` |
| clears on the brake rising edge when DISENGAGE is set | the falling edge never latches when `disengage_on_brake` |
| clears on ACC-main falling | `wrongCarMode` (= `not cruiseState.available`) is a blocking event |

Deliberately **narrower** than the panda in two places, both fail-to-stock (openpilot simply stops
steering where the panda would still have permitted it — which costs nothing, because openpilot is
the only thing that ever sends a steering command):

1. **Any other disabling event ends lateral, even mid-brake.** The panda's `!braking.current` test
   would let a CANCEL press *during* a brake keep the latch; here it does not. "Disabling" =
   carrying `ET.USER_DISABLE`, `ET.IMMEDIATE_DISABLE` or `ET.SOFT_DISABLE`, which is what covers
   the reverse-gear / door-open / ESP / ACC-main-off class of hazard. `ET.NO_ENTRY` is excluded
   (it gates *entering* engagement, and `belowEngageSpeed` is NO_ENTRY-only and true for long
   stretches of ordinary driving).
2. Exactly **two** events are tolerated, and a test asserts the list is exactly these two:
   `pedalPressed` (openpilot's own brake disengage — its *gas* case cannot smuggle lateral through
   because the falling edge additionally requires the brake to be down) and `pcmDisable` (the
   stock PCM dropping cruise; `car_specific.py` re-raises it EVERY frame while cruise is off, not
   just on the falling edge, which is what makes the lateral-only state last).

Releasing the brake does **not** end it. That is the point: the brake took the speed, not the
steering — and the panda's latch does not reset either.

### One divergence the mirror had to close: the panda revokes first

The panda sets `controls_allowed` on the **rising edge of stock cruise engaging**. So if cruise
comes back while openpilot does *not* engage with it — a `NO_ENTRY` is standing (calibration
incomplete after a car swap, `resumeBlocked`, distracted, …) — `controlsd` sends
`cruiseControl.cancel`, stock cruise drops, and the panda sees `controls_allowed` **fall with the
brake up**, which revokes `controls_allowed_lateral`. openpilot would see no edge of its own and
keep commanding lateral into a panda that is now blocking it: a **silent** no-steer, with no
detector at all (this tree's `PandaState` has no `controls_allowed_lateral` field, so there is no
lateral `mismatch_counter`). `MadsPnw` therefore tracks the same cruise **rising edge** and stands
down. It must be the rising edge, not a level test: right after a brake press
`cruiseState.enabled` can still read True for a frame or two before the PCM drops it, and a level
test there would kill the feature on the very frame it arms. (Found by the Fable review, which
then verified that the rising edge is the *exact* mirror and not a compromise: on the Ford the
panda's `controls_allowed` has exactly **one** rising source — `pcm_cruise_check()` at
`ford.h:512`, driven by `CcStat_D_Actl in (4,5)` — which is the same signal `carstate.py:74`
decodes into `cruiseState.enabled`. So "panda `controls_allowed` rises" ⟺ "`cruiseState.enabled`
rises", frame for frame. The one case it cannot see is a sub-10 ms `CcStat` blip that the panda
observes per-message and `card`'s 100 Hz snapshot misses — a level test would miss that too, and
only the `controlsAllowedLateral` mismatch counter of pre-flash item 3 can catch it.)

### Two consequences of "openpilot is disengaged while steering"

Both were found by review and are fixed here; both are the same class of bug — a subsystem that
keys off `selfdriveState.enabled` and quietly switches itself off exactly while the truck steers.

1. **`ET.WARNING` alerts were being swallowed.** In lateral-only openpilot's own state machine sits
   in `disabled`, whose `current_alert_types` is `[ET.PERMANENT]`, and `update_alerts()` *clears*
   every WARNING that is not in that list. `steerSaturated` ("Take Control") is still raised
   (`lac.active` is true), lane changes still execute, `belowSteerSpeed` still applies — and none
   of them would have been shown or sounded. `selfdrived.step()` now re-admits `ET.WARNING` for
   exactly the frames `mads.active and not self.active`.
2. **Driver monitoring reset every frame.** `helpers.py` fed `op_engaged` from
   `selfdriveState.enabled`, so `_update_events()` took the `not op_engaged` branch and called
   `_reset_awareness()` — i.e. zero distraction monitoring while the car steered itself.
   `dmonitoringd` now subscribes to `madsState` and `op_engaged` is
   `selfdriveState.enabled or (madsState.available and madsState.lateralOnly)`.

### Behaviour to expect on the first drive

Braking to a full stop leaves MADS latched. Pulling away manually with cruise still off means the
truck **starts steering again** above `minSteerSpeed`, with the "Steering only" banner and the grey
border but no cruise. That is REMAIN_ACTIVE working as specified (stock Ford Lane Centering behaves
the same way), but it is the moment most likely to surprise the driver — check it deliberately.

### Reading these logs with upstream tools

`madsLateralOnly @101` is a fork-local `EventName` (as `greenLight @99` / `leadDeparting @100`
already are). An unmodified upstream `logreader` will raise on `onroadEvents` from a route that
contains it — analysis needs this branch's `cereal`.

### The wire

New `madsState` message (`cereal/custom.capnp`, `CustomReserved10` @136 renamed — the same idiom
`VtscState`/`MapdOut` used): `available` / `enabled` / `active` / `lateralOnly` /
`disengageOnBrake`. `selfdrived` publishes it every frame **immediately before** `selfdriveState`,
so `controlsd` — which polls on `selfdriveState` — always finds the same frame's authority already
queued. `Controls.lat_authorised()` is the single place that reads it and falls back to
`selfdriveState.active` whenever it is unavailable, stale or invalid.

### The UI must never say "disengaged" while the truck steers

* New event `madsLateralOnly` (`ET.PERMANENT` only, so it cannot influence the state machine that
  already ran) paints a persistent **"Steering only / Cruise is off - openpilot is still
  steering"** banner.
* `ui_state.py` paints `UIStatus.OVERRIDE` (grey) instead of `DISENGAGED` while
  `madsState.lateralOnly`. Guarded on `available`, so it is dead code on every car without the
  flashed panda.

### Files

| File | Change |
|---|---|
| `selfdrive/selfdrived/mads_pnw.py` | new — the state machine (pure: no params, no sockets, no clock) |
| `selfdrive/selfdrived/selfdrived.py` | construct from `CP.alternativeExperience`; `mads.update()` after the state machine; raise the alert; publish `madsState` before `selfdriveState` |
| `selfdrive/controls/controlsd.py` | `lat_authorised()`; used by `CC.latActive` and the `steer_limited_by_safety` check |
| `selfdrive/selfdrived/events.py` | the `madsLateralOnly` banner |
| `selfdrive/ui/ui_state.py` | OVERRIDE instead of DISENGAGED while lateral-only |
| `cereal/{custom,log}.capnp`, `cereal/services.py` | the `madsState` message + the `madsLateralOnly` event name |
| `selfdrive/monitoring/dmonitoringd.py`, `selfdrive/monitoring/helpers.py` | driver monitoring counts lateral-only as engaged |
| `selfdrive/test/process_replay/process_replay.py` | `madsState` added to dmonitoringd's pubs |
| `selfdrive/selfdrived/tests/test_mads_pnw.py` | new — 64 tests |

### 2. ~~No lateral watchdog (`heartbeat_engaged_mads`)~~ — DONE, on `madsheartbeat2pnw`

`panda/board/main.c` has a 3 s watchdog for `controls_allowed`:

```c
if (controls_allowed && !heartbeat_engaged) {
  heartbeat_engaged_mismatches += 1U;
  if (heartbeat_engaged_mismatches >= 3U) { controls_allowed = false; }
} else { heartbeat_engaged_mismatches = 0U; }
```

There is now a **lateral twin**, restored verbatim from upstream:

| repo | branch | what |
|---|---|---|
| `pnw-opendbc` | `madsheartbeat2pnw` | `heartbeat_engaged_mads` + `heartbeat_engaged_mads_mismatches` globals and `mads_heartbeat_engaged_check()` in `opendbc/safety/pnw/mads.h`; the externs, the `MADS_DISENGAGE_REASON_HEARTBEAT_ENGAGED_MISMATCH = 32` reason, and `SAFETY_UNUSED` entries for the MISRA TU |
| `pnw-panda` | `madsheartbeat2pnw` | `case 0xf3:` reads `heartbeat_engaged_mads = (req->param2 == 1U)`; the 1 Hz block calls `mads_heartbeat_engaged_check()`; the heartbeat-lost block clears the flag |
| `pnw-pilot` | `madsheartbeat2pnw` | `Panda::send_heartbeat(engaged, engaged_mads)` → `control_write(0xf3, engaged, engaged_mads)`; `pandad` subscribes `madsState` and sends `available && enabled` |

**What it is for, precisely.** *Not* "openpilot died" — that is already covered: no `0xf3` at all →
`heartbeat_counter` climbs → the panda drops to `SAFETY_SILENT` → `set_safety_hooks` →
`mads_set_system_state(false,…)` → `m_mads_state_init()` → `controls_allowed_lateral = false`. The
watchdog covers openpilot **still talking but no longer intending lateral** (selfdrived restarted,
`madsState` went stale/invalid, MADS turned itself off): within 3 s the panda takes lateral back.

**It can only ever REVOKE.** The only authority write is `mads_exit_controls()`, which can only
clear; the counter only advances while the latch is *already* up; and `heartbeat_engaged_mads`
initialises **false**, so a panda that never hears from a MADS-aware openpilot revokes and stays
revoked. `pandad` sends `false` for every failure mode (`madsState` missing, stale, invalid, or
simply unavailable). **Missing means revoke.**

`madsState.**enabled**` is sent, not `.active` — `.active` additionally goes false in
`preEnabled`/`softDisabling`, where the panda's latch legitimately stays up, and sending `.active`
there would revoke the latch a few seconds into an ordinary engage. `enabled` is the exact mirror
of `controls_allowed_lateral` (and is what upstream sends). `.active ⊆ .enabled` always, so this is
the weaker, correct claim.

### 3. ~~Latch state is invisible ON THE PANDA SIDE~~ — DONE, on `madsheartbeat2pnw`

`health_t` gains `uint8_t controls_allowed_lateral_pkt`, filled with
`(controls_allowed || controls_allowed_lateral)` — exactly the expression every lateral tx gate in
`opendbc/safety/lateral.h` evaluates. It reaches openpilot as `PandaState.controlsAllowedLateral`
(`log.capnp @38`), and `selfdrived` counts a `lateral_mismatch_counter` over it, mirroring
`mismatch_counter`, raising `madsControlsMismatchLateral` after 2 s. That event carries
**`ET.IMMEDIATE_DISABLE` *and* `ET.PERMANENT`**, and the second one is load-bearing: in the
lateral-only state openpilot's own state machine sits in `disabled`, whose `current_alert_types`
is `[ET.PERMANENT]` **only**, so an `IMMEDIATE_DISABLE`/`NO_ENTRY`-only event would produce no
text and no sound at all and the driver's only cue would be the "Steering only" banner quietly
vanishing. The `PERMANENT` entry is a full critical alert with `warningImmediate`, 4 s (the event
itself fires for a single frame — it ends the lateral-only state, which resets the counter).

The panda also publishes **why** it revoked: `health_t.mads_disengage_reason_pkt` →
`PandaState.madsDisengageReason` (`log.capnp @39`), the `DisengageReason` enum from
`mads_declarations.h` (BRAKE / LAG / ACC_MAIN_OFF / OP_DISENGAGE / HEARTBEAT_ENGAGED_MISMATCH /
STEERING_DISENGAGE). Diagnostic only — nothing reads it for control — and added in the *same*
health-version bump so it does not cost a second flash.
Because that event carries a disable type, `mads_pnw.has_blocking_event()` ends the lateral-only
state, so openpilot stops steering into a panda that is already blocking it — instead of the
silent no-steer this used to be.

The counter is pinned to zero unless MADS is **available AND holding lateral alone AND openpilot
itself is disengaged**, so it can never disengage a normally-engaged car. On any panda without the
MADS safety build `controls_allowed_lateral` is permanently false and the field simply equals
`controlsAllowed`.

#### The health packet is a versioned wire struct — what happens on a mismatch

`HEALTH_PACKET_VERSION` is a sha256 over `panda/board/health.h`, computed identically by
`panda/python/constants.py` and `panda/board/SConscript`, so **it bumps itself**:
`0xf63f9de2` → `0xd18411db`. There is no hand-maintained constant to forget. The struct grew
59 → **61 bytes** (two new `uint8_t`), still under `USBPACKET_MAX_SIZE` (64), so
`get_health_pkt`'s `COMPILE_TIME_ASSERT` holds.

* **NEW openpilot + OLD panda** (the one that WILL happen, because the panda is flashed
  separately):
  * `pandad.py` at boot compares the flashed firmware signature to the built binary and
    **reflashes** — so on a normal device this state is transient.
  * If it is not reflashed, `pandad`'s `connect()` already throws *"Panda firmware out of date"*.
  * If even that is skipped (`BOARDD_SKIP_FW_CHECK`), `Panda::get_state()` now rejects the short
    `0xd2` read (`err != sizeof(health)`) with a named `LOGE` instead of keeping the
    zero-initialised tail — which would have been a fabricated `controlsAllowedLateral = false`.
    `connect()` reads once at startup and throws a message that names the cause. Two guards on
    that throw:
    * it is gated on a `health_packet_mismatch` flag that only a **short read** sets, so a
      transient SPI/USB failure can never be misdiagnosed as a version mismatch;
    * it is gated on `is_supported` — **the same gate the firmware-signature check uses** — so the
      set of pandas it can refuse is exactly the set already refusable today. This matters because
      **the Raven runs a SECOND panda** that `pandad.py` flashes from a checked-in prebuilt
      (`selfdrive/pandad/fw/panda_f4.bin.signed`, *not* rebuilt from `panda/`) or skips outright,
      and can therefore legitimately run an older `health_t` forever. `get_state()` correspondingly
      does **not** return `nullopt` on a short read — doing so would stop `pandaStates` and the
      heartbeat for the *whole car* and crash-loop `pandad`, taking the Raven off the road over a
      Ford-only feature. It logs once and returns the packet with its zeroed tail, which is the
      fail-safe value ("lateral not permitted", "no disengage reason") and is ignored anyway
      because that panda sits in a safety mode in `IGNORED_SAFETY_MODES`.
  * pypanda (`@ensure_health_packet_version`) raises *"health packet version mismatch: panda's
    firmware vX, library vY. Reflash panda."*
* **NEW panda + OLD openpilot** — the old build asks for the shorter struct.
  * Over **USB** the firmware truncates (`panda/board/drivers/usb.h:588`,
    `MIN(resp_len, wLength)`) and the new trailing bytes are simply never read.
  * Over **SPI — the 3X's actual transport — it does NOT truncate.** The firmware returns
    `comms_control_handler()`'s full length with no `MIN(…, spi_data_len_miso)`
    (`panda/board/drivers/spi.h:146`), and the host copies `rx_data_len` bytes into the caller's
    buffer without checking `max_rx_len` (`selfdrive/pandad/spi.cc:394`). An old `pandad` would
    take a **2-byte stack overwrite** past its 59-byte `health_t`.
  * In practice `pandad.py` reflashes the panda down to the old firmware on the signature mismatch
    **before** the old `pandad` C++ ever connects, so this is not reachable on a normal boot — but
    it IS reachable with `BOARDD_SKIP_FW_CHECK`, and it is not "harmless".
  * ⚠️ **FLAGGED, NOT FIXED (pre-existing, out of scope for this branch — CLAUDE.md Rule 3):**
    `PandaSpiHandle::spi_transfer()` should bound `rx_data_len` by `max_rx_len` before the
    `memcpy`. Fixing it here would not help the case above (the vulnerable code is the OLD build,
    which by definition does not have the fix) and it touches the CAN read hot path, so it wants
    its own change with its own tests.
* **The direction that matters is never silent.** New openpilot meeting an old panda is always a
  refusal, never a mis-parse. The reverse direction is bounded by `pandad.py`'s reflash.


## The panda flash procedure (owner's call, with the cars present — NOT done here)

Blockers 1–3 above are now closed in code. **The flash itself is still the owner's call**, because
the same device is moved between a single-panda Lightning and a **dual-panda Raven**, and a
mismatched matched-set has previously left the Raven's aux panda in DFU.

### ⚠️ Merging this to a channel IS the flash

`pandad.py` compares the flashed panda's firmware signature against the binary built from the
checked-out `panda/` submodule at **every boot**, and reflashes on a mismatch. So promoting these
branches to `3devpnw`/`3testpnw` does not "stage" a flash — the next boot performs it,
unattended, on whichever car the device is plugged into. That is why these stay on feature
branches until the owner is present.

### Pin-bump order (the matched set)

`panda/board/` and `opendbc/safety/` share `CANPacket_t`, and the firmware embeds
`CAN_PACKET_VERSION_HASH` computed over it; `health_t` now carries its own
`HEALTH_PACKET_VERSION`. A panda flashed from one pair and driven by an openpilot pinned to a
different pair refuses to talk. Order matters:

1. **Push the companions first**, in either order — nothing depends on them yet:
   * `pnw-opendbc` `madsheartbeat2pnw` → merge into `master-pnw` (or the SHA the channel pins)
   * `pnw-panda` `madsheartbeat2pnw` → merge into `master-pnw`
2. **Bump BOTH submodule pins in ONE `pnw-pilot` commit.** Never bump one and not the other: an
   opendbc with `mads_heartbeat_engaged_check()` and a panda that never calls it is a silent
   loss of the watchdog; a panda that calls it against an opendbc without it does not link.
3. Only then merge that `pnw-pilot` commit to a channel.

Note the current channel pins are **not** `master-pnw` on either companion (see
`[[deployed-pin-not-master-pnw]]`) — check what `3devpnw` actually pins before merging, and base
the bump on that, not on the local branch tips.

### Flash + verify (car disengaged; disengaged is the ONLY condition)

```bash
cd /data/openpilot/panda && scons -u -j4 && python board/flash.py
```

Then verify, in order:

0. **BEFORE flashing, on the RAVEN with the car on, identify the AUX panda's hw type**
   (`python -c "from panda import Panda; [print(s, Panda(s).get_type()) for s in Panda.list()]"`).
   If it is an H7 device the normal flash covers it. If it is an F4/deprecated device,
   `pandad.py` flashes it from the checked-in `selfdrive/pandad/fw/panda_f4.bin.signed`, which is
   **not** rebuilt from `panda/` — that panda keeps the 59-byte `health_t`, `get_state()` logs one
   size-mismatch line for it, and `connect()` deliberately does not refuse it (it is not
   `is_supported`). Confirm that log line appears and that the car still drives. This is the
   single most likely way this change could hurt the Raven, and it has NOT been tested on hardware.
1. `pandaState.pandaType` is `tres` (and both pandas are present on the Raven).
2. No `relayMalfunction`, no `controlsMismatch`.
3. The panda's reported `alternativeExperience` matches `CarParams.alternativeExperience`.
4. **New:** `pandaState.controlsAllowedLateral` tracks `pandaState.controlsAllowed` before
   `PandaMadsSafety` is set. If it is stuck false while `controlsAllowed` is true, the flashed
   firmware is not this build.
5. No `panda health packet size mismatch` in the pandad log **for the internal panda**
   (see step 0 for the aux panda).
6. `pandaState.madsDisengageReason` is 0.
7. **Only then** set `PandaMadsSafety=1` and reboot; the "Disengage on brake" toggle un-greys.

### Rollback

1. Re-pin **both** submodules to the SHAs the channel previously carried (`e18d40cf` opendbc /
   `56920ec6` panda as of this writing) in one commit, rebuild, reflash.
2. Set `PandaMadsSafety=0`.
3. Keep a known-good `panda.bin.signed` on the device **before** flashing — `pandad` reflashes to
   whatever `FW_PATH` holds, so a half-flashed panda recovers by putting the old signed binary
   back and rebooting.
4. The dual-panda flash blocker in `XNOR2BP.md` is why this is the owner's call.

### What is still UNVERIFIED until someone flashes

* ~~The STM32 firmware has **never been compiled**~~ — **CLOSED 2026-09-06.** Built on the DEVICE,
  which does have the toolchain at `/usr/local/venv/bin/arm-none-eabi-gcc` (a bare shell lacks that
  PATH — the `DEVICE-TOOLBOX.md` trap; `command -v` in a plain ssh session reports it missing and
  nearly produced a false "cannot be built" blocker). Standalone `panda/` build, scons exit 0,
  `board/obj/panda_h7.bin.signed` produced at 82,560 bytes (was 82,456). The compile line confirms
  the version claims independently:
    * `HEALTH_PACKET_VERSION` `0xF63F9DE2` -> `0xD18411DB` — changed, as designed.
    * `CAN_PACKET_VERSION_HASH` `0x75ABF276` -> `0x75ABF276` — **UNCHANGED**, so the matched-set
      hash guarding the Raven's dual-panda pairing is not disturbed by this flash.
  Note the full-tree `scons -u` does NOT work for this: it needs `rednose_repo` and the
  `agnos19-compat` overlay on `PYTHONPATH`. Build `panda/` standalone.
* The 0xf3 `param2` round trip (openpilot → USB/SPI → `heartbeat_engaged_mads`) has no test that
  crosses the wire; each half is tested separately.
* `controls_allowed_lateral_pkt` being filled correctly by real firmware.
* Everything about the dual-panda Raven pairing after a flash — **including the aux panda's hw
  type and whether it ends up with the 61-byte `health_t`** (flash-procedure step 0). This is the
  one finding from the Fable review that could not be closed in code, only mitigated.
* The claim "byte-for-byte unchanged with `PandaMadsSafety=0`" is **behaviourally** true (the
  control path is identical and the latch can never rise) but not *literally*: the health wire
  struct grew 59 → 61 bytes, `0xf3` now carries a `param2`, `pandad` opens an extra `madsState`
  subscriber, and `connect()` does one extra `0xd2` read.


## Test evidence

* Full panda safety suite (all cars, serial): **3067 passed**, 0 failed. Under xdist the Ford file
  is **flaky at the base commit too** (shared C statics in a process-wide singleton); run it
  serially (`-o addopts=""`) for a trustworthy result. A `tearDown` was added to stop the MADS
  tests contributing to that.
* MISRA/cppcheck: **43 violations, identical to the base commit** — zero new.
* `ruff`: identical to the base commit.
* openpilot side: 10 tests in `test_mads_alternative_experience.py`.
* **Mutation testing: 22 mutations, 0 survivors** (see the branch's commit message for the list),
  including two anti-weakening mutants — removing the brake clear of `controls_allowed`, and
  making the reset latch ignore authority entirely — both killed loudly.

## Test evidence — `madsheartbeat2pnw` (2026-09-05)

* **Panda safety suite, serial** (`-o addopts=""`): **3096 passed**, 0 failed. The one failure in
  the directory, `misra/test_mutation.py::test_misra_mutation`, is **identical at the parent
  commit** — it asserts the clean tree passes MISRA, and this tree has 43 pre-existing violations.
* **MISRA/cppcheck: 43 violations, byte-identical to the parent commit.** Zero new.
* **openpilot side:** `selfdrive/selfdrived/tests/` + `test_mads_alternative_experience.py` +
  `test_car_interfaces.py` — **357 passed**. `test_mads_pnw.py` alone is now **81 tests** (64 + 17
  new). `ruff check` clean (8 pre-existing opendbc errors, identical to the parent). (`test_alerts.py` does not collect on the dev host — missing acados
  `c_generated_code`; `test_cruise_speed.py` likewise.)
* **`pandad` C++:** `panda.cc` and `pandad.cc` type-check clean (`g++ -fsyntax-only -Wall -Wextra`)
  against the **regenerated** cereal headers and the real `panda/board/health.h`; no new warnings.
  A full link was not possible on this host (no `libusb-1.0` dev headers).
* **Reviewed by Gemini (`gemini-flash-latest`) and Fable in parallel; Fable had the last word.**
  Fable returned APPROVE WITH CHANGES; every finding is addressed below or in the flash procedure.
  The two that changed real behaviour: (1) `get_state()` originally returned `nullopt` on a short
  read, which would have **crash-looped `pandad` on the Raven** if its aux panda runs a deprecated
  F4 firmware — now flagged/logged and refused only in `connect()`, gated on `is_supported`;
  (2) the new event carried no displayable alert type, so in the `disabled` state it would have
  been a **log line with no text and no sound** — `ET.PERMANENT` added. Gemini independently
  raised the same `connect()` concern. Both also corrected two overstated claims (the SPI
  truncation story, and "byte-for-byte unchanged"), fixed above.
* **Mutation testing, 37 mutations, ZERO survivors:**
  * opendbc (13): threshold 3→4 and 3→2; counter never advances; counter never resets; latch guard
    dropped; heartbeat sense inverted; **watchdog GRANTS instead of revoking**; **watchdog also
    clears `controls_allowed`**; `heartbeat_engaged_mads` defaulting to `true`; watchdog reading
    `controls_allowed` instead of the lateral latch; wrong disengage reason; the counter not
    cleared on the latch rising edge; reason enum colliding with `LAG`.
  * openpilot (24): counter never increments / never resets; `self.enabled` guard dropped;
    `mads.available` guard dropped; `lateral_only` guard dropped; panda-permits test inverted;
    silent pandas no longer ignored; event never raised; threshold 200 → 20000; event downgraded
    from `IMMEDIATE_DISABLE` to a banner; `pandad` hardcoding the mads heartbeat true; `pandad`
    ignoring `madsState` staleness; `pandad` not publishing the panda's authority; `param2`
    hardcoded; a short health read silently accepted; `connect()` no longer refusing a
    mismatched panda; **`connect()` refusing ANY panda including the Raven aux panda**; the
    mismatch flag never set; **a short read killing `pandaStates` for every panda**; the disengage
    reason not published; **the mismatch alert no longer reaching the driver**; the alert being
    silent; the alert vanishing in one frame; a comms error misdiagnosed as a version mismatch.
  * One survivor was found and fixed rather than hidden: the C initializer
    `bool heartbeat_engaged_mads = false;` was masked because the libsafety harness forced it
    `true` in `init_tests()`. The harness no longer does that (a harness that lies about a
    safety default hides exactly this), and the initializer is pinned by
    `test_mads_heartbeat_default_is_revoke`.
