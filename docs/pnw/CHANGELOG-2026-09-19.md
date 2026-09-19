# CHANGELOG — 2026-09-19 (Saturday)

Continues [`CHANGELOG-2026-09-18.md`](CHANGELOG-2026-09-18.md). Four branches shipped, one
measurement returned a verdict of *dead*, and one owner decision closed a proposal.

**Channel tip:** `origin/3devpnw` = `915a030207`.

⚠️ **NOT VERIFIED ON THE CAR.** The device is alive (it phoned the uploader at 11:22:59 PT) but sits
behind a LAN this host cannot route to — `local_ip 192.168.1.79` under public `98.97.34.51`, neither
the home nor the hotspot subnet. So **everything below is pushed, and none of it is confirmed
installed.** The updater fetches on its own ~1.5 h cycle; the install lands at the next reboot.

---

## 🅿️ SHIPPED — `ces_events` stops logging a PARKED truck (`parkgate2pnw`, `0e4f446a7c`..`1dd8a1a313`)

**The finding.** `IsOnroad` follows ignition (Rule 3), so the Lightning parked and charging wrote a
~1 Hz `ces_events` breadcrumb indefinitely. Six archived generations pulled from S3 — 42,918 records
/ ~120 MB — are **93.7 % stationary**, 39,983 of them the identical `{"ev":"tick","reason":"stopLatch"}`
row. The archive was **6.3 % driving**.

**What it cost, and this is the part that matters.** `CES_ARCHIVE_MAX_BYTES` = 2 GB was justified as
"~95 days at 21 MB/day". Both halves true; the conclusion wrong. At 6.3 % driving content the budget
held about **12.9 DRIVING HOURS** — and curvedb Phase 2's go/no-go needs 6–8 weeks of corridor driving
*retained*. **The corpus the decision depends on was being deleted before the decision could be taken.**

**The gate.** `selfdrive/controls/lib/ces_pnw/park_tick_gate.py`, on **GEAR** — live
`carState.gearShifter`, compared as an enum (never `str()`; see [[capnp-enum-str-trap]]), never speed,
standstill or `IsOnroad`. A red light is `vEgo` 0 **in Drive** and keeps logging at full rate. Fails
open on every ambiguity. One explicitly-marked heartbeat per minute while suppressing, so a hold can
never be misread as a dead logger. `gear` and `park` added to every record — **there was no gear field
at all**, which is why this hid for months. Kill switch is the existing `RecordWhileParked`, scope
widened from route segments to segments + the breadcrumb.

**After: ~140–150 driving hours in the same 2 GB**, and parked content is now CAPPED however long the
truck sits. Independently re-measured against the S3 archive: driving density **6.3 % → 66.3 %**,
**10.5× retention**. Derivation is reproducible in [`PARKGATE2PNW.md`](PARKGATE2PNW.md).

**Fable found a real hole in the TESTS, not the code** — it constructed two mutants that survived,
both turning on the difference between "the gate was *evaluated*" and "a record was *written*". Round 2
pins that the gate runs **once per RECORD, not once per 100 Hz tick**. All four killed.

### The four stale "~95 days" claims are now corrected
`ces_pnw.py:89` (on the branch), and today `system/loggerd/uploader.py:58`,
`~/gh/comma/docs/CURVEDB2PNW.md` and `~/gh/comma/docs/DEVICE-STATE.md`. Each now carries the 12.9 h /
140–150 h pair instead of a calendar-day count that measured bytes rather than content.

## 🔭 SHIPPED — curvedb Phase 2's ON-CAR half, **SHADOW ONLY** (`curvedbshadow2pnw`, `b10a724124`..`fed2e52d6d`)

Yesterday's §12 said *"do not build Phase 2's on-car half."* The owner overruled that and asked for
§11's own recommended path instead — *"build this feature now and make it no-op and observe it over
the next few weeks… as opposed to not building it at all and theorizing about it."* He was right that
the design already said shadow-first; I had been framing it as binary.

On every record the shadow looks the site up, computes §6.4's authority and `cancel_target`, and writes
the answer into the tick (`cdbOn`/`cdbRows`/`cdbObs`/`cdbSite`/`cdbRow`/…). **No control path reads it.**

**The read boundary is enforced by a test, not by a comment** — `test_curvedb_read_boundary.py`: the
module is parsed with `ast` and may not import `cereal`/`opendbc`/`selfdrive`/`system`; a run with the
shadow raising on *every* call must be byte-identical to a run without it. That last one is the check
that would have caught a silent dependency.

**This answers §12's blocker from the car's own driving.** §12 measured site recurrence over `drives/`,
which is a pile of **analysis windows** (median span 62 min) — what we chose to pull, not what the truck
drives. `cdbRow`'s true-rate among `cdbSite` records is the unbiased version of that number.

1,113 tests, **36/36 mutants**. Fable's review caught that L5's empirical boundary test was **VACUOUS**
and that the site rate would have blown the latency budget; a follow-up commit found the corpus load
**does** trip `selfdrivedLagging` and bounded it (`OBS_MAX_BYTES` 1.5 MB → **512 KB**, with a loud
truncation log rather than a silent short read).

## 👁 SHIPPED — the model's curvature on EVERY tick (`viskvis2pnw`, `21f723d2c7`..`bc7ff5d978`)

`icbmKVis` was written only by `_curvelead_note` **inside `_icbm_step`'s lead-pacing block** — i.e. only
on ticks where ICBM already had a target. So the one witness that could say *"the map was wrong here"*
was recorded only where the map had already won. Across the whole corpus a correctly-computed vision
curvature coexists with an ICBM decision on **ONE drive, 56 ticks**.

`visKMax` (tightest predicted curvature) and `visKRch` (horizon reach) now ride every `ces_events`
record. **Telemetry only** — no control path reads either. +34 B/record (~1.3 %).

Fable's must-fix was comment-only and is the same defect class this week keeps producing: **a comment
that says something false about its own field.** Mine said both are *"null on a hiccup, never 0.0"* —
true of `visKMax`, **false of `visKRch`**, which logs **0.0** on a hiccup because `icbm_vision_curvature`
and its `except` both return `(None, 0.0)`. An analyst reading `visKRch=0.0` would have taken it for a
real horizon. The three states *are* distinguishable and the comment now says so: `(0.0, >0)` straight
road, `(null, 0.0)` hiccup, `(null, null)` a car that never computes it.

Its optional finding exposed a real gap: every live reading in the positive test was an exact 0.0
because the fixture road is straight, so **a field hard-wired to 0.0 would have passed**. Added a test
that drives a real curve and requires the reading to move, plus the mutant that pins it. 1,116 tests,
**5/5 mutants**.

> **Note on the commit message.** Backticks in the `-m` heredoc hit command substitution, so
> `bc7ff5d978`'s body is missing two inline identifiers ("and ␣ in the same record", "mutated the ␣
> gate"). The message is otherwise intact. **Not amended** — it is the channel tip the device tracks,
> and force-pushing a channel branch to restore two words is not a trade worth making.

## ⛔ MEASURED AND DEAD — the bounded vision give-back (`VISIONBACK2PNW.md`, pushed `915a030207`)

The owner asked: *"would vision make a good contribution if it was 100 m away regardless of the 13 %
inaccuracy?"* and then *"please measure it, I think Vision can make a valuable contribution here."*

Measured over 88 ICBM curve episodes above 25 mph, truth = `max(kPeak, |slKActl|)` at the apex:

| N (mph) given back | engaged | median recovery | **max resulting a_lat** | ≥ 4.5 | ≥ 5.0 |
|---|---|---|---|---|---|
| 0 (shipped) | 0 / 88 | — | 3.81 | 0 | 0 |
| **2** | 38 / 88 | +2.00 mph | **4.10** | 0 | 0 |
| **5** | 40 / 88 | +3.05 mph | **4.70** | 2 | 0 |
| **10** | 45 / 88 | +3.33 mph | **5.80** | 4 | **1** |
| 15 | 46 / 88 | +4.69 mph | 7.01 | 8 | 3 |

On the owner's own rule — comfortably under the Lightning's measured ~4.5 m/s² hands-off ceiling —
**N = 10 is dead at 5.80 on one real bend, and N = 5 reaches 4.70 with no margin.** Only **N = 2** stays
under, and 2 mph is two taps of the executor's 1 mph step, inside its own set-report latency.

**And the reason is not accuracy.** The trust gate turns out to be a pass-through: vision says "no real
curve" on **87 %** of ticks. That is the same wall the unwired `icbm_map_sanity` veto hit. 100 % under
`docs/` — nine analysis scripts with their outputs committed alongside, so the table is reproducible.

## 🐢 CORRECTION to yesterday's entry — `icbmslow2pnw` **did** ship, 09-18 19:27

[`CHANGELOG-2026-09-18.md`](CHANGELOG-2026-09-18.md) says the map-rating floor is *"in Fable re-review,
NOT pushed"* and gives the tip as `a46b90f2b5`. Both were true when written and stale within the hour:
the floor landed as `a9329c6d75`, the regenerated washout registry as `ef558400c6`. That changelog's
header has been corrected in place. **`icbm_map_floor_frac` is on the car's channel at its default 1.0
— the floor is ON at full strength.**

## 🔴 `curvedb2pnw` re-review — §6.2's DOWN rule could never fire (`8f2247b0bc`, `d3b7242a61`)

Yesterday reported *"§6.2's DOWN rule produced ZERO observations on 2.2 GB"* and explained it as the
interventions being below the trigger. **That was the reverse of the truth.** The rule was
**structurally unreachable**: the passage was discarded as `drv`-dirty *before* the DOWN loop ran, and
a DOWN **is** a `strPrs` tick. The rule that exists to record the driver's own interventions had never
recorded one, anywhere. Fixed: **127 observations**.

It did **not** move the headline — the replay still acts on **zero of 170 episodes** under
leave-one-date-out — but a zero that is a bug is not a finding, and it had been written up as one.
345 tests, **96/96 mutants**. Eight further findings applied, all under `tools/`.

## 🛰 Car GPS for mapd — `MapdUseCarGps` enabled on the device

Set to `1` at the owner's instruction (*"ok do this"*). mapd now navigates on the **truck's** GNSS
rather than the comma's, via the existing `mapdIn` relay; it is read inside `mapd_configd`'s loop
(line 723), so **no reboot was needed** — I said one was and corrected that in the same turn.

⚠️ **This is a device-local param write, not a repo change** — `params_keys.h` still defaults it to
`"0"`. It survives reboots and updates but not a factory reset, and nothing in the tree records that it
is on. Tracked in `PENDING-WORK.md`: decide whether the default should follow.

## 🚫 CLOSED BY THE OWNER — no curve gate on nudgeless lane change

After the 09-17 18:31 PT self-initiated lane change in a curve, I proposed gating nudgeless ALC on
curvature. Owner: **"no chnage needed"**, and again **"no chnage please"**. Recorded as an explicit
**DO-NOT-BUILD** in `PENDING-WORK.md` and in the drive report so it is not re-proposed.

---

## 🔁 The pattern worth naming: five checks this week that could not fail

Not a feature, but it is the most reusable thing to come out of these two days. Every one of these
passed, and every one was empty:

1. The **sign test** that pinned `kPose` — it agreed with itself because both sides came from the same
   negation.
2. The **site-shuffle control** in the curvedb replay — invariant under leave-one-date-out, so it could
   not have detected a leak.
3. **Mutant M4a**, killed by a `TypeError` before reaching the assertion it was meant to test.
4. `parkgate2pnw`'s **mutation harness**, which structurally could not report a survivor.
5. The shadow's **L5 empirical boundary test**, vacuous — it proved nothing about the boundary.

Four of the five were caught by Fable, not by me, and all five are the same shape: *the success path
and the failure path are indistinguishable* (CLAUDE.md Rule 2). The countermeasure that actually works
is the one now applied to every mutant in this repo — **`compile()`-check it AND anchor-check it for
exactly one match before it is allowed to count.**
