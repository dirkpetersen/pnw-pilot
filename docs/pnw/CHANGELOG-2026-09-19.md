# CHANGELOG — 2026-09-19 (Saturday)

Continues [`CHANGELOG-2026-09-18.md`](CHANGELOG-2026-09-18.md). Four branches shipped, one
measurement returned a verdict of *dead*, and one owner decision closed a proposal.

**Channel tip:** `origin/3devpnw` = `586446b6f3`.

⚠️ **NOT VERIFIED ON THE CAR — and it is the documented Starlink case, not a mystery network.** The
device is alive (it phoned the uploader at 11:22:59 PT) from `local_ip 192.168.1.79` under public
`98.97.34.51`. That is the **same LAN address as the 09-17 Starlink session**; only the public IP
rotated within the ISP's block (`98.97.43.186` → `98.97.34.51`). **Starlink is CGNAT: there is no
inbound route from anywhere**, so this is SSH being *impossible*, not blocked — no `SIGHUP` to force a
fetch, no reboot, no post-install health check, no param read.

A stable `local_ip` with a drifting `src_ip` is the signature of this link. **Do not read the changed
public IP as a different network, and do not read "no answer on port 22" as "the device is down"** —
it uploaded 50 seconds before I looked. Everything below is **pushed and unconfirmed**; the updater
fetches outbound on its own ~1.5 h cycle and installs at the next reboot, and S3 is the only install
evidence available until the truck joins a routable network.

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

## 🧪 SHIPPED — the uploader's tests had been RED for two weeks (`uploadtest2pnw`, `1f9a352538`)

Found while integration-testing the channel tip after four branches landed in one day — not by looking
for it. **4 of 39 tests in `system/loggerd/tests/test_uploader.py` were failing on the channel.**

**Bisected:** 1 red at `4bb6a2e23b` (08-14), still 1 at `702487b133` (08-27), **4 at `6ad65ca264`**
(`uploadanywifi2pnw`, 09-05), still 4 at `718079e75a` (09-10 — which touched this very file).
`uploadanywifi2pnw` made pass 2 run on any qualifying WiFi, so the tests now move **4** files where
stock moves **2**, while `gen_order` still returned stock's expectation. **The code was right and the
test was stale.**

**Why that is worse here than in most files.** This is the component whose API_HOST fallback once
marked files uploaded *without them ever reaching S3* — silent data loss, idle uploader, clean-looking
device. `test_upload_ignored` exists to pin that it cannot come back. **`test_upload_ignored` was one
of the four red.**

The restored suite asserts the fork's contract: the real key set, no duplicates, creation order, and
that **`dcamera.hevc` never leaves the device** — the driver-facing camera, which `list_upload_files`
yields and only one tier keeps unpicked. Also fixed a latent helper bug: `.with_suffix("")` maps
`qlog.zst` → `qlog` correctly and `fcamera.hevc` → `fcamera` incorrectly; stock never hit it because
stock only ever checked qlog keys.

### Fable caught that I had made failing tests pass by WEAKENING them

I dropped stock's exact-sequence assertion, claiming the pass interleave made a global sequence *"a
race"*. **It is not a race.** `main()` is single-threaded, the tests disable every sleep, and the order
is a pure function of `PASS2_INTERLEAVE`. Fable re-measured the 24-key sequence **five times and got
byte-identical output**, then derived it from the constant.

Two real regressions went through the hole that claim opened, both now proven by mutation:

| mutant | my set-based check | the restored sequence check |
|---|---|---|
| boot tier swapped below qlog (boot stops going first) | **survives** | **KILLED** |
| `pass1_run >= PASS2_INTERLEAVE` gate deleted (video starves) | **survives** | **KILLED** |

Neither moves a file in or out of the set, so a set-plus-per-kind check structurally cannot see either
— **and stock would have caught the first one.** `gen_sequence` now rebuilds the exact order *from*
`PASS2_INTERLEAVE` rather than pasting a captured sequence, so a constant bump updates the expectation
while a genuine reordering still fails.

**8 mutants, 8 KILLED, 0 survived, 0 not built**, source restored byte-identical; 39 passed (was
4 failed / 35 passed), 1,849 across the full suite. The harness reports **the assertion that killed
each mutant**, because `-q` truncates messages to `Ass...` and a mutant killed by the wrong assertion
would otherwise look identical to one killed by the right one. Reproducible at
[`uploadtest-evidence/`](uploadtest-evidence/README.md).

> **One mutant SURVIVED and is recorded rather than quietly dropped.** M7 as first written
> (`PASS2_INTERLEAVE` 4 → 1) survived — but it is a **mis-specified mutant, not a coverage hole**: it
> mutates the very constant the expectation derives from, so contract and expectation move together,
> which is the documented intended property. Rewritten as the ordering change it was meant to be, it dies.

### The general gap this exposes
**Nothing in this workbench runs the test suite against the channel tip after a merge.** Every branch
is tested on its own base. That is how four branches shipped in one day today, and how a file could
stay red for two weeks while a commit edited it. A CI-equivalent — run the suite on `origin/3devpnw`
after each push — would have caught this on 2026-09-05. **Not built; recorded as a proposal in
`PENDING-WORK.md`.**

## ✅ VERIFIED ON THE CAR — the map-rating floor IS live, established without SSH

The truck is on **CGNAT Starlink**, so there is no inbound route. But a new route
(`000001b3--2bb58f4d0d`) began uploading at 11:18 PT, and **every route's qlog carries
`initData.gitCommit`** — and qlogs upload. Read straight out of S3:

```
gitCommit  d0a6b08abce254fa1ff5118b852c6f2e0e5ea52c
gitBranch  3devpnw     dirty False     version 0.11.1
```

`a9329c6d75` (`icbmslow2pnw`) is an **ancestor** of that, so **`icbm_map_floor_frac` is on the truck at
its default 1.0 — the floor is ON at full strength, on today's drives.** The ICBM over-slow reported on
09-17 should already be softened: +1.31 mph on targets, median lateral accel 1.41 → 1.65 m/s², zero
slowdowns lost.

**The car is 22 commits behind**, so none of today's four ships are on it yet — the ordinary reason,
now distinguishable from an unknowable one. Recipe recorded in `DEVICE-STATE.md`; evidence kept in
`drives/2026-09-19/parkgate-baseline/`.

## 🚨 NEW REQUIRED STEP — test the CHANNEL TIP after every push (CLAUDE.md **Rule 9**)

Owner directive: *"build this and add it to CLAUDE.md as a required step."* Built as
`scripts/check-channel-tip.sh` + `scripts/_check_params_so.py`.

**Its first run on the real tip returned `319 failed / 2,578 passed / 232 errors`.** All of it had
been true for weeks. Nothing in this workbench had ever run the fork's test directories *together*.

### One line was corrupting 319 tests, with two unrelated-looking symptoms
`common/tests/test_connect_backend.py` did a bare
`sys.modules["openpilot.common.swaglog"] = _swaglog_stub` and never undid it. `sys.modules` is
per-**process**, so every test collected after it got the stub.

| | |
|---|---|
| tip as-is | **319 failed, 2,578 passed, 232 errors** |
| that one file fixed | **1 failed, 3,286 passed** |

The second symptom looked like a different bug entirely: `common/tests/test_swaglog_rotation.py` —
**26 tests for our own swaglog-rotation feature** — erroring with `cannot import name
'SwaglogRotatingFileHandler' … (unknown location)`, never run. Same stub; it has no such attribute. I
detoured onto `--import-mode=importlib`, which "fixed" that symptom by changing collection order and
left the cause in place.

### A driver-visible alert had been overflowing the screen since 2026-07-14
`canBusMissing`'s PERMANENT text measured **1928 px against an 1860 px limit**. `canoff2pnw`
introduced it; `test_alert_text_length` exists to catch exactly this and lives in a directory nothing
ran. Shortened to *"CAN Bus Disconnected — Vehicle Off or Wiring"* = 1681 px. Fable swept the file:
this is now the widest alert and the next is upstream's at 1568, so nothing else is near the limit.

### Two of my three exclusion reasons were FALSE, and both hid tests for OUR code
This is the part worth carrying forward.

| I wrote | actually |
|---|---|
| `test_following_distance` — "upstream harness drift" | The traceback says that; the diagnosis was wrong. The plant harness is unmodified upstream — what reads `.alive`/`.valid` on its plain dict is **our** code (`tightfollow2pnw` `sm.alive['mapdOut']`, `leadlossgate2pnw` `sm.valid['carState']`). **The 18 tests validating our follow distance had been dead since those features landed.** |
| `test_leads` — "Hyundai flag, inert for our cars" | True of car behaviour, false of infrastructure. The stale `CANFD_LKA_STEERING` is in `process_replay/migration.py`, which every `replay_process_with_name` consumer in the fork depends on. |
| `test_loggerd.py` — "needs camera/encoder hardware" | Wrong too: the binaries were simply **not built**. Building `system/loggerd` recovers 13 of 16, **including all five `test_skip_video_when_parked*`** — the Rule-3 "don't record in Park" feature. Only 3 genuinely cannot run here, now deselected **by name** with measured reasons. |

`selfdrive/controls/tests` went **32 passed (two files excluded) → 51 passed, nothing excluded.**

> **An exclusion is a claim about the world. Check it before writing it.** Now in Rule 9.

### My script committed the exact sin it exists to prevent
On the red tip it printed *"only 2578 tests passed … Either collection broke or a whole path stopped
being discovered"* — **a real failure reported as a tooling problem.** The exit-code check came after
the count check. Fable caught it; the order is swapped and the RED path now names the failure. It
also found **`except Exception: pass`** in `_check_params_so.py` — the idiom Rule 2 bans by name, in
the file whose docstring cites Rule 2.

`MIN_TESTS=3000` against 3,287 measured was likewise nearly a check that could not fail: only the
four largest paths could ever trip it, so silently losing `selfdrive/selfdrived/tests` (which caught
the alert) or `system/loggerd/tests` (the uploader tests that motivated all this) still passed.
Replaced with **per-path collection floors**.

### And no `params_pyx.so` in the workbench was current
Every one was missing at least one key `params_keys.h` declares. A stale one manufactured **31
phantom failures/errors in `ces_pnw` alone** and pointed at an unrelated file. The script now BUILDS
it from the tip's own header — submodules symlinked from the main checkout, **each verified against
the channel pin**, with an opendbc mismatch fatal since it is on `PYTHONPATH` and is car code — then
VALIDATES it before trusting any result.

**Scope:** 12 paths, ~3,300 tests, ~90 s once built. 3 tests deselected by name, 0 paths excluded.

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
