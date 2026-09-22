# CHANGELOG — 2026-09-21 (Monday)

Continues [`CHANGELOG-2026-09-20.md`](CHANGELOG-2026-09-20.md). All times PT.

**Channel tip:** `origin/3devpnw` = `674e0df124`, **GREEN: 3,655 passed, 0 failed** (Rule 9 gate, with the
scope widened today, see §3).
**Installed on the truck:** still `93cd7c4` (09-20 17:21). ⚠️ **`674e0df124` is PUSHED, NOT INSTALLED, NOT
VERIFIED ON THE DEVICE.** See §1, "Still owed".

---

## 1. ✅ SHIPPED `capnpfork2pnw` — the fork's events and MADS fields move out of upstream's `log.capnp`

Commits `f71061a598` (code), `6e0f49c4a7` (design doc [`CAPNP-FORK-ORDINALS.md`](CAPNP-FORK-ORDINALS.md)),
`674e0df124` (review notes). This is the wire-format blocker that `docs/CHESTNUT-PORT.md` §1 puts ahead of
any 0.11.2 port. It is **worse than §1 said**:

* our upstream base is commaai master of 2026-03-14, so `@99 greenLight` already collides with **0.11.1 itself**
  (`lateralManeuver`);
* commaai master has since taken `@104` too, so **all six fork `EventName` ordinals collide**.

**What changed on the wire:**

| was (upstream's `log.capnp`) | now |
|---|---|
| `EventName @99..@104` (greenLight, leadDeparting, madsLateralOnly, **madsControlsMismatchLateral**, cruiseOffRequested, madsResumeSetTooHigh) | `custom.capnp` `OnroadEventsPnw`, in Event slot **`@137`** (upstream's never-used `CustomReserved11`) |
| `PandaState @39 madsDisengageReason :UInt8` | `custom.capnp` `PandaStatesPnw`, in Event slot **`@138`**, published by `pandad` |
| `PandaState @40 healthPacketMismatch :Bool` | `@39 :Bool`, **on the same data bit (482)** as the old `@40`, so old logs read correctly with no migration. Verified with pycapnp on both schemas: same bit, same `dataWordCount` |

In Python the fork events stay in the **same** `Events` object, under a separate key range from 65536.
`madsControlsMismatchLateral` still forces a disengage, `cruiseOffRequested` still blocks engagement, and
alert priority and tie-breaking are unchanged.

**Old logs** are migrated in `migrate_all` by reading the **writer's own `log.capnp` from git at
`initData.gitCommit`**, never by guessing from content. It raises `PnwLogSchemaError` whenever it can't be
certain, so a 0.11.2 `carNotReady @103` is never turned into `cruiseOffRequested`. xnor-era writer commits are
not in this repo, so such logs raise; that is intended.

**Evidence:**
* The migration was run over 7,561 real logs: 0 errors, 0 contested ordinals left.
* Mutation testing: 35 mutants, 0 survived.
* The guard test compares the whole compiled schema against a frozen snapshot of the upstream base.
* **Fable: APPROVE** (all three commits), with no blocking findings. Fable independently confirmed: the safety
  events are unchanged; fork keys can't leak into `onroadEvents` (pycapnp rejects them); services are
  registered in `services.h`; `pandad.cc` compiles under the exact scons flags; the bit-482 claim holds; and
  the `@137`/`@138` slots are unused on commaai master, `release-chestnut` and `release-tizi`.
* **`pandad` LINKS on the host.** This was the one thing unverified at review. `libusb-1.0-0-dev` was unpacked
  without root (`apt-get download`) and placed under `third_party/` only for the build. `scons` RC=0, the
  binary carries `pandaStatesPnw`, and startup runs to "no pandas found, exiting".

**Still owed (Fable's condition, and the doc's §8):** the offline import test in the device venv against the
**staged** tree **before a reboot**. Then after the boot: `pandad` linked, manager PIDs stable, and both
`onroadEventsPnw` and `pandaStatesPnw` published. **It hasn't been done**: at push time (~22:37) the device
was on an unfamiliar WiFi (`192.168.1.79`, last check-in 22:34) that this host can't reach. If the truck
power-cycles first, the next boot installs `674e0df124` unchecked. Rollback is `git revert` of the three
commits. But a failed boot build means `manager` doesn't start, so the updater can't self-heal; it needs SSH.

**Known follow-ups, none blocking:** the `test_processes` ref logs will diff on the new `selfdrived`
subscription until regenerated (not in the gate). Ad-hoc scripts under `drives/` that look for fork event
names in `onroadEvents` must go through `migrate_all` or read `onroadEventsPnw`.

## 2. ✅ SHIPPED `oplongtest2pnw` (`27ce8b5826`) — two tests red since 09-05, unseen. TEST-ONLY.

`TestOpLongResetFailureRetry` simulated a failed op-long reset by replacing `card.PnwVehicle` module-wide.
`mads2pnw` (`86b868a5a5`, 09-05) added an **unguarded** `PnwVehicle(self.CP)` call in
`Car._alternative_experience()`, so the injected raise escaped `Car.__init__` there. The raise is now confined
to `op_long_native`. `card.py` is unchanged. 11/11 pass, and two mutants each turn 2 tests red. **Fable:
APPROVE.**

⚠️ The first version of this fix **silently removed the tests from collection**: a module-level helper inserted
inside the class ended the class body. A mutant that reported *"no tests ran"* caught it.

🔓 **Open, safety path:** that unguarded call means a `PnwVehicle` failure would crash `card` at init, while
every other `PnwVehicle` call in `__init__` degrades with `cloudlog.exception`. Crash (loud; openpilot down)
or fall back to the stock panda contract with a logged error? **Not decided.** It's in `docs/PENDING-WORK.md`.

## 3. 🧪 The Rule 9 gate did not run seven of OUR test files

`scripts/check-channel-tip.sh` (workdir `cb0e1ec`, `036c550`) excluded all of `selfdrive/car/tests` as "we do
not modify it". That was false for seven files: accdrop, calswap, fpcache, gear_park, mads-altexp, oplong and
pscmlim. They are now listed one by one (107 tests). The floor counter now accepts file entries; the old regex
counted a file as 0 forever. The `selfdrived/tests` floor went 155 → 210 after §1 added 60 tests.
Result: `614dfd50fd` 2 failed → `27ce8b5826` 3,595 passed → `674e0df124` 3,655 passed.

**Observed once, not investigated:** the `ui` child process of upstream `test_raylib_ui` segfaulted at teardown
(mici onboarding texture load, stopped mid-init under 8-worker load). The test passed because liveness is
checked before `stop()`. It didn't recur in the next two runs.

## 4. 🔴 Drive analysis: ICBM rejected a real curve (no code change)

I-5 SB near Tumwater, 21:20–21:22.
[`drives/2026-09-21/i5-south-olympia-curves/DRIVE_REPORT.md`](../../../drives/2026-09-21/i5-south-olympia-curves/DRIVE_REPORT.md).

* **Left curve:** mapd rated it 59.9, which was about right. The sweeper-end composite ×1.3475 × 0.92 made it
  74.3, inside the 2.2 mph dead band against set 74.9, so it was **rejected**. The S-bend's restore then took
  the truck **64 → 73 mph into the curve**. openpilot saturated before the driver steered in.
* **Right curve:** mapd rated it 49.7, and ICBM was exactly at mapd's number. The driver pressed the
  accelerator to 70.

This is the first on-road instance of 09-18 open item 3. **Owner question:** does the 09-18 "mapd's rating is
the ceiling" decision cover the sweeper end?

## 5. Also found today

* **`car.capnp` `SafetyModel` collides too:** ours `mg @35` / `teslaLegacy @36` (panda `35U`/`36U`), upstream
  `byd @35` / `volvo @36` / `bmw @37` / `mg @38` (panda `SAFETY_MG 38U`). That number is the **panda
  safety-mode ID**. It is inert while opendbc and panda stay pinned together, and must be settled before any
  opendbc rebase. Recorded in `docs/CHESTNUT-PORT.md` §1.
* **The truck runs CD210, not Lebowski** (`docs/DRIVING-MODELS.md`): Lebowski is 0.11.2's big eGPU model and
  was never built for the 3X. commaai model LFS moved to Hugging Face on 09-10, while our `.lfsconfig` still
  points at GitLab.

## What was NOT done

* No on-device verification of `674e0df124` (device unreachable; see §1).
* No ICBM change. §4 needs the owner's answer, then a design, a corpus replay and a Fable review.
* No decision on the unguarded MADS `PnwVehicle` call (§2).
* The police miss (~21:5x): evidence saved under `drives/2026-09-21/i5-south-police-miss/`. **There's no report
  yet**, because the driver hasn't said where the Waze report was.
* The local `3devpnw` ref was not fast-forwarded: it is checked out in `pnw/wt-stophold2pnw`, which holds
  someone's **staged, uncommitted** changes (incl. `cereal/log.capnp`) from 09-19. Left untouched; see Rule 10.
