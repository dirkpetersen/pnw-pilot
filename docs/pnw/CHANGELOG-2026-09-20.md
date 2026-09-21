# CHANGELOG — 2026-09-20 (Sunday)

Continues [`CHANGELOG-2026-09-19.md`](CHANGELOG-2026-09-19.md).

**Channel tip:** `origin/3devpnw` = `f3f19d2cb1`, **verified GREEN twice in a row: 3,473 passed, 0 failed.**

---

## ✅ Rule 9's first run on someone else's work — and it found a blind spot in itself

`everdrive2pnw` landed 7 commits overnight touching `common/params_keys.h`, the **`opendbc_repo`
submodule pin**, and UI code. That is exactly a [Rule 9](../../../CLAUDE.md) trigger, and it had not
been checked. It came back **GREEN (3,320 passed, 73 s)**, and both mechanisms built yesterday proved
themselves on work that was not mine:

* **`opendbc_repo` auto-moved to the bumped pin `a68eea339b`** — fetched and checked out the new
  revision rather than testing a stale one. Under the previous borrow-from-the-main-checkout setup
  this would have silently tested whatever that checkout happened to be on.
* **`params_pyx.so` validated against 219 keys** (was 217 — `everdrive2pnw` added two). Because the
  script BUILDS it from the tip's own header, all 219 are known. A borrowed `.so` would have called
  those two keys *unknown*, and `UnknownKeyName` raised deep in feature code would have surfaced as
  unrelated tests failing.

### ⚠️ But it reported GREEN without importing the feature's own tests

`everdrive2pnw` shipped **`selfdrive/ui/onroad/tests/test_everdrive_status_pnw.py`** into a directory
`TEST_PATHS` did not cover. The run was green and the feature's 29 tests were never collected.

**A scope that never GROWS is as blind as one that silently shrinks** — and only one of those two was
guarded against. Added `selfdrive/ui/onroad/tests`, `selfdrive/ui/tests` and `system/athena/tests`
(**+146 tests, 3,320 → 3,466**), with the rule now written into Rule 9: *if a feature lands tests in a
new directory, add the directory.*

### Out-of-scope reasons made concrete rather than vague
Replaced the hand-wave with named causes: `selfdrive/car/tests` (every upstream car model, minutes,
we do not modify it); **`system/manager/test`** (2 of 25 need `rednose.helpers.ekf_sym_pyx`, which
comes from the full locationd/rednose build — including it would mean a complete openpilot build on
every push); and the device-hardware dirs (`locationd`, `pandad`, `camerad`, `sensord`,
`hardware/tici`, `qcomgpsd`).

## 🧪 The gate itself was FLAKY — four test bugs, and my own diagnosis was the fifth error

The gate returned **1 failed, then 11 failed, then green seven times on IDENTICAL code at an
identical tip.** A gate that is right half the time is worse than no gate: the next real regression
gets waved off as *"that flaky thing again"*.

### ⚠️ My root cause was wrong, and I wrote it into the shared script
I blamed two pytest runs sharing `~/.comma/params/d` and added a `PARAMS_ROOT` export. **`Params()`
resolves to `$PARAMS_ROOT/<OPENPILOT_PREFIX>`, and `conftest.py:48`'s autouse
`openpilot_function_fixture` already wraps EVERY test in `OpenpilotPrefix` with a random prefix** —
measured, two tests in one process report different params paths. My export closed **nothing**, which
is exactly why the failure could not be reproduced with it. The comment in `check-channel-tip.sh` now
states this rather than leaving a false cause for the next reader; the export stays only because it
covers a `Params()` built at *import* time, before the fixture runs.

### The four real causes, all demonstrated, all in TEST code

| # | bug | before → after |
|---|---|---|
| 1 | **`test_alertmanager.py` wrote `.duration` on the SHARED `Alert` singletons in `EVENTS`.** `create_alerts()` returns `EVENTS[e][et]` *itself*, so one run clobbered **~61 of 123** alerts with a random 1–99 for the rest of that xdist worker. **This is the `assert 37 >= 100` in `test_mads_pnw`.** | pair fails under a pinned seed → green |
| 2 | `test_logger.cc` `rm -rf`'d a hardcoded `/tmp/test_logger`; concurrent runs deleted each other's segments mid-write and the binary aborted | **5 fails / 6 → 0 / 16** concurrent |
| 3 | `test_athenad.py` bound hardcoded TCP port **45454**, no `SO_REUSEADDR` | **2 fails / 10 → 0 / 24** concurrent |
| 4 | `TestSkipWideFullLoop`'s flat `sleep(1.5)` — margin collapses **4.3× → 1.35×** under the exact "gate + review agent" load, failing as *"ecamera did not upload"*, indistinguishable from a real uploader regression | waits on its own expectation now |

**#4 was NOT loosened.** It waits on the test's own expectation (the idiom `wait_for` already uses in
that file); every assertion is byte-identical and the negative test spins 0.5 s *longer*.
Mutation-proved: disable the skip, drop the rlog priority tier, or break pass 2 — the right test
fails each time. **Honest caveat:** the exact ecamera failure was never reproduced in ~65 attempts;
what exists is the margin measurement, not a demonstration.

### The best part of the fix is a recurrence guard
`test_alertmanager.py` now carries an autouse fixture that fails **in that file** if the table is
mutated again: *"65 shared Alert(s) in EVENTS were mutated by this test"*. Verified by removing the
`copy.copy` and watching it fire. **The bug was silent by construction** — the damage always landed
on an innocent test in a different module, which is why it read as flakiness rather than a defect.
Same shape and same remedy as `_drop_synthetic_event` from 09-19.

### And the deeper cause of the windows-on-the-desktop problem
`system/ui/lib/application.py:856` builds `gui_app = GuiApplication()` at **module scope**, and its
`__init__` calls `rl.init_window(1, 1, "")`. So **merely importing a UI test module opens a real OS
window** — 197 tests across four modules do. `application.py:204` gates it on
`if PC and os.getenv("SCALE") is None`, so the gate now exports **`SCALE=1`** (nothing opens a window
to be imported) **and** wraps pytest in `xvfb-run` (for `test_raylib_ui`, which spawns the real `ui`
process and genuinely needs a display). Flagged but NOT changed: constructing a window at import time
is product code that affects UI startup.

**Validation:** 10 sequential runs green, then **20 as two CONCURRENT loops — 20/20**, where that same
setup was 3 red of 20 before. Then two more full gate runs at `f3f19d2cb1`: **3,473 passed, both.**

## 🟠 ICBM phantom re-measure: BLOCKED. Posted-limit clamp: DESIGNED, not built.

Owner asked for both (and chose **option (b)**, the posted-limit sanity clamp, over raising
`ICBM_FLOOR_MAX_LIMIT`). Neither could be finished, and the reasons are worth keeping.

### The phantom cannot be re-measured yet — three verified blockers
1. The offline corpus ends **2026-09-17**; the map-rating floor shipped **09-18 19:27**.
2. ~~**`mapRaw` / `mapEff` are DEAD TELEMETRY**~~ — ❌ **RETRACTED SAME DAY. I was wrong, twice, and
   this was pushed before it was checked.**

   **(a) The field is not dead.** `vtscState = 'idle'` on **all 58,614** device records — only 1,781
   ticks moving >5 m/s, `reason='stopLatch'` on 56,304. **VTSC never triggered**, and `cap()` resets
   `_tele_map_raw = 0.0` at the top of every tick, so `0.0` is the CORRECT idle value. The offline
   corpus cannot contradict it: it carries no `vtscState` at all. **This is the uniform-zero trap in
   Rule 2** — quoted repeatedly in this very changelog, then walked into. My own filter made it
   worse by treating `''`/`'none'`/`-1.0` as "not alive" when those ARE the idle values.

   **(b) `mapRaw` is not the field the floor uses anyway.** `icbm_map_floor_ms()` is fed
   `sig["map_target_v"]` (`ces_pnw.py:4796`) and the result is logged as **`icbmMapFlr`, which IS
   alive** (103 non-zero). It never blocked anything.
3. The device retains only **106 ICBM ticks**, every one `secondary`/`tertiary` at 25 mph posted.
   No post-floor highway ICBM data exists.

> ✅ **Checked before reporting:** `icbmMapFlrHit` reads true on **102 of 102** ticks, which looks like
> an always-true flag. It is not — `target > penalised` means "the floor gave penalty back", and at
> `icbm_map_floor_frac = 1.0` it nearly always does. Read the source rather than filing a bug.

### The clamp's naive form is DEAD, and the measurement says so

| 2026-09-08 event | gap below posted | peak achLat |
|---|---|---|
| **20:28:51 PHANTOM** (69 → 43 on a 60 road) | **15.8 mph** | **−0.70 m/s²** (straight) |
| 19:44:07 REAL CURVE | **25.9 mph** | **−3.13 m/s²** (genuine) |

**The real curve has the BIGGER gap.** A "more than N mph below posted" rule fires *harder* on the
genuine slowdown than on the phantom, and corpus-wide the gap's p90 is 14.2 / p95 16.5 mph — the
phantom is not even an outlier. So `when there is no curve` is not a refinement of option (b), **it is
the entire rule**, and N is meaningless without it.

**Every "is there a curve?" witness is currently unusable:** `mapRaw` dead; `visK` dead in corpus
(p50, p90 **and** max all exactly 0.00000 over 819 ticks); `achLat` lags, so using it would cancel
real anticipatory braking — the dangerous direction; `hwyClass` alive but `motorway` ≠ "no curve".

**Building it now would put a rule into the CONTROL PATH that cannot tell the phantom from a real
curve on the only data we have.** Stopped instead.

### What unblocks both — already half-done
**`visKMax` went live yesterday** (`viskvis2pnw`, verified on the truck) and records on EVERY tick
rather than only where ICBM already acted. **One highway drive with the floor live** yields the
phantom's post-floor recurrence, `visKMax` on a real curve vs a phantom, and therefore both
thresholds. **Nothing needs to be coded first** — the "fix `mapRaw` first" advice in the retracted
item above was based on the same two errors.

## 📌 Not pushed, deliberately
Seven worktrees carry unpushed commits that are **not** part of this effort — `redlight-stop2pnw`,
`mapdstate2pnw`, `lcabort2pnw`, `mapdlog2pnw`, `policemiss2pnw`, `speedadjustreset2pnw`, `uicpu2pnw`.
`PENDING-WORK.md` flags this class as landmines: **`policemiss2pnw` predates `policeship2pnw` and
would delete its TTL re-check**, and `lcabort2pnw` is held by an explicit owner decision. Shipping
any of them needs a rebase onto the current tip, a check of what it reverts, and a review first.
