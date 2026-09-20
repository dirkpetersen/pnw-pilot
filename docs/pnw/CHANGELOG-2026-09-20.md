# CHANGELOG — 2026-09-20 (Sunday)

Continues [`CHANGELOG-2026-09-19.md`](CHANGELOG-2026-09-19.md).

**Channel tip:** `origin/3devpnw` = `a162607014`, **verified GREEN by `check-channel-tip.sh`: 3,466
passed, 0 failed, 15 paths all clearing their collection floors, 154 s.**

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

## 📌 Not pushed, deliberately
Seven worktrees carry unpushed commits that are **not** part of this effort — `redlight-stop2pnw`,
`mapdstate2pnw`, `lcabort2pnw`, `mapdlog2pnw`, `policemiss2pnw`, `speedadjustreset2pnw`, `uicpu2pnw`.
`PENDING-WORK.md` flags this class as landmines: **`policemiss2pnw` predates `policeship2pnw` and
would delete its TTL re-check**, and `lcabort2pnw` is held by an explicit owner decision. Shipping
any of them needs a rebase onto the current tip, a check of what it reverts, and a review first.
