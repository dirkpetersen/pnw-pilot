# CHANGELOG — 2026-09-16 (Wednesday)

Continues [`CHANGELOG-2026-09-15.md`](CHANGELOG-2026-09-15.md), which ran past midnight — the nine installs it
records finished at 02:03 PT today. This file covers everything after that.

**Channel tip:** `origin/3devpnw` = `18a023cf74` · **on the truck:** `23f0f47d59`, `BootCount` 249. The one
commit between them is **docs-only**, so the truck is level on code. (⚠️ the *local* `3devpnw` ref is still at
`23f0f47d59` — fast-forward it before branching anything, or you will build on a base that is missing commits;
see [[feedback-keep-local-remote-in-sync]].)

---

## ⚠️ The old local `*2pnw` branches are landmines — three would have caused regressions

Surveyed on the owner's instruction to "ship everything that is ready". **Nothing was ready**, and shipping
what looked ready would have undone work already on the truck. Full table in `docs/PENDING-WORK.md`.

| what it looked like | what it actually was |
|---|---|
| `policeship2pnw` "ready (UNPUSHED) @ `126dcb1d92`" — a week idle | **already shipped** as `47be7e2175`, an ancestor of the channel. The SHA in PENDING-WORK predated a rebase. |
| `pscmlimlog2pnw` — built, Fable-reviewed | ⛔ would move `opendbc_repo` **`7ff5d541` → `97be35a7`**, reverting `units2pnw` — the km/h cruise-speed fix shipped 09-14. Its content is already on the channel as `c1f3ffd1ab`. |
| `policemiss2pnw` — recent (09-14), 3 commits | ⛔ predates `policeship2pnw`; its diff **removes** the 1 Hz TTL re-check and the Rule 2 `err` passthrough that makes a held police report explain why it is not refreshing. Only a few UI lines are novel. |

**Every** unpushed branch carries a stale `opendbc_repo` pin — six different ones, the oldest from July.

**The rule that came out of it:** "ahead of `origin/3devpnw`" means nothing. Check (a) is the tip already an
ancestor of the channel, (b) does its submodule pin match, (c) does `git diff origin/3devpnw <branch> -- <src>`
contain `-` lines that delete shipped work. One command each; each one caught a real regression today.

## Corrections to things this workbench believed

* **`CLAUDE.md` said "there is no openpilot 0.11.2".** True in May, wrong since. Corrected twice, because the
  first correction over-claimed: there is **no `v0.11.2` tag**, commaai now ships per-hardware
  `release-<hw>` branches, `RELEASES.md` reads 0.11.2 (2026-08-12) on `master`/`nightly`/`release-chestnut`,
  and **`release-tizi` — the comma 3X channel — is still 0.11.1 on the OLD tree layout**.
* **"31 satellites" was a sentinel, not a measurement.** `GPS_Sat_num_in_view` is 5 bits `[0|29]` with
  `VAL_ 31 "Invalid"`; the truck emits 31 constantly (83,084 publishes, zero real counts). The truck's GPS is
  still the better fix — on **HDOP 0.4**, which is real — but the satellite count never supported that claim.
* **A grep trap worth keeping:** searching the device swaglogs for a feature name returns hits that are the
  **`updated` daemon echoing the commit message**. It read as "the feature is alive" three times in one night.
  Check the daemon field, never the bare count.

## Designs written and reviewed today (nothing built for the car yet)

| doc | state |
|---|---|
| [`CURVEDB2PNW.md`](../../../docs/CURVEDB2PNW.md) | **v1 REJECTED** by Fable (12 defects; the proof of concept was a dead Tesla sensor on the opposite carriageway). Rewritten as **two phases**; **Phase 1 approved to build** with 7 changes, Phase 2 gated on 6–8 weeks of data and a replay showing zero real slowdowns cancelled. |
| [`MAPDCARGPS2PNW.md`](../../../docs/MAPDCARGPS2PNW.md) | **BUILD WITH CHANGES** (8). v1 was car-only with a stall; rewritten to **relay the SELECTED fix**. Implemented and reworked; in review. |
| [`ICBMFALSIFY2PNW.md`](../../../docs/ICBMFALSIFY2PNW.md) | Designed today; the owner wants it for the 09-17 drive home. Implementation running. |

### What the night's analysis settled about ICBM phantom slowdowns
Four corroborators measured and **rejected** — `icbm_map_sanity` (7 real curves suppressed), the polyline
veto (33 vetoes / 5 false, `icbmK` under-reads by up to 77×), vision as a veto (only unbiased inside ~50 m =
1.7 s at highway speed), and raising `ICBM_FLOOR_MAX_LIMIT` (forbidden on physics). **There is no safe way to
veto a curve slowdown BEFORE the curve on this truck.** What survives is the **abort** rule: 35 aborts,
**0 false aborts**, **759 s** of unnecessary slowdown cut across 123 map episodes — in-sample, and it buys
only ~3 s of the 34 on the 09-08 event, because the speed is lost on the approach.

Also measured, answering the owner's question about combining mapd and vision: today `icbm_curve_target()`
takes the **`min()`** of map, vision and far-map — **vision can only ADD slowing and can never talk the map
down at any range.** Over the weekend the deciding source was map 1123 ticks, far 1032, **vision 6**.

## Shipped

| Commit | What changed | Notes |
|---|---|---|
| `23f0f47d59` **mapdcargps2pnw** (installed 19:11 PT, BootCount 249) | **mapd can now navigate from the truck's own GPS** — driven by the Chestnut GPU install, which comma say interferes with the comma 3X's receiver. mapd used the modem fix exclusively and never checks `hasFix`. It **relays whichever fix the Python side already selected**, so both halves stay in one frame, mapd stalls only when both receivers are dead, and it **inherits `gpsfix2pnw`'s `hasFix` gate for free** — fixing the known "mapd consumes a no-fix position 2.2 km off" bug without forking mapd. Param `MapdUseCarGps`, **default OFF**. | **No mapd patch needed**: `cereal/gps.go` already prefers `gpsLocationExternal` and latches to it, and that service was empty (0 messages vs 60 for `gpsLocation`). Fable **SHIP**, both optional hardenings applied: the relayed Event now sets `valid=True` like the real publishers, and the `get_bool` is contained — an `UnknownKeyName` would otherwise fire every loop at 20 Hz and take the whole mapd→CES bridge with it, which the first commit message understated. `horizontalAccuracy` and `satelliteCount` are hard **0** on both branches: the first is mapd's way-matching tolerance (`way.go:361`) and inflating it widens adjacent-ramp matching; the second is the DBC's `Invalid` sentinel. `mapd_configd` is now `restart_if_crash=True` since it becomes mapd's sole GPS source. 163 tests, **23/23 mutants**. Verified after the reboot: `MapdUseCarGps=0`, **zero** relay log lines and zero `gpsLocationExternal` messages — the default is genuinely inert — selfdrived/mapd_configd PIDs stable, one soundd boot assert and nothing else. |

## ⛔ Late 09-16 — the abort rule the owner wanted for the 09-17 drive is DEAD CODE, and shipped work already covers it

`icbmfalsify2pnw` is finished — 66 tests, **21/21 mutants killed**, committed `894e04a059` — and **deliberately
not pushed**. `behindrun2pnw` (`57d79657bc`, shipped 09-15, installed the same night) drops a passed map point
once it is 5 m behind along mapd's path; the abort rule cannot arm until the candidate has receded **10 m**
from closest approach *and then held 2 s*, which is always strictly later. Measured end to end through the
real `_icbm_step` on all 7 replay windows:

| | gate reached | verdicts | cap-ends |
|---|---|---|---|
| behindrun **LIVE** (today's truck) | **252 ticks** | 244 `approaching`, 8 `gap`, **0 falsified** | **0** |
| behindrun **OFF** (the code the design was measured on) | **330 ticks** | 52 `falsified` | **27 / 3 episodes** |

Not a harness artifact — a uniform zero is exactly the shape Rule 2 says to distrust, so it was checked by
turning behindrun off on the same corpus, which produces 52 falsifications. The design's 35-abort / **759 s**
corpus stands as recorded: it was measured **09-11..13, before `behindrun2pnw` existed**, and is not retracted.
Residual value is only where behindrun declines — **2 of 23,828** moving weekend ticks, and honestly
">= 0.008 %" rather than exactly it, because `icbm_passed_points` has six decline reasons and that study saw
three. Recommendation recorded in `docs/PENDING-WORK.md` and design-doc §10: keep the branch as the record,
revisit only if `behindrun2pnw` is ever narrowed. `TestPreemptedByBehindrun` fails loudly the day that changes.

**Worth stating plainly: the owner's drive home is already covered — by something installed the night before,
earlier in the episode and without needing a measurement.**

## 🗄 Late 09-16 — the CES corpus now leaves the device (owner request)

Owner, on being told the archive is write-only while the Phase 2 gate wants 6–8 weeks retained *and*
reachable: *"add all the logs including the ces log to aws s3 upload."* Built as `ceslogup2pnw`, stacked on
`curvedbtel2pnw`: the archived generations ride the **existing** S3 uploader under a `pnwlogs/` key prefix,
**in place** (no copy, no hardlink, no staging — the deleter must never be able to reclaim them, and a
hardlink would defeat the archive's own 2 GB budget), on unmetered links only, **last** in pass 1 so drive
data always goes first, with a `.zst` key on a plain file so `do_upload` compresses ~10× on the fly.

`/data/pnw` holds the Waze API key, so eligibility is an explicit allowlist, never a sweep: a named
directory, a required filename prefix, **and** a regular-file test. The third gate is not belt-and-braces —
Fable defeated the first two with a symlink wearing the right name, which `open()` follows.

## In flight at the time of writing
Both halves of curvedb Phase 1 are **built and in Fable review, none pushed**: `curvedbtel2pnw`
(`e6918f5a50`) and `ceslogup2pnw` (`f0911ba3d8`, stacked on it). Fable returned **SHIP WITH CHANGES** on the
first commit of each; the must-fix commits are what is under review now. `mapdcargps2pnw` **shipped** (see
above). `icbmfalsify2pnw` is finished and held, by recommendation, as an owner decision. Each ship gets its
own reboot and health check, as always.
