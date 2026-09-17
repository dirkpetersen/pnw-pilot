# CHANGELOG — 2026-09-17 (Thursday)

Continues [`CHANGELOG-2026-09-16.md`](CHANGELOG-2026-09-16.md). That file's overnight work lands here.

**Channel tip:** `origin/3devpnw` = `267f665604` · **on the truck:** `23f0f47d59`, `BootCount` 249 — the truck
is **11 code commits + 2 docs commits behind** until the next fetch+reboot, and they will all install together.

> ⚠️ **This ship could NOT be verified on the car.** The comma is on **Starlink** — LAN `192.168.1.79`, public
> `98.97.43.186`, behind CGNAT — so there is no inbound route from the dev host and SSH is impossible, not
> merely blocked. No `SIGHUP` to force a fetch, no reboot, no post-install health check. The updater fetches
> on its own ~1.5 h cycle and installs at the next ignition-off. **First evidence the install took will be
> `/data/pnw/ces_archive` generations appearing in the S3 upload stream**, not a probe:
> `aws --profile dipeit s3 ls s3://comma-connect/drives/2fd850c60cc5bfef/pnwlogs/`.
>
> Both halves are **pure Python** — no new param keys, no `cereal` change — so no `params_pyx.so` rebuild is
> needed and the `UnknownKeyName` UI crash-loop class of failure is structurally unreachable here.

---

## Shipped

| Commit | What changed | Notes |
|---|---|---|
| `4b901737b2` — **curvedb Phase 1**, 9 commits (`curvedbtel2pnw` 1–6 + `ceslogup2pnw` 1–3) | **The truck now measures curves properly, keeps the measurements, and gets them off the device.** Three parts: (1) §3 telemetry — `kPeak`/`kPoseP` (per-second curvature PEAK, not the 1-in-100 sample), `kPose`/`achLatPose` (livePose-derived achieved curvature, **the only one the Tesla has** — `CS.yawRate` is 0.0 on 7,218 of 7,221 moving Raven ticks), `dq`/`dqWhy` (per-second disqualifier OR), `strTq`, and `mapLat`/`mapLon`/`mapCandD` (where the CURVE is, not where the truck is). (2) §3.8 retention — the generation `rotate_event_log` used to DESTROY is now `os.replace`d into **`/data/pnw/ces_archive`** (2 GB ≈ 95 days) instead of overwritten. (3) The archive rides the **existing S3 uploader** under a `pnwlogs/` key prefix. **TELEMETRY ONLY — nothing reads any of it, and Phase 2 stays gated on §7.** | **Four Fable rounds.** See below — rounds 2 and 3 each found a defect that the feature's own tests could not see. 862 tests in `ces_pnw/tests/` (849 before), 32 in `test_ceslogup2pnw.py`, **17/17 + 21/21 mutants**, loggerd 20 failed/73 passed against a control baseline of 20 failed/67 passed with the failure sets diffed IDENTICAL. No submodule pin moved. |

### What four review rounds actually caught

Worth recording, because in three of the four rounds the defect was **a check that passed for the wrong
reason** — not code that was obviously wrong.

| round | verdict | the finding |
|---|---|---|
| 1 | SHIP WITH CHANGES | `mapLat`/`mapLon` keyed on CES's 10 s candidate (`mapDist`) instead of ICBM's 500 m one, so they were **null on exactly the far-candidate phantoms §3.4 exists to locate**. Also: the 2 GB budget was justified "against the 8.9 GB free on /data" — a misread of deleterd's floor, which is pinned there by construction. Archive bytes displace drive segments **1:1**. |
| 2 | SHIP WITH CHANGES | The candidate distance is measured from ICBM's **projected** position but was re-matched against the **raw** fix, so the matcher's 30 m tolerance returned **the neighbouring node** — the 300 m candidate logged as the 339 m node. The integration test passed *because both its assertions were measured against the same wrong number*. Also: the "negative control" for non-map sources could not fail — the harness never produces a vision or restore record. |
| 3 | SHIP WITH CHANGES | ICBM latches the point up to 250 ms before the record is built, and `_publish_status` runs *before* `_icbm_step` in the same 100 Hz cycle — so a GPS blip in between emitted **coordinates with no distance**, which is precisely the orphan the feature's own acceptance checker FAILS on. A single blink of the fix would have failed the instrument on the first drive. |
| 4 | **SHIP** | No must-fixes. Three optional gaps taken: the "I3 filters on mapDist again" mutant was being killed **by a TypeError, not by detection**; `Report.rows` was initialised and never appended (a field that exists and is always empty — the visK shape, in the checker itself); and the no-fix guard covered a missing fix but not a failed latch. |

Round 4 also answered the ship question explicitly, by enumerating every operation rather than trusting a
docstring: **nothing in the nine commits can raise into the control loop, change `IcbmTarget` / a cap / a
target / a gate, or alter disk writes beyond the documented archive.** The new state (`_curve_peak`,
`_pose_k`, `_str_tq`, `_icbm_cand_d`, `_icbm_cand_pt`) has exactly three readers tree-wide: `_curve_tele`,
the two record builders, and the checker's writer table.

### Two claims in the code that turned out to be false, and are now fixed

* **"Two independent gates."** `/data/pnw` holds the Waze API key, so upload eligibility is an explicit
  allowlist — a named directory AND a required filename prefix. Fable defeated both with a symlink wearing
  the right name, which `open()` follows: `ces_events.jsonl.link -> …/police_proxy.json` would have uploaded
  the key to S3. There is a **third** gate now (regular-file test), and the same check fixes a non-security
  bug — a *directory* named `ces_events.jsonl.somedir` returned `getsize()` 4096, raised `IsADirectoryError`
  on open, and retried every 15 minutes forever with no error code and therefore no `UP ERR` anywhere.
* **"No upload can overwrite an earlier one in S3."** The gateway presigns a plain `put_object` with no
  `IfNoneMatch`, returns no 412, and silently overwrites. The device-side `.N` collision loop only
  disambiguates while the earlier file still exists — and the 3X's **dead RTC** stamps pre-NTP rotations
  `1970`, which `prune_ces_archive` (sorting by mtime) always evicts FIRST, freeing the name. The quiet half
  was worse than the overwrite: `xattr_cache` memoises "uploaded" against the **path**, so a reused name
  inherits the previous file's mark and the new generation is **never sent at all**. Fixed where the name is
  chosen — a random `.b<hex>` token when `st_mtime < CLOCK_VALID_EPOCH`.

### The trade this ship makes, stated plainly

deleterd pins free space at `max(5 GB, 10 % of /data)` by evicting the oldest **drive segments**, so `/data`
reads ~8.9 GB free no matter what else is stored. The archive lives outside `Paths.log_root()` and the
deleter cannot reach it — so **every archive byte displaces a drive segment 1:1**. 2 GB ≈ 150 segments
(~2.5 h of driving) evicted earlier than they otherwise would be. Deliberate, and it is why the upload half
shipped in the same push: an evicted segment is already in S3, whereas an evicted generation had no other copy.

---

## ⛔ The abort rule the owner asked for is preempted by work that shipped two days earlier

`icbmfalsify2pnw` is finished — 66 tests, 21/21 mutants — and rebased onto this tip. **`behindrun2pnw`
(`57d79657bc`, shipped 09-15) ends the episode strictly earlier, every time**: it drops a passed map point
once it is 5 m behind along mapd's path, while the abort rule cannot arm until the candidate has receded
10 m from closest approach *and then held 2 s*.

| | gate reached | verdicts | cap-ends |
|---|---|---|---|
| behindrun **LIVE** (today's truck) | **252 ticks** | 244 `approaching`, 8 `gap`, **0 falsified** | **0** |
| behindrun **OFF** (the code §3's corpus was recorded on) | **330 ticks** | 52 `falsified` | **27 / 3 episodes** |

A uniform zero is exactly the shape Rule 2 says to distrust, so it was checked by disabling behindrun on the
same corpus rather than reported straight. The design's 35-abort / **759 s** figures are **not retracted** —
they were measured 09-11..13, before `behindrun2pnw` existed. Residual value is only where behindrun declines:
**2 of 23,828** moving weekend ticks, and honestly ">= 0.008 %" rather than exactly it, because
`icbm_passed_points` has six decline reasons and that study observed three.

The owner has been told this and has said ship anyway, so it **SHIPPED as `267f665604`** (2 commits) after
its first Fable review — Rule 8 does not bend for an instruction to hurry, so the review happened first.

### ⚠️ That review found the one bug that would have mattered, and it was in the TESTS

`achLat` is **signed** — negative is a right-hand bend. `icbm_measured_curvature` takes `abs()` of it before
dividing by v². **Drop that one call and every right-hand curve reads a NEGATIVE `k_meas`**, which makes
`k_meas * RATIO < k_map` true for *any* map claim at all: the rule would then falsify every right-hand curve
after its 2 s hold, cancelling real slowdowns **on one side of the road only**.

The code was correct. Nothing pinned it. The `abs_dropped` mutant **survived all 21 original mutants and all
930 tests** — every synthetic case used positive curvature, and although 6 of the 7 fixture windows contain
negative `kActl`, none of them arrives-and-measures on a right-hander. Pinned two ways now (a direct
both-signs assertion, and the real-curve integration test parametrized over both directions); 22nd mutant
added; **932 tests, 22/22 killed**. A one-sided failure is exactly the kind nobody thinks to go looking for,
and it took an adversarial mutant to find it rather than a reading of the code.

Fable's safety verdict on the rule itself: it **cannot end a cap for a node the truck has not yet reached** —
checked over 300 synthetic two-node scenes through the real `_icbm_step` (5 velocity pairs × 5 spacings ×
6 speeds × behindrun on/off) plus all 7 real windows, with a geometric reason rather than an absence of
counterexamples: after a 5 m-behind removal the next node is within `15 + v·dt`, so the "arrived" window
lasts ≤ `4/v + dt` ≤ 0.75 s at the 8 m/s floor and can never reach the 2 s hold. Fable also retracted its own
interim "confirmed hole" — that was a harness bug, a second node placed off the 20 m point grid.

Two smaller Rule 2 items taken: `icbm_falsify_tick`'s "Never raises" docstring was false for a denormal
target (`t*t` underflows to 0.0 while `t > 0` passes → `ZeroDivisionError`), and `TestPreemptedByBehindrun` —
whose entire job is to fail the day preemption stops being true — did not assert the gate was ever
**reached**, so it would have passed vacuously once a change made the gate unreachable. Both fixed. A
tripwire that cannot trip is not a tripwire.

> **Both pushes land in ONE install.** The device had not fetched `4b901737b2` before `267f665604` went out,
> so the next reboot installs 11 commits at once and per-change attribution is lost — normally against the
> one-change-per-install rule. Accepted here because Phase 1 is recording-only and this rule is
> measured-inert, so neither can change how the truck drives; and because there is no route to the device to
> stage them separately anyway.

## Corrections to things this workbench believed

* **`lcroc2pnw` is NOT "a branch, not merged"** — its tip is an **ancestor of the channel**, i.e. shipped.
  `PENDING-WORK.md` said otherwise under the Tesla over-steer item, which is the highest-severity open item,
  so the error mattered. Verified in the channel's own source rather than from the branch graph:
  `correction_roc` is a **live hot-reloadable knob** (default `0.0009` 1/m·s, clamped `0.0001–0.0018`, read
  from `/data/pnw/lanecenter_tuning.json`), and `lcLimN` is emitted in **both** `ces_events` record families
  including the CES-off `"ev":"steer"` breadcrumb. **So the forensics that item asks for is already
  recording, and a candidate fix is testable in one ignition cycle with no deploy and no push.**
* **`mapdstate2pnw` shipped by another route.** Its `coverage.py` at `b20b0c5b32` is byte-identical to the
  channel's and `region_for_gps`/`RefreshLocationMap` are live. The local branch is a stale duplicate.
* **`mapdcargps2pnw` was still listed as "NOT pushed, NOT reviewed"** in `PENDING-WORK.md` at SHA
  `f7a008a54d`. It shipped 09-16 as `23f0f47d59`. Same stale-SHA class as the `policeship2pnw` entry.

### Stale-branch audit (the three checks the 09-16 entry prescribes)

| branch | commits NOT on channel | channel commits it is MISSING | verdict |
|---|---:|---:|---|
| `lcroc2pnw` | **0** | 201 | ✅ shipped |
| `mapdstate2pnw` | 2 | 262 | ✅ content shipped elsewhere; delete the branch |
| `lcabort2pnw` | 2 | 64 | real, held by the owner |
| `redlight-stop2pnw` | 1 | 262 | real, a month old |
| `speedadjustreset2pnw` | 1 | 249 | real, a month old |
| `uicpu2pnw` | 6 (4 docs) | 375 | real, two months old |

> ⚠️ **The table above counted the wrong thing, and it is corrected here the same day.** "Commits not on the
> channel" over-counts: a commit can be absent while its CONTENT is present, because the work was re-ported or
> rebased and shipped under a different SHA. Re-audited by diffing the actual FILES against `origin/3devpnw`:
>
> | branch | first verdict | truth |
> |---|---|---|
> | `redlight-stop2pnw` | "real, a month old" | ✅ **SHIPPED** — `drive_helpers.py` byte-identical to the channel |
> | `speedadjustreset2pnw` | "real, a month old" | ✅ **SHIPPED** — all three parts, incl. the on-device-confirmed `_cur_speed` AttributeError crash fix |
> | `uicpu2pnw` | "real, two months old" | ✅ **SHIPPED** — `_fast_param_time`/`_record_audio_param` live in `ui_state.py` |
> | `mapdlog2pnw` | not audited | ✅ superseded by `mapdlogmgr2pnw` — see below |
> | `madsbrakerace2pnw-blocked` | not audited | ✅ **SHIPPED** — `MADS_BRAKE_GRACE_FRAMES = 45` (branch had 50, tuned in review) |
> | `lcabort2pnw` | "real, held by the owner" | ✅ correct — the **only** branch with unshipped content (17 lines in `desire_helper.py`) |
>
> So there is exactly ONE branch left with unshipped work and it is owner-held. Nothing needs re-porting.
> All of them still carry a stale `opendbc_repo` pin and still must not be merged.

🔪 **`mapdlog2pnw` is the sharpest example of why "ahead of the channel" means nothing.** Its one commit adds
a `cloudlog.warning()` inside `installer.py` — an approach later MEASURED NOT TO WORK: the installer is a
subprocess that exits in milliseconds, before `logmessaged` is up, and swaglog's socket lingers only 10 ms,
so its cloudlog lines are dropped at boot (on-truck 2026-09-13: override active, pin ignored, **zero**
warning lines). The channel instead has `mapdlogmgr2pnw`, where **manager logs it itself** via
`present_status()` — a function the branch does not contain while the channel's `manager.py:147` imports it.
Merging it would replace a working fix with one proven not to work, and risk an ImportError in manager at
boot.

## 🔴 Afternoon — Phase 1's first real drive found a bug in Phase 1, exactly as designed

Installed at the 12:47 ignition. Four hours later the section-3.7 acceptance checker, run against the
first real corpus (46 min, Lightning, pulled over SSH), **FAILED on I2** — and it was right.

**`kPose` / `achLatPose` shipped SIGN-INVERTED.** Not noise, not scale:

* median `kPose/slKActl` = **−0.892** (n=1,074 moving).
* on unambiguous bends (|k| > 0.0008 on BOTH): **285 of 292 opposite — 97.6 %**, |ratio| median
  **0.992**. Magnitude always right.
* third witness, the steering wheel (|strAng| > 8°, n=213): `slKActl` **95.3 %**, `achLat` **95.3 %**,
  `kPose` **0.5 %**.

**Root cause, from frame definitions rather than from the correlation:** the device frame is
`[Forward, Right, Down]`, so +z about a downward axis is a RIGHT turn, while `carState.yawRate` is
ISO-8855 positive-LEFT. openpilot negates the same quantity itself at `paramsd.py:98`. Shipped as
**`36f914a17c`** (Fable SHIP, 18/18 mutants); on the corpus I2 moves **−0.892 → +0.892, FAIL → PASS**.

**Why a sign mattered here more than signs usually do:** `kPose` is the ONLY achieved curvature the
Tesla has (D1 — `CS.yawRate` is 0.0 on 7,218 of 7,221 moving Raven ticks), and the curve DB keys rows
on (site, **heading**). Every Tesla curve would have been stored bending the wrong way. On the
Lightning it is **invisible**, because `slKActl` is there and correct — so without the signed
cross-check it would have surfaced months later as the two cars inexplicably disagreeing, inside a
database already feeding braking. "Is the field alive" could not see it. Only a signed comparison
against a second source could.

**And the checker itself had a defect the same corpus exposed** (`837326fe52`, checker + tests only):
**I3b would have gone red on every drive.** It flagged any map candidate within 1 m of the truck; one
record had `mapDist` 1.0 m with the node 0.71 m away — the truck was driving OVER a map node, which
happens constantly. A gate that always fails is one a human learns to ignore, so it now tests
CONSISTENCY (coordinates on the truck *while `mapDist` says metres away*) and prints the benign count
rather than dropping it.

## 🟠 Afternoon — the driver's live complaint, now quantified: ICBM over-slows

Driver, 14:25 PT: *"button control management also took me down to 38 mph… it's going too slow."*
Full analysis: [`drives/2026-09-17/curvedb-first-capture/DRIVE_REPORT.md`](../../../../drives/2026-09-17/curvedb-first-capture/DRIVE_REPORT.md).

**The event.** ICBM commanded **38 mph** where the map asked **44**. Measured `kPeak` 0.00394 →
**R ≈ 254 m**. At 38 mph that bend is **1.1 m/s²**; at 50 mph it is 2.0; at the driver's set of 62 it
would have been 3.0 — so the curve was real, and **38 was ~12 mph more slowdown than the geometry
justified.**

**Across the day** (same-tick, map-sourced, >25 mph, k > 0.0015 — **n = 39**):

| | |
|---|---|
| commanded BELOW the map's own target | **22/39 = 56 %**, median **−2.8 mph** |
| lat accel at the ICBM-commanded speed | median **1.40 m/s²** |
| lat accel at the map-asked speed | median **1.62 m/s²** |
| design target `A_LAT` | **2.50 m/s²** |

**Two contributions, and the second is the larger:** ICBM undercuts the map (the Lightning
`curve_speed_penalty_ms` stacking on an already-reduced apex), **and the map's own numbers are
conservative** — even at `mapV` the truck only pulls 1.62 m/s². ⚠️ **n = 39, one drive, one corridor;
re-run over a week before touching a constant.**

Also captured: 310 s of angle saturation and 350 s of driver steering override across 6,782 moving
seconds (4.6 % / 5.2 %) — the driver's steering warning is real but **not root-caused**; it needs the
exact time or the matching qlog.

## In flight

Nothing. Everything built is shipped; the four branches with real unshipped work need re-porting (table above),
which is work rather than a push.
