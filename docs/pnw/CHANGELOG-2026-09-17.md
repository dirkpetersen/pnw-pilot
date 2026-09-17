# CHANGELOG — 2026-09-17 (Thursday)

Continues [`CHANGELOG-2026-09-16.md`](CHANGELOG-2026-09-16.md). That file's overnight work lands here.

**Channel tip:** `origin/3devpnw` = `4b901737b2` (pushed 07:2x PT) · **on the truck:** `23f0f47d59`, `BootCount`
249 — the truck is **9 code commits + 2 docs commits behind** until the next fetch+reboot.

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

The owner has been told this and has said ship anyway. **It is in Fable review now** — it has never had one,
and Rule 8 does not bend for an owner instruction to hurry. Status will land in the next changelog entry.

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

**All six carry a stale `opendbc_repo` pin**, and none of the four live ones can be merged as-is — each is
64–375 channel commits behind, so merging would revert shipped work exactly as `pscmlimlog2pnw` would have.
They need **re-porting onto the current channel, one commit at a time**. That is work, not a push, which is
why "ship everything" does not reach them.

## In flight

`icbmfalsify2pnw` (`e8678635fb`, rebased, in its first Fable review). Nothing else is built and waiting.
