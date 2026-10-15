# CHANGELOG — 2026-10-15 (Thursday)

All times PT. `curvedbraiseonly2pnw` shipped to `3devpnw` at `e1d905a986` (`2d76dda741` plus review-fix commit `e1d905a986`), Opus-reviewed (SHIP-WITH-FIXES, fixes applied), channel tip green, and installed on the comma 2026-10-15 ~01:05 PT.

## 1. `curvedbraiseonly2pnw` (owner-approved 2026-10-15, "fix that curve database")

- **Problem.** A branch refused at build time by the date-spread rule ("dates disagree", `roadtable.row_verdict`) was exported with `k = null`, so the car saw
  `noAuthority`. A well-driven curve whose branch window pooled a merging side road lost its row, and the Lightning's ICBM fell back to mapd's raw rating
  (~10 mph too low on every pass).
- **Change.** `v2_live_export.py --raise-only` exports such a branch (and ONLY such a branch) as a raise-only row: `k` = the sharpest single pass
  (`roadtable.raise_only_info`; honours `exclude_date`), branch element 5 = bitmask 2, element 6 = the fastest recorded pass speed, manifest flags
  `lowerBound,raiseOnly` + `raise_only_rows`. `PassObs` gains `v_ms` (the MINIMUM speed over the pass's k extent, i.e. the in-curve speed, not the speed at the anchor; NaN in older tables - such a branch does not qualify).
  The speed cap uses only non-Tesla passes of known cars. `--raise-only` exits non-zero and writes nothing when 0 rows qualify.
  Lightning: may only raise, capped at fastest non-Tesla pass + 5 mph and every existing cap; never in the ADD scan. Tesla: byte-identical behaviour (coverage 3).
  Semantics: `docs/pnw/VTSC-DB-FIRST.md` section 3a.
- **Telemetry.** `cdb2RO`, `cdb2Last` on the tick; `curvedb_v2_loaded` gains `raise_only_rows`, `age_days` and a warning when the table is older than 14 days (evaluated once, when the clock first becomes valid; it cannot kill the load thread).
- **CODE BEFORE TABLE.** The new file is rejected loudly (malformed branch, DB OFF on BOTH cars) by a loader without this change. Install the code, verify
  the boot log shows `curvedb_v2_loaded` with `raise_only_rows`, then install the table; on rollback restore the previous table first. A table built
  WITHOUT `--raise-only` loads in both old and new code.
- **Golden (`tools/curvebrain`).** Against the 2026-09-27 set with `--exclude cdb2RO --exclude cdb2Last`: no `pub`/`rec`/`st` line changes; only `log`
  differs, on the lines carrying the `curvedb_v2_loaded` event (new fields `age_days`, `raise_only_rows`). The goldens need a re-record.
- **Test fixture.** `roaddb_fixture.isolate` writes today's date as `last_date` so the stale-table warning cannot fail nothing-logged tests; `write_db`'s default
  stays constant (the golden records through it).

All times PT.

## `icbmlatestart2pnw` (branch `icbmlatestart2pnw`, not yet pushed): a roundabout slows later and arrives at an appropriate speed

Owner 2026-10-15: "the roundabout should slow later and arrive at an appropriate speed, not crawl".

**Cause.** Not the 0.8 m/s2 start envelope. The icbmtrack2pnw TRACKING WINDOW start rule (`_icbm_track_apex` / `icbm_track_window_m`: window =
max(ref, v) x (steps x 0.4 s + 4 s), capped 350 m) also fires on an open road, so the cap started ~50-60 m before the envelope would; the executor
then jumped the set to the apex (truck minimum set 15 mph). The truck cannot slow faster through set taps (large drops average 0.78-0.87 m/s2), so
the decel assumption is NOT raised and there is no walk-down: only WHEN the cap starts changes.

**Change.**
- `icbm_curve_target(..., late_drop=0.0)`: when > 0 and `ref - effective apex >= late_drop`, the tracking-window start is skipped for the MAP and
  FAR candidates; they may start only through the v_ego decel envelope (0.8 m/s2 + 30 m). Default 0 = byte-identical to before.
- `_icbm_step`: `late_ok` = START tick only (episode not in its cap phase) AND no lead seen for 5 s (`ICBM_LATE_LEAD_WAIT_S`) AND not lead-bound
  (`v_ego >= ref - 1.5 mph`), pure `icbm_late_start_gate`. On a late tick `firm = 0.0` (the far scan and the start rule both price at the comfort
  decel, so the firm-ramp ranking cannot hide a tighter point) and `late_drop = knob`. The tick's two values are stored (`_icbm_firm_t`,
  `_icbm_late_t`) and read back (`_icbm_tick_params`) by the far scan, both main calls, the passed-point gate, the shape stage and the curve-DB
  path, so every call of one tick decides on the same inputs. A running cap episode, a lead / lead-bound start, vision, restore and the penalties are
  unchanged (the running-cap exemption matters: a truck decelerating faster than 0.8 would otherwise un-bind its own cap and restore mid-approach).
- Knob `icbm_late_start_drop_mph` in `/data/pnw/curve.json` `lightning` section: default **18**; `0` = off; a value in (0, 18) is clamped UP to 18
  with a logged warning (a bad config can only drift toward the default rule); > 60 clamps to 60; an unreadable file is logged and the default
  stays. `PnwVehicle.icbm_late_start_drop_ms` returns 0.0 off the Lightning (ICBM runs only under the Lightning shadow path; the Tesla is untouched).
- Telemetry `icbmStartWhy` (ces_events tick records, and `IcbmTarget` state): `env` (the envelope started it), `track` (tracking window, small
  drop or knob off), `trackLead` / `trackBound` (tracking window, and a lead / lead-bound truck is why it was not held to the envelope),
  `lateWait` (the tracking window WOULD have started it and the late rule withheld it), `err` (classification itself failed; logged), else null.
  START ticks only.

**Effect (synthetic roundabout, v 20.2 m/s, set 20.6, rated curve 6.8 m/s):** first start moves from ~330 m (tracking window) to ~255 m (envelope).
In the 51 -> 14 mph geometry a late tick starts on the 0.8 envelope (~330 m) instead of the firm-ramp distance (~286 m).

**Tests** `ces_pnw/tests/test_icbm_latestart.py` (34): pure function (no start at 300 m, start at 250 m, default 0 regression, small drops
unchanged, threshold on the drop), gate, firm=0 on the far scan, lead now / lead 3 s ago / lead-bound byte-identical to knob off, running cap
keeps the tracking window, closed loop through the Ford executor (a truck decelerating faster than 0.8 keeps its cap bound, no clear/restore),
all call sites of one tick agree (step, shape, curve DB, passed-point gate), knob (default, 0, clamp-up, non-Lightning 0.0, unreadable logged),
telemetry values, roundabout fixture.

**Review fixes (Opus BLOCK, same day), second commit:**
- F1 flapping: a cap that started late keeps the start tick's envelope (firm 0) on its running ticks (`_icbm_late_latch`, cleared when the
  episode leaves the cap phase). Before, the next tick reverted to the firm ramp + the 350 m-capped window, un-bound the cap within 0.6 s and
  the next tick re-started it (2 Hz engage/clear; each re-engage latched the ceiling at our own lowered set, 70 -> 61 mph lost). Closed-loop test at
  65/70/75 mph from 520-560 m: exactly one engage, ceiling never moves.
- F2 penalty gap: the late rule now applies only to apexes at or below `low_v_mph` (30 mph; `PnwVehicle.icbm_late_start_apex_max_ms`). The
  penalties lower the target AFTER the envelope was sized, so a 40-47 mph apex arrived 3.5-5 mph over target; those keep today's tracking start.
- F3 shallow-first: on a late-eligible tick, if a shallower curve that is NOT covered by the rule would itself start the episode by the tracking window
  (`icbm_far_map_candidate(skip_late=...)` probe, unpriced), no late start that tick (`icbmStartWhy` = `trackShallow`). Could not reproduce a
  failure in the closed loop (arrival at the deep node within 0.5 mph of today across a sweep with and without the probe); kept as a precaution.
- A `restore`-phase tick is no longer a START tick; a lead seen during a Chill interlude now updates the 5 s lead clock.

**Second review round (same day), third commit:**
- Owner-visible: at highway speed into a tight (<= 30 mph) node the cap now starts EARLIER than today's firm ramp. The old 1.4 m/s2 assumption is not
  achievable through set taps (large drops average 0.78-0.87), so today the truck arrives too fast; if it brakes harder than 0.8 it reaches the apex
  speed a bit early instead.
- A late-started cap now HOLDS its covered candidate (apex <= 30 mph, drop >= knob) while it is ahead: the 350 m tracking cap and an envelope that
  shrinks when the truck slows faster than 0.8 m/s2 can no longer un-bind it (reviewer repro: 65 mph, 29 mph apex, 1.6 m/s2 = 7 engages, ceiling 65 -> 58;
  now exactly one engage, unchanged ceiling at 1.2 and 1.6 m/s2).
- Scope: firm 0, the latch and the hold apply only when the candidate binding at the comfort decel is covered. A 33-43 mph apex keeps today's trace
  exactly (was starting earlier, e.g. 489 m vs 426 m). The rule's apex bound is its own 30 mph constant: `low_v_mph` (curve.json, up to 80) cannot widen it.
- `icbmStartWhy` = `trackShallow` only when a deeper covered node exists and was withheld; ordinary small-drop tracking starts stay `track`.

**Final check (same day), fourth commit:** the hold is for the STARTING candidate only. It had bound ANY covered node while the latch was set,
so between two roundabouts 400-500 m apart the truck crawled at 15 mph for 430-530 m (knob off: back to 37-41 mph). The starting node's
along-path distance is now dead-reckoned (distance at the last tick - v_ego * dt, re-anchored on the matching candidate within 40 m each tick);
only a covered candidate within 40 m of it is held, and the latch ends 25 m past it. The next node goes through the normal start logic on its own
merits. Regression test: two nodes 400/450/500/600 m apart (45 mph, real Ford executor) - the knob is never slower than off between them.
Still true: highway single node = one engage at 1.2/1.6 m/s2, release after passing, 8 m GPS jitter = one engage, a vanishing node releases.


**Final check, round 2 (same day), fifth commit:** map-node distances are unsigned, so past the start node a node the passed-point gate cannot call passed (slow
< 5 m/s on a tight roundabout, no heading) read as a positive distance again and the re-anchor pulled the tracked distance back up: the 25 m pass exit never fired
and the latch lived until the node left the 500 m horizon (cap ~150 m longer than origin). The re-anchor can now raise the tracked distance by at most 5 m per
tick and stops within 5 m of the node. Tests: roundabout rated 3.5 / 4.5 / 5.5 m/s with and without a heading - the latch ends within 45 m past the node
and the cap ends no later than origin's.
