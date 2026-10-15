# CHANGELOG — 2026-10-14 (Wednesday)

All times PT. Both sections shipped to 3devpnw, Opus-reviewed, not yet installed.

## 1. `deleterhdfirst2pnw` (shipped 3devpnw `43f7021e78`, Opus-reviewed, not yet installed)

On the 10-10/10-11 Lightning road trip (no signal, disk ~90% full) the deleter destroyed ~77 GB of NOT-uploaded data (513 `deleterLoss`
events: 507 ecamera, 506 fcamera, 328 rlog), so a whole day of drives lost its rlog, the file analysis needs most (~10 MB/segment vs ~70 MB
of HD video).

Order before: `sorted(dirs, key=(boot/crash, user.preserve, has un-uploaded rlog/fcamera/ecamera))`, then the FIRST candidate was removed
with `shutil.rmtree` (whole segment dir, rlog included) - uploaded segments first, then the oldest un-uploaded one. dcamera and qlog/qcamera
are not in `FIREHOSE_FILES`, so they never counted as un-uploaded and were deleted silently with their segment.

Order now (same free-space thresholds, same sort key):
1. fully-uploaded segments (unchanged, whole dir);
2. once only un-uploaded ordinary segments remain: the HD video (fcamera/ecamera/dcamera) of the OLDEST un-uploaded ordinary segment that still
   has any is deleted, the segment's logs stay; one segment per 0.1 s sweep, repeated until no un-uploaded ordinary segment has HD video;
3. only then whole un-uploaded segments, oldest first, rlog included (as before).
boot/crash and `user.preserve` segments keep their existing, later place.

- `system/loggerd/deleter.py`: new `strip_hd_video()`; the sweep runs the HD pass before the first un-uploaded ordinary segment is destroyed.
  `deleterLoss` records now carry `"kind"` (`"HD sacrificed"` = HD video stripped, logs kept; `"LOG LOST"` = whole segment) and the
  since-boot totals `n_log` / `n_hd`. The offroad alert text has separate counters (segments whose logs were lost / segments HD-stripped,
  total MB), e.g. "3 segments' logs lost, 12 HD-stripped, 900 MB since boot", so a segment is not counted twice as "N segments".
  A loss event is emitted for every removed file the uploader would have sent (fcamera, ecamera unless SkipWideCameraUpload, rlog);
  dcamera is never uploaded by design and is removed without a loss event.
- Whole-segment delete now also checks qlog / qlog.zst / qcamera.ts: if un-uploaded they are named in the `deleterLoss` record when a
  firehose file was lost too; if ONLY these small files are un-uploaded it is a `cloudlog.error` line naming them and no alert
  (qcamera.ts is legitimately not uploaded on metered links).
- An HD file that cannot be removed is logged (exception) once per path per boot, not every 0.1 s sweep; the deleter moves on and, with no
  HD removable, falls through to the whole-segment delete.
- Uploader: no change needed. `list_upload_files` walks `os.listdir`, so a segment with logs and no video just yields its logs.
- Tests: `test_deleter.py` +9 (HD before any un-uploaded log, rlog survives while HD exists anywhere, uploaded segments go first, `LOG LOST`
  only when no HD is left, uploaded HD stripped without a loss event); `test_uploader.py` +1 (segment without video lists cleanly).
- Mutation check: HD pass disabled (4 fail), un-uploaded sorted before uploaded (2), HD loss event suppressed (4), small-file scan removed (2), remove-failure logged every sweep (1).

## 2. `cesasyncwrite2pnw` (shipped 3devpnw `2025540286`, Opus-reviewed, not yet installed): the selfdrived loop never does ces_events file I/O

Cause (drives/2026-10-11/system-lagging): the two `selfdrivedLagging` soft-disables (TAKE CONTROL IMMEDIATELY at 56 and 64 mph) were each a
single ~123 ms stall of the selfdrived loop; loopdiag2pnw put 96-99% of it in the synchronous append to `/data/pnw/ces_events.jsonl`
(`CESController._append_event`: the ces stage 112/116 ms, the write 107.5/114.8 ms) on a ~90% full `/data` with loggerd writing hard.

- `_append_event` now only `json.dumps` + `put_nowait()` on a bounded `queue.Queue` (2000). ONE daemon thread (`ces_events_writer`, log context
  bound to `daemon=selfdrived`) does the size check, `rotate_event_log`, archive hand-off and the append, unchanged (same file names, 20 MB
  rotation, 8 generations, open-per-append, no fsync). The park-edge `rotate_at_park` goes through the same thread.
- Rule 2: queue full or any enqueue failure (e.g. `Thread.start()` RuntimeError) -> the record is dropped and counted, never raised into the loop
  (a raise on the adopt path would force Chill), `cloudlog.error` at most once per 30 s with the running count; the message says "writer DEAD"
  once the restart cap is hit, "stalled" only for a live-but-slow writer. The cumulative drop count rides in the data as `evDrop` on every record
  written after the first drop, and the loopdiag slow-loop line shows `ces_event_drops` / `ces_event_qdepth`.
  Writer death -> logged by the thread, detected by `is_alive()` on the next `_append_event`, restarted (max 5, then loud). The writer warns when one
  append or rotate takes > 50 ms, so disk stalls stay visible. NOTE the loopdiag `ces_event_write*` fields now time the ENQUEUE (~0 ms), not the disk.
- Stop: `atexit` does NOT run in selfdrived (a forked child that exits via `os._exit`), so `SelfdriveD.run()`'s `finally` calls
  `ces_pnw.close_events(1.0)` (the manager SIGKILLs ~5 s after SIGINT): it drains the queue and logs how many records were queued / flushed / LOST.
  `atexit` stays as a harmless extra. A SIGKILL or a stalled disk can still lose the queue (<= ~2 records/s).
- STILL SYNCHRONOUS on the loop (same stall class, follow-up): `curvedb_shadow._write` appends one observation to the curvedb obs jsonl per curve
  pass, with up to 2 `getsize` + an append on `/data`. Its caller's verdict depends on the write result, so it was not moved here. Other
  `/data` I/O reachable from the loop in `ces_pnw/` is read-only (`curvedb_live`, `curve_brain`, load/reload) or `/dev/shm`. Writers in other
  processes (`accdrop_pnw`/`pscmlim_pnw`/`epsref_pnw` via `append_to_ces_log`, the deleter) still append to the same file.
- The stock selfdrived lagging check is untouched.
- Tests: new `ces_pnw/tests/test_cesasyncwrite2pnw.py`; `test_loopdiag_pnw` asserts the stall does not reach the loop and that `run()`'s finally
  calls `close_events`; autouse fixtures stop/join every writer before `CES_EVENT_LOG` is restored (no leftover thread can touch the real file).

## 3. `curvedbasync2pnw` (branch `curvedbasync2pnw`, not yet pushed): the curvedb observation write leaves the selfdrived loop too

The follow-up named in section 2. `CurveDBShadow._write` (reached from `self._cdb.tick`'s passage close, up to 2x per curve pass) did a
`getsize` + append of `/data/pnw/curvedb_obs.jsonl` on the loop. It now only `json.dumps` + hands `(shadow, line)` to the SAME ces_events
writer thread via `CESController._enqueue_event` (no new thread). The writer calls `CurveDBShadow.write_line`, which is the old size check +
append moved verbatim (`OBS_MAX_BYTES` freeze, `_write_stopped`, same file, open-per-append). Per-file order is the queue order.

- NOTE the task text assumed obs rotation/archive; the shipped code has none: the obs file is append-only and FREEZES at `OBS_MAX_BYTES`
  (never rotated). The `curvedb_archive` hand-off is a per-boot snapshot made in the shadow's constructor, before engagement, so it is not on the
  loop and was not touched. Thresholds, names and semantics are unchanged.
- Verdict (`cdbEv`) is now honest: `upQueued` / `downQueued` = handed to the writer, NOT yet on disk; plain `up` / `down` only when there is no
  writer (standalone construction, tests); `upDropped:<why>` with new why `queue` (full / writer dead). `cdbObs` keeps meaning "WRITTEN" and now
  lags the verdicts. New telemetry field `cdbDrop` (in `CURVEDB_TELE_KEYS`, all three return paths): observations lost since boot (queue full,
  writer dead, cap-refused AFTER queueing, write error, lost at stop; loop-side refusals after the cap freeze show only as the `upDropped:stopped` verdict). Curvedb drops are counted on the shadow, NOT in ces_events `evDrop`.
- Rule 2: queue drops log `curvedb_shadow: observation(s) DROPPED before reaching <path> (<why>)`, at most once per 30 s with the running count;
  writer-side append failures go through the shadow's `_fail` and name the file; the cap freeze logs once, lines already queued behind it are
  counted in `cdbDrop` without per-line log spam. `close_events` flushes curvedb items too (same queue).
- Other `/data` writes reachable from the selfdrived loop in `ces_pnw/`: none left. `rotate_at_park` is already on the writer; `mem_params.put_nonblocking`
  (CurveBrain, IcbmTarget, CESStatus) and the 5 Hz SteerEvent read are `/dev/shm`; `curvedb_live` / `curve_brain` / the boot snapshot are reads or
  construction-time. Not in `ces_pnw/`: other processes still append to ces_events (`append_to_ces_log`), as section 2 notes.
- Tests: new `ces_pnw/tests/test_curvedbasync2pnw.py` (10); `test_curvedb_read_boundary.py` lists it as an allowed importer.
- Verify on-car with loopdiag stage times on the first curve passes after install; `cdbDrop` must stay 0.

## steersatfloor2pnw (owner decision 2026-10-14; review fixes in the second commit)

- "Take Control" (steerSaturated) on the Lightning needs desired lateral accel > **2.2 m/s2** (was 1.0) WHILE ITS ANGLE LATERAL PATH IS LIVE.
  `STEER_SAT_ANGLE_FLOOR` in `pnw_vehicle.py`, capability `PnwVehicle.steer_sat_demand_floor` (Lightning only; Tesla stays 1.0). Alert text, the
  2.5 deg / 1 s timer and the 1.2 ratio are unchanged. No per-car JSON: the only lateral JSON, `lataccel_limits.json`, is the curvature clip schedule.
- Which mode is live is NOT in CarParams: the Ford interface always declares `steerControlType=angle`, and the carcontroller picks angle vs
  curvature from `FordAngleLateral` (+ `NoFordAngleSteering`) and can fall back to curvature mid-drive. selfdrived therefore polls the carcontroller's
  `FordLatStatus` mem-param every 25 frames: mode "angle" -> floor 2.2; any other / stale (> 5 s) / unreadable -> stock 1.0. Every mode change is
  cloudlogged (the boot log shows the mode the floor assumed).
- Escapes (the alert stays ON below the floor): wheel error >= 10 deg, or the suppressed condition lasting >= 2 s in one episode
  (`STEER_SAT_ESC_ANG_ERR_DEG`, `STEER_SAT_ESC_DURATION_S`). Ordinary suppressed onsets sat at 3.7-8.5 deg; outliers 12.3 / 13.8 / 20.8 / 39.5.
- Floor chosen on the RUNTIME quantity (modelV2.action.desiredCurvature * v^2, 100 Hz) replayed over Sunday 10-11's 11 rlog segments (21 stock
  episodes; **Saturday has no rlog**). Episodes kept (no escapes / with escapes): 1.0 -> 21 | 1.5 -> 14 / 15 | 2.0 -> 8 / 9 | **2.2 -> 5 / 8** |
  2.4 -> 4 / 7 | 2.5 -> 3 / 7. The 4 episodes with runtime peak >= 2.4 (2.41-3.39) all still alert at 2.2 (margin 0.2-0.3). One day / 21
  episodes is a small sample: 2.2 is a conservative first value. (The first cut of this change tuned on CES `slLatDem`, a different quantity; superseded.)
- Rule 2: a suppressed episode is logged, never silent, under the DISTINCT name `steerSaturatedSuppressed` (so alert counts cannot mix): a
  `phase:"start"` record and a `phase:"end"` record (`ev:"alertSuppressed"`, only for episodes with at least one genuinely suppressed frame) with `peakDemand`, `peakAngErr`, `durationS`, `escaped`, plus `supCount`, plus cloudlog warnings.
- Not changed: panda safety, steering commands, the lateral controller. Pre-existing and left alone: `log_take_control_alert` in `ces_pnw.py`
  ends in a bare `except Exception: pass` (and the CES stub's version is a no-op), so if CES failed to construct only the cloudlog line survives.
- Tests: `selfdrive/selfdrived/tests/test_steersatfloor_pnw.py` (9; real `SelfdriveD.__init__`, 100 Hz replay fixture `tests/data/steersat_rlog_runtime_2026-10-11.json`).
- Verify on-car: `steerSaturatedSuppressed` records on canyon curves; boot log line `lateral mode now 'angle'`; genuine over-demand still alerts.
