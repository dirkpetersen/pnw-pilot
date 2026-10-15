# CHANGELOG — 2026-10-14 (Wednesday)

All times PT. Branch work, not yet shipped.

## 1. `deleterhdfirst2pnw` (branch, not yet shipped)

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

## cesasyncwrite2pnw: the selfdrived loop never does ces_events file I/O

Cause (drives/2026-10-11/system-lagging): the two `selfdrivedLagging` soft-disables (TAKE CONTROL IMMEDIATELY at 56 and 64 mph) were each a
single ~123 ms stall of the selfdrived loop; loopdiag2pnw put 96-99% of it in the synchronous append to `/data/pnw/ces_events.jsonl`
(`CESController._append_event`: the ces stage 112/116 ms, the write 107.5/114.8 ms) on a ~90% full `/data` with loggerd writing hard.

- `_append_event` now only `json.dumps` + `put_nowait()` on a bounded `queue.Queue` (2000). ONE daemon thread (`ces_events_writer`) does the
  size check, `rotate_event_log`, archive hand-off and the append, unchanged (same file names, 20 MB rotation, 8 generations, open-per-append,
  no fsync). The park-edge `rotate_at_park` is enqueued to the same thread (it was the only other disk call on the loop's path).
- Rule 2: queue full -> record dropped, counted (`_ev_drops`), `cloudlog.error` at most once per 30 s with the running count; never blocks.
  Writer death -> logged by the thread, detected by `is_alive()` in the next `_append_event`, restarted (max 5, then a loud once-per-30 s line).
  The writer warns when one append or rotate takes > 50 ms, so disk stalls stay visible. `flush_events()` runs at atexit (best effort; a SIGKILL or
  an unhandled SIGTERM loses the queue, <= ~2 records/s). The loopdiag `ces_event_write*` fields now time the enqueue (~0 ms).
- Other `/data` I/O reachable from the loop in `ces_pnw/`: `curvedb_shadow._write` (appends one observation line to `curvedb_obs.jsonl`
  per curve pass, with a `getsize` first) is STILL synchronous: its caller's verdict depends on the write result, so it was not moved; it is
  per-observation, not per-tick. JSON reads (`curvedb_live`, `curve_brain`, `curvedb_shadow` load) are load-time/reload, not writes. The
  `SteerEvent` read is `/dev/shm`. Writers in other processes (`accdrop_pnw`/`pscmlim_pnw`/`epsref_pnw` via `append_to_ces_log`, the deleter) are
  threaded or not in selfdrived and unchanged; they still append to the same file.
- The stock selfdrived lagging check is untouched.
- Tests: new `ces_pnw/tests/test_cesasyncwrite2pnw.py` (+15); `test_loopdiag_pnw` slow-write test now asserts the stall does not reach the loop.
