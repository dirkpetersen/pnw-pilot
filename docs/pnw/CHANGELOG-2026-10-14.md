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
  `deleterLoss` records (and the offroad alert) now carry `"kind"`: `"HD sacrificed"` (HD video stripped, logs kept) or `"LOG LOST"` (whole
  segment). Emitted for every removed file the uploader would have sent (fcamera, ecamera unless SkipWideCameraUpload, rlog); dcamera is never
  uploaded by design and is removed without a loss event.
- Uploader: no change needed. `list_upload_files` walks `os.listdir`, so a segment with logs and no video just yields its logs.
- Tests: `test_deleter.py` +5 (HD before any un-uploaded log, rlog survives while HD exists anywhere, uploaded segments go first, `LOG LOST`
  only when no HD is left, uploaded HD stripped without a loss event); `test_uploader.py` +1 (segment without video lists cleanly).
- Mutation check: HD pass disabled (4 fail), un-uploaded sorted before uploaded (2), HD loss event suppressed (4).
- Not done: qlog/qcamera.ts are still not counted as lost when a whole segment goes (they were not before either).
