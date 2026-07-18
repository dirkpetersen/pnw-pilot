# DEFER_HD_UPLOAD — "Defer HD Video Upload" toggle

**Branch:** `4devpnw` (in `pnw/pnw-pilot`; code tagged `connect2pnw`) · **Status:** implemented,
Gemini-reviewed & fixed, **DEPLOYED** on the 3X (`4devpnw @ 4816a48`, param seeded to `0`).
Builds directly on the two-pass uploader — see `CONNECT2XNOR.md` (pass-1 = small files,
pass-2 = `FIREHOSE_FILES` rlog/fcamera/ecamera) and `UPLOAD2XNOR.md`.

## Requirement (driver, 2026-07-06)

> A normally-**OFF** toggle. When **ON**, prevent the large HD video files from uploading — so the
> uploader "doesn't suck my entire connection dry" on a link that *qualifies* as unmetered WiFi but is
> actually precious (RV-park WiFi, a borrowed hotspot) — while **still** uploading log data to the
> self-hosted connect server. (This fork uploads far more than stock, which never uploads HD at all.)

## Behavior

| Toggle | qlog / rlog / qcamera (logs + low-res) | fcamera / ecamera / dcamera (HD video) |
|--------|----------------------------------------|----------------------------------------|
| **OFF** (default) | upload | upload |
| **ON** | upload | **held** (skipped, not marked) |

- **Held = skipped-not-marked.** Deferred HD files are *never* xattr-stamped `user.upload`, so the
  moment the toggle goes back OFF they upload normally on the next pass — *provided the disk cleaner
  (`deleter.py`) hasn't reclaimed them first* at ~90 % full. Deferring on a precious link trades HD
  footage retention for bandwidth; it does not lose logs.
- Default OFF ⇒ **behavior-neutral** vs. the existing two-pass uploader.

## Implementation (`system/loggerd/uploader.py`)

- `HD_VIDEO_FILES = {"fcamera.hevc", "ecamera.hevc", "dcamera.hevc"}` — the deferrable subset.
- `DEFER_HD_PARAM = "DeferHDVideoUpload"`; `self._defer_hd` snapshot refreshed **once per listing**
  in `list_upload_files()` (`get_bool`, `try/except → False`), not per file.
- The skip sits **above the pass-1/pass-2 split**:
  ```python
  if self._defer_hd and name in HD_VIDEO_FILES:
      continue
  ```
  **Why above the split (Gemini fix):** `dcamera.hevc` is **not** in `FIREHOSE_FILES`, so it flows in
  *pass 1*. Putting the defer check only inside the pass-2 branch would let the driver-camera HD slip
  through while ON. Lifting it above catches HD in *either* pass.

## Param + UI

- `common/params_keys.h:109` — `{"DeferHDVideoUpload", {PERSISTENT, BOOL, "0"}}`. **A new key ⇒
  on-device `params_pyx.so` rebuild** (done at deploy; without it → `UnknownKeyName` → UI crash-loop).
- `selfdrive/ui/layouts/settings/toggles.py` — toggle **"Defer HD Video Upload"** (icon
  `network.png`, default `False`) + a `DESCRIPTIONS` entry explaining held-then-resumes-when-off.

## Deploy / rollback

Pure Python + one param. Landed via the standard `4devpnw` GitHub deploy (`git reset --hard` +
`params_pyx.so` rebuild + pyc clear + manager restart). Rollback = flip the toggle OFF (HD resumes) or
revert the two files. Verify: `cat /data/params/d/DeferHDVideoUpload` (0 = off), and with it ON the
uploader logs `upload_success` for qlog/rlog/qcamera but no `fcamera/ecamera/dcamera` keys.

## Files
- `system/loggerd/uploader.py` — `HD_VIDEO_FILES`, `DEFER_HD_PARAM`, `_defer_hd`, above-split skip
- `common/params_keys.h` — `DeferHDVideoUpload` key
- `selfdrive/ui/layouts/settings/toggles.py` — toggle + description
