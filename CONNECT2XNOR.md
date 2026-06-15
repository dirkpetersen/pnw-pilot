# CONNECT2XNOR — Proactive full-data upload to the self-hosted backend

Branch: `connect2xnor` (base `3e1a2e8044`, off the `wifi`/tethering fixes).

## Problem

Stock openpilot has **two implicit upload tiers**:

| Tier | Files | How it uploads today |
|------|-------|----------------------|
| SMALL | `qlog.zst`, `qcamera.ts` | Proactively, by `system/loggerd/uploader.py` |
| LARGE | `rlog.zst`, `fcamera.hevc`, `ecamera.hevc` | **Only** when the backend sends an Athena `uploadFilesToUrls` request via `system/athena/athenad.py` |

The user's self-hosted AWS backend issues **no Athena upload requests**, so large files never
leave the device. `system/loggerd/deleter.py` then prunes the oldest segments at ~90 % disk →
**drive video + rlogs are lost before they are ever uploaded.**

## Solution overview

Four behavior-neutral-when-off changes, all gated on **`deviceState.networkType == NetworkType.wifi`**:

1. **Two-pass uploader** — a NEW pass 2 proactively uploads the large files, with no Athena round-trip, ONLY on real external WiFi.
2. **Deleter preservation** — segments whose large files are not yet uploaded are kept preferentially; last-resort deletion is explicit and logged.
3. **Firehose indicator (repurposed)** — "Firehose Mode" status now means *pass-2 video upload is actively in flight*.
4. **Connect indicator = WiFi-only** — the sidebar CONNECT metric is ONLINE only on real external WiFi.

### Why `networkType == wifi` is THE gate

`system/hardware/tici/hardware.py:get_network_type()` returns `NetworkType.wifi` **only** when
NetworkManager's `PrimaryConnection` (the default route) is a WiFi *client*. The comma's own
hotspot connection is `never-default`, so while the device is hotspotting the PrimaryConnection
stays LTE and `networkType` reports `cell`. Therefore:

> `networkType == wifi` ⇒ real external WiFi — **not** the hotspot, **not** LTE.

This single check protects against burning cellular data and against uploading over the device's
own AP. It is used identically in the uploader gate, the firehose indicator, and the connect
indicator.

---

## 1. Two-pass upload — `system/loggerd/uploader.py`

New module constants:

```python
FIREHOSE_FILES       = {"rlog", "rlog.zst", "fcamera.hevc", "ecamera.hevc"}
PASS2_NETWORK_TYPES  = {int(NetworkType.wifi)}      # strict WiFi-only gate
FIREHOSE_ACTIVE_PARAM = "FirehoseActive"            # UI indicator param
def pass2_allowed(network_type: int) -> bool: ...   # network_type in PASS2_NETWORK_TYPES
```

- `list_upload_files(metered, pass2=False)` — gained a `pass2` flag. Pass 1 (default) **skips**
  any name in `FIREHOSE_FILES`; pass 2 yields **only** those names. The two passes partition the
  segment cleanly and never interleave.
- `next_pass2_file_to_upload(metered)` — returns the **oldest** un-uploaded large file
  (`listdir_by_creation` is oldest-first), so data leaves the device before the deleter reaches it.
- `step(network_type, metered, pass2=False)` — when `pass2=True`, sets `FirehoseActive=1` for the
  duration of the transfer (`try/finally`, so it always clears) and uploads via the **existing**
  presigned-PUT path (`do_upload` → `GET v1.4/{dongle}/upload_url/?path=…` → `requests.put`).
- `main()` loop — runs pass 1 first (unchanged). **Only when pass 1 returns `None`** (nothing small
  left to do) **and** `pass2_allowed(networkType)` does it run pass 2. So small files always have
  priority and large files never burn cellular.
- `FirehoseActive` is cleared on `Uploader.__init__` so a crash can't leave a stale "uploading" flag.

`dcamera.hevc` (driver camera) is intentionally **not** in `FIREHOSE_FILES`: it falls into pass 1's
set but is not in `immediate_priority`, so it behaves exactly as in stock (not proactively
uploaded). We do not proactively push the driver-facing camera.

**Deviation from the brief:** the brief suggested possibly extending priority logic; the cleanest
real integration was a `pass2` flag threaded through `list_upload_files`/`step`, because the stock
`next_file_to_upload` is hard-coded to only return `immediate_folders`/`immediate_priority` files —
the large files were simply never selectable. The two-pass split reuses all existing upload, xattr,
compression and metered logic verbatim.

## 2. Deleter preservation — `system/loggerd/deleter.py`

New helper:

```python
def has_unuploaded_firehose(d: str) -> bool:
    # True if segment d still has an rlog/fcamera/ecamera WITHOUT user.upload=1
```

The delete-ordering sort key gained a third element:

```python
sorted(dirs, key=lambda d: (d in DELETE_LAST, d in preserved_dirs, d in unuploaded_dirs))
```

Ascending → first element is deleted first. So the priority of *what survives* is:

1. `boot` / `crash` (DELETE_LAST) — survive longest
2. `user.preserve` segments
3. **segments with un-uploaded large files (NEW)**
4. everything else (fully-uploaded ordinary segments) — deleted first

**Last-resort safety:** if *every* remaining segment is un-uploaded (truly out of space), the
oldest is still deleted so logging never stalls, and we emit
`cloudlog.error("connect2xnor: deleting UN-UPLOADED segment to free space: …")`. We never block
freeing space — priority is safety/stability > data retention.

A flaky `getxattr`/`listdir` is treated as "already handled" (returns `False`), so a bad stat can
never wedge the deleter.

## 3. Firehose indicator (repurposed) — `selfdrive/ui/mici/layouts/settings/firehose.py`

`FirehoseLayoutBase._get_status()` now reflects the real pass-2 upload state (the non-mici
`selfdrive/ui/layouts/settings/firehose.py` inherits this base, so both UIs are covered):

| Condition | Status text | Color |
|-----------|-------------|-------|
| `FirehoseActive` param is set | `UPLOADING` | green |
| else, on WiFi | `READY` | green |
| else | `INACTIVE: connect to Wi-Fi` | red |

`FirehoseActive` is the param the uploader sets/clears around each pass-2 transfer. No polling
thread or new socket — it reads the existing `Params` instance already on the layout.

New param: `common/params_keys.h` → `{"FirehoseActive", {CLEAR_ON_MANAGER_START, BOOL, "0"}}`
(additive, transient; cleared every manager start).

## 4. Connect indicator = WiFi-only — `selfdrive/ui/layouts/sidebar.py`

`_update_connection_status()` now requires `deviceState.networkType == NetworkType.wifi` in
addition to the existing recent-Athena-ping check. On the hotspot or LTE the CONNECT metric reads
**OFFLINE**; it is **ONLINE** only on a genuine external WiFi client connection with a fresh backend
ping. (`NetworkType` was already imported in this file.)

---

## Tests — `system/loggerd/tests/test_connect2xnor.py`

- `TestPass2Gate` — `pass2_allowed()` is True only for `wifi`; False for `none`, all `cellNG`,
  and `ethernet`.
- `TestPass2Selection` — pass 1 excludes firehose files; pass 2 yields only firehose files;
  already-uploaded large files are skipped; empty pass 2 returns `None`.
- `TestDeleterPreserveUnuploaded` — `has_unuploaded_firehose()` correctness; a fully-uploaded
  *newer* segment is deleted before an un-uploaded *older* one; last-resort path deletes the oldest
  un-uploaded segment when nothing else is freeable.

Run (in a built fork venv, needs `cereal`/`xattr` compiled):

```bash
source .venv/bin/activate
pytest system/loggerd/tests/test_connect2xnor.py -v
```

> NOTE: in this isolated worktree there is no built `.venv` (no `cereal`/`capnp`/`xattr`), so the
> suite could not be executed here. The pure gate/partition logic was verified standalone and every
> changed/added file passes `python -m py_compile` and `ruff check`. Run the suite in a real fork
> checkout before deploy.

---

## Deploy (file overlay to `/data/openpilot`)

This fork deploys as a **file overlay**, not a git checkout. Copy the changed files in place, rebuild
`params_pyx` (params_keys.h changed), clear pyc, restart, then set the persistence guards.

Changed files to overlay:

```
common/params_keys.h
system/loggerd/uploader.py
system/loggerd/deleter.py
selfdrive/ui/layouts/sidebar.py
selfdrive/ui/mici/layouts/settings/firehose.py
system/loggerd/tests/test_connect2xnor.py   # optional, tests only
```

On-device (device venv is `/usr/local/venv`, no local `.venv`):

```bash
source /usr/local/venv/bin/activate
export PYTHONPATH=/data/openpilot:/data/openpilot/opendbc_repo
cd /data/openpilot
# params_keys.h changed -> rebuild the params extension
PATH=/usr/local/venv/bin:$PATH scons -u -j$(nproc) common/params_pyx.so
find . -name '*.pyc' -delete
# restart
sudo systemctl restart comma   # or: tmux kill-server; the manager relaunches
```

### Persistence guards (REQUIRED to survive reboot)

```bash
sudo rm -rf /data/safe_staging/finalized
touch -d "2020-01-01" /data/openpilot/.overlay_init
touch /data/openpilot/prebuilt
# also: DisableUpdates=1 (param + UI "Allow auto updates" = OFF)
```

### On-device verification

1. Connect the device to **real external WiFi** (not its own hotspot). Sidebar CONNECT should read
   ONLINE (green). On LTE/hotspot it should read OFFLINE.
2. Settings → Firehose Mode should read **READY** on WiFi while idle, **UPLOADING** while a large
   file transfers, and **INACTIVE: connect to Wi-Fi** otherwise.
3. `cat /data/params/d/FirehoseActive` flips to `1` during a pass-2 transfer, `0` otherwise.
4. After a drive on WiFi, confirm `rlog.zst`/`fcamera.hevc`/`ecamera.hevc` get
   `getfattr -n user.upload <file>` == `1` and appear in S3 under
   `drives/{dongle_id}/{segment}/…`.
5. On LTE only, confirm large files do **not** upload (`user.upload` stays unset) and
   `FirehoseActive` stays `0`.
6. Fill disk near threshold; confirm `deleter` deletes fully-uploaded segments first and logs the
   `connect2xnor: deleting UN-UPLOADED segment` error only as a last resort.

---

## BACKEND SUGGESTIONS — `/home/dp/gh/comma/comma-connect`  (read-only review)

**Key finding: no new endpoint is required.** The device's own background uploader already does a
proactive, Athena-free presigned-PUT, and the self-hosted backend already serves the matching
endpoint. The pass-2 path added here reuses that exact mechanism.

### Current upload mechanism (verified)

- The actual upload backend is **not** the React SPA in `comma-connect/` — it is an **AWS Lambda**
  behind API Gateway (source at `/tmp/comma-uploader-api/handler.py`, documented in
  `comma-connect/CLAUDE.md` lines ~100-205). `API_HOST` on the device already points at it.
- Device flow (in `system/loggerd/uploader.py:do_upload`): `GET v1.4/{dongle_id}/upload_url/?path={key}`
  with header `Authorization: JWT <token>` → `{"url": <presigned S3 PUT>, "headers": {}}` →
  `requests.put(url, data=stream, headers=headers)`. **No Athena, no websocket.** This is exactly
  the path pass-2 uses.
- Lambda `handle_upload_url` presigns `put_object` for
  `Bucket=comma-connect, Key=drives/{dongle_id}/{path}, ExpiresIn=3600`. Auth = JWT decoded
  **without signature verification**, requires `payload['identity'] == dongle_id` (+ optional
  `ALLOWED_DONGLES` allowlist).
- The **Athena round-trip** path is only the web-UI "user clicks upload" flow in
  `comma-connect/src/actions/files.js` (`uploadFilesToUrls` / `uploadFileToUrl` over the Athena
  JSON-RPC relay). That is the path we are bypassing — and the device already bypasses it by calling
  the Lambda directly.

### S3 key layout (verified)

```
s3://comma-connect/drives/{dongle_id}/{route_datetime}--{segment_num}/{filename}
e.g. drives/2fd850c60cc5bfef/2024-01-15--12-34-56--3/rlog.zst
```

`path` sent by the device is exactly `{segment}/{filename}` (`uploader.py` `key = os.path.join(logdir, name)`).

### Recommendations (do NOT edit the backend — proposals only)

1. **Nothing required for pass-2 to work.** `GET /v1.4/{dongle_id}/upload_url/?path=…` already
   returns a presigned PUT URL for any path, including `…/fcamera.hevc` and `…/rlog.zst`. Pass-2
   uses it unchanged.
2. **Harden the existing Lambda** (security, independent of this feature):
   - Verify the JWT **signature** instead of blind-decoding (`handler.py` line ~11-17, 32).
   - Sanitize `file_path` before interpolating into the S3 `Key` (reject `..` and leading `/`),
     since it is concatenated directly (`handler.py` line ~43).
   - Set/echo a `ContentType` on the presign for video (`video/mp2t` for `.ts`, `video/x-h265` /
     `application/octet-stream` for `.hevc`) so stored objects carry correct metadata.
3. **Optional ergonomic endpoint** (only if a non-openpilot uploader is ever wanted), a thin v2
   alias reusing the same presign logic:

   ```
   POST /v2/{dongle_id}/upload          Authorization: JWT <jwt>
   req:  {"path":"2024-01-15--12-34-56--3/fcamera.hevc","content_type":"video/x-h265","expires_in":3600}
   resp: {"url":"<presigned PUT>","method":"PUT","headers":{},
          "key":"drives/{dongle_id}/2024-01-15--12-34-56--3/fcamera.hevc","expires_in":3600}
   ```

   Same auth (`identity == dongle_id`, optional allowlist), same key scheme. Near-copy of
   `handle_upload_url`. Not needed for this fork — the `v1.4` GET path is sufficient.
4. **Backend visibility (optional):** a `GET /v1.4/{dongle_id}/segments?uploaded=false` that lists
   keys present in S3 would let a future device reconcile, but is not needed: the device already
   tracks upload state locally via the `user.upload` xattr.

**Bottom line:** the device → backend handoff for proactive video upload needs **zero backend
changes** — pass-2 simply calls the presigned-PUT endpoint the backend already exposes, gated to
WiFi. The only backend work worth doing is the security hardening in (2).
