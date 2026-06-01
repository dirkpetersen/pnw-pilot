# MAPD2XNOR.md — Deploying the OSM speed-limit feature to a comma 3X (xnor-dev)

> This is the **deployment** companion to the design doc. It documents how
> `mapd2xnor` was actually deployed to the live device, every pitfall hit, and
> the exact recovery used when the UI failed to come up. Read this before
> deploying mapd2xnor anywhere.

**Branch:** `mapd2xnor` (openpilot-xnor worktree), off `c0d78143` (xnor prebuilt base).
**Device:** comma **3X** (`tizi`) @ `comma@192.168.13.154`, distro branch `xnor-dev`.
**What it adds:** OSM speed-limit **display** on the HUD + a red "reduced speed
limit" warning when the limit drops and you're >20% over. Display only — never
actuates the car. Bundles the 9.37 MB pfeiferj mapd binary, a cereal message
(`liveMapDataSP`), a new daemon pair (`mapd` + `mapd_manager`), and 16 params.

---

## Why this deploy is heavier than DM / fingerprint deploys

Unlike the relaxed-DM or F-150-fingerprint deploys (pure file overlays), mapd2xnor
**requires an on-device build**:

1. **Cereal schema change** — `custom.capnp` (`CustomReserved8` → `LiveMapDataSP`,
   id preserved) + `log.capnp` (`customReserved8 @115` → `liveMapDataSP @115`).
   The generated C++/python schema must be **regenerated with `scons`**; a file
   overlay alone does nothing.
2. **16 new param keys** in `common/params_keys.h` — `Params.checkKey` validates
   against keys compiled into `params_pyx.so`, so the keys are useless until
   `params_pyx.so` is rebuilt. Unregistered key → `UnknownKeyName` at runtime.

The xnor-dev 3X **can** build on-device: `/usr/local/venv/bin/scons` 4.10.0 +
Cython 3.1.4 + g++/clang, with a populated `.sconsign.dblite`. (This is the same
reason the DM param-key deploy worked here and not on the sunnypilot release-tizi
device — see DMON.md §14 vs §16.)

---

## The deploy procedure that worked

All paths on device unless noted. Run from the dev host (`~/gh/comma/openpilot-xnor`,
branch `mapd2xnor` checked out — files are read with `git show mapd2xnor:<path>`
so the *checked-out* branch doesn't matter, but be deliberate).

### 0. Pre-flight
- Confirm device is a 3X with build env: `ssh comma@… 'ls /usr/local/venv/bin/scons; ls /data/openpilot/.sconsign.dblite'`.
- Confirm the car is **offroad** before any restart (`cat /data/params/d/IsOnroad` → 0).
- Confirm which of the *modified-existing* files on the device are pristine vs.
  carry earlier deploys (see the params_keys.h pitfall below).

### 1. Back up every modified-existing file
```
ORG=/data/dirk/org/mapd2xnor
for f in cereal/custom.capnp cereal/log.capnp cereal/services.py common/params_keys.h \
         selfdrive/selfdrived/alerts_offroad.json selfdrive/ui/onroad/augmented_road_view.py \
         selfdrive/ui/ui_state.py system/hardware/hw.py system/manager/process_config.py; do
  mkdir -p "$ORG/$(dirname "$f")"; [ -f "$ORG/$f" ] || cp "/data/openpilot/$f" "$ORG/$f"
done
```

### 2. Push files
- New dirs: `sunnypilot/`, `sunnypilot/mapd/live_map_data/`, `sunnypilot/navd/`, `third_party/mapd_pfeiferj/`.
- New python + modified files: `git show mapd2xnor:<f> | ssh comma@… 'cat > /data/openpilot/<f>'`.
- **params_keys.h**: push the **MERGED** version, not the branch's raw file (see pitfall 1).
- **Binary** (`third_party/mapd_pfeiferj/mapd`, 9371832 bytes): extract to a temp file,
  `scp` it, `chmod +x`, verify `file` shows `ELF … aarch64` and size matches exactly.
- **Symlink**: `cd /data/openpilot/openpilot && ln -sfn ../sunnypilot sunnypilot`
  (lets `openpilot.sunnypilot.mapd…` imports resolve).

### 3. Build (the load-bearing step)
```
cd /data/openpilot && export PATH=/usr/local/venv/bin:$PATH
scons -u -j$(nproc) cereal/                 # regenerates liveMapDataSP
scons -u -j$(nproc) common/params_pyx.so    # registers the new keys
```
Verify before restart:
- `liveMapDataSP` constructs: `python -c "import cereal.messaging as m; m.new_message('liveMapDataSP')"`.
- Every new key resolves without `UnknownKeyName` (incl. `Offroad_OSMUpdateRequired`!).
- `mapd_manager`, `speed_limit.py`, `process_config` all import.

### 4. Persistence guards + caches
`DisableUpdates=1`; kill `updated`; remove `.overlay_init` + `.overlay_consistent`;
`touch` every changed source; clear `__pycache__`/`*.pyc` in all touched dirs.

### 5. Restart
`sudo systemctl restart comma.service` (the stack runs under tmux session `comma`
via `launch_chffrplus.sh`). On boot it runs a **fuller scons rebuild** of changed
files — expect 2–4 min and load spiking to ~9 before manager/UI come up.

### 6. Verify healthy
`mapd` + `mapd_manager` both `running=True`, `shouldRun-but-not: []`, no
`UnknownKeyName` in `/data/log/error.log`. First map download auto-arms (PNW:
Washington/Oregon/Idaho) from `https://map-data.pfeifer.dev/` into `/data/media/0/osm`
— needs network. Speed-limit sign needs a GPS fix (outdoors / driving).

---

## Pitfalls (all hit during the real deploy)

### Pitfall 1 — `params_keys.h` overlay clobbers other features' keys ⚠️
The device already had the **DM toggle keys** (`SensitiveDriverMonitoring`,
`AllowSoftwareUpdates`) from an earlier deploy. mapd2xnor's `params_keys.h` does
**not** contain those keys. A wholesale overlay would have **deleted** them →
`UnknownKeyName` → DM/UI crash.

**Fix:** never blindly overlay `params_keys.h` when another feature already added
keys to the device. **Merge**: start from the *device's current* `params_keys.h`
and insert only this feature's key block. Verify the merge is **purely additive**
(`diff device_file merged | grep '^<'` must be empty — nothing removed). This is a
general rule whenever multiple param-adding features are deployed to one device but
kept on separate branches.

### Pitfall 2 — `Offroad_OSMUpdateRequired` was never registered (branch bug) ⚠️⚠️
mapd2xnor added `Offroad_OSMUpdateRequired` to `alerts_offroad.json` but **forgot
to register it in `params_keys.h`**. `set_offroad_alert()` does `Params().remove()/put()`
on that key, and the UI's `offroad_alerts.py` does `Params().get()` on it — both
call `check_key` → `UnknownKeyName`. Result:
- `mapd_manager` crash-loops on startup, **and**
- **the UI itself crash-loops** (offroad alert widget) → device stuck on the comma
  logo, no UI.

**Fix:** registered it as `{"Offroad_OSMUpdateRequired", {CLEAR_ON_MANAGER_START, JSON}}`
like the other `Offroad_*` alerts (branch commit `2c3aa1e4`). **Lesson:** any key
referenced by `set_offroad_alert` / `alerts_offroad.json` MUST also be in
`params_keys.h`. Grep new code for every param string and confirm each is registered
before deploying.

### Pitfall 3 — interrupted `scp` of `params_keys.h` bricked the UI ⚠️⚠️⚠️
While pushing the *corrected* `params_keys.h`, the device dropped off the network
mid-transfer. The half-written / not-yet-applied file meant the device booted with
the **broken** `params_keys.h` (missing the OSM key) → UI crash-loop (Pitfall 2) →
stuck on comma logo. The crash-loop drove **load to ~19**, which made SSH itself
time out, compounding the problem.

**Fix / lesson:**
- Push critical build inputs to a **temp path, verify `md5sum` against the local
  file, then atomically `cp`/`mv` into place**. Never let an interrupted transfer
  leave `params_keys.h` (or any build input) partially written in place.
- When a crash-loop pins the CPU, SSH is slow/unavailable. **First action on
  reconnect: stop the stack** (`sudo systemctl stop comma.service; pkill -9 -f
  manager.py`) to calm the box, *then* fix the file. Don't try to fix while it
  thrashes.

### Pitfall 4 — the device auto-rebuilds on boot
`launch_chffrplus.sh` runs `scons` on boot. After pushing changed files it will
rebuild them (2–4 min, high load) before manager starts. This is normal — don't
mistake the build delay for a hang. But it also means a **bad build input (e.g.
truncated `params_keys.h`) blocks the whole boot**, not just the feature.

### Pitfall 5 — `pkill -f "system.updated.updated"` self-match
Running that inside an SSH `bash -c` matches the SSH session's own argv and kills
the session. Use a bracket regex: `pkill -f "[s]ystem.updated.updated"`.
(Same footgun documented in DMON.md §16.)

---

## Recovery recipe (if the UI is stuck on the comma logo after a mapd deploy)

SSH still works even when openpilot is down (it's independent of the stack):
```
# 1. calm the box — stop the crash-loop
sudo systemctl stop comma.service; pkill -9 -f manager.py

# 2. check the suspect file is intact + has the key
wc -l /data/openpilot/common/params_keys.h        # expect ~152 lines, ends with '};'
grep -c Offroad_OSMUpdateRequired /data/openpilot/common/params_keys.h   # must be 1

# 3. restore from backup or push a md5-verified good copy, then rebuild
cd /data/openpilot && PATH=/usr/local/venv/bin:$PATH scons -u -j$(nproc) common/params_pyx.so

# 4. prove the crashing call works
PYTHONPATH=/data/openpilot /usr/local/venv/bin/python -c \
  "from openpilot.common.params import Params; Params().get('Offroad_OSMUpdateRequired'); print('ok')"

# 5. restart
sudo systemctl start comma.service
```
Full pristine backups of every modified file are at `/data/dirk/org/mapd2xnor/`.

---

## Deploy status (as of this writing)

- ✅ All files + 9.37 MB binary on device; cereal + `params_pyx.so` rebuilt.
- ✅ `Offroad_OSMUpdateRequired` registered (branch `2c3aa1e4`); UI no longer crash-loops.
- ✅ `mapd` + `mapd_manager` both `running=True`; `shouldRun-but-not: []`; no `UnknownKeyName`.
- ✅ DM toggles, F-150 fingerprint, update-gate all preserved (merged `params_keys.h`).
- ⬜ On-road validation pending: GPS fix → speed-limit sign; drive onto a lower-limit
  road → red REDUCED SPEED LIMIT banner. First map pull needs network.

## Cross-feature notes
- This device also carries the relaxed-DM + update-gate work (DMON.md §16, branch
  `dmon2xnor-b`) and a pending F-150 fingerprint (branch `light2xnor`, applied to
  disk via `update-f150-fingerprint.sh`). All kept on **separate branches**; the
  only place they collide on-device is `params_keys.h`, which must be **merged**,
  never overlaid (Pitfall 1).
- The core-hotplug `set_core_affinity` fix (DMON.md §16, commit `55fbcd44` on
  `dmon2xnor-b`) is **not** on `mapd2xnor`. It's required on this 3X for DM to
  survive engage; if you ever build a single combined deploy branch, include it.

---

## Post-deploy fixes: why the speed-limit sign was blank (and how it was fixed)

After deploy the sign stayed blank ("-"). Two separate bugs, both now fixed on `mapd2xnor`:

### Fix 1 — wrong GPS source (commit `7ed0284c`)
The bridge (`live_map_data/base_map_data.py`, `osm_map_data.py`) subscribed to
**`gpsLocationExternal`**, which the comma **3X does not publish** — its internal qcom GPS
(`qcomgpsd`) publishes **`gpsLocation`** (with a valid fix). So `LastGPSPosition` was never
written → the mapd binary had no position → `MapSpeedLimit` empty → `liveMapDataSP.speedLimitValid`
always False → blank sign.

**Fix:** select the GPS stream at runtime via `common.gps.get_gps_location_service(params)` —
exactly how sunnypilot does it. It returns `gpsLocationExternal` only when `UbloxAvailable`,
else `gpsLocation`. The 3X has no ublox, so it now reads `gpsLocation`. `base_map_data`
subscribes to the resolved service and reads `.hasFix`; `osm_map_data` reads
`latitude`/`longitude`/`bearingDeg` from it and writes `LastGPSPosition`.

Verified: `LastGPSPosition` now populated; binary loaded the local tile and resolved e.g.
**20 mph on 7th Avenue Northwest** (Seattle, 47.67/-122.36).

### Fix 2 — mapd processes never respawned after exit (commit below)
The repeated "download stalls" were **not** a download-logic problem — the **mapd binary
simply wasn't running**. `NativeProcess("mapd", …)` / `PythonProcess("mapd_manager", …)`
were created without `restart_if_crash=True`, so when the binary exited (it does a single
non-resuming download pass and can finish/die mid-pass, or get killed during maintenance),
`manager.ensure_running` did **not** relaunch it (it only restarts crashed processes when
`restart_if_crash` is set — same default as sunnypilot). The daemon stayed dead until a full
stack restart, and the sign went blank.

**Fix:**
- `system/manager/process.py` — `NativeProcess.__init__` now accepts `restart_if_crash`
  (previously only `PythonProcess` did). The base-class attribute and the
  `ensure_running` check (`if p.restart_if_crash and not p.proc.is_alive(): p.restart()`)
  already existed, so this just plumbs the flag through to native processes.
- `system/manager/process_config.py` — both `mapd` and `mapd_manager` now pass
  `restart_if_crash=True`, so manager relaunches them on exit and the download daemon
  self-heals across crashes/reboots.

### mapd download behavior (worth knowing)
- The binary downloads a state's whole bounding box in **one sequential pass** over a
  lat/lon grid (`offline/<lat>/<lon>/<tile>`), **no resume** (v1.12.0). If interrupted it
  must be re-armed (`OSMDownloadLocations`) and restarts the pass from the beginning;
  already-present tiles are skipped/re-fetched. WA+OR+ID is ~hundreds of MB, so let it run
  uninterrupted on Wi-Fi. `update_osm_db()` re-arms (throttled) until `OSMDownloadProgress`
  shows `downloaded_files >= total_files` (commit `65b96fafee`).
- Speed limit only shows where OSM has a `maxspeed` tag for the road **and** there's a GPS
  fix; residential streets often have a limit, but some roads are untagged → no sign there.
