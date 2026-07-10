---
name: pnw-pilot-deploy
description: >-
  How to deploy code to the PNW comma 3X (Tesla Raven / Ford Lightning) and verify it — the auto-update
  promote channel (primary path), the manual git-deploy loop (urgent path), the build-on-boot model,
  LFS/submodule handling, persistence guards, the IsOnroad/tap-reset safety gates, and the recurring
  verification gotchas. Use whenever pushing code to the car at ~/gh/comma/pnw/pnw-pilot, debugging a
  deploy, or deciding auto-update vs manual.
---

# PNW on-device deploy / update

Distribution: `~/gh/comma/pnw/pnw-pilot`. The car runs `/data/openpilot` as a **git checkout**
(origin = `dirkpetersen/pnw-pilot`), NOT the classic overlay. Device = one comma 3X moved between the
Tesla Model S Raven and the Ford F-150 Lightning.

## ⚡ CHANNEL MAP (2026-07-10 — auto-update is ON and e2e-validated)

| Branch | Role |
|---|---|
| **`3devpnw`** | **The car's channel.** Device checked out here; `UpdaterTargetBranch=3devpnw`, `DisableUpdates=0`. Ongoing dev (incl. Tesla opendbc/panda work) lands here. |
| **`4devpnw`** | **FROZEN known-good fallback** (`261cc6ad90`, the fully-validated 2026-07-10 state). Reinstall/reset target if opendbc/panda experiments break the car. Do not advance casually. |
| **`3testpnw`** | **The FRIENDS' install channel — NEVER experiment on it** (mistake made + reverted 2026-07-09). Promote only truly validated states. |
| `3pnw` / `pnwprod` | Release; untouched by day-to-day work. |

## Deploy path 1 — AUTO-UPDATE (THE deploy path — including test deploys)

**Driver directive (2026-07-10): ALL deploys — including "just testing" new code on the device — go
through GitHub (push to `3devpnw`) followed by a reboot.** Do not scp/patch files onto the device to
try code out; the git checkout must stay == origin/3devpnw so every on-car state is reproducible and
the updater never fights manual edits. Path 2 below is for RECOVERY, not for shipping.

1. Commit on `3devpnw`, **Gemini-review** (gemini skill, always `gemini-pro-latest`; pipe diffs through
   `sed 's/@/[at]/g'` — the Gemini CLI treats `@token` in prompts as FILE ATTACHMENTS: raw diffs 400 with
   "Unable to process input image" and it hallucinates from attached binaries), push.
2. The updater checks ~every **1.5 h**. Immediate: `pkill -HUP -f system.updated.updated`
   (SIGHUP = user-requested fetch; SIGUSR1 = check only).
3. Watch: `UpdaterState` → `checking...` → `finalizing update...` → `idle`.
4. **REBOOT-TIMING GOTCHA (cost a wasted reboot 2026-07-10):** do NOT reboot the moment
   `git -C /data/safe_staging/finalized rev-parse` shows the new SHA — `.overlay_consistent` is written
   LAST in finalize; a reboot before it lands makes launch SKIP the staging (updater self-heals and
   re-finalizes after boot, but the update didn't install). Reboot only when **`UpdateAvailable=1`
   AND `/data/safe_staging/finalized/.overlay_consistent` exists**.
5. Reboot (or wait for the driver's next ignition-off) → swap runs → **build-on-boot** makes everything
   consistent (params_pyx for new keys, camerad C++, even the model compile — no manual rebuild steps
   on this path). Verify (section below).
- Validated through the updater 2026-07-09/10: LFS models arrive as REAL blobs (not pointers), tinygrad
  submodule content carried, param keys rebuilt, branch switches honored. Still unverified once: an
  update whose commit CHANGES a model LFS pointer — babysit the first one.

## Deploy path 2 — MANUAL (urgent fixes; also the rollback tool)

1. **Gate: only when `IsOnroad=0`** (`cat /data/params/d/IsOnroad`). It LAGS (~15 s after parking; stays
   `1` while charging). Never restart onroad — that restarts the control stack → disengage.
2. `cd /data/openpilot && git fetch --no-tags origin 3devpnw && GIT_LFS_SKIP_SMUDGE=1 git reset --hard <sha>
   && git lfs pull` — **always skip-smudge + separate lfs pull**: smudge-during-checkout of the ~61 MB
   model OOM'd git on the 3X (`fatal: Out of memory, realloc failed`, 2026-07-08) and left a half-reset
   tree + stale submodule pin. `.lfsconfig` fetchexcludes the 1.7 GB `big_driving_supercombo.onnx`
   (USB-GPU-only; `git lfs pull --include big_driving_supercombo.onnx` if ever needed).
3. **If the delta bumps a submodule pin** (opendbc_repo/panda/tinygrad_repo/msgq_repo):
   `git submodule update --init <path>` — `reset --hard` does NOT update submodule content.
   (Host-side gotcha: `git add <submodule-path>` records the LOCAL worktree's checkout, silently
   clobbering a `git update-index --cacheinfo` pin — check out the intended commit in the submodule
   dir first.)
4. `find /data/openpilot -name '*.pyc' -delete`. If `params_keys.h` changed and you can't wait for
   build-on-boot: `PATH=/usr/local/venv/bin:$PATH scons -j4 common/params_pyx.so` (else the restart's
   boot-build covers it). New key without rebuild → `UnknownKeyName` → UI crash-loop.
5. **Offline self-test in the venv** before restart: `Params().get_bool("<new key>")` + import the changed
   modules. venv is `/usr/local/venv` (`source /usr/local/venv/bin/activate;
   PYTHONPATH=/data/openpilot:/data/openpilot/opendbc_repo python3 ...`) — forgetting it =
   `No module named 'zmq'`.
6. Guards + restart: `sudo rm -rf /data/safe_staging/finalized; touch /tmp/booted;
   tmux kill-session -t comma; sudo systemctl restart comma`. `touch /tmp/booted` BEFORE the restart or
   comma.sh tap-resets the launch (recoverable: touch + restart; a real `sudo reboot` resets touch_count
   so cold reboots are safe). Note: with auto-update ON, removing `finalized` just discards staged work —
   the updater re-stages on its next cycle.
7. Note the device's checked-out branch is **`3devpnw`** — a manual reset to a different branch will be
   "corrected" by the next auto-update unless `UpdaterTargetBranch` is changed too.

## Companion repos — pnw-opendbc / pnw-panda (real submodules, `master-pnw`)

`opendbc_repo` and `panda` are **real git submodules on ALL channel branches** (`3devpnw`, `4devpnw`,
`3testpnw`, `3pnw` — wiring aligned 2026-07-10), pinned by SHA. `.gitmodules` points at
`../pnw-opendbc.git` / `../pnw-panda.git` with `branch = master-pnw`; the relative URLs resolve
against the superproject origin → `github.com/dirkpetersen/pnw-{opendbc,panda}`. (The old
"work branches vendor opendbc/panda inline" claim is obsolete — don't act on it.)

- **The work branch in each companion repo is `master-pnw`** — Tesla-era opendbc/panda modifications
  land there. Local clones: `~/gh/comma/pnw/pnw-opendbc` and `~/gh/comma/pnw/pnw-panda` (checked out
  on `master-pnw`). `master-xnor` = xnor upstream mirror; each fork also has a `4devpnw` branch
  snapshotting the frozen-fallback pins. The pin may deliberately LAG the `master-pnw` tip (frozen
  validated state) — a pin behind tip is not an error.
- **Shipping an opendbc/panda change:**
  1. Commit on `master-pnw` in the companion repo and **`git push origin master-pnw` FIRST**. A pin
     not reachable from the pushed branch makes the device's `git submodule update --init` fail
     mid-update.
  2. Bump the pin in pnw-pilot: `cd opendbc_repo && git fetch origin && git checkout <sha> && cd ..
     && git add opendbc_repo` (checking out the intended commit first avoids the add-clobbers-pin
     gotcha above). Commit on `3devpnw`, Gemini-review, push.
- The updater runs `git submodule sync` + `update --init --recursive` + `foreach reset --hard` on
  every fetch (`system/updated/updated.py:397`), so URL changes AND pin bumps flow through
  auto-update with no manual step.
- **A panda pin bump = a panda FLASH at next start** (pandad reflashes when the FW hash differs).
  Babysit the first one per era; a bad flash on the Raven is the matched-set/no-panda failure mode
  (memory `raven-matched-set-no-panda`). Keep the opendbc and panda pins a **MATCHED SET** — never
  advance one past compatibility with the other (`CAN_PACKET_VERSION_HASH` discipline).

## opendbc/panda pin-bump discipline (born 2026-07-10 — the MG_ZS→MOCK dashcam incident)

A pin bump ships EVERY commit between the pins, not just yours. The audit rule that failed and its
replacement:

- **"Different brand = inert for our cars" is FALSE.** Brand code (carstate/interface) only runs for
  the matched car, but several opendbc registries are GLOBAL — every platform in them participates in
  every car's behavior: **`FW_VERSIONS`** (fingerprint matching — all platforms are candidates for
  every car), **`opendbc/safety/modes/*`** (compiled into the panda firmware → hash change → reflash),
  **`torque_data/*.toml`**, shared DBC files. Audit ride-along commits against these, not just
  against "does it touch tesla/ford dirs".
- The incident: xnor's `CAR.MG_ZS: {}` — an EMPTY placeholder in `FW_VERSIONS` — had zero ECUs to
  invalidate it, so it **exact-matched every car on Earth**; every fingerprint became ambiguous
  (`{MG_ZS, real car}` → no unique match → MOCK/dashcam). Guard added in `2236bcd5` (empty FW record
  never matches), but the audit rule stands.
- **MANDATORY pre-reboot smoke test for ANY pin bump that touches opendbc** — run against the STAGED
  tree, takes 5 s, offline, catches every matcher/DB regression for our fleet:
  ```bash
  ssh comma@<ip> 'source /usr/local/venv/bin/activate; F=/data/safe_staging/finalized;
  PYTHONPATH=$F:$F/opendbc_repo python3 - <<EOF
  from opendbc.car.structs import CarParams
  from opendbc.car.fw_versions import match_fw_to_car
  fw = CarParams.CarFw(ecu=CarParams.Ecu.eps, address=0x730, bus=0,
                       fwVersion=b"SX_0.0.0 (99),SR013.7", brand="tesla")  # the Raven EPS answer
  exact, m = match_fw_to_car([fw], "00000000000000000")
  assert m == {"TESLA_MODEL_S_HW3"} or str(m).find("TESLA_MODEL_S_HW3") >= 0 and len(m) == 1, m
  print("fingerprint smoke test OK:", m)
  EOF'
  ```
- **Raven fingerprint facts** (so healthy output is recognizable): it matches on a SINGLE ECU —
  eps `0x730`, `b'SX_0.0.0 (99),SR013.7'` — so `fw: 1` is NORMAL, `carVin` all-zeros is NORMAL,
  `fingerprintSource: fw` is the good path. `source: can` on a MOCK means FW matching fell through.
- **The FW cache hides matcher regressions for WEEKS.** Every normal drive fingerprints from
  `CarParamsCache`, so "it's been detecting fine" is NOT evidence the matcher works — the full query
  only runs when the cache is lost (which is exactly when you least want a surprise). Hence the
  offline smoke test above, which exercises the matcher directly.
- **ECU-wake quirk:** a cold Raven's ECUs may not answer the first query round (it wakes them; the
  second round answers — `fingerprint.sh` docs). A failed fingerprint costs nothing since the
  never-persist-MOCK fix — recovery is simply an ignition cycle with the car awake.
- **Forensics triage for "car not recognized"** (this cracked the incident in minutes): compare the
  three CarParams snapshots — `CarParams` (this session), `CarParamsPersistent` (what offroad UI
  shows), `CarParamsPrevRoute` (last good) — print `carFingerprint | fingerprintSource | carVin |
  len(carFw)` and the raw `carFw` entries. Identical FW bytes between good and failing sessions =
  matcher regression (test `match_fw_to_car` directly); missing/short FW list = query problem (check
  pandaStates: both pandas present, `faultStatus: none`, aux `blackPanda` connected).

## Device access
- **Find the device FIRST via the CloudWatch locator, don't probe/sweep** (roams WiFi segments; guest
  networks have client isolation): `aws --profile dipeit logs filter-log-events --region us-west-2
  --log-group-name /aws/lambda/comma-uploader-api --filter-pattern CLIENT_IP
  --start-time $(( ($(date +%s)-3600)*1000 )) --query 'events[-1].message' --output text`
  (uploader self-reports `local_ip` on every upload_url request). Recent Lambda hits also prove
  "manager is up" without SSH (uploader only runs under manager).
- Common IPs: home `192.168.13.154`, iPhone hotspot `172.20.10.10` (port 22), work/roaming `10.16.x.x`
  (changes constantly). Car-powered: car off = device off. Cell dead zones on mountain passes.

## build-on-boot model
`launch_chffrplus.sh`: if `prebuilt` is absent, boot runs `build.py` (scons, `CacheDir
/data/scons_cache`) BEFORE manager. We run with `prebuilt` removed permanently. Costs: no-op boot
seconds; params_keys.h ~1-2 min; `cereal/*.capnp` or camerad C++ = WIDE rebuild; a driving-model change
adds the tinygrad model compile (~4 min total observed for lebowski). **A long-silent boot pane showing
the spinner (`gbm_create_device`) usually means scons/compile is running — check
`ps -eo etimes,pcpu,args | grep -E "[s]cons|[c]lang|[b]uild.py"` before assuming a hang.**
**Build FAILURE shows the text.py screen**: `ps | grep "text.py openpilot failed"` → read the FULL error
with `cat /proc/<pid>/cmdline | tr '\0' ' '` (the real scons error is at the END, after pages of
SyntaxWarnings).

## Verification (after either path)
HEAD == expected sha + expected BRANCH; `tmux capture-pane -t comma -p | grep -ci "got taps"` == 0;
manager/ui/uploader/mapd up with GROWING etimes; zero `UnknownKeyName`; new params resolve; feature
smoke-test. Parse errors properly — `strings` splits long JSON lines, so grep-count "Traceback" matches
fragments; pipe through `python3 json.loads` filtering `levelnum>=40` / `exc_info` per daemon instead.

**CAR RECOGNITION — MANDATORY, the deploy is NOT done until it passes (driver directive 2026-07-10):**
after every deploy+reboot, log back in and verify the car is still detected. With the car powered ON
(ignition/READY — `card` only fingerprints onroad):
```bash
python3 -c "from cereal import car
cp = car.CarParams.from_bytes(open('/data/params/d/CarParamsPersistent','rb').read())
with cp as p: print(p.carFingerprint, len(p.carFw))"   # in the device venv, PYTHONPATH set
```
Must print the real platform (`TESLA_MODEL_S_HW3` / `FORD_F_150_LIGHTNING_MK1`) with a healthy FW
count — **`MOCK` = dashcam mode = FAILED; keep fixing until detected.** Known failure mode: a deploy
reboot while the car sleeps + the driver waking the car mid-boot races the FW query → ~1 FW answer →
MOCK (happened 2026-07-10). Recovery: car fully awake → cycle ignition so card re-fingerprints (the
MOCK cache is skipped by `car_helpers`' `brand != "mock"` check; a fresh query runs). The
never-persist-MOCK fix in `card.py` keeps a flaky read from overwriting the good
`CarParamsCache`/`CarParamsPersistent`, but the CURRENT session still runs passive until re-fingerprinted.

## Verification gotchas (each cost real time)
- **`pgrep -f <pat>` self-matches your SSH command**; use `ps -eo pid,etimes,cmd | grep '[m]apd'`
  (bracketed first char). Processes run as DOTTED MODULES (`system.loggerd.uploader`), not `.py` paths.
- **Empty-grep awk false positive**: `ps | grep X | awk '{exit ($1>=60)?0:1}'` exits **0 when grep
  matches nothing** (awk never runs the block) — a wait-loop on that "succeeds" while the process
  doesn't exist. Require a non-empty match count AND the etimes condition.
- **`pkill` a manager child ≠ it comes back**: plain `PythonProcess` entries (e.g. `uploader`) are NOT
  restart-on-crash — pkill leaves them dead until a full comma restart. (And `pkill -f` of a pattern
  contained in your own ssh command kills your shell mid-script.)
- **Crash-loop check**: `grep -c "starting python <module>"` newest swaglog — a few starts in ~3 min =
  boot settling; dozens = loop. Onroad-only crashers (e.g. `Widget._layout` shadowing) never show in
  offline import tests.
- **mem params (`/dev/shm/params`) still need registration in `params_keys.h`**; fast-updating ones →
  `CLEAR_ON_MANAGER_START`. Reading a PERSISTENT key from the mem store returns False forever.
- **soundd is safety-critical**: only guarded, isolated additions; deploy sound changes WITH the user
  (can't audio-verify remotely).
- **mapd boot-wedge** = usually INCOMPLETE OSM data; completing the WA/OR/ID download fixes it.
- Map DATA (`/data/media/0/osm`) + caches (`/data/pnw/location/`, incl. the police-proxy KEY file that
  must never enter the repo) live OUTSIDE the tree and survive resets; a FACTORY reset wipes them.

## Recovery
UI/params crash-loop: `sudo systemctl stop comma`, manually deploy a known-good sha (path 2), restart.
Bricked-feeling ~3-min warm-reboot loop = params/UI mismatch (see memory `params-mismatch-reboot-loop`).
Catastrophic (opendbc/panda era): reinstall from the frozen **`4devpnw`** fallback. AGNOS sshd is
independent of comma.service — SSH survives everything above.

See also: `~/gh/comma/docs/DEVICE-STATE.md` (param registry + channel map), memory notes
(`auto-update-blocked-finalize-no-build` = the validated updater posture, `device-locator-cloudwatch`,
`params-mismatch-reboot-loop`, `device-manager-restart-tmux`).
