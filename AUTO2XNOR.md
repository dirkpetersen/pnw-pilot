# AUTO2XNOR.md — Deploying Nudgeless Lane Change + No-Disengage-on-Braking to a comma 3X

> **Deployment** companion for the `auto2xnor` branch. Documents exactly how the two
> toggles were deployed to the live device, why `params_keys.h` (and `toggles.py`)
> must be **patched, not overlaid**, and how the deploy is made **connection-loss safe**.

**Branch:** `auto2xnor` (openpilot-xnor worktree), off `c0d78143` (xnor prebuilt base).
**Device:** comma **3X** (`tizi`) @ `comma@192.168.13.154`, distro branch `xnor-dev`.

## What it adds (2 toggles, both default OFF)
- **Nudgeless Lane Change** (`NudgelessLaneChange`) — in `desire_helper.py`, hold the
  blinker ~1.5 s above 20 mph with no blindspot → lane change starts without a wheel
  nudge. Nudge path preserved; blindspot still blocks; speed gate unchanged. Re-read ~3 s.
- **No Disengage on Braking** (`NoDisengageOnBrake`) — in `selfdrived.py`, suppresses the
  brake/regen-braking `pedalPressed` disengage so OP stays engaged through brake presses
  (resumes on release). Gas-pedal disengage unaffected. Re-read live in `params_thread`.

Panda safety untouched — openpilot-side logic only. The brake toggle **reduces a safety
boundary** (its UI description says so); both default OFF per CLAUDE.md (safety > features).

## Files changed (4)
| File | How it deploys |
|------|----------------|
| `selfdrive/controls/lib/desire_helper.py` | **overlay** (pristine on device) |
| `selfdrive/selfdrived/selfdrived.py` | **overlay** (pristine on device) |
| `common/params_keys.h` | **PATCH in place** (carries DM + mapd keys — must not clobber) |
| `selfdrive/ui/layouts/settings/toggles.py` | **PATCH in place** (carries DM + mapd toggles) |

---

## Why a build is required
`common/params_keys.h` registers 2 new keys. `Params.checkKey` validates against the keys
compiled into `params_pyx.so`, so an unregistered key → `UnknownKeyName` at runtime
(which, via the UI/daemon, can crash-loop the stack — see MAPD2XNOR.md pitfalls). The
xnor-dev 3X has an on-device build env (`/usr/local/venv/bin/scons` 4.10.0 + Cython + g++),
so we rebuild `params_pyx.so` on the device. `desire_helper.py` / `selfdrived.py` are pure
Python (no build needed) but still need a stack restart and pyc-cache clear to take effect.

## Why PATCH `params_keys.h` + `toggles.py`, never overlay ⚠️
This device already carries other features' additions to **both** files:
- `params_keys.h`: `SensitiveDriverMonitoring`, `AllowSoftwareUpdates` (dmon2xnor-b), the
  16 mapd keys + `Offroad_OSMUpdateRequired` (mapd2xnor).
- `toggles.py`: the "Sensitive Driver Monitoring" + "Allow software updates" toggle entries.

A wholesale copy of the auto2xnor branch versions would **delete all of those** → DM/UI/mapd
crash on `UnknownKeyName`. So `patch-auto2xnor.py` inserts **only** the auto2xnor blocks,
anchored on stock lines (`{"AlwaysOnDM", ...}` and the `"AlwaysOnDM"` toggle-def/description),
which exist in every variant. It is **idempotent** (re-run = no-op) and ends by `ast.parse`-ing
`toggles.py` so a bad insert can't reach a restart.

> General rule for this device: any file multiple separate feature branches touch
> (`params_keys.h`, `toggles.py`) is **patched per-feature on-device**, never overlaid.

---

## Connection-loss-safe deploy (the procedure used)
The hard lesson from the mapd deploy (MAPD2XNOR.md, Pitfall 3): an SSH drop mid-operation
left a half-applied build input and bricked the UI. So this deploy is structured to **finish
on its own even if my SSH session dies**:

1. **Upload everything FIRST** to a staging dir, before any patching runs:
   ```
   /data/dirk/stage/auto2xnor/desire_helper.py     # branch version
   /data/dirk/stage/auto2xnor/selfdrived.py        # branch version
   /data/dirk/stage/auto2xnor/patch-auto2xnor.py   # the in-place patcher
   /data/dirk/update-auto2xnor.sh                  # orchestrator
   ```
   Each upload is **md5-verified** against the local file before proceeding.

2. **Launch the orchestrator DETACHED** so it survives SSH loss:
   ```
   ssh comma@… 'setsid nohup bash /data/dirk/update-auto2xnor.sh >/dev/null 2>&1 &'
   ```
   `update-auto2xnor.sh`:
   - backs up the 4 originals once → `/data/dirk/org/auto2xnor/`
   - overlays the 2 pristine logic files (md5-verified, atomic `mv`)
   - runs `patch-auto2xnor.py` to insert the keys/toggles in place
   - verifies the DM/mapd keys are still present (not clobbered)
   - clears `__pycache__`/`*.pyc`, bumps mtimes
   - rebuilds `params_pyx.so` (`scons -u -j$(nproc) common/params_pyx.so`)
   - verifies both new keys resolve, and all 3 py files `ast.parse`
   - sets `DisableUpdates=1`, removes overlay sentinels
   - `systemctl restart comma.service`
   - writes a sentinel: `/data/dirk/auto2xnor.DONE` on success, `auto2xnor.FAIL` on error
   - logs every step to `/data/dirk/auto2xnor-deploy.log`

3. **On reconnect**, the dev host polls for `auto2xnor.DONE` / `auto2xnor.FAIL` and tails
   `auto2xnor-deploy.log` — so a dropped SSH never leaves the deploy in an unknown state.

---

## Verify after deploy
```
cat /data/dirk/auto2xnor.DONE            # exists on success
tail -40 /data/dirk/auto2xnor-deploy.log
# keys resolve + nothing clobbered:
PYTHONPATH=/data/openpilot /usr/local/venv/bin/python -c \
  "from openpilot.common.params import Params; p=Params(); \
   print('nudgeless',p.get_bool('NudgelessLaneChange'),'brake',p.get_bool('NoDisengageOnBrake'), \
         'DM',p.get_bool('SensitiveDriverMonitoring'))"
# UI + stack healthy (no UnknownKeyName, nothing crash-looping):
pgrep -f selfdrive.ui.ui ; grep -c UnknownKeyName /data/log/error.log
```
On-road: toggle ON "Nudgeless Lane Change", >20 mph, hold blinker ~1.5 s with clear blindspot
→ lane change starts without a nudge. Toggle ON "No Disengage on Braking" → tap brake, OP
stays engaged and resumes on release. Both live-refresh (no restart to toggle).

## Rollback
```
ORG=/data/dirk/org/auto2xnor
cp $ORG/selfdrive/controls/lib/desire_helper.py        /data/openpilot/selfdrive/controls/lib/
cp $ORG/selfdrive/selfdrived/selfdrived.py             /data/openpilot/selfdrive/selfdrived/
cp $ORG/common/params_keys.h                           /data/openpilot/common/
cp $ORG/selfdrive/ui/layouts/settings/toggles.py       /data/openpilot/selfdrive/ui/layouts/settings/
cd /data/openpilot && PATH=/usr/local/venv/bin:$PATH scons -u -j$(nproc) common/params_pyx.so
# clear pyc, then: sudo systemctl restart comma.service
```
> NOTE: the `org/auto2xnor/` backups capture `params_keys.h`/`toggles.py` **as they were
> on the device at first run** — i.e. WITH the DM/mapd additions. Restoring them removes only
> the auto2xnor additions, preserving DM/mapd. Correct by construction.

## Cross-feature notes
This device also carries: relaxed DM + update-gate (`dmon2xnor-b`, DMON.md §16), a pending
F-150 fingerprint (`light2xnor`, applied via `update-f150-fingerprint.sh`), and OSM
speed-limit display (`mapd2xnor`, MAPD2XNOR.md). All kept on **separate branches**; the only
on-device collision is `params_keys.h` + `toggles.py`, handled by per-feature patching here.
The core-hotplug `set_core_affinity` fix (`55fbcd44`, dmon2xnor-b) is required on this 3X but
is **not** on auto2xnor.
