# AUTO2XNOR.md — Nudgeless Lane Change + No-Disengage-on-Braking + Overtake Assist (comma 3X)

> **Deployment** companion for the `auto2xnor` branch. Documents the three toggles, why
> they are **Tesla-only (greyed out on the Ford Lightning)**, why `params_keys.h` /
> `toggles.py` must be **merged, not overlaid**, and how the deploy is **connection-loss safe**.

**Branch:** `auto2xnor` (openpilot-xnor worktree), off `c0d78143` (xnor prebuilt base).
**Device:** comma **3X** (`tizi`) @ `comma@192.168.13.154`, distro branch `xnor-dev`.

## What it adds (3 toggles, all default OFF, all Tesla-only)
- **Nudgeless Lane Change** (`NudgelessLaneChange`) — in `desire_helper.py`, hold the
  blinker ~1.5 s above 20 mph with no blindspot → lane change starts without a wheel
  nudge. Nudge path preserved; blindspot still blocks; speed gate unchanged. Re-read ~3 s.
- **No Disengage on Braking** (`NoDisengageOnBrake`) — in `selfdrived.py`, suppresses the
  brake/regen-braking `pedalPressed` disengage so OP stays engaged through brake presses
  (resumes on release). Gas-pedal disengage unaffected. Re-read live in `params_thread`.
- **Overtake Assist** (`OvertakeAssist`) — DISPLAY-ONLY prompt (`overtake_assist.py` +
  `augmented_road_view.py`). When closing on a slower lead on the highway with a clear
  adjacent-lane blind spot, shows a green arrow + "Signal to overtake". openpilot does NOT
  steer — the driver flicks the blinker (nudgeless then completes it). Replaced an earlier
  auto-steer `AutoInitiateLaneChange` (reverted: OP can't command the Raven blinker and
  can't see fast traffic approaching in the target lane).

**Tesla-only gating:** all three are in `TogglesLayout.TESLA_ONLY_TOGGLES`
(`NudgelessLaneChange`, `OvertakeAssist`, `NoDisengageOnBrake`). The `_update_toggles` loop
disables (greys out) and force-OFFs them when `ui_state.CP.brand != "tesla"` — so on the
**Ford F-150 Lightning** they are visible but disabled. Their descriptions say "Tesla only".

Panda safety untouched — openpilot-side logic only. The brake toggle **reduces a safety
boundary** (its UI description says so); all default OFF per CLAUDE.md (safety > features).

## Files changed (8)
| File | How it deploys |
|------|----------------|
| `selfdrive/controls/lib/desire_helper.py` | **overlay** (auto2xnor HEAD; Tesla-gated nudgeless) |
| `selfdrive/modeld/modeld.py` | **overlay** (`DH = DesireHelper(CP)` — CP brand gates nudgeless) |
| `selfdrive/selfdrived/selfdrived.py` | **overlay** (NoDisengageOnBrake) |
| `selfdrive/ui/onroad/overtake_assist.py` | **new file** (OvertakeAssistRenderer) |
| `common/params_keys.h` | **MERGE in place** (carries DM + mapd keys — must not clobber) |
| `selfdrive/ui/layouts/settings/toggles.py` | **MERGE in place** (carries DM + mapd toggles) |
| `selfdrive/ui/ui_state.py` | **MERGE in place** (`overtake_assist` read alongside mapd `show_speed_limit`) |
| `selfdrive/ui/onroad/augmented_road_view.py` | **MERGE in place** (both Overtake + mapd SpeedLimit renderers) |

> **Merge, not overlay:** `params_keys.h`, `toggles.py`, `ui_state.py`, and
> `augmented_road_view.py` are also edited by `mapd2xnor` (speed-limit) and `dmon2xnor-b`
> (relaxed DM / update gate). The device runs all three feature sets, so a wholesale overlay
> of auto2xnor's versions would DELETE the mapd/DM additions. The resolved 3-way merge is
> recorded on branch **`integration-device`** (`74e90ff8`) — see "Cross-branch integration".

---

## Why a build is required
`common/params_keys.h` registers new keys (`NudgelessLaneChange`, `NoDisengageOnBrake`, `OvertakeAssist`). `Params.checkKey` validates against the keys
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

---

## Cross-branch integration (the on-device merge) + deploy history

`auto2xnor`, `mapd2xnor`, and `dmon2xnor-b` all edit the same 4 shared files
(`params_keys.h`, `toggles.py`, `ui_state.py`, `augmented_road_view.py`), and the device
runs all three at once. They are kept on **separate branches**; the on-device files are the
**3-way merge** of all three, recorded on branch **`integration-device`** (commit `74e90ff8`):

- `params_keys.h` — auto (`NudgelessLaneChange`/`NoDisengageOnBrake`/`OvertakeAssist`) +
  mapd (`ShowSpeedLimit`/`OsmStateName`=`WA,OR,ID`/`Offroad_OSMUpdateRequired`) +
  DM (`SensitiveDriverMonitoring`/`AllowSoftwareUpdates`). The stray auto-steer key
  `AutoInitiateLaneChange` is **removed** (auto2xnor reverted to display-only OvertakeAssist).
- `toggles.py` — all 6 toggles; auto's three Tesla-gated via `TESLA_ONLY_TOGGLES`.
- `ui_state.py` — `overtake_assist` read alongside mapd `show_speed_limit`/`show_road_name`.
- `augmented_road_view.py` — both `OvertakeAssistRenderer` (auto) and `SpeedLimitRenderer` (mapd).

The deploy was connection-loss-safe (stage md5-verified → `setsid nohup` →
`DONE`/`FAIL` sentinel + log) via `update-auto2xnor-integ.sh`. Backups at
`/data/dirk/org/auto-integ/`. Verified after a cold reboot: all three feature sets'
params resolve, 0 `UnknownKeyName`, `shouldRun-but-not: ['updated']` (expected w/ DisableUpdates).

### Ford grey-out (commit `8b029580`)
All three auto toggles are **Tesla-only**: not supported on the Ford F-150 Lightning, so they
are disabled (greyed out) and force-OFF on non-Tesla cars by the `is_tesla` loop in
`_update_toggles`. `NoDisengageOnBrake` was added to `TESLA_ONLY_TOGGLES` in `8b029580`
(it had been left out); `NudgelessLaneChange` + `OvertakeAssist` were already gated.

> **NOTE — not yet deployed:** the `8b029580` Ford grey-out commit is on the `auto2xnor`
> branch but has **not** been pushed to the device yet. The live `toggles.py` still greys out
> only Nudgeless + OvertakeAssist; redeploy `toggles.py` (merge in place, no params rebuild
> needed — it's a pure-Python UI change) to apply the NoDisengageOnBrake grey-out on Ford.
