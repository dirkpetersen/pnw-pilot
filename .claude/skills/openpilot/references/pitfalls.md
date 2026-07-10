# Pitfalls — hard-won lessons

A consolidated list of mistakes that have actually cost time, money, or a near-brick on this
workbench. Each entry: the trap, why it happens, what to do instead. Skim this before any
non-trivial action — most of these are invisible until you've already hit them.

The deeper context for each lives in the topic references; this file is the fast warning layer.

## Strategy & direction

- **Don't port xnor's Tesla support into sunnypilot/bluepilot.** It's nearly impossible to land
  (the panda flash blocks it), and two serious attempts (`xnor2bp`, `xnor2sunny`) got the code done
  but never drove. **Port other features INTO the xnor base instead** (`<feature>2xnor`). xnor is
  currently very up to date, so you don't sacrifice freshness. → `references/forks-and-porting.md`.

- **Any change that forces a panda reflash on a non-xnor base reintroduces the wall.** The
  `CAN_PACKET_VERSION_HASH` must match between firmware and library; xnor works because it's a matched
  set. If a port needs a panda rebuild on a different opendbc, expect hash-mismatch + enumeration
  failures. Prefer panda-safe (UI/carstate/control) changes. → `references/panda-and-safety.md`.

- **Don't confuse `xnor-dirk` with `xnor/master`.** `xnor/master` is a huge commaai fork with **no**
  Tesla legacy files — basing on it silently loses the Raven. Base on `xnor-dirk` (the "prebuilt"
  Tesla branch) and verify the legacy files are present first.

## Repo & branches

- **The root `~/gh/comma` is not a git repo.** `git status` there fails. The real `.git` is in
  `openpilot/`; `bluepilot`/`xnor/openpilot worktrees of it.

- **Don't trust assumed branch names.** Branches get renamed between sessions; always
  `git -C <dir> branch --show-current`. The feature branches (`auto2xnor`, `dmon2sunny`, …) and the
  `<fork>-dirk` base branches coexist.

- **Read the branch's own in-tree doc, not just the root mirror.** `AUTO2XNOR.md`/`MAPD2XNOR.md`/
  `DEPLOY.md` committed on the branch are the current truth and can be ahead of root copies.

- **The branch-protection hook is not a bug.** Commits/pushes to `main`/`master`/`bp-dev` are blocked
  by design — switch to a feature branch rather than fighting it.

## Forks & cross-branch deploys

This workbench is many forks + many feature branches. Each `*2pnw`/`*2xnor` feature branch is cut
off `main`/the base and carries **only its own** params/cereal/code; the integration branch
(`3pnwtest`, `integration2xnor`) is the **union** of all features. Mixing them on a device is the
single most expensive trap here.

- **Deploying ONE feature branch's shared files onto a device built from a DIFFERENT branch silently
  breaks it — and the symptom is a reboot loop that looks like a power/hardware fault.** Real case
  (hours lost): deploying `mapd2pnw`'s `common/params_keys.h` (off `main`, so missing the CES/network
  params) onto a device running `3pnwtest` content **dropped `ConditionalExperimentalSwitching`**. The
  UI's `toggles.py` then called `get_bool("ConditionalExperimentalSwitching")` → **`UnknownKeyName`** →
  the raylib UI crashed building `SettingsLayout` → `restart_if_crash` crash-looped it → the **SDE/DRM
  display driver (panic-on-error) warm-rebooted the comma 3X every ~180 s.** No visible param error,
  just reboots. **Always deploy shared files (`params_keys.h`, `toggles.py`, `cereal/*`) from the
  INTEGRATION branch (the union), never a single feature branch's subset.** A clean comma-installer
  install of the integration branch sidesteps the whole file-overlay-subset hazard.

- **Reboot-loop triage — it's almost always in-stack, not power.** `sudo systemctl stop comma`. If
  uptime then climbs past the reboot point, the reboot originates INSIDE the openpilot stack, not the
  power rail / PMIC / AGNOS. With the stack stopped you have a stable SSH box; reproduce the actual
  crash by running the suspect process by hand —
  `cd /data/openpilot && PYTHONPATH=/data/openpilot timeout 12 python3 selfdrive/ui/ui.py 2>&1 | tail -40`
  printed the exact `UnknownKeyName` traceback in seconds. Don't burn time on PMIC reason codes,
  AGNOS-version, overlay-finalize, or offroad low-voltage shutdown first — `stop comma` tells you in
  one step whether any of that is even relevant.

- **The `openpilot` Python package is a wrapper of SYMLINKS.** `openpilot/` = `__init__.py` + symlinks
  `common`/`selfdrive`/`system`/`tools` → `../`. A NEW top-level package a fork adds (e.g. `sunnypilot/`)
  is **not** importable as `openpilot.sunnypilot` until a committed symlink `openpilot/sunnypilot ->
  ../sunnypilot` exists — without it, `ModuleNotFoundError: openpilot.sunnypilot` at manager startup
  takes the **whole stack down**. Put new fork code under an already-symlinked dir (`system/`,
  `selfdrive/`) to avoid it entirely, or add the symlink and commit it.

- **Harden param-driven UI against branch skew.** A toggle whose param isn't registered hard-crashes
  the UI (and can escalate to the reboot loop above). `toggles.py` now filters `_toggle_defs` to only
  resolvable params (hides + logs the rest, guards per-feature overrides) so a params/UI mismatch
  degrades gracefully instead of bricking the device — keep that defensive pattern for any
  param-backed UI you add.

- **The submodule (opendbc/panda) and the superproject push SEPARATELY — and forgetting the submodule
  push silently breaks every fresh clone.** Real case (hours): work was done inside pnw-pilot's
  `opendbc_repo/` checkout, the superproject pointer (`3pnwtest → opendbc 0e59874b`) was committed and
  pushed, **but `0e59874b` was never pushed to `dirkpetersen/pnw-opendbc`** — it lived only in the local
  submodule `.git`. So `git clone … && git submodule update --init` tries to fetch a commit the fork
  doesn't have → the submodule comes up empty/broken ("doesn't respect the submodule structure"). After
  ANY change in a submodule: **push the submodule first, then bump+push the superproject.** The
  **2026‑06‑20 fork migration** (re-fork xnor-tech → `pnw-*`, repoint remotes) is exactly when this
  stranding happened — superproject pointers got re-pushed, pinned submodule commits didn't. Diagnose
  with `git -C opendbc_repo branch -r --contains <pinned-sha>` — empty = unpushed = dangling.

- **`*2pnw` branches cut off commaai-modern `main` SILENTLY DROP the Tesla Model S Raven.** The modern
  commaai opendbc has only `TESLA_MODEL_3/X/Y` (DAS_status blind-spot); the **Model S Raven**
  (`TESLA_MODEL_S_HW3` + `tesla_legacy.h` safety + `tesla_raven_party.dbc` + legacy carstate) lives ONLY
  on the opendbc **`tesla-raven`** branch. A feature branch built off modern `main` therefore has **no
  Raven** — it boots fine offroad (the car layer is dormant) but **won't drive the Model S** (unsupported
  fingerprint / wrong safety). This is why "is it ready?" must include "which opendbc commit, and does it
  have `TESLA_MODEL_S_HW3`?", and why bsm2xnor's `0x399` blind-spot looked "redundant" — the whole legacy
  carstate it attaches to was absent. Fix: merge `tesla-raven` into the integration opendbc, verify both
  `tesla.h` AND `tesla_legacy.h` are in `safety.h`, publish, bump the pointer.

- **opendbc feature branches mirror the openpilot ones:** `pnw-opendbc/tesla2pnw` (Raven) +
  `pnw-opendbc/lightning2pnw` (Ford 2025 fp); the integration opendbc (`pnw-opendbc/3pnwtest`) is their
  merge. There is NO `tesla2pnw` in pnw-pilot — the Raven is entirely opendbc + panda; the openpilot
  superproject has no Tesla-specific code (bsm was skipped). Don't create empty mirror branches.

- **Feature branches CAN carry opendbc/panda as inline trees OR submodule gitlinks — check which.** Most
  `*2pnw` branches use submodule gitlinks (160000) to `../pnw-opendbc`/`../pnw-panda`; `testing` vendors
  them inline (040000 tree). `git ls-tree <branch> opendbc_repo` shows the mode. A stray `M opendbc_repo`
  in `git status` is the gitlink drifting — `git checkout -- opendbc_repo` before committing unrelated
  work so you don't accidentally bump it.

## Deploy & device

- **A reboot wipes your overlay unless persistence guards are set.** `launch_chffrplus.sh` swaps the
  overlay and reflashes the panda to stock on boot. Set the guards (`rm -rf
  /data/safe_staging/finalized`, old-mtime `.overlay_init`, `prebuilt`, `DisableUpdates=1`, ensure the
  signed panda bin is *our* build) every deploy. → `references/deploy-toolchain.md`.

- **`DisableUpdates=1` after every deploy**, or the auto-updater overwrites your on-device changes. A
  reinstall resets this param (and the SSH host key).

- **Never wholesale-replace a device file.** A full-file copy pulls in imports the device install
  lacks → import crash. Patch surgically at anchors, idempotently, and `ast.parse` at the end.

- **Shared files accumulate across features.** `common/params_keys.h` and `toggles.py` hold multiple
  features' entries. Anchor inserts on **stock** lines, and grep afterward to confirm sibling
  features' keys survived. Clobbering them silently disables another feature.

- **Deploys must survive an SSH drop.** Stage+md5 first, run detached (`setsid`/`nohup`), signal via
  `*.DONE`/`*.FAIL` sentinel, swap files atomically. A dropped car-network link mid-deploy must not
  leave a half-applied state.

- **The device IP follows the network, not the device.** Home `192.168.13.154`, mobile a `10.x`.
  Set `COMMA_IP`; "unreachable (asleep?)" usually means the car is asleep — wake it.

- **The car must be awake to deploy and powered to flash/fingerprint.** No exceptions.

- **After changing a param default in `params_keys.h`, rebuild `params_pyx.so`** and flip the live
  param — the default is compiled in, so the on-disk value won't change on its own.

- **ALWAYS clear `__pycache__`/`*.pyc` after editing ANY device `.py` — make it a reflex.** If you
  don't, the device runs stale bytecode and your change appears to do nothing (or worse, you chase a
  phantom bug). Tells you're staring at stale bytecode: a traceback whose **line number + function
  don't match the current source** (e.g. `AssertionError` reported at a line that is now `return x`,
  or `in main` for code that now lives in another function) — the bytecode is from the *old* file.
  `rsync -a` makes this WORSE: it preserves the source's *old* mtime, so a freshly-copied `.py` can be
  *older* than the on-device `.pyc`, and Python keeps the stale cache. After copying, **bump mtimes**
  (`touch`) AND delete pyc. Clear with a broad name match (`find /data/openpilot -name '<mod>*.pyc'
  -delete`), not a path glob that misses `__pycache__/`. **The v0.11.2 install has a NESTED layout**
  (`/data/openpilot/openpilot/...`) — but it's the *package dir* whose `selfdrive`/`common`/`system`
  are **symlinks to `../`**, so it's the same files; pyc still lives under `__pycache__`. Verify
  against the path in the traceback / `module.__file__`.

- **Clearing pyc is NOT enough if the manager already imported the module — restart the manager (or
  reboot).** `system/manager` imports each managed process's module **into its own interpreter** and
  launches procs by **forking**; the children inherit the parent's already-imported (stale) module.
  So after you fix `pandad.py`, clear pyc, and `pkill pandad`, the manager re-forks the **old
  in-memory code** — your fix never runs. The symptom is maddening: correct source on disk, pyc
  cleared, yet the old traceback persists verbatim. Fix: `sudo systemctl restart comma` (fresh
  manager → fresh import) or reboot. Killing only the leaf process is never sufficient for a Python
  change.

- **On-device `scons` fails to even READ SConscripts while openpilot is running — the modeld probe
  needs the GPU.** `selfdrive/modeld/SConscript` runs `from tinygrad import Device;
  Device.get_available_devices()` in a subprocess during the *read* phase to pick the build backend.
  If openpilot (modeld) is running it **holds the QCOM GPU lock**, the probe exits non-zero, and the
  WHOLE build aborts (`CalledProcessError ... probe_devices`) — even if your target is just `pandad`.
  Two ways through: (a) stop openpilot so the GPU frees (the probe then prints `QCOM CL CPU DSP`), or
  (b) if you can't safely stop the stack on a live car, temporarily wrap `available = probe_devices()`
  in `try/except` returning `{"QCOM","CL","CPU","DSP"}` so the read succeeds and your non-modeld
  target builds while openpilot keeps running — then revert the SConscript.

- **Stopping openpilot is harder than `systemctl stop comma`.** `comma.service` does
  `tmux new-session -s comma -d /usr/comma/comma.sh`; the manager runs inside that **detached tmux
  session and reparents out of the service cgroup**, so `systemctl stop` marks the unit inactive but
  the manager/modeld keep running and `comma.sh` relaunches them. To actually hold it down: kill the
  tmux session AND `pkill -9 -f system.manager`, or just `systemctl restart comma` when you want a
  clean fresh manager. Note `pkill`-ing pandad/manager can briefly **drop SSH** (the internal-panda
  reset cycles a USB hub shared with connectivity) — expect exit 255 and reconnect; it's not a brick.

- **No `prebuilt` flag → openpilot rebuilds on EVERY boot (slow), and pandad won't appear for
  minutes.** After a `restart comma` (or any boot) with changed/dirty source and no
  `/data/openpilot/prebuilt`, `launch_chffrplus.sh` runs a full on-boot `scons` before the manager
  starts — so "pandad missing + swaglog frozen" right after a restart often just means *it's still
  building*. Check `pgrep -fc 'scons|cc1plus'` before assuming a hang or rebooting into the same wait.

- **`systemctl restart comma` can be a silent NO-OP — verify the manager PID actually changed.** On a
  device whose `comma.service` shows `active (exited)` (oneshot) with the stack living under a
  **persistent `tmux` session**, `restart` re-runs `ExecStart` = `tmux new-session -s comma …`, which
  hits **"duplicate session"** and changes *nothing* — the manager and every process keep their old
  PIDs (and your edited `.py` is never re-imported). So `restart comma` is NOT a reliable way to cycle
  the stack. After it, confirm `pgrep -f manager.py` got a **new** PID; if not, **reboot** (or
  `tmux kill-server` and let `comma.sh` relaunch). Observed 2026-06-24 — this is the corrective to the
  "just `systemctl restart comma`" suggestion above.

- **Killing an `always_run` process that is NOT `restart_if_crash` does NOT bring it back.**
  `launch_chffrplus.sh` runs `./manager.py` **once**, then `while true; do sleep 1; done` (no respawn
  loop), and the manager won't relaunch a non-`restart_if_crash` proc that exited or was killed. So
  `pkill`-ing e.g. `uploader`/`updated`/`network_arbiterd` leaves them **dead until a reboot** (only
  `restart_if_crash=True` procs like `ui` revive). To cycle one of these, reboot — don't expect a
  `pkill` to "restart" it.

- **Updates install on REBOOT (overlay swap), not on download — and `DisableUpdates` makes `updated`
  exit and stay dead.** "Check for updates" only fetches/finalizes into `safe_staging/finalized`; the
  swap that activates the new code happens in `launch_chffrplus.sh` on the **next boot** (gated by the
  `.overlay_init`-vs-`.git` mtime check). `updated.py:main()` does `exit(0)` immediately when
  `DisableUpdates` is set, and since `updated` isn't `restart_if_crash`, turning `DisableUpdates` back
  OFF does **not** relaunch it — it needs a reboot. So any `DisableUpdates` change only takes effect
  after a reboot, and a freshly-downloaded update isn't live until you reboot.

- **swaglog rotates FAST and offroad events never reach a qlog — persist diagnostics to a file.**
  `/data/log/swaglog.*` is 1000+ tiny rotating files; a reboot plus a few minutes of activity pushes
  recent events out of the retained window, and an offroad `cloudlog.event(...)` only lands in swaglog
  (there's no qlog segment offroad, and qlogs upload only once online). So a device-side diagnostic
  you need to survive reboots/rotation must be written to a **persistent file** (e.g.
  `/data/<feature>_last.json`), not read back from swaglog after the fact.

- **On a git-checkout device, the boot REVERTS your uncommitted on-device edits.** When `/data/openpilot`
  is a clean git checkout (the post-UI-updater state, not a file overlay), `launch_chffrplus.sh`/the
  updater does a `git reset`/clean checkout of the tracked branch on boot. Any files you `rsync`/edit
  in place **and a binary you rebuild** are wiped back to the committed branch state, then rebuilt
  from it. Symptom: you deploy a fix, restart, and the *old* code runs with a **fresh** error counter
  (reset to a low number) — proving the boot re-checked-out and re-ran the original. To test a change
  on such a device you must either **commit it to the branch the device tracks** (then update), or
  `git checkout` the feature branch on the device itself — not leave it as working-tree changes. (This
  is the opposite of the old file-overlay model where in-place edits persisted with the guard files.)

- **NEVER sync device files with a broad `find -path '*…/<file>.py'`.** This UI ships **parallel
  trees** — the standard `selfdrive/ui/layouts/...` AND the small-screen `selfdrive/ui/mici/layouts/...`
  — with same-named files (`toggles.py`, `software.py`) defining **different** classes
  (`TogglesLayout` vs `TogglesLayoutMici`). A broad `find` matched both and overwrote the mici file
  with the standard one → `ImportError: cannot import name 'TogglesLayoutMici'` → **whole UI
  crash-loops** even on a `tizi` device, because `ui.py` imports `MiciMainLayout` at module top
  regardless of which layout it renders. Copy to **exact target paths** only.

- **`tizi` (comma 3X) renders the standard `MainLayout`**, not mici — `gui_app.big_ui()` is True for
  `tici`/`tizi`. So settings UI edits for the 3X go in `selfdrive/ui/layouts/settings/*` (e.g.
  `software.py`), NOT the `mici/` tree. But `ui.py` still **imports** the mici tree at top level, so a
  broken mici file crashes the 3X UI too (see above).

- **A `ListItem` with BOTH `action_item=` AND `description=` makes the action button (EDIT/ADD/VIEW)
  non-tappable / look greyed** in the v0.11.2 raylib list_view. `get_right_item_rect()` clamps the
  action hit-rect to `content_width - title_width`, and `set_parent_rect` isn't propagated to the
  action_item; a long title shrinks the tappable area below the drawn button width. Stock action rows
  (password "EDIT", APN) omit `description` and use **short titles** — copy that pattern. Note: the
  `LIST_ACTION` button style is a dark grey **even when enabled**, so "looks greyed" ≠ "disabled";
  the enabled state resolving True does not mean it's receiving taps.

- **`ConfirmDialog` has no `set_callback()` — pass `callback=` in the constructor.** Building the
  dialog then trying `dlg.set_callback(fn)` leaves `_callback=None`, so Confirm/Cancel silently do
  nothing. The dialog pops itself *before* firing the callback, so pushing a follow-up dialog from
  inside the callback is safe.

- **When a KNOWN-GOOD control also stops responding, suspect the device/input layer first, not your
  code.** A cable/touchscreen issue made *every* action button (incl. stock ones) non-tappable; it was
  fixed by re-seating cables, not code. Test a stock control as an A/B before deep-diving your change.

## Panda flash (the danger zone — read `RAVEN.md` first)

- **Stop `manager.py` before `flash.py`.** pandad holds the SPI `flock()`; otherwise flash.py hangs
  at "resetting" (the write already succeeded — it just can't verify).

- **A soft `reboot` does NOT power-cycle the panda chip** on the cuatro/comma 3X (it's powered through
  the device). Turn the **car ignition fully OFF for ~30 s** to truly power-cycle. A hung chip won't
  recover from a soft reboot.

- **Build the panda on the device and verify the hash before flashing.** Building elsewhere gives the
  wrong `CAN_PACKET_VERSION_HASH`. Known-good xnor hash: `0x75ABF276`.

- **Watch for the cherry-pick leftovers**: missing `opendbc/safety/ignition.h`, and duplicate
  `ignition_can_hook` in `can_common.h`. Both are handled by `patch-raven.py`, but recognize them if a
  manual build fails.

## Cars

- **The Raven can't auto-fingerprint** (EPS rejects UDS). Use the fixed `CarPlatformBundle` param;
  "selected manually" is expected. **Clear the fixed fingerprint before moving the device to the
  Ford**, or the Lightning is misdetected as a Tesla.

- **The 2025 F-150's EPS only answers Mazda's UDS query.** Adding FW strings isn't enough — mark EPS
  non-essential **for MK1 only** so the other 3 ECUs identify the car (BluePilot PR #130).

- **Raven uses the radar on the xnor base** — `radarUnavailable=False` for HW3 (verified on-device);
  openpilot does its own longitudinal in all modes (engage via the cruise stalk, PCM/TACC, after
  disabling stock Autopilot). Radar usage is NOT mode-dependent. The "vision-only / radarUnavailable=
  True" claim is **sunnypilot `xnor2sunny`-only** (a bus-mapping workaround) — don't apply it to the
  xnor deployment.

## Safety (never trade away)

- **Never weaken a safety check or bypass a panda safety model**, even to make something work.
  New toggles default OFF. If a fix tempts you to touch `opendbc/safety/` or a panda limit, stop and
  flag it. Priority: **safety > stability > quality > features.**

---

## PNW-era additions (2026-07 sprint — auto-update / lebowski / live-tuning era)

- **`.overlay_consistent` is written LAST in updater finalize** — rebooting the moment the staged
  SHA appears makes launch skip the install (updater self-heals but the update didn't land). Reboot
  only on `UpdateAvailable=1` + the marker file. (Full deploy procedure: the `pnw-pilot-deploy` skill.)
- **`git reset --hard` updates NEITHER LFS content NOR submodules.** Use `GIT_LFS_SKIP_SMUDGE=1`
  reset + separate `git lfs pull` (smudge-during-checkout OOM'd git on the 3X) +
  `git submodule update --init <path>` for pin bumps. Host side: `git add <submodule>` records the
  LOCAL checkout and silently clobbers an `update-index --cacheinfo` pin.
- **Gemini CLI treats `@tokens` in prompts as file attachments** — a raw diff 400s ("Unable to
  process input image") or, worse, silently attaches binaries and hallucinates about them. Always
  `sed 's/@/[at]/g'` diffs; always adjudicate findings against the tree (its 5-finding lebowski
  review was 0-for-5; other reviews caught real bugs — verify, never rubber-stamp).
- **Every `params.get/put` in ported/copied code needs a registered key** — a snapshot port carried
  `UsbGpuPresent/UsbGpuCompiled` uses that would have crash-looped modeld; audit with a grep of
  param names vs `params_keys.h` before shipping.
- **Empty-grep awk false positive**: `... | grep X | awk '{exit cond?0:1}'` exits 0 when grep matched
  NOTHING (awk never runs). Wait-loops must require a non-empty match too.
- **`pkill` on a manager child that isn't restart_if_crash leaves it dead** (e.g. uploader) — revive
  needs a full comma restart. And `pkill -f <pattern-in-your-own-ssh-cmdline>` kills your shell.
- **`strings` splits long swaglog JSON lines** — grep-counting "Traceback" matches fragments; parse
  with json.loads and filter `levelnum>=40` / `exc_info` per daemon.
- **Onroad alerts often aren't in swaglog** — parse segment qlogs (`onroadEvents` via
  tools/lib/logreader on-device) to identify which alert fired.
- **Heading-line geometry breaks beyond mapd's ~350 m path** — "ahead" projections onto the straight
  heading ray flap on curves for anything >~2 mi out. Fixes that worked: corridor-IDENTITY matching
  (rest areas via WayRef + per-file corridor tags), near-field straight-line bypass (EV ≤2.5 mi),
  range caps. Don't build new long-range perp filters on the heading line.
- **Uploading HD while driving starves the control stack** (selfdrivedLagging + locationd inputsOK
  blips on a loaded 3X) — heavy background IO/CPU belongs offroad-only.
- **The comma installer-era `3testpnw` is the FRIENDS' channel** — never point experiments at it;
  a wrong force-push reaches other people's cars on their next update cycle.
- **comma 3X ≈ comma four in compute** (same SD845 class; four differs in cameras/thermals/power) —
  the big_ USB-GPU model is irrelevant to both without the GPU accessory; don't download its 1.7 GB.
- **Upstream cherry-pick boundary**: commaai's 2026-06-20/21 "nested openpilot/" restructure — commits
  after it need path rewrites against our layout; prefer pre-restructure picks or snapshot ports.
