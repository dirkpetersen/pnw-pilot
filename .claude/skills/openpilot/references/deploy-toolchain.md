# Deploy toolchain & on-device operations

How patches get from `~/gh/comma/` onto a moving comma device, safely and reproducibly. This is the
part of the workbench most likely to cause real damage (a bricked panda, a boot-loop, a wiped
overlay), so the conventions here are defensive on purpose.

## Table of contents

- [The overlay model](#the-overlay-model)
- [On-device paths & environment](#on-device-paths--environment)
- [Script families](#script-families)
- [The surgical-patch philosophy](#the-surgical-patch-philosophy)
- [Connection-loss-safe deploy pattern](#connection-loss-safe-deploy-pattern)
- [Persistence guards (survive reboot)](#persistence-guards-survive-reboot)
- [Shared-file layering](#shared-file-layering)
- [Anatomy of an update-*.sh](#anatomy-of-an-update-sh)
- [Rollback & restore](#rollback--restore)
- [Adding a new feature deploy](#adding-a-new-feature-deploy)
- [Device IP & reachability](#device-ip--reachability)

## The overlay model

The car runs openpilot from **`/data/openpilot`**, which is a **writable file overlay**, not a git
checkout you can `git pull` into. We deploy by copying/patching individual files in place. This is
why patchers are surgical (see below) and why a reboot can revert everything (see persistence
guards). We do **not** push git branches to the device.

The backup directory `~/gh/comma/comma-eb1f2f7/` is a snapshot of one device's `/data` state
(bp-6.0 era) and a restore source — it is not a repo and not a deploy target.

## On-device paths & environment

| Path | What |
|------|------|
| `/data/openpilot` | the running openpilot file overlay |
| `/data/openpilot/opendbc_repo` | the in-tree opendbc (car DBCs, safety) |
| `/data/dirk` | where `upload.sh` drops helper scripts; also holds `org/` backups and `stage/` inputs |
| `/data/dirk/org/<feature>/` | one-time backup of the device's original files for a feature |
| `/data/dirk/stage/<feature>/` | staged, md5-verified inputs for a deploy |
| `/usr/local/venv` | the device Python venv (**there is no `.venv` on the device**) |
| `/data/params/d/<Key>` | on-disk param values (e.g. `IsOnroad`, `DisableUpdates`) |

On-device Python needs both the venv and the right `PYTHONPATH`:

```bash
source /usr/local/venv/bin/activate
export PYTHONPATH=/data/openpilot:/data/openpilot/opendbc_repo
```

On-device builds use that venv's scons (no local `.venv`):

```bash
( cd /data/openpilot && PATH=/usr/local/venv/bin:$PATH scons -u -j"$(nproc)" )
# or a single target, e.g. after editing common/params_keys.h:
( cd /data/openpilot && PATH=/usr/local/venv/bin:$PATH scons -u -j"$(nproc)" common/params_pyx.so )
```

## Script families

All live at `~/gh/comma/` root. Naming is consistent: a feature `<feat>` has up to five scripts.

| Pattern | Runs on | Role |
|---------|---------|------|
| `patch-<feat>.py` | device | **Surgical, idempotent** patcher. Inserts changes at anchor points in the device's pristine files; never wholesale-replaces. Re-run = no-op. |
| `<feat>_assets.py` (`raven_assets.py`, `dmon_assets.py`) | device | base64+gzip bundle of the **non-`.py`** files a patcher needs (DBCs, `.h`, PNGs). `upload.sh` only ships `.sh`/`.py`, so binary/text assets are embedded here and extracted on-device. |
| `update-<feat>.sh` | device | Deploy driver: backup → overlay/patch → clear pyc → scons → flip params → verify → restart `comma.service`. |
| `rollback-<feat>.sh` | device | Restore live files from `/data/dirk/org/<feat>/` (leaves the backup in place). |
| `restore-backup-<feat>.sh` | device | Restore the device's original pre-deploy files. |

Orchestration scripts (run on the **dev host**):

| Script | Role |
|--------|------|
| `upload.sh` | scp **every** root `.sh`/`.py` to `device:/data/dirk`. Add a script → it ships automatically (`COMMA_IP` env, default `192.168.12.218`). |
| `deploy_xnor2sunny.sh` | Full Raven overlay to the comma 3X: scp ~16 opendbc files + panda `0x348` ignition block + UI toggle + on-device panda build + verify. Flags: `--flash` (reflash panda), `--reboot`. Idempotent. |
| `fingerprint.sh` | One-command ECU fingerprinting (replaces the manual two-tmux `pandad` + `fw_versions.py` dance). Car must be powered. See `references/panda-and-safety.md`. |
| `set-raven-fixed-fingerprint.py` | Set/remove the fixed `CarPlatformBundle=TESLA_MODEL_S_HW3` param (the Raven can't auto-fingerprint). |

Features currently scripted: `raven`, `dmon`, `f150-fingerprint`, `mapd-toggle`, `mapd-states`,
`auto2xnor`, `raven-fingerprint`.

## The surgical-patch philosophy

A `patch-<feat>.py` must obey three rules, because the device's install is not identical to any
worktree:

1. **Never wholesale-replace a file.** A full-file copy drags in imports/symbols the device's
   specific install doesn't have, causing import crashes. Instead, read the file, find a stable
   **anchor** (an exact existing line/block), and insert the addition adjacent to it.
2. **Be idempotent.** Check whether the addition is already present and skip if so. Re-running the
   whole deploy must change nothing the second time.
3. **Bail loudly on a missing anchor.** If the anchor isn't found, `raise` — never guess a location.
   A moved anchor means the device file changed and the patch needs review, not force.

End every patcher with an `ast.parse()` of each edited `.py` file so a syntax error is caught on the
device before restart, not at runtime.

## Connection-loss-safe deploy pattern

The car's network link is flaky (it's a car). A dropped SSH session must never leave a half-applied
deploy. Every `update-<feat>.sh` follows this shape:

1. **Stage first, verify with md5.** scp all inputs into `/data/dirk/stage/<feat>/`, compute
   `md5sum` on both ends, compare. Only proceed if they match.
2. **Back up originals once.** Copy live files to `/data/dirk/org/<feat>/` only if no backup exists
   yet (so re-runs don't overwrite the true original with an already-patched file).
3. **Run detached.** Launch the driver with `setsid`/`nohup` so a dropped shell doesn't SIGHUP it.
4. **Signal via sentinel.** `rm -f <feat>.DONE <feat>.FAIL` at the start; `touch …DONE` on success,
   `touch …FAIL` (with a message) on any failure. The dev host polls for the sentinel rather than
   holding the SSH session open.
5. **Atomic file swaps.** `cp src dst.tmp` → md5-check `dst.tmp` → `mv dst.tmp dst`. Never write the
   live file in place.

A `fail()` shell function that echoes the reason, `touch`es `.FAIL`, and `exit 1`s is the standard
idiom — call it at every checkpoint.

## Persistence guards (survive reboot)

**This is the guard that has cost the most debugging time.** On boot, `launch_chffrplus.sh` performs
an overlay-swap that **reverts `/data/openpilot` to the finalized image and reflashes the panda to
stock** — silently undoing your deploy. To prevent that, a deploy must set:

```bash
sudo rm -rf /data/safe_staging/finalized          # remove the finalized image it would swap to
touch -d "2020-01-01" /data/openpilot/.overlay_init # mark overlay as already-initialized (old mtime)
touch /data/openpilot/prebuilt                      # mark tree as prebuilt (skip rebuild-on-boot)
# DisableUpdates=1 — set as a param AND via the UI "Allow auto updates" toggle (OFF)
PYTHONPATH=/data/openpilot /usr/local/venv/bin/python -c \
  "from openpilot.common.params import Params; Params().put_bool('DisableUpdates', True)"
# The panda firmware at board/obj/panda_h7.bin.signed must be OUR build, or pandad reflashes to stock.
```

Also kill the updater so it can't re-stage during the session:
`pkill -f "[s]ystem.updated.updated"`, and clear
`/data/safe_staging/finalized/.overlay_consistent` and `/data/openpilot/.overlay_init` as the
specific deploy requires. The Raven overlay has the most complete version — see
`deploy_xnor2sunny.sh` step 1 and `RAVEN.md`.

Verified behavior: with the full guard set in place, a deploy survives a clean reboot.

## Shared-file layering

Several features write into the **same** files. A patch for one feature must preserve the others'
entries — these files are additive accumulators:

| Shared file | Features that layer into it |
|-------------|----------------------------|
| `common/params_keys.h` | every feature that adds a param (DM keys, `ShowSpeedLimit`, `NudgelessLaneChange`, `MapdVersion`, …) |
| `selfdrive/ui/layouts/settings/toggles.py` | every feature with a UI toggle (anchored on stock `AlwaysOnDM` lines) |

Pattern: anchor the insert on a **stock** line (one that's always present), not on another feature's
line (which may or may not be deployed). After patching, the mapd patcher explicitly greps that
`SensitiveDriverMonitoring`, `NudgelessLaneChange`, etc. still exist — a cheap guard against
clobbering a sibling feature. Mirror that check when touching these files.

## Anatomy of an update-*.sh

`update-mapd-toggle.sh` is the canonical reference. Its steps, in order:

1. `rm -f DONE FAIL`; redirect all output to a log; define `fail()`.
2. Preconditions: required dirs/files exist (else `fail`).
3. Back up originals once to `$ORG`.
4. Overlay any whole new files (md5-verified, atomic `.tmp`→`mv`).
5. Run `patch-<feat>.py $OP_DIR` for the surgical edits; grep to confirm the edits landed.
6. Clear bytecode: delete `__pycache__`/`*.pyc` under the touched dirs; `touch` edited files to bump mtimes.
7. Rebuild any compiled artifact the change affects (e.g. `scons … common/params_pyx.so` when a param default changed).
8. Flip live params to ship-state (e.g. force a new feature **OFF** by default).
9. Verify: `ast.parse` the edited `.py` files; read params back to confirm expected values.
10. Persistence guards (above).
11. Restart: `sudo systemctl restart comma.service` (fallback: `tmux kill-session -t comma` then start).
12. `touch DONE`.

## Rollback & restore

- `rollback-<feat>.sh` — restore live files from `/data/dirk/org/<feat>/`, delete any new files the
  feature added, clear pyc, tell the user to reboot. Leaves the backup so you can redeploy.
- `restore-backup-<feat>.sh` — restore the device's original pre-deploy files (deeper reset).
- To fully start over for a feature: delete `/data/dirk/org/<feat>/` so the next `update` re-captures
  a clean backup.

If the device UI bricks after a UI-file patch, the recovery recipe (clear pyc, restore the toggles
file from `org/`, restart `comma.service`) is documented per-feature in the MAPD/AUTO docs.

## Adding a new feature deploy

To add `<feat>`:
1. Write `patch-<feat>.py` (surgical, idempotent, anchor-based, `ast.parse` at the end).
2. If it needs non-`.py` assets, embed them in `<feat>_assets.py` (base64+gzip) with an extractor.
3. Write `update-<feat>.sh` following the 12-step anatomy, with `DONE`/`FAIL` sentinels and detach.
4. Write `rollback-<feat>.sh` (restore from `org/`).
5. `COMMA_IP=<ip> ./upload.sh` ships all of them automatically — no edit to `upload.sh` needed.
6. Document the feature in its branch's in-tree doc (`<FEAT>.md` / `DEPLOY.md`).

## Device IP & reachability

The device's address depends on the network it's joined: `192.168.13.154` on the home network, a
`10.x` address on mobile. Set `COMMA_IP` to the current one. If `deploy_xnor2sunny.sh` reports
"device unreachable (asleep?)", the car is likely asleep — **the car must be awake** (ACC/ignition
on, or recently woken) for any deploy, and **powered** for fingerprinting or panda flashing.
