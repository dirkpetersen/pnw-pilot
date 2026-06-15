# INTEGRATION2XNOR

Integration of three openpilot feature branches into one branch `integration2xnor`.
**Merge + build + test only. NOTHING was deployed to any device.**

## Branches integrated (verified tips)

| Branch | Tip | Brings |
|--------|-----|--------|
| `light-ces-gentle` (base) | `fc278adeec` | DEPLOYED CES/VTSC base + gentle profile + 3-way `CESMode` selector + grey-out fix |
| `network2xnor` | `e7dc9851e8` | F-150 nudgeless (auto2xnor lineage) + tethering fixes + perpetual tethering + priority-WiFi arbiter |
| `connect2xnor` | `d47b213bf9` | two-pass WiFi-only upload + deleter preservation + firehose/connect indicators |

Note: `ces2xnor` (`c9718a77ab`) is the base of `light-ces-gentle`; `auto2xnor` (`b5196f7121`) is the
shared base of both `network2xnor` and `connect2xnor`. ces2xnor and auto2xnor diverged (auto has 2
newer F-150 nudgeless commits); the merge brings the union, as intended.

## Merge order

```
git checkout -b integration2xnor light-ces-gentle
git merge --no-ff network2xnor      # e1d1df6f61
git merge --no-ff connect2xnor      # 79c6efd2f2
```
network before connect (connect's wifi work builds on network's tethering).

## Conflicts and resolutions

### Merge 1 — network2xnor: NO CONFLICTS (auto-merged by ort)
Git's 3-way merge auto-resolved every overlapping file correctly. Verified by hand:
- **common/params_keys.h** — union produced automatically: kept `CESMode` + legacy
  `ConditionalExperimentalSwitching` (HEAD) AND added `TetheringEnabled` / `TetheringPriorityWifi`
  (network). No key dropped.
- **selfdrive/ui/layouts/settings/toggles.py** — kept light-ces-gentle's version verbatim.
  network2xnor made **no real toggles.py change** (`git diff <base> network2xnor -- toggles.py` is
  empty; the 13-line stat seen earlier was purely ancestry/CES-diff noise). Result has exactly ONE
  CES control (the 3-way `CESMode` `multiple_button_item`), ONE `ces_group_enabled(cp)` helper, ONE
  `CES_GROUP` grey-out loop, and ZERO old CES bool `toggle_item`.
- **selfdrive/controls/lib/desire_helper.py, selfdrive/modeld/modeld.py** — took network2xnor's
  (F-150 nudgeless) version. Confirmed identical to network2xnor (the nudgeless-Lightning change is
  preserved, not lost).
- **system/ui/lib/wifi_manager.py** — took network2xnor's SUPERSET. connect2xnor's version is a
  strict subset of the same hunks; network adds the `TetheringEnabled` persist in
  `set_tethering_active`. Verified `set_tethering_active` writes `TetheringEnabled`.
- **system/manager/process_config.py** — `network_arbiterd` registered (`enabled=TICI`).

### Merge 2 — connect2xnor: 2 CONFLICTS, both resolved

**common/params_keys.h** — UNION. Kept all of HEAD's CES* keys (`ConditionalExperimentalSwitching`,
`CESMode`, `CESCurves`, `CESStops`, `CESLowSpeed`, `CESLead`, `CESButtonState`, `CESStatus`,
`VTSCStatus`) AND added connect2xnor's `FirehoseActive` (CLEAR_ON_MANAGER_START BOOL "0"). No key
dropped. (Tethering keys were already present from merge 1, outside the conflict hunk.)

**system/loggerd/uploader.py** — COMBINED both features (neither dropped):
1. (HEAD / xnor lineage) the HOME-WiFi-ONLY metered hard-skip: `if networkMetered and not
   force_wifi: continue` — never spend cellular data.
2. (connect2xnor) the `network_type_raw`/`metered` computation, the pass-1 `uploader.step(...)`, and
   the pass-2 firehose upload `if success is None and pass2_allowed(...): uploader.step(...,
   pass2=True)`.
The metered skip runs first, then connect's pass-1/pass-2 logic. The only difference between the
merged file and pristine connect2xnor is that re-added metered block — exactly the intended union.

**system/ui/lib/wifi_manager.py** — auto-merged; already at network2xnor's superset (connect's hunks
are a subset). Nothing folded in from connect was lost.

All other connect2xnor files (deleter.py, sidebar.py, firehose.py, test_connect2xnor.py) merged with
no conflict and are byte-identical to connect2xnor.

## Union verification (all PASS)

```
TetheringEnabled                 1   (common/params_keys.h)
TetheringPriorityWifi            1
FirehoseActive                   1
CESMode                          1
ConditionalExperimentalSwitching 1
toggles.py: 1 CESMode selector, 1 ces_group_enabled helper, 0 old CES bool toggle
system/networkd/network_arbiterd.py present, registered in process_config.py (enabled=TICI)
uploader.py pass2=True present (x2: gate + call); deleter.py firehose preservation present
git grep conflict markers (excl. cereal/gen comment dividers): NONE
```

Integrated source diff vs `ces2xnor` is exactly the union of all three features' source changes
(22 source files, +1086/-46) — no feature lost.

## Build

Environment: x86_64 WSL2. `uv sync --frozen --python 3.12 --extra testing` created `.venv` (full
deps + pytest/ruff/ty). `tools/op.sh setup` was NOT used (would need sudo apt for system libs).

- **`common/params_pyx.so` — BUILT OK.** This validates the `params_keys.h` union compiles and the
  three new params read at their defaults at runtime (CESMode=0, TetheringEnabled=False,
  FirehoseActive=False).
- **`msgq` cython .so's (ipc_pyx, visionipc_pyx) — BUILT OK** (needed for the messaging test stack).
- **Full `scons -u` — PARTIAL.** Fails on missing SYSTEM libraries, NOT on any merged code:
  - `Unexpected non-vendored library 'acados'` (lateral/longitudinal MPC solver — not vendored here)
  - `libusb-1.0/libusb.h: No such file` (pandad)
  - ffmpeg/VA-API `undefined reference to va*` (encoderd/loggerd link)
  These are missing apt packages a full device/CI `op.sh setup` installs. The tinygrad model
  compile/test step completed. All merged code is Python + the params header, which compiled cleanly.

## Tests

Ran with `pytest -o addopts=""` (pyproject pins xdist flags unsupported by the installed plugin).

### The four required suites
| Suite | Result |
|-------|--------|
| `selfdrive/controls/lib/ces_xnor/tests/test_ces_mode.py` | **17 passed, 1 FAILED** |
| `selfdrive/controls/lib/vtsc_xnor/tests/` | **28 passed** |
| `system/networkd/tests/test_network_arbiter.py` | **16 passed** |
| `system/loggerd/tests/test_connect2xnor.py` | **11 passed** |

**Combined: 72 passed, 1 failed.**

The one failure — `test_ces_controller_runtime_mode_switch_rebuilds_sm` — is **PRE-EXISTING on
`light-ces-gentle`, not a merge regression.** `ces_xnor.py`, `ces_xnor_constants.py`, and
`test_ces_mode.py` are byte-identical to `light-ces-gentle` (`git diff light-ces-gentle HEAD` on
those files is empty). Root cause: `CESController._read_params()` only re-reads `CESMode` once per
~1 s (`if self._frame % (1/DT_CTRL) == 0`); the test calls `_read_params()` twice back-to-back
without advancing `_frame` by ~100, so the second (runtime mode-switch) read is throttled out and
`_gentle` never flips. This is a test/impl mismatch the human owns on the CES branch — NOT touched
or caused by integration. Left as-is per "do not silently alter feature code."

### Broad sanity pass (`pytest selfdrive/.../ces_xnor vtsc_xnor system/networkd system/loggerd`)
**120 passed, 16 failed, 2 skipped.** The 16 failures break down as:
- **1** = the CES test above (pre-existing, light-ces-gentle).
- **4** `system/loggerd/tests/test_uploader.py` ("files uploaded twice", etc.) — **PRE-EXISTING on
  `connect2xnor`.** connect2xnor added `test_connect2xnor.py` but never updated the STOCK upstream
  `test_uploader.py`, which doesn't know about the pass-2 firehose re-walk and counts pass-2 uploads
  as duplicates. The merged stock `test_uploader.py` is byte-identical to connect2xnor's. This is a
  loose end on the connect2xnor feature branch, surfaced by integration but not caused by it.
- **11** `system/loggerd/tests/test_loggerd.py` (OSError / format errors) — **ENVIRONMENTAL.** These
  spawn the `loggerd`/`encoderd`/`bootlog` C++ binaries as subprocesses; the only binaries available
  are the committed **aarch64 (device)** builds (the x86 C++ build couldn't link — missing
  acados/libusb/vaapi system libs), so they fail to exec on x86. The merge only touched Python
  (`deleter.py`, `uploader.py`); `test_loggerd.py` exercises the C++ loggerd, which integration did
  not modify.

### Lint
`ruff check` on the merged files: **4 errors, all `E702` (semicolons) in `ces_xnor.py`** — all
PRE-EXISTING on `light-ces-gentle` (that file is identical post-merge). Every merge-touched file
(uploader, deleter, toggles, wifi_manager, networkd/*, vtsc/*) is **ruff-clean**.

## TODO / things flagged for human review before deploying

1. **CES runtime mode-switch test fails on light-ces-gentle itself** (`test_ces_mode.py:119`). Either
   the test should advance `_frame` ~100 steps between mode changes, or `_read_params` should re-read
   `CESMode` immediately. Pre-existing; not fixed here (CES branch is the human's). Behavior on-car:
   a runtime Light<->Standard switch takes up to ~1 s to take effect — likely benign.
2. **Stock `test_uploader.py` fails on connect2xnor's pass-2 design** ("uploaded twice"). The stock
   test wasn't updated for the pass-2 firehose re-walk. Decide whether the stock test needs updating
   or the pass-2 selection needs to dedupe against pass-1. Pre-existing on connect2xnor.
3. **Full C++ build not validated in this environment** (missing acados/libusb/vaapi system libs).
   `params_pyx.so` (the only C++ the merge affects) built clean. The C++ loggerd/encoderd/pandad/UI
   need a proper device or CI build to confirm — but the merge changed none of that C++.
4. **`ces_xnor.py` has 4 ruff E702 warnings** (pre-existing). Cosmetic.

Nothing in the merge resolution itself is uncertain: params_keys.h and uploader.py were resolved as
explicit unions (kept both sides), and every union-verification grep passed.
