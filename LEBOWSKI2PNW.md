# LEBOWSKI2PNW — port the commaai-master modeld stack + lebowski driving model

**STATUS: DEPLOYED 2026-07-09** — merged to `4devpnw`, live on the 3X @ **`09fcf8cf2d`**. On-device
build succeeded (~4 min scons incl. driving_tinygrad.pkl + dmonitoring pkls), zero errors after boot,
`DmMode=1` preserved. **NOT yet driven** — the first drive is calibration-grade (CES/VTSC tuned
against the old model). Post-port fixes found during deploy: UsbGpu params registered (crash-loop),
tinygrad pin re-recorded (git add had clobbered update-index), SConscript hw.py dep path
(common/hardware→system/hardware), and OOM during LFS smudge worked around via
GIT_LFS_SKIP_SMUDGE=1 reset + git lfs pull (runbook updated). Gemini review: its 5 findings were all
refuted against this tree; the real bugs above were caught by direct verification + the device build.

## What & why

commaai's post-0.11.2 master carries the **lebowski** driving model (`fa0c6876d3`, 2026-07-01) on a
substantially rebuilt modeld runtime (~40 commits since our 2026-03-14 merge-base): split
vision+policy models merged into a **single combined `driving_supercombo.onnx`**, new build-time
compile pipeline (`compile_modeld.py`, jit-pkl chunking via `common/file_chunker.py`), tinygrad
backend autodetect / no-runtime-compile, split warp, deep-model prereqs. Cherry-picking was not
viable (model swap/revert cycles + the June restructure in the middle) → **snapshot port**: master's
`selfdrive/modeld/` taken as a unit.

## What the port contains (all in `40cd9cb627`)

- `selfdrive/modeld/` fully replaced with master's (new: `compile_modeld.py`, `compile_dm_warp.py`,
  `helpers.py`; gone: `compile_warp.py`, split model files).
- Models (as **git-LFS pointers**, same convention as before — real blobs live on commaai's GitLab
  LFS): `driving_supercombo.onnx` (~61 MB), `big_driving_supercombo.onnx`, and a **new
  `dmonitoring_model.onnx` — the 0.11.1 sleep-prob DM model (= GLARE Layer B), which travels with
  its runtime**.
- `cereal/log.capnp`: `DriverStateV2.DriverData.sleepProb @14` added (the only real schema delta;
  master's other diffs are wire-compatible deprecated-group refactors, deliberately skipped).
- `common/file_chunker.py` (new dep), `openpilot/cereal` symlink (master imports `openpilot.cereal.*`).
- `tinygrad_repo` submodule pin bumped `2f55005a → e6fbede157`.
- pnw mod re-applied: `DH = DesireHelper(CP)` (auto2pnw nudgeless gate). It is the ONLY pnw
  modification modeld carried.

Verified on host: all files py_compile + ruff clean; every import resolves in our layout;
`common/transformations/{model,camera}.py` byte-identical to master (no port needed); all `modelV2`
fields our consumers read (CES/VTSC/planner: `action`, `acceleration`, `meta`, `position`,
`velocity`) are still filled by the new `fill_model_msg`; the `drivingModelData` compression is
modeld-internal.

## Deploy runbook (when ready — NOT yet)

1. Parked, `IsOnroad=0`. `git fetch && git reset --hard <sha>` as usual, **plus two extra steps**
   `reset --hard` does NOT do:
   - `git submodule update --init tinygrad_repo` (new pin; needs GitHub reachability)
   - `git lfs pull` (fetches the ~61 MB supercombo + new DM model from commaai's GitLab LFS; device
     has git-lfs 3.4.1 and the matching `.lfsconfig` — verified 2026-07-08)
2. Full build-on-boot (`prebuilt` absent): **`compile_modeld` runs at BUILD time** producing the jit
   pkls — expect a LONG first build (model compile on-device). Watch the tmux pane.
3. Verify: modeld + dmonitoringmodeld up and publishing (`modelV2`, `driverStateV2`), no frame-drop
   alerts, UI model overlay sane; DM: `sleepProb` present in driverStateV2.
4. **First drive = calibration-grade caution.** CES/VTSC were tuned against the old model's
   curvature/velocity outputs (A_LAT 2.5, MAP_SPEED_SCALE 1.8, accel ramp) — expect a retune pass
   (`drives/` analysis) before trusting curve behavior. DM: the new model's probs may shift the
   glare Layer-C thresholds' effective behavior — watch `docs/GLARE.md` symptoms.

## Rollback

`git reset --hard 19bedc17c5` + restart (old models are separate LFS objects already smudged on
device; the old tree needs no LFS pull). tinygrad pin reverts with the reset; run
`git submodule update` to re-sync the old pin's content if scons complains.

## Open items

- [ ] On-device build + parked smoke test (modeld publishing, timing/lag within limits on the 3X —
      master targets comma four-class hardware too; watch `modelExecutionTime`).
- [ ] First drive + CES/VTSC behavior check; retune if needed.
- [ ] Decide whether to keep the new DM model active or pin the old `dmonitoring_model.onnx` OID
      while evaluating (they're separable at the LFS-pointer level).
- [ ] msgq pin was NOT bumped (master moved `ed277774 → 1d422a23`); build/runtime will tell if the
      new modeld actually needs it — bump only on evidence.

## See also
- `UPSTREAM2PNW.md` — the Tier-1/2 picks that preceded this (merged to `4devpnw`); lebowski was
  excluded there as "its own effort" — this is that effort.
- `GLARE.md` — Layer B arrives via the bundled DM model.
- `VTSC.md` / `CES_I90.md` — the tunes that must be re-validated against the new model.
