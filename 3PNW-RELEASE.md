# 3pnw — release branch + CI

`3pnw` is the **release branch**, cut from `3pnwdev`. It carries a GitHub Actions workflow that
builds the branch and publishes a prebuilt to **`3pnw-release`**.

## What the CI does

`.github/workflows/3pnw-release.yaml` runs on **push to `3pnw`** and on **manual dispatch**, on a
GitHub-hosted **ARM runner** (`ubuntu-24.04-arm`). It mirrors the proven `tests.yaml` "build release"
recipe and then packs + pushes:

1. `release/build_stripped.sh` → `release_files.py` selects the shippable file set into `/tmp/3pnw-release`.
2. `./tools/op.sh setup` → toolchain.
3. `python3 system/manager/build.py` → **compiles openpilot (aarch64)**.
4. `release/pack_3pnw.sh` → strip intermediates, keep compiled artifacts, `touch prebuilt`,
   force-push the tree to **`3pnw-release`**.

## ⚠️ Caveats — read before trusting a build

This is **experimental** and does **not** replace the on-device release:

- **aarch64 ≠ larch64.** GitHub ARM runners are generic `aarch64`; the comma 3X is comma's `larch64`
  (different vendored `third_party` libs and SCons flags). A CI prebuilt is a **compile/integration
  check**, **not guaranteed to run on the device.**
- **Panda firmware is unsigned.** The release signing cert lives only on the device
  (`/data/pandaextra/certs/release`). The device must keep its **matched-set** panda fw
  (opendbc `7ddf6559` / panda `56920ec6`) and must **not** reflash from `3pnw-release`.
- **LFS / submodules** must resolve in CI (onnx/svg via LFS; opendbc/panda submodules pinned to the
  matched set). A checkout failure here is the first thing to check in the Actions log.

## The authoritative device release (unchanged)

The real, deployable release is still built **on the comma** with `release/build_release.sh`
(needs the device arch + panda cert), or deployed via the root surgical-patch toolchain. See the
workbench `CLAUDE.md` and `DEVICE-STATE.md`.

## Promoting work into a release

1. Land features on `3pnwdev` (reviewed).
2. Fast-forward / merge `3pnwdev` → `3pnw`.
3. Push `3pnw` → CI builds and publishes `3pnw-release`.
4. For the car: build/deploy on-device (above), validating against `DEVICE-STATE.md`.
