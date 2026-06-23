# 3pnw — compile-verification CI

`3pnw` is the **release branch**, cut from `3pnwdev`. It carries a GitHub Actions workflow that
**compile-checks** the branch on an aarch64 runner — a regression gate, not a prebuilt publisher.

## What the CI does

`.github/workflows/3pnw-release.yaml` runs on **push to `3pnw`** and on **manual dispatch**, on a
GitHub-hosted **ARM runner** (`ubuntu-24.04-arm`). It mirrors the proven `tests.yaml` "build release"
recipe up to the compile, but **skips the device-coupled media stack** and **does not push a prebuilt**:

1. `release/build_stripped.sh` → `release_files.py` selects the shippable file set into `/tmp/3pnw-release`.
2. `./tools/op.sh setup` → toolchain; then `apt install libusb-1.0-0-dev` (the only extra device lib
   the remaining targets link against).
3. `python3 system/manager/build.py` with `SCONS_NO_MEDIA=1` → **compiles openpilot (aarch64)** with
   `scons --no-media`, verifying cereal, common, panda firmware, pandad, the MPC libs, modeld/tinygrad,
   locationd, rednose, the UI, and the cython extensions all build clean.

A green run means the C/C++/cython tree compiles on aarch64 — it catches code regressions (e.g. the
earlier crashes) before they reach the car.

## Why it is a compile check, not a prebuilt (the device-coupling wall)

GitHub ARM runners are generic `aarch64`; the comma 3X is comma's `larch64`. The **media/encoder
stack** — `system/loggerd` (`encoderd`/`loggerd`) and `system/camerad` — links the vendored ffmpeg
wheel, which on a generic aarch64 host references VAAPI/Vulkan **hardware-encode** symbols
(`vaCreateContext`, `vaRenderPicture`, vulkan refs) that have **no off-device link target**. Installing
`libva-dev`/`libdrm-dev`/`libvulkan-dev` does not resolve it because the vendored ffmpeg's link line is
built for the device's HW pipeline, not a generic libva. This is a fundamental `larch64 ≠ aarch64`
coupling, so the CI **skips that stack** (`scons --no-media`, driven by `SCONS_NO_MEDIA=1` in
`system/manager/build.py`; the option is defined in `SConstruct`). On-device (`AGNOS`) builds never set
it and always build the full media stack natively.

## ⚠️ Caveats

- **aarch64 ≠ larch64.** The compiled output is a **compile/integration check**, **not** a runnable or
  device-deployable prebuilt (and the media stack isn't even built). It is **not** pushed anywhere.
- **Panda firmware is built UNSIGNED** in CI (the release signing cert lives only on the device at
  `/data/pandaextra/certs/release`). The device must keep its **matched-set** panda fw and must **not**
  reflash from CI output.
- **LFS / submodules** must resolve in CI (onnx/svg via LFS; opendbc/panda submodules pinned to the
  matched set). A checkout failure here is the first thing to check in the Actions log.

## The authoritative device release (unchanged)

The real, deployable release is still built **on the comma** with `release/build_release.sh`
(needs the device arch + panda cert + the full native media stack), or deployed via the root
surgical-patch toolchain. See the workbench `CLAUDE.md` and `DEVICE-STATE.md`.

## Promoting work into a release

1. Land features on `3pnwdev` (reviewed).
2. Fast-forward / merge `3pnwdev` → `3pnw`.
3. Push `3pnw` → CI compile-checks the branch (green = the tree builds on aarch64).
4. For the car: build/deploy on-device (above), validating against `DEVICE-STATE.md`.
