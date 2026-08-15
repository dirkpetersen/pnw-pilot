# AGNOS19-COMPAT — run 3devpnw (openpilot 0.11.1) on AGNOS 19.6

## Problem

The comma 3X was upgraded to **AGNOS 19.6** (by flashing `xnor-sync-master`, openpilot 0.11.2).
Installing **pnw-pilot `3devpnw`** (openpilot **0.11.1**, which pinned **AGNOS 17.2**) on top of it
**hangs at the comma logo** — boot never finishes.

Two independent breakages, both fixed here:

### 1. Unconditional AGNOS downgrade-flash (fixed by the pin)
`launch_chffrplus.sh:agnos_init` runs `if [ $(< /VERSION) != "$AGNOS_VERSION" ]` → a full 7-partition
AGNOS **downgrade**-flash (no direction / anti-rollback check). With `/VERSION`=19.6 and the old pin
17.2 this fired on every boot. **Fix:** pin `AGNOS_VERSION="19.6"` in `launch_env.sh` and swap
`system/hardware/tici/agnos.json` to the 19.6 partition hashes (schema is identical between 17.2 and
19.6). Versions now match → the flash block is skipped. (commit `agnos: pin 3devpnw to AGNOS 19.6`.)

### 2. AGNOS 19.6's baked venv dropped packages 0.11.1 needs (the real logo-hang)
The device has **no per-tree Python env** — every process runs on the image-baked
`/usr/local/venv`, built at image time from agnos-builder's `userspace/uv/uv.lock`. Between 17.2 and
19.6 that venv shrank **116 → 91 packages**. The ones 0.11.1 still imports:

| dropped pkg | consumer | failure without it |
|---|---|---|
| `bzip2`, `libjpeg`, `libyuv` | `SConstruct:44` `importlib.import_module(...)` | **scons dies before any target** → build fails |
| `casadi` | lat/long MPC codegen (build) + plannerd (runtime) | build fails at MPC step; onroad planner dead |
| `raylib` 5.5.0.2 | the **entire Python UI** (`system/ui`, `selfdrive/ui`) | UI never starts → comma logo forever |
| `pyserial`, `crcmod-plus` | `qcomgpsd` | GPS/mapd dead |
| `xattr` | loggerd → uploader | uploader crash-loop |
| `json-rpc` | athenad | athena/remote dead |

Post-pin the symptom is unchanged (logo forever) because scons now dies at `import bzip2` instead of
the flash loop. (Also present on 19.6 via renamed `comma-deps-*`: capnproto/eigen/ffmpeg/ncurses/
zeromq/zstd, plus psutil/cffi/pillow/sympy/cython/pycapnp 2.1.0 — same on both images. We still
overlay capnproto/eigen/ncurses/zeromq/zstd with 0.11.1's static builds, but take **ffmpeg from the
venv** — see the ffmpeg note below. `/TICI` is still created by 19.6, so the `larch64`/scons-cache
path is fine — verified.)

## Fix: a /data package overlay (survives reflash) + 3 tree edits

Do **not** touch the RO OS image. Stage the exact 17.2-era **aarch64** wheels 0.11.1 expects into
`/data/pnw/agnos19-compat/site-packages` and put that dir on PYTHONPATH for openpilot processes only.
PYTHONPATH precedes the venv site-packages, so the overlay intentionally shadows 19.6's newer builds
(raylib 6.x, shared ffmpeg) with the 5.5 / static builds this tree is written against.

**Overlay contents** (all verified downloadable, no on-device compiling): 8 commaai/dependencies
native-dep wheels (bzip2 1.0.8, capnproto 1.0.1, eigen 3.4.0, libjpeg 3.1.0, libyuv 1922.0,
ncurses 6.5, zeromq 4.3.5, zstd 1.5.6 — static libs+headers), raylib 5.5.0.2
(commaai/raylib-python-cffi release 8 — the exact 17.2 artifact), casadi 3.7.2, crcmod-plus 2.3.1,
xattr 1.3.0, pyserial 3.5, json-rpc 1.15.0.

> **⚠️ ffmpeg is deliberately NOT overlaid** (learned on the first on-car build). The
> commaai/dependencies ffmpeg wheel is the *off-device* build with VAAPI compiled into
> `libavutil.a`; on larch64, `system/loggerd/SConscript` intentionally does **not** link
> libva/libva-drm (the device ffmpeg has no VAAPI), so shadowing it makes `loggerd` fail to link
> with undefined `va*` symbols. The venv's `comma_deps_ffmpeg` (shared, no-VAAPI, device build) is
> the correct one — 0.11.1 imports it from the venv. Same principle didn't bite the other 5
> dual-present deps (capnproto/eigen/ncurses/zeromq/zstd): their static overlay builds match what
> 0.11.1 expects and compile clean.

**Tree edits (this branch):**
1. `launch_chffrplus.sh` — append the overlay to the launcher `PYTHONPATH` (covers build.py, scons,
   manager, all managed processes).
2. `SConstruct` — append the overlay to `env.ENV["PYTHONPATH"]` too; scons **replaces** the env for
   its codegen subprocesses (`python3 lat_mpc.py` → `from casadi import ...`), so the launcher's
   export alone is not inherited there.
3. `launch_chffrplus.sh:agnos_init` — `rm -f /data/scons_cache/config.lock` (mirrors 0.11.2;
   clears a stale SCons CacheDir lock left by the device's earlier 0.11.2 builds).

## Deploy

`/data` (incl. `/data/params`, i.e. your SSH key) survives openpilot reinstalls and AGNOS reflashes;
it is only wiped by a full factory reset. So switch code **in place over SSH** — never via the
installer, which is the reset path that lost the SSH key before.

```bash
# 0. one-time: get SSH back if locked out — reinstall xnor-sync-master (matches 19.6, boots),
#    add your GitHub SSH key in Settings, then come back here.

# 1. build + stage the overlay (does an on-device import test at the end)
COMMA_IP=192.168.13.154 ./scripts/deploy-agnos19-compat.sh      # SSH_PORT=22 home / 22 hotspot@172.20.10.10

# 2. point /data/openpilot at 3devpnw (which carries the pin + the 3 edits).
#    The submodule + prebuilt handling is REQUIRED if step 0 reinstalled xnor-sync-master:
#    a plain reset leaves the 6 submodules at 0.11.2-xnor content/URLs (broken matched-set),
#    and a leftover `prebuilt` file makes launch skip build.py entirely.
ssh comma@$COMMA_IP 'cd /data/openpilot && git fetch origin 3devpnw && git reset --hard FETCH_HEAD && \
  git submodule sync --recursive && git submodule update --init --recursive --force && \
  git lfs pull && rm -f prebuilt'

# 3. persistence guards + restart (see CLAUDE.md deploy section)
ssh comma@$COMMA_IP 'sudo rm -rf /data/safe_staging/finalized; touch /tmp/booted; \
  tmux kill-session -t comma 2>/dev/null; sudo systemctl restart comma'
```

**First boot is a from-scratch native build** (the scons cache holds 0.11.2 objects) — allow
**~10–20 min** at the spinner with visible progress before judging it hung.

### Verify after boot
- `ls -la /TICI /AGNOS` — both present.
- Offline import in the venv: `source /usr/local/venv/bin/activate;
  PYTHONPATH=/data/openpilot:/data/pnw/agnos19-compat/site-packages python3 -c "import bzip2,casadi,pyray,xattr"`.
- manager PIDs stable (not cycling); UI up; no exception in `tmux capture-pane -t comma -p`.
- `du -h selfdrive/modeld/models/*.onnx` — MB-scale, not byte-size LFS pointers (else `git lfs pull`).
- `git submodule status` — no `-` (uninitialized) or `+` (wrong SHA) rows; all six at pnw SHAs.

## Caveats / follow-ups
- **Auto-update:** this must stay merged into `3devpnw` (the one own-car device, already on 19.6), or
  the next update reinstalls the 17.2 pin and re-triggers the downgrade loop.
- **⚠️ Do NOT merge to `3testpnw` (friends channel) yet.** The overlay is a *manual per-device* deploy;
  the pin alone would make a friend's device (on 17.2) *upgrade*-flash to 19.6 (via `agnos_init` or
  `updated.py`'s background flash) and then hang exactly like this one, with no overlay staged. Hold
  until the overlay staging is automated (or done per-device) before this reaches `3testpnw`.
- This is a **compat bridge**, not the endgame. The clean 19.6 answer is a real forward-port of the
  `*2pnw` features onto the 0.11.2 (`xnor-sync-master`) base.
- Inactive A/B slot may hold a part-written 17.2 image from the pre-pin flash attempts; `abctl
  --set_success` on the good slot once booted.
