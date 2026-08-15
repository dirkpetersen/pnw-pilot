#!/usr/bin/env bash
#
# deploy-agnos19-compat.sh — build + deploy the AGNOS-19.6 compatibility overlay.
#
# Why: openpilot 0.11.1 (this pnw tree) runs on AGNOS's image-baked /usr/local/venv.
# AGNOS 19.6's venv dropped ~a dozen packages 0.11.1 needs (bzip2/libjpeg/libyuv make
# SConstruct's `importlib.import_module` fail before any target; raylib 5.5 is the whole
# Python UI; casadi is MPC codegen; xattr/pyserial/crcmod-plus/json-rpc are runtime).
# We stage the exact 17.2-era aarch64 wheels into /data (survives reflash) and put that
# dir on PYTHONPATH (done in launch_chffrplus.sh + SConstruct). See AGNOS19-COMPAT.md.
#
# Usage:  COMMA_IP=192.168.13.154 ./scripts/deploy-agnos19-compat.sh          # build + deploy + verify
#         ./scripts/deploy-agnos19-compat.sh --build-only                     # just assemble locally
#         OUTDIR=/some/dir ./scripts/deploy-agnos19-compat.sh --build-only    # assemble into OUTDIR (env var)
#
# Idempotent. Only rsyncs; never touches /data/params (your SSH key is safe).
set -euo pipefail

COMMA_IP="${COMMA_IP:-192.168.13.154}"
SSH_PORT="${SSH_PORT:-22}"
DEVICE_DIR="/data/pnw/agnos19-compat"
OUT="${OUTDIR:-$(mktemp -d)}/agnos19-overlay"
SP="$OUT/site-packages"
WHEELS="$OUT/wheels"

URLS=(
  # commaai/dependencies — the 9 SConstruct native-dep modules (static libs + headers)
  "https://github.com/commaai/dependencies/releases/download/bzip2/v1.0.8/bzip2-1.0.8-py3-none-linux_aarch64.whl"
  "https://github.com/commaai/dependencies/releases/download/capnproto/v1.0.1/capnproto-1.0.1-py3-none-linux_aarch64.whl"
  "https://github.com/commaai/dependencies/releases/download/eigen/v3.4.0/eigen-3.4.0-py3-none-linux_aarch64.whl"
  # NOTE: ffmpeg is deliberately NOT overlaid. The commaai/dependencies ffmpeg wheel is the
  # OFF-DEVICE build with VAAPI compiled into libavutil.a; on larch64 loggerd/SConscript does
  # not link libva/libva-drm, so shadowing the venv's device ffmpeg breaks the loggerd link
  # (undefined va* symbols). The venv's comma_deps_ffmpeg (shared, no-VAAPI, device build) is
  # correct — let 0.11.1 import it from the venv. Do NOT re-add ffmpeg here.
  "https://github.com/commaai/dependencies/releases/download/libjpeg/v3.1.0/libjpeg-3.1.0-py3-none-linux_aarch64.whl"
  "https://github.com/commaai/dependencies/releases/download/libyuv/v1922.0/libyuv-1922.0-py3-none-linux_aarch64.whl"
  "https://github.com/commaai/dependencies/releases/download/ncurses/v6.5/ncurses-6.5-py3-none-linux_aarch64.whl"
  "https://github.com/commaai/dependencies/releases/download/zeromq/v4.3.5/zeromq-4.3.5-py3-none-linux_aarch64.whl"
  "https://github.com/commaai/dependencies/releases/download/zstd/v1.5.6/zstd-1.5.6-py3-none-linux_aarch64.whl"
  # raylib 5.5.0.2 — the EXACT 17.2 image artifact (entire Python UI)
  "https://github.com/commaai/raylib-python-cffi/releases/download/8/raylib-5.5.0.2-cp312-cp312-linux_aarch64.whl"
  # PyPI — MPC codegen + runtime deps (aarch64 / cp312 / abi3, manylinux2014 = glibc 2.17 OK on Ubuntu 24.04)
  "https://files.pythonhosted.org/packages/f3/24/4cf05469ddf8544da5e92f359f96d716a97e7482999f085a632bc4ef344a/casadi-3.7.2-cp312-none-manylinux2014_aarch64.whl"
  "https://files.pythonhosted.org/packages/bb/e8/f5d66778b5a1bff915807016561a02b5cebf6b3840fb8a2be40bbb0c8575/crcmod_plus-2.3.1-cp311-abi3-manylinux2014_aarch64.manylinux_2_17_aarch64.manylinux_2_28_aarch64.whl"
  "https://files.pythonhosted.org/packages/a9/24/cc350bcdbed006dfcc6ade0ac817693b8b3d4b2787f20e427fd0697042e4/xattr-1.3.0-cp312-cp312-manylinux2014_aarch64.manylinux_2_17_aarch64.whl"
  "https://files.pythonhosted.org/packages/07/bc/587a445451b253b285629263eb51c2d8e9bcea4fc97826266d186f96f558/pyserial-3.5-py2.py3-none-any.whl"
  "https://files.pythonhosted.org/packages/94/9e/820c4b086ad01ba7d77369fb8b11470a01fac9b4977f02e18659cf378b6b/json_rpc-1.15.0-py2.py3-none-any.whl"
)

echo "== assembling overlay in $SP =="
rm -rf "$OUT"; mkdir -p "$SP" "$WHEELS"
for u in "${URLS[@]}"; do
  f="$WHEELS/$(basename "$u")"
  echo ">> $(basename "$u")"
  curl -fsSL "$u" -o "$f"
  unzip -oq "$f" -d "$SP"
done
# Replicate the wheel .data scheme (what pip does): hoist purelib/platlib to the root
shopt -s nullglob
for d in "$SP"/*.data; do
  for cat in purelib platlib; do [ -d "$d/$cat" ] && cp -a "$d/$cat/." "$SP/"; done
  rm -rf "$d"
done
rm -rf "$SP"/*.dist-info "$SP/dummy.txt"

# Sanity: every module SConstruct/UI/MPC/runtime needs must resolve at top level
missing=0
# ffmpeg intentionally excluded (comes from the venv — see NOTE above)
for m in bzip2 capnproto eigen libjpeg libyuv ncurses zeromq zstd casadi crcmod jsonrpc pyray raylib serial xattr; do
  [ -e "$SP/$m" ] || [ -e "$SP/$m.py" ] || { echo "MISSING module: $m"; missing=1; }
done
[ "$missing" = 0 ] || { echo "!! overlay incomplete — aborting"; exit 1; }
echo "== overlay OK ($(du -sh "$SP" | cut -f1), $(find "$SP" -name '*.so' | wc -l) .so) =="

if [ "${1:-}" = "--build-only" ]; then echo "built at: $SP"; exit 0; fi

echo "== deploying to comma@$COMMA_IP:$DEVICE_DIR (port $SSH_PORT) =="
ssh -p "$SSH_PORT" "comma@$COMMA_IP" "mkdir -p $DEVICE_DIR"
rsync -az --delete -e "ssh -p $SSH_PORT" "$SP/" "comma@$COMMA_IP:$DEVICE_DIR/site-packages/"

echo "== on-device import verification (in the device venv, with the overlay on PYTHONPATH) =="
ssh -p "$SSH_PORT" "comma@$COMMA_IP" bash -s <<REMOTE
set -e
source /usr/local/venv/bin/activate
export PYTHONPATH="/data/openpilot:$DEVICE_DIR/site-packages"
python3 - <<'PY'
import importlib
mods = ['bzip2','capnproto','eigen','ffmpeg','libjpeg','libyuv','ncurses','zeromq','zstd',
        'casadi','crcmod','jsonrpc','pyray','raylib','serial','xattr']
bad=[]
for m in mods:
    try: importlib.import_module(m)
    except Exception as e: bad.append(f"{m}: {type(e).__name__}: {e}")
if bad:
    print("IMPORT FAILURES:"); [print(" ", b) for b in bad]; raise SystemExit(1)
print(f"all {len(mods)} overlay modules import cleanly in the device venv")
PY
REMOTE
echo "== done. Overlay live at $DEVICE_DIR/site-packages =="
echo "   Next: on-device \`cd /data/openpilot && git fetch && git reset --hard origin/3devpnw\`,"
echo "   then rm -f /data/safe_staging/finalized; touch /tmp/booted; tmux kill-session -t comma; sudo systemctl restart comma"
