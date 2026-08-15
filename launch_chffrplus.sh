#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"

source "$DIR/launch_env.sh"

function agnos_init {
  # agnos19-compat: clear a stale SCons CacheDir lock (0.11.2 does this in agnos_init).
  # The device ran 0.11.2 builds before pnw; a killed build can leave this lock and
  # stall `scons --cache-populate`.
  rm -f /data/scons_cache/config.lock

  # TODO: move this to agnos
  sudo rm -f /data/etc/NetworkManager/system-connections/*.nmmeta

  # set success flag for current boot slot
  sudo abctl --set_success

  # TODO: do this without udev in AGNOS
  # udev does this, but sometimes we startup faster
  sudo chgrp gpu /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0
  sudo chmod 660 /dev/adsprpc-smd /dev/ion /dev/kgsl-3d0

  # Check if AGNOS update is required
  if [ $(< /VERSION) != "$AGNOS_VERSION" ]; then
    AGNOS_PY="$DIR/system/hardware/tici/agnos.py"
    MANIFEST="$DIR/system/hardware/tici/agnos.json"
    if $AGNOS_PY --verify $MANIFEST; then
      sudo reboot
    fi
    $DIR/system/hardware/tici/updater $AGNOS_PY $MANIFEST
  fi
}

function launch {
  # Remove orphaned git lock if it exists on boot
  [ -f "$DIR/.git/index.lock" ] && rm -f $DIR/.git/index.lock

  # Check to see if there's a valid overlay-based update available. Conditions
  # are as follows:
  #
  # 1. The DIR init file has to exist, with a newer modtime than anything in
  #    the DIR Git repo. This checks for local development work or the user
  #    switching branches/forks, which should not be overwritten.
  # 2. The FINALIZED consistent file has to exist, indicating there's an update
  #    that completed successfully and synced to disk.

  if [ -f "${DIR}/.overlay_init" ]; then
    find ${DIR}/.git -newer ${DIR}/.overlay_init | grep -q '.' 2> /dev/null
    if [ $? -eq 0 ]; then
      echo "${DIR} has been modified, skipping overlay update installation"
    else
      if [ -f "${STAGING_ROOT}/finalized/.overlay_consistent" ]; then
        if [ ! -d /data/safe_staging/old_openpilot ]; then
          echo "Valid overlay update found, installing"
          LAUNCHER_LOCATION="${BASH_SOURCE[0]}"

          mv $DIR /data/safe_staging/old_openpilot
          mv "${STAGING_ROOT}/finalized" $DIR
          cd $DIR

          echo "Restarting launch script ${LAUNCHER_LOCATION}"
          unset AGNOS_VERSION
          exec "${LAUNCHER_LOCATION}"
        else
          echo "openpilot backup found, not updating"
          # TODO: restore backup? This means the updater didn't start after swapping
        fi
      fi
    fi
  fi

  # handle pythonpath
  ln -sfn $(pwd) /data/pythonpath
  # agnos19-compat: overlay of 17.2-era wheels this 0.11.1 tree needs but AGNOS 19.6's
  # baked venv dropped (bzip2/libjpeg/libyuv/casadi/raylib/... — see AGNOS19-COMPAT.md).
  # Overlay lives in /data (survives reflash). PYTHONPATH is searched before the venv
  # site-packages, so the overlay intentionally shadows 19.6's newer builds (raylib 6.x,
  # shared ffmpeg) with the 5.5 / static builds this 0.11.1 tree is written against.
  export PYTHONPATH="$PWD:/data/pnw/agnos19-compat/site-packages"

  # agnos19-compat: SELF-PROVISION the overlay on a from-scratch install (a factory reset wipes
  # /data, and there is no SSH to stage it by hand). The overlay tarball is bundled in the repo
  # via git-LFS, so the installer's clone already carries it; extract it here — BEFORE build.py
  # imports bzip2/casadi/pyray — if /data has no overlay yet. No network, no pip. See AGNOS19-COMPAT.md.
  if [ ! -d /data/pnw/agnos19-compat/site-packages ] && [ -f "$DIR/agnos19-compat/overlay.tar.zst" ]; then
    echo "agnos19-compat: staging venv overlay from bundled tarball..."
    mkdir -p /data/pnw/agnos19-compat
    tar --zstd -xf "$DIR/agnos19-compat/overlay.tar.zst" -C /data/pnw/agnos19-compat 2>/dev/null \
      || zstd -dc "$DIR/agnos19-compat/overlay.tar.zst" | tar -xf - -C /data/pnw/agnos19-compat
  fi

  # hardware specific init
  if [ -f /AGNOS ]; then
    agnos_init
  fi

  # write tmux scrollback to a file
  tmux capture-pane -pq -S-1000 > /tmp/launch_log

  # start manager
  cd system/manager
  if [ ! -f $DIR/prebuilt ]; then
    ./build.py
  fi
  ./manager.py

  # if broken, keep on screen error
  while true; do sleep 1; done
}

launch
