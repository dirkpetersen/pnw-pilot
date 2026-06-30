#!/usr/bin/env python3
"""
mapd2pnw: download-at-launch installer for the pfeiferj `mapd` binary.

The mapd binary is ~20 MB. Vendoring it in git bloats every clone and every
on-device update, so PNW does NOT commit it. Instead the release is pinned in
`mapd_release.json` at the repo root (url + version + sha256 + install_path) and
this module fetches it once, verifying the sha256, into `install_path`. It is
idempotent: a no-op when the pinned binary is already installed and valid.

  python3 -m openpilot.system.mapd.installer            # ensure installed
  python3 -m openpilot.system.mapd.installer --check     # report status, no download

This lives under system/ (which is symlinked into the `openpilot` package), so it
imports as `openpilot.system.mapd.installer` with no extra package wiring — unlike
a top-level package, which would need its own symlink into openpilot/.
"""
from __future__ import annotations

import hashlib
import json
import os
import shutil
import urllib.request

# Resolve paths from this file's REAL location, not BASEDIR. On the device, system/
# is a symlink into the openpilot package, so BASEDIR points into the symlinked tree
# where the root-level config (mapd_release.json) isn't reachable. realpath() collapses
# the symlink to the flat repo root, which works in both the dev checkout and on-device.
# system/mapd/installer.py -> up 3 dirs = repo root.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
MAPD_RELEASE_CONFIG = os.path.join(REPO_ROOT, "mapd_release.json")


def load_release() -> dict:
  with open(MAPD_RELEASE_CONFIG) as f:
    return json.load(f)


# Legacy in-tree install path (pre-2026-06-29). The binary is UNTRACKED, so an auto-update's
# `git clean -xdff` DELETES it from the working tree, forcing a flaky boot re-download (DNS/clock
# not ready before NTP) and leaving the map dead until a lucky retry. See the 2026-06-29 incident.
def _legacy_in_tree() -> str:
  try:
    return os.path.join(REPO_ROOT, load_release()["install_path"])
  except Exception:
    return os.path.join(REPO_ROOT, "selfdrive", "mapd")


# PERSISTENT install location OUTSIDE the git working tree so `git clean` can never delete it. On the
# device /data is persistent and survives every update/reboot; off-device (dev/CI, no writable /data)
# fall back to the in-tree path — mapd doesn't run there anyway (gated on the binary existing). Resolved
# once at import so callers (process_config's should_run + exec path) reference a single MAPD_BINARY.
MAPD_PERSIST_DIR = "/data/mapd"
if os.path.isdir("/data") and os.access("/data", os.W_OK):
  MAPD_BINARY = os.path.join(MAPD_PERSIST_DIR, "mapd")
else:
  MAPD_BINARY = _legacy_in_tree()


def _sha256(path: str) -> str:
  h = hashlib.sha256()
  with open(path, "rb") as f:
    for chunk in iter(lambda: f.read(1 << 20), b""):
      h.update(chunk)
  return h.hexdigest()


def is_installed(rel: dict | None = None) -> bool:
  rel = rel or load_release()
  dest = MAPD_BINARY
  if not os.path.exists(dest) or not os.access(dest, os.X_OK):
    return False
  expected = rel.get("sha256")
  return (not expected) or _sha256(dest) == expected


def ensure_mapd(retries: int = 3) -> str:
  """Download + install the pinned mapd binary if missing/stale. Returns its path.

  Atomic (download to a temp file in the same dir, sha-verify, chmod, rename) so a
  killed download or dropped link never leaves a half-written executable in place.
  """
  rel = load_release()
  dest = MAPD_BINARY
  expected = rel.get("sha256")

  if is_installed(rel):
    return dest

  os.makedirs(os.path.dirname(dest), exist_ok=True)

  # Migration / no-download path: if a valid binary already exists at the legacy in-tree location
  # (first boot after this change, or a fresh download landed there), copy it to the persistent dir
  # instead of re-downloading. This is what lets the fix survive even when the boot network is down.
  legacy = _legacy_in_tree()
  if legacy != dest and os.path.exists(legacy) and os.access(legacy, os.X_OK):
    try:
      if (not expected) or _sha256(legacy) == expected:
        # atomic: copy to a temp in the same dir, chmod, then os.replace — so the manager never
        # execs a half-written file (ETXTBSY / partial binary) and a hard-reboot mid-copy can't
        # leave a corrupt dest that is_installed() would falsely accept when the release has no sha.
        tmp = dest + ".migrate"
        shutil.copy2(legacy, tmp)
        os.chmod(tmp, 0o755)
        os.replace(tmp, dest)
        print(f"mapd installer: copied existing binary {legacy} -> {dest} (no download)")
        return dest
    except Exception as e:
      print(f"mapd installer: migrate from {legacy} failed ({e}); will download")
      try:
        if os.path.exists(dest + ".migrate"):
          os.remove(dest + ".migrate")
      except OSError:
        pass

  url = rel["url"]
  # Static temp name (not mkstemp): a download hard-killed mid-flight (ignition off,
  # reboot) leaves at most ONE stale temp that the next run simply overwrites — no
  # accumulating 20 MB orphans. os.replace() onto dest is still atomic.
  tmp = dest + ".download"
  last_err: Exception | None = None
  for attempt in range(1, retries + 1):
    try:
      req = urllib.request.Request(url, headers={"User-Agent": "pnw-mapd-installer"})
      with urllib.request.urlopen(req, timeout=60) as r, open(tmp, "wb") as out:
        while True:
          buf = r.read(1 << 20)
          if not buf:
            break
          out.write(buf)
      got = _sha256(tmp)
      if expected and got != expected:
        raise ValueError(f"sha256 mismatch for mapd {rel.get('version')}: expected {expected}, got {got}")
      os.chmod(tmp, 0o755)
      os.replace(tmp, dest)
      print(f"mapd installer: installed {rel.get('version')} -> {dest} (sha256 {got[:12]}…)")
      return dest
    except Exception as e:
      last_err = e
      print(f"mapd installer: attempt {attempt}/{retries} failed: {e}")
  if os.path.exists(tmp):
    try:
      os.remove(tmp)
    except OSError:
      pass
  raise RuntimeError(f"mapd installer: failed to download {url}: {last_err}")


def main() -> None:
  import sys
  rel = load_release()
  if "--check" in sys.argv:
    print(f"mapd {rel.get('version')} install_path={rel['install_path']} installed={is_installed(rel)} dest={MAPD_BINARY}")
    return
  ensure_mapd()


if __name__ == "__main__":
  main()
