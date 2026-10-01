"""tailscale2pnw: pinned, sha256-verified install of the Tailscale static arm64 release.

Binaries live in /data/pnw/tailscale -- OUTSIDE /data/openpilot, because the updater's `git clean` deletes
untracked files inside the repo (the mapd lesson, 2026-06-29). The release is a version + sha256 constant in
this file: a bump is a reviewed commit, never "latest". The tarball is hashed WHILE it downloads and nothing is
extracted unless the digest matches. Every failure raises InstallError with the reason; callers publish it.
"""
from __future__ import annotations

import hashlib
import os
import platform
import tarfile
import urllib.request
from collections.abc import Callable

from openpilot.common.swaglog import cloudlog

# Pin. sha256 = the arm64 tarball, taken from https://pkgs.tailscale.com/stable/tailscale_1.102.4_arm64.tgz.sha256
# and re-computed locally from the downloaded file on 2026-09-30 (both agree).
TAILSCALE_VERSION = "1.102.4"
TAILSCALE_SHA256 = "9dd1e6a592a014bbaea0103167ffe299adeda4ba14e078ce9c2895364f6c4c3f"
TAILSCALE_URL = f"https://pkgs.tailscale.com/stable/tailscale_{TAILSCALE_VERSION}_arm64.tgz"

INSTALL_DIR = "/data/pnw/tailscale"
BINARIES = ("tailscaled", "tailscale")
MARKER = "INSTALLED"  # holds "<version> <sha256>", written LAST, only after a verified install

CHUNK = 1 << 20


class InstallError(Exception):
  """Install failed. `permanent` = retrying the same pin cannot help (hash mismatch, wrong CPU)."""

  def __init__(self, reason: str, permanent: bool = False):
    super().__init__(reason)
    self.permanent = permanent


def _marker_text() -> str:
  return f"{TAILSCALE_VERSION} {TAILSCALE_SHA256}"


def is_installed(install_dir: str = INSTALL_DIR) -> bool:
  try:
    with open(os.path.join(install_dir, MARKER)) as f:
      if f.read().strip() != _marker_text():
        return False
  except FileNotFoundError:
    return False  # not installed yet: the normal first-run answer, not an error
  return all(os.access(os.path.join(install_dir, b), os.X_OK) for b in BINARIES)


def _download(url: str, dest: str) -> str:
  """Stream url -> dest, returning the sha256 hex of exactly the bytes written."""
  h = hashlib.sha256()
  with urllib.request.urlopen(url, timeout=60) as resp, open(dest, "wb") as out:
    for chunk in iter(lambda: resp.read(CHUNK), b""):
      h.update(chunk)
      out.write(chunk)
  return h.hexdigest()


def ensure_installed(install_dir: str = INSTALL_DIR, fetch: Callable[[str, str], str] = _download,
                     machine: Callable[[], str] = platform.machine) -> None:
  """Make `install_dir` hold the pinned binaries. No-op when already installed. Raises InstallError."""
  if is_installed(install_dir):
    return
  arch = machine()
  if arch not in ("aarch64", "arm64"):
    raise InstallError(f"unsupported CPU {arch!r} (the pinned release is arm64)", permanent=True)

  try:
    os.makedirs(install_dir, mode=0o700, exist_ok=True)
  except OSError as e:
    raise InstallError(f"cannot create {install_dir}: {e}") from e

  tgz = os.path.join(install_dir, ".download.tgz")
  cloudlog.warning(f"tailscale: downloading {TAILSCALE_URL}")
  try:
    try:
      got = fetch(TAILSCALE_URL, tgz)
    except (OSError, ValueError) as e:  # URLError/HTTPError/TimeoutError/socket errors are all OSError
      raise InstallError(f"download failed: {e}") from e
    if got != TAILSCALE_SHA256:
      raise InstallError(f"sha256 mismatch (got {got[:12]}..., pinned {TAILSCALE_SHA256[:12]}...); refusing to install",
                         permanent=True)
    _extract(tgz, install_dir)
  finally:
    try:
      os.remove(tgz)
    except FileNotFoundError:
      pass  # already gone: nothing to clean up
    except OSError as e:
      cloudlog.error(f"tailscale: could not remove {tgz}: {e}")

  try:
    with open(os.path.join(install_dir, MARKER), "w") as f:
      f.write(_marker_text() + "\n")
  except OSError as e:
    raise InstallError(f"cannot write install marker: {e}") from e
  cloudlog.warning(f"tailscale: installed {TAILSCALE_VERSION} into {install_dir}")


def _extract(tgz: str, install_dir: str) -> None:
  """Extract ONLY the two binaries (by exact basename, regular files), via temp name + atomic rename."""
  found = set()
  try:
    with tarfile.open(tgz, "r:gz") as tf:
      for m in tf:
        name = os.path.basename(m.name)
        if not m.isfile() or name not in BINARIES or name in found:
          continue
        src = tf.extractfile(m)
        if src is None:
          raise InstallError(f"tarball member {m.name} is unreadable")
        tmp = os.path.join(install_dir, f".{name}.new")
        try:
          with open(tmp, "wb") as out:
            while chunk := src.read(CHUNK):
              out.write(chunk)
          os.chmod(tmp, 0o755)
          os.replace(tmp, os.path.join(install_dir, name))
        finally:
          if os.path.exists(tmp):  # an interrupted write must not leave a half-written file behind
            os.remove(tmp)
        found.add(name)
  except (tarfile.TarError, OSError, EOFError) as e:
    raise InstallError(f"extract failed: {e}") from e
  missing = set(BINARIES) - found
  if missing:
    raise InstallError(f"tarball is missing {sorted(missing)}", permanent=True)
