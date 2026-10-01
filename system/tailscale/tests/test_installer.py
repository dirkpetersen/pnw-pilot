"""tailscale2pnw installer: sha256 pin enforced BEFORE anything is extracted, every failure is an InstallError."""
import hashlib
import io
import os
import pathlib
import re
import tarfile

import pytest

from openpilot.system.tailscale import installer


def _tgz(path, members):
  with tarfile.open(path, "w:gz") as tf:
    for name, data in members.items():
      ti = tarfile.TarInfo(name)
      ti.size = len(data)
      ti.mode = 0o644
      tf.addfile(ti, io.BytesIO(data))


def _digest(path):
  return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


@pytest.fixture
def pinned(tmp_path, monkeypatch):
  """A fake release whose digest is pinned, plus a fetch that 'downloads' it."""
  src = tmp_path / "release.tgz"
  _tgz(src, {"tailscale_x_arm64/tailscaled": b"daemon", "tailscale_x_arm64/tailscale": b"cli",
             "tailscale_x_arm64/systemd/tailscaled.service": b"unit"})
  monkeypatch.setattr(installer, "TAILSCALE_SHA256", _digest(src))
  calls = []

  def fetch(url, dest):
    calls.append(url)
    pathlib.Path(dest).write_bytes(src.read_bytes())
    return _digest(dest)
  return src, fetch, calls, str(tmp_path / "ts")


def test_pin_shape():
  assert re.fullmatch(r"[0-9a-f]{64}", installer.TAILSCALE_SHA256)
  assert installer.TAILSCALE_VERSION in installer.TAILSCALE_URL and installer.TAILSCALE_URL.endswith("_arm64.tgz")
  assert installer.INSTALL_DIR.startswith("/data/pnw/") and not installer.INSTALL_DIR.startswith("/data/openpilot")


def test_install_ok_extracts_only_the_two_binaries(pinned):
  _, fetch, calls, d = pinned
  installer.ensure_installed(d, fetch=fetch, machine=lambda: "aarch64")
  assert calls == [installer.TAILSCALE_URL]
  assert sorted(os.listdir(d)) == sorted([*installer.BINARIES, installer.MARKER])  # no tmp, no systemd files
  for b in installer.BINARIES:
    assert os.access(os.path.join(d, b), os.X_OK)
  assert installer.is_installed(d)


def test_sha256_mismatch_refuses_and_extracts_nothing(pinned, monkeypatch):
  """Mutation: drop the digest comparison -> this fails (binaries get installed)."""
  _, fetch, _, d = pinned
  monkeypatch.setattr(installer, "TAILSCALE_SHA256", "0" * 64)
  with pytest.raises(installer.InstallError, match="sha256 mismatch") as ei:
    installer.ensure_installed(d, fetch=fetch, machine=lambda: "aarch64")
  assert ei.value.permanent
  assert not installer.is_installed(d)
  assert sorted(os.listdir(d)) == []  # downloaded tarball removed, nothing extracted, no marker


def test_download_error_is_reported_and_retryable(pinned):
  _, _, _, d = pinned

  def boom(url, dest):
    raise OSError("network unreachable")
  with pytest.raises(installer.InstallError, match="download failed: network unreachable") as ei:
    installer.ensure_installed(d, fetch=boom, machine=lambda: "aarch64")
  assert not ei.value.permanent


def test_wrong_cpu_refused(pinned):
  _, fetch, calls, d = pinned
  with pytest.raises(installer.InstallError, match="unsupported CPU") as ei:
    installer.ensure_installed(d, fetch=fetch, machine=lambda: "x86_64")
  assert ei.value.permanent and calls == []


def test_already_installed_is_a_noop(pinned):
  _, fetch, calls, d = pinned
  installer.ensure_installed(d, fetch=fetch, machine=lambda: "aarch64")
  installer.ensure_installed(d, fetch=fetch, machine=lambda: "aarch64")
  assert len(calls) == 1


def test_pin_bump_reinstalls(pinned, monkeypatch):
  _, fetch, calls, d = pinned
  installer.ensure_installed(d, fetch=fetch, machine=lambda: "aarch64")
  monkeypatch.setattr(installer, "TAILSCALE_VERSION", "9.9.9")
  assert not installer.is_installed(d)
  installer.ensure_installed(d, fetch=fetch, machine=lambda: "aarch64")
  assert len(calls) == 2


def test_tarball_missing_a_binary_is_an_error(tmp_path, monkeypatch):
  src = tmp_path / "bad.tgz"
  _tgz(src, {"x/tailscaled": b"daemon"})
  monkeypatch.setattr(installer, "TAILSCALE_SHA256", _digest(src))

  def fetch(url, dest):
    pathlib.Path(dest).write_bytes(src.read_bytes())
    return _digest(dest)
  with pytest.raises(installer.InstallError, match="missing"):
    installer.ensure_installed(str(tmp_path / "ts"), fetch=fetch, machine=lambda: "aarch64")
  assert not installer.is_installed(str(tmp_path / "ts"))


def test_corrupt_tarball_with_matching_hash_is_an_error(tmp_path, monkeypatch):
  src = tmp_path / "junk.tgz"
  src.write_bytes(b"not a tarball")
  monkeypatch.setattr(installer, "TAILSCALE_SHA256", _digest(src))

  def fetch(url, dest):
    pathlib.Path(dest).write_bytes(b"not a tarball")
    return _digest(dest)
  with pytest.raises(installer.InstallError, match="extract failed"):
    installer.ensure_installed(str(tmp_path / "ts"), fetch=fetch, machine=lambda: "aarch64")
