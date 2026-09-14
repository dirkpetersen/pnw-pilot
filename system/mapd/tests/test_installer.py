"""mapdlog2pnw: ensure_mapd() must surface an ignored pin bump through cloudlog, not just print().

manager.py runs ensure_mapd() in a SEPARATE subprocess (system/manager/manager.py's _install_mapd)
and never reads that subprocess's stdout -- the only cloudlog line manager itself emits for this
case is the generic "mapd installer: binary present", which is true but silently hides that
/data/mapd/.override is masking an ignored pin bump. Before this fix the only place the real warning
went was print(), which lands in the subprocess's stdout and nowhere else.

This test drives ensure_mapd() directly against a fake persistent dir + release file and asserts
cloudlog.warning fires exactly when override_shadows_pin() is True (override active AND the
installed binary's sha256 does not match the pin), and does not fire otherwise.
"""
import hashlib
import json
import os
import stat

import openpilot.system.mapd.installer as installer


def _write_binary(path: str, content: bytes) -> str:
  with open(path, "wb") as f:
    f.write(content)
  st = os.stat(path)
  os.chmod(path, st.st_mode | stat.S_IEXEC | stat.S_IXGRP | stat.S_IXOTH)
  return hashlib.sha256(content).hexdigest()


def _setup(tmp_path, monkeypatch, *, installed: bytes, pinned: bytes, override: bool):
  """Point every installer module path at a scratch dir and write a fake installed binary +
  a fake mapd_release.json pin. Returns (installed_sha, pinned_sha)."""
  persist_dir = tmp_path / "mapd"
  persist_dir.mkdir()
  binary = persist_dir / "mapd"
  installed_sha = _write_binary(str(binary), installed)
  pinned_sha = hashlib.sha256(pinned).hexdigest()

  release = tmp_path / "mapd_release.json"
  release.write_text(json.dumps({
    "version": "v2.3.1",
    "url": "https://example.invalid/mapd",
    "sha256": pinned_sha,
  }))

  monkeypatch.setattr(installer, "MAPD_PERSIST_DIR", str(persist_dir))
  monkeypatch.setattr(installer, "MAPD_BINARY", str(binary))
  monkeypatch.setattr(installer, "MAPD_OVERRIDE_FLAG", str(persist_dir / ".override"))
  monkeypatch.setattr(installer, "MAPD_RELEASE_CONFIG", str(release))

  if override:
    (persist_dir / ".override").touch()

  return installed_sha, pinned_sha


class TestOverrideShadowsPinCloudlogWarning:
  def test_warns_via_cloudlog_when_override_masks_a_pin_mismatch(self, tmp_path, monkeypatch, mocker):
    _setup(tmp_path, monkeypatch, installed=b"old-binary-bytes", pinned=b"new-pinned-bytes", override=True)

    mock_warning = mocker.patch.object(installer.cloudlog, "warning")
    mock_print = mocker.patch("builtins.print")

    dest = installer.ensure_mapd()

    assert dest == installer.MAPD_BINARY
    # The print() path must be preserved (the ask was to ADD cloudlog, not replace print) ...
    mock_print.assert_called_once()
    # ... AND the same information must now also reach cloudlog, since manager never reads the
    # installer subprocess's stdout.
    mock_warning.assert_called_once()
    (msg,), _kwargs = mock_warning.call_args
    assert "IGNORED" in msg
    assert "v2.3.1" in msg
    assert installer.MAPD_OVERRIDE_FLAG in msg

  def test_no_cloudlog_warning_when_override_active_and_pin_matches(self, tmp_path, monkeypatch, mocker):
    _setup(tmp_path, monkeypatch, installed=b"same-bytes", pinned=b"same-bytes", override=True)

    mock_warning = mocker.patch.object(installer.cloudlog, "warning")
    installer.ensure_mapd()

    mock_warning.assert_not_called()

  def test_no_cloudlog_warning_when_no_override_and_pin_matches(self, tmp_path, monkeypatch, mocker):
    _setup(tmp_path, monkeypatch, installed=b"same-bytes", pinned=b"same-bytes", override=False)

    mock_warning = mocker.patch.object(installer.cloudlog, "warning")
    installer.ensure_mapd()

    mock_warning.assert_not_called()


class TestPresentStatus:
  """mapdlogmgr2pnw: manager logs present_status() itself, because the installer subprocess's cloudlog lines
  are dropped at boot (measured on the truck). These drive the real override/sha comparison."""

  def test_ignored_pin_is_named(self, tmp_path, monkeypatch):
    _setup(tmp_path, monkeypatch, installed=b"old-binary-bytes", pinned=b"new-pinned-bytes", override=True)
    msg = installer.present_status()
    assert "IGNORED" in msg and "v2.3.1" in msg and installer.MAPD_OVERRIDE_FLAG in msg

  def test_plain_when_pin_matches(self, tmp_path, monkeypatch):
    _setup(tmp_path, monkeypatch, installed=b"same-bytes", pinned=b"same-bytes", override=True)
    assert installer.present_status() == "mapd installer: binary present"

  def test_plain_when_no_override(self, tmp_path, monkeypatch):
    _setup(tmp_path, monkeypatch, installed=b"same-bytes", pinned=b"same-bytes", override=False)
    assert installer.present_status() == "mapd installer: binary present"

  def test_unreadable_release_is_said_not_swallowed(self, tmp_path, monkeypatch):
    _setup(tmp_path, monkeypatch, installed=b"old-binary-bytes", pinned=b"new-pinned-bytes", override=True)
    (tmp_path / "mapd_release.json").write_text("{not json")
    msg = installer.present_status()
    assert "could not be compared" in msg
