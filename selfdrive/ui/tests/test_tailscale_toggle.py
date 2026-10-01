"""tailscale2pnw: the 'Remote SSH (Tailscale)' toggle and its live status line.

TogglesLayout needs a raylib window, so the layout pieces are pinned from source and the status method is
exercised unbound with a stand-in `self`."""
import inspect
from types import SimpleNamespace

from openpilot.selfdrive.ui.layouts.settings import toggles as T


class _Params:
  def __init__(self, enabled, status):
    self.enabled, self.status = enabled, status

  def get_bool(self, k):
    assert k == "TailscaleEnabled"
    return self.enabled

  def get(self, k):
    assert k == "TailscaleStatus"
    return self.status


def _self(enabled, status):
  return SimpleNamespace(_params=_Params(enabled, status), _ts_read_at=-1e9, _ts_text="off")


def test_status_line_shows_state_when_on_and_off_when_off():
  f = T.TogglesLayout._tailscale_text
  assert f(_self(False, "connected 100.1.2.3")) == "off"          # stale status never shown while the toggle is off
  assert f(_self(True, b"needs auth key")) == "needs auth key"
  assert f(_self(True, "error up failed")) == "error up failed"
  assert f(_self(True, None)) == "starting"                        # on, daemon not heard from yet


def test_status_is_throttled():
  s = _self(True, "connecting")
  T.TogglesLayout._tailscale_text(s)
  s._params.status = "connected 100.1.2.3"
  assert T.TogglesLayout._tailscale_text(s) == "connecting"       # re-read at most every 2 s, not per frame


def test_toggle_def_and_description():
  assert "docs/pnw/TAILSCALE.md" in T.DESCRIPTIONS["TailscaleEnabled"]
  init = inspect.getsource(T.TogglesLayout.__init__)
  i = init.index('"TailscaleEnabled": (')
  block = init[i:init.index("\n      ),", i)]
  assert "Remote SSH (Tailscale)" in block and block.rstrip().rstrip(",").endswith("False")  # no restart
  assert 'toggle.set_description(lambda: tr(DESCRIPTIONS["TailscaleEnabled"]) + "\\n\\nStatus: "' in init


def test_title_suffix_empty_when_off():
  assert T.TogglesLayout._tailscale_title_suffix(SimpleNamespace(_tailscale_text=lambda: "off")) == ""
  assert T.TogglesLayout._tailscale_title_suffix(SimpleNamespace(_tailscale_text=lambda: "needs auth key")) == " - needs auth key"
