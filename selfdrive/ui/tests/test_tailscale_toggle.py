"""tailscale2pnw / toggles2pnw: the 'Disable Remote SSH (Tailscale)' toggle and its live status line.

TogglesLayout needs a raylib window, so the layout pieces are pinned from source and the status method is
exercised unbound with a stand-in `self`."""
import inspect
from types import SimpleNamespace

from openpilot.selfdrive.ui.layouts.settings import toggles as T


class _Params:
  def __init__(self, enabled, status):
    self.enabled, self.status = enabled, status    # enabled = Remote SSH enabled = DisableTailscale is OFF

  def get_bool(self, k):
    assert k == "DisableTailscale"
    return not self.enabled

  def get(self, k):
    assert k == "TailscaleStatus"
    return self.status


def _self(enabled, status):
  return SimpleNamespace(_params=_Params(enabled, status), _ts_read_at=-1e9, _ts_text="disconnected")


def test_status_line_shows_state_when_enabled_and_disabled_when_disabled():
  f = T.TogglesLayout._tailscale_text
  assert f(_self(False, "connected 100.1.2.3")) == T.ts_status.DISABLED_TEXT  # stale status never shown while Disable is ON
  assert f(_self(True, "connected 100.1.2.3")) == "connected 100.1.2.3"   # the default: the row shows 'connected'
  assert f(_self(True, "off")) == "connecting"                      # daemon's own stale 'off' from before the flip
  stuck = "error tailscaled still running after toggle off; could not stop it"
  assert f(_self(False, stuck)) == stuck                          # an error survives toggle OFF
  assert T.TogglesLayout._tailscale_title_suffix(SimpleNamespace(_tailscale_text=lambda: stuck)) == " - error"
  assert f(_self(True, b"unconfigured")) == "unconfigured"
  assert f(_self(True, "error up failed")) == "error up failed"
  assert f(_self(True, None)) == "connecting"                        # on, daemon not heard from yet


def test_status_is_throttled():
  s = _self(True, "connecting")
  T.TogglesLayout._tailscale_text(s)
  s._params.status = "connected 100.1.2.3"
  assert T.TogglesLayout._tailscale_text(s) == "connecting"       # re-read at most every 2 s, not per frame


def test_toggle_def_and_description():
  d = T.DESCRIPTIONS["DisableTailscale"]
  assert "docs/pnw/TAILSCALE.md" in d and "once configured it is ON by default" in d and "unconfigured" in d and "Turn this ON to disable" in d
  assert "TailscaleEnabled" not in T.DESCRIPTIONS and "TailscaleEnabled" not in inspect.getsource(T)
  init = inspect.getsource(T.TogglesLayout.__init__)
  i = init.index('"DisableTailscale": (')
  block = init[i:init.index("\n      ),", i)]
  assert "Disable Remote SSH (Tailscale)" in block and block.rstrip().rstrip(",").endswith("False")  # no restart
  assert 'toggle.set_description(lambda: tr(DESCRIPTIONS["DisableTailscale"]) + "\\n\\nStatus: "' in init


def test_title_suffix_for_disabled_and_connected():
  suffix = T.TogglesLayout._tailscale_title_suffix
  assert suffix(SimpleNamespace(_tailscale_text=lambda: T.ts_status.DISABLED_TEXT)) == " - disconnected"
  assert suffix(SimpleNamespace(_tailscale_text=lambda: "disconnected - no internet")) == " - disconnected"
  assert suffix(SimpleNamespace(_tailscale_text=lambda: "unconfigured")) == " - unconfigured"
  assert suffix(SimpleNamespace(_tailscale_text=lambda: "connected 100.64.0.5")) == " - connected"
  assert suffix(SimpleNamespace(_tailscale_text=lambda: "off")) == ""
  assert T.TogglesLayout._tailscale_title_suffix(SimpleNamespace(_tailscale_text=lambda: "connecting")) == " - connecting"


def test_title_carries_only_the_state_word_the_reason_stays_in_the_description():
  """Fable F3 mutation: put the full text in the title -> the long error overruns the switch."""
  long_err = "error not connected for over 3 min: " + "x" * 80
  suffix = T.TogglesLayout._tailscale_title_suffix
  assert suffix(SimpleNamespace(_tailscale_text=lambda: long_err)) == " - error"
  assert suffix(SimpleNamespace(_tailscale_text=lambda: "connected 100.64.0.5")) == " - connected"
  assert suffix(SimpleNamespace(_tailscale_text=lambda: "installing - deferred until parked")) == " - installing"
  assert "Status: " in inspect.getsource(T.TogglesLayout.__init__) and "self._tailscale_text()" in inspect.getsource(T.TogglesLayout.__init__)
