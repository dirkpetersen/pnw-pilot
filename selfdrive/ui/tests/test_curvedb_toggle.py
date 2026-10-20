"""curvedbpull2pnw: the 'Disable Curve DB Auto-Install' row. TogglesLayout needs a raylib window, so the layout pieces
are pinned from source and the status method is exercised unbound (same approach as test_tailscale_toggle)."""
import inspect
from types import SimpleNamespace

from openpilot.selfdrive.ui.layouts.settings import toggles as T


class _Params:
  def __init__(self, disabled, status):
    self.disabled, self.status = disabled, status

  def get_bool(self, k):
    assert k == "DisableCurveDbAutoInstall"
    return self.disabled

  def get(self, k):
    assert k == "CurveDbAutoInstallStatus"
    return self.status


def _self(disabled, status):
  return SimpleNamespace(_params=_Params(disabled, status), _cdb_read_at=-1e9, _cdb_text_v="starting")


def test_status_line():
  f = T.TogglesLayout._curvedb_text
  assert f(_self(False, "waiting for unmetered network")) == "waiting for unmetered network"
  assert f(_self(False, b"")) == "starting"
  assert f(_self(True, "installed abcd1234 (10-15) @1")) == T.cdb_status.DISABLED_TEXT


def test_row_definition_is_device_wide_opt_out_with_status():
  assert "Turn this ON to disable" in T.DESCRIPTIONS["DisableCurveDbAutoInstall"]
  init = inspect.getsource(T.TogglesLayout.__init__)
  i = init.index('"DisableCurveDbAutoInstall": (')
  block = init[i:init.index("\n      ),", i)]
  assert "Disable Curve DB Auto-Install" in block and block.rstrip().rstrip(",").endswith("False")   # no restart
  assert 'toggle.set_description(lambda: tr(DESCRIPTIONS["DisableCurveDbAutoInstall"]) + "\\n\\nStatus: "' in init
  assert "DisableCurveDbAutoInstall" not in inspect.getsource(T.car_gate) and "DisableCurveDbAutoInstall" not in str(T.CAR_GATED)  # never greyed
