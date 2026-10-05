"""toggles2pnw phase 2: car-specific toggles are GREYED on the other car (never hidden), by PnwVehicle capability, display
only, and EVERY row stays enabled when the car is unknown (no last-known CarParams).

TogglesLayout needs a raylib window to construct, so `_update_toggles` is run unbound against a stand-in `self` whose toggles are
recording fakes; `car_gate` (pure) is tested directly with real PnwVehicle objects built from minimal CarParams stand-ins."""
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle
from openpilot.selfdrive.ui.layouts.settings import toggles as T
from openpilot.selfdrive.ui.ui_state import ui_state

LIGHTNING = "FORD_F_150_LIGHTNING_MK1"
TESLA = "TESLA_MODEL_S_HW3"


def cp(fp, brand):
  return SimpleNamespace(carFingerprint=fp, brand=brand, openpilotLongitudinalControl=False, alphaLongitudinalAvailable=False)


CARS = {"lightning": cp(LIGHTNING, "ford"), "tesla": cp(TESLA, "tesla"), "none": None,
        "mock": cp("MOCK", "mock"), "empty": cp("", "")}
# param -> the cars on which it is OPERABLE when the car is known
OPERABLE_ON = {
  "DisableCoopSteer": {"tesla"},
  "NoFordAngleSteering": {"lightning"},
  "DisableFordSignSpeedLimit": {"lightning"},
  "DisableEverDrive": {"lightning"},
  "DisableMapdCarGps": {"lightning"},
  "NudgeForLaneChange": {"lightning", "tesla"},
  "DisengageOnBrake": {"lightning", "tesla"},
}
# owner 2026-10-04: affects only the Lightning but is settable on EVERY car (set in advance, before the first start in the truck)
SETTABLE_EVERYWHERE = "DisableFordConvenience"


@pytest.mark.parametrize("param", sorted(OPERABLE_ON))
@pytest.mark.parametrize("car", sorted(CARS))
def test_car_gate_table(param, car):
  ok, reason = T.car_gate(PnwVehicle(CARS[car]), param)
  if car in ("none", "mock", "empty"):          # unknown car: everything enabled, nothing greyed
    assert (ok, reason) == (True, None)
  elif car in OPERABLE_ON[param]:
    assert (ok, reason) == (True, None)
  else:
    assert ok is False and reason and "only" in reason


def test_table_covers_exactly_the_car_specific_toggles_and_each_has_a_reason():
  assert set(T.CAR_GATED) == set(OPERABLE_ON)
  assert all(isinstance(r, str) and "only" in r for _, r in T.CAR_GATED.values())


@pytest.mark.parametrize("car", sorted(CARS))
def test_ford_convenience_is_operable_on_every_car_and_not_gated(car):
  assert SETTABLE_EVERYWHERE not in T.CAR_GATED
  assert T.car_gate(PnwVehicle(CARS[car]), SETTABLE_EVERYWHERE) == (True, None)


def test_a_param_outside_the_table_is_never_gated():
  assert T.car_gate(PnwVehicle(CARS["tesla"]), "DisableLaneCentering") == (True, None)


class _Item:
  def __init__(self):
    self.enabled, self.state = True, False
    self.action_item = self

  def set_enabled(self, v):
    self.enabled = bool(v)

  def set_state(self, v):
    self.state = bool(v)


class _Params:
  def __init__(self, vals=None):
    self.vals, self.writes = vals or {}, []

  def get_bool(self, k):
    return bool(self.vals.get(k, False))

  def put_bool(self, k, v):
    self.writes.append((k, v))
    self.vals[k] = v


def run_update(monkeypatch, car, stored=None):
  monkeypatch.setattr(ui_state, "update_params", lambda: None)
  monkeypatch.setattr(ui_state, "CP", CARS[car], raising=False)
  monkeypatch.setattr(ui_state, "has_longitudinal_control", False, raising=False)
  params = _Params(stored)
  names = list(OPERABLE_ON) + [SETTABLE_EVERYWHERE, "DisableLaneCentering", "RefreshLocationMap"]
  toggles = {n: _Item() for n in names}
  toggles["CESMode"], toggles["AutoSpeedReduce"] = _Item(), _Item()
  me = SimpleNamespace(_params=params, _toggles=toggles, _toggle_defs=dict.fromkeys(names, (None, None, None, False)),
                       _locked_toggles=set(), _long_personality_setting=_Item(), _grey_reason={})
  monkeypatch.setattr(type(ui_state), "engaged", property(lambda s: False), raising=False)
  T.TogglesLayout._update_toggles(me)
  return me, params


@pytest.mark.parametrize("car", ["tesla", "lightning"])
def test_update_toggles_greys_never_hides_and_never_writes(monkeypatch, car):
  stored = {"DisableCoopSteer": True, "DisableEverDrive": True, "DisableFordSignSpeedLimit": True, "DisableMapdCarGps": True, "DisableFordConvenience": True,
            "FordAngleLateral": False}
  me, params = run_update(monkeypatch, car, dict(stored))
  for param, cars in OPERABLE_ON.items():
    assert param in me._toggles, "greyed means present, never removed"
    if param in ("DisableCoopSteer", "DisableEverDrive", "DisableFordSignSpeedLimit", "DisableMapdCarGps"):
      assert me._toggles[param].enabled == (car in cars), param
    assert (me._grey_reason[param] is None) == (car in cars), param
    if car not in cars:
      assert "only" in me._grey_reason[param]
  assert SETTABLE_EVERYWHERE in me._toggles and SETTABLE_EVERYWHERE not in me._grey_reason, "never greyed, on either car"
  assert me._toggles[SETTABLE_EVERYWHERE].enabled is True
  assert params.writes == [], "greying is display only: the stored params are never written"
  for k in ("DisableCoopSteer", "DisableEverDrive", "DisableFordSignSpeedLimit", "DisableMapdCarGps", "DisableFordConvenience"):
    assert params.vals[k] is True


@pytest.mark.parametrize("car", ["none", "mock", "empty"])
def test_unknown_car_leaves_everything_enabled_and_unforced(monkeypatch, car):
  me, params = run_update(monkeypatch, car)
  for param in ("DisableCoopSteer", "DisableEverDrive", "DisableFordSignSpeedLimit", "DisableFordConvenience", "NoFordAngleSteering",
                "NudgeForLaneChange"):
    assert me._toggles[param].enabled is True, param
  assert me._toggles["NoFordAngleSteering"].state is False, "an unknown car must not paint 'no angle steering' ON"
  assert me._toggles["NudgeForLaneChange"].state is False
  assert all(v is None for v in me._grey_reason.values())
  assert params.writes == []


def test_known_other_car_keeps_the_existing_forced_display(monkeypatch):
  """Unchanged by phase 2: on the Tesla the angle-steering row is greyed and painted ON (display only)."""
  me, params = run_update(monkeypatch, "tesla")
  assert me._toggles["NoFordAngleSteering"].enabled is False and me._toggles["NoFordAngleSteering"].state is True
  assert params.writes == []


def test_grey_reason_reaches_the_description():
  me = SimpleNamespace(_grey_reason={"DisableCoopSteer": "Tesla Model S HW3 only"})
  assert "Tesla Model S HW3 only" in T.TogglesLayout._grey_suffix(me, "DisableCoopSteer")
  assert T.TogglesLayout._grey_suffix(me, "DisableEverDrive") == ""


def test_ford_convenience_toggle_is_defined_opt_out_and_default_off():
  """toggles2pnw phase 3: title, description (lists exactly what it gates NOW), no restart, default 0."""
  import inspect
  from openpilot.common.params import Params
  assert Params().get("DisableFordConvenience", return_default=True) is False
  d = T.DESCRIPTIONS["DisableFordConvenience"]
  assert "Pro Power" in d and "Stops the comma" in d and "troubleshooting" in d and "Lightning only" in d
  assert "Can be set in advance on any car so the comma never sends a convenience frame the first time it starts in the truck." in d
  init = inspect.getsource(T.TogglesLayout.__init__)
  i = init.index('"DisableFordConvenience": (')
  block = init[i:init.index("\n      ),", i)]
  assert "Disable Ford Convenience Features" in block and block.rstrip().rstrip(",").endswith("False")  # no restart


def test_mapd_car_gps_row_exists_defaults_off_and_is_registered():
  """mapdcargpsdefault2pnw: opt-out row, reason names the Lightning, default "0" in params_keys.h, old key gone."""
  import re
  from pathlib import Path
  assert "DisableMapdCarGps" in T.DESCRIPTIONS
  assert T.CAR_GATED["DisableMapdCarGps"][1] == "Ford F-150 Lightning only"
  hdr = (Path(__file__).resolve().parents[3] / "common" / "params_keys.h").read_text()
  assert re.search(r'\{"DisableMapdCarGps", \{PERSISTENT, BOOL, "0"\}\}', hdr)
  assert "MapdUseCarGps" not in hdr
