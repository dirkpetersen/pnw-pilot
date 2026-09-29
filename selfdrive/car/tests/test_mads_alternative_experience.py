#!/usr/bin/env python3
"""mads2pnw: the openpilot-side half of "lateral survives a brake press".

These tests hold down the claims card.py makes:
  1. alternativeExperience is COMPUTED, not hardcoded to 0 (the pre-mads2pnw state, which made
     every alternative-experience bit unreachable);
  2. the MADS bits are set ONLY for a car with the mads_lateral capability (the Lightning and, since
     teslamads2pnw, the Tesla Raven), and ONLY when PandaMadsSafety declares the flashed panda actually
     carries controls_allowed_lateral -- so neither car can receive them on a stock panda, and no other
     car ever can;
  3. the toggle's INVERTED polarity: DisengageOnBrake OFF (the shipping default) = REMAIN_ACTIVE
     (steering survives the brake); ON = DISENGAGE (stock).

They call the real Car._alternative_experience through a stub `self`, so a change to the method's
logic (or to PnwVehicle.mads_lateral) fails here.
"""
import pytest

from opendbc.safety import ALTERNATIVE_EXPERIENCE
from openpilot.selfdrive.car.card import Car
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle


class _StubCP:
  def __init__(self, fingerprint):
    self.carFingerprint = fingerprint
    self.brand = "ford" if fingerprint.startswith("FORD") else "tesla"
    self.openpilotLongitudinalControl = False


class _StubParams:
  def __init__(self, values):
    self._values = values

  def get_bool(self, key):
    return self._values.get(key, False)


class _StubCar:
  """Just enough of Car for _alternative_experience -- deliberately NOT a mock: the real method
  runs, so removing the capability check, the panda check or the toggle check fails these."""
  def __init__(self, fingerprint, disengage_on_brake=False, panda_mads=True):
    self.CP = _StubCP(fingerprint)
    self.params = _StubParams({"DisengageOnBrake": disengage_on_brake,
                               "PandaMadsSafety": panda_mads})

  _alternative_experience = Car._alternative_experience


LIGHTNING = "FORD_F_150_LIGHTNING_MK1"
RAVEN = "TESLA_MODEL_S_HW3"

REMAIN_ACTIVE = ALTERNATIVE_EXPERIENCE.ENABLE_MADS
DISENGAGE = ALTERNATIVE_EXPERIENCE.ENABLE_MADS | ALTERNATIVE_EXPERIENCE.MADS_DISENGAGE_LATERAL_ON_BRAKE


class TestMadsAlternativeExperience:
  def test_shipping_default_is_remain_active(self):
    """DisengageOnBrake ships "0" (default-OFF rule) and OFF is the NEW behaviour: lateral
    survives the brake. Deliberate inverted polarity, not a slip."""
    assert _StubCar(LIGHTNING)._alternative_experience() == REMAIN_ACTIVE

  def test_toggle_on_is_stock_disengage(self):
    assert _StubCar(LIGHTNING, disengage_on_brake=True)._alternative_experience() == DISENGAGE

  def test_pause_mode_is_never_exposed(self):
    for toggle in (False, True):
      alt_exp = _StubCar(LIGHTNING, disengage_on_brake=toggle)._alternative_experience()
      assert not alt_exp & ALTERNATIVE_EXPERIENCE.MADS_PAUSE_LATERAL_ON_BRAKE

  @pytest.mark.parametrize("toggle_on", [False, True])
  def test_stock_panda_gets_nothing(self, toggle_on):
    """THE SAFETY GATE: with no MADS-capable panda flashed we send 0 -- the stock contract --
    so openpilot and the panda can never disagree about who owns lateral."""
    car = _StubCar(LIGHTNING, disengage_on_brake=toggle_on, panda_mads=False)
    assert car._alternative_experience() == ALTERNATIVE_EXPERIENCE.DEFAULT
    assert car._alternative_experience() == 0

  @pytest.mark.parametrize("toggle_on", [False, True])
  def test_raven_gets_mads_only_with_the_flashed_panda_declaration(self, toggle_on):
    """teslamads2pnw: the Raven is treated exactly like the Lightning -- bits only with PandaMadsSafety,
    inverted toggle polarity -- and gets NOTHING on a stock panda."""
    assert _StubCar(RAVEN, toggle_on, panda_mads=False)._alternative_experience() == 0
    expected = DISENGAGE if toggle_on else REMAIN_ACTIVE
    assert _StubCar(RAVEN, toggle_on, panda_mads=True)._alternative_experience() == expected

  def test_other_cars_never_get_mads(self):
    """No other car, in any toggle/panda combination (defense in depth for the capability list)."""
    for toggle_on in (False, True):
      for panda_mads in (False, True):
        for fp in ("TESLA_MODEL_3_PARTY", "FORD_F_150_MK14", "HONDA_CIVIC"):
          assert _StubCar(fp, toggle_on, panda_mads)._alternative_experience() == 0, fp

  def test_capability_list(self):
    assert PnwVehicle(_StubCP(LIGHTNING)).mads_lateral
    assert PnwVehicle(_StubCP(RAVEN)).mads_lateral
    assert not PnwVehicle(_StubCP("TESLA_MODEL_3_PARTY")).mads_lateral
    assert not PnwVehicle(None).mads_lateral

  def test_raven_has_no_auto_resume(self):
    """mads_resume needs the stock-ACC button path, which the Raven does not have."""
    assert not PnwVehicle(_StubCP(RAVEN)).mads_resume
