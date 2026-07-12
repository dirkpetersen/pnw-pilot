# fpcache2pnw: a MOCK (flaky/no-car fingerprint) session must not erase driver preferences.
# During the 2026-07-11 dashcam incident, selfdrived's param cleanup ran with the session CP=MOCK
# (alphaLongitudinalAvailable=False, openpilotLongitudinalControl=False) and silently deleted
# AlphaLongitudinalEnabled (the op-long vs ICBM A/B switch) and would delete ExperimentalMode.
from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from opendbc.car import structs
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD


def _make_cp(brand, fingerprint, alpha_avail=False, op_long=False):
  cp = structs.CarParams.new_message()
  cp.brand = brand
  cp.carFingerprint = fingerprint
  cp.alphaLongitudinalAvailable = alpha_avail
  cp.openpilotLongitudinalControl = op_long
  return cp


class TestMockSessionParamPreservation:
  def test_mock_session_preserves_preferences(self):
    with OpenpilotPrefix():
      params = Params()
      params.put_bool("AlphaLongitudinalEnabled", True)
      params.put_bool("ExperimentalMode", True)
      SelfdriveD(CP=_make_cp("mock", "MOCK"))
      assert params.get_bool("AlphaLongitudinalEnabled")
      assert params.get_bool("ExperimentalMode")

  def test_real_car_lacking_alpha_long_still_cleans_up(self):
    # the upstream cleanup semantics must survive for REAL fingerprints
    with OpenpilotPrefix():
      params = Params()
      params.put_bool("AlphaLongitudinalEnabled", True)
      params.put_bool("ExperimentalMode", True)
      SelfdriveD(CP=_make_cp("ford", "FORD_F_150_LIGHTNING_MK1", alpha_avail=False, op_long=False))
      assert params.get("AlphaLongitudinalEnabled") is None
      assert params.get("ExperimentalMode") is None

  def test_real_car_with_capabilities_keeps_params(self):
    with OpenpilotPrefix():
      params = Params()
      params.put_bool("AlphaLongitudinalEnabled", True)
      params.put_bool("ExperimentalMode", True)
      SelfdriveD(CP=_make_cp("ford", "FORD_F_150_LIGHTNING_MK1", alpha_avail=True, op_long=True))
      assert params.get_bool("AlphaLongitudinalEnabled")
      assert params.get_bool("ExperimentalMode")
