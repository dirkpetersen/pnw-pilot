# fpcache2pnw: CarParamsCache write-gating in card (2026-07-11 poisoned-cache dashcam incident).
# A fixed-source (fleet-VIN-fallback) fingerprint carries an FW set that already failed
# match_fw_to_car — caching it dooms every later card restart of the session to MOCK.
from types import SimpleNamespace

from openpilot.common.params import Params
from openpilot.common.prefix import OpenpilotPrefix
from opendbc.car import structs
from openpilot.selfdrive.car.card import Car

LIGHTNING = "FORD_F_150_LIGHTNING_MK1"


def _make_cp(brand="ford", fingerprint=LIGHTNING, source="fw"):
  cp = structs.CarParams.new_message()
  cp.brand = brand
  cp.carFingerprint = fingerprint
  cp.fingerprintSource = source
  return cp


def _run_card(cp):
  # CI provided -> card skips fingerprinting/CAN wait and runs only the persist logic under test
  CI = SimpleNamespace(CP=cp, CC=None, CS=SimpleNamespace(secoc_key=None))
  Car(CI=CI, RI=SimpleNamespace())


class TestCarParamsCacheGating:
  def test_fw_source_writes_cache_and_persistent(self):
    with OpenpilotPrefix():
      params = Params()
      _run_card(_make_cp(source="fw"))
      assert params.get("CarParamsCache") is not None
      assert params.get("CarParamsPersistent") is not None

  def test_fixed_source_not_cached_but_persistent_updates(self):
    # The poisoned-cache guard: fleet-fallback (fixed) fingerprints must never enter the FW cache,
    # while the persistent copy (offroad UI identity) still updates.
    with OpenpilotPrefix():
      params = Params()
      _run_card(_make_cp(source="fixed"))
      assert params.get("CarParamsCache") is None
      persistent = params.get("CarParamsPersistent")
      assert persistent is not None
      with structs.CarParams.from_bytes(persistent) as cp:
        assert cp.carFingerprint == LIGHTNING

  def test_mock_never_persists_over_previous_good_fingerprint(self):
    # never-persist-MOCK (fingerprint2pnw) must be preserved exactly.
    with OpenpilotPrefix():
      params = Params()
      good = _make_cp(source="fw").to_bytes()
      params.put("CarParamsCache", good)
      params.put("CarParamsPersistent", good)
      _run_card(_make_cp(brand="mock", fingerprint="MOCK", source="can"))
      with structs.CarParams.from_bytes(params.get("CarParamsCache")) as cp:
        assert cp.carFingerprint == LIGHTNING
      with structs.CarParams.from_bytes(params.get("CarParamsPersistent")) as cp:
        assert cp.carFingerprint == LIGHTNING
      # session CarParams itself is MOCK (runs passive) — that is expected
      with structs.CarParams.from_bytes(params.get("CarParams")) as cp:
        assert cp.carFingerprint == "MOCK"
