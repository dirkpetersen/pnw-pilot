"""
Unit tests for the light-ces-gentle CESMode 3-way master (Off / Light / Standard).

Covers:
  - the pure mode helpers (ces_enabled / ces_is_gentle / read_ces_mode + bool back-compat)
  - CESController: CESMode -> (enabled, gentle profile / dwell, curve suppression)
  - VTSCController: CESMode -> (enabled, GENTLE vs DEFAULT tune)
  - the grey-out symmetry helper (ces_group_enabled): off on long-off, ON on long-on

Pure logic — no cereal, no car. A tiny FakeParams stands in for the param store.
"""
from openpilot.selfdrive.controls.lib.ces_xnor import ces_xnor_constants as C
from openpilot.selfdrive.controls.lib.ces_xnor.ces_xnor import CESController
from openpilot.selfdrive.controls.lib.vtsc_xnor.vtsc_controller import VTSCController
from openpilot.selfdrive.controls.lib.vtsc_xnor import vtsc_constants as VC


class FakeParams:
  """Minimal Params stand-in: get() returns ints/objects, get_bool() returns bools."""
  def __init__(self, store=None):
    self.store = dict(store or {})

  def get(self, key, return_default=False):
    return self.store.get(key)

  def get_bool(self, key):
    return bool(self.store.get(key, False))

  def put(self, key, val):
    self.store[key] = val

  def put_bool(self, key, val):
    self.store[key] = bool(val)

  def put_nonblocking(self, key, val):
    self.store[key] = val


class FakeCP:
  def __init__(self, long_ok=True, fp="TESLA_MODEL_S_HW3"):
    self.openpilotLongitudinalControl = long_ok
    self.carFingerprint = fp


# ---- pure mode helpers ------------------------------------------------------
def test_ces_enabled_mapping():
  assert C.ces_enabled(C.CES_MODE_OFF) is False
  assert C.ces_enabled(C.CES_MODE_LIGHT) is True
  assert C.ces_enabled(C.CES_MODE_STANDARD) is True


def test_ces_is_gentle_mapping():
  assert C.ces_is_gentle(C.CES_MODE_OFF) is False
  assert C.ces_is_gentle(C.CES_MODE_LIGHT) is True
  assert C.ces_is_gentle(C.CES_MODE_STANDARD) is False


def test_read_ces_mode_int():
  for m in (0, 1, 2):
    assert C.read_ces_mode(FakeParams({"CESMode": m})) == m


def test_read_ces_mode_backcompat_bool():
  # CESMode absent/0 but legacy bool set -> Standard (2)
  assert C.read_ces_mode(FakeParams({"ConditionalExperimentalSwitching": True})) == C.CES_MODE_STANDARD
  assert C.read_ces_mode(FakeParams({"CESMode": 0, "ConditionalExperimentalSwitching": True})) == C.CES_MODE_STANDARD
  # CESMode wins when set
  assert C.read_ces_mode(FakeParams({"CESMode": 1, "ConditionalExperimentalSwitching": True})) == C.CES_MODE_LIGHT


def test_read_ces_mode_defaults_off():
  assert C.read_ces_mode(FakeParams({})) == C.CES_MODE_OFF


# ---- CESController: mode -> enabled + profile -------------------------------
def _ces(mode, long_ok=True):
  ctl = CESController(FakeCP(long_ok=long_ok), params=FakeParams({"CESMode": mode}))
  ctl._read_params()
  return ctl


def test_ces_controller_off_disabled():
  ctl = _ces(C.CES_MODE_OFF)
  assert ctl.enabled() is False
  assert ctl._gentle is False


def test_ces_controller_light_enabled_gentle():
  ctl = _ces(C.CES_MODE_LIGHT)
  assert ctl.enabled() is True
  assert ctl._gentle is True
  # gentle profile lengthens the dwell on the state machine
  assert ctl._sm._exp_min == C.GENTLE_EXP_MIN_DWELL_S
  assert ctl._sm._chill_min == C.GENTLE_CHILL_MIN_DWELL_S


def test_ces_controller_standard_enabled_default():
  ctl = _ces(C.CES_MODE_STANDARD)
  assert ctl.enabled() is True
  assert ctl._gentle is False
  assert ctl._sm._exp_min == C.EXP_MIN_DWELL_S
  assert ctl._sm._chill_min == C.CHILL_MIN_DWELL_S


def test_ces_controller_disabled_when_no_long():
  # any mode but no openpilot longitudinal -> disabled
  assert _ces(C.CES_MODE_LIGHT, long_ok=False).enabled() is False
  assert _ces(C.CES_MODE_STANDARD, long_ok=False).enabled() is False


def test_ces_controller_runtime_mode_switch_rebuilds_sm():
  p = FakeParams({"CESMode": C.CES_MODE_STANDARD})
  ctl = CESController(FakeCP(), params=p)
  ctl._read_params()
  assert ctl._gentle is False and ctl._sm._exp_min == C.EXP_MIN_DWELL_S
  # user switches to Light at runtime
  p.put("CESMode", C.CES_MODE_LIGHT)
  ctl._read_params()
  assert ctl._gentle is True and ctl._sm._exp_min == C.GENTLE_EXP_MIN_DWELL_S


def test_ces_controller_light_independent_of_fingerprint():
  # gentle now comes from CESMode, NOT carFingerprint: a Tesla in Light is gentle; a Lightning in
  # Standard is NOT gentle (the inverse of the old fingerprint gating).
  tesla_light = CESController(FakeCP(fp="TESLA_MODEL_S_HW3"), params=FakeParams({"CESMode": C.CES_MODE_LIGHT}))
  tesla_light._read_params()
  assert tesla_light._gentle is True
  ford_std = CESController(FakeCP(fp="FORD_F_150_LIGHTNING_MK1"), params=FakeParams({"CESMode": C.CES_MODE_STANDARD}))
  ford_std._read_params()
  assert ford_std._gentle is False


# ---- VTSCController: mode -> enabled + tune ---------------------------------
def _vtsc(mode, long_ok=True, fp="TESLA_MODEL_S_HW3"):
  ctl = VTSCController(FakeCP(long_ok=long_ok, fp=fp), params=FakeParams({"CESMode": mode}))
  ctl._read_enabled(1e9)   # force the ~1 Hz read
  return ctl


def test_vtsc_off_disabled_default_tune():
  ctl = _vtsc(C.CES_MODE_OFF)
  assert ctl.enabled() is False
  assert ctl.tune == dict(VC.DEFAULT_PROFILE)


def test_vtsc_light_enabled_gentle_tune():
  ctl = _vtsc(C.CES_MODE_LIGHT)
  assert ctl.enabled() is True
  assert ctl.tune == dict(VC.GENTLE_PROFILE)


def test_vtsc_standard_enabled_default_tune():
  ctl = _vtsc(C.CES_MODE_STANDARD)
  assert ctl.enabled() is True
  assert ctl.tune == dict(VC.DEFAULT_PROFILE)


def test_vtsc_disabled_when_no_long():
  assert _vtsc(C.CES_MODE_LIGHT, long_ok=False).enabled() is False


def test_vtsc_tune_independent_of_fingerprint():
  # Lightning in Standard now uses the DEFAULT tune (fingerprint no longer selects gentle)
  ford_std = _vtsc(C.CES_MODE_STANDARD, fp="FORD_F_150_LIGHTNING_MK1")
  assert ford_std.tune == dict(VC.DEFAULT_PROFILE)
  # Tesla in Light uses the GENTLE tune
  tesla_light = _vtsc(C.CES_MODE_LIGHT, fp="TESLA_MODEL_S_HW3")
  assert tesla_light.tune == dict(VC.GENTLE_PROFILE)


def test_vtsc_runtime_mode_switch_reselects_tune():
  p = FakeParams({"CESMode": C.CES_MODE_STANDARD})
  ctl = VTSCController(FakeCP(), params=p)
  ctl._read_enabled(1e9)
  assert ctl.tune == dict(VC.DEFAULT_PROFILE)
  p.put("CESMode", C.CES_MODE_LIGHT)
  ctl._read_enabled(2e9)
  assert ctl.tune == dict(VC.GENTLE_PROFILE)


# ---- grey-out symmetry ------------------------------------------------------
def test_ces_group_enabled_symmetry():
  import pytest
  try:
    from openpilot.selfdrive.ui.layouts.settings.toggles import ces_group_enabled
  except Exception as e:   # UI stack (pyray) unavailable off-device — skip; runs on-device / in CI
    pytest.skip(f"UI deps unavailable: {e}")

  class CP:
    def __init__(self, long_ok):
      self.openpilotLongitudinalControl = long_ok

  # long control OFF -> whole group disabled; long control ON -> whole group enabled.
  assert ces_group_enabled(CP(False)) is False
  assert ces_group_enabled(CP(True)) is True
  assert ces_group_enabled(None) is False   # no car seen yet
  # the SAME helper drives both directions, so re-enabling on long-on can't be partial
  off_then_on = [ces_group_enabled(CP(False)), ces_group_enabled(CP(True))]
  assert off_then_on == [False, True]
