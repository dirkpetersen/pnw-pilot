"""teslayaw2pnw: a car with no CAN yaw sensor publishes None, never a confident 0.0, for yaw-derived telemetry.

Two halves:
  * PnwVehicle.yaw_rate_source -- the ONE place that says which cars have a real carState.yawRate.
  * controlsd's kActl / kErr / peakAchLat (achLat follows kActl through _ach_lat_ms2) -- the EXACT shipped
    statements, AST-extracted from controlsd.py and executed against a stub, the same technique
    ces_pnw/tests/test_steerpower2pnw.py uses (controlsd cannot be imported without its whole process).
"""
import ast
import textwrap
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle

CONTROLSD = Path(__file__).resolve().parents[2] / "controlsd.py"


def _cp(fp, brand):
  return NS(carFingerprint=fp, brand=brand, openpilotLongitudinalControl=False, dashcamOnly=False)


class TestCapability:
  @pytest.mark.parametrize("fp, brand", [
    ("FORD_F_150_LIGHTNING_MK1", "ford"),   # Yaw_Data_FD1
    ("FORD_BRONCO_SPORT_MK1", "ford"),      # ford/carstate.py assigns it for every Ford
    ("TESLA_MODEL_S_HW3", "tesla"),         # the Raven: BrakeMessage 0x20a (pnw-opendbc teslayaw2pnw)
  ])
  def test_cars_with_a_yaw_sensor(self, fp, brand):
    assert PnwVehicle(_cp(fp, brand)).yaw_rate_source is True

  @pytest.mark.parametrize("fp, brand", [
    ("TESLA_MODEL_S_HW2", "tesla"),   # shares the DBC message, but nobody verified the field there
    ("TESLA_MODEL_S_HW1", "tesla"),
    ("TESLA_MODEL_3", "tesla"),       # the non-legacy carstate never assigns yawRate
    ("MOCK", "mock"),
  ])
  def test_cars_without_one(self, fp, brand):
    assert PnwVehicle(_cp(fp, brand)).yaw_rate_source is False

  def test_no_car_yet(self):
    assert PnwVehicle(None).yaw_rate_source is False


def _src():
  src = CONTROLSD.read_text()
  return src, ast.parse(src)


def _seg(src, node):
  return textwrap.dedent(" " * node.col_offset + ast.get_source_segment(src, node))


def _assign(name):
  src, tree = _src()
  hits = [n for n in ast.walk(tree) if isinstance(n, ast.Assign) and len(n.targets) == 1
          and isinstance(n.targets[0], ast.Name) and n.targets[0].id == name]
  assert len(hits) == 1, f"expected exactly one `{name} = ...` in controlsd.py, found {len(hits)}"
  return _seg(src, hits[0])


def _dict_value(key):
  src, tree = _src()
  hits = [v for n in ast.walk(tree) if isinstance(n, ast.Dict)
          for k, v in zip(n.keys, n.values, strict=True)
          if isinstance(k, ast.Constant) and k.value == key]
  assert hits, f"no dict entry {key!r} in controlsd.py"
  return [ast.get_source_segment(src, v) for v in hits]


def _k(source: bool, yaw: float, v: float = 20.0, k_cmd: float = 0.002):
  ns = {"self": NS(_yaw_rate_source=source), "CS": NS(yawRate=yaw), "v_ego_kappa": v, "k_cmd": k_cmd}
  exec(compile(_assign("k_actl") + "\n" + _assign("k_err"), "<controlsd>", "exec"), ns)
  return ns["k_actl"], ns["k_err"]


class TestControlsdTelemetry:
  def test_with_a_sensor_kactl_is_yaw_over_speed(self):
    k_actl, k_err = _k(True, yaw=0.2, v=20.0, k_cmd=0.012)
    assert k_actl == pytest.approx(0.01)
    assert k_err == pytest.approx(0.002)

  def test_with_a_sensor_a_real_zero_stays_zero(self):
    assert _k(True, yaw=0.0) == (0.0, pytest.approx(0.002))

  def test_without_a_sensor_both_are_none_not_zero(self):
    """THE bug: 7,752 of 7,753 Raven ticks read kActl 0.0 = 'driving perfectly straight'."""
    assert _k(False, yaw=0.0) == (None, None)

  def test_published_kactl_and_kerr_carry_the_none(self):
    for key, var in (("kActl", "k_actl"), ("kErr", "k_err")):
      exprs = _dict_value(key)
      evaluated = [eval(e, {"round": round, "float": float, var: None}) for e in exprs
                   if var in e]  # the steer_limit_status builder; the sample dict just forwards it
      assert evaluated == [None], f"{key}: {exprs}"

  def test_peak_ach_lat_is_none_without_a_sensor(self):
    (expr,) = _dict_value("peakAchLat")
    peak = 0.0   # all the 100 Hz accumulator ever saw on such a car
    assert eval(expr, {"round": round, "self": NS(_yaw_rate_source=False, _flight_peak_achlat=peak)}) is None
    assert eval(expr, {"round": round, "self": NS(_yaw_rate_source=True, _flight_peak_achlat=2.3456)}) == 2.346

  def test_the_missing_source_is_logged_once_at_start(self):
    """Rule 2: the None is announced, not silent. Extract the __init__ `if not self._yaw_rate_source:` block."""
    src, tree = _src()
    hits = [n for n in ast.walk(tree) if isinstance(n, ast.If) and "self._yaw_rate_source" in ast.get_source_segment(src, n.test)
            and isinstance(n.test, ast.UnaryOp)]
    assert len(hits) == 1
    block = _seg(src, hits[0])
    for source, want in ((False, 1), (True, 0)):
      said: list[str] = []
      log = NS(warning=said.append)
      exec(compile(block, "<init>", "exec"), {"self": NS(_yaw_rate_source=source, CP=NS(carFingerprint="X")),
                                              "cloudlog": log})
      assert len(said) == want and all("yaw" in s for s in said)
