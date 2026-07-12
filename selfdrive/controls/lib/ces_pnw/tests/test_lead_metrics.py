"""
vtsctele2pnw: unit tests for the pure lead-follow telemetry helpers — lead_metrics() and its
wiring into decision_telemetry() (explicit lead bool, gap seconds, lead speed delta). Built from
the 2026-07-12 westbound I-90 leg where lead state had to be inferred from dRel>0.

Run:  pytest selfdrive/controls/lib/ces_pnw/tests/test_lead_metrics.py
"""
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import lead_metrics, decision_telemetry


ALL_ON = {"curves": True, "stops": True, "low_speed": True, "lead": True}


def base(**kw):
  s = {
    "v_ego": 30.0, "has_lead": False, "lead_vlead": 0.0, "lead_drel": 0.0, "blinker": False,
    "map_target_v": 0.0, "map_target_dist": float('inf'),
    "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0,
    "model_should_stop": False, "v_set": 0.0, "spd_lim": 0.0, "toggles": ALL_ON,
  }
  s.update(kw)
  return s


def test_no_lead_is_all_zero():
  assert lead_metrics(False, 0.0, 0.0, 30.0) == (0.0, 0.0)
  # even with junk residual dRel/vLead values, no-lead must report zeros
  assert lead_metrics(False, 55.0, 28.0, 30.0) == (0.0, 0.0)


def test_following_gap_and_delta():
  # 2026-07-12 17:31:34Z tick: dRel=47 m, vLead=28.0, vEgo=27.3 -> gap 1.7 s, lead pulling away 0.7
  gap, dv = lead_metrics(True, 47.0, 28.0, 27.3)
  assert gap == 1.7
  assert dv == 0.7


def test_closing_lead_negative_delta():
  gap, dv = lead_metrics(True, 23.0, 30.9, 34.5)   # 17:31:24Z: closing hard on a slower lead
  assert gap == 0.7
  assert dv == -3.6


def test_stopped_ego_no_divide_by_zero():
  gap, dv = lead_metrics(True, 12.0, 0.0, 0.0)
  assert gap == 0.0                                # near-stopped -> gap reported 0.0, no crash
  assert dv == 0.0


def test_decision_telemetry_carries_lead_fields():
  s = base(has_lead=True, lead_drel=47.0, lead_vlead=28.0, v_ego=27.3)
  t = decision_telemetry(s)
  assert t["lead"] is True
  assert t["gapS"] == 1.7
  assert t["dV"] == 0.7
  t0 = decision_telemetry(base())
  assert t0["lead"] is False and t0["gapS"] == 0.0 and t0["dV"] == 0.0
  # plain JSON-safe types only (capnp/json discipline)
  assert isinstance(t["lead"], bool) and isinstance(t["gapS"], float) and isinstance(t["dV"], float)
