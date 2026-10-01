"""mapsl2pnw: mapd_configd clears RoadContext to "" when mapd dies. Police keeps running (position from LastGPSPosition),
so "" must keep the last verdict -- else the police `cap` (blue banner + speedadjust slowdown) is dropped on a freeway."""
import inspect

from openpilot.system.location_services import location_servicesd as lsd
from openpilot.system.location_services.tests.test_police_miss import (REPORT_0912_US97, TRACK_0912_US97, _FakePolice, _real,
                                                                       _recede)


def _payload_through_hold(ctxs):
  """Run the REAL RoadCtxHold then _police_payload (the two steps main() chains) over the real US-97 track."""
  lat, lon, uuid, age, _ = REPORT_0912_US97
  hold, recede, outs = lsd.RoadCtxHold(), _recede(), []
  for ctx, (la, lo, b) in zip(ctxs, TRACK_0912_US97, strict=False):
    on_fw = hold.update(ctx) == "freeway"
    outs.append(lsd._police_payload(on_fw, _FakePolice([_real(lat, lon, uuid, age, 6)], armed=False), la, lo, b, [], recede))
  return outs


def test_freeway_then_cleared_keeps_the_cap_and_the_line():
  n = len(TRACK_0912_US97)
  outs = _payload_through_hold(["freeway"] + [""] * (n - 1))
  assert all(o["state"] == "alert" and "cap" in o for o in outs), outs


def test_never_set_keeps_the_existing_default():
  assert all(o == {"state": "nodata"} for o in _payload_through_hold([""] * len(TRACK_0912_US97)))   # not armed: nodata


def test_city_is_city_and_a_live_verdict_replaces_the_held_one():
  h = lsd.RoadCtxHold()
  assert h.update("freeway") == "freeway" and h.update("") == "freeway"
  assert h.update("city") == "city" and h.update("") == "city"
  assert h.update("unknown") == "unknown"


def test_one_log_per_transition(monkeypatch):
  lines = []
  monkeypatch.setattr(lsd.cloudlog, "warning", lambda m, *a, **k: lines.append(("w", m)))
  monkeypatch.setattr(lsd.cloudlog, "info", lambda m, *a, **k: lines.append(("i", m)))
  h = lsd.RoadCtxHold()
  for c in ["", "freeway", "", "", "", "freeway", "", ""]:
    h.update(c)
  assert [k for k, _ in lines] == ["w", "i", "w"]      # held, live again, held again; never-set "" logs nothing


def test_main_runs_the_verdict_through_the_hold():
  src = inspect.getsource(lsd.main)
  assert src.count("road_hold.update(ctx)") == 1 and src.index("road_hold.update(ctx)") < src.index("on_freeway = ")
