"""vtscfloor2pnw release-later replay harness: feed recorded 1 Hz frames to the REAL VTSCController.cap() at 20 Hz. Open loop: vEgo,
the camera curve and the map points follow the recording. closed=True: the car follows min(cap, set) at accel in [-1.5, +0.8] m/s^2.
The controller's own state machine, fold, rate limits and floors are the real code.

Only two inputs are stubbed, both read from the model rather than from anything the fix touches: model_curve_state (returns the
recorded camera curve speed / apex distance) and apex_turn_direction (telemetry). Time is a fake monotonic clock."""

from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as VC
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_constants as C

MPH = 0.44704
LAT0, LON0 = 44.5, -123.1
M_PER_DEG = 111320.0


class FakeCP:
  def __init__(self, fp="TESLA_MODEL_S_HW3", brand="tesla", op_long=True):
    self.carFingerprint, self.brand, self.openpilotLongitudinalControl = fp, brand, op_long


class FakeParams:
  d = {"CESMode": "2", "VtscMapCurves": True}
  def get(self, k, return_default=False):
    return self.d.get(k)
  def get_bool(self, k):
    return bool(self.d.get(k, False))
  def put_nonblocking(self, k, v):
    pass


class FakeMem:
  def get(self, k, return_default=False):
    return None
  def put_nonblocking(self, k, v):
    pass


class _NS:
  pass


def secs(hms: str) -> float:
  h, m, s = hms.split(":")
  return int(h) * 3600 + int(m) * 60 + int(s)


def make_controller(monkeypatch, fp="TESLA_MODEL_S_HW3", brand="tesla", release_later=None):
  """A real VTSCController (Standard mode, map curves ON) on a fake clock; returns (ctrl, clock list)."""
  clock = [1000.0]
  monkeypatch.setattr(VC.time, "monotonic", lambda: clock[0])
  monkeypatch.setattr(VC, "apex_turn_direction", lambda model: 0)
  ctrl = VC.VTSCController(FakeCP(fp, brand), params=FakeParams())
  ctrl.mem_params = FakeMem()
  if release_later is not None and ctrl.veh.curve_brain_vtsc:
    ctrl.veh._tesla_curve_cfg["release_later"] = release_later
  ctrl._read_enabled(clock[0])
  assert ctrl._enabled and ctrl._map_curves
  return ctrl, clock


def replay(monkeypatch, frames, t0=None, t1=None, dt=0.05, closed=False, **kw):
  """Replay frames (release_later_frames tuples) -> list of dicts, one per 20 Hz cycle. `apply` keys: t (s since midnight PT),
  v (vEgo), cap (VTSC's cap), state, win (curveWin), set (m/s), vis (camera speed), msg, pay (the VTSCStatus payload), ctrl."""
  ctrl, clock = make_controller(monkeypatch, **kw)
  vis = {}
  monkeypatch.setattr(VC, "model_curve_state", lambda model, v_cruise, a_lat: vis["s"])
  ns = _NS()
  ns.orientationNED = [0.0, 0.0, 0.0]
  ns.enabled = True
  sm = {"modelV2": object(), "carControl": ns}
  out = []
  v_sim = None            # closed loop (Opus model): the car follows min(cap, set) at accel in [-1.5, +0.8] m/s^2, P gain 1
  for i in range(len(frames) - 1):
    a, b = frames[i], frames[i + 1]
    ta, tb = secs(a[0]), secs(b[0])
    if (t0 is not None and tb < t0) or (t1 is not None and ta > t1):
      continue
    n = int(round((tb - ta) / dt))
    for j in range(n):
      f = j / n
      t = ta + j * dt
      v_ego = a[1] + (b[1] - a[1]) * f
      if closed:
        v_sim = v_ego if v_sim is None else v_sim
        v_ego = v_sim
      v_set = a[2]
      vv = a[3] + (b[3] - a[3]) * f
      vd = a[4] + (b[4] - a[4]) * f
      clock[0] += dt
      pts = []
      for (d, raw) in a[7]:
        d2 = d - a[1] * (j * dt)
        if d2 > 0.0:
          pts.append({"latitude": LAT0 + d2 / M_PER_DEG, "longitude": LON0, "velocity": raw})
      ctrl._map_targets = pts
      ctrl._cur_lat, ctrl._cur_lon, ctrl._cur_bearing = LAT0, LON0, 0.0
      ctrl._last_read = clock[0]
      ctrl._gps_fix_ts = clock[0] - 1.4
      if vv > 0.0 and vd >= 0.0:
        k = C.A_LAT_TARGET / (vv * vv)
        vis["s"] = (k, vd, vv)
      else:
        vis["s"] = (0.0, -1.0, float("inf"))
      cap = ctrl.cap(sm, v_set, v_ego)
      if closed:
        v_sim += max(min((min(cap, v_set) - v_sim) / 1.0, 0.8), -1.5) * dt
      out.append(dict(t=t, v=v_ego, cap=cap, state=ctrl._state,
                      win=ctrl._tele_curve_win, set=v_set, vis=vv, msg=dict(ctrl.msg), pay=ctrl.overlay_payload(), ctrl=ctrl))
  return out
