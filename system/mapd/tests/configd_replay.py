"""Drive the REAL mapd_configd.main() loop with a scripted message stream (gpsfix2pnw).

Not a test file (no `test_` prefix): a harness the GPS tests share. What is real and what is not:
  real  -- mapd_configd.main() itself, cereal's SubMaster bookkeeping (updated / alive / recv_time,
           via SubMaster.update_msgs), capnp messages round-tripped through bytes.
  faked -- the msgq sockets (SubMaster.update pops the next scripted step instead of polling),
           Params (dict stores that record every write), the clock, and cloudlog (records events).

A step is (t, [(service, {field: value}), ...]). Each step is one loop iteration at monotonic time t,
delivering those messages; an empty list is a poll that timed out. The loop ends when the script
runs out (main() is `while True`, so the fake update() raises _ReplayDone).
"""
import json
from types import SimpleNamespace

import cereal.messaging as messaging

from openpilot.system.mapd import mapd_configd

_RealSubMaster = messaging.SubMaster   # captured before any test patches messaging.SubMaster


class _ReplayDone(Exception):
  pass


class FakeParams:
  def __init__(self, name, store=None):
    self.name = name
    self.store = {} if store is None else store
    self.writes = []   # (t, key, value) in call order
    self.gets = []     # keys read, in call order
    self.clock = None

  def get(self, key, return_default=False, block=False):
    self.gets.append(key)
    return self.store.get(key)

  def get_bool(self, key, block=False):
    self.gets.append(key)
    return bool(self.store.get(key, False))

  def put(self, key, value):
    self.store[key] = value
    self.writes.append((self.clock.t, key, value))

  put_nonblocking = put

  def put_bool(self, key, value):
    self.put(key, bool(value))

  put_bool_nonblocking = put_bool


class FakeLog:
  def __init__(self):
    self.events = []

  def event(self, name, **kw):
    self.events.append((name, kw))

  def _line(self, *a, **kw):
    self.events.append(("line", {"msg": a[0] if a else ""}))

  warning = error = info = debug = exception = _line


def gps_msg(lat, lon, has_fix=True, speed=0.0, bearing=0.0, vacc=5.0, unix_ms=0):
  return ("gpsLocation", {"latitude": lat, "longitude": lon, "hasFix": has_fix, "speed": speed,
                          "bearingDeg": bearing, "verticalAccuracy": vacc, "unixTimestampMillis": unix_ms})


def _build(service, fields):
  msg = messaging.new_message(service)
  body = getattr(msg, service)
  for k, v in fields.items():
    setattr(body, k, v)
  return messaging.log_from_bytes(msg.to_bytes())


def run(monkeypatch, steps, persistent=None, mem=None, mem_script=None, on_step=None, params_script=None):
  """Run main() over `steps`. `mem_script` = {t: {key: value}} applied to the mem store at the START
  of the step with that t (how a test models another process publishing a mem-param, e.g. CarGps);
  `params_script` does the same for the persistent store (e.g. card writing CarParams).
  `on_step(t, mem_store)` runs after each loop iteration has finished, i.e. what a consumer reading
  the mem store at that instant would see.

  Returns SimpleNamespace(mem, params, log, sm, sent) after the script is exhausted."""
  clock = SimpleNamespace(t=0.0)
  params = FakeParams("persistent", dict(persistent or {}))
  memp = FakeParams("/dev/shm/params", dict(mem or {}))
  params.clock = memp.clock = clock
  log = FakeLog()
  script = list(steps)
  mem_script = dict(mem_script or {})
  params_script = dict(params_script or {})
  holder = {"prev_t": None}
  sent = []

  def fake_params(path=None):
    return memp if path == "/dev/shm/params" else params

  def fake_submaster(services, **kw):
    monkeypatch.setattr(messaging, "sub_sock", lambda *a, **k: object())
    sm = _RealSubMaster(services, **kw)

    def update(timeout=100):
      if on_step is not None and holder["prev_t"] is not None:
        on_step(holder["prev_t"], memp.store)
      if not script:
        raise _ReplayDone
      t, msgs = script.pop(0)
      clock.t = holder["prev_t"] = t
      for key, val in mem_script.pop(t, {}).items():
        memp.store[key] = val
      for key, val in params_script.pop(t, {}).items():
        params.store[key] = val
      sm.update_msgs(t, [_build(s, f) for s, f in msgs])

    sm.update = update
    holder["sm"] = sm
    return sm

  fake_time = SimpleNamespace(monotonic=lambda: clock.t, time=lambda: 1.8e9 + clock.t, sleep=lambda s: None)
  monkeypatch.setattr(mapd_configd, "Params", fake_params)
  monkeypatch.setattr(mapd_configd, "cloudlog", log)
  monkeypatch.setattr(mapd_configd, "time", fake_time)
  monkeypatch.setattr(mapd_configd.messaging, "SubMaster", fake_submaster)
  monkeypatch.setattr(mapd_configd.messaging, "PubMaster", lambda services: SimpleNamespace(send=lambda s, m: sent.append((clock.t, s, m))))
  monkeypatch.setattr(mapd_configd, "_maps_on_disk", lambda: False)
  try:
    mapd_configd.main()
  except _ReplayDone:
    pass
  return SimpleNamespace(mem=memp, params=params, log=log, sm=holder.get("sm"), sent=sent)


def positions(res):
  """Every LastGPSPosition write as (t, decoded dict)."""
  return [(t, json.loads(v)) for t, k, v in res.mem.writes if k == "LastGPSPosition"]


def fix_events(res):
  return [kw for name, kw in res.log.events if name == "mapd_configd_gps_fix"]
