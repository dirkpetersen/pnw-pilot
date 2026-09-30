"""curvebrain2b2pnw stages 4 + 6 -- the Tesla VTSC consumes the shared curve brain (vtsc_controller._apply_brain).

The rules pinned, each by a test that fails when it is broken (mutation-checked, see the commit message):
  * LOWER-ONLY: with the brain on, the cap is never above the cap without it, whatever the entry says (fuzz);
  * shadow / off / a disagreeing brain / a stale, malformed or absent entry / a car without the capability all give a
    cap IDENTICAL to a VTSC that never heard of the brain -- and the ignored ones are counted and shown;
  * "lower" applies the need through VTSC's own decel envelope and rate limiter, is floored at V_MIN, is bounded by the
    freeway floor, follows the rain margin, dead-reckons the entry's age, and stops the instant the mode is not acting;
  * the Lightning's VTSC never even reads the param;
  * the VTSCStatus payload carries the cb* keys ces_pnw lifts into ces_events.
"""
import json
import math
import os
import random
import types

import pytest

from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import VTSC_TELE_KEYS
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_constants as C
from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as vc
from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_pnw import brake_cap_for_apex

MPH = 0.44704
V_SET = 33.5                    # 75 mph
TESLA = ("TESLA_MODEL_S_HW3", "tesla")
LIGHTNING = ("FORD_F_150_LIGHTNING_MK1", "ford")
CB_KEYS = ("cbUse", "cbAge", "cbWould", "cbWouldV", "vtscPre", "cbCap", "cbStaleN", "cbBadN", "cbMode")


class _CP:
  def __init__(self, fp, brand):
    self.carFingerprint, self.brand, self.openpilotLongitudinalControl, self.dashcamOnly = fp, brand, True, False


class _Params:
  def __init__(self, mode="2"):
    self.mode = mode

  def get(self, k, return_default=False):
    return self.mode if k == "CESMode" else None

  def get_bool(self, k):
    return False


_CLOCK = [1000.0]


class _Mem:
  """entry(t) -> the CurveBrain value (dict / str / None / an exception to raise). Records every key read."""
  def __init__(self, entry=lambda t: None, sl=None, freeway=False):
    self.entry, self.reads, self.sl, self.freeway = entry, [], sl, freeway

  def get(self, k, return_default=False):
    self.reads.append(k)
    if k == "CurveBrain":
      v = self.entry(_CLOCK[0])
      if isinstance(v, BaseException):
        raise v
      return v
    if k == "MapSpeedLimit":
      return str(self.sl) if self.sl else None
    if k == "RoadContext":
      return "freeway" if self.freeway else None
    return None

  def put_nonblocking(self, k, v):
    pass


class _NS:
  pass


def _sm():
  """A dead-straight road: vision has nothing, so any slowdown in these tests is the brain's (or nobody's)."""
  m = _NS()
  m.orientationRate, m.velocity, m.position, m.action = _NS(), _NS(), _NS(), _NS()
  m.orientationRate.z = [0.0] * 20
  m.orientationRate.t = [i * 0.25 for i in range(20)]
  m.velocity.x = [V_SET] * 20
  m.position.x = [V_SET * i * 0.25 for i in range(20)]
  m.action.shouldStop = False
  cc = _NS()
  cc.orientationNED = [0.0, 0.0, 0.0]
  cc.longActive = True    # vtscpass2pnw: engaged
  return {"modelV2": m, "carControl": cc}


@pytest.fixture(autouse=True)
def env(monkeypatch, tmp_path):
  _CLOCK[0] = 1000.0
  monkeypatch.setattr(vc, "time", types.SimpleNamespace(monotonic=lambda: _CLOCK[0]))
  path = tmp_path / "curve.json"
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(path))
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))
  return path


def _set_mode(path, mode):
  if mode is None:
    return
  path.write_text(json.dumps({"tesla": {"curve_brain": mode}}))


def entry(v=25.0, d=200.0, mode="lower", age=0.0, ev="measured"):
  return lambda t: {"ts": t - age, "seq": 1, "mode": mode, "v": v, "d": d, "src": "db", "ev": ev, "a": 3.0,
                    "k": 0.0048, "row": "1:0"}


def _drive(env_path, mode, entry_fn, car=TESLA, ticks=240, v_ego=None, mem=None, hook=None, v_set=V_SET):
  """A fresh controller at exactly 20 Hz. Per tick: (cap, payload). v_ego defaults to the set speed."""
  _set_mode(env_path, mode)
  _CLOCK[0] = 1000.0
  c = vc.VTSCController(_CP(*car), params=_Params())
  mem = mem or _Mem(entry_fn)
  c.mem_params = mem
  out = []
  for i in range(ticks):
    _CLOCK[0] = 1000.0 + i / 20.0
    if hook:
      hook(i, c)
    cap = c.cap(_sm(), v_set, v_ego if v_ego is not None else v_set)
    out.append((cap, c.overlay_payload()))
  return out, c, mem


def caps(run):
  return [r[0] for r in run]


# =====================================================================================================
# identity: everything that is not "acting on a fresh, agreeing entry" is a VTSC that never heard of the brain
# =====================================================================================================
class TestIdentity:
  def _baseline(self, env):
    return caps(_drive(env, "off", lambda t: None)[0])

  @pytest.mark.parametrize("why, mode, fn", [
    ("shadow", "shadow", entry(v=15.0, d=80.0)),
    ("off", "off", entry(v=15.0, d=80.0)),
    ("brain disagrees (its mode is shadow)", "lower", entry(v=15.0, d=80.0, mode="shadow")),
    ("brain disagrees (its mode is off)", "lower", entry(v=15.0, d=80.0, mode="off")),
    ("stale entry", "lower", entry(v=15.0, d=80.0, age=1.5)),
    ("an entry from the future", "lower", entry(v=15.0, d=80.0, age=-2.0)),
    ("malformed entry", "lower", lambda t: {"ts": t, "mode": "lower", "v": "fast", "d": 1.0, "ev": "measured"}),
    ("evidence VTSC may not act on", "lower", entry(v=15.0, d=80.0, ev="rated")),
    ("no entry at all", "lower", lambda t: None),
    ("a heartbeat with no need", "lower", lambda t: {"ts": t, "seq": 1, "mode": "lower", "v": None}),
    ("a need faster than VTSC's own cap", "lower", entry(v=40.0, d=10.0)),
    ("an unreadable param", "lower", lambda t: RuntimeError("shm gone")),
  ])
  def test_the_cap_is_exactly_todays(self, env, why, mode, fn):
    got = caps(_drive(env, mode, fn)[0])
    assert got == self._baseline(env), why

  def test_the_lightning_never_reads_the_param_and_is_unchanged(self, env):
    mem = _Mem(entry(v=15.0, d=80.0))
    run, c, _ = _drive(env, "lower", None, car=LIGHTNING, mem=mem)
    assert "CurveBrain" not in mem.reads
    assert caps(run) == caps(_drive(env, "off", lambda t: None, car=LIGHTNING)[0])
    assert all(r[1]["cbUse"] is None and r[1]["cbCap"] is None for r in run)

  def test_mode_off_does_not_even_read_the_param(self, env):
    mem = _Mem(entry(v=15.0, d=80.0))
    _drive(env, "off", None, mem=mem)
    assert "CurveBrain" not in mem.reads

  def test_a_car_without_the_capability_is_not_the_tesla(self):
    assert not pv.PnwVehicle(_CP(*LIGHTNING)).curve_brain_vtsc and pv.PnwVehicle(_CP(*TESLA)).curve_brain_vtsc


# =====================================================================================================
# the shadow: what it WOULD have done, control identical
# =====================================================================================================
class TestShadow:
  def test_it_logs_would_and_changes_nothing(self, env):
    run, _, _ = _drive(env, "shadow", entry(v=22.0, d=150.0))
    assert set(caps(run)) == {V_SET}
    p = run[-1][1]
    assert p["cbUse"] == "shadow" and p["cbWould"] is True and p["cbCap"] is None and p["cbMode"] == "shadow"
    assert p["vtscPre"] == pytest.approx(V_SET, abs=0.05) and p["cbAge"] == 0.0
    assert p["cbWouldV"] == pytest.approx(brake_cap_for_apex(22.0, 150.0, V_SET, C.DEFAULT_PROFILE["A_DECEL"]), abs=0.01)

  def test_would_is_false_when_the_brain_would_not_bind(self, env):
    p = _drive(env, "shadow", entry(v=40.0, d=10.0))[0][-1][1]
    assert p["cbWould"] is False and p["cbUse"] == "shadow"

  def test_the_ignored_entries_are_counted_and_named(self, env):
    run, _, _ = _drive(env, "lower", entry(v=15.0, d=80.0, age=1.5))
    p = run[-1][1]
    assert p["cbUse"] == "stale" and p["cbStaleN"] > 100 and p["cbBadN"] == 0
    run, _, _ = _drive(env, "lower", lambda t: {"ts": t, "mode": "lower", "v": "x", "d": 1, "ev": "measured"})
    p = run[-1][1]
    assert p["cbUse"] == "bad" and p["cbBadN"] > 100 and p["cbCap"] == pytest.approx(V_SET, abs=0.05)   # acting, not binding

  def test_an_ignored_entry_is_said_once_a_minute_with_the_consequence(self, env, monkeypatch):
    lines = []
    monkeypatch.setattr(vc.cloudlog, "error", lambda msg, *a, **k: lines.append(msg))
    _drive(env, "lower", entry(v=15.0, d=80.0, age=1.5), ticks=400)                  # 20 s of stale entries
    assert len(lines) == 1 and "ignored (stale" in lines[0] and "INACTIVE" in lines[0]
    lines.clear()
    _drive(env, "lower", lambda t: None, ticks=400)                                   # nothing published yet: normal
    _drive(env, "lower", lambda t: {"ts": t, "seq": 1, "mode": "lower", "v": None}, ticks=400)   # a live heartbeat: normal
    assert lines == []

  def test_an_unreadable_param_is_logged_once_a_minute_and_counted(self, env, monkeypatch):
    lines = []
    monkeypatch.setattr(vc.cloudlog, "exception", lambda msg, *a, **k: lines.append(msg))
    run, _, _ = _drive(env, "lower", lambda t: RuntimeError("shm gone"))
    assert len(lines) == 1 and "curve brain read FAILED" in lines[0] and "RuntimeError" in lines[0]
    assert run[-1][1]["cbUse"] == "bad" and run[-1][1]["cbBadN"] > 100


# =====================================================================================================
# lower: the need, bounded
# =====================================================================================================
class TestLower:
  def test_a_binding_need_lowers_the_cap_toward_the_envelope(self, env):
    run, c, _ = _drive(env, "lower", entry(v=22.0, d=0.0), ticks=400)
    cs = caps(run)
    assert cs[0] == pytest.approx(V_SET, abs=0.15) and cs[-1] == pytest.approx(22.0, abs=0.05) and min(cs) >= 22.0 - 1e-6
    p = run[-1][1]
    assert p["cbUse"] == "lower" and p["cbCap"] == pytest.approx(22.0, abs=0.05) and c._tele_curve_win == "brain"
    assert p["vtscPre"] == pytest.approx(V_SET, abs=0.05) and p["cbMode"] == "lower"

  def test_the_envelope_starts_the_slowdown_before_the_row_not_at_it(self, env):
    """d = 300 m at 33.5 m/s: the cap sits above the need until VTSC's decel envelope (1.2 m/s^2, finished 2.5 s early)
    says otherwise -- it must not step to 22 m/s the moment the row is seen."""
    run, _, _ = _drive(env, "lower", entry(v=22.0, d=300.0), ticks=40, v_ego=V_SET)
    want = brake_cap_for_apex(22.0, 300.0, V_SET, C.DEFAULT_PROFILE["A_DECEL"])
    assert want > 25.0
    assert caps(run)[-1] == pytest.approx(min(V_SET, want), abs=0.1)

  def test_the_drop_is_rate_limited_to_the_regen_decel(self, env):
    run, _, _ = _drive(env, "lower", entry(v=15.0, d=0.0), ticks=100)
    cs = caps(run)
    a_max = min(C.DEFAULT_PROFILE["A_DECEL_MAX"], C.REGEN_A_DECEL)
    steps = [b - a for a, b in zip(cs, cs[1:], strict=False)]
    assert min(steps) >= -a_max / 20.0 - 1e-9 and min(steps) < -0.05          # it does drop, never faster than regen

  def test_it_is_floored_at_v_min(self, env):
    run, _, _ = _drive(env, "lower", entry(v=1.0, d=0.0), ticks=600)
    assert min(caps(run)) == pytest.approx(C.V_MIN, abs=0.01)

  def test_rain_lowers_the_need_like_every_other_curve_speed(self, env):
    dry = caps(_drive(env, "lower", entry(v=22.0, d=0.0), ticks=400)[0])[-1]

    def rain(i, c):
      c.veh.set_rain_tier(2)
    wet = caps(_drive(env, "lower", entry(v=22.0, d=0.0), ticks=400, hook=rain)[0])[-1]
    assert wet == pytest.approx(dry - _rain_ms(), abs=0.05) and wet < dry - 1.0

  def test_the_freeway_floor_still_bounds_it(self, env):
    """T2 never crosses the freeway floor: a freeway curve needing 15 m/s at a 29 m/s posted limit stays at the limit."""
    mem = _Mem(entry(v=15.0, d=0.0), sl=29.06, freeway=True)
    run, _, _ = _drive(env, "lower", None, mem=mem, ticks=600)
    assert min(caps(run)) == pytest.approx(29.06, abs=0.05)
    off = _Mem(entry(v=15.0, d=0.0))
    assert min(caps(_drive(env, "lower", None, mem=off, ticks=600)[0])) < 16.0        # the same need without the floor

  def test_the_age_is_dead_reckoned(self, env):
    """The same need 100 m ahead, read 0.5 s late at 33.5 m/s, is 16.75 m closer: the envelope cap drops accordingly."""
    fresh = _drive(env, "lower", entry(v=22.0, d=250.0, age=0.0), ticks=30)[0][-1][1]["cbWouldV"]
    aged = _drive(env, "lower", entry(v=22.0, d=250.0, age=0.5), ticks=30)[0][-1][1]["cbWouldV"]
    assert aged == pytest.approx(brake_cap_for_apex(22.0, 250.0 - 33.5 * 0.5, V_SET, C.DEFAULT_PROFILE["A_DECEL"]), abs=0.05)
    assert aged < fresh - 0.5

  def test_the_gentle_profile_uses_its_own_decel(self, env):
    env_path = env
    _set_mode(env_path, "lower")

    class Gentle(_Params):
      mode = "1"
    _CLOCK[0] = 1000.0
    c = vc.VTSCController(_CP(*TESLA), params=Gentle("1"))
    c.mem_params = _Mem(entry(v=22.0, d=250.0))
    for i in range(30):
      _CLOCK[0] = 1000.0 + i / 20.0
      c.cap(_sm(), V_SET, V_SET)
    assert c.overlay_payload()["cbWouldV"] == pytest.approx(brake_cap_for_apex(22.0, 250.0, V_SET, C.GENTLE_PROFILE["A_DECEL"]),
                                                            abs=0.05)

  def test_it_releases_at_the_relax_rate_when_the_row_is_passed(self, env):
    fn = lambda t: entry(v=22.0, d=0.0)(t) if t < 1000.0 + 10.0 else {"ts": t, "seq": 2, "mode": "lower", "v": None}   # noqa: E731
    run, _, _ = _drive(env, "lower", fn, ticks=400)
    cs = caps(run)
    assert min(cs) == pytest.approx(22.0, abs=0.05) and cs[-1] == V_SET
    ups = [b - a for a, b in zip(cs, cs[1:], strict=False) if b > a]
    assert max(ups) <= C.A_RELAX / 20.0 + 1e-9

  def test_raise_is_lower_and_says_it_is_not_built(self, env, monkeypatch):
    errs = []
    monkeypatch.setattr(vc.cloudlog, "error", lambda msg, *a, **k: errs.append(msg))
    run, _, _ = _drive(env, "raise", entry(v=22.0, d=0.0), ticks=400)
    assert caps(run)[-1] == pytest.approx(22.0, abs=0.05) and run[-1][1]["cbUse"] == "lower"
    assert len([e for e in errs if "raise stage (T3) is NOT built" in e]) == 1


def _rain_ms():
  v = pv.PnwVehicle(_CP(*TESLA))
  v.set_rain_tier(2)
  return v.rain_penalty_ms()


# =====================================================================================================
# the kill switch, live
# =====================================================================================================
class TestKillSwitch:
  def _flip(self, env, to):
    def hook(i, c):
      if i == 200:
        c.veh._tesla_cfg_poll = -1e9                       # the poll clock is time.monotonic in production
        env.write_text(json.dumps({"tesla": {"curve_brain": to}}))
        st = os.stat(env)
        os.utime(env, ns=(st.st_atime_ns, st.st_mtime_ns + 5_000_000_000))
    return hook

  @pytest.mark.parametrize("to", ["shadow", "off"])
  def test_the_brain_term_is_dropped_within_a_second_of_the_edit(self, env, to):
    run, _, _ = _drive(env, "lower", entry(v=22.0, d=0.0), ticks=300, hook=self._flip(env, to))
    cs = caps(run)
    assert cs[199] == pytest.approx(22.0, abs=0.05)               # acting before the edit
    assert cs[-1] == V_SET                                        # released after it
    first_free = next(i for i in range(200, 300) if cs[i] == V_SET)
    assert first_free - 200 <= 40                                 # ticks at 20 Hz: the 1 s read cadence + one tick, not eased
    assert run[-1][1]["cbUse"] == to

  def test_a_typo_mid_drive_does_not_switch_it_back_on(self, env):
    def hook(i, c):
      if i == 100:
        c.veh._tesla_cfg_poll = -1e9
        env.write_text('{"tesla": {"curve_brain": "lowr"}}')
        st = os.stat(env)
        os.utime(env, ns=(st.st_atime_ns, st.st_mtime_ns + 5_000_000_000))
    run, c, _ = _drive(env, "shadow", entry(v=22.0, d=0.0), ticks=200, hook=hook)
    assert set(caps(run)) == {V_SET} and c.veh.curve_brain == "shadow"


# =====================================================================================================
# the property: lower-only, always
# =====================================================================================================
class TestLowerOnlyProperty:
  def test_with_the_brain_the_cap_is_never_above_the_cap_without(self, env):
    """500 random entries (need speed, distance, age, mode, evidence) x random driving state: for every tick the cap with
    the brain is <= the cap of an identically-driven VTSC with the brain off."""
    rng = random.Random(7)
    for trial in range(60):
      v_ego = rng.uniform(8.0, 40.0)
      v_set = rng.uniform(max(v_ego, 10.0), 42.0)
      spec = dict(v=rng.uniform(0.5, 60.0), d=rng.uniform(0.0, 700.0), age=rng.uniform(0.0, 1.2),
                  mode=rng.choice(["lower", "raise", "shadow", "lower"]), ev=rng.choice(["measured", "measured", "rated"]))
      mem_on = _Mem(entry(**spec), sl=rng.choice([None, 26.8]), freeway=rng.random() < 0.5)
      mem_off = _Mem(lambda t: None, sl=mem_on.sl, freeway=mem_on.freeway)
      on, _, _ = _drive(env, "lower", None, mem=mem_on, v_ego=v_ego, v_set=v_set, ticks=120)
      off, _, _ = _drive(env, "off", None, mem=mem_off, v_ego=v_ego, v_set=v_set, ticks=120)
      for (a, _), (b, _) in zip(on, off, strict=True):
        assert a <= b + 1e-9, (trial, spec, a, b)
        assert a >= min(C.V_MIN, b) - 1e-9 and math.isfinite(a)


# =====================================================================================================
# telemetry contract
# =====================================================================================================
class TestTelemetry:
  def test_the_cb_keys_are_on_the_lift_list_and_on_the_channel(self, env):
    for k in CB_KEYS:
      assert k in VTSC_TELE_KEYS
    run, _, _ = _drive(env, "lower", entry(v=22.0, d=0.0), ticks=50)
    p = run[-1][1]
    assert set(CB_KEYS) <= set(p)
    json.dumps(p)                                                   # JSON-safe: a NaN would lose the whole snapshot

  def test_the_lightning_reports_nulls_not_missing_keys(self, env):
    p = _drive(env, "off", lambda t: None, car=LIGHTNING, ticks=10)[0][-1][1]
    assert set(CB_KEYS) <= set(p) and p["cbUse"] is None and p["cbStaleN"] == 0


# =====================================================================================================
# Opus review: the brain term's state must not outlive a gap in which VTSC did not run
# =====================================================================================================
class TestNoStaleBrainState:
  def _run(self, env, gap):
    """Act in a curve (need 20 m/s), then `gap` ticks off, then a straight road with no need: the cap must be the set."""

    def fn(t):
      return entry(v=20.0, d=0.0)(t) if t < 1000.0 + 5.0 else {"ts": t, "seq": 2, "mode": "lower", "v": None}

    def hook(i, c):
      if gap == "ces":
        c.params.mode = "0" if 100 <= i < 140 else "2"
        c._last_read = -1e9
      elif gap == "model" and 100 <= i < 140:
        c._model_gone = True
      elif gap == "model":
        c._model_gone = False
    _set_mode(env, "lower")
    _CLOCK[0] = 1000.0
    c = vc.VTSCController(_CP(*TESLA), params=_Params())
    c.mem_params = _Mem(fn)
    out = []
    for i in range(240):
      _CLOCK[0] = 1000.0 + i / 20.0
      hook(i, c)
      sm = _sm()
      if getattr(c, "_model_gone", False):
        sm = {}
      out.append(c.cap(sm, V_SET, V_SET))
    return out

  @pytest.mark.parametrize("gap", ["ces", "model"])
  def test_no_phantom_brake_after_a_gap(self, env, gap):
    out = self._run(env, gap)
    assert min(out[:100]) < 26.0                       # it really was acting before the gap
    assert min(out[140:]) > V_SET - 0.2, min(out[140:])  # and nothing is left over after it

  def test_reset_clears_the_brain_term(self, env):
    c = vc.VTSCController(_CP(*TESLA), params=_Params())
    c._cb_applied = 12.0
    c._reset()
    assert c._cb_applied is None
