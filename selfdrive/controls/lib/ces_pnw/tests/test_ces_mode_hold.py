"""cesmodehold2pnw -- an unreadable CES master keeps the LAST GOOD mode for 10 s, then falls back and says so.

silentexc3pnw made read_ces_mode() log its failed reads; it still switched that caller's CES master Off on the very read
that failed (owner decision 2026-09-14 on Fable's Q1: hold instead). Each caller (`who`: CES in selfdrived, VTSC in
plannerd, the CES overlay in ui -- module state, so per process) now keeps its last successfully read mode for
CES_MODE_HOLD_S, then falls back to what the read computed. The window is BOUNDED because the failure this guards in
practice -- a params_keys.h / params_pyx.so mismatch -- is not transient. The UI overlay raises its NO-SIGNAL alarm once
the hold has expired (not during it: the held mode is the driver's own and CES is still running and publishing on it).

Identity: with no failed read, every car and every mode behaves EXACTLY as before the hold existed (proved against the
pre-change behaviour, `return mode`).
"""
import logging
import types

import pytest

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_ces_mode_read_failure_logged import (
  CARS, SCENES, UKN, UKN_LEGACY, _Records, _ces_run, _P, _vtsc_drive,
)


@pytest.fixture
def logs():
  h = _Records()
  cloudlog.addHandler(h)
  yield h
  cloudlog.removeHandler(h)


@pytest.fixture(autouse=True)
def fresh_state(monkeypatch, tmp_path):
  """Each test starts with no failure history and no held mode. Lightnings read the built-in curve defaults."""
  monkeypatch.setattr(C, "_ces_mode_read_err", {})
  monkeypatch.setattr(C, "_ces_mode_hold_st", {})
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "absent.json"))
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))


@pytest.fixture
def clock(monkeypatch):
  """One fake clock for the constants module, VTSC, CES and the UI overlay."""
  from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
  from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_controller as vc
  from openpilot.selfdrive.ui.onroad import ces_status as cs
  t = [1000.0]
  ns = types.SimpleNamespace(monotonic=lambda: t[0], time=lambda: t[0])
  for mod in (C, vc, m, cs):
    monkeypatch.setattr(mod, "time", ns)
  return t


def _reads(clock, who, mode, secs, legacy=lambda t: False):
  """read_ces_mode once a second (what CES / VTSC / the overlay do) -> [(t, mode), ...]."""
  p = _P(clock, mode=mode, legacy=legacy)
  out = []
  for t in secs:
    clock[0] = 1000.0 + t
    out.append((t, C.read_ces_mode(p, who=who)))
  return out


def _hold_lines(logs, who):
  """The change-only hold lines for one caller (NOT the rate-limited read-failure line, which names its param key)."""
  head = f"read_ces_mode ({who}): CES master "
  return [r for r in logs.records if isinstance(r.msg, str) and r.msg.startswith(head)]


# ---------------------------------------------------------------- the hold itself
def test_a_failure_during_light_holds_light_for_ten_seconds_then_off(clock, logs):
  """The owner's case: driving in Light (1), the master goes unreadable -> Light for 10 s, then Off."""
  assert C.CES_MODE_HOLD_S == 10.0
  got = _reads(clock, "VTSC", lambda t: 1 if t < 5.0 else UKN, range(20))
  assert [m for t, m in got if t < 5] == [1] * 5                     # positive control: reads work
  assert [m for t, m in got if 5 <= t < 15] == [1] * 10              # held for exactly 10 s of failed reads
  assert [m for t, m in got if t >= 15] == [0] * 5                   # then the fallback
  lines = _hold_lines(logs, "VTSC")
  assert len(lines) == 2                                             # change-only, not once per read
  assert "holds its last good mode 1 for up to 10 s" in lines[0].msg
  assert "the 10 s hold of mode 1 expired" in lines[1].msg and "falls back to mode 0" in lines[1].msg
  assert [r.levelno for r in lines] == [logging.WARNING, logging.ERROR]
  assert len(logs.mode("VTSC")) == 1                                 # the read-failure log is still rate-limited


def test_a_recovery_inside_the_hold_resets_it(clock, logs):
  """Reads that come back before the window is up leave no trace in the mode, and restart the window if they fail again."""
  # fails 5..7, good again at 8, fails from 9 -> the second outage holds its own full 10 s (9..18), Off from 19
  def mode(t):
    return UKN if (5.0 <= t < 8.0 or t >= 9.0) else 1
  got = dict(_reads(clock, "CES", mode, range(25)))
  assert [got[t] for t in (5, 6, 7)] == [1, 1, 1]                    # held through the first outage
  assert got[8] == 1                                                 # recovered
  assert [got[t] for t in (9, 17, 18)] == [1, 1, 1]                  # second outage: the window restarted at t=9
  assert [got[t] for t in (19, 24)] == [0, 0]
  lines = _hold_lines(logs, "CES")   # hold started / readable again / hold started / expired
  assert len(lines) == 4 and "readable again after 3.0 s -- mode 1" in lines[1].msg
  assert C.ces_mode_lost("CES") is True                              # ... and the overlay would alarm now


def test_a_failure_before_any_good_read_is_off_at_once(clock, logs):
  """Boot with a broken params layer: there is no mode to hold, so Off immediately -- today's behaviour."""
  got = _reads(clock, "CES", lambda t: UKN, range(3))
  assert [m for _, m in got] == [0, 0, 0]
  lines = _hold_lines(logs, "CES")
  assert len(lines) == 1 and "no good read yet (it failed at startup)" in lines[0].msg
  assert "falls back to mode 0" in lines[0].msg and lines[0].levelno >= logging.ERROR
  assert C.ces_mode_lost("CES") is True


def test_an_unreadable_legacy_bool_holds_the_migrated_mode_too(clock, logs):
  """A device still migrating on the legacy bool: CESMode reads a genuine 0 and the bool says Standard, so the mode is
  2. When the BOOL is what goes unreadable, the effective mode is just as lost -> hold 2 for 10 s, then Off."""
  got = dict(_reads(clock, "CES", lambda t: 0, range(20), legacy=lambda t: True if t < 5.0 else UKN_LEGACY))
  assert [got[t] for t in (0, 4)] == [2, 2]                          # positive control: the migration works
  assert [got[t] for t in (5, 14)] == [2, 2]                         # held
  assert [got[t] for t in (15, 19)] == [0, 0]                        # then the fallback
  lines = _hold_lines(logs, "CES")
  assert len(lines) == 2 and "holds its last good mode 2" in lines[0].msg
  assert len(logs.mode("CES", "ConditionalExperimentalSwitching")) == 1


def test_each_caller_holds_on_its_own_clock(clock, logs):
  """CES, VTSC and the overlay are separate processes; here they even fail at different times."""
  fail_at = {"CES": 2.0, "VTSC": 7.0, "CES overlay": 7.0}
  got = {who: dict(_reads(clock, who, lambda t, a=a: UKN if t >= a else 2, range(20))) for who, a in fail_at.items()}
  assert got["CES"][11] == 2 and got["CES"][12] == 0                 # held 2..11, falls back at 12
  assert got["VTSC"][16] == 2 and got["VTSC"][17] == 0               # held 7..16, falls back at 17
  assert got["CES overlay"][16] == 2 and got["CES overlay"][17] == 0
  # ... and while CES has already fallen back, VTSC is still holding -- one caller's outage never moves another's
  assert C.ces_mode_lost("CES") and C.ces_mode_lost("VTSC")
  for who in fail_at:
    assert len(_hold_lines(logs, who)) == 2


def test_a_healthy_caller_is_untouched_by_a_broken_one(clock, logs):
  """VTSC reading fine must not inherit CES's hold (the state is keyed per caller, not global)."""
  p_bad, p_good = _P(clock, mode=lambda t: UKN), _P(clock, mode=lambda t: 0)
  for i in range(20):
    clock[0] = 1000.0 + i
    assert C.read_ces_mode(p_bad, who="CES") == 0
    assert C.read_ces_mode(p_good, who="VTSC") == 0                  # live reads keep winning
  assert C.ces_mode_lost("CES") is True and C.ces_mode_lost("VTSC") is False


def test_ces_mode_lost_is_false_while_reads_work_during_the_hold_and_when_ces_was_off(clock, logs):
  """The overlay's alarm condition: only once the fallback is in force AND it is not what the driver chose."""
  got = _reads(clock, "CES overlay", lambda t: 1 if t < 5.0 else UKN, [0, 4])
  assert [m for _, m in got] == [1, 1] and C.ces_mode_lost("CES overlay") is False        # reads fine
  _reads(clock, "CES overlay", lambda t: UKN, [5, 14])
  assert C.ces_mode_lost("CES overlay") is False                                          # during the hold
  _reads(clock, "CES overlay", lambda t: UKN, [15])
  assert C.ces_mode_lost("CES overlay") is True                                           # hold expired
  # a driver who had CES OFF loses nothing when the fallback (Off) takes over -> no alarm
  _reads(clock, "ces-off", lambda t: 0 if t < 30.0 else UKN, [20, 30, 45])
  assert C.ces_mode_lost("ces-off") is False
  assert C.ces_mode_lost("never-read") is False


def test_a_dead_logger_cannot_change_the_held_mode_or_escape(clock, logs, monkeypatch):
  """The UI overlay calls read_ces_mode with no try of its own -- nothing in here may raise."""
  def boom(*a, **k):
    raise RuntimeError("logger down")
  for level in ("exception", "error", "warning", "info"):
    monkeypatch.setattr(C.cloudlog, level, boom)
  got = _reads(clock, "CES overlay", lambda t: 2 if t < 3.0 else (1 if t == 8.0 else UKN), range(25))
  assert [m for t, m in got if 3 <= t < 8] == [2] * 5                # held
  assert [m for t, m in got if 8 <= t < 19] == [1] * 11              # recovered at 8, then held the NEW mode
  assert [m for t, m in got if t >= 19] == [0] * 6
  assert C.ces_mode_lost("CES overlay") is True


def test_a_broken_hold_still_returns_the_read_and_is_logged(clock, logs, monkeypatch):
  """Rule 2: if the hold bookkeeping itself fails, the caller still gets this read's mode AND the failure is logged."""
  class Broken(dict):
    def setdefault(self, *a):
      raise RuntimeError("hold state corrupt")
  monkeypatch.setattr(C, "_ces_mode_hold_st", Broken())
  assert C.read_ces_mode(_P(clock, mode=lambda t: 1), who="VTSC") == 1
  assert C.read_ces_mode(_P(clock, mode=lambda t: UKN), who="VTSC") == 0
  lines = logs.mode("VTSC", "mode hold")
  assert len(lines) == 1 and lines[0].levelno >= logging.ERROR and "(RuntimeError)" in lines[0].msg
  assert "uses this read's own mode" in lines[0].msg


# ---------------------------------------------------------------- identity when nothing fails (both cars, all modes)
@pytest.fixture
def no_hold(monkeypatch):
  """The pre-cesmodehold2pnw behaviour of read_ces_mode: whatever this read computed, with no hold."""
  monkeypatch.setattr(C, "_ces_mode_hold", lambda who, mode, ok: mode)


def test_read_ces_mode_is_identical_for_every_good_read(clock, logs, no_hold):
  for v, want in ((0, 0), (1, 1), (2, 2), (None, 0)):
    assert C.read_ces_mode(_P(clock, mode=lambda t, v=v: v), who="CES") == want
  assert C.read_ces_mode(_P(clock, mode=lambda t: 0, legacy=lambda t: True), who="CES") == 2
  assert logs.errors() == []


@pytest.mark.parametrize("fp, brand", CARS, ids=["tesla", "lightning-oplong"])
@pytest.mark.parametrize("mode", [0, 1, 2], ids=["off", "light", "standard"])
def test_vtsc_identity_when_nothing_fails(clock, logs, request, fp, brand, mode):
  """Every car x every mode: a healthy plannerd drives bit-identically with and without the hold."""
  got = _vtsc_drive(clock, fp, brand, lambda t, v=mode: v)
  request.getfixturevalue("no_hold")
  assert got == _vtsc_drive(clock, fp, brand, lambda t, v=mode: v)
  assert logs.errors() == []
  if mode:
    assert min(cap for cap, _, _ in got) < 30.3                      # positive control: an enabled VTSC brakes


@pytest.mark.parametrize("fp, brand, op_long, lead, curve0, v0, what", SCENES)
@pytest.mark.parametrize("mode", [0, 1, 2], ids=["off", "light", "standard"])
def test_ces_identity_when_nothing_fails(clock, logs, request, fp, brand, op_long, lead, curve0, v0, what, mode):
  """Every car x every mode: a healthy selfdrived decides bit-identically with and without the hold."""
  got = _ces_run(clock, fp, brand, op_long, lambda t, v=mode: v, lead, curve0, v0)
  request.getfixturevalue("no_hold")
  assert got == _ces_run(clock, fp, brand, op_long, lambda t, v=mode: v, lead, curve0, v0)
  assert logs.errors() == []
  acted = sum(got["dec"]) if what == "dec" else sum(1 for _, k, p in got["puts"]
                                                   if k == "IcbmTarget" and p.get("target") is not None)
  assert (acted > 20) if mode else (acted == 0)                      # positive control


# ---------------------------------------------------------------- the UI overlay's NO-SIGNAL alarm
def _ui(monkeypatch, clock, params):
  """A CesStatusRenderer poll loop with no raylib: fonts are never touched, text width is stubbed."""
  from openpilot.selfdrive.ui.onroad import ces_status as cs
  monkeypatch.setattr(cs, "ui_state", types.SimpleNamespace(params=params, started=True, is_metric=False, sm={}))
  monkeypatch.setattr(cs, "measure_text_cached", lambda font, text, fs: types.SimpleNamespace(x=100.0))

  class Mem:   # a HEALTHY CES publisher: fresh ts, CES itself idle so no card/line layout is built
    def get(self, k, return_default=False):
      return {"ts": clock[0], "enabled": False} if k == "CESStatus" else None

  r = object.__new__(cs.CesStatusRenderer)
  r.__dict__.update(_last_poll=-1e9, _mem=Mem(), _onroad_t0=None, _ces_enabled=False, _no_signal=False, _dump=False,
                    _st={}, _vtsc={}, _mapdl="", _last_spdlim=None, _spdlim_change_t=0.0, _fordlat={}, _steerfault_n=0,
                    _clean_streak=0, _4sig_ok=False, _cached_layout=None, _card_layout=None, font=None, font_bold=None)
  return r


def _ui_run(monkeypatch, clock, mode, T, legacy=lambda t: False):
  """Poll at 5 Hz for T seconds -> {t: (no_signal, alarm shown, CESStatus data still held)}."""
  r = _ui(monkeypatch, clock, _P(clock, mode=mode, legacy=legacy))
  out = {}
  for i in range(int(T * 5) + 1):
    clock[0] = 1000.0 + i / 5.0
    r._update_state()
    out[round(i / 5.0, 1)] = (r._no_signal, r._cached_layout is not None, bool(r._st))
  return out


def test_the_overlay_holds_its_display_then_alarms_when_the_hold_expires(clock, logs, monkeypatch):
  """The NO-SIGNAL choice: quiet during the hold (the held mode IS the driver's and CES is publishing on it),
  alarm once the fallback is in force."""
  got = _ui_run(monkeypatch, clock, lambda t: 1 if t < 15.0 else UKN, 30.0)
  assert got[14.0] == (False, False, True)                 # healthy, past the 10 s onroad grace
  assert got[15.0] == (False, False, True) and got[24.8] == (False, False, True)   # in the hold: nothing changes
  assert got[25.0] == (True, True, False)                  # hold expired -> the red NO-SIGNAL alarm, data dropped
  assert got[30.0] == (True, True, False)
  assert "the 10 s hold of mode 1 expired" in _hold_lines(logs, "CES overlay")[1].msg


def test_the_overlay_alarm_still_waits_for_the_onroad_grace(clock, logs, monkeypatch):
  """A read broken at boot has no mode to hold, but the dead-man's 10 s onroad grace still applies (spawn window)."""
  got = _ui_run(monkeypatch, clock, lambda t: UKN, 15.0)
  assert got[0.0] == (False, False, False) and got[9.8] == (False, False, False)
  assert got[10.2] == (True, True, False) and got[15.0] == (True, True, False)


def test_the_overlay_stays_silent_when_the_driver_had_ces_off(clock, logs, monkeypatch):
  """CES Off + an unreadable master = the same blank screen as CES Off: the fallback took nothing away."""
  got = _ui_run(monkeypatch, clock, lambda t: 0 if t < 15.0 else UKN, 30.0)
  assert set(got.values()) == {(False, False, False)}
  off = _ui_run(monkeypatch, clock, lambda t: 0, 30.0)
  assert got == off                                        # identical to plain CES Off


def test_the_overlay_never_raises_when_the_master_is_unreadable(clock, logs, monkeypatch):
  """A UI crash-loop looks like a hardware fault on this device -- _update_state must not raise on any of these."""
  for mode, legacy in ((lambda t: UKN, lambda t: False), (lambda t: UKN, lambda t: UKN_LEGACY),
                       (lambda t: 0, lambda t: UKN_LEGACY), (lambda t: "abc", lambda t: False)):
    C._ces_mode_hold_st.clear()
    assert len(_ui_run(monkeypatch, clock, mode, 15.0, legacy=legacy)) == 76
