"""icbmdbraise2pnw review fixes, closed loop through the real Ford executor + SpeedAdjustController (harness: test_zone_set_closed_loop).

Gemini 4 Argon review of 97cfcdce68:
  1. a DB raise's SET+ tap still in flight when the curve clears must not poison the restore baseline (the restore died silently);
  2. speedadjust must read the DB raise's SET+ taps as ICBM's own, never as the driver's override.
"""
import pytest

from openpilot.selfdrive.controls.lib.speedadjust_pnw.tests.test_zone_set_closed_loop import MPH, simulate

LIMIT = 50


def _raise_then_clear(t_end):
  """Set 61 walked to 47 by a curve cap, then the DB raises to 52.6 from t=15 and the curve ends at t_end."""
  def script(t):
    if t < 5 or t >= t_end:
      return LIMIT, None, None
    return LIMIT, (47 if t < 15 else 52.6), None
  return script


def _db(t_end):
  return lambda t: 52.6 if 15 <= t < t_end else None


class TestRestoreSurvivesAnInFlightRaiseTap:
  @pytest.mark.parametrize("t_end", [17.0 + 0.25 * i for i in range(13)])
  def test_the_restore_still_returns_to_the_ceiling(self, monkeypatch, t_end):
    final, trace, _ = simulate(monkeypatch, _raise_then_clear(t_end), 90, v0_mph=61, db_script=_db(t_end))
    assert any(r[1] > 48 for r in trace if 15 <= r[0] <= 22), "the DB raise never lifted the set: this proves nothing"
    assert final == pytest.approx(61, abs=0.01), (t_end, trace[-3:])


class TestSpeedadjustReadsTheRaiseTapsAsIcbms:
  def test_no_override_verdict_while_the_raise_taps(self, monkeypatch):
    final, trace, c = simulate(monkeypatch, _raise_then_clear(30.0), 60, v0_mph=61, db_script=_db(30.0))
    assert max(r[1] for r in trace if 15 <= r[0] <= 30) > 48, "no raise tap happened: this proves nothing"
    assert "applied" not in c._ovr_seen, c._ovr_seen


def _police(t):
  from openpilot.selfdrive.controls.lib.speedadjust_pnw.tests.test_zone_set_closed_loop import POLICE
  return POLICE if 8 <= t < 90 else None


class TestAPoliceCapIsNotDismissedByTheRaise:
  """The review's case: speedadjust holds a cap (_cap_out set) when the curve brain's DB-raise SET+ arrives."""

  @pytest.mark.parametrize("why, verdict", [("dbRaise", "icbmTap"), (None, "applied")])
  def test_the_dbraise_marker_decides_whose_tap_it_is(self, monkeypatch, why, verdict):
    from openpilot.selfdrive.controls.lib.speedadjust_pnw.tests.test_zone_set_closed_loop import POLICE, _Clock, _SM
    f, _, c = simulate(monkeypatch, lambda t: (60, (35 if 5 <= t < 30 else None), _police(t)), 40, v0_mph=75)
    assert c._cap_out is not None, "speedadjust holds no cap: this proves nothing"
    sm, s = _SM(), f * MPH
    for dv in (0.0, MPH):
      _Clock.t += 1.0
      sm.cs.cruiseState.speed = s + dv
      c.mem_params.icbm = {"target": s + 10 * MPH, "ceiling": 75 * MPH, "ts": _Clock.t, "dir": "inc", **({"why": why} if why else {})}
      c.mem_params.police, c.mem_params.sl = POLICE, 60 * MPH
      c.cap(sm, s + dv, s + dv, s + dv, True)
    assert c._ovr == verdict          # a SET+ with no marker during our own cap is still the driver's
