"""mapdgrace2pnw: a tile that is still loading at the first fix must not trigger a whole-state download.

mapd loads a tile only while it processes a gpsLocation message, and mapd_configd reacts to that same
message first. On 111 of 113 observed boots 09-02..09-14 that race sent a whole-state download request
(OR ~161 MB of archives) for maps already on disk, mostly over LTE
(drives/2026-09-14/map-redownload/DRIVE_REPORT.md, raw/rlog_race_output.txt, raw/classify_summary.txt).

Every test runs the REAL mapd_configd.main() loop through configd_replay.py.
"""
import math

import pytest

from openpilot.system.mapd import mapd_configd
from openpilot.system.mapd.tests import configd_replay as R

GRACE = mapd_configd.COVERAGE_GRACE_S
OR_CORVALLIS = (44.5685, -123.3144)   # where the 09-14 boots happened (report s5)
WA_VANCOUVER = (45.70, -122.67)
OR_PORTLAND = (45.50, -122.67)

# The two full-rate boot rlogs (raw/rlog_race_output.txt), seconds after mapd_configd's loop started
# receiving mapdOut: (mapdOut first, first GPS message = first fix, tileLoaded -> True).
BOOT_1236 = (46.33, 53.13, 53.28)   # 0000015e: request went out at 53.16, the tile loaded at 53.28
BOOT_1259 = (45.04, 55.09, 55.19)   # 0000015f: request 55.12, tile 55.19
# Every distinct "tile loaded after the trigger" delay over the 111 race boots (raw/classify_summary.txt,
# 1 Hz qlogs: median 0.7 s, max 1.9 s), plus 2.9 s = the max plus that 1 Hz sampling uncertainty.
QLOG_TILE_DELAYS = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.9, 2.9]


def _grid(t):
  return round(math.ceil(round(t / 0.05, 6)) * 0.05, 3)


def _script(t_end, gps, tile, mapd_out=(0.0, None), ext=(0.0, None)):
  """A 20 Hz loop. `gps` = [(t, lat, lon, has_fix)], each delivered on the first loop at or after t.
  `tile(t)` = mapdOut.tileLoaded at t. mapdOut publishes at 20 Hz inside `mapd_out` = (start, stop) and
  mapdExtendedOut at 1 Hz inside `ext` (stop None = to the end)."""
  steps = {}
  n = int(round(t_end / 0.05))
  for k in range(n + 1):
    t = round(k * 0.05, 3)
    msgs = []
    if mapd_out is not None and mapd_out[0] <= t and (mapd_out[1] is None or t < mapd_out[1]):
      msgs.append(("mapdOut", {"tileLoaded": bool(tile(t))}))
    if ext is not None and ext[0] <= t and (ext[1] is None or t < ext[1]) and round((t - _grid(ext[0])) / 0.05) % 20 == 0:
      msgs.append(("mapdExtendedOut", {}))
    steps[t] = msgs
  for t, lat, lon, fix in gps:
    g = _grid(t)
    if g <= t_end:
      steps[g] = steps.get(g, []) + [R.gps_msg(lat, lon, has_fix=fix, vacc=5.0 if fix else 500.0)]
  return sorted(steps.items())


def _fixes(t0, t1, ll, fix=True):
  return [(t0 + i, ll[0], ll[1], fix) for i in range(int(round(t1 - t0)))]


def _requests(res):
  return [(t, m.mapdIn.str) for t, s, m in res.sent if s == "mapdIn"]


def _lines(res, needle):
  return [kw["msg"] for name, kw in res.log.events if name == "line" and needle in kw["msg"]]


class TestBootRace:
  @pytest.mark.parametrize("boot", [BOOT_1236, BOOT_1259], ids=["09-14_12:36", "09-14_12:59"])
  def test_full_rate_boot_sends_no_request(self, monkeypatch, boot):
    mapd_first, fix_t, tile_t = boot
    res = R.run(monkeypatch, _script(fix_t + 40.0, _fixes(fix_t, fix_t + 40.0, OR_CORVALLIS),
                                     tile=lambda t: t >= tile_t, mapd_out=(mapd_first, None), ext=(mapd_first, None)))
    assert _requests(res) == [], "the boot race still downloads a whole state"
    resolved = _lines(res, "coverage grace resolved")
    assert len(resolved) == 1 and "avoided_request=True" in resolved[0], res.log.events
    took = float(resolved[0].split("after ")[1].split(" s")[0])
    assert took == pytest.approx(tile_t - fix_t, abs=0.051)
    assert len(_lines(res, "coverage grace start OR")) == 1
    assert _lines(res, "coverage grace expired") == []

  @pytest.mark.parametrize("delay", QLOG_TILE_DELAYS)
  def test_every_observed_tile_delay_sends_no_request(self, monkeypatch, delay):
    fix_t = 12.0
    res = R.run(monkeypatch, _script(fix_t + GRACE + 5.0, _fixes(fix_t, fix_t + GRACE + 5.0, OR_CORVALLIS),
                                     tile=lambda t: t >= fix_t + delay))
    assert _requests(res) == []
    assert len(_lines(res, "avoided_request=True")) == 1

  def test_tile_loaded_at_the_first_fix_logs_no_episode(self, monkeypatch):
    res = R.run(monkeypatch, _script(30.0, _fixes(10.0, 30.0, OR_CORVALLIS), tile=lambda t: t >= 5.0))
    assert _requests(res) == [] and _lines(res, "coverage grace") == []

  def test_avoided_request_is_false_when_the_old_code_would_not_have_sent(self, monkeypatch):
    """mapdExtendedOut silent: the pre-grace code would not have requested either, so it is not counted."""
    res = R.run(monkeypatch, _script(25.0, _fixes(10.0, 25.0, OR_CORVALLIS), tile=lambda t: t >= 10.5, ext=None))
    resolved = _lines(res, "coverage grace resolved")
    assert len(resolved) == 1 and "avoided_request=False" in resolved[0]

  def test_avoided_request_is_counted_per_episode(self, monkeypatch):
    """The boot race is avoided (mapdExtendedOut alive); a later blip after mapdExtendedOut went silent
    at 14 s (alive to 24 s) is not, and must not inherit the first episode's count."""
    res = R.run(monkeypatch, _script(30.0, _fixes(10.0, 30.0, OR_CORVALLIS),
                                     tile=lambda t: t >= 10.3 and not 25.0 <= t < 26.0, ext=(0.0, 14.0)))
    resolved = _lines(res, "coverage grace resolved")
    assert len(resolved) == 2, resolved
    assert "avoided_request=True" in resolved[0] and "avoided_request=False" in resolved[1], resolved


class TestGenuinelyUncovered:
  def test_requests_once_when_the_window_expires(self, monkeypatch):
    fix_t = 10.0
    res = R.run(monkeypatch, _script(fix_t + 40.0, _fixes(fix_t, fix_t + 40.0, OR_CORVALLIS), tile=lambda t: False))
    reqs = _requests(res)
    assert [k for _, k in reqs] == ["us_state.OR"], reqs
    assert fix_t + GRACE <= reqs[0][0] <= fix_t + GRACE + 0.05, reqs
    expired = _lines(res, "coverage grace expired")
    assert len(expired) == 1 and "request_due=True" in expired[0]

  def test_window_boundary(self, monkeypatch):
    """Not one loop early: 9.95 s after the first fix is still inside the window."""
    fix_t = 10.0
    res = R.run(monkeypatch, _script(fix_t + GRACE - 0.05, _fixes(fix_t, fix_t + GRACE, OR_CORVALLIS), tile=lambda t: False))
    assert _requests(res) == []
    res = R.run(monkeypatch, _script(fix_t + GRACE, _fixes(fix_t, fix_t + GRACE + 1.0, OR_CORVALLIS), tile=lambda t: False))
    assert len(_requests(res)) == 1

  def test_backoff_still_governs_the_retries(self, monkeypatch):
    fix_t = 10.0
    res = R.run(monkeypatch, _script(fix_t + 75.0, _fixes(fix_t, fix_t + 75.0, OR_CORVALLIS), tile=lambda t: False))
    reqs = _requests(res)
    assert len(reqs) == 2, reqs
    assert reqs[1][0] - reqs[0][0] >= mapd_configd.REGION_RESEND_INTERVAL_S


class TestMidDrive:
  def test_crossing_into_an_uncovered_state_requests_it(self, monkeypatch):
    """Covered in WA, cross the line into OR where the square is missing: OR is requested after the window."""
    cross = 30.0
    gps = _fixes(10.0, cross, WA_VANCOUVER) + _fixes(cross, cross + 20.0, OR_PORTLAND)
    res = R.run(monkeypatch, _script(cross + 20.0, gps, tile=lambda t: t < cross))
    reqs = _requests(res)
    assert [k for _, k in reqs] == ["us_state.OR"], reqs
    assert cross + GRACE <= reqs[0][0] <= cross + GRACE + 0.05

  def test_crossing_out_of_an_uncovered_state_does_not_race_the_new_square(self, monkeypatch):
    """Uncovered in WA (window expired, WA requested), then into OR whose square mapd loads from the
    same GPS message: the OR region gets its own window, so OR is not requested."""
    cross = 30.0
    gps = _fixes(10.0, cross, WA_VANCOUVER) + _fixes(cross, cross + 20.0, OR_PORTLAND)
    res = R.run(monkeypatch, _script(cross + 20.0, gps, tile=lambda t: t >= cross + 0.3))
    assert [k for _, k in _requests(res)] == ["us_state.WA"]
    assert len(_lines(res, "coverage grace start OR (region change")) == 1

  def test_a_fix_jittering_across_a_state_edge_still_requests(self, monkeypatch):
    """A region that flips WA/OR/WA on every message must not hold a genuinely uncovered area off forever."""
    gps = [(10.0 + i, *(WA_VANCOUVER if i % 2 == 0 else OR_PORTLAND), True) for i in range(30)]
    res = R.run(monkeypatch, _script(40.0, gps, tile=lambda t: False))
    assert sorted(k for _, k in _requests(res)) == ["us_state.OR", "us_state.WA"], _requests(res)

  def test_a_fix_gap_restarts_the_window(self, monkeypatch):
    """Fixes 10-13, receiver silent until 20 (has_fix stays `alive` for 10 s): the window restarts at 20."""
    gps = _fixes(10.0, 14.0, OR_CORVALLIS) + _fixes(20.0, 45.0, OR_CORVALLIS)
    res = R.run(monkeypatch, _script(45.0, gps, tile=lambda t: False))
    reqs = _requests(res)
    assert len(reqs) == 1 and 20.0 + GRACE <= reqs[0][0] <= 20.0 + GRACE + 0.05, reqs
    assert len(_lines(res, "coverage grace reset")) == 1 and "GPS fix lost" in _lines(res, "coverage grace reset")[0]

  def test_a_gap_on_a_10hz_receiver_restarts_the_window(self, monkeypatch):
    """gpsLocationExternal (ubloxd, 10 Hz) is `alive` for only 1 s, less than the 3 s GPS_SILENT_S: a 2 s gap
    ends the episode there too, instead of carrying on with no position (region None)."""
    steps = _script(40.0, [], tile=lambda t: False)
    gps_t = [round(10.0 + k * 0.1, 3) for k in range(40)] + [round(16.0 + k * 0.1, 3) for k in range(240)]
    ext_fix = ("gpsLocationExternal", {"latitude": OR_CORVALLIS[0], "longitude": OR_CORVALLIS[1], "hasFix": True,
                                       "verticalAccuracy": 5.0})
    steps = [(t, msgs + ([ext_fix] if t in set(gps_t) else [])) for t, msgs in steps]
    res = R.run(monkeypatch, steps, persistent={"UbloxAvailable": True})
    reqs = _requests(res)
    assert len(reqs) == 1 and 16.0 + GRACE <= reqs[0][0] <= 16.0 + GRACE + 0.05, reqs
    assert _lines(res, "coverage grace start None") == []

  def test_no_fix_messages_restart_the_window(self, monkeypatch):
    gps = _fixes(10.0, 14.0, OR_CORVALLIS) + _fixes(14.0, 18.0, OR_CORVALLIS, fix=False) + _fixes(18.0, 45.0, OR_CORVALLIS)
    res = R.run(monkeypatch, _script(45.0, gps, tile=lambda t: False))
    reqs = _requests(res)
    assert len(reqs) == 1 and 18.0 + GRACE <= reqs[0][0] <= 18.0 + GRACE + 0.05, reqs


class TestRefreshLocationMap:
  def test_refresh_is_not_delayed_by_the_window(self, monkeypatch):
    monkeypatch.setattr(mapd_configd, "_delete_region_tiles", lambda region, is_us_state: 0)
    res = R.run(monkeypatch, _script(40.0, _fixes(10.0, 40.0, OR_CORVALLIS), tile=lambda t: False),
                params_script={10.5: {"RefreshLocationMap": True}})
    reqs = _requests(res)
    assert reqs[0] == (10.5, "us_state.OR"), reqs
    assert len(reqs) == 1, "the window's expiry re-sent on top of the refresh"


class TestMapdSilent:
  def test_no_request_and_a_loud_log_while_mapd_is_silent(self, monkeypatch):
    """mapdExtendedOut alive but no mapdOut: the pre-grace code requested at the first fix. Now nothing
    is requested until mapdOut reports, and then only after a full window."""
    levels = []

    class LevelLog(R.FakeLog):
      def error(self, *a, **kw):
        levels.append(("error", a[0]))
        self._line(*a, **kw)

    monkeypatch.setattr(R, "FakeLog", LevelLog)
    mapd_back = 30.0
    res = R.run(monkeypatch, _script(55.0, _fixes(10.0, 55.0, OR_CORVALLIS), tile=lambda t: False, mapd_out=(mapd_back, None)))
    reqs = _requests(res)
    assert len(reqs) == 1 and mapd_back + GRACE <= reqs[0][0] <= mapd_back + GRACE + 0.05, reqs
    errors = [m for lvl, m in levels if "coverage unknown" in m]
    assert len(errors) == 1, levels
    assert len(_lines(res, "mapdOut silence with a fix ended")) == 1

  def test_mapd_dying_mid_window_restarts_it(self, monkeypatch):
    res = R.run(monkeypatch, _script(50.0, _fixes(10.0, 50.0, OR_CORVALLIS), tile=lambda t: False,
                                     mapd_out=(0.0, None)))
    base = _requests(res)
    assert len(base) == 1 and base[0][0] == pytest.approx(20.0, abs=0.051)
    # the same, but mapdOut drops out 15-17 s (a mapd restart): the window restarts when it is back
    steps = [(t, [m for m in msgs if not (m[0] == "mapdOut" and 15.0 <= t < 17.0)])
             for t, msgs in _script(50.0, _fixes(10.0, 50.0, OR_CORVALLIS), tile=lambda t: False)]
    res = R.run(monkeypatch, steps)
    reqs = _requests(res)
    assert len(reqs) == 1 and 17.0 + GRACE <= reqs[0][0] <= 17.0 + GRACE + 0.05, reqs
    assert len(_lines(res, "coverage grace reset")) == 1 and "mapdOut silent" in _lines(res, "coverage grace reset")[0]

  def test_a_restarted_mapd_gets_a_fresh_window_after_an_earlier_request(self, monkeypatch):
    """Uncovered, requested at 20 s, the tile loads at 70 s (the pull finished). mapd restarts 85-87 s (its
    state is empty again) and loads the tile 0.3 s after it is back. The retry is due again by then
    (> 60 s idle), so only a fresh window stops a second whole-state request."""
    steps = [(t, [m for m in msgs if not (m[0] == "mapdOut" and 85.0 <= t < 87.0)])
             for t, msgs in _script(95.0, _fixes(10.0, 95.0, OR_CORVALLIS), tile=lambda t: 70.0 <= t < 85.0 or t >= 87.3)]
    res = R.run(monkeypatch, steps)
    reqs = _requests(res)
    assert len(reqs) == 1 and reqs[0][0] == pytest.approx(20.0, abs=0.051), reqs
