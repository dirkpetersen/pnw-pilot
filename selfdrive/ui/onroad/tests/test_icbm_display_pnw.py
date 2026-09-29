"""uimax2pnw -- the screen MAX SPEED keeps the driver's max during an ICBM slowdown (display only)."""
import math

import pytest

from openpilot.selfdrive.ui.onroad import icbm_display as D
from openpilot.selfdrive.ui.onroad.icbm_display import IcbmMaxDisplay, max_speed_display

MPH = 0.44704
KPH_PER_MPH = 1.609344
NOW = 1_000_000.0


def status(phase="cap", c=60 * MPH, ts=NOW - 0.2, rcap=0.0):
  return {"icbmPhase": phase, "icbmC": c, "ts": ts, "icbmRCap": rcap}


def show(cluster_mph, st, capable=True, now=NOW):
  v, why = max_speed_display(cluster_mph * KPH_PER_MPH, st, now, capable)
  return v / KPH_PER_MPH, why


class TestChooser:
  @pytest.mark.parametrize("phase", ["cap", "restore"])
  def test_active_episode_shows_the_drivers_ceiling(self, phase):
    v, why = show(40, status(phase))            # truck set slowed to 40, driver's max is 60
    assert math.isclose(v, 60, abs_tol=1e-6) and why is None

  def test_idle_shows_the_truck_set_and_is_not_a_fallback(self):
    assert show(40, status("idle")) == (pytest.approx(40), None)

  def test_gas_phase_shows_the_truck_set(self):
    assert show(40, status("gas")) == (pytest.approx(40), None)

  def test_driver_set_change_mid_episode_ends_it_and_the_screen_follows(self):
    # ces_pnw resets the episode to idle on a driver SET+/-; the status the UI then sees has phase idle
    assert show(58, status("idle", c=None))[0] == pytest.approx(58)

  def test_no_capability_is_the_truck_set_untouched(self):
    assert show(40, status("cap"), capable=False) == (pytest.approx(40), None)

  @pytest.mark.parametrize("ts", [NOW - 10, NOW + 10, None, "x", float("nan")])
  def test_stale_or_bad_ts_falls_back_with_a_reason(self, ts):
    v, why = show(40, status(ts=ts))
    assert v == pytest.approx(40) and why

  @pytest.mark.parametrize("c", [None, "x", float("nan"), float("inf"), 0.0, -5.0, True])
  def test_bad_ceiling_falls_back_with_a_reason(self, c):
    v, why = show(40, status(c=c))
    assert v == pytest.approx(40) and why

  def test_ceiling_below_the_set_falls_back_with_a_reason(self):
    v, why = show(65, status(c=60 * MPH))       # driver raised the set past a stale latch
    assert v == pytest.approx(65) and why

  def test_ceiling_within_rounding_of_the_set_is_shown(self):
    v, why = show(60.5, status(c=60 * MPH))
    assert why is None and v == pytest.approx(60)

  def test_a_dropped_limit_bounds_the_shown_max_like_the_restore(self):
    v, why = show(35, status("restore", c=60 * MPH, rcap=45 * MPH))
    assert why is None and v == pytest.approx(45, abs=0.2)

  def test_rcap_zero_or_above_ceiling_is_ignored(self):
    assert show(40, status(rcap=0.0))[0] == pytest.approx(60)
    assert show(40, status(rcap=80 * MPH))[0] == pytest.approx(60)

  def test_unreadable_status_falls_back_with_a_reason(self):
    v, why = show(40, None)
    assert v == pytest.approx(40) and why


class FakeMem:
  def __init__(self, st):
    self.st, self.calls = st, 0

  def get(self, key, return_default=False):
    self.calls += 1
    if isinstance(self.st, Exception):
      raise self.st
    return self.st


class TestPoller:
  def _mk(self, st, t=None):
    t = t if t is not None else [0.0]
    return IcbmMaxDisplay(mem=FakeMem(st), clock=lambda: t[0], wall=lambda: NOW), t

  def test_shows_ceiling_and_polls_at_most_5hz(self):
    m, t = self._mk(status())
    assert m.choose(40 * KPH_PER_MPH, True) == pytest.approx(60 * KPH_PER_MPH)
    m.choose(40 * KPH_PER_MPH, True)
    assert m._mem.calls == 1
    t[0] += 0.3
    m.choose(40 * KPH_PER_MPH, True)
    assert m._mem.calls == 2

  def test_incapable_never_touches_the_store(self):
    m, _ = self._mk(status())
    assert m.choose(64.0, False) == 64.0 and m._mem.calls == 0

  def test_fallback_is_logged_once_per_change_and_normal_idle_never(self, monkeypatch):
    logged = []
    monkeypatch.setattr(D.cloudlog, "warning", lambda m, *a, **k: logged.append(m))
    m, t = self._mk(status("idle"))
    for _ in range(5):
      m.choose(64.0, True)
      t[0] += 0.3
    assert logged == []
    m._mem.st = status(ts=NOW - 99)              # publisher dead mid-episode
    for _ in range(5):
      m.choose(64.0, True)
      t[0] += 0.3
    assert len(logged) == 1 and "stale" in logged[0]
    m._mem.st = status()                         # recovered -> healthy, then a NEW fault logs again
    m.choose(64.0, True)
    t[0] += 0.3
    m._mem.st = status(c=None)
    m.choose(64.0, True)
    assert len(logged) == 2

  def test_a_read_error_is_logged_loudly_once_and_falls_back(self, monkeypatch):
    errs = []
    monkeypatch.setattr(D.cloudlog, "error", lambda m, *a, **k: errs.append(m))
    m, t = self._mk(OSError("boom"))
    for _ in range(4):
      assert m.choose(64.0, True) == 64.0
      t[0] += 0.3
    assert len(errs) == 1 and "boom" in errs[0]

  def test_unpublished_status_is_the_normal_no_episode_case(self, monkeypatch):
    logged = []
    monkeypatch.setattr(D.cloudlog, "warning", lambda m, *a, **k: logged.append(m))
    m, _ = self._mk(None)
    assert m.choose(64.0, True) == 64.0 and logged == []


class TestAgainstTheRealEpisode:
  """Feed the real IcbmEpisode's phase/ceiling/zone_cap through the chooser (the 5 Hz CESStatus shape)."""

  def _st(self, ep, ts=NOW):
    return {"icbmPhase": ep.phase, "icbmC": ep.ceiling, "ts": ts,
            "icbmRCap": float(ep.zone_cap) if ep.zone_cap is not None else 0.0}

  def test_cap_then_restore_then_driver_set_change(self):
    from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import IcbmEpisode, ICBM_RESTORE_DELAY_S
    ep = IcbmEpisode()
    t = 100.0
    ep.step(t, 40 * MPH, 60 * MPH, 60 * MPH, True, False)             # cap latches ceiling 60
    assert show(60, self._st(ep))[0] == pytest.approx(60)
    ep.step(t + 1, 40 * MPH, 40 * MPH, 40 * MPH, True, False)         # truck taps down
    assert show(40, self._st(ep))[0] == pytest.approx(60)             # screen still the driver's 60
    ep.step(t + 2, None, 40 * MPH, 40 * MPH, True, False)
    ep.step(t + 2 + ICBM_RESTORE_DELAY_S + 0.1, None, 40 * MPH, 40 * MPH, True, False)
    assert ep.phase == "restore" and show(41, self._st(ep))[0] == pytest.approx(60)
    ep.step(t + 30, None, 38 * MPH, 38 * MPH, True, False)            # driver SET- during restore -> abort
    assert ep.phase == "idle" and show(38, self._st(ep))[0] == pytest.approx(38)


# ---------------------------------------------------------------- HUD wiring (both HUDs)
try:
  from openpilot.selfdrive.ui.onroad import hud_renderer as _hud
  from openpilot.selfdrive.ui.mici.onroad import hud_renderer as _mici
except Exception as _e:                                    # pragma: no cover - environment guard
  _hud = _mici = None
  _HUD_SKIP = f"raylib/params UI deps unavailable in this tree: {_e}"
else:
  _HUD_SKIP = ""

from types import SimpleNamespace as NS


def _sm(cluster_kph, enabled=True):
  return {"controlsState": NS(vCruiseDEPRECATED=0.0),
          "carState": NS(vCruiseCluster=cluster_kph, vEgoCluster=10.0, vEgo=10.0, steeringAngleDeg=0.0),
          "selfdriveState": NS(enabled=enabled),
          "recv_frame": {"carState": 5}}


class _Sm(dict):
  @property
  def recv_frame(self):
    return self["recv_frame"]


def _uis(cluster_kph, capable, enabled=True):
  return NS(sm=_Sm(_sm(cluster_kph, enabled)), started_frame=0, is_metric=True, icbm_display_ceiling=capable)


@pytest.mark.skipif(_hud is None, reason=_HUD_SKIP or "n/a")
@pytest.mark.parametrize("capable,expect", [(True, 100.0), (False, 60.0)])
def test_hud_number_uses_the_chooser(monkeypatch, capable, expect):
  monkeypatch.setattr(_hud, "ui_state", _uis(60.0, capable))
  r = object.__new__(_hud.HudRenderer)
  r.v_ego_cluster_seen = False
  r._icbm_max = NS(choose=lambda cluster, cap: 100.0 if cap else cluster)
  r._update_state()
  assert r.set_speed == expect and r.is_cruise_set


@pytest.mark.skipif(_mici is None, reason=_HUD_SKIP or "n/a")
def test_mici_hud_number_uses_the_chooser_and_no_flash_when_only_the_truck_set_changes(monkeypatch):
  r = object.__new__(_mici.HudRenderer)
  r.v_ego_cluster_seen, r.set_speed, r._engaged, r._set_speed_changed_time = False, 100.0, True, 0.0
  # the truck set drops 100 -> 60 for a curve; the chooser keeps showing the driver's 100
  r._icbm_max = NS(choose=lambda cluster, cap: 100.0)
  monkeypatch.setattr(_mici, "ui_state", _uis(60.0, True))
  monkeypatch.setattr(_mici.rl, "get_time", lambda: 123.0)
  r._update_state()
  assert r.set_speed == 100.0 and r._set_speed_changed_time == 0.0          # displayed value unchanged -> no flash
  # without the capability the truck's own value shows and the flash fires (unchanged upstream behaviour)
  r._icbm_max = NS(choose=lambda cluster, cap: cluster)
  r._update_state()
  assert r.set_speed == 60.0 and r._set_speed_changed_time == 123.0
