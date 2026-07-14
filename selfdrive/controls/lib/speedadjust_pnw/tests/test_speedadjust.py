"""speedadjust2pnw unit tests — the reduce-only cap math for limit-drop + police, mode-gated.
Tests drive the internal state directly (skipping the ~1 Hz param read) so they are deterministic."""
import time

from openpilot.selfdrive.controls.lib.speedadjust_pnw.speedadjust_controller import (
  SpeedAdjustController, MPH_TO_MS, POLICE_MARGIN, POLICE_NO_LIMIT_TRIM, MIN_CAP)

MPH = MPH_TO_MS
V75 = 75 * MPH
V60 = 60 * MPH
V45 = 45 * MPH


class _CP:
  def __init__(self, op_long=True):
    self.openpilotLongitudinalControl = op_long


class _Params:
  def get(self, *a, **k):
    return None

  def get_bool(self, *a, **k):
    return False


def _ctrl(op_long=True, mode=2, sl=0.0, sl_ref=0.0, ratio=0.0, police=None):
  c = SpeedAdjustController(_CP(op_long), params=_Params())
  c._mode = mode
  c._sl = sl
  c._sl_ref = sl_ref
  c._ratio = ratio
  c._police = police
  c._last_read = time.monotonic() + 1e6   # never re-read params inside cap()
  return c


def _cap(c, v_cruise, v_ego):
  return c.cap(None, v_cruise, v_ego)


# ratio anchored as if we cruised V75 at a 60 mph baseline, now in a 45 mph zone
def _drop(**kw):
  return _ctrl(sl_ref=V60, ratio=V75 / V60, sl=V45, **kw)


def test_off_is_neutral():
  assert _cap(_drop(mode=0), V75, V60) == V75


def test_no_oplong_is_neutral():
  assert _cap(_drop(op_long=False, mode=2), V75, V60) == V75


def test_limit_drop_proportional():
  out = _cap(_drop(mode=2), V75, V60)          # 60->45 = 25% drop -> 75 * 45/60 = 56.25
  assert abs(out - V75 * (V45 / V60)) < 1e-6
  assert out < V75                             # reduce-only


def test_limit_drop_no_double_reduction():
  # driver re-scrolls their set DOWN to 50 while capped. Old bug: cap = 50*45/60 = 37.5 (forces below
  # their set). Fixed: cap stays the ANCHORED 56.25 (> 50), so reduce-only respects the driver's 50.
  out = _cap(_drop(mode=2), 50 * MPH, V45)
  assert abs(out - 50 * MPH) < 1e-6


def test_limit_rise_releases():
  c = _ctrl(mode=2, sl_ref=V45, ratio=V75 / V45, sl=V60)   # limit rose above baseline
  assert _cap(c, V75, V60) == V75
  assert abs(c._sl_ref - V60) < 1e-6                        # baseline re-tracked up


def test_limit_drop_ignored_in_police_only_mode():
  assert _cap(_drop(mode=1), V75, V60) == V75


def test_small_drop_ignored():
  c = _ctrl(mode=2, sl_ref=V60, ratio=V75 / V60, sl=59 * MPH)   # <5% dip = noise
  assert _cap(c, V75, V60) == V75


def test_garbage_high_limit_rejected():
  # a garbage-high reading is sanitized to _sl=0 by _read_speed_limit -> no cap
  c = _ctrl(mode=2, sl_ref=V60, ratio=V75 / V60, sl=0.0)
  assert _cap(c, V75, V60) == V75


def test_police_engages_within_window():
  c = _ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})   # ttr ~24 s < 30
  assert abs(_cap(c, V75, V60) - (V60 + POLICE_MARGIN)) < 1e-6


def test_police_far_no_action():
  c = _ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 15.0})  # ttr >> 30 s
  assert _cap(c, V75, V60) == V75


def test_police_latch_holds_when_slowed():
  # THE oscillation fix: once latched, slowing to a crawl (ttr balloons) must NOT release the cap
  c = _ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  assert abs(_cap(c, V75, V60) - (V60 + POLICE_MARGIN)) < 1e-6          # latches
  c._police = {"state": "alert", "dist_mi": 0.1}                        # crawling, ttr now huge
  assert abs(_cap(c, V75, 1.0) - (V60 + POLICE_MARGIN)) < 1e-6          # still held


def test_police_nan_distance_no_action():
  c = _ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": float("nan")})
  assert _cap(c, V75, V60) == V75
  assert c._police_latched is False


def test_police_no_limit_fallback_trim():
  c = _ctrl(mode=1, sl=0.0, police={"state": "alert", "dist_mi": 0.3})   # no posted limit
  out = _cap(c, V75, V60)
  assert abs(out - (V75 - POLICE_NO_LIMIT_TRIM)) < 1e-6


def test_police_clear_releases():
  c = _ctrl(mode=1, sl=V60, police={"state": "clear"})
  assert _cap(c, V75, V60) == V75
  assert c._police_latched is False


def test_reduce_only_never_raises():
  c = _ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.3})
  v_set = 40 * MPH                                   # target 65 > set 40 -> unchanged (reduce-only)
  assert _cap(c, v_set, V45) == v_set


def test_min_cap_floor():
  c = _ctrl(mode=2, sl_ref=70 * MPH, ratio=V75 / (70 * MPH), sl=5 * MPH)   # huge drop
  assert _cap(c, V75, V45) >= MIN_CAP - 1e-9


def test_min_of_both_sources():
  c = _drop(mode=2)
  c._police = {"state": "alert", "dist_mi": 0.3}    # police target 45+5 = 50 vs limit-drop 56.25
  assert abs(_cap(c, V75, V60) - (V45 + POLICE_MARGIN)) < 1e-6
