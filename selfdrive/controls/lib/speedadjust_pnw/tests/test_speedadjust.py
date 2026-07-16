"""speedadjust2pnw unit tests — the reduce-only cap math for limit-drop + police, mode-gated.
Tests drive the internal state directly (skipping the ~1 Hz param read) so they are deterministic.
The emitted cap SLEWS toward its target (never steps), so value assertions use _settle(), which
replays cap() with simulated 0.5 s ticks until the output converges."""
import time

from openpilot.selfdrive.controls.lib.speedadjust_pnw.speedadjust_controller import (
  SpeedAdjustController, MPH_TO_MS, POLICE_MARGIN, MIN_CAP, CAP_SLEW, RELEASE_S)

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
  c._sl_valid_t = time.monotonic() + 1e6  # keep the injected _sl from being aged out by the hold logic
  c._sl_ref = sl_ref
  c._ratio = ratio
  c._police = police
  c._last_read = time.monotonic() + 1e6   # never re-read params inside cap()
  return c


def _cap(c, v_cruise, v_ego):
  return c.cap(None, v_cruise, v_ego)


def _settle(c, v_cruise, v_ego, max_iter=400):
  """Run cap() with simulated 0.5 s ticks until the slewed output converges."""
  out = c.cap(None, v_cruise, v_ego)
  for _ in range(max_iter):
    c._last_t -= 0.5                       # pretend 0.5 s passed since the last call
    new = c.cap(None, v_cruise, v_ego)
    if abs(new - out) < 1e-9:
      return new
    out = new
  return out


# ratio anchored as if we cruised V75 at a 60 mph baseline, now in a 45 mph zone
def _drop(**kw):
  return _ctrl(sl_ref=V60, ratio=V75 / V60, sl=V45, **kw)


def test_off_is_neutral():
  assert _cap(_drop(mode=0), V75, V60) == V75


def test_no_oplong_is_neutral():
  assert _cap(_drop(op_long=False, mode=2), V75, V60) == V75


def test_limit_drop_proportional():
  out = _settle(_drop(mode=2), V75, V60)       # 60->45 = 25% drop -> 75 * 45/60 = 56.25
  assert abs(out - V75 * (V45 / V60)) < 1e-6
  assert out < V75                             # reduce-only


def test_limit_drop_no_double_reduction():
  # driver re-scrolls their set DOWN to 50 while capped. Old bug: cap = 50*45/60 = 37.5 (forces below
  # their set). Fixed: cap stays the ANCHORED 56.25 (> 50), so reduce-only respects the driver's 50.
  out = _settle(_drop(mode=2), 50 * MPH, V45)
  assert abs(out - 50 * MPH) < 1e-6


def test_limit_drop_under_limit_no_slow():
  # Corvallis city bug: driver set 30 in a 45 zone (ratio 0.67, UNDER the limit); limit drops to 25.
  # Must NOT slow them (was wrongly capping to ~16 mph). ratio < 1 -> no cap.
  V30 = 30 * MPH
  V25 = 25 * MPH
  c = _ctrl(mode=2, sl_ref=V45, ratio=V30 / V45, sl=V25)
  assert _cap(c, V30, V25) == V30


def test_limit_drop_never_below_limit():
  # even at ratio just over 1, the cap floors at the posted limit — never slows below it
  V25 = 25 * MPH
  c = _ctrl(mode=2, sl_ref=V60, ratio=1.02, sl=V25)   # was barely over a 60 baseline
  out = _settle(c, V75, V25)
  assert out >= V25 - 1e-9                              # never below the 25 mph posted limit


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
  assert abs(_settle(c, V75, V60) - (V60 + POLICE_MARGIN)) < 1e-6


def test_police_far_no_action():
  c = _ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 15.0})  # ttr >> 30 s
  assert _cap(c, V75, V60) == V75


def test_police_latch_holds_when_slowed():
  # THE oscillation fix: once latched, slowing to a crawl (ttr balloons) must NOT release the cap
  c = _ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  assert abs(_settle(c, V75, V60) - (V60 + POLICE_MARGIN)) < 1e-6       # latches
  c._police = {"state": "alert", "dist_mi": 0.1}                        # crawling, ttr now huge
  assert abs(_settle(c, V75, 1.0) - (V60 + POLICE_MARGIN)) < 1e-6       # still held


def test_police_nan_distance_no_action():
  c = _ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": float("nan")})
  assert _cap(c, V75, V60) == V75
  assert c._police_latched is False


def test_police_no_limit_no_cap():
  # driver directive 2026-07-15 (the city 15-mph hold): no posted limit -> NO basis for a target ->
  # no cap at all. The latch still arms so a limit appearing mid-approach starts capping then.
  c = _ctrl(mode=1, sl=0.0, police={"state": "alert", "dist_mi": 0.3})
  c._sl_valid_t = -1e9                                  # truly no limit (hold expired)
  assert _settle(c, V75, V60) == V75
  assert c._police_latched is True                      # armed, awaiting a usable limit
  c._sl = V45                                           # limit becomes known mid-approach
  c._sl_valid_t = time.monotonic() + 1e6
  assert abs(_settle(c, V75, V60) - (V45 + POLICE_MARGIN)) < 1e-6


def test_police_clear_releases():
  c = _ctrl(mode=1, sl=V60, police={"state": "clear"})
  assert _cap(c, V75, V60) == V75
  assert c._police_latched is False


def test_reduce_only_never_raises():
  c = _ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.3})
  v_set = 40 * MPH                                   # target 65 > set 40 -> unchanged (reduce-only)
  assert _settle(c, v_set, V45) == v_set


def test_min_cap_floor():
  c = _ctrl(mode=2, sl_ref=70 * MPH, ratio=V75 / (70 * MPH), sl=5 * MPH)   # huge drop
  assert _settle(c, V75, V45) >= MIN_CAP - 1e-9


def test_min_of_both_sources():
  c = _drop(mode=2)
  c._police = {"state": "alert", "dist_mi": 0.3}    # police target 45+5 = 50 vs limit-drop 56.25
  assert abs(_settle(c, V75, V60) - (V45 + POLICE_MARGIN)) < 1e-6


# ---- smoothness guards (the 2026-07-15 "wild horse" fixes) ------------------

def test_cap_slews_not_steps():
  # the emitted cap must RAMP from the driver's set toward the target, bounded by CAP_SLEW per second
  c = _drop(mode=2)                                   # target 56.25 mph, set 75 mph
  first = _cap(c, V75, V60)
  assert abs(first - V75) < 1e-9                      # first cycle: starts AT the set, no jump
  c._last_t -= 0.5
  second = _cap(c, V75, V60)
  assert second < first                               # descending...
  assert first - second <= CAP_SLEW * 0.5 + 1e-9      # ...but no faster than the slew bound


def test_release_debounce_holds_then_releases():
  # a 1-sample source dropout must NOT release the cap (flapping was half the wild-horse ride)
  c = _ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  settled = _settle(c, V75, V60)
  assert abs(settled - (V60 + POLICE_MARGIN)) < 1e-6
  c._police = {"state": "clear"}                      # source clears
  c._last_t -= 0.5
  held = _cap(c, V75, V60)
  assert abs(held - settled) < 1e-6                   # still held during the debounce window
  c._release_t -= (RELEASE_S + 0.1)                   # debounce window elapses
  c._last_t -= 0.5
  assert _cap(c, V75, V60) == V75                     # NOW it releases


def test_sl_dropout_hold():
  # _read_speed_limit: a limit valid within SL_HOLD_S is held through read dropouts (mem read = 0)
  c = _ctrl(mode=2)
  c.mem_params = None                                 # every read yields "unknown"
  c._sl = V45
  c._sl_valid_t = time.monotonic()                    # was valid just now
  assert abs(c._read_speed_limit() - V45) < 1e-6      # held
  c._sl_valid_t = time.monotonic() - 10.0             # hold expired
  assert c._read_speed_limit() == 0.0
