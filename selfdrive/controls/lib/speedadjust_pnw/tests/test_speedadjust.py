"""speedadjust2pnw unit tests — the reduce-only cap math for limit-drop + police, mode-gated.
Tests drive the internal state directly (skipping the ~1 Hz param read) so they are deterministic.
The emitted cap SLEWS toward its target (never steps), so value assertions use _settle(), which
replays cap() with simulated 0.5 s ticks until the output converges.

speedanchor2pnw: cap() takes v_cruise_set (the driver's raw, PRE-VTSC set) separately from v_cruise
(the current EFFECTIVE ceiling) plus v_cruise_initialized. The _cap()/_settle() helpers default
v_cruise_set=v_cruise and v_cruise_initialized=True so every pre-existing test (which never modeled
VTSC or an unset cruise) keeps calling them unchanged; the new tests below pass v_cruise_set /
v_cruise_initialized explicitly to exercise the three speedanchor2pnw fixes."""
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


def _cap(c, v_cruise, v_ego, v_cruise_set=None, v_cruise_initialized=True):
  if v_cruise_set is None:
    v_cruise_set = v_cruise                # default: no VTSC in effect -> raw set == effective ceiling
  return c.cap(None, v_cruise_set, v_cruise, v_ego, v_cruise_initialized)


def _settle(c, v_cruise, v_ego, v_cruise_set=None, v_cruise_initialized=True, max_iter=400):
  """Run cap() with simulated 0.5 s ticks until the slewed output converges."""
  if v_cruise_set is None:
    v_cruise_set = v_cruise
  out = c.cap(None, v_cruise_set, v_cruise, v_ego, v_cruise_initialized)
  for _ in range(max_iter):
    c._last_t -= 0.5                       # pretend 0.5 s passed since the last call
    new = c.cap(None, v_cruise_set, v_cruise, v_ego, v_cruise_initialized)
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


# ---- speedanchor2pnw fixes (2026-07-18) -------------------------------------

# F2: the limit-drop ratio anchor and the cap-slew seed must use v_cruise_set (the driver's raw,
# PRE-VTSC set), not v_cruise (the current EFFECTIVE ceiling, already reduced by VTSC for a curve).

def test_idle_reanchor_uses_raw_set_not_vtsc_capped():
  # idle (mode 0) re-anchor: a curve active RIGHT NOW (v_cruise=45, VTSC-capped) must not poison the
  # ratio — it has to anchor off the driver's real 75 mph set.
  c = _ctrl(mode=0, sl=V60)
  _cap(c, V45, V60, v_cruise_set=V75)
  assert abs(c._ratio - V75 / V60) < 1e-6              # anchored off the raw set...
  assert c._ratio != V45 / V60                         # ...NOT the curve-capped effective ceiling


def test_active_reanchor_uses_raw_set_not_vtsc_capped():
  # active (mode >= 1) re-anchor via _update_baseline(): same requirement, while uncapped (sl >= sl_ref).
  c = _ctrl(mode=2, sl=V60, sl_ref=V60)                 # at baseline -> _update_baseline() re-anchors
  _cap(c, V45, V60, v_cruise_set=V75)                   # a curve is capping the effective ceiling to 45
  assert abs(c._ratio - V75 / V60) < 1e-6


def test_cap_slew_seeds_from_raw_set_not_vtsc_capped():
  # engaging a drop-cap while VTSC is mid-curve: the slew must SEED from the driver's raw set (75), not
  # the curve-reduced effective ceiling (45) — else it crawls up from curve speed after VTSC releases
  # instead of already sitting near the real target. Output on this tick is still bounded by the curve
  # (no jump), but the internal _cap_out reflects the correct high seed.
  c = _drop(mode=2)                                     # ratio anchored 75/60, sl=45 -> target 56.25
  first = _cap(c, V45, V60, v_cruise_set=V75)            # VTSC is capping the effective ceiling to 45
  assert abs(first - V45) < 1e-9                        # output bounded by the curve -> no jump
  assert abs(c._cap_out - V75) < 1e-9                    # but the slew SEEDED at the raw 75, not 45


def test_post_curve_cap_does_not_crawl_from_curve_speed():
  # end-to-end regression for the "won't come back up to my set" bug: once VTSC releases (effective
  # ceiling jumps back to the raw set), the speedadjust cap must already be at/near its real target —
  # not stuck ramping up from the curve speed at CAP_SLEW.
  c = _drop(mode=2)                                      # target 56.25 mph
  _cap(c, V45, V60, v_cruise_set=V75)                     # tick 1: engage mid-curve (VTSC capping to 45)
  c._last_t -= 0.5
  out = _cap(c, V75, V60, v_cruise_set=V75)               # tick 2: curve ends, VTSC releases instantly
  # with the old (buggy) seed-from-effective-v_cruise behaviour this would be ~45 + 0.5 (barely off the
  # curve speed); with the fix it's already ramping down from the correct 75 seed toward 56.25.
  assert out > V60                                        # nowhere near stuck at curve speed


# F3: the limit-drop baseline (_sl_ref/_ratio) must stay current in mode 1 (police-only), not just
# mode 2, so an AutoSpeedReduce 1->2 switch mid-drive never resumes off a stale baseline.

def test_mode1_keeps_baseline_current():
  c = _ctrl(mode=1, sl=V60, sl_ref=V45, ratio=V75 / V45)  # stale: anchored hours ago at a 45 mph zone
  _cap(c, V75, V60)                                       # a mode-1 (police-only) tick at the 60 limit
  assert abs(c._sl_ref - V60) < 1e-6                      # baseline tracked up to the CURRENT limit
  assert abs(c._ratio - V75 / V60) < 1e-6                 # ratio re-anchored off the current set/limit


def test_mode1_to_mode2_switch_no_surprise_cap():
  # end-to-end: baseline stays fresh through mode 1, so switching to mode 2 with a lower limit on the
  # SAME drive produces the correct proportional trim off the CURRENT baseline, not a surprise cap
  # computed from a baseline captured possibly hours earlier.
  c = _ctrl(mode=1, sl=V60, sl_ref=V45, ratio=V75 / V45)  # stale baseline from a much earlier drive
  _cap(c, V75, V60)                                       # drive continues in mode 1 at the 60 limit
  c._mode = 2
  c._sl = V45                                             # NOW the limit drops to 45
  out = _settle(c, V75, V60)
  assert abs(out - V75 * (V45 / V60)) < 1e-6              # correct trim off the fresh 60 baseline


# F_uninit (Fable-caught): anchoring/seeding must be skipped while cruise was never set — else the
# V_CRUISE_UNSET sentinel (~145 km/h) inflates _ratio and seeds _cap_out far above the real set.

def test_uninitialized_cruise_skips_anchor_and_seed():
  V90 = 90 * MPH                                          # stand-in for the ~145 km/h UNSET sentinel
  c = _ctrl(mode=2, sl=V60)                                # _ratio=0.0, _cap_out=None by default
  out = _cap(c, V90, V60, v_cruise_set=V90, v_cruise_initialized=False)
  assert out == V90                                        # neutral passthrough
  assert c._ratio == 0.0                                   # untouched — no bogus anchor recorded
  assert c._cap_out is None                                # untouched — nothing seeded


def test_uninitialized_idle_skips_ratio_anchor():
  V90 = 90 * MPH
  c = _ctrl(mode=0, sl=V60)
  _cap(c, V90, V60, v_cruise_set=V90, v_cruise_initialized=False)
  assert c._ratio == 0.0                                   # idle branch's ratio-anchor was skipped too


def test_becomes_initialized_anchors_correctly():
  # once the driver actually sets cruise, normal anchoring resumes cleanly on the very next tick
  c = _ctrl(mode=0, sl=V60)
  V90 = 90 * MPH
  _cap(c, V90, V60, v_cruise_set=V90, v_cruise_initialized=False)   # not set yet — skipped
  assert c._ratio == 0.0
  _cap(c, V75, V60, v_cruise_set=V75, v_cruise_initialized=True)    # driver sets cruise to 75
  assert abs(c._ratio - V75 / V60) < 1e-6
