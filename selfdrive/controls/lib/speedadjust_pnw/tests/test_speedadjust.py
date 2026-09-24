"""speedadjust2pnw unit tests — the reduce-only cap math for limit-drop + police, mode-gated.
Tests drive the internal state directly (skipping the ~1 Hz param read) so they are deterministic.
The emitted cap SLEWS toward its target (never steps), so value assertions use _settle(), which
replays cap() with simulated 0.5 s ticks until the output converges.

speedanchor2pnw: cap() takes v_cruise_set (the driver's raw, PRE-VTSC set) separately from v_cruise
(the current EFFECTIVE ceiling) plus v_cruise_initialized. The _cap()/_settle() helpers default
v_cruise_set=v_cruise and v_cruise_initialized=True so every pre-existing test (which never modeled
VTSC or an unset cruise) keeps calling them unchanged; the new tests below pass v_cruise_set /
v_cruise_initialized explicitly to exercise the three speedanchor2pnw fixes."""
import json
import time

import pytest

from openpilot.selfdrive.controls.lib.speedadjust_pnw.speedadjust_controller import (
  SpeedAdjustController, MPH_TO_MS, POLICE_MARGIN, MIN_CAP, CAP_SLEW, RELEASE_S, RESTORE_WINDOW_S,
  SA_DRIVER_LOWER_TOL, SET_CHANGE_EPS, SA_ACTUATION_GRACE_S, SL_DROP_CONFIRM_S, SL_RISE_CONFIRM_S,
  ZONE_SET_TIMEOUT_S, ZONE_SET_DONE_TOL, SA_ICBM_FRESH_S,
  _police_key)

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
    c._pub_last -= 0.5                     # speedadjust-exec2pnw: also age the publish throttle so
                                            # repeated test cap() calls (near-zero real wall time
                                            # apart) don't get silently swallowed by PUB_THROTTLE_S
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


# limitdropexact2pnw (owner rule 1b, 2026-09-24): a driver AT or UNDER the old limit is slowed to EXACTLY the new
# limit -- never below it, and not at all if the set is already at/below the new limit. The 07-14 "Corvallis"
# guard (ratio < 1 -> no cap) left the truck at 41 mph in a 25 zone on 2026-09-24 09:18 PT. The Corvallis bug it
# fixed (30 in a 45 scaled to ~16 mph) stays fixed by the posted-limit floor.
def test_limit_drop_under_limit_slows_to_exactly_the_new_limit():
  V30 = 30 * MPH
  V25 = 25 * MPH
  c = _ctrl(mode=2, sl_ref=V45, ratio=V30 / V45, sl=V25)     # 30 in a 45 (Corvallis case), limit drops to 25
  out = _settle(c, V30, V25)
  assert abs(out - V25) < 1e-6, f"want exactly 25 mph, got {out / MPH:.2f}"


def test_limit_drop_the_2026_09_24_case_41_in_a_45_to_a_25():
  V41 = 41 * MPH
  V25 = 25 * MPH
  c = _ctrl(mode=2, sl_ref=V45, ratio=V41 / V45, sl=V25)
  out = _settle(c, V41, V25)
  assert abs(out - V25) < 1e-6, f"41 in a 45 -> 25 zone must settle at 25, got {out / MPH:.2f}"


def test_limit_drop_at_the_limit_slows_to_exactly_the_new_limit():
  V25 = 25 * MPH
  c = _ctrl(mode=2, sl_ref=V45, ratio=1.0, sl=V25)
  out = _settle(c, V45, V25)
  assert abs(out - V25) < 1e-6


def test_limit_drop_already_below_the_new_limit_is_untouched():
  V20 = 20 * MPH
  V25 = 25 * MPH
  c = _ctrl(mode=2, sl_ref=V45, ratio=V20 / V45, sl=V25)     # 20 in a 45; the new 25 limit is above the set
  assert _cap(c, V20, V25) == V20


def test_limit_drop_over_the_limit_keeps_the_same_percentage():
  # rule 1 unchanged: 15 % over 35 -> 15 % over 25 = 28.75 mph
  V35 = 35 * MPH
  V25 = 25 * MPH
  set_ = 1.15 * V35
  c = _ctrl(mode=2, sl_ref=V35, ratio=1.15, sl=V25)
  out = _settle(c, set_, V25)
  assert abs(out - 1.15 * V25) < 1e-6, f"want 28.75 mph, got {out / MPH:.2f}"


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


def test_police_latch_releases_when_cap_report_changes():
  # policelatch2pnw REGRESSION (observed on-car 2026-08-21, cap report 8.3 mi out while still capped):
  # the latch belongs to ONE report. After passing report A the CONTROL channel switches to the next
  # confirmed report B, which may be miles away. _police_latched survived that switch, so the
  # POLICE_ENGAGE_S approach gate was skipped and the car stayed capped indefinitely -- on a corridor
  # where some confirmed report is nearly always in range, the speed never resumed.
  #
  # NOTE this is NOT covered by test_switching_reports_also_drops_the_stale_latch: that one sets
  # _police_suppressed first, so it only exercises the DISMISSAL branch. This is the plain switch.
  c = _ctrl(sl=V45, police=_police(0.3, "confirmed", cap_uuid="A"))
  assert c._police_cap(V75, V75) == V45 + POLICE_MARGIN
  assert c._police_latched is True and c._police_latched_key == "A"
  c._police = _police(8.3, "confirmed", cap_mi=8.3, cap_uuid="B")   # passed A; B is 8.3 mi out
  assert c._police_cap(V75, V75) is None, "must resume, not inherit A's latch"
  assert c._police_latched is False


def test_police_latch_survives_same_report_across_ticks():
  # the flip side: the identity check must not break latch-holds-when-slowed (the oscillation fix)
  c = _ctrl(sl=V45, police=_police(0.3, "confirmed", cap_uuid="A"))
  assert c._police_cap(V75, V75) == V45 + POLICE_MARGIN
  c._police = _police(0.05, "confirmed", cap_mi=0.05, cap_uuid="A")   # same report, now crawling
  assert c._police_cap(V75, 1.0) == V45 + POLICE_MARGIN               # ttr balloons -> still held
  assert c._police_latched is True


def test_police_new_report_must_earn_its_own_latch():
  # after the switch, B caps only once IT is inside the approach window -- never inherited from A
  c = _ctrl(sl=V45, police=_police(0.3, "confirmed", cap_uuid="A"))
  c._police_cap(V75, V75)
  c._police = _police(8.3, "confirmed", cap_mi=8.3, cap_uuid="B")
  assert c._police_cap(V75, V75) is None
  c._police = _police(0.3, "confirmed", cap_mi=0.3, cap_uuid="B")     # now B is close
  assert c._police_cap(V75, V75) == V45 + POLICE_MARGIN
  assert c._police_latched is True and c._police_latched_key == "B"


def test_police_unresolvable_key_cannot_inherit_the_latch():
  # fail toward resuming: a cap report we cannot attribute must not carry a latch earned by another.
  # (A keyless report that is genuinely close still caps -- it re-earns the latch via the approach
  # gate on the same tick -- which is correct; what must not happen is inheriting one from far away.)
  c = _ctrl(sl=V45, police=_police(0.3, "confirmed", cap_uuid="A"))
  assert c._police_cap(V75, V75) == V45 + POLICE_MARGIN
  c._police = {"state": "alert", "dist_mi": 8.3, "tier": "confirmed",
               "cap": {"dist_mi": 8.3, "age_min": 1}}                  # no uuid, no key
  assert c._police_cap(V75, V75) is None
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


# ---- speedadjust-exec2pnw: SpeedAdjustTarget mem-param publish + bounded restore ------------------
# The RETURN value of cap() is unaffected by any of this (see test_no_oplong_is_neutral above, which
# still passes unmodified) -- these tests exercise the NEW mem-param SIDE EFFECT that feeds the shared
# stock-ACC button executor (opendbc/car/ford/icbm_pnw.py) on a car with no op-long.

class _FakeMemParams:
  """Records SpeedAdjustTarget publishes for assertion; mirrors the mem_params.put_nonblocking(key,
  payload) interface the real /dev/shm Params exposes. Real Params() is unavailable on this host
  (no built params_pyx.so), so production code already falls back to mem_params=None here — tests
  inject this fake directly to observe the publish side effect."""
  def __init__(self):
    self.calls = []          # [(key, payload), ...] in call order

  def put_nonblocking(self, key, payload):
    self.calls.append((key, payload))

  @property
  def target_calls(self):
    # satele2pnw: the ACTUATOR channel only. speedadjust now also publishes a read-only diagnostic
    # channel ("SpeedAdjustStatus") on EVERY car including op-long; tests that assert "this car must
    # never be actuated" mean SpeedAdjustTarget specifically, not "no mem-param write of any kind".
    return [c for c in self.calls if c[0] == "SpeedAdjustTarget"]

  @property
  def last(self):
    tc = self.target_calls
    return tc[-1][1] if tc else None


def _stock_ctrl(**kw):
  """Same as _ctrl() but for a stock-ACC (no op-long) car with an inspectable fake mem_params."""
  c = _ctrl(op_long=False, **kw)
  c.mem_params = _FakeMemParams()
  return c


def _tick(c, v_cruise=V75, v_ego=V60, v_cruise_set=None, sm=None, dt=0.5):
  """Advance both the slew clock and the publish-throttle clock together, then call cap() once."""
  if v_cruise_set is None:
    v_cruise_set = v_cruise
  c._last_t -= dt
  c._pub_last -= dt
  return c.cap(sm, v_cruise_set, v_cruise, v_ego, True)


def _settle_pub(c, v_cruise=V75, v_ego=V60, iters=250):
  """_settle() detects convergence off cap()'s RETURN value, which for a stock-ACC controller (no
  op-long) is ALWAYS the neutral v_cruise -- it never moves, so _settle() would report "converged"
  after a single tick even while the SLEWED _cap_out/published target is still far from its final
  value. Stock-ACC tests that need the published target to actually finish slewing use this instead:
  a fixed iteration count comfortably longer than any realistic engage swing needs at CAP_SLEW."""
  c.cap(None, v_cruise, v_cruise, v_ego, True)   # first call: initializes _last_t (dt=0), like _settle()
  for _ in range(iters):
    _tick(c, v_cruise=v_cruise, v_ego=v_ego)


def test_no_publish_when_idle_from_start():
  c = _stock_ctrl(mode=0)
  _cap(c, V75, V60)
  assert c.mem_params.target_calls == []


def test_no_publish_on_oplong_car():
  # same scenario as test_publish_dec_while_capping below, but op_long=True: the mem-param is never
  # touched -- op-long cars are steered solely through cap()'s return value.
  c = _ctrl(op_long=True, mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  c.mem_params = _FakeMemParams()   # inject AFTER construction so we can observe (a lack of) calls
  _settle(c, V75, V60)
  assert c.mem_params.target_calls == []


def test_publish_dec_while_capping():
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle_pub(c, V75, V60)                              # need the full slew, not just return-value
                                                         # convergence (always neutral on a stock car)
  payload = c.mem_params.last
  assert payload is not None
  assert "dir" not in payload                          # default cap direction is "dec"
  assert abs(payload["target"] - (V60 + POLICE_MARGIN)) < 0.01   # 2-decimal m/s rounding in the publish
  assert abs(payload["ceiling"] - V75) < 0.01           # latched at the driver's raw set on engage
  assert "ts" in payload


def test_publish_target_matches_op_long_cap_value():
  # the stock-ACC executor must tap toward the IDENTICAL value the op-long MPC would have consumed --
  # build the same scenario on both an op-long and a stock-ACC controller and compare.
  long_c = _ctrl(op_long=True, mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  long_out = _settle(long_c, V75, V60)
  stock_c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle_pub(stock_c, V75, V60)
  assert abs(stock_c.mem_params.last["target"] - long_out) < 0.01


def test_restore_begins_after_cap_clears():
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle_pub(c, V75, V60)                              # cap engages + fully slews, latches ceiling=75
  assert abs(c.mem_params.last["ceiling"] - V75) < 0.01
  assert abs(c.mem_params.last["target"] - (V60 + POLICE_MARGIN)) < 0.01   # fully slewed by now
  c._police = {"state": "clear"}                        # source clears
  _tick(c)                                               # still holding through the debounce window
  assert abs(c.mem_params.last["target"] - (V60 + POLICE_MARGIN)) < 0.01  # unchanged during the hold
  c._release_t -= (RELEASE_S + 0.1)                      # debounce elapses
  _tick(c)                                               # cap releases -> restore should start
  payload = c.mem_params.last
  assert payload.get("dir") == "inc"
  assert abs(payload["target"] - V75) < 0.01
  assert abs(payload["ceiling"] - V75) < 0.01
  assert c._restore_ceiling is not None


def test_restore_never_above_driver_ceiling():
  # a curve happening to hand a HIGHER v_cruise while the restore is in progress must not change the
  # restore target -- it stays pinned to the ceiling latched at the ORIGINAL cap engage.
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle_pub(c, V75, V60)
  c._police = {"state": "clear"}
  _tick(c)
  c._release_t -= (RELEASE_S + 0.1)
  _tick(c)
  assert abs(c.mem_params.last["target"] - V75) < 0.01   # never above the original 75 mph ceiling


def test_restore_expires_after_window():
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle(c, V75, V60)
  c._police = {"state": "clear"}
  _tick(c)
  c._release_t -= (RELEASE_S + 0.1)
  _tick(c)                                               # restore begins
  assert c._restore_ceiling is not None
  c._restore_deadline -= (RESTORE_WINDOW_S + 1.0)        # window expires
  _tick(c)
  assert c._restore_ceiling is None
  assert c.mem_params.last == {}


def test_new_cap_preempts_restore():
  # DEC ALWAYS WINS: a fresh cap engaging mid-restore must cancel the restore immediately, not race it
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle(c, V75, V60)
  c._police = {"state": "clear"}
  _tick(c)
  c._release_t -= (RELEASE_S + 0.1)
  _tick(c)                                               # restore begins
  assert c._restore_ceiling is not None
  c._police = {"state": "alert", "dist_mi": 0.4}         # a NEW police report appears
  _tick(c)
  assert c._restore_ceiling is None                      # preempted
  payload = c.mem_params.last
  assert "dir" not in payload                             # back to a dec (cap), not inc


class _FakeCruiseState:
  def __init__(self, enabled=True, speed=0.0):
    self.enabled = enabled
    self.speed = speed          # restore-hardening: the truck's OWN reported stock-ACC set speed


class _FakeCS:
  def __init__(self, gas=False, brake=False, cruise_enabled=True, speed=0.0):
    self.gasPressed = gas
    self.brakePressed = brake
    self.cruiseState = _FakeCruiseState(cruise_enabled, speed)


def test_restore_aborted_by_driver_gas():
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle(c, V75, V60)
  c._police = {"state": "clear"}
  _tick(c)
  c._release_t -= (RELEASE_S + 0.1)
  _tick(c)                                               # restore begins
  assert c._restore_ceiling is not None
  _tick(c, sm={"carState": _FakeCS(gas=True)})
  assert c._restore_ceiling is None
  assert c.mem_params.last == {}


def test_restore_aborted_by_acc_off():
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle(c, V75, V60)
  c._police = {"state": "clear"}
  _tick(c)
  c._release_t -= (RELEASE_S + 0.1)
  _tick(c)                                               # restore begins
  assert c._restore_ceiling is not None
  _tick(c, sm={"carState": _FakeCS(cruise_enabled=False)})
  assert c._restore_ceiling is None
  assert c.mem_params.last == {}


def test_restore_survives_missing_or_none_sm():
  # sm may legitimately be None (unit tests) or missing carState -- must never raise, never abort
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle(c, V75, V60)
  c._police = {"state": "clear"}
  _tick(c)
  c._release_t -= (RELEASE_S + 0.1)
  _tick(c)                                               # restore begins
  assert c._restore_ceiling is not None
  _tick(c, sm=None)
  assert c._restore_ceiling is not None                  # still restoring
  _tick(c, sm={})                                        # sm present but no carState key
  assert c._restore_ceiling is not None                  # still restoring, no crash


def test_mode_off_cancels_pending_restore():
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle(c, V75, V60)
  c._police = {"state": "clear"}
  _tick(c)
  c._release_t -= (RELEASE_S + 0.1)
  _tick(c)                                               # restore begins
  assert c._restore_ceiling is not None
  c._mode = 0
  _tick(c)
  assert c._restore_ceiling is None
  assert c.mem_params.last == {}


class _FakeAxis:
  def __init__(self, x=None, z=None):
    self.x = x
    self.z = z


class _FakeModelV2:
  """orientationRate.z[0] * velocity.x[0] == lat_accel when v_ego != 0 (matches ces_pnw's own
  measured-now lateral accel formula, which _in_curve() mirrors)."""
  def __init__(self, lat_accel=0.0, v_ego=30.0):
    self.orientationRate = _FakeAxis(z=[lat_accel / v_ego if v_ego else 0.0])
    self.velocity = _FakeAxis(x=[v_ego])


def _sm(gas=False, brake=False, cruise_enabled=True, speed=0.0, lat_accel=None, v_ego=30.0):
  d = {"carState": _FakeCS(gas=gas, brake=brake, cruise_enabled=cruise_enabled, speed=speed)}
  if lat_accel is not None:
    d["modelV2"] = _FakeModelV2(lat_accel=lat_accel, v_ego=v_ego)
  return d


# ---- restore2pnw-hardening (2026-08, Gemini + Fable review) -----------------
# finding #1 (BLOCKER): the restore must never command the truck above the driver's CURRENT live
# stock set -- neither by ignoring a driver SET- during the cap, nor by ignoring one during the
# restore window itself.

def test_restore_blocked_when_driver_set_lower_than_commanded_during_cap():
  # THE critical regression: while capping, the driver independently taps SET- on the truck's OWN
  # stock ACC, landing the reported stock set BELOW anything speedadjust ever asked for -- that is the
  # driver's own intent. When the cap later clears, speedadjust must NOT restore up to the stale
  # pre-cap ceiling (it would accelerate the truck past what the driver just chose).
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle_pub(c, V75, V60)                              # cap engages + fully slews; ceiling latched 75
  target_now = c.mem_params.last["target"]              # our own commanded floor (~V60+POLICE_MARGIN)
  driver_set = target_now - 5 * MPH                      # driver pushed noticeably BELOW our own target
  for _ in range(5):
    _tick(c, v_cruise=V75, v_ego=V60, sm=_sm(speed=driver_set))
  c._police = {"state": "clear"}                        # source clears
  _tick(c, sm=_sm(speed=driver_set))
  c._release_t -= (RELEASE_S + 0.1)
  _tick(c, sm=_sm(speed=driver_set))                     # cap fully releases
  assert c._restore_ceiling is None                      # NO restore offered -- driver's own intent wins
  payload = c.mem_params.last
  assert payload == {} or payload.get("dir") != "inc"    # never asked to press SET+


def test_restore_proceeds_when_driver_dip_explained_by_our_own_tap_lag():
  # counter-case: a small dip that's still ABOVE anything we ever commanded (our own tap latency, not
  # a genuine driver SET-) must NOT block the restore.
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle_pub(c, V75, V60)
  target_now = c.mem_params.last["target"]
  slight_lag = target_now + 0.1 * MPH                    # still above our own commanded floor
  _tick(c, v_cruise=V75, v_ego=V60, sm=_sm(speed=slight_lag))
  c._police = {"state": "clear"}
  _tick(c, sm=_sm(speed=slight_lag))
  c._release_t -= (RELEASE_S + 0.1)
  _tick(c, sm=_sm(speed=slight_lag))
  assert c._restore_ceiling is not None                  # restore proceeds normally
  assert abs(c._restore_ceiling - V75) < 0.5


def test_restore_ceiling_ratchets_down_to_live_driver_set_during_restore():
  # the restore window itself must track a driver SET- happening WHILE it's in progress -- never
  # publish/press toward anything above the truck's own current live reported set. A restore starts
  # BELOW the ceiling by design (that's the gap being walked up), so the ratchet is relative to the
  # last OBSERVED value during the restore (mirrors icbm_pnw.RestoreGuard), not the raw current
  # reading -- establish a rising baseline first (as real taps would produce), then a genuine decrease.
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle_pub(c, V75, V60)
  c._police = {"state": "clear"}
  _tick(c)
  c._release_t -= (RELEASE_S + 0.1)
  _tick(c)                                                # restore begins, ceiling ~= V75
  assert abs(c._restore_ceiling - V75) < 0.5
  near_ceiling = V75 - 2 * MPH                            # taps have walked it most of the way up
  _tick(c, sm=_sm(speed=near_ceiling))                    # establishes the observed baseline
  assert c._restore_ceiling is not None and c._restore_ceiling > near_ceiling
  lower = 65 * MPH                                        # driver taps SET- mid-restore (a real drop)
  assert lower < near_ceiling - SA_DRIVER_LOWER_TOL        # sanity: a real, unambiguous decrease
  _tick(c, sm=_sm(speed=lower))
  assert c._restore_ceiling <= lower + 0.05                # ratcheted down (2-decimal publish rounding)
  payload = c.mem_params.last
  if payload.get("dir") == "inc":
    assert payload["target"] <= lower + 0.05
    assert payload["ceiling"] <= lower + 0.05


def test_restore_ceiling_never_ratchets_up():
  # the ratchet is DOWN-only: a stock set reading ABOVE the current restore ceiling (e.g. our own tap
  # having just landed) must never raise the ceiling past the original driver ceiling.
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle_pub(c, V75, V60)
  c._police = {"state": "clear"}
  _tick(c)
  c._release_t -= (RELEASE_S + 0.1)
  _tick(c)
  ceiling_before = c._restore_ceiling
  above = V75 + 10 * MPH                                  # implausible reading above the driver's set
  _tick(c, sm=_sm(speed=above))
  assert c._restore_ceiling <= ceiling_before + 1e-6


def test_pedal_press_during_cap_does_not_stop_dec_publish():
  # restore-hardening must NEVER weaken the dec/cap slow-down side: a pedal press mid-cap clears the
  # RESTORE bookkeeping (_pub_ceiling) but the cap (dec) target must keep publishing every tick.
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle_pub(c, V75, V60)
  assert c._pub_ceiling is not None
  _tick(c, sm=_sm(gas=True))                              # driver presses the gas mid-cap
  assert c._pub_ceiling is None                            # restore bookkeeping cleared
  assert c._min_pub_target is None
  payload = c.mem_params.last
  assert payload is not None and payload != {}
  assert "dir" not in payload                              # still a dec publish, never dropped
  assert payload.get("ceiling") is not None


def test_pedal_press_during_cap_prevents_later_restore():
  # mirrors ces_pnw.IcbmEpisode: a pedal press mid-cap kills the restore episode identity entirely --
  # even after the pedal is released and the cap continues/re-releases, no restore is offered this run.
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle_pub(c, V75, V60)
  _tick(c, sm=_sm(gas=True))                              # pedal mid-cap
  _tick(c)                                                # pedal released, still capping
  c._police = {"state": "clear"}
  _tick(c)
  c._release_t -= (RELEASE_S + 0.1)
  _tick(c)                                                # cap releases
  assert c._restore_ceiling is None                        # no restore -- episode was killed


# finding #3: the restore must not BEGIN/CONTINUE while laterally loaded in a curve.

def test_restore_pauses_in_curve():
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle_pub(c, V75, V60)
  c._police = {"state": "clear"}
  _tick(c)
  c._release_t -= (RELEASE_S + 0.1)
  _tick(c)                                                # restore begins
  assert c._restore_ceiling is not None
  n_before = len(c.mem_params.target_calls)
  c._last_t -= 0.5
  c._pub_last -= 1.0
  out = c.cap(_sm(speed=V60, lat_accel=2.5, v_ego=30.0), V75, V75, V60, True)
  assert out == V75                                        # neutral op-long return, unaffected
  assert c._restore_ceiling is not None                     # episode still alive -- PAUSED, not aborted
  assert len(c.mem_params.target_calls) == n_before         # no fresh publish while paused


def test_restore_resumes_after_curve_clears():
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle_pub(c, V75, V60)
  c._police = {"state": "clear"}
  _tick(c)
  c._release_t -= (RELEASE_S + 0.1)
  _tick(c)                                                # restore begins
  c._last_t -= 0.5
  c._pub_last -= 1.0
  c.cap(_sm(speed=V60, lat_accel=2.5, v_ego=30.0), V75, V75, V60, True)   # in curve: paused
  assert c._restore_ceiling is not None
  _tick(c, sm=_sm(speed=V60))                              # curve clears (no modelV2 -> not in curve)
  payload = c.mem_params.last
  assert payload.get("dir") == "inc"                        # resumed


def test_uninitialized_cruise_cancels_pending_restore():
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle(c, V75, V60)
  c._police = {"state": "clear"}
  _tick(c)
  c._release_t -= (RELEASE_S + 0.1)
  _tick(c)                                               # restore begins
  assert c._restore_ceiling is not None
  c._last_t -= 0.5
  c._pub_last -= 0.5
  c.cap(None, V75, V60, V60, False)                      # cruise goes uninitialized
  assert c._restore_ceiling is None
  assert c.mem_params.last == {}


# ---- speedadjustreset2pnw (2026-08-16): manual set-change = override, resets BOTH latches ----------
# Driver directive: ANY change to the cruise SET speed (up or down) is an explicit "resume — don't
# slow me for this" override — more intuitive for a novice than hunting the AutoSpeedReduce toggle.
# These tests use op-long controllers (self._long_ok=True) unless noted, matching the module
# docstring's proof that v_cruise_set is feedback-safe there (cap()'s own return value never writes
# back into CS.vCruise, so _is_own_actuation() is always False / a no-op on that path — see the
# separate stock-ACC section below for the case where it actually matters).

V80 = 80 * MPH


def test_manual_bump_up_releases_police_cap_and_does_not_relatch():
  c = _ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})   # ttr ~24 s < 30 -> latches
  settled = _settle(c, V75, V60)
  assert abs(settled - (V60 + POLICE_MARGIN)) < 1e-6
  assert c._police_latched is True
  # driver bumps the set UP -- explicit override
  out = _cap(c, V80, V60, v_cruise_set=V80)
  assert out == V80                              # released immediately, no debounce wait
  assert c._cap_out is None
  assert c._police_suppressed is True
  # same alert persists (still well within the 30 s approach window) -- must NOT re-latch/re-cap
  for _ in range(5):
    c._last_t -= 0.5
    out = c.cap(None, V80, V80, V60, True)
  assert out == V80
  assert c._cap_out is None


def test_manual_bump_down_releases_police_cap_and_does_not_relatch():
  c = _ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle(c, V75, V60)
  assert c._police_latched is True
  v_new = 55 * MPH                                # driver bumps DOWN, still an explicit override
  out = _cap(c, v_new, V60, v_cruise_set=v_new)
  assert out == v_new
  assert c._police_suppressed is True
  for _ in range(5):
    c._last_t -= 0.5
    out = c.cap(None, v_new, v_new, V60, True)
  assert out == v_new                             # not re-capped while the same alert persists
  assert c._cap_out is None


def test_manual_change_then_alert_clears_and_new_alert_rearms():
  c = _ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle(c, V75, V60)
  out = _cap(c, V80, V60, v_cruise_set=V80)         # dismiss via manual bump
  assert out == V80
  assert c._police_suppressed is True
  # the SAME report clears
  c._police = {"state": "clear"}
  _cap(c, V80, V60, v_cruise_set=V80)
  assert c._police_latched is False
  assert c._police_suppressed is False              # re-armed
  # a NEW alert appears -- must cap again, normally
  c._police = {"state": "alert", "dist_mi": 0.4}
  out = _settle(c, V80, V60, v_cruise_set=V80)
  assert abs(out - (V60 + POLICE_MARGIN)) < 1e-6


def test_tiny_jitter_in_set_does_not_reset():
  c = _ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  settled = _settle(c, V75, V60)
  assert c._police_latched is True
  jitter = V75 + (SET_CHANGE_EPS * 0.5)             # well under the epsilon
  out = _cap(c, jitter, V60, v_cruise_set=jitter)
  assert abs(out - settled) < 1e-6                  # cap still fully engaged, unchanged
  assert c._police_suppressed is False
  assert c._cap_out is not None


def test_manual_change_releases_active_limit_drop_cap():
  c = _drop(mode=2)                                 # sl_ref=V60, ratio=V75/V60, sl=V45 -> target 56.25
  settled = _settle(c, V75, V60)
  assert settled < V75                              # confirm it's actively trimming
  v_new = 50 * MPH                                  # driver nudges the set -- explicit override
  out = _cap(c, v_new, V60, v_cruise_set=v_new)
  assert out == v_new                               # released immediately
  assert c._cap_out is None
  assert abs(c._sl_ref - c._sl) < 1e-6               # baseline re-anchored to the current (dropped) limit
  # future FURTHER drops below this new baseline still re-cap (not permanently disabled)
  c._sl = 40 * MPH
  out2 = _settle(c, v_new, V60, v_cruise_set=v_new)
  assert out2 < v_new


def test_no_reset_across_uninitialized_to_initialized_transition():
  c = _ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  V90 = 90 * MPH
  _cap(c, V90, V60, v_cruise_set=V90, v_cruise_initialized=False)   # not set yet
  assert c._last_v_set is None
  out = _cap(c, V75, V60, v_cruise_set=V75, v_cruise_initialized=True)   # driver sets cruise
  assert c._police_suppressed is False              # NOT treated as a manual override reset
  assert c._last_v_set == V75
  # the police cap engages normally on the very next tick(s)
  out = _settle(c, V75, V60)
  assert abs(out - (V60 + POLICE_MARGIN)) < 1e-6


# ---- speedadjustreset2pnw + stock-ACC: our OWN button taps must never self-cancel the feature ------
# See _is_own_actuation()'s docstring: on a pcmCruise=True car, CS.vCruise mirrors the truck's live
# stock ACC dial, which speedadjust-exec2pnw's own SET-/SET+ taps actually move. Without the guard,
# every one of our own taps would look identical to a driver override.

def test_own_dec_taps_do_not_self_cancel_stock_cap():
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _cap(c, V75, V60)                                 # tick 1: engage, _cap_out seeded at V75, _last_v_set=V75
  assert c._cap_out is not None
  # simulate the truck's OWN reported set moving DOWN toward our _cap_out (our own SET- taps landing)
  for _ in range(6):
    c._last_t -= 0.5
    new_set = max(c._cap_out - 0.01, MIN_CAP)        # stays at/just above our own current target
    c.cap(None, new_set, new_set, V60, True)
  assert c._police_suppressed is False              # never mistaken for a manual override
  assert c._cap_out is not None                     # still actively capping


def test_own_restore_taps_do_not_self_cancel_stock_restore():
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle_pub(c, V75, V60)                          # cap engages + fully slews, ceiling latched V75
  c._police = {"state": "clear"}
  _tick(c)
  c._release_t -= (RELEASE_S + 0.1)
  _tick(c)                                          # restore begins, _restore_ceiling ~= V75
  assert c._restore_ceiling is not None
  ceiling = c._restore_ceiling
  # simulate the truck's OWN reported set moving UP toward the restore ceiling (our own SET+ taps)
  cur = c._last_v_set
  for _ in range(6):
    cur = min(cur + 0.4 * MPH, ceiling)
    c._last_t -= 0.5
    c._pub_last -= 0.5
    c.cap(None, cur, cur, V60, True)
  assert c._police_suppressed is False              # never mistaken for a manual override
  assert c._restore_ceiling is not None             # restore still in progress, not cancelled


def test_genuine_driver_bump_on_stock_car_still_resets():
  # a stock-ACC driver bump that is NOT explainable by our own commanded target (a big jump well past
  # what we ourselves would ever ask for) must still be treated as a real manual override.
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle_pub(c, V75, V60)
  assert c._pub_ceiling is not None
  driver_set = V80                                  # far above anything a dec-cap would ever command
  _tick(c, v_cruise=driver_set, v_cruise_set=driver_set, sm=None)
  assert c._police_suppressed is True
  assert c._cap_out is None


# ---- speedadjustreset2pnw hardening (2026-08-16, second Fable + Gemini review pass) -----------------

# FIX A (Fable major): the Tesla's cruiseState.speed is floored at max(DI_digitalSpeed·conv, 1e-3),
# never exactly 0, so v_cruise_initialized stays True straight through STANDBY -- the uninit guard
# alone never protects it. _cruise_engaged() additionally gates on cruiseState.enabled.

def test_engage_transition_near_latched_alert_does_not_suppress():
  c = _ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})   # ttr ~24 s < 30 -> latches
  floor_val = 1e-3                                    # Tesla standby floor (m/s) -- never exactly 0
  c.cap(_sm(cruise_enabled=False), floor_val, floor_val, V60, True)      # disengaged tick
  assert c._police_latched is True                    # latching doesn't check engagement
  assert c._last_v_set is None                         # FIX A: not-engaged -> no baseline recorded
  c._last_t -= 0.5
  c.cap(_sm(cruise_enabled=True), V75, V75, V60, True)  # ENGAGE transition: set jumps floor -> V75
  assert c._police_suppressed is False                  # must NOT read as a manual override
  assert c._last_v_set == V75
  out = None
  for _ in range(400):
    c._last_t -= 0.5
    out = c.cap(_sm(cruise_enabled=True), V75, V75, V60, True)
  assert abs(out - (V60 + POLICE_MARGIN)) < 1e-6        # the cap engages normally afterward


def test_two_engaged_ticks_with_no_delta_after_engage_does_not_misfire():
  c = _ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  c.cap(_sm(cruise_enabled=False), V75, V75, V60, True)
  c.cap(_sm(cruise_enabled=True), V75, V75, V60, True)   # engage, same set as before disengage
  assert c._police_suppressed is False
  c._last_t -= 0.5
  c.cap(_sm(cruise_enabled=True), V75, V75, V60, True)   # second engaged tick, still unchanged
  assert c._police_suppressed is False


def test_not_engaged_tick_never_records_baseline_even_mid_drive():
  c = _ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  c.cap(_sm(cruise_enabled=True), V75, V75, V60, True)
  assert c._last_v_set == V75
  c._last_t -= 0.5
  c.cap(_sm(cruise_enabled=False), V75, V75, V60, True)  # driver disengages (no set change)
  assert c._last_v_set is None                           # baseline forgotten while disengaged


# FIX B (Fable major): _is_own_actuation() reads _cap_out/_restore_ceiling at OBSERVATION time, but
# our own SET+/SET- taps have real actuation->CAN->carState latency while those fields null INSTANTLY
# at phase transitions -- an in-flight tap can land 1-2 ticks after and get misread as an override.

def test_inflight_tap_after_restore_preempted_does_not_self_cancel():
  # mirrors the exact bug scenario: a NEW alert preempts an in-progress restore (_restore_ceiling
  # nulled, fresh _cap_out seeded, same tick) -- but a SET+ tap issued just before the preemption can
  # still land afterward, moving v_cruise_set UP right as the new cap is trying to hold it down.
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle_pub(c, V75, V60)
  c._police = {"state": "clear"}
  _tick(c)
  c._release_t -= (RELEASE_S + 0.1)
  _tick(c)                                                # restore begins
  assert c._restore_ceiling is not None
  c._police = {"state": "alert", "dist_mi": 0.4}          # a NEW alert preempts the restore
  _tick(c)
  assert c._restore_ceiling is None
  assert c._cap_out is not None
  late_tap_set = c._last_v_set + 1.0 * MPH                # in-flight SET+ tap lands one tick later
  c.cap(None, late_tap_set, late_tap_set, V60, True)
  assert c._police_suppressed is False                    # NOT mistaken for a driver override


def test_actuation_grace_window_expires_then_detects_real_change():
  # negative control for the above: once the grace window has genuinely elapsed, the SAME kind of
  # delta must still register as a real driver override -- the grace window is time-bounded, not a
  # permanent hole.
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle_pub(c, V75, V60)
  c._police = {"state": "clear"}
  _tick(c)
  c._release_t -= (RELEASE_S + 0.1)
  _tick(c)                                                # restore begins
  c._police = {"state": "alert", "dist_mi": 0.4}          # a NEW alert preempts the restore
  _tick(c)
  assert c._cap_out is not None
  c._last_actuation_transition_t -= (SA_ACTUATION_GRACE_S + 0.5)   # grace window elapses
  driver_set = c._last_v_set + 5 * MPH
  c.cap(None, driver_set, driver_set, V60, True)
  assert c._police_suppressed is True


# FIX C (Gemini BLOCKER): on stock-ACC, a value-tolerance check can't distinguish our own tap from a
# driver's SAME-direction tap -- _is_own_actuation() is now direction-only there. Only an OPPOSITE-
# direction move is unambiguous. Op-long keeps full any-direction detection (no actuator sharing).

def test_fixc_stock_acc_opposite_direction_during_dec_cap_resets():
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle_pub(c, V75, V60)
  assert c._cap_out is not None
  bumped = c._last_v_set + 5 * MPH                        # SET+ -- opposite our own SET- dec taps
  c.cap(None, bumped, bumped, V60, True)
  assert c._police_suppressed is True


def test_fixc_stock_acc_same_direction_during_dec_cap_does_not_reset():
  # accepted limitation (documented in _is_own_actuation()): a single same-direction SET- during an
  # active dec-cap is indistinguishable from our own tap, even a large one.
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle_pub(c, V75, V60)
  assert c._cap_out is not None
  nudged_down = c._last_v_set - 5 * MPH
  c.cap(None, nudged_down, nudged_down, V60, True)
  assert c._police_suppressed is False


def test_fixc_op_long_either_direction_resets():
  c_up = _ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle(c_up, V75, V60)
  bumped_up = V75 + 5 * MPH
  out_up = _cap(c_up, bumped_up, V60, v_cruise_set=bumped_up)
  assert c_up._police_suppressed is True
  assert out_up == bumped_up

  c_down = _ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle(c_down, V75, V60)
  bumped_down = 55 * MPH
  _cap(c_down, bumped_down, V60, v_cruise_set=bumped_down)
  assert c_down._police_suppressed is True


# FIX D (Fable minor): only dismiss an alert the driver could actually perceive acting on.

def test_fixd_pre_latch_set_change_does_not_suppress_future_alert():
  c = _ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 15.0})   # ttr >> 30 s, not latched
  _cap(c, V75, V60)
  assert c._police_latched is False
  bumped = V75 + 5 * MPH
  _cap(c, bumped, V60, v_cruise_set=bumped)
  assert c._police_suppressed is False                    # FIX D: nothing to dismiss yet
  c._police = {"state": "alert", "dist_mi": 0.4}           # the report gets close later
  out = _settle(c, bumped, V60, v_cruise_set=bumped)
  assert abs(out - (V60 + POLICE_MARGIN)) < 1e-6           # caps normally, unaffected by the earlier nudge


# FIX E (Fable minor, telemetry only): the override-release block must clear _engaged too.

def test_fixe_engaged_cleared_on_manual_override_release():
  c = _ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle(c, V75, V60)
  assert c._engaged is True
  bumped = V75 + 5 * MPH
  _cap(c, bumped, V60, v_cruise_set=bumped)
  assert c._engaged is False


# ---- speedadjustreset2pnw hardening (2026-08-16, third review pass) --------------------------------

# FIX 1 (Fable F2 / Gemini #2): the police-suppression gate must key off _police_latched ALONE. A
# police-sourced _cap_out always implies _police_latched already, so "or _cap_out is not None" only
# ever added LIMIT-DROP (mode 2) caps -- wrongly suppressing a pending, not-yet-latched police alert
# whenever the driver overrode an active limit trim.

def test_fix1_override_releases_limit_drop_but_does_not_suppress_pending_police():
  c = _drop(mode=2, police={"state": "alert", "dist_mi": 15.0})   # far away, ttr >> 30s -- NOT latched
  settled = _settle(c, V75, V60)
  assert settled < V75                          # confirm it's actively trimming via the limit-drop cap
  assert c._police_latched is False             # the pending alert genuinely hasn't latched yet
  v_new = 50 * MPH                              # driver nudges the set -- explicit override
  out = _cap(c, v_new, V60, v_cruise_set=v_new)
  assert out == v_new                           # limit trim released immediately
  assert c._cap_out is None
  assert c._police_suppressed is False          # FIX 1: NOT suppressed -- this was a limit-drop cap, not police
  # later, the SAME report closes into the approach window -- must still engage normally. c._sl stays
  # V45 throughout this test (never changed), so the police target is V45 + POLICE_MARGIN, not V60's.
  c._police = {"state": "alert", "dist_mi": 0.4}
  out2 = _settle(c, v_new, V60, v_cruise_set=v_new)
  assert c._police_latched is True
  assert abs(out2 - (V45 + POLICE_MARGIN)) < 1e-6


# FIX 2 (Fable F1): the driver-intervening (gas/brake/ACC-off) block that clears an in-progress
# restore must ALSO stamp the actuation-transition grace window (edge-guarded on a restore having
# actually been active), or a late in-flight own SET+ tap from the just-cleared restore can land after
# a brand-new cap has engaged and get misread as a driver override, suppressing it.

def test_fix2_late_tap_after_driver_intervening_clears_restore_does_not_self_cancel():
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle_pub(c, V75, V60)
  c._police = {"state": "clear"}
  _tick(c)
  c._release_t -= (RELEASE_S + 0.1)
  _tick(c)                                                # restore begins (stamps the unrelated release transition)
  assert c._restore_ceiling is not None
  # age out that earlier, unrelated stamp so ONLY FIX 2's own stamp can protect the late tap below
  c._last_actuation_transition_t -= (SA_ACTUATION_GRACE_S + 0.5)
  c.cap(_sm(gas=True), c._last_v_set, c._last_v_set, V60, True)   # driver gas mid-restore -- FIX 2 stamps here
  assert c._restore_ceiling is None
  # a NEW alert appears, engaging a fresh dec-cap
  c._police = {"state": "alert", "dist_mi": 0.4}
  _tick(c, sm=_sm(gas=False))
  assert c._cap_out is not None
  # an in-flight own SET+ tap from the just-cleared restore lands now
  late_tap_set = c._last_v_set + 1.0 * MPH
  c.cap(None, late_tap_set, late_tap_set, V60, True)
  assert c._police_suppressed is False                    # NOT mistaken for a driver override


# --- speedlimitconfirm2pnw: a LOWER posted limit must persist before it is acted on ---------------
# 2026-08-18 live drive: a mid-drive reboot made mapd re-match position and briefly land on a parallel
# surface street, so MapSpeedLimit read 25 mph on I-5 at 70. With AutoSpeedReduce=2 that feeds
# _limit_drop_cap() directly (which caps at max(sl, sl*ratio)) -- a transient ~29 mph command at 70 mph.

class _MemParams:
  """Stands in for the /dev/shm Params store; `value` is the raw MapSpeedLimit string mapd_configd writes."""
  def __init__(self, value):
    self.value = value

  def get(self, key, return_default=False):
    if key != "MapSpeedLimit":
      return None
    return None if self.value is None else str(self.value)


def _sl_reader(initial_sl, first_read):
  c = SpeedAdjustController(_CP(True), params=_Params())
  c.mem_params = _MemParams(first_read)
  c._sl = initial_sl
  c._sl_valid_t = time.monotonic()
  return c


def test_limit_drop_is_held_until_confirmed():
  c = _sl_reader(V60, V45 / MPH * MPH)          # 60 -> 45 mph drop appears
  assert c._read_speed_limit() == V60, "an unconfirmed drop must keep the previous limit"


def test_limit_drop_is_accepted_after_confirmation_window():
  c = _sl_reader(V60, V45)
  assert c._read_speed_limit() == V60
  c._sl_pending_t -= (SL_DROP_CONFIRM_S + 0.1)  # simulate the value persisting past the window
  assert c._read_speed_limit() == V45


def test_transient_drop_never_takes_effect():
  # the exact reboot artifact: 60 -> 25 for one read -> back to 60. The 25 must never be returned.
  c = _sl_reader(V60, 25 * MPH)
  assert c._read_speed_limit() == V60
  c.mem_params.value = V60
  assert c._read_speed_limit() == V60
  assert c._sl_pending == 0.0, "pending drop must be discarded once the reading moves off it"


def test_a_different_lower_value_restarts_the_window():
  c = _sl_reader(V60, V45)
  c._read_speed_limit()
  c._sl_pending_t -= 10.0                       # age the window well past confirmation
  aged_t = c._sl_pending_t
  c.mem_params.value = 30 * MPH                 # reading moved to another low value
  assert c._read_speed_limit() == V60, "a NEW pending value must not inherit the old window"
  assert c._sl_pending == 30 * MPH
  assert c._sl_pending_t > aged_t, "the confirmation window restarts on a new pending value"


def test_rising_limit_must_PERSIST_before_it_is_adopted():
  """CHANGED by sazoneset2pnw. This test used to be `test_rising_limit_is_accepted_immediately` and
  pinned "the release direction is never gated". Fable measured why that was wrong once a limit drop
  became permanent: one stray HIGHER read re-anchored the baseline, the return to the real limit then
  confirmed as a 'drop' 2 s later, and the set was trimmed with the police restore withheld. Under the
  zone model that trim is never given back. A higher limit now has to persist SL_RISE_CONFIRM_S."""
  c = _sl_reader(V45, V60)
  assert c._read_speed_limit() == V45, "a single higher read was adopted at once"
  assert c._sl_rise_pending == V60
  c._sl_rise_pending_t -= (SL_RISE_CONFIRM_S + 0.1)
  assert c._read_speed_limit() == V60, "a rise that persisted was not adopted"
  assert c._sl_pending == 0.0 and c._sl_rise_pending == 0.0


def test_unknown_read_breaks_the_confirmation_chain():
  c = _sl_reader(V60, V45)
  c._read_speed_limit()
  assert c._sl_pending == V45
  c.mem_params.value = None                     # map dropout
  c._read_speed_limit()
  assert c._sl_pending == 0.0, "a drop that flickers valid/unknown has not persisted"


def test_dropout_hold_still_works():
  c = _sl_reader(V60, None)
  assert c._read_speed_limit() == V60, "brief dropout must still hold the last valid limit"


# --- policetier2pnw: only a CONFIRMED police report may command a slowdown ------------------------
# Driver directive 2026-08-18: "slow down only for the ones that show up on Waze". An UNCONFIRMED
# report (past the lifetime the Waze app would give it) still displays -- in amber -- but must never
# cap the car. A missing/unknown tier keeps the previous behaviour (treated as confirmed).

def _police(dist_mi, tier=None, state="alert", cap_mi="same", cap_uuid="c"):
  """Display line as location_servicesd publishes it. `tier` present => post-split payload, and the
  CONTROL channel is the separate `cap` dict (nearest CONFIRMED report). cap_mi="same" mirrors the
  display distance; None means nothing confirmed ahead. cap_uuid defaults to a single shared identity
  so pre-policelatch2pnw tests keep their exact behaviour; pass it explicitly to model the CONTROL
  channel switching to a DIFFERENT report (which is what happens once you drive past one)."""
  p = {"state": state, "dist_mi": dist_mi}
  if tier is not None:
    p["tier"] = tier
    if cap_mi == "same":
      cap_mi = dist_mi if tier == "confirmed" else None
    if cap_mi is not None:
      p["cap"] = {"dist_mi": cap_mi, "uuid": cap_uuid, "age_min": 1}
  return p


def test_confirmed_police_report_still_caps():
  c = _ctrl(sl=V45, police=_police(0.3, "confirmed"))
  assert c._police_cap(V75, V75) == V45 + POLICE_MARGIN


def test_unconfirmed_police_report_never_caps():
  c = _ctrl(sl=V45, police=_police(0.3, "unconfirmed"))
  assert c._police_cap(V75, V75) is None


def test_missing_tier_is_treated_as_confirmed():
  # pre-tier location_servicesd payload / cached body -> must not silently disable police braking
  c = _ctrl(sl=V45, police=_police(0.3))
  assert c._police_cap(V75, V75) == V45 + POLICE_MARGIN


def test_report_aging_out_of_confirmed_mid_approach_releases_the_latch():
  c = _ctrl(sl=V45, police=_police(0.3, "confirmed"))
  assert c._police_cap(V75, V75) == V45 + POLICE_MARGIN
  assert c._police_latched is True
  c._police = _police(0.3, "unconfirmed")          # crossed its effective lifetime while approaching
  assert c._police_cap(V75, V75) is None
  assert c._police_latched is False, "an unconfirmed report must not keep holding the cap"


def test_unconfirmed_report_does_not_latch_for_a_later_confirmed_one():
  c = _ctrl(sl=V45, police=_police(0.3, "unconfirmed"))
  c._police_cap(V75, V75)
  assert c._police_latched is False
  c._police = _police(0.3, "confirmed")
  assert c._police_cap(V75, V75) == V45 + POLICE_MARGIN


# --- display/control split: the controller must read `cap`, never the displayed report -----------

def test_amber_display_line_still_caps_for_a_confirmed_report_behind_it():
  """The masking bug: an amber report at 0.2 mi is what is DISPLAYED, but a confirmed one at 0.5 mi
  is what the car must react to. Reading the display fields here suppressed the slowdown entirely.
  0.5 mi at 75 mph is ~24 s of driving, inside POLICE_ENGAGE_S; 0.8 mi would be ~38 s and would
  legitimately not latch yet, which is a property of the engage window, not of the split."""
  p = _police(0.2, "unconfirmed", cap_mi=0.5)
  c = _ctrl(sl=V45, police=p)
  assert c._police_cap(V75, V75) == V45 + POLICE_MARGIN
  assert c._police_latched is True


def test_control_channel_distance_is_used_not_the_display_distance():
  # display 0.05 mi (would latch instantly), confirmed report 6 mi out (must NOT latch at 75 mph)
  c = _ctrl(sl=V45, police=_police(0.05, "unconfirmed", cap_mi=6.0))
  assert c._police_cap(V75, V75) is None
  assert c._police_latched is False


def test_no_confirmed_report_means_no_cap_however_close_the_amber_one_is():
  c = _ctrl(sl=V45, police=_police(0.05, "unconfirmed", cap_mi=None))
  assert c._police_cap(V75, V75) is None


def test_legacy_payload_without_tier_uses_the_display_line():
  # pre-split location_servicesd: no `tier`, no `cap` -> previous behaviour, braking still works
  c = _ctrl(sl=V45, police={"state": "alert", "dist_mi": 0.3})
  assert c._police_cap(V75, V75) == V45 + POLICE_MARGIN


# --- Fable finding 3: the key-based dismissal had no regression net at all --------------------------

def test_dismissal_holds_while_the_same_report_is_the_cap():
  c = _ctrl(sl=V45, police=_police(0.3, "confirmed"))
  c._police_cap(V75, V75)
  c._police_suppressed = True
  c._police_suppressed_uuid = _police_key(c._police["cap"])
  assert c._police_cap(V75, V75) is None, "a dismissed report must stay dismissed"


def test_dismissal_clears_when_a_different_report_becomes_the_cap():
  c = _ctrl(sl=V45, police=_police(0.3, "confirmed"))
  c._police_cap(V75, V75)
  c._police_suppressed = True
  c._police_suppressed_uuid = "some-other-report"
  assert c._police_cap(V75, V75) == V45 + POLICE_MARGIN, "a NEW report must not inherit a dismissal"


def test_a_missing_key_never_carries_a_dismissal_onto_another_report():
  # two reports both lacking an alert_id would both key None; None == None must NOT suppress
  p = {"state": "alert", "dist_mi": 0.3, "tier": "unconfirmed",
       "cap": {"dist_mi": 0.3, "uuid": None, "key": None}}
  c = _ctrl(sl=V45, police=p)
  c._police_suppressed = True
  c._police_suppressed_uuid = None
  assert c._police_cap(V75, V75) == V45 + POLICE_MARGIN


def test_switching_reports_also_drops_the_stale_latch():
  # Fable finding 1: carrying the latch across a report switch skipped the 30 s engage gate, capping
  # immediately for a report still miles away.
  c = _ctrl(sl=V45, police=_police(0.3, "confirmed"))
  assert c._police_cap(V75, V75) == V45 + POLICE_MARGIN
  assert c._police_latched is True
  c._police_suppressed = True
  c._police_suppressed_uuid = "old-report"
  c._police = _police(6.0, "confirmed", cap_mi=6.0)   # different report, 6 mi out
  assert c._police_cap(V75, V75) is None, "must re-earn the latch via the approach window"
  assert c._police_latched is False


def test_cap_absent_clears_both_latch_and_dismissal():
  c = _ctrl(sl=V45, police=_police(0.3, "confirmed"))
  c._police_cap(V75, V75)
  c._police_suppressed = True
  c._police = _police(0.3, "unconfirmed", cap_mi=None)
  assert c._police_cap(V75, V75) is None
  assert c._police_latched is False and c._police_suppressed is False


# ---- sanorestore2pnw: a slowdown for a lower POSTED LIMIT never walks the set back up -------------
# Driver directive 2026-09-13: "I want the button control management to never accelerate to the previous
# speed. I only want it to accelerate to the previous speed if there is a police warning -- that's the
# only exception." Curve slowdowns (icbm2pnw) are a separate brain and keep restoring, per the same
# conversation. These tests drive a full stock-ACC episode through cap() and assert on what the executor
# would actually be told (the SpeedAdjustTarget publishes), not on internal flags alone.

def _release(c, v_cruise=V75, v_ego=V60):
  """Clear sources have already been set by the caller: run through the release debounce."""
  _tick(c, v_cruise=v_cruise, v_ego=v_ego)
  if c._release_t is not None:
    c._release_t -= (RELEASE_S + 0.1)
  _tick(c, v_cruise=v_cruise, v_ego=v_ego)


def _published_incs(c):
  return [p for _, p in c.mem_params.calls if isinstance(p, dict) and p.get("dir") == "inc"]


def test_a_limit_drop_slowdown_does_NOT_restore_when_the_limit_rises():
  """THE DIRECTIVE. 60 -> 45 zone trims the set 75 -> 56; the limit rises back to 60 and the set must
  stay where it is. Before this change the executor was told to SET+ back to 75."""
  c = _stock_ctrl(mode=2, sl_ref=V60, ratio=V75 / V60, sl=V45)
  _settle_pub(c, V75, V45)
  assert c.mem_params.last and c.mem_params.last["target"] < V75 - 5 * MPH, "the limit drop never capped"
  assert c._ep_limit_drop is True
  c._sl = V60                                            # limit rises back
  _release(c)
  for _ in range(20):
    _tick(c)
  assert _published_incs(c) == [], f"a limit-drop episode published a restore: {_published_incs(c)}"
  assert c._restore_ceiling is None
  assert c._no_restore_why == "limitDrop"


def test_a_police_slowdown_STILL_restores():
  """The one exception, pinned so this change cannot quietly remove it."""
  c = _stock_ctrl(mode=2, sl=V60, sl_ref=V60, ratio=V75 / V60, police={"state": "alert", "dist_mi": 0.4})
  _settle_pub(c, V75, V60)
  assert c._ep_limit_drop is False
  c._police = {"state": "clear"}
  _release(c)
  incs = _published_incs(c)
  assert incs and abs(incs[-1]["target"] - V75) < 0.01, "a police slowdown no longer restores"
  assert c._no_restore_why is None


def test_an_episode_with_BOTH_police_and_a_limit_drop_does_not_restore():
  """Restoring would raise the set past a limit that dropped during the episode."""
  c = _stock_ctrl(mode=2, sl=V60, sl_ref=V60, ratio=V75 / V60, police={"state": "alert", "dist_mi": 0.4})
  _settle_pub(c, V75, V60)                               # police cap engages first
  c._sl = V45                                            # ...then the limit drops mid-episode
  for _ in range(10):
    _tick(c, v_ego=V45)
  assert c._ep_limit_drop is True
  c._police = {"state": "clear"}
  c._sl = V60
  _release(c)
  for _ in range(10):
    _tick(c)
  assert _published_incs(c) == [], "a mixed police + limit-drop episode restored past the dropped limit"


def test_the_flag_does_not_leak_into_the_NEXT_episode():
  """A limit-drop episode, then later an unrelated police-only episode: the police one must restore.
  Without the per-episode reset the first episode would silently disable every later police restore."""
  c = _stock_ctrl(mode=2, sl_ref=V60, ratio=V75 / V60, sl=V45)
  _settle_pub(c, V75, V45)
  c._sl = V60
  _release(c)
  for _ in range(10):
    _tick(c)
  assert c._ep_limit_drop is True and _published_incs(c) == []
  n_before = len(c.mem_params.calls)
  c._police = {"state": "alert", "dist_mi": 0.4}          # a new, police-only episode on the 60 road
  c._ratio = V75 / V60
  _settle_pub(c, V75, V60)
  assert c._ep_limit_drop is False, "the limit-drop flag leaked into the next episode"
  c._police = {"state": "clear"}
  _release(c)
  incs = [p for _, p in c.mem_params.calls[n_before:] if isinstance(p, dict) and p.get("dir") == "inc"]
  assert incs, "a police-only episode after a limit-drop episode did not restore"


def test_the_withheld_restore_is_LOGGED_and_in_telemetry(monkeypatch):
  """Rule 2: a set that deliberately stays low must be explainable after the fact."""
  import openpilot.selfdrive.controls.lib.speedadjust_pnw.speedadjust_controller as m
  events = []
  monkeypatch.setattr(m.cloudlog, "event", lambda name, **kw: events.append((name, kw)))
  c = _stock_ctrl(mode=2, sl_ref=V60, ratio=V75 / V60, sl=V45)
  _settle_pub(c, V75, V45)
  c._sl = V60
  _release(c)
  assert any(n == "speedadjust_no_restore" and kw.get("reason") == "limitDrop" for n, kw in events)
  c._sa_pub_t -= 1.0                                     # the status publish is throttled at 5 Hz on the real clock
  _tick(c)
  status = [p for k, p in c.mem_params.calls if k == "SpeedAdjustStatus"]
  assert status and status[-1].get("noRst") == "limitDrop" and "epLim" in status[-1]


def test_every_published_status_key_is_forwarded_into_ces_events():
  """satele2pnw lesson: a key published to /dev/shm but not cherry-picked by ces_pnw evaporates. The first
  version of this test grepped ces_pnw's source for the two new key names, which a comment would satisfy
  (Fable review). This compares the keys the publisher ACTUALLY emits with the forwarding list."""
  from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import SA_TELE_KEYS
  c = _stock_ctrl(mode=2, sl_ref=V60, ratio=V75 / V60, sl=V45)
  _settle_pub(c, V75, V45)
  c._sa_pub_t -= 1.0
  _tick(c)
  published = [p for k, p in c.mem_params.calls if k == "SpeedAdjustStatus"]
  assert published, "no status was published -- the test proves nothing"
  missing = set(published[-1]) - set(SA_TELE_KEYS)
  assert not missing, f"published but never forwarded into ces_events: {sorted(missing)}"


def test_the_no_restore_reason_resets_with_each_episode():
  """Fable: `noRst` stayed 'limitDrop' through the next police-only episode's whole cap phase."""
  c = _stock_ctrl(mode=2, sl_ref=V60, ratio=V75 / V60, sl=V45)
  _settle_pub(c, V75, V45)
  c._sl = V60
  _release(c)
  assert c._no_restore_why == "limitDrop"
  c._police = {"state": "alert", "dist_mi": 0.4}
  _settle_pub(c, V75, V60)
  assert c._no_restore_why is None, "a stale reason from the previous episode"


def test_a_release_with_no_latched_ceiling_says_so():
  """Fable: the intervention path opened no restore but reported None, which means 'it did / n.a.'."""
  c = _stock_ctrl(mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  _settle_pub(c, V75, V60)
  c._pub_ceiling = None                                  # as if the driver intervened at engage
  c._police = {"state": "clear"}
  _release(c)
  assert _published_incs(c) == []
  assert c._no_restore_why == "noCeiling"


def test_police_starting_during_a_limit_drop_release_debounce_is_the_SAME_episode():
  """Gemini review framed this as the flag 'leaking into a subsequent police episode'. It is not a new
  episode: during the release debounce the cap has not released, so the police cap continues the SAME
  episode, with the ceiling latched when the LIMIT DROP engaged. Restoring that ceiling when police
  clears would also give back the limit-drop trim -- exactly what the directive forbids. Pinned so the
  behaviour cannot change by accident."""
  c = _stock_ctrl(mode=2, sl_ref=V60, ratio=V75 / V60, sl=V45)
  _settle_pub(c, V75, V45)
  c._sl = V60                                            # limit rises: the limit-drop source clears
  _tick(c)                                               # ...now inside the release debounce
  assert c._release_t is not None and c._cap_out is not None
  c._police = {"state": "alert", "dist_mi": 0.4}          # police arrives before the cap released
  for _ in range(10):
    _tick(c)
  assert c._ep_limit_drop is True, "the episode forgot it began as a limit drop"
  c._police = {"state": "clear"}
  _release(c)
  for _ in range(10):
    _tick(c)
  assert _published_incs(c) == []


def test_a_limit_drop_SHADOWED_by_a_lower_police_cap_still_marks_the_episode():
  """Gemini suggested marking the episode only when the limit-drop cap actually BINDS. Rejected: with a
  police cap below it the limit drop 'does nothing' this episode, but the limit did drop, and restoring
  the pre-episode set afterwards would put the truck above the new, lower limit. A one-tick map flicker
  cannot trigger this at all -- SL_DROP_CONFIRM_S requires a lower limit to persist 2 s first."""
  c = _stock_ctrl(mode=2, sl=V60, sl_ref=V60, ratio=V75 / V60, police={"state": "alert", "dist_mi": 0.4})
  _settle_pub(c, V75, V60)                               # police cap ~65
  c._sl = 50 * MPH                                       # a real 60 -> 50 drop (>5%): trim 62.5 mph, police cap 55
  c._sl_valid_t = time.monotonic() + 1e6
  for _ in range(10):
    _tick(c)
  lc = c._limit_drop_cap()
  assert lc is not None and lc > c._cap_out, "the scenario is not shadowed -- the test proves nothing"
  assert c._ep_limit_drop is True


# ---- sazoneset2pnw: a limit drop is a ONE-SHOT zone set, then speedadjust forgets it ----------------
# Driver directive 2026-09-13: entering a lower zone sets "the same percentage above the speed limit as I
# was driving before ... at that point there should be no memory anymore ... then I am just driving that
# speed." These drive cap() with a stock set that FOLLOWS the published SpeedAdjustTarget one 1 mph tap per
# 0.5 s tick, so "the truck's own reported set reached the zone target" is exercised the way the car does it.

def _zone_ctrl(stock_mph=75, sl_ref=V60, sl=V45, mode=2, op_long=False):
  c = _stock_ctrl(mode=mode, sl_ref=sl_ref, ratio=(stock_mph * MPH) / sl_ref, sl=sl) if not op_long else \
      _ctrl(op_long=True, mode=mode, sl_ref=sl_ref, ratio=(stock_mph * MPH) / sl_ref, sl=sl)
  if op_long:
    c.mem_params = _FakeMemParams()
  return c


def _drive(c, stock, ticks, follow=True, cruise_enabled=True, gas=False, speed_readable=True):
  """Tick cap() with the truck's set = `stock`; the fake executor follows a dec target by 1 mph per tick."""
  if c._last_t is None:                                 # first call initialises the slew/publish clocks (as _settle_pub)
    c.cap(_sm(speed=stock if speed_readable else 0.0, cruise_enabled=cruise_enabled, gas=gas), stock, stock, stock, True)
  for _ in range(ticks):
    sm = _sm(speed=stock if speed_readable else 0.0, cruise_enabled=cruise_enabled, gas=gas)
    _tick(c, v_cruise=stock, v_ego=stock, v_cruise_set=stock, sm=sm)
    last = c.mem_params.last
    if follow and isinstance(last, dict) and "target" in last and last.get("dir") != "inc" \
       and stock > last["target"] + ZONE_SET_DONE_TOL:
      stock -= 1 * MPH
  return stock


def test_zone_entry_sets_the_drivers_percentage_ONCE_then_goes_silent():
  """75 on a 60 (+25%) into a 45 zone -> 56.25; once the truck is there, speedadjust ends the episode, clears
  SpeedAdjustTarget and publishes nothing further. Before this change it kept re-publishing its dec every
  0.25 s for the whole zone -- which Fable measured starving every curve restore in arbitrate()."""
  c = _zone_ctrl()
  stock = _drive(c, 75 * MPH, 80)
  assert abs(stock - 45 * MPH * 1.25) <= 1 * MPH, f"zone set landed at {stock / MPH:.1f}, want ~56.25"
  assert c._cap_out is None and c._no_restore_why == "zoneSet"
  assert c.mem_params.last == {}, "SpeedAdjustTarget was not cleared after the zone set"
  assert abs(c._sl_ref - V45) < 1e-6, "the higher baseline was not forgotten"
  n = len(c.mem_params.target_calls)
  _drive(c, stock, 60)
  assert len(c.mem_params.target_calls) == n, "speedadjust kept publishing after the zone set"


def test_a_limit_RISE_after_the_zone_set_does_not_restore():
  c = _zone_ctrl()
  stock = _drive(c, 75 * MPH, 80)
  c._sl = V60
  after = _drive(c, stock, 60)
  assert after == stock and not any(isinstance(p, dict) and p.get("dir") == "inc" for p in c.mem_params.target_calls)


def test_a_further_drop_repeats_from_the_CURRENT_set():
  """No memory of the 75: the second zone is relative to the set the truck is now driving."""
  c = _zone_ctrl()
  stock = _drive(c, 75 * MPH, 80)
  c._sl = 35 * MPH
  stock2 = _drive(c, stock, 80)
  want = 35 * MPH * (stock / V45)
  assert abs(stock2 - want) <= 1 * MPH, f"second zone landed at {stock2 / MPH:.1f}, want {want / MPH:.1f}"
  assert c._no_restore_why == "zoneSet"


def test_an_unreadable_truck_set_never_completes_the_zone():
  """Absence of evidence: without the truck's reported set the zone is not 'reached' -- the old continuous
  cap stays, and nothing times out either (no actuatable time can be counted)."""
  c = _zone_ctrl()
  _drive(c, 75 * MPH, 200, follow=False, speed_readable=False)
  assert c._cap_out is not None and c._no_restore_why is None


def test_a_zone_that_can_never_be_reached_is_ABANDONED_and_logged(monkeypatch):
  """Taps not landing (executor blocked, a governor fault): bounded by ZONE_SET_TIMEOUT_S of actuatable
  time, then the zone is forgotten LOUDLY -- never a silent indefinite hold."""
  import openpilot.selfdrive.controls.lib.speedadjust_pnw.speedadjust_controller as m
  events, errors = [], []
  monkeypatch.setattr(m.cloudlog, "event", lambda name, **kw: events.append((name, kw)))
  monkeypatch.setattr(m.cloudlog, "error", lambda msg, *a, **k: errors.append(msg))
  c = _zone_ctrl()
  _drive(c, 75 * MPH, int(ZONE_SET_TIMEOUT_S / 0.5) + 10, follow=False)
  assert c._cap_out is None and c._no_restore_why == "zoneAbandoned"
  assert any(n == "speedadjust_zone_set_abandoned" for n, _ in events) and errors
  assert c.mem_params.last == {}


@pytest.mark.parametrize("kw", [dict(cruise_enabled=False), dict(gas=True)])
def test_the_timeout_only_counts_ACTUATABLE_time(kw):
  """ACC off or a pedal held: no press can land, so waiting must not burn the timeout."""
  c = _zone_ctrl()
  _drive(c, 75 * MPH, int(ZONE_SET_TIMEOUT_S / 0.5) + 40, follow=False, **kw)
  assert c._no_restore_why != "zoneAbandoned"
  assert c._zone_elapsed == 0.0


def test_a_police_cap_in_the_episode_keeps_the_OLD_hold():
  """Mixed episode: no zone completion, and (sanorestore2pnw) no restore either."""
  c = _zone_ctrl()
  c._police = {"state": "alert", "dist_mi": 0.2}
  stock = _drive(c, 75 * MPH, 80)
  assert c._cap_out is not None, "a mixed police + limit-drop episode was ended as a zone set"
  assert c._zone_target is None and stock < 75 * MPH
  # a still-capping check alone cannot see a wrongly ended zone: the police cap re-engages on the very next
  # tick (mutation A6 survived it). The zone must not have been completed or its baseline forgotten.
  assert c._no_restore_why != "zoneSet", "the mixed episode was completed as a zone set"
  assert abs(c._sl_ref - V60) < 1e-6, "the mixed episode forgot the higher baseline"


def test_the_tesla_op_long_path_never_ends_a_zone_and_publishes_nothing():
  """cap()'s return value on an op-long car keeps the continuous limit-drop cap for the whole zone."""
  c = _zone_ctrl(op_long=True)
  outs = [c.cap(_sm(speed=V75), V75, V75, V60, True)]
  for _ in range(120):
    outs.append(_tick(c, v_cruise=V75, v_ego=V60, v_cruise_set=V75, sm=_sm(speed=V75)))
  assert c._cap_out is not None and c._no_restore_why is None
  assert abs(outs[-1] - V45 * 1.25) < 0.05, "the op-long cap is no longer the continuous zone cap"
  assert c.mem_params.target_calls == []


def test_a_single_HIGHER_limit_read_cannot_manufacture_a_drop():
  """Fable's measured failure, through the REAL _read_speed_limit: set 66 in a 60 with a police cap, one
  65 read, back to 60. Before the rise hold the 65 re-anchored the baseline and the return to 60 confirmed
  as a 7.7% drop 2 s later: the set was trimmed and the police restore withheld. Under the zone model that
  trim would be permanent."""
  c = SpeedAdjustController(_CP(False), params=_Params())
  c._mode = 2
  mem = _FakeMemParams()
  reads = {"v": "26.8224"}                                # 60 mph
  mem.get = lambda k, return_default=False: reads["v"] if k == "MapSpeedLimit" else None
  c.mem_params = mem
  c._sl = 60 * MPH
  c._sl_ref = 60 * MPH
  c._sl_valid_t = time.monotonic()
  c._ratio = 66 / 60
  for v in ("29.0576", "26.8224", "26.8224", "26.8224"):  # one 65 read, then 60 again
    reads["v"] = v
    c._read_inputs()
    c._update_baseline(66 * MPH)
    c._sl_pending_t -= SL_DROP_CONFIRM_S + 0.1           # let any pending drop confirm, if one existed
  assert abs(c._sl - 60 * MPH) < 0.05 and abs(c._sl_ref - 60 * MPH) < 0.05, "the stray 65 was adopted"
  assert c._limit_drop_cap() is None, "a stray higher read manufactured a limit drop"


def test_the_zone_target_is_cleared_on_the_SAME_tick_the_zone_completes():
  """The executor treats a dec command as fresh for STALE_LIMIT_S; clear it at the completion tick itself,
  not a tick later via the idle path (mutation A9 survived a test that only looked much later)."""
  c = _zone_ctrl()
  stock = 75 * MPH
  c.cap(_sm(speed=stock), stock, stock, stock, True)
  for _ in range(200):
    _tick(c, v_cruise=stock, v_ego=stock, v_cruise_set=stock, sm=_sm(speed=stock))
    if c._no_restore_why == "zoneSet":
      assert c.mem_params.last == {}, "SpeedAdjustTarget still held a dec on the tick the zone completed"
      return
    last = c.mem_params.last
    if isinstance(last, dict) and "target" in last and stock > last["target"] + ZONE_SET_DONE_TOL:
      stock -= 1 * MPH
  raise AssertionError("the zone never completed -- the test proves nothing")


# ---- sazoneset2pnw, Fable review of the first version (DO NOT SHIP, two measured defects) -----------------------

class _IcbmMem(_FakeMemParams):
  """_FakeMemParams that also serves the IcbmTarget payload ces_pnw publishes (and the Ford executor reads)."""
  def __init__(self):
    super().__init__()
    self.icbm = {}

  def get(self, key, return_default=False):
    return self.icbm if key == "IcbmTarget" else None


def _icbm(target_mph, ceiling_mph, age_s=0.0, direction="dec"):
  d = {"target": target_mph * MPH, "ceiling": ceiling_mph * MPH, "ts": time.time() - age_s}  # noqa: TID251 -- the executor's wall-clock heartbeat
  if direction == "inc":
    d["dir"] = "inc"
  return d


def _curve_ctrl(sl=V60):
  """Set 75 on a 60 (+25%), uncapped, stock ACC, with the curve brain's mem-param readable."""
  c = _zone_ctrl(sl=sl)
  c.mem_params = _IcbmMem()
  c.cap(_sm(speed=V75), V75, V75, V75, True)
  return c


def _set_tick(c, stock, **sm_kw):
  c._icbm_read_t = -1e9                                  # the ~4 Hz IcbmTarget read, every tick here
  _tick(c, v_cruise=stock, v_ego=stock, v_cruise_set=stock, sm=_sm(speed=stock, **sm_kw))


def _tap_down(c, stock, n):
  for _ in range(n):
    stock -= 1 * MPH
    _set_tick(c, stock)
  return stock


def test_a_curve_brain_SET_minus_is_NOT_a_driver_override_and_the_ratio_keeps_the_pre_curve_set():
  """Defect 1(ii), measured: ICBM's SET- taps read as the driver's own set changes. Each one re-anchored the ratio
  to the tapped-down set, so a zone confirmed during the curve trimmed from 60/60 instead of 75/60 -- or not at
  all -- and the curve restore then handed back 75 in a 45 zone."""
  c = _curve_ctrl()
  c.mem_params.icbm = _icbm(35, 75)
  stock = V75
  for _ in range(15):
    stock -= 1 * MPH
    _set_tick(c, stock)
    assert c._ovr == "icbmTap", f"an ICBM tap read as {c._ovr!r}"
  assert c._icbm_hold and abs(c._ratio - 1.25) < 1e-9, f"ratio {c._ratio:.3f} followed the curve's taps"
  c._sl = V45                                            # the zone confirms mid-curve
  _set_tick(c, stock)
  assert c._zone_target is not None and abs(c._zone_target - V45 * 1.25) < 0.01, \
    f"zone target {c._zone_target} is not the driver's pre-curve percentage of 45"


def test_the_ratio_hold_survives_the_curve_brains_silent_gaps():
  """ICBM goes quiet (IcbmTarget {}) through its clear debounce and in-curve pauses while the set is still down."""
  c = _curve_ctrl()
  c.mem_params.icbm = _icbm(35, 75)
  stock = _tap_down(c, V75, 20)
  c.mem_params.icbm = {}
  for _ in range(10):
    _set_tick(c, stock)
  assert c._icbm_hold and abs(c._ratio - 1.25) < 1e-9, f"ratio {c._ratio:.3f} after ICBM went quiet"
  c._sl = 65 * MPH                                       # a limit RISE meanwhile rescales the same set, 75
  _set_tick(c, stock)
  assert abs(c._ratio * c._sl_ref - V75) < 0.01, f"reference set {c._ratio * c._sl_ref / MPH:.2f} mph, want 75"


@pytest.mark.parametrize("icbm", [None, "stale"])
def test_a_driver_SET_minus_with_no_live_curve_command_is_still_an_override(icbm):
  c = _curve_ctrl()
  if icbm == "stale":
    c.mem_params.icbm = _icbm(35, 75, age_s=SA_ICBM_FRESH_S + 0.5)
  _set_tick(c, V75 - 1 * MPH)
  assert c._ovr == "applied" and not c._icbm_hold


def test_a_curve_tap_landing_just_AFTER_the_curve_brain_went_quiet_is_still_its_own():
  """The truck reports the set with lag, so ICBM's last SET- can land after IcbmTarget is already {}."""
  c = _curve_ctrl()
  c.mem_params.icbm = _icbm(35, 75)
  stock = _tap_down(c, V75, 3)
  c.mem_params.icbm = {}
  _set_tick(c, stock - 1 * MPH)
  assert c._ovr == "icbmTap", f"an in-flight curve tap read as {c._ovr!r}"


def _restore_up(c, stock, n):
  for _ in range(n):
    stock += 1 * MPH
    _set_tick(c, stock)
  return stock


def test_a_curve_RESTORE_SET_plus_is_not_a_driver_override_and_the_ratio_keeps_the_pre_curve_set():
  """zonefollow2pnw (Fable, measured): ICBM's restore taps read as the driver raising the set, re-anchoring the ratio
  to the half-restored set -- a limit drop mid-restore then ended at 47 / 50 / 54 mph instead of 56."""
  c = _curve_ctrl()
  c.mem_params.icbm = _icbm(35, 75)
  stock = _tap_down(c, V75, 20)
  c.mem_params.icbm = _icbm(75, 75, direction="inc")
  for _ in range(8):
    stock += 1 * MPH
    _set_tick(c, stock)
    assert c._ovr == "icbmTap", f"a curve restore tap read as {c._ovr!r}"
  assert c._icbm_hold and abs(c._ratio - 1.25) < 1e-9, f"ratio {c._ratio:.3f} followed the restore"


def test_a_restore_tap_landing_just_AFTER_the_curve_brain_went_quiet_is_still_its_own():
  c = _curve_ctrl()
  c.mem_params.icbm = _icbm(35, 75)
  stock = _tap_down(c, V75, 10)
  c.mem_params.icbm = _icbm(75, 75, direction="inc")
  stock = _restore_up(c, stock, 2)
  c.mem_params.icbm = {}
  _set_tick(c, stock + 1 * MPH)
  assert c._ovr == "icbmTap"


@pytest.mark.parametrize("case", ["stale", "grace_expired"])
def test_a_driver_SET_plus_with_no_live_curve_restore_is_still_an_override(case):
  c = _curve_ctrl()
  c.mem_params.icbm = _icbm(35, 75)
  stock = _tap_down(c, V75, 5)
  c._icbm_dec_t -= SA_ACTUATION_GRACE_S + 0.1
  if case == "stale":
    c.mem_params.icbm = _icbm(75, 75, age_s=SA_ICBM_FRESH_S + 0.5, direction="inc")
  else:
    c.mem_params.icbm = _icbm(75, 75, direction="inc")
    _set_tick(c, stock)
    c.mem_params.icbm = {}
    c._icbm_inc_t -= SA_ACTUATION_GRACE_S + 0.1
  _set_tick(c, stock + 1 * MPH)
  assert c._ovr == "applied"


def test_a_driver_SET_plus_during_OUR_OWN_cap_is_still_an_override_even_with_a_curve_restore_published():
  """arbitrate() runs no inc while any dec is on the bus, so while speedadjust caps, a SET+ is the driver's -- e.g.
  dismissing a police slowdown. It must not be absorbed as a curve tap."""
  c = _zone_ctrl()                                        # 75 on a 60, limit now 45: speedadjust is capping
  c.mem_params = _IcbmMem()
  c.cap(_sm(speed=V75), V75, V75, V75, True)
  _set_tick(c, V75)
  assert c._cap_out is not None
  c.mem_params.icbm = _icbm(75, 75, direction="inc")
  _set_tick(c, V75)
  _set_tick(c, V75 + 1 * MPH)
  assert c._ovr == "applied", c._ovr


def test_each_speedadjust_instance_publishes_its_own_stable_id():
  """ICBM tells a plannerd restart from a zone count that simply did not move by this id."""
  a, b = _zone_ctrl(), None
  stock = _drive(a, V75, 3)
  ids = {p["inst"] for k, p in a.mem_params.calls if k == "SpeedAdjustStatus"}
  assert len(ids) == 1 and None not in ids, ids
  time.sleep(0.002)
  b = _zone_ctrl()
  _drive(b, stock, 3)
  assert {p["inst"] for k, p in b.mem_params.calls if k == "SpeedAdjustStatus"} != ids


def test_the_in_flight_grace_after_a_curve_dec_expires():
  c = _curve_ctrl()
  c.mem_params.icbm = _icbm(35, 75)
  stock = _tap_down(c, V75, 3)
  c.mem_params.icbm = {}
  c._icbm_dec_t -= SA_ACTUATION_GRACE_S + 0.1
  _set_tick(c, stock - 1 * MPH)
  assert c._ovr == "applied", "a SET- long after the curve brain stopped was not the driver's"


@pytest.mark.parametrize("end", ["gas", "acc_off", "driver_up"])
def test_the_ratio_hold_ends_when_the_set_is_the_drivers_again(end):
  c = _curve_ctrl()
  c.mem_params.icbm = _icbm(35, 75)
  stock = _tap_down(c, V75, 10)
  c.mem_params.icbm = {}
  for _ in range(3):
    _set_tick(c, stock)
  assert c._icbm_hold
  if end == "gas":
    _set_tick(c, stock, gas=True)
  elif end == "acc_off":
    _set_tick(c, stock, cruise_enabled=False)
  else:
    c._icbm_dec_t -= SA_ACTUATION_GRACE_S + 0.1
    _set_tick(c, stock + 1 * MPH)
    assert c._ovr == "applied"
  assert not c._icbm_hold, f"hold survived {end}"


def test_the_ratio_hold_ends_once_the_set_is_back_at_the_reference_with_the_curve_quiet():
  """A curve command that never needed a tap (target above the set, or already there) must not leave the hold on."""
  c = _curve_ctrl()
  c.mem_params.icbm = _icbm(74, 75)
  for _ in range(3):
    _set_tick(c, V75)
  assert c._icbm_hold
  c.mem_params.icbm = {}
  _set_tick(c, V75)
  assert not c._icbm_hold


def test_the_ratio_hold_does_not_end_while_the_set_is_still_down_and_the_curve_is_quiet():
  c = _curve_ctrl()
  c.mem_params.icbm = _icbm(35, 75)
  stock = _tap_down(c, V75, 10)
  c.mem_params.icbm = {}
  for _ in range(20):
    _set_tick(c, stock)
  assert c._icbm_hold


def test_the_tesla_never_reads_the_curve_brains_command():
  """op-long: byte-identical cap() -- no IcbmTarget read at all (even one serving a fresh dec), every set change is
  the driver's, no hold."""
  class _Spy(_IcbmMem):
    def __init__(self):
      super().__init__()
      self.reads = []

    def get(self, key, return_default=False):
      self.reads.append(key)
      return super().get(key, return_default)
  c = _ctrl(op_long=True, mode=2, sl_ref=V60, ratio=1.25, sl=V60)
  c.mem_params = _Spy()
  c.mem_params.icbm = _icbm(35, 75)
  c.cap(_sm(speed=V75), V75, V75, V75, True)
  c._icbm_read_t = -1e9
  _tick(c, v_cruise=V75 - MPH, v_ego=V75, v_cruise_set=V75 - MPH, sm=_sm(speed=V75 - MPH))
  assert "IcbmTarget" not in c.mem_params.reads
  assert c._ovr == "applied" and not c._icbm_hold


def test_an_unreadable_curve_command_is_LOGGED_once(monkeypatch):
  import openpilot.selfdrive.controls.lib.speedadjust_pnw.speedadjust_controller as m
  logged = []
  monkeypatch.setattr(m.cloudlog, "exception", lambda msg, *a, **k: logged.append(msg))
  class _Bad(_FakeMemParams):
    def get(self, key, return_default=False):
      if key == "IcbmTarget":
        raise OSError("shm gone")
      return None
  c = _zone_ctrl(sl=V60)
  c.mem_params = _Bad()
  c.cap(_sm(speed=V75), V75, V75, V75, True)
  for _ in range(5):
    c._icbm_read_t = -1e9
    _tick(c, v_cruise=V75, v_ego=V75, v_cruise_set=V75, sm=_sm(speed=V75))
  assert len([x for x in logged if "IcbmTarget" in x]) == 1


def test_zone_episodes_are_counted_and_the_last_target_outlives_the_zone():
  """ICBM bounds a restore by a zone that BEGAN during its episode, including one that opened and completed
  between two of its ~1 Hz status reads -- which needs a count, not just the in-progress target."""
  c = _zone_ctrl()
  assert c._zone_n == 0 and c._zone_last is None
  stock = _drive(c, 75 * MPH, 80)
  assert c._no_restore_why == "zoneSet" and c._zone_n == 1
  assert c._zone_target is None and abs(c._zone_last - V45 * 1.25) < 0.01
  c._sl = 35 * MPH
  stock = _drive(c, stock, 80)
  assert c._zone_n == 2, "a second zone episode was not counted"
  c._sa_pub_t = -1e9
  _tick(c, v_cruise=stock, v_ego=stock, v_cruise_set=stock, sm=_sm(speed=stock))
  st = [p for k, p in c.mem_params.calls if k == "SpeedAdjustStatus"][-1]
  assert st["zoneN"] == 2 and st["zoneLast"] is not None and "icbmHold" in st


def test_an_unknown_read_does_NOT_cancel_a_pending_rise():
  """Defect 2, measured: resetting a pending rise on an unknown read made a rise need 4 consecutive valid 1 Hz
  reads; under the map's documented valid<->unknown flicker the Tesla held a 35-zone trim on a 60 road forever."""
  c = _sl_reader(V45, V60)
  assert c._read_speed_limit() == V45 and c._sl_rise_pending == V60
  t0 = c._sl_rise_pending_t
  c.mem_params.value = None
  assert c._read_speed_limit() == V45
  assert c._sl_rise_pending == V60 and c._sl_rise_pending_t == t0, "an unknown read cancelled the pending rise"
  c.mem_params.value = V60
  c._sl_rise_pending_t -= SL_RISE_CONFIRM_S + 0.1
  assert c._read_speed_limit() == V60


def test_a_stray_higher_read_is_still_cancelled_by_the_real_limit_across_a_dropout():
  c = _sl_reader(V60, 65 * MPH)
  c._read_speed_limit()
  c.mem_params.value = None
  c._read_speed_limit()
  c.mem_params.value = V60
  assert c._read_speed_limit() == V60 and c._sl_rise_pending == 0.0


class _FlickerClock:
  t = 1000.0

  @classmethod
  def monotonic(cls):
    return cls.t

  @classmethod
  def time(cls):
    return cls.t


@pytest.mark.parametrize("pattern", [[True], [True, False], [True, True, False], [True, True, True, False],
                                     [True, True, True, True, False], [True, False, False]])
def test_the_tesla_releases_a_limit_rise_promptly_under_map_flicker(monkeypatch, pattern):
  """Fable's table, through the real reader and release path at 20 Hz: op-long, set 70 on a 60, limit 35 from 5 s
  (cap 40.8), back to 60 at 20 s; after that the 1 Hz read is valid or unknown per `pattern`. Base (no rise hold):
  3.1 / 4.1 / 3.1 / 3.1 / 3.1 / 5.2 s. The first version of this change: 6.2 / NEVER / NEVER / 66 / 10.4 / NEVER."""
  import openpilot.selfdrive.controls.lib.speedadjust_pnw.speedadjust_controller as m
  _FlickerClock.t = 1000.0
  monkeypatch.setattr(m, "time", _FlickerClock)
  c = SpeedAdjustController(_CP(True), params=type("P", (), {"get": lambda self, k, return_default=True: "2"})())

  class _Map(_FakeMemParams):
    def get(self, key, return_default=False):
      if key != "MapSpeedLimit":
        return None
      t = _FlickerClock.t - 1000.0
      v = 60 if t < 5 else 35 if t < 20 else (60 if pattern[int(t - 20.0) % len(pattern)] else 0)
      return str(v * MPH) if v else ""
  c.mem_params = _Map()
  v70, released = 70 * MPH, None
  while _FlickerClock.t - 1000.0 < 60.0:
    _FlickerClock.t += 0.05
    c.cap(_sm(speed=v70), v70, v70, 25.0, True)
    t = _FlickerClock.t - 1000.0
    if 15.0 < t < 20.0:
      assert c._cap_out is not None, "the test proves nothing: the 35 never capped"
    if t > 20.0 and released is None and c._cap_out is None:
      released = t - 20.0
  assert released is not None and released <= 6.5, f"released {released} s after the sign"


def test_the_rise_credit_is_used_only_by_the_release_on_the_same_tick(monkeypatch):
  """The persisted rise counts toward RELEASE_S only for the release it causes. A rise adopted while a POLICE cap
  still binds must not leave a stale credit that lets the later police release skip its whole debounce."""
  import openpilot.selfdrive.controls.lib.speedadjust_pnw.speedadjust_controller as m
  _FlickerClock.t = 1000.0
  monkeypatch.setattr(m, "time", _FlickerClock)
  police = {"state": "alert", "dist_mi": 0.1}
  c = SpeedAdjustController(_CP(True), params=type("P", (), {"get": lambda self, k, return_default=True: "2"})())

  class _Map(_FakeMemParams):
    def get(self, key, return_default=False):
      t = _FlickerClock.t - 1000.0
      if key == "MapSpeedLimit":
        return str((60 if t < 5 else 35 if t < 20 else 60) * MPH)
      if key == "LocationServices":
        return json.dumps({"police": police}) if t < 40 else "{}"
      return None
  c.mem_params = _Map()
  v70 = 70 * MPH
  first_clear = None
  while _FlickerClock.t - 1000.0 < 60.0:
    _FlickerClock.t += 0.05
    c.cap(_sm(speed=v70), v70, v70, 25.0, True)
    t = _FlickerClock.t - 1000.0
    if 30.0 < t < 39.0:
      assert c._cap_out is not None and c._sl == 60 * MPH, "the test proves nothing: police must still cap after the rise"
    if t > 40.0 and first_clear is None and c._cap_out is None:
      first_clear = t - 40.0
  assert first_clear is not None and first_clear >= RELEASE_S, f"police cap released {first_clear} s after it cleared"


def test_a_single_low_read_after_a_dropout_does_not_act():
  # limitdropexact2pnw (Fable F1): 55 in a 60, the limit goes unknown past SL_HOLD_S (self._sl = 0, the baseline
  # survives), then ONE bogus 25 read. It must wait for the drop confirm like any other drop.
  c = _sl_reader(0.0, 25 * MPH)
  c._sl_ref = V60
  assert c._read_speed_limit() == 0.0, "an unconfirmed first-after-dropout drop must not be returned"
  c.mem_params.value = V60
  assert c._read_speed_limit() == V60
  assert c._sl_pending == 0.0


def test_a_real_drop_after_a_dropout_is_accepted_after_confirmation():
  c = _sl_reader(0.0, 25 * MPH)
  c._sl_ref = V60
  assert c._read_speed_limit() == 0.0
  c._sl_pending_t -= (SL_DROP_CONFIRM_S + 0.1)
  assert c._read_speed_limit() == 25 * MPH


def test_first_read_after_a_dropout_at_the_baseline_is_taken_at_once():
  c = _sl_reader(0.0, V60)
  c._sl_ref = V60
  assert c._read_speed_limit() == V60


def test_limit_drop_41_in_a_45_on_stock_acc_publishes_exactly_25():
  # limitdropexact2pnw (Fable F2): the 2026-09-24 bug was on the Lightning (stock ACC) -- pin the published target.
  V41 = 41 * MPH
  V25 = 25 * MPH
  c = _stock_ctrl(mode=2, sl_ref=V45, ratio=V41 / V45, sl=V25)
  _settle_pub(c, V41, V41)
  payload = c.mem_params.last
  assert payload is not None, "the zone must publish a SET- target on stock ACC"
  assert "dir" not in payload
  assert abs(payload["target"] - V25) < 0.01, f"want exactly 25 mph, got {payload['target'] / MPH:.2f}"
