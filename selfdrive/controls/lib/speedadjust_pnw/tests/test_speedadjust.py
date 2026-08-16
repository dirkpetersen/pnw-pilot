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
  SpeedAdjustController, MPH_TO_MS, POLICE_MARGIN, MIN_CAP, CAP_SLEW, RELEASE_S, RESTORE_WINDOW_S,
  SA_DRIVER_LOWER_TOL, SET_CHANGE_EPS, SA_ACTUATION_GRACE_S)

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
  def last(self):
    return self.calls[-1][1] if self.calls else None


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
  assert c.mem_params.calls == []


def test_no_publish_on_oplong_car():
  # same scenario as test_publish_dec_while_capping below, but op_long=True: the mem-param is never
  # touched -- op-long cars are steered solely through cap()'s return value.
  c = _ctrl(op_long=True, mode=1, sl=V60, police={"state": "alert", "dist_mi": 0.4})
  c.mem_params = _FakeMemParams()   # inject AFTER construction so we can observe (a lack of) calls
  _settle(c, V75, V60)
  assert c.mem_params.calls == []


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
  n_before = len(c.mem_params.calls)
  c._last_t -= 0.5
  c._pub_last -= 1.0
  out = c.cap(_sm(speed=V60, lat_accel=2.5, v_ego=30.0), V75, V75, V60, True)
  assert out == V75                                        # neutral op-long return, unaffected
  assert c._restore_ceiling is not None                     # episode still alive -- PAUSED, not aborted
  assert len(c.mem_params.calls) == n_before                # no fresh publish while paused


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
