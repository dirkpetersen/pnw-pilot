"""icbmratchet2pnw — episode target ratchet robustness (pure IcbmEpisode, unit-tested).

Field event (2026-08-10, Ford F-150 Lightning): the truck slowed 50 -> 15 mph on a curve where the
computed ICBM target was ~31 mph (icbmT approx 13.86 m/s). ROOT CAUSE: IcbmEpisode._min_target only
ever ratcheted DOWN for the life of a cap episode (min(_min_target, cap_target) every ~4 Hz tick), so
a single transient low tick -- e.g. a vision-curvature reading distorted for one tick by momentary
off-line/understeer driving -- got treated exactly like a genuine sustained curve reading and
permanently floored the episode at that outlier value. Caps are DEC-only by design, so the very next
tick's recomputed (correct) higher target could never undo it -- only a full curve-clear + guarded
RESTORE recovers, and that goes all the way back to the driver's ORIGINAL ceiling, never to the
correct intermediate curve speed.

FIX (IcbmEpisode._ratchet_confirm + three deliberately-separate trackers -- see IcbmEpisode.__init__
for the full rationale): an outlier-sized single-tick drop below the CONFIRMED baseline must persist
ICBM_RATCHET_CONFIRM_S before it is adopted into _min_target (the confirmed baseline future ticks are
outlier-tested against). _committed_target tracks what is actually being published (so a genuinely
consistent-but-not-yet-confirmed reading doesn't flicker back to the driver's set while it confirms).
_min_published is a plain running min of every published value (mirroring the ORIGINAL pre-fix
_min_target semantics) and feeds the restore-eligibility driver-lower-guard, which must know about
everything the executor may actually have tapped toward, confirmed or not.

Two full adversarial review rounds (Fable + Gemini, independently) found and closed additional gaps
beyond the initial fix -- see the dedicated test sections below for each.
"""
import math

from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import (IcbmEpisode, ICBM_RATCHET_CONFIRM_S,
                                                              ICBM_RATCHET_OUTLIER_DROP_MS,
                                                              ICBM_RESTORE_DELAY_S,
                                                              ICBM_RESTORE_DELAY_FAST_S)

MPH = 0.44704
T0 = 100.0
TICK_S = 0.25   # matches the real ~4 Hz _icbm_step publish cadence


def _confirm(ep, t, target_mph, v_set_mph, stock_mph, max_ticks=12):
  """Drive ticks at the real cadence, feeding a constant reading, until _min_target confirms at it.
  Returns the final (t, out). Fails (via the caller's own assertions) if it never confirms."""
  out = None
  for _ in range(max_ticks):
    t += TICK_S
    out = ep.step(t, target_mph * MPH, v_set_mph * MPH, stock_mph * MPH, True, False)
    if ep._min_target is not None and math.isclose(ep._min_target, target_mph * MPH, rel_tol=1e-6):
      return t, out
  return t, out


def test_single_outlier_tick_does_not_latch_field_event():
  """THE 2026-08-10 field case, replayed with the real numbers: episode engages at 31 mph (driver's
  set 50), ONE tick's candidate plunges to 15 mph (a distorted vision-curvature reading), then the
  curve reading is SUSTAINED at 31 mph for many ticks (the vision candidate recovers, exactly what
  the field telemetry showed). The published target must NEVER touch 15 mph at any point, and
  _min_target must eventually confirm at the TRUE 31 mph, never at the 15 mph outlier."""
  ep = IcbmEpisode()
  t = T0
  out = ep.step(t, 31 * MPH, 50 * MPH, 50 * MPH, True, False)     # engage
  assert math.isclose(out[0], 31 * MPH)
  t += TICK_S
  out = ep.step(t, 15 * MPH, 50 * MPH, 31 * MPH, True, False)     # ONE anomalous low tick
  assert math.isclose(out[0], 31 * MPH), "a single outlier tick must not be published"
  # sustained recovery: the true 31 mph reading persists for many ticks (as the field telemetry did)
  never_hit_outlier = True
  for _ in range(10):
    t += TICK_S
    out = ep.step(t, 31 * MPH, 50 * MPH, 31 * MPH, True, False)
    if math.isclose(out[0], 15 * MPH, abs_tol=0.5):
      never_hit_outlier = False
  assert never_hit_outlier, "the published target must never lurch to the 15 mph outlier"
  assert math.isclose(out[0], 31 * MPH)
  assert math.isclose(ep._min_target, 31 * MPH), "must confirm at the TRUE value, never the outlier"


def test_sustained_low_target_does_brake():
  """The flip side: a GENUINE tightening curve where the candidate drops to 15 mph and STAYS there
  across consecutive ticks must still fully brake -- _min_target adopts the sustained low value
  within the confirmation window, exactly the drop the field event needed but a single glitch tick
  must not trigger."""
  ep = IcbmEpisode()
  ep.step(T0, 31 * MPH, 50 * MPH, 50 * MPH, True, False)
  t, out = _confirm(ep, T0, 15.0, 50.0, 31.0)
  assert math.isclose(out[0], 15 * MPH, rel_tol=1e-6), f"sustained low target was never adopted ({out})"
  assert math.isclose(ep._min_target, 15 * MPH)
  assert t - T0 >= ICBM_RATCHET_CONFIRM_S   # adoption never happens faster than the confirm window


def test_small_gradual_decrease_not_gated_once_confirmed():
  """Ordinary continuous refinement (small tick-to-tick drops relative to an ALREADY-CONFIRMED
  baseline, e.g. a curve estimate tightening slightly as distance closes) is never treated as an
  outlier -- no confirmation delay, ratchets immediately. (Right at engage, before anything is
  confirmed, even a small-looking drop from the driver's set still goes through one confirmation
  cycle -- see test_engage_itself_requires_confirmation_against_v_set -- this test isolates the
  steady-state behavior AFTER a baseline exists.)"""
  ep = IcbmEpisode()
  t = T0
  ep.step(t, 40 * MPH, 60 * MPH, 60 * MPH, True, False)
  t, out = _confirm(ep, t, 40.0, 60.0, 40.0)                       # let the engage value confirm
  assert math.isclose(ep._min_target, 40 * MPH)

  step_mph = (ICBM_RATCHET_OUTLIER_DROP_MS / MPH) * 0.5            # half the outlier threshold
  target = 40.0
  for _ in range(4):
    t += TICK_S
    target -= step_mph
    out = ep.step(t, target * MPH, 60 * MPH, target * MPH, True, False)
    assert math.isclose(out[0], target * MPH), "small gradual drops must publish immediately"
    assert math.isclose(ep._min_target, target * MPH)
  assert ep._pending_low_target is None


def test_rising_target_not_gated_once_confirmed():
  """A rise relative to an ALREADY-CONFIRMED baseline is never an 'outlier drop' -- publishes
  immediately, _min_target (the low-water mark used for restore eligibility) is untouched by a
  rise. (Immediately after engage, before anything is confirmed, even a rise still needs its own
  brief confirmation -- see test_engage_itself_requires_confirmation_against_v_set -- this test
  isolates the steady-state behavior.)"""
  ep = IcbmEpisode()
  t = T0
  ep.step(t, 30 * MPH, 60 * MPH, 60 * MPH, True, False)
  t, _ = _confirm(ep, t, 30.0, 60.0, 30.0)
  assert math.isclose(ep._min_target, 30 * MPH)
  t += TICK_S
  out = ep.step(t, 45 * MPH, 60 * MPH, 30 * MPH, True, False)
  assert math.isclose(out[0], 45 * MPH)
  assert math.isclose(ep._min_target, 30 * MPH)   # never raised by a rising candidate


def test_engage_itself_requires_confirmation_against_v_set():
  """Gemini review catch, round 2: the engage tick's OWN reading is still published/acted on
  immediately (bounded, single-tick impact -- preserves fast real-curve response), but must NOT
  become the confirmed _min_target/baseline until it (or a consistent value) persists the
  confirmation window against the driver's v_set -- otherwise a bad first tick permanently taints
  the baseline future ticks are outlier-tested against (see the dedicated exploit tests below).
  Right after engage, _min_target is still None; it only confirms once the reading (or a
  value within the outlier band of it) has persisted ICBM_RATCHET_CONFIRM_S."""
  ep = IcbmEpisode()
  out = ep.step(T0, 31 * MPH, 50 * MPH, 50 * MPH, True, False)
  assert math.isclose(out[0], 31 * MPH), "engage still publishes its own reading immediately"
  assert ep._min_target is None, "but does not yet CONFIRM it as the baseline"
  assert math.isclose(ep._committed_target, 31 * MPH)
  t, out = _confirm(ep, T0, 31.0, 50.0, 31.0)
  assert math.isclose(ep._min_target, 31 * MPH)


def test_outlier_pending_does_not_corrupt_restore_eligibility():
  """The ratchet mechanism must not leak a false-low floor into the restore-eligibility gate
  (ICBM_DRIVER_LOWER_TOL check, via _min_published): after an outlier tick is correctly rejected and
  the true 31 mph value is confirmed, a subsequent clear + restore must target the ORIGINAL ceiling
  -- the eligibility check must see 31 mph as the floor, never the never-adopted 15 mph outlier."""
  ep = IcbmEpisode()
  t = T0
  ep.step(t, 31 * MPH, 50 * MPH, 50 * MPH, True, False)            # ceiling 50
  t += TICK_S
  ep.step(t, 15 * MPH, 50 * MPH, 31 * MPH, True, False)            # outlier tick, rejected
  t, _ = _confirm(ep, t, 31.0, 50.0, 31.0)                         # sustained recovery confirms
  assert math.isclose(ep._min_target, 31 * MPH)
  assert math.isclose(ep._min_published, 31 * MPH), \
      "the outlier was never published, so _min_published must not reflect it either"

  # curve clears, stock set sits at 31 (never actually reached 15) -> sustained clear -> restore
  t += TICK_S
  ep.step(t, None, 31 * MPH, 31 * MPH, True, False)                # clear tick: debounce starts
  tgt, d = ep.step(t + ICBM_RESTORE_DELAY_S + 0.1, None, 31 * MPH, 31 * MPH, True, False)
  assert d == "inc" and math.isclose(tgt, 50 * MPH) and ep.phase == "restore"


def test_pending_outlier_takes_worst_seen_when_confirmed():
  """While a drop is pending, the WORST (lowest) value seen among CONSISTENT readings is what gets
  adopted -- conservative by design (never less braking than the sustained readings warranted), not
  whatever the LATEST (confirming) tick happened to report. Readings here (15, 14, 16, 16 mph) are
  all mutually close (within the outlier band of each other), so none triggers the deviation-restart
  in _ratchet_confirm -- see test_extreme_outlier_inside_pending_window_does_not_hijack_it below for
  the case where a reading is NOT close to the others."""
  ep = IcbmEpisode()
  ep.step(T0, 31 * MPH, 50 * MPH, 50 * MPH, True, False)
  readings = [15.0, 14.0, 16.0, 16.0]
  t = T0
  out = None
  for r in readings:
    t += TICK_S
    out = ep.step(t, r * MPH, 50 * MPH, 31 * MPH, True, False)
  assert math.isclose(out[0], 14 * MPH, abs_tol=0.05), out
  assert math.isclose(ep._min_target, 14 * MPH, abs_tol=0.05)


# ---- Fable + Gemini adversarial-review catches, round 1 (2026-08-11) -------------------------------
def test_pending_state_does_not_survive_a_cap_dropout_field_replay():
  """Fable review catch: _ratchet_confirm measures WALL TIME, not contiguous low readings. Without
  clearing the pending candidate when the cap goes silent, a glitch tick sighted just before a brief
  detection dropout could sit stale through the gap and then get instantly "confirmed" the moment
  ANY new, unrelated, merely outlier-sized-but-LEGITIMATE candidate rebinds -- recreating the exact
  field bug through a different path. Replay: confirmed at 31 mph -> one glitch tick to 15 mph ->
  three ticks of cap_target=None (a detection dropout, well inside the clear debounce) -> curve
  rebinds at a legitimate 20 mph (an outlier-sized drop from 31, same as the glitch was, so the old
  bug's stale-confirm path is squarely exercised). The 20 mph must never instantly confirm at the
  stale 15 mph via the old pending state -- it must start its own fresh confirmation."""
  ep = IcbmEpisode()
  t = T0
  ep.step(t, 31 * MPH, 50 * MPH, 50 * MPH, True, False)
  t, _ = _confirm(ep, t, 31.0, 50.0, 31.0)                         # confirmed 31 mph baseline
  ep._engage_t0 -= (ICBM_RATCHET_CONFIRM_S + 0.1)                  # long-since-confirmed engage
  t += TICK_S
  ep.step(t, 15 * MPH, 50 * MPH, 31 * MPH, True, False)            # glitch tick, pending
  assert ep._pending_low_target is not None and ep._pending_low_t0 is not None
  for _ in range(3):                                               # dropout: cap silent
    t += TICK_S
    out = ep.step(t, None, 50 * MPH, 31 * MPH, True, False)
    assert out == (None, None)
  assert ep._pending_low_target is None and ep._pending_low_t0 is None, \
      "pending outlier state must be voided by a cap dropout, not carried across it"
  t += TICK_S
  out = ep.step(t, 20 * MPH, 50 * MPH, 31 * MPH, True, False)      # legitimate rebind
  assert not math.isclose(out[0], 15 * MPH), "must not instantly confirm the stale glitch value"
  assert math.isclose(out[0], 31 * MPH), "20 mph must itself now be pending, not yet published"
  assert math.isclose(ep._min_target, 31 * MPH)


def test_fresh_engage_then_immediate_clear_does_not_survive_for_a_later_rebind():
  """Gemini review catch (round 1): an engage that goes silent again before ICBM_RATCHET_CONFIRM_S
  elapses is treated as never having proven itself -- the whole episode is wiped, so a later,
  unrelated, legitimate curve starts a completely FRESH episode instead of resuming the tainted
  one."""
  ep = IcbmEpisode()
  # tick 1: spurious 15 mph engage while driver's set is 50 (nowhere near a real curve yet)
  out = ep.step(T0, 15 * MPH, 50 * MPH, 50 * MPH, True, False)
  assert out == (15 * MPH, "dec") and ep.phase == "cap"
  # tick 2, well within ICBM_RATCHET_CONFIRM_S: the glitch clears -- engage never proved itself
  out = ep.step(T0 + TICK_S, None, 50 * MPH, 50 * MPH, True, False)
  assert out == (None, None)
  assert ep.phase == "idle" and ep.ceiling is None and ep._min_target is None, \
      "an engage that clears before confirming must wipe the episode entirely"
  # tick 3: a later, legitimate, DIFFERENT curve binds -- must start a completely fresh episode,
  # never resume/relatch anything from the wiped 15 mph engage
  out = ep.step(T0 + 2 * TICK_S, 45 * MPH, 50 * MPH, 50 * MPH, True, False)
  assert out == (45 * MPH, "dec") and math.isclose(ep.ceiling, 50 * MPH)


def test_engage_confirmed_by_persistence_is_not_wiped_on_a_later_clear():
  """The flip side of the fresh-engage guard: a REAL curve that stays bound for longer than
  ICBM_RATCHET_CONFIRM_S before eventually clearing must NOT be treated as unconfirmed -- the
  episode's ceiling survives to a normal clear-debounce/restore cycle exactly as before this
  review round."""
  ep = IcbmEpisode()
  ep.step(T0, 31 * MPH, 50 * MPH, 50 * MPH, True, False)
  t = T0 + ICBM_RATCHET_CONFIRM_S + 0.2               # curve stayed bound past the confirm window
  ep.step(t, 31 * MPH, 50 * MPH, 31 * MPH, True, False)
  t += TICK_S
  out = ep.step(t, None, 50 * MPH, 31 * MPH, True, False)    # now it clears
  assert out == (None, None) and ep.phase == "cap" and ep.ceiling is not None, \
      "a confirmed (long-bound) engage must survive a clear into the normal debounce"


def test_extreme_outlier_inside_pending_window_does_not_hijack_it():
  """Gemini review catch (round 1, "worst-seen hijacking"): a legitimate drop opens a pending window
  (real curve tightening 50 -> 40 mph); mid-window, ONE extreme single-tick glitch (10 mph) arrives,
  then the reading recovers back to the true 40 mph for the rest of the window. Without a consistency
  check, unconditional min() adopts the drive-by glitch (10) just because it happened to land inside
  a window some OTHER, legitimate reading opened. The fix restarts the confirmation window whenever a
  reading deviates from the currently-tracked pending candidate by more than the outlier band, so the
  10 mph blip can never hijack the 40 mph window -- the adopted value must be 40, never 10."""
  ep = IcbmEpisode()
  ep.step(T0, 50 * MPH, 60 * MPH, 60 * MPH, True, False)             # confirmed at 50
  t = T0
  t += TICK_S
  out = ep.step(t, 40 * MPH, 60 * MPH, 50 * MPH, True, False)        # legit drop opens the window
  assert math.isclose(out[0], 50 * MPH)                              # still pending, holds at 50
  t += TICK_S
  out = ep.step(t, 10 * MPH, 60 * MPH, 50 * MPH, True, False)        # one extreme glitch mid-window
  assert not math.isclose(out[0], 10 * MPH)
  t, out = _confirm(ep, t, 40.0, 60.0, 50.0)
  assert math.isclose(out[0], 40 * MPH, abs_tol=0.05), out
  assert math.isclose(ep._min_target, 40 * MPH, abs_tol=0.05), \
      "the confirmed value must be the legitimate 40 mph reading, never the 10 mph outlier"


def test_unconfirmed_cap_dist_does_not_corrupt_apex_passage():
  """Fable review catch: an unconfirmed (still-pending) outlier's distance must not feed the
  apex-passage / early-restore decision -- only a CONFIRMED reading's distance should. Engage far
  from the curve (dist=300 m, apex clearly NOT passed); a pending single-tick outlier then reports
  an implausibly-close distance (dist=10 m, which WOULD flip apex_passed to True and trigger the FAST
  1 s early-restore debounce if it were trusted) but never confirms; the clear immediately follows.
  The full (slow) debounce must still apply, proving _last_cap_dist stayed at the CONFIRMED 300 m
  snapshot, not the unconfirmed 10 m one."""
  ep = IcbmEpisode()
  v = 31 * MPH
  t = T0
  ep.step(t, 31 * MPH, 50 * MPH, 50 * MPH, True, False, cap_dist=300.0, v_ego=v)
  # drive to confirmation manually (not via the shared _confirm helper) so cap_dist=300.0 is passed
  # on every tick -- the helper omits cap_dist, which would itself overwrite the snapshot with None
  for _ in range(8):
    t += TICK_S
    ep.step(t, 31 * MPH, 50 * MPH, 31 * MPH, True, False, cap_dist=300.0, v_ego=v)
    if ep._min_target is not None:
      break
  assert math.isclose(ep._min_target, 31 * MPH)
  assert math.isclose(ep._last_cap_dist, 300.0)
  ep._engage_t0 -= (ICBM_RATCHET_CONFIRM_S + 0.1)                     # long-since-confirmed engage
  t += TICK_S
  out = ep.step(t, 15 * MPH, 50 * MPH, 31 * MPH, True, False, cap_dist=10.0, v_ego=v)
  assert math.isclose(out[0], 31 * MPH)                               # outlier still pending
  assert math.isclose(ep._last_cap_dist, 300.0), \
      "an unconfirmed reading must not overwrite the distance snapshot"
  t += TICK_S
  out = ep.step(t, None, 50 * MPH, 31 * MPH, True, False)             # clears immediately
  assert out == (None, None) and ep.phase == "cap"
  # the FAST (1 s) early-restore debounce must NOT apply (that only fires if apex_passed)
  out = ep.step(t + ICBM_RESTORE_DELAY_FAST_S + 0.1, None, 50 * MPH, 31 * MPH, True, False)
  assert out == (None, None) and ep.phase == "cap", \
      "must still be in the SLOW debounce (judged on the confirmed 300 m), not restoring early"
  out = ep.step(t + ICBM_RESTORE_DELAY_S + 0.1, None, 50 * MPH, 31 * MPH, True, False)
  assert out[1] == "inc" and ep.phase == "restore"


# ---- Gemini adversarial-review catch, round 2 (2026-08-11) -----------------------------------------
def test_bad_engage_recovering_without_a_clear_gap_does_not_poison_future_outlier_detection():
  """Gemini review catch (round 2): the round-1 fix still unconditionally set _min_target = the
  engage tick's raw value, so a glitchy engage whose recovery arrives on the very NEXT tick (no
  cap_target=None gap in between -- the _engage_t0/clear-debounce guard only fires on an actual
  clear, so it could not catch this) would permanently taint _min_target/baseline at the outlier
  value. Every LATER reading was then compared against that tainted floor, so a second, genuinely
  anomalous glitch could slip through completely ungated (accepted immediately as "not much of a
  drop" from the wrong floor) -- recreating the original bug's effect with a different trigger.

  Replay: engage glitches to 15 mph (driver's set 60); the very next tick recovers directly to a
  real, sustained 50 mph curve (no clear in between); MUCH later, a genuinely anomalous second
  glitch (10 mph) arrives. That later glitch must be gated exactly like any other outlier -- it must
  NOT be accepted immediately just because it happens to be "close" to the original tainted 15 mph
  reading, and _min_target must never regress to (or near) 15 mph."""
  ep = IcbmEpisode()
  t = T0
  ep.step(t, 15 * MPH, 60 * MPH, 60 * MPH, True, False)               # glitch engage
  t += TICK_S
  ep.step(t, 50 * MPH, 60 * MPH, 15 * MPH, True, False)                # recovers, NO clear gap
  # sustained real curve lets the true 50 mph value confirm
  t, out = _confirm(ep, t, 50.0, 60.0, 50.0)
  assert math.isclose(ep._min_target, 50 * MPH, abs_tol=0.05), \
      "must confirm at the TRUE 50 mph, not stay poisoned at the 15 mph glitch"

  # much later, a genuinely new/separate glitch to 10 mph
  t += 10.0
  out = ep.step(t, 10 * MPH, 60 * MPH, 50 * MPH, True, False)
  assert math.isclose(out[0], 50 * MPH), \
      "a later glitch must be gated against the TRUE 50 mph floor, not the tainted 15 mph one"
  assert not math.isclose(ep._min_target, 15 * MPH, abs_tol=1.0), \
      "_min_target must never have regressed toward the original glitch value"
  assert math.isclose(ep._min_target, 50 * MPH, abs_tol=0.05)


def test_engage_publish_stays_bounded_while_recovery_confirms():
  """Companion to the above: while the post-engage recovery is itself still confirming (the brief,
  self-correcting window right after a bad engage), the episode keeps publishing its own prior
  commitment (never silently drops the cap, never jumps early to an unconfirmed value) -- and the
  delay before the corrected value takes over is bounded to roughly the confirmation window, not
  indefinite."""
  ep = IcbmEpisode()
  t = T0
  ep.step(t, 15 * MPH, 60 * MPH, 60 * MPH, True, False)
  t += TICK_S
  out = ep.step(t, 50 * MPH, 60 * MPH, 15 * MPH, True, False)
  # immediately after the recovery tick, still bounded at the prior (over-braking-direction, safe)
  # commitment -- not yet jumped to 50, but also never anything LOWER than what was ever published
  assert out[0] <= 15 * MPH + 0.01
  t2, out2 = _confirm(ep, t, 50.0, 60.0, 50.0, max_ticks=8)
  assert math.isclose(out2[0], 50 * MPH, abs_tol=0.05)
  assert (t2 - t) <= ICBM_RATCHET_CONFIRM_S + 3 * TICK_S, \
      "the correction must land within roughly one confirmation window, not be delayed indefinitely"
