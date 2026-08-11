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

FIX (IcbmEpisode._ratchet_confirm): an outlier-sized single-tick drop (> ICBM_RATCHET_OUTLIER_DROP_MS
below the last CONFIRMED target) must persist ICBM_RATCHET_CONFIRM_S before the ratchet adopts (or
publishes) it. These tests replay the field numbers directly: a single 31->15 mph tick must NOT reach
_min_target or the published target; a SUSTAINED 31->15 mph drop (a real tightening curve) still
fully brakes, just delayed by the confirmation window.
"""
import math

from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import (IcbmEpisode, ICBM_RATCHET_CONFIRM_S,
                                                              ICBM_RATCHET_OUTLIER_DROP_MS,
                                                              ICBM_RESTORE_DELAY_S)

MPH = 0.44704
T0 = 100.0
TICK_S = 0.25   # matches the real ~4 Hz _icbm_step publish cadence


def test_single_outlier_tick_does_not_latch_field_event():
  """THE 2026-08-10 field case, replayed with the real numbers: episode confirmed at 31 mph
  (icbmT ~13.86 m/s), ONE tick's candidate drops to 15 mph (a distorted vision-curvature reading),
  the very next tick recomputes back to 31 mph. The published target -- and _min_target, which
  gates restore eligibility -- must NEVER see 15 mph; the truck keeps braking to 31 mph throughout,
  never lurches to 15."""
  ep = IcbmEpisode()
  # engage: curve binds at 31 mph, driver's set was 50
  out = ep.step(T0, 31 * MPH, 50 * MPH, 50 * MPH, True, False)
  assert out == (31 * MPH, "dec") and math.isclose(ep._min_target, 31 * MPH)

  # ONE anomalous low tick: candidate plunges to 15 mph
  out = ep.step(T0 + TICK_S, 15 * MPH, 50 * MPH, 31 * MPH, True, False)
  assert out[1] == "dec"
  assert math.isclose(out[0], 31 * MPH), "a single outlier tick must not be published"
  assert math.isclose(ep._min_target, 31 * MPH), "a single outlier tick must not ratchet _min_target"

  # next tick: recomputed target recovers to 31 mph (the outlier was noise) -- pending discarded
  out = ep.step(T0 + 2 * TICK_S, 31 * MPH, 50 * MPH, 31 * MPH, True, False)
  assert out == (31 * MPH, "dec")
  assert ep._pending_low_target is None and ep._pending_low_t0 is None
  assert math.isclose(ep._min_target, 31 * MPH)

  # a THIRD low tick right after the recovery must independently need its own confirmation --
  # proves the pending state was truly discarded, not just "used up"
  out = ep.step(T0 + 3 * TICK_S, 15 * MPH, 50 * MPH, 31 * MPH, True, False)
  assert math.isclose(out[0], 31 * MPH)


def test_sustained_low_target_does_brake():
  """The flip side: a GENUINE tightening curve where the candidate drops to 15 mph and STAYS there
  across consecutive ticks must still fully brake -- the ratchet adopts the sustained low value
  once it persists ICBM_RATCHET_CONFIRM_S, exactly the drop the field event needed but a single
  glitch tick must not trigger."""
  ep = IcbmEpisode()
  ep.step(T0, 31 * MPH, 50 * MPH, 50 * MPH, True, False)
  assert math.isclose(ep._min_target, 31 * MPH)

  t = T0
  confirmed = False
  last_out = None
  # drive ticks until the confirmation window elapses; every tick reports the SAME genuinely low
  # target (a real curve rating does not fluctuate)
  while t - T0 < ICBM_RATCHET_CONFIRM_S + 3 * TICK_S:
    t += TICK_S
    out = ep.step(t, 15 * MPH, 50 * MPH, 31 * MPH, True, False)
    last_out = out
    if math.isclose(out[0], 15 * MPH, rel_tol=1e-6):
      confirmed = True
      break
    # while still pending, the episode must keep commanding the last CONFIRMED (31 mph) target --
    # never silently go slack, never jump to the unconfirmed value
    assert math.isclose(out[0], 31 * MPH)

  assert confirmed, f"sustained low target was never adopted (last={last_out})"
  assert math.isclose(ep._min_target, 15 * MPH)
  # and the adoption happened at/after the confirmation window, not before it
  assert t - T0 >= ICBM_RATCHET_CONFIRM_S


def test_small_gradual_decrease_not_gated():
  """Ordinary continuous refinement (small tick-to-tick drops, e.g. a curve estimate tightening as
  distance closes) is never treated as an outlier -- no confirmation delay, ratchets immediately,
  exactly like before the fix."""
  ep = IcbmEpisode()
  ep.step(T0, 40 * MPH, 60 * MPH, 60 * MPH, True, False)
  step_mph = (ICBM_RATCHET_OUTLIER_DROP_MS / MPH) * 0.5   # half the outlier threshold, in mph
  target = 40.0
  t = T0
  for _ in range(4):
    t += TICK_S
    target -= step_mph
    out = ep.step(t, target * MPH, 60 * MPH, target * MPH, True, False)
    assert math.isclose(out[0], target * MPH), "small gradual drops must publish immediately"
    assert math.isclose(ep._min_target, target * MPH)
  assert ep._pending_low_target is None


def test_rising_target_never_gated():
  """A rise (or full recovery) is never an 'outlier drop' -- publishes immediately, _min_target
  (the low-water mark used for restore eligibility) is untouched by a rise."""
  ep = IcbmEpisode()
  ep.step(T0, 30 * MPH, 60 * MPH, 60 * MPH, True, False)
  out = ep.step(T0 + TICK_S, 45 * MPH, 60 * MPH, 30 * MPH, True, False)
  assert out == (45 * MPH, "dec")
  assert math.isclose(ep._min_target, 30 * MPH)   # never raised by a rising candidate


def test_outlier_pending_does_not_corrupt_restore_eligibility():
  """The 2026-08-10 mechanism must not leak a false-low _min_target into the restore-eligibility
  gate (ICBM_DRIVER_LOWER_TOL check): after an outlier tick is correctly rejected, a subsequent
  clear + restore must target the ORIGINAL ceiling, using the CONFIRMED 31 mph floor -- not the
  never-adopted 15 mph outlier."""
  ep = IcbmEpisode()
  ep.step(T0, 31 * MPH, 50 * MPH, 50 * MPH, True, False)          # ceiling 50, min_target 31
  ep.step(T0 + TICK_S, 15 * MPH, 50 * MPH, 31 * MPH, True, False)  # outlier tick, rejected
  ep.step(T0 + 2 * TICK_S, 31 * MPH, 50 * MPH, 31 * MPH, True, False)  # recovered, confirmed
  assert math.isclose(ep._min_target, 31 * MPH)

  # curve clears, stock set sits at 31 (never actually reached 15) -> sustained clear -> restore
  t = T0 + 3 * TICK_S
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
  # 4 consecutive outlier-sized-drop ticks at the real ~4 Hz cadence: 15, 14, 16, 16 mph.
  # ICBM_RATCHET_CONFIRM_S (0.6 s) elapses on the 4th tick (0.75 s since the 1st pending tick) --
  # the ADOPTED value must be 14 (the worst ever seen), not 16 (the latest / confirming reading).
  readings = [15.0, 14.0, 16.0, 16.0]
  t = T0
  out = None
  for r in readings:
    t += TICK_S
    out = ep.step(t, r * MPH, 50 * MPH, 31 * MPH, True, False)
  assert math.isclose(out[0], 14 * MPH, abs_tol=0.05), out
  assert math.isclose(ep._min_target, 14 * MPH, abs_tol=0.05)


# ---- Fable + Gemini adversarial-review catches (2026-08-11, second pass) --------------------------
def test_pending_state_does_not_survive_a_cap_dropout_field_replay():
  """Fable review catch: _ratchet_confirm measures WALL TIME, not contiguous low readings. Without
  clearing the pending candidate when the cap goes silent, a glitch tick sighted just before a brief
  detection dropout could sit stale through the gap and then get instantly "confirmed" the moment
  ANY new, unrelated, merely outlier-sized-but-LEGITIMATE candidate rebinds -- recreating the exact
  field bug through a different path. Replay: confirmed at 31 mph -> one glitch tick to 15 mph ->
  three ticks of cap_target=None (a detection dropout, well inside the clear debounce) -> curve
  rebinds at a legitimate 20 mph (an outlier-sized drop from 31, same as the glitch was, so the old
  bug's stale-confirm path is squarely exercised). The 20 mph must be treated as a FRESH pending
  candidate (its own confirmation window), never instantly confirmed at the stale 15 mph via the old
  pending state."""
  ep = IcbmEpisode()
  ep.step(T0, 31 * MPH, 50 * MPH, 50 * MPH, True, False)                      # confirmed 31 mph
  ep._engage_t0 -= (ICBM_RATCHET_CONFIRM_S + 0.1)   # engage itself is long-since confirmed
  t = T0 + TICK_S
  ep.step(t, 15 * MPH, 50 * MPH, 31 * MPH, True, False)                       # glitch tick, pending
  assert ep._pending_low_target is not None and ep._pending_low_t0 is not None
  for _ in range(3):                                                          # dropout: cap silent
    t += TICK_S
    out = ep.step(t, None, 50 * MPH, 31 * MPH, True, False)
    assert out == (None, None)
  assert ep._pending_low_target is None and ep._pending_low_t0 is None, \
      "pending outlier state must be voided by a cap dropout, not carried across it"
  t += TICK_S
  out = ep.step(t, 20 * MPH, 50 * MPH, 31 * MPH, True, False)                 # legitimate rebind
  assert not math.isclose(out[0], 15 * MPH), "must not instantly confirm the stale glitch value"
  assert math.isclose(out[0], 31 * MPH), "20 mph must itself now be pending, not yet published"
  assert math.isclose(ep._min_target, 31 * MPH)


def test_fresh_engage_then_immediate_clear_does_not_survive_for_a_later_rebind():
  """Gemini review catch: the original fix bypassed confirmation entirely on a FRESH episode's very
  first tick (there was no prior _min_target to compare against). An outlier vision glitch that
  itself TRIGGERS a new episode (binds because a very low target needs a very long brake distance,
  so it can bind well before the driver is anywhere near a real curve) would latch a bad ceiling/
  _min_target immediately, unprotected -- and because caps are DEC-only, a later correct, higher
  target published after the glitch clears would be forwarded but never actually undo the executor's
  already-completed downward taps (only the fully separate, guarded RESTORE can do that, and it only
  fires on a full curve-clear back to the ORIGINAL ceiling -- never to a later, different curve's
  correct target). Fix: an engage that goes silent again before ICBM_RATCHET_CONFIRM_S elapses is
  treated as never having proven itself -- the whole episode is wiped, so a later, unrelated,
  legitimate curve starts a completely FRESH episode/ceiling instead of resuming the tainted one."""
  ep = IcbmEpisode()
  # tick 1: spurious 15 mph engage while driver's set is 50 (nowhere near a real curve yet)
  out = ep.step(T0, 15 * MPH, 50 * MPH, 50 * MPH, True, False)
  assert out == (15 * MPH, "dec") and ep.phase == "cap" and math.isclose(ep._min_target, 15 * MPH)
  # tick 2, well within ICBM_RATCHET_CONFIRM_S: the glitch clears -- engage never proved itself
  out = ep.step(T0 + TICK_S, None, 50 * MPH, 50 * MPH, True, False)
  assert out == (None, None)
  assert ep.phase == "idle" and ep.ceiling is None and ep._min_target is None, \
      "an engage that clears before confirming must wipe the episode entirely"
  # tick 3: a later, legitimate, DIFFERENT curve binds -- must start a completely fresh episode,
  # never resume/relatch anything from the wiped 15 mph engage
  out = ep.step(T0 + 2 * TICK_S, 45 * MPH, 50 * MPH, 50 * MPH, True, False)
  assert out == (45 * MPH, "dec") and math.isclose(ep.ceiling, 50 * MPH)
  assert math.isclose(ep._min_target, 45 * MPH)


def test_engage_confirmed_by_persistence_is_not_wiped_on_a_later_clear():
  """The flip side of the fresh-engage guard: a REAL curve that stays bound for longer than
  ICBM_RATCHET_CONFIRM_S before eventually clearing must NOT be treated as unconfirmed -- the
  episode's ceiling/min_target survive to a normal clear-debounce/restore cycle exactly as before
  this review round."""
  ep = IcbmEpisode()
  ep.step(T0, 31 * MPH, 50 * MPH, 50 * MPH, True, False)
  t = T0 + ICBM_RATCHET_CONFIRM_S + 0.2               # curve stayed bound past the confirm window
  ep.step(t, 31 * MPH, 50 * MPH, 31 * MPH, True, False)
  t += TICK_S
  out = ep.step(t, None, 50 * MPH, 31 * MPH, True, False)    # now it clears
  assert out == (None, None) and ep.phase == "cap" and ep.ceiling is not None, \
      "a confirmed (long-bound) engage must survive a clear into the normal debounce"


def test_extreme_outlier_inside_pending_window_does_not_hijack_it():
  """Gemini review catch ("worst-seen hijacking"): a legitimate drop opens a pending window (real
  curve tightening 50 -> 40 mph); mid-window, ONE extreme single-tick glitch (10 mph) arrives, then
  the reading recovers back to the true 40 mph for the rest of the window. Without a consistency
  check, unconditional min() adopts the drive-by glitch (10) just because it happened to land inside
  a window some OTHER, legitimate reading opened. The fix restarts the confirmation window whenever
  a reading deviates from the currently-tracked pending candidate by more than the outlier band, so
  the 10 mph blip can never hijack the 40 mph window -- the adopted value must be 40, never 10."""
  ep = IcbmEpisode()
  ep.step(T0, 50 * MPH, 60 * MPH, 60 * MPH, True, False)             # confirmed at 50
  t = T0
  # legit drop opens the window
  t += TICK_S
  out = ep.step(t, 40 * MPH, 60 * MPH, 50 * MPH, True, False)
  assert math.isclose(out[0], 50 * MPH)                              # still pending, holds at 50
  # one extreme glitch tick lands mid-window
  t += TICK_S
  out = ep.step(t, 10 * MPH, 60 * MPH, 50 * MPH, True, False)
  assert not math.isclose(out[0], 10 * MPH)
  assert 10 * MPH not in (round(ep._min_target, 4) if ep._min_target else None,)
  # recovers to the true, legitimate value and stays there until confirmed
  t += TICK_S
  out = ep.step(t, 40 * MPH, 60 * MPH, 50 * MPH, True, False)
  while not math.isclose(out[0], 40 * MPH, abs_tol=0.05) and t - T0 < 3.0:
    t += TICK_S
    out = ep.step(t, 40 * MPH, 60 * MPH, 50 * MPH, True, False)
    assert not math.isclose(out[0], 10 * MPH), "the 10 mph glitch must never be published"
  assert math.isclose(out[0], 40 * MPH, abs_tol=0.05), out
  assert math.isclose(ep._min_target, 40 * MPH, abs_tol=0.05), \
      "the confirmed value must be the legitimate 40 mph reading, never the 10 mph outlier"
