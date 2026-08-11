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
  """While a drop is pending, the WORST (lowest) value seen during the window is what gets adopted
  -- conservative by design (never less braking than the sustained readings warranted), not
  whatever the LATEST (confirming) tick happened to report."""
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
