"""speedlimitconfirm2pnw — the lower-limit banner must not fire on a single mapd sample.

2026-08-18 live drive: after a mid-drive reboot mapd re-matched position, briefly landed on a parallel
surface street, and the overlay flashed a 25 mph LOWER-LIMIT warning while the car was on I-5 at 70.
The existing STALE_AFTER_S gate cannot catch this — both the 60 and the 25 were FRESH reads, so the
drop looked genuine. A lower limit must now persist DROP_CONFIRM_S before it reaches the screen.
"""
import pytest

# Importing the widget pulls in raylib AND ui_state, which reads params through the COMPILED
# params_pyx -- so this module only imports in a properly built tree (device / CI / a fork venv with
# scons run). Skip rather than error in a partially-built dev worktree; it still runs where it counts.
try:
  from openpilot.selfdrive.ui.onroad.speed_limit import (
    SpeedLimitRenderer, DROP_CONFIRM_S, STALE_AFTER_S, MIN_VALID_KPH)
except Exception as _e:                                    # pragma: no cover - environment guard
  pytest.skip(f"raylib/params UI deps unavailable in this tree: {_e}", allow_module_level=True)


def _renderer(shown_limit, speed):
  """Bypass __init__ (it loads fonts / needs a GL context) and set only the state under test."""
  r = object.__new__(SpeedLimitRenderer)
  r.speed = speed
  r.speed_limit_valid = True
  r.speed_limit_ahead_valid = False
  r.speed_limit_ahead = 0.0
  r.speed_limit_ahead_dist = 0.0
  r._shown_limit = shown_limit
  r._warn_value = 0.0
  r._warn_until = 0.0
  r._pending_drop = 0.0
  r._pending_drop_t = 0.0
  return r


def _tick(r, new_limit, now):
  """One _maybe_trigger_warning cycle at monotonic time `now`, mirroring _update_state's ordering."""
  import time as _t
  real = _t.monotonic
  _t.monotonic = lambda: now
  try:
    r._maybe_trigger_warning(new_limit)
  finally:
    _t.monotonic = real
  if r.speed_limit_valid and new_limit > MIN_VALID_KPH:
    r._shown_limit = new_limit
    r._shown_limit_t = now
  return r._warn_until > now


def test_transient_drop_never_warns():
  """The reboot artifact: 60 -> 25 for a single sample -> back to 60."""
  r = _renderer(shown_limit=60.0, speed=70.0)
  r._shown_limit_t = 100.0
  assert not _tick(r, 25.0, 100.5), "a single low sample must not put a red banner on screen"
  assert not _tick(r, 60.0, 101.0)
  assert r._pending_drop == 0.0


def test_sustained_drop_still_warns():
  r = _renderer(shown_limit=60.0, speed=70.0)
  r._shown_limit_t = 100.0
  assert not _tick(r, 25.0, 100.5)                       # armed, not fired
  assert _tick(r, 25.0, 100.5 + DROP_CONFIRM_S + 0.1), "a real, persistent drop must still warn"
  assert round(r._warn_value) == 25


def test_warning_is_delayed_by_at_most_the_confirm_window():
  r = _renderer(shown_limit=60.0, speed=70.0)
  r._shown_limit_t = 100.0
  _tick(r, 45.0, 100.5)
  assert not _tick(r, 45.0, 100.5 + DROP_CONFIRM_S - 0.2), "must not fire before the window elapses"
  assert _tick(r, 45.0, 100.5 + DROP_CONFIRM_S + 0.01)


def test_driver_slowing_during_confirmation_cancels_the_warning():
  # the banner exists to flag OVERSPEEDING into a lower limit; if they slowed, there is nothing to warn
  r = _renderer(shown_limit=60.0, speed=70.0)
  r._shown_limit_t = 100.0
  _tick(r, 45.0, 100.5)
  r.speed = 40.0
  assert not _tick(r, 45.0, 100.5 + DROP_CONFIRM_S + 0.5)
  assert r._pending_drop == 0.0


def test_stale_baseline_gate_still_applies():
  # pre-existing behaviour must survive: re-acquiring a limit after a long gap is a fresh fix, not a drop
  r = _renderer(shown_limit=60.0, speed=70.0)
  r._shown_limit_t = 100.0
  assert not _tick(r, 25.0, 100.0 + STALE_AFTER_S + 5.0)
  assert r._pending_drop == 0.0, "a stale baseline must not even arm a pending drop"
