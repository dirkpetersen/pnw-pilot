"""
network2xnor: LTE PDN-throttle backoff guard (PURE decision logic).

Problem (observed live 2026-06-15): an intense speed test (e.g. Ookla) can trip a carrier/modem PDN
throttle — the modem refuses new IPv4 bearer activations and NetworkManager logs
`pdn-ipv4-call-throttled`. NM then RETRIES the bearer ~once/second, and every failed attempt RESETS
the carrier's tower-side throttle timer, so it NEVER clears — LTE stays dead indefinitely (Verizon
treats the rapid-fire PDN requests like a DoS and keeps the IMEI flagged).

Fix: when we detect the throttle, STOP NM hammering (park the lte connection: autoconnect off + down)
for an exponentially increasing backoff, then bring it back up ONCE. If it's still throttled, back off
longer; once it gets an IP, reset. A silent, parked modem lets the carrier timer age out (~15-30 min),
and a single clean re-attempt then succeeds — so a future speed test self-recovers without locking out.

This module is PURE (no I/O, no time source) so it is unit-testable; `network_arbiterd` supplies the
clock + nmcli I/O. SI: all durations in seconds.
"""

# exponential backoff: 30 s -> 2 min -> 5 min -> 10 min (cap). Each consecutive throttle steps up.
BACKOFF_SCHEDULE_S = (30.0, 120.0, 300.0, 600.0)


def next_backoff_s(throttle_count: int) -> float:
  """Backoff duration for the Nth consecutive throttle (0-indexed). Clamps at the last (longest) step."""
  idx = max(0, min(throttle_count, len(BACKOFF_SCHEDULE_S) - 1))
  return BACKOFF_SCHEDULE_S[idx]


def decide_lte_guard(now: float, throttled: bool, lte_has_ip: bool,
                     parked: bool, parked_until: float, throttle_count: int):
  """PURE. One step of the LTE-throttle guard state machine.

  Inputs:
    now           monotonic seconds
    throttled     a `pdn-ipv4-call-throttled` error was seen recently (NM hammering the modem)
    lte_has_ip    wwan0 currently has an IPv4 address (LTE data is up and healthy)
    parked        are we currently in a backoff park?
    parked_until  monotonic deadline the current park ends at
    throttle_count how many consecutive throttle-parks we've done (drives the backoff length)

  Returns (action, new_parked, new_parked_until, new_throttle_count) where action is one of:
    'park'   -> stop NM hammering: `con down lte` + autoconnect off, stay parked until parked_until
    'unpark' -> backoff elapsed: re-enable autoconnect + `con up lte` ONCE (single clean attempt)
    'noop'   -> do nothing this tick

  Behavior-neutral when LTE is healthy (has IP) and never throttled: always 'noop', count stays 0.
  """
  if parked:
    if now >= parked_until:
      # backoff elapsed -> one clean re-attempt. Keep throttle_count so a re-throttle escalates.
      return "unpark", False, 0.0, throttle_count
    return "noop", True, parked_until, throttle_count

  # not parked
  if lte_has_ip:
    # healthy -> reset the escalation
    return "noop", False, 0.0, 0

  if throttled:
    # trip the backoff: park for the next (escalating) duration
    backoff = next_backoff_s(throttle_count)
    return "park", True, now + backoff, throttle_count + 1

  # no IP but not (yet) seeing the throttle string — let NM's own logic run, don't interfere
  return "noop", False, 0.0, throttle_count
