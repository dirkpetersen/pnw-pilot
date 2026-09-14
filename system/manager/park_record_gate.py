"""parknorec2pnw: the device does not record while the shifter is in Park.

Driver requirement 2026-09-05: "it should never be recording when the shifter is on Park, never."
IsOnroad follows IGNITION (CLAUDE.md Rule 3): a parked, charging Lightning is onroad indefinitely, and
loggerd wrote a segment every minute (measured 2026-09-13 at a hotel with SkipVideoWhenParked and
ThinRlogWhenParked both on: ~3 MB/min, ~180 MB/h, uploading over the driver's phone).

The gate stops LOGGERD through its manager should_run, so no route, segment, rlog, qlog or video is
created. Everything else keeps running -- camerad, modeld, card, controls, and encoderd -- so openpilot
is ready the instant the shifter leaves Park, and the video encoders are already warm (see the
PARKNOREC2PNW doc for the measured start-up latency).

Truth source: the GearPark param (card, change-only; rules in selfdrive/car/gear_park.py). The manager
must not subscribe to carState. Every doubt resolves toward RECORDING:
  - hold only after GearPark has read True continuously for PARK_HOLD_S (hysteresis: a P->D->P
    shuffle in a parking lot neither restarts loggerd nor splits routes),
  - release immediately when GearPark reads False,
  - a GearPark that card is not alive to vouch for is ignored (card crashed / not started),
  - RecordWhileParked=1 disables the gate (kill switch, re-read every tick, no deploy needed),
  - a params error records, and says so.
Every hold/release is logged with its reason, so a gate that does nothing can be told apart from a gate
that is holding.
"""
import time
from collections.abc import Callable

from openpilot.common.params import Params, UnknownKeyName
from openpilot.common.swaglog import cloudlog

PARK_HOLD_S = 30.0


class ParkRecordGate:
  def __init__(self, card_alive: Callable[[], bool], clock: Callable[[], float] = time.monotonic) -> None:
    self._card_alive = card_alive
    self._clock = clock
    self._park_since: float | None = None
    self.holding = False
    self._params_error_logged = False

  def _release(self, reason: str) -> bool:
    self._park_since = None
    if self.holding:
      cloudlog.event("park_record_gate", hold=False, reason=reason)
    self.holding = False
    return False

  def update(self, started: bool, params: Params) -> bool:
    """True = HOLD (loggerd must not run). Call once per manager tick."""
    if not started:
      return self._release("offroad")

    try:
      disabled = params.get_bool("RecordWhileParked")
      gear_park = params.get_bool("GearPark")
    except UnknownKeyName:
      if not self._params_error_logged:
        self._params_error_logged = True
        cloudlog.exception("park_record_gate: param key unknown (params_pyx.so not rebuilt?) -- recording")
      return self._release("params_error")
    self._params_error_logged = False

    if disabled:
      return self._release("RecordWhileParked")
    if not gear_park:
      return self._release("not_in_park")
    if not self._card_alive():
      # card crashed (or has not started yet at ignition-on): nobody is vouching for GearPark. Recording
      # resumes at once and a hold that was active is logged as released for this reason. The crash itself
      # is already loud (card: "card: fatal crash"; manager: "Restarting card"), and at ignition-on this
      # is routine for one tick, so it is deliberately not a second error event.
      return self._release("card_not_running")

    now = self._clock()
    if self._park_since is None:
      self._park_since = now
    if not self.holding and now - self._park_since >= PARK_HOLD_S:
      self.holding = True
      cloudlog.event("park_record_gate", hold=True, reason="park", park_s=round(now - self._park_since, 1))
    return self.holding
