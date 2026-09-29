"""teslastalk2pnw -- LOGGING ONLY: one ces_events record per change of the Raven's EPS-refusal verdict.

WHY. After teslamads2pnw the Raven steers with DI_cruiseState in STANDBY (a brake no longer ends lateral). Whether the EPS
accepts angle commands in that state has never been observed. opendbc's Tesla carstate now judges it (next_eps_refusal:
lateral-only commanded for EPS_REFUSAL_FRAMES consecutive frames while eacStatus != EAC_ACTIVE) and raises
steerFaultTemporary (SOFT_DISABLE: in steering-only it ends lateral, with the full-disengage chime and no text; if the wheel was touched
in the last 1.5 s only a WARNING-only steerTempUnavailableSilent results and the verdict re-fires every ~0.5 s). This is the record that lets a drive be analysed afterwards.

WHAT. {"ev":"epsRef"} appended to /data/pnw/ces_events.jsonl on a change of the verdict (held off to one record per HOLDOFF_S
while it flips; `suppressed` counts the held-off flips; the first onset and the final state are always recorded), plus one record at the first
frame (from null) so a drive with no refusal can be told apart from a logger that never ran:
  ref            the verdict now (true = the EPS is not following a lateral-only command)
  eac / err      EPAS_eacStatus / EPAS_eacErrorCode raw values at the change (2 = ACTIVE, 1 = AVAILABLE, 0 = INHIBITED)
  latActive, enabled, vEgo, sp   carControl / carState at the tick
  route / seg    loggerd's route and segment

NOT A CONTROL INPUT: it reads carstate attributes and writes a file on a daemon thread. A write failure is logged loudly
(Rule 2); the verdict itself does not depend on this module.
CAPABILITY VIEW: card builds it only when PnwVehicle.eps_refusal_alert.
"""
import threading
import time

from opendbc.car.holdoff_pnw import HoldOffGate
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.accdrop_pnw import append_to_ces_log, current_route_segment

EV_NAME = "epsRef"
SCHEMA = 1
# teslastalk2b: the verdict can flip every ~0.51 s while the wheel is touched (steerTempUnavailableSilent re-fires every 51
# frames) = ~4 records/s. Re-record at most once per this many seconds; the FIRST onset and the FINAL state are always recorded
# and each record carries `suppressed` = the flips held off since the previous record.
HOLDOFF_S = 2.0


class EpsRefusalLogger:
  def __init__(self, write_fn=append_to_ces_log, route_fn=current_route_segment, threaded: bool = True,
               clock=time.monotonic):
    self._write_fn = write_fn
    self._route_fn = route_fn
    self._threaded = threaded
    self._clock = clock
    self._gate = HoldOffGate(HOLDOFF_S, initial=None)
    self.write_errors = 0

  def update(self, refused: bool, eac: int | None, err: int | None, CC, CS) -> bool:
    """One card tick. Returns True when a record was emitted (tests)."""
    refused = bool(refused)
    out = self._gate.step(refused, self._clock())
    if out is None:
      return False
    prev, suppressed = out
    held = f" ({suppressed} flips suppressed by the {HOLDOFF_S:.0f} s hold-off)" if suppressed else ""
    if refused and prev is not True:
      cloudlog.error(f"epsref_pnw: EPS REFUSAL began{held} (eac={eac} err={err}, latActive={bool(CC.latActive)}, enabled={bool(CC.enabled)})")
    elif refused:
      cloudlog.error(f"epsref_pnw: EPS REFUSAL still flapping{held} (eac={eac} err={err})")
    elif prev:
      cloudlog.warning(f"epsref_pnw: EPS refusal cleared{held}")
    elif suppressed:
      cloudlog.warning(f"epsref_pnw: EPS refusal flapped and is clear again{held}")
    rec = {"t": round(time.time(), 2), "ev": EV_NAME, "v": SCHEMA,  # noqa: TID251 -- wall-clock log stamp
           "ref": refused, "from": prev, "suppressed": suppressed, "eac": eac, "err": err,
           "latActive": bool(CC.latActive), "enabled": bool(CC.enabled),
           "vEgo": round(float(CS.vEgo), 2), "sp": bool(CS.steeringPressed)}
    if self._threaded:
      threading.Thread(target=self._write, args=(rec,), daemon=True, name="epsreflog").start()
    else:
      self._write(rec)
    return True

  def _write(self, rec: dict) -> None:
    import json
    try:
      rec.update(self._route_fn(rec["t"]))
      self._write_fn(json.dumps(rec, separators=(",", ":")))
    except Exception:
      self.write_errors += 1
      cloudlog.exception(f"epsref_pnw: writing the record FAILED ({self.write_errors}) -- this refusal event is NOT in ces_events")
