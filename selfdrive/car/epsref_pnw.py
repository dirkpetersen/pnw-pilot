"""teslastalk2pnw -- LOGGING ONLY: one ces_events record per change of the Raven's EPS-refusal verdict.

WHY. After teslamads2pnw the Raven steers with DI_cruiseState in STANDBY (a brake no longer ends lateral). Whether the EPS
accepts angle commands in that state has never been observed. opendbc's Tesla carstate now judges it (next_eps_refusal:
lateral-only commanded for EPS_REFUSAL_FRAMES consecutive frames while eacStatus != EAC_ACTIVE) and raises
steerFaultTemporary (SOFT_DISABLE: in steering-only it ends lateral, with the full-disengage chime and no text; if the wheel was touched
in the last 1.5 s only a WARNING-only steerTempUnavailableSilent results and the verdict re-fires every ~0.5 s). This is the record that lets a drive be analysed afterwards.

WHAT. {"ev":"epsRef"} appended to /data/pnw/ces_events.jsonl on every change of the verdict, plus one record at the first
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

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.accdrop_pnw import append_to_ces_log, current_route_segment

EV_NAME = "epsRef"
SCHEMA = 1


class EpsRefusalLogger:
  def __init__(self, write_fn=append_to_ces_log, route_fn=current_route_segment, threaded: bool = True):
    self._write_fn = write_fn
    self._route_fn = route_fn
    self._threaded = threaded
    self._last: bool | None = None
    self.write_errors = 0

  def update(self, refused: bool, eac: int | None, err: int | None, CC, CS) -> bool:
    """One card tick. Returns True when a record was emitted (tests)."""
    refused = bool(refused)
    if refused == self._last:
      return False
    prev, self._last = self._last, refused
    if refused:
      cloudlog.error(f"epsref_pnw: EPS REFUSAL began (eac={eac} err={err}, latActive={bool(CC.latActive)}, enabled={bool(CC.enabled)})")
    elif prev:
      cloudlog.warning("epsref_pnw: EPS refusal cleared")
    rec = {"t": round(time.time(), 2), "ev": EV_NAME, "v": SCHEMA,  # noqa: TID251 -- wall-clock log stamp
           "ref": refused, "from": prev, "eac": eac, "err": err,
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
