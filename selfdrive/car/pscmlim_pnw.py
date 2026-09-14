"""pscmlimlog2pnw -- LOGGING ONLY: record every change of the steering rack's own lateral-limit report.

WHY. drives/2026-09-12/central-oregon-weekend/PSCM_LIMITREACHED.md: on 2026-09-08 19:44 PT the Lightning's PSCM reported
LimitClose, then LimitReached (Lane_Assist_Data3_FD1.LatCtlLim_D_Stat) while angle steering held the wheel on a -24 deg
plateau and kept raising its request. Nothing decoded that signal. It reached no alert, no log field and no qlog, so how
often the rack runs out of authority, and at what speed, is unknown.

WHAT. One {"ev":"pscmLim"} record per change of LatCtlLim_D_Stat, appended to /data/pnw/ces_events.jsonl:
  from / to   previous and new value: 0 LimitNotReached, 1 LimitClose, 2 LimitReached, 3 LimitWithDriverActive
  heldS       how long `from` lasted
  cpb         LatCtlCpblty_D_Stat from the same frame (its meaning on this truck is unknown -- report s2)
  latActive, kDes                      carControl at the tick the change was seen (kDes = actuators.curvature)
  vEgo, ang, tq, sp, yaw               carState at that tick (steering angle deg, column torque Nm, pressed, yaw rad/s)
  pa          the path angle LateralAngleExt last computed (its internal sign; the wire carries -pa), or null + paWhy
  route / seg loggerd's route and segment (accdrop_pnw.current_route_segment)
Also, once per card session: a record at the FIRST frame (from null + fromWhy), so a drive without a limit can be told
apart from a logger that never ran; or, if no frame arrived within UNSEEN_S, a record with to null + why.

NOT A CONTROL INPUT, AND IT MUST NOT BECOME ONE BY ACCIDENT. opendbc's lateral_angle_pnw.py reads
getattr(CS, 'lat_ctl_lim_stat', 0) and nothing sets that name, so its PSCM clamp branches are dead code. Feeding them
this signal would have frozen the 09-08 command below what the truck was still delivering (report s5, option H1). This
module writes nothing to CarState, the CarController or the CAN parser. test_pscmlim_pnw.py guards that the clamp
still sees 0 when a real limit frame has been decoded and logged.

HOW. Reads the parser row that opendbc's Ford carstate ALREADY registers on CAN-FD cars (it decodes LatCtlSte_D_Stat
from the same frame), so this needs no opendbc change, no pin bump and no cereal field. It reads vl_all, not vl:
vl_all holds every frame parsed this tick, so two frames in one CAN batch cannot hide a change. It never registers a
message (dict accessors only -- indexing CANParser.vl registers a name ALIVE-CHECKED; see accdrop_pnw).

COST. Per card tick: one dict.get on the parser row and a loop over the 0-1 values parsed this tick. carState and
carControl are read, and the record built, only on a change. The route lookup, JSON and the file append run on a daemon
thread. At most MAX_RECORDS_PER_MIN records a minute; changes beyond that are counted into the next record.

CAPABILITY VIEW. PnwVehicle.pscm_limit_report names the bus, message and signals: () on every car but the Lightning,
where card never builds this.
"""
import json
import math
import threading
import time
from collections import deque

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.accdrop_pnw import append_to_ces_log, current_route_segment

EV_NAME = "pscmLim"
SCHEMA = 1
MAX_RECORDS_PER_MIN = 20   # the 09-08..09-12 alert segments had 210 non-zero engaged frames in 36,814: edges are rare
UNSEEN_S = 10.0            # card starts after fingerprinting with the bus awake; 0x3CC arrives at ~33 Hz
RETRY_TICKS = 100          # re-check an unregistered message once a second


def _num(x, nd: int):
  """Rounded float, or None for a non-finite value (never a made-up 0)."""
  x = float(x)
  return round(x, nd) if math.isfinite(x) else None


class PscmLimitLogger:
  def __init__(self, can_parsers: dict, report: tuple, car_controller=None,
               write_fn=append_to_ces_log, route_fn=current_route_segment, threaded: bool = True):
    self._parsers = can_parsers
    self._bus, self._msg, self._lim_sig, self._cpb_sig = report
    self._cc_obj = car_controller
    self._write_fn = write_fn
    self._route_fn = route_fn
    self._threaded = threaded

    # Resolved on update(), not here: card builds this before its first CarInterface.update(), and carstate registers
    # the message lazily on that first update.
    self._row = None
    self._why = "not resolved yet"
    self._ticks = 0
    self._start = None
    self._val = None           # last LatCtlLim_D_Stat seen; None until the first frame
    self._since = None
    self._unseen_reported = False
    self._recent: deque = deque()
    self._suppressed = 0
    self._write_fail = 0

  def _resolve(self) -> None:
    p = self._parsers.get(self._bus)
    if p is None:
      self._why = f"no '{self._bus}' CAN parser on this car"
    elif not dict.__contains__(p.vl, self._msg):   # NEVER p.vl[name]: that registers it alive-checked
      self._why = (f"{self._msg} is not registered by opendbc carstate this session " +
                   "(the Ford carstate decodes it only on CAN-FD cars)")
    else:
      self._row = p.vl_all[self._msg]              # a plain dict: indexing it registers nothing
      self._why = f"{self._msg} is registered but no frame was received"

  # ---------------------------------------------------------------- 100 Hz, card loop
  def update(self, cs, cc, now: float) -> None:
    self._ticks += 1
    if self._start is None:
      self._start = now
    if self._row is None and (self._ticks == 1 or self._ticks % RETRY_TICKS == 0):
      self._resolve()
    vals = self._row.get(self._lim_sig) if self._row is not None else None   # .get: the defaultdict must not grow
    if not vals:
      if self._val is None:
        self._check_unseen(cs, cc, now)
      return
    for i, v in enumerate(vals):
      if v != self._val:
        cpbs = self._row.get(self._cpb_sig)
        self._change(cs, cc, now, v, cpbs[i] if cpbs is not None and i < len(cpbs) else None)

  def _change(self, cs, cc, now: float, v: float, cpb) -> None:
    first = self._val is None
    rec = {"from": None if first else int(self._val), "to": int(v),
           "heldS": None if first else round(now - self._since, 2),
           "cpb": None if cpb is None else int(cpb)}
    if first:
      rec["fromWhy"] = f"first frame this card session, {now - self._start:.1f} s after start"
      if self._unseen_reported:
        cloudlog.warning(f"pscmlimlog2pnw: first {self._msg} frame {now - self._start:.0f} s after start -- logging from here on")
    self._val, self._since = v, now
    self._record(rec, cs, cc, now)

  def _check_unseen(self, cs, cc, now: float) -> None:
    if self._unseen_reported or now - self._start < UNSEEN_S:
      return
    self._unseen_reported = True
    why = f"{self._why} {now - self._start:.0f} s after card started"
    cloudlog.error(f"pscmlimlog2pnw: {why} -- PSCM lateral-limit reports are NOT being logged")
    self._record({"from": None, "to": None, "heldS": None, "cpb": None, "why": why}, cs, cc, now)

  def _record(self, fields: dict, cs, cc, now: float) -> None:
    while self._recent and now - self._recent[0] > 60.0:
      self._recent.popleft()
    if len(self._recent) >= MAX_RECORDS_PER_MIN:
      self._suppressed += 1
      if self._suppressed == 1:
        cloudlog.warning(f"pscmlimlog2pnw: >{MAX_RECORDS_PER_MIN} records in 60 s -- suppressing (counted into the next one); " +
                         f"latest {fields.get('from')}->{fields.get('to')}")
      return
    self._recent.append(now)
    rec = {"t": round(time.time(), 2), "ev": EV_NAME, "v": SCHEMA, **fields,  # noqa: TID251 -- wall-clock log stamp
           "latActive": bool(cc.latActive), "kDes": _num(cc.actuators.curvature, 6),
           "vEgo": _num(cs.vEgo, 2), "ang": _num(cs.steeringAngleDeg, 1), "tq": _num(cs.steeringTorque, 2),
           "sp": bool(cs.steeringPressed), "yaw": _num(cs.yawRate, 4)}
    rec.update(self._path_angle())
    if self._suppressed:
      rec["suppressed"] = self._suppressed
      self._suppressed = 0
    if self._threaded:
      threading.Thread(target=self._write, args=(rec,), daemon=True, name="pscmlimlog").start()
    else:
      self._write(rec)

  def _path_angle(self) -> dict:
    if self._cc_obj is None:
      return {"pa": None, "paWhy": "no carcontroller"}
    if not hasattr(self._cc_obj, "_latext_angle"):
      return {"pa": None, "paWhy": "this carcontroller has no angle-mode strategy"}
    ext = self._cc_obj._latext_angle
    if ext is None:
      return {"pa": None, "paWhy": "angle mode is not running (FordAngleLateral off, or LateralAngleExt fell back this drive)"}
    return {"pa": _num(ext.path_angle_last, 4)}

  # ---------------------------------------------------------------- writer thread
  def _write(self, rec: dict) -> None:
    try:
      rec.update(self._route_fn(rec["t"]))
      self._write_fn(json.dumps(rec, allow_nan=False, separators=(",", ":")))
      self._write_fail = 0
    except Exception:
      self._write_fail += 1
      cloudlog.exception(f"pscmlimlog2pnw: FAILED to write the {rec.get('from')}->{rec.get('to')} record " +
                         f"({self._write_fail} in a row) -- this PSCM limit change is NOT in ces_events")
