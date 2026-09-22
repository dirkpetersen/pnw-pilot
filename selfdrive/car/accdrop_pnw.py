"""accdroplog2pnw -- LOGGING ONLY: make the next unexplained stock-ACC dropout explain itself.

WHY. drives/2026-09-12/central-oregon-weekend/DRIVE_REPORT.md ("Country-road speed drops"): 4 of 22 speed
drops were the Lightning's PCM taking stock ACC from Active (CcStat_D_Actl 5) to Standby (3) with no brake,
no button, no openpilot 0x083 frame and no IPMA cancel/deny bit -- in the messages that analysis decoded.
Several messages carstate ALREADY decodes every frame were never looked at (DesiredTorqBrk.CcDis_B_Cmd,
the ABS module's "cruise disable command" to the PCM; Cluster_Info1_FD1.AccDeny_B_RqIpc; the PSCM's
Lane_Assist_Data3_FD1; EPAS_INFO; IPMA_Data), and the rlog for the minute is only kept by luck.

WHAT. On every change of the stock-ACC state (off / standby / active / fault, from CarState) one
{"ev":"accDrop"} record is appended to /data/pnw/ces_events.jsonl, POST_S after the edge, carrying the
window [-PRE_S, +POST_S] around the edge:
  trace   10 Hz rows (plus the edge tick itself): vEgo, aEgo, steering angle/torque, openpilot's commanded
          curvature, and the capability-listed continuous CAN values (accelerator %, IPMA accel requests).
  chg     every CHANGE of: EVERY <=4-bit status signal of each capability-listed CAN message, at card's full
          100 Hz (not a hand-picked few, so an unexpected cause is visible); the CarState booleans + set speed
          ("cs") and the CarControl engagement/lat/long/cancel/resume bits ("cc" -- latActive without enabled
          is MADS lateral-only), sampled at 10 Hz and on the edge tick; onroadEvents names ("ev", incl.
          laneChange/preLaneChange*/steerOverride) and panda health ("panda") on every new message. Each
          channel gives its state going INTO the window ("in") and then only the signals that changed, with
          times relative to the edge.
  rx      per CAN message, how old its last frame was at the edge.
  tx083   every 0x083 frame card transmitted in the window: bus, raw hex, the button bits set, and what each
          requester was asking for at that tick (see WHO SENDS 0x083 below).
  route / seg   the loggerd route and segment, so that minute's rlog can be found and preserved.

WHO SENDS 0x083. Every frame is built by the Ford carcontroller inside card, on behalf of:
  cancel      CC.cruiseControl.cancel (controlsd)
  resume      CC.cruiseControl.resume (controlsd), or MadsResumeTarget mode "res" (selfdrived, madsresume2pnw)
  SET-        MadsResumeTarget mode "set" with cruise off (selfdrived, gasset2pnw); with cruise on, the
              arbitrated IcbmTarget (selfdrived ces_pnw, ICBM curves) / SpeedAdjustTarget (plannerd, speedadjust2pnw)
  SET+        the same two brains with dir "inc" (guarded restore)
  TJA toggle  the carcontroller itself, whenever the IPMA reports ACCDATA_3.Tja_D_Stat != 0
tx083 records the carcontroller's parsed command for each of those at the tick the frame went out.

RULE 2 (nothing fails silently). A signal that cannot be read is null WITH a reason, never a default:
  * a capability-listed message carstate does not register this session -> "notRegistered" names it and why.
    It is NEVER registered from here: CANParser.vl lazily registers an unknown name ALIVE-CHECKED on
    __getitem__, which would turn canValid False and make the car undriveable. dict.__contains__/dict.get
    do not register, and are the only accessors used on vl.
  * a registered message that has not been received yet -> no rows, "in": null, "inWhy": "never received".
  * a channel whose change buffer overflowed inside the window -> "in": null, "inWhy": "truncated ...".
  * events suppressed by the rate limit are counted into the next record ("suppressed") and warned once.
  * construction/update/write failures are cloudlog'd by the caller (card.py) and by the writer thread here.
  Not logged, by design: CarState.steeringRateDeg (Ford carstate never sets it -- it would read a constant 0;
  differentiate the 10 Hz angle instead).

COST. update() runs once per card tick AFTER sendcan went out: 11 C-level itemgetter calls + tuple compares,
4 capnp reads for the edge test, and 10 Hz / on-new-message samples; nothing is serialized or written in the
control loop. The edge tick copies the buffers (a few hundred tuples); JSON + file append + route lookup run
on a daemon thread. Bounded memory: CHG_MAXLEN tuples per channel, one trace deque, one tx deque. Measured
against card's own CarInterface.update() on a real Lightning rlog in the accdroplog2pnw replay (see commit).

LIMITATION. The record is written POST_S after the edge; if card exits inside that second (ignition off)
the pending record is lost. A dropout at speed is not that case.

CAPABILITY VIEW. Which messages to watch is PnwVehicle.acc_drop_status_msgs / acc_drop_trace (car-specific
DBC names live there, not here). Empty tuples on every car but the Lightning -> card never builds this.
"""
import json
import math
import os
import threading
import time
from collections import deque
from operator import itemgetter

from openpilot.common.swaglog import cloudlog

EV_NAME = "accDrop"
SCHEMA = 1
CES_EVENT_LOG = "/data/pnw/ces_events.jsonl"   # must equal ces_pnw.CES_EVENT_LOG (test-enforced)
PRE_S = 3.0
POST_S = 1.0
TRACE_EVERY = 10              # card ticks per trace row: 100 Hz -> 10 Hz
CHG_MAXLEN = 512              # changes kept per channel (a 50 Hz flicker still covers >5 s)
TX_MAXLEN = 1024              # a cancel is 2 frames/tick at 100 Hz: 1024 covers 5.1 s, more than the 4 s window
MAX_EVENTS_PER_MIN = 10       # a flapping CcStat must not rotate the whole ces_events log away
BUTTON_ADDR = 0x083
SEGMENT_S = 60.0              # loggerd segment length
STALE_ROUTE_S = 75.0         # newest segment older than this at the edge -> loggerd was not recording. Must clear
                             # loggerd's hard fallback rotation at SEGMENT_LENGTH*1.2 = 72 s (loggerd.cc): with a
                             # stalled encoder a RECORDING segment can legitimately run 70-72 s (was 70 -> false stale)

CS_COLS = ("ccEn", "ccAv", "accFault", "brake", "gas", "steerPressed", "standstill", "ccStandstill", "gear",
           "blinkL", "blinkR", "door", "belt", "espOff", "steerFaultTmp", "steerFaultPerm", "sensorsInvalid",
           "canValid", "setMs")
CC_COLS = ("opEn", "latAct", "longAct", "cancel", "resume", "blinkL", "blinkR")
PANDA_COLS = ("nPandas", "ctrlAllowed", "ctrlAllowedLat", "rxInvalid", "txBlocked", "rxChecksInvalid", "hbLost",
              "bus0Off", "bus0Err", "bus0RxLost", "bus0TxLost", "bus2Off", "bus2Err", "bus2RxLost", "bus2TxLost")
TRACE_COLS = ("t", "vEgo", "aEgo", "steerDeg", "steerTq", "curvCmd")


def status_signals(msg) -> tuple:
  """Every small status signal of a DBC message: <= 4 bits, no counters/checksums. Same rule as the drive
  report's rlog_cruise_end.py, so the live record and the offline analysis look at the same signals."""
  return tuple(sorted(k for k, s in msg.sigs.items() if s.size <= 4 and not any(x in k for x in ("Cnt", "_Cs", "No_"))))


def acc_state(cs) -> str:
  if cs.accFaulted:
    return "fault"
  if cs.cruiseState.enabled:
    return "active"
  if cs.cruiseState.available:
    return "standby"
  return "off"


def _cs_vals(cs) -> tuple:
  c = cs.cruiseState
  return (c.enabled, c.available, cs.accFaulted, cs.brakePressed, cs.gasPressed, cs.steeringPressed, cs.standstill,
          c.standstill, str(cs.gearShifter), cs.leftBlinker, cs.rightBlinker, cs.doorOpen, cs.seatbeltUnlatched,
          cs.espDisabled, cs.steerFaultTemporary, cs.steerFaultPermanent, cs.vehicleSensorsInvalid, cs.canValid,
          round(c.speed, 2) if c.speed == c.speed else None)   # NaN != NaN would log a "change" every sample


def _cc_vals(cc) -> tuple:
  return (cc.enabled, cc.latActive, cc.longActive, cc.cruiseControl.cancel, cc.cruiseControl.resume,
          cc.leftBlinker, cc.rightBlinker)


def _panda_vals(pandas) -> tuple:
  if len(pandas) == 0:
    return (0,) + (None,) * (len(PANDA_COLS) - 1)
  p = pandas[0]
  c0, c2 = p.canState0, p.canState2
  return (len(pandas), p.controlsAllowed, p.controlsAllowedLateral, p.safetyRxInvalid, p.safetyTxBlocked,
          p.safetyRxChecksInvalid, p.heartbeatLost, c0.busOff, c0.totalErrorCnt, c0.totalRxLostCnt, c0.totalTxLostCnt,
          c2.busOff, c2.totalErrorCnt, c2.totalRxLostCnt, c2.totalTxLostCnt)


def _cmd_summary(cc_obj, attr):
  """What one 0x083 requester was asking the carcontroller for at this tick. A dict, None (asking nothing), or
  an explicit "n/a:" reason -- never a silent default."""
  if cc_obj is None:
    return "n/a:no carcontroller"
  if not hasattr(cc_obj, attr):
    return f"n/a:carcontroller has no {attr}"
  c = getattr(cc_obj, attr)
  if c is None:
    return None
  return {k: getattr(c, k) for k in ("target_ms", "ceiling_ms", "dir", "mode", "set_ms") if hasattr(c, k)}


def _clean(v):
  """JSON-safe, compact: integral floats -> int, non-finite -> None, enums/other -> str."""
  if v is None or isinstance(v, (bool, int, str)):
    return v
  if isinstance(v, float):
    if not math.isfinite(v):
      return None
    return int(v) if v.is_integer() else round(v, 4)
  if isinstance(v, (list, tuple)):
    return [_clean(x) for x in v]
  if isinstance(v, dict):
    return {k: _clean(x) for k, x in v.items()}
  return str(v)


def append_to_ces_log(line: str) -> None:
  os.makedirs(os.path.dirname(CES_EVENT_LOG), exist_ok=True)
  with open(CES_EVENT_LOG, "a") as f:
    f.write(line + "\n")


def current_route_segment(edge_wall: float) -> dict:
  """loggerd's route (CurrentRoute param) and the segment directory the edge fell in. Nulls carry a reason."""
  out: dict = {"route": None, "seg": None}
  try:
    from openpilot.common.params import Params
    from openpilot.system.hardware.hw import Paths
    route = Params().get("CurrentRoute")
    if isinstance(route, bytes):
      route = route.decode()
    if not route:
      out["routeWhy"] = "CurrentRoute param unset (loggerd not recording?)"
      return out
    out["route"] = route
    root = Paths.log_root()
    segs = []
    with os.scandir(root) as it:
      for e in it:
        if e.name.startswith(route + "--") and e.name[len(route) + 2:].isdigit():
          segs.append((int(e.name[len(route) + 2:]), e.stat().st_ctime))
    if not segs:
      out["segWhy"] = f"no {route}--N directory under {root}"
      return out
    segs.sort()
    # the newest segment directory created at or before the edge (a rotation can land inside POST_S)
    before = [s for s in segs if s[1] <= edge_wall]
    n, ctime = before[-1] if before else segs[0]
    out["seg"] = n
    out["edgeInSegS"] = round(edge_wall - ctime, 1)
    # every rlog the window touches (loggerd segments are SEGMENT_S long) -- the ones to preserve
    keep = {n}
    if out["edgeInSegS"] < PRE_S and n > 0:
      keep.add(n - 1)
    if out["edgeInSegS"] + POST_S > SEGMENT_S:
      keep.add(n + 1)
    out["segsToKeep"] = sorted(keep)
    # loggerd never clears CurrentRoute (loggerd.cc), so when it is stopped -- parknorec2pnw in Park, or anything
    # else -- the param still names the last route. Recording creates a segment directory every SEGMENT_S (at most
    # every 72 s, loggerd's fallback), so a newest one that started longer ago than STALE_ROUTE_S means the edge is
    # in NO rlog. Derived from the directories already read here: no subscription, no dependency on why loggerd stopped.
    out["routeStale"] = out["edgeInSegS"] > STALE_ROUTE_S
    if out["routeStale"]:
      out["routeStaleWhy"] = (f"newest segment started {out['edgeInSegS']} s before the edge (> {STALE_ROUTE_S:.0f} s): " +
                              "loggerd was not recording (parked?) -- this edge is probably in no rlog")
    if not before:
      out["segWhy"] = "every segment directory is newer than the edge (clock step?) -- seg is the oldest"
  except Exception as e:
    out["segWhy"] = f"lookup failed: {type(e).__name__}: {e}"
  return out


class _Chan:
  """Change-only channel: appends (mono, values) only when the value tuple differs from the last one."""
  __slots__ = ("name", "cols", "dq", "last")

  def __init__(self, name: str, cols: tuple):
    self.name, self.cols = name, cols
    self.dq: deque = deque(maxlen=CHG_MAXLEN)
    self.last = None

  def feed(self, t: float, v) -> None:
    if v != self.last:
      self.dq.append((t, v))
      self.last = v


class AccDropLogger:
  def __init__(self, can_parsers: dict, status_msgs: tuple, trace_sigs: tuple, car_controller=None,
               write_fn=append_to_ces_log, route_fn=current_route_segment, threaded: bool = True):
    self._cc_obj = car_controller
    self._write_fn = write_fn
    self._route_fn = route_fn
    self._threaded = threaded
    self.not_registered: dict = {}

    self._parsers = can_parsers
    self._cs = _Chan("cs", CS_COLS)
    self._ccc = _Chan("cc", CC_COLS)
    self._ev = _Chan("ev", ("names",))
    self._panda = _Chan("panda", PANDA_COLS)
    # RESOLVED LATER, not here: card builds this before its first CarInterface.update(), and opendbc's carstate
    # registers the messages it reads lazily, on that first update. Resolving now would find nothing registered
    # and record no CAN at all (the rlog replay caught exactly that). update() resolves on its first call and
    # retries whatever is still unregistered once a second.
    self._unresolved_status = list(status_msgs)
    self._unresolved_trace = list(trace_sigs)
    self._status: list = []       # (chan, getter, vl_inner, ts_inner, first_sig, parser)
    self._trace_can: list = []    # (label, vl_inner, ts_inner, sig)
    self.trace_cols = TRACE_COLS
    self._trace: deque = deque(maxlen=int((PRE_S + POST_S + 1.0) * 100 / TRACE_EVERY))
    self._tx: deque = deque(maxlen=TX_MAXLEN)

    btn = None
    pt = can_parsers.get("pt")
    if pt is not None:
      btn = pt.dbc.addr_to_msg.get(BUTTON_ADDR)
    self._button_msg = btn

    self._frame = 0
    self._state = None
    self._pending: list = []
    self._recent: deque = deque()
    self._suppressed = 0
    self._write_fail = 0

  def _resolve(self) -> None:
    """Bind every capability-listed message carstate has registered by now. Never registers one itself.
    Status messages are retried (a new one is just a new channel); trace columns are fixed at the first call so
    trace rows never go ragged. Logs on the first call and whenever something newly binds -- not every retry."""
    first = self._frame == 1
    n_before = len(self._status)
    still = []
    for bus, name in self._unresolved_status:
      label = f"{bus}:{name}"
      p = self._parsers.get(bus)
      if p is None:
        self.not_registered[label] = f"no '{bus}' CAN parser on this car"
        continue                                         # permanent: no retry
      if not dict.__contains__(p.vl, name):              # NEVER p.vl[name]: that registers it alive-checked
        self.not_registered[label] = ("not registered by opendbc carstate this session: carstate never decodes it " +
                                      "(needs an opendbc decode + pin bump), or skips it in this mode (ACCDATA under op-long)")
        still.append((bus, name))
        continue
      sigs = status_signals(p.dbc.name_to_msg[name])
      if not sigs:
        self.not_registered[label] = "message has no <=4-bit status signals"
        continue
      self.not_registered.pop(label, None)
      getter = itemgetter(*sigs) if len(sigs) > 1 else (lambda d, s=sigs[0]: (d[s],))
      self._status.append((_Chan(label, sigs), getter, dict.get(p.vl, name), p.ts_nanos[name], sigs[0], p))
    self._unresolved_status = still
    if first:
      for bus, name, sig in self._unresolved_trace:
        p = self._parsers.get(bus)
        if p is None or not dict.__contains__(p.vl, name):
          self.not_registered[f"{bus}:{name}.{sig}"] = "trace signal's message not registered by opendbc carstate this session"
          continue
        self._trace_can.append((f"{name}.{sig}", dict.get(p.vl, name), p.ts_nanos[name], sig))
      self._unresolved_trace = []
      self.trace_cols = TRACE_COLS + tuple(x[0] for x in self._trace_can)
    if first or len(self._status) != n_before:
      cloudlog.warning(f"accdroplog2pnw: {len(self._status)} status messages recorded, trace {self.trace_cols}; " +
                       f"NOT recorded: {sorted(self.not_registered)}")

  # ---------------------------------------------------------------- 100 Hz, card loop
  def update(self, cs, sm, can_sends, now: float) -> None:
    # COST SPLIT (measured, see test_update_cost_is_bounded): a CarState/CarControl field is a pycapnp read,
    # several microseconds each, while a CAN status message is one C-level itemgetter over a dict. So the CAN
    # status channels and the edge test run every tick; the CarState/CarControl channels and the trace run at
    # 10 Hz and on the edge tick itself; onroadEvents/pandaStates only when a new message arrived.
    self._frame += 1
    if self._frame == 1 or (self._unresolved_status and self._frame % 100 == 0):
      self._resolve()
    for chan, getter, vl, ts, s0, _p in self._status:
      if ts[s0]:                                         # 0 until the first frame: no fake zeros in the log
        chan.feed(now, getter(vl))

    for c in can_sends:
      if c[0] == BUTTON_ADDR:
        cc = sm['carControl']
        self._tx.append((now, c[2], bytes(c[1]), cc.cruiseControl.cancel, cc.cruiseControl.resume,
                         _cmd_summary(self._cc_obj, "_icbm_cmd"), _cmd_summary(self._cc_obj, "_sa_cmd"),
                         _cmd_summary(self._cc_obj, "_resume_cmd")))

    st = acc_state(cs)
    # Not seeded until CAN is valid: before the first decode every car reads "off", which made one spurious
    # off->active record per card start (Fable). After seeding, every change counts.
    if self._state is None and not cs.canValid:
      st = None
    edge = self._state is not None and st != self._state
    if edge or self._frame % TRACE_EVERY == 0:
      cc = sm['carControl']
      self._cs.feed(now, _cs_vals(cs))
      self._ccc.feed(now, _cc_vals(cc))
      row = [now, cs.vEgo, cs.aEgo, cs.steeringAngleDeg, cs.steeringTorque, cc.actuators.curvature]
      for _label, vl, ts, sig in self._trace_can:
        row.append(vl[sig] if ts[sig] else None)
      self._trace.append(row)
    if sm.updated['onroadEvents']:
      # capnpfork2pnw: the fork's events (cruiseOffRequested, madsLateralOnly, ...) left onroadEvents for
      # onroadEventsPnw. selfdrived sends that one FIRST in the same frame, so it is already current here.
      names = [str(e.name) for e in sm['onroadEvents']] + [str(e.name) for e in sm['onroadEventsPnw'].events]
      self._ev.feed(now, (tuple(sorted(names)),))
    if sm.updated['pandaStates']:
      self._panda.feed(now, _panda_vals(sm['pandaStates']))
    if edge:
      self._on_edge(now, f"{self._state}->{st}")
    self._state = st

    while self._pending and now - self._pending[0]["mono"] >= POST_S:
      self._emit(self._pending.pop(0))

  def _on_edge(self, now: float, edge: str) -> None:
    while self._recent and now - self._recent[0] > 60.0:
      self._recent.popleft()
    if len(self._recent) >= MAX_EVENTS_PER_MIN:
      self._suppressed += 1
      if self._suppressed == 1:
        cloudlog.warning(f"accdroplog2pnw: >{MAX_EVENTS_PER_MIN} ACC state changes in 60 s -- suppressing records " +
                         f"(counted into the next one); latest {edge}")
      return
    self._recent.append(now)
    rx = {}
    for chan, _g, _vl, ts, s0, p in self._status:
      last = getattr(p, "_last_update_nanos", None)
      if not ts[s0]:
        rx[chan.name] = {"ageMs": None, "why": "never received"}
      elif last is None:
        rx[chan.name] = {"ageMs": None, "why": "parser has no _last_update_nanos"}
      else:
        rx[chan.name] = {"ageMs": round((last - ts[s0]) / 1e6)}
    self._pending.append({"mono": now, "wall": time.time(), "edge": edge, "rx": rx,  # noqa: TID251 -- log stamp
                          "suppressed": self._suppressed})
    self._suppressed = 0

  def _emit(self, p: dict) -> None:
    chans = [self._cs, self._ccc, self._ev, self._panda] + [s[0] for s in self._status]
    snap = {
      "trace": list(self._trace),
      "chans": [(c.name, c.cols, list(c.dq), c.dq.maxlen) for c in chans],
      "tx": list(self._tx),
      "txMaxlen": self._tx.maxlen,
      "notRegistered": dict(self.not_registered),   # copied HERE: the 1 Hz retry in update() may change it mid-write
    }
    if self._threaded:
      threading.Thread(target=self._write_event, args=(p, snap), daemon=True, name="accdroplog").start()
    else:
      self._write_event(p, snap)

  # ---------------------------------------------------------------- writer thread
  def _write_event(self, p: dict, snap: dict) -> None:
    try:
      rec = self.build_record(p, snap)
      rec.update(self._route_fn(p["wall"]))
      self._write_fn(json.dumps(_clean(rec), allow_nan=False, separators=(",", ":")))
      self._write_fail = 0
    except Exception:
      self._write_fail += 1
      cloudlog.exception(f"accdroplog2pnw: FAILED to write the {p.get('edge')} record ({self._write_fail} in a row) " +
                         "-- this ACC state change is NOT explained in ces_events")

  def build_record(self, p: dict, snap: dict) -> dict:
    t_edge = p["mono"]
    t0, t1 = t_edge - PRE_S, t_edge + POST_S

    def rel(t):
      return round(t - t_edge, 2)

    chg = {}
    for name, cols, entries, maxlen in snap["chans"]:
      before, rows = None, []
      for t, v in entries:
        if t <= t0:
          before = v
        elif t <= t1:
          rows.append((t, v))
      ch: dict = {"cols": list(cols)}
      if before is not None:
        ch["in"] = dict(zip(cols, before, strict=True))
      else:
        ch["in"] = None
        if not entries:
          ch["inWhy"] = "never received" if name in p["rx"] else "no sample yet"
        elif len(entries) >= maxlen:
          ch["inWhy"] = f"truncated: more than {maxlen} changes buffered, oldest is inside the window"
        else:
          ch["inWhy"] = f"first sample (logger start or first frame) at {rel(entries[0][0])} s from the edge"
      prev = before
      out_rows = []
      for t, v in rows:
        diff = dict(zip(cols, v, strict=True)) if prev is None else {c: x for c, x, o in zip(cols, v, prev, strict=True) if x != o}
        out_rows.append([rel(t), diff])
        prev = v
      ch["rows"] = out_rows
      del ch["cols"]
      chg[name] = ch

    trace = [[rel(r[0])] + r[1:] for r in snap["trace"] if t0 <= r[0] <= t1]
    tx = []
    try:
      from opendbc.can.parser import get_raw_value
      bit_names = sorted(k for k, s in self._button_msg.sigs.items() if s.size == 1) if self._button_msg is not None else None
    except Exception as e:
      get_raw_value, bit_names = None, None
      tx_why = f"button decode unavailable: {type(e).__name__}"
    else:
      tx_why = None if bit_names is not None else "no 0x083 message in this car's pt DBC"
    for t, bus, dat, cancel, resume, icbm, sa, res in snap["tx"]:
      if t0 <= t <= t1:
        bits = [k for k in bit_names if get_raw_value(dat, self._button_msg.sigs[k])] if bit_names is not None else None
        tx.append({"t": rel(t), "bus": bus, "hex": dat.hex(), "bits": bits, "ccCancel": cancel, "ccResume": resume,
                   "icbm": icbm, "sa": sa, "resume": res})

    rec = {
      "t": round(p["wall"], 2), "ev": EV_NAME, "v": SCHEMA, "edge": p["edge"],
      "window": [-PRE_S, POST_S],
      "trace": {"cols": ["dt"] + list(self.trace_cols[1:]), "rows": trace},
      "chg": chg,
      "rx": p["rx"],
      "notRegistered": snap["notRegistered"],
      "tx083": tx,
    }
    if len(snap["tx"]) >= snap["txMaxlen"] and snap["tx"][0][0] > t0:
      trunc = (f"truncated: the {snap['txMaxlen']}-frame buffer's oldest frame is {rel(snap['tx'][0][0])} s from " +
               "the edge, so earlier frames in the window were dropped")
      tx_why = f"{tx_why}; {trunc}" if tx_why else trunc
    if tx_why:
      rec["tx083Why"] = tx_why
    if p["suppressed"]:
      rec["suppressed"] = p["suppressed"]
    return rec
