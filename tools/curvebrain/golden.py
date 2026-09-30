#!/usr/bin/env python3
"""curvebrain2pnw 0/8 -- the GOLDEN REPLAY of ICBM's `_icbm_step` (docs/SHARED-CURVE-BRAIN-DESIGN.md s4).

WHY. Stage 2 moves ICBM's whole pricing chain out of ces_pnw.py into curve_brain.py, and every later stage adds to it.
The Lightning must stay BYTE-IDENTICAL through all of it. This harness is the proof: it drives a REAL CESController
(the same construction selfdrived uses, ShapeStage and the live curve DB included) through `_icbm_step` over four
corpora, and writes one canonical JSON line per ICBM tick:

  pub   the IcbmTarget payload `_icbm_step` published ({} = the empty publish, "skip" = nothing was published: its
        0.25 s throttle returned, or the step failed -- the tick's `log` then carries the exception)
  rec   every `_event_record("tick")` field named icbm* / cdb2* / shp* -- which includes curvelead2pnw
        (CURVELEAD_TELE_KEYS) and restorehold2pnw (icbmRHold*); both key sets are also added by name
  st    the UNROUNDED state behind them: every JSON-able `_icbm_*` attribute of the controller and every JSON-able
        attribute of its IcbmEpisode. The published fields are rounded (2 dp), so without this a 1e-9 change -- the
        self-test's ICBM_MARGIN_M mutation -- would be invisible, and so would a real drift of that size.
  log   every cloudlog call made on the main thread during the tick (event/info/warning/error/exception), in order.
        A swallowed exception is therefore IN the golden, never silently absent (Rule 2): `_icbm_step` catches its
        own failures, and a golden recorded over a broken step would otherwise look like a quiet road.

Canonical form: json.dumps(sort_keys, compact separators); floats go through float.__repr__ (the json module's own
float rendering), so every bit of a double is in the text. One sha256 per corpus over the exact bytes written.

THE CORPORA (design s4):
  1  the cached Lightning ticks (drives/2026-09-24/curveshape-replay/ticks.jsonl): the near-map, shape and penalty
     paths. The cache has NO vision fields, no vSet, no mapd path: those sig keys are left out (the code's own
     defaults apply), never invented. One `_icbm_step` per logged ~1 Hz tick.
  2  Lightning ces_events ticks since 2026-09-24 00:00 PT (the S3 pnwlogs archive, synced read-only), with mapd's
     path attached from the `mapdPath` records (mapdpathlog2pnw) where there are any. The full records carry
     vision, lead and speedadjust fields, so this corpus also covers the vision / lead-pace paths corpus 1 cannot.
     The report says how many ticks had a path -- ZERO is reported as zero, not as coverage.
  3  the private curve-DB replay fixture (test_curvedblive2pnw_replay): each episode's real geometry driven at 4 Hz
     with that episode's own leave-one-date-out rows -- the DB decide / hidden-curve / add paths on real roads.
  4  seeded fuzz: synthetic roads (straights, arcs, S-bends, phantom ratings, hidden second curves) with a synthetic
     curve DB along them, the whole car loop at 4 Hz (mapd refresh + _read_map at 1 Hz, an executor that taps the
     stock set toward the published target, driver gas/brake/set changes, leads, Chill interludes), over the matrix
     shape off/shadow/live x DB on/off x rain 0/1/2 x GPS proj/stale/raw/none, round-robin so every cell is hit.

COVERAGE (design s4): ticks per icbmSrc / icbmGate / cdb2Why / cdb2Dir / shpWhy / icbmPhase, against the universe of
values the code at the base commit can produce (UNIVERSE, read off the source). Every value hit fewer than MIN_HITS
times -- including never -- is LISTED. A value seen that is not in UNIVERSE is listed too: the universe is stale.

OUTPUT holds positions (the corpora are real drives). It goes to ~/gh/comma/drives/2026-09-27/curvebrain-golden/ by
default and NEVER into this repository.

  record   run every corpus, write <corpus>.jsonl.zst + manifest.json + coverage.txt into --out
  check    re-run and compare line by line with a recorded set (--golden); --exclude drops named rec keys
           (a later stage's new columns); --mutate NAME=DELTA perturbs a module constant first (the self-test)
Run from a pnw-pilot worktree:  PYTHONPATH=$PWD:$PWD/opendbc_repo python tools/curvebrain/golden.py record
Exit: 0 ok; 1 check found differences / a gate is not met; 2 the harness itself failed; 3 a corpus input is missing.
"""
from __future__ import annotations

import argparse
import contextlib
import copy
import glob
import gzip
import hashlib
import io
import json
import math
import os
import random
import shutil
import subprocess
import sys
import tempfile
import threading
import time as _real_time
from collections import Counter, defaultdict
from datetime import UTC, datetime
from zoneinfo import ZoneInfo

import zstandard

from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
from openpilot.selfdrive.controls.lib.ces_pnw import curve_brain as cb
from openpilot.selfdrive.controls.lib.ces_pnw import curvedb_live as cl
from openpilot.selfdrive.controls.lib.ces_pnw import curvedb_shadow as cds

PT = ZoneInfo("America/Los_Angeles")
MPH = 0.44704
WB = os.path.expanduser("~/gh/comma")
OUT = os.path.join(WB, "drives/2026-09-27/curvebrain-golden")
TICKS_CACHE = os.path.join(WB, "drives/2026-09-24/curveshape-replay/ticks.jsonl")
PNWLOGS = os.path.join(OUT, "pnwlogs")          # `aws --profile dipeit s3 sync` of the device's pnwlogs (read-only)
FIXTURE = os.environ.get("CURVEDB_V2_REPLAY_FIXTURE",
                         os.path.join(WB, "_scratch/curvedb-v2/live/replay_fixture.json.gz"))
# ovrcar2pnw: the per-curve override file the Lightning's controller reads. None = an EMPTY valid v2 file (no overrides, and no
# fail-safe error line in the replay); `check --overrides PATH` runs the replay with a real file (e.g. the private seed).
OVERRIDES_FILE = None
DB_DIR = os.path.join(WB, "_scratch/curvedb-v2/live-20260924")   # the table deployed 2026-09-24 (19,924 rows)
CORPUS2_SINCE = datetime(2026, 9, 24, tzinfo=PT).timestamp()
LIGHTNING = "FORD_F_150_LIGHTNING_MK1"
A_MAPD = 2                     # the truck's mapd A, top-level MapdSettings (DEVICE-VERIFIED 2026-09-24), an int there
SEGMENT_GAP_S = 30.0           # logged ticks further apart than this start a new controller (a new selfdrived, roughly)
MIN_MOVING_MS = 4.0            # corpora 1/2: moving ticks only (the cache's own rule)
FUZZ_FRAMES = 100_000
FUZZ_SEED = 20260927
DT = 0.25                      # ICBM's own cadence
MIN_HITS = 20
CORPORA = ("c1_cache", "c2_pnwlogs", "c3_fixture", "c4_fuzz")

REC_PREFIXES = ("icbm", "cdb2", "shp")
REC_NAMED = set(m.CURVELEAD_TELE_KEYS) | {"icbmRHold", "icbmRHoldA", "icbmRHoldV"}

# The values the base commit's code can produce, read off the source (dcaf2bbe41). str() of each; None is "None".
UNIVERSE = {
  "icbmSrc": ["None", "map", "far", "vis", "restore", "gpsHold"],
  # "visLate" needs a NaN time_to_curve: a vision candidate needs |lat| > ICBM_VISION_ENTER, and with ttc below
  # ICBM_VIS_MIN_TTC_S that same reading makes icbm_in_curve True, so the inCurve gate always fires first.
  "icbmGate": ["None", "inCurve", "visCovered", "visLate", "mapPassed", "mapPassedRun", "mapFalsified"],
  # CurveDbLive.gate / _decide / match_at / tele. "dbLoading" is unreachable here by construction (the harness loads
  # the DB synchronously); "crash" only by a defect in _decide; "noCandidatePoint" only when ICBM's candidate distance
  # names no mapd node within CURVE_CAND_TOL_M (a logged candidate against a path logged at another moment -- corpus 2
  # once it has paths); "noRow" only when ICBM's own map/far source has no candidate of its own in the DB's re-derivation
  # (no input found that does it).
  "cdb2Why": ["off", "idle", "dbLoading", "dbErr", "noA", "inCurveStart", "noGps", "waySel", "class", "noPosted",
              "noPath", "held", "notMin", "noRow", "notMap", "ok", "crash", "noCandidatePoint", "noAnchor",
              "branchUnknown", "branchAmbiguous", "noAuthority"],
  "cdb2Dir": ["None", "none", "raise", "lower", "add"],
  # ShapeStage.tick / tick_gate / shape_price / tele, plus the config why of a disabled stage -- on the Lightning only
  # "curve.json" (the default mode is shadow, so a disabled stage always came from the file). "err" only by a defect
  # in the pricing; "badInput" only for a candidate with no positive rating / price, which ICBM never hands it.
  "shpWhy": ["curve.json", "idle", "gps", "waySel", "class", "noA", "noCand", "badInput", "rawLow", "sparse",
             "notAhead", "near", "farApart", "unstable", "mapSharper", "polySharper", "noPosted", "ok", "err"],
  "icbmPhase": ["idle", "cap", "restore", "gas"],
}


class _Patch:
  """setattr for the length of a run, restored exactly after (an attribute that lived on the class is deleted from the
  instance again). A context manager, or start()/stop()."""

  def __init__(self, obj, name, val):
    self.obj, self.name, self.val = obj, name, val

  def start(self):
    self.own = name_in = self.name in vars(self.obj)
    self.old = vars(self.obj)[self.name] if name_in else None
    setattr(self.obj, self.name, self.val)
    return self

  def stop(self):
    if self.own:
      setattr(self.obj, self.name, self.old)
    else:
      delattr(self.obj, self.name)

  def __enter__(self):
    return self.start()

  def __exit__(self, *exc):
    self.stop()
    return False


class HarnessError(RuntimeError):
  """The harness could not do its job. Never a result."""


class MissingInput(HarnessError):
  """A corpus input is absent (exit 3): reported, never read as an empty corpus."""


# ---------------------------------------------------------------------------------------------------------------------
# canonical lines
# ---------------------------------------------------------------------------------------------------------------------
def _dflt(o):
  if hasattr(o, "item"):         # a numpy scalar -> its Python value
    return o.item()
  raise TypeError(f"golden: {type(o).__name__} is not serialisable -- extend the harness, do not repr() it")


def canon(obj) -> str:
  """One canonical line. float.__repr__ for floats (the json module's own), NaN/Infinity kept as tokens."""
  return json.dumps(obj, sort_keys=True, separators=(",", ":"), allow_nan=True, default=_dflt)


_SKIP = object()


def _jsonable(v, depth=0):
  if v is None or isinstance(v, (bool, int, float, str)):
    return v
  if depth > 3:
    return _SKIP
  if isinstance(v, (list, tuple)):
    out = [_jsonable(x, depth + 1) for x in v]
    return _SKIP if any(x is _SKIP for x in out) else out
  if isinstance(v, dict) and all(isinstance(k, str) for k in v):
    out = {k: _jsonable(x, depth + 1) for k, x in v.items()}
    return _SKIP if any(x is _SKIP for x in out.values()) else out
  if hasattr(v, "item") and not hasattr(v, "__len__"):   # numpy scalar
    return v.item()
  return _SKIP


def _scalars(d: dict) -> dict:
  out = {}
  for k in sorted(d):
    j = _jsonable(d[k])
    if j is not _SKIP:
      out[k] = j
  return out


def state_of(ctl) -> dict:
  """The unrounded ICBM state: JSON-able `_icbm_*` controller attributes and the episode's own attributes."""
  ep = getattr(ctl, "_icbm_ep", None)
  return {"ctl": _scalars({k: v for k, v in vars(ctl).items() if k.startswith("_icbm_")}),
          "ep": _scalars(vars(ep)) if ep is not None else None}


def pick(rec: dict) -> dict:
  return {k: v for k, v in rec.items() if k.startswith(REC_PREFIXES) or k in REC_NAMED}


class Sink:
  """Writes canonical lines (zstd) and hashes exactly the bytes written. path None: hash only. `scrub`: the run's
  temporary directory, replaced by "<tmp>" -- it reaches the text through error strings (cdb2Err "missing <dir>/...",
  a config path in a start event), and a per-run random name would make two identical runs differ."""

  def __init__(self, path=None, scrub=None):
    self.path, self.h, self.n, self.scrub = path, hashlib.sha256(), 0, scrub
    self._fh = self._z = None
    if path is not None:
      self._fh = open(path, "wb")
      self._z = zstandard.ZstdCompressor(level=10).stream_writer(self._fh)

  def write(self, line: dict) -> None:
    text = canon(line)
    if self.scrub:
      text = text.replace(self.scrub, "<tmp>")
    b = (text + "\n").encode()
    self.h.update(b)
    self.n += 1
    if self._z is not None:
      self._z.write(b)

  def close(self) -> str:
    if self._z is not None:
      self._z.close()           # also closes the file
    return self.h.hexdigest()


def read_golden(path):
  with open(path, "rb") as fh:
    for ln in io.TextIOWrapper(zstandard.ZstdDecompressor().stream_reader(fh), encoding="utf-8"):
      yield ln.rstrip("\n")


# ---------------------------------------------------------------------------------------------------------------------
# the environment: replay clock, isolated paths, captured logs
# ---------------------------------------------------------------------------------------------------------------------
class _Clock:
  """Stands in for the `time` module inside ces_pnw / ces_pnw_constants / curvedb_live: monotonic() and time() return
  the replay clock; everything else is the real module."""

  def __init__(self):
    self.mono, self.wall = 0.0, 0.0

  def set(self, mono, wall=None):
    self.mono = float(mono)
    self.wall = float(mono if wall is None else wall)

  def monotonic(self):
    return self.mono

  def time(self):
    return self.wall

  def __getattr__(self, name):
    return getattr(_real_time, name)


class FakeCP:
  carFingerprint = LIGHTNING
  brand = "ford"
  openpilotLongitudinalControl = False
  dashcamOnly = False


class FakeParams:
  """CESController reads its Params only in _read_params, which the harness never calls."""

  def get(self, k, return_default=False):
    return None

  def get_bool(self, k):
    return False


_NOPUB = "skip"


class FakeMem:
  """The /dev/shm mem-param store: `vals` feeds _read_map, the IcbmTarget publish is captured."""

  def __init__(self):
    self.vals: dict = {}
    self.pub = _NOPUB

  def get(self, k, return_default=False):
    return self.vals.get(k)

  def put_nonblocking(self, k, v):
    if k == "IcbmTarget":
      self.pub = copy.deepcopy(v)


def _reader_ok():
  return {"map_curve_target_lat_a": A_MAPD}, b"1"


def _reader_broken():
  return None, None          # MapdSettings absent -> A unreadable (curve DB with lat_a 0, shape stage: noA)


class Env:
  """Everything the replay patches, restored on exit. Construct once per run."""

  def __init__(self):
    self.clock = _Clock()
    self.logs: list = []
    self.reader = _reader_ok
    self._main = threading.get_ident()
    self._rows_cache: dict = {}
    self._n = 0
    self.thread_logs = 0          # cloudlog calls from other threads: counted, not replayed
    self.tmp = None
    self._stack = None

  def _log(self, level):
    """Every main-thread cloudlog call, in order: the next canonical line carries what was logged since the last one
    (a controller's construction lands in its `seg` line, a _read_map refresh in the tick after it)."""
    def f(msg=None, *a, **kw):
      if threading.get_ident() != self._main:
        self.thread_logs += 1     # the curvedb_shadow loader thread: its timing is not the replay's
        return
      if level == "event":
        entry = ["event", msg, kw]
      elif level == "exception":
        entry = ["exception", str(msg), type(sys.exc_info()[1]).__name__]
      else:
        entry = [level, str(msg % a) if a else str(msg)]
      self.logs.append(entry)
    return f

  def _load_rows(self, data_dir):
    key = os.path.abspath(data_dir)
    got = self._rows_cache.get(key)
    if got is None:
      got = self._rows_cache[key] = self._orig_load_rows(data_dir)   # raises exactly as the original does
    return got

  def __enter__(self):
    self.tmp = tempfile.mkdtemp(prefix="curvebrain-golden-")
    st = self._stack = contextlib.ExitStack()

    def p(obj, name, val):
      st.enter_context(_Patch(obj, name, val))
    for mod in (m, C, cl):
      p(mod, "time", self.clock)
    p(pv, "CURVE_CONFIG_PATH", os.path.join(self.tmp, "absent-curve.json"))
    p(pv, "RAIN_CONFIG_PATH", os.path.join(self.tmp, "absent-rain.json"))
    ovr = OVERRIDES_FILE
    if ovr is None:
      ovr = os.path.join(self.tmp, "curve_overrides.json")
      with open(ovr, "w") as fh:
        fh.write('{"version": 2, "overrides": []}')
    p(cb, "OVERRIDES_PATH", ovr)
    p(m, "CES_EVENT_LOG", os.path.join(self.tmp, "ces_events.jsonl"))
    p(cl, "BACKGROUND", [False])                 # load + read A in the constructor: no thread, no race
    p(cl, "A_MAX_AGE_S", float("inf"))           # the device's poll thread keeps A fresh; there is no thread here
    p(cl, "READ_PARAMS", [lambda: self.reader()])
    self._orig_load_rows = cl.load_rows
    p(cl, "load_rows", self._load_rows)          # the same immutable index, loaded once per directory
    p(cds, "OBS_PATH", os.path.join(self.tmp, "curvedb_obs.jsonl"))
    p(cds, "CONFIG_PATH", os.path.join(self.tmp, "curvedb.json"))
    for level in ("event", "info", "warning", "error", "exception"):
      p(cloudlog, level, self._log(level))
    return self

  def __exit__(self, *exc):
    self._stack.close()
    shutil.rmtree(self.tmp, ignore_errors=False)
    return False

  def take_logs(self) -> list:
    out, self.logs = self.logs, []
    return out

  # -- one controller ---------------------------------------------------------------------------------------------
  def build(self, cfg: dict, t0: float, wall0: float | None = None):
    """A real CESController for the Lightning. cfg keys (all optional): shape "off"/"shadow"/"live" (None = no
    curve.json key: the shipped default), db (bool, default on), db_dir, lat_a (curve.json curvedb_v2_lat_a; None =
    the shipped 2.5), rain 0/1/2, reader "ok"/"broken"."""
    self._n += 1
    light = {}
    if cfg.get("shape") is not None:
      light["icbm_shape"] = cfg["shape"]
    if cfg.get("db") is False:
      light["curvedb_v2_live"] = 0
    if cfg.get("lat_a") is not None:
      light["curvedb_v2_lat_a"] = cfg["lat_a"]
    path = os.path.join(self.tmp, "absent-curve.json")
    if light:
      path = os.path.join(self.tmp, f"curve-{self._n}.json")
      with open(path, "w") as f:
        json.dump({"lightning": light}, f)
    self.reader = _reader_broken if cfg.get("reader") == "broken" else _reader_ok
    self.clock.set(t0, wall0)
    self.logs = []
    with _Patch(pv, "CURVE_CONFIG_PATH", path), _Patch(cl, "DATA_DIR", cfg.get("db_dir") or DB_DIR):
      ctl = m.CESController(FakeCP(), params=FakeParams())
    ctl.mem_params = FakeMem()
    ctl._event_log_ok = False                        # nothing reaches CES_EVENT_LOG ...
    ctl._append_event = lambda rec: None              # ... and the mapdPath log has nowhere to go
    ctl._veh.set_rain_tier(cfg.get("rain", 0))
    ctl._str_ang = ctl._str_prs = None                # set per carState in experimental_request, never called here
    return ctl

  # -- one ICBM tick ----------------------------------------------------------------------------------------------
  def tick(self, ctl, mono: float, sig: dict, active: bool, wall: float | None = None) -> dict:
    self.clock.set(mono, wall)
    ctl.mem_params.pub = _NOPUB
    ctl._icbm_step(sig, active)
    rec = ctl._event_record("tick", {"vEgo": sig.get("v_ego")})
    return {"pub": ctl.mem_params.pub, "rec": pick(rec), "st": state_of(ctl), "log": self.take_logs()}


# ---------------------------------------------------------------------------------------------------------------------
# coverage + accounting
# ---------------------------------------------------------------------------------------------------------------------
class Coverage:
  def __init__(self):
    self.hits = {c: {f: Counter() for f in UNIVERSE} for c in CORPORA}
    self.acct = {c: Counter() for c in CORPORA}

  def add(self, corpus, line):
    rec = line["rec"]
    for f in UNIVERSE:
      self.hits[corpus][f][str(rec.get(f))] += 1
    a = self.acct[corpus]
    a["ticks"] += 1
    if line["pub"] == _NOPUB:
      a["no IcbmTarget publish (throttle or a failed step)"] += 1
    for lv in line["log"]:
      if lv[0] in ("error", "exception"):
        a[f"log {lv[0]}: {lv[1][:70]}"] += 1

  def total(self, f) -> Counter:
    t = Counter()
    for c in CORPORA:
      t.update(self.hits[c][f])
    return t

  def under(self) -> list:
    """(field, value, hits) for every universe value under MIN_HITS over all corpora, and every unknown value."""
    out = []
    for f, uni in UNIVERSE.items():
      t = self.total(f)
      for v in uni:
        if t[v] < MIN_HITS:
          out.append((f, v, t[v], ""))
      for v, n in sorted(t.items()):
        if v not in uni:
          out.append((f, v, n, "NOT IN UNIVERSE -- the universe is stale, update UNIVERSE"))
    return out

  def report(self, shas=None, notes=None) -> str:
    L = ["# curvebrain golden coverage", ""]
    for c in CORPORA:
      a = self.acct[c]
      L.append(f"## {c}: {a['ticks']} ticks" + (f"  sha256 {shas[c]}" if shas and c in shas else ""))
      for k, v in sorted(a.items()):
        if k != "ticks":
          L.append(f"  {k}: {v}")
      for f in UNIVERSE:
        h = self.hits[c][f]
        L.append(f"  {f}: " + ", ".join(f"{v} {n}" for v, n in sorted(h.items(), key=lambda x: -x[1])))
      L.append("")
    L.append(f"## ALL CORPORA -- every value hit fewer than {MIN_HITS} times is listed (never silently accepted)")
    for f in UNIVERSE:
      t = self.total(f)
      L.append(f"  {f}: " + ", ".join(f"{v} {t[v]}" for v in UNIVERSE[f]) +
               ("" if all(v in UNIVERSE[f] for v in t) else "  + unknown: " +
                ", ".join(f"{v} {n}" for v, n in t.items() if v not in UNIVERSE[f])))
    under = self.under()
    L.append("")
    L.append(f"UNDER {MIN_HITS}: {len(under)}")
    for f, v, n, why in under:
      L.append(f"  {f}={v}: {n}" + (f"  ({why})" if why else ""))
    if notes:
      L.append("")
      L.append("## notes")
      L.extend(f"  {n}" for n in notes)
    return "\n".join(L) + "\n"


# ---------------------------------------------------------------------------------------------------------------------
# logged records -> controller inputs (corpora 1 and 2)
# ---------------------------------------------------------------------------------------------------------------------
def f(x):
  try:
    x = float(x)
  except (TypeError, ValueError):
    return None
  return x if math.isfinite(x) else None


def sig_from_record(r: dict) -> dict:
  """The sig keys a record actually logged. A key that was not logged is LEFT OUT (the code's own .get default
  applies) -- never invented."""
  v = f(r.get("vEgo"))
  map_v = f(r.get("mapV")) or 0.0
  sig = {"v_ego": v, "v_set": f(r.get("vSet")) or f(r.get("stockSet")) or 0.0,
         "map_target_v": map_v, "map_target_dist": (f(r.get("mapDist")) if map_v > 0.0 else None),
         "spd_lim": f(r.get("spdLim")) or 0.0, "gas": r.get("gas") is True}
  if sig["map_target_dist"] is None:
    sig["map_target_dist"] = float("inf")
  lead = r.get("hasLead", r.get("lead"))
  if lead is not None:
    sig["has_lead"] = lead is True
  for key, src in (("lead_vlead", "vLead"), ("lead_drel", "dRel"), ("curve_lat_accel_vision", "visLat"),
                   ("time_to_curve", "visTtc"), ("vis_k_max", "visKMax"), ("vis_reach", "visKRch")):
    if src in r and r[src] is not None:
      sig[key] = f(r[src])
  return sig


def apply_record(ctl, r: dict, now: float, has_k: bool, path=None) -> None:
  """What _read_map would have left on the controller when this record was written."""
  ctl._cur_lat, ctl._cur_lon, ctl._cur_bearing = f(r.get("lat")), f(r.get("lon")), f(r.get("bearing"))
  ctl._gps_fix_ts = None          # no fix age is logged: ICBM uses the position unprojected ("raw"), as logged
  ctl._map_targets = path or []
  ctl._speed_limit = f(r.get("spdLim")) or 0.0
  ctl._hwy_class = r.get("hwyClass")
  ctl._way_sel = r.get("waySel")
  if has_k:
    ctl._icbm_k, ctl._icbm_k_dist = f(r.get("icbmK")) or 0.0, f(r.get("icbmKD")) or 0.0
    ctl._icbm_k_v, ctl._icbm_k_n = f(r.get("icbmKV")) or 0.0, int(f(r.get("icbmKN")) or 0)
    ctl._icbm_k_ahead = r.get("icbmKAhead") is not False
    ctl._icbm_k_t = now
  else:
    ctl._icbm_k = ctl._icbm_k_dist = ctl._icbm_k_v = 0.0
    ctl._icbm_k_n, ctl._icbm_k_ahead, ctl._icbm_k_t = 0, True, None
  ctl._stock_set = f(r.get("stockSet")) or 0.0
  ctl._stock_on = r.get("stockOn") is True
  ctl._sl_k_actl = f(r.get("slKActl"))
  ctl._sa_tele = {k: v for k, v in r.items() if k.startswith("sa") and len(k) > 2 and k[2].isupper()}


def _segments(ticks):
  seg, prev = [], None
  for r in ticks:
    if prev is not None and (r["_src"] != prev["_src"] or r["t"] - prev["t"] > SEGMENT_GAP_S):
      yield seg
      seg = []
    seg.append(r)
    prev = r
  if seg:
    yield seg


def _run_logged(env, sink, cov, name, ticks, active_of, has_k_of, path_of=None):
  for seg in _segments(ticks):
    t0 = seg[0]["t"]
    ctl = env.build({}, t0 - 1.0)
    sink.write({"seg": seg[0]["_src"], "t0": t0, "log": env.take_logs()})
    for r in seg:
      now = r["t"]
      path = path_of(r) if path_of else None
      apply_record(ctl, r, now, has_k_of(r), path)
      line = env.tick(ctl, now, sig_from_record(r), active_of(r))
      line["t"] = now
      sink.write(line)
      cov.add(name, line)
      if path:
        cov.acct[name]["ticks with a mapd path"] += 1


def corpus1(env, sink, cov, path=TICKS_CACHE, limit=None) -> list:
  if not os.path.exists(path):
    raise MissingInput(f"corpus 1: the tick cache {path} is absent (tools/curveshape/replay.py --rescan builds it)")
  ticks, perr = [], 0
  with open(path) as fh:
    for ln in fh:
      try:
        r = json.loads(ln)
      except ValueError:
        perr += 1
        continue
      if f(r.get("vEgo")) is None or f(r.get("t")) is None:
        cov.acct["c1_cache"]["skipped: no vEgo / t"] += 1
        continue
      ticks.append(r)
      if limit and len(ticks) >= limit:
        break
  if perr:
    cov.acct["c1_cache"]["cache lines that did not parse"] += perr
  if not ticks:
    raise HarnessError(f"corpus 1: {path} yielded NO ticks -- the input is broken, not empty road")
  _run_logged(env, sink, cov, "c1_cache", ticks, active_of=lambda r: True, has_k_of=lambda r: r.get("_hasK") is True)
  return [f"c1_cache: {len(ticks)} ticks from {path}; no vision / vSet / lead distance / mapd path in the cache " +
          "(left out of sig, not invented); one _icbm_step per logged ~1 Hz tick"]


def _open_any(p):
  if p.endswith(".zst"):
    return io.TextIOWrapper(zstandard.ZstdDecompressor().stream_reader(open(p, "rb")), errors="replace")
  if p.endswith(".gz"):
    return gzip.open(p, "rt", errors="replace")
  return open(p, errors="replace")


def scan_pnwlogs(d=PNWLOGS, since=CORPUS2_SINCE):
  """(ticks, paths_by_file, inventory). Every file is reported; an unreadable one is an ERROR row, never a zero."""
  files = sorted(glob.glob(os.path.join(d, "ces_events.jsonl*")))
  if not files:
    raise MissingInput(f"corpus 2: no ces_events files in {d} (sync them: aws --profile dipeit s3 sync " +
                       "s3://comma-connect/drives/2fd850c60cc5bfef/pnwlogs/ <dir> --exclude '*' --include 'ces_events*')")
  keep, paths, inv = {}, defaultdict(list), []
  for fn in files:
    row = Counter()
    err = None
    try:
      with _open_any(fn) as fh:
        for ln in fh:
          try:
            r = json.loads(ln)
          except ValueError:
            row["unparsable"] += 1
            continue
          if not isinstance(r, dict):
            row["unparsable"] += 1
            continue
          row["records"] += 1
          t = f(r.get("t"))
          if t is None or t < since or not str(r.get("car") or "").startswith("FORD"):
            continue
          if r.get("ev") == "mapdPath":
            row["mapdPath"] += 1
            paths[fn].append(r)
            continue
          if r.get("ev") != "tick":
            continue
          v = f(r.get("vEgo"))
          if v is None or v < MIN_MOVING_MS:
            continue
          row["moving Lightning ticks"] += 1
          r["_src"] = os.path.basename(fn)
          keep[round(t, 2)] = r
    except Exception as e:
      err = f"{type(e).__name__}: {e}"
    inv.append((os.path.basename(fn), dict(row), err))
  return [keep[k] for k in sorted(keep)], paths, inv


def _path_lookup(paths):
  """r -> mapd's point list from the newest mapdPath record at or before the tick in the same file (<= 120 s)."""
  by_src = {os.path.basename(fn): sorted(rs, key=lambda x: x["t"]) for fn, rs in paths.items()}

  def at(r):
    rs = by_src.get(r["_src"])
    if not rs:
      return None
    best = None
    for p in rs:
      if p["t"] <= r["t"]:
        best = p
      else:
        break
    if best is None or r["t"] - best["t"] > 120.0:
      return None
    return [{"latitude": a, "longitude": b, "velocity": v if v is not None else float("nan")}
            for a, b, v in m.mapd_path_decode(best)]
  return at


def corpus2(env, sink, cov, d=PNWLOGS, since=CORPUS2_SINCE) -> list:
  ticks, paths, inv = scan_pnwlogs(d, since)
  errs = [x for x in inv if x[2]]
  n_paths = sum(len(v) for v in paths.values())
  notes = [f"c2_pnwlogs: {len(inv)} files scanned in {d}, {len(ticks)} moving Lightning ticks since " +
           f"{datetime.fromtimestamp(since, PT):%Y-%m-%d %H:%M} PT, {n_paths} mapdPath records, {len(errs)} unreadable files"]
  for fn, _row, e in errs:
    notes.append(f"  UNREADABLE {fn}: {e}")
  if n_paths == 0:
    notes.append("  NO mapdPath RECORD EXISTS in this corpus: every tick ran with no mapd path (far candidate, DB decide " +
                 "and passed gate NOT exercised by real geometry here -- see corpora 3 and 4)")
  if errs:
    cov.acct["c2_pnwlogs"]["unreadable files"] += len(errs)
  if not ticks:
    raise HarnessError(f"corpus 2: {len(inv)} files, no Lightning tick since the cut -- the scan or the sync is broken")
  _run_logged(env, sink, cov, "c2_pnwlogs", ticks, active_of=lambda r: int(r.get("button") or 0) == C.BTN_CES,
              has_k_of=lambda r: "icbmK" in r, path_of=_path_lookup(paths))
  return notes


# ---------------------------------------------------------------------------------------------------------------------
# a simulated car on a known road (corpora 3 and 4)
# ---------------------------------------------------------------------------------------------------------------------
class Road:
  """A road sampled every STEP m: position, bearing (deg from north, clockwise) and signed curvature (+ = right)."""
  STEP = 2.0

  def __init__(self, lat, lon, brg, k):
    self.lat, self.lon, self.brg, self.k = lat, lon, brg, k
    self.length = (len(lat) - 1) * self.STEP

  @classmethod
  def from_segments(cls, lat0, lon0, h0_deg, segs):
    """segs: [(length_m, signed curvature)]"""
    x = y = 0.0
    h = math.radians(h0_deg)
    cos0 = math.cos(math.radians(lat0))
    lat, lon, brg, ks = [], [], [], []
    for length, k in segs:
      for _ in range(max(int(round(length / cls.STEP)), 1)):
        lat.append(lat0 + y / 111320.0)
        lon.append(lon0 + x / (111320.0 * cos0))
        brg.append(math.degrees(h) % 360.0)
        ks.append(k)
        h += k * cls.STEP
        x += cls.STEP * math.sin(h)
        y += cls.STEP * math.cos(h)
    return cls(lat, lon, brg, ks)

  @classmethod
  def from_path(cls, pts):
    """A logged pass ([[lat, lon], ...]) resampled every STEP m; curvature from the bearing change."""
    poly = cl.Polyline([{"latitude": a, "longitude": b} for a, b in pts])
    if not poly.ok:
      raise HarnessError("corpus 3: an episode path is not a usable polyline")
    n = int(poly.s_max // cls.STEP)
    lat, lon, brg = [], [], []
    for i in range(n + 1):
      la, lo = poly.at(i * cls.STEP)
      lat.append(la)
      lon.append(lo)
      brg.append(poly.heading(i * cls.STEP) % 360.0)
    ks = []
    for i in range(len(brg)):
      j0, j1 = max(i - 10, 0), min(i + 10, len(brg) - 1)
      db = (brg[j1] - brg[j0] + 540.0) % 360.0 - 180.0
      ks.append(math.radians(db) / max((j1 - j0) * cls.STEP, 1.0))
    return cls(lat, lon, brg, ks)

  def idx(self, s):
    return min(max(int(s / self.STEP), 0), len(self.lat) - 1)

  def at(self, s):
    i = self.idx(s)
    return self.lat[i], self.lon[i], self.brg[i], self.k[i]

  def k_max_ahead(self, s, dist):
    """(signed k, distance to it) of the sharpest point in (s, s + dist]."""
    i0, i1 = self.idx(s), self.idx(s + dist)
    best, bd = 0.0, float("inf")
    for i in range(i0, i1 + 1):
      if abs(self.k[i]) > abs(best):
        best, bd = self.k[i], i * self.STEP - s
    return best, bd


def mapd_nodes(road, spacing, rate_fn, rng=None):
  """mapd's nodes along the road: (s, lat, lon, rating). Rating from rate_fn(s, k), then the backward decel envelope
  (mapd publishes speeds the car can brake to)."""
  s, nodes = 0.0, []
  while s <= road.length:
    la, lo, _b, k = road.at(s)
    nodes.append([s, la, lo, rate_fn(s, k)])
    s += spacing * (1.0 + (rng.uniform(-0.2, 0.2) if rng else 0.0))
  for i in range(len(nodes) - 2, -1, -1):
    r_next = nodes[i + 1][3]
    if math.isfinite(r_next) and math.isfinite(nodes[i][3]):
      nodes[i][3] = min(nodes[i][3], math.sqrt(r_next * r_next + 2.0 * 1.0 * (nodes[i + 1][0] - nodes[i][0])))
  return nodes


def mapd_window(nodes, s_truck, back=40.0, ahead=600.0, corrupt=None, reverse=False):
  """mapd's published path: the nodes from `back` behind the truck (mapd publishes the whole current way) to `ahead`.
  reverse: the way's nodes in the opposite order -- the passed-point gate cannot tell ("reversed"), which is where
  icbmfalsify2pnw is the only thing that can end a phantom cap the truck drove through."""
  out = []
  for s, la, lo, v in nodes:
    if s_truck - back <= s <= s_truck + ahead:
      out.append({"latitude": la, "longitude": lo, "velocity": v})
  if reverse:
    out.reverse()
  if corrupt is not None and out:
    out[corrupt % len(out)] = {"latitude": float("nan"), "longitude": None, "velocity": "x"}
  return out


class Truck:
  """The loop around ICBM: speed, the stock set an executor taps toward the published target, the GPS fix mapd_configd
  would publish (lagged / frozen / unprojectable / absent), the driver's gas / brake / set moves."""

  def __init__(self, road, s0, v0, set0):
    self.road, self.s, self.v, self.set = road, s0, v0, set0
    self.on = True
    self.hist = [(0.0, s0)]
    self.freeze = None

  def s_at(self, t):
    for tt, ss in reversed(self.hist):
      if tt <= t:
        return ss
    return self.hist[0][1]

  def step(self, t, dt, *, lead_v=None, gas=False, brake=False):
    tgt = self.set if self.on else self.v
    if lead_v is not None:
      tgt = min(tgt, lead_v)
    if gas:
      tgt = self.v + 3.0
    if brake:
      tgt = max(self.v - 4.0, 0.0)
    up, dn = (1.5 if gas else 1.0), (2.5 if brake else 1.5)
    self.v = max(self.v + max(min(tgt - self.v, up * dt), -dn * dt), 0.0)
    self.s += self.v * dt
    self.hist.append((t, self.s))
    if len(self.hist) > 200:
      del self.hist[:100]

  def gps(self, t, mode, rng, lateral=0.0):
    """LastGPSPosition as mapd_configd writes it, or None. `lateral` (m, right of the road) puts the fix beside mapd's
    way -- beyond ICBM_PATH_MAX_PERP_M the passed-point gate cannot tell, and a running cap meets icbmfalsify2pnw."""
    if mode == "none":
      self.freeze = None
      return None
    if mode == "stale":
      if self.freeze is None:
        self.freeze = (t, self.s)
      t_fix, s_fix = self.freeze
    else:
      self.freeze = None
      age = rng.uniform(0.8, 2.4)
      t_fix, s_fix = t - age, self.s_at(t - age)
    la, lo, brg, _k = self.road.at(s_fix)
    if lateral:
      h = math.radians(brg + 90.0)
      la += lateral * math.cos(h) / 111320.0
      lo += lateral * math.sin(h) / (111320.0 * math.cos(math.radians(la)))
    pos = {"latitude": la, "longitude": lo, "bearing": brg, "src": "car", "ts": t}
    if mode != "raw":
      pos["fix_ts"] = t_fix
    return pos

  def executor(self, pub):
    """One tap every 0.5 s toward the published target (1 mph), the inc path only when the payload says so."""
    if not isinstance(pub, dict) or "target" not in pub:
      return
    tgt, ceil = pub["target"], pub.get("ceiling", pub["target"])
    if pub.get("dir") == "inc":
      if self.set < min(tgt, ceil) - 0.2:
        self.set += MPH
    elif self.set > tgt + 0.2:
      self.set -= MPH


def vision(road, s, v, rng, dropout, late=None):
  """What the model's plan would give CES: signed lateral accel of the sharpest point in 8 s, time to it, lateral
  accel now, the tightest curvature and the reach. A dropout is the model hiccup (0.0 / 10 s / None / 0.0); `late`
  (seconds) blinds the model to a curve until it is that close (a crest, a blind corner)."""
  if late is not None and not dropout:
    k, d = road.k_max_ahead(s, max(v, 1.0) * 8.0)
    dropout = abs(k) > 1e-5 and d / max(v, 1.0) > late
  if dropout:
    return {"curve_lat_accel_vision": 0.0, "time_to_curve": 10.0, "lat_accel_now": 0.0,
            "vis_k_max": None, "vis_reach": 0.0}
  reach = max(v, 1.0) * 8.0
  k, d = road.k_max_ahead(s, reach)
  noise = rng.uniform(0.85, 1.15)
  lat = k * v * v * noise
  ttc = d / max(v, 1.0) if abs(k) > 1e-5 else 10.0
  return {"curve_lat_accel_vision": lat, "time_to_curve": ttc, "lat_accel_now": road.at(s)[3] * v * v * noise,
          "vis_k_max": abs(k) * noise, "vis_reach": reach}


def drive(env, sink, cov, name, ctl, road, nodes, sc, rng, t0=1000.0, wall0=1.79e9, max_frames=10_000):
  """One simulated run at 4 Hz: mapd refresh + _read_map at 1 Hz, sig every tick, the executor after each publish.
  sc: s0, v0, set0, posted, hwy, gps (a mode or a list of (frame, mode) switches), way (a list of (frame, waySel)),
  lead (None or (gap_m, dv)), gas / brake / chill (sets of frame indices), pitch, corrupt (node index or None),
  set_moves ({frame: delta_mph}), end_s. Returns the number of frames driven."""
  tr = Truck(road, sc["s0"], sc["v0"], sc["set0"])
  gps_sw = sc["gps"] if isinstance(sc["gps"], list) else [(0, sc["gps"])]
  way_sw = sc["way"]
  gps_mode, way = gps_sw[0][1], way_sw[0][1]
  mem = ctl.mem_params
  frames = 0
  for fr in range(max_frames):
    t = t0 + fr * DT
    for f0, mode in gps_sw:
      if fr == f0:
        gps_mode = mode
    for f0, w in way_sw:
      if fr == f0:
        way = w
    gas, brake = fr in sc["gas"], fr in sc["brake"]
    lead_v = None
    if sc["lead"] is not None:
      lead_v = max(tr.set + sc["lead"][1], 5.0)
    tr.step(t, DT, lead_v=lead_v, gas=gas, brake=brake)
    if tr.s >= sc["end_s"] or (tr.v < 1.0 and fr > 8):
      break
    if fr in sc["set_moves"]:
      tr.set = max(tr.set + sc["set_moves"][fr] * MPH, 20 * MPH)
    if fr in sc.get("acc_off", ()):
      tr.on = not tr.on
    if fr % 4 == 0:           # mapd + mapd_configd + _read_map, ~1 Hz
      mem.vals = {"MapTargetVelocities": mapd_window(nodes, tr.s, back=sc.get("back", 40.0), ahead=sc.get("ahead", 600.0),
                                                     corrupt=sc["corrupt"] if fr % 40 == 0 else None,
                                                     reverse=sc.get("reverse", False)),
                  "LastGPSPosition": tr.gps(t, gps_mode, rng, sc.get("lateral", 0.0)),
                  "MapSpeedLimit": str(sc["posted"]) if sc["posted"] else "",
                  "MapHighwayClass": sc["hwy"] or "", "MapWaySel": way or "",
                  # a real yaw sensor never repeats itself exactly; a constant reading is FROZEN to icbmfalsify2pnw
                  "SteerLimitStatus": {"kActl": -road.at(tr.s)[3] * rng.uniform(0.8, 1.2) + rng.gauss(0.0, 2e-5)}}
      env.clock.set(t, wall0 + t)
      ctl._read_map()
    ctl._stock_set, ctl._stock_on = tr.set, tr.on
    v = tr.v
    map_v, map_d = m.upcoming_curve(ctl._map_targets, ctl._cur_lat, ctl._cur_lon, v, C.CURVE_MAP_LOOKAHEAD_S)
    sig = {"v_ego": v, "v_set": tr.set, "map_target_v": map_v, "map_target_dist": map_d, "spd_lim": sc["posted"],
           "gas": gas, "brake": brake, "pitch": sc["pitch"],
           "has_lead": lead_v is not None, "lead_vlead": lead_v or 0.0,
           "lead_drel": (sc["lead"][0] if sc["lead"] else 0.0),
           **vision(road, tr.s, v, rng, rng.random() < 0.05, sc.get("vis_late"))}
    line = env.tick(ctl, t, sig, fr not in sc["chill"], wall0 + t)
    line.update(t=t, fr=fr)
    sink.write(line)
    cov.add(name, line)
    if fr % 2 == 0:
      tr.executor(line["pub"])
    frames += 1
  return frames


def _db_rows_along(road, rng, arcs, extra_rows=(), p_bad=0.16, k_bias=(0.6, 1.4)):
  """Synthetic curve-DB anchors on a road: every ~50 m through each arc (from 60 m before it), the row curvature the
  arc's own +- 40 %, with the keying's failure modes mixed in (no authority, ambiguous branch, unknown branch).
  extra_rows: (s, k) rows the map does not rate (the ADD path) or rates wrongly (the phantom)."""
  anchors = []

  def one(s, k_row):
    if s + 160.0 > road.length or s < 0.0:
      return
    la, lo, brg, _k = road.at(s)
    e_la, e_lo, _b, _k2 = road.at(s + 150.0)
    u = rng.random()
    if u < p_bad / 3:
      branches = [[e_la, e_lo, None, 3]]                                  # noAuthority
    elif u < p_bad * 2 / 3:
      branches = [[e_la, e_lo, k_row, 3], [e_la + 2e-5, e_lo, k_row, 3]]   # branchAmbiguous
    elif u < p_bad:
      branches = [[e_la + 4e-4, e_lo + 4e-4, k_row, 3]]                    # branchUnknown
    else:
      branches = [[e_la, e_lo, k_row, 3]]
    anchors.append([la, lo, brg, branches])

  for a0, a1 in arcs:
    s = a0 - 60.0
    while s < a1:
      k_row = max(max(abs(road.k[road.idx(x)]) for x in (s, s + 50.0, s + 100.0, s + 150.0)), 2e-4)
      one(s, min(k_row * rng.uniform(*k_bias), 0.05))
      s += rng.uniform(40.0, 60.0)
  for s, k in extra_rows:
    one(s, k)
  return anchors


def write_db(d, anchors, params=None):
  """A rows file + manifest exactly as tools/curvedb/v2_live_export.py writes them (the ces_pnw test fixture's
  writer)."""
  from openpilot.selfdrive.controls.lib.ces_pnw.tests.roaddb_fixture import write_db as _w
  return _w(d, anchors, params=params)


# ---------------------------------------------------------------------------------------------------------------------
# corpus 3: the private DB replay fixture
# ---------------------------------------------------------------------------------------------------------------------
def corpus3(env, sink, cov, fixture=FIXTURE) -> list:
  if not os.path.exists(fixture):
    raise MissingInput(f"corpus 3: the PRIVATE replay fixture {fixture} is absent (tools/curvedb/v2_live_fixture.py)")
  with gzip.open(fixture, "rt") as fh:
    fx = json.load(fh)
  runs = [(f"ep{i}", e) for i, e in enumerate(fx["episodes"])] + [(f"add{i}", x) for i, x in enumerate(fx["adds"])]
  rng = random.Random(FUZZ_SEED + 3)
  n = 0
  for tag, e in runs:
    rows = e["rows"]
    db_dir = os.path.join(env.tmp, f"c3-{tag}")
    write_db(db_dir, rows["anchors"], params=rows["params"])
    road = Road.from_path(e["path"])
    s_site, _ = cl.Polyline([{"latitude": a, "longitude": b} for a, b in e["path"]]).project(*e["site"])
    ref = e["ref"] if "ref" in e else e["v_app_mph"] * MPH
    icbm = e.get("icbm")          # None on an ADD: mapd never rated that curve, the DB supplies it
    posted = e.get("posted") or 65 * MPH

    def rate(s, k, s_site=s_site, icbm=icbm):
      return icbm / 0.92 if icbm is not None and abs(s - s_site) <= 40.0 else 40.0
    nodes = mapd_nodes(road, 40.0, rate)        # the polyline's spacing gate wants legs of 25-300 m
    for shape in ("shadow", "live"):
      for gps in ("proj", "raw"):
        ctl = env.build({"shape": shape, "db_dir": db_dir}, 999.0, 1.79e9 + 999.0)
        sink.write({"seg": f"{tag}/{shape}/{gps}", "log": env.take_logs()})
        sc = {"s0": max(s_site - 700.0, 0.0), "v0": ref, "set0": ref, "posted": posted, "hwy": "motorway",
              "gps": gps, "way": [(0, "current")], "lead": None, "gas": set(), "brake": set(), "chill": set(),
              "pitch": None, "corrupt": None, "set_moves": {}, "end_s": road.length - 20.0}
        n += drive(env, sink, cov, "c3_fixture", ctl, road, nodes, sc, rng)
  return [f"c3_fixture: {len(runs)} logged passes x shape shadow/live x GPS proj/raw from {fixture}, {n} ticks"]


# ---------------------------------------------------------------------------------------------------------------------
# corpus 4: seeded fuzz
# ---------------------------------------------------------------------------------------------------------------------
MATRIX = [(shape, db, rain, gps) for shape in ("off", "shadow", "live") for db in (True, False) for rain in (0, 1, 2)
          for gps in ("proj", "stale", "raw", "none")]
KINDS = ("curve", "curve", "sbend", "twocurves", "phantom", "tight", "straight")
HWY = ("motorway", "motorway", "motorway", "trunk", "primary", "motorwayLink", None, "unknown")


def fuzz_scenario(rng, i, tmp):
  """(cfg, road, nodes, sc) for fuzz scenario i. The matrix cell is i % len(MATRIX); everything else is drawn."""
  shape, db, rain, gps = MATRIX[i % len(MATRIX)]
  kind = rng.choice(KINDS)
  lat0, lon0 = 44.0 + 0.02 * (i % 200), -121.0 - 0.02 * (i // 200)
  lead_in = rng.uniform(250.0, 900.0)
  segs, arcs, extra = [(lead_in, 0.0)], [], []
  sgn = rng.choice((-1.0, 1.0))
  # a curve that binds at highway speed on a road posted far below it, with rows gentler than the map: the DB's raise
  # is capped at posted + 10 mph, i.e. withheld ("held")
  low_posted = kind in ("curve", "tight") and rng.random() < 0.25
  if kind in ("curve", "tight"):
    r = rng.uniform(80.0, 250.0) if kind == "tight" else rng.uniform(150.0, 1800.0)
    if low_posted:
      r = rng.uniform(120.0, 300.0)
    ln = rng.uniform(60.0, 400.0)
    segs.append((ln, sgn / r))
    arcs.append((lead_in, lead_in + ln))
  elif kind == "sbend":
    r1, r2, l1, l2, gap = (rng.uniform(150.0, 900.0), rng.uniform(150.0, 900.0), rng.uniform(60.0, 250.0),
                           rng.uniform(60.0, 250.0), rng.uniform(0.0, 150.0))
    segs += [(l1, sgn / r1), (gap, 0.0), (l2, -sgn / r2)]
    arcs += [(lead_in, lead_in + l1), (lead_in + l1 + gap, lead_in + l1 + gap + l2)]
  elif kind == "twocurves":
    r1, r2, l1, l2, gap = (rng.uniform(300.0, 1500.0), rng.uniform(150.0, 600.0), rng.uniform(80.0, 250.0),
                           rng.uniform(80.0, 250.0), rng.uniform(150.0, 450.0))
    segs += [(l1, sgn / r1), (gap, 0.0), (l2, sgn / r2)]
    arcs += [(lead_in, lead_in + l1), (lead_in + l1 + gap, lead_in + l1 + gap + l2)]
  segs.append((900.0, 0.0))
  road = Road.from_segments(lat0, lon0, rng.uniform(0.0, 360.0), segs)
  err = rng.choice((0.7, 0.85, 1.0, 1.0, 1.15, 1.4))
  ph0 = lead_in + rng.uniform(-100.0, 100.0)
  straight_v = rng.uniform(40.0, 60.0)

  def rate(s, k):
    if kind == "phantom" and ph0 <= s <= ph0 + 60.0:
      return rng.uniform(14.0, 25.0)
    if abs(k) > 1e-5:
      return math.sqrt(A_MAPD / abs(k)) * err
    return straight_v
  nodes = mapd_nodes(road, rng.choice((10.0, 20.0, 30.0, 50.0, 80.0)), rate, rng)
  if kind == "phantom":
    extra.append((ph0 - 20.0, 2e-4))
  if kind == "straight" and rng.random() < 0.5:
    extra.append((lead_in + 100.0, rng.uniform(0.002, 0.006)))   # a curve the map does not rate: the DB adds it
  cfg = {"shape": shape, "db": db, "rain": rain}
  if db:
    d = os.path.join(tmp, f"fz-{i}")
    write_db(d, _db_rows_along(road, rng, arcs, extra, p_bad=rng.choice((0.16, 0.6)),
                               k_bias=(0.4, 0.8) if low_posted else rng.choice(((0.6, 1.4), (1.1, 1.8), (1.3, 1.8)))))
    cfg["db_dir"] = d
    u = rng.random()
    if u < 0.06:
      cfg["db_dir"] = os.path.join(tmp, "no-such-db")          # dbErr: the load fails, loudly
    elif u < 0.25:
      cfg["lat_a"] = 0                                         # the DB reads mapd's A ...
      if rng.random() < 0.3:
        cfg["reader"] = "broken"                               # ... and it is unreadable: noA
  if rng.random() < 0.05:
    cfg["reader"] = "broken"                                   # the shape stage's A unreadable
  vis_late = rng.uniform(1.0, 2.7) if rng.random() < 0.15 else None
  v0 = rng.uniform(27.0, 34.0) if low_posted else rng.uniform(15.0, 34.0)
  set0 = round((v0 + (rng.uniform(3.0, 10.0) if vis_late else 0.0)) / MPH) * MPH   # a late curve met below the set
  n_est = int((road.length - 60.0) / max(v0, 1.0) / DT)
  gps_sw = [(0, gps)]
  if gps == "proj" and rng.random() < 0.4:                     # a tunnel: the fix freezes mid-approach, then returns
    fz = int(lead_in * rng.uniform(0.4, 1.0) / v0 / DT)
    gps_sw += [(fz, "stale"), (fz + rng.randint(8, 400), "proj")]
  way = [(0, rng.choice(("current",) * 8 + ("predicted", "possible", None)))]
  for _ in range(rng.randint(0, 3)):
    f0 = rng.randint(1, max(n_est, 2))
    way += [(f0, rng.choice(("predicted", "possible", "extended", None))), (f0 + rng.randint(2, 16), "current")]
  way.sort(key=lambda x: x[0])

  def frames(p, lo, hi):
    out = set()
    if rng.random() < p:
      f0 = rng.randint(0, max(n_est, 1))
      out.update(range(f0, f0 + rng.randint(lo, hi)))
    return out
  posted = rng.choice((0.0, 0.0, 25 * MPH, 35 * MPH, 45 * MPH, 55 * MPH, 60 * MPH, 65 * MPH, 70 * MPH))
  if low_posted:
    posted = rng.choice((25 * MPH, 35 * MPH))
  sc = {"s0": 0.0, "v0": v0, "set0": set0, "posted": posted, "hwy": rng.choice(HWY), "gps": gps_sw, "way": way,
        "lead": (rng.uniform(20.0, 80.0), rng.uniform(-6.0, 1.0)) if rng.random() < (0.7 if vis_late else 0.25) else None,
        "gas": frames(0.3, 4, 24), "brake": frames(0.1, 2, 8), "chill": frames(0.1, 4, 30),
        "pitch": rng.choice((None, None, 0.0, -0.03, -0.07, 0.02)),
        "corrupt": rng.randint(0, 40) if rng.random() < 0.1 else None,
        "set_moves": {rng.randint(0, max(n_est, 1)): rng.choice((-5, -1, 1, 5))} if rng.random() < 0.2 else {},
        "acc_off": {rng.randint(0, max(n_est, 1))} if rng.random() < 0.05 else set(),
        "end_s": road.length - 60.0, "back": rng.choice((0.0, 40.0, 40.0, 150.0)),
        "vis_late": vis_late, "lateral": rng.uniform(42.0, 70.0) if rng.random() < (0.5 if kind == "phantom" else 0.08) else 0.0,
        "ahead": rng.uniform(30.0, 80.0) if rng.random() < 0.05 else 600.0,
        "reverse": rng.random() < (0.4 if kind == "phantom" else 0.04)}
  return cfg, road, nodes, sc


def corpus4(env, sink, cov, n_frames=FUZZ_FRAMES, seed=FUZZ_SEED) -> list:
  rng = random.Random(seed)
  total, i = 0, 0
  while total < n_frames:
    cfg, road, nodes, sc = fuzz_scenario(rng, i, env.tmp)
    ctl = env.build(cfg, 999.0, 1.79e9 + 999.0)
    sink.write({"seg": f"fz{i}", "cfg": {k: v for k, v in cfg.items() if k != "db_dir"}, "log": env.take_logs()})
    total += drive(env, sink, cov, "c4_fuzz", ctl, road, nodes, sc, rng, max_frames=n_frames - total)
    i += 1
  return [f"c4_fuzz: {i} scenarios, {total} ticks, seed {seed}, matrix {len(MATRIX)} cells round-robin"]


# ---------------------------------------------------------------------------------------------------------------------
# run / record / check
# ---------------------------------------------------------------------------------------------------------------------
RUNNERS = {"c1_cache": corpus1, "c2_pnwlogs": corpus2, "c3_fixture": corpus3, "c4_fuzz": corpus4}


def run(out_dir=None, corpora=CORPORA, opts=None):
  """Run the corpora; returns (shas, lines, cov, notes, missing). out_dir None: hash only."""
  opts = opts or {}
  if out_dir:
    os.makedirs(out_dir, exist_ok=True)
  cov, shas, lines, notes, missing = Coverage(), {}, {}, [], []
  with Env() as env:
    for c in corpora:
      sink = Sink(os.path.join(out_dir, f"{c}.jsonl.zst") if out_dir else None, scrub=env.tmp)
      try:
        notes += RUNNERS[c](env, sink, cov, **opts.get(c, {}))
      except MissingInput as e:
        missing.append(str(e))
        notes.append(f"{c}: SKIPPED -- {e}")
        sink.close()
        if out_dir:
          os.remove(os.path.join(out_dir, f"{c}.jsonl.zst"))
        continue
      shas[c], lines[c] = sink.close(), sink.n
    if env.thread_logs:
      notes.append(f"{env.thread_logs} cloudlog call(s) from other threads (the curvedb_shadow loader) were not replayed")
  return shas, lines, cov, notes, missing


def _sha_file(p):
  h = hashlib.sha256()
  with open(p, "rb") as fh:
    for b in iter(lambda: fh.read(1 << 20), b""):
      h.update(b)
  return h.hexdigest()


def _git(*a):
  here = os.path.dirname(os.path.abspath(__file__))
  return subprocess.run(["git", "-C", here, *a], check=True, capture_output=True, text=True).stdout.strip()


def cmd_record(args) -> int:
  os.makedirs(args.out, exist_ok=True)
  opts = {"c4_fuzz": {"n_frames": args.fuzz_frames}}
  t0 = _real_time.monotonic()
  shas, lines, cov, notes, missing = run(args.out, opts=opts)
  db_man = os.path.join(DB_DIR, "manifest.json")
  db_sha = None
  if os.path.exists(db_man):
    with open(db_man) as fh:
      db_sha = json.load(fh).get("sha256")
  # tracked changes the recording ran with; the submodule checkouts symlinked into a worktree show as typechanges
  # (" T") and are named by their own SHAs instead
  subs = ("opendbc_repo", "rednose_repo", "msgq_repo", "panda", "tinygrad_repo", "teleoprtc_repo")
  dirty = [ln for ln in _git("status", "--porcelain", "--untracked-files=no").splitlines()
           if not (ln.strip().startswith("T ") and ln.strip()[2:] in subs)]
  man = {"base_commit": _git("rev-parse", "HEAD"), "dirty": dirty,
         "submodules": {x: _git("-C", os.path.join(_git("rev-parse", "--show-toplevel"), x), "rev-parse", "HEAD")
                        for x in subs},
         "harness_sha256": _sha_file(os.path.abspath(__file__)),
         "recorded_utc": datetime.now(UTC).strftime("%Y-%m-%dT%H:%M:%SZ"),
         "seconds": round(_real_time.monotonic() - t0, 1), "fuzz_frames": args.fuzz_frames, "fuzz_seed": FUZZ_SEED,
         "inputs": {"c1_cache": TICKS_CACHE, "c2_pnwlogs": PNWLOGS, "c3_fixture": FIXTURE, "db_dir": DB_DIR,
                    "db_manifest_sha256": db_sha},
         "corpora": {c: {"file": f"{c}.jsonl.zst", "lines": lines[c], "sha256": shas[c]} for c in shas},
         "missing": missing, "notes": notes}
  with open(os.path.join(args.out, "manifest.json"), "w") as fh:
    json.dump(man, fh, indent=1)
  rep = cov.report(shas, notes)
  with open(os.path.join(args.out, "coverage.txt"), "w") as fh:
    fh.write(rep)
  print(rep)
  print(f"goldens in {args.out}")
  if missing:
    print("MISSING INPUT(S):\n  " + "\n  ".join(missing))
    return 3
  return 0


def _drop(obj, names):
  if isinstance(obj, dict):
    return {k: _drop(v, names) for k, v in obj.items() if k not in names}
  if isinstance(obj, list):
    return [_drop(v, names) for v in obj]
  return obj


def compare(golden_path, new_path, exclude=()) -> tuple[int, int, list]:
  """(lines compared, lines that differ, first diffs). Excluded names are dropped at every level before comparing."""
  names = set(exclude)
  n = bad = 0
  first = []
  a_it, b_it = read_golden(golden_path), read_golden(new_path)
  while True:
    a, b = next(a_it, None), next(b_it, None)
    if a is None and b is None:
      break
    n += 1
    if a is None or b is None:
      bad += 1
      first.append((n, "LENGTH", "golden ended" if a is None else "new run ended"))
      break
    if a == b:
      continue
    if names:
      ja, jb = _drop(json.loads(a), names), _drop(json.loads(b), names)
      if canon(ja) == canon(jb):
        continue
    bad += 1
    if len(first) < 5:
      ja, jb = _drop(json.loads(a), names), _drop(json.loads(b), names)
      keys = sorted(k for part in ("rec", "st", "pub", "log") for k in _diff_keys(ja.get(part), jb.get(part), part))
      first.append((n, keys[:12], ""))
  return n, bad, first


def _diff_keys(a, b, prefix):
  if isinstance(a, dict) and isinstance(b, dict):
    out = []
    for k in set(a) | set(b):
      out += _diff_keys(a.get(k), b.get(k), f"{prefix}.{k}")
    return out
  return [] if a == b else [prefix]


def mutate(spec):
  """NAME=DELTA: add DELTA to the constant NAME in every loaded openpilot.selfdrive.controls module that defines it
  (so the self-test still bites after stage 2 moves the helpers to curve_brain.py). Returns the patchers."""
  name, delta = spec.split("=", 1)
  pats = []
  for mod_name, mod in list(sys.modules.items()):
    if mod_name.startswith("openpilot.selfdrive.controls") and isinstance(getattr(mod, name, None), (int, float)):
      pats.append(_Patch(mod, name, getattr(mod, name) + float(delta)))
  if not pats:
    raise HarnessError(f"--mutate: no loaded module defines {name}")
  return pats


def cmd_check(args) -> int:
  man_path = os.path.join(args.golden, "manifest.json")
  if not os.path.exists(man_path):
    raise MissingInput(f"no manifest at {man_path}")
  with open(man_path) as fh:
    man = json.load(fh)
  here = _sha_file(os.path.abspath(__file__))
  if here != man["harness_sha256"]:
    print(f"WARNING: this harness ({here[:12]}) is NOT the one that recorded the goldens ({man['harness_sha256'][:12]}) " +
          "-- a difference may be the harness's, not the code's")
  corpora = [c for c in CORPORA if c in man["corpora"] and (not args.corpus or c in args.corpus)]
  opts = {"c4_fuzz": {"n_frames": man["fuzz_frames"]}}
  tmp = tempfile.mkdtemp(prefix="curvebrain-check-")
  pats = [p for spec in args.mutate for p in mutate(spec)]
  try:
    with contextlib.ExitStack() as st:
      for p in pats:
        st.enter_context(p)
      shas, lines, cov, notes, missing = run(tmp, corpora, opts)
    worst = 0
    for c in corpora:
      if c not in shas:
        print(f"{c}: NOT RUN -- " + "; ".join(missing))
        worst = max(worst, 3)
        continue
      same = shas[c] == man["corpora"][c]["sha256"]
      n, bad, first = (lines[c], 0, []) if same else compare(os.path.join(args.golden, man["corpora"][c]["file"]),
                                                               os.path.join(tmp, f"{c}.jsonl.zst"), args.exclude)
      print(f"{c}: {n} lines, {bad} differ; sha256 {'IDENTICAL' if same else 'differs'} ({shas[c][:16]})")
      for ln, keys, msg in first:
        print(f"   line {ln}: {msg or keys}")
      if bad:
        worst = max(worst, 1)
    return worst
  finally:
    shutil.rmtree(tmp)


def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  sub = ap.add_subparsers(dest="cmd", required=True)
  r = sub.add_parser("record")
  r.add_argument("--out", default=OUT)
  r.add_argument("--fuzz-frames", type=int, default=FUZZ_FRAMES)
  c = sub.add_parser("check")
  c.add_argument("--golden", default=OUT)
  c.add_argument("--corpus", action="append", default=[])
  c.add_argument("--exclude", action="append", default=[], help="a rec/st key a later stage adds (by name)")
  c.add_argument("--mutate", action="append", default=[], help="NAME=DELTA, e.g. ICBM_MARGIN_M=1e-9")
  c.add_argument("--overrides", default=None, help="a per-curve override file for the replay (default: an empty valid one)")
  args = ap.parse_args(argv)
  if getattr(args, "overrides", None):
    global OVERRIDES_FILE
    OVERRIDES_FILE = os.path.abspath(args.overrides)
  try:
    return cmd_record(args) if args.cmd == "record" else cmd_check(args)
  except MissingInput as e:
    print(f"MISSING INPUT: {e}")
    return 3
  except HarnessError as e:
    print(f"HARNESS FAILURE: {e}")
    return 2


if __name__ == "__main__":
  sys.exit(main())
