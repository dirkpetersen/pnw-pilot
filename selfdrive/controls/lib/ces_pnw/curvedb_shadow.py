"""curvedbshadow2pnw -- CURVEDB2PNW.md Phase 2, the ON-CAR half, **SHADOW ONLY**.

THE ONE THING TO UNDERSTAND ABOUT THIS FILE
===========================================
On every ICBM map/far curve decision the learned database says what it **would** have done, and
**nothing in the control path reads that answer**. It writes a log line. That is all it does, and
there is no other mode: `authority()` and `cancel_target()` are called here, their results are
rendered into a telemetry fragment, and that fragment's only destination is one `**`-splat into the
`ces_events` record dict. There is no code path -- gated, parameterised or otherwise -- by which a
row in this database can reach `IcbmTarget`, the planner, or the brakes.

**That boundary is enforced by a test, not by this paragraph.**
`tests/test_curvedb_read_boundary.py` parses `ces_pnw.py` and this module with `ast` and fails if
the shadow's output is bound to any name, reaches any actuator, or is read anywhere other than the
record splat -- and a second test proves it empirically by running the real controller twice with
two wildly different databases and asserting the published `IcbmTarget` is byte-identical. The
comment "nothing reads this" has been true-when-written and false-later in this codebase before
(`icbmK`, `visK`, the whole `icbm_map_sanity` block); a sentence is not a guarantee.

WHY SHADOW AT ALL (section 11, and section 12's result)
=======================================================
The offline section 7 replay was built, run over 84 files / 400,583 ticks / 170 ICBM episodes, and
returned **NO RESULT**: under leave-one-date-out the database matched a row on 4 of 170 episodes and
acted on zero. The blocker was **site recurrence** -- leave-one-date-out needs the same curve driven
on >= 3 separate dates, and only 4-15 sites in `drives/` had that. But `drives/` is not a log, it is
a collection of analysis windows pulled when something interesting happened (median span 62 min), so
that number measures *our sampling*, not the truck's driving. The question has never actually been
asked of an unbiased record.

This module asks it. `cdbRow` -- "did a row exist for this site and this approach direction" -- is
the single field the whole project is waiting on, and it now answers itself on every drive.

WHAT IT DOES, IN TWO HALVES
===========================
**WRITE (the passage tracker).** Every site the truck's map candidate names is followed from the
tick that named it until 150 m past it (section 6.3's extent). At completion the pass becomes:

* an **UP** observation (section 6.1) when no disqualifier fired: `k = kPeak` over the extent, the
  100 Hz peak of `max(|achieved|, |commanded|)`, never an average -- *"one genuinely tight pass must
  never be averaged away by ten gentle ones"*. The `max` across passes lives in `CurveRow.k_up`.
  **No lead-car gate (D7).**
* a **DOWN** observation (section 6.2) when the driver steered against the car while the road was
  actually loading it (`|a_lat| >= down_trigger_a_lat_ms2`): `k = |slKCmd|` at that moment.

**AND DOWN IS EVALUATED EVEN WHEN THE PASS IS DIRTY -- that is the bug this file exists to not
repeat.** `tools/curvedb/ingest.py::observations_for_drive` drops a disqualified passage with
`continue` *before* its DOWN loop, and a driver steering override is itself a disqualifier (`DQ_DRV`)
-- so section 6.2 is unreachable there and produced **zero observations on 2.2 GB**. A driver
intervention is not a contaminated measurement of the road, it *is* the measurement section 6.2
wants. Here UP admissibility and DOWN admissibility are independent tests of the same pass.

**READ (the prediction).** Once per `ces_events` record, the site the record names is looked up in
the database loaded at boot, `store.authority()` decides whether that row could ever act, and
`store.cancel_target()` computes the target it would have produced. Both numbers are logged. Neither
is returned to anything.

NON-CIRCULARITY, ON THE CAR
===========================
Observations written during this drive go to the JSONL and are **not** added to the live database.
The database a prediction is made against is exactly what was on disk when selfdrived started, so a
pass can never be judged against itself. The honest statement is *"the database is as of the last
boot"*, and `cloudlog` says so at startup with the row count and the date range it covers.

WHY `store.py` IS IMPORTED RATHER THAN RE-IMPLEMENTED
=====================================================
`tools/curvedb/store.py` is the C1 core of section 8.3: zero openpilot imports, no I/O, no clock.
The section 7 replay validated *that* matcher, *those* update rules and *that* authority gate. If
the car ran a second implementation the replay would have validated nothing -- so the car imports
the same module, and the only thing this file adds is the live plumbing around it.

DELIBERATE DEVIATIONS FROM THE DESIGN, ALL FLAGGED
==================================================
1. **No daemon.** Section 8.1 wants a compacting daemon and a snapshot; this loads the append-only
   JSONL directly, in a background thread, once per boot. At section 8.1's own stated scale (2,000-
   5,000 rows) a snapshot buys a startup second and costs a second source of truth. The load IS
   measurably slow (11.9 s for 15,000 observations on the dev host, and the Snapdragon is slower),
   which is exactly why it is off the constructor's thread and why `cdbOn` reports `"loading"`
   rather than looking like "no rows".
2. **The prediction runs at record cadence (~1 Hz), not inside `_icbm_step` (~4 Hz).** It is
   evaluated against the values that record itself reports (`icbmT`, `icbmSrc`, `mapLat`/`mapLon`),
   which is what the section 7 replay grades -- and it keeps this module out of the ICBM decision
   function entirely, which is a materially stronger boundary than a comment.
3. **The approach bearing at lookup time is the truck's current heading**, taken wherever the truck
   happens to be, while the WRITE half samples it at `approach_bearing_ref_m` exactly as ingest
   does. Section 12 flagged that section 6.3 never states at what distance the bearing is measured;
   `cdbBrgD` logs the distance the lookup bearing was taken at, so the disagreement is measurable
   instead of assumed away.
4. **Sites are only discovered while CES is enabled**, because the tick record is. ICBM does not run
   with CES off either, so no ICBM decision is missed -- but passes driven with CES off are.
5. **`PROVISIONAL_ENVELOPES` is keyed by `carFingerprint`.** The capability-view rule forbids feature
   code *branching* on a fingerprint; this is a data table lookup with no branch, and it must be the
   table the offline replay uses or the replay validates nothing. The boundary test asserts this
   module contains no comparison against a fingerprint string.
6. **Sites are discovered from ICBM's OWN latched candidate, not from every map candidate.** A
   sizing decision with a measurement behind it -- see `curvedb_tele`'s docstring for the 13x.

KNOWN, DELIBERATE DIVERGENCES FROM `tools/curvedb/ingest.py`
============================================================
The matcher, the update rules and the authority gate are literally the same code (`store.py`), and
the constants, the disqualifier bits, the PT date function and the odometer integration are pinned
equal by tests. These four remain, each small and each in the direction of a WEAKER row rather than
a stronger one, and none of them is worth a second implementation of the ingest pipeline on the car:

* **The extent's back edge.** Ingest takes `[s_p - 25, s_p + 150]` by bisecting a completed track;
  a forward-only tracker cannot look back, so the window opens at the first record within
  `extent_back_m` of the candidate (approximately the same place at highway speed, later at low
  speed). The forward 150 m -- where mapd's point sits 56-125 m before the bend, i.e. where the
  signal is -- is exact, and the close is at `s_p + extent_fwd_m` exactly as ingest does.
* **`posted` and `hwy` are last-seen over the extent**, not ingest's median and most-common. Both
  are context on the observation, never part of the key.
* **`drive_id`** is `platform:first_record_epoch`; ingest's first tick can be a CES-off `steer`
  record, so the two spellings differ for the same drive. Harmless unless the car's JSONL and an
  ingest run over the same period are ever MERGED, at which point one pass would count as two drives
  and satisfy half of D6 on its own. **Do not merge the two corpora.**
* **Which sites exist at all** (deviation 6 above): ingest's `find_sites` takes every map candidate,
  the car takes ICBM's.

Rule 2 throughout: every refusal carries a reason, every failure is counted and logged, and
`cdbOn`/`cdbErr`/`cdbRows` make "the shadow is dead" impossible to confuse with "no site here".
"""
from __future__ import annotations

import datetime
import json
import math
import os
import stat
import threading
import time

import openpilot.common.pnw_log_archive as pnw_log_archive
from openpilot.common.swaglog import cloudlog
from openpilot.common.time_helpers import wall_time_valid
from openpilot.tools.curvedb.store import (
  PROVISIONAL_ENVELOPES,
  PROVISIONAL_PARAMS,
  CurveDB,
  CurveDBError,
  CurveDBParams,
  Observation,
  authority,
  bearing_diff_deg,
  cancel_target,
  haversine_m,
  tighten,
)

# ---------------------------------------------------------------------------------------------
# paths + budgets
# ---------------------------------------------------------------------------------------------
# /data/pnw is the same persistent, outside-the-git-tree home lanecenter_tuning.json and
# lataccel_limits.json already use, so an auto-update's `git clean` cannot delete the corpus the way
# it deleted the mapd binary ([[mapd-binary-wiped-by-autoupdate]]).
CURVEDB_DIR = "/data/pnw"
OBS_PATH = os.path.join(CURVEDB_DIR, "curvedb_obs.jsonl")
CONFIG_PATH = os.path.join(CURVEDB_DIR, "curvedb.json")
# cesarchive2pnw: per-boot snapshots of the observation corpus, for the uploader (PNW_LOG_SOURCES,
# prefix "curvedb_obs.jsonl."). A subdirectory of the corpus's own directory, so a test that redirects
# obs_path never touches /data. Each snapshot is cumulative (<= OBS_MAX_BYTES), so old ones are
# redundant once uploaded; 50 MB keeps 100+ full-size snapshots.
CURVEDB_ARCHIVE_SUBDIR = "curvedb_archive"
CURVEDB_ARCHIVE_MAX_BYTES = 50 * 1024 * 1024

# THE CAP IS A LATENCY BUDGET, NOT A DISK BUDGET (Fable S3).
#
# `CurveDB.build` matches each observation against every row built so far, so it is superlinear:
# measured on the dev host, 2,000 rows -> 0.55 s, 5,000 -> 1.1 s, 10,000 -> 6.5 s, 20,000 -> 24 s.
# ⚠️ THE BUILD DOES RAISE `selfdrivedLagging` WHILE IT RUNS. An earlier version of this comment said
# "+5.2 ms p50, ~2x margin"; that was wrong in BOTH directions (Fable, 2026-09-19). Measured on the
# worst case this code permits (every observation a distinct site -> 4,122 rows, O(n^2) build): the
# main-thread overshoot during a 1.33 s build is **+11.2 ms p50 / +62 ms p99 / +15.8 ms mean**
# against an idle baseline of 0.24 ms. `Ratekeeper.lagging` is a 100-frame AVERAGE over 11.1 ms, so
# any async build (> ASYNC_LOAD_MIN_BYTES) trips it. On the device expect 3-4x longer still.
#
# WHY THAT IS NEVERTHELESS SAFE, and it is the whole argument -- read it before moving this code:
# the load thread is started from `CESController.__init__`, which runs inside `SelfdriveD.__init__`
# (selfdrived.py:217). Nothing can be ENGAGED before selfdrived exists, so `enabled` is necessarily
# False for the entire build. The cost is a **NO_ENTRY window of (build + ~1 s) after ignition-on**,
# never a SOFT_DISABLE mid-drive.
#   => NEVER construct or reload this object after startup. Doing so would move a multi-second
#      GIL-contending build into a window where the car can be engaged, and that IS a soft-disable.
#
# 512 KB, not 1.5 MB, for exactly that reason: the build must finish before a driver could plausibly
# engage. 512 KB is ~1,400 observations and a 0.21 s build on the dev host (~1 s on device); 1.5 MB
# measured 1.33 s here, i.e. plausibly 5+ s on the device -- overlapping the moment someone first
# reaches for the stalk. It is still ~170 driving hours at the measured site rate: on a real
# 46-minute Phase-1 drive ICBM named a map/far candidate at **6 distinct sites** (~8/h). (Discovering
# sites from ANY map candidate would have been 77 in the same 46 minutes -- ~13x -- which is why the
# write half is gated on ICBM's own candidate; see `curvedb_tele`. The broad rate stays recoverable
# OFFLINE from mapLat/mapLon, which `_curve_tele` still resolves on every record.)
#
# At the cap the writer STOPS and says so, loudly and repeatedly: a frozen corpus that announces
# itself is recoverable, a silently rotated one destroys the >= 2-dates history that is the entire
# point of collecting it.
OBS_MAX_BYTES = 512 * 1024
# Below this the boot load is inline; above it, a background thread. ~256 KB is ~800 observations,
# which groups in well under 0.1 s. See the comment at the call site for why a thread is not always
# used -- a background load makes `cdbOn` nondeterministic for the first fraction of a second.
ASYNC_LOAD_MIN_BYTES = 256 * 1024
CONFIG_MAX_BYTES = 64 * 1024
# The load is bounded by the file cap above; this only bounds a pathological single line.
OBS_MAX_LINE = 4096

# ---------------------------------------------------------------------------------------------
# PROVISIONAL constants. Mirrors of tools/curvedb/ingest.py's, pinned equal by
# test_curvedbshadow2pnw.py::TestIngestAgreement -- the car and the replay must measure the same
# thing or the replay validates nothing. Duplicated rather than imported so the control path's only
# `tools` import stays `store` (section 8.3's C1 core), which has zero openpilot imports.
# ---------------------------------------------------------------------------------------------
DRIVE_GAP_S = 300.0          # a gap this long is a new drive (ignition cycle)
MAX_TICK_DT_S = 5.0          # clamp, so one missing second cannot integrate a phantom kilometre
PASSAGE_MAX_M = 80.0         # how close the truck must come for the pass to count as driven
PASSAGE_SCAN_M = 1500.0      # how far past the naming tick to keep looking for the passage
K_MIN_USABLE = 1e-4          # below this a row is degenerate (R > 10 km). Never "the road is straight"
# Straight-line distance must grow by this much past the closest approach before the pass is called
# "driven past". One 1 Hz record is ~25 m at highway speed, so this is sub-record and only exists to
# stop GPS jitter at the apex from latching a passage early.
PASS_HYSTERESIS_M = 5.0
# The comma 3X's RTC battery is dead, so every cold boot stamps records with a pre-sync clock until
# NTP/GPS sync. A bogus timestamp manufactures a distinct *date*, which is half of D6's authority key --
# the same integrity concern that makes a missing tzdata disable the writer. "Pre-sync" is
# time_helpers.wall_time_valid, the definition ces_pnw's clock_bad uses too (clockvalid2pnw: it was a
# 2020 epoch, which let the pre-sync 2026-07-28 file rows under a date that never happened).
# Ceiling on simultaneously-tracked sites. ICBM names one candidate per tick, so this is never
# approached in practice; if it ever is, the eviction is loud (Rule 2) rather than a silent drop.
MAX_TRACKED = 16
# Ceiling on the per-drive dedup memory (two floats each). See `_arm`.
MAX_ARMED_PER_DRIVE = 4000

# Section 3.5's disqualifier bits, identical to ces_pnw's DQ_SAT/DQ_DRIVER/DQ_LANECHG/DQ_BLINKER and
# to ingest's DQ_SAT/DQ_DRV/DQ_LC/DQ_BLNK. Pinned equal to both by the test module.
DQ_SAT, DQ_DRV, DQ_LC, DQ_BLNK = 1, 2, 4, 8
DQ_ALL = DQ_SAT | DQ_DRV | DQ_LC | DQ_BLNK
_DQ_NAMES = ((DQ_SAT, "sat"), (DQ_DRV, "drv"), (DQ_LC, "lc"), (DQ_BLNK, "blnk"))

# Every key `CurveDBShadow.record()` emits, pinned by test_curvedbshadow2pnw.py against a record
# built by the REAL `_event_record`. Same contract as CURVE_TELE_KEYS / CURVELEAD_TELE_KEYS /
# COOP_TELEMETRY_KEYS: a key declared here but never emitted is a null column that reads as "the
# feature did not trigger", which has happened four times in ces_pnw's history (visK, icbmKVis,
# waysel2pnw's eight, lcSpdA).
CURVEDB_TELE_KEYS = (
  # -- health. Rule 2: "no site here" and "the shadow is dead" must never look alike ------------
  "cdbOn",     # str  : "on" | "loading" | "off" | "noStore" | "noTz" | "err"
  "cdbRows",   # int  : rows in the database loaded at boot (0 while loading, and 0 is a real answer)
  "cdbObs",    # int  : observations WRITTEN since boot (the write half's liveness)
  "cdbErr",    # int  : shadow failures since boot. 0 = healthy
  # -- the site this record named --------------------------------------------------------------
  "cdbSite",   # bool : this record named a map candidate the shadow could look up
  # -- THE NUMBER (section 12): did a row exist for this site + approach direction? -------------
  "cdbRow",    # bool | None
  "cdbD",      # float | None: metres from the candidate to the matched row's anchor
  "cdbBrg",    # float | None: degrees between the lookup bearing and the row's
  "cdbBrgD",   # float | None: metres from the truck to the candidate when the bearing was taken
  # -- what the row said ------------------------------------------------------------------------
  "cdbK",      # float | None: k_eff (1/m) = max(k_up, k_down)
  "cdbKUp",    # float | None: section 6.1
  "cdbKDn",    # float | None: section 6.2. Has NEVER been observed anywhere. A non-null is news
  "cdbN",      # int   | None: admissible passes (distinct DRIVES, per D6)
  "cdbDt",     # int   | None: distinct dates
  # -- what it would have done (section 6.4) ----------------------------------------------------
  "cdbRef",    # float | None: the reference speed the cancel formula used (m/s)
  "cdbVRow",   # float | None: sqrt(a_lat / k_eff), capped at posted (m/s). None = no authority
  "cdbWould",  # float | None: min(ref, max(icbm_target, v_row)) (m/s)
  "cdbGive",   # float | None: cdbWould - icbmT (m/s, >= 0). The slowdown it would have cancelled
  "cdbWhy",    # str   | None: the authority reason -- populated on GRANT as well as on refusal
  # -- the write half's verdict on a pass that just completed (one-shot) -------------------------
  "cdbEv",     # str | None: "up" | "down" | "upDown" | "dirty:drv,blnk" | "noK" | "noBrg" | ...
)

_NULL_TELE = dict.fromkeys(CURVEDB_TELE_KEYS)


def _dq_names(bits: int) -> str:
  return ",".join(n for b, n in _DQ_NAMES if int(bits) & b)


def pt_date(t: float, tz) -> str:
  """The PT calendar date of an epoch -- the leave-one-date-out key.

  Mirrors tools/curvedb/ingest.py::pt_date, pinned equal by the test module. `tz` is passed in
  rather than resolved here because a missing tzdata must DISABLE the writer (a row filed under the
  wrong date corrupts leave-one-date-out, which is the whole non-circularity argument), and that
  decision belongs to the caller that can log it."""
  return f"{datetime.datetime.fromtimestamp(t, tz):%Y-%m-%d}"


def _load_config(path: str = CONFIG_PATH) -> tuple[CurveDBParams, bool, str]:
  """`/data/pnw/curvedb.json`, same defensive idiom as pnw_vehicle's curve.json / rain.json.

  Returns (params, enabled, note). Missing file -> PROVISIONAL_PARAMS, enabled. A malformed file is
  NOT silently ignored: `note` names what went wrong and the caller logs it.

  Only the matching/authority half is tunable, and that is the point -- `cdbD`/`cdbBrg` are logged
  on every lookup precisely so `site_radius_m` and `heading_tol_deg` can be chosen from real data
  without a deploy. Nothing here can grant authority: there is no authority to grant."""
  enabled, note = True, ""
  overrides: dict = {}
  try:
    st = os.stat(path)
    if not stat.S_ISREG(st.st_mode):
      return PROVISIONAL_PARAMS, True, f"{path} is not a regular file"
    if st.st_size > CONFIG_MAX_BYTES:
      return PROVISIONAL_PARAMS, True, f"{path} is {st.st_size} B, over the {CONFIG_MAX_BYTES} B cap"
    with open(path) as f:
      data = json.load(f)
    if not isinstance(data, dict):
      return PROVISIONAL_PARAMS, True, f"{path} is a {type(data).__name__}, not an object"
    if "enabled" in data:
      enabled = bool(data["enabled"])
    # ONLY the matching half (Fable S8). The measurement half (`extent_*`,
    # `approach_bearing_ref_m`, `down_trigger_a_lat_ms2`, `min_speed_ms`) decides what a stored row
    # MEANS, and an `Observation` carries no record of the params it was measured under -- so a
    # hand edit would silently make new rows non-comparable with old ones and with the replay's.
    # `a_lat_comfort_ms2`/`min_passes`/`min_dates` are policy and belong in reviewed Python.
    for key in ("site_radius_m", "heading_tol_deg"):
      if key in data:
        overrides[key] = float(data[key])
    unknown = sorted(set(data) - {"enabled", "site_radius_m", "heading_tol_deg"})
    if unknown:
      note = f"{path}: ignoring {unknown} -- only the matching half is tunable"
  except FileNotFoundError:
    return PROVISIONAL_PARAMS, True, ""
  except Exception as e:
    # Rule 2: a config that cannot be read is NOT "the defaults are what you asked for".
    return PROVISIONAL_PARAMS, True, f"{path} unreadable ({type(e).__name__}: {e})"
  if not overrides:
    return PROVISIONAL_PARAMS, enabled, note
  try:
    # CurveDBParams.__post_init__ rejects non-finite / non-positive / >= 180 deg values, so a bad
    # override lands here rather than in the matcher.
    return tighten(PROVISIONAL_PARAMS, **overrides), enabled, note
  except (CurveDBError, TypeError) as e:
    return PROVISIONAL_PARAMS, enabled, f"{path} overrides rejected ({e}); using PROVISIONAL_PARAMS"


class _Site:
  """One map candidate being followed from the tick that named it until 150 m past it."""
  __slots__ = ("lat", "lon", "s_armed", "bearing", "bearing_d", "min_d", "s_min",
               "open_s", "k_peak", "dq_bits", "n_ticks", "down_k", "posted", "hwy", "t")

  def __init__(self, lat: float, lon: float, s: float, t: float):
    self.lat, self.lon = lat, lon
    self.s_armed = s
    # The epoch of CLOSEST APPROACH once the pass has been driven (the arming time until then), so
    # the observation is timestamped where ingest timestamps it: `drive.ticks[p.i_passage].t`.
    self.t = t
    self.bearing: float | None = None
    self.bearing_d: float | None = None       # distance the approach bearing was sampled at
    self.min_d = float("inf")
    self.s_min: float | None = None           # odometer at closest approach = ingest's `s_p`
    self.open_s: float | None = None          # s at which the extent opened, None = not yet
    self.k_peak = 0.0
    self.dq_bits = 0
    self.n_ticks = 0
    self.down_k: float | None = None          # section 6.2: |slKCmd| at the qualifying override
    self.posted: float | None = None
    self.hwy: str | None = None


class CurveDBShadow:
  """The on-car shadow. Constructed once per selfdrived start; never raises into the control loop.

  Two public methods, and the shapes are load-bearing for the read-boundary test:
  * `tick(...) -> None`  -- 100 Hz, accumulates. Returns nothing, so its call site must discard.
  * `record(...) -> dict` -- ~1 Hz, advances the passage tracker and predicts. Its return is a
    telemetry fragment whose only permitted destination is a `**` splat into the record dict.
  """

  # Class-level defaults, so a HALF-CONSTRUCTED instance still answers every attribute the 100 Hz
  # path touches. `_fail` incrementing `self.err` on an object whose `__init__` raised would be an
  # error handler that itself raises -- which is worse than none, and which this file's own boundary
  # test caught on the first run.
  platform = ""
  params = PROVISIONAL_PARAMS
  err = 0
  obs_written = 0
  _state = "err"
  _db = None
  _tz = None
  _enabled = False
  _obs_path = OBS_PATH
  _write_stopped = True
  _load_done = True
  _err_t = 0.0
  _k_peak = 0.0
  _dq_bits = 0
  _down_k = None
  _s = 0.0
  _last_t = None
  _last_v = None
  _drive_id = None
  _drive_t0 = None
  _armed = ()
  _ev_pending = ()

  def __init__(self, platform: str, obs_path: str | None = None, config_path: str | None = None,
               load_async: bool = True):
    try:
      self._init(platform, obs_path, config_path, load_async)
    except Exception as e:
      # ces_pnw constructs this inside CESController.__init__, which selfdrived constructs at
      # startup. A raise here would take selfdrived down at boot -- for a feature that is not
      # allowed to influence anything. The class defaults above leave a inert, honest object.
      self._state = "err"
      self.err = 1
      try:
        cloudlog.exception(f"curvedb_shadow: construction FAILED ({type(e).__name__}: {e}) -- " +
                           "the shadow is INERT for this boot; cdbOn will read 'err'")
      except Exception:
        pass

  def _init(self, platform, obs_path, config_path, load_async) -> None:
    # Paths resolved from the module globals rather than defaulted in the signature, because a
    # default argument is bound at def time and `monkeypatch.setattr(curvedb_shadow, "OBS_PATH",
    # tmp)` would then silently NOT redirect the test -- which on a dev host means writing to the
    # real /data/pnw. Same idiom pnw_vehicle's CURVE_CONFIG_PATH uses for the same reason.
    self.platform = str(platform or "")
    self._obs_path = OBS_PATH if obs_path is None else obs_path
    self.params, self._enabled, note = _load_config(CONFIG_PATH if config_path is None else config_path)
    if note:
      cloudlog.error(f"curvedb_shadow: {note}")

    self.err = 0
    self.obs_written = 0
    self._db: CurveDB | None = None
    # "off" and "noTz" are TERMINAL -- `_load` may only promote "loading" to "on", never overwrite a
    # state that records a disabled half. Without that rule a successful load would paper over a
    # missing tzdata and `cdbOn` would read "on" while the write half was silently dead.
    self._state = "off" if not self._enabled else "loading"
    self._sites: list[_Site] = []
    self._armed: list[tuple] = []      # every site armed this drive, for `find_sites`-style dedup
    self._s = 0.0                      # cumulative odometer, integrated exactly as split_drives does
    self._last_t: float | None = None
    self._last_v: float | None = None
    self._drive_id: str | None = None
    self._drive_t0: float | None = None
    self._ev_pending: list[str] = []
    self._write_stopped = False
    self._load_done = False
    self._err_t = 0.0

    # 100 Hz accumulator, drained by `record`. Its own, deliberately: sharing ces_pnw's `_curve_peak`
    # would couple two features through a destructive `take()` and make each one's window depend on
    # whether the other ran.
    self._k_peak = 0.0
    self._dq_bits = 0
    self._down_k: float | None = None   # the first qualifying override inside this window
    # No tick COUNTER here on purpose: `_k_peak` resets to 0.0 with every drain, so "the window ran
    # zero control ticks" and "the window measured nothing" are already the same reading -- and
    # Phase 1's `kPeakN`, fed from the identical call site, is the liveness witness in the same
    # record. A second counter would be a field nobody reads.

    # A row filed under the wrong calendar date silently corrupts leave-one-date-out, which is the
    # entire non-circularity argument -- so a missing tzdata DISABLES the writer rather than guessing
    # an offset. AGNOS is a minimal rootfs and this has never been verified on it.
    self._tz = None
    if self._enabled:
      try:
        import zoneinfo
        self._tz = zoneinfo.ZoneInfo("America/Los_Angeles")
      except Exception as e:
        self._state = "noTz"
        cloudlog.error("curvedb_shadow: no America/Los_Angeles tzdata " +
                       f"({type(e).__name__}: {e}) -- the WRITE half is disabled; a pass filed " +
                       "under a UTC date would corrupt leave-one-date-out")

    # The store's directory is created by CESController.__init__ (for ces_events.jsonl) BEFORE this
    # object is built, so on the car it always exists. Where it does not -- a dev host, a test that
    # did not redirect the paths -- the WRITE half is disabled with its own terminal state rather
    # than failing once per passage: an err counter that ticks up on a machine with no /data says
    # "the shadow is broken" when the truth is "this is not a comma".
    if self._enabled and not os.path.isdir(os.path.dirname(self._obs_path) or "."):
      self._state = "noStore"
      self._write_stopped = True
      # Rule 2, corrected (Fable S5). `cdbOn == "noStore"` was argued to be the loud channel
      # because it rides every record -- but on the CAR the only way /data/pnw is missing is
      # `os.makedirs` failing in CESController.__init__, which sets `_event_log_ok = False`, and
      # `_append_event` then writes NOTHING. In precisely the failure case, the claimed channel is
      # dead. So: log an ERROR where there is a /data at all (a device), and stay quiet where there
      # is not (a dev host, where this would fire on every CESController construction and trip
      # ces_pnw's own "a healthy drive logs no errors" invariants -- this feature degrading
      # another's signal).
      if os.path.isdir("/data"):
        cloudlog.error(f"curvedb_shadow: {os.path.dirname(self._obs_path)} does not exist -- " +
                       "the WRITE half is disabled; no passes will be recorded this boot")

    if self._enabled:
      # cesarchive2pnw: snapshot the corpus as of this boot into curvedb_archive/ for upload. It is
      # the live database and is only ever appended to (never rotated -- see OBS_MAX_BYTES), so the
      # uploader must never take the live file; a copy is taken instead, here, where the shadow is
      # built before selfdrived can engage (<= OBS_MAX_BYTES, a few ms). Skipped when the content is
      # unchanged since the last snapshot. Never raises; logs its own failures.
      archive_dir = os.path.join(os.path.dirname(self._obs_path) or ".", CURVEDB_ARCHIVE_SUBDIR)
      pnw_log_archive.snapshot_into_archive(self._obs_path, archive_dir, os.path.basename(self._obs_path))
      pnw_log_archive.prune_archive(archive_dir, os.path.basename(self._obs_path) + ".", CURVEDB_ARCHIVE_MAX_BYTES)
      size = 0
      try:
        size = os.path.getsize(self._obs_path)
      except OSError:
        pass
      # The thread exists ONLY because grouping a big corpus is slow (measured: 1.0 s for 3,000
      # observations, 11.9 s for 15,000 -- CurveDB.build matches each observation against every row
      # built so far, and the Snapdragon is slower still). An absent or small corpus is loaded
      # INLINE, which keeps a fresh device and every test deterministic: with a thread, `cdbOn`
      # reads "loading" or "on" depending on scheduling, and ces_pnw's own run-twice record-identity
      # invariants go flaky. (Found exactly that way, by test_ces_mode_hold.)
      if load_async and size > ASYNC_LOAD_MIN_BYTES:
        threading.Thread(target=self._load, name="curvedb_load", daemon=True).start()
      else:
        self._load()

  # -- boot load ------------------------------------------------------------------------------

  def _load(self) -> None:
    """Parse the append-only observation log and group it into rows. Off the constructor's thread
    when the corpus is large enough to be worth it (see OBS_MAX_BYTES for the measured cost, and for
    why the cap on that cost is a LATENCY budget).

    A torn last line is DISCARDED (section 8.1's append-only idiom); anything else that cannot be
    parsed is counted and named. Failing to a database of zero rows is correct; failing to one
    SILENTLY is the Rule 2 violation this whole project keeps paying for."""
    try:
      t0 = time.monotonic()
      obs, bad, n_lines = [], 0, 0
      try:
        size = os.path.getsize(self._obs_path)
      except OSError:
        size = 0
      try:
        with open(self._obs_path) as f:
          # EXACTLY the bytes that existed at the stat above (Fable C2). The control thread may be
          # appending while this runs, and a line that landed DURING the load would enter the live
          # database -- this drive judged against itself, which is the one property section 3.9
          # item 6 asks for. (A torn tail is still handled: it simply fails to parse.)
          # The WRITER stops at the cap, but a hand-placed or concatenated file is not bound by that
          # (Fable 2026-09-19). A 20 MB file builds in ~24 s on the dev host and minutes on the
          # device -- i.e. a minute-plus NO_ENTRY window after ignition, from a file nobody meant to
          # leave here. Bound the READ too, and say so: a silently truncated database would be worse
          # than a slow one.
          read_n = min(size, OBS_MAX_BYTES + OBS_MAX_LINE)
          if read_n < size:
            cloudlog.error(f"curvedb_shadow: {self._obs_path} is {size} B, over the {OBS_MAX_BYTES} B cap -- "
                           f"only the first {read_n} B are loaded; the tail is NOT in this boot's database")
          for line in f.read(read_n).splitlines(True):
            n_lines += 1
            if len(line) > OBS_MAX_LINE:
              bad += 1
              continue
            line = line.strip()
            if not line:
              continue
            try:
              obs.append(Observation(**json.loads(line)))
            except Exception:
              bad += 1
      except FileNotFoundError:
        pass                              # a fresh device. Zero rows, and cdbRows says so.
      db = CurveDB.build(obs, self.params)
      dates = sorted({o.date for o in obs})
      self._db = db
      if self._state == "loading":          # never promote over a terminal "off"/"noTz"
        self._state = "on"
      # The startup line that makes "the database is as of the last boot" checkable rather than
      # asserted: whoever reads a drive's cdbRow knows exactly what corpus produced it.
      # INFO, not ERROR. `cloudlog.event` routes to .error() whenever an `error=` kwarg is present
      # AT ALL (common/logging_extra.py:159 -- `if "error" in kwargs`, not `if kwargs["error"]`), and
      # ces_pnw's own invariant tests assert a healthy drive logs NO errors. A boot line is not a
      # failure; the failures below are, and they use cloudlog.error/exception.
      cloudlog.event("curvedb_shadow_loaded", rows=len(db.rows), obs=len(obs),
                     bad_lines=bad, lines=n_lines, bytes=size,
                     first_date=(dates[0] if dates else None),
                     last_date=(dates[-1] if dates else None),
                     load_s=round(time.monotonic() - t0, 2), platform=self.platform,
                     site_radius_m=self.params.site_radius_m,
                     heading_tol_deg=self.params.heading_tol_deg)
      if bad:
        cloudlog.error(f"curvedb_shadow: {bad} of {n_lines} observation lines were unparseable")
      if size > OBS_MAX_BYTES:
        self._write_stopped = True
        cloudlog.error(f"curvedb_shadow: {self._obs_path} is {size} B, over the " +
                       f"{OBS_MAX_BYTES} B cap -- the WRITE half has STOPPED. The corpus is " +
                       "frozen, not rotated: pull it off the device and truncate it deliberately.")
    except Exception as e:
      self._state = "err"
      self.err += 1
      cloudlog.exception(f"curvedb_shadow: boot load FAILED ({type(e).__name__}: {e}) -- " +
                         "the shadow database is EMPTY for this drive, cdbRow will read null")
    finally:
      # Set LAST and unconditionally: `cdbWhy` reads "loading" until this flips, so a load that
      # died would otherwise read as "still working on it" forever.
      self._load_done = True

  def wait_loaded(self, timeout: float = 30.0) -> bool:
    """Test/diagnostic helper: block until the boot load thread has settled. NOT called by any
    control code -- the boundary test asserts `ces_pnw.py` never names it."""
    deadline = time.monotonic() + timeout
    while not self._load_done and time.monotonic() < deadline:
      time.sleep(0.01)
    return self._load_done

  # -- the 100 Hz half ------------------------------------------------------------------------

  def tick(self, k_actl, k_cmd, k_pose, dq_bits: int, v_ego, str_prs: bool) -> None:
    """One control tick. Accumulates section 3.3's peak and section 6.2's override witness.

    RETURNS NOTHING, on purpose: the read-boundary test asserts every call site discards it, and a
    method that cannot hand anything back cannot leak into control even by accident.

    Section 3.3's estimator is `max(|commanded|, |achieved|)` -- never achieved alone. Achieved is
    bounded by steering authority, so where the truck cannot follow it UNDER-reads the road,
    precisely on the curves that matter (D2), and an under-read `k` derives too high a speed.

    Never raises -- selfdrived is `restart_if_crash=False` at 100 Hz. Failures are counted; `cdbErr`
    carries the count into every record, so a dead accumulator is visible in the data itself."""
    try:
      if self._state == "off":
        return
      # `k_peak` is `max(|achieved-from-CAN|, |commanded|)` and DELIBERATELY excludes `k_pose`
      # (Fable S2). That is exactly the quantity ces_pnw logs as `kPeak` and the ONLY quantity
      # ingest's `_tick_k` reads, and a stored row is labelled `estimator="kPeak100"` -- so folding
      # the localizer in would make the car's rows and the replay's rows two different numbers
      # under one label, on the Tesla in particular (where `k_actl` is dead by construction and the
      # row would have been pose-only on the car and commanded-only in the replay).
      # `k_pose` still feeds `k_ach`, the lateral-accel WITNESS section 6.2's trigger needs -- which
      # is the Tesla's only achieved reading (P1-A) and is not stored in any row.
      k_peak = None       # max(|k_actl|, |k_cmd|): section 3.3's estimator, == ces_pnw's kPeak
      k_ach = None        # max over the ACHIEVED pair: the lateral-accel witness DOWN wants
      for k, achieved, stored in ((k_actl, True, True), (k_cmd, False, True), (k_pose, True, False)):
        if k is None:
          continue
        try:
          a = abs(float(k))
        except (TypeError, ValueError):
          continue
        if not math.isfinite(a):
          continue
        if stored:
          k_peak = a if k_peak is None else max(k_peak, a)
        if achieved:
          k_ach = a if k_ach is None else max(k_ach, a)
      if k_peak is not None and k_peak > self._k_peak:
        self._k_peak = k_peak
      if dq_bits:
        self._dq_bits |= int(dq_bits)
      # Section 6.2's trigger: a steering override taken while the road was ACTUALLY loading the
      # truck. Of 2,986 overrides above 27 mph only ~113 ticks cleared 2.5 m/s^2; the rest are lane
      # changes, exits and repositioning. The witness is the ACHIEVED lateral accel where there is
      # one (`k_ach * v^2`, matching ingest's `abs(tk.ach_lat)`), falling back to the full peak --
      # `k_actl` is dead on the Tesla by construction (D1), which is what `k_pose` is for.
      # `k_cmd` is the MAGNITUDE source. Section 6.2 writes it as `|slKCmd|`, and this is fed
      # `controlsState.desiredCurvature`: controlsd sets `cs.desiredCurvature = self.desired_curvature`
      # and `slKCmd = -self.desired_curvature` (the sign flip into the Ford positive=left wire
      # convention, controlsd.py:532) -- so the two differ ONLY in sign and this takes `abs`. Same
      # number, and the offline DOWN magnitude and this one are therefore the same quantity.
      # An override with no commanded curvature is not a DOWN: it is dropped, with the pass's
      # verdict saying so.
      if str_prs and self._down_k is None and k_cmd is not None:
        v = _f(v_ego)
        kc = _f(k_cmd)
        if v is None or kc is None:
          return
        kc = abs(kc)
        a_lat = (k_ach if k_ach is not None else k_peak or 0.0) * v * v
        if (v >= self.params.min_speed_ms and math.isfinite(a_lat)
            and a_lat >= self.params.down_trigger_a_lat_ms2 and kc >= K_MIN_USABLE):
          self._down_k = kc
    except Exception as e:
      self._fail("tick", e)

  # -- the ~1 Hz half -------------------------------------------------------------------------

  def record(self, *, now_wall, lat, lon, bearing, v_ego, site_pt, posted_ms, hwy_class,
             icbm_src, icbm_target_ms, ref_ms) -> dict:
    """Advance the write half, then answer the read half. One telemetry fragment per record.

    The keyword-only signature is not decoration: `posted_ms` and `ref_ms` are both speeds and
    `icbm_src` and `hwy_class` are both strings, and a positional mix-up between any of those pairs
    would produce a plausible wrong answer rather than an error."""
    try:
      return self._record(now_wall=now_wall, lat=lat, lon=lon, bearing=bearing, v_ego=v_ego,
                          site_pt=site_pt, posted_ms=posted_ms, hwy_class=hwy_class,
                          icbm_src=icbm_src, icbm_target_ms=icbm_target_ms, ref_ms=ref_ms)
    except Exception as e:
      self._fail("record", e)
      return {**_NULL_TELE, "cdbOn": "err", "cdbRows": self._rows(), "cdbObs": self.obs_written,
              "cdbErr": self.err, "cdbSite": False}

  def _record(self, *, now_wall, lat, lon, bearing, v_ego, site_pt, posted_ms, hwy_class,
              icbm_src, icbm_target_ms, ref_ms) -> dict:
    out = {**_NULL_TELE, "cdbOn": self._state, "cdbRows": self._rows(),
           "cdbObs": self.obs_written, "cdbErr": self.err, "cdbSite": False}
    if self._state == "off":
      return out

    v = _f(v_ego)
    la, lo = _f(lat), _f(lon)
    # The odometer, integrated exactly as ingest's `split_drives` does -- the TRAPEZOID over the two
    # end speeds, not the current speed held over the whole interval -- so the extents the car
    # measures and the extents the replay measures are the same quantity and not merely similar.
    t = _f(now_wall)
    stale = True
    if t is not None:
      dt = None if self._last_t is None else t - self._last_t
      if dt is not None and dt >= 0.0 and v is not None:
        # CLAMPED, not skipped (Fable C3): ingest's `split_drives` adds `0.5*(v_a+v_b)*min(dt, 5)`
        # across a gap, so skipping it entirely would put the car's `s` and the replay's `s`
        # permanently out of step after the first missed record.
        self._s += 0.5 * (max(self._last_v or 0.0, 0.0) + max(v, 0.0)) * min(dt, MAX_TICK_DT_S)
        stale = dt > MAX_TICK_DT_S
      # A gap longer than a traffic light and shorter than a charging stop is a new DRIVE, and
      # `n_passes` counts distinct drives (D6) -- so without this a single un-rebooted session could
      # never accumulate the two passes authority requires.
      if (self._last_t is None or not 0.0 <= t - self._last_t <= DRIVE_GAP_S
          or self._drive_id is None):
        self._drive_t0 = t
        self._drive_id = f"{self.platform}:{t:.2f}"
        self._sites.clear()
        self._armed.clear()
      self._last_t = t
      self._last_v = v

    peak, dq_bits, down_k = self._k_peak, self._dq_bits, self._down_k
    self._k_peak, self._dq_bits, self._down_k = 0.0, 0, None
    # (2) DISCARD a window nobody drained in time. `tick` runs unconditionally at 100 Hz (it measures
    # steering/localizer facts, like _curve_peak_step), but `record` only runs while CES is ENABLED --
    # so across a CES-off stretch the accumulator would keep OR-ing disqualifiers and holding a peak
    # for minutes, and the first passage after CES came back would inherit all of it. The bound is
    # MAX_TICK_DT_S, the same "one missing second must not integrate a phantom kilometre" clamp the
    # odometer above already uses: a window spanning more than that is not this second's measurement.
    if stale:
      peak, dq_bits, down_k = 0.0, 0, None

    pt = _pt(site_pt)
    # NORMALISED ONCE (Fable S1). `Observation.__post_init__` rejects a bearing outside [0, 360), and
    # a GPS fix reporting exactly 360.0 would make `_finish` raise inside `_advance` -- unwinding
    # before the `self._sites = keep` rebuild, so the offending site is never removed and EVERY
    # subsequent record re-raises, killing the write half for the rest of the drive segment.
    # `ingest.normalise` does `bearing %= 360.0`; so does this, now.
    brg = _f(bearing)
    if brg is not None:
      brg %= 360.0

    # -- WRITE: advance every tracked site, then arm a new one if this record names one -----------
    if la is not None and lo is not None and v is not None:
      self._advance(now_wall=t, lat=la, lon=lo, bearing=brg, v_ego=v, peak=peak,
                    dq_bits=dq_bits, down_k=down_k, posted_ms=_f(posted_ms), hwy_class=hwy_class)
      if pt is not None and v >= self.params.min_speed_ms:
        self._arm(pt, t)
    # A record can complete more than one passage; joining rather than overwriting means the second
    # verdict is not silently dropped (Fable C5).
    out["cdbEv"] = ";".join(self._ev_pending) if self._ev_pending else None
    self._ev_pending = []

    # -- READ: the prediction ---------------------------------------------------------------------
    if pt is None:
      return out
    out["cdbSite"] = True
    # Bound ONCE: `_load` assigns `self._db` from the background thread, and reading it twice could
    # see None on the first read and a database on the second (or the reverse). The assignment itself
    # is atomic in CPython; two reads of it are not one observation.
    db = self._db
    if db is None:
      # `_load_done` distinguishes "still parsing" from a terminal state that happens to have no
      # database (Fable C5): `noTz` disables only the WRITE half, so reporting it here would tell an
      # analyst the read half was off when it was merely not ready yet.
      out["cdbWhy"] = self._state if self._load_done else "loading"
      return out
    # THE APPROACH BEARING, SYMMETRICALLY (Fable S6). The write half keys a row on the bearing
    # sampled at `approach_bearing_ref_m`; looking it up on the truck's heading HERE -- which in the
    # bend differs by the curve's sweep (median 19 deg, p90 62 deg per section 6.3) -- would make the
    # same site match on the approach and miss inside the bend, biasing `cdbRow`, the one number this
    # whole feature exists to measure, LOW. When the candidate is one the tracker is already
    # following, its own sampled approach bearing is used; `cdbBrgD` then reports the distance THAT
    # bearing was taken at, so the two cases stay distinguishable in the data. Section 12 flagged
    # that section 6.3 never states the reference distance; this at least makes both ends agree.
    #
    # Once the passage completes the site stops being followed and the lookup falls back to the
    # instantaneous heading. By then the candidate is `extent_fwd_m` behind the truck and ICBM is
    # not acting on it (behindgate2pnw), so the fallback only affects records that were never going
    # to be adjudicated -- and `cdbBrgD` still says which bearing was used.
    lk_brg, lk_d = brg, (None if la is None or lo is None
                         else round(haversine_m(la, lo, pt[0], pt[1]), 1))
    for site in self._sites:
      if site.bearing is not None and haversine_m(site.lat, site.lon, pt[0], pt[1]) <= self.params.site_radius_m:
        lk_brg, lk_d = site.bearing, (None if site.bearing_d is None else round(site.bearing_d, 1))
        break
    if lk_brg is None:
      # Rule 2: direction is half the key, so no fix is a REFUSAL with a reason -- never a lookup
      # that silently used north.
      out["cdbWhy"] = "no bearing"
      return out
    out["cdbBrgD"] = lk_d
    row = db.match(pt[0], pt[1], lk_brg)
    out["cdbRow"] = row is not None
    if row is not None:
      out["cdbD"] = round(haversine_m(pt[0], pt[1], row.site_lat, row.site_lon), 1)
      out["cdbBrg"] = round(bearing_diff_deg(lk_brg, row.bearing_deg), 1)
      out["cdbK"] = _r6(row.k_eff)
      out["cdbKUp"] = _r6(row.k_up)
      out["cdbKDn"] = _r6(row.k_down)
      out["cdbN"] = row.n_passes
      out["cdbDt"] = len(row.dates)
    auth = authority(row, posted_limit_ms=_f(posted_ms), highway_class=hwy_class,
                     platform=self.platform, params=self.params, envelopes=PROVISIONAL_ENVELOPES)
    out["cdbWhy"] = auth.reason
    out["cdbVRow"] = None if auth.v_row_ms is None else round(auth.v_row_ms, 2)
    icbm_t, ref = _f(icbm_target_ms), _f(ref_ms)
    if icbm_src not in ("map", "far"):
      # Not a hole: the cancel formula only ever applies to a map/far slowdown. Named so that a null
      # cdbWould is never read as "the database declined".
      out["cdbWhy"] = f"{auth.reason}; icbm src {icbm_src!r}"
      return out
    if icbm_t is None or ref is None:
      out["cdbWhy"] = f"{auth.reason}; no icbm target/reference to compare against"
      return out
    out["cdbRef"] = round(ref, 2)
    would = cancel_target(ref, icbm_t, auth.v_row_ms if auth.granted else None)
    out["cdbWould"] = round(would, 2)
    out["cdbGive"] = round(would - icbm_t, 2)
    return out

  # -- the passage tracker --------------------------------------------------------------------

  def _arm(self, pt, t) -> None:
    """Start following a candidate, deduplicated at the matcher's OWN radius, against every site
    armed SO FAR IN THIS DRIVE -- not merely the ones still in flight.

    Deduplicating at `site_radius_m` mirrors ingest's `find_sites`: two points the matcher would
    treat as one row must contribute ONE pass, or a single drive manufactures the multiple passes D6
    requires. Doing it against the whole drive (as `find_sites` does) rather than the in-flight list
    is what stops a candidate the truck has just finished measuring from being re-armed the moment
    the passage closes -- which produced a SECOND observation of the same site from one drive, made
    of nothing but that curve's run-out. `_armed` is cleared with the drive."""
    for la, lo in self._armed:
      if haversine_m(la, lo, pt[0], pt[1]) <= self.params.site_radius_m:
        return
    if len(self._sites) >= MAX_TRACKED:
      dropped = self._sites.pop(0)
      cloudlog.error(f"curvedb_shadow: {MAX_TRACKED} sites in flight, evicting the oldest " +
                     f"({dropped.lat:.5f},{dropped.lon:.5f}) -- passes are being lost")
    if len(self._armed) >= MAX_ARMED_PER_DRIVE:
      # Rule 2: at ~8 ICBM sites/hour this is ~500 driving hours in one un-rebooted session, so it
      # is unreachable in practice -- but a silently forgotten prefix would let the drive re-arm
      # sites it already measured, which is the exact defect the dedup above exists to prevent.
      cloudlog.error(f"curvedb_shadow: {MAX_ARMED_PER_DRIVE} sites armed in one drive; the " +
                     "oldest is being forgotten and may be re-armed and re-measured")
      del self._armed[0]
    self._armed.append((pt[0], pt[1]))
    self._sites.append(_Site(pt[0], pt[1], self._s, t if t is not None else 0.0))

  def _advance(self, *, now_wall, lat, lon, bearing, v_ego, peak, dq_bits, down_k,
               posted_ms, hwy_class) -> None:
    keep = []
    for s in self._sites:
      d = haversine_m(lat, lon, s.lat, s.lon)
      # The approach bearing, sampled where ingest samples it (section 6.3). `bearing_d` records
      # where it ACTUALLY landed, because a site armed closer than the reference distance can never
      # be sampled there and "we used whatever we had" must be visible, not assumed.
      if s.bearing is None and bearing is not None and d <= self.params.approach_bearing_ref_m:
        s.bearing, s.bearing_d = bearing, d
      if d < s.min_d:
        s.min_d = d
        s.s_min = self._s                   # ingest's `s_p`: the odometer at closest approach
        if now_wall is not None:
          s.t = now_wall                    # timestamp the pass at closest approach, as ingest does
      # The extent opens either at section 6.3's back edge, or -- for a candidate the truck passes
      # wide of, which mapd's point routinely is (56-125 m from the bend) -- at the moment it starts
      # receding, provided it came within PASSAGE_MAX_M at all.
      if s.open_s is None and (d <= self.params.extent_back_m
                               or (s.min_d <= PASSAGE_MAX_M and d > s.min_d + PASS_HYSTERESIS_M)):
        s.open_s = self._s
      if s.open_s is not None:
        if v_ego >= self.params.min_speed_ms:
          s.n_ticks += 1
          if peak > s.k_peak:
            s.k_peak = peak
          s.dq_bits |= dq_bits
          if s.down_k is None and down_k is not None:
            s.down_k = down_k
          if posted_ms and posted_ms > 0.0:
            s.posted = posted_ms
          if hwy_class:
            s.hwy = hwy_class
        # Closed at `s_p + extent_fwd_m`, where `s_p` is CLOSEST APPROACH -- ingest's own extent
        # (Fable C3). Measuring `extent_back + extent_fwd` forward from the OPEN instead made a
        # wide pass (one that opens late, on recession) cover [s_p+5, s_p+180] rather than
        # [s_p-25, s_p+150]: a different stretch of road under the same rule. `s_min` keeps moving
        # until the truck starts receding, which is exactly when it should stop.
        if s.s_min is not None and self._s - s.s_min >= self.params.extent_fwd_m:
          try:
            self._finish(s)
          except Exception as e:
            # A raise here (a rejected Observation, a bad date) would unwind BEFORE the
            # `self._sites = keep` rebuild below, leaving the offending site in flight so every
            # later record re-raises -- killing the write half, and every OTHER site with it, for
            # the rest of the drive segment (Fable S1). The site is dropped and the failure is
            # counted and named instead.
            self._fail("finish", e)
            self._ev_pending.append("finishFailed")
          continue
      elif self._s - s.s_armed > PASSAGE_SCAN_M:
        # Rule 2: a pass that never reached its candidate is a COUNTED drop, never a measurement.
        self._ev_pending.append(f"notReached:{s.min_d:.0f}m")
        cloudlog.event("curvedb_shadow_pass", verdict="notReached",
                       min_d=round(s.min_d, 1), lat=round(s.lat, 6), lon=round(s.lon, 6))
        continue
      keep.append(s)
    self._sites = keep

  def _finish(self, s: _Site) -> None:
    """One completed pass -> up to two observations. Section 6.1 and section 6.2, independently.

    **THE DOWN TEST IS NOT NESTED INSIDE THE UP TEST.** `observations_for_drive` drops a
    disqualified passage before ever reaching its DOWN loop, and a driver steering override IS a
    disqualifier -- which is why section 6.2 has produced zero observations on 2.2 GB of corpus.
    A driver's "too fast" is not a contaminated reading of the road; it is the reading section 6.2
    asked for."""
    verdicts = []
    if self._tz is None:
      # `datetime.fromtimestamp(t, None)` does NOT raise -- it returns LOCAL time, which on this
      # UTC device would file every row under the wrong PT date and silently corrupt
      # leave-one-date-out. Refuse here rather than write a plausible lie.
      self._ev_pending.append("noTz")
      return
    if not wall_time_valid(s.t):
      # Same integrity argument as `noTz`, for the other way the date can be wrong (Fable S7): the
      # 3X's RTC battery is dead, so a cold boot stamps 1970 until NTP/GPS sync -- and a bogus
      # timestamp manufactures a distinct DATE, which is half of D6's authority key. ces_pnw MARKS
      # such records (`clockBad`) rather than dropping them, because a record's other fields are
      # still real; a row's date is not one of its fields, it is its identity.
      self._ev_pending.append("clockBad")
      cloudlog.event("curvedb_shadow_pass", verdict="clockBad", t=s.t,
                     lat=round(s.lat, 6), lon=round(s.lon, 6))
      return
    common = dict(car=self.platform, drive_id=self._drive_id or "unknown",
                  site_lat=s.lat, site_lon=s.lon, site_src="logged",
                  posted_ms=s.posted, highway_class=s.hwy, dq_src="rollup100")
    # -- section 6.1 UP ------------------------------------------------------------------------
    if s.bearing is None:
      verdicts.append("noBrg")
    elif s.k_peak < K_MIN_USABLE:
      verdicts.append("noK")
    elif s.dq_bits & DQ_ALL:
      verdicts.append("dirty:" + _dq_names(s.dq_bits))
    else:
      # The verdict reports whether the observation was actually WRITTEN (Fable S4). Saying "up"
      # while the writer is frozen at its cap -- the very state section 8.1 says must be loud --
      # would make the per-record channel lie about the one thing it exists to report.
      verdicts.append("up" if self._write(Observation(
        date=self._date(s.t), t=s.t, bearing_deg=s.bearing, k=s.k_peak, kind="up",
        estimator="kPeak100", source="curvedb_shadow", n_ticks=s.n_ticks,
        dq_state="clean", **common)) else f"upDropped:{self._drop_why()}")
    # -- section 6.2 DOWN, evaluated regardless of the above ------------------------------------
    if s.down_k is not None:
      if s.bearing is None:
        verdicts.append("downNoBrg")
      else:
        # `dq_state="clean"` matches ingest's DOWN observation exactly: the override is the
        # measurement, so it is not a disqualifier OF ITSELF. DOWN raises k_eff and therefore only
        # ever LOWERS the derived speed (section 6.2's hard constraint), which is why `CurveRow`
        # can express "DOWN is never overwritten by UP" (D8) as a plain max.
        verdicts.append("down" if self._write(Observation(
          date=self._date(s.t), t=s.t, bearing_deg=s.bearing, k=s.down_k, kind="down",
          estimator="slKCmd_at_override", source="curvedb_shadow", n_ticks=1,
          dq_state="clean", **common)) else f"downDropped:{self._drop_why()}")
    verdict = "+".join(verdicts) if verdicts else "none"
    self._ev_pending.append(verdict)
    cloudlog.event("curvedb_shadow_pass", verdict=verdict,
                   lat=round(s.lat, 6), lon=round(s.lon, 6),
                   bearing=(None if s.bearing is None else round(s.bearing, 1)),
                   bearing_d=(None if s.bearing_d is None else round(s.bearing_d, 1)),
                   min_d=round(s.min_d, 1), k=round(s.k_peak, 6),
                   down_k=(None if s.down_k is None else round(s.down_k, 6)),
                   dq=_dq_names(s.dq_bits), n_ticks=s.n_ticks,
                   posted=s.posted, hwy=s.hwy, drive=self._drive_id)

  def _date(self, t: float) -> str:
    return pt_date(t, self._tz)

  def _write(self, obs: Observation) -> bool:
    """Append one observation. Section 8.1's append-only JSONL: a torn last line is discarded on
    read, history is never rewritten.

    The new observation is deliberately NOT added to the live database. A pass must never be judged
    against itself (section 3.9 item 6), and "the database is as of the last boot" is a property
    that can be stated and checked rather than reasoned about.

    Returns whether it wrote, so the caller's verdict cannot claim an observation that never
    reached the disk."""
    if self._tz is None or self._write_stopped:
      return False
    try:
      if os.path.getsize(self._obs_path) > OBS_MAX_BYTES:
        self._write_stopped = True
        cloudlog.error(f"curvedb_shadow: {self._obs_path} reached the {OBS_MAX_BYTES} B cap -- " +
                       "the WRITE half has STOPPED. Frozen, not rotated: the >= 2-dates history " +
                       "is the whole point of the corpus. Pull it and truncate deliberately.")
        return False
    except OSError:
      pass                                  # no file yet / stat race -> just append
    try:
      with open(self._obs_path, "a") as f:
        f.write(json.dumps(vars(obs)) + "\n")
      self.obs_written += 1
      return True
    except Exception as e:
      self._fail("write", e)
      return False

  def _drop_why(self) -> str:
    """Why `_write` refused, in one word, so a dropped pass names its own cause."""
    if self._state == "noStore":
      return "noStore"
    return "stopped" if self._write_stopped else "err"

  # -- plumbing -------------------------------------------------------------------------------

  def _rows(self) -> int:
    db = self._db
    return 0 if db is None else len(db.rows)

  def _fail(self, where: str, e: Exception) -> None:
    """Rule 2: counted always, logged at a bounded rate. `cdbErr` carries the count into every
    record, so the data itself says the shadow is unhealthy even if nobody reads swaglog."""
    try:
      self.err += 1
      now = time.monotonic()
      if now - self._err_t >= 60.0:
        self._err_t = now
        cloudlog.exception(f"curvedb_shadow: {where} FAILED ({type(e).__name__}: {e}) -- " +
                           f"{self.err} failure(s) so far; cdbErr carries the count")
    except Exception:
      pass                                  # an error handler that can itself raise is worse


def curvedb_tele(ctl, *, site_pt, now_wall, v_ego, v_set) -> dict:
  """The `ces_events` fragment, built from a controller. The ONE builder for the ONE call site.

  Mirrors `_curve_tele` / `_curvelead_tele` / `coop_telemetry_fields`: every controller attribute is
  reached through `getattr` with a default, so a permissive test stub (or a controller built before
  this feature) degrades to an all-null fragment instead of raising mid-record. A missing `_cdb`
  yields `cdbOn: "absent"`, which is deliberately NOT the same string as any live state -- "the
  feature is not installed" and "the feature found no site" must never read alike.

  **`site_pt` IS ICBM'S OWN LATCHED CANDIDATE, NOT EVERY MAP CANDIDATE, AND THAT IS A SIZING
  DECISION WITH A MEASUREMENT BEHIND IT.** `upcoming_curve` returns the slowest point in the next
  ~300 m, which exists on most highway records and advances continuously -- so discovering sites
  from it would arm a new site every few records. Measured on a real 46-minute Phase-1 drive
  (`drives/2026-09-17/curvedb-first-capture/`): 77 distinct sites from any map candidate versus
  **6** from ICBM's own, a 13x difference. At the broad rate an 8-week corpus is ~11,000 rows, where
  a lookup costs ~2.2 ms and the boot build ~6.5 s (dev host); at the ICBM rate it is ~900 rows,
  ~0.3 ms and well under a second.

  It is also the right POPULATION, not merely the affordable one: a row can only ever cancel an ICBM
  map/far slowdown, so a site ICBM never reacts to needs no row, and section 12's recurrence question
  was itself measured over ICBM episodes. The cost is that a pass over a site on a day when ICBM did
  NOT react there contributes nothing -- flagged, not hidden.

  **This function's return value has exactly one permitted destination: a `**` splat into
  `_event_record`'s record dict.** Enforced by tests/test_curvedb_read_boundary.py."""
  cdb = getattr(ctl, "_cdb", None)
  if cdb is None:
    return {**_NULL_TELE, "cdbOn": "absent", "cdbRows": 0, "cdbObs": 0, "cdbErr": 0,
            "cdbSite": False}
  ceiling = getattr(ctl, "_icbm_ceiling", None)
  return cdb.record(
    now_wall=now_wall,
    lat=getattr(ctl, "_cur_lat", None), lon=getattr(ctl, "_cur_lon", None),
    bearing=getattr(ctl, "_cur_bearing", None), v_ego=v_ego, site_pt=site_pt,
    posted_ms=getattr(ctl, "_speed_limit", None), hwy_class=getattr(ctl, "_hwy_class", None),
    icbm_src=getattr(ctl, "_icbm_src", None),
    icbm_target_ms=getattr(ctl, "_icbm_last_target", None),
    # The reference the cancel formula caps against: the episode's latched ceiling while a cap is
    # running, else the driver's own set speed -- exactly the `ref` _icbm_step computes.
    ref_ms=(ceiling if ceiling is not None else v_set),
  )


def _f(x):
  """A finite float, or None. Never a substituted zero -- an exact 0.0 standing in for a missing
  reading is the dead-sensor class of defect P1-B exists to prevent."""
  try:
    v = float(x)
  except (TypeError, ValueError):
    return None
  return v if math.isfinite(v) else None


def _r6(x):
  return None if x is None else round(x, 6)


def _pt(site_pt):
  """A usable (lat, lon), or None. `map_candidate_point` returns (None, None) when there is no
  candidate, which is a tuple and therefore truthy -- the exact shape that turns "no candidate" into
  a confident lookup at (0, 0) if it is not unpacked and checked."""
  if not site_pt:
    return None
  try:
    la, lo = site_pt
  except (TypeError, ValueError):
    return None
  la, lo = _f(la), _f(lo)
  if la is None or lo is None:
    return None
  if not -90.0 <= la <= 90.0 or not -180.0 <= lo <= 180.0:
    return None
  return (la, lo)
