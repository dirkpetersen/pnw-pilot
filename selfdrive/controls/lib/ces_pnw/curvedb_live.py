"""curvedblive2pnw -- curvedb v2 LIVE: the learned road table supplies ICBM's map/far curve target.

WHAT IT DOES
============
The table (built offline from the drive logs by tools/curvedb/v2_build.py, exported by
v2_live_export.py as zstd-compressed JSON + a SHA-256 manifest) holds the MEASURED curvature of the roads the owner drives, per position, approach
direction and branch. On every ICBM decision where the map (mapd) names the curve, the controller asks
this module for the row of that curve. Where there is one, the curve's speed becomes

    v_db = sqrt(A / k_row)          A = mapd's own lateral-accel target, read live from MapdSettings

and replaces mapd's number for that curve (it does not stack on it):

* RAISE (v_db above ICBM's target -- the map claimed a sharper curve than the road has): exactly v_db, the same
  speed a lowering uses, capped at +15 mph over ICBM's own target and at posted + 10 mph. Never above the
  driver's set. (terwilliger2pnw, owner 2026-09-24: the former 1.25x curvature margin on raises turned A = 2.5
  into an effective 2.0 -- mapd's own A -- so on the Terwilliger Curves the DB could raise mapd's 47 / 49 mph by
  only 1-2 mph when the road needed 54-56. LODO held-out passes at v_db, A = 2.5, no margin: 2.88 % reach
  >= 3.0 m/s^2 and 0.46 % (4 of 867, all city streets) >= 3.5. The truck's lataccel2pnw cap is the backstop.)
* LOWER (v_db below -- a real sharp curve mapd under-rated or missed): exactly v_db, no margin, no
  penalty on top. "Sharp curves must slow, but only to the level required, never too much" (owner).
* A curve the map missed entirely is found by scanning mapd's path ahead for rows (an ADD). It binds
  through the same envelope a far-map candidate uses, and it is labeled `far` downstream.
* A vision candidate is never changed; the lowest candidate still wins (DEC always wins).

OWNER DECISIONS 2026-09-24 (override docs/CURVEDB-V2-BUILD.md s5/s6): LIVE, not shadow -- the design's
N >= 60 episode gate is overridden by the owner ("keep testing and see the results as soon as we can");
both directions; mapd's rating is not a hard ceiling.

NO EFFECT AT ALL -- ICBM's target exactly as without this module -- when:
  the car lacks the capability, or curve.json sets curvedb_v2_live = 0 (the kill switch);
  the file is missing, corrupt, of another format, or still loading;
  A cannot be read (MapdSettings / LongitudinalPersonality absent, unparsable or implausible);
  mapd's way selection is not `current` (and was not `current` at any decision in the last WAYSEL_HOLD_S = 2 s:
  a shorter flicker is ridden through, cdb2WayHold), the road class is unknown or a ramp, no posted limit;
  no anchor within 40 m and 35 degrees of the curve, or the BRANCH is unknown or ambiguous (mapd's path
  does not reach 150 m past the anchor, or its end point lies within 15 m of no branch, of two, or of a
  branch the table refused);
  a new episode would start while already loaded in a curve.
Every one of these is a `cdb2Why` value on the record, and a load failure or an unreadable A is also a
cloudlog.error (change-only). Nothing fails open into partial data: the index is built completely in
locals and published by one assignment, or not at all.

THIS MODULE IMPORTS NOTHING FROM tools/curvedb. The offline keying (roadtable.nearest / branch_point /
row_verdict) is re-implemented here in ~60 lines against the exported file, and
tests/test_curvedblive2pnw.py pins the two against each other on the same geometry. That keeps the
curvedbshadow2pnw read boundary (one control-path importer of tools/curvedb: the shadow) intact.
"""
from __future__ import annotations

import bisect
import hashlib
import io
import json
import math
import os
import threading
import time

from openpilot.common.swaglog import cloudlog

MPH = 0.44704

FORMAT = "curvedb-v2-live/1"
DATA_DIR = "/data/pnw/curvedb_v2"
ROWS_NAME = "curvedb_v2_rows.json.zst"   # zstd-compressed JSON (owner 2026-09-24: like the device's other logs)
MANIFEST_NAME = "manifest.json"
MAX_MANIFEST_BYTES = 64 * 1024
MAX_ROWS_BYTES = 16 * 1024 * 1024
MAX_JSON_BYTES = 64 * 1024 * 1024   # decompressed; the 2026-09-24 table is ~2.9 MB
# The keying the table was built with (roadtable.PROVISIONAL_V2). A file built with anything else is
# refused rather than matched with these constants.
EXPECTED_PARAMS = {"site_radius_m": 40.0, "heading_tol_deg": 35.0, "extent_back_m": 25.0,
                   "extent_fwd_m": 150.0, "branch_radius_m": 15.0}

# terwilliger2pnw: NO raise margin (was 1.25, docs/CURVEDB-V2-BUILD.md s4.4). A raise and a lowering both use
# v_db = sqrt(A / k); the caps below are what bounds a raise.
RAISE_CAP_MS = 15.0 * MPH       # a raise never exceeds ICBM's own target + 15 mph (P1 first cut)
POSTED_MARGIN_MS = 10.0 * MPH   # ... nor posted + 10 mph (the build replay's bound; the table reproduces with it)
ACT_EPS_MS = 0.05               # smaller than this is not a change (telemetry direction only)

# A: mapd's map_curve_target_lat_a for the active personality. Outside these bounds the value is not
# believed (mapd ships 1.9 / 2.2 / 2.4); DB off for that decision.
A_MIN, A_MAX = 1.0, 3.5
A_POLL_S = 5.0
A_MAX_AGE_S = 30.0              # a reader that stopped updating is not a reading
PERSONALITIES = {0: "aggressive", 1: "standard", 2: "relaxed"}   # cereal LongitudinalPersonality

DECISION_FRESH_S = 1.0         # a record shows the latest decision only if ICBM made one this recently
# terwilliger2pnw: mapd's way selection flickers. On the 2026-09-24 Terwilliger right-hander ONE ~1 s tick of
# waySel != "current" switched the DB off mid-curve; the target fell to mapd's 47.2 and, because a running cap never
# raises the set, the truck stayed at 47 after the DB's 49.6 came back. A waySel that was "current" at any decision
# within this many seconds still counts as current (cdb2WayHold = true). A sustained non-current waySel -- a real exit
# onto another road -- gates the DB off once this has elapsed since the last "current".
WAYSEL_HOLD_S = 2.0
MAX_RAISE_ROUNDS = 3           # hidden curves re-derived behind a raised map/far candidate (decide)
SCAN_STEP_M = 25.0              # the table's own resampling step (roadtable.PROVISIONAL_V2.step_m)
R_EARTH_M = 6371000.0
RAMP_CLASSES = ("motorwayLink", "trunkLink", "primaryLink", "secondaryLink", "tertiaryLink")
UNKNOWN_CLASSES = ("", "unknown", "None", "none")

# Every key tele() emits. Pinned by the tests: a key added here and not emitted (or vice versa) is a
# silently-null column -- that happened four times in ces_pnw.py's history.
TELE_KEYS = ("cdb2On", "cdb2Err", "cdb2Rows", "cdb2A", "cdb2ASrc", "cdb2N", "cdb2NR", "cdb2NL",
             "cdb2Why", "cdb2Dir", "cdb2Src", "cdb2Base", "cdb2Tgt", "cdb2Row", "cdb2K", "cdb2VDb",
             "cdb2D", "cdb2Lat", "cdb2Lon", "cdb2WayHold")


class CurveDbFileError(Exception):
  """The rows file cannot be trusted. The DB is then OFF -- and says so."""


# ---------------------------------------------------------------------------------------------
# geometry -- identical formulas to tools/curvedb/store.py (pinned by a test)
# ---------------------------------------------------------------------------------------------

def haversine_m(lat1, lon1, lat2, lon2) -> float:
  p1, p2 = math.radians(lat1), math.radians(lat2)
  dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
  a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
  return 2.0 * R_EARTH_M * math.asin(min(1.0, math.sqrt(a)))


def initial_bearing_deg(lat1, lon1, lat2, lon2) -> float:
  p1, p2 = math.radians(lat1), math.radians(lat2)
  dl = math.radians(lon2 - lon1)
  y = math.sin(dl) * math.cos(p2)
  x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
  return math.degrees(math.atan2(y, x)) % 360.0


def bearing_diff_deg(a, b) -> float:
  return abs((a - b + 180.0) % 360.0 - 180.0)


# ---------------------------------------------------------------------------------------------
# the table
# ---------------------------------------------------------------------------------------------

class RowIndex:
  """Immutable spatial index over the exported anchors. Same cell scheme and nearest-anchor rule as
  roadtable.AnchorIndex, so the car picks the anchor the offline replay picked."""

  def __init__(self, anchors: list, params: dict):
    self.radius = float(params["site_radius_m"])
    self.tol = float(params["heading_tol_deg"])
    self.back = float(params["extent_back_m"])
    self.fwd = float(params["extent_fwd_m"])
    self.branch_r = float(params["branch_radius_m"])
    self._cell = self.radius / 111000.0
    self.anchors: list[tuple] = []
    self._grid: dict[tuple[int, int], list[int]] = {}
    self.n_rows = 0
    for n, a in enumerate(anchors):
      if not (isinstance(a, list) and len(a) == 4 and isinstance(a[3], list)):
        raise CurveDbFileError(f"anchor {n}: malformed")
      lat, lon, brg = (float(x) for x in a[:3])
      if not (-90 <= lat <= 90 and -180 <= lon <= 180 and 0 <= brg <= 360):
        raise CurveDbFileError(f"anchor {n}: position/bearing out of range")
      brs = []
      for b in a[3]:
        if not (isinstance(b, list) and len(b) == 4):
          raise CurveDbFileError(f"anchor {n}: malformed branch")
        e_lat, e_lon = float(b[0]), float(b[1])
        k = None if b[2] is None else float(b[2])
        if not (math.isfinite(e_lat) and math.isfinite(e_lon)):
          raise CurveDbFileError(f"anchor {n}: non-finite branch end")
        if k is not None:
          if not (math.isfinite(k) and 0.0 < k < 1.0):
            raise CurveDbFileError(f"anchor {n}: curvature {k} out of range")
          if int(b[3]) < 2:     # the owner's >= 2 dates rule, re-checked on the car
            raise CurveDbFileError(f"anchor {n}: a row with authority on {b[3]} date(s)")
          self.n_rows += 1
        brs.append((e_lat, e_lon, k, int(b[3])))
      self.anchors.append((lat, lon, brg, tuple(brs)))
      self._grid.setdefault(self._key(lat, lon), []).append(n)

  def _key(self, lat, lon):
    return (int(math.floor(lat / self._cell)),
            int(math.floor(lon / (self._cell / max(math.cos(math.radians(lat)), 1e-6)))))

  def nearest(self, lat, lon, brg):
    """(anchor id, distance m) of the nearest anchor within radius and heading tolerance, else (None, None)."""
    if not math.isfinite(brg):
      return None, None
    ky, kx = self._key(lat, lon)
    best, bd = None, None
    for dy in (-1, 0, 1):
      for dx in (-1, 0, 1):
        for ai in self._grid.get((ky + dy, kx + dx), ()):
          a = self.anchors[ai]
          if bearing_diff_deg(a[2], brg) > self.tol:
            continue
          d = haversine_m(lat, lon, a[0], a[1])
          if d <= self.radius and (bd is None or d < bd):
            best, bd = ai, d
    return best, bd


def load_rows(data_dir: str) -> tuple[RowIndex, dict]:
  """Read, verify and index the rows file. Raises CurveDbFileError with the reason; never returns a
  partial index."""
  mpath, rpath = os.path.join(data_dir, MANIFEST_NAME), os.path.join(data_dir, ROWS_NAME)
  try:
    if os.path.getsize(mpath) > MAX_MANIFEST_BYTES:
      raise CurveDbFileError("manifest too large")
    with open(mpath) as f:
      man = json.load(f)
  except FileNotFoundError as e:
    raise CurveDbFileError(f"missing {mpath}") from e
  except (OSError, ValueError) as e:
    raise CurveDbFileError(f"manifest unreadable: {type(e).__name__}: {e}") from e
  if not isinstance(man, dict) or man.get("format") != FORMAT or man.get("file") != ROWS_NAME:
    got = (man.get("format"), man.get("file")) if isinstance(man, dict) else type(man).__name__
    raise CurveDbFileError(f"manifest (format, file) {got!r} != {(FORMAT, ROWS_NAME)!r}")
  if man.get("exclude_date") is not None:
    raise CurveDbFileError(f"a leave-one-date-out TEST build (exclude_date {man['exclude_date']}) is not a car file")
  try:
    if os.path.getsize(rpath) > MAX_ROWS_BYTES:
      raise CurveDbFileError("rows file too large")
    with open(rpath, "rb") as f:
      blob = f.read()
  except FileNotFoundError as e:
    raise CurveDbFileError(f"missing {rpath}") from e
  except OSError as e:
    raise CurveDbFileError(f"rows unreadable: {type(e).__name__}: {e}") from e
  if len(blob) != man.get("bytes"):
    raise CurveDbFileError(f"size {len(blob)} != manifest {man.get('bytes')}")
  sha = hashlib.sha256(blob).hexdigest()
  if sha != man.get("sha256"):
    raise CurveDbFileError(f"sha256 {sha[:12]} != manifest {str(man.get('sha256'))[:12]}")
  # Imported HERE, not at module level: a missing package must mean "DB OFF, loudly", never a ces_pnw import error.
  # On the device it comes from the same /usr/local/venv (+ /data/pnw/agnos19-compat overlay) that runs the uploader
  # and manager.py -- both import it at module level through openpilot.common.utils. No fallback format: a file this
  # process cannot decompress is a file it does not load.
  try:
    import zstandard
  except ImportError as e:
    raise CurveDbFileError(f"zstandard is not importable in this process ({e}) -- cannot read {ROWS_NAME}") from e
  try:
    raw = zstandard.ZstdDecompressor().stream_reader(io.BytesIO(blob)).read(MAX_JSON_BYTES + 1)
    if len(raw) > MAX_JSON_BYTES:
      raise CurveDbFileError("rows decompress beyond the size bound")
    doc = json.loads(raw)
  except (zstandard.ZstdError, ValueError) as e:
    raise CurveDbFileError(f"rows undecodable: {type(e).__name__}") from e
  if not isinstance(doc, dict) or doc.get("format") != FORMAT or doc.get("exclude_date") is not None:
    raise CurveDbFileError("rows document format / exclude_date mismatch")
  params = doc.get("params")
  if not isinstance(params, dict) or any(float(params.get(k, float("nan"))) != v for k, v in EXPECTED_PARAMS.items()):
    raise CurveDbFileError(f"keying params {params} != {EXPECTED_PARAMS}")
  anchors = doc.get("anchors")
  if not isinstance(anchors, list) or not anchors:
    raise CurveDbFileError("no anchors")
  try:
    idx = RowIndex(anchors, params)
  except (TypeError, ValueError) as e:
    raise CurveDbFileError(f"rows malformed: {type(e).__name__}: {e}") from e
  if idx.n_rows != man.get("rows_with_authority") or idx.n_rows == 0:
    raise CurveDbFileError(f"{idx.n_rows} rows with authority != manifest {man.get('rows_with_authority')}")
  return idx, man


# ---------------------------------------------------------------------------------------------
# mapd's path as a polyline with an odometer
# ---------------------------------------------------------------------------------------------

class Polyline:
  """mapd's MapTargetVelocities path (ordered along the predicted route, straight nodes included) with
  cumulative along-path distance. Local flat-earth x/y in meters around the first point (the path is
  <= ~1 km, so the error is far below the 15 m branch test)."""

  def __init__(self, points):
    pts, self.src_idx = [], []
    for i, p in enumerate(points or ()):
      try:
        la, lo = float(p["latitude"]), float(p["longitude"])
      except (KeyError, TypeError, ValueError):
        continue
      if math.isfinite(la) and math.isfinite(lo) and not (la == 0.0 and lo == 0.0):
        pts.append((la, lo))
        self.src_idx.append(i)
    self.ok = len(pts) >= 2
    if not self.ok:
      return
    self.lat0, self.lon0 = pts[0]
    self._kx = math.radians(1.0) * R_EARTH_M * math.cos(math.radians(self.lat0))
    self._ky = math.radians(1.0) * R_EARTH_M
    self.xy = [((lo - self.lon0) * self._kx, (la - self.lat0) * self._ky) for la, lo in pts]
    self.s = [0.0]
    for i in range(1, len(self.xy)):
      self.s.append(self.s[-1] + math.hypot(self.xy[i][0] - self.xy[i - 1][0], self.xy[i][1] - self.xy[i - 1][1]))
    self.s_max = self.s[-1]

  def _to_ll(self, x, y):
    return self.lat0 + y / self._ky, self.lon0 + x / self._kx

  def project(self, lat, lon):
    """(s, distance off the path) of the nearest point on the path to (lat, lon)."""
    px, py = (lon - self.lon0) * self._kx, (lat - self.lat0) * self._ky
    best = (float("inf"), 0.0)
    for i in range(len(self.xy) - 1):
      (ax, ay), (bx, by) = self.xy[i], self.xy[i + 1]
      dx, dy = bx - ax, by - ay
      L2 = dx * dx + dy * dy
      f = 0.0 if L2 <= 0.0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
      d = math.hypot(px - (ax + f * dx), py - (ay + f * dy))
      if d < best[0]:
        best = (d, self.s[i] + f * math.sqrt(L2))
    return best[1], best[0]

  def at(self, s):
    """(lat, lon) at odometer s, clamped to the path."""
    s = max(0.0, min(s, self.s_max))
    j = max(1, min(bisect.bisect_left(self.s, s), len(self.s) - 1))
    s0, s1 = self.s[j - 1], self.s[j]
    f = 0.0 if s1 <= s0 else (s - s0) / (s1 - s0)
    (ax, ay), (bx, by) = self.xy[j - 1], self.xy[j]
    return self._to_ll(ax + f * (bx - ax), ay + f * (by - ay))

  def heading(self, s, half_m=SCAN_STEP_M / 2.0):
    """Chord bearing over [s - half, s + half] -- the offline pass bearing is the chord between the
    neighboring 25 m points, so this is the same estimator."""
    a, b = self.at(s - half_m), self.at(s + half_m)
    if haversine_m(a[0], a[1], b[0], b[1]) <= 1.0:
      return float("nan")
    return initial_bearing_deg(a[0], a[1], b[0], b[1])


class Match:
  __slots__ = ("why", "anchor", "branch", "k", "s_anchor", "lat", "lon", "s_q")

  def __init__(self, why, anchor=None, branch=None, k=None, s_anchor=None, lat=None, lon=None, s_q=None):
    self.why, self.anchor, self.branch, self.k, self.s_anchor, self.lat, self.lon = why, anchor, branch, k, s_anchor, lat, lon
    self.s_q = s_q            # path odometer of the query point

  @property
  def row_id(self):
    return None if self.anchor is None or self.branch is None else f"{self.anchor}:{self.branch}"


def match_at(idx: RowIndex, poly: Polyline, s_q: float, q_lat: float, q_lon: float) -> Match:
  """The row for the curve at path odometer s_q (query point q). The offline keying:

  * anchor = nearest within 40 m whose approach bearing is within 35 deg of the PATH's bearing at q;
  * branch = where the PATH is 150 m past the anchor (the anchor projected onto the path), which must
    lie within 15 m of exactly one of the anchor's branch end points, and that branch must have
    authority (roadtable.branch_point + row_verdict's branch filter).
  Anything else is a no-effect Match with the reason."""
  hdg = poly.heading(s_q)
  ai, d = idx.nearest(q_lat, q_lon, hdg)
  if ai is None:
    return Match("noAnchor")
  a = idx.anchors[ai]
  along = 0.0
  if d > 0.5:
    along = d * math.cos(math.radians(initial_bearing_deg(q_lat, q_lon, a[0], a[1]) - hdg))
  s_a = s_q + along
  s_end = s_a + idx.fwd
  if s_end > poly.s_max or s_end <= 0.0:
    return Match("branchUnknown", anchor=ai, s_anchor=s_a, lat=a[0], lon=a[1])   # mapd's path is too short
  e_lat, e_lon = poly.at(s_end)
  hits = [n for n, b in enumerate(a[3]) if haversine_m(e_lat, e_lon, b[0], b[1]) <= idx.branch_r]
  if not hits:
    return Match("branchUnknown", anchor=ai, s_anchor=s_a, lat=a[0], lon=a[1])
  if len(hits) > 1:
    return Match("branchAmbiguous", anchor=ai, s_anchor=s_a, lat=a[0], lon=a[1])
  b = a[3][hits[0]]
  if b[2] is None:
    return Match("noAuthority", anchor=ai, branch=hits[0], s_anchor=s_a, lat=a[0], lon=a[1])
  return Match("ok", anchor=ai, branch=hits[0], k=b[2], s_anchor=s_a, lat=a[0], lon=a[1])


def scan_ahead(idx: RowIndex, poly: Polyline, s_ego: float, horizon_m: float, step_m: float = SCAN_STEP_M):
  """Every granted row mapd's path passes within [s_ego, s_ego + horizon], one Match per row (the
  first sample that found it). The ADD half: curves the map did not rate, or under-rated."""
  out, seen = [], set()
  s = s_ego
  end = min(s_ego + horizon_m, poly.s_max)
  while s <= end:
    la, lo = poly.at(s)
    m = match_at(idx, poly, s, la, lo)
    if m.why == "ok" and m.row_id not in seen:
      seen.add(m.row_id)
      out.append(m)
    s += step_m
  return out


# ---------------------------------------------------------------------------------------------
# the target rule
# ---------------------------------------------------------------------------------------------

def v_db(a_lat: float, k: float) -> float:
  return math.sqrt(a_lat / k)


def db_target(today: float, k: float, a_lat: float, ref: float, posted) -> tuple[float, str]:
  """The DB's replacement for ONE map/far candidate whose pipeline value is `today` (m/s).
  Returns (value, direction). Pure; the rule of the module docstring:

    raise: exactly sqrt(A / k), capped at today + 15 mph and posted + 10 mph ("held" when a cap leaves it at today)
    lower: exactly sqrt(A / k)
  and never above `ref` (the driver's set / the episode ceiling)."""
  if not (k > 0.0 and a_lat > 0.0 and math.isfinite(today) and math.isfinite(ref)):
    raise ValueError(f"db_target: bad inputs today={today} k={k} A={a_lat} ref={ref}")
  v = v_db(a_lat, k)
  if v > today + ACT_EPS_MS:
    r = v
    cap = today + RAISE_CAP_MS
    if posted is not None and posted > 0.0:
      cap = min(cap, posted + POSTED_MARGIN_MS)
    r = min(max(today, min(r, cap)), ref)
    return r, ("raise" if r > today + ACT_EPS_MS else "held")
  if v < today - ACT_EPS_MS:
    return v, "lower"
  return today, "none"


# ---------------------------------------------------------------------------------------------
# the live object: loader thread, A reader, decision, telemetry
# ---------------------------------------------------------------------------------------------

def _default_read_params():
  from openpilot.common.params import Params
  p = Params()
  return p.get("MapdSettings"), p.get("LongitudinalPersonality", return_default=True)


# The reader CurveDbLive uses when none is passed. A one-slot list so a test harness can redirect every
# controller it builds (tests/roaddb_fixture.py) without the production path growing a test switch.
READ_PARAMS = [_default_read_params]
BACKGROUND = [True]           # False: load + read A synchronously in the constructor (test harness only)


def parse_a(settings_raw, personality_raw) -> tuple[float | None, str | None, str]:
  """(A, source, why) from mapd's own settings store. A is None whenever it cannot be read with certainty --
  never a default: every DB speed scales with sqrt(A).

  TWO LAYOUTS, both real:
  * pfeiferj mapd v2 (settings_version 2): personalities[<p>].map_curve_target_lat_a, <p> from
    LongitudinalPersonality (0 aggressive, 1 standard, 2 relaxed) -- source "mapd:<p>";
  * the truck's custom mapd 77bad867 (settings_version 1, DEVICE-VERIFIED 2026-09-24): a TOP-LEVEL
    map_curve_target_lat_a (2, an int) and no `personalities` key -- source "mapd:top". The personality is then
    not part of the lookup.
  Params returns the JSON param already parsed (a dict); bytes/str are parsed here."""
  s = settings_raw
  if s is None:
    return None, None, "MapdSettings absent"
  try:
    if isinstance(s, bytes):
      s = s.decode()
    if isinstance(s, str):
      s = json.loads(s)
  except (UnicodeDecodeError, ValueError) as e:
    return None, None, f"MapdSettings unparsable ({type(e).__name__})"
  if not isinstance(s, dict):
    return None, None, f"MapdSettings is a {type(s).__name__}, not an object"
  pers = s.get("personalities")
  try:
    if isinstance(pers, dict):
      try:
        name = PERSONALITIES.get(int(personality_raw.decode() if isinstance(personality_raw, bytes) else personality_raw))
      except (TypeError, ValueError, AttributeError):
        return None, None, f"personality unreadable ({personality_raw!r:.40})"
      if name is None:
        return None, None, f"personality {personality_raw!r:.20} unknown"
      if name not in pers:
        return None, f"mapd:{name}", f"MapdSettings has no personality {name}"
      a, src = float(pers[name]["map_curve_target_lat_a"]), f"mapd:{name}"
    elif "map_curve_target_lat_a" in s:
      a, src = float(s["map_curve_target_lat_a"]), "mapd:top"
    else:
      return None, None, "MapdSettings has neither personalities nor a top-level map_curve_target_lat_a"
  except (KeyError, TypeError, ValueError) as e:
    return None, None, f"MapdSettings unparsable ({type(e).__name__})"
  if not (math.isfinite(a) and A_MIN <= a <= A_MAX):
    return None, src, f"A {a} outside [{A_MIN}, {A_MAX}]"
  return a, src, "ok"


class CurveDbLive:
  def __init__(self, enabled: bool, data_dir: str | None = None, read_params=None, start: bool = True,
               a_override: float | None = None):
    """a_override: curve.json's curvedb_v2_lat_a (PnwVehicle.curvedb_v2_lat_a) -- a number means the DB uses it
    instead of mapd's A (source "curve.json"); None means mapd's A, read live."""
    self.enabled = bool(enabled)
    self.a_override = a_override
    self.data_dir = data_dir if data_dir is not None else DATA_DIR       # looked up at call time (tests redirect it)
    self._read_params = read_params or READ_PARAMS[0]
    self.state = "off" if not self.enabled else "loading"
    self.err = None
    self.index: RowIndex | None = None
    self.manifest: dict | None = None
    self._a = (None, None, "not read yet", -1e9)     # (A, personality, why, monotonic t)
    self._a_logged = None
    self._loaded = threading.Event()
    self._poly_src, self._poly = None, None
    self.n = self.n_raise = self.n_lower = 0
    self._last: dict = {}
    self._last_t = -1e9
    self._way_cur_t = -1e9           # monotonic time of the last decision that saw waySel == "current"
    self._way_holding = False        # change-only log of a WAYSEL_HOLD_S ride-through
    if not self.enabled:
      self._loaded.set()
      return
    if not start:
      return
    if BACKGROUND[0]:
      threading.Thread(target=self._run, name="curvedb_live", daemon=True).start()
    else:                     # test harness only (roaddb_fixture.py): deterministic, no thread
      self.load()
      if self.state == "ok":
        self.poll_a()

  # -- background: load once, then poll A ---------------------------------------------------------
  def load(self) -> None:
    t0 = time.monotonic()
    try:
      idx, man = load_rows(self.data_dir)
    except CurveDbFileError as e:
      self.err, self.state = str(e)[:160], "err"
      cloudlog.error(f"curvedb_v2: LOAD FAILED -- {e}. The curve DB is OFF: ICBM runs exactly as without it.")
    except Exception as e:   # a defect in the loader is a load failure too -- loudly, never partial data
      self.err, self.state = f"{type(e).__name__}: {e}"[:160], "err"
      cloudlog.exception("curvedb_v2: LOAD CRASHED -- the curve DB is OFF")
    else:
      self.index, self.manifest = idx, man   # one assignment each, after the index is complete
      self.state = "ok"
      cloudlog.event("curvedb_v2_loaded", rows=idx.n_rows, anchors=len(idx.anchors), sha256=man.get("sha256"),
                     first_date=man.get("first_date"), last_date=man.get("last_date"),
                     seconds=round(time.monotonic() - t0, 2))
    finally:
      self._loaded.set()

  def poll_a(self) -> None:
    if self.a_override is not None:
      a, name, why = self.a_override, "curve.json", "ok"
      self._a = (a, name, why, time.monotonic())
      if self._a_logged != (a, name, why):
        cloudlog.event("curvedb_v2_a", a_lat=a, source=name)
        self._a_logged = (a, name, why)
      return
    try:
      raw_s, raw_p = self._read_params()
      a, name, why = parse_a(raw_s, raw_p)
    except Exception as e:
      a, name, why = None, None, f"read failed ({type(e).__name__})"
    self._a = (a, name, why, time.monotonic())
    key = (a, name, why)
    if key != self._a_logged:              # change-only, both directions
      if a is None:
        cloudlog.error(f"curvedb_v2: mapd's lateral target A is UNREADABLE ({why}) -- the curve DB is OFF for every decision until it reads")
      else:
        cloudlog.event("curvedb_v2_a", a_lat=a, source=name)
      self._a_logged = key

  def _run(self) -> None:
    self.load()
    if self.state != "ok":
      return
    while True:
      try:
        self.poll_a()
      except Exception:
        cloudlog.exception("curvedb_v2: A poll crashed")
      time.sleep(A_POLL_S)

  def wait_loaded(self, timeout: float = 30.0) -> bool:
    return self._loaded.wait(timeout)

  def a_lat(self):
    """(A, source, why) -- A None when unreadable or stale. source: "curve.json" | "mapd:top" | "mapd:<personality>"."""
    a, name, why, t = self._a
    if a is not None and time.monotonic() - t > A_MAX_AGE_S:
      return None, name, f"A reading stale ({time.monotonic() - t:.0f} s)"
    return a, name, why

  def polyline(self, points) -> Polyline:
    if points is not self._poly_src:          # mapd's list is replaced ~1 Hz; rebuild only then
      self._poly_src, self._poly = points, Polyline(points)
    return self._poly

  # -- the decision -----------------------------------------------------------------------------
  def gate(self, *, way_sel, hwy, posted, plat, plon, allow) -> str | None:
    """Why the DB may not act at all this decision, or None."""
    if not self.enabled:
      return "off"
    if self.state != "ok":
      return "db" + self.state.capitalize()
    if self.a_lat()[0] is None:
      return "noA"
    if not allow:
      return "inCurveStart"
    if plat is None or plon is None:
      return "noGps"
    if way_sel != "current":
      return "waySel"
    if hwy in UNKNOWN_CLASSES or hwy in RAMP_CLASSES or hwy is None:
      return "class"
    if not (posted and posted > 0.0):   # both directions for now: no posted limit = no raise cap (owner call pending)
      return "noPosted"
    return None

  def decide(self, **kw):
    """_decide(), but a crash never leaves the PREVIOUS decision on the telemetry as if it were live: it is
    recorded as cdb2Why="crash" and re-raised (ICBM's caller logs it and runs without the DB)."""
    try:
      return self._decide(**kw)
    except Exception:
      self._last = {"cdb2Src": kw.get("src"), "cdb2Base": _r(kw.get("today"), 2), "cdb2Tgt": _r(kw.get("today"), 2),
                    "cdb2Dir": "none", "cdb2Why": "crash"}
      self._last_t = time.monotonic()
      raise

  def way_sel_held(self, way_sel, now) -> tuple[str | None, bool]:
    """(the waySel the gate uses, whether a flicker is being ridden through). Called on EVERY decision, before any
    gate, so the last-current time is tracked even while another gate is closed."""
    if way_sel == "current":
      self._way_cur_t = now
      hold = False
    else:
      hold = now - self._way_cur_t <= WAYSEL_HOLD_S
    if hold != self._way_holding:     # change-only, both edges
      if hold:
        cloudlog.event("curvedb_v2_waysel_hold", way_sel=way_sel, since_current_s=round(now - self._way_cur_t, 2))
      else:
        cloudlog.event("curvedb_v2_waysel_hold_end", way_sel=way_sel,
                       since_current_s=None if self._way_cur_t < 0 else round(now - self._way_cur_t, 2))
      self._way_holding = hold
    return ("current" if hold else way_sel), hold

  def _decide(self, *, today, src, cands_fn, recand_fn, points, plat, plon, ref, posted, horizon_m, bind_fn,
              min_drop, way_sel, hwy, allow):
    """One ICBM decision. Returns (target, src, far_dist or None).

    today      ICBM's target without the DB (post-penalty), or None
    src        its source ("map"/"far"/"vis"/None)
    cands_fn   () -> (cands, cand_pts), called only once every gate has passed:
               cands    {"map": (value, dist), "far": ..., "vis": ...} -- every candidate's binding,
                        post-penalty value, or None
               cand_pts {"map": (lat, lon), "far": (lat, lon)} -- the mapd node each map/far one names
    recand_fn  (src, points) -> ((value, dist), (lat, lon)) or None: the map/far candidate re-derived on a
               subset of mapd's points (see HIDDEN CURVES below)
    bind_fn    (v, dist) -> the binding value of a far-map candidate at that speed/distance, or None
    Unchanged (today, src, None) whenever the DB has no effect.

    HIDDEN CURVES. ICBM keeps ONE map and ONE far candidate: the most binding mapd point. When the DB raises
    that point, a second mapd-rated curve behind it would otherwise never be considered until the first is
    passed. So a raised candidate's row stretch [anchor - 25 m, anchor + 150 m] is removed from mapd's points
    and the source's candidate re-derived, up to MAX_RAISE_ROUNDS times; every candidate found takes part.

    THE MINIMUM WINS, and a DB value above `today` is a RAISE wherever it comes from: the candidate's own row,
    or a row found by the scan ahead, carries the raise caps (db_target). Only a value below `today`
    -- or any value when ICBM has no target at all -- is used exactly."""
    self.n += 1
    self._last_t = time.monotonic()
    way_sel, way_hold = self.way_sel_held(way_sel, self._last_t)
    rec = {"cdb2Src": src, "cdb2Base": _r(today, 2), "cdb2Tgt": _r(today, 2), "cdb2Dir": "none",
           "cdb2Row": None, "cdb2K": None, "cdb2VDb": None, "cdb2D": None, "cdb2Lat": None, "cdb2Lon": None,
           "cdb2WayHold": way_hold}
    why = self.gate(way_sel=way_sel, hwy=hwy, posted=posted, plat=plat, plon=plon, allow=allow)
    if why is None:
      poly = self.polyline(points)
      if not poly.ok:
        why = "noPath"
    if why is not None:
      rec["cdb2Why"] = why
      self._last = rec
      return today, src, None
    a_lat = self.a_lat()[0]
    idx = self.index
    cands, cand_pts = cands_fn()
    s_ego, _off = poly.project(plat, plon)
    s_of = dict(zip(poly.src_idx, poly.s, strict=True))      # mapd point index -> odometer

    def replace(value, dist, pt):
      """(value, dist, match, direction) for one map/far candidate with the DB's number."""
      if pt is None or pt[0] is None:
        return value, dist, Match("noCandidatePoint"), "none"
      s_q, _ = poly.project(pt[0], pt[1])
      m = match_at(idx, poly, s_q, pt[0], pt[1])
      m.s_q = s_q
      if m.why != "ok":
        return value, dist, m, "none"
      v, d = db_target(value, m.k, a_lat, ref, posted)
      return (None if v >= ref - min_drop else v), dist, m, d

    base = {s: c for s, c in cands.items() if c is not None}   # the pipeline, no DB
    pool = []                                                  # (value, dist, src, match, direction)
    own = None                                                 # ICBM's own candidate, replaced
    for s in ("map", "far", "vis"):
      c = cands.get(s)
      if c is None:
        continue
      if s == "vis":
        pool.append((c[0], c[1], s, None, "none"))
        continue
      value, dist, m, d = replace(c[0], c[1], cand_pts.get(s))
      if s == src:
        own = (value, m, d)
      pool.append((value, dist, s, m, d))
      excluded = []
      for _ in range(MAX_RAISE_ROUNDS):
        if d != "raise":
          break
        # the row's stretch, and the mapd node that named it (up to 40 m before the anchor)
        excluded.append((min(m.s_anchor - idx.back, m.s_q) - 1.0, m.s_anchor + idx.fwd))
        keep = [p for i, p in enumerate(points)
                if not any(lo <= s_of.get(i, -1e9) <= hi for lo, hi in excluded)]
        nxt = recand_fn(s, keep)
        if nxt is None:
          break
        (v2, d2), pt2 = nxt
        value, dist, m, d = replace(v2, d2, pt2)
        pool.append((value, dist, s, m, d))
    for m in scan_ahead(idx, poly, s_ego, horizon_m):
      dist = max(m.s_anchor - idx.back - s_ego, 0.0)
      v = v_db(a_lat, m.k) if today is None else db_target(today, m.k, a_lat, ref, posted)[0]
      b = bind_fn(v, dist)
      if b is not None:
        pool.append((b, dist, "add", m, "add"))
    live = [x for x in pool if x[0] is not None]
    new = min(live, key=lambda x: x[0], default=None)
    base_min = min((v for v, _d in base.values()), default=None)
    changed = (new is None) != (base_min is None) or (new is not None and abs(new[0] - base_min) > 1e-9)
    rep_m = own[1] if own is not None else None
    if not changed:
      out = (today, src, None)
      if rep_m is None:
        adds = [x for x in live if x[2] == "add"]
        rep_m = min(adds, key=lambda x: x[0])[3] if adds else None
      rec["cdb2Why"] = ("held" if own is not None and own[2] == "held" else   # a raise the caps / the set withheld
                        ("notMin" if rep_m.why == "ok" else rep_m.why) if rep_m is not None else
                        ("noRow" if src in ("map", "far") else "notMap"))   # notMin: a row matched, another cand binds
    else:
      if new is None:
        out = (None, None, None)
      elif new[2] == "add":
        out = (new[0], "far", new[1])
      else:
        out = (new[0], new[2], new[1] if new[2] == "far" else None)
      if new is not None and new[3] is not None:
        rep_m = new[3]
      rec["cdb2Why"] = "ok"
      if out[0] is None or (today is not None and out[0] > today + ACT_EPS_MS):
        rec["cdb2Dir"] = "raise"
        self.n_raise += 1
      elif today is None or out[0] < today - ACT_EPS_MS:
        rec["cdb2Dir"] = "add" if new is not None and new[2] == "add" else "lower"
        self.n_lower += 1
    if rep_m is not None and rep_m.anchor is not None:
      rec.update(cdb2Row=rep_m.row_id, cdb2Lat=round(rep_m.lat, 5), cdb2Lon=round(rep_m.lon, 5),
                 cdb2D=_r(max(rep_m.s_anchor - s_ego, 0.0), 0))
      if rep_m.k is not None:
        rec.update(cdb2K=round(rep_m.k, 6), cdb2VDb=_r(v_db(a_lat, rep_m.k), 2))
    rec["cdb2Tgt"] = _r(out[0], 2)
    self._last = rec
    return out

  def tele(self) -> dict:
    """The ces_events fragment: liveness always, the latest decision's fields when there was one."""
    a, name, why = self.a_lat() if self.enabled else (None, None, "off")
    out = dict.fromkeys(TELE_KEYS)
    out.update(cdb2On=self.state, cdb2Err=self.err, cdb2Rows=self.index.n_rows if self.index is not None else 0,
               cdb2A=a, cdb2ASrc=name, cdb2N=self.n, cdb2NR=self.n_raise, cdb2NL=self.n_lower)
    if self._last and time.monotonic() - self._last_t <= DECISION_FRESH_S:
      out.update(self._last)
    else:                                  # ICBM idle (Chill, CES off, no data): no stale decision beside icbmT=None
      out["cdb2Why"] = "off" if not self.enabled else "idle"
    return out


def _r(x, nd):
  return None if x is None else round(float(x), nd)
