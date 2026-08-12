"""
steerpower2pnw — LOGGING ONLY. Measures the truck's true hands-off steering capability by direction:
achLat (delivered lateral accel this tick), peakAchLat (the 100 Hz episode peak on steerEvent), and
heading (8-pt compass from bearing).

Environment note: this worktree (a fresh `git worktree add`, submodules not initialized/built) cannot
import openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw or controlsd.py directly -- both transitively
pull in `from cereal import log`, and cereal's capnp schema compile aborts the interpreter in this
environment (pre-existing, reproduced identically on a clean checkout of origin/3devpnw before any of
this feature's edits -- not something this change introduced). The built `pnw-pilot` worktree's own
.venv (with params_pyx.so + generated capnp bindings) is exactly what `tools/op.sh setup` /
`scons -u -j$(nproc)` produce and what CI actually runs against.

So this test AST-extracts the exact PURE functions/statements this feature added straight out of the
real source files and exec's them in a small stub harness -- proving the literal shipped code (not a
reimplementation of it) is correct, without needing the full package import chain. This mirrors the
"host lacks capnp for full pytest; scenario-test pure modules instead" guidance in the openpilot skill.
"""
import ast
import json
import math
import textwrap
import time
from pathlib import Path

import numpy as np

from openpilot.common.constants import CV   # imports cleanly standalone (no cereal pull-in), verified below

REPO_ROOT = Path(__file__).resolve().parents[5]
CES_PNW_PATH = REPO_ROOT / "selfdrive/controls/lib/ces_pnw/ces_pnw.py"
CONTROLSD_PATH = REPO_ROOT / "selfdrive/controls/controlsd.py"
DRIVE_HELPERS_PATH = REPO_ROOT / "selfdrive/controls/lib/drive_helpers.py"

MIN_SPEED = 1.0   # openpilot.selfdrive.controls.lib.drive_helpers.MIN_SPEED (verified by inspection --
                   # this worktree can't import drive_helpers.py either, same cereal-import blocker)


def _parse(path: Path):
  src = path.read_text()
  return src, ast.parse(src, filename=str(path))


def _segment(src: str, node: ast.AST) -> str:
  """ast.get_source_segment() for a node NOT at column 0 (e.g. a statement inside a method) returns
  its first line stripped of leading whitespace but every OTHER line with its original absolute
  indentation intact -- not directly re-parseable/exec'able as a standalone snippet. Restore the
  first line's real indentation (from node.col_offset) before dedenting the whole block uniformly."""
  seg = ast.get_source_segment(src, node)
  assert seg is not None
  return textwrap.dedent((" " * node.col_offset) + seg)


def _extract_func(src: str, tree: ast.AST, name: str) -> str:
  """Return the exact source text of the top-level `def <name>(...)` FunctionDef."""
  matches = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name]
  assert len(matches) == 1, f"expected exactly one def {name}(), found {len(matches)}"
  return _segment(src, matches[0])


def _extract_stmt(src: str, tree: ast.AST, node_type, predicate, allow_duplicates: bool = False) -> str:
  """Return the exact source text of the statement node of `node_type` matching `predicate`.
  Normally requires exactly one match; with allow_duplicates=True (e.g. the reset-to-0.0 statement,
  which is textually identical whether matched at __init__ or the fold site), any number of matches
  is fine as long as their source text all agrees -- still proves it's the real statement, not an
  invented one."""
  matches = [n for n in ast.walk(tree) if isinstance(n, node_type) and predicate(n)]
  segs = [_segment(src, n) for n in matches]
  assert segs
  if allow_duplicates:
    assert len(set(segs)) == 1, f"matches disagree on text: {set(segs)}"
  else:
    assert len(matches) == 1, f"expected exactly one matching {node_type.__name__}, found {len(matches)}"
  return segs[0]


def _extract_module_assign(src: str, tree: ast.AST, name: str) -> str:
  """Return the exact source text of the single module-level `<name> = ...` assignment."""
  matches = [n for n in tree.body if isinstance(n, ast.Assign)
             and len(n.targets) == 1 and isinstance(n.targets[0], ast.Name) and n.targets[0].id == name]
  assert len(matches) == 1, f"expected exactly one module-level {name} = ..., found {len(matches)}"
  return _segment(src, matches[0])


def _is_attr_target(node: ast.Assign, attr: str) -> bool:
  return (len(node.targets) == 1 and isinstance(node.targets[0], ast.Attribute)
          and node.targets[0].attr == attr)


def _exec_func(src: str, extra_globals: dict | None = None):
  ns = {"math": math}
  if extra_globals:
    ns.update(extra_globals)
  exec(compile(src, "<extracted>", "exec"), ns)
  # the def statement is the only top-level statement -> its name is the only new key besides math/extras
  new_names = [k for k in ns if k not in ("math", "__builtins__", *(extra_globals or {}))]
  assert len(new_names) == 1
  return ns[new_names[0]]


# ---------------------------------------------------------------------------------------------------
# (a) achLat = slKActl * vEgo^2, signed -- both copies (ces_pnw.py's _ach_lat, controlsd.py's
#     _ach_lat_ms2) are extracted and tested identically, since the two processes intentionally carry
#     independent copies of the same formula (see the docstrings added alongside each).
# ---------------------------------------------------------------------------------------------------
def _load_ces_pnw_ach_lat():
  src, tree = _parse(CES_PNW_PATH)
  return _exec_func(_extract_func(src, tree, "_ach_lat"))


def _load_ces_pnw_compass():
  src, tree = _parse(CES_PNW_PATH)
  # _compass() indexes the module-level _COMPASS_PTS tuple -- extract that too, same-file source,
  # not invented data, so the exec namespace has the real 8-point list the function actually uses.
  pts_src = _extract_module_assign(src, tree, "_COMPASS_PTS")
  ns = {"math": math}
  exec(compile(pts_src, "<pts>", "exec"), ns)
  return _exec_func(_extract_func(src, tree, "_compass"), extra_globals=ns)


def _load_controlsd_ach_lat():
  src, tree = _parse(CONTROLSD_PATH)
  return _exec_func(_extract_func(src, tree, "_ach_lat_ms2"))


def test_ach_lat_formula_signed_example():
  # driver-given example: slKActl=-0.011, v=20 -> achLat = -0.011 * 400 = -4.4
  for ach_lat in (_load_ces_pnw_ach_lat(), _load_controlsd_ach_lat()):
    result = ach_lat(-0.011, 20)
    assert math.isclose(result, -4.4, rel_tol=1e-9)


def test_ach_lat_sign_preserved_both_directions():
  ach_lat = _load_ces_pnw_ach_lat()
  # right turn (positive kActl, Ford wire convention positive=left per the module's own comments --
  # sign correctness here just means "matches the sign of k_actl", not a claim about which way is
  # physically left/right; that's the caller's convention, not this pure function's job).
  assert ach_lat(0.02, 15) > 0
  assert math.isclose(ach_lat(0.02, 15), 0.02 * 15 ** 2, rel_tol=1e-9)
  # left turn (negative kActl) -> negative achLat
  assert ach_lat(-0.02, 15) < 0


def test_ach_lat_none_and_nan_degrade_to_none_never_raise():
  for ach_lat in (_load_ces_pnw_ach_lat(), _load_controlsd_ach_lat()):
    assert ach_lat(None, 20) is None
    assert ach_lat(0.01, None) is None
    assert ach_lat(None, None) is None
    assert ach_lat(float("nan"), 20) is None
    assert ach_lat(float("inf"), 20) is None
    assert ach_lat(0.01, float("nan")) is None
    # malformed (non-numeric) input must also degrade, never raise
    assert ach_lat("garbage", 20) is None


# ---------------------------------------------------------------------------------------------------
# (c) _compass 8-point mapping + None/NaN -> null
# ---------------------------------------------------------------------------------------------------
def test_compass_8_point_mapping():
  compass = _load_ces_pnw_compass()
  cases = {
    0: "N", 45: "NE", 90: "E", 135: "SE", 180: "S", 225: "SW", 270: "W", 315: "NW", 359: "N",
  }
  for bearing, expected in cases.items():
    assert compass(bearing) == expected, f"bearing {bearing} -> {compass(bearing)}, expected {expected}"


def test_compass_none_and_nan_degrade_to_none():
  compass = _load_ces_pnw_compass()
  assert compass(None) is None
  assert compass(float("nan")) is None
  assert compass(float("inf")) is None
  assert compass("garbage") is None


# ---------------------------------------------------------------------------------------------------
# (b) peakAchLat: the SAME 100 Hz accumulator/fold/latch/reset pattern peakAngErr/angErrPk uses in
#     controlsd.py -- extract the ACTUAL statements (not a reimplementation) and drive them through a
#     synthetic episode, proving the true between-sample peak survives (a spike that a ~5 Hz-only
#     sample would alias away) and that a new episode does not inherit the previous episode's peak.
# ---------------------------------------------------------------------------------------------------
class _FakeCS:
  def __init__(self, yaw_rate, v_ego):
    self.yawRate = yaw_rate
    self.vEgo = v_ego


class _Harness:
  """Stub for `self` -- only the two attributes this feature's statements touch."""
  def __init__(self):
    self._flight_peak_achlat_acc = 0.0
    self._flight_peak_achlat = 0.0


def _load_controlsd_pieces():
  src, tree = _parse(CONTROLSD_PATH)
  ach_lat_ms2_src = _extract_func(src, tree, "_ach_lat_ms2")
  # the 100 Hz accumulator: the one Try block that touches both k_actl_now and the acc attribute
  accum_src = _extract_stmt(
    src, tree, ast.Try,
    lambda n: "k_actl_now" in (ast.get_source_segment(src, n) or "")
    and "_flight_peak_achlat_acc" in (ast.get_source_segment(src, n) or ""))
  # fold: "ach_lat_pk = round(self._flight_peak_achlat_acc, 3)"
  fold_pk_src = _extract_stmt(
    src, tree, ast.Assign,
    lambda n: len(n.targets) == 1 and isinstance(n.targets[0], ast.Name)
    and n.targets[0].id == "ach_lat_pk")
  # reset: "self._flight_peak_achlat_acc = 0.0" (identical text whether matched at __init__ or the
  # fold site -- both zero the same accumulator the same way; either match execs identically).
  reset_acc_src = _extract_stmt(
    src, tree, ast.Assign,
    lambda n: _is_attr_target(n, "_flight_peak_achlat_acc")
    and isinstance(n.value, ast.Constant) and n.value.value == 0.0,
    allow_duplicates=True)
  # onset latch: "self._flight_peak_achlat = ach_lat_pk"
  onset_src = _extract_stmt(
    src, tree, ast.Assign,
    lambda n: _is_attr_target(n, "_flight_peak_achlat")
    and isinstance(n.value, ast.Name) and n.value.id == "ach_lat_pk")
  # armed-state running max: "self._flight_peak_achlat = max(self._flight_peak_achlat, ach_lat_pk)"
  armed_src = _extract_stmt(
    src, tree, ast.Assign,
    lambda n: _is_attr_target(n, "_flight_peak_achlat")
    and isinstance(n.value, ast.Call) and isinstance(n.value.func, ast.Name)
    and n.value.func.id == "max")
  ach_lat_ms2 = _exec_func(ach_lat_ms2_src)
  return accum_src, fold_pk_src, reset_acc_src, onset_src, armed_src, ach_lat_ms2


def test_peak_ach_lat_100hz_accumulator_catches_between_sample_spike_and_resets_per_episode():
  accum_src, fold_pk_src, reset_acc_src, onset_src, armed_src, ach_lat_ms2 = _load_controlsd_pieces()

  self_obj = _Harness()

  def accum_tick(yaw_rate, v_ego):
    """Run the EXACT extracted 100 Hz accumulator statement for one control tick."""
    ns = {"self": self_obj, "CS": _FakeCS(yaw_rate, v_ego), "MIN_SPEED": MIN_SPEED,
          "_ach_lat_ms2": ach_lat_ms2}
    exec(compile(accum_src, "<accum>", "exec"), ns)

  def fold():
    """Run the EXACT extracted fold (round + reset) statements; returns ach_lat_pk."""
    ns = {"self": self_obj}
    exec(compile(fold_pk_src + "\n" + reset_acc_src, "<fold>", "exec"), ns)
    return ns["ach_lat_pk"]

  def onset(ach_lat_pk):
    exec(compile(onset_src, "<onset>", "exec"), {"self": self_obj, "ach_lat_pk": ach_lat_pk})

  def armed_update(ach_lat_pk):
    exec(compile(armed_src, "<armed>", "exec"), {"self": self_obj, "ach_lat_pk": ach_lat_pk})

  # ---- Episode 1, throttled window #1 (~20 ticks @ 100 Hz = one ~190 ms sample period): a small
  # steady achLat of ~1.0 m/s^2 EXCEPT one between-sample tick (#10) that spikes to a much larger
  # value -- exactly the aliasing case the 100 Hz accumulator exists to catch (a ~5 Hz-only sample
  # would only ever see the tick AT the sample boundary, missing this spike entirely).
  v_ego = 20.0
  steady_k = 1.0 / v_ego ** 2         # achLat ~= 1.0 m/s^2 steady
  spike_k = -6.0 / v_ego ** 2         # achLat ~= -6.0 m/s^2 spike, between samples, opposite sign
  for i in range(20):
    k = spike_k if i == 10 else steady_k
    accum_tick(k * v_ego, v_ego)      # yawRate = k_actl * v_ego (undoes the /v_ego in the extracted stmt)
  ach_lat_pk1 = fold()
  assert math.isclose(ach_lat_pk1, 6.0, rel_tol=1e-6), (
    f"expected the between-sample |{-6.0}| spike to survive the fold, got {ach_lat_pk1}")
  assert self_obj._flight_peak_achlat_acc == 0.0    # reset after fold

  # trigger fires -> armed onset -> episode peak latches to this window's fold value
  onset(ach_lat_pk1)
  assert self_obj._flight_peak_achlat == ach_lat_pk1

  # ---- Episode 1, window #2: a LOWER peak this window (~2.0 m/s^2) -- the episode peak must hold
  # the earlier, higher value (max-over-episode), not regress to the latest window's smaller peak.
  lower_k = 2.0 / v_ego ** 2
  for _ in range(20):
    accum_tick(lower_k * v_ego, v_ego)
  ach_lat_pk2 = fold()
  assert math.isclose(ach_lat_pk2, 2.0, rel_tol=1e-6)
  armed_update(ach_lat_pk2)
  assert self_obj._flight_peak_achlat == ach_lat_pk1, "episode peak must hold the earlier higher value"

  # ---- Episode 1, window #3: a NEW higher peak (~9.0 m/s^2) -- episode peak must advance to it.
  higher_k = 9.0 / v_ego ** 2
  for _ in range(20):
    accum_tick(higher_k * v_ego, v_ego)
  ach_lat_pk3 = fold()
  armed_update(ach_lat_pk3)
  assert math.isclose(self_obj._flight_peak_achlat, 9.0, rel_tol=1e-6)

  # ---- Episode 1 emits (peakAchLat == 9.0); a NEW episode (idle -> armed again) must NOT inherit
  # episode 1's peak -- onset() always LATCHES (not maxes), so this is exactly what "reset per
  # episode" means in this state machine (mirrors peakAngErr's own onset-latch behavior).
  ep1_peak = self_obj._flight_peak_achlat
  assert math.isclose(ep1_peak, 9.0, rel_tol=1e-6)

  quiet_k = 0.3 / v_ego ** 2
  for _ in range(20):
    accum_tick(quiet_k * v_ego, v_ego)
  ach_lat_pk_ep2 = fold()
  onset(ach_lat_pk_ep2)
  assert math.isclose(self_obj._flight_peak_achlat, 0.3, rel_tol=1e-6), (
    "a new episode's onset must latch to ITS OWN window peak, not carry over the previous episode's")
  assert self_obj._flight_peak_achlat < ep1_peak


def test_ach_lat_accumulator_ignores_non_finite_ticks():
  accum_src, fold_pk_src, reset_acc_src, _onset_src, _armed_src, ach_lat_ms2 = _load_controlsd_pieces()
  self_obj = _Harness()

  def accum_tick(yaw_rate, v_ego):
    ns = {"self": self_obj, "CS": _FakeCS(yaw_rate, v_ego), "MIN_SPEED": MIN_SPEED,
          "_ach_lat_ms2": ach_lat_ms2}
    exec(compile(accum_src, "<accum>", "exec"), ns)

  # a v_ego of 0 exercises the MIN_SPEED clamp in the denominator (yawRate/max(vEgo, MIN_SPEED)); the
  # achLat multiply then uses the raw (unclamped) CS.vEgo per _ach_lat_ms2's own contract, so v_ego=0
  # collapses achLat to 0.0 (finite, not a NaN/raise) -- must not crash and must not corrupt the peak.
  accum_tick(yaw_rate=5.0, v_ego=0.0)
  assert self_obj._flight_peak_achlat_acc == 0.0
  # a genuine finite tick afterward still accumulates normally.
  accum_tick(yaw_rate=3.0 * 10.0, v_ego=10.0)   # k_actl_now = 3.0 -> achLat = 300.0
  assert math.isclose(self_obj._flight_peak_achlat_acc, 300.0, rel_tol=1e-6)


# ---------------------------------------------------------------------------------------------------
# (e) Pure observation: none of this feature's added lines write to any control-affecting name.
# Structural check (not just grep) -- walk the AST of every extracted/added region and assert no
# Assign/AugAssign target touches a known control-state identifier.
# ---------------------------------------------------------------------------------------------------
CONTROL_STATE_NAMES = {
  "desired_curvature", "new_desired_curvature", "actuators", "curvature_limited",
  "steeringAngleDeg", "steer_limited_by_safety",
}


def test_added_controlsd_statements_never_assign_control_state():
  accum_src, fold_pk_src, reset_acc_src, onset_src, armed_src, _ = _load_controlsd_pieces()
  for snippet in (accum_src, fold_pk_src, reset_acc_src, onset_src, armed_src):
    tree = ast.parse(snippet)
    for node in ast.walk(tree):
      if isinstance(node, (ast.Assign, ast.AugAssign)):
        targets = node.targets if isinstance(node, ast.Assign) else [node.target]
        for t in targets:
          name = t.attr if isinstance(t, ast.Attribute) else (t.id if isinstance(t, ast.Name) else None)
          assert name not in CONTROL_STATE_NAMES, f"unexpected control-state write: {name} in {snippet}"


# ---------------------------------------------------------------------------------------------------
# I2 review fix: DEFAULT_LAT_ACCEL_BREAKPOINTS_MPH must include the [80, 3.0] taper-back-to-ISO anchor
# -- the schedule used to end at [70, 4.0], which (np.interp holds flat past the last x) pinned the
# cap at 4.0 m/s^2 -- above the ISO 3.0 baseline -- at every speed above 70 mph forever.
# ---------------------------------------------------------------------------------------------------
def _load_default_lat_accel_breakpoints():
  # DEFAULT_LAT_ACCEL_BREAKPOINTS_MPH is a module-level ANNOTATED assignment
  # (`NAME: list[list[float]] = [...]`) -- an ast.AnnAssign, not the plain ast.Assign
  # _extract_module_assign() handles, so extract it directly here.
  src, tree = _parse(DRIVE_HELPERS_PATH)
  matches = [n for n in tree.body if isinstance(n, ast.AnnAssign)
             and isinstance(n.target, ast.Name) and n.target.id == "DEFAULT_LAT_ACCEL_BREAKPOINTS_MPH"]
  assert len(matches) == 1, f"expected exactly one DEFAULT_LAT_ACCEL_BREAKPOINTS_MPH assign, found {len(matches)}"
  seg = _segment(src, matches[0])
  ns: dict = {}
  exec(compile(seg, "<default_bp>", "exec"), ns)
  return ns["DEFAULT_LAT_ACCEL_BREAKPOINTS_MPH"]


def test_default_breakpoints_taper_to_iso_by_80_mph():
  bps = _load_default_lat_accel_breakpoints()
  assert bps == [[50, 6.0], [60, 5.0], [70, 4.0], [80, 3.0]], bps


def test_default_breakpoints_interpolation_matches_documented_schedule():
  # Same conversion + np.interp _LatAccelSchedule.limit() itself uses (mph -> m/s via CV.MPH_TO_MS,
  # linear interpolation, held flat outside the ends) -- applied to the REAL extracted breakpoints,
  # not a hand-copied schedule, so this catches a future edit to the list as readily as today's fix.
  bps = _load_default_lat_accel_breakpoints()
  xs = np.array([p[0] * CV.MPH_TO_MS for p in bps], dtype=float)
  ys = np.array([p[1] for p in bps], dtype=float)

  def cap_at_mph(mph: float) -> float:
    return float(np.interp(mph * CV.MPH_TO_MS, xs, ys))

  assert math.isclose(cap_at_mph(50), 6.0, rel_tol=1e-9)
  assert math.isclose(cap_at_mph(60), 5.0, rel_tol=1e-9)
  assert math.isclose(cap_at_mph(70), 4.0, rel_tol=1e-9)
  assert math.isclose(cap_at_mph(75), 3.5, rel_tol=1e-9)   # I2: the new 70->80 taper leg
  assert math.isclose(cap_at_mph(80), 3.0, rel_tol=1e-9)   # I2: back at the ISO baseline by 80
  assert math.isclose(cap_at_mph(85), 3.0, rel_tol=1e-9)   # held flat above the last breakpoint


# ---------------------------------------------------------------------------------------------------
# I3 review fix: a steerEvent's heading must come from the bearing BUFFERED nearest the episode's
# ONSET time (controlsd's onsetT), not the live self._cur_bearing at emit time (0.75-5 s later, after
# heading may have rotated 15-20 deg/s through a curve).
# I4 review fix: heading nulls (not "N") when there's no GPS fix at the bearing sample used.
# ---------------------------------------------------------------------------------------------------
def _load_nearest_bearing():
  src, tree = _parse(CES_PNW_PATH)
  return _exec_func(_extract_func(src, tree, "_nearest_bearing"))


def _load_heading_if_fixed():
  src, tree = _parse(CES_PNW_PATH)
  ns = {"math": math}
  exec(compile(_extract_func(src, tree, "_compass"), "<compass>", "exec"), ns)
  pts_src = _extract_module_assign(src, tree, "_COMPASS_PTS")
  exec(compile(pts_src, "<pts>", "exec"), ns)
  return _exec_func(_extract_func(src, tree, "_heading_if_fixed"), extra_globals=ns)


def test_nearest_bearing_picks_closest_sample_by_wall_time():
  nearest_bearing = _load_nearest_bearing()
  hist = [(100.0, 10.0, True), (101.0, 20.0, True), (102.0, 30.0, True), (105.0, 40.0, True)]
  bearing, gps_valid = nearest_bearing(hist, 101.4)
  assert (bearing, gps_valid) == (20.0, True)   # 101.0 is closer to 101.4 than 102.0
  bearing, gps_valid = nearest_bearing(hist, 104.9)
  assert (bearing, gps_valid) == (40.0, True)


def test_nearest_bearing_empty_or_invalid_anchor_degrades_to_none_false():
  nearest_bearing = _load_nearest_bearing()
  assert nearest_bearing([], 100.0) == (None, False)
  assert nearest_bearing([(100.0, 10.0, True)], None) == (None, False)
  assert nearest_bearing([(100.0, 10.0, True)], "garbage") == (None, False)


def test_heading_if_fixed_gates_on_gps_valid_not_bearing_value():
  heading_if_fixed = _load_heading_if_fixed()
  # I4: a real fix (gps_valid True) with bearing 0.0 is genuinely north -- still reported.
  assert heading_if_fixed(0.0, True) == "N"
  assert heading_if_fixed(90.0, True) == "E"
  # I4: no fix (gps_valid False) must null the heading REGARDLESS of what the bearing value is --
  # this is the exact "absent bearing defaults to 0.0 -> reads as confident N" bug.
  assert heading_if_fixed(0.0, False) is None
  assert heading_if_fixed(90.0, False) is None


def _steer_event_harness_globals():
  src, tree = _parse(CES_PNW_PATH)
  ns = {"time": time, "json": json, "math": math}
  # clock_bad() reads the module-level CLOCK_VALID_EPOCH constant -- real source, not invented.
  exec(compile(_extract_module_assign(src, tree, "CLOCK_VALID_EPOCH"), "<epoch>", "exec"), ns)
  exec(compile(_extract_func(src, tree, "clock_bad"), "<clock_bad>", "exec"), ns)
  exec(compile(_extract_func(src, tree, "_nearest_bearing"), "<nearest_bearing>", "exec"), ns)
  pts_src = _extract_module_assign(src, tree, "_COMPASS_PTS")
  exec(compile(pts_src, "<pts>", "exec"), ns)
  exec(compile(_extract_func(src, tree, "_compass"), "<compass>", "exec"), ns)
  exec(compile(_extract_func(src, tree, "_heading_if_fixed"), "<heading_if_fixed>", "exec"), ns)
  # _steer_event_step is a real class method (self included) -- ast.walk() finds FunctionDefs nested
  # in a class body too, so _extract_func grabs its exact body unmodified, same as any top-level def.
  exec(compile(_extract_func(src, tree, "_steer_event_step"), "<steer_event_step>", "exec"), ns)
  return ns


class _MemParamsStub:
  def __init__(self, path):
    self._path = path

  def get_param_path(self, _key):
    return self._path


class _SteerEventHarness:
  """Stub for `self` -- only the attributes/methods _steer_event_step actually touches."""
  def __init__(self, mem_params, cur_lat, cur_lon, cur_bearing, bearing_hist):
    self.mem_params = mem_params
    self._steer_event_frame = 19        # next += 1 -> 20 -> passes the "% 20 == 0" throttle gate
    self._steer_event_raw_last = None
    self._steer_event_seen_id = None
    self._car = "TESTCAR"
    self._mode = 1
    self._cur_lat = cur_lat
    self._cur_lon = cur_lon
    self._cur_bearing = cur_bearing
    self._bearing_hist = bearing_hist
    self.captured: list = []

  def _append_event(self, rec):
    self.captured.append(rec)


def test_steer_event_heading_uses_onset_bearing_not_emit_time_bearing(tmp_path):
  """The core I3 scenario: through a curve, the bearing at the episode's ONSET (4 s before emit) is
  very different from the LIVE bearing at emit time. The old code stamped heading from the live value
  (self._cur_bearing) -- this asserts the fix uses the buffered onset-time sample instead."""
  ns = _steer_event_harness_globals()
  now = time.time()  # noqa: TID251 -- wall clock, matching the real onsetT/t epoch semantics under test
  onset_t = now - 4.0   # emitted 4 s after the saturation onset (within the 0.75-5 s documented lag)
  event = {
    "evId": "salt-1", "t": round(now, 1), "onsetT": round(onset_t, 1), "durationS": 4.0,
    "peakAngErr": 9.0, "peakAchLat": 4.2, "driverOverride": False, "capped": False,
    "frameId": 1, "modelLogMonoTime": 1,
  }
  event_path = tmp_path / "SteerEvent"
  event_path.write_bytes(json.dumps(event).encode())

  # Bearing history rotating through a curve: 90 deg (E) right at onset, ending far away (270, W) by
  # emit time -- exactly the 15-20 deg/s-through-a-curve scenario the I3 fix targets.
  bearing_hist = [
    (onset_t - 1.0, 60.0, True),
    (onset_t, 90.0, True),          # nearest sample to onset_t -> "E"
    (onset_t + 1.0, 130.0, True),
    (onset_t + 2.0, 160.0, True),
    (now, 270.0, True),             # live bearing at emit time -> "W" (the OLD, wrong behavior)
  ]
  harness = _SteerEventHarness(_MemParamsStub(str(event_path)), cur_lat=47.6, cur_lon=-122.3,
                                cur_bearing=270.0, bearing_hist=bearing_hist)

  ns["_steer_event_step"](harness)

  assert len(harness.captured) == 1
  rec = harness.captured[0]
  assert rec["heading"] == "E", f"expected onset-time heading 'E', got {rec['heading']!r}"
  assert rec["heading"] != "W", "must NOT be the live/emit-time bearing's heading"
  # lat/lon/bearing (position) intentionally stay the CURRENT/emit-time snapshot -- only heading
  # changes source (see the rec's own comment in ces_pnw.py).
  assert rec["bearing"] == 270.0
  assert rec["srcT"] == event["t"]   # unchanged field meaning (still the emit time)


def test_steer_event_heading_falls_back_to_emit_time_when_onsett_missing(tmp_path):
  """A steerEvent from a pre-fix controlsd build (no 'onsetT' key) must not crash -- the anchor falls
  back to the emit time 't', same as the pre-I3 behavior."""
  ns = _steer_event_harness_globals()
  now = time.time()  # noqa: TID251 -- wall clock, matching the real onsetT/t epoch semantics under test
  event = {
    "evId": "salt-2", "t": round(now, 1), "durationS": 1.0,   # no onsetT key
    "peakAngErr": 9.0, "peakAchLat": 4.2, "driverOverride": False, "capped": False,
  }
  event_path = tmp_path / "SteerEvent"
  event_path.write_bytes(json.dumps(event).encode())
  bearing_hist = [(now, 45.0, True)]
  harness = _SteerEventHarness(_MemParamsStub(str(event_path)), cur_lat=47.6, cur_lon=-122.3,
                                cur_bearing=999.0, bearing_hist=bearing_hist)

  ns["_steer_event_step"](harness)

  assert len(harness.captured) == 1
  assert harness.captured[0]["heading"] == "NE"   # nearest (only) sample, anchored on the emit time


def test_steer_event_heading_nulls_when_no_gps_fix_at_onset_sample(tmp_path):
  """I4: the buffered sample nearest onset has NO fix (gps_valid False) -- heading must be None, not
  a compass reading of whatever stale/defaulted bearing value happened to be stored."""
  ns = _steer_event_harness_globals()
  now = time.time()  # noqa: TID251 -- wall clock, matching the real onsetT/t epoch semantics under test
  onset_t = now - 2.0
  event = {
    "evId": "salt-3", "t": round(now, 1), "onsetT": round(onset_t, 1), "durationS": 2.0,
    "peakAngErr": 9.0, "peakAchLat": 4.2, "driverOverride": False, "capped": False,
  }
  event_path = tmp_path / "SteerEvent"
  event_path.write_bytes(json.dumps(event).encode())
  # the sample nearest onset_t has gps_valid=False (bearing defaulted to 0.0, no real fix)
  bearing_hist = [(onset_t, 0.0, False), (now, 90.0, True)]
  harness = _SteerEventHarness(_MemParamsStub(str(event_path)), cur_lat=None, cur_lon=None,
                                cur_bearing=None, bearing_hist=bearing_hist)

  ns["_steer_event_step"](harness)

  assert len(harness.captured) == 1
  assert harness.captured[0]["heading"] is None


# ---------------------------------------------------------------------------------------------------
# N1 review fix: a failed/absent vEgo read must degrade achLat to None, not the false "driving
# straight" of k_actl * 0.0**2 == 0.0. A GENUINE 0.0 reading (real standstill, non-noData record)
# must still compute achLat normally (0.0 is not over-nulled).
# ---------------------------------------------------------------------------------------------------
def _match_assign(src, tree, name, extra=lambda n: True):
  return _extract_stmt(src, tree, ast.Assign,
                        lambda n: len(n.targets) == 1 and isinstance(n.targets[0], ast.Name)
                        and n.targets[0].id == name and extra(n))


def test_event_record_achlat_none_on_missing_vego_or_nodata_sentinel():
  """Extracts the EXACT three statements _event_record uses to compute achLat (disambiguated from
  _steer_log_step's differently-shaped statements via AST node type / source-text predicates) and
  exec's them against synthetic `tele` dicts."""
  src, tree = _parse(CES_PNW_PATH)
  ach_lat_fn = _exec_func(_extract_func(src, tree, "_ach_lat"))
  raw_vego_src = _match_assign(
    src, tree, "raw_vego",
    lambda n: isinstance(n.value, ast.Call) and isinstance(n.value.func, ast.Attribute)
    and n.value.func.attr == "get" and isinstance(n.value.func.value, ast.Name)
    and n.value.func.value.id == "tele")
  no_data_src = _match_assign(src, tree, "no_data")
  ach_lat_src = _match_assign(src, tree, "ach_lat", lambda n: isinstance(n.value, ast.IfExp))
  combined = raw_vego_src + "\n" + no_data_src + "\n" + ach_lat_src

  def compute(tele: dict, k_actl):
    ns = {"tele": tele, "self": type("S", (), {"_sl_k_actl": k_actl})(), "_ach_lat": ach_lat_fn}
    exec(compile(combined, "<event_record_achlat>", "exec"), ns)
    return ns["ach_lat"]

  # missing vEgo key entirely -> None, regardless of k_actl
  assert compute({}, 0.02) is None
  # the _publish_status "noData" sentinel (vEgo present as the literal 0.0 placeholder) -> None
  assert compute({"vEgo": 0.0, "reason": "noData"}, 0.02) is None
  # a GENUINE standstill reading (vEgo really is 0.0, NOT the noData sentinel) -> a real 0.0, not None
  assert compute({"vEgo": 0.0, "reason": "curve"}, 0.02) == 0.0
  # a real moving reading computes normally
  assert compute({"vEgo": 20.0, "reason": "curve"}, 0.02) == ach_lat_fn(0.02, 20.0)


def test_steer_log_step_achlat_none_on_failed_vego_read():
  """Same N1 fix, the _steer_log_step copy: getattr(car_state, 'vEgo', None) (not ..., 0.0) feeds
  achLat, so a car_state missing the attribute degrades achLat to None while the DISPLAYED v_ego field
  still shows 0.0 (a deliberately different, cosmetic-only default -- see the code's own comment)."""
  src, tree = _parse(CES_PNW_PATH)
  ach_lat_fn = _exec_func(_extract_func(src, tree, "_ach_lat"))
  raw_vego_src = _match_assign(
    src, tree, "raw_vego",
    lambda n: isinstance(n.value, ast.Call) and isinstance(n.value.func, ast.Name)
    and n.value.func.id == "getattr")
  v_ego_src = _match_assign(
    src, tree, "v_ego",
    lambda n: isinstance(n.value, ast.IfExp))

  class _NoVEgo:
    pass   # deliberately no vEgo attribute -- simulates a malformed/failed read

  class _WithVEgo:
    vEgo = 20.0

  def compute(car_state):
    ns = {"car_state": car_state}
    exec(compile(raw_vego_src + "\n" + v_ego_src, "<steer_log_step_vego>", "exec"), ns)
    return ns["raw_vego"], ns["v_ego"]

  raw_vego, v_ego = compute(_NoVEgo())
  assert raw_vego is None and v_ego == 0.0   # display field still shows 0.0 (cosmetic-only fallback)
  assert ach_lat_fn(0.02, raw_vego) is None   # but achLat correctly degrades to None, not 0.0

  raw_vego, v_ego = compute(_WithVEgo())
  assert raw_vego == 20.0 and v_ego == 20.0
  assert ach_lat_fn(0.02, raw_vego) == ach_lat_fn(0.02, 20.0)


# ---------------------------------------------------------------------------------------------------
# (e) Pure observation, extended: the new I3/I4 helpers take no mutable state, and the bearing-history
# append in _read_map only ever appends to the bounded deque -- never touches any control-state name.
# ---------------------------------------------------------------------------------------------------
def test_new_helpers_are_free_functions_no_self():
  src, tree = _parse(CES_PNW_PATH)
  for name in ("_nearest_bearing", "_heading_if_fixed"):
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    args = [a.arg for a in fn.args.args]
    assert "self" not in args, f"{name} must stay a free function (no self), got args={args}"


def test_steer_event_step_never_assigns_control_state():
  src, tree = _parse(CES_PNW_PATH)
  fn_src = _extract_func(src, tree, "_steer_event_step")
  tree2 = ast.parse(fn_src)
  for node in ast.walk(tree2):
    if isinstance(node, (ast.Assign, ast.AugAssign)):
      targets = node.targets if isinstance(node, ast.Assign) else [node.target]
      for t in targets:
        name = t.attr if isinstance(t, ast.Attribute) else (t.id if isinstance(t, ast.Name) else None)
        assert name not in CONTROL_STATE_NAMES, f"unexpected control-state write: {name} in {fn_src}"


def test_ces_pnw_additions_only_read_never_gate_control():
  # ces_pnw.py's steer/steerEvent record builders are display/log-only by contract (see module
  # docstrings); confirm the two new pure helpers take no `self`/mutable state at all -- they cannot
  # write anything, control-related or otherwise.
  src, tree = _parse(CES_PNW_PATH)
  for name in ("_ach_lat", "_compass"):
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    args = [a.arg for a in fn.args.args]
    assert "self" not in args, f"{name} must stay a free function (no self), got args={args}"
