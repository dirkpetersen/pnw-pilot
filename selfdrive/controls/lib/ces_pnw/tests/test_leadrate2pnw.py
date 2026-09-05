"""
leadrate2pnw — LOGGING ONLY, three small telemetry additions:
  1. hasLead/dRel/vLead on the CES-off "steer" breadcrumb (_steer_log_step) + a "hasLead" alias on
     the tick/adopt records (_event_record) -- lets offline analysis separate "driver holding a speed
     by choice" from "speed forced by a slow lead".
  2. peakCmdRate/peakActRate on "steerEvent" -- peak steering-RATE (deg/s) telemetry, command vs
     achieved, in controlsd.py's flight recorder. The S-curve reversal that motivated this showed a
     ~37 deg/s commanded swing riding on top of an ~18 deg/s achieved-wheel ceiling -- the binding
     limit is a SLEW RATE, not lateral accel.
  3. The "alert" (Take Control) record was simply missing a "heading" key -- added, same
     _heading_if_fixed gate the steer/tick/adopt/steerEvent records already use.

Fable review fixes (this revision):
  I-1/I-2: the FIRST version of (2) differentiated every 100 Hz tick by the fixed nominal DT_CTRL --
    inaccurate for the exact slew number this field exists to measure (a jittered/late tick inflates
    the rate ~2x and the max-accumulator latches it; Ford's 0.1 deg-LSB steeringAngleDeg quantizes a
    per-tick diff into ~10 deg/s steps, and dither alone can fake a 10-20 deg/s peak). Reworked to a
    slope over the existing ~5 Hz FOLD WINDOW using the MEASURED elapsed time between folds (not
    DT_CTRL) -- see controlsd.py's fold-site comment. The raw CS.steeringRateDeg signal (real on
    Tesla) is unaffected -- it was always a plain |value| running max, never differentiated, so it
    never had either problem.
  N-1: dRel/vLead in the _steer_log_step lead fields are now NaN-guarded (mirrors the existing vEgo
    NaN guard in log_take_control_alert) -- a NaN would otherwise round-trip into a bare, invalid-JSON
    NaN token.

Environment note (same as test_steerpower2pnw.py in this directory): this worktree's conftest.py
requires openpilot.common.params_pyx (compiled Cython) and msgq (compiled), neither of which is built
here (a fresh `git worktree add`, no scons run) -- so pytest can't even COLLECT any test in this repo
(conftest.py import fails before test collection starts). `from cereal import log` and a bare
`import openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw` both work standalone in this environment,
but that's irrelevant once pytest insists on loading conftest.py first.

So, like test_steerpower2pnw.py, this file AST-extracts the exact statements/functions/methods this
feature added straight out of the real source files and exec's them in a small stub harness --
proving the literal shipped code (not a reimplementation of it) is correct, without needing the full
package import chain or pytest's conftest at all.
"""
import ast
import math
import textwrap
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[5]
CES_PNW_PATH = REPO_ROOT / "selfdrive/controls/lib/ces_pnw/ces_pnw.py"
CONTROLSD_PATH = REPO_ROOT / "selfdrive/controls/controlsd.py"


# ---------------------------------------------------------------------------------------------------
# AST extraction helpers -- same approach as test_steerpower2pnw.py in this directory (kept as local,
# self-contained copies rather than importing that test module, so this file has no coupling to it).
# ---------------------------------------------------------------------------------------------------
def _parse(path: Path):
  src = path.read_text()
  return src, ast.parse(src, filename=str(path))


def _segment(src: str, node: ast.AST) -> str:
  seg = ast.get_source_segment(src, node)
  assert seg is not None
  return textwrap.dedent((" " * node.col_offset) + seg)


def _extract_func(src: str, tree: ast.AST, name: str) -> str:
  """Return the exact source of the (possibly nested, e.g. a class method) `def <name>(...)`."""
  matches = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name]
  assert len(matches) == 1, f"expected exactly one def {name}(), found {len(matches)}"
  return _segment(src, matches[0])


def _extract_stmt(src: str, tree: ast.AST, node_type, predicate, allow_duplicates: bool = False) -> str:
  matches = [n for n in ast.walk(tree) if isinstance(n, node_type) and predicate(n)]
  segs = [_segment(src, n) for n in matches]
  assert segs, "no matching statement found"
  if allow_duplicates:
    assert len(set(segs)) == 1, f"matches disagree on text: {set(segs)}"
  else:
    assert len(matches) == 1, f"expected exactly one matching {node_type.__name__}, found {len(matches)}"
  return segs[0]


def _extract_module_assign(src: str, tree: ast.AST, name: str) -> str:
  matches = [n for n in tree.body if isinstance(n, ast.Assign)
             and len(n.targets) == 1 and isinstance(n.targets[0], ast.Name) and n.targets[0].id == name]
  assert len(matches) == 1, f"expected exactly one module-level {name} = ..., found {len(matches)}"
  return _segment(src, matches[0])


def _exec_func(src: str, extra_globals: dict | None = None):
  ns = {"math": math}
  if extra_globals:
    ns.update(extra_globals)
  exec(compile(src, "<extracted>", "exec"), ns)
  new_names = [k for k in ns if k not in ("math", "__builtins__", *(extra_globals or {}))]
  assert len(new_names) == 1
  return ns[new_names[0]]


def _assign_by_name(src, tree, name, extra=lambda n: True):
  return _extract_stmt(src, tree, ast.Assign,
                        lambda n: len(n.targets) == 1 and isinstance(n.targets[0], ast.Name)
                        and n.targets[0].id == name and extra(n))


def _attr_assign(src, tree, attr, value_pred, allow_duplicates=False):
  return _extract_stmt(
    src, tree, ast.Assign,
    lambda n: len(n.targets) == 1 and isinstance(n.targets[0], ast.Attribute)
    and n.targets[0].attr == attr and value_pred(n.value),
    allow_duplicates=allow_duplicates)


# =====================================================================================================
# (1) hasLead/dRel/vLead -- the CES-off "steer" breadcrumb (_steer_log_step)
# =====================================================================================================
def _ces_pnw_globals():
  src, tree = _parse(CES_PNW_PATH)
  ns = {"time": time, "math": math}
  exec(compile(_extract_module_assign(src, tree, "CLOCK_VALID_EPOCH"), "<epoch>", "exec"), ns)
  exec(compile(_extract_func(src, tree, "clock_bad"), "<clock_bad>", "exec"), ns)
  exec(compile(_extract_func(src, tree, "_ach_lat"), "<ach_lat>", "exec"), ns)
  pts_src = _extract_module_assign(src, tree, "_COMPASS_PTS")
  exec(compile(pts_src, "<pts>", "exec"), ns)
  exec(compile(_extract_func(src, tree, "_compass"), "<compass>", "exec"), ns)
  exec(compile(_extract_func(src, tree, "_heading_if_fixed"), "<heading_if_fixed>", "exec"), ns)
  return src, tree, ns


class _FakeLead:
  def __init__(self, status, dRel=0.0, vLead=0.0):
    self.status = status
    self.dRel = dRel
    self.vLead = vLead


class _FakeCarState:
  def __init__(self, vEgo=20.0):
    self.vEgo = vEgo


class _TickConst:
  TICK_S = 1.0   # generous throttle so a harness with _steer_tick_last far in the past always passes


class _SteerLogHarness:
  """Stub for `self` -- every attribute/method _steer_log_step actually touches."""
  def __init__(self, cur_lat, cur_lon, cur_bearing):
    self._enabled = False
    self._steer_tick_last = -1e9   # always passes the "now - last < TICK_S" throttle gate
    self._mode = 1
    self._car = "TESTCAR"
    self._cur_lat, self._cur_lon, self._cur_bearing = cur_lat, cur_lon, cur_bearing
    self._speed_limit = 0.0
    self._vtsc_cap = None
    self._vtsc_state = None
    self._lc_corr = self._lc_act = self._lc_gate = self._lc_err = None
    self._lc_lim_n = None   # lcroc2pnw: lane-centering ROC-cap clip counter
    self._sl_curv_lim = self._sl_safe_lim = False
    self._sl_ang_des = self._sl_ang_act = self._sl_ang_err = 0.0
    self._sl_lat_dem = self._sl_lat_max = self._sl_curv_max = 0.0
    self._sl_sat = self._sl_lat_active = self._sl_ang_sat = False
    self._sl_k_cmd = self._sl_k_actl = self._sl_k_err = 0.02
    self.captured: list = []
    self.read_map_calls = 0

  def _read_map(self):
    self.read_map_calls += 1   # real one refreshes sl*/lc*/vtsc*/GPS from mem-params -- no-op here

  def _append_event(self, rec):
    self.captured.append(rec)


def _run_steer_log_step(cur_lat, cur_lon, cur_bearing, sm, v_ego=20.0):
  src, tree, ns = _ces_pnw_globals()
  ns["C"] = _TickConst
  exec(compile(_extract_func(src, tree, "_steer_log_step"), "<steer_log_step>", "exec"), ns)
  harness = _SteerLogHarness(cur_lat, cur_lon, cur_bearing)
  ns["_steer_log_step"](harness, _FakeCarState(v_ego), sm)
  return harness


def test_steer_log_step_lead_fields_present_and_correct_when_lead_exists():
  sm = {"radarState": type("RS", (), {"leadOne": _FakeLead(True, dRel=42.3, vLead=18.7)})()}
  harness = _run_steer_log_step(47.6, -122.3, 90.0, sm)
  assert len(harness.captured) == 1
  rec = harness.captured[0]
  assert rec["hasLead"] is True
  assert rec["dRel"] == 42.3
  assert rec["vLead"] == 18.7
  # types: bool, float, float -- never a bare int/str standing in for the real thing
  assert isinstance(rec["hasLead"], bool)
  assert isinstance(rec["dRel"], float)
  assert isinstance(rec["vLead"], float)


def test_steer_log_step_lead_fields_null_when_no_lead():
  sm = {"radarState": type("RS", (), {"leadOne": _FakeLead(False)})()}
  harness = _run_steer_log_step(47.6, -122.3, 90.0, sm)
  rec = harness.captured[0]
  assert rec["hasLead"] is False
  # dRel/vLead null (not 0.0) -- 0.0 would be indistinguishable from a genuine lead at the bumper
  assert rec["dRel"] is None
  assert rec["vLead"] is None


def test_steer_log_step_lead_read_failure_degrades_to_none_and_never_raises():
  """A missing/malformed radarState (e.g. the message hasn't arrived yet) must not crash the whole
  breadcrumb -- the other fields (vEgo/sl*/achLat/heading) must still be logged. Also the N-2
  three-state contract: a read FAILURE is None, distinct from a genuine False "no lead"."""
  sm = {}   # sm['radarState'] raises KeyError
  harness = _run_steer_log_step(47.6, -122.3, 90.0, sm, v_ego=25.0)
  assert len(harness.captured) == 1, "a lead-read failure must not swallow the whole record"
  rec = harness.captured[0]
  assert rec["hasLead"] is None and rec["dRel"] is None and rec["vLead"] is None
  # everything else in the record still populated normally
  assert rec["ev"] == "steer" and rec["vEgo"] == 25.0
  assert rec["heading"] == "E"


def test_steer_log_step_disabled_gate_and_ev_name_unchanged():
  """Structural sanity: this is still the CES-off breadcrumb (ev="steer", cesOff=True) -- the new
  fields are additive, not a replacement of the existing contract."""
  sm = {"radarState": type("RS", (), {"leadOne": _FakeLead(True, dRel=10.0, vLead=15.0)})()}
  harness = _run_steer_log_step(None, None, None, sm)
  rec = harness.captured[0]
  assert rec["ev"] == "steer" and rec["cesOff"] is True
  assert rec["heading"] is None   # no GPS fix -> heading still nulls, unaffected by this feature
  assert harness.read_map_calls == 1


# =====================================================================================================
# (1d) N-1 review fix -- NaN-guard dRel/vLead. A corrupted radarState message can carry a NaN dRel/
# vLead; round(float(nan), 1) is itself still NaN, which json.dumps serializes as a bare `NaN` token
# (invalid per the JSON spec, breaks strict parsers). Mirrors the existing vEgo NaN guard.
# =====================================================================================================
def test_steer_log_step_lead_nan_drel_degrades_to_none():
  sm = {"radarState": type("RS", (), {"leadOne": _FakeLead(True, dRel=float("nan"), vLead=18.7)})()}
  harness = _run_steer_log_step(47.6, -122.3, 90.0, sm)
  rec = harness.captured[0]
  assert rec["hasLead"] is True   # the lead itself is still real -- only the poisoned field nulls
  assert rec["dRel"] is None, "a NaN dRel must degrade to None, never serialize as a bare NaN token"
  assert rec["vLead"] == 18.7, "the OTHER field must be unaffected by dRel's own NaN"


def test_steer_log_step_lead_nan_vlead_degrades_to_none():
  sm = {"radarState": type("RS", (), {"leadOne": _FakeLead(True, dRel=42.3, vLead=float("nan"))})()}
  harness = _run_steer_log_step(47.6, -122.3, 90.0, sm)
  rec = harness.captured[0]
  assert rec["hasLead"] is True
  assert rec["dRel"] == 42.3
  assert rec["vLead"] is None, "a NaN vLead must degrade to None, never serialize as a bare NaN token"


def test_steer_log_step_lead_inf_drel_degrades_to_none():
  """math.isfinite also rejects +/-inf, not just NaN -- a corrupted radarState could plausibly carry
  either."""
  sm = {"radarState": type("RS", (), {"leadOne": _FakeLead(True, dRel=float("inf"), vLead=18.7)})()}
  harness = _run_steer_log_step(47.6, -122.3, 90.0, sm)
  rec = harness.captured[0]
  assert rec["dRel"] is None


def test_steer_log_step_lead_finite_values_pass_through_unaffected_by_the_guard():
  """The N-1 guard must be a no-op for ordinary finite readings -- same values() as the original
  present/correct test, re-asserted here to prove the guard doesn't clip/alter real data."""
  sm = {"radarState": type("RS", (), {"leadOne": _FakeLead(True, dRel=42.3, vLead=18.7)})()}
  harness = _run_steer_log_step(47.6, -122.3, 90.0, sm)
  rec = harness.captured[0]
  assert rec["dRel"] == 42.3
  assert rec["vLead"] == 18.7


# =====================================================================================================
# (1b) "hasLead" alias on _event_record's tick/adopt output (one-line addition, sourced from the same
# tele["lead"] value decision_telemetry already derives from s["has_lead"]).
# =====================================================================================================
def test_event_record_dict_literal_has_lead_alias_sourced_from_tele_lead():
  src, tree = _parse(CES_PNW_PATH)
  # find the dict-literal key/value pair -- structural check that "hasLead" is present and reads
  # from the exact same tele.get("lead") expression as the pre-existing "lead" field, not reinvented.
  fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "_event_record")
  dicts = [n for n in ast.walk(fn) if isinstance(n, ast.Dict)]
  found = False
  for d in dicts:
    for k, v in zip(d.keys, d.values, strict=True):
      if isinstance(k, ast.Constant) and k.value == "hasLead":
        found = True
        seg = _segment(src, v)
        assert seg == 'tele.get("lead")', f"hasLead should alias tele.get('lead'), got: {seg}"
  assert found, "_event_record's dict literal is missing a 'hasLead' key"


# =====================================================================================================
# (2) peakCmdRate / peakActRate -- controlsd.py's fold-window slope (cmd/act-diff) + the standalone
# 100 Hz |steeringRateDeg| running max, folded together, then latched-at-onset / running-max over the
# episode, exactly like peakAngErr/peakAchLat.
# =====================================================================================================
class _FakeCS:
  def __init__(self, steeringRateDeg=0.0):
    self.steeringRateDeg = steeringRateDeg


class _RateHarness:
  """Stub for `self` -- only the leadrate2pnw attributes the extracted statements touch."""
  def __init__(self):
    self._flight_fold_prev_mono = None
    self._flight_fold_prev_cmd_ang = None
    self._flight_fold_prev_act_ang = None
    self._flight_peak_actrate_sig_acc = 0.0
    self._flight_peak_cmdrate = 0.0
    self._flight_peak_actrate_sig = 0.0
    self._flight_peak_actrate_diff = 0.0


def _load_controlsd_rate_pieces():
  src, tree = _parse(CONTROLSD_PATH)

  # (a) the standalone 100 Hz try block -- now ONLY the |CS.steeringRateDeg| running max (the
  # command/achieved-diff accumulators the I-1/I-2 review fix removed entirely; that computation
  # moved to the fold-window slope below).
  accum_src = _extract_stmt(
    src, tree, ast.Try,
    lambda n: "_flight_peak_actrate_sig_acc" in (ast.get_source_segment(src, n) or "")
    and "sig_rate" in (ast.get_source_segment(src, n) or "")
    and "getattr(CS" in (ast.get_source_segment(src, n) or ""))

  # (b) the fold-window slope computation -- assembled from its constituent statements (prelude +
  # the guarded If block + epilogue), joined in source order, so future comment/doc edits between
  # them can't break extraction (same technique the original per-tick-accumulator test used).
  fold_prelude = "\n".join([
    _assign_by_name(src, tree, "fold_dt"),
    _assign_by_name(src, tree, "cmd_rate_pk",
                     lambda n: isinstance(n.value, ast.Constant) and n.value.value == 0.0),
    _assign_by_name(src, tree, "act_rate_diff_pk",
                     lambda n: isinstance(n.value, ast.Constant) and n.value.value == 0.0),
  ])
  fold_if = _extract_stmt(
    src, tree, ast.If,
    lambda n: (ast.get_source_segment(src, n) or "").startswith("if fold_dt"))
  fold_epilogue = "\n".join([
    _attr_assign(src, tree, "_flight_fold_prev_mono",
                 lambda v: isinstance(v, ast.Name) and v.id == "now_mono"),
    _attr_assign(src, tree, "_flight_fold_prev_cmd_ang",
                 lambda v: isinstance(v, ast.Call)),
    _attr_assign(src, tree, "_flight_fold_prev_act_ang",
                 lambda v: isinstance(v, ast.Name) and v.id == "angle_actual"),
    _assign_by_name(src, tree, "cmd_rate_pk",
                     lambda n: isinstance(n.value, ast.Call) and n.value.func.id == "round"),
    _assign_by_name(src, tree, "act_rate_diff_pk",
                     lambda n: isinstance(n.value, ast.Call) and n.value.func.id == "round"),
    _assign_by_name(src, tree, "act_rate_sig_pk"),
    _attr_assign(src, tree, "_flight_peak_actrate_sig_acc",
                 lambda v: isinstance(v, ast.Constant) and v.value == 0.0, allow_duplicates=True),
  ])
  fold_src = "\n".join([fold_prelude, fold_if, fold_epilogue])

  onset_src = "\n".join([
    _attr_assign(src, tree, "_flight_peak_cmdrate",
                 lambda v: isinstance(v, ast.Name) and v.id == "cmd_rate_pk"),
    _attr_assign(src, tree, "_flight_peak_actrate_sig",
                 lambda v: isinstance(v, ast.Name) and v.id == "act_rate_sig_pk"),
    _attr_assign(src, tree, "_flight_peak_actrate_diff",
                 lambda v: isinstance(v, ast.Name) and v.id == "act_rate_diff_pk"),
  ])
  armed_src = "\n".join([
    _attr_assign(src, tree, "_flight_peak_cmdrate",
                 lambda v: isinstance(v, ast.Call) and isinstance(v.func, ast.Name) and v.func.id == "max"),
    _attr_assign(src, tree, "_flight_peak_actrate_sig",
                 lambda v: isinstance(v, ast.Call) and isinstance(v.func, ast.Name) and v.func.id == "max"),
    _attr_assign(src, tree, "_flight_peak_actrate_diff",
                 lambda v: isinstance(v, ast.Call) and isinstance(v.func, ast.Name) and v.func.id == "max"),
  ])
  fallback_src = _assign_by_name(src, tree, "act_rate_pk", lambda n: isinstance(n.value, ast.IfExp))

  fold_dt_min_src = _extract_module_assign(src, tree, "FLIGHT_RATE_FOLD_DT_MIN")
  fold_dt_max_src = _extract_module_assign(src, tree, "FLIGHT_RATE_FOLD_DT_MAX")
  const_ns: dict = {}
  exec(compile(fold_dt_min_src, "<min>", "exec"), const_ns)
  exec(compile(fold_dt_max_src, "<max>", "exec"), const_ns)
  fold_dt_min = const_ns["FLIGHT_RATE_FOLD_DT_MIN"]
  fold_dt_max = const_ns["FLIGHT_RATE_FOLD_DT_MAX"]

  return accum_src, fold_src, onset_src, armed_src, fallback_src, fold_dt_min, fold_dt_max


def _accum_tick(self_obj, accum_src, sig_rate):
  ns = {"self": self_obj, "CS": _FakeCS(sig_rate), "math": math}
  exec(compile(accum_src, "<accum>", "exec"), ns)


def _fold(self_obj, fold_src, fold_dt_min, fold_dt_max, now_mono, angle_des, angle_actual):
  ns = {"self": self_obj, "now_mono": now_mono, "angle_des": angle_des, "angle_actual": angle_actual,
        "math": math, "FLIGHT_RATE_FOLD_DT_MIN": fold_dt_min, "FLIGHT_RATE_FOLD_DT_MAX": fold_dt_max}
  exec(compile(fold_src, "<fold>", "exec"), ns)
  return ns["cmd_rate_pk"], ns["act_rate_sig_pk"], ns["act_rate_diff_pk"]


def _onset(self_obj, onset_src, pks):
  ns = {"self": self_obj, "cmd_rate_pk": pks[0], "act_rate_sig_pk": pks[1], "act_rate_diff_pk": pks[2]}
  exec(compile(onset_src, "<onset>", "exec"), ns)


def _armed(self_obj, armed_src, pks):
  ns = {"self": self_obj, "cmd_rate_pk": pks[0], "act_rate_sig_pk": pks[1], "act_rate_diff_pk": pks[2]}
  exec(compile(armed_src, "<armed>", "exec"), ns)


# --- (2a) the motivating scenario: the windowed slope reads the true 37 / 18 deg/s rates -------------
def test_windowed_slope_reads_37_and_18_degs_within_a_few_degs():
  """Validate item (a): a synthetic 37 deg/s COMMAND ramp and an 18 deg/s ACHIEVED ramp (steeringRateDeg
  dead -- the Ford-build diff-fallback case) must read back within a few deg/s of the true rate, NOT
  ~2x off the way the old fixed-DT_CTRL per-tick accumulator could read on a jittered tick."""
  accum_src, fold_src, onset_src, armed_src, _, dt_min, dt_max = _load_controlsd_rate_pieces()
  self_obj = _RateHarness()

  now = 1000.0
  cmd, act = 0.0, 0.0
  # first fold: no previous fold yet -> both rates must be exactly 0.0 (no fake reading from nothing)
  pks0 = _fold(self_obj, fold_src, dt_min, dt_max, now, cmd, act)
  assert pks0[0] == 0.0 and pks0[2] == 0.0

  # nominal ~200 ms fold window, steady 37 / 18 deg/s ramps
  FOLD_S = 0.2
  now += FOLD_S
  cmd += 37.0 * FOLD_S    # 7.4 deg of travel this window
  act += 18.0 * FOLD_S    # 3.6 deg of travel this window
  _accum_tick(self_obj, accum_src, sig_rate=0.0)   # steeringRateDeg dead the whole episode
  cmd_pk, sig_pk, diff_pk = _fold(self_obj, fold_src, dt_min, dt_max, now, cmd, act)

  assert abs(cmd_pk - 37.0) < 3.0, f"expected ~37 deg/s peak cmd rate (within a few deg/s), got {cmd_pk}"
  assert abs(diff_pk - 18.0) < 3.0, f"expected ~18 deg/s peak act rate (within a few deg/s), got {diff_pk}"
  assert sig_pk == 0.0, "steeringRateDeg never moved -- its accumulator must stay dead"

  _onset(self_obj, onset_src, (cmd_pk, sig_pk, diff_pk))
  assert self_obj._flight_peak_cmdrate == cmd_pk
  assert self_obj._flight_peak_actrate_diff == diff_pk


# --- (2b) jitter robustness: measured dt cancels the jitter, unlike the old fixed-DT_CTRL divide -----
def test_jittered_fold_with_proportional_motion_does_not_inflate_the_rate():
  """Validate item (b): a fold that runs 2x the nominal 200 ms window, carrying 2x the motion (both the
  jittered LATE-TICK case), must still read the SAME true rate -- the measured dt in the denominator
  cancels the extra time exactly like it cancels the extra travel. This is the I-1 fix under direct
  test: the old code divided by the FIXED nominal DT_CTRL regardless of how late the tick actually ran,
  which is what inflated the rate ~2x on a jittered/late tick."""
  accum_src, fold_src, onset_src, armed_src, _, dt_min, dt_max = _load_controlsd_rate_pieces()

  # Baseline: nominal fold, nominal motion.
  base = _RateHarness()
  now = 1000.0
  _fold(base, fold_src, dt_min, dt_max, now, 0.0, 0.0)
  now += 0.2
  base_pks = _fold(base, fold_src, dt_min, dt_max, now, 37.0 * 0.2, 18.0 * 0.2)

  # Jittered: a SINGLE fold that took 2x as long (0.4 s, still inside the sane band) and therefore
  # carried 2x the motion of a normal window -- the exact "one jittered/late tick sees 2-3x the
  # motion" scenario the I-1 review comment describes, just expressed at fold granularity.
  jit = _RateHarness()
  now_j = 2000.0
  _fold(jit, fold_src, dt_min, dt_max, now_j, 0.0, 0.0)
  now_j += 0.4                          # 2x the nominal fold period
  jit_pks = _fold(jit, fold_src, dt_min, dt_max, now_j, 37.0 * 0.4, 18.0 * 0.4)   # 2x the motion too

  assert math.isclose(jit_pks[0], base_pks[0], rel_tol=0.02), \
    f"jittered 2x-dt/2x-motion fold must read the SAME rate as the nominal fold: {jit_pks[0]} vs {base_pks[0]}"
  assert math.isclose(jit_pks[2], base_pks[2], rel_tol=0.02), \
    f"jittered 2x-dt/2x-motion fold must read the SAME rate as the nominal fold: {jit_pks[2]} vs {base_pks[2]}"
  assert not (jit_pks[0] > base_pks[0] * 1.5), "must NOT be inflated ~2x by the longer/late fold"


def test_fold_outside_sane_dt_band_is_skipped_not_garbage():
  """A fold_dt outside [FLIGHT_RATE_FOLD_DT_MIN, FLIGHT_RATE_FOLD_DT_MAX] (a stalled/starved loop, or
  a clock hiccup) must contribute 0.0, not divide-by-a-tiny/huge-number garbage."""
  accum_src, fold_src, onset_src, armed_src, _, dt_min, dt_max = _load_controlsd_rate_pieces()
  self_obj = _RateHarness()
  now = 1000.0
  _fold(self_obj, fold_src, dt_min, dt_max, now, 0.0, 0.0)
  # a fold that took 5 seconds (way outside the 0.5 s ceiling) with a big angle jump
  now += 5.0
  cmd_pk, sig_pk, diff_pk = _fold(self_obj, fold_src, dt_min, dt_max, now, 50.0, 50.0)
  assert cmd_pk == 0.0 and diff_pk == 0.0, "an out-of-band fold_dt must be skipped, not divided through"


# --- (2c) Ford 0.1 deg-LSB quantization robustness ----------------------------------------------------
def test_ford_quantized_18_degs_ramp_reads_close_not_floored_or_dithered():
  """Validate item (c): a Ford-style steeringAngleDeg ramp at 18 deg/s, ROUNDED to the real 0.1 deg
  LSB (and with +/-1-LSB dither applied at the fold boundary, exactly the noise the I-2 review comment
  describes), must read back close to 18 -- NOT floored to a ~10 deg/s quantization step and NOT
  dithered up to ~30 by dither alone. 200 ms of travel at 18 deg/s is 3.6 deg -- ~36 LSBs -- so a single
  +/-0.1 dither sample is a small fraction of the window's real travel."""
  accum_src, fold_src, onset_src, armed_src, _, dt_min, dt_max = _load_controlsd_rate_pieces()
  self_obj = _RateHarness()

  def lsb_round(x):
    return round(x, 1)   # Ford's real steeringAngleDeg LSB

  now = 1000.0
  act = 0.0
  _fold(self_obj, fold_src, dt_min, dt_max, now, 0.0, lsb_round(act))
  dithers = [+0.1, -0.1, +0.1, -0.1, 0.0]
  readings = []
  for dither in dithers:
    now += 0.2
    act += 18.0 * 0.2
    sampled = lsb_round(act + dither)
    _, _, diff_pk = _fold(self_obj, fold_src, dt_min, dt_max, now, 0.0, sampled)
    readings.append(diff_pk)

  for r in readings:
    assert abs(r - 18.0) <= 1.5, f"expected ~18 deg/s (+/- ~1), got {r} -- floored/dithered reading"
    assert r < 25.0, f"must not be dithered up toward ~30, got {r}"
    assert r > 12.0, f"must not be floored down toward ~10, got {r}"


# --- (2d) steeringRateDeg (Tesla, real 100 Hz signal, never had either problem) -----------------------
def test_peak_actrate_sig_accumulator_captures_a_live_steeringratedeg_signal():
  """Sanity check the OTHER side of the fallback: when steeringRateDeg genuinely moves (the Tesla
  case), the signal-based accumulator captures its peak, independent of the fold-window diff -- and
  since it's a plain |value| running max (never differentiated), it's unaffected by the I-1/I-2 fixes."""
  accum_src, fold_src, _onset_src, _armed_src, _, dt_min, dt_max = _load_controlsd_rate_pieces()
  self_obj = _RateHarness()

  for i in range(10):
    _accum_tick(self_obj, accum_src, sig_rate=(22.5 if i == 5 else 3.0))
  _, sig_pk, _ = _fold(self_obj, fold_src, dt_min, dt_max, 1000.0, 0.0, 0.0)
  assert math.isclose(sig_pk, 22.5, rel_tol=1e-6)


def test_sig_accumulator_ignores_non_finite_ticks():
  accum_src, fold_src, _onset_src, _armed_src, _, dt_min, dt_max = _load_controlsd_rate_pieces()
  self_obj = _RateHarness()
  _accum_tick(self_obj, accum_src, sig_rate=float("nan"))
  assert self_obj._flight_peak_actrate_sig_acc == 0.0, "NaN sig_rate must be ignored, not accumulated"
  _accum_tick(self_obj, accum_src, sig_rate=5.0)
  assert self_obj._flight_peak_actrate_sig_acc == 5.0


def test_episode_peak_holds_earlier_higher_value_and_new_episode_latches_fresh():
  accum_src, fold_src, onset_src, armed_src, _, dt_min, dt_max = _load_controlsd_rate_pieces()
  self_obj = _RateHarness()
  now = 1000.0
  _fold(self_obj, fold_src, dt_min, dt_max, now, 0.0, 0.0)
  now += 0.2
  pks1 = _fold(self_obj, fold_src, dt_min, dt_max, now, 37.0 * 0.2, 18.0 * 0.2)
  _onset(self_obj, onset_src, pks1)
  assert self_obj._flight_peak_cmdrate == pks1[0]

  # a LOWER-rate window -- episode peak must hold the earlier, higher value (running max, armed state)
  now += 0.2
  pks2 = _fold(self_obj, fold_src, dt_min, dt_max, now, 0.2, 0.1)
  _armed(self_obj, armed_src, pks2)
  assert self_obj._flight_peak_cmdrate == pks1[0], "episode peak must hold the earlier higher value"

  # a NEW episode (onset again) must NOT inherit the old episode's peak -- onset always LATCHES fresh.
  now += 0.2
  pks3 = _fold(self_obj, fold_src, dt_min, dt_max, now, 0.05, 0.05)
  _onset(self_obj, onset_src, pks3)
  assert self_obj._flight_peak_cmdrate == pks3[0]
  assert self_obj._flight_peak_cmdrate < pks1[0]


# =====================================================================================================
# (2e) The emit-site fallback: prefer the live steeringRateDeg signal, fall back to the steeringAngleDeg
# diff only when the signal never moved this episode. Must be a pure numeric comparison -- no
# carFingerprint/brand check anywhere (the capability-view rule: this file never branches on car
# identity in feature code). Unchanged by the I-1/I-2 rework -- re-asserted here for completeness.
# =====================================================================================================
def test_actrate_fallback_prefers_live_signal_when_alive():
  _, _, _, _, fallback_src, _, _ = _load_controlsd_rate_pieces()
  ns = {"self": type("S", (), {"_flight_peak_actrate_sig": 22.5, "_flight_peak_actrate_diff": 5.0})()}
  exec(compile(fallback_src, "<fallback>", "exec"), ns)
  assert ns["act_rate_pk"] == 22.5, "a live (non-trivial) steeringRateDeg signal must win"


def test_actrate_fallback_uses_diff_when_signal_dead():
  """The Ford Lightning case: opendbc_repo/opendbc/car/ford/carstate.py never sets steeringRateDeg,
  so it stays pinned at the capnp default 0.0 for the whole episode -- the diff accumulator must be
  used instead."""
  _, _, _, _, fallback_src, _, _ = _load_controlsd_rate_pieces()
  ns = {"self": type("S", (), {"_flight_peak_actrate_sig": 0.0, "_flight_peak_actrate_diff": 18.0})()}
  exec(compile(fallback_src, "<fallback>", "exec"), ns)
  assert ns["act_rate_pk"] == 18.0, "a dead (0.0) signal must fall back to the diff accumulator"


def test_actrate_fallback_source_selection_has_no_car_branch():
  """Structural check: the fallback expression itself must not reference carFingerprint/brand -- the
  decision is purely 'did this signal move', evaluated identically on every car."""
  _, _, _, _, fallback_src, _, _ = _load_controlsd_rate_pieces()
  assert "carFingerprint" not in fallback_src
  assert "brand" not in fallback_src
  assert ".CP." not in fallback_src


# =====================================================================================================
# (3) The "alert" (Take Control) record's heading -- was simply absent, now uses the same
# _heading_if_fixed gate the steer/tick/adopt/steerEvent records already use.
# =====================================================================================================
class _AlertHarness:
  """Stub for `self` -- every attribute log_take_control_alert touches."""
  def __init__(self, cur_lat, cur_lon, cur_bearing):
    self._car = "TESTCAR"
    self._mode = 1
    self._cur_lat, self._cur_lon, self._cur_bearing = cur_lat, cur_lon, cur_bearing
    self._sl_ang_des = self._sl_ang_act = self._sl_ang_err = 0.0
    self._sl_lat_dem = self._sl_lat_max = 0.0
    self.captured: list = []

  def _append_event(self, rec):
    self.captured.append(rec)


def _extract_alert_method(src: str, tree: ast.AST) -> str:
  """log_take_control_alert exists TWICE in this file -- the real CESController implementation and
  CESStub's inert no-op fallback (see CESStub's own docstring: "no event log writer exists in this
  fallback"). Disambiguate on the real one's body, which builds the {"ev":"alert",...} record."""
  matches = [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "log_take_control_alert"
             and '"ev": "alert"' in (ast.get_source_segment(src, n) or "")]
  assert len(matches) == 1, f"expected exactly one real log_take_control_alert(), found {len(matches)}"
  return _segment(src, matches[0])


def _run_log_take_control_alert(cur_lat, cur_lon, cur_bearing, payload):
  src, tree, ns = _ces_pnw_globals()
  exec(compile(_extract_alert_method(src, tree), "<alert>", "exec"), ns)
  harness = _AlertHarness(cur_lat, cur_lon, cur_bearing)
  ns["log_take_control_alert"](harness, payload)
  return harness


def test_alert_heading_populated_on_valid_gps_fix():
  payload = {"name": "steerSaturated", "phase": "start", "vEgo": 20.0, "otherEvents": []}
  harness = _run_log_take_control_alert(47.6, -122.3, 90.0, payload)
  assert len(harness.captured) == 1
  rec = harness.captured[0]
  assert rec["gps"] is True
  assert rec["heading"] == "E", f"expected 'E' from bearing 90.0 with a valid fix, got {rec['heading']!r}"


def test_alert_heading_none_on_genuine_no_fix():
  payload = {"name": "steerSaturated", "phase": "end", "vEgo": 20.0, "otherEvents": [], "durationS": 2.0}
  harness = _run_log_take_control_alert(None, None, None, payload)
  rec = harness.captured[0]
  assert rec["gps"] is False
  assert rec["heading"] is None


def test_alert_heading_key_present_in_dict_literal():
  """Structural check: 'heading' must be a real key in log_take_control_alert's dict literal, not
  something that only happens to read back as null because the key was absent entirely (the original
  bug -- there was no 'heading' key at all)."""
  src, tree = _parse(CES_PNW_PATH)
  fn_src = _extract_alert_method(src, tree)
  fn_tree = ast.parse(fn_src)
  dicts = [n for n in ast.walk(fn_tree) if isinstance(n, ast.Dict)]
  keys = {k.value for d in dicts for k in d.keys if isinstance(k, ast.Constant)}
  assert "heading" in keys


# =====================================================================================================
# (2c-passthrough) steerEvent pass-through: peakCmdRate/peakActRate appear in _steer_event_step's
# emitted record, sourced straight from the incoming mem-param payload (no re-derivation in ces_pnw.py).
# =====================================================================================================
def test_steer_event_step_passes_through_peak_cmd_and_act_rate():
  import json
  src, tree = _parse(CES_PNW_PATH)
  ns = {"time": time, "json": json, "math": math}
  exec(compile(_extract_module_assign(src, tree, "CLOCK_VALID_EPOCH"), "<epoch>", "exec"), ns)
  exec(compile(_extract_func(src, tree, "clock_bad"), "<clock_bad>", "exec"), ns)
  exec(compile(_extract_func(src, tree, "_nearest_bearing"), "<nearest_bearing>", "exec"), ns)
  pts_src = _extract_module_assign(src, tree, "_COMPASS_PTS")
  exec(compile(pts_src, "<pts>", "exec"), ns)
  exec(compile(_extract_func(src, tree, "_compass"), "<compass>", "exec"), ns)
  exec(compile(_extract_func(src, tree, "_heading_if_fixed"), "<heading_if_fixed>", "exec"), ns)
  exec(compile(_extract_func(src, tree, "_steer_event_step"), "<steer_event_step>", "exec"), ns)

  class _MemParamsStub:
    def __init__(self, path):
      self._path = path

    def get_param_path(self, _key):
      return self._path

  class _Harness:
    def __init__(self, mem_params):
      self.mem_params = mem_params
      self._steer_event_frame = 19
      self._steer_event_raw_last = None
      self._steer_event_seen_id = None
      self._car = "TESTCAR"
      self._mode = 1
      self._cur_lat = self._cur_lon = self._cur_bearing = None
      self._bearing_hist: list = []
      self.captured: list = []

    def _append_event(self, rec):
      self.captured.append(rec)

  import os
  import tempfile
  now = time.time()  # noqa: TID251 -- test setup, matches the real onsetT/t epoch semantics under test
  event = {
    "evId": "salt-1", "t": round(now, 1), "durationS": 2.0,
    "peakAngErr": 9.0, "peakAchLat": 4.2,
    "peakCmdRate": 37.0, "peakActRate": 18.0,
    "driverOverride": False, "capped": False,
  }
  fd, path = tempfile.mkstemp()
  os.close(fd)
  try:
    with open(path, "wb") as f:
      f.write(json.dumps(event).encode())
    harness = _Harness(_MemParamsStub(path))
    ns["_steer_event_step"](harness)
    assert len(harness.captured) == 1
    rec = harness.captured[0]
    assert rec["peakCmdRate"] == 37.0
    assert rec["peakActRate"] == 18.0
  finally:
    os.unlink(path)


# =====================================================================================================
# (e) Pure observation: none of this feature's added statements write to any control-affecting name.
# Same structural AST-walk check test_steerpower2pnw.py uses for its own additions.
# =====================================================================================================
CONTROL_STATE_NAMES = {
  "desired_curvature", "new_desired_curvature", "actuators", "curvature_limited",
  "steeringAngleDeg", "steer_limited_by_safety",
}


def _assert_no_control_writes(snippet: str):
  tree = ast.parse(snippet)
  for node in ast.walk(tree):
    if isinstance(node, (ast.Assign, ast.AugAssign)):
      targets = node.targets if isinstance(node, ast.Assign) else [node.target]
      for t in targets:
        name = t.attr if isinstance(t, ast.Attribute) else (t.id if isinstance(t, ast.Name) else None)
        assert name not in CONTROL_STATE_NAMES, f"unexpected control-state write: {name} in {snippet}"


def test_controlsd_rate_additions_never_assign_control_state():
  accum_src, fold_src, onset_src, armed_src, fallback_src, _, _ = _load_controlsd_rate_pieces()
  for snippet in (accum_src, fold_src, onset_src, armed_src, fallback_src):
    _assert_no_control_writes(snippet)


def test_ces_pnw_steer_log_step_never_assigns_control_state():
  src, tree = _parse(CES_PNW_PATH)
  fn_src = _extract_func(src, tree, "_steer_log_step")
  _assert_no_control_writes(fn_src)


def test_ces_pnw_alert_never_assigns_control_state():
  src, tree = _parse(CES_PNW_PATH)
  fn_src = _extract_alert_method(src, tree)
  _assert_no_control_writes(fn_src)
