"""
leadrate2pnw — LOGGING ONLY, three small telemetry additions:
  1. hasLead/dRel/vLead on the CES-off "steer" breadcrumb (_steer_log_step) + a "hasLead" alias on
     the tick/adopt records (_event_record) -- lets offline analysis separate "driver holding a speed
     by choice" from "speed forced by a slow lead".
  2. peakCmdRate/peakActRate on "steerEvent" -- two new 100 Hz peak accumulators in controlsd.py's
     flight recorder (same accumulate/fold/latch/reset pattern peakAngErr/peakAchLat already use).
     The S-curve reversal that motivated this showed a ~37 deg/s commanded swing riding on top of an
     ~18 deg/s achieved-wheel ceiling -- the binding limit is a SLEW RATE, not lateral accel.
  3. The "alert" (Take Control) record was simply missing a "heading" key -- added, same
     _heading_if_fixed gate the steer/tick/adopt/steerEvent records already use.

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

from openpilot.common.realtime import DT_CTRL   # imports cleanly standalone (no cereal pull-in)

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
  breadcrumb -- the other fields (vEgo/sl*/achLat/heading) must still be logged."""
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
# (2) peakCmdRate / peakActRate -- controlsd.py's 100 Hz accumulator + fold + onset + armed-running-max
# =====================================================================================================
class _FakeLacLog:
  def __init__(self, steeringAngleDesiredDeg):
    self.steeringAngleDesiredDeg = steeringAngleDesiredDeg


class _FakeCS:
  def __init__(self, steeringAngleDeg, steeringRateDeg=0.0):
    self.steeringAngleDeg = steeringAngleDeg
    self.steeringRateDeg = steeringRateDeg


class _RateHarness:
  """Stub for `self` -- only the leadrate2pnw attributes the extracted statements touch."""
  def __init__(self):
    self._flight_prev_cmd_ang = None
    self._flight_prev_act_ang = None
    self._flight_peak_cmdrate_acc = 0.0
    self._flight_peak_actrate_sig_acc = 0.0
    self._flight_peak_actrate_diff_acc = 0.0
    self._flight_peak_cmdrate = 0.0
    self._flight_peak_actrate_sig = 0.0
    self._flight_peak_actrate_diff = 0.0


def _load_controlsd_rate_pieces():
  src, tree = _parse(CONTROLSD_PATH)
  accum_src = _extract_stmt(
    src, tree, ast.Try,
    lambda n: "cmd_ang_now" in (ast.get_source_segment(src, n) or "")
    and "_flight_peak_cmdrate_acc" in (ast.get_source_segment(src, n) or ""))
  fold_src = "\n".join([
    _assign_by_name(src, tree, "cmd_rate_pk"),
    _attr_assign(src, tree, "_flight_peak_cmdrate_acc",
                 lambda v: isinstance(v, ast.Constant) and v.value == 0.0, allow_duplicates=True),
    _assign_by_name(src, tree, "act_rate_sig_pk"),
    _attr_assign(src, tree, "_flight_peak_actrate_sig_acc",
                 lambda v: isinstance(v, ast.Constant) and v.value == 0.0, allow_duplicates=True),
    _assign_by_name(src, tree, "act_rate_diff_pk"),
    _attr_assign(src, tree, "_flight_peak_actrate_diff_acc",
                 lambda v: isinstance(v, ast.Constant) and v.value == 0.0, allow_duplicates=True),
  ])
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
  return accum_src, fold_src, onset_src, armed_src, fallback_src


def test_peak_cmd_and_act_rate_capture_synthetic_37_and_18_deg_per_s_reversal():
  """The motivating scenario: over one ~5 Hz window, the commanded angle takes one big between-sample
  step equivalent to a 37 deg/s swing, and the achieved angle (steeringRateDeg dead/0.0, so the diff
  accumulator is what's exercised) takes a smaller 18 deg/s step -- exactly the S-curve reversal
  capability gap this feature exists to measure. A throttled ~5 Hz-only sample would alias both peaks
  away; the 100 Hz accumulator must not."""
  accum_src, fold_src, onset_src, armed_src, _ = _load_controlsd_rate_pieces()
  self_obj = _RateHarness()

  def accum_tick(cmd_ang, act_ang, sig_rate=0.0):
    ns = {"self": self_obj, "lac_log": _FakeLacLog(cmd_ang), "CS": _FakeCS(act_ang, sig_rate),
          "math": math, "DT_CTRL": DT_CTRL}
    exec(compile(accum_src, "<accum>", "exec"), ns)

  def fold():
    ns = {"self": self_obj}
    exec(compile(fold_src, "<fold>", "exec"), ns)
    return ns["cmd_rate_pk"], ns["act_rate_sig_pk"], ns["act_rate_diff_pk"]

  def onset(pks):
    ns = {"self": self_obj, "cmd_rate_pk": pks[0], "act_rate_sig_pk": pks[1], "act_rate_diff_pk": pks[2]}
    exec(compile(onset_src, "<onset>", "exec"), ns)

  def armed(pks):
    ns = {"self": self_obj, "cmd_rate_pk": pks[0], "act_rate_sig_pk": pks[1], "act_rate_diff_pk": pks[2]}
    exec(compile(armed_src, "<armed>", "exec"), ns)

  # ---- Episode 1, window #1: steady small motion (1 deg/s) EXCEPT tick #10, a single between-sample
  # step equivalent to 37 deg/s (cmd) / 18 deg/s (act) -- DT_CTRL=0.01s so that's a 0.37 / 0.18 deg
  # step in that one tick. steeringRateDeg stays 0.0 the whole time (the Ford-build "dead signal"
  # case), so act_rate_sig_pk must stay 0.0 and act_rate_diff_pk must carry the real peak.
  cmd, act = 0.0, 0.0
  for i in range(20):
    if i == 10:
      cmd += 37.0 * DT_CTRL
      act += 18.0 * DT_CTRL
    else:
      cmd += 1.0 * DT_CTRL
      act += 1.0 * DT_CTRL
    accum_tick(cmd, act, sig_rate=0.0)
  cmd_pk1, sig_pk1, diff_pk1 = fold()
  assert math.isclose(cmd_pk1, 37.0, rel_tol=1e-6), f"expected ~37 deg/s peak cmd rate, got {cmd_pk1}"
  assert math.isclose(diff_pk1, 18.0, rel_tol=1e-6), f"expected ~18 deg/s peak act rate, got {diff_pk1}"
  assert sig_pk1 == 0.0, "steeringRateDeg never moved -- its accumulator must stay dead"
  # accumulators reset after fold
  assert self_obj._flight_peak_cmdrate_acc == 0.0
  assert self_obj._flight_peak_actrate_sig_acc == 0.0
  assert self_obj._flight_peak_actrate_diff_acc == 0.0

  # trigger fires -> onset latches the episode peak to this window's fold values
  onset((cmd_pk1, sig_pk1, diff_pk1))
  assert self_obj._flight_peak_cmdrate == cmd_pk1
  assert self_obj._flight_peak_actrate_diff == diff_pk1

  # ---- window #2: a LOWER peak this window -- episode peak must hold the earlier, higher value.
  for _ in range(20):
    cmd += 0.5 * DT_CTRL
    act += 0.3 * DT_CTRL
    accum_tick(cmd, act, sig_rate=0.0)
  pks2 = fold()
  armed(pks2)
  assert self_obj._flight_peak_cmdrate == cmd_pk1, "episode peak must hold the earlier higher value"
  assert self_obj._flight_peak_actrate_diff == diff_pk1

  # ---- a NEW episode (onset again) must NOT inherit episode 1's peak -- onset always LATCHES.
  # Continue differentiating from the CURRENT cmd/act values (not a fresh 0.0) -- the real angle
  # signal doesn't teleport at an episode boundary, and resetting the local var here (while
  # self._flight_prev_cmd_ang still holds the true last-seen angle) would inject a fake discontinuity
  # that has nothing to do with the state machine under test.
  for _ in range(20):
    cmd += 0.2 * DT_CTRL
    act += 0.2 * DT_CTRL
    accum_tick(cmd, act, sig_rate=0.0)
  pks_ep2 = fold()
  onset(pks_ep2)
  assert self_obj._flight_peak_cmdrate < cmd_pk1
  assert self_obj._flight_peak_cmdrate == pks_ep2[0]


def test_peak_actrate_sig_accumulator_captures_a_live_steeringratedeg_signal():
  """Sanity check the OTHER side of the fallback: when steeringRateDeg genuinely moves (the Tesla
  case), the signal-based accumulator captures its peak too, independent of the diff accumulator."""
  accum_src, fold_src, _onset_src, _armed_src, _ = _load_controlsd_rate_pieces()
  self_obj = _RateHarness()

  def accum_tick(cmd_ang, act_ang, sig_rate):
    ns = {"self": self_obj, "lac_log": _FakeLacLog(cmd_ang), "CS": _FakeCS(act_ang, sig_rate),
          "math": math, "DT_CTRL": DT_CTRL}
    exec(compile(accum_src, "<accum>", "exec"), ns)

  for i in range(10):
    accum_tick(0.0, 0.0, sig_rate=(22.5 if i == 5 else 3.0))
  ns = {"self": self_obj}
  exec(compile(fold_src, "<fold>", "exec"), ns)
  assert math.isclose(ns["act_rate_sig_pk"], 22.5, rel_tol=1e-6)


def test_rate_accumulators_ignore_non_finite_and_none_ticks():
  accum_src, fold_src, _onset_src, _armed_src, _ = _load_controlsd_rate_pieces()
  self_obj = _RateHarness()

  def accum_tick(cmd_ang, act_ang, sig_rate=0.0):
    ns = {"self": self_obj, "lac_log": _FakeLacLog(cmd_ang), "CS": _FakeCS(act_ang, sig_rate),
          "math": math, "DT_CTRL": DT_CTRL}
    exec(compile(accum_src, "<accum>", "exec"), ns)

  # lac_log without steeringAngleDesiredDeg at all (a steerControlType without it) -> cmd accumulator
  # must no-op, never raise.
  class _NoAngDes:
    pass
  ns = {"self": self_obj, "lac_log": _NoAngDes(), "CS": _FakeCS(0.0, float("nan")), "math": math,
        "DT_CTRL": DT_CTRL}
  exec(compile(accum_src, "<accum>", "exec"), ns)   # must not raise
  assert self_obj._flight_peak_cmdrate_acc == 0.0
  assert self_obj._flight_peak_actrate_sig_acc == 0.0   # NaN sig_rate ignored, not accumulated

  # a genuine finite tick afterward still accumulates normally
  accum_tick(5.0, 0.0, sig_rate=0.0)
  accum_tick(5.0 + 1.0, 0.0, sig_rate=0.0)   # +1 deg over DT_CTRL=0.01s -> 100 deg/s
  ns2 = {"self": self_obj}
  exec(compile(fold_src, "<fold>", "exec"), ns2)
  assert math.isclose(ns2["cmd_rate_pk"], 100.0, rel_tol=1e-6)


# =====================================================================================================
# (2b) The emit-site fallback: prefer the live steeringRateDeg signal, fall back to the steeringAngleDeg
# diff only when the signal never moved this episode. Must be a pure numeric comparison -- no
# carFingerprint/brand check anywhere (the capability-view rule: this file never branches on car
# identity in feature code).
# =====================================================================================================
def test_actrate_fallback_prefers_live_signal_when_alive():
  _, _, _, _, fallback_src = _load_controlsd_rate_pieces()
  ns = {"self": type("S", (), {"_flight_peak_actrate_sig": 22.5, "_flight_peak_actrate_diff": 5.0})()}
  exec(compile(fallback_src, "<fallback>", "exec"), ns)
  assert ns["act_rate_pk"] == 22.5, "a live (non-trivial) steeringRateDeg signal must win"


def test_actrate_fallback_uses_diff_when_signal_dead():
  """The Ford Lightning case: opendbc_repo/opendbc/car/ford/carstate.py never sets steeringRateDeg,
  so it stays pinned at the capnp default 0.0 for the whole episode -- the diff accumulator must be
  used instead."""
  _, _, _, _, fallback_src = _load_controlsd_rate_pieces()
  ns = {"self": type("S", (), {"_flight_peak_actrate_sig": 0.0, "_flight_peak_actrate_diff": 18.0})()}
  exec(compile(fallback_src, "<fallback>", "exec"), ns)
  assert ns["act_rate_pk"] == 18.0, "a dead (0.0) signal must fall back to the diff accumulator"


def test_actrate_fallback_source_selection_has_no_car_branch():
  """Structural check: the fallback expression itself must not reference carFingerprint/brand -- the
  decision is purely 'did this signal move', evaluated identically on every car."""
  _, _, _, _, fallback_src = _load_controlsd_rate_pieces()
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
# (2c) steerEvent pass-through: peakCmdRate/peakActRate appear in _steer_event_step's emitted record,
# sourced straight from the incoming mem-param payload (no re-derivation in ces_pnw.py).
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
  accum_src, fold_src, onset_src, armed_src, fallback_src = _load_controlsd_rate_pieces()
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
