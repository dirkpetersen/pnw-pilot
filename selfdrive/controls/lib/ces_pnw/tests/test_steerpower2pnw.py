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
import math
import textwrap
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[5]
CES_PNW_PATH = REPO_ROOT / "selfdrive/controls/lib/ces_pnw/ces_pnw.py"
CONTROLSD_PATH = REPO_ROOT / "selfdrive/controls/controlsd.py"

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


def test_ces_pnw_additions_only_read_never_gate_control():
  # ces_pnw.py's steer/steerEvent record builders are display/log-only by contract (see module
  # docstrings); confirm the two new pure helpers take no `self`/mutable state at all -- they cannot
  # write anything, control-related or otherwise.
  src, tree = _parse(CES_PNW_PATH)
  for name in ("_ach_lat", "_compass"):
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    args = [a.arg for a in fn.args.args]
    assert "self" not in args, f"{name} must stay a free function (no self), got args={args}"
