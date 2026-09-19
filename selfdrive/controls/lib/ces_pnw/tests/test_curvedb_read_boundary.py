"""THE READ BOUNDARY. curvedbshadow2pnw's entire safety argument, enforced instead of asserted.

The claim being defended is exactly one sentence: **no control path reads the curve database's
answer.** That claim is what separates "a harmless observer writing a log line" from "an unvalidated,
self-editing database influencing the brakes", and CURVEDB2PNW.md section 12 is explicit that
nothing in this database has earned any authority -- the offline section 7 replay returned NO RESULT
over 170 ICBM episodes.

A comment saying so is worth nothing. This repository has been bitten by exactly that, repeatedly:
`icbmK` shipped with "TELEMETRY ONLY" above it and a wired check below it that had to be reverted;
`visK` was logged for months and never computed; `icbm_map_sanity` is deliberately unwired and the
only thing keeping it that way is prose. Every one of those was true when written.

FIVE LAYERS, each of which would catch a different way of breaking it:

  L1  ces_pnw.py, parsed with `ast`: `_cdb` and the telemetry builder may appear only in the three
      syntactic positions that cannot flow anywhere -- the constructor assignment, a discarded
      `tick()` statement, and a `**` splat into the record dict.
  L2  curvedb_shadow.py, parsed with `ast`: it cannot reach control even if it wanted to (no
      selfdrive/system/cereal/opendbc imports), and it never branches on a fingerprint.
  L3  the whole tree: nobody else imports it, and nobody in `selfdrive/`/`system/`/`common/` imports
      `tools.curvedb` behind its back.
  L4  the record dict itself never escapes: `_event_record`'s return goes only to `_append_event`.
  L5  EMPIRICAL. The real controller is run twice at 100 Hz over an identical scenario with two
      wildly different databases -- one empty, one holding a row that would cancel every slowdown --
      and every published `IcbmTarget`, every CES decision and every non-`cdb*` record field must be
      byte-identical. L1-L4 pin the shape; L5 pins the behaviour, and catches a hole in L1's
      allow-list that a later edit sneaks through.

If a future change genuinely wants the database to act, these tests are what it has to argue with --
which is the point. Deleting one of them is the change under review, not a cleanup.
"""
from __future__ import annotations

import ast
import copy
import json
import pathlib
import types

import pytest

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
from openpilot.selfdrive.controls.lib.ces_pnw import curvedb_shadow as cs
from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvelead2pnw import FakeCP, LAT0, LON0, _model, _scene
from openpilot.selfdrive.controls.lib.ces_pnw.tests.test_ces_mode_read_failure_logged import LIGHTNING, MPH, _P
from openpilot.tools.curvedb.store import PROVISIONAL_PARAMS, CurveDB, Observation

NS = types.SimpleNamespace

CES_PNW_PATH = pathlib.Path(m.__file__).resolve()
SHADOW_PATH = pathlib.Path(cs.__file__).resolve()
def _repo_root() -> pathlib.Path:
  """Walk up to the checkout. Counting `.parents[n]` got this wrong once already and the test then
  scanned an EMPTY file list, which passed -- a boundary check whose corpus is empty is not a check
  (`grep | head && echo OK` in another costume)."""
  for cur in CES_PNW_PATH.parents:
    if (cur / "selfdrive").is_dir() and (cur / "tools").is_dir():
      return cur
  raise AssertionError(f"no checkout root above {CES_PNW_PATH}")


REPO = _repo_root()


def _tree(path: pathlib.Path) -> ast.AST:
  return ast.parse(path.read_text(), filename=str(path))


def _parents(tree: ast.AST) -> dict[int, ast.AST]:
  """id(node) -> parent. `ast` gives no parent links and every structural claim below needs them."""
  out: dict[int, ast.AST] = {}
  for node in ast.walk(tree):
    for child in ast.iter_child_nodes(node):
      out[id(child)] = node
  return out


def _enclosing_def(node, parents) -> str:
  cur = parents.get(id(node))
  while cur is not None:
    if isinstance(cur, ast.FunctionDef | ast.AsyncFunctionDef):
      return cur.name
    cur = parents.get(id(cur))
  return "<module>"


# =====================================================================================================
# L1 -- ces_pnw.py: the shadow may only be touched in positions that cannot flow into control
# =====================================================================================================
class TestL1CesPnwSyntacticPositions:

  def test_cdb_is_only_constructed_and_ticked(self):
    """Every `self._cdb` in ces_pnw.py is one of exactly two things, and there is no third.

    Mutation this kills: `target = min(target, self._cdb.best())`, `if self._cdb.match(...)`, or any
    other read of the shadow object from the ICBM decision. Parsed, not grepped, so the prose in
    this file's docstrings cannot satisfy it."""
    tree = _tree(CES_PNW_PATH)
    parents = _parents(tree)
    hits = [n for n in ast.walk(tree) if isinstance(n, ast.Attribute) and n.attr == "_cdb"]
    assert hits, "self._cdb vanished from ces_pnw.py -- the shadow is no longer wired at all"
    n_ctor = n_tick = 0
    for node in hits:
      assert isinstance(node.value, ast.Name) and node.value.id == "self", ast.dump(node)
      parent = parents[id(node)]
      # (a) the constructor assignment: self._cdb = CurveDBShadow(...)
      if isinstance(parent, ast.Assign) and node in parent.targets:
        assert isinstance(node.ctx, ast.Store)
        assert isinstance(parent.value, ast.Call) and getattr(parent.value.func, "id", None) == "CurveDBShadow", \
          f"self._cdb is assigned something other than a CurveDBShadow: {ast.unparse(parent.value)}"
        assert _enclosing_def(node, parents) == "__init__"
        n_ctor += 1
        continue
      # (b) the 100 Hz feed: self._cdb.tick(...) -- and nothing else
      assert isinstance(parent, ast.Attribute), \
        f"self._cdb used bare (not as an attribute access) at line {node.lineno}: {ast.unparse(parent)}"
      assert parent.attr == "tick", \
        f"ces_pnw.py calls self._cdb.{parent.attr} -- only `tick` (which returns nothing) is allowed"
      call = parents[id(parent)]
      assert isinstance(call, ast.Call) and call.func is parent, ast.unparse(call)
      stmt = parents[id(call)]
      assert isinstance(stmt, ast.Expr), (
        f"self._cdb.tick(...) at line {node.lineno} is used as an EXPRESSION " +
        f"({ast.unparse(stmt)}) -- its value must be discarded")
      n_tick += 1
    assert (n_ctor, n_tick) == (1, 1), f"expected exactly one ctor and one tick, got {n_ctor}/{n_tick}"

  def test_the_telemetry_builder_is_only_ever_splatted_into_a_dict_literal(self):
    """`curvedb_tele(...)`'s return may be bound to NO name. It goes straight into `{**...}`.

    Mutation this kills: `cdb = curvedb_tele(...)` followed by any use of `cdb`. A `**` splat inside
    a dict display is the one position from which a value provably cannot be read again -- it has no
    name, and the dict it lands in is itself pinned by L4."""
    tree = _tree(CES_PNW_PATH)
    parents = _parents(tree)
    calls = [n for n in ast.walk(tree)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == "curvedb_tele"]
    assert len(calls) == 1, f"expected exactly one curvedb_tele call site, found {len(calls)}"
    call = calls[0]
    assert _enclosing_def(call, parents) == "_event_record"
    holder = parents[id(call)]
    assert isinstance(holder, ast.Dict), \
      f"curvedb_tele(...) is not inside a dict literal: {ast.unparse(holder)}"
    idx = [i for i, v in enumerate(holder.values) if v is call]
    assert len(idx) == 1 and holder.keys[idx[0]] is None, \
      "curvedb_tele(...) must be a ** splat (a None key in the Dict), not a keyed value"

  def test_no_shadow_field_name_is_ever_read_back_in_ces_pnw(self):
    """No `cdb*` key is read anywhere in ces_pnw.py -- not as a subscript, not as a comparison.

    Mutation this kills: `if rec["cdbWould"] < target: target = rec["cdbWould"]` further down the
    record builder, which L1's other two tests would not see because it never names `_cdb`."""
    tree = _tree(CES_PNW_PATH)
    keys = set(cs.CURVEDB_TELE_KEYS)
    for node in ast.walk(tree):
      if isinstance(node, ast.Constant) and isinstance(node.value, str) and node.value in keys:
        pytest.fail(f'ces_pnw.py names the shadow field "{node.value}" at line {node.lineno} -- ' +
                    "the record is written, never read")
      if isinstance(node, ast.Attribute) and node.attr in keys:
        pytest.fail(f"ces_pnw.py reads .{node.attr} at line {node.lineno}")

  def test_the_shadow_is_not_called_from_the_icbm_decision_at_all(self):
    """`_icbm_step` -- the function that computes and publishes `IcbmTarget` -- never mentions it.

    This is stronger than "the value is unused": the ICBM decision cannot be made to depend on the
    database by an edit that only moves a line, because the object is not in scope there."""
    tree = _tree(CES_PNW_PATH)
    fn = next(n for n in ast.walk(tree)
              if isinstance(n, ast.FunctionDef) and n.name == "_icbm_step")
    src = ast.unparse(fn)
    for name in ("_cdb", "curvedb", "cdb_pt", "CurveDBShadow"):
      assert name not in src, f"_icbm_step mentions {name!r}"


# =====================================================================================================
# L2 -- curvedb_shadow.py cannot reach control even if it tried
# =====================================================================================================
class TestL2ShadowModuleIsSealed:

  ALLOWED_OPENPILOT_IMPORTS = {"openpilot.common.swaglog", "openpilot.tools.curvedb.store"}

  def test_it_imports_nothing_from_the_control_stack(self):
    """The only openpilot imports are swaglog (to log) and the shared C1 store (to compute).

    A module that cannot import `cereal`, `opendbc`, `selfdrive` or `system` cannot publish a
    mem-param, send a message, or touch an actuator -- whatever its author later intends. It is also
    what keeps section 8.3's portability claim true: `cereal` is NOT import-stable across 0.11.1 and
    0.11.2, so any module reaching for it is version-locked."""
    tree = _tree(SHADOW_PATH)
    for node in ast.walk(tree):
      mods = []
      if isinstance(node, ast.Import):
        mods = [a.name for a in node.names]
      elif isinstance(node, ast.ImportFrom):
        mods = [node.module or ""]
      for mod in mods:
        root = mod.split(".")[0]
        if root in ("cereal", "opendbc", "msgq", "panda"):
          pytest.fail(f"curvedb_shadow imports {mod!r}")
        if root == "openpilot":
          assert mod in self.ALLOWED_OPENPILOT_IMPORTS, \
            f"curvedb_shadow imports {mod!r}; allowed: {sorted(self.ALLOWED_OPENPILOT_IMPORTS)}"

  @staticmethod
  def _docstring_ids(tree) -> set[int]:
    """Docstrings are `ast.Constant` nodes too, and this module's own prose explains WHY it does not
    branch on a fingerprint -- so a naive string scan fails on the explanation of the rule it is
    enforcing."""
    out = set()
    for node in ast.walk(tree):
      if isinstance(node, ast.Module | ast.ClassDef | ast.FunctionDef | ast.AsyncFunctionDef):
        body = getattr(node, "body", None) or []
        if body and isinstance(body[0], ast.Expr) and isinstance(body[0].value, ast.Constant) \
           and isinstance(body[0].value.value, str):
          out.add(id(body[0].value))
    return out

  def test_it_never_branches_on_a_car_fingerprint(self):
    """The capability-view rule (driver directive 2026-07-11): feature code reads a capability, it
    never compares a fingerprint string.

    `self.platform` is used ONLY as an opaque dict key into the envelope table and as a provenance
    string on an Observation -- which section 8.2 explicitly asks for ("an opaque platform string")
    and which the offline replay must share or the replay validates nothing. A comparison would be
    the thing the rule forbids."""
    tree = _tree(SHADOW_PATH)
    docs = self._docstring_ids(tree)
    for node in ast.walk(tree):
      if isinstance(node, ast.Compare):
        src = ast.unparse(node)
        if "platform" in src:
          pytest.fail(f"curvedb_shadow compares the platform at line {node.lineno}: {src}")
      if isinstance(node, ast.Constant) and isinstance(node.value, str) and id(node) not in docs:
        for marker in ("FORD_", "TESLA_", "carFingerprint"):
          assert marker not in node.value, \
            f"curvedb_shadow hardcodes {node.value!r} at line {node.lineno}"

  def test_tick_returns_nothing_ever(self):
    """`tick` is the 100 Hz call site's only contact with the shadow, and it must be incapable of
    handing anything back -- so the discarded-statement pin in L1 can never become a lie."""
    tree = _tree(SHADOW_PATH)
    fn = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "tick")
    for node in ast.walk(fn):
      if isinstance(node, ast.Return):
        assert node.value is None, f"tick returns {ast.unparse(node.value)} at line {node.lineno}"
    assert cs.CurveDBShadow.tick(cs.CurveDBShadow.__new__(cs.CurveDBShadow),
                                 None, None, None, 0, None, False) is None

  def test_the_module_never_writes_a_param_or_a_mem_param(self):
    """No `put`, `put_nonblocking` or `put_bool` anywhere: the mem-param bus is how every acting pnw
    brain reaches its executor (`IcbmTarget`, `SpeedAdjustTarget`), so a shadow that cannot write one
    cannot become a brain by accident."""
    tree = _tree(SHADOW_PATH)
    called = {n.func.attr for n in ast.walk(tree)
              if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
    for forbidden in ("put", "put_nonblocking", "put_bool", "send", "publish"):
      assert forbidden not in called, f"curvedb_shadow calls .{forbidden}()"


# =====================================================================================================
# L3 -- nobody else touches it
# =====================================================================================================
def _py_files():
  for sub in ("selfdrive", "system", "common", "cereal"):
    root = REPO / sub
    if root.is_dir():
      yield from root.rglob("*.py")


class TestL3NobodyElseImportsIt:

  def test_only_ces_pnw_and_tests_import_the_shadow(self):
    importers = set()
    for path in _py_files():
      try:
        text = path.read_text()
      except (OSError, UnicodeDecodeError):
        continue
      if "curvedb_shadow" in text:
        importers.add(path.relative_to(REPO).as_posix())
    expected = {
      "selfdrive/controls/lib/ces_pnw/ces_pnw.py",
      "selfdrive/controls/lib/ces_pnw/curvedb_shadow.py",
      "selfdrive/controls/lib/ces_pnw/tests/test_curvedb_read_boundary.py",
      "selfdrive/controls/lib/ces_pnw/tests/test_curvedbshadow2pnw.py",
    }
    assert importers == expected, f"unexpected references to curvedb_shadow: {importers ^ expected}"

  def test_no_control_path_module_reaches_into_tools_curvedb(self):
    """`tools/curvedb` is the offline half. Exactly one control-path module may import it (the
    shadow, and only `store` -- section 8.3's C1 core), because that shared implementation is what
    makes the section 7 replay mean anything. Anything else importing `ingest`/`replay` would drag
    the offline pipeline onto the car."""
    offenders = {}
    for path in _py_files():
      if "/tests/" in path.as_posix() or path.name.startswith("test_"):
        continue
      try:
        text = path.read_text()
      except (OSError, UnicodeDecodeError):
        continue
      if "tools.curvedb" in text:
        offenders[path.relative_to(REPO).as_posix()] = [
          ln for ln in text.splitlines() if "tools.curvedb" in ln]
    assert set(offenders) == {"selfdrive/controls/lib/ces_pnw/curvedb_shadow.py"}, offenders
    for line in offenders["selfdrive/controls/lib/ces_pnw/curvedb_shadow.py"]:
      assert "tools.curvedb.store" in line, f"the car imports more than the C1 core: {line}"


# =====================================================================================================
# L4 -- the record dict itself never escapes
# =====================================================================================================
class TestL4TheRecordGoesNowhereButTheLog:

  def test_event_record_results_are_only_ever_appended(self):
    """Every `self._event_record(...)` call is an argument to `self._append_event(...)`.

    Without this, L1's "the fragment is splatted into `rec`" argument is incomplete: a `rec` that
    someone later returned, cached or inspected would reopen the path the splat closed."""
    tree = _tree(CES_PNW_PATH)
    parents = _parents(tree)
    calls = [n for n in ast.walk(tree)
             if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
             and n.func.attr == "_event_record"]
    assert calls, "_event_record is never called -- the record builder is unwired"
    for call in calls:
      parent = parents[id(call)]
      # (a) the direct form: self._append_event(self._event_record(...))
      if isinstance(parent, ast.Call) and isinstance(parent.func, ast.Attribute) \
         and parent.func.attr == "_append_event":
        continue
      # (b) the adopt form: `rec = self._event_record(...)`, then `rec["from"], rec["to"] = ...`,
      #     then `self._append_event(rec)`. Permitted -- but every read of that name is checked:
      #     a subscript WRITE, or the append. Nothing may read a value back out of the record.
      assert isinstance(parent, ast.Assign) and len(parent.targets) == 1 \
        and isinstance(parent.targets[0], ast.Name), \
        f"_event_record result at line {call.lineno} goes to {ast.unparse(parent)}"
      name = parent.targets[0].id
      fn = next(n for n in ast.walk(tree)
                if isinstance(n, ast.FunctionDef) and _enclosing_def(call, parents) == n.name)
      fparents = _parents(fn)
      for use in ast.walk(fn):
        if not (isinstance(use, ast.Name) and use.id == name and isinstance(use.ctx, ast.Load)):
          continue
        up = fparents[id(use)]
        if isinstance(up, ast.Call) and isinstance(up.func, ast.Attribute) \
           and up.func.attr == "_append_event":
          continue
        if isinstance(up, ast.Subscript) and isinstance(up.ctx, ast.Store):
          continue
        pytest.fail(f"{name!r} (an _event_record result) is READ at line {use.lineno}: " +
                    f"{ast.unparse(up)}")


# =====================================================================================================
# L5 -- EMPIRICAL. Two wildly different databases, byte-identical control.
# =====================================================================================================
def _row_that_would_cancel_everything(tmp_path):
  """A database whose single row is as close to "cancel every slowdown" as the store permits:
  a nearly-straight k (so `sqrt(a_lat/k)` is enormous) at the exact candidate the scenario names,
  on a motorway, with plenty of passes across plenty of dates."""
  obs = []
  for i in range(6):
    obs.append(Observation(
      date=f"2026-09-{10 + i:02d}", t=1.0e9 + i, car=LIGHTNING, drive_id=f"d{i}",
      site_lat=LAT0 + 300.0 / 111320.0, site_lon=LON0, bearing_deg=0.0,
      k=1e-3, kind="up", estimator="kPeak100", site_src="logged", source="test",
      posted_ms=60 * MPH, highway_class="motorway", n_ticks=20,
      dq_state="clean", dq_src="rollup100"))
  return obs


def _run(monkeypatch, tmp_path, observations, ticks=400):
  """Drive the REAL CESController at 100 Hz towards a map curve, with `observations` pre-loaded into
  the shadow's store. Returns (IcbmTarget publishes, CES decisions, records)."""
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "absent.json"))
  monkeypatch.setattr(pv, "RAIN_CONFIG_PATH", str(tmp_path / "absent-rain.json"))
  tmp_path.mkdir(parents=True, exist_ok=True)
  obs_path = tmp_path / "obs.jsonl"
  obs_path.write_text("".join(json.dumps(vars(o)) + "\n" for o in observations))
  monkeypatch.setattr(cs, "OBS_PATH", str(obs_path))
  monkeypatch.setattr(cs, "CONFIG_PATH", str(tmp_path / "absent-curvedb.json"))

  clock = [5000.0]
  ns = types.SimpleNamespace(monotonic=lambda: clock[0], time=lambda: clock[0])
  for mod in (C, m):
    monkeypatch.setattr(mod, "time", ns)
  C._ces_mode_hold_st.clear()

  class Mem:
    def __init__(self):
      self.puts = []

    def get(self, k, return_default=False):
      if k == "MapTargetVelocities":
        return _scene(300.0, 180.0, 12.0)
      if k == "LastGPSPosition":
        return json.dumps({"latitude": LAT0, "longitude": LON0, "bearing": 0.0, "src": "device",
                           "ts": clock[0], "fix_ts": clock[0] - 0.3})
      if k == "MapSpeedLimit":
        return str(60 * MPH)
      return None

    def put_nonblocking(self, k, v):
      self.puts.append((k, copy.deepcopy(v)))

  params = _P(clock, mode=lambda t: 2, extra={"CESButtonState": "0"})
  c = m.CESController(FakeCP(LIGHTNING, "ford", False), params=params)
  assert c._cdb.wait_loaded(), "the shadow's boot load never finished"
  c.mem_params = Mem()
  recs = []
  c._event_log_ok = True
  c._append_event = lambda rec: recs.append(copy.deepcopy(rec))
  decisions = []
  v_ego, stock = 27.0, 60 * MPH
  for i in range(ticks):
    clock[0] = 5000.0 + (i + 1) * 0.01
    orz, vx, px, ts = _model(v_ego, 300.0, 180.0)
    model = NS(orientationRate=NS(z=orz, t=ts), velocity=NS(x=vx), position=NS(x=px),
               action=NS(shouldStop=False), meta=NS(laneChangeState="off"))
    sm = {"radarState": NS(leadOne=NS(status=False, vLead=0.0, dRel=0.0, aLeadK=0.0, vLeadK=0.0)),
          "modelV2": model, "carControl": NS(orientationNED=[0.0, 0.0, 0.0]),
          "livePose": NS(angularVelocityDevice=NS(x=0.0, y=0.0, z=0.02, valid=True)),
          "controlsState": NS(desiredCurvature=0.004,
                              lateralControlState=NS(which=lambda: "angleState",
                                                     angleState=NS(saturated=False)))}
    cstate = NS(vEgo=v_ego, aEgo=0.0, gasPressed=False, brakePressed=False,
                leftBlinker=False, rightBlinker=False, vCruise=stock * 3.6, standstill=False,
                steeringAngleDeg=0.0, steeringPressed=False, leftBlindspot=False,
                rightBlindspot=False, cruiseState=NS(speed=stock, enabled=True),
                yawRate=0.05, steeringTorque=0.0)
    decisions.append(c.experimental_request(cstate, sm))
  icbm = [v for k, v in c.mem_params.puts if k == "IcbmTarget"]
  return icbm, decisions, recs


class TestL5TheDatabaseChangesNothing:

  def test_an_extreme_database_leaves_every_control_output_byte_identical(self, monkeypatch, tmp_path):
    """The whole safety argument, measured rather than argued.

    Run A has NO database. Run B has a row at the exact candidate saying the road is nearly straight
    (k = 1e-3 -> sqrt(2.5/1e-3) = 50 m/s), granted on 6 passes over 6 dates, on a motorway, under the
    posted limit -- i.e. the strongest possible reason to cancel. Every `IcbmTarget` publish, every
    CES decision and every non-`cdb*` record field must be identical. If a later change wires the
    database into the ICBM decision, run B's targets will rise and this fails."""
    a_icbm, a_dec, a_rec = _run(monkeypatch, tmp_path / "a", [])
    b_icbm, b_dec, b_rec = _run(monkeypatch, tmp_path / "b", _row_that_would_cancel_everything(tmp_path))

    # The scenario must actually exercise ICBM, or "identical" is a statement about nothing.
    assert any(p.get("target") is not None for p in a_icbm), \
      "no ICBM target was ever published -- this test would pass vacuously"
    assert a_icbm == b_icbm, "the curve database moved the published IcbmTarget"
    assert a_dec == b_dec, "the curve database moved the CES decision"

    # ...and the database WAS loaded and DID say something different, or the comparison is vacuous.
    assert any(r.get("cdbRow") for r in b_rec), "run B never matched a row; the test proves nothing"
    assert not any(r.get("cdbRow") for r in a_rec), "run A matched a row with an empty database"

    assert len(a_rec) == len(b_rec)
    keys = set(cs.CURVEDB_TELE_KEYS)
    for ra, rb in zip(a_rec, b_rec, strict=True):
      assert {k: v for k, v in ra.items() if k not in keys} == \
             {k: v for k, v in rb.items() if k not in keys}, \
        "a non-cdb* record field differs between the two databases"

  def test_a_shadow_that_raises_on_every_call_changes_nothing(self, monkeypatch, tmp_path):
    """Rule 2's other half: the shadow failing must cost telemetry, never control.

    Mutation this kills: moving the `record()` call out of its own try/except, or letting a shadow
    exception escape into `_event_record` (which `_append_event` would then never receive)."""
    a_icbm, a_dec, a_rec = _run(monkeypatch, tmp_path / "ok", [])

    boom = tmp_path / "boom"

    def explode(self, **kw):
      raise RuntimeError("shadow is on fire")

    monkeypatch.setattr(cs.CurveDBShadow, "_record", explode)
    b_icbm, b_dec, b_rec = _run(monkeypatch, boom, [])
    assert a_icbm == b_icbm and a_dec == b_dec
    assert len(a_rec) == len(b_rec)
    assert all(r["cdbOn"] == "err" and r["cdbErr"] > 0 for r in b_rec), \
      "a failing shadow must SAY so in every record, not go quiet"
    keys = set(cs.CURVEDB_TELE_KEYS)
    for ra, rb in zip(a_rec, b_rec, strict=True):
      assert {k: v for k, v in ra.items() if k not in keys} == \
             {k: v for k, v in rb.items() if k not in keys}


# =====================================================================================================
# the store this shares with the replay must be the SAME store
# =====================================================================================================
def test_the_car_and_the_replay_share_one_matcher():
  """If the car ran a second implementation of the matcher, the update rules or the authority gate,
  the section 7 replay would have validated nothing. `store.py` is imported, never copied."""
  from openpilot.tools.curvedb import store as offline_store
  assert cs.CurveDB is offline_store.CurveDB
  assert cs.authority is offline_store.authority
  assert cs.cancel_target is offline_store.cancel_target
  assert cs.PROVISIONAL_PARAMS is offline_store.PROVISIONAL_PARAMS
  # ...and the matcher the shadow uses is the one `build` used, not a re-grouping under other params
  db = CurveDB.build([], PROVISIONAL_PARAMS)
  assert db.match.__func__ is offline_store.CurveDB.match
