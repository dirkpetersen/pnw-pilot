"""curvedblive2pnw: the build report's replay (docs/CURVEDB-V2-BUILD.md s4.2, raise margin 1.25, A = 2.2) re-run
through the CAR's code -- its keying (RowIndex / Polyline / match_at / scan_ahead) and its target rule -- on the
real geometry: each adjudicated episode's own pass stands in for mapd's path, and the table is the leave-one-date-out
export for that episode's date.

THE FIXTURE IS PRIVATE (the owner's driven positions) and is not in this repository. It is built by
tools/curvedb/v2_live_fixture.py into ~/gh/comma/_scratch/curvedb-v2/live/replay_fixture.json.gz, or wherever
CURVEDB_V2_REPLAY_FIXTURE points. Without it this module SKIPS, and says why -- it is not a pass.

`python -m openpilot.selfdrive.controls.lib.ces_pnw.tests.test_curvedblive2pnw_replay` prints the table.
"""
from __future__ import annotations

import gzip
import json
import os

import pytest

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw import curvedb_live as cl

FIXTURE = os.environ.get("CURVEDB_V2_REPLAY_FIXTURE",
                         os.path.expanduser("~/gh/comma/_scratch/curvedb-v2/live/replay_fixture.json.gz"))
MPH, A, REAL, UNWANTED, MIN_RED_MS = cl.MPH, 2.2, 2.5, 2.2, 1.0
LEAD_M = 250.0            # the decision is taken this far before the site (full-decision replay)

pytestmark = pytest.mark.skipif(not os.path.exists(FIXTURE),
                                reason=f"PRIVATE replay fixture absent ({FIXTURE}); build it with tools/curvedb/v2_live_fixture.py")


def _load():
  with gzip.open(FIXTURE, "rt") as f:
    return json.load(f)


def _geo(e):
  idx = cl.RowIndex(e["rows"]["anchors"], e["rows"]["params"])
  poly = cl.Polyline([{"latitude": a, "longitude": b} for a, b in e["path"]])
  return idx, poly


def _outcome(cls, ref, icbm, tgt, a_t):
  raised, gone = tgt is None or tgt > icbm + 0.5, tgt is None or tgt >= ref - MIN_RED_MS
  if cls == "unwanted":
    return "REMOVED" if gone else "reduced" if raised else "unchanged"
  if not raised:
    return "kept"
  return ("LOST" if gone else "WEAKENED") if a_t >= REAL else "raised, < 2.5"


def _bind(v_ego, ref):
  def f(v, d):
    return m.icbm_curve_target(v_ego, ref, 0.0, float("inf"), None, m.icbm_map_eff_scale, 0.0, float("inf"),
                               far_v=v, far_dist=d, track=True)[0]
  return f


def candidate_row(e, a=A):
  """The episode at ICBM's candidate (the site), as the offline replay judged it."""
  idx, poly = _geo(e)
  s_q, _ = poly.project(*e["site"])
  mt = cl.match_at(idx, poly, s_q, *e["site"])
  tgt, d = cl.db_target(e["icbm"], mt.k, a, e["ref"], e["posted"]) if mt.why == "ok" else (e["icbm"], "none")
  if tgt >= e["ref"] - m.ICBM_MIN_DROP_MS:
    tgt = None
  a_t = e["k_truth"] * (e["ref"] if tgt is None else tgt) ** 2
  return mt, tgt, d, a_t, _outcome(e["cls"], e["ref"], e["icbm"], tgt, a_t)


def full_decision(e, a=A):
  """The whole live decision (candidate row + the scan of every row on the path ahead), taken LEAD_M before the
  site at the driver's reference speed: what the truck would have published there."""
  idx, poly = _geo(e)
  s_site, _ = poly.project(*e["site"])
  ego = poly.at(max(s_site - LEAD_M, 0.0))
  db = cl.CurveDbLive(True, data_dir="/nonexistent", read_params=lambda: ({"personalities": {"standard": {
    "map_curve_target_lat_a": a}}}, 1), start=False)
  db.index, db.state = idx, "ok"
  db.poll_a()
  pts = [{"latitude": a, "longitude": b} for a, b in e["path"]]
  out = db.decide(today=e["icbm"], src="far", cands_fn=lambda: ({"far": (e["icbm"], LEAD_M)}, {"far": tuple(e["site"])}),
                  recand_fn=lambda s, p: None, points=pts, plat=ego[0], plon=ego[1], ref=e["ref"], posted=e["posted"],
                  horizon_m=m.ICBM_MAP_HORIZON_M, bind_fn=_bind(e["ref"], e["ref"]), min_drop=m.ICBM_MIN_DROP_MS,
                  way_sel="current", hwy="motorway", allow=True)
  tgt = out[0]
  a_t = e["k_truth"] * (e["ref"] if tgt is None else tgt) ** 2
  return tgt, a_t, _outcome(e["cls"], e["ref"], e["icbm"], tgt, a_t), db.tele()["cdb2Dir"]


def test_the_car_finds_the_rows_the_offline_replay_used():
  for e in _load()["episodes"]:
    mt, *_ = candidate_row(e)
    assert mt.why == "ok", (e["pt"], mt.why)
    # the car keys on the branch CENTER, the offline query on its own end point: MEASURED equal on 6 of 7,
    # 0.93 % apart on C (09-21 21:21:41)
    assert mt.k == pytest.approx(e["k_row_offline"], rel=0.02), e["pt"]


def test_the_build_reports_table_through_the_car_code():
  eps = _load()["episodes"]
  rows = [(e, *candidate_row(e)) for e in eps]
  for e, _mt, _tgt, _d, _a, out in rows:
    assert out == e["outcome_offline"], e["pt"]
  unwanted = [out for e, *_x, out in rows if e["cls"] == "unwanted"]
  assert len(unwanted) == 5 and all(o in ("REMOVED", "reduced") for o in unwanted), unwanted
  real = [(e, a, out) for e, _mt, _t, _d, a, out in rows if e["k_truth"] * e["ref"] ** 2 >= REAL]
  assert len(real) == 2 and not [x for x in real if x[2] in ("LOST", "WEAKENED")]
  assert all(a < REAL for _e, a, _o in real)


def test_the_full_live_decision_never_weakens_a_real_curve_and_never_raises_past_the_row():
  for e in _load()["episodes"]:
    _mt, cand, *_ = candidate_row(e)
    tgt, a_t, out, _dir = full_decision(e)
    assert out not in ("LOST", "WEAKENED"), (e["pt"], out)
    # the scan can only add rows to the minimum: never above the candidate row's own answer
    assert (tgt if tgt is not None else e["ref"]) <= (cand if cand is not None else e["ref"]) + 1e-6, e["pt"]
    if e["k_truth"] * e["ref"] ** 2 >= REAL:
      assert a_t < REAL, e["pt"]
    if e["cls"] == "unwanted":        # MEASURED: H is reduced here (52.7 mph), not removed -- a later row caps it
      assert out in ("REMOVED", "reduced"), (e["pt"], out)


def test_at_the_trucks_A_2_0_the_same_safety_holds():
  """DEVICE-VERIFIED 2026-09-24: mapd's A on the truck is 2 (top-level MapdSettings), not the 2.2 the build assumed.
  MEASURED at 2.0: all 5 unwanted are reduced (H no longer removed: 51.5 at the candidate), I and J kept."""
  for e in _load()["episodes"]:
    _mt, cand, _d, a_t, out = candidate_row(e, 2.0)
    ft, fa, f_out, _dir = full_decision(e, 2.0)
    if e["cls"] == "unwanted":
      assert out == "reduced" and f_out == "reduced", (e["pt"], out, f_out)
    if e["k_truth"] * e["ref"] ** 2 >= REAL:
      assert out == "kept" and f_out == "kept" and a_t < REAL and fa < REAL, e["pt"]


def test_the_tumwater_left_curve_is_added_at_about_69_mph():
  (x,) = _load()["adds"]
  idx, poly = _geo(x)
  s_site, _ = poly.project(*x["site"])
  ms = cl.scan_ahead(idx, poly, max(s_site - 500.0, 0.0), 500.0)
  at_site = min(ms, key=lambda mt: abs(mt.s_anchor - s_site))
  v = cl.v_db(A, at_site.k)
  assert v / MPH == pytest.approx(69.3, abs=0.5)
  v_app = x["v_app_mph"] * MPH
  assert x["k_truth"] * v_app ** 2 >= REAL                       # a real curve at the approach speed
  # it binds through ICBM's own far-candidate envelope when the truck is 150 m out at the approach speed
  b = _bind(v_app, 75 * MPH)(v, 150.0)
  assert b == pytest.approx(v)


def _mph(v):
  return "none" if v is None else f"{v / MPH:.1f}"


def table(a=A) -> str:
  fx = _load()
  out = [f"A = {a} m/s^2", "| PT | site | ref | ICBM | row k (car / offline) | v_db | v2 at the candidate | outcome | full live decision |",
         "|---|---|---|---|---|---|---|---|---|"]
  for e in fx["episodes"]:
    mt, cand, _d, a_t, oc = candidate_row(e, a)
    ft, fa, f_out, fdir = full_decision(e, a)
    cells = [e["pt"], e["cls"], f"{e['ref'] / MPH:.0f}", f"{e['icbm'] / MPH:.1f}",
             f"{mt.k:.6f} / {e['k_row_offline']:.6f}", f"{cl.v_db(a, mt.k) / MPH:.1f}",
             f"{_mph(cand)} (a {a_t:.2f})", oc, f"{_mph(ft)} (a {fa:.2f}) {f_out} [{fdir}]"]
    out.append("| " + " | ".join(cells) + " |")
  (x,) = fx["adds"]
  idx, poly = _geo(x)
  s_site, _ = poly.project(*x["site"])
  ms = cl.scan_ahead(idx, poly, max(s_site - 500.0, 0.0), 500.0)
  at_site = min(ms, key=lambda mt: abs(mt.s_anchor - s_site))
  tight = min(ms, key=lambda mt: cl.v_db(A, mt.k))
  out.append(f"\nadd {x['pt']}: at the site v_db {cl.v_db(a, at_site.k) / MPH:.1f} mph "
             + f"(offline {x['v_db_mph_offline']:.1f}); tightest row on the 500 m before it "
             + f"{cl.v_db(a, tight.k) / MPH:.1f} mph; approach {x['v_app_mph']:.1f}; "
             + f"truth a at the site's v_db {x['k_truth'] * cl.v_db(a, at_site.k) ** 2:.2f}")
  return "\n".join(out)


if __name__ == "__main__":
  print(table(2.2))
  print()
  print(table(2.0))


def test_at_A_2_0_the_left_curve_is_added_at_about_66_mph():
  (x,) = _load()["adds"]
  idx, poly = _geo(x)
  s_site, _ = poly.project(*x["site"])
  ms = cl.scan_ahead(idx, poly, max(s_site - 500.0, 0.0), 500.0)
  at_site = min(ms, key=lambda mt: abs(mt.s_anchor - s_site))
  assert cl.v_db(2.0, at_site.k) / MPH == pytest.approx(66.1, abs=0.5)
