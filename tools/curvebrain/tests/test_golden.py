"""tools/curvebrain/golden.py -- the golden harness's own rules, on synthetic data (no real positions).

The self-test the design asks for (s4): two runs at the same commit are byte-identical -- here across two PROCESSES with
different hash seeds -- and a 1e-9 change to ICBM_MARGIN_M is detected. Corpus 3 runs on the private fixture when it
is present and SKIPS, saying why, when it is not."""
import json
import math
import os
import subprocess
import sys

import pytest
import zstandard

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.tools.curvebrain import golden as g

SMALL = 1500          # fuzz frames for the fast tests (~5 scenarios, DB on and off, a far candidate that binds)
LIGHT = "FORD_F_150_LIGHTNING_MK1"


def _fuzz(out=None, n=SMALL, seed=g.FUZZ_SEED):
  return g.run(out, ("c4_fuzz",), {"c4_fuzz": {"n_frames": n, "seed": seed}})


# ---------------------------------------------------------------------------------------------------------------------
# the self-test
# ---------------------------------------------------------------------------------------------------------------------
def test_two_runs_in_two_processes_are_byte_identical(tmp_path):
  shas, lines, cov, _notes, missing = _fuzz(str(tmp_path))
  assert not missing and lines["c4_fuzz"] > SMALL                     # + one seg line per scenario
  code = ("import sys; from openpilot.tools.curvebrain import golden as g; " +
          f"s = g.run(None, ('c4_fuzz',), {{'c4_fuzz': {{'n_frames': {SMALL}}}}})[0]; print(s['c4_fuzz'])")
  env = {**os.environ, "PYTHONHASHSEED": "12345"}
  out = subprocess.run([sys.executable, "-c", code], env=env, check=True, capture_output=True, text=True).stdout
  assert out.strip().splitlines()[-1] == shas["c4_fuzz"]
  # and the sha is over exactly the bytes on disk
  import hashlib
  with open(tmp_path / "c4_fuzz.jsonl.zst", "rb") as fh:
    raw = zstandard.ZstdDecompressor().stream_reader(fh).read()
  assert hashlib.sha256(raw).hexdigest() == shas["c4_fuzz"]
  assert cov.acct["c4_fuzz"]["ticks"] == SMALL


def test_a_1e9_change_to_ICBM_MARGIN_M_is_detected(tmp_path):
  """ICBM_MARGIN_M moves a value CONTINUOUSLY in one place only: gpsdrgate2pnw's stale-GPS hold distance
  (`_icbm_stale_hold["left"]`). Everywhere else it is a threshold (the brake envelope, apex passage) or picks a winner
  without being part of its value (icbm_far_map_candidate's cap), which a 1e-9 change never flips. So the slice must
  contain a stale hold -- asserted, not assumed. MEASURED on the full goldens: 406 of 100,340 c4 lines differ, all in
  st.ctl._icbm_stale_hold.left; c1-c3 have no stale hold and do not see it."""
  n = 6000
  base, _lines, cov, _notes, _missing = _fuzz(str(tmp_path / "a"), n=n)
  assert cov.hits["c4_fuzz"]["icbmSrc"]["gpsHold"] > 0, "no stale-GPS hold in the slice: this test would prove nothing"
  base = base["c4_fuzz"]
  pats = g.mutate("ICBM_MARGIN_M=1e-9")
  for p in pats:
    p.start()
  try:
    assert m.ICBM_MARGIN_M == 30.0 + 1e-9
    mutated = _fuzz(str(tmp_path / "b"), n=n)[0]["c4_fuzz"]
  finally:
    for p in pats:
      p.stop()
  assert m.ICBM_MARGIN_M == 30.0
  assert mutated != base
  n, bad, first = g.compare(str(tmp_path / "a" / "c4_fuzz.jsonl.zst"), str(tmp_path / "b" / "c4_fuzz.jsonl.zst"))
  assert bad > 0 and first
  # the published fields are rounded to 2 dp: the difference is found in the UNROUNDED state, which is why it is kept
  assert any(k == "st.ctl._icbm_stale_hold.left" for _ln, keys, _msg in first for k in keys)


def test_mutate_names_nothing_it_cannot_find():
  with pytest.raises(g.HarnessError):
    g.mutate("NO_SUCH_CONSTANT_ANYWHERE=1")


# ---------------------------------------------------------------------------------------------------------------------
# canonical lines
# ---------------------------------------------------------------------------------------------------------------------
def test_canonical_form_is_sorted_compact_and_keeps_every_bit():
  assert g.canon({"b": 0.1 + 0.2, "a": float("nan"), "c": (1, 2), "d": float("inf")}) == \
    '{"a":NaN,"b":0.30000000000000004,"c":[1,2],"d":Infinity}'
  assert g.canon({"x": 30.0 + 1e-9}) != g.canon({"x": 30.0})


def test_an_unknown_type_is_an_error_never_a_repr():
  class Obj:
    pass
  with pytest.raises(TypeError, match="not serialisable"):
    g.canon({"o": Obj()})


def test_the_state_snapshot_keeps_values_and_drops_objects():
  class Box:
    pass
  b = Box()
  b._icbm_a, b._icbm_obj, b._icbm_d = 1.25, object(), {"left": 3.0}
  b._icbm_ep = m.IcbmEpisode()
  b.other = 5
  st = g.state_of(b)
  assert st["ctl"] == {"_icbm_a": 1.25, "_icbm_d": {"left": 3.0}}
  assert st["ep"]["phase"] == "idle" and "_last_cap_dist" in st["ep"]


def test_the_scrub_removes_the_runs_temporary_directory():
  s1, s2 = g.Sink(scrub="/tmp/curvebrain-golden-aaa"), g.Sink(scrub="/tmp/curvebrain-golden-bbb")
  s1.write({"cdb2Err": "missing /tmp/curvebrain-golden-aaa/fz-3/manifest.json"})
  s2.write({"cdb2Err": "missing /tmp/curvebrain-golden-bbb/fz-3/manifest.json"})
  assert s1.close() == s2.close()


# ---------------------------------------------------------------------------------------------------------------------
# Rule 2: a swallowed failure is in the golden, a missing input is never an empty corpus
# ---------------------------------------------------------------------------------------------------------------------
def test_a_step_that_fails_inside_its_own_except_is_visible(monkeypatch):
  def boom(*a, **kw):
    raise RuntimeError("injected")
  monkeypatch.setattr(m, "icbm_vision_apex", boom)
  _shas, _lines, cov, _notes, _missing = _fuzz(n=200)
  acct = cov.acct["c4_fuzz"]
  assert acct["no IcbmTarget publish (throttle or a failed step)"] == 200
  assert any(k.startswith("log exception: icbm: _icbm_step FAILED") for k in acct)


def test_corpus_inputs_that_are_absent_are_reported_not_empty(tmp_path):
  with g.Env() as env:
    with pytest.raises(g.MissingInput):
      g.corpus1(env, g.Sink(), g.Coverage(), path=str(tmp_path / "none.jsonl"))
    with pytest.raises(g.MissingInput):
      g.corpus2(env, g.Sink(), g.Coverage(), d=str(tmp_path))
    with pytest.raises(g.MissingInput):
      g.corpus3(env, g.Sink(), g.Coverage(), fixture=str(tmp_path / "none.json.gz"))
  # through run(): listed as missing, said in the notes, and no half-written golden left behind
  shas, _lines, _cov, notes, missing = g.run(str(tmp_path), ("c3_fixture",),
                                             {"c3_fixture": {"fixture": str(tmp_path / "none.json.gz")}})
  assert "c3_fixture" not in shas and len(missing) == 1 and any("SKIPPED" in n for n in notes)
  assert not (tmp_path / "c3_fixture.jsonl.zst").exists()


def test_a_cache_with_no_usable_tick_is_a_harness_failure(tmp_path):
  p = tmp_path / "ticks.jsonl"
  p.write_text("not json\n{\"t\": 1.0}\n")
  with g.Env() as env, pytest.raises(g.HarnessError, match="NO ticks"):
    g.corpus1(env, g.Sink(), g.Coverage(), path=str(p))


# ---------------------------------------------------------------------------------------------------------------------
# corpora 1 and 2 on synthetic records (a position in the Gulf of Guinea: nobody drives there)
# ---------------------------------------------------------------------------------------------------------------------
def _tick(t, **kw):
  r = {"t": t, "ev": "tick", "car": LIGHT, "vEgo": 30.0, "stockSet": 31.3, "stockOn": True, "mapV": 20.0,
       "mapDist": 150.0 - 30.0 * (t - 1000.0), "lat": 0.5 + 0.00027 * (t - 1000.0), "lon": 0.5, "bearing": 0.0,
       "spdLim": 29.0, "hwyClass": "motorway", "waySel": "current", "icbmK": 0.004, "icbmKD": 150.0, "icbmKN": 6,
       "icbmKAhead": True, "_src": "drives/2099-01-01/x/ces_events.jsonl", "_hasK": True}
  r.update(kw)
  return r


def test_corpus1_replays_every_logged_tick_and_segments_on_gaps(tmp_path):
  p = tmp_path / "ticks.jsonl"
  ticks = [_tick(1000.0 + i) for i in range(5)] + [_tick(1100.0 + i) for i in range(3)]
  p.write_text("".join(json.dumps(r) + "\n" for r in ticks))
  sink, cov = g.Sink(), g.Coverage()
  with g.Env() as env:
    g.corpus1(env, sink, cov, path=str(p))
  assert cov.acct["c1_cache"]["ticks"] == 8
  assert sink.n == 8 + 2                                                  # two segments (a 95 s gap), a seg line each
  assert cov.hits["c1_cache"]["icbmSrc"]["map"] > 0                       # the logged near candidate binds


def test_the_sig_carries_only_what_was_logged():
  sig = g.sig_from_record({"vEgo": 20.0, "stockSet": 25.0, "mapV": 0.0, "mapDist": 0.0})
  assert sig["map_target_dist"] == math.inf and sig["v_set"] == 25.0
  assert "curve_lat_accel_vision" not in sig and "time_to_curve" not in sig and "has_lead" not in sig
  sig = g.sig_from_record({"vEgo": 20.0, "vSet": 26.0, "stockSet": 25.0, "visLat": 2.1, "visTtc": 4.0, "lead": True,
                           "vLead": 18.0})
  assert sig["v_set"] == 26.0 and sig["curve_lat_accel_vision"] == 2.1 and sig["has_lead"] is True


def _zst(path, recs):
  with open(path, "wb") as fh:
    fh.write(zstandard.ZstdCompressor().compress("".join(
      (r if isinstance(r, str) else json.dumps(r)) + "\n" for r in recs).encode()))


def test_corpus2_takes_lightning_ticks_after_the_cut_and_attaches_the_path(tmp_path):
  t0 = g.CORPUS2_SINCE + 3600.0
  pts = [{"latitude": 0.5 + 0.0004 * i, "longitude": 0.5, "velocity": 40.0 if i != 6 else 18.0} for i in range(20)]
  _key, frag = m.mapd_path_encode(pts, 0.5, 0.5)
  recs = [{"t": t0 - 1.0, "ev": "mapdPath", "car": LIGHT, "seq": 1, "lat": 0.5, "lon": 0.5, "bearing": 0.0, **frag}]
  recs += [_tick(t0 + i, mapDist=200.0 - 30.0 * i) for i in range(6)]
  recs += [_tick(t0 + 10.0, car="TESLA_MODEL_S_HW3"), _tick(g.CORPUS2_SINCE - 10.0), "garbage {"]
  for r in recs:
    if isinstance(r, dict):
      r.pop("_src", None)
      r.pop("_hasK", None)
  _zst(tmp_path / "ces_events.jsonl.20990101T000000Z.zst", recs)
  ticks, paths, inv = g.scan_pnwlogs(str(tmp_path))
  assert len(ticks) == 6 and all(r["car"] == LIGHT and r["t"] >= g.CORPUS2_SINCE for r in ticks)
  assert sum(len(v) for v in paths.values()) == 1
  assert inv[0][1]["unparsable"] == 1 and inv[0][2] is None
  cov = g.Coverage()
  with g.Env() as env:
    notes = g.corpus2(env, g.Sink(), cov, d=str(tmp_path))
  assert cov.acct["c2_pnwlogs"]["ticks"] == 6 and cov.acct["c2_pnwlogs"]["ticks with a mapd path"] == 6
  assert "1 mapdPath records" in notes[0]


def test_corpus2_says_so_when_no_tick_has_a_path(tmp_path):
  recs = [_tick(g.CORPUS2_SINCE + 60.0 + i) for i in range(3)]
  for r in recs:
    r.pop("_src")
  _zst(tmp_path / "ces_events.jsonl.20990101T000000Z.zst", recs)
  with g.Env() as env:
    notes = g.corpus2(env, g.Sink(), g.Coverage(), d=str(tmp_path))
  assert any("NO mapdPath RECORD" in n for n in notes)


# ---------------------------------------------------------------------------------------------------------------------
# coverage, comparison, the fuzz matrix
# ---------------------------------------------------------------------------------------------------------------------
def test_coverage_lists_every_value_under_the_bar_and_every_unknown_one():
  cov = g.Coverage()

  def line(src):
    return {"rec": {"icbmSrc": src, "icbmGate": None, "cdb2Why": "off", "cdb2Dir": None, "shpWhy": "gps",
                    "icbmPhase": "idle"}, "pub": {}, "log": []}
  for _ in range(25):
    cov.add("c4_fuzz", line("map"))
  for _ in range(3):
    cov.add("c1_cache", line("vis"))
  cov.add("c4_fuzz", line("bogus"))
  under = {(f, v): (n, why) for f, v, n, why in cov.under()}
  assert ("icbmSrc", "map") not in under
  assert under[("icbmSrc", "vis")] == (3, "") and under[("icbmSrc", "gpsHold")] == (0, "")
  assert "NOT IN UNIVERSE" in under[("icbmSrc", "bogus")][1]
  rep = cov.report()
  assert "icbmSrc=gpsHold: 0" in rep and "icbmSrc=bogus: 1" in rep


def test_compare_finds_a_difference_and_an_exclusion_hides_only_its_key(tmp_path):
  a, b = g.Sink(str(tmp_path / "a.zst")), g.Sink(str(tmp_path / "b.zst"))
  for i in range(4):
    a.write({"rec": {"icbmT": i, "cbOn": "shadow"}, "st": {}, "pub": {}, "log": []})
    b.write({"rec": {"icbmT": i if i != 2 else 9, "cbOn": "off"}, "st": {}, "pub": {}, "log": []})
  a.close()
  b.close()
  n, bad, first = g.compare(str(tmp_path / "a.zst"), str(tmp_path / "b.zst"))
  assert (n, bad) == (4, 4)
  n, bad, first = g.compare(str(tmp_path / "a.zst"), str(tmp_path / "b.zst"), exclude=("cbOn",))
  assert (n, bad) == (4, 1) and first[0][0] == 3 and first[0][1] == ["rec.icbmT"]


def test_the_fuzz_walks_the_whole_matrix_and_is_seeded(tmp_path):
  import random
  assert len(g.MATRIX) == 72 and len(set(g.MATRIX)) == 72
  rng = random.Random(1)
  cells = set()
  for i in range(len(g.MATRIX)):
    cfg, _road, _nodes, sc = g.fuzz_scenario(rng, i, str(tmp_path))
    cells.add((cfg["shape"], cfg["db"], cfg["rain"], sc["gps"][0][1]))
  assert cells == set(g.MATRIX)
  a = g.fuzz_scenario(random.Random(7), 3, str(tmp_path / "x"))
  b = g.fuzz_scenario(random.Random(7), 3, str(tmp_path / "y"))
  c = g.fuzz_scenario(random.Random(8), 3, str(tmp_path / "z"))
  assert a[1].lat == b[1].lat and a[3] == b[3] and a[3] != c[3]


# ---------------------------------------------------------------------------------------------------------------------
# corpus 3: the private fixture
# ---------------------------------------------------------------------------------------------------------------------
@pytest.mark.skipif(not os.path.exists(g.FIXTURE),
                    reason=f"PRIVATE curve-DB replay fixture absent ({g.FIXTURE}): corpus 3 NOT exercised -- " +
                           "build it with tools/curvedb/v2_live_fixture.py")
def test_corpus3_drives_the_real_passes_through_the_curve_db():
  cov = g.Coverage()
  with g.Env() as env:
    g.corpus3(env, g.Sink(), cov)
  h = cov.hits["c3_fixture"]
  assert h["cdb2Dir"]["raise"] > 0 and h["cdb2Dir"]["add"] > 0          # the DB acts on real geometry
  assert not [k for k in cov.acct["c3_fixture"] if k.startswith("log exception")]
