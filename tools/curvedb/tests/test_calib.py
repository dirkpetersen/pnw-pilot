"""Unit tests for the calibration measurements.

These numbers argue for two of the design's decisions (log a 100 Hz peak; pin
`approach_bearing_ref_m`), and they were previously produced by an uncommitted scratch script. The
tests are therefore written against the ways such a script lies: a ratio computed the wrong way up,
a 100 Hz peak compared against itself, a spread taken over one reference point, and -- the one that
matters most -- a corpus with no `kPeak` at all silently reporting a perfect 1.0.
"""
import json
from collections import Counter

import pytest

from openpilot.tools.curvedb import calib as C
from openpilot.tools.curvedb import ingest as I
from openpilot.tools.curvedb.store import PROVISIONAL_PARAMS
from openpilot.tools.curvedb.tests.test_ingest import raw

P = PROVISIONAL_PARAMS


def ticks_of(records):
  return [t for t in (I.normalise(r, f"m:{i}") for i, r in enumerate(records)) if t is not None]


def curve_records(**over):
  recs = [raw(i, **over) for i in range(40)]
  for i in range(19, 23):
    recs[i]["slKActl"] = 0.005
  recs[8]["mapDist"] = 300.0
  return recs


# ----------------------------------------------------------------------------- the 1 Hz estimator

def test_the_1hz_sample_is_the_ladder_without_the_100hz_rung():
  """If this used `_tick_k` it would return `kPeak` where it exists and the ratio would be 1.0
  everywhere -- a measurement comparing the peak against itself."""
  tk = ticks_of([raw(0, kPeak=0.02, slKActl=0.004, slKCmd=0.006)])[0]
  assert C._sample_1hz(tk) == 0.006
  assert I._tick_k(tk)[0] == 0.02


def test_the_sample_is_none_when_no_1hz_column_exists():
  tk = ticks_of([raw(0, kPeak=0.02)])[0]
  assert C._sample_1hz(tk) is None


def test_the_per_tick_ratio_is_peak_over_sample_not_the_other_way_up():
  # kPeak 0.02 against a 1 Hz sample of 0.005 is a 4x under-read, not 0.25.
  per_tick, _, _, _, _ = C.measure(ticks_of([raw(0, kPeak=0.02, slKActl=0.005)]), P)
  assert per_tick == [4.0]


def test_a_corpus_with_no_kpeak_reports_NOTHING_not_a_perfect_ratio():
  """The `getfattr` shape: a missing column must not read as agreement. `n=0` and a loud line, not
  a p50 of 1.0."""
  per_tick, per_extent, _, why, _ = C.measure(ticks_of(curve_records()), P)
  assert per_tick == [] and per_extent == []
  assert why["tick: no kPeak (corpus predates 2026-09-17)"] == 40
  lines = []
  C.report(per_tick, per_extent, [], why, P, 40, 1, log=lines.append)
  assert any("NO kPeak ANYWHERE IN THIS CORPUS" in ln for ln in lines)
  assert not any("p50=1.0" in ln for ln in lines)


def test_the_per_extent_ratio_is_taken_over_the_extent_ingest_would_build_a_row_from():
  recs = curve_records()
  for i in range(19, 23):
    recs[i]["kPeak"] = 0.010                # the 100 Hz peak is 2x the 1 Hz sample of 0.005
  _, per_extent, _, _, _ = C.measure(ticks_of(recs), P)
  assert per_extent == [2.0]


def test_a_site_ingest_rejects_contributes_no_ratio_and_is_counted():
  recs = [raw(i) for i in range(40)]       # no curvature anywhere -> no passage
  recs[8]["mapDist"] = 300.0
  _, per_extent, spreads, why, _ = C.measure(ticks_of(recs), P)
  assert per_extent == [] and spreads == []
  assert why["ingest: pass_no_curvature_anywhere_in_extent"] == 1


# --------------------------------------------------------------------------- the bearing spread

def test_a_straight_drive_has_no_bearing_spread():
  # 40 ticks at 25 m/s is 975 m, enough for all three reference points before the site at s=500.
  _, _, spreads, _, _ = C.measure(ticks_of(curve_records()), P)
  assert spreads == [0.0]


def test_a_turn_inside_the_decision_range_shows_up_as_a_spread():
  """The whole point: if the truck's heading at 500 m out differs from its heading at 150 m out by
  more than the matching tolerance, ingest and the car can key the same curve differently."""
  recs = curve_records()
  for i in range(40):
    # the first 8 ticks (s < 200 m, i.e. >300 m before the site) approach from the west
    recs[i]["bearing"] = 90.0 if i < 8 else 0.0
  _, _, spreads, _, _ = C.measure(ticks_of(recs), P)
  assert spreads and spreads[0] == 90.0


def test_a_spread_needs_every_reference_point_and_says_so_when_it_cannot_get_them():
  # The drive starts 150 m in, so the 500 m reference point before the site does not exist.
  recs = [raw(i) for i in range(6, 32)]
  for i in range(13, 17):
    recs[i]["slKActl"] = 0.005
  recs[2]["mapDist"] = 300.0
  _, _, spreads, why, _ = C.measure(ticks_of(recs), P)
  assert spreads == []
  assert sum(v for k, v in why.items() if k.startswith("bearing: only")) == 1


# -------------------------------------------------------------------------------------- reporting

def test_the_report_states_the_speed_error_the_ratio_implies():
  recs = curve_records()
  for i in range(19, 23):
    recs[i]["kPeak"] = 0.020               # 4x under-read -> sqrt(4) = 2.0x too fast
  per_tick, per_extent, spreads, why, _ = C.measure(ticks_of(recs), P)
  lines = []
  C.report(per_tick, per_extent, spreads, why, P, 40, 1, log=lines.append)
  assert any("2.000x too high at the median" in ln for ln in lines)


def test_the_report_prints_the_share_over_the_matching_tolerance_with_its_denominator():
  lines = []
  C.report([], [], [10.0, 40.0, 50.0, 5.0], Counter(), P, 0, 0, log=lines.append)
  assert any("2 of 4 (50.0%)" in ln for ln in lines)


def test_main_refuses_an_empty_corpus_rather_than_reporting_a_ratio(tmp_path):
  path = tmp_path / "empty.jsonl"
  path.write_text("")
  with pytest.raises(SystemExit):
    C.main([str(path)])


def test_main_runs_end_to_end(tmp_path, capsys):
  path = tmp_path / "a.jsonl"
  recs = curve_records()
  for i in range(19, 23):
    recs[i]["kPeak"] = 0.010
  path.write_text("\n".join(json.dumps(r) for r in recs))
  assert C.main([str(path)]) == 0
  assert "per extent  n=1" in capsys.readouterr().out
