"""Unit tests for the section 7 go/no-go replay.

The replay is the one place a sloppy implementation would flatter the result, so these tests are
written against the failure modes rather than the happy path: a leave-one-date-out that leaks, a
counterfactual computed at the wrong speed, a verdict that reads a clean zero out of a matcher that
never matched, and a control that cannot tell a corrupted database from a working one.
"""
import math

import pytest

from openpilot.tools.curvedb import replay as R
from openpilot.tools.curvedb.store import (
  PROVISIONAL_ENVELOPES,
  PROVISIONAL_PARAMS,
  CurveDB,
  Observation,
  tighten,
)

P = PROVISIONAL_PARAMS
E = PROVISIONAL_ENVELOPES
LIGHTNING = "FORD_F_150_LIGHTNING_MK1"
LAT, LON, BRG = 45.0, -122.0, 90.0


def obs(date="2026-09-01", k=0.004, **over):
  base = dict(date=date, t=1789000000.0, car=LIGHTNING, drive_id="drive-" + date,
              site_lat=LAT, site_lon=LON,
              bearing_deg=BRG, k=k, kind="up", estimator="kPeak100", site_src="logged",
              source="x:1", posted_ms=29.0, highway_class="motorway", n_ticks=7,
              dq_state="clean", dq_src="rollup100")
  base.update(over)
  return Observation(**base)


def episode(**over):
  """A phantom by construction: the road ahead is k = 0.0005 (R = 2 km), and ICBM cut 29 -> 19."""
  e = dict(date="2026-09-05", t=1789000000.0, t_end=1789000010.0, car=LIGHTNING,
           drive_id="drive-2026-09-05",
           start_lat=LAT, start_lon=LON, start_bearing=BRG, v_ego_start=29.0,
           ref_ms=29.0, ref_src="icbmC", icbm_target_ms=19.0, reduction_ms=10.0, icbm_src="map",
           site_lat=LAT, site_lon=LON, site_src="logged", site_attrib="logged_candidate",
           site_dist_m=300.0, passage_err_m=3.0, approach_bearing=BRG, bearing_src="logged",
           k_truth=0.0005, k_estimator="kPeak100", k_n=7, k_v_ego=29.0,
           k_ahead_max=0.0005, k_ahead_v_ego=29.0,
           a_lat_measured=0.4, a_lat_src="achLat", dq_state="clean", dq_src="rollup100",
           dq_why="",
           posted_ms=29.0, highway_class="motorway", n_ticks=7, source="x:9")
  e.update(over)
  return e


def two_dates(k=0.0005):
  return [obs(date="2026-09-01", k=k), obs(date="2026-09-02", k=k)]


# --------------------------------------------------------------------------------- the arithmetic

def test_rule_of_three():
  assert R.rule_of_three(60) == pytest.approx(0.05)
  assert R.rule_of_three(3) == pytest.approx(1.0)
  assert R.rule_of_three(0) is None
  assert R.rule_of_three(-1) is None


def test_verdict_curvature_takes_the_worse_of_the_site_and_the_lookahead():
  # A cancel must be judged against any demanding bend in the 500 m the truck was about to drive,
  # not only against the node the attribution happened to pick.
  k, v = R._verdict_curvature(episode(k_truth=0.001, k_v_ego=10.0,
                                      k_ahead_max=0.006, k_ahead_v_ego=28.0))
  assert (k, v) == (0.006, 28.0)
  k, v = R._verdict_curvature(episode(k_truth=0.006, k_v_ego=28.0,
                                      k_ahead_max=0.001, k_ahead_v_ego=10.0))
  assert (k, v) == (0.006, 28.0)


@pytest.mark.parametrize("bad", [None, 0.0, -0.001, float("nan"), "0.004"])
def test_verdict_curvature_refuses_a_non_measurement(bad):
  assert R._verdict_curvature(episode(k_truth=bad, k_ahead_max=bad)) == (None, 0.0)


# ---------------------------------------------------------------------------------- the verdicts

def evaluate(observations, ep, params=P):
  return R.evaluate(ep, CurveDB.build(observations, params), params, E,
                    real_a_lat=R.REAL_SLOWDOWN_A_LAT_MS2, phantom_a_lat=R.PHANTOM_A_LAT_MS2)


def test_a_gentle_row_refutes_a_phantom():
  o = evaluate(two_dates(k=0.0005), episode())
  assert o.matched and o.acted
  # sqrt(2.5/0.0005) = 70.7 m/s, capped at the 29 m/s posted limit, which is also the reference.
  assert o.v_row_ms == 29.0
  assert o.new_target_ms == 29.0
  assert o.a_cf == pytest.approx(0.0005 * 29.0 ** 2)
  assert o.verdict == "phantom_refuted"


def test_a_row_that_understates_a_real_curve_is_a_false_cancel():
  # The database says gentle; the road turns out to be k = 0.004 at 29 m/s = 3.36 m/s^2.
  o = evaluate(two_dates(k=0.0005), episode(k_truth=0.004, k_ahead_max=0.004))
  assert o.acted
  assert o.verdict == "false_cancel"
  assert o.a_cf == pytest.approx(0.004 * 29.0 ** 2)


def test_the_grey_band_is_never_counted_as_a_win():
  # k = 0.0024 at 29 m/s = 2.02 m/s^2: above the phantom line, below the real line.
  o = evaluate(two_dates(k=0.0005), episode(k_truth=0.0024, k_ahead_max=0.0024))
  assert o.verdict == "grey"


def test_the_counterfactual_uses_the_speed_the_database_would_allow_not_icbms():
  # The distinction that decides everything: at ICBM's 19 m/s this curve pulls 1.4 m/s^2 and looks
  # harmless; at the 29 m/s the database would permit it pulls 3.4 and is a false cancel.
  ep = episode(k_truth=0.004, k_ahead_max=0.004)
  o = evaluate(two_dates(k=0.0005), ep)
  assert o.a_at_icbm == pytest.approx(0.004 * 19.0 ** 2)
  assert o.a_at_icbm < R.REAL_SLOWDOWN_A_LAT_MS2 < o.a_cf
  assert o.verdict == "false_cancel"


def test_an_action_with_no_ground_truth_is_neither_a_win_nor_a_loss():
  o = evaluate(two_dates(), episode(k_truth=None, k_ahead_max=None))
  assert o.acted
  assert o.verdict == "no_ground_truth"
  assert o.a_cf is None


def test_no_row_means_no_action():
  o = evaluate([], episode())
  assert not o.matched and not o.acted
  assert o.verdict == "no_row"
  assert o.new_target_ms == 19.0        # ICBM's own target, untouched


def test_a_matched_row_without_authority_does_not_act():
  o = evaluate([obs(date="2026-09-01")], episode())     # one pass, one date
  assert o.matched and not o.acted
  assert o.verdict == "no_authority"


def test_a_row_that_would_lower_the_target_does_not_count_as_acting():
  # k = 0.02 gives sqrt(2.5/0.02) = 11.2 m/s, below ICBM's 19: max() leaves the target alone.
  o = evaluate(two_dates(k=0.02), episode())
  assert o.matched and not o.acted
  assert o.verdict == "no_action"
  assert o.new_target_ms == 19.0


def test_a_row_can_never_raise_the_target_above_the_reference():
  o = evaluate(two_dates(k=1e-4), episode(ref_ms=24.0, posted_ms=40.0))
  assert o.new_target_ms == 24.0


def test_the_lookup_uses_the_approach_bearing_so_the_other_carriageway_is_a_different_road():
  o = evaluate(two_dates(), episode(approach_bearing=(BRG + 180.0) % 360.0))
  assert o.verdict == "no_row"


def test_an_episode_with_no_usable_fix_cannot_match():
  o = evaluate(two_dates(), episode(site_lat=None))
  assert o.verdict == "no_row"


# ------------------------------------------------------------------------- leave-one-date-out

def test_lodo_removes_the_episodes_own_date():
  same = [obs(date="2026-09-05"), obs(date="2026-09-05", t=1789000001.0)]
  ep = episode(date="2026-09-05")
  assert R.run(same, [ep], P, E, lodo=True)[0].verdict == "no_row"
  assert R.run(same, [ep], P, E, lodo=False)[0].matched


def test_lodo_keeps_every_other_date():
  mixed = [obs(date="2026-09-01"), obs(date="2026-09-02"), obs(date="2026-09-05")]
  out = R.run(mixed, [episode(date="2026-09-05")], P, E, lodo=True)
  assert out[0].matched and out[0].acted


def test_the_lodo_property_is_re_checked_not_assumed():
  db = CurveDB.build([obs(date="2026-09-05")], P)
  with pytest.raises(AssertionError):
    R._assert_lodo(db, "2026-09-05", "other-drive")
  with pytest.raises(AssertionError):
    R._assert_lodo(db, "2026-01-01", "drive-2026-09-05")     # same drive, different date
  R._assert_lodo(db, "2026-09-01", "other-drive")            # must not raise


def test_the_db_cache_returns_a_different_database_per_excluded_key():
  cache = R._DBCache([obs(date="2026-09-01"), obs(date="2026-09-02")], P)
  assert cache.get(("2026-09-01", "drive-2026-09-01")).rows[0].dates == ("2026-09-02",)
  assert cache.get(("2026-09-02", "drive-2026-09-02")).rows[0].dates == ("2026-09-01",)
  assert cache.get(None).rows[0].dates == ("2026-09-01", "2026-09-02")


def test_lodo_excludes_the_drive_even_when_it_crossed_pt_midnight():
  # An observation's date is the PT date of the PASSAGE; an episode's is the PT date of the
  # DECISION. A drive spanning midnight between the two would otherwise put the episode's own
  # pass straight back into the database that judges it.
  own_pass = obs(date="2026-09-06", drive_id="drive-X")       # next PT day, SAME drive
  other = obs(date="2026-09-01", drive_id="drive-Y")
  ep = episode(date="2026-09-05", drive_id="drive-X")
  out = R.run([own_pass, other], [ep], P, E, lodo=True)
  assert all(o.drive_id != "drive-X" for r in R._DBCache([own_pass, other], P)
             .get(("2026-09-05", "drive-X")).rows for o in r.observations)
  assert out[0].matched      # `other` is still there, so this is not a vacuous pass


# ------------------------------------------------------------------------------------- controls

def test_k_shuffle_keeps_the_places_and_moves_the_curvatures():
  o = [obs(date="2026-09-01", k=0.001), obs(date="2026-09-02", k=0.009, site_lat=46.0)]
  out = R.shuffle_k(o, seed=1)
  assert [x.k for x in out] != [x.k for x in o] or len(o) < 2
  assert sorted(x.k for x in out) == sorted(x.k for x in o)
  assert [x.site_lat for x in out] == [x.site_lat for x in o]


def _two_days_of_sites(n=12):
  return [obs(date=f"2026-09-{1 + i % 2:02d}", k=0.001 * (i + 1), site_lat=45.0 + 0.05 * i)
          for i in range(n)]


def test_site_shuffle_keeps_the_curvatures_and_moves_the_places():
  o = _two_days_of_sites()
  out = R.shuffle_sites(o, seed=1)
  assert [x.k for x in out] == [x.k for x in o]
  assert sorted(x.site_lat for x in out) == sorted(x.site_lat for x in o)
  assert [x.site_lat for x in out] != [x.site_lat for x in o]


def test_site_shuffle_permutes_WITHIN_a_date_and_never_across_one():
  # This is the whole point of the control: a global shuffle reassigns sites across dates and
  # MANUFACTURES the multi-date rows the real corpus lacks, which made the corrupted database look
  # more capable than the real one.
  o = _two_days_of_sites()
  out = R.shuffle_sites(o, seed=1)
  for date in ("2026-09-01", "2026-09-02"):
    before = sorted(x.site_lat for x in o if x.date == date)
    after = sorted(x.site_lat for x in out if x.date == date)
    assert before == after


def test_site_shuffle_is_seed_dependent():
  o = _two_days_of_sites()
  assert [x.site_lat for x in R.shuffle_sites(o, 7)] == [x.site_lat for x in R.shuffle_sites(o, 7)]
  assert [x.site_lat for x in R.shuffle_sites(o, 7)] != [x.site_lat for x in R.shuffle_sites(o, 8)]


def test_the_controls_are_deterministic():
  o = [obs(date=f"2026-09-{i:02d}", k=0.001 * (i + 1)) for i in range(1, 9)]
  assert [x.k for x in R.shuffle_k(o, 7)] == [x.k for x in R.shuffle_k(o, 7)]
  assert [x.k for x in R.shuffle_k(o, 7)] != [x.k for x in R.shuffle_k(o, 8)]


def test_k_shuffle_actually_changes_the_verdict():
  # The control has to be able to CHANGE something, or running it proves nothing -- and the proof
  # has to be the VERDICT, not merely that a number moved.
  tight = [obs(date="2026-09-01", k=0.02), obs(date="2026-09-02", k=0.02)]
  gentle = [obs(date="2026-09-01", k=0.0005, site_lat=46.0),
            obs(date="2026-09-02", k=0.0005, site_lat=46.0)]
  ep = episode(k_truth=0.004, k_ahead_max=0.004)
  # As built: the site the episode is at reads TIGHT, so the database will not raise the target.
  assert R.run(tight + gentle, [ep], P, E)[0].verdict == "no_action"
  # Find a seed that moves the gentle curvatures onto the episode's site; the verdict must follow.
  moved = [s for s in range(50)
           if R.run(R.shuffle_k(tight + gentle, s), [ep], P, E)[0].verdict != "no_action"]
  assert moved, "k-shuffle never changed a verdict in 50 seeds -- the control cannot fail"
  assert R.run(R.shuffle_k(tight + gentle, moved[0]), [ep], P, E)[0].verdict == "false_cancel"


# ------------------------------------------------------------------------------------- reporting

def test_summarise_counts_the_funnel(capsys):
  outs = R.run(two_dates(), [episode(), episode(k_truth=0.004, k_ahead_max=0.004)], P, E)
  res = R.summarise(outs, "t", real_a_lat=R.REAL_SLOWDOWN_A_LAT_MS2)
  assert res == {"n": 2, "acted": 2, "adjudicable": 2, "false_cancels": 1, "phantoms": 1,
                 "grey": 0, "matched": 2}
  out = capsys.readouterr().out
  assert "REAL SLOWDOWNS WRONGLY CANCELLED          1" in out
  assert "EVERY FALSE CANCEL" in out


def test_summarise_refuses_to_turn_zero_of_zero_into_a_bound(capsys):
  R.summarise(R.run([], [episode()], P, E), "t", real_a_lat=2.5)
  assert "0-of-0 is not evidence" in capsys.readouterr().out


def test_summarise_states_the_bound_when_there_is_one(capsys):
  eps = [episode(date=f"2026-09-{d:02d}") for d in range(5, 9)]
  R.summarise(R.run(two_dates(), eps, P, E), "t", real_a_lat=2.5)
  out = capsys.readouterr().out
  assert "rule of three, N=4" in out
  assert "NOT MET" in out                 # section 3.9 item 5 wants N >= 60


def test_summarise_flags_near_misses_under_the_threshold(capsys):
  # k = 0.0025 at 29 m/s is 2.10 m/s^2: 84 % of the 2.5 line. A zero with these stacked at the line
  # is not a safe zero, and the report has to say so.
  R.summarise(R.run(two_dates(), [episode(k_truth=0.0025, k_ahead_max=0.0025)], P, E),
              "t", real_a_lat=2.5)
  assert "within 20% below the threshold            1" in capsys.readouterr().out


def test_summarise_breaks_down_why_the_database_did_not_act(capsys):
  R.summarise(R.run([obs(date="2026-09-01")], [episode()], P, E), "t", real_a_lat=2.5)
  out = capsys.readouterr().out
  assert "why the database did NOT act" in out
  assert "only 1 pass" in out


# ------------------------------------------------------------------------------ the whole command

def write_jsonl(path, rows):
  import dataclasses
  import json
  with open(path, "w") as f:
    for r in rows:
      f.write(json.dumps(dataclasses.asdict(r) if isinstance(r, Observation) else r) + "\n")
  return str(path)


def test_main_returns_no_result_when_nothing_was_adjudicated(tmp_path, capsys):
  o = write_jsonl(tmp_path / "o.jsonl", [obs(date="2026-09-01")])
  e = write_jsonl(tmp_path / "e.jsonl", [episode()])
  assert R.main(["--observations", o, "--episodes", e]) == 2
  assert "VERDICT: NO RESULT" in capsys.readouterr().out


def test_main_fails_the_gate_on_a_single_false_cancel(tmp_path, capsys):
  o = write_jsonl(tmp_path / "o.jsonl", two_dates())
  e = write_jsonl(tmp_path / "e.jsonl", [episode(k_truth=0.004, k_ahead_max=0.004)])
  assert R.main(["--observations", o, "--episodes", e]) == 1
  assert "FAILS section 7" in capsys.readouterr().out


def test_main_passes_with_a_stated_bound(tmp_path, capsys):
  o = write_jsonl(tmp_path / "o.jsonl", two_dates())
  e = write_jsonl(tmp_path / "e.jsonl", [episode()])
  assert R.main(["--observations", o, "--episodes", e]) == 0
  out = capsys.readouterr().out
  assert "zero false cancels over N=1" in out
  assert "NOT MET" in out


def test_summarise_prints_the_literal_reading_alongside_the_counterfactual(capsys):
  # I1 claims both readings are reported. They must actually both be printed, or the claim is the
  # visK shape: a field written by ingest and never read.
  R.summarise(R.run(two_dates(), [episode(a_lat_measured=0.4)], P, E), "t", real_a_lat=2.5)
  out = capsys.readouterr().out
  # NOT just "MEASURED |achLat|" -- the not-available branch prints that too, so asserting on it
  # would pass with the measurement silently missing. Assert on the number and its label.
  assert "LITERAL 3.9-6 reading (n=1)" in out
  assert "p50 0.40" in out
  assert "counterfactual lateral accel" in out


def test_main_warns_when_the_self_match_diagnostic_finds_nothing_extra(tmp_path, capsys):
  o = write_jsonl(tmp_path / "o.jsonl", two_dates())
  e = write_jsonl(tmp_path / "e.jsonl", [episode(site_lat=46.0)])     # nowhere near any row
  R.main(["--observations", o, "--episodes", e, "--self-match"])
  assert "broken matcher, not a thin corpus" in capsys.readouterr().out


def test_main_dq_filter_excludes_unknown_disqualifiers_by_default(tmp_path, capsys):
  rows = [obs(date="2026-09-01", dq_state="unknown"), obs(date="2026-09-02", dq_state="unknown")]
  o = write_jsonl(tmp_path / "o.jsonl", rows)
  e = write_jsonl(tmp_path / "e.jsonl", [episode()])
  R.main(["--observations", o, "--episodes", e])
  assert "0 of 2 observations" in capsys.readouterr().out


def test_main_highway_only_filter_reports_what_it_removed(tmp_path, capsys):
  o = write_jsonl(tmp_path / "o.jsonl", two_dates())
  e = write_jsonl(tmp_path / "e.jsonl", [episode(), episode(highway_class="residential")])
  R.main(["--observations", o, "--episodes", e, "--highway-only"])
  assert "1 of 2 episodes are motorway/trunk" in capsys.readouterr().out


def test_main_labels_the_unknown_class_run_as_a_sensitivity_run(tmp_path, capsys):
  o = write_jsonl(tmp_path / "o.jsonl", two_dates())
  e = write_jsonl(tmp_path / "e.jsonl", [episode(highway_class="unknown")])
  R.main(["--observations", o, "--episodes", e, "--allow-unknown-highway-class"])
  out = capsys.readouterr().out
  assert "SENSITIVITY RUN, not a result" in out


def test_main_can_widen_the_matching_radius_from_the_command_line(tmp_path, capsys):
  far = [obs(date="2026-09-01", site_lat=LAT + 0.001), obs(date="2026-09-02", site_lat=LAT + 0.001)]
  o = write_jsonl(tmp_path / "o.jsonl", far)      # ~111 m away: outside the 40 m default
  e = write_jsonl(tmp_path / "e.jsonl", [episode()])
  assert R.main(["--observations", o, "--episodes", e]) == 2
  assert R.main(["--observations", o, "--episodes", e, "--site-radius-m", "200"]) == 0


def test_the_replay_thresholds_are_the_documented_values():
  assert R.REAL_SLOWDOWN_A_LAT_MS2 == 2.5          # section 3.9 item 6
  assert R.PHANTOM_A_LAT_MS2 == 1.5                # PROVISIONAL; the design does not define it
  assert R.ACT_EPS_MS == 0.1
  assert set(R.VERDICTS) == {"no_row", "no_authority", "no_action", "false_cancel", "grey",
                             "phantom_refuted", "no_ground_truth"}


def test_evaluate_is_pure_enough_to_drive_from_a_test():
  # No I/O, no globals: the same inputs give the same outcome, twice.
  db = CurveDB.build(two_dates(), P)
  a = R.evaluate(episode(), db, P, E, real_a_lat=2.5, phantom_a_lat=1.5)
  b = R.evaluate(episode(), db, P, E, real_a_lat=2.5, phantom_a_lat=1.5)
  assert a.verdict == b.verdict and a.new_target_ms == b.new_target_ms


def test_a_tighter_comfort_target_makes_the_database_more_cautious():
  # icbm_target is 10 m/s so that BOTH derived speeds sit above it and max() does not mask the
  # difference -- the design's formula can only ever raise the target, never lower it.
  ep = episode(k_truth=0.004, k_ahead_max=0.004, posted_ms=40.0, ref_ms=40.0, icbm_target_ms=10.0)
  loose = evaluate(two_dates(k=0.004), ep, params=tighten(P, a_lat_comfort_ms2=2.5))
  tight = evaluate(two_dates(k=0.004), ep, params=tighten(P, a_lat_comfort_ms2=1.0))
  assert tight.new_target_ms < loose.new_target_ms
  assert math.isclose(loose.new_target_ms, math.sqrt(2.5 / 0.004), rel_tol=1e-9)
  assert math.isclose(tight.new_target_ms, math.sqrt(1.0 / 0.004), rel_tol=1e-9)
