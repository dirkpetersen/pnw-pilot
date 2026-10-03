"""stopgo2pnw closed-loop scenarios (REAL LongitudinalPlanner + MPC + the gate, Tesla LongControl (startingState False), a first-order actuator
with 0.15 s delay and the real CES v1 / a nochill-latch stand-in). Built for the Opus review of ccbac3ad98, which found that a lead moving away
while the e2e model is RIGHT to hold (a red light the lead runs, a pedestrian) was answered with a lunge every few seconds (7 launch/brake
cycles, 14.8 m in 30 s). The scenarios below are the review's; the e2e model is a scripted stub (shouldStop = v < 0.3 and acc < 0.1).
"""

import functools


from openpilot.selfdrive.controls.lib.tests.stopgo_sim import lead_profile, metrics, run

TIMID = lambda t, v, d: 0.02  # noqa: E731  the stuck-stop e2e: "stay", desiredAcceleration ~0.02
BRAKE = lambda a: lambda t, v, d: 0.0 if v < 0.05 else a  # noqa: E731  the e2e model is right to hold: it brakes the moving car back down


def _lead(segments, d0):
  return lambda: lead_profile(segments, d0)


SCN = {
  "creep_restop": (dict(lead=_lead([(2, 0), (3.5, 1.0), (9, 0.0), (10.5, -1.0), (99, 0)], 5.8), e2e_acc=TIMID), "ces"),  # 07:54-like
  "crawl_1.0": (dict(lead=_lead([(2, 0), (3, 1.0), (99, 0.0)], 5.8), e2e_acc=TIMID), "ces"),
  "brisk": (dict(lead=_lead([(2, 0), (5, 1.5), (99, 0.0)], 6.0), e2e_acc=lambda t, v, d: 0.02 if t < 5 else 1.0), "ces"),
  "queue_then_leave": (dict(lead=_lead([(2, 0), (3.5, 1.0), (5, -1.0), (15, 0), (16.5, 1.0), (99, 0.0)], 5.8), e2e_acc=TIMID), "ces"),
  "e2e_brake_noisy": (dict(lead=_lead([(2, 0), (4, 1.2), (99, 0.0)], 6.0), e2e_acc=BRAKE(-1.5), d_noise=2.5), "real"),   # sd 2.5 m dRel noise
  "e2e_brake-1.5": (dict(lead=_lead([(2, 0), (4, 1.2), (99, 0.0)], 6.0), e2e_acc=BRAKE(-1.5)), "real"),
  "e2e_brake-0.5": (dict(lead=_lead([(2, 0), (4, 1.2), (99, 0.0)], 6.0), e2e_acc=BRAKE(-0.5)), "real"),
  "ghost_move": (dict(lead=lambda: lambda t, x: {"x": 9.0, "v": 0.0}, e2e_acc=TIMID, v_noise=0.4, v_bias=0.5, d_noise=0.3), "ces"),
}


@functools.cache
def sim(name, on, T=30.0):
  s, mode = SCN[name]
  sc = dict(s)
  sc["lead"] = s["lead"]()
  out, meta = run(sc, handoff_on=on, T=T, exp_mode=mode)
  return metrics(out, meta), out


def test_e2e_right_to_hold_gets_one_launch_then_a_hold_not_a_lunge_every_few_seconds():
  """Review F1. Old commit: 7 launch/brake cycles in 30 s, up to ~1.4 m/s each, 14.8 m forward (e2e braking -1.5)."""
  m, out = sim("e2e_brake-1.5", True)
  assert m["hand_edges"] == 1, m
  assert m["x_end"] <= 2.0 and m["vmax"] <= 1.0, m
  x_15 = next(o[2] for o in out if o[0] >= 15.0)
  assert m["x_end"] - x_15 < 0.05, "still creeping after the hold should have settled"


def test_a_milder_e2e_brake_is_one_creep_not_four_metres_per_cycle():
  """Old commit: ~4 m per cycle with e2e braking -0.5."""
  m, _ = sim("e2e_brake-0.5", True)
  assert m["hand_edges"] == 1 and m["x_end"] <= 3.0, m


def test_the_stuck_creeping_lead_launches_and_stays_behind_it():
  m_new, _ = sim("creep_restop", True)
  m_old, _ = sim("creep_restop", False)
  assert m_old["t_move"] is None  # control: stock never moves (the bug)
  assert m_new["t_move"] is not None and m_new["t_move"] <= 6.0, m_new
  assert m_new["min_gap"] >= 4.0 and m_new["final_gap"] >= 4.0, m_new


def test_a_queue_that_creeps_then_stops_then_leaves_ends_with_the_launch_after_the_leave():
  m_new, _ = sim("queue_then_leave", True)
  m_old, _ = sim("queue_then_leave", False)
  assert m_old["x_end"] == 0.0  # stock: held the whole 30 s
  assert m_new["x_end"] > 15.0 and m_new["min_gap"] >= 4.0, m_new


def test_a_crawling_lead_does_not_hold_the_handoff_past_the_cap():
  m, _ = sim("crawl_1.0", True)
  assert m["hand_edges"] == 1 and m["hand_s"] <= 5.5 and m["min_gap"] >= 4.0, m


def test_a_brisk_departure_is_not_slower_and_keeps_its_gap():
  m_new, _ = sim("brisk", True)
  m_old, _ = sim("brisk", False)
  assert m_new["t_move"] <= m_old["t_move"] + 0.1 and m_new["min_gap"] >= 4.0, (m_new, m_old)


def test_a_mistracked_parked_car_never_moves_the_ego():
  m, _ = sim("ghost_move", True)
  assert m["x_end"] == 0.0 and m["hand_edges"] == 0, m


def test_radar_noise_on_a_moving_lead_does_not_bring_the_lunges_back():
  """Review F1-noise: dRel noise (sd 2.5 m, so frequent > 3 m steps and the occasional 0 m reading) on the same moving lead, e2e right to hold."""
  m, _ = sim("e2e_brake_noisy", True)
  assert m["hand_edges"] <= 2 and m["x_end"] <= 3.5, m
