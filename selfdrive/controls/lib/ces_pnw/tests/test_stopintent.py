"""stopintent2pnw — the ABSOLUTE stop-intent fast path (driver-approved 2026-07-12).

Whenever model_should_stop asserts AND the decision ladder wants Experimental, entry bypasses the
CHILL_MIN_DWELL_S cooldown, the ~1 s condition filter and every anti-flap timer — churn TOWARD
stopping is the safe direction. The RETURN to Chill keeps the full normal dwell (asymmetry).
Closes: (1) the pullaway2pnw occlusion trap (Gemini STOP: lead departs occluding a red light,
shouldStop asserts only after the Chill adoption -> 5 s cooldown held non-stopping Chill), and
(2) the PRE-EXISTING 5 s stop-blind window after ANY accel-zone Chill adoption.
"""
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import ConditionalExperimentalSwitching

ALL_ON = {"curves": True, "stops": True, "low_speed": True, "lead": True}
DT = 0.1


def sig(**kw):
  s = {
    "v_ego": 30.0, "has_lead": False, "lead_vlead": 0.0, "lead_drel": 0.0, "blinker": False,
    "map_target_v": 0.0, "map_target_dist": float('inf'),
    "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0,
    "model_should_stop": False, "v_set": 0.0, "spd_lim": 0.0, "toggles": ALL_ON,
  }
  s.update(kw)
  return s


CRUISE = sig()                                          # 67 mph open road: no condition active
REDLIGHT = sig(v_ego=6.0, v_set=25 * CV.MPH_TO_MS, model_should_stop=True)   # no lead, stop wanted
LOWSPEED = sig(v_ego=6.0, v_set=25 * CV.MPH_TO_MS)      # city low-speed hold, no stop intent


def run(sm, s, seconds):
  for _ in range(int(seconds / DT)):
    sm.update_decision(s, DT)
  return sm.mode()


def test_fast_path_bypasses_chill_cooldown_one_cycle():
  """Fresh Chill (dwell 0 — the re-entry cooldown blocks every normal entry): stop intent enters
  Experimental in ONE decision cycle, tagged stopIntent."""
  sm = ConditionalExperimentalSwitching()
  assert sm.update_decision(REDLIGHT, DT) == "experimental"             # single cycle
  assert sm.status() == "stopIntent"


def test_normal_entry_still_waits_cooldown():
  """Asymmetry control: WITHOUT stop intent, the same fresh-Chill machine must wait out the
  cooldown + filter exactly as before (no general fast path leaked in)."""
  sm = ConditionalExperimentalSwitching()
  assert sm.update_decision(LOWSPEED, DT) == "chill"                    # one cycle: still chill
  assert run(sm, LOWSPEED, C.CHILL_MIN_DWELL_S - 1.0) == "chill"        # cooldown still enforced
  assert run(sm, LOWSPEED, 3.0) == "experimental"                       # then the normal entry


def test_preexisting_stop_blind_window_regression():
  """THE pre-existing hole (predates pullaway2pnw): Chill adopted via ANY accel-zone (far-lead
  shape), stop intent asserts 1 s later -> Experimental within one decision cycle, not 5 s."""
  sm = ConditionalExperimentalSwitching()
  low = sig(v_ego=4.0, v_set=36 * CV.MPH_TO_MS)                         # lowSpeed hold
  assert run(sm, low, 10.0) == "experimental"
  az = sig(v_ego=4.0, has_lead=True, lead_drel=60.0, lead_vlead=10.0,   # the far-lead accel-zone
           v_set=36 * CV.MPH_TO_MS)
  assert run(sm, az, C.EXP_MIN_DWELL_S + 3.0) == "chill"                # adopted Chill (az)
  # ~1 s of Chill cruising, then the model wants to stop (light ahead)
  assert run(sm, az, 1.0) == "chill"
  stop = dict(az, model_should_stop=True)
  assert sm.update_decision(stop, DT) == "experimental"                 # ONE cycle, not 5 s
  assert sm.status() == "stopIntent"


def test_occlusion_trap_resolves_end_to_end():
  """The pullaway2pnw Gemini STOP scenario on the combined branch: lowSpeed Experimental behind a
  lead -> lead pulls away (evidence-complete) -> Chill adopted (reason pullAway) -> the departing
  lead reveals a red light, shouldStop asserts -> Experimental again WITHIN ONE CYCLE."""
  sm = ConditionalExperimentalSwitching()
  v = 17 * CV.MPH_TO_MS
  behind = sig(v_ego=v, has_lead=True, lead_drel=20.0, lead_vlead=v, v_set=25 * CV.MPH_TO_MS)
  assert run(sm, behind, 10.0) == "experimental"                        # city low-speed hold
  pull = sig(v_ego=v, has_lead=True, lead_drel=30.0, lead_vlead=v + 2.0,
             v_set=25 * CV.MPH_TO_MS, lead_opening=True)                # pullaway2pnw evidence
  assert run(sm, pull, C.EXP_MIN_DWELL_S + 3.0) == "chill"              # adoption (the incident fix)
  # the lead clears the intersection and reveals the red: model stop intent fires
  reveal = dict(pull, model_should_stop=True)
  assert sm.update_decision(reveal, DT) == "experimental"               # instant re-entry
  assert sm.status() == "stopIntent"


def test_return_to_chill_keeps_full_dwell_after_fast_path():
  """Asymmetry (Gemini focus a): after a fast-path entry the RETURN side is untouched — even if
  stop intent (and every condition) drops on the very next cycle, Chill needs the sustained
  all-clear decay + the full EXP_MIN dwell."""
  sm = ConditionalExperimentalSwitching()
  assert sm.update_decision(REDLIGHT, DT) == "experimental"
  # stop intent flickers off immediately; open road — the safe direction stays latched
  assert run(sm, CRUISE, C.EXP_MIN_DWELL_S - 1.0) == "experimental"     # still held
  assert run(sm, CRUISE, 3.0) == "chill"                                # normal, dwelled exit


def test_flicker_cannot_oscillate_faster_than_exp_dwell():
  """Rapid shouldStop flicker: entries are instant (safe direction) but each exit still costs the
  full dwell -> oscillation period is bounded below by EXP_MIN_DWELL_S, no churn storm."""
  sm = ConditionalExperimentalSwitching()
  flips = 0
  last = sm.mode()
  t = 0.0
  for i in range(int(60.0 / DT)):                                       # 60 s of 1 Hz flicker
    s = REDLIGHT if (i % 10 == 0) else CRUISE                           # stop intent 1 cycle/second
    m = sm.update_decision(s, DT)
    if m != last:
      flips += 1
      last = m
    t += DT
  assert flips <= 2 * int(60.0 / C.EXP_MIN_DWELL_S) + 2                 # bounded by the dwell


def test_stops_toggle_disables_fast_path():
  """Per-condition toggle respected: with the stops condition disabled the fast path never fires
  (and the stop reason itself is disabled) — the driver's configuration wins."""
  toggles = dict(ALL_ON, stops=False)
  sm = ConditionalExperimentalSwitching()
  s = sig(v_ego=6.0, v_set=25 * CV.MPH_TO_MS, model_should_stop=True, toggles=toggles)
  assert sm.update_decision(s, DT) == "chill"                           # no instant entry
  # (lowSpeed may still enter later via the NORMAL path — only the bypass is gated off)


def test_fast_path_needs_the_ladder_not_bare_flicker():
  """A bare shouldStop flicker at highway speed behind a pacing lead (no CES condition active)
  must NOT yank the mode: the fast path requires decide_active to want Experimental."""
  sm = ConditionalExperimentalSwitching()
  s = sig(v_ego=30.0, has_lead=True, lead_drel=40.0, lead_vlead=30.5, model_should_stop=True,
          v_set=70 * CV.MPH_TO_MS)
  # stop reason needs no-lead; slowLead needs a slower lead; lowSpeed needs v < thr: raw is False
  assert sm.update_decision(s, DT) == "chill"
