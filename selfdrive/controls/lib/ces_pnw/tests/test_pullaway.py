"""pullaway2pnw — evidence-gated lead-pull-away exception below the redlight2pnw floor.

Incident (2026-07-12 ~14:0x PT, city, op-long, Experimental): ~17 mph behind a lead; the lead
pulled away; CES held the lowSpeed Experimental (e2e slow to accelerate) and the truck lost the
lead. The redlight2pnw floor (ACCEL_ZONE_MIN_V, 18 mph) correctly blocks the NO-LEAD red-light
approach but also blocked this legitimate case just below it. Driver's standing rule: "the lead
car cannot pull away, that's unacceptable."

The exception adopts Chill below the floor ONLY on full evidence: lead PRESENT in the 5-60 m band,
genuinely OPENING (>= +1 m/s AND monotonic dRel rise over 3 spaced samples), model does NOT want
to stop (now, nor within the last 2 s — the yellow-light trap), ego moving (>= 2 m/s). Any
condition false -> exactly the redlight2pnw behavior; above the floor byte-identical.
"""
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import (
  decide_active, _accelerate_zone, _accelerate_zone_base, _lead_pull_away, PullAwayTracker)

ALL_ON = {"curves": True, "stops": True, "low_speed": True, "lead": True}


def base(**kw):
  s = {
    "v_ego": 30.0, "has_lead": False, "lead_vlead": 0.0, "lead_drel": 0.0, "blinker": False,
    "map_target_v": 0.0, "map_target_dist": float('inf'),
    "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0,
    "model_should_stop": False, "v_set": 0.0, "spd_lim": 0.0, "toggles": ALL_ON,
  }
  s.update(kw)
  return s


def pull(**kw):
  """The incident shape: 16 mph behind an opening lead, city set speed, full evidence."""
  s = base(v_ego=7.2, has_lead=True, lead_drel=25.0, lead_vlead=8.5,
           v_set=25 * CV.MPH_TO_MS, lead_opening=True)
  s.update(kw)
  return s


# ---- the incident: pull-away below the floor adopts Chill with its own reason --------------------
def test_pullaway_16mph_adopts_chill_with_reason():
  s = pull()
  assert C.PULLAWAY_MIN_V <= s["v_ego"] < C.ACCEL_ZONE_MIN_V          # genuinely below the floor
  assert _lead_pull_away(s) is True
  assert _accelerate_zone(s) is True
  assert _accelerate_zone_base(s) is False                            # ONLY the exception fires
  assert decide_active(s) == (False, "pullAway")                      # named for field validation


def test_pullaway_15_and_17_mph_band():
  for mph in (15.0, 17.0):
    v = mph * CV.MPH_TO_MS
    s = pull(v_ego=v, lead_vlead=v + 1.5)
    assert decide_active(s) == (False, "pullAway"), mph


# ---- THE TRAP: red light / stop while the lead clears through ------------------------------------
def test_trap_lead_clears_yellow_ego_must_stop_stays_experimental():
  """The adversarial scenario: the LEAD accelerates through a yellow and pulls away while ego must
  stop. The model wants to stop -> the exception is dead (condition c), the accel-zone is dead
  (existing model gate), and the lowSpeed hold keeps Experimental — which stops for the light."""
  s = pull(model_should_stop=True)
  assert _lead_pull_away(s) is False
  assert _accelerate_zone(s) is False
  active, reason = decide_active(s)
  assert active is True and reason == "lowSpeed"                      # stays Experimental


def test_trap_stop_recency_blocks_after_flicker():
  """shouldStop flicker/lag while the lead clears: the tracker's recency guard keeps lead_opening
  False for PULLAWAY_STOP_CLEAR_S after the LAST stop intent, even with perfect opening evidence."""
  trk = PullAwayTracker()
  t = 100.0
  # build perfect opening evidence WHILE the model briefly wants to stop
  assert trk.update(t + 0.0, True, 20.0, True) is False
  assert trk.update(t + 0.3, True, 21.0, False) is False
  assert trk.update(t + 0.6, True, 22.0, False) is False              # evidence complete, stop recent
  assert trk.update(t + 0.9, True, 23.0, False) is False              # still inside the 2 s window
  assert trk.update(t + 2.1, True, 24.0, False) is True               # window cleared -> allowed


def test_trap_lead_vanishes_floor_holds():
  """Alternative trap arm: the cleared lead is DROPPED by radar -> has_lead False -> the exception
  requires a present lead, so the no-lead redlight2pnw floor holds exactly as before."""
  s = pull(has_lead=False, lead_drel=0.0, lead_vlead=0.0)
  assert _lead_pull_away(s) is False and _accelerate_zone(s) is False
  assert decide_active(s) == (True, "lowSpeed")


# ---- the remaining condition matrix ---------------------------------------------------------------
def test_standstill_never_fires():
  s = pull(v_ego=1.5, lead_vlead=4.0)                                 # below PULLAWAY_MIN_V
  assert _lead_pull_away(s) is False
  assert decide_active(s) == (True, "lowSpeed")


def test_drel_band_edges():
  assert _lead_pull_away(pull(lead_drel=4.0)) is False                # too close: not "pulled away"
  assert _lead_pull_away(pull(lead_drel=61.0, lead_vlead=8.5)) is False  # beyond the sane band
  assert _lead_pull_away(pull(lead_drel=5.0)) is True
  assert _lead_pull_away(pull(lead_drel=60.0)) is True


def test_dv_threshold():
  assert _lead_pull_away(pull(lead_vlead=7.2 + C.PULLAWAY_DV - 0.1)) is False   # not opening
  assert _lead_pull_away(pull(lead_vlead=7.2 + C.PULLAWAY_DV + 0.1)) is True


def test_no_opening_evidence_no_fire():
  s = pull(lead_opening=False)                                        # dv alone is NOT enough
  assert _lead_pull_away(s) is False
  assert decide_active(s) == (True, "lowSpeed")
  s.pop("lead_opening")                                               # older pure callers: no key
  assert _lead_pull_away(s) is False


def test_above_floor_byte_identical():
  """Above ACCEL_ZONE_MIN_V nothing changes: with a near opening lead (25 m) the base gate still
  says 'not open road' and the lowSpeed hold stands, exactly as on origin/3devpnw."""
  s = pull(v_ego=8.5, lead_vlead=10.5)                                # 19 mph, above the floor
  assert _lead_pull_away(s) is False
  assert _accelerate_zone(s) is _accelerate_zone_base(s) is False
  assert decide_active(s) == (True, "lowSpeed")


def test_base_far_lead_path_keeps_chill_reason():
  """When the pre-existing has-lead-far branch (dRel > 45 m) fires, the reason stays 'chill' —
  'pullAway' names ONLY decisions the new exception made."""
  s = base(v_ego=4.0, has_lead=True, lead_drel=60.0, lead_vlead=10.0,
           v_set=36 * CV.MPH_TO_MS, lead_opening=True)
  assert _accelerate_zone_base(s) is True
  assert decide_active(s) == (False, "chill")


# ---- PullAwayTracker evidence mechanics -----------------------------------------------------------
def test_tracker_monotonic_rise_required():
  trk = PullAwayTracker()
  t = 50.0
  assert trk.update(t + 0.0, True, 20.0, False) is False
  assert trk.update(t + 0.3, True, 21.0, False) is False
  assert trk.update(t + 0.6, True, 22.0, False) is True               # 3 spaced rising samples
  # noise-level "rise" below PULLAWAY_OPEN_EPS never proves opening
  trk2 = PullAwayTracker()
  for i, d in enumerate((20.0, 20.1, 20.2, 20.3)):
    assert trk2.update(t + 0.3 * i, True, d, False) is False


def test_tracker_sample_spacing_100hz_calls():
  """Called at the 100 Hz control rate, samples must still be >= PULLAWAY_SAMPLE_GAP_S apart —
  evidence takes >= ~0.5 s of genuine opening, not 3 consecutive noisy frames."""
  trk = PullAwayTracker()
  t, d = 50.0, 20.0
  fired_at = None
  for _ in range(120):                                                # 1.2 s at 100 Hz
    t += 0.01
    d += 0.02                                                         # 2 m/s opening rate
    if trk.update(t, True, d, False) and fired_at is None:
      fired_at = t
  assert fired_at is not None and fired_at - 50.0 >= 2 * C.PULLAWAY_SAMPLE_GAP_S


def test_tracker_jump_and_vanish_reset():
  trk = PullAwayTracker()
  t = 50.0
  trk.update(t + 0.0, True, 20.0, False)
  trk.update(t + 0.3, True, 21.0, False)
  assert trk.update(t + 0.6, True, 45.0, False) is False              # 24 m jump: lead swap -> reset
  assert trk.update(t + 0.9, True, 46.0, False) is False              # evidence restarting
  trk2 = PullAwayTracker()
  trk2.update(t + 0.0, True, 20.0, False)
  trk2.update(t + 0.3, True, 21.0, False)
  assert trk2.update(t + 0.6, False, 0.0, False) is False             # lead lost -> evidence dies
  assert trk2.update(t + 0.9, True, 22.0, False) is False             # must rebuild from scratch


def test_incident_replay_end_to_end():
  """The incident shape end-to-end at the pure layer: tracker builds evidence over ~0.6 s of a
  genuinely opening lead at 17 mph, then decide_active flips the lowSpeed hold to pullAway."""
  trk = PullAwayTracker()
  v = 17 * CV.MPH_TO_MS
  t, d = 200.0, 18.0
  opening = False
  for _ in range(8):                                                  # ~0.8 s, lead opening 2 m/s
    t += 0.1
    d += 0.2
    opening = trk.update(t, True, d, False)
  assert opening is True
  s = pull(v_ego=v, lead_drel=d, lead_vlead=v + 2.0, lead_opening=opening)
  assert decide_active(s) == (False, "pullAway")
  # and the moment the model predicts a stop, the same shape stays Experimental
  assert decide_active({**s, "model_should_stop": True}) == (True, "lowSpeed")
