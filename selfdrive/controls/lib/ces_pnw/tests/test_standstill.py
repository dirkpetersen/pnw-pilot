"""standstill2pnw — the standstill latch + close-lead release hold + no-cooldown promotion.

Field basis: drives/2026-07-13/lightning-hwy99/ces_events.jsonl.
  (1) 11:34-11:38Z: chill<->experimental flapping every ~10-20 s at vEgo=0.0 behind a lead at
      dRel 16-19 m — slowLead fires -> Experimental -> a radar lead DROPOUT at standstill decays
      the condition filter -> dwell expires -> Chill -> lead reacquired -> re-fires ("horse
      bucking": every flip changes the accel profile). Every exp->chill flip tick logged
      lead=False/dRel=0 — the dropout is the trigger.
  (2) 9 of 14 standstill releases launched in CHILL with a lead at only 9-14 m (aMax 1.6-2.6
      m/s^2 within 3 s — the red-light "jolt"). stp (model shouldStop) is FALSE at standstill
      behind a lead, so the stophold2pnw A2 machinery never armed.

The fix is car-agnostic decision-core logic (same bug on every car with CES active; applies in
Lightning shadow mode too so the shadow telemetry stays truthful):
  LATCH   Experimental may not demote at v < STANDSTILL_LATCH_V (pure demotion gate — no timers
          paused, dwell keeps accumulating, releases the instant v_ego rises).
  HOLD    release from standstill with a close lead (<= STANDSTILL_RELEASE_DREL) keeps
          Experimental until v > STANDSTILL_RELEASE_V or the gap opens past
          STANDSTILL_RELEASE_CLEAR_DREL.
  PROMOTE at standstill in Chill, a raw-active trigger (slowLead/stop) enters Experimental
          without the CHILL_MIN cooldown / filter charge, debounced on STANDSTILL_PROMOTE_LEAD_S
          of continuous lead presence.
"""
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


# the hwy99 geometry, from the 11:34-11:38Z records (vSet 15.2, lead 16-19 m, vLead ~0, stp False)
def ss_lead(drel=17.0, **kw):
  """Standstill behind a stopped lead — slowLead raw-active (vLead < STOPPED_LEAD_V), stp False."""
  base = dict(v_ego=0.0, has_lead=True, lead_vlead=0.0, lead_drel=drel, v_set=15.2)
  base.update(kw)
  return sig(**base)


def ss_dropout(**kw):
  """The radar dropout tick at standstill — exactly what the flip records show (lead=False, dRel 0)."""
  base = dict(v_ego=0.0, has_lead=False, lead_vlead=0.0, lead_drel=0.0, v_set=15.2)
  base.update(kw)
  return sig(**base)


def rolling(v, drel, vlead=None, **kw):
  """Moving behind a lead, raw-INACTIVE (matched lead speed, highway spd_lim kills lowSpeed) —
  only the standstill machinery can keep Experimental here."""
  base = dict(v_ego=v, has_lead=True, lead_vlead=v if vlead is None else vlead, lead_drel=drel,
              v_set=15.2, spd_lim=25.0)
  base.update(kw)
  return sig(**base)


def run(sm, s, seconds):
  for _ in range(int(round(seconds / DT))):
    sm.update_decision(s, DT)
  return sm.mode()


def run_count_flips(sm, seq):
  """Advance through [(signals, seconds), ...] counting mode transitions."""
  flips = 0
  last = sm.mode()
  for s, seconds in seq:
    for _ in range(int(round(seconds / DT))):
      m = sm.update_decision(s, DT)
      if m != last:
        flips += 1
        last = m
  return flips


# --- (a) THE 11:34-11:38 FLAPPING REPLAY --------------------------------------------------------

def test_flapping_replay_standstill_lead_dropouts_zero_flips():
  """The field pattern: standstill, lead at 17 m firing slowLead, periodic radar dropouts (every
  flip record logged lead=False). Pre-fix: each dropout decayed the filter, the dwell expired,
  Chill adopted, the reacquired lead re-fired — 17 adopts in 4 minutes. Post-fix: ONE promotion
  to Experimental, then the latch makes standstill absorbing — zero further flips."""
  sm = ConditionalExperimentalSwitching()
  # entry: the standstill promotion (needs 0.5 s of sustained lead — well under one field second)
  seq = [(ss_lead(), 2.0)]
  # 4 minutes of the field pattern: ~9 s lead + ~3 s dropout (the log's dropout cadence)
  for _ in range(20):
    seq.append((ss_lead(), 9.0))
    seq.append((ss_dropout(), 3.0))
  flips = run_count_flips(sm, seq)
  assert sm.mode() == "experimental"
  assert flips == 1                      # exactly the one chill->experimental promotion
  # cesnochill2pnw: v_ego is 0.0 for this whole replay, so the new pure-v_ego latch arms on tick 1
  # and stays armed throughout — status is unconditionally "stopLatch" for the whole episode
  # (masking the older slowLead/standstillHold tags underneath, by design: see the review note in
  # ces_pnw_constants.py's cesnochill2pnw block on why status is unconditional while armed).
  assert sm.status() == "stopLatch"


def test_latch_holds_through_dwell_expiry_at_standstill():
  """Minimal latch check: Experimental at standstill, condition fully cleared, dwell far past
  EXP_MIN — the demotion is gated (now by the cesnochill2pnw latch, which arms unconditionally at
  v_ego=0 and is tagged "stopLatch" for the whole hold — see the standstillHold->stopLatch note
  above)."""
  sm = ConditionalExperimentalSwitching()
  run(sm, ss_lead(), 2.0)
  assert sm.mode() == "experimental"
  assert run(sm, ss_dropout(), C.EXP_MIN_DWELL_S + 10.0) == "experimental"
  assert sm.status() == "stopLatch"


# --- (b) release behind a close lead ------------------------------------------------------------

def test_release_behind_close_lead_never_chill_until_launched():
  """The jolt fix: standstill behind a lead at 10 m (the field launches were into 9-14 m gaps),
  then release. Must stay Experimental at the release tick and through the whole launch while
  v <= STANDSTILL_RELEASE_V and the gap stays short — never a Chill MPC launch into 10 m."""
  sm = ConditionalExperimentalSwitching()
  run(sm, ss_lead(drel=10.0), 12.0)                 # long red: promoted + latched, dwell >> EXP_MIN
  assert sm.mode() == "experimental"
  # the release: creep away behind the lead, raw conditions INACTIVE (worst case for the old code:
  # dwell expired, filter clear -> pre-fix this demoted at the very release tick)
  for v in (0.6, 1.0, 2.0, 3.0, 4.0, 4.9):
    assert run(sm, rolling(v, drel=12.0), 0.5) == "experimental", f"demoted during launch at v={v}"
  assert sm.status() == "standstillHold"
  # past the release speed: the hold disarms and the normal (dwelled, decayed) exit releases
  assert run(sm, rolling(5.5, drel=18.0), 1.0) == "chill"


def test_release_hold_ends_when_gap_opens():
  """The other release edge: still slow (v 3 m/s) but the lead pulled ahead past
  STANDSTILL_RELEASE_CLEAR_DREL — the hold disarms and Chill (brisk catch-up) is allowed."""
  sm = ConditionalExperimentalSwitching()
  run(sm, ss_lead(drel=10.0), 12.0)
  assert run(sm, rolling(2.0, drel=12.0), 1.0) == "experimental"   # held: gap short
  assert run(sm, rolling(3.0, drel=30.0), 1.0) == "chill"          # gap open: hold disarmed


def test_release_hold_survives_lead_dropout_mid_launch():
  """A radar dropout DURING the launch (the same flicker that caused the standstill flapping)
  must not disarm the hold and re-open the jolt — only v > RELEASE_V or a SEEN open gap ends it."""
  sm = ConditionalExperimentalSwitching()
  run(sm, ss_lead(drel=10.0), 12.0)
  assert run(sm, rolling(2.0, drel=12.0), 0.5) == "experimental"
  assert run(sm, sig(v_ego=2.0, has_lead=False, v_set=15.2, spd_lim=25.0), 1.0) == "experimental"
  assert sm.status() == "standstillHold"
  assert run(sm, rolling(6.0, drel=20.0), 1.0) == "chill"          # bounded: ends at the speed gate


def test_no_hold_when_stopped_with_far_lead():
  """Lead beyond STANDSTILL_RELEASE_CLEAR_DREL at the stop: no close-lead evidence, so the release
  tick arms nothing and the exit follows the pre-fix path (open gap = Chill catch-up is fine)."""
  sm = ConditionalExperimentalSwitching()
  run(sm, ss_lead(drel=30.0), 12.0)                # promoted (lead present), latched at standstill
  assert sm.mode() == "experimental"
  assert run(sm, rolling(1.6, drel=30.0), 1.5) == "chill"   # above A2's band, no hold: normal exit


# --- (c) release with NO lead: the green-light path is unchanged -------------------------------

def test_release_no_lead_keeps_stophold_behavior():
  """Red light with NO lead (stop-intent entry), then green and a creep-away: exactly the
  stophold2pnw A2 behavior — held for STOP_CLEAR_HOLD_S of continuous shouldStop-clear, then
  released. No standstill hold arms (no lead was ever close); the latch does not outlive
  standstill; timers were never paused (the dwell built during the stop, so the exit is
  A2-timed, not dwell-blocked)."""
  sm = ConditionalExperimentalSwitching()
  red = sig(v_ego=0.0, model_should_stop=True, v_set=11.0)
  run(sm, red, 12.0)                                        # stopIntent fast-path entry + dwell
  assert sm.mode() == "experimental"
  # cesnochill2pnw: v_ego=1.4 is ABOVE NOCHILL_RELEASE_V(1.3) -- the new latch releases on the
  # very first green tick, handing off to the OLDER A2 machinery below (unmasked) to prove ITS
  # timing is intact underneath (a v=1.0 "rolling green" would still be inside the new latch's own
  # hold band and could never independently demonstrate A2's timer).
  green = sig(v_ego=1.4, v_set=11.0, spd_lim=25.0)          # rolling green: no lead, above the latch
  assert run(sm, green, 1.5) == "experimental"              # A2 window still holding
  assert sm.status() == "stopHold"                          # ...and it is A2, not the new machinery
  assert run(sm, green, 1.0) == "chill"                     # full 2 s clear: released as before


# --- (d) timers resume after standstill ---------------------------------------------------------

def test_dwell_expiry_works_again_at_speed_after_standstill_episode():
  """No frozen-timer leak: after a full standstill latch episode, a NORMAL at-speed cycle behaves
  exactly as shipped — entry waits the Chill cooldown, exit happens after EXP_MIN dwell + decay."""
  sm = ConditionalExperimentalSwitching()
  run(sm, ss_lead(), 15.0)                                  # standstill episode (latched)
  assert run(sm, rolling(6.0, drel=30.0), 2.0) == "chill"   # released cleanly (no lead hold: far)
  # normal at-speed entry: slowLead (closing on a much slower lead) — cooldown still enforced
  closing = sig(v_ego=20.0, has_lead=True, lead_vlead=10.0, lead_drel=60.0, v_set=25.0)
  assert run(sm, closing, 1.0) == "chill"                   # not instant: cooldown intact
  assert run(sm, closing, C.CHILL_MIN_DWELL_S + 2.0) == "experimental"   # entry lands mid-run
  # and the exit dwell expires normally at speed (the latch is standstill-only). The entry above
  # landed ~2.5 s into that run (cooldown counted from the exit tick), so ~4.5 s of dwell already
  # accumulated: 2 s more still holds (6.5 < EXP_MIN 8), 4 s beyond that expires (~10.5 > 8).
  cruise = sig(v_ego=25.0, v_set=25.0)
  assert run(sm, cruise, 2.0) == "experimental"             # dwell still holding
  assert run(sm, cruise, 4.0) == "chill"                    # then expires — no leak


# --- PROMOTE: no-cooldown standstill promotion --------------------------------------------------

def test_promotion_bypasses_cooldown_at_standstill():
  """Fresh Chill machine (dwell 0 — the 5 s cooldown blocks every normal entry) stopped behind a
  lead: promoted in ~PROMOTE_LEAD_S, not CHILL_MIN_DWELL_S + filter.

  cesnochill2pnw: superseded the old first assertion here. Pre-fix this read "chill" for the first
  ~0.3 s while the PROMOTE_LEAD_S debounce charged — a genuine chill tick at v_ego=0, exactly the
  field bug class (drives/2026-08-15/tesla-redlight-jolt) the hard latch now closes. The latch (pure
  v_ego predicate, no lead/debounce dependency) forces `experimental` from the very first tick
  instead, tagged "stopLatch" UNCONDITIONALLY for the whole armed episode (v_ego stays 0 throughout,
  so the latch never releases here — status no longer surfaces the underlying "slowLead" reason
  while armed; see the review note in ces_pnw_constants.py's cesnochill2pnw block)."""
  sm = ConditionalExperimentalSwitching()
  assert sm.update_decision(ss_lead(), DT) == "experimental"   # latched immediately (v_ego == 0)
  assert run(sm, ss_lead(), 0.5) == "experimental"
  assert sm.status() == "stopLatch"


def test_promotion_requires_sustained_lead_not_radar_ghost():
  """A flickering radar ghost at standstill (lead alternating every tick) must NOT spuriously
  promote VIA THE LEAD-EVIDENCE PATH — the debounce needs CONTINUOUS presence, and the normal
  path's filter never charges at 50% duty.

  cesnochill2pnw: mode() itself can no longer be "chill" here regardless — v_ego is 0.0 in BOTH
  ss_lead() and ss_dropout(), so the hard speed-only latch forces `experimental` on tick 1 (the
  standstill PROMOTE debounce never gets the chance to run: driver directive is never chill while
  stopped, full stop). Once latched, the state machine's own "already experimental" pass-through
  (update_decision_core: `if status != "chill": self._status = status`) legitimately re-tags every
  ss_lead() tick "slowLead" — that tag is real (decide_active does see a raw slowLead condition
  every other tick), just reached a different way (the latch, not the PROMOTE_LEAD_S debounce).
  The debounce-defeat property the old assertion protected is now: mode is NEVER chill, and dwell
  never reaches EXP_MIN_DWELL_S from this 50%-duty flicker alone (verified below) — i.e. nothing
  about the ghost's timing could independently sustain Experimental without the latch."""
  sm = ConditionalExperimentalSwitching()
  for i in range(int(8.0 / DT)):
    sm.update_decision(ss_lead() if i % 2 == 0 else ss_dropout(), DT)
  assert sm.mode() == "experimental"          # latched (v_ego == 0 throughout) -- never chill
  assert sm._dwell < C.EXP_MIN_DWELL_S        # the flicker itself never built real dwell evidence


def test_promotion_only_at_standstill():
  """Above the latch speed the cooldown stands exactly as shipped (no general fast path leaked)."""
  sm = ConditionalExperimentalSwitching()
  closing = sig(v_ego=2.0, has_lead=True, lead_vlead=0.5, lead_drel=15.0, v_set=15.2, spd_lim=25.0)
  assert run(sm, closing, 1.5) == "chill"                   # slowLead raw-active, but v >= latch


# --- precedence: latch/hold vs pullaway ---------------------------------------------------------

def test_pullaway_deferred_while_release_hold_then_proceeds():
  """DOCUMENTED PRECEDENCE: at standstill the latch always wins (pullaway is dead below its own
  2.0 m/s floor anyway). During the release hold, a lead genuinely pulling away flips the decision
  to (False, 'pullAway') — but the hold keeps Experimental while the gap is still short; the
  moment it opens past CLEAR_DREL the hold disarms and the pullaway Chill adoption proceeds
  exactly as shipped. Pullaway matters once moving; the latch owns the standstill."""
  sm = ConditionalExperimentalSwitching()
  run(sm, ss_lead(drel=12.0), 12.0)
  assert sm.mode() == "experimental"
  # launch, lead opening (evidence-complete pullaway geometry) but gap still short: HELD
  pull_near = sig(v_ego=3.0, has_lead=True, lead_vlead=5.5, lead_drel=15.0, v_set=15.2,
                  lead_opening=True)
  assert run(sm, pull_near, 1.5) == "experimental"
  assert sm.status() == "standstillHold"
  # gap opens past CLEAR_DREL at the same speed: hold disarms, pullaway Chill adoption proceeds
  pull_open = sig(v_ego=3.0, has_lead=True, lead_vlead=5.5, lead_drel=27.0, v_set=15.2,
                  lead_opening=True)
  assert run(sm, pull_open, 1.0) == "chill"


# --- safety edges (the Gemini review targets) ---------------------------------------------------

def test_latch_is_pure_speed_predicate_never_sticks_at_speed():
  """The latch must release on ANY v_ego above the threshold — it is a per-tick predicate on live
  signals, not a stored state. Jump straight from a latched standstill to highway speed: the very
  first exit-eligible evaluation demotes (no residual latch/hold at cruise)."""
  sm = ConditionalExperimentalSwitching()
  run(sm, ss_dropout(), 2.0)   # nothing active
  run(sm, ss_lead(), 20.0)     # long latched standstill (dwell >> EXP_MIN, filter cycles)
  assert sm.mode() == "experimental"
  # teleport to open-road cruise (worst case for a stuck latch): released within the filter decay
  assert run(sm, sig(v_ego=25.0, v_set=25.0), 1.0) == "chill"


def test_latch_boundary():
  """Strictly below STANDSTILL_LATCH_V: the OLDER standstill2pnw latch itself would hold (still
  true underneath), but it's now masked by the cesnochill2pnw latch (armed the whole time here,
  since v never exceeds its own, higher, NOCHILL_RELEASE_V) -- status reads "stopLatch". At/above
  NOCHILL_RELEASE_V (moving for real): both latches release, and A2/the release hold may still hold
  — isolated here with an open gap and a cleared A2 timer, same as before."""
  below = rolling(C.STANDSTILL_LATCH_V - 0.01, drel=30.0, vlead=1.5)   # vlead > STOPPED_LEAD_V so
  at = rolling(C.STANDSTILL_LATCH_V + 1.2, drel=30.0)   # slowLead stays raw-inactive; `at` is
                                                        # above A2's STANDSTILL_HOLD_V too
  sm = ConditionalExperimentalSwitching()
  run(sm, ss_lead(drel=30.0), 12.0)
  assert run(sm, below, 3.0) == "experimental"          # latched (v < cesnochill2pnw's RELEASE_V)
  assert sm.status() == "stopLatch"                     # cesnochill2pnw: unconditional while armed
  sm2 = ConditionalExperimentalSwitching()
  run(sm2, ss_lead(drel=30.0), 12.0)
  assert run(sm2, at, 1.5) == "chill"                   # not latched, no hold (gap open), A2 clear
