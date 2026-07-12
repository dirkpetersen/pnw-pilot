"""icbmrestore2pnw — guarded restore episode tests (pure IcbmEpisode + _icbm_step plumbing).

The restore may ONLY return the stock set to the driver's OWN latched ceiling, and any sign of
driver intent kills the episode entirely. Caps remain DEC-only; DEC always wins over INC.
"""
import math

from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import (IcbmEpisode, ICBM_RESTORE_WINDOW_S,
                                                              ICBM_RESTORE_DELAY_S, ICBM_EXEC_STEP_MS)

MPH = 0.44704
T0 = 100.0


def _to_restore(ep, ceiling=60 * MPH, target=40 * MPH, stock=None, t=T0):
  """Drive an episode: cap engages (latch ceiling) -> sustained clear -> RESTORE. Returns the time
  at which the restore began."""
  out = ep.step(t, target, ceiling, ceiling, True, False)                    # cap engages at set
  assert out == (target, "dec") and ep.phase == "cap" and math.isclose(ep.ceiling, ceiling)
  stock = target if stock is None else stock                                 # taps brought set down
  t += 1.0
  assert ep.step(t, None, stock, stock, True, False) == (None, None)         # clear tick 1: debounce
  assert ep.phase == "cap" and ep.ceiling is not None                        # ...ceiling retained
  t += ICBM_RESTORE_DELAY_S + 0.1
  tgt, d = ep.step(t, None, stock, stock, True, False)                       # sustained clear
  assert d == "inc" and math.isclose(tgt, ceiling) and ep.phase == "restore"
  return t


def test_full_lifecycle_cap_clear_restore_done():
  ep = IcbmEpisode()
  t = _to_restore(ep)
  # stock climbs under our taps; restore keeps publishing the ceiling
  # +2 mph per second stays inside the executor tap-cadence envelope (max ~2.5 mph/s)
  for i, stock_mph in enumerate((41, 43, 45, 47, 49)):
    tgt, d = ep.step(t + 1.0 + i, None, stock_mph * MPH, stock_mph * MPH, True, False)
    assert d == "inc" and math.isclose(tgt, 60 * MPH)                        # never above the ceiling
  # reaching the ceiling ends the episode (unlatched, silent)
  assert ep.step(t + 10.0, None, 59.9 * MPH, 59.9 * MPH, True, False) == (None, None)
  assert ep.phase == "idle" and ep.ceiling is None


def test_clear_flicker_keeps_original_ceiling():
  ep = IcbmEpisode()
  ep.step(T0, 40 * MPH, 60 * MPH, 60 * MPH, True, False)                     # cap, ceiling 60
  assert ep.step(T0 + 1, None, 40 * MPH, 40 * MPH, True, False) == (None, None)   # 1-tick dropout
  # curve re-binds INSIDE the debounce window: still the SAME episode, ORIGINAL ceiling
  tgt, d = ep.step(T0 + 2, 38 * MPH, 40 * MPH, 40 * MPH, True, False)
  assert d == "dec" and ep.phase == "cap" and math.isclose(ep.ceiling, 60 * MPH)


def test_abort_matrix_during_restore():
  # each abort -> full reset (idle, unlatched, silent)
  for name, args in {
    "gas/brake":    dict(stock=45 * MPH, on=True, pedal=True, dt=1.0),
    "acc_off":      dict(stock=45 * MPH, on=False, pedal=False, dt=1.0),
    "window":       dict(stock=45 * MPH, on=True, pedal=False, dt=ICBM_RESTORE_WINDOW_S + 1.0),
  }.items():
    ep = IcbmEpisode()
    t = _to_restore(ep)
    out = ep.step(t + args["dt"], None, args["stock"], args["stock"], args["on"], args["pedal"])
    assert out == (None, None) and ep.phase == "idle" and ep.ceiling is None, name


def test_abort_on_manual_set_decrease():
  ep = IcbmEpisode()
  t = _to_restore(ep)                                                        # restoring, stock 40
  # driver presses SET-: set drops 1 mph -> only a human does that now -> abort
  out = ep.step(t + 1.0, None, 39 * MPH, 39 * MPH, True, False)
  assert out == (None, None) and ep.phase == "idle"


def test_abort_on_faster_rise_than_our_taps():
  ep = IcbmEpisode()
  t = _to_restore(ep)                                                        # restoring, stock 40
  # +5 mph in 0.25 s = a driver SET+ hold (Ford hold steps 5) -> abort
  out = ep.step(t + 0.25, None, 45 * MPH, 45 * MPH, True, False)
  assert out == (None, None) and ep.phase == "idle"


def test_own_cadence_rise_is_tolerated():
  ep = IcbmEpisode()
  t = _to_restore(ep)
  # ~1 mph per 0.5 s = our own executor cadence -> keeps restoring
  stock = 40 * MPH
  for i in range(5):
    stock += ICBM_EXEC_STEP_MS
    tgt, d = ep.step(t + 0.5 * (i + 1), None, stock, stock, True, False)
    assert d == "inc"


def test_dec_beats_inc_new_cap_cancels_restore_and_relatches_lower():
  ep = IcbmEpisode()
  t = _to_restore(ep)                                                        # restoring toward 60
  # new curve binds while the set is still at 45: DEC WINS; the restore episode dies and the NEW
  # episode latches at the CURRENT set (45) — never the old 60
  tgt, d = ep.step(t + 1.0, 35 * MPH, 45 * MPH, 45 * MPH, True, False)
  assert d == "dec" and math.isclose(tgt, 35 * MPH)
  assert ep.phase == "cap" and math.isclose(ep.ceiling, 45 * MPH)


def test_no_restore_when_driver_lowered_below_our_target():
  ep = IcbmEpisode()
  ep.step(T0, 40 * MPH, 60 * MPH, 60 * MPH, True, False)                     # we commanded 40
  # driver went to 30 themselves (below our lowest target by >> tolerance)
  ep.step(T0 + 1, None, 30 * MPH, 30 * MPH, True, False)
  out = ep.step(T0 + 1 + ICBM_RESTORE_DELAY_S + 0.1, None, 30 * MPH, 30 * MPH, True, False)
  assert out == (None, None) and ep.phase == "idle" and ep.ceiling is None   # their intent: no restore


def test_no_restore_without_a_cap_episode():
  ep = IcbmEpisode()
  # stock sits below the set (driver lowered it long ago) but ICBM never capped: stay idle forever
  for i in range(20):
    assert ep.step(T0 + i, None, 45 * MPH, 45 * MPH, True, False) == (None, None)
  assert ep.phase == "idle" and ep.ceiling is None


def test_no_restore_when_already_at_ceiling():
  ep = IcbmEpisode()
  ep.step(T0, 59.5 * MPH, 60 * MPH, 60 * MPH, True, False)                   # tiny cap
  ep.step(T0 + 1, None, 59.8 * MPH, 59.8 * MPH, True, False)
  out = ep.step(T0 + 1 + ICBM_RESTORE_DELAY_S + 0.1, None, 59.8 * MPH, 59.8 * MPH, True, False)
  assert out == (None, None) and ep.phase == "idle"                          # nothing worth restoring


def test_invalid_set_speed_resets_everything():
  ep = IcbmEpisode()
  t = _to_restore(ep)
  assert ep.step(t + 1.0, None, 41 * MPH, 41 * MPH, True, False)[1] == "inc"
  assert ep.step(t + 2.0, None, 0.0, 41 * MPH, True, False) == (None, None)  # ACC set vanished
  assert ep.phase == "idle"


def test_restore_requires_acc_on_at_entry():
  ep = IcbmEpisode()
  ep.step(T0, 40 * MPH, 60 * MPH, 60 * MPH, True, False)
  ep.step(T0 + 1, None, 40 * MPH, 40 * MPH, True, False)
  # ACC off at the moment the clear sustains -> no restore at all
  out = ep.step(T0 + 1 + ICBM_RESTORE_DELAY_S + 0.1, None, 40 * MPH, 40 * MPH, False, False)
  assert out == (None, None) and ep.phase == "idle"


# ---- _icbm_step plumbing: the "dir": "inc" marker + restore telemetry -----------------------------
def test_icbm_step_publishes_inc_marker_and_restore_src(tmp_path, monkeypatch):
  import inspect
  import time as _t
  from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
  from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
  from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle

  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "nope.json"))
  cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_icbm_step"))

  class FakeMem:
    def put_nonblocking(self, k, v): self.last = v

  class FakeCP:
    carFingerprint = "FORD_F_150_LIGHTNING_MK1"
    brand = "ford"
    openpilotLongitudinalControl = False

  class Stub:
    pass
  mgr = Stub()
  mgr.mem_params = FakeMem()
  mgr._icbm_ceiling = None
  mgr._veh = PnwVehicle(FakeCP())
  mgr._map_targets = []
  mgr._cur_lat = None
  mgr._cur_lon = None
  mgr._icbm_ep = m.IcbmEpisode(clear_delay_s=0.0)   # no debounce wait in the unit test
  mgr._icbm_dir = None
  step = cls._icbm_step.__get__(mgr)

  def run(sig, stock_set, stock_on):
    mgr._stock_set, mgr._stock_on = stock_set, stock_on
    mgr._icbm_last_pub = _t.monotonic() - 1.0
    step(sig, active=True)
    return mgr.mem_params.last

  cap_sig = {"v_ego": 60 * MPH, "v_set": 60 * MPH, "map_target_v": 30 * MPH, "map_target_dist": 40.0,
             "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0, "gas": False, "brake": False}
  clear_sig = {"v_ego": 45 * MPH, "v_set": 45 * MPH, "map_target_v": 0.0, "map_target_dist": float("inf"),
               "curve_lat_accel_vision": 0.0, "time_to_curve": 10.0, "gas": False, "brake": False}

  # 1) cap: a plain dict WITHOUT "dir" (wire-compatible with the pre-restore executor)
  out = run(cap_sig, 60 * MPH, True)
  assert out.get("target") is not None and "dir" not in out and mgr._icbm_dir == "dec"
  # 2) curve cleared, stock tapped down to 45: restore -> "dir": "inc", target == latched ceiling 60
  out = run(clear_sig, 45 * MPH, True)
  assert out.get("dir") == "inc" and abs(out["target"] - round(60 * MPH, 2)) < 0.01
  assert mgr._icbm_src == "restore" and mgr._icbm_dir == "inc"
  # 3) driver touches the gas: hard abort -> empty publish, episode dead
  out = run({**clear_sig, "gas": True}, 45 * MPH, True)
  assert out == {} and mgr._icbm_ep.phase == "idle" and mgr._icbm_dir is None


# ---- Gemini adversarial-review fixes (2026-07-12): the three intent-conflict guards ---------------
def test_acc_off_during_cap_kills_episode_no_blast_restore():
  """Gemini scenario: cap 75->45, driver brakes (ACC off), re-engages at 55. The old ceiling (75)
  must be GONE — the fresh episode latches at 55, so a restore can only ever go back to 55."""
  ep = IcbmEpisode()
  ep.step(T0, 45 * MPH, 75 * MPH, 75 * MPH, True, False)             # cap, ceiling 75
  # driver brakes -> ACC off: episode dies (cap still forwarded, but no episode/latch)
  out = ep.step(T0 + 1, 45 * MPH, 75 * MPH, 0.0, False, True)
  assert out == (45 * MPH, "dec") and ep.phase == "idle" and ep.ceiling is None
  # re-engaged at 55, same curve still binding: FRESH episode latched at 55
  ep.step(T0 + 2, 45 * MPH, 55 * MPH, 55 * MPH, True, False)
  assert math.isclose(ep.ceiling, 55 * MPH)
  # clear + hold + restore: target is 55 — NEVER the old 75
  ep.step(T0 + 3, None, 55 * MPH, 45 * MPH, True, False)
  tgt, d = ep.step(T0 + 3 + ICBM_RESTORE_DELAY_S + 0.1, None, 55 * MPH, 45 * MPH, True, False)
  assert d == "inc" and math.isclose(tgt, 55 * MPH)


def test_pedal_during_cap_kills_episode():
  ep = IcbmEpisode()
  ep.step(T0, 40 * MPH, 60 * MPH, 60 * MPH, True, False)
  out = ep.step(T0 + 1, 40 * MPH, 60 * MPH, 55 * MPH, True, True)    # gas blip mid-cap
  assert out == (40 * MPH, "dec") and ep.phase == "idle" and ep.ceiling is None


def test_set_movement_during_hold_window_blocks_restore():
  """Gemini scenario (the 3 s blind spot): the brain is silent through the hold, so ANY set
  movement there is a human choosing a speed -> no restore at all."""
  for moved_mph in (39.0, 41.5, 44.0):                               # down, up-a-bit, up-a-lot
    ep = IcbmEpisode()
    ep.step(T0, 40 * MPH, 60 * MPH, 60 * MPH, True, False)
    ep.step(T0 + 1, None, 60 * MPH, 40 * MPH, True, False)           # hold entry: snapshot 40
    out = ep.step(T0 + 1 + ICBM_RESTORE_DELAY_S + 0.1, None, 60 * MPH, moved_mph * MPH, True, False)
    assert out == (None, None) and ep.phase == "idle", moved_mph


def test_driver_two_taps_below_target_blocks_restore():
  """Driver-lower guard, latency-tolerant edition (icbmmapfirst2pnw): the tolerance now absorbs the
  executor's own worst-case self-overshoot (DEADBAND 0.6 + ONE report-latency tap = 1.6 steps — the
  2026-07-12 18:08:26Z field false positive that killed a wanted restore), so a single ambiguous tap
  restores (documented residual, ceiling-bounded). TWO deliberate SET- taps (>= 2 steps below the
  lowest commanded target) are beyond anything the executor can self-inflict -> still no restore."""
  ep = IcbmEpisode()
  ep.step(T0, 40 * MPH, 60 * MPH, 60 * MPH, True, False)             # min_target 40
  ep.step(T0 + 1, None, 60 * MPH, 38 * MPH, True, False)             # driver tapped twice to 38
  out = ep.step(T0 + 1 + ICBM_RESTORE_DELAY_S + 0.1, None, 60 * MPH, 38 * MPH, True, False)
  assert out == (None, None) and ep.phase == "idle"


def test_executor_overshoot_below_target_still_restores():
  """The flip side: the executor's own <=0.4-step overshoot must NOT be mistaken for a driver."""
  ep = IcbmEpisode()
  ep.step(T0, 40 * MPH, 60 * MPH, 60 * MPH, True, False)
  stock = 40 * MPH - 0.3 * ICBM_EXEC_STEP_MS                         # legit executor end-state
  ep.step(T0 + 1, None, 60 * MPH, stock, True, False)
  tgt, d = ep.step(T0 + 1 + ICBM_RESTORE_DELAY_S + 0.1, None, 60 * MPH, stock, True, False)
  assert d == "inc" and math.isclose(tgt, 60 * MPH)


def test_latency_tap_below_target_still_restores_field_18_08():
  """THE 2026-07-12 18:08:26Z field case, replayed with the real numbers: vis episode, targets
  81.1/78.9/77.3/77.8 mph, taps walked the set 85->77, then the FINAL in-flight tap landed at 76
  AFTER the brain went silent (reported-set lag). min_target 77.3, set 76.0 = 1.3 steps below.
  The old 0.6-step tolerance read that as a driver SET- and never restored (driver sat at 76 vs
  ceiling 85 for 33 s and had to gas + SET+ manually). Now: the late tap is absorbed and the
  restore publishes the latched 85 ceiling."""
  ep = IcbmEpisode()
  t = T0
  for tgt_mph, set_mph in ((81.1, 85.0), (78.9, 82.0), (77.3, 80.0), (77.8, 77.0)):
    out = ep.step(t, tgt_mph * MPH, 85 * MPH, set_mph * MPH, True, False)
    assert out[1] == "dec"
    t += 1.0
  ep.step(t, None, 76 * MPH, 77 * MPH, True, False)                  # clear tick: snapshot 77
  ep.step(t + 0.5, None, 76 * MPH, 76 * MPH, True, False)            # late tap lands (within grace)
  tgt, d = ep.step(t + ICBM_RESTORE_DELAY_S + 0.1, None, 76 * MPH, 76 * MPH, True, False)
  assert d == "inc" and math.isclose(tgt, 85 * MPH), (tgt, d, ep.phase)
