"""stopgo2pnw closed-loop sim (from the Opus review of ccbac3ad98).
REAL LongitudinalPlanner (Tesla HW3 CP, real MPC + real stopgo gate) + replicated LongControl (Tesla params)
+ first-order actuator + CES-like Experimental latch. e2e model is a scripted stub (shouldStop = v<0.3 and acc<0.1)."""

import collections
import numpy as np
from cereal import log, car
import cereal.messaging as messaging
from openpilot.common.realtime import DT_MDL, DT_CTRL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.values import CAR


class SM(dict):
  @property
  def alive(self):
    return collections.defaultdict(bool, dict.fromkeys(self, True))

  @property
  def valid(self):
    return collections.defaultdict(bool, dict.fromkeys(self, True))


def make_cp():
  CP = CarInterface.get_non_essential_params(CAR.TESLA_MODEL_S_HW3)
  return CP


def run(scn, handoff_on=True, T=30.0, exp_mode='ces', seed=0, verbose=False):
  rng = np.random.default_rng(seed)
  CP = make_cp()
  pl = LongitudinalPlanner(CP, init_v=0.0)
  pl._stopgo_mem = None
  pl.veh._tesla_curve_cfg = dict(pl.veh._tesla_curve_cfg, stop_go=handoff_on)
  pl.veh._tesla_cfg_poll = 1e18  # no reloads
  assert pl.veh.stop_go_handoff == handoff_on, (pl.veh.stop_go_handoff, pl.veh.curve_brain_vtsc)
  lc = LongControl(CP)
  v = 0.0
  x = 0.0
  a_act = 0.0
  cmd_hist = collections.deque([0.0] * 15, maxlen=15)  # 0.15 s pure delay at 100 Hz
  armed = True
  ss_hold = False
  from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import ConditionalExperimentalSwitching

  ces = ConditionalExperimentalSwitching()
  ces_mode = 'experimental'
  ces_status = ''
  last_e2e = 0.02
  plan_a, plan_stop = 0.0, True
  out = []
  n = int(T / DT_CTRL)
  hand_t = 0.0
  first_hand = None
  for i in range(n):
    t = i * DT_CTRL
    L = scn['lead'](t, x)  # dict: x_lead (abs pos), v_lead, present
    d_true = L['x'] - x
    if exp_mode == 'real':
      sig = {
        "v_ego": float(v),
        "has_lead": bool(L.get('present', True)),
        "lead_vlead": float(L['v']),
        "lead_drel": float(d_true),
        "blinker": False,
        "map_target_v": 0.0,
        "map_target_dist": float('inf'),
        "curve_lat_accel_vision": 0.0,
        "time_to_curve": 10.0,
        "model_should_stop": bool(v < 0.3 and last_e2e < 0.1),
        "v_set": 12.5,
        "spd_lim": 0.0,
        "a_ego": float(a_act),
        "gas": False,
        "toggles": {"curves": True, "stops": True, "low_speed": True, "lead": True},
      }
      ces_mode = ces.update_decision(sig, DT_CTRL)
      ces_status = ces.status()
    if i % 5 == 0:  # planner 20 Hz
      # CES-like experimental: nochill latch + standstill hold
      if armed and v > 1.3:
        armed = False
      elif not armed and v < 1.0:
        armed = True
      if v < 0.5 and L.get('present', True) and d_true <= 20:
        ss_hold = True
      if ss_hold and (v > 5.0 or d_true > 25):
        ss_hold = False
      exp = True if exp_mode == 'always' else ((ces_mode == 'experimental') if exp_mode == 'real' else (armed or ss_hold))
      radar = messaging.new_message('radarState')
      lead = radar.radarState.leadOne
      present = L.get('present', True)
      noise_d = scn.get('d_noise', 0.0) * rng.standard_normal()
      noise_v = scn.get('v_noise', 0.0) * rng.standard_normal() + scn.get('v_bias', 0.0)
      lead.dRel = float(max(0.0, d_true + noise_d))
      lead.yRel = 0.0
      lead.vLead = float(L['v'] + noise_v)
      lead.vLeadK = lead.vLead
      lead.vRel = float(lead.vLead - v)
      lead.aLeadK = 0.0
      lead.aLeadTau = 1.5
      lead.status = bool(present)
      lead.modelProb = float(L.get('prob', 0.99))
      lead.radar = bool(L.get('radar', True))
      cs = messaging.new_message('carState').carState
      cs.vEgo = float(v)
      cs.aEgo = float(a_act)
      cs.standstill = bool(v < 0.01)
      cs.vCruise = float(50 * 3.6)
      cs.brakePressed = False
      ctl = messaging.new_message('controlsState').controlsState
      ctl.longControlState = LongCtrlState.pid
      ctl.forceDecel = False
      sds = messaging.new_message('selfdriveState').selfdriveState
      sds.experimentalMode = exp
      sds.enabled = True
      sds.personality = log.LongitudinalPersonality.standard
      cc = messaging.new_message('carControl').carControl
      cc.orientationNED = [0.0, 0.0, 0.0]
      cc.longActive = True
      lp = messaging.new_message('liveParameters').liveParameters
      m = messaging.new_message('modelV2').modelV2
      e2e_acc = scn['e2e_acc'](t, v, d_true)
      last_e2e = e2e_acc
      m.action.desiredAcceleration = float(e2e_acc)
      m.action.shouldStop = bool(v < 0.3 and e2e_acc < 0.1)
      pos = log.XYZTData.new_message()
      pos.x = [float(q) for q in (v + 0.5) * np.array(ModelConstants.T_IDXS)]
      m.position = pos
      vel = log.XYZTData.new_message()
      vel.x = [float(v + 0.5)] * len(ModelConstants.T_IDXS)
      m.velocity = vel
      acc = log.XYZTData.new_message()
      acc.x = [0.0] * len(ModelConstants.T_IDXS)
      m.acceleration = acc
      m.meta.disengagePredictions.gasPressProbs = [1.0] * 6
      sm = SM(
        {'radarState': radar.radarState, 'carState': cs, 'carControl': cc, 'controlsState': ctl, 'selfdriveState': sds, 'liveParameters': lp, 'modelV2': m}
      )
      pl.update(sm)
      plan_a, plan_stop = float(pl.output_a_target), bool(pl.output_should_stop)
      hand = pl.stopgo.active
      if hand:
        hand_t += DT_MDL
        if first_hand is None:
          first_hand = t
      out.append((t, v, x, d_true, plan_a, plan_stop, hand, pl.stopgo.why, exp, a_act, lc.long_control_state, ces_status))
    CSc = car.CarState.new_message(vEgo=float(v), aEgo=float(a_act), brakePressed=False)
    CSc.cruiseState.standstill = False
    cmd = lc.update(True, CSc, plan_a, plan_stop, [-3.5, 2.0])
    cmd_hist.append(cmd)
    a_act += (cmd_hist[0] - a_act) * DT_CTRL / 0.25
    if v <= 0 and a_act < 0:
      a_act_eff = 0.0
    else:
      a_act_eff = a_act
    v = max(0.0, v + a_act_eff * DT_CTRL)
    x += v * DT_CTRL
  return out, dict(hand_s=round(hand_t, 2), first_hand=first_hand)


def metrics(out, meta):
  ts = np.array([o[0] for o in out])
  vs = np.array([o[1] for o in out])
  xs = np.array([o[2] for o in out])
  ds = np.array([o[3] for o in out])
  hs = np.array([o[6] for o in out])
  mv = np.where(vs > 0.05)[0]
  return dict(
    t_move=ts[mv[0]] if len(mv) else None,
    min_gap=float(ds.min()),
    final_gap=float(ds[-1]),
    x_end=float(xs[-1]),
    vmax=float(vs.max()),
    hand_edges=int(np.sum(hs[1:] & ~hs[:-1])),
    hand_s=meta['hand_s'],
    first_hand=meta['first_hand'],
  )


def lead_profile(segments, d0):
  """segments: list of (t_end, accel); lead starts at rest at d0."""
  state = {'t': 0.0, 'x': d0, 'v': 0.0}

  def f(t, ego_x):
    while state['t'] < t - 1e-9:
      dt = min(DT_CTRL, t - state['t'])
      a = 0.0
      for te, ac in segments:
        if state['t'] < te:
          a = ac
          break
      state['v'] = max(0.0, state['v'] + a * dt)
      state['x'] += state['v'] * dt
      state['t'] += dt
    return {'x': state['x'], 'v': state['v']}

  return f
