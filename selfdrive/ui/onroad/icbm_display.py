"""uimax2pnw -- the comma screen's MAX SPEED keeps the DRIVER's max during an ICBM slowdown.

On the Lightning, ICBM steers the truck's stock-ACC set speed down for a curve (and back up after). The HUD
number is carState.vCruiseCluster, so the screen used to follow the truck's dash down and the driver's own max
vanished for the length of every curve. Owner 2026-09-29: the SCREEN keeps the driver's max; only the truck dash
changes. DISPLAY ONLY -- hudControl.setSpeed (controlsd) and everything on the control path are untouched.

The driver's max is IcbmEpisode.ceiling, published at 5 Hz in the CESStatus mem-param as `icbmC` (m/s), with the
episode `icbmPhase` and the restore bound `icbmRCap` (m/s, 0 = none; lowers the effective max when the speed
limit dropped mid-episode, exactly as the restore itself is bounded). Gas, brake and ACC-off end the episode in ces_pnw
(ceiling cleared, phase idle / gas), and a driver SET+/- ends it during the RESTORE phase. During the CAP phase
ces_pnw does NOT judge a set change (it keeps phase=cap and the ceiling while the curve binds), so the chooser
does: the ceiling is shown only while the cluster set is explainable by ICBM's own taps, i.e. not more than
SET_TAP_TOL_KPH below the published target icbmT (the set only ever sits AT or ABOVE the target while ICBM taps it
down). A set below that = the driver lowered it -> the screen follows the cluster. icbmC is published in the
CESStatus shadow block of ces_pnw._publish_status (one line added for this feature).

Pure chooser + a small poller. No raylib / ui_state import here, so the tests run in any tree.
"""
import math
import time

from openpilot.common.swaglog import cloudlog

MS_TO_KPH = 3.6
STALE_S = 4.0            # s: CESStatus `ts` older than this => publisher silent; never trust a stale ceiling
# Mirror of ces_pnw.ICBM_DRIVER_LOWER_TOL (1.7 executor taps). NOT imported: ces_pnw is the whole controller and the UI
# process must not load it; test_driver_lower_tol_matches_ces_pnw pins the two together.
DRIVER_LOWER_TOL_MS = 1.7 * 0.44704   # cap phase -- a cluster set this far BELOW the lowest icbmT was lowered by the driver
GAS_HOLD_S = 2.0         # s: a gas phase shorter than this does not flip the shown max (60->40->60 flicker / mici flash)
CLUSTER_TOL_KPH = 2.0    # kph: cluster set is a rounded display value; a ceiling this far BELOW it is a stale latch
POLL_S = 0.2             # s: CESStatus is published at 5 Hz
_EPISODE_PHASES = ("cap", "restore")   # "gas" (driver pedal suspends the episode) deliberately shows the cluster set


def _num(v) -> float | None:
  """A finite float, or None. Bools, strings, NaN and inf are all 'not a number' here."""
  if isinstance(v, bool):
    return None
  try:
    f = float(v)
  except (TypeError, ValueError):
    return None
  return f if math.isfinite(f) else None


def max_speed_display(cluster_kph: float, ces_status, now: float, capable: bool,
                      ref_target_ms: float | None = None) -> tuple[float, str | None]:
  """(kph to show as MAX SPEED, fallback_reason). reason is None for the two normal cases (not capable / not in
  an episode: cluster unchanged) and for a healthy override; any other string is a fallback worth logging.
  `now` is wall-clock (CESStatus `ts` is wall-clock). `ref_target_ms`: the last icbmT seen this episode, used in the
  CAP phase while icbmT is briefly None (ICBM silent during the clear debounce)."""
  if not capable:
    return cluster_kph, None
  if not isinstance(ces_status, dict):
    return cluster_kph, "CESStatus unreadable"
  phase = ces_status.get("icbmPhase")
  if phase not in _EPISODE_PHASES:
    return cluster_kph, None                     # idle / gas / absent: the normal case, not logged
  ts = _num(ces_status.get("ts"))
  if ts is None or now - ts > STALE_S or ts - now > STALE_S:
    return cluster_kph, f"episode {phase} but CESStatus ts stale/invalid ({ces_status.get('ts')!r})"
  ceil_ms = _num(ces_status.get("icbmC"))
  if ceil_ms is None or ceil_ms <= 0.0:
    return cluster_kph, f"episode {phase} but icbmC invalid ({ces_status.get('icbmC')!r})"
  rcap_ms = _num(ces_status.get("icbmRCap"))
  if rcap_ms is not None and 0.0 < rcap_ms < ceil_ms:
    ceil_ms = rcap_ms                            # limit dropped mid-episode: the driver-effective max is the bound
  ceil_kph = ceil_ms * MS_TO_KPH
  if phase == "cap":
    # The published target can RISE during a cap (lead pace, floor, source switch) but the executor only taps DOWN,
    # so the truck set sits at the LOWEST target reached: compare against the minimum, never the latest.
    cands = [t for t in (_num(ces_status.get("icbmT")), ref_target_ms) if t is not None]
    tgt_ms = min(cands) if cands else None
    if tgt_ms is None:
      return cluster_kph, "episode cap but no ICBM target seen (icbmT None) -- cannot tell ICBM's set from the driver's"
    if cluster_kph < tgt_ms * MS_TO_KPH - DRIVER_LOWER_TOL_MS * MS_TO_KPH:
      return cluster_kph, (f"episode cap but cluster set {cluster_kph:.1f} kph is below ICBM's target "  # noqa: ISC002
                           f"{tgt_ms * MS_TO_KPH:.1f} -- driver lowered the set")
  if ceil_kph < cluster_kph - CLUSTER_TOL_KPH:
    return cluster_kph, None                     # normal (a zone cap below the set, a SET+ past the latch): not log noise
  return ceil_kph, None


class IcbmMaxDisplay:
  """Polls CESStatus (<= 5 Hz) and applies max_speed_display; logs each fallback reason once per change."""

  def __init__(self, mem=None, clock=time.monotonic, wall=time.time):  # noqa: TID251 -- wall clock: CESStatus `ts` is wall-clock
    self._mem = mem
    self._clock, self._wall = clock, wall
    self._st = None
    self._last_poll = -1e9
    self._last_reason: str | None = None
    self._read_err: str | None = None
    self._ref_target: float | None = None    # last icbmT (m/s) seen in the current cap phase
    self._held: float | None = None          # last override shown (kph), for the gas hold-off
    self._held_t = -1e9
    if self._mem is None:
      try:
        from openpilot.common.params import Params
        self._mem = Params("/dev/shm/params")
      except Exception:
        cloudlog.exception("uimax2pnw: /dev/shm/params unavailable -- the screen MAX SPEED falls back to the truck set speed")
        self._mem = None

  def _poll(self) -> None:
    t = self._clock()
    if t - self._last_poll < POLL_S:
      return
    self._last_poll = t
    if self._mem is None:
      self._st = None
      return
    try:
      st = self._mem.get("CESStatus", return_default=True)
      self._st = st if isinstance(st, dict) else {}   # not published (CES off) is NOT an error: no episode
      self._read_err = None
    except Exception as e:
      self._st = None
      err = f"CESStatus read failed: {type(e).__name__}: {e}"
      if err != self._read_err:
        self._read_err = err
        cloudlog.error(f"uimax2pnw: {err} -- the screen MAX SPEED falls back to the truck set speed")

  def choose(self, cluster_kph: float, capable: bool) -> float:
    if not capable:
      return cluster_kph
    self._poll()
    st = self._st if isinstance(self._st, dict) else {}
    phase = st.get("icbmPhase")
    t = _num(st.get("icbmT"))
    if phase == "cap" and t is not None:      # NOT gas: a target forwarded during gas is never tapped by the executor
      self._ref_target = t if self._ref_target is None else min(self._ref_target, t)   # lowest target this episode
    elif phase in ("idle", None, "restore"):  # restore -> cap is always a NEW episode in ces_pnw (reset() clears its min)
      self._ref_target = None
      if phase != "restore":
        self._held = None
    value, reason = max_speed_display(cluster_kph, self._st, self._wall(), capable, self._ref_target)
    now = self._clock()
    if reason is None and value != cluster_kph:
      self._held, self._held_t = value, now
    elif reason is None and phase == "gas" and self._held is not None and now - self._held_t < GAS_HOLD_S:
      value = self._held                        # a short gas press must not flip the shown max
    if reason != self._last_reason:
      self._last_reason = reason
      if reason is not None and self._read_err is None:
        cloudlog.warning(f"uimax2pnw: {reason} -- the screen MAX SPEED falls back to the truck set speed")
    return value
