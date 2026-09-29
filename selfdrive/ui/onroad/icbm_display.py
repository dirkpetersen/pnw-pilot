"""uimax2pnw -- the comma screen's MAX SPEED keeps the DRIVER's max during an ICBM slowdown.

On the Lightning, ICBM steers the truck's stock-ACC set speed down for a curve (and back up after). The HUD
number is carState.vCruiseCluster, so the screen used to follow the truck's dash down and the driver's own max
vanished for the length of every curve. Owner 2026-09-29: the SCREEN keeps the driver's max; only the truck dash
changes. DISPLAY ONLY -- hudControl.setSpeed (controlsd) and everything on the control path are untouched.

The driver's max is IcbmEpisode.ceiling, published at 5 Hz in the CESStatus mem-param as `icbmC` (m/s), with the
episode `icbmPhase` and the restore bound `icbmRCap` (m/s, 0 = none; lowers the effective max when the speed
limit dropped mid-episode, exactly as the restore itself is bounded). Driver SET+/-, gas, brake and ACC-off end the
episode in ces_pnw (ceiling cleared, phase idle), so this module has no second notion of "driver intent".

Pure chooser + a small poller. No raylib / ui_state import here, so the tests run in any tree.
"""
import math
import time

from openpilot.common.swaglog import cloudlog

MS_TO_KPH = 3.6
STALE_S = 4.0            # s: CESStatus `ts` older than this => publisher silent; never trust a stale ceiling
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


def max_speed_display(cluster_kph: float, ces_status, now: float, capable: bool) -> tuple[float, str | None]:
  """(kph to show as MAX SPEED, fallback_reason). reason is None for the two normal cases (not capable / not in
  an episode: cluster unchanged) and for a healthy override; any other string is a fallback worth logging.
  `now` is wall-clock (CESStatus `ts` is wall-clock)."""
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
  if ceil_kph < cluster_kph - CLUSTER_TOL_KPH:
    return cluster_kph, f"episode {phase} but ceiling {ceil_kph:.1f} kph is below the cluster set {cluster_kph:.1f}"
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
    value, reason = max_speed_display(cluster_kph, self._st, self._wall(), capable)
    if reason != self._last_reason:
      self._last_reason = reason
      if reason is not None and self._read_err is None:
        cloudlog.warning(f"uimax2pnw: {reason} -- the screen MAX SPEED falls back to the truck set speed")
    return value
