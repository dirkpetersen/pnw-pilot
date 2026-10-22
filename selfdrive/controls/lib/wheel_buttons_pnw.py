"""wheelbtnlog2pnw -- LOGGING ONLY: which steering-wheel buttons the DRIVER pressed, for ces_events.

WHY. drives/2026-10-21/corvallis-portland-i5/DRIVE_REPORT.md: the 85 mph in a 65 was the driver holding SET+
for 4.4 s, but no ces_events field recorded wheel presses (the old tick field `button` is the CES on-screen mode
button), so the analysis blamed openpilot. The Lightning's cancel/resume rocker (Steering_Data_FD1
CcAslButtnCnclResPress) is not turned into a carState ButtonEvent by the pinned opendbc, so driver ACC toggles
looked like stock-ACC dropouts.

WHAT. One small tracker, fed (name, pressed) pairs:
  * carState.buttonEvents, named by str() of the capnp ButtonEvent.Type enum, which is the BARE name
    ("accelCruise", "decelCruise", "resumeCruise", "cancel", "gapAdjustCruise", ...). Never compare that string
    against "ButtonType.x" (the capnp enum str() trap).
  * the raw rocker bits card reads from CAN (PnwVehicle.wheel_rocker), named by their label ("cnclRes", "cncl").
    card (accdrop_pnw.py) publishes those to the WHEEL_ROCKER_PARAM mem-param; ces_pnw merges them in.
Record fields: wbLast (last pressed name), wbAge (s since that press edge), wbN (press edges in the window),
wbHeld (a button down right now, the most recently pressed one), each None when there is nothing to report.
"""
import math
from collections import deque

WHEEL_ROCKER_PARAM = "WheelRocker"   # /dev/shm mem-param: card -> ces_pnw, {"edges": [[t, name], ...], "held": [...]}
EDGES_MAXLEN = 64                    # far more presses than a hand makes in one ~1 s tick or a 3 s accDrop window


class WheelButtons:
  def __init__(self):
    self.edges: deque = deque(maxlen=EDGES_MAXLEN)   # (monotonic t, name) of each press edge; NOT sorted (merged)
    self.held: dict = {}                              # name -> t it went down

  def feed(self, now: float, events) -> None:
    for name, pressed in events:
      if pressed:
        self.edges.append((now, name))
        self.held.setdefault(name, now)
      else:
        self.held.pop(name, None)

  def fields(self, now: float, since: float, extra_held=()) -> dict:
    """The record fields for (since, now]. extra_held: names another source says are down (the rocker); each is
    dated by its newest press edge in self.edges, so wbHeld is the most recently pressed of ALL held buttons."""
    n = sum(1 for t, _ in self.edges if since < t <= now)
    past = [e for e in self.edges if e[0] <= now]
    last = max(past, key=lambda e: e[0]) if past else None
    down = dict(self.held)
    for name in extra_held:
      down.setdefault(name, max((t for t, n in self.edges if n == name), default=-math.inf))
    held = max(down, key=down.__getitem__) if down else None
    return {"wbLast": last[1] if last else None, "wbAge": round(now - last[0], 2) if last else None,
            "wbN": n, "wbHeld": held}
