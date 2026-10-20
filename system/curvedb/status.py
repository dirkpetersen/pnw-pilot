"""curvedbpull2pnw: what the settings row shows for 'Disable Curve DB Auto-Install' (pure, no I/O)."""
from __future__ import annotations

import re

DISABLED_TEXT = "disabled by this toggle"
STARTING_TEXT = "starting"


def pack(text: str, wall: float) -> str:
  """The CurveDbAutoInstallStatus param: human text + ' @<epoch of the last check>'."""
  return f"{text} @{int(wall)}"


def _ago(secs: float) -> str:
  if secs < 90:
    return "just now"
  if secs < 5400:
    return f"{round(secs / 60)} min ago"
  if secs < 172800:
    return f"{round(secs / 3600)} h ago"
  return f"{round(secs / 86400)} d ago"


def ui_text(disabled: bool, raw: str, now: float) -> str:
  """Disabled hides everything except an error, which must stay visible (like the Tailscale row)."""
  text, m = raw, re.search(r" @(\d+)$", raw)
  if m:
    text = raw[:m.start()]
  if disabled:
    return text if text.startswith("error") else DISABLED_TEXT
  if not text:
    return STARTING_TEXT
  return f"{text}, checked {_ago(now - int(m.group(1)))}" if m else text
