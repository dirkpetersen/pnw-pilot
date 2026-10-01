"""tailscale2pnw: pure status vocabulary + `tailscale status --json` classification (no I/O, no imports of
anything heavy, so the settings UI and the tests can import it)."""
from __future__ import annotations

# The TailscaleStatus param holds exactly one of these shapes (the UI shows it verbatim). The FIRST WORD is one of
# connected / disconnected / unconfigured / connecting / installing / error (title_word); anything after it is a reason.
OFF = "off"                 # the daemon's own steady state while Disable Remote SSH is ON; the UI shows DISABLED_TEXT
UNCONFIGURED = "unconfigured"   # no auth key file AND no node state: nothing to run; the toggle stays OFF (the default)
INSTALLING = "installing"
CONNECTING = "connecting"
INSTALL_DEFERRED = "installing - deferred until parked"
CONNECTED_PREFIX = "connected "
DISCONNECTED_PREFIX = "disconnected"
ERROR_PREFIX = "error "
DISABLED_TEXT = DISCONNECTED_PREFIX + " - disabled by this toggle"
NO_INTERNET = DISCONNECTED_PREFIX + " - no internet"
WORDS = ("connected", "disconnected", "unconfigured", "connecting", "installing", "error")

MAX_REASON = 120


def connected(ip: str) -> str:
  return f"{CONNECTED_PREFIX}{ip}"


def error(reason: str) -> str:
  one_line = " ".join(str(reason).split())
  if len(one_line) > MAX_REASON:
    one_line = one_line[:MAX_REASON - 1] + "…"
  return f"{ERROR_PREFIX}{one_line}"


def classify(status: dict) -> tuple[str, str | None]:
  """Map `tailscale status --json` to (state, ip). state is one of running / needs_login / stopped / starting /
  needs_approval / unknown. An unrecognised BackendState is 'unknown' -- the caller reports it as an error, it is
  never quietly treated as 'connecting'."""
  backend = status.get("BackendState")
  if backend == "Running":
    ips = status.get("TailscaleIPs") or (status.get("Self") or {}).get("TailscaleIPs") or []
    v4 = [ip for ip in ips if "." in ip]
    ip = (v4 or ips or [None])[0]
    return ("running", ip) if ip else ("starting", None)  # Running with no address yet: still coming up
  if backend == "NeedsLogin":
    return "needs_login", None
  if backend == "Stopped":
    return "stopped", None
  if backend in ("Starting", "NoState"):
    return "starting", None
  if backend == "NeedsMachineAuth":
    return "needs_approval", None
  return "unknown", None


def title_word(text: str) -> str:
  """Short, bounded state word for the toggle title ('off' -> ''); the reason/address stays in the description."""
  if text == OFF:
    return ""
  first = text.split(" ", 1)[0]
  return first if first in WORDS else text  # an unrecognised status is shown whole, never hidden


def ui_text(enabled: bool, raw: str) -> str:
  """What the settings screen shows for the 'Disable Remote SSH (Tailscale)' toggle. `enabled` = Remote SSH enabled,
  i.e. the toggle is OFF (the default). Disabled -> 'disconnected - disabled by this toggle' no matter what a stale param
  says, EXCEPT an error (e.g. 'tailscaled still running after toggle off') which must stay visible. Enabled but the
  daemon has not published yet, or still shows its own 'off' from before the toggle flipped -> 'connecting' (not
  silence, not a stale 'off')."""
  if not enabled:
    return raw if raw.startswith(ERROR_PREFIX) else DISABLED_TEXT
  return raw if raw and raw != OFF else CONNECTING
