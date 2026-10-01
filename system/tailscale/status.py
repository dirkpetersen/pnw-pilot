"""tailscale2pnw: pure status vocabulary + `tailscale status --json` classification (no I/O, no imports of
anything heavy, so the settings UI and the tests can import it)."""
from __future__ import annotations

# The TailscaleStatus param holds exactly one of these shapes (the UI shows it verbatim).
OFF = "off"
INSTALLING = "installing"
NEEDS_AUTH_KEY = "needs auth key"
CONNECTING = "connecting"
INSTALL_DEFERRED = "install deferred until parked"
CONNECTED_PREFIX = "connected "
ERROR_PREFIX = "error "

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
  if text.startswith(CONNECTED_PREFIX):
    return "connected"
  if text.startswith(ERROR_PREFIX):
    return "error"
  return text  # installing / install deferred until parked / needs auth key / connecting / starting


def ui_text(enabled: bool, raw: str) -> str:
  """What the settings screen shows for the toggle. Toggle off -> 'off' no matter what a stale param says, EXCEPT an
  error (e.g. 'tailscaled still running after toggle off') which must stay visible; toggle on but the daemon has not
  published yet -> 'starting' (not silence)."""
  if not enabled:
    return raw if raw.startswith(ERROR_PREFIX) else OFF
  return raw if raw else "starting"
