"""
network2xnor: captive-portal auto-accept handlers.

Some "home/work" WiFi networks (e.g. Peak Internet's "Visitor" SSID) gate traffic behind a captive
portal whose only requirement is submitting a terms-of-service form. The comma has no browser, so NM
flags the network as having no connectivity and our uploads to S3 never go out. For portals that are a
simple form POST (NOT a username/password or SMS login — those are not bypassable headlessly), we can
satisfy them with one HTTP POST.

A priority-network entry opts in by setting its "portal" field to a handler key below (e.g. "peak").
The daemon, after switching to that SSID, calls `accept(handler, online_check)` which POSTs the form
and re-checks connectivity. Best-effort and idempotent: if already online it does nothing; all errors
are swallowed (a portal we can't satisfy just stays offline, exactly as before).

NOTHING here touches panda/safety. The only side effect is an outbound HTTP POST to the portal.
"""
from __future__ import annotations

from openpilot.common.swaglog import cloudlog

# handler key -> (portal POST url, form fields). Add new portals here.
PORTALS: dict[str, dict] = {
  # Peak Internet "Visitor" SSID — accept-TOS form (provided by the device owner).
  "peak": {
    "url": "http://hotspot-lebjc.peak.org/index.php",
    "data": {"accept_tos": "true", "submit": "Connect"},
  },
}

PORTAL_TIMEOUT_S = 8


def known(handler: str | None) -> bool:
  return bool(handler) and handler in PORTALS


def accept(handler: str | None, already_online: bool = False) -> bool:
  """Best-effort: POST the captive portal's accept form. Returns True if the POST was sent OK.

  `already_online` short-circuits: if connectivity is already good we never poke the portal. All
  network/exception failures are logged and swallowed (return False) — a portal we can't satisfy must
  never crash the arbiter loop.
  """
  if already_online or not known(handler):
    return False
  spec = PORTALS[handler]
  try:
    import requests
    resp = requests.post(spec["url"], data=spec["data"], timeout=PORTAL_TIMEOUT_S,
                         allow_redirects=True)
    ok = resp.status_code < 400
    cloudlog.event("network2xnor_captive_portal", handler=handler, url=spec["url"],
                   status=resp.status_code, ok=ok)
    return ok
  except Exception:
    cloudlog.exception(f"network2xnor: captive-portal '{handler}' POST failed")
    return False
