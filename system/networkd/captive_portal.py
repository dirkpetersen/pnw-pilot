"""
network2xnor: captive-portal auto-accept handlers.

Some "home/work" WiFi networks (e.g. Peak Internet's "Visitor" SSID) gate traffic behind a captive
portal whose only requirement is clicking a terms-of-service "connect" button. The comma has no
browser, so NM flags the network as having no connectivity and our uploads to S3 never go out. For
portals that are a simple form submit (NOT a username/password or SMS login — those are not bypassable
headlessly), we can satisfy them by replaying that form over HTTP.

A priority-network entry opts in by setting its "portal" field to a handler key below (e.g. "peak").
The daemon, after switching to that SSID, calls `accept(handler)`; the arbiter re-checks connectivity
on its next loop. Best-effort and idempotent: if already online it does nothing; ALL errors are
swallowed (a portal we can't satisfy just stays offline, exactly as before).

How it works (mirrors what a browser does, so it's robust to portals that fill fields server-side):
  1. GET a neutral http URL so the gateway redirects us to the portal page WITH the per-client UAM
     params (mac / ip / link-login / dst) filled in — those are blank if you GET the portal directly.
  2. Parse the rendered <form>, replay EVERY field (hidden inputs + the submit button), POST it back.
  3. Follow up to a couple more form hops (MikroTik external portals often render a second auto-submit
     form to the gateway's link-login after the TOS POST).
  4. Re-checking connectivity is the arbiter's job. We just submit and log what happened.

The form is replayed from a FRESH fetch on the comma's own connection, so the gateway authorizes the
COMMA's MAC/session — never a stale capture from another device.

NOTHING here touches panda/safety. The only side effect is outbound HTTP to the portal.
"""
from __future__ import annotations

from html.parser import HTMLParser

from openpilot.common.swaglog import cloudlog

# handler key -> portal spec. Add new portals here.
PORTALS: dict[str, dict] = {
  # Peak Internet "Visitor" / OSU "Free Wifi" hotspot: MikroTik-style, TOS-accept only
  # (username "T-", NO password). `probe` triggers the captive redirect so the portal form
  # comes back with mac/ip/link-login/dst filled; `force` guarantees the CONNECT button is
  # sent even if button-parsing misses it. `fallback` is the portal page itself, tried if
  # the probe doesn't redirect to a form.
  "peak": {
    "probe": "http://www.msftconnecttest.com/redirect",
    "fallback": "http://hotspot-lebjc.peak.org/",
    "force": {"formLoginSubmit": "Submit"},
  },
}

PORTAL_TIMEOUT_S = 8
MAX_FORM_HOPS = 3
_UA = "Mozilla/5.0 (X11; Linux aarch64) comma-captive-portal"


def known(handler: str | None) -> bool:
  return bool(handler) and handler in PORTALS


class _FirstForm(HTMLParser):
  """Extract the first <form>'s action, method, and all submittable name=value pairs."""
  def __init__(self) -> None:
    super().__init__()
    self.action: str | None = None
    self.method: str = "post"
    self.fields: dict[str, str] = {}
    self._in_form = False
    self._done = False

  def _tag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
    if self._done:
      return
    a = {k.lower(): (v or "") for k, v in attrs}
    if tag == "form" and not self._in_form:
      self._in_form = True
      self.action = a.get("action")
      self.method = a.get("method", "post") or "post"
    elif self._in_form and tag in ("input", "button") and a.get("name"):
      t = a.get("type", "").lower()
      if t in ("checkbox", "radio") and "checked" not in a:
        return  # skip un-checked toggles
      if t in ("submit", "button", "image") and tag == "input" and "value" not in a:
        return
      self.fields[a["name"]] = a.get("value", "")  # last value wins

  def handle_starttag(self, tag, attrs):
    self._tag(tag, attrs)

  def handle_startendtag(self, tag, attrs):  # self-closing <input .../>
    self._tag(tag, attrs)

  def handle_endtag(self, tag):
    if tag == "form" and self._in_form:
      self._done = True
      self._in_form = False


def _parse_form(html: str) -> tuple[str | None, str, dict[str, str]]:
  p = _FirstForm()
  try:
    p.feed(html or "")
  except Exception:
    pass
  return p.action, p.method, p.fields


def accept(handler: str | None, already_online: bool = False) -> bool:
  """Best-effort: walk the captive portal's TOS form(s). Returns True if a submit went through OK.

  `already_online` short-circuits. ALL network/exception failures are logged and swallowed (return
  False) — a portal we can't satisfy must never crash the arbiter loop.
  """
  if already_online or not known(handler):
    return False
  spec = PORTALS[handler]
  try:
    import requests
    from urllib.parse import urljoin

    s = requests.Session()
    s.headers["User-Agent"] = _UA

    # 1) get the portal form. Try the redirect-trigger probe first (fills the UAM params),
    #    then the portal page directly as a fallback.
    resp = None
    for url in (spec.get("probe"), spec.get("fallback"), spec.get("url")):
      if not url:
        continue
      r = s.get(url, timeout=PORTAL_TIMEOUT_S, allow_redirects=True)
      if _parse_form(r.text)[2]:        # found a form with fields
        resp = r
        break
      resp = r                          # keep the last response for logging even if no form

    # 2) replay the form, then follow a few hops (TOS form -> gateway auto-submit form -> done)
    hops = []
    for _ in range(MAX_FORM_HOPS):
      action, method, fields = _parse_form(resp.text)
      if not fields:
        break
      fields.update(spec.get("force", {}))
      url = urljoin(resp.url, action) if action else resp.url
      if method.lower() == "get":
        resp = s.get(url, params=fields, timeout=PORTAL_TIMEOUT_S, allow_redirects=True)
      else:
        resp = s.post(url, data=fields, timeout=PORTAL_TIMEOUT_S, allow_redirects=True)
      hops.append({"url": url, "status": resp.status_code, "fields": sorted(fields)})

    ok = bool(hops) and resp.status_code < 400
    cloudlog.event("network2xnor_captive_portal", handler=handler, hops=hops,
                   final_url=getattr(resp, "url", ""),
                   status=getattr(resp, "status_code", None), ok=ok)
    return ok
  except Exception:
    cloudlog.exception(f"network2xnor: captive-portal '{handler}' failed")
    return False
