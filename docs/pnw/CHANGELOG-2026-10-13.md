# CHANGELOG — 2026-10-13 (Tuesday)

All times PT. Branch work, not yet shipped.

## 1. `portalrecheck2pnw` (branch, not yet shipped)

Fixes the captive portal never re-logging in after its session expires (Visitor WiFi). Before: once `captive_portal.accept()`
succeeded, `portal_done_for = ssid` and the arbiter never looked at the portal again while on that SSID. The Peak "Visitor"
session expires after about 5 h (login 15:37, found dead 20:43 on 2026-10-13); every WiFi request is then redirected to the login
page, NM connectivity still reads `full` because LTE is up, and uploads over WiFi stalled with nothing logged.

- `system/networkd/network_arbiterd.py`: while on a configured portal SSID with `portal_done_for == ssid`, every
  `PORTAL_RECHECK_S = 300` s (monotonic) a probe runs in the existing captive-portal daemon thread (one thread at a time, so the loop,
  LTE logging and tethering NAT upkeep never block).
  - Probe says redirect/portal page: ONE `network2xnor_portal_expired` event (ssid, `since_ok_s`), `portal_done_for`/tries/result for
    that SSID are reset, and the existing `accept()` path re-runs on the same tick (still bounded by `PORTAL_MAX_TRIES` per expiry).
    Success logs `network2xnor_portal_relogin_ok` (with `down_s`).
  - Tries exhausted and still offline: stays loud, one `network2xnor_portal_still_expired` event per 300 s recheck (no further accept()
    calls until the SSID is left and re-entered).
  - Probe ERROR (timeout, DNS, odd status): `network2xnor_portal_probe_error` event, nothing reset, retried at the next interval.
    An error is neither "online" nor "expired".
  - Leaving the SSID resets the state (existing else-branch), so the clock restarts on re-entry.
- `system/networkd/captive_portal.py`: new `probe_online()` (204 + empty body = True; 200/3xx = False; transport error or other status
  RAISES). It uses the same unbound `requests` path as `accept()`'s own online check; WiFi holds the default route (metric 600 vs LTE 1000).
- Tests: `system/networkd/tests/test_portal_recheck.py` (8): no re-probe before 300 s, the interval is 300 s, redirect gives the expiry
  event plus accept() re-run plus relogin event, online changes nothing, probe error logged and retried, exhausted tries stay loud,
  leaving the SSID resets the clock, the probe runs in a thread (the loop keeps ticking while it blocks).
- Mutation check: dropping the `portal_done_for` reset (2 tests fail), interval 300 to 3000 (1), error treated as expired (1), probe run
  inline (1), the still-expired event removed (1).
- Not changed: the probe does not bind to wlan0 (neither does accept()). If WiFi were not the default route the probe would pass over LTE
  and report online; today that cannot happen with the metric 600 vs 1000 routes.
