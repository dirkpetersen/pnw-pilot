# TODO — PNW Pilot known gaps / future work

## TBD: Ford SecOC false-lockout fix (`fordsecoc2xnor`)

**Status:** removed from `pnwtest`/`pnwprod` — needs validation before re-adding.

**What it fixes:** the original `interface.py` SecOC check fires `dashcamOnly = True` not just when
a TRON message is present at 16 bytes (genuine SecOC), but also when the message is simply **absent**
(`None != 8` is true). This causes a flaky false SecOC lockout on a fully-supported 2025 F-150
Lightning: if the ADAS camera (IPMA) hasn't broadcast `0x3d6`/`0x186` yet inside the short (~1 s)
fingerprint window (e.g. right after boot), the truck drops to `dashcamOnly` even though it's not a
TRON platform.

**The fix** (branch `fordsecoc2xnor`, commit `326d05b127`): check `(addr in cam and cam[addr] != 8)`
instead of `cam.get(addr) != 8` — absent means absent, not SecOC; genuine TRON cars still caught
(they broadcast at 16 bytes).

**Why removed for now:** needs to be tested on the actual 2025 Lightning to confirm it doesn't
accidentally allow a real SecOC/TRON truck through. Once validated, cherry-pick `326d05b127` back
onto `pnwtest`, verify, then promote to `pnwprod`.

**Source branch:** `fordsecoc2xnor` (tip `326d05b127`) — lives in `pnw-pilot` and in
`~/gh/comma/xnor/openpilot`.
