# FINGERPRINT2XNOR — Lightning should not look like "dashcam" when the truck is off

**Branch:** `fingerprint2xnor` (off `integration2xnor`) · **Status:** option A (display-only) DRAFTED &
committed; **NOT deployed** — needs one on-device visual check + a decision on whether the real
complaint is cosmetic (A) or functional/ignition (B). **Device:** ONE shared comma 3X moved between
the Tesla Model S Raven and the Ford F-150 Lightning.

## Goal (and the hard constraint)

Make the device **recognize/show the last-known car (Lightning) when the truck is OFF** instead of a
bare "dashcam" home screen — **without** pinning a fixed fingerprint (a static `CarPlatformBundle`
pin forces the wrong car after a move and breaks the other car's auto-detect). Both cars must keep
**auto-detecting when powered**.

## What it actually is (corrected root cause, with evidence)

The first theory ("`card.py` blocks on its CAN-wait when off") was **wrong**. Findings from a code
investigation:

1. **`card` is `only_onroad`** (`system/manager/process_config.py:98`) — it doesn't run at all when
   offroad. So the off-state isn't a `card` fingerprint block; it's just the **offroad home screen**.
2. **Onroad is gated on panda ignition** — `system/hardware/hardwared.py:230`
   (`ignition = any(ps.ignitionLine or ps.ignitionCan ...)`) and the start decision at
   `hardwared.py:333`. So "truck off" = no ignition = offroad. A *slow/marginal* Lightning ignition
   could make it sit offroad while the truck is actually on (the functional failure mode → option B).
3. **The offroad UI shows no car identity at all.** It switches HOME vs ONROAD purely on
   `ui_state.started` (`selfdrive/ui/layouts/main.py:66-80`); the sidebar only shows panda
   connection ("VEHICLE / ONLINE") (`selfdrive/ui/layouts/sidebar.py:149-152`). Nothing offroad reads
   the car. That generic home screen *is* the "dashcam mode" the user sees.
4. **`dashcamOnly` is NOT set for the Lightning** when it fingerprints — it's only set for MOCK /
   unrecognized cars (`opendbc_repo/opendbc/car/mock/interface.py:20`,
   `car_helpers.py:155-157`). So this is not a true dashcam-only car; it's just offroad.
5. **No existing mechanism** remembers/show the last car offroad. `CarParamsPersistent` is written
   onroad (`card.py`) and read by the UI every ~5 s into `ui_state.CP` (`ui_state.py:85,184`), but only
   to gate experimental features — never to display the car.

## Shared-device safety principle (held by the fix)

The cache (`CarParamsPersistent` / `ui_state.CP`) holds the **last** car and can be stale after a
move. The fix uses it **only to DISPLAY** a brand label offroad. It **never** influences
fingerprinting or control — `card` is `only_onroad` and re-fingerprints authoritatively when the car
powers on, so a stale cache can at worst show the wrong brand for a moment while parked, never
mis-control.

## Option A — DRAFTED (display-only, safe): show the last car in the sidebar offroad

`selfdrive/ui/layouts/sidebar.py` `_update_panda_status()`: when **offroad** and a recognized car is
cached, show its brand (e.g. `VEHICLE / FORD`) instead of `VEHICLE / ONLINE`. Hard-guarded so it can
never crash the UI (which is `restart_if_crash`):

```python
try:
    cp = ui_state.CP
    if (not ui_state.started and cp is not None and cp.brand and cp.brand != "mock"
        and not cp.dashcamOnly):
        self._panda_status.update(tr_noop("VEHICLE"), cp.brand.upper(), Colors.GOOD)
        return
except Exception:
    pass
self._panda_status.update(tr_noop("VEHICLE"), tr_noop("ONLINE"), Colors.GOOD)
```

- **Zero control risk** — UI-only; `card` stays `only_onroad`.
- **Device-swap safe** — onroad fingerprinting re-persists the real car before any control.
- **Known cosmetic caveat** — immediately after moving the device to the *other* car while parked, the
  sidebar shows the previous brand until that car is powered on once (then it self-corrects). This is
  display-only and harmless.
- `py_compile` + `ruff` clean. **Not yet visually verified on-device.**

This addresses the **cosmetic** reading of the complaint ("when off it should show the Lightning, not
dashcam").

## Option B — if the real complaint is FUNCTIONAL (decide on-device)

If the truck-ON state is **slow/flaky to go onroad** (i.e. it's "stuck in dashcam after I start the
truck" rather than "I want a label when parked"), the fix is **ignition detection**, not the sidebar:
verify `pandaStates.ignitionLine` / `ignitionCan` for the Lightning come up promptly
(`hardwared.py:230`). That's a panda/ignition-source change, separate from option A.

## On-device runbook (distinguishes A vs B; run when the device is back)

1. **Photograph the off-state screen** — bare offroad home screen (option A) vs a literal "Dashcam
   mode" banner (would mean `dashcamOnly`, unexpected here).
2. **Start the truck; time to "Ford F-150 Lightning" + engageable.** Fast self-recovery → it's
   cosmetic (A is enough). Slow / needs reboot → it's ignition (B).
3. **Check ignition while ON:** `grep -aiE "ignition|onroad|started" ` newest `/data/log/swaglog.*`;
   confirm `ignitionLine`/`ignitionCan` true promptly.
4. **Confirm the cache is valid** (so A even has something to show):
   `PYTHONPATH=/data/openpilot python -c "from openpilot.common.params import Params; from cereal import car; r=Params().get('CarParamsPersistent'); print('none' if not r else (lambda c:(c.brand,c.carFingerprint,len(c.carFw)))(car.CarParams.from_bytes(r)))"`

## Deploy (after the on-device check)

`sidebar.py` is pure-Python UI loaded at runtime — **no build needed** (like the BSM deploy). Deploy =
md5-guarded file copy + clear pyc + restart the UI / next boot. Rollback = restore backup. I'll
generate the `update-fingerprint.sh` / `rollback-fingerprint.sh` pair on request once A is confirmed
the right fix.

**Do NOT pin `CarPlatformBundle`** (`set-*-fixed-fingerprint.py`) as the fix — it breaks dual
auto-detect.

## Files
- `selfdrive/ui/layouts/sidebar.py` — option A (display-only offroad car label) — **drafted**
- `FINGERPRINT2XNOR.md` — this doc
- Related: `shared-device-fingerprint` memory; `set-*-fixed-fingerprint.py` (manual pin — NOT this fix)
