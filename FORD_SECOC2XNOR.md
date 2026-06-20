# FORD_SECOC2XNOR — fix the flaky Ford SecOC dashcam false-positive

**Branch:** `fordsecoc2xnor` (off `light2xnor`) · **Status:** implemented, Gemini-reviewed, validated;
**NOT deployed** (preventative — the cache-clear already fixed the live truck). Deploy = patch the
device's `ford/interface.py` + clear `CarParams*` cache; takes effect on the next fingerprint.

## Symptom

The F-150 Lightning intermittently came up **`dashcamOnly=True`** (dashcam mode, can't engage) even
though it's a fully supported, non-SecOC truck. Clearing the `CarParams*` cache + re-fingerprinting
fixed it — proving it was a flaky fingerprint, not a real limitation.

## Root cause

`opendbc/car/ford/interface.py` locks CAN-FD Fords to dashcam if SecOC (Ford's "TRON" message
authentication) is detected on the camera bus:

```python
if len(fingerprint[CAN.camera]):
    if fingerprint[CAN.camera].get(0x3d6) != 8 or fingerprint[CAN.camera].get(0x186) != 8:
        ret.dashcamOnly = True   # 'SecOC is unsupported'
```

`0x3d6` = `LateralMotionControl2`, `0x186` = `ACCDATA` — both broadcast by the ADAS camera (IPMA),
**8 bytes normally, 16 bytes on SecOC**. The bug: `.get(addr) != 8` is **also true when the message
is ABSENT** (`None != 8`). The CAN fingerprint window for a Ford is only **~1 s** (it FW-fingerprints,
so the legacy CAN-address candidates eliminate to empty and `can_fingerprint` exits at
`frame > FRAME_FINGERPRINT`). If the ADAS camera hasn't broadcast those two messages within that ~1 s
(e.g. right after a reboot, camera still coming up), they're absent → false SecOC → dashcam.

Confirmed on the live truck: `0x3d6` and `0x186` are present on the camera bus (bus 2) **at 8 bytes**
→ no SecOC. So the lockout was purely a boot-timing fingerprint race.

## Fix (minimal, Ford-only, safe)

Flag SecOC **only when a message is actually PRESENT but not 8 bytes** (the genuine 16-byte TRON
signature) — never when merely absent:

```python
cam = fingerprint[CAN.camera]
if cam:
    secoc = (0x3d6 in cam and cam[0x3d6] != 8) or (0x186 in cam and cam[0x186] != 8)
    if secoc:
        ret.dashcamOnly = True
```

**Why it's safe (not a weakened safety check):** genuine SecOC/TRON cars broadcast these messages at
**16 bytes**, so they're still caught (`present and != 8`). The change only stops treating an *absent*
message — which is "camera not captured yet," not SecOC — as a lockout. A truck that's briefly missing
these at fingerprint will have them moments later (at 8 bytes), and carstate handles not-yet-seen
messages normally. No panda/safety-model code is touched.

Behavior table (unit-tested, old → new):

| camera fingerprint | old dashcam | new dashcam |
|---|---|---|
| normal, both 8B | False | False |
| real SecOC, both 16B | **True** | **True** |
| one msg 16B | **True** | **True** |
| absent (camera slow @ boot) | **True (false +)** | **False (fixed)** |
| camera bus empty | False | False |

## Deploy (when wanted)

`ford/interface.py` is pure Python (runtime). Patch the device's copy + clear `CarParams*` so the next
fingerprint uses the new logic. It's **preventative** — the live truck is already fine after the
cache-clear — so no rush; bundle it the next time we touch the device. A one-liner `fix-lightning-
dashcam.sh` (cache-clear) remains the manual fallback if it ever recurs before this is deployed.

## Files
- `opendbc_repo/opendbc/car/ford/interface.py` — the SecOC condition (only change)
