# Panda firmware, flashing, safety & fingerprinting

The panda is the CAN/safety hardware (STM32H725) between openpilot and the car. This is the most
dangerous area of the workbench — a bad flash bricks the panda, and safety code is life-critical.
**Read the pitfalls before flashing anything.**

## Table of contents

- [Safety first (non-negotiable)](#safety-first-non-negotiable)
- [The CAN_PACKET_VERSION_HASH rule](#the-can_packet_version_hash-rule)
- [Panda build & flash](#panda-build--flash)
- [The four panda-flash failure modes](#the-four-panda-flash-failure-modes)
- [Real power-cycle vs soft reboot](#real-power-cycle-vs-soft-reboot)
- [Recovery if the panda goes dark](#recovery-if-the-panda-goes-dark)
- [Fingerprinting a car](#fingerprinting-a-car)
- [Tesla 0x348 ignition](#tesla-0x348-ignition)

## Safety first (non-negotiable)

Safety-critical code lives in `panda/` (C, STM32H725) and `opendbc/safety/`. openpilot follows
ISO 26262. The bootstub verifies an **RSA signature** of the app before booting it, so firmware is
signed.

- **Never weaken a safety check, raise a torque/rate limit, or bypass a panda safety model.**
- New feature toggles default **OFF**.
- If a change would touch panda safety (a `safety/modes/*.h`, a limit, an ignition condition), **stop
  and flag it to the user** rather than proceeding. Priority order: **safety > stability > quality >
  features.**
- Safety modes: `SILENT`, `NOOUTPUT`, `ALLOUTPUT` (test only), `ELM327` (OBD-II), and car-specific
  modes compiled from opendbc. The Tesla legacy car uses `tesla_legacy` (`SAFETY_TESLA_LEGACY 36U`)
  with `FLAG_HW3`.

## The CAN_PACKET_VERSION_HASH rule

**This is the root cause behind most "the Raven won't drive on fork X" failures.** The panda firmware
and the openpilot library each compute a hash of `opendbc/safety/can.h`. If the firmware was built
against a *different* `can.h` than the running library uses, the hashes disagree and CAN comms fail
(or the panda is treated as foreign and pandad loops).

```python
# how the hash is computed (panda/SConscript), and how to check it matches the device's opendbc:
python3 -c "import hashlib,opendbc,os; \
  d=open(os.path.join(opendbc.INCLUDE_PATH,'opendbc/safety/can.h'),'rb').read().replace(b'\r',b''); \
  print('CAN_PACKET_VERSION_HASH=0x%08X'%int.from_bytes(hashlib.sha256(d).digest()[:4],'little'))"
```

Consequences and rules:
- **Always build the panda against the *device's* opendbc**, then verify the printed hash matches the
  running openpilot's opendbc before flashing. xnor's known-good hash is `0x75ABF276`.
- xnor "just works" because it ships a **matched set** (firmware + library from one opendbc snapshot).
- This is *the* reason the porting doctrine says keep xnor as the base — see
  `references/forks-and-porting.md`.

## Panda build & flash

Build the panda firmware **on the device** (so it links against the device's opendbc and gets the
right hash):

```bash
# On the device:
source /usr/local/venv/bin/activate
export PYTHONPATH=/data/openpilot:/data/openpilot/opendbc_repo:/data/openpilot/panda
cd /data/openpilot/panda
scons -j4                      # build firmware (~1 min)
# verify hash matches the running opendbc (see command above) BEFORE flashing
sudo pkill -f manager.py       # CRITICAL: release the SPI lock first (see failure mode 1)
sleep 5
python board/flash.py          # flash app firmware
# then FULL ignition power-cycle (not a soft reboot) — see below
```

Useful panda tools (from `panda/CLAUDE.md`): `board/recover.py` (flash bootstub / recovery),
`scripts/reflash_internal_panda.py` (Tres/Cuatro internal panda), `scripts/can_health.py`,
`scripts/can_printer.py`, `scripts/debug_console.py`. After flashing, `Panda().bootstub == False`
means it booted the new app; `Panda.list()` returns the serial.

## The four panda-flash failure modes

All four were hit during the May 2026 Raven deploy and are now defended by the scripts; recognize
them so you can recover.

1. **`flash.py` hangs at "flash: resetting".** `pandad` holds an exclusive `flock()` on
   `/dev/spidev0.0`, so flash.py can't reconnect to verify (the write already succeeded). **Fix:**
   `sudo pkill -f manager.py` before flashing; Ctrl-C the hung flash, pkill, retry.

2. **`fatal error: opendbc/safety/ignition.h: No such file or directory`.** xnor's panda commit moved
   ignition detection into opendbc, but the device's older opendbc snapshot lacks that file. **Fix:**
   `raven_assets.py` bundles `ignition.h` (with the Tesla 0x348 exception merged in) and
   `patch-raven.py` deploys it.

3. **`conflicting types for 'ignition_can_hook'`.** The xnor cherry-pick left *both* the
   `#include "opendbc/safety/ignition.h"` and a local `ignition_can_hook` definition in
   `can_common.h` → duplicate definition. **Fix:** `patch-raven.py` brace-matches and removes the
   local block; the Tesla logic already lives in the bundled `ignition.h`.

4. **`AttributeError: type object 'CarHarness' has no attribute 'tesla_model_x_hw1'`.** The legacy
   Tesla `values.py` references 4 `CarHarness` entries the device's `docs_definitions.py` doesn't
   have (it ships only `tesla_a`/`tesla_b`); the whole Tesla module fails to import → driving stack
   down. **Fix:** `patch-raven.py` inserts the 4 entries after the `tesla_b` line.

The throughline: porting Tesla into a fork whose opendbc/panda predate xnor's changes produces
exactly these cascading import/build failures — another argument for the xnor-base doctrine.

## Real power-cycle vs soft reboot

On the cuatro panda (integrated in the comma 3X), the panda chip is powered through the comma device,
not the OBD port. `sudo reboot` restarts the SoC userspace but the device's regulators keep the panda
chip's 12 V rails up — **the panda chip never power-cycles**, so a hung chip stays hung. A real
power-cycle requires removing 12 V from the harness: **turn the car's ignition fully OFF for ~30 s**,
which lets the regulators bleed down and finally power-cycles the panda. Always do this after a flash.

## Recovery if the panda goes dark

`Panda.list() == []` with no process holding `/dev/spidev0.0`, after a soft reboot. Escalate:
1. Full ignition power-cycle (~30 s off) — fixes a hung-but-alive chip.
2. `python board/recover.py` (reflash bootstub) from `/data/openpilot/panda`.
3. DFU recovery if the bootstub itself is gone.

See `RAVEN.md` for the full escalation ladder. **Read `RAVEN.md` top-to-bottom before any
destructive flash.**

## Fingerprinting a car

openpilot identifies a car by querying ECU firmware versions over UDS at boot. To capture them:

```bash
~/gh/comma/fingerprint.sh        # one command; car must be powered (ACC/engine on), harness connected
```

This replaces the manual two-tmux procedure (window A: stop manager, run `./pandad` in foreground to
hold the panda; window B: `python fw_versions.py` twice — first run wakes ECUs, second gets real
responses). Reboot afterward to restore normal operation.

**The Raven cannot auto-fingerprint** — its EPS rejects FW-version UDS queries. It uses a fixed
fingerprint param `CarPlatformBundle=TESLA_MODEL_S_HW3` (`set-raven-fixed-fingerprint.py`). The UI
showing "vehicle fingerprint selected manually" is **expected, not an error**. Sticky-warning: the
fixed fingerprint forces *every* car to the Raven — **clear it before moving the device to the Ford**,
or the Lightning is misdetected. See `references/cars.md` for the Ford's own EPS fingerprint quirk.

## Tesla 0x348 ignition

The Raven has no conventional ignition line openpilot recognizes, so the panda detects ignition from
CAN: message **`0x348` (`GTW_status`) on bus 1**, bit `data[0] & 0x1` (`GTW_driveRailReq`), true only
when the car is genuinely in Drive/READY. This lives in `opendbc/safety/ignition.h` (the bundled,
known-good version) and the pinned `dirkpetersen/panda@dirk` firmware. Without it, the device never
goes onroad in the Tesla.
