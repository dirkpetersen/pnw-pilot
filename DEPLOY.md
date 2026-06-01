# DEPLOY.md — `xnor2sunny`: Tesla Model S HW3 (Raven) on SunnyPilot

Deploying the `xnor2sunny` branch to the comma 4 to drive the **Tesla Model S HW3
(Raven) 2021**. This is the **pivot from BluePilot** — the BluePilot `xnor2bp`
Raven work was functionally complete but its **panda flash was blocked** by an
unrecoverable dual-panda enumeration loop (see `../bluepilot/DEPLOY.md`). SunnyPilot
is BluePilot's upstream and is closer to xnor's base, so we redo the panda path here.

Device: `comma@192.168.13.154` (host key changes on reinstall →
`ssh-keygen -R 192.168.13.154`).

> **TODO (loose end):** the sunnypilot worktree pins `opendbc_repo` at `ac4b8d69`
> (pre-`mg @35`). The `mg @35` capnp fix is committed in the opendbc repo at
> `d1c429f5` but NOT yet reflected in the submodule pointer. The device is correct
> (it has the fix as overlaid files), but to make a clean checkout correct: push
> opendbc `d1c429f5`, then re-pin + commit the submodule bump in sunnypilot.

---

## Why SunnyPilot avoids the two BluePilot disasters

Both BluePilot blockers were investigated and shown to be **structurally absent**
on SunnyPilot before any change was deployed:

### 1. CAN-hash mismatch ("CANbus disconnected / faulty cable") — IMPOSSIBLE here
The firmware↔library `CAN_PACKET_VERSION_HASH` is computed from
`opendbc/safety/can.h` (`panda/SConscript:159`). The Raven opendbc port **does not
touch `can.h`** — verified: the file's SHA-256 is **byte-identical** between our
port and the device (`76f2ab75…`). So the firmware and the running library cannot
disagree on the hash. On-device build confirmed `CAN_PACKET_VERSION_HASH=0x75ABF276`
baked into the firmware, matching the library's computed value exactly.

### 2. Dual-panda "Internal panda is missing" loop — handled by SunnyPilot pandad
The device enumerates **two pandas** (the same hardware condition that blocked
BluePilot):
- `3f001b000651333038363231` — internal car panda (SPI **and** USB)
- `2e0050001051323430373133` — spurious/unprovisioned panda (USB only)

SunnyPilot's `selfdrive/pandad/pandad.py` → `check_panda_support()` calls
**`Panda.spi_list()` first**, and on this device `spi_list()` returns **only the
internal panda**. BluePilot's pandad lacked this SPI-priority path and fell into the
"internal panda missing" loop. (Open confirmation item: this must be re-verified live
after the flash — see step 6.)

### 3. No `can_common.h` dedup needed (unlike BluePilot/dirk panda)
SunnyPilot panda uses its **own** `board/drivers/can_common.h::ignition_can_hook`
(it does NOT `#include opendbc/safety/ignition.h` like the dirk panda). So there is
**no duplicate-`ignition_can_hook` build error** to fix — but the Raven 0x348
ignition case must be **added** to that hook (step 3 below). The forward declaration
`void ignition_can_hook(CANPacket_t *to_push);` is already present in `drivers.h`.

---

## Reference material

- **opendbc Raven port:** `/home/dp/gh/comma/opendbc` @ `ac4b8d69` (branch `xnor2sunny`),
  pinned by the sunnypilot worktree at commit `9ed2a1705b`. 15 files (full list below).
- **Authoritative ignition source:** `~/gh/comma/panda-xnor` @ `origin/master-xnor`,
  `board/drivers/can_common.h` (xnor puts 0x348 in `can_common.h`, same architecture
  as sunnypilot — NOT in `ignition.h`).
- **Validation:** our `tesla_legacy.h` is **byte-identical** to xnor's `master-xnor`
  → the panda safety we flash is exactly what xnor drove on the road.
- xnor also ships safety unit tests `opendbc/safety/tests/test_tesla_hw1.py` /
  `test_tesla_hw23.py` (NOT yet pulled into our port — optional pre-drive validation).

---

## Deploy log — what was done, in order

Status as of **2026-05-31**: steps 0–5, 5.5, 7 **DONE and verified on-device**.
Panda flashed (sig matches build), dual-panda loop **resolved**, `canValid=True`.
**Only step 6 remains: the physical ignition power-cycle with the car ON** — after
which `safetyModel` should read `teslaLegacy` and `ignitionCan=True`.

### 0. Pre-flight — DONE
- Device reachable; running stock **sunnypilot `release-tizi` v2026.001.007**
  (commit `fba34f3`), opendbc + panda submodules both pinned at `fba34f3`.
- Confirmed device Tesla files match the port's base exactly (155/155/70 lines) →
  clean overlay, no release-tizi divergence clobbered.

### 1. Backup (reversibility) — DONE
`/data/xnor2sunny_backup/pre_overlay.tgz` holds the pre-overlay copies of the 12
existing files (the 3 brand-new files don't exist yet → rollback deletes them).
Submodule commits recorded in `opendbc_commit.txt` / `panda_commit.txt`.
```bash
# rollback if ever needed:
tar xzf /data/xnor2sunny_backup/pre_overlay.tgz -C /data/openpilot
rm /data/openpilot/opendbc_repo/opendbc/car/tesla/teslacan_legacy.py \
   /data/openpilot/opendbc_repo/opendbc/safety/modes/tesla_legacy.h \
   /data/openpilot/opendbc_repo/opendbc/dbc/tesla_raven_party.dbc
```

### 2. Overlay the 15 opendbc Raven port files — DONE
`scp`'d from `/home/dp/gh/comma/opendbc` to `/data/openpilot/opendbc_repo/<path>`.
Checksums verified byte-identical on device. Files:
```
opendbc/car/__init__.py                 opendbc/car/tesla/radar_interface.py
opendbc/car/car.capnp                   opendbc/car/tesla/teslacan_legacy.py   (new)
opendbc/car/docs_definitions.py         opendbc/car/tesla/values.py
opendbc/car/tesla/carcontroller.py      opendbc/car/torque_data/override.toml
opendbc/car/tesla/carstate.py           opendbc/dbc/tesla_raven_party.dbc      (new)
opendbc/car/tesla/fingerprints.py       opendbc/safety/declarations.h
opendbc/car/tesla/interface.py          opendbc/safety/modes/tesla_legacy.h    (new)
                                        opendbc/safety/safety.h
```
Safety registration confirmed: `safety.h` has `#include ".../tesla_legacy.h"` +
`{SAFETY_TESLA_LEGACY, &tesla_legacy_hooks}`; `declarations.h` has
`#define SAFETY_TESLA_LEGACY 36U` + `extern ... tesla_legacy_hooks`;
`car.capnp` has `teslaLegacy`.

### 2.5. ⚠️ FIX — capnp ordinal hole `mg @35` (caught on device, NOT in original port)
The first thing that ran on-device crashed **every** Python process that imports cereal:
```
data/openpilot/cereal/car.capnp:637: failed: Skipped ordinal @35.
Ordinals must be sequential with no holes.
```
The port set `SAFETY_TESLA_LEGACY = 36U` (to match xnor's panda value) and added
`teslaLegacy @36` to the `SafetyModel` enum — but jumped straight from
`volkswagenMeb @34`, leaving `@35` empty. capnp forbids gaps. xnor fills `@35` with
a placeholder `mg @35;` (MG safety, which we don't otherwise port). Fix applied to
`opendbc/car/tesla/car.capnp` (device path
`/data/openpilot/opendbc_repo/opendbc/car/car.capnp`, which `cereal/car.capnp`
symlinks to):
```capnp
    volkswagenMeb @34;
    mg @35;          # placeholder to keep ordinals sequential before teslaLegacy
    teslaLegacy @36;
```
Also fixed in the source repo `/home/dp/gh/comma/opendbc` so the branch matches.
After the fix: `cereal.car.CarParams.SafetyModel.teslaLegacy == 36` loads cleanly.
**Lesson:** the BluePilot `xnor2bp` checklist already warned "teslaLegacy @36 (+ mg @35)" —
the `mg @35` filler is mandatory, not optional.

### 3. Add Raven 0x348 ignition block to panda `can_common.h` — DONE
Inserted into `ignition_can_hook` in
`/data/openpilot/panda/board/drivers/can_common.h`, **outside** the `bus==0` block
(it matches bus 0 **or** 1, so it uses its own `(int)GET_LEN(msg)`):
```c
// Tesla Model S exception (Raven HW1/HW2/HW3)
if (((msg->bus == 0) || (msg->bus == 1)) && (msg->addr == 0x348U) && ((int)GET_LEN(msg) == 8)) {
  int counter = msg->data[6] & 0xFU;
  static int prev_counter_tesla_legacy = -1;
  if ((counter == ((prev_counter_tesla_legacy + 1) % 16)) && (prev_counter_tesla_legacy != -1)) {
    // GTW_status
    ignition_can = (msg->data[0] & 0x1U) != 0U;
    ignition_can_cnt = 0U;
  }
  prev_counter_tesla_legacy = counter;
}
```
No `drivers.h` change needed (forward-decl already present). No can_common.h dedup
needed (sunnypilot doesn't include opendbc's ignition.h).

### 4. Build panda ON THE DEVICE — DONE ✅
The firmware and the openpilot library must agree on `CAN_PACKET_VERSION_HASH`. The
only guarantee is building on-device against the same on-disk `can.h`.
```bash
ssh comma@192.168.13.154
cd /data/openpilot/panda
source /usr/local/venv/bin/activate
export PYTHONPATH=/data/openpilot:/data/openpilot/opendbc_repo
# expected hash (from the running library):
python3 -c "import hashlib,opendbc,os; d=open(os.path.join(opendbc.INCLUDE_PATH,'opendbc/safety/can.h'),'rb').read().replace(b'\r',b''); print('0x%08X'%int.from_bytes(hashlib.sha256(d).digest()[:4],'little'))"
#   → 0x75ABF276
mkdir -p board/obj
scons -j4
#   → "scons: done building targets", exit 0, builds under -Werror
#   → firmware compiled with -DCAN_PACKET_VERSION_HASH=0x75ABF276U  (MATCHES library ✅)
#   → board/obj/panda_h7.bin.signed produced (96844 bytes)
```
Toolchain lives at `/usr/local/venv/bin/arm-none-eabi-*` (not on default PATH).
The clean `-Werror` build with our edited `can_common.h` is proof the 0x348 block
compiled in; the signed binary timestamp is newer than the source edit.

### 5. Flash the internal car panda (`3f001b000651333038363231`) — DONE ✅
**Pitfall hit first:** a naive `sudo pkill -9 -f manager.py` then flash **fails** —
the `launch_chffrplus.sh` watchdog (running in the `comma` tmux session) **respawns
manager within seconds**, and it reconnects/holds the panda mid-flash. The first
attempt left the panda on OLD fw (sig `b9f396d1`) while the build's signed fw was
`8d76a4f0`; it also produced a python crash log at 23:49 (harmless, stale).

**What worked — kill the watchdog too, and run the flash detached** so an SSH drop
can't interrupt it:
```bash
cat > /tmp/flash_raven.sh <<'SCRIPT'
#!/bin/bash
source /usr/local/venv/bin/activate
exec > /tmp/flash_raven.log 2>&1
sudo pkill -9 -f launch_chffrplus.sh    # kill the WATCHDOG, not just manager
sudo pkill -9 -f launch_openpilot.sh
sudo pkill -9 -f manager.py
sleep 6
cd /data/openpilot/panda
export PYTHONPATH=/data/openpilot:/data/openpilot/opendbc_repo:/data/openpilot/panda
python3 - <<'PY'
from panda import Panda
fw="/data/openpilot/panda/board/obj/panda_h7.bin.signed"
exp=Panda.get_signature_from_firmware(fw).hex()
p=Panda(serial="3f001b000651333038363231")     # ALWAYS explicit serial (avoid spurious 2e0050…)
print("before:", p.get_version(), p.get_signature().hex()[:16])
p.flash(fw); p.close()
import time; time.sleep(2)
p=Panda(serial="3f001b000651333038363231")
got=p.get_signature().hex()
print("after :", p.get_version(), got[:16], "MATCH:", got==exp)
p.close()
PY
SCRIPT
chmod +x /tmp/flash_raven.sh
setsid /tmp/flash_raven.sh < /dev/null > /dev/null 2>&1 &   # detached
# then: cat /tmp/flash_raven.log
```
**Result (verified):**
```
before: DEV-unknown-DEBUG  sig b9f396d1
flash: unlocking / erasing sectors 1-1 / flashing / resetting
after : DEV-fba34f34-DEBUG sig 8d76a4f0   MATCH: True   ✅
```
`8d76a4f0` == the freshly built firmware's signature (and == pandad's *expected*
signature in `FW_PATH`), so pandad will accept it without reflashing.

### 5.5. Recover the openpilot stack (it was killed in step 5) — DONE
Killing `launch_chffrplus.sh` tears down the `comma` tmux session and it does NOT
auto-respawn (`comma.service` is `active (exited)`). Bring the whole stack back the
same way boot does:
```bash
sudo systemctl restart comma.service     # relaunches tmux → comma.sh → manager → pandad
# wait ~30s for manager + UI to boot
```
Verified after restart: `manager UP`, `pandad UP`.

### 6. ★ STILL TODO — FULL IGNITION POWER-CYCLE (you must do this at the car)
A `sudo reboot` / systemctl restart does NOT reload panda **firmware** (the panda
keeps its rails up and runs the OLD image). The freshly-flashed firmware only takes
over for good after a **real ignition cycle: fully OFF → wait 30s → ON.**

**Dual-panda blocker: RESOLVED ✅** (verified with ignition OFF, right after the
step-5.5 restart — pandad came up clean, NO "internal panda missing" loop):
```
pandaStates valid=True       safetyModel=noOutput   pandaType=tres
canValid=True   (1401 CAN msgs in 14s)   faultStatus=none
ignitionLine=False  ignitionCan=False   controlsAllowed=False
```
- `safetyModel=noOutput` and `ignition*=False` are EXPECTED with the car off.
- The SunnyPilot `spi_list()`-priority pandad selected the internal panda and
  **ignored the spurious `2e0050…`** — exactly the behavior BluePilot lacked. The
  whole pivot's open question is answered: **SunnyPilot does not hit the loop.**

After your ignition power-cycle with the **car ON**, re-verify the Raven is live:
```bash
source /usr/local/venv/bin/activate; cd /data/openpilot
PYTHONPATH=/data/openpilot:/data/openpilot/opendbc_repo python3 -c "
import cereal.messaging as messaging, time
sm=messaging.SubMaster(['pandaStates'])
for _ in range(40):
    sm.update(200)
    if sm.updated['pandaStates']: break
for p in sm['pandaStates']:
    if str(p.pandaType)!='unknown':
        print('safetyModel=', str(p.safetyModel), 'ignCan=', p.ignitionCan)
"
# car ON expectation: safetyModel=teslaLegacy, ignCan=True (0x348 GTW_status detected)
```
If `ignCan` stays False with the car on → the 0x348 block isn't matching; check the
Raven is actually putting GTW_status (0x348) on bus 0/1.

### 7. Raven fixed fingerprint + DisableUpdates — DONE ✅
The Raven CANNOT auto-fingerprint (its EPS rejects all FW-version UDS queries). Both
params were set on-device and verified:
```bash
PYTHONPATH=/data/openpilot python3 - <<'PY'
from openpilot.common.params import Params
p=Params()
p.put("CarPlatformBundle", {"platform":"TESLA_MODEL_S_HW3","make":"Tesla","brand":"tesla",
  "model":"Model S (with HW3)","year":["2020","2021","2022","2023"],"package":"All",
  "name":"Tesla Model S (with HW3) 2020-23"})
p.put_bool("DisableUpdates", True)
print("CarPlatformBundle set; DisableUpdates on")
PY
```
Verified: `CarPlatformBundle` = TESLA_MODEL_S_HW3 bundle, `DisableUpdates=True`,
and the interface loads: `carFingerprint=TESLA_MODEL_S_HW3 safetyModel=teslaLegacy`.
⚠️ **STICKY** — this forces EVERY car to be seen as the Raven. Before moving the
device to the **Ford**, clear it or the Lightning is mis-detected:
```bash
PYTHONPATH=/data/openpilot python3 -c "from openpilot.common.params import Params; Params().remove('CarPlatformBundle')"
```

---

## Issues encountered & resolved

| Issue | Resolution |
|-------|-----------|
| Local `sunnypilot/opendbc_repo` submodule empty; pinned commit `ac4b8d6` a "bad object" | The Raven port lives in the standalone `/home/dp/gh/comma/opendbc` @ `ac4b8d69` (branch `xnor2sunny`). Overlay sourced from there. |
| Will overlaying clobber `release-tizi`-specific Tesla changes? | No — device Tesla files match the port's base exactly (155/155/70 lines). Clean overlay. |
| Does the port change the CAN hash (the BluePilot disaster)? | No — `can.h` byte-identical (`76f2ab75…`). Hash stays `0x75ABF276`. Confirmed in the on-device build. |
| `grep teslaLegacy safety.h` returned 0 | Expected — `safety.h` uses the C names `SAFETY_TESLA_LEGACY` / `tesla_legacy_hooks`; `teslaLegacy` is only the capnp enum name. Registration is correct. |
| Where does the 0x348 ignition logic belong on sunnypilot? | In `panda/board/drivers/can_common.h::ignition_can_hook` (xnor does the same), NOT in `opendbc/safety/ignition.h`. No dedup needed. |
| `arm-none-eabi-objdump/nm` "command not found" | Toolchain is under `/usr/local/venv/bin`, not on PATH. scons resolves it via its build env. The `-Werror` build + signed-binary timestamp are sufficient proof. |
| **capnp crash: "Skipped ordinal @35. Ordinals must be sequential"** — every cereal import died | Port added `teslaLegacy @36` after `volkswagenMeb @34`, leaving a hole at `@35`. Added placeholder `mg @35;` (step 2.5), fixed on device AND in source repo. |
| First flash left panda on OLD fw (sig `b9f396d1` ≠ build `8d76a4f0`) | `launch_chffrplus.sh` watchdog respawned manager mid-flash and it grabbed the panda. Fix: kill the watchdog (`launch_chffrplus.sh`) too, run flash **detached** via `setsid` + logfile (step 5). Re-flash → MATCH True. |
| After flash, no processes publishing / `pandaStates` empty | Killing the watchdog tore down the `comma` tmux session; it doesn't auto-respawn. Recover with `sudo systemctl restart comma.service` (step 5.5). |
| **Dual-panda "internal panda missing" loop (the whole reason for the pivot)** | **Did NOT occur on SunnyPilot.** After flash + stack restart: `pandaStates valid=True`, `canValid=True`, internal panda selected via `spi_list()`, spurious `2e0050…` ignored. Blocker resolved. |
| **Reboot WIPED everything** — opendbc overlay gone, panda reflashed to stock, `CarPlatformBundle` cleared | The boot **overlay-swap** in `launch_chffrplus.sh` replaces `/data/openpilot` with a clean finalized tree when `${STAGING_ROOT}/finalized/.overlay_consistent` exists; pandad then reflashes the panda from the (stock) `FW_PATH`. `DisableUpdates` alone does NOT stop the *swap*. Fix = freeze guards (below). |
| **0x348 ignition not detected with car merely screen-on** | Correct behavior. Ignition bit = `GTW_driveRailReq` (`data[0]&0x1`); only true when car is truly **READY** (brake, in drive). Confirmed `ignCan=True`, 140/140 samples, when ready. |
| **0x348 "disappeared" / only bus 0 has traffic** | During FW-fingerprinting pandad enables **OBD multiplexing** (ELM327 safety, `ObdMultiplexingEnabled`), which reroutes bus 1 → OBD port. Don't clear `CarParams` / kill `card` mid-session; let it finish so safety→`teslaLegacy` and bus 1 returns. |
| **Repeated mid-boot process kills left launch hung at "launching system reset, got taps"** | Don't iteratively `pkill` during boot. Recover with one clean `sudo reboot` (guards make it persist) or `tmux new-session -s comma -d /usr/comma/comma.sh`. |
| **"No panda" after a power-plug pull** — internal `3f001b…` absent from `spi_list()`, only spurious `2e0050…` on USB | The internal panda hung on SPI. `HARDWARE.reset_internal_panda()` (GPIO `STM_RST_N`) recovers it **only when nothing else holds the panda** — must stop manager+pandad first. A clean reboot also recovers it. Panda is fine (13.9V, harness=2). |
| **★ "Unknown Vehicle Variant" alert onroad (even though Vehicle tab shows TESLA_MODEL_S_HW3)** | Misleading text — SunnyPilot maps `EventName.canError` → the string *"Unknown Vehicle Variant"* (`events.py:770`). Root cause was **`card` crash-looping** (see next row), which raised `canError`+`processNotRunning`. NOT a fingerprint problem — the Raven fingerprints correctly. |
| **★ `card` crash-loops: `AssertionError` in CANParser, `carstate.py:238` RCM_status** | The Raven (fixed-fingerprint → empty `fingerprint` dict → `0x201` never present → `NO_SDM1` flag always set) reads `RCM_status->RCM_buckleDriverStatus` for the seatbelt, but `tesla_can.dbc` lacked `RCM_status` (529). **Fix: added RCM_status (529) to `tesla_can.dbc`** (from xnor). Confirmed live: car sends 529 on bus 1 (NOT 513/SDM1), so NO_SDM1 is correct; DBC was just incomplete. card now stays up, alert cleared (`EVENTS: []`). |
| **`card` resolves the Raven but live panda stays `elm327` / `ControlsReady=False`** | Downstream of the card crash. pandad's `configureSafetyMode` only pushes the real safety model (`teslaLegacy`) once `is_onroad && ControlsReady`. card crashing → no CarParams on bus → controlsd never sets `ControlsReady` → panda stuck elm327. Fixed by the RCM_status DBC fix (card stays up → ControlsReady → teslaLegacy). |
| **Car "awake for charging" ≠ drive-ready; CAN buses silent** | When parked/charging, the car powers down chassis/powertrain CAN — 0 msgs, no 0x348, no ignition — even though panda reads 13.9V. Must be in actual **Drive/READY** (foot on brake) for CAN + 0x348 to flow. Confirmed: in READY, 8000+ msgs/s, 0x348 `driveRailReq=1`, `ignCan=True`. |

### ★ Persistence guards — REQUIRED so a reboot doesn't wipe the overlay
The boot swap is skipped when the dir looks locally-modified and no finalized update is pending. Set ALL of:
```bash
sudo rm -rf /data/safe_staging/finalized          # remove any pending swap
touch -d "2020-01-01" /data/openpilot/.overlay_init # mark tree as locally modified (launcher skips swap)
touch /data/openpilot/prebuilt                      # skip ./build.py rebuild on boot
# DisableUpdates=1 (via the "Allow auto updates" toggle OFF) — updated.py exits, never finalizes a new swap
```
Verified: with these set, a clean reboot preserved the 0x348 block, `tesla_legacy.h`, the opendbc
overlay, and the flashed panda firmware (`8d76a4f0`).

---

## "Allow auto updates" toggle (replaces hidden "Disable Updates")

`selfdrive/ui/sunnypilot/layouts/settings/software.py` (`SoftwareLayoutSP`) now shows an
**"Allow auto updates"** toggle in Settings → Software:
- **OFF by default** (`initial_state = not DisableUpdates`; device ships `DisableUpdates=1`).
- **OFF → `DisableUpdates=True`** → `updated.py` exits at startup (`updated.py:405`), so it never
  fetches AND never creates `finalized/.overlay_consistent` → the boot overlay-swap can't fire either.
  This blocks the *entire* update path, which is exactly what protects our on-device changes.
- **ON → `DisableUpdates=False`** → normal updates resume.
- Always visible (removed the old `ShowAdvancedControls` gate); editable only offroad; prompts a
  reboot to take effect.
It's a pure UI change over the already-registered+enforced `DisableUpdates` param — no C++ rebuild.

---

## Re-deploy / recovery script

`/home/dp/gh/comma/deploy_xnor2sunny.sh` re-applies the whole overlay idempotently (freeze guards →
15 opendbc files → 0x348 block → UI toggle → on-device panda build → Raven params). Flags:
`--flash` (also flash the panda, detached), `--reboot` (clean reboot at end). Run it whenever the
device may have reverted or to push fresh changes. Requires the car **awake**.

---

## Quick failure-mode lookup (carried from BluePilot, mapped to SunnyPilot)

| Symptom | Cause | Fix |
|---------|-------|-----|
| `AttributeError: CarHarness has no tesla_model_x_hw1` | docs_definitions missing | branch (step 2) |
| `KeyError: 'TESLA_MODEL_S_HW3'` | torque override missing | branch (step 2) |
| `AttributeError: SafetyModel has no teslaLegacy` | car.capnp not registered | branch (step 2) |
| panda build `conflicting types for ignition_can_hook` | (BluePilot/dirk only) | N/A on sunnypilot — no opendbc/ignition.h include |
| "CANbus disconnected / faulty cable", `canValid=False` | CAN hash mismatch | step 4 build-on-device — verified impossible (can.h unchanged) |
| panda still on old fw after flash | soft reboot insufficient | step 6 ignition power-cycle |
| "Internal panda is missing" loop | dual-panda; pandad rejects internal | SunnyPilot `spi_list()` priority should avoid it — verify at step 6 |
| Raven not recognized | EPS can't auto-fingerprint | step 7 fixed fingerprint |
| Ford detected as Tesla after swap | sticky CarPlatformBundle | step 7 cleanup |

---

## Phase 2 (later) — bring BluePilot Ford/Lightning support onto this base

Once the Raven drives on SunnyPilot, re-apply the BluePilot Ford changes: Ford
control logic (`opendbc/car/ford/carcontroller.py` + `lateral_curv_ext.py`), the
2025 F-150 Lightning fingerprint (`ford/fingerprints.py` `TL38-2D053-AD` ABS,
`RB5T-14D049-AB` radar) + `ford/values.py` year range, and the `FordPref*`/`BP*`
params for any ported features. Find touchpoints via `grep -r "BluePilot:"` in the
bluepilot worktree. Goal: one SunnyPilot install driving BOTH the Lightning and the
Raven — on a base whose panda we can flash.

---

## Software updates (DisableUpdates)

After deploy, **`DisableUpdates` must be `1`** or the auto-updater silently
overwrites the on-device changes. A full reinstall resets this param and the SSH
host key — re-set it immediately after any reinstall, before the updater runs.
```bash
PYTHONPATH=/data/openpilot python3 -c "from openpilot.common.params import Params; Params().put_bool('DisableUpdates', True)"
cat /data/params/d/DisableUpdates   # must print 1
```
