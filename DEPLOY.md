# DEPLOY.md — Deploying the `xnor2bp` branch (Ford + Tesla Raven on BluePilot 6.0)

Step-by-step to deploy this branch to a comma device and get **both** the Ford
F-150 Lightning and the Tesla Model S HW3 Raven working. Follow in order — the
ordering matters (each step has bitten us when skipped).

Device: `comma@192.168.13.154` (host key changes on reinstall → `ssh-keygen -R 192.168.13.154`).

---

## What this branch already contains (no manual patching needed)

Everything that used to be applied by the on-device `patch-raven.py` is now
**committed to this branch**, so a clean checkout has it all:

- Raven legacy opendbc port (`tesla/*.py`, `teslacan_legacy.py`, `tesla_raven_party.dbc`, `tesla_legacy.h`)
- Ford F-150 Lightning 2025 fingerprint
- Safety-model registration: `car.capnp` `teslaLegacy @36` + `declarations.h` + `safety.h` hook
- `car/__init__.py` `Bus.ap_pt`
- **`docs_definitions.py`** Tesla harness entries (else: `AttributeError: CarHarness has no attribute tesla_model_x_hw1`)
- **`torque_data/override.toml`** legacy Tesla rows (else: `KeyError: 'TESLA_MODEL_S_HW3'`)
- **`opendbc/safety/ignition.h`** with Tesla 0x348 (else: panda build `fatal error: ignition.h`)
- carstate seatbelt SDM1 branch, `tesla_can.dbc` superset (RCM_status), legacy fingerprints
- `.gitmodules` + submodule pin → `dirkpetersen/panda@dirk` (`ca373f1f`)

The **only** things NOT in this branch (they live in the panda submodule or are
runtime params) are handled by the steps below: the panda `can_common.h` dedup,
the panda build+flash, and the fixed-fingerprint param.

---

## Deploy steps

### 0. Pre-flight
```bash
ssh-keygen -f ~/.ssh/known_hosts -R 192.168.13.154   # if reinstalled
ssh comma@192.168.13.154 'cat /data/openpilot/launch_env.sh >/dev/null && echo reachable'
```

### 1. Get the branch onto the device's `/data/openpilot`
Use your normal install method, then check out `xnor2bp`, OR (if iterating) copy the
changed files. The branch is based on pristine `bp-6.0` (`e52611b`), so the changed
file set vs the device is exactly:
```bash
# from the bluepilot worktree:
git diff --name-only e52611bb1c..xnor2bp | grep -v '^panda$'
```
Copy each of those to `/data/openpilot/<path>` on the device.

### 2. Point the panda submodule at the dirk panda
```bash
ssh comma@192.168.13.154
cd /data/openpilot/panda
git fetch https://github.com/dirkpetersen/panda.git dirk
git checkout ca373f1f4217d7361e18dae8d12f8ab92c4b9066
```

### 3. Panda build prereq — dedup `can_common.h`
The dirk panda's `board/drivers/can_common.h` BOTH includes
`opendbc/safety/ignition.h` AND defines a local `ignition_can_hook` → build fails
with `conflicting types for 'ignition_can_hook'`. Remove the local copy:
```bash
cd /data/openpilot
python3 - <<'PY'
import re
p="/data/openpilot/panda/board/drivers/can_common.h"
src=open(p).read()
if "ignition_can_hook removed" in src:
    print("already deduped"); raise SystemExit
m=re.search(r'void ignition_can_hook\(CANPacket_t \*msg\) \{', src)
start=m.start(); depth=0; i=m.end()-1
while i<len(src):
    if src[i]=='{': depth+=1
    elif src[i]=='}':
        depth-=1
        if depth==0: end=i+1; break
    i+=1
src=src[:start]+"// ignition_can_hook removed — provided by opendbc/safety/ignition.h\n"+src[end:]
open(p,"w").write(src); print("deduped")
PY
```
(`opendbc/safety/ignition.h` is already on disk from step 1 — it's committed to the branch.)

### 4. ★ CRITICAL — build the panda ON THE DEVICE against its own `can.h`
The panda firmware and the openpilot python library must agree on
`CAN_PACKET_VERSION_HASH`. The library computes it from
`opendbc/safety/can.h` at runtime; the firmware bakes it in at build time. **If
they differ, every CAN packet is rejected → `canValid=False` → "CANbus
disconnected / likely faulty cable"** (this cost us a full debugging saga — the
cable was never the problem).

The ONLY way to guarantee they match is to build the panda **on the device**, so
both use the same on-disk `can.h`. Do NOT flash a panda built elsewhere.
```bash
cd /data/openpilot/panda
source /usr/local/venv/bin/activate
export PYTHONPATH=/data/openpilot:/data/openpilot/opendbc_repo
# print the hash the build WILL use (informational — it just has to match the lib):
python3 -c "import hashlib,opendbc,os; d=open(os.path.join(opendbc.INCLUDE_PATH,'opendbc/safety/can.h'),'rb').read().replace(b'\r',b''); print('CAN_PACKET_VERSION_HASH=0x%08X'%int.from_bytes(hashlib.sha256(d).digest()[:4],'little'))"
# on bp-6.0 this prints 0x135F8827 — that's fine; it just must match the running library.
mkdir -p board/obj
scons -j4            # build; confirm "BUILD RC: 0" and the logged hash matches the line above
```

### 5. Flash the internal car panda (serial `3f001b000651333038363231`)
Stop manager first so pandad releases the SPI lock:
```bash
sudo pkill -9 -f manager.py; sleep 5
cd /data/openpilot/panda
PYTHONPATH=/data/openpilot:/data/openpilot/opendbc_repo:/data/openpilot/panda \
python3 -c "from panda import Panda; p=Panda(serial='3f001b000651333038363231'); print('before:',p.get_version()); p.flash(); print('flashed')"
```

### 6. ★ FULL IGNITION POWER-CYCLE (not a reboot)
A `sudo reboot` is NOT enough — the comma 4's internal panda keeps its power rails
up through a soft reboot and keeps running the OLD firmware. You MUST:
**ignition fully OFF → wait 30s → ON.** (Confirmed repeatedly: after reboot the
panda still reported the old version; only an ignition cycle loaded the new one.)

### 7. Set the Raven fixed fingerprint (Tesla only)
The Raven CANNOT auto-fingerprint (its EPS rejects all FW-version UDS queries).
Select it manually:
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
**⚠️ STICKY — this forces EVERY car to be seen as the Raven.** Before moving the
device back to the **Ford**, you MUST clear it or the Lightning is mis-detected:
```bash
PYTHONPATH=/data/openpilot python3 -c "from openpilot.common.params import Params; Params().remove('CarPlatformBundle')"
```
(Or use the on-device UI: Settings → Vehicle selector — tap to clear.)

### 8. Verify (car ON, in drive)
```bash
PYTHONPATH=/data/openpilot:/data/openpilot/opendbc_repo python3 -c "
from opendbc.car.tesla.interface import CarInterface
print(CarInterface.get_non_essential_params('TESLA_MODEL_S_HW3').carFingerprint)"
# expect: TESLA_MODEL_S_HW3 (no AttributeError, no KeyError)
```
On the message bus (car on): expect `canValid=True`, `canTimeout=False`,
`panda safety=teslaLegacy`. The `interruptRateCan2` fault and CAN1 `stuffError`
are HARMLESS background noise on the Raven bus — the working xnor install shows
them too. Ignore them.

---

## Failure modes → which step fixes them (quick lookup)

| Symptom | Cause | Fixed by |
|---------|-------|----------|
| `AttributeError: CarHarness has no attribute tesla_model_x_hw1` | docs_definitions missing harness entries | branch (committed) — ensure step 1 copied `docs_definitions.py` |
| `KeyError: 'TESLA_MODEL_S_HW3'` (get_torque_params) | torque override missing legacy rows | branch (committed) — ensure step 1 copied `torque_data/override.toml` |
| panda build `fatal error: opendbc/safety/ignition.h: No such file` | ignition.h missing | branch (committed) — ensure step 1 copied `opendbc/safety/ignition.h` |
| panda build `conflicting types for 'ignition_can_hook'` | can_common.h duplicate | step 3 |
| `AttributeError: SafetyModel has no attribute teslaLegacy` | car.capnp not registered | branch (committed) — `car.capnp` + `declarations.h` + `safety.h` |
| "CANbus disconnected / faulty cable", `canValid=False` | panda fw vs library CAN hash mismatch | step 4 (build on-device) + step 6 (power-cycle) |
| panda still on old fw after flash | soft reboot insufficient | step 6 (ignition power-cycle) |
| Raven not recognized / no carParams | EPS can't auto-fingerprint | step 7 (fixed fingerprint) |
| Ford detected as Tesla after car swap | sticky CarPlatformBundle left set | step 7 cleanup (remove CarPlatformBundle) |

See `../RAVEN.md` and `../XNOR2BP.md` (repo root) for the full root-cause writeups.
