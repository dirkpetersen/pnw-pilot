# CLAUDE-PNW.md — pnw-pilot branch map

Overview of every branch in `dirkpetersen/pnw-pilot` and what it is for. PNW is our production
openpilot distribution for the **Seattle ↔ Corvallis I-5 corridor**, forked from
`xnor-tech/openpilot` (which carries the proven Tesla **Raven** HW1/HW2/HW3 drivers).

> Lineage: `commaai/openpilot` → `xnor-tech/openpilot` (Tesla Raven) → `dirkpetersen/pnw-pilot` (PNW).
> Tips drift — this is the map, not the SHAs. Refresh with
> `git for-each-ref --format='%(refname:short) %(subject)' refs/remotes/origin/`.

---

## Production / distribution branches

| Branch | Purpose |
|--------|---------|
| **`main`** | **DEFAULT branch.** The shippable PNW distribution = `master` + the PNW README intro. What users and the openpilot installer see. |
| `master` | **Pristine stock / xnor mirror** — kept clean (no PNW changes). Updated from `xnor-tech/openpilot`; we then decide what to merge into `main`. Do **not** sync `master` up to `main`. |
| `pnw-submodules` | Submodule wiring: repoints `opendbc`/`panda` to `../pnw-opendbc.git` / `../pnw-panda.git`. Carries the PNW distribution overview at the top of its README. |
| `testing` | Full snapshot of **everything currently LIVE on the comma 3X** — `integration2xnor` + bsm + fingerprint + 2025 Ford fingerprint + SecOC fix + relaxed driver monitoring. See `TESTING.md`. Excludes undeployed `light2xnor` Tier-2 and `upload2xnor`. |
| **`pnwprod`** | **Production install branch for the comma.ai installer.** Install URL: `installer.comma.ai/dirkpetersen/pnwprod`. The stable PNW build users flash. |
| **`pnwtest`** | **Test/staging install branch for the comma.ai installer.** Install URL: `installer.comma.ai/dirkpetersen/pnwtest`. Pre-production validation before promoting to `pnwprod`. |

> **comma.ai installer branches:** the device setup screen takes
> `installer.comma.ai/dirkpetersen/<branch>`, which clones `dirkpetersen/openpilot` (GitHub-redirects
> to `dirkpetersen/pnw-pilot`) at `<branch>`. The two installer-facing branches are **`pnwprod`**
> (production) and **`pnwtest`** (test/staging).

## Integration branches (feature aggregation)

| Branch | Purpose |
|--------|---------|
| `integration2xnor` | **The DEPLOYED merge** of the feature efforts: network + connect + light-ces-gentle + ces + auto + dmon param keys + ceslog. Source of truth for what's on the device. |
| `integration-device` | Older feature-aggregation branch — **superseded** by `integration2xnor`. |

## Feature efforts (`*2xnor` — features ported onto the xnor base)

| Branch | Purpose |
|--------|---------|
| `network2xnor` | WiFi/LTE/hotspot: tethering NAT fix, perpetual tethering, priority-WiFi switching, GPS geo-gated scanning + Set Home Location button, LTE PDN-throttle guard, gsm-enforcer removed. |
| `connect2xnor` | Drive-data upload: two-pass (small files auto → video on real WiFi), deleter preserves un-uploaded data, firehose=uploading indicator, connect WiFi-only. |
| `ces2xnor` | **Conditional Experimental Switching (CES) + Vision Turn Speed Control (VTSC)** core — chill by default, auto-Experimental for curves/low-speed/stop-lights/slow-lead; VTSC caps cruise speed through curves (Terwilliger-calibrated). |
| `ceslog2xnor` | Adds a `cesState` cereal message that logs CES decisions (merged into `integration2xnor`). |
| `light-ces-gentle` | CES 3-way selector **Off / Light / Standard** (param `CESMode`) + a gentle Lightning VTSC profile + grey-out fix. |
| `light2xnor` | Ford F-150 Lightning **Tier 2**: radar (camera lead) + opt-in openpilot longitudinal. **NOT deployed** (excluded from `testing`). |
| `auto2xnor` | Nudgeless lane change + no-disengage-on-brake (incl. on the Ford F-150 Lightning). |
| `mapd2xnor` | OSM speed-limit display + lower-limit warnings (PNW = WA/OR/ID map data). |
| `bsm2xnor` | Tesla Raven blind-spot monitoring from `AutopilotStatus` (CAN `0x399`). |
| `fingerprint2xnor` | Show last-known car offroad; never persist a MOCK fingerprint over a good one (fixes the off-state "dashcam" home screen). |
| `fordsecoc2xnor` | Don't false-flag a SecOC "dashcam" when camera messages are absent at fingerprint time. |
| `ford-lightning-2025-fingerprint` | 2025 F-150 Lightning fingerprint (marks `Ecu.eps` non-essential for the match). |
| `upload2xnor` | Gate uploads on a home-WiFi geofence + crash-safety. **NOT deployed** (excluded from `testing`). |
| `dmon2xnor` | Relaxed dual-counter driver monitoring (5 min pose / 15 min phone). |
| `dmon2xnor-b` | `dmon2xnor` + the offline-CPU-core crash fix (`dmonitoringd` "Process Not Running"). |
| `dmon` | Base driver-monitoring test branch. |

## Cross-fork porting (features leaving PNW — reference)

| Branch | Purpose |
|--------|---------|
| `xnor2sunny` | Tesla Raven → sunnypilot. |
| `xnor2bp` | Tesla Raven → BluePilot (**deferred** — dual-panda flash blocker). |
| `dmon2sunny` | Driver-monitoring port → sunnypilot. |

## Base / reference branches (inherited from the xnor-tech network — not PNW work)

| Branch(es) | Purpose |
|--------|---------|
| `xnor`, `xnor-dirk`, `xnor-dev`, `xnor-c3`, `xnor-c3-dev` | xnor-tech base branches. |
| `rx`, `rx-src`, `rx-dev`, `rx-dev-src`, `rx-fix`, `rx-fix-src`, `rx-master`, `rx-master-development` | xnor-tech "rx" reference branches. |
| `tesla` | opendbc Tesla Model S HW1/HW2/HW3 (Raven) legacy support (reference). |
| `tesla-unity` | Tesla unity UI variant (reference). |
| `bluepilot-dirk`, `sunnypilot-dirk`, `bp-6.0`, `dirk` | Other-fork base branches (reference). |

---

**Deploy reality:** the device runs `/data/openpilot` as a **file overlay**, not a git checkout — we
don't push branches to the car. The deploy toolchain (`patch-*.py`, `update-*.sh`, `upload.sh`,
`COMMA_IP`, persistence guards) and the device source-of-truth live at the root workbench
`~/gh/comma/` (`DEVICE-STATE.md`). This repo holds *what* ships; the root holds *how* it lands.

## External documentation

- **xnor wiki** — <https://wiki.xnor.shop/docs/> (the xnor-tech base: Tesla Raven install, supported
  cars/hardware, the `xnor` / `xnor-c3` / `tesla-unity` versions).
- **openpilot docs** — <https://docs.comma.ai/> (upstream openpilot architecture, car port layer,
  tools, safety).
