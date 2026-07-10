# openpilot architecture, build, test

The "big picture" of an openpilot checkout — enough to find where a behavior lives and to build/test
a change. Applies to every fork (commaai/sunnypilot/bluepilot/xnor); fork-specific layers are noted.

## Table of contents

- [Process model](#process-model)
- [Messaging (cereal / msgq)](#messaging-cereal--msgq)
- [The car support layer](#the-car-support-layer)
- [Where common behaviors live](#where-common-behaviors-live)
- [Build / test / lint](#build--test--lint)
- [Repo conventions (enforced)](#repo-conventions-enforced)

## Process model

openpilot is a set of **independent processes** communicating over a pub/sub message bus (**msgq**),
supervised by `manager`. There is no monolith — to change a behavior, find the process that owns it.

| Process | Path | Role |
|---------|------|------|
| `manager` | `system/manager/` | launches & supervises all processes |
| `selfdrived` | `selfdrive/selfdrived/` | top-level state machine (enabled/disabled), alert/event generation |
| `controlsd` | `selfdrive/controls/controlsd.py` | lateral + longitudinal control → actuator commands |
| `plannerd` | `selfdrive/controls/plannerd.py` | longitudinal + lateral planning |
| `modeld` | `selfdrive/modeld/` | driving neural network (tinygrad runtime) |
| `dmonitoringmodeld` | `selfdrive/modeld/` | driver-monitoring NN (face/pose) |
| `pandad` | `selfdrive/pandad/` | CAN/safety bridge to panda hardware (holds the SPI lock) |
| `card` | `selfdrive/car/card.py` | car interface: CAN parse ↔ actuator output; instantiates the brand interface |
| `radard` | `selfdrive/controls/radard.py` | radar tracks → `liveTracks`/`radarState` |
| `locationd` | `selfdrive/locationd/` | Kalman filter for pose/velocity |
| `camerad` | `system/camerad/` | camera capture |
| `loggerd` | `system/loggerd/` | logs all messages to disk |
| `ui` | `selfdrive/ui/` | Raylib on-device UI (settings, onroad HUD) |
| `athena` | `system/athena/` | device↔cloud (SSH tunnel, OTA) |

A subtle but important coupling: if `card` doesn't publish a message a downstream consumer expects
(e.g. `radard` never emits `liveTracks` because `RadarInterface.update()` returns `None`), the
consumer **stalls** and surfaces as a `commIssue`/process-not-alive error far from the real cause.
The Raven `radarUnavailable` fix is exactly this class of bug — see `references/cars.md`.

## Messaging (cereal / msgq)

- Messages are **Cap'n Proto**, schemas in `cereal/log.capnp` and `cereal/car.capnp`.
- **Fork-specific messages go in `cereal/custom.capnp`, never in `log.capnp`** — keeps merges with
  upstream clean.
- Cap'n Proto fields are **ordinal-numbered** and the ordinals must be contiguous; a hole (a missing
  `@N`) crashes *every* cereal import. (A real bug here: a missing `mg @35;` next to
  `teslaLegacy @36;` crashed all imports until the hole was filled — see `HANDOFF.md`.)
- All quantities are **SI units** unless the field name says otherwise (e.g. `…Mph`).

## The car support layer

Each brand lives under `opendbc_repo/opendbc/car/<brand>/` (in the standalone `opendbc/` repo it's
`opendbc/car/<brand>/`):

| File | Owns |
|------|------|
| `values.py` | the `CAR` enum, `CarSpecs` (mass/wheelbase/steerRatio), platform flags, `FW_QUERY_CONFIG` |
| `fingerprints.py` | FW version strings used to identify the car at boot |
| `carstate.py` | raw CAN → `CarState` (speed, steering, gear, buttons, seatbelt, …) |
| `carcontroller.py` | actuator commands → outbound CAN frames |
| `interface.py` | `CarInterfaceBase` subclass; sets params (`radarUnavailable`, safety flags, …) |
| `radar_interface.py` | radar tracks → `RadarData` |

DBCs (CAN signal definitions) live in `opendbc/dbc/*.dbc`. Safety models (C) live in
`opendbc/safety/` — see `references/panda-and-safety.md`.

## Where common behaviors live

| Behavior | Where to look |
|----------|---------------|
| Driver monitoring timeouts/thresholds | `selfdrive/monitoring/helpers.py` (see DMON.md for the dual-counter design) |
| Lateral / lane-change logic | `selfdrive/controls/` planner + the `desire`/lane-change state machine (LANE.md) |
| Longitudinal / speed targets | planner + map/vision speed sources (LONG.md) |
| UI settings toggles | `selfdrive/ui/layouts/settings/toggles.py` (+ `common/params_keys.h` for the param) |
| Param definitions & defaults | `common/params_keys.h` (defaults compiled into `params_pyx.so`) |
| Fork UI variants | bluepilot: `selfdrive/ui/bp/`; sunnypilot: `selfdrive/ui/sunnypilot/` |

## Build / test / lint

Run inside an openpilot worktree, in that fork's venv:

```bash
tools/op.sh setup && source .venv/bin/activate   # one-time per fork; creates .venv
scons -u -j$(nproc)              # full build
scons -u -j$(nproc) --minimal    # minimal (skips tests/tools) — faster iteration
scons -u -j$(nproc) <target>     # single target, e.g. common/params_pyx.so

pytest                            # all tests
pytest path/to/test.py::Class::test_name   # one test
pytest -m 'not slow'              # skip slow tests
# @pytest.mark.tici tests run ON-DEVICE only

ruff check . && ruff format --check . && codespell && ty check
```

Build system is **SCons** (`SConstruct`); platform auto-detected (`x86_64`, `aarch64` on-device,
`larch64`, `Darwin`). Native deps are **vendored** — do not add apt-installed system libs outside
the `SConstruct` allowlist; it breaks the on-device build.

On the device there is no `.venv`; build with `PATH=/usr/local/venv/bin:$PATH scons …` (see
`references/deploy-toolchain.md`).

## Repo conventions (enforced)

- **Import roots**: use `openpilot.*` (e.g. `from openpilot.common.params import Params`). Bare
  `selfdrive`/`common`/`system` imports are **banned** by `ruff TID251` and will fail lint.
- **Time**: use `time.monotonic`, not `time.time`, for anything measuring elapsed time.
- **BluePilot edits**: wrap changes to upstream files in `# BluePilot: <reason>` … `# End BluePilot`
  so `grep -r "BluePilot:"` finds every merge touchpoint.
- **Branch rule**: never commit/push to `main`/`master`/`bp-dev`; a `PreToolUse` hook blocks it.
