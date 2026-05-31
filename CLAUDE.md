# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What is this

This is the **BluePilot 6.0 branch** (`bp-6.0`) — the current development release of BluePilot, forked from `BluePilotDev/bluepilot` and mirrored to `dirkpetersen/bluepilot`. It is a Ford-focused fork of SunnyPilot (itself a fork of commaai/openpilot). Version: `6.0.0`.

This worktree is **read-reference / cherry-pick source**. Custom changes (Tesla, Lightning fingerprint) live in the `bluepilot-dirk` worktree at `/home/dp/gh/comma/bluepilot`.

See the root CLAUDE.md at `/home/dp/gh/comma/CLAUDE.md` for the full multi-worktree setup.

## Build & Test

```bash
tools/op.sh setup
source .venv/bin/activate

scons -u -j$(nproc)            # full build
scons -u -j$(nproc) --minimal  # skip tests/tools

pytest                          # all tests (parallel)
pytest path/to/test.py::Class::test_name
pytest -m 'not slow'
pytest -m tici                  # device-only tests

ruff check .
ruff format --check .
codespell
ty check
```

## Architecture

BluePilot uses a **three-layer inheritance architecture** — see [`AGENTS.md`](AGENTS.md) for full detail.

```
Layer 1: commaai/openpilot    → selfdrive/ui/onroad/
Layer 2: sunnypilot           → selfdrive/ui/sunnypilot/onroad/
Layer 3: BluePilot (Ford)     → selfdrive/ui/bp/onroad/
```

- `selfdrive/ui/layouts/main.py` is the key wiring file — conditional imports swap stock classes for BP versions
- BluePilot classes use `BP` suffix; files use `_bp.py`
- Changes to upstream files are wrapped: `# BluePilot: <reason>` … `# End BluePilot`
- Find all touchpoints: `grep -r "BluePilot:"`

### Ford control

All Ford-specific lateral and longitudinal logic:
```
opendbc_repo/opendbc/car/ford/carcontroller.py
```
Uses Params feature flags (not inheritance). Gas and brake must never be simultaneously nonzero.

### Key files

| Task | File |
|------|------|
| UI class wiring | `selfdrive/ui/layouts/main.py` |
| Ford lateral/longitudinal control | `opendbc_repo/opendbc/car/ford/carcontroller.py` |
| BP onroad renderers (TICI) | `selfdrive/ui/bp/onroad/` |
| BP onroad renderers (MICI) | `selfdrive/ui/bp/mici/onroad/` |
| BP settings menu | `selfdrive/ui/bp/layouts/settings/bluepilot.py` |
| Parameter definitions | `bluepilot/params/params.json` |
| Portal server | `bluepilot/backend/bp_portal.py` |
| Process registry | `system/manager/process_config.py` |

### Conventions

- All quantities in SI units unless the field name says otherwise.
- Use `openpilot.*` import roots — bare `selfdrive`/`common`/`system` imports are banned (`ruff TID251`).
- Use `time.monotonic`, not `time.time`.
- Fork-specific Cap'n Proto messages go in `cereal/custom.capnp`, never `log.capnp`.
- Tests marked `@pytest.mark.tici` only run on-device.
