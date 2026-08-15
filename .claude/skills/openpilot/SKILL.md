---
name: openpilot
description: >-
  Expert guide for the multi-fork openpilot self-driving workbench at ~/gh/comma:
  the PNW production distribution (pnw-pilot, the lebowski model era), working
  across the commaai / sunnypilot / bluepilot / xnor forks, porting features
  between them, and operating the physical comma 3X running a Tesla Model S HW3
  (Raven) and a Ford F-150 Lightning. Use this skill WHENEVER the task touches
  openpilot, comma devices, panda firmware, opendbc, CAN bus, car
  fingerprinting, driver monitoring, mapd/OSM, CES/VTSC, or lateral/longitudinal
  control — or mentions the forks (pnw-pilot, bluepilot, sunnypilot, xnor), the
  cars (F-150 Lightning, Model S, Raven, Tesla, Ford), the feature branches
  (*2pnw, *2xnor, 3devpnw, 4devpnw, 3testpnw, lebowski2pnw), or device concepts
  (/data/openpilot, auto-update channels, persistence guards, panda flashing,
  COMMA_IP). Trigger even when the user is clearly working in ~/gh/comma or on
  these cars without naming openpilot. For the actual deploy procedure, ALSO
  load the pnw-pilot-deploy skill.
---

# openpilot multi-fork workbench

This skill is the operating manual for **`~/gh/comma/`** — a workbench for developing
[openpilot](https://github.com/commaai/openpilot) across several forks and running it on two real
cars via the **PNW distribution** (`dirkpetersen/pnw-pilot`). It exists because this setup deviates
from a stock checkout in ways that will bite you if you assume defaults.

Read this whole file first — it's the map. Then load the one or two `references/` files relevant to
the task. **`references/pnw-era.md` is the current-era picture (2026-07)** — prefer it when it and
an older reference disagree.

## Before you touch anything: orient

The single most common mistake here is acting on a stale mental model. The repo evolves FAST — in
one 3-day sprint (2026-07-07..10) the device gained a new driving model, auto-updates, and a dozen
features; any cached assumption can be wrong. So **verify, don't assume**:

```bash
cat ~/gh/comma/CLAUDE.md                        # authoritative workbench map, kept current
cat ~/gh/comma/docs/DEVICE-STATE.md | head -30  # what is ON THE CAR right now (source of truth)
git -C ~/gh/comma/pnw/pnw-pilot branch --show-current
```

If this skill and CLAUDE.md/DEVICE-STATE.md disagree, trust the repo docs and tell the user about
the drift.

## The mental model (2026-07 era)

1. **The production line is PNW** (`~/gh/comma/pnw/pnw-pilot`, fork of xnor-tech which adds Tesla
   Raven support to commaai). The xnor-era `*2xnor` branches and the root patch-script toolchain are
   the PREVIOUS era — still documented in the older references, but day-to-day work happens on the
   `*2pnw` branches in pnw-pilot.

2. **The car is a git checkout with AUTO-UPDATE, not a file overlay.** `/data/openpilot` tracks
   `origin/3devpnw` via the stock updater (validated end-to-end 2026-07-09 incl. git-LFS models and
   submodules). Channel map: **`3devpnw`** = the car; **`4devpnw`** = frozen known-good fallback;
   **`3testpnw` = the FRIENDS' install channel — never experiment on it**; `3pnw`/`pnwprod` =
   release. **All deploy mechanics live in the `pnw-pilot-deploy` skill — load it for any deploy.**

3. **The device runs the LEBOWSKI driving model** (commaai-master snapshot port, `docs/pnw/LEBOWSKI2PNW.md`)
   — combined `driving_supercombo.onnx`, models as git-LFS pointers, tinygrad-compiled at build time,
   ~27 ms inference on the 3X. CES/VTSC tunes are calibrated against it from live drives.

4. **Work = feature branches + in-tree docs.** Every effort has a branch and a doc committed on it
   (`docs/pnw/DMROAD2PNW.md`, `docs/pnw/LEBOWSKI2PNW.md`, `docs/pnw/UPSTREAM2PNW.md`, …), mirrored under `~/gh/comma/docs/`
   with `docs/INDEX.md` as the complete catalog. **Read the effort's doc before touching it.**

5. **Port features INTO the base — never port Tesla Raven support out.** The most expensive lesson
   on this workbench: moving Raven/`tesla_legacy` into sunnypilot/bluepilot is blocked by the panda
   flash (two finished-but-undriveable attempts). The pnw line IS the Raven base; pull features
   toward it, panda-safe. See `references/forks-and-porting.md`.

6. **Safety is non-negotiable.** Safety-critical code = panda C + `opendbc/safety/`. Never weaken a
   safety check; new toggles default OFF; priority **safety > stability > quality > features**. If a
   change would touch panda safety, stop and flag it. (Current era: Tesla opendbc/panda
   modifications — done in the companion repos `~/gh/comma/pnw/pnw-opendbc` + `pnw-panda` on their
   **`master-pnw`** branches, consumed by pnw-pilot as real SHA-pinned submodules on all channel
   branches; `4devpnw` is the frozen fallback for exactly that. Pin-bump workflow lives in the
   pnw-pilot-deploy skill.)

**Before any non-trivial action, skim `references/pitfalls.md`** — the consolidated traps that have
actually cost time or nearly bricked hardware (kept additive; newest era at the bottom).

## Decision guide — where to go next

| If the task is about… | Read |
|------------------------|------|
| **Anything current-era**: the PNW distribution, channel map, lebowski, CES/VTSC/DM feature set, device locator, telemetry-driven tuning workflow | `references/pnw-era.md` |
| **Deploying to the car** (auto-update promote, manual deploy, verification, recovery) | **the `pnw-pilot-deploy` SKILL** (not this skill's deploy-toolchain.md, which is the legacy overlay era) |
| openpilot internals — processes, msgq, Cap'n Proto, car support layer, build/test/lint | `references/architecture.md` |
| Porting between forks; worktrees/branches/remotes; xnor-base strategy; upstream cherry-pick surveys | `references/forks-and-porting.md` + `~/gh/comma/docs/UPSTREAM2PNW.md` (the method) |
| panda firmware, flashing, recovery, CAN framing, safety modes, fingerprinting | `references/panda-and-safety.md` |
| The Lightning or the Raven specifically (CAN topology, `tesla_legacy`, known fixes) | `references/cars.md` |
| Legacy overlay-era deploys (`patch-*.py`, `/data/dirk`, sentinels) — historical reference only | `references/deploy-toolchain.md` |
| **Writing/refactoring ANY pnw feature code** — capability-view rule (never fingerprints), cross-car policy, fork patterns | `references/conventions.md` (READ BEFORE CODING) |
| The trap list | `references/pitfalls.md` |

## Operating principles (the habits that keep this safe)

- **Gemini-review every change that ships** (gemini skill, `gemini-pro-latest`). Two disciplines
  learned the hard way: (a) escape `@` in any diff you pipe to the CLI (`sed 's/@/[at]/g'`) — it
  treats `@tokens` as file attachments and both 400s and hallucinates; (b) **adjudicate its
  findings against the actual tree** — track record is mixed (real catches: a missing capnp-field
  dependency, log-injection, float-compare fragility; hallucinations: pycapnp API, msgq API,
  schema "corruption"). Verify each claim, accept or refute with evidence, record the verdict in
  the commit message.

- **Telemetry-driven tuning loop.** The car logs a per-second CES event stream
  (`/data/pnw/ces_events.jsonl`: mode/reason, vEgo/vSet/vLead/dRel, curve%, vtscCap/State, gas) and
  qlogs carry `onroadEvents`. The proven workflow: driver reports a moment → extract the window →
  name the exact mechanism → fix with a scenario-replay test built from the real telemetry → deploy
  at the next stop. Every drive analysis goes in `~/gh/comma/drives/<date>/<name>/DRIVE_REPORT.md`
  (CLAUDE.md Rule 5) with the raw telemetry saved alongside.

- **Never commit/push to a default branch** (`main`/`master`/`bp-dev` — hook-enforced) and **never
  experiment on `3testpnw`** (friends' channel).

- **Verify on the device, not just locally.** Host py_compile+ruff+scenario tests, then on-device
  venv import/param self-test (`/usr/local/venv`, `PYTHONPATH=/data/openpilot:...opendbc_repo`),
  then live verification. Offline tests cannot catch onroad-only crashers.

- **Find the device via the CloudWatch locator, don't sweep networks** — see pnw-era.md.

- **Secrets never enter the repo** — the Waze/RapidAPI police key lives only in
  `/data/pnw/location/police_proxy.json` on the device (mode 600, survives resets, dies on factory
  reset).

## Quick command reference

```bash
# What's on the car + where is it
head -30 ~/gh/comma/docs/DEVICE-STATE.md
aws --profile dipeit logs filter-log-events --region us-west-2 \
  --log-group-name /aws/lambda/comma-uploader-api --filter-pattern CLIENT_IP \
  --start-time $(( ($(date +%s)-3600)*1000 )) --query 'events[-1].message' --output text

# Ship to the car (see pnw-pilot-deploy skill for the full procedure)
git -C ~/gh/comma/pnw/pnw-pilot push origin 3devpnw   # then SIGHUP updated / wait for cycle

# Build / test / lint inside a worktree (host)
cd ~/gh/comma/pnw/pnw-pilot
scons -u -j$(nproc)            # (host lacks capnp for full pytest; scenario-test pure modules instead)
ruff check . && python3 -m py_compile <changed files>
```
