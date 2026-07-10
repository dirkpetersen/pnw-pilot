# Forks, worktrees & cross-fork porting

How the four forks relate, how the worktrees/branches/remotes are wired, and — most importantly —
**which direction to port features**, a decision that has repeatedly made or broken efforts here.

## Table of contents

- [The porting doctrine (read this first)](#the-porting-doctrine-read-this-first)
- [The forks](#the-forks)
- [Worktree & repo layout](#worktree--repo-layout)
- [Remotes](#remotes)
- [Branch & doc conventions](#branch--doc-conventions)
- [How to port a feature (the method that works)](#how-to-port-a-feature-the-method-that-works)
- [Choosing the right base commit](#choosing-the-right-base-commit)

## The porting doctrine (read this first)

**This is the single most expensive lesson from this workbench. Internalize it before proposing any
cross-fork work.**

> **Port other features INTO the xnor base. Do NOT port xnor's Tesla support OUT into
> sunnypilot/bluepilot.**

Why this direction and not the other:

- **Tesla Raven support is extremely hard to graft into a modern fork.** The Raven uses Tesla's old
  multi-bus CAN topology and needs a `tesla_legacy` panda safety model. Bringing it into
  sunnypilot/bluepilot requires ~15 tightly-coupled, individually-crashy pieces (legacy opendbc
  files, a new safety model registered through `car.capnp` + `declarations.h` + `safety.h`, new
  `CarHarness` enum entries, torque overrides, a DBC superset, an ignition header, a pinned panda
  submodule). Each missing piece is a documented crash. Two serious attempts —
  `bluepilot/xnor2bp` and `sunnypilot/xnor2sunny` — got the *code* functionally complete but
  **both stalled on the panda flash**, which is the part that actually makes the car drive.

- **The panda is the wall.** xnor "just works" because it ships a **matched set**: its panda
  firmware and its openpilot library are built from the **same opendbc snapshot**, so their
  `CAN_PACKET_VERSION_HASH` agree (observed: `0x75ABF276`). Port the Tesla code into a different fork
  and you must rebuild + reflash the panda against *that* fork's opendbc — which reintroduces the
  hash-mismatch and dual-panda enumeration failures. (Details in `references/panda-and-safety.md`.)

- **So keep xnor as the base and pull everything else toward it.** The reverse-direction efforts
  (`light2xnor`, `dmon2xnor`, `mapd2xnor`, `auto2xnor`) succeed because they add **panda-safe**
  features (UI toggles, driver monitoring, OSM speed limits, Ford fingerprint/control) onto the
  already-working Raven base **without touching panda firmware**. The Raven keeps driving; the new
  feature layers on top.

- **xnor is currently very up to date**, which removes the usual reason to prefer sunnypilot/bluepilot
  as a base (they're newer than commaai). Right now xnor gives you both modern openpilot *and*
  working Tesla — so it's the natural home. (Re-check this is still true before relying on it; forks
  drift.)

**Practical rule:** if a proposed port would require **reflashing the panda on a non-xnor base**, or
moving `tesla_legacy`/Raven support into sunnypilot or bluepilot, stop and recommend the
reverse direction (`<feature>2xnor`) instead. Only consider the hard direction if the user
explicitly wants to re-attempt the panda-flash investigation and accepts that risk.

## The forks

```
commaai/openpilot   ──fork──>  sunnypilot/openpilot  ──fork──>  BluePilotDev/bluepilot
   (upstream)                  (MADS, NNLC, speed-limit,        (Ford/Lightning support,
                               newer features)                   3-layer UI inheritance)

xnor-tech/openpilot — a commaai fork carrying working Tesla Model S legacy (Raven) support.
   The `xnor-dirk` branch (the "prebuilt" Tesla branch) is the one that drives; xnor/master is
   the big 17k-commit fork WITHOUT the Tesla legacy files — don't confuse them.
```

| Fork | Strengths | Role here |
|------|-----------|-----------|
| commaai | canonical upstream | reference/baseline (`openpilot/` on `master`) |
| sunnypilot | MADS, nudgeless lane change, speed-limit assist, model selection | feature donor; `sunny/sunnypilot/` worktree |
| bluepilot | Ford/F-150 Lightning control + fingerprint, UI | Ford donor; `sunny/bluepilot/` worktree |
| xnor | **working Tesla Model S HW3 (Raven) legacy support**, currently up to date | **the deployment base** (`xnor/openpilot/`) |
| FrogPilot | commaai v0.9.7 fork (FrogAi), very current, DEC-style features, inline panda+opendbc | feature reference; `FrogPilot/` standalone |

## Worktree & repo layout

`openpilot/` holds the real `.git`; `sunny/bluepilot/`, `xnor/openpilot/`, `sunny/sunnypilot/` are **worktrees**
of it — they share the object store, so all branches are visible from any of them and cherry-picks
across forks are local operations. `sunny/opendbc/` and `sunny/panda/` are **standalone** repos. The `*-xnor`
clones are **read-only known-good references**.

| Path | Kind | Typical branch | Role |
|------|------|----------------|------|
| `openpilot/` | worktree (main `.git`) | `master` | commaai upstream |
| `sunny/bluepilot/` | worktree | `xnor2bp` / `bluepilot-dirk` | BluePilot Ford+Tesla install |
| `xnor/openpilot/` | worktree | `auto2xnor` / `xnor-dirk` | **xnor base — active feature ports** |
| `sunny/sunnypilot/` | worktree | `dmon2sunny` / `sunnypilot-dirk` | sunnypilot fork |
| `sunny/opendbc/` | standalone | `xnor2sunny` / `dirk` | car DBCs + safety; Raven fixes committed here |
| `sunny/panda/` | standalone | `dirk` | panda firmware (Tesla `GTW_status` 0x348 ignition) |
| `xnor/opendbc/` | standalone clone | `origin/master-xnor` | **reference** — proven Raven opendbc |
| `xnor/panda/` | standalone clone | xnor branches | **reference** — proven Raven panda |
| `comma-eb1f2f7/` | not a repo | — | device backup / restore source |

Always confirm the live branch (`git -C <dir> branch --show-current`) — these move.

## Remotes

openpilot worktrees share one remote set: `origin` (dirkpetersen/openpilot), `commaai`, `sunnypilot`
(dirkpetersen/sunnypilot), `sunnypilot-upstream` (sunnypilot/sunnypilot), `bluepilot`
(dirkpetersen/bluepilot), `bluepilotdev` (BluePilotDev/bluepilot), `xnor` (xnor-tech/openpilot).

`opendbc` and `panda` each have `origin` (dirkpetersen/…), `commaai`, `sunnypilot`, `xnor`. Note:
`sunnypilot` is added as a **remote**, not a separate GitHub fork — GitHub blocks a second fork of
`sunnypilot/opendbc` under the same account, so a remote sidesteps the name conflict.

```bash
git cherry-pick <sha>               # works across forks (worktrees share objects)
git -C panda log xnor/master-xnor   # browse another fork's history
git -C xnor/openpilot show auto2xnor:AUTO2XNOR.md   # read a branch's in-tree doc
```

## Branch & doc conventions

- Each feature-porting effort gets a branch named `<feature>2<target>` (`light2xnor`, `dmon2xnor`,
  `mapd2xnor`, `auto2xnor`, `xnor2bp`, `xnor2sunny`) or a fork base branch `<fork>-dirk`.
- **Each branch carries its own in-tree doc**, committed on the branch: xnor feature branches use
  `<FEATURE>.md` at the repo root (e.g. `AUTO2XNOR.md`); the sunnypilot/bluepilot porting branches
  use `DEPLOY.md`. The `integration-device` branch aggregates several. **Read the branch's own doc
  first** — it's the most current account of how to deploy and what pitfalls apply, and can be ahead
  of any copy mirrored at the `~/gh/comma/` root.
- Never commit/push to `main`/`master`/`bp-dev` — a `PreToolUse` hook blocks it; use the feature branch.

## The PNW trio & submodule structure (the production distribution)

`~/gh/comma/pnw/` holds the deployable distribution: three sibling forks of the xnor-tech network —
`pnw-pilot` (openpilot), `pnw-opendbc`, `pnw-panda` (all `dirkpetersen/pnw-*`, created 2026‑06‑20).

- **pnw-pilot references opendbc/panda as SUBMODULES with RELATIVE urls** (`.gitmodules`:
  `opendbc_repo → ../pnw-opendbc.git`, `panda → ../pnw-panda.git`). The relative url resolves against
  the superproject's remote, so a clone of `dirkpetersen/pnw-pilot` pulls
  `dirkpetersen/pnw-opendbc`/`-panda`. (`testing` is the exception — it vendors them inline.)
- **The submodule and the superproject are pushed SEPARATELY.** This is the #1 footgun here: you
  commit work in `opendbc_repo/`, bump+push the superproject pointer, and forget to push the submodule
  commit → the pointer dangles on the remote and fresh clones can't init the submodule (see
  `references/pitfalls.md`). **Always:** `git -C opendbc_repo push origin <branch>` → then in the
  superproject `git add opendbc_repo && git commit && git push`. Verify with
  `git -C opendbc_repo branch -r --contains <pinned-sha>` (empty = unpushed).
- **CANONICAL BRANCHES = `master` + `master-xnor` only (since 2026-06-22).** `pnw-opendbc` and
  `pnw-panda` were consolidated down to just these two; the scattered feature branches
  (`tesla2pnw`, `lightning2pnw`, `tesla-raven`, `3pnwtest`, `ces2pnw`, `light2xnor`, `xnor2sunny`,
  `dirk`, `master-c3`, panda `dirk`/`master-xnor-c3`) were **deleted** (SHAs preserved in
  `~/gh/comma/pnw/_ford-readd/RECOVERY.md`). `master-xnor` IS the proven xnor-tech Raven base and is
  self-sufficient (`TESLA_MODEL_S_HW3` + both `tesla.h` AND `tesla_legacy.h` registered). `pnw-pilot`'s
  `3pnwtest` (and `main`) pin **`master-xnor`** on both submodules — opendbc `534ac1f2`, panda `56920ec6`.
- **THE MATCHED-SET LAW (the expensive 2026-06-22 lesson — Raven "no panda").** The xnor panda firmware
  and xnor opendbc are built from the SAME snapshot; their `CAN_PACKET_VERSION_HASH` agree AND the
  opendbc carries `fw query: restore optional num_pandas for aux panda support` (+ tesla radar fixes)
  that the Raven's **aux/dual panda** needs. A "3pnwtest" integration built by merging *commaai* opendbc
  + `tesla-raven` and pairing it with a *commaai* panda (`move can ignition to opendbc #2396`) **broke
  the set** — it dropped the aux-panda fw-query commit, so the second panda sat in DFU and pandad
  looped on `flash_and_connect` → the car showed **"no panda"**. Fix = pin BOTH submodules to the xnor
  `master-xnor` matched-set; never pair a commaai panda with a Raven-carrying opendbc that lacks the
  xnor aux-panda commits. The Ford 2025 Lightning fingerprint + per-car op-long do NOT live on
  master-xnor — re-apply them ON TOP (patches in `_ford-readd/`), never by swapping back to a
  commaai-derived opendbc/panda.

## How to port a feature (the method that works)

This is the audit method proven on the successful ports:

1. **Pick the base** = `xnor-dirk` (or its current descendant) so the working Raven comes along. Branch `<feature>2xnor` off it.
2. **Isolate the donor's real changes.** Diff the donor fork's feature files against *pristine commaai* (e.g. `commaai/__nightly`) — this strips away the donor's unrelated fork noise and leaves only the feature's actual diff.
3. **Confirm the base already has the prerequisites.** Often the xnor base already carries the upstream platform (e.g. it already has `FORD_F_150_LIGHTNING_MK1`), so the port is smaller than expected.
4. **Apply the minimal, panda-safe slice.** Prefer additive changes to `carstate.py`/`carcontroller.py`/`values.py`/UI. **Avoid anything that forces a panda reflash.**
5. **Default new toggles OFF** and live-refresh where possible (no restart).
6. **Write the deploy scripts** (`patch-<feature>.py` + `update-<feature>.sh`, surgical/idempotent — see `references/deploy-toolchain.md`) and the branch's in-tree doc.
7. **Verify on the device**, not just locally.

## Choosing the right base commit

There can be two very different branches with the same fork name. For xnor specifically:

| Candidate | Has working Raven? | Use as base? |
|-----------|--------------------|--------------|
| `xnor-dirk` (the "prebuilt" Tesla branch, e.g. `c0d78143`) | yes (`teslacan_legacy.py`, `tesla_legacy.h` present) | **yes** |
| `xnor/master` (full ~17k-commit commaai fork) | **no** (no `tesla_legacy*` files) | no — would lose Raven |

Before branching, verify the base actually contains the support you depend on
(`git -C xnor/openpilot show <branch>:opendbc_repo/opendbc/car/tesla/teslacan_legacy.py | head`).
