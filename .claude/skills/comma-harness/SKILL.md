---
name: comma-harness
description: >-
  The Claude Code harness for the ~/gh/comma openpilot workbench — its deterministic
  anchors (enforced checks), how to run them, what to do when one fires, and how to add
  a new one. Use this skill when working on the AGENT HARNESS rather than the car: adding
  or debugging hooks, writing or fixing a checker script, editing anything under
  scripts/ or .claude/, responding to a "Rule 2" or anchor failure, splitting CLAUDE.md,
  or planning agent autonomy (loops, subagents, verifiers). Also use it when a deploy or
  rollback script is being written or reviewed, since those are what the anchors guard.
  NOT for driving, deploying to the car, or car-code changes — use the openpilot and
  pnw-pilot-deploy skills for those.
---

# The comma workbench harness

This workbench's operating rules live in `CLAUDE.md` (cut from 869 to ~200 lines on 2026-09-22; the
long-form text is in `docs/WORKBENCH-REFERENCE.md` §7). Rules that are only prose do
not hold here — measured: the most emphasised rule in that file (Rule 2) was violated **228 times
across 26 of 31 scripts**, while the one rule with a hook had **zero** violations.

**So the design principle is: an operating rule that matters gets an anchor, not a paragraph.**

Full analysis, verdict and roadmap: **`docs/AGENT-AUTONOMY.md`**. Read it before proposing
loops, graphs, or subagents — the verdict is *anchors before orchestration*, with evidence.

---

## The anchors (enforced, not advisory)

| Anchor | Enforces | Wiring | Fires when |
|---|---|---|---|
| branch guard | never commit/push to `main`/`master`/`bp-dev` | `PreToolUse` on `Bash` | a `git commit\|push` targets a default branch |
| `scripts/check-rule2.sh` | Rule 2 — no silent-failure patterns in the deploy toolchain | `PostToolUse` on `Write\|Edit` → `.claude/hooks/script-anchors.sh` | a `scripts/*.sh` file is written or edited |
| `scripts/check-restart-guard.sh` | `touch /tmp/booted` before any `comma` restart (tap-reset trap) | same hook, `.claude/hooks/script-anchors.sh` | a `scripts/*.sh` file is written or edited |
| session state | prints checkouts, device reachability, anchor status at session start | `SessionStart` → `.claude/hooks/session-state.sh` | every session start |

```bash
./scripts/check-rule2.sh                  # whole deploy toolchain
./scripts/check-rule2.sh scripts/foo.sh   # one file (what the hook calls)
./scripts/check-restart-guard.sh scripts/foo.sh
./scripts/audit-docs.sh                   # doc-index drift (manual; not yet an anchor)
```

## When the Rule 2 anchor fires

It flags **load-bearing** silent failure only. The two real shapes:

```bash
[ -f "$F" ] && sed -i '...' "$F" || true    # returns 0 when $F is ABSENT -> silent no-op
cmd_that_matters ... || true                # failure swallowed, script continues
```

**Do one of:**
1. **Fix it** — make failure loud and the success path distinguishable from the failure path.
   For a *guarded mutation*, report what changed (`removed N line(s)`) and fail on a missing
   required file.
2. **Justify it** — `# rule2-ok: <reason>` on the same line, or the preceding line for
   heredocs/multi-line substitutions. Legitimate: `rm -f`, `pkill`, `find -delete`,
   `ssh 'sudo reboot'` (non-zero rc expected), probes explicitly marked informational.

**Never** silence it by weakening the checker. It is an anchor precisely because it does not move.

## Writing a deploy or rollback script here

Non-negotiables, each learned from a real incident (see `docs/AGENT-AUTONOMY.md` §10):

- **Verify a backup exists BEFORE destroying the live thing.** `rm -rf <live>` followed by
  `cp <backup> … || true` deleted a package and continued when the backup was missing.
- **A restart is not done until the service is up.** `systemctl restart … || { fallback; }` with
  no `is-active` check reported success while manager never came back.
- **`touch /tmp/booted` before `systemctl restart comma`** — without it `comma.sh`'s factory-reset
  detection can show the tap-reset prompt and manager never starts. (9 of 9 scripts violated this;
  now enforced by `scripts/check-restart-guard.sh` — justify exceptions with `# restart-ok: <reason>`.)
- **A message that prints unconditionally is not a report.** `[ -d x ] && cp … ; echo "installed"`
  prints "installed" either way.
- **Distinguish "not yet" from "the link is down."** `ssh … test -f … 2>/dev/null` makes rc 1 and
  rc 255 identical.

## Reading `GearPark` — and params like it

**`GearPark` surfaces `carState.gearShifter == park`** (`CLAUDE.md` Rule 3). `1` = in Park,
`0` = in gear. It exists so **background daemons can gate on parked without a 100 Hz `carState`
subscription** — the 2026-07-13 `commIssue` cascade came from exactly such a sub. **Control-path
code should read the live `carState` instead.**

Two properties worth knowing before quoting it:

- `card.py:177` seeds it `False` at startup — **deliberately fail-safe**, so a stale `True` from a
  hard power cut cannot hold the uploader gate open during a drive that never touches Park.
- `card.py:309` corrects it on the first **valid** CAN tick, and only a valid read may flip it
  (an invalid/no-CAN tick holds the last value, so it does not flap).

So the value **is** meaningful while `card` runs. It is uninformative only when **`card` is not
running** (it is `only_onroad`, `process_config.py:103`) or before the first valid CAN tick —
then `0` is the seeded/cleared default (`params_keys.h:126`, `CLEAR_ON_MANAGER_START`).

**Check `card` is up before reading it.** `.claude/hooks/session-state.sh` does, and reports the
two cases differently.

> **Post-mortem, 2026-09-07 — over-correction is also an error.** An assistant read `GearPark=0`
> and said "the truck may not be in Park" — **correct**. Challenged, it over-corrected to "`0`
> carries no information," which is wrong whenever `card` is up, and shipped a hook that labelled a
> genuine reading `AMBIGUOUS`. Applying default-vs-observation reasoning without checking whether
> the writer is actually running turned a right answer into a wrong one. **Before down-weighting a
> reading, check whether its writer is live.**

## Adding a new anchor

1. **Write the checker as a script** in `scripts/`, exit non-zero on violation.
2. **Make it print what it scanned, not just what it found** — an empty result must be
   distinguishable from a broken run. (`audit-docs.sh` documents two earlier cuts that were wrong
   in opposite directions and both looked plausible.)
3. **Give it a suppression convention** with a mandatory reason. A checker with a false-positive
   rate and no escape hatch gets muted, and then you have neither the check nor the rule.
4. **Wire it to a hook** — until then it is a script someone must remember to run, which is the
   failure mode you are fixing.
5. **Test the checker against a known-bad and a known-good input** before trusting a clean run.

## What NOT to build

From `docs/AGENT-AUTONOMY.md` §4 and §9, with reasons:

- **No self-improving loop.** No trustworthy judge; reward hacking is the documented outcome.
- **No graph.** The deploy sequence is a genuine chain — every step consumes the previous step's
  output, so the fake-edge test says there is nothing to parallelise.
- **No deploy agent, no panda-flashing agent.** A reboot happens only after a live
  `selfdriveState.enabled == False` read in its own tool call (the only reboot condition — see
  `CLAUDE.md` and the `pnw-pilot-deploy` skill); panda safety flashes stay supervised.
- **No more prose rules in `CLAUDE.md`.** The marginal rule is worth ~0. Convert, don't add.
- **A verifier with no anchor to reach is not a verifier** — it "agrees with itself in a different
  font." Blind maker/checker (P2) depends on the anchors, it does not replace them.

## Roadmap position

P1 anchors ✅ done · **P2 blind maker/checker (subagent)** · P3 capped doc-audit loop ·
P4 `SessionStart` state hook ✅ done (`session-state.sh`) · P5 split `CLAUDE.md` — partly done
2026-09-22 (~200 lines, detail moved to `docs/WORKBENCH-REFERENCE.md` and the skills).

The gate between stages: re-measure using **anchor-reported counts**, never self-reported incident
counts — an agent optimising "fewer incidents" can simply report fewer.
