---
updated: 2026-09-21
status: BUILT on branch capnpfork2pnw -- NOT pushed, NOT deployed, awaiting Fable review (CLAUDE.md Rule 8)
---

# CAPNP-FORK-ORDINALS -- the fork's ordinals come out of upstream's `log.capnp` (capnpfork2pnw)

> **One sentence.** The fork had put six events into upstream's `OnroadEvent.EventName` (@99-@104) and
> a `UInt8` into a `PandaState` slot upstream reserved as `Bool`. Upstream has since spent every one of
> those event ordinals itself. capnp ordinals are **wire format**, so this was a silent-corruption bug
> waiting for the next rebase: the events move to `custom.capnp`, `PandaState` keeps only the two Bool
> slots upstream reserved for forks, a test now fails if anything like it happens again, and logs
> already recorded are migrated on read, from the writer's own schema, refusing to guess.

---

## 1. The problem, measured (2026-09-21)

Measured with a node-id-keyed audit of the compiled schemas, not by reading diffs. The fork's upstream
base is **commaai master `a68ea44af341` (2026-03-14)**, not the `v0.11.1` release commit. That matters:

### `OnroadEvent.EventName`: **all six** fork enumerants collide with current upstream

| ordinal | ours (until capnpfork2pnw) | 0.11.1 `release-tizi` | 0.11.2 `release-chestnut` | `master` `521db4c825` (09-20) |
|---|---|---|---|---|
| @99  | greenLight | **lateralManeuver** | lateralManeuver | lateralManeuver |
| @100 | leadDeparting | -- | **bigModelLoading** | bigModelLoading |
| @101 | madsLateralOnly | -- | **bigModelReadyDEPRECATED** | bigModelReadyDEPRECATED |
| @102 | **madsControlsMismatchLateral** (MADS safety, IMMEDIATE_DISABLE) | -- | **bigModelFailed** | bigModelFailed |
| @103 | cruiseOffRequested | -- | **carNotReady** | carNotReady |
| @104 | madsResumeSetTooHigh | -- | -- | **userBookmarkNotPaired** |

Two things CHESTNUT-PORT.md §1 did not have: **@99 already collides with 0.11.1 itself** (the fork's base
predates it), and **@104 collided since 0.11.2** -- upstream allocated it on master within five weeks.

### `PandaState`: one type collision, one unreserved slot

| ordinal | ours (until capnpfork2pnw) | upstream (0.11.1, 0.11.2, master) | wire |
|---|---|---|---|
| @38 | controlsAllowedLateral :Bool | controlsAllowedRESERVED1 :Bool | same type -- fine |
| @39 | madsDisengageReason **:UInt8** | controlsAllowedRESERVED2 **:Bool** | **TYPE collision** |
| @40 | healthPacketMismatch :Bool | -- (not reserved) | fork field in upstream's struct |

### Everything else the fork added to `log.capnp` is fine
The `Custom.*` renames of `CustomReservedN` Event slots (`vtscState @116`, `madsState @136`,
`mapdExtendedOut/In/Out @143-@145`) are the sanctioned pattern. Three more additions are **upstream's own
later fields, backported** -- bit-identical by node id (ordinal, name, type, data offset) to upstream:
`LivePose.timestamp @8`, `DriverStateV2.DriverData.sleepProb @14`, `LiveDelayData.version @7` (0.11.2
renamed the structs `DeviceMotion` / `LateralDelay`; the members are the same).

---

## 2. Options, and the choice

| option | what | verdict |
|---|---|---|
| **(a)** fork events in `custom.capnp`, own enum, own service | CLAUDE.md's stated convention; immune to every future upstream addition | **CHOSEN** |
| (b) take upstream's 0.11.2 names at @99-@103, shift ours to @104+ | makes THIS port trivial | **Rejected, with evidence:** it had already failed before it could be written -- master took @104 (`userBookmarkNotPaired`) five weeks after 0.11.2. Every upstream release would mean another renumber and another migration, and it would carry five upstream events (`bigModel*`, `carNotReady`) we do not implement. |
| (c) sunnypilot's wiring: a second `EventsSP` object | sunnypilot keeps `OnroadEventSP.EventName` in `custom.capnp` and publishes `onroadEventsSP` -- (a)'s schema, which we copy | Schema: **same as (a)**. Python wiring: **not copied** -- see below |

**Why not sunnypilot's second `Events` object.** In sunnypilot the SP events do **not** reach openpilot's
own state machine (`state_machine.update(self.events)` sees only upstream events; MADS runs its own).
Ours **do**: `cruiseOffRequested` is a `NO_ENTRY`, `madsControlsMismatchLateral` an `IMMEDIATE_DISABLE`
that `mads_pnw.has_blocking_event()` must see. Splitting them into a second object would mean merging the
two back inside `StateMachine.update`, `contains()`, `create_alerts()` and `has_blocking_event()` -- more
change to the MADS safety path, not less. So the fork's events stay in the ONE `Events` object, under a
disjoint key space (below), and only the wire changes.

---

## 3. The design

### Schema
- `custom.capnp`
  - `OnroadEventsPnw` (reuses the **CustomReserved11** id, Event **@137**) = `events :List(OnroadEventPnw)`.
  - `OnroadEventPnw` (new id): `name :EventName` + the same ten event-type Bools, same ordinals, as
    `log.OnroadEvent`. `EventName` = `greenLight @0, leadDeparting @1, madsLateralOnly @2,
    madsControlsMismatchLateral @3, cruiseOffRequested @4, madsResumeSetTooHigh @5` -- **append only**, and
    deliberately in their old @99..@104 order (see ordering, below).
  - `PandaStatesPnw` (reuses **CustomReserved12**, Event **@138**) = `pandas :List(PandaStatePnw)`, one per
    panda in `pandaStates` order; `PandaStatePnw.madsDisengageReason :UInt8`.
- `log.capnp`
  - `EventName` is **exactly** the upstream base's 99 enumerants (the block is byte-identical to upstream).
  - `PandaState`: `controlsAllowedLateral @38 :Bool` (unchanged), **`healthPacketMismatch @39 :Bool`**
    (was @40), `madsDisengageReason` gone (-> `pandaStatesPnw`).
- `services.py`: `onroadEventsPnw (True, 1., 1)`, `pandaStatesPnw (True, 10., 1)` -- the rate and qlog
  decimation of the message each accompanies.

**Why `healthPacketMismatch` takes @39 and `madsDisengageReason` leaves, not the other way round.**
1. Upstream reserved exactly two slots, both `Bool`. Only our two Bools can use them.
2. `healthPacketMismatch` gates selfdrived's lateral mismatch detector, which must read it from the **same
   message** as `controlsAllowed`/`controlsAllowedLateral` for that panda. Moving it to a second message
   would put a cross-message alignment problem into a safety path.
3. **Measured, not assumed:** capnp lays `Bool @39` on data bit **482 -- the exact bit the old `Bool @40`
   used** (`@38` stays on 481; the old `UInt8 @39` was byte 74). So every log recorded before the move
   reads `healthPacketMismatch` unchanged under the new schema, and new logs read correctly under the old
   one: no migration, in either direction. Proven sample-for-sample on real logs (§6).
4. `madsDisengageReason` is diagnostic only: nothing in the tree, the drive scripts or the workbench tools
   reads it (grep, 2026-09-21). It keeps being recorded, on `pandaStatesPnw`.
Sunnypilot uses @39 for `controlsAllowedLongitudinal :Bool`; we have no such flag. The rule that matters
is upstream's reserved **type**, which is what the guard enforces.

### Python: one `Events` object, a disjoint key space
- `events.py`: `PNW_EVENT_BASE = 1 << 16`; `EventNamePnw.<name> = PNW_EVENT_BASE + <custom ordinal>`.
  capnp enumerants are **UInt16 on the wire**, so no upstream `EventName` can ever reach 65536.
- **Ordering is preserved exactly.** `Events` keeps its list sorted; `AlertManager` breaks
  (priority, start_frame) ties by dict insertion order, i.e. by that sort. Fork keys sort after every
  upstream key and among themselves in their old @99..@104 order -- the same relative position they had.
- `EVENT_NAME` merges both; `alertType` strings (`"greenLight/permanent"`, the green banner keyed on it in
  `alert_renderer.py`) are unchanged. An upstream name equal to a fork name raises `ImportError` at import.
- `to_msg()` = upstream events only (`onroadEvents`); `to_msg_pnw()` = fork events only
  (`onroadEventsPnw`). `add_from_msg()` **raises `TypeError`** on an `OnroadEventPnw` list -- its raw 0..5 is
  upstream's `canError..seatbeltNotLatched`, and accepting it would silently be the wrong event.
- pycapnp itself refuses to write an unknown enumerant or an out-of-range int into `OnroadEvent.name`
  (`Member was null` / `Value out-of-range`), so a fork key cannot be silently truncated into
  `onroadEvents` either.
- `selfdrived` publishes `onroadEventsPnw` **first**, then `onroadEvents`, in the same frame under the same
  condition -- so a consumer that reacts to `onroadEvents` (card's accdrop logger) already holds that
  frame's fork events. Same reasoning as `madsState` before `selfdriveState`.

### Consumers changed
| file | change |
|---|---|
| `selfdrive/pandad/pandad.cc` | publishes `pandaStatesPnw` right after `pandaStates` (same pandas, order, validity); `setHealthPacketMismatch` unchanged (field name kept) |
| `selfdrive/selfdrived/{events,selfdrived}.py` | as above |
| `selfdrive/car/card.py`, `accdrop_pnw.py` | card subscribes `onroadEventsPnw`; accdrop's `"ev"` channel merges both lists (cruiseOffRequested, madsLateralOnly would otherwise silently vanish from accDrop records). card's gates are scoped to `carControl`, so the subscription adds no check. |
| `selfdrive/debug/count_events.py` | counts fork events separately |
| `selfdrive/test/process_replay/process_replay.py` | selfdrived outputs + card inputs include `onroadEventsPnw` |
| `selfdrive/test/process_replay/migration.py` | the migration (§5), on by default in `migrate_all` |

Unchanged and verified: `controlsd`/`joystickd` read override types from `onroadEvents` only -- a test pins
that no fork event carries an override type. card's `selfdriveInitializing`, the mici HUD's lane-change
events, `ces_pnw`'s `otherEvents` (built from `EVENT_NAME`) all use upstream names or the merged table.
No C++ consumer of `OnroadEvent`, and nothing in `tools/curvedb` or `comma-connect`.

---

## 4. The guard -- `selfdrive/selfdrived/tests/test_capnp_fork_ordinals_pnw.py`

A convention nobody enforced is how this happened. The guard compares the **whole compiled `log.capnp` +
`legacy.capnp`** -- every struct and enum, keyed by 64-bit node id, every member's ordinal, name, type,
union discriminant and data offset -- against a frozen snapshot of the upstream base
(`capnpfork_fixtures/upstream_base_log_schema.json.zst`, 221 nodes, `make_fixtures.py --snapshot a68ea44af341`).
The only departures allowed are listed, each with its reason:
- **additions**: `PandaState @38/@39` (Bool, bits 481/482 -- upstream's reserved slots) and the three
  upstream backports;
- **renames**: the seven `Event` `customReservedN` slots, pinned by type to upstream's own
  `CustomReservedN` struct id at that ordinal.

Also: `EventName` equals the base exactly; every ordinal we define is the same event in 0.11.1, 0.11.2 and
master (renames of the same event only, e.g. `driverDistracted1`); no fork event name exists upstream; the
fork enum keeps the old relative order. **The guard is run on the schema that actually shipped
(`pre_capnpfork2pnw_log_schema.json.zst`) and must report exactly its three collisions** -- a guard that has
never been seen to fail proves nothing. It lives in `selfdrive/selfdrived/tests`, which the Rule 9 gate
already runs, so no `TEST_PATHS` change is needed.

If it fires on a future change: the fork field belongs in `custom.capnp` (rename a free `CustomReservedN`
struct, keep its id). Only if upstream *itself* reserves a slot for forks may the allowlist grow.

---

## 5. Logs already recorded -- `migration.py: migrate_pnwOnroadEvents`

### How an old-schema log is identified -- the hard part
**By its writer's own schema, never by its content.** After a rebase an old @99 is a perfectly valid
upstream enumerant, so no content test can tell "greenLight written by us" from "lateralManeuver written
by upstream". What each candidate gives:

| candidate | verdict |
|---|---|
| content sniffing (raw values) | **unusable** for the reason above; also the future `UInt8 @40` a new upstream PandaState field would get lands on the very byte 74 `madsDisengageReason` used |
| `initData.version` | "0.11.1" for old and new builds alike |
| `initData.gitRemote` | **measured unreliable**: 2,056 of 2,069 local logs say `dirkpetersen/openpilot.git` (the old, deleted repo name), 13 say `pnw-pilot.git`; the device also ran bluepilot/sunnypilot builds (`bp-7.0` has upstream's `lateralManeuver @99`) |
| wall time / deploy date | per device, clock can be bad, friends' channel updates on its own schedule |
| a new schema marker in `initData` | would itself need a field in upstream's `InitData` -- the very thing this fixes |
| **`initData.gitCommit` -> that commit's `cereal/log.capnp`** | **CHOSEN.** The ground truth, not a proxy. All 115 distinct writer commits in the corpus (78 local, 106 in S3 since 09-05) resolve in this repo (2026-09-21). |

Decision, per log (`_pnw_fork_ordinals`):
1. The log already has `onroadEventsPnw` -> written after the fix -> nothing to do (the framework's
   `product` rule; `selfdrived` publishes the two in lockstep).
2. No `onroadEvents` entry at raw >= 99 -> nothing to do. The result is identical whoever wrote the log,
   so even an unknowable writer is fine here.
3. Otherwise the writer MUST be established, else **`PnwLogSchemaError`**: no `initData`; not exactly one
   `gitCommit`; an empty one; a **dirty** writer (`initData.dirty`); a commit git cannot read; a raw
   ordinal the writer's own schema does not define; a fork event at an ordinal no audited build used
   (every fork branch among the 195 refs in pnw-pilot carries a prefix of the same @99..@104 table; the rest have no fork enumerants); an upstream name the current
   schema cannot represent (a 0.11.2 log's `carNotReady @103` raises -- it is never turned into
   `cruiseOffRequested`).
4. Writer's name is a fork event -> moved to a synthesized `onroadEventsPnw` (one per `onroadEvents`, same
   `logMonoTime`/`valid`, the **recorded** flags -- not today's `EVENTS`), and removed from `onroadEvents`.

Override for a writer git cannot resolve (a commit never pushed, a foreign clone):
`PNW_LOG_WRITER_SCHEMA=fork-v1|upstream` -- a human's assertion, warned about on every use, and `fork-v1`
still refuses an ordinal the table never had.

`migrate_all()` runs it by default, so `juggle.py`, `tools/clip`, `count_events.py` and process replay get
it. **Ad-hoc scripts under `drives/` that read fork names out of `onroadEvents` (e.g.
`cruise_end_sweep.py`, `qlog_health.py`) must either call `migrate_all()` or read `onroadEventsPnw`** --
on logs recorded after this ships they would otherwise find nothing.

**What an unmigrated old log does today, on this schema:** reading a fork entry raises
(`str()`, `==`, `to_dict()` all give `Member was null`; only `.raw` works) -- loud. The *silent*
mis-decode only appears after a rebase onto a base that defines @99+, which is why the migration must exist
before the port.

### What is NOT migrated
- `PandaState.healthPacketMismatch` / `controlsAllowedLateral`: nothing to migrate (same bits, §3).
- **`madsDisengageReason` in old logs is not migrated.** It sits at byte 74 of the old `PandaState` data
  section; recovering it would need a raw data-section read gated on writer detection for every
  pandaStates-bearing log, for a field nothing reads. It is not lost: decode those logs with the writer's
  schema (`git show <initData.gitCommit>:cereal/log.capnp`, what `make_fixtures.py` does). Values seen in
  the local corpus: 0, 8, 16, 32. **Owner decision if that is not acceptable.**

---

## 6. Evidence

- **Real fixtures** (`capnpfork_fixtures/`, original message bytes, trimmed to initData + onroadEvents
  [+ pandaStates]; ground truth decoded by each segment's own writer schema in a child process):

  | fixture | S3 source (`drives/2fd850c60cc5bfef/…/qlog.zst`) | writer | fork events in it |
  |---|---|---|---|
  | `lightning_101_104` | `0000015f--8f71b7536e--1` | `705c74b931` | madsLateralOnly x35, madsResumeSetTooHigh x1; 600 pandaStates (reason 16 on 137) |
  | `lightning_101_103` | `0000012b--2f02194797--44` | `eedddfa617` | madsLateralOnly x5, cruiseOffRequested x10 |
  | `lightning_101_100` | `000000f0--c714d35b1d--4` | `20d1826088` | madsLateralOnly x23, leadDeparting x1 |
  | `lightning_101_99`  | `0000011f--2eee9417c4--9` | `c31acd4593` | madsLateralOnly x29, greenLight x1 |
  | `tesla_healthmismatch` | `00000106--2335375565--0` | `4b7067409a` | none; 612 pandaStates, panda 1 `healthPacketMismatch` True |

- **`madsControlsMismatchLateral` has never been recorded.** Zero raw-@102 across **all 5,492 qlogs** uploaded
  since it was introduced (2026-09-05; 378,228 onroadEvents messages, 0 read errors) and 2,069 local log
  files -- while @99/@100/@101/@103/@104 were all found (46/38/9,283/255/2), so the scan is not blind to
  the range. An independent verifier confirmed it and found the mechanism: its path needs
  `PandaMadsSafety=1` and a reflashed panda, neither of which has happened. It is therefore tested by
  **injecting @102 into a real segment whose writer defines it**, and checked to come out as
  `madsControlsMismatchLateral` with `immediateDisable/noEntry/permanent`, exactly once, gone from
  `onroadEvents`.
- `healthPacketMismatch` + `controlsAllowedLateral`: new-schema decode == writer-schema decode on every one
  of the 1,212 pandaStates messages (1,824 per-panda samples, both Raven pandas) in the two fixtures.
- **The whole real corpus, migrated**: all **7,561** log files (5,492 S3 qlogs since 09-05 + 2,069 local
  rlogs/qlogs) through `migrate_all`: **0 errors** (every file with a contested ordinal was decidable),
  moved == found for every name (greenLight 61, leadDeparting 46, madsLateralOnly 11,540,
  cruiseOffRequested 367, madsResumeSetTooHigh 2), **0** contested ordinals left in `onroadEvents`, and one
  `onroadEventsPnw` per `onroadEvents` wherever it applied. All 115 writer commits resolve in git.
- **Mutation testing** (35 mutants; each counted only if its anchor matched EXACTLY once and the result
  compiled -- `compile()` for Python, the schema loading in pycapnp for `.capnp`; files restored
  byte-for-byte, sha256-checked): **35 built, 34 killed by the test designated to catch them, 1 (a fork key
  base overlapping upstream) killed earlier by `events.py`'s import-time collision check, 0 survived.**
  Covered: fork enumerant/field re-added to `log.capnp` (EventName, PandaState @39 type, @40, an unrelated
  struct, a new struct, a non-reserved slot rename, another enum), the guard's own checks disabled one by
  one, the migration's table swapped / off by one / dropping only @102 / leaving moved events behind / every
  refusal turned into a guess / flags not copied / not registered / the override made silent, and the
  events wiring (fork keys into `onroadEvents`, `add_from_msg` accepting fork events, publish order swapped,
  accdrop dropping fork names, the MADS event losing IMMEDIATE_DISABLE). Two gaps the first run exposed --
  the guard had no test of a same-ordinal TYPE change and none of a NEW node -- are closed by
  `test_the_guard_flags_each_kind_of_wire_change`.
- **pandad**: `pandad.cc`/`panda.cc` compiled under `-Werror` against the regenerated capnp headers with the
  exact scons command line; **not linked** on the dev host (no libusb library for x86 here -- same reason the
  Rule 9 gate leaves pandad out of scope). The device build is the first link.

---

## 7. Rollback

Revert the commit(s) on the channel; the device updates to the old schema as a whole tree, so no process
ever talks to one of a different schema (mapd's Go binary uses only the `mapd*` members, whose
discriminants this does not change -- the guard checks discriminants).
Logs recorded **while the new schema ran**, read by the old one: fork events are invisible (they are in a
slot the old schema calls `customReserved11`); `healthPacketMismatch` still reads correctly (bit 482);
`madsDisengageReason` reads 0 (byte 74 unwritten). So read that window's logs with the new schema. The
migration does not run backwards; nothing needs to.

---

## 8. On-device verification -- REQUIRED before this is deployed

A schema that fails to load bricks **every** openpilot process. So:

1. **Offline import test in the device venv, BEFORE any reboot** (after the updater has staged the commit,
   against the staged tree; the recipe is DEVICE-TOOLBOX's):
   ```bash
   PYTHONPATH=/data/openpilot:/data/openpilot/opendbc_repo:/data/pnw/agnos19-compat/site-packages \
     /usr/local/venv/bin/python3 -c '
   from cereal import log, custom
   import cereal.messaging
   for s in ("onroadEventsPnw", "pandaStatesPnw", "onroadEvents", "pandaStates", "mapdOut", "madsState"):
       log.Event.new_message().init(s)
   ps = {f.name: f.ordinal.explicit for f in log.PandaState.schema.node.struct.fields}
   assert ps["controlsAllowedLateral"] == 38 and ps["healthPacketMismatch"] == 39
   from openpilot.selfdrive.selfdrived.events import EVENTS, EventNamePnw
   assert EventNamePnw.madsControlsMismatchLateral in EVENTS
   print("schema OK")'
   ```
   Fails -> do not reboot; the staged update must not be installed.
2. **Build-on-boot**: cereal (capnpc) and pandad compile and **link** on the device for the first time --
   check the build log; a link failure keeps pandad down.
3. After the reboot (disengaged is the only condition): manager PIDs stable (pandad, selfdrived, card,
   loggerd not cycling); no `Member was null` / `KjException` / `TypeError: Events.add_from_msg` in the logs.
4. `scripts/device-probe.sh`: `pandaStatesPnw` at ~10 Hz with `len(pandas) == len(pandaStates)` (2 on the
   Raven); on the Raven, `pandaStates[1].healthPacketMismatch` still **True**; `onroadEventsPnw` published
   once `selfdrived` is onroad (ignition on -- remember Rule 3, that is not "driving").
5. First drive: the qlog contains both new services; a green light or a MADS lateral-only stretch shows up
   in `onroadEventsPnw`, and **not** in `onroadEvents`; the alert/banner/chime behave as before.

---

## 9. What this does NOT fix (found on the way -- flagged, not touched)
- **`car.capnp` `SafetyModel @35 mg` / `@36 teslaLegacy` collide with upstream 0.11.2 `byd` / `volvo`.**
  Same class of bug, in opendbc (and the panda firmware's safety-mode numbers). Must be settled before any
  0.11.2 port; out of scope here (Rule 5). `CarState.CruiseState @7 speedClusterUnit` is also a fork field in
  upstream's `car.capnp` (free upstream today).
- **The old `testing` branch put `cesState` at Event @136**, the slot `madsState` uses now -- a fork-internal
  slot reuse across time. Testing-era logs (June 2026) decode that slot as `madsState`. None of the local or
  S3 corpus scanned here is from that era.
- **`selfdrive/car/tests/test_accdrop_pnw.py` is ours but outside the Rule 9 gate** (it excludes all of
  `selfdrive/car/tests`). It was run explicitly for this branch.
- Upstream's own `migrate_onroadEvents` (from `onroadEventsDEPRECATED`) prints and drops events it cannot
  name. Upstream code, pre-existing; noted only.
