"""capnpfork2pnw: the fork must never again put its own ordinals into upstream's log.capnp.

WHAT HAPPENED. The fork added six events at log.capnp OnroadEvent.EventName @99-@104 and three
PandaState fields at @38-@40. Upstream has since allocated every one of those event ordinals itself
(@99 lateralManeuver in 0.11.1, @100-@103 bigModel*/carNotReady in 0.11.2, @104
userBookmarkNotPaired on master) and reserved PandaState @38/@39 as Bool -- so our @39 UInt8 was a
TYPE collision. A capnp ordinal is wire format: on a newer upstream base our
madsControlsMismatchLateral @102 would decode as bigModelFailed, silently. The convention that
forbids this ("fork-specific messages go in custom.capnp -- never in log.capnp") existed the whole
time; nothing enforced it. This file is the enforcement, plus the proof that the fix and the
migration for logs already recorded both work. docs/pnw/CAPNP-FORK-ORDINALS.md.

The fixtures in capnpfork_fixtures/ are REAL: trimmed qlog segments from the car (original message
bytes), with ground truth decoded by each segment's own writer schema (make_fixtures.py).
"""
import importlib.util
import inspect
import json
import os
import pathlib
import struct

import capnp
import pytest
import zstandard

import cereal
from cereal import custom, log
import cereal.messaging as messaging
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.selfdrived.events import (EVENTS, ET, EVENT_NAME, Events, EventNamePnw, PNW_EVENT_BASE,
                                                    AudibleAlert, is_pnw_event)
from openpilot.selfdrive.selfdrived.mads_pnw import has_blocking_event
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
from openpilot.selfdrive.selfdrived.state import StateMachine
from openpilot.selfdrive.test.process_replay import migration as M
from openpilot.tools.lib.logreader import LogReader

EventName = log.OnroadEvent.EventName
State = log.SelfdriveState.OpenpilotState
FIX = pathlib.Path(__file__).parent / "capnpfork_fixtures"
_spec = importlib.util.spec_from_file_location("capnpfork_make_fixtures", FIX / "make_fixtures.py")
mf = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(mf)

# The upstream commit this fork is built on (git merge-base with commaai/master). The whole-schema
# snapshot below was frozen from it: `make_fixtures.py --snapshot a68ea44af341 ...`.
UPSTREAM_BASE = "a68ea44af3418cf80d6b8d34bd1f7299af05c19a"

REAL_FIXTURES = ("lightning_101_104", "lightning_101_103", "lightning_101_100", "lightning_101_99", "tesla_healthmismatch")


def _zst_json(path: pathlib.Path):
  return json.loads(zstandard.ZstdDecompressor().decompress(path.read_bytes()))


def _ours():
  legacy = capnp.load(os.path.join(cereal.CEREAL_PATH, "legacy.capnp"))  # cereal does not export it
  return {k: v for k, v in mf.schema_nodes(log, legacy).items() if v["name"].startswith(("log.capnp:", "legacy.capnp:"))}


# =================================================================================================
# 1. THE GUARD: log.capnp is upstream's, member for member, except for an explicit allowlist.
# =================================================================================================

# Members the fork may have that the upstream base does not: (struct, ordinal) -> (name, type, data
# offset). Nothing else, anywhere in log.capnp/legacy.capnp. Each with the reason it is allowed.
ALLOWED_ADDITIONS = {
  # upstream RESERVED PandaState @38/@39 for forks from 0.11.1 on, as controlsAllowedRESERVED1/2 :Bool.
  # A fork may use them with THAT type only. @39 on bit 482 is load-bearing: it is the bit the old @40
  # healthPacketMismatch used, so logs recorded before the move read back unchanged.
  ("log.capnp:PandaState", "38"): ("controlsAllowedLateral", "bool", 481),
  ("log.capnp:PandaState", "39"): ("healthPacketMismatch", "bool", 482),
  # upstream's OWN later fields, backported: identical to upstream by node id (ordinal, name, type,
  # offset) -- 0.11.1 has the first two, 0.11.2 and master all three (where LivePose is DeviceMotion
  # and LiveDelayData is LateralDelay). A port meets them as no-ops.
  ("log.capnp:LivePose", "8"): ("timestamp", "uint64", 1),
  ("log.capnp:DriverStateV2.DriverData", "14"): ("sleepProb", "float32", 8),
  ("log.capnp:LiveDelayData", "7"): ("version", "int32", 5),
}

# Upstream members the fork renamed. Same ordinal, same type, same discriminant = same wire; a name is
# not wire format. Allowed ONLY for the Event slots upstream reserves for exactly this (custom.capnp:
# "DO rename the structs, DON'T change the identifier"), which the type check below pins to the very
# CustomReservedN struct id upstream put in each slot.
ALLOWED_RENAMES = {
  ("log.capnp:Event", "116"): ("customReserved9", "vtscState"),
  ("log.capnp:Event", "136"): ("customReserved10", "madsState"),
  ("log.capnp:Event", "137"): ("customReserved11", "onroadEventsPnw"),
  ("log.capnp:Event", "138"): ("customReserved12", "pandaStatesPnw"),
  ("log.capnp:Event", "143"): ("customReserved17", "mapdExtendedOut"),
  ("log.capnp:Event", "144"): ("customReserved18", "mapdIn"),
  ("log.capnp:Event", "145"): ("customReserved19", "mapdOut"),
}


def schema_violations(ours: dict, base: dict) -> list[str]:
  """Every way `ours` departs from upstream's `base` on the wire, beyond the allowlists above."""
  v = []
  for nid, b in base.items():
    o = ours.get(nid)
    name = b["name"]
    if o is None:
      v.append(f"{name}: upstream node {nid} is missing here")
      continue
    if b["kind"] == "enum":
      if o["members"] != b["members"]:
        added = {k: n for k, n in o["members"].items() if k not in b["members"]}
        changed = {k: (b["members"][k], o["members"].get(k)) for k in b["members"] if o["members"].get(k) != b["members"][k]}
        v.append(f"{name}: enumerants differ from upstream -- added {added}, changed {changed}. " +
                 "Fork enumerants belong in custom.capnp (see OnroadEventPnw)")
      continue
    for k, bm in b["members"].items():
      om = o["members"].get(k)
      if om is None:
        v.append(f"{name} @{k} {bm[0]}: upstream member missing here")
      elif om[1:] != bm[1:]:
        v.append(f"{name} @{k}: upstream {bm} is {om} here -- a WIRE change")
      elif om[0] != bm[0] and ALLOWED_RENAMES.get((name, k)) != (bm[0], om[0]):
        v.append(f"{name} @{k}: upstream {bm[0]!r} renamed {om[0]!r} -- only the Event customReserved slots may be renamed")
    for k, om in o["members"].items():
      if k in b["members"]:
        continue
      if ALLOWED_ADDITIONS.get((name, k)) != (om[0], om[1], om[3]):
        v.append(f"{name} @{k} {om[0]} :{om[1]} is the FORK's, in upstream's schema -- move it to custom.capnp")
  for nid, o in ours.items():
    if nid not in base:
      v.append(f"{o['name']}: a node upstream does not have, in log.capnp/legacy.capnp -- fork types belong in custom.capnp")
  return v


class TestLogCapnpIsUpstreams:
  def test_log_capnp_adds_nothing_to_upstream(self):
    violations = schema_violations(_ours(), _zst_json(FIX / "upstream_base_log_schema.json.zst"))
    assert violations == [], "\n".join(violations)

  def test_the_guard_catches_the_collision_that_actually_shipped(self):
    """The guard, run on the schema 3devpnw carried until capnpfork2pnw (614dfd50fd), must name every
    collision -- and nothing else. A guard that has never been seen to fail proves nothing."""
    pre = _zst_json(FIX / "pre_capnpfork2pnw_log_schema.json.zst")
    violations = schema_violations(pre, _zst_json(FIX / "upstream_base_log_schema.json.zst"))
    text = "\n".join(violations)
    assert len(violations) == 3, text
    assert any(x.startswith("log.capnp:OnroadEvent.EventName: enumerants differ") for x in violations), text
    for fork_event in ("greenLight", "leadDeparting", "madsLateralOnly", "madsControlsMismatchLateral",
                       "cruiseOffRequested", "madsResumeSetTooHigh"):
      assert fork_event in text, fork_event
    assert any("PandaState @39 madsDisengageReason :uint8" in x for x in violations), text
    assert any("PandaState @40 healthPacketMismatch :bool" in x for x in violations), text

  @pytest.mark.parametrize("change", ["type", "discriminant", "offset", "rename", "deleted member", "deleted node",
                                      "added member", "added node", "enumerant added", "enumerant renamed"])
  def test_the_guard_flags_each_kind_of_wire_change(self, change):
    """Each kind of departure, applied to one upstream member that the fork does NOT touch, must be
    named. The collision that shipped only exercised "addition"; these pin the rest."""
    base = _zst_json(FIX / "upstream_base_log_schema.json.zst")
    ours = json.loads(json.dumps(base))  # an exact copy is clean...
    assert schema_violations(ours, base) == []
    ps = next(k for k, v in ours.items() if v["name"] == "log.capnp:PandaState")
    ev = next(k for k, v in ours.items() if v["name"] == "log.capnp:Event")
    en = next(k for k, v in ours.items() if v["name"] == "log.capnp:OnroadEvent.EventName")
    m = ours[ps]["members"]["37"]  # soundOutputLevel :UInt16
    if change == "type":
      m[1] = "uint8"
    elif change == "discriminant":
      ours[ev]["members"]["107"][2] += 1
    elif change == "offset":
      m[3] += 1
    elif change == "rename":
      m[0] = "pnwSoundLevel"
    elif change == "deleted member":
      del ours[ps]["members"]["37"]
    elif change == "deleted node":
      del ours[ps]
    elif change == "added member":
      ours[ps]["members"]["40"] = ["madsDisengageReason", "uint8", None, 74]  # the exact shape that shipped
    elif change == "added node":
      ours["0xd2e9f5a1b3c4e5f6"] = {"kind": "struct", "name": "log.capnp:PnwOops", "members": {}}
    elif change == "enumerant added":
      ours[en]["members"]["99"] = "pnwEvent"
    elif change == "enumerant renamed":
      ours[en]["members"]["98"] = "pnwEvent"
    assert len(schema_violations(ours, base)) == 1, change  # ...and exactly one departure is exactly one finding

  def test_event_name_is_exactly_the_upstream_base(self):
    base = _zst_json(FIX / "upstream_base_log_schema.json.zst")
    enum_id = f"{log.OnroadEvent.EventName.schema.node.id:#x}"
    upstream = {int(k): n for k, n in base[enum_id]["members"].items()}
    ours = {v: k for k, v in EventName.schema.enumerants.items()}
    assert ours == upstream
    assert len(ours) == 99 and ours[98] == "stockLkas"

  # Upstream renamed some of these events after our base -- the SAME event, same ordinal, so the same
  # wire. (ours, upstream) pairs, measured 2026-09-21 against capnpfork_fixtures/upstream_eventnames.json.
  SAME_EVENT_RENAMED_UPSTREAM = {
    ("preDriverDistracted", "driverDistracted1"), ("promptDriverDistracted", "driverDistracted2"),
    ("driverDistracted", "driverDistracted3"), ("preDriverUnresponsive", "driverUnresponsive1"),
    ("promptDriverUnresponsive", "driverUnresponsive2"), ("driverUnresponsive", "driverUnresponsive3"),
    ("lowBattery", "lowBatteryDEPRECATED"), ("deviceFalling", "deviceFallingDEPRECATED"),
    ("usbError", "usbErrorDEPRECATED"), ("audioFeedback", "audioFeedbackDEPRECATED"),
  }

  def _upstream_releases(self):
    rel = json.loads((FIX / "upstream_eventnames.json").read_text())
    return {k: v["eventNames"] for k, v in rel.items() if not k.startswith("_")}

  def test_every_ordinal_we_define_means_the_same_event_upstream(self):
    """A port onto 0.11.1, 0.11.2 or today's master only APPENDS to our EventName: every ordinal we
    have is the same event there. That is what makes the eventual port a no-op for this enum."""
    ours = [n for n, _ in sorted(EventName.schema.enumerants.items(), key=lambda kv: kv[1])]
    releases = self._upstream_releases()
    assert set(releases) == {"0.11.1", "0.11.2", "master"}
    for release, up in releases.items():
      assert len(up) > len(ours), f"{release} does not extend our EventName"
      for o, name in enumerate(ours):
        assert up[o] == name or (name, up[o]) in self.SAME_EVENT_RENAMED_UPSTREAM, f"{release} @{o}: ours {name}, upstream {up[o]}"

  def test_no_fork_event_name_is_used_by_any_upstream_release(self):
    """The regression, stated directly: none of the fork's events sits in upstream's EventName at all."""
    fork = set(custom.OnroadEventPnw.EventName.schema.enumerants)
    assert not fork & set(EventName.schema.enumerants)
    for up in self._upstream_releases().values():
      assert not fork & set(up)

  def test_pandastate_only_uses_the_reserved_bool_slots(self):
    fields = {f.name: f for f in log.PandaState.schema.node.struct.fields}
    fork = {f.ordinal.explicit: (n, f.slot.type.which(), f.slot.offset) for n, f in fields.items()
            if f.which() == "slot" and f.ordinal.explicit >= 38}
    assert fork == {38: ("controlsAllowedLateral", "bool", 481), 39: ("healthPacketMismatch", "bool", 482)}

  def test_fork_messages_sit_in_reserved_event_slots(self):
    ev = {f.name: f for f in log.Event.schema.node.struct.fields}
    assert ev["onroadEventsPnw"].ordinal.explicit == 137
    assert ev["pandaStatesPnw"].ordinal.explicit == 138
    # the CustomReserved11/12 struct ids, unchanged -- the "DON'T change the identifier" half
    assert custom.OnroadEventsPnw.schema.node.id == 0xc2243c65e0340384
    assert custom.PandaStatesPnw.schema.node.id == 0x9ccdc8676701b412

  def test_fork_event_flags_mirror_upstream_onroad_event(self):
    """OnroadEventPnw carries the same event-type flags, with the same ordinals, as log.OnroadEvent."""
    up = {f.name: f.ordinal.explicit for f in log.OnroadEvent.schema.node.struct.fields if f.name != "name"}
    ours = {f.name: f.ordinal.explicit for f in custom.OnroadEventPnw.schema.node.struct.fields if f.name != "name"}
    assert ours == up
    assert set(ours) == set(M._ONROAD_EVENT_FLAGS)

  def test_fork_enum_keeps_the_old_relative_order(self):
    """AlertManager breaks (priority, start_frame) ties by insertion order, which follows the sorted
    event list. The fork's events must sort among themselves exactly as they did at @99-@104."""
    enum = custom.OnroadEventPnw.EventName.schema.enumerants
    assert [n for n, _ in sorted(enum.items(), key=lambda kv: kv[1])] == [M.PNW_FORK_V1_ORDINALS[o] for o in range(99, 105)]


# =================================================================================================
# 2. THE EVENTS STILL WORK -- above all the MADS safety event.
# =================================================================================================

class TestForkEventsWiring:
  def test_every_fork_event_has_an_alert_entry(self):
    for name, v in custom.OnroadEventPnw.EventName.schema.enumerants.items():
      assert PNW_EVENT_BASE + v in EVENTS, f"{name} missing from EVENTS"

  def test_keys_are_disjoint_and_sort_after_every_upstream_event(self):
    upstream = set(EventName.schema.enumerants.values())
    fork = set(vars(EventNamePnw).values())
    assert not upstream & fork
    assert max(upstream) < PNW_EVENT_BASE <= min(fork)
    assert all(is_pnw_event(k) for k in fork) and not any(is_pnw_event(k) for k in upstream)
    assert (1 << 16) == PNW_EVENT_BASE, "must exceed any UInt16 enumerant"

  def test_event_names_render_as_before(self):
    """alertType strings (and alert_renderer's green banner) are built from EVENT_NAME."""
    for name in ("greenLight", "leadDeparting", "madsLateralOnly", "madsControlsMismatchLateral", "cruiseOffRequested"):
      assert EVENT_NAME[getattr(EventNamePnw, name)] == name
    assert len(set(EVENT_NAME.values())) == len(EVENT_NAME)

  def test_to_msg_and_to_msg_pnw_partition_the_events(self):
    ev = Events()
    for e in (EventName.pcmDisable, EventNamePnw.madsLateralOnly, EventName.steerSaturated, EventNamePnw.cruiseOffRequested):
      ev.add(e)
    up = ev.to_msg()
    pnw = ev.to_msg_pnw()
    assert sorted(e.name.raw for e in up) == sorted([EventName.pcmDisable, EventName.steerSaturated])
    assert sorted(str(e.name) for e in pnw) == ["cruiseOffRequested", "madsLateralOnly"]
    assert pnw[[str(e.name) for e in pnw].index("cruiseOffRequested")].noEntry
    assert len(up) + len(pnw) == len(ev)

  def test_add_from_msg_refuses_fork_events(self):
    """An OnroadEventPnw's raw ordinal is 0..5 = upstream canError..seatbeltNotLatched. Silently
    accepting one would turn cruiseOffRequested into seatbeltNotLatched."""
    ev = Events()
    msgs = [custom.OnroadEventPnw.new_message(name="cruiseOffRequested")]
    with pytest.raises(TypeError):
      ev.add_from_msg(msgs)
    ev.add_from_msg([log.OnroadEvent.new_message(name="pcmDisable")])
    assert ev.names == [EventName.pcmDisable]

  def test_no_fork_event_carries_an_override_type(self):
    """controlsd and joystickd read overrideLateral/Longitudinal from onroadEvents ONLY. A fork event
    with an override type would now be invisible to them."""
    for name, v in custom.OnroadEventPnw.EventName.schema.enumerants.items():
      types = EVENTS[PNW_EVENT_BASE + v]
      assert ET.OVERRIDE_LATERAL not in types and ET.OVERRIDE_LONGITUDINAL not in types, name

  def test_selfdrived_publishes_fork_events_first_under_the_same_condition(self):
    src = inspect.getsource(SelfdriveD.publish_selfdriveState)
    cond = src.index("if (self.sm.frame % int(1. / DT_CTRL) == 0) or (self.events.names != self.events_prev):")
    pnw = src.index("self.pm.send('onroadEventsPnw', pe_send)")
    up = src.index("self.pm.send('onroadEvents', ce_send)")
    assert cond < pnw < up
    inside = src[cond:up].splitlines()[1:]
    assert all(line.startswith("      ") or not line.strip() for line in inside), "both sends must be inside the one if-block"
    assert "'onroadEventsPnw'" in inspect.getsource(SelfdriveD.__init__)

  def test_the_mads_safety_event_survives_the_wire(self):
    """madsControlsMismatchLateral end to end: raised -> onroadEventsPnw -> bytes -> back, with its
    IMMEDIATE_DISABLE intact, and never in onroadEvents where upstream calls @102 bigModelFailed."""
    ev = Events()
    ev.add(EventNamePnw.madsControlsMismatchLateral)
    assert ev.to_msg() == []
    msg = messaging.new_message('onroadEventsPnw')
    msg.onroadEventsPnw.events = ev.to_msg_pnw()
    with log.Event.from_bytes(msg.to_bytes()) as back:
      (e,) = back.onroadEventsPnw.events
      assert str(e.name) == "madsControlsMismatchLateral"
      assert e.immediateDisable and e.noEntry and e.permanent
    assert has_blocking_event(ev), "mads_pnw must still end the lateral-only state on it"

  def test_the_mads_safety_event_still_reaches_the_driver(self):
    """The same property as test_mads_pnw's test_event_actually_reaches_the_driver, via the moved key."""
    sm = StateMachine()
    ev = Events()
    ev.add(EventNamePnw.madsControlsMismatchLateral)
    sm.update(ev)
    assert sm.state == State.disabled and ET.PERMANENT in sm.current_alert_types
    alerts = ev.create_alerts(sm.current_alert_types, [None, None, None, False, 0, 0])
    assert len(alerts) == 1 and alerts[0].audible_alert != AudibleAlert.none
    assert alerts[0].duration >= int(1.0 / DT_CTRL)
    assert alerts[0].alert_type == "madsControlsMismatchLateral/permanent"


# =================================================================================================
# 3. THE MIGRATION -- logs recorded with the old ordinals.
# =================================================================================================

def _read(name):
  return list(LogReader(str(FIX / f"{name}.qlog.zst"), sort_by_time=True))


def _expected(name):
  return _zst_json(FIX / f"{name}.expected.json.zst")


def _with_init(lr, **changes):
  """The same log with initData fields changed (a writer the migration must refuse or treat otherwise)."""
  out = []
  for m in lr:
    if m.which() == "initData":
      b = m.as_builder()
      for k, v in changes.items():
        setattr(b.initData, k, v)
      m = b.as_reader()
    out.append(m)
  return out


def _onroad_event_msg(raw: int, t: int, **flags):
  """An onroadEvents message whose single event has raw ordinal `raw` -- one this schema cannot even
  write (setting an unknown enumerant raises), so it is built as stockLkas and the uint16 patched."""
  m = messaging.new_message('onroadEvents', 1, valid=True, logMonoTime=t)
  m.onroadEvents[0].name = "stockLkas"
  for f, v in flags.items():
    setattr(m.onroadEvents[0], f, v)
  b = bytearray(m.to_bytes())
  # the list's single element is the LAST word written; its first 16 bits are `name`
  (cur,) = struct.unpack_from("<H", b, len(b) - 8)
  assert cur == EventName.stockLkas, "message layout not as expected -- refusing to patch blind"
  struct.pack_into("<H", b, len(b) - 8, raw)
  with log.Event.from_bytes(bytes(b)) as r:
    out = r.as_builder().as_reader()
  assert out.onroadEvents[0].name.raw == raw
  return out


class TestMigrationOnRealLogs:
  @pytest.mark.parametrize("name", REAL_FIXTURES)
  def test_fixture_is_a_pre_fix_log(self, name):
    """The fixtures must actually exercise the problem: under this schema their fork events are
    unreadable ("Member was null"), not silently something else. And the ground truth must be the
    writer's own decode."""
    exp = _expected(name)
    lr = _read(name)
    (init,) = [m for m in lr if m.which() == "initData"]
    assert init.initData.gitCommit == exp["writerCommit"]
    raws = [e.name.raw for m in lr if m.which() == "onroadEvents" for e in m.onroadEvents]
    if name.startswith("lightning"):
      assert any(r >= M.PNW_FIRST_FORK_ORDINAL for r in raws)
      bad = next(e for m in lr if m.which() == "onroadEvents" for e in m.onroadEvents if e.name.raw >= 99)
      with pytest.raises(Exception, match="Member was null"):
        str(bad.name)

  @pytest.mark.parametrize("name", REAL_FIXTURES)
  def test_migrated_events_equal_the_writers_own_decode(self, name):
    exp = [m for m in _expected(name)["messages"] if m["which"] == "onroadEvents"]
    out = M.migrate_all(_read(name))
    onroad = [m for m in out if m.which() == "onroadEvents"]
    pnw = [m for m in out if m.which() == "onroadEventsPnw"]
    assert [m.logMonoTime for m in onroad] == [m["logMonoTime"] for m in exp]
    fork_in_truth = any(e["raw"] >= 99 for m in exp for e in m["events"])
    assert len(pnw) == (len(exp) if fork_in_truth else 0), "one onroadEventsPnw per onroadEvents, as selfdrived now sends"

    by_t = {m.logMonoTime: m for m in pnw}
    for got, want in zip(onroad, exp, strict=True):
      assert all(e.name.raw < 99 for e in got.onroadEvents), "a contested ordinal survived in onroadEvents"
      want_up = [e for e in want["events"] if e["raw"] < 99]
      want_fork = [e for e in want["events"] if e["raw"] >= 99]
      assert [str(e.name) for e in got.onroadEvents] == [e["name"] for e in want_up]
      for g, w in zip(got.onroadEvents, want_up, strict=True):
        assert {f: getattr(g, f) for f in M._ONROAD_EVENT_FLAGS} == {f: w[f] for f in M._ONROAD_EVENT_FLAGS}
      if fork_in_truth:
        p = by_t[got.logMonoTime].onroadEventsPnw.events
        assert [str(e.name) for e in p] == [e["name"] for e in want_fork]
        for g, w in zip(p, want_fork, strict=True):
          assert {f: getattr(g, f) for f in M._ONROAD_EVENT_FLAGS} == {f: w[f] for f in M._ONROAD_EVENT_FLAGS}

  @pytest.mark.parametrize("name", ("lightning_101_104", "tesla_healthmismatch"))
  def test_old_pandastates_read_correctly_with_no_migration(self, name):
    """healthPacketMismatch moved @40 -> @39 onto the SAME data bit; controlsAllowedLateral kept @38.
    So the new schema reads the old bytes to exactly the writer's values, sample for sample --
    including the Raven's second panda (healthPacketMismatch True) and Lightning samples whose old
    @39 madsDisengageReason byte is non-zero (it must bleed into nothing)."""
    exp = [m for m in _expected(name)["messages"] if m["which"] == "pandaStates"]
    got = [m for m in M.migrate_all(_read(name)) if m.which() == "pandaStates"]
    assert len(got) == len(exp) > 500
    seen = set()
    for g, w in zip(got, exp, strict=True):
      assert g.logMonoTime == w["logMonoTime"]
      for gp, wp in zip(g.pandaStates, w["pandas"], strict=True):
        assert (gp.controlsAllowed, gp.controlsAllowedLateral, gp.healthPacketMismatch) == \
               (wp["controlsAllowed"], wp["controlsAllowedLateral"], wp["healthPacketMismatch"])
        seen.add((wp["healthPacketMismatch"], wp["madsDisengageReason"] != 0))
    want = {(True, False)} if name.startswith("tesla") else {(False, True)}
    assert want <= seen, f"fixture no longer covers {want - seen}"

  def test_real_log_covers_five_of_the_six_events(self):
    """What the real corpus can show. madsControlsMismatchLateral has NEVER been recorded: zero in all
    5,492 qlogs uploaded since it was introduced, because its path needs PandaMadsSafety=1 and a
    reflashed panda (neither has happened). It is tested below by injection into a real segment."""
    names = {e["name"] for n in REAL_FIXTURES for m in _expected(n)["messages"] for e in m.get("events", []) if e["raw"] >= 99}
    assert names == {"greenLight", "leadDeparting", "madsLateralOnly", "cruiseOffRequested", "madsResumeSetTooHigh"}

  def test_injected_mads_mismatch_maps_to_the_safety_event_and_only_there(self):
    """@102 injected into a REAL segment whose writer (705c74b931) defines it. It must come out as
    madsControlsMismatchLateral with its flags -- never dropped, never renamed, never left behind."""
    lr = _read("lightning_101_104")
    t = next(m.logMonoTime for m in lr if m.which() == "onroadEvents") + 1
    lr.append(_onroad_event_msg(102, t, immediateDisable=True, noEntry=True, permanent=True))
    out = M.migrate_all(lr)
    (p,) = [m for m in out if m.which() == "onroadEventsPnw" and m.logMonoTime == t]
    (e,) = p.onroadEventsPnw.events
    assert str(e.name) == "madsControlsMismatchLateral"
    assert (e.immediateDisable, e.noEntry, e.permanent, e.softDisable) == (True, True, True, False)
    (o,) = [m for m in out if m.which() == "onroadEvents" and m.logMonoTime == t]
    assert len(o.onroadEvents) == 0
    total = sum(1 for m in out if m.which() == "onroadEventsPnw" for x in m.onroadEventsPnw.events
                if str(x.name) == "madsControlsMismatchLateral")
    assert total == 1

  def test_already_migrated_or_new_logs_are_left_alone(self):
    once = M.migrate_all(_read("lightning_101_103"))
    twice = M.migrate_all(once)
    assert [m.as_builder().to_bytes() for m in twice] == [m.as_builder().to_bytes() for m in once]

  def test_a_log_with_no_contested_ordinal_needs_no_writer(self):
    """No @99+ anywhere: the outcome is the same whoever wrote it, so even an unknowable writer is fine."""
    lr = [m for m in _read("tesla_healthmismatch") if m.which() != "initData"]
    out = M.migrate_all(lr)
    assert not any(m.which() == "onroadEventsPnw" for m in out)


class TestMigrationRefusesToGuess:
  """Every way the writer cannot be established must RAISE -- never map, never drop."""

  def _lr(self):
    return _read("lightning_101_103")

  def test_no_init_data(self):
    with pytest.raises(M.PnwLogSchemaError, match="no initData"):
      M.migrate_all([m for m in self._lr() if m.which() != "initData"])

  def test_empty_commit(self):
    with pytest.raises(M.PnwLogSchemaError, match="writer commit"):
      M.migrate_all(_with_init(self._lr(), gitCommit=""))

  def test_two_writers(self):
    lr = self._lr()
    other = _with_init([m for m in lr if m.which() == "initData"], gitCommit="c31acd4593da21c38afb6eae062483bca0eed292")
    with pytest.raises(M.PnwLogSchemaError, match="2 writer commit"):
      M.migrate_all(lr + other)

  def test_dirty_writer(self):
    with pytest.raises(M.PnwLogSchemaError, match="DIRTY"):
      M.migrate_all(_with_init(self._lr(), dirty=True))

  def test_commit_git_does_not_have(self):
    with pytest.raises(M.PnwLogSchemaError, match="cannot be read from the git repo"):
      M.migrate_all(_with_init(self._lr(), gitCommit="0123456789abcdef0123456789abcdef01234567"))

  def test_writer_schema_without_that_ordinal(self):
    """3testpnw-era writer d54a88e4e82e only defined @99/@100: a log claiming it but carrying @103
    does not match its own initData."""
    with pytest.raises(M.PnwLogSchemaError, match="has no @101"):
      M.migrate_all(_with_init(self._lr(), gitCommit="d54a88e4e82e84f032975bc0b617c596fa98930f"))

  def test_upstream_0_11_2_writer_is_never_read_as_the_fork(self):
    """THE silent mis-map this whole effort prevents, from the other side: upstream 0.11.2's @103 is
    carNotReady. A 0.11.2 log carrying it must not become cruiseOffRequested -- and since this schema
    cannot represent carNotReady either, it raises rather than keep an unreadable event."""
    lr = [m for m in self._lr() if m.which() == "initData"] + [_onroad_event_msg(103, 10**9, noEntry=True)]
    with pytest.raises(M.PnwLogSchemaError, match="carNotReady"):
      M.migrate_all(_with_init(lr, gitCommit="473eba53f280743d09c5bebc2d0613ea46883e02"))

  def test_fork_event_at_an_ordinal_no_audited_build_used(self, monkeypatch):
    """No real writer ever put a fork event anywhere but PNW_FORK_V1_ORDINALS -- which is exactly why a
    writer that did must be refused rather than trusted: nobody has checked that build."""
    shifted = {**M.pnw_writer_event_names(_expected("lightning_101_103")["writerCommit"]), 103: "madsLateralOnly"}
    monkeypatch.setattr(M, "pnw_writer_event_names", lambda commit: shifted)
    with pytest.raises(M.PnwLogSchemaError, match="no audited build did"):
      M.migrate_all(self._lr())

  def test_override_fork_v1_is_explicit_and_warned(self, monkeypatch):
    monkeypatch.setenv(M.PNW_WRITER_SCHEMA_ENV, "fork-v1")
    with pytest.warns(UserWarning, match="ASSERTED"):
      out = M.migrate_all(_with_init(self._lr(), gitCommit="0123456789abcdef0123456789abcdef01234567"))
    assert any(str(e.name) == "cruiseOffRequested" for m in out if m.which() == "onroadEventsPnw" for e in m.onroadEventsPnw.events)

  def test_override_is_reported_on_every_use_not_just_the_first(self, monkeypatch, capsys):
    # warnings.warn alone is de-duplicated per call site, so a batch of logs would warn once and then go
    # quiet. Every migrated log must say its writer schema was asserted.
    monkeypatch.setenv(M.PNW_WRITER_SCHEMA_ENV, "upstream")
    with pytest.warns(UserWarning):
      M.migrate_all(self._lr())
      M.migrate_all(self._lr())
    assert capsys.readouterr().err.count("writer schema is ASSERTED") == 2

  def test_override_upstream_leaves_the_log(self, monkeypatch):
    monkeypatch.setenv(M.PNW_WRITER_SCHEMA_ENV, "upstream")
    with pytest.warns(UserWarning):
      out = M.migrate_all(self._lr())
    assert not any(m.which() == "onroadEventsPnw" for m in out)

  def test_override_must_be_a_known_value(self, monkeypatch):
    monkeypatch.setenv(M.PNW_WRITER_SCHEMA_ENV, "yes")
    with pytest.warns(UserWarning), pytest.raises(M.PnwLogSchemaError, match="must be"):
      M.migrate_all(self._lr())

  def test_override_fork_v1_refuses_an_ordinal_it_never_had(self, monkeypatch):
    monkeypatch.setenv(M.PNW_WRITER_SCHEMA_ENV, "fork-v1")
    lr = [m for m in self._lr() if m.which() == "initData"] + [_onroad_event_msg(105, 10**9)]
    with pytest.warns(UserWarning), pytest.raises(M.PnwLogSchemaError, match="never defined"):
      M.migrate_all(lr)


class TestWriterSchemaFromGit:
  def test_every_fork_writer_in_the_fixtures_is_resolved(self):
    for name in REAL_FIXTURES:
      w = M.pnw_writer_event_names(_expected(name)["writerCommit"])
      assert w[98] == "stockLkas" and w[99] == "greenLight"

  def test_upstream_and_base_writers_parse(self):
    assert max(M.pnw_writer_event_names(UPSTREAM_BASE)) == 98
    assert M.pnw_writer_event_names("473eba53f280743d09c5bebc2d0613ea46883e02")[103] == "carNotReady"  # 0.11.2 layout
