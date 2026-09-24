"""cesarchive2pnw: ces_events generations reach the archive (and so S3) at ROTATION, not at eviction.

Measured on the device 2026-09-23: since parkgate2pnw stopped logging a parked truck, the 8-generation
ring covered 2026-09-19 07:49 -> 09-23 12:53 PT and NONE of it was archived -- a generation only moved
into ces_archive when it fell off the end of the ring, so telemetry reached S3 ~4 days late.

Fix, and what these tests pin:
  1. rotate_event_log hardlinks the new .1 into the archive immediately, named exactly as the
     eviction path would name it (one shared helper, _archive_dest).
  2. eviction of a generation that is already linked (nlink > 1, other link PROVEN to be an uploadable
     archive entry) just unlinks the ring copy -- no second archive name, no double upload.
  3. the live file is rotated at the end of a drive (the Park edge), so the drive's tail is archived
     that night.
"""
import os

import pytest
from cereal import car

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw.park_tick_gate import ParkTickGate

GearShifter = car.CarState.GearShifter
BASE = "ces_events.jsonl"
GOOD_MTIME = 1789000000.0          # 2026-09-10T00:26:40Z
GOOD_NAME = f"{BASE}.20260910T002640Z"


@pytest.fixture
def arc(tmp_path, monkeypatch):
  a = tmp_path / "arc"
  monkeypatch.setattr(m, "CES_ARCHIVE_DIR", str(a))
  return a


@pytest.fixture
def said(monkeypatch):
  out = {"error": [], "warning": [], "exception": [], "event": []}
  monkeypatch.setattr(m.cloudlog, "error", lambda msg, *a, **k: out["error"].append(msg))
  monkeypatch.setattr(m.cloudlog, "warning", lambda msg, *a, **k: out["warning"].append(msg))
  monkeypatch.setattr(m.cloudlog, "exception", lambda msg, *a, **k: out["exception"].append(msg))
  monkeypatch.setattr(m.cloudlog, "event", lambda name, *a, **k: out["event"].append((name, k)))
  return out


def _read(p):
  with open(p) as f:
    return f.read()


def _live(tmp_path, text="live\n", mtime=GOOD_MTIME):
  p = tmp_path / BASE
  p.write_text(text)
  os.utime(p, (mtime, mtime))
  return p


# ---------------------------------------------------------------------------------------------------
# 1. hardlink at rotation
# ---------------------------------------------------------------------------------------------------
class TestLinkAtRotation:
  def test_the_new_generation_1_is_hardlinked_into_the_archive_with_the_mtime_name(self, tmp_path, arc):
    live = _live(tmp_path)
    m.rotate_event_log(str(live), 8)
    gen1 = tmp_path / f"{BASE}.1"
    names = [p.name for p in arc.iterdir()]
    assert names == [GOOD_NAME], names
    assert os.stat(arc / GOOD_NAME).st_ino == os.stat(gen1).st_ino, "a LINK, not a copy"
    assert os.stat(gen1).st_nlink == 2

  def test_a_dead_clock_generation_gets_the_random_suffix(self, tmp_path, arc):
    live = _live(tmp_path, mtime=60.0)                  # 1970-01-01T00:01:00Z: dead-RTC boot
    m.rotate_event_log(str(live), 8)
    (name,) = [p.name for p in arc.iterdir()]
    assert name.startswith(f"{BASE}.19700101T000100Z.b"), name
    assert len(name) == len(f"{BASE}.19700101T000100Z.b") + 8, name

  def test_link_and_move_choose_the_same_name(self, tmp_path):
    """The two paths share _archive_dest, so they cannot drift (the uploader's name gate and the
    lexical oldest-first walk depend on this exact shape)."""
    a1, a2 = tmp_path / "a1", tmp_path / "a2"
    live = _live(tmp_path)
    gen8 = tmp_path / f"{BASE}.8"
    gen8.write_text("old\n")
    os.utime(gen8, (GOOD_MTIME, GOOD_MTIME))
    moved = m.archive_rotated_generation(str(live), 8, str(a1), max_bytes=10 ** 9)
    linked = m.link_rotated_generation(str(live), BASE, str(a2), max_bytes=10 ** 9)
    assert os.path.basename(moved) == os.path.basename(linked) == GOOD_NAME

  def test_a_name_collision_gets_the_dot_n_suffix(self, tmp_path, arc):
    arc.mkdir()
    (arc / GOOD_NAME).write_text("someone else\n")
    live = _live(tmp_path)
    dest = m.link_rotated_generation(str(live), BASE)
    assert os.path.basename(dest) == f"{GOOD_NAME}.1"
    assert (arc / GOOD_NAME).read_text() == "someone else\n"

  def test_a_failing_link_is_loud_and_the_rotation_still_happens(self, tmp_path, arc, said, monkeypatch):
    def boom(src, dst):
      raise OSError(18, "Invalid cross-device link")
    monkeypatch.setattr(m.os, "link", boom)
    live = _live(tmp_path)
    m.rotate_event_log(str(live), 8)                    # must not raise
    assert (tmp_path / f"{BASE}.1").read_text() == "live\n"
    assert any("hardlink FAILED" in s and "EVICTION" in s for s in said["error"]), said["error"]

  def test_the_archive_budget_is_enforced_after_the_link(self, tmp_path, arc, monkeypatch):
    monkeypatch.setattr(m, "CES_ARCHIVE_MAX_BYTES", 10)
    arc.mkdir()
    old = arc / f"{BASE}.20200101T000000Z"
    old.write_text("x" * 100)
    os.utime(old, (1577836900.0, 1577836900.0))
    live = _live(tmp_path, text="y" * 5)
    m.rotate_event_log(str(live), 8)
    assert [p.name for p in arc.iterdir()] == [GOOD_NAME], "the oldest archive file is pruned"


# ---------------------------------------------------------------------------------------------------
# 2. eviction of an already-linked generation
# ---------------------------------------------------------------------------------------------------
def _ring(tmp_path, n=8):
  for i in range(1, n + 1):
    p = tmp_path / f"{BASE}.{i}"
    p.write_text(f"gen{i}\n")
    os.utime(p, (GOOD_MTIME - i * 3600, GOOD_MTIME - i * 3600))


class TestEviction:
  def test_an_already_linked_generation_is_unlinked_not_archived_twice(self, tmp_path, arc):
    _ring(tmp_path)
    arc.mkdir()
    os.link(tmp_path / f"{BASE}.8", arc / f"{BASE}.whatever-name-the-owner-chose")
    live = _live(tmp_path)

    dest = m.archive_rotated_generation(str(live), 8, str(arc), max_bytes=10 ** 9)

    assert dest == str(arc / f"{BASE}.whatever-name-the-owner-chose")
    assert not (tmp_path / f"{BASE}.8").exists()
    assert [p.name for p in arc.iterdir()] == [f"{BASE}.whatever-name-the-owner-chose"], "no second name"
    assert (arc / f"{BASE}.whatever-name-the-owner-chose").read_text() == "gen8\n"
    assert os.stat(dest).st_nlink == 1

  def test_the_legacy_unlinked_generation_is_still_moved(self, tmp_path, arc):
    _ring(tmp_path)
    live = _live(tmp_path)
    ino = os.stat(tmp_path / f"{BASE}.8").st_ino
    dest = m.archive_rotated_generation(str(live), 8, str(arc), max_bytes=10 ** 9)
    assert os.stat(dest).st_ino == ino and _read(dest) == "gen8\n"
    assert not (tmp_path / f"{BASE}.8").exists()

  def test_a_link_that_lives_OUTSIDE_the_archive_does_not_count(self, tmp_path, arc, said):
    """nlink alone does not say where the other link is. Moving in is the safe fallback."""
    _ring(tmp_path)
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir()
    os.link(tmp_path / f"{BASE}.8", elsewhere / "copy")
    live = _live(tmp_path)
    dest = m.archive_rotated_generation(str(live), 8, str(arc), max_bytes=10 ** 9)
    assert dest is not None and _read(dest) == "gen8\n"
    assert os.path.dirname(dest) == str(arc)
    assert any("no uploadable link" in s for s in said["warning"])

  def test_a_link_under_a_non_uploadable_name_does_not_count(self, tmp_path, arc):
    """The uploader only takes `ces_events.jsonl.<...>`; a link under another name never reaches S3."""
    _ring(tmp_path)
    arc.mkdir()
    os.link(tmp_path / f"{BASE}.8", arc / "not-the-prefix")
    live = _live(tmp_path)
    dest = m.archive_rotated_generation(str(live), 8, str(arc), max_bytes=10 ** 9)
    assert os.path.basename(dest).startswith(f"{BASE}.") and _read(dest) == "gen8\n"

  def test_an_unscannable_archive_falls_back_to_the_move_and_says_so(self, tmp_path, arc, said, monkeypatch):
    _ring(tmp_path)
    arc.mkdir()
    os.link(tmp_path / f"{BASE}.8", arc / f"{BASE}.x")

    def boom(_):
      raise PermissionError(13, "Permission denied")
    monkeypatch.setattr(m.os, "scandir", boom)
    live = _live(tmp_path)
    dest = m.archive_rotated_generation(str(live), 8, str(arc), max_bytes=10 ** 9)
    assert dest is not None and _read(dest) == "gen8\n", "data must never be dropped on a guess"
    assert any("scan FAILED" in s for s in said["error"])

  def test_a_failing_unlink_is_loud_and_never_raises(self, tmp_path, arc, said, monkeypatch):
    _ring(tmp_path)
    arc.mkdir()
    os.link(tmp_path / f"{BASE}.8", arc / f"{BASE}.x")

    def boom(p):
      raise PermissionError(13, "Permission denied")
    monkeypatch.setattr(m.os, "unlink", boom)
    live = _live(tmp_path)
    dest = m.archive_rotated_generation(str(live), 8, str(arc), max_bytes=10 ** 9)
    assert dest == str(arc / f"{BASE}.x")
    assert any("could not unlink ring copy" in s for s in said["error"])

  def test_many_rotations_archive_every_generation_exactly_once(self, tmp_path, arc):
    """End to end: 20 rotations through an 8-ring. Every generation lands once, none twice, none lost."""
    live = tmp_path / BASE
    for i in range(20):
      live.write_text(f"drive{i}\n")
      os.utime(live, (GOOD_MTIME + i * 3600, GOOD_MTIME + i * 3600))
      m.rotate_event_log(str(live), 8)
    contents = sorted(p.read_text() for p in arc.iterdir())
    assert contents == sorted(f"drive{i}\n" for i in range(20))
    ring = [tmp_path / f"{BASE}.{i}" for i in range(1, 9)]
    assert all(os.stat(p).st_nlink == 2 for p in ring), "every ring file shares its inode with the archive"

  def test_a_pruned_archive_file_still_in_the_ring_is_moved_back_at_eviction(self, tmp_path, arc):
    """prune_ces_archive only removes the ARCHIVE's link; the ring keeps the data (nlink drops to 1),
    and eviction then takes the legacy move path -- the data is not lost, only re-archived."""
    live = _live(tmp_path)
    m.rotate_event_log(str(live), 1)
    (linked,) = list(arc.iterdir())
    os.remove(linked)                                    # what prune does to the oldest file
    assert os.stat(tmp_path / f"{BASE}.1").st_nlink == 1
    _live(tmp_path, text="next\n", mtime=GOOD_MTIME + 60)
    m.rotate_event_log(str(live), 1)
    assert sorted(p.read_text() for p in arc.iterdir()) == ["live\n", "next\n"]


# ---------------------------------------------------------------------------------------------------
# 3. rotation at the end of a drive
# ---------------------------------------------------------------------------------------------------
class TestParkEdge:
  def _run(self, gears, v=0.0, gate_on=True):
    g = ParkTickGate()
    edges = []
    for i, gear in enumerate(gears):
      g.update(gear, v, float(i), gate_on)
      edges.append(g.park_edge)
    return edges

  def test_drive_then_park_is_one_edge_however_long_it_stays_parked(self):
    edges = self._run([GearShifter.drive] * 5 + [GearShifter.park] * 3600)
    assert edges.count(True) == 1 and edges[5] is True

  def test_a_boot_in_park_is_no_edge(self):
    assert True not in self._run([None, None] + [GearShifter.park] * 100)

  def test_the_kill_switch_does_not_hide_the_edge(self):
    """RecordWhileParked=1 keeps parked logging -- the drive's tail still needs archiving."""
    assert self._run([GearShifter.drive, GearShifter.park], gate_on=False).count(True) == 1

  def test_each_new_park_after_moving_is_a_new_edge(self):
    seq = [GearShifter.drive, GearShifter.park, GearShifter.park, GearShifter.reverse, GearShifter.park]
    assert self._run(seq) == [False, True, False, False, True]


class TestRotateAtPark:
  def test_below_the_minimum_nothing_rotates(self, tmp_path, arc):
    live = _live(tmp_path, text="x" * (m.CES_PARK_ROTATE_MIN_BYTES - 1))
    assert m.rotate_at_park(str(live), 8) is False
    assert live.exists() and not (tmp_path / f"{BASE}.1").exists()

  def test_at_the_minimum_it_rotates_and_archives(self, tmp_path, arc, said):
    live = _live(tmp_path, text="x" * m.CES_PARK_ROTATE_MIN_BYTES)
    assert m.rotate_at_park(str(live), 8) is True
    assert not live.exists() and (tmp_path / f"{BASE}.1").exists()
    assert [p.name for p in arc.iterdir()] == [GOOD_NAME]
    assert [n for n, _ in said["event"]] == ["ces_park_rotate"]

  def test_no_live_file_is_a_quiet_no(self, tmp_path, said):
    assert m.rotate_at_park(str(tmp_path / BASE), 8) is False
    assert not said["error"] and not said["exception"]

  def test_a_failing_rotation_is_loud_and_never_raises(self, tmp_path, arc, said, monkeypatch):
    live = _live(tmp_path, text="x" * m.CES_PARK_ROTATE_MIN_BYTES)

    def boom(*a):
      raise RuntimeError("synthetic")
    monkeypatch.setattr(m, "rotate_event_log", boom)
    assert m.rotate_at_park(str(live), 8) is False
    assert any("park rotation FAILED" in s for s in said["exception"])


def _decider(monkeypatch, tmp_path):
  """A CESController with __init__ bypassed, carrying only what _park_decision touches."""
  c = m.CESController.__new__(m.CESController)
  c._park_gate = ParkTickGate()
  c._park_gate_on = True
  c._park_err_t, c._park_err_n = None, 0
  c._gear, c._v_ego_raw = None, None
  monkeypatch.setattr(m, "CES_EVENT_LOG", str(tmp_path / BASE))
  return c


class TestTheControllerRotatesOncePerPark:
  def test_drive_then_a_long_park_rotates_exactly_once(self, tmp_path, arc, monkeypatch):
    c = _decider(monkeypatch, tmp_path)
    calls = []
    real = m.rotate_event_log
    monkeypatch.setattr(m, "rotate_event_log", lambda *a: (calls.append(a), real(*a)))
    _live(tmp_path, text="x" * (200 * 1024))
    for i in range(10):
      c._gear, c._v_ego_raw = GearShifter.drive, 20.0
      c._park_decision(float(i))
    for i in range(10, 1000):
      c._gear, c._v_ego_raw = GearShifter.park, 0.0
      c._park_decision(float(i))
    assert len(calls) == 1
    assert len(list(arc.iterdir())) == 1

  def test_the_gate_decision_is_unchanged_by_the_rotation(self, tmp_path, arc, monkeypatch):
    """The rotation is housekeeping bolted on after the decision; the record gate must not notice it."""
    c = _decider(monkeypatch, tmp_path)
    _live(tmp_path, text="x" * (200 * 1024))
    c._gear, c._v_ego_raw = GearShifter.drive, 20.0
    assert c._park_decision(0.0) == m.PARK_LOG
    c._gear, c._v_ego_raw = GearShifter.park, 0.0
    assert c._park_decision(1.0) == m.PARK_LOG          # inside the 30 s debounce: still logging
    assert (tmp_path / f"{BASE}.1").exists()
