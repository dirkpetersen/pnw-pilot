"""cesretain2pnw: multi-generation rotation for /data/pnw/ces_events.jsonl.

Field basis: the 2026-08-26 Olympic Peninsula trip (261 mi, two cars) had ALREADY rotated out of the
single `.1` generation by the time it was analysed -- `.1` began AFTER the driving ended, so the
whole trip had to be reconstructed from S3 qlogs instead of the per-second CES stream.
"""
import os

import pytest

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import (rotate_event_log,
                                                              CES_EVENT_LOG_GENERATIONS,
                                                              CES_EVENT_LOG_MAX_BYTES)


# Snapshotted at import, BEFORE the autouse fixture below redirects it -- the shipped value is
# itself under test (it must sit on /data, next to the log, or the move stops being a rename).
SHIPPED_ARCHIVE_DIR = m.CES_ARCHIVE_DIR


@pytest.fixture(autouse=True)
def archive_in_tmp(tmp_path, monkeypatch):
  """curvedbtel2pnw: rotation now MOVES the generation it used to destroy into CES_ARCHIVE_DIR.
  Point that at tmp_path so a dev-host test run never touches (or tries to create) /data/pnw."""
  monkeypatch.setattr(m, "CES_ARCHIVE_DIR", str(tmp_path / "archive"))


def _write(p, text):
  with open(p, "w") as f:
    f.write(text)


def _read(p):
  with open(p) as f:
    return f.read()


class TestRotateEventLog:
  def test_live_file_becomes_generation_1(self, tmp_path):
    log = str(tmp_path / "ces_events.jsonl")
    _write(log, "live")
    rotate_event_log(log, CES_EVENT_LOG_GENERATIONS)
    assert not os.path.exists(log)          # caller reopens in append mode
    assert _read(log + ".1") == "live"

  def test_generations_shift_down_by_one(self, tmp_path):
    log = str(tmp_path / "ces_events.jsonl")
    _write(log, "live")
    _write(log + ".1", "gen1")
    _write(log + ".2", "gen2")
    rotate_event_log(log, CES_EVENT_LOG_GENERATIONS)
    assert _read(log + ".1") == "live"
    assert _read(log + ".2") == "gen1"
    assert _read(log + ".3") == "gen2"

  def test_oldest_generation_is_dropped_at_the_cap(self, tmp_path):
    n = CES_EVENT_LOG_GENERATIONS
    log = str(tmp_path / "ces_events.jsonl")
    _write(log, "live")
    for i in range(1, n + 1):
      _write(f"{log}.{i}", f"gen{i}")
    rotate_event_log(log, n)
    # the oldest generation is out of the rotation; nothing is created past the cap
    assert _read(f"{log}.{n}") == f"gen{n - 1}"
    assert not os.path.exists(f"{log}.{n + 1}")
    # curvedbtel2pnw (section 3.8): it is ARCHIVED, not destroyed. 8 generations is ~17.6 driving
    # hours, and the curvedb Phase-1 gate needs 6-8 WEEKS retained.
    # cesarchive2pnw: and the generation that just LEFT live is hardlinked in at the same time.
    archived = sorted(q.read_text() for q in (tmp_path / "archive").iterdir())
    assert archived == sorted([f"gen{n}", "live"])

  def test_holes_in_the_chain_are_tolerated(self, tmp_path):
    # a crash mid-rotate can leave a missing generation; rotation must not raise
    log = str(tmp_path / "ces_events.jsonl")
    _write(log, "live")
    _write(log + ".3", "gen3")
    rotate_event_log(log, CES_EVENT_LOG_GENERATIONS)
    assert _read(log + ".1") == "live"
    assert _read(log + ".4") == "gen3"
    assert not os.path.exists(log + ".2")

  def test_repeated_rotations_walk_a_record_to_the_end_then_drop_it(self, tmp_path):
    n = CES_EVENT_LOG_GENERATIONS
    log = str(tmp_path / "ces_events.jsonl")
    _write(log, "oldest")
    for i in range(n):
      rotate_event_log(log, n)
      _write(log, f"fill{i}")
    assert _read(f"{log}.{n}") == "oldest"     # walked all the way to the last generation
    rotate_event_log(log, n)                   # one more push evicts it from the rotation
    survivors = [_read(f"{log}.{i}") for i in range(1, n + 1) if os.path.exists(f"{log}.{i}")]
    assert "oldest" not in survivors
    # ... and lands in the archive rather than being lost (curvedbtel2pnw section 3.8)
    assert "oldest" in [q.read_text() for q in (tmp_path / "archive").iterdir()]

  def test_single_generation_matches_the_old_behaviour(self, tmp_path):
    log = str(tmp_path / "ces_events.jsonl")
    _write(log, "live")
    _write(log + ".1", "gen1")
    rotate_event_log(log, 1)
    assert _read(log + ".1") == "live"        # old .1 overwritten, exactly as os.replace did
    assert not os.path.exists(log + ".2")

  def test_retention_window_covers_a_week_of_heavy_driving(self):
    # The Peninsula trip wrote ~21 MB in a day of mixed two-car driving. Only ROTATED generations
    # count as retained history -- the live file is still being appended to and will itself rotate.
    # This assert is what caught 7 generations being 8 MB short of the stated one-week goal.
    mb_per_heavy_day = 21
    retained = CES_EVENT_LOG_GENERATIONS * CES_EVENT_LOG_MAX_BYTES
    assert retained >= 7 * mb_per_heavy_day * 1024 * 1024, f"{retained} bytes is under a week"


# =====================================================================================================
# curvedbtel2pnw (CURVEDB2PNW.md section 3.8) -- THE ARCHIVE PATH.
#
# ces_events rotates at 20 MB x 8 generations = 160 MB, i.e. ~17.6 driving hours at the measured
# 9.1 MB/h. The curvedb Phase-1 exit criteria need 6-8 WEEKS of corridor driving RETAINED, and
# nothing on 3devpnw archives ces_events off-device -- so as shipped, Phase 1 would generate exactly
# the dataset its go/no-go decision needs and then delete it weeks before that decision.
#
# The generation the rotation would destroy is os.replace()d into CES_ARCHIVE_DIR first. A rename on
# the same filesystem: constant time, atomic, no duplicate bytes. That matters because
# rotate_event_log runs synchronously inside selfdrived's 100 Hz loop -- a 20 MB copy (~100-200 ms)
# or a gzip (seconds) would stall control for tens of frames.
# =====================================================================================================
def _gen(tmp_path, name, size=16, mtime=None):
  p = tmp_path / name
  p.write_text("x" * size)
  if mtime is not None:
    os.utime(p, (mtime, mtime))
  return p


class TestArchive:
  def test_the_generation_the_rotation_would_destroy_is_archived(self, tmp_path):
    """M6. rotate_event_log shifts .7 -> .8, which OVERWRITES .8 -- at 8 generations that file is
    only ~17.6 driving hours old, while section 3.9 needs 6-8 WEEKS retained."""
    live = tmp_path / "ces_events.jsonl"
    live.write_text("live\n")
    for i in range(1, 9):
      _gen(tmp_path, f"ces_events.jsonl.{i}", size=10 + i)
    arc = tmp_path / "arc"
    doomed = (tmp_path / "ces_events.jsonl.8").read_text()

    m.archive_rotated_generation(str(live), 8, str(arc), max_bytes=10 ** 9)

    kept = list(arc.iterdir())
    assert len(kept) == 1, "the oldest generation must be moved out before the shift overwrites it"
    assert kept[0].read_text() == doomed
    assert not (tmp_path / "ces_events.jsonl.8").exists(), "a MOVE, not a copy (no duplicate bytes)"

  def test_rotation_calls_it_and_the_oldest_content_survives_end_to_end(self, tmp_path, monkeypatch):
    live = tmp_path / "ces_events.jsonl"
    live.write_text("live\n")
    for i in range(1, 9):
      (tmp_path / f"ces_events.jsonl.{i}").write_text(f"gen{i}\n")
    arc = tmp_path / "arc"
    monkeypatch.setattr(m, "CES_ARCHIVE_DIR", str(arc))

    m.rotate_event_log(str(live), 8)

    # gen8 moved in at eviction (it was never linked); "live" hardlinked in at rotation (cesarchive2pnw)
    assert sorted(p.read_text() for p in arc.iterdir()) == ["gen8\n", "live\n"]
    assert (tmp_path / "ces_events.jsonl.1").read_text() == "live\n"    # the rotation still works
    assert (tmp_path / "ces_events.jsonl.8").read_text() == "gen7\n"

  def test_nothing_to_archive_before_the_first_full_cycle(self, tmp_path):
    live = tmp_path / "ces_events.jsonl"
    live.write_text("live\n")
    arc = tmp_path / "arc"
    assert m.archive_rotated_generation(str(live), 8, str(arc)) is None

  def test_the_budget_evicts_oldest_first(self, tmp_path):
    arc = tmp_path / "arc"
    arc.mkdir()
    for i, mt in enumerate((1000.0, 2000.0, 3000.0)):
      _gen(arc, f"g{i}", size=100, mtime=mt)
    removed = m.prune_ces_archive(str(arc), max_bytes=250)
    assert removed == 1
    assert sorted(p.name for p in arc.iterdir()) == ["g1", "g2"], "the NEWEST data must survive"

  def test_under_budget_deletes_nothing(self, tmp_path):
    arc = tmp_path / "arc"
    arc.mkdir()
    _gen(arc, "g0", size=100)
    assert m.prune_ces_archive(str(arc), max_bytes=10 ** 6) == 0
    assert len(list(arc.iterdir())) == 1

  def test_an_eviction_is_loud(self, tmp_path, monkeypatch):
    """Rule 2: losing the oldest data silently is the one outcome section 3.8 exists to prevent."""
    said = []
    monkeypatch.setattr(m.cloudlog, "error", lambda msg, *a, **k: said.append(msg))
    arc = tmp_path / "arc"
    arc.mkdir()
    for i, mt in enumerate((1000.0, 2000.0)):
      _gen(arc, f"g{i}", size=100, mtime=mt)
    m.prune_ces_archive(str(arc), max_bytes=100)
    assert any("OVER BUDGET" in s and "TRUNCATED" in s for s in said)

  def test_a_failed_archive_is_loud_and_never_breaks_the_rotation(self, tmp_path, monkeypatch):
    said = []
    monkeypatch.setattr(m.cloudlog, "error", lambda msg, *a, **k: said.append(msg))
    live = tmp_path / "ces_events.jsonl"
    live.write_text("live\n")
    for i in range(1, 9):
      (tmp_path / f"ces_events.jsonl.{i}").write_text(f"gen{i}\n")
    arc = tmp_path / "arc"
    monkeypatch.setattr(m, "CES_ARCHIVE_DIR", str(arc))
    monkeypatch.setattr(m.os, "replace", _only_fail_into(str(arc)))

    m.rotate_event_log(str(live), 8)                    # must not raise

    assert any("ces_archive move FAILED" in s and "DESTROYED" in s for s in said)
    assert (tmp_path / "ces_events.jsonl.1").read_text() == "live\n", "the rotation itself still ran"

  def test_a_dead_clock_generation_cannot_REUSE_a_name_after_the_first_was_evicted(self, tmp_path):
    """ceslogup2pnw (Fable 2026-09-16). The 3X's RTC battery is dead, so a pre-NTP rotation stamps
    1970. The `.N` collision loop only disambiguates while the earlier file still EXISTS -- and
    prune_ces_archive sorts by mtime, so 1970 files are the first evicted, which frees the name.

    Once these generations go to S3 that is silent data loss twice over: the gateway presigns a
    plain put_object (no 412), so the earlier object is overwritten; and xattr_cache memoises
    "uploaded" against the PATH, so a reused name inherits the previous file's mark and the new
    file is never sent at all. The name must therefore be unique on its own, not by coincidence."""
    live = tmp_path / "ces_events.jsonl"
    live.write_text("live\n")
    arc = tmp_path / "arc"
    seen = set()
    for i in range(6):
      _gen(tmp_path, "ces_events.jsonl.8", size=10 + i, mtime=60.0)   # 1970-01-01T00:01:00Z
      dest = m.archive_rotated_generation(str(live), 8, str(arc), max_bytes=10 ** 9)
      assert dest is not None
      seen.add(os.path.basename(dest))
      os.unlink(dest)             # what prune_ces_archive does to the oldest file -- 1970 sorts first
    assert len(seen) == 6, f"a dead-clock name was reused after eviction: {sorted(seen)}"
    for n in seen:
      assert n.startswith("ces_events.jsonl."), f"{n} no longer passes the uploader's prefix gate"
      assert n.startswith("ces_events.jsonl.19700101T000100Z."), n

  def test_a_good_clock_still_gets_the_plain_mtime_name(self, tmp_path):
    """The negative control: the random token is for the dead-clock case ONLY. A normal name is the
    sortable mtime stamp, which is what makes the uploader's lexical oldest-first walk chronological."""
    live = tmp_path / "ces_events.jsonl"
    live.write_text("live\n")
    _gen(tmp_path, "ces_events.jsonl.8", mtime=1789000000.0)      # 2026-09-10T00:26:40Z
    dest = m.archive_rotated_generation(str(live), 8, str(tmp_path / "arc"), max_bytes=10 ** 9)
    assert os.path.basename(dest) == "ces_events.jsonl.20260910T002640Z", os.path.basename(dest)

  def test_the_archive_directory_is_on_data_so_the_move_is_a_rename(self):
    """Not decoration: a cross-filesystem destination would make os.replace raise EXDEV, and the
    fallback would then have to be a 20 MB copy inside selfdrived's 100 Hz loop."""
    assert SHIPPED_ARCHIVE_DIR.startswith(os.path.dirname(m.CES_EVENT_LOG) + os.sep)

  def test_the_budget_covers_the_gate_window(self):
    """Section 3.9 item 3 wants 6-8 weeks retained; heavy driving measures ~21 MB/day."""
    days = m.CES_ARCHIVE_MAX_BYTES / (21 * 1024 * 1024)
    assert days >= 56, f"only {days:.0f} days of headroom -- section 3.9 needs at least 56"


def _only_fail_into(directory):
  """os.replace that fails ONLY for a destination inside `directory` -- i.e. the archive move, never
  the rotation's own shifts. Matched on the parent dir, not a substring: pytest's tmp_path is named
  after the test, so a substring match would also hit the rotation and prove the wrong thing."""
  real = os.replace

  def fake(src, dst):
    if os.path.dirname(str(dst)) == directory:
      raise OSError(18, "Invalid cross-device link")
    return real(src, dst)
  return fake
