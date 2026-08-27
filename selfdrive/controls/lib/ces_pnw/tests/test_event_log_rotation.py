"""cesretain2pnw: multi-generation rotation for /data/pnw/ces_events.jsonl.

Field basis: the 2026-08-26 Olympic Peninsula trip (261 mi, two cars) had ALREADY rotated out of the
single `.1` generation by the time it was analysed -- `.1` began AFTER the driving ended, so the
whole trip had to be reconstructed from S3 qlogs instead of the per-second CES stream.
"""
import os

from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import (rotate_event_log,
                                                              CES_EVENT_LOG_GENERATIONS,
                                                              CES_EVENT_LOG_MAX_BYTES)


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
    # the oldest generation is gone; nothing is created past the cap
    assert _read(f"{log}.{n}") == f"gen{n - 1}"
    assert not os.path.exists(f"{log}.{n + 1}")

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
    rotate_event_log(log, n)                   # one more push evicts it
    survivors = [_read(f"{log}.{i}") for i in range(1, n + 1) if os.path.exists(f"{log}.{i}")]
    assert "oldest" not in survivors

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
