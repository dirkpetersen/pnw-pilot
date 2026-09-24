"""cesarchive2pnw: the metered exception -- small device logs only, within 50 MB per Pacific day.

On a metered link no drive file moves (qlog, qcamera, rlog, video: the 2026-09-10 incident sent
2.6 GB of video over a metered Starlink). The ONE exception: ces_events, curvedb_obs, boot logs and
net_events, in that priority, until the bytes sent while metered reach PNW_LOG_METERED_DAILY_BYTES
for the Pacific day. The counter survives restarts, fails closed, and needs a valid clock.
"""
import datetime
import json
import os
import zoneinfo

import pytest

from openpilot.system.loggerd import uploader
from openpilot.system.loggerd.uploader import PNW_LOG_METERED_DAILY_BYTES as CAP, Uploader

PT = zoneinfo.ZoneInfo("America/Los_Angeles")
# 2026-09-23 03:00 UTC = 2026-09-22 20:00 PDT: the UTC and Pacific dates DIFFER here on purpose.
NOW = datetime.datetime(2026, 9, 23, 3, 0, tzinfo=datetime.UTC).timestamp()
TODAY_PT = "2026-09-22"
SEG = "00000004--0ac3964c96--7"
DRIVE_FILES = ("qlog", "qcamera.ts", "rlog", "fcamera.hevc", "ecamera.hevc", "dcamera.hevc")


class _Params:
  def get(self, k):
    return None

  def get_bool(self, k):
    return False

  def put(self, k, v):
    pass

  def put_bool(self, k, v):
    pass


@pytest.fixture
def env(tmp_path, monkeypatch):
  """Three archive dirs + a log root under tmp_path, an in-memory xattr store, a fixed clock, and a
  fake PUT that records what was sent."""
  arc = {w: tmp_path / d for w, d in (("ces_events.jsonl.", "ces_archive"), ("curvedb_obs.jsonl.", "curvedb_archive"),
                                      ("net_events.jsonl.", "net_archive"))}
  for d in arc.values():
    d.mkdir()
  monkeypatch.setattr(uploader, "PNW_LOG_SOURCES", tuple((str(d), "pnwlogs", w) for w, d in arc.items()))
  monkeypatch.setattr(uploader, "METERED_BUDGET_PATH", str(tmp_path / "metered_budget.json"))
  monkeypatch.setattr(uploader.time, "time", lambda: NOW)
  marks = {}
  monkeypatch.setattr(uploader, "getxattr", lambda fn, a: marks.get(fn))
  monkeypatch.setattr(uploader, "setxattr", lambda fn, a, v: marks.__setitem__(fn, v))
  events, errors = [], []
  monkeypatch.setattr(uploader.cloudlog, "event", lambda n, **kw: events.append((n, kw)))
  monkeypatch.setattr(uploader.cloudlog, "error", lambda m, *a, **k: errors.append(m))
  root = tmp_path / "realdata"
  root.mkdir()
  return {"tmp": tmp_path, "arc": arc, "root": root, "marks": marks, "events": events, "errors": errors}


def _uploader(env, sent=None, status=200):
  u = Uploader.__new__(Uploader)
  u.root = str(env["root"])
  u.params = _Params()
  u.immediate_priority = {"qlog": 0, "qlog.zst": 0, "qcamera.ts": 1}
  u.immediate_folders = ["crash/", "boot/"]
  u._retry_after = {}
  u._defer_hd = u._skip_wide = False
  u._pnw_log_state = {}
  u._metered_full_day = None
  u._412_streak_start = None
  u._upload_err_desc = None
  u.last_filename, u.last_upload_mbps = "", 0.0
  sent = [] if sent is None else sent

  def fake_do_upload(key, fn):
    sent.append(key)
    r = uploader.FakeResponse()
    r.status_code = status
    return r
  u.do_upload = fake_do_upload
  u.sent = sent
  return u


def _put(d, name, nbytes, mtime=None):
  p = d / name
  p.write_bytes(os.urandom(nbytes))            # incompressible: the wire size is ~the file size
  if mtime is not None:
    os.utime(p, (mtime, mtime))
  return p


def _drain(u, limit=50):
  for _ in range(limit):
    if u.step_metered(1) is None:
      return
  raise AssertionError("step_metered never went idle")


def _budget(env):
  with open(env["tmp"] / "metered_budget.json") as f:
    return json.load(f)


def _wire(p, key):
  return Uploader._wire_size(key, str(p))


# ---------------------------------------------------------------------------------------------------
class TestTheCap:
  def test_uploads_stop_at_the_cap_and_the_counter_is_the_bytes_sent(self, env):
    ces = env["arc"]["ces_events.jsonl."]
    a = _put(ces, "ces_events.jsonl.20260901T000000Z", CAP // 3)
    b = _put(ces, "ces_events.jsonl.20260902T000000Z", CAP // 3)
    _put(ces, "ces_events.jsonl.20260903T000000Z", CAP // 2)        # does not fit after a + b
    u = _uploader(env)
    _drain(u)
    assert u.sent == ["pnwlogs/ces_events.jsonl.20260901T000000Z.zst", "pnwlogs/ces_events.jsonl.20260902T000000Z.zst"]
    want = _wire(a, "x.zst") + _wire(b, "x.zst")
    assert _budget(env) == {"date": TODAY_PT, "bytes": want}, "the .zst bytes actually sent, nothing else"
    notes = [kw for n, kw in env["events"] if kw.get("state") == "metered_budget_reached"]
    assert len(notes) == 1 and "metered telemetry budget reached" in notes[0]["note"]

  def test_a_file_that_WOULD_exceed_is_not_started(self, env):
    p = _put(env["arc"]["ces_events.jsonl."], "ces_events.jsonl.20260901T000000Z", 1000)
    size = _wire(p, "x.zst")
    (env["tmp"] / "metered_budget.json").write_text(json.dumps({"date": TODAY_PT, "bytes": CAP - size + 1}))
    u = _uploader(env)
    assert u.step_metered(1) is None
    assert u.sent == [], "one byte over the cap must not be sent"
    (env["tmp"] / "metered_budget.json").write_text(json.dumps({"date": TODAY_PT, "bytes": CAP - size}))
    u = _uploader(env)
    assert u.step_metered(1) is True, "exactly at the cap is allowed"
    assert _budget(env)["bytes"] == CAP

  def test_once_reached_the_day_is_not_re_measured_every_loop(self, env, monkeypatch):
    _put(env["arc"]["ces_events.jsonl."], "ces_events.jsonl.20260901T000000Z", 1000)
    (env["tmp"] / "metered_budget.json").write_text(json.dumps({"date": TODAY_PT, "bytes": CAP}))
    u = _uploader(env)
    calls = []
    real = Uploader._wire_size
    monkeypatch.setattr(Uploader, "_wire_size", staticmethod(lambda k, f: (calls.append(k), real(k, f))[1]))
    for _ in range(20):
      assert u.step_metered(1) is None
    assert len(calls) == 1, "compressing the same file every loop to learn it still does not fit"


class TestTheDay:
  def test_the_pacific_day_rollover_resets_the_budget(self, env):
    _put(env["arc"]["ces_events.jsonl."], "ces_events.jsonl.20260901T000000Z", 1000)
    (env["tmp"] / "metered_budget.json").write_text(json.dumps({"date": "2026-09-21", "bytes": CAP}))
    u = _uploader(env)
    assert u.step_metered(1) is True
    assert _budget(env)["date"] == TODAY_PT

  def test_the_day_is_PACIFIC_not_UTC(self, env):
    """At NOW the UTC date is already 2026-09-23. A budget exhausted on PT 2026-09-22 must hold."""
    _put(env["arc"]["ces_events.jsonl."], "ces_events.jsonl.20260901T000000Z", 1000)
    (env["tmp"] / "metered_budget.json").write_text(json.dumps({"date": TODAY_PT, "bytes": CAP}))
    u = _uploader(env)
    assert u.step_metered(1) is None and u.sent == []

  def test_a_persisted_budget_survives_a_new_uploader(self, env):
    ces = env["arc"]["ces_events.jsonl."]
    p = _put(ces, "ces_events.jsonl.20260901T000000Z", CAP // 2)
    _put(ces, "ces_events.jsonl.20260902T000000Z", CAP // 2 + 4096)
    u1 = _uploader(env)
    assert u1.step_metered(1) is True
    u2 = _uploader(env)                         # a restart / reboot: fresh process state
    assert u2.step_metered(1) is None and u2.sent == []
    assert _budget(env)["bytes"] == _wire(p, "x.zst")

  def test_an_invalid_clock_blocks_metered_uploads_and_says_so(self, env, monkeypatch):
    _put(env["arc"]["ces_events.jsonl."], "ces_events.jsonl.20260901T000000Z", 1000)
    monkeypatch.setattr(uploader.time, "time", lambda: 60.0)          # dead-RTC boot: 1970
    u = _uploader(env)
    assert u.step_metered(1) is None and u.sent == []
    assert any(kw.get("state") == "metered_clock_invalid" for _, kw in env["events"])
    assert not (env["tmp"] / "metered_budget.json").exists(), "an untrusted day must not be written"


class TestFailClosed:
  @pytest.mark.parametrize("content", ["{not json", '{"date": "2026-09-22"}', '{"date": 5, "bytes": 1}',
                                       '{"date": "2026-09-22", "bytes": -1}', '{"date": "2026-09-22", "bytes": true}', "[]"])
  def test_a_corrupt_state_file_blocks_and_is_loud(self, env, content):
    _put(env["arc"]["ces_events.jsonl."], "ces_events.jsonl.20260901T000000Z", 1000)
    (env["tmp"] / "metered_budget.json").write_text(content)
    u = _uploader(env)
    assert u.step_metered(1) is None and u.sent == []
    assert any("metered_corrupt" in e for e in env["errors"]), env["errors"]
    assert _budget(env) == {"date": TODAY_PT, "bytes": CAP}, "rewritten as exhausted, so it heals tomorrow"

  def test_an_unwritable_counter_means_no_upload(self, env, monkeypatch):
    _put(env["arc"]["ces_events.jsonl."], "ces_events.jsonl.20260901T000000Z", 1000)
    monkeypatch.setattr(uploader, "METERED_BUDGET_PATH", str(env["tmp"] / "no_such_dir" / "b.json"))
    u = _uploader(env)
    assert u.step_metered(1) is None and u.sent == [], "never send bytes the counter does not hold"
    assert any("metered_write_failed" in e for e in env["errors"])

  def test_a_failed_upload_is_refunded(self, env):
    _put(env["arc"]["ces_events.jsonl."], "ces_events.jsonl.20260901T000000Z", 1000)
    u = _uploader(env, status=500)
    assert u.step_metered(1) is False
    assert _budget(env)["bytes"] == 0, "only successful uploads count"


class TestWhatMayGoOnMetered:
  def _everything(self, env):
    _put(env["arc"]["net_events.jsonl."], "net_events.jsonl.20260901T000000Z", 100)
    _put(env["arc"]["curvedb_obs.jsonl."], "curvedb_obs.jsonl.20260901T000000Z", 100)
    _put(env["arc"]["ces_events.jsonl."], "ces_events.jsonl.20260902T000000Z", 100)
    _put(env["arc"]["ces_events.jsonl."], "ces_events.jsonl.20260901T000000Z", 100)
    (env["root"] / "boot").mkdir()
    _put(env["root"] / "boot", "0000001a--1234567890", 100, mtime=NOW - 100)
    _put(env["root"] / "boot", "00000019--0987654321", 100, mtime=NOW - 200)
    seg = env["root"] / SEG
    seg.mkdir()
    for n in DRIVE_FILES:
      _put(seg, n, 100)

  def test_priority_order_ces_curvedb_boot_net_and_oldest_first(self, env):
    self._everything(env)
    u = _uploader(env)
    _drain(u)
    assert u.sent == [
      "pnwlogs/ces_events.jsonl.20260901T000000Z.zst",
      "pnwlogs/ces_events.jsonl.20260902T000000Z.zst",
      "pnwlogs/curvedb_obs.jsonl.20260901T000000Z.zst",
      "boot/00000019--0987654321.zst",
      "boot/0000001a--1234567890.zst",
      "pnwlogs/net_events.jsonl.20260901T000000Z.zst",
    ]

  def test_drive_files_are_NEVER_sent_on_metered(self, env):
    """The regression guard: qlog, qcamera, rlog and all video stay blocked on metered."""
    self._everything(env)
    u = _uploader(env)
    _drain(u)
    leaked = [k for k in u.sent if k.startswith(SEG)]
    assert leaked == [], f"drive files went out on a metered link: {leaked}"
    assert u.step_metered(1) is None, "with only drive files left, metered must go idle"

  def test_a_short_budget_is_spent_on_the_higher_priority_source(self, env):
    ces = env["arc"]["ces_events.jsonl."]
    p = _put(ces, "ces_events.jsonl.20260901T000000Z", 2000)
    _put(env["arc"]["net_events.jsonl."], "net_events.jsonl.20260801T000000Z", 10)   # older, but lower tier
    (env["tmp"] / "metered_budget.json").write_text(json.dumps({"date": TODAY_PT, "bytes": CAP - _wire(p, "x.zst")}))
    u = _uploader(env)
    _drain(u)
    assert u.sent == ["pnwlogs/ces_events.jsonl.20260901T000000Z.zst"]

  def test_the_allowlist_gates_hold_for_every_new_source(self, env):
    for want, d in env["arc"].items():
      (d / "police_proxy.json").write_text('{"key": "SECRET"}')
      os.symlink(env["tmp"] / "secret", d / f"{want}link")
      (d / f"{want}adir").mkdir()
    (env["tmp"] / "secret").write_text("SECRET")
    u = _uploader(env)
    _drain(u)
    assert u.sent == []

  def test_unmetered_is_unaffected(self, env):
    """Unmetered: the normal listing takes all three sources, and the budget is never touched."""
    self._everything(env)
    (env["tmp"] / "metered_budget.json").write_text(json.dumps({"date": TODAY_PT, "bytes": CAP}))
    u = _uploader(env)
    keys = [k for _, k, _ in u.list_upload_files(metered=False)]
    for w in env["arc"]:
      assert any(k.startswith(f"pnwlogs/{w}") for k in keys), w
    assert u.step(1, False) is True
    assert _budget(env)["bytes"] == CAP, "unmetered uploads must not count against the budget"

  def test_the_normal_listing_still_offers_no_pnw_log_on_metered(self, env):
    self._everything(env)
    u = _uploader(env)
    assert not [k for _, k, _ in u.list_upload_files(metered=True) if k.startswith("pnwlogs/")]


def test_the_shipped_metered_order_and_sources():
  assert uploader.METERED_ORDER == ("ces_events.jsonl.", "curvedb_obs.jsonl.", "boot/", "net_events.jsonl.")
  wants = {w for _, _, w in uploader.PNW_LOG_SOURCES}
  assert {t for t in uploader.METERED_ORDER if t != "boot/"} == wants, "every metered tier must be a real source"
  assert CAP == 50 * 1024 * 1024


# ===================================================================================================
# Scope 3 (owner, 2026-09-23): on UNMETERED links every log precedes any video, smallest first.
#   1 small device logs (boot, then pnwlogs)  2 qlog  3 qcamera  4 rlog  5 HD video (dcamera: never)
# Each tier drains across ALL segments before the next starts.
# ===================================================================================================
WIFI = next(iter(uploader.PASS2_NETWORK_TYPES))
SEG2 = "00000004--0ac3964c96--8"


def _drain_unmetered(u, limit=100):
  """main()'s decision, verbatim: pass 1 every loop; pass 2 only once pass 1 has nothing left."""
  for _ in range(limit):
    if u.step(WIFI, False) is None and u.step(WIFI, False, pass2=True) is None:
      return
  raise AssertionError("never went idle")


def _backlog(env):
  _put(env["arc"]["net_events.jsonl."], "net_events.jsonl.20260901T000000Z", 100)
  _put(env["arc"]["curvedb_obs.jsonl."], "curvedb_obs.jsonl.20260901T000000Z", 100)
  _put(env["arc"]["ces_events.jsonl."], "ces_events.jsonl.20260901T000000Z", 100)
  (env["root"] / "boot").mkdir()
  _put(env["root"] / "boot", "00000019--0987654321", 100)
  for seg in (SEG, SEG2):
    (env["root"] / seg).mkdir()
    for n in DRIVE_FILES:
      _put(env["root"] / seg, n, 100)


class TestUnmeteredTierOrder:
  def test_small_logs_then_qlog_then_qcamera_then_rlog_then_hd(self, env):
    _backlog(env)
    u = _uploader(env)
    _drain_unmetered(u)
    assert u.sent[:10] == [
      "boot/00000019--0987654321.zst",
      "pnwlogs/ces_events.jsonl.20260901T000000Z.zst",
      "pnwlogs/curvedb_obs.jsonl.20260901T000000Z.zst",
      "pnwlogs/net_events.jsonl.20260901T000000Z.zst",
      f"{SEG}/qlog.zst", f"{SEG2}/qlog.zst",
      f"{SEG}/qcamera.ts", f"{SEG2}/qcamera.ts",
      f"{SEG}/rlog.zst", f"{SEG2}/rlog.zst",
    ], u.sent
    # HD last, oldest segment first; fcamera/ecamera order inside a segment is os.listdir order.
    hd = u.sent[10:]
    assert sorted(hd[:2]) == [f"{SEG}/ecamera.hevc", f"{SEG}/fcamera.hevc"], hd
    assert sorted(hd[2:]) == [f"{SEG2}/ecamera.hevc", f"{SEG2}/fcamera.hevc"], hd
    assert not [k for k in u.sent if k.endswith("dcamera.hevc")], "the driver camera never leaves"

  def test_a_cooling_down_rlog_does_not_block_video_forever(self, env, monkeypatch):
    seg = env["root"] / SEG
    seg.mkdir()
    for n in ("rlog", "fcamera.hevc"):
      _put(seg, n, 100)
    u = _uploader(env)
    monkeypatch.setattr(uploader.time, "monotonic", lambda: 1000.0)
    u._retry_after = {str(seg / "rlog"): 1000.0 + uploader.RETRY_COOLDOWN_S}
    assert u.next_pass2_file_to_upload(False)[1] == f"{SEG}/fcamera.hevc"
    u._retry_after = {}                                  # the cooldown lapses -> rlog goes first again
    assert u.next_pass2_file_to_upload(False)[1] == f"{SEG}/rlog"

  def test_a_cooling_down_qlog_does_not_hold_back_pass_2(self, env, monkeypatch):
    seg = env["root"] / SEG
    seg.mkdir()
    _put(seg, "qlog", 100)
    u = _uploader(env)
    monkeypatch.setattr(uploader.time, "monotonic", lambda: 1000.0)
    u._retry_after = {str(seg / "qlog"): 1000.0 + uploader.RETRY_COOLDOWN_S}
    assert u.next_file_to_upload(False) is None, "pass 1 must read as drained, so main() starts pass 2"

  def test_the_interleave_is_gone(self):
    assert not hasattr(uploader, "PASS2_INTERLEAVE")
