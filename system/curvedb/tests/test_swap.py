"""swap.py: the safety rules ported from the ssh gate. Failure paths first."""
import importlib.machinery
import importlib.util
import itertools
import os
import pathlib
import subprocess

import pytest

from openpilot.system.curvedb import swap
from openpilot.system.curvedb.tests.conftest import table

GATE = os.path.expanduser("~/gh/comma/workdir/curvedb/device/curvedb-swap-gate")
CASES = [(g, o, et) for g in ("0", "1", "?") for o in ("0", "1", "?") for et in ("", "?", "0", "120", "120.5", "500")]
GOLDEN = {   # the rule, spelled out: Rule 3 -> gear, not IsOnroad
  ("0", "0", ""): True, ("?", "0", "?"): True,                # offroad is always fine
  ("0", "1", "500"): False,                                   # driving
  ("1", "1", "120"): False, ("1", "1", "0"): False,           # Park but selfdrived not settled (> 120 s needed)
  ("1", "1", "121"): True, ("1", "1", "500"): True,
  ("1", "1", ""): False, ("1", "1", "?"): False,              # uptime unknown
  ("?", "1", "500"): False, ("1", "?", "500"): False,         # unreadable state
}


@pytest.mark.parametrize("args,ok", GOLDEN.items())
def test_decide_golden(args, ok):
  assert swap.decide(*args)[0] is ok


def test_decide_parity_with_the_ssh_gate():
  if not os.path.exists(GATE):
    pytest.skip(f"private gate not present at {GATE}; the golden table above still pins the rule")
  loader = importlib.machinery.SourceFileLoader("gate", GATE)
  gate = importlib.util.module_from_spec(importlib.util.spec_from_loader("gate", loader))
  loader.exec_module(gate)
  assert len(CASES) == 54
  for c in CASES:
    assert swap.decide(*c) == gate.decide(*c), c
  assert (swap.MAX_ROWS, swap.MAX_MAN, swap.MIN_FREE_MB, swap.MIN_UPTIME_S, swap.KEEP_AUTO, swap.LOADER_AS_BYTES) == \
         (gate.MAX_ROWS, gate.MAX_MAN, gate.MIN_FREE_MB, gate.MIN_UPTIME_S, gate.KEEP_AUTO, gate.LOADER_AS_BYTES)
  assert swap.LOADER_PY == gate.LOADER_PY


def test_lock_file_is_the_gates():
  assert swap.LOCK_FILE == "/data/pnw/curvedb-swap-gate.log.lock"   # the module default, unpatched
  if os.path.exists(GATE):
    src = pathlib.Path(GATE).read_text()
    assert 'LOGF + ".lock"' in src and 'LOGF = ROOT + "/data/pnw/curvedb-swap-gate.log"' in src


def test_lock_excludes_a_second_holder(env):
  h = swap.lock()
  with pytest.raises(swap.Failed, match="holds the lock"):
    swap.lock(wait_s=0.3)
  h.close()
  swap.lock(wait_s=0.3).close()


def test_stage_refuses_wrong_sha_oversize_and_bad_manifest(env):
  sha, rows, man = table("x")
  with pytest.raises(swap.Failed, match="sha mismatch"):
    swap.stage(env.new_sha, rows, man)
  with pytest.raises(swap.Failed, match="size out of range"):
    swap.stage(sha, b"a" * (swap.MAX_ROWS + 1), man)
  with pytest.raises(swap.Failed, match="size out of range"):
    swap.stage(sha, rows, b"{" * (swap.MAX_MAN + 1))
  with pytest.raises(swap.Failed, match="bad manifest"):
    swap.stage(sha, rows, b'{"sha256": "00"}')
  assert not os.path.exists(swap.NEW)


def test_stage_and_swap_refuse_unsafe_states(env):
  for state, free, why in [(("0", "1", "500"), 2000, "driving"), (("1", "1", "60"), 2000, "selfdrived up only"),
                           (("1", "1", "500"), 100, "MB free")]:
    env.state, env.free = state, free
    with pytest.raises(swap.Refused, match=why):
      swap.stage(env.new_sha, env.new_rows, env.new_man)
  assert not os.path.exists(swap.NEW) and env.live_sha() == env.old_sha


def test_swap_rechecks_right_before_the_rename(env, monkeypatch):
  swap.stage(env.new_sha, env.new_rows, env.new_man)
  seq = itertools.chain([("1", "1", "500")], itertools.repeat(("0", "1", "500")))   # ok at the first check, driving at the re-check
  monkeypatch.setattr(swap, "now_state", lambda: next(seq))
  with pytest.raises(swap.Refused, match="state changed during the swap"):
    swap.swap(env.new_sha)
  assert env.live_sha() == env.old_sha          # the live table was never touched


def test_swap_installs_with_backup_and_rotates_only_auto_backups(env):
  for d in ["20260101-auto", "20260102-auto", "20260103-auto", "20260104-auto", "20260105-auto", "20260106-auto", "20260107-autob"]:
    os.makedirs(f"{env.live}.bak-{d}")
  for keep in ["20260101-manual", "pre-gate", "20260101"]:
    os.makedirs(f"{env.live}.bak-{keep}")
  swap.stage(env.new_sha, env.new_rows, env.new_man)
  bak = swap.swap(env.new_sha)
  assert env.live_sha() == env.new_sha and bak.startswith("curvedb_v2.bak-") and bak.endswith("-auto")
  assert pathlib.Path(f"{env.live}.bak-{bak.split('bak-')[1]}/{swap.ROWS}").read_bytes() == b"old" * 50
  names = os.listdir(os.path.dirname(env.live))
  assert sum(n.endswith(("-auto", "-autob")) or "-auto" in n for n in names) == swap.KEEP_AUTO
  assert all(k in names for k in ["curvedb_v2.bak-20260101-manual", "curvedb_v2.bak-pre-gate", "curvedb_v2.bak-20260101"])
  assert not os.path.exists(swap.NEW) and not os.path.exists(swap.REPL)


def test_second_swap_same_day_picks_a_new_backup_name(env):
  swap.stage(env.new_sha, env.new_rows, env.new_man)
  a = swap.swap(env.new_sha)
  sha2, rows2, man2 = table("two")
  swap.stage(sha2, rows2, man2)
  assert swap.swap(sha2) != a


def test_loader_runs_in_a_limited_child(env):
  swap.stage(env.new_sha, env.new_rows, env.new_man)
  v = swap.staged_check(env.new_sha)
  assert v["cwd"] == "/" and v["nice"] == 19 and v["as"] == swap.LOADER_AS_BYTES and v["cpu"] == 120 and v["blas"] == "1"


def test_loader_failure_blocks_the_swap(env, tmp_path, monkeypatch):
  bad = tmp_path / "bad.py"
  bad.write_text("import sys; sys.exit('corrupt table')")
  monkeypatch.setattr(swap, "LOADER_SCRIPT", str(bad))
  swap.stage(env.new_sha, env.new_rows, env.new_man)
  with pytest.raises(swap.Failed, match="refused the table"):
    swap.swap(env.new_sha)
  assert env.live_sha() == env.old_sha


def test_recover_from_missing_live_dir(env):
  swap.stage(env.new_sha, env.new_rows, env.new_man)
  swap.swap(env.new_sha)
  subprocess.run(["rm", "-rf", env.live], check=True)
  assert swap.recover().endswith("-auto")
  assert env.live_sha() == env.old_sha
  with pytest.raises(swap.Failed, match="nothing to recover"):
    swap.recover()


def test_verify_reads_swaglog_after_the_install(env):
  swap.stage(env.new_sha, env.new_rows, env.new_man)
  swap.swap(env.new_sha)
  assert swap.verify(env.new_sha) == ("pending", "")
  line = '{"event$s": "curvedb_v2_loaded", "x": 1, "sha256$s": "%s"}\n'
  pathlib.Path(f"{swap.SWAGLOG}/swaglog.1").write_text(line % env.new_sha)
  assert swap.verify(env.new_sha)[0] == "loaded"
  pathlib.Path(f"{swap.SWAGLOG}/swaglog.1").write_text(line % env.old_sha)
  assert swap.verify(env.new_sha) == ("mismatch", env.old_sha[:8])
