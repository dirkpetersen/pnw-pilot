"""curvedb_updated: one cycle per test. Every refusal path has its own distinct status; the safety gates are pinned."""
import ast
import json
import os
import pathlib
import re

import pytest
import requests

from openpilot.system.curvedb import curvedb_updated as cu
from openpilot.system.curvedb import status as st
from openpilot.system.curvedb import swap
from openpilot.system.curvedb.tests.conftest import Resp

from openpilot.system.hardware.base import NetworkType


def text(env):
  return st.ui_text(False, env.status(), 0).split(", checked")[0]


# ---- gates before any network traffic ----------------------------------------------------------------------------
def test_metered_network_makes_no_request(env):
  env.net = (int(NetworkType.cell2G), True)
  assert env.u.cycle() == 300.0
  assert env.http.calls == [] and text(env) == "waiting for unmetered network"


def test_no_network_and_no_deviceState_make_no_request(env):
  env.net = (int(NetworkType.none), False)
  env.u.cycle()
  assert env.http.calls == [] and text(env) == "waiting for a network"
  env.u.net_fn = lambda: None
  env.u.cycle()
  assert env.http.calls == []


def test_invalid_clock_makes_no_request(env):
  env.u.wall = lambda: 1000.0
  assert env.u.cycle() == 60.0
  assert env.http.calls == [] and text(env) == "waiting for a valid clock"


def test_toggle_on_disables_everything_and_an_error_stays_visible(env):
  env.params.d["DisableCurveDbAutoInstall"] = True
  env.u.cycle()
  assert env.http.calls == [] and st.ui_text(True, env.status(), 0) == st.DISABLED_TEXT
  assert st.ui_text(True, "error: auth HTTP 401 @5", 6) == "error: auth HTTP 401"


# ---- the pointer: each outcome is distinct -----------------------------------------------------------------------
def test_pointer_outcomes_are_distinct(env):
  seen = {}
  for name, r, delay in [("404", Resp(404), 3600.0), ("401", Resp(401), 3600.0), ("403", Resp(403), 3600.0),
                         ("503", Resp(503), 60.0), ("timeout", requests.exceptions.Timeout("x"), 120.0),
                         ("conn", requests.exceptions.ConnectionError("y"), 240.0), ("418", Resp(418), 3600.0)]:
    env.http.routes[("GET", "/approved")] = r
    assert env.u.cycle() == delay, name
    seen[name] = text(env)
  assert seen["404"] == "none published"
  assert seen["401"] == "error: auth HTTP 401" and seen["403"] == "error: auth HTTP 403"
  assert seen["503"].startswith("gateway unavailable (HTTP 503)")
  assert seen["timeout"] == "pointer GET: Timeout" and seen["conn"] == "pointer GET: ConnectionError"
  assert len(set(seen.values())) == len(seen)
  assert env.http.n("GET", "SECRETROWS?sig=abc") == 0              # nothing was downloaded on any of them


def test_backoff_resets_after_success(env):
  env.http.routes[("GET", "/approved")] = Resp(503)
  assert [env.u.cycle() for _ in range(7)] == [60, 120, 240, 480, 960, 1800, 1800]
  env.route()
  env.u.cycle()
  env.http.routes[("GET", "/approved")] = Resp(503)
  assert env.u.cycle() == 60


def test_pointer_is_sent_with_the_device_jwt_to_the_dongle_route(env):
  env.u.cycle()
  m, url, kw = env.http.calls[0]
  assert (m, url) == ("GET", f"{env.base}/v1/curvedb/2fd850c60cc5bfef/approved") and kw["headers"] == {"Authorization": "JWT TOKEN"}


@pytest.mark.parametrize("patch", [{"sha256": "zz"}, {"manifest_sha256": "A" * 64}, {"rows_bytes": 4 * 1024 * 1024 + 1}, {"rows_bytes": 0},
                                   {"manifest_bytes": 65 * 1024}, {"rows_bytes": "5"}, {"rows_url": "http://insecure/x"}, {"manifest_url": None}])
def test_bad_pointer_is_refused(env, patch):
  env.route({**env.ptr, **patch})
  env.u.cycle()
  assert text(env) == "error: bad pointer" and env.http.n("GET", "SECRETROWS?sig=abc") == 0 and env.live_sha() == env.old_sha


def test_pointer_missing_a_field_is_refused(env):
  env.route({k: v for k, v in env.ptr.items() if k != "sha256"})
  env.u.cycle()
  assert text(env) == "error: bad pointer"


# ---- install ---------------------------------------------------------------------------------------------------
def test_happy_path_installs_records_and_reports(env):
  env.record(env.old_sha)
  assert env.u.cycle() == 3600.0
  assert env.live_sha() == env.new_sha
  assert re.fullmatch(rf"installed {env.new_sha[:8]} \(\d\d-\d\d\)", text(env))
  rec = json.loads(pathlib.Path(cu.RECORD_FILE).read_text())
  assert rec["last_auto_sha"] == env.new_sha and rec["auto_utc"] > 1
  assert any(n.startswith("curvedb_v2.bak-") and n.endswith("-auto") for n in os.listdir(pathlib.Path(env.live).parent))
  posts = [c for c in env.http.calls if c[0] == "POST"]
  assert len(posts) == 1 and posts[0][1] == f"{env.base}/v1/curvedb/2fd850c60cc5bfef/report"
  assert posts[0][2]["headers"] == {"Authorization": "JWT TOKEN"}


def test_report_payload_schema(env):
  env.record(env.old_sha)
  env.u.git_sha = "abc123"
  env.u.cycle()
  p = [c for c in env.http.calls if c[0] == "POST"][0][2]["json"]
  assert set(p) == {"live_sha", "pointer_sha", "last_auto_sha", "auto_utc", "loaded", "state", "reason", "free_mb", "git_sha"}
  assert p["live_sha"] == p["pointer_sha"] == p["last_auto_sha"] == env.new_sha
  assert isinstance(p["auto_utc"], int) and p["loaded"] is False and p["state"] == "installed" and p["free_mb"] == 2000 and p["git_sha"] == "abc123"
  assert len(p["reason"]) <= 200 and re.fullmatch(r"[\x20-\x7e]*", p["reason"])


def test_report_reason_is_sanitized_and_capped(env):
  s = cu.Stop("error", "error: x", "bad\n\x00é" + "z" * 500)
  env.u.net_ok = True
  env.u.maybe_report(s)
  r = [c for c in env.http.calls if c[0] == "POST"][0][2]["json"]["reason"]
  assert len(r) == 200 and "\n" not in r and "é" not in r


def test_report_on_change_and_at_least_daily_not_every_cycle(env):
  clock = [0.0]
  env.u.mono = lambda: clock[0]
  env.record(env.old_sha)
  env.u.cycle()
  env.u.cycle()                                    # same state ("installed" -> now "current"): a change, one more report
  n = env.http.n("POST")
  env.u.cycle()
  assert env.http.n("POST") == n                    # unchanged state, < 24 h: no report
  clock[0] += 86401
  env.u.cycle()
  assert env.http.n("POST") == n + 1                # daily


def test_failed_report_is_logged_and_retried_not_fatal(env):
  env.record(env.old_sha)
  env.http.routes[("POST", "/report")] = Resp(500)
  env.u.cycle()
  assert env.live_sha() == env.new_sha and " report failed " in pathlib.Path(cu.LOG_FILE).read_text()
  env.http.routes[("POST", "/report")] = Resp(200)
  env.u.cycle()
  assert env.http.n("POST") == 2


def test_urls_are_never_logged_or_published(env):
  env.record(env.old_sha)
  env.u.cycle()
  env.http.routes[("GET", "SECRETROWS?sig=abc")] = requests.exceptions.ConnectionError("https://s3.example/SECRETROWS?sig=abc")
  env.u.cycle()
  everything = pathlib.Path(cu.LOG_FILE).read_text() + json.dumps(env.params.puts) + json.dumps([c[2].get("json") for c in env.http.calls])
  assert "SECRETROWS" not in everything and "sig=abc" not in everything and "TOKEN" not in pathlib.Path(cu.LOG_FILE).read_text()


def test_rows_sha_mismatch_is_refused(env):
  env.record(env.old_sha)
  env.http.routes[("GET", "SECRETROWS?sig=abc")] = Resp(200, b"x" * len(env.new_rows))
  env.u.cycle()
  assert text(env) == "error: rows sha/size mismatch" and env.live_sha() == env.old_sha and not pathlib.Path(swap.NEW).exists()


def test_manifest_sha_mismatch_is_refused(env):
  env.record(env.old_sha)
  env.http.routes[("GET", "SECRETMAN?sig=abc")] = Resp(200, b"y" * len(env.new_man))
  env.u.cycle()
  assert text(env) == "error: manifest sha/size mismatch" and env.live_sha() == env.old_sha


def test_download_larger_than_announced_is_cut_off(env):
  env.record(env.old_sha)
  env.http.routes[("GET", "SECRETROWS?sig=abc")] = Resp(200, env.new_rows + b"EXTRA")
  env.u.cycle()
  assert text(env) == "error: rows larger than announced" and env.live_sha() == env.old_sha


def test_download_http_error_retries_with_backoff(env):
  env.record(env.old_sha)
  env.http.routes[("GET", "SECRETROWS?sig=abc")] = Resp(403)
  assert env.u.cycle() == 60.0 and text(env) == "rows download HTTP 403" and env.live_sha() == env.old_sha


def test_device_loader_refusal_keeps_the_old_table(env, tmp_path, monkeypatch):
  env.record(env.old_sha)
  bad = tmp_path / "bad.py"
  bad.write_text("import sys; sys.exit('corrupt')")
  monkeypatch.setattr(swap, "LOADER_SCRIPT", str(bad))
  env.u.cycle()
  assert text(env).startswith("error: ") and env.live_sha() == env.old_sha


# ---- when it may install (Rule 3: gear, not IsOnroad) ------------------------------------------------------------
@pytest.mark.parametrize("state,free,want,delay", [(("0", "1", "900"), 2000, "driving", 3600.0), (("1", "1", "60"), 2000, "selfdrived up only", 150.0),
                                                   (("1", "1", ""), 2000, "uptime unknown", 3600.0), (("?", "1", "900"), 2000, "unreadable", 3600.0),
                                                   (("1", "1", "900"), 100, "100 MB free", 3600.0)])
def test_unsafe_states_refuse_before_any_download(env, state, free, want, delay):
  env.record(env.old_sha)
  env.state, env.free = state, free
  assert env.u.cycle() == delay
  assert text(env).startswith("waiting: ") and want in text(env)
  assert env.http.n("GET", "SECRETROWS?sig=abc") == 0 and env.live_sha() == env.old_sha


@pytest.mark.parametrize("state", [("0", "0", ""), ("1", "0", ""), ("1", "1", "900")])
def test_offroad_or_settled_park_installs(env, state):
  env.record(env.old_sha)
  env.state = state
  env.u.cycle()
  assert env.live_sha() == env.new_sha


def test_driving_that_starts_mid_install_stops_at_the_swap(env, monkeypatch):
  import itertools
  env.record(env.old_sha)
  seq = itertools.chain([("1", "1", "900")] * 3, itertools.repeat(("0", "1", "900")))   # decide, stage, swap ok; re-check: driving
  monkeypatch.setattr(swap, "now_state", lambda: next(seq))
  env.u.cycle()
  assert text(env).startswith("waiting: ") and "state changed during the swap" in text(env) and env.live_sha() == env.old_sha


def test_safe_now_is_gear_not_isonroad(env):
  p = env.params
  p.d.update(IsOnroad=True, GearPark=False)
  assert env.u.safe_now() is False                   # ignition on, in gear = driving
  p.d.update(IsOnroad=True, GearPark=True)
  assert env.u.safe_now() is True                    # parked and charging reads IsOnroad=1: still safe
  p.d.update(IsOnroad=False, GearPark=False)
  assert env.u.safe_now() is True


# ---- hand-change hold ---------------------------------------------------------------------------------------------
def test_table_changed_by_hand_holds_and_never_overrides(env):
  env.record("e" * 64)                               # we installed something else last: the live table is not ours
  env.u.cycle()
  assert text(env) == "held: table changed by hand" and env.live_sha() == env.old_sha and env.http.n("GET", "SECRETROWS?sig=abc") == 0
  assert [c for c in env.http.calls if c[0] == "POST"][0][2]["json"]["state"] == "held"


def test_no_install_record_holds_until_adopt_then_resumes(env):
  env.u.cycle()
  assert text(env) == "held: table changed by hand" and env.live_sha() == env.old_sha
  assert cu.adopt() == 0
  assert env.u.cycle() == 3600.0 and env.live_sha() == env.new_sha


def test_adopt_without_a_live_table_fails_loudly(env, capsys):
  import shutil
  shutil.rmtree(env.live)
  assert cu.adopt() == 1 and "no live table" in capsys.readouterr().err


def test_missing_live_dir_recovers_then_holds(env):
  env.record(env.old_sha)
  swap.stage(env.new_sha, env.new_rows, env.new_man)
  swap.swap(env.new_sha)                             # leaves an -auto backup of the old table
  env.route({**env.ptr, "sha256": "d" * 64})
  import shutil
  shutil.rmtree(env.live)
  env.u.cycle()
  assert text(env).startswith("held: live table was missing") and env.live_sha() == env.old_sha


def test_corrupt_record_is_treated_as_no_record_and_logged(env):
  pathlib.Path(cu.RECORD_FILE).write_text("{not json")
  env.u.cycle()
  assert text(env) == "held: table changed by hand" and "unreadable" in pathlib.Path(cu.LOG_FILE).read_text()


# ---- already current ------------------------------------------------------------------------------------------------
def test_current_table_downloads_nothing_and_reports(env):
  env.route({**env.ptr, "sha256": env.old_sha})
  env.record("z" * 64)                               # a record that disagrees with an approved-equal table is repaired, not held
  env.u.cycle()
  assert re.fullmatch(rf"installed {env.old_sha[:8]} \(\d\d-\d\d\)", text(env)) and env.http.n("GET", "SECRETROWS?sig=abc") == 0
  assert json.loads(pathlib.Path(cu.RECORD_FILE).read_text())["last_auto_sha"] == env.old_sha
  assert [c for c in env.http.calls if c[0] == "POST"][0][2]["json"]["state"] == "current"


def test_current_but_selfdrived_loaded_a_different_table_is_an_error(env):
  env.route({**env.ptr, "sha256": env.old_sha})
  env.record(env.old_sha)
  pathlib.Path(swap.SWAGLOG, "swaglog.1").write_text('{"event$s": "curvedb_v2_loaded", "sha256$s": "' + env.new_sha + '"}\n')
  env.u.cycle()
  assert text(env).startswith("error: selfdrived loaded")


def test_current_and_loaded_reports_loaded_true(env):
  env.route({**env.ptr, "sha256": env.old_sha})
  env.record(env.old_sha)
  pathlib.Path(swap.SWAGLOG, "swaglog.1").write_text('{"event$s": "curvedb_v2_loaded", "sha256$s": "' + env.old_sha + '"}\n')
  env.u.cycle()
  assert [c for c in env.http.calls if c[0] == "POST"][0][2]["json"]["loaded"] is True


# ---- the shared lock ------------------------------------------------------------------------------------------------
def test_install_waits_when_the_ssh_gate_holds_the_lock(env, monkeypatch):
  env.record(env.old_sha)
  real = swap.lock
  monkeypatch.setattr(swap, "lock", lambda wait_s=0.2: real(wait_s))
  gate = real()                                      # the gate (or another swapper) holds the one lock file
  env.u.cycle()
  assert text(env) == "waiting: another swap in progress" and env.live_sha() == env.old_sha
  gate.close()
  env.u.cycle()
  assert env.live_sha() == env.new_sha


# ---- structure ------------------------------------------------------------------------------------------------------
def _code_words(mod) -> set[str]:
  """Every identifier, attribute and string literal in the module, minus docstrings and comments."""
  tree = ast.parse(pathlib.Path(mod.__file__).read_text())
  doc = {id(n.body[0].value) for n in ast.walk(tree) if isinstance(n, (ast.Module, ast.FunctionDef, ast.ClassDef)) and n.body
         and isinstance(n.body[0], ast.Expr) and isinstance(n.body[0].value, ast.Constant)}
  out = set()
  for n in ast.walk(tree):
    if isinstance(n, ast.Name):
      out.add(n.id.lower())
    elif isinstance(n, ast.Attribute):
      out.add(n.attr.lower())
    elif isinstance(n, ast.Constant) and isinstance(n.value, str) and id(n) not in doc:
      out.add(n.value.lower())
  return out


def test_nothing_in_the_feature_can_reboot_or_touch_panda():
  for mod in (cu, swap):
    words = _code_words(mod)
    for banned in ("reboot", "shutdown", "systemctl", "system", "poweroff", "panda", "sendcan", "carstate", "hardware_reboot"):
      assert not any(banned == w or (len(banned) > 6 and banned in w) for w in words), (mod.__name__, banned)


def test_daemon_subscribes_to_devicestate_only_and_declares_the_process():
  src = pathlib.Path(cu.__file__).read_text()
  assert 'messaging.SubMaster(["deviceState"])' in src and src.count("SubMaster(") == 1
  assert "carState" not in src.replace("carState).", "").split("def main")[1]
  from openpilot.system.manager.process_config import managed_processes
  p = managed_processes["curvedb_updated"]
  assert p.restart_if_crash and p.module == "system.curvedb.curvedb_updated" and p.should_run(False, None, None)


def test_params_registered_with_default_off():
  from openpilot.common.params import Params
  p = Params()
  assert p.get_bool("DisableCurveDbAutoInstall") is False        # opt-out toggle: default OFF = feature runs
  assert p.get("CurveDbAutoInstallStatus") in (None, b"", "")    # no persistent stale state
