"""Every report the daemon can emit, validated against the DEPLOYED gateway's exact rules (comma-connect 9f72377, Lambda v2).
A 400 from the gateway is OUR bug, so these rules are copied here and must be kept in step with the gateway."""
import json
import pathlib
import re

import pytest
import requests

from openpilot.system.curvedb import curvedb_updated as cu
from openpilot.system.curvedb.tests.conftest import Resp

KEYS = {"live_sha", "pointer_sha", "last_auto_sha", "auto_utc", "loaded", "state", "reason", "free_mb", "git_sha"}
STATES = {"disabled", "waiting_clock", "waiting_unmetered", "none_published", "current", "installed", "held_hand_change", "refused", "error"}
HEX = re.compile(r"[0-9a-f]{64}")
TOKEN = re.compile(r"[A-Za-z0-9:._+-]{1,64}")


def validate(body: dict) -> None:
  assert set(body) <= KEYS, set(body) - KEYS
  assert len(json.dumps(body)) <= 2048
  for k in ("live_sha", "pointer_sha", "last_auto_sha"):
    assert body[k] is None or (isinstance(body[k], str) and HEX.fullmatch(body[k])), k
  assert body["auto_utc"] is None or (isinstance(body["auto_utc"], str) and TOKEN.fullmatch(body["auto_utc"])), "auto_utc"
  assert isinstance(body["loaded"], bool)
  assert body["state"] in STATES
  assert isinstance(body["reason"], str) and len(body["reason"]) <= 200 and re.fullmatch(r"[\x20-\x7e]*", body["reason"])
  assert type(body["free_mb"]) is int and body["free_mb"] >= 0
  assert isinstance(body["git_sha"], str) and TOKEN.fullmatch(body["git_sha"])


def posted(env):
  return [c[2]["json"] for c in env.http.calls if c[0] == "POST"]


def run(env, setup):
  setup()
  env.u.cycle()
  return posted(env)


def test_every_reachable_report_state_is_valid(env, tmp_path, monkeypatch):
  seen = set()

  def fresh(setup):
    env.http.calls.clear()
    env.u.rep_key = None
    for b in run(env, setup):
      validate(b)
      seen.add(b["state"])

  fresh(lambda: env.record(env.old_sha))                                                # installed
  fresh(lambda: None)                                                                   # current (live == pointer now)
  env.record("e" * 64)
  env.route({**env.ptr, "sha256": "f" * 64})
  fresh(lambda: None)                                                                   # held_hand_change
  env.record(env.new_sha)
  env.state = ("0", "1", "900")
  fresh(lambda: None)                                                                   # refused (driving)
  env.state = ("1", "1", "900")
  env.http.routes[("GET", "/approved")] = Resp(404)
  fresh(lambda: None)                                                                   # none_published (pointer_sha null)
  for r in (Resp(401), Resp(503), requests.exceptions.Timeout("t")):                    # error
    env.http.routes[("GET", "/approved")] = r
    fresh(lambda: None)
  env.route()
  env.http.routes[("GET", "SECRETROWS?sig=abc")] = Resp(200, b"x" * len(env.new_rows))  # error (sha mismatch)
  env.record(env.live_sha())
  fresh(lambda: None)
  env.u.git_sha = None                                                                  # git_sha path incl. the unavailable fallback
  monkeypatch.setattr("openpilot.common.git.get_commit", lambda *a, **k: (_ for _ in ()).throw(OSError("no git")))
  fresh(lambda: None)
  assert {"installed", "current", "held_hand_change", "refused", "none_published", "error"} <= seen


def test_validator_rejects_the_old_shape():
  old = {"live_sha": "", "pointer_sha": "", "last_auto_sha": "", "auto_utc": 5, "loaded": False, "state": "held", "reason": "x",
         "free_mb": 1, "git_sha": ""}
  with pytest.raises(AssertionError):
    validate(old)


def test_a_400_is_logged_loudly_and_not_retried_every_cycle(env):
  env.record(env.old_sha)
  env.http.routes[("POST", "/report")] = Resp(400)
  env.u.cycle()                                    # installed -> 400
  env.u.cycle()                                    # state changed to current -> one more try
  n = env.http.n("POST")
  env.u.cycle()
  env.u.cycle()
  assert n == 2 and env.http.n("POST") == n        # unchanged state: no retry spin
  assert "REJECTED" in pathlib.Path(cu.LOG_FILE).read_text()


def test_git_sha_is_clamped_to_the_gateway_charset(env, monkeypatch):
  monkeypatch.setattr("openpilot.common.git.get_commit", lambda *a, **k: "ab c/d\n" + "x" * 100)
  env.record(env.old_sha)
  env.u.cycle()
  validate(posted(env)[0])
