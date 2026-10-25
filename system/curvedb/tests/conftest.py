"""Shared fixtures: a fake /data tree, a fake gateway, controllable safety state. No real device, network or clock needed."""
import hashlib
import json
import os
import pathlib
import textwrap

import pytest
import requests

from openpilot.system.curvedb import curvedb_updated as cu
from openpilot.system.curvedb import swap

LOADER = textwrap.dedent("""
  import json, os, resource, sys
  import pathlib
  m = json.loads(pathlib.Path(sys.argv[1], "manifest.json").read_text())
  print(json.dumps({"rows": 3, "manifest_rows": 3, "sha": m["sha256"], "cwd": os.getcwd(), "nice": os.nice(0),
                    "as": resource.getrlimit(resource.RLIMIT_AS)[0], "cpu": resource.getrlimit(resource.RLIMIT_CPU)[0],
                    "blas": os.environ.get("OPENBLAS_NUM_THREADS")}))
""")


def table(tag: str) -> tuple[str, bytes, bytes]:
  rows = (tag * 50).encode()
  sha = hashlib.sha256(rows).hexdigest()
  return sha, rows, json.dumps({"sha256": sha, "rows_with_authority": 3}).encode()


class Resp:
  def __init__(self, status=200, body=b"", js=None):
    self.status_code, self.content, self._js = status, body, js

  def json(self):
    if self._js is None:
      raise ValueError("no json")
    return self._js

  def iter_content(self, n):
    for i in range(0, len(self.content), n):
      yield self.content[i:i + n]


class FakeHttp:
  exceptions = requests.exceptions

  def __init__(self):
    self.routes: dict[tuple[str, str], object] = {}    # (method, url suffix) -> Resp | Exception
    self.calls: list[tuple[str, str, dict]] = []

  def request(self, method, url, **kw):
    self.calls.append((method, url, kw))
    for (m, suffix), r in self.routes.items():
      if m == method and url.endswith(suffix):
        if isinstance(r, Exception):
          raise r
        return r
    raise AssertionError(f"unrouted {method} {url}")

  def n(self, method=None, suffix=None):
    return sum(1 for m, u, _ in self.calls if (method is None or m == method) and (suffix is None or u.endswith(suffix)))


class FakeParams:
  def __init__(self, **kw):
    self.d = {"DongleId": "2fd850c60cc5bfef", **kw}
    self.puts = []

  def get_bool(self, k):
    return bool(self.d.get(k))

  def get(self, k):
    return self.d.get(k)

  def put(self, k, v):
    self.puts.append((k, v))
    self.d[k] = v


class Env:
  pass


@pytest.fixture
def env(tmp_path, monkeypatch):
  e = Env()
  data = tmp_path / "data"
  (data / "pnw").mkdir(parents=True)
  (data / "log").mkdir()
  e.live = str(data / "pnw/curvedb_v2")
  monkeypatch.setattr(swap, "LIVE", e.live)
  monkeypatch.setattr(swap, "NEW", e.live + ".new")
  monkeypatch.setattr(swap, "REPL", e.live + ".replaced")
  monkeypatch.setattr(swap, "LOCK_FILE", str(data / "pnw/curvedb-swap-gate.log.lock"))
  monkeypatch.setattr(swap, "SWAGLOG", str(data / "log"))
  monkeypatch.setattr(swap, "ROOT", str(tmp_path))
  monkeypatch.setattr(cu, "LOG_FILE", str(data / "pnw/curvedb-autoinstall.log"))
  monkeypatch.setattr(cu, "RECORD_FILE", str(data / "pnw/curvedb_autoinstall.json"))
  monkeypatch.setattr(cu, "LOADED_MARKER", str(data / "pnw/curvedb_loaded.json"))
  loader = tmp_path / "loader.py"
  loader.write_text(LOADER)
  monkeypatch.setattr(swap, "LOADER_SCRIPT", str(loader))
  e.state = ("1", "1", "500")     # gear park, onroad (ignition) = 1, selfdrived up 500 s: Park with a settled selfdrived
  e.free = 2000
  monkeypatch.setattr(swap, "now_state", lambda: e.state)
  monkeypatch.setattr(swap, "free_mb", lambda: e.free)
  e.old_sha, rows, man = table("old")
  os.makedirs(e.live)
  pathlib.Path(e.live, swap.ROWS).write_bytes(rows)
  pathlib.Path(e.live, "manifest.json").write_bytes(man)
  e.new_sha, e.new_rows, e.new_man = table("new")
  e.http = FakeHttp()
  e.params = FakeParams()
  e.net = (1, False)              # wifi, not metered
  e.u = cu.Updater(e.params, lambda: e.net, http=e.http, token_fn=lambda d: "TOKEN")
  e.tmp = tmp_path
  e.base = "https://jh69za4byd.execute-api.us-west-2.amazonaws.com"
  e.ptr = {"sha256": e.new_sha, "rows_bytes": len(e.new_rows), "manifest_sha256": hashlib.sha256(e.new_man).hexdigest(),
           "manifest_bytes": len(e.new_man), "rows_url": "https://s3.example/SECRETROWS?sig=abc", "manifest_url": "https://s3.example/SECRETMAN?sig=abc"}

  def route(ptr=None):
    e.http.routes[("GET", "/approved")] = Resp(200, b"{}", js=ptr or e.ptr)
    e.http.routes[("GET", "SECRETROWS?sig=abc")] = Resp(200, e.new_rows)
    e.http.routes[("GET", "SECRETMAN?sig=abc")] = Resp(200, e.new_man)
    e.http.routes[("POST", "/report")] = Resp(200)
  e.route = route
  route()
  e.record = lambda sha: pathlib.Path(cu.RECORD_FILE).write_text(json.dumps({"last_auto_sha": sha, "auto_utc": 1}))
  e.status = lambda: e.params.d.get("CurveDbAutoInstallStatus", "")
  e.live_sha = lambda: swap.live_sha()
  return e
