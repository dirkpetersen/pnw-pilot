"""curvedblive2pnw test support: a valid, tiny curve-DB file and an A reader, so every CESController a test
builds runs the LIVE DB against known data instead of whatever is (or is not) in /data/pnw/curvedb_v2.

Why an autouse fixture (conftest.py) and not a production switch: on the car a missing file is a loud
cloudlog.error, and the Lightning "nothing fails -> no errors" tests would otherwise see it, from a thread,
at a moment that depends on scheduling. The synthetic DB holds one anchor far from every test scene, so it
has no effect on any existing test's control output -- which those tests then prove.
"""
from __future__ import annotations

import hashlib
import json
import os

import zstandard

from openpilot.selfdrive.controls.lib.ces_pnw import curvedb_live as cl

FAR_ANCHOR = [0.5, 0.5, 0.0, [[0.5013, 0.5, 0.001, 3]]]   # the Gulf of Guinea: no test drives there
MAPD_SETTINGS = {"personalities": {"aggressive": {"map_curve_target_lat_a": 2.4},
                                   "standard": {"map_curve_target_lat_a": 2.2},
                                   "relaxed": {"map_curve_target_lat_a": 1.9}}}


def write_db(d: str, anchors: list, *, params: dict | None = None, fmt: str = cl.FORMAT, exclude_date=None,
             tamper=None) -> str:
  """Write a rows file + manifest the way tools/curvedb/v2_live_export.py does. `tamper(blob) -> blob`
  corrupts the rows AFTER the manifest hash was taken."""
  os.makedirs(d, exist_ok=True)
  doc = {"format": fmt, "params": params or dict(cl.EXPECTED_PARAMS), "exclude_date": exclude_date,
         "anchors": anchors}
  blob = zstandard.ZstdCompressor().compress(json.dumps(doc).encode())
  n_rows = sum(1 for a in anchors for b in a[3] if b[2] is not None)
  man = {"format": fmt, "file": cl.ROWS_NAME, "bytes": len(blob), "sha256": hashlib.sha256(blob).hexdigest(),
         "anchors": len(anchors), "rows_with_authority": n_rows, "first_date": "2026-07-13",
         "last_date": "2026-09-22", "exclude_date": exclude_date}
  if tamper is not None:
    blob = tamper(blob)
  with open(os.path.join(d, cl.ROWS_NAME), "wb") as f:
    f.write(blob)
  with open(os.path.join(d, cl.MANIFEST_NAME), "w") as f:
    json.dump(man, f)
  return d


def reader(a_std=2.2, personality=1):
  s = json.loads(json.dumps(MAPD_SETTINGS))
  s["personalities"]["standard"]["map_curve_target_lat_a"] = a_std
  return lambda: (s, personality)


import openpilot.selfdrive.controls.lib.pnw_vehicle as pv
SHIPPED_LAT_A = pv._CURVE_DEFAULTS["curvedb_v2_lat_a"]   # the shipped Lightning default, before any test pins it


def isolate(monkeypatch, tmp_path_factory):
  d = write_db(str(tmp_path_factory.mktemp("roaddb")), [FAR_ANCHOR])
  monkeypatch.setattr(cl, "DATA_DIR", d)
  monkeypatch.setattr(cl, "READ_PARAMS", [reader()])
  monkeypatch.setattr(cl, "BACKGROUND", [False])     # load in the constructor: no thread, no race
  monkeypatch.setattr(cl, "A_MAX_AGE_S", float("inf"))   # A is read once there and never re-polled
  # The shipped default is a fixed 2.2 (owner 2026-09-24); these controller tests exercise the "A from mapd" path
  # the reader above feeds, so pin the knob to 0 = mapd. test_the_shipped_default_is_2_5 covers the default itself.
  monkeypatch.setitem(pv._CURVE_DEFAULTS, "curvedb_v2_lat_a", 0.0)
  return d
