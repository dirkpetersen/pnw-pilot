"""curvedb v2 -- loading helpers shared by v2_validate / v2_build / v2_replay (I/O only, no decisions)."""
from __future__ import annotations

import datetime
import glob
import gzip
import json
import os
import zoneinfo
from collections import Counter

PT = zoneinfo.ZoneInfo("America/Los_Angeles")
STREAMS = ("pose", "cs", "cc", "gps", "mapd")


def pt_date(t: float) -> str:
  """PT calendar date of a UNIX epoch -- the leave-one-date-out key (Rule 7: logs are UTC, driver is PT)."""
  return datetime.datetime.fromtimestamp(t, PT).strftime("%Y-%m-%d")


def pt_str(t: float) -> str:
  return datetime.datetime.fromtimestamp(t, PT).strftime("%m-%d %H:%M:%S")


def route_of(seg: str) -> tuple[str, int]:
  r, n = seg.rsplit("--", 1)
  return r, int(n)


def list_routes(tracks_dir: str, kind: str = "qlog") -> dict[str, list[str]]:
  """route -> segment files, in segment order."""
  out: dict[str, list[tuple[int, str]]] = {}
  for f in glob.glob(os.path.join(tracks_dir, f"*.{kind}.json.gz")):
    seg = os.path.basename(f)[: -len(f".{kind}.json.gz")]
    r, n = route_of(seg)
    out.setdefault(r, []).append((n, f))
  return {r: [f for _, f in sorted(v)] for r, v in out.items()}


def load_route(files: list[str], tally: Counter | None = None) -> dict | None:
  """Concatenate a route's segments. logMonoTime is continuous within a boot, so the streams simply
  append. The route's UNIX offset is the MEDIAN of its segments' GPS offsets; a segment whose own offset
  disagrees with it by more than 2 s is counted (it would mean the route spans a reboot). Returns None
  (and counts why) when no segment ever had a GPS fix."""
  doc = {k: [] for k in STREAMS}
  offs, fp = [], None
  for f in files:
    with gzip.open(f, "rt") as fh:
      d = json.load(fh)
    for k in STREAMS:
      doc[k].extend(d[k])
    if d.get("off_s") is not None:
      offs.append(d["off_s"])
    fp = fp or d.get("fp")
  if not offs:
    if tally is not None:
      tally["route dropped: no GPS fix in any segment"] += 1
    return None
  offs.sort()
  off = offs[len(offs) // 2]
  if tally is not None and any(abs(o - off) > 2.0 for o in offs):
    tally["route with a segment offset >2 s from the route median"] += 1
  for k in STREAMS:
    doc[k].sort(key=lambda r: r[0])
  doc["off_s"], doc["fp"] = off, fp
  return doc


def load_ces(paths: list[str], evs=("tick", "adopt")) -> list[dict]:
  """ces_events records (any generation), deduplicated on (car, ev, t) like ingest.py, sorted by t."""
  seen, out = set(), []
  for p in paths:
    op = gzip.open if p.endswith(".gz") else open
    with op(p, "rt") as fh:
      for line in fh:
        try:
          r = json.loads(line)
        except ValueError:
          continue
        if r.get("ev") not in evs or not isinstance(r.get("t"), (int, float)):
          continue
        key = (r.get("car"), r.get("ev"), round(r["t"], 2))
        if key in seen:
          continue
        seen.add(key)
        out.append(r)
  out.sort(key=lambda r: r["t"])
  return out
