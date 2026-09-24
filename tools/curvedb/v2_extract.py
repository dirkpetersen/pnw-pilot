#!/usr/bin/env python3
"""curvedb v2 -- qlog/rlog -> compact per-segment track (the I/O half of the road table).

v1 built rows from `ces_events` only, which carries Phase-1-grade curvature from 2026-09-17 onward and
only for the Lightning. v2 builds the ROAD TABLE from the drive logs instead: `livePose` yaw rate and
`carState.vEgo`, which exist on both cars for every uploaded segment (~105 moving hours). This module
only READS a log and writes the raw streams it needs; every decision (curvature, smoothing, admission,
keying) is a pure function in `roadtable.py`, so it can be tested without a log.

Streams kept (times are the log's monotonic seconds; `off_s` converts them to UNIX seconds):

  pose  [t, yaw_z]                         livePose.angularVelocityDevice.z (rad/s), valid samples only
  cs    [t, vEgo, steeringPressed, blinker, park, gasPressed, yawRate]
  cc    [t, latActive]
  gps   [t, hasFix, lat, lon, bearingDeg, bearingAccuracyDeg, speed]
  mapd  [t, wayId, highwayClass, speedLimit, mapCurveSpeed, roadName, wayRef]

`off_s` is the MEDIAN over this segment's GPS fixes of (unixTimestampMillis - logMonoTime). The comma's
RTC is dead at boot, so wall time comes from GPS, never from the log's own clock. A segment with no GPS
fix gets `off_s = null` and the consumer must drop it (and count it) -- it cannot be placed in time.

Run (the 3devpnw schema is required -- an older checkout decodes new fields as zero):
  PYTHONPATH=~/gh/comma/pnw/pnw-pilot:~/gh/comma/pnw/pnw-pilot/opendbc_repo \\
    ~/gh/comma/pnw/pnw-pilot/.venv/bin/python tools/curvedb/v2_extract.py \\
    --logs <dir with <route>--<seg>/{qlog,rlog}.zst> --out <dir> [--segments list.txt] [-j 12]

Rule 2: a segment that fails to decode is written to <out>/FAILED.txt with its traceback and the run
exits non-zero. It is never skipped quietly.
"""
from __future__ import annotations

import argparse
import glob
import gzip
import json
import os
import sys
import traceback
from multiprocessing import Pool


def extract(path: str) -> dict:
  """Read one log file and return the stream dict described in the module docstring."""
  from openpilot.tools.lib.logreader import LogReader
  pose, cs, cc, gps, mapd = [], [], [], [], []
  offs: list[float] = []
  fp = None
  for m in LogReader(path):
    w = m.which()
    t = m.logMonoTime * 1e-9
    if w == "livePose":
      p = m.livePose.angularVelocityDevice
      if p.valid:
        pose.append([round(t, 3), round(p.z, 6)])
    elif w == "carState":
      c = m.carState
      cs.append([round(t, 3), round(c.vEgo, 3), int(c.steeringPressed), int(c.leftBlinker or c.rightBlinker),
                 int(str(c.gearShifter) == "park"), int(c.gasPressed), round(c.yawRate, 5)])
    elif w == "carControl":
      cc.append([round(t, 3), int(m.carControl.latActive)])
    elif w in ("gpsLocation", "gpsLocationExternal"):
      g = getattr(m, w)
      if g.hasFix and g.unixTimestampMillis > 1.7e12:
        offs.append(g.unixTimestampMillis * 1e-3 - t)
      gps.append([round(t, 3), int(g.hasFix), round(g.latitude, 7), round(g.longitude, 7),
                  round(g.bearingDeg, 2), round(g.bearingAccuracyDeg, 2), round(g.speed, 3)])
    elif w == "mapdOut":
      d = m.mapdOut
      mapd.append([round(t, 3), int(d.wayId), str(d.highwayClass), round(d.speedLimit, 3),
                   round(d.mapCurveSpeed, 3), str(d.roadName), str(d.wayRef)])
    elif w == "carParams" and fp is None:
      fp = m.carParams.carFingerprint
  offs.sort()
  return {"fp": fp, "off_s": offs[len(offs) // 2] if offs else None, "n_fix": len(offs),
          "pose": pose, "cs": cs, "cc": cc, "gps": gps, "mapd": mapd}


def _job(args):
  path, out_dir = args
  seg = os.path.basename(os.path.dirname(path))
  kind = "rlog" if os.path.basename(path).startswith("rlog") else "qlog"
  out = os.path.join(out_dir, f"{seg}.{kind}.json.gz")
  if os.path.exists(out):
    return seg, "skip", None
  try:
    doc = extract(path)
    doc.update(seg=seg, kind=kind)
    with gzip.open(out + ".part", "wt") as f:
      json.dump(doc, f, separators=(",", ":"))
    os.replace(out + ".part", out)
    return seg, "ok", None
  except Exception:
    return seg, "fail", traceback.format_exc(limit=4)


def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--logs", required=True, help="directory holding <route>--<seg>/{qlog,rlog}.zst")
  ap.add_argument("--kind", default="qlog", choices=("qlog", "rlog"))
  ap.add_argument("--out", required=True)
  ap.add_argument("--segments", help="optional file: one <route>--<seg> per line; others are skipped")
  ap.add_argument("-j", type=int, default=12)
  a = ap.parse_args(argv)
  os.makedirs(a.out, exist_ok=True)
  paths = sorted(glob.glob(os.path.join(a.logs, "*", f"{a.kind}.zst")) +
                 glob.glob(os.path.join(a.logs, "*", f"{a.kind}.bz2")))
  if a.segments:
    want = {ln.strip() for ln in open(a.segments) if ln.strip()}
    paths = [p for p in paths if os.path.basename(os.path.dirname(p)) in want]
    print(f"segment filter: {len(want)} wanted, {len(paths)} found on disk", flush=True)
  if not paths:
    print(f"no {a.kind} files found under {a.logs} -- refusing to report an empty corpus", file=sys.stderr)
    return 2
  tally: dict[str, int] = {}
  with Pool(a.j) as pool, open(os.path.join(a.out, "FAILED.txt"), "a") as ff:
    for i, (seg, st, err) in enumerate(pool.imap_unordered(_job, [(p, a.out) for p in paths], chunksize=4)):
      tally[st] = tally.get(st, 0) + 1
      if err:
        ff.write(f"{seg}\n{err}\n")
      if i % 500 == 0:
        print(i, len(paths), tally, flush=True)
  print("DONE", tally, flush=True)
  return 1 if tally.get("fail") else 0


if __name__ == "__main__":
  sys.exit(main())
