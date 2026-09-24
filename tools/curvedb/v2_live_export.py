#!/usr/bin/env python3
"""curvedb v2 LIVE -- export the road table as the file the car loads (curvedblive2pnw).

  table.json.gz (v2_build.py) -> curvedb_v2_rows.json.zst (zstd JSON) + manifest.json (SHA-256 of the .zst)

THE FILE IS PRIVATE. It is a map of the owner's driven positions, his home area included, and
pnw-pilot is a PUBLIC repository: never commit the output to it. It lives on the device at
/data/pnw/curvedb_v2/ and in the private workdir repo.

WHAT IS IN IT. Every anchor of the table (not only the ones with authority) with every branch it has:

  anchors: [[lat, lon, brg, [[end_lat, end_lon, k_or_null, n_dates], ...]], ...]

* `k` is set ONLY on a branch that `roadtable.row_verdict` grants authority to, with the owner's rules
  unchanged: >= 2 distinct PT dates, never Tesla-only evidence, no pass whose extent reaches a ramp,
  a known highway class, dates that agree. That is the only curvature in the file.
* A refused branch is exported WITHOUT a curvature (`null`). It exists so the car can tell "my path
  takes the refused branch" from "my path takes the granted one": a query whose end point lies within
  the branch radius of a refused branch, or of two branches, gets NO DB effect (ambiguous). Leaving
  refused branches out would let a query that takes a refused exit ramp match the mainline row next to
  it.
* An anchor with no granted branch is exported too, so the car's nearest-anchor search picks the same
  anchor the offline replay picked; a nearest anchor with nothing granted means no DB effect.

`--exclude-date` builds the same file leave-one-date-out (the replay test's fixture; never shipped).

Run:
  PYTHONPATH=<worktree> python tools/curvedb/v2_live_export.py --table <v2>/db/table.json.gz --out <dir>
"""
from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import os
import sys

import zstandard

from openpilot.tools.curvedb import roadtable as rt
from openpilot.tools.curvedb.v2_replay import load_table

FORMAT = "curvedb-v2-live/1"          # the car refuses any other value (curvedb_live.FORMAT)
ROWS_NAME = "curvedb_v2_rows.json.zst"   # zstd JSON (owner 2026-09-24); the manifest hashes THIS file
MANIFEST_NAME = "manifest.json"
# The keying parameters the car re-implements. Written into the file so the car can refuse a file
# built with different ones instead of silently matching with its own constants.
KEY_PARAMS = ("site_radius_m", "heading_tol_deg", "extent_back_m", "extent_fwd_m", "branch_radius_m")


def export_doc(idx: rt.AnchorIndex, exclude_date: str | None = None) -> tuple[dict, dict]:
  """(document, counts). Pure apart from reading `idx`."""
  p = idx.p
  anchors, n_rows, n_branches, dates = [], 0, 0, set()
  for a in idx.anchors:
    obs_dates = {o.date for o in a.obs if o.date != exclude_date}
    dates |= obs_dates
    brs = []
    # Branch centers come from the passes that remain after the exclusion, exactly as the replay's
    # query side sees them (row_verdict filters by date first, then by branch).
    kept = rt.Anchor(a.lat, a.lon, a.brg, [o for o in a.obs if o.date != exclude_date])
    for br in rt.branches(kept, p.branch_radius_m):
      v = rt.row_verdict(a, p, exclude_date=exclude_date, branch=br)
      n_branches += 1
      k = round(v.k, 8) if v.granted else None
      n_rows += v.granted
      brs.append([round(br[0], 6), round(br[1], 6), k, v.n_dates])
    anchors.append([round(a.lat, 6), round(a.lon, 6), round(a.brg, 1), brs])
  doc = {"format": FORMAT, "params": {k: getattr(p, k) for k in KEY_PARAMS},
         "exclude_date": exclude_date, "anchors": anchors}
  counts = {"anchors": len(anchors), "branches": n_branches, "rows_with_authority": n_rows,
            "dates": sorted(dates)}
  return doc, counts


def write(doc: dict, counts: dict, out: str, source: str) -> dict:
  os.makedirs(out, exist_ok=True)
  raw = json.dumps(doc, separators=(",", ":")).encode()
  # zstd frames carry no timestamp: the same table always gives the same bytes, so the same hash
  blob = zstandard.ZstdCompressor(level=19).compress(raw)
  path = os.path.join(out, ROWS_NAME)
  tmp = path + ".tmp"
  with open(tmp, "wb") as f:
    f.write(blob)
  os.replace(tmp, path)
  with open(source, "rb") as f:
    src_sha = hashlib.sha256(f.read()).hexdigest()
  man = {"format": FORMAT, "file": ROWS_NAME, "bytes": len(blob), "sha256": hashlib.sha256(blob).hexdigest(),
         "anchors": counts["anchors"], "branches": counts["branches"],
         "rows_with_authority": counts["rows_with_authority"],
         "first_date": counts["dates"][0] if counts["dates"] else None,
         "last_date": counts["dates"][-1] if counts["dates"] else None, "n_dates": len(counts["dates"]),
         "exclude_date": doc["exclude_date"], "source_table": os.path.basename(source), "source_sha256": src_sha,
         "built_utc": datetime.datetime.now(datetime.UTC).strftime("%Y-%m-%dT%H:%M:%SZ")}
  mpath = os.path.join(out, MANIFEST_NAME)
  with open(mpath + ".tmp", "w") as f:
    json.dump(man, f, indent=1)
    f.write("\n")
  os.replace(mpath + ".tmp", mpath)
  return man


def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("--table", required=True, help="table.json.gz from v2_build.py")
  ap.add_argument("--out", required=True)
  ap.add_argument("--exclude-date", help="leave-one-date-out build (test fixture only; never ship it)")
  ap.add_argument("--expect-rows", type=int, help="fail unless exactly this many rows have authority")
  a = ap.parse_args(argv)
  idx = load_table(a.table)
  doc, counts = export_doc(idx, a.exclude_date)
  if counts["rows_with_authority"] == 0:
    print("ZERO rows with authority -- refusing to write a file the car would load as an empty DB")
    return 2
  if a.expect_rows is not None and counts["rows_with_authority"] != a.expect_rows:
    print(f"rows with authority {counts['rows_with_authority']} != expected {a.expect_rows} -- not written")
    return 2
  man = write(doc, counts, a.out, a.table)
  print(json.dumps(man, indent=1))
  return 0


if __name__ == "__main__":
  sys.exit(main())
