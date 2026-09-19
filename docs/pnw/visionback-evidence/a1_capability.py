#!/usr/bin/env python3
"""A1: per-corpus field availability. Rule 2 -- a corpus contributing nothing must be a VISIBLE row."""
import datetime
import json
import zoneinfo
from collections import defaultdict

PT = zoneinfo.ZoneInfo("America/Los_Angeles")
INV = "/home/dp/gh/comma/_scratch/visionback/inventory.json"

# corpus bucket = the drive folder (date[/name]) so rotated generations of one drive are one corpus
def bucket(rel):
  if rel.startswith("arch/"):
    return "ARCHIVE (/tmp/arch, continuous)"
  p = rel.split("/")
  return "/".join(p[:2]) if len(p) > 2 else p[0]


SHOW = ["icbmT", "mapV", "mapDist", "icbmSrc", "icbmC", "stockSet", "spdLim",
        "icbmKVis", "mdlEndX", "visLat", "visTtc",
        "kPeak", "slKActl", "achLat", "icbmKAt", "icbmKAtGap"]


def main():
  inv = json.load(open(INV))
  agg = defaultdict(lambda: {"files": 0, "records": 0, "err": 0, "icbm": 0, "fordmov": 0,
                             "t_min": None, "t_max": None, "cars": defaultdict(int),
                             "present": defaultdict(int)})
  for row in inv:
    a = agg[bucket(row["file"])]
    a["files"] += 1
    a["records"] += row["records"]
    a["err"] += row["parse_err"]
    a["icbm"] += row["icbm_ticks"]
    a["fordmov"] += row["ford_moving"]
    for k, v in row["cars"].items():
      a["cars"][k] += v
    for k, v in row["present"].items():
      a["present"][k] += v
    for k in ("t_min", "t_max"):
      if row[k] is not None:
        a[k] = row[k] if a[k] is None else (min(a[k], row[k]) if k == "t_min" else max(a[k], row[k]))

  def ts(x):
    return datetime.datetime.fromtimestamp(x, PT).strftime("%m-%d %H:%M") if x else "-"

  hdr = f"{'corpus':<50}{'f':>3}{'rec':>8}{'err':>5}{'fordmov':>8}{'icbm':>6}  "
  hdr += "".join(f"{k:>10}" for k in SHOW)
  print("PER-CORPUS FIELD AVAILABILITY (non-null record counts; 'fordmov' = Ford ticks vEgo>4 m/s)")
  print(hdr)
  print("-" * len(hdr))
  tot = defaultdict(int)
  for name in sorted(agg, key=lambda n: agg[n]["t_min"] or 0):
    a = agg[name]
    lbl = f"{name} {ts(a['t_min'])}"
    line = f"{lbl:<50}{a['files']:>3}{a['records']:>8}{a['err']:>5}{a['fordmov']:>8}{a['icbm']:>6}  "
    line += "".join(f"{a['present'][k]:>10}" for k in SHOW)
    print(line)
    for k in SHOW:
      tot[k] += a["present"][k]
    for k in ("files", "records", "err", "fordmov", "icbm"):
      tot[k] += a[k]
  print("-" * len(hdr))
  line = f"{'TOTAL':<50}{tot['files']:>3}{tot['records']:>8}{tot['err']:>5}{tot['fordmov']:>8}{tot['icbm']:>6}  "
  line += "".join(f"{tot[k]:>10}" for k in SHOW)
  print(line)

  # Rule 2: records with NO top-level vEgo are structured event rows (`steerEvent`, `accDrop`),
  # not records of a stationary truck. Reported under their own name so a "parked ticks" count
  # built from `records - moving` cannot silently absorb them.
  nov = sum(r["no_vego"] for r in inv)
  print(f"\nFord records with NO top-level vEgo (structured event rows, excluded from BOTH the "
        f"moving and the stationary tallies): {nov}")

  print("\nCAR MIX per corpus")
  for name in sorted(agg, key=lambda n: agg[n]["t_min"] or 0):
    print(f"  {name:<50} {dict(agg[name]['cars'])}")


if __name__ == "__main__":
  main()
