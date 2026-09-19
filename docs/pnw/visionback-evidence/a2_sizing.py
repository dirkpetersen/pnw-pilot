#!/usr/bin/env python3
"""A2: how many ICBM map/far ticks can actually carry a give-back decision, per corpus.

Every stage prints its n AND what was lost, so an empty stage is attributable rather than silent.
"""
import json
from collections import defaultdict

TICKS = "/home/dp/gh/comma/_scratch/visionback/ticks.jsonl"
MPH = 0.44704
LO, HI = 50.0, 150.0     # the owner's give-back window (m to the candidate)


def num(x):
  try:
    v = float(x)
  except (TypeError, ValueError):
    return None
  return v if v == v and abs(v) != float("inf") else None


def bucket(rel):
  if rel.startswith("arch/"):
    return "ARCHIVE(continuous)"
  p = rel.split("/")
  return "/".join(p[:2]) if len(p) > 2 else p[0]


def main():
  rows = [json.loads(x) for x in open(TICKS)]
  print(f"ford moving ticks: {len(rows)}")
  st = defaultdict(lambda: defaultdict(int))
  for r in rows:
    c = bucket(r["_src"])
    st[c]["ticks"] += 1
    if r.get("icbmT") is None:
      continue
    st[c]["icbm"] += 1
    src = r.get("icbmSrc")
    if src not in ("map", "far"):
      st[c]["icbm_notmapfar"] += 1
      continue
    st[c]["mapfar"] += 1
    d = num(r.get("mapDist")) if src == "map" else None
    # far candidates have no logged distance field of their own; mapDist is CES's 10 s candidate and
    # reads 0.0 on a far tick -- recorded as a loss reason rather than silently substituted.
    if src == "far":
      st[c]["far_nodist"] += 1
      continue
    if d is None or d <= 0:
      st[c]["map_nodist"] += 1
      continue
    st[c]["map_withdist"] += 1
    if not (LO <= d <= HI):
      st[c]["map_outofwindow"] += 1
      continue
    st[c]["inwindow"] += 1
    if num(r.get("icbmKVis")) is not None:
      st[c]["inwin_kvis"] += 1
    if num(r.get("visLat")) is not None:
      st[c]["inwin_visLat"] += 1
      if abs(num(r.get("visLat"))) > 1e-9:
        st[c]["inwin_visLat_nz"] += 1
    if num(r.get("kPeak")) is not None:
      st[c]["inwin_kPeak"] += 1
    if num(r.get("slKActl")) is not None and abs(num(r.get("slKActl"))) > 1e-9:
      st[c]["inwin_slK_nz"] += 1
    if num(r.get("mdlEndX")) is not None:
      st[c]["inwin_mdlEndX"] += 1

  cols = ["ticks", "icbm", "icbm_notmapfar", "mapfar", "far_nodist", "map_nodist", "map_withdist",
          "map_outofwindow", "inwindow", "inwin_kvis", "inwin_visLat", "inwin_visLat_nz",
          "inwin_mdlEndX", "inwin_kPeak", "inwin_slK_nz"]
  w = max(len(c) for c in cols) + 1
  print(f"\n{'corpus':<42}" + "".join(f"{c:>{w}}" for c in cols))
  tot = defaultdict(int)
  for c in sorted(st):
    print(f"{c:<42}" + "".join(f"{st[c][k]:>{w}}" for k in cols))
    for k in cols:
      tot[k] += st[c][k]
  print(f"{'TOTAL':<42}" + "".join(f"{tot[k]:>{w}}" for k in cols))

  # distance histogram of map-sourced ICBM ticks -- is 50-150 m even where the decisions live?
  hist = defaultdict(int)
  for r in rows:
    if r.get("icbmT") is None or r.get("icbmSrc") != "map":
      continue
    d = num(r.get("mapDist"))
    if d is None or d <= 0:
      continue
    hist[int(d // 50) * 50] += 1
  print("\nmap-sourced ICBM ticks by candidate distance (50 m bins):")
  for k in sorted(hist):
    print(f"  {k:4d}-{k + 50:4d} m: {hist[k]:6d}")


if __name__ == "__main__":
  main()
