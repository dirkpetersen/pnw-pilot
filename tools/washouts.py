#!/usr/bin/env python3
"""descentcurve2pnw: washout regression registry generator.

Scans drive telemetry (drives/*/lightning-*/ces_events*.jsonl — the per-second CES event stream)
for HIGH-SPEED STEERING-OVERRIDE CLUSTERS on the Lightning (the "washout" signature: the driver
grabs the wheel at speed because the truck entered a curve too fast) and emits a JSON fixture:
one record per cluster with GPS, entry speed, the recorded binding cap (VTSC applied cap if
op-long, else the ICBM target), and the curve direction.

This is the driver's "database of trouble spots" — used ONLY for VALIDATION of the generic fix
(the unit test in selfdrive/controls/lib/ces_pnw/tests/test_washout_registry.py asserts the new
penalty pipeline demands a meaningfully lower entry speed at every recorded washout). It is NEVER
read by control code.

Direction convention: strAng > 0 => "left". The recorded `strAng` is carState.steeringAngleDeg,
which on this truck is LEFT-positive (openpilot's convention; the sign is unchanged since July).
Before 2026-09-24 this said strAng < 0 = left -- WRONG: it mislabelled 28 of the 35 binding
2026-07-11 washouts, including the 18:12 PT one cited as a "downhill left" (it was a right-hander),
and that false premise is what justified the Lightning left_factor (removed 2026-09-24). Evidence:
drives/2026-09-24/vision-left-flag-check.md and a recount against the device's GPS `bearing`.

Usage:
  python3 tools/washouts.py /home/dp/gh/comma/drives \
      --out selfdrive/controls/lib/ces_pnw/tests/washouts_2026_07_11.json [--date 2026-07-11]
"""
import argparse
import glob
import json
import os
import sys

MPH_TO_MS = 0.44704
MIN_SPEED_MS = 55 * MPH_TO_MS   # "at speed": >55 mph, same threshold as the drive report analysis
CLUSTER_GAP_S = 3.0             # ticks within this gap belong to the same override cluster


def _load_folder_records(folder: str, skipped: dict[str, int] | None = None) -> list[dict]:
  """All records from every ces_events*.jsonl in one drive folder, deduped by timestamp (the
  evening/final files overlap — same drive, re-pulled later) and time-sorted.

  cesretain2pnw: also picks up ROTATED generations. On-device rotation names them
  `ces_events.jsonl.1` .. `.N`, which the `*.jsonl` glob alone does not match — so a multi-day trip
  pulled off the device generation-by-generation was silently ignored here. Dedup by timestamp makes
  the overlap between a copied live file and its own rotated copy harmless.

  rule2fixes2pnw (Rule 2): unparseable lines were skipped without a count. They are still skipped, but
  counted per file into `skipped` (path -> count, when given) and warned about on stderr."""
  by_t: dict[float, dict] = {}
  paths = set(glob.glob(os.path.join(folder, "ces_events*.jsonl")))
  paths |= set(glob.glob(os.path.join(folder, "ces_events*.jsonl.[0-9]*")))
  for path in sorted(paths):
    bad = 0
    with open(path) as f:
      for line in f:
        try:
          r = json.loads(line)
        except json.JSONDecodeError:
          bad += 1
          continue
        t = r.get("t")
        if isinstance(t, (int, float)):
          by_t[float(t)] = r
    if bad:
      print(f"WARNING: {path}: skipped {bad} unparseable line(s)", file=sys.stderr)
    if skipped is not None:
      skipped[path] = bad
  return [by_t[t] for t in sorted(by_t)]


def _clusters(records: list[dict]) -> list[dict]:
  """Group consecutive at-speed steering-override ticks into washout clusters."""
  out: list[dict] = []
  cur: dict | None = None
  for r in records:
    v = float(r.get("vEgo") or 0.0)
    if not (r.get("strPrs") and v > MIN_SPEED_MS):
      continue
    if cur is not None and float(r["t"]) - cur["t_end"] <= CLUSTER_GAP_S:
      cur["t_end"] = float(r["t"])
      cur["n"] += 1
      cur["v_entry_ms"] = max(cur["v_entry_ms"], v)
      cur["angs"].append(float(r.get("strAng") or 0.0))
      for k, key in (("vtscCap", "caps"), ("icbmT", "icbm")):
        if r.get(k) is not None:
          cur[key].append(float(r[k]))
    else:
      if cur is not None:
        out.append(cur)
      cur = {
        "t": float(r["t"]), "t_end": float(r["t"]), "n": 1, "v_entry_ms": v,
        "angs": [float(r.get("strAng") or 0.0)],
        "caps": [float(r["vtscCap"])] if r.get("vtscCap") is not None else [],
        "icbm": [float(r["icbmT"])] if r.get("icbmT") is not None else [],
        "gps": [r.get("lat"), r.get("lon")],
        "shadow": bool(r.get("shadow")),
      }
  if cur is not None:
    out.append(cur)
  return out


def _emit(cluster: dict, drive: str, idx: int) -> dict:
  # recorded binding cap: the lowest VTSC applied cap in the cluster (op-long drives), else the
  # lowest ICBM target (stock-ACC drives), else None (no longitudinal authority was active).
  cap = min(cluster["caps"]) if cluster["caps"] else (min(cluster["icbm"]) if cluster["icbm"] else None)
  # dominant steering direction over the cluster (strAng > 0 = LEFT, steeringAngleDeg convention)
  mean_ang = sum(cluster["angs"]) / len(cluster["angs"])
  direction = "left" if mean_ang > 0 else ("right" if mean_ang < 0 else "")
  v_entry = round(cluster["v_entry_ms"], 2)
  return {
    "id": f"{drive}#{idx:02d}",
    "t": cluster["t"],
    "gps": [round(cluster["gps"][0], 5) if cluster["gps"][0] is not None else None,
            round(cluster["gps"][1], 5) if cluster["gps"][1] is not None else None],
    "v_entry_ms": v_entry,
    "cap_ms": round(cap, 2) if cap is not None else None,
    # binding = the recorded cap sat BELOW the truck's actual speed (the washout signature: the
    # truck arrived over the cap). Only binding washouts drive the regression assertion.
    "binding": bool(cap is not None and cap < v_entry),
    "dir": direction,
    "mean_str_ang": round(mean_ang, 1),
    "n_ticks": cluster["n"],
    "shadow": cluster["shadow"],
  }


def main() -> None:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("drives_root", help="e.g. /home/dp/gh/comma/drives")
  ap.add_argument("--out", required=True, help="output fixture JSON path")
  ap.add_argument("--date", default=None, help="only scan drives/<date>/ (default: all dates)")
  args = ap.parse_args()

  pattern = os.path.join(args.drives_root, args.date or "*", "lightning-*")
  washouts = []
  skipped: dict[str, int] = {}
  for folder in sorted(glob.glob(pattern)):
    if not os.path.isdir(folder):
      continue
    drive = os.path.relpath(folder, args.drives_root)   # e.g. 2026-07-11/lightning-icbm-nofire
    recs = _load_folder_records(folder, skipped)
    for i, c in enumerate(_clusters(recs), 1):
      washouts.append(_emit(c, drive, i))

  fixture = {
    "comment": "descentcurve2pnw washout registry — validation-only, never read by control code",
    "min_speed_mph": 55, "cluster_gap_s": CLUSTER_GAP_S,
    "dir_convention": "strAng>0 = left (carState.steeringAngleDeg, left-positive; fixed 2026-09-24)",
    "washouts": washouts,
  }
  with open(args.out, "w") as f:
    json.dump(fixture, f, indent=1)
    f.write("\n")
  binding = sum(1 for w in washouts if w["binding"])
  n_bad = sum(skipped.values())
  print(f"{len(washouts)} washout clusters ({binding} with a binding recorded cap) -> {args.out}; " +
        f"scanned {len(skipped)} file(s), skipped {n_bad} unparseable line(s)")
  if n_bad:
    print(f"WARNING: {n_bad} unparseable line(s) skipped in total -- see the per-file warnings above", file=sys.stderr)


if __name__ == "__main__":
  main()
