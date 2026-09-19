#!/usr/bin/env python3
"""visionback: corpus sweep for the BOUNDED VISION GIVE-BACK question.

Sweeps BOTH corpora and reports per-file what it found, so a corpus that contributes nothing is an
explicit row rather than a silence (Rule 2):

  A. /home/dp/gh/comma/drives/**/ces*.jsonl[.N|.gz|.zst]   -- the analysis-window corpus
  B. /tmp/arch/ces_events.jsonl.2026*                      -- the CONTINUOUS rotating archive

Outputs
  inventory.json   per-file: bytes, lines, records, parse errors, PT span, per-field non-null count,
                   car mix, ICBM tick count, moving-tick count
  ticks.jsonl      deduped FORD moving ticks (vEgo > 4 m/s) with the fields the analysis needs

Ford-only is deliberate and is NOT a silent filter: ICBM is the Lightning's stock-ACC button-tapper
(it does not exist on the Tesla), and `slKActl` is 0 % live on the Tesla (docs/CURVEDB2PNW.md D1), so
a Tesla row could contribute neither a decision nor a truth curvature. The Tesla record count is
reported per file.
"""
import glob
import gzip
import io
import json
import os
import sys

DRIVES = "/home/dp/gh/comma/drives"
ARCH = "/tmp/arch"
OUT = "/home/dp/gh/comma/_scratch/visionback"

# every field this analysis can consume; presence is counted per file
KEYS = [
  "t", "car", "ev", "vEgo", "vSet", "stockSet", "stockOn", "aEgo",
  # ICBM decision
  "icbmT", "icbmC", "icbmSrc", "icbmOwnT", "icbmPhase", "icbmFlr", "icbmFlrHit", "icbmGate",
  "icbmDir", "icbmSaneT", "icbmSaneWhy", "icbmBehind", "icbmLeadT", "icbmLeadWhy",
  # map candidate
  "mapV", "mapDist", "mapRaw", "mapEff", "mapReach", "mapPts", "mapCandD", "mapLat", "mapLon",
  # map geometry witnesses
  "icbmK", "icbmKD", "icbmKN", "icbmKAt", "icbmKAtD", "icbmKAtN", "icbmKAtGap", "icbmKAhead",
  # VISION witnesses -- the subject of this study
  "icbmKVis", "visLat", "visTtc", "visK", "visV", "visD", "mdlEndX",
  # TRUTH witnesses
  "kPeak", "kPose", "slKActl", "achLat", "slKCmd", "slLatAct",
  # context / disqualifiers
  "strAng", "strPrs", "slAngSat", "slSat", "slCurvLim", "lcGate", "blnk",
  "spdLim", "condSpdLim", "hwy", "hwyClass", "vtscPitch", "lat", "lon", "bearing",
  "hasLead", "dRel", "vLead", "shadow",
]

# Corpora predating the `car` telemetry field (added 2026-07-13); each identified from its own
# DRIVE_REPORT.md header. Copied verbatim from _scratch/icbmslow/scan.py so the two sweeps bucket
# the same files the same way (that table's own history: filtering on `car` silently dropped three
# whole corpora).
PRE_CAR_FIELD = {
  "2026-06-27": "TESLA", "2026-07-01": "TESLA", "2026-07-06": "TESLA", "2026-07-08": "TESLA",
  "2026-07-10/bsm-first-live-test": "TESLA", "2026-07-10/lightning-oplong-first-lap": "FORD",
  "2026-07-11/lightning-icbm-nofire": "FORD",
  "2026-07-12/ellensburg-snoqualmie-westbound": "FORD",
  "2026-07-12/snoqualmie-ellensburg-icbm": "FORD",
  "2026-07-12/tesla-redlight": "TESLA",
  "2026-06-18": "UNKNOWN",   # ces_experimental_transitions.txt -- 0 parseable records
}


def opener(path):
  if path.endswith(".gz"):
    return gzip.open(path, "rt", errors="replace")
  if path.endswith(".zst"):
    import zstandard
    fh = open(path, "rb")
    return io.TextIOWrapper(zstandard.ZstdDecompressor().stream_reader(fh), errors="replace")
  return open(path, errors="replace")


def car_for(rel, rec_car):
  if rec_car:
    s = str(rec_car)
    return "FORD" if s.startswith("FORD") else ("TESLA" if s.startswith("TESLA") else s)
  for pref, car in PRE_CAR_FIELD.items():
    if rel.startswith(pref):
      return car
  return "UNKNOWN"


def enumerate_files():
  out = []
  pats = ["ces*.jsonl", "ces*.jsonl.[0-9]*", "ces*.jsonl.gz", "ces*.jsonl.zst", "ces*.txt"]
  seen = set()
  for p in pats:
    for f in glob.glob(os.path.join(DRIVES, "**", p), recursive=True):
      if f.endswith((".py", "_summary.txt", "stderr.txt")) or f in seen:
        continue
      seen.add(f)
      out.append(("drives", f, os.path.relpath(f, DRIVES)))
  for f in sorted(glob.glob(os.path.join(ARCH, "ces_events.jsonl.2026*"))):
    if f.endswith(".zst") or f in seen:       # the .zst are compressed copies of the same generation
      continue
    seen.add(f)
    out.append(("arch", f, "arch/" + os.path.basename(f)))
  return sorted(out, key=lambda x: x[2])


def main():
  files = enumerate_files()
  inventory, errors, keep = [], [], {}
  for corpus, path, rel in files:
    row = {"corpus": corpus, "file": rel, "bytes": os.path.getsize(path), "lines": 0,
           "records": 0, "parse_err": 0, "t_min": None, "t_max": None,
           "present": {k: 0 for k in KEYS}, "cars": {}, "icbm_ticks": 0,
           "ford_moving": 0, "error": None}
    try:
      with opener(path) as fh:
        for line in fh:
          row["lines"] += 1
          line = line.strip()
          if not line:
            continue
          try:
            r = json.loads(line)
          except Exception:                                    # noqa: BLE001 -- counted, not hidden
            row["parse_err"] += 1
            continue
          if not isinstance(r, dict):
            row["parse_err"] += 1
            continue
          row["records"] += 1
          t = r.get("t")
          if isinstance(t, (int, float)):
            row["t_min"] = t if row["t_min"] is None else min(row["t_min"], t)
            row["t_max"] = t if row["t_max"] is None else max(row["t_max"], t)
          for k in KEYS:
            if r.get(k) is not None:
              row["present"][k] += 1
          c = car_for(rel, r.get("car"))
          row["cars"][c] = row["cars"].get(c, 0) + 1
          if r.get("icbmT") is not None:
            row["icbm_ticks"] += 1
          if isinstance(t, (int, float)) and c == "FORD":
            try:
              v = float(r.get("vEgo") or 0.0)
            except (TypeError, ValueError):
              v = 0.0
            if v > 4.0:
              row["ford_moving"] += 1
              slim = {k: r.get(k) for k in KEYS if k != "t"}
              slim["t"] = t
              slim["_src"] = rel
              slim["_corpus"] = corpus
              keep[t] = slim
    except Exception as exc:            # Rule 2: an unreadable file is an ERROR row, never a zero row
      row["error"] = f"{type(exc).__name__}: {exc}"
      errors.append((rel, row["error"]))
    inventory.append(row)
    print(f"{rel}: rec={row['records']} err={row['parse_err']} icbm={row['icbm_ticks']} "
          f"fordmov={row['ford_moving']} {row['error'] or ''}", file=sys.stderr)

  with open(os.path.join(OUT, "inventory.json"), "w") as f:
    json.dump(inventory, f)
  with open(os.path.join(OUT, "ticks.jsonl"), "w") as f:
    for t in sorted(keep):
      f.write(json.dumps(keep[t]) + "\n")
  print(f"\nfiles={len(files)} unreadable={len(errors)} ford_moving_deduped_ticks={len(keep)}")
  for rel, e in errors:
    print("  ERROR", rel, e)


if __name__ == "__main__":
  main()
