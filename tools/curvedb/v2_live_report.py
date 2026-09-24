#!/usr/bin/env python3
"""curvedblive2pnw -- what the LIVE curve DB did, per drive, from ces_events.

Reads ces_events JSONL (plain, .zst or .gz; the device's /data/pnw/ces_events.jsonl* or the S3 pnwlogs archive)
and prints, per drive (records more than 10 min apart start a new one), in Pacific time:

  * liveness: cdb2On states seen, cdb2Err, the rows loaded, A (mapd's lateral target) and its personality;
  * decisions: how many ICBM decisions consulted the DB (cdb2N), records with a raise / lower / add;
  * why the DB did nothing, by reason (cdb2Why) -- a drive that is all "waySel" or "noA" is not a quiet road;
  * sites: every run of consecutive records acting on the same row -- where (lat/lon of the anchor), direction,
    ICBM's own target -> the applied target (mph), the row's curvature and v_db.

Records are ~1 Hz and carry the LATEST ICBM decision (ICBM decides at ~4 Hz); counts of records are therefore
seconds of effect, not decisions -- cdb2N/cdb2NR/cdb2NL are the controller's own decision counters.

Run:
  python tools/curvedb/v2_live_report.py /data/pnw/ces_events.jsonl [more files ...] [--since 2026-09-24]
"""
from __future__ import annotations

import argparse
import datetime
import gzip
import io
import json
import sys
from collections import Counter
from zoneinfo import ZoneInfo

PT = ZoneInfo("America/Los_Angeles")
MPH = 0.44704
DRIVE_GAP_S = 600.0


def read_records(paths):
  """Yield (record) from every file; count what could not be parsed (Rule 2: never a silent skip)."""
  bad = Counter()
  for p in paths:
    if p.endswith(".zst"):
      import zstandard
      with open(p, "rb") as f:
        text = io.TextIOWrapper(zstandard.ZstdDecompressor().stream_reader(f), encoding="utf-8", errors="replace")
        lines = list(text)
    elif p.endswith(".gz"):
      with gzip.open(p, "rt", errors="replace") as f:
        lines = list(f)
    else:
      with open(p, errors="replace") as f:
        lines = list(f)
    for ln in lines:
      try:
        r = json.loads(ln)
      except ValueError:
        bad[p] += 1
        continue
      if isinstance(r, dict) and isinstance(r.get("t"), (int, float)):
        yield r
      else:
        bad[p] += 1
  for p, n in bad.items():
    print(f"WARNING: {n} unparsable line(s) in {p}", file=sys.stderr)


def pt(t):
  return datetime.datetime.fromtimestamp(t, PT)


def drives(recs):
  cur = []
  for r in sorted(recs, key=lambda r: r["t"]):
    if cur and r["t"] - cur[-1]["t"] > DRIVE_GAP_S:
      yield cur
      cur = []
    cur.append(r)
  if cur:
    yield cur


def _inc(xs):
  """Decisions within the drive from a cumulative counter that restarts at every selfdrived start. Counted from
  the drive's first record (decisions before it belong to an earlier drive of the same boot)."""
  tot, prev = 0, xs[0] if xs else 0
  for x in xs:
    tot += x - prev if x >= prev else x
    prev = x
  return tot


def mph(v):
  return None if v is None else v / MPH


def sites(recs):
  """Runs of consecutive records with a DB effect on the same row (or the same direction without a row)."""
  out, cur = [], None
  for r in recs:
    d = r.get("cdb2Dir")
    if d not in ("raise", "lower", "add"):
      cur = None
      continue
    key = (r.get("cdb2Row"), d)
    if cur is None or cur["key"] != key or r["t"] - cur["t1"] > 5.0:
      cur = {"key": key, "t0": r["t"], "t1": r["t"], "n": 0, "dir": d, "row": r.get("cdb2Row"),
             "lat": r.get("cdb2Lat"), "lon": r.get("cdb2Lon"), "k": r.get("cdb2K"), "vdb": r.get("cdb2VDb"),
             "base": [], "tgt": [], "icbmT": [], "vEgo": [], "src": Counter()}
      out.append(cur)
    cur["t1"], cur["n"] = r["t"], cur["n"] + 1
    for k_in, k_out in (("cdb2Base", "base"), ("cdb2Tgt", "tgt"), ("icbmT", "icbmT"), ("vEgo", "vEgo")):
      if r.get(k_in) is not None:
        cur[k_out].append(r[k_in])
    cur["src"][r.get("cdb2Src")] += 1
  return out


def report(recs) -> str:
  lines = []
  for d in drives(recs):
    live = [r for r in d if "cdb2On" in r]
    t0, t1 = pt(d[0]["t"]), pt(d[-1]["t"])
    lines.append(f"\n=== drive {t0:%Y-%m-%d %H:%M}-{t1:%H:%M} PT ({len(d)} records, car {Counter(r.get('car') for r in d).most_common(1)[0][0]})")
    if not live:
      lines.append("  no cdb2* fields at all: this build predates curvedblive2pnw (or the fields evaporated)")
      continue
    on = Counter(r["cdb2On"] for r in live)
    errs = Counter(r.get("cdb2Err") for r in live if r.get("cdb2Err"))
    a = Counter((r.get("cdb2A"), r.get("cdb2Pers")) for r in live)
    rows = Counter(r.get("cdb2Rows") for r in live)
    lines.append(f"  liveness: cdb2On {dict(on)}; rows loaded {dict(rows)}; A {dict(a)}")
    for e, n in errs.items():
      lines.append(f"  ERROR ({n} records): {e}")
    n_dec = [r.get("cdb2N") or 0 for r in live]
    nr = [r.get("cdb2NR") or 0 for r in live]
    nl = [r.get("cdb2NL") or 0 for r in live]
    lines.append(f"  decisions consulting the DB: {_inc(n_dec)} (raises {_inc(nr)}, lowers/adds {_inc(nl)}); "
                 + f"records by direction {dict(Counter(r.get('cdb2Dir') for r in live))}")
    whys = Counter(r.get("cdb2Why") for r in live if r.get("icbmT") is not None or r.get("cdb2Dir") not in (None, "none"))
    lines.append(f"  why-not while ICBM had a target or the DB acted: {dict(whys.most_common())}")
    ss = sites(live)
    if not ss:
      lines.append("  no site where the DB changed ICBM's target")
    for s in ss:
      base = min(s["base"]) if s["base"] else None
      tgt = min(s["tgt"]) if s["tgt"] else None
      chg = None if base is None or tgt is None else mph(tgt) - mph(base)
      v = f"{mph(min(s['vEgo'])):.0f}-{mph(max(s['vEgo'])):.0f}" if s["vEgo"] else "?"
      vdb = "?" if s["vdb"] is None else f"{mph(s['vdb']):.1f}"
      icbm = "none" if base is None else f"{mph(base):.1f}"
      db = "none" if tgt is None else f"{mph(tgt):.1f}"
      parts = [f"  {pt(s['t0']):%H:%M:%S}-{pt(s['t1']):%H:%M:%S} PT {s['dir']:<5} row {s['row']}",
               f"({s['lat']}, {s['lon']}) k {s['k']} v_db {vdb} mph |",
               f"ICBM {icbm} -> DB {db} mph" + ("" if chg is None else f" ({chg:+.1f})"),
               f"| v {v} mph | {s['n']} s | src {dict(s['src'])}"]
      lines.append(" ".join(parts))
  return "\n".join(lines)


def main(argv=None) -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("files", nargs="+")
  ap.add_argument("--since", help="PT date YYYY-MM-DD: only drives starting on or after it")
  a = ap.parse_args(argv)
  recs = list(read_records(a.files))
  if a.since:
    t_min = datetime.datetime.fromisoformat(a.since).replace(tzinfo=PT).timestamp()
    recs = [r for r in recs if r["t"] >= t_min]
  if not recs:
    print(f"NO records read from {len(a.files)} file(s) -- nothing to report (check the paths / --since)")
    return 2
  print(f"{len(recs)} records from {len(a.files)} file(s)")
  print(report(recs))
  return 0


if __name__ == "__main__":
  sys.exit(main())
