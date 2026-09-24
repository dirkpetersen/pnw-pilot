"""curvedblive2pnw: tools/curvedb/v2_live_report.py reads what the car writes (plain and .zst) and finds the sites."""
import json

import zstandard

from openpilot.tools.curvedb import v2_live_report as rep

MPH = 0.44704
T0 = 1790050800.0          # 2026-09-21 21:20 PT


def _rec(i, **kw):
  r = {"t": T0 + i, "car": "FORD_F_150_LIGHTNING_MK1", "vEgo": 33.0, "icbmT": None, "cdb2On": "ok", "cdb2Err": None,
       "cdb2Rows": 17158, "cdb2A": 2.2, "cdb2Pers": "standard", "cdb2N": 4 * i, "cdb2NR": 0, "cdb2NL": 0,
       "cdb2Why": "idle", "cdb2Dir": None, "cdb2Src": None, "cdb2Base": None, "cdb2Tgt": None, "cdb2Row": None,
       "cdb2K": None, "cdb2VDb": None, "cdb2D": None, "cdb2Lat": None, "cdb2Lon": None}
  r.update(kw)
  return r


def _drive():
  recs = [_rec(i) for i in range(5)]
  recs += [_rec(5 + i, icbmT=28.3, cdb2Why="ok", cdb2Dir="raise", cdb2Src="far", cdb2Base=21.6, cdb2Tgt=28.3,
                cdb2Row="812:0", cdb2K=0.00191, cdb2VDb=33.9, cdb2Lat=47.001, cdb2Lon=-122.001, cdb2NR=4 * i + 1)
           for i in range(6)]
  recs += [_rec(11 + i, icbmT=30.97, cdb2Why="ok", cdb2Dir="add", cdb2Src=None, cdb2Tgt=30.97, cdb2Row="933:0",
                cdb2K=0.002293, cdb2VDb=30.97, cdb2Lat=47.002, cdb2Lon=-122.002, cdb2NR=21, cdb2NL=4 * i + 1)
           for i in range(3)]
  recs += [_rec(3000 + i, cdb2On="err", cdb2Err="missing /data/pnw/curvedb_v2/manifest.json") for i in range(3)]
  return recs


def test_sites_and_drives(tmp_path):
  p = tmp_path / "ces_events.jsonl"
  p.write_text("".join(json.dumps(r) + "\n" for r in _drive()) + "{broken\n")
  out = rep.report(list(rep.read_records([str(p)])))
  assert out.count("=== drive") == 2
  assert "raise row 812:0" in out and "ICBM 48.3 -> DB 63.3 mph (+15.0)" in out
  assert "add   row 933:0" in out and "ICBM none -> DB 69.3 mph" in out
  assert "ERROR (3 records): missing /data/pnw/curvedb_v2/manifest.json" in out


def test_zst_input(tmp_path):
  p = tmp_path / "ces_events.jsonl.20260922T000000Z.zst"
  raw = "".join(json.dumps(r) + "\n" for r in _drive()).encode()
  p.write_bytes(zstandard.ZstdCompressor().compress(raw))
  assert len(list(rep.read_records([str(p)]))) == len(_drive())


def test_nothing_read_is_an_error(tmp_path):
  p = tmp_path / "empty.jsonl"
  p.write_text("")
  assert rep.main([str(p)]) == 2
