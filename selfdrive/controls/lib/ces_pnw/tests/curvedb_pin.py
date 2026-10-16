"""The FROZEN curve DB table the real-table tests read (private data: the table holds positions, this repo is public).

The tests pin exact per-row prices, so they must not read the live table (`workdir/data/curvedb_v2`), which the nightly
auto-update rewrites. They read the frozen copy `workdir/data/curvedb_v2/` and assert its sha256 here: a mismatch
FAILS LOUDLY naming both shas; an absent directory SKIPS with the path in the reason (other machines have no private data).
"""
import hashlib
import os
import pathlib

from openpilot.selfdrive.controls.lib.ces_pnw import curvedb_live as cl

TABLE = pathlib.Path(os.path.expanduser("~/gh/comma/workdir/data/curvedb_v2.test-pin"))
PIN_SHA256 = "f02282e3d26429ee82bbe2eb87649269a97415e750382ffca0cb1aceb9adbd40"     # curvedb_v2_rows.json.zst
TABLE_SKIP = f"private frozen curve DB table not present at {TABLE}"


def verify_pinned_table() -> None:
  got = hashlib.sha256((TABLE / cl.ROWS_NAME).read_bytes()).hexdigest()
  msg = f"the frozen test table {TABLE} is sha256 {got}, but the tests are pinned to {PIN_SHA256}. "
  msg += "Restore the frozen copy (workdir git d3521db), or move the pin deliberately: update PIN_SHA256 AND the expected numbers together. "
  msg += "This is NOT skipped on purpose."
  assert got == PIN_SHA256, msg


def load_pinned_rows():
  verify_pinned_table()
  return cl.load_rows(str(TABLE))[0]
