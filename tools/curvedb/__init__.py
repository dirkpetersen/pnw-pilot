"""curvedb -- CURVEDB2PNW.md Phase 2, OFFLINE half.

`ingest.py`, `replay.py` and `recurrence.py` are OFFLINE ONLY -- imported by `tools/` scripts and by
tests, never by `selfdrive/`, `system/` or `cereal/`.

**`store.py` IS THE ONE EXCEPTION, AND ALWAYS WAS THE INTENTION.** It is imported unchanged by
`selfdrive/controls/lib/ces_pnw/curvedb_shadow.py` (curvedbshadow2pnw), because if the section 7
replay validated one matcher and the car ran a second one the replay would have proved nothing. That
import is the ONLY control-path import of this package, and
`selfdrive/controls/lib/ces_pnw/tests/test_curvedb_read_boundary.py` (L3) fails if another appears.

The on-car half is SHADOW ONLY: it records, predicts and logs what it WOULD have done, and no control
path reads the answer. Phase 2 has no authority and is not deployed. The question the design defers
to data is still open: would a learned curve database have refuted phantom slowdowns without ever
cancelling a real one.
"""
