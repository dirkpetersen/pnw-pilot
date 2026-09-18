"""curvedb -- CURVEDB2PNW.md Phase 2, OFFLINE half.

NOTHING IN THIS PACKAGE IS ON THE CONTROL PATH. It is imported by `tools/` scripts and by tests,
never by `selfdrive/`, `system/` or `cereal/`. Phase 2 has no authority and is not deployed; the
package exists so the section 7 go/no-go replay can be run against real corpora and answer the one
question the design defers to data: would a learned curve database have refuted phantom slowdowns
without ever cancelling a real one.

`store.py` is deliberately the module a future on-car consumer would import unchanged -- if the
replay validated one implementation and the car later ran a different one, the replay proved nothing.
"""
