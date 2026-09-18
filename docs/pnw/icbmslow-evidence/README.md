# icbmslow2pnw — evidence appendix

Raw output of every analysis step behind [`../ICBMSLOW2PNW.md`](../ICBMSLOW2PNW.md), committed so the
numbers in that document are auditable from the repo. Regenerated 2026-09-17.

| file | what it is |
|---|---|
| `out_a1_capability.txt` | per-corpus capability: Lightning moving ticks, ICBM-commanding ticks and their sources, and which curvature witness each corpus carries. **A corpus that contributes nothing has a row saying so** — the whole point of the table. |
| `out_a2_curvature.txt` | can `strAng` stand in as a curvature witness for the pre-2026-08-11 corpora? Sign agreement 99.2 %, but p90 relative error 0.60 even with a fitted understeer gradient → **no**, not for a lateral-accel claim. |
| `out_a3_model.txt` | validation of the forward model of the ICBM map path against the logged `icbmT`, per corpus. Median residual +0.03…+0.10 m/s from 2026-08-12 on; −1.7…−6.0 before, which IS the `icbmcurve2pnw` scale change. |
| `out_a5_replay.txt` | the episodes, the apex-window sensitivity, the lateral-accel table, and every knob counterfactual replayed through the shipped code (including the `lost` column: how many slowdowns a variant deletes outright). |
| `out_a6_driverpref.txt` | what lateral accel this truck actually gets driven at, with ICBM silent — the anchor for "2.5 m/s² is the top of the envelope, not a target". |
| `out_a7_washouts.txt` | the 2026-07-11 washout sites replayed against today's ICBM. ⚠️ This run predates the registry regeneration, so it covers the **original 27** binding clusters; the three ICBM-live clusters the regenerated 35 added are analysed tick-by-tick in `ICBMSLOW2PNW.md` instead. |

The harness itself is in the workbench at `/home/dp/gh/comma/_scratch/icbmslow/` (scripts `scan.py`,
`a1`…`a7`, `mutate.py`, plus a `README.md` recording the three method errors it caught in itself).
It is not committed because it depends on the full 2.2 GB `drives/` corpus, which is not in the repo.
