# CHANGELOG — 2026-10-03 (Saturday)

All times PT. Channel tip `56f242e18d`, GREEN (6290 passed). Installed on the comma 2026-10-04 about 10:17 PT.

## 1. `stopgo2pnw` (`ccbac3ad98`, `8d3d1ef9de`, `56f242e18d`)

The Tesla (op-long), stopped, hands the stop decision to the lead-aware MPC when the lead is in lane, vision-confirmed,
radar-backed, pulling away (at least 8 m, opened at least 1 m over 1 s). Accel cap 0.8 m/s^2. Kill switch
`curve.json` `tesla.stop_go_handoff` (default ON). Details: [`STOPGO-HANDOFF.md`](STOPGO-HANDOFF.md).

Review history (Opus): first review BLOCK (repeated lunges in a closed-loop sim when the e2e model is right to hold);
fixed with one handoff per stop, an e2e-braking exit and a 5 s cap. Second review SHIP-WITH-CHANGES: the noisy
"lead lost" and "dRel jump" re-arm triggers were removed (radar glitches re-armed the lunge). Then shipped.

Replay of 112 stops (26 engaged): 3 of 4 stuck stops fixed. Behaviour change: behind a creeping lead the car creeps too.

## 2. Test floors

Floors raised: `controls/lib/tests` 685, `ces_pnw/tests` 1680.

## 3. Installed 2026-10-04

The comma runs `56f242e18d` plus a stricter curve table (private data, not in this repo).
