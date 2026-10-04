---
updated: 2026-10-04
status: current
---

# Stop-and-go handoff (`stopgo2pnw`, Tesla op-long)

Shipped 2026-10-03 (`ccbac3ad98`, `8d3d1ef9de`, `56f242e18d`). Tesla (Raven) with openpilot longitudinal only.

## Problem

At a stop, the end-to-end model sometimes keeps holding the car after the lead has clearly pulled away ("not resuming").
A replay of 112 stops found most "not resuming" cases (86) were op-long simply OFF (driver manual, brake cancelled
cruise, gas and steer); those are not this feature's job. The Tesla cannot re-engage longitudinal by itself after a
brake press: panda safety requires the driver's stalk, and a spoofed stalk frame would collide with the real sender.
This feature covers the remaining case: op-long ON, stopped, lead gone.

## Rule

While stopped, the stop decision is handed to the lead-aware MPC when ALL hold:

- the lead is in lane, vision-confirmed and radar-backed;
- the lead is pulling away: at least 8 m away and opened at least 1 m over 1 s.

Limits while handed off: acceleration cap 0.8 m/s^2; ONE handoff per stop; at most 5 s.

The handoff ends (re-latches to the model) on: lead lost, lead off-lane, gap shrinking, lead stops, brake, forceDecel,
vEgo >= 1.3 m/s, or the e2e model braking (vEgo > 0.3 and e2e accel < -0.3).
It re-arms only after the lead has stopped for 0.5 s, ego is above 3 m/s, the driver presses gas, or op-long or the
switch goes off. Radar glitches ("lead lost", distance jumps) deliberately do NOT re-arm it: in a closed-loop sim they
caused repeated lunges when the model was right to hold.

## Telemetry

New memory param `StopGoStatus`; per-tick fields in the CES event log: `eng`, `lAct`, `e2eStop`, `mpcA`, `mpcStop`,
`stopHand`, `stopHandWhy` (why the handoff started or ended).

## Kill switch

`curve.json`, key `tesla.stop_go_handoff` (default ON). Set `false` to restore the previous behaviour.
There is no on-screen toggle yet (open owner question).

## Known behaviour change

Behind a creeping lead the car now creeps too (about 10.5 m gap, ends about 6 m behind, never under 4 m), and it follows
a left-turner once the turner is into the crosswalk.

Replay (112 stops, 26 engaged): 3 of 4 stuck stops fixed; the fourth (vision lost the lead for 3 s, longer than the 2 s
grace) is not.
