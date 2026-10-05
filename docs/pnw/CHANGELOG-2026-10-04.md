# CHANGELOG — 2026-10-04 (Sunday)

All times PT. Channel tip `f53fbcbbfb`, GREEN (6292 passed). Staged on the comma (running `56f242e18d`, needs a reboot).

## 1. `convtesla2pnw` (`f53fbcbbfb`; pnw-opendbc `master-pnw` `0b3d0012`, tests only)

The "Disable Ford Convenience Features" toggle (param `DisableFordConvenience`) is now operable on every car, Tesla and
unknown car included. Reason (owner request): set it in advance so the comma never sends a convenience CAN frame
(Pro Power re-arm, 0x455) the first time it starts in the truck.

- Removed from `CAR_GATED`. This is a documented exception to the "car-specific toggles are greyed on the other car" rule:
  see [`TOGGLE-CONVENTIONS.md`](TOGGLE-CONVENTIONS.md).
- Proof (test): with the param pre-set ON, the opendbc `ConvenienceGate` reads it synchronously in
  `CarController.__init__`, so `ProPowerArmer` is never constructed and no 0x455 frame is sent from the first cycle.
  With it OFF, the first frame comes at least 8 s (`PPO_SETTLE_S`) after start.
- Remaining by-design hole: an unreadable params store fails open, with a loud log line.
- Not Opus-reviewed: UI permission, tests and docs only; no car-behaviour change.
