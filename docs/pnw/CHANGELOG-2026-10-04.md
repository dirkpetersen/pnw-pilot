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

## 2. `mapdcargpsdefault2pnw` (branch, not yet shipped)

Maps use the truck's GPS by default on the Lightning (owner request). The opt-in param `MapdUseCarGps` (default 0, no UI, set by
hand to 1 on the owner's device since 2026-09-19) is replaced by the opt-out `DisableMapdCarGps` (default 0) with a UI row
"Disable Ford GPS for Maps", greyed (never hidden) on a car without `PnwVehicle.car_gps` with the reason "Ford F-150 Lightning only".

- `mapd_configd.py`: `ext_ok = car_gps_capable and not ext_self_feed and (relay already live or not params.get_bool("DisableMapdCarGps"))`. An unreadable
  param still fails safe and loud (relay OFF, logged once). Stop reason is now `DisableMapdCarGps set`. The Tesla is unchanged:
  the relay is gated on the capability and the param is never read there.
- Unchanged on purpose: once mapd latches to `gpsLocationExternal` it cannot fall back until reboot. Therefore the param only
  gates the START of the relay (review fix F1): setting it while the relay is already running does NOT stop it (a stop would
  freeze mapd on its last position with `mapdOut` still publishing, which keeps the nudgeless freeway gate open off-freeway).
  It logs once `DisableMapdCarGps set while relaying -- applies after reboot` and takes effect at the next start of the comma.
  Residual (documented, not built): if `mapd_configd` itself crashes and restarts with the param set, the relay does not resume.
  An unreadable param before the relay starts keeps it OFF with a loud log; an unreadable param while the relay is live KEEPS it
  running (a stop would freeze mapd) and logs once `unreadable while relaying -- relay kept, applies after reboot`.
- Migration: the owner's device has `MapdUseCarGps=1`. The key is gone from `params_keys.h`; the manager's `clear_all` at start
  unlinks files that are not registered keys (`common/params.cc`), so the stale file is deleted, harmless, and the new default
  yields the same behaviour. A device that had it unset or 0 now gets the truck GPS on the Lightning; set `DisableMapdCarGps=1` to opt out.
- Tests: `system/mapd/tests/test_mapd_car_gps_ext.py` (default on, set off, unreadable off + log, not capable off),
  `selfdrive/ui/tests/test_car_gating.py` (greyed on Tesla with the reason, enabled on Lightning, never written).

