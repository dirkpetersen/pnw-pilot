# CHANGELOG — 2026-09-26 (Saturday)

Continues [`CHANGELOG-2026-09-24.md`](CHANGELOG-2026-09-24.md) (there was no 09-25 changelog file; that day's
one ship, `teslayaw2pnw`, is recorded below). All times PT.

**Channel tip:** `origin/3devpnw` = `15ec45a7d5`, **GREEN**, 4373 passed. **Installed on the truck:**
`3376696a42` (`teslayaw2pnw`), installed 09-25. `curveshape2pnw` is on the channel tip but **not yet
installed** — it will install at the next reboot while openpilot is disengaged.

## 1. ✅ `teslayaw2pnw` (`3376696a42`, pnw-opendbc `4069b86198`) — the Raven Tesla now has a real yaw rate

Before this, every yaw-derived number on the Tesla Model S (HW3/Raven) — the steering telemetry fields
`kActl`/`kErr`/`achLat`/`peakAchLat`, and the CAN half of the CES telemetry's `kPeak` — was a confident
**0.0**, meaning "driving perfectly straight," 7,752 of 7,753 ticks on one drive. The car had no yaw rate
signal decoded at all, so "no data" and "dead straight" were indistinguishable.

The companion `pnw-opendbc` change decodes `carState.yawRate` on the Raven from a chassis-bus CAN message
(positive = left). Checked against 7 real drives' onboard pose estimate: correlation 0.84–0.9996, scale
0.89–1.008, RMS error 0.11–0.23 deg/s, and the sign agrees with the steering angle on essentially every
turning sample.

On any car that still has no yaw signal, the affected telemetry fields now publish **`None`** instead of a
misleading `0.0`, with one log line naming the car at startup. **Telemetry only** — nothing in the control
path reads yaw rate on the Tesla, so this does not change how the car drives. Reviewed under the standing
Fable-before-push policy.

## 2. 🟡 `curveshape2pnw` (`5eb7692cbc` … `15ec45a7d5`) — pricing a highway curve from its own measured shape, SHADOW only

**On the channel tip; NOT YET installed on the truck.** Fable verdict: **SHIP-WITH-FIXES** — the requested
fixes are already applied at the tip commit below.

Today's ICBM map-curve pricing (mapd's posted-curve rating × two calibration multipliers) occasionally
disagrees badly with the road's own shape: on one Oregon curve it inflated a curve mapd had rated
correctly into something so fast the candidate was thrown out entirely, and the truck carried far more
speed into the bend than it should have.

This ships a new pricing path, Lightning-only, that works from the curve's own measured geometry instead of
just mapd's rating:

* **Measured-shape pricing.** ICBM now also has access to a price derived from the road's own polyline
  curvature blended with mapd's own curvature (not just mapd's speed rating): roughly
  `v = sqrt(2.5 / mean(polyline curvature, mapd curvature))`. This measured price is only used at all when
  the two independent curvature readings **agree within a ×1.35 band** and mapd's own rating is **at least
  50 mph** — outside that band, or below 50 mph, today's existing pricing is left completely unchanged.
  Raises are capped; lowerings are exact.
* **Shadow by default.** The new stage runs and logs what it *would* have priced on every tick (new `shp*`
  telemetry fields), but a Lightning running the default configuration drives **byte-identically** to before
  this shipped — verified in tests. Going live is a single config-file flag
  (`/data/pnw/curve.json`, `{"lightning": {"icbm_shape": "live"}}`), not yet flipped.
  Two matching tuning values (the target lateral acceleration used above and below 70 mph) are also only
  reachable through that same file, both defaulting to the same value used in the formula above.
* **Curve-geometry telemetry, on every car.** A separate, always-on piece (`mapdpathlog2pnw`) now logs
  mapd's own curve-candidate path into the device's telemetry whenever its geometry changes (bounded, at
  most a couple hundred points, only on an actual shape change — a speed-only update doesn't re-log it).
  This runs on the Tesla too and is what let the new pricing be checked offline against real driving before
  any of it went live.
* **Offline replay harness.** A new tool (`tools/curveshape/`) reruns the exact pricing and persistence code
  against real recorded driving telemetry, so the new pricing logic can be checked against history before
  it's ever switched on. Seven pass/fail checks (no unwanted change on curves the current code already
  handles correctly, no regression on curves it doesn't touch, and the target curve actually getting caught)
  all pass at the ×1.35 band. That band was tightened down from an initially wider ×1.5 after the harness
  caught one westbound curve where the wider band would have under-priced a real curve with no lead vehicle
  ahead to mask the effect.

Net effect once shipped in shadow: **nothing changes for the driver yet.** The target curve is now caught in
shadow mode (would bind roughly 200 m out, versus being discarded before), with a handful of new shadow-only
slowdowns elsewhere, most of them no-ops (the car was already going that speed) and none of them false
positives in the replay's judgment. Going live is a deliberate, separate step after a couple of real shadow
drives.

Design doc: `docs/CURVE-MEASURED-SHAPE-DESIGN.md`.
