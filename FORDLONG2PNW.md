# FORDLONG2PNW — BluePilot highway follow control (LongitudinalExt) on the Lightning

**Status: DEPLOYED on `3devpnw`** (opendbc `master-pnw` `45d5b19c`, pin `e8f35e3e32`; city-hop fix
`1fb73dce`, pin `2ce441fb40` — 2026-07-11/12). **Inert until Alpha Longitudinal is ON** (proven by
test); with Alpha Long OFF the Lightning runs stock ACC + ICBM (see `docs/ICBM2PNW.md` — the
Alpha-Long toggle is the op-long vs ICBM A/B switch). Attribution: mechanism by **alan-polk**
(BluePilot) — see `CREDITS.md`.

## What it is

Port of BP's follow-aware longitudinal shaping (`longitudinal_ext.py` → our
`opendbc/car/ford/longitudinal_ext_pnw.py`) for the op-long path:

- lead classification **gaining / pacing / trailing** with per-state gas/accel limits;
- follow-mode **downward-accel rate limit** with a TTC<8 s emergency bypass;
- **split brake/precharge hysteresis** in the ACCDATA builder (extended `create_acc_msg` in
  `fordcan_pnw.py`);
- **50→45 mph speed deadband** — the BP shaping applies at highway speed only.

Gating (capability view, no fingerprints in feature code): `bp_long_follow` = op_long AND Lightning,
and only alongside a live 4-signal ext (shares its radarState SubMaster). Any exception → permanent
stock-long fallback for the drive (fault-injection tested); `float()` casts on every output (the
`FORDSAFETY2PNW.md` numpy lesson applied preemptively); outputs clipped to the same
ACCEL_MIN/MAX/MIN_GAS as stock — panda ACCDATA limits unchanged.

## Three Gemini-review divergences from BP (adjudicated, fixed in the port)

1. BP zeroes ALL lead-less braking (its phantom-pulse suppression) — that would neuter our VTSC/CES
   curve braking above 50 mph. Divergence: suppress only **mild** decel (> −0.75 m/s²); genuine
   braking passes through (lead-less hard-braking passthrough pinned by test).
2. BP's brake/precharge hysteresis re-inits False every scan → chatter in the −0.14..−0.06 deadband.
   Fixed with a proper cross-scan latch.
3. The follow-mode rate limiter is scoped to lead-present only (moot in BP with no lead; combined
   with fix 1 it would have slewed VTSC decel at 0.1 m/s²/s).

## ⚠️ The city-hop incident (2026-07-12) and the `bp_long_used` gate

Field: city stop-and-go behind a lead in Experimental — the truck was **"hopping like a horse."**
Root cause (`1fb73dce`): with LongitudinalExt merely *constructed*, the BP acc-message builder (with
its narrow −0.14/−0.06 brake hysteresis) owned the brake bit at ALL speeds; below the 50 mph
deadband, gentle city decels sit exactly inside that band → the brake request flapped → physical
brake pulses. Fix: **gate the BP acc message on `lng.bp_long_used`** (BP follow shaping actually
applied this scan); otherwise the stock builder with the stock 0.0/0.3 hysteresis runs —
byte-identical to pre-port city behavior. Lesson: porting a message *builder* alongside a gated
*algorithm* silently widens the blast radius to all speeds unless the builder is gated on the
algorithm actually being in charge.

## Verification

Engaged-path smoke suite extended: op-long sweep through the BP path, fault injection for BOTH exts,
lead-less hard-braking passthrough proof, inert-with-Alpha-Long-off proof. Ford executor suites
unchanged.

## Related

`FORDSAFETY2PNW.md` (4-signal lateral this rides alongside) · `docs/ICBM2PNW.md` (the stock-ACC
alternative arm of the A/B) · `drives/2026-07-12/` (the hop incident drive day) · `CREDITS.md`.
