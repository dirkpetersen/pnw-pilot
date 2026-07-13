# PNW design docs

PNW-specific feature/effort design docs (moved here from the repo root 2026-07-13 to declutter).
The user-facing feature list is [`../../PNW-PILOT-FEATURES.md`](../../PNW-PILOT-FEATURES.md); these are
the deeper per-feature design/rationale docs.

| Doc | Covers |
|-----|--------|
| [CES_I90.md](CES_I90.md) | I-90/Snoqualmie curve-braking learnings; CES→Experimental over-brake fix; map-curve braking on by default |
| [CURVESLOW2PNW.md](CURVESLOW2PNW.md) | Low-speed / city curve slowdown work |
| [SHARPCURVE2PNW.md](SHARPCURVE2PNW.md) | Earlier/smoother blind-sharp-curve VTSC (full map lookahead, regen-coast, apex timing) |
| [LOCATION2PNW.md](LOCATION2PNW.md) | "Happening Ahead" display overlay — police / rest areas / EV chargers |
| [LEBOWSKI2PNW.md](LEBOWSKI2PNW.md) | Driving-model snapshot-port (lebowski) method + deploy |
| [DMROAD2PNW.md](DMROAD2PNW.md) | Road-gated driver-monitoring timeout selector (`DmMode`) |
| [DM-VARIABLE.md](DM-VARIABLE.md) | Variable driver-monitoring design |
| [FORDLONG2PNW.md](FORDLONG2PNW.md) | Ford Lightning longitudinal (op-long / ICBM) work |
| [FORDSAFETY2PNW.md](FORDSAFETY2PNW.md) | Ford panda safety-C changes (4-signal lateral, reset-latch hardening) |
| [UPSTREAM2PNW.md](UPSTREAM2PNW.md) | Upstream (commaai) sync / rebase notes |

See also the workbench-wide catalog at `~/gh/comma/docs/INDEX.md`.
