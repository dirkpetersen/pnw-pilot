# Credits

PNW-pilot stands on the work of these projects and people. Nearly every feature in this
distribution is a port, derivative, or re-implementation of something invented elsewhere in the
openpilot community. Attribution below is based on the git history of the upstream repositories;
GitHub handles were taken from commit author emails (`NNNNN+handle@users.noreply.github.com`)
wherever available.

## Base projects

| Project | GitHub | What it is |
|---------|--------|------------|
| openpilot | [comma.ai](https://github.com/commaai/openpilot) ([@commaai](https://github.com/commaai)) | The base driver-assistance system everything here builds on. |
| xnor-tech openpilot | [@xnor-tech](https://github.com/xnor-tech) | The fork PNW-pilot is directly forked from — Tesla legacy (Raven) support, external/multi-panda integration. |
| sunnypilot | [@sunnypilot](https://github.com/sunnypilot) / [@sunnyhaibin](https://github.com/sunnyhaibin) (Jason Wen) | Community fork; origin of ICBM, DEC, MADS, and the Vision/Map Turn Speed Control lineage we studied. |
| BluePilot | [@BluePilotDev](https://github.com/BluePilotDev) / [@alan-polk](https://github.com/alan-polk) | Ford-focused fork; origin of the Ford lateral improvements and the Ford ICBM port, and collaborator on the 2025 F-150 Lightning fingerprint. |
| FrogPilot | [@FrogAi](https://github.com/FrogAi) | Community fork; origin of the Conditional Experimental Mode concept. |

## Feature-by-feature attribution

### Tesla Raven (HW1/HW2/HW3 legacy) support
- **[@lukasloetkolben](https://github.com/lukasloetkolben)** (Lukas, xnor-tech) — author of the
  Tesla legacy support this distribution ships: "xnor-tech: Tesla legacy & MG support, external
  panda integration" (opendbc) and "xnor-tech: F4 panda support, Tesla/VW MEB ignition detection"
  (panda), plus the openpilot-side multi-panda firmware-query work.
- Robbe Derks (comma.ai) contributed a Tesla radar-interface fix on the xnor line.
- The legacy-Tesla effort builds on years of earlier community Tesla ports (e.g. BogGyver's
  pre-AP/legacy Tesla work, visible in the sunnypilot fork history) and contributors.

### ICBM — Intelligent Cruise Button Management (stock-ACC button control)
- **[@sunnyhaibin](https://github.com/sunnyhaibin)** (Jason Wen, sunnypilot) — invented ICBM;
  landed in sunnypilot 2025-09-18 ("Intelligent Cruise Button Management (ICBM)", sunnypilot PR
  #1242) and iterated on it.
- **[@alan-polk](https://github.com/alan-polk)** (BluePilot) — the Ford ICBM port
  (`opendbc/sunnypilot/car/ford/icbm.py`, first committed 2026-02-05) and substantial follow-on
  work on the ICBM base classes, in both BluePilot and sunnypilot.
- **[@lukasloetkolben](https://github.com/lukasloetkolben)** and
  **[@tonesto7](https://github.com/tonesto7)** (Anthony Santilli) — further ICBM contributions in
  sunnypilot/BluePilot.
- PNW-pilot's Lightning "ICBM" (SET− curve slow-downs via 0x083) is our own re-implementation of
  the same idea, inspired by the above.

### Ford lateral control (predicted-curvature blend, `lateral_curv_ext`, anti-overshoot)
- **[@alan-polk](https://github.com/alan-polk)** (BluePilot) — sole author of
  `lateral_curv_ext.py` and the overwhelming majority of the Ford `carcontroller.py` lateral work
  (including the anti-overshoot logic) on the BluePilot `bp-dev` line.
- Individual contributions there also from **[@tonesto7](https://github.com/tonesto7)** and
  John Christman.

### Conditional Experimental Mode (the concept PNW's CES derives from)
- **[@FrogAi](https://github.com/FrogAi)** (James, "frogsgomoo") — invented Conditional
  Experimental Mode (`frogpilot/controls/lib/conditional_experimental_mode.py` in FrogPilot).
  PNW-pilot's CES (Conditional Experimental Switching) is an independent implementation of the
  same concept.

### Dynamic Experimental Control (DEC — the sunnypilot analog we studied)
- **[@sunnyhaibin](https://github.com/sunnyhaibin)** (Jason Wen) — brought DEC into sunnypilot
  ("Dynamic Longitudinal Control", 2023).
- **Rick Lan** ([dragonpilot-community](https://github.com/dragonpilot-community/dragonpilot)) —
  the DEC decision logic is repeatedly synced from dragonpilot ("DEC: Update logic from
  dragonpilot-community/dragonpilot"), so the underlying logic originates there.
- **[@rav4kumar](https://github.com/rav4kumar)** (Kumar Desai) and
  **[@tonesto7](https://github.com/tonesto7)** — principal maintainers/contributors of the
  sunnypilot DEC controller.

### mapd, OSM speed limits, and Vision/Map Turn Speed Control
- **[@pfeiferj](https://github.com/pfeiferj)** (Jacob Pfeifer) — author of
  [pfeiferj/mapd](https://github.com/pfeiferj/mapd), the Go OSM speed-limit/curvature engine
  PNW-pilot ships (v2.x), and of the curvature-based turn-speed-control approach carried into
  many forks (FrogPilot's mapd is explicitly "PFEIFER - MAPD - Modified by FrogAi").
- **The Move Fast team** ([@move-fast](https://github.com/move-fast)) — original authors of the
  VisionTurnController / TurnSpeedController / SpeedLimitControl stack that sunnypilot imported
  in 2023 ("move-fast: mapd, Speed Limit Control, Vision & Map Turn Speed Control").
- PNW-pilot's VTSC/MTSC is our own implementation of these ideas, with
  **[@FrogAi](https://github.com/FrogAi)** and **[@sunnyhaibin](https://github.com/sunnyhaibin)**'s
  derivatives used as references.

## Thank you

To everyone above, the wider sunnypilot, BluePilot, FrogPilot, dragonpilot, and xnor-tech
communities, and comma.ai for openpilot itself — and all their contributors: thank you.
