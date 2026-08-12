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
| StarPilot | [@firestar5683](https://github.com/firestar5683) | Community openpilot fork; origin of the curvature-nudge lane-centering PNW-pilot ships. |

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

### Ford longitudinal follow control (`longitudinal_ext`)
- **[@alan-polk](https://github.com/alan-polk)** (BluePilot) — author of `longitudinal_ext.py`
  (lead classification gaining/pacing/trailing, per-state gas/accel shaping, split
  brake/precharge hysteresis, highway speed deadband), ported to PNW as
  `longitudinal_ext_pnw.py` (fordlong2pnw) with three PNW-specific integration fixes noted
  in that file.

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

### Green Light Alert
- **[@sunnyhaibin](https://github.com/sunnyhaibin)** (Jason Wen) and the **sunnypilot**
  contributors — the core mechanics PNW-pilot adopted
  (`sunnypilot/selfdrive/controls/lib/e2e_alerts_helper.py`): the armed/consumed one-ding-per-stop
  state machine, the model-trajectory-endpoint (>30 m) release trigger, the 0.3 s sustained-trigger
  debounce, and the 2 s not-recently-moving arming guard.
- **[@FrogAi](https://github.com/FrogAi)** (James) — FrogPilot's Green Light Alert
  (`frogpilot/controls/lib/frogpilot_events.py`), from which we adopted stop-context arming (only
  after the model actually held a stop) and the alert-regardless-of-engagement behavior, and whose
  lead handling informed our "suppress while a close lead is still stopped, ding once it departs"
  rule. Implementation: `selfdrive/controls/lib/ces_pnw/green_light.py` (adjudication documented
  in its docstring).

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

### Lane centering (curvature-nudge)
- **[@firestar5683](https://github.com/firestar5683)** (StarPilot) — author of StarPilot's
  curvature-nudge lane centering (`selfdrive/controls/lib/lane_centering.py`): a **car-agnostic**
  curvature-layer correction that nudges toward true lane center from the model's lane lines, with a
  gain cap (`_MAX_GAIN`), offset/center-error deadband, low-confidence and signal-loss release taus,
  and an e2e-path-std break-in. PNW-pilot ports it as `lanecenter2pnw` — same math, plus a
  hard-clamped safety envelope, live JSON tuning at `/data/pnw/lanecenter_tuning.json`, and a single
  `DisableLaneCentering` opt-out (ships ON). Implementation:
  `selfdrive/controls/lib/lane_centering.py` on the PNW line.

## Thank you

To everyone above, the wider sunnypilot, BluePilot, FrogPilot, dragonpilot, StarPilot, and xnor-tech
communities, and comma.ai for openpilot itself — and all their contributors: thank you.
