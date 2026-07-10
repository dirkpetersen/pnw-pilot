# The PNW era — current operating picture (written 2026-07-10)

This is the newest reference: the state of the workbench after the June→July transformation
(overlay-era → git-checkout + auto-update; xnor `*2xnor` efforts → pnw `*2pnw` production line;
old driving model → lebowski). Prefer this file over older references where they conflict.
Source-of-truth documents: `~/gh/comma/CLAUDE.md`, `~/gh/comma/docs/DEVICE-STATE.md`,
`~/gh/comma/docs/INDEX.md`.

## The distribution

`~/gh/comma/pnw/pnw-pilot` = `dirkpetersen/pnw-pilot`, fork of xnor-tech/openpilot (which adds Tesla
HW1/2/3 Raven support to commaai). Public README carries the feature list + installer URLs
(`installer.comma.ai/dirkpetersen/pnwtest|pnwprod`). Companion forks `pnw-opendbc`/`pnw-panda`
(vendored inline on work branches). Base version lineage: openpilot 0.11.1-era xnor + heavy
selective upstream adoption (see below).

### Channel map (2026-07-10, driver-defined — memorize)
- **`3devpnw`** — the car's channel. Device checked out here; auto-update tracks it. All dev lands here.
- **`4devpnw`** — FROZEN known-good fallback (`261cc6ad90`). Reinstall target when the upcoming Tesla
  opendbc/panda experiments break something. Don't advance casually.
- **`3testpnw`** — FRIENDS' install channel. Never experiment; promote only battle-tested states.
- `3pnw`, `pnwprod` — release line.

## What's on the car (headline features; DEVICE-STATE.md for the param registry)

- **Lebowski driving model** (commaai-master snapshot port, `docs/LEBOWSKI2PNW.md`): combined
  supercombo onnx, git-LFS model blobs (the 1.7 GB `big_*` variant is `.lfsconfig`-fetchexcluded —
  USB-GPU-only), tinygrad build-time compile, ~27 ms / 0% drops on the 3X. First-drive validated
  2026-07-09 (~2 h, flawless stability). The new sleep-prob DM model came with it (= GLARE Layer B);
  the driver-cam BPS ISP fix (= Layer A) came via upstream picks. e2e-longitudinal in forced
  Experimental still leaves gaps behind pulling-away leads — a model behavior, not a bug.
- **CES/VTSC longitudinal stack** (`ces_pnw`/`vtsc_pnw` under `selfdrive/controls/lib/`): chill by
  default, Experimental only when it helps. The July refinement set, each born from a live incident:
  - **tiered `MAP_SPEED_SCALE`** — trust mapd's target on tight curves (1.35×), override on sweepers
    (1.8×), shared `tiered_map_scale()` keeps the MTSC fold and CES sharp-classification in agreement;
  - **lead-pacing gate** — a lead ≤100 m not slower than ego suppresses curve-Experimental entirely
    ("the lead car cannot pull away" — driver directive; VTSC remains the physical cap);
  - **accel-zone gate** — merges/on-ramps (open road + set ≥6 m/s above ego) suppress
    curve-Experimental (a ramp IS a curve; was pinning merges at 39 mph);
  - **A_CRUISE highway bands** raised (0.14 g @ 56 mph, 0.12 g @ 89) from gas-override telemetry.
  - Top-right button: 3-state cycle CES→Exp→Chill; in CES state the icon is LIVE (bleached
    experimental in temporary chill, orange when CES switches). `HideCESDebug` toggles the
    bottom-right debug box (shows by default).
- **DM road-gating** (`DmMode`, `docs/DMROAD2PNW.md`): Default(stock-strict) / Highway(relaxed only
  on freeway or oneWay+lanes≥2, 90 s hold, via mapd mem-param bridge) / Relaxed(3 h/1 h). Driver
  runs Highway. Glare Layer-C knobs are independent and always active.
- **Location services** (`system/location_services/`, display-only NON_ESSENTIAL): police (Waze
  RapidAPI proxy; **Waze-app parity** — no staleness drop, age shown as "(NN min)"; per-report
  forensics in `/data/pnw/location/police_debug.jsonl`), rest areas (**corridor-identity selection**
  via filename-tagged refs matched to live `WayRef` — killed the heading-line flapping), EV chargers
  (2.5 mi near-field bypass; slow-L2 suppressed when a DC-fast exists within 5 mi OF THE L2,
  computed at load via lat-bucket pass).
- **Uploader** (`system/loggerd/uploader.py`): 2-pass to self-hosted S3 (Lambda presign,
  `jh69za4byd` gw, dipeit AWS account). Pass-2 (HD/rlog) is **offroad-only** (driving-time upload
  bursts caused selfdrivedLagging + locationd inputsOK blips — root-caused via qlog). Sidebar:
  GREEN = pass-1 active, BLUE + Mbps = pass-2. `DeferHDVideoUpload` toggle for precious WiFi.
  **Device locator**: every upload_url request carries `local_ip`; the Lambda logs
  `CLIENT_IP src_ip=… local_ip=…` to CloudWatch — the canonical way to find the roaming device.
- **Networking**: priority-WiFi + captive-portal auto-login incl. the MikroTik/OSU "osuvisitor"
  gateway-login variant (`T-<MAC>` username POST to the gateway `/login`; diagnostic at
  `/data/captive_portal_last.json`).
- **mapd**: pfeiferj v2.0.6 binary at `/data/mapd` (outside the tree), cereal bridge
  `system/mapd/mapd_configd.py` → mem params (`RoadContext`, `WayRef`, `MapOneWay`, `MapLanes`,
  `MapTargetVelocities`). Upstream PR submitted to expose the OSM highway class (ramps) —
  `docs/MAPD-RAMP-PROPOSAL.md`, pfeiferj/mapd PR #89; interim ramp handling is the accel-zone gate.
- **Watchdog retune**: `modeldLagging` threshold 1%→5% on the 3X (upstream's 1% was hair-trigger
  with the deep model).

## Selective upstream adoption — the method that works

`docs/UPSTREAM2PNW.md` is the template: survey commaai/master since the merge-base, classify into
clean picks / trivial-overlap / hand-port / excluded (no panda/opendbc bumps; the 2026-06 "nested
openpilot" restructure is a hard cherry-pick boundary — pre-restructure commits pick clean,
post-restructure need path rewrites). For big entangled chains (the lebowski modeld stack), a
**snapshot port** of the whole directory beats cherry-picking, with: import-surface sweep, capnp
minimal-delta (add only truly-new fields; skip wire-compatible refactors), param-key audit
(EVERY `params.get/put` in copied code must be registered — unregistered = crash-loop), re-apply
the fork's own modifications, host compile + on-device build.

## Workflow patterns proven this sprint

- **Live drive monitoring**: persistent Monitor over SSH-on-hotspot sampling ces_events (~22 s) +
  error counters; status line per minute, immediate alerts on modeld respawn/new errors/drive end.
  Deploys queue for `IsOnroad=0` (drive-end event) — pure-python planner changes need no restart
  when parked (onroad processes spawn fresh next drive).
- **Qlog forensics**: `tools/lib/logreader` on-device over recent segment qlogs answers "which alert
  fired and why" (`onroadEvents`), and message-level inspection (e.g. livePose validity flags around
  an event timestamp) names the failing subsystem. swaglog alone often lacks alert events.
- **Scenario-replay tests**: for pure decision modules (ces_pnw/vtsc_pnw), stub the impure imports
  and replay the actual incident telemetry as unit scenarios before shipping. This caught a design
  hole Gemini missed (sharp-curve exception defeating the lead-pacing gate).
- **Post-mortem every deploy verification**: parse logs as JSON (grep-counting "Traceback" on
  `strings` output matches fragments); classify each error to a daemon before reacting.

## Known-benign noise (don't chase these)

- `hardwared` modem eSIM/LPA `AT command failed: ERROR` traceback at boot.
- `athenad` websocket/SSL reconnect tracebacks on cellular handoffs while driving.
- `camerad VIDIOC_CAM_CONTROL op_code 266 errno 19` around ignition transitions.
- A couple of `modeld dropped N frames` at drive start (camera warm-up).
- One-off `locationdTemporaryError` per long night drive (was upload-contention; now rare after the
  offroad-only pass-2 gate — investigate only if clustering).
