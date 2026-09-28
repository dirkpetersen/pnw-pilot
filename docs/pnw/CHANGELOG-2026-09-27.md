# CHANGELOG — 2026-09-27 (Sunday)

Continues [`CHANGELOG-2026-09-26.md`](CHANGELOG-2026-09-26.md). All times PT.

**Channel tip:** `origin/3devpnw` = `1556124dae`, **GREEN**, 4560 passed. **Installed on the device:**
`1556124dae` — `coopsteer2pnw` (`1fb29be840`) installed 13:40 PT, then `curvebrain2pnw` stages 0+1
(`f4a330ef3f` … `1556124dae`) installed at a later reboot the same day, both while openpilot was
disengaged.

## 1. ✅ `coopsteer2pnw` (`1fb29be840`) — the Tesla Raven cooperative-steer nudge now actuates, default OFF

Previously shadow-only telemetry (`coopsteer-shadow2pnw`, since 09-07): the module computed and logged a
small steering-torque-following offset but nothing reached the actuators. This ships the actuating
version, calibrated against a dedicated light-hand-steering drive (56 engaged minutes, 100 Hz carState +
decoded EPAS torque) that specifically sampled the low-torque range the shadow version had under-sampled.

* **Sign, settled on real data.** Manual driving at meaningful torque and angle: `sign(torque) ==
  sign(angle)` in 99.5–99.8% of samples across two drives (morning n=7,679, afternoon n=6,813) — a torque
  onset near straight is followed by the wheel moving the same way 93–95% of the time. This confirms the
  existing module's assumption (offset = +k·torque pushes *with* the driver) rather than changing it.
* **What a light touch does today:** nothing. The EPS holds the commanded angle through 0.3–1.0 Nm of
  driver torque with lateral still engaged (actual-vs-commanded angle median 0.38°); no light touch dropped
  lateral in the calibration drive. The nudge only operates in that sub-1 Nm range — the 1–4 Nm fight that
  precedes the EPS's own hands-on abort is out of scope (a separate, not-yet-built override-blend problem).
* **Tuning changes from the calibration drive:** the dead zone moved from 0.3 to 0.5 Nm (0.2–0.4 Nm
  opposes the wheel's own motion 84% of the time — that is resting-hand drag, not intent); a 100 ms
  low-pass on the torque feeding the target kills sub-200 ms twitches (34 → 1 in the replay) and roughly
  halves the peak offset rate; a new HOLD behaviour on a same-direction driver override stops the offset
  from being shed at full rate *against* the driver (previously up to ~36° of unwanted pull-back over one
  drive's replay; now zero) while an override *against* the held offset still sheds normally. Washout (5s)
  and slew fractions are unchanged — the drive's longest light push was 1.8s, well inside where 2/3/5s
  washout settings replay identically.
* **Gated by a new `CoopSteer` param, default OFF** (Raven Tesla only; every other car is unaffected).
  Fail-safe: any param-read error reads OFF. **Superseded 2026-09-28:** `CoopSteer` was replaced by the
  `DisableCoopSteer` opt-out (default ON) in v2 — see
  [`CHANGELOG-2026-09-28.md`](CHANGELOG-2026-09-28.md). This entry is left as written for history.
* **Replay of the final module** (open loop, calibration drive): median peak intent offset 1.1°, zero
  spurious degree-seconds, zero fight against the driver, zero retract-against-driver, release under 0.1°
  within a 0.15s median (0.37s max), 7.2° maximum offset.

Reviewed under the standing Fable-before-push policy; channel tip GREEN (4498 passed at this commit).
Installed on the car 13:40 PT.

## 2. 🟡 `curvebrain2pnw` stages 0+1 of 8 (`f4a330ef3f`, `1556124dae`) — shared curve-brain design, first two stages: config and a test harness only

Today's ICBM (Lightning, stock ACC) and VTSC (Tesla, openpilot longitudinal) each price curve slowdowns
independently, with no shared source of "how fast can this car actually take this curve." Design doc:
`docs/SHARED-CURVE-BRAIN-DESIGN.md` ("one shared curve brain, two actuators") lays out an 8-stage plan to
give both a common need layer while keeping the Lightning byte-identical and never slowing the Tesla below
what it does today except in a later, explicitly gated stage. **These first two stages ship no behaviour
change on either car** — they are a test harness and a not-yet-consumed capability.

* **Stage 0 — golden replay harness** (`tools/curvebrain/golden.py`, 18 tests). Drives a real `CESController`
  through its existing curve-pricing code and records one canonical line per tick (the same fields already
  written to on-car telemetry), with a sha256 per corpus, so later stages that move this pricing code into
  a shared module can be checked byte-for-byte against a recorded baseline instead of by inspection. Includes
  a self-test (two recordings are identical; a deliberate tiny constant change is detected) and a coverage
  report over four corpora (real logged ticks, real ces_events with map-path records, a private curve-DB
  replay fixture, and 100k seeded synthetic frames). Tools and tests only — no runtime code path changed.
* **Stage 1 — the per-car lateral-acceleration capability** (`PnwVehicle.curve_lat_a`, a new `"tesla"`
  section in the on-device `curve.json` tuning file, and a `curve_brain` mode). The Lightning's value (2.5
  m/s², used only by future stages — ICBM keeps every one of its own existing tuning knobs unchanged) and a
  Tesla-only default of 2.8 m/s², capped below openpilot's own lateral-acceleration ceiling so the target
  never asks for more than the steering can deliver. **Nothing reads this new capability yet** — it is
  parsed, validated, logged at startup, and otherwise inert. The `curve_brain` mode defaults to "shadow" on
  the Tesla and "off" on every other car. A parsing bug caught in review (a malformed tuning value could
  have raised an exception that crash-looped the whole process on the Tesla) was fixed before shipping.

Both stages passed the full test suite unchanged plus their own new tests, and the golden replay showed
zero differences on the existing Lightning behaviour. **The Tesla's recommended 2.8 m/s² lateral target is
a default only — the owner has not yet signed off on that number**, nor on any of the design's other open
rollout decisions; both are tracked in `docs/work-pending/2026-09-24-curvedb-for-tesla-vtsc.md` (private
workbench docs) pending a calibration drive. Config and start-up telemetry only; installed.
