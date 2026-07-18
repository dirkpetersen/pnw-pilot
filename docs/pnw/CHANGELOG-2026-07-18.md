# CHANGELOG — 2026-07-12 (evening) → 2026-07-18

Written **Saturday 2026-07-18**. Continues `CHANGELOG-2026-07-12.md`, which closed at pnw-pilot
`f6db6bd816` (ces2core2pnw shadow merge, midday 07-12) / opendbc `1fb73dce`. Everything below is the
week that followed. Repos: **pnw-pilot** (channel `3devpnw`), **pnw-opendbc** (`master-pnw`),
**pnw-panda** (zero commits this window). Drive reports referenced live under `../drives/<date>/`.

**Tips at time of writing:** `origin/3devpnw` = `8fff658a81` (07-16, deployed + verified on the 3X).
⚠️ Repo-state note: the **local** `3devpnw` branch sits behind at `3968f6fe5c` (07-15) — the 07-16
work was pushed to the channel from branch `speedlimitdebug2pnw` without fast-forwarding the local
branch. opendbc `master-pnw` tip `3f6fd4b0` (07-13).

## TL;DR (the week in six lines)

1. **Red-light-lurch war won in stages**: stophold guards (07-12) → standstill latch + green/lead ding
   split (07-13) → gentle standstill launch + rolling-stop bind (07-14). Lurch root-caused from S3
   qlogs while the truck was unreachable at a campsite.
2. **A deploy broke the truck (07-13 commIssue cascade)** — live 10 Hz diagnosis, bisect to the broken
   build, reland-except-uploader; birthed the **params-only background-proc rule** (uploader re-arch)
   and the mapd pin/watchdog hardening.
3. **speedadjust2pnw** (auto speed reduce for limits+police) shipped OFF→ON with two field fixes (the
   Corvallis below-limit cap; the "wild horse" no-limit/slew fixes).
4. **tightfollow2pnw shipped → crashed plannerd on the road → fixed → REVERTED on data** (07-16); real
   root cause identified: the radarless Lightning's vision-lead path is unfiltered → `radarless2pnw`
   KF filter written (NOT deployed). Plus the plannerd `int(capnp enum)` crash lesson.
5. **Waze police went keyless** (wazeproxy2pnw edge proxy), CES got the dark-cockpit overlay + cards,
   rain mode, auto-recalibrate-on-car-swap, Ford lateral status indicators.
6. **EV ops lessons codified**: `IsOnroad` lies while parked+charging → `gearShifter=park` is the only
   valid parked gate (docs + deploy skill + memory); pre-drive log-save rule reaffirmed the hard way.

---

## 2026-07-12 evening (after the previous changelog's close)

All merged to `3devpnw`:

| Commit | Feature | What it does |
|--------|---------|--------------|
| `c5dbd89f97` (merge, impl `c579bb8786`) | **stophold2pnw** | Red-light lurch guards: A1 lead-present stop + A2 standstill hold + CES liveness badge + clockBad telemetry. Direct response to the Tesla red-light lurch (`drives/2026-07-12/tesla-redlight/CES_SILENCE_REPORT.md` — verdict: CES was never "silent"; the lurch was CES-mediated, a `decide_active` hole letting Chill adopt at 0.4 m/s behind a creeping lead). |
| `caccb03aef`→`c84680aee5`→`e65b671d89`/`138a3b2843` | **wazeproxy2pnw** | Keyless Waze police source via the AWS comma-connect caching edge proxy; proxy-primary with direct-key fallback; live endpoint. Removes the per-device RapidAPI key dependency. |
| `d3c34acc94` + `0e19b44e97` | **cesui2pnw** | "Dark cockpit" CES overlay redesign — tiered moving view (2 lines when quiet), health dots, standstill dump; None-hardening pass (Gemini blocker fixed). |
| `48007b779e`/`0390f1caa3` | **rain2pnw** | Driver-selected wet-weather curve slowdown (None/Light/Heavy; new param `RainMode`; magnitudes tunable via `/data/pnw/rain.json`). |
| `83090418b9` | **calswap2pnw** | Auto-recalibrate when the device moves between the two cars (new param). |
| `720d5eb29f` (+ opendbc `402b8aa4`) | **fordlatui2pnw** | Right-side overlay indicators (calibration / 4-signal lateral / rain); Ford carcontroller publishes `FordLatStatus`. |

**Drives:** `snoqualmie-ellensburg-icbm/` (two control regimes in one log: op-long/VTSC era then
stock-ACC/ICBM), `ellensburg-snoqualmie-westbound/` (route-note: telemetry shows a Hyak/Keechelus
out-and-back, not an Ellensburg leg), `tesla-redlight/CES_SILENCE_REPORT.md` (see stophold above).

## 2026-07-13 — the busiest day: standstill work, the commIssue cascade, upload re-architecture

**Morning (3devpnw):**
- `620fbb0a33`/`222c32649f` **standstill2pnw** — standstill latch (no Chill adoption at 0 mph).
- `f2bdd6dfc3`/`286e89beb8` **greenlead2pnw** — split the no-lead GREEN ding from the LEAD-departure
  ding (`leadDeparting` event) + visLat/visTtc/blnk vision-curve telemetry.
- `6d3b3e2184`/`c902a3f70b` **dumpui2pnw** — the standstill grouped cockpit card.
- `ea26f995fd`/`efaf78be04` **dmgate2pnw** — lead-departure ding gated on driver attention.
- `3f2da96775`/`0e6100b169` **dumpnarrow2pnw** — per-column card sizing (driver: "too wide").
- `91bd671752` connect-url2pnw — pairing QR → self-hosted endpoint.

**Afternoon — ⚠️ a deploy broke the truck** ("Communication Issue between Processes" cascade, full
live 10 Hz diagnosis in `drives/2026-07-13/lightning-commissue-cascade/`):
- Broken build `d63e663a79` isolated on side branch **`3devpnw-1`**; rollback point `0e6100b169`
  preserved as **`3devpnw-2`**. (The isparked2pnw/uicpu2pnw merges `42eef68f7e`/`52d2fa422a` live only
  on the broken branch; their content reached `3devpnw` via the reland below — odd topology, noted.)
- `b95a9e79f6` **reland-except-uploader** — everything except the culprit: firehose2pnw, the
  PNW-PILOT-FEATURES refresh, the `docs/pnw/` in-repo docs move, uicpu2pnw (UI param reads 60→2 Hz,
  from the `tesla-north-seattle-local` finding that `selfdrive.ui.ui` was the #1 CPU consumer),
  isparked2pnw.
- `081c256fa7`/`406b11d186` **uploadgate2pnw** — the uploader's home-first gate re-architected
  **params-only** (the culprit had been a carState subscription in a background process — now the
  standing [no-carState-subs-in-background-procs] rule).
- `5046a0f176` **mapdpin2pnw** `.override` flag; `9063aae0a5` **uploadretry2pnw** (no-silent-loss +
  `LastUploadError` surfacing); `73a3149eab` **mapready2pnw** (mapd NaN curve-target filter + honest
  warm/down readiness).
- `cdc46111c4` opendbc pin → `1ea6638d` — **fordregen2pnw Fix A**: Ford long PID damping (the EV-regen
  limit-cycle design from `FORDREGEN2PNW.md`).

**Upstream work** (`docs/GOMSGQ-SHADOW-FIX.md`): gomsgq shadow-reader race fix (`fd495fe`, PR #1,
Gemini-approved), mapd arm64 CI test build (`cabf871`), mapd PR #89 `highwayClass` **merged upstream**.

**Drives (4):** `lightning-hwy99/` (5 driver complaints), `tesla-north-seattle-local/` ("comm errors"
= one 0.3 s CAN transient; UI CPU finding), `aurora-ave-curves/` (the two takeovers were signaled
intersection turns the model didn't steer into — a lateral/model issue, refuting the curve-slowdown
framing), `lightning-commissue-cascade/` (the bisect).

**Docs born 07-13:** `FORDREGEN2PNW.md`, `GOMSGQ-SHADOW-FIX.md`, `ONROAD-CHARGING.md` (the
parked-charging-EV-reads-onroad design — became load-bearing on 07-16/18), `UI-CPU-TRIM.md`.

## 2026-07-14

| Commit | Feature | What it does |
|--------|---------|--------------|
| `a05ba63860` | **speedadjust2pnw** | Auto cruise-speed reduction for lower posted limits + police ahead (new param `AutoSpeedReduce`, default OFF). |
| `6dd8744582` | speedadjust city fix | Never cap below the posted limit — the Corvallis bug (`drives/2026-07-14/corvallis-no-resume/`: capped to 16 mph in a 25 zone). |
| `48ec82f8ea` | **standstillsoft2pnw** | Gentle Lightning standstill-launch accel ramp — the red-light "lurch" fix (root-caused from S3 qlogs in `drives/2026-07-14/lightning-standstill-lurch/` while the truck was unreachable at a campsite). |
| `8c837d6010` | standstillsoft rolling-stop bind | Re-applies the gentle ceiling on the OUTPUT so a rolling near-stop→launch can't jolt (+1.3–1.7 m/s² jolt confirmed real in `stop-jolt-and-jumpy-limitdrop/`; the "harsh decel" complaint there was the driver's own braking — no fix needed). |
| `47a99c9e7f` | **mapdheal2pnw** + **leadloss2pnw** | mapd self-heal watchdog; shadow (log-only) lead-dropout detector for the radarless Lightning. |
| `06e50c9eb6` | uploadretry polish | Only surface actionable HTTP errors. |
| `28e0800f8f` | **canoff2pnw** | Shutdown alert reword ("CAN Bus Disconnected — Vehicle Off or Check Wiring", not "Likely Faulty Cable"). |
| `a4a0767fde` | opendbc pin → `3f6fd4b0` | **fordregenb2pnw Fix B**: EV regen-bite gas compensation. (The opendbc subject says "DESIGN, not deployed" but the pin shipped it — the pin supersedes the message; live-testing per PENDING-WORK.) |

`DEVICE-STATE.md` updated.

## 2026-07-15

- `e21b51ec8f` **leadloss shadow retune** — from that day's drive data: 16 of 22 shadow events were
  distant-lead flicker at 100–120 m / 74 mph (would have random-braked if actuating) → time-headway
  distance gate, higher PROB_MIN. Validated the shadow-first doctrine.
- `3968f6fe5c` **speedadjust "wild horse" fixes** — no-limit=no-cap + smooth slewed cap (the jumpy
  city limit-drop behavior).
- mapd fork: branch `conditional-speed-limits` (`dbd0e09`) — upstream issue #50 exploration.

**Drive:** `i5-75mph-icbm-watch/` — the morning "AEB not available / couldn't engage" episode
root-caused as a transient quick-release-harness glitch (panda `safetyRxInvalid=389`), did not recur;
ICBM zero taps = correct (no sub-set curve targets all drive); leadloss shadow data pulled (above).

## 2026-07-16 — the tightfollow arc (shipped → crashed → fixed → reverted-on-data)

All on branch `speedlimitdebug2pnw`, pushed to `origin/3devpnw`
(full narrative: `drives/2026-07-16/i5-nb-corvallis-to-puget-sound/DRIVE_REPORT.md`):

| Commit | What | Outcome |
|--------|------|---------|
| `31705f5040` | **speedlimitdebug2pnw** — flash the OSM speed limit in the CES debug box for 6 s on a real change (the number was removed 07-01; this restores it as an *event*, not static text) | ✅ live |
| `c425a7c0fa` | **tightfollow2pnw** — Lightning-only Aggressive `T_FOLLOW` 1.25→1.0 s (capability-gated, Tesla untouched) | ⚠️ crashed plannerd |
| `ac17e83bc9` | **fix**: capnp enum comparison, not `int()` — `int(personality)` on a pycapnp `_DynamicEnum` raised `TypeError` and killed plannerd (no restart-on-crash) the moment Alpha-Long went live. Compile/lint + TWO diff reviews (Gemini, Opus) missed it; only the live capnp object fires it. **Lesson: compare capnp enums directly, never `int()` them.** | ✅ recovered |
| `8fff658a81` | **revert(tightfollow2pnw)** — on-road measurement: avg gap **worse** (1.92 s vs the 1.84 s baseline), hunting 1.20–3.80 s, aEgo σ 0.221→0.347 (driver: "stock cruise is much smoother"). Capability flag off, plumbing kept. | ✅ live (= channel tip) |

**Root cause (Fable design review):** the Lightning has **no radar** (`radarUnavailable=True` is
structural), so `radard.py`'s vision-only lead path feeds raw per-frame model noise into the MPC —
the obstacle-distance formula amplifies vLead noise ~10× at highway speed. A tighter target just
chased noise harder. **`radarless2pnw`** (a `VisionLeadFilter` KF1D on the vision-lead path,
mirroring the radar `Track` filter) was **written + Gemini-reviewed but NOT deployed** (deploys
paused by driver directive); plan: filter first, then re-tighten at ~1.15 s gated on lead stability.

**Also found this drive:** the Ford gap-adjust button silently cycles `LongitudinalPersonality`
(mod 3, zero dash feedback — stock upstream behavior when Alpha-Long is on) — the drive's "grandmother
gap" was Relaxed selected by accident; a `qcomgpsd` modem-race outage after a restart (no auto-restart
→ GPS+mapd down until the next restart); and the **EV park-detection gotcha** went from design doc to
operating rule: `IsOnroad` reads 1 while parked+charging → only a live `CarState.gearShifter=park`
read authorizes a restart.

## 2026-07-17

Quiet day — no commits in any repo, no drives, no docs.

## 2026-07-18 (today)

No code. Documentation day:
- **`docs/bluepilot.md`** (new) — "BluePilot 7.0 — what changed since bp-6.0 (and why angle control is
  back)", written as alan-polk merged the `bp-7.0` release branch (checked out in `sunny/bluepilot`,
  tip `19858f2888`).
- `drives/2026-07-16/i5-nb-corvallis-to-puget-sound/DRIVE_REPORT.md` written (reconstructed — the raw
  ces_events had rotated off the device; reaffirms the save-logs-with-the-report rule).
- The EV `gearShifter`-not-`IsOnroad` parked rule propagated into **both skills** (`pnw-pilot-deploy`,
  `openpilot`/pnw-era) + persistent memory.
- This changelog.

---

## Standing rules added/reaffirmed this week (the durable part)

1. **No carState subscriptions in background processes** — slow facts travel via change-only params
   (born of the 07-13 commIssue cascade; uploader re-architected params-only).
2. **Compare capnp enums directly** (`x != log.Enum.value`), never `int()` them — offline tests and
   diff reviews cannot catch the live `_DynamicEnum`; only runtime does (07-16 plannerd crash).
3. **EV parked = `CarState.gearShifter == park` (+ vEgo≈0), never `IsOnroad`** — `IsOnroad` is
   ignition-line-driven and reads 1 while parked+charging; `carState` absent = car off/asleep
   (07-16/07-18; `docs/ONROAD-CHARGING.md`, deploy skill, memory).
4. **Shadow-first for new actuating detectors** — leadloss2pnw's shadow run caught a
   would-have-random-braked trigger before any braking code existed (07-14/15).
5. **Save the raw device log next to every drive report** — the 07-16 report had to be reconstructed
   after rotation (reaffirms [feedback-save-device-logs]).
6. **Revert on data, keep the plumbing** — tightfollow's capability-flag disable preserved the
   override mechanism for the properly-sequenced v2.

## Open at week's end (see `PENDING-WORK.md` for the living list)

- **`radarless2pnw`** vision-lead KF filter: written, reviewed, awaiting deploy clearance.
- **tightfollow v2/v3**: re-tighten (~1.15 s, stability-gated, slewed) only after the filter;
  longer-term, inherit the driver's real stock gap tier from `AccTGap_D_Dsply`.
- **Lead-loss-hold actuating version**: blocked on a re-validated shadow after the retune.
- **On-screen `LongitudinalPersonality` indicator** (kill the invisible gap-button cycling).
- **Ford following under-braking** investigation; **speedadjust cap debounce** verification;
  **Ford SecOC** staged deploy; **fingerprint sidebar**; possible vision curve-penalty
  not-releasing-between-curves (unconfirmed, 07-16).
- Repo hygiene: local `3devpnw` behind `origin/3devpnw` (fast-forward when convenient); stash
  `f48371eb9c` on wazeproxy2pnw (stale PNW-PILOT-FEATURES edit) to drop or land.
