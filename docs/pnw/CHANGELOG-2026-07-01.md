# CHANGELOG — PNW distribution, 2026-06-29 → 2026-07-01

What changed in the PNW openpilot distribution (`dirkpetersen/pnw-pilot`, branch **`4devpnw`**) over
the last ~2 days, and how it landed on the car. Terse + pitfall-first, in the house style. All code
lives in `pnw/pnw-pilot/`; the comma-connect design proposal (F) lives in `comma-connect/`.

---

## TL;DR — what changed for the driver

- **Curves are safer, earlier, and not slower.** VTSC now looks the FULL ~500 m the map publishes (was
  ~370 m), so the blind sharp-curve "TAKE CONTROL" beep on I-90 descents should stop; braking is
  regen-coast (~0.2 g, no friction brake) unless a genuinely sharp curve needs a bounded firmer stop.
  Curve entrance is the slowest point → the car accelerates out. Flat twisty roads keep full speed;
  only winding **descents** trim the base cruise. No new toggle — rides the existing CES master.
- **The map stops dying after updates.** The `mapd` engine binary now installs to persistent
  `/data/mapd/mapd` (outside the git tree) so an auto-update's `git clean` can't delete it → no more
  silent "no map data".
- **Police alerts are quieter and cleaner.** Siren OFF (visual banner kept), banner only inside 0.25 mi,
  drops other-side-of-highway reports, only shows reports ≤20 min old. The overlay also shows the real
  poll failure (e.g. `quota (429)`) instead of a false "Clear", gives EV chargers a compass direction,
  and wraps the three advisory lines onto two lines. New rest-area data for I-82 and US-12/US-95.
- **The shipped Waze key is gone.** The RapidAPI key no longer ships in the repo — it lives only in
  `/data/pnw/location/police_proxy.json` on the device. No key → police degrades to no-data, no false
  alerts.

- **VTSC curves were then TUNED on a live drive** (Seattle→eastern-WA on I-90, Snoqualmie/Cle
  Elum/Ellensburg): the sharp-curve tune over-slowed and forced throttle overrides, so it was retuned
  in two on-the-road passes to a lower, RELIABLE lateral target **bounded by a new speed-limit floor**
  (VTSC never trims below the posted highway limit). The driver confirmed the result "works very well."
  Same drive added the Tesla EV Supercharger-alternation line and a charger drop-when-receding filter.
  See section **H** and the drive report `../drives/2026-07-01/hotspot-drive/DRIVE_REPORT.md`.

**Deploy status:** A–E + H are **LIVE** on the comma 3X (Tesla Model S Raven, HW3), `4devpnw` @
**`bb8c0d0`**, git-deployed from GitHub (`git fetch` + `git reset --hard origin/4devpnw`, no clean). F is
**design-only**.

---

## Commit map (`4devpnw`)

| SHA | Subject | Section |
|-----|---------|---------|
| `39997887f9` | sharpcurve2pnw: earlier 500m lookahead + regen-coast curve slowdown | A |
| `b124c8d1cc` | Merge sharpcurve2pnw into 4devpnw | A |
| `aaf8002c09` | mapd2pnw: install mapd binary to persistent /data/mapd | B |
| `13492c9d40` | location2pnw: Waze key→override file, poll-error UI, EV compass, 2-line wrap, +I-82/US-12 rest areas | C, D |
| `5c329434` | location2pnw: police siren OFF, banner 0.5→0.25mi, drop opposite-dir + fresher (20min) reports | E |
| `c6285c2ec3` | vtsc: less-aggressive curve tune (A_LAT 2.2→3.0, MAP_SPEED_SCALE 1.12→1.5) + CES/overlay tweaks (Snoqualmie I-90 live) | H |
| `bb8c0d0e01` | location2pnw + vtsc: Tesla EV alternation, charger drop-when-receding, A_LAT 3.0→2.5 + speed-limit floor | H |
| `e2a634b` (comma-connect, branch `dirk`) | WAZE-API.md fleet-scale Waze proxy design | F |

The device self-advanced `b124c8d → 13492c9` on its own (auto-update tracking `origin/4devpnw`), then was
manually fast-forwarded to `5c329434`, and finally to **`bb8c0d0`** after the live-drive retune (H). See
Ops notes (G).

---

## A. Sharp-curve take-control fix (VTSC full-horizon + regen-coast)

**Branch `sharpcurve2pnw` → merged as `b124c8d1cc`** (implementation `39997887f9`). Doc:
`pnw/pnw-pilot/SHARPCURVE2PNW.md`. Pure-Python, **no new params** — rides the CES master (`CESMode`),
behavior-neutral when CES is Off. **All changes only ever REDUCE speed.**

**Problem.** Recurring blind sharp-curve **"TAKE CONTROL"** beep on the tightest curves at high set
speed (I-90 descents): the slowdown started too late and the driver had to intervene.

**Root cause = lookahead, not braking.** pfeiferj mapd publishes a FIXED **500 m** path
(`MIN_WAY_DIST`), but VTSC scanned only `v_ego * MAP_LOOKAHEAD_S` (~370 m @ 70 mph) and CES ~310 m —
throwing away ~130–190 m (~6 s) of available warning. The braking distance to slow 70→30 mph at a
gentle decel (~330 m) FITS in 500 m but NOT in the old ~370 m horizon.

**Driver model (EV / Model S).** "On the freeway braking should almost never be required; as soon as you
decelerate, regen recoups and makes it much slower." Lifting the accelerator ≈ 0.2 g of regen. So the
fix leans on **lookahead + regen-coast**, not friction braking: get off the gas early so the curve
**ENTRANCE is the slowest point**, then accelerate out (sometimes before the apex).

**Solution — 4 parts, all in VTSC.**

1. **Distance-based full-horizon lookahead** (`most_binding_map_curve` in `vtsc_pnw.py`). Scans the FULL
   `MAP_SOURCE_HORIZON_M = 500 m` and picks the curve whose decel-envelope brake cap is the **lowest
   right now** (not nearest, not min-speed). A far sharp curve has a high, non-binding envelope until
   you're close enough, so scanning farther only buys earlier detection, never premature slowing. The
   MTSC `speed_scale` (1.12×) and `v_cruise` clamp are applied to each point **BEFORE** selection so the
   selected curve matches the value used downstream (no selection/use mismatch). `is_sharp` is
   classified on the **RAW** (unscaled) target — else the 1.12× scale inflates a genuinely sharp 28 m/s
   curve to 31.4 and drops its sharp flag.
2. **Regen-coast deceleration.** Normal commanded decel is capped to EV regen authority
   `REGEN_A_DECEL = 2.0 m/s²` (~0.2 g, coast/regen, no friction brake), set as the rate-limit ceiling in
   `vtsc_controller.py`. A genuinely **sharp** map curve (`SHARP_CURVE_V = 30 m/s`) that regen alone
   can't reach before its **entrance** (`required_decel(v_ego, v_curve, d_entrance) > REGEN_A_DECEL`,
   where `d_entrance = d_apex − v_ego*APEX_FINISH_S`) raises the ceiling to `SHARP_A_DECEL_MAX = 2.8` —
   **last resort only, still bounded** (never a slam).
3. **Apex retune → slowest at entrance, accelerate before apex.** `APEX_FINISH_S 1.2→2.5`,
   `HOLD_TTA_S 1.2→2.5`, `APEX_TTA_S 0.4→1.2`. The `at_safe` gate (`RELEASE_SPEED_MARGIN`) still guards
   RELEASE, so the car only accelerates pre-apex once actually slowed to curve-safe speed (lateral
   margin preserved).
4. **Twisty-DESCENT base trim** (`twisty_section_cap`). Trims the base cruise ONLY when BOTH (a) ≥
   `TWISTY_MIN_CURVES = 3` packed binding curves within the horizon AND (b) the road is descending
   (`pitch < TWISTY_DESCENT_PITCH = -0.035 rad`, from `carControl.orientationNED[1]`). Holds a lower
   base cruise (floor `TWISTY_MIN_FACTOR = 0.82`) through the section so it doesn't re-accelerate to full
   set between blind curves. **Flat twisty keeps full speed** (per-curve VTSC handles it) → no speed lost
   where it isn't needed.

New pure helpers in `vtsc_pnw.py`: `most_binding_map_curve`, `required_decel`, `twisty_section_cap`,
`_haversine_m` (self-contained copy so the module stays testable).

**Speed impact (driver: "don't lose speed overall").** Curve-safe (slowest) speed is UNCHANGED
(`A_LAT_TARGET = 2.2`); earlier lookahead replaces late hard braking with early gentle regen
(equal-or-faster, smoother); accelerating before the apex regains speed sooner. The ONLY deliberate
reduction is the twisty-DESCENT trim. The ≥1 mph curve cue (`CONFIDENCE_CUT`) on every real bend is
untouched.

**Files:**
- `selfdrive/controls/lib/vtsc_pnw/vtsc_pnw.py` (new pure functions)
- `selfdrive/controls/lib/vtsc_pnw/vtsc_constants.py` (new constants)
- `selfdrive/controls/lib/vtsc_pnw/vtsc_controller.py` (wiring: `_a_decel_max = REGEN_A_DECEL`,
  full-horizon scan, last-resort ceiling, twisty trim)
- `selfdrive/controls/lib/vtsc_pnw/tests/test_vtsc_pnw.py` (36 tests)
- `SHARPCURVE2PNW.md`

**Commits:** `39997887f9`, merge `b124c8d1cc`.

**Verification.** 36 unit tests pass. 2 rounds of Gemini review (gemini-pro-latest); fixes landed:
MTSC scale-before-selection, `is_sharp` on the raw target, use the full 500 m horizon, brake-to-entrance
(not apex), twisty trim gated on `v_cruise_set` + descent. **Deploy status: LIVE, but NOT yet driven** —
needs an on-road twisty/descent drive with `VtscMapCurves` ON + a non-Off `CESMode` to validate.

---

## B. mapd binary persistence — fixes "no map data" after an auto-update

**Commit `aaf8002c09`** (branch `mapd2pnw` line, on `4devpnw`). Memory:
`mapd-binary-wiped-by-autoupdate`.

**Problem.** After an auto-update the map silently showed "no data" even though the OSM DB was intact
(443 MB, `MapDownloadStatus=OK`, GPS good). Live incident 2026-06-29.

**Root cause.** The pfeiferj `mapd` Go binary was installed to the **untracked** in-tree path
`selfdrive/mapd`. `updated.py`'s `fetch_update()` runs `git clean -xdff`, which **deletes** it
(`.gitignore` does not help — `git clean -x` removes ignored files too). The boot re-download often
fails (DNS/clock not valid before NTP), so mapd stays down → `mapdOut`/`mapdExtendedOut` never publish →
the overlay reads `None` for everything (`mapPts=0`).

**Solution.** `system/mapd/installer.py` now installs to the **persistent** `/data/mapd/mapd` (outside
the git tree, survives every clean/reset/reboot). `MAPD_BINARY` resolves to `/data/mapd/mapd` when
`/data` is writable (else in-tree fallback for dev/CI). `ensure_mapd()` **migrates** an existing valid
in-tree binary with an atomic copy+replace (`shutil.copy2` → `chmod 0755` → `os.replace`) — no
re-download needed, which is what lets the fix survive even when the boot network is down.
`system/manager/process_config.py` execs the absolute `MAPD_BINARY`.

**Files:** `system/mapd/installer.py`, `system/manager/process_config.py`.

**Commit:** `aaf8002c09`.

**Verification.** On-device seeded `/data/mapd/mapd`, `installer --check` → `installed=True
dest=/data/mapd/mapd`, map served from the persistent binary; manager-supervised exec confirmed
surviving a full power-cycle. Gemini-reviewed.

---

## C. Waze police key removed from code → persistent override file

**Commit `13492c9d40`** (part 1).

**Problem.** The shared RapidAPI (Waze proxy) key shipped inside the repo (`DEFAULT_PROXY["key"]`). It
does not scale (a key-per-distribution), and the shared BASIC-plan key had been HTTP-429
monthly-quota-exhausted for weeks → every device showed no police data anyway.

**Solution.** `DEFAULT_PROXY["key"]` is now empty — the distribution ships **no** Waze key. The key
lives ONLY in `/data/pnw/location/police_proxy.json` (persistent `/data`, survives reboot AND git
reset/clean). `_load_cfg()` uses the override file only if it has both `key` + `url`; otherwise
`DEFAULT_PROXY` (keyless) → `run()` sets `nokey` → state `nodata`, and it **does not poll** (no wasted
requests, no false alerts). Note the shared key's monthly quota reset on 2026-07-01, so with the
override file present the feed went live again (HTTP 200) — which is what surfaced the "too many alerts"
tuning need in E.

**Files:** `system/location_services/location_servicesd.py` (`DEFAULT_PROXY`, `_load_cfg`, `run`).

**Commit:** `13492c9d40`.

---

## D. Location-services overlay improvements

**Commit `13492c9d40`** (part 2).

1. **Poll-error surfacing (no more false "Clear").** The police thread records a short error tag on any
   non-ok poll and exposes it via `snapshot()`. `_line_police` returns `{state:"nodata", err:…}`; the UI
   renders it in **RED** on the police line. Tags: `quota (429)` / `HTTP <code>` / `timeout` /
   `bad resp` / `net err` / `no key`. HTTPError is checked before URLError (it subclasses it) so the 429
   quota case is distinguished. It is **never** conflated with a genuine "Clear".
2. **EV charger compass direction.** e.g. `EV fast  8.0 mi (NW)` — absolute 8-point compass (`_compass8`
   in the daemon from `geo.bearing_deg`; published as `ev.compass`, rendered after the distance).
3. **Two-line hanging-indent wrap.** The three advisory lines (police/rest/EV) wrap onto two lines,
   breaking near `len(longest)//2 - 3` snapped to the nearest space so words aren't cut, continuation
   line indented 3 (`_CONT_INDENT`), all left-aligned (`_wrap` / `_split_near` in the UI renderer).
4. **New rest-area data.** Added I-82 (3 areas) and US-12/US-95 (6 areas) →
   `system/location_services/data/rest_areas/` (`i82_rest_areas.json`, `us12_us95_rest_areas.json`); i5/
   i90 files untouched. Merged, ~26 rest areas load total.

**Files:** `system/location_services/location_servicesd.py`,
`selfdrive/ui/onroad/location_services_status.py`, and the two new `data/rest_areas/*.json`.

**Commit:** `13492c9d40`.

---

## E. Police-alert tuning — after the restored feed fired too often

**Commit `5c329434`.** Driver request 2026-07-01: once C restored the live Waze feed, police-ahead
alerts fired too often on a live drive.

1. **Siren OFF.** `selfdrive/ui/soundd.py` `SIREN_ENABLED = False` gates the one-shot police-chirp block
   (arming + playback). The **visual** "POLICE AHEAD" banner is kept; the safety-alert audio path
   (`AudibleAlert` branch) is untouched. Flip to `True` to restore the chirp.
2. **Banner nearer.** `_POLICE_NEAR_MI` 0.5 → **0.25** mi (banner only when a report is that close
   ahead).
3. **Drop opposite-direction reports.** `_line_police` now drops reports whose `_police_dir` is `"opp"`
   (other carriageway). Same-direction (`same`) and unknown-direction (`none`, magvar missing) reports
   are KEPT — Waze often omits magvar, so dropping unknowns would silently miss most reports.
4. **Fresher only.** `POLICE_STALE_S` 45 min → **20 min** (drop crowd reports older than 20 min).

**Files:** `selfdrive/ui/soundd.py`, `system/location_services/location_servicesd.py`,
`selfdrive/ui/onroad/location_services_status.py`.

**Commit:** `5c329434`.

---

## F. WAZE-API.md — fleet-scale Waze proxy (DESIGN ONLY, not deployed)

**Commit `e2a634b` on branch `dirk` in `comma-connect/`.** Doc: `comma-connect/WAZE-API.md`.

**Problem.** Per-device Waze keys don't scale: Waze calls = N devices × once/min/drive, co-located cars
double-query, the shared key ships in the repo, and the BASIC key's monthly quota is exhausted.

**Recommendation.** A ~30-line **Cloudflare Worker** acting as an **on-demand caching reverse proxy**:
the device does a **keyless** `GET ?lat&lon`; the Worker holds the one shared key, computes a cache key
from the quantized position + a short time bucket, returns cached JSON on a hit (zero upstream calls),
and on a miss calls Waze once, transforms to the device's raw-alert shape, and caches it with a short
TTL (~180 s). This **dedups the whole fleet onto one key per cell per TTL window** — Waze calls ≈ unique
cells actually driven, independent of device count. It's **lazy** (only driven areas are ever fetched),
needs **no server to babysit**, and reuses tooling already in place (`wrangler` is used by
`comma-connect/deploy-preview.sh`).

**Explicitly rejected:** the static-S3-tiles + GitHub-Actions-cron approach — precompute-all is
wasteful, cron floors at ~5 min / jitters / disables after 60 days of inactivity / burns Actions
minutes, and static ≠ on-demand (S3 can only serve bytes that already exist). Also **corrected** an
earlier draft that assumed an nginx server: the live `connect` is a **static SPA on S3 + CloudFront** (no
server), so the nginx caching-proxy option only applies if connect is moved onto the self-hosted `go/`
container.

The device-side change (`_poll_proxy` + a `WazeSource` selector in `location_servicesd.py`) is
specified but **not written**. The `police` block contract to the UI is unchanged, so it's a drop-in
swap at `PoliceUpdater._poll`. **Status: no code changed, nothing deployed.**

---

## G. Ops notes (deploy / operations learnings)

Captured from this session's on-device work. See also memories `device-manager-restart-tmux`,
`auto-update-blocked-finalize-no-build`, `upload-api-host-412-footgun`, and the `pnw-deploy` skill.

1. **`sudo systemctl restart comma` alone is a NO-OP** when the stack is up. `comma.service` is
   `Type=oneshot` whose ExecStart is `tmux new-session -s comma …`; re-running it hits "duplicate
   session" and nothing cycles. The working full cycle is:
   `touch /tmp/booted` (avoid tap-reset) → `sudo rm -rf /data/safe_staging/finalized` (avoid the
   finalized-swap reverting the deploy) → `tmux kill-session -t comma` → `sudo systemctl restart comma`.
   The SSH shell is separate from the `comma` tmux, so killing the session doesn't drop SSH.
2. **The device auto-tracks `origin/4devpnw` and self-deploys pushes via build-on-boot.**
   `DisableUpdates` keeps reverting to 0 (the "Allow auto updates" UI toggle re-enabling it). Confirmed
   2026-07-01: it self-advanced `b124c8d → 13492c9` with no manual step. Safe for **pure-Python** pushes;
   a change touching `params_keys.h`/C relies on build-on-boot rebuilding `params_pyx.so` (the updater's
   `git clean -xdff` deletes `prebuilt`, so build-on-boot self-restores it for a real OTA — but verify
   `prebuilt` is absent after an OTA before trusting the build). A manual `git fetch + reset --hard
   origin/4devpnw` + cycle just makes a push live immediately instead of waiting for the update cycle.
3. **`pkill -f "<module>"` from an SSH one-liner self-matches the remote shell** and can kill your own
   session. Use `tmux kill-session` for a full cycle, or kill by numeric PID for a single process. (Same
   trap makes `pgrep -f`/`pkill -f` unreliable for crash-loop detection — bracket the first char, e.g.
   `grep '[l]ocation_servicesd'`, and watch etimes grow on a stable PID.)
4. **`location_servicesd` and `soundd` are NOT `restart_if_crash`** — manager does not revive them after
   a `pkill`; only a manager cycle brings them back. `soundd` is onroad-gated (driverview predicate), so
   it is **correctly absent while parked** (not a fault).

**This session's deploy.** All of A–E git-deployed from GitHub (`git fetch` + `git reset --hard
origin/4devpnw`, **no clean**, so `params_pyx.so`/`prebuilt` and the persistent map/POI data under
`/data` survived), left at `4devpnw` @ `5c329434`. Verified: processes stable (growing etimes, PIDs not
cycling), on-device markers present, no tracebacks — except an unrelated pre-existing modem/eSIM
`hardwared configure_modem` AT-command error. Device = Tesla Model S Long Range Plus 2021 (Raven, HW3),
the one physical comma 3X that moves between it and the Ford F-150 Lightning.

---

## H. Live-drive tuning (I-90 Snoqualmie → Ellensburg, 2026-07-01)

**Commits `c6285c2ec3` (pass 1) and `bb8c0d0e01` (pass 2, current LIVE tip).** Source of truth for the
drive: **`../drives/2026-07-01/hotspot-drive/DRIVE_REPORT.md`** (take-control episodes with lat/lons,
the 700-tick data pull, the tuning decisions, deferred items). Driver verdict after the retune: **"these
deployments work very well."**

The A. sharp-curve fix shipped with `A_LAT_TARGET = 2.2` (unchanged curve-safe speed). On the live drive
that **over-slowed** — the map safe-speeds (`mapV`) came back at 22–29 mph on curves the driver takes at
60–85 mph, so VTSC braked hard toward them and the driver had to **override with the gas** (two
take-controls: `47.36277,-121.37305` map-curve `mapV=29`, and `47.33498,-121.34702` a *mild* stretch
`curvePct=56`). Root cause was NOT under-braking/steering — it was the targets being far below this
driver's comfort. Two tuning passes followed, on the road (VTSC lives in the longitudinal planner, so a
git-deploy + manager restart applies it; pushed only when parked).

### The tuning loop (the valuable part)

1. **Pass 1 — carry more speed** (`c6285c2`). Bump the aggressiveness knobs so curves keep ~+10–12 mph.
   Confirmed live: a `curvePct=100` vision curve (~`46.944,-120.198`) carried **79 mph** (90→79) with no
   gas override, vs the old over-slow to ~58.
2. **Driver rule emerged: never cap below the posted speed limit on a highway.** Only trim from the set
   speed DOWN TOWARD the limit (90→75 in a 70 zone OK; never below 70). This structurally prevents deep
   over-slows and — crucially — makes a LOWER, more reliable `A_LAT_TARGET` safe (worst case is the
   limit, not 58 mph).
3. **The 700-tick data pull (east of Ellensburg) reframed the problem as INCONSISTENCY, not just
   aggressiveness.** Across 15 tight-curve episodes VTSC slowed hard on some (90→76 tgt28; 60→49 tgt22)
   but **held full speed through others the map rated 28–36 mph** — e.g. **89→89, curvePct=100,
   map-tgt=34, limit=70** (a take-control) — and once dipped **60→49, below the 60 limit**. So it was BOTH
   under-slowing tight curves AND occasionally over-dipping below the limit. Root: the map-curve fold is
   **intermittent**, and vision at `A_LAT=3.0` alone is too weak to reliably catch these curves.
4. **Pass 2 — lower A_LAT + speed-limit floor** (`bb8c0d0`). Land on reliable slowing bounded at the
   limit, plus the EV-line changes.

### Pass 1 — `c6285c2ec3` (VTSC retune #1 + overlay)

- **Problem.** Sharp-curve tune over-capped curves → throttle overrides / take-control.
- **Change.** `vtsc_constants.py`: `A_LAT_TARGET` **2.2 → 3.0**, `MAP_SPEED_SCALE` **1.12 → 1.5**,
  `GENTLE_PROFILE` A_LAT → 3.0 (tracks default). 4 unit tests repointed to tighter binding radii (R415 is
  safe at ~79 mph under 3.0, so it no longer binds at the 70 mph test cruise); 36 pass. `ces_status.py`:
  **removed the bottom OSM speed-limit line** (frees space) + its now-unused helper/local.
  `location_services_status.py`: police banner trigger **restored 0.25 → 0.5 mi** so it actually shows —
  **siren stays OFF** (E; audio was the CPU/comms-timing stressor on the near-capacity 3X).
- **Files.** `selfdrive/controls/lib/vtsc_pnw/vtsc_constants.py`,
  `selfdrive/controls/lib/vtsc_pnw/tests/test_vtsc_pnw.py`, `selfdrive/ui/onroad/ces_status.py`,
  `selfdrive/ui/onroad/location_services_status.py`.
- **Status.** LIVE and validated on the drive (79 mph through the curvePct=100 curve, no override; police
  banner + siren-off confirmed at 0.2 mi). Superseded by pass 2 for the A_LAT value.

### Pass 2 — `bb8c0d0e01` (A_LAT 2.5 + speed-limit floor, Tesla EV alternation, charger recede) — CURRENT TIP

- **Problem.** A_LAT=3.0 was inconsistent (missed tight curves → take-controls) and could dip below the
  limit; driver wanted a Tesla Supercharger-aware EV line and chargers to drop once passed.
- **VTSC change.** `A_LAT_TARGET` **3.0 → 2.5** for RELIABLE tight-curve slowing, made safe by the new
  **SPEED-LIMIT FLOOR** in `vtsc_controller.py`: on a highway VTSC never caps below the posted limit —
  `capped = min(v_cruise, max(capped, self._speed_limit))`, gated on `self._is_freeway and
  self._speed_limit > 0.0`. The limit + road context are read from the mapd bridge mem-params
  (`MapSpeedLimit`, `RoadContext == 'freeway'`) in `_read_enabled()`. Off-highway / no-limit-data → no
  floor (V_MIN still applies). A curve genuinely too tight for the limit is then the driver's to handle
  (stated priority = stay above the limit). `GENTLE_PROFILE` A_LAT also → 2.5. 36/38 vtsc tests pass (2
  pre-existing `setproctitle` env failures, unrelated).
- **EV change (Tesla only).** On a Tesla, the EV line **alternates every `EV_ALT_S = 4 s`** between the
  nearest Tesla **Supercharger** (`ev_network == "Tesla"`, `_is_supercharger`) and the nearest **other**
  charger — Supercharger first, each side anti-flickered with its own `_Hold`. Tesla is detected via
  `CarParamsPersistent` (`_read_is_tesla`, same parse as `ui_state.py`). Non-Tesla (Lightning) keeps
  today's fast/slow logic. Done in the daemon (1 Hz) so the UI is unchanged.
- **Charger drop-when-receding.** `_RecedeFilter` (`EV_RECEDE_MI = 1.0`, tracked within `EV_TRACK_MI =
  8 mi`) drops a charger once you're >1 mi PAST your closest approach → the next-nearest shows;
  re-eligible on re-approach. Applied to the candidate list before selection, so surface-street
  `_nearest_within` no longer clings to a just-passed charger.
- **Files.** `selfdrive/controls/lib/vtsc_pnw/vtsc_constants.py`,
  `selfdrive/controls/lib/vtsc_pnw/vtsc_controller.py`,
  `system/location_services/location_servicesd.py`.
- **Status.** LIVE — this is the deployed tip. Driver: "works very well."

### Deferred / next (a "police pass" + a VTSC-reliability pass — honest TODO)

Called out explicitly in the drive report; NOT done in these commits:

1. **Map-fold reliability deep-fix.** Why the map-curve fold is intermittent (`_fold_map_curve` binding
   test / map-data availability) — the biggest reliability lever; needs instrumented per-tick data.
2. **Restore the ~1 mph pre-curve cue** the driver misses at the START of every bend (the
   `CONFIDENCE_CUT` / `CUE_MIN_CURVATURE` nibble) — check why it's not visible under the new tune.
   *Both 1 & 2 need per-tick VTSC applied-cap data — `ces_events.jsonl` does not log the applied cap, so
   it can't tell commanded-and-gas-overridden from never-commanded.*
3. **Police — live <1 mi distance update** (daemon publishes the nearest report lat/lon; UI recomputes
   at 5 Hz from live GPS, no Waze re-query — CPU is near cap). Daemon's 1 Hz + 0.1 mi rounding reads as
   "stuck".
4. **Police — drop-when-receding** (clear a report once its distance starts increasing; stateful, keyed
   by uuid — mirrors the EV/rest `_Hold`/`_RecedeFilter`). Supersedes the earlier "lingers after passing"
   deferral. Group 3 + 4 as the next "police pass."
5. **Rest-area distance over-report on winding road** (Indian John Hill showed 2.3 mi vs ~1 mi sign;
   along-the-road projection onto a farther arm of a U-shaped stretch). Display-only, DEFERRED.
6. **"timeout" on the police line = working as designed** — network timeout in a dead zone; the
   error-surfacing correctly shows WHY there's no data. Not a bug.

---

## I. Live-drive fixes (I-82): DM glare + mapd msgq prefix (2026-07-06)

**Commit `2fc78f0fbd` (`4devpnw`, current LIVE tip), deployed 2026-07-06, verified live.** Two
independent fixes surfaced on an I-82 westbound low-sun drive, both Gemini-reviewed (gemini-pro-latest,
no issues). Neither adds a param key.

### I.1 — DM glare desensitization (Layer C band-aid from `GLARE.md`)

**File:** `selfdrive/monitoring/helpers.py`. Full analysis: `GLARE.md` (§2 root cause, §10 this deploy).

**Problem.** On a long west-bound low-sun stretch the driver-monitoring nag went off repeatedly while
the driver was fully attentive ("DM going mad").

**Root cause (confirmed on the road).** Side/back sunlight washes out the DM camera image → the pose
uncertainty `faceOrientationStd` spikes → after just **10 s** of high std `is_model_uncertain` flips
True → monitoring drops **OUT of the relaxed dual-counter ACTIVE mode** (the 3h-pose / 1h-phone
relaxation this fork ships) **INTO stock PASSIVE wheel-touch mode** → the false "distracted"
escalation. The existing relaxed timeouts only apply *in* active mode, so glare bypassed them
entirely — which is exactly why prior DM relaxation (`dmon2pnw`) never helped the glare complaint.

**Fix (keep DM in active/relaxed mode through a glare burst).** Three knobs, before → after:
- `_POSESTD_THRESHOLD` **0.30 → 0.45** — tolerate more pose uncertainty before a frame is "high std".
- `_HI_STD_FALLBACK_TIME` **10 s → 30 s** — wait 3× longer before the passive wheel-touch fallback.
- `_DCAM_UNCERTAIN_RESET_COUNT` **20 s → 2 s** — ports commaai `4ecbdb0d7a`; the offroad
  `Offroad_DriverMonitoringUncertain` nag clears faster (diagnostic/alert only — **not** live logic).

`_FACE_THRESHOLD` left at **0.70** (lowering it is the riskiest knob per `GLARE.md` — masks real
inattention in washed-out frames; the trade was kept modest). Applied **UNGATED** (no toggle),
consistent with how `dmon2pnw` relaxation was applied — a deliberate deviation from `GLARE.md`'s
"toggle-gated, default OFF" Layer-C recommendation. The safety tradeoff (a slightly wider inattention
window) is inherent to any Layer-C relaxation and was **explicitly requested by the driver** ("less
aggressive, if in doubt").

**Not the real fix.** Layers A (0.11.1 driver-cam BPS ISP/gamma pipeline) and B (0.11.1 sleep-prob DM
model) — the upstream fixes that improve what the camera/model actually *see* — remain NOT ported.
This Layer C only lengthens the tolerance window. See `GLARE.md` §4/§6.

**Verified live:** `POSESTD=0.45`, `HI_STD_FALLBACK=30 s`, `DCAM_RESET=2 s`, `FACE=0.70`.

### I.2 — mapd msgq prefix fix (cold-boot deaf-mute race)

**File:** `system/manager/process_config.py`. Memory: `mapd-binary-wiped-by-autoupdate` (adjacent —
different root cause).

**Problem.** No speed limit, no map curves (`MapTargetVelocities`), and no `RoadContext` — the last
also broke location-services freeway detection, showing chargers/rest areas as "nearby" instead of
"ahead". Map **data** was never missing (WA/OR/ID tiles all on disk). A plain reboot did NOT clear it.

**Root cause.** `mapd` is a Go native binary using the `gomsgq` lib, which **auto-detects** the
`/dev/shm` segment naming by stat'ing `/dev/shm/msgq_logMessage` at startup. At cold boot mapd
starts **BEFORE** the openpilot stack has created that segment, so gomsgq guesses the **unprefixed**
names and mapd is silently **deaf + mute for the entire session** — it publishes/subscribes on
segments nothing else uses. The race is **deterministic in the bad direction at boot**, which is why
rebooting never fixed it.

**Fix.** Launch mapd via `["/usr/bin/env", "USE_MSGQ_PREFIX=true", MAPD_BINARY]` so it always uses
the **prefixed** names matching this tree's C++ `msgq.cc` (`/dev/shm/msgq_<name>`), instead of
guessing from stat-timing.

**Interim hotfix during the drive:** mapd was manually relaunched with the env var (an orphan process
outside manager); the deploy's manager restart replaced that orphan with a proper managed instance.

**Verified live:** mapd relaunched by manager with `USE_MSGQ_PREFIX=true`, **8 prefixed segments**
present, the mapd bridge repopulating (`RoadContext` live again).

**Files:** `selfdrive/monitoring/helpers.py`, `system/manager/process_config.py`. **Commit:**
`2fc78f0fbd`.
</content>
</invoke>
