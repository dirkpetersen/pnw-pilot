# LATACCEL2PNW — speed-scheduled, JSON-tunable lateral-accel cap

Branch: `lataccel2pnw` (based on `origin/3devpnw`). Touches
`selfdrive/controls/lib/drive_helpers.py` (+ a documentation-only comment in
`selfdrive/ui/layouts/settings/toggles.py`, unrelated feature, see bottom of this doc).

## What changed

`clip_curvature()` — the function every car's lateral planner calls to bound commanded curvature to
ISO lateral-jerk and lateral-accel limits — used a single fixed cap,
`MAX_LATERAL_ACCEL_NO_ROLL = 3.0` m/s², at every speed. This port replaces that fixed cap with a new
`lat_accel_limit(v_ego)` helper that returns a **speed-scheduled** cap, hot-reloadable from a JSON
file on disk. `MAX_LATERAL_ACCEL_NO_ROLL` itself is unchanged (still 3.0 m/s², the ISO baseline) and
now also serves as `lat_accel_limit()`'s hardcoded fail-safe return value.

Everything else about `clip_curvature()` is untouched: the exact function signature
`clip_curvature(v_ego, prev_curvature, new_curvature, roll) -> tuple[float, bool]`, the ISO lateral
jerk limit (`MAX_LATERAL_JERK`, not touched), the `MIN_SPEED` floor, the roll-compensation math
(`roll_compensation = roll * ACCELERATION_DUE_TO_GRAVITY`, applied additively on top of the cap
exactly as before), and the returned `(curvature, limited)` tuple. The only line that changed inside
the function body is where the cap comes from:

```python
roll_compensation = roll * ACCELERATION_DUE_TO_GRAVITY
lat_accel_cap = lat_accel_limit(v_ego)          # was: MAX_LATERAL_ACCEL_NO_ROLL
max_lat_accel = lat_accel_cap + roll_compensation
min_lat_accel = -lat_accel_cap + roll_compensation
```

## The schedule

| Speed | Cap |
|---|---|
| ≤50 mph | 5.0 m/s² |
| 50 → 60 mph | tapers 5.0 → 4.0 m/s² (linear) |
| 60 → 70 mph | tapers 4.0 → 3.0 m/s² (linear) |
| ≥70 mph | 3.0 m/s² (ISO baseline, unchanged from stock) |

Breakpoints, held flat outside the ends: `(50 mph, 5.0)`, `(60 mph, 4.0)`, `(70 mph, 3.0)`.

### Rationale

- **Low/mid-speed authority bump.** Below the stock 3.0 m/s² ISO cap, the planner is more
  conservative than it needs to be at parking-lot / residential / campus / most-arterial speeds — it
  can make the car cut a tight turn or U-turn wider than a driver would expect, or need to slow
  further than necessary to complete a turn within the commanded curvature. Raising the cap to 5.0
  m/s² up to 50 mph gives the planner more lateral headroom through this whole range, and it also
  doubles as the layer-2 safety envelope described below.
- **High-speed stays at ISO 3.0.** By 70 mph the cap is deliberately back down at the same 3.0 m/s²
  stock openpilot has always used. The reasoning is specific: if the model wants to hold a curve at
  cruise speed that requires *more* than 3.0 m/s² of lateral acceleration, the correct fix is to
  **slow down before the curve**, not to steer harder through it — hard cornering is the wrong
  response to a curve that was misjudged or entered too fast. Speed reduction ahead of a curve is
  already this fork's job for exactly that scenario (VTSC's vision/map curvature-based cruise-speed
  reduction, and ICBM's stock-ACC SET− taps on the Lightning) — `lat_accel_limit()` intentionally does
  not try to duplicate or substitute for that by loosening the cap at speed.
- **Global, not car-gated.** Per this fork's capability-view convention (`selfdrive/controls/lib/
  pnw_vehicle.py`), feature code must never branch on `carFingerprint`. A max-lateral-accel envelope
  is a shared control limit, not a feature — `clip_curvature()` is called for every car, and raising
  the low/mid-speed cap can only ever give the curvature clamp *more* room to work with; it never
  makes any car (Tesla or Ford) more restricted than it was before this change. There is nothing here
  for a fingerprint check to gate.

### Layer-2 safety rationale (2026-08-11 retune: 30/45/60 → 50/60/70)

Curve-speed slowdown is **layer 1** — VTSC (vision/map curvature) and CES are what are supposed to
get the cruise speed down *before* a curve that needs more lateral authority than the ISO 3.0 m/s²
baseline provides. This lateral-accel schedule is **layer 2**: the backstop for when layer 1 doesn't
fire (missed detection, a curve that tightens faster than the model anticipated, a driver-set speed
that doesn't get reduced, etc.) — at that point the only thing left that can keep the car in its lane
is actually using the truck's available steering authority, and the old schedule undercut that.

Origin of the new anchors — the **2026-08-11 10:37 PDT Crown Hill left curve** (rlog-reconstructed,
see `drives/2026-08-11`): at 49 mph the curve demanded ~4.8 m/s² of lateral acceleration. The truck
was physically capable of it (EPS reached ~55° of wheel angle with authority to spare), but the old
breakpoint schedule had already tapered the cap down to ~3.5 m/s² by 49 mph, so `clip_curvature()`
clipped the demand and openpilot under-turned through the curve until the driver took over. The new
schedule keeps the cap **6.0 m/s²** out to 50 mph, tapering to 5.0 by 60 and 4.0 by 70. The cap was
raised (5/4/3 → 6/5/4, 2026-08-12) deliberately **above the truck's measured hands-off steering
capability (~4.5 m/s² angle-rate-limited)** so the cap is never the binding constraint during the
steering-capability measurement phase — i.e. so `peakAchLat` reflects the truck's true angle-rate limit,
not this ceiling. This is a values-only retune: the fail-safe direction (flat ISO 3.0 on any
missing/corrupt file), the `[1.0, 6.0]` hard clamp, and the `LAT_ACCEL_SLEW_RATE` transition logic are
all unchanged.

## JSON tuning file

Path: **`/data/pnw/lataccel_limits.json`** — deliberately outside the git tree, for the same reason
`system/mapd`'s downloaded binary and `/data/pnw/lanecenter_tuning.json` live outside the tree: an
auto-update's `git clean` wipes untracked files inside the repo, and this file must survive that (see
`docs/mapd-binary-wiped-by-autoupdate.md`).

Format:

```json
{
  "_comment": "Speed-scheduled max lateral-accel cap for the curvature clip -- LAYER-2 safety envelope: curve-speed slowdown (VTSC/CES) is layer 1, this is the steering-authority backstop for when it doesn't fire before a curve. ACTIVE ONLY WHILE THIS FILE VALIDLY PARSES. breakpoints=[[speed_mph, accel_mps2],...], linearly interpolated, held flat outside the ends. Missing/corrupt/non-finite -> falls back to flat ISO 3.0 (NOT this schedule), gently (rate-limited, not a step). Hot-reloaded (~every few seconds).",
  "breakpoints": [[50, 6.0], [60, 5.0], [70, 4.0]]
}
```

Breakpoints are `[speed_mph, accel_mps2]` pairs, in **MPH** (not m/s) specifically so they're easy to
read/edit by hand on the road. Internally they're converted to m/s (`CV.MPH_TO_MS`) once per reload
and interpolated against `v_ego`, which is already in m/s.

**The 5/4/3 schedule is ACTIVE ONLY WHILE a valid file is loaded.** If the file is missing, unreadable,
malformed, has fewer than 2 or more than 32 breakpoints, has non-finite/out-of-range/non-strictly-
increasing values, or hasn't loaded yet (e.g. right at boot), `lat_accel_limit()` falls back to the
**flat ISO baseline `MAX_LATERAL_ACCEL_NO_ROLL` (3.0 m/s²) at every speed** — i.e. plain upstream
behavior — NOT the 5/4/3 schedule. `DEFAULT_LAT_ACCEL_BREAKPOINTS_MPH` (the same three breakpoints
above) exists only as the content best-effort-written out to disk once as a starting point to edit —
this write is wrapped in a bare `try/except OSError` and its failure is silently ignored (e.g. a
read-only `/data/pnw` in CI/tests never breaks anything, it just means the cap stays flat 3.0). On the
real device the seed write succeeds, so the 5/4/3 schedule activates within
`_LAT_ACCEL_RELOAD_INTERVAL_S` (~5 s) of boot.

### Hot-reload

- `clip_curvature()` runs at 100 Hz (called every control-loop tick), so the file is **not** parsed
  every call. It is identity-checked (`os.stat()`'s `(st_mtime_ns, st_size)`, one syscall) at most once
  every 5 seconds (`_LAT_ACCEL_RELOAD_INTERVAL_S`, gated with `time.monotonic()`), and only re-parsed
  if that identity changed since the last successful (or last failed) load — a persistently-broken
  file is still cheaply `stat()`-ed every interval but never re-opened/re-read.
- A parsed-and-sanitized schedule is cached as precomputed `numpy` arrays (`self._xs`/`self._ys`) in a
  module-level `_LatAccelSchedule` instance (`_lat_accel_schedule`) shared by every `lat_accel_limit()`
  call in the process — built once per successful reload, never rebuilt inside `limit()`, and capped at
  32 breakpoints so a malicious/typo'd file can't blow up reload cost.

### Fail-safe / robustness

This is safety-adjacent lateral-control code — the JSON file is effectively a live, on-road-editable
remote-tuning channel, so it is treated as **untrusted input**, mirroring the hardening pattern already
used by `selfdrive/controls/lib/lane_centering.py`'s `LaneCenteringController` (also hot-reloaded JSON,
also `math.isfinite()`-gated against `json.load()`'s bare `NaN`/`Infinity` parsing):

- Any of the following discards the whole file and **resets to the flat 3.0 fail-safe** (never to the
  5/4/3 schedule, and never keeping a stale last-good schedule): file missing, parse error, wrong JSON
  shape, fewer than 2 or more than 32 breakpoints, any breakpoint entry that isn't a 2-element
  `[speed, accel]` pair, a `bool`/`str` where a number is required, any non-finite (`NaN`/`Infinity`/
  `-Infinity`) speed or accel value, a negative speed, duplicate/non-increasing speeds, or an accel
  value outside the hard clamp **[1.0, 6.0] m/s²**. Validation is all-or-nothing (not per-breakpoint
  clamp-and-continue) — a schedule is a shape, and silently "fixing" one bad entry could distort the
  curve in a way that's hard to notice while driving, so a bad file is rejected wholesale.
- **No latching:** deleting a previously-valid file, or overwriting it with a broken one, reverts to
  flat 3.0 on the very next reload check — the schedule never keeps running on a value that's no
  longer backed by a validly-loaded file.
- **Gentle transitions:** the cap `limit()` returns is additionally slew-rate-limited
  (`LAT_ACCEL_SLEW_RATE = 4.0` m/s² per second of wall-clock time) toward whatever the current target
  is, so a schedule hot-swap — including the fail-safe revert to 3.0 — ramps rather than steps the
  curvature clamp in a single 10 ms control tick. Speed-driven target changes are already gradual, so
  this only meaningfully engages on abrupt schedule swaps.
  **2026-08-11 retune (0.5 → 4.0):** live telemetry from a Crown Hill ~50 mph right curve showed the
  0.5 rate couldn't keep up with the scheduled cap's speed-driven rise during hard braking into the
  curve (speed drops fast → schedule interpolates toward its looser low-speed end fast → the 0.5 slew
  lagged → the effective cap pinned near its 3.0 floor → a legitimate ~3.8 m/s² cornering demand got
  clipped mid-brake — an under-turn, not the abrupt-swap case the slew was built for). Jerk bound: at
  saturation, a cap-change rate `R` (m/s²/s) induces lateral jerk of exactly `R` (m/s³), so any
  `R ≤ MAX_LATERAL_JERK` (5.0 m/s³, ISO baseline) stays inside the ISO jerk envelope. 4.0 leaves
  ample margin under 5.0 while comfortably tracking real deceleration (which only needs ~0.5–1.0
  m/s²/s here, 4–8× headroom), and still softens the abrupt-swap case: a worst-case full-range swap
  (cap 6.0 → 1.0, the full `[1.0, 6.0]` clamp span) now ramps over ~1.25 s instead of stepping in a
  single 10 ms tick.
- `lat_accel_limit(v_ego)` itself never raises and always returns a finite float clamped to
  **[1.0, 6.0] m/s²**, regardless of what's on disk or what `v_ego` is (a non-finite/non-numeric
  `v_ego` falls straight back to `MAX_LATERAL_ACCEL_NO_ROLL` = 3.0).
- The [1.0, 6.0] clamp is hardcoded in `drive_helpers.py` and is **not** reachable from the JSON — the
  file can only tune *within* that fixed envelope, matching the "safety-envelope contract" convention
  from `lane_centering.py`.
- `controlsd.py`'s `SteerLimitStatus` telemetry (`latMax`/`curvMax`) calls the same
  `lat_accel_limit(CS.vEgo)` clip_curvature() uses, so the logged steer-limit envelope always matches
  what was actually applied that tick.

## How to tune on the road

SSH to the device and edit the file directly (or push a new one over `scp`):

```bash
ssh comma@$COMMA_IP
vi /data/pnw/lataccel_limits.json   # edit the "breakpoints" array
```

Changes take effect within `_LAT_ACCEL_RELOAD_INTERVAL_S` (5 seconds) — no reboot, no manager restart
— ramped in (worst case ~1.25 seconds for a full [1.0, 6.0] swing) by the slew limiter rather than
stepping instantly. If the edit is
invalid (typo, non-finite value, cap outside [1.0, 6.0], duplicate speeds) `controlsd`'s `cloudlog`
will log a `drive_helpers: failed to load /data/pnw/lataccel_limits.json, reverting to flat 3.0 m/s^2
fail-safe (...)` line — keyed on the file's identity (mtime/size) so a second bad edit is logged again
even if the error message repeats — and the car reverts to the flat 3.0 ISO baseline, never to a
corrupted value and never stuck on a stale schedule.

## Unrelated same-commit note: `NoFordAngleSteering` / `FordAngleLateral` comment

This commit also adds a short factual comment in `selfdrive/ui/layouts/settings/toggles.py` next to
`NoFordAngleSteering`'s `DESCRIPTIONS` entry, documenting the intentional two-param bridge: the
driver-facing opt-out toggle is `NoFordAngleSteering`, but the opendbc submodule
(`opendbc_repo/opendbc/car/pnw_vehicle.py`, in `pnw-opendbc`'s `master-pnw`) still reads the older
positive-sense `FordAngleLateral` directly, and `system/manager/manager.py` re-derives
`FordAngleLateral = not NoFordAngleSteering` on every boot to keep the two in sync (this is what fixes
a stray/stale mirror value that otherwise shows up as a "STEER: STOCK" red mismatch on the sidebar).
This is documentation-only — no behavior change — bundled into this commit because it's a small,
adjacent finding made while reading `toggles.py` for the lane-centering hot-reload reference pattern.
