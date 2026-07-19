# REVIEW-CURVE-LOGGER — pre-deployment review of `scripts/angle_curve_logger.py`

**Date:** 2026-07-19 · **Reviewer:** analysis session (Claude) · **Scope:** the curve-triggered
event recorder for Alan Polk angle-steering tuning, reviewed as safety-critical *measurement*
(read-only instrument; not in the control path).
**Verdict on the version as submitted: FIX FIRST — it would have produced near-zero usable data.**
**All BLOCKER/MAJOR findings below were FIXED IN PLACE** (same file, rewritten; see §Fixes applied),
re-verified with `py_compile`, `ruff` (only the intentional E402 remains), and a 9-case synthetic
unit test of `summarize()` + the slice arithmetic. The fixed version also implements the added
**multi-car (Ford Lightning + Tesla Raven)** requirement (§Multi-car).

Line numbers below refer to the ORIGINAL file as submitted (the one under review).

---

## The single most likely way the script silently produces useless data

Two independent total failures, either alone sufficient:

1. **The state machine wedges permanently once the ring buffer fills** (~10 s of uptime at the
   *actual* sample rate — see BLOCKER-2). From then on **no curve ever completes and nothing is
   ever written for the life of the process** — while the process stays alive and looks healthy.
   A short bench test passes (ring not yet full); a month in the field yields an empty dataset.
2. **The wire decode never worked at all**: the script calls `cp.update_strings(...)` but this
   tree's pure-Python `CANParser` has no such method (it is `update()`, and it takes pre-parsed
   frame lists, not capnp bytes). The `AttributeError` was swallowed **every iteration** by
   `except Exception: pass` — so every wire field would have logged as 0.0 forever, and
   `cmd_mode==0` would have set `mode_zero_pulse=True` on **every** curve, making
   `clean_for_tuning=False` for the entire month.

---

## Finding counts

| Severity | Count | Fixed |
|---|---|---|
| BLOCKER | 5 | 5 |
| MAJOR | 6 | 6 |
| MINOR | 7 | 7 |
| NOTE | 6 | n/a (documented / accepted) |

---

## BLOCKERS

### BLOCKER-1 — ring-full index freeze: no curve can ever end after warm-up
`angle_curve_logger.py:247,253,257-258` (original). `curve_start_i` and `below_since` are raw
deque indices captured as `len(ring)-1`. Once the deque is full (`len(ring)==RING_N`), **every
append evicts from the front and `len(ring)` stops changing** — so after warm-up (51 s at the
design rate; ~10 s at the real rate, BLOCKER-2):
- at trigger time `curve_start_i = RING_N-1` and `curve_len_s = (len(ring)-1-curve_start_i)*DT ≡ 0`
  → **`MAX_CURVE_S` timeout can never fire**;
- `below_since = RING_N-1` and `(len(ring)-1-below_since)*DT ≡ 0` → **`ended` can never fire**;
- → `in_curve` sticks `True` forever, the trigger never re-arms, zero records for the rest of the
  process lifetime. Failure scenario: start logger, drive 1 minute, take one curve — every
  subsequent curve of the month is lost, silently.
Even before full: any curve still in progress at the moment the ring fills freezes the same way.
**Fix:** rows carry a monotonically-increasing absolute counter (`abs_n`); all window bookkeeping
is absolute counters + monotonic timestamps; conversion to ring offsets happens only at slice time
(`idx = abs_i - (abs_n - len(ring))`). All durations are timestamp-based (`row_t - start_t`),
never `index*DT`. This is exactly the "absolute sample counters" approach the review brief
proposed, and it is provably safe: a pending curve is flushed exactly `POST_ROLL_S` after its end,
so its oldest needed sample is ≤ `PRE+MAX+OFF_HOLD+POST = 48 s < RING_S = 51 s` old — no needed
sample can ever be evicted (and the slice clamps + integrity-checks anyway).

### BLOCKER-2 — the loop samples at ~100 Hz, not the assumed 20 Hz
`angle_curve_logger.py:196-204,240`. The loop appends a row on **every** `carState` update, and
`carState`/`carControl` publish at **100 Hz** (`card.py` runs at `DT_CTRL`=0.01). Every constant
scaled by `DT`/`RATE_HZ` was therefore wrong by 5×: the 51 s ring actually held **10.2 s** (a 30 s
curve + 8 s pre + 8 s post cannot fit — guaranteed to hit BLOCKER-1 mid-curve), `CURVE_OFF_HOLD_S`
fired at 0.4 s real, `MIN_CURVE_S` at 0.16 s, pre/post-roll were 1.6 s, and traces would have been
5× the budgeted size. **Fix:** explicit time-based decimation to 20 Hz
(`row_t - last_append_t >= DT - 0.005`), plus all durations timestamp-based so even rate drift
can't corrupt the windows.

### BLOCKER-3 — post-roll accumulation loop can spin forever
`angle_curve_logger.py:263-291`. `while (len(ring)-1 - curve_end_i) < need:` — with the ring full,
both terms are constant (same mechanism as BLOCKER-1), so the wait condition **never becomes true**
and the loop appends forever: the process burns CPU indefinitely, looks alive, writes nothing. The
hacky `curve_start_i -= max(0, (len(ring) == RING_N))` adjusted only `curve_start_i` (not
`curve_end_i`, not the loop condition), could drive `curve_start_i` negative (silently mis-slicing
via Python negative indexing in `summarize`), and `below_since` was never adjusted anywhere.
**Fix:** the separate accumulation loop is gone entirely. Completed curves go on a `pend` list
`{start_abs, end_abs, end_t, timed_out}` and the **main loop** keeps running; each pending curve is
written when `now - end_t >= POST_ROLL_S`. This also removes the duplicated row-building code and
allows a new curve to trigger *during* another curve's post-roll (back-to-back curves no longer
blind the logger for 8 s).

### BLOCKER-4 — wire decode dead on arrival: wrong parser API, error swallowed
`angle_curve_logger.py:201,269`. `cp.update_strings([m.as_builder().to_bytes() ...], sendcan=True)`
— this tree's `opendbc/can/parser.py` (pure Python) has **no `update_strings`**; it has
`update(strings, sendcan=False)` and expects **pre-parsed frame lists**
`[(nanos, [(address, dat, src), ...])]` (see `can_capnp_to_list` in
`pnw-pilot/selfdrive/pandad/pandad_api_impl.py:60`), not capnp bytes. The resulting
`AttributeError` was swallowed by the bare `except Exception: pass` on **every iteration** —
wire values pinned at 0.0 forever, `mode`≡0 → `mode_zero_pulse` true on every curve →
`clean_for_tuning` false on every curve. **Fix:** frames are built directly from the drained capnp
readers (`[(m.logMonoTime, [(f.address, bytes(f.dat), f.src) for f in m.sendcan])]`) and fed to
`cp.update(entries)` — no pandad import needed, no capnp re-serialization round-trip. Additionally
a **`wire_ok` freshness flag** (message parsed within 0.5 s) is recorded per row and a
`wire_dead_frac` per curve, so a silently-dead wire decode can never again masquerade as data:
`mode_zero_pulse` is computed only over `wire_ok` rows, and wire loss alone does **not** dirty
`clean_for_tuning` (the green/yellow core is wire-independent). Verified against the tree:
`update()` at `pnw-opendbc/opendbc/can/parser.py:216`, frame filter `src != self.bus`, Ford LMC2
sent on bus 0 at 20 Hz; neither `ford_lincoln_base_pt` nor `tesla_raven_party` matches any
`get_checksum_state` prefix (`dbc.py:190`), so passive decode is never checksum/counter-rejected.

### BLOCKER-5 — stale lock file after a hard reboot can disable logging forever
`angle_curve_logger.py:172-180`. The lock holds a bare PID, is **never removed**, and staleness is
only checked with `os.kill(pid, 0)`. After any reboot the lock survives on /data; if the PID has
been **reused by any other process** (small PID space, month of reboots — near-certain
eventually), `os.kill` succeeds and the logger exits "already running" **on every subsequent start,
forever**, with only a line in a log nobody reads. Also `os.kill` raising `PermissionError` (PID
reused by a root process) was uncaught → crash at startup. **Fix:** staleness is decided by reading
`/proc/<pid>/cmdline` and requiring it to actually be *this script*; `PermissionError`/
`FileNotFoundError` are treated as stale; the lock is removed via `atexit` when we own it.

---

## MAJOR

### MAJOR-1 — no exception guard on the main loop: any transient kills a month of logging
`angle_curve_logger.py:196-315`. The loop had no top-level try: one transient capnp decode error,
one `OSError` on a write with the disk momentarily full, one hiccup in `modelV2` → process dies
permanently (nothing supervises it; `setsid`, no systemd unit). **Fix:** per-iteration
`try/except` with throttled traceback (max 1/10 s) + 0.5 s backoff; all file writes individually
guarded; `summarize` inputs guarded. Startup errors (bad import, no DBC) still fail loudly — those
are deploy-time errors that *should* abort.

### MAJOR-2 — no ignition/session gap handling: windows stitched across car-off
The original would happily join samples from before and after an ignition cycle (or a *car swap*)
into one "curve" — monotonic time keeps running, the deque doesn't care. A curve pending post-roll
at ignition-off would wait, then complete its post-roll with next-session data. **Fix:** a
`GAP_S=3 s` hole in carState = session boundary → pending curves are force-flushed (flagged
`post_truncated`), the ring is cleared, curve state reset, and the car identity re-derived
(§Multi-car). Bonus: the last curve of a drive is now *written* (truncated) instead of lost.

### MAJOR-3 — verdict thresholds: ratio band 1.05/0.95 is not Alan Polk's rule at small peaks
`angle_curve_logger.py:157-159`. A *ratio* deadband scales with the peak: at a 40° peak, ±5 % = ±2°
(reasonable); at a 6° highway peak (high speed legitimately uses small wheel angles — his own
"Ford cuts back" observation), ±5 % = ±0.3°, far inside his ±5° road-noise floor → highway curves
systematically mislabeled oversteer/understeer. AUTOTUNER-DESIGN §C.2 already specifies the
correct rule: **absolute Δ = |yellow|−|green| with a ±3° deadband**. **Fix:** summary now records
`delta_deg` and the verdict is `oversteer if Δ>+3°, understeer if Δ<−3°, else ok` (ratio still
recorded for reporting); verdict is `None` unless `green_peak > NOISE_DEG` (guard kept).
Also fixed: `round(overshoot, 4) if overshoot` dropped a legitimate ratio of exactly 0.0
(`is not None` now).

### MAJOR-4 — summary was missing fields AUTOTUNER-DESIGN needs, one of them unrecoverable
Per §C/§E of AUTOTUNER-DESIGN, missing were: `v_peak` (speed at the green peak — required for the
`w_hi` low/high-factor attribution), `kappa_meas_peak` and `a_lat_peak` (required for the
"too fast" undershoot vetting — recoverable from the trace via `yaw/v`, but the summary is the
aggregatable dataset), `delta_deg` (the update-law input), `meta.laneChangeState` (a §C.1 gate —
**not recoverable**: it was never recorded anywhere), and **the tuning vector in force** (also
**not recoverable** — reconstructing config boundaries from file mtimes is exactly what burned
DRIVE_REPORT 2026-07-19). **Fix:** all added. Every row records `lcs` (+ `a` = aEgo for the
steady-speed gate); every summary embeds a snapshot of `/data/pnw/angle_tuning.json`
(content + mtime, `None` if absent — e.g. on the Tesla) and a `lane_change` flag that also feeds
`clean_for_tuning`.

### MAJOR-5 — exit_unwind_hold detector: not sign-aware, absolute thresholds, single-sample
`angle_curve_logger.py:122-126`. Three defects: (a) it used `abs(sa)`, so a merged/split S-curve's
*opposite* lobe (cmd crosses zero before sa — always, cmd leads) false-positives as a hold;
(b) `sa > 15°` absolute misses genuine holds on high-speed curves where the whole event lives
below 15°, and the 2026-07-19 finding is precisely that the hold pins sa near the *apex* value;
(c) one noisy sample sufficed. **Fix:** detector now requires, within 3 s post-curve, `|cmd| <
NOISE_DEG` while **same-sign** `sa` exceeds `max(2·NOISE_DEG, 0.5·yellow_peak)` for a **persistent
0.25 s** (the real hold lasted >1 s; unit-tested both ways: t4 detects the 15:06:37-style hold,
t4b rejects the S-curve false positive). Remaining limitation (NOTE-4): holds on curves whose
peak is ≤ ~10° are below the reliable detection floor — accepted, that is also below Alan's noise
floor.

### MAJOR-6 — idle CPU: busy-spin at ~200 Hz around the clock
`angle_curve_logger.py:197,204-206`. `sm.update(0)` (non-blocking) + `sleep(0.005)` → ~200
wakeups/s each doing a `drain_sock`, 24/7, on a thermally-constrained device that also sits parked
on the 12 V system. **Fix:** `sm.update(100)` (blocking poll) — wakes at carState rate (~100 Hz)
onroad, **10 Hz** when the car is off. Onroad per-iteration work is a poll, a sendcan drain
(~100 small msgs/s), and one row build per 50 ms — single-digit % of one of eight cores;
acceptable for a passive instrument. (If field measurement shows otherwise, the next lever is
decoding sendcan only in the 20 Hz decimation slots — trades ≤50 ms wire staleness for CPU.)

---

## MINOR

- **MINOR-1 — `curves.jsonl` unrotated.** ~600 B/curve → a few MB/month: not a real leak, but
  unbounded on principle and the brief flagged it. Fixed: rolled once to `curves.jsonl.1` at 50 MB
  (bounded 100 MB total; rolling, never deleting, so the dataset is preserved).
- **MINOR-2 — no absolute disk-free floor.** Rotation caps *our* growth at 500 MB, but with 9 GB
  free a runaway elsewhere plus our 500 MB is uncomfortable. Fixed: `statvfs` check per write —
  below 1 GB free traces stop (summaries continue, `trace: null`); below 200 MB nothing is written.
- **MINOR-3 — S-curve summary lost the second lobe.** Hysteresis (2 s under 0.0005 1/m) merges most
  S-curves into one record; the summary kept only the dominant-sign peak. Fixed: summary now also
  records `green_peak_opp_deg`/`yellow_peak_opp_deg` (unit test t5), and the full trace was always
  there. Merged is the *right* shape for peak tuning (each lobe judged at its peak, offline, from
  one trace); a slow S-transition (>2 s near zero) still correctly yields two records.
- **MINOR-4 — `MAX_CURVE_S` timeout record wasn't flagged.** A timed-out curve's "peak" may be
  mid-curve and a follow-on record covers the continuation (re-trigger is immediate) — analysis
  must know. Fixed: `flags.timed_out`, which also excludes the record from `clean_for_tuning`.
- **MINOR-5 — post-roll rows nulled `lat_delay`/lane fields.** The original's duplicated post-roll
  row builder hardcoded `lane_l/lane_r/lat_delay = None`, degrading exactly the window where the
  exit-hold and lane-excursion live. Fixed by construction (single row builder everywhere). Summary
  `lat_delay` is now the median over the curve, not a single peak-instant sample.
- **MINOR-6 — `dir_size()` dead code** (never called). Removed.
- **MINOR-7 — pycapnp enum robustness.** `int(md.meta.laneChangeState)` can raise on some pycapnp
  versions (DynamicEnum); now `int(getattr(x, "raw", x))`, and the modelV2 block was already
  exception-guarded so lanes survive regardless.

---

## NOTES (accepted behavior, documented)

- **NOTE-1 — trigger on `actuators.curvature` is correct for this instrument.** The 2026-07-19
  report refuted `curv_cmd` as a *delivery metric*; as a *trigger* it is the right signal: it is
  Alan Polk's own regime boundary (`curvature_factor_bp_hi = 0.001`), it defines "openpilot is
  commanding a curve" (the only curves with a green line), and triggering on measured yaw instead
  would fire on every hand-driven intersection turn — burning the 500 MB budget on records with no
  tuning value. Consequence, deliberate: **hand-driven (disengaged) curves are not captured**
  (actuators ≈ 0 when disengaged). Partial disengagement mid-curve *is* captured and flagged
  (`not_engaged`). Triggering on `cmd_sa` instead would make the threshold speed-dependent for the
  same lateral geometry; κ is the speed-invariant choice and matches his gain switch exactly.
- **NOTE-2 — peak-magnitude comparison + full traces is sufficient for the lag question.** Alan is
  explicit the lines never overlay and judgment is peaks-only; comparing magnitudes sidesteps lag
  by design (AUTOTUNER §C.2). For lag-corrected offline work, the 20 Hz trace (pre-roll + curve +
  post-roll, green/yellow/wire/yaw per row) supports per-curve cross-correlation; `lat_delay`
  (median per curve + per row) sanity-checks it. What this logger *cannot* provide is §C.2's
  whole-drive cross-correlation over all hands-free driving — it only keeps curve windows. If that
  estimator is ever required, it belongs in the rlog-based offline analyzer, not here.
- **NOTE-3 — control-path isolation CONFIRMED.** The script only: subscribes (`SubMaster` on
  carState/carControl/modelV2/liveDelay; `sub_sock("sendcan")` — msgq is multicast pub/sub, a
  subscriber cannot consume or delay the panda's copy), *decodes* sendcan with a passive
  `CANParser`, reads two params and one JSON file, and writes under `/data/dirk/angle_curves/`.
  No `pub_sock`, no `Params().put`, no CAN TX, no writes anywhere else. The only physical coupling
  to driving is CPU/thermal (MAJOR-6, addressed).
- **NOTE-4 — detection floors.** Curves whose green peak never clears 5° produce `verdict: null`
  (recorded, not judged), and exit-holds on ≤10° peaks are undetectable — both are inside Alan's
  own noise floor.
- **NOTE-5 — forced flush conservatism.** A pending curve force-flushed at a session gap is flagged
  `post_truncated` even if its post-roll happened to be complete; the trace length disambiguates.
  Curves pending when the process is SIGKILLed (not ignition — process kill) are lost; accepted.
- **NOTE-6 — the stdout log** (`/data/dirk/angle_curves.log`, shell-redirected) grows ~150 B/curve
  plus throttled errors — a few MB/month worst case. Not self-rotated (can't rotate an inherited
  fd sanely); if it matters, restart the logger monthly or redirect to `logrotate`-managed path.

---

## Multi-car support (added this pass, per the follow-up requirement)

One 3X moves between the **Ford F-150 Lightning** and the **Tesla Model S HW3 (Raven)**. Both are
`steerControlType=angle` (verified: `opendbc/car/tesla/interface.py`, `ford/interface.py`), so the
green/yellow core (`actuators.steeringAngleDeg` vs `carState.steeringAngleDeg`) is car-agnostic and
identical on both. Implementation:

- **Runtime car identity from CarParams** — `Params().get("CarParams")` (live) falling back to
  `"CarParamsPersistent"`, parsed via `messaging.log_from_bytes(..., car.CarParams)` (the same
  pattern as `ui/onroad/model_renderer.py:75` / `ui_state.py:193`). No hardcoded fingerprint. The
  brand-specific part is a single 2-row `WIRE_SPECS` table keyed on `CP.brand` (per the
  capability-view spirit: one table, no scattered conditionals; acceptable here because this is an
  offline instrument, not feature code). DBC name comes from the live fingerprint through
  `opendbc.car.<brand>.values.DBC[fp][bus_key]`.
- **Car change without restart** — chosen over exit-and-restart because *nothing supervises this
  process*; exiting would strand logging until a human intervenes, violating the month-unattended
  requirement. The car is re-derived (a) at every session gap (which a physical swap necessarily
  produces) and (b) every 60 s while carState is absent (covers late fingerprinting after boot).
  On change: pending curves force-flushed, ring cleared, parser rebuilt, session marker emitted.
- **Every summary and every session marker** (start / car_change lines in `curves.jsonl`) carries
  `{brand, fp, steer_type}` — mixed-fleet data is separable by construction; the two cars are never
  pooled.
- **Graceful degradation** — unknown car (MOCK, mid-fingerprint, no CarParams): `WIRE_SPECS` miss
  or parser-build failure → wire fields null, `wire_ok:false`, green/yellow logging continues;
  wire loss does not dirty `clean_for_tuning` (unit test t6). Parser construction, feeding, and
  car loading are all individually exception-guarded — no path refuses to log.
- **Tesla path — VERIFIED in-tree (not assumed):** the Raven carcontroller
  (`tesla/carcontroller.py:52-54`, `LEGACY_CARS` branch) sends `DAS_steeringControl` (addr 1160)
  via `TeslaCANRaven.create_steering_control` (`teslacan_legacy.py:19-29`) on `CANBUS.party = 0`,
  packing `DAS_steeringAngleRequest = -angle` **[deg]** (negated, same negate-at-the-wire pattern
  as Ford) and `DAS_steeringControlType` (0 = not enabled). Signals confirmed present in
  `opendbc/dbc/tesla_raven_party.dbc:85-91`, which is the `Bus.party` DBC for `TESLA_MODEL_S_HW3`
  (`tesla/values.py:109-118`). `tesla_raven_party` matches no checksum-state prefix in
  `opendbc/can/dbc.py:190` (only `tesla_model3_party` does), so the passive parser applies no
  checksum/counter rejection to it. Send rate: every 2nd 100 Hz frame → 50 Hz (spec table).
  There is no c2/c3/path-offset equivalent on this message → those fields are null on Tesla.
- **What remains ASSUMED for Tesla (verify on first Tesla drive):** (a) `carState.yawRate` may be
  unpopulated on the Raven (no `yawRate` assignment found in `tesla/carstate.py`) → `kappa_meas_peak`
  / `a_lat_peak` would read 0/None on Tesla; recoverable offline from `sa` via steer-ratio/wheelbase
  if needed. (b) `str(CP.steerControlType)` renders as `'angle'` on the device's pycapnp (worst
  case it renders verbosely; the field still distinguishes, and brand/fp are authoritative).
  (c) The first Tesla curve summary should be eyeballed for wire sign/magnitude sanity
  (`wire_angle` ≈ −`cmd_sa` within rate limits, in degrees).
- **Sign conventions documented in the file header** (requirement 6): wire values recorded AS-IS,
  Ford `LatCtlPath_An_Actl` [rad] = −internal path_angle; Tesla `DAS_steeringAngleRequest` [deg]
  = −commanded angle. Never compare `wire_angle` across brands (rad vs deg, different hardware).

---

## Fixes applied (complete list of changes to `scripts/angle_curve_logger.py`)

The file was rewritten in place. Every change:

1. Absolute sample counters (`abs_n`) + monotonic timestamps replace all raw deque indices;
   ring-offset conversion only at slice time, with clamping and an evicted-window integrity check
   (BLOCKER-1/-3).
2. Explicit 20 Hz time-based decimation of the 100 Hz carState stream; all durations
   timestamp-based (BLOCKER-2).
3. Post-roll rewritten as a `pend` queue serviced by the main loop; the nested accumulation loop
   and its duplicated row builder are gone; curves can trigger during another's post-roll
   (BLOCKER-3).
4. sendcan decode via `cp.update()` fed with frame tuples built from the drained capnp readers;
   per-row `wire_ok` freshness + per-curve `wire_dead_frac` (BLOCKER-4).
5. PID-reuse-proof lock (`/proc/<pid>/cmdline` check, `PermissionError` handled, `atexit` cleanup)
   (BLOCKER-5).
6. Per-iteration exception guard with throttled traceback; all writes individually guarded
   (MAJOR-1).
7. Session-gap (3 s) detection: force-flush, ring clear, state reset, car re-derivation (MAJOR-2).
8. Verdict on absolute `delta_deg` with ±3° deadband; ratio kept as a reported quantity; `0.0`
   ratio no longer dropped (MAJOR-3).
9. Summary fields added: `v_peak_ms`, `kappa_meas_peak`, `a_lat_peak`, `delta_deg`,
   `green/yellow_peak_opp_deg`, embedded tuning-JSON snapshot, `lat_delay` median; row fields
   added: `a` (aEgo), `lcs` (laneChangeState), `wire_off` (Ford path offset), `wire_ok`; flags
   added: `lane_change`, `timed_out`, `post_truncated`, `wire_dead_frac` (MAJOR-4, MINOR-3/-4/-5).
10. Exit-hold detector: sign-aware, peak-relative threshold, 0.25 s persistence (MAJOR-5).
11. `sm.update(100)` blocking poll; idle cost 10 Hz when the car is off (MAJOR-6).
12. `curves.jsonl` 50 MB single-generation roll; `statvfs` free-space floors (1 GB traces /
    200 MB everything) (MINOR-1/-2).
13. Multi-car: `WIRE_SPECS` table, `CarCtx`, `load_car`, session markers with `{brand, fp,
    steer_type}` in every summary (§Multi-car).
14. Removed dead `dir_size()`; pycapnp enum-robust `lcs` read (MINOR-6/-7).

**Verification:** `python3 -m py_compile` clean; `ruff check` → only the 3 intentional E402s
(sys.path block before imports, required for detached setsid runs); synthetic unit suite (9 cases:
left/right sign + verdicts, noise floor, exit-hold positive + S-curve negative, S-curve lobes,
wire-null cleanliness, dirty flags, absolute-counter slicing across 1980 evictions and a
mid-stream ring clear) — ALL PASS. API usage verified against the live trees:
`pnw-opendbc/opendbc/can/parser.py` (`update`, frame-tuple format, bus filter, checksum states),
`pnw-pilot/cereal/messaging/__init__.py` (`drain_sock`, `sub_sock`, `log_from_bytes`,
`SubMaster.update`), `ford_lincoln_base_pt.dbc` (all five LMC2 signals incl.
`LatCtlPathOffst_L_Actl`, `VehYaw_W_Actl` in rad/s), `tesla_raven_party.dbc` +
`teslacan_legacy.py` + `tesla/carcontroller.py` (Raven wire path).

**Not runtime-tested on the device.** Before trusting a month of collection: run it on the 3X for
one short drive with 2-3 deliberate curves, then confirm (1) `curves.jsonl` gained a session marker
with the correct brand/fp and one summary per curve, (2) the trace row count ≈ 20 Hz × window
seconds, (3) `wire_dead_frac` ≈ 0 while engaged, (4) a second start while running exits
"already running", and a start after `kill -9` + reboot does NOT.
