# angle2pnw-faithful2 — deviation manifest

**Audience: Alan Polk.** This document lists every place our port of your bp-7.0 Ford
path-angle-primary lateral control (`opendbc/sunnypilot/car/ford/lateral_angle_ext.py`,
`human_turn.py`, `lateral_curv_ext.py`, `fordcan_ext.py`, the Ford section of
`carcontroller.py`, `opendbc/safety/modes/ford.h`, `values_ext.py` — bp-7.0 tip `19858f2888`,
`sunny/bluepilot` in our workbench) differs from your shipping code, why, and what the risk is.
Anything not on this list is a bug in our port, not a deliberate choice.

Target repo: `pnw/pnw-opendbc`, branch `angle2pnw-faithful2`, branched from `master-pnw`.

This supersedes two earlier, unsuccessful attempts in our tree:
- `angle2pnw` (first pass) — road-tested, did not perform correctly. It dropped several of your
  mechanisms (VLT, both blips) but, on inspection, its sign-negation pattern already matched your
  code. See "What we found about the previous road failure" at the end of this document.
- `angle2pnw-faithful` (uncommitted, never driven) — built from an internal design summary of your
  published articles that turned out to be wrong on several points once we actually read your
  source (notably: it assumed `shadow_curvature` shouldn't exist at all, which is incorrect — your
  code sends it). Discarded; not this port's ancestor.

---

## 1. Deviation categories (per the task brief)

- **(a)** import/layout rewiring for our tree
- **(b)** MADS absent in our fork
- **(c)** our panda reset-latch hardening + separate `FORD_PATH_ANGLE_LIMITS_ANGLE` struct, carried
  forward from `angle2pnw`, already reviewed and passed
- **(d)** capability-view rule — no `carFingerprint` branches in feature code outside the gain-table
  lookup you already do
- **(e)** substitutions for BluePilot/sunnypilot-only params, messages, or UI surfaces we don't have

## 2. Full deviation list

### (a) Import / architecture rewiring

| # | Your code | Ours | Why |
|---|---|---|---|
| a1 | Module/class paths `opendbc/sunnypilot/car/ford/*_ext.py` | `opendbc/car/ford/*_pnw.py` | Tree naming convention (matches every other pnw feature). |
| a2 | `LateralAngleExt` is mixed into `CarController` via multiple inheritance alongside `LateralCurvExt`, sharing one `self.sm` `SubMaster(['modelV2','liveParameters','selfdriveState','radarState','liveDelay'])` and one `self.model` | `CarController` uses **composition**: at most one of `LateralCurvExt` / `LateralAngleExt` is ever constructed as a separate object (matches every other pnw feature — `LongitudinalExt`, ICBM, etc. are all composed, not mixed in). `LateralAngleExt` owns its own `SubMaster(['modelV2', 'liveDelay'])` | Rewiring the whole `CarController` to multiple inheritance just for this one feature would be a much larger, riskier change than giving the class its own (smaller) `SubMaster`. **Data dependency is identical**: your `update_angle_strategy` only ever *uses* `self.model` and `self.sm['liveDelay']` from that shared object. (Precision per review finding m2: your line 338 does `LP = self.lp` — a read-and-discard of `liveParameters`; `LP` is never referenced afterwards. We dropped that dead assignment. It never *uses* `liveParameters`, `selfdriveState`, or `radarState`.) So the subscription set is scoped exactly to what the control law actually consumes; runtime behavior is unaffected. |
| a3 | `lane_change_factor_bp` / `lane_change_factor_low` live on the shared `LateralCurvExt` mixin instance | Same two constants (`[4.4, 40.23]` m/s / `0.95`, unchanged values) owned locally as module constants | Direct consequence of a2 — no mixin instance to read them from. |
| a4 | `_ensure_lateral_curv_initialized(self.CP)` compatibility shim, called at the top of `update_angle_strategy` and `update_angle_params` | Not called at all | It's a no-op (`pass`) in your file. Omitting a call to a function that does nothing has zero behavioral effect. |
| a5 | Method named `update_angle_strategy(self, CC, CS, actuators, CP)` | Named `update(self, CC, CS, actuators, CP)` | Matches `LateralCurvExt.update`'s call signature so `carcontroller.py`'s two dispatch branches read the same way. Same arguments, same order, same body. |
| a6 | `from selfdrive.modeld.constants import ModelConstants` (unguarded) | `try: from openpilot.selfdrive.modeld.constants import ModelConstants; except ImportError: from selfdrive.modeld.constants import ModelConstants` | Our tree's import-root convention (`openpilot.*`, `ruff TID251`); falls back to the bare form so opendbc-only checkouts still import without exploding at parse time. |
| a7 | `update_angle_params(self, params)` called once per **100 Hz** carcontroller frame from `CarController.update()`, separately from `update_angle_strategy` | `update_angle_params` is called from inside `update()` itself, i.e. once per **20 Hz** `STEER_STEP` tick | The only thing this method does in our port is a throttled tuning-file re-poll (see e6); it never needs to run faster than the strategy itself, which itself only runs at 20 Hz. No behavioral effect on the control law. |

### (b) MADS

Not applicable to `lateral_angle_ext.py` itself — your file has no MADS references. (Your
`ford.h`'s MADS additions are handled under category (c) below, carried from the earlier
`angle2pnw` port.)

### (c) Safety hardening carried forward from `angle2pnw` (already reviewed and passed)

`opendbc/safety/modes/ford.h` and `opendbc/safety/tests/test_ford.py` in this port are taken
**verbatim** from the `angle2pnw` branch's safety commit (`git show angle2pnw:opendbc/safety/modes/ford.h`),
diffed byte-for-byte identical against what's currently pinned in `pnw-pilot`'s `opendbc_repo`
submodule (a separate, later branch — confirms this exact C file has already circulated).
Deviations from your `ford.h`, each pre-existing and pre-reviewed:

| # | Your `ford.h` | Ours | Why |
|---|---|---|---|
| c1 | One shared `FORD_PATH_ANGLE_LIMITS` struct, widened ~10x at the call site when angle mode is engaged (a runtime-conditional widen of a compile-time table) | A **separate** `FORD_PATH_ANGLE_LIMITS_ANGLE` struct with your same numeric values, selected at the call site by `angle_mode_active` | A single C struct can't be "conditionally wide" — your version's widening also loosens the ROC for the **already-deployed curvature-mode** path_angle trim signal, which never asked for a 10x looser rate limit. A separate struct makes the currently-driven curvature path's behavior provably untouched no matter what angle mode does. Numerically identical to yours when angle mode is engaged. |
| c2 | `controls_allowed \|\| controls_allowed_lateral` (MADS) | `controls_allowed` only | This tree has no MADS state machine; `controls_allowed_lateral` doesn't exist. Strictly narrower (fewer engaged states, never more). |
| c3 | Reset-bypass latch: a single `violation` bool, zeroed unconditionally by the latch (`violation = false`) | Split into `violation` (value-range checks + shadow-curvature deviation — **never** latch-relaxable) and `roc_violation` (rate-of-change checks — the latch's only legitimate target) | Pre-existing pnw hardening (2026-07-11, before this port): your original design let the latch's ~3 s amnesty window also waive the path_angle **value-range** check, so one armed frame could pass a path_angle at the raw DBC extreme. Proven via `test_angle_mode_value_range_survives_latch_amnesty`. |
| c4 | Latch is armed by any neutral (`curvature==0 && path_angle==0`) frame regardless of engagement | Latch is force-cleared whenever `!controls_allowed` | Pre-existing pnw hardening: openpilot sends neutral frames continuously while disengaged, which under your version left the latch nearly always armed — a bypass of the disengaged-steering block. Proven via `test_reset_latch_blocked_when_disengaged`. |
| c5 | Your `rx_hook` MADS additions (`mads_button_press`, the `Steering_Data_FD1` `RxCheck`, the `acc_main_on` write riding the same diff hunk) | Not ported | `mads_button_press` doesn't exist in this tree; `acc_main_on` is not read by any angle-mode check (verified: no reference to it in `ford.h`). |
| c6 | `ford_bp_angle_mode_engaged` set unconditionally from the LKA message's corroboration bit | Only updates while `controls_allowed`; force-`false` otherwise, and every check site additionally re-requires `controls_allowed` at check-time (`angle_mode_active = ford_bp_angle_mode_engaged && controls_allowed`) | Pre-existing pnw hardening: closes a window where a stale bit from a prior engaged session, or a disengaged-but-bit-set frame, could unlock the wide path_angle range. Proven via `test_angle_mode_wide_range_requires_controls_allowed`. |

| c7 | Python sender: `angle_mode_engaged` asserted whenever angle mode is *selected* (your carcontroller line 223), independent of `latActive` | **Same as yours** — restored 2026-07-19 after review finding M1 caught our first revision narrowing it to `lat_active` (an unlisted deviation, now reverted). One addition yours doesn't need: on our never-kill-card exception fallback (when the angle strategy dies and stock curvature resumes for the drive), the bit and shadow are force-dropped to 0 — the wire is curvature again, so ford.h must stop applying angle-mode rules to it. | Fidelity restored; the fallback drop is a direct consequence of deviation-category-c's fallback discipline, which your code doesn't have. |

All six ford.h rows are pre-existing, already-reviewed hardening from the earlier `angle2pnw` work — not new
for this port — carried forward per the task's explicit permission (category c); c7 is the Python-side
counterpart added at review time. **`opendbc/safety/tests/test_ford.py`
was rebuilt and run against `libsafety.so`: 123 passed, 9000 subtests passed, 0 failed.**

### (d) Capability-view (no `carFingerprint` in feature code)

| # | Your code | Ours | Why |
|---|---|---|---|
| d1 | `update_angle_params` does `fp = getattr(self.CP, 'carFingerprint', ''); if fp in _CANFD_BOF_CARS: ...` inline, every call | `opendbc.car.pnw_vehicle.PnwVehicle.angle_gain`, computed once at `LateralAngleExt.__init__` from the identical `_canfd_bof_cars` / `_canfd_suv_cars` set membership and identical `(0.95, 0.95)` / `(1.00, 1.05)` / `(1.00, 1.15)` values | Driver directive (`~/gh/comma/CLAUDE.md`): feature code never branches on `carFingerprint`; the one place that's allowed to is the capability-view module. Zero numeric change — same sets, same values, same F-150-Lightning-gets-`(0.95, 0.95)` result. |
| d2 | Your engine selects angle-vs-curvature per frame via `self.primary_lateral_control == PrimaryLateralControl.angle`, an enum read from the `FordPrefLateralControl` param inside `LateralCurvExt` | `PnwVehicle.angle_lat` (bool, from the `FordAngleLateral` param, gated on `carFingerprint == "FORD_F_150_LIGHTNING_MK1"` inside `pnw_vehicle.py`) decides **at `CarController.__init__` time** which of `LateralCurvExt` / `LateralAngleExt` gets constructed at all | Same "pick one of two lateral strategies" concept as yours, expressed as a boolean instead of a 2-value enum because our two strategies are separate composed objects rather than branches inside one shared class. Functionally equivalent for a binary choice. |

### (e) BluePilot/sunnypilot-only params and UI substituted

| # | Your code | Ours | Why |
|---|---|---|---|
| e1 | `FordLowSpeedFactor_ang`, `FordHighSpeedFactor_ang`, `lane_change_factor_high_ang` read from openpilot `Params` every `update_angle_params` call, clipped to `[0.5, 1.5]` / `[0.5, 1.5]` / `[0.85, 1.50]` | Same three values, same clip ranges, sourced from the on-road JSON tuning overlay (`angle_tuning_pnw.py`, `/data/pnw/angle_tuning.json`) instead of openpilot `Params` keys | These keys don't exist in our params registry (no BluePilot params UI in this tree). Defaults are your exact constants (`1.0`, `1.0`, `1.0`); absent file ⇒ byte-identical to your code. |
| e2 | `path_angle_blend_ratio` (`0.50`) and `vlt_extra_max` (`0.10`) — **in-code constants in your bp-7.0** (the names `FordPathAngleBlendRatio`/`FordVLTExtraMax` appear only in your comments; `update_angle_params` reads exactly three Params keys — review finding m4) | Same defaults, additionally exposed through the JSON overlay | Correction 2026-07-19: an earlier revision of this row wrongly asserted your code reads these from Params. It does not; ours makes them tunable where yours are constants. Defaults identical, absent file ⇒ identical behavior. |
| e3 | Your settings menu (`selfdrive/ui/bp/layouts/settings/...`) and live lateral-debug screen (`angle_factor_adjuster.py`) | **Not ported. No UI surface at all.** | **Explicit driver instruction (2026-07-19 addendum):** all tuning is SSH + JSON-file + the single `FordAngleLateral` boolean param — no settings screen, no live debug overlay. Anywhere your control code reads a UI-written value, we read the JSON overlay instead (default = your default). |
| e4 | Lane Change Factor UI slider range per your **published article**: 0.5–2.0 | We follow the **code** clip range, `[0.85, 1.50]` (`update_angle_params`'s `float(clip(..., 0.85, 1.50))`) | Explicit driver instruction: the article and the code disagree on this one number; the code wins, consistent with the rest of this port's "code over articles/docstrings" rule. Recorded here as a known article-vs-code discrepancy, not something we introduced. |
| e5 | `d_ref` / `pscm_d_ref_m()` computed every frame, consumed by nothing (see "docstring is stale" below) | Computed identically, still consumed by nothing, `# noqa: F841` marker so lint doesn't flag the intentionally-unused local | Not a params substitution — listed here because it's the same "keep what the code keeps, even the seemingly dead parts" principle. Zero behavioral cost. |
| e6 | Your `update_angle_params` re-reads `Params` every 100 Hz carcontroller frame | JSON overlay is loaded once at construction and re-polled every 100 calls of `update()` (~5 s at 20 Hz), not on every control tick | **Explicit driver instruction (2026-07-19 addendum):** "this runs inside the 20 Hz lateral path... do not add a new file-stat per control tick." A file the driver hand-edits over SSH doesn't need sub-second responsiveness; ~5 s is the same order of magnitude as the existing `/data/pnw/regen.json` reload pattern already in this tree's `carcontroller.py`. |

**JSON tuning overlay — the exposed keys and their clamp ranges** (all default to your exact
bp-7.0 constants; a missing/unreadable/malformed file, or any single out-of-range/wrong-typed key,
falls back to that key's own default and never crashes — see `opendbc/car/ford/angle_tuning_pnw.py`
and its unit tests, `opendbc/car/ford/tests/test_angle_tuning_pnw.py`):

| Key | Default | Clamp range |
|---|---|---|
| `gain_lowC_highV` | 0.95 (F-150 Lightning / CAN-FD BOF pair) | 0.5–1.5 |
| `gain_highC_highV` | 0.95 | 0.5–1.5 |
| `low_speed_curv_factor` | 1.0 | 0.5–1.5 |
| `high_speed_curv_factor` | 1.0 | 0.5–1.5 |
| `lane_change_factor_high_ang` | 1.0 | 0.85–1.50 |
| `path_angle_blend_ratio` | 0.50 | 0.0–1.0 |
| `vlt_extra_max` | 0.10 | 0.0–0.5 |
| `gain_speed_lo_ms` | 13.5 | 0–50 (must stay below `gain_speed_hi_ms`, else both revert) |
| `gain_speed_hi_ms` | 26.82 | 0–50 (must stay above `gain_speed_lo_ms`, else both revert) |
| `low_speed_boost` | 1.30 | 1.0–2.0 |
| `curvature_factor_bp_lo` | 0.0007 | 0.0001–0.01 (must stay below `curvature_factor_bp_hi`, else both revert) |
| `curvature_factor_bp_hi` | 0.001 | 0.0001–0.01 (must stay above `curvature_factor_bp_lo`, else both revert) |

**Deliberately NOT exposed** (safety-relevant or structural, per explicit driver instruction): the
soft-ROC table (mirrored in panda's `FORD_PATH_ANGLE_LIMITS_ANGLE` — changing it from Python alone
desyncs the safety backstop), `FORD_DBC_PATH_ANGLE_MIN/MAX`, `_PSCM_SAT_UNWIND_RATE`, all
`_STALL_*` / `_PRESS_BLIP_MIN_S` constants, the current-curvature deviation clip /
`CarControllerParams.CURVATURE_ERROR`, and the `_PSCM_DREF_*` table. A reference copy of the
overlay file (all keys at your defaults, with a `_doc` string per key) is committed at
`opendbc/car/ford/angle_tuning.reference.json` — scp it to `/data/pnw/angle_tuning.json` and edit.

---

## 3. Constant table — his value vs ours (proof every number matches)

| Constant | His value | Ours (default, no overlay) | Match |
|---|---|---|---|
| `_GAIN_CAN` | (1.00, 1.15) | (1.00, 1.15) | ✓ |
| `_GAIN_CANFD_BOF` | (0.95, 0.95) | (0.95, 0.95) | ✓ |
| `_GAIN_CANFD_SUV` | (1.00, 1.05) | (1.00, 1.05) | ✓ |
| `FORD_DBC_PATH_ANGLE_MIN` | -0.5 | -0.5 | ✓ |
| `FORD_DBC_PATH_ANGLE_MAX` | 0.5235 | 0.5235 | ✓ |
| `_PSCM_DREF_SPEEDS_MS` | (0.0, 4.17, 27.78, 41.67, 50.0, 55.56) | identical | ✓ |
| `_PSCM_DREF_M` | (0.5, 0.95, 1.4, 2.075, 2.75, 3.875) | identical | ✓ |
| `_FORD_PATH_ANGLE_BLEND_RATIO_DEFAULT` | 0.50 | 0.50 | ✓ |
| `_DT_MDL` | 0.05 | 0.05 | ✓ |
| `_VLT_T_EXTRA_MAX` | 0.10 | 0.10 | ✓ |
| `_VLT_V_LOW_MS` | 25.0 * 0.44704 (=11.176) | identical expression | ✓ |
| `_VLT_V_HIGH_MS` | 55.0 * 0.44704 (=24.5872) | identical expression | ✓ |
| `_VLT_KAPPA_FULL` | 0.005 | 0.005 | ✓ |
| `_VLT_KAPPA_TAPER` | 0.020 | 0.020 | ✓ |
| `_PSCM_SAT_UNWIND_RATE` | 0.02 | 0.02 | ✓ |
| `_STALL_GAP_MIN` | 2.0 * CURVATURE_ERROR (=0.004) | identical expression | ✓ |
| `_STALL_HOLD_S` | 0.5 | 0.5 | ✓ |
| `_STALL_BLIP_FRAMES` | 6 | 6 | ✓ |
| `_STALL_COOLDOWN_S` | 2.0 | 2.0 | ✓ |
| `_STALL_MAX_BLIPS` | 3 | 3 | ✓ |
| `_PRESS_BLIP_MIN_S` | 0.5 | 0.5 | ✓ |
| soft ROC bp `[9,10,15,25]` → `[0.055,0.055,0.0425,0.009]` | as shown | identical | ✓ |
| gain-interp speed bp `[13.5, 26.82]` | as shown | identical default (tunable) | ✓ |
| low-speed boost `1.30` (line 446) | 1.30 | identical default (tunable) | ✓ |
| curvature_factor bp `[0.0007, 0.001]` | as shown | identical default (tunable) | ✓ |
| `CarControllerParams.CURVATURE_ERROR` | 0.002 (shared `values.py`) | 0.002 (same file) | ✓ |
| `CarControllerParams.STEER_STEP` | 5 (shared `values.py`) | 5 (same file) | ✓ |
| `HUMAN_TURN_ANGLE_DEG` | 45.0 | 45.0 | ✓ |
| `HUMAN_TURN_HOLD_S` | 1.5 | 1.5 | ✓ |
| `HUMAN_TURN_HOLD_PRETURNED_S` | 3.0 | 3.0 | ✓ |
| `BP_ANGLE_LIMITS` (curvature rate up/down tables) | `([5,16,25],[0.0025,0.0012,0.00008])` / `([5,16,25],[0.0025,0.0014,0.00018])` | identical (`values_pnw.py`) | ✓ |

Every constant listed in the task's hard-fidelity rule 1 is present and numerically unchanged.

---

## 4. Considered and rejected

These deviations were considered during the port and **explicitly rejected** — kept here so the
reasoning isn't silently lost:

- **Multiple-inheritance mixin to match your architecture exactly.** Rejected — would require
  restructuring the whole `CarController` class hierarchy in this tree, is a much larger blast
  radius than an independent `SubMaster`, and (per a2 above) produces identical data dependencies
  either way. Not worth the risk for zero behavioral gain.
- **Gating `angle_lat` on `four_signal_lat`** (so angle mode only applies where the 4-signal panda
  safety happens to also be flashed). Rejected — angle mode's panda safety (`FORD_DBC_PATH_ANGLE_MIN/MAX`,
  `FORD_PATH_ANGLE_LIMITS_ANGLE`, `ford_shadow_curvature_error_check`) is entirely self-contained
  and does not depend on the 4-signal curvature_rate checks. Tying the two together would be an
  artificial coupling with no safety benefit.
- **"Fixing" the stale module docstring** (`path_angle = 1/2 * kappa * d_ref`) to match the actual
  code. Rejected — it's your comment, not executable, and this port's job is fidelity to your
  *behavior*, not curating your comments. Our port's own docstring documents the actual behavior
  and flags the discrepancy instead.
- **Dropping the stall-blip / press-blip** since we have no F-150-Lightning-specific evidence of
  the Mach-E PSCM attenuation they exist to fix. Rejected per explicit driver instruction — the
  previous port's on-road failure is attributed to exactly this kind of "we judged it unnecessary
  and dropped it" reasoning, so this port keeps every mechanism regardless of platform-specific
  validation status.
- **Retuning the exit-blend collapse factor** (`b * 0.25`) or the blend default (`0.50`) to make
  the stale "60%→~15%" comment true. Rejected — that would be retuning your numbers based on our
  own guess about which one (the comment or the constant) is the "real" intended value. Kept your
  constant (`0.50`) exactly; flagged the comment mismatch as a finding, not corrected it.
- **A live (sub-second) JSON-overlay poll**, closer to your literal 100 Hz `Params` cadence.
  Rejected per the 2026-07-19 addendum's explicit instruction not to file-stat every control tick.
- **Exposing the soft-ROC table, `_PSCM_SAT_UNWIND_RATE`, or the stall-blip timing via the JSON
  overlay** for maximum on-road tunability. Rejected per explicit driver instruction — these are
  either mirrored in panda's C safety code (soft ROC) or safety-adjacent enough that drifting them
  from Python alone was judged not worth the tuning convenience.
- **Reverting `ford.h` to your single shared `FORD_PATH_ANGLE_LIMITS` table** for maximum textual
  fidelity to your safety file. Rejected — doing so would loosen the ROC on the already-deployed,
  currently-driven curvature-mode path_angle trim signal by ~10x, which is a real safety regression
  to ship in the name of matching a comment. This is the one place "fidelity" as literal text and
  "fidelity" as "don't change what's currently protecting the driver" pointed in different
  directions; we chose the latter, and it's explicitly permitted as category (c).

---

## 5. Issues found in Alan Polk's code (reported, not fixed)

Per the task instruction: these are reported for your information, not silently corrected or
worked around in our port.

1. **Stale module docstring.** `lateral_angle_ext.py`'s module docstring states
   `path_angle = ½ κ d_ref`. The actual code (line 451) is
   `path_angle_calc = kappa_cmd * v_ego * self.curvature_factor` — no `d_ref` term at all.
   `pscm_d_ref_m()` / `d_ref` is computed every frame and never used. Either the docstring
   describes an earlier design that `curvature_factor`'s speed/curvature gain table replaced, or
   `d_ref` was meant to be wired in and isn't. We ported the code, not the docstring, and kept the
   dead `d_ref` computation for line-by-line fidelity.

2. **Stale inline comment on the exit-blend collapse.** The comment directly above the blend-collapse
   logic reads "drop model prediction weight from 60% → ~15%" and "full b=0.60" for the normal
   case. The actual default constant, `_FORD_PATH_ANGLE_BLEND_RATIO_DEFAULT`, is `0.50` — giving a
   real collapsed value of `0.125`, not `~0.15`, and a real normal-case value of `0.50`, not
   `0.60`. This reads like the comment was written against an earlier tuning pass (`0.60`) that was
   later retuned to `0.50` without updating the comment. We kept both the comment text and the
   constant exactly as they are in your file (see "considered and rejected" above).

3. **`getattr(CS, 'lat_ctl_lim_stat', 0)`** — we don't have visibility into whether this attribute
   is actually decoded anywhere in your `carstate.py` (out of scope for the files we were given).
   Your own comment says "In angle mode, LatCtlLim_D_Stat (→ lat_ctl_lim_stat) does not fire," which
   reads as an acknowledgment that this is effectively dead/always-zero in angle mode regardless.
   Our `getattr` with the same `0` default reproduces whatever your runtime value actually is,
   whether that's a real decoded signal or a permanently-absent one — this is fidelity-preserving
   either way, just flagging it since we couldn't independently verify which case it is.

None of these are bugs we introduced, and none of them are things we changed — they're reported as
found in your shipping bp-7.0 source.

---

## 6. What we found about the previous (`angle2pnw`) port's on-road failure

The task brief's working hypothesis was that the previous port's sign convention was inverted. We
diffed `angle2pnw`'s `carcontroller.py` against bp-7.0's negation pattern specifically to check
this: **`angle2pnw`'s negation was already correct** — it negated all four LMC/LMC2 signals
(`-lat.path_offset, -lat.path_angle, -lat.apply_curvature, -lat.curvature_rate`) and
`shadow_curvature` (`-self._latext_angle.bp_kappa_cmd`) exactly like your code does, byte-for-byte
matching what this faithful port also does (see `SIGN-CONVENTION-TRACE.md`).

What `angle2pnw` verifiably *did* drop, per its own module docstring ("Deliberately trimmed vs
bp-7.0"): the Variable Lookup Time mechanism (used a fixed 0.2 s lookup instead), the stall-blip
and press-blip entirely, and it used hardcoded `b=0.15/0.60` constants instead of your tunable
`path_angle_blend_ratio` with the proper `*0.25` collapse factor. This faithful re-port restores
all of those. We cannot independently confirm dropping VLT/blips was *the* root cause of the
on-road failure (we don't have telemetry from that drive in scope here), but we can confirm the
sign convention was not the differentiator between the two ports on this specific point — both are
faithful to your code's negation pattern. This is reported honestly rather than manufacturing a
sign bug that the evidence doesn't support.

---

## 7. Real bug found and fixed during this port's own verification (not a bp-7.0 deviation)

While writing the offline smoke test, we found that `PnwVehicle.four_signal_lat` is unconditionally
`True` for the F-150 Lightning (it's a capability, not a toggle). The first draft of
`carcontroller.py`'s wiring constructed `LateralCurvExt` (checking `four_signal_lat`) *before*
checking `angle_lat`, which meant flipping `FordAngleLateral` on would have had **zero effect** —
the 4-signal path would always win the race and `LateralAngleExt` would never be constructed. Fixed
by checking `angle_lat` first and gating `four_signal_lat`'s construction on `not angle_lat`.
Verified with a new regression test,
`test_angle_mode_takes_precedence_when_toggled_on`, and manually (mocking `PnwVehicle.angle_lat =
True` and confirming `CC._latext is None` / `CC._latext_angle is not None`). This is not a
deviation from your code — your `PrimaryLateralControl` enum has no equivalent failure mode since
it's a single flag on one shared class, not two independently-constructed objects racing each
other. It's specific to this tree's composition architecture, and now fixed.
