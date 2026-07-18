# UI-CPU-TRIM.md — trimming the on-device UI process CPU load

**Status:** 🆕 investigation + proposal (2026-07-13). **Nothing here is deployed.** Read-only device
inspection + code audit against `pnw/pnw-pilot`. A review-ready draft of the recommended first PR
lives on scratch branch **`uicpu2pnw`** (worktree `pnw/wt-uicpu`) — do NOT merge/deploy without the
driver's approval.

Driver's question, answered up front: **"Do we need to write some C code or what?"**
→ **No.** The hot path that the PNW overlay stack adds is **Python**. Optimize Python. A C rewrite
would only touch the base camera/model draw (already GL-accelerated and gated to the 20 Hz data
rate), which is not where the PNW-added cost lives. See §1.

---

## 0. TL;DR

- The UI is a **Python raylib** app (`pyray` bindings) running a **60 FPS** immediate-mode loop on
  the 3X. Every frame, `ui_state.update()` runs and every on-screen widget's
  `_update_state()`/`_layout()`/`_render()` runs in Python. GPU primitives (`draw_text_ex`,
  `draw_texture`, camera EGL blit) are C inside raylib, but the per-frame orchestration is Python.
  **→ C is not required to reclaim the PNW-overlay cost.**
- **Honest finding:** the big PNW overlays are **already well-optimized**. `ces_status.py` and
  `location_services_status.py` already poll their mem-params at **5 Hz** and cache the built layout
  (`_REFRESH_S = 0.2`, `self._cached_layout`), redrawing cheaply per frame. `speed_limit.py` and the
  base `model_renderer.py` gate their recompute on `sm.updated[...]` (20 Hz). So the "recompute
  layout every frame" hypothesis is **mostly already addressed**.
- The **verifiable residual per-frame waste** is **filesystem param reads on the 60 Hz path**:
  `exp_button.py` (3 reads/frame) + `ui_state.py` (3 reads/frame) = **~6 `Params.get` file reads
  per frame ≈ 360 reads/sec**. These values change only when the driver taps a toggle. This is the
  cleanest, lowest-risk trim.
- The **single largest lever** (UI at 60 FPS while the camera + model only produce new content at
  20 Hz, so ~2 of every 3 rendered frames are visually identical) is **explicitly excluded** by the
  task ("don't reduce the driving-view frame rate"). Documented in §5 as a flagged option, not
  recommended without driver sign-off.

Set expectations: the in-constraint Python trims are **modest** (single-digit % of the busy core),
because the previous engineer already did the high-value caching. The one change that would *halve*
UI CPU is the FPS cap, which is off the table per the task.

---

## 1. Architecture — is C needed?

**The render loop is Python.** `selfdrive/ui/ui.py:24` drives:

```python
for should_render in gui_app.render():   # generator in system/ui/lib/application.py
    ui_state.update()                    # ui_state.py:109 — runs EVERY iteration
    ...
```

`gui_app.render()` (`system/ui/lib/application.py:575`) is a Python generator loop. Per iteration it
`rl.begin_drawing()`, then for each top nav-stack widget calls `widget.render(...)`
(`application.py:616`). Target FPS: `_DEFAULT_FPS` (`application.py:25`) = **60** on the 3X
(`tici`/`tizi` map gives 20 only for `tizi`; the 3X is `tici` → 60). `rl.set_target_fps(60)` caps it
(`application.py:321`).

`Widget.render()` (`system/ui/widgets/__init__.py:106`) runs, **every frame**:
`_update_state()` → `_layout()` → `_render()` → `_process_mouse_events()`. So every widget's Python
update+draw code runs at 60 Hz. (This is the known `_layout`-per-render pattern.)

**What is C vs Python:**
- **C (raylib / GL, cheap):** the actual `rl.draw_text_ex`, `rl.draw_texture`, `rl.draw_circle_*`,
  and the camera **EGL zero-copy blit** (`cameraview.py` `_render_egl`, larch64 path — texture is
  re-imaged only when `frame.idx` changes, i.e. throttled to new camera frames).
- **Python (per-frame, where PNW cost lives):** `ui_state.update()`, all widget `_update_state`/
  `_render` bodies, param reads, string building, numpy geometry, layout math.

**Conclusion:** the PNW overlay stack (CES / location / speed-limit / confidence-ball / BSM) is
**pure Python per-frame work**. Reclaiming it is a Python optimization task. **No C required.** A C
rewrite would only benefit the base cameraview/model render, which is already GL-accelerated and
gated to 20 Hz — low ROI, and not the PNW-added cost. Answer to the driver: **Python, not C.**

---

## 2. Where the CPU actually goes (per-frame audit)

Overlays are hosted in `selfdrive/ui/onroad/augmented_road_view.py:95-102`, drawn every frame in
this order: `model_renderer` → `hud_renderer` → `confidence_ball` → `speed_limit` → `ces_status` →
`location_services` → `alert_renderer` → `driver_state`. Plus a per-frame `uiDebug` publish
(`augmented_road_view.py:116`, upstream).

| Widget | File | Update cadence | Per-frame cost | Verdict |
|--------|------|----------------|----------------|---------|
| CES status/debug | `onroad/ces_status.py` | **5 Hz** poll of `/dev/shm/params`, layout cached (`_update_state:68`, `_cached_layout:60`) | redraw ~10-15 text lines; **re-measures each line every frame** (`_render:249`) | already throttled; tiny residual (§3, T2) |
| Location "Happening Ahead" | `onroad/location_services_status.py` | **5 Hz** poll, full wrapped layout cached (`_build_layout` at poll, `_update_state:75`) | redraw ~4-8 text lines | already throttled; fine |
| Speed limit / warning | `onroad/speed_limit.py` | gated on `sm.updated["mapdOut"]` (`_update_state:87`) | draw a sign (circle/rect + 1-3 texts) every frame | fine |
| Confidence ball (ball2pnw) | `onroad/confidence_ball.py` | **every frame** recomputes `list()/max()` over modelV2 probs (`_update_state:37`) though modelV2 is 20 Hz | 1 gradient circle draw | minor waste (§3, T3) |
| Exp/CES button | `onroad/exp_button.py` | **every frame**: 3 param **file reads** (`_update_state:52-55`) | 1-2 texture draws | **per-frame param reads (§3, T1)** |
| ui_state (global) | `ui_state.py` | **every frame**: 3 param **file reads** (`_update_state:143,145,146`) | — | **per-frame param reads (§3, T1)** |
| Model path/lanes | `onroad/model_renderer.py` | projection gated on `sm.updated['modelV2']`/radar (`_render:112`) | redraw cached polygons | base cost, already gated |
| Camera | `onroad/cameraview.py` | `conflate=True` recv, EGL image cached by `frame.idx` | GL blit | base cost, already gated |

**BSM note:** blind-spot indicators are referenced only in `selfdrive/ui/mici/onroad/alert_renderer.py`
(the small "mici" display variant). On the 3X **big UI** road view there is no separate BSM overlay
widget in `augmented_road_view` — BSM does not appear to be a measurable UI cost on the 3X. (Stated
with medium confidence — worth a quick confirm, but I found no big-UI BSM draw path.)

**Device snapshot (read-only, 2026-07-13, parked):** `selfdrive.ui.ui` at **44% CPU**, `loadavg
5.14`. No `py-spy` on device; `PROFILE_RENDER` requires a UI restart (not done — must not touch
running state). Numbers in §3 are estimated from cadence analysis, not a live profile.

---

## 3. Concrete trims (ranked by CPU-saved / risk)

All are **UI-only**; none touch the driving/control/safety path. New behavior is invisible to the
driver except lower CPU (and for T1, an instant-tap mitigation preserves button responsiveness).

### T1 — Stop reading params from the filesystem every frame ★ recommended first PR

**Problem.** Two widgets read `/data/params` files on the 60 Hz path:
- `exp_button.py:52-55` (`_update_state`): `ConditionalExperimentalSwitching`, `ExperimentalMode`,
  `CESButtonState` — 3 reads/frame.
- `ui_state.py:143,145,146` (`_update_state`): `RecordAudio`, `IsMetric`, `AlwaysOnDM` — 3
  reads/frame.

≈ **6 `Params.get` file opens/reads per frame ≈ 360/sec.** Each is a file open+read+parse in Python.
Every one of these values changes only on a settings toggle or a button tap — never continuously.

**Fix.** Throttle both blocks to ~2 Hz (a `time.monotonic()` gate, the same pattern the overlays
already use), or fold `ui_state`'s three into the existing `update_params()` (already throttled to
5 s at `ui_state.py:114`). For `exp_button`, keep the button visually instant by updating the cached
`_ces_button` **in the tap handler** right after `self._params.put("CESButtonState", nxt)`
(`exp_button.py:68`) so the icon never waits for the next poll.

- **Current cadence:** 6 param reads / frame (60 Hz).
- **Proposed cadence:** 6 param reads / ~0.5 s (2 Hz) + instant on tap.
- **Expected saving:** the bulk of the param-IO overhead — low single-digit % of the busy core
  (honest: param reads are page-cached, so this is syscall+Python overhead, not disk).
- **Risk:** **very low.** Values change rarely; the tap-handler write-through removes any perceptible
  lag on the CES button. Pure UI.
- **Locations:** `selfdrive/ui/onroad/exp_button.py:47-55` + `:57-68`; `selfdrive/ui/ui_state.py:118-146`.

### T2 — Cache the measured text width in the CES/location layout tuples

**Problem.** `ces_status.py:_render` calls `measure_text_cached(font, text, _FS)` **per line, per
frame** (`ces_status.py:249`) purely to right-align — even though the layout (and thus the text) is
only rebuilt at 5 Hz. `measure_text_cached` is LRU-cached, so this is a dict hash+lookup per line per
frame, not a re-measure, but it's still avoidable.

**Fix.** When building `_cached_layout` at poll time (`ces_status.py:110-114`), store the measured
width alongside each line: `(text, color, font, width)`. `_render` then just uses the cached width.
(`location_services_status.py` already left-aligns, so it needs no per-line measure — nothing to do
there.)

- **Current → proposed:** N `measure_text_cached` lookups/frame → 0 (moved to the 5 Hz poll).
- **Expected saving:** small (~sub-1% of the core), but free and zero-risk.
- **Risk:** none (UI-only, identical pixels).
- **Location:** `selfdrive/ui/onroad/ces_status.py:110-114` (build) + `:248-251` (render).

### T3 — Gate confidence_ball recompute on new model data

**Problem.** `confidence_ball.py:_update_state` (`:37`) builds `list(dp.brakeDisengageProbs)` /
`list(dp.steerOverrideProbs)` and takes `max()` **every frame**, though `modelV2` only updates at
20 Hz — so ⅔ of the work is on stale data.

**Fix.** Early-return unless `ui_state.sm.updated['modelV2']` (still call `self._filter.update()`
with the last conf so the FirstOrderFilter keeps its 60 Hz smoothing cadence — feed it the cached
`conf`, only recompute `conf` on new model data).

- **Current → proposed:** list/max every frame → every 3rd frame.
- **Expected saving:** small (lists are short); tidy.
- **Risk:** very low (keep the filter ticking at 60 Hz with the held value so the ball motion stays
  smooth).
- **Location:** `selfdrive/ui/onroad/confidence_ball.py:37-47`.

### T-higher-effort — render static overlays to a texture, blit each frame

The overlays clear+redraw every frame because raylib is immediate-mode (no dirty regions). The
overlays' *content* changes at ≤5 Hz. One could render each overlay to a `RenderTexture` at poll
time and blit the texture each frame (one `draw_texture` vs ~15 `draw_text_ex`). This is the only
change that meaningfully cuts the PNW overlays' *draw* cost, but it adds RenderTexture lifecycle
complexity and a per-overlay GPU texture. **Not recommended as a first step** — higher risk/effort
than T1-T3, and text-drawing 15 short lines is not the dominant cost. Listed for completeness.

---

## 4. What NOT to do

- **Do not lower the driving-view frame rate or alert responsiveness** (task constraint). §5 explains
  why the FPS cap is the biggest lever and why it's nonetheless excluded here.
- **Do not touch** `model_renderer`/`cameraview`/`hud_renderer` update gating — already correct.
- **Nothing here touches** control, planner, panda, or safety. All trims are UI-process-local.

---

## 5. The elephant: 60 FPS UI over 20 Hz data (excluded by constraint)

The camera (`roadCameraState`) and the model (`modelV2`) publish at ~20 Hz. At 60 FPS the UI redraws
each camera frame / model plan **three times** — ~⅔ of rendered frames are visually identical. A cap
at, say, 30-40 FPS would roughly **halve** the UI's render CPU while staying ≥ the 20 Hz data rate.

This is deliberately **left out of the recommendation** because the task says not to reduce the
driving-view frame rate, and because 60 FPS buys: smoother touch feedback, smoother confidence-ball
motion, and snappier blink/alert animation. If the driver ever wants the big reclaim, this — not the
Python micro-trims — is the lever, and it's a one-line change (`FPS` env / `init_window(fps=...)`,
`application.py:266/321`). **Requires explicit driver sign-off; not in the recommended PR.**

---

## 6. Recommended first PR

**Ship T1 alone first** (highest saving / lowest risk, self-contained, easy to verify): throttle the
per-frame param reads in `exp_button.py` and `ui_state.py` to ~2 Hz with a tap-handler write-through
for the CES button icon. Then fold in T2 + T3 (both trivial, zero-risk) as a small follow-up.

**Verification:** on-device, offroad → onroad, watch `top`/`ps` `%CPU` on `selfdrive.ui.ui` before/
after; confirm the CES button icon still flips instantly on tap, IsMetric/units still correct, speed
limit + overlays unchanged. Gemini-review the diff (escape `@`).

**Draft:** branch `uicpu2pnw` (worktree `pnw/wt-uicpu`) carries the T1 change, review-ready,
**uncommitted to any channel branch**.
