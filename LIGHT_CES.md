# LIGHT_CES.md — CES 3-way selector (Off / Light / Standard) + grey-out fix

Branch: **`light-ces-gentle`** (off the CES/VTSC `ces2xnor` lineage). Builds on commit
`74ae0ed436` which added the per-car "gentle" CES/VTSC profile. This change turns that gentle
profile into a **user-selectable mode** and replaces the CES master bool with a 3-way selector.

**Status: code + unit tests + docs only. NOT deployed, NOT driven. Behavior-neutral when Off
(the default for every car).**

---

## 1. The 3-way selector (replaces the master bool toggle)

The CES setting in `selfdrive/ui/layouts/settings/toggles.py` is now a **3-button selector**
(`multiple_button_item`, the same widget/style as "Driving Personality"):

```
Conditional Experimental Switching (CES)     [ Off ] [ Light ] [ Standard ]
```

- Buttons left→right: **Off / Light / Standard**.
- Placed **directly below the "Experimental Mode" toggle** (inserted right after `ExperimentalMode`
  in the build loop, mirroring how `LongitudinalPersonality` is inserted after `DisengageOnAccelerator`).
- Available on **ALL cars** (Tesla, F-150 Lightning, everything) — not fingerprint-gated for
  visibility. Only greyed out when openpilot does not control longitudinal (see §4).
- Backed by the new INT param **`CESMode`** (`0=Off, 1=Light, 2=Standard`), default `0`.
- Callback `_set_ces_mode` writes `CESMode` and **mirrors** the legacy bool
  `ConditionalExperimentalSwitching = (CESMode > 0)` so any back-compat reader stays consistent.

The old `ConditionalExperimentalSwitching` entry was removed from `_toggle_defs` (it was a
`toggle_item`); the selector is built explicitly as `self._ces_mode_setting` and registered in the
toggles dict under the key `"CESMode"`.

---

## 2. Param migration: bool → INT `CESMode`

`common/params_keys.h` (additive — no other entries removed):

```cpp
{"ConditionalExperimentalSwitching", {PERSISTENT, BOOL, "0"}},  // legacy master bool (back-compat; superseded)
{"CESMode", {PERSISTENT, INT, "0"}},                            // 0=Off 1=Light 2=Standard. Source of truth.
```

`CESMode` is the **source of truth**. Migration / back-compat is centralized in one pure helper,
`read_ces_mode(params)` in `ces_xnor_constants.py`:

- reads `CESMode` (INT);
- if `CESMode` is missing/`0` **but** the legacy bool `ConditionalExperimentalSwitching` is set,
  it returns **Standard (2)** — so an existing install that had CES "on" keeps working as the
  default tune after the upgrade;
- any failure ⇒ `Off (0)`.

Both runtime readers now use this helper (they used to each `get_bool("ConditionalExperimentalSwitching")`):

- `ces_xnor.py` → `CESController._read_params()`
- `vtsc_controller.py` → `VTSCController._read_enabled()`

Two more pure helpers express the mapping (unit-tested):

```python
ces_enabled(mode)   # mode > 0           -> CES + VTSC run at all
ces_is_gentle(mode) # mode == 1 (Light)  -> gentle profile; Standard/Off -> default tune
```

**The on-screen 3-state button is unchanged.** `CESMode` picks the *profile/aggressiveness*; the
top-right `CESButtonState` button (0=CES-auto, 1=forced Chill, 2=forced Experimental) still cycles
Chill↔CES↔Experimental within whatever mode is selected. Both Light and Standard go through the
exact same button/state-machine path.

### Debug overlay (ces_status.py)

`selfdrive/ui/onroad/ces_status.py` previously gated on `get_bool("ConditionalExperimentalSwitching")`.
It now gates on `ces_enabled(read_ces_mode(params))`, so the debug overlay shows for **BOTH Light
and Standard** (any non-Off) whenever the top-right button is in CES-auto mode (the existing
`button != 0 → hide` rule is unchanged). Light and Standard publish `CESStatus` identically, so the
overlay triggers for both.

---

## 3. Light vs Standard semantics ("full gentle behavior")

Selection is now driven by **`CESMode`, NOT `carFingerprint`** — the gentle profile is a user
choice on any car.

| Mode | CES enabled | VTSC tune | CES curve handling | Dwell |
|------|-------------|-----------|--------------------|-------|
| **0 Off** | no | — | — | — (behavior-neutral) |
| **1 Light** | yes | `GENTLE_PROFILE` (soft decel, slow recovery, anti-sawtooth) | curves **suppressed** in the CES decision → handed entirely to VTSC (no chill↔experimental flapping) | gentle (longer): `GENTLE_EXP/CHILL_MIN_DWELL_S` |
| **2 Standard** | yes | `DEFAULT_PROFILE` (today's tune) | curves **trip Experimental** (normal) | normal: `EXP/CHILL_MIN_DWELL_S` |

Implementation:

- **VTSCController**: `self.tune = GENTLE_PROFILE if ces_is_gentle(mode) else DEFAULT_PROFILE`,
  re-selected each ~1 Hz read in `_read_enabled`.
- **CESController**: `_set_mode(mode)` sets `self._gentle = ces_is_gentle(mode)` and **rebuilds the
  state machine only when the gentle flag flips** (so the dwell isn't reset every 1 Hz read).
  `self._gentle` already drives the curve-suppression line in `experimental_request`
  (`{**toggles, "curves": False}` when gentle), so Light suppresses curves and Standard does not.

The `GENTLE_PROFILE` / `DEFAULT_PROFILE` constants are unchanged.

### Removed gating

`GENTLE_FINGERPRINTS` (in both `vtsc_constants.py` and `ces_xnor_constants.py`) is **no longer
read** — the profile is chosen by `CESMode`, not by `carFingerprint`. The constant is kept (marked
`# unused — superseded by CESMode`) purely as a historical note of which car (F-150 Lightning)
originally motivated the gentle tune. It can be deleted later with no behavior change.

Net effect vs the previous branch: a Tesla in **Light** now gets the gentle tune (it never did
before); a Lightning in **Standard** now gets the default tune (it used to be forced gentle).
Default for every car is **Off**, so nothing changes until the user opts in.

---

## 4. Grey-out fix (CES group symmetry)

**Bug:** CES + its sub-options grey out when openpilot longitudinal control is OFF, but on turning
long control back ON not all of them re-enabled. The old code re-enabled **only** the single
`ConditionalExperimentalSwitching` toggle:

```python
ces_long_ok = cp is not None and cp.openpilotLongitudinalControl
self._toggles["ConditionalExperimentalSwitching"].action_item.set_enabled(ces_long_ok)
```

There was no coherent "CES group" — the new selector (and any future CES list item) would not be
re-enabled symmetrically.

**Fix:** one coherent block driven by a single pure helper, applied to the whole group:

```python
CES_GROUP = ("CESMode", "CESCurves", "CESStops", "CESLowSpeed", "CESLead")

def ces_group_enabled(cp) -> bool:           # pure, unit-tested
    return cp is not None and bool(getattr(cp, "openpilotLongitudinalControl", False))

ces_long_ok = ces_group_enabled(cp)
for param in self.CES_GROUP:
    item = self._toggles.get(param)
    if item is not None:
        item.action_item.set_enabled(ces_long_ok)
```

Because the **same** bool both disables (long-off) and enables (long-on) every item in the group,
re-enabling can never be partial. (Note: in this layout only the `CESMode` selector is an actual
list item today; `CESCurves/CESStops/CESLowSpeed/CESLead` are params, not list items. They're
listed in `CES_GROUP` anyway so any future CES list item is gated symmetrically by construction —
`self._toggles.get()` safely skips keys that aren't present.)

---

## 5. Files changed

| File | Change |
|------|--------|
| `common/params_keys.h` | **+** `CESMode` INT param (additive); annotated the legacy bool as superseded |
| `selfdrive/controls/lib/ces_xnor/ces_xnor_constants.py` | `CES_MODE_*` consts + `ces_enabled` / `ces_is_gentle` / `read_ces_mode`; `GENTLE_FINGERPRINTS` marked unused |
| `selfdrive/controls/lib/ces_xnor/ces_xnor.py` | `CESController` selects gentle by `CESMode` (not fingerprint); `_set_mode` rebuilds SM on flip; reads `CESMode` |
| `selfdrive/controls/lib/vtsc_xnor/vtsc_controller.py` | `VTSCController` selects tune by `CESMode` (not fingerprint); reads `CESMode` |
| `selfdrive/controls/lib/vtsc_xnor/vtsc_constants.py` | `GENTLE_FINGERPRINTS` marked unused |
| `selfdrive/ui/layouts/settings/toggles.py` | 3-way `CESMode` selector below Experimental Mode; `ces_group_enabled` helper + `CES_GROUP` block (grey-out fix); `_set_ces_mode` callback |
| `selfdrive/ui/onroad/ces_status.py` | overlay gates on `CESMode>0` (both Light & Standard) via `read_ces_mode` |
| `selfdrive/controls/lib/ces_xnor/tests/test_ces_mode.py` | **new** — CESMode→(enabled, profile) for both controllers + grey-out symmetry |

---

## 6. Deploy steps (file overlay to /data/openpilot)

`params_keys.h` changed, so the params Cython extension must be rebuilt on-device.

```bash
# (overlay the changed files into /data/openpilot first — same surgical-patch flow as other features)

# rebuild the params extension (REQUIRED — new CESMode key) + clear stale pyc
PATH=/usr/local/venv/bin:$PATH scons -u -j$(nproc) common/params_pyx.so
find /data/openpilot -name '*.pyc' -delete

# persistence guards (so the reboot overlay-swap doesn't wipe the overlay / reflash panda)
sudo rm -rf /data/safe_staging/finalized
touch -d "2020-01-01" /data/openpilot/.overlay_init
touch /data/openpilot/prebuilt

# restart
sudo systemctl restart comma   # or reboot
```

Existing installs: any device that had the old bool CES "on" reads back as **Standard** via the
`read_ces_mode` back-compat path until the user picks a mode in the new selector.

---

## 7. On-device test steps

1. **Selector visible + placed:** open Settings → Toggles. The CES selector
   (`Off / Light / Standard`) appears **directly under "Experimental Mode"** on every car.
2. **Off (default):** `CESMode=0`. No debug overlay, no VTSC capping, behavior == stock. Confirm the
   car drives exactly as without the feature.
3. **Light:** tap **Light** (`CESMode=1`). With the top-right button in **CES-auto**, drive and
   confirm: the **debug overlay shows**; VTSC trims curve speed **smoothly** (gentle profile, slow
   recovery, no sawtooth on a series of curves); CES does **not** flip to Experimental for curves
   (curves handed to VTSC) but still trips for stops / slow leads.
4. **Standard:** tap **Standard** (`CESMode=2`). With the button in CES-auto, confirm: the **debug
   overlay shows** (verify it triggers for Standard too, not only Light); VTSC uses the default tune;
   CES trips Experimental for curves as before.
5. **Button still cycles in every mode:** in Light and in Standard, tap the top-right button →
   it cycles CES-auto → forced-Chill → forced-Experimental (overlay hides in the two forced modes).
6. **Grey-out symmetry:** on a car / config where openpilot longitudinal control can be toggled,
   turn long control **OFF** → the CES selector greys out. Turn it back **ON** → confirm the CES
   selector (and the whole CES group) re-enables. Nothing stays greyed.
