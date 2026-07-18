# DM-CURRENT — as-deployed driver monitoring (single source of truth)

**STATUS (2026-07-12): dm-variable SHIPPED** — merged to **`3devpnw`** (`d0cc222be4` + UI follow-ups
`264124c6cf`/`f596d27af7`/`e396b8bf9b`), riding the device's auto-update channel. **The long
personal timeout magnitudes are GONE FROM SOURCE** (a source-purity test pins them out forever);
strict tier defaults live in `selfdrive/monitoring/dm_config.py`, personal values live ONLY in the
device-local **`/data/pnw/dm.json`** via the **`dm` CLI**. Canonical design doc:
`pnw-pilot:DM-VARIABLE.md` (`git -C pnw/pnw-pilot show 3devpnw:DM-VARIABLE.md`).

This doc resolves the ambiguity between `DMON.md` / `DMON2XNOR.md` (superseded values, a gate that
no longer exists) and `DMROAD2PNW.md` (the `DmMode` selector — **its 900/1800 s + 3 h/1 h magnitudes
are superseded in part** by dm-variable). When in doubt, trust this file and the code.

## The tiers (strict in source; personal values external-only)

| Tier | Pose / phone timeout | Gate |
|---|---|---|
| **default** | stock strict (`_DISTRACTED_TIME` = 11 s) | none — the fallback, independent of any JSON |
| **highway** (`DmMode=1` or JSON `mode:"highway"`) | **30 s / 60 s** strict defaults | **road-gated to freeways** (mapd bridge: `RoadContext=="freeway"` OR `MapOneWay && MapLanes>=2`, 90 s hold); off a qualifying road → stock strict, never a looser regime |
| **relaxed** (`DmMode=2` or JSON `mode:"relaxed"`) | **60 s / 120 s** strict defaults | **opt-in only**: `relaxed.enabled == true` (literal JSON boolean) in `dm.json`; without it, relaxed runs the strict 60/120 defaults and JSON `mode:"relaxed"` falls back to default |

- JSON-supplied timeouts clamped to **[10 s, 14400 s]** (4 h ceiling — long personal values are
  configurable on-device but never appear in the repo). Pre/prompt leads capped at t/2 and t/4.
- JSON tier, when active, takes precedence over the `DmMode` param. Read once at dmonitoringd start
  (change → ignition cycle / dmonitoringd restart); the `DmMode` param + road gate poll ~1 Hz live.
- Loader hardened per Gemini review: single `os.stat` gate — only a regular file ≤ 64 KiB is ever
  opened (a FIFO/char-device or huge file can't hang/OOM dmonitoringd); never raises.

## The `dm` CLI + UI policy (driver directives 2026-07-11)

- CLI at `/data/openpilot/tools/dm` with a **`/usr/local/bin/dm` symlink** (symlink-resolving,
  `e396b8bf9b`): `dm show` / `dm mode default|highway|relaxed` / `dm highway <pose> <phone>` /
  `dm relaxed <pose> <phone>` / `dm relaxed --enable|--disable`. Atomic writes, creates `/data/pnw`.
- **Relaxed is INVISIBLE in the UI unless unlocked** via the CLI (`relaxed.enabled=true` in
  `dm.json`): without it the Settings selector offers Default/Highway only and clamps a stale
  `DmMode=2` back to 0 (`264124c6cf`). Unlock → visible after the next UI restart.
- **Help-text policy:** the DM help describes Default + Highway only, numbers printed live from
  `dm_config` source constants (can never drift); **no mention of the third tier or any tooling**
  (`f596d27af7`).
- `/data/pnw/dm.json` is persistent (outside the git tree — survives the auto-updater's git clean)
  and never committed.

## Architecture: dual-counter relaxed DM (unchanged — the DMON2XNOR design)

Two independent awareness counters run in active mode; the more-severe one drives the UI:
`awareness_pose` (pose+blink) and `awareness_phone`, separate decay steps, combined via
`min()`, snap-back to 1.0 after 2 s clean (`_RECOVERY_DEBOUNCE_FRAMES`). Passive mode (face lost /
model uncertain) falls through to stock single-counter wheel-touch logic. `_PHONE_THRESH=0.6`,
`_MAX_TERMINAL_ALERTS=10` remain as before.

### GLARE Layer-C knobs (unchanged, apply in every mode)

`_POSESTD_THRESHOLD=0.45` · `_HI_STD_FALLBACK_TIME=30 s` · `_DCAM_UNCERTAIN_RESET_COUNT=2 s` ·
`_FACE_THRESHOLD=0.7` (untouched) · `_FACE_LOST_GRACE_TIME=30 s`. See `GLARE.md` §10–11. Upstream
Layers A (driver-cam BPS ISP) + B (sleep-prob model): Layer B effectively arrived with the lebowski
bundle's 0.11.1 DM model (`LEBOWSKI2PNW.md`); Layer A via the upstream2pnw Tier-1 picks.

## History (how we got here — don't act on these)

1. **Ungated 3 h / 1 h** (dmon2pnw era, ≤2026-07-07) — no toggle at all. Superseded.
2. **`DmMode` selector, road-gated** (`DMROAD2PNW.md`, deployed 2026-07-08) — 0=stock /
   1=Highway 900/1800 s / 2=Relaxed 3 h/1 h. The selector + road gate survive; **the long
   magnitudes were deleted from source by dm-variable** (2026-07-12).
3. **dm-variable** (this doc's current state).
4. Removed params (do not grep for them): `SensitiveDriverMonitoring`, `BPRelaxedDriverMonitoring`,
   `AllowSoftwareUpdates`. `AlwaysOnDM` (stock) is unrelated (DM-while-disengaged only).

## See also

`pnw-pilot:DM-VARIABLE.md` (canonical) · `DMROAD2PNW.md` (selector + road gate; magnitudes
superseded) · `DMON2XNOR.md` (dual-counter architecture) · `GLARE.md` · `DMON.md` (history).
