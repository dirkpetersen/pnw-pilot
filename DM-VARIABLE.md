# DM-VARIABLE — JSON-configurable driver-monitoring timeout tiers

**STATUS:** built on branch **`dm-variable`** (off `3devpnw`), NOT deployed. Branch development only.

Adds an opt-in, file-configurable layer on top of the existing DM machinery: three timeout **tiers**
for the dual-counter (pose + phone) active-monitoring timeouts, selected and tuned via a persistent
JSON file, with a small `dm` CLI to edit it. **With no config file present, behavior is
byte-identical to the current tree** — the default tier is hardcoded in Python and never depends on
the JSON.

## The three tiers (first value = pose/attention timeout, second = phone timeout)

| Tier | Pose timeout | Phone timeout | Source of the values |
|---|---|---|---|
| **default** | whatever the in-code `DmMode` machinery produces today (stock strict 11 s at `DmMode=0`, road-gated 900/1800 s at Highway, 3 h/1 h at Relaxed) | same | **hardcoded in Python** (`helpers.py` / `DRIVER_MONITOR_SETTINGS`) — the fallback, independent of the JSON |
| **highway** | **30 s** default | **60 s** default | `selfdrive/monitoring/dm_config.py` hardcoded defaults, overridable via JSON |
| **relaxed** | **60 s** default | **120 s** default | same — **and only available when `relaxed.enabled == true`** |

All JSON-supplied timeouts are clamped to **[10 s, 600 s]** (`TIMEOUT_MIN_S`/`TIMEOUT_MAX_S`); no
JSON input can configure a timeout beyond 10 minutes. Green/orange lead times for a JSON tier are
capped at `pose_t/2` (pre) and `pose_t/4` (prompt) so short timeouts keep a sane progression.

## Config file: `/data/pnw/dm.json`

Persistent on purpose: `/data/pnw` is outside the git tree, so it survives the auto-updater's
`git clean` (the same reason the police-proxy key lives under `/data/pnw`). Schema:

```json
{
  "mode": "default",
  "highway": {"pose_s": 30, "phone_s": 60},
  "relaxed": {"enabled": false, "pose_s": 60, "phone_s": 120}
}
```

- `mode` (string): `"default"` | `"highway"` | `"relaxed"`. Absent/invalid → `"default"`.
- `highway.pose_s` / `highway.phone_s` (numbers, seconds): absent → 30/60; non-numeric or
  non-finite → that field's default with a warning; out of range → clamped with a warning.
- `relaxed.enabled` (JSON boolean): **strict opt-in** — only literal `true` counts (`"true"`, `1`,
  absent all mean disabled). With `mode: "relaxed"` but not enabled, the effective tier is
  **default**.
- `relaxed.pose_s` / `relaxed.phone_s`: as highway, defaults 60/120.

## Selection & precedence

Resolution lives in `selfdrive/monitoring/dm_config.py::load_dm_tier()` → `None` (default tier) or
`(tier, pose_s, phone_s)`:

- **default tier (`None`)**: `helpers.py` changes nothing — the existing `DmMode` param selector
  (Off/Highway/Relaxed, road-gated) runs exactly as before.
- **highway / relaxed tier**: the tier's timeouts replace the `DmMode`-derived active-mode
  timeouts **everywhere** (fixed, not road-gated) — the JSON tier takes precedence over the
  `DmMode` param. Writing the JSON is the explicit driver opt-in.

**Read at process start only:** `DriverMonitoring.__init__` (dmonitoringd startup) loads the file
once. Changing `dm.json` takes effect at the next ignition cycle / dmonitoringd restart. (The ~1 Hz
`_refresh_dm_mode` poll still services the `DmMode` param for the default tier.)

## The `dm` CLI — `tools/dm` (stdlib only, chmod +x)

```
dm show                            # raw file + resolved effective config
dm mode default|highway|relaxed    # select the active tier
dm highway <pose_s> <phone_s>      # set highway timeouts (pose first, phone second)
dm relaxed <pose_s> <phone_s>      # set relaxed timeouts
dm relaxed --enable | --disable    # the relaxed opt-in flag
dm ... --file PATH                 # operate on another file (testing)
```

Numeric args are validated (reject non-numbers/NaN/inf) and clamped to [10, 600] with a message.
Writes are atomic (`tempfile` + `os.replace`), create `/data/pnw` if missing, and preserve
unrelated keys. The CLI prints hints when a write won't take effect (mode not pointing at the tier,
relaxed not enabled).

## Safety posture

- **Never crashes dmonitoringd:** `load_dm_tier` never raises (missing/unreadable/malformed/
  mistyped input → hardcoded defaults + `cloudlog.warning`), and the `helpers.py` call site wraps
  it in `try/except` anyway — a DM process crash is a safety event.
- **No file = current behavior, byte-identical.** The hardcoded default tier lives only in the
  Python source.
- **Relaxed is opt-in only** (`relaxed.enabled: true` required), and even then capped at 600 s —
  far stricter than the existing `DmMode=2` (3 h/1 h) path, which is untouched.
- **No panda changes, no existing DM check removed or weakened.** Passive wheel-touch mode, the
  GLARE Layer-C knobs, terminal-alert lockout, and the `DmMode` machinery are all unchanged.

## Files

- `selfdrive/monitoring/dm_config.py` — loader/validator (stdlib only)
- `selfdrive/monitoring/helpers.py` — 3 additive edits (import, init load, `_apply_dm_timeouts` branch)
- `selfdrive/monitoring/test_dm_config.py` — 35 offline tests (loader, helpers wiring, CLI round-trip)
- `tools/dm` — the CLI
- `DM-VARIABLE.md` — this file

## Open questions for the driver

1. Should the JSON **highway** tier reuse the `DmMode=1` road gate (relaxed only on
   freeway/divided, strict elsewhere)? Currently it applies everywhere for simplicity.
2. Precedence: JSON tier currently **overrides** the Settings `DmMode` selector when active — OK,
   or should `DmMode` win?
3. Live re-read (e.g. on mtime change) instead of restart-to-apply?
