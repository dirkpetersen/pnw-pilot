# DM-VARIABLE — JSON-configurable driver-monitoring timeout tiers

**STATUS: MERGED to `3devpnw` 2026-07-12** (`d0cc222be4`) and riding the device's auto-update channel.
Post-merge additions: the `dm` CLI resolves symlinks so it works from `/usr/local/bin/dm`
(`e396b8bf9b`); the UI **hides the Relaxed option entirely** unless `relaxed.enabled=true` in
`/data/pnw/dm.json` (driver directive: relaxed only ever via the `dm` script, never the UI —
`264124c6cf`, a locked UI also clamps a stale `DmMode=2` back to 0); the DM help text describes
**Default + Highway only**, numbers printed live from `dm_config` source constants, no mention of
the third tier or any tooling (`f596d27af7`). As-deployed summary: `docs/DM-CURRENT.md`.

**Philosophy (driver directive 2026-07-11): tight in source, personal values external-only.** The
repo carries only strict, defensible timeout defaults. Any loosening lives exclusively in the
device-local, persistent **`/data/pnw/dm.json`** (never committed), written via the `dm` CLI. The
formerly in-source long timeout magnitudes for the `DmMode` Highway/Relaxed regimes were **removed
from source** in the same pass — without `dm.json` the device runs the strict defaults everywhere,
and a source-purity unit test pins that the old long constants can never return.

## The three tiers (first value = pose/attention timeout, second = phone timeout)

| Tier | Pose | Phone | Where the values live |
|---|---|---|---|
| **default** | stock strict (`_DISTRACTED_TIME` = 11 s) | same | hardcoded in `helpers.py` — the fallback, fully independent of the JSON |
| **highway** | **30 s** strict default | **60 s** strict default | `selfdrive/monitoring/dm_config.py`; personal values only via `dm.json` |
| **relaxed** | **60 s** strict default | **120 s** strict default | same — values beyond the defaults require `relaxed.enabled == true` |

JSON-supplied timeouts are clamped to **[10 s, 14400 s]** (`TIMEOUT_MIN_S`/`TIMEOUT_MAX_S`). The
ceiling is deliberately wide (4 h): the point is that long personal values are *configurable* on the
device but *never appear in the repo*. Green/orange lead times are capped at `t/2` (pre) and `t/4`
(prompt) so short timeouts keep a sane progression.

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
- `highway.pose_s` / `highway.phone_s` (numbers, seconds): absent → strict defaults; non-numeric or
  non-finite → that field's default with a warning; out of range → clamped with a warning.
- `relaxed.enabled` (JSON boolean): **strict opt-in** — only literal `true` counts (`"true"`, `1`,
  absent all mean disabled). Gates relaxed values in **both** selection paths (JSON `mode` and the
  Settings `DmMode=2` param): not enabled ⇒ the relaxed regime runs the strict 60/120 defaults and
  JSON `mode: "relaxed"` falls back to the default tier.
- `relaxed.pose_s` / `relaxed.phone_s`: as highway.

## Selection, precedence & the road gate

One read at dmonitoringd start (`dm_config.load_dm_timeouts()`) resolves:

- **`tier`** — the JSON `mode` selection (`None` = default). When active it **takes precedence over
  the Settings `DmMode` param**:
  - `highway` tier: **road-gated to freeways** using the *exact same gate* as `DmMode=1` —
    `_refresh_dm_mode()` reads the mapd bridge mem-params (`RoadContext == "freeway"` OR
    `MapOneWay && MapLanes >= 2`, with the 90 s hold over map dropouts). Off a qualifying road the
    tier is inert and DM falls to **stock strict** (never through to a looser `DmMode` regime).
  - `relaxed` tier: everywhere (requires `relaxed.enabled`).
- **`highway` / `relaxed` value pairs** — consumed by the `DmMode` param regimes too: `DmMode=1`
  (Highway, road-gated) uses the highway values, `DmMode=2` (Relaxed) uses the relaxed values.
  Without `dm.json` those are the strict 30/60 and 60/120 defaults — the old in-source long
  magnitudes are gone.

**Read at process start only:** changing `dm.json` takes effect at the next ignition cycle /
dmonitoringd restart. (The ~1 Hz `_refresh_dm_mode` poll services the `DmMode` param and road gate
live, as before.)

## The `dm` CLI — `tools/dm` (stdlib only, chmod +x)

```
dm show                            # raw file + resolved effective config dmonitoringd will use
dm mode default|highway|relaxed    # select the active tier
dm highway <pose_s> <phone_s>      # set highway timeouts (pose first, phone second)
dm relaxed <pose_s> <phone_s>      # set relaxed timeouts
dm relaxed --enable | --disable    # the relaxed opt-in flag
dm ... --file PATH                 # operate on another file (testing)
```

Numeric args are validated (reject non-numbers/NaN/inf) and clamped to [10, 14400] with a message.
Writes are atomic (`tempfile` + `os.replace`), create `/data/pnw` if missing, and preserve
unrelated keys. The CLI help explains the source-strict/device-personal split and prints hints when
a write won't take effect (mode not pointing at the tier, relaxed not enabled).

## Safety posture

- **Never crashes or hangs dmonitoringd:** `load_dm_timeouts` never raises (missing/unreadable/
  malformed/mistyped input → strict defaults + `cloudlog.warning`), and the `helpers.py` call site
  wraps it in `try/except` anyway — a DM process crash is a safety event. Per the Gemini review
  (gemini-pro-latest, 2026-07-11): only a plain regular file ≤ 64 KiB is ever opened — a FIFO/
  char-device at the path (would block `open()`/`json.load` forever) or an oversized file (OOM
  risk) is rejected on the `stat` with a warning.
- **No file = strict defaults everywhere.** Nothing in the source can loosen DM beyond 30/60
  (highway, freeway-only) / 60/120 (relaxed, opt-in); anything longer exists only in the driver's
  device-local JSON, capped at 4 h.
- **Relaxed is opt-in only** (`relaxed.enabled: true`), in both selection paths.
- **No panda changes, no existing DM check removed or weakened.** Passive wheel-touch mode, the
  GLARE Layer-C knobs, terminal-alert lockout, the road gate, and the `DmMode` selector UI are all
  unchanged — only the timeout *magnitudes* moved out of source.

## Files

- `selfdrive/monitoring/dm_config.py` — loader/validator + the strict tier defaults (stdlib only)
- `selfdrive/monitoring/helpers.py` — init load, `_apply_dm_timeouts` regime resolution, shared road gate
- `selfdrive/monitoring/test_dm_config.py` — 51 offline tests incl. the source-purity grep
- `tools/dm` — the CLI
- `DM-VARIABLE.md` — this file (see also `DMROAD2PNW.md` for the `DmMode` selector this builds on)
