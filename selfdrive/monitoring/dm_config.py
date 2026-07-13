"""dm-variable: JSON-configurable driver-monitoring timeout tiers.

Philosophy: TIGHT IN SOURCE, PERSONAL VALUES EXTERNAL-ONLY. The repo carries only the strict,
defensible tier defaults below. Any loosening beyond them lives exclusively in /data/pnw/dm.json
(device-local, persistent — /data survives the auto-update git-clean — and never committed),
written via the tools/dm CLI. Without the file the device runs the strict defaults, period.

This module resolves the JSON to either None (= "default" tier: strict in-code behavior) or an
explicit opt-in tier ("highway" / "relaxed") with pose/phone timeouts in seconds, plus the
per-regime timeout values the DmMode param selector consumes.

SAFETY POSTURE
- This module must NEVER raise out of load_dm_tier(): it runs inside dmonitoringd (a safety
  process). Any missing / unreadable / malformed / mistyped input degrades to the hardcoded
  defaults with a warning — never an exception.
- The hardcoded defaults below live only in this Python source. They do not depend on the JSON
  file existing. With no file present the resolved tier is None and DM behavior is unchanged.
- "relaxed" is opt-in only: it applies solely when the JSON carries relaxed.enabled == true
  (JSON boolean). Anything else (absent, false, "true", 1) means the relaxed tier is UNAVAILABLE
  and the effective tier falls back to default.
- Timeouts are clamped to [TIMEOUT_MIN_S, TIMEOUT_MAX_S]: no JSON input can push DM beyond the
  absolute ceiling, and nothing below 10 s.

Config is read once at dmonitoringd process start (DriverMonitoring.__init__). Changing dm.json
requires a dmonitoringd restart (ignition cycle or pkill) to take effect — documented in
docs/pnw/DM-VARIABLE.md.

stdlib only — also loaded standalone (importlib by path) by the tools/dm CLI.
"""
import json
import math
import os
import stat

DM_CONFIG_PATH = "/data/pnw/dm.json"

# Refuse to parse anything bigger than this (a sane config is <1 KiB). Guards against OOM from a
# huge file accidentally landing at the config path (Gemini review finding, 2026-07-11).
MAX_CONFIG_BYTES = 65536

# Hardcoded tier defaults (seconds). First value = pose/attention timeout, second = phone timeout.
# These are the fallbacks whenever the JSON is missing or a field is absent/invalid.
HIGHWAY_DEFAULT_POSE_S = 30.0
HIGHWAY_DEFAULT_PHONE_S = 60.0
RELAXED_DEFAULT_POSE_S = 60.0
RELAXED_DEFAULT_PHONE_S = 120.0

# Sane bounds: anything outside is clamped (and warned about). The ceiling is the absolute cap —
# no JSON input may configure a timeout longer than this (4 h). It is deliberately wide: the
# driver's personal values are meant to live ONLY in the device-local JSON, never in this source.
TIMEOUT_MIN_S = 10.0
TIMEOUT_MAX_S = 14400.0

VALID_MODES = ("default", "highway", "relaxed")

TIER_DEFAULTS = {
  "highway": (HIGHWAY_DEFAULT_POSE_S, HIGHWAY_DEFAULT_PHONE_S),
  "relaxed": (RELAXED_DEFAULT_POSE_S, RELAXED_DEFAULT_PHONE_S),
}


def _noop_warn(msg: str) -> None:
  pass


def _sanitize_timeout(value, fallback: float, name: str, warn) -> float:
  """Return a finite float clamped to [TIMEOUT_MIN_S, TIMEOUT_MAX_S]; fallback on any bad type."""
  if value is None:  # field simply absent -> quiet fallback to the tier default
    return fallback
  # bool is an int subclass in Python: reject explicitly (true/false is not a timeout)
  if isinstance(value, bool) or not isinstance(value, (int, float)):
    warn(f"dm_config: {name}={value!r} is not a number, using default {fallback:g}s")
    return fallback
  v = float(value)
  if not math.isfinite(v):
    warn(f"dm_config: {name}={value!r} is not finite, using default {fallback:g}s")
    return fallback
  if v < TIMEOUT_MIN_S:
    warn(f"dm_config: {name}={v:g}s below minimum, clamped to {TIMEOUT_MIN_S:g}s")
    return TIMEOUT_MIN_S
  if v > TIMEOUT_MAX_S:
    warn(f"dm_config: {name}={v:g}s above maximum, clamped to {TIMEOUT_MAX_S:g}s")
    return TIMEOUT_MAX_S
  return v


def read_raw_config(path: str = DM_CONFIG_PATH, warn=None):
  """Best-effort read of the JSON file. Returns a dict ({} if missing/unreadable/not-a-dict)."""
  warn = warn or _noop_warn
  try:
    try:
      st = os.stat(path)
    except FileNotFoundError:
      return {}  # missing file is the normal state
    # Only parse a plain regular file: a FIFO would block open() forever and a character device
    # (or a huge file) would hang/OOM json.load() — dmonitoringd is a safety process.
    if not stat.S_ISREG(st.st_mode):
      warn(f"dm_config: {path} is not a regular file, ignoring it")
      return {}
    if st.st_size > MAX_CONFIG_BYTES:
      warn(f"dm_config: {path} is {st.st_size} bytes (max {MAX_CONFIG_BYTES}), ignoring it")
      return {}
    with open(path, encoding="utf-8") as f:
      data = json.load(f)
  except Exception as e:
    warn(f"dm_config: cannot read {path} ({e.__class__.__name__}: {e}), using hardcoded defaults")
    return {}
  if not isinstance(data, dict):
    warn(f"dm_config: {path} top level is {type(data).__name__}, expected object; using hardcoded defaults")
    return {}
  return data


def resolve_tier_timeouts(data: dict, tier: str, warn=None) -> tuple[float, float]:
  """(pose_s, phone_s) for a tier from an already-read config dict, sanitized and clamped."""
  warn = warn or _noop_warn
  d_pose, d_phone = TIER_DEFAULTS[tier]
  section = data.get(tier)
  if section is None:
    section = {}
  elif not isinstance(section, dict):
    warn(f"dm_config: '{tier}' section is {type(section).__name__}, expected object; using defaults")
    section = {}
  pose_s = _sanitize_timeout(section.get("pose_s"), d_pose, f"{tier}.pose_s", warn)
  phone_s = _sanitize_timeout(section.get("phone_s"), d_phone, f"{tier}.phone_s", warn)
  return pose_s, phone_s


def relaxed_enabled(data: dict) -> bool:
  """Strict opt-in: only a JSON boolean true counts."""
  section = data.get("relaxed")
  return isinstance(section, dict) and section.get("enabled") is True


def _resolve_mode(data: dict, warn) -> str:
  """Validated JSON 'mode' -> one of VALID_MODES ('default' on anything invalid)."""
  mode = data.get("mode", "default")
  if not isinstance(mode, str) or mode not in VALID_MODES:
    warn(f"dm_config: mode={mode!r} is not one of {VALID_MODES}, using default tier")
    return "default"
  return mode


def load_dm_timeouts(path: str = DM_CONFIG_PATH, warn=None) -> dict:
  """Resolve everything DM needs from the JSON config in one read. Never raises.

  Returns {
    "tier":    None | (tier_name, pose_s, phone_s),  # JSON 'mode' selection (relaxed opt-in enforced)
    "highway": (pose_s, phone_s),                    # values for any highway regime (DmMode=1 or JSON)
    "relaxed": (pose_s, phone_s),                    # values for the DmMode=2 regime; stays at the
                                                     # STRICT defaults unless relaxed.enabled is true
  }
  With no/invalid file everything is the strict hardcoded defaults and tier is None.
  """
  warn = warn or _noop_warn
  strict = {
    "tier": None,
    "highway": TIER_DEFAULTS["highway"],
    "relaxed": TIER_DEFAULTS["relaxed"],
  }
  try:
    data = read_raw_config(path, warn)
    if not data:
      return strict
    out = dict(strict)
    out["highway"] = resolve_tier_timeouts(data, "highway", warn)
    # relaxed values beyond the strict defaults require the explicit enabled flag, no matter how
    # the relaxed regime gets selected (JSON mode or the DmMode Settings param)
    if relaxed_enabled(data):
      out["relaxed"] = resolve_tier_timeouts(data, "relaxed", warn)
    mode = _resolve_mode(data, warn)
    if mode == "highway":
      out["tier"] = ("highway", *out["highway"])
    elif mode == "relaxed":
      if relaxed_enabled(data):
        out["tier"] = ("relaxed", *out["relaxed"])
      else:
        warn("dm_config: mode is 'relaxed' but relaxed.enabled is not true — relaxed tier unavailable, using default tier")
    return out
  except Exception as e:  # belt and braces: a DM process crash is a safety event
    try:
      warn(f"dm_config: unexpected error resolving config ({e.__class__.__name__}: {e}), using strict defaults")
    except Exception:
      pass
    return strict


def load_dm_tier(path: str = DM_CONFIG_PATH, warn=None):
  """JSON 'mode' tier only: None for default, else (tier_name, pose_s, phone_s). Never raises."""
  return load_dm_timeouts(path, warn)["tier"]
