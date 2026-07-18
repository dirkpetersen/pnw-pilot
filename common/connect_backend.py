"""connectsel2pnw: connect backend selection (PNW self-hosted / Konik / Custom / Offline).

Ported from BluePilot 7.0's BPConnectBackend (bluepilot/backend_switch.py), adapted for this
fork: comma's backend is NOT an option here, the default is the PNW self-hosted connect, and a
driver-entered Custom URL is added. ConnectBackend (INT) selects where the device sends routes,
uploads, athena and pairing:

  0 = PNW (default)  — this fork's self-hosted connect: API gateway
      https://jh69za4byd.execute-api.us-west-2.amazonaws.com -> Lambda presigned-PUT ->
      s3://comma-connect, web UI at comma-connect.aws.internetchen.de. An unset/fresh param
      MUST behave exactly like a build without this feature (see the 412 silent-data-loss
      note in common/api.py) — index 0 therefore changes nothing, including leaving
      ATHENA_HOST at its stock default.
  1 = Konik Stable   — api.konik.ai / athena.konik.ai, pairing at stable.konik.ai.
  2 = Custom URL     — driver-entered https:// base URL (ConnectCustomUrl); athena and
      pairing are derived from the same host. Falls back to PNW when the URL is missing or
      invalid, so a half-configured device never silently targets a wrong backend.
  3 = Offline        — RFC 2606 .invalid hosts, so uploads/athena can never egress.

launch_env.sh exports API_HOST / ATHENA_HOST from these params at boot; openpilot already
reads those env vars everywhere it talks to a backend (common/api.py, system/athena/athenad.py,
registration, uploader), so all host wiring lives there and a backend switch takes effect on
reboot. No process subscribes to anything for this — params only, read at process start.

Dongle ID handling (register() calls reconcile_backend() on every manager start): each real
backend issues its own dongle ID at registration, so the last-seen ID is cached PER BACKEND
(Custom: per URL, keyed by sha256(url)[:16] inside a JSON-dict param). Switching backends swaps
DongleId in from the target's cache — or clears it to force a fresh registration against the new
backend — which makes every switch reversible without identity churn. Offline keeps whatever
DongleId is present and never registers.
"""

import hashlib
import json

from openpilot.common.swaglog import cloudlog

UNREGISTERED_DONGLE_ID = "UnregisteredDevice"

BACKEND_PNW = "pnw"
BACKEND_KONIK = "konik"
BACKEND_CUSTOM = "custom"
BACKEND_OFFLINE = "offline"

# Index order matches the ConnectBackend param and the Network -> Advanced selector buttons.
BACKENDS = (BACKEND_PNW, BACKEND_KONIK, BACKEND_CUSTOM, BACKEND_OFFLINE)

PAIRING_HOST = {
  BACKEND_PNW: "comma-connect.aws.internetchen.de",  # connect-url2pnw: the self-hosted connect web UI
  BACKEND_KONIK: "stable.konik.ai",
  BACKEND_OFFLINE: "pairing.invalid",  # RFC 2606 .invalid never resolves
}

# Fixed backends cache their dongle ID in one param each; Custom URLs share a JSON-dict param
# (params are a fixed key set, so per-URL entries can't get their own key).
CACHE_PARAM = {
  BACKEND_PNW: "DongleIdCachePnw",
  BACKEND_KONIK: "DongleIdCacheKonik",
}
CUSTOM_CACHE_PARAM = "DongleIdCacheCustom"

PARAM_KEY = "ConnectBackend"
CUSTOM_URL_PARAM = "ConnectCustomUrl"
ACTIVE_PARAM = "ConnectActiveBackend"


def valid_custom_url(url) -> bool:
  # Minimal validation per driver requirement: https:// scheme with a non-empty host.
  return isinstance(url, str) and url.startswith("https://") and len(url.removeprefix("https://").split("/")[0]) > 0


def custom_url(params) -> str:
  raw = params.get(CUSTOM_URL_PARAM)
  return raw.strip().rstrip("/") if isinstance(raw, str) else ""


def custom_host(url: str) -> str:
  return url.removeprefix("https://").split("/")[0]


def _custom_key(url: str) -> str:
  # Cache keying for Custom backends: a hash of the base URL, so every distinct URL gets its own
  # dongle-ID slot and switching between two Custom backends is just as reversible as PNW <-> Konik.
  return hashlib.sha256(url.encode()).hexdigest()[:16]


def get_connect_backend(params) -> str:
  """Return the EFFECTIVE backend name: pnw, konik, custom, or offline."""
  try:
    raw = params.get(PARAM_KEY)
    idx = int(raw) if raw not in (None, "") else 0
  except (TypeError, ValueError):
    idx = 0
  backend = BACKENDS[idx] if 0 <= idx < len(BACKENDS) else BACKEND_PNW
  if backend == BACKEND_CUSTOM and not valid_custom_url(custom_url(params)):
    # Half-configured Custom (no/invalid URL) behaves as the PNW default rather than guessing.
    return BACKEND_PNW
  return backend


def pairing_host(params) -> str:
  """Host the pairing QR / instructions should point at for the selected backend."""
  backend = get_connect_backend(params)
  if backend == BACKEND_CUSTOM:
    return custom_host(custom_url(params))
  return PAIRING_HOST.get(backend, PAIRING_HOST[BACKEND_PNW])


def _target_token(backend: str, url: str) -> str:
  # The active-backend token distinguishes DIFFERENT Custom URLs ("custom:<hash>"), so editing the
  # URL while staying on Custom still counts as a backend switch and re-reconciles the dongle ID.
  return f"{BACKEND_CUSTOM}:{_custom_key(url)}" if backend == BACKEND_CUSTOM else backend


def _cache_read(params, token: str) -> str | None:
  if token in CACHE_PARAM:
    return params.get(CACHE_PARAM[token])
  if token.startswith(BACKEND_CUSTOM + ":"):
    try:
      cache = json.loads(params.get(CUSTOM_CACHE_PARAM) or "{}")
      if isinstance(cache, dict):
        return cache.get(token.split(":", 1)[1])
    except (TypeError, ValueError):
      pass
  return None  # offline has no cache slot


def _cache_write(params, token: str, dongle_id: str) -> None:
  if token in CACHE_PARAM:
    params.put(CACHE_PARAM[token], dongle_id)
  elif token.startswith(BACKEND_CUSTOM + ":"):
    key = token.split(":", 1)[1]
    try:
      cache = json.loads(params.get(CUSTOM_CACHE_PARAM) or "{}")
      if not isinstance(cache, dict):
        cache = {}
    except (TypeError, ValueError):
      cache = {}
    if cache.get(key) != dongle_id:
      cache[key] = dongle_id
      params.put(CUSTOM_CACHE_PARAM, json.dumps(cache))
  # offline: nothing to stash


def reconcile_backend(params) -> str:
  """Align DongleId with the backend selected by ConnectBackend.

  Called at the top of register(). Returns the effective backend name. Callers skip the
  /persist comma dongle ID restore when the backend is not PNW (restoring the factory comma ID
  would short-circuit registration against Konik/Custom). Offline never attempts network
  registration.
  """
  try:
    target = get_connect_backend(params)
    target_token = _target_token(target, custom_url(params))
    active_token = params.get(ACTIVE_PARAM) or BACKEND_PNW
    dongle_id = params.get("DongleId")
    registered = dongle_id is not None and dongle_id != UNREGISTERED_DONGLE_ID

    if target_token == active_token:
      # Keep the cache fresh for real backends after a successful registration.
      if registered and _cache_read(params, target_token) != dongle_id:
        _cache_write(params, target_token, dongle_id)
      return target

    # Backend changed: stash the outgoing backend's ID in its cache slot (no-op for offline).
    if registered:
      _cache_write(params, active_token, dongle_id)

    if target == BACKEND_OFFLINE:
      # Stay offline with whatever DongleId we already have; no registration.
      params.put(ACTIVE_PARAM, target_token)
      cloudlog.event("connectsel_backend_switch", backend=target, dongle_id=dongle_id, restored=False)
      return target

    cached = _cache_read(params, target_token)
    if cached:
      # NOTE: this fork's Params.put blocks until written and takes NO block= kwarg (xnor-era API).
      params.put("DongleId", cached)
      cloudlog.event("connectsel_backend_switch", backend=target, dongle_id=cached, restored=True)
    else:
      params.remove("DongleId")
      cloudlog.event("connectsel_backend_switch", backend=target, restored=False)

    params.put(ACTIVE_PARAM, target_token)
    return target
  except Exception:
    # Never block registration: fall back to the PNW default behavior on any failure.
    cloudlog.exception("connectsel_backend_switch failed, using PNW backend behavior")
    return BACKEND_PNW
