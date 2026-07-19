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
CUSTOM_CACHE_MAX = 8  # cap on distinct Custom-URL dongle-ID cache entries -- without this a typo'd
# or one-off URL adds a permanent sha256(url)[:16] slot with no eviction and DongleIdCacheCustom
# grows forever (Gemini finding #3). Not a true recency LRU (entries are only re-ordered when their
# DongleId is actually (re)written, not on every read) -- see _cache_write's custom branch. A
# stable, actively-used Custom backend whose DongleId never changes won't get bumped either, so in
# principle enough *other* distinct URLs could still push it out; that's an accepted, bounded
# degradation (worst case: one extra re-registration cycle), not silent unbounded growth.

PARAM_KEY = "ConnectBackend"
CUSTOM_URL_PARAM = "ConnectCustomUrl"
ACTIVE_PARAM = "ConnectActiveBackend"


def valid_custom_url(url) -> bool:
  # https:// scheme, non-empty host, and -- matching launch_env.sh's literal shell prefix/host
  # check exactly (see connectsel2pnw finding #2) -- no leading/trailing whitespace tolerance. A
  # stray space defeats bash's `${VAR#https://}` prefix match on the shell side; Python used to
  # silently `.strip()` the value before ever reaching this check (in custom_url() below), so a
  # whitespace-padded ConnectCustomUrl could validate here while launch_env.sh rejected it (or a
  # trailing-space value could pass Python's old startswith/host check while baking the space into
  # the exported host on neither side agreeing) -- a Custom vs PNW split-brain. This function is now
  # the ONLY place that decides validity, and custom_url() below must not pre-clean its input.
  if not isinstance(url, str) or url != url.strip():
    return False
  return url.startswith("https://") and len(url.removeprefix("https://").split("/")[0]) > 0


def custom_url(params) -> str:
  raw = params.get(CUSTOM_URL_PARAM)
  # No .strip() here -- see valid_custom_url's docstring for why that used to mask whitespace that
  # launch_env.sh would reject. rstrip("/") is purely cosmetic (trailing slashes never affect
  # validity or host derivation in either this file or launch_env.sh, which both derive the host as
  # everything up to the first "/"); the degenerate "https://".rstrip("/") -> "https:" case still
  # correctly fails the startswith("https://") check right below.
  return raw.rstrip("/") if isinstance(raw, str) else ""


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
      # Drop-then-reinsert so dict insertion order doubles as a touch-order queue (Python dicts
      # preserve insertion order): the just-written key always lands at the end. Cap growth
      # (finding #3) by evicting from the front -- the least-recently-(re)written entries -- until
      # back at CUSTOM_CACHE_MAX. See CUSTOM_CACHE_MAX's comment for what "recently" means here.
      cache.pop(key, None)
      cache[key] = dongle_id
      while len(cache) > CUSTOM_CACHE_MAX:
        cache.pop(next(iter(cache)))
      params.put(CUSTOM_CACHE_PARAM, json.dumps(cache))
  # offline: nothing to stash


def reconcile_backend(params) -> str:
  """Align DongleId with the backend selected by ConnectBackend.

  Called at the top of register(). Returns the effective backend name. Callers skip the
  /persist comma dongle ID restore when the backend is not PNW (restoring the factory comma ID
  would short-circuit registration against Konik/Custom). Offline never attempts network
  registration.

  CRASH SAFETY (connectsel2pnw finding #1, adjudicated against Gemini's "clean" verdict --
  Fable was right, a real data-loss window existed here; see the commit message for the full
  trace). Params writes are not transactional, so a power loss can land between any two of the
  writes below. The write order is deliberately:

    1. stash the OUTGOING backend's current DongleId into ITS OWN cache slot (idempotent: it's
       writing back the value that's already correctly there, this only completes it if the
       PREVIOUS run got cut off before finishing this same step)
    2. unconditionally clear DongleId
    3. commit ACTIVE_PARAM = target_token
    4. (for non-offline targets) restore DongleId from the target's own cache slot, if any

  The original code did (1), then conditionally either `put(DongleId, cached)` or
  `remove(DongleId)`, and only THEN wrote ACTIVE_PARAM last. That put a real dongle ID on disk
  (the target's) while ACTIVE_PARAM still named the OUTGOING backend. A crash there, followed by
  a resumed reconcile_backend() call, mismatched target_token != active_token AGAIN (since
  ACTIVE_PARAM never got written) -- so the resumed run re-entered the "backend changed" branch
  and re-ran step (1), stashing whatever DongleId currently held (by then, the TARGET's already-
  restored ID) into the OUTGOING backend's cache slot, permanently overwriting its real identity
  with the wrong one. Concretely: registered on Konik, switch to PNW, `put(DongleId, PNW_ID)`
  succeeds, crash before `put(ACTIVE_PARAM, "pnw")` -> next boot sees active_token still "konik"
  but DongleId already PNW_ID -> re-stashes PNW_ID into DongleIdCacheKonik, destroying KONIK_ID.

  The fix here is NOT simply moving the ACTIVE_PARAM write earlier (that alone creates a mirror-
  image bug: if DongleId still held the outgoing backend's real ID when ACTIVE_PARAM raced ahead
  of it, a resumed run would misattribute that value to the NEW active_token's cache slot
  instead). The actual fix is unconditionally clearing DongleId (step 2) BEFORE committing
  ACTIVE_PARAM (step 3): this guarantees `registered` is False at the only point where
  target_token can equal active_token while the restore (step 4) is still pending, so a resumed
  run is forced onto the self-heal branch below (which only restores FROM the target's cache,
  never stashes INTO any cache) instead of the cache-stomping "keep fresh" branch. Every
  intermediate state left by a crash between steps 1-4 is safe to resume from:
    - crash before step 1, or between 1 and 2: target_token != active_token still, and DongleId
      (if not yet cleared) still genuinely belongs to active_token -> re-stash is a same-value
      no-op, replay converges normally.
    - crash between 2 and 3: DongleId is None -> registered is False -> step 1's stash is
      skipped on replay (nothing to mis-stash), replay converges normally.
    - crash between 3 and 4: target_token == active_token now, DongleId is None (registered is
      False) -> the token-match branch's self-heal restores from the target's OWN cache slot;
      it never touches the outgoing backend's cache, so that identity was never at risk here.

  There is one more defensive check below, ahead of step 1: if DongleId already equals what's
  cached for the TARGET (not outgoing) backend while target_token != active_token, some prior run
  already completed the swap and only failed to commit ACTIVE_PARAM -- this can't happen from a
  clean run of THIS ordering, but could be left over from an older/pre-fix build or a hand-edited
  param, so it's still guarded rather than assumed impossible.
  """
  try:
    target = get_connect_backend(params)
    target_token = _target_token(target, custom_url(params))
    active_token = params.get(ACTIVE_PARAM) or BACKEND_PNW
    dongle_id = params.get("DongleId")
    registered = dongle_id is not None and dongle_id != UNREGISTERED_DONGLE_ID

    if target_token == active_token:
      if registered:
        # Keep the cache fresh for real backends after a successful registration.
        if _cache_read(params, target_token) != dongle_id:
          _cache_write(params, target_token, dongle_id)
      else:
        # Self-heal: a prior reconcile may have committed ACTIVE_PARAM = target_token but crashed
        # before restoring DongleId from the target's cache (see the crash-safety note above).
        # Finish that restore instead of falling through to fresh registration.
        cached = _cache_read(params, target_token)
        if cached:
          params.put("DongleId", cached)
          cloudlog.event("connectsel_backend_switch", backend=target, dongle_id=cached, restored=True, resumed=True)
      return target

    # Defensive short-circuit: if DongleId already equals what's cached for the TARGET (not the
    # outgoing) backend, some prior run -- this call's own crash-safe ordering makes this
    # unreachable from a clean deploy, but an already-corrupted on-disk state left by an OLDER
    # build (pre-dating this fix) or a hand-edited param could still present it -- already
    # completed the DongleId swap and only failed to commit ACTIVE_PARAM. Do NOT fall through to
    # the stash below: dongle_id no longer belongs to active_token, and stashing it there would
    # be exactly the data-loss bug this function exists to prevent. Just finish the commit.
    if registered and target != BACKEND_OFFLINE and _cache_read(params, target_token) == dongle_id:
      params.put(ACTIVE_PARAM, target_token)
      cloudlog.event("connectsel_backend_switch", backend=target, dongle_id=dongle_id, restored=True, resumed=True)
      return target

    # Backend changed: stash the outgoing backend's ID in its cache slot (no-op for offline, and
    # a no-op-by-construction if a prior crash already cleared DongleId -- registered is False).
    if registered:
      _cache_write(params, active_token, dongle_id)

    if target == BACKEND_OFFLINE:
      # Stay offline with whatever DongleId we already have; no registration, DongleId untouched.
      params.put(ACTIVE_PARAM, target_token)
      cloudlog.event("connectsel_backend_switch", backend=target, dongle_id=dongle_id, restored=False)
      return target

    # Clear the outgoing identity and commit the switch BEFORE restoring the incoming one -- see
    # the crash-safety note above for why this order (not "restore, then commit") is what closes
    # the data-loss window.
    params.remove("DongleId")
    params.put(ACTIVE_PARAM, target_token)

    cached = _cache_read(params, target_token)
    if cached:
      # NOTE: this fork's Params.put blocks until written and takes NO block= kwarg (xnor-era API).
      params.put("DongleId", cached)
      cloudlog.event("connectsel_backend_switch", backend=target, dongle_id=cached, restored=True)
    else:
      cloudlog.event("connectsel_backend_switch", backend=target, restored=False)

    return target
  except Exception:
    # Never block registration: fall back to the PNW default behavior on any failure.
    cloudlog.exception("connectsel_backend_switch failed, using PNW backend behavior")
    return BACKEND_PNW
