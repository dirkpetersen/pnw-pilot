#!/usr/bin/env python3
"""connectsel2pnw: tests for the crash-safety, validation-unification, and cache-cap fixes made
in response to the Fable + Gemini review (see the docstring on reconcile_backend() for the full
data-loss trace this is regression-testing).

Host-side pure-module test: no capnp/cereal available outside the device venv (see the openpilot
skill's "host lacks capnp for full pytest" note), so openpilot.common.swaglog is stubbed out
before import -- connect_backend.py only uses cloudlog.event()/cloudlog.exception(), both no-ops
here. Params is faked with a plain dict; connect_backend.py's Params usage is limited to
get/put/remove, which is all FakeParams implements.
"""

import json
import subprocess
import sys
import types
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

# Stub the swaglog import chain (it pulls in cereal -> capnp, unavailable on host).
_swaglog_stub = types.ModuleType("openpilot.common.swaglog")


class _CloudlogStub:
  def event(self, *args, **kwargs):
    pass

  def exception(self, *args, **kwargs):
    pass

  def warning(self, *args, **kwargs):
    pass


_swaglog_stub.cloudlog = _CloudlogStub()
sys.modules["openpilot.common.swaglog"] = _swaglog_stub

from openpilot.common import connect_backend as cb


class FakeParams:
  """Dict-backed stand-in for common.params.Params -- get/put/remove is the full interface
  connect_backend.py uses."""

  def __init__(self, initial: dict[str, str] | None = None):
    self._d: dict[str, str] = dict(initial or {})

  def get(self, key):
    return self._d.get(key)

  def put(self, key, value):
    self._d[key] = str(value)

  def remove(self, key):
    self._d.pop(key, None)


# ---------------------------------------------------------------------------------------------
# Finding #1: reconcile-backend crash-safety / dongle-cache data-loss window
# ---------------------------------------------------------------------------------------------


def test_reconcile_normal_switch_konik_to_pnw_no_crash():
  """Sanity: an uninterrupted konik -> pnw switch round-trips both identities."""
  p = FakeParams(
    {
      "ConnectBackend": "0",  # target = pnw
      "ConnectActiveBackend": "konik",
      "DongleId": "KONIK_ID",
      "DongleIdCacheKonik": "KONIK_ID",
      "DongleIdCachePnw": "PNW_ID",
    }
  )
  backend = cb.reconcile_backend(p)
  assert backend == "pnw"
  assert p.get("DongleId") == "PNW_ID"
  assert p.get("ConnectActiveBackend") == "pnw"
  assert p.get("DongleIdCacheKonik") == "KONIK_ID"  # outgoing identity preserved
  assert p.get("DongleIdCachePnw") == "PNW_ID"


def test_reconcile_resume_after_crash_before_active_param_commit():
  """The exact Fable-flagged window: power loss after DongleId is set to the target's cached ID
  but before ConnectActiveBackend is committed. Simulated by hand-constructing that exact
  on-disk state (as if reconcile_backend() got cut off mid-way) and then calling
  reconcile_backend() again, the way the device would on the next boot.

  Pre-fix behavior: the resumed call would re-enter the "backend changed" branch (since
  ConnectActiveBackend still read "konik"), and re-stash the CURRENT DongleId (already PNW_ID at
  this point) into DongleIdCacheKonik -- permanently overwriting the real KONIK_ID with PNW_ID.
  Post-fix: DongleId is unconditionally cleared before ConnectActiveBackend is committed, so by
  the time a resumed run can see target_token == active_token, `registered` is always False,
  routing it onto the self-heal restore path instead of the cache-stomping freshen path.
  """
  p = FakeParams(
    {
      "ConnectBackend": "0",  # target = pnw
      "ConnectActiveBackend": "konik",  # STALE: crash happened before this got updated
      "DongleId": "PNW_ID",  # already swapped in by the interrupted run
      "DongleIdCacheKonik": "KONIK_ID",  # the real, still-correct identity
      "DongleIdCachePnw": "PNW_ID",
    }
  )
  backend = cb.reconcile_backend(p)
  assert backend == "pnw"
  assert p.get("DongleId") == "PNW_ID"
  assert p.get("ConnectActiveBackend") == "pnw"
  # The konik identity must NOT have been clobbered by the resumed run.
  assert p.get("DongleIdCacheKonik") == "KONIK_ID", "konik identity was destroyed by the resumed reconcile"
  assert p.get("DongleIdCachePnw") == "PNW_ID"


def test_reconcile_resume_after_crash_dongleid_cleared_no_cache_yet():
  """Same window, but the target (pnw) had no cached ID yet, so the interrupted run's step was
  `remove(DongleId)` rather than a restore put. Crash lands after the clear, before
  ConnectActiveBackend commits. Resume must finish the switch (commit ACTIVE_PARAM) without
  fabricating an identity or touching konik's cache."""
  p = FakeParams(
    {
      "ConnectBackend": "0",
      "ConnectActiveBackend": "konik",
      # DongleId already removed by the interrupted run (no PNW cache existed to restore from).
      "DongleIdCacheKonik": "KONIK_ID",
    }
  )
  backend = cb.reconcile_backend(p)
  assert backend == "pnw"
  assert p.get("DongleId") is None
  assert p.get("ConnectActiveBackend") == "pnw"
  assert p.get("DongleIdCacheKonik") == "KONIK_ID"


def test_reconcile_crash_before_dongleid_cleared_replays_safely():
  """Crash before step 2 (DongleId still holds the OUTGOING (konik) id, ACTIVE_PARAM still
  stale). Resume must re-stash the same (correct) value -- idempotent, no corruption -- then
  complete the switch."""
  p = FakeParams(
    {
      "ConnectBackend": "0",
      "ConnectActiveBackend": "konik",
      "DongleId": "KONIK_ID",  # not yet touched by the interrupted run
      "DongleIdCacheKonik": "KONIK_ID",
      "DongleIdCachePnw": "PNW_ID",
    }
  )
  backend = cb.reconcile_backend(p)
  assert backend == "pnw"
  assert p.get("DongleId") == "PNW_ID"
  assert p.get("ConnectActiveBackend") == "pnw"
  assert p.get("DongleIdCacheKonik") == "KONIK_ID"


def test_reconcile_offline_switch_stash_then_crash_resume():
  """Switching TO offline stashes the outgoing id, then commits ACTIVE_PARAM in one write (no
  DongleId mutation for offline). A crash between the stash and the ACTIVE_PARAM commit must
  replay safely."""
  p = FakeParams(
    {
      "ConnectBackend": "3",  # target = offline
      "ConnectActiveBackend": "konik",
      "DongleId": "KONIK_ID",
      "DongleIdCacheKonik": "KONIK_ID",
    }
  )
  backend = cb.reconcile_backend(p)
  assert backend == "offline"
  assert p.get("DongleId") == "KONIK_ID"  # offline keeps whatever DongleId is present
  assert p.get("ConnectActiveBackend") == "offline"
  assert p.get("DongleIdCacheKonik") == "KONIK_ID"


def test_reconcile_multiple_round_trips_preserve_both_identities():
  """konik -> pnw -> konik with no crashes: both identities must still be independently cached
  and correctly restored on the way back."""
  p = FakeParams(
    {
      "ConnectBackend": "0",
      "ConnectActiveBackend": "konik",
      "DongleId": "KONIK_ID",
      "DongleIdCacheKonik": "KONIK_ID",
    }
  )
  assert cb.reconcile_backend(p) == "pnw"
  assert p.get("DongleId") is None  # no PNW cache existed yet
  # Simulate a fresh PNW registration handing back a brand-new ID.
  p.put("DongleId", "PNW_ID")

  p.put("ConnectBackend", "1")  # switch back to konik
  assert cb.reconcile_backend(p) == "konik"
  assert p.get("DongleId") == "KONIK_ID"
  assert p.get("DongleIdCachePnw") == "PNW_ID"
  assert p.get("DongleIdCacheKonik") == "KONIK_ID"


# ---------------------------------------------------------------------------------------------
# Finding #2: Python <-> shell custom-URL validation must agree
# ---------------------------------------------------------------------------------------------

PYTHON_VALIDATION_CASES = [
  ("https://foo.com", True),
  (" https://foo.com", False),  # leading space
  ("https://foo.com ", False),  # trailing space
  ("https://", False),  # empty host
  ("https:///", False),  # empty host, explicit path
  ("", False),
  ("http://foo.com", False),  # wrong scheme
  ("https://foo.com/path", True),
  ("https://foo.com///", True),  # host still non-empty, trailing slashes are cosmetic
]


def test_python_validation_matches_expected_table():
  for url, expected in PYTHON_VALIDATION_CASES:
    assert cb.valid_custom_url(url) == expected, f"valid_custom_url({url!r}) expected {expected}"


LAUNCH_ENV_CHECK_SNIPPET = r'''
check() {
  local CONNECT_CUSTOM_URL="$1"
  local CONNECT_CUSTOM_URL_TRIMMED="${CONNECT_CUSTOM_URL#"${CONNECT_CUSTOM_URL%%[![:space:]]*}"}"
  CONNECT_CUSTOM_URL_TRIMMED="${CONNECT_CUSTOM_URL_TRIMMED%"${CONNECT_CUSTOM_URL_TRIMMED##*[![:space:]]}"}"
  local CONNECT_CUSTOM_HOST="${CONNECT_CUSTOM_URL#https://}"
  CONNECT_CUSTOM_HOST="${CONNECT_CUSTOM_HOST%%/*}"
  if [ "$CONNECT_CUSTOM_URL" = "$CONNECT_CUSTOM_URL_TRIMMED" ] \
     && [ "${CONNECT_CUSTOM_URL#https://}" != "$CONNECT_CUSTOM_URL" ] && [ -n "$CONNECT_CUSTOM_HOST" ]; then
    echo VALID
  else
    echo INVALID
  fi
}
check "$1"
'''


def _shell_validate(url: str) -> bool:
  """Runs the exact validation snippet extracted from launch_env.sh's elif condition against a
  candidate URL, so this test breaks if the two files drift apart again."""
  result = subprocess.run(
    ["bash", "-c", LAUNCH_ENV_CHECK_SNIPPET, "_", url],
    capture_output=True,
    text=True,
    timeout=10,
  )
  return result.stdout.strip() == "VALID"


def test_shell_validation_matches_python_for_every_case():
  for url, expected in PYTHON_VALIDATION_CASES:
    shell_result = _shell_validate(url)
    assert shell_result == expected, f"launch_env.sh check({url!r}) = {shell_result}, python says {expected} -- split-brain"


def test_launch_env_sh_contains_the_tested_snippet():
  """Make sure the snippet above is actually still what's shipped in launch_env.sh (guards
  against this test silently testing a stale copy after a future edit)."""
  launch_env = (REPO_ROOT / "launch_env.sh").read_text()
  assert 'CONNECT_CUSTOM_URL_TRIMMED="${CONNECT_CUSTOM_URL#"${CONNECT_CUSTOM_URL%%[![:space:]]*}"}"' in launch_env
  assert '[ -n "$CONNECT_CUSTOM_HOST" ]' in launch_env


# ---------------------------------------------------------------------------------------------
# Finding #3: DongleIdCacheCustom must not grow unbounded
# ---------------------------------------------------------------------------------------------


def test_custom_cache_evicts_oldest_beyond_cap():
  p = FakeParams()
  urls = [f"https://custom{i}.example.com" for i in range(cb.CUSTOM_CACHE_MAX + 3)]
  for i, url in enumerate(urls):
    token = cb._target_token(cb.BACKEND_CUSTOM, url)
    cb._cache_write(p, token, f"ID_{i}")

  cache = json.loads(p.get(cb.CUSTOM_CACHE_PARAM))
  assert len(cache) == cb.CUSTOM_CACHE_MAX, f"cache grew to {len(cache)}, expected cap {cb.CUSTOM_CACHE_MAX}"

  # The earliest-written entries (URLs 0, 1, 2) must have been evicted; the most recent
  # CUSTOM_CACHE_MAX must all still be present.
  evicted_keys = {cb._custom_key(u) for u in urls[:3]}
  kept_keys = {cb._custom_key(u) for u in urls[3:]}
  assert evicted_keys.isdisjoint(cache.keys()), "oldest entries were not evicted"
  assert kept_keys == set(cache.keys()), "wrong set of entries survived the cap"


def test_custom_cache_rewrite_of_same_value_does_not_grow_or_reorder_unnecessarily():
  p = FakeParams()
  token = cb._target_token(cb.BACKEND_CUSTOM, "https://a.example.com")
  cb._cache_write(p, token, "ID_A")
  before = p.get(cb.CUSTOM_CACHE_PARAM)
  cb._cache_write(p, token, "ID_A")  # same value again -- should be a no-op (no extra param write)
  assert p.get(cb.CUSTOM_CACHE_PARAM) == before


def test_custom_cache_stays_at_cap_indefinitely():
  """Regression for 'grows forever': hammer many more distinct URLs than the cap and confirm the
  cache size never exceeds CUSTOM_CACHE_MAX at any point, not just at the end."""
  p = FakeParams()
  for i in range(50):
    token = cb._target_token(cb.BACKEND_CUSTOM, f"https://typo{i}.example.com")
    cb._cache_write(p, token, f"ID_{i}")
    cache = json.loads(p.get(cb.CUSTOM_CACHE_PARAM))
    assert len(cache) <= cb.CUSTOM_CACHE_MAX
