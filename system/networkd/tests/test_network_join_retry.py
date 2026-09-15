"""hotspotretry2pnw — classify_join_failure: which `nmcli con up` failures are worth one retry.

The loop-level consequences (the one free retry, the bound, the ledger) are pinned in
test_network_arbiter_sequences.py; this file pins the classifier itself, because it is a list of
strings matched against another program's output and that is exactly the kind of thing that rots
silently. The two strings at the top are the MEASURED ones from the truck, 2026-09-14 18:55/18:57 PT.
"""
from openpilot.system.networkd.network_arbiter import TRANSIENT_JOIN_ERRORS, classify_join_failure


class TestTheMeasuredFailures:
  def test_the_1855_secrets_failure(self):
    """wpa_supplicant: `CTRL-EVENT-SSID-TEMP-DISABLED ... reason=WRONG_KEY`; NM then asks for secrets and
    the arbiter's nmcli has no agent to answer. The same profile joined the same phone cleanly 3 h later."""
    assert classify_join_failure(
      "Error: Connection activation failed: (7) Secrets were required, but not provided.") == "transient"

  def test_the_1857_ssid_not_found_failure(self):
    """An iPhone hides its hotspot unless the Personal Hotspot screen is open or a client is attached."""
    assert classify_join_failure(
      "Error: Connection activation failed: (53) The Wi-Fi network could not be found.") == "transient"
    assert classify_join_failure("Error: Connection activation failed: ssid-not-found") == "transient"
    assert classify_join_failure("Error: Connection activation failed: association took too long") == "transient"


class TestTheDefaultIsReal:
  """Under-matching costs nothing (today's behaviour); over-matching costs one 20 s poll. So anything
  not recognised keeps the ledger exactly as it is."""

  def test_an_unknown_error_is_real(self):
    assert classify_join_failure("Error: Connection activation failed: (1) Unknown reason.") == "real"

  def test_an_empty_error_is_real(self):
    assert classify_join_failure("") == "real"
    assert classify_join_failure(None) == "real"

  def test_a_MISSING_PROFILE_is_real_not_a_missing_network(self):
    """The trap in matching 'not found': `Error: unknown connection 'openpilot connection X'` and
    `Error: Connection 'X' (…) does not exist` are configuration faults, not a sleeping hotspot. The
    list matches 'network could not be found', never a bare 'not found'."""
    assert classify_join_failure("Error: unknown connection 'openpilot connection Dirk's iPhone 13'.") == "real"
    assert classify_join_failure("Error: Connection 'X' does not exist.") == "real"
    assert not any(tok.strip() == "not found" for tok in TRANSIENT_JOIN_ERRORS)

  def test_a_dead_router_is_real(self):
    """The failure the escalating backoff exists for: associated, DHCP never answers."""
    assert classify_join_failure(
      "Error: Connection activation failed: (7) IP configuration could not be reserved.") == "real"


class TestItReadsTheTextTheWayNmcliPrintsIt:
  def test_case_and_whitespace_do_not_matter(self):
    assert classify_join_failure("ERROR: SECRETS WERE REQUIRED, BUT NOT PROVIDED.") == "transient"
    assert classify_join_failure("Secrets   were\n  required") == "transient"

  def test_a_multi_line_stderr_still_matches(self):
    """nmcli prints a hint line under the error on some builds."""
    stderr = ("Error: Connection activation failed: (7) Secrets were required, but not provided.\n"
              + "Hint: use 'journalctl -xe NM_CONNECTION=...'")
    assert classify_join_failure(stderr) == "transient"
