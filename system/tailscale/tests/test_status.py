from openpilot.system.tailscale import status as st


def test_classify_running_prefers_ipv4():
  assert st.classify({"BackendState": "Running", "TailscaleIPs": ["fd7a::1", "100.64.0.5"]}) == ("running", "100.64.0.5")


def test_classify_running_without_address_is_starting():
  assert st.classify({"BackendState": "Running", "TailscaleIPs": []}) == ("starting", None)


def test_classify_states():
  for backend, want in [("NeedsLogin", "needs_login"), ("Stopped", "stopped"), ("Starting", "starting"),
                        ("NoState", "starting"), ("NeedsMachineAuth", "needs_approval")]:
    assert st.classify({"BackendState": backend})[0] == want


def test_classify_unknown_is_not_silently_connecting():
  assert st.classify({"BackendState": "Wat"})[0] == "unknown"
  assert st.classify({})[0] == "unknown"


def test_error_is_one_line_and_bounded():
  e = st.error("a\nb   c" + "x" * 500)
  assert e.startswith("error a b c") and "\n" not in e and len(e) <= len(st.ERROR_PREFIX) + st.MAX_REASON


def test_ui_text():
  assert st.ui_text(False, "connected 100.1.1.1") == "off"      # stale param never shown while the toggle is off
  assert st.ui_text(True, "") == "starting"                     # on, daemon not yet heard from
  assert st.ui_text(True, "error boom") == "error boom"
