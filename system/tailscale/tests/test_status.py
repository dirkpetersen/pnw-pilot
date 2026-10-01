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
  # first arg = Remote SSH ENABLED (the Disable toggle is OFF)
  assert st.ui_text(False, "connected 100.1.1.1") == st.DISABLED_TEXT  # stale param never shown while disabled
  assert st.ui_text(False, "connecting") == st.DISABLED_TEXT
  stuck = "error tailscaled still running after toggle off; could not stop it"
  assert st.ui_text(False, stuck) == stuck                      # ...but a stuck daemon must stay visible after OFF
  assert st.ui_text(True, "off") == "connecting"                # the daemon's own pre-flip 'off' is not shown as state
  assert st.title_word(st.DISABLED_TEXT) == "disconnected" and st.title_word("off") == ""
  assert st.title_word(st.NO_LINK) == "disconnected" and st.title_word(st.UNCONFIGURED) == "unconfigured"
  assert st.title_word(st.INSTALL_DEFERRED) == "installing" and st.title_word("connected 100.1.1.1") == "connected"
  assert st.title_word("error boom") == "error" and st.title_word("mystery") == "mystery"
  assert st.ui_text(True, "") == "connecting"                     # on, daemon not yet heard from
  assert st.ui_text(True, "error boom") == "error boom"
