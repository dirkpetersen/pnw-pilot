from openpilot.system.curvedb import status as st


def test_ui_text():
  assert st.ui_text(False, "", 0) == st.STARTING_TEXT
  assert st.ui_text(False, st.pack("installed e32ccf54 (10-15)", 1000), 1000 + 2 * 3600) == "installed e32ccf54 (10-15), checked 2 h ago"
  assert st.ui_text(False, st.pack("waiting for unmetered network", 1000), 1010) == "waiting for unmetered network, checked just now"
  assert st.ui_text(False, "held: table changed by hand", 5) == "held: table changed by hand"        # no epoch: shown whole
  assert st.ui_text(False, st.pack("error: auth HTTP 401", 0), 3 * 86400).endswith("checked 3 d ago")
  assert st.ui_text(True, st.pack("installed abc", 0), 9) == st.DISABLED_TEXT                       # toggle ON hides a stale state
  assert st.ui_text(True, st.pack("error: boom", 0), 9) == "error: boom"                            # ...but never an error
