"""Unit tests for the pure GSM/LTE profile enforcer (network2xnor)."""
from openpilot.system.networkd.gsm_profile import (
  GsmProfile, enforce_gsm_profiles,
  LTE_AUTOCONNECT, LTE_PRIORITY, PINNED_AUTOCONNECT, PINNED_PRIORITY,
)


def _p(name, sim_id="", autoconnect="yes", priority="0", apn=""):
  return GsmProfile(name=name, sim_id=sim_id, autoconnect=autoconnect, priority=priority, apn=apn)


def test_healthy_pair_no_changes():
  # unpinned lte already correct + pinned esim already disabled -> zero actions (idempotent, no noise)
  profiles = [
    _p("lte", sim_id="", autoconnect=LTE_AUTOCONNECT, priority=LTE_PRIORITY, apn=""),
    _p("esim", sim_id="89852350524080097762", autoconnect=PINNED_AUTOCONNECT, priority=PINNED_PRIORITY),
  ]
  assert enforce_gsm_profiles(profiles) == []


def test_the_real_bug_state_gets_corrected():
  # exactly the broken live state: esim pinned + autoconnect on + higher priority; lte priority 0
  profiles = [
    _p("lte", sim_id="--", autoconnect="yes", priority="0", apn=""),
    _p("esim", sim_id="89852350524080097762", autoconnect="yes", priority="2", apn=""),
  ]
  actions = dict(enforce_gsm_profiles(profiles))
  # esim must be disabled + deprioritized
  assert "esim" in actions
  assert "connection.autoconnect" in actions["esim"]
  i = actions["esim"].index("connection.autoconnect")
  assert actions["esim"][i + 1] == PINNED_AUTOCONNECT
  # lte priority must be bumped to top
  assert "lte" in actions
  assert "connection.autoconnect-priority" in actions["lte"]


def test_pinned_profile_disabled():
  profiles = [_p("esim", sim_id="123", autoconnect="yes", priority="5")]
  actions = dict(enforce_gsm_profiles(profiles))
  assert actions["esim"] == ["connection.autoconnect", PINNED_AUTOCONNECT,
                             "connection.autoconnect-priority", PINNED_PRIORITY]


def test_unpinned_stale_apn_blanked():
  # the stale AT&T 'ereseller' APN on the unpinned profile must be cleared to auto-negotiate
  profiles = [_p("lte", sim_id="", autoconnect=LTE_AUTOCONNECT, priority=LTE_PRIORITY, apn="ereseller")]
  actions = dict(enforce_gsm_profiles(profiles))
  assert "gsm.apn" in actions["lte"]
  i = actions["lte"].index("gsm.apn")
  assert actions["lte"][i + 1] == ""


def test_unpinned_autoconnect_off_gets_enabled():
  # lte left with autoconnect off (throttle-debug leftover) -> must be re-enabled
  profiles = [_p("lte", sim_id="", autoconnect="no", priority=LTE_PRIORITY, apn="")]
  actions = dict(enforce_gsm_profiles(profiles))
  i = actions["lte"].index("connection.autoconnect")
  assert actions["lte"][i + 1] == LTE_AUTOCONNECT


def test_dash_and_empty_both_mean_unpinned():
  for sid in ("", "--", "  "):
    profiles = [_p("lte", sim_id=sid, autoconnect="no", priority="0", apn="")]
    actions = dict(enforce_gsm_profiles(profiles))
    # treated as the active profile -> enabled, not disabled
    i = actions["lte"].index("connection.autoconnect")
    assert actions["lte"][i + 1] == LTE_AUTOCONNECT


def test_empty_input_no_actions():
  assert enforce_gsm_profiles([]) == []
