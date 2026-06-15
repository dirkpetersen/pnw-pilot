"""
network2xnor: self-healing GSM/LTE profile enforcement (PURE decision logic).

Problem (observed live 2026-06-15): the device carries TWO NetworkManager gsm connection profiles —
a working `lte` (no SIM pin, works with the physical SIM) and a leftover `esim` PINNED to a specific
eSIM (`gsm.sim-id=...`) that ISN'T present in this standard-SIM device. The pinned `esim` had the
higher autoconnect-priority, so when WiFi dropped (leaving home) NM tried `esim` FIRST, it failed to
bind to the modem ("No suitable device found ... device lo"), and LTE never came up at all — no
connectivity away from home, not just a missing indicator.

The runtime fix (nmcli) lives outside the /data/openpilot overlay (in /etc/NetworkManager/), so a
factory reset / AGNOS reflash could reintroduce the broken state. This enforcer lets the always-on
`network_arbiterd` RE-ASSERT the correct profile state on startup, making it durable & self-healing.

Rule (general, not hardcoded to profile names):
  - A gsm profile WITH a sim-id pin is tied to a specific SIM that may be absent -> DISABLE autoconnect
    (and drop its priority) so it can't win the autoconnect race and fail to `lo`.
  - The single UNPINNED gsm profile (works with whatever physical SIM is inserted) -> ENABLE autoconnect,
    blank APN (carrier auto-negotiation — proven across AT&T/Verizon), and give it top priority.

PURE: takes a snapshot of profiles, returns the list of `nmcli con modify` changes to apply. No I/O.
`network_arbiterd` reads the profiles and applies the changes. Idempotent: only emits changes for
settings that are actually wrong, so a healthy device produces zero changes (and zero log noise).
"""
from dataclasses import dataclass

# desired settings for the active (unpinned) LTE profile
LTE_AUTOCONNECT = "yes"
LTE_PRIORITY = "3"
LTE_APN = ""                  # blank -> modem asks carrier for its default (works on AT&T + Verizon)
# desired settings for any SIM-pinned profile that can't bind here
PINNED_AUTOCONNECT = "no"
PINNED_PRIORITY = "-10"


@dataclass(frozen=True)
class GsmProfile:
  name: str            # NM connection id
  sim_id: str          # gsm.sim-id ("" / "--" = unpinned)
  autoconnect: str     # "yes"/"no"
  priority: str        # autoconnect-priority as a string
  apn: str             # gsm.apn ("" = auto)


def _pinned(p: GsmProfile) -> bool:
  """True if the profile is locked to a specific SIM (so it may fail to bind on a different SIM)."""
  sid = (p.sim_id or "").strip()
  return sid not in ("", "--")


def enforce_gsm_profiles(profiles: list[GsmProfile]) -> list[tuple[str, list[str]]]:
  """PURE. Given the current gsm profiles, return the corrective actions as
  (connection_name, [nmcli-modify-args]) tuples — only for settings that are wrong (idempotent).

  Empty list => everything already correct (no-op, no log noise).

  We treat EVERY unpinned profile as an active-LTE candidate (normally there is exactly one) and every
  pinned profile as a should-be-disabled candidate. We never delete anything (reversible).
  """
  actions: list[tuple[str, list[str]]] = []
  for p in profiles:
    if _pinned(p):
      # SIM-pinned -> must not auto-activate (it loses to `lo` and blocks LTE)
      mods: list[str] = []
      if p.autoconnect != PINNED_AUTOCONNECT:
        mods += ["connection.autoconnect", PINNED_AUTOCONNECT]
      if p.priority != PINNED_PRIORITY:
        mods += ["connection.autoconnect-priority", PINNED_PRIORITY]
      if mods:
        actions.append((p.name, mods))
    else:
      # unpinned working profile -> autoconnect on, blank APN, top priority
      mods = []
      if p.autoconnect != LTE_AUTOCONNECT:
        mods += ["connection.autoconnect", LTE_AUTOCONNECT]
      if p.priority != LTE_PRIORITY:
        mods += ["connection.autoconnect-priority", LTE_PRIORITY]
      if (p.apn or "").strip() not in ("", '""'):
        mods += ["gsm.apn", LTE_APN]
      if mods:
        actions.append((p.name, mods))
  return actions
