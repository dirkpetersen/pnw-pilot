"""waysel2pnw: guard the VTSCStatus publish -> ces_events cherry-pick contract.

This contract has silently broken three times: 62a51a4772, satele2pnw's polKey, and waysel2pnw itself.
The failure is always invisible -- the field is computed, the column exists in ces_events, and it reads
null, which is indistinguishable from "the feature never triggered". The waysel2pnw case shipped to the
production channel and was only caught by inspecting the car afterwards.

There are two distinct ways to break it, and both are tested here:
  1. ces_pnw asks for a key the publisher never emits.
  2. the publisher puts a key on `self.msg` (which feeds the CEREAL publisher's fixed whitelist)
     instead of on the /dev/shm VTSCStatus dict, which is the only channel ces_pnw reads.
"""
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw import VTSC_TELE_KEYS
from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_controller import VTSCController


def _payload():
  """Build an overlay payload without constructing a real controller (no Params, no device)."""
  c = object.__new__(VTSCController)
  c.msg = dict(enabled=False, active=False, state="idle", vTarget=0.0, vCruise=0.0,
               apexDist=-1.0, apexCurvature=0.0, vCurveSafe=0.0, timeToApex=-1.0)
  c._tele_pen = 0.0
  c._tele_pitch = None
  c._tele_dir = ""
  c._tele_map_raw = c._tele_map_eff = c._tele_map_d = 0.0
  c._tele_map_floored = False
  c._tele_mapk = c._tele_mapk_d = c._tele_mapk_v = 0.0
  c._tele_mapk_n = 0
  c._tele_mapk_ahead = False
  c._tele_vis_k = c._tele_vis_d = c._tele_vis_v = 0.0
  c._tele_curve_win = "none"
  c._tele_rsn_map = c._tele_rsn_vis = -1.0
  return c.overlay_payload()


class TestOverlayContract:
  def test_publisher_emits_every_key_ces_pnw_reads(self):
    """The load-bearing assertion. Fails on the shipped-broken waysel2pnw; passes once fixed."""
    published = set(_payload())
    missing = sorted(set(VTSC_TELE_KEYS) - published)
    assert not missing, f"ces_pnw reads {missing} from VTSCStatus but overlay_payload() never emits them"

  def test_payload_is_json_safe(self):
    """It is serialised into a param; a stray non-serialisable value would be swallowed by the
    bare except in _publish_overlay and lose the WHOLE snapshot, not just one field."""
    import json
    json.dumps(_payload())

  def test_no_nan_or_inf_in_a_default_payload(self):
    """NaN/Inf serialise to bare NaN/Infinity, which is not valid JSON and breaks strict readers."""
    import math
    bad = {k: v for k, v in _payload().items()
           if isinstance(v, float) and not math.isfinite(v)}
    assert not bad, f"non-finite values would reach ces_events.jsonl as bare tokens: {bad}"

  def test_keys_the_curve_investigation_needs_are_present(self):
    """Named explicitly so a future refactor cannot quietly drop the fields the Tumwater 2026-09-03
    over-slowdown showed were missing: who authored the cap, on what evidence, and how far ahead."""
    published = set(_payload())
    for k in ("curveWin", "rsnMap", "rsnVis", "apexCurvature", "apexDist", "vCurveSafe", "timeToApex"):
      assert k in published, f"{k} missing from the VTSCStatus payload"
