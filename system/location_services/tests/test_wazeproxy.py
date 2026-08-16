"""wazeproxy2pnw scenario tests — the pure source-selection + proxy-body-transform logic.

The contract under test (WAZE-API.md):
  * default (no override file / empty key) -> keyless PROXY mode
  * legacy override file with key+url      -> DIRECT mode (existing installs unchanged)
  * {"source": "proxy"}                    -> proxy even with a key present
  * proxy body -> the SAME raw-alert shape _poll() builds (downstream untouched)
  * proxy `error` tag -> _ProxyUpstreamErr (state 'nodata' + tag on the err line, never false clear)
  * stale generated_at -> [] (empty)
"""
import json

import pytest

from openpilot.system.location_services import location_servicesd as lsd
from openpilot.system.location_services.location_servicesd import PoliceUpdater, _ProxyUpstreamErr, _now_epoch


def _cfg(**over):
  cfg = dict(lsd.DEFAULT_PROXY)
  cfg.update(over)
  return cfg


class TestSourceSelection:
  def test_default_is_proxy(self):
    # fresh install: no override file -> shipped defaults -> keyless proxy
    assert PoliceUpdater._use_proxy(_cfg()) is True

  def test_key_present_still_proxy_primary_with_fallback(self):
    # rollout phase 1 (owner decision 2026-07-12): proxy is PRIMARY even with a key configured;
    # the key becomes the automatic fallback for when the proxy/website is down
    cfg = _cfg(key="k" * 20)
    assert PoliceUpdater._use_proxy(cfg) is True
    assert PoliceUpdater._fallback_allowed(cfg) is True

  def test_no_key_means_no_fallback(self):
    assert PoliceUpdater._fallback_allowed(_cfg()) is False

  def test_explicit_proxy_wins_over_key_and_disables_fallback(self):
    cfg = _cfg(key="k" * 20, source="proxy")
    assert PoliceUpdater._use_proxy(cfg) is True
    assert PoliceUpdater._fallback_allowed(cfg) is False   # explicit pin = exactly one path

  def test_explicit_direct(self):
    cfg = _cfg(source="direct", key="k" * 20)
    assert PoliceUpdater._use_proxy(cfg) is False
    assert PoliceUpdater._fallback_allowed(cfg) is False

  def test_proxy_needs_url(self):
    assert PoliceUpdater._use_proxy(_cfg(proxy_url="")) is False
    assert PoliceUpdater._use_proxy(_cfg(proxy_url="", source="proxy")) is False

  def test_no_source_at_all_means_nodata_gate(self):
    cfg = _cfg(proxy_url="")
    assert not (PoliceUpdater._use_proxy(cfg) or cfg.get("key"))

  def test_non_string_source_does_not_throw(self):
    # Gemini review finding: {"source": 1} in the JSON must not AttributeError the thread
    assert PoliceUpdater._use_proxy(_cfg(source=1)) is True
    assert PoliceUpdater._use_proxy(_cfg(source=None)) is True


class TestLoadCfgMerge:
  def test_no_file_returns_defaults(self, monkeypatch, tmp_path):
    monkeypatch.setattr(lsd, "PROXY_CFG", str(tmp_path / "absent.json"))
    assert PoliceUpdater._load_cfg(PoliceUpdater.__new__(PoliceUpdater)) == dict(lsd.DEFAULT_PROXY)

  def test_legacy_file_overlays_defaults(self, monkeypatch, tmp_path):
    p = tmp_path / "police_proxy.json"
    p.write_text(json.dumps({"url": "https://x/alerts", "host": "x", "key": "SECRET"}))
    monkeypatch.setattr(lsd, "PROXY_CFG", str(p))
    cfg = PoliceUpdater._load_cfg(PoliceUpdater.__new__(PoliceUpdater))
    assert cfg["key"] == "SECRET" and cfg["proxy_url"]          # merge keeps the proxy defaults around
    assert PoliceUpdater._use_proxy(cfg) is True                # proxy primary...
    assert PoliceUpdater._fallback_allowed(cfg) is True         # ...key = automatic fallback

  def test_corrupt_file_falls_back(self, monkeypatch, tmp_path):
    p = tmp_path / "police_proxy.json"
    p.write_text("{not json")
    monkeypatch.setattr(lsd, "PROXY_CFG", str(p))
    assert PoliceUpdater._load_cfg(PoliceUpdater.__new__(PoliceUpdater)) == dict(lsd.DEFAULT_PROXY)


class TestParseProxyBody:
  def _body(self, alerts, age_s=0, **extra):
    return json.dumps({"generated_at": _now_epoch() - age_s, "ttl_s": 180, "alerts": alerts, **extra})

  def test_pass_through_shape(self):
    alerts = [{"type": "POLICE", "lat": 47.6011, "lon": -122.3128, "magvar": 270,
               "ts": 1751303880000, "uuid": "a1b2", "street": "I-5 N", "town": "Seattle"}]
    out = PoliceUpdater._parse_proxy_body(self._body(alerts))
    assert out == [{"lat": 47.6011, "lon": -122.3128, "magvar": 270,
                    "ts": 1751303880000, "uuid": "a1b2", "street": "I-5 N", "town": "Seattle"}]

  def test_optional_fields_default(self):
    out = PoliceUpdater._parse_proxy_body(self._body([{"lat": 1.0, "lon": 2.0}]))
    assert out == [{"lat": 1.0, "lon": 2.0, "magvar": None, "ts": None,
                    "uuid": None, "street": "", "town": ""}]

  def test_malformed_alert_skipped_not_fatal(self):
    out = PoliceUpdater._parse_proxy_body(self._body([{"lat": "nope", "lon": 2.0}, {"lat": 1.0, "lon": 2.0}, "junk"]))
    assert len(out) == 1

  def test_stale_body_raises_never_false_clear(self):
    # Gemini review finding #1: stale-but-parseable data must go to 'nodata', not an empty 'ok'
    alerts = [{"lat": 1.0, "lon": 2.0}]
    with pytest.raises(_ProxyUpstreamErr, match="stale proxy"):
      PoliceUpdater._parse_proxy_body(self._body(alerts, age_s=lsd.POLICE_PROXY_MAX_AGE_S + 60))

  def test_null_generated_at_is_stale_not_typeerror(self):
    body = json.dumps({"generated_at": None, "ttl_s": 180, "alerts": []})
    with pytest.raises(_ProxyUpstreamErr, match="stale proxy"):
      PoliceUpdater._parse_proxy_body(body)

  def test_garbage_generated_at_is_stale(self):
    body = json.dumps({"generated_at": "soon", "ttl_s": 180, "alerts": []})
    with pytest.raises(_ProxyUpstreamErr, match="stale proxy"):
      PoliceUpdater._parse_proxy_body(body)

  def test_error_tag_raises_with_tag(self):
    with pytest.raises(_ProxyUpstreamErr, match="upstream 429"):
      PoliceUpdater._parse_proxy_body(self._body([], error="upstream 429"))

  def test_non_dict_payload_rejected(self):
    with pytest.raises(ValueError):
      PoliceUpdater._parse_proxy_body(json.dumps([1, 2, 3]))
    with pytest.raises(ValueError):
      PoliceUpdater._parse_proxy_body(json.dumps({"generated_at": _now_epoch(), "alerts": "nope"}))

  def test_html_error_page_rejected(self):
    with pytest.raises(ValueError):
      PoliceUpdater._parse_proxy_body(b"<html>502 Bad Gateway</html>")


class TestSpeedGate:
  """wazespeedgate2pnw: hysteresis gate on GPS speed (m/s) for the paid Waze poll. Arm at
  POLICE_MIN_SPEED_MS, disarm below POLICE_RESUME_SPEED_MS (2 mph lower), hold prev state inside the
  band, fail-closed on unknown speed. Threshold-agnostic -- reads the module constants so the tuning
  knob (POLICE_GATE_MPH) can change without touching these tests."""

  def test_at_or_above_threshold_arms(self):
    assert PoliceUpdater._speed_gate(lsd.POLICE_MIN_SPEED_MS, False) is True
    assert PoliceUpdater._speed_gate(lsd.POLICE_MIN_SPEED_MS + 1.0, False) is True

  def test_below_resume_disarms(self):
    assert PoliceUpdater._speed_gate(lsd.POLICE_RESUME_SPEED_MS - 0.01, True) is False
    assert PoliceUpdater._speed_gate(0.0, True) is False

  def test_inside_band_holds_previous(self):
    mid = (lsd.POLICE_MIN_SPEED_MS + lsd.POLICE_RESUME_SPEED_MS) / 2   # inside the hysteresis band
    assert PoliceUpdater._speed_gate(mid, True) is True
    assert PoliceUpdater._speed_gate(mid, False) is False

  def test_none_speed_fails_closed(self):
    assert PoliceUpdater._speed_gate(None, True) is False
    assert PoliceUpdater._speed_gate(None, False) is False


class _FakeMem:
  """Stand-in for the /dev/shm Params store `_cur_speed` reads -- just enough of the `.get()`
  contract (return_default=True never raises) to drive PoliceUpdater._cur_speed without a real
  Params instance."""
  def __init__(self, value):
    self._value = value

  def get(self, key, return_default=False):
    return self._value


class TestCurSpeed:
  """_cur_speed root-cause regression (on-device traceback 2026-08-16): LastGPSPosition unset ->
  Params.get returns None -> pos stays None (not bytes/str, so json.loads is skipped) ->
  pos.get("ts") raised AttributeError, which escaped the (KeyError, TypeError, ValueError) except
  tuple and aborted the whole police-thread poll cycle every tick GPS was missing. Fixed by an
  isinstance(pos, dict) guard before any pos.get(...), plus AttributeError in the except tuple as
  belt-and-suspenders. _cur_speed is only ever called via `self._mem`, so drive it through a bare
  PoliceUpdater instance (no threading.Thread.__init__/Params needed) with _mem swapped for a fake."""

  @staticmethod
  def _updater(mem_value):
    upd = object.__new__(PoliceUpdater)   # skip __init__ (no real Params/thread needed)
    upd._mem = _FakeMem(mem_value)
    return upd

  def test_missing_param_returns_none_not_raise(self):
    # LastGPSPosition unset -> Params.get(..., return_default=True) returns None
    assert self._updater(None)._cur_speed() is None

  def test_non_dict_json_returns_none_not_raise(self):
    # a parsed-but-non-dict blob (e.g. a bare JSON string or a list) must not raise either
    assert self._updater('"just a string"')._cur_speed() is None
    assert self._updater("[1, 2, 3]")._cur_speed() is None

  def test_bare_string_non_json_returns_none(self):
    # not valid JSON at all -> json.loads raises ValueError, caught same as before
    assert self._updater("not json")._cur_speed() is None

  def test_valid_dict_still_returns_speed(self):
    # freshness guard must still work for the normal dict case (unchanged behavior)
    import time
    pos = {"speed": 12.3, "ts": time.monotonic()}
    assert self._updater(json.dumps(pos))._cur_speed() == pytest.approx(12.3)

  def test_stale_dict_still_returns_none(self):
    import time
    pos = {"speed": 12.3, "ts": time.monotonic() - lsd.POLICE_SPEED_MAX_AGE_S - 1.0}
    assert self._updater(json.dumps(pos))._cur_speed() is None


class TestResolveDeviceId:
  """wazespeedgate2pnw: the x-device-id sent to the proxy for its per-device daily limit (750/day).
  _resolve_device_id is a pure static helper (get_fn: callable(key) -> value|None) so it's testable
  without a real Params store."""

  def test_hardware_serial_wins(self):
    vals = {"HardwareSerial": "eb1f2f7", "DongleId": "someDongle"}
    assert PoliceUpdater._resolve_device_id(vals.get) == "eb1f2f7"

  def test_falls_back_to_dongle_id(self):
    vals = {"HardwareSerial": None, "DongleId": "someDongle"}
    assert PoliceUpdater._resolve_device_id(vals.get) == "someDongle"

  def test_neither_set_falls_back_to_noid(self):
    vals = {"HardwareSerial": None, "DongleId": None}
    assert PoliceUpdater._resolve_device_id(vals.get) == "noid"

  def test_empty_string_falls_back_to_noid(self):
    # an empty/whitespace-only serial must not ship as a blank x-device-id
    vals = {"HardwareSerial": "   ", "DongleId": None}
    assert PoliceUpdater._resolve_device_id(vals.get) == "noid"

  def test_bytes_are_decoded(self):
    vals = {"HardwareSerial": b"eb1f2f7", "DongleId": None}
    assert PoliceUpdater._resolve_device_id(vals.get) == "eb1f2f7"
