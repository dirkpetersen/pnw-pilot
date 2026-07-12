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

  def test_legacy_key_forces_direct(self):
    # existing installs (key+url in the override file) keep polling RapidAPI directly
    assert PoliceUpdater._use_proxy(_cfg(key="k" * 20)) is False

  def test_explicit_proxy_wins_over_key(self):
    assert PoliceUpdater._use_proxy(_cfg(key="k" * 20, source="proxy")) is True

  def test_explicit_direct(self):
    assert PoliceUpdater._use_proxy(_cfg(source="direct")) is False

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
    assert PoliceUpdater._use_proxy(cfg) is False               # ...but the key selects direct

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
