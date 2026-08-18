"""curveoverride2pnw — curated per-location curve ceilings.

A curated table is a SAFETY NET for locations that have repeatedly forced a takeover, layered on top
of vision and map rather than replacing them. These tests pin the properties that make it safe: it
can only ever lower a target, it is direction-gated, malformed data is dropped rather than clamped,
and it can never raise into the control loop.
"""
import json
import math

import pytest

from openpilot.selfdrive.controls.lib.vtsc_pnw import curve_overrides as CO
from openpilot.selfdrive.controls.lib.vtsc_pnw.curve_overrides import CurveOverrides, MPH_TO_MS

WOODLAND = (45.96455, -122.80992, 147.0)     # lat, lon, southbound bearing (from the drive report)


def _fresh(monkeypatch, entries, local=None):
    monkeypatch.setattr(CO, "_load_dir", lambda: entries)
    monkeypatch.setattr(CO, "_load_local", lambda: (local or []))
    return CurveOverrides()


def _entry(**over):
    e = {"name": "test curve", "lat": WOODLAND[0], "lon": WOODLAND[1],
         "bearing": WOODLAND[2], "v_max_mph": 72}
    e.update(over)
    return e


class TestShippedTable:
    def test_every_shipped_entry_is_valid_and_carries_evidence(self):
        """A curated entry without a drive report behind it is a guess. The table must not accumulate
        guesses that later read as measurements."""
        import os
        rows = []
        for fn in os.listdir(CO._DATA_DIR):
            if fn.endswith(".json"):
                with open(os.path.join(CO._DATA_DIR, fn)) as f:
                    rows.extend(json.load(f))
        assert rows, "the shipped table must not be empty"
        for e in rows:
            assert CO._valid(e) is not None, f"shipped entry is invalid: {e.get('name')}"
            assert e.get("evidence"), f"entry {e.get('name')} has no evidence path"
            assert e.get("note"), f"entry {e.get('name')} has no explanation"
            assert e.get("dir"), f"entry {e.get('name')} has no direction"


class TestResolution:
    def test_matches_when_approaching_in_the_right_direction(self, monkeypatch):
        ov = _fresh(monkeypatch, [_entry()])
        got = ov.resolve(0.0, WOODLAND[0] + 0.003, WOODLAND[1] - 0.003, 147.0)
        assert got is not None
        v, d, name = got
        assert v == pytest.approx(72 * MPH_TO_MS, rel=1e-6)
        assert 0.0 < d < 800.0 and name == "test curve"

    def test_ignores_the_opposite_direction(self, monkeypatch):
        """A curve is only a problem one way round; the northbound carriageway must not be slowed."""
        ov = _fresh(monkeypatch, [_entry()])
        assert ov.resolve(0.0, WOODLAND[0], WOODLAND[1], 327.0) is None

    def test_ignores_a_location_far_away(self, monkeypatch):
        ov = _fresh(monkeypatch, [_entry()])
        assert ov.resolve(0.0, WOODLAND[0] + 0.5, WOODLAND[1], 147.0) is None

    def test_picks_the_lowest_ceiling_not_the_nearest(self, monkeypatch):
        """A tighter curve just past a gentler one must not be masked by proximity."""
        near = _entry(name="near gentle", v_max_mph=70)
        far = _entry(name="far tight", v_max_mph=45,
                     lat=WOODLAND[0] - 0.002, lon=WOODLAND[1] + 0.002)
        ov = _fresh(monkeypatch, [near, far])
        v, _d, name = ov.resolve(0.0, WOODLAND[0] + 0.001, WOODLAND[1] - 0.001, 147.0)
        assert name == "far tight" and v == pytest.approx(45 * MPH_TO_MS, rel=1e-6)

    def test_missing_bearing_keeps_the_entry(self, monkeypatch):
        ov = _fresh(monkeypatch, [_entry()])
        assert ov.resolve(0.0, WOODLAND[0], WOODLAND[1], None) is not None


class TestBadDataIsDroppedNotClamped:
    @pytest.mark.parametrize("bad", [
        {"lat": 999.0, "lon": 0.0, "v_max_mph": 60},          # impossible latitude
        {"lat": 45.0, "lon": -122.0, "v_max_mph": 5},          # below the floor
        {"lat": 45.0, "lon": -122.0, "v_max_mph": 200},        # absurdly high
        {"lat": 45.0, "lon": -122.0, "v_max_mph": float('nan')},
        {"lat": "x", "lon": -122.0, "v_max_mph": 60},
        {"lat": 45.0, "lon": -122.0},                          # no ceiling at all
        "not a dict",
    ])
    def test_malformed_entries_are_dropped(self, bad):
        assert CO._valid(bad) is None

    def test_a_bad_entry_does_not_poison_the_good_ones(self, monkeypatch):
        ov = _fresh(monkeypatch, [{"lat": "bad"}, _entry()])
        assert ov.resolve(0.0, WOODLAND[0], WOODLAND[1], 147.0) is not None

    def test_resolve_never_raises(self, monkeypatch):
        ov = _fresh(monkeypatch, [_entry()])
        for args in ((0.0, None, None, 147.0), (0.0, float('nan'), 0.0, None),
                     (0.0, WOODLAND[0], WOODLAND[1], float('nan'))):
            ov.resolve(*args)          # must not raise


class TestDeviceLocalTuning:
    def test_local_file_overrides_a_shipped_entry_by_name(self, monkeypatch):
        ov = _fresh(monkeypatch, [_entry(v_max_mph=72)], local=[_entry(v_max_mph=60)])
        v, _d, _n = ov.resolve(0.0, WOODLAND[0], WOODLAND[1], 147.0)
        assert v == pytest.approx(60 * MPH_TO_MS, rel=1e-6), "on-road tuning must win"

    def test_a_malformed_local_file_falls_back_to_the_shipped_table(self, monkeypatch):
        ov = _fresh(monkeypatch, [_entry(v_max_mph=72)], local=[{"lat": "junk"}])
        v, _d, _n = ov.resolve(0.0, WOODLAND[0], WOODLAND[1], 147.0)
        assert v == pytest.approx(72 * MPH_TO_MS, rel=1e-6)


def test_the_ceiling_can_never_undercut_v_min():
    from openpilot.selfdrive.controls.lib.vtsc_pnw import vtsc_constants as C
    assert CO._V_MIN_MPH * MPH_TO_MS >= C.V_MIN
    assert math.isfinite(CO._V_MAX_MPH)
