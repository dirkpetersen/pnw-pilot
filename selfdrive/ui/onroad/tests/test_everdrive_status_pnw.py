"""everdrive2pnw: the one-line EverDrive box in the lower-right corner.

Pure formatting/visibility logic. Nothing here draws: `_render` is never called, no raylib window is
opened and no device is needed.

WHY THE MODULE IS LOADED WITH STUBS RATHER THAN IMPORTED
`everdrive_status.py` imports `openpilot.common.params` (a COMPILED params_pyx), `ui_state` and the
raylib application layer; in an unbuilt worktree those abort the interpreter while loading the capnp
schemas, and on a built tree they would drag a real /dev/shm params store and a real font atlas into
a unit test. So the REAL SOURCE FILE is executed here (a typo or a bad import in it still fails this
module), with only its six leaf dependencies replaced -- and sys.modules is restored immediately
afterwards so nothing leaks into the rest of the suite. What is NOT covered by this choice is stated
plainly: the raylib draw calls in `_render`, and the real font metrics behind `measure_text_cached`.
"""
import contextlib
import importlib.util
import pathlib
import sys
import types

import pytest

_SRC = pathlib.Path(__file__).resolve().parents[1] / "everdrive_status.py"
assert _SRC.is_file(), f"everdrive_status.py not found at {_SRC}"

_MISSING = object()


class FakeMem:
  """The /dev/shm params store. `payload = None` is a key that was never written -- which is what
  Params.get(..., return_default=True) returns, and the NORMAL state on a truck with no EverDrive."""

  def __init__(self):
    self.payload = None
    self.raises = None
    self.reads = 0

  def get(self, key, return_default=False):
    assert key == "EverDriveStatus", key
    self.reads += 1
    if self.raises is not None:
      raise self.raises
    return self.payload


class FakeUiParams:
  def __init__(self):
    self.disabled = False
    self.raises = None

  def get_bool(self, key):
    assert key == "DisableEverDrive", key
    if self.raises is not None:
      raise self.raises
    return self.disabled


class FakeLog:
  def __init__(self):
    self.calls = []

  def exception(self, *a, **k):
    self.calls.append(("exception", a))

  def error(self, *a, **k):
    self.calls.append(("error", a))

  def warning(self, *a, **k):
    self.calls.append(("warning", a))


MEM = FakeMem()
UI = types.SimpleNamespace(params=FakeUiParams())
LOG = FakeLog()
MEASURED: list[str] = []


def _measure(font, text, fs, spacing=0):
  """Deterministic monotone stand-in for measure_text_cached. NOT the device's font metrics -- the
  real px widths are verified separately against the .fnt files; here only the CALL COUNT and the
  fact that measurement is deferred are under test."""
  MEASURED.append(text)
  return types.SimpleNamespace(x=len(text) * fs * 0.5, y=fs)


def _mod(name, **attrs):
  m = types.ModuleType(name)
  for k, v in attrs.items():
    setattr(m, k, v)
  return m


class _StubWidget:
  def __init__(self):
    self._rect = None


try:
  import pyray as _rl                                        # noqa: F401  (real when available)
  _PYRAY = {}
except Exception:                                            # pragma: no cover - CI without raylib
  _PYRAY = {"pyray": _mod("pyray", Color=lambda *a: a, Rectangle=lambda *a: a, Vector2=lambda *a: a)}


_STUBS = {
  "openpilot.common.params": _mod("openpilot.common.params", Params=lambda *a, **k: MEM),
  "openpilot.common.swaglog": _mod("openpilot.common.swaglog", cloudlog=LOG),
  "openpilot.selfdrive.ui.ui_state": _mod("openpilot.selfdrive.ui.ui_state", ui_state=UI),
  "openpilot.system.ui.lib.application": _mod(
    "openpilot.system.ui.lib.application",
    gui_app=types.SimpleNamespace(font=lambda w: w),
    FontWeight=types.SimpleNamespace(MEDIUM="Inter-Medium.ttf")),
  "openpilot.system.ui.lib.text_measure": _mod("openpilot.system.ui.lib.text_measure",
                                               measure_text_cached=_measure),
  "openpilot.system.ui.widgets": _mod("openpilot.system.ui.widgets", Widget=_StubWidget),
  **_PYRAY,
}


@contextlib.contextmanager
def _stubbed():
  saved = {k: sys.modules.get(k, _MISSING) for k in _STUBS}
  sys.modules.update(_STUBS)
  try:
    yield
  finally:
    for k, v in saved.items():
      if v is _MISSING:
        sys.modules.pop(k, None)
      else:
        sys.modules[k] = v


_spec = importlib.util.spec_from_file_location("everdrive_status_under_test", _SRC)
ed = importlib.util.module_from_spec(_spec)
with _stubbed():
  _spec.loader.exec_module(ed)


# ---------------------------------------------------------------- fixtures / data

# The MEASURED payload, as the producer publishes it (see ENERGY-RANGE-SIGNALS.md):
#   rangeKm 180.2 is the driver-confirmed dash reading, 112 mi
#   effWhKm 320.0 is VehElEffAvg_No_Dsply, raw 42, constant for the whole 13-minute drive
BASE = {"ts": 0.0, "acKw": 1.4, "acSeen": True, "rangeKm": 180.2, "effWhKm": 320.0,
        "effOk": True, "socPct": 49.99, "vMs": 0.0, "capKwh": 127.0}
# everdrive2pnw 2026-09-20: 49.99 % of the derived 127.0 kWh -> the "(63.49kwh)" every form carries.
PACK = "(63.49kwh)"
MPH = 1.0 / 2.23694


@pytest.fixture
def now(monkeypatch):
  """One fake wall clock for the widget's staleness check (`ts` is a wall-clock heartbeat)."""
  t = [1_000_000.0]
  monkeypatch.setattr(ed, "time", types.SimpleNamespace(time=lambda: t[0], monotonic=lambda: t[0]))
  return t


def st(now, **over):
  return {**BASE, "ts": now[0], **over}


@pytest.fixture
def widget(now):
  MEM.payload = None
  MEM.raises = None
  MEM.reads = 0
  UI.params.disabled = False
  UI.params.raises = None
  LOG.calls.clear()
  MEASURED.clear()
  return ed.EverDriveStatusRenderer()


def poll(w, payload, disabled=False):
  """One _update_state cycle with the poll interval forced to have elapsed."""
  MEM.payload = payload
  UI.params.disabled = disabled
  w._last_poll = -1e9
  w._update_state()
  return w._cached_layout


# ---------------------------------------------------------------- T8


class TestTheFiveDocumentedOutputs:
  """The exact strings the owner approved (compact shapes, 2026-09-19). Lowercase `kw`/`mi`, no
  space after the colon, comma separators, ASCII `->`."""

  def test_charging_and_moving_projects_the_range(self, widget, now):
    assert widget._build_text(st(now, vMs=62 * MPH)) == "1.4kw,112m(63.49kwh)->117m"

  def test_charging_and_stopped_shows_the_gain_rate(self, widget, now):
    assert widget._build_text(st(now, vMs=0.0)) == "1.4kw,112m(63.49kwh),+2.7m/h"

  def test_acSeen_false_is_not_charging_and_never_a_manufactured_zero(self, widget, now):
    """acSeen False means the EverDrive message has NOT been received, so acKw is a CANParser
    pre-fill. "0.0 kW" would be a fabricated reading presented as a measurement."""
    out = widget._build_text(st(now, acSeen=False, acKw=0.0, vMs=62 * MPH))
    assert out == "--,112m(63.49kwh)"
    # Assert the POWER FIELD specifically, not a bare substring: since 2026-09-20 the line carries a
    # "(NN.NNkwh)" energy term, so "kw" appears in every form and is no longer a proxy for "a power
    # reading was printed". What must never appear is a fabricated 0.0 kW power term.
    assert out.startswith("--,"), out
    assert "0.0kw" not in out and "kw," not in out, out
    # and the guard is on acSeen ALONE, not on the value: a payload claiming power while saying the
    # message was never received is not trustworthy at any magnitude
    assert widget._build_text(st(now, acSeen=False, acKw=1.4, vMs=62 * MPH)) == "--,112m(63.49kwh)"
    assert widget._build_text(st(now, acSeen=False, acKw=1.4, vMs=0.0)) == "--,112m(63.49kwh)"

  def test_a_real_measured_zero_reads_the_same_way_to_the_driver(self, widget, now):
    """Charger unplugged, meter live: a genuine 0 kW. Same form -- "not charging" is the truth in
    both cases -- but it must come from the acKw <= _AC_ZERO_KW branch, not from acSeen."""
    assert widget._build_text(st(now, acSeen=True, acKw=0.0, vMs=62 * MPH)) == "--,112m(63.49kwh)"
    assert widget._build_text(st(now, acSeen=True, acKw=ed._AC_ZERO_KW, vMs=62 * MPH)) == "--,112m(63.49kwh)"
    assert widget._build_text(st(now, acSeen=True, acKw=0.06, vMs=0.0)).startswith("0.1kw,")

  def test_effOk_false_drops_the_projection_and_substitutes_nothing(self, widget, now):
    """effWhKm at its -100 Wh/km encoding floor is "not available". A default efficiency would
    manufacture a confident projection out of a signal we do not have."""
    out = widget._build_text(st(now, effOk=False, effWhKm=-100.0, vMs=62 * MPH))
    assert out == "1.4kw,112m(63.49kwh)"
    assert "->" not in out and "mi/h" not in out
    # and the guard is on effOk ALONE: a payload that says "not usable" while carrying a
    # usable-looking number must still be believed about the flag, not about the number
    assert widget._build_text(st(now, effOk=False, effWhKm=320.0, vMs=62 * MPH)) == "1.4kw,112m(63.49kwh)"
    assert widget._build_text(st(now, effOk=False, effWhKm=320.0, vMs=0.0)) == "1.4kw,112m(63.49kwh)"

  def test_a_stale_ts_hides_the_box(self, widget, now):
    assert widget._build_text(st(now, ts=now[0] - 12.0, vMs=62 * MPH)) is None
    assert widget._build_text(st(now, ts=now[0] - (ed._STALE_S + 0.01))) is None
    assert widget._build_text(st(now, ts=now[0] - (ed._STALE_S - 0.01))) is not None   # boundary control

  def test_every_emitted_character_exists_in_the_device_font_atlas(self, widget, now):
    """process.py builds the atlas from chr(32..126). A unicode arrow or a non-breaking space would
    render as tofu on the car and nowhere else."""
    outs = [widget._build_text(s) for s in (
      st(now, vMs=62 * MPH), st(now, vMs=0.0), st(now, acSeen=False),
      st(now, effOk=False), st(now, rangeKm=0.0), st(now, acKw=99.9, vMs=99 * MPH))]
    for out in outs:
      if out is None:
        continue
      assert all(32 <= ord(c) <= 126 for c in out), repr(out)


  def test_no_plausible_line_reaches_the_width_ceiling(self, widget, now):
    """REPLACED the exemplar-coverage test 2026-09-20, when the box started hugging the text. The
    risk is no longer clipping (the box grows to fit) but WIDTH: past _MAX_BOX_W the box begins
    reaching toward the green driving path. Checked in characters -- the stub measurement is not the
    device font -- across the whole PLAUSIBLE domain: a 9.6 kW L2 charger, the DBC's maximum range,
    and both ends of the efficiency band. The real px ceiling is pinned separately."""
    widest = 35     # chars: the longest shape the formatter can emit, "00.0kw,000m(000.00kwh),+00.0m/h"
    cases = [st(now, **o) for o in (
      {"vMs": 62 * MPH}, {"vMs": 0.0}, {"acSeen": False}, {"effOk": False},
      {"acKw": 9.6, "rangeKm": 409.3, "vMs": 0.0},
      {"acKw": 9.6, "rangeKm": 409.3, "vMs": 99 * MPH},
      {"acKw": 9.6, "rangeKm": 409.3, "effWhKm": 150.0, "vMs": 99 * MPH},
      {"acKw": 9.6, "rangeKm": 409.3, "effWhKm": 1159.9, "vMs": 99 * MPH},
      {"rangeKm": 0.0}, {"rangeKm": None}, {"acKw": 0.0})]
    for s in cases:
      out = widget._build_text(s)
      assert out is not None
      assert len(out) <= widest, f"{out!r} ({len(out)}) is wider than any exemplar ({widest})"

  def test_an_implausible_acKw_drops_the_gain_term_instead_of_clipping(self, widget, now):
    """GAP CLOSED 2026-09-19. This test was originally `test_KNOWN_GAP_...` and asserted the BROKEN
    behaviour (`ED:99.9kw,112mi,+194.0mi/h`, wider than the box) so the gap stayed visible. It is now
    inverted to pin the fix.

    Two independent guards close it, and this one is the second line of defence:
      * the producer bands 0x2A7 to 0..100 A / 0..277 V, so acKw can no longer exceed ~27.7 kW
        (gain <= ~53.8 mi/h, two integer digits, which is what the exemplar reserves);
      * the UI does not TRUST that, because acKw arrives over a mem-param from an aftermarket
        module -- so _GAIN_MAX_MI_H drops the term rather than printing it clipped.
    Showing LESS is honest; a clipped number silently corrupts the whole line, not just its last
    term (Rule 2). The change of form is the visible signal, exactly as it is for the projection."""
    out = widget._build_text(st(now, acKw=99.9, vMs=0.0))
    assert out == "99.9kw,112m(63.49kwh)", "the un-showable gain term must be dropped, not clipped"
    assert len(out) <= 35, "must stay within the longest shape the formatter can emit"


# ---------------------------------------------------------------- T9


class TestGuards:
  def test_a_missing_key_hides_the_box_and_logs_NOTHING(self, widget):
    """Absence is the normal steady state on the Tesla and on a Lightning with no module fitted. An
    alarm there would fire on every drive of one of the two cars, i.e. it would be the bug."""
    assert poll(widget, None) is None
    assert widget.stack_height == 0.0, "the CES box must render exactly where it does today"
    assert LOG.calls == [], f"absence must be silent, got {LOG.calls}"
    assert MEASURED == [], "no text measurement may happen while the box is hidden"

  def test_a_missing_key_backs_the_poll_off_but_never_permanently(self, widget, now):
    assert poll(widget, None) is None
    assert widget._poll_interval == ed._ABSENT_REFRESH_S
    assert widget._cached_layout is None, "no layout work at all while hidden"
    # ... and the charger can still be plugged in mid-drive
    assert poll(widget, st(now, vMs=62 * MPH)) is not None
    assert widget._poll_interval == ed._REFRESH_S
    assert widget._cached_layout is not None and widget.stack_height > 0.0
    # and it goes back to hidden + slow when the module is unplugged again
    assert poll(widget, None) is None
    assert widget._poll_interval == ed._ABSENT_REFRESH_S
    assert widget.stack_height == 0.0

  def test_every_numeric_field_explicitly_None_does_not_crash(self, widget, now):
    """A JSON null sails straight through dict.get(k, default); float(None) raises, and an exception
    in the UI render loop is this fork's documented brick scenario."""
    s = {"ts": now[0], "acKw": None, "acSeen": True, "rangeKm": None, "effWhKm": None,
         "effOk": True, "socPct": None, "vMs": None}
    out = widget._build_text(s)
    assert out == "--,--m", out
    assert "None" not in out and "nan" not in out

  def test_a_non_dict_payload_is_treated_as_absent(self, widget):
    for junk in ("not a dict", 42, [1, 2, 3], b"bytes"):
      assert poll(widget, junk) is None
    assert LOG.calls == []

  def test_a_missing_ts_key_hides_rather_than_showing_dead_numbers(self, widget, now):
    s = st(now)
    del s["ts"]
    assert widget._build_text(s) is None

  def test_range_zero_never_prints_zero_miles(self, widget, now):
    """rangeKm 0 is what a never-received VehElRnge would decode to. "0 mi" of range is a lie the
    driver would act on."""
    for out in (widget._build_text(st(now, rangeKm=0.0, vMs=62 * MPH)),
                widget._build_text(st(now, rangeKm=None, vMs=62 * MPH)),
                widget._build_text(st(now, rangeKm=0.0, acSeen=False))):
      # ",--m" pins the RANGE FIELD exactly. A bare "0m" check is no longer usable: since the kWh
      # term landed, an honest range like "110m" contains "0m" and would false-positive. If range
      # were ever printed as zero it would read ",0m" here, which this assertion excludes.
      assert ",--m" in out, out

  def test_crawling_uses_the_stopped_form_not_a_diverged_projection(self, widget, now):
    """rangeMi * P/(P - acKw) diverges as P approaches acKw. The cutoff is DERIVED from
    _PROJ_HEADROOM, so there is no separate magic mph threshold to drift."""
    for mph in (0.0, 0.5, 1.0, 2.0, 3.0):
      out = widget._build_text(st(now, vMs=mph * MPH))
      assert "->" not in out, f"{mph} mph projected: {out}"
      assert out == "1.4kw,112m(63.49kwh),+2.7m/h"

  def test_the_projection_switches_on_only_once_it_is_credible(self, widget, now):
    """Positive control for the test above: above the cutoff the projection DOES appear, and it
    tightens toward the plain range as speed rises.

    NOTE the cutoff is set by _PROJ_MAX_RATIO (2.0), not by _PROJ_HEADROOM (1.05): proj <= 2*range
    requires P_assumed >= 2*acKw, which is the stricter of the two. Both land on the stopped form,
    so the behaviour is right -- but the _PROJ_HEADROOM comment's "the low-speed cutoff is DERIVED
    from this guard" describes the weaker guard."""
    kwh_per_mi = 320.0 * ed._KM_PER_MI / 1000.0
    v_mph = ed._PROJ_MAX_RATIO * 1.4 / kwh_per_mi            # the speed the clamp actually requires
    assert v_mph > 1.4 * ed._PROJ_HEADROOM / kwh_per_mi      # ... and it is the binding one
    assert "->" not in widget._build_text(st(now, vMs=(v_mph - 0.2) * MPH))
    assert "->" in widget._build_text(st(now, vMs=(v_mph + 0.2) * MPH))
    projected = [int(widget._build_text(st(now, vMs=m * MPH)).split("->")[1].rstrip("mi"))
                 for m in (20, 40, 62, 80)]
    assert projected == sorted(projected, reverse=True) and projected[-1] >= 112

  def test_an_absurd_projection_falls_back_to_the_stopped_form(self, widget, now):
    """_PROJ_MAX_RATIO: a glitched efficiency can squeeze the denominator toward zero while still
    clearing _PROJ_HEADROOM. The change of FORM is the visible failure signal; a 4-digit projection
    would also overflow the fixed-width box.

    UPDATED 2026-09-19 (_GAIN_MAX_MI_H): an efficiency this glitched makes BOTH derived terms
    nonsense, not just the projection -- 3 Wh/km implies a gain of ~290 mi/h. So the fallback now
    goes one step further and drops the gain rate too, leaving ONLY the two measured quantities
    (charger power and the truck's own range). "When the efficiency is untrustworthy, show only what
    was measured" is the stronger and more honest property, so it is what this now asserts.
    The sane-efficiency low-speed case still shows the gain -- see the stopped-form test in T8."""
    out = widget._build_text(st(now, effWhKm=3.0, vMs=60 * MPH))
    assert "->" not in out, "an absurd projection must not print"
    assert not out.endswith("mi/h"), "a gain derived from an absurd efficiency must not print either"
    assert out == "1.4kw,112m(63.49kwh)"

  def test_the_disable_toggle_hides_the_box_and_backs_the_poll_off(self, widget, now):
    assert poll(widget, st(now, vMs=62 * MPH), disabled=True) is None
    assert widget._poll_interval == ed._ABSENT_REFRESH_S
    assert widget.stack_height == 0.0
    assert poll(widget, st(now, vMs=62 * MPH), disabled=False) is not None

  def test_an_unreadable_toggle_defaults_to_SHOWING_and_says_so_once(self, widget, now):
    """A params_keys.h / params_pyx.so mismatch raises UnknownKeyName. Defaulting to hidden would be
    a feature that quietly does nothing."""
    UI.params.raises = RuntimeError("UnknownKeyName: DisableEverDrive")
    for _ in range(5):
      assert poll(widget, st(now, vMs=62 * MPH)) is not None
    UI.params.raises = None
    assert [c for c in LOG.calls if c[0] == "exception"], "a real fault must be logged"
    assert len(LOG.calls) == 1, f"once, not once per poll: {LOG.calls}"

  def test_an_unreadable_status_key_hides_the_box_and_says_so_once(self, widget, now):
    """Distinct from an ABSENT key (which returns None through Params and must stay silent)."""
    MEM.raises = RuntimeError("UnknownKeyName: EverDriveStatus")
    for _ in range(5):
      assert poll(widget, None) is None
    assert len(LOG.calls) == 1 and LOG.calls[0][0] == "exception"

  def test_the_two_read_faults_are_logged_independently(self, widget, now):
    """One shared "already logged" flag would let the second, different failure go unreported."""
    UI.params.raises = RuntimeError("toggle")
    poll(widget, st(now))
    MEM.raises = RuntimeError("status")
    poll(widget, None)
    assert len(LOG.calls) == 2, LOG.calls

  def test_the_box_hugs_the_text_and_the_text_right_edge_never_moves(self, widget, now):
    """REPLACED the exemplar-width test 2026-09-20. The box used to be sized from fixed worst-case
    exemplars so it could not dance; the driver reported that this left 4-5 characters of empty black
    beside the line, so the box now hugs the ACTUAL text.

    That is safe for the reason the exemplars existed: the box is right-anchored and the text is
    right-aligned inside it, so shrinking the box moves only the BACKGROUND's left edge -- the text's
    right edge is invariant. This test pins both halves."""
    seen = []
    for mph, kw in ((62.0, 1.4), (0.0, 1.4), (62.0, 9.6), (35.0, 0.0)):
      text, box_w, text_w = poll(widget, st(now, vMs=mph * MPH, acKw=kw))
      assert box_w == pytest.approx(text_w + 2 * ed._PAD), "the box must hug the text"
      seen.append((box_w, text_w))
    # right edge of the text = (right margin) + _PAD, independent of box_w -- so it cannot move
    right_edges = {round(bw - tw - ed._PAD, 6) for bw, tw in seen}
    assert len(right_edges) == 1, f"the text's right inset must be constant, got {right_edges}"
    assert len({bw for bw, _ in seen}) > 1, "positive control: the box SHOULD resize with the form"

  def test_a_line_wider_than_the_ceiling_is_reported_not_silently_drawn(self, widget, now):
    """Rule 2 tripwire. _MAX_BOX_W is the width at which the box would start reaching toward the
    green driving path. The producer's bands make it unreachable, so tripping it means an input is
    out of band -- it must be logged, and NOT clamped (clamping would hide the fault)."""
    assert ed._MAX_BOX_W == 1000.0
    widget._too_wide_logged = False
    LOG.calls.clear()
    text, box_w, _ = poll(widget, st(now, vMs=62 * MPH))
    assert box_w <= ed._MAX_BOX_W, "this realistic line must NOT trip the ceiling"
    assert not LOG.calls, "and must not log"

    # Now FORCE an over-wide line. No plausible payload can reach the ceiling (the formatter tops out
    # around 32 chars and the stub measures len*fs*0.5), so the only way to exercise the tripwire is
    # to hand _update_state a pathological string directly. Without this, swapping the report for a
    # silent clamp survives the whole suite -- mutation W3 did exactly that.
    widget._too_wide_logged = False
    LOG.calls.clear()
    widget._build_text = lambda _st: "X" * 60
    widget._last_poll = -1e9
    widget._update_state()
    _t, wide_box_w, wide_text_w = widget._cached_layout
    assert wide_box_w > ed._MAX_BOX_W, "the box must NOT be clamped -- clamping hides the fault"
    assert wide_box_w == pytest.approx(wide_text_w + 2 * ed._PAD), "still hugging, not clamped"
    assert [c for c in LOG.calls if "exceeds" in str(c)], "Rule 2: an over-wide line must be LOGGED"

    n = len(LOG.calls)
    widget._last_poll = -1e9
    widget._update_state()
    assert len(LOG.calls) == n, "and logged at most once per session, not every poll"

  def test_the_stack_height_is_exactly_zero_while_hidden(self, widget, now):
    """augmented_road_view subtracts this from the CES box position every frame. Anything other
    than 0.0 while hidden moves an in-use overlay for a box that is not on screen."""
    assert poll(widget, None) is None and widget.stack_height == 0.0
    assert poll(widget, st(now, vMs=62 * MPH)) is not None
    assert widget.stack_height == ed._BOX_H + ed._STACK_GAP
    assert poll(widget, st(now, ts=now[0] - 12.0)) is None and widget.stack_height == 0.0


class TestThePackEnergyTerm:
  """everdrive2pnw (driver req 2026-09-20): "(NN.NNkwh)" beside the range it buys.

  The value is socPct x capKwh, and capKwh is DERIVED by the producer from the truck's own
  RngPerChrgAvg x VehElEffAvg -- nothing is hardcoded here. Both inputs must be real."""

  def test_the_term_is_socPct_times_capKwh(self, widget, now):
    out = widget._build_text(st(now, socPct=49.99, capKwh=127.0, vMs=62 * MPH))
    assert "(63.49kwh)" in out, out

  def test_it_tracks_both_inputs(self, widget, now):
    """A positive control: if it were a constant, these would not move."""
    assert "(25.40kwh)" in widget._build_text(st(now, socPct=20.0, capKwh=127.0, vMs=62 * MPH))
    assert "(65.50kwh)" in widget._build_text(st(now, socPct=50.0, capKwh=131.0, vMs=62 * MPH))

  @pytest.mark.parametrize("missing", ["capKwh", "socPct"])
  def test_the_term_is_OMITTED_when_an_input_is_missing_never_zero(self, widget, now, missing):
    """Rule 2. "(0.00kwh)" would read as an empty pack to a driver -- the most alarming possible
    lie this box could tell, and it would be pure fabrication from a missing signal."""
    out = widget._build_text(st(now, **{missing: None}, vMs=62 * MPH))
    assert "kwh" not in out, out
    assert "0.00" not in out, out
    assert out == "1.4kw,112m->117m", out      # the rest of the line is unaffected

  def test_the_term_appears_in_every_form_that_shows_a_range(self, widget, now):
    """Consistency: the driver should not have to wonder why it vanished at a stoplight."""
    assert "(63.49kwh)" in widget._build_text(st(now, vMs=62 * MPH))          # moving
    assert "(63.49kwh)" in widget._build_text(st(now, vMs=0.0))               # stopped
    assert "(63.49kwh)" in widget._build_text(st(now, acSeen=False))          # not charging
    assert "(63.49kwh)" in widget._build_text(st(now, effOk=False))           # no efficiency

  def test_miles_are_abbreviated_m_not_mi(self, widget, now):
    """Driver req 2026-09-20 -- "mi" -> "m" is what bought the width for the kWh term at _FS 48."""
    for out in (widget._build_text(st(now, vMs=62 * MPH)), widget._build_text(st(now, vMs=0.0))):
      assert "mi" not in out, out


class TestTheComputedRangeAtCurrentSpeed:
  """everdrive2pnw (2026-09-20) introduced a measured "@" range from `grossKw`. everdriveinstant2pnw
  (2026-10-31) moved every measured range to the MOVING form (TestTheMovingForm), so these now pin
  that a stopped payload IGNORES grossKw and keeps the truck's own range, exactly as it shipped."""

  # 59.35 kWh at 20 kW and 62 mph -> 183.985 mi, and the projection at 1.4 kW in -> 197.8 mi
  MEAS = {"energyKwh": 59.35, "grossKw": 20.0, "vMs": 62 * MPH}

  @pytest.mark.parametrize("over", [
    {"grossKw": None},                       # stopped / below 10 mph / window not filled
    {"energyKwh": None},                     # socPct or capKwh missing upstream
    {"grossKw": 0.0},                        # would be a division by zero
    {"grossKw": -5.0},                       # would be a NEGATIVE range
    {"energyKwh": 0.0},                      # an empty pack is not a range of zero miles
    {},                                      # since 2026-10-31 a GOOD grossKw is ignored too
  ])
  def test_every_unusable_input_falls_back_to_the_trucks_own_range(self, widget, now, over):
    out = widget._build_text(st(now, **{**self.MEAS, **over}))
    assert out == "1.4kw,112m(63.49kwh)->117m", out
    assert "@" not in out, "a fallback must not be dressed up as a measurement"

  def test_a_stationary_truck_never_prints_a_computed_zero(self, widget, now):
    """Kills mutation U8 (`0.0 < r` -> `0.0 <= r`). At 0 mph the computed range is exactly 0, and
    "@0m" would tell the driver he has no range while the truck sits with a half-full pack. The
    producer already withholds grossKw below ~10 mph, but this box does not get to assume that --
    acKw arrives over a mem-param from an aftermarket module and the whole payload is untrusted."""
    out = widget._build_text(st(now, energyKwh=59.35, grossKw=20.0, vMs=0.0))
    assert "@" not in out and "@0" not in out, out
    assert out == "1.4kw,112m(63.49kwh),+2.7m/h", out

  def test_a_missing_grossKw_key_entirely_is_the_fallback_not_a_crash(self, widget, now):
    """An older producer, or the very first payload of a session, simply has no such key."""
    s = st(now, vMs=62 * MPH)
    assert "grossKw" not in s and "energyKwh" not in s
    assert widget._build_text(s) == "1.4kw,112m(63.49kwh)->117m"

  def test_the_widest_line_this_can_emit_stays_inside_the_ceiling(self, widget, now):
    """Character-count guard (the stub measurement is not the device font; the real px worst case is
    pinned in the _FS comment, measured against the device's own Inter-Medium.fnt). 31 chars is
    '44.4kw,444m(444.44kwh),+44.4m/h' -- every range term is bounded to 3 digits by _RANGE_MAX_MI
    (499, so the projection is <= 998) and the truck's own 254 mi band."""
    widest = 31
    cases = [
      st(now, **self.MEAS),
      st(now, **self.MEAS, acKw=27.7),
      st(now, energyKwh=474.63, grossKw=59.0, vMs=62 * MPH, acKw=27.7, socPct=100.0, capKwh=474.63),
      st(now, energyKwh=59.35, grossKw=15.0, vMs=62 * MPH, acKw=9.6, socPct=100.0, capKwh=474.63),
      st(now, energyKwh=0.5, grossKw=277.7, vMs=10 * MPH, acKw=27.7, socPct=100.0, capKwh=474.63),
      st(now, **self.MEAS, acSeen=False),
    ]
    for s in cases:
      out = widget._build_text(s)
      assert out is not None
      assert len(out) <= widest, f"{out!r} ({len(out)}) is wider than the bounded worst case"
      assert all(32 <= ord(c) <= 126 for c in out), repr(out)   # '@' is 64, in the device atlas



class TestTheMovingForm:
  """everdriveinstant2pnw (owner 2026-10-31): while the producer says `moving`, the line is built from
  the INSTANT pack load pNowKw and the last-mile energy -- no window wait, no projection."""

  # 59.35 kWh at 20 kW net and 62 mph -> 184 mi; last mile 0.5 kWh -> 118.7 mi
  # mileEst False: the mile fields in these tests are a fully measured mile unless a test says not.
  MOV = {"moving": True, "vMs": 62 * MPH, "energyKwh": 59.35, "pNowKw": 20.0, "acKw": 0.0, "mileEst": False}

  def test_the_first_moving_payload_shows_the_current_load_range_at_once(self, widget, now):
    """THE owner requirement: no wait. No grossKw, no mile yet -- the number is there anyway."""
    assert widget._build_text(st(now, **self.MOV)) == "184m(63.49kwh),1mi:184m"

  def test_before_the_first_mile_the_last_mile_term_repeats_the_instant_one(self, widget, now):
    """Owner 2026-11-01: the shape never changes while driving."""
    assert widget._build_text(st(now, **self.MOV, mileKwh=None, mileAcKwh=None)) == "184m(63.49kwh),1mi:184m"
    assert widget._build_text(st(now, **self.MOV, mileKwh=0.5, mileAcKwh=0.0)) == "184m(63.49kwh),1mi:119m"

  def test_the_terms_track_their_inputs(self, widget, now):
    """Positive control: constants would not move."""
    out = widget._build_text(st(now, **{**self.MOV, "pNowKw": 40.0, "vMs": 31 * MPH}, mileKwh=1.0, mileAcKwh=0.0))
    assert out == "46m(63.49kwh),1mi:59m", out

  def test_generator_on_puts_the_kw_first_nets_the_charge_and_drops_the_kwh(self, widget, now):
    out = widget._build_text(st(now, **{**self.MOV, "acKw": 1.4, "pNowKw": 21.4}, mileKwh=0.5, mileAcKwh=0.0233))
    assert out == "1.4kw,120m,1mi:125m", out          # since 2026-11-01: dash 112 x 21.4 / 20.0
    assert "kwh" not in out, "the pack kWh is never shown while charging and moving"

  def test_acSeen_false_is_generator_off_whatever_acKw_says(self, widget, now):
    assert widget._build_text(st(now, **{**self.MOV, "acKw": 1.4}, acSeen=False)) == "184m(63.49kwh),1mi:184m"

  @pytest.mark.parametrize("over", [
    {"pNowKw": -5.0},                    # regen
    {"pNowKw": 0.0},                     # zero load
    {"pNowKw": 1.0},                     # coasting: 3680 mi, over the ceiling
    {"energyKwh": None},                 # no pack energy
  ])
  def test_an_unusable_current_load_falls_back_to_the_trucks_range(self, widget, now, over):
    out = widget._build_text(st(now, **{**self.MOV, **over}))
    assert out == "112m(63.49kwh),1mi:112m", out
    out = widget._build_text(st(now, **{**self.MOV, **over}, mileKwh=0.5, mileAcKwh=0.0))
    # with no pack energy the last mile has nothing to divide either
    assert out == ("112m(63.49kwh),1mi:--" if "energyKwh" in over else "112m(63.49kwh),1mi:119m"), out

  def test_net_charging_while_moving_has_no_finite_range(self, widget, now):
    """9.6 kW in, 5 kW load: the pack is gaining. No "m/h" while moving (owner 2026-11-01)."""
    assert widget._build_text(st(now, **{**self.MOV, "acKw": 9.6, "pNowKw": 5.0})) == "9.6kw,>999m,1mi:>999m"

  @pytest.mark.parametrize("mile,ac", [(-0.1, 0.0), (0.0, 0.0), (0.1, 0.0)])   # net charging: TestGrowthWhileMoving
  def test_an_unusable_last_mile_is_shown_as_dashes_not_a_number(self, widget, now, mile, ac):
    """Net regen / net charging over the mile, or 593 mi over the ceiling: '1mi:--', never a bent number."""
    out = widget._build_text(st(now, **self.MOV, mileKwh=mile, mileAcKwh=ac))
    assert out == "184m(63.49kwh),1mi:--", out

  def test_no_truck_range_falls_back_to_dashes_never_zero(self, widget, now):
    # No 0x441 and no efficiency: no estimate either, and "?" is never put on "--m".
    out = widget._build_text(st(now, **{**self.MOV, "pNowKw": None}, rangeKm=None, effOk=False))
    assert out == "--m(63.49kwh),1mi:--m", out

  def test_stopped_or_parked_is_exactly_the_old_form_and_ignores_the_instant_terms(self, widget, now):
    """moving False (or absent): the kWh is shown and the instant terms play no part."""
    for moving in (False, None):
      s = st(now, **{**self.MOV, "moving": moving, "acKw": 1.4, "vMs": 0.0}, mileKwh=0.5, mileAcKwh=0.0)
      assert widget._build_text(s) == "1.4kw,112m(63.49kwh),+2.7m/h"
      s = st(now, **{**self.MOV, "moving": moving, "vMs": 0.0})
      assert widget._build_text(s) == "--,112m(63.49kwh)"

  def test_the_stopped_form_ignores_grossKw(self, widget, now):
    """The measured '@' path was removed from the stopped form; grossKw is calibration-only now."""
    assert widget._build_text(st(now, energyKwh=59.35, grossKw=20.0, vMs=62 * MPH)) == "1.4kw,112m(63.49kwh)->117m"

  def test_moving_forms_stay_inside_the_stopped_worst_case(self, widget, now):
    """All-max moving payloads. The real-font table at _FS has the px numbers; this is the char guard."""
    cases = [
      st(now, **{**self.MOV, "acKw": 27.7, "pNowKw": 27.8, "energyKwh": 474.63}, mileKwh=0.96, mileAcKwh=0.0),
      st(now, **{**self.MOV, "pNowKw": 59.0, "energyKwh": 474.63}, mileKwh=0.96, mileAcKwh=0.0,
         socPct=100.0, capKwh=474.63),
      st(now, **{**self.MOV, "pNowKw": -1.0}, mileKwh=-1.0, mileAcKwh=0.0, socPct=100.0, capKwh=474.63),
    ]
    for s in cases:
      out = widget._build_text(s)
      assert len(out) <= 31, out
      assert all(32 <= ord(c) <= 126 for c in out), repr(out)


class TestReviewFixes:
  """Gemini 4 Argon review of a9e7ec1e (2026-11-01)."""

  MOV = TestTheMovingForm.MOV

  def test_a_computed_range_under_half_a_mile_never_prints_zero(self, widget, now):
    """0.4 kWh at 40 kW and 20 mph is 0.2 mi: '@0m' is a zero range. Fall back instead."""
    out = widget._build_text(st(now, **{**self.MOV, "energyKwh": 0.4, "pNowKw": 40.0, "vMs": 20 * MPH},
                                mileKwh=1.0, mileAcKwh=0.0))
    assert "@0m" not in out and "1mi:0m" not in out, out
    assert out.startswith("112m(") and out.endswith(",1mi:--"), out
    # positive control at the boundary: 0.6 mi still prints, as "1"
    assert "1m" in widget._build_text(st(now, **{**self.MOV, "energyKwh": 1.2, "pNowKw": 40.0, "vMs": 20 * MPH}))

  def test_mileAcKwh_is_ignored_without_acSeen_but_used_after_the_charger_stops(self, widget, now):
    s = st(now, **self.MOV, mileKwh=0.5, mileAcKwh=0.1)
    assert widget._build_text({**s, "acSeen": False}).endswith(",1mi:119m"), "no acSeen: AC energy not trusted"
    assert widget._build_text(s).endswith(",1mi:148m"), "acSeen, charger now off: the mile's AC energy counts"

  @pytest.mark.parametrize("moving", [True, False])
  def test_a_truck_range_under_half_a_mile_is_dashes_not_zero(self, widget, now, moving):
    out = widget._build_text(st(now, **{**self.MOV, "moving": moving, "pNowKw": None, "vMs": 0.0},
                                rangeKm=0.5, acSeen=False))
    assert "--m" in out and ",0m" not in out and not out.startswith("0m"), out


class TestRoundThenBound:
  """Gemini BLOCK on 7ee73b47 (2026-11-01): every printed number is bounded AFTER rounding."""

  MOV = TestTheMovingForm.MOV
  MI_PER_KWH = 1000.0 / (320.0 * 1.60934)

  def test_the_gain_text_rounds_first_then_bounds(self):
    """The BLOCK cases, on the shared helper (the moving form no longer prints a gain at all)."""
    assert ed._gain_text(-0.0) is None and ed._gain_text(0.0) is None, "never '+-0.0' / '+0.0'"
    assert ed._gain_text(0.04) is None and ed._gain_text(0.05) is not None
    assert ed._gain_text(99.96) is None, "rounds to 100.0: three integer digits"
    assert ed._gain_text(99.94) == "+99.9m/h"

  def test_the_stopped_gain_has_the_same_bound(self, widget, now):
    ac = 99.96 / self.MI_PER_KWH
    out = widget._build_text(st(now, acKw=ac, vMs=0.0))
    assert "m/h" not in out, out

  def test_a_range_of_exactly_half_a_mile_never_prints_zero(self, widget, now):
    # r = e / p * mph = 0.5 exactly: format(0.5, ".0f") is "0" (round half to even)
    out = widget._build_text(st(now, **{**self.MOV, "energyKwh": 0.5, "pNowKw": 62.0}))
    assert "@0m" not in out and out.startswith("112m"), out
    assert widget._build_text(st(now, rangeKm=0.5 * 1.60934, vMs=0.0, acSeen=False)).startswith("--,--m")

  @pytest.mark.parametrize("field", ["pNowKw", "energyKwh", "acKw", "mileKwh", "rangeKm", "vMs"])
  def test_an_infinite_or_nan_field_never_crashes(self, widget, now, field):
    for bad in (float("inf"), float("-inf"), float("nan")):
      out = widget._build_text(st(now, **{**self.MOV, "acKw": 1.4, "mileKwh": 0.5, "mileAcKwh": 0.0, field: bad}))
      assert out is not None and "inf" not in out and "nan" not in out, out

  def test_a_computed_overflow_never_crashes(self, widget, now):
    """Finite inputs can still overflow: 59 kWh / 1e-320 kW is inf, and round(inf) raises."""
    out = widget._build_text(st(now, **{**self.MOV, "pNowKw": 1e-320, "mileKwh": 1e-320, "mileAcKwh": 0.0}))
    assert out == "112m(63.49kwh),1mi:--", out


class TestPresenceIsAReading:
  """Review of 63557d96 (2026-11-01): a present but non-finite or non-numeric value is no reading."""

  MOV = TestTheMovingForm.MOV

  @pytest.mark.parametrize("bad", [float("nan"), float("inf"), "bad"])
  def test_a_bad_capacity_or_soc_omits_the_kwh_never_zero(self, widget, now, bad):
    for field in ("capKwh", "socPct"):
      out = widget._build_text(st(now, vMs=0.0, **{field: bad}))
      assert "kwh" not in out and "0.00" not in out, out

  @pytest.mark.parametrize("bad", [float("nan"), float("inf"), "bad"])
  def test_a_bad_pNowKw_is_no_motor_signal_not_zero_load(self, widget, now, bad):
    # no reading -> the speed estimate, marked "?": 112 x 33.13 / 23.13 = 160
    assert widget._build_text(st(now, **{**self.MOV, "acKw": 10.0, "pNowKw": bad})) == \
      "10.0kw,?160m,1mi:?160m"

  @pytest.mark.parametrize("bad", [float("nan"), float("inf"), "bad"])
  def test_a_bad_mileKwh_is_no_mile_yet(self, widget, now, bad):
    out = widget._build_text(st(now, **self.MOV, mileKwh=bad, mileAcKwh=0.0))
    assert out == "184m(63.49kwh),1mi:184m", out          # no mile yet: the instant term


class TestGeneratorOnAlwaysThreeTerms:
  """Owner 2026-11-01: moving with the generator on is ALWAYS "<kw>kw,<R1>,1mi:<R2>"."""

  MOV = TestTheMovingForm.MOV
  DASH = 180.2 / 1.60934                                       # BASE rangeKm, 111.97 mi

  def _on(self, widget, now, **over):
    return widget._build_text(st(now, **{**self.MOV, "acKw": 10.8, **over}))

  def test_the_owners_example_141_mi_30_kw_10_8_kw_is_220(self, widget, now):
    assert self._on(widget, now, pNowKw=30.0, rangeKm=141.0 * 1.60934) == "10.8kw,220m,1mi:220m"
    assert round(141.0 * 30.0 / 19.2) == 220

  def test_R1_needs_no_capacity(self, widget, now):
    out = self._on(widget, now, pNowKw=30.0, energyKwh=None, capKwh=None, socPct=None, rangeKm=141.0 * 1.60934)
    assert out == "10.8kw,220m,1mi:220m", out

  @pytest.mark.parametrize("p_now", [10.8, 5.0, -20.0])
  def test_the_generator_covering_the_load_is_over_999(self, widget, now, p_now):
    assert self._on(widget, now, pNowKw=p_now) == "10.8kw,>999m,1mi:>999m"

  def test_no_motor_signal_is_the_speed_estimate_marked(self, widget, now):
    assert self._on(widget, now, pNowKw=None) == "10.8kw,?166m,1mi:?166m"     # 112 x 33.13 / 22.33

  def test_before_the_first_mile_R2_repeats_R1(self, widget, now):
    assert self._on(widget, now, pNowKw=15.0) == "10.8kw,400m,1mi:400m"

  def test_a_mile_that_netted_charging_is_over_999(self, widget, now):
    assert self._on(widget, now, pNowKw=30.0, mileKwh=0.10, mileAcKwh=0.135) == "10.8kw,175m,1mi:>999m"

  def test_a_computable_mile_is_its_own_range(self, widget, now):
    # 59.35 kWh / (0.5 - 0.0233) kWh per mile = 124.5
    assert self._on(widget, now, pNowKw=30.0, mileKwh=0.5, mileAcKwh=0.0233) == "10.8kw,175m,1mi:125m"

  def test_a_mile_that_cannot_be_computed_repeats_R1(self, widget, now):
    assert self._on(widget, now, pNowKw=30.0, mileKwh=0.5, mileAcKwh=0.0, energyKwh=None) == "10.8kw,175m,1mi:175m"

  def test_a_mile_range_under_half_a_mile_repeats_R1_never_zero(self, widget, now):
    # 0.2 kWh / 0.5 kWh per mile = 0.4 mi -> rounds to 0, not a range
    assert self._on(widget, now, pNowKw=30.0, mileKwh=0.5, mileAcKwh=0.0, energyKwh=0.2) == "10.8kw,175m,1mi:175m"

  def test_over_999_on_either_term(self, widget, now):
    assert self._on(widget, now, pNowKw=11.0) == "10.8kw,>999m,1mi:>999m"       # 112 x 11 / 0.2
    assert self._on(widget, now, pNowKw=30.0, mileKwh=0.05, mileAcKwh=0.0) == "10.8kw,175m,1mi:>999m"

  def test_the_999_bound_is_on_the_rounded_value(self, widget, now):
    for target, want in ((999.4, "999m"), (999.6, ">999m")):
      p = 10.8 * target / (target - self.DASH)
      assert self._on(widget, now, pNowKw=p).startswith(f"10.8kw,{want},"), target

  def test_no_dash_range_is_the_only_dashes(self, widget, now):
    assert self._on(widget, now, pNowKw=30.0, rangeKm=None) == "10.8kw,--m,1mi:--m"
    assert self._on(widget, now, pNowKw=30.0, rangeKm=None, mileKwh=0.5, mileAcKwh=0.0) == "10.8kw,--m,1mi:119m"

  def test_the_form_is_always_three_terms(self, widget, now):
    import re
    shape = re.compile(r"^\d+\.\dkw,\??(\d+|>999|--)m,1mi:\??(\d+|>999|--)m$")
    for p_now in (None, -20.0, 0.0, 5.0, 10.8, 10.81, 11.0, 15.0, 30.0, 200.0, float("nan")):
      for mile in ({}, {"mileKwh": 0.5, "mileAcKwh": 0.0}, {"mileKwh": 0.0, "mileAcKwh": 0.2},
                   {"mileKwh": -0.1, "mileAcKwh": 0.0}, {"mileKwh": 0.5, "energyKwh": None}):
        out = self._on(widget, now, pNowKw=p_now, **mile)
        assert shape.match(out), out

  def test_no_moving_payload_ever_renders_m_per_h(self, widget, now):
    """Owner 2026-11-01: no m/h numbers while driving, generator on or off."""
    for ac in (0.0, 0.06, 1.4, 10.8, 27.7, 60.0):
      for p_now in (None, -50.0, -5.0, 0.0, 0.03, 1.0, 1.4, 5.0, 10.8, 30.0):
        for mile in ({}, {"mileKwh": 0.5, "mileAcKwh": 0.0}, {"mileKwh": 0.0, "mileAcKwh": 0.2},
                     {"mileKwh": -0.1, "mileAcKwh": 0.0}, {"mileKwh": 0.1, "mileAcKwh": 0.135}):
          for acseen in (True, False):
            out = widget._build_text(st(now, **{**self.MOV, "acKw": ac, "pNowKw": p_now}, acSeen=acseen, **mile))
            assert "m/h" not in out, out

  def test_generator_off_a_mile_that_netted_charging_is_over_999_not_a_rate(self, widget, now):
    out = widget._build_text(st(now, **self.MOV, mileKwh=0.10, mileAcKwh=0.135))
    assert out == "184m(63.49kwh),1mi:>999m", out

  def test_the_widest_form_stays_inside_the_char_bound(self, widget, now):
    out = self._on(widget, now, acKw=27.7, pNowKw=27.7, mileKwh=0.0, mileAcKwh=0.2)
    assert out == "27.7kw,>999m,1mi:>999m" and len(out) <= 31, out

  def test_stopped_forms_keep_their_gain(self, widget, now):
    assert widget._build_text(st(now, acKw=1.4, vMs=0.0)) == "1.4kw,112m(63.49kwh),+2.7m/h"


class TestEstimateMarker:
  """Owner 2026-11-01: no "@" anywhere; a leading "?" on every figure NOT from the live pNowKw.
  BASE: 320 Wh/km at 62 mph (99.78 km/h) = 31.93 kW + 1.2 kW accessories = 33.13 kW estimated."""

  MOV = TestTheMovingForm.MOV
  P_EST = 320.0 / 1000.0 * (62.0 / 2.23694 * 3.6) + 1.2

  def test_the_estimate_mirrors_the_producer_accessory_load(self):
    assert ed._P_ACC_KW == 1.2, "change everdrive_pnw.P_ACC_KW (opendbc) together with this"

  def test_no_pNowKw_is_the_speed_estimate_with_a_question_mark(self, widget, now):
    out = widget._build_text(st(now, **{**self.MOV, "pNowKw": None}))
    assert out == f"?{round(59.35 / self.P_EST * 62)}m(63.49kwh),1mi:?111m", out
    assert self.P_EST == pytest.approx(33.13, abs=0.01)
    assert widget._build_text(st(now, **{**self.MOV, "acKw": 10.8, "pNowKw": None})) == \
      f"10.8kw,?{round(112 * self.P_EST / (self.P_EST - 10.8))}m,1mi:?166m"

  def test_the_question_mark_drops_as_soon_as_pNowKw_is_live(self, widget, now):
    assert widget._build_text(st(now, **self.MOV)) == "184m(63.49kwh),1mi:184m"
    assert widget._build_text(st(now, **{**self.MOV, "acKw": 10.8, "pNowKw": 30.0})) == "10.8kw,175m,1mi:175m"

  def test_over_999_without_a_question_mark_only_from_a_measured_load(self, widget, now):
    assert widget._build_text(st(now, **{**self.MOV, "acKw": 10.8, "pNowKw": 10.0})) == "10.8kw,>999m,1mi:>999m"

  def test_an_estimate_at_or_under_the_charge_is_question_over_999(self, widget, now):
    out = widget._build_text(st(now, **{**self.MOV, "acKw": 40.0, "pNowKw": None}))
    assert out == "40.0kw,?>999m,1mi:?>999m", out

  def test_a_missing_reading_never_prints_a_bare_over_999(self, widget, now):
    for ac in (0.0, 5.0, 33.0, 40.0, 60.0):
      for mile in ({}, {"mileKwh": 0.0, "mileAcKwh": 0.2, "mileEst": True}, {"mileKwh": 0.5, "mileEst": True}):
        for eff in (True, False):
          out = widget._build_text(st(now, **{**self.MOV, "acKw": ac, "pNowKw": None, **mile}, effOk=eff))
          assert ">999m" not in out.replace("?>999m", ""), out

  def test_a_fully_measured_mile_has_no_question_mark(self, widget, now):
    out = widget._build_text(st(now, **{**self.MOV, "acKw": 10.8, "pNowKw": 30.0}, mileKwh=0.5, mileAcKwh=0.0233))
    assert out == "10.8kw,175m,1mi:125m", out

  def test_a_partly_estimated_mile_has_a_question_mark(self, widget, now):
    out = widget._build_text(st(now, **{**self.MOV, "acKw": 10.8, "pNowKw": 30.0, "mileEst": True},
                                mileKwh=0.5, mileAcKwh=0.0233))
    assert out == "10.8kw,175m,1mi:?125m", out
    out = widget._build_text(st(now, **{**self.MOV, "mileEst": True}, mileKwh=0.5, mileAcKwh=0.0))
    assert out == "184m(63.49kwh),1mi:?119m", out
    out = widget._build_text(st(now, **{**self.MOV, "acKw": 10.8, "pNowKw": 30.0, "mileEst": True},
                                mileKwh=0.1, mileAcKwh=0.135))
    assert out == "10.8kw,175m,1mi:?>999m", out

  def test_a_mile_from_an_older_producer_without_mileEst_is_not_called_measured(self, widget, now):
    mov = {k: v for k, v in self.MOV.items() if k != "mileEst"}
    out = widget._build_text(st(now, **mov, mileKwh=0.5, mileAcKwh=0.0))
    assert out == "184m(63.49kwh),1mi:?119m", out

  def test_before_the_first_mile_the_term_carries_the_instant_question_mark(self, widget, now):
    out = widget._build_text(st(now, **{**self.MOV, "pNowKw": None, "mileEst": None}))
    assert out == "?111m(63.49kwh),1mi:?111m", out

  def test_no_at_sign_and_no_m_per_h_in_any_moving_form(self, widget, now):
    n = 0
    for ac in (0.0, 1.4, 10.8, 40.0):
      for p_now in (None, -5.0, 0.0, 5.0, 10.8, 30.0, float("nan")):
        for mile in ({}, {"mileKwh": 0.5, "mileAcKwh": 0.0}, {"mileKwh": 0.0, "mileAcKwh": 0.2},
                     {"mileKwh": 0.1, "mileAcKwh": 0.135}):
          for mile_est in (True, False, None):
            for eff in (True, False):
              out = widget._build_text(st(now, **{**self.MOV, "acKw": ac, "pNowKw": p_now, "mileEst": mile_est},
                                          effOk=eff, **mile))
              assert "@" not in out and "m/h" not in out and "?--" not in out, out
              n += 1
    assert n == 4 * 7 * 4 * 3 * 2

  def test_no_at_sign_in_any_stopped_form(self, widget, now):
    for ac in (0.0, 1.4, 10.8):
      for v in (0.0, 2.0):
        out = widget._build_text(st(now, acKw=ac, vMs=v))
        assert "@" not in out, out
