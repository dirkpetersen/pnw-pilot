"""nudgelesshighway2pnw unit tests — highway/freeway gate on the AUTO (nudgeless) lane-change path.

Driver report: a family member flipped the turn signal to make an ordinary CITY turn and openpilot
auto-lane-changed into cross-traffic. These tests lock in the fix:
  on_highway = (MapHighwayClass in FREEWAY_CLASSES) OR (v_ego >= HIGHWAY_MIN_SPEED)
and that it gates ONLY the auto (no-touch) path — the manual steering-torque nudge is untouched and
must still work on any road, including a city street.

Params/mem-params are fully mocked (never touch real /data/params or /dev/shm/params) so these tests
run standalone regardless of what's baked into the local params_pyx.so build.
"""
import openpilot.common.params as params_module
import openpilot.selfdrive.controls.lib.desire_helper as dh_module
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import AUTO_LANE_CHANGE_DELAY, DesireHelper, LaneChangeState

V35 = 35 * CV.MPH_TO_MS   # city arterial, above LANE_CHANGE_SPEED_MIN, below the highway speed floor
V30 = 30 * CV.MPH_TO_MS   # moderate speed, below the highway speed floor -- must rely on map class
V50 = 50 * CV.MPH_TO_MS   # above HIGHWAY_MIN_SPEED (45 mph) -- speed floor alone must allow auto


class _CP:
  def __init__(self, brand="tesla", fp=""):
    self.brand = brand
    self.carFingerprint = fp


class _NoopParams:
  """Stand-in used only to get DesireHelper.__init__ past its Params()/mem-Params construction
  without touching real params (this dev host's params_pyx.so may not even have NudgeForLaneChange
  baked in). Overwritten immediately after construction with the real per-test fakes below."""
  def __init__(self, *a, **k):
    pass

  def get_bool(self, *a, **k):
    return False

  def get(self, *a, **k):
    return None


class _Params:
  """Fake of the on-disk Params() the toggle is read from."""
  def __init__(self, nudge_required=False):
    self._nudge_required = nudge_required

  def get_bool(self, key):
    if key == "NudgeForLaneChange":
      return self._nudge_required
    return False


class _FakeMem:
  """Fake of the /dev/shm/params mem-Params mapd bridges MapHighwayClass into."""
  def __init__(self, highway_class=""):
    self.highway_class = highway_class

  def get(self, key, return_default=True):
    if key == "MapHighwayClass":
      return self.highway_class
    return None


class _CS:
  def __init__(self, v_ego, left_blinker=False, right_blinker=False, steering_pressed=False,
               steering_torque=0.0, left_blindspot=False, right_blindspot=False):
    self.vEgo = v_ego
    self.leftBlinker = left_blinker
    self.rightBlinker = right_blinker
    self.steeringPressed = steering_pressed
    self.steeringTorque = steering_torque
    self.leftBlindspot = left_blindspot
    self.rightBlindspot = right_blindspot


def _dh(monkeypatch, highway_class="", nudge_required=False, brand="tesla", fp=""):
  # __init__ constructs both self.params = Params() and (locally) mem-Params("/dev/shm/params");
  # patch both call sites to the no-op so construction can't blow up on this host, then swap in the
  # real per-test fakes below.
  monkeypatch.setattr(dh_module, "Params", _NoopParams)
  monkeypatch.setattr(params_module, "Params", _NoopParams)
  d = DesireHelper(CP=_CP(brand, fp))
  d.params = _Params(nudge_required)
  d.nudgeless_lane_change = d.nudgeless_supported and not nudge_required
  d.mem_params = _FakeMem(highway_class)
  d._map_highway_class = highway_class  # prime immediately -- skip the ~1s (20-cycle) read cadence
  return d


def _run(d, cs, seconds, lane_change_prob=0.0):
  for _ in range(int(round(seconds / DT_MDL))):
    d.update(cs, True, lane_change_prob)
  return d.lane_change_state


def test_nudgeless_supported_on_tesla(monkeypatch):
  d = _dh(monkeypatch)
  assert d.nudgeless_supported
  assert d.nudgeless_lane_change


# (a) low-speed city, blocked -----------------------------------------------------------------

def test_city_residential_class_blocks_auto(monkeypatch):
  d = _dh(monkeypatch, highway_class="residential")
  cs = _CS(V35, left_blinker=True)
  state = _run(d, cs, 3.0)  # well past AUTO_LANE_CHANGE_DELAY
  assert state == LaneChangeState.preLaneChange  # never reaches laneChangeStarting via auto


def test_city_unknown_class_blocks_auto(monkeypatch):
  # fail-safe: no map data at all (mapd not running / stale) still must not auto-change on a city street
  d = _dh(monkeypatch, highway_class="")
  cs = _CS(V35, left_blinker=True)
  state = _run(d, cs, 3.0)
  assert state == LaneChangeState.preLaneChange


# (b) freeway class allows auto ----------------------------------------------------------------

def test_freeway_class_allows_auto_below_speed_floor(monkeypatch):
  d = _dh(monkeypatch, highway_class="motorway")
  cs = _CS(V30, left_blinker=True)  # above LANE_CHANGE_SPEED_MIN, below HIGHWAY_MIN_SPEED
  state = _run(d, cs, AUTO_LANE_CHANGE_DELAY + 0.2)
  assert state == LaneChangeState.laneChangeStarting


def test_trunk_class_also_allows_auto(monkeypatch):
  d = _dh(monkeypatch, highway_class="trunk")
  cs = _CS(V30, left_blinker=True)
  state = _run(d, cs, AUTO_LANE_CHANGE_DELAY + 0.2)
  assert state == LaneChangeState.laneChangeStarting


# (c) high speed allows auto regardless of (unknown) class -------------------------------------

def test_high_speed_allows_auto_with_unknown_class(monkeypatch):
  d = _dh(monkeypatch, highway_class="")
  cs = _CS(V50, left_blinker=True)
  state = _run(d, cs, AUTO_LANE_CHANGE_DELAY + 0.2)
  assert state == LaneChangeState.laneChangeStarting


# (d) blindspot still blocks, even on a highway -------------------------------------------------

def test_blindspot_blocks_auto_on_highway(monkeypatch):
  d = _dh(monkeypatch, highway_class="motorway")
  cs = _CS(V50, left_blinker=True, left_blindspot=True)
  state = _run(d, cs, AUTO_LANE_CHANGE_DELAY + 0.5)
  assert state == LaneChangeState.preLaneChange


# (e) manual torque-nudge path is untouched, works on a city street -----------------------------

def test_manual_nudge_works_on_city_street(monkeypatch):
  d = _dh(monkeypatch, highway_class="residential")
  cs = _CS(V35, left_blinker=True, steering_pressed=True, steering_torque=1.0)
  d.update(cs, True, 0.0)  # rising blinker edge -> preLaneChange
  assert d.lane_change_state == LaneChangeState.preLaneChange
  d.update(cs, True, 0.0)  # torque nudge evaluated -> laneChangeStarting, gate not consulted
  assert d.lane_change_state == LaneChangeState.laneChangeStarting


def test_manual_nudge_works_at_low_speed_offset_city_no_map(monkeypatch):
  # same as above but with no map data at all -- still must not block the manual nudge
  d = _dh(monkeypatch, highway_class="")
  cs = _CS(V35, left_blinker=True, steering_pressed=True, steering_torque=1.0)
  d.update(cs, True, 0.0)
  d.update(cs, True, 0.0)
  assert d.lane_change_state == LaneChangeState.laneChangeStarting


# toggle OFF (nudge required) -- gate is irrelevant, auto path never fires at all ---------------

def test_nudge_required_never_auto_even_on_highway(monkeypatch):
  d = _dh(monkeypatch, highway_class="motorway", nudge_required=True)
  cs = _CS(V50, left_blinker=True)
  state = _run(d, cs, AUTO_LANE_CHANGE_DELAY + 0.5)
  assert state == LaneChangeState.preLaneChange
