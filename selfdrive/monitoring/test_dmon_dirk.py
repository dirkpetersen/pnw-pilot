"""
Standalone test for the dmon-branch dual-counter relaxed DM (5min pose / 15min phone).

Why a standalone harness:
  The full openpilot test env requires building Cython modules (params_pyx) and
  capnp bindings, which need `scons` and the full setup pipeline. To validate
  our changes to selfdrive/monitoring/helpers.py without that, we stub the heavy
  compiled deps with minimal in-process fakes, then import the REAL helpers module.

What we test (covers DMON.md §11 Phase 2):
  - 4 min pose distracted     → still active (no terminal)
  - 6 min pose distracted     → terminal (red)
  - 14 min phone distracted   → still active
  - 16 min phone distracted   → terminal
  - 12 × (30 s phone + 30 s clear) → snap-back recovery prevents terminal
  - Simultaneous pose + phone → pose hits terminal first (5 min < 15 min)
  - _PHONE_THRESH raised to 0.6
  - _MAX_TERMINAL_ALERTS raised to 10

Run from inside the bluepilot worktree:
  python3 selfdrive/monitoring/test_dmon_dirk.py
"""
from __future__ import annotations
import sys
import types
import os


# ───────────────────────── stubs for unbuilt deps ─────────────────────────
def _install_stubs():
    """Inject minimal fake modules for things we don't have built in this env."""

    # setproctitle
    sp = types.ModuleType("setproctitle")
    sp.getproctitle = lambda: "test"
    sp.setproctitle = lambda *a, **k: None
    sys.modules["setproctitle"] = sp

    # smbus2 (used by hardware on some paths)
    sys.modules["smbus2"] = types.ModuleType("smbus2")

    # openpilot.common.params — Params object with get_bool/put_bool_nonblocking no-ops
    class FakeParams:
        def __init__(self, *a, **k): pass
        def get_bool(self, k): return False
        def put_bool(self, k, v): pass
        def put_bool_nonblocking(self, k, v): pass
        def get(self, k): return None
        def put(self, k, v): pass

    op_common_params = types.ModuleType("openpilot.common.params")
    op_common_params.Params = FakeParams
    sys.modules["openpilot.common.params"] = op_common_params

    # cereal.messaging — minimal new_message that returns an object with attribute access
    class FakeMsg:
        def __init__(self): self.driverMonitoringState = types.SimpleNamespace()
    cereal_messaging = types.ModuleType("cereal.messaging")
    cereal_messaging.new_message = lambda *a, **k: FakeMsg()
    sys.modules["cereal.messaging"] = cereal_messaging

    # Events / alertmanager — accept arbitrary calls
    class FakeEvents:
        def __init__(self): self.names = []
        def add(self, name): self.names.append(name)
        def to_msg(self): return self.names
        def __len__(self): return len(self.names)
    op_events = types.ModuleType("openpilot.selfdrive.selfdrived.events")
    op_events.Events = FakeEvents
    sys.modules["openpilot.selfdrive.selfdrived.events"] = op_events

    op_alertmgr = types.ModuleType("openpilot.selfdrive.selfdrived.alertmanager")
    op_alertmgr.set_offroad_alert = lambda *a, **k: None
    sys.modules["openpilot.selfdrive.selfdrived.alertmanager"] = op_alertmgr

    # openpilot.common.realtime — just need DT_DMON
    op_realtime = types.ModuleType("openpilot.common.realtime")
    op_realtime.DT_DMON = 0.05
    sys.modules["openpilot.common.realtime"] = op_realtime

    # openpilot.common.filter_simple — FirstOrderFilter
    class FirstOrderFilter:
        def __init__(self, x0, ts, dt): self.x = x0; self.ts = ts; self.dt = dt
        def update(self, v): self.x = self.x + self.dt / self.ts * (v - self.x); return self.x
    op_filter = types.ModuleType("openpilot.common.filter_simple")
    op_filter.FirstOrderFilter = FirstOrderFilter
    sys.modules["openpilot.common.filter_simple"] = op_filter

    # openpilot.common.stat_live — RunningStatFilter
    class _Stat:
        def __init__(self): self.n = 0; self.M = 0.
        def mean(self): return self.M
    class RunningStatFilter:
        def __init__(self, raw_priors=None, max_trackable=None):
            self.filtered_stat = _Stat()
        def push_and_update(self, v):
            self.filtered_stat.n += 1
            self.filtered_stat.M = ((self.filtered_stat.M * (self.filtered_stat.n - 1)) + v) / self.filtered_stat.n
    op_stat = types.ModuleType("openpilot.common.stat_live")
    op_stat.RunningStatFilter = RunningStatFilter
    sys.modules["openpilot.common.stat_live"] = op_stat

    # openpilot.common.transformations.camera — DEVICE_CAMERAS
    cam_ns = types.SimpleNamespace
    fake_cam = cam_ns(dcam=cam_ns(width=1928, height=1208))
    op_camera = types.ModuleType("openpilot.common.transformations.camera")
    op_camera.DEVICE_CAMERAS = {("tici", "ar0231"): fake_cam}
    sys.modules["openpilot.common.transformations.camera"] = op_camera

    # parent packages
    for parent in ("openpilot", "openpilot.common", "openpilot.common.transformations",
                   "openpilot.selfdrive", "openpilot.selfdrive.selfdrived",
                   "openpilot.system"):
        sys.modules.setdefault(parent, types.ModuleType(parent))

    # openpilot.system.hardware.HARDWARE
    class FakeHardware:
        @staticmethod
        def get_device_type(): return "tici"
    op_hardware = types.ModuleType("openpilot.system.hardware")
    op_hardware.HARDWARE = FakeHardware()
    sys.modules["openpilot.system.hardware"] = op_hardware

    # cereal.{car, log} — provide EventName + GearShifter + DriverStateV2 builder
    class _EventNames:
        # populate every name referenced by helpers.py
        preDriverDistracted    = "preDriverDistracted"
        promptDriverDistracted = "promptDriverDistracted"
        driverDistracted       = "driverDistracted"
        preDriverUnresponsive    = "preDriverUnresponsive"
        promptDriverUnresponsive = "promptDriverUnresponsive"
        driverUnresponsive       = "driverUnresponsive"
        tooDistracted          = "tooDistracted"

    class _GearShifter:
        drive = "drive"; low = "low"; park = "park"; reverse = "reverse"; neutral = "neutral"

    class _DriverData:
        # Provides BOTH old (bp-6.0) and new (bp-dev) DriverData API fields so
        # this test works whether helpers.py uses leftBlinkProb / rightBlinkProb /
        # leftEyeProb / rightEyeProb / sunglassesProb (bp-6.0 release schema)
        # or the newer eyesVisibleProb / eyesClosedProb (bp-dev development schema).
        def __init__(self):
            self.faceOrientation = [0., 0., 0.]
            self.facePosition = [0., 0.]
            self.faceOrientationStd = [0., 0., 0.]
            self.facePositionStd = [0., 0.]
            self.faceProb = 0.
            # new API (bp-dev)
            self.eyesVisibleProb = 0.
            self.eyesClosedProb = 0.
            # old API (bp-6.0)
            self.leftEyeProb = 0.
            self.rightEyeProb = 0.
            self.leftBlinkProb = 0.
            self.rightBlinkProb = 0.
            self.sunglassesProb = 0.
            # both
            self.phoneProb = 0.

    class FakeDriverStateV2:
        def __init__(self):
            self.frameId = 0
            self.wheelOnRightProb = 0.
            self.leftDriverData  = _DriverData()
            self.rightDriverData = _DriverData()
        @classmethod
        def new_message(cls): return cls()

    cereal_log = types.ModuleType("cereal.log")
    cereal_log.OnroadEvent = types.SimpleNamespace(EventName=_EventNames())
    cereal_log.DriverStateV2 = FakeDriverStateV2
    sys.modules["cereal.log"] = cereal_log

    cereal_car = types.ModuleType("cereal.car")
    cereal_car.CarState = types.SimpleNamespace(GearShifter=_GearShifter)
    cereal_car.CarParams = types.SimpleNamespace()
    cereal_car.CarControl = types.SimpleNamespace()
    sys.modules["cereal.car"] = cereal_car

    cereal_pkg = types.ModuleType("cereal")
    cereal_pkg.car = cereal_car
    cereal_pkg.log = cereal_log
    cereal_pkg.messaging = cereal_messaging
    sys.modules["cereal"] = cereal_pkg


_install_stubs()


# Insert worktree root on path and import the REAL helpers
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
from selfdrive.monitoring.helpers import (        # noqa: E402
    DriverMonitoring, DRIVER_MONITOR_SETTINGS, DistractedType,
)


# ───────────────────────── helpers ─────────────────────────
DT = 0.05  # DT_DMON, matches realtime.py


def make_msg(face=True, pose_distracted=False, blink_distracted=False, phone_prob=0.0):
    """Build a fake DriverStateV2 message. left-side only (RHD=0)."""
    from cereal.log import DriverStateV2
    ds = DriverStateV2.new_message()
    ds.wheelOnRightProb = 0.
    left = ds.leftDriverData
    left.faceOrientation = [0., 0., 0.]
    # pose distraction: huge yaw error
    if pose_distracted:
        left.faceOrientation = [0., 1.5, 0.]    # |yaw| >> _POSE_YAW_THRESHOLD
    left.facePosition = [0., 0.]
    left.faceOrientationStd = [0., 0., 0.]
    left.facePositionStd = [0., 0.]
    left.faceProb = 1.0 if face else 0.0
    left.eyesVisibleProb = 1.0
    left.eyesClosedProb  = 1.0 if blink_distracted else 0.0
    left.phoneProb = phone_prob
    return ds


def run_seconds(dm: DriverMonitoring, seconds: float, msg_factory, driver_engaged=False, op_engaged=True, standstill=False):
    """Tick the DM at 20 Hz for `seconds`. msg_factory(frame_idx) -> DriverStateV2.

    Note: _update_states' signature differs between bp-dev (has steering_angle_deg)
    and bp-6.0 (no steering_angle_deg). Use positional args only so we work on both.
    """
    n = int(seconds / DT)
    last_events = None
    for i in range(n):
        dm._update_states(msg_factory(i), [0, 0, 0], 30.0, op_engaged, standstill)
        dm._update_events(driver_engaged, op_engaged, standstill, False, 30.0)
        last_events = dm.current_events
    return last_events


# ───────────────────────── assertions / tests ─────────────────────────
PASS = "\033[32mPASS\033[0m"
FAIL = "\033[31mFAIL\033[0m"
results = []


def check(name, cond, detail=""):
    tag = PASS if cond else FAIL
    line = f"  {tag}  {name}" + (f"   ({detail})" if detail else "")
    print(line)
    results.append((name, cond, detail))


def test_settings_values():
    print("\n[settings] locked-in constants")
    s = DRIVER_MONITOR_SETTINGS(device_type="tici")
    check("_PHONE_THRESH == 0.6",              s._PHONE_THRESH == 0.6,            f"got {s._PHONE_THRESH}")
    check("_MAX_TERMINAL_ALERTS == 10",        s._MAX_TERMINAL_ALERTS == 10,      f"got {s._MAX_TERMINAL_ALERTS}")
    check("_POSE_DISTRACTED_TIME == 300",      s._POSE_DISTRACTED_TIME == 300.,   f"got {s._POSE_DISTRACTED_TIME}")
    check("_PHONE_DISTRACTED_TIME == 900",     s._PHONE_DISTRACTED_TIME == 900.,  f"got {s._PHONE_DISTRACTED_TIME}")
    check("_RECOVERY_DEBOUNCE_FRAMES == 40",   s._RECOVERY_DEBOUNCE_FRAMES == 40, f"got {s._RECOVERY_DEBOUNCE_FRAMES}")


def test_phone_14min_active():
    print("\n[phone] 14 min continuous phone distracted → still active")
    dm = DriverMonitoring()
    # phone_prob = 0.9 well above 0.6 threshold
    run_seconds(dm, 14 * 60, lambda i: make_msg(face=True, phone_prob=0.9))
    check("awareness_phone > 0 after 14 min",  dm.awareness_phone > 0, f"awareness_phone={dm.awareness_phone:.4f}")
    check("awareness_pose unaffected (== 1.0)", abs(dm.awareness_pose - 1.0) < 1e-6, f"awareness_pose={dm.awareness_pose:.4f}")
    check("no terminal alert",                  dm.terminal_alert_cnt == 0, f"terminal_alert_cnt={dm.terminal_alert_cnt}")


def test_phone_16min_terminal():
    print("\n[phone] 16 min continuous → terminal")
    dm = DriverMonitoring()
    run_seconds(dm, 16 * 60, lambda i: make_msg(face=True, phone_prob=0.9))
    check("awareness_phone <= 0",         dm.awareness_phone <= 0,       f"awareness_phone={dm.awareness_phone:.4f}")
    check("terminal_alert_cnt >= 1",      dm.terminal_alert_cnt >= 1,    f"got {dm.terminal_alert_cnt}")


def test_pose_4min_active():
    print("\n[pose] 4 min continuous pose distracted → still active")
    dm = DriverMonitoring()
    run_seconds(dm, 4 * 60, lambda i: make_msg(face=True, pose_distracted=True))
    check("awareness_pose > 0 after 4 min", dm.awareness_pose > 0, f"awareness_pose={dm.awareness_pose:.4f}")
    check("no terminal alert",              dm.terminal_alert_cnt == 0, f"terminal_alert_cnt={dm.terminal_alert_cnt}")


def test_pose_6min_terminal():
    print("\n[pose] 6 min continuous pose → terminal")
    dm = DriverMonitoring()
    run_seconds(dm, 6 * 60, lambda i: make_msg(face=True, pose_distracted=True))
    check("awareness_pose <= 0",          dm.awareness_pose <= 0,      f"awareness_pose={dm.awareness_pose:.4f}")
    check("terminal_alert_cnt >= 1",      dm.terminal_alert_cnt >= 1,   f"got {dm.terminal_alert_cnt}")


def test_phone_snapback_12_glances():
    print("\n[phone] 12 × (30s phone + 30s clear) → snap-back, no terminal")
    dm = DriverMonitoring()
    # Each cycle: 30s phone, then 30s no phone. 60s/cycle × 12 = 12 min total.
    for _ in range(12):
        run_seconds(dm, 30, lambda i: make_msg(face=True, phone_prob=0.9))
        run_seconds(dm, 30, lambda i: make_msg(face=True))  # all clear
    check("awareness_phone ≈ 1.0 (snapped back)",
          abs(dm.awareness_phone - 1.0) < 0.01, f"awareness_phone={dm.awareness_phone:.4f}")
    check("no terminal",                  dm.terminal_alert_cnt == 0, f"terminal_alert_cnt={dm.terminal_alert_cnt}")


def test_pose_hits_first_when_simultaneous():
    print("\n[combined] simultaneous pose+phone for 5 min 5 s → pose past terminal, phone partially decayed")
    dm = DriverMonitoring()
    # Run just past the 5-min boundary to avoid float-precision tie at awareness=0.
    run_seconds(dm, 5 * 60 + 5, lambda i: make_msg(face=True, pose_distracted=True, phone_prob=0.9))
    check("awareness_pose < 0 (past terminal)", dm.awareness_pose < 0,     f"awareness_pose={dm.awareness_pose:.6f}")
    # phone decayed (305/900) = ~0.339 of full range, so awareness_phone ≈ 0.661
    check("awareness_phone ≈ 0.66 (one third decayed)",
          0.60 < dm.awareness_phone < 0.70,                                f"awareness_phone={dm.awareness_phone:.4f}")
    check("combined awareness = pose (the lower)",
          abs(dm.awareness - dm.awareness_pose) < 1e-9)


def test_too_distracted_lockout():
    print("\n[lockout] 10 terminal events sets DriverTooDistracted")
    dm = DriverMonitoring()
    # 6 min pose runs one terminal cycle. Repeat 11 × 6 min = 66 min.
    # But on each cycle awareness can only go to 0 ONCE per drive before being reset by driver_engaged.
    # In the test we never engage, so terminal_time just accumulates. Easier: directly bump
    # terminal_alert_cnt by simulating attentive-then-distracted phases that re-trigger.
    # Simplest: run pose-distracted to terminal, then 3s clear (snap-back), then distracted again.
    for cycle in range(11):
        run_seconds(dm, 6 * 60, lambda i: make_msg(face=True, pose_distracted=True))
        run_seconds(dm, 3, lambda i: make_msg(face=True))  # snap-back debounce
    check("terminal_alert_cnt >= 10", dm.terminal_alert_cnt >= 10, f"terminal_alert_cnt={dm.terminal_alert_cnt}")
    check("too_distracted flag set",  dm.too_distracted is True,    f"too_distracted={dm.too_distracted}")


def test_phone_prob_below_threshold_no_decay():
    print("\n[threshold] phone_prob just below 0.6 does NOT decay phone counter")
    dm = DriverMonitoring()
    run_seconds(dm, 60, lambda i: make_msg(face=True, phone_prob=0.55))
    check("awareness_phone unchanged at 0.55", abs(dm.awareness_phone - 1.0) < 1e-6,
          f"awareness_phone={dm.awareness_phone:.6f}")


if __name__ == "__main__":
    test_settings_values()
    test_phone_14min_active()
    test_phone_16min_terminal()
    test_pose_4min_active()
    test_pose_6min_terminal()
    test_phone_snapback_12_glances()
    test_pose_hits_first_when_simultaneous()
    test_too_distracted_lockout()
    test_phone_prob_below_threshold_no_decay()

    passed = sum(1 for _, ok, _ in results if ok)
    failed = sum(1 for _, ok, _ in results if not ok)
    print(f"\n{'=' * 60}")
    print(f"  Total: {passed + failed}   Passed: {passed}   Failed: {failed}")
    print(f"{'=' * 60}")
    sys.exit(0 if failed == 0 else 1)
