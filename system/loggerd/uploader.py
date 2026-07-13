#!/usr/bin/env python3
import json
import os
import random
import requests
import socket
import threading
import time
import traceback
import datetime
from collections.abc import Iterator

from cereal import car, log
import cereal.messaging as messaging
from openpilot.common.api import Api
from openpilot.common.utils import get_upload_stream
from openpilot.common.params import Params
from openpilot.common.realtime import set_core_affinity
from openpilot.system.hardware.hw import Paths
from openpilot.system.loggerd.xattr_cache import getxattr, setxattr
from openpilot.common.swaglog import cloudlog

NetworkType = log.DeviceState.NetworkType
UPLOAD_ATTR_NAME = 'user.upload'
UPLOAD_ATTR_VALUE = b'1'

# connect2xnor: pass-2 ("firehose") files. These are the LARGE files that stock
# openpilot only uploads when the backend issues an Athena upload request. The
# user's self-hosted backend issues no such request, so we proactively upload
# them ourselves -- but ONLY over real external WiFi (see PASS2_NETWORK_TYPES).
FIREHOSE_FILES = {"rlog", "rlog.zst", "fcamera.hevc", "ecamera.hevc"}
# connect2pnw: the HD-video subset of pass 2, deferrable via the DeferHDVideoUpload toggle (driver
# req 2026-07-06): on a connection that qualifies as unmetered but is actually precious (RV park
# WiFi, borrowed hotspot), keep uploading logs (qlog/rlog/qcam) but HOLD the big video files.
# Deferred = skipped-but-kept: never xattr-marked, so they upload normally once the toggle is off.
HD_VIDEO_FILES = {"fcamera.hevc", "ecamera.hevc", "dcamera.hevc"}
DEFER_HD_PARAM = "DeferHDVideoUpload"

# connect2xnor: only real external WiFi clients qualify for pass-2. The comma's
# own hotspot is never-default, so NM's PrimaryConnection stays LTE while
# hotspotting -> networkType reports `cell`, never `wifi`. So wifi cleanly means
# "not hotspot, not LTE". This is THE gate for proactive large-file uploads.
PASS2_NETWORK_TYPES = {int(NetworkType.wifi)}

# connect2xnor: param the UI reads to light the "Firehose Mode" indicator. Set
# only while a pass-2 (video/rlog) transfer is actually in flight.
FIREHOSE_ACTIVE_PARAM = "FirehoseActive"

# connect2pnw: Mbps of the in-flight pass-2 transfer, published once per completed HD file (~1/min)
# so the sidebar can show "CONNECT <n> Mbps" instead of a static "UPLOADING". One param write per
# file = negligible overhead (the speed is already computed for the upload_success log event).
FIREHOSE_SPEED_PARAM = "FirehoseSpeed"

# connect2pnw HD-interleave: force one pass-2 (HD/rlog) upload after this many consecutive successful
# pass-1 (small) uploads, so large video isn't starved behind a long small-file backlog. Small files
# still get priority (pass 1 runs every loop); this just guarantees HD makes steady progress.
PASS2_INTERLEAVE = 4

# connect2pnw device-locator (2026-07-08): the device roams WiFi segments and its LAN IP keeps
# changing, defeating SSH. AWS only ever sees the public NAT address, so the uploader self-reports
# its LAN IP as a `local_ip` query param on every upload_url request; the comma-uploader-api Lambda
# prints it as a CLIENT_IP line in CloudWatch. Look the device up any time with:
#   aws --profile dipeit logs filter-log-events --region us-west-2 \
#     --log-group-name /aws/lambda/comma-uploader-api --filter-pattern CLIENT_IP \
#     --start-time $(( ($(date +%s) - 3600) * 1000 )) --query 'events[-1].message' --output text


def _get_local_ip() -> str:
  # primary-route LAN address; the UDP connect only selects a route, no packet is sent.
  # Uncached on purpose: it must track WiFi roams, and the syscall cost is negligible per upload.
  try:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
      s.connect(("8.8.8.8", 80))
      return s.getsockname()[0]
    finally:
      s.close()
  except OSError:
    return ""

MAX_UPLOAD_SIZES = {
  "qlog": 25*1e6,  # can't be too restrictive here since we use qlogs to find
                   # bugs, including ones that can cause massive log sizes
  "qcam": 5*1e6,
}

# firehose2pnw: after a hard failure (non-2xx / network error), a file is NOT marked done so it will
# retry -- but it must not be re-picked every loop (that would livelock the whole uploader on the oldest
# failing file, starving every other file, and on metered it would re-PUT the payload each cycle). So a
# failed file is skipped for this cooldown, letting the uploader advance to the next file and come back
# later. Retry is still effectively forever (the entry clears once the cooldown lapses), just round-robin.
RETRY_COOLDOWN_S = 300.0


def _is_parked(sm) -> bool:
  # isparked2pnw: gear in PARK == "not drivable right now" (charging / parked), the root signal that an
  # EV keeping ignitionLine on while charging is NOT actually being driven. Unlike `standstill` (which
  # also matches stopped-at-a-red-light, where a 75 MB pass-2 burst would continue into motion), Park
  # cannot be true mid-maneuver. Fail-safe: False whenever carState is missing/invalid, so a car that is
  # actually being DRIVEN can never be mistaken for parked.
  cs = sm['carState']
  return bool(sm.valid['carState']) and cs.gearShifter == car.CarState.GearShifter.park and cs.vEgo < 0.5


def pass2_allowed(network_type: int, metered: bool, onroad: bool = False,
                  at_home: bool = False, parked: bool = False) -> bool:
  # connect2xnor: strict gate -- proactive large-file uploads happen ONLY on real external WiFi
  # (NetworkType.wifi). Never on LTE, never on the hotspot.
  # connect2pnw: ...and never when the active connection is flagged METERED (e.g. a phone hotspot the
  # user marked metered -> deviceState.networkMetered True). HD video/rlog must not burn metered data
  # (pass 1 small files still run, throttled). FirehoseActive is only set during a pass-2 transfer, so
  # gating pass 2 here also keeps the green "uploading" indicator off on metered connections.
  # connect2pnw ONROAD gate (2026-07-09 night drive): ...and never while DRIVING. 75 MB pass-2 bursts +
  # segment rotation caused transient CPU/IO stalls that surfaced as selfdrivedLagging ('System
  # Lagging') and locationdTemporaryError (inputsOK false ~0.4 s: late camOdo frames) alerts. Pass 1
  # (qlog/qcam, tiny) still flows onroad so the dashboard stays live; the HD bulk uploads when parked.
  # firehose2pnw: the onroad block is right while DRIVING, but an EV (F-150 Lightning / Tesla Raven)
  # parked and CHARGING keeps ignitionLine on -> onroad True, wrongly forbidding the best upload window
  # (parked for hours on home WiFi). So relax the onroad block:
  #   - AT a priority (home / geo-gated) WiFi network -> onroad never blocks (pure location override);
  #   - on any OTHER WiFi -> allow onroad pass-2 only while PARKED (gear in Park). isparked2pnw upgraded
  #     this from `standstill`: Park cannot be true mid-maneuver, so a 75 MB burst can never start at a
  #     red light and continue into motion (the exact selfdrivedLagging / locationd risk the gate guards).
  if network_type not in PASS2_NETWORK_TYPES or metered:
    return False
  if onroad and not (at_home or parked):
    return False
  return True

allow_sleep = bool(int(os.getenv("UPLOADER_SLEEP", "1")))
force_wifi = os.getenv("FORCEWIFI") is not None
fake_upload = os.getenv("FAKEUPLOAD") is not None


class FakeRequest:
  def __init__(self):
    self.headers = {"Content-Length": "0"}


class FakeResponse:
  def __init__(self):
    self.status_code = 200
    self.request = FakeRequest()


def get_directory_sort(d: str) -> list[str]:
  # ensure old format is sorted sooner
  o = ["0", ] if d.startswith("2024-") else ["1", ]
  return o + [s.rjust(10, '0') for s in d.rsplit('--', 1)]

def listdir_by_creation(d: str) -> list[str]:
  if not os.path.isdir(d):
    return []

  try:
    paths = [f for f in os.listdir(d) if os.path.isdir(os.path.join(d, f))]
    paths = sorted(paths, key=get_directory_sort)
    return paths
  except OSError:
    cloudlog.exception("listdir_by_creation failed")
    return []

def clear_locks(root: str) -> None:
  for logdir in os.listdir(root):
    path = os.path.join(root, logdir)
    try:
      for fname in os.listdir(path):
        if fname.endswith(".lock"):
          os.unlink(os.path.join(path, fname))
    except OSError:
      cloudlog.exception("clear_locks failed")


class Uploader:
  def __init__(self, dongle_id: str, root: str):
    self.dongle_id = dongle_id
    self.api = Api(dongle_id)
    self.root = root

    self.params = Params()

    # stats for last successfully uploaded file
    self.last_filename = ""
    self.last_upload_mbps = 0.0   # connect2pnw: speed of the last measured upload (Mbps), for the UI indicator

    self.immediate_folders = ["crash/", "boot/"]
    self.immediate_priority = {"qlog": 0, "qlog.zst": 0, "qcamera.ts": 1}
    self._defer_hd = False   # DeferHDVideoUpload snapshot, refreshed per pass-2 selection
    self._upload_err_n = 0   # firehose2pnw: consecutive hard-failure count for the LastUploadError tag
    self._retry_after: dict[str, float] = {}   # firehose2pnw: fn -> monotonic time it may be re-tried

    # firehose2pnw: a hard-failure UI tag from a previous process run is stale on restart -> clear it so
    # the CES overlay never shows a frozen 'UP ERR' after the uploader (or manager) restarts.
    try:
      self.params.remove("LastUploadError")
    except Exception:
      cloudlog.exception("failed to clear stale upload error")

    # connect2xnor: clear the firehose indicator on startup so a stale param
    # from a crash doesn't leave the UI showing "uploading" forever.
    self._set_firehose_active(False)
    # connect2pnw phase indicator: same startup clear for the pass-1 flag (sidebar GREEN)
    self._pass1_active = None
    self.set_pass1_active(False)

  def set_pass1_active(self, active: bool) -> None:
    # connect2pnw (driver req 2026-07-09): sidebar upload-phase colors — GREEN while pass-1 (qlog/qcam)
    # uploads make progress, BLUE while a pass-2 HD transfer is in flight (FirehoseActive). Write only
    # on CHANGE (the main loop iterates fast; a per-iteration param write would be needless churn).
    if active == self._pass1_active:
      return
    self._pass1_active = active
    try:
      self.params.put_bool_nonblocking("Pass1UploadActive", active)
    except Exception:
      cloudlog.exception("failed to set pass1 active param")

  def _set_firehose_active(self, active: bool) -> None:
    # connect2xnor: drives the repurposed "Firehose Mode" UI indicator. ON only
    # while a pass-2 transfer is actually in flight.
    try:
      self.params.put_bool(FIREHOSE_ACTIVE_PARAM, active)
    except Exception:
      cloudlog.exception("failed to set firehose active param")

  def _record_upload_error(self, code: int | None, last_exc) -> None:
    # firehose2pnw: publish a compact last-error string for the CES debug overlay, with a running
    # consecutive-failure count so a persistent problem is visibly stuck. Best-effort; never raises.
    try:
      if code is not None:
        desc = f"HTTP {code}"
      elif last_exc is not None:
        desc = type(last_exc[0]).__name__
      else:
        desc = "error"
      self._upload_err_n += 1
      self.params.put("LastUploadError", f"{desc} x{self._upload_err_n}")
    except Exception:
      cloudlog.exception("failed to record upload error")

  def _clear_upload_error(self) -> None:
    # firehose2pnw: a real 2xx clears the stuck-upload tag (only when one is currently shown).
    try:
      if self._upload_err_n:
        self._upload_err_n = 0
        self.params.remove("LastUploadError")
    except Exception:
      cloudlog.exception("failed to clear upload error")

  def _set_firehose_speed(self, mbps: float) -> None:
    # connect2pnw: publish the just-finished pass-2 file's speed (Mbps) so the sidebar can show
    # "CONNECT <n> Mbps". Written once per completed HD file (~1/min) -- negligible overhead.
    try:
      self.params.put(FIREHOSE_SPEED_PARAM, round(mbps))
    except Exception:
      cloudlog.exception("failed to set firehose speed param")

  def list_upload_files(self, metered: bool, pass2: bool = False) -> Iterator[tuple[str, str, str]]:
    r = self.params.get("AthenadRecentlyViewedRoutes")
    requested_routes = [] if r is None else [route for route in r.split(",") if route]
    try:
      self._defer_hd = self.params.get_bool(DEFER_HD_PARAM)   # refresh once per listing
    except Exception:
      self._defer_hd = False

    # firehose2pnw: drop lapsed retry-cooldowns (so those files become eligible again) and keep the map
    # bounded to files that failed within the last RETRY_COOLDOWN_S.
    now = time.monotonic()
    if self._retry_after:
      self._retry_after = {f: t for f, t in self._retry_after.items() if t > now}

    for logdir in listdir_by_creation(self.root):
      path = os.path.join(self.root, logdir)
      try:
        names = os.listdir(path)
      except OSError:
        continue

      if any(name.endswith(".lock") for name in names):
        continue

      for name in sorted(names, key=lambda n: self.immediate_priority.get(n, 1000)):
        key = os.path.join(logdir, name)
        fn = os.path.join(path, name)
        # skip files already uploaded
        try:
          ctime = os.path.getctime(fn)
          is_uploaded = getxattr(fn, UPLOAD_ATTR_NAME) == UPLOAD_ATTR_VALUE
        except OSError:
          cloudlog.event("uploader_getxattr_failed", key=key, fn=fn)
          # deleter could have deleted, so skip
          continue
        if is_uploaded:
          continue

        # firehose2pnw: a file that just hard-failed is on cooldown -> skip it (advance to the next
        # file) so one persistently-failing file can't livelock the whole uploader / re-burn metered.
        if self._retry_after.get(fn, 0.0) > now:
          continue

        # connect2xnor: split the two passes. Pass 1 (default) handles the small
        # files stock openpilot uploads proactively; pass 2 handles ONLY the
        # large firehose files (rlog/fcamera/ecamera). Each pass ignores the
        # other's files so they never interleave.
        # DeferHDVideoUpload: hold video in EITHER pass (Gemini: dcamera isn't a pass-2 file), keep
        # logs flowing. Skipped-not-marked -> uploads normally once the toggle is off.
        if self._defer_hd and name in HD_VIDEO_FILES:
          continue

        if pass2:
          if name not in FIREHOSE_FILES:
            continue
        else:
          if name in FIREHOSE_FILES:
            continue

        # limit uploading on metered connections
        if metered:
          dt = datetime.timedelta(hours=12)
          if logdir in self.immediate_folders and (datetime.datetime.now() - datetime.datetime.fromtimestamp(ctime)) < dt:
            continue

          if name == "qcamera.ts" and not any(logdir.startswith(r.split('|')[-1]) for r in requested_routes):
            continue

        yield name, key, fn

  def next_file_to_upload(self, metered: bool) -> tuple[str, str, str] | None:
    upload_files = list(self.list_upload_files(metered))

    for name, key, fn in upload_files:
      if any(f in fn for f in self.immediate_folders):
        return name, key, fn

    for name, key, fn in upload_files:
      if name in self.immediate_priority:
        return name, key, fn

    return None

  def next_pass2_file_to_upload(self, metered: bool) -> tuple[str, str, str] | None:
    # connect2xnor: oldest un-uploaded large file (listdir_by_creation already
    # sorts oldest-first), so video/logs leave the device before the deleter
    # reaches them.
    for name, key, fn in self.list_upload_files(metered, pass2=True):
      return name, key, fn
    return None

  def do_upload(self, key: str, fn: str):
    # connect2pnw device-locator: local_ip rides along as a query param (api_get forwards **kwargs
    # as query-string params); the Lambda logs it so the roaming device's LAN IP is findable in AWS.
    url_resp = self.api.get("v1.4/" + self.dongle_id + "/upload_url/", timeout=10, path=key,
                            local_ip=_get_local_ip(), access_token=self.api.get_token())
    if url_resp.status_code == 412:
      return url_resp

    url_resp_json = json.loads(url_resp.text)
    url = url_resp_json['url']
    headers = url_resp_json['headers']
    cloudlog.debug("upload_url v1.4 %s %s", url, str(headers))

    if fake_upload:
      return FakeResponse()

    stream = None
    try:
      compress = key.endswith('.zst') and not fn.endswith('.zst')
      stream, _ = get_upload_stream(fn, compress)
      response = requests.put(url, data=stream, headers=headers, timeout=10)
      return response
    finally:
      if stream:
        stream.close()

  def upload(self, name: str, key: str, fn: str, network_type: int, metered: bool) -> bool:
    try:
      sz = os.path.getsize(fn)
    except OSError:
      cloudlog.exception("upload: getsize failed")
      return False

    cloudlog.event("upload_start", key=key, fn=fn, sz=sz, network_type=network_type, metered=metered)

    if sz == 0:
      # tag files of 0 size as uploaded
      success = True
    elif name in MAX_UPLOAD_SIZES and sz > MAX_UPLOAD_SIZES[name]:
      cloudlog.event("uploader_too_large", key=key, fn=fn, sz=sz)
      success = True
    else:
      start_time = time.monotonic()

      stat = None
      last_exc = None
      try:
        stat = self.do_upload(key, fn)
      except Exception as e:
        last_exc = (e, traceback.format_exc())

      # firehose2pnw (driver req: NEVER lose data): only a real 2xx marks a file done. 412
      # ("already in S3" / backend-declined), 401/403 (rejected or expired presigned S3 PUT, or a bad
      # auth token — e.g. right after boot before the clock/token is valid) and network errors are NOT
      # success -> the file stays un-xattr-marked and retries on every eligible window until it truly
      # uploads or the deleter reclaims it. Previously 401/403/412 counted as success, xattr-marking the
      # file done even though nothing reached S3 (silent data loss). Retrying forever is cheap and safe:
      # a 412 returns from the upload_url GET before any bytes move, and the oldest un-uploaded files are
      # deleted as the disk fills while driving, so nothing accumulates unbounded.
      code = stat.status_code if stat is not None else None
      if code in (200, 201):
        self.last_filename = fn
        dt = time.monotonic() - start_time
        content_length = int(stat.request.headers.get("Content-Length", 0))
        speed = (content_length / 1e6) / dt
        self.last_upload_mbps = speed * 8.0   # connect2pnw: MB/s -> Mbps for the sidebar indicator
        cloudlog.event("upload_success", key=key, fn=fn, sz=sz, content_length=content_length,
                       network_type=network_type, metered=metered, speed=speed)
        success = True
        self._retry_after.pop(fn, None)
        self._clear_upload_error()
      else:
        success = False
        # firehose2pnw: NOT marked done (so it retries), but put it on cooldown so it isn't re-picked
        # every loop -> the uploader advances to other files instead of livelocking on this one, and on
        # metered it can't re-PUT the payload every cycle.
        self._retry_after[fn] = time.monotonic() + RETRY_COOLDOWN_S
        if code == 412:
          # benign: the gateway already has it (or declined) -> retry after cooldown, don't surface
          cloudlog.event("upload_ignored", key=key, fn=fn, sz=sz, network_type=network_type, metered=metered)
        else:
          cloudlog.event("upload_failed", stat=stat, exc=last_exc, key=key, fn=fn, sz=sz, network_type=network_type, metered=metered)
          self._record_upload_error(code, last_exc)

    if success:
      # tag file as uploaded
      try:
        setxattr(fn, UPLOAD_ATTR_NAME, UPLOAD_ATTR_VALUE)
      except OSError:
        cloudlog.event("uploader_setxattr_failed", exc=last_exc, key=key, fn=fn, sz=sz)

    return success


  def step(self, network_type: int, metered: bool, pass2: bool = False) -> bool | None:
    # connect2xnor: pass2 picks the next large firehose file; pass1 (default) is
    # unchanged stock behavior.
    d = self.next_pass2_file_to_upload(metered) if pass2 else self.next_file_to_upload(metered)
    if d is None:
      return None

    name, key, fn = d

    # qlogs and bootlogs need to be compressed before uploading
    if key.endswith(('qlog', 'rlog')) or (key.startswith('boot/') and not key.endswith('.zst')):
      key += ".zst"

    # connect2xnor: light the firehose indicator only while the large transfer runs.
    if pass2:
      self._set_firehose_active(True)
    try:
      ok = self.upload(name, key, fn, network_type, metered)
      if pass2 and ok:
        self._set_firehose_speed(self.last_upload_mbps)   # connect2pnw: publish Mbps for the UI (~1/file)
      return ok
    finally:
      if pass2:
        self._set_firehose_active(False)


def _firehose_network_guard(uploader: Uploader, exit_event: threading.Event) -> None:
  # connect2pnw: the firehose ("uploading") indicator is set True for the full duration of a pass-2
  # HD PUT and only cleared in step()'s finally. If WiFi drops (or the connection is marked metered)
  # mid-transfer, the main uploader loop stays blocked inside that PUT until it times out (~10s), so
  # the green CONNECT->UPLOADING logo lingers while networkType already shows LTE / metered -- even
  # though no further HD data should move (the stalled PUT fails and the file re-sends on the next
  # eligible WiFi window). This daemon watches deviceState on its OWN SubMaster (the main loop is
  # busy) and clears the indicator within ~0.5s of the network leaving WiFi or becoming metered. It
  # only ever CLEARS (never sets) the flag, so it can't make the UI show "uploading" when ineligible;
  # the uploader still sets it True only under pass2_allowed (real, non-metered WiFi).
  if force_wifi:
    return
  sm = messaging.SubMaster(['deviceState', 'carState'])
  while not exit_event.is_set():
    # a raised exception here would silently kill this daemon and revert to the stale-indicator bug
    # for the rest of the process, so swallow + back off (mirrors _set_firehose_active's safety).
    try:
      sm.update(1000)
      if not sm.updated['deviceState']:
        continue
      ds = sm['deviceState']
      onroad = ds.started   # onroad gate: clear the indicator too when a drive starts mid-transfer
      # firehose2pnw: mirror the main loop's relaxed gate so a legitimate onroad-at-home (EV charging)
      # or onroad-parked pass-2 transfer isn't falsely cleared by this guard.
      at_home = uploader.params.get_bool("OnPriorityNetwork")
      parked = _is_parked(sm)
      if not pass2_allowed(ds.networkType.raw, ds.networkMetered, onroad, at_home, parked) and uploader.params.get_bool(FIREHOSE_ACTIVE_PARAM):
        uploader._set_firehose_active(False)
        uploader._set_firehose_speed(0)   # connect2pnw: drop the stale Mbps too, so it can't show on resume
    except Exception:
      cloudlog.exception("firehose network guard iteration failed")
      time.sleep(1)


def main(exit_event: threading.Event | None = None) -> None:
  if exit_event is None:
    exit_event = threading.Event()

  try:
    set_core_affinity([0, 1, 2, 3])
  except Exception:
    cloudlog.exception("failed to set core affinity")

  clear_locks(Paths.log_root())

  params = Params()
  dongle_id = params.get("DongleId")

  if dongle_id is None:
    cloudlog.info("uploader missing dongle_id")
    raise Exception("uploader can't start without dongle id")

  sm = messaging.SubMaster(['deviceState', 'carState'])   # firehose2pnw: carState for the parked gate
  uploader = Uploader(dongle_id, Paths.log_root())

  # connect2pnw: clear the firehose ("uploading") indicator promptly when WiFi drops mid-transfer.
  # The main loop blocks inside the in-flight pass-2 PUT (up to its ~10s timeout) and can't clear the
  # flag itself in time; this daemon watcher does, on its own deviceState sub. Dies with the process.
  threading.Thread(target=_firehose_network_guard, args=(uploader, exit_event), daemon=True).start()

  backoff = 0.1
  pass1_run = 0   # consecutive successful small (pass-1) uploads since the last HD (pass-2) upload
  while not exit_event.is_set():
    sm.update(0)
    offroad = params.get_bool("IsOffroad")
    onroad = not offroad   # connect2pnw onroad gate: pass 2 (75 MB bursts) only while parked
    # firehose2pnw: at a priority (home / geo-gated) WiFi, onroad no longer blocks pass 2 (EV charging
    # keeps ignitionLine on -> onroad); on any other WiFi, onroad pass-2 is allowed only while PARKED
    # (gear in Park). _is_parked() fails safe to False on missing/invalid carState, so a car actually
    # being driven can never open the onroad-other-WiFi burst window.
    at_home = params.get_bool("OnPriorityNetwork")
    parked = _is_parked(sm)
    network_type = sm['deviceState'].networkType if not force_wifi else NetworkType.wifi
    if network_type == NetworkType.none:
      if allow_sleep:
        time.sleep(60 if offroad else 5)
      continue

    # connect2xnor: honor force_wifi (test/debug) for the raw value too.
    network_type_raw = int(NetworkType.wifi) if force_wifi else sm['deviceState'].networkType.raw
    metered = sm['deviceState'].networkMetered
    p1 = uploader.step(network_type_raw, metered)               # pass 1 (small files)
    uploader.set_pass1_active(p1 is True)   # sidebar GREEN while pass-1 progresses (change-only write)
    if p1 is None:
      pass1_run = 0
    elif p1:
      pass1_run += 1

    # connect2pnw: PASS 2 (large "firehose" files: rlog + HD video), ONLY on real external WiFi that
    # is NOT flagged metered (networkType==wifi is never true on the hotspot or LTE, and a WiFi the
    # user marked metered is excluded too, so this can't burn cellular or metered data).
    # HD-interleave: run pass 2 when pass 1 has nothing left (p1 is None) OR after every
    # PASS2_INTERLEAVE successful small uploads, so HD video makes steady progress instead of being
    # starved behind a long backlog of small files (e.g. right after a multi-segment drive). Small
    # files keep priority (pass 1 runs every iteration); HD just never waits indefinitely.
    p2 = None
    if pass2_allowed(network_type_raw, metered, onroad, at_home, parked) and (p1 is None or pass1_run >= PASS2_INTERLEAVE):
      p2 = uploader.step(network_type_raw, metered, pass2=True)
      pass1_run = 0

    # backoff from the combined outcome: None=nothing to do anywhere; True=made progress; False=failure
    results = [r for r in (p1, p2) if r is not None]
    if not results:
      success = None
    elif any(results):
      success = True
    else:
      success = False

    if success is None:
      backoff = 60 if offroad else 5
    elif success:
      backoff = 0.1
    else:
      cloudlog.info("upload backoff %r", backoff)
      backoff = min(backoff*2, 120)
    if allow_sleep:
      time.sleep(backoff + random.uniform(0, backoff))


if __name__ == "__main__":
  main()
