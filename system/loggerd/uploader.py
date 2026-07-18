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

from cereal import log
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

# uploadretry2pnw: GENTLE retry cooldown. On a hard failure a file is NOT marked done (no silent data
# loss) but goes on this long per-file cooldown so it isn't re-picked every loop — no chatter, no
# livelock, no metered re-burn. 15 min is deliberately gentle; retries are effectively forever (the
# entry clears when the cooldown lapses) but spaced far apart.
RETRY_COOLDOWN_S = 900.0

# upload412ui2pnw: telling a benign dedupe 412 apart from a systemic outbreak (finding F1).
#
# A lone 412 is usually dedupe: the gateway already has this exact file from an earlier attempt whose
# 2xx we lost. That is NOT rare here -- this fork's only-2xx-marks-uploaded rule (uploadretry2pnw)
# means a dedupe'd file is a PERMANENT ORPHAN: it re-412s every RETRY_COOLDOWN_S forever (the deleter
# preserves un-uploaded files). A naive "N 412s in a row" counter false-alarms on that single orphan
# every night it sits parked with an otherwise-drained queue. A batch of several DISTINCT dedupe files
# (e.g. a connection drop right after the gateway accepted 3+ files but their 2xx ACKs were lost) makes
# it worse: those 412 sequentially with no shared cooldown, so a plain count crosses any small threshold
# in seconds. Neither case is the footgun -- both are files the gateway has legitimately already seen.
#
# The real outbreak signature (API_HOST reverted to the wrong backend, so EVERY upload_url GET 412s
# forever) is different: it declines files the gateway has *never* seen, including ones recorded AFTER
# the trouble started. Dedupe is impossible for those -- the gateway cannot already have a file that
# didn't exist yet when the streak of 412s began. So: remember the wall-clock time the current 412
# streak started (first 412 since the last successful upload, any file/pass); if a LATER 412 in that
# same streak lands on a file whose ctime is AFTER the streak's start, that file could not possibly be
# legitimate dedupe -- surface it. Both false-alarm cases above involve only files that predate the
# streak (they were recorded, and first attempted, before any 412 happened), so they never trigger no
# matter how many of them there are or how long the orphan keeps re-412ing.
#
# Tradeoff (intentional): if nothing new is being recorded (parked, no drive), an outbreak can only
# ever re-412 pre-existing files, so it stays silent until a drive starts recording fresh files again.
# That's fine -- UP ERR only renders on the ONROAD CES overlay, so the only moment the alarm is useful
# to the driver is while driving, which is exactly when a fresh file will appear within seconds.

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


def pass1_allowed(metered: bool) -> bool:
  # uploadgate2pnw (driver spec 2026-07-13): METERED -> no drive-FILE uploads at all, not even the
  # small pass-1 files (stock uploads them throttled on metered; the driver wants zero metered file
  # traffic). Unmetered (WiFi or LTE) -> pass 1 (qlog/qcam) flows. Location services are NOT gated by
  # this (driver qualification): the CloudWatch device locator gets its own tiny heartbeat below
  # (LOCATOR_PING_S) that runs on ANY connection, so the device stays findable on metered too.
  return not metered


# uploadgate2pnw: device-locator heartbeat period. On a connection where file uploads are blocked
# (metered) no upload_url requests happen, which used to silence the CloudWatch CLIENT_IP locator.
# This standalone ping (~1 KB request, no file transferred) keeps the locator alive everywhere.
LOCATOR_PING_S = 300.0


def pass2_allowed(network_type: int, metered: bool, at_home: bool, onroad: bool, parked: bool) -> bool:
  # uploadgate2pnw (driver spec 2026-07-13, replacing the firehose2pnw gate that caused the commIssue
  # cascade): the FIRST check is the priority (GPS-gated home) WiFi. Pass 2 (rlog + HD video, 75 MB
  # bursts) runs ONLY when connected to one of the driver's priority networks — NEVER anywhere else,
  # no matter how unmetered the connection is. Rationale: an on-the-road hotspot may report unmetered,
  # but background 75 MB uploads saturate it and break the driver's own connectivity.
  #   at_home  = OnPriorityNetwork param (network_arbiterd, change-only write on SSID match)
  #   parked   = GearPark param (card, change-only write on gear transitions) — no msgq subscription
  #              in this process (lesson 2026-07-13: uploader carState subs caused the cascade).
  # The onroad block stays for the driving case; Park (or offroad) opens it so an EV charging at home
  # (ignition line on -> onroad True) still uploads. Metered blocks everything, everywhere.
  if network_type not in PASS2_NETWORK_TYPES or metered:
    return False
  if not at_home:
    return False
  if onroad and not parked:
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
    self._retry_after: dict[str, float] = {}   # uploadretry2pnw: fn -> monotonic time it may retry
    self._upload_err_desc: str | None = None   # uploadretry2pnw: last LastUploadError we wrote (change-only)
    # upload412ui2pnw: wall-clock start of the current run of 412s with no success in between; None
    # means no streak in progress. Reset to None on the next 2xx. datetime (not time.monotonic) because
    # it must compare against file ctimes -- same idiom list_upload_files already uses for the metered
    # 12h window below. See the "telling a benign dedupe 412 apart from a systemic outbreak" comment
    # above RETRY_COOLDOWN_S for the design.
    self._412_streak_start: datetime.datetime | None = None
    # clear any stale hard-error tag from a previous process run so the CES overlay never shows a ghost
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

  def _set_firehose_speed(self, mbps: float) -> None:
    # connect2pnw: publish the just-finished pass-2 file's speed (Mbps) so the sidebar can show
    # "CONNECT <n> Mbps". Written once per completed HD file (~1/min) -- negligible overhead.
    try:
      self.params.put(FIREHOSE_SPEED_PARAM, round(mbps))
    except Exception:
      cloudlog.exception("failed to set firehose speed param")

  def _record_upload_error(self, code: int | None, last_exc) -> None:
    # uploadretry2pnw: surface a compact last-error for the CES overlay, CHANGE-ONLY (write only when
    # the description differs from what's shown) so a bad-network drive can't spam param writes — that
    # per-failure churn was part of the 2026-07-13 chattiness. No count (a count would change every
    # failure and defeat change-only).
    #
    # actionable-only (driver 2026-07-14): ONLY show an error the driver could actually act on — an HTTP
    # status FROM the server (e.g. 412 = the silent-data-loss footgun, 40x = auth). A code of None means
    # the request never reached the server (connection reset / timeout / DNS) — a transient network drop,
    # nothing to do about it while on the road — so it is NOT surfaced. The file is still preserved and
    # gently retried; a genuine server-side problem still shows. This kills the "UP ERR Connection error"
    # that lingered on flaky links with no actionable cause.
    #
    # upload412ui2pnw: 412 reaches this function too now (finding F1 — it used to take the
    # upload_ignored branch and never call here at all, so a systemic 412 outbreak never surfaced).
    # The dedupe-vs-outbreak judgment call is made by the CALLER (the streak/ctime discriminator in
    # upload(), see the comment above RETRY_COOLDOWN_S), not here — this function just records
    # whatever code it's handed, same as any other hard failure.
    try:
      if code is None:
        return                                 # transient network failure -> no actionable UP ERR
      desc = f"HTTP {code}"
      if desc != self._upload_err_desc:
        self._upload_err_desc = desc
        self.params.put("LastUploadError", desc)
    except Exception:
      cloudlog.exception("failed to record upload error")

  def _clear_upload_error(self) -> None:
    # uploadretry2pnw: a real 2xx clears the tag — change-only (only touch the param when one is shown).
    try:
      if self._upload_err_desc is not None:
        self._upload_err_desc = None
        self.params.remove("LastUploadError")
    except Exception:
      cloudlog.exception("failed to clear upload error")

  def list_upload_files(self, metered: bool, pass2: bool = False) -> Iterator[tuple[str, str, str]]:
    r = self.params.get("AthenadRecentlyViewedRoutes")
    requested_routes = [] if r is None else [route for route in r.split(",") if route]
    try:
      self._defer_hd = self.params.get_bool(DEFER_HD_PARAM)   # refresh once per listing
    except Exception:
      self._defer_hd = False

    # uploadretry2pnw: drop lapsed retry-cooldowns (those files become eligible again) — keeps the map
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

        # uploadretry2pnw: a just-failed file is on cooldown -> skip it (advance to the next file) so
        # one persistently-failing file can't livelock the uploader or re-burn a connection.
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

      # uploadretry2pnw (driver req: never lose data): ONLY a real 2xx marks a file uploaded. 412
      # ("already in S3"/backend-declined), 401/403 (rejected/expired presigned PUT or bad token) and
      # network errors are NOT success -> the file stays un-xattr-marked and retries later (never
      # marked done without reaching S3). Retries are GENTLE: a failed file goes on a long per-file
      # cooldown so it isn't re-picked every loop (no livelock, no chatter) and the LastUploadError UI
      # tag is written CHANGE-ONLY (once when a new error appears, once when it clears — never per
      # failure; per-failure param writes were part of the 2026-07-13 chattiness we removed).
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
        self._412_streak_start = None   # upload412ui2pnw: a clean upload proves the backend is reachable again
        self._clear_upload_error()
      else:
        success = False
        self._retry_after[fn] = time.monotonic() + RETRY_COOLDOWN_S   # gentle: back off this file
        if code == 412:
          # upload412ui2pnw (finding F1 fix): still behaves exactly as before by default -- NOT marked
          # uploaded, cooled down, silently retried, logged as upload_ignored. It only escalates to
          # _record_upload_error if THIS file's ctime postdates the streak's start (see the "telling a
          # benign dedupe 412 apart from a systemic outbreak" comment above RETRY_COOLDOWN_S): that is
          # the one case dedupe cannot explain, because the gateway can't already have a file that
          # didn't exist when the streak of 412s began.
          cloudlog.event("upload_ignored", key=key, fn=fn, sz=sz, network_type=network_type, metered=metered)
          if self._412_streak_start is None:
            # first 412 of a new streak -- this file necessarily predates "now" (it had to exist to be
            # attempted), so it can never itself satisfy the freshness check below. It only marks where
            # the streak began, for whichever LATER 412 (if any) to be compared against.
            self._412_streak_start = datetime.datetime.now()
          else:
            try:
              is_fresh = datetime.datetime.fromtimestamp(os.path.getctime(fn)) > self._412_streak_start
            except OSError:
              is_fresh = False   # deleter raced us; ctime unknowable -- don't guess, stay quiet
            if is_fresh:
              self._record_upload_error(412, last_exc)
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
  sm = messaging.SubMaster(['deviceState'])
  while not exit_event.is_set():
    # a raised exception here would silently kill this daemon and revert to the stale-indicator bug
    # for the rest of the process, so swallow + back off (mirrors _set_firehose_active's safety).
    try:
      sm.update(1000)
      if not sm.updated['deviceState']:
        continue
      ds = sm['deviceState']
      onroad = ds.started   # onroad gate: clear the indicator too when a drive starts mid-transfer
      # uploadgate2pnw: gate inputs from PARAMS only (change-only writers elsewhere) — this guard
      # thread deliberately subscribes to nothing beyond the 2 Hz deviceState.
      at_home = uploader.params.get_bool("OnPriorityNetwork")
      parked = uploader.params.get_bool("GearPark")
      if not pass2_allowed(ds.networkType.raw, ds.networkMetered, at_home, onroad, parked) and uploader.params.get_bool(FIREHOSE_ACTIVE_PARAM):
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

  sm = messaging.SubMaster(['deviceState'])
  uploader = Uploader(dongle_id, Paths.log_root())

  # connect2pnw: clear the firehose ("uploading") indicator promptly when WiFi drops mid-transfer.
  # The main loop blocks inside the in-flight pass-2 PUT (up to its ~10s timeout) and can't clear the
  # flag itself in time; this daemon watcher does, on its own deviceState sub. Dies with the process.
  threading.Thread(target=_firehose_network_guard, args=(uploader, exit_event), daemon=True).start()

  backoff = 0.1
  pass1_run = 0   # consecutive successful small (pass-1) uploads since the last HD (pass-2) upload
  last_locator_ping = 0.0   # uploadgate2pnw: standalone CloudWatch locator heartbeat (any connection)
  while not exit_event.is_set():
    sm.update(0)
    offroad = params.get_bool("IsOffroad")
    onroad = not offroad   # connect2pnw onroad gate: pass 2 (75 MB bursts) only while parked/at home
    network_type = sm['deviceState'].networkType if not force_wifi else NetworkType.wifi
    if network_type == NetworkType.none:
      if allow_sleep:
        time.sleep(60 if offroad else 5)
      continue

    # uploadgate2pnw: keep the CloudWatch device locator alive on EVERY connection, incl. metered
    # (driver: location services are never gated). A bare upload_url GET (~1 KB, no file follows)
    # carries the local_ip param the Lambda logs. Best-effort; never blocks the loop on failure.
    if time.monotonic() - last_locator_ping >= LOCATOR_PING_S:
      last_locator_ping = time.monotonic()
      try:
        uploader.api.get("v1.4/" + dongle_id + "/upload_url/", timeout=5, path="locator/ping",
                         local_ip=_get_local_ip(), access_token=uploader.api.get_token())
      except Exception:
        cloudlog.event("locator_ping_failed")

    # connect2xnor: honor force_wifi (test/debug) for the raw value too.
    network_type_raw = int(NetworkType.wifi) if force_wifi else sm['deviceState'].networkType.raw
    metered = sm['deviceState'].networkMetered

    # uploadgate2pnw (driver spec): metered -> NO file uploads at all (pass 1 previously ran throttled
    # on metered; now it needs an unmetered connection — WiFi or LTE both fine).
    p1 = None
    if pass1_allowed(metered):
      p1 = uploader.step(network_type_raw, metered)             # pass 1 (small files)
    uploader.set_pass1_active(p1 is True)   # sidebar GREEN while pass-1 progresses (change-only write)
    if p1 is None:
      pass1_run = 0
    elif p1:
      pass1_run += 1

    # uploadgate2pnw: PASS 2 (rlog + HD video) ONLY at a priority (home) network — never on other
    # WiFi/hotspots however unmetered (a background 75 MB burst kills an on-the-road hotspot), never
    # metered, and only offroad-or-parked (GearPark param from card; no msgq subs in this process).
    # HD-interleave: run pass 2 when pass 1 has nothing left (p1 is None) OR after every
    # PASS2_INTERLEAVE successful small uploads, so HD video makes steady progress instead of being
    # starved behind a long backlog of small files. Small files keep priority.
    at_home = params.get_bool("OnPriorityNetwork")
    parked = params.get_bool("GearPark")
    p2 = None
    if pass2_allowed(network_type_raw, metered, at_home, onroad, parked) and (p1 is None or pass1_run >= PASS2_INTERLEAVE):
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
