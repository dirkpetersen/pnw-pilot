#!/usr/bin/env python3
import json
import os
import random
import requests
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

# connect2xnor: only real external WiFi clients qualify for pass-2. The comma's
# own hotspot is never-default, so NM's PrimaryConnection stays LTE while
# hotspotting -> networkType reports `cell`, never `wifi`. So wifi cleanly means
# "not hotspot, not LTE". This is THE gate for proactive large-file uploads.
PASS2_NETWORK_TYPES = {int(NetworkType.wifi)}

# connect2xnor: param the UI reads to light the "Firehose Mode" indicator. Set
# only while a pass-2 (video/rlog) transfer is actually in flight.
FIREHOSE_ACTIVE_PARAM = "FirehoseActive"

# connect2pnw HD-interleave: force one pass-2 (HD/rlog) upload after this many consecutive successful
# pass-1 (small) uploads, so large video isn't starved behind a long small-file backlog. Small files
# still get priority (pass 1 runs every loop); this just guarantees HD makes steady progress.
PASS2_INTERLEAVE = 4

MAX_UPLOAD_SIZES = {
  "qlog": 25*1e6,  # can't be too restrictive here since we use qlogs to find
                   # bugs, including ones that can cause massive log sizes
  "qcam": 5*1e6,
}


def pass2_allowed(network_type: int) -> bool:
  # connect2xnor: strict gate -- proactive large-file uploads happen ONLY on
  # real external WiFi (NetworkType.wifi). Never on LTE, never on the hotspot.
  return network_type in PASS2_NETWORK_TYPES

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

    self.immediate_folders = ["crash/", "boot/"]
    self.immediate_priority = {"qlog": 0, "qlog.zst": 0, "qcamera.ts": 1}

    # connect2xnor: clear the firehose indicator on startup so a stale param
    # from a crash doesn't leave the UI showing "uploading" forever.
    self._set_firehose_active(False)

  def _set_firehose_active(self, active: bool) -> None:
    # connect2xnor: drives the repurposed "Firehose Mode" UI indicator. ON only
    # while a pass-2 transfer is actually in flight.
    try:
      self.params.put_bool(FIREHOSE_ACTIVE_PARAM, active)
    except Exception:
      cloudlog.exception("failed to set firehose active param")

  def list_upload_files(self, metered: bool, pass2: bool = False) -> Iterator[tuple[str, str, str]]:
    r = self.params.get("AthenadRecentlyViewedRoutes")
    requested_routes = [] if r is None else [route for route in r.split(",") if route]

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

        # connect2xnor: split the two passes. Pass 1 (default) handles the small
        # files stock openpilot uploads proactively; pass 2 handles ONLY the
        # large firehose files (rlog/fcamera/ecamera). Each pass ignores the
        # other's files so they never interleave.
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
    url_resp = self.api.get("v1.4/" + self.dongle_id + "/upload_url/", timeout=10, path=key, access_token=self.api.get_token())
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

      if stat is not None and stat.status_code in (200, 201, 401, 403, 412):
        self.last_filename = fn
        dt = time.monotonic() - start_time
        if stat.status_code == 412:
          cloudlog.event("upload_ignored", key=key, fn=fn, sz=sz, network_type=network_type, metered=metered)
        else:
          content_length = int(stat.request.headers.get("Content-Length", 0))
          speed = (content_length / 1e6) / dt
          cloudlog.event("upload_success", key=key, fn=fn, sz=sz, content_length=content_length,
                         network_type=network_type, metered=metered, speed=speed)
        success = True
      else:
        success = False
        cloudlog.event("upload_failed", stat=stat, exc=last_exc, key=key, fn=fn, sz=sz, network_type=network_type, metered=metered)

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
      return self.upload(name, key, fn, network_type, metered)
    finally:
      if pass2:
        self._set_firehose_active(False)


def _firehose_network_guard(uploader: Uploader, exit_event: threading.Event) -> None:
  # connect2pnw: the firehose ("uploading") indicator is set True for the full duration of a pass-2
  # HD PUT and only cleared in step()'s finally. If WiFi drops mid-transfer, the main uploader loop
  # stays blocked inside that PUT until it times out (~10s), so the green CONNECT->UPLOADING logo
  # lingers while networkType already shows LTE -- even though no HD data can move over cellular (the
  # stalled PUT just fails and the file re-sends on the next WiFi window). This daemon watches
  # deviceState on its OWN SubMaster (the main loop is busy) and clears the indicator within ~0.5s of
  # the network leaving WiFi. It only ever CLEARS (never sets) the flag, so it can't make the UI show
  # "uploading" on LTE; the uploader still sets it True only under pass2_allowed (real WiFi).
  if force_wifi:
    return
  sm = messaging.SubMaster(['deviceState'])
  while not exit_event.is_set():
    sm.update(1000)
    if not sm.updated['deviceState']:
      continue
    if not pass2_allowed(sm['deviceState'].networkType.raw) and uploader.params.get_bool(FIREHOSE_ACTIVE_PARAM):
      uploader._set_firehose_active(False)


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
  while not exit_event.is_set():
    sm.update(0)
    offroad = params.get_bool("IsOffroad")
    network_type = sm['deviceState'].networkType if not force_wifi else NetworkType.wifi
    if network_type == NetworkType.none:
      if allow_sleep:
        time.sleep(60 if offroad else 5)
      continue

    # connect2xnor: honor force_wifi (test/debug) for the raw value too.
    network_type_raw = int(NetworkType.wifi) if force_wifi else sm['deviceState'].networkType.raw
    metered = sm['deviceState'].networkMetered
    p1 = uploader.step(network_type_raw, metered)               # pass 1 (small files)
    if p1 is None:
      pass1_run = 0
    elif p1:
      pass1_run += 1

    # connect2pnw: PASS 2 (large "firehose" files: rlog + HD video), ONLY on real external WiFi
    # (networkType==wifi is never true on the hotspot or LTE, so this can't burn cellular).
    # HD-interleave: run pass 2 when pass 1 has nothing left (p1 is None) OR after every
    # PASS2_INTERLEAVE successful small uploads, so HD video makes steady progress instead of being
    # starved behind a long backlog of small files (e.g. right after a multi-segment drive). Small
    # files keep priority (pass 1 runs every iteration); HD just never waits indefinitely.
    p2 = None
    if pass2_allowed(network_type_raw) and (p1 is None or pass1_run >= PASS2_INTERLEAVE):
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
