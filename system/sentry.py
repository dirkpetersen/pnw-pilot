"""Install exception handler for process crash."""
import os
import traceback
from datetime import datetime
from enum import Enum
from importlib.metadata import PackageNotFoundError, version as pkg_version

import sentry_sdk
from sentry_sdk.integrations.threading import ThreadingIntegration

from openpilot.common.params import Params
from openpilot.system.athena.registration import UNREGISTERED_DONGLE_ID
from openpilot.system.hardware import HARDWARE
from openpilot.system.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog
from openpilot.system.version import get_build_metadata, get_version

from openpilot.sunnypilot.sunnylink.api import UNREGISTERED_SUNNYLINK_DONGLE_ID

CRASHES_DIR = Paths.crash_log_root()


class SentryProject(Enum):
  # python project
  # SELFDRIVE = "https://186a6736b7927e5ae9b92c869ba81b6b@o1138119.ingest.us.sentry.io/4508660076052480"
  SELFDRIVE = "https://dd96fe788b05e1b9a59fb68bd7604314@o4510926103379968.ingest.us.sentry.io/4510926105739264"

  # native project
  SELFDRIVE_NATIVE = SELFDRIVE


def report_tombstone(fn: str, message: str, contents: str) -> None:
  cloudlog.error({'tombstone': message})

  with sentry_sdk.configure_scope() as scope:
    set_user()
    scope.set_extra("tombstone_fn", fn)
    scope.set_extra("tombstone", contents)
    sentry_sdk.capture_message(message=message)
    sentry_sdk.flush()


def capture_exception(*args, **kwargs) -> None:
  cloudlog.error("crash", exc_info=kwargs.get('exc_info', 1))

  try:
    save_exception(traceback.format_exc())

    set_user()
    sentry_sdk.capture_exception(*args, **kwargs)
    sentry_sdk.flush()  # https://github.com/getsentry/sentry-python/issues/291
  except Exception:
    cloudlog.exception("sentry exception")


def save_exception(content: str) -> None:
  try:
    if not os.path.exists(CRASHES_DIR):
      os.makedirs(CRASHES_DIR)

    files = [
      os.path.join(CRASHES_DIR, datetime.now().strftime("%Y-%m-%d--%H-%M-%S.log")),
      os.path.join(CRASHES_DIR, "error.log")
    ]

    for fn in files:
      with open(fn, 'w') as f:
        if fn == "error.log":
          lines = content.splitlines()[-3:]
          f.write("\n".join(lines))
        else:
          f.write(content)

    cloudlog.error(f"logged crash to {files}")
  except Exception:
    cloudlog.exception("error when attempting to save exception")


def capture_fingerprint_mock() -> None:
  try:
    set_user()
    message = "car doesn't match any fingerprints"
    sentry_sdk.capture_message(message=message, level="error")
    sentry_sdk.flush()
  except Exception as e:
    cloudlog.exception(f"sentry fingerprint MOCK exception: {e}")


def capture_fingerprint(candidate: str, car_name: str) -> None:
  try:
    set_user()
    sentry_sdk.set_tag("carFingerprint", candidate)
    sentry_sdk.set_tag("carName", car_name)

    message = f"Fingerprinted {candidate}"
    sentry_sdk.capture_message(message=message, level="info")
    sentry_sdk.flush()
  except Exception as e:
    cloudlog.exception(f"sentry fingerprint exception: {e}")


def set_tag(key: str, value: str) -> None:
  sentry_sdk.set_tag(key, value)


def set_user() -> None:
  dongle_id, git_username, _ = get_properties()
  sentry_sdk.set_user({"id": dongle_id, "name": git_username})


def get_properties() -> tuple[str, str, str]:
  params = Params()
  hardware_serial: str = params.get("HardwareSerial") or ""
  git_username: str = params.get("GithubUsername") or ""
  dongle_id: str = params.get("DongleId") or f"{UNREGISTERED_DONGLE_ID}-{hardware_serial}"
  sunnylink_dongle_id: str = params.get("SunnylinkDongleId") or UNREGISTERED_SUNNYLINK_DONGLE_ID

  return dongle_id, git_username, sunnylink_dongle_id


# BluePilot: enable_logs exists only on sentry-sdk >= ~2.35; older builds (incl. some AGNOS venvs) raise TypeError.
def _sentry_sdk_supports_enable_logs() -> bool:
  # sentry_sdk.init(enable_logs=...) raises TypeError on SDKs that predates the option (e.g. some AGNOS venvs).
  try:
    v = pkg_version("sentry-sdk")
  except PackageNotFoundError:
    return False
  parts: list[int] = []
  for segment in v.split(".")[:3]:
    digits = "".join(ch for ch in segment if ch.isdigit())
    if not digits:
      break
    parts.append(min(int(digits), 9999))
    if len(parts) == 3:
      break
  while len(parts) < 3:
    parts.append(0)
  return tuple(parts) >= (2, 35, 0)


# End BluePilot

def init(project: SentryProject) -> bool:
  build_metadata = get_build_metadata()

  env = build_metadata.channel_type
  dongle_id, git_username, sunnylink_dongle_id = get_properties()

  integrations = []
  if project == SentryProject.SELFDRIVE:
    integrations.append(ThreadingIntegration(propagate_hub=True))

  # BluePilot: gate enable_logs on installed sentry-sdk (see helper above).
  init_kw = dict(
    default_integrations=False,
    release=get_version(),
    integrations=integrations,
    traces_sample_rate=1.0,
    max_value_length=8192,
    environment=env,
  )
  if _sentry_sdk_supports_enable_logs():
    init_kw["enable_logs"] = True

  sentry_sdk.init(project.value, **init_kw)
  # End BluePilot

  sentry_sdk.set_user({"id": dongle_id, "name": git_username})
  sentry_sdk.set_tag("dirty", build_metadata.openpilot.is_dirty)
  sentry_sdk.set_tag("origin", build_metadata.openpilot.git_origin)
  sentry_sdk.set_tag("branch", build_metadata.channel)
  sentry_sdk.set_tag("commit", build_metadata.openpilot.git_commit)
  sentry_sdk.set_tag("device", HARDWARE.get_device_type())
  sentry_sdk.set_tag("sunnylink_dongle_id", sunnylink_dongle_id)

  return True
