import math
import numpy as np
import time
import wave


from cereal import car, messaging
from openpilot.common.basedir import BASEDIR
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import Ratekeeper
from openpilot.common.utils import retry
from openpilot.common.swaglog import cloudlog
from openpilot.common.params import Params

from openpilot.system import micd
from openpilot.system.hardware import HARDWARE

SAMPLE_RATE = 48000
SAMPLE_BUFFER = 4096 # (approx 100ms)
MAX_VOLUME = 1.0
MIN_VOLUME = 0.1

# location2pnw: one-shot police siren chirp, played ONCE when a police report appears <= this far AHEAD
# (mirrors the UI "POLICE AHEAD" banner trigger). Fully isolated from the safety alert sounds: it ONLY
# plays when no AudibleAlert is active, so it can never delay or mask a warning/disengage. A failed siren
# load just disables the chirp; soundd still starts (alert sounds are essential).
SIREN_ENABLED = False  # driver req 2026-07-01: police siren OFF (keep the visual banner only). Flip to True to restore.
SIREN_FILE = "police_siren.wav"
SIREN_VOLUME = 0.7
POLICE_NEAR_MI = 0.5
POLICE_CHECK_S = 0.5   # how often soundd polls the police mem param
SELFDRIVE_STATE_TIMEOUT = 5 # 5 seconds
FILTER_DT = 1. / (micd.SAMPLE_RATE / micd.FFT_SAMPLES)

AMBIENT_DB = 24 # DB where MIN_VOLUME is applied
DB_SCALE = 30 # AMBIENT_DB + DB_SCALE is where MAX_VOLUME is applied

VOLUME_BASE = 20
if HARDWARE.get_device_type() == "tizi":
  AMBIENT_DB = 30
  VOLUME_BASE = 10

AudibleAlert = car.CarControl.HUDControl.AudibleAlert


sound_list: dict[int, tuple[str, int | None, float]] = {
  # AudibleAlert, file name, play count (none for infinite)
  AudibleAlert.engage: ("engage.wav", 1, MAX_VOLUME),
  AudibleAlert.disengage: ("disengage.wav", 1, MAX_VOLUME),
  AudibleAlert.refuse: ("refuse.wav", 1, MAX_VOLUME),

  AudibleAlert.prompt: ("prompt.wav", 1, MAX_VOLUME),
  AudibleAlert.promptRepeat: ("prompt.wav", None, MAX_VOLUME),
  AudibleAlert.promptDistracted: ("prompt_distracted.wav", None, MAX_VOLUME),

  AudibleAlert.warningSoft: ("warning_soft.wav", None, MAX_VOLUME),
  AudibleAlert.warningImmediate: ("warning_immediate.wav", None, MAX_VOLUME),
}
if HARDWARE.get_device_type() == "tizi":
  sound_list.update({
    AudibleAlert.engage: ("engage_tizi.wav", 1, MAX_VOLUME),
    AudibleAlert.disengage: ("disengage_tizi.wav", 1, MAX_VOLUME),
  })

def check_selfdrive_timeout_alert(sm):
  ss_missing = time.monotonic() - sm.recv_time['selfdriveState']

  if ss_missing > SELFDRIVE_STATE_TIMEOUT:
    if sm['selfdriveState'].enabled and (ss_missing - SELFDRIVE_STATE_TIMEOUT) < 10:
      return True

  return False


class Soundd:
  def __init__(self):
    self.load_sounds()

    self.current_alert = AudibleAlert.none
    self.current_volume = MIN_VOLUME
    self.current_sound_frame = 0

    self.selfdrive_timeout_alert = False

    self.spl_filter_weighted = FirstOrderFilter(0, 2.5, FILTER_DT, initialized=False)

    # police siren (isolated, one-shot) — load guarded so a missing/bad file can't stop soundd starting
    self.siren_sound = None
    self.siren_frame = None          # None = idle; int = playback position
    self.siren_uuid = None           # last police report we chirped for (one chirp per report)
    self.siren_near = False          # were we within range last check (handles uuid=None re-trigger)
    self.last_police_check = 0.0
    try:
      self.mem = Params("/dev/shm/params")
    except Exception:
      self.mem = None
    try:
      with wave.open(BASEDIR + "/selfdrive/assets/sounds/" + SIREN_FILE, 'r') as wf:
        assert wf.getnchannels() == 1 and wf.getsampwidth() == 2 and wf.getframerate() == SAMPLE_RATE
        self.siren_sound = np.frombuffer(wf.readframes(wf.getnframes()), dtype=np.int16).astype(np.float32) / (2**16/2)
    except Exception:
      cloudlog.exception("soundd: police siren load failed (chirp disabled)")

  def load_sounds(self):
    self.loaded_sounds: dict[int, np.ndarray] = {}

    # Load all sounds
    for sound in sound_list:
      filename, play_count, volume = sound_list[sound]

      with wave.open(BASEDIR + "/selfdrive/assets/sounds/" + filename, 'r') as wavefile:
        assert wavefile.getnchannels() == 1
        assert wavefile.getsampwidth() == 2
        assert wavefile.getframerate() == SAMPLE_RATE

        length = wavefile.getnframes()
        self.loaded_sounds[sound] = np.frombuffer(wavefile.readframes(length), dtype=np.int16).astype(np.float32) / (2**16/2)

  def get_sound_data(self, frames): # get "frames" worth of data from the current alert sound, looping when required

    ret = np.zeros(frames, dtype=np.float32)

    if self.current_alert != AudibleAlert.none:
      num_loops = sound_list[self.current_alert][1]
      sound_data = self.loaded_sounds[self.current_alert]
      written_frames = 0

      current_sound_frame = self.current_sound_frame % len(sound_data)
      loops = self.current_sound_frame // len(sound_data)

      while written_frames < frames and (num_loops is None or loops < num_loops):
        available_frames = sound_data.shape[0] - current_sound_frame
        frames_to_write = min(available_frames, frames - written_frames)
        ret[written_frames:written_frames+frames_to_write] = sound_data[current_sound_frame:current_sound_frame+frames_to_write]
        written_frames += frames_to_write
        self.current_sound_frame += frames_to_write
    elif self.siren_sound is not None and self.siren_frame is not None:
      # No safety alert active -> play the one-shot police siren at its own fixed volume. Isolated: this
      # branch is unreachable while any AudibleAlert is sounding, so it can never delay/mask a warning.
      # SNAPSHOT siren_frame to a LOCAL first (this runs on the PortAudio callback thread while soundd_thread
      # may re-arm siren_frame=0): reading the attribute multiple times could make a malformed slice ->
      # ValueError -> crash the callback -> halt ALL audio incl. safety alerts. (The alert path above
      # snapshots current_sound_frame for the same reason.)
      frame = self.siren_frame
      if frame is not None and frame < len(self.siren_sound):
        n = min(frames, len(self.siren_sound) - frame)
        ret[:n] = self.siren_sound[frame:frame + n]
        if self.siren_frame == frame:          # only advance if soundd_thread didn't re-arm mid-callback
          self.siren_frame = None if frame + n >= len(self.siren_sound) else frame + n
        return ret * SIREN_VOLUME

    return ret * self.current_volume

  def callback(self, data_out: np.ndarray, frames: int, time, status) -> None:
    if status:
      cloudlog.warning(f"soundd stream over/underflow: {status}")
    data_out[:frames, 0] = self.get_sound_data(frames)

  def update_alert(self, new_alert):
    current_alert_played_once = self.current_alert == AudibleAlert.none or self.current_sound_frame > len(self.loaded_sounds[self.current_alert])
    if self.current_alert != new_alert and (new_alert != AudibleAlert.none or current_alert_played_once):
      self.current_alert = new_alert
      self.current_sound_frame = 0

  def get_audible_alert(self, sm):
    if sm.updated['selfdriveState']:
      new_alert = sm['selfdriveState'].alertSound.raw
      self.update_alert(new_alert)
    elif check_selfdrive_timeout_alert(sm):
      self.update_alert(AudibleAlert.warningImmediate)
      self.selfdrive_timeout_alert = True
    elif self.selfdrive_timeout_alert:
      self.update_alert(AudibleAlert.none)
      self.selfdrive_timeout_alert = False

  def calculate_volume(self, weighted_db):
    volume = ((weighted_db - AMBIENT_DB) / DB_SCALE) * (MAX_VOLUME - MIN_VOLUME) + MIN_VOLUME
    return math.pow(VOLUME_BASE, (np.clip(volume, MIN_VOLUME, MAX_VOLUME) - 1))

  @retry(attempts=10, delay=3)
  def get_stream(self, sd):
    # reload sounddevice to reinitialize portaudio
    sd._terminate()
    sd._initialize()
    return sd.OutputStream(channels=1, samplerate=SAMPLE_RATE, callback=self.callback, blocksize=SAMPLE_BUFFER)

  def soundd_thread(self):
    # sounddevice must be imported after forking processes
    import sounddevice as sd

    sm = messaging.SubMaster(['selfdriveState', 'soundPressure'])

    with self.get_stream(sd) as stream:
      rk = Ratekeeper(20)

      cloudlog.info(f"soundd stream started: {stream.samplerate=} {stream.channels=} {stream.dtype=} {stream.device=}, {stream.blocksize=}")
      while True:
        sm.update(0)

        if sm.updated['soundPressure'] and self.current_alert == AudibleAlert.none: # only update volume filter when not playing alert
          self.spl_filter_weighted.update(sm["soundPressure"].soundPressureWeightedDb)
          self.current_volume = self.calculate_volume(float(self.spl_filter_weighted.x))

        self.get_audible_alert(sm)

        # police siren: arm one chirp per new report that's <= POLICE_NEAR_MI ahead (mirrors the UI banner).
        # Reading a /dev/shm mem param; fully guarded so it can never disrupt the alert audio path.
        if SIREN_ENABLED and self.mem is not None and self.siren_sound is not None:
          now = time.monotonic()
          if now - self.last_police_check > POLICE_CHECK_S:
            self.last_police_check = now
            try:
              ls = self.mem.get("LocationServices", return_default=True)
              p = ls.get("police", {}) if isinstance(ls, dict) else {}
              pd = p.get("dist_mi")
              near = p.get("state") == "alert" and pd is not None and pd <= POLICE_NEAR_MI
              if near:
                uuid = p.get("uuid")
                if not self.siren_near or uuid != self.siren_uuid:   # new appearance or new report -> chirp
                  self.siren_frame = 0
                  self.siren_uuid = uuid
              self.siren_near = near
            except Exception:
              pass

        rk.keep_time()

        assert stream.active


def main():
  s = Soundd()
  s.soundd_thread()


if __name__ == "__main__":
  main()
