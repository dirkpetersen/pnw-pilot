#!/usr/bin/env python3
import os
import re
from pathlib import Path

HERE = os.path.abspath(os.path.dirname(__file__))
ROOT = HERE + "/.."

blacklist = [
  ".git/",
  ".github/workflows/",

  "matlab.*.md",

  # no LFS or submodules in release
  ".lfsconfig",
  ".gitattributes",
  ".git$",
  ".gitmodules",

  # ==============================================================================================
  # EXCLUDED BY DEFAULT — AND IT MUST STAY THAT WAY. Paired with .lfsconfig's fetchexclude; if you
  # ever change one, change the other.
  # ==============================================================================================
  # big_driving_supercombo.onnx is 1.7 GB and is ONLY ever read on an external-GPU
  # (eGPU-via-tinygrad) host — modeld builds the big pkl solely when UsbGpuPresent. The comma 3X
  # never touches its content, so shipping it would add ~95% to every install and update download
  # for zero benefit on the car.
  #
  # It is also excluded from LFS fetches (.lfsconfig `fetchexclude`), which means `git lfs pull`
  # deliberately leaves it as a ~135 B POINTER on disk. Including it here would therefore put a
  # pointer file masquerading as a 1.7 GB model into the release tree — and trip
  # build_stripped.sh's `git lfs ls-files` guard, aborting the release with "LFS files detected!"
  # (this broke the 3pnw aarch64 build on 2026-07-20).
  #
  # Planned: an eGPU toggle will make this opt-in for the (rare) tinygrad-on-external-GPU setup.
  # Until then, fetch it by hand if you actually need it:
  #   git lfs pull --include big_driving_supercombo.onnx
  "big_driving_supercombo.onnx",
]

# gets you through the blacklist
whitelist: list[str] = [
]

if __name__ == "__main__":
  for f in Path(ROOT).rglob("**/*"):
    if not (f.is_file() or f.is_symlink()):
      continue

    rf = str(f.relative_to(ROOT))
    blacklisted = any(re.search(p, rf) for p in blacklist)
    whitelisted = any(re.search(p, rf) for p in whitelist)
    if blacklisted and not whitelisted:
      continue

    print(rf)
