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

  # PNW: the 1.7 GB big_driving_supercombo.onnx is USB-GPU-only — the comma 3X never reads its
  # content (modeld only builds the big pkl when UsbGpuPresent), so .lfsconfig sets
  # `fetchexclude = *big_driving_supercombo.onnx` to cut ~95% of install/update download bytes.
  # Consequence: `git lfs pull` deliberately leaves it as a ~135 B POINTER, and build_stripped.sh's
  # `git lfs ls-files` guard then aborts the release with "LFS files detected!". Shipping a pointer
  # file masquerading as a model would be worse than omitting it, so exclude it from the release
  # tree outright. Anyone who genuinely needs it on a USB-GPU host pulls it explicitly:
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
