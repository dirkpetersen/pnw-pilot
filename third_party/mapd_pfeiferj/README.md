# MAPD implementation by pfeiferj

Upstream: https://github.com/pfeiferj/mapd/releases/

The `mapd` binary is **not vendored** in this repo (it is ~20 MB). It is pinned in
[`mapd_release.json`](../../mapd_release.json) at the repo root and downloaded at
launch by [`system/mapd/installer.py`](../../system/mapd/installer.py), which
verifies the release sha256 and installs it to `selfdrive/mapd` (gitignored).

To upgrade mapd, edit `mapd_release.json` (version + url + sha256 + size).
