#!/usr/bin/env python3
"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

mapd2xnor: trimmed installer. The pfeiferj mapd binary is BUNDLED in the branch
(third_party/mapd_pfeiferj/mapd, aarch64), so no runtime download is performed.
This module only tracks the bundled version. To bump the binary, replace the
file in third_party/mapd_pfeiferj/ and update VERSION here.
"""
from openpilot.common.params import Params

VERSION = "v1.12.0"
URL = f"https://github.com/pfeiferj/openpilot-mapd/releases/download/{VERSION}/mapd"


def update_installed_version(version: str, params: Params = None) -> None:
  if params is None:
    params = Params()

  params.put("MapdVersion", version)
