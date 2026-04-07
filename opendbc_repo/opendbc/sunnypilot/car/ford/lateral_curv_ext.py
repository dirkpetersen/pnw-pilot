"""
BluePilot Ford lateral curvature extension module.

This module renames the historical `lateral_ext` path to `lateral_curv_ext`
while preserving compatibility with existing class naming.
"""

from opendbc.sunnypilot.car.ford.lateral_ext import LateralExt as LateralCurvExt

# Backward-compatible alias for any legacy imports expecting LateralExt.
LateralExt = LateralCurvExt
