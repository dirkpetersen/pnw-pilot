# mapd2pnw: the mapd binary is no longer vendored in git. It is downloaded at
# launch by system/mapd/installer.py to the path pinned in mapd_release.json
# (selfdrive/mapd). MAPD_PATH is re-exported from there so existing importers
# (process_config, mapd_manager) keep resolving to wherever the binary lives.
from openpilot.system.mapd.installer import MAPD_BINARY as MAPD_PATH

__all__ = ["MAPD_PATH"]
