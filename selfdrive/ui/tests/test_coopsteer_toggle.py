"""coopsteer2pnw: the DisableCoopSteer settings toggle (opt-out: default off = feature on).

Constructing TogglesLayout needs a raylib window (fonts), so these pin the source instead: the toggle
exists as a plain toggle_item (a description on a toggle is fine -- the ListItem trap is action
BUTTONS), needs no restart (controlsd re-reads the param at ~1 Hz), is grayed on a car without
PnwVehicle.coop_steer, and is DISPLAY-ONLY there (never written from the UI: one device, two cars).
"""
import ast
import inspect
import textwrap

from openpilot.selfdrive.ui.layouts.settings import toggles as T


def _update_src():
  return textwrap.dedent(inspect.getsource(T.TogglesLayout._update_toggles))


def test_description_exists_and_names_the_car():
  d = T.DESCRIPTIONS["DisableCoopSteer"]
  assert "Tesla" in d and "Lightning" in d and "ON by default" in d and "Turn this ON to disable" in d


def test_toggle_def_needs_no_restart():
  """Mutation: needs_restart True -> the toggle grays while engaged and flipping it restarts openpilot."""
  init = inspect.getsource(T.TogglesLayout.__init__)
  i = init.index('"DisableCoopSteer": (')
  block = init[i:init.index("\n      ),", i)]
  assert 'DESCRIPTIONS["DisableCoopSteer"]' in block
  assert block.rstrip().rstrip(",").endswith("False")


def test_grayed_by_capability_and_display_only():
  """Mutation: set_enabled(True), or a put_bool/set_state on CoopSteer in _update_toggles."""
  tree = ast.parse(_update_src())
  calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
           and "DisableCoopSteer" in ast.unparse(n.func.value)]
  assert [ast.unparse(c) for c in calls if c.func.attr == "set_enabled"] == \
         ["self._toggles['DisableCoopSteer'].action_item.set_enabled(veh.coop_steer)"]
  assert not [c for c in calls if c.func.attr in ("set_state", "put_bool")]
  assert "put_bool(\"DisableCoopSteer\"" not in inspect.getsource(T) and "put_bool('DisableCoopSteer'" not in inspect.getsource(T)
