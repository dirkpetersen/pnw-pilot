"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import os

from openpilot.selfdrive.ui.layouts.settings.software import SoftwareLayout
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.hardware import HARDWARE
from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.multilang import tr, tr_noop
from openpilot.system.ui.widgets import DialogResult
from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog

from openpilot.system.ui.sunnypilot.widgets.list_view import toggle_item_sp
from openpilot.system.ui.sunnypilot.widgets.tree_dialog import TreeOptionDialog, TreeNode, TreeFolder


DESCRIPTIONS = {
  # BluePilot: "Allow auto updates" — inverted DisableUpdates, OFF by default, always visible.
  'allow_updates_offroad': tr_noop(
    "When disabled, all automatic updates are blocked (OFF by default to protect on-device changes).<br><b>Requires a reboot to take effect.</b>"
  ),
  'allow_updates_onroad': tr_noop(
    "Please enable \"Always Offroad\" mode or turn off the vehicle to adjust these toggles."
  )
}


class SoftwareLayoutSP(SoftwareLayout):
  def __init__(self):
    super().__init__()
    # BluePilot: present an "Allow auto updates" toggle (inverse of DisableUpdates).
    # ON  -> DisableUpdates = False (updates allowed)
    # OFF -> DisableUpdates = True  (all updates blocked). OFF is the default.
    self.allow_updates_toggle = toggle_item_sp(
      lambda: tr("Allow auto updates"),
      description="",
      initial_state=not ui_state.params.get_bool("DisableUpdates"),
      callback=self._on_allow_updates_toggled,
    )
    self._scroller.add_widget(self.allow_updates_toggle)

  def _handle_reboot(self, result):
    if result == DialogResult.CONFIRM:
      # toggle state is "allow updates"; DisableUpdates is the inverse
      ui_state.params.put_bool("DisableUpdates", not self.allow_updates_toggle.action_item.get_state())
      ui_state.params.put_bool("DoReboot", True)
    else:
      self.allow_updates_toggle.action_item.set_state(not ui_state.params.get_bool("DisableUpdates"))

  def _on_allow_updates_toggled(self, enabled):
    dialog = ConfirmDialog(tr("System reboot required for changes to take effect. Reboot now?"), tr("Reboot"), callback=self._handle_reboot)
    gui_app.push_widget(dialog)

  def _on_select_branch(self):
    current_git_branch = ui_state.params.get("GitBranch") or ""
    branches_str = ui_state.params.get("UpdaterAvailableBranches") or ""
    branches = [b for b in branches_str.split(",") if b]
    current_target = ui_state.params.get("UpdaterTargetBranch") or ""
    top_level_branches = [current_git_branch, "release-mici", "release-tizi", "staging", "dev", "master"]

    if HARDWARE.get_device_type() == "tici":
      top_level_branches = ["release-tici", "staging-tici"]
      branches = [b for b in branches if b.endswith("-tici")]

    top_level_nodes = [TreeNode(b, {'display_name': b}) for b in top_level_branches if b in branches]
    remaining_branches = [b for b in branches if b not in top_level_branches]
    prebuilt_nodes = [TreeNode(b, {'display_name': b}) for b in remaining_branches if b.endswith("-prebuilt")]
    non_prebuilt_nodes = [TreeNode(b, {'display_name': b}) for b in remaining_branches if not b.endswith("-prebuilt")]

    folders = [
      TreeFolder("", top_level_nodes),
      TreeFolder("Prebuilt Branches", prebuilt_nodes),
      TreeFolder("Non-Prebuilt Branches", non_prebuilt_nodes),
    ]

    def _on_branch_selected(result):
      if result == DialogResult.CONFIRM and self._branch_dialog is not None:
        selection = self._branch_dialog.selection_ref
        if selection:
          ui_state.params.put("UpdaterTargetBranch", selection)
          self._branch_btn.action_item.set_value(selection)
          os.system("pkill -SIGUSR1 -f system.updated.updated")
      self._branch_dialog = None

    self._branch_dialog = TreeOptionDialog(tr("Select a branch"), folders, current_target, "",
                                           on_exit=_on_branch_selected)

    gui_app.push_widget(self._branch_dialog)

  def _update_state(self):
    super()._update_state()
    # BluePilot: always visible (not gated on ShowAdvancedControls). Only editable offroad.
    self.allow_updates_toggle.action_item.set_enabled(ui_state.is_offroad())

    allow_updates_desc = tr(DESCRIPTIONS["allow_updates_offroad"] if ui_state.is_offroad() else DESCRIPTIONS["allow_updates_onroad"])
    self.allow_updates_toggle.set_description(allow_updates_desc)
