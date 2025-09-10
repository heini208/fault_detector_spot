import typing
import py_trees
from fault_detector_spot.behaviour_tree.nodes.mapping.slam_toolbox_helper import SlamToolboxHelper


class StopMapping(py_trees.behaviour.Behaviour):
    """
    Stops the currently running mapping/localization process and waits
    until it is fully terminated.
    """

    def __init__(self, helper: SlamToolboxHelper, name="StopMapping", with_save: bool = True):
        super().__init__(name)
        self._stop_called = False
        self.helper = helper
        self.with_save = with_save

    def update(self) -> py_trees.common.Status:
        if not self.with_save:
            self.helper.stop_without_save()

        # Only attempt to stop once
        elif not self._stop_called:
            self.helper.stop_current_process()
            self._stop_called = True
            self.feedback_message = "Stop requested, waiting for Slam Toolbox to terminate"

        # Check if Slam Toolbox is still running
        if self.helper.is_slam_running():
            return py_trees.common.Status.RUNNING  # Keep behaviour running
        else:
            self.feedback_message = "Slam Toolbox stopped successfully"
            self._stop_called = False
            return py_trees.common.Status.SUCCESS
