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
        if not (self.helper.is_slam_running() or self.helper.nav2_helper.is_running()):
            self.feedback_message = "Slam Toolbox stopped successfully"
            self._stop_called = False
            return py_trees.common.Status.SUCCESS
        elif not self._stop_called:
            if not self.with_save:
                self.helper.stop_without_save()
                return py_trees.common.Status.SUCCESS #dont wait when not saving
            else:
                self.helper.stop_current_process()
            self._stop_called = True
            self.feedback_message = "Stop requested, waiting for Slam Toolbox to terminate"

        return py_trees.common.Status.RUNNING  # Keep behaviour running


