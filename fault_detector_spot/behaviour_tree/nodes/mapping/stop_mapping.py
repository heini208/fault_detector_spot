import typing

import py_trees
import signal
import os

from fault_detector_spot.behaviour_tree.nodes.mapping.rtab_helper import RTABHelper
from py_trees.blackboard import Blackboard


class StopMapping(py_trees.behaviour.Behaviour):
    """
    Stops the currently running mapping/localization process if one is active.
    """

    def __init__(self, name="StopMapping"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)

    def setup(self, **kwargs: typing.Any) -> None:
        self.helper = RTABHelper(kwargs.get("node"), self.blackboard)

    def update(self) -> py_trees.common.Status:
        try:
            self.helper.stop_current_process()
            return py_trees.common.Status.SUCCESS
        except Exception as e:
            self.feedback_message = f"Failed to stop mapping: {e}"
            return py_trees.common.Status.FAILURE
