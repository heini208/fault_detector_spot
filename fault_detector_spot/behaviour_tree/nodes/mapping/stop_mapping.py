import py_trees
import signal
import os
from py_trees.blackboard import Blackboard


class StopMapping(py_trees.behaviour.Behaviour):
    """
    Stops the currently running mapping/localization process if one is active.
    """

    def __init__(self, name="StopMapping"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key("mapping_launch_process", access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        proc = getattr(self.blackboard, "mapping_launch_process", None)

        if proc is None:
            self.feedback_message = "No mapping process to stop"
            return py_trees.common.Status.SUCCESS

        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            self.blackboard.mapping_launch_process = None
            self.feedback_message = "Stopped mapping/localization launch"
            return py_trees.common.Status.SUCCESS
        except Exception as e:
            self.feedback_message = f"Failed to stop mapping: {e}"
            return py_trees.common.Status.FAILURE
