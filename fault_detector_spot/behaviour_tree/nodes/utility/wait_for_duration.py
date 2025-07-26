# In fault_detector_spot/behaviour_tree/nodes/wait_for_duration.py

import time
import py_trees
from py_trees.common import Status

class WaitForDuration(py_trees.behaviour.Behaviour):
    """
    Waits for `duration` seconds, where duration is read from
    blackboard.wait_duration at each new tick‑initialisation.
    Returns RUNNING until the time has elapsed, then SUCCESS.
    If wait_duration is missing or <=0, immediately returns SUCCESS.
    """

    def __init__(self, name: str = "WaitForDuration"):
        super().__init__(name)
        self.start_time: float = None

    def setup(self, **kwargs) -> bool:
        # attach to the blackboard and register the key you will read
        self.blackboard = self.attach_blackboard_client()
        self.blackboard.register_key(
            key="last_command", access=py_trees.common.Access.READ
        )

        return True

    def initialise(self) -> None:
        self.start_time = time.time()

    def update(self) -> Status:
        duration = self.blackboard.last_command.duration if hasattr(self.blackboard.last_command, 'duration') else None

        # missing or non‑positive duration? skip the wait
        if duration is None or duration <= 0.0:
            return Status.SUCCESS
        # still waiting?
        if time.time() - self.start_time < duration:
            return Status.RUNNING
        # done
        return Status.SUCCESS
