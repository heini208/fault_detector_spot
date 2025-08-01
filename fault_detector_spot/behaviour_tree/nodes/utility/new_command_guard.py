import py_trees
from rclpy.time import Time


class NewCommandGuard(py_trees.behaviour.Behaviour):
    """
    Leaf behaviour that succeeds exactly once per new command_stamp,
    and fails otherwise.
    """

    def __init__(self, name: str = "NewCommandGuard"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()

    def setup(self, **kwargs):
        # Register read/write keys
        self.blackboard.register_key(
            key="last_command", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="last_processed_command", access=py_trees.common.Access.WRITE
        )

        self.blackboard.last_processed_command = None

    def update(self) -> py_trees.common.Status:
        if not self.stamp_exists():
            self.feedback_message = "No command received yet"
            return py_trees.common.Status.FAILURE

        if self.is_first_stamp():
            self.feedback_message = "First command – processing"
            return py_trees.common.Status.SUCCESS

        # 3) Compare timestamps
        if self.has_newer_stamp():
            self.feedback_message = "New command – processing"
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = "Duplicate command – skipping"
            return py_trees.common.Status.FAILURE

    def stamp_exists(self):
        if not self.blackboard.exists("last_command") or \
                self.blackboard.last_command is None:
            return False
        return True

    def is_first_stamp(self):
        if self.blackboard.last_command is None:
            return False
        if self.blackboard.last_processed_command is None or \
                self.blackboard.last_processed_command.stamp is None:
            self.blackboard.last_processed_command = self.blackboard.last_command
            return True
        return False

    def has_newer_stamp(self) -> bool:
        """
        Compare two timestamps for equality.
        """
        previous = self.blackboard.last_processed_command.stamp
        current = self.blackboard.last_command.stamp
        if (current.sec, current.nanosec) > (previous.sec, previous.nanosec):
            self.blackboard.last_processed_command = self.blackboard.last_command
            return True
        else:
            return False
