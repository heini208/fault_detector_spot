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
        self.current = None

    def setup(self, **kwargs):
        # Register read/write keys
        self.blackboard.register_key(
            key="last_command_stamp", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="last_processed_command_stamp", access=py_trees.common.Access.WRITE
        )

        self.blackboard.last_processed_command_stamp = None


    def update(self) -> py_trees.common.Status:
        if not self.stamp_exists():
            self.feedback_message = "No command received yet"
            return py_trees.common.Status.FAILURE

        self.current = self.blackboard.last_command_stamp

        if not self.is_first_stamp():
            self.feedback_message = "First command – processing"
            return py_trees.common.Status.SUCCESS

        # 3) Compare timestamps
        if self.same_time_stamp(self.current, self.blackboard.last_processed_command_stamp):
            self.feedback_message = "Duplicate command – skipping"
            return py_trees.common.Status.FAILURE
        else:
            self.feedback_message = "New command – processing"
            return py_trees.common.Status.SUCCESS

    def stamp_exists(self):
        if not self.blackboard.exists("last_command_stamp") or \
           self.blackboard.last_command_stamp is None:
            return False
        return True

    def is_first_stamp(self):
        if self.current is None:
            return False
        if not self.blackboard.exists("last_processed_command_stamp") or \
           self.blackboard.last_processed_command_stamp is None:
            self.blackboard.last_processed_command_stamp = self.current
            return True

    def same_time_stamp(self, current: Time, previous: Time) -> bool:
        """
        Compare two timestamps for equality.
        """
        if (current.sec != previous.sec) or (current.nanosec != previous.nanosec):
            self.blackboard.last_processed_command_stamp = current
            return False
        else:
            return True

