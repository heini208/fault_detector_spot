import py_trees
from fault_detector_spot.behaviour_tree.commands.manipulator_to_tag_command import ManipulatorToTagCommand


# Removed unused TFListenerWrapper import

class ManipulatorGetGoalTag(py_trees.behaviour.Behaviour):
    """
    Checks if a goal tag ID is set and visible.
    Updates the command with the tag's current pose.
    """

    def __init__(self, name: str = "ManipulatorGetGoalTag"):
        super(ManipulatorGetGoalTag, self).__init__(name)
        self.blackboard = self.attach_blackboard_client()
    def setup(self, **kwargs):
        self.blackboard.register_key(key="reachable_tags", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="last_command", access=py_trees.common.Access.WRITE)
        self.node = kwargs.get("node")

    def update(self) -> py_trees.common.Status:
        if not self.blackboard.exists("last_command") or self.blackboard.last_command is None:
            self.feedback_message = "No goal set"
            return py_trees.common.Status.FAILURE

        if not isinstance(self.blackboard.last_command, ManipulatorToTagCommand):
            self.feedback_message = f"Expected ManipulatorToTagCommand"
            return py_trees.common.Status.FAILURE

        command: ManipulatorToTagCommand = self.blackboard.last_command
        goal_id = command.tag_id

        if not self.blackboard.exists("reachable_tags") or not self.blackboard.reachable_tags:
            self.feedback_message = "No reachable tags available"
            return py_trees.common.Status.FAILURE

        reachable_tags = self.blackboard.reachable_tags

        if goal_id not in reachable_tags:
            self.feedback_message = f"Goal tag {goal_id} is not currently visible"
            return py_trees.common.Status.FAILURE

        # Update command with tag pose
        command.initial_goal_pose = reachable_tags[goal_id].pose

        self.feedback_message = f"Found goal tag {goal_id}"
        return py_trees.common.Status.SUCCESS