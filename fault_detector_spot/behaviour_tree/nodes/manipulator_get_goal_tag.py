import py_trees
from geometry_msgs.msg import PoseStamped
from typing import Optional


class ManipulatorGetGoalTag(py_trees.behaviour.Behaviour):
    """
    Checks if a goal tag ID has been set by the UI and if the tag is currently visible.
    If visible, makes the tag position available for subsequent arm control nodes.
    """

    def __init__(self, name: str = "ManipulatorGetGoalTag"):
        super(ManipulatorGetGoalTag, self).__init__(name)
        self.blackboard = self.attach_blackboard_client()

    def setup(self, **kwargs):
        """Register necessary blackboard variables."""
        self.blackboard.register_key(
            key="visible_tags", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="goal_tag_id", access=py_trees.common.Access.WRITE
        )
        self.blackboard.register_key(
            key="goal_tag_pose", access=py_trees.common.Access.WRITE
        )

    def update(self) -> py_trees.common.Status:
        """
        Check if a goal tag ID is set and visible, then store its pose.
        Returns SUCCESS if tag is found, FAILURE otherwise.
        """
        # Check if goal_tag_id is set
        if not self.blackboard.exists("goal_tag_id") or self.blackboard.goal_tag_id is None:
            self.feedback_message = "No goal tag ID set"
            return py_trees.common.Status.FAILURE

        goal_id = self.blackboard.goal_tag_id

        # Check if the tag is visible
        if not self.blackboard.exists("visible_tags") or not self.blackboard.visible_tags:
            self.feedback_message = "No visible tags available"
            return py_trees.common.Status.FAILURE

        visible_tags = self.blackboard.visible_tags

        if goal_id not in visible_tags:
            self.feedback_message = f"Goal tag {goal_id} is not currently visible"
            return py_trees.common.Status.FAILURE

        # Tag is visible, store its pose for the arm controller
        self.blackboard.goal_tag_pose = visible_tags[goal_id].pose
        self.feedback_message = f"Found goal tag {goal_id}"
        if self.blackboard.exists("goal_tag_id"):
            self.blackboard.goal_tag_id = None
        return py_trees.common.Status.SUCCESS