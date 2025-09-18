import re

import py_trees
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped


class SetTagAsGoal(py_trees.behaviour.Behaviour):
    """
    Looks up a tag from blackboard.last_command, transforms its pose into the map frame
    using TF, and writes it directly to blackboard.last_command.goal_pose (PoseStamped).
    """

    def __init__(self, name: str = "SetTagAsGoal", frame_pattern: str = r"tag_(\d+)"):
        super().__init__(name)
        self.node: rclpy.node.Node | None = None
        self.tf_buffer: tf2_ros.Buffer | None = None
        self.tf_listener: tf2_ros.TransformListener | None = None
        self.blackboard = self.attach_blackboard_client()
        self.frame_pattern = re.compile(frame_pattern)

    def setup(self, **kwargs):
        """Initialize node and TF listener, register blackboard keys."""
        self.node = kwargs.get("node")
        if not self.node:
            raise RuntimeError("SetTagAsGoal requires a ROS node in setup()")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        self.blackboard.register_key("last_command", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("visible_tags_map_frame", access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        # No command set
        if not self.blackboard.exists("last_command") or self.blackboard.last_command is None:
            self.feedback_message = "No last_command set"
            return py_trees.common.Status.FAILURE

        goal_id = getattr(self.blackboard.last_command, "tag_id", None)
        if goal_id is None:
            self.feedback_message = "No tag_id in last_command"
            return py_trees.common.Status.FAILURE

        if not self.blackboard.exists("visible_tags_map_frame") or self.blackboard.visible_tags_map_frame is None:
            self.feedback_message = "No tags in map frame"
            return py_trees.common.Status.FAILURE

        # Search TF for the goal tag

        tag_element = self.blackboard.visible_tags_map_frame.get(goal_id)
        if tag_element is None:
            self.feedback_message = f"Tag {goal_id} not visible in map frame"
            return py_trees.common.Status.FAILURE

        # Use its PoseStamped directly
        tag_pose = PoseStamped()
        tag_pose.header = tag_element.pose.header
        tag_pose.pose = tag_element.pose.pose

        # Write directly to last_command.goal_pose
        self.blackboard.last_command.goal_pose = tag_pose
        self.feedback_message = f"Set goal from tag {goal_id} in map frame"
        return py_trees.common.Status.SUCCESS