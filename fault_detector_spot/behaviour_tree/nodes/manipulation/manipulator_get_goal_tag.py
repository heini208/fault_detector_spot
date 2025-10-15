from synchros2.tf_listener_wrapper import TFListenerWrapper

import py_trees
import rclpy
from fault_detector_spot.behaviour_tree.commands.manipulator_tag_command import ManipulatorTagCommand
from rclpy.duration import Duration


class ManipulatorGetGoalTag(py_trees.behaviour.Behaviour):
    """
    Checks if a goal tag ID has been set by the UI and if the tag is currently visible.
    If visible, makes the tag position available for subsequent arm control nodes.
    """

    def __init__(self, name: str = "ManipulatorGetGoalTag"):
        super(ManipulatorGetGoalTag, self).__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.tf_listener: TFListenerWrapper = None

    def setup(self, **kwargs):
        """Register necessary blackboard variables."""
        self.blackboard.register_key(
            key="reachable_tags", access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key="last_command", access=py_trees.common.Access.WRITE
        )
        self.node = kwargs.get("node")
        self.tf_listener = TFListenerWrapper(self.node)

    def _check_tranform_exists(self, command: ManipulatorTagCommand) -> bool:
        if not hasattr(self, "_tf_wait_start_time"):
            self._tf_wait_start_time = self.node.get_clock().now()

        if not self.tf_listener._tf_buffer.can_transform(
                command.goal_pose.header.frame_id,
                command.offset.header.frame_id,
                rclpy.time.Time()):
            elapsed = self.node.get_clock().now() - self._tf_wait_start_time
            if elapsed > Duration(seconds=5.0):
                self.feedback_message = "TF →hand timeout (5s)"
                del self._tf_wait_start_time  # reset for next run
                return py_trees.common.Status.FAILURE
            self.feedback_message = "Waiting for TF →hand"
            return py_trees.common.Status.RUNNING

            # Success — transform available, clear timer
        if hasattr(self, "_tf_wait_start_time"):
            del self._tf_wait_start_time
        return py_trees.common.Status.SUCCESS

    def update(self) -> py_trees.common.Status:
        """
        Check if a goal tag ID is set and visible, then store its pose.
        Returns SUCCESS if tag is found, FAILURE otherwise.
        """
        # Check if goal_tag_id is set
        if not self.blackboard.exists("last_command") or self.blackboard.last_command is None:
            self.feedback_message = "No goal set"
            return py_trees.common.Status.FAILURE

        if not isinstance(self.blackboard.last_command, ManipulatorTagCommand):
            self.feedback_message = f"Expected ManipulatorTagCommand on blackboard.last_command, got {type(self.blackboard.last_command).__name__}"
            return py_trees.common.Status.FAILURE

        command: ManipulatorTagCommand = self.blackboard.last_command
        goal_id = command.tag_id

        # Check if the tag is visible
        if not self.blackboard.exists("reachable_tags") or not self.blackboard.reachable_tags:
            self.feedback_message = "No reachable tags available"
            return py_trees.common.Status.FAILURE

        reachable_tags = self.blackboard.reachable_tags

        if goal_id not in reachable_tags:
            self.feedback_message = f"Goal tag {goal_id} is not currently visible"
            return py_trees.common.Status.FAILURE

        transform_result = self._check_tranform_exists(command)
        if transform_result != py_trees.common.Status.SUCCESS:
            return transform_result

        command.goal_pose = reachable_tags[goal_id].pose
        # Tag is visible, convert to manipulator goal command for action
        self.blackboard.last_command = command.get_as_manipulator_move_command_with_offset(self.tf_listener)
        self.feedback_message = f"Found goal tag {goal_id}"
        return py_trees.common.Status.SUCCESS