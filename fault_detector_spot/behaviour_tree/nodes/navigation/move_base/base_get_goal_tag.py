import py_trees
from fault_detector_spot.behaviour_tree.commands.base_to_tag_command import BaseToTagCommand
from synchros2.tf_listener_wrapper import TFListenerWrapper


class BaseGetGoalTag(py_trees.behaviour.Behaviour):
    """
    Reads a visible tag and prepares a base navigation goal (in odom/world frame),
    applying any offset defined in the BaseToTagCommand.
    """

    def __init__(self, name: str = "BaseGetGoalTag"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.tf_listener: TFListenerWrapper = None
        self.node = None

    def setup(self, **kwargs):
        self.node = kwargs.get("node")
        self.tf_listener = TFListenerWrapper(self.node)

        # Blackboard keys: read visible tags, read/write last command
        self.blackboard.register_key(key="visible_tags", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="last_command", access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        command = getattr(self.blackboard, "last_command", None)
        if not isinstance(command, BaseToTagCommand):
            self.feedback_message = "Expected BaseTagCommand"
            return py_trees.common.Status.FAILURE

        # Get visible tags
        visible_tags = getattr(self.blackboard, "visible_tags", {})
        if not visible_tags:
            self.feedback_message = "No visible tags"
            return py_trees.common.Status.FAILURE

        tag_id = command.tag_id
        if tag_id not in visible_tags:
            self.feedback_message = f"Tag {tag_id} not visible"
            return py_trees.common.Status.FAILURE

        # Use the visible tag's pose
        command.initial_goal_pose = visible_tags[tag_id].pose

        self.feedback_message = f"Prepared base goal for visible tag {tag_id}"
        return py_trees.common.Status.SUCCESS