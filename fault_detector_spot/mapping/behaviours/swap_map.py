import py_trees
from fault_detector_spot.mapping.runtime.rtab_helper import RTABHelper


class SwapMap(py_trees.behaviour.Behaviour):
    """Switch the active RTAB-Map database without stranding the command."""

    def __init__(self, slam_helper: RTABHelper, name="SwapMap"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            "last_command",
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            "active_map_name",
            access=py_trees.common.Access.READ,
        )
        self.slam_helper = slam_helper

    def _validate_last_command(self):
        last_command = getattr(self.blackboard, "last_command", None)
        if not last_command or not getattr(last_command, "map_name", None):
            self.feedback_message = "No map_name in last_command"
            return None
        return last_command.map_name

    def update(self):
        requested_map = self._validate_last_command()
        if not requested_map:
            return py_trees.common.Status.FAILURE

        try:
            current_map = self.blackboard.active_map_name
            if requested_map == current_map:
                self.feedback_message = (
                    f"Map '{requested_map}' already active"
                )
                return py_trees.common.Status.SUCCESS

            result = self.slam_helper.change_map(requested_map)
        except Exception as exception:
            self.feedback_message = (
                f"Failed to switch to map '{requested_map}': {exception}"
            )
            return py_trees.common.Status.FAILURE

        if result is False:
            self.feedback_message = (
                f"Failed to switch to map '{requested_map}'"
            )
            return py_trees.common.Status.FAILURE

        self.feedback_message = f"Active map changed to '{requested_map}'"
        return py_trees.common.Status.SUCCESS
