import py_trees
from fault_detector_spot.mapping.runtime.rtab_helper import RTABHelper


class EnableLocalization(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        slam_helper: RTABHelper,
        name: str = "EnableLocalization",
    ):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.slam_helper = slam_helper
        self.blackboard.register_key(
            "active_map_name",
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            "last_command",
            access=py_trees.common.Access.READ,
        )
        self.launched_initialized = False

    def _requested_map(self):
        command = getattr(self.blackboard, "last_command", None)
        return getattr(command, "map_name", "").strip()

    def _synchronize_requested_map(self):
        requested_map = self._requested_map()
        if not requested_map:
            return True

        current_map = getattr(
            self.blackboard,
            "active_map_name",
            None,
        )
        if current_map == requested_map:
            return True

        return self.slam_helper.change_map(requested_map) is not False

    def update(self) -> py_trees.common.Status:
        if not self.launched_initialized:
            try:
                if not self._synchronize_requested_map():
                    self.feedback_message = (
                        "Could not synchronize the selected map"
                    )
                    return py_trees.common.Status.FAILURE
            except Exception as exception:
                self.feedback_message = (
                    f"Could not synchronize the selected map: {exception}"
                )
                return py_trees.common.Status.FAILURE

        if not self.blackboard.active_map_name:
            self.feedback_message = (
                "No active map set, cannot enable Localization"
            )
            return py_trees.common.Status.FAILURE

        self.feedback_message = "Launching Localization"
        if not self.launched_initialized:
            self.launched_initialized = True
            try:
                self.slam_helper.start_localization()
            except Exception as exception:
                self.launched_initialized = False
                self.feedback_message = (
                    f"Failed to launch localization: {exception}"
                )
                return py_trees.common.Status.FAILURE

        if self.slam_helper.nav2_helper.is_running():
            self.launched_initialized = False
            self.feedback_message = "Localization enabled"
            return py_trees.common.Status.SUCCESS

        self.feedback_message = "Waiting for Nav2..."
        return py_trees.common.Status.RUNNING
