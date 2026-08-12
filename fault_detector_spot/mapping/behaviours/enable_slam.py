import py_trees
from fault_detector_spot.mapping.runtime.rtab_helper import RTABHelper


class EnableSLAM(py_trees.behaviour.Behaviour):
    """Start RTAB-Map mapping and finish once its launch process is alive."""

    def __init__(self, slam_helper: RTABHelper, name: str = "EnableSLAM"):
        super().__init__(name)
        self.slam_helper = slam_helper
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            "active_map_name",
            access=py_trees.common.Access.READ,
        )
        self.blackboard.register_key(
            "last_command",
            access=py_trees.common.Access.READ,
        )
        self._launch_requested = False

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
        if not self._launch_requested:
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
                "No active map set, cannot enable SLAM"
            )
            return py_trees.common.Status.FAILURE

        if not self._launch_requested:
            self.feedback_message = "Launching Mapping"
            try:
                process = self.slam_helper.start_mapping_from_existing()
            except Exception as exception:
                self.feedback_message = (
                    f"Failed to launch mapping: {exception}"
                )
                return py_trees.common.Status.FAILURE

            if process is None:
                self.feedback_message = (
                    "Mapping launch did not return a process"
                )
                return py_trees.common.Status.FAILURE

            self._launch_requested = True
            return py_trees.common.Status.RUNNING

        if self.slam_helper.is_rtabmap_running():
            self._launch_requested = False
            self.feedback_message = "Mapping enabled"
            return py_trees.common.Status.SUCCESS

        self._launch_requested = False
        self.feedback_message = (
            "Mapping launch process stopped before becoming active"
        )
        return py_trees.common.Status.FAILURE
