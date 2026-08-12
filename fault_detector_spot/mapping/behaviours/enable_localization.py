import py_trees

from fault_detector_spot.mapping.runtime.rtab_helper import RTABHelper


class EnableLocalization(py_trees.behaviour.Behaviour):
    """Start RTAB-Map localization without blocking the BT executor."""

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
        self._operation_name = f"enable_localization:{name}"
        self._launch_requested = False

    def _requested_map(self):
        command = getattr(self.blackboard, "last_command", None)
        return getattr(command, "map_name", "").strip()

    def _start_localization(self, requested_map, current_map):
        if requested_map and requested_map != current_map:
            if self.slam_helper.change_map(requested_map) is False:
                raise RuntimeError(
                    "Could not synchronize the selected map"
                )
        process = self.slam_helper.start_localization()
        if process is None:
            raise RuntimeError(
                "Localization launch did not return a process"
            )
        return True

    def update(self) -> py_trees.common.Status:
        if not self._launch_requested:
            requested_map = self._requested_map()
            current_map = getattr(
                self.blackboard,
                "active_map_name",
                None,
            )
            if not requested_map and not current_map:
                self.feedback_message = (
                    "No active map set, cannot enable Localization"
                )
                return py_trees.common.Status.FAILURE

            try:
                started = self.slam_helper.begin_runtime_operation(
                    self._operation_name,
                    self._start_localization,
                    requested_map,
                    current_map,
                )
            except Exception as exception:
                self.feedback_message = (
                    f"Failed to launch localization: {exception}"
                )
                return py_trees.common.Status.FAILURE

            if not started:
                self.feedback_message = (
                    "Waiting for another mapping runtime operation to finish"
                )
                return py_trees.common.Status.RUNNING

            self._launch_requested = True
            self.feedback_message = "Launching Localization"
            return py_trees.common.Status.RUNNING

        try:
            result = self.slam_helper.poll_runtime_operation(
                self._operation_name
            )
        except Exception as exception:
            self._launch_requested = False
            self.feedback_message = (
                f"Failed to launch localization: {exception}"
            )
            return py_trees.common.Status.FAILURE

        if result is None:
            self.feedback_message = "Launching Localization"
            return py_trees.common.Status.RUNNING

        self._launch_requested = False
        if self.slam_helper.is_localization_running():
            self.feedback_message = "Localization enabled"
            return py_trees.common.Status.SUCCESS

        if not self.slam_helper.is_rtabmap_running():
            self.feedback_message = (
                "RTAB-Map stopped before localization became active"
            )
        elif not self.slam_helper.nav2_helper.is_running():
            self.feedback_message = (
                "Nav2 stopped before localization became active"
            )
        else:
            self.feedback_message = (
                "Localization runtime did not reach localization mode"
            )
        return py_trees.common.Status.FAILURE
