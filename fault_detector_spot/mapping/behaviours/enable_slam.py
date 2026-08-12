import py_trees

from fault_detector_spot.mapping.runtime.rtab_helper import RTABHelper


class EnableSLAM(py_trees.behaviour.Behaviour):
    """Start RTAB-Map mapping without blocking the BT executor."""

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
        self._operation_name = f"enable_slam:{name}"
        self._launch_requested = False

    def _requested_map(self):
        command = getattr(self.blackboard, "last_command", None)
        return getattr(command, "map_name", "").strip()

    def _start_mapping(self, requested_map, current_map):
        if requested_map and requested_map != current_map:
            if self.slam_helper.change_map(requested_map) is False:
                raise RuntimeError(
                    "Could not synchronize the selected map"
                )
        process = self.slam_helper.start_mapping_from_existing()
        if process is None:
            raise RuntimeError(
                "Mapping launch did not return a process"
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
                    "No active map set, cannot enable SLAM"
                )
                return py_trees.common.Status.FAILURE

            try:
                started = self.slam_helper.begin_runtime_operation(
                    self._operation_name,
                    self._start_mapping,
                    requested_map,
                    current_map,
                )
            except Exception as exception:
                self.feedback_message = (
                    f"Failed to launch mapping: {exception}"
                )
                return py_trees.common.Status.FAILURE

            if not started:
                self.feedback_message = (
                    "Waiting for another mapping runtime operation to finish"
                )
                return py_trees.common.Status.RUNNING

            self._launch_requested = True
            self.feedback_message = "Launching Mapping"
            return py_trees.common.Status.RUNNING

        try:
            result = self.slam_helper.poll_runtime_operation(
                self._operation_name
            )
        except Exception as exception:
            self._launch_requested = False
            self.feedback_message = (
                f"Failed to launch mapping: {exception}"
            )
            return py_trees.common.Status.FAILURE

        if result is None:
            self.feedback_message = "Launching Mapping"
            return py_trees.common.Status.RUNNING

        self._launch_requested = False
        if self.slam_helper.is_mapping_running():
            self.feedback_message = "Mapping enabled"
            return py_trees.common.Status.SUCCESS

        self.feedback_message = (
            "Mapping launch process stopped before becoming active"
        )
        return py_trees.common.Status.FAILURE
