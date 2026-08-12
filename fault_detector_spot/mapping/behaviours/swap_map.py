import py_trees

from fault_detector_spot.mapping.runtime.rtab_helper import RTABHelper


class SwapMap(py_trees.behaviour.Behaviour):
    """Switch the active RTAB-Map database without blocking the BT."""

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
        self._operation_name = f"swap_map:{name}"
        self._requested_map = ""
        self._previous_mode = RTABHelper.MODE_NONE
        self._swap_requested = False

    def _validate_last_command(self):
        last_command = getattr(self.blackboard, "last_command", None)
        if not last_command or not getattr(last_command, "map_name", None):
            self.feedback_message = "No map_name in last_command"
            return None
        return last_command.map_name.strip()

    def update(self):
        if not self._swap_requested:
            requested_map = self._validate_last_command()
            if not requested_map:
                return py_trees.common.Status.FAILURE

            current_map = self.blackboard.active_map_name
            if requested_map == current_map:
                self.feedback_message = (
                    f"Map '{requested_map}' already active"
                )
                return py_trees.common.Status.SUCCESS

            try:
                previous_mode = self.slam_helper.get_running_mode()
                started = self.slam_helper.begin_runtime_operation(
                    self._operation_name,
                    self.slam_helper.change_map,
                    requested_map,
                )
            except Exception as exception:
                self.feedback_message = (
                    f"Failed to switch to map '{requested_map}': {exception}"
                )
                return py_trees.common.Status.FAILURE

            if not started:
                self.feedback_message = (
                    "Waiting for another mapping runtime operation to finish"
                )
                return py_trees.common.Status.RUNNING

            self._requested_map = requested_map
            self._previous_mode = previous_mode
            self._swap_requested = True
            self.feedback_message = (
                f"Switching active map to '{requested_map}'"
            )
            return py_trees.common.Status.RUNNING

        try:
            result = self.slam_helper.poll_runtime_operation(
                self._operation_name
            )
        except Exception as exception:
            requested_map = self._requested_map
            self._reset()
            self.feedback_message = (
                f"Failed to switch to map '{requested_map}': {exception}"
            )
            return py_trees.common.Status.FAILURE

        if result is None:
            self.feedback_message = (
                f"Switching active map to '{self._requested_map}'"
            )
            return py_trees.common.Status.RUNNING

        requested_map = self._requested_map
        previous_mode = self._previous_mode
        self._reset()

        if result is False:
            self.feedback_message = (
                f"Failed to switch to map '{requested_map}'"
            )
            return py_trees.common.Status.FAILURE

        if self.blackboard.active_map_name != requested_map:
            self.feedback_message = (
                f"Map switch completed without selecting '{requested_map}'"
            )
            return py_trees.common.Status.FAILURE

        if (
            previous_mode == RTABHelper.MODE_MAPPING
            and not self.slam_helper.is_mapping_running()
        ):
            self.feedback_message = (
                "Map switch did not restore mapping mode"
            )
            return py_trees.common.Status.FAILURE

        if (
            previous_mode == RTABHelper.MODE_LOCALIZATION
            and not self.slam_helper.is_localization_running()
        ):
            self.feedback_message = (
                "Map switch did not restore localization mode"
            )
            return py_trees.common.Status.FAILURE

        self.feedback_message = f"Active map changed to '{requested_map}'"
        return py_trees.common.Status.SUCCESS

    def _reset(self):
        self._requested_map = ""
        self._previous_mode = RTABHelper.MODE_NONE
        self._swap_requested = False
