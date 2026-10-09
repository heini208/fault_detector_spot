import py_trees

from fault_detector_spot.mapping.runtime.rtabmap_runtime_manager import RtabmapRuntimeManager


class EnableLocalization(py_trees.behaviour.Behaviour):
    """Start RTAB-Map localization without blocking the BT executor."""

    def __init__(
        self,
        rtabmap_runtime: RtabmapRuntimeManager,
        name: str = "EnableLocalization",
    ):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.rtabmap_runtime = rtabmap_runtime
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
        self._launch_map_name = ""

    def initialise(self):
        # This leaf is reused across queued commands. A preempted worker is
        # drained by the runtime before accepting this invocation's target.
        self._launch_requested = False
        self._launch_map_name = ""

    def _requested_map(self):
        command = getattr(self.blackboard, "last_command", None)
        return getattr(command, "map_name", "").strip()

    def _start_localization(self, map_name):
        # The runtime owns the complete stop/start transition. Changing maps
        # separately would first launch the target in the previous mode.
        process = self.rtabmap_runtime.start_localization(map_name)
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
                started = self.rtabmap_runtime.begin_runtime_operation(
                    self._operation_name,
                    self._start_localization,
                    requested_map or current_map,
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
            self._launch_map_name = requested_map or current_map
            self.feedback_message = "Launching Localization"
            return py_trees.common.Status.RUNNING

        try:
            result = self.rtabmap_runtime.poll_runtime_operation(
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
        if self.blackboard.active_map_name != self._launch_map_name:
            self.feedback_message = (
                f"Localization completed without selecting '{self._launch_map_name}'"
            )
            return py_trees.common.Status.FAILURE
        if self.rtabmap_runtime.is_localization_running():
            self.feedback_message = "Localization enabled"
            return py_trees.common.Status.SUCCESS

        if not self.rtabmap_runtime.is_running():
            self.feedback_message = (
                "RTAB-Map stopped before localization became active"
            )
        elif not self.rtabmap_runtime.nav2_runtime.is_running():
            self.feedback_message = (
                "Nav2 stopped before localization became active"
            )
        else:
            self.feedback_message = (
                "Localization runtime did not reach localization mode"
            )
        return py_trees.common.Status.FAILURE
