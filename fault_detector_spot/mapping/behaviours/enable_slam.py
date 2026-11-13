import py_trees

from fault_detector_spot.mapping.runtime.rtabmap_runtime_manager import RtabmapRuntimeManager


class EnableSLAM(py_trees.behaviour.Behaviour):
    """Start RTAB-Map mapping without blocking the BT executor."""

    def __init__(self, rtabmap_runtime: RtabmapRuntimeManager, name: str = "EnableSLAM"):
        super().__init__(name)
        self.rtabmap_runtime = rtabmap_runtime
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
        self._launch_map_name = ""

    def initialise(self):
        # Reusing the leaf must submit this invocation's target after the
        # runtime drains any preempted worker, rather than poll its result.
        self._launch_requested = False
        self._launch_map_name = ""

    def _requested_map(self):
        command = getattr(self.blackboard, "last_command", None)
        return getattr(command, "map_name", "").strip()

    def _start_mapping(self, map_name, rviz):
        # The runtime owns the map/mode transition. Passing the target directly
        # avoids relying on an active-map snapshot taken before worker admission.
        process = self.rtabmap_runtime.start_mapping(map_name, rviz=rviz)
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
                started = self.rtabmap_runtime.begin_runtime_operation(
                    self._operation_name,
                    self._start_mapping,
                    requested_map or current_map,
                    getattr(getattr(self.blackboard, "last_command", None), "mapping_rviz", True),
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
            self._launch_map_name = requested_map or current_map
            self.feedback_message = "Launching Mapping"
            return py_trees.common.Status.RUNNING

        try:
            result = self.rtabmap_runtime.poll_runtime_operation(
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
        if self.blackboard.active_map_name != self._launch_map_name:
            self.feedback_message = (
                f"Mapping completed without selecting '{self._launch_map_name}'"
            )
            return py_trees.common.Status.FAILURE
        if self.rtabmap_runtime.is_mapping_running():
            self.feedback_message = "Mapping enabled"
            return py_trees.common.Status.SUCCESS

        self.feedback_message = (
            "Mapping launch process stopped before becoming active"
        )
        return py_trees.common.Status.FAILURE
