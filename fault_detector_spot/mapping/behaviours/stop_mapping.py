import py_trees

from fault_detector_spot.mapping.runtime.rtab_helper import RTABHelper


class StopMapping(py_trees.behaviour.Behaviour):
    """Stop mapping or localization without blocking the BT executor."""

    def __init__(
        self,
        helper: RTABHelper,
        name="StopMapping",
        with_save: bool = True,
    ):
        super().__init__(name)
        self.helper = helper
        self.with_save = with_save
        self._operation_name = f"stop_mapping:{name}"
        self._stop_requested = False

    def update(self) -> py_trees.common.Status:
        if not self._stop_requested:
            if not (
                self.helper.is_rtabmap_running()
                or self.helper.nav2_helper.is_running()
            ):
                self.feedback_message = "Mapping runtime stopped"
                return py_trees.common.Status.SUCCESS

            callback = (
                self.helper.stop_current_process
                if self.with_save
                else self.helper.stop_without_save
            )
            try:
                started = self.helper.begin_runtime_operation(
                    self._operation_name,
                    callback,
                )
            except Exception as exception:
                self.feedback_message = (
                    f"Failed to request mapping stop: {exception}"
                )
                return py_trees.common.Status.FAILURE

            if not started:
                self.feedback_message = (
                    "Waiting for another mapping runtime operation to finish"
                )
                return py_trees.common.Status.RUNNING

            self._stop_requested = True
            self.feedback_message = "Stopping mapping runtime"
            return py_trees.common.Status.RUNNING

        try:
            result = self.helper.poll_runtime_operation(
                self._operation_name
            )
        except Exception as exception:
            self._stop_requested = False
            self.feedback_message = (
                f"Failed to stop mapping runtime: {exception}"
            )
            return py_trees.common.Status.FAILURE

        if result is None:
            self.feedback_message = "Stopping mapping runtime"
            return py_trees.common.Status.RUNNING

        self._stop_requested = False
        if (
            self.helper.is_rtabmap_running()
            or self.helper.nav2_helper.is_running()
        ):
            self.feedback_message = (
                "Mapping runtime stop completed but a process is still alive"
            )
            return py_trees.common.Status.FAILURE

        self.feedback_message = "Mapping runtime stopped"
        return py_trees.common.Status.SUCCESS
