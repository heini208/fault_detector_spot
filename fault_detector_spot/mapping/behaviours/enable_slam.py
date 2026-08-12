import typing

import py_trees
from fault_detector_spot.mapping.runtime.rtab_helper import RTABHelper
from nav_msgs.msg import OccupancyGrid


class EnableSLAM(py_trees.behaviour.Behaviour):
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

        self.launched_initialized = False
        self.map_received_after_launch = False

    def setup(self, **kwargs: typing.Any) -> None:
        self.node = kwargs.get("node")
        self.sub = self.node.create_subscription(
            OccupancyGrid,
            "/map",
            self._map_callback,
            10,
        )

    def _map_callback(self, msg: OccupancyGrid):
        if self.launched_initialized:
            self.map_received_after_launch = True

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
            self.feedback_message = "No active map set, cannot enable SLAM"
            return py_trees.common.Status.FAILURE

        if not self.launched_initialized:
            self.feedback_message = "Launching Mapping"
            self.launched_initialized = True
            self.map_received_after_launch = False
            try:
                self.slam_helper.start_mapping_from_existing()
            except Exception as exception:
                self.launched_initialized = False
                self.feedback_message = (
                    f"Failed to launch mapping: {exception}"
                )
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        if (
            self.slam_helper.is_slam_running()
            and self.map_received_after_launch
        ):
            self.feedback_message = "Mapping enabled (SLAM running)"
            self.launched_initialized = False
            return py_trees.common.Status.SUCCESS

        self.feedback_message = "Waiting for map updates..."
        return py_trees.common.Status.RUNNING
