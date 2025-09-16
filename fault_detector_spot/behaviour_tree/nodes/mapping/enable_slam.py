import typing
import py_trees

from fault_detector_spot.behaviour_tree.nodes.mapping.slam_toolbox_helper import SlamToolboxHelper
from nav_msgs.msg import OccupancyGrid


class EnableSLAM(py_trees.behaviour.Behaviour):
    def __init__(self, slam_helper: SlamToolboxHelper, name: str = "EnableSLAM"):
        super().__init__(name)
        self.slam_helper = slam_helper
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key("active_map_name", access=py_trees.common.Access.READ)

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
        # only set to True if we’re in the launched state
        if self.launched_initialized:
            self.map_received_after_launch = True

    def update(self) -> py_trees.common.Status:
        if not self.blackboard.active_map_name:
            self.feedback_message = "No active map set, cannot enable SLAM"
            return py_trees.common.Status.FAILURE

        if not self.launched_initialized:
            self.feedback_message = "Launching Mapping"
            self.launched_initialized = True
            self.map_received_after_launch = False
            self.slam_helper.start_mapping_from_existing()
            return py_trees.common.Status.RUNNING
        elif self.slam_helper.is_slam_running() and self.map_received_after_launch:
            self.feedback_message = "Mapping enabled (SLAM running)"
            self.launched_initialized = False
            return py_trees.common.Status.SUCCESS

        self.feedback_message = "Waiting for map updates..."
        return py_trees.common.Status.RUNNING
