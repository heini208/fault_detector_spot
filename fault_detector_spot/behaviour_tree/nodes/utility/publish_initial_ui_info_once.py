import os

import py_trees
from ament_index_python import get_package_share_directory
from fault_detector_msgs.msg import StringArray
from fault_detector_spot.behaviour_tree.QOS_PROFILES import LATCHED_QOS


class PublishInitialUIInfoOnce(py_trees.behaviour.Behaviour):
    def __init__(self, name: str = "PublishInitialMapListOnce"):
        super().__init__(name)
        self.recordings_dir = os.path.join(
            get_package_share_directory("fault_detector_spot"),
            "maps"
        )
        self._published_once = False

    def setup(self, **kwargs):
        node = kwargs.get("node")
        if node is None:
            raise RuntimeError("Node must be passed to setup() for ROS publishing")
        self.node = node
        self.publisher = node.create_publisher(StringArray, "map_list", LATCHED_QOS)

    def update(self) -> py_trees.common.Status:
        if self._published_once:
            return py_trees.common.Status.SUCCESS

        # Only publish if at least one subscriber exists
        if self.publisher.get_subscription_count() > 0:
            self.publish_map_list()
            self._published_once = True
            return py_trees.common.Status.SUCCESS
        else:
            # Keep ticking until a subscriber appears
            return py_trees.common.Status.SUCCESS

    def publish_map_list(self):
        map_files = []
        if os.path.isdir(self.recordings_dir):
            for f in sorted(os.listdir(self.recordings_dir)):
                if f.endswith(".posegraph"):
                    map_files.append(f[:-10])

        msg = StringArray()
        msg.names = map_files
        self.publisher.publish(msg)
