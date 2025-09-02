import os
import py_trees
from ament_index_python.packages import get_package_share_directory
from fault_detector_msgs.msg import StringArray
from fault_detector_spot.behaviour_tree.QOS_PROFILES import LATCHED_QOS

class PublishInitialUIInfoOnce(py_trees.behaviour.Behaviour):
    """
    Publishes all existing maps in the recordings folder once at startup, then always returns SUCCESS.
    """

    def __init__(self, name: str = "PublishInitialMapListOnce"):
        super().__init__(name)
        self.recordings_dir = os.path.join(
            get_package_share_directory("fault_detector_spot"),
            "maps"
        )
        self._has_run = False

    def setup(self, **kwargs):
        node = kwargs.get("node")
        if node is None:
            raise RuntimeError("Node must be passed to setup() for ROS publishing")
        self.publisher = node.create_publisher(StringArray, "map_list", LATCHED_QOS)

    def update(self) -> py_trees.common.Status:
        if not self._has_run:
            self.publish_map_list()
            self._has_run = True
        return py_trees.common.Status.SUCCESS

    def publish_map_list(self):
        if self.publisher is None:
            return

        map_files = []
        if os.path.isdir(self.recordings_dir):
            for f in sorted(os.listdir(self.recordings_dir)):
                if f.endswith(".db"):
                    map_files.append(f[:-3])

        msg = StringArray()
        msg.names = map_files
        self.publisher.publish(msg)
