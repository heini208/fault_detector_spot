import os
import json
import py_trees
from ament_index_python.packages import get_package_share_directory
from fault_detector_msgs.msg import StringArray
from fault_detector_spot.behaviour_tree.QOS_PROFILES import LATCHED_QOS
from fault_detector_spot.behaviour_tree.commands.generic_complex_command import GenericCommand

class CreateEmptyMap(py_trees.behaviour.Behaviour):
    """
    Creates an empty .db file and a corresponding JSON file for landmarks.
    The map name is expected in blackboard.last_command.map_name (GenericCommand).
    """

    def __init__(self, name: str = "CreateEmptyMap"):
        super(CreateEmptyMap, self).__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.recordings_dir = os.path.join(
            get_package_share_directory("fault_detector_spot"),
            "maps"
        )

    def setup(self, **kwargs):
        self.blackboard.register_key(
            key="last_command",
            access=py_trees.common.Access.READ
        )
        node = kwargs.get("node")
        if node is None:
            raise RuntimeError("Node must be passed to setup() for ROS publishing")
        self.publisher = node.create_publisher(StringArray, "map_list", LATCHED_QOS)

    def update(self) -> py_trees.common.Status:
        """Create the map files."""
        if not self.is_command_valid():
            return py_trees.common.Status.FAILURE

        cmd = self.blackboard.last_command
        map_name = cmd.map_name.strip()
        self.create_map_files(map_name)
        self.publish_map_list()
        return py_trees.common.Status.SUCCESS

    def create_map_files(self, map_name: str) -> bool:
        os.makedirs(self.recordings_dir, exist_ok=True)

        db_path = os.path.join(self.recordings_dir, f"{map_name}.db")
        json_path = os.path.join(self.recordings_dir, f"{map_name}.json")

        # Create empty .db if it doesn't exist
        if not os.path.exists(db_path):
            open(db_path, "a").close()

        # Create empty JSON if it doesn't exist
        if not os.path.exists(json_path):
            with open(json_path, "w") as f:
                json.dump({"landmarks": []}, f, indent=4)

        self.feedback_message = f"Created map files: {map_name}.db and {map_name}.json"

    def is_command_valid(self) -> bool:
        if not self.blackboard.exists("last_command") or self.blackboard.last_command is None:
            self.feedback_message = "No last_command on blackboard"
            return False

        cmd = self.blackboard.last_command
        if not isinstance(cmd, GenericCommand):
            self.feedback_message = f"Expected GenericCommand, got {type(cmd).__name__}"
            return False

        if not cmd.map_name or cmd.map_name.strip() == "":
            self.feedback_message = "No map_name provided in last_command"
            return False
        return True

    def publish_map_list(self):
        """Collect all .db filenames in the recordings folder and publish as StringArray."""
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