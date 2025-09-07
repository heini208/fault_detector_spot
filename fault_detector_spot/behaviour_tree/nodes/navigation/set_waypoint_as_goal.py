import os
import json
import py_trees
from fault_detector_msgs.msg import ComplexCommand
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory

class SetWaypointAsGoal(py_trees.behaviour.Behaviour):
    """
    Reads a waypoint name from last_command and sets last_command.goal_pose
    to the corresponding PoseStamped from the map's JSON file.
    """

    def __init__(self, name: str = "SetWaypointAsGoal"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client()
        self.json_path = None

    def setup(self, **kwargs):
        """
        Registers blackboard keys and sets up paths.
        Expects 'node' in kwargs.
        """
        self.node = kwargs.get("node")
        if self.node is None:
            raise RuntimeError("Setup requires a ROS node passed as 'node' kwarg")

        self.blackboard.register_key("last_command", access=py_trees.common.Access.WRITE)

        # Recordings directory
        self.recordings_dir = os.path.join(
            get_package_share_directory("fault_detector_spot"), "maps"
        )

    def update(self) -> py_trees.common.Status:
        # Check if last_command exists
        if not self.blackboard.exists("last_command") or self.blackboard.last_command is None:
            self.feedback_message = "No last_command on blackboard"
            return py_trees.common.Status.FAILURE

        last_command : ComplexCommand = self.blackboard.last_command

        if not last_command.waypoint_name or not last_command.map_name:
            self.feedback_message = "No waypoint_name or map_name in last_command"
            return py_trees.common.Status.FAILURE

        # Load waypoint JSON
        json_path = os.path.join(self.recordings_dir, f"{last_command.map_name}.json")
        if not os.path.exists(json_path):
            self.feedback_message = f"Map JSON not found: {json_path}"
            return py_trees.common.Status.FAILURE

        with open(json_path, "r") as f:
            data = json.load(f)

        waypoints = data.get("waypoints", [])
        wp_data = next((wp for wp in waypoints if wp.get("name") == last_command.waypoint_name), None)

        if wp_data is None:
            self.feedback_message = f"Waypoint '{last_command.waypoint_name}' not found in map '{last_command.map_name}'"
            return py_trees.common.Status.FAILURE

        # Convert to PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.node.get_clock().now().to_msg()
        pose_msg.pose.position.x = wp_data["pose"]["position"]["x"]
        pose_msg.pose.position.y = wp_data["pose"]["position"]["y"]
        pose_msg.pose.position.z = wp_data["pose"]["position"]["z"]
        pose_msg.pose.orientation.x = wp_data["pose"]["orientation"]["x"]
        pose_msg.pose.orientation.y = wp_data["pose"]["orientation"]["y"]
        pose_msg.pose.orientation.z = wp_data["pose"]["orientation"]["z"]
        pose_msg.pose.orientation.w = wp_data["pose"]["orientation"]["w"]

        # Save to last_command.goal_pose
        last_command.goal_pose = pose_msg
        self.feedback_message = f"Set goal_pose to waypoint '{last_command.waypoint_name}'"
        return py_trees.common.Status.SUCCESS
