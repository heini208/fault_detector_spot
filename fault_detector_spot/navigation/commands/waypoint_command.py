from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped

from fault_detector_spot.application.commanding.simple_command import (
    SimpleCommand,
)


class WaypointCommand(SimpleCommand):
    """Execution command for resolving and navigating to a saved waypoint."""

    def __init__(
        self,
        command_id: str,
        stamp: Time,
        map_name: str,
        waypoint_name: str,
    ):
        super().__init__(command_id, stamp)
        self.map_name = map_name
        self.waypoint_name = waypoint_name
        self.goal_pose: PoseStamped | None = None
