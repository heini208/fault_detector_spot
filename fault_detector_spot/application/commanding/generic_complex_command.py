from builtin_interfaces.msg import Time
from fault_detector_msgs.msg import InspectionCommand
from geometry_msgs.msg import PoseStamped, Quaternion
from .simple_command import SimpleCommand


class GenericCommand(SimpleCommand):
    """
    A generic super command that contains every possible field
    from all command types, so it can act as a fallback container.
    """

    def __init__(
            self,
            command_id: str,
            stamp: Time,
            duration: float = None,
            goal_pose: PoseStamped = None,
            tag_id: int = None,
            offset: PoseStamped = None,
            orientation_mode: str = None,
            map_name: str = None,
            waypoint_name: str = None,
            inspection: InspectionCommand = None,
    ):
        super().__init__(command_id, stamp)
        self.duration = duration
        self.goal_pose = goal_pose
        self.tag_id = tag_id
        self.offset = offset
        self.orientation_mode = orientation_mode
        self.map_name = map_name
        self.waypoint_name = waypoint_name
        self.inspection = inspection

    def __repr__(self):
        ts = f"{self.stamp.sec}.{self.stamp.nanosec:09d}"
        parts = [f"id={self.command_id!r}", f"at={ts}"]

        if self.duration is not None:
            parts.append(f"duration={self.duration}s")
        if self.tag_id is not None:
            parts.append(f"tag_id={self.tag_id}")
        if self.orientation_mode:
            parts.append(f"orientation={self.orientation_mode}")
        if self.goal_pose:
            p = self.goal_pose.pose.position
            parts.append(f"pose=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")
        if self.inspection is not None:
            object_id = self.inspection.object.object_id
            routine_id = self.inspection.routine.routine_id
            probe_point_id = self.inspection.probe_point_id
            if object_id:
                parts.append(f"object_id={object_id!r}")
            if routine_id:
                parts.append(f"routine_id={routine_id!r}")
            if probe_point_id:
                parts.append(f"probe_point_id={probe_point_id!r}")
            if self.inspection.replace_existing:
                parts.append("replace_existing=True")

        return "<GenericCommand " + " ".join(parts) + ">"
