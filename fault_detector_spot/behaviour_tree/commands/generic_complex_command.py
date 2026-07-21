# fault_detector_spot/behaviour_tree/generic_command.py
from builtin_interfaces.msg import Time
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
            object_id: str = None,
            routine_id: str = None,
            display_name: str = None,
            reference_tag_id: int = None,
            reference_tag_family: str = None,
            sensor_id: str = None,
            probe_frame: str = None,
            replace_existing: bool = False,
    ):
        super().__init__(command_id, stamp)
        self.duration = duration
        self.goal_pose = goal_pose
        self.tag_id = tag_id
        self.offset = offset
        self.orientation_mode = orientation_mode
        self.map_name = map_name
        self.waypoint_name = waypoint_name
        self.object_id = object_id
        self.routine_id = routine_id
        self.display_name = display_name
        self.reference_tag_id = reference_tag_id
        self.reference_tag_family = reference_tag_family
        self.sensor_id = sensor_id
        self.probe_frame = probe_frame
        self.replace_existing = replace_existing

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
        if self.object_id:
            parts.append(f"object_id={self.object_id!r}")
        if self.routine_id:
            parts.append(f"routine_id={self.routine_id!r}")
        if self.display_name:
            parts.append(f"display_name={self.display_name!r}")
        if self.reference_tag_id is not None:
            parts.append(f"reference_tag_id={self.reference_tag_id}")
        if self.reference_tag_family:
            parts.append(
                f"reference_tag_family={self.reference_tag_family!r}"
            )
        if self.sensor_id:
            parts.append(f"sensor_id={self.sensor_id!r}")
        if self.probe_frame:
            parts.append(f"probe_frame={self.probe_frame!r}")
        if self.replace_existing:
            parts.append("replace_existing=True")

        return "<GenericCommand " + " ".join(parts) + ">"
