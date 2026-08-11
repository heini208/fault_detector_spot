"""Runtime state for a live inspection object pose."""

from dataclasses import dataclass
from enum import Enum
from typing import Optional

from builtin_interfaces.msg import Time as TimeMessage
from geometry_msgs.msg import PoseStamped


class ObjectPoseState(str, Enum):
    """Availability state of an inspection object pose."""

    LIVE = "live"
    UNAVAILABLE = "unavailable"
    INVALID = "invalid"


@dataclass
class ResolvedObjectPose:
    """Fresh map-independent object pose for probe execution."""

    object_id: str
    tag_id: Optional[int]
    state: ObjectPoseState
    selected_pose: Optional[PoseStamped] = None
    message: str = ""
    frame_id: str = ""
    observation_timestamp: Optional[TimeMessage] = None
    observation_age_sec: Optional[float] = None
    observation_source: str = ""

    @property
    def is_available(self) -> bool:
        """Return whether an object pose is available."""
        return self.selected_pose is not None

    @property
    def is_live(self) -> bool:
        """Return whether the selected pose is live."""
        return self.state == ObjectPoseState.LIVE

    @property
    def is_probe_usable(self) -> bool:
        """Return whether the pose can reference probe motion."""
        return (
            self.state == ObjectPoseState.LIVE
            and self.selected_pose is not None
            and self.observation_source == "base"
        )
