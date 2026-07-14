"""Convert live tag messages into resolver input poses."""

from copy import deepcopy
from typing import Dict, Mapping, Optional

from geometry_msgs.msg import PoseStamped


def tag_elements_to_pose_stamped(
    tag_elements: Optional[Mapping],
) -> Dict[int, PoseStamped]:
    """Convert TagElement entries into map-frame poses."""
    if not tag_elements:
        return {}

    poses = {}

    for tag_id, tag_element in tag_elements.items():
        if not hasattr(tag_element, "pose"):
            raise TypeError(
                f"Tag {tag_id} does not contain a pose"
            )

        pose = tag_element.pose

        if not isinstance(pose, PoseStamped):
            raise TypeError(
                f"Tag {tag_id} pose must be PoseStamped"
            )

        poses[int(tag_id)] = deepcopy(pose)

    return poses