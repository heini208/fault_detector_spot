"""Supported cameras for synchronized inspection reference views."""

from dataclasses import dataclass
from typing import Dict, Tuple


@dataclass(frozen=True)
class ReferenceCameraConfig:
    """ROS topics and display metadata for one selectable camera."""

    camera_id: str
    display_name: str
    rgb_topic: str
    depth_topic: str
    camera_info_topic: str


REFERENCE_CAMERAS: Tuple[ReferenceCameraConfig, ...] = (
    ReferenceCameraConfig(
        camera_id="frontleft",
        display_name="Front Left",
        rgb_topic="/camera/frontleft/image",
        depth_topic="/depth_registered/frontleft/image",
        camera_info_topic="/depth_registered/frontleft/camera_info",
    ),
    ReferenceCameraConfig(
        camera_id="frontright",
        display_name="Front Right",
        rgb_topic="/camera/frontright/image",
        depth_topic="/depth_registered/frontright/image",
        camera_info_topic="/depth_registered/frontright/camera_info",
    ),
    ReferenceCameraConfig(
        camera_id="left",
        display_name="Left",
        rgb_topic="/camera/left/image",
        depth_topic="/depth_registered/left/image",
        camera_info_topic="/depth_registered/left/camera_info",
    ),
    ReferenceCameraConfig(
        camera_id="right",
        display_name="Right",
        rgb_topic="/camera/right/image",
        depth_topic="/depth_registered/right/image",
        camera_info_topic="/depth_registered/right/camera_info",
    ),
    ReferenceCameraConfig(
        camera_id="back",
        display_name="Back",
        rgb_topic="/camera/back/image",
        depth_topic="/depth_registered/back/image",
        camera_info_topic="/depth_registered/back/camera_info",
    ),
    ReferenceCameraConfig(
        camera_id="hand",
        display_name="Hand",
        rgb_topic="/camera/hand/image",
        depth_topic="/depth_registered/hand/image",
        camera_info_topic="/depth_registered/hand/camera_info",
    ),
)

REFERENCE_CAMERA_BY_ID: Dict[str, ReferenceCameraConfig] = {
    camera.camera_id: camera for camera in REFERENCE_CAMERAS
}


def get_reference_camera(camera_id: str) -> ReferenceCameraConfig:
    """Return one supported camera or raise a clear validation error."""
    try:
        return REFERENCE_CAMERA_BY_ID[camera_id]
    except KeyError as exception:
        supported = ", ".join(REFERENCE_CAMERA_BY_ID)
        raise ValueError(
            f"Unsupported reference camera '{camera_id}'. "
            f"Supported cameras: {supported}"
        ) from exception


def validate_reference_camera_slots(camera_ids) -> Tuple[Tuple[int, str], ...]:
    """Validate three command slots and return selected slot-camera pairs."""
    if len(camera_ids) != 3:
        raise ValueError("Exactly three reference camera slots are required")

    selected = []
    seen = set()
    for slot_index, raw_camera_id in enumerate(camera_ids):
        camera_id = str(raw_camera_id).strip()
        if not camera_id:
            continue
        get_reference_camera(camera_id)
        if camera_id in seen:
            raise ValueError(
                f"Reference camera '{camera_id}' is selected more than once"
            )
        seen.add(camera_id)
        selected.append((slot_index, camera_id))

    if not selected:
        raise ValueError("Select at least one reference camera")
    return tuple(selected)
