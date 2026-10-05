"""Movement orientation modes and observed tag frame prefixes."""

from enum import Enum


class OrientationModes(str, Enum):
    CUSTOM_ORIENTATION = "custom"
    STRAIGHT = "look_straight"
    TAG_ORIENTATION = "relative_to_tag"
    LOOK_LEFT = "left"
    LOOK_RIGHT = "right"
    LOOK_UP = "up"
    LOOK_DOWN = "down"


class TagFrames(str, Enum):
    SPOT_FRAME = "fiducial_"
    APRILTAG_ROS_FRAME = "tag36h11:"
    SPOT_FRAME_FILTERED = "filtered_fiducial_"
