from enum import Enum


class CommandID(str, Enum):
    STOW_ARM = "stow_arm"
    READY_ARM = "ready_arm"
    MOVE_ARM_TO_TAG = "move_to_tag"
    MOVE_ARM_TO_TAG_AND_WAIT = "move_tag_and_wait"
    MOVE_ARM_RELATIVE = "move_arm_relative"
    MOVE_CLOSE_TO_SURFACE = "move_close_to_surface"
    STAND_UP = "stand_up"
    WAIT_TIME = "wait_time"
    EMERGENCY_CANCEL = "cancel_all"
    SCAN_ALL_IN_RANGE = "scan_all_in_range"
    TOGGLE_GRIPPER = "toggle_gripper"
    CLOSE_GRIPPER = "close_gripper"
    ESTOP_STATE = "estop_state"
    STOP_BASE = "stop_base"
    START_SLAM = "start_slam"
    START_LOCALIZATION = "start_localization"
    STOP_MAPPING = "stop_mapping"
    SWAP_MAP = "swap_map"
    MOVE_TO_WAYPOINT = "move_to_waypoint"
    MOVE_BASE_TO_TAG = "move_base_to_tag"
    MOVE_BASE_RELATIVE = "move_base_relative"
    EXECUTE_PROBE_POINT = "execute_probe_point"


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
